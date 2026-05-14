#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/tasklet.h>
#include <linux/notifier.h>
#include <linux/kdebug.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/sysfs.h>
#include <uapi/linux/ioctl.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                           */
/* ------------------------------------------------------------------ */
#define DRIVER_NAME        "am335x_sensor_cap"
#define NUM_CHANNELS       4
#define FIFO_SIZE          4096          /* bytes, must be power-of-2  */
#define DMA_BUF_SIZE       4096          /* bytes, one page            */
#define MAX_MINOR          NUM_CHANNELS

/* AM335x GPIO1 base – documented in AM335x TRM Table 2-1             */
#define AM335X_GPIO1_BASE  0x4804C000UL
#define AM335X_GPIO3_BASE  0x481AC000UL
#define GPIO_REGION_SIZE   0x1000UL

/* GPIO register offsets (AM335x TRM sec 25.4)                        */
#define GPIO_IRQSTATUS_0   0x002C
#define GPIO_IRQENABLE_0   0x003C
#define GPIO_DATAIN        0x0138
#define GPIO_OE            0x0134

/* ------------------------------------------------------------------ */
/*  IOCTL definitions (shared with test app via sensor_cap_uapi.h)     */
/* ------------------------------------------------------------------ */
#define SENSOR_CAP_MAGIC    'S'
#define IOCTL_RESET_CH      _IO(SENSOR_CAP_MAGIC,  0)
#define IOCTL_GET_STATS     _IOR(SENSOR_CAP_MAGIC, 1, struct sc_stats)
#define IOCTL_SET_THRESHOLD _IOW(SENSOR_CAP_MAGIC, 2, __u32)
#define IOCTL_ENABLE_CH     _IO(SENSOR_CAP_MAGIC,  3)
#define IOCTL_DISABLE_CH    _IO(SENSOR_CAP_MAGIC,  4)

struct sc_stats {
    __u32 irq_count;
    __u32 overflow_count;
    __u32 fifo_used;
    __u64 last_timestamp_ns;
};

/* ------------------------------------------------------------------ */
/*  Per-event record written into the KFIFO                            */
/* ------------------------------------------------------------------ */
struct capture_event {
    u64  timestamp_ns;   /* ktime_get_ns() at IRQ entry               */
    u32  gpio_state;     /* snapshot of GPIO_DATAIN                   */
    u32  channel;
};

/* ------------------------------------------------------------------ */
/*  Per-channel private data                                            */
/* ------------------------------------------------------------------ */
struct channel_dev {
    /* cdev / sysfs */
    struct cdev         cdev;
    struct device      *sysfs_dev;
    int                 minor;

    /* GPIO / IRQ */
    struct gpio_desc   *gpio_desc;
    int                 irq;
    void __iomem       *gpio_base;   /* ioremap of GPIO bank           */

    /* Ring-buffer – kfifo protected by fifo_lock (spinlock)          */
    DECLARE_KFIFO_PTR(fifo, struct capture_event);
    spinlock_t          fifo_lock;

    /* DMA-coherent buffer – mmapped to user-space                    */
    void               *dma_vaddr;
    dma_addr_t          dma_paddr;
    struct device      *dma_dev;     /* parent platform device         */

    /* Mutex – serialises open/release/ioctl (process context)        */
    struct mutex        lock;

    /* Statistics (atomic – safe from any context)                    */
    atomic_t            irq_count;
    atomic_t            overflow_count;
    u64                 last_ts_ns;

    /* Threshold for sysfs/ioctl configurability                      */
    u32                 threshold;
    bool                enabled;

    /* Wait-queue for poll()/select()                                  */
    wait_queue_head_t   wq;

    /* Tasklet – bottom-half: drains shadow-buffer into kfifo         */
    struct tasklet_struct tasklet;

    /* Shadow record filled by hard-IRQ handler                       */
    struct capture_event shadow_event;

    /* Debugfs                                                         */
    struct dentry      *dbg_dir;
};

/* ------------------------------------------------------------------ */
/*  Global driver state                                                 */
/* ------------------------------------------------------------------ */
static struct class  *sc_class;
static dev_t          sc_devt;          /* first device number         */
static struct channel_dev *channels[NUM_CHANNELS];
static struct dentry  *sc_dbg_root;
static struct platform_device *sc_pdev; /* for DMA device ref          */

/* ------------------------------------------------------------------ */
/*  Tasklet – bottom half                                               */
/*                                                                      */
/*  Runs in softirq context (non-reentrant per channel).               */
/*  Copies shadow_event (written by hard-IRQ) into the kfifo, then     */
/*  copies the same event into the DMA buffer (zero-copy mmap target). */
/*  Using a shadow copy avoids holding the spinlock inside the ISR for  */
/*  more than a register-read; the tasklet races only with concurrent  */
/*  user-space reads, which are also spinlock-protected.               */
/* ------------------------------------------------------------------ */
static void sc_tasklet_fn(unsigned long data)
{
    struct channel_dev *ch = (struct channel_dev *)data;
    struct capture_event ev;
    unsigned long flags;
    int ret;

    /* Snapshot the shadow (no need for lock – only tasklet writes it  *
     * after the hard-IRQ, and tasklets are not concurrent per CPU)    */
    ev = ch->shadow_event;

    spin_lock_irqsave(&ch->fifo_lock, flags);
    ret = kfifo_in(&ch->fifo, &ev, 1);
    spin_unlock_irqrestore(&ch->fifo_lock, flags);

    if (ret == 0) {
        atomic_inc(&ch->overflow_count);
        pr_warn_ratelimited(DRIVER_NAME ": ch%d FIFO overflow\n", ch->minor);
    }

    /* Copy to DMA buffer so user mmap sees latest event               */
    memcpy(ch->dma_vaddr, &ev, sizeof(ev));

    /* Unblock any poll()ing user-space threads                        */
    wake_up_interruptible(&ch->wq);
}

/* ------------------------------------------------------------------ */
/*  Hard IRQ handler – minimal work, schedule tasklet                   */
/* ------------------------------------------------------------------ */
static irqreturn_t sc_irq_handler(int irq, void *dev_id)
{
    struct channel_dev *ch = dev_id;

    /* Record time immediately – pre-emption disabled in IRQ context   */
    ch->shadow_event.timestamp_ns = ktime_get_ns();
    ch->shadow_event.channel      = ch->minor;

    /* Read GPIO data register for pin state snapshot                  */
    if (ch->gpio_base)
        ch->shadow_event.gpio_state =
            readl_relaxed(ch->gpio_base + GPIO_DATAIN);
    else
        ch->shadow_event.gpio_state = gpiod_get_value(ch->gpio_desc);

    ch->last_ts_ns = ch->shadow_event.timestamp_ns;
    atomic_inc(&ch->irq_count);

    tasklet_schedule(&ch->tasklet);
    return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/*  File operations                                                     */
/* ------------------------------------------------------------------ */
static int sc_open(struct inode *inode, struct file *filp)
{
    struct channel_dev *ch =
        container_of(inode->i_cdev, struct channel_dev, cdev);

    if (mutex_lock_interruptible(&ch->lock))
        return -ERESTARTSYS;

    filp->private_data = ch;
    mutex_unlock(&ch->lock);
    return 0;
}

static int sc_release(struct inode *inode, struct file *filp)
{
    /* Nothing to free – buffers are persistent for re-open            */
    return 0;
}

/* read() – returns one capture_event struct per call                  */
static ssize_t sc_read(struct file *filp, char __user *buf,
                       size_t count, loff_t *ppos)
{
    struct channel_dev *ch = filp->private_data;
    struct capture_event ev;
    unsigned long flags;
    unsigned int n;
    int ret;

    if (count < sizeof(ev))
        return -EINVAL;

    /* Block if FIFO empty (unless O_NONBLOCK)                         */
    ret = wait_event_interruptible(ch->wq,
            ({ spin_lock_irqsave(&ch->fifo_lock, flags);
               n = kfifo_len(&ch->fifo);
               spin_unlock_irqrestore(&ch->fifo_lock, flags);
               n > 0; })
            || !ch->enabled);
    if (ret)
        return ret;

    spin_lock_irqsave(&ch->fifo_lock, flags);
    n = kfifo_out(&ch->fifo, &ev, 1);
    spin_unlock_irqrestore(&ch->fifo_lock, flags);

    if (!n)
        return -EAGAIN;

    if (copy_to_user(buf, &ev, sizeof(ev)))
        return -EFAULT;

    return sizeof(ev);
}

/* poll() / select() support                                            */
static __poll_t sc_poll(struct file *filp, poll_table *wait)
{
    struct channel_dev *ch = filp->private_data;
    unsigned long flags;
    __poll_t mask = 0;

    poll_wait(filp, &ch->wq, wait);

    spin_lock_irqsave(&ch->fifo_lock, flags);
    if (kfifo_len(&ch->fifo) > 0)
        mask |= EPOLLIN | EPOLLRDNORM;
    spin_unlock_irqrestore(&ch->fifo_lock, flags);

    return mask;
}

/* ioctl()                                                              */
static long sc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct channel_dev *ch = filp->private_data;
    struct sc_stats stats;
    u32 threshold;
    unsigned long flags;

    if (mutex_lock_interruptible(&ch->lock))
        return -ERESTARTSYS;

    switch (cmd) {
    case IOCTL_RESET_CH:
        spin_lock_irqsave(&ch->fifo_lock, flags);
        kfifo_reset(&ch->fifo);
        spin_unlock_irqrestore(&ch->fifo_lock, flags);
        atomic_set(&ch->irq_count, 0);
        atomic_set(&ch->overflow_count, 0);
        break;

    case IOCTL_GET_STATS:
        spin_lock_irqsave(&ch->fifo_lock, flags);
        stats.fifo_used       = kfifo_len(&ch->fifo);
        spin_unlock_irqrestore(&ch->fifo_lock, flags);
        stats.irq_count       = atomic_read(&ch->irq_count);
        stats.overflow_count  = atomic_read(&ch->overflow_count);
        stats.last_timestamp_ns = ch->last_ts_ns;
        if (copy_to_user((void __user *)arg, &stats, sizeof(stats))) {
            mutex_unlock(&ch->lock);
            return -EFAULT;
        }
        break;

    case IOCTL_SET_THRESHOLD:
        if (copy_from_user(&threshold, (void __user *)arg, sizeof(threshold))) {
            mutex_unlock(&ch->lock);
            return -EFAULT;
        }
        ch->threshold = threshold;
        break;

    case IOCTL_ENABLE_CH:
        ch->enabled = true;
        enable_irq(ch->irq);
        break;

    case IOCTL_DISABLE_CH:
        ch->enabled = false;
        disable_irq(ch->irq);
        break;

    default:
        mutex_unlock(&ch->lock);
        return -ENOTTY;
    }

    mutex_unlock(&ch->lock);
    return 0;
}

/*
 * mmap() – maps the per-channel DMA-coherent buffer into user VMA.
 *
 * Key subtlety on ARM: dma_alloc_coherent() returns a kernel virtual
 * address whose underlying pages are already marked non-cacheable in
 * the kernel's page tables.  We must use dma_mmap_coherent() (not
 * remap_pfn_range()) so the MMU attributes are preserved correctly in
 * the user-space PTE – otherwise the user may see a cached alias of a
 * non-cached buffer, causing coherency issues on ARM without CCI.
 */
static int sc_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct channel_dev *ch = filp->private_data;
    size_t size = vma->vm_end - vma->vm_start;

    if (size > DMA_BUF_SIZE)
        return -EINVAL;

    return dma_mmap_coherent(ch->dma_dev, vma,
                             ch->dma_vaddr, ch->dma_paddr, size);
}

static const struct file_operations sc_fops = {
    .owner          = THIS_MODULE,
    .open           = sc_open,
    .release        = sc_release,
    .read           = sc_read,
    .poll           = sc_poll,
    .unlocked_ioctl = sc_ioctl,
    .mmap           = sc_mmap,
};

/* ------------------------------------------------------------------ */
/*  Sysfs attributes                                                    */
/* ------------------------------------------------------------------ */
static ssize_t irq_count_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    struct channel_dev *ch = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%d\n", atomic_read(&ch->irq_count));
}
static DEVICE_ATTR_RO(irq_count);

static ssize_t overflow_count_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    struct channel_dev *ch = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%d\n", atomic_read(&ch->overflow_count));
}
static DEVICE_ATTR_RO(overflow_count);

static ssize_t threshold_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    struct channel_dev *ch = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%u\n", ch->threshold);
}
static ssize_t threshold_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
    struct channel_dev *ch = dev_get_drvdata(dev);
    u32 val;
    if (kstrtou32(buf, 10, &val))
        return -EINVAL;
    ch->threshold = val;
    return count;
}
static DEVICE_ATTR_RW(threshold);

static struct attribute *sc_attrs[] = {
    &dev_attr_irq_count.attr,
    &dev_attr_overflow_count.attr,
    &dev_attr_threshold.attr,
    NULL
};
ATTRIBUTE_GROUPS(sc);

/* ------------------------------------------------------------------ */
/*  Debugfs                                                             */
/* ------------------------------------------------------------------ */
static int sc_dbg_stats_show(struct seq_file *s, void *v)
{
    struct channel_dev *ch = s->private;
    unsigned long flags;
    u32 fifo_used;

    spin_lock_irqsave(&ch->fifo_lock, flags);
    fifo_used = kfifo_len(&ch->fifo);
    spin_unlock_irqrestore(&ch->fifo_lock, flags);

    seq_printf(s, "channel        : %d\n",   ch->minor);
    seq_printf(s, "irq_count      : %d\n",   atomic_read(&ch->irq_count));
    seq_printf(s, "overflow_count : %d\n",   atomic_read(&ch->overflow_count));
    seq_printf(s, "fifo_used      : %u\n",   fifo_used);
    seq_printf(s, "last_ts_ns     : %llu\n", ch->last_ts_ns);
    seq_printf(s, "threshold      : %u\n",   ch->threshold);
    seq_printf(s, "enabled        : %d\n",   ch->enabled);
    seq_printf(s, "dma_paddr      : 0x%llx\n", (u64)ch->dma_paddr);
    return 0;
}
DEFINE_SHOW_ATTRIBUTE(sc_dbg_stats);

static void sc_setup_debugfs(struct channel_dev *ch)
{
    char name[16];
    snprintf(name, sizeof(name), "ch%d", ch->minor);
    ch->dbg_dir = debugfs_create_dir(name, sc_dbg_root);
    debugfs_create_file("stats", 0444, ch->dbg_dir, ch, &sc_dbg_stats_fops);
}

/* ------------------------------------------------------------------ */
/*  Die/panic notifier – register to collect state on kernel oops      */
/* ------------------------------------------------------------------ */
static int sc_die_notify(struct notifier_block *nb,
                         unsigned long val, void *args)
{
    int i;
    pr_err(DRIVER_NAME ": *** OOPS/PANIC – dumping channel state ***\n");
    for (i = 0; i < NUM_CHANNELS; i++) {
        if (!channels[i]) continue;
        pr_err("  ch%d: irqs=%d overflows=%d last_ts=%llu\n",
               i,
               atomic_read(&channels[i]->irq_count),
               atomic_read(&channels[i]->overflow_count),
               channels[i]->last_ts_ns);
    }
    return NOTIFY_OK;
}

static struct notifier_block sc_die_nb = {
    .notifier_call = sc_die_notify,
};

/* ------------------------------------------------------------------ */
/*  Platform driver probe                                               */
/* ------------------------------------------------------------------ */
static int sc_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct channel_dev *ch;
    int i, ret;

    sc_pdev = pdev;

    /* Allocate the major/minor range                                  */
    ret = alloc_chrdev_region(&sc_devt, 0, NUM_CHANNELS, DRIVER_NAME);
    if (ret) {
        dev_err(dev, "alloc_chrdev_region failed: %d\n", ret);
        return ret;
    }

    sc_class = class_create(THIS_MODULE, DRIVER_NAME);
    if (IS_ERR(sc_class)) {
        ret = PTR_ERR(sc_class);
        goto err_unreg_region;
    }
    sc_class->dev_groups = sc_groups;

    /* Debugfs root                                                    */
    sc_dbg_root = debugfs_create_dir(DRIVER_NAME, NULL);

    /* Register die notifier                                           */
    register_die_notifier(&sc_die_nb);

    pm_runtime_enable(dev);

    for (i = 0; i < NUM_CHANNELS; i++) {
        ch = devm_kzalloc(dev, sizeof(*ch), GFP_KERNEL);
        if (!ch) { ret = -ENOMEM; goto err_cleanup; }

        ch->minor   = i;
        ch->enabled = true;
        ch->dma_dev = dev;

        mutex_init(&ch->lock);
        spin_lock_init(&ch->fifo_lock);
        init_waitqueue_head(&ch->wq);
        tasklet_init(&ch->tasklet, sc_tasklet_fn, (unsigned long)ch);
        atomic_set(&ch->irq_count, 0);
        atomic_set(&ch->overflow_count, 0);

        /* Allocate kfifo                                              */
        ret = kfifo_alloc(&ch->fifo, FIFO_SIZE / sizeof(struct capture_event),
                          GFP_KERNEL);
        if (ret) {
            dev_err(dev, "kfifo_alloc ch%d failed\n", i);
            goto err_cleanup;
        }

        /* DMA-coherent buffer                                         */
        ch->dma_vaddr = dma_alloc_coherent(dev, DMA_BUF_SIZE,
                                           &ch->dma_paddr, GFP_KERNEL);
        if (!ch->dma_vaddr) {
            dev_err(dev, "dma_alloc_coherent ch%d failed\n", i);
            ret = -ENOMEM;
            goto err_cleanup;
        }

        /* GPIO descriptor from Device Tree                            */
        ch->gpio_desc = devm_gpiod_get_index(dev, "capture", i, GPIOD_IN);
        if (IS_ERR(ch->gpio_desc)) {
            dev_err(dev, "gpiod_get ch%d failed\n", i);
            ret = PTR_ERR(ch->gpio_desc);
            goto err_cleanup;
        }

        /* Map the GPIO bank register space                            */
        ch->gpio_base = devm_ioremap(dev,
                            (i < 2) ? AM335X_GPIO1_BASE : AM335X_GPIO3_BASE,
                            GPIO_REGION_SIZE);
        if (!ch->gpio_base)
            dev_warn(dev, "ioremap ch%d GPIO bank failed – using gpiod\n", i);

        /* IRQ                                                         */
        ch->irq = gpiod_to_irq(ch->gpio_desc);
        if (ch->irq < 0) {
            dev_err(dev, "gpiod_to_irq ch%d failed\n", i);
            ret = ch->irq;
            goto err_cleanup;
        }

        ret = devm_request_irq(dev, ch->irq, sc_irq_handler,
                               IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                               DRIVER_NAME, ch);
        if (ret) {
            dev_err(dev, "request_irq ch%d failed: %d\n", i, ret);
            goto err_cleanup;
        }

        /* Character device                                            */
        cdev_init(&ch->cdev, &sc_fops);
        ch->cdev.owner = THIS_MODULE;
        ret = cdev_add(&ch->cdev, MKDEV(MAJOR(sc_devt), i), 1);
        if (ret) goto err_cleanup;

        ch->sysfs_dev = device_create(sc_class, dev,
                                      MKDEV(MAJOR(sc_devt), i),
                                      ch, "sensor_cap%d", i);
        if (IS_ERR(ch->sysfs_dev)) {
            ret = PTR_ERR(ch->sysfs_dev);
            goto err_cleanup;
        }

        sc_setup_debugfs(ch);
        channels[i] = ch;

        dev_info(dev, "channel %d ready: /dev/sensor_cap%d  irq=%d  dma=0x%llx\n",
                 i, i, ch->irq, (u64)ch->dma_paddr);
    }

    return 0;

err_cleanup:
    /* devm resources are freed automatically on probe failure          */
    debugfs_remove_recursive(sc_dbg_root);
    unregister_die_notifier(&sc_die_nb);
    if (!IS_ERR_OR_NULL(sc_class))
        class_destroy(sc_class);
err_unreg_region:
    unregister_chrdev_region(sc_devt, NUM_CHANNELS);
    return ret;
}

static int sc_remove(struct platform_device *pdev)
{
    int i;
    for (i = 0; i < NUM_CHANNELS; i++) {
        if (!channels[i]) continue;
        tasklet_kill(&channels[i]->tasklet);
        device_destroy(sc_class, MKDEV(MAJOR(sc_devt), i));
        cdev_del(&channels[i]->cdev);
        dma_free_coherent(&pdev->dev, DMA_BUF_SIZE,
                          channels[i]->dma_vaddr,
                          channels[i]->dma_paddr);
        kfifo_free(&channels[i]->fifo);
        channels[i] = NULL;
    }
    debugfs_remove_recursive(sc_dbg_root);
    unregister_die_notifier(&sc_die_nb);
    class_destroy(sc_class);
    unregister_chrdev_region(sc_devt, NUM_CHANNELS);
    pm_runtime_disable(&pdev->dev);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Device Tree match table                                             */
/* ------------------------------------------------------------------ */
static const struct of_device_id sc_of_match[] = {
    { .compatible = "ti,am335x-sensor-cap" },
    { }
};
MODULE_DEVICE_TABLE(of, sc_of_match);

static struct platform_driver sc_platform_driver = {
    .probe  = sc_probe,
    .remove = sc_remove,
    .driver = {
        .name           = DRIVER_NAME,
        .of_match_table = sc_of_match,
        .owner          = THIS_MODULE,
    },
};

module_platform_driver(sc_platform_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Pamith H A");
MODULE_DESCRIPTION("AM335x Multi-Channel GPIO-IRQ Sensor Capture – Character Driver");
MODULE_VERSION("1.0");