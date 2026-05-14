# Multi-channel GPIO-IRQ Sensor Capture Device Driver

Character Driver
 *
 * Target : TI AM335x (BeagleBone Black)
 * Author : Pamith H A
 *
 * Architecture
 * ============
 *  - Platform driver, probed via Device Tree (compatible = "ti,am335x-sensor-cap")
 *  - 4 independent capture channels, each backed by a GPIO IRQ
 *  - Per-channel KFIFO ring-buffer (4096 bytes) in kernel virtual address space
 *  - mmap() path: contiguous DMA-coherent buffer shared zero-copy with user-space
 *  - Interrupt bottom-half via dedicated per-channel tasklets
 *  - Fine-grained locking:
 *      spinlock  – IRQ context / FIFO producer (ISR/tasklet)
 *      mutex     – ioctl / open / release (process context)
 *      atomic_t  – overflow counter visible from sysfs
 *  - IOCTLs: RESET_CHANNEL, GET_STATS, SET_THRESHOLD, ENABLE/DISABLE_CHANNEL
 *  - /sys/class/sensor_cap/sensor_capX/… attribute nodes
 *  - Debugfs subtree: /sys/kernel/debug/sensor_cap/<channel>/
 *  - poll()/select() support via wait_queue_head_t
 *  - Graceful oops/panic handler registered via die_notifier
 *
 * Physical Memory Map (AM335x)
 * ============================
 *  0x4804_C000 – 0x4804_CFFF  GPIO1 registers          (4 KB)
 *  0x481A_C000 – 0x481A_CFFF  GPIO3 registers          (4 KB)
 *  0x44E0_7000 – 0x44E0_7FFF  CM_PER (Clock Module)    (4 KB)  [used for gate]
 *  DMA-coherent buffer (per channel, 4 KB each) allocated via
 *    dma_alloc_coherent() → appears in ZONE_DMA, bus address given to user via mmap
 *
 * Kernel Virtual Memory layout of one channel
 * ============================================
 *   kfifo  (vmalloc arena)      : 4096 bytes  ring-buffer for timestamped events
 *   dma_buf (lowmem, contiguous): 4096 bytes  zero-copy mmap target
 *   gpio_base (ioremap)         : 4096 bytes  GPIO register window
 *