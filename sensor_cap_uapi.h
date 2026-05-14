/* sensor_cap_uapi.h — shared between kernel driver and user-space test app */
#ifndef _SENSOR_CAP_UAPI_H
#define _SENSOR_CAP_UAPI_H

#include <linux/types.h>
#include <linux/ioctl.h>

/**
 * struct sc_stats - per-channel statistics returned by IOCTL_GET_STATS
 * @irq_count:        total hardware interrupts handled since open/reset
 * @overflow_count:   number of events dropped due to FIFO full
 * @fifo_used:        current number of unread events in the kernel FIFO
 * @last_timestamp_ns: ktime_get_ns() value at last IRQ entry
 */
struct sc_stats {
    __u32 irq_count;
    __u32 overflow_count;
    __u32 fifo_used;
    __u64 last_timestamp_ns;
};

/**
 * struct capture_event - one timestamped event returned by read()
 * @timestamp_ns: absolute monotonic time of the GPIO edge (nanoseconds)
 * @gpio_state:   snapshot of the full GPIO bank DATAIN register
 * @channel:      which channel (0-3) fired
 */
struct capture_event {
    __u64 timestamp_ns;
    __u32 gpio_state;
    __u32 channel;
};

#define SENSOR_CAP_MAGIC    'S'
#define IOCTL_RESET_CH      _IO(SENSOR_CAP_MAGIC,  0)
#define IOCTL_GET_STATS     _IOR(SENSOR_CAP_MAGIC, 1, struct sc_stats)
#define IOCTL_SET_THRESHOLD _IOW(SENSOR_CAP_MAGIC, 2, __u32)
#define IOCTL_ENABLE_CH     _IO(SENSOR_CAP_MAGIC,  3)
#define IOCTL_DISABLE_CH    _IO(SENSOR_CAP_MAGIC,  4)

#endif /* _SENSOR_CAP_UAPI_H */
