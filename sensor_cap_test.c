/*
 * sensor_cap_test.c – User-space test application for am335x_sensor_cap driver
 *
 * Demonstrates: open, read, poll, ioctl, and mmap paths.
 *
 * Build on BBB:
 *   gcc -o sensor_cap_test sensor_cap_test.c -lpthread
 *
 * Usage:
 *   ./sensor_cap_test /dev/sensor_cap0          # poll + read loop
 *   ./sensor_cap_test /dev/sensor_cap0 --mmap   # zero-copy mmap demo
 *   ./sensor_cap_test /dev/sensor_cap0 --stats  # print stats and exit
 */

Sensor_cap_test.c
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdint.h>
#include <pthread.h>
#include <signal.h>
#include "sensor_cap_uapi.h"

#define PAGE_SIZE 4096

static volatile int g_running = 1;

static void sigint_handler(int s) {
    (void)s;
    g_running = 0;
}

/* ---- Stats demo --------------------------------------------------- */
static void demo_stats(int fd)
{
    struct sc_stats s;
    if (ioctl(fd, IOCTL_GET_STATS, &s) < 0) {
        perror("IOCTL_GET_STATS"); return;
    }
    printf("=== Channel Stats ===\n");
    printf("  IRQ count      : %u\n", s.irq_count);
    printf("  Overflow count : %u\n", s.overflow_count);
    printf("  FIFO used      : %u events\n", s.fifo_used);
    printf("  Last timestamp : %llu ns\n", (unsigned long long)s.last_timestamp_ns);
}

/* ---- Poll + read loop --------------------------------------------- */
static void demo_poll_read(int fd)
{
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    struct capture_event ev;
    int n;

    printf("[poll+read mode] Waiting for GPIO events. Ctrl-C to stop.\n");

    while (g_running) {
        n = poll(&pfd, 1, 1000);   /* 1s timeout */
        if (n < 0) { perror("poll"); break; }
        if (n == 0) { printf("  (timeout – no event)\n"); continue; }

        if (pfd.revents & POLLIN) {
            ssize_t r = read(fd, &ev, sizeof(ev));
            if (r != sizeof(ev)) { perror("read"); break; }
            printf("  ch=%u  gpio_state=0x%08x  ts=%llu ns\n",
                   ev.channel, ev.gpio_state,
                   (unsigned long long)ev.timestamp_ns);
        }
    }
}

/* ---- mmap zero-copy demo ------------------------------------------ */
static void demo_mmap(int fd)
{
    void *vaddr = mmap(NULL, PAGE_SIZE, PROT_READ, MAP_SHARED, fd, 0);
    if (vaddr == MAP_FAILED) { perror("mmap"); return; }

    printf("[mmap mode] DMA buffer mapped at %p. Ctrl-C to stop.\n", vaddr);

    struct capture_event *ev = (struct capture_event *)vaddr;
    uint64_t last_ts = 0;

    while (g_running) {
        /* Spin-poll the DMA buffer for a new timestamp                 *
         * In production you'd use a semaphore or futex signaled by     *
         * the driver via a shared flag in the DMA buffer itself.       */
        if (ev->timestamp_ns != last_ts) {
            last_ts = ev->timestamp_ns;
            printf("  [mmap] ch=%u  gpio=0x%08x  ts=%llu ns\n",
                   ev->channel, ev->gpio_state,
                   (unsigned long long)ev->timestamp_ns);
        }
        usleep(100);
    }

    munmap(vaddr, PAGE_SIZE);
}

/* ---- Thread: reset stats every 10 s ------------------------------- */
static void *reset_thread(void *arg)
{
    int fd = *(int *)arg;
    while (g_running) {
        sleep(10);
        if (ioctl(fd, IOCTL_RESET_CH) < 0)
            perror("IOCTL_RESET_CH");
        else
            printf("  [reset_thread] channel reset\n");
    }
    return NULL;
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <device> [--mmap|--stats]\n", argv[0]);
        return 1;
    }

    signal(SIGINT, sigint_handler);

    int fd = open(argv[1], O_RDWR);
    if (fd < 0) { perror("open"); return 1; }

    /* Set threshold via ioctl */
    uint32_t thr = 50;
    if (ioctl(fd, IOCTL_SET_THRESHOLD, &thr) < 0)
        perror("IOCTL_SET_THRESHOLD (non-fatal)");

    if (argc >= 3 && strcmp(argv[2], "--stats") == 0) {
        demo_stats(fd);
    } else if (argc >= 3 && strcmp(argv[2], "--mmap") == 0) {
        demo_mmap(fd);
    } else {
        pthread_t tid;
        pthread_create(&tid, NULL, reset_thread, &fd);
        demo_poll_read(fd);
        pthread_join(tid, NULL);
    }

    demo_stats(fd);
    close(fd);
    return 0;
}
