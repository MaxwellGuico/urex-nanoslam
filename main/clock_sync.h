#pragma once
#include <stdbool.h>
#include <stdint.h>
#define CLOCK_SYNC_WINDOW 16
/* PX4 boot time = ESP monotonic time + offset_us. No clock is stepped. */
typedef struct {
    int64_t offset_us;
    uint32_t rtt_us, age_us;
    uint16_t samples;
    bool locked;
} clock_sync_status_t;
typedef struct {
    int64_t offsets[CLOCK_SYNC_WINDOW];
    uint32_t rtts[CLOCK_SYNC_WINDOW];
    int64_t last_us;
    uint16_t count;
    uint8_t next;
} clock_sync_filter_t;
void clock_sync_reset(clock_sync_filter_t *f);
bool clock_sync_observe(clock_sync_filter_t *f, int64_t sent_us,
                        int64_t received_us, int64_t remote_us);
clock_sync_status_t clock_sync_status(const clock_sync_filter_t *f, int64_t now_us);
