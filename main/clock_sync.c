#include "clock_sync.h"
#include <limits.h>
#include <stdlib.h>
#include <string.h>
void clock_sync_reset(clock_sync_filter_t *f) { memset(f, 0, sizeof(*f)); }
clock_sync_status_t clock_sync_status(const clock_sync_filter_t *f, int64_t now_us)
{
    clock_sync_status_t out = {.rtt_us = UINT32_MAX, .age_us = UINT32_MAX};
    if (!f->count) return out;
    unsigned n = f->count < CLOCK_SYNC_WINDOW ? f->count : CLOCK_SYNC_WINDOW;
    for (unsigned i=0; i<n; ++i) {
        if (f->rtts[i] < out.rtt_us) {
            out.rtt_us = f->rtts[i];
            out.offset_us = f->offsets[i];
        }
    }
    int64_t age = now_us - f->last_us;
    out.age_us = age >= 0 && age < UINT32_MAX ? (uint32_t)age : UINT32_MAX;
    out.samples = f->count;
    out.locked = f->count >= 8 && out.rtt_us <= 5000 && out.age_us <= 2000000;
    return out;
}
bool clock_sync_observe(clock_sync_filter_t *f, int64_t sent_us,
                        int64_t received_us, int64_t remote_us)
{
    int64_t rtt = received_us - sent_us;
    if (sent_us <= 0 || remote_us <= 0 || rtt < 0 || rtt > 20000) return false;
    int64_t offset = remote_us - (sent_us + rtt/2);
    clock_sync_status_t old = clock_sync_status(f, received_us);
    /* Reacquire after a clock reset/jump, silence, or changed boot epoch. */
    if (f->count && (old.age_us > 2000000 || llabs(offset-old.offset_us)>100000))
        clock_sync_reset(f);
    f->offsets[f->next] = offset;
    f->rtts[f->next] = (uint32_t)rtt;
    f->next = (f->next+1) % CLOCK_SYNC_WINDOW;
    if (f->count < UINT16_MAX) ++f->count;
    f->last_us = received_us;
    return true;
}
