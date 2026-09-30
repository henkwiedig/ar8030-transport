#include "clock_sync.h"

#include <string.h>
#include <time.h>

uint64_t clock_sync_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

void clock_sync_init(clock_sync_t *cs)
{
    memset(cs, 0, sizeof(*cs));
    pthread_mutex_init(&cs->mu, NULL);
}

int clock_sync_due(clock_sync_t *cs, uint64_t now_us, uint32_t *seq, uint64_t *t1_us)
{
    int due = 0;
    pthread_mutex_lock(&cs->mu);
    uint64_t period_us =
        1000ull * (cs->n < CLOCK_SYNC_WINDOW ? CLOCK_SYNC_FAST_PERIOD_MS : CLOCK_SYNC_PERIOD_MS);
    if (!cs->last_sent_us || now_us - cs->last_sent_us >= period_us) {
        cs->last_sent_us = now_us;
        *seq = cs->seq++;
        *t1_us = now_us;
        due = 1;
    }
    pthread_mutex_unlock(&cs->mu);
    return due;
}

void clock_sync_on_reply(clock_sync_t *cs, uint64_t t1_us, uint64_t t2_us, uint64_t t3_us, uint64_t t4_us)
{
    /* A reply to a probe from before a restart, or with a turnaround
     * longer than the whole round trip, is garbage. */
    if (t4_us < t1_us || t3_us < t2_us || (t4_us - t1_us) < (t3_us - t2_us))
        return;
    uint64_t rtt = (t4_us - t1_us) - (t3_us - t2_us);
    int64_t offset = (((int64_t)t2_us - (int64_t)t1_us) + ((int64_t)t3_us - (int64_t)t4_us)) / 2;

    pthread_mutex_lock(&cs->mu);
    /* An offset that no round trip in the window can explain means the air
     * clock itself jumped (the air unit rebooted): the old samples describe
     * a clock that no longer exists, so start over rather than let their
     * lower rtt keep winning for a whole window. */
    for (unsigned i = 0; i < cs->n; i++) {
        int64_t d = offset - cs->samples[i].offset_us;
        if (d < 0)
            d = -d;
        if ((uint64_t)d > rtt + cs->samples[i].rtt_us + 5000) {
            cs->n = 0;
            cs->next = 0;
            break;
        }
    }
    cs->samples[cs->next].offset_us = offset;
    cs->samples[cs->next].rtt_us = rtt;
    cs->next = (cs->next + 1) % CLOCK_SYNC_WINDOW;
    if (cs->n < CLOCK_SYNC_WINDOW)
        cs->n++;
    cs->replies++;
    pthread_mutex_unlock(&cs->mu);
}

int clock_sync_get(clock_sync_t *cs, int64_t *offset_us, uint32_t *uncertainty_us)
{
    int ok = 0;
    pthread_mutex_lock(&cs->mu);
    if (cs->n) {
        unsigned best = 0;
        for (unsigned i = 1; i < cs->n; i++)
            if (cs->samples[i].rtt_us < cs->samples[best].rtt_us)
                best = i;
        *offset_us = cs->samples[best].offset_us;
        uint64_t half = cs->samples[best].rtt_us / 2;
        *uncertainty_us = half > UINT32_MAX ? UINT32_MAX : (uint32_t)half;
        ok = 1;
    }
    pthread_mutex_unlock(&cs->mu);
    return ok;
}
