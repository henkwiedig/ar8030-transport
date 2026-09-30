#ifndef AR8030_TRANSPORT_CLOCK_SYNC_H
#define AR8030_TRANSPORT_CLOCK_SYNC_H

/*
 * Maps the air unit's CLOCK_MONOTONIC onto the ground's, NTP style, over
 * the video socket's control messages (common/ar8030_chunk.h "SYNC" /
 * "SYNCR"):
 *
 *   ground t1 --SYNC--> air t2 ... air t3 --SYNCR--> ground t4
 *   offset = ((t2 - t1) + (t3 - t4)) / 2   (air - ground)
 *   rtt    = (t4 - t1) - (t3 - t2)
 *
 * The one assumption is that both directions take equally long; any
 * asymmetry is at most rtt/2, so the sample with the smallest rtt in a
 * short window is the one trusted, and rtt/2 of it is reported as the
 * uncertainty. The window is short (CLOCK_SYNC_WINDOW probes at
 * CLOCK_SYNC_PERIOD_MS) so two crystals drifting apart at a few tens of
 * ppm cannot build up more than a fraction of a millisecond in it.
 *
 * The relay thread (rx/idr_relay.c) sends the probes; rx's main loop feeds
 * the replies in. Both go through the mutex.
 */

#include <pthread.h>
#include <stdint.h>

#define CLOCK_SYNC_WINDOW 16
#define CLOCK_SYNC_PERIOD_MS 500      /* steady state */
#define CLOCK_SYNC_FAST_PERIOD_MS 100 /* until the first window is full */

typedef struct {
    pthread_mutex_t mu;
    struct {
        int64_t offset_us;
        uint64_t rtt_us;
    } samples[CLOCK_SYNC_WINDOW];
    unsigned n;    /* valid samples, up to CLOCK_SYNC_WINDOW */
    unsigned next; /* ring write index */
    uint32_t seq;
    uint64_t last_sent_us;
    uint64_t replies;
} clock_sync_t;

uint64_t clock_sync_now_us(void); /* ground CLOCK_MONOTONIC, us */

void clock_sync_init(clock_sync_t *cs);

/* Returns 1 and the seq/t1 to put into the next SYNC when one is due. */
int clock_sync_due(clock_sync_t *cs, uint64_t now_us, uint32_t *seq, uint64_t *t1_us);

/* A SYNCR arrived at t4_us (ground clock). */
void clock_sync_on_reply(clock_sync_t *cs, uint64_t t1_us, uint64_t t2_us, uint64_t t3_us, uint64_t t4_us);

/* Returns 1 with the current best estimate (air - ground, us) and its
 * +- bound once at least one reply has come back, 0 before that. */
int clock_sync_get(clock_sync_t *cs, int64_t *offset_us, uint32_t *uncertainty_us);

#endif
