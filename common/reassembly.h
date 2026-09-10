#ifndef AR8030_TRANSPORT_REASSEMBLY_H
#define AR8030_TRANSPORT_REASSEMBLY_H

/*
 * Reassembles ar8030_chunk_hdr-prefixed chunks (common/ar8030_chunk.h)
 * back into whole frames. Shared by rx/main.c (fed from bb_socket_read())
 * and test/roundtrip_test.c, so the exact reassembly logic under test is
 * the same logic that ships.
 *
 * Assumes the transport preserves chunk order (bb_socket is a queued
 * point-to-point channel, not a packet-switched network -- see
 * rx/main.c's file header comment). A chunk that doesn't extend the
 * frame currently being assembled drops whatever was collected so far;
 * no waiting, no NACK, freshness over completeness.
 */

#include "ar8030_chunk.h"

#include <stdint.h>

typedef struct {
    uint8_t *buf;
    uint32_t cap;
    uint16_t frame_seq;
    uint16_t chunk_count;
    uint16_t next_chunk_idx;
    uint32_t write_off;
    uint32_t frame_pts;
    uint8_t flags;
    int active;

    /* Lifetime counters for -v stats reporting (rx/main.c) -- persist
     * across ar8030_reassembly_reset() calls, unlike everything above.
     * completed_frames is every feed() that returned 1; dropped_frames is
     * every feed() that discarded an in-progress (or about-to-start)
     * frame due to a gap, an out-of-order/inconsistent chunk, or a
     * buffer overflow -- see the three call sites in reassembly.c. A
     * dropped_frames rate much higher than expected packet loss for the
     * link is a sign something upstream (chunk_stream resyncing, wrong
     * chunk size on one end, etc.) is corrupting the chunk stream rather
     * than just losing occasional chunks to the radio. */
    uint64_t completed_frames;
    uint64_t dropped_frames;
    uint16_t last_bad_seq;     /* dedup for dropped_frames -- see reassembly.c's note_dropped() */
    int have_last_bad_seq;
} ar8030_reassembly_t;

/* buf/cap must be set by the caller before first use; everything else
 * (including the lifetime counters) is zeroed by this call. */
void ar8030_reassembly_init(ar8030_reassembly_t *r, uint8_t *buf, uint32_t cap);

/* Drops any in-progress frame without completing it. */
void ar8030_reassembly_reset(ar8030_reassembly_t *r);

/* Feeds one received chunk (already parsed header + its payload bytes).
 * Returns 1 when this chunk completed a frame -- r->buf[0..write_off) is
 * the reassembled Annex-B data, r->frame_pts/flags are that frame's
 * values; the caller should consume them before the next feed() call,
 * which may reuse r->buf. Returns 0 otherwise (frame still in progress,
 * or this chunk was discarded as out of order/oversized). */
int ar8030_reassembly_feed(ar8030_reassembly_t *r, const struct ar8030_chunk_hdr *hdr,
                            const uint8_t *payload, uint32_t payload_len);

#endif /* AR8030_TRANSPORT_REASSEMBLY_H */
