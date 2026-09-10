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
} ar8030_reassembly_t;

/* buf/cap must be set by the caller before first use; everything else is
 * zeroed by this call. */
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
