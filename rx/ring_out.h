#ifndef AR8030_TRANSPORT_RX_RING_OUT_H
#define AR8030_TRANSPORT_RX_RING_OUT_H

/*
 * rx's frame-shm output (-R): the reassembled video published into a
 * venc_frame_ring for a same-host consumer (PixelPilot) instead of RTP over
 * the loopback. See docs/ground-frame-ring.md.
 *
 * rx is the ring's producer and owner: it creates the shared memory object
 * (replacing a stale one from an earlier run) and a consumer attaches to it.
 *
 * An entry is what rx would otherwise have sent as RTP: a whole access unit,
 * or -- for a frame the air streams slice by slice -- one forwarded part with
 * VENC_FRAME_FLAG_PARTIAL, and the frame's last part without it. The last
 * part may be empty (meta only): it then just ends the access unit.
 *
 * VENC_FRAME_FLAG_DISCONT goes on the first entry after anything went
 * missing in front of it:
 *   - a gap in the air's frame counter (frames lost, or a tx restart),
 *   - a frame whose first parts were published but whose last part never
 *     came (the rest of the picture was lost),
 *   - an entry this producer could not publish itself (ring full, or bigger
 *     than a slot).
 */

#include "venc_frame_ring.h"

#include <stdint.h>

typedef struct {
    venc_frame_ring_t *ring;

    /* Loss tracking, see the header comment. */
    int frame_open;          /* parts of open_seq published, its last one not yet */
    uint16_t open_seq;
    int have_last_ended;
    uint16_t last_ended_seq; /* the last frame whose final entry went out (or was dropped) */
    int discont_pending;

    /* Lifetime counters for rx's -v stats. */
    uint64_t entries;
    uint64_t partial_entries;
    uint64_t discont_entries;
    uint64_t dropped;        /* entries not published: ring full or oversize */
    uint32_t max_entry;      /* largest entry payload so far (meta excluded) */
} rx_ring_out_t;

/* Creates the ring "name" (with or without the leading '/') with slot_count
 * slots (a power of two) of slot_data_size bytes each, the 8-byte entry
 * meta included. Returns 0, or -1 with the reason on stderr. */
int rx_ring_out_open(rx_ring_out_t *o, const char *name, uint32_t slot_count, uint32_t slot_data_size);

/* Unmaps and unlinks the ring. Safe on a ring that never opened. */
void rx_ring_out_close(rx_ring_out_t *o);

/* Publishes one entry of frame frame_seq: a[0..alen) then b[0..blen) (either
 * may be empty, e.g. a timing SEI followed by the rest of a picture).
 * frame_flags are the frame's AR8030_CHUNK_FLAG_* bits, of which IDR, GDR
 * and ENHANCE carry over (same values as VENC_FRAME_FLAG_*). partial = more
 * of this frame follows. pts goes into the entry meta unchanged.
 * Returns 0 if published, -1 if dropped (counted, and the next entry that is
 * published carries DISCONT). */
int rx_ring_out_publish(rx_ring_out_t *o, uint16_t frame_seq, uint32_t pts, uint8_t frame_flags,
                        int partial, const uint8_t *a, uint32_t alen, const uint8_t *b, uint32_t blen);

#endif /* AR8030_TRANSPORT_RX_RING_OUT_H */
