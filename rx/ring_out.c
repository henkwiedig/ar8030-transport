#include "ring_out.h"

#include "ar8030_chunk.h"

#include <stdio.h>
#include <string.h>

/* The frame flags that mean the same in a chunk header and in an entry. */
#define FRAME_FLAGS_KEPT (AR8030_CHUNK_FLAG_IDR | AR8030_CHUNK_FLAG_GDR | AR8030_CHUNK_FLAG_ENHANCE)

_Static_assert(AR8030_CHUNK_FLAG_IDR == VENC_FRAME_FLAG_IDR, "IDR bit differs");
_Static_assert(AR8030_CHUNK_FLAG_GDR == VENC_FRAME_FLAG_GDR, "GDR bit differs");
_Static_assert(AR8030_CHUNK_FLAG_ENHANCE == VENC_FRAME_FLAG_ENHANCE, "ENHANCE bit differs");

int rx_ring_out_open(rx_ring_out_t *o, const char *name, uint32_t slot_count, uint32_t slot_data_size)
{
    memset(o, 0, sizeof(*o));
    o->ring = venc_frame_ring_create(name, slot_count, slot_data_size);
    if (!o->ring) {
        fprintf(stderr, "rx: cannot create frame-shm ring '%s' (%u x %u bytes)\n", name, slot_count,
                slot_data_size);
        return -1;
    }
    return 0;
}

void rx_ring_out_close(rx_ring_out_t *o)
{
    if (o->ring)
        venc_frame_ring_destroy(o->ring);
    o->ring = NULL;
}

/* The frame's state after its final entry, published or not. */
static void end_frame(rx_ring_out_t *o, uint16_t frame_seq)
{
    o->frame_open = 0;
    o->last_ended_seq = frame_seq;
    o->have_last_ended = 1;
}

int rx_ring_out_publish(rx_ring_out_t *o, uint16_t frame_seq, uint32_t pts, uint8_t frame_flags,
                        int partial, const uint8_t *a, uint32_t alen, const uint8_t *b, uint32_t blen)
{
    if (!o->frame_open || o->open_seq != frame_seq) {
        /* The first entry of a frame: did anything go missing in front of it? */
        if (o->frame_open)
            o->discont_pending = 1; /* the previous picture never ended */
        if (o->have_last_ended && frame_seq != (uint16_t)(o->last_ended_seq + 1u))
            o->discont_pending = 1; /* whole frames missing (or a tx restart) */
        o->frame_open = 1;
        o->open_seq = frame_seq;
    }

    VencFrameMeta meta = {
        .pts = pts,
        .codec = VENC_FRAME_CODEC_H265,
        .flags = (uint8_t)((frame_flags & FRAME_FLAGS_KEPT) | (partial ? VENC_FRAME_FLAG_PARTIAL : 0) |
                           (o->discont_pending ? VENC_FRAME_FLAG_DISCONT : 0)),
        .gdr_pos = 0,
        .gdr_len = 0,
    };

    /* begin_write fails on a full ring, append on an entry bigger than a slot
     * (the ring counts both in its header for the consumer). */
    int begun = venc_frame_ring_begin_write(o->ring, &meta) == 0;
    int fits = begun && (!alen || venc_frame_ring_append(o->ring, a, alen) == 0) &&
               (!blen || venc_frame_ring_append(o->ring, b, blen) == 0);
    if (begun && !fits)
        venc_frame_ring_abort_write(o->ring);

    if (!fits) {
        o->dropped++;
        o->discont_pending = 1;
        if (!partial)
            end_frame(o, frame_seq);
        return -1;
    }

    venc_frame_ring_commit_write(o->ring);
    o->entries++;
    if (partial)
        o->partial_entries++;
    if (o->discont_pending)
        o->discont_entries++;
    o->discont_pending = 0;
    if (alen + blen > o->max_entry)
        o->max_entry = alen + blen;
    if (!partial)
        end_frame(o, frame_seq);
    return 0;
}
