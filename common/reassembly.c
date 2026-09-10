#include "reassembly.h"

#include <stdio.h>
#include <string.h>

void ar8030_reassembly_init(ar8030_reassembly_t *r, uint8_t *buf, uint32_t cap)
{
    memset(r, 0, sizeof(*r));
    r->buf = buf;
    r->cap = cap;
}

void ar8030_reassembly_reset(ar8030_reassembly_t *r)
{
    r->active = 0;
    r->write_off = 0;
    r->next_chunk_idx = 0;
}

int ar8030_reassembly_feed(ar8030_reassembly_t *r, const struct ar8030_chunk_hdr *hdr,
                            const uint8_t *payload, uint32_t payload_len)
{
    int is_new_frame = !r->active || hdr->frame_seq != r->frame_seq;

    if (is_new_frame) {
        if (hdr->chunk_idx != 0) {
            /* Mid-frame chunk for a frame we never started (e.g. we just
             * started up, or dropped chunk 0) -- nothing to do but wait
             * for the next frame_seq to bring a fresh chunk 0. */
            ar8030_reassembly_reset(r);
            return 0;
        }
        r->active = 1;
        r->frame_seq = hdr->frame_seq;
        r->chunk_count = hdr->chunk_count;
        r->frame_pts = hdr->frame_pts;
        r->flags = hdr->flags;
        r->write_off = 0;
        r->next_chunk_idx = 0;
    }

    if (hdr->chunk_idx != r->next_chunk_idx || hdr->chunk_count != r->chunk_count) {
        /* Out-of-order/duplicate/inconsistent chunk -- the transport is
         * expected to preserve order, so this means we missed one. Drop
         * what we had; this chunk itself is unusable without its
         * predecessors. */
        ar8030_reassembly_reset(r);
        return 0;
    }

    if (r->write_off + payload_len > r->cap) {
        fprintf(stderr, "reassembly: frame %u exceeds buffer (%u bytes), dropping\n", hdr->frame_seq,
                r->cap);
        ar8030_reassembly_reset(r);
        return 0;
    }

    memcpy(r->buf + r->write_off, payload, payload_len);
    r->write_off += payload_len;
    r->next_chunk_idx++;

    if (r->next_chunk_idx == r->chunk_count)
        return 1; /* complete */

    return 0;
}
