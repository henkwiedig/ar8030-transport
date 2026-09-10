#include "chunk_stream.h"

#include "crc16.h"

#include <string.h>

void ar8030_chunk_stream_init(ar8030_chunk_stream_t *s, ar8030_chunk_read_fn read_fn, void *read_ctx,
                               uint8_t *buf, uint32_t cap, int read_timeout_ms,
                               const volatile int *stop_flag)
{
    s->read_fn = read_fn;
    s->read_ctx = read_ctx;
    s->read_timeout_ms = read_timeout_ms;
    s->stop_flag = stop_flag;
    s->buf = buf;
    s->cap = cap;
    s->len = 0;
    s->chunks_read = 0;
    s->bytes_consumed = 0;
    s->resync_dropped_bytes = 0;
    s->checksum_fails = 0;
}

/* Appends whatever read_fn() has available right now. Returns 1 if it
 * added bytes, 0 on timeout/no-data. */
static int fill(ar8030_chunk_stream_t *s)
{
    if (s->len >= s->cap)
        return 0; /* see ar8030_chunk_stream_init()'s cap contract -- should not happen */

    int n = s->read_fn(s->read_ctx, s->buf + s->len, s->cap - s->len, s->read_timeout_ms);
    if (n <= 0)
        return 0;

    s->len += (uint32_t)n;
    return 1;
}

int ar8030_chunk_stream_read(ar8030_chunk_stream_t *s, struct ar8030_chunk_hdr *hdr,
                              uint8_t *payload_buf, uint32_t payload_buf_cap,
                              uint32_t *out_payload_len)
{
    for (;;) {
        if (s->stop_flag && *s->stop_flag)
            return 0;

        /* Resync: while we have enough buffered bytes to even ask the
         * question, keep dropping one leading byte at a time until
         * buf[0..HDR_SIZE) looks like a real header. In steady state this
         * loop body never runs more than once (the previous call already
         * left a valid header at the front, or none at all) -- it only
         * does real work right after a corrupted read or a sender
         * restart mid-stream.
         *
         * magic is the only field this can validate: payload_len can no
         * longer be range-checked against AR8030_CHUNK_MAX_PAYLOAD as a
         * secondary filter now that the ceiling is a uint16_t's full
         * range (65535) -- every value the field can hold is "in range"
         * by construction, so that comparison would always be true. The
         * actual bound that matters for memory safety (payload_len vs.
         * payload_buf_cap) is still checked below, per chunk. */
        while (s->len >= AR8030_CHUNK_HDR_SIZE) {
            memcpy(hdr, s->buf, AR8030_CHUNK_HDR_SIZE);
            if (hdr->magic == AR8030_CHUNK_MAGIC)
                break;
            memmove(s->buf, s->buf + 1, --s->len);
            s->resync_dropped_bytes++;
        }

        if (s->len >= AR8030_CHUNK_HDR_SIZE) {
            uint32_t need = (uint32_t)AR8030_CHUNK_HDR_SIZE + hdr->payload_len;
            if (s->len >= need) {
                if (hdr->payload_len > payload_buf_cap)
                    return -1; /* caller's payload buffer is misconfigured, not a stream error */

                const uint8_t *payload = s->buf + AR8030_CHUNK_HDR_SIZE;
                int checksum_ok = (ar8030_crc16(payload, hdr->payload_len) == hdr->checksum);
                if (hdr->payload_len)
                    memcpy(payload_buf, payload, hdr->payload_len);

                /* Either way, this chunk's bytes are spent: a checksum
                 * failure means the header was fine (nothing to resync
                 * past) and the payload is simply corrupt, so skip past
                 * it and keep looking for the next chunk in this same
                 * call rather than handing corrupt video data upstream. */
                memmove(s->buf, s->buf + need, s->len - need);
                s->len -= need;

                if (!checksum_ok) {
                    s->checksum_fails++;
                    continue;
                }

                *out_payload_len = hdr->payload_len;
                s->chunks_read++;
                s->bytes_consumed += need;
                return 1;
            }
        }

        if (!fill(s))
            return 0;
    }
}
