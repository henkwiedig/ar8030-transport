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

/* Number of leading bytes of buf[0..len) that are junk for certain: the
 * offset of the first complete chunk magic, or, with no complete match,
 * everything but the last three bytes (which may be the start of a magic
 * that the next read completes).
 *
 * One memchr() pass for the magic's first byte instead of testing every
 * offset, and the caller then drops the whole junk run with a single
 * memmove(): resyncing a buffer of n bytes costs O(n), not the O(n) per
 * dropped byte of shifting it down one byte at a time (measured on the
 * VR04: 223 us per dropped byte with a 262 KB buffer, so a 47 KB junk run
 * kept the receiver busy for seconds and let the socket overflow again). */
static uint32_t scan_for_magic(const uint8_t *buf, uint32_t len)
{
    const uint32_t magic = AR8030_CHUNK_MAGIC;
    uint8_t mb[sizeof(magic)];
    memcpy(mb, &magic, sizeof(mb));

    uint32_t i = 0;
    while (len - i >= sizeof(mb)) {
        const uint8_t *p = memchr(buf + i, mb[0], len - i - (sizeof(mb) - 1));
        if (!p)
            break;
        i = (uint32_t)(p - buf);
        if (memcmp(p, mb, sizeof(mb)) == 0)
            return i;
        i++;
    }
    return len >= sizeof(mb) - 1 ? len - (uint32_t)(sizeof(mb) - 1) : 0;
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

        /* Resync: drop everything in front of the next chunk magic. In
         * steady state there is nothing to drop (the previous call left
         * a valid header at the front, or none at all) -- it only does
         * real work right after a corrupted read, a hole in the stream
         * or a sender restart mid-stream.
         *
         * magic is the only field this can validate: payload_len can no
         * longer be range-checked against AR8030_CHUNK_MAX_PAYLOAD as a
         * secondary filter now that the ceiling is a uint16_t's full
         * range (65535) -- every value the field can hold is "in range"
         * by construction, so that comparison would always be true. The
         * actual bound that matters for memory safety (payload_len vs.
         * payload_buf_cap) is still checked below, per chunk. */
        if (s->len >= sizeof(uint32_t)) {
            uint32_t junk = scan_for_magic(s->buf, s->len);
            if (junk) {
                s->len -= junk;
                memmove(s->buf, s->buf + junk, s->len);
                s->resync_dropped_bytes += junk;
            }
        }

        if (s->len >= AR8030_CHUNK_HDR_SIZE) {
            memcpy(hdr, s->buf, AR8030_CHUNK_HDR_SIZE);
            uint32_t need = (uint32_t)AR8030_CHUNK_HDR_SIZE + hdr->payload_len;
            if (s->len >= need) {
                if (hdr->payload_len > payload_buf_cap)
                    return -1; /* caller's payload buffer is misconfigured, not a stream error */

                const uint8_t *payload = s->buf + AR8030_CHUNK_HDR_SIZE;
                int checksum_ok = (ar8030_crc16(payload, hdr->payload_len) == hdr->checksum);

                if (!checksum_ok) {
                    /* Not even payload_len can be trusted now: bytes lost
                     * from the middle of the stream (the socket's receive
                     * buffer overflowing while this process was stalled)
                     * make a chunk look shorter on the wire than its
                     * header claims, so the claimed length reaches into
                     * the chunks behind it. Skipping that length would
                     * throw those away as well and force a long resync;
                     * dropping just the magic lets the next scan pick the
                     * following chunk up wherever it really starts. A
                     * chunk that was only corrupted in place costs the
                     * same either way: the scan passes over its payload. */
                    s->checksum_fails++;
                    s->len -= sizeof(uint32_t);
                    memmove(s->buf, s->buf + sizeof(uint32_t), s->len);
                    s->resync_dropped_bytes += sizeof(uint32_t);
                    continue;
                }

                if (hdr->payload_len)
                    memcpy(payload_buf, payload, hdr->payload_len);
                memmove(s->buf, s->buf + need, s->len - need);
                s->len -= need;

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
