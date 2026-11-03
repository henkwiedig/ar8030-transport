#include "chunker.h"

#include "ar8030_chunk.h"
#include "crc16.h"

#include <string.h>

/* Shared by both entry points. total_count is the chunk_count every chunk
 * carries, except that with frame_end the last one carries
 * first_idx + count instead, plus AR8030_CHUNK_FLAG_FRAME_END. */
static int chunk_run(uint16_t frame_seq, uint8_t flags, uint8_t codec, uint32_t frame_pts, const uint8_t *data,
                     uint32_t data_len, uint32_t chunk_payload, uint32_t first_idx, uint32_t total_count,
                     int frame_end, ar8030_chunk_send_fn send, void *send_ctx, uint32_t *out_chunk_count,
                     uint8_t *scratch, uint32_t scratch_cap)
{
    if (chunk_payload == 0 || chunk_payload > AR8030_CHUNK_MAX_PAYLOAD)
        return -1;
    if (scratch_cap < (uint32_t)AR8030_CHUNK_HDR_SIZE + chunk_payload)
        return -1;

    uint32_t chunk_count = (data_len + chunk_payload - 1) / chunk_payload;
    if (chunk_count == 0)
        chunk_count = 1; /* still send one (empty) chunk for a zero-length frame */
    if (first_idx + chunk_count > 0xFFFFu)
        return -1;
    if (out_chunk_count)
        *out_chunk_count = chunk_count;

    uint8_t *sendbuf = scratch;
    int sent = 0;

    for (uint32_t i = 0; i < chunk_count; i++) {
        uint32_t off = i * chunk_payload;
        uint32_t len = data_len - off;
        if (len > chunk_payload)
            len = chunk_payload;
        int final = frame_end && i + 1 == chunk_count;

        struct ar8030_chunk_hdr hdr;
        hdr.magic = AR8030_CHUNK_MAGIC;
        hdr.frame_seq = frame_seq;
        hdr.chunk_idx = (uint16_t)(first_idx + i);
        hdr.chunk_count = (uint16_t)(final ? first_idx + chunk_count : total_count);
        hdr.payload_len = (uint16_t)len;
        hdr.flags = (uint8_t)(final ? flags | AR8030_CHUNK_FLAG_FRAME_END : flags);
        hdr.codec = codec;
        hdr.frame_pts = frame_pts;
        hdr.checksum = ar8030_crc16(data + off, len);
        hdr.reserved = 0;

        memcpy(sendbuf, &hdr, AR8030_CHUNK_HDR_SIZE);
        if (len)
            memcpy(sendbuf + AR8030_CHUNK_HDR_SIZE, data + off, len);

        if (send(send_ctx, sendbuf, AR8030_CHUNK_HDR_SIZE + len) != 0)
            break; /* caller's send failed -- stop, don't retry (see header comment) */
        sent++;
    }

    return sent;
}

int ar8030_chunk_frame(uint16_t frame_seq, uint8_t flags, uint8_t codec, uint32_t frame_pts,
                        const uint8_t *data, uint32_t data_len, uint32_t chunk_payload,
                        ar8030_chunk_send_fn send, void *send_ctx, uint32_t *out_chunk_count,
                        uint8_t *scratch, uint32_t scratch_cap)
{
    if (chunk_payload == 0 || chunk_payload > AR8030_CHUNK_MAX_PAYLOAD)
        return -1;
    uint32_t chunk_count = (data_len + chunk_payload - 1) / chunk_payload;
    if (chunk_count == 0)
        chunk_count = 1;
    if (chunk_count > 0xFFFFu)
        return -1;
    return chunk_run(frame_seq, flags, codec, frame_pts, data, data_len, chunk_payload, 0, chunk_count, 0, send,
                     send_ctx, out_chunk_count, scratch, scratch_cap);
}

int ar8030_chunk_part(uint16_t frame_seq, uint8_t flags, uint8_t codec, uint32_t frame_pts,
                       const uint8_t *data, uint32_t data_len, uint32_t chunk_payload,
                       uint32_t first_idx, int last, ar8030_chunk_send_fn send, void *send_ctx,
                       uint32_t *out_chunk_count, uint8_t *scratch, uint32_t scratch_cap)
{
    return chunk_run(frame_seq, flags, codec, frame_pts, data, data_len, chunk_payload, first_idx, 0, last, send,
                     send_ctx, out_chunk_count, scratch, scratch_cap);
}
