#include "rtp_h265.h"

#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define RTP_VERSION2 0x80
#define RTP_PT_H265 96 /* dynamic PT; PixelPilot's H265 caps don't pin one (see gstrtpreceiver.cpp
                         * gst_create_rtp_caps: only H264's caps string pins payload=96) */
#define RTP_HDR_LEN 12
#define H265_NAL_FU 49

static uint32_t seed_from_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_nsec ^ (ts.tv_sec * 2654435761u));
}

void rtp_h265_init(rtp_h265_ctx_t *ctx, int sockfd, uint16_t max_payload)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->sockfd = sockfd;
    srand(seed_from_time());
    ctx->ssrc = ((uint32_t)rand() << 16) ^ (uint32_t)rand();
    ctx->seq = (uint16_t)rand();
    /* Clamp to what send_packet()'s fixed buffer (RTP_SEND_BUF_MAX) can
     * actually hold; a caller passing something absurd gets a working,
     * smaller MTU instead of silently-dropped packets. */
    ctx->max_payload = max_payload > 8192 ? 8192 : max_payload;
}

static void put_rtp_header(uint8_t *buf, uint16_t seq, uint32_t ts, uint32_t ssrc, int marker)
{
    buf[0] = RTP_VERSION2;
    buf[1] = (uint8_t)((marker ? 0x80 : 0x00) | (RTP_PT_H265 & 0x7F));
    uint16_t seq_n = htons(seq);
    memcpy(buf + 2, &seq_n, 2);
    uint32_t ts_n = htonl(ts);
    memcpy(buf + 4, &ts_n, 4);
    uint32_t ssrc_n = htonl(ssrc);
    memcpy(buf + 8, &ssrc_n, 4);
}

/* Fixed send buffer: RTP header + up to a 3-byte FU prefix + the caller's
 * max_payload (rx/main.c's -M flag is validated against this bound). */
#define RTP_SEND_BUF_MAX (RTP_HDR_LEN + 3 + 8192)

static int send_packet(rtp_h265_ctx_t *ctx, const uint8_t *payload_a, uint32_t len_a,
                        const uint8_t *payload_b, uint32_t len_b, uint32_t ts, int marker)
{
    uint8_t buf[RTP_SEND_BUF_MAX];
    if (RTP_HDR_LEN + len_a + len_b > sizeof(buf))
        return -1;

    put_rtp_header(buf, ctx->seq, ts, ctx->ssrc, marker);
    uint32_t off = RTP_HDR_LEN;
    if (len_a) {
        memcpy(buf + off, payload_a, len_a);
        off += len_a;
    }
    if (len_b) {
        memcpy(buf + off, payload_b, len_b);
        off += len_b;
    }

    ssize_t wr = send(ctx->sockfd, buf, off, 0);
    ctx->seq++;
    return wr < 0 ? -1 : 0;
}

/* Finds the next Annex-B NAL after "from". Returns 1 and fills nal/nal_len
 * on success (NAL bytes, start code excluded), 0 when no more NALs remain. */
static int next_nal(const uint8_t *data, uint32_t size, uint32_t *from, const uint8_t **nal,
                     uint32_t *nal_len)
{
    uint32_t i = *from;
    uint32_t start = 0;
    int found_start = 0;
    while (i + 2 < size) {
        if (data[i] == 0 && data[i + 1] == 0) {
            if (data[i + 2] == 1) {
                start = i + 3;
                found_start = 1;
                break;
            }
            if (i + 3 < size && data[i + 2] == 0 && data[i + 3] == 1) {
                start = i + 4;
                found_start = 1;
                break;
            }
        }
        i++;
    }
    if (!found_start)
        return 0;

    uint32_t j = start;
    uint32_t end = size;
    while (j + 2 < size) {
        if (data[j] == 0 && data[j + 1] == 0 && (data[j + 2] == 1 || (j + 3 < size && data[j + 2] == 0 && data[j + 3] == 1))) {
            end = j;
            break;
        }
        j++;
    }

    if (end <= start)
        return 0;

    *nal = data + start;
    *nal_len = end - start;
    *from = end;
    return 1;
}

int rtp_h265_send_frame(rtp_h265_ctx_t *ctx, const uint8_t *annexb, uint32_t len, uint32_t rtp_ts_90k)
{
    uint32_t cursor = 0;
    const uint8_t *nal;
    uint32_t nal_len;
    int packets = 0;

    /* Collect NAL offsets first so we know which one is last (for the RTP
     * marker bit) without a second scan pass. Frames are whole access
     * units from the AR8030 chunk reassembly, so NAL counts are small
     * (tens, not thousands) -- a fixed-size offset table is fine. */
    struct {
        const uint8_t *p;
        uint32_t len;
    } nals[256];
    int nal_count = 0;
    while (nal_count < 256 && next_nal(annexb, len, &cursor, &nal, &nal_len)) {
        nals[nal_count].p = nal;
        nals[nal_count].len = nal_len;
        nal_count++;
    }
    if (nal_count == 0)
        return -1;

    uint16_t max_frag = ctx->max_payload > 3 ? (uint16_t)(ctx->max_payload - 3) : 1;

    for (int n = 0; n < nal_count; n++) {
        const uint8_t *p = nals[n].p;
        uint32_t plen = nals[n].len;
        int is_last_nal = (n == nal_count - 1);

        if (plen < 2)
            continue; /* not a valid NAL (needs a 2-byte H.265 header) */

        if (plen <= ctx->max_payload) {
            if (send_packet(ctx, p, plen, NULL, 0, rtp_ts_90k, is_last_nal) == 0)
                packets++;
            continue;
        }

        uint8_t nal_hdr0 = p[0];
        uint8_t nal_hdr1 = p[1];
        uint8_t nal_type = (uint8_t)((nal_hdr0 >> 1) & 0x3F);
        uint8_t payload_hdr0 = (uint8_t)((nal_hdr0 & 0x81) | ((H265_NAL_FU & 0x3F) << 1));
        uint8_t payload_hdr1 = nal_hdr1;

        const uint8_t *body = p + 2;
        uint32_t body_len = plen - 2;
        uint32_t off = 0;
        while (off < body_len) {
            uint32_t frag_len = body_len - off;
            if (frag_len > max_frag)
                frag_len = max_frag;

            int start = (off == 0);
            int end = (off + frag_len >= body_len);
            uint8_t fu_hdr = (uint8_t)(((start ? 1 : 0) << 7) | ((end ? 1 : 0) << 6) | (nal_type & 0x3F));
            uint8_t prefix[3] = { payload_hdr0, payload_hdr1, fu_hdr };

            int marker = is_last_nal && end;
            if (send_packet(ctx, prefix, sizeof(prefix), body + off, frag_len, rtp_ts_90k, marker) == 0)
                packets++;

            off += frag_len;
        }
    }

    return packets;
}
