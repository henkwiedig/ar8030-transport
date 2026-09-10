/*
 * Host-only round-trip smoke test for the wire protocol, with no AR8030
 * SDK and no hardware involved:
 *
 *   synthetic Annex-B frame
 *     -> common/chunker.c            (same code tx/main.c uses)
 *     -> common/reassembly.c         (same code rx/main.c uses)
 *     -> assert byte-identical to the original frame
 *     -> rx/rtp_h265.c               (same code rx/main.c uses)
 *     -> a connected AF_UNIX SOCK_DGRAM socketpair standing in for the
 *        UDP socket to PixelPilot
 *     -> a small RTP/H.265 depacketizer (written only for this test)
 *     -> assert the reconstructed NALs match the originals exactly, and
 *        that RTP framing (marker bit, constant per-frame timestamp,
 *        monotonic sequence numbers) is well-formed.
 *
 * Exit 0 = PASS, 1 = FAIL. No pthread/SDK dependency -- builds with
 * plain `make -C test`.
 */

#include "ar8030_chunk.h"
#include "chunker.h"
#include "reassembly.h"
#include "rtp_h265.h"

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define FAIL(...)                                                                                \
    do {                                                                                         \
        fprintf(stderr, "FAIL: " __VA_ARGS__);                                                   \
        fprintf(stderr, "\n");                                                                   \
        exit(1);                                                                                 \
    } while (0)

/* ---- build a synthetic Annex-B frame with a small and a large NAL ---- */

struct nal_ref {
    const uint8_t *p; /* NAL bytes (header+body), start code excluded */
    uint32_t len;
};

static uint32_t build_annexb(uint8_t *out, uint32_t out_cap, struct nal_ref *nals, int *nal_count)
{
    uint32_t off = 0;
    int n = 0;

    /* NAL 1: small "VPS-like" NAL, type 32 -> header byte0 = (32<<1)=0x40 */
    static const uint8_t start4[4] = { 0, 0, 0, 1 };
    uint8_t nal1[40];
    nal1[0] = 0x40;
    nal1[1] = 0x01;
    for (unsigned i = 2; i < sizeof(nal1); i++)
        nal1[i] = (uint8_t)(0xA0 + i);

    /* NAL 2: small "SPS-like" NAL, type 33 -> header byte0 = (33<<1)=0x42 */
    uint8_t nal2[60];
    nal2[0] = 0x42;
    nal2[1] = 0x01;
    for (unsigned i = 2; i < sizeof(nal2); i++)
        nal2[i] = (uint8_t)(0x10 + i);

    /* NAL 3: large "slice" NAL, type 1 -> header byte0 = (1<<1)=0x02.
     * Large enough to force FU-A fragmentation at the test's chosen RTP
     * MTU and to span several AR8030 chunks at the test's chosen chunk
     * payload size. */
    static uint8_t nal3[5000];
    nal3[0] = 0x02;
    nal3[1] = 0x01;
    for (unsigned i = 2; i < sizeof(nal3); i++)
        nal3[i] = (uint8_t)(i * 37 + 11);

    const uint8_t *src[3] = { nal1, nal2, nal3 };
    uint32_t lens[3] = { sizeof(nal1), sizeof(nal2), sizeof(nal3) };

    for (int i = 0; i < 3; i++) {
        if (off + 4 + lens[i] > out_cap)
            FAIL("synthetic frame exceeds test buffer");
        memcpy(out + off, start4, 4);
        memcpy(out + off + 4, src[i], lens[i]);
        nals[n].p = out + off + 4;
        nals[n].len = lens[i];
        n++;
        off += 4 + lens[i];
    }

    *nal_count = n;
    return off;
}

/* ---- chunker -> reassembly round trip ---- */

struct chunk_capture {
    uint8_t bufs[256][AR8030_CHUNK_HDR_SIZE + AR8030_CHUNK_MAX_PAYLOAD];
    uint32_t lens[256];
    int count;
};

static int capture_chunk(void *ctx, const uint8_t *buf, uint32_t len)
{
    struct chunk_capture *cap = (struct chunk_capture *)ctx;
    if (cap->count >= 256)
        FAIL("test captured more chunks than expected");
    memcpy(cap->bufs[cap->count], buf, len);
    cap->lens[cap->count] = len;
    cap->count++;
    return 0;
}

/* ---- minimal RTP/H.265 depacketizer, for this test only ---- */

struct rtp_capture {
    uint8_t nals[16][8192];
    uint32_t nal_lens[16];
    int nal_count;
    uint16_t last_seq;
    int have_seq;
    uint32_t ts;
    int have_ts;
    int marker_count;
    uint32_t ssrc;

    /* in-progress FU reassembly */
    uint8_t fu_buf[8192];
    uint32_t fu_len;
    int fu_active;
};

static void depacketize_one(struct rtp_capture *rc, const uint8_t *pkt, uint32_t len)
{
    if (len < 12)
        FAIL("RTP packet shorter than a header (%u bytes)", len);
    if ((pkt[0] & 0xC0) != 0x80)
        FAIL("RTP version field wrong");

    int marker = (pkt[1] & 0x80) != 0;
    uint16_t seq;
    memcpy(&seq, pkt + 2, 2);
    seq = ntohs(seq);
    uint32_t ts;
    memcpy(&ts, pkt + 4, 4);
    ts = ntohl(ts);
    uint32_t ssrc;
    memcpy(&ssrc, pkt + 8, 4);
    ssrc = ntohl(ssrc);

    if (rc->have_seq && (uint16_t)(seq - rc->last_seq) != 1)
        FAIL("RTP sequence not monotonic (prev=%u this=%u)", rc->last_seq, seq);
    rc->last_seq = seq;
    rc->have_seq = 1;

    if (!rc->have_ts) {
        rc->ts = ts;
        rc->have_ts = 1;
    } else if (rc->ts != ts) {
        FAIL("RTP timestamp changed within one frame (%u -> %u)", rc->ts, ts);
    }

    if (rc->nal_count == 0 && rc->fu_active == 0 && ssrc != rc->ssrc && rc->ssrc != 0)
        FAIL("RTP SSRC changed mid-session");
    rc->ssrc = ssrc;

    if (marker)
        rc->marker_count++;

    const uint8_t *payload = pkt + 12;
    uint32_t plen = len - 12;
    if (plen < 2)
        FAIL("RTP payload too short to carry a NAL header");

    uint8_t type = (uint8_t)((payload[0] >> 1) & 0x3F);
    if (type != 49) {
        /* single-NAL packet */
        if (rc->nal_count >= 16)
            FAIL("test captured more NALs than expected");
        memcpy(rc->nals[rc->nal_count], payload, plen);
        rc->nal_lens[rc->nal_count] = plen;
        rc->nal_count++;
        return;
    }

    /* FU */
    if (plen < 3)
        FAIL("FU packet too short for PayloadHdr+FU header");
    uint8_t payload_hdr0 = payload[0];
    uint8_t payload_hdr1 = payload[1];
    uint8_t fu_hdr = payload[2];
    int s = (fu_hdr & 0x80) != 0;
    int e = (fu_hdr & 0x40) != 0;
    uint8_t fu_type = (uint8_t)(fu_hdr & 0x3F);

    if (s) {
        if (rc->fu_active)
            FAIL("FU start while a previous FU was still in progress");
        rc->fu_active = 1;
        rc->fu_len = 0;
        rc->fu_buf[rc->fu_len++] = (uint8_t)((payload_hdr0 & 0x81) | (fu_type << 1));
        rc->fu_buf[rc->fu_len++] = payload_hdr1;
    } else if (!rc->fu_active) {
        FAIL("FU continuation with no active FU");
    }

    uint32_t body_len = plen - 3;
    if (rc->fu_len + body_len > sizeof(rc->fu_buf))
        FAIL("reassembled FU exceeds test buffer");
    memcpy(rc->fu_buf + rc->fu_len, payload + 3, body_len);
    rc->fu_len += body_len;

    if (e) {
        if (rc->nal_count >= 16)
            FAIL("test captured more NALs than expected");
        memcpy(rc->nals[rc->nal_count], rc->fu_buf, rc->fu_len);
        rc->nal_lens[rc->nal_count] = rc->fu_len;
        rc->nal_count++;
        rc->fu_active = 0;
    }
}

int main(void)
{
    /* 1. Build the synthetic frame. */
    static uint8_t annexb[8192];
    struct nal_ref expected_nals[8];
    int expected_nal_count = 0;
    uint32_t annexb_len = build_annexb(annexb, sizeof(annexb), expected_nals, &expected_nal_count);
    fprintf(stderr, "built synthetic frame: %u bytes, %d NALs\n", annexb_len, expected_nal_count);

    /* 2. Chunk it (small payload to force many chunks) and feed the
     * chunks through reassembly in order, exactly as the wire carries
     * them. */
    const uint32_t chunk_payload = 300;
    struct chunk_capture cap;
    memset(&cap, 0, sizeof(cap));
    uint16_t frame_seq = 42;
    uint8_t flags = AR8030_CHUNK_FLAG_IDR;
    uint32_t frame_pts = 1234567;
    int sent = ar8030_chunk_frame(frame_seq, flags, AR8030_CHUNK_CODEC_H265, frame_pts, annexb,
                                   annexb_len, chunk_payload, capture_chunk, &cap);
    if (sent != cap.count)
        FAIL("ar8030_chunk_frame returned %d but callback saw %d chunks", sent, cap.count);
    fprintf(stderr, "chunked into %d chunks of <= %u bytes payload\n", cap.count, chunk_payload);

    static uint8_t reasm_buf[65536];
    ar8030_reassembly_t reasm;
    ar8030_reassembly_init(&reasm, reasm_buf, sizeof(reasm_buf));

    int completed = 0;
    for (int i = 0; i < cap.count; i++) {
        struct ar8030_chunk_hdr hdr;
        memcpy(&hdr, cap.bufs[i], AR8030_CHUNK_HDR_SIZE);
        const uint8_t *payload = cap.bufs[i] + AR8030_CHUNK_HDR_SIZE;
        uint32_t payload_len = cap.lens[i] - AR8030_CHUNK_HDR_SIZE;

        if (ar8030_reassembly_feed(&reasm, &hdr, payload, payload_len)) {
            completed = 1;
            break;
        }
    }
    if (!completed)
        FAIL("reassembly never completed");
    if (reasm.frame_seq != frame_seq)
        FAIL("reassembled frame_seq mismatch");
    if (reasm.frame_pts != frame_pts)
        FAIL("reassembled frame_pts mismatch");
    if (reasm.flags != flags)
        FAIL("reassembled flags mismatch");
    if (reasm.write_off != annexb_len)
        FAIL("reassembled length mismatch: got %u want %u", reasm.write_off, annexb_len);
    if (memcmp(reasm.buf, annexb, annexb_len) != 0)
        FAIL("reassembled bytes differ from the original frame");
    fprintf(stderr, "chunker+reassembly round trip: byte-identical (%u bytes)\n", annexb_len);

    /* 3. RTP/H.265-packetize the reassembled frame over a connected
     * AF_UNIX SOCK_DGRAM pair (same send()-on-connected-socket path
     * rx/main.c uses for the real UDP socket) and depacketize it back. */
    int sv[2];
    if (socketpair(AF_UNIX, SOCK_DGRAM, 0, sv) != 0)
        FAIL("socketpair failed");

    rtp_h265_ctx_t rtp_ctx;
    rtp_h265_init(&rtp_ctx, sv[0], 200 /* small MTU to force FU-A on the big NAL */);

    uint32_t rtp_ts = (uint32_t)(((uint64_t)frame_pts * 90ull) / 1000ull);
    int rtp_packets = rtp_h265_send_frame(&rtp_ctx, reasm.buf, reasm.write_off, rtp_ts);
    if (rtp_packets <= 0)
        FAIL("rtp_h265_send_frame sent no packets");
    fprintf(stderr, "RTP/H.265 packetized into %d packets\n", rtp_packets);

    struct rtp_capture rc;
    memset(&rc, 0, sizeof(rc));
    for (int i = 0; i < rtp_packets; i++) {
        uint8_t pkt[2048];
        ssize_t n = recv(sv[1], pkt, sizeof(pkt), 0);
        if (n <= 0)
            FAIL("recv() got no packet %d/%d", i, rtp_packets);
        depacketize_one(&rc, pkt, (uint32_t)n);
    }
    close(sv[0]);
    close(sv[1]);

    if (rc.ts != rtp_ts)
        FAIL("RTP timestamp mismatch: got %u want %u", rc.ts, rtp_ts);
    if (rc.marker_count != 1)
        FAIL("expected exactly one marker-bit packet, saw %d", rc.marker_count);
    if (rc.nal_count != expected_nal_count)
        FAIL("depacketized %d NALs, expected %d", rc.nal_count, expected_nal_count);

    for (int i = 0; i < expected_nal_count; i++) {
        if (rc.nal_lens[i] != expected_nals[i].len)
            FAIL("NAL %d length mismatch: got %u want %u", i, rc.nal_lens[i], expected_nals[i].len);
        if (memcmp(rc.nals[i], expected_nals[i].p, expected_nals[i].len) != 0)
            FAIL("NAL %d content mismatch after RTP round trip", i);
    }
    fprintf(stderr, "RTP round trip: %d/%d NALs byte-identical, marker bit correct, "
                     "sequence monotonic, timestamp constant\n",
            rc.nal_count, expected_nal_count);

    fprintf(stderr, "PASS\n");
    return 0;
}
