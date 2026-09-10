#ifndef AR8030_TRANSPORT_RX_RTP_H265_H
#define AR8030_TRANSPORT_RX_RTP_H265_H

/*
 * Minimal RFC 7798 RTP/H.265 packetizer: single-NAL packets when a NAL
 * fits the configured max payload, RFC 7798 SS4.4.3 Fragmentation Units
 * (FU) otherwise. Self-contained Annex-B NAL scanner -- deliberately not
 * sharing code with waybeam's h26x_util.c, since the ground side never
 * checks out waybeam source (see third_party/waybeam_frame_ring/README.md
 * for why the *ring* code is vendored but this one is written fresh).
 *
 * Targets PixelPilot_rk's GstRtpReceiver pipeline
 * (gstrtpreceiver.cpp: rtph265depay ! h265parse ...), which expects
 * standard RTP/H.265 -- nothing PixelPilot-specific here, this is plain
 * RFC 7798.
 */

#include <stdint.h>

typedef struct {
    int sockfd;      /* UDP socket, already connect()ed to the target */
    uint32_t ssrc;    /* random, chosen once at startup */
    uint16_t seq;     /* next RTP sequence number to send */
    uint16_t max_payload; /* RTP payload cap, e.g. 1400 (see rx/main.c -M) */
} rtp_h265_ctx_t;

/* sockfd must already be a connected UDP socket (see rx/main.c). Picks a
 * random initial sequence number and SSRC. */
void rtp_h265_init(rtp_h265_ctx_t *ctx, int sockfd, uint16_t max_payload);

/* Packetizes one whole Annex-B access unit (as reassembled by rx/main.c
 * from AR8030 chunks) into one or more RTP/H.265 packets and sends them.
 * rtp_ts_90k is the RTP 90kHz timestamp for every packet of this frame
 * (see rx/main.c's derivation from ar8030_chunk_hdr.frame_pts). Returns
 * the number of RTP packets sent, or -1 if annexb contains no NALs. */
int rtp_h265_send_frame(rtp_h265_ctx_t *ctx, const uint8_t *annexb, uint32_t len,
                         uint32_t rtp_ts_90k);

#endif /* AR8030_TRANSPORT_RX_RTP_H265_H */
