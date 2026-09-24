#ifndef AR8030_TRANSPORT_CHUNK_H
#define AR8030_TRANSPORT_CHUNK_H

/*
 * Wire format for one whole encoded video frame sent over an AR8030
 * bb_socket. A frame from waybeam's frame-shm ring (up to slot_data_size,
 * hundreds of KB) is split into chunks of at most AR8030_CHUNK_MAX_PAYLOAD
 * bytes; each chunk is one bb_socket_write() call, prefixed with this
 * header. There is no ACK/NACK and no FEC in this version -- a chunk lost
 * on the radio just makes its frame incomplete, and the receiver drops the
 * whole frame rather than blocking to wait for it (see rx/main.c).
 *
 * bb_socket runs in stream mode (no BB_SOCK_FLAG_DATAGRAM -- see
 * README.md "Stream mode, not datagram", matching the stock vendor
 * streamer's own bb_socket_open() calls, reverse-engineered from
 * ar_ldyhs_sky), so unlike an earlier version of this format, a
 * bb_socket_read() is NOT guaranteed to return exactly one write's worth
 * of bytes -- it can return a partial chunk, several concatenated, or
 * anything in between. payload_len is therefore load-bearing: it is how
 * common/chunk_stream.c knows where one chunk ends and the next begins
 * in the raw byte stream, which the transport itself no longer tells us.
 *
 * checksum (common/crc16.c) covers this chunk's payload bytes only, not
 * the header -- the header's own well-formedness (magic, payload_len in
 * range) is what chunk_stream.c's resync already validates, so this is
 * purely about catching bit errors in the payload itself that the AR8030
 * baseband's own error handling didn't. See crc16.h's comment for why
 * this was added: tx/rx showed zero write failures, zero resyncs, zero
 * dropped frames, yet visibly corrupted video -- a decoder fed
 * corrupted-but-present data, not missing data.
 *
 * _reserved is kept zeroed and is the only field a future FEC scheme can
 * add to without breaking this version's wire format (a receiver that
 * doesn't understand it just ignores it, same as today).
 */

#include <stdint.h>

#define AR8030_CHUNK_MAGIC 0x4D524641u /* "AFRM" little-endian on the wire */

/* Default per-chunk payload cap. The stock vendor streamer (ar_ldyhs_sky,
 * reverse-engineered) does not chunk at the application layer at all: it
 * accumulates a whole encoded frame and hands it to bb_socket_write() in
 * ONE call (looped only on partial-write progress, same as
 * chunk_send_to_socket() in tx/main.c), relying on stream mode's
 * partial-write tolerance rather than any fixed transport-level slice
 * size -- see README.md "Chunk sizing" for the full writeup. Given our
 * own reassembly is all-or-nothing per frame (no FEC, no NACK -- any one
 * missing/corrupt chunk drops the whole frame regardless of which chunk),
 * small fixed chunks buy nothing but cost more: every bb_socket_write()
 * is an RPC round trip to ar8030d, and more, smaller chunks per frame
 * means more RPC volume (which is what filled ar8030d's own debug log
 * and caused an OOM -- see README's "Diagnosing 'nothing is getting
 * through'") and more independent chances for one transfer glitch to
 * drop an entire frame. So this defaults to the wire format's actual
 * ceiling (see AR8030_CHUNK_MAX_PAYLOAD) rather than a conservative
 * fraction of it: most frames go out as a single chunk, matching the
 * vendor's own proven approach. -c can still lower it for experimentation
 * on a particularly poor link. */
#define AR8030_CHUNK_DEFAULT_PAYLOAD 65535u

/* Hard ceiling: payload_len below is a uint16_t, so no chunk can ever
 * carry more than 65535 bytes of payload regardless of how large
 * chunk_payload is asked to be -- this is a wire-format limit, not a
 * buffer-sizing choice. A frame bigger than this still needs more than
 * one chunk no matter what -c is set to. */
#define AR8030_CHUNK_MAX_PAYLOAD 65535u

/* Mirrors VENC_FRAME_FLAG_* from waybeam's venc_frame_ring.h bit-for-bit,
 * so tx/main.c can copy VencFrameMeta.flags straight across without a
 * translation table. */
#define AR8030_CHUNK_FLAG_IDR 0x01u
#define AR8030_CHUNK_FLAG_GDR 0x02u
#define AR8030_CHUNK_FLAG_ENHANCE 0x04u

#define AR8030_CHUNK_CODEC_H265 0x01u
/* A chunk of this codec carries one whole, already RTP-packetized audio
 * datagram (waybeam's cv610_audio.c own RTP/Opus packetizer output, PT=98)
 * verbatim -- see audio_tx/main.c and audio_rx/main.c. Unlike H.265, there
 * is no Annex-B re-parsing or RTP re-packetization on either end: the
 * bytes that go in on the air side are the exact bytes handed to a UDP
 * `send()` on the ground side. */
#define AR8030_CHUNK_CODEC_OPUS 0x02u

/* A control message, ground -> air, on the reverse direction of the same
 * video socket (both ends open it TX|RX). One single-chunk frame whose
 * payload is one ASCII command line, no terminator:
 *   "IDR <token>"  request a keyframe -- PixelPilot's IDR token (formerly
 *                  sent to alink_idr on UDP 11223), relayed by
 *                  rx/idr_relay.c and handled by tx/idr_ctrl.c, which
 *                  calls waybeam's GET /request/idr.
 * Unknown commands are ignored, so new ones can be added later. frame_pts
 * and flags are 0; frame_seq counts control messages on their own. */
#define AR8030_CHUNK_CODEC_CTRL 0x03u
#define AR8030_CTRL_MAX_PAYLOAD 64u

#pragma pack(push, 1)
struct ar8030_chunk_hdr {
    uint32_t magic;       /* AR8030_CHUNK_MAGIC */
    uint16_t frame_seq;   /* wraps; one value per source frame */
    uint16_t chunk_idx;   /* 0-based index within this frame */
    uint16_t chunk_count; /* total chunks this frame was split into */
    uint16_t payload_len; /* bytes following this header for this chunk;
                            * see the file header comment on why stream
                            * mode makes this load-bearing, not advisory */
    uint8_t flags;        /* AR8030_CHUNK_FLAG_* */
    uint8_t codec;        /* AR8030_CHUNK_CODEC_* */
    uint32_t frame_pts;   /* VencFrameMeta.pts (us), same value on every
                            * chunk of a frame -- the RX side's RTP
                            * timestamp comes from this, not wall clock */
    uint16_t checksum;    /* ar8030_crc16() over this chunk's payload bytes */
    uint16_t reserved;    /* zero; future FEC group id */
};
#pragma pack(pop)

#define AR8030_CHUNK_HDR_SIZE 22

#ifdef __cplusplus
static_assert(sizeof(struct ar8030_chunk_hdr) == AR8030_CHUNK_HDR_SIZE,
              "ar8030_chunk_hdr must be exactly 22 bytes");
#else
_Static_assert(sizeof(struct ar8030_chunk_hdr) == AR8030_CHUNK_HDR_SIZE,
               "ar8030_chunk_hdr must be exactly 22 bytes");
#endif

#endif /* AR8030_TRANSPORT_CHUNK_H */
