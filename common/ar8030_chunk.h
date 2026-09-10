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
 * _reserved is kept zeroed and is the only field a future FEC scheme can
 * add to without breaking this version's wire format (a receiver that
 * doesn't understand it just ignores it, same as today).
 */

#include <stdint.h>

#define AR8030_CHUNK_MAGIC 0x4D524641u /* "AFRM" little-endian on the wire */

/* Default per-chunk payload cap. bw_update_demo's own H.26x file-streaming
 * mode (app/bw_update_demo, "-l 9216") sends single bb_socket_write()
 * payloads well above this size successfully as a synthetic bandwidth
 * test, but bb_socket_write() waits (per-call timeout) for the daemon's
 * write ack before returning, and on a real, contended RF link under a
 * bursty real-time source (an IDR frame's worth of chunks arriving back
 * to back) that ack can lag well behind a short timeout -- confirmed on
 * bench hardware: 4096 produced sustained "bb_socket_write failed"
 * bursts once the link was under load. 1024 keeps each write's ack-wait
 * shorter and each dropped chunk cheaper, at the cost of more chunks per
 * frame. Tune with -c; AR8030_CHUNK_MAX_PAYLOAD is the hard ceiling this
 * can be raised to, not the default. */
#define AR8030_CHUNK_DEFAULT_PAYLOAD 1024u

/* Hard ceiling chunker.c's fixed scratch buffer is sized for. Comfortably
 * above bw_update_demo's own proven single-write sizes (see the comment
 * above) and above BB_CONFIG_MAC_TX_BUF_SIZE-class buffer sizes this SDK
 * uses elsewhere; raise it (and the scratch buffer) together if a
 * deployment genuinely needs bigger chunks. */
#define AR8030_CHUNK_MAX_PAYLOAD 8192u

/* Mirrors VENC_FRAME_FLAG_* from waybeam's venc_frame_ring.h bit-for-bit,
 * so tx/main.c can copy VencFrameMeta.flags straight across without a
 * translation table. */
#define AR8030_CHUNK_FLAG_IDR 0x01u
#define AR8030_CHUNK_FLAG_GDR 0x02u
#define AR8030_CHUNK_FLAG_ENHANCE 0x04u

#define AR8030_CHUNK_CODEC_H265 0x01u

#pragma pack(push, 1)
struct ar8030_chunk_hdr {
    uint32_t magic;       /* AR8030_CHUNK_MAGIC */
    uint16_t frame_seq;   /* wraps; one value per source frame */
    uint16_t chunk_idx;   /* 0-based index within this frame */
    uint16_t chunk_count; /* total chunks this frame was split into */
    uint8_t flags;        /* AR8030_CHUNK_FLAG_* */
    uint8_t codec;        /* AR8030_CHUNK_CODEC_* */
    uint32_t frame_pts;   /* VencFrameMeta.pts (us), same value on every
                            * chunk of a frame -- the RX side's RTP
                            * timestamp comes from this, not wall clock */
    uint16_t reserved;    /* zero; future FEC group id */
};
#pragma pack(pop)

#define AR8030_CHUNK_HDR_SIZE 18

#ifdef __cplusplus
static_assert(sizeof(struct ar8030_chunk_hdr) == AR8030_CHUNK_HDR_SIZE,
              "ar8030_chunk_hdr must be exactly 18 bytes");
#else
_Static_assert(sizeof(struct ar8030_chunk_hdr) == AR8030_CHUNK_HDR_SIZE,
               "ar8030_chunk_hdr must be exactly 18 bytes");
#endif

#endif /* AR8030_TRANSPORT_CHUNK_H */
