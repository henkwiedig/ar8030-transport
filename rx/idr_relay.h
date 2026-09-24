#ifndef AR8030_TRANSPORT_IDR_RELAY_H
#define AR8030_TRANSPORT_IDR_RELAY_H

#include "ar8030_link.h"

/*
 * Keyframe requests from PixelPilot to the air unit.
 *
 * PixelPilot asks for an IDR by sending a short random token ("abc\n") to
 * UDP port 11223 on the RTP sender's address -- on an Artosyn ground that
 * is this process. The listener used to be alink_idr on the air unit
 * (sickgreg/aalink_idr5): it answered "ACK:<token>\n" and called
 * waybeam's GET /request/idr. Here the request crosses the radio instead:
 * each token is answered with the same ACK right away (PixelPilot itself
 * doesn't wait for it -- it stops retrying once an IDR shows up in the
 * stream) and written as one AR8030_CHUNK_CODEC_CTRL "IDR <token>" chunk
 * into the reverse direction of the video socket, where
 * ar8030-transport-tx turns it into the waybeam call (tx/idr_ctrl.c).
 */

typedef struct {
    ar8030_link_t *link;           /* sockfd loaded atomically per write */
    int udp_port;                  /* 11223 like alink_idr */
    int verbose;
    const volatile int *stop_flag;
} idr_relay_cfg_t;

/* pthread entry point; cfg must outlive the thread. */
void *idr_relay_thread_main(void *cfg);

#endif
