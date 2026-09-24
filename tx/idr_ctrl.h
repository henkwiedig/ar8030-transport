#ifndef AR8030_TRANSPORT_IDR_CTRL_H
#define AR8030_TRANSPORT_IDR_CTRL_H

#include "ar8030_link.h"

/*
 * Air side of the keyframe request path (see rx/idr_relay.h): reads the
 * reverse direction of the video socket -- opened TX|RX, the default --
 * for AR8030_CHUNK_CODEC_CTRL chunks and turns "IDR <token>" into
 * waybeam's GET /request/idr, the call alink_idr used to make.
 *
 * Two filters before calling waybeam:
 *  - a repeated token (a retransmission of the same request) is ignored
 *    for dedup_ms, like alink_idr's --keep-ms;
 *  - a new token within coalesce_ms of the last honored request is
 *    dropped too. PixelPilot sends each request as a burst of 3 distinct
 *    tokens 100 ms apart, and waybeam's own IDR rate limit only coalesces
 *    requests less than 100 ms apart -- so without this one burst could
 *    force up to three keyframes. A request that PixelPilot repeats
 *    because no IDR arrived comes after the window and gets through.
 */

typedef struct {
    ar8030_link_t *link;       /* sockfd loaded atomically per read */
    const char *waybeam_host;
    int waybeam_port;
    int coalesce_ms;           /* 0 = honor every new token */
    int dedup_ms;
    int verbose;
    const volatile int *stop_flag;
} idr_ctrl_cfg_t;

/* pthread entry point; cfg must outlive the thread. */
void *idr_ctrl_thread_main(void *cfg);

#endif
