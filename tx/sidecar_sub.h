#ifndef AR8030_TRANSPORT_SIDECAR_SUB_H
#define AR8030_TRANSPORT_SIDECAR_SUB_H

#include "outq.h"

/*
 * Subscribes to waybeam's RTP timing sidecar on the air unit (UDP on
 * loopback, waybeam's outgoing.sidecarPort) and queues every MSG_FRAME
 * datagram, verbatim, for the ground as an AR8030_CHUNK_CODEC_SIDECAR
 * chunk (see common/ar8030_chunk.h and rx/sidecar_srv.c).
 *
 * waybeam only sends while a subscriber keeps refreshing its slot
 * (5 s TTL), so SUBSCRIBE is re-sent every 2 s -- which also recovers on
 * its own after a waybeam restart.
 */

typedef struct {
    int port; /* waybeam outgoing.sidecarPort */
    tx_outq_t *outq;
    int verbose;
    const volatile int *stop_flag;
    uint64_t forwarded; /* written by the thread only; read racily for -v stats */
} sidecar_sub_cfg_t;

/* pthread entry point; cfg must outlive the thread. */
void *sidecar_sub_thread_main(void *cfg);

#endif
