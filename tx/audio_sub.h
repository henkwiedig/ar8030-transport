#ifndef AR8030_TRANSPORT_AUDIO_SUB_H
#define AR8030_TRANSPORT_AUDIO_SUB_H

#include "outq.h"

/*
 * Receives waybeam's audio -- RTP/Opus packets on a loopback UDP port,
 * waybeam's outgoing.audioPort -- and queues each one, verbatim, for the
 * ground as an AR8030_CHUNK_CODEC_OPUS chunk on the video socket, the way
 * tx/sidecar_sub.c does with the sidecar. The main send loop writes it
 * together with the next video chunk; the ground routes it out before
 * reassembly (rx publishes it to its audio ring, or as RTP next to the
 * video).
 *
 * The video socket because audio has no port of its own: only bb ports 2
 * and 3 are usable and ar_net0 has 3 (audio_tx/audio_rx predate that).
 * docs/ground-frame-ring.md, "Audio".
 */

typedef struct {
    int port; /* waybeam outgoing.audioPort, bound on 127.0.0.1 */
    tx_outq_t *outq;
    int verbose;
    const volatile int *stop_flag;
    uint64_t forwarded; /* written by the thread only; read racily for -v stats */
    uint64_t too_big;   /* packets over AR8030_SIDECAR_MAX_PAYLOAD, not sent */
} audio_sub_cfg_t;

/* pthread entry point; cfg must outlive the thread. */
void *audio_sub_thread_main(void *cfg);

#endif /* AR8030_TRANSPORT_AUDIO_SUB_H */
