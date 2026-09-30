#ifndef AR8030_TRANSPORT_SIDECAR_SRV_H
#define AR8030_TRANSPORT_SIDECAR_SRV_H

/*
 * Serves the waybeam RTP timing sidecar on the ground, from the datagrams
 * tx/sidecar_sub.c tunnels over the link (AR8030_CHUNK_CODEC_SIDECAR).
 *
 * To a ground consumer this looks like waybeam itself: bind nothing,
 * send SUBSCRIBE to the port, refresh within 5 s, receive MSG_FRAME
 * datagrams unchanged; up to 4 subscribers, keyed by address. SYNC_REQ is
 * answered on the air unit's clock (ground clock + the SYNC/SYNCR offset
 * from rx/clock_sync.c), so a probe's own clock mapping lines up with the
 * capture/encode times inside the frames it receives.
 *
 * The listening side runs on its own thread so SYNC_REQ turnaround is not
 * held up by the video loop; rx's main loop only calls
 * sidecar_srv_publish().
 */

#include <netinet/in.h>
#include <pthread.h>
#include <stdint.h>

#include "clock_sync.h"
#include "sidecar_wire.h"

typedef struct {
    int port; /* local UDP port, 0 = off */
    clock_sync_t *sync;
    const volatile int *stop_flag;
    int verbose;

    /* Private. */
    int fd;
    pthread_mutex_t mu;
    struct {
        struct sockaddr_in addr;
        uint64_t expires_us;
    } subs[SIDECAR_MAX_SUBS];
    /* capture -> encode done, us, exponential average; 0 = none yet. Read
     * by rx's main loop for the timing SEI. */
    uint32_t encode_avg_us;
    uint64_t published;
} sidecar_srv_t;

/* Initialises s and binds the port (nothing is bound for port 0). Returns
 * 0, or -1 (logged) if binding failed -- s is still usable, unserved. */
int sidecar_srv_open(sidecar_srv_t *s);
/* pthread entry point for the subscribe/sync listener. */
void *sidecar_srv_thread_main(void *s);
/* One tunnelled MSG_FRAME datagram: updates the encode-time average and
 * sends it to every live subscriber. */
void sidecar_srv_publish(sidecar_srv_t *s, const uint8_t *dgram, uint32_t len);
void sidecar_srv_close(sidecar_srv_t *s);

#endif
