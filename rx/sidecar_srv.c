#include "sidecar_srv.h"

#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define SIDECAR_SRV_POLL_MS 200

int sidecar_srv_open(sidecar_srv_t *s)
{
    memset(s->subs, 0, sizeof(s->subs));
    s->encode_avg_us = 0;
    s->published = 0;
    pthread_mutex_init(&s->mu, NULL);
    s->fd = -1;
    if (s->port <= 0)
        return 0; /* not served, but publish() still tracks the encode time */

    s->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (s->fd < 0) {
        fprintf(stderr, "rx: sidecar: socket failed: %s\n", strerror(errno));
        return -1;
    }
    /* Loopback only, like waybeam's sidecar is on the air unit: a
     * consumer on another host can still reach it through a forwarder. */
    struct sockaddr_in addr = {.sin_family = AF_INET, .sin_port = htons((uint16_t)s->port),
                               .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
    if (bind(s->fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "rx: sidecar: bind 127.0.0.1:%d failed: %s -- sidecar not served\n", s->port,
                strerror(errno));
        close(s->fd);
        s->fd = -1;
        return -1;
    }
    fprintf(stderr, "rx: serving the air unit's sidecar on 127.0.0.1:%d\n", s->port);
    return 0;
}

void sidecar_srv_close(sidecar_srv_t *s)
{
    if (s->fd >= 0)
        close(s->fd);
    s->fd = -1;
}

static void subscribe(sidecar_srv_t *s, const struct sockaddr_in *from, uint64_t now)
{
    pthread_mutex_lock(&s->mu);
    int slot = -1;
    for (int i = 0; i < SIDECAR_MAX_SUBS; i++) {
        if (s->subs[i].expires_us > now && s->subs[i].addr.sin_addr.s_addr == from->sin_addr.s_addr &&
            s->subs[i].addr.sin_port == from->sin_port) {
            slot = i;
            break;
        }
    }
    for (int i = 0; slot < 0 && i < SIDECAR_MAX_SUBS; i++)
        if (s->subs[i].expires_us <= now)
            slot = i;
    if (slot >= 0) {
        if (s->verbose && s->subs[slot].expires_us <= now)
            fprintf(stderr, "rx: sidecar subscriber %s:%u\n", inet_ntoa(from->sin_addr), ntohs(from->sin_port));
        s->subs[slot].addr = *from;
        s->subs[slot].expires_us = now + SIDECAR_SUB_TTL_US;
    }
    pthread_mutex_unlock(&s->mu);
}

void *sidecar_srv_thread_main(void *arg)
{
    sidecar_srv_t *s = arg;
    while (!*s->stop_flag) {
        struct pollfd pfd = {.fd = s->fd, .events = POLLIN};
        if (poll(&pfd, 1, SIDECAR_SRV_POLL_MS) <= 0)
            continue;
        uint8_t buf[64];
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        ssize_t n = recvfrom(s->fd, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
        uint64_t t2 = clock_sync_now_us();
        if (n < (ssize_t)sizeof(struct sidecar_hdr))
            continue;
        const struct sidecar_hdr *h = (const struct sidecar_hdr *)buf;
        if (ntohl(h->magic) != SIDECAR_MAGIC || h->version != SIDECAR_VERSION)
            continue;

        /* Both message types keep a subscription alive, as in waybeam. */
        subscribe(s, &from, t2);

        if (h->msg_type == SIDECAR_MSG_SYNC_REQ && n >= (ssize_t)sizeof(struct sidecar_sync_req)) {
            int64_t offset;
            uint32_t unc;
            if (!clock_sync_get(s->sync, &offset, &unc))
                continue; /* no air clock yet: let the probe retry rather than answer wrongly */
            const struct sidecar_sync_req *req = (const struct sidecar_sync_req *)buf;
            struct sidecar_sync_resp resp = {
                .hdr = {.magic = htonl(SIDECAR_MAGIC), .version = SIDECAR_VERSION,
                        .msg_type = SIDECAR_MSG_SYNC_RESP},
                .t1_us = req->t1_us, /* echoed as received, already network order */
                .t2_us = sidecar_be64((uint64_t)((int64_t)t2 + offset)),
            };
            resp.t3_us = sidecar_be64((uint64_t)((int64_t)clock_sync_now_us() + offset));
            sendto(s->fd, &resp, sizeof(resp), 0, (struct sockaddr *)&from, from_len);
        }
    }
    return NULL;
}

void sidecar_srv_publish(sidecar_srv_t *s, const uint8_t *dgram, uint32_t len)
{
    if (len >= sizeof(struct sidecar_frame)) {
        struct sidecar_frame f;
        memcpy(&f, dgram, sizeof(f));
        uint64_t capture = sidecar_be64(f.capture_us);
        uint64_t ready = sidecar_be64(f.frame_ready_us);
        /* capture is CLOCK_MONOTONIC and ready CLOCK_MONOTONIC_RAW; on an
         * air unit without NTP they tick together, and anything outside a
         * plausible encode time is ignored rather than trusted. */
        if (capture && ready > capture && ready - capture < 200000) {
            uint32_t enc = (uint32_t)(ready - capture);
            uint32_t avg = __atomic_load_n(&s->encode_avg_us, __ATOMIC_RELAXED);
            avg = avg ? (avg * 7 + enc) / 8 : enc;
            __atomic_store_n(&s->encode_avg_us, avg, __ATOMIC_RELAXED);
        }
    }
    s->published++;
    if (s->fd < 0)
        return;

    uint64_t now = clock_sync_now_us();
    pthread_mutex_lock(&s->mu);
    for (int i = 0; i < SIDECAR_MAX_SUBS; i++)
        if (s->subs[i].expires_us > now)
            sendto(s->fd, dgram, len, MSG_DONTWAIT, (struct sockaddr *)&s->subs[i].addr, sizeof(s->subs[i].addr));
    pthread_mutex_unlock(&s->mu);
}
