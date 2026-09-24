#include "idr_ctrl.h"

#include "ar8030_chunk.h"
#include "chunk_stream.h"
#include "http_get.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define IDR_CTRL_READ_TIMEOUT_MS 200
#define IDR_CTRL_TOKENS          16 /* recent tokens remembered for dedup */

static long long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* ar8030_chunk_read_fn on the video socket's reverse direction. The fd is
 * reloaded per call: tx's main loop reopens the socket after a daemon
 * reconnect (see ar8030_link.h); a read on a closed/stale fd just fails
 * and the stream reader retries. */
static int read_reverse(void *ctx, uint8_t *buf, uint32_t cap, int timeout_ms)
{
    ar8030_link_t *link = ctx;
    int sockfd = __atomic_load_n(&link->sockfd, __ATOMIC_ACQUIRE);
    if (sockfd < 0) {
        usleep((useconds_t)timeout_ms * 1000);
        return 0;
    }
    int n = bb_socket_read(sockfd, buf, cap, timeout_ms);
    if (n < 0)
        usleep((useconds_t)timeout_ms * 1000); /* don't spin on a dead fd */
    return n;
}

struct seen_token {
    char token[AR8030_CTRL_MAX_PAYLOAD];
    long long at_ms;
};

static void handle_idr(idr_ctrl_cfg_t *cfg, const char *token, struct seen_token *seen, long long *last_honored_ms)
{
    long long now = now_ms();
    for (int i = 0; i < IDR_CTRL_TOKENS; i++) {
        if (seen[i].at_ms && now - seen[i].at_ms < cfg->dedup_ms && strcmp(seen[i].token, token) == 0) {
            if (cfg->verbose)
                fprintf(stderr, "tx: idr request token=%s: duplicate, ignored\n", token);
            return;
        }
    }
    int oldest = 0;
    for (int i = 1; i < IDR_CTRL_TOKENS; i++)
        if (seen[i].at_ms < seen[oldest].at_ms)
            oldest = i;
    snprintf(seen[oldest].token, sizeof(seen[oldest].token), "%s", token);
    seen[oldest].at_ms = now;

    if (*last_honored_ms && now - *last_honored_ms < cfg->coalesce_ms) {
        if (cfg->verbose)
            fprintf(stderr, "tx: idr request token=%s: coalesced (%lld ms after the last)\n", token,
                    now - *last_honored_ms);
        return;
    }
    *last_honored_ms = now;
    int status = http_get_status(cfg->waybeam_host, cfg->waybeam_port, "/request/idr", 1000);
    if (cfg->verbose || status != 200)
        fprintf(stderr, "tx: idr request token=%s -> waybeam /request/idr status=%d\n", token, status);
}

void *idr_ctrl_thread_main(void *arg)
{
    idr_ctrl_cfg_t *cfg = arg;

    /* chunk_stream needs room for the largest chunk a header may claim,
     * even though control chunks are tiny -- see its init() comment. */
    uint32_t stream_cap = AR8030_CHUNK_HDR_SIZE + AR8030_CHUNK_MAX_PAYLOAD;
    uint8_t *stream_buf = malloc(stream_cap);
    uint8_t *payload = malloc(AR8030_CHUNK_MAX_PAYLOAD);
    if (!stream_buf || !payload) {
        fprintf(stderr, "tx: idr ctrl: OOM, keyframe requests disabled\n");
        free(stream_buf);
        free(payload);
        return NULL;
    }

    ar8030_chunk_stream_t stream;
    ar8030_chunk_stream_init(&stream, read_reverse, cfg->link, stream_buf, stream_cap, IDR_CTRL_READ_TIMEOUT_MS,
                             cfg->stop_flag);

    struct seen_token seen[IDR_CTRL_TOKENS];
    memset(seen, 0, sizeof(seen));
    long long last_honored_ms = 0;

    while (!*cfg->stop_flag) {
        struct ar8030_chunk_hdr hdr;
        uint32_t len = 0;
        if (ar8030_chunk_stream_read(&stream, &hdr, payload, AR8030_CHUNK_MAX_PAYLOAD, &len) <= 0)
            continue;
        if (hdr.codec != AR8030_CHUNK_CODEC_CTRL || len == 0 || len >= AR8030_CTRL_MAX_PAYLOAD)
            continue;
        char cmd[AR8030_CTRL_MAX_PAYLOAD];
        memcpy(cmd, payload, len);
        cmd[len] = '\0';
        if (strncmp(cmd, "IDR ", 4) == 0 && cmd[4])
            handle_idr(cfg, cmd + 4, seen, &last_honored_ms);
        else if (cfg->verbose)
            fprintf(stderr, "tx: unknown control message \"%s\", ignored\n", cmd);
    }

    free(stream_buf);
    free(payload);
    return NULL;
}
