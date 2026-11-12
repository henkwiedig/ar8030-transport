#ifndef AR8030_TRANSPORT_OUTQ_H
#define AR8030_TRANSPORT_OUTQ_H

/*
 * Small air -> ground chunks that are not video: SYNCR clock-sync replies
 * (tx/idr_ctrl.c), tunnelled waybeam sidecar datagrams (tx/sidecar_sub.c)
 * and waybeam's RTP/Opus audio (tx/audio_sub.c).
 *
 * Those come from other threads, but the video socket is stream mode: a
 * chunk written from a second thread could land in the middle of a video
 * frame's chunks and corrupt both. So producers only queue here, and the
 * main send loop drains the queue between two frames (tx/main.c), which
 * keeps every write on one thread.
 *
 * Full queue: the oldest entry is dropped. A stale sidecar datagram or
 * sync reply is worth less than a fresh one, and neither is ever retried.
 */

#include <pthread.h>
#include <stdint.h>

#include "ar8030_chunk.h"

/* 32: room for the audio (~50 packets/s) next to the sidecar (one per frame)
 * and the SYNCR replies between two video chunks. */
#define TX_OUTQ_DEPTH 32

typedef struct {
    uint8_t codec; /* AR8030_CHUNK_CODEC_CTRL, _SIDECAR or _OPUS */
    /* SYNCR replies are formatted at send time so t3 is the real write
     * time; everything else is sent from data/len as is. */
    int is_syncr;
    uint32_t sync_seq;
    uint64_t t1_us;
    uint64_t t2_us;
    uint16_t len;
    uint8_t data[AR8030_SIDECAR_MAX_PAYLOAD];
} tx_outq_item_t;

typedef struct {
    pthread_mutex_t mu;
    tx_outq_item_t items[TX_OUTQ_DEPTH];
    unsigned head;
    unsigned count;
    uint64_t dropped;
} tx_outq_t;

void tx_outq_init(tx_outq_t *q);
/* Copies *item in; drops the oldest entry if the queue is full. */
void tx_outq_push(tx_outq_t *q, const tx_outq_item_t *item);
/* Returns 1 and fills *out if an entry was queued, 0 if empty. */
int tx_outq_pop(tx_outq_t *q, tx_outq_item_t *out);

#endif
