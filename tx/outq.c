#include "outq.h"

#include <string.h>

void tx_outq_init(tx_outq_t *q)
{
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->mu, NULL);
}

void tx_outq_push(tx_outq_t *q, const tx_outq_item_t *item)
{
    pthread_mutex_lock(&q->mu);
    if (q->count == TX_OUTQ_DEPTH) {
        q->head = (q->head + 1) % TX_OUTQ_DEPTH;
        q->count--;
        q->dropped++;
    }
    q->items[(q->head + q->count) % TX_OUTQ_DEPTH] = *item;
    q->count++;
    pthread_mutex_unlock(&q->mu);
}

int tx_outq_pop(tx_outq_t *q, tx_outq_item_t *out)
{
    int got = 0;
    pthread_mutex_lock(&q->mu);
    if (q->count) {
        *out = q->items[q->head];
        q->head = (q->head + 1) % TX_OUTQ_DEPTH;
        q->count--;
        got = 1;
    }
    pthread_mutex_unlock(&q->mu);
    return got;
}
