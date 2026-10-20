#ifndef AR8030_RETX_EVENT_H
#define AR8030_RETX_EVENT_H

/*
 * BB_EVENT_RETX_TOO_MANY payload -- header-only, shared by tx/
 * (bitrate_ctl) and linkctl/ (`retx-watch`).
 *
 *  - The chip evaluates the BB_SET_RETX_EVENT_STATUS window once per
 *    super-frame (~5.6 ms), on the AP only, in 1V1 only, and only once
 *    win != 0 (lifecycled sets it on every connect).
 *  - It raises the event on every *edge*: busy on, busy off, conti on,
 *    conti off. A recovery raises the same event as a fade.
 *  - The payload is 136 bytes. ar8030d (daemon/callback_node.c) and the
 *    client library (app/ar8030/session_callback.c) pass them through
 *    unchanged as the callback's `arg`.
 *
 * Confirmed on the Ascent air (2026-10-20, linkctl retx-watch): win..
 * conti_idle read back lifecycled's 10/6/4/2/0, and each short burst is a
 * rise (two retx bits, conti_on=1) and a fall one super-frame later.
 *
 * Only bytes 0..12 mean anything; the rest is not retx state. Stock's
 * ar_ldyhs_sky acts only on conti_on == 1 (its payload[6] check) and
 * latches busy_on (payload[5]) separately.
 */

#include <stdint.h>
#include <string.h>

#define AR8030_RETX_EVENT_LEN 0x88

typedef struct {
    uint32_t bits;       /* retx bitmap of the last `win` super-frames */
    uint8_t  pos;        /* window position of the next frame */
    uint8_t  busy_on;    /* >= busy of the last win frames were retx */
    uint8_t  conti_on;   /* the last conti_busy frames were all retx */
    uint8_t  win, busy, idle, conti_busy, conti_idle; /* as set by BB_SET_RETX_EVENT_STATUS */
} ar8030_retx_state_t;

/* Decodes the event's `arg`. Returns 0, or -1 when arg is NULL or does
 * not look like the firmware's retx record (a flag other than 0/1, win 0,
 * pos outside the window); then treat the firing as plain "retx pressure". */
static inline int ar8030_retx_event_decode(const void* arg, ar8030_retx_state_t* st)
{
    const uint8_t* p = (const uint8_t*)arg;

    if (!p)
        return -1;
    memcpy(&st->bits, p, sizeof(st->bits));
    st->pos        = p[4];
    st->busy_on    = p[5];
    st->conti_on   = p[6];
    st->win        = p[8];
    st->busy       = p[9];
    st->idle       = p[10];
    st->conti_busy = p[11];
    st->conti_idle = p[12];
    if (st->busy_on > 1 || st->conti_on > 1 || st->win == 0 || st->pos >= st->win)
        return -1;
    return 0;
}

/* 1 when busy_on or conti_on went 0 -> 1 between two events (pressure
 * rose), 0 for a falling edge. Every flag change raises an event and one
 * event never mixes rising and falling edges, so an event that shows no
 * change means *prev is stale (chip reset in between, or the first
 * event); then any flag that is set counts. */
static inline int ar8030_retx_event_rising(const ar8030_retx_state_t* prev, const ar8030_retx_state_t* cur)
{
    if (cur->busy_on == prev->busy_on && cur->conti_on == prev->conti_on)
        return cur->busy_on || cur->conti_on;
    return (cur->busy_on && !prev->busy_on) || (cur->conti_on && !prev->conti_on);
}

#endif
