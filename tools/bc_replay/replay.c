/*
 * bc_replay -- replays a flight trace (extract_trace.py) through the real
 * tx/bitrate_ctl.c in virtual time and reports how the video would have
 * fared.
 *
 *   replay <trace.csv> <rules|pi> [key=value ...]
 *
 * bb_ioctl, http_get_status, clock_gettime and usleep are mocked: every
 * usleep() of the controller advances a virtual clock and steps the plant
 * model, so a 15-minute flight replays in well under a second,
 * single-threaded and deterministic. key=value overrides PI/rules
 * parameters (see apply_override()) for tuning sweeps.
 *
 * Plant model (constants measured on the Ascent air, 2026-10-02/03):
 *  - encoder: 100 fps, output rate follows the set rate with a 300 ms
 *    first-order lag, a keyframe every 100 frames (gopSize 1.0 s) at 4x
 *    the average frame, the other frames scaled so the mean stays at the
 *    rate; waybeam refuses rates below 1000 kbps (409);
 *  - encode latency 12 ms (healthy "pts age" on the bench);
 *  - frame-shm ring of 8 slots between encoder and transmitter, a frame
 *    arriving at a full ring is dropped (waybeam full_drops);
 *  - transmitter drains frame by frame at 0.59 x link capacity (21.5 of
 *    36.7 Mbit/s measured at MCS 12), nothing while disconnected;
 *  - the delay the controller sees is pts -> last byte written (EWMA 1/8),
 *    exactly what tx/main.c publishes;
 *  - radio distress from the trace: ground SNR / LDPC drive the LDPC ratio
 *    the air would read and BB_EVENT_RETX_TOO_MANY events.
 */
#include "bitrate_ctl.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ------------------------------------------------------------ trace */
typedef struct {
    double t;
    int conn;
    double cap;
    double snr;
    int ldpc;
} sample_t;
static sample_t *tr;
static int tr_n;

static void load_trace(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        perror(path);
        exit(1);
    }
    char line[256];
    int cap = 0;
    if (!fgets(line, sizeof(line), f)) /* header */
        exit(1);
    while (fgets(line, sizeof(line), f)) {
        if (tr_n == cap) {
            cap = cap ? cap * 2 : 1024;
            tr = realloc(tr, cap * sizeof(*tr));
        }
        sample_t *s = &tr[tr_n];
        if (sscanf(line, "%lf,%d,%lf,%lf,%d", &s->t, &s->conn, &s->cap, &s->snr, &s->ldpc) == 5)
            tr_n++;
    }
    fclose(f);
}

static int tr_idx;
static const sample_t *trace_at(double t)
{
    while (tr_idx + 1 < tr_n && tr[tr_idx + 1].t <= t)
        tr_idx++;
    return &tr[tr_idx];
}

/* ------------------------------------------------------- virtual time */
static uint64_t vt_us; /* virtual CLOCK_MONOTONIC */

int clock_gettime(clockid_t id, struct timespec *ts)
{
    (void)id;
    ts->tv_sec = (time_t)(vt_us / 1000000u);
    ts->tv_nsec = (long)(vt_us % 1000000u) * 1000;
    return 0;
}

/* ------------------------------------------------------------- plant */
#define FPS          100
#define GOP          100
#define ENC_LAG_S    0.3
#define ENC_LAT_MS   20.0
#define RING_SLOTS   8
#define WB_FLOOR     1000
/* calibrated against the recorded flights, overridable via env for that */
static double IDR_FACTOR = 4.0;
static double DRAIN_EFF = 0.72; /* calibrated: rules replay vs recorded 03.10 flights */

static bitrate_ctl_cfg_t cfg;
static volatile int g_stop;
static venc_frame_ring_hdr_t ring_hdr;
static venc_frame_ring_t ring = {.hdr = &ring_hdr};

static double set_kbps = 8000;   /* what waybeam was told */
static double enc_kbps = 8000;   /* what the encoder currently produces */
static double next_frame_us;
static unsigned frame_no;
typedef struct {
    double bits;
    double pts_us;
} frame_t;
static frame_t q[RING_SLOTS];
static int q_head, q_n;
static int inflight;
static frame_t cur;
static int cur_behind;      /* frames waiting in the ring when cur was taken */
static uint64_t cur_take_us;
static double cur_left;
static double delay_ewma_us = 0;
static int low_water = RING_SLOTS;
static uint64_t lw_window_start;
static double next_retx_us;
static double tl_from = -1, tl_to = -1; /* TIMELINE=from,to (trace seconds) */

/* bb_ioctl event callbacks */
static void (*retx_cb)(void *, void *);
static void (*mcs_cb)(void *, void *);
static double last_cap;

/* stats */
static uint64_t n_frames, n_drop, n_late50, n_late100;
static double max_delay_ms, sum_set_good, n_set_good, sum_set, n_set, sum_sent;
static double sec_start_sum, sec_start_n, sec_ring_max;
static uint64_t sec_start_us;
static double *sec_avgs;
static size_t n_sec;
static uint64_t n_sec_ring50;
static double *delays;
static size_t n_delays, cap_delays;

static void record_delay(double ms)
{
    if (n_delays == cap_delays) {
        cap_delays = cap_delays ? cap_delays * 2 : 65536;
        delays = realloc(delays, cap_delays * sizeof(double));
    }
    delays[n_delays++] = ms;
    if (ms > 50)
        n_late50++;
    if (ms > 100)
        n_late100++;
    if (ms > max_delay_ms)
        max_delay_ms = ms;
}

static void plant_step_1ms(void)
{
    double t_s = vt_us / 1e6;
    const sample_t *s = trace_at(t_s);
    double cap = s->conn ? s->cap : 0;
    if (cap != last_cap && mcs_cb)
        mcs_cb(NULL, NULL);
    last_cap = cap;

    /* encoder */
    enc_kbps += (set_kbps - enc_kbps) * (0.001 / ENC_LAG_S);
    if (vt_us >= next_frame_us) {
        next_frame_us += 1e6 / FPS;
        double avg = enc_kbps * 1000.0 / FPS;
        double bits = (frame_no % GOP == 0) ? avg * IDR_FACTOR : avg * (GOP - IDR_FACTOR) / (GOP - 1);
        frame_no++;
        if (s->conn) {
            n_frames++;
            if (q_n == RING_SLOTS) {
                n_drop++;
            } else {
                q[(q_head + q_n) % RING_SLOTS] = (frame_t){bits, (double)vt_us - ENC_LAT_MS * 1000};
                q_n++;
            }
        }
    }

    /* link down: the air resets the ring and the transmitter (a new
     * session starts on reconnect), nothing queued survives */
    if (!s->conn) {
        q_n = 0;
        inflight = 0;
    }

    /* transmitter */
    double drain_bits = cap * 1000.0 * DRAIN_EFF / 1000.0; /* kbps -> bits per ms */
    while (drain_bits > 0) {
        if (!inflight) {
            if (!q_n)
                break;
            cur = q[q_head];
            q_head = (q_head + 1) % RING_SLOTS;
            q_n--;
            cur_left = cur.bits;
            inflight = 1;
            cur_behind = q_n;
            cur_take_us = vt_us;
            sec_start_sum += ((double)vt_us - cur.pts_us) / 1000.0;
            sec_start_n++;
        }
        double take = cur_left < drain_bits ? cur_left : drain_bits;
        cur_left -= take;
        drain_bits -= take;
        sum_sent += take;
        if (cur_left <= 0) {
            inflight = 0;
            double d_us = (double)vt_us - cur.pts_us;
            record_delay(d_us / 1000.0);
            /* what tx/main.c publishes: frames behind x 10 ms + write time */
            double q_us = cur_behind * (1e6 / FPS) + (double)(vt_us - cur_take_us);
            delay_ewma_us += (q_us - delay_ewma_us) / 8.0;
            __atomic_store_n(&cfg.delay_us, (uint32_t)delay_ewma_us, __ATOMIC_RELEASE);
            __atomic_store_n(&cfg.last_tx_done_ms, vt_us / 1000, __ATOMIC_RELEASE);
        }
    }

    /* ring low water, published per 200 ms window like waybeam */
    if (q_n < low_water)
        low_water = q_n;
    if (vt_us - lw_window_start >= 200000) {
        __atomic_store_n(&ring_hdr.low_water_slots, (uint16_t)low_water, __ATOMIC_RELAXED);
        low_water = q_n;
        lw_window_start = vt_us;
    }

    /* radio distress -> RETX_TOO_MANY events */
    if (s->conn && retx_cb && vt_us >= next_retx_us) {
        double period = 0;
        if (s->snr < 6 || s->ldpc >= 300)
            period = 0.3;
        else if (s->snr < 10 || s->ldpc > 0)
            period = 1.0;
        if (period > 0) {
            retx_cb(NULL, NULL);
            next_retx_us = vt_us + period * 1e6;
        }
    }

    /* per-second figures comparable with the recorded "tx stats" */
    if (vt_us - sec_start_us >= 1000000) {
        sec_ring_max = q_n; /* instantaneous at the boundary, like the "tx stats" print */
        if (s->conn && sec_start_n) {
            sec_avgs = realloc(sec_avgs, (n_sec + 1) * sizeof(double));
            sec_avgs[n_sec++] = sec_start_sum / sec_start_n;
            if (sec_ring_max >= RING_SLOTS / 2)
                n_sec_ring50++;
        }
        sec_start_sum = sec_start_n = sec_ring_max = 0;
        sec_start_us = vt_us;
    }

    if (tl_from >= 0 && t_s >= tl_from && t_s <= tl_to && vt_us % 250000 == 0)
        printf("tl %7.2f cap %6.0f set %6.0f enc %6.0f delay %6.1f ring %d\n", t_s, cap, set_kbps, enc_kbps,
               delay_ewma_us / 1000.0, q_n);

    /* bookkeeping */
    if (s->conn) {
        sum_set += set_kbps;
        n_set++;
        if (cap >= 18288) {
            sum_set_good += set_kbps;
            n_set_good++;
        }
    }
}

int usleep(useconds_t us)
{
    for (useconds_t i = 0; i < us / 1000; i++) {
        vt_us += 1000;
        plant_step_1ms();
    }
    if (vt_us / 1e6 > tr[tr_n - 1].t)
        g_stop = 1;
    return 0;
}

/* --------------------------------------------------------- SDK mocks */
int bb_ioctl(bb_dev_handle_t *dev, uint32_t req, const void *in, void *out)
{
    (void)dev;
    const sample_t *s = trace_at(vt_us / 1e6);
    if (req == BB_GET_MCS) {
        if (!s->conn)
            return -1;
        ((bb_get_mcs_out_t *)out)->throughput = (uint32_t)s->cap;
        return 0;
    }
    if (req == BB_GET_USER_QUALITY) {
        bb_get_user_quality_out_t *o = out;
        double r = (s->snr < 5 || s->ldpc >= 300) ? 1.0 : (s->snr < 9 || s->ldpc > 0) ? 0.3 : 0.0;
        o->qualities[0].ldpc_num = 1000;
        o->qualities[0].ldpc_err = (uint16_t)(r * 1000);
        return s->conn ? 0 : -1;
    }
    if (req == BB_SET_EVENT_SUBSCRIBE) {
        const bb_set_event_callback_t *sub = in;
        if (sub->event == BB_EVENT_RETX_TOO_MANY)
            retx_cb = (void (*)(void *, void *))sub->callback;
        if (sub->event == BB_EVENT_MCS_CHANGE)
            mcs_cb = (void (*)(void *, void *))sub->callback;
        return 0;
    }
    return 0;
}

int http_get_status(const char *host, int port, const char *path, int timeout_ms)
{
    (void)host;
    (void)port;
    (void)timeout_ms;
    const char *b = strstr(path, "bitrate=");
    if (!b)
        return 200;
    unsigned v = (unsigned)atoi(b + 8);
    if (v < WB_FLOOR)
        return 409;
    set_kbps = v;
    return 200;
}

/* ------------------------------------------------------------ config */
static void defaults(int mode)
{
    static ar8030_link_t link = {.dev = (bb_dev_handle_t *)1, .sockfd = -1};
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = mode;
    cfg.link = &link;
    cfg.waybeam_host = "127.0.0.1";
    cfg.waybeam_port = 80;
    /* tx/main.c defaults */
    cfg.margin = 0.545;
    cfg.min_kbps = 512;
    cfg.max_kbps = 35000;
    cfg.hysteresis = 0.05;
    cfg.min_interval_ms = 500;
    cfg.poll_interval_ms = 500;
    cfg.ring = &ring;
    cfg.ring_backlog_high_slots = 6;
    cfg.ring_backoff = 0.92;
    cfg.ramp_step = 0.25;
    cfg.ramp_settle_ms = 1000;
    cfg.unstable_window_ms = 3000;
    cfg.unstable_ramp_step = 0.10;
    cfg.probe_ceiling_frac = 0.95;
    cfg.probe_hold_ms = 15000;
    cfg.roi_max_kbps = 0;
    cfg.roi_recovery_ms = 5000;
    cfg.ldpc_ratio_high = 0; /* = tx/main.c: off, the air reads its uplink */
    cfg.ldpc_backoff = 0.85;
    cfg.ldpc_severe_ratio = 0;
    cfg.retx_severe_events = 3;
    cfg.severe_backoff = 0.4;
    cfg.retx_event_backoff = 0.85;
    /* = tx/main.c's PI defaults */
    cfg.pi_delay_set_ms = 10;
    cfg.pi_kp_up = 0.006;
    cfg.pi_kp_down = 0.02;
    cfg.pi_ki_up = 0.01;
    cfg.pi_ki_down = 0.05;
    cfg.pi_k_min = 0.2;
    cfg.pi_k_max = 1.1;
    cfg.pi_slew_up = 2.0;
    cfg.pi_severe_k = 0.3;
    cfg.pi_mild_trim = 0.85;
    cfg.stop_flag = &g_stop;
}

static void apply_override(const char *kv)
{
    char key[64];
    double v;
    if (sscanf(kv, "%63[^=]=%lf", key, &v) != 2) {
        fprintf(stderr, "bad override %s\n", kv);
        exit(1);
    }
#define D(name) else if (!strcmp(key, #name)) cfg.name = v
#define I(name) else if (!strcmp(key, #name)) cfg.name = (int)v
    if (0) {
    }
    D(pi_delay_set_ms); D(pi_kp_up); D(pi_kp_down); D(pi_ki_up); D(pi_ki_down); D(pi_k_min); D(pi_k_max);
    D(pi_slew_up); D(pi_severe_k); D(pi_mild_trim); D(margin); D(ramp_step); D(severe_backoff);
    D(ldpc_ratio_high); D(ldpc_severe_ratio); I(retx_severe_events);
    I(max_kbps); I(unstable_window_ms); I(ramp_settle_ms);
    else {
        fprintf(stderr, "unknown key %s\n", key);
        exit(1);
    }
}

static int cmp_d(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return x < y ? -1 : x > y;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: replay <trace.csv> <rules|pi> [key=value ...]\n");
        return 1;
    }
    load_trace(argv[1]);
    defaults(!strcmp(argv[2], "pi") ? BITRATE_CTL_MODE_PI : BITRATE_CTL_MODE_RULES);
    for (int i = 3; i < argc; i++)
        apply_override(argv[i]);
    if (getenv("TIMELINE"))
        sscanf(getenv("TIMELINE"), "%lf,%lf", &tl_from, &tl_to);
    if (getenv("IDR_FACTOR"))
        IDR_FACTOR = atof(getenv("IDR_FACTOR"));
    if (getenv("DRAIN_EFF"))
        DRAIN_EFF = atof(getenv("DRAIN_EFF"));
    if (!getenv("REPLAY_LOG") && !freopen("/dev/null", "w", stderr))
        return 1;

    vt_us = 1000000;
    next_frame_us = vt_us;
    bitrate_ctl_run(&cfg);

    qsort(delays, n_delays, sizeof(double), cmp_d);
    qsort(sec_avgs, n_sec, sizeof(double), cmp_d);
    if (getenv("CALIB"))
        printf("calib %-6s write-start avg/s p50 %5.1f p90 %5.1f max %6.1f ms  ring>=50%% in %.1f%% of s  dropped %.2f%%\n",
               argv[2], n_sec ? sec_avgs[n_sec / 2] : 0, n_sec ? sec_avgs[n_sec * 9 / 10] : 0,
               n_sec ? sec_avgs[n_sec - 1] : 0, n_sec ? 100.0 * n_sec_ring50 / n_sec : 0,
               n_frames ? 100.0 * n_drop / n_frames : 0);
    double p50 = n_delays ? delays[n_delays / 2] : 0, p99 = n_delays ? delays[n_delays * 99 / 100] : 0;
    printf("%-6s frames %6llu  dropped %5llu (%.2f%%)  delay p50 %5.1f p99 %6.1f max %7.1f ms  "
           ">50ms %5.2f%% >100ms %5.2f%%  mean set %6.0f kbps (good link %6.0f)  sent %5.2f Mbit/s\n",
           argv[2], (unsigned long long)n_frames, (unsigned long long)n_drop,
           n_frames ? 100.0 * n_drop / n_frames : 0, p50, p99, max_delay_ms,
           n_delays ? 100.0 * n_late50 / n_delays : 0, n_delays ? 100.0 * n_late100 / n_delays : 0,
           n_set ? sum_set / n_set : 0, n_set_good ? sum_set_good / n_set_good : 0,
           n_set ? sum_sent / (n_set / 1000.0) / 1e6 : 0);
    return 0;
}
