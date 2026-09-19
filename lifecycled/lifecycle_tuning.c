#include "lifecycle_tuning.h"
#include "lc_log.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Same MHz<->bb_bandwidth_e mapping ar8030-transport's own ar8030-linkctl
 * uses (its parse_bandwidth()/bandwidth_name()) -- kept as one table here
 * since this file only ever needs the round trip, not string parsing. */
static const int BW_MHZ_BY_ENUM[BB_BW_MAX] = {
    [BB_BW_1_25M] = 1, /* rounded down from 1.25 -- there is no fractional
                        * MHz CLI value elsewhere in this project either
                        * (--default-bandwidth takes a plain int). */
    [BB_BW_2_5M]  = 2,
    [BB_BW_5M]    = 5,
    [BB_BW_10M]   = 10,
    [BB_BW_20M]   = 20,
    [BB_BW_40M]   = 40,
};

static int mhz_to_bw_enum(int mhz)
{
    for (int e = 0; e < BB_BW_MAX; e++) {
        if (BW_MHZ_BY_ENUM[e] == mhz) {
            return e;
        }
    }
    return -1;
}

int lc_tuning_valid_mhz(int mhz)
{
    return mhz_to_bw_enum(mhz) >= 0;
}

static void tuning_sidecar_path(const char* cfg_path, char* out, size_t out_sz)
{
    const char* slash = strrchr(cfg_path, '/');
    if (slash) {
        size_t dirlen = (size_t)(slash - cfg_path) + 1;
        if (dirlen >= out_sz) {
            dirlen = out_sz - 1;
        }
        memcpy(out, cfg_path, dirlen);
        snprintf(out + dirlen, out_sz - dirlen, "ar8030.tuning");
    } else {
        snprintf(out, out_sz, "ar8030.tuning");
    }
}

int lc_tuning_load(const char* cfg_path)
{
    char path[512];
    tuning_sidecar_path(cfg_path, path, sizeof(path));

    FILE* f = fopen(path, "r");
    if (!f) {
        return -1;
    }
    int mhz = -1;
    int ok  = fscanf(f, "%d", &mhz) == 1;
    fclose(f);
    if (!ok || mhz_to_bw_enum(mhz) < 0) {
        lc_log("lifecycle: tuning: %s has no valid bandwidth, ignoring", path);
        return -1;
    }
    return mhz;
}

int lc_tuning_save(const char* cfg_path, int bandwidth_mhz)
{
    if (mhz_to_bw_enum(bandwidth_mhz) < 0) {
        lc_log("lifecycle: tuning: refusing to persist invalid bandwidth %d", bandwidth_mhz);
        return -1;
    }

    char path[512];
    tuning_sidecar_path(cfg_path, path, sizeof(path));
    char tmp_path[520];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);

    FILE* f = fopen(tmp_path, "w");
    if (!f) {
        lc_log("lifecycle: tuning: can't open %s for writing: %s", tmp_path, strerror(errno));
        return -1;
    }
    fprintf(f, "%d\n", bandwidth_mhz);
    fclose(f);

    if (rename(tmp_path, path) != 0) {
        lc_log("lifecycle: tuning: rename %s -> %s failed: %s", tmp_path, path, strerror(errno));
        return -1;
    }
    return 0;
}

int lc_tuning_read_current(bb_dev_handle_t* handle)
{
    bb_get_status_in_t  in = {0xffff};
    bb_get_status_out_t out;
    memset(&out, 0, sizeof(out));

    if (bb_ioctl(handle, BB_GET_STATUS, &in, &out) != 0) {
        return -1;
    }
    /* This project only ever runs single-user mode -- see
     * lifecycle_pair.c's own slot-scan comment for the equivalent
     * reasoning on the slot side; user 0 is always the real data user. */
    int bw = out.user_status[0].tx_status.bandwidth;
    if (bw < 0 || bw >= BB_BW_MAX) {
        return -1;
    }
    return BW_MHZ_BY_ENUM[bw];
}

int lc_tuning_apply(bb_dev_handle_t* handle, int slot, int bandwidth_mhz)
{
    int bw_enum = mhz_to_bw_enum(bandwidth_mhz);
    if (bw_enum < 0) {
        lc_log("lifecycle: tuning: %d MHz is not a valid AR8030 bandwidth, not applying", bandwidth_mhz);
        return -1;
    }
    bb_set_bandwidth_t sb = {
        .slot      = (uint8_t)slot,
        .dir       = BB_DIR_TX,
        .bandwidth = (uint8_t)bw_enum,
    };
    int ret = bb_ioctl(handle, BB_SET_BANDWIDTH, &sb, NULL);
    lc_log("lifecycle: tuning: BB_SET_BANDWIDTH(slot=%d, tx, %dMHz) ret=%d", slot, bandwidth_mhz, ret);
    return ret;
}

int lc_tuning_resolve_connected_slot(bb_dev_handle_t* handle)
{
    bb_get_status_in_t  in = {0xffff};
    bb_get_status_out_t out;
    memset(&out, 0, sizeof(out));

    if (bb_ioctl(handle, BB_GET_STATUS, &in, &out) != 0) {
        return -1;
    }
    for (int s = 0; s < BB_SLOT_MAX; s++) {
        if (out.link_status[s].state == BB_LINK_STATE_CONNECT) {
            return s;
        }
    }
    return -1;
}

int lc_retx_valid(int win, int busy, int idle, int conti_busy, int conti_idle)
{
    return win >= 0 && win <= 255 && busy >= 0 && busy <= 255 && idle >= 0 && idle <= 255 && conti_busy >= 0 &&
           conti_busy <= 255 && conti_idle >= 0 && conti_idle <= 255;
}

static void retx_sidecar_path(const char* cfg_path, char* out, size_t out_sz)
{
    const char* slash = strrchr(cfg_path, '/');
    if (slash) {
        size_t dirlen = (size_t)(slash - cfg_path) + 1;
        if (dirlen >= out_sz) {
            dirlen = out_sz - 1;
        }
        memcpy(out, cfg_path, dirlen);
        snprintf(out + dirlen, out_sz - dirlen, "ar8030.retx");
    } else {
        snprintf(out, out_sz, "ar8030.retx");
    }
}

int lc_retx_load(const char* cfg_path, bb_retx_cfg_t* out)
{
    char path[512];
    retx_sidecar_path(cfg_path, path, sizeof(path));

    FILE* f = fopen(path, "r");
    if (!f) {
        return -1;
    }
    int win, busy, idle, conti_busy, conti_idle;
    int ok = fscanf(f, "%d %d %d %d %d", &win, &busy, &idle, &conti_busy, &conti_idle) == 5;
    fclose(f);
    if (!ok || !lc_retx_valid(win, busy, idle, conti_busy, conti_idle)) {
        lc_log("lifecycle: tuning: %s has no valid retx config, ignoring", path);
        return -1;
    }
    memset(out, 0, sizeof(*out));
    out->win         = (uint8_t)win;
    out->busy        = (uint8_t)busy;
    out->idle        = (uint8_t)idle;
    out->conti_busy  = (uint8_t)conti_busy;
    out->conti_idle  = (uint8_t)conti_idle;
    return 0;
}

int lc_retx_save(const char* cfg_path, int win, int busy, int idle, int conti_busy, int conti_idle)
{
    if (!lc_retx_valid(win, busy, idle, conti_busy, conti_idle)) {
        lc_log("lifecycle: tuning: refusing to persist invalid retx config");
        return -1;
    }

    char path[512];
    retx_sidecar_path(cfg_path, path, sizeof(path));
    char tmp_path[520];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);

    FILE* f = fopen(tmp_path, "w");
    if (!f) {
        lc_log("lifecycle: tuning: can't open %s for writing: %s", tmp_path, strerror(errno));
        return -1;
    }
    fprintf(f, "%d %d %d %d %d\n", win, busy, idle, conti_busy, conti_idle);
    fclose(f);

    if (rename(tmp_path, path) != 0) {
        lc_log("lifecycle: tuning: rename %s -> %s failed: %s", tmp_path, path, strerror(errno));
        return -1;
    }
    return 0;
}

int lc_retx_apply(bb_dev_handle_t* handle, int win, int busy, int idle, int conti_busy, int conti_idle)
{
    if (!lc_retx_valid(win, busy, idle, conti_busy, conti_idle)) {
        lc_log("lifecycle: tuning: retx config out of range, not applying");
        return -1;
    }
    bb_retx_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.win         = (uint8_t)win;
    cfg.busy        = (uint8_t)busy;
    cfg.idle        = (uint8_t)idle;
    cfg.conti_busy  = (uint8_t)conti_busy;
    cfg.conti_idle  = (uint8_t)conti_idle;
    int ret = bb_ioctl(handle, BB_SET_RETX_EVENT_STATUS, &cfg, NULL);
    lc_log("lifecycle: tuning: BB_SET_RETX_EVENT_STATUS(win=%d,busy=%d,idle=%d,conti_busy=%d,conti_idle=%d) ret=%d",
           win, busy, idle, conti_busy, conti_idle, ret);
    return ret;
}

int lc_frame_change_apply(bb_dev_handle_t* handle, int mode)
{
    bb_set_frame_change_t fc;
    memset(&fc, 0, sizeof(fc));
    fc.mode = mode ? 1 : 0;
    int ret = bb_ioctl(handle, BB_SET_FRAME_CHANGE, &fc, NULL);
    lc_log("lifecycle: tuning: BB_SET_FRAME_CHANGE(mode=%u) ret=%d", fc.mode, ret);
    return ret;
}
