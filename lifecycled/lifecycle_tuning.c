#include "lifecycle_tuning.h"
#include "lc_log.h"
#include "../common/ar8030_link_policy.h"
#include <errno.h>
#include <math.h>
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

/* <dir of cfg_path>/<name> -- every sidecar this file owns lives next to
 * the baseband JSON it tunes. */
void lc_sidecar_path(const char* cfg_path, const char* name, char* out, size_t out_sz)
{
    const char* slash = strrchr(cfg_path, '/');
    if (slash) {
        size_t dirlen = (size_t)(slash - cfg_path) + 1;
        if (dirlen >= out_sz) {
            dirlen = out_sz - 1;
        }
        memcpy(out, cfg_path, dirlen);
        snprintf(out + dirlen, out_sz - dirlen, "%s", name);
    } else {
        snprintf(out, out_sz, "%s", name);
    }
}

static void tuning_sidecar_path(const char* cfg_path, char* out, size_t out_sz)
{
    lc_sidecar_path(cfg_path, "ar8030.tuning", out, out_sz);
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

int lc_tuning_read_current(bb_dev_handle_t* handle, int is_ap)
{
    bb_get_status_in_t  in = {0xffff};
    bb_get_status_out_t out;
    memset(&out, 0, sizeof(out));

    if (bb_ioctl(handle, BB_GET_STATUS, &in, &out) != 0) {
        return -1;
    }
    /* Single-user mode only -- see lifecycle_pair.c's slot-scan comment. */
    int bw = out.user_status[is_ap ? BB_USER_BR_CS : BB_USER_0].tx_status.bandwidth;
    if (bw < 0 || bw >= BB_BW_MAX) {
        return -1;
    }
    return BW_MHZ_BY_ENUM[bw];
}

int lc_tuning_read_uplink(bb_dev_handle_t* handle)
{
    bb_get_status_in_t  in = {0xffff};
    bb_get_status_out_t out;
    memset(&out, 0, sizeof(out));

    if (bb_ioctl(handle, BB_GET_STATUS, &in, &out) != 0) {
        return -1;
    }
    int bw = out.user_status[BB_USER_0].rx_status.bandwidth;
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
    lc_sidecar_path(cfg_path, "ar8030.retx", out, out_sz);
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
    if (mode < 0 || mode > LC_FRAME_CHANGE_MAX) {
        lc_log("lifecycle: tuning: frame change mode %d out of range, not sent", mode);
        return -1;
    }
    bb_set_frame_change_t fc;
    memset(&fc, 0, sizeof(fc));
    fc.mode = (uint8_t)mode;
    int ret = bb_ioctl(handle, BB_SET_FRAME_CHANGE, &fc, NULL);
    lc_log("lifecycle: tuning: BB_SET_FRAME_CHANGE(mode=%u) ret=%d", fc.mode, ret);
    return ret;
}

int lc_frame_change_load(const char* cfg_path, int* out)
{
    char path[512];
    lc_sidecar_path(cfg_path, "ar8030.frame_change", path, sizeof(path));

    FILE* f = fopen(path, "r");
    if (!f) {
        return -1;
    }
    int mode = -1;
    int ok   = fscanf(f, "%d", &mode) == 1;
    fclose(f);
    if (!ok || mode < 0 || mode > LC_FRAME_CHANGE_MAX) {
        lc_log("lifecycle: tuning: %s has no valid frame change mode, ignoring", path);
        return -1;
    }
    *out = mode;
    return 0;
}

int lc_frame_change_save(const char* cfg_path, int mode)
{
    if (mode < 0 || mode > LC_FRAME_CHANGE_MAX) {
        lc_log("lifecycle: tuning: refusing to persist invalid frame change mode %d", mode);
        return -1;
    }

    char path[512];
    lc_sidecar_path(cfg_path, "ar8030.frame_change", path, sizeof(path));
    char tmp_path[520];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);

    FILE* f = fopen(tmp_path, "w");
    if (!f) {
        lc_log("lifecycle: tuning: can't open %s for writing: %s", tmp_path, strerror(errno));
        return -1;
    }
    fprintf(f, "%d\n", mode);
    fclose(f);

    if (rename(tmp_path, path) != 0) {
        lc_log("lifecycle: tuning: rename %s -> %s failed: %s", tmp_path, path, strerror(errno));
        return -1;
    }
    return 0;
}

int lc_channel_parse(const char* s, int* out)
{
    if (!s || !*s) {
        return -1;
    }
    if (strcmp(s, "auto") == 0) {
        *out = LC_CHANNEL_AUTO;
        return 0;
    }
    if (strcmp(s, "none") == 0) {
        *out = LC_CHANNEL_NONE;
        return 0;
    }
    char* end;
    long  v = strtol(s, &end, 10);
    if (end == s || *end != '\0' || v < 0 || v >= BB_CONFIG_MAX_CHAN_NUM) {
        return -1;
    }
    *out = (int)v;
    return 0;
}

int lc_channel_load(const char* cfg_path, int* out)
{
    char path[512];
    lc_sidecar_path(cfg_path, "ar8030.channel", path, sizeof(path));

    FILE* f = fopen(path, "r");
    if (!f) {
        return -1;
    }
    char buf[16] = {0};
    int  ok      = fscanf(f, "%15s", buf) == 1;
    fclose(f);
    int chan;
    if (!ok || lc_channel_parse(buf, &chan) != 0 || chan == LC_CHANNEL_NONE) {
        lc_log("lifecycle: tuning: %s has no valid channel, ignoring", path);
        return -1;
    }
    *out = chan;
    return 0;
}

int lc_channel_save(const char* cfg_path, int chan)
{
    if (chan != LC_CHANNEL_AUTO && (chan < 0 || chan >= BB_CONFIG_MAX_CHAN_NUM)) {
        lc_log("lifecycle: tuning: refusing to persist invalid channel %d", chan);
        return -1;
    }

    char path[512];
    lc_sidecar_path(cfg_path, "ar8030.channel", path, sizeof(path));
    char tmp_path[520];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);

    FILE* f = fopen(tmp_path, "w");
    if (!f) {
        lc_log("lifecycle: tuning: can't open %s for writing: %s", tmp_path, strerror(errno));
        return -1;
    }
    if (chan == LC_CHANNEL_AUTO) {
        fprintf(f, "auto\n");
    } else {
        fprintf(f, "%d\n", chan);
    }
    fclose(f);

    if (rename(tmp_path, path) != 0) {
        lc_log("lifecycle: tuning: rename %s -> %s failed: %s", tmp_path, path, strerror(errno));
        return -1;
    }
    return 0;
}

static const char* const DAEMON_LOG_NAMES[] = {"debug", "info", "warn", "error"};

int lc_daemon_log_parse(const char* s)
{
    if (!strcmp(s, "keep")) {
        return -1;
    }
    for (int i = 0; i < (int)(sizeof(DAEMON_LOG_NAMES) / sizeof(DAEMON_LOG_NAMES[0])); i++) {
        if (!strcmp(s, DAEMON_LOG_NAMES[i])) {
            return i;
        }
    }
    return -2;
}

int lc_daemon_log_apply(bb_host_t* host, int level)
{
    daemon_dbg_ctrl c;
    memset(&c, 0, sizeof(c));
    snprintf(c.names, sizeof(c.names), "print_level");
    c.level = level;
    bb_daemon_set_dbg_level(host, &c);
    c = bb_daemon_get_dbg_level(host, "print_level");
    lc_log("lifecycle: ar8030d log level %s (wanted %s)", c.level >= 0 && c.level <= 3 ? DAEMON_LOG_NAMES[c.level] : "?",
           DAEMON_LOG_NAMES[level]);
    return c.level == level ? 0 : -1;
}

int lc_channel_read(bb_dev_handle_t* handle, int* auto_mode, int* work_chan, int* chan_num)
{
    bb_get_chan_info_out_t out;
    memset(&out, 0, sizeof(out));
    if (bb_ioctl(handle, BB_GET_CHAN_INFO, NULL, &out) != 0) {
        return -1;
    }
    *auto_mode = out.auto_mode ? 1 : 0;
    *work_chan = out.work_chan;
    *chan_num  = out.chan_num;
    return 0;
}

int lc_channel_read_link_khz(bb_dev_handle_t* handle, uint32_t* down, uint32_t* up)
{
    bb_get_status_in_t  in = {0xffff};
    bb_get_status_out_t out;
    memset(&out, 0, sizeof(out));

    *down = *up = 0;
    if (bb_ioctl(handle, BB_GET_STATUS, &in, &out) != 0) {
        return -1;
    }
    *down = out.user_status[BB_USER_BR_CS].tx_status.freq_khz;
    *up   = out.user_status[BB_USER_0].rx_status.freq_khz;
    return 0;
}

int lc_channel_read_table(bb_dev_handle_t* handle, uint32_t* khz, int max)
{
    bb_get_chan_info_out_t out;
    memset(&out, 0, sizeof(out));
    if (bb_ioctl(handle, BB_GET_CHAN_INFO, NULL, &out) != 0) {
        return -1;
    }
    int n = out.chan_num;
    if (n > BB_CONFIG_MAX_CHAN_NUM) {
        n = BB_CONFIG_MAX_CHAN_NUM;
    }
    if (n > max) {
        n = max;
    }
    for (int i = 0; i < n; i++) {
        khz[i] = out.freq[i];
    }
    return n;
}

int lc_channel_matches(int chan, int auto_mode, int work_chan)
{
    if (chan == LC_CHANNEL_AUTO) {
        return auto_mode;
    }
    return !auto_mode && work_chan == chan;
}

int lc_channel_apply_mode(bb_dev_handle_t* handle, int chan)
{
    bb_set_chan_mode_t cm = {.auto_mode = (uint8_t)(chan == LC_CHANNEL_AUTO)};
    int                ret = bb_ioctl(handle, BB_SET_CHAN_MODE, &cm, NULL);
    lc_log("lifecycle: tuning: BB_SET_CHAN_MODE(auto_mode=%u) ret=%d", cm.auto_mode, ret);
    return ret;
}

int lc_channel_apply_local(bb_dev_handle_t* handle, int chan)
{
    int ret = lc_channel_apply_mode(handle, chan);
    if (ret != 0 || chan == LC_CHANNEL_AUTO) {
        return ret;
    }
    /* RX only: TX-direction channel control is director mode only (see
     * bb_set_chan_t), and an RX change applies to every slot at once. */
    bb_set_chan_t sc = {.chan_dir = BB_DIR_RX, .chan_index = (uint8_t)chan};
    ret              = bb_ioctl(handle, BB_SET_CHAN, &sc, NULL);
    lc_log("lifecycle: tuning: BB_SET_CHAN(rx, index=%d) ret=%d", chan, ret);
    return ret;
}

int lc_channel_apply_linked(bb_dev_handle_t* handle, int slot, int chan)
{
    int ret = lc_channel_apply_local(handle, chan);
    if (ret != 0) {
        lc_log("lifecycle: tuning: local channel change failed, not pushing to peer");
        return ret;
    }
    bb_set_remote_t sr;
    memset(&sr, 0, sizeof(sr));
    sr.slot     = (uint8_t)slot;
    sr.type_bmp = 1u << BB_REMOTE_TYPE_CHAN_MODE;
    sr.setting.auto_chan = (uint8_t)(chan == LC_CHANNEL_AUTO);
    if (chan != LC_CHANNEL_AUTO) {
        sr.type_bmp |= 1u << BB_REMOTE_TYPE_TARGET_CHAN;
        sr.setting.target_chan = (uint8_t)chan;
    }
    ret = bb_ioctl(handle, BB_SET_REMOTE, &sr, NULL);
    lc_log("lifecycle: tuning: BB_SET_REMOTE(slot=%d, auto_chan=%u, target_chan=%d) ret=%d", slot,
           sr.setting.auto_chan, chan, ret);
    if (ret != 0) {
        lc_log("lifecycle: tuning: channel changed locally but the peer push failed -- link likely desynced");
    }
    return ret;
}

/* Output power levels: each board offers its own list of levels in mW
 * (--power-levels), and a level's chip dBm target is 10 * log10(mW),
 * rounded -- the same label means the same dBm on every board. Without the
 * option a role gets the levels stock offers: air 400/200/100/25 mW (Caddx's
 * labels), ground auto/500/200/100/25 mW. */
static const int AP_DEFAULT_MW[]  = {400, 200, 100, 25};
static const int DEV_DEFAULT_MW[] = {500, 200, 100, 25};

/* The ground's bounds outside auto, as stock sends them ({OFF, 17, 14}). */
#define DEV_OFF_MAX_DBM 17
#define DEV_OFF_MIN_DBM 14

int lc_power_dbm(int mw)
{
    return (int)lround(10.0 * log10((double)mw));
}

int lc_power_parse(const char* s, int* out)
{
    if (!s || !*s) {
        return -1;
    }
    if (strcmp(s, "auto") == 0) {
        *out = LC_POWER_AUTO;
        return 0;
    }
    if (strcmp(s, "none") == 0) {
        *out = LC_POWER_NONE;
        return 0;
    }
    char* end;
    long  v = strtol(s, &end, 10);
    if (end == s || (*end != '\0' && strcmp(end, "mw") != 0 && strcmp(end, "mW") != 0) || v <= 0 || v > 10000) {
        return -1;
    }
    *out = (int)v;
    return 0;
}

static int table_add(lc_power_table_t* t, int mw, char* err, size_t err_sz)
{
    int dbm = lc_power_dbm(mw);
    if (dbm < LC_POWER_MIN_DBM || dbm > LC_POWER_MAX_DBM) {
        snprintf(err, err_sz, "%d mW is %d dBm, outside the chip's %d-%d", mw, dbm, LC_POWER_MIN_DBM,
                 LC_POWER_MAX_DBM);
        return -1;
    }
    for (int i = 0; i < t->n; i++) {
        if (t->mw[i] == mw) {
            snprintf(err, err_sz, "%d mW is listed twice", mw);
            return -1;
        }
    }
    if (t->n >= LC_POWER_MAX_LEVELS) {
        snprintf(err, err_sz, "more than %d levels", LC_POWER_MAX_LEVELS);
        return -1;
    }
    int i = t->n++; /* kept highest first */
    while (i > 0 && t->mw[i - 1] < mw) {
        t->mw[i] = t->mw[i - 1];
        i--;
    }
    t->mw[i] = mw;
    return 0;
}

void lc_power_table_default(int is_ap, lc_power_table_t* t)
{
    char         err[64];
    const int*   mw = is_ap ? AP_DEFAULT_MW : DEV_DEFAULT_MW;
    const size_t n =
        is_ap ? sizeof(AP_DEFAULT_MW) / sizeof(AP_DEFAULT_MW[0]) : sizeof(DEV_DEFAULT_MW) / sizeof(DEV_DEFAULT_MW[0]);
    memset(t, 0, sizeof(*t));
    t->auto_ok = !is_ap;
    for (size_t i = 0; i < n; i++) {
        table_add(t, mw[i], err, sizeof(err));
    }
}

int lc_power_table_parse(const char* s, int is_ap, lc_power_table_t* t, char* err, size_t err_sz)
{
    char buf[128];
    if (!s || !*s || strlen(s) >= sizeof(buf)) {
        snprintf(err, err_sz, "empty or too long");
        return -1;
    }
    snprintf(buf, sizeof(buf), "%s", s);
    memset(t, 0, sizeof(*t));
    char* save = NULL;
    for (char* tok = strtok_r(buf, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
        int level;
        if (lc_power_parse(tok, &level) != 0 || level == LC_POWER_NONE) {
            snprintf(err, err_sz, "'%s' is not a level in mW", tok);
            return -1;
        }
        if (level == LC_POWER_AUTO) {
            if (is_ap) {
                snprintf(err, err_sz, "auto is a ground level only");
                return -1;
            }
            t->auto_ok = 1;
        } else if (table_add(t, level, err, err_sz) != 0) {
            return -1;
        }
    }
    if (t->n == 0) {
        snprintf(err, err_sz, "no level in mW");
        return -1;
    }
    return 0;
}

void lc_power_table_cap(lc_power_table_t* t, int max_mw)
{
    int n = 0;
    for (int i = 0; i < t->n; i++) {
        if (t->mw[i] <= max_mw) {
            t->mw[n++] = t->mw[i];
        }
    }
    t->n = n;
    if (max_mw < DEV_DEFAULT_MW[0]) { /* as --max-power always did */
        t->auto_ok = 0;
    }
}

int lc_power_valid(const lc_power_table_t* t, int level)
{
    if (level == LC_POWER_AUTO) {
        return t->auto_ok && t->n > 0;
    }
    for (int i = 0; i < t->n; i++) {
        if (t->mw[i] == level) {
            return 1;
        }
    }
    return 0;
}

int lc_power_default(const lc_power_table_t* t)
{
    return t->n > 0 ? t->mw[0] : -1;
}

void lc_power_levels_json(const lc_power_table_t* t, char* out, size_t out_sz)
{
    size_t len = 0;
    out[0]     = '\0';
    if (t->auto_ok) {
        len += (size_t)snprintf(out, out_sz, "\"auto\"");
    }
    for (int i = 0; i < t->n && len < out_sz; i++) {
        len += (size_t)snprintf(out + len, out_sz - len, "%s%d", len ? "," : "", t->mw[i]);
    }
}

int lc_power_load(const char* cfg_path, int* out)
{
    char path[512];
    lc_sidecar_path(cfg_path, "ar8030.power", path, sizeof(path));

    FILE* f = fopen(path, "r");
    if (!f) {
        return -1;
    }
    char buf[16] = {0};
    int  ok      = fscanf(f, "%15s", buf) == 1;
    fclose(f);
    int level;
    if (!ok || lc_power_parse(buf, &level) != 0 || level == LC_POWER_NONE) {
        lc_log("lifecycle: tuning: %s has no valid power level, ignoring", path);
        return -1;
    }
    *out = level;
    return 0;
}

int lc_power_save(const char* cfg_path, int level)
{
    if (level != LC_POWER_AUTO && level <= 0) {
        lc_log("lifecycle: tuning: refusing to persist invalid power level %d", level);
        return -1;
    }

    char path[512];
    lc_sidecar_path(cfg_path, "ar8030.power", path, sizeof(path));
    char tmp_path[520];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);

    FILE* f = fopen(tmp_path, "w");
    if (!f) {
        lc_log("lifecycle: tuning: can't open %s for writing: %s", tmp_path, strerror(errno));
        return -1;
    }
    if (level == LC_POWER_AUTO) {
        fprintf(f, "auto\n");
    } else {
        fprintf(f, "%d\n", level);
    }
    fclose(f);

    if (rename(tmp_path, path) != 0) {
        lc_log("lifecycle: tuning: rename %s -> %s failed: %s", tmp_path, path, strerror(errno));
        return -1;
    }
    return 0;
}

#ifdef BB_HAVE_PWR_AUTO_BOUNDS
static int power_auto_ioctl(bb_dev_handle_t* handle, int mode, int max, int min)
{
    bb_set_pwr_auto_in_t pa = {.pwr_auto = (uint8_t)mode, .max = (uint8_t)max, .min = (uint8_t)min};
    int                  ret = bb_ioctl(handle, BB_SET_POWER_AUTO, &pa, NULL);
    lc_log("lifecycle: tuning: BB_SET_POWER_AUTO(mode=%d, max=%d, min=%d) ret=%d", mode, max, min, ret);
    return ret;
}
#endif

#ifdef BB_HAVE_PWR_AUTO_BOUNDS
/* The air side's sequence (see lc_power_apply() below), with stock's
 * power-auto mode 3 switched on (normal) or off (standby) first. */
static int ap_power_apply_dbm(bb_dev_handle_t* handle, int dbm, int auto3, int fem_ctrl)
{
    int ret = 0;
    if (fem_ctrl) {
        /* Mode 3 isn't in bb_pwr_auto_mode_e: stock sends {3, 1, 1}
         * before every power set in its normal (flying) mode and
         * {3, 0, 0} before its low-power one. */
        ret |= power_auto_ioctl(handle, 3, auto3, auto3);
    }
    ret |= power_auto_ioctl(handle, BB_PWR_AUTO_DISCONNECT, dbm, dbm);

    bb_set_pwr_in_t sp = {.usr = BB_USER_BR_CS, .pwr = (uint8_t)dbm};
    int             r  = bb_ioctl(handle, BB_SET_POWER, &sp, NULL);
    lc_log("lifecycle: tuning: BB_SET_POWER(usr=%d, pwr=%ddBm) ret=%d", BB_USER_BR_CS, dbm, r);
    if (fem_ctrl) {
        int on = dbm >= AR8030_FEM_CTRL_MIN_DBM;
        int rf = ar8030_set_fem_ctrl(handle, on);
        lc_log("lifecycle: tuning: fem ctrl %d (%d dBm) ret=%d", on, dbm, rf);
        r |= rf;
    }
    return ret | r;
}
#endif

/* Stock's sequences (see lifecycle_tuning.h), with the level's dBm:
 *  air:    {DISCONNECT, dbm, dbm}, then BB_SET_POWER(BR/CS user, dbm) --
 *          the AP's own power is set on the BR/CS user, not user 0
 *          (confirmed on hardware: BB_SET_POWER on user 0 moves what
 *          BB_GET_CUR_POWER reports, but not what the ground receives)
 *  ground: {ON, top, lowest} of the board's levels for auto (starting at
 *          the top one) / {OFF, 17, 14} otherwise, then {DISCONNECT, dbm,
 *          dbm}, then BB_SET_POWER(user 0, dbm). Stock's disconnect power
 *          is 3 dB below its target; here both are the level's. */
int lc_power_apply(bb_dev_handle_t* handle, int is_ap, const lc_power_table_t* t, int level, int fem_ctrl)
{
#ifdef BB_HAVE_PWR_AUTO_BOUNDS
    if (!lc_power_valid(t, level)) {
        lc_log("lifecycle: tuning: power level %d is not one of this board's, not applying", level);
        return -1;
    }
    int dbm = lc_power_dbm(level == LC_POWER_AUTO ? t->mw[0] : level);
    if (is_ap) {
        return ap_power_apply_dbm(handle, dbm, 1, fem_ctrl);
    }
    int ret = 0;
    if (level == LC_POWER_AUTO) {
        ret |= power_auto_ioctl(handle, BB_PWR_AUTO_ON, dbm, lc_power_dbm(t->mw[t->n - 1]));
    } else {
        ret |= power_auto_ioctl(handle, BB_PWR_AUTO_OFF, DEV_OFF_MAX_DBM, DEV_OFF_MIN_DBM);
    }
    ret |= power_auto_ioctl(handle, BB_PWR_AUTO_DISCONNECT, dbm, dbm);

    bb_set_pwr_in_t sp = {.usr = BB_USER_0, .pwr = (uint8_t)dbm};
    int             r  = bb_ioctl(handle, BB_SET_POWER, &sp, NULL);
    lc_log("lifecycle: tuning: BB_SET_POWER(usr=%d, pwr=%ddBm) ret=%d", BB_USER_0, dbm, r);
    return ret | r;
#else
    (void)handle;
    (void)is_ap;
    (void)t;
    (void)level;
    (void)fem_ctrl;
    lc_log("lifecycle: tuning: SDK lacks the BB_SET_POWER_AUTO size fix, not touching output power");
    return -1;
#endif
}

/* Stock's low-power sequence (ar_ldyhs_sky fpv_sky_standby_mode_thread on
 * the Lite): {3, 0, 0}, then fpv_bb_set_local_power(10) -- values below 20
 * are taken as dBm -- i.e. {DISCONNECT, 10, 10}, BB_SET_POWER(BR/CS, 10)
 * and the fem ctrl bit off. */
int lc_power_apply_standby(bb_dev_handle_t* handle, int dbm, int fem_ctrl)
{
#ifdef BB_HAVE_PWR_AUTO_BOUNDS
    return ap_power_apply_dbm(handle, dbm, 0, fem_ctrl);
#else
    (void)handle;
    (void)dbm;
    (void)fem_ctrl;
    lc_log("lifecycle: tuning: SDK lacks the BB_SET_POWER_AUTO size fix, not touching output power");
    return -1;
#endif
}

int lc_power_read_dbm(bb_dev_handle_t* handle, int is_ap)
{
    bb_get_cur_pwr_in_t  in = {.usr = (uint8_t)(is_ap ? BB_USER_BR_CS : BB_USER_0)};
    bb_get_cur_pwr_out_t out;
    memset(&out, 0, sizeof(out));
    if (bb_ioctl(handle, BB_GET_CUR_POWER, &in, &out) != 0) {
        return -1;
    }
    return out.pwr;
}

int lc_distance_read_m(bb_dev_handle_t* handle)
{
    bb_get_distc_result_in_t  in = {.slot_bmp = 1u << BB_SLOT_0};
    bb_get_distc_result_out_t out;
    memset(&out, 0, sizeof(out));
    if (bb_ioctl(handle, BB_GET_DISTC_RESULT, &in, &out) != 0) {
        return -1;
    }
    return out.distance[BB_SLOT_0] >= 0 ? out.distance[BB_SLOT_0] : -1;
}
