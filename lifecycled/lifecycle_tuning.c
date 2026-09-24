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

/* <dir of cfg_path>/<name> -- every sidecar this file owns lives next to
 * the baseband JSON it tunes. */
static void sidecar_path(const char* cfg_path, const char* name, char* out, size_t out_sz)
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
    sidecar_path(cfg_path, "ar8030.tuning", out, out_sz);
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
    sidecar_path(cfg_path, "ar8030.retx", out, out_sz);
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
    sidecar_path(cfg_path, "ar8030.channel", path, sizeof(path));

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
    sidecar_path(cfg_path, "ar8030.channel", path, sizeof(path));
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

int lc_channel_matches(int chan, int auto_mode, int work_chan)
{
    if (chan == LC_CHANNEL_AUTO) {
        return auto_mode;
    }
    return !auto_mode && work_chan == chan;
}

int lc_channel_apply_local(bb_dev_handle_t* handle, int chan)
{
    bb_set_chan_mode_t cm = {.auto_mode = (uint8_t)(chan == LC_CHANNEL_AUTO)};
    int                ret = bb_ioctl(handle, BB_SET_CHAN_MODE, &cm, NULL);
    lc_log("lifecycle: tuning: BB_SET_CHAN_MODE(auto_mode=%u) ret=%d", cm.auto_mode, ret);
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

/* mW level -> chip dBm target, per role, from the stock streamers' own
 * tables (Ghidra, see lifecycle_tuning.h): ar_ldyhs_sky's table for the
 * Lite's board type (25->13, 100->19, 150->21, 200->22, 500->26) and
 * ar_ldy_gnd's (25->11, 100->17, 200->20, 500->24). The air side's top
 * level is stock's own "500 mW" entry, offered as 400 mW (26 dBm). */
typedef struct {
    int mw;
    int dbm;
} lc_power_level_t;

static const lc_power_level_t AP_POWER_LEVELS[] = {{400, 26}, {200, 22}, {100, 19}, {25, 13}};
static const lc_power_level_t DEV_POWER_LEVELS[] = {{500, 24}, {200, 20}, {100, 17}, {25, 11}};

/* Ground "auto": stock's 501 mW (500 + auto bit): target 24, adaptation
 * bounded to [14, 27] -- 14 is ar_ldy_gnd's floor when the air unit is a
 * Lite (sky project type 4/7), 27 its auto ceiling for 501 mW. */
#define DEV_AUTO_DBM     24
#define DEV_AUTO_MIN_DBM 14
#define DEV_AUTO_MAX_DBM 27

/* ar_ldy_gnd adds +3 to the ground's own BB_SET_POWER target when the air
 * unit is a type 4/6/7 board (the Lite is 4/7) -- +6 when a config flag we
 * haven't decoded is clear; the smaller of the two is used here. */
#define DEV_TARGET_OFFSET_DBM 3

static const lc_power_level_t* power_levels(int is_ap, size_t* n)
{
    if (is_ap) {
        *n = sizeof(AP_POWER_LEVELS) / sizeof(AP_POWER_LEVELS[0]);
        return AP_POWER_LEVELS;
    }
    *n = sizeof(DEV_POWER_LEVELS) / sizeof(DEV_POWER_LEVELS[0]);
    return DEV_POWER_LEVELS;
}

static int power_level_dbm(int is_ap, int level)
{
    if (level == LC_POWER_AUTO) {
        return is_ap ? -1 : DEV_AUTO_DBM;
    }
    size_t                  n;
    const lc_power_level_t* t = power_levels(is_ap, &n);
    for (size_t i = 0; i < n; i++) {
        if (t[i].mw == level) {
            return t[i].dbm;
        }
    }
    return -1;
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

int lc_power_valid(int is_ap, int level)
{
    return power_level_dbm(is_ap, level) >= 0;
}

int lc_power_default(int is_ap)
{
    return is_ap ? 400 : 500;
}

void lc_power_levels_json(int is_ap, char* out, size_t out_sz)
{
    size_t                  n;
    const lc_power_level_t* t   = power_levels(is_ap, &n);
    size_t                  len = 0;
    out[0]                      = '\0';
    if (!is_ap) {
        len += (size_t)snprintf(out, out_sz, "\"auto\"");
    }
    for (size_t i = 0; i < n && len < out_sz; i++) {
        len += (size_t)snprintf(out + len, out_sz - len, "%s%d", len ? "," : "", t[i].mw);
    }
}

int lc_power_load(const char* cfg_path, int* out)
{
    char path[512];
    sidecar_path(cfg_path, "ar8030.power", path, sizeof(path));

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
    sidecar_path(cfg_path, "ar8030.power", path, sizeof(path));
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

/* Stock's sequences (see lifecycle_tuning.h):
 *  air:    {DISCONNECT, dbm, dbm}, then BB_SET_POWER(BR/CS user, dbm) --
 *          the AP's own power is set on the BR/CS user, not user 0
 *          (confirmed on hardware: BB_SET_POWER on user 0 moves what
 *          BB_GET_CUR_POWER reports, but not what the ground receives)
 *  ground: {ON, 27, 14} for auto / {OFF, 17, 14} otherwise, then
 *          {DISCONNECT, dbm, dbm}, then BB_SET_POWER(user 0, dbm + 3) */
int lc_power_apply(bb_dev_handle_t* handle, int is_ap, int level)
{
#ifdef BB_HAVE_PWR_AUTO_BOUNDS
    int dbm = power_level_dbm(is_ap, level);
    if (dbm < 0) {
        lc_log("lifecycle: tuning: power level %d is not valid for this role, not applying", level);
        return -1;
    }
    int ret = 0;
    int target;
    int usr;
    if (is_ap) {
        usr    = BB_USER_BR_CS;
        target = dbm;
    } else {
        usr    = BB_USER_0;
        target = dbm + DEV_TARGET_OFFSET_DBM;
        if (level == LC_POWER_AUTO) {
            ret |= power_auto_ioctl(handle, BB_PWR_AUTO_ON, DEV_AUTO_MAX_DBM, DEV_AUTO_MIN_DBM);
        } else {
            ret |= power_auto_ioctl(handle, BB_PWR_AUTO_OFF, DEV_AUTO_MIN_DBM + 3, DEV_AUTO_MIN_DBM);
        }
    }
    ret |= power_auto_ioctl(handle, BB_PWR_AUTO_DISCONNECT, dbm, dbm);

    bb_set_pwr_in_t sp = {.usr = (uint8_t)usr, .pwr = (uint8_t)target};
    int             r  = bb_ioctl(handle, BB_SET_POWER, &sp, NULL);
    lc_log("lifecycle: tuning: BB_SET_POWER(usr=%d, pwr=%ddBm) ret=%d", usr, target, r);
    return ret | r;
#else
    (void)handle;
    (void)is_ap;
    (void)level;
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
