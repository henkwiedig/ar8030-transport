#ifndef AR8030_LINK_POLICY_H
#define AR8030_LINK_POLICY_H

/*
 * Stock's runtime link policy -- header-only, shared by lifecycled/
 * (applied automatically) and linkctl/ (manual pushes).
 *
 * Neither stock app leaves the MCS adaptation on ar8030.json's tables.
 * Reverse-engineered with Ghidra:
 *
 *  - Which table matters: the receiver's. The chip picks its peer's TX MCS
 *    from its own RX SNR/LDPC against its own table, so the goggles' table
 *    drives the video downlink and the air's drives the uplink. Item mcs
 *    numbers are ar8030.json's "mcs" + 2 (json -1/0/3/5/6/8/10 = item
 *    1/2/5/7/8/10/12 = BB_GET_MCS 2.3/4.6/9.1/13.8/18.3/27.7/36.7 Mbit/s).
 *
 *  - Goggles (ar_ldy_gnd FUN_0018aed8 -> FUN_001a20b0 -> FUN_001a1c68):
 *    ~2 s after every connect, if video_strategy is 1 or 2, it reloads all
 *    7 items of its table. Strategy 2 ("video high quality", what the
 *    Ascent Lite ships with: video_strategy=2 in /factory/fact_env.json)
 *    has thresholds ~2.5-3 dB below ar8030.json's dev table -- e.g. MCS 10
 *    at snr >= 1062 instead of 1888, kept down to 843 instead of 1191 --
 *    and gates on more LDPC errors instead. Without it the downlink sits
 *    one or two MCS steps lower at the same SNR.
 *
 *  - Air (ar_ldyhs_sky fpv_ap_set_mcs_policy(), end of fpv_bb_init):
 *    3 items (MCS 1/2/5) with higher, slower thresholds, then
 *    BB_SET_MCS_RANGE(slot 0, 1|2, 2) -- the uplink never goes above
 *    MCS 2, so the ground's ACKs/feedback stay robust -- and
 *    rfo_kikp (PRJ 0x8c) = 1 for strategy 2, else range (2, 2) and 0.
 *
 *  - Air output power (ar_ldyhs_sky FUN_00060900 / fpv_bb_set_local_power):
 *    in normal (flying) mode BB_SET_POWER_AUTO {3, 1, 1} first, then
 *    {2, dBm, dBm} + BB_SET_POWER(BR/CS, dBm), and on the Ascent Lite
 *    (project type 7) the "fem ctrl" bit -- PRJ 0x0e, data[4] = 0x83,
 *    data[5] = 0x50, data[6] bit 1 -- set for >= 24 dBm (the 500 mW / our
 *    400 mW level), cleared below. Its idle low-power mode sends {3, 0, 0}
 *    instead and 10 dBm. What mode 3 and the fem bit do inside the chip is
 *    not decoded; they are replayed as stock sends them.
 */

#include "ar8030.h"
#include "bb_api.h"
#include <stdint.h>
#include <string.h>

#define AR8030_PRJ_RFO_KIKP 0x8c
#define AR8030_PRJ_FEM_CTRL 0x0e

/* Stock's own minimum for the fem bit (ar_ldyhs_sky: uVar6 < 0x18). */
#define AR8030_FEM_CTRL_MIN_DBM 24

typedef struct {
    uint8_t  mcs;
    uint8_t  ldpc_up_num;
    uint16_t snr_up;
    uint16_t snr_dw;
    uint8_t  rsv2;
    uint8_t  ldpc_dw_num;
    uint16_t up_keep_time;
    uint16_t dw_keep_time;
} ar8030_mcs_item_t;

#define AR8030_GND_MCS_ITEMS 7
#define AR8030_AIR_MCS_ITEMS 3

/* ar_ldy_gnd FUN_001a1c68: [0] = video_strategy 0/1, [1] = strategy 2. */
static const ar8030_mcs_item_t ar8030_gnd_mcs_tables[2][AR8030_GND_MCS_ITEMS] = {
    {
        {1, 2, 0x24, 0x1d, 0, 4, 1000, 15},
        {2, 2, 0x5c, 0x41, 0, 4, 1500, 15},
        {5, 2, 0xa8, 0x77, 0, 4, 1500, 15},
        {7, 2, 0x12f, 0xf1, 0, 4, 800, 15},
        {8, 2, 0x256, 0x1db, 0, 3, 800, 15},
        {10, 2, 0x4a8, 0x3b3, 0, 4, 1000, 12},
        {12, 2, 0x736, 0x5ba, 0, 2, 1000, 1},
    },
    {
        {1, 3, 0x24, 0x1d, 0, 5, 1000, 0},
        {2, 4, 0x41, 0x34, 0, 6, 1500, 100},
        {5, 3, 0x72, 0x5a, 0, 5, 1500, 100},
        {7, 3, 300, 0xd6, 0, 5, 700, 25},
        {8, 3, 0x214, 0x1a6, 0, 5, 800, 15},
        {10, 3, 0x426, 0x34b, 0, 5, 1000, 0},
        {12, 3, 0x736, 0x5bb, 0, 5, 1000, 0},
    },
};

/* ar_ldyhs_sky fpv_ap_reload_mcs_tab (byte 10 = 2 in every entry, unlike
 * the ground's). */
static const ar8030_mcs_item_t ar8030_air_mcs_table[AR8030_AIR_MCS_ITEMS] = {
    {1, 2, 0x42, 0x2f, 2, 4, 1000, 500},
    {2, 2, 0x83, 0x5d, 2, 3, 500, 10},
    {5, 2, 0xee, 0xa9, 2, 4, 500, 30},
};

static inline const ar8030_mcs_item_t* ar8030_gnd_mcs_table(int strategy)
{
    return ar8030_gnd_mcs_tables[strategy == 2 ? 1 : 0];
}

/* One BB_SET_MCS_ITEM on slot 0. Returns bb_ioctl's result. */
static inline int ar8030_set_mcs_item(bb_dev_handle_t* h, const ar8030_mcs_item_t* e)
{
    bb_set_mcs_item_t item;
    memset(&item, 0, sizeof(item));
    item.mcs          = e->mcs;
    item.ldpc_up_num  = e->ldpc_up_num;
    item.snr_up       = e->snr_up;
    item.snr_dw       = e->snr_dw;
    item.rsv2         = e->rsv2;
    item.ldpc_dw_num  = e->ldpc_dw_num;
    item.up_keep_time = e->up_keep_time;
    item.dw_keep_time = e->dw_keep_time;
    return bb_ioctl(h, BB_SET_MCS_ITEM, &item, NULL);
}

static inline int ar8030_set_mcs_range(bb_dev_handle_t* h, int slot, int mcs_min, int mcs_max)
{
    bb_set_mcs_range_in_t mr;
    memset(&mr, 0, sizeof(mr));
    mr.slot    = (uint8_t)slot;
    mr.mcs_min = (uint8_t)mcs_min;
    mr.mcs_max = (uint8_t)mcs_max;
    return bb_ioctl(h, BB_SET_MCS_RANGE, &mr, NULL);
}

/* BB_SET_PRJ_DISPATCH: byte 0 = cmd, bytes 4.. payload. */
static inline int ar8030_prj_set(bb_dev_handle_t* h, uint8_t cmd, const uint8_t* data, size_t len)
{
    uint8_t buf[256];
    memset(buf, 0, sizeof(buf));
    buf[0] = cmd;
    if (len > sizeof(buf) - 4) {
        len = sizeof(buf) - 4;
    }
    memcpy(&buf[4], data, len);
    return bb_ioctl(h, BB_SET_PRJ_DISPATCH, buf, NULL);
}

static inline int ar8030_set_rfo_kikp(bb_dev_handle_t* h, int on)
{
    uint8_t d = on ? 1 : 0;
    return ar8030_prj_set(h, AR8030_PRJ_RFO_KIKP, &d, 1);
}

static inline int ar8030_set_fem_ctrl(bb_dev_handle_t* h, int on)
{
    uint8_t d[3] = {0x83, 0x50, (uint8_t)(on ? 0x02 : 0x00)};
    return ar8030_prj_set(h, AR8030_PRJ_FEM_CTRL, d, sizeof(d));
}

#endif
