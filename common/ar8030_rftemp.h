#ifndef AR8030_RFTEMP_H
#define AR8030_RFTEMP_H

/*
 * RF-board temperature via the AR8030's own ADC -- header-only, shared by
 * lifecycled/ (periodic poll) and linkctl/ (one-shot `rf-temp`).
 *
 * Reverse-engineered from stock's ar_ldyhs_sky (Caddx Ascent air unit,
 * Ghidra):
 *
 *  - fpv_bb_init() arms ADC channel 4 once, for project types 5/8 only:
 *    fpv_bb_set_adc_meas(4, 500) = BB_SET_PRJ_DISPATCH, data[0] = 0x8a,
 *    data[4] = channel, u32 data[8] = 500 (same "0x8a <ch> <u32>
 *    (adc_meas)" linkctl's prj-cmd usage already lists).
 *  - fpv_bb_get_prj_dispatch_adc(4, 0) reads it: BB_GET_PRJ_DISPATCH,
 *    data[0] = 0x89, data[4] = channel; the reading (mV) comes back as
 *    the u32 at out.data[4].
 *  - fpv_bb_update_rf_board_temp() maps mV -> degC through the 12-point
 *    thermistor table below and smooths it (new = 0.75*old + 0.25*sample,
 *    in 0.1 degC); fpv_cmd_get_rf_temp() reports that / 10.
 *
 * Deviations from stock, deliberately: stock interpolates using the pair
 * *above* the reading (extrapolating backwards) -- this uses plain linear
 * interpolation between the two neighbours. Below the first entry stock
 * reports a fixed 40 degC; this reports "no sensor" instead (see
 * AR8030_RFTEMP_NONE). Above the last entry stock reports 130 degC; kept.
 *
 * Only some boards have the thermistor: stock reads it for project types
 * 5/8 alone. The type comes from ar_ldyhs_sky's --board_type argument
 * (482 -> 4, 4861 -> 5, 492 -> 6, 472 -> 7, 4862 -> 8), which stock's
 * fpv_run_by_type.sh picks from the SoC's own LSADC channel 1. So only
 * the CX4861/CX4862 boards (separate RF board, --rf_board) have it; the
 * Ascent Lite is CX472 (type 7), where channel 0 is the supply voltage
 * (see ar8030_batt.h), channel 3 a ~860 mV reference and channel 4 floats
 * at a few hundred mV.
 *
 * The table is the Ascent's own thermistor curve. Another AR8030 board
 * may use a different channel (hence lifecycled's --rf-temp-adc) and
 * would need its own curve.
 */

#include "ar8030.h"
#include "bb_api.h"
#include <stdint.h>
#include <string.h>

#define AR8030_RFTEMP_CMD_ARM       0x8a
#define AR8030_RFTEMP_CMD_READ      0x89
#define AR8030_RFTEMP_ARM_PERIOD    500 /* stock's own value; presumably ms */
#define AR8030_RFTEMP_DEFAULT_ADC   4   /* Caddx Ascent RF board */

static const struct {
    int temp_c;
    int mv;
} ar8030_rftemp_table[] = {
    {0, 500},    {15, 740},   {25, 900},   {40, 1076},  {50, 1193},  {65, 1370},
    {75, 1440},  {85, 1510},  {95, 1570},  {105, 1660}, {115, 1700}, {125, 1740},
};
#define AR8030_RFTEMP_TABLE_LEN ((int)(sizeof(ar8030_rftemp_table) / sizeof(ar8030_rftemp_table[0])))

/* Starts periodic ADC measurement on `channel`. Chip-local state, lost on
 * every chip reset -- re-arm after one. Returns bb_ioctl's result. */
static inline int ar8030_rftemp_arm(bb_dev_handle_t* h, int channel)
{
    uint8_t  buf[256];
    uint32_t period = AR8030_RFTEMP_ARM_PERIOD;
    memset(buf, 0, sizeof(buf));
    buf[0] = AR8030_RFTEMP_CMD_ARM;
    buf[4] = (uint8_t)channel;
    memcpy(&buf[8], &period, sizeof(period));
    return bb_ioctl(h, BB_SET_PRJ_DISPATCH, buf, NULL);
}

/* Reads the last measurement on `channel` in mV. Returns bb_ioctl's
 * result; *mv is only written on success. 0 mV means "not armed yet". */
static inline int ar8030_rftemp_read_mv(bb_dev_handle_t* h, int channel, int* mv)
{
    bb_get_prj_dispatch_in_t  in;
    bb_get_prj_dispatch_out_t out;
    uint32_t                  val;
    memset(&in, 0, sizeof(in));
    memset(&out, 0, sizeof(out));
    in.data[0] = AR8030_RFTEMP_CMD_READ;
    in.data[4] = (uint8_t)channel;
    int ret = bb_ioctl(h, BB_GET_PRJ_DISPATCH, &in, &out);
    if (ret == 0) {
        memcpy(&val, &out.data[4], sizeof(val));
        *mv = (int)val;
    }
    return ret;
}

/* Returned for readings below the table: no thermistor on this channel. */
#define AR8030_RFTEMP_NONE (-10000)

/* mV -> tenths of a degC through ar8030_rftemp_table, or
 * AR8030_RFTEMP_NONE below its first entry. */
static inline int ar8030_rftemp_mv_to_c10(int mv)
{
    if (mv < ar8030_rftemp_table[0].mv) {
        return AR8030_RFTEMP_NONE;
    }
    for (int i = 1; i < AR8030_RFTEMP_TABLE_LEN; i++) {
        if (mv < ar8030_rftemp_table[i].mv) {
            int t0 = ar8030_rftemp_table[i - 1].temp_c * 10, t1 = ar8030_rftemp_table[i].temp_c * 10;
            int v0 = ar8030_rftemp_table[i - 1].mv, v1 = ar8030_rftemp_table[i].mv;
            return t0 + (t1 - t0) * (mv - v0) / (v1 - v0);
        }
    }
    return 1300;
}

#endif
