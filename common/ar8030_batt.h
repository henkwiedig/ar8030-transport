#ifndef AR8030_BATT_H
#define AR8030_BATT_H

/*
 * Supply (battery) voltage via the AR8030's own ADC -- header-only, shared
 * by lifecycled/ (periodic poll) and linkctl/ (one-shot `batt`). Uses the
 * same ADC access as ar8030_rftemp.h (arm 0x8a / read 0x89 over
 * BB_*_PRJ_DISPATCH).
 *
 * Stock (ar_ldyhs_sky, Ghidra) on the Caddx Ascent Lite -- board CX472,
 * project type 7, picked by fpv_run_by_type.sh from the SoC LSADC
 * channel 1 board-ID resistor (896 mV here):
 *
 *  - fpv_bb_init() arms AR8030 ADC channel 3 (200 ms period).
 *  - fpv_sys_update_batt_volt() averages 10 channel-3 readings and keeps
 *    offset = 900 - avg (channel 3 is treated as a 900 mV reference),
 *    then re-arms channel 0 (300 ms) and smooths it 3:1 from then on.
 *  - fpv_sys_get_batt_volt_mv() returns
 *      corr = ch0 * (900 + offset) / 900
 *      type 7: (corr + 25) * 16      types 4/6: corr * 16 + 200
 *    and to_gnd_send_period_info() ships that to the goggles.
 *
 * Measured on real hardware with a lab PSU at the board's power input
 * (2026-09, air unit), channel 0 averaged over 10 reads:
 *
 *      PSU     ch0      stock      16*ch0 + 1200
 *      6.0 V   299 mV   5.38 V     5.98 V
 *      9.0 V   488 mV   8.52 V     9.01 V
 *     12.0 V   674 mV  11.57 V    11.98 V
 *
 * A least-squares fit gives slope 16.00 and offset 1206 mV, residuals
 * within +/-25 mV (one ADC step is a few mV, x16 on the supply). Stock's
 * channel-3 "calibration" makes it worse on this board: channel 3 sits at
 * 856-870 mV, so stock always scales up by ~4% and still adds only 400 mV,
 * reading 0.4-0.6 V low. So this does the plain linear mapping, with the
 * scale/offset as parameters for other boards:
 *
 *      supply_mv = adc_mv * scale + offset_mv
 *
 * On USB power alone (nothing on the power input) channel 0 reads 0 mV --
 * reported as "no reading", not as offset_mv. Arming any channel starts
 * the whole ADC (confirmed: every channel keeps reading live after the
 * arm), so this can share the ADC with rf-temp without taking turns.
 */

#include "ar8030_rftemp.h"

#define AR8030_BATT_DEFAULT_ADC       0    /* Caddx Ascent Lite */
#define AR8030_BATT_DEFAULT_SCALE     16   /* on-board divider */
#define AR8030_BATT_DEFAULT_OFFSET_MV 1200 /* measured, see above */

/* ADC mV -> supply mV, or -1 for adc_mv <= 0 (no supply on the power
 * input, or the ADC isn't armed yet). */
static inline int ar8030_batt_mv(int adc_mv, int scale, int offset_mv)
{
    return adc_mv > 0 ? adc_mv * scale + offset_mv : -1;
}

#endif
