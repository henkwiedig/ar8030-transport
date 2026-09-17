#ifndef __LIFECYCLE_GPIO_H__
#define __LIFECYCLE_GPIO_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "lifecycle.h"

/*
 * Re-pulses the AR8030's hardware reset line, forcing a fresh firmware
 * push -- the in-daemon equivalent of S65ar8030-transport-tx's `reset`
 * action (devmem pokes) and S97ar8030's `enable_rf()` (raw GPIO sequence).
 * Which backend runs is a runtime choice (cfg->reset_method), not a
 * compile-time one -- both are always compiled in on both repos.
 *
 * Checks lifecycle_shutdown_requested() between the two pulse phases of
 * whichever backend runs and, if a shutdown was requested mid-sequence,
 * completes the current pulse (never leaves the reset line asserted)
 * before returning early.
 *
 * Returns 0 on success, -1 on failure (logged internally).
 */
int lifecycle_gpio_reset(const lc_config_t* cfg);

/* Reads gpio<gpio>/value into *out (0 or 1). Sibling to the internal
 * lc_gpio_write() this module already has -- unlike the reset backends,
 * this does not export the pin or set its direction; that stays the
 * device overlay's job (muxes.sh already exports/configures the bind
 * button pin as an input before this daemon ever starts, the same way
 * it does for the LED pins hooks.d scripts write to). Used by
 * lifecycle_bind.c to poll the physical bind button. Returns 0 on
 * success, -1 if the sysfs node can't be opened/read. */
int lc_gpio_read(int gpio, int* out);

#ifdef __cplusplus
}
#endif
#endif
