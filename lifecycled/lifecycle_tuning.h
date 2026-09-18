#ifndef LIFECYCLE_TUNING_H
#define LIFECYCLE_TUNING_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ar8030.h"
#include "bb_api.h"

/*
 * Bandwidth-only, deliberately: the shell logic this replaces
 * (S65ar8030-transport-tx's own apply_link_tuning()) only ever managed
 * bandwidth, never channel -- this matches that scope rather than
 * growing it. Channel is additionally a harder no for now: this
 * project's own ar8030-linkctl found BB_GET_CHAN_INFO segfaults
 * deterministically on this device's 42-channel config (its output
 * struct's fixed 32-slot arrays overflow), so there is no safe way to
 * read the chip's current channel at all right now -- and reading the
 * current value is exactly what re-persisting a live operator change
 * needs. Setting a channel blind (no readback) also means coordinating
 * a live push to the connected peer to keep both sides in sync, real
 * complexity with no working way to verify it landed -- not something
 * to take on without dedicated hardware time.
 */

/* Non-zero if mhz is one of the AR8030's actual bandwidth gears
 * (1/2/5/10/20/40 -- see BW_MHZ_BY_ENUM). Exported so callers outside
 * this file (the HTTP control API) can reject a bad value up front with
 * a clear 400 instead of letting it fail silently deeper in
 * lc_tuning_apply()/lc_tuning_save(). */
int lc_tuning_valid_mhz(int mhz);

/* Reads the persisted bandwidth (MHz: 1/2/5/10/20/40) from the sidecar
 * file next to cfg_path (same directory, fixed name "ar8030.tuning").
 * Returns the bandwidth in MHz on success, -1 if no sidecar exists yet
 * or it's unreadable/invalid -- callers fall back to their own
 * configured default in that case. */
int lc_tuning_load(const char* cfg_path);

/* Atomically persists bandwidth_mhz to the sidecar file next to
 * cfg_path. Returns 0 on success. */
int lc_tuning_save(const char* cfg_path, int bandwidth_mhz);

/* Reads the AR8030's own currently-active TX bandwidth (MHz) for user 0
 * via BB_GET_STATUS. Returns -1 on ioctl failure or an out-of-range
 * reading. */
int lc_tuning_read_current(bb_dev_handle_t* handle);

/* Applies bandwidth_mhz (TX direction) to the given slot via
 * BB_SET_BANDWIDTH. slot must be the slot actually reporting CONNECT
 * (see lc_tuning_resolve_connected_slot()) -- which slot a peer lands
 * on isn't fixed across reboots, confirmed by this project's own
 * ar8030-linkctl (`bandwidth ... -s auto`). Returns the ioctl's own
 * return code (0 on success). */
int lc_tuning_apply(bb_dev_handle_t* handle, int slot, int bandwidth_mhz);

/* Scans BB_GET_STATUS for whichever slot is actually reporting
 * BB_LINK_STATE_CONNECT right now. Returns the slot index, or -1 if
 * none are connected. */
int lc_tuning_resolve_connected_slot(bb_dev_handle_t* handle);

#ifdef __cplusplus
}
#endif
#endif
