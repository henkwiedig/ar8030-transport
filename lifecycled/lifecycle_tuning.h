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

/*
 * BB_SET_RETX_EVENT_STATUS / BB_GET_RETX_EVENT_STATUS (see bb_api.h's own
 * doc comment on bb_retx_cfg_t) -- the windowed retransmission
 * controller's own tuning parameters. Confirmed live on real hardware
 * (2026-09) that a SET after boot actually takes effect (read back
 * matches what was written), unlike bandwidth this is chip-wide, not
 * per-slot -- BB_SET_RETX_EVENT_STATUS's own struct carries no slot
 * field. Only the first 5 of the struct's 136 bytes are understood
 * (win/busy/idle/conti_busy/conti_idle); this file only ever reads/
 * writes those 5, leaving the rest zeroed on SET and ignored on GET.
 */

/* Non-zero if every one of the 5 values fits a uint8_t (0-255) -- the
 * only constraint currently known; unlike lc_tuning_valid_mhz() there is
 * no fixed set of legal values to check against since the real units
 * of these thresholds are still unconfirmed (see bb_retx_cfg_t's own
 * doc comment). */
int lc_retx_valid(int win, int busy, int idle, int conti_busy, int conti_idle);

/* Reads the persisted retx config from the sidecar file next to
 * cfg_path (same directory, fixed name "ar8030.retx"). Returns 0 and
 * fills *out on success, -1 if no sidecar exists yet or it's
 * unreadable/invalid. */
int lc_retx_load(const char* cfg_path, bb_retx_cfg_t* out);

/* Atomically persists the 5 values to the sidecar file next to
 * cfg_path. Returns 0 on success. */
int lc_retx_save(const char* cfg_path, int win, int busy, int idle, int conti_busy, int conti_idle);

/* Applies the 5 values via BB_SET_RETX_EVENT_STATUS (chip-wide, no slot
 * parameter). Returns the ioctl's own return code (0 on success). */
/* BB_SET_FRAME_CHANGE (1V1 only). mode=1 exchanges the frame structure
 * (confirmed live on air: BB_GET_MCS throughput at MCS 12 / 20M goes from
 * 25933 to 36688 kbps, matching what stock reaches); mode=0 restores the
 * original. Does not survive a reboot or re-link -- must be re-applied
 * after every connect. Idempotent: confirmed live that repeating mode=1
 * on an already-exchanged link leaves throughput unchanged, so it is safe
 * to re-assert periodically. Returns the ioctl's own return code. */
int lc_frame_change_apply(bb_dev_handle_t* handle, int mode);

int lc_retx_apply(bb_dev_handle_t* handle, int win, int busy, int idle, int conti_busy, int conti_idle);

#ifdef __cplusplus
}
#endif
#endif
