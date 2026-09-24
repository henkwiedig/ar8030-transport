#ifndef LIFECYCLE_TUNING_H
#define LIFECYCLE_TUNING_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ar8030.h"
#include "bb_api.h"

/*
 * Bandwidth, retx and channel. Bandwidth mirrors the shell logic this
 * replaced (S65ar8030-transport-tx's own apply_link_tuning()). Channel
 * came later, once BB_GET_CHAN_INFO became readable: it used to segfault
 * on this device's 42-channel table (the output struct's fixed 32-slot
 * arrays overflowed) until the ar8030 package raised
 * BB_CONFIG_MAX_CHAN_NUM to 60 (patch 0022/0029) -- see
 * ar8030-linkctl's own cmd_status() comment.
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

/*
 * Channel: either an index into the chip's pre-configured channel table
 * (ar8030.json's baseband.basic.channel.freq[]) in manual mode, or the
 * chip's own channel adaptation (auto mode). Stored as one int:
 * LC_CHANNEL_AUTO, LC_CHANNEL_NONE ("don't touch the chip's channel at
 * all", --default-channel none) or an index >= 0.
 */
#define LC_CHANNEL_AUTO (-1)
#define LC_CHANNEL_NONE (-2)

/* Parses "auto", "none" or a channel index (0..BB_CONFIG_MAX_CHAN_NUM-1).
 * Returns 0 and fills *out on success, -1 on anything else. */
int lc_channel_parse(const char* s, int* out);

/* Reads the persisted channel (LC_CHANNEL_AUTO or an index) from the
 * sidecar file next to cfg_path (fixed name "ar8030.channel"). Returns 0
 * and fills *out on success, -1 if no sidecar exists yet or it's
 * unreadable/invalid. */
int lc_channel_load(const char* cfg_path, int* out);

/* Atomically persists chan (LC_CHANNEL_AUTO or an index) to the sidecar
 * file next to cfg_path. Returns 0 on success. */
int lc_channel_save(const char* cfg_path, int chan);

/* BB_GET_CHAN_INFO: fills auto_mode (1 = adaptive), work_chan and
 * chan_num (size of the chip's channel table). Returns 0 on success. */
int lc_channel_read(bb_dev_handle_t* handle, int* auto_mode, int* work_chan, int* chan_num);

/* Non-zero if the chip's reported (auto_mode, work_chan) is what chan
 * asks for -- in auto mode any working channel counts. */
int lc_channel_matches(int chan, int auto_mode, int work_chan);

/* Retunes this radio only: BB_SET_CHAN_MODE, plus BB_SET_CHAN(rx, index)
 * in manual mode. Meant for before a link exists, so both ends (each
 * applying its own persisted value at startup) meet on the same channel.
 * Returns 0 if every ioctl succeeded. */
int lc_channel_apply_local(bb_dev_handle_t* handle, int chan);

/* Retunes both ends of a live link: the local change above, then
 * BB_SET_REMOTE to push the same mode/channel to the peer on `slot` --
 * ar8030-linkctl's own cmd_channel() sequence (the vendor's bb_test.c
 * one), confirmed on hardware to drive a synchronized "safe hop" without
 * dropping CONNECT. Returns 0 if every ioctl succeeded. */
int lc_channel_apply_linked(bb_dev_handle_t* handle, int slot, int chan);

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
