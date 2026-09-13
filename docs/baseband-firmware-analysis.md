# AR8030 baseband firmware analysis — reliability plane, control surface, and gap matrix

Status: **analysis + proposal only.** Everything here was derived from
publicly-downloadable firmware and binaries on a host; **nothing has been
tested against real AR8030 hardware.** The intent is to hand the repo owner a
map of what the closed baseband already provides, which knobs it exposes, and a
concrete phased plan to try on-device.

Method: static analysis of `bb_demo_sky_3v3.img` (AR8030 baseband firmware,
RV32) and `ar_ldyhs_sky` (vendor ARM streamer) from the Caddx/Walksnail Ascent
V18.21.10 update. Tools: GNU objdump 2.44 (`riscv64`/`arm-linux-gnueabihf`),
`ubi_reader`, `binwalk`. No hardware. Treat every "implements/handles" below as
"the code and config for it are present", not "measured active on a link".

## TL;DR

- This transport's own wire format has "no ACK/NACK/FEC" (`common/ar8030_chunk.h`),
  and the code assumes recovery is "whatever the AR8030 baseband does". That
  assumption is *much* stronger than it sounds.
- The baseband **contains a real recovery plane**: LDPC + time interleaving
  (PHY), and HARQ + windowed retransmission + RX combining + TX diversity (MAC),
  plus a retransmission controller with a **configurable window and four
  thresholds**.
- The stock vendor app (`ar_ldyhs_sky`) already drives that retransmission
  controller and uses its retransmission pressure as a **bitrate-control
  signal**. This transport does neither.
- **Verdict: do not add a userspace FEC layer.** Surface the baseband's
  retransmission telemetry and control, and use it to steer bitrate/MCS. The
  best-documented "sketchy" failures so far are in the host-side
  `bb_socket`/SDIO path (the subject of the `kmod/` rewrite), which application
  FEC cannot fix without adding latency and airtime. Whether the *radio* side
  also contributes materially is exactly what Phase 0 measures.

---

## 1. How this was recovered (reproducible on any Linux host)

The AR8030 firmware is not in this repo; it is loaded on-device from
`/lib/firmware`-style storage. It ships inside the public Caddx/Walksnail
Ascent update.

### 1.1 Download the update

Version V18.21.10 is what this analysis used (the same release the `kmod`
comments reference for the vendor `daemon_sdiov12`).

```
# Air unit (the side that runs waybeam + ar8030-transport-tx)
curl -L -o Ascent_H_Sky_18_21_10.img \
  'https://download.walksnail.app/1c009071-a926-4460-a70a-5f40c6fc5b4f/Ascent_H_Sky_18_21_10.img?download'
# Ground unit (also has an AR8030; larger image)
curl -L -o Ascent_G_Gnd_18_21_10.img \
  'https://download.walksnail.app/2f0d16a6-b062-4bf5-9980-9e6b11bf50ff/Ascent_G_Gnd_18_21_10.img?download'
```

The file is an `ASW` flash container (u-boot + FIT + a **UBI** volume). The UBI
image header (`UBI#` magic) sits at file offset `0x3AB3A1`, found by scanning
for the magic:

```
rg -aob 'UBI#' Ascent_H_Sky_18_21_10.img | head      # -> 3847073 == 0x3AB3A1
```

### 1.2 Extract the rootfs

```
# carve from the UBI# magic
dd if=Ascent_H_Sky_18_21_10.img of=sky_ubi.img bs=1 skip=$((0x3AB3A1))

# ubi_reader (pip install ubi_reader) extracts the UBIFS volumes
ubireader_extract_files -o sky_root sky_ubi.img
```

Two UBIFS images come out; the **newer sequence directory** is the current FPV
application partition (`sky_root/<seq>/ubifs/fpv/`, where `<seq>` is the
higher-numbered one; avoid hard-coding it). It contains:

```
fpv/boot_ar8030/boot_ar8030.sh        # insmod artosyn_sdio.ko fw_name=... cfg_name=...
fpv/boot_ar8030/bb_demo_sky_3v3.img   # <-- the AR8030 baseband firmware (428 KB)
fpv/boot_ar8030/bb_demo_sky_cx472.img #     variant for the cx472 board
fpv/boot_ar8030/bb_config_sky.json    # <-- baseband config (PHY/MAC knobs)
fpv/boot_ar8030/bb_config_sky_cx472*.json
fpv/boot_ar8030/artosyn_sdio.ko       # vendor SDIO kernel module (not stripped)
fpv/daemon_sdiov12                    # vendor daemon (ARM, stripped)
fpv/ar_ldyhs_sky                      # vendor streamer (ARM, static SDK, exported dynsym)
```

### 1.3 Carve the baseband firmware (RISC-V)

`bb_demo_sky_3v3.img` uses the vendor `STRU_SPL_HEADER` format parsed by
`artosyn_download_firmware()` in `kmod/artosyn_drv.c`:

| Segment | File offset | Length |
|---|---|---|
| header | 0x0 | 64 B |
| troot | 0x200 | 944 B |
| signature | 0x600 | 16384 B |
| **spl (the firmware)** | **0x4600** | **0x64300** |

The `spl` segment loads at VA `0x204000` and is **RV32 RISC-V** (reset vector
sets `mtvec`; BSS is cleared from `0x268300` to `0x26de00`).

```
dd if=bb_demo_sky_3v3.img of=spl.bin bs=1 skip=$((0x4600)) count=$((0x64300))
riscv64-linux-gnu-objdump -D -b binary -m riscv:rv32 --adjust-vma=0x204000 spl.bin > spl.asm
```

The firmware is **not encrypted** and retains `__func__`-style log strings and
hardware register addresses, so it is very readable in a raw disassembly.

### 1.4 Vendor app / daemon (ARM)

`ar_ldyhs_sky` is an ARM `ET_EXEC` that statically links the AR8030 SDK and
exports dynamic symbols (`bb_ioctl`, `fpv_bb_*`). This is where the
host→firmware RPC opcodes are recoverable.

```
arm-linux-gnueabihf-objdump -d ar_ldyhs_sky > ar_ldyhs_sky.asm
```

Addresses in the first LOAD segment map as `file_offset = VA - 0x10000`; the
code uses PC-relative literal accesses (`ldr rX,[pc]; add rX,pc`), so a small
script (or radare2 with analysis) is needed to resolve string/data xrefs. The
firmware's RPC descriptor table is found by parsing 12-byte
`{u32 opcode, u32 in_size, u32 out_size}` entries at `spl.bin` VA `0x254800`
(file `0x50800`); it has 91 entries.

---

## 2. What the baseband contains

Confirmed by strings, symbols, and config keys in `spl.bin` / `ar_ldyhs_sky`
(mechanism names are name-based inferences unless noted):

| Mechanism | Evidence |
|---|---|
| **LDPC FEC** | `LDPC:`, `ldpc_up_num`/`ldpc_dw_num` in config; `bb_phy_calc_code_len`, `bb_phy_calc_max_byte_length`; status `LDPC(%u, %u) LDPC_CONTI(%d)` |
| **HARQ** | `HARQ RD/WR ERR : %X/%X`; handler reads baseband registers at `0xa110f04`, `0xa110f78`, `0xa110f7c`, `0xa110f88` (`spl.asm` @ `0x21da60`–`0x21da82`) |
| **Windowed retransmission** | `bb_link_node_retx_handle`, `bb_link_retx_ctrl_cfg/feed`, `bb_link_{ap,node}_retx_local_monitor`, `bb_link_retx_evt_stat_cfg_reset`; status `retx count`, `retx max stat`, `window`, `timeout`; per-peer `retx(0x%02x)` |
| **RX combining** | `MRC RD ERR : %X`; per-slot dual RSSI `RSSI(%u,%u)`; config modes `2T2R_STBC`/`2T2R_MIMO` |
| **TX diversity** | config `2TX_STBC`, `2TX_MIMO`; `tx_mode`/`rx_mode` |
| **Time interleaving** | config `enable_tintlv`, `tintlv_num`, `tintlv_len`; `bl - calc byte length, bl [bw] [tintlv_len] [qam] [cr] [rep] [mimo]` |
| **Repetition coding** (inferred from the `rep` term) | same PHY size calculator |
| **Frequency-hop retransmit + rollback** | `blind hop retx`, `ds blind hop retx`, `safe hop roll back!` |
| **Retx-aware power control** | `main power (%d), opt power (%d) retx (%u) %u -> %u` |
| **Adaptive MCS** | `bb_link_mcs_change_timeout`, `bb_phy_do_mcs_req`, `bb_link_mcs_set_req` |

This is consistent with why the stock streamer needs no application-layer ARQ
and only adds a payload checksum.

---

## 3. Recovered host control surface

### 3.1 `bb_ioctl` ABI

```c
int bb_ioctl(void *handle, uint32_t cmd, void *in, void *out);
```

Opcode encoding observed: high byte `0x01` = GET, `0x02` = SET. The firmware
publishes an in/out size for each command (descriptor table at `spl.bin`
VA `0x254800` / file `0x50800`), which is the authoritative way to size the
structs.

### 3.2 Retransmission configuration (the key finding)

| Command | Opcode | Direction | Struct |
|---|---|---|---|
| `BB_SET_RETX_EVENT_STATUS` | `0x02000026` | in | 136 B (`0x88`) |
| `BB_GET_RETX_EVENT_STATUS` | `0x01000014` | out | 136 B (`0x88`) |

**Struct prefix (confirmed from both the vendor's set and get paths):**

```c
struct bb_retx_cfg {          /* 136 bytes total */
    uint8_t win;              /* offset 0: retransmission window */
    uint8_t busy;             /* offset 1 */
    uint8_t idle;             /* offset 2 */
    uint8_t conti_busy;       /* offset 3 */
    uint8_t conti_idle;       /* offset 4 */
    uint8_t reserved[131];
};
```

Evidence: `fpv_bb_init` fills `[win, param0..3]` from the context and calls
`bb_ioctl(handle, 0x02000026, &cfg, NULL)` (`ar_ldyhs_sky.asm` @ `0x68c90`–
`0x68cbe`). The get path prints them with (format string at file `0x18498b`):

```
"win=%d,busy=%d,idle=%d,conti busy=%d,conti idle=%d\n"
```

Vendor defaults when no config is present: **`win=10`, `{busy,idle,conti_busy,
conti_idle}={6,4,2,0}`** (`0x68d18`–`0x68d28`; `conti_idle` is 0 from the
preceding `memset`).

**What the fields drive (firmware side).** The baseband's retransmission
controller is a per-user windowed state machine:

- `win` is the window length; the controller also tracks `timeout` and an
  `enable` flag. Its own status dump prints `enable : %u`,
  `window : %u`, `timeout : %u` (format strings at `spl.bin` VA `0x2628dc`,
  `0x2628ec`, `0x2628fc`; printer @ `0x24b85c`).
- `bb_link_retx_ctrl_feed` (`0x230cc6`) maintains per-user busy/idle bitmasks
  across that window. `busy`/`idle` are the per-window thresholds; the
  `conti_busy`/`conti_idle` names indicate "consecutive" thresholds before the
  controller declares the link in trouble (name-based inference; the state
  machine uses `1 << n` bitmask ops, consistent with windowed counting).
- `bb_link_retx_evt_stat_cfg_reset` (`0x22f74c`) clears a per-user event/stat
  block (`[0]=0xffffffff`, `[1]=0`, `[2]=0`).
- `bb_link_node_retx_handle` and `bb_link_{ap,node}_retx_local_monitor` consume
  that state to decide actual retransmissions.

This is consistent with the status strings `retx count`, `retx max stat`,
`peer slot ... retx(0x%02x)`, `retx req enabled/disabled locally`, and
`user %u retx hold!`. In short: `retx_win`/`retx_param` tune *how aggressively
the baseband detects a bad link and retransmits*, not whether retransmission
happens at all.

The vendor app configures this from a debug JSON file on the device, read at
startup (`fpv_debug_cfg_read_all` @ `0x60308`):

```
/factory/fpv_debug_cfg.json
{
  "mcs_throughput": [ ... ],   /* up to 15 entries */
  "use_dbg_cjson": 0,
  "retx_disable": 0,           /* key is `retx_disable`; log label is `retx_det_disable` */
  "retx_win": 10,
  "retx_param": [6, 4, 2, 0]   /* busy, idle, conti_busy, conti_idle */
}
```

**What `retx_disable` actually does** (correcting a plausible-but-wrong
assumption): it does **not** turn off baseband retransmission. The value is
stored at gctx+0xc4 and its **only** reader is `fpv_bb_is_send_retx_too_many()`
(`0x6a696`, returns 0 when nonzero). It therefore suppresses the *host-side
"retx too many" bitrate-backoff trigger*, not the recovery plane. It is an A/B
switch for the rate controller, not for retransmission.

> Runtime caveat: the vendor only calls `BB_SET_RETX_EVENT_STATUS` once at
> `fpv_bb_init`. Whether a live re-SET after boot actually takes effect is
> unverified — confirm before building on it (Phase 1).

> `bb_api.h` may or may not declare these names, but the names do appear as
> vendor log strings and the vendor app calls them via `bb_ioctl`; if the SDK
> header lacks them, use the numeric opcodes above.

### 3.3 Command descriptor table (recovered subset)

Full table is 91 entries; the ones relevant to link integrity and control:

```
GET:
  0x01000000  ->  0x012c   BB_GET_STATUS (link state, per-slot mcs/bw/freq)
  0x01000006  ->  0x0008   BB_GET_MCS (already used by tx/bitrate_ctl.c)
  0x0100000a  ->  0x0404   BB_GET_CHAN_INFO
  0x0100000c  ->  0x0004   BB_GET_AP_TIME
  0x01000011  ->  0x0288   ring-buffer left-size (host backlog)
  0x01000012  ->  0x0081   get work channel list
  0x01000014  ->  0x0088   BB_GET_RETX_EVENT_STATUS
  0x01000064  ->  0x0100   read BB register
  0x01000071  ->  0x0004   get power offset
SET:
  0x02000003  in 0x0004    set peer MAC
  0x02000004  in 0x0192    set peer candidate MAC
  0x02000005  in 0x0001    BB_SET_CHAN_MODE
  0x02000006  in 0x0002    BB_SET_CHAN
  0x02000018  in 0x0003    set RF
  0x02000023  in 0x0081    set work channel list
  0x02000024  in 0x0010    BB_SET_MCS_ITEM (16-byte MCS entry)
  0x02000026  in 0x0088    BB_SET_RETX_EVENT_STATUS
  0x02000064  in 0x0104    write BB register
  0x0200006e  in 0x0004    set power offset
  0x020000c8  in 0x0100    project/PRJ dispatch
```

### 3.4 Telemetry that already exists

- **Firmware-side status strings** (`spl.bin`): `retx count : %u`,
  `retx max stat : %u`, `window : %u`, `timeout : %u`, `LDPC(%u, %u)
  LDPC_CONTI(%d)`, per-peer `retx(0x%02x)` (an 8-bit value; "bitmap" is an
  inference), `RSSI(%u,%u)`.
- **Host-side counters** (`ar_ldyhs_sky`): `retx,busy flag,[%d,%d,%d],check cnt
  [%u,%u]`, `ReTX User %u  : %u %u %u`, `Retx(R|T)`, and
  `fpv_bb_is_send_retx_too_many()`.

### 3.5 The vendor's own use of retx as a control signal

`fpv_video_buffer_cache_monitor()` calls `fpv_bb_is_send_retx_too_many()` and,
when true, calls `fpv_video_set_venc_bitrate()` — logging
`send_retx_too_many!, flag_set_kbps=%d`. That is a **radio-derived congestion
signal** this repo's `tx/bitrate_ctl.c` does not have (it currently uses only
local `BB_GET_MCS` + the frame-shm low-water mark). It is present in the vendor
stack; its *effect on this specific link* is untested and is what Phase 2
measures.

### 3.6 Baseband config (`bb_config_sky.json`) — integrity-relevant knobs

- **PHY/robustness (shipped users: `br` and `slot`):** `bandwidth`,
  `enable_tintlv`, `tintlv_num`, `tintlv_len`, `tx_mode`/`rx_mode`
  (`1TX`/`2TX_STBC`/`2TX_MIMO`/`2T2R_STBC`/`2T2R_MIMO`), `pre_encode`,
  **`retx_count`** (br=100, slot=6), `fch_info_len`.
  (Note: the shipped `br` user is already `2TX_STBC`/`2T2R_STBC`; only `slot`
  is `1TX`/`1T1R`.)
- **MCS/LDPC table:** per entry `{mcs, snr_up, snr_dw, ldpc_up_num,
  ldpc_dw_num, up_keep_time, dw_keep_time}`; per user `{enable, mode, init,
  hold_time, max_wait_time}`. (`ldpc_dw_conti_num` exists as a firmware string
  but is **not** a key in the shipped config.)
- **Channel:** `subchan.{main_bw,sub_bw,chan_num,offset}`, `fs_bw`,
  `auto_band.*` (this is **2G/5G band selection**, not bandwidth widening),
  `br_hop.*`, `rc_hop.*`, `multi_mode.hop_para.{retx_cnt,snr_min,gain_max,
  power_diff,multi_pwr_diff,chan_inr,hop_mode}`.
- **Power:** `power.{mode,pwr_init,pwr_auto,pwr_range}`,
  `pwr_auto.<mod>.*` (gain_min/max, up/down_thresh, pwr_min/max, steps),
  `pwr_auto.strategy.*`, `power_calibration.*`.
- **LNA:** `lna.inner.*`, `lna.fem.{on_thred,off_thred,on_must_thred,
  on_fs_thred,off_fs_thred,keep}`.

---

## 4. Gap matrix

| # | Area | This repo today | Baseband reality | Gap / risk | Proposed action |
|---|---|---|---|---|---|
| 1 | Recovery plane | App has no ACK/NACK/FEC; assumes baseband | Contains LDPC + interleave + HARQ + retx + combining + TX diversity | None functionally — but zero visibility/control | Do **not** add app FEC; surface baseband retx telemetry/control |
| 2 | Retx control | Fixed/opaque | Window + 4 thresholds, settable via `0x02000026`; vendor uses `retx_win`/`retx_param` | Link never tuned for the bench/deploy link | Add `linkctl retx`; A/B vendor defaults |
| 3 | Rate-control input | Local `BB_GET_MCS` + ring low-water | Retx pressure (`send_retx_too_many`) + LDPC counts | Controller blind to radio repair pressure | Feed retx/LDPC into `bitrate_ctl`; add retx backoff (Phase 2) |
| 4 | MCS policy | `BB_SET_MCS`/mode | Full LDPC/SNR threshold table per user | Thresholds may not match this link | Tune `snr_*`/`ldpc_*` table |
| 5 | Diversity | This repo's own TX/RX config is separate; baseband `br` already STBC, `slot` not | `2TX_STBC`/`2T2R_STBC`/MIMO selectable | Possibly leaving diversity unused on the `slot` user | Confirm which user the video path uses, then set `tx_mode`/`rx_mode` |
| 6 | Interleaving | Not considered | `enable_tintlv`, `tintlv_num`, `tintlv_len` | Burst-loss resilience may be reduced | Tune `tintlv_*` |
| 7 | Telemetry | `-v` counters (frames, CRC16, resync) | Per-peer retx, LDPC counts, window/timeout | Cannot see *why* the radio is dropping | Surface `BB_GET_RETX_EVENT_STATUS` + per-peer stats |
| 8 | Bandwidth auto-widen | Client `bw_auto` "req 5 not found"; manual `BB_SET_BANDWIDTH` works | No `bw_auto` knob in the shipped config; `auto_band.*` is 2G/5G **band** selection | Auto-widen unavailable via current client call | Keep manual `BB_SET_BANDWIDTH`; treat auto-widen as an SDK/client gap, not a config knob |
| 9 | Host path (`bb_socket`/SDIO) | The best-documented stalls/corruption | Not addressed by the baseband | Likely the dominant "sketchy" cause | Continue `kmod/` rewrite + watchdog |
| 10 | App-layer FEC | Deferred to "Phase 2" (`ar8030_chunk_hdr.reserved`) | Already provided below (HARQ/LDPC/retx) | Redundant latency + airtime if built blindly | Defer; gate on measured residual frame loss |
| 11 | Retx backoff A/B | Not present | `retx_disable` suppresses only the host-side backoff trigger | Easy to mis-test the wrong thing | Use `retx_disable` to A/B the rate-control input, not the retx plane |

---

## 5. Proposed follow-up plan (owner, on hardware)

Each phase is independently useful and safe to stop after. Phase 0 establishes
the numbers every later phase is judged against.

### Phase 0 — baseline measurement (read-only)
1. On a known-good boot, log `BB_GET_STATUS`, `BB_GET_MCS`, and
   `BB_GET_RETX_EVENT_STATUS`, plus per-peer retx/LDPC/RSSI, during a real
   session.
2. Record the **residual frame-loss/corruption rate** and whether retx pressure
   is actually elevated. This is what decides whether the radio side (vs. the
   host path) needs attention.

### Phase 1 — telemetry plumbing
1. Add a `linkctl retx [--set win p0 p1 p2 p3]` subcommand using
   `0x02000026`/`0x01000014` (struct above). First verify a live SET after boot
   actually changes the GET output (the vendor only sets at init).
2. Feed retx/LDPC into `tx/bitrate_ctl.c` so the controller can see the radio's
   repair pressure, not just MCS and host backlog.

### Phase 2 — retx-driven bitrate backoff
Mirror the vendor (`send_retx_too_many!` → reduce venc bitrate). This targets
the **radio-side** pressure signal; expect it to help only if Phase 0/1 show
elevated retx. It is not a fix for host-side `bb_socket` stalls.

### Phase 3 — config sweep (one variable at a time, measure each)
1. `retx_win` / `retx_param` via `/factory/fpv_debug_cfg.json` (start from
   vendor default `10,[6,4,2,0]`).
2. `tx_mode`/`rx_mode` → `2TX_STBC`/`2T2R_STBC` on the active user;
   `retx_count`; `tintlv_*`.
3. MCS/LDPC thresholds (`snr_up/dw`, `ldpc_up_num/dw_num`).
For each: record loss/corruption rate and retx/LDPC counts before/after, and
keep only changes that move the target metric.

### Phase 4 — decide on app FEC (measurement-gated)
Only if, after Phases 1–3 and the host-path fixes, frames are still lost where
HARQ/retx is exhausted under deep fades **and** there is latency budget, then
add a thin adaptive last-resort FEC. Do not build it speculatively.

### Safety / method notes
- Always test from a **clean boot**; the AR8030 is sensitive to
  cross-module-reload contamination (already documented in the README).
- `tx_mode`/`rx_mode` must match on **both** ends; a mismatch can kill the link.
- Back up the original `bb_config_sky.json` and any debug JSON before editing,
  and keep the JSON parseable — the app reads it during init and a malformed
  file can prevent bring-up. Have a serial/recovery path ready before a config
  edit that could block boot.
- Confirm `/factory` is writable/mounted as expected on the target before
  relying on `fpv_debug_cfg.json`; the boot script itself references
  `/usrdata/fpv/boot_ar8030/`.

---

## 6. Known unknowns / next RE work

- **Full 136-byte record beyond the first 5 bytes** — only the 5-byte config
  prefix is consumed by the stock app. The same 136-byte size appears on several
  commands (`0x01000014`, `0x01000015`, `0x02000015`, `0x02000026`), so it is a
  shared per-user config/status record. The firmware's own builder for
  `0x02000015` zeroes 136 bytes and then sets byte 0, a word at offset 4, and
  bytes at offsets 13–14 (`spl.asm` @ `0x24788c`), but the full map is not
  statically pinned. Recover it by dumping `BB_GET_RETX_EVENT_STATUS` on
  hardware and diffing while changing `retx_win`/`retx_param`, MCS, and link
  state (procedure in Appendix C).
- **Exact `win`/`busy`/`idle`/`conti_*` units** — the controller is windowed and
  tracks `enable`/`window`/`timeout`; the thresholds are windowed
  busy/idle/consecutive counts. Exact units (frames? windows?) still need a
  hardware A/B.
- **`retx_count` (config) vs `retx_win`/`retx_param` (debug JSON)** — likely
  different layers (per-user max retransmissions vs the controller window).
- Names for the remaining ~80 opcodes in the 91-entry table (only the vendor
  app's call sites are currently named).

---

## Appendix A — reproduction cheat-sheet

```
# 1. firmware update
curl -L -o sky.img 'https://download.walksnail.app/1c009071-a926-4460-a70a-5f40c6fc5b4f/Ascent_H_Sky_18_21_10.img?download'
# 2. locate + carve the UBI image ('UBI#' magic)
rg -aob 'UBI#' sky.img                       # -> offset (0x3AB3A1 for V18.21.10)
dd if=sky.img of=sky_ubi.img bs=1 skip=$((0x3AB3A1))
ubireader_extract_files -o sky_root sky_ubi.img   # use the newer <seq>/ubifs/fpv/
# 3. carve RISC-V firmware and disassemble
dd if=sky_root/<seq>/ubifs/fpv/boot_ar8030/bb_demo_sky_3v3.img of=spl.bin bs=1 skip=$((0x4600)) count=$((0x64300))
riscv64-linux-gnu-objdump -D -b binary -m riscv:rv32 --adjust-vma=0x204000 spl.bin > spl.asm
# 4. vendor app
arm-linux-gnueabihf-objdump -d sky_root/<seq>/ubifs/fpv/ar_ldyhs_sky > app.asm
# 5. opcode table (12-byte entries) at spl VA 0x254800 (file 0x50800)
xxd -s $((0x50800)) -l 0x48c spl.bin
```

## Appendix B — retx config, ready to wire

```c
/* Opcodes recovered from the vendor app + firmware descriptor table.
 * If bb_api.h does not declare these names, use the numeric values. */
#define BB_SET_RETX_EVENT_STATUS 0x02000026u   /* in,  sizeof(struct bb_retx_cfg) */
#define BB_GET_RETX_EVENT_STATUS 0x01000014u   /* out, sizeof(struct bb_retx_cfg) */

struct bb_retx_cfg {
    uint8_t win;         /* retransmission window */
    uint8_t busy;
    uint8_t idle;
    uint8_t conti_busy;
    uint8_t conti_idle;
    uint8_t reserved[131];
};

/* Vendor defaults: win=10, {busy,idle,conti_busy,conti_idle}={6,4,2,0}. */
struct bb_retx_cfg cfg = { .win = 10, .busy = 6, .idle = 4, .conti_busy = 2, .conti_idle = 0 };
bb_ioctl(dev, BB_SET_RETX_EVENT_STATUS, &cfg, NULL);

struct bb_retx_cfg st = {0};
bb_ioctl(dev, BB_GET_RETX_EVENT_STATUS, NULL, &st);
/* st.win, st.busy, st.idle, st.conti_busy, st.conti_idle */
```

## Appendix C — dumping/annotating the 136-byte retx record (hardware)

The first 5 bytes are known; the rest is unmapped. To annotate it, dump and
diff the record under controlled changes:

```c
struct bb_retx_cfg st = {0};
if (bb_ioctl(dev, 0x01000014, NULL, &st) == 0) {
    for (int i = 0; i < (int)sizeof(st); i += 16) {
        fprintf(stderr, "%04x:", i);
        for (int j = 0; j < 16; j++) fprintf(stderr, " %02x", ((uint8_t *)&st)[i + j]);
        fprintf(stderr, "\n");
    }
}
```

Suggested changes to diff against:
1. `retx_win` = 1, 10, 50, 255.
2. `retx_param` = vendor default `{6,4,2,0}`, then `{0,0,0,0}`, then a large
   value.
3. Move the craft in/out of range / force reconnects to make retransmission
   actually happen.
4. Cross-check against the firmware's own status fields (`enable`, `window`,
   `timeout`, `retx count`, `retx max stat`, per-peer `retx(0x%02x)`), which are
   printed by the stock stack and should correspond to bytes in the record.

This is the concrete next step for turning the opaque 131 bytes into a readable
per-user retransmission status block.

## Appendix D — firmware-side retx functions (RV32 anchors)

Useful entry points if you want to keep decompiling in Ghidra (RISC-V):

| Function | VA (`spl.bin`, load base `0x204000`) |
|---|---|
| `bb_link_retx_ctrl_cfg` (inlined; log @ `0x246ebe`) | `0x246e6c` |
| `bb_link_retx_ctrl_feed` | `0x230cc6` |
| `bb_link_retx_evt_stat_cfg_reset` | `0x22f74c` |
| retx status printer (`enable`/`window`/`timeout`/`offset`) | `0x24b85c` |
| RPC dispatcher (jump table via `sh2add` + `jr`) | `0x244c8c` |
| descriptor table (`{op,in,out}`, 91 entries) | `0x254800` (file `0x50800`) |
