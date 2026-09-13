# AR8030 baseband firmware analysis — reliability plane, control surface, and gap matrix

Status: **analysis + proposal only.** Everything here was derived from
publicly-downloadable firmware and binaries on a host; **nothing has been
tested against real AR8030 hardware.** The intent is to hand the repo owner a
map of what the closed baseband already does, which knobs it exposes, and a
concrete phased plan to try on-device.

## TL;DR

- This transport's own wire format has "no ACK/NACK/FEC" (`common/ar8030_chunk.h`),
  and the code correctly assumes recovery is "whatever the AR8030 baseband
  does". That assumption is *much* stronger than it sounds.
- The AR8030 baseband **is** the recovery plane: **LDPC + time interleaving +
  repetition coding** (PHY), **HARQ + windowed retransmission + MRC + STBC/MIMO**
  (MAC), plus a retransmission controller with **configurable window and four
  thresholds**.
- The stock vendor app (`ar_ldyhs_sky`) **already drives that retransmission
  controller** and uses its retransmission pressure as a **bitrate-control
  signal**. This transport does neither.
- **Verdict: do not add a userspace FEC layer.** Instead, surface the baseband's
  retransmission telemetry and control, and use it to steer bitrate/MCS. The
  observed "sketchy" behavior is dominated by the host-side `bb_socket`/SDIO
  path (the subject of the `kmod/` rewrite), which no application FEC can fix
  without adding latency and airtime.

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

The file is an `ASW` flash container: `u-boot.bin` + FIT image + a **UBI**
volume. The UBI erase-count header is at file offset `0x3AB3A1` in the Sky
image.

### 1.2 Extract the rootfs

```
# carve the UBI image
dd if=Ascent_H_Sky_18_21_10.img of=sky_ubi.img bs=1 skip=$((0x3AB3A1))

# ubi_reader (pip install ubi_reader) extracts the UBIFS volumes
ubireader_extract_files -o sky_root sky_ubi.img
```

Two UBIFS images come out; the newer one (`sky_root/<seq>/ubifs/fpv/`) is the
FPV application partition. It contains:

```
fpv/boot_ar8030/boot_ar8030.sh        # insmod artosyn_sdio.ko fw_name=... cfg_name=...
fpv/boot_ar8030/bb_demo_sky_3v3.img   # <-- the AR8030 baseband firmware (428 KB)
fpv/boot_ar8030/bb_demo_sky_cx472.img #     variant for the cx472 board
fpv/boot_ar8030/bb_config_sky.json    # <-- baseband config (all the PHY/MAC knobs)
fpv/boot_ar8030/bb_config_sky_cx472*.json
fpv/boot_ar8030/artosyn_sdio.ko       # vendor SDIO kernel module (not stripped)
fpv/daemon_sdiov12                    # vendor daemon (ARM, stripped)
fpv/ar_ldyhs_sky                      # vendor streaming app (ARM, static SDK, has symbols)
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
sets `mtvec`, clears BSS to `0x69e00`).

```
dd if=bb_demo_sky_3v3.img of=spl.bin bs=1 skip=$((0x4600)) count=$((0x64300))
riscv64-linux-gnu-objdump -D -b binary -m riscv:rv32 --adjust-vma=0x204000 spl.bin > spl.asm
```

The firmware is **not encrypted** and retains `__func__`-style log strings and
hardware register addresses, so it is very readable in a raw disassembly.

### 1.4 Vendor app / daemon (ARM)

`ar_ldyhs_sky` is an ARM `ET_EXEC` that statically links the AR8030 SDK and
exports symbols (`bb_ioctl`, `fpv_bb_*`). This is where the host→firmware RPC
opcodes are recoverable.

```
arm-linux-gnueabihf-objdump -d ar_ldyhs_sky > ar_ldyhs_sky.asm
```

Addresses in the first LOAD segment map as `file_offset = VA - 0x10000`; the
code uses PC-relative literal accesses (`ldr rX,[pc]; add rX,pc`), so a small
script (or radare2 with analysis) is needed to resolve string/data xrefs. The
firmware's RPC descriptor table is found by parsing 12-byte
`{u32 opcode, u32 in_size, u32 out_size}` entries at `spl.bin` VA `0x254800`.

---

## 2. What the baseband actually implements

Confirmed by strings and code in `spl.bin`:

| Mechanism | Evidence |
|---|---|
| **LDPC FEC** | `LDPC:`, `ldpc_up_num`/`ldpc_dw_num`/`ldpc_dw_conti_num` in config; `bb_phy_calc_code_len`, `bb_phy_calc_max_byte_length`; status `LDPC(%u/%u) LDPC_CONTI(%d)` |
| **HARQ** | `HARQ RD/WR ERR : %X/%X`; handler polls hw regs at `0xa110f04/f78/f80/ffc` |
| **Windowed retransmission** | `bb_link_node_retx_handle`, `bb_link_retx_ctrl_cfg/feed`, `bb_link_{ap,node}_retx_local_monitor`, `bb_link_retx_evt_stat_cfg_reset`; status `retx count`, `retx max stat`, `window : %u`, `timeout : %u`; per-peer `retx(0x%02x)` |
| **MRC RX combining** | `MRC RD ERR : %X`; per-slot dual RSSI `RSSI(%u,%u)`; modes `2T2R_STBC`/`2T2R_MIMO` |
| **TX diversity** | `2TX_STBC`, `2TX_MIMO`; config `tx_mode`/`rx_mode` |
| **Time interleaving** | config `enable_tintlv`, `tintlv_num`, `tintlv_len`; `bl - calc byte length, bl [bw] [tintlv_len] [qam] [cr] [rep] [mimo]` |
| **Repetition coding** | the `rep` term in the same PHY size calculator |
| **Frequency-hop retransmit + rollback** | `blind hop retx`, `ds blind hop retx`, `safe hop roll back!` |
| **Retx-aware power control** | `main power (%d), opt power (%d) retx (%u) %u -> %u` |
| **Adaptive MCS** | `bb_link_mcs_change_timeout`, `bb_phy_do_mcs_req`, `bb_link_mcs_set_req` |

This is why the stock streamer needs no application-layer ARQ and only adds a
payload checksum — the heavy lifting is below it.

---

## 3. Recovered host control surface

### 3.1 `bb_ioctl` ABI

```c
int bb_ioctl(void *handle, uint32_t cmd, void *in, void *out);
```

Opcode encoding observed: high byte `0x01` = GET, `0x02` = SET, low bits =
command. The firmware publishes an in/out size for each command (descriptor
table at `spl.bin` VA `0x254800`), which is the authoritative way to size the
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
`bb_ioctl(handle, 0x02000026, &cfg, NULL)`. The get path prints them with:

```
"win=%d,busy=%d,idle=%d,conti busy=%d,conti idle=%d\n"
```

Vendor defaults when no config is present: **`win=10`, `{busy,idle,conti_busy,
conti_idle} = {6,4,2,0}`**.

The vendor app configures this from a debug JSON file on the device:

```
/factory/fpv_debug_cfg.json
{
  "retx_disable": 0,          /* read as "retx_det_disable=%d" */
  "retx_win": 10,
  "retx_param": [6, 4, 2, 0],  /* busy, idle, conti_busy, conti_idle */
  "mcs_throughput": [ ... ],   /* <=15 entries */
  "use_dbg_cjson": 0
}
```

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

> `bb_api.h` may not declare names for the retx opcodes; the numeric values
> above work with the same `bb_ioctl()` this repo already calls (the vendor app
> calls them numerically).

### 3.4 Telemetry the firmware/host already holds

- Per-peer: `retx(0x%02x)` (8-bit retransmission bitmap), `LDPC(%u/%u)`,
  `RSSI(%u,%u)`, `SNR`.
- Retx status: `retx count`, `retx max stat`, `window`, `timeout`.
- Host-side retx pressure: `fpv_bb_is_send_retx_too_many()` and counters
  (`retx count      : %u`, `retx max stat   : %u`, busy flags).

### 3.5 The vendor's own use of retx as a control signal

`fpv_video_buffer_cache_monitor()` calls `fpv_bb_is_send_retx_too_many()` and,
when true, calls `fpv_video_set_venc_bitrate()` — logging
`send_retx_too_many!, flag_set_kbps=%d`. That is a **radio-derived congestion
signal** the repo's `tx/bitrate_ctl.c` does not have (it currently uses only
local `BB_GET_MCS` + the frame-shm low-water mark).

### 3.6 Baseband config (`bb_config_sky.json`) — integrity-relevant knobs

- **PHY/robustness (per user `br`/`slot`/`slot0..7`):** `bandwidth`,
  `enable_tintlv`, `tintlv_num`, `tintlv_len`, `tx_mode`/`rx_mode`
  (`1TX`/`2TX_STBC`/`2TX_MIMO`/`2T2R_STBC`/`2T2R_MIMO`), `pre_encode`,
  **`retx_count`** (br=100, slot=6), `fch_info_len`.
- **MCS/LDPC table:** per entry `{mcs, snr_up, snr_dw, ldpc_up_num,
  ldpc_dw_num, ldpc_dw_conti_num, up_keep_time, dw_keep_time}`; per user
  `{enable, mode, init, hold_time, max_wait_time}`.
- **Channel:** `subchan.{main_bw,sub_bw,chan_num,offset}`, `fs_bw`,
  `auto_band.{snr_thred,scan_count,scan_inr,round_inr,5g_to_2g,2g_to_5g}`,
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
| 1 | Recovery plane | App has no ACK/NACK/FEC; assumes baseband | LDPC + interleave + HARQ + retx + MRC + STBC | None functionally — but zero visibility/control | Do **not** add app FEC; surface baseband retx telemetry/control |
| 2 | Retx control | Fixed/opaque | Window + 4 thresholds, settable via `0x02000026`; vendor uses `retx_win`/`retx_param` | Link never tuned for the bench/deploy link | Add `linkctl retx`; A/B vendor defaults |
| 3 | Rate control input | Local `BB_GET_MCS` + ring low-water | Retx pressure (`send_retx_too_many`) + LDPC counts | Rate controller blind to radio repair pressure | Feed retx/LDPC into `bitrate_ctl`; add retx backoff |
| 4 | MCS policy | `BB_SET_MCS`/mode | Full LDPC/SNR threshold table per user | Thresholds may not match this link | Tune `snr_*`/`ldpc_*` table |
| 5 | Diversity | Effectively single-path default | `2TX_STBC`/`2T2R_STBC`/MIMO selectable | Leaving diversity unused | Set `tx_mode`/`rx_mode`; measure |
| 6 | Interleaving | Not considered | `enable_tintlv`, `tintlv_num`, `tintlv_len` | Burst-loss resilience may be reduced | Tune `tintlv_*` |
| 7 | Telemetry | `-v` counters (frames, CRC16, resync) | Per-peer retx bitmap, LDPC counts, window/timeout | Cannot see *why* the radio is dropping | Surface `BB_GET_RETX_EVENT_STATUS` + per-peer stats |
| 8 | Bandwidth auto | Client `bw_auto` "req 5 not found"; manual `BB_SET_BANDWIDTH` works | Firmware has auto-band machinery + config `auto_band.*` | Auto-widen unusable via current client call | Use config `auto_band.*`, or another opcode; else set manually |
| 9 | Host path (`bb_socket`/SDIO) | The actual observed stalls/corruption | Not addressed by the baseband | **The real "sketchy"** | Continue `kmod/` rewrite + watchdog |
| 10 | App-layer FEC | Deferred to "Phase 2" (`ar8030_chunk_hdr.reserved`) | Already provided below (HARQ/LDPC/retx) | Redundant latency + airtime if built blindly | Defer; gate on measured residual frame loss |

---

## 5. Proposed follow-up plan (owner, on hardware)

Each phase is independently useful and safe to stop after.

### Phase 0 — baseline measurement (read-only)
1. On a known-good DRV/SDIO boot, read `BB_GET_STATUS`, `BB_GET_MCS`, and add
   `BB_GET_RETX_EVENT_STATUS` to a throwaway probe.
2. Log per-peer `retx(0x%02x)`, `LDPC(%u/%u)`, `RSSI(%u,%u)` and the
   `win/busy/idle/conti_*` values alongside `-v` stats during a real session.
3. Establish the **residual frame-loss / corruption rate** now, before any
   tuning, so every later change is measured against it.

### Phase 1 — telemetry plumbing
1. Add a `linkctl retx [--set win p0 p1 p2 p3]` subcommand using
   `0x02000026`/`0x01000014` (the struct above).
2. Feed retx/LDPC into `tx/bitrate_ctl.c` so the controller can see the radio's
   own repair pressure, not just MCS and host backlog.

### Phase 2 — retx-driven bitrate backoff
Mirror the vendor: on sustained retx pressure, reduce the venc bitrate
(`send_retx_too_many!` behavior). This is the single most likely fix for
"sketchy" video and is proven by the stock app.

### Phase 3 — config sweep (one variable at a time)
1. `retx_win` / `retx_param` via `/factory/fpv_debug_cfg.json` (start from
   vendor default `10,[6,4,2,0]`).
2. `tx_mode`/`rx_mode` → `2TX_STBC` / `2T2R_STBC`; `retx_count`; `tintlv_*`.
3. MCS/LDPC thresholds (`snr_up/dw`, `ldpc_up_num/dw_num`).

### Phase 4 — decide on app FEC (measurement-gated)
Only if, after Phases 1–3 and the host-path fixes, frames are still lost where
HARQ/retx is exhausted under deep fades **and** there is latency budget, then
add a thin adaptive last-resort FEC. Do not build it speculatively.

### Safety / method notes
- Always test from a **clean boot**; the AR8030 is sensitive to
  cross-module-reload contamination (already documented in the README).
- Back up and restore `/factory/fpv_debug_cfg.json` and any modified
  `bb_config_sky.json`.
- `retx_disable: 1` is a useful A/B control (disables the retx plane) — expect
  a large regression if the plane is doing its job.

---

## 6. Known unknowns / next RE work

- **Full 136-byte GET struct beyond the first 5 bytes** — only the 5-byte
  config prefix is consumed by the stock app; the remainder is opaque. Decode
  by diffing `BB_GET_RETX_EVENT_STATUS` output on hardware under known
  conditions.
- **Exact units/meaning of `win` and the 4 thresholds** — names are known
  (`win`, `busy`, `idle`, `conti_busy`, `conti_idle`) but not the units or the
  state machine they feed. Trace `bb_link_retx_ctrl_cfg` in the firmware (RV32)
  to finish this.
- **`retx_count` (config) vs `retx_win`/`retx_param` (debug JSON)** — likely
  different layers (per-user max retransmissions vs the controller window).
- The rest of the 91-entry opcode table needs names mapped (only the vendor
  app's call sites are currently named).

---

## Appendix A — reproduction cheat-sheet

```
# 1. firmware update
curl -L -o sky.img 'https://download.walksnail.app/1c009071-a926-4460-a70a-5f40c6fc5b4f/Ascent_H_Sky_18_21_10.img?download'
# 2. UBI + UBIFS
dd if=sky.img of=sky_ubi.img bs=1 skip=$((0x3AB3A1))
ubireader_extract_files -o sky_root sky_ubi.img
# 3. carve RISC-V firmware and disassemble
dd if=sky_root/*/ubifs/fpv/boot_ar8030/bb_demo_sky_3v3.img of=spl.bin bs=1 skip=$((0x4600)) count=$((0x64300))
riscv64-linux-gnu-objdump -D -b binary -m riscv:rv32 --adjust-vma=0x204000 spl.bin > spl.asm
# 4. vendor app
arm-linux-gnueabihf-objdump -d sky_root/*/ubifs/fpv/ar_ldyhs_sky > app.asm
# 5. opcode table (12-byte entries) at spl VA 0x254800 (file 0x50800)
xxd -s $((0x50800)) -l 0x48c spl.bin
```

## Appendix B — retx config, ready to wire

```c
/* Opcodes recovered from the vendor app + firmware descriptor table. */
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
