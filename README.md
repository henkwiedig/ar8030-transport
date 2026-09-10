# ar8030-transport

Video bridge between [waybeam](../waybeam_venc) (the OpenIPC FPV video
encoder) and [PixelPilot_rk](../PixelPilot_rk) (the ground-station
decoder/OSD) that rides the AR8030 link module's **non-IP baseband data
channel** (`bb_socket`, the same "xdata" pass-through the vendor SDK's
`xdata_test`/`bw_update_demo` demos exercise) instead of the AR8030's IP
link.

Two standalone binaries, one per end of the link:

- **`ar8030-transport-tx`** (`tx/`) — runs on the **air unit** (AR8030 in
  **AP** role) next to `waybeam` and `ar8030d`. Attaches to waybeam's
  `frame-shm://` ring, splits each encoded H.265 access unit into chunks,
  and writes them to a `bb_socket`. A second thread watches the AR8030's
  own local TX MCS/throughput and throttles waybeam's live bitrate over
  loopback HTTP so the encoder doesn't outrun the radio.
- **`ar8030-transport-rx`** (`rx/`) — runs on the **ground station**
  (AR8030 in **DEV** role) next to `pixelpilot` and `ar8030d`. Reads
  chunks off its `bb_socket`, reassembles whole frames, RTP/H.265-
  packetizes them, and sends them over UDP to PixelPilot's existing RTP
  listener. **No PixelPilot_rk source changes** — point a stock
  `pixelpilot -p 5600` (its own default) at this tool's output.

## Why not the AR8030's IP link?

waybeam's `frame-shm://` ring hands over whole encoded access units with
IDR/GDR/enhance-layer metadata already attached (see
`third_party/waybeam_frame_ring/`), which is the natural unit to protect
and prioritize on a lossy RF link — exactly what a link layer wants
before it gets fragmented into transport packets, not after. Riding
`bb_socket` directly keeps that frame boundary all the way across the
link; going out over IP and back would mean re-deriving it from RTP
sequence numbers on the other side, plus paying for a full IP/UDP stack
(and its own buffering/backpressure behavior) on a link this
bandwidth-constrained.

## Protocol

See `common/ar8030_chunk.h` for the authoritative struct. Summary: each
encoded frame becomes one or more `bb_socket_write()` calls, each
prefixed with an 18-byte header (magic, frame sequence, chunk index/count,
IDR/GDR/enhance flags, codec, capture pts). No ACK, no NACK, no FEC in
this version — a chunk lost on the radio makes its frame incomplete, and
`ar8030-transport-rx` drops the whole frame and moves on to the next one
rather than waiting (see "Phase 2" below for where FEC would slot in).

**Reassembly ordering.** `bb_socket` (in `BB_SOCK_FLAG_DATAGRAM` mode,
which both apps use) is a queued point-to-point channel, not a
packet-switched network — there is no equivalent of IP routing/multipath
that could reorder two writes from the same sender. `rx/main.c`'s
reassembly therefore assumes chunks arrive in the order they were sent
and treats any violation of that (an out-of-order `chunk_idx`, or a new
`frame_seq` starting before the previous one finished) as data loss:
drop what was collected and wait for the next frame's first chunk.

**Chunk size vs. RTP MTU.** These are two independent numbers:
`ar8030-transport-tx -c` controls how a whole encoded frame is split for
the `bb_socket` hop; `ar8030-transport-rx -M` controls how the
*reassembled* frame is re-split into RTP/FU-A packets for the UDP hop to
PixelPilot. There's no reason they need to match, and in practice they
won't — `-c` is sized for AR8030 throughput, `-M` for a conventional
Ethernet/Wi-Fi-range UDP MTU.

## Required AR8030 SDK fix: datagram-mode symmetry

Both apps open their `bb_socket` with `BB_SOCK_FLAG_DATAGRAM` so that one
`bb_socket_write()` reliably becomes one `bb_socket_read()` on the other
end, preserving this project's chunk framing. The upstream
`app/ar8030/session_socket.c` in `yz_host_drv` has a bug where the
socket-open RPC always strips that flag before telling the local chip,
regardless of what the caller asked for — see
`sbc-groundstations/package/ar8030/0008-session_socket-preserve-datagram-flag-on-open.patch`
(already applied there) and the equivalent
`builder/package/ar8030/0008-session_socket-preserve-datagram-flag-on-open.patch`
(ported to the air side as part of this project, since it previously only
existed on the ground side). **Both patches need to be applied** — one
end honoring datagram mode while the other silently downgrades to a
non-datagram socket reproduces the exact "symmetric zero rx_bytes despite
a healthy physical link" failure the ground-side patch's commit message
documents, and caps any single write at ~690 bytes on the unpatched end.

## Bitrate control

`ar8030-transport-tx` reads its own current AP-side TX throughput
estimate directly from the SDK (`BB_GET_MCS` ->
`bb_get_mcs_out_t.throughput`, already in kbps) — this is **local**
information the air unit already has, no round trip to the ground needed.
`BB_EVENT_MCS_CHANGE`/`BB_EVENT_LINK_STATE` are subscribed only as a
"check sooner" wake hint; a periodic poll (`bitrate_ctl.c`'s
`poll_interval_ms`) is what actually drives it, so a missed/coalesced
event callback can never wedge the loop. The observed throughput is
scaled by a safety margin (`-m`, default 0.70), clamped to `[-n, -x]`
kbps, rate-limited and given hysteresis so it doesn't chatter at an MCS
boundary, and applied via `GET /api/v1/set?video0.bitrate=<kbps>` on
waybeam's loopback HTTP API (`documentation/HTTP_API_CONTRACT.md` in
waybeam_venc — `video0.bitrate` is `MUT_LIVE`, applied without a pipeline
restart).

## Build

### Everything, auto-detecting both cross toolchains

```sh
make            # cross-builds tx/ar8030-transport-tx and rx/ar8030-transport-rx
make print-config  # show what toolchain/SDK paths were actually detected
```

The top-level `Makefile` expects `../builder` and `../sbc-groundstations`
(this project's actual sibling layout) to be Buildroot trees that have
already built their `ar8030` package at least once, and auto-detects each
side's cross `gcc` and AR8030 SDK staging dir from there:

- **tx** from `../builder/openipc/output/` (builder's thin-overlay clone
  of `OpenIPC/firmware` — note `builder.sh` `rm -rf`s and re-clones this
  on every run, so treat the detected path as best-effort, not stable).
- **rx** from `../sbc-groundstations/output/<defconfig>/` (its own native
  Buildroot tree; if more than one defconfig has been built, pass
  `GS_DEFCONFIG=<name>` to pick one).

Override any piece explicitly (useful if your checkout layout differs, or
to force a specific toolchain): `make TX_CC=... TX_SDK_INC=... TX_SDK_LIB=...`
/ `RX_CC=... RX_SDK_INC=... RX_SDK_LIB=...`. `make clean` cleans all three
subdirectories (`tx/build`, `rx/build`, `test/build`, plus the built
binaries) — each subdirectory builds into its own `build/` to avoid
handing a wrong-architecture object file from one side's build to the
other's link step (`tx/`, `rx/`, and the host test all compile some of
the same shared `common/*.c` sources, for three different toolchains).

### Host smoke test (no AR8030 SDK, no cross toolchain)

```sh
make test      # or: make -C test check
```

Exercises `common/chunker.c` -> `common/reassembly.c` -> `rx/rtp_h265.c`
end to end against a synthetic frame (byte-identical reassembly, correct
RTP/H.265 framing including FU-A fragmentation) with plain host `gcc`.
This is the only part of the protocol that's meaningfully testable off
real hardware — everything AR8030-specific needs the SDK and, for a real
run, both boards.

### Cross-compiling one side by hand

Both `tx/` and `rx/` need the AR8030 SDK's headers (`bb_api.h`,
`bb_config.h`, ...) and `libar8030_client.so`/`.a`. If you've built
`yz_host_drv` yourself, that's its `release/inc` and `release/lib`
(README.md there); if you're building from one of the OpenIPC Buildroot
trees, it's `$(STAGING_DIR)/usr/include/ar8030` and
`$(STAGING_DIR)/usr/lib` after building the `ar8030` package (this is
exactly what the top-level Makefile automates).

```sh
make -C tx CC=<cross-gcc> \
           AR8030_SDK_INC=/path/to/ar8030/inc \
           AR8030_SDK_LIB=/path/to/ar8030/lib

make -C rx CC=<cross-gcc> \
           AR8030_SDK_INC=/path/to/ar8030/inc \
           AR8030_SDK_LIB=/path/to/ar8030/lib
```

## Buildroot integration

- **Air side** — `builder/package/ar8030-transport-tx/` (see that
  project's `CLAUDE.md`: `builder/` is a thin overlay copied into a fresh
  `OpenIPC/firmware` Buildroot clone every build, `package/*` is where
  local additions like this one live).
- **Ground side** — `sbc-groundstations/package/ar8030-transport-rx/`
  (a normal package in that project's own `BR2_EXTERNAL` tree, alongside
  the existing `ar8030`/`pixelpilot` packages).

Both packages use `SITE_METHOD = git` pointing at
`https://github.com/henkwiedig/ar8030-transport.git` (matching how
`ar8030`/`pixelpilot` are fetched), which 404s until this repo is
actually pushed there. Until then, point Buildroot at a working tree
instead — the same `<PKG>_OVERRIDE_SRCDIR` mechanism `builder/package/waybeam`
already uses, documented at the top of each package's `.mk`:
`echo 'AR8030_TRANSPORT_TX_OVERRIDE_SRCDIR=/path/to/ar8030-transport' >> $(O)/local.mk`
(or as a plain environment variable). Neither package is wired into any
device's defconfig yet — that's a deliberate per-device decision left to
whoever enables it (`BR2_PACKAGE_AR8030_TRANSPORT_TX=y` /
`BR2_PACKAGE_AR8030_TRANSPORT_RX=y`).

## Runtime deployment

Both binaries retry their connections independently, so start order
relative to `waybeam`/`pixelpilot`/`ar8030d` doesn't matter beyond
`ar8030d` itself needing to come up eventually:

- `ar8030-transport-tx` waits for waybeam's frame-shm ring to exist and
  retries `ar8030d`'s connection separately; either can start first.
- `ar8030-transport-rx` opens its UDP socket to PixelPilot immediately
  (UDP is fire-and-forget — PixelPilot need not be listening yet) and
  retries `ar8030d`'s connection.

Default `bb_socket` logical port is **2** on both sides (`-o`) —
`BB_CONFIG_MAX_TRANSPORT_PER_SLOT` is only 4 (ports 0-3), and `ar8030d`
reserves ports 0 and 1 for the `ar_net0` IP bridge on this project's
devices regardless of whether that bridge is actually up (confirmed on
bench hardware: `bb_socket_open()` on either fails with `ar_net0` down
and never started). Run `ar8030-status` first on a new deployment and
check its `BB_GET_SOCK_INFO` output — any port it lists is already
claimed by something — before assuming port 2 is free there too.

## Bench findings (Caddx Ascent Hi3516CV610 air unit)

- **`bb_socket_write()` aborts the whole call on its first internal
  timeout.** In the vendor SDK (`app/ar8030/session_socket.c`), a
  datagram-mode write blocks waiting for the daemon's write-completion
  ack and returns `-1` immediately if that single wait times out — it
  does not retry or partially succeed. Under a real, loaded RF link the
  ack can lag past a couple hundred ms during a burst (an IDR frame's
  worth of chunks arriving back to back), so a too-short `-t` turns
  ordinary link jitter into "everything fails" (visible on-device as
  repeated `bb_socket_write failed` lines interleaved with `recv bad
  socket pack` — that second line is the *late* ack for a write this
  tool had already given up on, arriving with nothing left registered to
  receive it; it is evidence the daemon *did* eventually finish the
  write, just too slowly). `-c` (chunk size) and `-t` (ack-wait timeout)
  are the two knobs to tune against a specific link; `-m` (bitrate
  margin) is the knob that controls how much data enters the pipe in the
  first place, and is the one to lower first if hiccups persist even
  after tuning `-c`/`-t`.
- **This particular air unit has very little RAM headroom** (Mem-Info
  showed `managed:58272kB` — under 60 MB total). `waybeam` was OOM-killed
  twice while bench-testing `ar8030-transport-tx` alongside it; the OOM
  dump's own per-process accounting showed `ar8030-transport-tx` using a
  few hundred KB RSS at the time, not an obviously large contributor, so
  this looks like a pre-existing tight-memory condition on this device
  rather than a leak in this tool — but it means there is close to no
  margin for a third long-running process on a box this small. Watch
  `free -m` / `dmesg` for `oom-kill` while testing a new device.

## Phase 2 (explicitly out of scope here)

- **FEC.** `ar8030_chunk_hdr.reserved` is the only field reserved for
  this; no implementation yet. A chunk lost today just drops its frame.
- **Cross-link telemetry-based bitrate.** The current loop only uses
  each side's own local `BB_GET_MCS` reading; no RTT/loss feedback from
  the peer.
- **Per-device Buildroot defconfig wiring.** The packages build; turning
  them on for a specific device is a follow-up.
- **PixelPilot's IDR-request-burst mechanism** (UDP port 11223,
  `gstrtpreceiver.cpp`) is not wired up from `ar8030-transport-rx`.
  PixelPilot's own decode-stall/RTP-gap detection already requests IDRs
  on its own for the common cases; a dropped-frame-aware trigger from the
  reassembly layer here would be more precise but isn't implemented.
