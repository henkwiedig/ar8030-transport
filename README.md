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
prefixed with a 22-byte header (magic, frame sequence, chunk index/count,
this chunk's payload length, IDR/GDR/enhance flags, codec, capture pts, a
CRC16 over the payload). No ACK, no NACK, no FEC in this version — a
chunk lost on the radio makes its frame incomplete, and
`ar8030-transport-rx` drops the whole frame and moves on to the next one
rather than waiting (see "Phase 2" below for where FEC would slot in).

**Payload checksum.** Added after bench testing showed zero write
failures, zero resyncs and zero dropped frames, yet still visibly
corrupted video ("green blocks", recovering at the next IDR) — the
classic signature of a decoder fed corrupted-but-present data, not
missing data. The chunk header's magic only ever validated *framing*
(where one chunk ends and the next begins); it said nothing about
whether the payload bytes themselves survived the radio intact. Every
chunk now carries a CRC16 (`common/crc16.c`) over its payload, checked in
`common/chunk_stream.c`; a chunk whose checksum doesn't match is silently
skipped (not handed to the reassembler at all), which then surfaces as an
ordinary gap — the frame it belonged to gets dropped the same way a
lost chunk always was, rather than being reassembled with corrupt bytes
inside. See `-v`'s `checksum_fails` counter below.

**Reassembly ordering.** `bb_socket` is a queued point-to-point channel,
not a packet-switched network — there is no equivalent of IP
routing/multipath that could reorder two writes from the same sender.
`rx/main.c`'s frame reassembly therefore assumes chunks arrive in the
order they were sent and treats any violation of that (an out-of-order
`chunk_idx`, or a new `frame_seq` starting before the previous one
finished) as data loss: drop what was collected and wait for the next
frame's first chunk. (This is a different guarantee from `payload_len`
below: ordering is about which *chunk* comes next, `payload_len` is about
where one chunk's *bytes* end within a stream that no longer marks that
boundary for us.)

**Chunk size vs. RTP MTU.** These are two independent numbers:
`ar8030-transport-tx -c` controls how a whole encoded frame is split for
the `bb_socket` hop; `ar8030-transport-rx -M` controls how the
*reassembled* frame is re-split into RTP/FU-A packets for the UDP hop to
PixelPilot. There's no reason they need to match, and in practice they
won't — `-c` is sized for AR8030 throughput, `-M` for a conventional
Ethernet/Wi-Fi-range UDP MTU.

## Stream mode, not datagram

Both apps open their `bb_socket` **without** `BB_SOCK_FLAG_DATAGRAM` —
matching the stock vendor streamer's own `bb_socket_open()` calls,
reverse-engineered from `ar_ldyhs_sky` (the Ascent air unit's real
production video/audio streamer): `bb_socket_open(dev, slot=0, port=3,
flags=0x27, &opt)` for video, `port=2` for audio, neither with the
datagram bit set. This replaced an earlier datagram-mode version of this
transport, for two reasons:

- **Datagram mode's failure mode is worse under real congestion.**
  `bb_socket_write()` in datagram mode waits once for the daemon's
  write-completion ack and aborts the *entire* call the instant that
  single wait times out (`app/ar8030/session_socket.c`). On a real,
  contended RF link under a bursty source (an IDR frame's chunks
  arriving back to back), that produced sustained `bb_socket_write
  failed` storms on bench hardware (see "Bench findings" below — that's
  what those numbers were tuning around). In stream mode, the same
  internal wait can return a **partial** byte count without that being
  an error at all — the vendor's own send loop
  (`fpv_bb_video_stream_send`) just resubmits the remainder and keeps
  going, only treating a write that makes *zero* progress as a real
  failure. `tx/main.c`'s `chunk_send_to_socket()` does the same, with the
  same 1500ms per-attempt timeout the vendor uses.
- **It sidesteps the datagram-mode symmetry bug entirely.** Upstream
  `app/ar8030/session_socket.c` has (had) a bug where the socket-open RPC
  always strips `BB_SOCK_FLAG_DATAGRAM` before telling the local chip,
  regardless of what the caller asked for —
  `sbc-groundstations/package/ar8030/0008-session_socket-preserve-datagram-flag-on-open.patch`
  and its air-side counterpart (`builder/package/ar8030/0008-...patch`,
  ported as part of this project) fix that, but neither patch is a
  dependency of this transport anymore now that it never asks for
  datagram mode at all. (They may still matter for other things — e.g.
  `net_dev`/`ar_net0` — this project just no longer needs them.)

The cost: stream mode has no message-boundary framing at the transport
level — a `bb_socket_read()` can return less than one chunk, several
concatenated, or a chunk split arbitrarily across reads, none of which
was possible in datagram mode. `ar8030_chunk_hdr.payload_len` (see
"Protocol" above) and `common/chunk_stream.c` exist specifically to
reassemble the raw byte stream back into discrete chunks, resyncing
byte-at-a-time on a bad magic if alignment is ever lost — which should
only happen right after a corrupted read or a sender restart mid-stream,
not in steady state.

## Chunk sizing

`ar8030-transport-tx -c` defaults to 65535 bytes — the wire format's
actual ceiling, not a conservative fraction of it (see
`AR8030_CHUNK_MAX_PAYLOAD` in `common/ar8030_chunk.h`; `payload_len` is a
`uint16_t`, so no chunk can ever carry more than that regardless of `-c`).
Most encoded frames go out as a single chunk; only the largest IDR frames
need more than one.

This changed from an earlier, much smaller default (1024–4096, tuned
during the datagram-mode era — see "Bench findings" below) after
reverse-engineering how the stock vendor streamer (`ar_ldyhs_sky`)
actually handles this. It **doesn't chunk at the application layer at
all**: `fpv_video_send_thread` accumulates a whole encoded frame (the
encoder can emit it as several slices; the accumulation loop reassembles
them into one buffer before ever touching the socket), then
`fpv_bb_video_stream_send` hands the *entire* frame to `bb_socket_write()`
in one call — the same partial-write retry loop this project's
`chunk_send_to_socket()` uses, at the same 1500ms per-attempt timeout —
relying on stream mode's partial-write tolerance rather than any
fixed transport-level slice size. It even checksums the payload (XOR32,
prepended header, magic trailer), independently validating this
project's own CRC16 addition above.

Given this project's reassembly is all-or-nothing per frame (no FEC, no
NACK — any one missing or checksum-failed chunk drops the whole frame
regardless of *which* chunk), small fixed chunks bought nothing but cost
real overhead: every `bb_socket_write()` is an RPC round trip to
`ar8030d`, and more, smaller chunks per frame means more RPC volume —
which is exactly what was flooding `ar8030d`'s own debug log and filling
a tiny tmpfs `/tmp` (see "Bench findings" below) — plus more independent
chances for one transfer glitch to take out an entire frame for no
benefit. Raising the default to match the vendor's one-chunk-per-frame
approach cuts RPC/syscall volume roughly in proportion to the old
chunk-per-frame count, with no loss of correctness (the CRC16 still
covers whatever ends up in one chunk, however large). `-c` can still be
lowered for experimentation on a particularly poor link.

`common/chunker.c`'s per-chunk header+payload staging buffer is
caller-provided (`ar8030_chunk_frame()`'s `scratch`/`scratch_cap`
params) rather than an internal fixed-size array, specifically so a
65535-byte ceiling doesn't mean a 65KB+ buffer on every call's stack on
a RAM-constrained target — `tx/main.c` mallocs it once at startup, sized
to the actual `-c` in use, not the ceiling.

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

**Second signal: ring backlog.** MCS says what the radio *should*
currently be able to carry; it says nothing about whether this side's own
`bb_socket_write()` pipeline is actually draining frames that fast — a
local congestion signal MCS can't see. The stock vendor streamer has an
equivalent second signal (`com_bb_video_buffer_query_left_frames()`,
reverse-engineered from `fpv_video_buffer_cache_monitor` — cuts bitrate
harder when its own outbound frame queue backs up, regardless of what MCS
nominally allows). This project's analog reads waybeam's frame-shm ring's
own `low_water_slots` field (`third_party/waybeam_frame_ring/venc_frame_ring.h`)
— the lowest occupancy the *producer* (waybeam) saw in each ~200ms window,
published specifically for a co-located rate controller to read. It's a
low-water rather than a high-water check deliberately (see that header's
own comment): a healthy ring routinely spikes its fill percentage on
ordinary bursts, but low-water asks whether the ring ever failed to drain
at all during the window, which is what actually distinguishes standing
backlog (this side genuinely can't keep up) from a normal burst.
`bitrate_ctl.c` checks this every ~100ms tick (a plain shared-memory
read, no RPC, so unlike the MCS check it doesn't need to wait for
`poll_interval_ms`) and, once `low_water_slots` reaches
`ring_backlog_high_slots` (default 2 — `venc_frame_ring.h`'s own
threshold for "standing backlog"), immediately cuts the last-applied
bitrate by `ring_backoff` (default 0.85), bypassing hysteresis, rate
limited to once per 250ms rather than the normal path's 1500ms. Logged
with an `URGENT` tag in stderr / `-v` output to distinguish it from an
ordinary MCS-driven change.

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

## Diagnosing "nothing is getting through"

Both binaries take a `-v` flag that prints one stats line to stderr per
second, so you can see *where* in the pipeline things stop moving rather
than only knowing that they did:

- **`ar8030-transport-tx -v`**: frames in/complete/incomplete per second,
  chunks sent/failed per second, the resulting Mbit/s, and the frame-shm
  ring's own health (`producer_writes`/`our_reads`/`full_drops`/
  `other_drops`). `producer_writes` is `ring->hdr->write_idx` read
  directly — **not** `venc_frame_ring_get_fill()`'s own `writes` field,
  which is a per-attached-instance counter only a *producer* increments
  and therefore always reads 0 for this tool (it only ever consumes);
  using that field here would make "waybeam isn't producing anything"
  indistinguishable from "everything is fine", so `producer_writes` reads
  the real shared counter instead. Read the line in order:
  `producer_writes=0` and not climbing means waybeam itself isn't
  producing frames (not an `ar8030-transport-tx` or radio problem at
  all); `frames in` > 0 but `chunks failed` climbing means the radio link
  is the bottleneck, not waybeam or the ring.
- **`ar8030-transport-rx -v`**: chunks in per second (with their Mbit/s),
  **resync drops** — bytes discarded while scanning for the next valid
  chunk header, which should sit at (or very near) zero; anything else
  means the byte stream lost alignment (every chunk in between was
  lost) — and **checksum fails** — chunks whose header was fine but
  whose payload's CRC16 didn't match, silently skipped rather than
  forwarded (see "Protocol" above). Unlike resync drops, a nonzero
  checksum-fail rate means framing is intact but the radio is delivering
  corrupted bytes inside otherwise-valid chunks — exactly the failure
  mode that produced visible video corruption with zero write failures
  and zero resyncs on bench hardware before this counter existed. Also
  frames complete/dropped per second (`dropped` is deduplicated per
  `frame_seq` — see `common/reassembly.c`'s `note_dropped()` — so it
  counts distinct lost frames, not every stray chunk that belonged to
  one), and RTP packets/Mbit/s actually sent toward PixelPilot. Zero
  chunks in at all here, with `tx -v` showing chunks being sent, points
  at the radio link itself or a port/role mismatch between the two ends
  (see "Runtime deployment" above); chunks arriving but frames never
  completing (`dropped` or `checksum_fails` climbing while `complete`
  stays near zero) points at loss/corruption severe enough that some
  chunk of nearly every frame is unusable.

## Bench findings (Caddx Ascent Hi3516CV610 air unit)

- **What led to the datagram → stream mode switch above.** The first,
  datagram-mode version of this transport hit sustained
  `bb_socket_write failed` storms under real load: `bb_socket_write()`
  in datagram mode aborts the *entire* call the instant its one
  internal wait times out (no partial success), and on a real, loaded
  RF link the daemon's write-completion ack can lag past a couple
  hundred ms during a burst (an IDR frame's worth of chunks arriving
  back to back) — visible on-device as repeated `bb_socket_write
  failed` lines interleaved with `recv bad socket pack` (that second
  line is the *late* ack for a write this tool had already given up on;
  it's evidence the daemon *did* eventually finish the write, just too
  slowly). Raising the timeout and shrinking the chunk size helped but
  didn't remove the underlying issue; switching to stream mode
  (matching the vendor's own approach — see "Stream mode, not datagram")
  did, since a partial write there just continues rather than failing.
  `-c` (chunk size) and `-t` (per-attempt write timeout) are still the
  two knobs to tune against a specific link if hiccups persist; `-m`
  (bitrate margin) controls how much data enters the pipe in the first
  place and is the one to lower first.
- **This particular air unit has very little RAM headroom** (Mem-Info
  showed `managed:58272kB` — under 60 MB total), and `waybeam` was
  OOM-killed on real hardware after `ar8030-transport-tx` had been
  running only a few minutes (it had previously run for hours over
  waybeam's own direct UDP/RNDIS output with no issue). Neither
  `ar8030-transport`'s own code nor waybeam's frame-shm producer path had
  a leak (confirmed by manual audit plus an ASAN+LeakSanitizer stress
  run, and a from-scratch trace of every frame-shm backend) — the actual
  cause was `ar8030d` itself: `com_log_init()`
  (`yz_host_drv/com/com_log.c`) opens a per-run debug/RPC trace log with
  no size cap or rotation, and every `bb_socket_write()`/`BB_GET_MCS`
  call this transport makes is itself an RPC logged at `INFO` level — far
  more RPC volume than the plain IP/RNDIS path the daemon was previously
  exercised against. That log filled the air unit's 28.5 MB tmpfs `/tmp`
  in well under an hour; once full, the *system-wide* memory pressure
  from tmpfs pages (invisible in any single process's own RSS, which is
  why `ar8030-transport-tx` itself looked innocent in the OOM dump) got
  an arbitrary process picked by the kernel's OOM killer — waybeam in the
  observed case, unrelated to the actual leak. Fixed upstream in the SDK,
  not in this repo: `builder/package/ar8030/0009-com_log-cap-daemon-log-file-size.patch`
  and its ground-side counterpart
  (`sbc-groundstations/package/ar8030/0010-...patch`) cap that log file at
  2 MB, truncating it back to empty instead of growing forever. Still
  worth watching `free -m` / `df -h /tmp` / `dmesg` for `oom-kill` on a
  new device regardless — the tmpfs is small enough that anything else
  writing to it steadily could reproduce the same failure mode.

## Phase 2 (explicitly out of scope here)

- **FEC.** `ar8030_chunk_hdr.reserved` is the only field reserved for
  this; no implementation yet. A chunk lost today just drops its frame.
- **Cross-link telemetry-based bitrate.** The current loop only uses
  each side's own local `BB_GET_MCS` reading; no RTT/loss feedback from
  the peer.
- **Per-MCS-level bitrate margin table.** `bitrate_ctl.c` applies one
  flat `-m` margin to whatever `BB_GET_MCS` reports. The vendor's own
  `fpv_bb_get_cur_tgt_videobitrate` (reverse-engineered from
  `ar_ldyhs_sky`) instead looks up a different percentage per MCS level
  from a config table (30/40/60/70% in its hardcoded fallback), which is
  presumably tuned because headroom needed at a marginal MCS isn't the
  same as at a comfortable one. Worth adopting if a flat margin proves
  too conservative at some MCS levels and not enough at others.
- **Per-device Buildroot defconfig wiring.** The packages build; turning
  them on for a specific device is a follow-up.
- **PixelPilot's IDR-request-burst mechanism** (UDP port 11223,
  `gstrtpreceiver.cpp`) is not wired up from `ar8030-transport-rx`.
  PixelPilot's own decode-stall/RTP-gap detection already requests IDRs
  on its own for the common cases; a dropped-frame-aware trigger from the
  reassembly layer here would be more precise but isn't implemented.
