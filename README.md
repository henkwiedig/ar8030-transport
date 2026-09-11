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

## Bandwidth: the real bottleneck

On real hardware, `ar8030-transport-tx` plateaued around 4-7 Mbit/s no
matter what MCS or bitrate margin was in play, while the *stock* vendor
streamer reaches 20-25 Mbit/s on the same physical link. `BB_GET_MCS`
was reporting an excellent MCS (matching the peer's own `rx_mcs` exactly)
but a theoretical throughput far below what that MCS should give —
because channel **bandwidth** (1.25/2.5/5/10/20/40 MHz gears, see
`bb_bandwidth_e`) is a *separate* dimension from MCS in this chip, and
the link was pinned at its narrowest gear (`BB_BW_5M`) indefinitely,
even after several minutes of sustained, clean traffic that should have
given any auto-widen mechanism every chance to act.

The mechanism that would auto-widen it (`BB_CFG_SLOT_RX_MCS`'s
`bw_auto` policy) turned out to be unimplemented in this SDK build's
client library — the daemon rejects it outright (`"req 5 not found"`,
from the ioctl dispatch table in `com/ioctl_tab.c` never having an entry
for it). But `BB_SET_BANDWIDTH` — a direct manual override, "Manually
change bandwidth in 1V1 mode" per its own doc comment — *is* registered
and works immediately: forcing `BB_BW_20M` (matching
`ar8030.json`'s own `subchan.main_bw: "20"`, so already permitted by
config) took `BB_GET_MCS`'s reported throughput from 6483 → 25933 kbps
with zero change in link quality (same MCS, same SNR, same zero LDPC
errors) — and real, paced (not benchmark-hammered) throughput through
this project's own transport followed, up to ~18 Mbit/s once the second
bottleneck below was found.

This isn't persistent at the chip level, so it has to be reapplied every
time the link reaches CONNECT (fresh boot, or a reconnect after a drop)
— wired into `S60ar8030`'s own `autoreconnect()` function, right after
`ar8030-pair` succeeds.

**`ar8030-transport/linkctl/`** is the tool this produced: a small,
standalone `ar8030-linkctl` binary (built for both toolchains via the
top-level Makefile and installed by both `ar8030-transport-tx`/`-rx`
packages, since bandwidth/channel/MCS control is useful on either side)
wrapping `status`, `bandwidth`, `channel`/`channel-mode`,
`mcs`/`mcs-mode`, `freq`, and `force-close-socket`/`force-close-all`
(recovers a `bb_socket` stuck reporting `"already opened"`/`"socket
open error = 257"` after a client crashed without a clean
`bb_socket_close()` — confirmed to happen on real hardware during this
same investigation's own benchmarking). See `linkctl/main.c` for the
full command reference (`-h`).

## SDIO chardev: implemented, not yet proven

Even with the bandwidth fix above, real (paced) throughput through this
project's own `bb_socket_write()` calls capped hard around 5-7 Mbit/s —
confirmed via a raw benchmark tool isolating the transport layer
entirely from waybeam/chunking (`bb_socket_write()` itself blocking to
match whatever the real drain rate was, regardless of write size,
pacing, or `BB_SOCK_FLAG_SBUS`). The proximate cause: `ar8030d` here
runs `-i 3` (`INTF_TYPE_DRV`, reaching the chip through
`/dev/ar_mdev<N>`, this project's own out-of-tree kernel driver), while
the *stock* streamer's own daemon (`daemon_sdiov12`, extracted from an
official Ascent firmware release) runs `-i 1` (`INTF_TYPE_SDIO`,
`/dev/artosyn_sdio`, a thin passthrough this SDK's `daemon/dev8030/
sdio8030/sdio_dev.c` already has full source for — it was simply never
built). Loading the vendor's own prebuilt `artosyn_sdio.ko` +
`daemon_sdiov12` side by side with this project's *unmodified*
`ar8030-transport-tx` confirmed ~18 Mbit/s — i.e. the DRV-mode kernel
path itself is the bottleneck, not anything in this repo.

**Decision: reimplement the SDIO backend ourselves** (option 2) rather
than ship the vendor's closed `artosyn_sdio.ko`/`daemon_sdiov12`
binaries (option 1 — fast, proven, but an unlicensed, unrebuildable
binary pinned to one exact kernel build, in an otherwise-open project).
`bus/sdio.c` (part of the `ar8030` SDK checkout, not this repo) already
implements everything the daemon side needs — probe, firmware download,
IRQ handling, and even `artosyn_sdio_write()`/`artosyn_read()` functions
with the right signature — it just never exposed them as a character
device; only `control/ar_chardev.c` does, built around the heavier
`ar_mdev<N>`/`oal_mdev.c` message-multiplexing architecture DRV mode
uses (a fresh `skb` allocation *per write*, queued through
`oal_send_msg_req()`, built for multiplexing up to 8 `ar_mdev` instances
over one physical channel — overhead a single physical device doesn't
need, and the likely source of the DRV-mode ceiling).

Three real, independent kernel bugs came out of chasing this, all
patched in `builder/package/ar8030/` (mirrored in
`sbc-groundstations/package/ar8030/` where applicable, though the ground
unit is USB-connected and never uses the SDIO path itself):

- **`0010-sdio-fix-double-free-when-probe-fails.patch`** (applied) —
  `sdio_artosyn_probe()`'s failure path frees `dev` without clearing the
  `sdio_set_drvdata()` pointer it published earlier, so a later
  `sdio_artosyn_remove()` call (module unload, or `rmmod` during a
  normal `reboot`'s shutdown) double-frees it. Unreachable before
  `0005-dnld-retry-...patch_skip` existed (a failed firmware chunk used
  to be silently treated as success, so probe never actually failed);
  confirmed live as a full kernel panic (`kernel BUG at mm/slub.c`,
  double free in `kfree()`) the first time a real failure hit this path.
- **`0011-sdio-add-direct-artosyn_sdio-chardev.patch`** (applied) — the
  `/dev/artosyn_sdio` character device itself, calling straight into
  `bus/sdio.c`'s existing `artosyn_sdio_write()`/`artosyn_read()`,
  bypassing `oal_mdev.c`'s multiplexing layer entirely. Builds and loads
  cleanly, and the device node appears once the chip reaches RTOS mode.
  Two more bugs surfaced only once this was actually exercised with
  `ar8030d` built for `-i 1` (see below), both fixed in the same patch:
  - The kernel's own RX workqueue (`ar_sdio_rx_func()`, feeding
    `oal_mdev.c`'s DRV-mode queue) is wired unconditionally to every
    SDIO RX-ready interrupt, regardless of which userspace interface is
    actually selected at runtime (the kernel has no visibility into the
    daemon's own `-i` choice). With the chardev in use, that workqueue
    thread and a chardev reader both call `artosyn_read()` on the same
    device concurrently — confirmed live as a continuous
    `"artosyn_read: busy"` (`-EBUSY`) / `"read_valid_size ==
    read_offset"` (`-EAGAIN`) flood, each caller starving the other.
  - The first fix for that (skip the workqueue call whenever the
    chardev is *registered*) was itself wrong: `sdio_chardev_registered`
    is true unconditionally from `probe()` onward, regardless of whether
    any daemon ever opens `/dev/artosyn_sdio` — so it permanently broke
    plain DRV mode too (`total_rx_pkts` stuck at 0 forever, on every
    single boot, confirmed live). Fixed for real with an atomic
    open-count (`sdio_chardev_open_count`, tracked in
    `sdio_chardev_open()`/`release()`) — the workqueue is now skipped
    only while something actually has the chardev open.

**Separately, and probably the bigger practical finding this round:**
the "link is flaky after a reboot" symptom that had been deferred
earlier turned out to be partly self-inflicted, not purely an RF-pairing
issue:

- **`ar8030-transport-tx` (air, AP role) opened its `bb_socket` on a
  hardcoded slot 0** — but `bb_api.h`'s own doc comment for
  `bb_socket_open()` says the slot parameter on the AP side targets the
  *actual connected DEV peer's slot*, which varies across reboots
  (observed landing on slot 2 one boot, slot 0 another). Writing into a
  socket bound to a slot with no real peer left the chip's outbound
  queue with nowhere to drain, and `artosyn_sdio_write()`'s own retry
  budget (five ~10-jiffy waits, well under a second total) gives up
  permanently on that write with no higher-level retry — explaining the
  `"write wait timeout"` / frozen `total_tx_pkts` wedge seen repeatedly
  during this investigation, independent of any kernel bug. Fixed: `tx/
  main.c` now resolves the actually-connected slot from `BB_GET_STATUS`
  before opening the socket (`-s auto`-style, `-s <slot>` still works as
  an explicit override for bench use).
- The same hardcoded-slot bug existed in `ar8030-linkctl bandwidth`
  (used by `S60ar8030`'s post-pairing bandwidth widen) — fixed the same
  way, `-s auto` resolves the connected slot from live status instead.
- `S60ar8030`/`S97ar8030`'s `autoreconnect()` loop was one-shot: it only
  retried `ar8030-pair` until the *first* success, then exited — a later
  drop (confirmed to happen on real hardware) was never retried,
  matching "needed to manually bring it up again". Fixed: it now keeps
  polling link state after a successful pair and re-enters the retry
  loop if the link ever leaves `CONNECT`.
- A stopped `ar8030-transport-tx` doesn't always release its `bb_socket`
  cleanly, leaving the port reporting `bb_socket_open() failed (ret=-1)`
  on the next start — recovered with `ar8030-linkctl force-close-all`
  (see its own section below); not yet root-caused why the close isn't
  clean.

**Status: SDIO daemon mode itself still not confirmed working
end-to-end.** With both kernel bugs above fixed, `ar8030d` built with
`USING_8030SDIO=ON` (now the case — alongside the existing
`USING_8030DRV=ON` in `builder/package/ar8030/ar8030.mk`; both compile
into one binary, `-i` selects at runtime) still hit a *different*
failure on its first real attempt: every write into `/dev/artosyn_sdio`
times out (`"artosyn_sdio_write: write wait timeout"`, `total_tx_pkts`
frozen at 0) and the device never registers with `bb_dev_getlist()`
(`ar8030-linkctl status` reports `"no AR8030 device known to the
daemon"`) — even though the exact same physical chip, same firmware,
same fresh boot, works fine in DRV mode (`-i 3`) moments later. Not yet
root-caused; testing was paused here to restore a working DRV-mode link
rather than keep iterating live on a single dev unit.

**Update: the "-i 1 first write always times out" failure above was a
settle-time artifact, not a real bug.** Retesting against an
already-settled chip (module loaded and running fine in DRV mode for
several minutes first, then switching the daemon to `-i 1` without any
module reload) registered the device cleanly and passed `BB_GET_STATUS`
immediately — no timeout, no wedge. The failure only ever reproduced
right after a *fresh* module/firmware reload, before the chip's own SDIO
command-processing task was ready. Not yet fixed at the driver level
(no code change makes this reliable from a cold boot); the practical
workaround for now is simply not exercising `-i 1` immediately after a
reload.

**Then: real throughput was measured for the first time, and it was
*worse* than DRV mode, not better** (~0.3 Mbit/s, vs. DRV mode's own
much higher numbers once its bandwidth bug — see above — was also
fixed). The daemon's own debug-pad log
(`/var/log/ar8030/daemon_log/<mac>.log`, written by `ar8030d` itself —
worth knowing about for any future debugging session) showed
`dev_dat_so_write_proc`'s "send ok"/"send cpl" entries completing in a
clean ~100-110ms cadence regardless of chunk size: the signature of a
fixed per-write round-trip cost, not a real bandwidth ceiling.
Root-caused by decompiling the vendor's own `artosyn_sdio.ko` (Ghidra,
`ghidra-mcp`) for comparison: its own write function waits via a plain,
effectively-unbounded `prepare_to_wait_event()`/`schedule()` loop — no
short per-iteration timeout — while this driver's `artosyn_sdio_write()`
only ever waited 5×10 jiffies (well under a second) before giving up and
falling back to `WORKAROUND_FOR_INTERRUPT_LOST_ISSUE`'s own ~10ms-interval
polling path, which was satisfying nearly every single write instead of
the real TX-ready interrupt (which does fire correctly — confirmed
`artosyn_sdio_irqhandler()` already calls `wake_up_interruptible_all()`
on it). Fixed in **`0012-sdio-write-wait-for-real-interrupt-not-100ms-poll.patch`**
(applied): one much longer (`SDIO_WRITE_WAIT_MS`, 2000ms) wait instead of
the short retry loop, matching the vendor's patience. `artosyn_read()`
was compared the same way but deliberately left alone — its own design
already checks the condition directly before ever waiting, and the
debug-pad log showed no equivalent read-side stall.

Also worth knowing for next time: **DRV mode's own throughput turned out
to fluctuate a lot even with the bandwidth bug fixed** (a real, ~18Mbps-
capable session was reported as visibly unstable, vs. the user's own
memory of the stock vendor SDIO driver+daemon being "rock solid") — since
`oal_mdev.c` (DRV mode) calls into this exact same `artosyn_sdio_write()`,
**`0012` fixes DRV-mode stability too, not just SDIO throughput —
confirmed on real hardware**: the same session that fluctuated wildly
before `0012` (swinging well below and above the true link capacity)
settled to a steady ~17 Mbit/s (occasional dips to the low teens) at
mcs=12/bw=20M afterward, through plain DRV mode with no other change.
This makes `0012` valuable independent of whether the SDIO daemon path
ever gets finished — it's a real, general fix to this driver's own
write-wait design, not something specific to the raw chardev.

**Retested `-i 1` with `0012` in place: a new, different, earlier-stage
stall, not the throughput problem `0012` fixes.** The daemon's data
socket never finished opening at all -- its own debug-pad log's last
line was `rpc_socket_read_proc:socket try init slot 0 port 2`, then
nothing further, ever (confirmed the log file's mtime itself had gone
stale, not just a display lag). Crucially, `dmesg` showed **no**
`"write wait timeout"` at all during this stall, even minutes in --
with `0012`'s now-2-second timeout, a real low-level write attempt would
have logged one by then. That means this particular stall happens
*before* any `artosyn_sdio_write()`/`artosyn_read()` call is ever
reached at all -- somewhere in the daemon's own session/socket
establishment sequence (`rpc_socket_read_proc` and whatever it calls),
not in the SDIO transport layer `0012` touches. Correct slot/port were
confirmed targeted (this is not a repeat of the earlier hardcoded-slot
bug) and the device itself was already registered and passing
`BB_GET_STATUS` cleanly at the time.

**Root-caused and fixed the socket-open stall.** Comparing `bus/sdio.c`'s
own `sdio_chardev_poll()` against the working `tx_q`/`rx_q` split
elsewhere in the file: `poll_wait()` only ever registered on `dev->tx_q`.
The chip's `so_open` acknowledgment -- the very thing the daemon's
socket state machine (`daemon/sock_node_rpc.c`, `daemon/sock_node.c`)
was waiting on to leave `sock_wait_usb_cmd` -- arrives as an RX-ready
mailbox event, and that branch's own `wake_up_interruptible_all(&dev->rx_q)`
call had been left commented out (`oal_mdev.c`'s DRV-mode consumer never
waits on `rx_q`, only via its own workqueue, so nothing needed it before
this chardev existed). At the exact moment of a fresh socket open there
is no TX traffic yet either, so nothing was coincidentally re-waking
`poll()` the way ordinary bidirectional streaming might once data
starts flowing -- confirming why this stall was 100% reproducible right
at the start, not intermittent. Fixed in `0011`'s own **UPDATE 3**:
uncommented the `rx_q` wakeup in both `artosyn_sdio_irqhandler()`'s and
`artosyn_sdio_reg_check()`'s rx-ready branches, and `sdio_chardev_poll()`
now waits on both `tx_q` and `rx_q`. Confirmed on real hardware: with
this plus `0012`, the socket opened immediately and a burst of real
video data flowed at a genuinely fast pace (`dev_dat_so_write_proc`
completing in single-digit milliseconds, not the ~100ms-per-write
pattern from before `0012` -- a real, large improvement).

**Then: a *third*, still-unfixed bug -- video stalls again after a short
burst under sustained load.** After maybe a second or two of fast,
correct-looking traffic, the daemon's own ring buffer
(`sock_dev_push_data`) starts logging `"warning loss rpc data ... push =
0"` repeatedly, with `buf_wr_index`/`buf_head_index` frozen at the exact
same values across many seconds -- the send side has stopped draining
entirely, permanently, not just falling behind. Critically, `dmesg`
showed **no** `"write wait timeout"` at any point during this stall,
even minutes in, and neither `ar8030d` nor `ar8030-transport-tx` was
burning CPU (both idle/sleeping in `top`) -- ruling out a busy-loop and
suggesting the send thread is blocked somewhere that never reaches (or
never returns from) `artosyn_sdio_write()` again, rather than that
function itself timing out. Not yet root-caused. Recovering requires
switching back to DRV mode (`-i 3`) -- no kernel module reload needed,
the module itself keeps working fine, only the SDIO daemon's own session
state gets stuck.

**Narrowed the sustained-load stall considerably (still not fixed).**
Compared the two most likely reply-handling paths in
`daemon/sock_node.c`'s `dev_dat_so_write_proc()` (the normal `pack->sta
>= 0` completion path, and the `-0x107`/"send pending" flow-control
path) line-by-line against the vendor's real `daemon_sdiov12` (Ghidra,
`ghidra-mcp`, same technique that found `0011`/`0012`) -- both are
**logically equivalent** between our SDK source and the vendor's actual
binary (the `-0x108` case is a no-op, just a log line, in *both*). So
this is not a reply-handling logic bug. Retested live and looked at
what's actually stuck: `ar8030-transport-tx`'s own process survives (not
crashed, not spinning -- 0% CPU throughout), but `/proc/<pid>/task/*/wchan`
showed its main thread blocked in `pthread_join()` waiting on the
bitrate-control thread, which was itself blocked (almost certainly
inside a `bb_ioctl()` call, e.g. `BB_GET_MCS`, waiting on an RPC reply
that never arrives) -- i.e. **the control-plane RPC got stuck too, not
just the data socket.** Critically, a *completely separate, freshly-
started* client (`ar8030-linkctl status`, a brand new connection) worked
instantly while this was happening -- so neither the daemon nor the
SDIO channel/chip is globally wedged; this is specific to one
long-lived client session getting into a stuck state once something
goes wrong with its data socket. `dmesg` showed zero `"bytes for read
lost"` events either time this was checked, ruling out the kernel-level
mailbox-notification-drop mechanism as the direct cause too.

Current best hypothesis (not yet confirmed): a response-matching/reqid
desync in the client-side RPC library itself (`libar8030_client.so`,
same SDK source on both sides of this project) -- something causes one
in-flight RPC reply to be consumed by the wrong waiter, or dropped, such
that whichever call is waiting for it blocks forever, but only for that
one already-open session; a fresh session's own request/reply pairing
is unaffected.

**Next steps, for whoever picks this back up:**
- Trace the client library's own RPC request/reply matching (wherever
  `bb_ioctl()`/`bb_socket_write()` correlate a reply to the call that's
  waiting for it -- likely in this SDK's shared `com/` or client-side
  RPC code, used identically by every tool including `ar8030-linkctl`,
  so the same code that just proved fine for a fresh session needs to
  be checked for what differs once a session has an active data socket
  alongside other RPC traffic).
**Implemented a workaround, not a fix: `S65ar8030-transport-tx` now
watchdogs itself.** `start_tx()`/`watchdog()` in that script track `-v`'s
own `"totals: ... bytes=N"` field (not just log mtime -- confirmed on
real hardware that mtime alone misses a *second* failure mode, below)
and force-restart the whole process (`kill -9` + respawn) if it goes
unchanged for two consecutive 6s checks. Confirmed working on real
hardware: it correctly detected a stall and recovered the socket cleanly
without any manual intervention. But confirmed **not sufficient for
production SDIO use as-is** -- the underlying stall recurs roughly every
10-15 seconds under sustained load, so auto-recovery just produces
frequent hiccups rather than smooth video. This is a real safety net
(and cheap insurance for DRV mode too, where the same class of bug could
in principle also occur, just apparently far more rarely) but the actual
protocol-level bug above still needs fixing for SDIO mode to be usable.

Also found, incidentally, a **second, different failure signature** on
one retest: instead of a silent freeze, `bb_socket_write()` started
returning immediately with zero progress, repeatedly (`"bb_socket_write
made no progress (len=N, sent=0, ret=0)"`), while the main loop kept
iterating and printing fresh stats every second -- an mtime-only
watchdog would never notice this one, since the log file *is* still
being written to; only the byte-progress check catches it. Whether
this and the "blocked forever" signature from earlier are the same
underlying bug manifesting two ways, or two separate bugs, is unknown.

**Re-assurance test against the stock vendor SDIO stack -- and a
methodology fix that matters for all future testing here.** Loaded the
vendor's own real `artosyn_sdio.ko` + `daemon_sdiov12` (Ascent V18.21.10,
`libstdc++.so.6`/`libgcc_s.so.1` alongside -- musl's own libc suffices,
those two are the only extra libraries it needs) to compare directly
against our own stack under identical conditions. First attempt (right
after our own module had already been loaded once this same boot) found
the vendor daemon *also* unresponsive -- initially read as a vendor-side
problem, but retesting on a genuinely fresh reboot (nothing else having
touched the chip first) showed it responding instantly and then running
a real waybeam/PixelPilot session at a clean, stable ~14-18 Mbit/s for
minutes with zero hiccups. **The cross-module-reload contamination this
project has run into all night (our module wedging after the vendor's
had run, and vice versa) is real and affects any stack, not a defect
specific to either implementation** -- meaningful for how to test any
future fix here: always from a fresh boot, never right after another
module has already touched the chip. Confirmed practical recipe:
rename `S60ar8030`/`S65ar8030-transport-tx` to keep them from
auto-starting (`rcS` only checks `-f`, not `-x`, so `chmod -x` alone
does *not* skip a script here), reboot, then bring the stack under test
up by hand.

**Redoing our own SDIO stack the exact same clean way surfaced the real
remaining gap, cleanly isolated from that contamination confound for the
first time.** Our own module + daemon (`-i 1`), brought up by hand on an
equally fresh boot, still recurringly hit `"bb_socket_write made no
progress"` -- capping real throughput around ~1-3 Mbit/s (`bitrate_ctl`
correctly estimated the link at ~26 Mbit/s and asked for ~18 Mbit/s
video; the shortfall is entirely in the transport, not the bitrate
control loop) -- while the vendor's stack, tested identically, held
~14-18 Mbit/s indefinitely. This is a real difference between the two
implementations that fresh-boot testing did not explain away.

Comparing `artosyn_sdio_irqhandler()`/`artosyn_sdio_reg_check()` against
the vendor's real decompiled equivalent (Ghidra, same technique that
found `0012`) turned up one more real design difference, not yet
present in `0004`'s already-matching size-decode fix: **the vendor wakes
a single shared waitqueue for every mailbox event** (either channel,
rx-ready, and tx-ready alike), while this driver used separate `tx_q`/
`rx_q` queues woken only by their own matching event type -- so a writer
blocked on `tx_q` only got a chance to re-check when a genuine TX-ready
event fired, never on unrelated RX or channel activity the vendor's
design would have used as an extra chance to notice a state change.
Matched (0011's own UPDATE 4) by having every wake site -- both mailbox
channels, rx-ready, tx-ready, in both `artosyn_sdio_irqhandler()` and
`artosyn_sdio_reg_check()` -- wake both `tx_q` and `rx_q`, rather than
introducing one literal shared queue for the same practical effect.

**Confirmed a real, partial improvement, but not a fix.** Retested the
exact same clean-boot way: the failure mode changed from *permanent*
stalls (recovery previously needed the watchdog's kill-and-restart) to
brief, self-recovering blips roughly every 15-20 seconds -- a genuine
resilience improvement from giving the blocked writer more chances to
notice a real state change. But real throughput stayed capped around
~1.2 Mbit/s; the periodic blips, though no longer permanent, still cost
enough to bottleneck it far below the vendor's ~14-18 Mbit/s under
identical conditions. The underlying root cause -- why the real TX-ready
condition doesn't become true promptly and periodically, on both
implementations' shared low-level state machine, only reliably on the
vendor's -- is still not found.

**Next steps, for whoever picks this back up:**
- **Always test from a fresh reboot**, nothing else having touched the
  chip first (see the contamination finding above) -- this invalidates
  any test methodology that reloads modules back-to-back in one boot.
- Root-cause why the vendor's stack never hits this stall at all while
  ours still does periodically, even with the wake-propagation gap
  closed. The client-side RPC library's own request/reply matching
  (`libar8030_client.so`, shared by every tool including
  `ar8030-linkctl`) remains a candidate, not yet directly investigated
  -- a fresh client session was confirmed unaffected while an existing
  one was stuck, in an earlier (contaminated) test tonight; worth
  re-confirming under the clean-boot methodology.
- The watchdog in `S65ar8030-transport-tx` (byte-progress based, see
  above) remains valuable as a safety net regardless -- keep it even
  once the root cause is found, as insurance against whatever residual
  rate of stalls remains.
- Once sustained throughput actually holds at something close to the
  vendor's ~14-18+ Mbit/s: confirm picture quality/stability under a
  full waybeam/PixelPilot session, not just a raw benchmark number.
- Given DRV mode now performs close to vendor-parity levels on its own
  (once both the bandwidth bug and this write-wait bug are fixed), the
  SDIO daemon path may no longer be a hard requirement for production —
  it remains a valid path to pursue for the last stretch of headroom
  (and structurally simpler/lower-overhead than DRV mode's `oal_mdev.c`
  multiplexing either way), but isn't blocking a usable release the way
  it looked earlier in this investigation.
- The historical "intermittent firmware-download failure on a freshly
  rebuilt module" mystery from earlier in this investigation did not
  recur once builds went through the real Buildroot pipeline
  (`builder/package.sh ar8030`) against a properly-synced
  `openipc/general/package/` tree, rather than ad-hoc manual `make`
  invocations — but this was observed only incidentally, not
  deliberately re-isolated, so treat it as "not currently reproducing"
  rather than "fixed".
- **Do not `cat` (or otherwise read) `/proc/ar_drv/dev0/dbg`** (or
  presumably its sibling proc files under `/proc/ar_drv/dev0/`) on real
  hardware — confirmed live to crash the kernel outright (`Unable to
  handle kernel NULL pointer dereference`, `proc_write+0x32` called from
  `proc_reg_read_iter`: this proc entry's read path is wired to the
  write handler by mistake, a genuine pre-existing bug in this vendor
  driver's own `ar_proc.c`). Setting `dbg_log_level` via the documented
  `echo dbg_log_level=N > ...` write interface is presumably still fine
  (untested since the crash) — just never read it back.

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
