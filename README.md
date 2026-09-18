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

## Embedded audio (Majestic-style, on a separate `bb_socket` port)

`audio_tx`/`audio_rx` bridge waybeam's optional audio output across the
same AR8030 link as video, landing on the **same UDP destination** as
`ar8030-transport-rx`'s video output — the way the stock Majestic
streamer put video and audio on one UDP port, demultiplexed by RTP
payload type (H.265 = 97, Opus = 98) rather than a second port.

This is two new standalone binaries, not two new threads inside
`tx`/`rx`: video's connect/reconnect state machine already carries a lot
of hard-won hardware-verified subtlety (see "`ar8030d` connection:
surviving a daemon restart" below); adding a second `bb_socket`'s
lifecycle to it would risk the working video path to fix audio. Each
audio binary owns its own `ar8030d` connection and its own `bb_socket`,
on logical port **3** by default (port 2 is `tx`'s video default; ports
0/1 are reserved for the `ar_net0` IP bridge — see `tx/main.c`'s
`DEFAULT_VIDEO_PORT` comment). This mirrors the vendor's own precedent:
the stock Ascent streamer opens video on port 3 and audio on port 2
*concurrently*, on the same chip — see "Stream mode, not datagram" above
— so two ports serving two purposes at once is the proven shape, not a
new one.

Waybeam's own audio path (`cv610_audio.c` on the CV610 backend) already
emits complete RTP/Opus packets (PT=98) to a **loopback** UDP
destination — `outgoing.audioPort` — whenever `outgoing.server` is
`unix://` or `frame-shm://` (see waybeam's
`documentation/AUDIO_UDP_OUTPUT_FEASIBILITY.md` and
`cv610_validation.c`), which is exactly what this project's own video
path uses (`third_party/waybeam_frame_ring`). That loopback destination
is the seam:

```
waybeam (frame-shm video)             air unit                    ground unit
  cv610_audio.c ──UDP/RTP/Opus──► audio_tx (binds 127.0.0.1:5601)
                                       │  ar8030_chunk_frame(codec=OPUS)
                                       ▼
                                  bb_socket (port 3, TX)
                                       │  ...over the air...
                                       ▼
                                  bb_socket (port 3, RX) ◄── audio_rx
                                                               │  ar8030_reassembly_feed()
                                                               ▼
                                                       send() -- verbatim, no re-packetization
                                                               │
                                                               ▼
                                                   udp://<same -H:-p as rx/main.c>
```

Unlike video, there is **no re-encoding or re-packetization on either
end**: `audio_tx` forwards each whole UDP datagram it receives from
waybeam as one chunk (almost always exactly one — an Opus/RTP packet at
32 kbit/s, 20 ms frames, is well under 200 bytes); `audio_rx` reassembles
and `send()`s the resulting bytes unmodified. The bytes that land on the
ground UDP socket are byte-for-byte what `cv610_audio.c`'s own RTP
packetizer built. `common/ar8030_chunk.h`'s existing per-frame `codec`
field (`AR8030_CHUNK_CODEC_OPUS`, alongside `AR8030_CHUNK_CODEC_H265`)
is what lets a chunk on this port self-identify; nothing else in
`common/` needed to change to support this.

Point `audio_rx -H`/`-p` at the exact same host/port as `rx -H`/`-p` --
that is what makes the two streams converge on one UDP destination.
`audio_tx -u` must match waybeam's own `outgoing.audioPort`
(`-U` is the bind host, default `127.0.0.1`, matching that field's
loopback contract). `-o` on both audio binaries must agree with each
other and must differ from `tx`/`rx`'s own `-o` (default 3 vs. video's
default 2).

Audio is entirely optional and independently start/stoppable: with
`audio.enabled: false` in waybeam's config (or with `audio_tx`/`audio_rx`
simply not running), video is completely unaffected — this was the whole
point of not sharing state with `tx`/`rx`.

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
boundary, and applied via `GET /api/v1/live/set?video0.bitrate=<kbps>` on
waybeam's loopback HTTP API (`documentation/HTTP_API_CONTRACT.md` in
waybeam_venc — `video0.bitrate` is `MUT_LIVE`, applied without a pipeline
restart). **`/live/set`, not the persisting `/api/v1/set`** — the latter
is what this file actually called until this was found and fixed:
waybeam's own docs describe `/live/set` as built specifically for
"high-cadence automated writers (waybeam-link adaptive bitrate/caps/fps
actuation)... persist-on-set would wear flash and boot into the last
adaptive transient" — exactly this file's own access pattern, and exactly
the two failure modes fixed by switching to it (unnecessary flash writes
on every adaptive change, and a crash/reboot right after a link-quality
dip no longer boots back up pinned at that low bitrate).

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

**Last-resort measure: centre-priority ROI.** waybeam supports
centre-priority horizontal delta-QP bands (`fpv.roiEnabled`/`roiQp`/
`roiSteps`/`roiCenter` — concentrates bits on the middle of frame, where a
pilot is actually looking, at the expense of the edges). `bitrate_ctl.c`
engages it (`fpv.roiEnabled=true`, via `/api/v1/live/set`, at waybeam's
own shipped-default `roiQp`/`roiSteps`/`roiCenter` — this project sets
none of those itself) only when *both* the standing-backlog signal above
*and* `last_applied_kbps < roi_max_kbps` (default 3000, i.e. below
~3Mbit/s) are true — deliberately not on every backlog blip, since
`HTTP_API_CONTRACT.md` documents a real bitrate-overshoot risk from
turning ROI on (`roi_qp`'s delta is subtracted from frame QP, so CBR pays
for it by raising the base QP roughly 1:1, and once `base_qp +
|roi_qp|` passes the encoder's QP ceiling the rate controller saturates
and the target is missed by multiples — measured on this same CV610
backend at ~1.4x at the default `max_qp` ceiling this project leaves
untouched, but 5.8x-12x once `max_qp` is *also* lowered elsewhere, which
this codebase never does). A keyframe-sized burst at an otherwise-healthy
bitrate skips ROI entirely and pays none of that risk; it only engages
once the link has already forced bitrate down near the floor. Disengages
(`fpv.roiEnabled=false`) once backlog has been clear *and*
`last_applied_kbps` has recovered back above `roi_max_kbps` for
`roi_recovery_ms` (default 5000) — hysteresis against flapping right at
the boundary. Any residual overshoot from turning ROI on is just more
backlog, handled by this same loop on its next tick like any other
overshoot source; no separate bitrate compensation is attempted.

**Third signal: LDPC block-error ratio.** MCS and ring backlog above are
both blind to a real failure mode an external firmware
reverse-engineering analysis (static analysis of the AR8030 baseband
firmware and vendor streamer binary, not part of this repo) surfaced: the
stock vendor
streamer's own `fpv_video_buffer_cache_monitor()` also checks
`fpv_bb_is_send_retx_too_many()` and cuts bitrate specifically when the
baseband's own retransmission/FEC layer is under repair pressure — a
"radio actively fixing errors" signal that this project's loop had no
equivalent of. The doc's own reverse-engineered windowed retx-event
controller (`BB_GET_RETX_EVENT_STATUS`/`BB_SET_RETX_EVENT_STATUS`) has
since been added (same "req %x not found" gap `BB_SET_MCS_ITEM` had
before its own patch — see `0031-bb_api-add-missing-BB_RETX_EVENT_STATUS.patch`,
mirrored to sbc-groundstations as `0024-*`) and confirmed live: a `SET`
after boot really does take effect (read back matches every value
written, swept across the full range with no adverse link effect), and
`ar8030-lifecycled` now exposes it as a runtime-tunable
(`POST /api/v1/retx-tuning?win=&busy=&idle=&conti_busy=&conti_idle=`,
mirroring `/api/v1/bandwidth`'s own mailbox pattern, persisted to an
`ar8030.retx` sidecar and re-applied on every connect — vendor defaults
`10,6,4,2,0` apply automatically when nothing's been persisted, since an
unconfigured controller has no meaningful state to ever read a signal
from). But it does **not** feed `bitrate_ctl.c` as a rate-control
signal, and is not expected to: live-monitoring the reply on a real
link found the struct's 131 bytes past its 5 documented fields are not
real per-opcode data at all, just leaked/reused `ar8030d`-internal
buffer memory (one captured sample decoded byte-for-byte as the
daemon's own unrelated error log string). There is no retx-pressure
signal to read from this opcode. What's used instead for the LDPC
signal below, with zero RE risk, is `BB_GET_USER_QUALITY`'s
already-fully-typed `bb_quality_t`
(`snr`, `ldpc_err`, `ldpc_num`, `gain_a`, `gain_b`) — the LDPC block-error
ratio (`ldpc_err`/`ldpc_num`) is a direct measure of how much FEC repair
work the baseband is doing right now, and the closest available proxy to
the vendor's own signal without the unverified opcode. `bitrate_ctl.c`
checks this every ~250ms (`ldpc_ratio_high`, default 0.10) and cuts the
last-applied bitrate by `ldpc_backoff` (default 0.85) the same way the
ring-backlog path does — bypassing hysteresis, cut only (recovery happens
through the normal MCS-driven path's next poll).

Confirmed live on real hardware: the vendor SDK's own `bb_quality_t`
(used by `BB_GET_USER_QUALITY` and `BB_GET_PEER_QUALITY`) was declared
8 bytes but the real per-entry wire size is 16 — every call was silently
truncated by the client library's own reply-length clamp, spamming
"reply datalen=160 exceeds expected 80, truncating" on every single call
once this project started polling it at a useful cadence. Fixed at the
source rather than worked around: `0030-bb_api-fix-bb_quality_t-real-size.patch`
(mirrored to sbc-groundstations as `0023-*`) widens `bb_quality_t` by the
confirmed-real 8 extra bytes (not yet decoded, but confirmed genuinely
populated on the wire rather than padding), so the daemon's real reply is
never truncated. `qualities[0]` is the only array index confirmed
populated on this firmware's single-user-mode link (cross-validated
between air and ground — see this patch's own commit message for the
full investigation); every other index reads zero.
`ar8030-linkctl status` also now prints this same per-user/peer LDPC ratio
(plus `BB_GET_PEER_QUALITY` for the connected peer's own slot) for
visibility outside the control loop.

## Frame-shm ring: surviving a waybeam restart

`ar8030-transport-tx` attaches to waybeam's frame-shm ring
(`venc_frame_ring_attach()`) once at startup and keeps that `mmap()` for
its whole lifetime. Some waybeam config changes require a pipeline
restart to take effect, and waybeam can also simply crash and be
restarted by its own init script — and `venc_frame_ring_create()`
(waybeam's producer side) does `shm_unlink()` then
`shm_open(O_CREAT|O_EXCL)` on **every single start** ("stale-ring guard"
in its own comment), never reopening an existing object. `shm_unlink()`
does not invalidate an already-`mmap()`'d region in another process —
exactly like unlinking a regular file someone still has open — so an
already-attached consumer's mapping keeps working precisely as before,
now reading a permanently orphaned copy of the ring that the new
producer will never write to again. Confirmed live: no crash, no signal,
no error — just a read timeout that recurs forever, since at this
project's normal fps a healthy ring almost never goes 200ms without a
fresh frame.

**Fix:** `tx/main.c`'s `stat_named_shm()` does a fresh `shm_open()` +
`fstat()` by name (independent of the existing mapping) on every read
timeout and compares the inode against the one recorded at the last
successful attach — the same technique `tail -F` uses to notice a
rotated log file. A mismatch (or the name resolving to nothing at all,
mid-restart) means waybeam restarted; the read loop destroys the stale
mapping, retries `venc_frame_ring_attach()` until it succeeds (mirroring
the startup wait), reallocates the read buffer if the new ring's slot
size differs, and republishes the new pointer.

**A real race, caught on the very first live test against an actual
waybeam restart, not a hypothetical:** `bitrate_ctl_run()` runs on its
own thread and reads the same ring pointer concurrently every ~100ms
tick for its own backlog check (see above). The first version of this
fix destroyed (and freed) the old ring, then only published the new
pointer to that thread *after* a successful reattach — leaving a window
where the bitrate thread could load the stale pointer and dereference
already-freed memory. Segfaulted the whole process (one thread crashing
takes the rest down with it) on the very first real restart tested
against, immediately after logging "waybeam restarted, reattaching" and
before "reattached to ring". Fixed by publishing `NULL` to the shared
ring pointer *before* freeing the old ring, not after attaching the new
one — `NULL` is already a supported state for that field (it disables
the backlog check entirely), so the other thread just skips its check
for the brief duration of the reattach instead of ever seeing a dangling
pointer. Confirmed clean across two consecutive real `waybeam restart`
calls afterward: no crash, brief ring-fill spike immediately
absorbed by the existing backlog throttle, streaming back to the
pre-restart bitrate within a few seconds each time.

## ar8030d connection: surviving a daemon restart

Both `tx/main.c` and `rx/main.c` only ever called `ar8030_link_connect_retry()`
once, before their main loop. Past that point neither noticed if `ar8030d`
itself crashed or was restarted -- `link.dev`/`link.sockfd` just went
stale, and every subsequent `bb_ioctl()`/`bb_socket_write()`/
`bb_socket_read()` call on them would fail forever with no automatic
recovery. The same shape of gap as the frame-shm ring above, one layer
down.

**Fix, shared by both sides via `common/ar8030_link.c`:** a periodic
`ar8030_link_is_alive()` health check (`BB_GET_STATUS`, a few seconds
apart) as a *separate* signal from data-path timeouts -- a
`bb_socket_write()`/`read()` timeout looks identical whether the RF link
is merely saturated (self-clearing) or the daemon itself is gone (never
clears without a fresh reconnect), and conflating the two either
reconnects too eagerly on ordinary congestion or too slowly on a real
outage. On failure: `ar8030_link_reconnect_retry()` (close + retry
connect, same semantics as the startup call), then the caller re-resolves
whatever it needs (tx re-resolves its peer's connected slot if `-s auto`
was requested -- not guaranteed to land on the same slot as before the
restart; rx always targets the fixed `BB_SLOT_AP`) and reopens the data
socket.

**tx's `bitrate_ctl` thread reads the same `link`/`ring` concurrently --
a real use-after-free, caught on the very first live test, not a
hypothetical.** `ar8030_link_t.dev` is read by that thread's own
`BB_GET_MCS` poll every tick. The first version of the daemon-reconnect
fix closed and freed the old `dev` handle, then only atomically published
the new one *after* a successful reconnect -- leaving a window where the
bitrate thread could load the stale pointer and dereference already-freed
memory. Fixed the same way as the ring fix above: every access to
`ar8030_link_t.dev` (and `bitrate_ctl_cfg_t.slot`, which tx also updates
post-reconnect) now goes through `__atomic_*()`, and `ar8030_link_close()`
publishes `NULL` *before* freeing the handle, not after -- `NULL` is
already a value that thread has to treat as "skip this tick" regardless,
so publishing it first closes the window at zero extra cost.

**A second real bug, also caught live: breaking out of tx's main loop on
a fatal reconnect failure without setting `g_stop` first.**
`pthread_join(bc_thread, ...)` runs right after the loop exits, but
`bitrate_ctl_run()`'s own loop only exits once `*cfg->stop_flag` is true
-- so the process hung forever (`bitrate_ctl` still ticking, `pthread_join`
waiting on it, nothing progressing) instead of actually exiting the one
time this path was live-tested. Fixed: `g_stop = 1` before every `break`
that represents a real, unrecoverable failure (not one that already
implies `g_stop` was set, like the daemon-wait/slot-wait loops returning
early because the process is shutting down anyway).

**A third real bug, also only visible under a real daemon kill: reopening
the data socket right after a reconnect routinely failed, with a fix that
took two attempts to actually work.** The daemon's own log showed the
open succeeding, then being torn back down again about a millisecond
later -- a daemon killed abruptly never runs its own `bb_socket_close()`
teardown, so the fresh daemon instance's view of that slot/port can be
left in a stuck "already opened" state. First attempt:
`ar8030_link_force_close_socket()` (`BB_FORCE_CLS_SOCKET`, matching
linkctl's own `force-close-socket`) called right before reopening --
confirmed live this **did not** clear it (`BB_FORCE_CLS_SOCKET` itself
returned `-2`). What actually worked: `ar8030_link_force_close_all_sockets()`
(`BB_FORCE_CLS_SOCKET_ALL`, matching linkctl's `force-close-all`) in its
place, plus a short retry loop around the open call itself (up to 5
attempts, 500ms apart) -- matching every other step already in this same
connect/reconnect sequence (`ar8030_link_connect_retry()`,
`resolve_connected_slot()`), none of which assume their first attempt
succeeds either. Applied to **both** the reconnect path and the original
startup path -- confirmed live that a fresh startup can hit the exact
same stuck state if a prior run crashed uncleanly and the device wasn't
rebooted since.

**Confirmed clean on real hardware, both sides, `ar8030d` killed and
restarted with the exact same binary (the actual scenario this feature
exists to handle):**
- **tx** (air, SDIO): two consecutive kills, both recovered fully within
  the same process (no crash, no restart) -- detect, reconnect, re-resolve
  slot, reopen socket, resume streaming at the pre-outage bitrate. A
  handful of frames go `incomplete` during each outage (expected: nothing
  can be sent while genuinely disconnected) and the count stops growing
  the moment it recovers.
- **rx** (ground, USB): one kill, same result -- detect, reconnect, socket
  reopened on the first attempt this time, RTP output resumed at the
  pre-outage rate, `dropped=0` throughout.

Both required **redeploying the whole daemon+lib+client triple together**,
not just the client binary -- see "Redeploying `ar8030-linkctl`" below
in Buildroot integration for why a locally-rebuilt client can silently
stop matching whatever's actually flashed, and why mixing artifacts from
different builds is exactly the failure mode this project's own
non-reproducible daemon build makes easy to hit by accident.

## Build

### Everything, auto-detecting both cross toolchains

```sh
make            # cross-builds tx/rx plus audio_tx/audio_rx (see "Embedded audio" above)
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

### Redeploying `ar8030-linkctl`/`ar8030-transport-{tx,rx}`: the daemon+lib+client triple must match

**The vendor `ar8030` SDK's build is not reproducible, even though its
source is pinned to a fixed commit.** `AR8030_VERSION` in
`package/ar8030/ar8030.mk` names one exact git SHA, and this project's
own patches on top of it are version-controlled too — but the *toolchain*
building it isn't: `builder/`'s own `OpenIPC/firmware` clone (and
`sbc-groundstations`' own Buildroot tree) aren't pinned to a fixed commit,
and rebuilding on a later day can pull a different upstream toolchain/
Buildroot revision. Confirmed live: rebuilding the `ar8030` package twice
in the same day, same machine, same pinned SDK commit, produced two
different `libar8030_client.so` binaries (different md5sum) both times.

**This means a locally-rebuilt `ar8030-linkctl` (or `ar8030-transport-tx`/
`-rx`) can silently stop matching whatever `ar8030d`/`libar8030_client.so`
is actually flashed on the device.** Confirmed live as a real, reproduced
segfault: a client built against a newer/different SDK snapshot corrupted
memory partway through a `bb_ioctl()` call whose reply size the two sides
disagreed on. The struct that happened to differ (`bb_get_chan_info_out_t`)
turned out to have an unrelated, genuine overflow bug of its own once
investigated further (see `linkctl/main.c`'s own comment on why it never
calls `BB_GET_CHAN_INFO`) — but the ABI-mismatch risk that made it hard to
diagnose in the first place is real and general, not specific to that one
struct.

**Practical rule: always rebuild and redeploy the whole daemon+lib+client
set together, verified by hash, never just the one binary you meant to
change.** Concretely:

- `builder/`'s tree has per-package isolation (`BR2_PER_PACKAGE_DIRECTORIES`)
  — use `builder/package.sh ar8030` to force a truly clean rebuild of the
  daemon+lib (wipes both its `per-package/` and `build/` caches, unlike a
  manual `rm -rf build/ar8030-*` alone, which was confirmed live to leave
  a stale `per-package/ar8030` untouched), then `builder/package.sh
  ar8030-transport-tx` (with `AR8030_TRANSPORT_TX_OVERRIDE_SRCDIR` set) so
  the client links against that exact same rebuild.
- `sbc-groundstations/` has no per-package isolation and no `package.sh`
  equivalent — its single global `staging/`/`target/` reflects whatever
  was built into it most recently. `rm -rf output/<defconfig>/build/ar8030-*`
  then `make -C output/<defconfig> ar8030-dirclean ar8030-rebuild
  ar8030-transport-rx-dirclean ar8030-transport-rx-rebuild` (with
  `AR8030_TRANSPORT_RX_OVERRIDE_SRCDIR` set) achieves the same thing.
- Before deploying anything, `md5sum` the freshly-built
  `libar8030_client.so` the client actually linked against (its build log
  names the exact path) against what the client binary itself hashes to,
  and treat a plan to deploy only the client while leaving a
  differently-built daemon/lib in place as unsafe by default.
- Expect to need a full `ar8030d` restart after swapping the daemon/lib,
  not just the client — and expect that, on the SDIO (air) side
  specifically, a plain process restart with the kernel module *not*
  reloaded can leave the chip unable to re-handshake with a differently-
  built daemon instance (confirmed live: `"no AR8030 device known to the
  daemon"` persisting until a full device reboot). A full reboot after
  swapping the daemon/lib is the reliably clean path there; the USB
  (ground) side's own `artosyn_drv` kernel module re-enumerates the
  device as part of a plain daemon restart already, so a reboot is not
  needed there.

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

## SDIO chardev: implemented, not yet proven (superseded -- see `kmod/`)

**Update: this whole investigation's conclusion is a full clean-room
rewrite, `kmod/artosyn_drv.c`, which resolved it.** Confirmed on real
hardware: zero stalls, zero bad-socket-pack messages, sustained
~18.5Mbit/s, over 439MB transferred across a ~190 second run -- the
first time in this entire investigation SDIO mode has run completely
clean, matching the vendor's own reference performance. See "Clean-room
rewrite: `kmod/artosyn_drv.c`" below for the full story and what's left
before this fully replaces the patched combined driver in normal use.
Everything below this point is the investigation history that led
there -- kept for the reasoning, not as current guidance; don't restart
patching the old combined driver (`ascent/8030_sdk/yz_host_drv/driver/
linux`) without first checking whether `kmod/artosyn_drv.c` already
covers what you're trying to fix.

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
- **Root-caused (see "Isolation test and the real root cause" below):**
  the "intermittent firmware-download failure on a freshly rebuilt
  module" mystery from earlier in this investigation was never a
  Buildroot/toolchain issue at all — it was this project's own working
  copy of `driver/linux/bus/sdio.c` (`ascent/8030_sdk/yz_host_drv/`,
  hand-edited directly across many sessions instead of via the numbered
  patch stack) having silently drifted out of sync with the patch
  series not once but *twice* (patches `0004` and `0003`, found on two
  separate occasions). **Before deploying any freshly rebuilt
  `artosyn_drv.ko` to real hardware, verify it carries all three SDIO
  device-ID aliases** (`grep -a alias= the.ko` or `modinfo`):
  `sdio:c*v4152d8031*`, `sdio:c*v1D6Bd8030*`, `sdio:c*v4152d8030*` — a
  missing `4152d8031` (patch `0003`'s fix) reproduces exactly this
  failure: firmware uploads fine, the chip re-enumerates from its boot
  identity (`4152:8030`) to its post-firmware-push identity
  (`4152:8031`), and `sdio_artosyn_probe()` never gets invoked again
  because there's no matching `sdio_device_id` entry for it — permanent
  `/dev/artosyn_sdio` / `/dev/ar_mdev0` absence, every time, not
  flaky hardware. The safe way to resync a drifted working copy: `git
  checkout` it back to pristine and cleanly re-apply `0001`-`0013` in
  order (skip `0005`, `.patch_skip`) — never hand-restore a patch from
  memory into the working copy and treat that as equivalent to the
  patch actually being applied.
- **Do not `cat` (or otherwise read) `/proc/ar_drv/dev0/dbg`** (or
  presumably its sibling proc files under `/proc/ar_drv/dev0/`) on real
  hardware — confirmed live to crash the kernel outright (`Unable to
  handle kernel NULL pointer dereference`, `proc_write+0x32` called from
  `proc_reg_read_iter`: this proc entry's read path is wired to the
  write handler by mistake, a genuine pre-existing bug in this vendor
  driver's own `ar_proc.c`). Setting `dbg_log_level` via the documented
  `echo dbg_log_level=N > ...` write interface is presumably still fine
  (untested since the crash) — just never read it back.

### Isolation test and the real root cause

The remaining `"bb_socket_write made no progress"` stalls were finally
isolated to a specific half of the stack by swapping components one at
a time, always from a genuinely clean boot (auto-start scripts renamed
out of `/etc/init.d/S??*`, never a live `rmmod`/`insmod` chain):

| Kernel module | Daemon | Result |
|---|---|---|
| Ours | Ours | Frequent stalls, every ~15-20s, self-recovering blips |
| Vendor's `artosyn_sdio.ko` | Vendor's `daemon_sdiov12` | Zero stalls at 18-19Mbit, sustained |
| **Vendor's** | **Ours** | **Zero stalls across 22,000+ writes, including a 33KB chunk, even under forced bitrate overshoot** |

Swapping *only* the kernel module (keeping our own daemon, including
the fix below) eliminated the stalls. **The bug is in our kernel
driver's `/dev/artosyn_sdio` chardev implementation (`0011`/`0012` on
top of `bus/sdio.c`), not the daemon.**

**A real (but minor) daemon-side fix, landed regardless
(`0013-daemon-act-on-wanted_pos-hint-outside-idle-state-too.patch`):**
`dev_dat_so_write_proc()`'s handling of the chip's `-0x108` "wanted_pos"
idle-notify (telling the daemon what position to resend from if
anything was dropped) only acted on it when `sock_sta ==
sock_wait_usb_data` specifically. Live-captured evidence: the chip can
report a dropped write within ~2ms via this message, well before the
client's own timeout, but a race between `_sock_dev_pack_make()`
(tx-side thread) and this reply handler (rx-side thread) over that same
field meant the hint got silently discarded if it landed while
`sock_sta` was still `sock_can_send_data` — the socket then just sat
there until some unrelated later event happened to re-check the same
stale hint, observed to take up to ~2.5s (exactly matching
`bb_socket_write()`'s own client-side timeout). Fix: also act on the
hint while `sock_can_send_data`. Confirmed via live daemon-log
correlation to fire exactly as designed — but confirmed via the
isolation test above to explain only a minority of stalls, not the
dominant cause. Keep it; don't expect it alone to fix throughput.

**A promising-looking kernel-side idea that turned out to be a dead
end — don't retry this without solving the prerequisite first:**
Ghidra-decompiling the vendor's `artosyn_sdio.ko` poll() fop
(`FUN_00010220`) found it does something ours never did: when neither
read nor write is currently ready, it calls its irqhandler equivalent
*synchronously, right there*, with the SDIO host already claimed — an
active, ~10Hz self-healing re-check against a lost interrupt, since the
daemon's `sdio_ev_loop()` calls `poll()` on this fd continuously. Ours
only had the narrower `WORKAROUND_FOR_INTERRUPT_LOST_ISSUE` fallback
inside `artosyn_sdio_write()`, nothing on the read side. Porting this
literally (`sdio_chardev_poll()` calling `artosyn_sdio_irqhandler()`
when neither condition is true) made things dramatically *worse* — a
hard, permanent wedge (throughput pinned to 0) instead of the
intermittent stall it was meant to fix.

**Root cause of the poll() regression is not fully confirmed — two
candidate explanations, correcting an earlier over-confident guess in
this same investigation:**
- *Initially suspected:* the vendor's own poll() only calls its
  irqhandler equivalent when **both of its two "msg_valid"-style flags
  are also clear**, and this project's `dev->msg_valid[]` array is set
  to 1 on a mailbox event but **never cleared anywhere in this entire
  driver** (grepped the whole `driver/linux` tree — confirmed true).
  Decompiling `artosyn_unlocked_ioctl` in the vendor's `.ko` found
  exactly where the vendor's own userspace is expected to clear it: a
  `READ_MESSAGE`-style ioctl (`_IOWR('v', 2, ...)`, same command number
  this project's own `sdio.h` already defines) that queries a mailbox
  channel's pending message and clears `msg_valid[channel]` as a side
  effect. **But this project's own `sdio.c` has no `.unlocked_ioctl`
  fop at all**, and the daemon's SDIO backend
  (`daemon/dev8030/sdio8030/sdio_dev.c`) never calls `ioctl()` on this
  chardev either — so `message[]`/`msg_valid[]`/`mailbox_q` are, as far
  as could be confirmed, entirely dead/unused plumbing in this
  project's actual data-plane usage pattern. That undercuts the
  "corruption via the msg_valid gate" theory: the *unconditional*
  version tested never consulted `msg_valid` at all, so its clearing
  status shouldn't have mattered to that specific failure.
- *More likely, not yet tested in isolation:* simple SDIO bus
  contention. `artosyn_sdio_irqhandler()` does at least one
  `sdio_readb()` MMIO transaction (a real bus round-trip, not free),
  and `sdio_claim_host()` serializes the whole bus against the actual
  `sdio_memcpy_toio()`/`_fromio()` data transfers. The daemon's
  `sdio_ev_loop()` calls `poll()` far more often than its nominal
  100ms timeout suggests once bytes are actually flowing (it loops
  back immediately after any read/write completes) — plausibly
  hundreds of times a second under the ~18Mbit load this was tested
  at. Adding one extra bus transaction per poll(), right when the bus
  is already busiest, could alone explain "throughput instantly
  collapses to 0" without any logic corruption at all.
- **Before retrying this idea**, isolate which of the two it actually
  is (e.g. instrument a call counter on the added irqhandler call and
  correlate its rate against the stall onset, independent of whether
  `msg_valid` is wired up) rather than assuming either explanation.

**Also checked and ruled out:** whether `artosyn_sdio_reg_check()` and
`artosyn_sdio_irqhandler()` — this project's two hand-maintained copies
of the same register-check logic (one for the real IRQ, one for the
`WORKAROUND_FOR_INTERRUPT_LOST_ISSUE` workqueue fallback, which must be
kept in sync by hand — see `0004`'s own commit message, which had to
fix both) — had silently drifted apart from each other, the same way
the working copy drifted from the patch stack. Diffed both functions
line-by-line (comments and blank lines stripped): aside from expected
`sdio_claim_host()`/`sdio_release_host()` ownership differences
(irqhandler runs with the host already claimed by the real IRQ path;
reg_check must claim it itself from workqueue context) and cosmetic
log-level/stats-counter differences, they are functionally identical —
both correctly carry `0004`'s single-byte decode fix and `0011`
UPDATE 4's wake-both-queues fix. Not the bug.

**A second real, confirmed improvement
(`0014-sdio-read-wait-for-real-interrupt-not-100ms-poll.patch`):**
Re-reading `artosyn_read()` with the isolation test's narrowed focus
found it had never received `0012`'s own fix — it still ran the exact
same `msleep(10)` × 10-retry, 100ms-capped anti-pattern `0012`'s own
commit message diagnosed and replaced on the write side, right down to
a commented-out `wait_event_interruptible_timeout()` call immediately
above the loop, abandoned for reasons lost to history (the same shape
of abandoned attempt `0012` found on the write side too). Fixed the
same way: a real `wait_event_interruptible_timeout(dev->rx_q, ...,
SDIO_READ_WAIT_MS)` (2000ms), with the same courtesy `reg_workqueue`
kick beforehand. **Confirmed on real hardware, same clean-boot
methodology, same 60s window at ~14Mbit:** stalls dropped from ~18 to
**3** — roughly a 6x reduction — and every one of the 3 remaining
stalls self-recovered smoothly (throughput bounced straight back,
no hard wedge). This is real, measurable progress, not a full fix —
the user's own assessment: "better but not fully there yet." The
theoretical concern going in (that `sdio_ev_loop()` always gates
`read()` behind a successful `poll()` first, so this fix "shouldn't"
matter much) turned out to undersell it — there's evidently some real
path under sustained load where the interrupt-driven wait actually gets
exercised, not just the trivial single-poll-then-read case this
project's own analysis was based on. Whatever residual mechanism still
causes the remaining ~3 stalls per 60s is now the actual open
question — likely something
closer to the daemon-visible symptom (a completion ack not arriving
promptly) than a missed-interrupt-at-the-driver-level issue, since the
two known interrupt-loss workarounds (read and write side) are now
both patched symmetrically with no more asymmetry between them left to
find at that level.

**A third real, confirmed fix for a genuine hard-lockup mechanism
(`0015-daemon-clamp-force-update-write-address-to-buf-head.patch`):** a
tight, sub-second-granularity live capture (poll the tx log every 0.2s,
snapshot the daemon's own debug-pad log the instant a stall is seen)
caught one of `0014`'s remaining stalls turning into a genuinely
*permanent* lockup — throughput pinned at exactly 0.00 Mbit/s, every
subsequent write timing out, no self-recovery at all (unlike the
brief, self-healing hiccups `0014` left behind). The captured log
showed `sock_dev_push_data()`'s own sanity-check warning firing
repeatedly with **`buf_head_index` greater than `buf_wr_index`** — an
inverted state that should be structurally impossible (confirmed
received can never exceed dispatched-so-far). Once inverted,
`sock_dev_need_write()`'s own `(uint32_t)(buf_wr_index -
buf_head_index)` computation underflows to a huge bogus ring-buffer
offset, permanently breaking its "is there anything to send" check for
that socket regardless of how much genuinely new data piles up behind
it (visible in the same capture: `add_new_wr_node()`'s own `cur`
climbing normally the whole time, completely decoupled from the frozen
`buf_wr_index`). Traced to the one unguarded write site: every other
place that sets `buf_wr_index` (the normal ack path, the `-0x108`
force-resend path `0013` already touches) explicitly keeps it `>=
buf_head_index`; the `-0x107` "force update write address" handler
accepted the chip-reported position completely unguarded, and the
capture caught a burst of dozens of `-0x107` messages in a few
milliseconds all reporting the same already-superseded (stale) position.
Fixed by clamping. **Confirmed on real hardware: this specific
inverted-pair lockup signature has not reproduced since.**

**A second, distinct hard-lockup mechanism found in the same session —
still open, not fixed by `0015` or anything else here:** a *different*
capture (same tight methodology, a later test run) caught a hard
lockup with the **opposite, valid-looking** state: `buf_wr_index`
correctly *ahead* of `buf_head_index` by a large, stuck gap (~500KB in
the observed case), with `sock_dev_push_data()` logging `"warning loss
rpc data ... push=0"` on every new client write — the ring buffer was
completely full and could accept nothing more, because nothing had
confirmed-received (`buf_head_index`) any of that backlog in a long
time. Tracing the daemon's own debug-pad log back to the last activity
on this socket found only recurring `-0x108` "written pos / wanted
pos" messages (the periodic idle-heartbeat, ~every 2.5s) with
**`wanted_pos` frozen at the same stale value** across 12+ seconds,
even while the chip's own `written pos` in that same message kept
climbing normally. Since that frozen `wanted_pos` was already *behind*
`buf_head_index`, the (correct, already-guarded) force-resend condition
never fires — the daemon isn't wrongly declining to act, there's just
nothing else in this codebase that flushes a newly-arrived backlog once
the chip's own idle-heartbeat stops reporting anything actionable.
Confirmed this session it's unrelated to thermal issues (chip
temperature read a normal 63.8°C via `caddx-ascent-lite-temp` with the
fan running during the capture). This may be a genuine chip-firmware
behavior (its own `wanted_pos` tracking getting stuck) rather than
something fixable purely in this daemon's own bookkeeping — or there
may be a host-side trigger for it not yet identified. **This is the
actual next thing to chase**, and unlike the `0015` bug, doesn't yet
have a clear, single unguarded-write-site smoking gun — it needs either
a way to reproduce it more reliably for further live capture, or
Ghidra-decompiling more of the chip's own RTOS firmware (out of scope
so far — this investigation has stayed on the host-side kernel driver
and daemon) to understand what `wanted_pos` staleness actually means
from the chip's side.

**Tried and correctly reverted — do not retry this without new evidence:**
Ghidra-decompiling the vendor's real `daemon_sdiov12`'s equivalent of
`dev_dat_so_write_proc()` (`FUN_00013be4`) found its `-0x108` case body
is *only* a log call — no force-resend, no `buf_wr_index` write, no
state transition at all, unlike this project's own `-0x108` handler
(the whole mechanism `0013` patched a race in). Combined with the
isolation test's own proof that the vendor's real kernel+daemon combo
runs completely stall-free with no such logic, this looked like strong
evidence the whole force-resend mechanism was an unnecessary — and,
given its role in the `0015` bug, possibly actively harmful — piece of
this codebase that the vendor's own production build simply doesn't
carry. Removed it (matching the vendor exactly: log only) and rebuilt.
**Live-tested and found to be a real regression, not an improvement:**
with the force-resend logic gone, a *routine* one-off data-gap right at
the very start of a session (chip's own `wanted_pos` freezing at a
small position near the start, exactly like the still-open mechanism
above) became a **guaranteed, permanent, unrecoverable lockup every
time** — the mechanism this patch removed turns out to be exactly what
was recovering from that gap in the working `0013`+`0014`+`0015`
baseline (rare, self-healing blips) the rest of the time. Reverted
immediately; never landed as a numbered patch. Takeaway: the vendor's
own build not needing this logic doesn't mean *this* codebase's version
of the surrounding state machine doesn't rely on it — the two have
diverged enough elsewhere (this whole investigation's history is full
of examples) that "the vendor doesn't have this" isn't sufficient
justification for removing something on its own; only build on this
finding again with a fix that's *narrower* than a full removal (e.g.
something that makes the still-open freeze detectable/recoverable
without discarding the mechanism wholesale), and re-confirm on real
hardware before trusting it.

**Next steps, updated:** the isolation test firmly narrows the search
to our own kernel driver's SDIO read/write/interrupt-handling logic
(`bus/sdio.c`) specifically — the daemon, the size-decode registers
(`0004`), and the device-ID re-enumeration handling (`0003`) are all
now confirmed *not* the cause. The vendor's `artosyn_sdio.ko` has ~30
non-thunk functions total (small enough to decompile exhaustively via
Ghidra-mcp) — most of the ones relevant to sustained-throughput
reliability (`artosyn_write`, `FUN_000106de` the write worker,
`artosyn_read`, `artosyn_sdio_irqhandler`, `FUN_00010090`/`FUN_000100d0`
the read/write conditions, `artosyn_poll`) have already been decompiled
and compared line-by-line against ours with no further differences
found beyond the poll() one above (which isn't safely portable yet).
Not yet decompiled/compared: `sdio_artosyn_probe` (the biggest
function, firmware download + initial setup — a good next candidate,
since a *setup-time* difference could plausibly cause a
runs-for-a-while-then-degrades symptom that a pure read/write logic
diff wouldn't), `artosyn_unlocked_ioctl`, `FUN_00010a04` (called from
between `artosyn_close` and `artosyn_read` in the address layout —
likely a firmware-chunk-send helper worth checking against this
project's own `sdio_rom_send()`), and the `proc_*`/`artosyn_root_proc_*`
family (lower priority — debug-interface only).

## Clean-room rewrite: `kmod/artosyn_drv.c`

After the daemon-side fixes above (`0013`-`0015`) narrowed the SDIO
stalls from severe (permanent wedges) to rare and self-healing, but not
zero, the decision was made to stop bug-hunting the existing patched
combined driver (`ascent/8030_sdk/yz_host_drv/driver/linux`, DRV+SDIO+
USB in one module, incrementally patched across this whole
investigation) and instead do a full clean-room rewrite of just the
SDIO chardev, informed by everything learned so far. `kmod/` in this
repo is that rewrite: a small (~700 line), standalone out-of-tree kernel
module, built against the vendor SDK's own shared `bus/sdio.h` (ioctl
commands, protocol structs/constants) via `kmod/Makefile`'s
`AR8030_SDK_DRIVER_INC`.

**Result, confirmed on real hardware:** zero `"bb_socket_write made no
progress"` stalls, zero `"recv bad socket pack"` messages, sustained
~18.5Mbit/s, 439MB+ transferred over a ~190 second run, `incomplete=0`
the entire time. The first completely clean SDIO-mode run anywhere in
this investigation, matching the vendor's own real `artosyn_sdio.ko`'s
reference performance rather than just approaching it. (That first run
didn't happen to push past ~10-12Mbit/s -- see "Follow-up: the
~10-12Mbit/s ceiling, root-caused and fixed" further down for a real
regression only a higher-bitrate soak surfaced, and its fix.)

### Design decisions that came out of the investigation history

- **One register-drain routine, not two.** The old combined driver
  carried two independent, hand-duplicated copies of the mailbox/rx-
  ready/tx-ready register-reading logic (`artosyn_sdio_irqhandler()` for
  the real IRQ, `artosyn_sdio_reg_check()` as a workqueue-based fallback
  for `WORKAROUND_FOR_INTERRUPT_LOST_ISSUE`) that had to be kept in sync
  by hand -- and didn't always stay in sync (`0004`'s decode bug existed
  in both copies; `0011`'s shared-waitqueue fix had to touch both).
  `kmod/artosyn_drv.c` has exactly one such routine
  (`artosyn_check_events()`), called from both the real IRQ handler and
  from `poll()` when idle -- matching the vendor's own actual
  architecture (confirmed via Ghidra decompilation of their real
  `artosyn_sdio.ko`), not an approximation of it grafted onto a
  different structure.
- **`poll()` self-heals, safely this time.** Porting "call the drain
  routine from `poll()` when idle" onto the *old* combined driver's dual-
  path architecture caused a severe regression (see the investigation
  history above) -- most likely SDIO bus contention between the two
  independent paths under saturated throughput. With only one path here,
  that specific failure mode doesn't apply, and the self-heal is exactly
  what gives this rewrite a real, low-latency recovery from a missed
  hardware interrupt on *both* the read and write sides, not just the
  narrower one-sided fallback the old driver had wired only into its own
  write path.
- **Patient, real interrupt-driven waits from the start** (matching
  `0012`/`0014`'s already-validated `wait_event_interruptible_timeout`
  approach), not the 100ms-capped `msleep`-and-repoll pattern the vendor
  SDK's own upstream source still carries in both `artosyn_sdio_write()`
  and `artosyn_read()`.
- **`rom_mode` bypass, the one real bug this rewrite's own first live
  test hit:** during boot-ROM firmware upload, the chip doesn't generate
  the normal TX/RX-ready mailbox events at all, so waiting on them (as
  the normal read/write path does) times out on literally the first
  firmware chunk. Missed this initially (it's easy to, reading the
  vendor's decompiled probe/write functions in isolation without
  noticing this exact interaction); the existing combined driver's own
  `dev->rom_mode` field (a real, working, previously-unremarked-upon
  detail of the code this whole investigation had been reading past for
  weeks) was the tell once the symptom (every firmware chunk send
  failing with `-EIO`) pointed back at `artosyn_do_write()`'s own wait.
- **Firmware download chunking logic reused nearly verbatim** from this
  project's own already-proven `sdio_rom_send()`/`oal_init_fw()` (the
  "SD"-magic 12-byte header, block-alignment/residue handling, the
  `STRU_SPL_HEADER` firmware image format) rather than re-derived from
  the vendor's own considerably denser decompiled equivalent
  (`FUN_00010a04`) -- no reason to re-invent something already validated
  by this same investigation's extensive real-hardware testing.
- **Deliberately out of scope:** the OAL/DRV-mode multiplexed transport
  (`/dev/ar_mdev<N>`) is untouched, separate, already-working code from
  the existing combined driver -- this module doesn't coexist with it
  (only one driver can own the physical SDIO function at a time); it's a
  drop-in alternative for SDIO-mode use. The `proc_*`/debug-interface
  family from the vendor's real driver was also left out (lower
  priority, not needed for data-plane operation) -- `READ_MESSAGE`/
  `WRITE_MESSAGE`/`READ_BYTE`/etc. ioctls *are* implemented (matching
  `bus/sdio.h`'s existing command definitions) since those looked
  possibly relevant to mailbox-channel messaging, though nothing in this
  project's own daemon currently calls them either.

### Build and test

```
cd kmod
make print-config   # confirm KDIR/CROSS_COMPILE/AR8030_SDK_DRIVER_INC auto-detection
make                # -> artosyn_drv.ko
```

Deploy exactly like any other kernel-module change tested in this
investigation: **always from a genuinely clean boot** (auto-start
scripts renamed out of `/etc/init.d/S??*` first, never a live module
swap while the old combined driver's own `/dev/artosyn_sdio` is in use
-- they can't coexist), then:

```
insmod artosyn_drv.ko fw_name=ar8030/ar8030.img cfg_name=ar8030/ar8030.json
ar8030d -i 1 > /dev/null 2>&1 &
ar8030-pair --no-persist --skip-if-connected -c /lib/firmware/ar8030/ar8030.json
ar8030-linkctl bandwidth 20 -d tx -s auto -w 20
ar8030-transport-tx -v
```

**Never redirect `ar8030d`'s stdout to a file when testing this by hand**
(`> some.log`, not `> /dev/null`) -- confirmed live to OOM-kill an
unrelated process (`waybeam`) on a real air unit. `ar8030d` has no log-
level flag and defaults to verbose per-RPC/per-socket-event tracing on
stdout; `0009-com_log-cap-daemon-log-file-size.patch`'s own commit
message already documents this exact failure mode for the daemon's
*internal* file-based log (`/var/log/ar8030/daemon_log/*.log`, capped at
2MB by that patch) filling the 28.5MB tmpfs-backed `/tmp` on this same
Hi3516CV610 air unit and starving an arbitrary victim process of memory
-- stdout mirrors the same volume of chatter but isn't covered by that
cap, so capturing it to a file during manual testing reproduces the
identical OOM by a different path. `/dev/null` it, or `-l <level>` if a
future daemon build adds one; don't accumulate it.

- **Only one clean-boot test run so far.** Extraordinary result, but
  one run -- re-confirm across multiple clean boots, different link
  conditions (lower MCS, weaker signal), and a longer soak (this run was
  ~190s; run it for the length of an actual flight) before fully
  trusting it.
- **Buildroot packaging is done.** `ar8030-transport-tx.mk` now builds
  `kmod/artosyn_drv.c` via Buildroot's own `kernel-module` infra
  (`AR8030_TRANSPORT_TX_MODULE_SUBDIRS = kmod`, `AR8030_SDK_DRIVER_INC`
  pointed at `$(AR8030_DIR)/driver/linux/bus` -- the `ar8030` package's
  own extracted+patched source tree, a standard Buildroot cross-package
  reference), and its own `S65ar8030-transport-tx` init script now loads
  it, starts `ar8030d -i 1`, and runs the pairing/autoreconnect loop --
  absorbing what used to be `ar8030`'s own `S60ar8030`. The `ar8030`
  package itself dropped kernel-module building entirely (`Config.in`'s
  host-bus `choice` and `BR2_PACKAGE_AR8030_INIT` are gone, along with
  the six patches that only ever fixed the old combined driver:
  0003/0004/0010/0011/0012/0014). Confirmed on the actual build host:
  `artosyn_drv.ko` compiles and installs cleanly through this package,
  `USING_8030DRV=OFF` correctly reaches `ar8030`'s CMake configure (no
  more `drv8030`/`oal_mdev` build target), and the installed module's
  own alias table has all three device IDs
  (`sdio:c*v4152d8031*`/`c*v1D6Bd8030*`/`c*v4152d8030*`). Not yet
  confirmed: a full from-scratch image build + real-hardware flash test
  of this wiring (blocked, at the time of writing, by an unrelated
  pre-existing issue: the `waybeam` package's pinned git commit no
  longer resolves against its upstream remote -- a separate package,
  untouched by this change).
- **DRV-mode is gone, not just deprioritized.** `artosyn_drv.c` never
  implements `oal_mdev.c`'s multiplexing, and `ar8030`'s own `Config.in`
  now says so plainly: only `-i 1` (SDIO) has a kernel-side counterpart
  left to open. This was a deliberate choice (confirmed with the
  project owner), not a temporary gap -- there is no "old driver" left
  to fall back to any more, so there is no coexistence/switchover
  question left to resolve.
- **`READ_MESSAGE`/mailbox-channel ioctls are implemented but
  untested** -- nothing in this project's own daemon calls them, so
  they've only been checked to compile and match the vendor's protocol
  shape on paper, not exercised on real hardware.
- **No automated test/CI** for this module at all yet (the userspace
  `test/roundtrip_test.c` in this repo doesn't cover kernel code).

### Follow-up: the ~10-12Mbit/s ceiling, root-caused and fixed

Addresses the first "not yet done" item above -- multiple clean-boot
runs now confirmed, not just the original single 190s one -- and a real
regression that only showed up once someone actually pushed bitrate past
what that first run happened to test.

**Symptom:** video streamed cleanly up to ~10Mbit/s, but pushing higher
(the link was tuned to `mcs=12`, `BB_GET_MCS` reporting a 25933kbps
ceiling -- plenty of headroom on paper) produced occasional
`bitrate_ctl: URGENT ring backlog=N slots -> video0.bitrate=... kbps`
throttling once past roughly 12-14Mbit/s. Already known not to be a
hardware limit: this exact chip/antenna combination previously held a
clean ~18-19Mbit/s with the vendor's own `artosyn_sdio.ko` + this
project's daemon (see the isolation test table above), and separately
with the vendor's stock image outright.

**Root cause:** `artosyn_do_write()` and `artosyn_do_read()` in
`kmod/artosyn_drv.c` both ran their "courtesy" mailbox-register check
(`sdio_claim_host()` + `sdio_readb(REG_IRQ_STATUS)`, a real SDIO bus
round-trip) **unconditionally on every single call**, even when
`write_valid_size`/`read_valid_size` already had room left over from a
previous TX/RX-ready event -- unlike `artosyn_poll()` in this same file,
which correctly gates the identical check behind
`if (!artosyn_read_condition(dev) && !artosyn_write_condition(dev))`.
This is exactly the same class of bug this rewrite's own design notes
already flag as a proven regression source (see `artosyn_check_events()`
own comment about the old combined driver's poll()-calls-irqhandler
experiment) -- an extra register read costs real bus time under
saturated throughput -- just present here in the read/write paths
instead of `poll()`, and easy to miss precisely because `poll()` right
next to it does this correctly. The cost is fixed per *call*, not per
byte, so it doesn't show up at low bitrates (few calls/sec) and only
becomes bus-contending once call frequency (which scales with
requested-bitrate ÷ per-write chunk size) climbs high enough --
matching the observed "fine below ~10-12Mbit/s, throttles above" shape
exactly.

**Fix:** gate both courtesy checks the same way `artosyn_poll()` already
does -- only claim the host and read the register when the condition is
not already true. Correctness is unaffected (`wait_event_interruptible_
timeout()` already re-checks the condition itself before deciding
whether to actually sleep); this only removes a redundant bus
transaction in the common case where room/data is already known to be
available.

**Confirmed on real hardware, clean-boot methodology (auto-start
renamed out, manual bring-up per "Build and test" above):** sustained
**~18.5-18.6Mbit/s**, zero `incomplete` frames, zero `failed` chunks,
zero `full_drops`/`other_drops`, for the length of the soak -- matching
the vendor-driver-era ceiling this project had previously only reached
with the vendor's own kernel module.

**Residual, rare `URGENT` events are not a transport bug.** A tight
real-time capture (poll `ar8030-transport-tx -v`'s own log for `URGENT`,
snapshot the daemon's own debug-pad log the instant one appears -- same
method as the isolation-test era above) caught every occurrence: at each
one, `dev_dat_so_write_proc`'s `send ok`/`send cpl` pairs were completing
back-to-back with a **maximum 6-8ms gap** between consecutive log lines
-- nowhere near what a real SDIO/daemon-level stall looks like elsewhere
in this investigation (hundreds of ms to permanent). No stall signature
at all at the moment the ring backlog is reported. This is consistent
with a normal H.265 keyframe burst (periodically larger frames
transiently exceeding the instantaneous transmit rate even at a
well-tuned average bitrate) hitting `bitrate_ctl`'s own reactive
backoff exactly as designed, not a driver/daemon defect. Event frequency
also dropped sharply (from roughly every 10-15s to roughly every
80-200+s, isolated singles instead of clusters) after switching channel
mid-test, suggesting the remaining rate has some RF-interference
component too, separate from anything fixable in this codebase.

**Operational note for future sessions testing this by hand:** see the
`ar8030d` stdout-redirection warning in "Build and test" above -- hit
live during this same round of testing.

## `ar8030-lifecycled`: link supervisor + bind button

`lifecycled/` builds `ar8030-lifecycled`, a separate process (not a
thread inside `ar8030d` -- see `lifecycled/main.c`'s own header comment
for why) that owns the AR8030 chip's whole lifecycle from outside: an
initial hardware reset, following the daemon's `BB_EVENT_LINK_STATE`
events (plus a `BB_GET_STATUS` fallback poll) to notice a real
connect/drop, re-applying persisted bandwidth tuning, and running
`hooks.d/<event>/*` scripts (`connected`/`dropped`/`idle`/`pairing`) on
each real transition -- a board's own overlay drops executable scripts
under a configured `--hook-dir` to react to any of these (LED state,
bringing a TUN interface up/down, starting/stopping a video bridge)
without this binary needing to know anything about what a given board
actually wants to do.

Built and installed on both sides exactly like `ar8030-linkctl` is (see
"Build" above and each Buildroot package's own `.mk`) -- against the
`ar8030` package's staged `libar8030_client.so`/headers, not inside the
vendor SDK's own CMake tree. It moved here from
`builder/package/ar8030`'s own patch stack (and
`sbc-groundstations/package/ar8030`'s byte-identical duplicate of it)
because it is entirely original code with no vendor lineage -- this is
its actual home now, the same reasoning `kmod/artosyn_drv.c`'s own
"Clean-room rewrite" section above already explains for the kernel side.

**It never triggers a pairing dispatch itself.** `lifecycle.c`'s own
comments cover this in detail: Ghidra decompiles of the stock
`ar_ldy_gnd`/`ar_ldyhs_sky` streamers found their reconnect path never
calls the pairing-dispatch ioctl either -- an already-paired chip's RTOS
firmware reconnects to its configured candidate entirely on its own; the
dispatch call is exclusively the vendor GUI's bind-button handler's job.
An earlier version of this code did trigger pairing itself on every drop
of an already-paired link and that caused a real, reproducible flapping
bug (connect-then-drop-within-a-second), root-caused to exactly this
mismatch with stock behavior.

**`--bind-gpio <n>`** is the one thing this binary *does* initiate:
watching a board's physical bind button directly (`lifecycle_bind.c`),
running the SDK's own `ar8030-pair` binary (`dev_helper/bb_pair`,
`package/ar8030` -- not reimplemented here) on a debounced press, and
driving `hooks.d/pairing`/`connected`/`idle` directly from its result.
This used to be a separate shell script with no shared state with this
daemon, which caused its own bug: a rebind to an already-connected peer
produced no observable transition in `lifecycle_thread_main`'s state
machine (it only fires `connected` on the `IDLE -> CONNECTED`
transition, never merely because a link is still up), so nothing ever
told the script's LED loop to stop. Moving the button into this process
fixes that at the root -- the button's own success/failure path drives
the hooks it needs directly instead of hoping the reconnect-follower
thread notices. Omit `--bind-gpio` (the default) on a board with no
physical bind button; nothing about the reconnect-following/tuning/hook
machinery above depends on it.

## HTTP control API

`lifecycled/lifecycle_http.c` adds a small, opt-in REST + HTML control
API to `ar8030-lifecycled` (`--http-port <n>`, `0`/unset = disabled, the
default; `--http-bind <addr>`, default `0.0.0.0`) so either end of the
link can be queried/driven with plain `curl` -- the actual motivation
being air<->ground integration: a ground-side tool (or a person) can
check/change either radio's link state over the same network the video
already crosses, without shelling into either box. Runs on both air and
ground (same binary, same flag) -- air's `S65ar8030-transport-tx` and
ground's `S98ar8030-transport-rx` both pass it through unchanged via
their own `AR8030_LIFECYCLED_ARGS` (`/etc/default/ar8030-transport-{tx,rx}`),
e.g. `AR8030_LIFECYCLED_ARGS="--http-port 8899"`.

**No auth, bound to every interface by default.** Same trust model as
this project's other loopback-style control surfaces (waybeam's own
`/api/v1/live/set`, `ar8030d`'s own RPC port) -- meant for a trusted
local/link network, not for exposure beyond that. `--http-bind
127.0.0.1` restricts it to loopback if that's all a given deployment
needs.

Endpoints (see `lifecycle_http.h`'s own header comment for the full
design rationale):

- **`GET /`** -- a small self-contained HTML control panel (status,
  pair button, bandwidth selector, a raw `ar8030-linkctl` passthrough
  panel). No external assets -- this is served by the device itself,
  often with no other network reachable.
- **`GET /api/v1/status`** -- this daemon's own view of the link:
  ```
  curl http://<host>:8899/api/v1/status
  {"ok":true,"role":"ap","state":"connected","connected_slot":0,"bandwidth_mhz":20,"paired":true}
  ```
- **`POST /api/v1/pair`** -- runs the exact same fork+exec
  `ar8030-pair`+hook-dispatch sequence the physical bind button already
  runs (`lifecycle_pair.c`'s `lc_pair_run()`), for boards with no
  physical button or for triggering a rebind remotely:
  `curl -X POST http://<host>:8899/api/v1/pair`.
- **`POST /api/v1/bandwidth?mhz=<1|2|5|10|20|40>`** -- the *persisted*
  way to change bandwidth: queues the request for the lifecycle thread
  to persist (`lc_tuning_save()`, sticks across the next reconnect) and
  apply (`lc_tuning_apply()`, if currently connected) on its own next 1s
  tick, rather than issuing the `bb_ioctl` directly from the HTTP
  thread -- `ctx->client.handle` is only ever safe to call from the
  lifecycle thread itself (see `main.c`'s own header comment on why this
  daemon is a whole separate process from `ar8030d` in the first place;
  the same reasoning rules out a second thread in *this* process making
  concurrent `bb_ioctl` calls on the same handle).
  `curl -X POST 'http://<host>:8899/api/v1/bandwidth?mhz=20'`.
- **`POST /api/v1/retx-tuning?win=&busy=&idle=&conti_busy=&conti_idle=`**
  -- same mailbox/persist/apply pattern as `/api/v1/bandwidth` above, for
  the windowed retransmission controller's own tuning parameters
  (`BB_SET_RETX_EVENT_STATUS`, see `lifecycle_tuning.h`'s own doc comment
  on `lc_retx_apply()`). All 5 values are required (0-255 each) -- there
  is no partial-update support, since this process doesn't cache the
  chip's own current values anywhere it could fill gaps in from; read
  `ar8030-linkctl retx` first if you only want to change one field.
  Persists to an `ar8030.retx` sidecar next to `cfg_path`, and the
  vendor's own defaults (`win=10,busy=6,idle=4,conti_busy=2,
  conti_idle=0` -- confirmed SAFE by this project's own hardware sweep,
  not confirmed *correct*: the units of these thresholds are still
  unverified) apply automatically on every connect when nothing has
  been persisted yet, so the controller is never left in its raw,
  never-configured state.
  `curl -X POST 'http://<host>:8899/api/v1/retx-tuning?win=10&busy=6&idle=4&conti_busy=2&conti_idle=0'`.
  **Confirmed NOT usable as a rate-control signal**: live-monitoring
  `BB_GET_RETX_EVENT_STATUS`'s reply on a real link found the struct's
  131 bytes past these 5 fields are leaked/reused `ar8030d`-internal
  buffer memory, not real per-opcode telemetry (one captured sample
  decoded byte-for-byte as the daemon's own unrelated error log string)
  -- this endpoint exists for tuning the controller itself, not for
  reading retx pressure back out of it.
- **`POST /api/v1/linkctl?cmd=<subcommand>&args=<space-separated args>`**
  -- a generic, allow-listed passthrough to the standalone
  `ar8030-linkctl` binary, covering every one of its own subcommands
  (`status`, `channel-mode`, `channel`, `mcs-mode`, `mcs`, `mcs-range`,
  `mcs-table`, `power-mode`, `power`, `freq`, `force-close-socket`,
  `force-close-all`, and `bandwidth` for one-shot use) -- effectively
  `linkctl -h`'s whole command surface, reachable over HTTP the same way
  `curl` would invoke the CLI directly:
  ```
  curl -X POST 'http://<host>:8899/api/v1/linkctl?cmd=channel&args=5+-s+auto+-w+5'
  {"ok":true,"exit_code":0,"output":"BB_SET_CHAN_MODE(auto_mode=0) ret=0\n...\n"}
  ```
  Safe to add this way specifically because every `ar8030-linkctl`
  invocation opens its own independent, one-shot connection to `ar8030d`
  and exits (`linkctl/main.c`'s own header comment) -- it never touches
  this daemon's own `ctx->client` at all, so it needs no mailbox and runs
  directly on the HTTP worker thread, the same "fork-per-call, no shared
  mutable state" safety argument `lc_pair_run()` already relies on.
  **Not persisted**: a raw `linkctl bandwidth` call through this
  passthrough is a one-shot override that `lifecycle.c`'s own periodic
  tuning re-assert (its "re-apply, not detect-and-persist" logic, see
  that file's own comment) will silently overwrite with whatever was
  last persisted/queued within `LC_POLL_FALLBACK_S` seconds while
  connected -- use `/api/v1/bandwidth` above for anything meant to stick.
  `args` is tokenized on plain whitespace straight into `execvp()`'s
  `argv[]` -- no shell involved, so there is no quoting support and no
  injection surface beyond "which argv entries does `ar8030-linkctl`
  itself get", exactly as if each token had been typed as a separate CLI
  argument.

## Phase 2 (explicitly out of scope here)

- **FEC.** `ar8030_chunk_hdr.reserved` is the only field reserved for
  this; no implementation yet. A chunk lost today just drops its frame.
- **Cross-link telemetry-based bitrate.** The loop now also reads
  `BB_GET_USER_QUALITY`'s local LDPC block-error ratio (see "Bitrate
  control" above) — still no RTT/loss feedback carried *from the peer*
  over the link itself. The baseband's own windowed retx-event
  controller (`BB_GET_RETX_EVENT_STATUS`) is now wired in as a tunable
  (see "HTTP control API"'s own `/api/v1/retx-tuning` above) but
  confirmed NOT usable as a rate-
  control signal — its own reply carries no real telemetry past 5
  configured bytes (see that section's own comment on the leaked-memory
  finding), so there is nothing left to gate a bitrate loop on via this
  opcode.
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
