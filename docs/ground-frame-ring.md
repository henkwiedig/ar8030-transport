# Ground frame ring: rx -> PixelPilot without RTP

Status, 2026-11-12: phases 1-4 done and bench-tested on the VR04 with a P1
air unit (see "Results"); the ring is the default on every Artosyn ground
(sbc-groundstations 7b15901), the VRX Pro (Rockchip/MPP) untested. Phase 5
(audio): built, the transport bench-tested with a synthetic Opus stream on
the P1 (which has no microphone); playback waits for an Ascent air unit and
a ground with an audio output (the VR04 has none). Decisions taken with the
user are in "Decisions" at the end.

## Why

On the ground, rx hands PixelPilot the video as RTP over the loopback, and
PixelPilot's GStreamer pipeline (`udpsrc ! tee ! rtph265depay ! h265parse !
appsink`) turns it back into access units. Measured on the BetaFPV VR04
(AR9311, 2x A53 at 800 MHz) at 21 Mbit/s, 60 fps, with perf:

| thread | CPU (% of one core) | what it does |
|---|---|---|
| PixelPilot `udpsrc0:src` | 22-33 % | per packet: recvmsg, buffer alloc, last-hop probe (formats the sender IP), sequence tracking, tee, depayloader, h265parse |
| PixelPilot GStreamer pull thread (`pixelpilot`, FIFO 30) | ~8 % | appsink -> the decoder feed |
| rx main thread | ~13 % (after the CRC16 fix) | of which a good part is RTP packetizing, `next_nal` scanning and ~750-2200 `sendto()`/s |
| ar8030d | ~16 % | SDK daemon, loopback TCP to rx (not touched here) |

The box is 52-61 % busy with the CRC16 fix and `-M 8192`; before them it was
70-89 %, and that load showed up as display drops and lost frames whenever
anything else ran (DVR start, an scp, the support collector). Every byte of
video is copied about six times on its way from rx to the decoder (reassembly
buffer, sendto, the udpsrc buffer, depayloader, parser, appsink -> decoder
buffer).

The air already solves the same problem the other way round: waybeam writes
whole frames into a frame-shm ring (`/venc_frames`, `venc_frame_ring` v2,
third_party/waybeam_frame_ring/) and tx reads them. The ground does the
mirror image: **rx writes reassembled frames into a frame-shm ring,
PixelPilot reads them and feeds its decoder directly.**

## Ring or rx inside PixelPilot?

A ring. With the ring, merging rx into PixelPilot would save one memcpy
of 2.6 MB/s (well under 1 % of a core) and one futex wake per frame; that is
all that is left between the two once the RTP hop is gone. Keeping them
apart keeps:

- crash isolation: a PixelPilot crash or restart (pixelpilot.sh restarts
  it) does not drop rx's link state, clock sync, sidecar or IDR relay, and
  an rx problem does not take the display down;
- scheduling: rx and ar8030d can get their own real-time priority (the
  DVR frame-loss fix wants exactly that), independent of PixelPilot's;
- rx's other uses: the RTP output stays available for a laptop and for
  tests, and rx keeps running (its log, stats, clock sync) without
  PixelPilot;
- the existing format: frame-shm is specified and has three consumers
  already (tx, radeon-vrx, waybeam-link); PixelPilot becomes a fourth, and a
  test reader is a few lines.

## Producer: rx

New option `-R <name>` (e.g. `-R gnd_frames`): rx creates
`/dev/shm/gnd_frames` (`venc_frame_ring_create`, 8 slots x 512 KB = 4 MB,
the same geometry as the CV610 air ring and rx's own reassembly cap) and
publishes every frame it would otherwise send as RTP. With `-R` the RTP video
output is off (decided: no RTP while the ring is in use); only a test option
(`--also-rtp`) keeps both, for the phase-1 comparison.

What goes into an entry is exactly what rx sends as RTP today:

- a frame sent whole: one entry, the access unit with the timing SEI
  inserted ahead of the first slice (`timing_sei_insert`);
- a streamed frame (slices forwarded as they arrive, `SLICE_END`): one entry
  per forwarded part with `VENC_FRAME_FLAG_PARTIAL`, and the last part (the
  suffix timing SEI plus the rest of the picture) without it. That is the
  ring's existing slice convention (waybeam `sliceOutput`).

The timing SEI stays in-band, so PixelPilot's timing code (capture/rx_done
times, vsync lock, latency facts) does not change, and a raw DVR recording
keeps it.

`VencFrameMeta`:

| field | value |
|---|---|
| `pts` | capture time on the ground clock, truncated to 32 bits, when the clock sync is valid; otherwise rx_done (same as the SEI's fields, for consumers that do not parse SEI) |
| `codec` | `VENC_FRAME_CODEC_H265` (the only codec the air sends) |
| `flags` | IDR/GDR/ENHANCE from the chunk header flags, PARTIAL for slices, plus the loss flag below |
| `gdr_pos`, `gdr_len` | 0: the link does not carry them |

Loss: PixelPilot today finds holes from RTP sequence gaps and requests an
IDR. With the ring, rx knows better than any consumer: it sees frame_seq
gaps, streamed frames whose rest never arrived, and its own full-ring drops.
So rx sets a receiver-side flag `VENC_FRAME_FLAG_DISCONT = 0x20` ("one or
more frames or parts are missing before this entry") on the first entry
after any of the three. It is reserved in waybeam's canonical header (like
`SALVAGED` 0x08, also receiver-set) and the vendored copy is re-synced. Not
the SEI's `frame_seq`/`frames_lost` instead: rx inserts the SEI only once
the clock sync is valid, so the first seconds after link-up would go
unchecked.

Copies: phase 1 keeps rx's reassembly buffer (the SEI insertion and the
slice offsets work on it) and copies each entry into its slot: one memcpy of
the payload, ~2.6 MB/s. Reassembling straight into the slot is possible
later but not worth it on its own.

Full ring (PixelPilot gone or stalled): the producer drops the new entry, as
the format defines, counts it in `full_drops`, and marks the next entry it
does publish DISCONT.

## Consumer: PixelPilot

A live source next to `udpsrc`, selected in artosyn mode, e.g.
`--frame-shm gnd_frames` (from `/etc/default/pixelpilot`):

- a reader thread (FIFO, like today's pull thread) blocks in
  `venc_frame_ring_read_wait()` (futex, no polling) and reads each entry
  straight into the frame buffer it hands to the existing
  `NEW_FRAME_CALLBACK`: whole entries go to `feed_decoder()`, PARTIAL
  entries to the slice feed (`au_end` = not PARTIAL). V4L2 (wave5) and MPP
  (VRX Pro) both sit behind that callback already;
- attach: flush what is queued (`read_idx = write_idx`, as tx's
  `flush_ring()` does) and request an IDR, so a restarted PixelPilot never
  decodes a stale backlog;
- rx restart: rx creates a fresh ring; on a read timeout PixelPilot checks
  the name's dev/inode (tx's `stat_named_shm()`) and reattaches;
- stream up/down from entry arrival times; IDR requests go to rx's relay
  (`127.0.0.1:11223`) as now, with no last-hop discovery; DISCONT -> IDR
  request, replacing the RTP gap detector;
- raw DVR: while recording, each access unit is pushed into an
  `appsrc ! h265parse ! splitmuxsink` bin (the existing record bin with an
  appsrc in front instead of the RTP tee). Outside recordings there is no
  GStreamer in the live path at all;
- unchanged: re-encode DVR, OSD, vsync lock, the menu;
- not in ring mode: the network restream (`restream_sink`); audio comes
  through its own ring (see "Audio").

PixelPilot vendors `venc_frame_ring.h/.c` (MIT) the way ar8030-transport
does.

## Audio

Today: `audio_tx`/`audio_rx` carry waybeam's RTP/Opus packets (PT 98) on
their own `bb_socket`, port 3, and `audio_rx` sends them to the same UDP
port as the video, where PixelPilot splits them by payload type, plays them
(`rtpopusdepay ! opusdec ! alsasink`) and muxes them into the raw DVR. None
of it is in an image yet: no target packages the audio binaries, waybeam has
`audio.enabled: false`, PixelPilot's `PIXELPILOT_AUDIO` is empty.

And it cannot work next to `ar_net0` as it is: only ports 2 and 3 are
usable (0 and 1 are claimed inside the chip firmware), video has 2 and
`ar_net0` has 3. So audio has to ride the video socket, the way the control
(SYNCR) and sidecar messages already do: tx takes over `audio_tx`'s job
(reading waybeam's RTP/Opus on `127.0.0.1:5601`) and writes each packet as
an `AR8030_CHUNK_CODEC_OPUS` single-chunk frame between two video frames
(the stream-mode socket must not interleave chunks of different frames). At
60 fps a packet waits about one frame interval at most; 32 kbit/s Opus is
~50 kbit/s with the RTP and chunk headers, well under 1 % of the link. rx, which skips non-H.265 chunks today, then sees both streams
and is the one producer of both.

rx writes audio into a ring of its own, `/gnd_audio` (e.g. 64 slots x
1 KB), not into the video ring: the video ring keeps its geometry, a reader
that stalls on one stream cannot push the other out, and frame-shm consumers
that expect video only ever see video. An entry is the RTP/Opus packet as
the air sent it (its sequence number and timestamp still drive the
depayloader), `codec` = a new `VENC_FRAME_CODEC_OPUS = 0x02` (reserved in
waybeam's header together with DISCONT), `pts` = rx's receive time on the
ground clock. PixelPilot reads it in a second thread into an
`appsrc caps=application/x-rtp,...OPUS` in front of its existing audio
branch, and the raw DVR bin takes audio from there as it does now.

Both rings carry ground-clock times, so audio could later be held back to
line up with the vsync-locked video; today audio plays as soon as it
arrives (`sync=false`), and that stays.

### What was built (phase 5)

- tx `-a <port>` (tx/audio_sub.c): binds 127.0.0.1:<port> (waybeam's
  `outgoing.audioPort`, 5601), queues each RTP/Opus packet on tx/outq.h
  like the sidecar (queue 16 -> 32 entries), written with the next video
  chunk as an `AR8030_CHUNK_CODEC_OPUS` single-chunk frame. The air units'
  `hooks.d/*/40-socat.sh` relayed that port over ar_net0 to the ground's
  UDP 5600 (09-23, "socat for audio forward"); with PixelPilot reading the
  ring that went nowhere, so the hooks go and tx takes the port.
- rx `-A <name>`: each OPUS chunk into the audio ring (64 x 1 KB,
  `VENC_FRAME_CODEC_OPUS`, pts = rx's receive time), and as RTP to the UDP
  target while rx sends RTP; `-v` prints an "rx audio:" line.
- PixelPilot `--frame-shm-audio <name>` (with `--audio`): a reader pushes
  each packet into `appsrc ! tee name=rtp_tee`, so the RTP path's playback
  branch and the recorders' audio tap work unchanged.
- Bench (P1 + VR04): a 50 packets/s Opus test stream (host GStreamer ->
  socat on the P1 -> tx -a) arrived in the ground's audio ring at 49.8/s,
  0 dropped, read by PixelPilot; video unaffected.

## Expected effect (VR04)

Gone from the live path: rx's RTP packetizing and sends, `udpsrc0:src`, the
GStreamer pull thread, depayloader and parser copies. Estimate 35-45 % of a
core (about a fifth of the box) and 1-3 ms less latency (one loopback hop and
one GStreamer queue). The VRX Pro (4 cores) gains the same work, less
visibly.

## Phases

1. rx producer (`-R`, RTP off; `--also-rtp` for the check below), plus a test reader (`frame-shm-dump`:
   entries/s, sizes, flags, gaps, SEI latency). Bench check: with both
   outputs on, every ring entry equals the depayloaded RTP access unit.
2. PixelPilot ring source, live path only (decode + vsync lock + OSD), behind
   the new option; DISCONT and attach/reattach handling.
3. PixelPilot raw DVR through appsrc while recording.
4. sbc-gs: `S98ar8030-transport-rx` and `/etc/default/pixelpilot` switch
   artosyn mode to the ring (RTP off), with a fallback to UDP when the ring is
   missing; A/B on the VR04 (CPU, latency, drops), then the VRX Pro, P1 VRX
   and SinYine VRX.
5. Audio: tx muxes waybeam's Opus onto the video socket, rx writes
   `/gnd_audio`, PixelPilot plays it through an appsrc; `audio_tx`/`audio_rx`
   retire (their port 3 belongs to `ar_net0`), and so does the air units'
   socat relay of the audio port over ar_net0 (the user's choice, 11-12).

## Results (VR04 + P1 air, 2026-11-12)

### Phase 1: rx producer

`rx -R gnd_frames` (`rx/ring_out.c`, host test `test/ring_out_test.c`) and
`ringdump/` (`ar8030-ringdump`), on the bench VR04 with the P1 sending
SC231AI 1080p60 in two strips, ~21.8 Mbit/s:

- `rx -R gnd_frames -P -p 5610` + `ar8030-ringdump -u 5610`: 1201 access
  units compared, every one identical to the depayloaded RTP (NAL units and
  their bytes), 0 RTP sequence gaps.
- 120 entries/s: per frame one PARTIAL entry (the first strip) and one
  closing entry (suffix timing SEI + the second strip). Largest entry 30 KB,
  largest access unit 53 KB (the P1 sends GDR, no IDRs; the Ascent's IDRs
  are still to be measured before the slots shrink).
- rx finishing a frame -> the reader having it (`rx_done_ground_us` against
  the reader's clock): 0.10-0.15 ms average, 0.6-3.8 ms worst per second.
- rx with the ring only (no RTP): main thread 7.4 % + 2.9 % of a core,
  against 15.5 % + 3.5 % with RTP at `-M 8192`.
- Without a reader the ring fills in 8 entries and rx counts the rest as
  dropped (~354 in the 3 s before the reader attached); the first entry the
  reader then gets carries DISCONT.

### Phases 2-4: PixelPilot, DVR, defaults

PixelPilot `--frame-shm` (PixelPilot_rk 24f265f, dbd37c9), ring vs RTP
interleaved 2 x 90 s each, both with the table CRC16 and RTP at `-M 8192`:

| | PixelPilot | rx | box busy | p50 | p99 | vsync target | display drops |
|---|---|---|---|---|---|---|---|
| RTP | 45 % of a core | 16 % | 51 % | 33.2 ms | 45.6 ms | 47.1 ms | 4.8 / 10 s |
| ring | 21 % | 11 % | 35 % | 30.0 ms | 36.3 ms | 43.2 ms | 2.6 / 10 s |

- the wave5 decoder must not get GDR pictures before the parameter sets
  (h265parse holds them back on the RTP path): PixelPilot feeds from the
  first access unit with an SPS
- raw DVR from the ring through an appsrc; recorders start and stop in a
  thread of their own (starting one took 0.8-1 s, which overflowed the
  8-entry ring when it ran on the ring thread)
- an rx restart: PixelPilot sees the ring gone or replaced and reattaches
- the re-encoding DVR (the VR04's default) stalls the decoder feed now and
  then (the wave5 encoder next to the decoder): with one thread reading the
  ring and feeding the decoder that lost ~1100 entries in 84 s. PixelPilot
  88c4787 reads the ring in one thread (SCHED_FIFO) into a 2 MB queue and
  feeds the decoder from it in another, as udpsrc's socket buffer and the
  pull thread did: 0 lost. 50 s each, display drops per 10 s: no DVR 3.0,
  raw 4.5, re-encode 7.3
- memory: the VR04 runs close to its limit. With 9 MB of test files in /tmp
  (MemAvailable 16 MB, CMA nearly empty) recording froze the whole process
  for 1-6 s at a time; with ~25 MB available it was clean. The ring itself
  is resident (4 MB in /dev/shm); 8 x 256 KB would halve that if the
  Ascent's IDRs fit
- defaults (sbc-groundstations): S98ar8030-transport-rx passes
  `-R gnd_frames`, pixelpilot.sh adds `--frame-shm gnd_frames` in receiver
  mode artosyn (not /etc/default/pixelpilot, whose gsmenu-edited overlay
  copies would keep older arguments)

## Decisions (2026-11-12, with the user)

- Loss: `VENC_FRAME_FLAG_DISCONT` 0x20 in the shared frame-shm format
  (waybeam header + vendored copy), together with `VENC_FRAME_CODEC_OPUS`
  0x02 for the audio ring.
- Slots: 8 x 512 KB, rx's reassembly cap (a frame bigger than a slot could
  not be published at all). Phase 1's reader measures the largest frames;
  shrink only if they leave room.
- RTP: off whenever the ring is in use.
- Audio: its own ring, phase 5 (above); needs an Ascent air unit.
- No fallback to RTP for PixelPilot builds without `--frame-shm`: every
  Artosyn ground image moves to the ring at once.
