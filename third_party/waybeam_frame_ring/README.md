# Vendored: waybeam frame-shm ring

`venc_frame_ring.h` / `venc_frame_ring.c` are copied verbatim from
[`waybeam_venc`](../../../waybeam_venc) (the `waybeam` repo was later renamed
from `waybeam_venc`; see its README) so that `ar8030-transport-rx` does not
need waybeam checked out on the ground side, and so `ar8030-transport-tx`
builds against a pinned, known-good copy instead of a moving sibling
checkout.

- Source commit: `cd3c5a27a9a515bcdd76900de1558bd342c3d10d`
  (`include/venc_frame_ring.h`, `src/venc_frame_ring.c`)
- License: MIT (see `LICENSE` in this directory, copied from upstream)
- Ring format version pinned by these files: `VENC_FRAME_RING_VERSION` = 2

Only `ar8030-transport-tx` uses this (as a pure consumer — it only calls
`venc_frame_ring_attach()` / `venc_frame_ring_read_wait()` /
`venc_frame_ring_destroy()`, never `_create()`). `ar8030-transport-rx` does
not need it at all: the ground side never touches waybeam's shm ring, only
the reassembled Annex-B bytes carried over the AR8030 link.

**Do not hand-edit these two files.** If waybeam bumps
`VENC_FRAME_RING_VERSION` or changes the wire layout, re-copy both files
from the new waybeam commit, update the commit hash above, and re-check
`tx/main.c` against `protocols/frame-shm.md` in waybeam for any semantic
changes (not just the struct layout).
