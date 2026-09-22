# AR8030 native net_device over SDIO

This note records how the vendor's `yz_host_drv/driver/linux` (the "OAL/DRV"
kernel driver) gets from an SDIO probe to a Linux `net_device`, and how
`kmod/` reuses that design on top of this project's own clean-room SDIO
transport instead of importing the vendor tree.

## 1. Vendor call chain (as found in the source)

All paths are relative to `yz_host_drv/driver/linux/`.

```
module_init(oal_init)                                   oal_main.c
 └─ sdio_artosyn_init() → sdio_register_driver()        bus/sdio.c

sdio_artosyn_probe(func)                                 bus/sdio.c
 ├─ kzalloc(struct artosyn_sdio_device)  (tx_q/rx_q/mailbox_q)
 ├─ sdio_claim_host; sdio_enable_func
 ├─ sdio_claim_irq(func, artosyn_sdio_irqhandler)       ← the BUS layer owns the IRQ
 ├─ sdio_writeb(1, 0x14); sdio_writeb(0xf0, 0x68)
 ├─ mode = ROM id ? OAL_CARD_MODE_FW_DNLD : OAL_CARD_MODE_RPC
 ├─ oal_add_card(dev, NULL, mode, &oal_cb)               oal_main.c
 │   ├─ kzalloc(oal_handle); m_handle[idx] = handle
 │   ├─ cb.oal_update_card_type = ar_sdio_update_card_type   (dev->handle = handle)
 │   ├─ oal_init_sw()
 │   │   ├─ oal_init_skbreq_queues()   msgout 100 / pktin 100 / pktout 200 reqs
 │   │   └─ oal_init_workqueue_all()   OAL_MAIN / OAL_TX / OAL_RX workqueues
 │   ├─ cb.oal_register_dev            NULL for SDIO (USB only)
 │   ├─ FW_DNLD: oal_init_fw() → cb.oal_send_dnld_req/done → sdio_rom_send()
 │   └─ RPC:     state = AR_BUS_STATE_UP; oal_init_platform()
 │       ├─ ar_rpc_init()              rpc/ar_rpc.c  (128 KiB datagram ringbuffer,
 │       │                                            session_socket_list)
 │       ├─ ar_proc_init()             /proc/<dir>/dev<idx>
 │       ├─ class_create("ar_drv<idx>")
 │       ├─ oal_mdev_init()            /dev/ar_mdev<idx>  (DRV-mode daemon chardev)
 │       └─ ar_net_dev_init()          net/net_dev.c
 │            └─ /dev/ar_net<idx> CONTROL chardev only, net_dev_inited = true
 │               *** no net_device is allocated here ***
 ├─ oal_init_workqueue(SDIO_REG_WORK_QUEUE, ar_sdio_reg_check)
 ├─ oal_init_workqueue(SDIO_RX_WORK_QUEUE,  ar_sdio_rx_func)
 └─ misc_register("artosyn_sdio")      (RPC mode only)

net_device creation is a separate, userspace-triggered step
(app/ar8030/net_dev.c bb_net_dev_create(), app/net_dev_demo):
 open("/dev/ar_net0"); ioctl(AR_CHARDEV_IOCTL_NET_HANDLE_NETIF, {op=AR_NET_OP_CREATE,
                                   slot, socket_port, mac, name, tx/rx_buf_size})
 → ar_chardev ioctl → net_mdev_ioctl() → linux_handle_netif()
 → ar_net_netif_create()                                   net/net_dev.c
     ├─ ar_net_init_dev(): alloc_etherdev, ether_setup, register_netdev()   ← HERE
     ├─ create_socket_session(slot, port, TX|RX|DATAGRAM)  rpc/session_socket.c
     ├─ tasklet_init(ar_net_task); dev_open()
     └─ ar_bb_rpc_socket_open() → ar_rpc_tx_msg() → oal_send_msg_req()

RX:  artosyn_sdio_irqhandler (0x13 iir, 0x68 mbox, 0x5c blocks<<9)
     → rx-ready → oal_queue_work(dev->rx_workqueue)
     → ar_sdio_rx_func → dev_alloc_skb(4097) → cb.oal_read_data_sync
       = ar_sdio_read_data_sync → artosyn_read → sdio_memcpy_fromio
     → PKTIN_POST → OAL_MAIN wq → OAL_RX wq → oal_process_rx_req → oal_recv_complete
     → cb.oal_recv_packet_complete = ar_sdio_recv_packet_complete
     → ar_rpc_rx_pkt → ar_rpc_rx_pkt_schedule_task:
         unpack_usb_pack(); if reqid[31:24]==4 (socket) && ar_net_exists(slot,port):
            so_read  → strip RPC hdr/tail → ringbuffer → unpack_socket_datagram_pack_rb
                       → ar_net_rx_enqueue → tasklet ar_net_task → eth_type_trans → netif_rx
            so_write → ack, freed (ignored)
         else → ar_drv_mdev.rx_q → /dev/ar_mdev read() (userspace daemon)

TX:  ndo_start_xmit = ar_net_device_tx → ar_rpc_socket_write (datagram 0xab..0xbc)
     → ar_rpc_tx_pkt (u64 wr_index + RPC hdr 0xaa..0xbb) → oal_send_packet_req
     → PKTOUT_POST → OAL_MAIN wq → OAL_TX wq → oal_process_tx_task
     → cb.oal_write_data_sync = ar_sdio_write_data_sync (pad to blksize)
     → artosyn_sdio_write → sdio_memcpy_toio
     → oal_send_packet_complete → ar_net_device_tx_cb → netif_wake_queue
```

`OAL_CARD_MODE_RPC` is what enables all of the above; `OAL_CARD_MODE_FW_DNLD`
only pushes firmware. Native networking therefore requires the RPC-mode
state, but of that state only `ar_rpc` (framing + demux), the socket session
and `net_dev` are actually needed for the data path.

### Observations that matter for a port

* **IRQ ownership**: OAL never calls `sdio_claim_irq()`. The bus layer's
  handler reads the mailbox and *queues work*; OAL only sees skbs. So the
  existing clean-room IRQ handler can stay the single owner.
* **RX is a demultiplexer**: every SDIO transfer is parsed as RPC; frames for
  a socket that has a netif go to the netdev, everything else goes to the
  userspace daemon. Only one reader may drain the FIFO.
* **net_device creation is ioctl-driven** (name, MAC, slot/port and buffer
  sizes come from userspace).
* **Framing**: RPC frame `0xaa | len(LE32) | reqid(BE32) | msgid(BE32) |
  sta(BE32) | xor | data | 0xbb` (19 bytes overhead, xor seeded 0xff).
  `reqid = 4<<24 | op<<16 | slot<<8 | port` for sockets (op: 0 open, 1 write,
  2 read, 3 close). so_write data starts with the socket's u64 stream
  position. Datagram framing inside the socket byte stream is
  `0xab | len(LE32) | xor | payload | 0xbc`, identical to the userspace
  library's `session_socket_datagram_ext.c`, so a kernel netdev on one end
  and `ar8030-tun` on the other interoperate.
* **so_open payload** is `flags, tx_buf_size, rx_buf_size` (u32 each). The
  daemon's `sock_node.c` names the fields `rx_buff_len/tx_buff_len` but just
  forwards the client's bytes, so on the wire the order is tx, rx.
* **Vendor bugs not to copy**: `oal_init_lock()` never stores the lock it
  allocates (so `op_rd/op_wr_spinlock` are NULL no-ops); `buf_wr_index` is a
  single global for all sockets; so_write acks (`-0x107` "send pending",
  `-0x108` "wanted pos") are ignored, so the first lost write desyncs the
  stream position for good; the RX skb is only 4 KiB so larger transfers
  are split and the RX path relies on another interrupt to finish reading.

## 2. What kmod/ takes from it

| Vendor piece | kmod/ |
|---|---|
| bus/sdio.c probe, IRQ, mailbox, fw download | **kept as is** (`artosyn_sdio.c`, the existing clean-room code) |
| oal_main.c skbreq queues + 3 workqueues | not needed: one ordered TX workqueue; RX runs from the IRQ drain |
| oal_mdev.c `/dev/ar_mdev` | not needed: `/dev/artosyn_sdio` stays the daemon interface |
| ar_rpc.c framing + RX demux | reimplemented (`artosyn_net.c`) |
| session_socket.c open/close/write | reimplemented, plus so_write ack resync |
| net_dev.c netdev, datagram RX, tasklet | reimplemented; netdev registered at RTOS probe |
| `/dev/ar_net` ioctl, ar_chardev, ar_proc, USB, utils | not needed |

## 3. Architecture in kmod/

```
AR8030 ── SDIO ── artosyn_sdio.c ─┬─ sdio_claim_irq(artosyn_irqhandler)   single IRQ owner
                                  │    └─ artosyn_check_events()
                                  │         └─ rx-ready (native_net=1): read the whole
                                  │            transfer → artosyn_net_rx() demux
                                  │               ├─ socket frames for net_slot/net_port
                                  │               │   → datagram reassembly → netif_rx
                                  │               └─ everything else → cdev rx queue
                                  │                   → /dev/artosyn_sdio read()/poll()
                                  │                   → ar8030d (unchanged)
                                  └─ tx_mutex serialises every FIFO write:
                                       ar8030d write()  and  artosyn_net TX worker
                                       (each write is whole RPC frames, so frames from
                                        the two sources never interleave mid-frame)
```

* The ROM-id probe is untouched: firmware download, then re-enumeration.
  Only the RTOS-id probe registers the netdev (`native_net=1`, the default).
* `ip link set ar_net0 up` sends `so_open(net_slot, net_port,
  TX|RX|DATAGRAM, net_tx_buf, net_rx_buf)`; `down` sends `so_close`. This is
  the same lifecycle `ar8030-tun` had (lifecycled `connected`/`dropped` hooks
  run `ifup`/`ifdown`).
* `native_net=0` restores the previous behaviour: the daemon reads the FIFO
  itself and no netdev is created.

## 4. Found on hardware

* **One RPC frame per RX transfer, plus stale bytes.** Each RX-ready
  transfer (a multiple of 512 bytes) starts with one RPC frame; the rest of
  the block is *not* zeroed, it still holds leftovers of an older, longer
  transfer. The demux therefore stops at the first byte that is neither a
  frame start nor zero padding, and hands that remainder to userspace
  untouched (or drops it if it contains no `0xaa` at all). This is the same
  "first frame only" behaviour as the vendor's `unpack_usb_pack()` +
  `skb_trim(usedlen)`.
* **One RPC frame per TX write.** Neither the daemon nor the vendor driver
  ever packs two frames into one SDIO write, so the netdev doesn't either.
  Frames longer than one block are zero-padded to whole blocks.
* **so_write replies** for the netdev socket are mostly `sta >= 0`
  followed by `-0x106` ("send ok"); under bulk load the chip also sends
  `-0x108` (wanted pos) and occasionally `-0x107` (pending). Following
  them keeps TCP going; ignoring them (vendor) desyncs the socket.
* **Module reload** leaves the daemon without a TX-ready window, and it
  does so with the pre-change module too, so it isn't caused by this work.
  Reboot to reload.
