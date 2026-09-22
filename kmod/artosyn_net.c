// SPDX-License-Identifier: GPL-2.0
/*
 * artosyn_net.c -- native AR8030 net_device on top of artosyn_sdio.c.
 *
 * Rebuilds the data path of the vendor's OAL/DRV driver (yz_host_drv
 * driver/linux: rpc/ar_rpc.c, rpc/session_socket.c, net/net_dev.c) without
 * its OAL queue/workqueue machinery or its /dev/ar_mdev + /dev/ar_net
 * chardevs -- see ../doc/native-netdev.md for the vendor call chain and
 * what was kept. In short:
 *
 *   ndo_start_xmit -> txq -> TX worker: RPC so_write frame carrying one
 *     datagram (0xab..0xbc) per packet -> artosyn_write_all() -> FIFO
 *   IRQ drain -> artosyn_net_rx(): RPC frames for (net_slot, net_port) are
 *     consumed here (so_read -> datagram reassembly -> netif_rx, so_write
 *     acks -> stream-position resync, so_open replies), all other bytes go
 *     back to /dev/artosyn_sdio for ar8030d.
 *
 * The net_device is registered at RTOS probe; `ip link set <dev> up`
 * opens the chip-side bb socket and `down` closes it.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/string.h>
#include <asm/unaligned.h>

#include "artosyn_drv.h"

static char *net_name = "ar_net%d";
module_param(net_name, charp, 0444);
MODULE_PARM_DESC(net_name, "Interface name (default ar_net%d)");

static char *net_mac;
module_param(net_mac, charp, 0444);
MODULE_PARM_DESC(net_mac, "Interface MAC aa:bb:cc:dd:ee:ff (default: random)");

static uint net_slot;
module_param(net_slot, uint, 0444);
MODULE_PARM_DESC(net_slot, "bb socket slot (default 0)");

/* Port 3: what ar8030-tun used on both ends (port 2 is the video socket,
 * ports 0/1 are held by the chip firmware itself). */
static uint net_port = 3;
module_param(net_port, uint, 0444);
MODULE_PARM_DESC(net_port, "bb socket port (default 3)");

/* Same defaults as ar8030-tun (tuntap_bb). Never make tx tiny: a tx
 * buffer of a few bytes wedges the chip's TX path until a power cycle. */
static uint net_tx_buf = 60000;
module_param(net_tx_buf, uint, 0444);
MODULE_PARM_DESC(net_tx_buf, "bb socket tx buffer size in bytes (default 60000)");

static uint net_rx_buf = 40000;
module_param(net_rx_buf, uint, 0444);
MODULE_PARM_DESC(net_rx_buf, "bb socket rx buffer size in bytes (default 40000)");

/* --- wire format (see doc/native-netdev.md) ----------------------------- */

#define RPC_HEAD            0xaa
#define RPC_TAIL            0xbb
#define RPC_HDR_LEN         18      /* head, len LE32, reqid BE32, msgid BE32, sta BE32, xor */
#define RPC_OVERHEAD        (RPC_HDR_LEN + 1)
#define RPC_XOR_LEN         17

#define DGRAM_HEAD          0xab
#define DGRAM_TAIL          0xbc
#define DGRAM_HDR_LEN       6       /* head, len LE32, xor */
#define DGRAM_OVERHEAD      (DGRAM_HDR_LEN + 1)
#define DGRAM_XOR_LEN       5

#define SO_POS_LEN          8       /* so_write data starts with the u64 stream position */

#define BB_REQ_SOCKET       4
enum { SO_OPEN = 0, SO_WRITE = 1, SO_READ = 2, SO_CLOSE = 3 };

#define BB_SOCK_FLAG_RX         (1 << 0)
#define BB_SOCK_FLAG_TX         (1 << 1)
#define BB_SOCK_FLAG_DATAGRAM   (1 << 3)

/* so_write reply status codes (daemon/sock_node.c dev_dat_so_write_proc) */
#define SO_STA_SEND_OK      (-0x106)
#define SO_STA_PENDING      (-0x107) /* chip buffer full: continue from pos */
#define SO_STA_WANTED       (-0x108) /* chip idle, wants data from wanted_pos */
#define SO_STA_ALREADY_OPEN 0x101

#define NET_TXQ_STOP        64
#define NET_TXQ_WAKE        16
#define NET_TX_STAGE_SIZE   (16 * 1024)
#define NET_RXBUF_SIZE      (64 * 1024)
#define NET_MAX_DGRAM       (8 * 1024)
#define NET_MAX_MTU         (NET_MAX_DGRAM - ETH_HLEN - VLAN_HLEN)
#define NET_OPEN_TIMEOUT_MS 1000

struct artosyn_net {
	struct artosyn_dev *adev;
	struct net_device *ndev;
	u8 slot;
	u8 port;

	spinlock_t lock;            /* wr_index, msgid */
	u64 wr_index;               /* socket stream position of the next so_write */
	u32 msgid;

	bool sock_open;             /* chip socket open (or opening): claim its RX frames */
	int open_sta;
	struct completion open_done;

	struct sk_buff_head txq;
	struct work_struct tx_work;
	struct workqueue_struct *wq;
	u8 *tx_stage;

	/* Datagram reassembly across so_read frames. Only touched from the
	 * RX drain, which runs with the SDIO host claimed, so it's never
	 * concurrent with itself; ndo_open resets it the same way. */
	u8 *rxbuf;
	unsigned int rxlen;

	unsigned long rpc_rx_frames;
	unsigned long rpc_tx_frames;
	unsigned long tx_writes;
	unsigned long rx_unparsed;
	unsigned long rx_truncated;
	unsigned long rx_dgram_errors;
	unsigned long ack_ok;
	unsigned long ack_send_ok;
	unsigned long ack_pending;
	unsigned long ack_wanted;
	unsigned long ack_other;
};

static u8 art_xor(const u8 *p, unsigned int n)
{
	u8 x = 0xff;

	while (n--)
		x ^= *p++;
	return x;
}

static u32 so_reqid(struct artosyn_net *net, u8 op)
{
	return BB_REQ_SOCKET << 24 | op << 16 | net->slot << 8 | net->port;
}

static void rpc_put_hdr(u8 *p, u32 datalen, u32 reqid, u32 msgid)
{
	p[0] = RPC_HEAD;
	put_unaligned_le32(datalen, p + 1);
	put_unaligned_be32(reqid, p + 5);
	put_unaligned_be32(msgid, p + 9);
	put_unaligned_be32(0, p + 13);
	p[17] = art_xor(p, RPC_XOR_LEN);
	p[RPC_HDR_LEN + datalen] = RPC_TAIL;
}

static u32 net_next_msgid(struct artosyn_net *net)
{
	unsigned long flags;
	u32 id;

	spin_lock_irqsave(&net->lock, flags);
	id = net->msgid++;
	spin_unlock_irqrestore(&net->lock, flags);
	return id;
}

/* SDIO writes longer than one block must be whole blocks (the core rounds
 * anything else down); zero padding is skipped by the chip's frame
 * parser. Same rule as the daemon's sdio_write() and the vendor's
 * ar_sdio_write_data_sync(). */
static size_t net_pad(u8 *buf, size_t len, size_t cap)
{
	size_t padded = len;

	if (len > SDIO_DEVICE_BLOCK_SIZE)
		padded = ALIGN(len, SDIO_DEVICE_BLOCK_SIZE);
	if (padded > cap)
		return len;
	memset(buf + len, 0, padded - len);
	return padded;
}

/* so_open/so_close: small control frames, written synchronously. */
static int net_send_ctl(struct artosyn_net *net, u8 op, const void *data, u32 dlen)
{
	u8 *buf;
	int ret;

	buf = kzalloc(RPC_OVERHEAD + dlen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	rpc_put_hdr(buf, dlen, so_reqid(net, op), net_next_msgid(net));
	if (dlen)
		memcpy(buf + RPC_HDR_LEN, data, dlen);

	ret = artosyn_write_all(net->adev, buf, RPC_OVERHEAD + dlen);
	kfree(buf);
	return ret;
}

/* --- TX ------------------------------------------------------------------- */

static unsigned int net_frame_len(unsigned int payload)
{
	return RPC_OVERHEAD + SO_POS_LEN + DGRAM_OVERHEAD + payload;
}

/* One so_write RPC frame carrying one datagram, built at p. */
static void net_build_frame(struct artosyn_net *net, u8 *p, const struct sk_buff *skb)
{
	u32 dlen = SO_POS_LEN + DGRAM_OVERHEAD + skb->len;
	u8 *d = p + RPC_HDR_LEN + SO_POS_LEN;
	unsigned long flags;
	u64 pos;

	spin_lock_irqsave(&net->lock, flags);
	pos = net->wr_index;
	net->wr_index += DGRAM_OVERHEAD + skb->len;
	spin_unlock_irqrestore(&net->lock, flags);

	rpc_put_hdr(p, dlen, so_reqid(net, SO_WRITE), net_next_msgid(net));
	put_unaligned_le64(pos, p + RPC_HDR_LEN);

	d[0] = DGRAM_HEAD;
	put_unaligned_le32(skb->len, d + 1);
	d[5] = art_xor(d, DGRAM_XOR_LEN);
	skb_copy_bits(skb, 0, d + DGRAM_HDR_LEN, skb->len);
	d[DGRAM_HDR_LEN + skb->len] = DGRAM_TAIL;

	art_dbg(net->adev, ART_DBG_TX, "tx so_write slot %u port %u pos %llu len %u\n",
		net->slot, net->port, pos, skb->len);
}

static void net_tx_work(struct work_struct *work)
{
	struct artosyn_net *net = container_of(work, struct artosyn_net, tx_work);
	struct net_device *ndev = net->ndev;
	struct sk_buff *skb;

	/* One RPC frame per SDIO write, like both the daemon and the vendor
	 * driver: nothing on the host side ever hands the chip two frames in
	 * one transfer, so that stays untested territory. */
	while ((skb = skb_dequeue(&net->txq)) != NULL) {
		unsigned int flen = net_frame_len(skb->len);
		unsigned int bytes = skb->len;
		size_t wlen;
		int ret;

		if (netif_queue_stopped(ndev) && skb_queue_len(&net->txq) < NET_TXQ_WAKE)
			netif_wake_queue(ndev);

		if (flen > NET_TX_STAGE_SIZE) {
			ndev->stats.tx_dropped++;
			dev_kfree_skb_any(skb);
			continue;
		}
		net_build_frame(net, net->tx_stage, skb);
		dev_consume_skb_any(skb);

		wlen = net_pad(net->tx_stage, flen, NET_TX_STAGE_SIZE);
		ret = artosyn_write_all(net->adev, net->tx_stage, wlen);
		if (ret) {
			ndev->stats.tx_errors++;
			net_err_ratelimited("%s: SDIO write of %zu bytes failed: %d\n", ndev->name, wlen, ret);
			continue;
		}
		net->tx_writes++;
		net->rpc_tx_frames++;
		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += bytes;
	}

	if (netif_queue_stopped(ndev))
		netif_wake_queue(ndev);
}

static netdev_tx_t artosyn_net_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct artosyn_net *net = netdev_priv(ndev);

	if (unlikely(!READ_ONCE(net->sock_open))) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	skb_queue_tail(&net->txq, skb);
	if (skb_queue_len(&net->txq) >= NET_TXQ_STOP)
		netif_stop_queue(ndev);
	queue_work(net->wq, &net->tx_work);
	return NETDEV_TX_OK;
}

/* --- RX ------------------------------------------------------------------- */

static void net_deliver(struct artosyn_net *net, const u8 *data, unsigned int len)
{
	struct net_device *ndev = net->ndev;
	struct sk_buff *skb;

	if (len < ETH_HLEN) {
		ndev->stats.rx_length_errors++;
		ndev->stats.rx_errors++;
		return;
	}

	skb = netdev_alloc_skb_ip_align(ndev, len);
	if (!skb) {
		ndev->stats.rx_dropped++;
		return;
	}
	skb_put_data(skb, data, len);
	skb->protocol = eth_type_trans(skb, ndev);
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += len;

	/* Process context (IRQ thread / poll()): run the softirq here rather
	 * than leaving it pending. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
	netif_rx_ni(skb);
#else
	netif_rx(skb);
#endif
}

/* so_read payload is the peer's socket byte stream; datagrams can span
 * so_read frames, so reassemble before delivering. Resyncs on garbage
 * the same way the vendor's unpack_socket_datagram_pack_rb() does
 * (scan for 0xab with a valid xor and tail). */
static void net_rx_stream(struct artosyn_net *net, const u8 *data, unsigned int dlen)
{
	unsigned int pos = 0;

	if (net->rxlen + dlen > NET_RXBUF_SIZE) {
		net->rx_dgram_errors++;
		net->rxlen = 0;
		if (dlen > NET_RXBUF_SIZE)
			return;
	}
	memcpy(net->rxbuf + net->rxlen, data, dlen);
	net->rxlen += dlen;

	while (net->rxlen - pos >= DGRAM_OVERHEAD) {
		u8 *q = net->rxbuf + pos;
		u32 len;

		if (q[0] != DGRAM_HEAD) {
			u8 *next = memchr(q, DGRAM_HEAD, net->rxlen - pos);

			net->rx_dgram_errors++;
			pos = next ? next - net->rxbuf : net->rxlen;
			continue;
		}
		len = get_unaligned_le32(q + 1);
		if (art_xor(q, DGRAM_XOR_LEN) != q[5] || len > NET_MAX_DGRAM) {
			net->rx_dgram_errors++;
			pos++;
			continue;
		}
		if (net->rxlen - pos < DGRAM_OVERHEAD + len)
			break; /* rest arrives in a later so_read */
		if (q[DGRAM_HDR_LEN + len] != DGRAM_TAIL) {
			net->rx_dgram_errors++;
			pos++;
			continue;
		}
		net_deliver(net, q + DGRAM_HDR_LEN, len);
		pos += DGRAM_OVERHEAD + len;
	}

	net->rxlen -= pos;
	memmove(net->rxbuf, net->rxbuf + pos, net->rxlen);
}

static void net_rx_write_ack(struct artosyn_net *net, s32 sta, const u8 *data, u32 dlen)
{
	unsigned long flags;
	u64 pos, arg;

	if (sta >= 0) {
		net->ack_ok++;
		return;
	}
	if (dlen < 16) {
		net->ack_other++;
		return;
	}
	pos = get_unaligned_le64(data);
	arg = get_unaligned_le64(data + 8);

	/* Unlike the daemon there is no retransmit buffer: packets the chip
	 * didn't take are simply lost (IP copes), but the stream position has
	 * to follow the chip's or every later write is rejected too. */
	switch (sta) {
	case SO_STA_SEND_OK:
		net->ack_send_ok++;
		break;
	case SO_STA_PENDING:
		net->ack_pending++;
		spin_lock_irqsave(&net->lock, flags);
		net->wr_index = pos;
		spin_unlock_irqrestore(&net->lock, flags);
		art_dbg(net->adev, ART_DBG_TX, "so_write pending: resync pos to %llu\n", pos);
		break;
	case SO_STA_WANTED:
		net->ack_wanted++;
		spin_lock_irqsave(&net->lock, flags);
		net->wr_index = arg;
		spin_unlock_irqrestore(&net->lock, flags);
		art_dbg(net->adev, ART_DBG_TX, "so_write wanted: written %llu, resync pos to %llu\n", pos, arg);
		break;
	default:
		net->ack_other++;
		art_dbg(net->adev, ART_DBG_TX, "so_write reply sta %d\n", sta);
		break;
	}
}

static void net_rx_frame(struct artosyn_net *net, u8 op, s32 sta, const u8 *data, u32 dlen)
{
	net->rpc_rx_frames++;
	art_dbg(net->adev, ART_DBG_RX, "rx rpc op %u sta %d len %u\n", op, sta, dlen);

	switch (op) {
	case SO_READ:
		if (netif_running(net->ndev))
			net_rx_stream(net, data, dlen);
		break;
	case SO_WRITE:
		net_rx_write_ack(net, sta, data, dlen);
		break;
	case SO_OPEN:
		net->open_sta = sta;
		complete(&net->open_done);
		break;
	default:
		art_dbg(net->adev, ART_DBG_INIT, "socket %u/%u: op %u reply sta %d\n",
			net->slot, net->port, op, sta);
		break;
	}
}

struct sk_buff *artosyn_net_rx(struct artosyn_dev *dev, struct sk_buff *skb)
{
	struct artosyn_net *net = dev->net;
	const u8 *p = skb->data;
	unsigned int len = skb->len;
	unsigned int off = 0, keep_from = 0;
	struct sk_buff *rest = NULL;

	if (!net || !READ_ONCE(net->sock_open))
		return skb;

	while (off + RPC_OVERHEAD <= len) {
		u32 dlen, flen, reqid;

		if (p[off] != RPC_HEAD) {
			if (!p[off]) {
				off++;  /* block padding between frames */
				continue;
			}
			/* The chip doesn't clear its transfer buffer, so after
			 * the frame(s) there are usually leftovers of an older,
			 * longer transfer. Only a transfer that doesn't start
			 * with a frame at all is worth counting. */
			if (off == 0)
				net->rx_unparsed++;
			break;  /* leave the rest to userspace untouched */
		}

		dlen = get_unaligned_le32(p + off + 1);
		if (dlen > len - off - RPC_OVERHEAD) {
			net->rx_truncated++;
			break;  /* continues in the next transfer: not ours to split */
		}
		flen = RPC_OVERHEAD + dlen;
		if (p[off + flen - 1] != RPC_TAIL || art_xor(p + off, RPC_XOR_LEN) != p[off + RPC_XOR_LEN]) {
			net->rx_unparsed++;
			break;
		}

		reqid = get_unaligned_be32(p + off + 5);
		if (reqid >> 24 == BB_REQ_SOCKET &&
		    ((reqid >> 8) & 0xff) == net->slot && (reqid & 0xff) == net->port) {
			if (!rest) {
				rest = alloc_skb(len, GFP_KERNEL);
				if (!rest) {
					dev->rx_errors++;
					return skb;
				}
			}
			skb_put_data(rest, p + keep_from, off - keep_from);
			net_rx_frame(net, (reqid >> 16) & 0xff, (s32)get_unaligned_be32(p + off + 13),
				     p + off + RPC_HDR_LEN, dlen);
			keep_from = off + flen;
		}
		off += flen;
	}

	if (!rest)
		return skb;  /* nothing for us: pass the transfer through unchanged */

	skb_put_data(rest, p + keep_from, len - keep_from);
	kfree_skb(skb);
	/* Only padding/stale leftovers without a single frame start left:
	 * nothing the daemon's parser could use, don't wake it for that. */
	if (!memchr(rest->data, RPC_HEAD, rest->len)) {
		kfree_skb(rest);
		return NULL;
	}
	return rest;
}

/* --- net_device ----------------------------------------------------------- */

static int artosyn_net_open(struct net_device *ndev)
{
	struct artosyn_net *net = netdev_priv(ndev);
	struct artosyn_dev *adev = net->adev;
	u8 opt[12];
	long left;
	int ret;

	/* Fresh socket, fresh stream: same as the daemon's sock_node.c,
	 * which starts every socket at buf_wr_index = 0. */
	sdio_claim_host(adev->func);
	net->rxlen = 0;
	sdio_release_host(adev->func);
	net->wr_index = 0;
	net->open_sta = INT_MIN;
	reinit_completion(&net->open_done);
	WRITE_ONCE(net->sock_open, true);

	/* flags, tx_buf_size, rx_buf_size -- the order the userspace client
	 * library puts on the wire (bb_sock_opt_t follows the flags). */
	put_unaligned_le32(BB_SOCK_FLAG_TX | BB_SOCK_FLAG_RX | BB_SOCK_FLAG_DATAGRAM, opt);
	put_unaligned_le32(net_tx_buf, opt + 4);
	put_unaligned_le32(net_rx_buf, opt + 8);

	ret = net_send_ctl(net, SO_OPEN, opt, sizeof(opt));
	if (ret) {
		netdev_err(ndev, "so_open(slot %u port %u) write failed: %d\n", net->slot, net->port, ret);
		goto fail;
	}

	left = wait_for_completion_timeout(&net->open_done, msecs_to_jiffies(NET_OPEN_TIMEOUT_MS));
	if (!left) {
		/* The vendor driver never waits for this reply at all. */
		netdev_warn(ndev, "so_open(slot %u port %u): no reply in %d ms, assuming open\n",
			    net->slot, net->port, NET_OPEN_TIMEOUT_MS);
	} else if (net->open_sta == SO_STA_ALREADY_OPEN) {
		netdev_warn(ndev, "so_open(slot %u port %u): socket was already open on the chip\n",
			    net->slot, net->port);
	} else if (net->open_sta) {
		netdev_err(ndev, "so_open(slot %u port %u) refused by chip: sta %d\n",
			   net->slot, net->port, net->open_sta);
		ret = -EIO;
		goto fail;
	}

	netdev_info(ndev, "bb socket slot %u port %u open (tx_buf %u rx_buf %u)\n",
		    net->slot, net->port, net_tx_buf, net_rx_buf);
	netif_carrier_on(ndev);
	netif_start_queue(ndev);
	return 0;

fail:
	WRITE_ONCE(net->sock_open, false);
	return ret;
}

static int artosyn_net_stop(struct net_device *ndev)
{
	struct artosyn_net *net = netdev_priv(ndev);
	int ret;

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	WRITE_ONCE(net->sock_open, false);
	cancel_work_sync(&net->tx_work);
	skb_queue_purge(&net->txq);

	if (net->adev->removed)
		return 0;

	/* No payload, like the daemon's sock_node.c close. */
	ret = net_send_ctl(net, SO_CLOSE, NULL, 0);
	if (ret)
		netdev_warn(ndev, "so_close(slot %u port %u) write failed: %d\n", net->slot, net->port, ret);
	else
		netdev_info(ndev, "bb socket slot %u port %u closed\n", net->slot, net->port);
	return 0;
}

static int artosyn_net_change_mtu(struct net_device *ndev, int new_mtu)
{
	ndev->mtu = new_mtu;
	return 0;
}

static const struct net_device_ops artosyn_netdev_ops = {
	.ndo_open            = artosyn_net_open,
	.ndo_stop            = artosyn_net_stop,
	.ndo_start_xmit      = artosyn_net_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr   = eth_validate_addr,
	.ndo_change_mtu      = artosyn_net_change_mtu,
};

/* /sys/class/net/<if>/ar8030/stats -- the counters `ip -s link` can't show */
static ssize_t stats_show(struct device *d, struct device_attribute *attr, char *buf)
{
	struct artosyn_net *net = netdev_priv(to_net_dev(d));
	struct artosyn_dev *adev = net->adev;

	return scnprintf(buf, PAGE_SIZE,
			 "socket_open %d slot %u port %u wr_index %llu\n"
			 "irq_events %lu rx_transfers %lu rx_transfer_bytes %lu rx_errors %lu\n"
			 "cdev_rx_drops %lu cdev_rxq_bytes %u cdev_open %d\n"
			 "rpc_rx_frames %lu rx_unparsed %lu rx_truncated %lu rx_dgram_errors %lu\n"
			 "rpc_tx_frames %lu tx_writes %lu txq %u\n"
			 "ack_ok %lu ack_send_ok %lu ack_pending %lu ack_wanted %lu ack_other %lu\n",
			 READ_ONCE(net->sock_open), net->slot, net->port, net->wr_index,
			 adev->irq_events, adev->rx_transfers, adev->rx_transfer_bytes, adev->rx_errors,
			 adev->cdev_rx_drops, adev->cdev_rxq_bytes, atomic_read(&adev->open_count),
			 net->rpc_rx_frames, net->rx_unparsed, net->rx_truncated, net->rx_dgram_errors,
			 net->rpc_tx_frames, net->tx_writes, skb_queue_len(&net->txq),
			 net->ack_ok, net->ack_send_ok, net->ack_pending, net->ack_wanted, net->ack_other);
}
static DEVICE_ATTR_RO(stats);

static struct attribute *artosyn_net_attrs[] = {
	&dev_attr_stats.attr,
	NULL,
};

static const struct attribute_group artosyn_net_group = {
	.name = "ar8030",
	.attrs = artosyn_net_attrs,
};

static void artosyn_net_set_mac(struct net_device *ndev)
{
	u8 mac[ETH_ALEN];

	if (net_mac && mac_pton(net_mac, mac) && is_valid_ether_addr(mac)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
		eth_hw_addr_set(ndev, mac);
#else
		ether_addr_copy(ndev->dev_addr, mac);
#endif
		return;
	}
	if (net_mac)
		netdev_warn(ndev, "invalid net_mac '%s', using a random one\n", net_mac);
	eth_hw_addr_random(ndev);
}

int artosyn_net_probe(struct artosyn_dev *adev)
{
	struct device *dev = &adev->func->dev;
	struct net_device *ndev;
	struct artosyn_net *net;
	int ret;

	if (net_slot > 0xff || net_port > 0xff) {
		dev_err(dev, "native_net: net_slot/net_port out of range (%u/%u)\n", net_slot, net_port);
		return -EINVAL;
	}

	ndev = alloc_netdev(sizeof(*net), net_name, NET_NAME_USER, ether_setup);
	if (!ndev) {
		dev_err(dev, "native_net: alloc_netdev failed\n");
		return -ENOMEM;
	}

	net = netdev_priv(ndev);
	net->adev = adev;
	net->ndev = ndev;
	net->slot = net_slot;
	net->port = net_port;
	spin_lock_init(&net->lock);
	init_completion(&net->open_done);
	skb_queue_head_init(&net->txq);
	INIT_WORK(&net->tx_work, net_tx_work);

	ret = -ENOMEM;
	net->tx_stage = kmalloc(NET_TX_STAGE_SIZE, GFP_KERNEL);
	net->rxbuf = kmalloc(NET_RXBUF_SIZE, GFP_KERNEL);
	if (!net->tx_stage || !net->rxbuf) {
		dev_err(dev, "native_net: buffer allocation failed\n");
		goto err_free;
	}

	net->wq = alloc_ordered_workqueue("artosyn_net_tx", WQ_HIGHPRI | WQ_MEM_RECLAIM);
	if (!net->wq) {
		dev_err(dev, "native_net: workqueue allocation failed\n");
		goto err_free;
	}

	SET_NETDEV_DEV(ndev, dev);
	ndev->netdev_ops = &artosyn_netdev_ops;
	ndev->priv_flags &= ~IFF_TX_SKB_SHARING;
	ndev->max_mtu = NET_MAX_MTU;
	ndev->sysfs_groups[0] = &artosyn_net_group;
	artosyn_net_set_mac(ndev);

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(dev, "native_net: register_netdev(%s) failed: %d\n", net_name, ret);
		goto err_wq;
	}
	netif_carrier_off(ndev);

	/* Publish to the RX drain last, under the host claim it runs with. */
	sdio_claim_host(adev->func);
	adev->net = net;
	sdio_release_host(adev->func);

	dev_info(dev, "native_net: registered %s (%pM), bb socket slot %u port %u on ifup\n",
		 ndev->name, ndev->dev_addr, net->slot, net->port);
	return 0;

err_wq:
	destroy_workqueue(net->wq);
err_free:
	kfree(net->rxbuf);
	kfree(net->tx_stage);
	free_netdev(ndev);
	return ret;
}

void artosyn_net_remove(struct artosyn_dev *adev)
{
	struct artosyn_net *net = adev->net;
	struct net_device *ndev;

	if (!net)
		return;
	ndev = net->ndev;

	/* ndo_stop (socket close) runs in here if the interface is up, while
	 * the RX drain still routes this socket's replies to us. */
	unregister_netdev(ndev);

	sdio_claim_host(adev->func);
	adev->net = NULL;
	sdio_release_host(adev->func);

	destroy_workqueue(net->wq);
	skb_queue_purge(&net->txq);
	kfree(net->rxbuf);
	kfree(net->tx_stage);
	dev_info(&adev->func->dev, "native_net: %s unregistered\n", ndev->name);
	free_netdev(ndev);
}
