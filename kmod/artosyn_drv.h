/* SPDX-License-Identifier: GPL-2.0 */
/*
 * artosyn_drv.h -- shared between the SDIO core (artosyn_sdio.c) and the
 * native net_device layer (artosyn_net.c). See ../doc/native-netdev.md for
 * how the two fit together and which vendor code each part replaces.
 */
#ifndef ARTOSYN_DRV_H
#define ARTOSYN_DRV_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/skbuff.h>
#include <linux/miscdevice.h>
#include <linux/mmc/sdio_func.h>

/* Shared with the vendor tree (see Makefile's AR8030_SDK_DRIVER_INC):
 * ioctl commands, artosyn_{rw,msg,cmd}_args, FIFO_ADDRESS,
 * SDIO_TRANS_MAX_SZIE, SDIO_MAILBOX_CHANNEL_COUNT, SDIO_DEVICE_BLOCK_SIZE,
 * SDIO_BOOT_*, ARTOSDIO_MAGIC_HEADER_*. */
#include "sdio.h"

#define DRV_NAME "artosyn_drv"

/* Debug classes for the `debug` module parameter (bitmask, writable at
 * runtime via /sys/module/artosyn_drv/parameters/debug). 0 = quiet;
 * errors and one-line lifecycle messages are always printed. */
#define ART_DBG_INIT   0x01 /* probe/remove, netdev + socket lifecycle */
#define ART_DBG_IRQ    0x02 /* every mailbox event */
#define ART_DBG_RX     0x04 /* every RX transfer / RPC frame */
#define ART_DBG_TX     0x08 /* every RPC frame written by the netdev */
#define ART_DBG_DUMP   0x10 /* hex dumps of the above */

extern unsigned int artosyn_debug;

#define art_dbg(adev, cls, fmt, ...)						\
	do {									\
		if (unlikely(artosyn_debug & (cls)))				\
			dev_info(&(adev)->func->dev, fmt, ##__VA_ARGS__);	\
	} while (0)

struct artosyn_net;

struct artosyn_dev {
	struct sdio_func *func;
	struct miscdevice miscdev;
	char name[32];

	struct mutex io_mutex;   /* serializes read()/write() vs. each other and vs. remove() */
	bool removed;

	/* Serializes every write into the chip's FIFO -- /dev/artosyn_sdio
	 * write() and the netdev TX worker. Held across the wait for a
	 * TX-ready window, so a second writer queues up behind the first
	 * instead of getting -EBUSY. Each holder writes whole RPC frames, so
	 * frames from the two sources never interleave mid-frame. */
	struct mutex tx_mutex;

	/* True only while the chip is still running its boot-ROM firmware
	 * (i.e. between insmod and artosyn_download_firmware() completing).
	 * The boot ROM's own SDIO handling doesn't generate the normal
	 * TX/RX-ready mailbox events at all -- waiting on them here would
	 * simply time out on every firmware chunk. This project's own
	 * already-working combined driver has the exact same bypass
	 * (dev->rom_mode in bus/sdio.c) for the exact same reason; missing
	 * it was this rewrite's very first real-hardware bug. */
	bool rom_mode;

	wait_queue_head_t rx_q;
	wait_queue_head_t tx_q;
	wait_queue_head_t mailbox_q;

	unsigned int read_offset;
	unsigned int read_valid_size;
	bool reading;

	unsigned int write_offset;
	unsigned int write_valid_size;

	unsigned char message[SDIO_MAILBOX_CHANNEL_COUNT];
	unsigned char msg_valid[SDIO_MAILBOX_CHANNEL_COUNT];

	/* Native networking (RTOS-id probe with native_net=1 only). When set,
	 * the kernel drains every RX transfer itself (from the IRQ drain,
	 * artosyn_check_events()) and hands it to artosyn_net_rx(); whatever
	 * isn't for the netdev is queued on cdev_rxq for /dev/artosyn_sdio
	 * read(). When clear, read() drains the FIFO directly as before. */
	bool net_mode;
	struct artosyn_net *net;
	struct sk_buff_head cdev_rxq;
	unsigned int cdev_rxq_bytes;   /* protected by cdev_rxq.lock */
	void *rx_scratch;              /* SDIO_TRANS_MAX_SZIE, drain target if alloc_skb() fails */
	atomic_t open_count;

	/* Counters, reported via the netdev's sysfs stats file. */
	unsigned long irq_events;
	unsigned long rx_transfers;
	unsigned long rx_transfer_bytes;
	unsigned long rx_errors;
	unsigned long cdev_rx_drops;
};

/* artosyn_sdio.c: write whole buffer into the FIFO, looping over as many
 * TX-ready windows as needed while holding tx_mutex. */
int artosyn_write_all(struct artosyn_dev *dev, const void *buf, size_t len);

/* artosyn_net.c */
int artosyn_net_probe(struct artosyn_dev *dev);
void artosyn_net_remove(struct artosyn_dev *dev);
/* Consumes the RPC frames addressed to the netdev's socket from one RX
 * transfer. Returns what is left for userspace (possibly skb itself,
 * possibly a new skb, or NULL if nothing is left). Called with the SDIO
 * host claimed, from process context. */
struct sk_buff *artosyn_net_rx(struct artosyn_dev *dev, struct sk_buff *skb);

#endif /* ARTOSYN_DRV_H */
