// SPDX-License-Identifier: GPL-2.0
/*
 * artosyn_drv.c -- clean-room AR8030 SDIO chardev driver.
 *
 * This is a from-scratch rewrite, not a patched copy of anything. It grew
 * out of an extended debugging investigation (see ../README.md's "SDIO
 * chardev" section for the full history) into why this project's own
 * incrementally-patched combined driver (ascent/8030_sdk/yz_host_drv/
 * driver/linux, DRV+SDIO+USB all in one module) still stalled under
 * sustained SDIO throughput while the vendor's real, standalone
 * artosyn_sdio.ko never did, even after several genuine bugs in the
 * patched version were found and fixed. Rather than keep patching that
 * architecture bug-by-bug, this reimplements just the SDIO chardev
 * (/dev/artosyn_sdio) from scratch, informed by:
 *
 *   - Ghidra-decompiling the vendor's real artosyn_sdio.ko (pulled from a
 *     stock Ascent firmware release) function-by-function -- all ~30 of
 *     its non-thunk functions were read during this investigation.
 *   - This project's own hardware-validated register-level protocol
 *     knowledge (patches 0003/0004/0010/0012 in builder/package/ar8030/,
 *     confirmed correct via extensive real-hardware testing) -- reused
 *     directly rather than re-derived from ambiguous decompiled offset
 *     arithmetic where this project already has a proven answer.
 *   - The vendor's own architecture where it's cleaner than this
 *     project's history ended up: ONE register-drain routine shared by
 *     both the real hardware IRQ and poll() (see artosyn_check_events()
 *     below), rather than two hand-duplicated copies (irqhandler +
 *     workqueue-based reg_check fallback) that have to be kept in sync
 *     by hand -- a real, confirmed source of bugs in this project's own
 *     patched driver (0004 had to fix the exact same decode bug in two
 *     places; 0011's shared-waitqueue fix likewise).
 *
 * Deliberately out of scope: the OAL/DRV-mode multiplexed transport
 * (/dev/ar_mdev<N>, driver/linux/oal_mdev.c) is untouched, separate,
 * already-working code -- this module does not coexist with it (only
 * one driver can own the physical SDIO function at a time); it's a
 * drop-in alternative for SDIO-mode use, not an addition to DRV mode.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

/* Shared with the existing combined driver (see this file's own Makefile
 * for the include path): ioctl commands, artosyn_{rw,msg,cmd}_args, and
 * the handful of protocol constants (FIFO_ADDRESS, SDIO_TRANS_MAX_SZIE,
 * SDIO_MAILBOX_CHANNEL_COUNT, SDIO_DEVICE_BLOCK_SIZE, SDIO_BOOT_*,
 * ARTOSDIO_MAGIC_HEADER_*) this rewrite has no reason to redefine. */
#include "sdio.h"

#define DRV_NAME "artosyn_drv"

/* Declared here (rather than down by their module_param() calls) since
 * artosyn_download_firmware() needs them and is defined well before
 * that point in this file. */
static char *fw_name;
static char *cfg_name;

/* SDIO function/vendor/device IDs. The chip enumerates as ARTO_ROMCODE_*
 * (its boot-ROM identity) on power-up; after a successful firmware push
 * it re-enumerates as ARTO_RTOS_ALT_* (or, on some units, ARTO_RTOS_*) --
 * probe() runs a second time for that second identity, and must *not*
 * redo the firmware push then (see sdio_device_id table below and the
 * is_rtos_id() check in artosyn_probe()). Missing the post-boot ID here
 * is a real, previously-hit bug in this project's own patched driver
 * (0003-sdio-recognize-the-post-boot-8031-device-id.patch) -- probe()
 * simply never runs again without a matching table entry, so
 * /dev/artosyn_sdio never appears even though firmware upload succeeded. */
#define ARTO_ROMCODE_VID       0x4152
#define ARTO_ROMCODE_PID       0x8030
#define ARTO_RTOS_VID          0x1d6b
#define ARTO_RTOS_PID          0x8030
#define ARTO_RTOS_ALT_VID      0x4152
#define ARTO_RTOS_ALT_PID      0x8031

/* Register map (function-1 register space), reverse-engineered and
 * confirmed correct via extensive real-hardware testing in this
 * project's own patched driver -- see builder/package/ar8030/0004-*
 * for the size-decode bug this got right that an earlier attempt at
 * this same offset table got wrong (0x5c/0x60 are single-byte *block
 * counts*, not part of some wider multi-byte value spanning other
 * unrelated addresses). */
#define REG_IRQ_STATUS         0x13 /* bit0: legacy, unused; bit4: mailbox event pending */
#define REG_INT_ENABLE         0x14 /* write 1 once at probe time to enable interrupts */
#define REG_MAILBOX_MSG(ch)    (0x54 + (ch) * 4) /* one byte per channel, ch < SDIO_MAILBOX_CHANNEL_COUNT */
#define REG_RX_READY_BLOCKS    0x5c /* byte count of newly-available read data, in SDIO_DEVICE_BLOCK_SIZE units */
#define REG_TX_READY_BLOCKS    0x60 /* byte count of newly-available write room, in SDIO_DEVICE_BLOCK_SIZE units */
#define REG_MAILBOX_ENABLE     0x68 /* write 0xf0 once at probe time; read back as a 4-bit event mask (bit i: mailbox channel i; bit2: rx-ready; bit3: tx-ready) */

#define MB_EVT_RX_READY        (1 << 2)
#define MB_EVT_TX_READY        (1 << 3)

/* Same values this project's own artosyn_sdio_write()/artosyn_read()
 * settled on after 0012/0014 replaced a 100ms-capped msleep-and-repoll
 * loop with a real interrupt-driven wait on each side in turn -- 0012
 * for write (confirmed real hardware improvement), 0014 for read
 * (symmetry fix, same anti-pattern 0012 already diagnosed). Keeping the
 * same tuning here since it's already validated, not because 2000ms is
 * some fundamental constant. */
#define SDIO_WRITE_WAIT_MS     2000
#define SDIO_READ_WAIT_MS      2000

/* Firmware image header, matching the format this project's own
 * oal_dnld.c (driver/linux/oal_dnld.h, STRU_SPL_HEADER) already parses
 * for the combined driver's own firmware push -- copied here rather than
 * pulled in via that header, which drags in the much larger OAL/DRV-mode
 * machinery (oal_main.h) this standalone module has no other use for. */
#define SPL_HEADER_LOAD_ADDR   (0x002f0000 + 0x40)
#define SPL_IMAGE_SIZE_ALIGN   512
#define ROUNDUP(x, y)          (((x) + ((y) - 1)) & ~((y) - 1))

#pragma pack(push, 1)
struct spl_header {
	unsigned int magic;
	unsigned short img_type;
	unsigned short header_len;
	unsigned int header_checksum;
	unsigned int img_version;
	unsigned int flag;
	unsigned int boot_info;
	unsigned int spl_load_addr;
	unsigned int spl_len;
	unsigned int troot_load_addr;
	unsigned int troot_len;
	unsigned int signature_load_addr;
	unsigned int signature_len;
	unsigned int spl_dtb_offset;
	unsigned long checksum;
	unsigned int id_masks;
	unsigned char patch_len[8];
};
#pragma pack(pop)

struct artosyn_dev {
	struct sdio_func *func;
	struct miscdevice miscdev;
	char name[32];

	struct mutex io_mutex;   /* serializes read()/write() vs. each other and vs. remove() */
	bool removed;

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
	bool writing;

	unsigned char message[SDIO_MAILBOX_CHANNEL_COUNT];
	unsigned char msg_valid[SDIO_MAILBOX_CHANNEL_COUNT];
};

/* One instance at a time: this chip exposes a single SDIO function, and
 * sdio_claim_irq() only ever registers one handler for it. Matches the
 * vendor's own module (a single static "the device" pointer, not a
 * lookup table) -- this isn't a multi-card driver. */
static struct artosyn_dev *g_dev;

/*
 * Shared register-drain routine.
 *
 * This is the one piece of architecture deliberately borrowed from the
 * vendor's real driver rather than this project's own patched one: the
 * vendor's poll() calls its irqhandler equivalent *directly*, host
 * already claimed, as a self-heal against a missed real interrupt --
 * this project's own combined driver never had an equivalent on the
 * read side at all, only a narrower one wired into artosyn_sdio_write()
 * itself. Confirmed on real hardware (this same investigation, see the
 * README's own writeup) that porting *just* the poll()-calls-the-drain-
 * routine idea onto that project's existing dual irqhandler/reg_check
 * architecture caused a severe regression -- almost certainly extra
 * SDIO bus contention from the added register reads under saturated
 * throughput, competing with the real data-transfer traffic, since that
 * was tested at ~10Hz+ call rates under load. Here there is no separate
 * reg_check path to duplicate or contend with: this exact function is
 * the *only* place mailbox state ever gets read, called from exactly
 * two places (the real IRQ handler's own drain loop below, and poll()
 * when idle) -- matching the vendor's actual structure, not a patched
 * approximation of it, so that regression's root cause (two independent
 * paths touching the same registers) doesn't apply here.
 *
 * Caller must already hold the SDIO host claim. Returns true if an
 * event was found and handled (caller should keep draining), false once
 * the status register reports nothing pending.
 */
static bool artosyn_check_events(struct artosyn_dev *dev)
{
	struct sdio_func *func = dev->func;
	int err = -1;
	u8 iir;
	u8 mb_ena;
	int ch;

	iir = sdio_readb(func, REG_IRQ_STATUS, &err);
	if (err || !iir)
		return false;

	if (!(iir & (1 << 4)))
		return true; /* status byte was non-zero but carried nothing we handle; keep draining */

	mb_ena = sdio_readb(func, REG_MAILBOX_ENABLE, NULL);

	for (ch = 0; ch < SDIO_MAILBOX_CHANNEL_COUNT; ch++) {
		if (mb_ena & (1 << ch)) {
			dev->message[ch] = sdio_readb(func, REG_MAILBOX_MSG(ch), NULL);
			dev->msg_valid[ch] = 1;
			wake_up_interruptible_all(&dev->mailbox_q);
		}
	}

	if (mb_ena & MB_EVT_RX_READY) {
		dev->read_offset = 0;
		dev->read_valid_size = (unsigned int)sdio_readb(func, REG_RX_READY_BLOCKS, NULL) << 9;
		wake_up_interruptible_all(&dev->rx_q);
	}

	if (mb_ena & MB_EVT_TX_READY) {
		dev->write_offset = 0;
		dev->write_valid_size = (unsigned int)sdio_readb(func, REG_TX_READY_BLOCKS, NULL) << 9;
		wake_up_interruptible_all(&dev->tx_q);
	}

	return true;
}

static void artosyn_irqhandler(struct sdio_func *func)
{
	struct artosyn_dev *dev = sdio_get_drvdata(func);

	if (!dev)
		return;

	/* SDIO core already holds the host claim for the duration of a
	 * real registered IRQ handler -- do not claim it here. Drain
	 * everything pending in one shot; the status register goes back
	 * to reporting nothing once we've caught up. */
	while (artosyn_check_events(dev))
		;
}

static bool artosyn_read_condition(struct artosyn_dev *dev)
{
	return dev->removed || dev->read_offset < dev->read_valid_size;
}

static bool artosyn_write_condition(struct artosyn_dev *dev)
{
	return dev->removed || dev->write_offset < dev->write_valid_size;
}

/*
 * Low-level write: waits (patiently -- see SDIO_WRITE_WAIT_MS's own
 * comment) for room, then hands at most one SDIO_TRANS_MAX_SZIE-sized,
 * block-aligned chunk to the chip's FIFO. May return less than count;
 * callers loop (this project's own userspace already does, in both
 * ar8030-transport-tx's chunk_send_to_socket() and the AR8030 SDK
 * daemon's sdio_write()) -- a short return here is normal flow control,
 * not an error, exactly like the vendor's real driver and this
 * project's existing one.
 */
static ssize_t artosyn_do_write(struct artosyn_dev *dev, const void *buf, size_t count)
{
	struct sdio_func *func = dev->func;
	unsigned int room;
	int ret;
	long left;

	mutex_lock(&dev->io_mutex);
	if (dev->removed) {
		mutex_unlock(&dev->io_mutex);
		return -EIO;
	}
	if (dev->writing) {
		mutex_unlock(&dev->io_mutex);
		return -EBUSY;
	}
	dev->writing = true;
	mutex_unlock(&dev->io_mutex);

	if (dev->rom_mode) {
		/* Boot ROM doesn't generate TX-ready mailbox events -- see
		 * this field's own comment on struct artosyn_dev. Force the
		 * window open for exactly this call instead of waiting for
		 * a condition that will never become true on its own. */
		dev->write_offset = 0;
		dev->write_valid_size = count;
	} else {
		/* Courtesy poll before the real wait, in case the mailbox
		 * interrupt that would satisfy write_condition() below
		 * already came and went (missed) before we got here -- this
		 * is the *only* recovery for a lost interrupt now, since
		 * there's no separate workqueue fallback in this design;
		 * poll() (see artosyn_poll() below) covers the same case for
		 * callers blocked in poll() rather than a blocking write(). */
		sdio_claim_host(func);
		artosyn_check_events(dev);
		sdio_release_host(func);

		left = wait_event_interruptible_timeout(dev->tx_q, artosyn_write_condition(dev),
							 msecs_to_jiffies(SDIO_WRITE_WAIT_MS));
		if (left <= 0) {
			dev->writing = false;
			return -EIO;
		}
	}

	mutex_lock(&dev->io_mutex);
	if (dev->removed) {
		dev->writing = false;
		mutex_unlock(&dev->io_mutex);
		return -EIO;
	}

	sdio_claim_host(func);

	room = dev->write_valid_size - dev->write_offset;
	if (count > room)
		count = room;
	if (count > SDIO_TRANS_MAX_SZIE)
		count = SDIO_TRANS_MAX_SZIE;
	if (count > func->cur_blksize)
		count -= count % func->cur_blksize;

	if (count == 0) {
		sdio_release_host(func);
		dev->writing = false;
		mutex_unlock(&dev->io_mutex);
		return -EAGAIN;
	}

	ret = sdio_memcpy_toio(func, FIFO_ADDRESS, (void *)buf, count);
	if (!ret)
		dev->write_offset = dev->write_valid_size;

	sdio_release_host(func);
	dev->writing = false;
	mutex_unlock(&dev->io_mutex);

	return ret ? ret : (ssize_t)count;
}

static ssize_t artosyn_do_read(struct artosyn_dev *dev, void *buf, size_t count)
{
	struct sdio_func *func = dev->func;
	unsigned int avail;
	int ret;
	long left;

	mutex_lock(&dev->io_mutex);
	if (dev->removed) {
		mutex_unlock(&dev->io_mutex);
		return -EIO;
	}
	if (dev->reading) {
		mutex_unlock(&dev->io_mutex);
		return -EBUSY;
	}
	dev->reading = true;
	mutex_unlock(&dev->io_mutex);

	if (dev->rom_mode) {
		/* See the identical bypass in artosyn_do_write() -- same
		 * reasoning, kept symmetric even though nothing in this
		 * driver currently reads during firmware download. */
		dev->read_offset = 0;
		dev->read_valid_size = count;
	} else {
		sdio_claim_host(func);
		artosyn_check_events(dev);
		sdio_release_host(func);

		left = wait_event_interruptible_timeout(dev->rx_q, artosyn_read_condition(dev),
							 msecs_to_jiffies(SDIO_READ_WAIT_MS));
		if (left <= 0) {
			dev->reading = false;
			return -EIO;
		}
	}

	mutex_lock(&dev->io_mutex);
	if (dev->removed) {
		dev->reading = false;
		mutex_unlock(&dev->io_mutex);
		return -EIO;
	}

	sdio_claim_host(func);

	avail = dev->read_valid_size - dev->read_offset;
	if (count > avail)
		count = avail;
	if (count > SDIO_TRANS_MAX_SZIE)
		count = SDIO_TRANS_MAX_SZIE;

	if (count == 0) {
		sdio_release_host(func);
		dev->reading = false;
		mutex_unlock(&dev->io_mutex);
		return -EAGAIN;
	}

	ret = sdio_memcpy_fromio(func, buf, FIFO_ADDRESS, count);
	if (!ret)
		dev->read_offset += count;

	sdio_release_host(func);
	dev->reading = false;
	mutex_unlock(&dev->io_mutex);

	return ret ? ret : (ssize_t)count;
}

/*
 * Firmware download, run once from probe() when the chip enumerates
 * under its boot-ROM identity. Chunking protocol (the "SD"-magic
 * 12-byte header ahead of each payload chunk, block-aligned residue
 * handling, a final zero-length "done" packet) matches this project's
 * own already-working sdio_rom_send() -- proven correct via this whole
 * investigation's extensive real-hardware testing, so reused as-is
 * rather than re-derived from the vendor's own considerably denser
 * decompiled equivalent (FUN_00010a04).
 */
static int artosyn_fw_send_chunk(struct artosyn_dev *dev, const void *data, u32 dest_addr, u32 len)
{
	u8 *tx;
	u32 sent_total = 0;

	if (len == 0) {
		u8 done[SDIO_BOOT_PKG_HEADER_SIZE] = { ARTOSDIO_MAGIC_HEADER_1, ARTOSDIO_MAGIC_HEADER_2 };
		ssize_t sent = artosyn_do_write(dev, done, sizeof(done));

		return sent == sizeof(done) ? 0 : -EIO;
	}

	tx = kzalloc(SDIO_BOOT_ONESHOT_MAX_SIZE, GFP_KERNEL);
	if (!tx)
		return -ENOMEM;

	while (sent_total < len) {
		u32 chunk = len - sent_total;
		u32 pkt_len;
		ssize_t sent;

		if (chunk > SDIO_BOOT_ONESHOT_MAX_SIZE - SDIO_BOOT_PKG_HEADER_SIZE)
			chunk = SDIO_BOOT_ONESHOT_MAX_SIZE - SDIO_BOOT_PKG_HEADER_SIZE;

		/* Keep each on-wire packet's payload a multiple of
		 * SDIO_DEVICE_BLOCK_SIZE once header + payload cross that
		 * boundary, splitting the remainder into its own follow-up
		 * packet(s) -- same block-alignment rule this project's own
		 * sdio_rom_send() already applies. */
		if (chunk + SDIO_BOOT_PKG_HEADER_SIZE > SDIO_DEVICE_BLOCK_SIZE &&
		    (chunk + SDIO_BOOT_PKG_HEADER_SIZE) % SDIO_DEVICE_BLOCK_SIZE)
			chunk -= (chunk + SDIO_BOOT_PKG_HEADER_SIZE) % SDIO_DEVICE_BLOCK_SIZE;

		pkt_len = chunk + SDIO_BOOT_PKG_HEADER_SIZE;

		memset(tx, 0, SDIO_BOOT_PKG_HEADER_SIZE);
		tx[0] = ARTOSDIO_MAGIC_HEADER_1;
		tx[1] = ARTOSDIO_MAGIC_HEADER_2;
		*(u16 *)&tx[2] = (u16)chunk;
		*(u64 *)&tx[4] = dest_addr + sent_total;
		memcpy(tx + SDIO_BOOT_PKG_HEADER_SIZE, (const u8 *)data + sent_total, chunk);

		sent = artosyn_do_write(dev, tx, pkt_len);
		if (sent != pkt_len) {
			dev_err(&dev->func->dev, "%s: chunk send failed at offset %u (ret=%zd)\n",
				__func__, sent_total, sent);
			kfree(tx);
			return -EIO;
		}

		sent_total += chunk;
	}

	kfree(tx);
	return 0;
}

static int artosyn_download_firmware(struct artosyn_dev *dev)
{
	const struct firmware *fw = NULL, *cfg = NULL;
	const struct spl_header *hdr;
	u32 offset;
	int ret;

	if (!fw_name) {
		dev_err(&dev->func->dev, "no fw_name= module param given, cannot boot the chip\n");
		return -EINVAL;
	}

	ret = request_firmware(&fw, fw_name, &dev->func->dev);
	if (ret) {
		dev_err(&dev->func->dev, "request_firmware(%s) failed: %d\n", fw_name, ret);
		return ret;
	}

	if (cfg_name) {
		ret = request_firmware(&cfg, cfg_name, &dev->func->dev);
		if (ret)
			dev_warn(&dev->func->dev, "request_firmware(%s) failed: %d (continuing without it)\n",
				 cfg_name, ret);
	}

	hdr = (const struct spl_header *)fw->data;
	offset = ROUNDUP(hdr->header_len, SPL_IMAGE_SIZE_ALIGN);

	ret = artosyn_fw_send_chunk(dev, fw->data, SPL_HEADER_LOAD_ADDR, hdr->header_len);
	if (ret)
		goto out;

	if (hdr->troot_len) {
		ret = artosyn_fw_send_chunk(dev, fw->data + offset, hdr->troot_load_addr, hdr->troot_len);
		if (ret)
			dev_err(&dev->func->dev, "upgrade transfer failed (troot, addr=0x%x)\n",
				hdr->troot_load_addr);
		offset += ROUNDUP(hdr->troot_len, SPL_IMAGE_SIZE_ALIGN);

		ret = artosyn_fw_send_chunk(dev, fw->data + offset, hdr->signature_load_addr, hdr->signature_len);
		if (ret)
			dev_err(&dev->func->dev, "upgrade transfer failed (signature, addr=0x%x)\n",
				hdr->signature_load_addr);
		offset += ROUNDUP(hdr->signature_len, SPL_IMAGE_SIZE_ALIGN);
	}

	ret = artosyn_fw_send_chunk(dev, fw->data + offset, hdr->spl_load_addr, hdr->spl_len);
	if (ret) {
		dev_err(&dev->func->dev, "upgrade transfer failed (spl, addr=0x%x)\n", hdr->spl_load_addr);
		goto out;
	}
	dev_info(&dev->func->dev, "upgrade spl success\n");

	if (cfg) {
		ret = artosyn_fw_send_chunk(dev, cfg->data, hdr->signature_load_addr, cfg->size);
		if (ret) {
			dev_err(&dev->func->dev, "upgrade bb_cfg failed (addr=0x%x)\n", hdr->signature_load_addr);
			goto out;
		}
		dev_info(&dev->func->dev, "upgrade bb_cfg success\n");
	}

	ret = artosyn_fw_send_chunk(dev, NULL, 0, 0); /* zero-length "done" packet */
	if (ret) {
		dev_err(&dev->func->dev, "exit_usb_boot_in_bl1 failed: %d\n", ret);
		goto out;
	}
	dev_info(&dev->func->dev, "exit_usb_boot_in_bl1 bb_cfg success\n");

out:
	if (cfg)
		release_firmware(cfg);
	release_firmware(fw);
	return ret;
}

/* --- chardev fops ------------------------------------------------- */

static int artosyn_open(struct inode *inode, struct file *filp)
{
	if (!g_dev)
		return -ENODEV;
	filp->private_data = g_dev;
	return 0;
}

static int artosyn_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t artosyn_fops_read(struct file *filp, char __user *ubuf, size_t count, loff_t *ppos)
{
	struct artosyn_dev *dev = filp->private_data;
	void *bounce;
	ssize_t ret;

	if (count > SDIO_TRANS_MAX_SZIE)
		count = SDIO_TRANS_MAX_SZIE;

	bounce = kmalloc(count, GFP_KERNEL);
	if (!bounce)
		return -ENOMEM;

	ret = artosyn_do_read(dev, bounce, count);
	if (ret > 0 && copy_to_user(ubuf, bounce, ret))
		ret = -EFAULT;

	kfree(bounce);
	return ret;
}

static ssize_t artosyn_fops_write(struct file *filp, const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct artosyn_dev *dev = filp->private_data;
	void *bounce;
	ssize_t ret;

	if (count > SDIO_TRANS_MAX_SZIE)
		count = SDIO_TRANS_MAX_SZIE;

	bounce = kmalloc(count, GFP_KERNEL);
	if (!bounce)
		return -ENOMEM;

	if (copy_from_user(bounce, ubuf, count)) {
		kfree(bounce);
		return -EFAULT;
	}

	ret = artosyn_do_write(dev, bounce, count);
	kfree(bounce);
	return ret;
}

static __poll_t artosyn_poll(struct file *filp, poll_table *wait)
{
	struct artosyn_dev *dev = filp->private_data;
	__poll_t mask = 0;

	poll_wait(filp, &dev->rx_q, wait);
	poll_wait(filp, &dev->tx_q, wait);

	if (dev->removed)
		return POLLERR;

	/* This is the vendor-matching self-heal this rewrite exists to add:
	 * see artosyn_check_events()'s own comment for why it's safe *here*
	 * in a way it wasn't when tried on this project's older, dual-path
	 * combined driver. */
	sdio_claim_host(dev->func);
	if (!artosyn_read_condition(dev) && !artosyn_write_condition(dev))
		artosyn_check_events(dev);
	sdio_release_host(dev->func);

	if (artosyn_read_condition(dev))
		mask |= POLLIN | POLLRDNORM;
	if (artosyn_write_condition(dev))
		mask |= POLLOUT | POLLWRNORM;

	return mask;
}

static long artosyn_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct artosyn_dev *dev = filp->private_data;
	void __user *uarg = (void __user *)arg;

	switch (cmd) {
	case READ_MESSAGE: {
		/* Query-and-clear, matching the vendor's real ioctl (Ghidra:
		 * daemon_sdiov12's artosyn_unlocked_ioctl equivalent uses
		 * this exact request/reply shape) -- request.valid[i] means
		 * "tell me about channel i"; reply carries message[i] and
		 * clears msg_valid[i] as a side effect, so a channel already
		 * consumed doesn't report stale data to the next caller. */
		struct artosyn_msg_args msg;
		int ch;

		if (copy_from_user(&msg, uarg, sizeof(msg)))
			return -EFAULT;

		for (ch = 0; ch < SDIO_MAILBOX_CHANNEL_COUNT; ch++) {
			if (msg.valid[ch] && dev->msg_valid[ch]) {
				msg.message[ch] = dev->message[ch];
				dev->msg_valid[ch] = 0;
			} else {
				msg.valid[ch] = 0;
			}
		}

		if (copy_to_user(uarg, &msg, sizeof(msg)))
			return -EFAULT;
		return 0;
	}
	case READ_BYTE:
	case WRITE_BYTE:
	case READ_F0_BYTE:
	case WRITE_F0_BYTE: {
		struct artosyn_rw_args rw;
		int err = 0;

		if (copy_from_user(&rw, uarg, sizeof(rw)))
			return -EFAULT;

		sdio_claim_host(dev->func);
		if (cmd == READ_BYTE)
			rw.val = sdio_readb(dev->func, rw.addr, &err);
		else if (cmd == WRITE_BYTE)
			sdio_writeb(dev->func, rw.val, rw.addr, &err);
		else if (cmd == READ_F0_BYTE)
			rw.val = sdio_f0_readb(dev->func, rw.addr, &err);
		else
			sdio_f0_writeb(dev->func, rw.val, rw.addr, &err);
		sdio_release_host(dev->func);

		if (err)
			return err;
		if ((cmd == READ_BYTE || cmd == READ_F0_BYTE) && copy_to_user(uarg, &rw, sizeof(rw)))
			return -EFAULT;
		return 0;
	}
	default:
		return -ENOTTY;
	}
}

static const struct file_operations artosyn_fops = {
	.owner = THIS_MODULE,
	.open = artosyn_open,
	.release = artosyn_release,
	.read = artosyn_fops_read,
	.write = artosyn_fops_write,
	.poll = artosyn_poll,
	.unlocked_ioctl = artosyn_ioctl,
};

/* --- SDIO driver plumbing ------------------------------------------ */

static bool is_rtos_id(struct sdio_func *func)
{
	return !(func->vendor == ARTO_ROMCODE_VID && func->device == ARTO_ROMCODE_PID);
}

static int artosyn_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
	struct artosyn_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->func = func;
	mutex_init(&dev->io_mutex);
	init_waitqueue_head(&dev->rx_q);
	init_waitqueue_head(&dev->tx_q);
	init_waitqueue_head(&dev->mailbox_q);
	sdio_set_drvdata(func, dev);

	sdio_claim_host(func);

	ret = sdio_enable_func(func);
	if (ret) {
		dev_err(&func->dev, "sdio_enable_func failed: %d\n", ret);
		goto err_release_host;
	}

	ret = sdio_claim_irq(func, artosyn_irqhandler);
	if (ret) {
		dev_err(&func->dev, "sdio_claim_irq failed: %d\n", ret);
		goto err_disable_func;
	}

	/* One-time interrupt/mailbox enable. Register values reverse-
	 * engineered from the vendor's real probe() (Ghidra) and confirmed
	 * to match this project's own already-working combined driver. */
	sdio_writeb(func, 1, REG_INT_ENABLE, NULL);
	sdio_writeb(func, 0xf0, REG_MAILBOX_ENABLE, NULL);

	sdio_release_host(func);

	snprintf(dev->name, sizeof(dev->name), "artosyn_sdio");
	dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	dev->miscdev.name = dev->name;
	dev->miscdev.fops = &artosyn_fops;

	ret = misc_register(&dev->miscdev);
	if (ret) {
		dev_err(&func->dev, "misc_register failed: %d\n", ret);
		goto err_claim_host_for_cleanup;
	}

	if (is_rtos_id(func)) {
		/* Already running the chip's own RTOS firmware (this is the
		 * post-firmware-push re-enumeration, or a unit that never
		 * needed a push in the first place) -- nothing left to do. */
		dev_info(&func->dev, "%s: chip already running RTOS firmware (vid=0x%04x pid=0x%04x)\n",
			 DRV_NAME, func->vendor, func->device);
		g_dev = dev;
		return 0;
	}

	dev->rom_mode = true;
	ret = artosyn_download_firmware(dev);
	dev->rom_mode = false;
	if (ret) {
		dev_err(&func->dev, "firmware download failed: %d\n", ret);
		misc_deregister(&dev->miscdev);
		goto err_claim_host_for_cleanup;
	}

	/* The chip will now reset its SDIO identity and re-enumerate as
	 * ARTO_RTOS_ALT_x or ARTO_RTOS_x id -- probe() runs again for that
	 * new identity (see the is_rtos_id() branch above), this instance's
	 * job is done. Leave dev in place; the mmc core will call
	 * artosyn_remove() for it once the re-enumeration tears down this
	 * identity. */
	g_dev = dev;
	return 0;

err_claim_host_for_cleanup:
	sdio_claim_host(func);
	sdio_release_irq(func);
err_disable_func:
	sdio_disable_func(func);
err_release_host:
	sdio_release_host(func);
	kfree(dev);
	return ret;
}

static void artosyn_remove(struct sdio_func *func)
{
	struct artosyn_dev *dev = sdio_get_drvdata(func);

	if (!dev)
		return;

	mutex_lock(&dev->io_mutex);
	dev->removed = true;
	mutex_unlock(&dev->io_mutex);
	wake_up_interruptible_all(&dev->rx_q);
	wake_up_interruptible_all(&dev->tx_q);
	wake_up_interruptible_all(&dev->mailbox_q);

	/* Give any in-flight read()/write() a moment to notice dev->removed
	 * and return before we free dev out from under them. */
	while (dev->reading || dev->writing)
		msleep(10);

	misc_deregister(&dev->miscdev);

	sdio_claim_host(func);
	sdio_release_irq(func);
	sdio_disable_func(func);
	sdio_release_host(func);

	if (g_dev == dev)
		g_dev = NULL;
	kfree(dev);
}

static const struct sdio_device_id artosyn_ids[] = {
	{ SDIO_DEVICE(ARTO_ROMCODE_VID, ARTO_ROMCODE_PID) },
	{ SDIO_DEVICE(ARTO_RTOS_VID, ARTO_RTOS_PID) },
	{ SDIO_DEVICE(ARTO_RTOS_ALT_VID, ARTO_RTOS_ALT_PID) },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(sdio, artosyn_ids);

static struct sdio_driver artosyn_driver = {
	.name = DRV_NAME,
	.id_table = artosyn_ids,
	.probe = artosyn_probe,
	.remove = artosyn_remove,
};

module_param(fw_name, charp, 0);
MODULE_PARM_DESC(fw_name, "AR8030 firmware image path (request_firmware name)");
module_param(cfg_name, charp, 0);
MODULE_PARM_DESC(cfg_name, "AR8030 baseband config path (request_firmware name)");

static int __init artosyn_drv_init(void)
{
	return sdio_register_driver(&artosyn_driver);
}

static void __exit artosyn_drv_exit(void)
{
	sdio_unregister_driver(&artosyn_driver);
}

module_init(artosyn_drv_init);
module_exit(artosyn_drv_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Clean-room AR8030 SDIO chardev driver (/dev/artosyn_sdio)");
