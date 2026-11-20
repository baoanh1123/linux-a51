// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Samsung Electronics Co., Ltd.
 * Copyright 2020 Google LLC.
 * Copyright 2024 Linaro Ltd.
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/container_of.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/mailbox/exynos-message.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox/exynos-message.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/math.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>

static const struct file_operations acpm_sram_dump_fops;
static const struct file_operations acpm_chans_fops;
static const struct file_operations acpm_plugins_fops;
static const struct file_operations acpm_fvmap_raw_fops;

#include "exynos-acpm.h"
#include "exynos-acpm-pmic.h"

/*
 * The sequence number lives in bits 21:16 of the message's first word, which
 * is exactly what the vendor 9610 driver does before queueing a request
 * (DOWNSTREAM_READONLY/drivers/soc/samsung/acpm/acpm_ipc.c:
 *  cfg->cmd[0] |= (seq & 0x3f) << ACPM_IPC_PROTOCOL_SEQ_NUM).
 *
 * The OWN/RSP/ID/INDIRECTION bits that acpm_ipc_ctrl.h defines are *not* part
 * of a request: the vendor never sets them, and adding them corrupts the
 * payload word - for DVFS the firmware then sees domain id 0xa0000002 instead
 * of 2 and silently ignores the request (no response, no rate change).
 */
#define ACPM_PROTOCOL_SEQNUM		GENMASK(21, 16)

/* Channel memory model (vendor fw_header/framework.h: enum channel_type). */
#define ACPM_CHAN_TYPE_QUEUE		1
#define ACPM_CHAN_TYPE_BUFFER		2

#define ACPM_POLL_TIMEOUT_US		(100 * USEC_PER_MSEC)
#define ACPM_TX_TIMEOUT_US		500000

#define ACPM_GS101_INITDATA_BASE	0xa000

/**
 * struct acpm_shmem - shared memory configuration information.
 * @reserved:	unused fields.
 * @chans:	offset to array of struct acpm_chan_shmem.
 * @reserved1:	unused fields.
 * @num_chans:	number of channels.
 */
struct acpm_shmem {
	u32 reserved[2];
	u32 chans;
	u32 reserved1[3];
	u32 num_chans;
};

/**
 * struct acpm_chan_shmem - descriptor of a shared memory channel.
 *
 * @id:			channel ID.
 * @reserved:		unused fields.
 * @rx_rear:		rear pointer of APM RX queue (TX for AP).
 * @rx_front:		front pointer of APM RX queue (TX for AP).
 * @rx_base:		base address of APM RX queue (TX for AP).
 * @reserved1:		unused fields.
 * @tx_rear:		rear pointer of APM TX queue (RX for AP).
 * @tx_front:		front pointer of APM TX queue (RX for AP).
 * @tx_base:		base address of APM TX queue (RX for AP).
 * @qlen:		queue length. Applies to both TX/RX queues.
 * @mlen:		message length. Applies to both TX/RX queues.
 * @reserved2:		unused fields.
 * @poll_completion:	true when the channel works on polling.
 */
struct acpm_chan_shmem {
	u32 id;
	u32 field;
	u32 owner;
	u32 type;
	u32 rx_rear;
	u32 rx_front;
	u32 rx_base;
	u32 reserved1[3];
	u32 tx_rear;
	u32 tx_front;
	u32 tx_base;
	u32 qlen;
	u32 mlen;
	u32 reserved2[2];
	u32 poll_completion;
};

/**
 * struct acpm_queue - exynos acpm queue.
 *
 * @rear:	rear address of the queue.
 * @front:	front address of the queue.
 * @base:	base address of the queue.
 */
struct acpm_queue {
	void __iomem *rear;
	void __iomem *front;
	void __iomem *base;
};

/**
 * struct acpm_rx_data - RX queue data.
 *
 * @cmd:	pointer to where the data shall be saved.
 * @n_cmd:	number of 32-bit commands.
 * @rxcnt:	expected length of the response in 32-bit words.
 */
struct acpm_rx_data {
	u32 *cmd;
	size_t n_cmd;
	size_t rxcnt;
};

#define ACPM_SEQNUM_MAX    64

/* Interrupt driven channels answer within a few microseconds at most. */
#define ACPM_IRQ_TIMEOUT_MS	100

/**
 * struct acpm_chan - driver internal representation of a channel.
 * @cl:		mailbox client.
 * @chan:	mailbox channel.
 * @acpm:	pointer to driver private data.
 * @tx:		TX queue. The enqueue is done by the host.
 *			- front index is written by the host.
 *			- rear index is written by the firmware.
 *
 * @rx:		RX queue. The enqueue is done by the firmware.
 *			- front index is written by the firmware.
 *			- rear index is written by the host.
 * @tx_lock:	protects TX queue.
 * @rx_lock:	protects RX queue.
 * @qlen:	queue length. Applies to both TX/RX queues.
 * @mlen:	message length. Applies to both TX/RX queues.
 * @seqnum:	sequence number of the last message enqueued on TX queue.
 * @id:		channel ID.
 * @type:		channel memory model: queue or single buffer.
 * @poll_completion:	indicates if the transfer needs to be polled for
 *			completion or interrupt mode is used.
 * @irq_done:	completed when the firmware signals an RX for this channel.
 * @bitmap_seqnum: bitmap that tracks the messages on the TX/RX queues.
 * @rx_data:	internal buffer used to drain the RX queue.
 */
struct acpm_chan {
	struct mbox_client cl;
	struct mbox_chan *chan;
	struct acpm_info *acpm;
	struct acpm_queue tx;
	struct acpm_queue rx;
	struct mutex tx_lock;
	struct mutex rx_lock;

	unsigned int qlen;
	unsigned int mlen;
	unsigned int type;
	/* Raw descriptor offsets, kept so debugfs can bound its reads: some
	 * firmware-only channels store absolute addresses here instead.
	 */
	u32 tx_off_front;
	u32 tx_off_rear;
	u32 rx_off_front;
	u32 rx_off_rear;
	u8 seqnum;
	u8 id;
	bool poll_completion;
	struct completion irq_done;

	DECLARE_BITMAP(bitmap_seqnum, ACPM_SEQNUM_MAX - 1);
	struct acpm_rx_data rx_data[ACPM_SEQNUM_MAX];
};

/**
 * struct acpm_info - driver's private data.
 * @shmem:	pointer to the SRAM configuration data.
 * @sram_base:	base address of SRAM.
 * @chans:	pointer to the ACPM channel parameters retrieved from SRAM.
 * @dev:	pointer to the exynos-acpm device.
 * @handle:	instance of acpm_handle to send to clients.
 * @num_chans:	number of channels available for this controller.
 */
struct acpm_info {
	struct acpm_shmem __iomem *shmem;
	void __iomem *sram_base;
	struct dentry *debugfs;
	struct acpm_chan *chans;
	struct device *dev;
	struct acpm_handle handle;
	size_t sram_size;
	loff_t initdata_base;
	u32 num_chans;
	u32 num_chans_offset;
};

/**
 * struct acpm_match_data - of_device_id data.
 * @initdata_base:	offset in SRAM where the channels configuration resides.
 */
struct acpm_match_data {
	loff_t initdata_base;
	u32 num_chans_offset;
};

static void acpm_rx_callback(struct mbox_client *cl, void *mssg);

#define client_to_acpm_chan(c) container_of(c, struct acpm_chan, cl)
#define handle_to_acpm_info(h) container_of(h, struct acpm_info, handle)

/**
 * acpm_get_saved_rx() - get the response if it was already saved.
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer to get response for.
 * @tx_seqnum:	xfer TX sequence number.
 */
static void acpm_get_saved_rx(struct acpm_chan *achan,
			      const struct acpm_xfer *xfer, u32 tx_seqnum)
{
	const struct acpm_rx_data *rx_data = &achan->rx_data[tx_seqnum - 1];
	u32 rx_seqnum;

	if (!rx_data->rxcnt)
		return;

	rx_seqnum = FIELD_GET(ACPM_PROTOCOL_SEQNUM, rx_data->cmd[0]);

	if (rx_seqnum == tx_seqnum) {
		memcpy(xfer->rxd, rx_data->cmd, xfer->rxlen);
		clear_bit(rx_seqnum - 1, achan->bitmap_seqnum);
	}
}

/**
 * acpm_get_rx() - get response from RX queue.
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer to get response for.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_get_rx(struct acpm_chan *achan, const struct acpm_xfer *xfer)
{
	u32 rx_front, rx_seqnum, tx_seqnum, seqnum;
	const void __iomem *base, *addr;
	struct acpm_rx_data *rx_data;
	u32 i, val, mlen;
	bool rx_set = false;

	guard(mutex)(&achan->rx_lock);

	rx_front = readl(achan->rx.front);
	i = readl(achan->rx.rear);

	tx_seqnum = FIELD_GET(ACPM_PROTOCOL_SEQNUM, xfer->txd[0]);

	if (i == rx_front) {
		acpm_get_saved_rx(achan, xfer, tx_seqnum);
		return 0;
	}

	base = achan->rx.base;
	mlen = achan->mlen;

	/*
	 * A TYPE_BUFFER channel keeps a single message at rx.base and never
	 * moves the front/rear indices, so the queue walk below can never
	 * match a response there.  DVFS (channel 5) is such a channel.
	 */
	if (achan->type == ACPM_CHAN_TYPE_BUFFER) {
		val = readl(base);
		if (FIELD_GET(ACPM_PROTOCOL_SEQNUM, val) == tx_seqnum) {
			__ioread32_copy(xfer->rxd, base, xfer->rxlen / 4);
			clear_bit(tx_seqnum - 1, achan->bitmap_seqnum);
			exynos_mbox_chan_ack(achan->chan, achan->id);
		}

		return 0;
	}

	/* Drain RX queue. */
	do {
		/* Read RX seqnum. */
		addr = base + mlen * i;
		val = readl(addr);

		rx_seqnum = FIELD_GET(ACPM_PROTOCOL_SEQNUM, val);
		if (!rx_seqnum)
			return -EIO;
		/*
		 * mssg seqnum starts with value 1, whereas the driver considers
		 * the first mssg at index 0.
		 */
		seqnum = rx_seqnum - 1;
		rx_data = &achan->rx_data[seqnum];

		if (rx_data->rxcnt) {
			if (rx_seqnum == tx_seqnum) {
				__ioread32_copy(xfer->rxd, addr,
						xfer->rxlen / 4);
				rx_set = true;
				clear_bit(seqnum, achan->bitmap_seqnum);
			} else {
				/*
				 * The RX data corresponds to another request.
				 * Save the data to drain the queue, but don't
				 * clear yet the bitmap. It will be cleared
				 * after the response is copied to the request.
				 */
				__ioread32_copy(rx_data->cmd, addr,
						rx_data->rxcnt);
			}
		} else {
			clear_bit(seqnum, achan->bitmap_seqnum);
		}

		i = (i + 1) % achan->qlen;
	} while (i != rx_front);

	/* We saved all responses, mark RX empty. */
	writel(rx_front, achan->rx.rear);

	/*
	 * Clear the pending bit and return a credit, as the vendor does after
	 * draining, but only when our own response was consumed: acking every
	 * poll of a shared channel raises needless firmware interrupts.
	 */
	if (rx_set)
		exynos_mbox_chan_ack(achan->chan, achan->id);

	/*
	 * If the response was not in this iteration of the queue, check if the
	 * RX data was previously saved.
	 */
	if (!rx_set)
		acpm_get_saved_rx(achan, xfer, tx_seqnum);

	return 0;
}

/**
 * acpm_dequeue_by_polling() - RX dequeue by polling.
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer being waited for.
 *
 * Return: 0 on success, -errno otherwise.
 */
/*
 * Bring-up aid: dump the queue pointers and the raw RX slots of one channel.
 * Gated by the dump_chan parameter (default: channel 5, the DVFS channel) so
 * the PMIC channel does not flood the log.
 */
static unsigned int acpm_dump_chan_id = 5;
module_param_named(dump_chan, acpm_dump_chan_id, uint, 0444);
MODULE_PARM_DESC(dump_chan, "ACPM channel to dump on every transfer");

static void acpm_dump_chan(struct acpm_chan *achan, const char *tag)
{
	struct device *dev = achan->acpm->dev;
	u32 raw[ACPM_SEQNUM_MAX > 8 ? 8 : 8];
	u32 i;

	if (achan->id != acpm_dump_chan_id)
		return;

	dev_info(dev, "%s ch%u qlen=%u mlen=%u tx f/r=%u/%u rx f/r=%u/%u\n",
		 tag, achan->id, achan->qlen, achan->mlen,
		 readl(achan->tx.front), readl(achan->tx.rear),
		 readl(achan->rx.front), readl(achan->rx.rear));

	for (i = 0; i < achan->qlen && i < ARRAY_SIZE(raw); i++) {
		__ioread32_copy(raw, achan->rx.base + achan->mlen * i,
				achan->mlen / 4);
		dev_info(dev, "%s ch%u rxslot%u: %8phN\n", tag, achan->id, i,
			 raw);
		__ioread32_copy(raw, achan->tx.base + achan->mlen * i,
				achan->mlen / 4);
		dev_info(dev, "%s ch%u txslot%u: %8phN\n", tag, achan->id, i,
			 raw);
	}
}

static int acpm_dequeue_by_polling(struct acpm_chan *achan,
				   const struct acpm_xfer *xfer)
{
	struct device *dev = achan->acpm->dev;
	ktime_t timeout;
	u32 seqnum;
	int ret;

	seqnum = FIELD_GET(ACPM_PROTOCOL_SEQNUM, xfer->txd[0]);

	timeout = ktime_add_us(ktime_get(), ACPM_POLL_TIMEOUT_US);
	do {
		ret = acpm_get_rx(achan, xfer);
		if (ret)
			return ret;

		if (!test_bit(seqnum - 1, achan->bitmap_seqnum))
			return 0;

		/* Determined experimentally. */
		udelay(20);
	} while (ktime_before(ktime_get(), timeout));

	dev_err(dev, "Timeout! ch:%u s:%u bitmap:%lx intstat:%08x.\n",
		achan->id, seqnum, achan->bitmap_seqnum[0],
		exynos_mbox_int_status(achan->chan));
	acpm_dump_chan(achan, "timeout");

	return -ETIME;
}

/**
 * acpm_wait_for_queue_slots() - wait for queue slots.
 *
 * @achan:		ACPM channel info.
 * @next_tx_front:	next front index of the TX queue.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_wait_for_queue_slots(struct acpm_chan *achan, u32 next_tx_front)
{
	u32 val, ret;

	/*
	 * Wait for RX front to keep up with TX front. Make sure there's at
	 * least one element between them.
	 */
	ret = readl_poll_timeout(achan->rx.front, val, next_tx_front != val, 0,
				 ACPM_TX_TIMEOUT_US);
	if (ret) {
		dev_err(achan->acpm->dev, "RX front can not keep up with TX front.\n");
		return ret;
	}

	ret = readl_poll_timeout(achan->tx.rear, val, next_tx_front != val, 0,
				 ACPM_TX_TIMEOUT_US);
	if (ret)
		dev_err(achan->acpm->dev, "TX queue is full.\n");

	return ret;
}

/**
 * acpm_prepare_xfer() - prepare a transfer before writing the message to the
 * TX queue.
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer being prepared.
 */
static void acpm_prepare_xfer(struct acpm_chan *achan,
			      const struct acpm_xfer *xfer)
{
	struct acpm_rx_data *rx_data;
	u32 *txd = (u32 *)xfer->txd;

	/* Prevent chan->seqnum from being re-used */
	do {
		if (++achan->seqnum == ACPM_SEQNUM_MAX)
			achan->seqnum = 1;
	} while (test_bit(achan->seqnum - 1, achan->bitmap_seqnum));

	txd[0] |= FIELD_PREP(ACPM_PROTOCOL_SEQNUM, achan->seqnum);

	/* Clear data for upcoming responses */
	rx_data = &achan->rx_data[achan->seqnum - 1];
	memset(rx_data->cmd, 0, sizeof(*rx_data->cmd) * rx_data->n_cmd);
	/* zero means no response expected */
	rx_data->rxcnt = xfer->rxlen / 4;

	/* Flag the index based on seqnum. (seqnum: 1~63, bitmap: 0~62) */
	set_bit(achan->seqnum - 1, achan->bitmap_seqnum);
}

/**
 * acpm_wait_for_message_response - an helper to group all possible ways of
 * waiting for a synchronous message response.
 *
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer being waited for.
 *
 * Return: 0 on success, -errno otherwise.
 */
/**
 * acpm_rx_callback() - mailbox client callback for interrupt driven channels
 * @cl:		mailbox client of the channel
 * @mssg:	unused
 *
 * The mailbox controller only signals that a response arrived; the payload is
 * then read from the shared memory RX queue by the waiter.
 */
static void acpm_rx_callback(struct mbox_client *cl, void *mssg)
{
	struct acpm_chan *achan = container_of(cl, struct acpm_chan, cl);

	complete(&achan->irq_done);
}

static int acpm_wait_for_message_response(struct acpm_chan *achan,
					  const struct acpm_xfer *xfer)
{
	/*
	 * Poll the RX queue for every channel.  Linux on this SoC never sees
	 * the ACPM interrupt (the vendor kernel's mailbox IRQ counter stays at
	 * zero while CPU DVFS runs), so the firmware is expected to be polled
	 * even for channels whose shared-memory descriptor advertises
	 * interrupt completion.
	 */
	return acpm_dequeue_by_polling(achan, xfer);
}

/**
 * acpm_do_xfer() - do one transfer.
 * @handle:	pointer to the acpm handle.
 * @xfer:	transfer to initiate and wait for response.
 *
 * Return: 0 on success, -errno otherwise.
 */
/*
 * Raw message exchange used by firmware-owned subsystems (CPU DVFS, ...) that
 * do not fit the register oriented PMIC model.  The request words are also
 * used as the response buffer, mirroring what the firmware expects.
 */
int acpm_do_cmd(struct acpm_handle *handle, unsigned int acpm_chan_id,
		u32 *cmd, unsigned int words)
{
	struct acpm_xfer xfer = {
		.txd = cmd,
		.rxd = cmd,
		.txlen = words * sizeof(u32),
		.rxlen = words * sizeof(u32),
		.acpm_chan_id = acpm_chan_id,
	};

	return acpm_do_xfer(handle, &xfer);
}
EXPORT_SYMBOL_GPL(acpm_do_cmd);

int acpm_do_xfer(struct acpm_handle *handle, const struct acpm_xfer *xfer)
{
	struct acpm_info *acpm = handle_to_acpm_info(handle);
	struct exynos_mbox_msg msg;
	struct acpm_chan *achan;
	u32 idx, tx_front;
	int ret;

	if (xfer->acpm_chan_id >= acpm->num_chans)
		return -EINVAL;

	achan = &acpm->chans[xfer->acpm_chan_id];

	if (!xfer->txd || xfer->txlen > achan->mlen || xfer->rxlen > achan->mlen)
		return -EINVAL;

	/*
	 * Use the channel id from shared memory, not the array index: the
	 * vendor ACPM driver rings the doorbell as 1 << channel->id, and the id
	 * need not match the position in the channel table.
	 */
	msg.chan_id = achan->id;
	msg.chan_type = EXYNOS_MBOX_CHAN_TYPE_DOORBELL;

	if (!achan->poll_completion)
		reinit_completion(&achan->irq_done);

	scoped_guard(mutex, &achan->tx_lock) {
		tx_front = readl(achan->tx.front);

		/*
		 * A single-slot channel has no wrap-around: the slot is always
		 * index 0 and there is no front/rear gap to maintain, so the
		 * queue-slot wait cannot be satisfied and must be skipped.
		 */
		if (achan->qlen == 1) {
			idx = 0;
		} else {
			idx = (tx_front + 1) % achan->qlen;

			ret = acpm_wait_for_queue_slots(achan, idx);
			if (ret)
				return ret;
		}

		acpm_dump_chan(achan, "pre");
		acpm_prepare_xfer(achan, xfer);

		/* Write TX command. */
		__iowrite32_copy(achan->tx.base + achan->mlen * tx_front,
				 xfer->txd, xfer->txlen / 4);

		/*
		 * Advance TX front.  A TYPE_BUFFER channel holds exactly one
		 * message and q_len is 1, so "front = (front + 1) % q_len"
		 * always leaves front == rear and the firmware, which uses
		 * equality as the empty marker, never sees a pending message.
		 * Publish rear + 1 instead: a raw counter the firmware can
		 * compare, and which is restored when it consumes the slot.
		 */
		if (achan->type == ACPM_CHAN_TYPE_BUFFER &&
		    xfer->front_mode == ACPM_FRONT_VENDOR)
			writel(achan->qlen > 1 ?
			       (readl(achan->tx.front) + 1) % achan->qlen : 0,
			       achan->tx.front);
		else if (achan->type == ACPM_CHAN_TYPE_BUFFER)
			writel(readl(achan->tx.rear) + 1, achan->tx.front);
		else
			writel(idx, achan->tx.front);


		ret = mbox_send_message(achan->chan, (void *)&msg);
		if (ret < 0)
			return ret;

		mbox_client_txdone(achan->chan, 0);
		acpm_dump_chan(achan, "post");
	}

	/*
	 * Fire-and-forget clients - the DVFS ones - are answered by the
	 * firmware acting on the request, not by a response word, so waiting
	 * here only produces a timeout on a message that was delivered.
	 */
	if (xfer->no_wait)
		return 0;

	return acpm_wait_for_message_response(achan, xfer);
}
EXPORT_SYMBOL_GPL(acpm_do_xfer);

/**
 * acpm_chan_shmem_get_params() - get channel parameters and addresses of the
 * TX/RX queues.
 * @achan:	ACPM channel info.
 * @chan_shmem:	__iomem pointer to a channel described in shared memory.
 */
static void acpm_chan_shmem_get_params(struct acpm_chan *achan,
				struct acpm_chan_shmem __iomem *chan_shmem)
{
	void __iomem *base = achan->acpm->sram_base;
	struct acpm_queue *rx = &achan->rx;
	struct acpm_queue *tx = &achan->tx;

	achan->type = readl(&chan_shmem->type);
	achan->mlen = readl(&chan_shmem->mlen);
	achan->poll_completion = readl(&chan_shmem->poll_completion);
	achan->id = readl(&chan_shmem->id);
	achan->qlen = readl(&chan_shmem->qlen);

	achan->tx_off_front = readl(&chan_shmem->rx_front);
	achan->tx_off_rear = readl(&chan_shmem->rx_rear);
	achan->rx_off_front = readl(&chan_shmem->tx_front);
	achan->rx_off_rear = readl(&chan_shmem->tx_rear);

	tx->base = base + readl(&chan_shmem->rx_base);
	tx->rear = base + readl(&chan_shmem->rx_rear);
	tx->front = base + readl(&chan_shmem->rx_front);

	rx->base = base + readl(&chan_shmem->tx_base);
	rx->rear = base + readl(&chan_shmem->tx_rear);
	rx->front = base + readl(&chan_shmem->tx_front);

	dev_vdbg(achan->acpm->dev, "ID = %d poll = %d, mlen = %d, qlen = %d\n",
		 achan->id, achan->poll_completion, achan->mlen, achan->qlen);
}

/**
 * acpm_achan_alloc_cmds() - allocate buffers for retrieving data from the ACPM
 * firmware.
 * @achan:	ACPM channel info.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_achan_alloc_cmds(struct acpm_chan *achan)
{
	struct device *dev = achan->acpm->dev;
	struct acpm_rx_data *rx_data;
	size_t cmd_size, n_cmd;
	int i;

	if (achan->mlen == 0)
		return 0;

	cmd_size = sizeof(*(achan->rx_data[0].cmd));
	n_cmd = DIV_ROUND_UP_ULL(achan->mlen, cmd_size);

	for (i = 0; i < ACPM_SEQNUM_MAX; i++) {
		rx_data = &achan->rx_data[i];
		rx_data->n_cmd = n_cmd;
		rx_data->cmd = devm_kcalloc(dev, n_cmd, cmd_size, GFP_KERNEL);
		if (!rx_data->cmd)
			return -ENOMEM;
	}

	return 0;
}

/**
 * acpm_free_mbox_chans() - free mailbox channels.
 * @data:	pointer to driver data.
 */
static void acpm_free_mbox_chans(void *data)
{
	struct acpm_info *acpm = data;
	int i;

	for (i = 0; i < acpm->num_chans; i++)
		if (!IS_ERR_OR_NULL(acpm->chans[i].chan))
			mbox_free_channel(acpm->chans[i].chan);
}

/**
 * acpm_channels_init() - initialize channels based on the configuration data in
 * the shared memory.
 * @acpm:	pointer to driver data.
 *
 * Return: 0 on success, -errno otherwise.
 */
/*
 * Port diagnostic only: read s2mpu09 registers over ACPM to prove the PMIC
 * channel works. Read-only - no PMIC register is written.
 *
 * The request encoding follows the vendor acpm_mfd.h protocol, which matches
 * mainline's ACPM PMIC fields for this part: cmd[0] = TYPE[15:8] | REG[7:0],
 * cmd[1] = FUNC[7:0] with FUNC_READ = 0, and the data byte is returned in
 * bits [15:8] of cmd[1]. The s2mpu09 MFD uses type 0 for the TOP bank and
 * type 1 for the PMIC bank (DOWNSTREAM_READONLY/drivers/mfd/s2mpu09-core.c:
 * I2C_ADDR_TOP/I2C_ADDR_PMIC), so the rail registers must be read with type 1.
 * ACPM channel 2 is the PMIC channel in the live vendor DT; the register
 * addresses are from the vendor include/linux/mfd/samsung/s2mpu09-regulator.h
 * (LDO2/vqmmc = 0x3a/0x3b, LDO35/vmmc = 0x5c).
 */
#define ACPM_EXYNOS9611_PMIC_CHAN	2

static void acpm_pmic_readout_probe(struct acpm_info *acpm)
{
	/*
	 * PMICID 0x00 identifies the PMIC; B1OUT/B4CTRL/B4OUT are rails the
	 * boot chain must have programmed, so they prove the read path; then
	 * LDO2 (vqmmc, 0x3a/0x3b) and LDO35 (vmmc, 0x5c).
	 */
	static const struct { u8 type; u8 reg; } regs[] = {
		{ 0, 0x00 }, { 0, 0x19 }, { 0, 0x1e }, { 0, 0x1f },
		{ 1, 0x00 }, { 1, 0x3a }, { 1, 0x3b }, { 1, 0x5c },
	};
	unsigned int i;

	/*
	 * The vendor acpm_ipc init programs INTMR1 (the AP->APM doorbell mask)
	 * from the firmware's per-channel ap_poll flag as
	 * mask |= polling << channel id, and never relies on the boot chain
	 * leaving it in that state. Reproduce it here and log the mailbox
	 * registers so the doorbell path can be checked.
	 */
	if (acpm->dev->of_node) {
		struct device_node *mbox_node;
		struct resource res;
		void __iomem *mbox_base;
		u32 mask = 0;

		mbox_node = of_parse_phandle(acpm->dev->of_node, "mboxes", 0);
		if (mbox_node) {
			for (i = 0; i < acpm->num_chans; i++)
				mask |= (acpm->chans[i].poll_completion ? 1 : 0)
					<< acpm->chans[i].id;

			if (!of_address_to_resource(mbox_node, 0, &res)) {
				mbox_base = devm_ioremap(acpm->dev, res.start,
							 resource_size(&res));
				if (mbox_base) {
					/* Exynos9611 compact map: INTMR0 0x10,
					 * INTGR1 0x1c, INTMR1 0x24, INTSR1 0x28.
					 */
					dev_info(acpm->dev,
						 "mailbox before: INTMR0=%#x INTMR1=%#x INTGR1=%#x INTSR1=%#x\n",
						 readl(mbox_base + 0x10),
						 readl(mbox_base + 0x24),
						 readl(mbox_base + 0x1c),
						 readl(mbox_base + 0x28));
					writel(mask, mbox_base + 0x24);
					dev_info(acpm->dev,
						 "mailbox after: INTMR1=%#x mask=%#x\n",
						 readl(mbox_base + 0x24), mask);
				}
			}
			of_node_put(mbox_node);
		}
	}

	for (i = 0; i < acpm->num_chans && i < 16; i++)
		dev_info(acpm->dev,
			 "ACPM ch%u: id=%u qlen=%u mlen=%u poll=%u\n",
			 i, acpm->chans[i].id, acpm->chans[i].qlen,
			 acpm->chans[i].mlen, acpm->chans[i].poll_completion);

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		u8 value = 0;
		int ret;

		ret = acpm_pmic_read_reg(&acpm->handle,
					 ACPM_EXYNOS9611_PMIC_CHAN, regs[i].type,
					 regs[i].reg, 0, &value);

		dev_info(acpm->dev, "PMIC read type=%u reg %#04x: ret=%d value=%#04x\n",
			 regs[i].type, regs[i].reg, ret, value);
	}

	/*
	 * Controlled rail test: the readout above shows LDO2 (vqmmc) and
	 * LDO35 (vmmc) with their enable bits clear. Enable both at 3.3 V
	 * (vsel 0x3c, enable bits 7:6 = 0x3) so the MMC host can identify
	 * the card. This is a test-only write, gated by a DT property.
	 */
	if (of_property_read_bool(acpm->dev->of_node,
				  "samsung,pmic-rail-test")) {
		/*
		 * Match the stock kernel: vqmmc (LDO2) stays at 1.8 V with
		 * enable bits 7:6 set (0xc0), vmmc (LDO35) is 3.3 V with
		 * enable bits set (0xfc). The earlier 0xfc on LDO2 raised
		 * vqmmc to 3.3 V and did not enumerate.
		 */
		static const struct { u8 reg; u8 value; } rails[] = {
			{ 0x3a, 0xc0 }, { 0x5c, 0xfc },
		};
		u8 value;

		for (i = 0; i < ARRAY_SIZE(rails); i++) {
			int ret;

			ret = acpm_pmic_write_reg(&acpm->handle,
						  ACPM_EXYNOS9611_PMIC_CHAN,
						  1, rails[i].reg, 0,
						  rails[i].value);
			value = 0;
			if (!ret)
				ret = acpm_pmic_read_reg(&acpm->handle,
							 ACPM_EXYNOS9611_PMIC_CHAN,
							 1, rails[i].reg, 0, &value);
			dev_info(acpm->dev,
				 "PMIC rail write reg %#04x: ret=%d readback=%#04x\n",
				 rails[i].reg, ret, value);
		}
	}

	if (acpm->dev->of_node) {
		struct device_node *mbox_node;
		struct resource res;
		void __iomem *mbox_base;

		mbox_node = of_parse_phandle(acpm->dev->of_node, "mboxes", 0);
		if (mbox_node) {
			if (!of_address_to_resource(mbox_node, 0, &res)) {
				mbox_base = devm_ioremap(acpm->dev, res.start,
							 resource_size(&res));
				if (mbox_base)
					dev_info(acpm->dev,
						 "mailbox final: INTMR1=%#x INTGR0=%#x INTGR1=%#x INTSR1=%#x INTMSR1=%#x\n",
						 readl(mbox_base + 0x24),
						 readl(mbox_base + 0x08),
						 readl(mbox_base + 0x1c),
						 readl(mbox_base + 0x28),
						 readl(mbox_base + 0x2c));
			}
			of_node_put(mbox_node);
		}
	}
}

static int acpm_channels_init(struct acpm_info *acpm)
{
	struct acpm_shmem __iomem *shmem = acpm->shmem;
	struct acpm_chan_shmem __iomem *chans_shmem;
	struct device *dev = acpm->dev;
	int i, ret;

	acpm->num_chans = readl((void __iomem *)shmem + acpm->num_chans_offset);
	/* Port diagnostic: confirm the boot chain's shared memory is readable. */
	dev_info(dev, "ACPM shared memory: %u channels at offset %#x\n",
		 acpm->num_chans, readl(&shmem->chans));
	acpm->chans = devm_kcalloc(dev, acpm->num_chans, sizeof(*acpm->chans),
				   GFP_KERNEL);
	if (!acpm->chans)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, acpm_free_mbox_chans, acpm);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add mbox free action.\n");

	chans_shmem = acpm->sram_base + readl(&shmem->chans);

	for (i = 0; i < acpm->num_chans; i++) {
		struct acpm_chan_shmem __iomem *chan_shmem = &chans_shmem[i];
		struct acpm_chan *achan = &acpm->chans[i];
		struct mbox_client *cl = &achan->cl;

		achan->acpm = acpm;

		acpm_chan_shmem_get_params(achan, chan_shmem);

		/*
		 * Port diagnostic: raw descriptor, to compare a channel that
		 * works (ch2, PMIC) with one the firmware ignores (ch5, DVFS).
		 */
		dev_info(dev,
			 "ACPM desc ch%u: id=%u txbase=%#x txfront=%#x txrear=%#x rxbase=%#x rxfront=%#x rxrear=%#x qlen=%u mlen=%u poll=%u\n",
			 i, readl(&chan_shmem->id),
			 readl(&chan_shmem->tx_base), readl(&chan_shmem->tx_front),
			 readl(&chan_shmem->tx_rear), readl(&chan_shmem->rx_base),
			 readl(&chan_shmem->rx_front), readl(&chan_shmem->rx_rear),
			 readl(&chan_shmem->qlen), readl(&chan_shmem->mlen),
			 readl(&chan_shmem->poll_completion));

		ret = acpm_achan_alloc_cmds(achan);
		if (ret)
			return ret;

		mutex_init(&achan->rx_lock);
		mutex_init(&achan->tx_lock);
		/*
		 * The waitqueue must be initialised before any RX interrupt can
		 * arrive: acpm_do_xfer() only reinitialises it, and complete()
		 * on a zeroed completion takes a spinlock on a NULL pointer
		 * (SError/Oops in complete() from acpm_rx_callback).
		 */
		init_completion(&achan->irq_done);

		cl->dev = dev;
		cl->rx_callback = acpm_rx_callback;

		/*
		 * One mailbox channel is enough: the doorbell bit comes from
		 * exynos_mbox_msg.chan_id, which acpm_do_xfer() fills with the
		 * ACPM channel id from shared memory.
		 */
		achan->chan = mbox_request_channel(cl, 0);
		if (IS_ERR(achan->chan))
			return PTR_ERR(achan->chan);
	}

	return 0;
}

/**
 * acpm_setup_ops() - setup the operations structures.
 * @acpm:	pointer to the driver data.
 */
static void acpm_setup_ops(struct acpm_info *acpm)
{
	struct acpm_pmic_ops *pmic_ops = &acpm->handle.ops.pmic_ops;

	pmic_ops->read_reg = acpm_pmic_read_reg;
	pmic_ops->bulk_read = acpm_pmic_bulk_read;
	pmic_ops->write_reg = acpm_pmic_write_reg;
	pmic_ops->bulk_write = acpm_pmic_bulk_write;
	pmic_ops->update_reg = acpm_pmic_update_reg;
}

static int acpm_probe(struct platform_device *pdev)
{
	const struct acpm_match_data *match_data;
	struct device *dev = &pdev->dev;
	struct device_node *shmem;
	struct acpm_info *acpm;
	resource_size_t size;
	struct resource res;
	int ret;

	acpm = devm_kzalloc(dev, sizeof(*acpm), GFP_KERNEL);
	if (!acpm)
		return -ENOMEM;

	shmem = of_parse_phandle(dev->of_node, "shmem", 0);
	ret = of_address_to_resource(shmem, 0, &res);
	of_node_put(shmem);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get shared memory.\n");

	size = resource_size(&res);
	acpm->sram_size = size;
	acpm->sram_base = devm_ioremap(dev, res.start, size);
	if (!acpm->sram_base)
		return dev_err_probe(dev, -ENOMEM,
				     "Failed to ioremap shared memory.\n");

	match_data = of_device_get_match_data(dev);
	if (!match_data)
		return dev_err_probe(dev, -EINVAL,
				     "Failed to get match data.\n");

	acpm->initdata_base = match_data->initdata_base;
	acpm->shmem = acpm->sram_base + match_data->initdata_base;
	acpm->num_chans_offset = match_data->num_chans_offset;
	acpm->dev = dev;

	ret = acpm_channels_init(acpm);
	if (ret)
		return ret;

	acpm_setup_ops(acpm);

	if (of_property_read_bool(dev->of_node, "samsung,pmic-readout-probe"))
		acpm_pmic_readout_probe(acpm);

	platform_set_drvdata(pdev, acpm);

	acpm->debugfs = debugfs_create_dir("acpm", NULL);
	debugfs_create_file("sram", 0444, acpm->debugfs, acpm,
			    &acpm_sram_dump_fops);
	debugfs_create_file("chans", 0444, acpm->debugfs, acpm,
			    &acpm_chans_fops);
	debugfs_create_file("plugins", 0444, acpm->debugfs, acpm,
			    &acpm_plugins_fops);
	debugfs_create_file("fvmap_raw", 0444, acpm->debugfs, acpm,
			    &acpm_fvmap_raw_fops);

	return devm_of_platform_populate(dev);
}

/**
 * acpm_handle_put() - release the handle acquired by acpm_get_by_phandle.
 * @handle:	Handle acquired by acpm_get_by_phandle.
 */
static void acpm_handle_put(struct acpm_handle *handle)
{
	struct acpm_info *acpm = handle_to_acpm_info(handle);
	struct device *dev = acpm->dev;

	module_put(dev->driver->owner);
	/* Drop reference taken with of_find_device_by_node(). */
	put_device(dev);
}

/**
 * devm_acpm_release() - devres release method.
 * @dev: pointer to device.
 * @res: pointer to resource.
 */
static void devm_acpm_release(struct device *dev, void *res)
{
	acpm_handle_put(*(struct acpm_handle **)res);
}

/**
 * acpm_get_by_node() - get the ACPM handle using node pointer.
 * @dev:	device pointer requesting ACPM handle.
 * @np:		ACPM device tree node.
 *
 * Return: pointer to handle on success, ERR_PTR(-errno) otherwise.
 *
 * Note: handle CANNOT be pointer to const
 */
static struct acpm_handle *acpm_get_by_node(struct device *dev,
					    struct device_node *np)
{
	struct platform_device *pdev;
	struct device_link *link;
	struct acpm_info *acpm;

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	acpm = platform_get_drvdata(pdev);
	if (!acpm) {
		platform_device_put(pdev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	if (!try_module_get(pdev->dev.driver->owner)) {
		platform_device_put(pdev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	link = device_link_add(dev, &pdev->dev, DL_FLAG_AUTOREMOVE_SUPPLIER);
	if (!link) {
		dev_err(&pdev->dev,
			"Failed to create device link to consumer %s.\n",
			dev_name(dev));
		platform_device_put(pdev);
		module_put(pdev->dev.driver->owner);
		return ERR_PTR(-EINVAL);
	}

	return &acpm->handle;
}

/**
 * devm_acpm_get_by_node() - managed get handle using node pointer.
 * @dev: device pointer requesting ACPM handle.
 * @np:  ACPM device tree node.
 *
 * Return: pointer to handle on success, ERR_PTR(-errno) otherwise.
 */
struct acpm_handle *devm_acpm_get_by_node(struct device *dev,
					  struct device_node *np)
{
	struct acpm_handle **ptr, *handle;

	ptr = devres_alloc(devm_acpm_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	handle = acpm_get_by_node(dev, np);
	if (!IS_ERR(handle)) {
		*ptr = handle;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	return handle;
}
EXPORT_SYMBOL_GPL(devm_acpm_get_by_node);

/**
 * acpm_sram_dump_show() - dump the framework header and channel table.
 *
 * The ACPM firmware publishes its own descriptor in shared memory; the header
 * carries the channel table offset/count and the log ring pointers, so dumping
 * both is the fastest way to compare our view against the firmware's.
 */
static int acpm_sram_dump_show(struct seq_file *s, void *unused)
{
	struct acpm_info *acpm = s->private;
	const struct acpm_info *info = acpm;
	void __iomem *shmem = info->shmem;
	u32 chans_off, nchans, ap_max, i, j;

	seq_puts(s, "framework @initdata-base:\n");
	for (i = 0; i < 0x60; i += 4)
		seq_printf(s, "  +%02x: %08x\n", i, readl(shmem + i));

	chans_off = readl(shmem + 0x08);
	nchans = readl(shmem + 0x0c);
	ap_max = readl(shmem + 0x18);
	seq_printf(s, "ipc_channels=0x%x num_ipc_channels=%u ipc_ap_max=%u\n",
		   chans_off, nchans, ap_max);

	/* Firmware log ring: {char str[8]; u32 val; u32 level} entries. */
	{
		u32 rear_off = readl(shmem + 0x38);
		u32 front_off = readl(shmem + 0x3c);
		u32 data_off = readl(shmem + 0x40);
		u32 esize = readl(shmem + 0x44);
		u32 elen = readl(shmem + 0x48);
		u32 rear, front, n, k, idx;

		seq_printf(s, "\nlog ring: data=0x%x esize=%u elen=%u\n",
			   data_off, esize, elen);
		if (!esize || !elen || esize > 64 || elen > 0x1000) {
			seq_puts(s, "  (not initialised)\n");
			goto channels;
		}

		rear = readl(info->sram_base + rear_off);
		front = readl(info->sram_base + front_off);
		seq_printf(s, "  rear=%u front=%u\n", rear, front);
		n = (front + elen - rear) % elen;
		if (n > 64)
			n = 64;
		for (k = 0; k < n; k++) {
			void __iomem *e;
			u32 hdr, val;
			char tag[9];

			idx = (rear + n - 1 - k) % elen;
			e = info->sram_base + data_off + esize * idx;

			/*
			 * Entry layout (fw_header/fw: acpm_log_print):
			 * +0 header: id[31:28] level[27] index[26:22] tick[15:0]
			 * +4 string, esize - 8 bytes
			 * +esize-4 value
			 */
			hdr = readl(e);
			val = readl(e + esize - 4);
			for (i = 0; i < esize - 8 && i < 8; i++) {
				u8 c = readb(e + 4 + i);

				tag[i] = (c >= 0x20 && c < 0x7f) ? c : '.';
			}
			tag[i] = '\0';
			seq_printf(s, "  [%3u] fw%u lvl%u tick=%u %-8s val=0x%08x\n",
				   idx, (hdr >> 28) & 0xf, (hdr >> 27) & 1,
				   hdr & 0xffff, tag, val);
		}
	}

channels:
	for (i = 0; i < min(nchans, 16U); i++) {
		void __iomem *ch = info->sram_base + chans_off + 0x48 * i;

		seq_printf(s, "ch%u: id=%u field=%u owner=%d type=%u\n", i,
			   readl(ch + 0x00), readl(ch + 0x04),
			   (s32)readl(ch + 0x08), readl(ch + 0x0c));
		seq_puts(s, "   rx: rear/front/base = ");
		for (j = 0; j < 3; j++)
			seq_printf(s, "0x%x ", readl(ch + 0x10 + 4 * j));
		seq_puts(s, "\n   tx: rear/front/base = ");
		for (j = 0; j < 3; j++)
			seq_printf(s, "0x%x ", readl(ch + 0x28 + 4 * j));
		seq_printf(s, "\n   q_len=%u q_elem_size=%u ap_poll=%u\n",
			   readl(ch + 0x34), readl(ch + 0x38),
			   readl(ch + 0x44));
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(acpm_sram_dump);

/*
 * The ACPM firmware is modular: the header starts with the offset of a plugin
 * table and the number of entries.  A plugin whose handler is not built into
 * the framework (DVFS is one) is loaded by the OS at boot - the vendor's
 * plugins_init() reads fw_name, request_firmware()s the blob and copies it into
 * shared memory at base_addr.  Decoding the table is the first step of porting
 * that, because it names the blobs the firmware expects.
 */
#define ACPM_PLUGIN_ENTRY_SIZE	0x30
#define ACPM_PLUGIN_MAX		32
#define ACPM_FW_NAME_MAX	64

/* struct plugin, fw_header/common.h, 32-bit layout */
struct acpm_plugin {
	u32 id;
	u32 base_addr;
	u32 acpm_ops;
	u32 plugin_ops;
	u32 secure_func_mask;
	u32 extern_func_mask;
	u32 timer_period;
	u32 timer_multiplier;
	u8 is_attached;
	u32 size;
	u8 stay_attached;
	u32 fw_name;
};

/*
 * The ACPM firmware is modular: the header starts with the offset of a plugin
 * table and the number of entries.  A plugin whose handler is not built into
 * the framework (DVFS is one) is loaded by the OS at boot - the vendor's
 * plugins_init() reads fw_name, request_firmware()s the blob and copies it into
 * shared memory at base_addr.
 *
 * Only 32-bit reads of the table region are done here.  Byte-wide accesses into
 * ACPM SRAM and reads of descriptors that do not point at real SRAM both faulted
 * the bus (SError) in the first version of this file, so anything not proven
 * reachable - channel buffers of firmware-only channels in particular - is
 * skipped.  The blob names live in the same window and are decoded from the
 * words below rather than by following the fw_name offset.
 */
#define ACPM_PLUGIN_ENTRY_WORDS	12	/* 0x30 bytes per struct plugin */
#define ACPM_PLUGIN_MAX		32
#define ACPM_PLUGIN_DUMP_WORDS	(ACPM_PLUGIN_MAX * ACPM_PLUGIN_ENTRY_WORDS)

static bool acpm_off_readable(struct acpm_info *acpm, u32 off)
{
	return off && off + 4 <= acpm->sram_size;
}

/**
 * acpm_plugin_decode() - print one plugin entry as fields, plus its fw_name
 *
 * The name is stored as a C string in shared memory.  It is read as whole
 * words and the bytes are unpacked here, because byte-wide accesses into ACPM
 * SRAM abort the bus.
 */
static void acpm_plugin_decode(struct seq_file *s, struct acpm_info *acpm,
			       u32 base_off)
{
	u32 id = readl(acpm->sram_base + base_off);
	u32 base = readl(acpm->sram_base + base_off + 4) & ~1U;
	u32 size = readl(acpm->sram_base + base_off + 9 * 4);
	u32 fw = readl(acpm->sram_base + base_off + 11 * 4);
	u32 attached = readl(acpm->sram_base + base_off + 8 * 4) & 0xff;
	u32 stay = readl(acpm->sram_base + base_off + 10 * 4) & 0xff;
	char name[48];

	name[0] = '\0';

	if (acpm_off_readable(acpm, fw)) {
		unsigned int i, c = 0;

		for (i = 0; i + 3 < sizeof(name) && fw + i * 4 + 4 <= acpm->sram_size; i++) {
			u32 v = readl(acpm->sram_base + fw + i * 4);
			unsigned int b;

			for (b = 0; b < 4; b++) {
				char ch = (v >> (8 * b)) & 0xff;

				if (!ch)
					goto done;
				if (ch >= 0x20 && ch < 0x7f)
					name[c++] = ch;
			}
		}
done:
		name[c] = '\0';
	}

	seq_printf(s,
		   "    id=%u base=%#06x size=%#06x attached=%u stay=%u fw_name=\"%s\"\n",
		   id, base, size, attached, stay, name);
}

static int acpm_plugins_show(struct seq_file *s, void *unused)
{
	struct acpm_info *acpm = s->private;
	u32 table = readl(acpm->sram_base + acpm->initdata_base);
	u32 count = readl(acpm->sram_base + acpm->initdata_base + 4);
	unsigned int i, w;

	seq_printf(s, "plugin table @%#x, %u entries\n", table, count);

	if (!table || table >= acpm->sram_size) {
		seq_puts(s, "table offset is outside shared memory\n");
		return 0;
	}
	if (count > ACPM_PLUGIN_MAX)
		count = ACPM_PLUGIN_MAX;

	/*
	 * The DVFS plugin needs an FV map (rate/voltage table) written into its
	 * own SRAM window at fvmap_offset.  The bootloader may or may not have
	 * filled it in - a zeroed header means the kernel is expected to write
	 * it, which is what the vendor's fvmap_init() does from the ASV tables.
	 */
	{
		u32 dvfs_base = 0;
		u32 fvmap_off = 0x6000;

		of_property_read_u32(acpm->dev->of_node, "fvmap_offset",
				     &fvmap_off);

		for (i = 0; i < count; i++) {
			u32 off = table + i * ACPM_PLUGIN_ENTRY_WORDS * 4;
			u32 base = readl(acpm->sram_base + off + 4) & ~1U;

			if (base && readl(acpm->sram_base + off) == 1)
				dvfs_base = base;
		}

		u32 fv = dvfs_base + fvmap_off;

		seq_printf(s, "fvmap @%#06x (dvfs base %#06x + %#x):",
			   fv, dvfs_base, fvmap_off);
		if (acpm_off_readable(acpm, fv)) {
			for (w = 0; w < 6; w++)
				seq_printf(s, " %08x",
					   readl(acpm->sram_base + fv + w * 4));
		} else {
			seq_puts(s, " (outside shared memory)");
			seq_puts(s, "\n");
			goto plugins;
		}
		seq_puts(s, "\n");

		/*
		 * struct fvmap_header: dvfs_type, num_of_lv, num_of_members,
		 * num_of_pll, num_of_mux, num_of_div, gearratio, init_lv,
		 * num_of_gate, reserved[2], block_addr[3], o_members,
		 * o_ratevolt, o_tables.  o_ratevolt points at an array of
		 * struct rate_volt { rate, volt }, i.e. the OPP list the
		 * firmware will accept for this domain.
		 */
		/*
		 * struct fvmap_header: dvfs_type, num_of_lv, num_of_members,
		 * num_of_pll, num_of_mux, num_of_div, gearratio, init_lv,
		 * num_of_gate, reserved[2], block_addr[3], o_members,
		 * o_ratevolt, o_tables.  o_ratevolt points at an array of
		 * struct rate_volt { rate, volt } - the OPP list the firmware
		 * accepts - and o_tables links to the next domain's header.
		 */
		{
			u32 table_off = 0;
			unsigned int t;

			for (t = 0; t < 12; t++) {
				u32 hdr = fv + table_off;
				u32 w0 = readl(acpm->sram_base + hdr);
				u32 w5;
				u32 lv = (w0 >> 8) & 0xff;
				u32 type = w0 & 0xff;
				u32 o_ratevolt, o_tables;
				unsigned int n;

				if (acpm_off_readable(acpm, hdr + 24) == false)
					break;

				w5 = readl(acpm->sram_base + hdr + 5 * 4);
				o_ratevolt = w5 & 0xffff;
				o_tables = (w5 >> 16) & 0xffff;

				seq_printf(s, "  domain type=%u lv=%u o_ratevolt=%#x o_tables=%#x\n",
					   type, lv, o_ratevolt, o_tables);

				if (acpm_off_readable(acpm, hdr + o_ratevolt)) {
					seq_puts(s, "    rate/volt:");
					for (n = 0; n < min(lv, 16U); n++)
						seq_printf(s, " %u/%u",
							   readl(acpm->sram_base +
								 hdr + o_ratevolt + n * 8),
							   readl(acpm->sram_base +
								 hdr + o_ratevolt + n * 8 + 4));
					seq_puts(s, "\n");
				}

				if (!o_tables)
					break;
				table_off = o_tables;
			}
		}
	}

plugins:
	for (i = 0; i < count; i++) {
		u32 base_off = table + i * ACPM_PLUGIN_ENTRY_WORDS * 4;

		if (base_off + ACPM_PLUGIN_ENTRY_WORDS * 4 > acpm->sram_size)
			break;

		/* id, base_addr, acpm_ops, plugin_ops, masks, timer, then
		 * is_attached/size/stay_attached/fw_name - all as raw words.
		 */
		seq_printf(s, "[%u] @%#06x:", i, base_off);
		for (w = 0; w < ACPM_PLUGIN_ENTRY_WORDS; w++)
			seq_printf(s, " %08x",
				   readl(acpm->sram_base + base_off + w * 4));
		seq_puts(s, "\n");
		acpm_plugin_decode(s, acpm, base_off);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(acpm_plugins);

/**
 * acpm_fvmap_raw_show() - dump the DVFS plugin's FV map window as words
 *
 * One fvmap block describes one DVFS domain, and the chain field (o_tables) is
 * a per-level clock table rather than a link to the next domain, so the only
 * way to find the other domains' maps is to read the whole window and look for
 * headers.  Word reads only, bounded by the plugin's own size.
 */
static int acpm_fvmap_raw_show(struct seq_file *s, void *unused)
{
	struct acpm_info *acpm = s->private;
	u32 fvmap_off = 0x6000;
	u32 table = readl(acpm->sram_base + acpm->initdata_base);
	u32 count = readl(acpm->sram_base + acpm->initdata_base + 4);
	u32 dvfs_base = 0, dvfs_size = 0;
	u32 off, end;
	unsigned int i, n = 0;

	of_property_read_u32(acpm->dev->of_node, "fvmap_offset", &fvmap_off);

	if (count > ACPM_PLUGIN_MAX)
		count = ACPM_PLUGIN_MAX;

	for (i = 0; i < count; i++) {
		u32 e = table + i * ACPM_PLUGIN_ENTRY_WORDS * 4;

		if (!acpm_off_readable(acpm, e + 44))
			break;

		if (readl(acpm->sram_base + e) == 1) {
			dvfs_base = readl(acpm->sram_base + e + 4) & ~1U;
			dvfs_size = readl(acpm->sram_base + e + 9 * 4);
		}
	}

	if (!dvfs_base || !dvfs_size) {
		seq_puts(s, "no DVFS plugin\n");
		return 0;
	}

	off = dvfs_base + fvmap_off;
	end = dvfs_base + dvfs_size;
	if (end > acpm->sram_size)
		end = acpm->sram_size;

	seq_printf(s, "# fvmap window %#x..%#x\n", off, end);
	for (; off + 4 <= end; off += 4, n++) {
		if (!(n % 8))
			seq_printf(s, "%06x:", off);
		seq_printf(s, " %08x", readl(acpm->sram_base + off));
		if (n % 8 == 7)
			seq_puts(s, "\n");
	}
	if (n % 8)
		seq_puts(s, "\n");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(acpm_fvmap_raw);

/**
 * acpm_chans_show() - dump the live pointer values of every channel
 *
 * The sram dump shows where the descriptors live; this shows what they hold
 * right now, which is the only way to tell whether the firmware consumed a
 * message (rear catches up with front) or never looked at it.
 */
static int acpm_chans_show(struct seq_file *s, void *unused)
{
	struct acpm_info *acpm = s->private;
	unsigned int i;

	for (i = 0; i < acpm->num_chans; i++) {
		struct acpm_chan *achan = &acpm->chans[i];

		seq_printf(s,
			   "ch%u id=%u type=%u qlen=%u mlen=%u poll=%u\n",
			   i, achan->id, achan->type, achan->qlen,
			   achan->mlen, achan->poll_completion);

		/*
		 * Every read is bounded by the descriptor offsets.  Channels
		 * the firmware keeps to itself carry absolute addresses in
		 * these fields, and dereferencing one aborts the bus, so they
		 * are reported instead of followed.  No slot contents either:
		 * only ch5's buffer is proven reachable.
		 */
		if (acpm_off_readable(acpm, achan->tx_off_front) &&
		    acpm_off_readable(acpm, achan->tx_off_rear))
			seq_printf(s, "  tx front=%u rear=%u\n",
				   readl(achan->tx.front),
				   readl(achan->tx.rear));
		else
			seq_printf(s, "  tx front/rear offsets %#x/%#x (not SRAM)\n",
				   achan->tx_off_front, achan->tx_off_rear);

		if (acpm_off_readable(acpm, achan->rx_off_front) &&
		    acpm_off_readable(acpm, achan->rx_off_rear))
			seq_printf(s, "  rx front=%u rear=%u\n",
				   readl(achan->rx.front),
				   readl(achan->rx.rear));
		else
			seq_printf(s, "  rx front/rear offsets %#x/%#x (not SRAM)\n",
				   achan->rx_off_front, achan->rx_off_rear);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(acpm_chans);

static const struct acpm_match_data acpm_gs101 = {
	.initdata_base = ACPM_GS101_INITDATA_BASE,
	.num_chans_offset = offsetof(struct acpm_shmem, num_chans),
};

/*
 * Exynos9611: the live vendor DT acpm_ipc has initdata-base = <0x6f00>, and the
 * firmware's descriptor is struct acpm_framework (see the vendor
 * drivers/soc/samsung/acpm/fw_header/framework.h):
 *	+0x00 plugins, +0x04 num_plugins, +0x08 ipc_channels,
 *	+0x0c num_ipc_channels, +0x10 pid_framework, +0x14 pid_max,
 *	+0x18 ipc_ap_max ...
 * so the channel table offset is at 0x08 (same as the gs101 layout) but the
 * channel count is at 0x0c, while gs101 keeps it at 0x18.
 */
static const struct acpm_match_data acpm_exynos9611 = {
	.initdata_base = 0x6f00,
	.num_chans_offset = 0x0c,
};

static const struct of_device_id acpm_match[] = {
	{
		.compatible = "google,gs101-acpm-ipc",
		.data = &acpm_gs101,
	},
	{
		.compatible = "samsung,exynos9611-acpm-ipc",
		.data = &acpm_exynos9611,
	},
	{},
};
MODULE_DEVICE_TABLE(of, acpm_match);

static struct platform_driver acpm_driver = {
	.probe	= acpm_probe,
	.driver	= {
		.name = "exynos-acpm-protocol",
		.of_match_table	= acpm_match,
	},
};
module_platform_driver(acpm_driver);

MODULE_AUTHOR("Tudor Ambarus <tudor.ambarus@linaro.org>");
MODULE_DESCRIPTION("Samsung Exynos ACPM mailbox protocol driver");
MODULE_LICENSE("GPL");
