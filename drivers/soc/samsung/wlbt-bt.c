// SPDX-License-Identifier: GPL-2.0
/*
 * Bluetooth transport for the Samsung SCSC/WLBT block.
 *
 * The peer does not expose Bluetooth as a UART or as HCI over HIP4.  It is an
 * SCSC service (SCSC_SERVICE_ID_BT) with its own shared-memory protocol,
 * BSMHCP, described by a BHCS block handed to it in the service start request.
 * The service management channel already used for WLAN carries the request, so
 * this file only has to lay out BHCS and BSMHCP in MIF DRAM, start the service,
 * and move HCI packets between the BSMHCP rings and the kernel HCI core.
 *
 * The vendor driver streams H4 through a character device and reassembles
 * fragments there.  The rings already contain whole HCI commands and events,
 * and whole ACL packets, so the bridge here is a direct one.
 */
#include <linux/bitmap.h>
#include <linux/firmware.h>
#include <linux/skbuff.h>
#include "exynos9610-wlbt.h"

#if IS_ENABLED(CONFIG_BT)
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#define BSMHCP_ACL_BUFFER_SIZE		1024
#define BSMHCP_CMD_EVT_BUFFER_SIZE	258
#define BSMHCP_ACL_TX_BUFFERS		30
#define BSMHCP_RING_CMD		8
#define BSMHCP_RING_EVT		32
#define BSMHCP_RING_ACL		32
#define BSMHCP_RING_AVDTP	16
#define BSMHCP_RING_IQ		8
#define BSMHCP_ALIGNMENT		32
#define BSMHCP_FEATURE_M4_INTERRUPTS	0x00000002
#define BSMHCP_EVENT_TYPE_NONE		0x00
#define BSMHCP_ACL_L2CAP_FLAG_END	0x01
#define BSMHCP_MAX_HANDLE		0x1000

#define BSMHCP_SERVICE_ID_BT		2
#define BSMHCP_SM_MSG_START_REQ		0
#define BSMHCP_SM_MSG_START_CFM		1
#define BSMHCP_SM_MSG_STOP_REQ		2
#define BSMHCP_SM_MSG_STOP_CFM		3
#define MMTRANS_CHAN_ID_SERVICE_MANAGEMENT	1

struct bsmhcp_header {
	u32 magic_value;			/* 0x00 */
	u16 ap_to_fg_m4_int_src;		/* 0x04 */
	u8 service_request;			/* 0x06 */
	u8 reserved1;				/* 0x07 */
	u32 acl_buffer_size;			/* 0x08 */
	u32 cmd_evt_buffer_size;		/* 0x0c */
	u32 acl_tx_buffers;			/* 0x10 */
	u16 ap_to_bg_int_src;			/* 0x14 */
	u16 ap_to_fg_int_src;			/* 0x16 */
	u16 bg_to_ap_int_src;			/* 0x18 */
	u16 fg_to_ap_int_src;			/* 0x1a */
	u32 mailbox_offset;			/* 0x1c */
	u32 reserved1_u32;			/* 0x20 */
	u32 mailbox_hci_cmd_write;		/* 0x24 */
	u32 mailbox_hci_evt_read;		/* 0x28 */
	u32 mailbox_acl_tx_write;		/* 0x2c */
	u32 mailbox_acl_free_read;		/* 0x30 */
	u32 mailbox_acl_rx_read;		/* 0x34 */
	u32 abox_offset;			/* 0x38 */
	u32 abox_length;			/* 0x3c */
	u16 panic_deathbed_confession;		/* 0x40 */
	u16 panic_diatribe_value;		/* 0x42 */
	u32 mailbox_hci_cmd_read;		/* 0x44 */
	u32 mailbox_hci_evt_write;		/* 0x48 */
	u32 controller_flags;			/* 0x4c */
	u32 firmware_features;			/* 0x50 */
	u16 reserved_u16;			/* 0x54 */
	u8 service_state;			/* 0x56 */
	u8 reserved_u8;				/* 0x57 */
	u32 reserved4_u32;			/* 0x58 */
	u32 mailbox_avdtp_read;			/* 0x5c */
	u32 mailbox_acl_tx_read;			/* 0x60 */
	u32 mailbox_acl_free_write;		/* 0x64 */
	u32 mailbox_acl_rx_write;			/* 0x68 */
	u32 mailbox_timing_write;		/* 0x6c */
	u32 mailbox_iq_report_write;		/* 0x70 */
	u8 reserved4[0x0c];			/* 0x74 */
	u32 reserved6_u32;			/* 0x80 */
	u8 reserved5[0x1c];			/* 0x84 */
	u32 mailbox_timing_read;			/* 0xa0 */
	u32 mailbox_avdtp_write;			/* 0xa4 */
	u32 mailbox_iq_report_read;		/* 0xa8 */
	u32 reserved9_u32;			/* 0xac */
	u32 reserved10_u32;			/* 0xb0 */
	u32 reserved11_u32;			/* 0xb4 */
	u32 reserved12_u32;			/* 0xb8 */
	u16 info_ap_to_bg_int_src;		/* 0xbc */
	u16 info_bg_to_ap_int_src;		/* 0xbe */
	u32 btlog_enables0_low;		/* 0xc0 */
	u32 firmware_control;			/* 0xc4 */
	u32 reserved13_u32;			/* 0xc8 */
	u32 btlog_enables0_high;		/* 0xcc */
	u32 btlog_enables1_low;			/* 0xd0 */
	u32 btlog_enables1_high;		/* 0xd4 */
	u8 reserved6[0x14];			/* 0xd8 */
	u32 smm_debug_read;			/* 0xec */
	u32 smm_debug_write;			/* 0xf0 */
	u32 smm_exception;			/* 0xf4 */
	u32 avdtp_detect_stream_id;		/* 0xf8 */
	u32 smm_int_ready;			/* 0xfc */
};

struct bsmhcp_td_control {
	u16 length;
	u8 data[BSMHCP_CMD_EVT_BUFFER_SIZE];
};

struct bsmhcp_td_hci_evt {
	u16 length;
	u16 hci_connection_handle;
	u16 event_type;
	u8 data[BSMHCP_CMD_EVT_BUFFER_SIZE];
};

struct bsmhcp_td_acl_rx {
	u16 hci_connection_handle;
	u16 length;
	u8 broadcast_flag;
	u8 packet_boundary;
	u8 disconnected;
	u8 reserved;
	u8 data[BSMHCP_ACL_BUFFER_SIZE];
};

struct bsmhcp_td_acl_tx_data {
	u16 length;
	u8 buffer_index;
	u8 flags;
	u16 hci_connection_handle;
	u16 l2cap_cid;
};

struct bsmhcp_td_acl_tx_free {
	u8 buffer_index;
	u8 reserved;
	u16 hci_connection_handle;
};

struct bsmhcp_td_avdtp {
	u32 flags;
	u16 l2cap_cid;
	u16 hci_connection_handle;
	u32 reserved;
};

struct bsmhcp_timing_packet {
	u16 source;
	u16 sequence_number;
	u32 interrupt_enter;
	u32 critical_section_enter;
	u32 time[4];
	u32 critical_section_leave;
	u32 interrupt_leave;
};

struct bsmhcp_fw_info {
	u32 r4_from_ap_interrupt_count;
	u32 m4_from_ap_interrupt_count;
	u32 r4_to_ap_interrupt_count;
	u32 m4_to_ap_interrupt_count;
	u32 bt_deep_sleep_time_total;
	u32 bt_deep_sleep_wakeup_duration;
	u32 sched_n_messages;
	u32 user_defined_count;
	u32 user_defined[96];
};

struct bsmhcp_td_iq {
	u8 subevent_code;
	u8 packet_status;
	u16 connection_handle;
	u16 sync_handle;
	u8 rx_phy;
	u8 channel_index;
	s16 rssi;
	u8 rssi_antenna_id;
	u8 cte_type;
	u8 slot_durations;
	u8 sample_count;
	u16 event_count;
	u8 data[164];
};

struct bsmhcp_protocol {
	struct bsmhcp_header header;			/* 0x00000 */
	struct bsmhcp_td_control hci_cmd_transfer_ring[BSMHCP_RING_CMD];
	struct bsmhcp_td_avdtp avdtp_transfer_ring[BSMHCP_RING_AVDTP];
	u8 to_air_reserved[0x1fc0];
	struct bsmhcp_td_acl_tx_data acl_tx_data_transfer_ring[BSMHCP_RING_ACL];
	u8 acl_tx_buffer[BSMHCP_ACL_TX_BUFFERS][BSMHCP_ACL_BUFFER_SIZE];
	u8 reserved[0x20];
	struct bsmhcp_td_hci_evt hci_evt_transfer_ring[BSMHCP_RING_EVT];
	struct bsmhcp_timing_packet timing_transfer_ring[64];
	struct bsmhcp_fw_info information;
	u8 from_air_reserved[0x11e0 - sizeof(struct bsmhcp_fw_info)];
	struct bsmhcp_td_iq iq_reporting_transfer_ring[BSMHCP_RING_IQ];
	struct bsmhcp_td_acl_rx acl_rx_transfer_ring[BSMHCP_RING_ACL];
	struct bsmhcp_td_acl_tx_free acl_tx_free_transfer_ring[BSMHCP_RING_ACL];
};

struct bhcs {
	u32 version;
	u32 bsmhcp_protocol_offset;
	u32 bsmhcp_protocol_length;
	u32 configuration_offset;
	u32 configuration_length;
	u32 bluetooth_address_lap;
	u8 bluetooth_address_uap;
	u16 bluetooth_address_nap;
};

struct wlbt_bt {
	struct exynos_wlbt *w;
	struct hci_dev *hdev;
	struct dentry *debugfs;
	struct mutex lock;		/* service start/stop */
	spinlock_t ring_lock;		/* ring indices and tx buffers */
	struct completion start_cfm;
	struct completion stop_cfm;
	struct work_struct rx_work;
	u32 bhcs_ref;
	u32 bsmhcp_ref;
	u32 config_ref;
	u32 config_len;
	u64 addr;
	bool started;
	unsigned long tx_used[BITS_TO_LONGS(BSMHCP_ACL_TX_BUFFERS)];
	u16 l2cap_remaining[BSMHCP_MAX_HANDLE];
	u16 l2cap_cid[BSMHCP_MAX_HANDLE];
};

/* MIF-relative offsets of the pieces of BSMHCP the bridge touches. */
#define BT_OFF(field)		offsetof(struct bsmhcp_protocol, field)
#define BT_HDR_OFF(field)	offsetof(struct bsmhcp_protocol, header.field)

static_assert(sizeof(struct bsmhcp_header) == 0x100);
static_assert(BT_OFF(hci_cmd_transfer_ring) == 0x100);
static_assert(BT_OFF(avdtp_transfer_ring) == 0x920);
static_assert(BT_OFF(acl_tx_data_transfer_ring) == 0x29a0);
static_assert(BT_OFF(acl_tx_buffer) == 0x2aa0);
static_assert(BT_OFF(hci_evt_transfer_ring) == 0xa2c0);
static_assert(BT_OFF(timing_transfer_ring) == 0xc3c0);
static_assert(BT_OFF(information) == 0xccc0);
static_assert(BT_OFF(iq_reporting_transfer_ring) == 0xdea0);
static_assert(BT_OFF(acl_rx_transfer_ring) == 0xe440);
static_assert(BT_OFF(acl_tx_free_transfer_ring) == 0x16540);
static_assert(sizeof(struct bsmhcp_protocol) == 0x165c0);

#define BSMHCP_MAGIC \
	((BSMHCP_ACL_BUFFER_SIZE | BSMHCP_CMD_EVT_BUFFER_SIZE | \
	  (BT_OFF(acl_tx_buffer) << 15)) ^ sizeof(struct bsmhcp_protocol))

#define BSMHCP_DEFAULT_BT_ADDR	0x001122334455ULL

/* Zero means: read the address from bt_addr_file, then fall back. */
static u64 bt_addr;
module_param(bt_addr, ullong, 0644);
MODULE_PARM_DESC(bt_addr, "Bluetooth address override (0 reads bt_addr_file)");

static char bt_addr_file[64] = "mx140/bt_addr";
module_param_string(bt_addr_file, bt_addr_file, sizeof(bt_addr_file), 0444);
MODULE_PARM_DESC(bt_addr_file, "firmware file with the Bluetooth address");

static int wlbt_bt_start(struct wlbt_bt *bt);
static void wlbt_bt_stop(struct wlbt_bt *bt);

static void *bt_mif(struct wlbt_bt *bt)
{
	return bt->w->dram;
}

static u32 bt_read(struct wlbt_bt *bt, u32 off)
{
	return readl(bt_mif(bt) + bt->bsmhcp_ref + off);
}

static void bt_write(struct wlbt_bt *bt, u32 off, u32 val)
{
	writel(val, bt_mif(bt) + bt->bsmhcp_ref + off);
}

static void bt_doorbell(struct wlbt_bt *bt, u32 bit)
{
	writel(BIT(bit), bt->w->mbox + MBOX_INTGR1);
}

static void bt_unmask(struct wlbt_bt *bt)
{
	u32 val = readl(bt->w->mbox + MBOX_INTMR0);

	val &= ~((BIT(WLBT_BIT_BT_BG_TO_HOST) | BIT(WLBT_BIT_BT_FG_TO_HOST)) << 16);
	writel(val, bt->w->mbox + MBOX_INTMR0);
}

static void bt_mask(struct wlbt_bt *bt)
{
	u32 val = readl(bt->w->mbox + MBOX_INTMR0);

	val |= (BIT(WLBT_BIT_BT_BG_TO_HOST) | BIT(WLBT_BIT_BT_FG_TO_HOST)) << 16;
	writel(val, bt->w->mbox + MBOX_INTMR0);
}

/*
 * The peer does not send HCI Number Of Completed Packets events.  It returns
 * spent ACL buffers through the BSMHCP free ring instead, and the vendor stack
 * turns one returned buffer into one credit in a synthetic event.  Without
 * this the HCI core's acl_pkts count only ever falls, and the link stalls after
 * the first 30 packets.
 */
static struct sk_buff *bt_acl_reclaim(struct wlbt_bt *bt)
{
	struct sk_buff *skb;
	u16 handles[BSMHCP_RING_ACL];
	u16 counts[BSMHCP_RING_ACL];
	u32 rd = bt_read(bt, BT_HDR_OFF(mailbox_acl_free_read));
	u32 wr = bt_read(bt, BT_HDR_OFF(mailbox_acl_free_write));
	u32 first = rd;
	int n = 0, i;

	if (rd >= BSMHCP_RING_ACL || wr >= BSMHCP_RING_ACL) {
		dev_warn_ratelimited(bt->w->dev,
			"Bluetooth ACL free ring index out of range (%u/%u)\n",
			rd, wr);
		return NULL;
	}

	while (rd != wr) {
		void __iomem *td = bt_mif(bt) + bt->bsmhcp_ref +
			BT_OFF(acl_tx_free_transfer_ring) +
			rd * sizeof(struct bsmhcp_td_acl_tx_free);
		u16 handle = readw(td + offsetof(struct bsmhcp_td_acl_tx_free,
						 hci_connection_handle));

		for (i = 0; i < n; i++)
			if (handles[i] == handle)
				break;
		if (i == n) {
			if (n == BSMHCP_RING_ACL)
				break;
			handles[n] = handle;
			counts[n] = 1;
			n++;
		} else {
			counts[i]++;
		}
		rd = (rd + 1) % BSMHCP_RING_ACL;
	}

	if (!n)
		return NULL;

	skb = alloc_skb(3 + 4 * n, GFP_ATOMIC);
	if (!skb)
		return NULL;

	hci_skb_pkt_type(skb) = HCI_EVENT_PKT;
	skb_put_u8(skb, HCI_EV_NUM_COMP_PKTS);
	skb_put_u8(skb, 1 + 4 * n);
	skb_put_u8(skb, n);
	for (i = 0; i < n; i++) {
		put_unaligned_le16(handles[i], skb_put(skb, 2));
		put_unaligned_le16(counts[i], skb_put(skb, 2));
	}

	rd = first;
	while (rd != wr) {
		void __iomem *td = bt_mif(bt) + bt->bsmhcp_ref +
			BT_OFF(acl_tx_free_transfer_ring) +
			rd * sizeof(struct bsmhcp_td_acl_tx_free);
		u8 idx = readb(td + offsetof(struct bsmhcp_td_acl_tx_free,
					      buffer_index));

		if (idx < BSMHCP_ACL_TX_BUFFERS)
			clear_bit(idx, bt->tx_used);
		rd = (rd + 1) % BSMHCP_RING_ACL;
	}
	/* Do not let the peer reuse an entry until its contents have been read. */
	rmb();
	bt_write(bt, BT_HDR_OFF(mailbox_acl_free_read), wr);

	return skb;
}

static int bt_send_cmd(struct wlbt_bt *bt, struct sk_buff *skb)
{
	u32 rd = bt_read(bt, BT_HDR_OFF(mailbox_hci_cmd_read));
	u32 wr = bt_read(bt, BT_HDR_OFF(mailbox_hci_cmd_write));
	void __iomem *td;

	if (skb->len > BSMHCP_CMD_EVT_BUFFER_SIZE)
		return -EMSGSIZE;
	if (rd >= BSMHCP_RING_CMD || wr >= BSMHCP_RING_CMD) {
		dev_err_ratelimited(bt->w->dev,
			"Bluetooth command ring index out of range (%u/%u)\n",
			rd, wr);
		return -EIO;
	}
	if ((wr + 1) % BSMHCP_RING_CMD == rd)
		return -ENOSPC;

	td = bt_mif(bt) + bt->bsmhcp_ref + BT_OFF(hci_cmd_transfer_ring) +
	     wr * sizeof(struct bsmhcp_td_control);
	writew(skb->len, td + offsetof(struct bsmhcp_td_control, length));
	memcpy_toio(td + offsetof(struct bsmhcp_td_control, data), skb->data, skb->len);
	wmb();
	bt_write(bt, BT_HDR_OFF(mailbox_hci_cmd_write), (wr + 1) % BSMHCP_RING_CMD);
	wmb();
	bt_doorbell(bt, WLBT_BIT_BT_AP_TO_BG);

	return 0;
}

static int bt_send_acl(struct wlbt_bt *bt, struct sk_buff *skb,
		       struct sk_buff **credit)
{
	u32 rd = bt_read(bt, BT_HDR_OFF(mailbox_acl_tx_read));
	u32 wr = bt_read(bt, BT_HDR_OFF(mailbox_acl_tx_write));
	u16 handle_flags = get_unaligned_le16(skb->data);
	u16 len = get_unaligned_le16(skb->data + 2);
	u16 handle = handle_flags & 0x0fff;
	u8 pb = (handle_flags >> 12) & 0x03;
	u16 cid = 0;
	u8 flags;
	int idx;
	void __iomem *td;
	void __iomem *buf;

	*credit = NULL;

	if (skb->len < 4 || skb->len != len + 4)
		return -EINVAL;
	if (len > BSMHCP_ACL_BUFFER_SIZE)
		return -EMSGSIZE;
	if (pb == 3)
		return -EINVAL;

	*credit = bt_acl_reclaim(bt);
	if (rd >= BSMHCP_RING_ACL || wr >= BSMHCP_RING_ACL) {
		dev_err_ratelimited(bt->w->dev,
			"Bluetooth ACL TX ring index out of range (%u/%u)\n",
			rd, wr);
		return -EIO;
	}
	if ((wr + 1) % BSMHCP_RING_ACL == rd)
		return -ENOSPC;

	idx = find_first_zero_bit(bt->tx_used, BSMHCP_ACL_TX_BUFFERS);
	if (idx >= BSMHCP_ACL_TX_BUFFERS)
		return -ENOSPC;

	flags = (handle_flags >> 8) & 0xf0;
	if (pb == 0 || pb == 2) {
		u16 l2cap_len = 0;

		if (len >= 4) {
			l2cap_len = get_unaligned_le16(skb->data + 4);
			cid = get_unaligned_le16(skb->data + 6);
			bt->l2cap_cid[handle] = cid;
		}
		if (l2cap_len <= len - 4) {
			flags |= BSMHCP_ACL_L2CAP_FLAG_END;
			bt->l2cap_remaining[handle] = 0;
		} else {
			bt->l2cap_remaining[handle] = l2cap_len - (len - 4);
		}
	} else {
		cid = bt->l2cap_cid[handle];
		if (bt->l2cap_remaining[handle] <= len) {
			flags |= BSMHCP_ACL_L2CAP_FLAG_END;
			bt->l2cap_remaining[handle] = 0;
		} else {
			bt->l2cap_remaining[handle] -= len;
		}
	}

	td = bt_mif(bt) + bt->bsmhcp_ref + BT_OFF(acl_tx_data_transfer_ring) +
	     wr * sizeof(struct bsmhcp_td_acl_tx_data);
	writew(len, td + offsetof(struct bsmhcp_td_acl_tx_data, length));
	writeb(idx, td + offsetof(struct bsmhcp_td_acl_tx_data, buffer_index));
	writeb(flags, td + offsetof(struct bsmhcp_td_acl_tx_data, flags));
	writew(handle, td + offsetof(struct bsmhcp_td_acl_tx_data, hci_connection_handle));
	writew(cid, td + offsetof(struct bsmhcp_td_acl_tx_data, l2cap_cid));
	buf = bt_mif(bt) + bt->bsmhcp_ref + BT_OFF(acl_tx_buffer) +
	      idx * BSMHCP_ACL_BUFFER_SIZE;
	memcpy_toio(buf, skb->data + 4, len);
	set_bit(idx, bt->tx_used);
	wmb();
	bt_write(bt, BT_HDR_OFF(mailbox_acl_tx_write), (wr + 1) % BSMHCP_RING_ACL);
	wmb();
	if (bt_read(bt, BT_HDR_OFF(firmware_features)) & BSMHCP_FEATURE_M4_INTERRUPTS)
		bt_doorbell(bt, WLBT_BIT_BT_AP_TO_FG_M4);
	else
		bt_doorbell(bt, WLBT_BIT_BT_AP_TO_FG);

	return 0;
}

static int wlbt_bt_hci_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct wlbt_bt *bt = hci_get_drvdata(hdev);
	struct sk_buff *credit = NULL;
	unsigned long flags;
	int err;

	spin_lock_irqsave(&bt->ring_lock, flags);
	if (!bt->started) {
		err = -ENODEV;
	} else if (hci_skb_pkt_type(skb) == HCI_COMMAND_PKT) {
		err = bt_send_cmd(bt, skb);
	} else if (hci_skb_pkt_type(skb) == HCI_ACLDATA_PKT) {
		err = bt_send_acl(bt, skb, &credit);
	} else {
		err = -EOPNOTSUPP;
	}
	spin_unlock_irqrestore(&bt->ring_lock, flags);

	if (credit)
		hci_recv_frame(hdev, credit);

	return err;
}

static void wlbt_bt_rx_work(struct work_struct *work)
{
	struct wlbt_bt *bt = container_of(work, struct wlbt_bt, rx_work);
	struct sk_buff *skb, *credit;
	unsigned long flags;

	for (;;) {
		/* Returned ACL buffers turn into completed-packet credits. */
		spin_lock_irqsave(&bt->ring_lock, flags);
		credit = bt->started ? bt_acl_reclaim(bt) : NULL;
		spin_unlock_irqrestore(&bt->ring_lock, flags);
		if (credit) {
			hci_recv_frame(bt->hdev, credit);
			continue;
		}

		skb = NULL;
		spin_lock_irqsave(&bt->ring_lock, flags);
		if (bt->started) {
			u32 rd = bt_read(bt, BT_HDR_OFF(mailbox_hci_evt_read));
			u32 wr = bt_read(bt, BT_HDR_OFF(mailbox_hci_evt_write));

			if (rd >= BSMHCP_RING_EVT || wr >= BSMHCP_RING_EVT) {
				dev_warn_ratelimited(bt->w->dev,
					"Bluetooth event ring index out of range (%u/%u)\n",
					rd, wr);
			} else if (rd != wr) {
				void __iomem *td = bt_mif(bt) + bt->bsmhcp_ref +
					BT_OFF(hci_evt_transfer_ring) +
					rd * sizeof(struct bsmhcp_td_hci_evt);
				u16 len = readw(td + offsetof(struct bsmhcp_td_hci_evt, length));
				u8 plen = readb(td + offsetof(struct bsmhcp_td_hci_evt, data) + 1);

				/* The same check the vendor stack makes: one HCI
				 * event header plus its parameter length.
				 */
				if (len < 2 || len > BSMHCP_CMD_EVT_BUFFER_SIZE ||
				    plen + 2 != len) {
					dev_warn_ratelimited(bt->w->dev,
						"Bluetooth event with bad length %u (plen %u)\n",
						len, plen);
				} else {
					skb = alloc_skb(len, GFP_ATOMIC);
					if (skb) {
						hci_skb_pkt_type(skb) = HCI_EVENT_PKT;
						memcpy_fromio(skb_put(skb, len),
							td + offsetof(struct bsmhcp_td_hci_evt, data), len);
					}
				}
				/* The copy above must finish before the peer
				 * may overwrite the descriptor.
				 */
				rmb();
				bt_write(bt, BT_HDR_OFF(mailbox_hci_evt_read),
					 (rd + 1) % BSMHCP_RING_EVT);
			}
		}
		spin_unlock_irqrestore(&bt->ring_lock, flags);
		if (skb) {
			hci_recv_frame(bt->hdev, skb);
			continue;
		}

		skb = NULL;
		spin_lock_irqsave(&bt->ring_lock, flags);
		if (bt->started) {
			u32 rd = bt_read(bt, BT_HDR_OFF(mailbox_acl_rx_read));
			u32 wr = bt_read(bt, BT_HDR_OFF(mailbox_acl_rx_write));

			if (rd >= BSMHCP_RING_ACL || wr >= BSMHCP_RING_ACL) {
				dev_warn_ratelimited(bt->w->dev,
					"Bluetooth ACL RX ring index out of range (%u/%u)\n",
					rd, wr);
			} else if (rd != wr) {
				void __iomem *td = bt_mif(bt) + bt->bsmhcp_ref +
					BT_OFF(acl_rx_transfer_ring) +
					rd * sizeof(struct bsmhcp_td_acl_rx);
				u16 len = readw(td + offsetof(struct bsmhcp_td_acl_rx, length));
				u16 handle = readw(td + offsetof(struct bsmhcp_td_acl_rx, hci_connection_handle));
				u8 pb = readb(td + offsetof(struct bsmhcp_td_acl_rx, packet_boundary));
				u8 bc = readb(td + offsetof(struct bsmhcp_td_acl_rx, broadcast_flag));

				if (len > 0 && len <= BSMHCP_ACL_BUFFER_SIZE) {
					skb = alloc_skb(4 + len, GFP_ATOMIC);
					if (skb) {
						u8 *hdr;

						hci_skb_pkt_type(skb) = HCI_ACLDATA_PKT;
						hdr = skb_put(skb, 4);
						put_unaligned_le16((handle & 0x0fff) |
								   ((pb & 3) << 12) |
								   ((bc & 3) << 14), hdr);
						put_unaligned_le16(len, hdr + 2);
						memcpy_fromio(skb_put(skb, len),
							td + offsetof(struct bsmhcp_td_acl_rx, data), len);
					}
				}
				rmb();
				bt_write(bt, BT_HDR_OFF(mailbox_acl_rx_read),
					 (rd + 1) % BSMHCP_RING_ACL);
			}
		}
		spin_unlock_irqrestore(&bt->ring_lock, flags);
		if (!skb)
			break;
		hci_recv_frame(bt->hdev, skb);
	}
}

static int bt_addr_from_text(const u8 *buf, size_t len, u64 *addr)
{
	u8 nibble[12];
	int n = 0;
	size_t i;

	for (i = 0; i < len; i++) {
		u8 c = buf[i];
		int v;

		if (c == ':' || c == '-' || c == ' ' || c == '\n' ||
		    c == '\r' || c == '\t')
			continue;
		if (c >= '0' && c <= '9')
			v = c - '0';
		else if (c >= 'a' && c <= 'f')
			v = c - 'a' + 10;
		else if (c >= 'A' && c <= 'F')
			v = c - 'A' + 10;
		else
			return -EINVAL;
		if (n == ARRAY_SIZE(nibble))
			return -EINVAL;
		nibble[n++] = v;
	}

	if (n != ARRAY_SIZE(nibble))
		return -EINVAL;

	*addr = 0;
	for (i = 0; i < ARRAY_SIZE(nibble); i++)
		*addr = (*addr << 4) | nibble[i];

	return 0;
}

static void wlbt_bt_load_addr(struct wlbt_bt *bt)
{
	struct exynos_wlbt *w = bt->w;
	const struct firmware *fw;
	u64 addr;

	if (bt_addr) {
		bt->addr = bt_addr;
		return;
	}

	if (bt_addr_file[0] &&
	    !request_firmware(&fw, bt_addr_file, w->dev)) {
		if (!bt_addr_from_text(fw->data, fw->size, &addr)) {
			bt->addr = addr;
			dev_info(w->dev, "Bluetooth address %012llx from %s\n",
				 bt->addr, bt_addr_file);
		} else {
			dev_warn(w->dev, "invalid Bluetooth address in %s\n",
				 bt_addr_file);
		}
		release_firmware(fw);
	}

	if (!bt->addr) {
		bt->addr = BSMHCP_DEFAULT_BT_ADDR;
		dev_warn(w->dev, "Bluetooth address file unavailable, using %012llx\n",
			 bt->addr);
	}
}

static int wlbt_bt_start(struct wlbt_bt *bt)
{
	struct exynos_wlbt *w = bt->w;
	struct bsmhcp_header __iomem *hdr;
	struct bhcs __iomem *bhcs;
	u32 hdr_off;
	u8 frame[7] = { MMTRANS_CHAN_ID_SERVICE_MANAGEMENT, BSMHCP_SERVICE_ID_BT,
			 BSMHCP_SM_MSG_START_REQ };
	unsigned long timeout;
	int err = 0;

	mutex_lock(&bt->lock);
	if (bt->started) {
		mutex_unlock(&bt->lock);
		return 0;
	}
	if (!w->hip4_control || !w->heap_next) {
		err = -ENODEV;
		goto out;
	}

	/*
	 * Allocate each block once.  Keeping a successful allocation across a
	 * failed start means a retry does not leak another hole in the heap.
	 */
	if (!bt->bhcs_ref)
		bt->bhcs_ref = wlbt_heap_alloc_aligned(w, sizeof(struct bhcs),
						BSMHCP_ALIGNMENT);
	if (!bt->bsmhcp_ref)
		bt->bsmhcp_ref = wlbt_heap_alloc_aligned(w, sizeof(struct bsmhcp_protocol),
						  BSMHCP_ALIGNMENT);
	if (!bt->bhcs_ref || !bt->bsmhcp_ref) {
		err = -ENOMEM;
		goto out;
	}

	/* The BT configuration is a small HCF file, not a MIB. */
	if (!bt->config_ref) {
		const struct firmware *fw;

		if (!request_firmware(&fw, "mx140/mx140_bt.hcf", w->dev)) {
			bt->config_ref = wlbt_heap_alloc_aligned(w, fw->size,
							BSMHCP_ALIGNMENT);
			if (bt->config_ref) {
				memcpy_toio(w->dram + bt->config_ref, fw->data, fw->size);
				bt->config_len = fw->size;
			}
			release_firmware(fw);
		} else {
			dev_warn(w->dev, "mx140_bt.hcf not available, starting without it\n");
		}
	}

	/* Rebuild BSMHCP on every start so a stopped service starts clean. */
	hdr_off = bt->bsmhcp_ref;
	memset_io(w->dram + hdr_off, 0, sizeof(struct bsmhcp_protocol));
	hdr = w->dram + hdr_off;
	hdr->magic_value = cpu_to_le32(BSMHCP_MAGIC);
	hdr->bg_to_ap_int_src = WLBT_BIT_BT_BG_TO_HOST;
	hdr->fg_to_ap_int_src = WLBT_BIT_BT_FG_TO_HOST;
	hdr->ap_to_bg_int_src = WLBT_BIT_BT_AP_TO_BG;
	hdr->ap_to_fg_int_src = WLBT_BIT_BT_AP_TO_FG;
	hdr->ap_to_fg_m4_int_src = WLBT_BIT_BT_AP_TO_FG_M4;
	hdr->info_bg_to_ap_int_src = WLBT_BIT_BT_INFO_BG_TO_HOST;
	hdr->info_ap_to_bg_int_src = WLBT_BIT_BT_INFO_AP_TO_BG;
	hdr->acl_buffer_size = cpu_to_le32(BSMHCP_ACL_BUFFER_SIZE);
	hdr->cmd_evt_buffer_size = cpu_to_le32(BSMHCP_CMD_EVT_BUFFER_SIZE);
	hdr->acl_tx_buffers = cpu_to_le32(BSMHCP_ACL_TX_BUFFERS);
	hdr->firmware_control = 0;

	bitmap_zero(bt->tx_used, BSMHCP_ACL_TX_BUFFERS);
	memset(bt->l2cap_remaining, 0, sizeof(bt->l2cap_remaining));
	memset(bt->l2cap_cid, 0, sizeof(bt->l2cap_cid));

	bhcs = w->dram + bt->bhcs_ref;
	bhcs->version = cpu_to_le32(2);
	bhcs->bsmhcp_protocol_offset = cpu_to_le32(bt->bsmhcp_ref);
	bhcs->bsmhcp_protocol_length = cpu_to_le32(sizeof(struct bsmhcp_protocol));
	bhcs->configuration_offset = cpu_to_le32(bt->config_ref);
	bhcs->configuration_length = cpu_to_le32(bt->config_len);
	bhcs->bluetooth_address_lap = cpu_to_le32(bt->addr & 0xffffff);
	bhcs->bluetooth_address_uap = (bt->addr >> 24) & 0xff;
	bhcs->bluetooth_address_nap = cpu_to_le16((bt->addr >> 32) & 0xffff);
	wmb();

	put_unaligned_le32(bt->bhcs_ref, frame + 3);
	reinit_completion(&bt->start_cfm);
	if (!wlbt_stream_write(w, &w->mgmt_out, frame, sizeof(frame))) {
		err = -EIO;
		goto out;
	}
	timeout = wait_for_completion_timeout(&bt->start_cfm, msecs_to_jiffies(3000));
	if (!timeout) {
		dev_err(w->dev, "Bluetooth service start timed out (BHCS 0x%x)\n",
			bt->bhcs_ref);
		err = -ETIMEDOUT;
		goto out;
	}

	/* Set the flag before unmasking: an event that lands the instant the
	 * line is unmasked must not be acknowledged and then dropped by the
	 * interrupt path, which would leave it in the ring with no edge to
	 * come back for it.
	 */
	spin_lock_irq(&bt->ring_lock);
	bt->started = true;
	spin_unlock_irq(&bt->ring_lock);
	bt_unmask(bt);
	dev_info(w->dev, "Bluetooth service running (BHCS 0x%x, BSMHCP 0x%x, hcf %u bytes, features 0x%x)\n",
		 bt->bhcs_ref, bt->bsmhcp_ref, bt->config_len,
		 bt_read(bt, BT_HDR_OFF(firmware_features)));
	out:
	mutex_unlock(&bt->lock);
	return err;
}

static void wlbt_bt_stop(struct wlbt_bt *bt)
{
	struct exynos_wlbt *w = bt->w;
	u8 frame[7] = { MMTRANS_CHAN_ID_SERVICE_MANAGEMENT, BSMHCP_SERVICE_ID_BT,
			 BSMHCP_SM_MSG_STOP_REQ };
	unsigned long timeout;

	mutex_lock(&bt->lock);
	if (!bt->started) {
		mutex_unlock(&bt->lock);
		return;
	}

	bt_mask(bt);
	reinit_completion(&bt->stop_cfm);
	if (wlbt_stream_write(w, &w->mgmt_out, frame, sizeof(frame))) {
		timeout = wait_for_completion_timeout(&bt->stop_cfm,
						      msecs_to_jiffies(3000));
		if (!timeout)
			dev_warn(w->dev, "Bluetooth service stop timed out\n");
	}

	spin_lock_irq(&bt->ring_lock);
	bt->started = false;
	spin_unlock_irq(&bt->ring_lock);
	cancel_work_sync(&bt->rx_work);
	mutex_unlock(&bt->lock);
}

static int wlbt_bt_hci_open(struct hci_dev *hdev)
{
	return wlbt_bt_start(hci_get_drvdata(hdev));
}

static int wlbt_bt_hci_close(struct hci_dev *hdev)
{
	wlbt_bt_stop(hci_get_drvdata(hdev));
	return 0;
}

bool wlbt_bt_mgmt(struct wlbt_bt *bt, const u8 *msg, u32 len)
{
	if (!bt || len < 6 || msg[0] != MMTRANS_CHAN_ID_SERVICE_MANAGEMENT ||
	    msg[1] != BSMHCP_SERVICE_ID_BT)
		return false;

	switch (msg[2]) {
	case BSMHCP_SM_MSG_START_CFM:
		complete(&bt->start_cfm);
		break;
	case BSMHCP_SM_MSG_STOP_CFM:
		complete(&bt->stop_cfm);
		break;
	default:
		break;
	}
	return true;
}

void wlbt_bt_irq(struct wlbt_bt *bt, u32 status)
{
	if (!bt || !bt->started)
		return;
	if (status & (BIT(WLBT_BIT_BT_BG_TO_HOST) | BIT(WLBT_BIT_BT_FG_TO_HOST)))
		schedule_work(&bt->rx_work);
}

static int wlbt_bt_dbg_get(void *data, u64 *val)
{
	struct wlbt_bt *bt = data;

	*val = bt->started;
	return 0;
}

static int wlbt_bt_dbg_set(void *data, u64 val)
{
	struct wlbt_bt *bt = data;

	if (val)
		return wlbt_bt_start(bt);
	wlbt_bt_stop(bt);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(wlbt_bt_fops, wlbt_bt_dbg_get, wlbt_bt_dbg_set, "%llu\n");

struct wlbt_bt *wlbt_bt_init(struct exynos_wlbt *w)
{
	struct wlbt_bt *bt;
	int err;

	bt = devm_kzalloc(w->dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return ERR_PTR(-ENOMEM);

	bt->w = w;
	wlbt_bt_load_addr(bt);
	mutex_init(&bt->lock);
	spin_lock_init(&bt->ring_lock);
	init_completion(&bt->start_cfm);
	init_completion(&bt->stop_cfm);
	INIT_WORK(&bt->rx_work, wlbt_bt_rx_work);

	bt->hdev = hci_alloc_dev();
	if (!bt->hdev) {
		err = -ENOMEM;
		goto err;
	}
	bt->hdev->bus = HCI_VIRTUAL;
	bt->hdev->open = wlbt_bt_hci_open;
	bt->hdev->close = wlbt_bt_hci_close;
	bt->hdev->send = wlbt_bt_hci_send;

	/*
	 * Sensible values until Read Buffer Size comes back.  The free ring
	 * has one slot per ACL TX buffer, and the firmware reports the real
	 * sizes during HCI init.
	 */
	bt->hdev->acl_mtu = BSMHCP_ACL_BUFFER_SIZE;
	bt->hdev->acl_pkts = BSMHCP_ACL_TX_BUFFERS;
	bt->hdev->le_mtu = 251;
	bt->hdev->le_pkts = BSMHCP_ACL_TX_BUFFERS;
	SET_HCIDEV_DEV(bt->hdev, w->dev);
	hci_set_drvdata(bt->hdev, bt);

	err = hci_register_dev(bt->hdev);
	if (err) {
		dev_err(w->dev, "cannot register the Bluetooth HCI device: %d\n", err);
		hci_free_dev(bt->hdev);
		bt->hdev = NULL;
		goto err;
	}

	bt->debugfs = debugfs_create_file("bt", 0600, w->debugfs, bt,
					  &wlbt_bt_fops);
	dev_info(w->dev, "Bluetooth HCI device hci%d registered\n", bt->hdev->id);

	return bt;
err:
	return ERR_PTR(err);
}

void wlbt_bt_exit(struct wlbt_bt *bt)
{
	if (!bt)
		return;
	debugfs_remove(bt->debugfs);
	/* Stop the peer while the service-management channel still routes its
	 * STOP_CFM back here, then take the HCI device down and drain any work
	 * that an already-asserted mailbox line may have queued.
	 */
	wlbt_bt_stop(bt);
	hci_unregister_dev(bt->hdev);
	cancel_work_sync(&bt->rx_work);
	hci_free_dev(bt->hdev);
	mutex_destroy(&bt->lock);
}

#else /* !CONFIG_BT */

struct wlbt_bt;

struct wlbt_bt *wlbt_bt_init(struct exynos_wlbt *w)
{
	return NULL;
}

void wlbt_bt_exit(struct wlbt_bt *bt)
{
}

bool wlbt_bt_mgmt(struct wlbt_bt *bt, const u8 *msg, u32 len)
{
	return false;
}

void wlbt_bt_irq(struct wlbt_bt *bt, u32 status)
{
}

#endif /* CONFIG_BT */
