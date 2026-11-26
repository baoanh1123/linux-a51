/* SPDX-License-Identifier: GPL-2.0 */
/* Shared definitions for the Exynos 9610 SCSC/WLBT driver. */
#ifndef _EXYNOS9610_WLBT_H
#define _EXYNOS9610_WLBT_H
/*
 * Samsung Exynos 9610 SCSC/WLBT power and boot handshake.
 *
 * The Galaxy A51 has no SDIO and no PCIe WiFi.  WiFi and Bluetooth live in a
 * separate Samsung subsystem - "SCSC", with the combo chip the vendor calls
 * mx140 - that the AP boots by hand and then talks to over a mailbox.  This
 * driver is the hardware half of that: get the subsystem out of reset and
 * answer its configuration request so that it can start executing firmware.
 *
 * There is no public documentation for the block, so every sequence below is
 * taken from the vendor driver:
 *   DOWNSTREAM_READONLY/drivers/misc/samsung/scsc/platform_mif_9610.c
 *   DOWNSTREAM_READONLY/drivers/misc/samsung/scsc/mif_reg_S5E9610.h
 *
 * Boot protocol (platform_mif_9610.c:1021-1069 "platform_mif_reset_release"):
 *
 *	WLBT_CTRL_NS |= MASK_PWR_REQ	   AP takes power ownership
 *	WLBT_CTRL_NS &= ~WLBT_RESET_SET	   release the subsystem reset
 *	WLBT_CTRL_S  |= WLBT_START	   start the boot sequencer
 *	...  the boot ROM asserts SPI 73 (WB2AP_CFG_REQ)  ...
 *	[CFG_REQ handler]		   open the TZPC, program the bus
 *					   windows, patch the ROM, ACK
 *
 * The peer only becomes reachable once that handshake completes, so reaching
 * CFG_DONE is the first real milestone of the port: it proves the PMU bits,
 * the secure monitor call, the bus window programming, the ROM patch and the
 * mailbox interrupt are all correct.  Firmware download and the HIP transport
 * come on top of it.
 *
 * Every write below hits a WLBT register, in the WLBT power domain, so this
 * cannot disturb the rest of the SoC.  It is still a module on purpose: a
 * mistake here must never be able to stop the platform from booting.
 */

#include <linux/arm-smccc.h>
#include <linux/unaligned.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/nmi.h>
#include <linux/timer.h>
#include <linux/debugfs.h>
#include <linux/rtnetlink.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reboot.h>
#include <linux/seq_file.h>
#include <linux/etherdevice.h>
#include <linux/inet.h>
#include <net/cfg80211.h>
#include <linux/slab.h>
#include <linux/skbuff.h>

/* PMU (0x11860000) - WLBT power and reset control, Exynos 9610 UM 9.6.6. */
#define WLBT_CTRL_NS			0x0050	/* non-secure control */
#define WLBT_CTRL_S			0x0054	/* secure control */
#define WLBT_STAT			0x0058
#define WLBT_DEBUG			0x005c

#define WLBT_RESET_SET			BIT(0)	/* 1 = hold WLBT in reset */
#define WLBT_PWRON			BIT(1)
#define WLBT_ACTIVE_EN			BIT(5)
#define WLBT_RESET_REQ_EN		BIT(7)
#define WLBT_RESET_REQ_CLR		BIT(8)	/* 1 = clear the reset request */
#define MASK_PWR_REQ			BIT(18)	/* 1 = AP owns the power request */
#define MASK_TCXO_REQ			BIT(20)

#define WLBT_START			BIT(0)	/* start the reset sequencer */
#define WLBT_PWRDN_DONE			BIT(0)
#define WLBT_ACCESS_MIF			BIT(4)

#define CENTRAL_SEQ_WLBT_CONFIGURATION	0x0180
#define CENTRAL_SEQ_WLBT_STATUS		0x0184
#define SYS_PWR_CFG			BIT(0)
#define SYS_PWR_CFG_2			(BIT(0) | BIT(1))
#define SYS_PWR_CFG_16			BIT(16)
#define STATES				0xff0000

/*
 * The low-power configuration registers the vendor clears before asserting
 * reset; "0" in LOWPWR mode means the block is powered down there.
 */
#define RESET_AHEAD_WLBT_SYS_PWR_REG	0x1360
#define CLEANY_BUS_WLBT_SYS_PWR_REG	0x1364
#define LOGIC_RESET_WLBT_SYS_PWR_REG	0x1368
#define TCXO_GATE_WLBT_SYS_PWR_REG	0x136c
#define WLBT_DISABLE_ISO_SYS_PWR_REG	0x1370
#define WLBT_RESET_ISO_SYS_PWR_REG	0x1374

/* MAILBOX_AP2WLBT (0x119c0000), mif_reg_S5E9610.h:11-34. */
#define MBOX_MCUCTRL			0x000
#define MBOX_INTGR0			0x008	/* raise IRQ to WLBT, [31:16] */
#define MBOX_INTCR0			0x00c	/* clear IRQ from WLBT, [31:16] */
#define MBOX_INTMR0			0x010	/* mask IRQ from WLBT, [31:16] */
#define MBOX_INTSR0			0x014	/* status, [31:16] */
#define MBOX_INTMSR0			0x018	/* unmasked status, [31:16] */
#define MBOX_INTGR1			0x01c	/* raise IRQ to WLBT, [15:0] */
#define MBOX_INTCR1			0x020	/* clear IRQ to WLBT, [15:0] */
#define MBOX_INTMR1			0x024	/* mask IRQ to WLBT, [15:0] */
#define MBOX_INTSR1			0x028	/* status, [15:0] */
#define MBOX_INTMSR1			0x02c	/* unmasked status, [15:0] */
#define MBOX_IS_VERSION			0x050
#define MBOX_ISSR_BASE			0x080	/* shared registers, 4 bytes each */
#define MBOX_SSC			0x180	/* semaphores, 4 bytes each */
#define MBOX_NUM_SSC			12
#define MBOX_NUM_ISSR			8

/* Bus window controllers.  The end address is exclusive. */
#define DBUS_BAAW_BASE			0x14c20000
#define PBUS_BAAW_BASE			0x14c30000
#define WLBT_REMAP_BASE			0x14c50000
#define BOOT_CFG_BASE			0x14c60000

#define BAAW_0_START			0x80000000
#define BAAW_0_END			0x81000000	/* 16 MiB = the MIF DRAM */
#define BAAW_ACCESS_CTRL		(BIT(31) | BIT(1) | BIT(0))

#define REMAP_BAAW_0			0x00
#define REMAP_CHIP_VERSION_ID		0x10

/* EMA parameters overloaded on CHIP_VERSION_ID, mif_reg_S5E9610.h:143-149. */
#define CHIP_VERSION_ID_EMA_MASK	0x003fffff
#define CHIP_VERSION_ID_EMA_VALUE	(BIT(20) | BIT(18) | BIT(13) | \
					 BIT(11) | BIT(5) | BIT(2))

/*
 * The peer starts its firmware when it is released from reset and finds an
 * entry point in mailbox 0, so the four mailbox registers have to be filled
 * before the power sequence runs (mxman.c:1155 "mbox_init"):
 *
 *	ISSR0 = firmware entry point
 *	ISSR1 = MIF-relative address of the firmware configuration (mxconf)
 *	ISSR2 = 0xbcdeedcb, the "you may start" magic
 *	ISSR3 = startup flags (0 = run normally)
 */
/* Maxwell agent messages, must match the firmware (mxman.c:529). */
#define WLBT_MM_START_IND		0
#define WLBT_MM_HALT_REQ		1
#define WLBT_MM_FORCE_PANIC		2
#define WLBT_MM_HOST_SUSPEND		3
#define WLBT_MM_HOST_RESUME		4
#define WLBT_MM_FW_CONFIG		5
#define WLBT_MM_HALT_RSP			6
#define WLBT_MM_SYSERR_IND		9

#define MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT	0
#define MMTRANS_CHAN_ID_SERVICE_MANAGEMENT	1

/* Service-management messages carry [service_id][msg][u32 optional_data]
 * after the channel byte (servman_messages.h), unlike the management channel.
 */
#define WLBT_SM_MSG_START_REQ		0
#define WLBT_SM_MSG_START_CFM		1
#define WLBT_SM_MSG_STOP_REQ		2

/* SCSC_SERVICE_ID_* (include/scsc/scsc_mx.h). */
#define WLBT_SERVICE_ID_WLAN		1

#define MBOX_ISSR_ENTRY			0	/* fw entry point */
#define MBOX_ISSR_CONFIG		1	/* mxconf, MIF-relative */
#define MBOX_ISSR_MAGIC			2
#define MBOX_ISSR_FLAGS			3
#define MBOX_START_MAGIC		0xbcdeedcb

/*
 * Infrastructure configuration.  The peer boots, then reads a pointer out of
 * mailbox 1 to this structure, which tells it where the shared streams are in
 * DRAM and which interrupt bit signals each of them (mxconf.h, mxman.c:1086).
 *
 * The vendor's own header says the structures are deliberately *not* packed -
 * the old "__packed struct" syntax never took effect and correcting it would
 * desynchronise the firmware - so these are plain C structs with natural
 * alignment, laid out exactly like mxconf.h.
 */
#define MXCONF_MAGIC			0x79828486
#define MXCONF_VERSION_MAJOR		0
#define MXCONF_VERSION_MINOR		5
#define MXCONF_GDB_M4_MAX		0

/*
 * Stream buffer sizes, from the vendor transports: the management channel is
 * 512 bytes of 128-byte messages (mxmgmt_transport.c:241), the firmware log is
 * 16 KiB of 4-byte cells (mxlog_transport.c:17).
 */
#define MGMT_PACKET_SIZE		128
#define MGMT_NUM_PACKETS		4
#define GDB_PACKET_SIZE			4
#define GDB_NUM_PACKETS			(2 * 1024 / GDB_PACKET_SIZE)
#define LOG_PACKET_SIZE			4
#define LOG_BUF_LENGTH			(16 * 1024)
#define LOG_NUM_PACKETS			(LOG_BUF_LENGTH / LOG_PACKET_SIZE)

/* Log packet header: 16 bits of sync, 8 bits of level, 8 bits of length
 * (mxlog.c:310-345).  Phase 4 payloads are plain strings, phase 5 ones are
 * compressed against a string table, which the AP side does not have yet.
 */
#define MXLOG_SYNC_PHASE_4		0xa55a
#define MXLOG_SYNC_PHASE_5		0x9669

/*
 * Interrupt bit allocation.  The vendor allocates these dynamically from the
 * same pool it keeps bit 0 in for PANIC, so streams start at bit 1; the peer
 * learns which is which from the configuration, so the numbers only have to be
 * consistent between the two sides.
 */
/*
 * Interrupt bit indices, as the vendor's mifintrbit allocates them: bit 0 of
 * the from-host bitmap is reserved for forcing a panic, so the first
 * allocation lands on 1 and everything after it shifts by one.
 */
#define WLBT_BIT_MGMT_IN_READ		1
#define WLBT_BIT_MGMT_IN_WRITE		0
#define WLBT_BIT_MGMT_OUT_READ		1
#define WLBT_BIT_MGMT_OUT_WRITE		2
#define WLBT_BIT_GDB_R4_IN_READ		3
#define WLBT_BIT_GDB_R4_IN_WRITE	2
#define WLBT_BIT_GDB_R4_OUT_READ	3
#define WLBT_BIT_GDB_R4_OUT_WRITE	0	/* reserved panic bit */
#define WLBT_BIT_GDB_M4_IN_READ		1
#define WLBT_BIT_GDB_M4_IN_WRITE	4
#define WLBT_BIT_GDB_M4_OUT_READ	5
#define WLBT_BIT_GDB_M4_OUT_WRITE	0	/* reserved panic bit */
#define WLBT_BIT_LOG_READ		4
#define WLBT_BIT_LOG_WRITE		6

/*
 * HIP4, the frame transport the WLAN service runs on.  The peer is told where
 * the shared region is and which interrupt bits belong to it (hip4.h), so these
 * only have to stay clear of the management/log/gdb bits above.
 */
#define WLBT_BIT_HIP4_FROM_HOST		7	/* host -> peer */
#define WLBT_BIT_HIP4_TO_HOST		8	/* peer -> host */
/*
 * Bluetooth uses the next free mailbox bits.  The peer writes the two to-host
 * bits; the host raises the two from-host bits.  They stay masked until the BT
 * service is started so a stray edge cannot take the shared IRQ line.
 */
#define WLBT_BIT_BT_BG_TO_HOST		9	/* peer -> host, HCI events */
#define WLBT_BIT_BT_FG_TO_HOST		10	/* peer -> host, ACL data */
#define WLBT_BIT_BT_AP_TO_BG		11	/* host -> peer, HCI commands */
#define WLBT_BIT_BT_AP_TO_FG		12	/* host -> peer, ACL data */
#define WLBT_BIT_BT_AP_TO_FG_M4		14	/* host -> peer, ACL data (M4) */
#define WLBT_BIT_BT_INFO_BG_TO_HOST	13	/* peer -> host, info channel */
#define WLBT_BIT_BT_INFO_AP_TO_BG	15	/* host -> peer, info channel */

#define HIP4_CONFIG_OFFSET		0x00000
#define HIP4_CONFIG_SIZE		0x02000	/* 8 KiB, the control block */
#define HIP4_MIB_OFFSET		(HIP4_CONFIG_OFFSET + HIP4_CONFIG_SIZE)
#define HIP4_MIB_SIZE			0x08000	/* 32 KiB */
#define HIP4_TX_OFFSET			(HIP4_MIB_OFFSET + HIP4_MIB_SIZE)
#define HIP4_TX_DAT_SIZE		0x100000
#define HIP4_TX_CTL_OFFSET		(HIP4_TX_OFFSET + HIP4_TX_DAT_SIZE)
#define HIP4_TX_CTL_SIZE		0x10000
#define HIP4_TX_SIZE			(HIP4_TX_DAT_SIZE + HIP4_TX_CTL_SIZE)
#define HIP4_RX_OFFSET			(HIP4_TX_CTL_OFFSET + HIP4_TX_CTL_SIZE)
#define HIP4_RX_SIZE			0x100000
#define HIP4_TOTAL_MEM			(HIP4_CONFIG_SIZE + HIP4_MIB_SIZE + \
					 HIP4_TX_SIZE + HIP4_RX_SIZE)
/* Hardware configuration files carry this version word after their prologue. */
#define MIB_PROLOGUE_SIZE	8
#define MIB_PROLOGUE_MARKER	0x0120

#define HIP4_ALLOC_SIZE			0x280000	/* what hip4_init asks for */
/* Control frames kept for the debugfs "tx" file. */
#define WLBT_TX_SLOTS	512
#define WLBT_TX_BODY	40
#define WLBT_TX_DATA	64

/** struct wlbt_tx_rec - one control frame sent, as the vendor logs it */
struct wlbt_tx_rec {
	s64 t;
	u16 id;
	u16 len;
	u16 dlen;		/* payload appended behind the signal */
	u16 pad;
	u8 body[WLBT_TX_BODY];
	u8 data[WLBT_TX_DATA];
};

/* Received frames kept for the debugfs "frames" file. */
#define WLBT_FRAME_RING			32
#define WLBT_FRAME_DATA			64

/*
 * The peer reports its own faults as a DEBUG_FAULT_IND (0x8302) with the
 * offending id and a DEBUG_WORD12IND (0x8301) carrying twelve words of context
 * right behind it.  They are rare, so they get their own ring and survive the
 * frame ring being recycled.
 */
#define WLBT_FAULT_RING			16
/*
 * The peer's own log records are far too numerous to keep, but the ones a
 * protected frame provokes have to be told apart from the background.  They
 * are counted by module, sub identifier and the first two words instead, so
 * two runs can simply be subtracted.
 */
#define WLBT_LOG_SLOTS			192
/*
 * A histogram of the fapi identifiers the peer sends, with the last body of
 * each: counting them is the only way to see an indication that arrives once
 * per frame without printing from the drain.
 */
#define WLBT_ID_SLOTS			48
#define WLBT_ID_BODY			32
#define WLBT_FAULT_DATA			40

#define HIP4_Q_NUM			6
#define HIP4_Q_LEN			256
#define HIP4_Q_IDX_SIZE			1
#define HIP4_DAT_SLOTS			512
#define HIP4_CTL_SLOTS			32

#define HIP4_CONFIG_MAGIC		0xcaba0401
#define HIP4_INIT_MAGIC			0xcaaa0400

/* Both configuration blocks are packed; only the container has alignment. */
struct wlbt_hip4_init {
	__le32 magic;
	__le32 conf_ver;		/* peer owned: which block it chose */
	__le32 version_a_ref;
	__le32 version_b_ref;
} __packed;

struct wlbt_hip4_cfg_v4 {
	__le32 magic;
	__le16 ver;
	__le16 len;
	__le32 compat_flag;		/* peer owned */
	__le16 sap_mlme_ver;
	__le16 sap_ma_ver;
	__le16 sap_debug_ver;
	__le16 sap_test_ver;
	__le32 fw_build_id;
	__le32 fw_patch_id;
	u8 unidat_req_headroom;
	u8 unidat_req_tailroom;
	u8 bulk_buffer_align;
	u8 host_cache_line;
	__le32 host_buf_loc;
	__le32 host_buf_sz;
	__le32 fw_buf_loc;
	__le32 fw_buf_sz;
	__le32 mib_loc;
	__le32 mib_sz;
	__le32 log_config_loc;
	__le32 log_config_sz;
	u8 mif_fh_int_n;		/* host -> peer bit */
	u8 rsv1[3];
	u8 mif_th_int_n[HIP4_Q_NUM];	/* peer -> host bit per queue */
	u8 rsv2[2];
	__le32 scbrd_loc;
	__le16 q_num;
	__le16 q_len;
	__le16 q_idx_sz;
	u8 rsv3[2];
	__le32 q_loc[HIP4_Q_NUM];
	u8 rsv_nosmapper[99];
	u8 rsv4[16];
} __packed;

struct wlbt_hip4_cfg_v5 {
	__le32 magic;
	__le16 ver;
	__le16 len;
	__le32 compat_flag;		/* peer owned */
	__le16 sap_mlme_ver;
	__le16 sap_ma_ver;
	__le16 sap_debug_ver;
	__le16 sap_test_ver;
	__le32 fw_build_id;
	__le32 fw_patch_id;
	u8 unidat_req_headroom;
	u8 unidat_req_tailroom;
	u8 bulk_buffer_align;
	u8 host_cache_line;
	__le32 host_buf_loc;
	__le32 host_buf_sz;
	__le32 fw_buf_loc;
	__le32 fw_buf_sz;
	__le32 mib_loc;
	__le32 mib_sz;
	__le32 log_config_loc;
	__le32 log_config_sz;
	u8 mif_fh_int_n;
	u8 mif_th_int_n;
	u8 rsv1[2];
	__le32 scbrd_loc;
	__le16 q_num;
	__le16 q_len;
	__le16 q_idx_sz;
	u8 rsv2[2];
	__le32 q_loc[HIP4_Q_NUM];
	u8 rsv3[16];
} __packed;

/*
 * The peer cache-invalidates a queue array with DCIMVAC in 32 byte steps and
 * asserts when the base is not 32 byte aligned, so each element is padded to a
 * whole number of cache lines (1027 bytes of content, 1056 on the wire).
 */
struct wlbt_hip4_q {
	__le32 array[HIP4_Q_LEN];
	u8 idx_read;
	u8 idx_write;
	u8 total;
	u8 rsv[29];
};

/*
 * HIP4 datapath.  Control and data frames travel as "mbulk" segments: a 20 byte
 * descriptor, then the FAPI signal with four bytes reserved in front of it for
 * the peer, then the payload.  The host hands the peer the MIF offset of each
 * segment through a 256 entry array and keeps the queue indices in the
 * scoreboard, so both sides map the same memory.
 *
 * Scoreboard byte layout (hip4.c q_idx_layout): the host owns the write index
 * of the from-host queues and the read index of the to-host ones, the peer owns
 * the mirror image at +64.
 */
#define HIP4_SLOT_SIZE			2048
#define HIP4_SCRATCH_SLOTS		64
#define HIP4_MBULK_SIZE			20
#define HIP4_SIG_RESERVED		4	/* peer's own signal header */
#define HIP4_MAX_SLOTS			512
#define HIP4_SIG_DUMP			64
#define HIP4_SCB_FW_OWN			64

#define HIP4_Q_FH_CTL			0
#define HIP4_Q_FH_DAT			1
#define HIP4_Q_FH_RFB			2
#define HIP4_Q_TH_CTL			3
#define HIP4_Q_TH_DAT			4
#define HIP4_Q_TH_RFB			5

/* mbulk flags, classes and pool ids (mbulk_def.h, mbulk.h). */
#define MBULK_F_SIG			0x01
#define MBULK_F_OBOUND			0x04
#define WLBT_AMSDU_FRAME		16384
#define WLBT_ETH_FRAME			2400
#define MBULK_F_CHAIN_HEAD		0x40
#define MBULK_F_CHAIN			0x80
#define MBULK_MAX_CHAIN			16
#define MBULK_CLASS_FROM_HOST_DAT	4
#define MBULK_CLASS_FROM_HOST_CTL	5
#define MBULK_POOL_ID_DATA		0
#define MBULK_POOL_ID_CTRL		1

/* FAPI signals name this side as the sender (mlme.c:3545). */
#define FAPI_SENDER_PID			0xc001
#define FAPI_MLME_ADD_SCAN_REQ		0x2005
#define FAPI_MLME_ADD_INFO_ELEMENTS_REQ	0x2004
/* The peer reads the elements named this way as the link's own description. */
#define FAPI_PURPOSE_ASSOCIATION_REQUEST	0x0010
#define FAPI_MLME_SET_IP_ADDRESS_REQ	0x2015
#define FAPI_MLME_CONNECT_REQ		0x200b
#define FAPI_MLME_DISCONNECT_REQ	0x200e
#define FAPI_MLME_DEL_VIF_REQ		0x2008
#define FAPI_MLME_CONNECT_RES		0x2200
#define FAPI_MLME_CONNECTED_RES		0x2201
#define FAPI_MLME_SET_ACL_REQ		0x2016
#define FAPI_MA_UNITDATA_REQ		0x1000
#define FAPI_MA_UNITDATA_IND		0x1300
#define FAPI_PRIORITY_CONTENTION	0x8000
/* The data the peer carries for a station is ethernet, not 802.11. */
#define FAPI_DATAUNIT_IEEE802_3		0x0001
#define FAPI_MLME_SET_CACHED_CHANNELS	0x201f
#define FAPI_MLME_ADD_VIF_REQ		0x2007
#define FAPI_MLME_SET_PMK_REQ		0x201c
#define FAPI_MLME_SET_HOST_STATE_REQ	0x2028
#define FAPI_MLME_SETKEYS_REQ		0x2018
#define FAPI_MLME_SET_COUNTRY_REQ	0x2033

/* Key types the peer knows (fapi.h). */
#define FAPI_KEYTYPE_GROUP	0
#define FAPI_KEYTYPE_PAIRWISE	1
#define FAPI_KEYTYPE_WEP	2
#define FAPI_KEYTYPE_IGTK	3

struct wlbt_mbulk {
	__le32 next_offset;
	u8 flag;
	u8 clas;
	u8 pid;
	u8 refcnt;
	__le16 dat_bufsz;
	__le16 sig_bufsz;
	__le16 len;
	__le16 head;
	__le32 chain_next_offset;
} __packed;

struct wlbt_fapi_hdr {
	__le16 id;
	__le16 receiver_pid;
	__le16 sender_pid;
	__le32 fw_reference;
} __packed;

/* Producer and consumer index byte of every queue. */

struct wlbt_hip4_pool {
	u32 base;			/* MIF offset of the first slot */
	u32 slots;
	u32 next;
	DECLARE_BITMAP(used, HIP4_MAX_SLOTS);
};

/*
 * The peer finds both configuration blocks through this header, so the refs we
 * store here are the only thing that has to line up with the firmware - the
 * offsets inside the container are ours to choose.
 */
struct wlbt_hip4_control {
	struct wlbt_hip4_init init;
	struct wlbt_hip4_cfg_v5 v5 __aligned(32);
	struct wlbt_hip4_cfg_v4 v4 __aligned(32);
	__le32 scoreboard[256] __aligned(64);
	struct wlbt_hip4_q q[HIP4_Q_NUM] __aligned(64);
} __aligned(4096);

/* Every bit allocated to-host is one this side wants to hear about. */
#define WLBT_IRQ_BITS			GENMASK(6, 0)

#define MXCONF_FWCONFIG_UNUSED		0

/* Firmware header (fwhdr.c:14-31): "smxf" at offset 8, header at the start of
 * the image.  The entry point is a MIF-relative address, so it is added to the
 * window the peer sees, not to the physical base.
 */
#define FW_HDR_MAGIC_OFFSET		8
#define FW_HDR_RUNTIME_LENGTH_OFFSET	36
#define FW_HDR_ENTRY_OFFSET		40
#define FW_HDR_PANIC_OFFSET		176	/* R4 panic record */
#define FW_HDR_BUILD_ID_OFFSET		48
#define FW_HDR_BUILD_ID_MAX		64
#define FW_HDR_MIN_SIZE			192

#define BOOT_CFG_HOLD			0x00	/* 1 = keep the peer held */
#define BOOT_CFG_ACK			0x04	/* 1 = configuration done */
#define BOOT_CFG_ROM			0x1000	/* replacement boot ROM words */

/* SMC_CMD_CONN_IF / EXYNOS_SET_CONN_TZPC, include/linux/smc.h and
 * mif_reg_S5E9610.h:180.  Same call the vendor makes before it touches the
 * windows: without it the WLBT TZPC stays secure and every window write is
 * ignored.
 */
#define SMC_CMD_CONN_IF			0x82000710
#define EXYNOS_SET_CONN_TZPC		0
#define EXYNOS_WLBT			1

/* Boot state machine, platform_mif_9610.c:122-127. */
enum wlbt_state {
	WLBT_IN_RESET = 0,
	WLBT_WAIT_CFG_REQ,
	WLBT_ACK_CFG_REQ,
	WLBT_CFG_DONE,
	WLBT_CFG_ERROR,
};

/** struct wlbt_cbuf - circular packet buffer, as the peer sees it (mxcbufconf) */
struct wlbt_cbuf {
	u32 buffer_loc;		/* MIF-relative offset of the cells */
	u32 num_packets;
	u32 packet_size;
	u32 read_index_loc;	/* MIF-relative offset of the read cursor */
	u32 write_index_loc;
};

/** struct wlbt_stream_conf - one simplex stream (mxstreamconf) */
struct wlbt_stream_conf {
	struct wlbt_cbuf buf_conf;
	u8 read_bit_idx;
	u8 write_bit_idx;
};

/** struct wlbt_trans_conf - a pair of streams (mxtransconf) */
struct wlbt_trans_conf {
	struct wlbt_stream_conf to_ap;
	struct wlbt_stream_conf from_ap;
};

/** struct wlbt_log_conf - the firmware log stream (mxlogconf) */
struct wlbt_log_conf {
	struct wlbt_stream_conf stream_conf;
};

/** struct wlbt_mxconf - the whole configuration (mxconf) */
struct wlbt_mxconf {
	u32 magic;
	u16 version_major;
	u16 version_minor;
	struct wlbt_trans_conf trans;		/* management transport */
	struct wlbt_trans_conf gdb_r4;		/* unused here */
	struct wlbt_trans_conf gdb_m4;		/* unused here */
	struct wlbt_log_conf logconf;
	u32 soc_revision;
	u32 flags;
	struct {
		u32 offset;
		u32 size;
	} fwconfig;				/* common HCF, absent on this board */
	struct wlbt_trans_conf gdb_m4_1;	/* unused here */
};

/**
 * struct wlbt_stream - one stream, host side
 * @buf: offset of the cells in MIF DRAM
 * @ridx: offset of the read cursor
 * @widx: offset of the write cursor
 */
struct wlbt_stream {
	const char *name;
	u32 buf;
	u32 ridx;
	u32 widx;
	u32 num_packets;
	u32 packet_size;
	u8 read_bit;
	u8 write_bit;
};

/**
 * struct wlbt_pbus_window - one peripheral bus window
 * @start: first address the peer sees
 * @end: first address the peer does not see
 * @target: physical base the window is redirected to
 */
struct wlbt_pbus_window {
	u32 start;
	u32 end;
	u32 target;
};

/*
 * platform_mif_9610.c:772-805.  Six windows, in the order the controller
 * expects them: CP mailbox, SHUB mailbox, USI CMGP, SYSREG CMGP, GPIO CMGP and
 * SHUB.  The peer needs all of them before it will run.
 */

/*
 * Replacement boot ROM for the Maxwell PMU+PROC, written into the config
 * window at BOOT_CFG_ROM while the peer is held.  The vendor calls it a "low
 * temp fix" (platform_mif_9610.c:618-681); the ROM it replaces is the one that
 * runs the CFG_REQ handshake, so it is not optional.
 */

/**
 * struct exynos_wlbt - SCSC platform state
 */
#define WLBT_PW_REGS	7

struct wlbt_pw_reg {
	u32 reg;
	u32 val;
};

#define WLBT_TRACE_MAX	4096

struct wlbt_trace_rec {
	u32 us;
	u32 val;
};

/* An SAE authentication frame is a header plus a curve point and a token. */
#define WLBT_SAE_FRAME			512

struct exynos_wlbt {
	struct device *dev;
	void __iomem *mbox;
	struct regmap *pmu;
	struct regmap *baaw_p;
	struct regmap *dbus_baaw;
	struct regmap *pbus_baaw;
	struct regmap *remap;
	struct regmap *boot_cfg;
	struct mutex lock;		/* serialises boot and reset */
	struct notifier_block reboot_nb;
	struct completion cfg_ack;
	enum wlbt_state state;
	phys_addr_t mem_base;		/* MIF DRAM, 16 MiB */
	resource_size_t mem_size;
	void __iomem *dram;		/* uncached view of it */
	u32 fw_entry;			/* MIF-relative entry point */
	u32 fw_runtime_length;
	u32 fw_size;			/* bytes of image copied in */
	u32 fw_panic_offset;
	bool fw_loaded;
	u32 heap_next;			/* MIF-relative bump pointer */
	u32 mxconf_offset;
	u32 hip4_ref;			/* MIF-relative HIP4 shared region */
	u32 hip4_control;
	spinlock_t hip4_lock;		/* one reader of the peer's queues */
	u32 hip4_scratch_seq;		/* rotates the reference segments */
	/*
	 * The types of frame seen recently, kept as data rather than as printk:
	 * the drain runs with the queue lock held, so the console cannot be the
	 * place a frame is inspected from.
	 */
	struct {
		u32 ref;
		u16 id;
		u8 flag;
		u8 len;
		u8 data[64];
	} frames[WLBT_FRAME_RING];
	unsigned int frame_next;
	unsigned int frame_total;
	/*
	 * Control frames this side has handed to the peer, in order.  The vendor
	 * stack logs the same thing (scsc logring bin_wifi_ctrl_tx, "HEX[n/n]"),
	 * so a connect can be compared signal by signal against the one Android
	 * makes on the same firmware - that is what says whether the peer's
	 * refusal is about the requests or about its own state.
	 */
	struct wlbt_tx_rec *tx;
	unsigned int tx_next;
	unsigned int tx_used;
	struct {
		u16 id;
		u8 len;
		u8 data[WLBT_FAULT_DATA];
	} faults[WLBT_FAULT_RING];
	unsigned int fault_next;
	struct {
		u16 module;
		u16 sub;
		u16 word0;
		u16 word1;
		u32 count;
	} log_slots[WLBT_LOG_SLOTS];
	unsigned int log_used;
	bool hip4_started;		/* peer took the service start request */
	struct wlbt_hip4_pool hip4_pool[2];
	unsigned int n_hip4_tx;
	unsigned int n_hip4_rx;
	unsigned int n_rx_amsdu_frames;
	unsigned int n_chain_probe;
	u64 drain_ns;
	unsigned int drain_calls;
	u8 amsdu_frame[WLBT_AMSDU_FRAME];
	u8 eth_frame[WLBT_ETH_FRAME];
	struct wiphy *wiphy;
	struct net_device *netdev;
	struct cfg80211_scan_request *scan_req;
	struct delayed_work scan_work;
	struct delayed_work connect_work;
	/*
	 * Diagnostics for a send that never comes back.  A management frame is
	 * written straight into the peer's memory, so if the peer has stopped
	 * reading its bus, the store never completes and the CPU that issued it
	 * stays there with interrupts off: the machine stops without leaving a
	 * single line behind.  A timer on another CPU notices the deadline and
	 * prints every CPU's stack into the persistent console before panicking,
	 * so the next reset explains itself.
	 */
	struct timer_list stall_timer;
	struct work_struct stall_report;
	unsigned long stall_deadline;
	/* Which store inside the send was in flight when the deadline passed. */
	unsigned int stall_phase;
	struct work_struct peer_work;	/* answers the connect indications */
	/* EAPOL that arrived while the post-connect housekeeping was still running. */
	struct sk_buff_head eapol_hold;
	bool post_connect_done;
	unsigned long scan_done;	/* when the last scan finished */
	bool port_authorized;	/* the peer has a pairwise key, so data may flow */
	unsigned int n_tx_unauthed;	/* frames dropped before the 4-way finished */
	u8 default_key_idx;	/* the index userspace marked as default */
	struct work_struct peer_gone_work;	/* takes the link down after a panic */
	int wdog_irq;
	bool peer_dead;
	u8 connect_bssid[ETH_ALEN];
	u16 scan_id;
	/* The last pairwise key the supplicant handed over, kept so that it can be
	 * sent again on demand.  The debugfs send path is not equivalent to this
	 * one - the peer installs a request from the driver and merely stores one
	 * injected there - so a faithful repeat install needs the driver to own
	 * the key.
	 */
	u8 ptk_key[32];
	u8 ptk_addr[ETH_ALEN];
	u32 ptk_cipher;
	u16 ptk_len;
	u16 ptk_id;
	bool ptk_valid;
	bool connecting;
	u8 connect_ssid[IEEE80211_MAX_SSID_LEN];
	size_t connect_ssid_len;
	u16 connect_freq;		/* channel of the join in progress */
	u64 mgmt_cookie;		/* names management frames sent to the peer */
	/* An authentication frame the peer handed up for SAE, copied out of the
	 * drain (which runs with interrupts off) for the work item that can sleep.
	 */
	struct work_struct sae_work;
	struct delayed_work sae_retry_work;
	unsigned int sae_retries;

	/*
	 * Scan results the peer reports arrive in the drain, in interrupt
	 * context, where cfg80211 must not be entered: it takes locks that a
	 * scan request in process context can already hold.  The frames are
	 * copied here and reported from scan_rep_work instead.
	 */
	struct work_struct scan_rep_work;
	spinlock_t scan_rep_lock;
	u8 scan_rep_head;
	u8 scan_rep_tail;
	u16 scan_rep_len[8];
	s16 scan_rep_rssi[8];
	u8 scan_rep_frame[8][512];
	u8 sae_frame[WLBT_SAE_FRAME];
	/* Frames waiting for delivery, in arrival order. */
	u8 sae_q[8][WLBT_SAE_FRAME];
	size_t sae_q_len[8];
	s16 sae_q_sig[8];
	unsigned int sae_q_head;
	unsigned int sae_q_tail;
	spinlock_t sae_q_lock;
	size_t sae_frame_len;
	bool sae_no_frame;		/* the indication named the AP without a frame */
	int sae_signal;
	/* Where the peer's SAE indication kept its frame, kept as data: the
	 * drain cannot print, and guessing wrong looks like a peer that never
	 * answered.
	 */
	u32 sae_dbg[8];
	/* The peer re-indicates a frame it has already handed up. */
	u8 sae_last[WLBT_SAE_FRAME];
	size_t sae_last_len;
	unsigned long sae_last_jiffies;
	/* An external auth is with the supplicant: do not cut it short. */
	bool sae_pending;
	/* A confirm has been handed up: later commits are stale replays. */
	bool sae_confirmed;
	bool sae_commit_seen;
	bool sae_sent_confirm;
	u8 sae_last_confirm[64];
	size_t sae_last_confirm_len;
	unsigned long sae_last_confirm_jiffies;
	/* A join held back until the SAE exchange finishes. */
	bool sae_defer;
	u8 defer_sig[sizeof(struct wlbt_fapi_hdr) + 24];
	size_t defer_sig_len;
	u8 defer_dat[2 + IEEE80211_MAX_SSID_LEN + 512];
	size_t defer_dat_len;
	unsigned int n_sae_ind;
	unsigned int n_mgmt_rx;
	/* The peer logs what its own state machine does, and those events say why
	 * a join was abandoned.  Kept as ids: this is the drain path.
	 */
	struct {
		u16 event;
		u16 vif;
	} fw_events[32];
	unsigned int n_fw_event;
	bool unsync_vif;		/* the pre-association interface is up */
	bool station_channel_set;	/* the station interface was put on a channel */
	u16 unsync_index;		/* the interface the pre-association frames use */
	unsigned int sae_chain;
	bool vif_created;		/* MLME_ADD_VIF was accepted */
	bool connect_reported;		/* the peer answered the join */
	u16 connect_status;		/* FAPI_RESULTCODE from MLME_CONNECT_IND */
	/* Indications that need a reply, raised by the drain and answered by
	 * the work item because the reply has to be sent outside the queue
	 * lock and cfg80211 cannot be called from the interrupt.
	 */
	bool connected;			/* cfg80211 was told the link is up */
	/*
	 * Room the peer needs in front of and behind the frame in a unitdata
	 * request, which it publishes in the HIP4 configuration once it has
	 * started: that is where it writes the 802.11 header it builds for an
	 * associated station.
	 */
	u16 tx_headroom;
	u16 tx_tailroom;
	/*
	 * Names each transmitted frame.  The vendor allocates them with the
	 * traffic queue in bits 0-1 and one increment per queue entry, i.e. by
	 * four (dev.h: slsi_tx_host_tag, bit 15 is ARP flow control), so the
	 * queue bits are always 0 for a management frame.  Counting by one made
	 * every frame claim a different queue in turn.
	 */
	u16 host_tag;
	bool peer_connect_cfm;		/* the peer took the join request */
	bool connect_res_sent;
	bool peer_connect_ind;
	bool peer_connected_ind;
	bool peer_disconnected_ind;
	u16 peer_aid;
	u16 peer_disconnect_reason;
	/*
	 * The peer announces a received frame with MLME_RECEIVED_FRAME_IND and,
	 * at least for the EAPOL frames a secured join depends on, sends the
	 * indication without the frame: the body is a descriptor, a frequency
	 * and what looks like an address.  The last one is kept so it can be
	 * read back without the log.
	 */
	u8 connect_pmk[32];		/* kept until the peer asks for the join */
	u32 connect_pmk_len;
	u8 peer_rxind[18];
	u32 peer_rxind_len;
	/* The last received-frame indication whole, so where the frame sits in
	 * it can be looked at without the drain having to guess.
	 */
	u8 rxind_raw[HIP4_SLOT_SIZE];
	u16 rxind_raw_len;
	u8 chain_probe[160];
	u16 chain_probe_len;
	u32 chain_probe_ref;
	/*
	 * The answer to MLME_GET: the peer's own counters are the only way to see
	 * why it discards a frame, since a discard is silent.
	 */
	u8 mibresp[64];
	u32 mibresp_len;
	u32 mibresp_psid;
	u32 peer_rxind_geom[4];		/* sig_bufsz, dat_bufsz, len, head */
	u32 n_rxind;
	struct wlbt_stream log;
	struct wlbt_stream mgmt_in;
	struct wlbt_stream mgmt_out;
	struct wlbt_stream gdb_r4_in;
	struct wlbt_stream gdb_r4_out;
	struct wlbt_stream gdb_m4_in;
	struct wlbt_stream gdb_m4_out;
	unsigned int n_fwlog;
	unsigned int n_fwlog_bad;
	unsigned int n_mgmt;
	unsigned int n_cfg_req;
	unsigned int n_cfg_bad;
	unsigned int n_wdog;
	unsigned int n_tx_dropped;
	unsigned int n_tx_calls;
	unsigned int n_mbox;
	u32 last_mbox_status;
	u32 peek_offset;		/* MIF-relative, set by debugfs write */
	u32 peek_words;
	/* The WLBT sequencer configuration is set by the bootloader; dropping it
	 * in wlbt_power_down() is one way, so keep the original values to be able
	 * to boot the peer again without reloading this module.
	 */
	struct wlbt_pw_reg pw_cfg[WLBT_PW_REGS];
	bool pw_cfg_dirty;
	struct {
		u16 id;
		u32 count;
		u8 len;
		u8 body[WLBT_ID_BODY];
	} ids[WLBT_ID_SLOTS];
	unsigned int n_ids;
	char find_needle[32];
	u32 find_len;
	u32 find_hits[64];
	unsigned int n_find;
	u32 trace_offset;		/* word to sample, MIF-relative */
	u32 trace_field;		/* non-zero: dereference, then add this */
	unsigned int trace_us;
	unsigned int n_trace;
	atomic_t trace_running;
	struct wlbt_trace_rec *trace;
	struct dentry *debugfs;
	struct wlbt_bt *bt;		/* Bluetooth service and HCI bridge */
};

u32 wlbt_stream_read(struct exynos_wlbt *w, struct wlbt_stream *s,
			    void *dst, u32 bytes);
bool wlbt_stream_write(struct exynos_wlbt *w, struct wlbt_stream *s,
			      const void *src, u32 bytes);
void wlbt_log_drain(struct exynos_wlbt *w);
bool wlbt_mgmt_send(struct exynos_wlbt *w, u8 channel, u8 ma_msg,
			   u32 arg);
void wlbt_mgmt_drain(struct exynos_wlbt *w);
bool wlbt_trace_start(struct exynos_wlbt *w);


extern const u8 wlbt_hip4_idx[HIP4_Q_NUM][2];
extern const struct wlbt_pbus_window wlbt_pbus_windows[];
extern const u32 wlbt_rom_patch[];

/* Cross-file declarations are appended below as the split needs them. */

struct wlbt_bt;
struct wlbt_bt *wlbt_bt_init(struct exynos_wlbt *w);
void wlbt_bt_exit(struct wlbt_bt *bt);
bool wlbt_bt_mgmt(struct wlbt_bt *bt, const u8 *msg, u32 len);
void wlbt_bt_irq(struct wlbt_bt *bt, u32 status);

void wlbt_boot(struct exynos_wlbt *w);
int wlbt_build_mxconf(struct exynos_wlbt *w);
bool wlbt_channel_passive(u16 freq);
void wlbt_debugfs_init(struct exynos_wlbt *w);
int wlbt_download_mibs(struct exynos_wlbt *w);
void wlbt_dram_bsses(struct exynos_wlbt *w, struct seq_file *s);
struct exynos_wlbt *wlbt_from_wiphy(struct wiphy *wiphy);
u32 wlbt_heap_alloc(struct exynos_wlbt *w, u32 bytes);
u32 wlbt_heap_alloc_aligned(struct exynos_wlbt *w, u32 bytes, u32 align);
int wlbt_hip4_build(struct exynos_wlbt *w);
void wlbt_hip4_drain(struct exynos_wlbt *w);
void wlbt_hip4_push(struct exynos_wlbt *w, u32 q, u32 ref);
void wlbt_hip4_read_limits(struct exynos_wlbt *w);
int wlbt_hip4_send(struct exynos_wlbt *w, bool ctrl, const void *sig, size_t sig_len, const void *data, size_t data_len);
void wlbt_hip4_state(struct exynos_wlbt *w);
u32 wlbt_load_mib(struct exynos_wlbt *w);
void wlbt_mib_send_slice(struct exynos_wlbt *w, const u8 *mib, size_t len);
int wlbt_mlme_add_scan(struct exynos_wlbt *w, u16 scan_id, struct cfg80211_scan_request *req);
int wlbt_power_down(struct exynos_wlbt *w);
int wlbt_register_wiphy(struct exynos_wlbt *w);
u32 wlbt_soc_revision(struct exynos_wlbt *w);
const char *wlbt_state_name(enum wlbt_state state);
void wlbt_trace(struct exynos_wlbt *w, const char *fmt, ...);
void wlbt_unregister_wiphy(struct exynos_wlbt *w);
extern bool connect_res_at_key;
extern bool connect_res_late;
extern char * firmware_name;
extern bool fw_log;
extern bool hip4_log;
extern bool hip4_ref_bare;
extern bool hip4_return;
extern bool host_keys;
extern bool info_elements;
extern bool install_gtk;
extern bool load_mib;
extern uint log_module;
extern int mgmt_tx_auth_dwell_us;
extern unsigned int mgmt_tx_auth_type;
extern unsigned int mgmt_tx_dwell_ms;
extern unsigned int mgmt_tx_freq_mode;
extern unsigned int mgmt_tx_vif;
extern bool mib_early;
extern unsigned int mib_slice_max;
extern bool prekey;
extern uint ptk_id;
extern uint ptk_type;
extern bool rx_chain;
extern bool rx_chain_frame;
extern bool rx_chain_probe;
extern bool rx_ind_off;
extern bool rx_off;
extern bool rx_ptr;
extern bool sae_defer_join;
extern bool sae_join_retry;
extern bool scan_log;
extern bool sec_ie_only;
extern bool send_pmk;
extern unsigned int send_ref;
extern char * set_ip;
extern int sig_refcnt;
extern bool sig_self_ref;
extern bool tx_log;
extern struct ieee80211_channel wlbt_2ghz_channels[13];
extern struct ieee80211_channel wlbt_5ghz_channels[25];
#define FAPI_MLME_SEND_FRAME_REQ	0x2010
#define FAPI_MESSAGETYPE_EAPOL_KEY_M123	0x0002
#define FAPI_MESSAGETYPE_EAPOL_KEY_M4	0x0003
#define FAPI_MESSAGETYPE_IEEE80211_ACTION	0x0010
#define FAPI_MESSAGETYPE_IEEE80211_MGMT		0x0011
#define FAPI_DATAUNIT_IEEE802_3_FRAME	0x0001
#define FAPI_DATAUNIT_IEEE802_11_FRAME	0x0000
#define FAPI_MLME_SET_CHANNEL_REQ	0x200a
#define FAPI_MLME_REGISTER_ACTION_FRAME_REQ 0x200f
#define FAPI_VIFTYPE_UNSYNCHRONISED	0x0000
#define WLBT_UNSYNC_VIF			2
#define FAPI_ACTION_FRAME_PUBLIC	BIT(4)
#define FAPI_ACTION_FRAME_RADIO_MEAS	BIT(5)
#define WLBT_STA_ACTION_FRAME_BITMAP \
	(BIT(4) | BIT(17) | BIT(10) | BIT(1) | BIT(9) | BIT(5))
#define WLBT_STA_ACTION_FRAME_SUSPEND \
	(BIT(4) | BIT(17) | BIT(10) | BIT(1) | BIT(9))
#define FAPI_MLME_SYNCHRONISED_RES	0x2205
#define FAPI_MLME_SYNCHRONISED_IND	0x231b
#define FAPI_AUTHENTICATIONTYPE_SAE	0x0003
#define FAPI_MLME_SYNCHRONISED_SIG	14	/* vif, rssi, two words of spare */
#define WLBT_CHIPID_PHYS	0x10000000
#define WLBT_CHIPID_REV		0x10
#define MIB_PSID_SUPPORTED_DATA_RATES	0x07f9
#define MIB_PSID_EXTENDED_CAPABILITIES	0x07ef
#define WLBT_SCAN_CHUNK		8192
#define WLBT_SCAN_OVERLAP	1024
#define WLBT_SCAN_SEEN		64
#define WLBT_SCAN_IE_MAX	512
#define WLBT_SCAN_RSSI		(-65 * 100)	/* dBm, this firmware reports none */
#define WLBT_SCAN_DELAY_MS	5000
#define WLBT_DEBUGFS_OPS(_write)					\
	{								\
		.owner = THIS_MODULE,					\
		.open = simple_open,					\
		.write = (_write),					\
		.llseek = noop_llseek,					\
	}
#define WLBT_PEEK_MAX_WORDS	4096
#define WLBT_FIND_CHUNK	1024	/* words */
#define FAPI_MLME_SET_REQ	0x2002
#define MIB_PSID_FW_ONLY	0x8000
#define MIB_SLICE_MAX		1800
struct wlbt_golden_mib {
	u16 psid;
	const u8 *value;
	size_t len;
};
extern const struct wlbt_golden_mib wlbt_golden_mibs[2];
struct exynos_wlbt *wlbt_from_netdev(struct net_device *ndev);
extern bool eapol_mlme;
extern bool fourway_1x;
extern bool fourway_offload;
extern bool sae_support;
extern struct ieee80211_supported_band wlbt_band_2ghz;
extern struct ieee80211_supported_band wlbt_band_5ghz;
extern const struct cfg80211_ops wlbt_cfg80211_ops;
extern const u32 wlbt_cipher_suites[5];
extern struct wiphy * wlbt_sae_wiphy;
extern const struct ieee80211_txrx_stypes wlbt_mgmt_stypes[NUM_NL80211_IFTYPES];
void wlbt_sae_retry_work(struct work_struct *work);
void wlbt_sae_work(struct work_struct *work);
void wlbt_scan_rep_work(struct work_struct *work);
void wlbt_scan_work(struct work_struct *work);
void wlbt_peer_work(struct work_struct *work);
void wlbt_eapol_drain(struct exynos_wlbt *w);
void wlbt_eapol_purge(struct exynos_wlbt *w);
void wlbt_connect_work(struct work_struct *work);
void wlbt_peer_gone_work(struct work_struct *work);
void wlbt_stall_report(struct work_struct *work);
void wlbt_stall_check(struct timer_list *t);
int wlbt_wlan_start(struct exynos_wlbt *w);
bool wlbt_bss_frame(const u8 *buf, size_t len, size_t *flen, u16 *freq);
int wlbt_mlme_setkeys(struct exynos_wlbt *w, u16 key_id, u16 key_type, const u8 *address, u32 cipher, const u8 *key, size_t key_len, const u8 *seq, size_t seq_len);
#endif /* _EXYNOS9610_WLBT_H */
