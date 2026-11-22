// SPDX-License-Identifier: GPL-2.0
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
#include <linux/debugfs.h>
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
#include <net/cfg80211.h>
#include <linux/slab.h>

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
#define WLBT_FAULT_DATA			28

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
#define FAPI_MLME_CONNECT_REQ		0x200b
#define FAPI_MLME_CONNECT_RES		0x2200
#define FAPI_MLME_CONNECTED_RES		0x2201
#define FAPI_MLME_SET_ACL_REQ		0x2016
#define FAPI_MA_UNITDATA_REQ		0x1000
#define FAPI_MA_UNITDATA_IND		0x1300
#define FAPI_PRIORITY_CONTENTION	0x8000
/* The data the peer carries for a station is ethernet, not 802.11. */
#define FAPI_DATAUNIT_IEEE802_3		0x0001
#define FAPI_MLME_SET_CACHED_CHANNELS	0x201f

/* Channels offered to the peer once a connection is up. */
#define WLBT_CACHED_CHANNELS		11
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
static const u8 wlbt_hip4_idx[HIP4_Q_NUM][2] = {
	{ 0, HIP4_SCB_FW_OWN + 0 },		/* from host, control */
	{ 1, HIP4_SCB_FW_OWN + 1 },		/* from host, data */
	{ HIP4_SCB_FW_OWN + 2, 2 },		/* from host, freed */
	{ HIP4_SCB_FW_OWN + 3, 3 },		/* to host, control */
	{ HIP4_SCB_FW_OWN + 4, 4 },		/* to host, data */
	{ 5, HIP4_SCB_FW_OWN + 5 },		/* to host, freed */
};

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
static const struct wlbt_pbus_window wlbt_pbus_windows[] = {
	{ 0xa0000000, 0xa0010000, 0x11950000 },
	{ 0xa0010000, 0xa0060000, 0x119a0000 },
	{ 0xa0060000, 0xa0100000, 0x11d00000 },
	{ 0xa0100000, 0xa0110000, 0x11c80000 },
	{ 0xa0110000, 0xa0120000, 0x11c20000 },
	{ 0xa0120000, 0xa0160000, 0x11200000 },
};

/*
 * Replacement boot ROM for the Maxwell PMU+PROC, written into the config
 * window at BOOT_CFG_ROM while the peer is held.  The vendor calls it a "low
 * temp fix" (platform_mif_9610.c:618-681); the ROM it replaces is the one that
 * runs the CFG_REQ handshake, so it is not optional.
 */
static const u32 wlbt_rom_patch[] = {
	0x90750002, 0x11a4c218, 0x75671191, 0x9075e090, 0x54b3e5e7, 0x30f76001,
	0xb14315a2, 0xb4b2e503, 0xb153fb03, 0xa90185fc, 0xacf50774, 0x75fdadb5,
	0xb47508a0, 0x54b3e501, 0x75fa7001, 0x907500b4, 0x78cb8018, 0x80837982,
	0x07a075c5, 0xb0783779, 0xb40754e6, 0x0b800207, 0xc404f6d9, 0xaf7590f5,
	0x75038000, 0x53229090, 0xce53eff7, 0xd90479fe, 0xfdce53fe, 0xfed90c79,
	0x75fbce53, 0x91530b92, 0xf7ce53fd, 0x5308f943, 0xf922fef9, 0xfbd8fed9,
	0x019e7522, 0x75cfc175, 0xc375a4c2, 0x47c4754a, 0x75a4c575, 0xc7756dc6,
	0x03d27540, 0x7510d375, 0xca7500c9, 0x00cb75d0, 0x7500cc75, 0x9b75009a,
	0x009c75c0, 0x78009d75, 0x12827402, 0xc6438b80, 0x74097802, 0x8b8012e7,
	0x75d09075, 0x9e750291, 0x01a97502, 0x00000022,
};

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
	struct {
		u16 id;
		u8 len;
		u8 data[WLBT_FAULT_DATA];
	} faults[WLBT_FAULT_RING];
	unsigned int fault_next;
	bool hip4_started;		/* peer took the service start request */
	struct wlbt_hip4_pool hip4_pool[2];
	unsigned int n_hip4_tx;
	unsigned int n_hip4_rx;
	struct wiphy *wiphy;
	struct net_device *netdev;
	struct cfg80211_scan_request *scan_req;
	struct delayed_work scan_work;
	struct delayed_work connect_work;
	struct work_struct peer_work;	/* answers the connect indications */
	u8 connect_bssid[ETH_ALEN];
	u16 scan_id;
	bool connecting;
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
	u16 host_tag;			/* names each transmitted frame */
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
};

static u32 wlbt_stream_read(struct exynos_wlbt *w, struct wlbt_stream *s,
			    void *dst, u32 bytes);
static bool wlbt_stream_write(struct exynos_wlbt *w, struct wlbt_stream *s,
			      const void *src, u32 bytes);
static void wlbt_log_drain(struct exynos_wlbt *w);
static bool wlbt_mgmt_send(struct exynos_wlbt *w, u8 channel, u8 ma_msg,
			   u32 arg);
static void wlbt_mgmt_drain(struct exynos_wlbt *w);
static bool wlbt_trace_start(struct exynos_wlbt *w);

static char *firmware_name = "mx140/mx140.bin";
module_param(firmware_name, charp, 0444);
MODULE_PARM_DESC(firmware_name, "Firmware image for the WLBT subsystem");

static bool load_firmware;
module_param(load_firmware, bool, 0644);
MODULE_PARM_DESC(load_firmware, "Copy the firmware image into MIF DRAM and let the peer run it");

static bool hip4_return = true;
module_param(hip4_return, bool, 0644);
MODULE_PARM_DESC(hip4_return, "Hand received HIP4 segments back to the peer");

/*
 * The receive queue is walked with the queue lock held and interrupts off, so
 * a frame by frame trace is only printed when it is asked for: at a few hundred
 * frames the console time alone is enough to lose the mailbox interrupts.
 */
/*
 * Data frames are sent on the data queue, which is what the class in the bulk
 * header says; the control queue is an experiment because the peer does not
 * consume the data queue yet.
 */
static bool hip4_log;
module_param(hip4_log, bool, 0644);
MODULE_PARM_DESC(hip4_log, "Record every received HIP4 frame for debugfs frames");

static bool load_mib = true;
module_param(load_mib, bool, 0644);
MODULE_PARM_DESC(load_mib, "Hand the peer its hardware configuration files (MIB)");

/*
 * The peer calibrates its PHY against the chip revision the host reports, and
 * the vendor passes the value from the chipid block (mxman.c:1095).  The
 * register lives at 0x10000000 with the revision in bits 23:16 for this
 * family (drv_data_exynos9 in the vendor chipid driver).
 */
#define WLBT_CHIPID_PHYS	0x10000000
#define WLBT_CHIPID_REV		0x10

static int soc_rev = -1;
module_param(soc_rev, int, 0644);
MODULE_PARM_DESC(soc_rev, "SoC revision reported to the peer (-1 reads the chipid block)");

static u32 wlbt_soc_revision(struct exynos_wlbt *w)
{
	void __iomem *id;
	u32 val;

	if (soc_rev >= 0)
		return soc_rev;

	id = ioremap(WLBT_CHIPID_PHYS, 0x100);
	if (!id)
		return 0;
	val = readl(id + WLBT_CHIPID_REV);
	iounmap(id);
	dev_info(w->dev, "SoC revision 0x%02x (chipid 0x%08x)\n",
		 (val >> 16) & 0xff, val);

	/*
	 * The vendor packs it as (main_rev << 4) | sub_rev out of bits 23:16
	 * (exynos-chipid.c:167, drv_data_exynos9).
	 */
	return ((val >> 16) & 0xff);
}

static unsigned int mib_slice_max = 1024;
module_param(mib_slice_max, uint, 0644);
MODULE_PARM_DESC(mib_slice_max, "Largest MIB batch handed over in one signal");

/*
 * The peer validates the reference in fw_reference against the HIP4 transmit
 * and receive windows only (0x802e44f0..0x802e44fc hold their bounds), so it
 * is a pointer into the segment memory rather than a cookie.  Send the
 * address of the segment the signal itself travels in.
 */
static bool sig_self_ref;
module_param(sig_self_ref, bool, 0644);
MODULE_PARM_DESC(sig_self_ref, "Point fw_reference at the signal's own segment");

static unsigned int send_ref;
static int sig_refcnt = -1;
module_param(send_ref, uint, 0644);

/* Name a plain zeroed buffer instead of a segment in the firmware reference. */
static bool hip4_ref_bare;

/* Override the mbulk reference count of frames sent on the datapath. */
module_param(sig_refcnt, int, 0644);
module_param(hip4_ref_bare, bool, 0644);
MODULE_PARM_DESC(send_ref, "fw_reference to put in signals sent from the hip4 node");

static bool mib_early;
module_param(mib_early, bool, 0644);
MODULE_PARM_DESC(mib_early, "Download the MIB entries before the WLAN start request");

static char *trace_boot;
module_param(trace_boot, charp, 0444);
MODULE_PARM_DESC(trace_boot, "Sample one peer word from the moment the firmware boots: <hex offset>:<hex field>:<microseconds>");

static const u32 wlbt_pw_regs[WLBT_PW_REGS] = {
	CENTRAL_SEQ_WLBT_CONFIGURATION,
	RESET_AHEAD_WLBT_SYS_PWR_REG,
	CLEANY_BUS_WLBT_SYS_PWR_REG,
	LOGIC_RESET_WLBT_SYS_PWR_REG,
	TCXO_GATE_WLBT_SYS_PWR_REG,
	WLBT_DISABLE_ISO_SYS_PWR_REG,
	WLBT_RESET_ISO_SYS_PWR_REG,
};

static void wlbt_pw_cfg_save(struct exynos_wlbt *w)
{
	unsigned int i;

	for (i = 0; i < WLBT_PW_REGS; i++) {
		w->pw_cfg[i].reg = wlbt_pw_regs[i];
		if (regmap_read(w->pmu, wlbt_pw_regs[i], &w->pw_cfg[i].val))
			w->pw_cfg[i].val = 0;
	}
}

static void wlbt_pw_cfg_restore(struct exynos_wlbt *w)
{
	unsigned int i;

	if (!w->pw_cfg_dirty)
		return;

	for (i = 0; i < WLBT_PW_REGS; i++) {
		if (regmap_write(w->pmu, w->pw_cfg[i].reg, w->pw_cfg[i].val))
			dev_err(w->dev, "cannot restore PMU 0x%x to 0x%x\n",
				w->pw_cfg[i].reg, w->pw_cfg[i].val);
	}
	w->pw_cfg_dirty = false;
	dev_dbg(w->dev, "restored the WLBT sequencer configuration\n");
}

static const char *wlbt_state_name(enum wlbt_state state)
{
	switch (state) {
	case WLBT_IN_RESET:
		return "in-reset";
	case WLBT_WAIT_CFG_REQ:
		return "wait-cfg-req";
	case WLBT_ACK_CFG_REQ:
		return "ack-cfg-req";
	case WLBT_CFG_DONE:
		return "cfg-done";
	case WLBT_CFG_ERROR:
		return "cfg-error";
	}

	return "?";
}

/*
 * Everything the vendor prints when the handshake goes wrong
 * (platform_mif_9610.c:715-733).  The state machine value is the useful one:
 * 0x80 means "WLBT is in reset", anything else means it is somewhere in its
 * boot sequence.
 */
static void wlbt_dump(struct exynos_wlbt *w, const char *why)
{
	u32 val;

	regmap_read(w->pmu, CENTRAL_SEQ_WLBT_STATUS, &val);
	dev_info(w->dev, "%s: CENTRAL_SEQ_WLBT_STATUS 0x%08x (state 0x%02x)\n",
		 why, val, (val & STATES) >> 16);
	regmap_read(w->pmu, WLBT_CTRL_NS, &val);
	dev_info(w->dev, "%s: WLBT_CTRL_NS 0x%08x\n", why, val);
	regmap_read(w->pmu, WLBT_CTRL_S, &val);
	dev_info(w->dev, "%s: WLBT_CTRL_S 0x%08x\n", why, val);
	regmap_read(w->pmu, WLBT_DEBUG, &val);
	dev_info(w->dev, "%s: WLBT_DEBUG 0x%08x\n", why, val);
	dev_info(w->dev, "%s: mailbox IS_VERSION 0x%08x INTSR0 0x%08x INTMSR0 0x%08x\n",
		 why, readl(w->mbox + MBOX_IS_VERSION),
		 readl(w->mbox + MBOX_INTSR0), readl(w->mbox + MBOX_INTMSR0));
}

/*
 * The WLBT TZPC is secure by default; the vendor makes it non-secure through
 * the EL3 monitor at the top of the CFG_REQ handler (platform_mif_9610.c:741).
 * The GNSS block does the same thing (DOWNSTREAM_READONLY/drivers/misc/
 * gnss_if/pmu-gnss.c:322), and our own GPU power domain driver proves that the
 * monitor is still there under uniLoader.
 */
static int wlbt_request_tzpc(struct exynos_wlbt *w)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CMD_CONN_IF, ((u64)EXYNOS_WLBT << 32) |
		      EXYNOS_SET_CONN_TZPC, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		dev_warn(w->dev, "TZPC SMC returned %ld\n", (long)res.a0);

	return res.a0;
}

/*
 * Reproduce the address space the peer expects: 16 MiB of DRAM at 0x80000000
 * over the data bus, and the peripheral windows on the peripheral bus
 * (platform_mif_9610.c:748-805).  Until this is done the peer cannot read its
 * own configuration, let alone firmware.
 */
static int wlbt_program_windows(struct exynos_wlbt *w)
{
	unsigned int i;
	u32 id;
	int ret;

	if (!w->mem_base || !w->mem_size) {
		dev_err(w->dev, "no MIF DRAM window\n");
		return -EINVAL;
	}
	if (w->mem_size < BAAW_0_END - BAAW_0_START) {
		dev_err(w->dev, "MIF DRAM too small: %pa size %pa\n",
			&w->mem_base, &w->mem_size);
		return -EINVAL;
	}

	ret = regmap_write(w->remap, REMAP_BAAW_0, BAAW_0_START >> 12);
	if (ret)
		return ret;

	/* Keep the version field, replace the EMA parameters. */
	ret = regmap_read(w->remap, REMAP_CHIP_VERSION_ID, &id);
	if (ret)
		return ret;
	id &= ~CHIP_VERSION_ID_EMA_MASK;
	id |= CHIP_VERSION_ID_EMA_VALUE;
	ret = regmap_write(w->remap, REMAP_CHIP_VERSION_ID, id);
	if (ret)
		return ret;

	ret = regmap_write(w->dbus_baaw, 0x00, BAAW_0_START >> 12);
	ret |= regmap_write(w->dbus_baaw, 0x04, BAAW_0_END >> 12);
	ret |= regmap_write(w->dbus_baaw, 0x08, w->mem_base >> 12);
	ret |= regmap_write(w->dbus_baaw, 0x0c, BAAW_ACCESS_CTRL);
	if (ret) {
		dev_err(w->dev, "DBUS_BAAW write failed: %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(wlbt_pbus_windows); i++) {
		const struct wlbt_pbus_window *win = &wlbt_pbus_windows[i];

		ret = regmap_write(w->pbus_baaw, i * 0x10 + 0x00, win->start >> 12);
		ret |= regmap_write(w->pbus_baaw, i * 0x10 + 0x04, win->end >> 12);
		ret |= regmap_write(w->pbus_baaw, i * 0x10 + 0x08, win->target >> 12);
		ret |= regmap_write(w->pbus_baaw, i * 0x10 + 0x0c, BAAW_ACCESS_CTRL);
		if (ret) {
			dev_err(w->dev, "PBUS_BAAW window %u failed: %d\n", i, ret);
			return ret;
		}
	}

	dev_info(w->dev, "windows programmed: MIF DRAM %pa at 0x%08x\n",
		 &w->mem_base, BAAW_0_START);

	return 0;
}

/*
 * Answer the peer's configuration request (platform_mif_9610.c:806-846): hold
 * it, write the ROM patch, release it, then ACK.  The peer may crash as soon as
 * the ACK lands - the vendor notes exactly that - so the state is moved first.
 */
static int wlbt_cfg_ack(struct exynos_wlbt *w)
{
	unsigned int i;
	int ret;

	ret = regmap_write(w->boot_cfg, BOOT_CFG_HOLD, 1);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(wlbt_rom_patch); i++) {
		ret = regmap_write(w->boot_cfg, BOOT_CFG_ROM + i * 4,
				   wlbt_rom_patch[i]);
		if (ret)
			return ret;
	}

	ret = regmap_write(w->boot_cfg, BOOT_CFG_HOLD, 0);
	if (ret)
		return ret;

	w->state = WLBT_ACK_CFG_REQ;

	ret = regmap_write(w->boot_cfg, BOOT_CFG_ACK, 1);
	if (ret)
		return ret;

	dev_info(w->dev, "BOOT_CFG_ACK done\n");

	/* The peer clears CFG_REQ in response to the ACK; give it a moment so
	 * the level interrupt is gone before it is enabled again.
	 */
	udelay(100);

	/* Hand the power request back to the firmware, as the vendor does. */
	ret = regmap_update_bits(w->pmu, WLBT_CTRL_NS, MASK_PWR_REQ, 0);
	if (ret)
		return ret;

	w->state = WLBT_CFG_DONE;

	return 0;
}

static irqreturn_t wlbt_cfg_req_isr(int irq, void *data)
{
	struct exynos_wlbt *w = data;
	int ret;

	disable_irq_nosync(irq);
	w->n_cfg_req++;

	/* A CFG_REQ out of nowhere means the previous shutdown left the peer
	 * running; the vendor refuses to touch the hardware after that.
	 */
	if (w->state != WLBT_WAIT_CFG_REQ) {
		w->n_cfg_bad++;
		dev_err(w->dev, "spurious CFG_REQ in state %s\n",
			wlbt_state_name(w->state));
		wlbt_dump(w, "spurious-cfg-req");
		w->state = WLBT_CFG_ERROR;
		complete(&w->cfg_ack);
		return IRQ_HANDLED;
	}

	dev_info(w->dev, "CFG_REQ received, configuring the peer\n");

	/* The CBUS should be ready when CFG_REQ arrives, but the vendor waits
	 * 100 us anyway (platform_mif_9610.c:737).
	 */
	udelay(100);

	wlbt_request_tzpc(w);

	ret = wlbt_program_windows(w);
	if (!ret)
		ret = wlbt_cfg_ack(w);
	if (ret) {
		dev_err(w->dev, "configuration failed: %d\n", ret);
		w->state = WLBT_CFG_ERROR;
		complete(&w->cfg_ack);
		return IRQ_HANDLED;
	}

	dev_info(w->dev, "peer configured, state %s\n",
		 wlbt_state_name(w->state));
	complete(&w->cfg_ack);

	return IRQ_HANDLED;
}

/*
 * The peer asks to be reset when its own watchdog fires
 * (platform_mif_9610.c:592-614).  Clearing the request keeps it alive; a full
 * reboot of the subsystem belongs to the transport driver that will sit on top.
 */
static irqreturn_t wlbt_wdog_isr(int irq, void *data)
{
	struct exynos_wlbt *w = data;
	int ret;

	/* During the config handshake the SoC can raise this spuriously; the
	 * vendor just consumes it (platform_mif_9610.c:591).
	 */
	if (w->state == WLBT_WAIT_CFG_REQ)
		return IRQ_HANDLED;

	w->n_wdog++;

	/* The peer holds the line until something resets it, so a level
	 * interrupt here storms the AP - which is exactly what the vendor
	 * guards against by disabling the line when no reset handler is
	 * registered.  One request is enough to know the peer is gone.
	 */
	disable_irq_nosync(irq);
	dev_err(w->dev, "peer watchdog reset request, line disabled\n");

	ret = regmap_update_bits(w->pmu, WLBT_CTRL_NS, WLBT_RESET_REQ_CLR,
				 WLBT_RESET_REQ_CLR);
	if (ret)
		dev_err(w->dev, "cannot clear the reset request: %d\n", ret);

	return IRQ_HANDLED;
}

/*
 * The management channel carries the infrastructure protocol (firmware version,
 * halt, panic notification, ...).  Without the message table ported yet, dump
 * what arrives so the first exchange can be read off the wire.
 */
/*
 * One management message: the channel byte, then a packed
 * struct ma_msg_packet { u8 ma_msg; u32 arg; } (mxman.c:547).
 */
static bool wlbt_mgmt_send(struct exynos_wlbt *w, u8 channel, u8 ma_msg, u32 arg)
{
	u8 frame[6] = { channel, ma_msg };
	__le32 le = cpu_to_le32(arg);

	memcpy(frame + 2, &le, sizeof(le));

	return wlbt_stream_write(w, &w->mgmt_out, frame, sizeof(frame));
}

static void wlbt_mgmt_drain(struct exynos_wlbt *w)
{
	char hex[3 * 32 + 1];
	u8 msg[MGMT_PACKET_SIZE];
	u32 got;
	int i;

	while ((got = wlbt_stream_read(w, &w->mgmt_in, msg, MGMT_PACKET_SIZE))) {
		w->n_mgmt++;
		memset(hex, 0, sizeof(hex));
		for (i = 1; i < got && i < 32; i++)
			snprintf(hex + (i - 1) * 3, 4, " %02x", msg[i]);
		dev_info(w->dev, "mgmt message: channel %u:%s\n", msg[0], hex);
	}
}

/*
 * Doorbell from the peer: one bit per stream, INTSR0[31:16].
 */
static void wlbt_hip4_drain(struct exynos_wlbt *w);
static void wlbt_dram_bsses(struct exynos_wlbt *w, struct seq_file *s);

static irqreturn_t wlbt_mbox_isr(int irq, void *data)
{
	struct exynos_wlbt *w = data;
	u32 status;

	/* INTMSR0 is the pending set after the mask, which is what the vendor
	 * reads too (platform_mif_9610.c:1334).
	 */
	status = readl(w->mbox + MBOX_INTMSR0) >> 16;
	w->n_mbox++;
	w->last_mbox_status = status;

	if (!status) {
		dev_warn(w->dev, "mailbox IRQ without a status bit\n");
		return IRQ_HANDLED;
	}

	/* Acknowledge before reading: any message that lands in the meantime
	 * raises the bit again and the streams are drained until empty anyway.
	 */
	writel(status << 16, w->mbox + MBOX_INTCR0);

	if (status & BIT(w->log.write_bit))
		wlbt_log_drain(w);
	if (status & BIT(w->mgmt_in.write_bit))
		wlbt_mgmt_drain(w);
	if (status & BIT(WLBT_BIT_HIP4_TO_HOST))
		wlbt_hip4_drain(w);

	return IRQ_HANDLED;
}

/*
 * Copy the firmware into the MIF DRAM window the peer boots from.  The peer
 * reads it over the data bus, which does not snoop our caches, so the mapping
 * has to be write-combining like the vendor's platform_mif_map_region(); a
 * cached copy would leave the peer with stale lines.
 *
 * The image is placed at offset 0 and the peer is entered at the entry point in
 * its header.  Everything past the image stays zeroed so the firmware's BSS is
 * clean, which is what the vendor gets from freshly allocated DMA memory.
 */
static int wlbt_load_firmware(struct exynos_wlbt *w)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, firmware_name, w->dev);
	if (ret) {
		dev_warn(w->dev, "firmware %s not available: %d\n",
			 firmware_name, ret);
		return ret;
	}

	if (fw->size < FW_HDR_MIN_SIZE ||
	    memcmp(fw->data + FW_HDR_MAGIC_OFFSET, "smxf", 4)) {
		dev_err(w->dev, "%s is not a Maxwell firmware image\n",
			firmware_name);
		ret = -EINVAL;
		goto out;
	}

	w->fw_entry = get_unaligned_le32(fw->data + FW_HDR_ENTRY_OFFSET);
	w->fw_runtime_length =
		get_unaligned_le32(fw->data + FW_HDR_RUNTIME_LENGTH_OFFSET);

	if (w->fw_runtime_length > w->mem_size) {
		dev_err(w->dev, "firmware needs %u bytes, the window is %pa\n",
			w->fw_runtime_length, &w->mem_size);
		ret = -ENOSPC;
		goto out;
	}

	w->fw_size = fw->size;
	if (fw->size >= FW_HDR_PANIC_OFFSET + 4)
		w->fw_panic_offset =
			get_unaligned_le32(fw->data + FW_HDR_PANIC_OFFSET);

	if (!w->dram) {
		w->dram = memremap(w->mem_base, w->mem_size, MEMREMAP_WC);
		if (!w->dram) {
			ret = -ENOMEM;
			goto out;
		}
	}

	memset_io(w->dram, 0, w->mem_size);
	memcpy_toio(w->dram, fw->data, fw->size);
	/* The peer reads the image over the bus; drain the writes first. */
	wmb();

	w->fw_loaded = true;
	ret = 0;

	dev_info(w->dev, "firmware loaded: %zu bytes, runtime %u, entry 0x%x\n",
		 fw->size, w->fw_runtime_length, w->fw_entry);
	dev_info(w->dev, "firmware build: %.*s\n", FW_HDR_BUILD_ID_MAX,
		 fw->data + FW_HDR_BUILD_ID_OFFSET);
out:
	release_firmware(fw);

	return ret;
}

/*
 * Hand the peer its start parameters (mxman.c:1155-1183).
 */
static void wlbt_mbox_init(struct exynos_wlbt *w)
{
	writel(w->fw_entry, w->mbox + MBOX_ISSR_BASE + MBOX_ISSR_ENTRY * 4);
	writel(w->mxconf_offset, w->mbox + MBOX_ISSR_BASE + MBOX_ISSR_CONFIG * 4);
	writel(MBOX_START_MAGIC, w->mbox + MBOX_ISSR_BASE + MBOX_ISSR_MAGIC * 4);
	writel(0, w->mbox + MBOX_ISSR_BASE + MBOX_ISSR_FLAGS * 4);
	/* The peer reads these through the bus; make sure they are there. */
	wmb();
}

/*
 * The MIF DRAM is split the way the vendor splits it (mxman.c:1494): the
 * firmware image occupies [0, fw_runtime_length), and everything else - heaps,
 * streams, configuration - lives above it.  A bump allocator is all that is
 * needed, because nothing is ever freed while the peer is running.
 */
/*
 * miframman.c allocates size + align + sizeof(void *) per block and parks a
 * back pointer just below the aligned block, with the alignment clamped up to
 * sizeof(void *).  Spending the same slack puts every stream at the address
 * the vendor's own driver would have picked, which is one less difference to
 * explain when the peer refuses to read one of them.
 */
static u32 wlbt_heap_alloc(struct exynos_wlbt *w, u32 bytes)
{
	const u32 align = sizeof(void *);
	u32 offset, end;

	end = w->heap_next + bytes + align + align;
	if (!w->heap_next || end > w->mem_size) {
		dev_err(w->dev, "MIF heap is full (%u of %pa used)\n",
			w->heap_next, &w->mem_size);
		return 0;
	}

	offset = ALIGN(w->heap_next + align + align, align);
	w->heap_next = end;
	memset_io(w->dram + offset, 0, bytes);

	return offset;
}

/*
 * Same, but honouring a larger alignment.  HIP4 wants its control block on a
 * 4 KiB boundary (hip4.h), which is the only caller of this.
 */
static u32 wlbt_heap_alloc_aligned(struct exynos_wlbt *w, u32 bytes, u32 align)
{
	u32 offset, end;

	if (align < sizeof(void *))
		align = sizeof(void *);

	end = w->heap_next + bytes + align + align;
	if (!w->heap_next || end > w->mem_size) {
		dev_err(w->dev, "MIF heap is full (%u of %pa used)\n",
			w->heap_next, &w->mem_size);
		return 0;
	}

	offset = ALIGN(w->heap_next + align, align);
	w->heap_next = end;
	memset_io(w->dram + offset, 0, bytes);

	return offset;
}

/*
 * Copy the hardware configuration files (HCF) into the HIP4 MIB area.  The
 * vendor concatenates them there and hands the firmware the address; without
 * them the WLAN service has no RF/driver parameters and its start never
 * completes (hip4.c:2416-2455).
 */
/*
 * The MIB files only carry the entries above PSID 0x08cf.  The golden, host owned
 * ones below that range are not in any of the shipped variants, and the peer will
 * not join a network without them: its join builds the basic rate set of the
 * association from SLSI_PSID_UNIFI_SUPPORTED_DATA_RATES and refuses the request
 * with fault 0x2c1f - reported as 0x0030 NOT_ALLOWED - when that set is empty.
 *
 * The rate list is N followed by N rates in units of 500 kbit/s, which is the
 * 802.11b/g set: 1, 2, 5.5 and 11 Mbit/s then 6, 9, 12, 18, 24, 36, 48 and 54.
 */
#define MIB_PSID_SUPPORTED_DATA_RATES	0x07f9
#define MIB_PSID_EXTENDED_CAPABILITIES	0x07ef

static const u8 wlbt_golden_rates[] = {
	12, 0x02, 0x04, 0x0b, 0x16, 0x0c, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60, 0x6c,
};

/* The vendor driver advertises these extended capabilities and clears the rest. */
static const u8 wlbt_golden_ext_caps[] = {
	9, 0xff, 0xff, 0xf7, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff,
};

static const struct {
	u16 psid;
	const u8 *value;
	size_t len;
} wlbt_golden_mibs[] = {
	{ MIB_PSID_SUPPORTED_DATA_RATES, wlbt_golden_rates,
	  sizeof(wlbt_golden_rates) },
	{ MIB_PSID_EXTENDED_CAPABILITIES, wlbt_golden_ext_caps,
	  sizeof(wlbt_golden_ext_caps) },
};

static u32 wlbt_load_mib(struct exynos_wlbt *w)
{
	static const char *const names[] = {
		"mx140/mx140_wlan.hcf",
		"mx140/mx140_wlan_sw.hcf",
	};
	u32 total = 0;
	size_t skip;
	unsigned int i;

	/* Clear the area so the peer never picks up a stale MIB. */
	memset_io(w->dram + w->hip4_ref + HIP4_MIB_OFFSET, 0, HIP4_MIB_SIZE);

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		const struct firmware *fw;

		if (request_firmware(&fw, names[i], w->dev)) {
			dev_warn(w->dev, "MIB %s not available\n", names[i]);
			continue;
		}
		/*
		 * The files carry an eight byte prologue (a hash, a checksum
		 * and a format version) in front of the entry stream, but the
		 * peer is handed the start of the stream: its walker reads
		 * {u16 id, u16 len} from the first byte it is pointed at and
		 * would otherwise take the prologue for an entry with a bogus
		 * length and assert.
		 */
		skip = 0;
		if (fw->size > MIB_PROLOGUE_SIZE &&
		    get_unaligned_le16(fw->data + 6) == MIB_PROLOGUE_MARKER)
			skip = MIB_PROLOGUE_SIZE;

		if (total + fw->size - skip > HIP4_MIB_SIZE) {
			dev_warn(w->dev, "MIB %s does not fit (%u+%zu > %u)\n",
				 names[i], total, fw->size - skip,
				 HIP4_MIB_SIZE);
			release_firmware(fw);
			break;
		}

		memcpy_toio(w->dram + w->hip4_ref + HIP4_MIB_OFFSET + total,
			    fw->data + skip, fw->size - skip);
		total += fw->size - skip;
		release_firmware(fw);
	}

	/*
	 * The peer reads this area when it is started and caches what it finds,
	 * so the golden entries have to be here and not only in a later request.
	 */
	for (i = 0; i < ARRAY_SIZE(wlbt_golden_mibs); i++) {
		size_t len = 4 + wlbt_golden_mibs[i].len;
		size_t padded = ALIGN(len, 2);
		u8 rec[4 + 16];

		if (len > sizeof(rec) || total + padded > HIP4_MIB_SIZE)
			break;

		put_unaligned_le16(wlbt_golden_mibs[i].psid, rec);
		put_unaligned_le16(wlbt_golden_mibs[i].len, rec + 2);
		memcpy(rec + 4, wlbt_golden_mibs[i].value, wlbt_golden_mibs[i].len);
		memset(rec + len, 0, padded - len);
		memcpy_toio(w->dram + w->hip4_ref + HIP4_MIB_OFFSET + total,
			    rec, padded);
		total += padded;
	}

	if (total)
		dev_info(w->dev, "MIB in the HIP4 area: %u bytes at 0x%x\n",
			 total, w->hip4_ref + HIP4_MIB_OFFSET);

	return total;
}

/*
 * Build the HIP4 shared region the WLAN service is started with.  The vendor
 * allocates 2.5 MiB, drops a hip4_hip_control at the base and points the peer at
 * it through the optional_data field of SM_MSG_START_REQ (hip4.c:2263-2560).
 *
 * The peer walks init.version_a_ref / version_b_ref to the two configuration
 * blocks, so both versions are filled in even though this firmware build will
 * pick one of them.  No MIB and no logging block are supplied: those are the
 * hardware configuration files, and zero means "use what you were built with".
 */
static int wlbt_hip4_build(struct exynos_wlbt *w)
{
	struct wlbt_hip4_control *c;
	u32 base, i, mib;

	if (w->hip4_control)
		return 0;

	base = wlbt_heap_alloc_aligned(w, HIP4_ALLOC_SIZE, 4096);
	if (!base)
		return -ENOMEM;

	w->hip4_ref = base;
	w->hip4_control = base + HIP4_CONFIG_OFFSET;
	c = (struct wlbt_hip4_control *)(w->dram + w->hip4_control);

	/* Scoreboard and queue arrays first: their refs go into both blocks. */
	c->init.magic = cpu_to_le32(HIP4_INIT_MAGIC);
	c->init.conf_ver = 0;
	c->init.version_a_ref = cpu_to_le32(w->hip4_control +
					     offsetof(struct wlbt_hip4_control, v4));
	c->init.version_b_ref = cpu_to_le32(w->hip4_control +
					     offsetof(struct wlbt_hip4_control, v5));

	c->v4.magic = cpu_to_le32(HIP4_CONFIG_MAGIC);
	c->v4.ver = cpu_to_le16(4);
	c->v4.len = cpu_to_le16(sizeof(c->v4));
	c->v4.host_cache_line = 64;
	c->v4.host_buf_loc = cpu_to_le32(w->hip4_ref + HIP4_TX_OFFSET);
	c->v4.host_buf_sz = cpu_to_le32(HIP4_TX_SIZE);
	c->v4.fw_buf_loc = cpu_to_le32(w->hip4_ref + HIP4_RX_OFFSET);
	c->v4.fw_buf_sz = cpu_to_le32(HIP4_RX_SIZE);
	c->v4.mif_fh_int_n = WLBT_BIT_HIP4_FROM_HOST;
	c->v4.scbrd_loc = cpu_to_le32(w->hip4_control +
				      offsetof(struct wlbt_hip4_control, scoreboard));
	c->v4.q_num = cpu_to_le16(HIP4_Q_NUM);
	c->v4.q_len = cpu_to_le16(HIP4_Q_LEN);
	c->v4.q_idx_sz = cpu_to_le16(HIP4_Q_IDX_SIZE);

	c->v5.magic = cpu_to_le32(HIP4_CONFIG_MAGIC);
	c->v5.ver = cpu_to_le16(5);
	c->v5.len = cpu_to_le16(sizeof(c->v5));
	c->v5.host_cache_line = 64;
	c->v5.host_buf_loc = cpu_to_le32(w->hip4_ref + HIP4_TX_OFFSET);
	c->v5.host_buf_sz = cpu_to_le32(HIP4_TX_SIZE);
	c->v5.fw_buf_loc = cpu_to_le32(w->hip4_ref + HIP4_RX_OFFSET);
	c->v5.fw_buf_sz = cpu_to_le32(HIP4_RX_SIZE);
	c->v5.mif_fh_int_n = WLBT_BIT_HIP4_FROM_HOST;
	c->v5.mif_th_int_n = WLBT_BIT_HIP4_TO_HOST;
	c->v5.scbrd_loc = cpu_to_le32(w->hip4_control +
				      offsetof(struct wlbt_hip4_control, scoreboard));
	c->v5.q_num = cpu_to_le16(HIP4_Q_NUM);
	c->v5.q_len = cpu_to_le16(HIP4_Q_LEN);
	c->v5.q_idx_sz = cpu_to_le16(HIP4_Q_IDX_SIZE);

	for (i = 0; i < HIP4_Q_NUM; i++) {
		u32 q = cpu_to_le32(w->hip4_control +
				    offsetof(struct wlbt_hip4_control, q) +
				    i * sizeof(struct wlbt_hip4_q) +
				    offsetof(struct wlbt_hip4_q, array));

		c->v4.q_loc[i] = q;
		c->v5.q_loc[i] = q;
		c->v4.mif_th_int_n[i] = WLBT_BIT_HIP4_TO_HOST;
	}

	mib = load_mib ? wlbt_load_mib(w) : 0;
	if (mib) {
		c->v4.mib_loc = cpu_to_le32(w->hip4_ref + HIP4_MIB_OFFSET);
		c->v4.mib_sz = cpu_to_le32(mib);
		c->v5.mib_loc = cpu_to_le32(w->hip4_ref + HIP4_MIB_OFFSET);
		c->v5.mib_sz = cpu_to_le32(mib);
	}

	/* The peer reads the block over the bus as soon as the request lands. */
	wmb();

	/* Every segment is free until the peer hands one back on a release
	 * queue, so start with an empty scoreboard and an empty bitmap.
	 */
	w->hip4_pool[0].base = w->hip4_ref + HIP4_TX_OFFSET;
	w->hip4_pool[0].slots = HIP4_DAT_SLOTS;
	w->hip4_pool[0].next = 0;
	bitmap_zero(w->hip4_pool[0].used, HIP4_MAX_SLOTS);
	w->hip4_pool[1].base = w->hip4_ref + HIP4_TX_CTL_OFFSET;
	w->hip4_pool[1].slots = HIP4_CTL_SLOTS;
	w->hip4_pool[1].next = 0;
	bitmap_zero(w->hip4_pool[1].used, HIP4_MAX_SLOTS);

	memset_io(w->dram + w->hip4_control +
		  offsetof(struct wlbt_hip4_control, scoreboard), 0,
		  sizeof(__le32) * 256 +
		  HIP4_Q_NUM * sizeof(struct wlbt_hip4_q));
	/* Empty queues and a zero scoreboard before the peer starts using them. */
	wmb();

	dev_info(w->dev, "HIP4 at MIF+0x%x: control 0x%x, TX 0x%x, RX 0x%x, %u KiB\n",
		 w->hip4_ref, w->hip4_control, w->hip4_ref + HIP4_TX_OFFSET,
		 w->hip4_ref + HIP4_RX_OFFSET, HIP4_ALLOC_SIZE / 1024);

	return 0;
}

static u8 *wlbt_hip4_scb(struct exynos_wlbt *w)
{
	return w->dram + w->hip4_control +
	       offsetof(struct wlbt_hip4_control, scoreboard);
}

static u32 *wlbt_hip4_q_slot(struct exynos_wlbt *w, u32 q, u8 idx)
{
	return w->dram + w->hip4_control +
	       offsetof(struct wlbt_hip4_control, q) +
	       q * sizeof(struct wlbt_hip4_q) + idx * sizeof(__le32);
}

/*
 * A queue entry is free once the peer hands the segment back on the release
 * queue, so the slots are tracked with a bitmap rather than a cursor.
 */
static int wlbt_hip4_slot_get(struct exynos_wlbt *w, bool ctrl, u32 *ref)
{
	struct wlbt_hip4_pool *p = &w->hip4_pool[ctrl ? 1 : 0];
	unsigned int i;

	for (i = 0; i < p->slots; i++) {
		u32 slot = p->next;

		p->next = (p->next + 1) & (p->slots - 1);
		if (!test_and_set_bit(slot, p->used)) {
			*ref = p->base + slot * HIP4_SLOT_SIZE;
			return 0;
		}
	}

	return -ENOSPC;
}

static void wlbt_hip4_slot_put(struct exynos_wlbt *w, u32 ref)
{
	struct wlbt_hip4_pool *p;
	u32 off;

	for (off = 0; off < ARRAY_SIZE(w->hip4_pool); off++) {
		p = &w->hip4_pool[off];
		if (ref < p->base || ref >= p->base + p->slots * HIP4_SLOT_SIZE)
			continue;
		clear_bit((ref - p->base) / HIP4_SLOT_SIZE, p->used);
		return;
	}
}

/*
 * The peer checks every reference it is handed against the two buffers named in
 * the HIP4 configuration, and one that falls outside them is an assertion on its
 * side, so a segment that is not plainly inside those buffers is never queued.
 */
static bool wlbt_hip4_ref_valid(struct exynos_wlbt *w, u32 ref)
{
	return ref >= w->hip4_ref + HIP4_TX_OFFSET &&
	       ref < w->hip4_ref + HIP4_RX_OFFSET + HIP4_RX_SIZE;
}

static void wlbt_hip4_push(struct exynos_wlbt *w, u32 q, u32 ref)
{
	u8 *scb = wlbt_hip4_scb(w);
	u8 idx = readb(scb + wlbt_hip4_idx[q][0]);

	writel(ref, wlbt_hip4_q_slot(w, q, idx));
	/* The entry has to be visible before the index publishes it. */
	wmb();
	writeb((idx + 1) & (HIP4_Q_LEN - 1), scb + wlbt_hip4_idx[q][0]);
	/* Publish the index before the doorbell the peer reads. */
	wmb();

	writel(BIT(WLBT_BIT_HIP4_FROM_HOST), w->mbox + MBOX_INTGR1);
}

/*
 * Show the bookkeeping of every queue: the host half of the index pair and the
 * mirror the peer maintains.  When the peer wedges on a segment reference this
 * is what says whether it ran ahead, fell behind, or read a slot nobody wrote.
 */
static void wlbt_hip4_state(struct exynos_wlbt *w)
{
	static const char *const names[HIP4_Q_NUM] = {
		"FH_CTL", "FH_DAT", "FH_RFB", "TH_CTL", "TH_DAT", "TH_RFB",
	};
	u8 *scb = wlbt_hip4_scb(w);
	unsigned int q;

	for (q = 0; q < HIP4_Q_NUM; q++) {
		u8 hw = readb(scb + wlbt_hip4_idx[q][0]);
		u8 hr = readb(scb + wlbt_hip4_idx[q][1]);
		u32 next = hw & (HIP4_Q_LEN - 1);

		dev_info(w->dev, "HIP4 %-6s host w %3u r %3u  next ref 0x%x\n",
			 names[q], hw, hr,
			 readl(wlbt_hip4_q_slot(w, q, next)));
	}
}

/*
 * The peer's scan validator rejects a request whose scan id is below 0x100
 * when the sender id carries the 0xc000 bits, which ours does, so scan ids
 * start at 0x100 and the upper byte names the interface - the same encoding
 * the vendor driver uses.  The information elements travel as payload: a
 * wildcard SSID, the scan timing element and the channel list, in that order.
 */
static int wlbt_hip4_send(struct exynos_wlbt *w, bool ctrl, const void *sig,
			  size_t sig_len, const void *data, size_t data_len);

static int wlbt_mlme_add_scan(struct exynos_wlbt *w, u16 scan_id)
{
	static const u8 scan_mac[6] = { 0x8c, 0x77, 0x12, 0xaa, 0xbb, 0xcc };
	static const u16 channels[] = { 2412, 2437, 2462 };
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 26];
	u8 dat[2 + 19 + 7 + 3 * ARRAY_SIZE(channels)];
	struct wlbt_fapi_hdr hdr = { };
	unsigned int i, n = 0;
	u8 *c;

	/* Body: interface 0 for a scan, because a scan is not tied to a vif. */
	put_unaligned_le16(0, sig + sizeof(hdr) + 0);
	put_unaligned_le16(scan_id, sig + sizeof(hdr) + 2);
	put_unaligned_le16(2, sig + sizeof(hdr) + 4);	/* full scan */
	memcpy(sig + sizeof(hdr) + 6, scan_mac, sizeof(scan_mac));
	put_unaligned_le16(4, sig + sizeof(hdr) + 12);	/* real time results */
	memset(sig + sizeof(hdr) + 14, 0, 12);

	/* Wildcard SSID: a broadcast scan always carries one. */
	dat[n++] = 0;
	dat[n++] = 0;

	/* Scan timing element, 19 bytes, body left at zero for a plain scan. */
	dat[n++] = 0xdd;
	dat[n++] = 0x11;
	dat[n++] = 0x00;
	dat[n++] = 0x16;
	dat[n++] = 0x32;
	dat[n++] = 0x01;
	dat[n++] = 0x01;
	memset(dat + n, 0, 12);
	n += 12;

	/* Channel list: the length byte starts at five and grows by three. */
	c = dat + n;
	c[0] = 0xdd;
	c[1] = 5;
	c[2] = 0x00;
	c[3] = 0x16;
	c[4] = 0x32;
	c[5] = 0x01;
	c[6] = 0x02;
	n += 7;
	for (i = 0; i < ARRAY_SIZE(channels); i++) {
		put_unaligned_le16(channels[i] * 2, dat + n);
		dat[n + 2] = 0;			/* active */
		n += 3;
		c[1] += 3;
	}

	hdr.id = cpu_to_le16(FAPI_MLME_ADD_SCAN_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(hdr) + 26, dat, n);
}

/*
 * A throwaway segment for the firmware reference to name.  It carries nothing
 * but a signal header, has one reference on it, and sits in the last data slot
 * so the ordinary allocator never hands it out.
 */
static u32 wlbt_hip4_scratch(struct exynos_wlbt *w, const void *data,
			     size_t data_len)
{
	/*
	 * The peer frees whatever the reference names, so each request gets its
	 * own segment from the tail of the data pool rather than all of them
	 * naming one buffer that is freed and then freed again.
	 */
	u32 slot = HIP4_DAT_SLOTS - 1 - w->hip4_scratch_seq % HIP4_SCRATCH_SLOTS;
	u32 ref = w->hip4_ref + HIP4_TX_OFFSET + slot * HIP4_SLOT_SIZE;
	struct wlbt_mbulk mb = { };
	void *m = w->dram + ref;

	if (hip4_ref_bare) {
		/*
		 * A plain zeroed buffer: the peer's range check skips a
		 * segment whose signal size is zero, and nothing else in the
		 * request points at the reference.
		 */
		memset_io(m, 0, HIP4_SLOT_SIZE);
	} else {
		mb.flag = MBULK_F_SIG | MBULK_F_OBOUND;
		mb.clas = MBULK_CLASS_FROM_HOST_DAT;
		mb.pid = MBULK_POOL_ID_DATA;
		mb.refcnt = 1;
		mb.sig_bufsz = cpu_to_le16(ALIGN(HIP4_SIG_RESERVED +
						 sizeof(struct wlbt_fapi_hdr), 4));
		mb.head = mb.sig_bufsz;
		/*
		 * The peer parses this segment: it reads a length and walks the
		 * contents, and a segment with no data parses to nothing, which
		 * it rejects.  Carry the payload the request carries so there is
		 * something to walk.
		 */
		if (data_len && data_len <= HIP4_SLOT_SIZE - HIP4_SIG_RESERVED -
						 sizeof(struct wlbt_fapi_hdr)) {
			mb.len = cpu_to_le16(data_len);
			mb.dat_bufsz = cpu_to_le16(data_len);
		}
		memset_io(m, 0, HIP4_SLOT_SIZE);
		memcpy_toio(m, &mb, sizeof(mb));
		if (le16_to_cpu(mb.len))
			memcpy_toio(m + HIP4_MBULK_SIZE + le16_to_cpu(mb.head),
				    data, data_len);
	}
	/* The descriptor must be in DRAM before the peer is handed the ref. */
	wmb();

	w->hip4_scratch_seq++;

	return ref;
}

/*
 * The peer owns two words of the configuration it is given: how much room it
 * wants reserved in front of and behind the frame of a unitdata request.  It
 * builds the 802.11 header there, so the reservation only matters for frames
 * that reach the air - a scan or a join carries information elements and works
 * without it, which is why this went unnoticed until a station transmitted.
 */
static void wlbt_hip4_read_limits(struct exynos_wlbt *w)
{
	struct wlbt_hip4_control *c;

	if (!w->hip4_control)
		return;

	c = (struct wlbt_hip4_control *)(w->dram + w->hip4_control);
	if (le32_to_cpu(c->init.conf_ver) == 5) {
		w->tx_headroom = c->v5.unidat_req_headroom;
		w->tx_tailroom = c->v5.unidat_req_tailroom;
	} else {
		w->tx_headroom = c->v4.unidat_req_headroom;
		w->tx_tailroom = c->v4.unidat_req_tailroom;
	}

	dev_info(w->dev, "HIP4 unitdata limits: %u byte headroom, %u byte tailroom\n",
		 w->tx_headroom, w->tx_tailroom);
}

static int wlbt_hip4_send(struct exynos_wlbt *w, bool ctrl, const void *sig,
			  size_t sig_len, const void *data, size_t data_len)
{
	struct wlbt_mbulk mb = { };
	u32 sig_bufsz, dat_bufsz, head, ref;
	unsigned long flags;
	void *m;
	int ret;

	if (!w->hip4_control)
		return -ENODEV;
	if (sig_len < sizeof(struct wlbt_fapi_hdr))
		return -EINVAL;

	/* The pools are shared with the drain that runs in interrupt context. */
	spin_lock_irqsave(&w->hip4_lock, flags);
	ret = wlbt_hip4_slot_get(w, ctrl, &ref);
	if (ret) {
		spin_unlock_irqrestore(&w->hip4_lock, flags);
		return ret;
	}

	sig_bufsz = ALIGN(sig_len + HIP4_SIG_RESERVED, 4);
	/*
	 * A frame the peer will put on the air needs room for the header it
	 * builds in front of the payload, and for whatever it appends behind it.
	 * The reservation is expressed the way the peer's own allocator does:
	 * the data buffer covers headroom, payload and tailroom, and the head
	 * tells it where the payload starts.
	 */
	head = sig_bufsz;
	dat_bufsz = data_len;
	if (!ctrl && data_len) {
		head = sig_bufsz + w->tx_headroom;
		dat_bufsz = w->tx_headroom + data_len + w->tx_tailroom;
	}
	if (HIP4_MBULK_SIZE + head + data_len > HIP4_SLOT_SIZE) {
		spin_unlock_irqrestore(&w->hip4_lock, flags);
		return -ENOSPC;
	}

	mb.flag = MBULK_F_SIG | MBULK_F_OBOUND;
	mb.clas = ctrl ? MBULK_CLASS_FROM_HOST_CTL : MBULK_CLASS_FROM_HOST_DAT;
	mb.pid = ctrl ? MBULK_POOL_ID_CTRL : MBULK_POOL_ID_DATA;
	/*
	 * The peer's allocator ties the reference count to the data buffer,
	 * but its release path asserts when it finds zero, so the right value
	 * is not obvious from either side alone.  -1 keeps the allocator rule,
	 * anything else is written as given.
	 */
	mb.refcnt = sig_refcnt < 0 ? (data_len ? 1 : 0) : sig_refcnt;
	mb.dat_bufsz = cpu_to_le16(dat_bufsz);
	mb.sig_bufsz = cpu_to_le16(sig_bufsz);
	mb.len = cpu_to_le16(data_len);
	mb.head = cpu_to_le16(head);

	m = w->dram + ref;
	memcpy_toio(m, &mb, sizeof(mb));
	memset_io(m + HIP4_MBULK_SIZE, 0, HIP4_SIG_RESERVED);
	memcpy_toio(m + HIP4_MBULK_SIZE + HIP4_SIG_RESERVED, sig, sig_len);
	if (data_len)
		memcpy_toio(m + HIP4_MBULK_SIZE + head, data, data_len);
	if (sig_self_ref) {
		/*
		 * The peer validates this word against the transmit and receive
		 * bounds it kept from the HIP4 configuration, and then treats the
		 * segment it names as one of its own: it is handed back to the
		 * pool when the request is done.  So it has to name a disposable
		 * segment with a live reference count, not the frame the signal
		 * travels in - freeing that one leaves the frame double freed.
		 */
		__le32 self = cpu_to_le32(wlbt_hip4_scratch(w, data, data_len));

		memcpy_toio(m + HIP4_MBULK_SIZE + HIP4_SIG_RESERVED +
			    offsetof(struct wlbt_fapi_hdr, fw_reference),
			    &self, sizeof(self));
	}
	/* The whole segment has to be in DRAM before the peer sees the ref. */
	wmb();

	wlbt_hip4_push(w, ctrl ? HIP4_Q_FH_CTL : HIP4_Q_FH_DAT, ref);
	w->n_hip4_tx++;
	spin_unlock_irqrestore(&w->hip4_lock, flags);

	return 0;
}

/*
 * The radio parameters live in the peer's MIB database, not in the buffers
 * above: service start only tells it where the hardware configuration files
 * are.  slsi_start() follows the service start with one MLME_SET_REQ per
 * slice of those files, leaving out the entries the firmware owns (PSID bit
 * 15) and keeping every signal inside the hostio limit (mgt.c:1679-1712).
 * A confirm carrying data means the peer rejected those PSIDs.
 */
#define FAPI_MLME_SET_REQ	0x2002
#define MIB_PSID_FW_ONLY	0x8000
#define MIB_SLICE_MAX		1800

static void wlbt_mib_send_slice(struct exynos_wlbt *w, const u8 *mib,
				size_t len)
{
	struct wlbt_fapi_hdr hdr = { };
	u8 *buf;
	int ret;

	buf = kmalloc(sizeof(hdr) + 14 + len, GFP_KERNEL);
	if (!buf)
		return;

	hdr.id = cpu_to_le16(FAPI_MLME_SET_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(buf, &hdr, sizeof(hdr));
	memset(buf + sizeof(hdr), 0, 14);	/* vif and three spares */
	memcpy(buf + sizeof(hdr) + 14, mib, len);

	ret = wlbt_hip4_send(w, true, buf, sizeof(hdr) + 14 + len, NULL, 0);
	if (ret)
		dev_err(w->dev, "MIB slice of %zu bytes rejected: %d\n",
			len, ret);
	kfree(buf);

	/*
	 * The peer applies a slice asynchronously, so give it room to finish
	 * and to hand the segment back before the next one is queued: the
	 * vendor waits for the MLME_SET confirm here (mgt.c:1701).
	 */
	msleep(100);
	wlbt_hip4_drain(w);
}

/*
 * Walk the entry stream {u16 id, u16 len, data[len]} (padded to even) and hand
 * it over in slices the peer can take in one signal.
 */
static int wlbt_download_mibs(struct exynos_wlbt *w)
{
	static const char *const names[] = {
		"mx140/mx140_wlan.hcf",
		"mx140/mx140_wlan_sw.hcf",
	};
	unsigned int slices = 0, entries = 0, skipped = 0;
	u8 *slice;
	size_t slice_len = 0;
	unsigned int i;

	if (mib_slice_max > MIB_SLICE_MAX)
		mib_slice_max = MIB_SLICE_MAX;
	slice = kmalloc(MIB_SLICE_MAX, GFP_KERNEL);
	if (!slice)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		const struct firmware *fw;
		size_t off, skip;

		if (request_firmware(&fw, names[i], w->dev)) {
			dev_warn(w->dev, "MIB %s not available\n", names[i]);
			continue;
		}

		skip = 0;
		if (fw->size > MIB_PROLOGUE_SIZE &&
		    get_unaligned_le16(fw->data + 6) == MIB_PROLOGUE_MARKER)
			skip = MIB_PROLOGUE_SIZE;

		for (off = skip; off + 4 <= fw->size;) {
			u16 psid = get_unaligned_le16(fw->data + off);
			size_t len = 4 + get_unaligned_le16(fw->data + off + 2);
			size_t padded = ALIGN(len, 2);

			if (off + padded > fw->size)
				break;

			if (psid & MIB_PSID_FW_ONLY) {
				skipped++;
				off += padded;
				continue;
			}

			if (slice_len + padded > mib_slice_max) {
				wlbt_mib_send_slice(w, slice, slice_len);
				slices++;
				slice_len = 0;
			}

			memcpy(slice + slice_len, fw->data + off, padded);
			slice_len += padded;
			entries++;
			off += padded;
		}

		release_firmware(fw);

		/* Each file ends its own slice, as the vendor does per file. */
		if (slice_len) {
			dev_info(w->dev, "MIB slice %u: %zu bytes\n",
				 slices, slice_len);
			wlbt_mib_send_slice(w, slice, slice_len);
			slices++;
			slice_len = 0;
		}
	}

	/* The golden host owned entries the firmware files do not carry. */
	for (i = 0; i < ARRAY_SIZE(wlbt_golden_mibs); i++) {
		size_t len = 4 + wlbt_golden_mibs[i].len;
		size_t padded = ALIGN(len, 2);

		if (slice_len + padded > mib_slice_max) {
			wlbt_mib_send_slice(w, slice, slice_len);
			slices++;
			slice_len = 0;
		}

		put_unaligned_le16(wlbt_golden_mibs[i].psid, slice + slice_len);
		put_unaligned_le16(wlbt_golden_mibs[i].len, slice + slice_len + 2);
		memcpy(slice + slice_len + 4, wlbt_golden_mibs[i].value,
		       wlbt_golden_mibs[i].len);
		memset(slice + slice_len + len, 0, padded - len);
		slice_len += padded;
		entries++;
	}

	kfree(slice);
	dev_info(w->dev,
		 "MIB download: %u entries (%u firmware owned) in %u slices\n",
		 entries, skipped, slices);

	return 0;
}

/* Take back the host segments the peer has finished with. */
static void wlbt_hip4_release(struct exynos_wlbt *w)
{
	u8 *scb = wlbt_hip4_scb(w);
	u8 q = HIP4_Q_FH_RFB;

	while (readb(scb + wlbt_hip4_idx[q][1]) !=
	       readb(scb + wlbt_hip4_idx[q][0])) {
		u8 idx = readb(scb + wlbt_hip4_idx[q][1]);

		wlbt_hip4_slot_put(w, readl(wlbt_hip4_q_slot(w, q, idx)));
		writeb((idx + 1) & (HIP4_Q_LEN - 1), scb + wlbt_hip4_idx[q][1]);
		/* Release the index before the segment is reused. */
		wmb();
	}
}

static void wlbt_fault_record(struct exynos_wlbt *w, u16 id, const u8 *data,
			      size_t len)
{
	unsigned int slot = w->fault_next & (WLBT_FAULT_RING - 1);

	if (len > WLBT_FAULT_DATA)
		len = WLBT_FAULT_DATA;
	w->faults[slot].id = id;
	w->faults[slot].len = len;
	memcpy(w->faults[slot].data, data, len);
	w->fault_next++;
}

static void wlbt_frame_record(struct exynos_wlbt *w, u32 ref, u8 flag, u16 id,
			      const void *data, size_t len)
{
	unsigned int slot = w->frame_next & (WLBT_FRAME_RING - 1);

	memset(&w->frames[slot], 0, sizeof(w->frames[slot]));
	w->frames[slot].ref = ref;
	w->frames[slot].id = id;
	w->frames[slot].flag = flag;
	if (len > WLBT_FRAME_DATA)
		len = WLBT_FRAME_DATA;
	w->frames[slot].len = len;
	if (len)
		memcpy(&w->frames[slot].data, data, len);
	w->frame_next++;
	w->frame_total++;
}

/*
 * Print one segment of a scatter/gather frame.  The scan indications carry the
 * beacon in a chained segment, so the head alone says nothing about what was
 * received - but it is also the only place the information elements ever
 * appear, so they are worth dumping.
 */
static void wlbt_hip4_show_segment(struct exynos_wlbt *w, u32 ref,
				   const struct wlbt_mbulk *mb)
{
	/*
	 * Nothing is kept per segment: a chained frame would push everything
	 * else out of the ring, and the head record names the frame anyway.
	 */
	(void)mb;
	(void)ref;
}

/*
 * A received frame arrives as a unitdata indication whose payload is already an
 * ethernet frame.  It is copied into an skb and handed to the stack; nothing
 * else is needed because the peer did the 802.11 part.
 */
static void wlbt_rx_frame(struct exynos_wlbt *w, const void *data, size_t len);

/*
 * A received frame does not always sit in the indication that announces it: the
 * peer sends MLME_RECEIVED_FRAME_IND with an empty segment and puts the frame -
 * already ethernet, the 802.11 part is its business - in the segment chained
 * behind it.  That segment carries no FAPI header of its own, only a bulk
 * descriptor and the payload, so it has to be walked to be seen at all.
 */
static void wlbt_rx_chain(struct exynos_wlbt *w, u32 ref)
{
	unsigned int n;

	for (n = 0; n < MBULK_MAX_CHAIN; n++) {
		struct wlbt_mbulk mb;
		u16 len, head;

		memcpy_fromio(&mb, w->dram + ref, sizeof(mb));
		len = le16_to_cpu(mb.len);
		head = le16_to_cpu(mb.head);
		if (len && len <= HIP4_SLOT_SIZE - HIP4_MBULK_SIZE - head)
			wlbt_rx_frame(w, (const u8 __iomem *)w->dram + ref +
				      HIP4_MBULK_SIZE + head, len);

		if (!(mb.flag & MBULK_F_CHAIN))
			break;
		ref = le32_to_cpu(mb.chain_next_offset);
		if (!wlbt_hip4_ref_valid(w, ref))
			break;
	}
}

static void wlbt_rx_frame(struct exynos_wlbt *w, const void *data, size_t len)
{
	struct sk_buff *skb;

	if (len < ETH_HLEN || len > 4096)
		return;

	skb = netdev_alloc_skb_ip_align(w->netdev, len);
	if (!skb) {
		w->n_tx_dropped++;
		return;
	}

	memcpy_fromio(skb_put(skb, len), data, len);
	skb->protocol = eth_type_trans(skb, w->netdev);
	skb->ip_summed = CHECKSUM_NONE;
	w->netdev->stats.rx_packets++;
	w->netdev->stats.rx_bytes += len;
	netif_rx(skb);
}

static void wlbt_hip4_show(struct exynos_wlbt *w, u32 q)
{
	u8 *scb = wlbt_hip4_scb(w);
	u8 ridx = readb(scb + wlbt_hip4_idx[q][1]);
	u8 widx = readb(scb + wlbt_hip4_idx[q][0]);
	u32 chain[MBULK_MAX_CHAIN];
	unsigned int n = 0, n_chain, i;

	while (ridx != widx && n < HIP4_Q_LEN) {
		u32 ref = readl(wlbt_hip4_q_slot(w, q, ridx));
		struct wlbt_mbulk mb;
		void *m;

		if (!wlbt_hip4_ref_valid(w, ref)) {
			dev_warn(w->dev, "HIP4 q%u[%u]: bogus ref 0x%x\n",
				 q, ridx, ref);
			ridx = (ridx + 1) & (HIP4_Q_LEN - 1);
			continue;
		}

		m = w->dram + ref;
		memcpy_fromio(&mb, m, sizeof(mb));
		if (hip4_log)
			dev_info(w->dev,
				 "HIP4 q%u[%u]: ref 0x%x flag 0x%02x clas %u pid %u sig %u len %u\n",
				 q, ridx, ref, mb.flag, mb.clas, mb.pid,
				 le16_to_cpu(mb.sig_bufsz), le16_to_cpu(mb.len));

		if (le16_to_cpu(mb.sig_bufsz) >= HIP4_SIG_RESERVED +
						  sizeof(struct wlbt_fapi_hdr)) {
			u8 body[HIP4_SIG_DUMP];
			size_t len = le16_to_cpu(mb.sig_bufsz) -
				     HIP4_SIG_RESERVED;
			struct wlbt_fapi_hdr hdr;

			memcpy_fromio(&hdr, m + HIP4_MBULK_SIZE +
				      HIP4_SIG_RESERVED, sizeof(hdr));
			dev_info(w->dev,
				 "  fapi id 0x%04x receiver 0x%04x sender 0x%04x ref 0x%x\n",
				 le16_to_cpu(hdr.id),
				 le16_to_cpu(hdr.receiver_pid),
				 le16_to_cpu(hdr.sender_pid),
				 le32_to_cpu(hdr.fw_reference));

			/*
			 * MA_UNITDATA_IND carries a received frame: the payload
			 * starts after the bulk header and runs for the length
			 * the segment records.
			 */
			if (le16_to_cpu(hdr.id) == FAPI_MA_UNITDATA_IND) {
				size_t dlen = le16_to_cpu(mb.len);
				u16 off = le16_to_cpu(mb.head);

				if (dlen && dlen <= HIP4_SLOT_SIZE - HIP4_MBULK_SIZE - off)
					wlbt_rx_frame(w, m + HIP4_MBULK_SIZE + off,
						      dlen);
			}

			/*
			 * The received-frame indication is taken apart for its
			 * trailing word: if the peer keeps the frame rather than
			 * sending it, that word is the only pointer to it.
			 */
			/*
			 * MLME_RECEIVED_FRAME_IND is how the peer hands over a
			 * frame it received but did not fill into the
			 * indication itself.
			 */
			if (le16_to_cpu(hdr.id) == 0x230d &&
			    (mb.flag & MBULK_F_CHAIN))
				wlbt_rx_chain(w, ref);

			if (le16_to_cpu(hdr.id) == 0x230d) {
				size_t blen = len > sizeof(hdr) ?
					      len - sizeof(hdr) : 0;

				blen = min_t(size_t, blen, sizeof(w->peer_rxind));
				memcpy(w->peer_rxind, body, blen);
				w->peer_rxind_len = blen;
				w->peer_rxind_geom[0] = le16_to_cpu(mb.sig_bufsz);
				w->peer_rxind_geom[1] = le16_to_cpu(mb.dat_bufsz);
				w->peer_rxind_geom[2] = le16_to_cpu(mb.len);
				w->peer_rxind_geom[3] = le16_to_cpu(mb.head);
				w->n_rxind++;
			}

			/*
			 * A confirmation carries the result code first; when it
			 * is not success the request was refused, and knowing
			 * which one is worth having.  It goes into the ring
			 * rather than the log: this runs with the queue lock
			 * held and interrupts off.
			 */
			if ((le16_to_cpu(hdr.id) & 0xff00) == 0x2100 &&
			    len >= sizeof(hdr) + 2 &&
			    get_unaligned_le16(body))
				wlbt_fault_record(w, le16_to_cpu(hdr.id), body,
						  min_t(size_t, len - sizeof(hdr),
							WLBT_FAULT_DATA));

			/*
			 * MLME_CONNECT_CFM is the peer saying it took the join
			 * request.  For an open network the vendor answers the
			 * connect at that point, not after the indication, and
			 * without the answer the peer drops the link again a few
			 * seconds after associating.
			 */
			if (le16_to_cpu(hdr.id) == 0x210b) {
				w->peer_connect_cfm = true;
				schedule_work(&w->peer_work);
			}

			/* MLME_CONNECT_IND is {le16 vif, le16 result} and is the
			 * only place the outcome of a join shows up.
			 */
			if (le16_to_cpu(hdr.id) == 0x2303 &&
			    len >= sizeof(hdr) + 4) {
				w->connect_status = get_unaligned_le16(body + 2);
				w->connect_reported = true;
				w->peer_connect_ind = true;
				schedule_work(&w->peer_work);
			}

			/*
			 * MLME_CONNECTED_IND is {le16 vif, le16 aid}: the link is
			 * up and the peer wants the association id confirmed.
			 */
			if (le16_to_cpu(hdr.id) == 0x2304) {
				if (len >= sizeof(hdr) + 4)
					w->peer_aid = get_unaligned_le16(body + 2);
				w->peer_connected_ind = true;
				schedule_work(&w->peer_work);
			}

			/* MLME_DISCONNECTED_IND is {le16 vif, u8 bssid[6], le16
			 * reason}: the link the peer had is gone.
			 */
			if (le16_to_cpu(hdr.id) == 0x2309) {
				if (len >= sizeof(hdr) + 10)
					w->peer_disconnect_reason =
						get_unaligned_le16(body + 8);
				w->peer_disconnected_ind = true;
				schedule_work(&w->peer_work);
			}

			if (len > sizeof(body))
				len = sizeof(body);
			memcpy_fromio(body, m + HIP4_MBULK_SIZE +
				      HIP4_SIG_RESERVED + sizeof(hdr),
				      len > sizeof(hdr) ? len - sizeof(hdr) : 0);
			/*
			 * The peer's own log frames would fill the ring on their
			 * own, and it is the requests and indications that matter.
			 */
			if (le16_to_cpu(hdr.id) == 0x8301 ||
			    le16_to_cpu(hdr.id) == 0x8302)
				wlbt_fault_record(w, le16_to_cpu(hdr.id), body,
						  len > sizeof(hdr) ?
						  len - sizeof(hdr) : 0);

			if (hip4_log && le16_to_cpu(hdr.id) != 0x8301)
				wlbt_frame_record(w, ref, mb.flag,
						  le16_to_cpu(hdr.id), body,
						  len > sizeof(hdr) ?
						  len - sizeof(hdr) : 0);
		}

		/*
		 * Hand every segment back so the peer can reuse it.  A
		 * scatter/gather frame is a chain: the head carries the
		 * chain_next_offset of the next segment and every segment but
		 * the last says it is chained, so the peer needs all of them
		 * or it walks into a segment it already recycled.
		 */
		chain[0] = ref;
		n_chain = 1;
		while (n_chain < MBULK_MAX_CHAIN && (mb.flag & MBULK_F_CHAIN)) {
			struct wlbt_mbulk prev;
			u32 next;

			memcpy_fromio(&prev, w->dram + chain[n_chain - 1],
				      sizeof(prev));
			next = le32_to_cpu(prev.chain_next_offset);
			/*
			 * The link is only worth following once it is known to
			 * name our buffers.  A last segment that still advertises
			 * a successor usually terminates with zero, and passing
			 * that on is what makes the peer assert when it drains
			 * the queue of returned segments.
			 */
			if (!wlbt_hip4_ref_valid(w, next))
				break;
			memcpy_fromio(&mb, w->dram + next, sizeof(mb));
			chain[n_chain++] = next;
			wlbt_hip4_show_segment(w, next, &mb);
		}
		if (hip4_return)
			for (i = 0; i < n_chain; i++)
				wlbt_hip4_push(w, HIP4_Q_TH_RFB, chain[i]);

		ridx = (ridx + 1) & (HIP4_Q_LEN - 1);
		writeb(ridx, scb + wlbt_hip4_idx[q][1]);
		/* Only advance once the segment is handed back. */
		wmb();
		w->n_hip4_rx++;
		n++;
	}
}

static void wlbt_hip4_drain(struct exynos_wlbt *w)
{
	unsigned long flags;

	if (!w->hip4_control)
		return;

	/*
	 * The queues are consumed by the mailbox interrupt and by hand from
	 * debugfs, and no entry may be read twice: the peer marks a segment it
	 * has been handed back as freed, so receiving the same one again is an
	 * assertion on its side.  Holding the lock with interrupts off is what
	 * keeps the two contexts off the same index.
	 */
	spin_lock_irqsave(&w->hip4_lock, flags);
	wlbt_hip4_release(w);
	wlbt_hip4_show(w, HIP4_Q_TH_CTL);
	wlbt_hip4_show(w, HIP4_Q_TH_DAT);
	spin_unlock_irqrestore(&w->hip4_lock, flags);
}


/*
 * One stream is a ring of fixed-size cells plus the two cursors the peer reads
 * (cpacket_buffer.c:15-45).  An input stream is pre-filled with 0xff like the
 * vendor does, so a mis-parsed header is obvious rather than looking like a
 * valid message.
 */
static int wlbt_stream_init(struct exynos_wlbt *w, struct wlbt_stream *s,
			    const char *name, u32 num_packets, u32 packet_size,
			    u8 read_bit, u8 write_bit, bool to_ap)
{
	s->name = name;
	s->num_packets = num_packets;
	s->packet_size = packet_size;
	s->read_bit = read_bit;
	s->write_bit = write_bit;

	s->buf = wlbt_heap_alloc(w, num_packets * packet_size);
	s->ridx = wlbt_heap_alloc(w, 4);
	s->widx = wlbt_heap_alloc(w, 4);
	if (!s->buf || !s->ridx || !s->widx)
		return -ENOMEM;

	if (to_ap)
		memset_io(w->dram + s->buf, 0xff, num_packets * packet_size);

	return 0;
}

static void wlbt_stream_serialise(struct wlbt_stream *s,
				  struct wlbt_stream_conf *conf)
{
	conf->buf_conf.buffer_loc = s->buf;
	conf->buf_conf.num_packets = s->num_packets;
	conf->buf_conf.packet_size = s->packet_size;
	conf->buf_conf.read_index_loc = s->ridx;
	conf->buf_conf.write_index_loc = s->widx;

	/* Verbatim, as mif_stream_config_serialise() does: the two indices
	 * already carry opposite direction on the two sides.
	 */
	conf->read_bit_idx = s->read_bit;
	conf->write_bit_idx = s->write_bit;
}

/**
 * wlbt_stream_read() - take bytes out of a stream
 * @bytes: how much to read; the result is rounded up to whole cells
 *
 * Mirrors cpacketbuffer_read(): the cursors are cell counts, the read advances
 * the read cursor and tells the peer with a doorbell so it may reuse the cells.
 *
 * Return: bytes actually copied.
 */
static u32 wlbt_stream_read(struct exynos_wlbt *w, struct wlbt_stream *s,
			    void *dst, u32 bytes)
{
	u32 avail, cells, ridx, widx, first, off;

	if (!w->dram || !bytes)
		return 0;

	ridx = readl(w->dram + s->ridx);
	widx = readl(w->dram + s->widx);
	if (ridx >= s->num_packets || widx >= s->num_packets)
		return 0;

	avail = (widx + s->num_packets - ridx) % s->num_packets;
	if (!avail)
		return 0;

	cells = DIV_ROUND_UP(bytes, s->packet_size);
	if (cells > avail)
		cells = avail;
	bytes = min(bytes, cells * s->packet_size);

	off = s->buf + ridx * s->packet_size;
	first = min(cells, s->num_packets - ridx) * s->packet_size;
	memcpy_fromio(dst, w->dram + off, min(first, bytes));
	if (bytes > first)
		memcpy_fromio(dst + first, w->dram + s->buf, bytes - first);

	writel((ridx + cells) % s->num_packets, w->dram + s->ridx);

	/* Tell the peer those cells are free again (mifstream.c:163). */
	writel(BIT(s->read_bit), w->mbox + MBOX_INTGR1);

	return bytes;
}

/*
 * A short write still costs a whole cell: cpacketbuffer_write_gather() pads
 * the tail and advances the index once, so the peer never sees a half cell.
 */
static bool wlbt_stream_write(struct exynos_wlbt *w, struct wlbt_stream *s,
			      const void *src, u32 bytes)
{
	const u8 *p = src;
	u8 *cell;
	u32 cells, i, ridx, widx, left;

	if (!w->dram || !bytes)
		return false;

	ridx = readl(w->dram + s->ridx);
	widx = readl(w->dram + s->widx);
	if (ridx >= s->num_packets || widx >= s->num_packets)
		return false;

	cells = DIV_ROUND_UP(bytes, s->packet_size);
	if (cells >= s->num_packets)
		return false;

	cell = kzalloc(s->packet_size, GFP_KERNEL);
	if (!cell)
		return false;

	for (i = 0; i < cells; i++) {
		left = bytes - i * s->packet_size;
		memcpy(cell, p + i * s->packet_size, min(left, s->packet_size));
		memcpy_toio(w->dram + s->buf + ((widx + i) % s->num_packets) *
			    s->packet_size, cell, s->packet_size);
	}
	kfree(cell);

	/* memcpy_toio() orders the copies against the index store below. */
	writel((widx + cells) % s->num_packets, w->dram + s->widx);

	/* Kick the peer the way mif_stream_write_gather() does. */
	writel(BIT(s->write_bit), w->mbox + MBOX_INTGR1);

	return true;
}

static const char *wlbt_fw_level(u8 level)
{
	switch (level) {
	case 0:
		return KERN_ERR;	/* firmware errors */
	case 1:
		return KERN_WARNING;
	case 2:
		return KERN_NOTICE;
	case 3:
		return KERN_INFO;
	default:
		return KERN_DEBUG;
	}
}

/*
 * Drain the firmware's log stream and print it.  Phase 4 payloads are plain
 * NUL-terminated strings; phase 5 replaces repeated strings with indices into a
 * table the firmware carries, which needs the string map from mxlog.c that has
 * not been ported, so those are reported as unknown for now.
 */
static void wlbt_log_drain(struct exynos_wlbt *w)
{
	char text[260];
	u32 header, sync, len, level, got;

	while (wlbt_stream_read(w, &w->log, &header, sizeof(header)) ==
	       sizeof(header)) {
		sync = header >> 16;
		level = (header >> 8) & 0xff;

		if (sync == MXLOG_SYNC_PHASE_4) {
			len = header & 0xff;
		} else if (sync == MXLOG_SYNC_PHASE_5) {
			len = (header & 0xff) * 4;
		} else {
			/* Not a header at all: the stream is out of step. */
			w->n_fwlog_bad++;
			continue;
		}

		if (!len)
			continue;

		/* The payload has to be consumed either way, otherwise the
		 * next read starts in the middle of this message.
		 */
		got = wlbt_stream_read(w, &w->log, text, min(len, sizeof(text) - 1));
		text[got] = '\0';

		if (sync == MXLOG_SYNC_PHASE_4) {
			w->n_fwlog++;
			dev_printk(wlbt_fw_level(level), w->dev, "firmware: %s\n",
				   text);
		} else {
			w->n_fwlog_bad++;
			/* Phase 5 compresses repeated strings against a table
			 * the firmware carries; until that table is ported, show
			 * the raw words so the messages can still be read.
			 */
			if (w->n_fwlog_bad <= 64) {
				const __le32 *args = (const __le32 *)text;
				u32 nwords = got / 4;

				dev_info(w->dev, "fwlog5 lvl %u off 0x%x argc %u:%s\n",
					 level, nwords > 1 ? le32_to_cpu(args[1]) : 0,
					 nwords > 2 ? nwords - 2 : 0,
					 nwords > 2 ? "" : " (no args)");
				if (nwords > 2)
					dev_info(w->dev, "fwlog5 args:%*phN\n",
						 (int)((nwords - 2) * 4), text + 8);
			}
		}
	}
}

/*
 * Build the configuration the peer reads from mailbox 1: three streams (the
 * management channel in both directions and the log), the interrupt bits that
 * signal them, and an empty common HCF - this board ships no `conf/` directory
 * with the firmware, so the vendor runs with an empty one too
 * (mxfwconfig.c:283, "HW file is optional").
 */
static int wlbt_build_mxconf(struct exynos_wlbt *w)
{
	struct wlbt_mxconf config;
	struct wlbt_mxconf *conf = &config;
	u32 offset;
	int ret;

	w->heap_next = ALIGN(w->fw_runtime_length, 4);
	if (w->heap_next >= w->mem_size) {
		dev_err(w->dev, "no room for a heap after the firmware\n");
		return -ENOSPC;
	}

	ret = wlbt_stream_init(w, &w->mgmt_in, "mgmt-in", MGMT_NUM_PACKETS,
			       MGMT_PACKET_SIZE, WLBT_BIT_MGMT_IN_READ,
			       WLBT_BIT_MGMT_IN_WRITE, true);
	if (ret)
		return ret;
	ret = wlbt_stream_init(w, &w->mgmt_out, "mgmt-out", MGMT_NUM_PACKETS,
			       MGMT_PACKET_SIZE, WLBT_BIT_MGMT_OUT_READ,
			       WLBT_BIT_MGMT_OUT_WRITE, false);
	if (ret)
		return ret;
	/*
	 * The firmware validates the four GDB streams during bring-up and
	 * refuses to continue if one of them is missing, even though nothing
	 * here uses a debugger (0x801172fc in the image, "set up transports").
	 */
	ret = wlbt_stream_init(w, &w->gdb_r4_in, "gdb-r4-in", GDB_NUM_PACKETS,
			       GDB_PACKET_SIZE, WLBT_BIT_GDB_R4_IN_READ,
			       WLBT_BIT_GDB_R4_IN_WRITE, true);
	if (ret)
		return ret;
	ret = wlbt_stream_init(w, &w->gdb_r4_out, "gdb-r4-out", GDB_NUM_PACKETS,
			       GDB_PACKET_SIZE, WLBT_BIT_GDB_R4_OUT_READ,
			       WLBT_BIT_GDB_R4_OUT_WRITE, false);
	if (ret)
		return ret;
	ret = wlbt_stream_init(w, &w->gdb_m4_in, "gdb-m4-in", GDB_NUM_PACKETS,
			       GDB_PACKET_SIZE, WLBT_BIT_GDB_M4_IN_READ,
			       WLBT_BIT_GDB_M4_IN_WRITE, true);
	if (ret)
		return ret;
	ret = wlbt_stream_init(w, &w->gdb_m4_out, "gdb-m4-out", GDB_NUM_PACKETS,
			       GDB_PACKET_SIZE, WLBT_BIT_GDB_M4_OUT_READ,
			       WLBT_BIT_GDB_M4_OUT_WRITE, false);
	if (ret)
		return ret;

	ret = wlbt_stream_init(w, &w->log, "log", LOG_NUM_PACKETS,
			       LOG_PACKET_SIZE, WLBT_BIT_LOG_READ,
			       WLBT_BIT_LOG_WRITE, true);
	if (ret)
		return ret;

	offset = wlbt_heap_alloc(w, sizeof(*conf));
	if (!offset)
		return -ENOMEM;

	/* Build it in normal memory: the window is device memory, where byte and
	 * half-word stores are not something to rely on, and one block copy also
	 * makes the ordering against the peer's read obvious.
	 */
	memset(conf, 0, sizeof(*conf));
	conf->magic = MXCONF_MAGIC;
	conf->version_major = MXCONF_VERSION_MAJOR;
	conf->version_minor = MXCONF_VERSION_MINOR;
	conf->flags = 0;
	conf->soc_revision = wlbt_soc_revision(w);

	wlbt_stream_serialise(&w->mgmt_in, &conf->trans.to_ap);
	wlbt_stream_serialise(&w->mgmt_out, &conf->trans.from_ap);
	wlbt_stream_serialise(&w->gdb_r4_in, &conf->gdb_r4.to_ap);
	wlbt_stream_serialise(&w->gdb_r4_out, &conf->gdb_r4.from_ap);
	wlbt_stream_serialise(&w->gdb_m4_in, &conf->gdb_m4.to_ap);
	wlbt_stream_serialise(&w->gdb_m4_out, &conf->gdb_m4.from_ap);
	wlbt_stream_serialise(&w->log, &conf->logconf.stream_conf);

	conf->fwconfig.offset = MXCONF_FWCONFIG_UNUSED;
	conf->fwconfig.size = 0;

	memcpy_toio(w->dram + offset, conf, sizeof(*conf));
	/* The peer reads the configuration over the bus, so it has to be
	 * there before the doorbell rings.
	 */
	wmb();

	w->mxconf_offset = offset;

	dev_info(w->dev, "configuration at MIF+0x%x: mgmt %u/%u, log %u/%u, %zu bytes\n",
		 offset, MGMT_NUM_PACKETS, MGMT_PACKET_SIZE, LOG_NUM_PACKETS,
		 LOG_PACKET_SIZE, sizeof(*conf));

	return 0;
}

/*
 * Power up and let the peer run (platform_mif_9610.c:1043-1069).  The caller
 * holds w->lock and has already put the state machine in WAIT_CFG_REQ.
 */
static int wlbt_power_up(struct exynos_wlbt *w)
{
	int ret;

	ret = regmap_update_bits(w->pmu, WLBT_CTRL_NS, MASK_PWR_REQ,
				 MASK_PWR_REQ);
	if (ret) {
		dev_err(w->dev, "cannot request WLBT power: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(w->pmu, WLBT_CTRL_NS, WLBT_RESET_SET, 0);
	if (ret) {
		dev_err(w->dev, "cannot release the reset: %d\n", ret);
		return ret;
	}

	dev_info(w->dev, "reset released, starting the peer\n");

	ret = regmap_update_bits(w->pmu, WLBT_CTRL_S, WLBT_START, WLBT_START);
	if (ret) {
		dev_err(w->dev, "cannot start the peer: %d\n", ret);
		return ret;
	}

	if (!wait_for_completion_timeout(&w->cfg_ack, HZ)) {
		dev_err(w->dev, "timeout waiting for CFG_REQ\n");
		wlbt_dump(w, "cfg-timeout");
		return -ETIMEDOUT;
	}

	if (w->state != WLBT_CFG_DONE) {
		dev_err(w->dev, "peer configuration failed, state %s\n",
			wlbt_state_name(w->state));
		return -EIO;
	}

	dev_info(w->dev, "WLBT boot handshake complete\n");

	return 0;
}

/*
 * Put the peer back into reset and drop the WLBT low-power configuration, as
 * platform_mif_9610.c:1105-1187 does before asserting reset.  Only used by the
 * debugfs reset control; the boot path never needs it.
 */
static int wlbt_power_down(struct exynos_wlbt *w)
{
	unsigned long timeout;
	u32 val;
	int ret;

	/*
	 * Mask both directions first: the peer can be mid-transfer when this is
	 * called from a reboot notifier, and a line left asserted survives the
	 * reset of the SoC in the WLBT block's own registers.
	 */
	if (w->mbox) {
		writel(0xffff0000, w->mbox + MBOX_INTMR0);
		writel(0x0000ffff, w->mbox + MBOX_INTMR1);
	}

	ret = regmap_update_bits(w->pmu, CENTRAL_SEQ_WLBT_CONFIGURATION,
				 SYS_PWR_CFG_16, 0);
	ret |= regmap_update_bits(w->pmu, RESET_AHEAD_WLBT_SYS_PWR_REG,
				  SYS_PWR_CFG_2, 0);
	ret |= regmap_update_bits(w->pmu, CLEANY_BUS_WLBT_SYS_PWR_REG,
				  SYS_PWR_CFG, 0);
	ret |= regmap_update_bits(w->pmu, LOGIC_RESET_WLBT_SYS_PWR_REG,
				  SYS_PWR_CFG_2, 0);
	ret |= regmap_update_bits(w->pmu, TCXO_GATE_WLBT_SYS_PWR_REG,
				  SYS_PWR_CFG, 0);
	ret |= regmap_update_bits(w->pmu, WLBT_DISABLE_ISO_SYS_PWR_REG,
				  SYS_PWR_CFG, 1);
	ret |= regmap_update_bits(w->pmu, WLBT_RESET_ISO_SYS_PWR_REG,
				  SYS_PWR_CFG, 0);
	if (ret) {
		dev_err(w->dev, "cannot drop the WLBT power configuration: %d\n",
			ret);
		return ret;
	}

	ret = regmap_update_bits(w->pmu, WLBT_CTRL_NS, WLBT_RESET_SET,
				 WLBT_RESET_SET);
	if (ret) {
		dev_err(w->dev, "cannot assert the reset: %d\n", ret);
		return ret;
	}

	w->state = WLBT_IN_RESET;
	w->pw_cfg_dirty = true;

	/* The sequencer reports 0x80 once the domain is really down. */
	timeout = jiffies + msecs_to_jiffies(500);
	do {
		regmap_read(w->pmu, CENTRAL_SEQ_WLBT_STATUS, &val);
		if (((val & STATES) >> 16) == 0x80) {
			regmap_update_bits(w->pmu, WLBT_CTRL_NS, MASK_PWR_REQ, 0);
			dev_info(w->dev, "peer in reset\n");
			return 0;
		}
	} while (time_before(jiffies, timeout));

	dev_err(w->dev, "timeout waiting for the reset sequencer\n");
	wlbt_dump(w, "reset-timeout");

	return -ETIME;
}

static void wlbt_boot(struct exynos_wlbt *w)
{
	int ret;

	mutex_lock(&w->lock);
	w->state = WLBT_WAIT_CFG_REQ;
	reinit_completion(&w->cfg_ack);

	/*
	 * Loading firmware is opt-in until the transport that talks to it
	 * exists: without HIP4 nothing answers the peer, and a boot path that
	 * starts a subsystem nothing services is not worth having.
	 */
	if (load_firmware && !w->fw_loaded)
		wlbt_load_firmware(w);
	if (w->fw_loaded) {
		/* Only allow the interrupt bits the streams actually use. */
		writel((~WLBT_IRQ_BITS & 0xffff) << 16, w->mbox + MBOX_INTMR0);
		if (!wlbt_build_mxconf(w))
			wlbt_mbox_init(w);
		else
			dev_err(w->dev, "no configuration: the peer will not find its streams\n");
	}

	wlbt_pw_cfg_restore(w);

	if (trace_boot && w->trace) {
		unsigned int off, field, us;

		if (sscanf(trace_boot, "%x:%x:%u", &off, &field, &us) == 3) {
			w->trace_offset = off & ~3u;
			w->trace_field = field & ~3u;
			w->trace_us = min_t(unsigned int, us, 30 * USEC_PER_SEC);
			if (!wlbt_trace_start(w))
				dev_warn(w->dev, "trace already running\n");
		}
	}

	ret = wlbt_power_up(w);
	if (ret) {
		dev_err(w->dev, "boot failed: %d\n", ret);
	} else if (w->fw_loaded) {
		/* Give the firmware a moment to boot and say something, then
		 * take whatever it left in the log even if the doorbell was
		 * missed.
		 */
		msleep(300);
		wlbt_log_drain(w);
		dev_info(w->dev, "after boot: %u log messages, %u undecodable, INTSR0 0x%08x\n",
			 w->n_fwlog, w->n_fwlog_bad,
			 readl(w->mbox + MBOX_INTSR0));
	}
	mutex_unlock(&w->lock);
}

static ssize_t wlbt_debugfs_boot(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct exynos_wlbt *w = file->private_data;

	wlbt_boot(w);

	return len;
}

/*
 * Ask the peer to halt and wait for MM_HALT_RSP.  Nothing here needs the
 * reply, but a round trip is the only way to prove the outbound management
 * path works before HIP4 depends on it.
 */
static ssize_t wlbt_debugfs_halt(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct exynos_wlbt *w = file->private_data;
	unsigned int before = w->n_mgmt;
	bool sent;
	int i;

	mutex_lock(&w->lock);
	sent = wlbt_mgmt_send(w, MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT,
			      WLBT_MM_HALT_REQ, 0);

	/* Watch the doorbell before draining anything: the log acknowledgements
	 * write the same register, so a drain here would hide the result.
	 */
	for (i = 0; i < 10; i++) {
		if (!(readl(w->mbox + MBOX_INTSR1) & BIT(WLBT_BIT_MGMT_OUT_WRITE)))
			break;
		msleep(100);
	}
	dev_info(w->dev, "halt requested (%s), peer %s the doorbell, ring read %u write %u\n",
		 sent ? "written" : "failed",
		 i < 10 ? "took" : "ignored",
		 readl(w->dram + w->mgmt_out.ridx),
		 readl(w->dram + w->mgmt_out.widx));
	wlbt_mgmt_drain(w);
	dev_info(w->dev, "%u management message(s) in reply\n", w->n_mgmt - before);
	mutex_unlock(&w->lock);

	return sent ? len : -EIO;
}

/*
 * Send one management frame: write "<channel> <ma_msg> <arg>" where the three
 * fields are plain decimals and arg is optional.  This is how the peer is asked
 * to start a service (channel 1), and it reports what the peer did with the
 * frame so a silent peer cannot be mistaken for a working one.
 */
static ssize_t wlbt_debugfs_send(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct exynos_wlbt *w = file->private_data;
	unsigned int channel, msg, arg = 0;
	unsigned int before = w->n_mgmt;
	char text[64];
	bool sent;
	int i;

	if (len == 0 || len >= sizeof(text))
		return -EINVAL;
	if (copy_from_user(text, buf, len))
		return -EFAULT;
	text[len] = '\0';

	if (sscanf(text, "%u %u %u", &channel, &msg, &arg) < 2)
		return -EINVAL;
	if (channel > 255 || msg > 255)
		return -EINVAL;

	mutex_lock(&w->lock);
	sent = wlbt_mgmt_send(w, channel, msg, arg);

	/* Same doorbell trick as halt: look before draining, because the peer's
	 * log acknowledgements write the same register.
	 */
	for (i = 0; i < 10; i++) {
		if (!(readl(w->mbox + MBOX_INTSR1) & BIT(WLBT_BIT_MGMT_OUT_WRITE)))
			break;
		msleep(100);
	}
	dev_info(w->dev, "sent ch%u msg %u arg 0x%x (%s), peer %s the doorbell, ring read %u write %u\n",
		 channel, msg, arg, sent ? "written" : "failed",
		 i < 10 ? "took" : "ignored",
		 readl(w->dram + w->mgmt_out.ridx),
		 readl(w->dram + w->mgmt_out.widx));
	wlbt_mgmt_drain(w);
	dev_info(w->dev, "%u management message(s) in reply\n", w->n_mgmt - before);
	mutex_unlock(&w->lock);

	return sent ? len : -EIO;
}

static ssize_t wlbt_debugfs_reset(struct file *file, const char __user *buf,
				  size_t len, loff_t *ppos)
{	struct exynos_wlbt *w = file->private_data;
	int ret;

	mutex_lock(&w->lock);
	ret = wlbt_power_down(w);
	mutex_unlock(&w->lock);

	return ret ? ret : len;
}

/*
 * Start the WLAN service.  The peer reads the optional_data field of
 * SM_MSG_START_REQ as a MIF-relative address and walks it as a hip4_hip_control
 * (0x801185f4 converts the reference, 0x801178f8 hands it to the service), so
 * the HIP4 region has to exist before the request goes out - a zero reference
 * makes the peer read the firmware header as a control block and stop.
 */
static ssize_t wlbt_debugfs_wlan(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct exynos_wlbt *w = file->private_data;
	unsigned int before = w->n_mgmt;
	u8 frame[7];
	__le32 ref;
	int ret, i;

	mutex_lock(&w->lock);
	ret = wlbt_hip4_build(w);
	if (ret) {
		mutex_unlock(&w->lock);
		return ret;
	}

	/* The vendor fills the peer's MIB database between opening the service
	 * and starting it (mgt.c:800-870), so the entries are in place before
	 * the WLAN driver there runs its own start.
	 */
	if (mib_early)
		wlbt_download_mibs(w);

	frame[0] = MMTRANS_CHAN_ID_SERVICE_MANAGEMENT;
	frame[1] = WLBT_SERVICE_ID_WLAN;
	frame[2] = WLBT_SM_MSG_START_REQ;
	ref = cpu_to_le32(w->hip4_control);
	memcpy(frame + 3, &ref, sizeof(ref));

	ret = wlbt_stream_write(w, &w->mgmt_out, frame, sizeof(frame));
	dev_info(w->dev, "WLAN start request (%s), control block 0x%x\n",
		 ret ? "written" : "failed", w->hip4_control);
	if (ret) {
		w->hip4_started = true;
		wlbt_hip4_read_limits(w);
	}

	/* A start confirmation comes back on the same channel; give the peer a
	 * moment and show whatever arrives.
	 */
	for (i = 0; i < 20; i++) {
		msleep(50);
		wlbt_mgmt_drain(w);
		if (w->n_mgmt != before)
			break;
	}
	dev_info(w->dev, "%u management message(s) after WLAN start\n",
		 w->n_mgmt - before);
	mutex_unlock(&w->lock);

	return ret ? len : -EIO;
}

/*
 * Talk to the peer's WLAN service over HIP4.  "send <hex signal id> [hex
 * payload bytes]" puts one FAPI signal on the from-host control queue,
 * "drain" collects whatever is waiting on the to-host queues.
 */
static ssize_t wlbt_debugfs_hip4(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct exynos_wlbt *w = file->private_data;
	struct wlbt_fapi_hdr hdr = { };
	/*
	 * Static because w->lock serialises this handler and the buffers are
	 * too big for the stack.
	 */
	static u8 sig[sizeof(hdr) + 2048];
	static u8 dat[1024];
	static char kbuf[4608];
	char cmd[16];
	const char *p, *q;
	char *sep;
	unsigned int id, val, n = 0, dn = 0;
	unsigned long flags;
	int ret, used = 0;

	ret = simple_write_to_buffer(kbuf, sizeof(kbuf) - 1, ppos, buf, len);
	if (ret < 0)
		return ret;
	kbuf[ret] = '\0';

	mutex_lock(&w->lock);
	ret = wlbt_hip4_build(w);
	if (ret)
		goto out;

	if (sscanf(kbuf, "%15s", cmd) != 1) {
		ret = -EINVAL;
		goto out;
	}

	if (!strcmp(cmd, "state")) {
		wlbt_hip4_state(w);
		ret = len;
		goto out;
	}

	if (!strcmp(cmd, "drain")) {
		wlbt_hip4_drain(w);
		ret = len;
		goto out;
	}

	/*
	 * "sendd" queues one unitdata request on the data queue whatever the
	 * interface state is, so the peer can be asked whether it reads that
	 * queue before a connection exists as well as after one.  The frame is a
	 * real broadcast ARP request: if the peer does transmit it, it shows up
	 * in a capture on the access point.
	 */
	if (!strcmp(cmd, "sendd")) {
		static const u8 arp[42] = {
			0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
			0x8c, 0x77, 0x12, 0xaa, 0xbb, 0xcc,
			0x08, 0x06,			/* ARP */
			0x00, 0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x01,
			0x8c, 0x77, 0x12, 0xaa, 0xbb, 0xcc,
			0xc0, 0xa8, 0x63, 0x02,		/* 192.168.99.2 */
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0xc0, 0xa8, 0x63, 0x01,		/* 192.168.99.1 */
		};
		u8 probe[sizeof(hdr) + 24];
		struct wlbt_fapi_hdr dh = { };

		dh.id = cpu_to_le16(0x1000);		/* MA_UNITDATA_REQ */
		dh.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
		dh.fw_reference = cpu_to_le32(send_ref);
		memcpy(probe, &dh, sizeof(dh));
		put_unaligned_le16(1, probe + sizeof(dh) + 0);	/* vif */
		put_unaligned_le16(w->host_tag++, probe + sizeof(dh) + 2);
		put_unaligned_le16(FAPI_PRIORITY_CONTENTION,
				   probe + sizeof(dh) + 4);
		put_unaligned_le16(1, probe + sizeof(dh) + 6);	/* peer */
		put_unaligned_le16(FAPI_DATAUNIT_IEEE802_3,
				   probe + sizeof(dh) + 8);

		ret = wlbt_hip4_send(w, false, probe, sizeof(probe), arp,
				     sizeof(arp));
		dev_info(w->dev, "HIP4 data probe queued: %d\n", ret);
		if (ret)
			goto out;
		msleep(1500);
		wlbt_hip4_drain(w);
		ret = len;
		goto out;
	}

	/* Ring the from-host interrupt again without queueing anything. */
	if (!strcmp(cmd, "kick")) {
		writel(BIT(WLBT_BIT_HIP4_FROM_HOST), w->mbox + MBOX_INTGR1);
		ret = len;
		goto out;
	}

	if (!strcmp(cmd, "scan")) {
		if (!w->hip4_started) {
			ret = -EINVAL;
			goto out;
		}

		ret = wlbt_mlme_add_scan(w, 0x0100);
		if (ret)
			goto out;
		msleep(500);
		wlbt_hip4_drain(w);
		ret = len;
		goto out;
	}

	if (!strcmp(cmd, "mibs")) {
		if (!w->hip4_started) {
			ret = -EINVAL;
			goto out;
		}
		ret = wlbt_download_mibs(w);
		if (ret < 0)
			goto out;
		msleep(200);
		wlbt_hip4_drain(w);
		ret = len;
		goto out;
	}

	/*
	 * "mibset <psid> <hex bytes>" writes one entry into the peer's MIB database
	 * at run time, which is how a setting is tried out without a reboot: the
	 * files in the HIP4 area are only read when the peer starts.
	 */
	if (!strncmp(cmd, "mibset", 6)) {
		u8 rec[4 + 64];
		unsigned int psid, dlen = 0;
		char *h;

		if (sscanf(kbuf + 6, "%x", &psid) != 1 ||
		    !w->hip4_started) {
			ret = -EINVAL;
			goto out;
		}
		h = strchr(kbuf + 6, ' ');
		if (h) {
			for (h++; *h && dlen < sizeof(rec) * 2; h++) {
				if (!isxdigit(*h))
					continue;
				val = hex_to_bin(*h);
				if (dlen & 1)
					rec[5 + dlen / 2] |= val;
				else
					rec[5 + dlen / 2] = val << 4;
				dlen++;
			}
		}
		if (dlen & 1) {
			ret = -EINVAL;
			goto out;
		}
		dlen /= 2;
		put_unaligned_le16(psid, rec);
		put_unaligned_le16(dlen, rec + 2);
		if (dlen & 1)
			rec[5 + dlen / 2] = 0;
		wlbt_mib_send_slice(w, rec, ALIGN(4 + dlen, 2));
		dev_info(w->dev, "MIB 0x%04x set to %u bytes\n", psid, dlen);
		ret = len;
		goto out;
	}

	if (!strcmp(cmd, "ret")) {
		unsigned int ref;

		if (sscanf(kbuf + strlen(cmd), "%x", &ref) != 1) {
			ret = -EINVAL;
			goto out;
		}
		spin_lock_irqsave(&w->hip4_lock, flags);
		wlbt_hip4_push(w, HIP4_Q_TH_RFB, ref);
		spin_unlock_irqrestore(&w->hip4_lock, flags);
		dev_info(w->dev, "HIP4 returned segment 0x%x\n", ref);
		ret = len;
		goto out;
	}

	if ((strcmp(cmd, "send") && strcmp(cmd, "sendp")) || !w->hip4_started) {
		ret = -EINVAL;
		goto out;
	}

	p = kbuf + strlen(cmd);
	if (sscanf(p, "%x%n", &id, &used) != 1) {
		ret = -EINVAL;
		goto out;
	}
	p += used;
	while (*p == ' ')
		p++;

	/*
	 * "sendp" takes a second, whitespace separated run of bytes: the
	 * payload that follows the signal.  Requests carrying information
	 * elements want them there, not inside the signal.
	 */
	if (!strcmp(cmd, "sendp")) {
		sep = strchr(p, ' ');
		if (!sep) {
			ret = -EINVAL;
			goto out;
		}
		*sep = '\0';
		q = sep + 1;
		for (; *q && dn < sizeof(dat); q++) {
			if (!isxdigit(*q))
				continue;
			val = hex_to_bin(*q);
			if (dn & 1)
				dat[dn / 2] |= val;
			else
				dat[dn / 2] = val << 4;
			dn++;
		}
		if (dn & 1) {
			ret = -EINVAL;
			goto out;
		}
		dn /= 2;
	}

	/* The signal is a hex string with the bytes run together. */
	for (; *p && n < sizeof(sig) - sizeof(hdr); p++) {
		if (!isxdigit(*p))
			continue;
		val = hex_to_bin(*p);
		if (n & 1)
			sig[sizeof(hdr) + n / 2] |= val;
		else
			sig[sizeof(hdr) + n / 2] = val << 4;
		n++;
	}
	if (n & 1) {
		ret = -EINVAL;
		goto out;
	}
	n /= 2;

	hdr.id = cpu_to_le16(id);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	hdr.fw_reference = cpu_to_le32(send_ref);
	memcpy(sig, &hdr, sizeof(hdr));

	ret = wlbt_hip4_send(w, true, sig, sizeof(hdr) + n,
			     dn ? dat : NULL, dn);
	dev_info(w->dev, "HIP4 signal 0x%04x (%zu byte signal, %u data): %d\n",
		 id, sizeof(hdr) + n, dn, ret);
	if (ret)
		goto out;

	msleep(100);
	wlbt_hip4_drain(w);
	ret = len;

out:
	mutex_unlock(&w->lock);
	return ret;
}

#define WLBT_DEBUGFS_OPS(_write)					\
	{								\
		.owner = THIS_MODULE,					\
		.open = simple_open,					\
		.write = (_write),					\
		.llseek = noop_llseek,					\
	}

#define WLBT_PEEK_MAX_WORDS	4096

/*
 * The peer's RTOS keeps its kernel block, thread descriptors and stacks in MIF
 * DRAM, so a plain read of the right offset shows what the peer is doing
 * without touching a single doorbell.  Write "<hex offset> <words>" and read
 * the words back.
 */
static int wlbt_debugfs_peek_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	u32 i;

	if (!w->dram)
		return 0;

	for (i = 0; i < w->peek_words; i++) {
		u32 off = w->peek_offset + i * 4;

		if (off + 4 > w->mem_size)
			break;
		seq_printf(s, "%06x %08x%s", off, readl(w->dram + off),
			   i % 4 == 3 ? "\n" : "  ");
	}
	if (i % 4)
		seq_putc(s, '\n');

	return 0;
}

static int wlbt_peek_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_debugfs_peek_show, inode->i_private);
}

static ssize_t wlbt_debugfs_peek(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct exynos_wlbt *w = s->private;
	unsigned int off, words;
	char cmd[64];

	if (len >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, buf, len))
		return -EFAULT;
	cmd[len] = '\0';

	if (sscanf(cmd, "%x %u", &off, &words) != 2)
		return -EINVAL;

	w->peek_offset = off & ~3u;
	w->peek_words = min_t(unsigned int, words, WLBT_PEEK_MAX_WORDS);

	return len;
}

/*
 * Write "<hex offset> <hex value>" into the peer's DRAM.  Debug only, and the
 * narrow complement of peek: when a field the peer owns is the only thing
 * standing between a working and a broken data path, setting it by hand is how
 * the field is identified at all, before there is any code to read.
 */
static ssize_t wlbt_debugfs_poke(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct exynos_wlbt *w = file->private_data;
	unsigned int off, val;
	char cmd[64];

	if (!w->dram)
		return -ENODEV;
	if (len >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, buf, len))
		return -EFAULT;
	cmd[len] = '\0';

	if (sscanf(cmd, "%x %x", &off, &val) != 2)
		return -EINVAL;
	off &= ~3u;
	if (off + 4 > w->mem_size)
		return -EINVAL;

	writel(val, w->dram + off);
	/* The peer may be polling the word right now, so finish the store. */
	wmb();

	return len;
}

/*
 * The peer is a dual-core RTOS with a saved context per thread, so the PC it
 * will resume at is a plain word in DRAM.  Sampling that word from here shows
 * where the peer stops running, at the moment it stops, without a debugger.
 * "<hex offset> <hex field> <microseconds>": field 0 samples the word itself,
 * anything else treats the word as a pointer into MIF DRAM and samples the
 * word at that address plus the field.
 */
static u32 wlbt_trace_sample(struct exynos_wlbt *w)
{
	u32 v = readl(w->dram + w->trace_offset);
	u32 off;

	if (!w->trace_field)
		return v;

	if (v < 0x80000000 || v - 0x80000000 + w->trace_field + 4 > w->mem_size)
		return 0;
	off = (v - 0x80000000) + w->trace_field;

	return readl(w->dram + off);
}

static void wlbt_trace_run(struct exynos_wlbt *w)
{
	u64 start, now, end;
	u32 v, last = 0;
	unsigned int i = 0, n = 0;

	if (!w->dram || !w->trace_us || !w->trace)
		return;

	start = ktime_get_ns();
	end = start + (u64)w->trace_us * NSEC_PER_USEC;
	do {
		v = wlbt_trace_sample(w);
		if (v != last && i < WLBT_TRACE_MAX) {
			now = ktime_get_ns();
			w->trace[i].us = (u32)((now - start) / NSEC_PER_USEC);
			w->trace[i].val = v;
			i++;
			last = v;
		}
		now = ktime_get_ns();
		if (!(++n & 0x3fff))
			cond_resched();
	} while (now < end);

	w->n_trace = i;
}

static int wlbt_trace_thread(void *data)
{
	struct exynos_wlbt *w = data;

	wlbt_trace_run(w);
	atomic_set(&w->trace_running, 0);

	return 0;
}

static bool wlbt_trace_start(struct exynos_wlbt *w)
{
	struct task_struct *task;

	if (atomic_cmpxchg(&w->trace_running, 0, 1))
		return false;

	task = kthread_run(wlbt_trace_thread, w, "wlbt-trace");
	if (IS_ERR(task)) {
		atomic_set(&w->trace_running, 0);
		return false;
	}

	return true;
}

static int wlbt_debugfs_trace_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	unsigned int i;

	if (!w->dram)
		return 0;

	seq_printf(s, "%u us sampling %06x%s, %u changes%s\n",
		   w->trace_us, w->trace_offset,
		   w->trace_field ? " (indirect)" : "", w->n_trace,
		   atomic_read(&w->trace_running) ? " (still running)" : "");
	for (i = 0; i < w->n_trace; i++)
		seq_printf(s, "%8u us  %08x\n", w->trace[i].us, w->trace[i].val);

	return 0;
}

static int wlbt_trace_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_debugfs_trace_show, inode->i_private);
}

static ssize_t wlbt_debugfs_trace(struct file *file, const char __user *buf,
				  size_t len, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct exynos_wlbt *w = s->private;
	unsigned int off, field, us;
	char cmd[64];

	if (len >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, buf, len))
		return -EFAULT;
	cmd[len] = '\0';

	if (sscanf(cmd, "%x %x %u", &off, &field, &us) != 3)
		return -EINVAL;

	w->trace_offset = off & ~3u;
	w->trace_field = field & ~3u;
	w->trace_us = min_t(unsigned int, us, 30 * USEC_PER_SEC);

	if (!wlbt_trace_start(w))
		return -EBUSY;

	return len;
}

#define WLBT_FIND_CHUNK	1024	/* words */

/*
 * The peer's thread descriptors carry their name inline, so looking for a name
 * in MIF DRAM finds the descriptor without knowing the RTOS internals.
 */
static void wlbt_dram_find(struct exynos_wlbt *w)
{
	u32 limit, off, n;
	u8 *buf;

	w->n_find = 0;
	if (!w->dram || !w->find_len)
		return;

	/* The peer allocates from the MIF heap above its runtime image, so the
	 * whole window has to be searched, not only the loaded image.
	 */
	limit = w->mem_size;
	buf = kzalloc(WLBT_FIND_CHUNK * 4, GFP_KERNEL);
	if (!buf)
		return;

	for (off = 0; off + 4 <= limit; off += WLBT_FIND_CHUNK * 4) {
		u32 i;

		n = min((u32)(WLBT_FIND_CHUNK * 4), limit - off);
		memcpy_fromio(buf, w->dram + off, n);
		for (i = 0; i + w->find_len <= n; i++) {
			if (buf[i] != w->find_needle[0])
				continue;
			if (memcmp(buf + i, w->find_needle, w->find_len))
				continue;
			if (w->n_find < ARRAY_SIZE(w->find_hits))
				w->find_hits[w->n_find++] = off + i;
		}
		if (w->n_find >= ARRAY_SIZE(w->find_hits))
			break;
	}

	kfree(buf);
}

static int wlbt_debugfs_find_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	unsigned int i;

	seq_printf(s, "\"%s\": %u match(es) in %u bytes\n", w->find_needle,
		   w->n_find, w->fw_runtime_length);
	for (i = 0; i < w->n_find; i++)
		seq_printf(s, "  %06x\n", w->find_hits[i]);

	return 0;
}

static int wlbt_find_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_debugfs_find_show, inode->i_private);
}

static ssize_t wlbt_debugfs_find(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct exynos_wlbt *w = s->private;
	char needle[32];

	if (!len || len >= sizeof(needle))
		return -EINVAL;
	if (copy_from_user(needle, buf, len))
		return -EFAULT;
	if (needle[len - 1] == '\n')
		len--;
	if (!len || len >= sizeof(w->find_needle))
		return -EINVAL;

	memcpy(w->find_needle, needle, len);
	w->find_needle[len] = '\0';
	w->find_len = len;
	wlbt_dram_find(w);

	return len;
}

static const struct file_operations wlbt_find_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_find_open,
	.read = seq_read,
	.write = wlbt_debugfs_find,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations wlbt_trace_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_trace_open,
	.read = seq_read,
	.write = wlbt_debugfs_trace,
	.llseek = seq_lseek,
	.release = single_release,
};

static int wlbt_bss_show(struct seq_file *s, void *unused)
{
	wlbt_dram_bsses(s->private, s);
	return 0;
}

static int wlbt_bss_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_bss_show, inode->i_private);
}

static const struct file_operations wlbt_bss_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_bss_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations wlbt_poke_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_poke);

static const struct file_operations wlbt_peek_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_peek_open,
	.read = seq_read,
	.write = wlbt_debugfs_peek,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations wlbt_boot_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_boot);
static const struct file_operations wlbt_reset_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_reset);
static const struct file_operations wlbt_halt_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_halt);
static const struct file_operations wlbt_send_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_send);
static const struct file_operations wlbt_wlan_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_wlan);
static const struct file_operations wlbt_hip4_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_hip4);

/*
 * What did the peer do with its memory?  The image occupies [0, fw_size) and
 * the runtime, heap and panic record live above it, so anything non-zero up
 * there was written by the peer - the cheapest proof that the firmware is
 * really running, and where it stopped if it is not.
 */
static void wlbt_dram_report(struct exynos_wlbt *w, struct seq_file *s)
{
	unsigned int i, nz = 0, first_nz = 0;
	u32 val;

	if (!w->dram || !w->fw_loaded)
		return;

	for (i = w->fw_size & ~3u; i + 4 <= w->fw_runtime_length; i += 4) {
		if (readl(w->dram + i)) {
			if (!nz)
				first_nz = i;
			nz++;
		}
	}

	seq_printf(s, "peer memory: %u non-zero words above the image (%u bytes), first at 0x%x\n",
		   nz, w->fw_size, first_nz);

	if (nz) {
		seq_printf(s, "peer words @0x%x:", first_nz);
		for (i = 0; i < 8; i++)
			seq_printf(s, " %08x",
				   readl(w->dram + first_nz + i * 4));
		seq_puts(s, "\n");
	}

	if (w->fw_panic_offset && w->fw_panic_offset + 32 <= w->mem_size) {
		seq_printf(s, "panic record @0x%x:", w->fw_panic_offset);
		for (i = 0; i < 8; i++) {
			val = readl(w->dram + w->fw_panic_offset + i * 4);
			seq_printf(s, " %08x", val);
		}
		seq_puts(s, "\n");
	}
}

static void wlbt_stream_status(struct exynos_wlbt *w, struct seq_file *s,
			       struct wlbt_stream *st)
{
	if (!w->dram)
		return;

	seq_printf(s, "stream %-10s buffer +0x%06x cells %4u x %3u  bits %u/%u  read %u write %u\n",
		   st->name, st->buf, st->num_packets, st->packet_size,
		   st->read_bit, st->write_bit,
		   readl(w->dram + st->ridx), readl(w->dram + st->widx));
}

static int wlbt_debugfs_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	unsigned int i;
	u32 val;

	seq_printf(s, "state %s  MIF DRAM %pa  irq cfg-req %u bad %u wdog %u mbox %u\n",
		   wlbt_state_name(w->state), &w->mem_base, w->n_cfg_req,
		   w->n_cfg_bad, w->n_wdog, w->n_mbox);
	seq_printf(s, "firmware %s, entry 0x%x, runtime %u, dram %s\n",
		   firmware_name, w->fw_entry, w->fw_runtime_length,
		   w->fw_loaded ? "loaded" : "not loaded");
	seq_printf(s, "mailbox last status 0x%04x\n", w->last_mbox_status);
	if (w->n_rxind)
		seq_printf(s, "rxind %u, sig %u dat %u len %u head %u, body %*phN\n",
			   w->n_rxind, w->peer_rxind_geom[0], w->peer_rxind_geom[1],
			   w->peer_rxind_geom[2], w->peer_rxind_geom[3],
			   (int)min_t(u32, w->peer_rxind_len, 18), w->peer_rxind);
	if (w->connect_reported)
		seq_printf(s, "connect result 0x%04x\n", w->connect_status);
	seq_printf(s, "link %s, tx %lu called %u dropped %u, hip4 tx %u rx %u\n",
		   w->connected ? "up" : "down",
		   w->netdev ? w->netdev->stats.tx_packets : 0UL,
		   w->n_tx_calls, w->n_tx_dropped, w->n_hip4_tx, w->n_hip4_rx);
	if (w->hip4_ref)
		seq_printf(s, "hip4 0x%x control 0x%x mib 0x%x (%u bytes)\n",
			   w->hip4_ref, w->hip4_control,
			   w->hip4_ref + HIP4_MIB_OFFSET, HIP4_MIB_SIZE);

	regmap_read(w->pmu, CENTRAL_SEQ_WLBT_STATUS, &val);
	seq_printf(s, "CENTRAL_SEQ_WLBT_STATUS 0x%08x\n", val);
	regmap_read(w->pmu, WLBT_CTRL_NS, &val);
	seq_printf(s, "WLBT_CTRL_NS 0x%08x\n", val);
	regmap_read(w->pmu, WLBT_CTRL_S, &val);
	seq_printf(s, "WLBT_CTRL_S 0x%08x\n", val);
	regmap_read(w->pmu, CENTRAL_SEQ_WLBT_CONFIGURATION, &val);
	seq_printf(s, "CENTRAL_SEQ_WLBT_CONFIGURATION 0x%08x\n", val);

	seq_printf(s, "irq to peer: INTGR1 0x%08x INTMR1 0x%08x INTSR1 0x%08x\n",
		   readl(w->mbox + MBOX_INTGR1), readl(w->mbox + MBOX_INTMR1),
		   readl(w->mbox + MBOX_INTSR1));
	seq_printf(s, "mailbox IS_VERSION 0x%08x MCUCTRL 0x%08x INTSR0 0x%08x INTMR0 0x%08x\n",
		   readl(w->mbox + MBOX_IS_VERSION),
		   readl(w->mbox + MBOX_MCUCTRL),
		   readl(w->mbox + MBOX_INTSR0),
		   readl(w->mbox + MBOX_INTMR0));

	wlbt_dram_report(w, s);

	if (w->dram && w->mxconf_offset) {
		wlbt_stream_status(w, s, &w->mgmt_in);
		wlbt_stream_status(w, s, &w->mgmt_out);
		wlbt_stream_status(w, s, &w->gdb_r4_in);
		wlbt_stream_status(w, s, &w->gdb_r4_out);
		wlbt_stream_status(w, s, &w->gdb_m4_in);
		wlbt_stream_status(w, s, &w->gdb_m4_out);
		wlbt_stream_status(w, s, &w->log);
		seq_printf(s, "streams: log %u bytes queued (fw %u, bad %u), mgmt %u messages\n",
			   ((readl(w->dram + w->log.widx) + w->log.num_packets -
			     readl(w->dram + w->log.ridx)) % w->log.num_packets) *
			   w->log.packet_size,
			   w->n_fwlog, w->n_fwlog_bad, w->n_mgmt);
			seq_printf(s, "mxconf at MIF+0x%x, heap next 0x%x\n",
			   w->mxconf_offset, w->heap_next);

		/* The firmware reports its progress as 0x2222_0000 | stage in
		 * mailbox 0 (0x8011d568 in the image).
		 */
		val = readl(w->mbox + MBOX_ISSR_BASE);
		if ((val & 0xffff0000) == 0x22220000)
			seq_printf(s, "peer boot stage %u\n", val & 0xffff);
	}

	if (w->dram)
		for (i = 0; i < 8; i++)
			seq_printf(s, "dram[%u] 0x%08x%s", i,
				   readl(w->dram + i * 4),
				   i % 4 == 3 ? "\n" : "  ");

	/* The peer writes mailboxes 8 and 9 (image 0x80117f2a), past the eight
	 * the vendor's mbox manager knows about.
	 */
	for (i = 0; i < 12; i++)
		seq_printf(s, "ISSR%u 0x%08x%s", i,
			   readl(w->mbox + MBOX_ISSR_BASE + i * 4),
			   i % 4 == 3 ? "\n" : "  ");

	return 0;
}

static int wlbt_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_debugfs_show, inode->i_private);
}

static const struct file_operations wlbt_dump_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int wlbt_frames_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	unsigned int i, n;

	if (!w->frame_total) {
		seq_puts(s, "no frames recorded\n");
		return 0;
	}

	seq_printf(s, "%u frame(s), last %u kept\n", w->frame_total,
		   min_t(unsigned int, w->frame_total, WLBT_FRAME_RING));
	n = min_t(unsigned int, w->frame_total, WLBT_FRAME_RING);
	for (i = 0; i < n; i++) {
		unsigned int slot = (w->frame_next - n + i) &
				    (WLBT_FRAME_RING - 1);

		seq_printf(s, "ref 0x%x flag 0x%02x id 0x%04x len %u\n",
			   w->frames[slot].ref, w->frames[slot].flag,
			   w->frames[slot].id, w->frames[slot].len);
		seq_hex_dump(s, "  ", DUMP_PREFIX_NONE, 32, 1,
			     w->frames[slot].data, w->frames[slot].len, false);
	}

	return 0;
}

static int wlbt_frames_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_frames_show, inode->i_private);
}

static const struct file_operations wlbt_frames_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_frames_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int wlbt_faults_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	unsigned int i, n;

	if (!w->fault_next) {
		seq_puts(s, "no fault reports\n");
		return 0;
	}

	n = min_t(unsigned int, w->fault_next, WLBT_FAULT_RING);
	seq_printf(s, "%u report(s), last %u kept\n", w->fault_next, n);
	for (i = 0; i < n; i++) {
		unsigned int slot = (w->fault_next - n + i) &
				    (WLBT_FAULT_RING - 1);

		seq_printf(s, "id 0x%04x len %u\n", w->faults[slot].id,
			   w->faults[slot].len);
		seq_hex_dump(s, "  ", DUMP_PREFIX_NONE, 28, 1,
			     w->faults[slot].data, w->faults[slot].len, false);
	}

	return 0;
}

static int wlbt_faults_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_faults_show, inode->i_private);
}

static const struct file_operations wlbt_faults_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_faults_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void wlbt_debugfs_init(struct exynos_wlbt *w)
{
	w->debugfs = debugfs_create_dir("exynos9610-wlbt", NULL);
	if (IS_ERR_OR_NULL(w->debugfs)) {
		w->debugfs = NULL;
		return;
	}

	debugfs_create_file("dump", 0400, w->debugfs, w, &wlbt_dump_fops);
	debugfs_create_file("boot", 0200, w->debugfs, w, &wlbt_boot_fops);
	debugfs_create_file("off", 0200, w->debugfs, w, &wlbt_reset_fops);
	debugfs_create_file("halt", 0200, w->debugfs, w, &wlbt_halt_fops);
	debugfs_create_file("send", 0200, w->debugfs, w, &wlbt_send_fops);
	debugfs_create_file("wlan", 0200, w->debugfs, w, &wlbt_wlan_fops);
	debugfs_create_file("hip4", 0200, w->debugfs, w, &wlbt_hip4_fops);
	debugfs_create_file("peek", 0600, w->debugfs, w, &wlbt_peek_fops);
	debugfs_create_file("poke", 0200, w->debugfs, w, &wlbt_poke_fops);
	debugfs_create_file("trace", 0600, w->debugfs, w, &wlbt_trace_fops);
	debugfs_create_file("find", 0600, w->debugfs, w, &wlbt_find_fops);
	debugfs_create_file("bss", 0400, w->debugfs, w, &wlbt_bss_fops);
	debugfs_create_file("frames", 0400, w->debugfs, w, &wlbt_frames_fops);
	debugfs_create_file("faults", 0400, w->debugfs, w, &wlbt_faults_fops);
}

static struct regmap *wlbt_syscon(struct exynos_wlbt *w, const char *name)
{
	struct regmap *map;

	map = syscon_regmap_lookup_by_phandle(w->dev->of_node, name);
	if (IS_ERR(map)) {
		dev_err(w->dev, "cannot find %s: %ld\n", name, PTR_ERR(map));
		return map;
	}

	return map;
}

static int wlbt_parse_memory(struct exynos_wlbt *w)
{
	struct device_node *node;
	struct resource res;
	int ret;

	node = of_parse_phandle(w->dev->of_node, "memory-region", 0);
	if (!node) {
		dev_err(w->dev, "no memory-region for the MIF DRAM\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(node, 0, &res);
	of_node_put(node);
	if (ret) {
		dev_err(w->dev, "cannot read the MIF DRAM window: %d\n", ret);
		return ret;
	}

	w->mem_base = res.start;
	w->mem_size = resource_size(&res);

	return 0;
}

/*
 * cfg80211 front end
 *
 * The peer is a full MAC: it scans and joins by itself and the driver only
 * carries requests in and results out.  Its scan indications are metadata
 * only - frequency, RSSI and a scan id, never a frame - so the beacons and
 * probe responses it keeps for its own use are read back out of its DRAM once
 * the scan is done.  A scan result without information elements is worthless
 * to a supplicant, and this is the only place they can be had.
 */
#define WLBT_SCAN_CHUNK		8192
#define WLBT_SCAN_OVERLAP	1024
#define WLBT_SCAN_SEEN		64
#define WLBT_SCAN_IE_MAX	512
#define WLBT_SCAN_RSSI		(-65 * 100)	/* dBm, this firmware reports none */
#define WLBT_SCAN_DELAY_MS	5000

static struct ieee80211_channel wlbt_2ghz_channels[] = {
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2412, .hw_value = 1 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2417, .hw_value = 2 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2422, .hw_value = 3 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2427, .hw_value = 4 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2432, .hw_value = 5 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2437, .hw_value = 6 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2442, .hw_value = 7 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2447, .hw_value = 8 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2452, .hw_value = 9 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2457, .hw_value = 10 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2462, .hw_value = 11 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2467, .hw_value = 12 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2472, .hw_value = 13 },
};

static struct ieee80211_rate wlbt_2ghz_rates[] = {
	{ .bitrate = 10 },
	{ .bitrate = 20, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 55, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 110, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 60 },
	{ .bitrate = 90 },
	{ .bitrate = 120 },
	{ .bitrate = 180 },
	{ .bitrate = 240 },
	{ .bitrate = 360 },
	{ .bitrate = 480 },
	{ .bitrate = 540 },
};

/*
 * cfg80211 validates the cipher and AKM suites a supplicant asks for against
 * these before it ever calls the driver, so a wiphy that advertises none of
 * them makes every connection attempt fail with -EINVAL.  The peer does the
 * encryption, so all of them can be offered.
 */
static const u32 wlbt_cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_AES_CMAC,
};

static struct ieee80211_supported_band wlbt_band_2ghz = {
	.band = NL80211_BAND_2GHZ,
	.channels = wlbt_2ghz_channels,
	.n_channels = ARRAY_SIZE(wlbt_2ghz_channels),
	.bitrates = wlbt_2ghz_rates,
	.n_bitrates = ARRAY_SIZE(wlbt_2ghz_rates),
};

static struct exynos_wlbt *wlbt_from_wiphy(struct wiphy *wiphy)
{
	return *(struct exynos_wlbt **)wiphy_priv(wiphy);
}

/*
 * The private area of the net device is its wireless_dev, so the driver data is
 * reached through the wiphy and not by casting that area.
 */
static struct exynos_wlbt *wlbt_from_netdev(struct net_device *ndev)
{
	struct wireless_dev *wdev = netdev_priv(ndev);

	if (!wdev || !wdev->wiphy)
		return NULL;

	return wlbt_from_wiphy(wdev->wiphy);
}

/*
 * A management frame worth reporting: a beacon or a probe response, both
 * addresses being the access point, with elements that parse to the end.  The
 * peer's DRAM is full of everything else, so the test has to be strict.
 */
static bool wlbt_bss_frame(const u8 *buf, size_t avail, size_t *frame_len,
			   u16 *freq)
{
	static const u8 bssid_ofs = 16;
	size_t off = 36;
	bool have_ssid = false;
	u8 channel = 0;
	u16 fc;

	if (avail < 36)
		return false;

	fc = get_unaligned_le16(buf);
	if (fc != (IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_BEACON) &&
	    fc != (IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_PROBE_RESP))
		return false;
	if (memcmp(buf + 10, buf + bssid_ofs, ETH_ALEN))
		return false;
	/* A zeroed or multicast address is one of the peer's other buffers
	 * being mistaken for a frame, not an access point.
	 */
	if (is_zero_ether_addr(buf + bssid_ofs) ||
	    !is_valid_ether_addr(buf + bssid_ofs))
		return false;

	while (off + 2 <= avail) {
		u8 id = buf[off];
		u8 len = buf[off + 1];

		if (off + 2 + len > avail)
			break;
		if (id == WLAN_EID_SSID) {
			if (len > 32)
				break;
			have_ssid = true;
		} else if (id == WLAN_EID_DS_PARAMS && len >= 1) {
			channel = buf[off + 2];
		}
		off += 2 + len;
		if (off - 36 > WLBT_SCAN_IE_MAX)
			break;
	}

	/*
	 * Zero length elements are a legal way to pad, so the only hard
	 * requirement is that the information elements were there at all and
	 * that an SSID was among them.
	 */
	if (!have_ssid || off <= 36)
		return false;

	/*
	 * A channel the wiphy does not know makes cfg80211 dereference a NULL
	 * channel pointer, and since this runs from the scan work that oops
	 * takes cfg80211_scan_done() with it.  Anything outside the band is a
	 * misread buffer, not an access point.
	 */
	if (channel) {
		if (channel > 13)
			return false;
		*freq = 2412 + 5 * (channel - 1);
	} else {
		*freq = 2437;
	}

	*frame_len = off;
	return true;
}

static void wlbt_scan_report_one(struct exynos_wlbt *w, const u8 *frame,
				 size_t len, u16 freq)
{
	struct cfg80211_inform_bss data = {
		.chan = ieee80211_get_channel(w->wiphy, freq),
		.signal = WLBT_SCAN_RSSI,
	};
	struct cfg80211_bss *bss;

	if (!data.chan) {
		dev_warn_once(w->dev, "scan: frame on unknown frequency %u\n",
			      freq);
		return;
	}

	bss = cfg80211_inform_bss_frame_data(w->wiphy, &data,
					     (struct ieee80211_mgmt *)frame,
					     len, GFP_KERNEL);
	dev_info(w->dev, "bss %pM freq %u len %zu chan %s -> %s\n",
		 frame + 16, freq, len, data.chan ? "known" : "UNKNOWN",
		 IS_ERR(bss) ? "rejected" : "ok");
	if (!IS_ERR_OR_NULL(bss))
		cfg80211_put_bss(w->wiphy, bss);
}

static void wlbt_scan_collect(struct exynos_wlbt *w)
{
	u8 seen[WLBT_SCAN_SEEN][ETH_ALEN];
	unsigned int n_seen = 0, n_frame = 0;
	u32 off, limit = w->mem_size;
	u8 *buf;

	if (!w->dram || !w->wiphy)
		return;

	buf = kvmalloc(WLBT_SCAN_CHUNK + WLBT_SCAN_OVERLAP, GFP_KERNEL);
	if (!buf)
		return;

	for (off = 0; off < limit; off += WLBT_SCAN_CHUNK) {
		size_t n = min_t(u32, WLBT_SCAN_CHUNK + WLBT_SCAN_OVERLAP,
				 limit - off);
		size_t i;

		memcpy_fromio(buf, w->dram + off, n);
		for (i = 0; i + 36 <= n && i < WLBT_SCAN_CHUNK; i++) {
			size_t flen;
			u16 freq;
			unsigned int k;

			if (!wlbt_bss_frame(buf + i, n - i, &flen, &freq))
				continue;
			for (k = 0; k < n_seen; k++)
				if (!memcmp(seen[k], buf + i + 16, ETH_ALEN))
					break;
			if (k < n_seen)
				continue;
			if (n_seen < WLBT_SCAN_SEEN)
				memcpy(seen[n_seen++], buf + i + 16, ETH_ALEN);
			wlbt_scan_report_one(w, buf + i, flen, freq);
			n_frame++;
		}
	}

	kvfree(buf);
	dev_info(w->dev, "scan: %u beacon(s) reported from the peer's memory\n",
		 n_frame);
}

/*
 * List what the peer's memory holds, the same way the scan collector reads it.
 * Choosing a target for a join by hand needs to know which access points the
 * peer actually cached, and there is exactly one join attempt per boot - a failed
 * join wedges the join state machine until the next boot.
 */
static void wlbt_dram_bsses(struct exynos_wlbt *w, struct seq_file *s)
{
	u8 seen[WLBT_SCAN_SEEN][ETH_ALEN];
	unsigned int n_seen = 0, n = 0;
	u32 off, limit = w->mem_size;
	u8 *buf;

	if (!w->dram)
		return;

	buf = kvmalloc(WLBT_SCAN_CHUNK + WLBT_SCAN_OVERLAP, GFP_KERNEL);
	if (!buf)
		return;

	for (off = 0; off < limit; off += WLBT_SCAN_CHUNK) {
		size_t len = min_t(u32, WLBT_SCAN_CHUNK + WLBT_SCAN_OVERLAP,
				   limit - off);
		size_t i;

		memcpy_fromio(buf, w->dram + off, len);
		for (i = 0; i + 36 <= len && i < WLBT_SCAN_CHUNK; i++) {
			const u8 *f = buf + i;
			size_t flen;
			u16 freq;
			unsigned int k;
			unsigned int j = 36;
			const u8 *ssid = NULL, *rsn = NULL, *rsn_caps = NULL;
			u8 ssid_len = 0;
			bool wpa = false;

			if (!wlbt_bss_frame(f, len - i, &flen, &freq))
				continue;
			for (k = 0; k < n_seen; k++)
				if (!memcmp(seen[k], f + 16, ETH_ALEN))
					break;
			if (k < n_seen)
				continue;
			if (n_seen < WLBT_SCAN_SEEN)
				memcpy(seen[n_seen++], f + 16, ETH_ALEN);

			while (j + 2 <= flen) {
				const u8 *ie = f + j + 2;
				unsigned int ielen = f[j + 1];

				if (j + 2 + ielen > flen)
					break;
				if (f[j] == WLAN_EID_SSID && !ssid) {
					ssid = ie;
					ssid_len = ielen;
				} else if (f[j] == WLAN_EID_RSN) {
					rsn = ie;
					rsn_caps = ielen >= 4 ? ie + ielen - 2 : NULL;
				} else if (f[j] == WLAN_EID_VENDOR_SPECIFIC &&
					   ielen >= 4 && ie[0] == 0x00 &&
					   ie[1] == 0x50 && ie[2] == 0xf2 &&
					   ie[3] == 0x01) {
					wpa = true;
				}
				j += 2 + ielen;
			}

			seq_printf(s, "%pM  %u MHz  ", f + 16, freq);
			if (ssid)
				seq_printf(s, "\"%.*s\"  ", ssid_len, ssid);
			else
				seq_puts(s, "(no SSID)  ");
			seq_printf(s, "%s%s%s%s\n",
				   (f[34] & 0x10) ? "privacy " : "open ",
				   rsn ? "RSN " : "", wpa ? "WPA " : "",
				   rsn && rsn_caps[0] != 0 ? "MFP " : "");
			n++;
		}
	}

	kvfree(buf);
	seq_printf(s, "%u access point(s)\n", n);
}

static void wlbt_scan_work(struct work_struct *work)
{
	struct exynos_wlbt *w = container_of(to_delayed_work(work),
					     struct exynos_wlbt, scan_work);
	struct cfg80211_scan_info info = { };
	struct cfg80211_scan_request *req = w->scan_req;

	w->scan_req = NULL;
	if (!req)
		return;

	mutex_lock(&w->lock);
	wlbt_scan_collect(w);
	mutex_unlock(&w->lock);

	cfg80211_scan_done(req, &info);
}

static int wlbt_cfg_scan(struct wiphy *wiphy,
			 struct cfg80211_scan_request *request)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);
	int ret;

	if (!w->hip4_control)
		return -EIO;
	if (w->scan_req) {
		dev_info_ratelimited(w->dev, "scan refused: one is pending\n");
		return -EBUSY;
	}

	/*
	 * A scan request queued while the station is associated is never
	 * consumed: the peer stays on the channel of the connection, and its
	 * single reader walks the queues in order, so everything queued behind
	 * the scan - data included - waits with it.  Refuse instead of
	 * reporting a scan that will not happen.
	 */
	if (w->connected || netif_carrier_ok(w->netdev)) {
		dev_info_ratelimited(w->dev,
				     "scan refused: connected %d carrier %d\n",
				     w->connected, netif_carrier_ok(w->netdev));
		return -EBUSY;
	}

	/* The scan id names the interface in its upper byte and the peer
	 * refuses anything below 0x100, so the low byte just wraps.
	 */
	w->scan_id = 0x0100 | ((w->scan_id + 1) & 0xff);
	ret = wlbt_mlme_add_scan(w, w->scan_id);
	if (ret)
		return ret;

	w->scan_req = request;
	schedule_delayed_work(&w->scan_work,
			      msecs_to_jiffies(WLBT_SCAN_DELAY_MS));
	return 0;
}

/*
 * Bringing the interface up is a firmware sequence, not a driver one: the peer
 * needs a vif record before it will accept a join against it, and it refuses
 * the join with INVALID_VIF_INDEX (0x800a, fault 0x2614) otherwise.  The vendor
 * driver sends MLME_ADD_VIF, the host state and the country when the net device
 * opens; all three are sent here on the first join instead, because the peer has
 * to be running before any of them makes sense.
 */
static int wlbt_mlme_add_vif(struct exynos_wlbt *w)
{
	static const u8 mac[6] = { 0x8c, 0x77, 0x12, 0xaa, 0xbb, 0xcc };
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 28];
	struct wlbt_fapi_hdr hdr = { };

	/* Body: interface, own address, type, partner address - the order the
	 * peer accepted when the vif was created by hand.
	 */
	memset(sig, 0, sizeof(sig));
	put_unaligned_le16(1, sig + sizeof(hdr) + 0);	/* vif */
	memcpy(sig + sizeof(hdr) + 2, mac, sizeof(mac));
	put_unaligned_le16(2, sig + sizeof(hdr) + 8);	/* FAPI_VIFTYPE_STATION */
	memcpy(sig + sizeof(hdr) + 10, mac, sizeof(mac));

	hdr.id = cpu_to_le16(FAPI_MLME_ADD_VIF_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(hdr) + 28, NULL, 0);
}

static int wlbt_mlme_host_state(struct exynos_wlbt *w, u16 state)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 16];
	struct wlbt_fapi_hdr hdr = { };

	memset(sig, 0, sizeof(sig));
	put_unaligned_le16(1, sig + sizeof(hdr) + 0);	/* vif */
	put_unaligned_le16(0xffff, sig + sizeof(hdr) + 2);
	put_unaligned_le16(state, sig + sizeof(hdr) + 4);

	hdr.id = cpu_to_le16(FAPI_MLME_SET_HOST_STATE_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(hdr) + 16, NULL, 0);
}

/*
 * The peer will not transmit on a channel its regulatory domain does not allow,
 * which shows up as a join refused with NOT_ALLOWED rather than as an error at
 * scan time.  The vendor sends this right after the host state.
 */
static int wlbt_mlme_set_country(struct exynos_wlbt *w, const char *alpha2)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 14];
	struct wlbt_fapi_hdr hdr = { };

	memset(sig, 0, sizeof(sig));
	put_unaligned_le16(0, sig + sizeof(hdr) + 0);	/* not tied to a vif */
	put_unaligned_le16((alpha2[0] << 8) | alpha2[1],
			   sig + sizeof(hdr) + 2);

	hdr.id = cpu_to_le16(FAPI_MLME_SET_COUNTRY_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(hdr) + 14, NULL, 0);
}

static int wlbt_vif_up(struct exynos_wlbt *w)
{
	int ret;

	if (w->vif_created)
		return 0;

	ret = wlbt_mlme_add_vif(w);
	if (ret)
		return ret;
	/* The peer answers on its own thread; give it the moment it needs
	 * before the join that depends on the record.
	 */
	msleep(150);

	ret = wlbt_mlme_host_state(w, 2);	/* cellular active */
	if (ret)
		return ret;
	msleep(50);

	ret = wlbt_mlme_set_country(w, "VN");
	if (ret)
		return ret;
	msleep(50);


	w->vif_created = true;
	return 0;
}

/*
 * The join is the peer's job: the driver only builds MLME_CONNECT_REQ and waits
 * for the firmware to report the outcome.  The body is the vendor's layout -
 * interface, BSSID, authentication type at +8 and the channel frequency at +10
 * as twice the real value - which is the one the peer accepts and acts on.
 *
 * The information elements are the association request's.  cfg80211 does not
 * include the SSID in what it hands over, and the peer refuses a request without
 * it (INVALID_PARAMETERS) as well as one without an RSN element
 * (REJECTED_INVALID_IE).
 */
/*
 * The peer announces a connection with MLME_CONNECT_IND and then waits for the
 * host to say how to proceed; without an answer it drops the link again a few
 * seconds later (MLME_DISCONNECTED_IND, reason 3).  Both replies are the vif
 * number and three spare words.
 */
static void wlbt_mlme_connect_res(struct exynos_wlbt *w)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 14] = { };
	struct wlbt_fapi_hdr hdr = { };
	u16 vif = 1;
	int ret;

	hdr.id = cpu_to_le16(FAPI_MLME_CONNECT_RES);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));
	put_unaligned_le16(vif, sig + sizeof(hdr));

	ret = wlbt_hip4_send(w, true, sig, sizeof(sig), NULL, 0);
	dev_info(w->dev, "connect: replied to MLME_CONNECT_IND (%d)\n", ret);
}

/*
 * The peer keeps a list of the channels the host considers usable and wants it
 * refreshed when a connection is up, not before: sent ahead of a join it makes
 * the peer refuse the join with NOT_ALLOWED.  The list travels as a vendor
 * element of channel entries.
 */
static int wlbt_mlme_set_cached_channels(struct exynos_wlbt *w)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 14];
	u8 dat[7 + 3 * WLBT_CACHED_CHANNELS];
	struct wlbt_fapi_hdr hdr = { };
	unsigned int i, n = 0;

	memset(sig, 0, sizeof(sig));
	put_unaligned_le16(1, sig + sizeof(hdr) + 0);	/* vif 1 */

	dat[n++] = 0xdd;			/* vendor specific element */
	dat[n++] = 5;				/* OUI, type and subtype */
	dat[n++] = 0x00;
	dat[n++] = 0x16;
	dat[n++] = 0x32;
	dat[n++] = 0x01;			/* scan parameters */
	dat[n++] = 0x02;			/* channel list */

	for (i = 0; i < WLBT_CACHED_CHANNELS; i++) {
		put_unaligned_le16((2412 + 5 * i) * 2, dat + n);
		dat[n + 2] = 0x18;		/* 2.4 GHz and 5 GHz */
		n += 3;
		dat[1] += 3;
	}

	hdr.id = cpu_to_le16(FAPI_MLME_SET_CACHED_CHANNELS);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(hdr) + 14, dat, n);
}

/*
 * MLME_SET_PACKET_FILTER with no patterns.  The vendor sends this on connect for
 * an open network, and the peer appears to need it before it will carry frames.
 */
static int wlbt_mlme_set_packet_filter(struct exynos_wlbt *w)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 14] = { };
	struct wlbt_fapi_hdr hdr = { };

	hdr.id = cpu_to_le16(0x2014);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));
	put_unaligned_le16(1, sig + sizeof(hdr));	/* vif 1 */

	return wlbt_hip4_send(w, true, sig, sizeof(sig), NULL, 0);
}

/* MLME_SET_ACL with no entries: the station accepts its peer and nothing else. */
static int wlbt_mlme_set_acl(struct exynos_wlbt *w)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 18] = { };
	struct wlbt_fapi_hdr hdr = { };

	hdr.id = cpu_to_le16(FAPI_MLME_SET_ACL_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));
	put_unaligned_le16(1, sig + sizeof(hdr));	/* vif 1 */

	return wlbt_hip4_send(w, true, sig, sizeof(sig), NULL, 0);
}

static void wlbt_mlme_connected_res(struct exynos_wlbt *w, u16 aid)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 16] = { };
	struct wlbt_fapi_hdr hdr = { };
	int ret;

	hdr.id = cpu_to_le16(FAPI_MLME_CONNECTED_RES);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));
	put_unaligned_le16(1, sig + sizeof(hdr));
	put_unaligned_le16(aid, sig + sizeof(hdr) + 2);

	ret = wlbt_hip4_send(w, true, sig, sizeof(sig), NULL, 0);
	dev_info(w->dev, "connect: replied to MLME_CONNECTED_IND %u (%d)\n",
		 aid, ret);
}

/*
 * The vendor gives the peer the supplicant's information elements twice: once
 * named as the association request's elements (mlme.c:2430), and again inside
 * the connect request.  The copy in the connect request is what ends up in the
 * frame on air, which is why the association looks right without this; the
 * copy sent here is what the peer reads to work out what the link *is*, the RSN
 * element above all.  Sent only the connect request, the peer associates, runs
 * the four way handshake, stores the keys, confirms them - and still treats the
 * link as open, because it was never told that it is protected.
 */
static int wlbt_mlme_add_info_elements(struct exynos_wlbt *w, const u8 *ies,
				       size_t len)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 16] = { };
	struct wlbt_fapi_hdr hdr = { };

	if (!ies || !len)
		return 0;

	hdr.id = cpu_to_le16(FAPI_MLME_ADD_INFO_ELEMENTS_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));
	put_unaligned_le16(1, sig + sizeof(hdr) + 0);		/* vif */
	put_unaligned_le16(FAPI_PURPOSE_ASSOCIATION_REQUEST,
			   sig + sizeof(hdr) + 2);

	return wlbt_hip4_send(w, true, sig, sizeof(sig), ies, len);
}

static int wlbt_mlme_connect(struct exynos_wlbt *w,
			     const struct cfg80211_connect_params *sme,
			     const u8 *bssid, u16 freq)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 24];
	u8 dat[2 + IEEE80211_MAX_SSID_LEN + 512];
	struct wlbt_fapi_hdr hdr = { };
	size_t ssid_len = sme->ssid_len, n = 0;

	if (!bssid || ssid_len > IEEE80211_MAX_SSID_LEN)
		return -EINVAL;

	memset(sig, 0, sizeof(sig));
	put_unaligned_le16(1, sig + sizeof(hdr) + 0);		/* vif 1 */
	memcpy(sig + sizeof(hdr) + 2, bssid, ETH_ALEN);
	put_unaligned_le16(sme->auth_type, sig + sizeof(hdr) + 8);
	put_unaligned_le16(freq * 2, sig + sizeof(hdr) + 10);

	dat[n++] = WLAN_EID_SSID;
	dat[n++] = ssid_len;
	memcpy(dat + n, sme->ssid, ssid_len);
	n += ssid_len;

	if (sme->ie_len) {
		if (sme->ie_len > sizeof(dat) - n)
			return -EINVAL;
		memcpy(dat + n, sme->ie, sme->ie_len);
		n += sme->ie_len;
	}

	hdr.id = cpu_to_le16(FAPI_MLME_CONNECT_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(hdr) + 24, dat, n);
}

/*
 * Answer whatever the peer asked for in its connect indications, and tell
 * cfg80211 about the outcome.  This cannot run in the drain: the replies are
 * sends that take the queue lock, and cfg80211 may sleep.
 */
static int wlbt_mlme_set_pmk(struct exynos_wlbt *w, const u8 *pmk,
			     size_t len);

static void wlbt_peer_work(struct work_struct *work)
{
	struct exynos_wlbt *w = container_of(work, struct exynos_wlbt, peer_work);
	unsigned long flags;
	bool cind, cted, disc;
	u16 status, aid, reason;

	spin_lock_irqsave(&w->hip4_lock, flags);
	cind = w->peer_connect_ind;
	cted = w->peer_connected_ind;
	disc = w->peer_disconnected_ind;
	status = w->connect_status;
	aid = w->peer_aid;
	reason = w->peer_disconnect_reason;
	w->peer_connect_cfm = false;
	w->peer_connect_ind = false;
	w->peer_connected_ind = false;
	w->peer_disconnected_ind = false;
	spin_unlock_irqrestore(&w->hip4_lock, flags);

	/*
	 * The peer asks once the connection is there, and its data path stays
	 * shut until it is answered: without this the frames are taken off the
	 * data queue and none of them ever reaches the air.  The confirm that
	 * precedes the indication only says the request was accepted, so
	 * answering that is both too early and unnecessary.
	 */
	if (cind && !w->connect_res_sent) {
		/*
		 * Before the join the peer has no record to attach a key to, so
		 * the one sent with the connect may have gone nowhere; this is
		 * the first moment it can act on it.
		 */
		if (w->connect_pmk_len)
			wlbt_mlme_set_pmk(w, w->connect_pmk, w->connect_pmk_len);

		wlbt_mlme_connect_res(w);
		w->connect_res_sent = true;
	}

	if (cind) {
		if (status) {
			dev_info(w->dev, "connect: peer refused the join: 0x%04x\n",
				 status);
			w->connecting = false;
			cfg80211_disconnected(w->netdev, status, NULL, 0, true,
					      GFP_KERNEL);
			return;
		}

		/*
		 * The vendor reports the connection here, on the indication,
		 * and follows it with the access list and the channel list the
		 * peer keeps for its own scans.  MLME_CONNECTED_IND arrives
		 * later, if at all, so waiting for it means the supplicant has
		 * already given up.
		 */
		wlbt_mlme_set_packet_filter(w);
		msleep(20);
		wlbt_mlme_set_acl(w);
		msleep(20);
		wlbt_mlme_set_cached_channels(w);

		w->connected = true;
		if (w->connecting) {
			w->connecting = false;
			netif_carrier_on(w->netdev);
			dev_info(w->dev, "connect: connected to %pM\n",
				 w->connect_bssid);
			cfg80211_connect_result(w->netdev, w->connect_bssid,
						NULL, 0, NULL, 0,
						WLAN_STATUS_SUCCESS, GFP_KERNEL);
		}
	}

	if (cted && !netif_carrier_ok(w->netdev)) {
		wlbt_mlme_connected_res(w, aid);
		netif_carrier_on(w->netdev);
	}

	if (disc && w->connected) {
		w->connected = false;
		netif_carrier_off(w->netdev);
		dev_info(w->dev, "connect: peer lost the link, reason %u\n",
			 reason);
		w->connecting = false;
		cfg80211_disconnected(w->netdev, reason, NULL, 0,
				      !w->connect_reported, GFP_KERNEL);
	}
}

static void wlbt_connect_work(struct work_struct *work)
{
	struct exynos_wlbt *w = container_of(to_delayed_work(work),
					     struct exynos_wlbt, connect_work);

	if (!w->connecting)
		return;
	w->connecting = false;

	if (w->connect_reported && !w->connect_status) {
		dev_info(w->dev, "connect: associated with %pM\n",
			 w->connect_bssid);
		cfg80211_connect_result(w->netdev, w->connect_bssid, NULL, 0,
					NULL, 0, WLAN_STATUS_SUCCESS, GFP_KERNEL);
		return;
	}

	dev_info(w->dev, "connect: failed, peer result 0x%04x%s\n",
		 w->connect_status,
		 w->connect_reported ? "" : " (no indication)");
	cfg80211_connect_result(w->netdev, w->connect_bssid, NULL, 0, NULL, 0,
				WLAN_STATUS_UNSPECIFIED_FAILURE, GFP_KERNEL);
}


static int wlbt_cfg_connect(struct wiphy *wiphy, struct net_device *ndev,
			    struct cfg80211_connect_params *sme)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);
	struct cfg80211_bss *bss;
	u8 bssid[ETH_ALEN];
	u16 freq = 2437;
	int ret;

	if (!w->hip4_control)
		return -EIO;
	if (w->connecting)
		return -EBUSY;

	ret = wlbt_vif_up(w);
	if (ret)
		return ret;

	/*
	 * A PSK network hands the pairwise master key over with the connect, so
	 * that the peer's key database has it before the join.  The host still
	 * runs the handshake, but the peer is the one that holds the link's
	 * keys, and a key database that does not know the master key cannot be
	 * told what the pairwise key it is given belongs to.
	 */
	if (sme->crypto.psk) {
		dev_info(w->dev, "connect: pairwise master key supplied\n");
		memcpy(w->connect_pmk, sme->crypto.psk, WLAN_PMK_LEN);
		w->connect_pmk_len = WLAN_PMK_LEN;
		ret = wlbt_mlme_set_pmk(w, sme->crypto.psk, WLAN_PMK_LEN);
		if (ret)
			return ret;
	} else {
		w->connect_pmk_len = 0;
	}

	if (sme->channel && sme->channel->center_freq)
		freq = sme->channel->center_freq;

	/*
	 * A connect request does not have to name an access point: when it does
	 * not, the one the scan found is used, which is what "iw connect" and
	 * the supplicant's first attempt rely on.
	 */
	if (sme->bssid) {
		memcpy(bssid, sme->bssid, ETH_ALEN);
	} else {
		bss = cfg80211_get_bss(wiphy, sme->channel, NULL, sme->ssid,
				       sme->ssid_len, IEEE80211_BSS_TYPE_ESS,
				       IEEE80211_PRIVACY_ANY);
		if (!bss) {
			dev_info(w->dev, "connect: no access point for \"%.*s\"\n",
				 (int)sme->ssid_len, sme->ssid);
			return -ENOENT;
		}
		memcpy(bssid, bss->bssid, ETH_ALEN);
		if (!sme->channel && bss->channel)
			freq = bss->channel->center_freq;
		cfg80211_put_bss(wiphy, bss);
	}

	dev_info(w->dev, "connect: %pM freq %u auth %u ssid \"%.*s\" ies %zu\n",
		 bssid, freq, sme->auth_type, (int)sme->ssid_len, sme->ssid,
		 sme->ie_len);

	memcpy(w->connect_bssid, bssid, ETH_ALEN);
	w->connect_reported = false;
	w->connect_status = 0;
	w->connect_res_sent = false;
	w->peer_connect_cfm = false;

	ret = wlbt_mlme_add_info_elements(w, sme->ie, sme->ie_len);
	if (ret)
		return ret;

	ret = wlbt_mlme_connect(w, sme, bssid, freq);
	if (ret)
		return ret;

	w->connecting = true;
	schedule_delayed_work(&w->connect_work, msecs_to_jiffies(10000));
	return 0;
}

/*
 * A disconnect the supplicant asked for needs no report back - cfg80211 issued
 * it and already knows.  All this has to do is forget the connection, or the
 * next scan is refused as if the station were still associated.
 */
static int wlbt_cfg_disconnect(struct wiphy *wiphy, struct net_device *ndev,
			       u16 reason_code)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);

	cancel_delayed_work_sync(&w->connect_work);
	cancel_work_sync(&w->peer_work);
	w->connecting = false;
	w->connected = false;
	netif_carrier_off(ndev);

	return 0;
}

/*
 * MLME_SET_PMK hands the peer the pairwise master key: it keeps a key database
 * of its own, and the key travels as the payload after a sixteen byte body, the
 * way the vendor's fapi_append_data() lays it out.  A zero length request is
 * how the vendor asks for an entry to be dropped.
 */
static int wlbt_mlme_set_pmk(struct exynos_wlbt *w, const u8 *pmk, size_t len)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 16] = { };
	struct wlbt_fapi_hdr hdr = { };

	if (len > 64)
		return -EINVAL;

	put_unaligned_le16(1, sig + sizeof(hdr));	/* vif */
	hdr.id = cpu_to_le16(FAPI_MLME_SET_PMK_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(sig), pmk, len);
}

static int wlbt_cfg_set_pmk(struct wiphy *wiphy, struct net_device *dev,
			    const struct cfg80211_pmk_conf *conf)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);

	if (!w->hip4_control)
		return -EIO;

	dev_info(w->dev, "set_pmk: aa %pM len %u\n", conf->aa, conf->pmk_len);

	return wlbt_mlme_set_pmk(w, conf->pmk, conf->pmk_len);
}

static int wlbt_cfg_del_pmk(struct wiphy *wiphy, struct net_device *dev,
			    const u8 *aa)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);

	if (!w->hip4_control)
		return -EIO;

	dev_info(w->dev, "del_pmk: aa %pM\n", aa);

	return wlbt_mlme_set_pmk(w, NULL, 0);
}

/*
 * MLME_SETKEYS is how the peer is told about the keys a host supplicant
 * negotiated - the firmware does not do the four way handshake itself, so this
 * is the only way a WPA2 connection can carry traffic.  The length field is in
 * bits, the packet sequence numbers are packed as little endian half words, and
 * the key itself is the payload, the way the vendor's fapi_append_data() leaves
 * it.
 */
static int wlbt_mlme_setkeys(struct exynos_wlbt *w, u16 key_id, u16 key_type,
			     const u8 *address, u32 cipher, const u8 *key,
			     size_t key_len, const u8 *seq, size_t seq_len)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 46];
	struct wlbt_fapi_hdr hdr = { };
	size_t n = sizeof(hdr);
	unsigned int i;

	if (!key || !key_len || key_len > 32 || !address)
		return -EINVAL;

	memset(sig, 0, sizeof(sig));
	put_unaligned_le16(1, sig + n + 0);		/* vif */
	put_unaligned_le16(key_len * 8, sig + n + 2);	/* length in bits */
	put_unaligned_le16(key_id, sig + n + 4);
	put_unaligned_le16(key_type, sig + n + 6);
	memcpy(sig + n + 8, address, ETH_ALEN);

	for (i = 0; seq && i + 1 < seq_len && i < 16; i += 2)
		put_unaligned_le16((seq[i + 1] << 8) | seq[i],
				   sig + n + 14 + i);

	put_unaligned_le32(cipher, sig + n + 30);

	hdr.id = cpu_to_le16(FAPI_MLME_SETKEYS_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	/*
	 * The key material travels behind the signal, not inside it: the peer
	 * reads it from the payload at the offset the signal length implies, so
	 * a key appended to the signal is read back as whatever follows it.
	 */
	return wlbt_hip4_send(w, true, sig, n + 46, key, key_len);
}

static int wlbt_cfg_add_key(struct wiphy *wiphy, struct net_device *ndev,
			    int link_id, u8 key_index, bool pairwise,
			    const u8 *mac_addr, struct key_params *params)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);
	u16 type;

	if (!w->hip4_control)
		return -EIO;

	if (params->cipher == WLAN_CIPHER_SUITE_WEP40 ||
	    params->cipher == WLAN_CIPHER_SUITE_WEP104)
		type = FAPI_KEYTYPE_WEP;
	else if (pairwise)
		type = FAPI_KEYTYPE_PAIRWISE;
	else if (params->cipher == WLAN_CIPHER_SUITE_AES_CMAC ||
		 params->cipher == WLAN_CIPHER_SUITE_BIP_GMAC_128 ||
		 params->cipher == WLAN_CIPHER_SUITE_BIP_GMAC_256)
		type = FAPI_KEYTYPE_IGTK;
	else
		type = FAPI_KEYTYPE_GROUP;

	/*
	 * A group key is not addressed to a peer: cfg80211 says so by leaving the
	 * address out, and the peer still wants the access point named, the same
	 * way the vendor takes the station peer record for a group key.
	 */
	if (!mac_addr) {
		if (type != FAPI_KEYTYPE_GROUP || !is_valid_ether_addr(w->connect_bssid))
			return -EINVAL;
		mac_addr = w->connect_bssid;
	}

	dev_info(w->dev, "setkeys: id %u type %u cipher 0x%x len %d %pM\n",
		 key_index, type, params->cipher, params->key_len, mac_addr);

	return wlbt_mlme_setkeys(w, key_index, type, mac_addr, params->cipher,
				 params->key, params->key_len, params->seq,
				 params->seq_len);
}

static int wlbt_cfg_del_key(struct wiphy *wiphy, struct net_device *ndev,
			    int link_id, u8 key_index, bool pairwise,
			    const u8 *mac_addr)
{
	/* This firmware has no delete-keys request and the vendor driver does
	 * not send one either: a key is replaced by setting it again.
	 */
	return 0;
}
static const struct cfg80211_ops wlbt_cfg80211_ops = {
	.scan = wlbt_cfg_scan,
	.connect = wlbt_cfg_connect,
	.disconnect = wlbt_cfg_disconnect,
	.add_key = wlbt_cfg_add_key,
	.del_key = wlbt_cfg_del_key,
	.set_pmk = wlbt_cfg_set_pmk,
	.del_pmk = wlbt_cfg_del_pmk,
};

/*
 * The data path is not there yet, so the interface reports no carrier and
 * drops whatever it is handed.  It still has to exist: registering a netdev
 * goes through netdev_ops whatever the driver is able to do afterwards.
 */
static int wlbt_net_open(struct net_device *ndev)
{
	struct exynos_wlbt *w = wlbt_from_netdev(ndev);

	/*
	 * The interface is usually brought up before the join, but it can also be
	 * cycled afterwards, and the carrier then reflects the link that already
	 * exists rather than being cleared.
	 */
	if (w->connected)
		netif_carrier_on(ndev);
	else
		netif_carrier_off(ndev);
	netif_start_queue(ndev);
	return 0;
}

static int wlbt_net_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	return 0;
}

/*
 * The peer takes ethernet frames and does the 802.11 conversion itself, so a
 * transmit is the frame handed over on the data queue with a MA_UNITDATA_REQ in
 * front of it.  The body names the interface, a tag the confirm echoes, the
 * contention priority and the peer, and says the payload is an 802.3 frame.
 */
static netdev_tx_t wlbt_net_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct exynos_wlbt *w = wlbt_from_netdev(ndev);
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 24] = { };
	struct wlbt_fapi_hdr hdr = { };
	int ret;

	if (!w->hip4_control || !w->connected || skb->len < ETH_HLEN) {
		dev_kfree_skb(skb);
		w->n_tx_dropped++;
		return NETDEV_TX_OK;
	}

	w->n_tx_calls++;
	hdr.id = cpu_to_le16(FAPI_MA_UNITDATA_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));
	/*
	 * The header is the packed ten byte signal header, and the body of a
	 * unitdata request starts with the interface index, so this word is both
	 * the header's vif and the first word of the body.
	 */
	put_unaligned_le16(1, sig + sizeof(hdr) + 0);		/* vif */
	put_unaligned_le16(w->host_tag++, sig + sizeof(hdr) + 2);
	put_unaligned_le16(FAPI_PRIORITY_CONTENTION, sig + sizeof(hdr) + 4);
	put_unaligned_le16(1, sig + sizeof(hdr) + 6);		/* peer index */
	put_unaligned_le16(FAPI_DATAUNIT_IEEE802_3, sig + sizeof(hdr) + 8);

	ret = wlbt_hip4_send(w, false, sig, sizeof(sig), skb->data, skb->len);
	if (ret) {
		w->n_tx_dropped++;
		dev_info_ratelimited(w->dev, "tx: send failed: %d\n", ret);
	} else {
		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += skb->len;
	}

	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops wlbt_netdev_ops = {
	.ndo_open = wlbt_net_open,
	.ndo_stop = wlbt_net_stop,
	.ndo_start_xmit = wlbt_net_xmit,
};

static const u8 wlbt_mac[ETH_ALEN] = { 0x8c, 0x77, 0x12, 0xaa, 0xbb, 0xcc };

static int wlbt_register_wiphy(struct exynos_wlbt *w)
{
	struct wireless_dev *wdev;
	struct net_device *ndev;
	struct wiphy *wiphy;
	int ret;

	wiphy = wiphy_new(&wlbt_cfg80211_ops, sizeof(struct exynos_wlbt *));
	if (!wiphy)
		return -ENOMEM;

	*(struct exynos_wlbt **)wiphy_priv(wiphy) = w;
	set_wiphy_dev(wiphy, w->dev);
	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	wiphy->max_scan_ssids = 1;
	wiphy->bands[NL80211_BAND_2GHZ] = &wlbt_band_2ghz;
	wiphy->cipher_suites = wlbt_cipher_suites;
	wiphy->n_cipher_suites = ARRAY_SIZE(wlbt_cipher_suites);
	wiphy->max_num_akm_suites = 8;
	/* The supplicant names the EAPOL ethertype it wants; accepting the
	 * attribute is enough for now, the peer's data path is not there yet.
	 */
	wiphy->flags |= WIPHY_FLAG_CONTROL_PORT_PROTOCOL;

	ret = wiphy_register(wiphy);
	if (ret)
		goto err_free;

	ndev = alloc_netdev(sizeof(*wdev), "wlan%d", NET_NAME_ENUM, ether_setup);
	if (!ndev) {
		ret = -ENOMEM;
		goto err_unregister;
	}

	wdev = netdev_priv(ndev);
	wdev->wiphy = wiphy;
	wdev->iftype = NL80211_IFTYPE_STATION;
	ndev->ieee80211_ptr = wdev;
	ndev->netdev_ops = &wlbt_netdev_ops;
	SET_NETDEV_DEV(ndev, w->dev);
	eth_hw_addr_set(ndev, wlbt_mac);

	ret = cfg80211_register_netdevice(ndev);
	if (ret) {
		free_netdev(ndev);
		goto err_unregister;
	}

	w->wiphy = wiphy;
	w->netdev = ndev;
	INIT_DELAYED_WORK(&w->scan_work, wlbt_scan_work);
	INIT_DELAYED_WORK(&w->connect_work, wlbt_connect_work);
	INIT_WORK(&w->peer_work, wlbt_peer_work);
	dev_info(w->dev, "registered %s\n", netdev_name(ndev));
	return 0;

err_unregister:
	wiphy_unregister(wiphy);
err_free:
	wiphy_free(wiphy);
	return ret;
}

static void wlbt_unregister_wiphy(struct exynos_wlbt *w)
{
	if (!w->wiphy)
		return;

	cancel_delayed_work_sync(&w->scan_work);
	cancel_delayed_work_sync(&w->connect_work);
	if (w->scan_req) {
		struct cfg80211_scan_info info = { .aborted = true };

		cfg80211_scan_done(w->scan_req, &info);
		w->scan_req = NULL;
	}
	if (w->netdev) {
		cfg80211_unregister_netdevice(w->netdev);
		free_netdev(w->netdev);
		w->netdev = NULL;
	}
	wiphy_unregister(w->wiphy);
	wiphy_free(w->wiphy);
	w->wiphy = NULL;
}

/*
 * A warm reboot does not reset the WLBT block: its power configuration lives in
 * the PMU and the peer keeps running across the reset of the SoC.  Leaving it
 * like that is how a boot turns into a hang, so shut the peer down on the way
 * out as well.
 */
static int wlbt_reboot_notifier(struct notifier_block *nb, unsigned long action,
				void *data)
{
	struct exynos_wlbt *w = container_of(nb, struct exynos_wlbt, reboot_nb);

	if (!mutex_trylock(&w->lock))
		return NOTIFY_DONE;
	wlbt_power_down(w);
	mutex_unlock(&w->lock);

	return NOTIFY_DONE;
}

static int wlbt_probe(struct platform_device *pdev)
{
	struct exynos_wlbt *w;
	int irq_cfg, irq_mbox, irq_wdog;
	int ret;

	w = devm_kzalloc(&pdev->dev, sizeof(*w), GFP_KERNEL);
	if (w)
		w->trace = devm_kzalloc(&pdev->dev,
					WLBT_TRACE_MAX * sizeof(*w->trace),
					GFP_KERNEL);
	if (!w)
		return -ENOMEM;

	w->dev = &pdev->dev;
	platform_set_drvdata(pdev, w);
	mutex_init(&w->lock);
	spin_lock_init(&w->hip4_lock);
	init_completion(&w->cfg_ack);
	w->state = WLBT_IN_RESET;

	w->mbox = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(w->mbox))
		return PTR_ERR(w->mbox);

	w->pmu = wlbt_syscon(w, "samsung,syscon-phandle");
	if (IS_ERR(w->pmu))
		return PTR_ERR(w->pmu);
	wlbt_pw_cfg_save(w);

	/*
	 * The window controllers are optional here: they are only needed once
	 * the peer asks for its configuration, and a board without them should
	 * still be able to probe and report why the handshake cannot happen.
	 */
	w->dbus_baaw = wlbt_syscon(w, "samsung,dbus_baaw-syscon-phandle");
	w->pbus_baaw = wlbt_syscon(w, "samsung,pbus_baaw-syscon-phandle");
	w->remap = wlbt_syscon(w, "samsung,wlbt_remap-syscon-phandle");
	w->boot_cfg = wlbt_syscon(w, "samsung,boot_cfg-syscon-phandle");
	if (IS_ERR(w->dbus_baaw) || IS_ERR(w->pbus_baaw) ||
	    IS_ERR(w->remap) || IS_ERR(w->boot_cfg))
		return -EPROBE_DEFER;

	ret = wlbt_parse_memory(w);
	if (ret)
		return ret;

	irq_cfg = platform_get_irq_byname(pdev, "CFG_REQ");
	if (irq_cfg < 0)
		return irq_cfg;
	irq_mbox = platform_get_irq_byname(pdev, "MBOX");
	if (irq_mbox < 0)
		return irq_mbox;
	irq_wdog = platform_get_irq_byname(pdev, "WDOG");
	if (irq_wdog < 0)
		return irq_wdog;

	/* The state has to be correct before the handler can run; requesting
	 * the interrupt is itself the barrier that guarantees it.
	 */
	w->state = WLBT_IN_RESET;

	ret = devm_request_irq(w->dev, irq_cfg, wlbt_cfg_req_isr, IRQF_TRIGGER_HIGH,
			       "wlbt-cfg-req", w);
	if (ret) {
		dev_err(w->dev, "cannot request the CFG_REQ interrupt: %d\n", ret);
		return ret;
	}

	/* WLBT watchdog and doorbell will only matter once the transport is
	 * there, but taking them keeps a level interrupt from storming.
	 */
	ret = devm_request_irq(w->dev, irq_wdog, wlbt_wdog_isr, IRQF_TRIGGER_HIGH,
			       "wlbt-wdog", w);
	if (ret)
		return ret;

	ret = devm_request_irq(w->dev, irq_mbox, wlbt_mbox_isr, IRQF_TRIGGER_HIGH,
			       "wlbt-mbox", w);
	if (ret)
		return ret;

	wlbt_debugfs_init(w);

	w->reboot_nb.notifier_call = wlbt_reboot_notifier;
	if (devm_register_reboot_notifier(w->dev, &w->reboot_nb))
		dev_warn(w->dev, "cannot register the reboot notifier\n");

	/* The interface is registered even when the peer is not started: the
	 * scan op reports -EIO until the transport exists, which is a better
	 * answer than no interface at all.
	 */
	ret = wlbt_register_wiphy(w);
	if (ret)
		dev_warn(w->dev, "no wireless interface: %d\n", ret);

	dev_info(w->dev, "mailbox at %pR, MIF DRAM %pa (%pa)\n",
		 platform_get_resource(pdev, IORESOURCE_MEM, 0),
		 &w->mem_base, &w->mem_size);

	wlbt_boot(w);

	return 0;
}

static void wlbt_remove(struct platform_device *pdev)
{
	struct exynos_wlbt *w = platform_get_drvdata(pdev);

	wlbt_unregister_wiphy(w);

	mutex_lock(&w->lock);
	w->vif_created = false;
	wlbt_power_down(w);
	mutex_unlock(&w->lock);

	debugfs_remove(w->debugfs);

	if (w->dram)
		memunmap(w->dram);
}

static const struct of_device_id wlbt_of_match[] = {
	{ .compatible = "samsung,scsc_wifibt" },
	{ }
};
MODULE_DEVICE_TABLE(of, wlbt_of_match);

static struct platform_driver wlbt_driver = {
	.probe = wlbt_probe,
	.remove = wlbt_remove,
	.driver = {
		.name = "exynos9610-wlbt",
		.of_match_table = wlbt_of_match,
	},
};
module_platform_driver(wlbt_driver);

MODULE_DESCRIPTION("Samsung Exynos 9610 SCSC/WLBT power and boot handshake");
MODULE_LICENSE("GPL");
