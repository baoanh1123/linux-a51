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
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
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
#define WLBT_BIT_MGMT_IN_WRITE		0
#define WLBT_BIT_MGMT_OUT_WRITE		1
#define WLBT_BIT_GDB_R4_IN_WRITE	2
#define WLBT_BIT_GDB_R4_OUT_WRITE	3
#define WLBT_BIT_GDB_M4_IN_WRITE	4
#define WLBT_BIT_GDB_M4_OUT_WRITE	5
#define WLBT_BIT_LOG_WRITE		6

#define WLBT_BIT_MGMT_IN_READ		0
#define WLBT_BIT_MGMT_OUT_READ		1
#define WLBT_BIT_GDB_R4_IN_READ		2
#define WLBT_BIT_GDB_R4_OUT_READ	0	/* the bit reserved for panics */
#define WLBT_BIT_LOG_READ		3
#define WLBT_BIT_GDB_M4_IN_READ		0
#define WLBT_BIT_GDB_M4_OUT_READ	0	/* the bit reserved for panics */

#define WLBT_IRQ_BITS			(BIT(WLBT_BIT_MGMT_IN_WRITE) | \
					 BIT(WLBT_BIT_MGMT_OUT_WRITE) | \
					 BIT(WLBT_BIT_GDB_R4_IN_WRITE) | \
					 BIT(WLBT_BIT_GDB_R4_OUT_WRITE) | \
					 BIT(WLBT_BIT_GDB_M4_IN_WRITE) | \
					 BIT(WLBT_BIT_GDB_M4_OUT_WRITE) | \
					 BIT(WLBT_BIT_LOG_WRITE))

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
	unsigned int n_mbox;
	u32 last_mbox_status;
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

static char *firmware_name = "mx140/mx140.bin";
module_param(firmware_name, charp, 0444);
MODULE_PARM_DESC(firmware_name, "Firmware image for the WLBT subsystem");

static bool load_firmware;
module_param(load_firmware, bool, 0644);
MODULE_PARM_DESC(load_firmware, "Copy the firmware image into MIF DRAM and let the peer run it");

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

	w->n_wdog++;
	dev_warn(w->dev, "WLBT watchdog reset request\n");

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
static u32 wlbt_heap_alloc(struct exynos_wlbt *w, u32 bytes)
{
	u32 offset;

	bytes = ALIGN(bytes, 4);
	if (!w->heap_next || w->heap_next + bytes > w->mem_size) {
		dev_err(w->dev, "MIF heap is full (%u of %pa used)\n",
			w->heap_next, &w->mem_size);
		return 0;
	}

	offset = w->heap_next;
	w->heap_next += bytes;
	memset_io(w->dram + offset, 0, bytes);

	return offset;
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
				 struct wlbt_stream_conf *conf, bool to_ap)
{
	conf->buf_conf.buffer_loc = s->buf;
	conf->buf_conf.num_packets = s->num_packets;
	conf->buf_conf.packet_size = s->packet_size;
	conf->buf_conf.read_index_loc = s->ridx;
	conf->buf_conf.write_index_loc = s->widx;
	if (to_ap) {
		conf->read_bit_idx = s->read_bit;
		conf->write_bit_idx = s->write_bit;
	} else {
		conf->read_bit_idx = s->write_bit;
		conf->write_bit_idx = s->read_bit;
	}
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
	ret = wlbt_stream_init(w, &w->log, "log", LOG_NUM_PACKETS,
			       LOG_PACKET_SIZE, WLBT_BIT_LOG_READ,
			       WLBT_BIT_LOG_WRITE, true);
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
	conf->soc_revision = 0;

	wlbt_stream_serialise(&w->mgmt_in, &conf->trans.to_ap, true);
	wlbt_stream_serialise(&w->mgmt_out, &conf->trans.from_ap, false);
	wlbt_stream_serialise(&w->gdb_r4_in, &conf->gdb_r4.to_ap, true);
	wlbt_stream_serialise(&w->gdb_r4_out, &conf->gdb_r4.from_ap, false);
	wlbt_stream_serialise(&w->gdb_m4_in, &conf->gdb_m4.to_ap, true);
	wlbt_stream_serialise(&w->gdb_m4_out, &conf->gdb_m4.from_ap, false);
	wlbt_stream_serialise(&w->log, &conf->logconf.stream_conf, true);

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

static ssize_t wlbt_debugfs_reset(struct file *file, const char __user *buf,
				  size_t len, loff_t *ppos)
{
	struct exynos_wlbt *w = file->private_data;
	int ret;

	mutex_lock(&w->lock);
	ret = wlbt_power_down(w);
	mutex_unlock(&w->lock);

	return ret ? ret : len;
}

#define WLBT_DEBUGFS_OPS(_write)					\
	{								\
		.owner = THIS_MODULE,					\
		.open = simple_open,					\
		.write = (_write),					\
		.llseek = noop_llseek,					\
	}

static const struct file_operations wlbt_boot_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_boot);
static const struct file_operations wlbt_reset_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_reset);
static const struct file_operations wlbt_halt_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_halt);

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

	seq_printf(s, "stream %-10s cells %4u x %3u  read %u write %u\n",
		   st->name, st->num_packets, st->packet_size,
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

	for (i = 0; i < MBOX_NUM_ISSR; i++)
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

static int wlbt_probe(struct platform_device *pdev)
{
	struct exynos_wlbt *w;
	int irq_cfg, irq_mbox, irq_wdog;
	int ret;

	w = devm_kzalloc(&pdev->dev, sizeof(*w), GFP_KERNEL);
	if (!w)
		return -ENOMEM;

	w->dev = &pdev->dev;
	platform_set_drvdata(pdev, w);
	mutex_init(&w->lock);
	init_completion(&w->cfg_ack);
	w->state = WLBT_IN_RESET;

	w->mbox = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(w->mbox))
		return PTR_ERR(w->mbox);

	w->pmu = wlbt_syscon(w, "samsung,syscon-phandle");
	if (IS_ERR(w->pmu))
		return PTR_ERR(w->pmu);

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

	dev_info(w->dev, "mailbox at %pR, MIF DRAM %pa (%pa)\n",
		 platform_get_resource(pdev, IORESOURCE_MEM, 0),
		 &w->mem_base, &w->mem_size);

	wlbt_boot(w);

	return 0;
}

static void wlbt_remove(struct platform_device *pdev)
{
	struct exynos_wlbt *w = platform_get_drvdata(pdev);

	mutex_lock(&w->lock);
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
