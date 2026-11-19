// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos 9610/9611 DSIM (MIPI DSI master) - handoff stage.
 *
 * The Samsung boot loader initialises DSIM0, the D-PHY and the panel before it
 * starts uniLoader, and leaves the link running in MIPI command mode with the
 * panel's own timing.  This driver keeps that inherited link configuration and
 * adds the packet paths on top of it: DCS reads for identification, DCS writes
 * for panel control (brightness so far), and the stop/restore helpers used by
 * the DPMS work.  The boot loader's link init is still authoritative - see
 * notes/dsim.md for why replaying registers is not enough to bring the link
 * back up.
 *
 * Register semantics follow the vendor source at
 * DOWNSTREAM_READONLY/drivers/video/fbdev/exynos/dpu20/cal_9610/regs-dsim.h and
 * dsim_reg.c; see notes/dsim.md for the captured state and the staged plan.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>

#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>

#include <video/mipi_display.h>

#include "exynos9610_dpu.h"
#include "exynos9610_dsim_dphy.h"

#define DSIM_VERSION			0x000
#define DSIM_LINK_STATUS0		0x008
#define DSIM_LINK_STATUS1		0x00c
#define DSIM_LINK_STATUS3		0x014
#define DSIM_MIPI_STATUS		0x018
#define DSIM_DPHY_STATUS		0x01c
#define DSIM_CLK_CTRL			0x020
#define DSIM_NUM_OF_TRANSFER		0x030
#define DSIM_RESOL			0x03c
#define DSIM_VPORCH			0x040
#define DSIM_CONFIG			0x04c
#define DSIM_PLLCTRL			0x0a0
#define DSIM_PLL_CTRL1			0x0a4
#define DSIM_PLL_CTRL2			0x0a8
#define DSIM_PLLTMR			0x0ac
#define DSIM_PHYCTRL_B1			0x0b0
#define DSIM_PHYCTRL_B3			0x0b8
#define DSIM_PHYCTRL_B4			0x0bc
#define DSIM_PHYCTRL_M1			0x0c0
#define DSIM_PHYCTRL_M2			0x0c4
#define DSIM_PHYCTRL_M3			0x0c8
#define DSIM_PHYCTRL_M4			0x0cc
#define DSIM_PHY_TIMING			0x0d0
#define DSIM_PHY_TIMING1		0x0d4
#define DSIM_PHY_TIMING2		0x0d8
#define DSIM_ESCMODE			0x02c
#define DSIM_TIMEOUT			0x028
#define DSIM_PHYCTRL_B2			0x0b4
#define DSIM_SWRST			0x004
#define DSIM_SWRST_RESET		BIT(0)
#define DSIM_SWRST_DPHY_RST		BIT(16)
#define DSIM_INTSRC			0x050
#define DSIM_INTMSK			0x054
#define DSIM_PKTHDR			0x058
#define DSIM_PAYLOAD			0x05c
#define DSIM_RXFIFO			0x060
#define DSIM_FIFOCTRL			0x068
#define DSIM_NUM_OF_TRANSFER		0x030
#define DSIM_UNDERRUN_CTRL		0x034
#define DSIM_THRESHOLD			0x038
#define DSIM_VPORCH			0x040
#define DSIM_HPORCH			0x044
#define DSIM_SFR_CTRL			0x064

#define DSIM_LINK_STATUS1_CMD_MODE	BIT(26)
#define DSIM_LINK_STATUS3_PLL_STABLE	BIT(0)
#define DSIM_DPHY_STATUS_TX_READY_CLK	BIT(10)

#define DSIM_CLK_CTRL_CLOCK_SEL		BIT(26)
#define DSIM_CLK_CTRL_WORDCLK_EN	BIT(17)
#define DSIM_CLK_CTRL_ESCCLK_EN		BIT(16)
#define DSIM_CLK_CTRL_ESC_PRESCALER(v)	((v) & 0xff)
#define DSIM_CLK_CTRL_LANE_ESCCLK_EN(v)	((v) << 8)
#define DSIM_CLK_CTRL_TX_REQUEST_HSCLK	BIT(20)

#define DSIM_CONFIG_BURST_MODE		BIT(26)
#define DSIM_CONFIG_VIDEO_MODE		BIT(18)
#define DSIM_CONFIG_PIXEL_FORMAT(v)	(((v) >> 9) & 0x3f)
#define DSIM_CONFIG_DATA_LANES(v)	((((v) >> 5) & 0x3) + 1)
/* Lane enable field: one bit per clock and data lane */
#define DSIM_CONFIG_LANES_EN(v)		((v) & 0x1f)
#define DSIM_CONFIG_NUM_OF_DATA_LANE(v)	(((v) & 0x3) << 5)
#define DSIM_CONFIG_EOTP_EN		BIT(7)

#define DSIM_RESOL_VRESOL(v)		(((v) >> 16) & 0x1fff)
#define DSIM_RESOL_HRESOL(v)		((v) & 0x1fff)

#define DSIM_PLLCTRL_PLL_EN		BIT(23)
#define DSIM_PLLCTRL_PMS(p, m, s)	((((p) & 0x3f) << 13) |	\
					 (((m) & 0x3ff) << 3) |	\
					 ((s) & 0x7))
#define DSIM_PLLCTRL_P(v)		(((v) >> 13) & 0x3f)
#define DSIM_PLLCTRL_M(v)		(((v) >> 3) & 0x3ff)
#define DSIM_PLLCTRL_S(v)		((v) & 0x7)

#define DSIM_REF_CLK_HZ			26000000ULL

#define DSIM_INTSRC_RX_DATA_DONE	BIT(18)
#define DSIM_INTSRC_RX_TE		BIT(17)
#define DSIM_INTSRC_RX_ACK		BIT(16)
#define DSIM_INTSRC_BTA_TOUT		BIT(20)
#define DSIM_INTSRC_LPRX_TOUT		BIT(21)
#define DSIM_INTSRC_ERR_RX_ECC		BIT(15)
#define DSIM_INTSRC_RX_CRC		BIT(14)

#define DSIM_MIPI_STATUS_TE		BIT(0)
#define DSIM_CMD_TE_CTRL0		0x084
#define DSIM_CMD_TE_CTRL1		0x088

#define DSIM_FIFOCTRL_EMPTY_RX		BIT(12)
#define DSIM_FIFOCTRL_INIT_RX		BIT(2)
#define DSIM_FIFOCTRL_EMPTY_PH_SFR	BIT(10)
#define DSIM_FIFOCTRL_EMPTY_PL_SFR	BIT(8)

#define DSIM_PKTHDR_BTA_TYPE(v)		((v) << 24)
#define DSIM_PKTHDR_DATA1(v)		((v) << 16)
#define DSIM_PKTHDR_DATA0(v)		((v) << 8)
#define DSIM_PKTHDR_ID(v)		((v) << 0)
#define DSIM_PKTHDR_DATA		GENMASK(24, 0)

/* Vendor values from dpu20/cal_9610/dsim_reg.c */
#define DSIM_STOP_STATE_CNT		0xa
#define DSIM_BTA_TIMEOUT		0xff
#define DSIM_LP_RX_TIMEOUT		0xffff
#define DSIM_PLL_STABLE_TIME		0x13880
#define DSIM_DPHY_CHARIC_M_PLL_CTRL1	0x0
#define DSIM_DPHY_CHARIC_M_PLL_CTRL2	0x0
#define DSIM_DPHY_CHARIC_B_DPHY_CTRL2	0x800
#define DSIM_DPHY_CHARIC_B_DPHY_CTRL3	0x0
#define DSIM_DPHY_CHARIC_B_DPHY_CTRL4	0x40
#define DSIM_DPHY_CHARIC_M_DPHY_CTRL1	0x0
#define DSIM_DPHY_CHARIC_M_DPHY_CTRL2	0x0
#define DSIM_DPHY_CHARIC_M_DPHY_CTRL3	0x0
#define DSIM_DPHY_CHARIC_M_DPHY_CTRL4	0x0
#define DSIM_DEFAULT_HS_CLK_MHZ		1200
#define DSIM_DEFAULT_ESC_CLK_MHZ	16

#define DSIM_POLL_TIMEOUT_US		100000
#define DSIM_POLL_INTERVAL_US		10

#define DSIM_SWRST_RESET		BIT(0)
#define DSIM_DPHY_STATUS_STOP_STATE_CLK	BIT(8)
#define DSIM_CLK_CTRL_TX_REQUEST_HSCLK	BIT(20)
#define DSIM_CLK_CTRL_CLOCK_SEL		BIT(26)
#define DSIM_CLK_CTRL_LANE_ESCCLK_EN_MASK	(0x1f << 8)
#define DSIM_CONFIG_LANES_MASK		GENMASK(4, 0)
#define DSIM_CONFIG_DATA_LANES_MASK	GENMASK(6, 5)

#define DSIM_SFR_LEN			0x100

/* PMU: M4S4 D-PHY isolation bypass (1 = powered) */
#define PMU_MIPI_PHY_CON		0x70c
#define PMU_MIPI_PHY_ISO_BYPASS		BIT(0)

/* SYSREG_DISPAUD: DISP_DPU_MIPI_PHY_CON */
#define SYSREG_DPU_MIPI_PHY_CON		0x08
#define SYSREG_M_RESETN_M4S4_TOP	BIT(0)	/* 1 = reset released */
#define SYSREG_SEL_RESET_DPHY_DSIM0	BIT(4)	/* 0 = DSIM drives the reset */

/**
 * struct exynos9610_dsim - DSIM handoff state
 * @dev: device
 * @base: DSIM SFR window
 * @debugfs: debugfs directory for the bring-up helpers
 * @last_read: result text of the last DCS read
 */
struct exynos9610_dsim {
	struct device *dev;
	void __iomem *base;
	struct mipi_dsi_host host;
	struct drm_bridge *link_bridge;
	struct dentry *debugfs;
	struct regmap *pmu;
	struct regmap *sysreg;
	u32 snapshot[DSIM_SFR_LEN / 4];
	u32 lanes;
	u32 last_intsrc;
	int irq;
	atomic64_t te_count;
	atomic64_t te_other;
	unsigned int brightness;
	char last_read[240];
	char last_link[160];
};

static u32 exynos9610_dsim_read(struct exynos9610_dsim *dsim, u32 reg)
{
	return readl(dsim->base + reg);
}

/*
 * Packet FIFO access.  The DSIM exposes two FIFOs: a header FIFO (one word per
 * packet, encoding data type and the two short-packet parameters) and a payload
 * FIFO for long packets.  Both have to be empty before a new header is pushed,
 * and a read request must carry the BTA flag so the peripheral turns the link
 * around.  Semantics per vendor dsim_drv.c / dsim_reg.c.
 */
static int exynos9610_dsim_wait_fifo(struct exynos9610_dsim *dsim, u32 mask)
{
	unsigned int timeout = DSIM_POLL_TIMEOUT_US;

	while (!(exynos9610_dsim_read(dsim, DSIM_FIFOCTRL) & mask)) {
		if (!timeout)
			return -ETIMEDOUT;

		usleep_range(DSIM_POLL_INTERVAL_US, 2 * DSIM_POLL_INTERVAL_US);
		timeout -= DSIM_POLL_INTERVAL_US;
	}

	return 0;
}

static int exynos9610_dsim_send(struct exynos9610_dsim *dsim, u32 type, u32 data0,
				u32 data1, bool bta, const u8 *payload, u32 len)
{
	u32 word, i;
	int ret;

	ret = exynos9610_dsim_wait_fifo(dsim, DSIM_FIFOCTRL_EMPTY_PH_SFR);
	if (ret)
		return ret;

	for (i = 0; i < len; i += 4) {
		word = payload[i];
		if (i + 1 < len)
			word |= payload[i + 1] << 8;
		if (i + 2 < len)
			word |= payload[i + 2] << 16;
		if (i + 3 < len)
			word |= payload[i + 3] << 24;

		writel(word, dsim->base + DSIM_PAYLOAD);
	}

	writel(DSIM_PKTHDR_BTA_TYPE(bta) | DSIM_PKTHDR_ID(type) |
	       DSIM_PKTHDR_DATA0(data0) | DSIM_PKTHDR_DATA1(data1),
	       dsim->base + DSIM_PKTHDR);

	return 0;
}

/*
 * Read DCS registers back from the panel through the inherited link.  The
 * command does not change link or panel state, so it is the first DSIM write
 * that is safe to perform on a running system: it either proves the packet
 * path works or times out with the SFR state reported.
 */
static int exynos9610_dsim_dcs_read(struct exynos9610_dsim *dsim, u8 cmd,
				    u8 len, u32 *words, unsigned int max_words,
				    unsigned int *nwords)
{
	unsigned int timeout, count = 0;
	u32 intsrc;
	int ret;

	/* Tell the peripheral how many bytes we are going to read back */
	ret = exynos9610_dsim_send(dsim, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
				   len, 0, false, NULL, 0);
	if (ret)
		return ret;

	/* Start from an empty receive FIFO, the boot chain may have left data */
	writel(DSIM_FIFOCTRL_INIT_RX, dsim->base + DSIM_FIFOCTRL);

	writel(DSIM_INTSRC_RX_DATA_DONE | DSIM_INTSRC_RX_ACK |
	       DSIM_INTSRC_RX_TE | DSIM_INTSRC_BTA_TOUT |
	       DSIM_INTSRC_LPRX_TOUT | DSIM_INTSRC_ERR_RX_ECC |
	       DSIM_INTSRC_RX_CRC, dsim->base + DSIM_INTSRC);

	ret = exynos9610_dsim_send(dsim, MIPI_DSI_DCS_READ, cmd, 0, true, NULL, 0);
	if (ret)
		return ret;

	timeout = DSIM_POLL_TIMEOUT_US;
	for (;;) {
		intsrc = exynos9610_dsim_read(dsim, DSIM_INTSRC);
		if (intsrc & (DSIM_INTSRC_RX_DATA_DONE | DSIM_INTSRC_BTA_TOUT |
			      DSIM_INTSRC_LPRX_TOUT | DSIM_INTSRC_RX_CRC |
			      DSIM_INTSRC_ERR_RX_ECC))
			break;

		if (!timeout)
			return -ETIMEDOUT;

		usleep_range(DSIM_POLL_INTERVAL_US, 2 * DSIM_POLL_INTERVAL_US);
		timeout -= DSIM_POLL_INTERVAL_US;
	}

	writel(intsrc, dsim->base + DSIM_INTSRC);

	dsim->last_intsrc = intsrc;

	if (!(intsrc & DSIM_INTSRC_RX_DATA_DONE))
		return (intsrc & DSIM_INTSRC_BTA_TOUT) ? -ETIMEDOUT : -EIO;

	while (!(exynos9610_dsim_read(dsim, DSIM_FIFOCTRL) & DSIM_FIFOCTRL_EMPTY_RX)) {
		if (count == max_words)
			break;

		words[count++] = exynos9610_dsim_read(dsim, DSIM_RXFIFO);
	}

	*nwords = count;

	return 0;
}

/*
 * Link power down and power up.
 *
 * The boot chain leaves a complete, locked configuration behind, and linux
 * keeps running on top of it; this is what makes the display work at all
 * without a full D-PHY init (see notes/dsim.md).  The sequences below are the
 * vendor dsim_reg_stop() / dsim_reg_init() ones, so the link can now be powered
 * down and brought back up by the driver itself.
 *
 * What is *not* replayed here is the DSIM configuration proper (resolution,
 * porches, command mode triggers, packet thresholds): those registers survive
 * the link power down, and re-deriving them from the display mode is a separate
 * step.  A full power-off of the DSIM block therefore still needs the boot
 * loader - noted as remaining work in notes/todo.md.
 */
static void exynos9610_dsim_snapshot(struct exynos9610_dsim *dsim)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dsim->snapshot); i++)
		dsim->snapshot[i] = exynos9610_dsim_read(dsim, i * 4);
}

static int exynos9610_dsim_wait_bit(struct exynos9610_dsim *dsim, u32 reg,
				    u32 mask, bool set)
{
	unsigned int timeout = DSIM_POLL_TIMEOUT_US;

	for (;;) {
		bool state = exynos9610_dsim_read(dsim, reg) & mask;

		if (state == set)
			return 0;

		if (!timeout)
			return -ETIMEDOUT;

		usleep_range(DSIM_POLL_INTERVAL_US, 2 * DSIM_POLL_INTERVAL_US);
		timeout -= DSIM_POLL_INTERVAL_US;
	}
}

static void exynos9610_dsim_update(struct exynos9610_dsim *dsim, u32 reg,
				   u32 mask, u32 val)
{
	u32 old = exynos9610_dsim_read(dsim, reg);

	writel((old & ~mask) | (val & mask), dsim->base + reg);
}

/* Mirror of the vendor dsim_reg_stop() */
static int exynos9610_dsim_link_stop(struct exynos9610_dsim *dsim)
{
	writel(~0, dsim->base + DSIM_INTSRC);
	writel(~0, dsim->base + DSIM_INTMSK);

	/* HS clock off first, and wait for the clock lane to reach stop state */
	exynos9610_dsim_update(dsim, DSIM_CLK_CTRL,
			       DSIM_CLK_CTRL_TX_REQUEST_HSCLK, 0);
	exynos9610_dsim_wait_bit(dsim, DSIM_DPHY_STATUS,
				 DSIM_DPHY_STATUS_STOP_STATE_CLK, true);

	/* Clock selection back to the oscillator, lanes and escape clock off */
	exynos9610_dsim_update(dsim, DSIM_CLK_CTRL,
			       DSIM_CLK_CTRL_CLOCK_SEL |
			       DSIM_CLK_CTRL_LANE_ESCCLK_EN_MASK, 0);
	exynos9610_dsim_update(dsim, DSIM_CONFIG, DSIM_CONFIG_LANES_MASK, 0);

	/* Then the PLL, its prescaler and finally the block reset */
	exynos9610_dsim_update(dsim, DSIM_PLLCTRL, DSIM_PLLCTRL_PLL_EN, 0);
	exynos9610_dsim_update(dsim, DSIM_CLK_CTRL,
			       DSIM_CLK_CTRL_ESCCLK_EN |
			       DSIM_CLK_CTRL_ESC_PRESCALER(~0), 0);

	writel(DSIM_SWRST_RESET, dsim->base + DSIM_SWRST);
	exynos9610_dsim_wait_bit(dsim, DSIM_SWRST, DSIM_SWRST_RESET, false);

	return 0;
}

/*
 * A link power cycle clears the frame configuration of the controller.  These
 * carry the panel's resolution, porches and command mode timing values, and
 * until they are derived from the display mode they are restored from the
 * configuration the boot chain left behind.
 */
static const u32 exynos9610_dsim_config_regs[] = {
	DSIM_NUM_OF_TRANSFER,
	DSIM_UNDERRUN_CTRL,
	DSIM_THRESHOLD,
	DSIM_RESOL,
	DSIM_VPORCH,
	DSIM_HPORCH,
	DSIM_SFR_CTRL,
	DSIM_CMD_TE_CTRL0,
	DSIM_CMD_TE_CTRL1,
};

static void exynos9610_dsim_restore_config(struct exynos9610_dsim *dsim)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(exynos9610_dsim_config_regs); i++) {
		u32 reg = exynos9610_dsim_config_regs[i];

		writel(dsim->snapshot[reg / 4], dsim->base + reg);
	}
}

static int exynos9610_dsim_pll_start(struct exynos9610_dsim *dsim, u32 hs_clk_mhz)
{
	struct exynos9610_dsim_pll pll;
	u32 ref_khz = DSIM_REF_CLK_HZ / 1000;
	u32 val;
	int ret;

	ret = exynos9610_dsim_pll_params(hs_clk_mhz * 1000, ref_khz, &pll);
	if (ret)
		return ret;

	val = DSIM_PLLCTRL_PMS(pll.p, pll.m, pll.s);
	writel(val, dsim->base + DSIM_PLLCTRL);

	/* Lock time, in reference clock cycles */
	writel(DSIM_PLL_STABLE_TIME, dsim->base + DSIM_PLLTMR);

	writel(val | DSIM_PLLCTRL_PLL_EN, dsim->base + DSIM_PLLCTRL);

	dev_dbg(dsim->dev, "DSIM PLL: p=%u m=%u s=%u -> %u kHz\n",
		pll.p, pll.m, pll.s, pll.freq_khz);

	return exynos9610_dsim_wait_bit(dsim, DSIM_LINK_STATUS3,
					DSIM_LINK_STATUS3_PLL_STABLE, true);
}

static void exynos9610_dsim_dphy_timings(struct exynos9610_dsim *dsim,
					 struct exynos9610_dsim_dphy_timing *t)
{
	writel((t->lpx << 8) | t->hs_exit, dsim->base + DSIM_PHY_TIMING);
	writel((t->clk_prepare << 24) | (t->clk_zero << 16) |
	       (t->clk_post << 8) | t->clk_trail,
	       dsim->base + DSIM_PHY_TIMING1);
	writel((t->hs_prepare << 16) | (t->hs_zero << 8) | t->hs_trail,
	       dsim->base + DSIM_PHY_TIMING2);
	writel(t->b_dphyctl, dsim->base + DSIM_PHYCTRL_B1);
}

/*
 * Mirror of the vendor dsim_reg_init() with panel_ctrl = false: assert the
 * D-PHY reset through SYSREG, reset the DSIM block, program escape clock,
 * timings and PLL, enable the lanes, then release the reset and switch the link
 * to the word clock.
 */
static int exynos9610_dsim_link_start(struct exynos9610_dsim *dsim)
{
	struct exynos9610_dsim_dphy_timing timing;
	u32 hs_clk = DSIM_DEFAULT_HS_CLK_MHZ;
	u32 esc_clk = DSIM_DEFAULT_ESC_CLK_MHZ;
	u32 esc_div, lanes, val;
	int ret;

	/*
	 * Like the vendor driver, leave a link that the boot loader already
	 * powered and locked alone: re-initialising the D-PHY under a running
	 * DECON pipe wedges it.
	 */
	if (exynos9610_dsim_read(dsim, DSIM_LINK_STATUS3) &
	    DSIM_LINK_STATUS3_PLL_STABLE) {
		dev_info(dsim->dev, "link already powered, keeping the inherited state\n");
		return 0;
	}

	of_property_read_u32(dsim->dev->of_node, "samsung,hs-clock-mhz", &hs_clk);
	of_property_read_u32(dsim->dev->of_node, "samsung,escape-clock-mhz",
			     &esc_clk);

	esc_div = exynos9610_dsim_esc_div(hs_clk, esc_clk);
	esc_clk = hs_clk / 16 / esc_div;
	lanes = dsim->lanes;

	ret = exynos9610_dsim_dphy_timing(hs_clk, esc_clk, &timing);
	if (ret) {
		dev_err(dsim->dev, "no D-PHY timings for %u MHz HS clock\n",
			hs_clk);
		return ret;
	}

	/* Oscillator as link clock, then hold the D-PHY in reset */
	exynos9610_dsim_update(dsim, DSIM_CLK_CTRL, DSIM_CLK_CTRL_CLOCK_SEL, 0);
	if (dsim->sysreg)
		regmap_update_bits(dsim->sysreg, SYSREG_DPU_MIPI_PHY_CON,
				   SYSREG_M_RESETN_M4S4_TOP, 0);

	writel(DSIM_SWRST_RESET, dsim->base + DSIM_SWRST);
	ret = exynos9610_dsim_wait_bit(dsim, DSIM_SWRST, DSIM_SWRST_RESET,
				       false);
	if (ret)
		return ret;

	/* Escape clock prescaler, D-PHY timings and the PHY characterisation */
	exynos9610_dsim_update(dsim, DSIM_CLK_CTRL,
			       DSIM_CLK_CTRL_ESCCLK_EN |
			       DSIM_CLK_CTRL_ESC_PRESCALER(~0),
			       DSIM_CLK_CTRL_ESCCLK_EN |
			       DSIM_CLK_CTRL_ESC_PRESCALER(esc_div));

	exynos9610_dsim_dphy_timings(dsim, &timing);

	writel(DSIM_DPHY_CHARIC_M_PLL_CTRL1, dsim->base + DSIM_PLL_CTRL1);
	writel(DSIM_DPHY_CHARIC_M_PLL_CTRL2, dsim->base + DSIM_PLL_CTRL2);
	writel(DSIM_DPHY_CHARIC_B_DPHY_CTRL2, dsim->base + DSIM_PHYCTRL_B2);
	writel(DSIM_DPHY_CHARIC_B_DPHY_CTRL3, dsim->base + DSIM_PHYCTRL_B3);
	writel(DSIM_DPHY_CHARIC_B_DPHY_CTRL4, dsim->base + DSIM_PHYCTRL_B4);
	writel(DSIM_DPHY_CHARIC_M_DPHY_CTRL1, dsim->base + DSIM_PHYCTRL_M1);
	writel(DSIM_DPHY_CHARIC_M_DPHY_CTRL2, dsim->base + DSIM_PHYCTRL_M2);
	writel(DSIM_DPHY_CHARIC_M_DPHY_CTRL3, dsim->base + DSIM_PHYCTRL_M3);
	writel(DSIM_DPHY_CHARIC_M_DPHY_CTRL4, dsim->base + DSIM_PHYCTRL_M4);

	/* Timeouts and stop state count */
	exynos9610_dsim_update(dsim, DSIM_TIMEOUT,
			       GENMASK(31, 16) | GENMASK(15, 0),
			       (DSIM_BTA_TIMEOUT << 16) | DSIM_LP_RX_TIMEOUT);
	exynos9610_dsim_update(dsim, DSIM_ESCMODE, GENMASK(31, 21),
			       DSIM_STOP_STATE_CNT << 21);

	ret = exynos9610_dsim_pll_start(dsim, hs_clk);
	if (ret)
		return ret;

	/*
	 * The lane count and EOTP fields are cleared when the lanes are turned
	 * off; the vendor driver restores them from the panel timing in
	 * dsim_reg_set_config(), which is not replayed here.
	 */
	val = DSIM_CONFIG_NUM_OF_DATA_LANE(lanes - 1) | DSIM_CONFIG_EOTP_EN |
	      DSIM_CONFIG_LANES_EN(BIT(lanes + 1) - 1);
	exynos9610_dsim_update(dsim, DSIM_CONFIG,
			       DSIM_CONFIG_LANES_MASK |
			       DSIM_CONFIG_DATA_LANES_MASK |
			       DSIM_CONFIG_EOTP_EN, val);
	/* The vendor driver waits for the lanes to settle before releasing reset */
	usleep_range(400, 500);

	/* Release the D-PHY reset, select the word clock, enable the clocks */
	if (dsim->sysreg)
		regmap_update_bits(dsim->sysreg, SYSREG_DPU_MIPI_PHY_CON,
				   SYSREG_M_RESETN_M4S4_TOP,
				   SYSREG_M_RESETN_M4S4_TOP);

	exynos9610_dsim_update(dsim, DSIM_CLK_CTRL, DSIM_CLK_CTRL_CLOCK_SEL,
			       DSIM_CLK_CTRL_CLOCK_SEL);
	exynos9610_dsim_update(dsim, DSIM_CLK_CTRL,
			       DSIM_CLK_CTRL_LANE_ESCCLK_EN_MASK,
			       DSIM_CLK_CTRL_LANE_ESCCLK_EN(BIT(lanes + 1) - 1));
	exynos9610_dsim_update(dsim, DSIM_CLK_CTRL, DSIM_CLK_CTRL_WORDCLK_EN,
			       DSIM_CLK_CTRL_WORDCLK_EN);

	/* Frame configuration: resolution, porches, TE timing, shadow mode */
	exynos9610_dsim_restore_config(dsim);

	/* Start: HS clock on and wait for the clock lane to become ready */
	exynos9610_dsim_update(dsim, DSIM_CLK_CTRL,
			       DSIM_CLK_CTRL_TX_REQUEST_HSCLK,
			       DSIM_CLK_CTRL_TX_REQUEST_HSCLK);

	ret = exynos9610_dsim_wait_bit(dsim, DSIM_DPHY_STATUS,
				       DSIM_DPHY_STATUS_TX_READY_CLK, true);
	if (ret)
		return ret;

	writel(~0, dsim->base + DSIM_INTSRC);

	return 0;
}

/* Wrapper so the bridge reference count is set up by devm_drm_bridge_alloc() */
struct exynos9610_dsim_link_bridge {
	struct drm_bridge bridge;
	struct exynos9610_dsim *dsim;
};

static inline struct exynos9610_dsim *bridge_to_dsim(struct drm_bridge *bridge)
{
	return container_of(bridge, struct exynos9610_dsim_link_bridge,
			    bridge)->dsim;
}

/*
 * The D-PHY has to be up before the panel driver sends its init sequence, and it
 * can only be powered down once DECON has stopped - stopping the link while the
 * pipe is mid-frame leaves DECON waiting for a frame-done that never comes.  The
 * atomic helpers call pre_enable before the panel bridge's prepare and
 * post_disable before the CRTC is disabled, so both ends are handled here.
 */
static void exynos9610_dsim_bridge_atomic_pre_enable(struct drm_bridge *bridge,
						     struct drm_atomic_state *state)
{
	struct exynos9610_dsim *dsim = bridge_to_dsim(bridge);
	int ret;

	ret = exynos9610_dsim_link_start(dsim);
	if (ret)
		dev_err(dsim->dev, "link power up failed: %d\n", ret);
}

/*
 * Runs after the panel has been prepared (its init sequence has gone out) and
 * before display on: the panel tears again from here, so this is where DECON can
 * be switched back on.
 */
static void exynos9610_dsim_bridge_atomic_enable(struct drm_bridge *bridge,
						 struct drm_atomic_state *state)
{
	exynos9610_dpu_decon_start();
}

static void exynos9610_dsim_bridge_atomic_post_disable(struct drm_bridge *bridge,
						       struct drm_atomic_state *state)
{
	struct exynos9610_dsim *dsim = bridge_to_dsim(bridge);

	/* Drain DECON first, then cut the link */
	exynos9610_dpu_decon_stop();
	exynos9610_dsim_link_stop(dsim);
}

static const struct drm_bridge_funcs exynos9610_dsim_bridge_funcs = {
	.atomic_pre_enable = exynos9610_dsim_bridge_atomic_pre_enable,
	.atomic_enable = exynos9610_dsim_bridge_atomic_enable,
	.atomic_post_disable = exynos9610_dsim_bridge_atomic_post_disable,
};

struct drm_bridge *exynos9610_dsim_link_bridge(struct mipi_dsi_host *host)
{
	struct exynos9610_dsim *dsim = container_of(host, struct exynos9610_dsim,
						    host);

	return dsim->link_bridge;
}
EXPORT_SYMBOL_GPL(exynos9610_dsim_link_bridge);

static ssize_t exynos9610_dsim_link_write(struct file *file,
					  const char __user *ubuf, size_t len,
					  loff_t *ppos)
{
	struct seq_file *seq = file->private_data;
	struct exynos9610_dsim *dsim = seq->private;
	char buf[16];
	int ret;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	strim(buf);

	if (!strcmp(buf, "stop")) {
		ret = exynos9610_dsim_link_stop(dsim);
	} else if (!strcmp(buf, "start")) {
		ret = exynos9610_dsim_link_start(dsim);
	} else if (!strcmp(buf, "dpms-off")) {
		ret = exynos9610_dpu_decon_stop();
		if (!ret)
			ret = exynos9610_dsim_link_stop(dsim);
	} else if (!strcmp(buf, "dpms-on")) {
		ret = exynos9610_dsim_link_start(dsim);
		if (!ret)
			ret = exynos9610_dpu_decon_start();
	} else if (!strcmp(buf, "snapshot")) {
		exynos9610_dsim_snapshot(dsim);
		ret = 0;
	} else {
		return -EINVAL;
	}

	scnprintf(dsim->last_link, sizeof(dsim->last_link),
		  "%s: ret=%d pll_stable=%u hs_clk=%u clk_ctrl=0x%08x config=0x%08x intsrc=0x%08x\n",
		  buf, ret,
		  !!(exynos9610_dsim_read(dsim, DSIM_LINK_STATUS3) &
		     DSIM_LINK_STATUS3_PLL_STABLE),
		  !!(exynos9610_dsim_read(dsim, DSIM_DPHY_STATUS) &
		     DSIM_DPHY_STATUS_TX_READY_CLK),
		  exynos9610_dsim_read(dsim, DSIM_CLK_CTRL),
		  exynos9610_dsim_read(dsim, DSIM_CONFIG),
		  exynos9610_dsim_read(dsim, DSIM_INTSRC));

	return len;
}

/*
 * Registers that no longer match the inherited baseline.  Status registers are
 * skipped: they are expected to change while the link runs, and only the
 * configuration that has to be reprogrammed after a link power up matters.
 */
static bool exynos9610_dsim_is_status_reg(unsigned int reg)
{
	switch (reg) {
	case DSIM_VERSION:
	case DSIM_LINK_STATUS0:
	case DSIM_LINK_STATUS1:
	case DSIM_LINK_STATUS3:
	case DSIM_MIPI_STATUS:
	case DSIM_DPHY_STATUS:
	case DSIM_INTSRC:
	case 0x060:	/* RX FIFO */
	case 0x068:	/* FIFO status */
		return true;
	default:
		return false;
	}
}

static int exynos9610_dsim_diff_show(struct seq_file *seq, void *unused)
{
	struct exynos9610_dsim *dsim = seq->private;
	unsigned int i, diff = 0;

	for (i = 0; i < ARRAY_SIZE(dsim->snapshot); i++) {
		u32 reg = i * 4;
		u32 live;

		if (exynos9610_dsim_is_status_reg(reg) || !dsim->snapshot[i])
			continue;

		live = exynos9610_dsim_read(dsim, reg);
		if (live == dsim->snapshot[i])
			continue;

		seq_printf(seq, "0x%03x: inherited 0x%08x live 0x%08x\n",
			   reg, dsim->snapshot[i], live);
		diff++;
	}

	seq_printf(seq, "%u register(s) differ from the inherited baseline\n", diff);

	return 0;
}

static int exynos9610_dsim_diff_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos9610_dsim_diff_show, inode->i_private);
}

static const struct file_operations exynos9610_dsim_diff_fops = {
	.owner = THIS_MODULE,
	.open = exynos9610_dsim_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int exynos9610_dsim_link_show(struct seq_file *seq, void *unused)
{
	struct exynos9610_dsim *dsim = seq->private;

	seq_puts(seq, "Write \"snapshot\", \"stop\", \"start\", \"dpms-off\" or \"dpms-on\".\n");
	seq_printf(seq, "last: %s", dsim->last_link[0] ? dsim->last_link : "(none)\n");
	seq_printf(seq, "inherited: lanes=%u config=0x%08x pll=%u dphy_status=0x%08x mipi_status=0x%08x\n",
		   dsim->lanes, dsim->snapshot[DSIM_CONFIG / 4],
		   !!(dsim->snapshot[DSIM_PLLCTRL / 4] & DSIM_PLLCTRL_PLL_EN),
		   dsim->snapshot[DSIM_DPHY_STATUS / 4],
		   dsim->snapshot[DSIM_MIPI_STATUS / 4]);

	return 0;
}

static int exynos9610_dsim_link_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos9610_dsim_link_show, inode->i_private);
}

static const struct file_operations exynos9610_dsim_link_fops = {
	.owner = THIS_MODULE,
	.open = exynos9610_dsim_link_open,
	.read = seq_read,
	.write = exynos9610_dsim_link_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * Panel brightness.
 *
 * The ea8076 (Samsung AMS646UJ09) takes a 10-bit code in DCS 0x51; branch
 * brightness 0..365 is mapped onto it by the vendor curve below, taken from
 * ea8076_a51_param.h.  Index 255 is the UI maximum, anything above it is high
 * brightness mode, which also needs DCS 0x53 to switch the panel's own boost on.
 * The level-2 command set has to be unlocked with the F0 test key first.
 */
#define EA8076_BRIGHTNESS_MAX		365
#define EA8076_BRIGHTNESS_UI_MAX	255

static const u16 ea8076_brightness_table[EA8076_BRIGHTNESS_MAX + 1] = {
	3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36,
	39, 42, 45, 48, 52, 55, 59, 62, 66, 69, 73, 77,
	80, 84, 87, 91, 95, 98, 102, 105, 109, 112, 116, 120,
	123, 127, 130, 134, 138, 141, 145, 148, 152, 156, 159, 163,
	166, 170, 173, 177, 181, 184, 188, 191, 195, 199, 202, 206,
	209, 213, 216, 220, 224, 227, 231, 234, 238, 242, 245, 249,
	252, 256, 259, 263, 267, 270, 274, 277, 281, 285, 288, 292,
	295, 299, 302, 306, 310, 313, 317, 320, 324, 328, 331, 335,
	338, 342, 345, 349, 353, 356, 360, 363, 367, 371, 374, 378,
	381, 385, 388, 392, 396, 399, 403, 406, 410, 414, 417, 421,
	424, 428, 431, 435, 439, 442, 446, 449, 453, 457, 462, 466,
	471, 475, 480, 484, 489, 493, 498, 502, 507, 511, 516, 520,
	525, 529, 534, 538, 543, 547, 552, 556, 561, 565, 570, 574,
	579, 583, 588, 592, 597, 601, 606, 610, 615, 619, 624, 628,
	633, 637, 641, 646, 650, 655, 659, 664, 668, 673, 677, 682,
	686, 691, 695, 700, 704, 709, 713, 718, 722, 727, 731, 736,
	740, 745, 749, 754, 758, 763, 767, 772, 776, 781, 785, 790,
	794, 799, 803, 808, 812, 817, 821, 826, 830, 834, 839, 843,
	848, 852, 857, 861, 866, 870, 875, 879, 884, 888, 893, 897,
	902, 906, 911, 915, 920, 924, 929, 933, 938, 942, 947, 951,
	956, 960, 965, 969, 974, 978, 983, 987, 992, 996, 1001, 1005,
	1010, 1014, 1018, 1023, 3, 7, 11, 15, 18, 22, 26, 29,
	33, 37, 40, 44, 48, 51, 55, 59, 62, 66, 70, 73,
	77, 81, 84, 88, 92, 95, 99, 103, 107, 110, 114, 118,
	121, 125, 129, 132, 136, 140, 143, 147, 151, 154, 158, 162,
	165, 169, 173, 176, 180, 184, 187, 191, 195, 198, 202, 206,
	209, 213, 217, 220, 224, 228, 231, 235, 239, 242, 246, 250,
	253, 257, 261, 264, 268, 272, 275, 279, 283, 286, 290, 294,
	297, 301, 305, 309, 312, 316, 320, 323, 327, 331, 334, 338,
	342, 345, 349, 353, 356, 360, 364, 367, 371, 375, 378, 382,
	386, 389, 393, 397, 400, 404,
};

static int exynos9610_dsim_write(struct exynos9610_dsim *dsim, const u8 *data,
				 unsigned int len)
{
	int ret;

	switch (len) {
	case 1:
		ret = exynos9610_dsim_send(dsim, MIPI_DSI_DCS_SHORT_WRITE,
					   data[0], 0, false, NULL, 0);
		break;
	case 2:
		ret = exynos9610_dsim_send(dsim, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
					   data[0], data[1], false, NULL, 0);
		break;
	default:
		/* Long packet: the header carries the word count */
		ret = exynos9610_dsim_send(dsim, MIPI_DSI_DCS_LONG_WRITE,
					   len & 0xff, (len >> 8) & 0xff, false,
					   data, len);
		break;
	}
	if (ret)
		return ret;

	/* Let both FIFOs drain so the next command cannot overtake this one */
	ret = exynos9610_dsim_wait_fifo(dsim, DSIM_FIFOCTRL_EMPTY_PH_SFR |
					DSIM_FIFOCTRL_EMPTY_PL_SFR);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	return 0;
}

static int exynos9610_dsim_set_brightness(struct exynos9610_dsim *dsim,
					  unsigned int index)
{
	static const u8 unlock[] = { 0xf0, 0x5a, 0x5a };
	static const u8 hbm_off[] = { 0x53, 0x28 };
	static const u8 hbm_on[] = { 0x53, 0xe8 };
	static const u8 acl_off[] = { 0x55, 0x00 };
	static const u8 lock[] = { 0xf0, 0xa5, 0xa5 };
	u16 value = ea8076_brightness_table[index];
	u8 brightness[3] = { 0x51, (value >> 8) & 0x3, value & 0xff };
	const u8 *hbm = index > EA8076_BRIGHTNESS_UI_MAX ? hbm_on : hbm_off;
	int ret;

	ret = exynos9610_dsim_write(dsim, unlock, sizeof(unlock));
	if (ret)
		return ret;

	ret = exynos9610_dsim_write(dsim, hbm, sizeof(hbm));
	if (!ret)
		ret = exynos9610_dsim_write(dsim, brightness, sizeof(brightness));
	if (!ret)
		ret = exynos9610_dsim_write(dsim, acl_off, sizeof(acl_off));

	if (exynos9610_dsim_write(dsim, lock, sizeof(lock)) && !ret)
		ret = -EIO;

	if (!ret) {
		dsim->brightness = index;
		dev_info(dsim->dev, "brightness %u -> 0x51 0x%02x 0x%02x (%s)\n",
			 index, brightness[1], brightness[2],
			 index > EA8076_BRIGHTNESS_UI_MAX ? "HBM" : "UI");
	}

	return ret;
}

static ssize_t exynos9610_dsim_brightness_write(struct file *file,
						const char __user *ubuf,
						size_t len, loff_t *ppos)
{
	struct seq_file *seq = file->private_data;
	struct exynos9610_dsim *dsim = seq->private;
	unsigned int index;
	char *buf;
	int ret;

	buf = memdup_user_nul(ubuf, min_t(size_t, len, 16));
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	ret = kstrtouint(buf, 0, &index);
	kfree(buf);
	if (ret)
		return ret;

	if (index > EA8076_BRIGHTNESS_MAX)
		return -EINVAL;

	ret = exynos9610_dsim_set_brightness(dsim, index);
	if (ret) {
		scnprintf(dsim->last_read, sizeof(dsim->last_read),
			  "brightness %u: failed (%d), intsrc=0x%08x\n", index,
			  ret, exynos9610_dsim_read(dsim, DSIM_INTSRC));
		return ret;
	}

	return len;
}

static int exynos9610_dsim_brightness_show(struct seq_file *seq, void *unused)
{
	struct exynos9610_dsim *dsim = seq->private;

	seq_printf(seq, "index: %u (0-%u, %u = UI max)\n", dsim->brightness,
		   EA8076_BRIGHTNESS_MAX, EA8076_BRIGHTNESS_UI_MAX);
	seq_printf(seq, "0x51 value: 0x%03x%s\n",
		   ea8076_brightness_table[dsim->brightness],
		   dsim->brightness > EA8076_BRIGHTNESS_UI_MAX ? " (HBM)" : "");

	return 0;
}

static int exynos9610_dsim_brightness_open(struct inode *inode,
					   struct file *file)
{
	return single_open(file, exynos9610_dsim_brightness_show,
			   inode->i_private);
}

static const struct file_operations exynos9610_dsim_brightness_fops = {
	.owner = THIS_MODULE,
	.open = exynos9610_dsim_brightness_open,
	.read = seq_read,
	.write = exynos9610_dsim_brightness_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/* Raw DCS write: "<cmd> [param ...]", for panel experiments */
static ssize_t exynos9610_dsim_raw_write(struct file *file,
					 const char __user *ubuf, size_t len,
					 loff_t *ppos)
{
	struct seq_file *seq = file->private_data;
	struct exynos9610_dsim *dsim = seq->private;
	u8 data[64];
	char *buf, *cur;
	unsigned int i = 0;
	char *tok;
	int ret;

	buf = memdup_user_nul(ubuf, min_t(size_t, len, sizeof(data) * 4));
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	cur = buf;
	while ((tok = strsep(&cur, " \t\n")) && i < sizeof(data)) {
		u8 byte;

		if (!*tok)
			continue;

		ret = kstrtou8(tok, 16, &byte);
		if (ret) {
			kfree(buf);
			return ret;
		}

		data[i++] = byte;
	}

	kfree(buf);

	if (!i)
		return -EINVAL;

	ret = exynos9610_dsim_write(dsim, data, i);
	scnprintf(dsim->last_read, sizeof(dsim->last_read),
		  "write %u bytes: %s (%d), intsrc=0x%08x\n", i,
		  ret ? "failed" : "sent", ret,
		  exynos9610_dsim_read(dsim, DSIM_INTSRC));

	return ret ? ret : len;
}

static int exynos9610_dsim_raw_show(struct seq_file *seq, void *unused)
{
	struct exynos9610_dsim *dsim = seq->private;

	seq_puts(seq, "Write \"<cmd-hex> [param-hex ...]\" (up to 64 bytes).\n");
	seq_printf(seq, "last: %s", dsim->last_read[0] ? dsim->last_read :
		   "(none)\n");

	return 0;
}

static int exynos9610_dsim_raw_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos9610_dsim_raw_show, inode->i_private);
}

static const struct file_operations exynos9610_dsim_raw_fops = {
	.owner = THIS_MODULE,
	.open = exynos9610_dsim_raw_open,
	.read = seq_read,
	.write = exynos9610_dsim_raw_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * MIPI DSI host.
 *
 * The panel driver speaks DCS through this: every message is turned into the
 * packet FIFO protocol below, so the panel sees exactly the byte stream the
 * vendor driver produced.  Reads use the same turnaround path as the debugfs
 * helper, writes keep the packet type the caller asked for.
 */
static ssize_t exynos9610_dsim_host_transfer(struct mipi_dsi_host *host,
					     const struct mipi_dsi_msg *msg)
{
	struct exynos9610_dsim *dsim =
		container_of(host, struct exynos9610_dsim, host);
	const u8 *tx = msg->tx_buf;
	u32 words[8];
	unsigned int nwords = 0;
	u32 d0 = 0, d1 = 0;
	int ret;

	if (msg->tx_len > 0)
		d0 = tx[0];
	if (msg->tx_len > 1)
		d1 = tx[1];

	if (msg->rx_buf) {
		if (msg->type != MIPI_DSI_DCS_READ)
			return -EOPNOTSUPP;

		ret = exynos9610_dsim_dcs_read(dsim, d0, msg->rx_len, words,
					       ARRAY_SIZE(words), &nwords);
		if (ret)
			return ret;

		if (nwords) {
			u8 *rx = msg->rx_buf;
			unsigned int i, len = min(msg->rx_len, 3u);

			for (i = 0; i < len; i++)
				rx[i] = (words[0] >> (8 * (i + 1))) & 0xff;
		}

		return nwords ? msg->rx_len : 0;
	}

	switch (msg->type) {
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
		ret = exynos9610_dsim_send(dsim, msg->type, d0, d1, false, NULL, 0);
		break;
	case MIPI_DSI_DCS_LONG_WRITE:
	case MIPI_DSI_GENERIC_LONG_WRITE:
		ret = exynos9610_dsim_send(dsim, msg->type, msg->tx_len & 0xff,
					   (msg->tx_len >> 8) & 0xff, false,
					   tx, msg->tx_len);
		break;
	default:
		dev_warn(dsim->dev, "unsupported DSI message 0x%02x\n",
			 msg->type);
		return -EOPNOTSUPP;
	}
	if (ret)
		return ret;

	ret = exynos9610_dsim_wait_fifo(dsim, DSIM_FIFOCTRL_EMPTY_PH_SFR |
					DSIM_FIFOCTRL_EMPTY_PL_SFR);
	if (ret)
		return ret;

	return msg->tx_len;
}

static int exynos9610_dsim_host_attach(struct mipi_dsi_host *host,
				       struct mipi_dsi_device *device)
{
	struct exynos9610_dsim *dsim =
		container_of(host, struct exynos9610_dsim, host);

	dev_info(dsim->dev, "attached %s, %u lanes, format %u\n",
		 dev_name(&device->dev), device->lanes, device->format);

	return 0;
}

static int exynos9610_dsim_host_detach(struct mipi_dsi_host *host,
				       struct mipi_dsi_device *device)
{
	return 0;
}

static const struct mipi_dsi_host_ops exynos9610_dsim_host_ops = {
	.attach = exynos9610_dsim_host_attach,
	.detach = exynos9610_dsim_host_detach,
	.transfer = exynos9610_dsim_host_transfer,
};

/*
 * Tearing effect from the panel.
 *
 * In hardware trigger mode the TE signal is muxed straight into DECON
 * (gpc2-3, pin function 0x0f), so the CPU never sees it and frames are paced
 * without any software involvement.  The panel can also announce the same
 * event as a DSI packet, which shows up as INTSRC.RX_TE - that is what this
 * file exposes, because it is the only TE source a driver could use to pace
 * frames itself.
 */
static irqreturn_t exynos9610_dsim_irq(int irq, void *data)
{
	struct exynos9610_dsim *dsim = data;
	u32 pending = exynos9610_dsim_read(dsim, DSIM_INTSRC);
	u32 handled = pending & DSIM_INTSRC_RX_TE;

	/*
	 * The read path polls INTSRC for RX_DATA_DONE, ACK and error bits, so
	 * only acknowledge what is consumed here.  Acknowledging everything
	 * makes reads time out with the response sitting in the RX FIFO.
	 */
	if (pending & ~DSIM_INTSRC_RX_TE)
		atomic64_inc(&dsim->te_other);

	if (!handled)
		return IRQ_NONE;

	writel(handled, dsim->base + DSIM_INTSRC);
	atomic64_inc(&dsim->te_count);

	return IRQ_HANDLED;
}

static int exynos9610_dsim_te_show(struct seq_file *seq, void *unused)
{
	struct exynos9610_dsim *dsim = seq->private;
	u32 intmsk = exynos9610_dsim_read(dsim, DSIM_INTMSK);

	seq_printf(seq, "rx_te unmasked: %s\n",
		   (intmsk & DSIM_INTSRC_RX_TE) ? "no" : "yes");
	seq_printf(seq, "rx_te packets: %llu\n",
		   (u64)atomic64_read(&dsim->te_count));
	seq_printf(seq, "other interrupts: %llu\n",
		   (u64)atomic64_read(&dsim->te_other));
	seq_printf(seq, "mipi_status: 0x%08x (te bit %u), intmsk 0x%08x\n",
		   exynos9610_dsim_read(dsim, DSIM_MIPI_STATUS),
		   !!(exynos9610_dsim_read(dsim, DSIM_MIPI_STATUS) & BIT(0)),
		   intmsk);
	seq_printf(seq, "cmd_te_ctrl0: 0x%08x, cmd_te_ctrl1: 0x%08x\n",
		   exynos9610_dsim_read(dsim, DSIM_CMD_TE_CTRL0),
		   exynos9610_dsim_read(dsim, DSIM_CMD_TE_CTRL1));

	return 0;
}

static ssize_t exynos9610_dsim_te_write(struct file *file,
					const char __user *ubuf, size_t len,
					loff_t *ppos)
{
	struct seq_file *seq = file->private_data;
	struct exynos9610_dsim *dsim = seq->private;
	char buf[8];
	u32 intmsk;

	if (len >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;

	buf[len] = '\0';

	intmsk = exynos9610_dsim_read(dsim, DSIM_INTMSK);

	if (!strncmp(buf, "on", 2))
		intmsk &= ~DSIM_INTSRC_RX_TE;
	else if (!strncmp(buf, "off", 3))
		intmsk |= DSIM_INTSRC_RX_TE;
	else
		return -EINVAL;

	writel(intmsk, dsim->base + DSIM_INTMSK);

	return len;
}

static int exynos9610_dsim_te_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos9610_dsim_te_show, inode->i_private);
}

static const struct file_operations exynos9610_dsim_te_fops = {
	.owner = THIS_MODULE,
	.open = exynos9610_dsim_te_open,
	.read = seq_read,
	.write = exynos9610_dsim_te_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t exynos9610_dsim_dcs_write(struct file *file,
					 const char __user *ubuf, size_t len,
					 loff_t *ppos)
{
	struct seq_file *seq = file->private_data;
	struct exynos9610_dsim *dsim = seq->private;
	u32 words[16];
	unsigned int nwords = 0, i;
	u8 args[32], cmd, count;
	char *buf;
	int ret, n;

	buf = memdup_user_nul(ubuf, min_t(size_t, len, sizeof(args) * 2));
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	ret = sscanf(buf, "%hhx %hhu", &cmd, &count);
	kfree(buf);
	if (ret < 1)
		return -EINVAL;

	if (ret < 2)
		count = 1;
	if (count == 0 || count > 4)
		return -EINVAL;

	ret = exynos9610_dsim_dcs_read(dsim, cmd, count, words, ARRAY_SIZE(words),
				       &nwords);
	if (ret) {
		scnprintf(dsim->last_read, sizeof(dsim->last_read),
			  "0x%02x: failed (%d), intsrc=0x%08x fifoctrl=0x%08x\n",
			  cmd, ret, exynos9610_dsim_read(dsim, DSIM_INTSRC),
			  exynos9610_dsim_read(dsim, DSIM_FIFOCTRL));
		return len;
	}

	n = scnprintf(dsim->last_read, sizeof(dsim->last_read),
		      "0x%02x: intsrc=0x%08x words=%u", cmd, dsim->last_intsrc,
		      nwords);
	for (i = 0; i < nwords && n < sizeof(dsim->last_read) - 64; i++)
		n += scnprintf(dsim->last_read + n, sizeof(dsim->last_read) - n,
			       " [type=0x%02x data=0x%02x 0x%02x 0x%02x]",
			       words[i] & 0xff, (words[i] >> 8) & 0xff,
			       (words[i] >> 16) & 0xff, (words[i] >> 24) & 0xff);
	scnprintf(dsim->last_read + n, sizeof(dsim->last_read) - n, "\n");

	return len;
}

static int exynos9610_dsim_dcs_show(struct seq_file *seq, void *unused)
{
	struct exynos9610_dsim *dsim = seq->private;

	seq_puts(seq, "Write \"<cmd> [len]\" (hex cmd, 1-4 byte read).\n");
	seq_printf(seq, "last: %s", dsim->last_read[0] ? dsim->last_read : "(none)\n");

	return 0;
}

static int exynos9610_dsim_dcs_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos9610_dsim_dcs_show, inode->i_private);
}

static const struct file_operations exynos9610_dsim_dcs_fops = {
	.owner = THIS_MODULE,
	.open = exynos9610_dsim_dcs_open,
	.read = seq_read,
	.write = exynos9610_dsim_dcs_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * Report what the boot chain left behind.  Every value here has been observed
 * live on an A51 (see notes/dsim.md), so a difference means the boot chain
 * behaved differently and the handoff assumptions have to be revisited.
 */
static void exynos9610_dsim_report(struct exynos9610_dsim *dsim)
{
	struct device *dev = dsim->dev;
	u32 link0 = exynos9610_dsim_read(dsim, DSIM_LINK_STATUS0);
	u32 link1 = exynos9610_dsim_read(dsim, DSIM_LINK_STATUS1);
	u32 link3 = exynos9610_dsim_read(dsim, DSIM_LINK_STATUS3);
	u32 mipi = exynos9610_dsim_read(dsim, DSIM_MIPI_STATUS);
	u32 dphy = exynos9610_dsim_read(dsim, DSIM_DPHY_STATUS);
	u32 clk = exynos9610_dsim_read(dsim, DSIM_CLK_CTRL);
	u32 resol = exynos9610_dsim_read(dsim, DSIM_RESOL);
	u32 xfers = exynos9610_dsim_read(dsim, DSIM_NUM_OF_TRANSFER);
	u32 config = exynos9610_dsim_read(dsim, DSIM_CONFIG);
	u32 pll = exynos9610_dsim_read(dsim, DSIM_PLLCTRL);
	u64 bitclk = 0;

	if (pll & DSIM_PLLCTRL_PLL_EN)
		bitclk = DSIM_REF_CLK_HZ * DSIM_PLLCTRL_M(pll) /
			 (DSIM_PLLCTRL_P(pll) * (1 << DSIM_PLLCTRL_S(pll)));

	dev_info(dev, "DSIM: version=0x%08x %s mode, %s, %s\n",
		 exynos9610_dsim_read(dsim, DSIM_VERSION),
		 (link1 & DSIM_LINK_STATUS1_CMD_MODE) ? "command" : "video",
		 (link3 & DSIM_LINK_STATUS3_PLL_STABLE) ? "PLL locked" : "PLL unlocked",
		 (dphy & DSIM_DPHY_STATUS_TX_READY_CLK) ? "HS clock ready" : "no HS clock");

	dev_info(dev, "DSIM: %ux%u out, %u transfers/frame, %u data lanes, format 0x%02x\n",
		 DSIM_RESOL_HRESOL(resol), DSIM_RESOL_VRESOL(resol), xfers,
		 DSIM_CONFIG_DATA_LANES(config), DSIM_CONFIG_PIXEL_FORMAT(config));

	dev_info(dev, "DSIM: PLL %s P=%u M=%u S=%u -> bit clock %llu kHz, esc prescaler %u\n",
		 (pll & DSIM_PLLCTRL_PLL_EN) ? "enabled" : "disabled",
		 DSIM_PLLCTRL_P(pll), DSIM_PLLCTRL_M(pll), DSIM_PLLCTRL_S(pll),
		 bitclk / 1000, DSIM_CLK_CTRL_ESC_PRESCALER(clk));

	dev_info(dev, "DSIM: config=0x%08x (%s, %s mode, lanes_en=0x%02x), mipi_status=0x%08x\n",
		 config,
		 (config & DSIM_CONFIG_BURST_MODE) ? "burst" : "no burst",
		 (config & DSIM_CONFIG_VIDEO_MODE) ? "video" : "command",
		 DSIM_CONFIG_LANES_EN(config), mipi);

	if (link0)
		dev_dbg(dev, "DSIM: link_status0=0x%08x\n", link0);
}

static int exynos9610_dsim_probe(struct platform_device *pdev)
{
	struct exynos9610_dsim_link_bridge *link;
	struct exynos9610_dsim *dsim;
	int ret;

	dsim = devm_kzalloc(&pdev->dev, sizeof(*dsim), GFP_KERNEL);
	if (!dsim)
		return -ENOMEM;

	dsim->dev = &pdev->dev;
	dsim->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsim->base))
		return dev_err_probe(dsim->dev, PTR_ERR(dsim->base),
				     "failed to map DSIM\n");

	dsim->pmu = syscon_regmap_lookup_by_phandle(dsim->dev->of_node,
						    "samsung,pmu-syscon");
	if (IS_ERR(dsim->pmu)) {
		dev_warn(dsim->dev, "no PMU syscon: %ld, D-PHY power stays inherited\n",
			 PTR_ERR(dsim->pmu));
		dsim->pmu = NULL;
	}

	/*
	 * The boot loader brought the link up with the panel's own lane count.
	 * Remember it: the lane enable field is a count, and the data lane field
	 * has to be preserved for a later link power up.
	 */
	exynos9610_dsim_snapshot(dsim);
	dsim->lanes = DSIM_CONFIG_DATA_LANES(exynos9610_dsim_read(dsim, DSIM_CONFIG));
	dev_info(dsim->dev, "inherited config 0x%08x: %u data lanes\n",
		 exynos9610_dsim_read(dsim, DSIM_CONFIG), dsim->lanes);

	link = devm_drm_bridge_alloc(dsim->dev,
				     struct exynos9610_dsim_link_bridge, bridge,
				     &exynos9610_dsim_bridge_funcs);
	if (IS_ERR(link))
		return PTR_ERR(link);

	link->dsim = dsim;
	link->bridge.of_node = dsim->dev->of_node;
	dsim->link_bridge = &link->bridge;

	dsim->sysreg = syscon_regmap_lookup_by_phandle(dsim->dev->of_node,
						       "samsung,disp-sysreg");
	if (IS_ERR(dsim->sysreg)) {
		dev_warn(dsim->dev, "no DISP SYSREG: %ld, PHY reset stays inherited\n",
			 PTR_ERR(dsim->sysreg));
		dsim->sysreg = NULL;
	}

	dsim->host.dev = dsim->dev;
	dsim->host.ops = &exynos9610_dsim_host_ops;
	ret = mipi_dsi_host_register(&dsim->host);
	if (ret)
		return dev_err_probe(dsim->dev, ret, "failed to register DSI host\n");

	platform_set_drvdata(pdev, dsim);
	exynos9610_dsim_report(dsim);

	exynos9610_dsim_snapshot(dsim);

	dsim->irq = platform_get_irq_optional(pdev, 0);
	if (dsim->irq == -ENXIO || dsim->irq == -ENODEV)
		dsim->irq = 0;
	else if (dsim->irq < 0)
		return dsim->irq;

	if (dsim->irq) {
		ret = devm_request_irq(dsim->dev, dsim->irq,
				       exynos9610_dsim_irq, 0,
				       dev_name(dsim->dev), dsim);
		if (ret) {
			dev_warn(dsim->dev, "no DSIM interrupts: %d\n", ret);
			dsim->irq = 0;
		}
	}

	dsim->debugfs = debugfs_create_dir(dev_name(dsim->dev), NULL);
	debugfs_create_file("dcs", 0600, dsim->debugfs, dsim,
			    &exynos9610_dsim_dcs_fops);
	debugfs_create_file("link", 0600, dsim->debugfs, dsim,
			    &exynos9610_dsim_link_fops);
	debugfs_create_file("regs", 0400, dsim->debugfs, dsim,
			    &exynos9610_dsim_diff_fops);
	debugfs_create_file("brightness", 0600, dsim->debugfs, dsim,
			    &exynos9610_dsim_brightness_fops);
	debugfs_create_file("raw", 0600, dsim->debugfs, dsim,
			    &exynos9610_dsim_raw_fops);
	debugfs_create_file("te", 0600, dsim->debugfs, dsim,
			    &exynos9610_dsim_te_fops);

	return 0;
}

static void exynos9610_dsim_remove(struct platform_device *pdev)
{
	struct exynos9610_dsim *dsim = platform_get_drvdata(pdev);

	debugfs_remove_recursive(dsim->debugfs);
	mipi_dsi_host_unregister(&dsim->host);
}

static const struct of_device_id exynos9610_dsim_of_match[] = {
	{ .compatible = "samsung,exynos9610-dsim" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos9610_dsim_of_match);

static struct platform_driver exynos9610_dsim_platform_driver = {
	.probe = exynos9610_dsim_probe,
	.remove = exynos9610_dsim_remove,
	.driver = {
		.name = "exynos9610-dsim",
		.of_match_table = exynos9610_dsim_of_match,
	},
};
module_platform_driver(exynos9610_dsim_platform_driver);

MODULE_AUTHOR("Exynos 9611 mainline port");
MODULE_DESCRIPTION("Samsung Exynos 9610/9611 DSIM handoff driver");
MODULE_LICENSE("GPL");
