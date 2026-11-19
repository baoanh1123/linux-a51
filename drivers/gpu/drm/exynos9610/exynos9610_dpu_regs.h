/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Register definitions for the Samsung Exynos 9610/9611 DPU 2.0 display
 * controller ("exynos9-decon").
 *
 * Only the registers that the handoff driver actually uses are described here;
 * the full block is documented in the vendor source tree at
 * drivers/video/fbdev/exynos/dpu20/cal_9610/regs-decon.h and regs-dpp.h.
 */

#ifndef __EXYNOS9610_DPU_REGS_H__
#define __EXYNOS9610_DPU_REGS_H__

/* DECON, base 0x148b0000 (DECON 0x0000-0x0fff, BLENDER 0x1000-0x1fff) */
#define DECON_GLOBAL_CONTROL			0x0000
/* Bit 1 is the enable request, bit 0 its frame-synced counterpart (vendor) */
#define DECON_GLOBAL_CONTROL_DECON_EN		BIT(1)
#define DECON_GLOBAL_CONTROL_DECON_EN_F		BIT(0)
#define DECON_GLOBAL_CONTROL_SRESET		BIT(28)
#define DECON_GLOBAL_CONTROL_RUN_STATUS		BIT(4)
#define DECON_GLOBAL_CONTROL_OPERATION_MODE	GENMASK(9, 8)
#define DECON_GLOBAL_CONTROL_OPERATION_MODE_CMD	BIT(8)

/*
 * Interrupts are latched in DECON and reach the GIC as one level-sensitive
 * line.  Pending bits are write-1-to-clear, and the boot chain leaves its own
 * frame start/done latches set, so they have to be drained before the line is
 * usable.
 */
#define DECON_INTERRUPT_ENABLE			0x0040
#define DECON_INT_EN				BIT(0)
#define DECON_EXTRA_INT_EN			BIT(4)
#define DECON_FRAME_START_INT_EN		BIT(12)
#define DECON_FRAME_DONE_INT_EN			BIT(13)

#define DECON_EXTRA_INTERRUPT_ENABLE		0x0044
#define DECON_RESOURCE_CONFLICT_INT_EN		BIT(8)
#define DECON_TIME_OUT_INT_EN			BIT(4)

#define DECON_INTERRUPT_PENDING			0x004c
#define DECON_FRAME_START_INT_PEND		BIT(12)
#define DECON_FRAME_DONE_INT_PEND		BIT(13)
#define DECON_EXTRA_INT_PEND			BIT(4)

#define DECON_EXTRA_INTERRUPT_PENDING		0x0050
#define DECON_RESOURCE_CONFLICT_INT_PEND	BIT(8)
#define DECON_TIME_OUT_INT_PEND			BIT(4)

#define DECON_UPDATE_REQ			0x0060
#define DECON_UPDATE_REQ_GLOBAL			BIT(31)
#define DECON_UPDATE_REQ_WIN(n)			BIT(n)
#define DECON_UPDATE_REQ_FOR_DECON		GENMASK(5, 0)

#define DECON_HW_SW_TRIG_CONTROL		0x0070
#define DECON_HW_TRIG_EN			BIT(0)
#define DECON_HW_TRIG_MASK_DECON		BIT(4)
#define DECON_SW_TRIG_EN			BIT(8)
#define DECON_HW_TRIG_EDGE_POLARITY		BIT(12)

#define DECON_SCALED_SIZE_CONTROL		0x00a0
#define DECON_OUTFIFO_SIZE_CONTROL		0x0120
#define DECON_OUTFIFO_SIZE_GET(val)		((val) & 0x3fff)
#define DECON_OUTFIFO_SIZE_WIDTH(h, w)		((((h) & 0x3fff) << 16) | ((w) & 0x3fff))

#define DECON_BLENDER_BG_IMAGE_SIZE		0x0200
#define DECON_BLENDER_BG_IMAGE_COLOR_0		0x0208
#define DECON_BLENDER_BG_IMAGE_COLOR_1		0x020c

#define DECON_DATA_PATH_CONTROL_0		0x0214
#define DECON_DATA_PATH_WIN_EN(n)		BIT(4 * (n))
#define DECON_DATA_PATH_WIN_MAPCOLOR_EN(n)	BIT(4 * (n) + 1)

#define DECON_DATA_PATH_CONTROL_1		0x0218
#define DECON_DATA_PATH_WIN_CHMAP(n)		(0x7 << (4 * (n)))

#define DECON_DATA_PATH_CONTROL_2		0x0230
#define DECON_DSIM_CONNECTION_CONTROL		0x0250
#define DECON_DSIM_TE_TIMEOUT_CONTROL(dsim)	(0x0254 + 4 * (dsim))
#define DECON_FRAME_COUNT			0x02a0

#define DECON_WIN_CONTROL(n)			(0x1000 + 0x30 * (n))
#define DECON_WIN_FUNC(val)			(((val) >> 8) & 0xf)
#define DECON_WIN_ALPHA_MULT_SRC_SEL(val)	((val) & 0x3)

#define DECON_WIN_START_POSITION(n)		(0x1008 + 0x30 * (n))
#define DECON_WIN_END_POSITION(n)		(0x100c + 0x30 * (n))
#define DECON_WIN_START_TIME(n)			(0x1018 + 0x30 * (n))
#define DECON_WIN_XY(x, y)			((((y) & 0x3fff) << 16) | ((x) & 0x3fff))

/* IDMA (DPU_DMA), G0 channel base 0x14881000 */
#define IDMA_ENABLE				0x0000
#define IDMA_SFR_UPDATE_FORCE			BIT(4)

#define IDMA_IN_CON				0x0008
#define IDMA_IN_CON_IMG_FORMAT_MASK		GENMASK(16, 11)
#define IDMA_IN_CON_IMG_FORMAT(v)		(((v) & 0x3f) << 11)
#define IDMA_IMG_FORMAT_ARGB8888		0
#define IDMA_IMG_FORMAT_ABGR8888		1
#define IDMA_IMG_FORMAT_XRGB8888		4
#define IDMA_IMG_FORMAT_XBGR8888		5
#define IDMA_IMG_FORMAT_RGB565			8

/*
 * SRC_SIZE holds the row stride in pixels (the vendor passes the framebuffer's
 * virtual x resolution here) and IMG_SIZE the visible area, which is how a
 * padded framebuffer - clients align pitches to 64 bytes - is described.
 */
#define IDMA_SRC_SIZE				0x0010
#define IDMA_SIZE_WIDTH(v)			((v) & 0xffff)
#define IDMA_SIZE_HEIGHT(v)			(((v) & 0x1fff) << 16)
#define IDMA_SIZE(w, h)				(IDMA_SIZE_WIDTH(w) | IDMA_SIZE_HEIGHT(h))
#define IDMA_SRC_OFFSET				0x0014
#define IDMA_IMG_SIZE				0x0018
#define IDMA_IN_BASE_ADDR_Y			0x0040
#define IDMA_IN_BASE_ADDR_C			0x0044

#endif /* __EXYNOS9610_DPU_REGS_H__ */
