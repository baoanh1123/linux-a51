/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung Exynos 9610/9611 MIPI DSI: D-PHY timing and PLL arithmetic.
 *
 * The timing table is the vendor table from dpu20/cal_9610/dsim_reg.c, which
 * the AP team calibrated per D-PHY speed grade.  It is kept as data rather than
 * derived from MIPI formulas so that the numbers match what the vendor driver
 * programs on this SoC.
 */

#ifndef __EXYNOS9610_DSIM_DPHY_H__
#define __EXYNOS9610_DSIM_DPHY_H__

#include <linux/types.h>

/**
 * struct exynos9610_dsim_dphy_timing - D-PHY timing parameters
 * @bps: HS clock the values were selected for, in Mbps
 * @b_dphyctl: bias control value for the escape clock
 */
struct exynos9610_dsim_dphy_timing {
	u32 bps;
	u32 clk_prepare;
	u32 clk_zero;
	u32 clk_post;
	u32 clk_trail;
	u32 hs_prepare;
	u32 hs_zero;
	u32 hs_trail;
	u32 lpx;
	u32 hs_exit;
	u32 b_dphyctl;
};

/**
 * struct exynos9610_dsim_pll - D-PHY PLL parameters
 * @p: divider
 * @m: multiplier
 * @s: shift
 * @freq_khz: resulting HS clock
 */
struct exynos9610_dsim_pll {
	u32 p;
	u32 m;
	u32 s;
	u32 freq_khz;
};

int exynos9610_dsim_dphy_timing(u32 hs_clk_mhz, u32 esc_clk_mhz,
				struct exynos9610_dsim_dphy_timing *timing);
int exynos9610_dsim_pll_params(u32 target_khz, u32 ref_khz,
			       struct exynos9610_dsim_pll *pll);
u32 exynos9610_dsim_esc_div(u32 hs_clk_mhz, u32 esc_clk_mhz);

#endif /* __EXYNOS9610_DSIM_DPHY_H__ */
