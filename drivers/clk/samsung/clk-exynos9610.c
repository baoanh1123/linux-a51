// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clock driver for Samsung Exynos9610/Exynos9611.
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include <dt-bindings/clock/samsung,exynos9610-cmu.h>

#include "clk.h"

/* CMU_TOP offsets from DOWNSTREAM_READONLY/drivers/soc/samsung/cal-if/
 * exynos9610/cmucal-sfr.c.
 */
#define PLL_CON0_SHARED0		0x0120
#define PLL_LOCKTIME_SHARED0		0x0004
#define PLL_CON0_SHARED1		0x0140
#define PLL_LOCKTIME_SHARED1		0x0008
#define PLL_CON0_MMC			0x0100
#define PLL_CON3_MMC			0x010c
#define MUX_PERI_BUS			0x1078
#define MUX_PERI_IP			0x107c
#define MUX_PERI_UART			0x1080
#define MUX_CORE_BUS			0x1018
#define MUX_APM_BUS			0x1000
#define MUX_FSYS_BUS			0x103c
#define MUX_FSYS_MMC_CARD		0x1040
#define MUX_FSYS_UFS_EMBD		0x1048
#define GATE_PERI_BUS			0x2084
#define GATE_PERI_IP			0x2088
#define GATE_PERI_UART			0x208c
#define GATE_CORE_BUS			0x2020
#define GATE_APM_BUS			0x2008
#define GATE_FSYS_BUS			0x2044
#define GATE_FSYS_MMC_CARD		0x2048
#define GATE_FSYS_UFS_EMBD		0x2050
#define MUX_USB_USB30DRD		0x108c
#define DIV_USB_USB30DRD		0x1894
#define GATE_USB_USB30DRD		0x2098
#define DIV_SHARED0_DIV2		0x18a8
#define DIV_SHARED0_DIV3		0x18ac
#define DIV_SHARED0_DIV4		0x18b0
#define DIV_SHARED1_DIV2		0x18b4
#define DIV_SHARED1_DIV3		0x18b8
#define DIV_SHARED1_DIV4		0x18bc
#define DIV_MMC_DIV2			0x18a4
#define DIV_FSYS_UFS_EMBD		0x1850
#define DIV_CORE_BUS			0x1820
#define DIV_APM_BUS			0x1808
#define DIV_FSYS_BUS			0x1844
#define DIV_FSYS_MMC_CARD		0x1848
#define DIV_PERI_BUS			0x1880
#define DIV_PERI_IP			0x1884
#define DIV_PERI_UART			0x1888
#define MUX_CIS_CLK0			0x1008
#define MUX_CIS_CLK1			0x100c
#define MUX_CIS_CLK2			0x1010
#define MUX_CIS_CLK3			0x1014
#define DIV_CIS_CLK0			0x1810
#define DIV_CIS_CLK1			0x1814
#define DIV_CIS_CLK2			0x1818
#define DIV_CIS_CLK3			0x181c
#define GATE_CIS_CLK0			0x2010
#define GATE_CIS_CLK1			0x2014
#define GATE_CIS_CLK2			0x2018
#define GATE_CIS_CLK3			0x201c

/* CMU_CORE offsets from DOWNSTREAM_READONLY/drivers/soc/samsung/cal-if/
 * exynos9610/cmucal-sfr.c.
 */
#define MUX_CORE_BUS_USER		0x0100
#define DIV_CORE_BUSP			0x1800
#define GATE_CORE_CMU_PCLK		0x2000

/* CMU_APM offsets from DOWNSTREAM_READONLY/drivers/soc/samsung/cal-if/
 * exynos9610/cmucal-sfr.c.
 */
#define MUX_APM_BUS_USER		0x0100
#define MUX_DLL_USER			0x0120
#define MUX_CLK_APM_BUS			0x1004
#define DIV_CLK_APM_BUS			0x1804
#define GATE_APM_CMU_PCLK		0x2004
#define GATE_PMU_ALIVE_PCLK		0x2018
#define GATE_GPIO_ALIVE_PCLK		0x2014
#define GATE_APM_BUS_RSTSYNC		0x209c
#define GATE_SYSREG_APM_PCLK		0x20a8
#define GATE_RTC_PCLK			0x201c
#define GATE_TOP_RTC_PCLK		0x2020
#define QCH_SPEEDY_APM			0x30ac

#define GATE_CLKCMU_CMGP_BUS		0x2000
#define MUX_CMGP_ADC			0x1000
#define MUX_CMGP_I2C			0x1004
#define MUX_CMGP_USI00			0x1008
#define DIV_CMGP_ADC			0x1800
#define DIV_CMGP_I2C			0x1804
#define DIV_CMGP_USI00			0x1808

#define MUX_FSYS_BUS_USER		0x0100
#define MUX_FSYS_MMC_CARD_USER		0x0120
#define MUX_FSYS_UFS_EMBD_USER		0x0160
#define GATE_FSYS_MMC_CARD_ACLK		0x2020
#define GATE_FSYS_MMC_CARD_SDCLKIN	0x2024
#define QCH_FSYS_MMC_CARD		0x3020
#define GATE_FSYS_UFS_EMBD_ACLK		0x2054
#define GATE_FSYS_UFS_EMBD_UNIPRO	0x2058
#define GATE_FSYS_UFS_EMBD_FMP		0x205c
#define QCH_FSYS_UFS_EMBD_FMP		0x303c
#define QCH_FSYS_UFS_EMBD_UFS		0x3040

#define GATE_CMGP_CMU_PCLK		0x2004
#define GATE_CMGP_GPIO_PCLK		0x2014
#define GATE_CMGP_SYSREG_PCLK		0x2074

#define DLL_APM_N_DCO			0x0004
#define DLL_APM_N_DCO_MASK		GENMASK(13, 0)

/* CMU_PERI offsets from DOWNSTREAM_READONLY/drivers/soc/samsung/cal-if/
 * exynos9610/cmucal-sfr.c.
 */
#define MUX_PERI_BUS_USER		0x0100
#define MUX_PERI_IP_USER		0x0120
#define MUX_PERI_UART_USER		0x0140
#define GATE_PERI_I2C			0x2008
#define GATE_PERI_USI_I2C		0x2018
#define GATE_PERI_USI_USI		0x201c
#define GATE_PERI_SPI0			0x200c
#define GATE_PERI_SPI1			0x2010
#define GATE_PERI_SPI2			0x2014
#define DIV_PERI_I2C			0x1800
#define DIV_PERI_USI_I2C		0x1814
#define DIV_PERI_USI_USI		0x1818
#define DIV_PERI_SPI0			0x1804
#define DIV_PERI_SPI1			0x1808
#define DIV_PERI_SPI2			0x180c
#define GATE_I2C_0_PCLK			0x204c
#define GATE_I2C_1_PCLK			0x2050
#define GATE_I2C_2_PCLK			0x2054
#define GATE_I2C_3_PCLK			0x2058
#define GATE_I2C_4_PCLK			0x205c
#define GATE_I2C_5_PCLK			0x2060
#define GATE_I2C_6_PCLK			0x2064
#define GATE_CAMI2C_0_PCLK		0x202c
#define GATE_CAMI2C_1_PCLK		0x2034
#define GATE_CAMI2C_2_PCLK		0x203c
#define GATE_CAMI2C_3_PCLK		0x2044
#define GATE_CAMI2C_0_IPCLK		0x2028
#define GATE_CAMI2C_1_IPCLK		0x2030
#define GATE_CAMI2C_2_IPCLK		0x2038
#define GATE_CAMI2C_3_IPCLK		0x2040
#define GATE_PERI_UART_RSTSYNC		0x208c
#define GATE_PERI_I2C_RSTSYNC		0x207c
#define GATE_PERI_SPI0_RSTSYNC		0x2080
#define GATE_PERI_SPI1_RSTSYNC		0x2084
#define GATE_PERI_SPI2_RSTSYNC		0x2088
#define GATE_PERI_USI_I2C_RSTSYNC	0x2090
#define GATE_PERI_USI_USI_RSTSYNC	0x2094
#define GATE_AXI2AHB_MSD32_PERI_ACLK	0x2020
#define GATE_LHM_AXI_P_PERI_I_CLK	0x2068
#define GATE_PERI_BUS_RSTSYNC		0x2078
#define GATE_PERI_CMU_PCLK		0x2000
#define GATE_USI00_I2C_IPCLK		0x20bc
#define GATE_USI00_I2C_PCLK		0x20c0
#define GATE_USI00_USI_IPCLK		0x20c4
#define GATE_USI00_USI_PCLK		0x20c8
#define GATE_SPI0_IPCLK			0x2098
#define GATE_SPI0_PCLK			0x209c
#define GATE_SPI1_IPCLK			0x20a0
#define GATE_SPI1_PCLK			0x20a4
#define GATE_SPI2_IPCLK			0x20a8
#define GATE_SPI2_PCLK			0x20ac
#define GATE_UART_IPCLK			0x20b4
#define GATE_UART_PCLK			0x20b8
#define GATE_GPIO_PERI_PCLK		0x2048
#define GATE_SYSREG_PERI_PCLK		0x20b0
#define GATE_WDT_CLUSTER0_PCLK		0x20cc
#define GATE_WDT_CLUSTER1_PCLK		0x20d0
#define GATE_MCT_PCLK			0x206c
#define GATE_BUSIF_TMU_PCLK		0x2024
#define GATE_OTP_CON_TOP_PCLK		0x2070
#define GATE_PWM_MOTOR_PCLK		0x2074

#define EXYNOS9610_GATE_CG_VALUE	BIT(21)
#define EXYNOS9610_GATE_MANUAL		BIT(20)
#define EXYNOS9610_GATE_AUTO		BIT(28)
#define EXYNOS9610_QCH_ENABLE		BIT(0)
#define EXYNOS9610_QCH_REQUEST		BIT(1)

struct exynos9610_gate {
	struct clk_hw hw;
	void __iomem *reg;
	/* Shared with the other registers in this Samsung clock provider. */
	spinlock_t *lock;
};

struct exynos9610_qch {
	struct clk_hw hw;
	void __iomem *reg;
	/* Shared with the other registers in this Samsung clock provider. */
	spinlock_t *lock;
};

struct exynos9610_mmc_pll {
	struct clk_hw hw;
	void __iomem *con0;
	void __iomem *con3;
};

struct exynos9610_dll_dco {
	struct clk_hw hw;
	void __iomem *n_dco;
};

static inline struct exynos9610_mmc_pll *
to_exynos9610_mmc_pll(struct clk_hw *hw)
{
	return container_of(hw, struct exynos9610_mmc_pll, hw);
}

static unsigned long exynos9610_mmc_pll_recalc_rate(struct clk_hw *hw,
						    unsigned long parent_rate)
{
	struct exynos9610_mmc_pll *pll = to_exynos9610_mmc_pll(hw);
	u32 con0 = readl(pll->con0);
	u32 con3 = readl(pll->con3);
	u32 mdiv = (con0 >> 16) & 0x3ff;
	u32 pdiv = (con0 >> 8) & 0x3f;
	u32 sdiv = con0 & 0x7;
	s16 kdiv = (s16)(con3 & 0xffff);
	s64 scaled_mdiv = ((s64)mdiv << 16) + kdiv;
	u64 rate;

	if (!pdiv || scaled_mdiv <= 0)
		return 0;

	rate = (u64)parent_rate * scaled_mdiv;
	do_div(rate, pdiv << sdiv);

	return rate >> 16;
}

static const struct clk_ops exynos9610_mmc_pll_ops = {
	.recalc_rate = exynos9610_mmc_pll_recalc_rate,
};

static inline struct exynos9610_dll_dco *
to_exynos9610_dll_dco(struct clk_hw *hw)
{
	return container_of(hw, struct exynos9610_dll_dco, hw);
}

static unsigned long exynos9610_dll_dco_recalc_rate(struct clk_hw *hw,
						    unsigned long parent_rate)
{
	struct exynos9610_dll_dco *dco = to_exynos9610_dll_dco(hw);
	u32 n_dco = readl(dco->n_dco) & DLL_APM_N_DCO_MASK;

	(void)parent_rate;
	return (n_dco + 1) * 65536UL;
}

static const struct clk_ops exynos9610_dll_dco_ops = {
	.recalc_rate = exynos9610_dll_dco_recalc_rate,
};

static inline struct exynos9610_gate *to_exynos9610_gate(struct clk_hw *hw)
{
	return container_of(hw, struct exynos9610_gate, hw);
}

static int exynos9610_gate_enable(struct clk_hw *hw)
{
	struct exynos9610_gate *gate = to_exynos9610_gate(hw);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(gate->lock, flags);
	val = readl(gate->reg);
	if (val & EXYNOS9610_GATE_MANUAL)
		val |= EXYNOS9610_GATE_CG_VALUE;
	else
		val &= ~EXYNOS9610_GATE_AUTO;
	writel(val, gate->reg);
	spin_unlock_irqrestore(gate->lock, flags);

	return 0;
}

static void exynos9610_gate_disable(struct clk_hw *hw)
{
	struct exynos9610_gate *gate = to_exynos9610_gate(hw);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(gate->lock, flags);
	val = readl(gate->reg);
	if (val & EXYNOS9610_GATE_MANUAL)
		val &= ~EXYNOS9610_GATE_CG_VALUE;
	else
		val |= EXYNOS9610_GATE_AUTO;
	writel(val, gate->reg);
	spin_unlock_irqrestore(gate->lock, flags);
}

static int exynos9610_gate_is_enabled(struct clk_hw *hw)
{
	struct exynos9610_gate *gate = to_exynos9610_gate(hw);
	unsigned long flags;
	u32 val;
	int enabled;

	spin_lock_irqsave(gate->lock, flags);
	val = readl(gate->reg);
	if (val & EXYNOS9610_GATE_MANUAL)
		enabled = !!(val & EXYNOS9610_GATE_CG_VALUE);
	else
		enabled = !(val & EXYNOS9610_GATE_AUTO);
	spin_unlock_irqrestore(gate->lock, flags);

	return enabled;
}

static const struct clk_ops exynos9610_gate_ops = {
	.enable = exynos9610_gate_enable,
	.disable = exynos9610_gate_disable,
	.is_enabled = exynos9610_gate_is_enabled,
};

static inline struct exynos9610_qch *to_exynos9610_qch(struct clk_hw *hw)
{
	return container_of(hw, struct exynos9610_qch, hw);
}

static int exynos9610_qch_enable(struct clk_hw *hw)
{
	struct exynos9610_qch *qch = to_exynos9610_qch(hw);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(qch->lock, flags);
	val = readl(qch->reg);
	if (!(val & EXYNOS9610_QCH_ENABLE)) {
		val |= EXYNOS9610_QCH_REQUEST;
		writel(val, qch->reg);
	}
	spin_unlock_irqrestore(qch->lock, flags);

	return 0;
}

static void exynos9610_qch_disable(struct clk_hw *hw)
{
	struct exynos9610_qch *qch = to_exynos9610_qch(hw);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(qch->lock, flags);
	val = readl(qch->reg);
	if (!(val & EXYNOS9610_QCH_ENABLE)) {
		val &= ~EXYNOS9610_QCH_REQUEST;
		writel(val, qch->reg);
	}
	spin_unlock_irqrestore(qch->lock, flags);
}

static int exynos9610_qch_is_enabled(struct clk_hw *hw)
{
	struct exynos9610_qch *qch = to_exynos9610_qch(hw);
	unsigned long flags;
	u32 val;
	int enabled;

	spin_lock_irqsave(qch->lock, flags);
	val = readl(qch->reg);
	enabled = !!(val & (EXYNOS9610_QCH_ENABLE |
			    EXYNOS9610_QCH_REQUEST));
	spin_unlock_irqrestore(qch->lock, flags);

	return enabled;
}

static const struct clk_ops exynos9610_qch_ops = {
	.enable = exynos9610_qch_enable,
	.disable = exynos9610_qch_disable,
	.is_enabled = exynos9610_qch_is_enabled,
};

static int __init
exynos9610_register_mode_aware_gate(struct samsung_clk_provider *ctx,
				    unsigned int id, const char *name,
				    const char *parent_name,
				    unsigned long offset)
{
	struct exynos9610_gate *gate;
	struct clk_init_data init = {};
	const char *parent = parent_name;
	int ret;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return -ENOMEM;

	init.name = name;
	init.ops = &exynos9610_gate_ops;
	init.flags = CLK_IGNORE_UNUSED;
	init.parent_names = &parent;
	init.num_parents = 1;
	gate->hw.init = &init;
	gate->reg = ctx->reg_base + offset;
	gate->lock = &ctx->lock;

	ret = clk_hw_register(NULL, &gate->hw);
	if (ret) {
		pr_err("%s: failed to register %s: %d\n", __func__, name, ret);
		kfree(gate);
		return ret;
	}

	samsung_clk_add_lookup(ctx, &gate->hw, id);
	return 0;
}

static int __init
exynos9610_register_qch(struct samsung_clk_provider *ctx, unsigned int id,
			const char *name, const char *parent_name,
			unsigned long offset)
{
	struct exynos9610_qch *qch;
	struct clk_init_data init = {};
	const char *parent = parent_name;
	int ret;

	qch = kzalloc(sizeof(*qch), GFP_KERNEL);
	if (!qch)
		return -ENOMEM;

	init.name = name;
	init.ops = &exynos9610_qch_ops;
	init.flags = CLK_IGNORE_UNUSED;
	init.parent_names = &parent;
	init.num_parents = 1;
	qch->hw.init = &init;
	qch->reg = ctx->reg_base + offset;
	qch->lock = &ctx->lock;

	ret = clk_hw_register(NULL, &qch->hw);
	if (ret) {
		pr_err("%s: failed to register %s: %d\n", __func__, name, ret);
		kfree(qch);
		return ret;
	}

	samsung_clk_add_lookup(ctx, &qch->hw, id);
	return 0;
}

static int __init
exynos9610_register_mmc_pll(struct samsung_clk_provider *ctx)
{
	struct exynos9610_mmc_pll *pll;
	struct clk_init_data init = {};
	const char *parent = "fin_pll";
	int ret;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return -ENOMEM;

	init.name = "fout_mmc_pll";
	init.ops = &exynos9610_mmc_pll_ops;
	init.parent_names = &parent;
	init.num_parents = 1;
	pll->hw.init = &init;
	pll->con0 = ctx->reg_base + PLL_CON0_MMC;
	pll->con3 = ctx->reg_base + PLL_CON3_MMC;

	ret = clk_hw_register(NULL, &pll->hw);
	if (ret) {
		pr_err("%s: failed to register MMC PLL: %d\n", __func__, ret);
		kfree(pll);
		return ret;
	}

	samsung_clk_add_lookup(ctx, &pll->hw, CLK_TOP_FOUT_MMC_PLL);
	return 0;
}

static int __init
exynos9610_register_dll_dco(struct exynos9610_dll_dco **dco_out,
			    void __iomem *base)
{
	struct exynos9610_dll_dco *dco;
	struct clk_init_data init = {};
	int ret;

	dco = kzalloc(sizeof(*dco), GFP_KERNEL);
	if (!dco)
		return -ENOMEM;

	init.name = "clk_dll_dco";
	init.ops = &exynos9610_dll_dco_ops;
	dco->hw.init = &init;
	dco->n_dco = base + DLL_APM_N_DCO;

	ret = clk_hw_register(NULL, &dco->hw);
	if (ret) {
		pr_err("%s: failed to register APM DLL DCO: %d\n",
		       __func__, ret);
		kfree(dco);
		return ret;
	}

	*dco_out = dco;
	return 0;
}

PNAME(mout_peri_bus_p) = {
	"dout_shared0_div4",
	"dout_shared1_div4",
};

PNAME(mout_peri_ip_p) = {
	"fin_pll",
	"dout_shared0_div4",
	"dout_shared1_div4",
	"fin_pll",
};

PNAME(mout_peri_uart_p) = {
	"fin_pll",
	"dout_shared0_div4",
	"dout_shared1_div4",
	"fin_pll",
};

PNAME(mout_cis_clk_p) = {
	"fin_pll",
	"dout_shared0_div4",
};

PNAME(mout_core_bus_p) = {
	"dout_shared1_div2",
	"dout_shared0_div3",
	"dout_shared0_div4",
	"dout_mmc_div2",
};

PNAME(mout_apm_bus_p) = {
	"dout_shared0_div4",
	"dout_shared1_div4",
};

PNAME(mout_fsys_mmc_card_p) = {
	"fin_pll",
	"dout_shared0_div2",
	"dout_shared1_div2",
	"dout_shared0_div3",
	"dout_shared1_div3",
	"fout_mmc_pll",
	"fin_pll",
	"fin_pll",
};

PNAME(mout_usb_usb30drd_p) = {
	"fin_pll",
	"dout_shared0_div4",
	"dout_shared1_div4",
	"fin_pll",
};

PNAME(mout_fsys_bus_p) = {
	"dout_shared0_div2",
	"dout_shared1_div2",
};

PNAME(mout_fsys_bus_user_p) = {
	"oscclk_fsys",
	"dout_fsys_bus",
};

PNAME(mout_fsys_mmc_card_user_p) = {
	"oscclk_fsys",
	"dout_fsys_mmc_card",
};

/*
 * CLKCMU_FSYS_UFS_EMBD parents, downstream
 * cmucal-node.c:205-210 (OSCCLK_CMU, PLL_SHARED0_DIV4, PLL_SHARED1_DIV4,
 * OSCCLK_CMU).
 */
PNAME(mout_fsys_ufs_embd_p) = {
	"fin_pll",
	"dout_shared0_div4",
	"dout_shared1_div4",
	"fin_pll",
};

/* cmucal-node.c:483-486: OSCCLK_FSYS, CLKCMU_FSYS_UFS_EMBD. */
PNAME(mout_fsys_ufs_embd_user_p) = {
	"oscclk_fsys",
	"dout_fsys_ufs_embd",
};

PNAME(mout_peri_bus_user_p) = {
	"fin_pll",
	"dout_peri_bus",
};

PNAME(mout_peri_ip_user_p) = {
	"fin_pll",
	"dout_peri_ip",
};

PNAME(mout_peri_uart_user_p) = {
	"fin_pll",
	"dout_peri_uart",
};

PNAME(mout_core_bus_user_p) = {
	"fin_pll",
	"dout_core_bus",
};

PNAME(mout_apm_bus_user_p) = {
	"oscclk_rco_apm",
	"dout_apm_bus",
};

PNAME(mout_dll_user_p) = {
	"oscclk_rco_apm",
	"clk_dll_dco",
};

PNAME(mout_apm_bus_cmu_p) = {
	"mout_apm_bus_user",
	"mout_dll_user",
};

static const struct samsung_pll_clock exynos9610_top_pll_clks[] __initconst = {
	PLL(pll_1051x, CLK_TOP_FOUT_SHARED0_PLL, "fout_shared0_pll", "fin_pll",
	    PLL_LOCKTIME_SHARED0, PLL_CON0_SHARED0, NULL),
	PLL(pll_1051x, CLK_TOP_FOUT_SHARED1_PLL, "fout_shared1_pll", "fin_pll",
	    PLL_LOCKTIME_SHARED1, PLL_CON0_SHARED1, NULL),
};

static const struct samsung_mux_clock exynos9610_top_mux_clks[] __initconst = {
	MUX_F(CLK_TOP_MOUT_PERI_BUS, "mout_peri_bus", mout_peri_bus_p,
	      MUX_PERI_BUS, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_TOP_MOUT_PERI_IP, "mout_peri_ip", mout_peri_ip_p,
	      MUX_PERI_IP, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_TOP_MOUT_PERI_UART, "mout_peri_uart",
	      mout_peri_uart_p, MUX_PERI_UART, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_TOP_MOUT_CIS_CLK0, "mout_cis_clk0", mout_cis_clk_p,
	      MUX_CIS_CLK0, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_TOP_MOUT_CIS_CLK1, "mout_cis_clk1", mout_cis_clk_p,
	      MUX_CIS_CLK1, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_TOP_MOUT_CIS_CLK2, "mout_cis_clk2", mout_cis_clk_p,
	      MUX_CIS_CLK2, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_TOP_MOUT_CIS_CLK3, "mout_cis_clk3", mout_cis_clk_p,
	      MUX_CIS_CLK3, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_TOP_MOUT_CORE_BUS, "mout_core_bus", mout_core_bus_p,
	      MUX_CORE_BUS, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_TOP_MOUT_APM_BUS, "mout_apm_bus", mout_apm_bus_p,
	      MUX_APM_BUS, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_TOP_MOUT_FSYS_MMC_CARD, "mout_fsys_mmc_card",
	      mout_fsys_mmc_card_p, MUX_FSYS_MMC_CARD, 0, 3, 0, 0),
	MUX_F(CLK_TOP_MOUT_FSYS_UFS_EMBD, "mout_fsys_ufs_embd",
	      mout_fsys_ufs_embd_p, MUX_FSYS_UFS_EMBD, 0, 2, 0, 0),
	MUX_F(CLK_TOP_MOUT_FSYS_BUS, "mout_fsys_bus", mout_fsys_bus_p,
	      MUX_FSYS_BUS, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_TOP_MOUT_USB_USB30DRD, "mout_usb_usb30drd",
	      mout_usb_usb30drd_p, MUX_USB_USB30DRD, 0, 2, 0,
	      CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock exynos9610_top_div_clks[] __initconst = {
	DIV_F(CLK_TOP_DOUT_SHARED0_DIV2, "dout_shared0_div2",
	      "fout_shared0_pll", DIV_SHARED0_DIV2, 0, 1, 0,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_SHARED0_DIV3, "dout_shared0_div3",
	      "fout_shared0_pll", DIV_SHARED0_DIV3, 0, 2, 0,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_SHARED0_DIV4, "dout_shared0_div4",
	      "dout_shared0_div2", DIV_SHARED0_DIV4, 0, 1, 0,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_SHARED1_DIV2, "dout_shared1_div2",
	      "fout_shared1_pll", DIV_SHARED1_DIV2, 0, 1, 0,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_SHARED1_DIV3, "dout_shared1_div3",
	      "fout_shared1_pll", DIV_SHARED1_DIV3, 0, 2, 0,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_SHARED1_DIV4, "dout_shared1_div4",
	      "dout_shared1_div2", DIV_SHARED1_DIV4, 0, 1, 0,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_FSYS_UFS_EMBD, "dout_fsys_ufs_embd",
	      "gout_fsys_ufs_embd", DIV_FSYS_UFS_EMBD, 0, 4, 0,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_PERI_BUS, "dout_peri_bus", "gout_peri_bus",
	      DIV_PERI_BUS, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_PERI_IP, "dout_peri_ip", "gout_peri_ip",
	      DIV_PERI_IP, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_PERI_UART, "dout_peri_uart",
	      "gout_peri_uart", DIV_PERI_UART, 0, 4, 0,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_CIS_CLK0, "dout_cis_clk0", "gout_cis_clk0",
	      DIV_CIS_CLK0, 0, 5, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_CIS_CLK1, "dout_cis_clk1", "gout_cis_clk1",
	      DIV_CIS_CLK1, 0, 5, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_CIS_CLK2, "dout_cis_clk2", "gout_cis_clk2",
	      DIV_CIS_CLK2, 0, 5, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_CIS_CLK3, "dout_cis_clk3", "gout_cis_clk3",
	      DIV_CIS_CLK3, 0, 5, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_MMC_DIV2, "dout_mmc_div2", "fout_mmc_pll",
	      DIV_MMC_DIV2, 0, 1, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_CORE_BUS, "dout_core_bus", "gout_core_bus",
	      DIV_CORE_BUS, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_APM_BUS, "dout_apm_bus", "gout_apm_bus",
	      DIV_APM_BUS, 0, 3, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_FSYS_MMC_CARD, "dout_fsys_mmc_card",
	      "gout_fsys_mmc_card", DIV_FSYS_MMC_CARD, 0, 9, 0, 0),
	DIV_F(CLK_TOP_DOUT_FSYS_BUS, "dout_fsys_bus", "gout_fsys_bus",
	      DIV_FSYS_BUS, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_TOP_DOUT_USB_USB30DRD, "dout_usb_usb30drd",
	      "mout_usb_usb30drd", DIV_USB_USB30DRD, 0, 4, 0,
	      CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_cmu_info exynos9610_top_cmu_info __initconst = {
	.pll_clks	= exynos9610_top_pll_clks,
	.nr_pll_clks	= ARRAY_SIZE(exynos9610_top_pll_clks),
	.mux_clks	= exynos9610_top_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(exynos9610_top_mux_clks),
	.div_clks	= exynos9610_top_div_clks,
	.nr_div_clks	= ARRAY_SIZE(exynos9610_top_div_clks),
	.nr_clk_ids	= CLK_TOP_NR_CLKS,
};

static const struct samsung_mux_clock exynos9610_fsys_mux_clks[] __initconst = {
	MUX_F(CLK_FSYS_MOUT_BUS_USER, "mout_fsys_bus_user",
	      mout_fsys_bus_user_p, MUX_FSYS_BUS_USER, 4, 1, 0,
	      CLK_MUX_READ_ONLY),
	MUX_F(CLK_FSYS_MOUT_MMC_CARD_USER, "mout_fsys_mmc_card_user",
	      mout_fsys_mmc_card_user_p, MUX_FSYS_MMC_CARD_USER, 4, 1, 0,
	      CLK_MUX_READ_ONLY),
	MUX_F(CLK_FSYS_MOUT_UFS_EMBD_USER, "mout_fsys_ufs_embd_user",
	      mout_fsys_ufs_embd_user_p, MUX_FSYS_UFS_EMBD_USER, 4, 1, 0,
	      CLK_MUX_READ_ONLY),
};

static const struct samsung_cmu_info exynos9610_fsys_cmu_info __initconst = {
	.mux_clks	= exynos9610_fsys_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(exynos9610_fsys_mux_clks),
	.nr_clk_ids	= CLK_FSYS_NR_CLKS,
};

static const struct samsung_mux_clock exynos9610_apm_mux_clks[] __initconst = {
	MUX_F(CLK_APM_MOUT_BUS_USER, "mout_apm_bus_user",
	      mout_apm_bus_user_p, MUX_APM_BUS_USER, 4, 1, 0,
	      CLK_MUX_READ_ONLY),
	MUX_F(CLK_APM_MOUT_DLL_USER, "mout_dll_user", mout_dll_user_p,
	      MUX_DLL_USER, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_APM_MOUT_BUS, "mout_clk_apm_bus", mout_apm_bus_cmu_p,
	      MUX_CLK_APM_BUS, 0, 1, 0, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock exynos9610_apm_div_clks[] __initconst = {
	DIV_F(CLK_APM_DOUT_BUS, "div_clk_apm_bus", "mout_clk_apm_bus",
	      DIV_CLK_APM_BUS, 0, 3, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_cmu_info exynos9610_apm_cmu_info __initconst = {
	.mux_clks		= exynos9610_apm_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(exynos9610_apm_mux_clks),
	.div_clks		= exynos9610_apm_div_clks,
	.nr_div_clks		= ARRAY_SIZE(exynos9610_apm_div_clks),
	.nr_clk_ids		= CLK_APM_NR_CLKS,
};

PNAME(cmgp_mux_i2c_p) = {
	"oscclk_rco_cmgp",
	"gout_clkcmu_cmgp_bus",
};

PNAME(cmgp_mux_usi00_p) = {
	"oscclk_rco_cmgp",
	"gout_clkcmu_cmgp_bus",
};

static const struct samsung_mux_clock exynos9610_cmgp_mux_clks[] __initconst = {
	MUX_F(CLK_CMGP_MUX_I2C, "mux_cmgp_i2c", cmgp_mux_i2c_p,
	      MUX_CMGP_I2C, 0, 1, 0, CLK_GET_RATE_NOCACHE),
	MUX_F(CLK_CMGP_MUX_USI00, "mux_cmgp_usi00", cmgp_mux_usi00_p,
	      MUX_CMGP_USI00, 0, 1, 0, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock exynos9610_cmgp_div_clks[] __initconst = {
	DIV_F(CLK_CMGP_DIV_I2C, "div_clk_cmgp_i2c", "mux_cmgp_i2c",
	      DIV_CMGP_I2C, 0, 4, 0, CLK_GET_RATE_NOCACHE),
	DIV_F(CLK_CMGP_DIV_USI00, "div_clk_cmgp_usi00", "mux_cmgp_usi00",
	      DIV_CMGP_USI00, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_CMGP_DIV_ADC, "div_clk_cmgp_adc", "gout_clkcmu_cmgp_bus",
	      DIV_CMGP_ADC, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_cmu_info exynos9610_cmgp_cmu_info __initconst = {
	.mux_clks		= exynos9610_cmgp_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(exynos9610_cmgp_mux_clks),
	.div_clks		= exynos9610_cmgp_div_clks,
	.nr_div_clks		= ARRAY_SIZE(exynos9610_cmgp_div_clks),
	.nr_clk_ids		= CLK_CMGP_NR_CLKS,
};

static const struct samsung_cmu_info exynos9610_usb_cmu_info __initconst = {
	.nr_clk_ids		= CLK_USB_NR_CLKS,
};

static void __init exynos9610_cmu_usb_init(struct device_node *np)
{
	struct samsung_clk_provider *ctx;

	ctx = samsung_cmu_register_one(np, &exynos9610_usb_cmu_info);
	if (!ctx) {
		pr_err("%s: failed to register USB CMU\n", __func__);
		return;
	}

	if (exynos9610_register_qch(ctx, CLK_USB_QCH_USB30,
				    "qch_usb30", "gout_usb_usb30drd",
				    0x3028))
		pr_err("%s: failed to register USB30 QCH\n", __func__);
	if (exynos9610_register_qch(ctx, CLK_USB_QCH_USBPHY_20CTRL,
				    "qch_usb20", "gout_usb_usb30drd",
				    0x302c))
		pr_err("%s: failed to register USB20 PHY QCH\n", __func__);
	if (exynos9610_register_qch(ctx, CLK_USB_QCH_USBPHY_30CTRL_0,
				    "qch_usb30_0", "gout_usb_usb30drd",
				    0x3030))
		pr_err("%s: failed to register USB30 PHY0 QCH\n", __func__);
	if (exynos9610_register_qch(ctx, CLK_USB_QCH_USBPHY_30CTRL_1,
				    "qch_usb30_1", "gout_usb_usb30drd",
				    0x3034))
		pr_err("%s: failed to register USB30 PHY1 QCH\n", __func__);
}

CLK_OF_DECLARE(exynos9610_cmu_usb, "samsung,exynos9610-cmu-usb",
	       exynos9610_cmu_usb_init);

static const struct samsung_mux_clock exynos9610_peri_mux_clks[] __initconst = {
	MUX_F(CLK_PERI_MOUT_BUS_USER, "mout_peri_bus_user",
	      mout_peri_bus_user_p, MUX_PERI_BUS_USER, 4, 1, 0,
	      CLK_MUX_READ_ONLY),
	MUX_F(CLK_PERI_MOUT_IP_USER, "mout_peri_ip_user",
	      mout_peri_ip_user_p, MUX_PERI_IP_USER, 4, 1, 0,
	      CLK_MUX_READ_ONLY),
	MUX_F(CLK_PERI_MOUT_UART_USER, "mout_peri_uart_user",
	      mout_peri_uart_user_p, MUX_PERI_UART_USER, 4, 1, 0,
	      CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock exynos9610_peri_div_clks[] __initconst = {
	DIV_F(CLK_PERI_DOUT_I2C, "dout_peri_i2c", "gout_peri_i2c",
	      DIV_PERI_I2C, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_PERI_DOUT_USI_I2C, "dout_peri_usi_i2c",
	      "gout_peri_usi_i2c", DIV_PERI_USI_I2C, 0, 4, 0,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_PERI_DOUT_USI_USI, "dout_peri_usi_usi",
	      "gout_peri_usi_usi", DIV_PERI_USI_USI, 0, 8, 0, 0),
	DIV_F(CLK_PERI_DOUT_SPI0, "dout_peri_spi0", "gout_peri_spi0",
	      DIV_PERI_SPI0, 0, 8, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_PERI_DOUT_SPI1, "dout_peri_spi1", "gout_peri_spi1",
	      DIV_PERI_SPI1, 0, 8, 0, 0),
	DIV_F(CLK_PERI_DOUT_SPI2, "dout_peri_spi2", "gout_peri_spi2",
	      DIV_PERI_SPI2, 0, 8, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_fixed_factor_clock exynos9610_peri_fixed_clks[] __initconst = {
	/*
	 * Vendor CCF models the I2C QCH clocks as virtual gates with no
	 * enable/disable register operation; retain that behavior.
	 */
	FFACTOR(CLK_PERI_I2C_0_QCH, "gate_i2c_0_qch",
		"mout_peri_bus_user", 1, 1, 0),
	FFACTOR(CLK_PERI_I2C_1_QCH, "gate_i2c_1_qch",
		"mout_peri_bus_user", 1, 1, 0),
	FFACTOR(CLK_PERI_I2C_2_QCH, "gate_i2c_2_qch",
		"mout_peri_bus_user", 1, 1, 0),
	FFACTOR(CLK_PERI_I2C_3_QCH, "gate_i2c_3_qch",
		"mout_peri_bus_user", 1, 1, 0),
	FFACTOR(CLK_PERI_I2C_4_QCH, "gate_i2c_4_qch",
		"mout_peri_bus_user", 1, 1, 0),
	FFACTOR(CLK_PERI_I2C_5_QCH, "gate_i2c_5_qch",
		"mout_peri_bus_user", 1, 1, 0),
	FFACTOR(CLK_PERI_I2C_6_QCH, "gate_i2c_6_qch",
		"mout_peri_bus_user", 1, 1, 0),
};

static const struct samsung_cmu_info exynos9610_peri_cmu_info __initconst = {
	.mux_clks		= exynos9610_peri_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(exynos9610_peri_mux_clks),
	.div_clks		= exynos9610_peri_div_clks,
	.nr_div_clks		= ARRAY_SIZE(exynos9610_peri_div_clks),
	.fixed_factor_clks	= exynos9610_peri_fixed_clks,
	.nr_fixed_factor_clks	= ARRAY_SIZE(exynos9610_peri_fixed_clks),
	.nr_clk_ids		= CLK_PERI_NR_CLKS,
};

static const struct samsung_mux_clock exynos9610_core_mux_clks[] __initconst = {
	MUX_F(CLK_CORE_MOUT_BUS_USER, "mout_core_bus_user",
	      mout_core_bus_user_p, MUX_CORE_BUS_USER, 4, 1, 0,
	      CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock exynos9610_core_div_clks[] __initconst = {
	DIV_F(CLK_CORE_DOUT_BUSP, "div_clk_core_busp", "mout_core_bus_user",
	      DIV_CORE_BUSP, 0, 2, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_cmu_info exynos9610_core_cmu_info __initconst = {
	.mux_clks	= exynos9610_core_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(exynos9610_core_mux_clks),
	.div_clks	= exynos9610_core_div_clks,
	.nr_div_clks	= ARRAY_SIZE(exynos9610_core_div_clks),
	.nr_clk_ids	= CLK_CORE_NR_CLKS,
};

static void __init exynos9610_cmu_top_init(struct device_node *np)
{
	struct samsung_clk_provider *ctx;

	/* Do not use exynos_arm64_register_cmu(): it forces manual gate mode. */
	ctx = samsung_cmu_register_one(np, &exynos9610_top_cmu_info);
	if (!ctx) {
		pr_err("%s: failed to register TOP CMU\n", __func__);
		return;
	}

	if (exynos9610_register_mmc_pll(ctx))
		pr_err("%s: failed to register MMC PLL rate clock\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx, CLK_TOP_GOUT_PERI_BUS,
						"gout_peri_bus", "mout_peri_bus",
						GATE_PERI_BUS))
		pr_err("%s: failed to register PERI bus gate\n", __func__);
	if (exynos9610_register_mode_aware_gate(ctx, CLK_TOP_GOUT_PERI_IP,
						"gout_peri_ip", "mout_peri_ip",
						GATE_PERI_IP))
		pr_err("%s: failed to register PERI IP gate\n", __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_TOP_GOUT_PERI_UART,
						"gout_peri_uart",
						"mout_peri_uart",
						GATE_PERI_UART))
		pr_err("%s: failed to register PERI UART rate gate\n",
		       __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_TOP_GOUT_CIS_CLK0,
						"gout_cis_clk0",
						"mout_cis_clk0",
						GATE_CIS_CLK0))
		pr_err("%s: failed to register CIS CLK0 gate\n", __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_TOP_GOUT_CIS_CLK1,
						"gout_cis_clk1",
						"mout_cis_clk1",
						GATE_CIS_CLK1))
		pr_err("%s: failed to register CIS CLK1 gate\n", __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_TOP_GOUT_CIS_CLK2,
						"gout_cis_clk2",
						"mout_cis_clk2",
						GATE_CIS_CLK2))
		pr_err("%s: failed to register CIS CLK2 gate\n", __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_TOP_GOUT_CIS_CLK3,
						"gout_cis_clk3",
						"mout_cis_clk3",
						GATE_CIS_CLK3))
		pr_err("%s: failed to register CIS CLK3 gate\n", __func__);
	if (exynos9610_register_mode_aware_gate(ctx, CLK_TOP_GOUT_CORE_BUS,
						"gout_core_bus",
						"mout_core_bus",
						GATE_CORE_BUS))
		pr_err("%s: failed to register CORE bus gate\n", __func__);
	if (exynos9610_register_mode_aware_gate(ctx, CLK_TOP_GOUT_APM_BUS,
						"gout_apm_bus",
						"mout_apm_bus",
						GATE_APM_BUS))
		pr_err("%s: failed to register APM bus gate\n", __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_TOP_GOUT_FSYS_MMC_CARD,
						"gout_fsys_mmc_card",
						"mout_fsys_mmc_card",
						GATE_FSYS_MMC_CARD))
		pr_err("%s: failed to register FSYS MMC card gate\n",
		       __func__);
	if (exynos9610_register_mode_aware_gate(ctx, CLK_TOP_GOUT_FSYS_UFS_EMBD,
						"gout_fsys_ufs_embd",
						"mout_fsys_ufs_embd",
						GATE_FSYS_UFS_EMBD))
		pr_err("%s: failed to register FSYS UFS gate\n", __func__);
	if (exynos9610_register_mode_aware_gate(ctx, CLK_TOP_GOUT_FSYS_BUS,
						"gout_fsys_bus",
						"mout_fsys_bus",
						GATE_FSYS_BUS))
		pr_err("%s: failed to register FSYS bus gate\n", __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_TOP_GOUT_USB_USB30DRD,
						"gout_usb_usb30drd",
						"dout_usb_usb30drd",
						GATE_USB_USB30DRD))
		pr_err("%s: failed to register USB30DRD gate\n", __func__);
}

CLK_OF_DECLARE(exynos9610_cmu_top, "samsung,exynos9610-cmu-top",
	       exynos9610_cmu_top_init);

static void __init exynos9610_cmu_fsys_init(struct device_node *np)
{
	struct samsung_clk_provider *ctx;

	ctx = samsung_cmu_register_one(np, &exynos9610_fsys_cmu_info);
	if (!ctx) {
		pr_err("%s: failed to register FSYS CMU\n", __func__);
		return;
	}

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_FSYS_GOUT_MMC_CARD_ACLK,
						"gout_fsys_mmc_card_aclk",
						"mout_fsys_bus_user",
						GATE_FSYS_MMC_CARD_ACLK))
		pr_err("%s: failed to register FSYS MMC card ACLK gate\n",
		       __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_FSYS_GOUT_MMC_CARD_SDCLKIN,
						"gout_fsys_mmc_card_sdclkin",
						"mout_fsys_mmc_card_user",
						GATE_FSYS_MMC_CARD_SDCLKIN))
		pr_err("%s: failed to register FSYS MMC card SDCLKIN gate\n",
		       __func__);
	if (exynos9610_register_qch(ctx, CLK_FSYS_GOUT_MMC_CARD_QCH,
				    "gout_fsys_mmc_card_qch",
				    "gout_fsys_mmc_card_aclk",
				    QCH_FSYS_MMC_CARD))
		pr_err("%s: failed to register FSYS MMC card QCH\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_FSYS_GOUT_UFS_EMBD_ACLK,
						"gout_fsys_ufs_embd_aclk",
						"mout_fsys_bus_user",
						GATE_FSYS_UFS_EMBD_ACLK))
		pr_err("%s: failed to register FSYS UFS ACLK gate\n",
		       __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_FSYS_GOUT_UFS_EMBD_UNIPRO,
						"gout_fsys_ufs_embd_unipro",
						"mout_fsys_ufs_embd_user",
						GATE_FSYS_UFS_EMBD_UNIPRO))
		pr_err("%s: failed to register FSYS UFS UNIPRO gate\n",
		       __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_FSYS_GOUT_UFS_EMBD_FMP,
						"gout_fsys_ufs_embd_fmp",
						"mout_fsys_ufs_embd_user",
						GATE_FSYS_UFS_EMBD_FMP))
		pr_err("%s: failed to register FSYS UFS FMP gate\n", __func__);
	if (exynos9610_register_qch(ctx, CLK_FSYS_GOUT_UFS_EMBD_QCH_UFS,
				    "qch_fsys_ufs_embd_ufs",
				    "gout_fsys_ufs_embd_aclk",
				    QCH_FSYS_UFS_EMBD_UFS))
		pr_err("%s: failed to register FSYS UFS QCH\n", __func__);
	if (exynos9610_register_qch(ctx, CLK_FSYS_GOUT_UFS_EMBD_QCH_FMP,
				    "qch_fsys_ufs_embd_fmp",
				    "gout_fsys_ufs_embd_fmp",
				    QCH_FSYS_UFS_EMBD_FMP))
		pr_err("%s: failed to register FSYS UFS FMP QCH\n", __func__);
}

CLK_OF_DECLARE(exynos9610_cmu_fsys, "samsung,exynos9610-cmu-fsys",
	       exynos9610_cmu_fsys_init);

static void __init exynos9610_cmu_peri_init(struct device_node *np)
{
	struct samsung_clk_provider *ctx;

	ctx = samsung_cmu_register_one(np, &exynos9610_peri_cmu_info);
	if (!ctx) {
		pr_err("%s: failed to register PERI CMU\n", __func__);
		return;
	}

	if (exynos9610_register_mode_aware_gate(ctx, CLK_PERI_GOUT_I2C,
						"gout_peri_i2c",
						"mout_peri_ip_user",
						GATE_PERI_I2C))
		pr_err("%s: failed to register PERI I2C rate gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_I2C_0_PCLK,
						"gout_i2c_0_pclk",
						"mout_peri_bus_user",
						GATE_I2C_0_PCLK))
		pr_err("%s: failed to register I2C0 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_I2C_1_PCLK,
						"gout_i2c_1_pclk",
						"mout_peri_bus_user",
						GATE_I2C_1_PCLK))
		pr_err("%s: failed to register I2C1 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_I2C_2_PCLK,
						"gout_i2c_2_pclk",
						"mout_peri_bus_user",
						GATE_I2C_2_PCLK))
		pr_err("%s: failed to register I2C2 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_I2C_3_PCLK,
						"gout_i2c_3_pclk",
						"mout_peri_bus_user",
						GATE_I2C_3_PCLK))
		pr_err("%s: failed to register I2C3 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_I2C_4_PCLK,
						"gout_i2c_4_pclk",
						"mout_peri_bus_user",
						GATE_I2C_4_PCLK))
		pr_err("%s: failed to register I2C4 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_I2C_5_PCLK,
						"gout_i2c_5_pclk",
						"mout_peri_bus_user",
						GATE_I2C_5_PCLK))
		pr_err("%s: failed to register I2C5 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_I2C_6_PCLK,
						"gout_i2c_6_pclk",
						"mout_peri_bus_user",
						GATE_I2C_6_PCLK))
		pr_err("%s: failed to register I2C6 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_CAMI2C_0_PCLK,
						"gout_cami2c_0_pclk",
						"mout_peri_bus_user",
						GATE_CAMI2C_0_PCLK))
		pr_err("%s: failed to register CAMI2C0 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_CAMI2C_1_PCLK,
						"gout_cami2c_1_pclk",
						"mout_peri_bus_user",
						GATE_CAMI2C_1_PCLK))
		pr_err("%s: failed to register CAMI2C1 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_CAMI2C_2_PCLK,
						"gout_cami2c_2_pclk",
						"mout_peri_bus_user",
						GATE_CAMI2C_2_PCLK))
		pr_err("%s: failed to register CAMI2C2 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_CAMI2C_3_PCLK,
						"gout_cami2c_3_pclk",
						"mout_peri_bus_user",
						GATE_CAMI2C_3_PCLK))
		pr_err("%s: failed to register CAMI2C3 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_CAMI2C_0,
						"gout_cami2c_0",
						"dout_peri_i2c",
						GATE_CAMI2C_0_IPCLK))
		pr_err("%s: failed to register CAMI2C0 functional gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_CAMI2C_1,
						"gout_cami2c_1",
						"dout_peri_i2c",
						GATE_CAMI2C_1_IPCLK))
		pr_err("%s: failed to register CAMI2C1 functional gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_CAMI2C_2,
						"gout_cami2c_2",
						"dout_peri_i2c",
						GATE_CAMI2C_2_IPCLK))
		pr_err("%s: failed to register CAMI2C2 functional gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_CAMI2C_3,
						"gout_cami2c_3",
						"dout_peri_i2c",
						GATE_CAMI2C_3_IPCLK))
		pr_err("%s: failed to register CAMI2C3 functional gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_PERI_I2C_RSTSYNC,
						"gout_peri_i2c_rstsync",
						"dout_peri_i2c",
						GATE_PERI_I2C_RSTSYNC))
		pr_err("%s: failed to register PERI I2C reset-sync gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_PERI_SPI0_RSTSYNC,
						"gout_peri_spi0_rstsync",
						"dout_peri_spi0",
						GATE_PERI_SPI0_RSTSYNC))
		pr_err("%s: failed to register PERI SPI0 reset-sync gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_PERI_SPI1_RSTSYNC,
						"gout_peri_spi1_rstsync",
						"dout_peri_spi1",
						GATE_PERI_SPI1_RSTSYNC))
		pr_err("%s: failed to register PERI SPI1 reset-sync gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_PERI_SPI2_RSTSYNC,
						"gout_peri_spi2_rstsync",
						"dout_peri_spi2",
						GATE_PERI_SPI2_RSTSYNC))
		pr_err("%s: failed to register PERI SPI2 reset-sync gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_PERI_USI_I2C_RSTSYNC,
						"gout_peri_usi_i2c_rstsync",
						"dout_peri_usi_i2c",
						GATE_PERI_USI_I2C_RSTSYNC))
		pr_err("%s: failed to register USI I2C reset-sync gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_PERI_USI_USI_RSTSYNC,
						"gout_peri_usi_usi_rstsync",
						"dout_peri_usi_usi",
						GATE_PERI_USI_USI_RSTSYNC))
		pr_err("%s: failed to register USI reset-sync gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_PERI_UART_RSTSYNC,
						"gout_peri_uart_rstsync",
						"mout_peri_uart_user",
						GATE_PERI_UART_RSTSYNC))
		pr_err("%s: failed to register UART reset-sync gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_AXI2AHB_MSD32_PERI_ACLK,
						"gout_axi2ahb_msd32_peri_aclk",
						"mout_peri_bus_user",
						GATE_AXI2AHB_MSD32_PERI_ACLK))
		pr_err("%s: failed to register PERI AXI2AHB ACLK gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_LHM_AXI_P_PERI_I_CLK,
						"gout_lhm_axi_p_peri_i_clk",
						"mout_peri_bus_user",
						GATE_LHM_AXI_P_PERI_I_CLK))
		pr_err("%s: failed to register PERI AXI master gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_PERI_BUS_RSTSYNC,
						"gout_peri_bus_rstsync",
						"mout_peri_bus_user",
						GATE_PERI_BUS_RSTSYNC))
		pr_err("%s: failed to register PERI bus reset-sync gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_PERI_CMU_PCLK,
						"gout_peri_cmu_pclk",
						"mout_peri_bus_user",
						GATE_PERI_CMU_PCLK))
		pr_err("%s: failed to register PERI CMU PCLK gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_USI_I2C_RATE,
						"gout_peri_usi_i2c",
						"mout_peri_ip_user",
						GATE_PERI_USI_I2C))
		pr_err("%s: failed to register USI I2C rate gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx, CLK_PERI_GOUT_USI_I2C,
						"gout_usi00_i2c",
						"dout_peri_usi_i2c",
						GATE_USI00_I2C_IPCLK))
		pr_err("%s: failed to register USI00 I2C functional gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_USI_I2C_PCLK,
						"gout_usi00_i2c_pclk",
						"mout_peri_bus_user",
						GATE_USI00_I2C_PCLK))
		pr_err("%s: failed to register USI00 I2C PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_USI_USI_RATE,
						"gout_peri_usi_usi",
						"mout_peri_ip_user",
						GATE_PERI_USI_USI))
		pr_err("%s: failed to register USI rate gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx, CLK_PERI_GOUT_USI_USI,
						"gout_usi00_usi",
						"dout_peri_usi_usi",
						GATE_USI00_USI_IPCLK))
		pr_err("%s: failed to register USI00 functional gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_USI_USI_PCLK,
						"gout_usi00_usi_pclk",
						"mout_peri_bus_user",
						GATE_USI00_USI_PCLK))
		pr_err("%s: failed to register USI00 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_SPI0_RATE,
						"gout_peri_spi0",
						"mout_peri_ip_user",
						GATE_PERI_SPI0))
		pr_err("%s: failed to register SPI0 rate gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx, CLK_PERI_GOUT_SPI0,
						"gout_spi0", "dout_peri_spi0",
						GATE_SPI0_IPCLK))
		pr_err("%s: failed to register SPI0 functional gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx, CLK_PERI_GOUT_SPI0_PCLK,
						"gout_spi0_pclk",
						"mout_peri_bus_user",
						GATE_SPI0_PCLK))
		pr_err("%s: failed to register SPI0 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_SPI1_RATE,
						"gout_peri_spi1",
						"mout_peri_ip_user",
						GATE_PERI_SPI1))
		pr_err("%s: failed to register SPI1 rate gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx, CLK_PERI_GOUT_SPI1,
						"gout_spi1", "dout_peri_spi1",
						GATE_SPI1_IPCLK))
		pr_err("%s: failed to register SPI1 functional gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx, CLK_PERI_GOUT_SPI1_PCLK,
						"gout_spi1_pclk",
						"mout_peri_bus_user",
						GATE_SPI1_PCLK))
		pr_err("%s: failed to register SPI1 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_SPI2_RATE,
						"gout_peri_spi2",
						"mout_peri_ip_user",
						GATE_PERI_SPI2))
		pr_err("%s: failed to register SPI2 rate gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx, CLK_PERI_GOUT_SPI2,
						"gout_spi2", "dout_peri_spi2",
						GATE_SPI2_IPCLK))
		pr_err("%s: failed to register SPI2 functional gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx, CLK_PERI_GOUT_SPI2_PCLK,
						"gout_spi2_pclk",
						"mout_peri_bus_user",
						GATE_SPI2_PCLK))
		pr_err("%s: failed to register SPI2 PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx, CLK_PERI_GOUT_UART,
						"gout_uart",
						"mout_peri_uart_user",
						GATE_UART_IPCLK))
		pr_err("%s: failed to register UART functional gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_UART_PCLK,
						"gout_uart_pclk",
						"mout_peri_bus_user",
						GATE_UART_PCLK))
		pr_err("%s: failed to register UART PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_GPIO_PERI_PCLK,
						"gout_gpio_peri_pclk",
						"mout_peri_bus_user",
						GATE_GPIO_PERI_PCLK))
		pr_err("%s: failed to register GPIO_PERI PCLK gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_SYSREG_PERI_PCLK,
						"gout_sysreg_peri_pclk",
						"mout_peri_bus_user",
						GATE_SYSREG_PERI_PCLK))
		pr_err("%s: failed to register SYSREG_PERI PCLK gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_WDT_CLUSTER0_PCLK,
						"gout_wdt_cluster0_pclk",
						"mout_peri_bus_user",
						GATE_WDT_CLUSTER0_PCLK))
		pr_err("%s: failed to register WDT cluster 0 PCLK gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_WDT_CLUSTER1_PCLK,
						"gout_wdt_cluster1_pclk",
						"mout_peri_bus_user",
						GATE_WDT_CLUSTER1_PCLK))
		pr_err("%s: failed to register WDT cluster 1 PCLK gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_MCT_PCLK,
						"gout_mct_pclk",
						"mout_peri_bus_user",
						GATE_MCT_PCLK))
		pr_err("%s: failed to register MCT PCLK gate\n", __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_BUSIF_TMU_PCLK,
						"gout_busif_tmu_pclk",
						"mout_peri_bus_user",
						GATE_BUSIF_TMU_PCLK))
		pr_err("%s: failed to register bus-interface TMU PCLK gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_OTP_CON_TOP_PCLK,
						"gout_otp_con_top_pclk",
						"mout_peri_bus_user",
						GATE_OTP_CON_TOP_PCLK))
		pr_err("%s: failed to register OTP controller PCLK gate\n",
		       __func__);

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_PERI_GOUT_PWM_MOTOR_PCLK,
						"gout_pwm_motor_pclk",
						"mout_peri_bus_user",
						GATE_PWM_MOTOR_PCLK))
		pr_err("%s: failed to register PWM motor PCLK gate\n",
		       __func__);
}

CLK_OF_DECLARE(exynos9610_cmu_peri, "samsung,exynos9610-cmu-peri",
	       exynos9610_cmu_peri_init);

static void __init exynos9610_cmu_apm_init(struct device_node *np)
{
	struct exynos9610_dll_dco *dco;
	struct samsung_clk_provider *ctx;
	void __iomem *dll_base;
	int ret;

	dll_base = of_iomap(np, 1);
	if (!dll_base) {
		pr_err("%s: failed to map APM DLL registers\n", __func__);
		return;
	}

	ret = exynos9610_register_dll_dco(&dco, dll_base);
	if (ret) {
		iounmap(dll_base);
		pr_err("%s: failed to register APM DLL DCO: %d\n",
		       __func__, ret);
		return;
	}

	ctx = samsung_cmu_register_one(np, &exynos9610_apm_cmu_info);
	if (!ctx) {
		clk_hw_unregister(&dco->hw);
		kfree(dco);
		iounmap(dll_base);
		pr_err("%s: failed to register APM CMU\n", __func__);
		return;
	}

	samsung_clk_add_lookup(ctx, &dco->hw, CLK_APM_CLK_DLL_DCO);

	if (exynos9610_register_mode_aware_gate(ctx, CLK_APM_GOUT_CMU_PCLK,
						"gout_apm_cmu_pclk",
						"div_clk_apm_bus",
						GATE_APM_CMU_PCLK))
		pr_err("%s: failed to register APM CMU PCLK gate\n", __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_APM_GOUT_SYSREG_PCLK,
						"gout_sysreg_apm_pclk",
						"div_clk_apm_bus",
						GATE_SYSREG_APM_PCLK))
		pr_err("%s: failed to register APM SYSREG PCLK gate\n",
		       __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_APM_GOUT_PMU_ALIVE_PCLK,
						"gout_pmu_alive_pclk",
						"div_clk_apm_bus",
						GATE_PMU_ALIVE_PCLK))
		pr_err("%s: failed to register PMU ALIVE PCLK gate\n",
		       __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_APM_GOUT_GPIO_ALIVE_PCLK,
						"gout_gpio_alive_pclk",
						"div_clk_apm_bus",
						GATE_GPIO_ALIVE_PCLK))
		pr_err("%s: failed to register GPIO ALIVE PCLK gate\n",
		       __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_APM_GOUT_BUS_RSTSYNC,
						"gout_apm_bus_rstsync",
						"div_clk_apm_bus",
						GATE_APM_BUS_RSTSYNC))
		pr_err("%s: failed to register APM bus reset-sync gate\n",
		       __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_APM_GOUT_CMGP_BUS,
						"gout_clkcmu_cmgp_bus",
						"div_clk_apm_bus",
						GATE_CLKCMU_CMGP_BUS))
		pr_err("%s: failed to register CMGP bus gate\n", __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_APM_GOUT_RTC_PCLK,
						"gout_rtc_pclk",
						"div_clk_apm_bus",
						GATE_RTC_PCLK))
		pr_err("%s: failed to register APM RTC PCLK gate\n",
		       __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_APM_GOUT_TOP_RTC_PCLK,
						"gout_top_rtc_pclk",
						"div_clk_apm_bus",
						GATE_TOP_RTC_PCLK))
		pr_err("%s: failed to register APM TOP RTC PCLK gate\n",
		       __func__);
	if (exynos9610_register_qch(ctx, CLK_APM_GOUT_SPEEDY_APM_QCH,
				    "gout_speedy_apm_qch",
				    "div_clk_apm_bus", QCH_SPEEDY_APM))
		pr_err("%s: failed to register Speedy APM QCH\n", __func__);
}

CLK_OF_DECLARE(exynos9610_cmu_apm, "samsung,exynos9610-cmu-apm",
	       exynos9610_cmu_apm_init);

static struct clk_hw *exynos9610_cmgp_i2c_mux_hw;
static void __iomem *exynos9610_cmgp_base;

static void __init exynos9610_cmgp_i2c_set_div(void __iomem *base)
{
	u32 val;
	int i;

	for (i = 0; i < 100; i++) {
		val = readl(base + DIV_CMGP_I2C);
		if (!(val & BIT(16)))
			break;
		udelay(10);
	}
	writel(val & ~0xf, base + DIV_CMGP_I2C);
	for (i = 0; i < 100; i++) {
		if (!(readl(base + DIV_CMGP_I2C) & BIT(16)))
			break;
		udelay(10);
	}
}

static int __init exynos9610_cmgp_i2c_fixup(void)
{
	struct clk_hw *parent;
	int ret;

	if (!exynos9610_cmgp_i2c_mux_hw)
		return 0;

	parent = clk_hw_get_parent_by_index(exynos9610_cmgp_i2c_mux_hw, 0);
	if (!parent) {
		pr_err("CMGP I2C: 30 MHz parent clock is not registered\n");
		return 0;
	}

	ret = clk_hw_set_parent(exynos9610_cmgp_i2c_mux_hw, parent);
	if (ret) {
		pr_err("CMGP I2C: reparent to 30 MHz failed: %d\n", ret);
		return 0;
	}

	/* The mux transition resets the divider to its maximum; set it last. */
	exynos9610_cmgp_i2c_set_div(exynos9610_cmgp_base);

	return 0;
}
late_initcall(exynos9610_cmgp_i2c_fixup);

static void __init exynos9610_cmu_cmgp_init(struct device_node *np)
{
	struct samsung_clk_provider *ctx;

	ctx = samsung_cmu_register_one(np, &exynos9610_cmgp_cmu_info);
	if (!ctx) {
		pr_err("%s: failed to register CMGP CMU\n", __func__);
		return;
	}

	/*
	 * Downstream Android runs CMGP I2C from the 30 MHz OSCCLK_RCO_CMGP
	 * parent with a divider of 1. The boot state leaves the mux on the
	 * 199.875 MHz CMGP bus and the divider at 2. Clear the divider here
	 * and reparent the mux later, once the fixed clock is registered.
	 */
	exynos9610_cmgp_i2c_mux_hw = ctx->clk_data.hws[CLK_CMGP_MUX_I2C];
	exynos9610_cmgp_base = ctx->reg_base;

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_CMGP_GOUT_CMU_PCLK,
						"gout_cmgp_cmu_pclk",
						"gout_clkcmu_cmgp_bus",
						GATE_CMGP_CMU_PCLK))
		pr_err("%s: failed to register CMGP CMU PCLK gate\n", __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_CMGP_GOUT_SYSREG_PCLK,
						"gout_sysreg_cmgp_pclk",
						"gout_clkcmu_cmgp_bus",
						GATE_CMGP_SYSREG_PCLK))
		pr_err("%s: failed to register CMGP SYSREG PCLK gate\n",
		       __func__);
	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_CMGP_GOUT_GPIO_PCLK,
						"gout_gpio_cmgp_pclk",
						"gout_clkcmu_cmgp_bus",
						GATE_CMGP_GPIO_PCLK))
		pr_err("%s: failed to register CMGP GPIO PCLK gate\n",
		       __func__);
}

CLK_OF_DECLARE(exynos9610_cmu_cmgp, "samsung,exynos9610-cmu-cmgp",
	       exynos9610_cmu_cmgp_init);

static void __init exynos9610_cmu_core_init(struct device_node *np)
{
	struct samsung_clk_provider *ctx;

	ctx = samsung_cmu_register_one(np, &exynos9610_core_cmu_info);
	if (!ctx) {
		pr_err("%s: failed to register CORE CMU\n", __func__);
		return;
	}

	if (exynos9610_register_mode_aware_gate(ctx,
						CLK_CORE_GOUT_CMU_PCLK,
						"gout_core_cmu_pclk",
						"div_clk_core_busp",
						GATE_CORE_CMU_PCLK))
		pr_err("%s: failed to register CORE CMU PCLK gate\n", __func__);
}

CLK_OF_DECLARE(exynos9610_cmu_core, "samsung,exynos9610-cmu-core",
	       exynos9610_cmu_core_init);
