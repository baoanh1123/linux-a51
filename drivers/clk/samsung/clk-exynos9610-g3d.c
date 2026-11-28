// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 9610 G3D (Mali-G72) clock control.
 *
 * The boot loader leaves PLL_G3D at 260 MHz, which is also what the G3D CMU
 * mux selects.  The vendor kernel changes this PLL through ACPM, but ACPM DVFS
 * channel 5 is not serviced on this port, so the AP reprograms the PLL itself.
 *
 * G3D is a power domain and its CMU registers must not be touched while it is
 * off.  The provider is a device in that domain, so clk_set_rate() takes a
 * runtime PM reference before touching any register.  The mapping must cover
 * the whole CMU block: the bus mux lives at +0x1004 and the divider at +0x1800,
 * beyond the first 4 KiB page.
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#define G3D_FIN_HZ		26000000UL
#define G3D_BOOT_HZ		260000000UL

#define G3D_PLL_LOCKTIME	0x0000
#define G3D_SW_USER		0x0100
#define G3D_PLL_CON0		0x0120
#define G3D_MUX_BUSD		0x1004

#define PLL_ENABLE		BIT(31)
#define PLL_OUT_SEL		BIT(4)	/* PLL_MUX_SEL */
#define PLL_M_MASK		GENMASK(25, 16)
#define PLL_P_MASK		GENMASK(13, 8)
#define PLL_S_MASK		GENMASK(2, 0)

#define SW_USER_OSC		BIT(4)	/* 0 = OSCCLK_G3D, 1 = G3D switch */
#define MUX_BUSD_PLL		BIT(0)	/* 0 = PLL_G3D, 1 = SW_USER */
#define MUX_BUSD_BUSY		BIT(16)

#define PLL_LOCK_CYCLES		150
#define PLL_LOCK_UDELAY		300

struct g3d_pll_rate {
	unsigned long rate;
	u16 m;
	u8 p;
	u8 s;
};

/*
 * All settings keep the VCO (26 MHz * M / P) inside the PLL1052X 600-1200 MHz
 * band and the phase detector (26 MHz / P) inside its valid range.  P=26 is
 * used where it gives an exact integer M; P=13 and the output divider cover
 * the lower rates without pushing the VCO out of band.
 */
static const struct g3d_pll_rate g3d_rates[] = {
	{ 1053000000, 162,  4, 0 },
	{  949000000, 949, 26, 0 },
	{  839000000, 839, 26, 0 },
	{  764000000, 764, 26, 0 },
	{  683000000, 683, 26, 0 },
	{  572000000, 572, 13, 1 },
	{  546000000, 546, 13, 1 },
	{  455000000, 455, 13, 1 },
	{  385000000, 385, 13, 1 },
	{  338000000, 338, 13, 1 },
	{  260000000, 160,  4, 2 },
};

struct exynos9610_g3d_clk {
	struct clk_hw hw;
	struct device *dev;
	void __iomem *base;
	struct mutex lock;
	unsigned long rate;
};

static const struct g3d_pll_rate *g3d_rate_find(unsigned long rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g3d_rates); i++)
		if (g3d_rates[i].rate == rate)
			return &g3d_rates[i];

	return NULL;
}

static const struct g3d_pll_rate *g3d_rate_nearest(unsigned long rate)
{
	const struct g3d_pll_rate *best = &g3d_rates[0];
	unsigned long best_err = abs_diff(rate, best->rate);
	int i;

	for (i = 1; i < ARRAY_SIZE(g3d_rates); i++) {
		unsigned long err = abs_diff(rate, g3d_rates[i].rate);

		if (err < best_err) {
			best = &g3d_rates[i];
			best_err = err;
		}
	}

	return best;
}

static unsigned long g3d_clk_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct exynos9610_g3d_clk *g3d = container_of(hw,
						struct exynos9610_g3d_clk, hw);

	return g3d->rate;
}

static int g3d_clk_determine_rate(struct clk_hw *hw,
				  struct clk_rate_request *req)
{
	req->rate = g3d_rate_nearest(req->rate)->rate;

	return 0;
}

static int g3d_mux_wait(void __iomem *mux)
{
	unsigned int i;

	for (i = 0; i < 1000; i++) {
		if (!(readl(mux) & MUX_BUSD_BUSY))
			return 0;
		udelay(1);
	}

	return -ETIMEDOUT;
}

static int g3d_clk_set_rate_locked(struct exynos9610_g3d_clk *g3d,
				  const struct g3d_pll_rate *r)
{
	void __iomem *con0 = g3d->base + G3D_PLL_CON0;
	void __iomem *sw = g3d->base + G3D_SW_USER;
	void __iomem *mux = g3d->base + G3D_MUX_BUSD;
	u32 val;
	int ret = 0;

	/* Put G3D on the 26 MHz oscillator before touching PLL_G3D. */
	writel(readl(sw) & ~SW_USER_OSC, sw);
	udelay(2);
	writel(readl(mux) | MUX_BUSD_PLL, mux);
	ret = g3d_mux_wait(mux);
	if (ret)
		return ret;
	udelay(5);

	/* Same stop/start order as the CPU PLLs: mux out, disable, settle. */
	writel(readl(con0) & ~PLL_OUT_SEL, con0);
	udelay(2);
	writel(readl(con0) & ~PLL_ENABLE, con0);
	usleep_range(20, 40);

	writel(r->p * PLL_LOCK_CYCLES, g3d->base + G3D_PLL_LOCKTIME);

	val = readl(con0);
	val &= ~(PLL_M_MASK | PLL_P_MASK | PLL_S_MASK);
	val |= FIELD_PREP(PLL_M_MASK, r->m) |
	       FIELD_PREP(PLL_P_MASK, r->p) |
	       FIELD_PREP(PLL_S_MASK, r->s) |
	       PLL_OUT_SEL | PLL_ENABLE;
	writel(val, con0);
	udelay(PLL_LOCK_UDELAY);

	/* Hand the GPU back to the PLL even if the readback looks odd. */
	writel(readl(mux) & ~MUX_BUSD_PLL, mux);
	ret = g3d_mux_wait(mux);
	if (ret)
		return ret;

	if ((readl(con0) & (PLL_M_MASK | PLL_P_MASK | PLL_S_MASK |
			  PLL_ENABLE | PLL_OUT_SEL)) !=
	    (val & (PLL_M_MASK | PLL_P_MASK | PLL_S_MASK |
		    PLL_ENABLE | PLL_OUT_SEL)))
		return -EIO;

	g3d->rate = r->rate;
	pr_info("g3d-clk: %lu Hz (M=%u P=%u S=%u)\n",
		r->rate, r->m, r->p, r->s);

	return 0;
}

static int g3d_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct exynos9610_g3d_clk *g3d = container_of(hw,
						struct exynos9610_g3d_clk, hw);
	const struct g3d_pll_rate *r;
	int ret;

	r = g3d_rate_find(rate);
	if (!r)
		return -EINVAL;

	/*
	 * The G3D CMU registers are in the G3D power domain.  The clock
	 * provider is a device in that domain, so taking a runtime PM
	 * reference here makes the domain (and its TZPC save/restore) come
	 * up before any register is touched.
	 */
	ret = pm_runtime_get_sync(g3d->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(g3d->dev);
		return ret;
	}

	guard(mutex)(&g3d->lock);

	if (g3d->rate != r->rate)
		ret = g3d_clk_set_rate_locked(g3d, r);

	pm_runtime_put(g3d->dev);

	return ret;
}

static const struct clk_ops g3d_clk_ops = {
	.recalc_rate		= g3d_clk_recalc_rate,
	.determine_rate		= g3d_clk_determine_rate,
	.set_rate		= g3d_clk_set_rate,
};

static int exynos9610_g3d_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos9610_g3d_clk *g3d;
	struct clk_init_data init = { };
	u32 boot_hz = G3D_BOOT_HZ;
	int ret;

	g3d = devm_kzalloc(dev, sizeof(*g3d), GFP_KERNEL);
	if (!g3d)
		return -ENOMEM;

	g3d->dev = dev;

	g3d->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(g3d->base))
		return PTR_ERR(g3d->base);

	mutex_init(&g3d->lock);
	of_property_read_u32(dev->of_node, "clock-frequency", &boot_hz);
	g3d->rate = boot_hz;

	init.name = "g3d";
	init.ops = &g3d_clk_ops;
	init.num_parents = 0;
	g3d->hw.init = &init;

	ret = devm_clk_hw_register(dev, &g3d->hw);
	if (ret)
		return ret;

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, &g3d->hw);
	if (ret)
		return ret;

	pm_runtime_set_active(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	/*
	 * The provider starts active; drop the reference so the domain can
	 * power down with the GPU.  Every set_rate() takes it again.
	 */
	pm_runtime_get_noresume(dev);
	pm_runtime_put(dev);

	platform_set_drvdata(pdev, g3d);

	dev_info(dev, "registered at %u Hz\n", boot_hz);

	return 0;
}

static const struct of_device_id exynos9610_g3d_clk_of_match[] = {
	{ .compatible = "samsung,exynos9610-g3d-clock" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos9610_g3d_clk_of_match);

static struct platform_driver exynos9610_g3d_clk_driver = {
	.probe = exynos9610_g3d_clk_probe,
	.driver = {
		.name = "exynos9610-g3d-clock",
		.of_match_table = exynos9610_g3d_clk_of_match,
	},
};
module_platform_driver(exynos9610_g3d_clk_driver);

MODULE_DESCRIPTION("Samsung Exynos 9610 G3D clock control");
MODULE_LICENSE("GPL");
