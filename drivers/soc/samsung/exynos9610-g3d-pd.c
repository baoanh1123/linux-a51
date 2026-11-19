// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos 9610 G3D power-domain experiment driver.
 *
 * The Mali-G72 sits in its own power domain (vendor DT: pd-g3d@11864060, i.e.
 * PMU + 0x4060) plus its own CMU (CMU_G3D at 0x11430000).  Mainline currently
 * leaves that domain powered for the whole uptime, because the generic
 * exynos-pm-domains driver only writes LOCAL_PWR_CFG and polls STATUS, while
 * this domain additionally needs:
 *
 *   - the G3D QoS channel (QCH_CON_G3D_QCH) quiesced before the switch,
 *   - nine PMU SYS_PWR registers programmed so the CMU is not reset and the
 *     PLL is not disabled,
 *   - and, on power-up, the CMU_G3D register file restored, because the CMU
 *     and PLL state does not survive the domain losing power.
 *
 * The sequences below are a direct port of the vendor code
 * (DOWNSTREAM_READONLY/drivers/soc/samsung/cal-if/exynos9610/
 * pmucal_local_exynos9610.h: g3d_on[], g3d_off[], g3d_save[]).
 *
 * The sequence was first validated by hand through the debugfs knobs below
 * (with panfrost idle), which is how the missing piece was found: powering the
 * domain down without the vendor's TZPC save/restore SMC raises a SError.  With
 * that SMC in place the cycle is clean - PLL_G3D comes back locked and kmscube
 * renders through panfrost afterwards - so the domain is now also registered as
 * a genpd, and the gpu node references it with "power-domains".
 */

#include <linux/arm-smccc.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

/* PMU registers used by the G3D sequence. */
#define PMU_G3D_CONFIGURATION	0x4060
#define PMU_G3D_STATUS		0x4064
#define PMU_CLKSTOP_OPEN	0x13cc
#define PMU_FORCE_AUTOCLKGATE	0x144c
#define PMU_CLKSTOP		0x148c
#define PMU_DISABLE_PLL		0x14cc
#define PMU_RESET_LOGIC		0x150c
#define PMU_MEMORY		0x154c
#define PMU_RESET_CMU		0x158c
#define PMU_LPI_RESIDUAL	0x160c

/* CMU_G3D registers. */
#define CMU_G3D_QCH_G3D		0x301c
#define CMU_G3D_PLL_CON0	0x0120

#define G3D_ON_MASK		0xfu
#define G3D_OFF_QCH		0x2u
#define G3D_ON_QCH		0x6u

#define G3D_POLL_LOOPS		100
#define G3D_POLL_US		100

/*
 * Powering the domain off while the GPU is active, or without the TZPC
 * save/restore SMC in place, panics the SoC with a SError (see notes/gpu.md).
 * The manual debugfs knobs are therefore fenced off behind an explicit opt-in;
 * automatic gating through genpd is always allowed.
 */
static bool allow_power_off;
module_param(allow_power_off, bool, 0644);
MODULE_PARM_DESC(allow_power_off, "Allow the manual debugfs power off/on test");

enum g3d_block {
	G3D_BLK_CMU,
	G3D_BLK_CMU_PWR,
};

/**
 * struct g3d_reg - one register that has to survive a domain power cycle
 * @base: register block the offset belongs to (filled in at init)
 * @off:  register offset within that block
 * @blk:  which of the two register files @off is relative to
 * @val:  value captured before power down
 */
struct g3d_reg {
	void __iomem *base;
	u32 off;
	enum g3d_block blk;
	u32 val;
};

/*
 * The vendor wraps every power-domain transition in a call to the EL3 monitor,
 * which saves/restores the TZPC state of the domain in secure world
 * (DOWNSTREAM_READONLY/drivers/soc/samsung/exynos-el3_mon.c:
 * exynos_pd_tz_save()/exynos_pd_tz_restore(), plain "smc #0" with x0..x3).
 * Skipping it is what made the first power-off attempt die with a SError.
 */
#define EXYNOS_SMC_PREPARE_PD_ONOFF	0x82000410
#define EXYNOS_PD_GET_IN_PD_DOWN	0
#define EXYNOS_PD_WAKEUP_PD_DOWN	1
#define G3D_TZPC_ADDR			0x11410204

static long g3d_tz_smc(u32 op)
{
	struct arm_smccc_res res;

	arm_smccc_smc(EXYNOS_SMC_PREPARE_PD_ONOFF, op, G3D_TZPC_ADDR, 0,
		      0, 0, 0, 0, &res);

	return (long)res.a0;
}

static void __iomem *pmu;
static void __iomem *cmu_g3d;
static void __iomem *cmu_pwr;
static struct dentry *g3d_debugfs;
static DEFINE_MUTEX(g3d_lock);
static bool g3d_is_off;

/*
 * Everything the vendor saves before switching the domain off and restores
 * afterwards (g3d_save[]).  QCH_CON_G3D_QCH is captured too, but its low bits
 * are written explicitly by the on/off sequences.
 */
static struct g3d_reg g3d_saved[] = {
	{ .off = 0x0100 },	/* PLL_CON0_MUX_CLKCMU_G3D_SWITCH_USER */
	{ .off = 0x0120 },	/* PLL_CON0_PLL_G3D */
	{ .off = 0x0124 },	/* PLL_CON1_PLL_G3D */
	{ .off = 0x0128 },	/* PLL_CON2_PLL_G3D */
	{ .off = 0x0130 },	/* PLL_CON4_PLL_G3D */
	{ .off = 0x0800 },	/* G3D_CMU_G3D_CONTROLLER_OPTION */
	{ .off = 0x0820 },	/* G3D_SHORTSTOP */
	{ .off = 0x1004 },	/* CLK_CON_MUX_MUX_CLK_G3D_BUSD */
	{ .off = 0x1804 },	/* CLK_CON_DIV_DIV_CLK_G3D_BUSP */
	{ .off = 0x3010 },	/* QCH_CON_BTM_G3D_QCH */
	{ .off = 0x3014 },	/* QCH_CON_BUSIF_HPMG3D_QCH */
	{ .off = 0x3018 },	/* QCH_CON_G3D_CMU_G3D_QCH */
	{ .off = 0x301c },	/* QCH_CON_G3D_QCH */
	{ .off = 0x3020 },	/* QCH_CON_LHM_AXI_G3DSFR_QCH */
	{ .off = 0x3024 },	/* QCH_CON_LHM_AXI_P_G3D_QCH */
	{ .off = 0x3028 },	/* QCH_CON_LHS_AXI_D_G3D_QCH */
	{ .off = 0x302c },	/* QCH_CON_LHS_AXI_G3DSFR_QCH */
	{ .off = 0x3030 },	/* QCH_CON_PGEN_LITE_G3D_QCH */
	{ .off = 0x3034 },	/* QCH_CON_SYSREG_G3D_QCH */
	/* cmu_pwr block: LPP low-power state and the DRCG enable. */
	{ .off = 0x0300, .blk = G3D_BLK_CMU_PWR },	/* LPP_MCS */
	{ .off = 0x0304, .blk = G3D_BLK_CMU_PWR },	/* LPP_MCSW */
	{ .off = 0x0308, .blk = G3D_BLK_CMU_PWR },	/* LPP_MCSRD */
	{ .off = 0x0104, .blk = G3D_BLK_CMU_PWR },	/* BUS_COMPONENT_DRCG_EN */
};

static void g3d_update(void __iomem *base, u32 off, u32 mask, u32 val)
{
	u32 tmp = readl_relaxed(base + off);

	tmp = (tmp & ~mask) | (val & mask);
	writel_relaxed(tmp, base + off);
}

static int g3d_wait_status(u32 want)
{
	int i;

	for (i = 0; i < G3D_POLL_LOOPS; i++) {
		if ((readl_relaxed(pmu + PMU_G3D_STATUS) & G3D_ON_MASK) == want)
			return 0;
		usleep_range(G3D_POLL_US, G3D_POLL_US + 20);
	}

	return -ETIMEDOUT;
}

static void g3d_save_state(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(g3d_saved); i++) {
		struct g3d_reg *r = &g3d_saved[i];

		r->val = readl_relaxed(r->base + r->off);
	}
}

static void g3d_restore_state(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(g3d_saved); i++)
		writel_relaxed(g3d_saved[i].val, g3d_saved[i].base + g3d_saved[i].off);
}

/*
 * g3d_power_off - vendor g3d_off[]: quiesce the QoS channel, program the PMU
 * clock/reset/retention controls, then drop the domain.
 */
static int g3d_power_off(void)
{
	long smc;

	g3d_save_state();

	smc = g3d_tz_smc(EXYNOS_PD_GET_IN_PD_DOWN);
	if (smc)
		pr_warn("exynos9610-g3d: TZPC save SMC returned %ld\n", smc);

	g3d_update(cmu_g3d, CMU_G3D_QCH_G3D, 0x7, G3D_OFF_QCH);
	g3d_update(pmu, PMU_CLKSTOP_OPEN, 0x3, 0x3);
	g3d_update(pmu, PMU_FORCE_AUTOCLKGATE, 0x1, 0x0);
	g3d_update(pmu, PMU_CLKSTOP, 0x1, 0x1);
	g3d_update(pmu, PMU_DISABLE_PLL, 0x1, 0x0);
	g3d_update(pmu, PMU_RESET_LOGIC, 0x3, 0x0);
	g3d_update(pmu, PMU_MEMORY, 0x3, 0x0);
	g3d_update(pmu, PMU_RESET_CMU, 0x3, 0x0);
	g3d_update(pmu, PMU_LPI_RESIDUAL, 0x1, 0x0);

	writel_relaxed(0, pmu + PMU_G3D_CONFIGURATION);

	return g3d_wait_status(0);
}

/*
 * g3d_power_on - vendor g3d_on[] plus the restore half of g3d_save[]: the
 * CMU_G3D and PLL register file is gone, so it is written back before the QoS
 * channel is handed back to the GPU.
 */
static int g3d_power_on(void)
{
	int ret;

	writel_relaxed(G3D_ON_MASK, pmu + PMU_G3D_CONFIGURATION);

	long smc;

	ret = g3d_wait_status(G3D_ON_MASK);
	if (ret)
		return ret;

	smc = g3d_tz_smc(EXYNOS_PD_WAKEUP_PD_DOWN);
	if (smc)
		pr_warn("exynos9610-g3d: TZPC restore SMC returned %ld\n", smc);

	g3d_restore_state();
	g3d_update(cmu_g3d, CMU_G3D_QCH_G3D, 0x7, G3D_ON_QCH);

	return 0;
}

struct g3d_pd {
	struct generic_pm_domain genpd;
};

static int g3d_pd_power_on(struct generic_pm_domain *genpd)
{
	int ret;

	mutex_lock(&g3d_lock);
	ret = g3d_power_on();
	if (!ret)
		g3d_is_off = false;
	mutex_unlock(&g3d_lock);

	return ret;
}

static int g3d_pd_power_off(struct generic_pm_domain *genpd)
{
	int ret;

	mutex_lock(&g3d_lock);
	ret = g3d_power_off();
	if (!ret)
		g3d_is_off = true;
	mutex_unlock(&g3d_lock);

	return ret;
}

static int g3d_dump_show(struct seq_file *s, void *unused)
{
	static const struct {
		const char *name;
		u32 off;
	} pmu_regs[] = {
		{ "G3D_CONFIGURATION", PMU_G3D_CONFIGURATION },
		{ "G3D_STATUS", PMU_G3D_STATUS },
		{ "CLKSTOP_OPEN", PMU_CLKSTOP_OPEN },
		{ "FORCE_AUTOCLKGATE", PMU_FORCE_AUTOCLKGATE },
		{ "CLKSTOP", PMU_CLKSTOP },
		{ "DISABLE_PLL", PMU_DISABLE_PLL },
		{ "RESET_LOGIC", PMU_RESET_LOGIC },
		{ "MEMORY", PMU_MEMORY },
		{ "RESET_CMU", PMU_RESET_CMU },
		{ "LPI_RESIDUAL", PMU_LPI_RESIDUAL },
	};
	unsigned int i;

	mutex_lock(&g3d_lock);

	seq_printf(s, "domain: %s\n", g3d_is_off ? "off" : "on");
	for (i = 0; i < ARRAY_SIZE(pmu_regs); i++)
		seq_printf(s, "pmu  %-18s 0x%04x = 0x%08x\n", pmu_regs[i].name,
			   pmu_regs[i].off, readl_relaxed(pmu + pmu_regs[i].off));

	/*
	 * CMU_G3D lives inside the domain: touching it while the domain is off
	 * can stall the interconnect, so only the (ALIVE-domain) PMU registers
	 * are safe to read in that state.
	 */
	if (!g3d_is_off) {
		u32 pll = readl_relaxed(cmu_g3d + CMU_G3D_PLL_CON0);

		seq_printf(s, "cmu  PLL_CON0           0x0120 = 0x%08x (lock=%u enable=%u)\n",
			   pll, (pll >> 29) & 1, (pll >> 4) & 1);
		seq_printf(s, "cmu  QCH_CON_G3D_QCH    0x301c = 0x%08x\n",
			   readl_relaxed(cmu_g3d + CMU_G3D_QCH_G3D));
	} else {
		seq_puts(s, "cmu  (not readable while the domain is off)\n");
	}

	mutex_unlock(&g3d_lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(g3d_dump);

static ssize_t g3d_off_write(struct file *f, const char __user *buf,
			     size_t len, loff_t *ppos)
{
	char cmd[8];
	int ret;

	if (len >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, buf, len))
		return -EFAULT;
	cmd[len] = '\0';

	if (!allow_power_off) {
		pr_err("exynos9610-g3d: refusing to power off; load with allow_power_off=1 (see notes/gpu.md)\n");
		return -EPERM;
	}

	mutex_lock(&g3d_lock);
	if (!g3d_is_off) {
		ret = g3d_power_off();
		if (ret) {
			pr_err("exynos9610-g3d: power off failed (%d)\n", ret);
		} else {
			g3d_is_off = true;
			pr_info("exynos9610-g3d: domain powered off\n");
		}
	}
	mutex_unlock(&g3d_lock);

	return ret ? ret : len;
}

static ssize_t g3d_on_write(struct file *f, const char __user *buf,
			    size_t len, loff_t *ppos)
{
	char cmd[8];
	int ret = 0;

	if (len >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, buf, len))
		return -EFAULT;
	cmd[len] = '\0';

	mutex_lock(&g3d_lock);
	if (g3d_is_off) {
		ret = g3d_power_on();
		if (ret) {
			pr_err("exynos9610-g3d: power on failed (%d)\n", ret);
		} else {
			g3d_is_off = false;
			pr_info("exynos9610-g3d: domain powered on\n");
		}
	}
	mutex_unlock(&g3d_lock);

	return ret ? ret : len;
}

static const struct file_operations g3d_off_fops = {
	.owner = THIS_MODULE,
	.write = g3d_off_write,
	.llseek = noop_llseek,
};

static const struct file_operations g3d_on_fops = {
	.owner = THIS_MODULE,
	.write = g3d_on_write,
	.llseek = noop_llseek,
};

static ssize_t g3d_tz_write(struct file *f, const char __user *buf,
			    size_t len, loff_t *ppos)
{
	long save, restore;

	mutex_lock(&g3d_lock);
	save = g3d_tz_smc(EXYNOS_PD_GET_IN_PD_DOWN);
	restore = g3d_tz_smc(EXYNOS_PD_WAKEUP_PD_DOWN);
	mutex_unlock(&g3d_lock);

	pr_info("exynos9610-g3d: TZPC smc save=%ld restore=%ld\n", save, restore);

	return len;
}

static const struct file_operations g3d_tz_fops = {
	.owner = THIS_MODULE,
	.write = g3d_tz_write,
	.llseek = noop_llseek,
};

static void __iomem *g3d_ioremap(struct platform_device *pdev, const char *name)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return ERR_PTR(-ENODEV);

	return devm_ioremap(&pdev->dev, res->start, resource_size(res));
}

static int g3d_pd_probe(struct platform_device *pdev)
{
	struct g3d_pd *pd;
	unsigned int i;
	int ret;

	/*
	 * These windows are ioremapped, not claimed: the PMU block already
	 * belongs to the exynos-pmu syscon device, and requesting it again
	 * fails with -EBUSY.
	 */
	pmu = g3d_ioremap(pdev, "pmu");
	if (IS_ERR(pmu))
		return PTR_ERR(pmu);

	cmu_g3d = g3d_ioremap(pdev, "cmu-g3d");
	if (IS_ERR(cmu_g3d))
		return PTR_ERR(cmu_g3d);

	cmu_pwr = g3d_ioremap(pdev, "cmu-pwr");
	if (IS_ERR(cmu_pwr))
		return PTR_ERR(cmu_pwr);

	for (i = 0; i < ARRAY_SIZE(g3d_saved); i++)
		g3d_saved[i].base = g3d_saved[i].blk == G3D_BLK_CMU_PWR ?
				    cmu_pwr : cmu_g3d;

	pd = devm_kzalloc(&pdev->dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->genpd.name = "pd-g3d";
	pd->genpd.power_on = g3d_pd_power_on;
	pd->genpd.power_off = g3d_pd_power_off;

	/* The boot chain leaves the domain powered, so start out "on". */
	ret = pm_genpd_init(&pd->genpd, NULL, false);
	if (ret)
		return ret;

	ret = of_genpd_add_provider_simple(pdev->dev.of_node, &pd->genpd);
	if (ret) {
		pm_genpd_remove(&pd->genpd);
		return ret;
	}

	platform_set_drvdata(pdev, pd);

	g3d_debugfs = debugfs_create_dir("exynos9610-g3d", NULL);
	debugfs_create_file("dump", 0400, g3d_debugfs, NULL, &g3d_dump_fops);
	debugfs_create_file("off", 0200, g3d_debugfs, NULL, &g3d_off_fops);
	debugfs_create_file("on", 0200, g3d_debugfs, NULL, &g3d_on_fops);
	debugfs_create_file("tz-probe", 0200, g3d_debugfs, NULL, &g3d_tz_fops);

	dev_info(&pdev->dev, "G3D power domain registered\n");

	return 0;
}

static void g3d_pd_remove(struct platform_device *pdev)
{
	struct g3d_pd *pd = platform_get_drvdata(pdev);

	debugfs_remove_recursive(g3d_debugfs);
	of_genpd_del_provider(pdev->dev.of_node);
	pm_genpd_remove(&pd->genpd);
}

static const struct of_device_id g3d_pd_of_match[] = {
	{ .compatible = "samsung,exynos9610-g3d-pd" },
	{ }
};
MODULE_DEVICE_TABLE(of, g3d_pd_of_match);

static struct platform_driver g3d_pd_driver = {
	.probe = g3d_pd_probe,
	.remove = g3d_pd_remove,
	.driver = {
		.name = "exynos9610-g3d-pd",
		.of_match_table = g3d_pd_of_match,
	},
};
module_platform_driver(g3d_pd_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exynos 9610 G3D power-domain experiment");
