// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 9610 DISPAUD power domain.
 *
 * CMU_DISPAUD, the ABOX register block, the ABOX SRAM and the ABOX SysMMU
 * all sit behind the DISPAUD power domain.  The domain is off when the
 * bootloader hands over, and touching any of those blocks while it is off
 * wedges the AXI bus, so the domain has to be powered before any of the
 * audio drivers probe.
 *
 * The PMU handshake is the one from the vendor pmucal tables:
 *
 *   PMU_DISPAUD_CONFIGURATION (0x11864000) = 0xf
 *   wait for PMU_DISPAUD_STATUS (0x11864004) & 0xf == 0xf
 *
 * There is no TZPC or SMC handshake in the vendor DISPAUD sequence.  The
 * domain is kept powered for now: turning it off needs the CMU/PLL state
 * save and restore that the vendor clock driver does, and nothing needs it
 * yet.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>

#define DISPAUD_PMU_CONFIG	0x0
#define DISPAUD_PMU_STATUS	0x4
#define DISPAUD_PMU_ON		0xf

#define TOP_GATE_DISPAUD_AUD	0x2038
#define TOP_GATE_DISPAUD_CPU	0x203c
#define AUD_GATE_SMMU_ABOX	0x20b0
#define GATE_ON			BIT(21)

struct dispaud_pd {
	struct generic_pm_domain genpd;
	void __iomem *pmu;
	void __iomem *cmu;
	void __iomem *top;
};

static int dispaud_pd_power_on(struct generic_pm_domain *genpd)
{
	struct dispaud_pd *pd = container_of(genpd, struct dispaud_pd, genpd);
	unsigned int status;
	int i;

	writel(DISPAUD_PMU_ON, pd->pmu + DISPAUD_PMU_CONFIG);

	for (i = 0; i < 100; i++) {
		status = readl(pd->pmu + DISPAUD_PMU_STATUS);
		if ((status & DISPAUD_PMU_ON) == DISPAUD_PMU_ON) {
			/* Open the gates the ABOX and its SysMMU need. */
			if (pd->top) {
				writel(GATE_ON, pd->top + TOP_GATE_DISPAUD_AUD);
				writel(GATE_ON, pd->top + TOP_GATE_DISPAUD_CPU);
			}
			if (pd->cmu)
				writel(GATE_ON, pd->cmu + AUD_GATE_SMMU_ABOX);
			return 0;
		}
		udelay(10);
	}

	pr_err("dispaud: power on timed out, status=%#x\n", status);
	return -ETIMEDOUT;
}

static int dispaud_pd_power_off(struct generic_pm_domain *genpd)
{
	/* Keep the domain powered until clock save/restore exists. */
	return 0;
}

static int __init dispaud_pd_init(void)
{
	struct device_node *np;
	struct dispaud_pd *pd;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos9610-dispaud-pd");
	if (!np)
		return 0;

	pd = kzalloc(sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		of_node_put(np);
		return -ENOMEM;
	}

	pd->pmu = of_iomap(np, 0);
	if (!pd->pmu) {
		pr_err("dispaud: cannot map PMU registers\n");
		kfree(pd);
		of_node_put(np);
		return -ENOMEM;
	}

	/*
	 * Map the gate registers now, but leave the domain off: platform_probe()
	 * powers a device's power domain on before calling its driver, so the
	 * ABOX SysMMU gets its clock gate opened just before exynos-iommu reads
	 * the version register.
	 */
	{
		struct device_node *abox_np;

		abox_np = of_find_compatible_node(NULL, NULL,
						  "samsung,exynos9610-abox");
		if (abox_np) {
			pd->cmu = of_iomap(abox_np, 1);
			pd->top = of_iomap(abox_np, 2);
			of_node_put(abox_np);
		}
	}

	pd->genpd.name = "pd-dispaud";
	pd->genpd.power_on = dispaud_pd_power_on;
	pd->genpd.power_off = dispaud_pd_power_off;

	/*
	 * Register the domain off.  The ABOX probe asks for it through runtime
	 * PM, so a machine that never touches audio never writes the PMU
	 * registers at all.  Doing this in postcore was what made a bad
	 * DISPAUD state hang the whole boot before display came up.
	 */
	pm_genpd_init(&pd->genpd, NULL, true);

	ret = of_genpd_add_provider_simple(np, &pd->genpd);
	if (ret) {
		pr_err("dispaud: cannot register power domain: %d\n", ret);
		pm_genpd_remove(&pd->genpd);
		iounmap(pd->pmu);
		kfree(pd);
	}

	of_node_put(np);
	return ret;
}
postcore_initcall(dispaud_pd_init);
