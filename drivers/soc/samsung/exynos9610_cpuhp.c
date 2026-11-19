// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos 9610/9611 CPU hotplug power sequencing
 *
 * The PSCI implementation provided by the bootloader on this SoC only knows
 * how to start cores that are already powered: CPU_ON works while the
 * bootloader holds the cores powered, but after a CPU_OFF the secure monitor
 * returns INTERNAL_FAILURE (reported by Linux as -EIO) and the core never
 * comes back.  The vendor kernel does not use PSCI for this at all, it drives
 * the CPUCFG block directly:
 *
 *   big cluster        CPUCL1_SHORTSTOP           0x10800000 + 0x0820
 *                      CLUSTER1_CPUSEQUENCER_OPTION 0x11860000 + 0x24a8
 *   per core           CONFIGURATION              0x11860000 + 0x2000 + 0x80 * n
 *                                                 (cluster 1: 0x2200 + ...)
 *
 * Writing 0xf into the low nibble of a core's configuration register powers
 * it up, 0x0 powers it down.  Doing that before PSCI is asked to start a CPU
 * is enough to make hotplug work, so the power gate is handled here and PSCI
 * keeps delivering the entry point, which it already does correctly on the
 * cold boot path.
 */

#include <linux/bitops.h>
#include <linux/cpuhotplug.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/smp.h>

#define EXYNOS9610_CPUCONFIG_CLUSTER0	0x2000
#define EXYNOS9610_CPUCONFIG_CLUSTER1	0x2200
#define EXYNOS9610_CPUCONFIG_STRIDE	0x80
#define EXYNOS9610_CPUCONFIG_ENABLE	GENMASK(3, 0)

#define EXYNOS9610_CPUCONFIG_SEQ_OPTION	0x24a8
#define EXYNOS9610_CPUCONFIG_SEQ_CL1	BIT(4)
#define EXYNOS9610_CPUCONFIG_SEQ_OFF	BIT(0)

#define EXYNOS9610_CMU_CPUCL1_SHORTSTOP	0x0820

struct exynos9610_cpuhp {
	struct device *dev;
	struct regmap *pmu;
	void __iomem *cmu;
};

static struct exynos9610_cpuhp *cpuhp_data;

static void exynos9610_cpuhp_set_core(struct exynos9610_cpuhp *priv,
				      unsigned int cpu, bool on)
{
	unsigned int cluster = cpu / 4;
	unsigned int offset;
	u32 val;

	offset = (cluster ? EXYNOS9610_CPUCONFIG_CLUSTER1
			  : EXYNOS9610_CPUCONFIG_CLUSTER0) +
		 EXYNOS9610_CPUCONFIG_STRIDE * (cpu % 4);

	val = 0;
	if (on)
		val = EXYNOS9610_CPUCONFIG_ENABLE;

	regmap_update_bits(priv->pmu, offset, EXYNOS9610_CPUCONFIG_ENABLE, val);
}

static int exynos9610_cpuhp_startup(unsigned int cpu)
{
	struct exynos9610_cpuhp *priv = cpuhp_data;

	if (!priv)
		return 0;

	if (cpu >= 4) {
		/* Release the big cluster before powering its cores. */
		writel(0xffffffff, priv->cmu + EXYNOS9610_CMU_CPUCL1_SHORTSTOP);

		regmap_update_bits(priv->pmu, EXYNOS9610_CPUCONFIG_SEQ_OPTION,
				   EXYNOS9610_CPUCONFIG_SEQ_CL1 |
				   EXYNOS9610_CPUCONFIG_SEQ_OFF,
				   EXYNOS9610_CPUCONFIG_SEQ_CL1);
	}

	exynos9610_cpuhp_set_core(priv, cpu, true);

	return 0;
}

static int exynos9610_cpuhp_teardown(unsigned int cpu)
{
	struct exynos9610_cpuhp *priv = cpuhp_data;

	if (!priv)
		return 0;

	exynos9610_cpuhp_set_core(priv, cpu, false);

	return 0;
}

/* Bring-up aid: force a core's power gate, "cpu on|off". */
static ssize_t power_write(struct file *file, const char __user *ubuf,
			   size_t count, loff_t *ppos)
{
	struct exynos9610_cpuhp *priv = file->private_data;
	unsigned int cpu;
	char buf[24], state[8];

	if (count == 0 || count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';
	if (sscanf(buf, "%u %7s", &cpu, state) != 2)
		return -EINVAL;

	if (cpu >= num_possible_cpus())
		return -EINVAL;

	exynos9610_cpuhp_set_core(priv, cpu, state[0] == 'o' && state[1] == 'n');
	*ppos += count;

	return count;
}

static const struct file_operations power_fops = {
	.open = simple_open,
	.write = power_write,
	.llseek = noop_llseek,
};

static int exynos9610_cpuhp_probe(struct platform_device *pdev)
{
	struct exynos9610_cpuhp *priv;
	struct device *dev = &pdev->dev;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	priv->pmu = syscon_regmap_lookup_by_phandle(dev->of_node,
						    "samsung,pmu-syscon");
	if (IS_ERR(priv->pmu))
		return dev_err_probe(dev, PTR_ERR(priv->pmu),
				     "failed to get PMU syscon\n");

	priv->cmu = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->cmu))
		return dev_err_probe(dev, PTR_ERR(priv->cmu),
				     "failed to map CMU\n");

	cpuhp_data = priv;
	platform_set_drvdata(pdev, priv);

	/*
	 * The startup callback of a BP_PREPARE state runs on the controlling
	 * CPU before the target CPU is brought up, which is exactly where the
	 * power gate has to be programmed.
	 */
	ret = cpuhp_setup_state_nocalls(CPUHP_BP_PREPARE_DYN,
					"soc/samsung:exynos9610-cpuhp",
					exynos9610_cpuhp_startup,
					exynos9610_cpuhp_teardown);
	if (ret < 0) {
		cpuhp_data = NULL;
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register hotplug state\n");
	}

	debugfs_create_file("power", 0200, NULL, priv, &power_fops);

	dev_info(&pdev->dev, "CPUCFG power sequencing registered\n");

	return 0;
}

static void exynos9610_cpuhp_remove(struct platform_device *pdev)
{
	cpuhp_data = NULL;
}

static const struct of_device_id exynos9610_cpuhp_match[] = {
	{ .compatible = "samsung,exynos9610-cpuhp" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos9610_cpuhp_match);

static struct platform_driver exynos9610_cpuhp_driver = {
	.probe = exynos9610_cpuhp_probe,
	.remove = exynos9610_cpuhp_remove,
	.driver = {
		.name = "exynos9610-cpuhp",
		.of_match_table = exynos9610_cpuhp_match,
	},
};
module_platform_driver(exynos9610_cpuhp_driver);

MODULE_DESCRIPTION("Samsung Exynos 9610/9611 CPU hotplug power sequencing");
MODULE_LICENSE("GPL");
