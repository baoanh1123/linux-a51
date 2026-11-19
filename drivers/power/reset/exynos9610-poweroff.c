// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 9610/9611 power off
 *
 * The generic "syscon-poweroff" pulls PS_HOLD low exactly once and then
 * returns to the kernel.  That is not enough on this SoC: the PMIC does not
 * always latch the low level straight away, and the kernel ends up sitting in
 * a halted state with the board still powered.  The vendor handler
 * (DOWNSTREAM_READONLY/drivers/soc/samsung/exynos-reboot.c:208-253) instead
 * keeps pulling PS_HOLD low in a loop and never returns, which is what this
 * driver does.
 *
 * The same register is used for reboot's sibling action, but reboot is a
 * one-shot SWRESET write and already works through "syscon-reboot", so only
 * power off is handled here.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

#define EXYNOS9610_PS_HOLD_CONTROL	0x330c
#define EXYNOS9610_PS_HOLD_MASK		BIT(8)

/**
 * struct exynos9610_poweroff - driver state
 * @dev:	device pointer
 * @regmap:	PMU system controller
 * @offset:	PS_HOLD_CONTROL offset inside the PMU
 * @mask:	bit that drives PS_HOLD low when cleared
 */
struct exynos9610_poweroff {
	struct device *dev;
	struct regmap *regmap;
	u32 offset;
	u32 mask;
};

static void exynos9610_do_poweroff(struct exynos9610_poweroff *pwr)
{
	int tries;

	pr_emerg("Power down: pulling PS_HOLD low\n");

	/*
	 * Keep the line low forever: once the PMIC notices, this CPU stops
	 * executing, so returning is not an option.
	 */
	for (tries = 1; ; tries++) {
		regmap_update_bits(pwr->regmap, pwr->offset, pwr->mask, 0);

		if (!(tries % 20))
			pr_emerg("Power down: still powered after %d tries\n",
				 tries);

		mdelay(50);
	}
}

static int exynos9610_poweroff_handler(struct sys_off_data *data)
{
	exynos9610_do_poweroff(data->cb_data);

	return NOTIFY_DONE;
}

static int exynos9610_poweroff_probe(struct platform_device *pdev)
{
	struct exynos9610_poweroff *pwr;
	int ret;

	pwr = devm_kzalloc(&pdev->dev, sizeof(*pwr), GFP_KERNEL);
	if (!pwr)
		return -ENOMEM;

	pwr->dev = &pdev->dev;
	pwr->offset = EXYNOS9610_PS_HOLD_CONTROL;
	pwr->mask = EXYNOS9610_PS_HOLD_MASK;

	pwr->regmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						     "samsung,pmu-syscon");
	if (IS_ERR(pwr->regmap))
		return dev_err_probe(&pdev->dev, PTR_ERR(pwr->regmap),
				     "Failed to get PMU regmap\n");

	ret = of_property_read_u32(pdev->dev.of_node, "offset", &pwr->offset);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Missing offset\n");

	ret = of_property_read_u32(pdev->dev.of_node, "mask", &pwr->mask);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Missing mask\n");

	ret = devm_register_power_off_handler(&pdev->dev,
					      exynos9610_poweroff_handler, pwr);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to register power off handler\n");

	dev_info(&pdev->dev, "Power off via PS_HOLD at %#x/%#x\n",
		 pwr->offset, pwr->mask);

	return 0;
}

static const struct of_device_id exynos9610_poweroff_of_match[] = {
	{ .compatible = "samsung,exynos9610-poweroff" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos9610_poweroff_of_match);

static struct platform_driver exynos9610_poweroff_driver = {
	.driver = {
		.name = "exynos9610-poweroff",
		.of_match_table = exynos9610_poweroff_of_match,
	},
	.probe = exynos9610_poweroff_probe,
};
module_platform_driver(exynos9610_poweroff_driver);

MODULE_DESCRIPTION("Exynos 9610/9611 power off");
MODULE_LICENSE("GPL");
