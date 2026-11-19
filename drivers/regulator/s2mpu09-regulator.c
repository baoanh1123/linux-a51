// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal Samsung S2MPU09 regulator support over the Exynos ACPM PMIC
 * protocol.
 *
 * The Galaxy A51 (Exynos 9611) exposes the microSD and USB PHY rails on the
 * S2MPU09 PMIC bank (ACPM PMIC type 1, channel 2):
 *
 *   LDO2  CTRL1 0x3a - vqmmc, used at 1.8 V
 *   LDO35 CTRL  0x5c - vmmc,  used at 3.3 V
 *   LDO12 CTRL  0x45 - USB PHY, 850 mV in stock Android
 *   LDO13 CTRL  0x46 - USB PHY, 1.8 V in stock Android
 *   LDO14 CTRL  0x47 - USB PHY, 3.0 V in stock Android
 *
 * The USB rail voltages and enable state come from the stock kernel's
 * regulator summary, where all three rails are enabled with 13200000.usb as
 * the consumer (notes/device-dump/20261113-stock-android-live).
 *
 * Every register follows the vendor LDO layout: bits 7:6 are the enable
 * field (0x3 = enabled) and bits 5:0 select the output voltage, with the
 * start voltage and step depending on the rail.
 *
 * This is intentionally a small bring-up provider: it only registers the
 * rails described in the "regulators" child node, and all register access
 * goes through the ACPM PMIC read/write helpers so it follows the same path
 * the vendor kernel uses.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>

#define S2MPU09_PMIC_CHAN		2
#define S2MPU09_PMIC_TYPE		1

#define S2MPU09_LDO_ENABLE_MASK		GENMASK(7, 6)
#define S2MPU09_LDO_ENABLE_VAL		GENMASK(7, 6)
#define S2MPU09_LDO_VSEL_MASK		GENMASK(5, 0)
#define S2MPU09_LDO_N_VOLTAGES		BIT(6)

struct s2mpu09_regulator {
	struct acpm_handle *acpm;
	u8 ctrl_reg;
	struct regulator_desc desc;
};

static int s2mpu09_read(struct s2mpu09_regulator *reg, u8 *val)
{
	const struct acpm_pmic_ops *ops = &reg->acpm->ops.pmic_ops;

	return ops->read_reg(reg->acpm, S2MPU09_PMIC_CHAN,
			     S2MPU09_PMIC_TYPE, reg->ctrl_reg, 0, val);
}

static int s2mpu09_write(struct s2mpu09_regulator *reg, u8 val)
{
	const struct acpm_pmic_ops *ops = &reg->acpm->ops.pmic_ops;

	return ops->write_reg(reg->acpm, S2MPU09_PMIC_CHAN,
			      S2MPU09_PMIC_TYPE, reg->ctrl_reg, 0, val);
}

static int s2mpu09_update_bits(struct s2mpu09_regulator *reg, u8 mask, u8 val)
{
	u8 old;
	int ret;

	ret = s2mpu09_read(reg, &old);
	if (ret)
		return ret;

	old &= ~mask;
	old |= val & mask;

	return s2mpu09_write(reg, old);
}

static int s2mpu09_ldo_enable(struct regulator_dev *rdev)
{
	struct s2mpu09_regulator *reg = rdev_get_drvdata(rdev);

	return s2mpu09_update_bits(reg, S2MPU09_LDO_ENABLE_MASK,
				   S2MPU09_LDO_ENABLE_VAL);
}

static int s2mpu09_ldo_disable(struct regulator_dev *rdev)
{
	struct s2mpu09_regulator *reg = rdev_get_drvdata(rdev);

	return s2mpu09_update_bits(reg, S2MPU09_LDO_ENABLE_MASK, 0);
}

static int s2mpu09_ldo_is_enabled(struct regulator_dev *rdev)
{
	struct s2mpu09_regulator *reg = rdev_get_drvdata(rdev);
	u8 val;
	int ret;

	ret = s2mpu09_read(reg, &val);
	if (ret)
		return ret;

	return (val & S2MPU09_LDO_ENABLE_MASK) == S2MPU09_LDO_ENABLE_VAL;
}

static int s2mpu09_ldo_set_voltage_sel(struct regulator_dev *rdev,
				       unsigned int sel)
{
	struct s2mpu09_regulator *reg = rdev_get_drvdata(rdev);

	return s2mpu09_update_bits(reg, S2MPU09_LDO_VSEL_MASK, sel);
}

static int s2mpu09_ldo_get_voltage_sel(struct regulator_dev *rdev)
{
	struct s2mpu09_regulator *reg = rdev_get_drvdata(rdev);
	u8 val;
	int ret;

	ret = s2mpu09_read(reg, &val);
	if (ret)
		return ret;

	return val & S2MPU09_LDO_VSEL_MASK;
}

static const struct regulator_ops s2mpu09_ldo_ops = {
	.enable			= s2mpu09_ldo_enable,
	.disable		= s2mpu09_ldo_disable,
	.is_enabled		= s2mpu09_ldo_is_enabled,
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.set_voltage_sel	= s2mpu09_ldo_set_voltage_sel,
	.get_voltage_sel	= s2mpu09_ldo_get_voltage_sel,
};

/*
 * min_uv/uV_step follow the vendor S2MPU09 regulator table (LDO_MINx and
 * LDO_STEPx in include/linux/mfd/samsung/s2mpu09-regulator.h).  boot_uv is
 * the voltage programmed by the eager enable below; power_cycle applies to
 * the microSD rails only.
 */
static const struct s2mpu09_rail_info {
	const char *node_name;
	u8 ctrl_reg;
	int min_uv;
	int step_uv;
	int boot_uv;
	bool power_cycle;
} s2mpu09_rails[] = {
	{ "LDO2",  0x3a, 1800000, 25000, 3300000, true },
	{ "LDO35", 0x5c, 1800000, 25000, 3300000, true },
	{ "LDO12", 0x45,  700000, 12500,  850000, false },
	{ "LDO13", 0x46,  700000, 25000, 1800000, false },
	{ "LDO14", 0x47, 1800000, 25000, 3000000, false },
};

static const struct s2mpu09_rail_info *s2mpu09_rail_match(const char *name)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(s2mpu09_rails); i++)
		if (!strcmp(name, s2mpu09_rails[i].node_name))
			return &s2mpu09_rails[i];

	return NULL;
}

static int s2mpu09_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *regulators, *child;
	struct acpm_handle *acpm;
	int ret = 0;

	acpm = devm_acpm_get_by_node(dev, dev->parent->of_node);
	if (IS_ERR(acpm))
		return dev_err_probe(dev, PTR_ERR(acpm),
				     "failed to get ACPM handle\n");

	regulators = of_get_child_by_name(dev->of_node, "regulators");
	if (!regulators)
		return dev_err_probe(dev, -ENODEV,
				     "missing regulators child node\n");

	for_each_child_of_node(regulators, child) {
		const struct s2mpu09_rail_info *info;
		struct regulator_config config = { .dev = dev };
		struct s2mpu09_regulator *rail;
		struct regulator_dev *rdev;

		info = s2mpu09_rail_match(child->name);
		if (!info) {
			dev_dbg(dev, "skipping unsupported rail %s\n",
				child->name);
			continue;
		}

		rail = devm_kzalloc(dev, sizeof(*rail), GFP_KERNEL);
		if (!rail) {
			ret = -ENOMEM;
			goto put_child;
		}
		rail->acpm = acpm;
		rail->ctrl_reg = info->ctrl_reg;
		/*
		 * The regulator core keeps a pointer to this descriptor for
		 * the lifetime of the device, so it must not live on the
		 * stack.
		 */
		rail->desc.owner = THIS_MODULE;
		rail->desc.type = REGULATOR_VOLTAGE;
		rail->desc.ops = &s2mpu09_ldo_ops;
		rail->desc.name = child->name;
		rail->desc.min_uV = info->min_uv;
		rail->desc.uV_step = info->step_uv;
		rail->desc.n_voltages = S2MPU09_LDO_N_VOLTAGES;

		config.driver_data = rail;
		config.of_node = child;
		config.init_data = of_get_regulator_init_data(dev, child,
							      &rail->desc);
		if (!config.init_data) {
			ret = -ENOMEM;
			goto put_child;
		}

		rdev = devm_regulator_register(dev, &rail->desc, &config);
		if (IS_ERR(rdev)) {
			ret = dev_err_probe(dev, PTR_ERR(rdev),
					    "failed to register %s\n",
					    child->name);
			goto put_child;
		}

		/*
		 * Always-on rails are needed before their consumers probe;
		 * regulator_init_complete() runs later than device_initcall,
		 * so enable them here as well.
		 *
		 * Power-cycle the SD rails once at probe.  A previous
		 * kernel (e.g. stock Android) can leave the card latched
		 * in UHS/1.8 V signaling while the rails stay powered
		 * across reboot; the card then answers commands at 3.3 V
		 * but its data outputs are not readable.  Dropping power
		 * resets the card to its default 3.3 V state before the
		 * mainline MMC host initializes it.
		 *
		 * The USB PHY rails are only enabled at their stock
		 * voltages; no power cycle is needed there.
		 */
		if (of_property_read_bool(child, "regulator-always-on")) {
			unsigned int sel = (info->boot_uv - info->min_uv) /
					   info->step_uv;
			u8 val = 0;
			int err;

			err = s2mpu09_read(rail, &val);
			dev_info(dev, "%s before: %#04x (read=%d)\n",
				 child->name, val, err);
			if (info->power_cycle) {
				s2mpu09_ldo_disable(rdev);
				msleep(1000);
				val = 0;
				err = s2mpu09_read(rail, &val);
				dev_info(dev, "%s after disable: %#04x (read=%d)\n",
					 child->name, val, err);
			}
			err = s2mpu09_ldo_set_voltage_sel(rdev, sel);
			if (err)
				dev_warn(dev, "failed to set %s to %d uV: %d\n",
					 child->name, info->boot_uv, err);
			msleep(100);
			ret = s2mpu09_ldo_enable(rdev);
			val = 0;
			err = s2mpu09_read(rail, &val);
			dev_info(dev,
				 "%s after enable: %#04x (sel=%u %d uV enable=%d read=%d)\n",
				 child->name, val, sel, info->boot_uv, ret, err);
			if (ret)
				dev_warn(dev, "failed to pre-enable %s: %d\n",
					 child->name, ret);
		}

		dev_info(dev, "registered %s (ctrl %#04x)\n",
			 child->name, rail->ctrl_reg);
	}

	of_node_put(regulators);
	return 0;

put_child:
	of_node_put(child);
	of_node_put(regulators);
	return ret;
}

static const struct of_device_id s2mpu09_regulator_of_match[] = {
	{ .compatible = "samsung,s2mpu09-pmic" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpu09_regulator_of_match);

static struct platform_driver s2mpu09_regulator_driver = {
	.probe = s2mpu09_regulator_probe,
	.driver = {
		.name = "s2mpu09-regulator",
		.of_match_table = s2mpu09_regulator_of_match,
	},
};
module_platform_driver(s2mpu09_regulator_driver);

MODULE_DESCRIPTION("Samsung S2MPU09 regulators over Exynos ACPM");
MODULE_LICENSE("GPL");
