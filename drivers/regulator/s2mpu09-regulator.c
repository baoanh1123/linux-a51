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
/*
 * LDOs and BUCKs both live on the S2MPU09 PMIC bank (ACPM PMIC type 1).
 * ACPM type 0 is the TOP/PMIC-ID bank and has no regulator registers.
 */
#define S2MPU09_PMIC_TYPE_PMIC	1

#define S2MPU09_LDO_ENABLE_MASK		GENMASK(7, 6)
#define S2MPU09_LDO_ENABLE_VAL		GENMASK(7, 6)
#define S2MPU09_LDO_VSEL_MASK		GENMASK(5, 0)
#define S2MPU09_LDO_N_VOLTAGES		BIT(6)
#define S2MPU09_XFER_RETRIES		5

struct s2mpu09_regulator {
	struct acpm_handle *acpm;
	u8 type;
	u8 ctrl_reg;
	u8 volt_reg;
	struct regulator_desc desc;
};

static int s2mpu09_read_reg(struct s2mpu09_regulator *reg, u8 addr, u8 *val)
{
	const struct acpm_pmic_ops *ops = &reg->acpm->ops.pmic_ops;
	int ret, i;

	for (i = 0; i < S2MPU09_XFER_RETRIES; i++) {
		ret = ops->read_reg(reg->acpm, S2MPU09_PMIC_CHAN,
				    reg->type, addr, 0, val);
		if (ret != -EAGAIN)
			break;
		usleep_range(1000, 2000);
	}

	return ret;
}

static int s2mpu09_write_reg(struct s2mpu09_regulator *reg, u8 addr, u8 val)
{
	const struct acpm_pmic_ops *ops = &reg->acpm->ops.pmic_ops;
	int ret, i;

	for (i = 0; i < S2MPU09_XFER_RETRIES; i++) {
		ret = ops->write_reg(reg->acpm, S2MPU09_PMIC_CHAN,
				     reg->type, addr, 0, val);
		if (ret != -EAGAIN)
			break;
		usleep_range(1000, 2000);
	}

	return ret;
}

static int s2mpu09_read(struct s2mpu09_regulator *reg, u8 *val)
{
	return s2mpu09_read_reg(reg, reg->ctrl_reg, val);
}

static int s2mpu09_update_bits(struct s2mpu09_regulator *reg, u8 addr,
			       u8 mask, u8 val)
{
	u8 old;
	int ret;

	ret = s2mpu09_read_reg(reg, addr, &old);
	if (ret)
		return ret;

	old &= ~mask;
	old |= val & mask;

	return s2mpu09_write_reg(reg, addr, old);
}

static int s2mpu09_ldo_enable(struct regulator_dev *rdev)
{
	struct s2mpu09_regulator *reg = rdev_get_drvdata(rdev);

	return s2mpu09_update_bits(reg, reg->ctrl_reg,
				   S2MPU09_LDO_ENABLE_MASK,
				   S2MPU09_LDO_ENABLE_VAL);
}

static int s2mpu09_ldo_disable(struct regulator_dev *rdev)
{
	struct s2mpu09_regulator *reg = rdev_get_drvdata(rdev);

	return s2mpu09_update_bits(reg, reg->ctrl_reg,
				   S2MPU09_LDO_ENABLE_MASK, 0);
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
	u8 old;
	int ret;

	ret = s2mpu09_read(reg, &old);
	if (ret)
		return ret;

	dev_info(rdev_get_dev(rdev), "set vsel=%u old=%#04x\n",
		 sel, old);

	return s2mpu09_update_bits(reg, reg->volt_reg,
				   S2MPU09_LDO_VSEL_MASK, sel);
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
 * BUCK5 (vdd_g3d) uses the same register layout as the LDOs: B5CTRL bits 7:6
 * are the enable field and B5OUT is the voltage selector.  The vendor kernel
 * describes it as always-on; keep the enable handling explicit so the rail
 * cannot silently stay off if a previous boot stage disabled it.
 */
static int s2mpu09_buck_set_voltage_sel(struct regulator_dev *rdev,
					unsigned int sel)
{
	struct s2mpu09_regulator *reg = rdev_get_drvdata(rdev);

	dev_info(rdev_get_dev(rdev), "set BUCK vsel=%u\n", sel);

	return s2mpu09_write_reg(reg, reg->volt_reg, sel);
}

static int s2mpu09_buck_get_voltage_sel(struct regulator_dev *rdev)
{
	struct s2mpu09_regulator *reg = rdev_get_drvdata(rdev);
	u8 val;
	int ret;

	ret = s2mpu09_read_reg(reg, reg->volt_reg, &val);
	if (ret)
		return ret;

	return val;
}

static const struct regulator_ops s2mpu09_buck_ops = {
	.enable			= s2mpu09_ldo_enable,
	.disable		= s2mpu09_ldo_disable,
	.is_enabled		= s2mpu09_ldo_is_enabled,
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.set_voltage_sel	= s2mpu09_buck_set_voltage_sel,
	.get_voltage_sel	= s2mpu09_buck_get_voltage_sel,
};

/*
 * min_uv/uV_step follow the vendor S2MPU09 regulator table (LDO_MINx and
 * LDO_STEPx in include/linux/mfd/samsung/s2mpu09-regulator.h).  boot_uv is
 * the voltage programmed by the eager enable below; power_cycle applies to
 * the microSD rails only.
 */
enum s2mpu09_rail_type {
	S2MPU09_RAIL_LDO,
	S2MPU09_RAIL_BUCK,
};

static const struct s2mpu09_rail_info {
	const char *node_name;
	u8 pmic_type;
	u8 ctrl_reg;
	u8 volt_reg;
	enum s2mpu09_rail_type type;
	int min_uv;
	int step_uv;
	int boot_uv;
	bool power_cycle;
} s2mpu09_rails[] = {
	{ "LDO2",  S2MPU09_PMIC_TYPE_PMIC, 0x3a, 0x3a, S2MPU09_RAIL_LDO, 1800000, 25000, 3300000, true },
	{ "LDO35", S2MPU09_PMIC_TYPE_PMIC, 0x5c, 0x5c, S2MPU09_RAIL_LDO, 1800000, 25000, 3300000, true },
	{ "LDO12", S2MPU09_PMIC_TYPE_PMIC, 0x45, 0x45, S2MPU09_RAIL_LDO,  700000, 12500,  850000, false },
	{ "LDO13", S2MPU09_PMIC_TYPE_PMIC, 0x46, 0x46, S2MPU09_RAIL_LDO,  700000, 25000, 1800000, false },
	{ "LDO14", S2MPU09_PMIC_TYPE_PMIC, 0x47, 0x47, S2MPU09_RAIL_LDO, 1800000, 25000, 3000000, false },
	/*
	 * vdd_g3d: selector written directly to B5OUT.  The selector base is
	 * 300 mV for every S2MPU09 buck, so min_uv here must stay at that base
	 * even though the DT constraint only lets consumers ask for 500 mV+.
	 * The vendor default is 800 mV; start at 900 mV so the GPU has a safe
	 * rail until its own OPP table takes over.
	 */
	{ "BUCK5", S2MPU09_PMIC_TYPE_PMIC, 0x20, 0x21, S2MPU09_RAIL_BUCK, 300000, 6250,  900000, false },
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
		rail->type = info->pmic_type;
		rail->ctrl_reg = info->ctrl_reg;
		rail->volt_reg = info->volt_reg;
		/*
		 * The regulator core keeps a pointer to this descriptor for
		 * the lifetime of the device, so it must not live on the
		 * stack.
		 */
		rail->desc.owner = THIS_MODULE;
		rail->desc.type = REGULATOR_VOLTAGE;
		rail->desc.name = child->name;
		rail->desc.min_uV = info->min_uv;
		rail->desc.uV_step = info->step_uv;
		if (info->type == S2MPU09_RAIL_BUCK) {
			rail->desc.ops = &s2mpu09_buck_ops;
			rail->desc.n_voltages =
				(1200000 - info->min_uv) / info->step_uv + 1;
		} else {
			rail->desc.ops = &s2mpu09_ldo_ops;
			rail->desc.n_voltages = S2MPU09_LDO_N_VOLTAGES;
		}

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
			u8 val = 0, vout = 0;
			int err;

			err = s2mpu09_read_reg(rail, rail->volt_reg, &vout);
			dev_info(dev, "%s vout before: %#04x (read=%d)\n",
				 child->name, vout, err);
			if (info->power_cycle) {
				s2mpu09_ldo_disable(rdev);
				msleep(1000);
				val = 0;
				err = s2mpu09_read(rail, &val);
				dev_info(dev, "%s after disable: %#04x (read=%d)\n",
					 child->name, val, err);
			}
			if (info->type == S2MPU09_RAIL_BUCK)
				err = s2mpu09_buck_set_voltage_sel(rdev, sel);
			else
				err = s2mpu09_ldo_set_voltage_sel(rdev, sel);
			if (err)
				dev_warn(dev, "failed to set %s to %d uV: %d\n",
					 child->name, info->boot_uv, err);
			msleep(100);
			ret = s2mpu09_ldo_enable(rdev);
			vout = 0;
			err = s2mpu09_read_reg(rail, rail->volt_reg, &vout);
			dev_info(dev,
				 "%s vout after: %#04x (sel=%u %d uV enable=%d read=%d)\n",
				 child->name, vout, sel, info->boot_uv, ret, err);
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
