// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5713 fuel gauge
 *
 * The SM5713 combines a switch-mode charger, a fuel gauge and a MUIC in one
 * package.  Only the fuel gauge is handled here; it lives on its own I2C
 * address (0x71) so no MFD is needed to reach it.  Register layout and the
 * fixed-point conversions come from the vendor driver,
 * DOWNSTREAM_READONLY/drivers/battery_v2/sm5713_fuelgauge.c and
 * DOWNSTREAM_READONLY/include/linux/mfd/sm5713-private.h.
 *
 * The device only exposes 16-bit registers; they are little endian (SMBus
 * word order), which matches what i2c_smbus_read_word_data() returns and was
 * verified on hardware by reading VBAT/SoC/temperature.
 */

#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

/* Fuel gauge registers (SM5713_FG_REG_* in the vendor header) */
#define SM5713_FG_DEVICE_ID	0x00
#define SM5713_FG_SOC		0x05
#define SM5713_FG_OCV		0x06
#define SM5713_FG_VBAT		0x07
#define SM5713_FG_CURRENT	0x08
#define SM5713_FG_TEMPERATURE	0x09
#define SM5713_FG_VSYS		0x0b

/* Masks and fixed-point scales */
#define SM5713_FG_VAL_MASK	GENMASK(13, 0)
#define SM5713_FG_SIGN		BIT(15)
#define SM5713_FG_SOC_INT	GENMASK(14, 8)
#define SM5713_FG_SOC_FRAC	GENMASK(7, 0)
#define SM5713_FG_TEMP_DATA	GENMASK(14, 4)

struct sm5713_fg {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *battery;
};

/*
 * Voltage and current share a scale of 1000/2048 per LSB, i.e. 0.488 mV and
 * 0.488 mA respectively.
 */
static int sm5713_fg_scale(u32 raw)
{
	return DIV_ROUND_CLOSEST((raw & SM5713_FG_VAL_MASK) * 1000, 2048);
}

static int sm5713_fg_read_soc(struct sm5713_fg *fg, int *decipercent)
{
	unsigned int reg;
	int ret;

	ret = regmap_read(fg->regmap, SM5713_FG_SOC, &reg);
	if (ret)
		return ret;

	if (reg & SM5713_FG_SIGN) {
		*decipercent = 0;
		return 0;
	}

	*decipercent = FIELD_GET(SM5713_FG_SOC_INT, reg) * 10;
	*decipercent += (FIELD_GET(SM5713_FG_SOC_FRAC, reg) * 10) / 256;

	return 0;
}

static int sm5713_fg_read_temp(struct sm5713_fg *fg, int *decidegc)
{
	unsigned int reg;
	int ret;

	ret = regmap_read(fg->regmap, SM5713_FG_TEMPERATURE, &reg);
	if (ret)
		return ret;

	*decidegc = DIV_ROUND_CLOSEST(FIELD_GET(SM5713_FG_TEMP_DATA, reg) * 10, 16);
	if (reg & SM5713_FG_SIGN)
		*decidegc = -*decidegc;

	return 0;
}

static int sm5713_fg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct sm5713_fg *fg = power_supply_get_drvdata(psy);
	unsigned int reg;
	int value;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = sm5713_fg_read_soc(fg, &value);
		if (ret)
			return ret;
		val->intval = value / 10;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = regmap_read(fg->regmap, SM5713_FG_VBAT, &reg);
		if (ret)
			return ret;
		val->intval = sm5713_fg_scale(reg) * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = regmap_read(fg->regmap, SM5713_FG_CURRENT, &reg);
		if (ret)
			return ret;
		value = sm5713_fg_scale(reg);
		if (reg & SM5713_FG_SIGN)
			value = -value;
		val->intval = value * 1000;
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		ret = sm5713_fg_read_temp(fg, &value);
		if (ret)
			return ret;
		val->intval = value;
		return 0;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property sm5713_fg_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc sm5713_fg_desc = {
	.name			= "battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= sm5713_fg_props,
	.num_properties		= ARRAY_SIZE(sm5713_fg_props),
	.get_property		= sm5713_fg_get_property,
};

static const struct regmap_config sm5713_fg_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 16,
	.val_format_endian	= REGMAP_ENDIAN_LITTLE,
	.max_register		= SM5713_FG_VSYS,
};

static int sm5713_fg_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct device *dev = &client->dev;
	struct sm5713_fg *fg;
	unsigned int id;
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->dev = dev;
	fg->regmap = devm_regmap_init_i2c(client, &sm5713_fg_regmap_config);
	if (IS_ERR(fg->regmap))
		return dev_err_probe(dev, PTR_ERR(fg->regmap),
				     "failed to initialise regmap\n");

	/*
	 * Fail early on a wrong address instead of reporting bogus battery
	 * data: the register is 0x01 on SM5713 and reads back as 0xff on an
	 * empty bus.
	 */
	ret = regmap_read(fg->regmap, SM5713_FG_DEVICE_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read device ID\n");
	if (id == 0 || id == 0xff || id == 0xffff)
		return dev_err_probe(dev, -ENODEV, "bad device ID 0x%x\n", id);

	psy_cfg.drv_data = fg;
	fg->battery = devm_power_supply_register(dev, &sm5713_fg_desc, &psy_cfg);
	if (IS_ERR(fg->battery))
		return dev_err_probe(dev, PTR_ERR(fg->battery),
				     "failed to register battery\n");

	dev_info(dev, "SM5713 fuel gauge (device ID 0x%x)\n", id);

	return 0;
}

static const struct of_device_id sm5713_fg_of_match[] = {
	{ .compatible = "siliconmitus,sm5713-fuelgauge" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sm5713_fg_of_match);

static const struct i2c_device_id sm5713_fg_i2c_id[] = {
	{ "sm5713-fuelgauge" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, sm5713_fg_i2c_id);

static struct i2c_driver sm5713_fg_driver = {
	.driver = {
		.name		= "sm5713-fuelgauge",
		.of_match_table	= sm5713_fg_of_match,
	},
	.probe		= sm5713_fg_probe,
	.id_table	= sm5713_fg_i2c_id,
};
module_i2c_driver(sm5713_fg_driver);

MODULE_AUTHOR("Bao Anh <baoanh@example.com>");
MODULE_DESCRIPTION("Silicon Mitus SM5713 fuel gauge driver");
MODULE_LICENSE("GPL");
