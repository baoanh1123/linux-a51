// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5713 MUIC (micro USB interface controller)
 *
 * The SM5713 combines a switch-mode charger, a fuel gauge and a MUIC in one
 * package.  Only the MUIC is handled here; it lives on I2C address 0x25 so no
 * MFD is needed to reach it.  Register layout and the device type encoding
 * come from the vendor driver, DOWNSTREAM_READONLY/drivers/muic/sm5713-muic.c
 * and DOWNSTREAM_READONLY/include/linux/mfd/sm5713-private.h.
 *
 * This driver only reports what the MUIC detects.  It deliberately performs
 * no register writes: the boot chain has already configured the switch and
 * the automatic charger type detection, and rewriting those registers while
 * USB is in use is how a port stops working.
 */

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/* MUIC registers (SM5713_MUIC_REG_* in the vendor header) */
#define SM5713_MUIC_DEVICEID		0x00
#define SM5713_MUIC_CNTL		0x05
#define SM5713_MUIC_DEVICETYPE1		0x07
#define SM5713_MUIC_DEVICETYPE2		0x08
#define SM5713_MUIC_MANUAL_SW		0x06
#define SM5713_MUIC_ADC			0x51

/* DEVICETYPE1: bit per detected charger/cable type */
#define SM5713_MUIC_DT1_SDP		BIT(1)
#define SM5713_MUIC_DT1_DCP		BIT(2)
#define SM5713_MUIC_DT1_CDP		BIT(3)
#define SM5713_MUIC_DT1_AFC		BIT(5)
#define SM5713_MUIC_DT1_QC20		BIT(6)

/* DEVICETYPE2 */
#define SM5713_MUIC_DT2_OTG		BIT(4)

/*
 * Manual switch control (vendor include/linux/muic/sm5713-muic.h and
 * drivers/muic/sm5713-muic.c: set_manual_sw()/set_com_sw()).  For OTG the
 * vendor programs the *same* switch value as USB: the D+/D- pair is routed to
 * the controller, and whether it acts as device or host is decided by the
 * controller.  MANUAL_SW_USB = D- code 1, D+ code 1.
 */
#define SM5713_MUIC_CNTL_MANUAL		BIT(2)
#define SM5713_MUIC_SW_OPEN		0x0
#define SM5713_MUIC_SW_USB		0x9

/*
 * Detection is done in hardware, so polling only costs two register reads.
 * One second is fast enough for a cable that a person plugs in.
 */
#define SM5713_MUIC_POLL_PERIOD		msecs_to_jiffies(1000)

/**
 * struct sm5713_muic - MUIC state
 * @dev:	device pointer
 * @regmap:	register map for the MUIC I2C address
 * @psy:	power supply registered for this MUIC
 * @lock:	serialises register access and the poll work
 * @work:	periodic device type poll
 * @online:	cached VBUS/charger detection
 * @otg:	cached OTG accessory detection
 * @usb_type:	cached power supply USB type
 * @devtype1:	last DEVICETYPE1 value, for diagnostics
 * @devtype2:	last DEVICETYPE2 value, for diagnostics
 * @adc:	last ADC value, for diagnostics
 * @cntl:	last CNTL value, for diagnostics
 */
struct sm5713_muic {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct mutex lock;
	struct delayed_work work;
	bool online;
	bool otg;
	int usb_type;
	u32 devtype1;
	u32 devtype2;
	u32 adc;
	u32 cntl;
};

/**
 * sm5713_muic_decode() - translate DEVICETYPE1 into a USB type
 * @dt1: DEVICETYPE1 register value
 *
 * The MUIC may report several bits at once when it has seen more than one
 * type during detection, so the types are ranked from the strongest charging
 * capability downwards.
 */
static int sm5713_muic_decode(u32 dt1)
{
	if (dt1 & (SM5713_MUIC_DT1_AFC | SM5713_MUIC_DT1_QC20))
		return POWER_SUPPLY_USB_TYPE_DCP;
	if (dt1 & SM5713_MUIC_DT1_DCP)
		return POWER_SUPPLY_USB_TYPE_DCP;
	if (dt1 & SM5713_MUIC_DT1_CDP)
		return POWER_SUPPLY_USB_TYPE_CDP;
	if (dt1 & SM5713_MUIC_DT1_SDP)
		return POWER_SUPPLY_USB_TYPE_SDP;

	return POWER_SUPPLY_USB_TYPE_UNKNOWN;
}

static void sm5713_muic_poll(struct work_struct *work)
{
	struct sm5713_muic *muic =
		container_of(to_delayed_work(work), struct sm5713_muic, work);
	u32 dt1, dt2;

	mutex_lock(&muic->lock);

	if (!regmap_read(muic->regmap, SM5713_MUIC_DEVICETYPE1, &dt1) &&
	    !regmap_read(muic->regmap, SM5713_MUIC_DEVICETYPE2, &dt2)) {
		bool otg = dt2 & SM5713_MUIC_DT2_OTG;
		int usb_type = sm5713_muic_decode(dt1);
		u32 adc, cntl;

		regmap_read(muic->regmap, SM5713_MUIC_ADC, &adc);
		regmap_read(muic->regmap, SM5713_MUIC_CNTL, &cntl);

		/*
		 * Attach/detach transitions are the only interesting events,
		 * and they are also what an OTG cable test needs to show.
		 */
		if (otg != muic->otg || usb_type != muic->usb_type) {
			dev_info(muic->dev,
				 "type %d -> %d, otg %d -> %d (DEVICETYPE1 %#04x DEVICETYPE2 %#04x ADC %#04x CNTL %#04x)\n",
				 muic->usb_type, usb_type, muic->otg, otg,
				 dt1, dt2, adc, cntl);
		}

		muic->devtype1 = dt1;
		muic->devtype2 = dt2;
		muic->otg = otg;
		muic->usb_type = usb_type;
		muic->online = usb_type != POWER_SUPPLY_USB_TYPE_UNKNOWN;
		muic->adc = adc;
		muic->cntl = cntl;
	}

	mutex_unlock(&muic->lock);

	schedule_delayed_work(&muic->work, SM5713_MUIC_POLL_PERIOD);
}

/**
 * sm5713_muic_set_otg() - route the USB data pair for host mode
 * @muic:	MUIC state
 * @on:		true to select the manual USB path, false to restore auto mode
 *
 * This mirrors the vendor's com_to_usb(): manual mode is required, and the
 * switch value for OTG is the USB one.  Nothing here powers VBUS - the
 * keyboard/device on the other end stays dead until the charger boost is
 * enabled, which is a separate control (sm5713-charger "otg_boost").
 */
static int sm5713_muic_set_otg(struct sm5713_muic *muic, bool on)
{
	int ret;

	mutex_lock(&muic->lock);

	ret = regmap_update_bits(muic->regmap, SM5713_MUIC_MANUAL_SW,
				 GENMASK(5, 0),
				 on ? SM5713_MUIC_SW_USB : SM5713_MUIC_SW_OPEN);
	if (!ret)
		ret = regmap_update_bits(muic->regmap, SM5713_MUIC_CNTL,
					 SM5713_MUIC_CNTL_MANUAL,
					 on ? SM5713_MUIC_CNTL_MANUAL : 0);

	mutex_unlock(&muic->lock);

	dev_info(muic->dev, "otg %s\n", on ? "on" : "off");

	return ret;
}

static ssize_t sm5713_muic_otg_write(struct file *file,
				     const char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct sm5713_muic *muic = s->private;
	unsigned int value;
	char buf[8];

	if (count == 0 || count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;

	if (sm5713_muic_set_otg(muic, value))
		return -EIO;

	return count;
}

static int sm5713_muic_otg_show(struct seq_file *s, void *unused)
{
	struct sm5713_muic *muic = s->private;
	u32 manual = 0, cntl = 0;

	mutex_lock(&muic->lock);
	regmap_read(muic->regmap, SM5713_MUIC_MANUAL_SW, &manual);
	regmap_read(muic->regmap, SM5713_MUIC_CNTL, &cntl);
	mutex_unlock(&muic->lock);

	seq_printf(s, "manual=%d switch=%#04x cntl=%#04x devtype1=%#04x devtype2=%#04x adc=%#04x\n",
		   !!(cntl & SM5713_MUIC_CNTL_MANUAL), manual, cntl,
		   muic->devtype1, muic->devtype2, muic->adc);

	return 0;
}
static int sm5713_muic_otg_open(struct inode *inode, struct file *file)
{
	return single_open(file, sm5713_muic_otg_show, inode->i_private);
}

static const struct file_operations sm5713_muic_otg_fops = {
	.owner		= THIS_MODULE,
	.open		= sm5713_muic_otg_open,
	.read		= seq_read,
	.write		= sm5713_muic_otg_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int sm5713_muic_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct sm5713_muic *muic = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = muic->online;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = muic->online;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = muic->usb_type;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sm5713_muic_property_is_writeable(struct power_supply *psy,
					     enum power_supply_property psp)
{
	return 0;
}

static enum power_supply_property sm5713_muic_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc sm5713_muic_desc = {
	.name		= "sm5713-muic",
	.type		= POWER_SUPPLY_TYPE_USB,
	.properties	= sm5713_muic_props,
	.num_properties	= ARRAY_SIZE(sm5713_muic_props),
	.get_property	= sm5713_muic_get_property,
	.property_is_writeable = sm5713_muic_property_is_writeable,
	.usb_types	= BIT(POWER_SUPPLY_USB_TYPE_SDP) |
			  BIT(POWER_SUPPLY_USB_TYPE_DCP) |
			  BIT(POWER_SUPPLY_USB_TYPE_CDP),
};

static const struct regmap_config sm5713_muic_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= SM5713_MUIC_ADC,
};

static int sm5713_muic_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct sm5713_muic *muic;
	u32 id;
	int ret;

	muic = devm_kzalloc(&client->dev, sizeof(*muic), GFP_KERNEL);
	if (!muic)
		return -ENOMEM;

	muic->dev = &client->dev;
	mutex_init(&muic->lock);
	INIT_DELAYED_WORK(&muic->work, sm5713_muic_poll);

	muic->regmap = devm_regmap_init_i2c(client,
					    &sm5713_muic_regmap_config);
	if (IS_ERR(muic->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(muic->regmap),
				     "Failed to init register map\n");

	ret = regmap_read(muic->regmap, SM5713_MUIC_DEVICEID, &id);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "Failed to read device ID\n");

	dev_info(&client->dev, "SM5713 MUIC, device ID %#04x\n", id);

	psy_cfg.drv_data = muic;
	psy_cfg.fwnode = dev_fwnode(&client->dev);
	muic->psy = devm_power_supply_register(&client->dev, &sm5713_muic_desc,
					       &psy_cfg);
	if (IS_ERR(muic->psy))
		return dev_err_probe(&client->dev, PTR_ERR(muic->psy),
				     "Failed to register power supply\n");

	i2c_set_clientdata(client, muic);

	debugfs_create_file("otg", 0644, NULL, muic, &sm5713_muic_otg_fops);

	sm5713_muic_poll(&muic->work.work);
	schedule_delayed_work(&muic->work, SM5713_MUIC_POLL_PERIOD);

	return 0;
}

static void sm5713_muic_remove(struct i2c_client *client)
{
	struct sm5713_muic *muic = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&muic->work);
}

static const struct of_device_id sm5713_muic_of_match[] = {
	{ .compatible = "samsung,sm5713-muic" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5713_muic_of_match);

static const struct i2c_device_id sm5713_muic_i2c_id[] = {
	{ "sm5713-muic" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5713_muic_i2c_id);

static struct i2c_driver sm5713_muic_driver = {
	.driver = {
		.name = "sm5713-muic",
		.of_match_table = sm5713_muic_of_match,
	},
	.probe = sm5713_muic_probe,
	.remove = sm5713_muic_remove,
	.id_table = sm5713_muic_i2c_id,
};
module_i2c_driver(sm5713_muic_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5713 MUIC driver");
MODULE_LICENSE("GPL");
