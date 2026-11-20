// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5713 switch-mode charger
 *
 * The SM5713 combines a switch-mode charger, a fuel gauge and a MUIC in one
 * package.  Only the charger is handled here; it lives on I2C address 0x49 so
 * no MFD is needed to reach it.  Register layout and the status encoding come
 * from the vendor driver, DOWNSTREAM_READONLY/drivers/battery_v2/
 * sm5713_charger.c and DOWNSTREAM_READONLY/include/linux/mfd/
 * sm5713-private.h.
 *
 * The boot chain leaves the charger programmed and enabled, but the charger
 * has a 90 second hardware watchdog that stops charging when it lapses.  On
 * this board the watchdog has always expired by the time Linux boots, which
 * shows up as bit 7 of STATUS2 being set and bit 3 (charging) being clear -
 * the pack simply never charges.  The vendor driver re-arms the watchdog on
 * every work cycle while charging, which is what this driver does too.
 *
 * Safe limits are deliberately left alone: the float voltage and the over
 * current protection are the values the boot chain programmed, and the input
 * current limit is not touched at all until cable type detection exists.
 */

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/* Charger registers (SM5713_CHG_REG_* in the vendor header) */
#define SM5713_CHG_STATUS1	0x0d
#define SM5713_CHG_STATUS2	0x0e
#define SM5713_CHG_STATUS3	0x0f
#define SM5713_CHG_CNTL1	0x13
#define SM5713_CHG_CNTL2	0x14
#define SM5713_CHG_CHGCNTL2	0x18
#define SM5713_CHG_CHGCNTL4	0x1a
#define SM5713_CHG_VBUSCNTL	0x15
#define SM5713_CHG_WDTCNTL	0x22
#define SM5713_CHG_BSTCNTL1	0x23
#define SM5713_CHG_FACTORY1	0x25
#define SM5713_CHG_DEVICEID	0x50

/* STATUS1 */
#define SM5713_CHG_STATUS1_VBUS_POK	BIT(0)
#define SM5713_CHG_STATUS1_VBUS_UV	BIT(1)
#define SM5713_CHG_STATUS1_VBUS_OV	BIT(2)

/* STATUS2 */
#define SM5713_CHG_STATUS2_NO_VBUS	BIT(2)
#define SM5713_CHG_STATUS2_CHARGING	BIT(3)
#define SM5713_CHG_STATUS2_TOPOFF	BIT(5)
#define SM5713_CHG_STATUS2_WDT_EXPIRED	BIT(7)

/* CNTL1 */
#define SM5713_CHG_CNTL1_ENQ4FET	BIT(3)

/* WDTCNTL: writing the reset bit re-arms the charger watchdog; self-clearing */
#define SM5713_CHG_WDTCNTL_RESET	BIT(6)
/* WDTCNTL[0] enables the watchdog; bits [2:1] select its timeout */
#define SM5713_CHG_WDTCNTL_EN		BIT(0)

/*
 * VBUSCNTL[6:0] sets the input current limit in 25 mA steps above 100 mA;
 * FACTORY1 bit 0 makes the part ignore the limit, so a part left in factory
 * mode must not be re-programmed.
 */
#define SM5713_CHG_VBUSCNTL_MASK	GENMASK(6, 0)
#define SM5713_CHG_FACTORY1_IN_FACTORY	BIT(0)

/* Input current limits per source, in mA (VCUSCNTL resolution is 25 mA) */
/*
 * Fast-charge (constant current) is CHGCNTL2[6:0] in 50 mA steps from
 * 100 mA; the battery regulation voltage is CHGCNTL4[5:0] (3.70-3.85 V in
 * 50 mV, 3.90/4.00 V then 4.05-4.62 V in 10 mV steps).  Encodings follow the
 * vendor driver (drivers/battery_v2/sm5713_charger.c: chg_set_fast_charging_
 * current and chg_set_batreg).
 */
#define SM5713_CHG_CHGCNTL2_MASK	GENMASK(6, 0)
#define SM5713_CHG_CHGCNTL4_VMASK	GENMASK(5, 0)
#define SM5713_CHG_CC_MIN_MA		100
#define SM5713_CHG_CC_STEP_MA		50
/* Conservative ceiling: the part allows 3500 mA, but the A51 does not. */
#define SM5713_CHG_CC_MAX_MA		2000

#define SM5713_CHG_LIMIT_SDP		500
#define SM5713_CHG_LIMIT_CDP		1500
#define SM5713_CHG_LIMIT_DCP		1800

/*
 * The watchdog is 90 s, so 30 s leaves room for scheduling delays while still
 * being far cheaper than the fuel gauge poll.
 */
#define SM5713_CHG_WDT_PERIOD		msecs_to_jiffies(30000)

/*
 * OTG (host mode) power: CNTL2[3:0] selects the operation mode and BSTCNTL1
 * sets the boost output and current limit.  Values from the vendor
 * (drivers/battery_v2/sm5713_charger_oper.c): CNTL2 mode 0x7 = USB OTG and
 * 0x5 = charging on VBUS, BSTCNTL1 output 0x6 = 5.1 V and bits [7:6] = 0x1
 * for a 900 mA OTG current limit.
 */
#define SM5713_CHG_CNTL2_MODE_MASK	GENMASK(3, 0)
#define SM5713_CHG_OP_MODE_CHG_VBUS	0x5
#define SM5713_CHG_OP_MODE_USB_OTG	0x7
#define SM5713_CHG_BSTOUT_5100MV	0x6
#define SM5713_CHG_OTG_CURRENT_900MA	(0x1 << 6)

/* Cable type comes from the MUIC's own 1 s poll, so matching it is enough */
#define SM5713_CHG_CABLE_PERIOD		msecs_to_jiffies(2000)

/**
 * struct sm5713_charger - charger state
 * @dev:	device pointer
 * @regmap:	register map for the charger I2C address
 * @psy:	power supply registered for this charger
 * @lock:	serialises register access and the watchdog work
 * @wdt_work:	periodic watchdog re-arm
 * @cable_work:	periodic cable type poll
 * @limit_work:	periodic charge-limit enforcement
 * @muic:	MUIC power supply, used to detect the cable type
 * @input_limit_ma: input current limit currently requested
 * @input_limit_user: the user set the input limit, so the cable poll must
 *		not overwrite it from the MUIC cable type
 * @end_threshold: battery level (percent) at which charging must stop, 0 = off
 * @limit_active: charging is currently inhibited by @end_threshold
 * @vbus:	cached VBUS presence
 * @charging:	cached charging state
 * @health:	cached health
 * @wdt_expired: cached watchdog expiry flag
 */
struct sm5713_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct mutex lock;
	struct delayed_work wdt_work;
	struct delayed_work cable_work;
	struct delayed_work limit_work;
	struct power_supply *muic;
	unsigned int input_limit_ma;
	bool input_limit_user;
	unsigned int end_threshold;
	bool limit_active;
	bool vbus;
	bool charging;
	int health;
	bool wdt_expired;
	bool suspended;
	bool wdtcntl_saved_valid;
	u32 wdtcntl_saved;
};

static int sm5713_chg_read(struct sm5713_charger *chg, u8 reg, u32 *val)
{
	return regmap_read(chg->regmap, reg, val);
}

static int sm5713_chg_update_bits(struct sm5713_charger *chg, u8 reg,
				  u32 mask, u32 val)
{
	return regmap_update_bits(chg->regmap, reg, mask, val);
}

/**
 * sm5713_chg_refresh() - latch the status registers
 * @chg: charger state
 *
 * Returns 0 on success, or a negative error code if the charger did not
 * answer.
 */
static int sm5713_chg_refresh(struct sm5713_charger *chg)
{
	u32 st1, st2;

	if (sm5713_chg_read(chg, SM5713_CHG_STATUS1, &st1))
		return -EIO;
	if (sm5713_chg_read(chg, SM5713_CHG_STATUS2, &st2))
		return -EIO;

	chg->vbus = st1 & SM5713_CHG_STATUS1_VBUS_POK;
	chg->charging = st2 & SM5713_CHG_STATUS2_CHARGING;
	chg->wdt_expired = st2 & SM5713_CHG_STATUS2_WDT_EXPIRED;

	if (!(st1 & SM5713_CHG_STATUS1_VBUS_POK))
		chg->health = POWER_SUPPLY_HEALTH_GOOD;
	else if (st1 & SM5713_CHG_STATUS1_VBUS_OV)
		chg->health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if (st1 & SM5713_CHG_STATUS1_VBUS_UV)
		chg->health = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
	else
		chg->health = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

/* Re-enable charging a few percent below the configured threshold. */
#define SM5713_CHG_LIMIT_HYSTERESIS	3

/**
 * __sm5713_chg_update_limit() - decide whether the charge limit inhibits charging
 * @chg:	charger state, @chg->lock held
 *
 * The level comes from the battery power supply, so this only flips state; the
 * caller applies it to the charger registers.
 */
static void __sm5713_chg_update_limit(struct sm5713_charger *chg)
{
	union power_supply_propval val;
	struct power_supply *bat;
	int ret;

	if (!chg->end_threshold) {
		chg->limit_active = false;
		return;
	}

	bat = power_supply_get_by_name("battery");
	if (!bat)
		return;

	ret = power_supply_get_property(bat, POWER_SUPPLY_PROP_CAPACITY, &val);
	power_supply_put(bat);
	if (ret)
		return;

	if (!chg->limit_active && val.intval >= chg->end_threshold)
		chg->limit_active = true;
	else if (chg->limit_active &&
		 val.intval <= (int)chg->end_threshold -
			       SM5713_CHG_LIMIT_HYSTERESIS)
		chg->limit_active = false;
}

/**
 * sm5713_chg_limit_work() - periodically enforce the charge limit
 * @work:	delayed work
 */
static void sm5713_chg_limit_work(struct work_struct *work)
{
	struct sm5713_charger *chg =
		container_of(to_delayed_work(work), struct sm5713_charger,
			     limit_work);

	mutex_lock(&chg->lock);
	__sm5713_chg_update_limit(chg);
	mutex_unlock(&chg->lock);

	if (!chg->suspended)
		schedule_delayed_work(&chg->limit_work,
				      SM5713_CHG_CABLE_PERIOD);
}

/**
 * sm5713_chg_kick_watchdog() - keep a plugged charger running
 * @chg: charger state
 *
 * Charging stops once the charger watchdog lapses, so re-arm it whenever VBUS
 * is present.  The charge path itself is enabled here too, because a charger
 * that was never enabled will not start just because the watchdog is fed.
 */
static void sm5713_chg_kick_watchdog(struct sm5713_charger *chg)
{
	mutex_lock(&chg->lock);

	if (sm5713_chg_refresh(chg))
		goto out;

	__sm5713_chg_update_limit(chg);

	if (chg->vbus && !chg->limit_active) {
		sm5713_chg_update_bits(chg, SM5713_CHG_CNTL1,
				       SM5713_CHG_CNTL1_ENQ4FET,
				       SM5713_CHG_CNTL1_ENQ4FET);
		sm5713_chg_update_bits(chg, SM5713_CHG_WDTCNTL,
				       SM5713_CHG_WDTCNTL_RESET,
				       SM5713_CHG_WDTCNTL_RESET);
	}

out:
	mutex_unlock(&chg->lock);
}

/**
 * sm5713_chg_set_cc_current() - program the constant charge current
 * @chg:	charger state
 * @mA:		requested current, clamped to the safe range
 *
 * Return: 0 on success, -errno otherwise.
 */
static int sm5713_chg_set_cc_current(struct sm5713_charger *chg, unsigned int mA)
{
	u32 factory;
	u32 code;

	mA = clamp(mA, SM5713_CHG_CC_MIN_MA, SM5713_CHG_CC_MAX_MA);
	code = (mA - SM5713_CHG_CC_MIN_MA) / SM5713_CHG_CC_STEP_MA;

	if (sm5713_chg_read(chg, SM5713_CHG_FACTORY1, &factory))
		return -EIO;
	if (factory & SM5713_CHG_FACTORY1_IN_FACTORY)
		return -EOPNOTSUPP;

	sm5713_chg_update_bits(chg, SM5713_CHG_CHGCNTL2,
			       SM5713_CHG_CHGCNTL2_MASK,
			       code & SM5713_CHG_CHGCNTL2_MASK);

	dev_info(chg->dev, "constant charge current %u mA (CHGCNTL2 %#04x)\n",
		 SM5713_CHG_CC_MIN_MA + code * SM5713_CHG_CC_STEP_MA, code);

	return 0;
}

/**
 * sm5713_chg_get_cc_current() - read back the constant charge current
 * @chg:	charger state
 * @mA:		returned current in mA
 */
static int sm5713_chg_get_cc_current(struct sm5713_charger *chg, int *mA)
{
	u32 reg;

	if (sm5713_chg_read(chg, SM5713_CHG_CHGCNTL2, &reg))
		return -EIO;

	*mA = SM5713_CHG_CC_MIN_MA +
	      (reg & SM5713_CHG_CHGCNTL2_MASK) * SM5713_CHG_CC_STEP_MA;

	return 0;
}

/**
 * sm5713_chg_get_cc_voltage() - read back the battery regulation voltage
 * @chg:	charger state
 * @uV:		returned voltage in microvolts
 */
static int sm5713_chg_get_cc_voltage(struct sm5713_charger *chg, int *uV)
{
	unsigned int off, mV;
	u32 reg;

	if (sm5713_chg_read(chg, SM5713_CHG_CHGCNTL4, &reg))
		return -EIO;

	off = reg & SM5713_CHG_CHGCNTL4_VMASK;
	if (off <= 3)
		mV = 3700 + 50 * off;			/* 3.70 - 3.85 V */
	else if (off <= 5)
		mV = 3900 + 100 * (off - 4);		/* 3.90 / 4.00 V */
	else
		mV = 4050 + 10 * (off - 6);		/* 4.05 - 4.62 V */

	*uV = mV * 1000;

	return 0;
}

/**
 * sm5713_chg_apply_input_limit() - program the input current limit
 * @chg: charger state
 * @limit: limit in mA
 *
 * Writes the limit and caches it.  A part left in factory mode ignores the
 * register, so leave it alone there rather than pretending the limit took.
 */
static int sm5713_chg_apply_input_limit(struct sm5713_charger *chg,
				       unsigned int limit)
{
	u32 factory, vbuscntl;
	unsigned int code;

	if (limit < 100)
		code = 0;
	else
		code = ((limit - 100) / 25) & SM5713_CHG_VBUSCNTL_MASK;

	if (sm5713_chg_read(chg, SM5713_CHG_FACTORY1, &factory))
		return -EIO;

	if (factory & SM5713_CHG_FACTORY1_IN_FACTORY)
		return -EOPNOTSUPP;

	if (sm5713_chg_read(chg, SM5713_CHG_VBUSCNTL, &vbuscntl))
		return -EIO;

	if ((vbuscntl & SM5713_CHG_VBUSCNTL_MASK) == code) {
		chg->input_limit_ma = limit;
		return 0;
	}

	sm5713_chg_update_bits(chg, SM5713_CHG_VBUSCNTL,
			       SM5713_CHG_VBUSCNTL_MASK, code);
	chg->input_limit_ma = limit;

	dev_info(chg->dev, "input current limit %u mA (VBUSCNTL %#04x)\n",
		 limit, code);

	return 0;
}

/**
 * sm5713_chg_cable_work() - follow the cable type detected by the MUIC
 * @work: work item
 *
 * A standard downstream port (a PC) may only supply 500 mA, a charging
 * downstream port 1.5 A and a dedicated charger 1.8 A.  Programming one
 * conservative limit for every source either draws too much from a PC or
 * leaves a real charger unused.
 */
static void sm5713_chg_cable_work(struct work_struct *work)
{
	struct sm5713_charger *chg =
		container_of(to_delayed_work(work), struct sm5713_charger,
			     cable_work);
	union power_supply_propval val;
	unsigned int limit;

	mutex_lock(&chg->lock);

	if (!chg->muic)
		chg->muic = power_supply_get_by_name("sm5713-muic");

	if (chg->muic) {
		if (power_supply_get_property(chg->muic,
					      POWER_SUPPLY_PROP_USB_TYPE, &val)) {
			limit = SM5713_CHG_LIMIT_SDP;
		} else {
			switch (val.intval) {
			case POWER_SUPPLY_USB_TYPE_DCP:
				limit = SM5713_CHG_LIMIT_DCP;
				break;
			case POWER_SUPPLY_USB_TYPE_CDP:
				limit = SM5713_CHG_LIMIT_CDP;
				break;
			default:
				limit = SM5713_CHG_LIMIT_SDP;
				break;
			}
		}

		if (!chg->input_limit_user)
			sm5713_chg_apply_input_limit(chg, limit);
	}

	mutex_unlock(&chg->lock);

	if (chg->suspended)
		return;

	schedule_delayed_work(&chg->cable_work, SM5713_CHG_CABLE_PERIOD);
}

static void sm5713_chg_wdt_work(struct work_struct *work)
{
	struct sm5713_charger *chg =
		container_of(to_delayed_work(work), struct sm5713_charger,
			     wdt_work);

	/* Resume restarts this; the I2C bus is gone while suspended. */
	if (chg->suspended)
		return;

	sm5713_chg_kick_watchdog(chg);
	sm5713_chg_cable_work(&chg->cable_work.work);
	schedule_delayed_work(&chg->wdt_work, SM5713_CHG_WDT_PERIOD);
}

/**
 * sm5713_chg_set_otg_boost() - output 5 V on VBUS for a host-mode accessory
 * @chg:	charger state
 * @on:		true to enter USB OTG mode, false to return to charging
 *
 * The keyboard, stick or hub on the other end of an OTG cable is powered from
 * this boost, so without it the accessory stays completely dead.  Charging and
 * boosting are mutually exclusive, hence the operation mode switch.
 */
static int sm5713_chg_set_otg_boost(struct sm5713_charger *chg, bool on)
{
	int ret;

	mutex_lock(&chg->lock);

	if (on) {
		ret = sm5713_chg_update_bits(chg, SM5713_CHG_BSTCNTL1,
					     GENMASK(7, 0),
					     SM5713_CHG_BSTOUT_5100MV |
					     SM5713_CHG_OTG_CURRENT_900MA);
		if (!ret)
			ret = sm5713_chg_update_bits(chg,
						     SM5713_CHG_CNTL2,
						     SM5713_CHG_CNTL2_MODE_MASK,
						     SM5713_CHG_OP_MODE_USB_OTG);
	} else {
		ret = sm5713_chg_update_bits(chg, SM5713_CHG_CNTL2,
					     SM5713_CHG_CNTL2_MODE_MASK,
					     SM5713_CHG_OP_MODE_CHG_VBUS);
	}

	mutex_unlock(&chg->lock);

	if (!ret && !on)
		sm5713_chg_kick_watchdog(chg);

	dev_info(chg->dev, "otg boost %s\n", on ? "on (5.1 V)" : "off");

	return ret;
}

static ssize_t sm5713_chg_otg_write(struct file *file,
				    const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct sm5713_charger *chg = s->private;
	unsigned int value;
	char buf[8];

	if (count == 0 || count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;

	if (sm5713_chg_set_otg_boost(chg, value))
		return -EIO;

	return count;
}

static int sm5713_chg_otg_show(struct seq_file *s, void *unused)
{
	struct sm5713_charger *chg = s->private;
	u32 cntl2 = 0, bst = 0;

	mutex_lock(&chg->lock);
	sm5713_chg_read(chg, SM5713_CHG_CNTL2, &cntl2);
	sm5713_chg_read(chg, SM5713_CHG_BSTCNTL1, &bst);
	mutex_unlock(&chg->lock);

	seq_printf(s, "cntl2=%#04x (mode %#x) bstcntl1=%#04x vbus=%d\n",
		   cntl2, (unsigned int)(cntl2 & SM5713_CHG_CNTL2_MODE_MASK),
		   bst, chg->vbus);

	return 0;
}
static int sm5713_chg_otg_open(struct inode *inode, struct file *file)
{
	return single_open(file, sm5713_chg_otg_show, inode->i_private);
}

static const struct file_operations sm5713_chg_otg_fops = {
	.owner		= THIS_MODULE,
	.open		= sm5713_chg_otg_open,
	.read		= seq_read,
	.write		= sm5713_chg_otg_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int sm5713_chg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct sm5713_charger *chg = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&chg->lock);
	if (sm5713_chg_refresh(chg)) {
		ret = -EIO;
		goto out;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (chg->charging)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (chg->vbus)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chg->vbus;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chg->vbus;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chg->health;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = chg->charging ?
			POWER_SUPPLY_CHARGE_TYPE_FAST :
			POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = chg->input_limit_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sm5713_chg_get_cc_current(chg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = sm5713_chg_get_cc_voltage(chg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		val->intval = chg->end_threshold;
		break;
	default:
		ret = -EINVAL;
		break;
	}

out:
	mutex_unlock(&chg->lock);
	return ret;
}

static int sm5713_chg_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct sm5713_charger *chg = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&chg->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sm5713_chg_apply_input_limit(chg,
						   val->intval / 1000);
		if (!ret)
			chg->input_limit_user = true;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sm5713_chg_set_cc_current(chg, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		if (val->intval < 0 || val->intval > 100) {
			ret = -ERANGE;
			break;
		}
		chg->end_threshold = val->intval;
		__sm5713_chg_update_limit(chg);
		/* Apply the new decision now rather than at the next kick. */
		if (chg->vbus) {
			sm5713_chg_update_bits(chg, SM5713_CHG_CNTL1,
					       SM5713_CHG_CNTL1_ENQ4FET,
					       chg->limit_active ? 0 :
					       SM5713_CHG_CNTL1_ENQ4FET);
		}
		dev_info(chg->dev, "charge limit %u%% (%s)\n",
			 chg->end_threshold,
			 chg->limit_active ? "inhibit" : "allow");
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&chg->lock);

	return ret;
}

static int sm5713_chg_property_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property sm5713_chg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
};

static const struct power_supply_desc sm5713_chg_desc = {
	.name		= "sm5713-charger",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.properties	= sm5713_chg_props,
	.num_properties	= ARRAY_SIZE(sm5713_chg_props),
	.get_property	= sm5713_chg_get_property,
	.set_property	= sm5713_chg_set_property,
	.property_is_writeable = sm5713_chg_property_is_writeable,
};

static const struct regmap_config sm5713_chg_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= SM5713_CHG_DEVICEID,
};

static int sm5713_chg_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct sm5713_charger *chg;
	u32 id;
	int ret;

	chg = devm_kzalloc(&client->dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = &client->dev;
	mutex_init(&chg->lock);
	INIT_DELAYED_WORK(&chg->wdt_work, sm5713_chg_wdt_work);
	INIT_DELAYED_WORK(&chg->limit_work, sm5713_chg_limit_work);
	INIT_DELAYED_WORK(&chg->cable_work, sm5713_chg_cable_work);

	chg->regmap = devm_regmap_init_i2c(client, &sm5713_chg_regmap_config);
	if (IS_ERR(chg->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(chg->regmap),
				     "Failed to init register map\n");

	ret = sm5713_chg_read(chg, SM5713_CHG_DEVICEID, &id);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "Failed to read device ID\n");

	dev_info(&client->dev, "SM5713 charger, device ID %#04x\n", id);

	psy_cfg.drv_data = chg;
	psy_cfg.fwnode = dev_fwnode(&client->dev);
	chg->psy = devm_power_supply_register(&client->dev, &sm5713_chg_desc,
					      &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(&client->dev, PTR_ERR(chg->psy),
				     "Failed to register power supply\n");

	i2c_set_clientdata(client, chg);

	debugfs_create_file("otg_boost", 0644, NULL, chg,
			    &sm5713_chg_otg_fops);

	sm5713_chg_kick_watchdog(chg);
	sm5713_chg_cable_work(&chg->cable_work.work);
	schedule_delayed_work(&chg->wdt_work, SM5713_CHG_WDT_PERIOD);
	if (!chg->suspended)
		schedule_delayed_work(&chg->limit_work,
				      SM5713_CHG_CABLE_PERIOD);

	return 0;
}

/*
 * System sleep.  In s2idle the kernel keeps running, so these delayed works keep
 * firing - but the I2C adapter has been suspended and every transfer warns and
 * fails.  Worse for charging than for the log: the charger's own watchdog stops
 * charging if it is not re-armed (WDTCNTL[6]) and the vendor programs it for
 * 90 s, so a suspend longer than that would silently end charging.
 *
 * The bus cannot be used, so the watchdog is switched off for the duration
 * instead (WDTCNTL[0]; the vendor's WDTCNTL layout is timer index in [2:1],
 * enable in bit 0, kick in bit 6).  The value found at suspend is put back on
 * resume, so whatever the boot chain or another agent programmed survives, and
 * the first kick after resume re-arms the timer anyway.
 */
static int sm5713_chg_suspend(struct device *dev)
{
	struct sm5713_charger *chg = dev_get_drvdata(dev);
	u32 val;

	chg->suspended = true;
	cancel_delayed_work_sync(&chg->wdt_work);
	cancel_delayed_work_sync(&chg->cable_work);
	cancel_delayed_work_sync(&chg->limit_work);

	mutex_lock(&chg->lock);
	if (!sm5713_chg_read(chg, SM5713_CHG_WDTCNTL, &val)) {
		chg->wdtcntl_saved = val;
		chg->wdtcntl_saved_valid = true;
		sm5713_chg_update_bits(chg, SM5713_CHG_WDTCNTL,
				       SM5713_CHG_WDTCNTL_EN, 0);
	}
	mutex_unlock(&chg->lock);

	return 0;
}

static int sm5713_chg_resume(struct device *dev)
{
	struct sm5713_charger *chg = dev_get_drvdata(dev);

	chg->suspended = false;

	mutex_lock(&chg->lock);
	if (chg->wdtcntl_saved_valid) {
		sm5713_chg_update_bits(chg, SM5713_CHG_WDTCNTL,
				       SM5713_CHG_WDTCNTL_EN,
				       chg->wdtcntl_saved &
				       SM5713_CHG_WDTCNTL_EN);
		chg->wdtcntl_saved_valid = false;
	}
	mutex_unlock(&chg->lock);

	schedule_delayed_work(&chg->wdt_work, SM5713_CHG_WDT_PERIOD);
	schedule_delayed_work(&chg->limit_work, SM5713_CHG_CABLE_PERIOD);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sm5713_chg_pm_ops, sm5713_chg_suspend,
				sm5713_chg_resume);

static void sm5713_chg_remove(struct i2c_client *client)
{
	struct sm5713_charger *chg = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chg->cable_work);
	cancel_delayed_work_sync(&chg->wdt_work);
	cancel_delayed_work_sync(&chg->limit_work);

	if (chg->muic)
		power_supply_put(chg->muic);
}

static const struct of_device_id sm5713_chg_of_match[] = {
	{ .compatible = "samsung,sm5713-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5713_chg_of_match);

static const struct i2c_device_id sm5713_chg_i2c_id[] = {
	{ "sm5713-charger" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5713_chg_i2c_id);

static struct i2c_driver sm5713_chg_driver = {
	.driver = {
		.name = "sm5713-charger",
		.of_match_table = sm5713_chg_of_match,
		.pm = pm_ptr(&sm5713_chg_pm_ops),
	},
	.probe = sm5713_chg_probe,
	.remove = sm5713_chg_remove,
	.id_table = sm5713_chg_i2c_id,
};
module_i2c_driver(sm5713_chg_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5713 charger driver");
MODULE_LICENSE("GPL");
