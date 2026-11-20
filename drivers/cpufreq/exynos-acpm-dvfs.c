// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos ACPM DVFS client
 *
 * On Exynos9610 the CPU/GPU clocks and their PMIC voltages are owned by the
 * ACPM firmware running on the Cortex-M23.  Linux does not program the CMU or
 * the PMIC rails for DVFS at all; it asks the firmware over an ACPM mailbox
 * channel instead.
 *
 * The message format (see DOWNSTREAM_READONLY/drivers/soc/samsung/cal-if/
 * acpm_dvfs.c) is four 32-bit words:
 *
 *	word 0: DVFS domain id
 *	word 1: rate in Hz (the firmware writes the current rate back here)
 *	word 2: command (0 = FREQ_REQ, 1 = FREQ_GET, ...)
 *	word 3: argument, unused for the commands above
 *
 * This driver implements the transport and the two rate commands.  It
 * deliberately does not register a cpufreq policy yet: first the domain ids
 * and the set of rates the firmware accepts have to be established on
 * hardware, which is what the debugfs interface below is for.
 */

#include <linux/debugfs.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* DVFS commands */
#define ACPM_DVFS_FREQ_REQ	0
#define ACPM_DVFS_FREQ_GET	1
#define ACPM_DVFS_SET_INIT_FREQ	3
#define ACPM_DVFS_DATA_INIT	5

#define ACPM_DVFS_CPUCL0	2	/* LITTLE, Cortex-A53 */
#define ACPM_DVFS_CPUCL1	3	/* big, Cortex-A73 */

/*
 * Candidate DVFS domain ids.  The vendor CAL exposes dvfs_cpucl0/dvfs_cpucl1
 * as 0x0b040002/0x0b040003 (see /sys/kernel/debug/vclk/dvfs_domain on the
 * vendor kernel) and GET_IDX() masks with 0xffff, so the ACPM payload may
 * carry either the masked value or the bare index.  Probe both, read-only.
 */
static const u32 acpm_dvfs_probe_ids[] = {
	0x4002, 0x4003,		/* masking 0x0b040002/3 with MASK_OF_ID */
	2, 3,			/* bare index with cpucl0/cpucl1 last */
};

struct acpm_dvfs {
	struct device *dev;
	struct acpm_handle *acpm;
	unsigned int chan;
	struct dentry *debugfs;
	enum acpm_front_mode front_mode;
};

/*
 * The ACPM shared memory window is only partly mapped for the AP: the
 * framework/plugin code below offset 0x5970 raises a synchronous external
 * abort when read from the non-secure world, and repeated aborts escalate to an
 * SError panic.  Only the channel table (0x5970..0x8000) and the top FV map
 * window (0x12400..0x15000) are safe to read, so this driver never touches the
 * window; the file dump in the ACPM core driver is limited to those regions.
 */

static int acpm_dvfs_send(struct acpm_dvfs *dvfs, unsigned int id, u32 cmd,
			  unsigned long rate, u32 arg, u32 *result)
{
	u32 msg[4] = { id, rate, cmd, arg };
	int ret;

	ret = acpm_do_cmd(dvfs->acpm, dvfs->chan, msg, ARRAY_SIZE(msg));
	if (ret)
		return ret;

	if (result)
		*result = msg[1];

	return 0;
}

/**
 * acpm_dvfs_send_nowait() - post a DVFS command without waiting for a reply
 * @dvfs: driver data
 * @msg: four words as the firmware expects them
 *
 * DVFS requests are fire-and-forget on this firmware: FREQ_REQ is answered by
 * changing the rate, not by writing a response word, so the vendor kernel
 * never waits either (which is why its mailbox interrupt count stays at zero).
 */
static int acpm_dvfs_send_nowait(struct acpm_dvfs *dvfs, u32 *msg)
{
	struct acpm_xfer xfer = {
		.txd = msg,
		.rxd = msg,
		.txlen = 4 * sizeof(u32),
		.rxlen = 4 * sizeof(u32),
		.acpm_chan_id = dvfs->chan,
		.no_wait = true,
		.front_mode = dvfs->front_mode,
	};

	return acpm_do_xfer(dvfs->acpm, &xfer);
}

/**
 * acpm_dvfs_get_rate() - ask the firmware for the current rate of a domain
 * @dvfs: driver data
 * @id: DVFS domain id
 * @rate: returns the current rate in Hz
 */
static int acpm_dvfs_get_rate(struct acpm_dvfs *dvfs, unsigned int id,
			      unsigned long *rate)
{
	u32 value;
	int ret;

	ret = acpm_dvfs_send(dvfs, id, ACPM_DVFS_FREQ_GET, 0, 0, &value);
	if (ret)
		return ret;

	*rate = value;

	return 0;
}

/*
 * Stock Exynos 9611 limits: Cortex-A73 up to 2.3 GHz, Cortex-A53 up to
 * 1.7 GHz.  The vendor kernel on this board is an overclocked build offering
 * 2.6 GHz / 2.054 GHz, and exceeding the stock numbers would run the SoC
 * outside its specification, so requests are clamped here.
 *
 * cpucl0 is the LITTLE cluster and cpucl1 the big one: the vendor's
 * acpm_dvfs_exynos9610.h pairs dvfs_cpucl0 with MARGIN_LIT and dvfs_cpucl1
 * with MARGIN_BIG, and dvfs_domain lists cpucl0 as 0xb040002 with base
 * ACPM_VCLK_TYPE, i.e. GET_IDX() gives 2 for LITTLE and 3 for the A73.
 */
#define ACPM_DVFS_CPUCL1_MAX	2300000UL	/* Cortex-A73 */
#define ACPM_DVFS_CPUCL0_MAX	1700000UL	/* Cortex-A53 */

/**
 * acpm_dvfs_set_rate() - request a new rate for a domain
 * @dvfs: driver data
 * @id: DVFS domain id
 * @rate: requested rate in Hz
 *
 * The firmware clamps the request to the rates it supports and rejects
 * requests for domains it does not manage.
 */
static int acpm_dvfs_set_rate(struct acpm_dvfs *dvfs, unsigned int id,
			      unsigned long rate)
{
	u32 max, msg[4];

	switch (id) {
	case 2:
	case 0x4002:
		max = ACPM_DVFS_CPUCL0_MAX;
		break;
	case 3:
	case 0x4003:
		max = ACPM_DVFS_CPUCL1_MAX;
		break;
	default:
		return -EINVAL;
	}

	if (rate > max)
		return -ERANGE;

	msg[0] = id;
	msg[1] = rate;
	msg[2] = ACPM_DVFS_FREQ_REQ;
	msg[3] = 0;

	return acpm_dvfs_send_nowait(dvfs, msg);
}

static int acpm_dvfs_dump_show(struct seq_file *s, void *unused)
{
	struct acpm_dvfs *dvfs = s->private;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(acpm_dvfs_probe_ids); i++) {
		unsigned long rate;
		u32 id = acpm_dvfs_probe_ids[i];
		int ret;

		ret = acpm_dvfs_get_rate(dvfs, id, &rate);
		if (ret)
			seq_printf(s, "id 0x%04x: error %d\n", id, ret);
		else
			seq_printf(s, "id 0x%04x: %lu Hz\n", id, rate);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(acpm_dvfs_dump);

/*
 * debugfs "set" interface: writing "<id> <rate>" sends a single FREQ_REQ.
 * -ETIME is reported as "sent" because the firmware acknowledges DVFS
 * requests by applying them rather than by writing a response word, which is
 * what the zero mailbox interrupt count on the vendor kernel shows too.
 */
static ssize_t acpm_dvfs_set_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct acpm_dvfs *dvfs = file->private_data;
	unsigned int id, cmd = ACPM_DVFS_FREQ_REQ, arg = 0;
	unsigned long rate;
	u32 msg[4];
	char buf[40];
	int ret;

	if (count == 0 || count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';
	if (sscanf(buf, "%u %lu %u %u", &id, &rate, &cmd, &arg) < 2)
		return -EINVAL;

	/*
	 * Send exactly the four words the vendor sends: { id, rate, cmd, arg }.
	 * SET_INIT_FREQ needs a non-zero arg (DATA_INIT), which the earlier
	 * two-message approximation could never produce.
	 */
	if (cmd == ACPM_DVFS_FREQ_REQ && !arg) {
		/* Plain rate request: clamped per domain, posted without waiting. */
		ret = acpm_dvfs_set_rate(dvfs, id, rate);
	} else {
		msg[0] = id;
		msg[1] = rate;
		msg[2] = cmd;
		msg[3] = arg;
		ret = acpm_dvfs_send_nowait(dvfs, msg);
	}
	if (ret)
		return ret;

	dev_info(dvfs->dev, "posted cmd %u id %#x rate %lu arg %u\n", cmd, id,
		 rate, arg);
	*ppos += count;

	return count;
}

/*
 * "mode" selects how a single-slot buffer channel advertises a new message.
 * 0 publishes rear + 1 (our own scheme), 1 replicates the vendor driver, which
 * leaves front at zero and relies on the doorbell.  Runtime selectable so both
 * can be tested against the same firmware without another flash.
 */
static int acpm_dvfs_mode_show(struct seq_file *s, void *unused)
{
	struct acpm_dvfs *dvfs = s->private;

	seq_printf(s, "%d\n", dvfs->front_mode);
	return 0;
}
static ssize_t acpm_dvfs_mode_write(struct file *file, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct acpm_dvfs *dvfs = s->private;
	unsigned int mode;
	char buf[8];

	if (count == 0 || count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	if (kstrtouint(buf, 0, &mode) || mode > ACPM_FRONT_VENDOR)
		return -EINVAL;

	dvfs->front_mode = mode;
	dev_info(dvfs->dev, "front mode %u\n", mode);

	return count;
}
static int acpm_dvfs_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, acpm_dvfs_mode_show, inode->i_private);
}
static const struct file_operations acpm_dvfs_mode_fops = {
	.owner = THIS_MODULE,
	.open = acpm_dvfs_mode_open,
	.read = seq_read,
	.write = acpm_dvfs_mode_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations acpm_dvfs_set_fops = {
	.open = simple_open,
	.write = acpm_dvfs_set_write,
	.llseek = noop_llseek,
};


static int acpm_dvfs_probe(struct platform_device *pdev)
{
	struct acpm_dvfs *dvfs;
	struct device *dev = &pdev->dev;
	u32 chan;
	int ret;

	ret = of_property_read_u32(dev->of_node, "acpm-ipc-channel", &chan);
	if (ret)
		return dev_err_probe(dev, ret, "missing acpm-ipc-channel\n");

	dvfs = devm_kzalloc(dev, sizeof(*dvfs), GFP_KERNEL);
	if (!dvfs)
		return -ENOMEM;

	dvfs->dev = dev;
	dvfs->chan = chan;
	dvfs->acpm = devm_acpm_get_by_node(dev, dev->parent->of_node);
	if (IS_ERR(dvfs->acpm))
		return dev_err_probe(dev, PTR_ERR(dvfs->acpm),
				     "failed to get ACPM handle\n");

	platform_set_drvdata(pdev, dvfs);


	dvfs->debugfs = debugfs_create_dir("acpm-dvfs", NULL);
	debugfs_create_file("dump", 0444, dvfs->debugfs, dvfs,
			    &acpm_dvfs_dump_fops);
	debugfs_create_file("mode", 0644, dvfs->debugfs, dvfs,
			    &acpm_dvfs_mode_fops);
	debugfs_create_file("set", 0200, dvfs->debugfs, dvfs,
			    &acpm_dvfs_set_fops);

	dev_info(dev, "ACPM DVFS channel %u\n", dvfs->chan);

	return 0;
}

static void acpm_dvfs_remove(struct platform_device *pdev)
{
	struct acpm_dvfs *dvfs = platform_get_drvdata(pdev);

	debugfs_remove_recursive(dvfs->debugfs);
}

static const struct of_device_id acpm_dvfs_of_match[] = {
	{ .compatible = "samsung,exynos-acpm-dvfs" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, acpm_dvfs_of_match);

static struct platform_driver acpm_dvfs_driver = {
	.probe = acpm_dvfs_probe,
	.remove = acpm_dvfs_remove,
	.driver = {
		.name = "exynos-acpm-dvfs",
		.of_match_table = acpm_dvfs_of_match,
	},
};
module_platform_driver(acpm_dvfs_driver);

MODULE_AUTHOR("Bao Anh <baoanh@example.com>");
MODULE_DESCRIPTION("Exynos ACPM DVFS client");
MODULE_LICENSE("GPL");
