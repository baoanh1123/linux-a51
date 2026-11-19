// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Samsung Electronics Co., Ltd.
 * Copyright 2020 Google LLC.
 * Copyright 2024 Linaro Ltd.
 */

#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox/exynos-message.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/**
 * struct exynos_mbox_soc_data - per-SoC register offsets.
 * @intmr0:	Interrupt Mask Register 0 (own interrupts).
 * @intgr1:	Interrupt Generation Register 1 (doorbell to the other side).
 * @intmr1:	Interrupt Mask Register 1 (other side's interrupts).
 * @intcr1:	Interrupt Clear Register 1.
 * @intsr1:	Interrupt Status Register 1.
 * @intmsr1:	Interrupt Mask Status Register 1.
 *
 * The gs101 mailboxes use a wider register window; the Exynos9611 APM
 * mailbox uses a compact one (vendor acpm_ipc.h), so the offsets cannot be
 * hardcoded for every SoC.
 */
struct exynos_mbox_soc_data {
	unsigned int intmr0;
	unsigned int intgr0;
	unsigned int intgr1;
	unsigned int intmr1;
	unsigned int intcr1;
	unsigned int intsr1;
	unsigned int intmsr1;
};

static const struct exynos_mbox_soc_data exynos_mbox_gs101 = {
	.intmr0 = 0x28,
	.intgr1 = 0x40,
	.intmr1 = 0x48,
	.intcr1 = 0x44,
	.intsr1 = 0x4c,
	.intmsr1 = 0x50,
};

/* Exynos9611: vendor drivers/soc/samsung/acpm/acpm_ipc.h. */
static const struct exynos_mbox_soc_data exynos_mbox_exynos9611 = {
	.intmr0 = 0x10,
	.intgr0 = 0x08,
	.intgr1 = 0x1c,
	.intmr1 = 0x24,
	.intcr1 = 0x20,
	.intsr1 = 0x28,
	.intmsr1 = 0x2c,
};

#define EXYNOS_MBOX_INTMR0_MASK		GENMASK(15, 0)
#define EXYNOS_MBOX_INTGR1_MASK		GENMASK(15, 0)

#define EXYNOS_MBOX_CHAN_COUNT		HWEIGHT32(EXYNOS_MBOX_INTGR1_MASK)

/**
 * struct exynos_mbox - driver's private data.
 * @regs:	mailbox registers base address.
 * @mbox:	pointer to the mailbox controller.
 * @pclk:	pointer to the mailbox peripheral clock.
 */
static struct exynos_mbox *exynos_mbox_inst;

struct exynos_mbox {
	void __iomem *regs;
	struct mbox_controller *mbox;
	struct mbox_chan *chans;
	const struct exynos_mbox_soc_data *soc;
	struct clk *pclk;
	int irq;
	u32 irq_status;
};

static int exynos_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct device *dev = chan->mbox->dev;
	struct exynos_mbox *exynos_mbox = dev_get_drvdata(dev);
	struct exynos_mbox_msg *msg = data;
	u32 mask;

	if (msg->chan_id >= exynos_mbox->mbox->num_chans) {
		dev_err(dev, "Invalid channel ID %d\n", msg->chan_id);
		return -EINVAL;
	}

	if (msg->chan_type != EXYNOS_MBOX_CHAN_TYPE_DOORBELL) {
		dev_err(dev, "Unsupported channel type [%d]\n", msg->chan_type);
		return -EINVAL;
	}

	/*
	 * The firmware answers on the interrupt bit of the ACPM channel the
	 * message belongs to (msg->chan_id), while startup() only unmasks the
	 * single mailbox channel the client requested - the ACPM protocol
	 * driver requests one mailbox channel and addresses ACPM channels
	 * through chan_id.  Without this the reply interrupt stays masked and
	 * every request times out with a mailbox IRQ count of zero.
	 */
	mask = readl(exynos_mbox->regs + exynos_mbox->soc->intmr1);
	if (mask & BIT(msg->chan_id))
		writel(mask & ~BIT(msg->chan_id),
		       exynos_mbox->regs + exynos_mbox->soc->intmr1);

	/*
	 * Exynos9611 ACPM firmware is woken through the APM NVIC: the vendor
	 * driver writes BIT(id) << 16 to INTGR0 for that, and only uses INTGR1
	 * to return RX credits. gs101 mailboxes use INTGR1 as the doorbell.
	 */
	if (exynos_mbox->soc->intgr0)
		writel(BIT(msg->chan_id) << 16,
		       exynos_mbox->regs + exynos_mbox->soc->intgr0);
	else
		writel(BIT(msg->chan_id),
		       exynos_mbox->regs + exynos_mbox->soc->intgr1);

	return 0;
}

/*
 * Interrupt driven channels need their interrupt unmasked in INTMR1; polling
 * channels (the PMIC channel, for instance) keep it masked and simply re-read
 * the shared memory queue.  Unmasking an extra channel is harmless: the
 * handler only reports a pending RX to the client, which is what the polling
 * path does on its own anyway.
 */
static int exynos_mbox_startup(struct mbox_chan *chan)
{
	struct exynos_mbox *exynos_mbox = chan->mbox->dev ? dev_get_drvdata(chan->mbox->dev) : NULL;
	unsigned long idx;
	u32 mask;

	if (!exynos_mbox)
		return 0;

	idx = chan - exynos_mbox->chans;
	mask = readl(exynos_mbox->regs + exynos_mbox->soc->intmr1);
	mask &= ~BIT(idx);
	writel(mask, exynos_mbox->regs + exynos_mbox->soc->intmr1);

	return 0;
}

static irqreturn_t exynos_mbox_irq(int irq, void *data)
{
	struct exynos_mbox *exynos_mbox = data;
	u32 status;

	status = readl(exynos_mbox->regs + exynos_mbox->soc->intsr1);
	if (!status)
		return IRQ_NONE;

	/* Acknowledge the pending interrupts before waking the thread. */
	writel(status, exynos_mbox->regs + exynos_mbox->soc->intcr1);
	exynos_mbox->irq_status = status;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t exynos_mbox_irq_thread(int irq, void *data)
{
	struct exynos_mbox *exynos_mbox = data;
	unsigned int bit;

	for (bit = 0; bit < exynos_mbox->mbox->num_chans; bit++) {
		if (!(exynos_mbox->irq_status & BIT(bit)))
			continue;

		mbox_chan_received_data(&exynos_mbox->chans[bit], NULL);
	}

	return IRQ_HANDLED;
}

/**
 * exynos_mbox_chan_ack() - acknowledge a channel and return its RX credit.
 * @chan:	mailbox channel.
 * @chan_id:	hardware channel id whose data was consumed.
 *
 * The ACPM firmware stops delivering on a channel until the AP clears the
 * pending bit (INTCR1) and hands back a credit (INTGR1, the doorbell towards
 * the firmware).  The vendor driver does both after draining a queue; without
 * it the firmware can go quiet after the first message.
 */
void exynos_mbox_chan_ack(struct mbox_chan *chan, unsigned int chan_id)
{
	struct exynos_mbox *exynos_mbox = dev_get_drvdata(chan->mbox->dev);

	if (!exynos_mbox || chan_id >= EXYNOS_MBOX_CHAN_COUNT)
		return;

	writel(BIT(chan_id), exynos_mbox->regs + exynos_mbox->soc->intcr1);
	if (exynos_mbox->soc->intgr1)
		writel(BIT(chan_id),
		       exynos_mbox->regs + exynos_mbox->soc->intgr1);
}
EXPORT_SYMBOL_GPL(exynos_mbox_chan_ack);

static const struct mbox_chan_ops exynos_mbox_chan_ops = {
	.send_data = exynos_mbox_send_data,
	.startup = exynos_mbox_startup,
};

static struct mbox_chan *exynos_mbox_of_xlate(struct mbox_controller *mbox,
					      const struct of_phandle_args *sp)
{
	int i;

	if (sp->args_count != 0)
		return ERR_PTR(-EINVAL);

	/*
	 * Return the first available channel. When we don't pass the
	 * channel ID from device tree, each channel populated by the driver is
	 * just a software construct or a virtual channel. We use 'void *data'
	 * in send_data() to pass the channel identifiers.
	 */
	for (i = 0; i < mbox->num_chans; i++)
		if (mbox->chans[i].cl == NULL)
			return &mbox->chans[i];
	return ERR_PTR(-EINVAL);
}

static const struct of_device_id exynos_mbox_match[] = {
	{ .compatible = "google,gs101-mbox", .data = &exynos_mbox_gs101 },
	{ .compatible = "samsung,exynos9611-mbox",
	  .data = &exynos_mbox_exynos9611 },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_mbox_match);

/**
 * exynos_mbox_int_status() - read the raw interrupt status register
 * @chan: mailbox channel used for the query
 *
 * The vendor drivers poll this register instead of a receive queue when they
 * wait for a response, so expose it for clients that do the same.
 *
 * Return: raw INTSR value, or 0 when no mailbox is registered.
 */
u32 exynos_mbox_int_status(struct mbox_chan *chan)
{
	struct exynos_mbox *mbox = exynos_mbox_inst;

	if (!mbox)
		return 0;

	return readl(mbox->regs + mbox->soc->intsr1);
}
EXPORT_SYMBOL_GPL(exynos_mbox_int_status);

static int exynos_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_mbox *exynos_mbox;
	struct mbox_controller *mbox;
	struct mbox_chan *chans;
	int i, ret;

	exynos_mbox = devm_kzalloc(dev, sizeof(*exynos_mbox), GFP_KERNEL);
	if (!exynos_mbox)
		return -ENOMEM;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	chans = devm_kcalloc(dev, EXYNOS_MBOX_CHAN_COUNT, sizeof(*chans),
			     GFP_KERNEL);
	if (!chans)
		return -ENOMEM;

	exynos_mbox->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(exynos_mbox->regs))
		return PTR_ERR(exynos_mbox->regs);

	/*
	 * Exynos9611 leaves the APM mailbox clock running from the boot chain
	 * (the vendor ACPM driver never enables one either), so accept a
	 * missing clock instead of failing the probe.
	 */
	exynos_mbox->soc = of_device_get_match_data(dev);
	if (!exynos_mbox->soc)
		return dev_err_probe(dev, -EINVAL, "Missing SoC data.\n");

	exynos_mbox->pclk = devm_clk_get_optional_enabled(dev, "pclk");
	if (IS_ERR(exynos_mbox->pclk))
		return dev_err_probe(dev, PTR_ERR(exynos_mbox->pclk),
				     "Failed to enable clock.\n");

	mbox->num_chans = EXYNOS_MBOX_CHAN_COUNT;
	mbox->chans = chans;
	mbox->dev = dev;
	mbox->ops = &exynos_mbox_chan_ops;
	mbox->of_xlate = exynos_mbox_of_xlate;

	for (i = 0; i < EXYNOS_MBOX_CHAN_COUNT; i++)
		chans[i].mbox = mbox;

	exynos_mbox->mbox = mbox;
	exynos_mbox->chans = chans;

	platform_set_drvdata(pdev, exynos_mbox);

	/*
	 * Leave INTMR0 alone.  It masks the APM "generate" path, i.e. which
	 * doorbells the AP may raise towards the firmware, and the vendor ACPM
	 * driver never writes it.  Masking it with 0xffff silently disabled
	 * every AP -> ACPM request while the firmware kept running its own
	 * periodic work, which is exactly what we saw: requests timed out with
	 * INTSR1 bit 0 and no firmware log entry for the channel.
	 *
	 * INTMR1 is the firmware -> AP direction and is set up from the
	 * channel table (sum of polling channels), as the vendor does.
	 */
	writel(EXYNOS_MBOX_INTGR1_MASK,
	       exynos_mbox->regs + exynos_mbox->soc->intmr1);

	exynos_mbox->irq = platform_get_irq(pdev, 0);
	if (exynos_mbox->irq > 0) {
		ret = devm_request_threaded_irq(dev, exynos_mbox->irq,
						exynos_mbox_irq,
						exynos_mbox_irq_thread,
						IRQF_ONESHOT, dev_name(dev),
						exynos_mbox);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to request IRQ.\n");
	}

	exynos_mbox_inst = exynos_mbox;

	return devm_mbox_controller_register(dev, mbox);
}

static struct platform_driver exynos_mbox_driver = {
	.probe	= exynos_mbox_probe,
	.driver	= {
		.name = "exynos-acpm-mbox",
		.of_match_table	= exynos_mbox_match,
	},
};
module_platform_driver(exynos_mbox_driver);

MODULE_AUTHOR("Tudor Ambarus <tudor.ambarus@linaro.org>");
MODULE_DESCRIPTION("Samsung Exynos mailbox driver");
MODULE_LICENSE("GPL");
