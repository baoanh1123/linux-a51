// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only Exynos9611 MSHC register snapshot for board bring-up.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define MSHC_SNAPSHOT_MAX	256

struct exynos9611_mshc_snapshot {
	struct miscdevice miscdev;
	char output[MSHC_SNAPSHOT_MAX];
	size_t output_len;
};

struct mshc_register {
	const char *name;
	u16 offset;
};

static const struct mshc_register mshc_registers[] = {
	{ "CLKDIV", 0x008 },
	{ "CLKENA", 0x010 },
	{ "CTYPE", 0x018 },
	{ "FIFOTH", 0x04c },
	{ "VERID", 0x06c },
	{ "HCON", 0x070 },
	{ "UHS_REG", 0x074 },
	{ "CLKSEL", 0x0a8 },
};

static ssize_t mshc_snapshot_read(struct file *file, char __user *buffer,
				  size_t count, loff_t *ppos)
{
	struct miscdevice *miscdev = file->private_data;
	struct exynos9611_mshc_snapshot *snapshot =
		container_of(miscdev, struct exynos9611_mshc_snapshot, miscdev);

	return simple_read_from_buffer(buffer, count, ppos, snapshot->output,
				       snapshot->output_len);
}

static const struct file_operations mshc_snapshot_fops = {
	.owner = THIS_MODULE,
	.read = mshc_snapshot_read,
	.llseek = default_llseek,
};

static int exynos9611_mshc_snapshot_probe(struct platform_device *pdev)
{
	static struct clk_bulk_data clocks[] = {
		{ .id = "aclk" },
		{ .id = "qch" },
	};
	struct exynos9611_mshc_snapshot *snapshot;
	void __iomem *base;
	size_t i;
	int ret;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return dev_err_probe(&pdev->dev, PTR_ERR(base),
				     "failed to map MSHC registers\n");

	ret = devm_clk_bulk_get(&pdev->dev, ARRAY_SIZE(clocks), clocks);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to get MSHC clocks\n");

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(clocks), clocks);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to enable MSHC clocks\n");

	snapshot = devm_kzalloc(&pdev->dev, sizeof(*snapshot), GFP_KERNEL);
	if (!snapshot) {
		ret = -ENOMEM;
		goto disable_clocks;
	}

	for (i = 0; i < ARRAY_SIZE(mshc_registers); i++) {
		u32 value = readl(base + mshc_registers[i].offset);
		int len;

		len = scnprintf(snapshot->output + snapshot->output_len,
				sizeof(snapshot->output) - snapshot->output_len,
				"%s 0x%08x\n", mshc_registers[i].name, value);
		snapshot->output_len += len;
	}

	clk_bulk_disable_unprepare(ARRAY_SIZE(clocks), clocks);

	snapshot->miscdev.minor = MISC_DYNAMIC_MINOR;
	snapshot->miscdev.name = "exynos9611-mshc-snapshot";
	snapshot->miscdev.fops = &mshc_snapshot_fops;
	snapshot->miscdev.mode = 0444;
	platform_set_drvdata(pdev, snapshot);

	ret = misc_register(&snapshot->miscdev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register snapshot endpoint\n");

	dev_info(&pdev->dev, "captured %zu read-only MSHC registers\n",
		 ARRAY_SIZE(mshc_registers));
	return 0;

disable_clocks:
	clk_bulk_disable_unprepare(ARRAY_SIZE(clocks), clocks);
	return ret;
}

static void exynos9611_mshc_snapshot_remove(struct platform_device *pdev)
{
	struct exynos9611_mshc_snapshot *snapshot =
		platform_get_drvdata(pdev);

	misc_deregister(&snapshot->miscdev);
}

static const struct of_device_id exynos9611_mshc_snapshot_of_match[] = {
	{ .compatible = "samsung,exynos9611-mshc-snapshot" },
	{}
};
MODULE_DEVICE_TABLE(of, exynos9611_mshc_snapshot_of_match);

static struct platform_driver exynos9611_mshc_snapshot_driver = {
	.probe = exynos9611_mshc_snapshot_probe,
	.remove = exynos9611_mshc_snapshot_remove,
	.driver = {
		.name = "exynos9611-mshc-snapshot",
		.of_match_table = exynos9611_mshc_snapshot_of_match,
	},
};
module_platform_driver(exynos9611_mshc_snapshot_driver);

MODULE_DESCRIPTION("Read-only Exynos9611 MSHC register snapshot");
MODULE_LICENSE("GPL");
