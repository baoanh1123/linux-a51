// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos 9610/9611 thermal management unit driver
 *
 * The TMU on these SoCs is a single block (one register window, one
 * interrupt) that carries several sensing points belonging to different
 * channels: BIG, LITTLE, G3D and ISP.  This driver follows the Exynos7
 * layout: one sensing point per register, a 9-bit temperature code in the
 * low bits, and one TRIMINFO word per sensing point holding the eFuse
 * calibration code measured at 25 C.
 *
 * Only polled readout is implemented.  Trip interrupts are left masked
 * because there are no cooling devices to act on them yet.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/units.h>

#define EXYNOS9610_TMU_REG_TRIMINFO0		0x00
#define EXYNOS9610_TMU_REG_CONTROL		0x20
#define EXYNOS9610_TMU_CORE_EN			BIT(0)
#define EXYNOS9610_TMU_REG_CURRENT_TEMP0	0x40
#define EXYNOS9610_TMU_REG_CURRENT_TEMP2	0x70

#define EXYNOS9610_TMU_TEMP_MASK		GENMASK(8, 0)

/* The eFuse calibration code is measured at this temperature. */
#define EXYNOS9610_TMU_FIRST_POINT_TRIM		25

/*
 * One code step is half a degree Celsius.  This is confirmed by the trim
 * codes (0x112 = 274 on this board) and the vendor threshold registers: the
 * trips in 0x50..0x6c decode to 55..75 C and a 115 C critical trip is code
 * 454, which fits the 9-bit field exactly.
 */
#define EXYNOS9610_TMU_CODE_PER_DEGREE		2

/*
 * Temperature codes are packed as contiguous 9-bit fields, two per register
 * (the vendor names 0x40 "CURRENT_TEMP1_0", 0x44 "CURRENT_TEMP4_2" ...), and
 * every sensor has its own eFuse trim word at 0x00 + 4 * sensor.
 *
 * Which field belongs to which cluster was established on hardware with
 * per-cluster CPU load: loading the LITTLE cores moves 0x40 and 0x70, while
 * 0x44 sits several degrees higher and follows the whole package.
 */
struct exynos9610_tmu_point {
	u16 reg;
	u8 shift;
};

static const struct exynos9610_tmu_point exynos9610_tmu_points[] = {
	{ EXYNOS9610_TMU_REG_CURRENT_TEMP0, 0 },	/* sensor 0 */
	{ EXYNOS9610_TMU_REG_CURRENT_TEMP0, 9 },	/* sensor 1 */
	{ EXYNOS9610_TMU_REG_CURRENT_TEMP0 + 4, 0 },	/* sensor 2 */
	{ EXYNOS9610_TMU_REG_CURRENT_TEMP0 + 4, 9 },	/* sensor 3 */
	{ EXYNOS9610_TMU_REG_CURRENT_TEMP2, 0 },	/* sensor 4 */
	{ EXYNOS9610_TMU_REG_CURRENT_TEMP2, 9 },	/* sensor 5 */
	{ EXYNOS9610_TMU_REG_CURRENT_TEMP2 + 4, 0 },	/* sensor 6 */
	{ EXYNOS9610_TMU_REG_CURRENT_TEMP2 + 4, 9 },	/* sensor 7 */
};

/* Sensor id used by device tree for "hottest of all sensing points". */
#define EXYNOS9610_TMU_SENSOR_MAX	ARRAY_SIZE(exynos9610_tmu_points)

struct exynos9610_tmu {
	void __iomem *base;
	struct clk *clk;
	u32 trim[ARRAY_SIZE(exynos9610_tmu_points)];
	unsigned int sensor;
};

/* Convert one raw 9-bit code to millidegrees Celsius. */
static int exynos9610_tmu_code_to_temp(struct exynos9610_tmu *tmu,
				       unsigned int sensor, unsigned int code)
{
	int delta = code - (tmu->trim[sensor] & EXYNOS9610_TMU_TEMP_MASK);

	return DIV_ROUND_CLOSEST(delta * MILLIDEGREE_PER_DEGREE,
				 EXYNOS9610_TMU_CODE_PER_DEGREE) +
	       EXYNOS9610_TMU_FIRST_POINT_TRIM * MILLIDEGREE_PER_DEGREE;
}

static int exynos9610_tmu_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct exynos9610_tmu *tmu = thermal_zone_device_priv(tz);
	unsigned int i, first, last;
	int hottest = INT_MIN;

	if (tmu->sensor < EXYNOS9610_TMU_SENSOR_MAX) {
		first = tmu->sensor;
		last = tmu->sensor + 1;
	} else {
		first = 0;
		last = EXYNOS9610_TMU_SENSOR_MAX;
	}

	for (i = first; i < last; i++) {
		const struct exynos9610_tmu_point *p = &exynos9610_tmu_points[i];
		unsigned int code;
		int temp;

		/*
		 * A sensing point without an eFuse trim word reports a raw
		 * code that decodes to a bogus temperature (TRIMINFO3..7 read
		 * back as zero on this board, and e.g. 0x70 >> 9 decodes to
		 * 127 C).  Such a value would trip the critical trip point and
		 * shut the device down, so ignore untrimmed points entirely.
		 */
		if (!(tmu->trim[i] & EXYNOS9610_TMU_TEMP_MASK))
			continue;

		code = (readl(tmu->base + p->reg) >> p->shift) &
		       EXYNOS9610_TMU_TEMP_MASK;
		if (!code)
			continue;

		temp = exynos9610_tmu_code_to_temp(tmu, i, code);
		if (temp < -40000 || temp > 150000)
			continue;

		hottest = max(hottest, temp);
	}

	if (hottest == INT_MIN)
		return -ENODATA;

	*temp = hottest;

	return 0;
}

/* Bring-up aid: dump the register window so the decode can be checked. */
static int exynos9610_tmu_raw_show(struct seq_file *s, void *unused)
{
	struct exynos9610_tmu *tmu = s->private;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tmu->trim); i++)
		seq_printf(s, "trim%u %08x\n", i, tmu->trim[i]);

	for (i = 0x00; i <= 0x1c; i += 4)
		seq_printf(s, "%02x %08x\n", i, readl(tmu->base + i));
	for (i = 0x20; i <= 0x74; i += 4)
		seq_printf(s, "%02x %08x\n", i, readl(tmu->base + i));
	for (i = 0x440; i <= 0x448; i += 4)
		seq_printf(s, "%03x %08x\n", i, readl(tmu->base + i));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(exynos9610_tmu_raw);

static const struct thermal_zone_device_ops exynos9610_tmu_ops = {
	.get_temp = exynos9610_tmu_get_temp,
};

static int exynos9610_tmu_probe(struct platform_device *pdev)
{
	struct exynos9610_tmu *tmu;
	struct thermal_zone_device *tz;
	struct device *dev = &pdev->dev;
	unsigned int i, zones;
	u32 ctrl;

	tmu = devm_kzalloc(dev, sizeof(*tmu), GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;

	tmu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tmu->base))
		return PTR_ERR(tmu->base);

	tmu->clk = devm_clk_get_enabled(dev, "tmu");
	if (IS_ERR(tmu->clk))
		return dev_err_probe(dev, PTR_ERR(tmu->clk),
				     "failed to enable TMU clock\n");

	for (i = 0; i < ARRAY_SIZE(tmu->trim); i++)
		tmu->trim[i] = readl(tmu->base +
				     EXYNOS9610_TMU_REG_TRIMINFO0 + i * 4);

	/* Enable the sensing core; trip interrupts stay masked. */
	ctrl = readl(tmu->base + EXYNOS9610_TMU_REG_CONTROL);
	ctrl |= EXYNOS9610_TMU_CORE_EN;
	writel(ctrl, tmu->base + EXYNOS9610_TMU_REG_CONTROL);

	platform_set_drvdata(pdev, tmu);

	/*
	 * One zone per sensing point referenced from device tree, plus the
	 * synthetic "hottest of all" id used by the aggregate zone.
	 */
	zones = 0;
	for (i = 0; i <= EXYNOS9610_TMU_SENSOR_MAX; i++) {
		tmu->sensor = i;
		tz = devm_thermal_of_zone_register(dev, i, tmu,
						   &exynos9610_tmu_ops);
		if (IS_ERR(tz)) {
			if (PTR_ERR(tz) == -ENODEV)
				continue;
			return dev_err_probe(dev, PTR_ERR(tz),
					     "failed to register zone %u\n", i);
		}
		zones++;
	}

	if (!zones)
		return dev_err_probe(dev, -ENODEV,
				     "no thermal zones reference this sensor\n");

	/*
	 * A single debugfs file shared by all zones is not possible with
	 * private data, so expose the raw window once with the aggregate zone
	 * (the last one registered) as its reader.
	 */
	debugfs_create_file("raw", 0444, NULL, tmu, &exynos9610_tmu_raw_fops);

	dev_info(dev, "%u thermal zone(s), trim %08x %08x %08x\n", zones,
		 tmu->trim[0], tmu->trim[2], tmu->trim[4]);

	return 0;
}

static const struct of_device_id exynos9610_tmu_match[] = {
	{ .compatible = "samsung,exynos9610-tmu" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos9610_tmu_match);

static struct platform_driver exynos9610_tmu_driver = {
	.probe = exynos9610_tmu_probe,
	.driver = {
		.name = "exynos9610-tmu",
		.of_match_table = exynos9610_tmu_match,
	},
};
module_platform_driver(exynos9610_tmu_driver);

MODULE_DESCRIPTION("Samsung Exynos 9610/9611 TMU driver");
MODULE_LICENSE("GPL");
