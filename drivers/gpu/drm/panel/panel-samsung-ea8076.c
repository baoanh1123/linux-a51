// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung EA8076 AMOLED panel driver (1080x2400, MIPI command mode).
 *
 * Found on the Galaxy A51 (SM-A515F, Exynos 9611), panel model AMS646UJ09.
 * The panel is a video-less command mode device: it keeps the last frame in its
 * own memory, so the host pushes a frame only when the display controller has
 * new content and the panel answers with a tearing effect packet.
 *
 * The power-on sequence below is the one the vendor driver sends in
 * ea8076_init() (downstream dpu20/panels/ea8076_a51_lcd_ctrl.c); it has been
 * verified verbatim against the running panel through the DSIM debugfs packet
 * path before being moved here.
 *
 * Brightness uses a 10 bit code in DCS 0x51, mapped from the 0..365 branch
 * index by the vendor's brightness curve.  Index 255 is the UI maximum; above
 * that the panel runs in high brightness mode, which also needs DCS 0x53 to
 * switch the panel's own boost on.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

#define EA8076_WIDTH_MM			64
#define EA8076_HEIGHT_MM		142

#define EA8076_BRIGHTNESS_MAX		365
#define EA8076_BRIGHTNESS_UI_MAX	255
#define EA8076_BRIGHTNESS_DEFAULT	128

/* DCS commands the panel does not expose through video/mipi_display.h */
#define EA8076_DCS_TEST_KEY_ON_F0	0xf0
#define EA8076_DCS_TEST_KEY_OFF_F0	0xf0
#define EA8076_DCS_TEST_KEY_ON_FC	0xfc
#define EA8076_DCS_TEST_KEY_OFF_FC	0xfc
#define EA8076_DCS_WRITE_HBM		0x53
#define EA8076_DCS_WRITE_ACL		0x55

static const u16 ea8076_brightness_table[EA8076_BRIGHTNESS_MAX + 1] = {
	3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36,
	39, 42, 45, 48, 52, 55, 59, 62, 66, 69, 73, 77,
	80, 84, 87, 91, 95, 98, 102, 105, 109, 112, 116, 120,
	123, 127, 130, 134, 138, 141, 145, 148, 152, 156, 159, 163,
	166, 170, 173, 177, 181, 184, 188, 191, 195, 199, 202, 206,
	209, 213, 216, 220, 224, 227, 231, 234, 238, 242, 245, 249,
	252, 256, 259, 263, 267, 270, 274, 277, 281, 285, 288, 292,
	295, 299, 302, 306, 310, 313, 317, 320, 324, 328, 331, 335,
	338, 342, 345, 349, 353, 356, 360, 363, 367, 371, 374, 378,
	381, 385, 388, 392, 396, 399, 403, 406, 410, 414, 417, 421,
	424, 428, 431, 435, 439, 442, 446, 449, 453, 457, 462, 466,
	471, 475, 480, 484, 489, 493, 498, 502, 507, 511, 516, 520,
	525, 529, 534, 538, 543, 547, 552, 556, 561, 565, 570, 574,
	579, 583, 588, 592, 597, 601, 606, 610, 615, 619, 624, 628,
	633, 637, 641, 646, 650, 655, 659, 664, 668, 673, 677, 682,
	686, 691, 695, 700, 704, 709, 713, 718, 722, 727, 731, 736,
	740, 745, 749, 754, 758, 763, 767, 772, 776, 781, 785, 790,
	794, 799, 803, 808, 812, 817, 821, 826, 830, 834, 839, 843,
	848, 852, 857, 861, 866, 870, 875, 879, 884, 888, 893, 897,
	902, 906, 911, 915, 920, 924, 929, 933, 938, 942, 947, 951,
	956, 960, 965, 969, 974, 978, 983, 987, 992, 996, 1001, 1005,
	1010, 1014, 1018, 1023, 3, 7, 11, 15, 18, 22, 26, 29,
	33, 37, 40, 44, 48, 51, 55, 59, 62, 66, 70, 73,
	77, 81, 84, 88, 92, 95, 99, 103, 107, 110, 114, 118,
	121, 125, 129, 132, 136, 140, 143, 147, 151, 154, 158, 162,
	165, 169, 173, 176, 180, 184, 187, 191, 195, 198, 202, 206,
	209, 213, 217, 220, 224, 228, 231, 235, 239, 242, 246, 250,
	253, 257, 261, 264, 268, 272, 275, 279, 283, 286, 290, 294,
	297, 301, 305, 309, 312, 316, 320, 323, 327, 331, 334, 338,
	342, 345, 349, 353, 356, 360, 364, 367, 371, 375, 378, 382,
	386, 389, 393, 397, 400, 404,
};

/**
 * struct ea8076 - panel instance
 * @panel: DRM panel
 * @dsi: DSI device on the inherited link
 * @backlight: backlight device
 * @prepared: panel has been powered up and initialised by this driver
 * @brightness: cached branch brightness index (0-365)
 */
struct ea8076 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct backlight_device *backlight;
	struct gpio_desc *enable_gpio[2];
	struct gpio_desc *reset_gpio;
	bool prepared;
	unsigned int brightness;
};

static inline struct ea8076 *panel_to_ea8076(struct drm_panel *panel)
{
	return container_of(panel, struct ea8076, panel);
}

static const struct {
	u8 len;
	u8 data[16];
} ea8076_init_sequence[] = {
	{ 3, { 0xf0, 0x5a, 0x5a } },		/* unlock level 2 */
	{ 3, { 0xfc, 0x5a, 0x5a } },		/* unlock level 3 */
	{ 5, { 0x2a, 0x00, 0x00, 0x04, 0x37 } },	/* column 0..1079 */
	{ 5, { 0x2b, 0x00, 0x00, 0x09, 0x5f } },	/* row 0..2399 */
	{ 12, { 0xe9, 0x11, 0x65, 0x98, 0x96, 0x80, 0xa9, 0x12, 0x8c, 0x00, 0x1a, 0xb8 } },
	{ 14, { 0xe1, 0x00, 0x00, 0x02, 0x10, 0x10, 0x10, 0x00,
		0x00, 0x20, 0x00, 0x00, 0x01, 0x19 } },
	{ 2, { 0xe0, 0x01 } },			/* vsync */
	{ 3, { 0xe3, 0x57, 0x12 } },		/* power control */
	{ 2, { 0xb0, 0x03 } },
	{ 2, { 0xd4, 0x40 } },
	{ 2, { 0xb0, 0xcc } },
	{ 4, { 0xb9, 0x55, 0x27, 0x65 } },	/* ACL setting */
	{ 2, { 0xb0, 0xd7 } },
	{ 8, { 0xb9, 0x02, 0x61, 0x24, 0x49, 0x41, 0xff, 0x00 } },
	{ 3, { 0xf0, 0xa5, 0xa5 } },		/* lock level 2 */
	{ 3, { 0xfc, 0xa5, 0xa5 } },		/* lock level 3 */
};

static int ea8076_dcs_write(struct ea8076 *ctx, const u8 *data, size_t len)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	ssize_t ret;

	ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	if (ret < 0) {
		dev_err(&dsi->dev, "failed to write DCS 0x%02x: %zd\n",
			data[0], ret);
		return ret;
	}

	return 0;
}

static int ea8076_write_brightness(struct ea8076 *ctx, unsigned int index)
{
	unsigned int value = ea8076_brightness_table[index];
	u8 brightness[3] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
			     (value >> 8) & 0x3, value & 0xff };
	u8 hbm = index > EA8076_BRIGHTNESS_UI_MAX ? 0xe8 : 0x28;
	u8 sequence[] = { EA8076_DCS_TEST_KEY_ON_F0, 0x5a, 0x5a };

	if (!ctx->prepared)
		return 0;

	if (ea8076_dcs_write(ctx, sequence, sizeof(sequence)))
		return -EIO;

	sequence[0] = EA8076_DCS_WRITE_HBM;
	sequence[1] = hbm;
	if (ea8076_dcs_write(ctx, sequence, 2))
		return -EIO;

	if (ea8076_dcs_write(ctx, brightness, sizeof(brightness)))
		return -EIO;

	sequence[0] = EA8076_DCS_WRITE_ACL;
	sequence[1] = 0x00;
	if (ea8076_dcs_write(ctx, sequence, 2))
		return -EIO;

	sequence[0] = EA8076_DCS_TEST_KEY_OFF_F0;
	sequence[1] = 0xa5;
	sequence[2] = 0xa5;

	return ea8076_dcs_write(ctx, sequence, 3);
}

/*
 * Panel power, from the vendor board description (decon_board_ea8076):
 * 1.8 V, then 3.0 V 1 ms later; reset is a low pulse once both rails are up.
 * Power down reverses it and keeps the reset asserted while the rails fall.
 */
static int ea8076_power_on(struct ea8076 *ctx)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ctx->enable_gpio); i++) {
		if (!ctx->enable_gpio[i])
			continue;

		gpiod_set_value_cansleep(ctx->enable_gpio[i], 1);
		usleep_range(1000, 2000);
	}

	usleep_range(10000, 11000);

	if (ctx->reset_gpio) {
		/* Already deasserted for 1 ms, then the reset pulse */
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		usleep_range(10000, 11000);
	}

	return 0;
}

static void ea8076_power_off(struct ea8076 *ctx)
{
	int i;

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		usleep_range(10000, 11000);
	}

	for (i = ARRAY_SIZE(ctx->enable_gpio) - 1; i >= 0; i--) {
		if (!ctx->enable_gpio[i])
			continue;

		gpiod_set_value_cansleep(ctx->enable_gpio[i], 0);
		usleep_range(5000, 6000);
	}
}

static int ea8076_prepare(struct drm_panel *panel)
{
	struct ea8076 *ctx = panel_to_ea8076(panel);
	static const u8 sleep_out = MIPI_DCS_EXIT_SLEEP_MODE;
	static const u8 te_on[] = { MIPI_DCS_SET_TEAR_ON, 0x00, 0x00 };
	static const u8 unlock[] = { EA8076_DCS_TEST_KEY_ON_F0, 0x5a, 0x5a };
	static const u8 lock[] = { EA8076_DCS_TEST_KEY_OFF_F0, 0xa5, 0xa5 };
	unsigned int i;
	int ret;

	if (ctx->prepared)
		return 0;

	ea8076_power_on(ctx);

	ret = ea8076_dcs_write(ctx, &sleep_out, 1);
	if (ret)
		return ret;

	/* The panel needs 10 ms after leaving sleep before it takes commands */
	usleep_range(10000, 11000);

	for (i = 0; i < ARRAY_SIZE(ea8076_init_sequence); i++) {
		ret = ea8076_dcs_write(ctx, ea8076_init_sequence[i].data,
				       ea8076_init_sequence[i].len);
		if (ret)
			return ret;
	}

	ctx->prepared = true;

	/* Brightness and tearing effect, in the panel's command mode */
	ret = ea8076_write_brightness(ctx, ctx->brightness);
	if (ret)
		return ret;

	ret = ea8076_dcs_write(ctx, unlock, sizeof(unlock));
	if (ret)
		return ret;

	ret = ea8076_dcs_write(ctx, te_on, sizeof(te_on));
	if (ret)
		return ret;

	ret = ea8076_dcs_write(ctx, lock, sizeof(lock));
	if (ret)
		return ret;

	/* The vendor driver waits 110 ms before display on */
	msleep(110);

	dev_info(&ctx->dsi->dev, "panel prepared, brightness %u\n",
		 ctx->brightness);

	return 0;
}

static int ea8076_enable(struct drm_panel *panel)
{
	struct ea8076 *ctx = panel_to_ea8076(panel);
	static const u8 display_on = MIPI_DCS_SET_DISPLAY_ON;

	if (!ctx->prepared)
		return 0;

	return ea8076_dcs_write(ctx, &display_on, 1);
}

static int ea8076_disable(struct drm_panel *panel)
{
	struct ea8076 *ctx = panel_to_ea8076(panel);
	static const u8 display_off = MIPI_DCS_SET_DISPLAY_OFF;

	if (!ctx->prepared)
		return 0;

	return ea8076_dcs_write(ctx, &display_off, 1);
}

static int ea8076_unprepare(struct drm_panel *panel)
{
	struct ea8076 *ctx = panel_to_ea8076(panel);
	static const u8 sleep_in = MIPI_DCS_ENTER_SLEEP_MODE;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = ea8076_dcs_write(ctx, &sleep_in, 1);
	ctx->prepared = false;

	/* 120 ms before the panel stops answering, per the vendor driver */
	msleep(120);

	ea8076_power_off(ctx);

	return ret;
}

static int ea8076_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_create(connector->dev);
	if (!mode)
		return -ENOMEM;

	/*
	 * Command mode: this timing describes the panel's own refresh, the host
	 * pushes frames as fast as the panel asks for them with its tearing
	 * effect packets.  The display controller's timing comes from its own
	 * device tree node and has to match this mode.
	 */
	mode->clock = 162000;
	mode->hdisplay = 1080;
	mode->hsync_start = 1080 + 20;
	mode->hsync_end = mode->hsync_start + 4;
	mode->htotal = mode->hsync_end + 20;
	mode->vdisplay = 2400;
	mode->vsync_start = 2400 + 10;
	mode->vsync_end = mode->vsync_start + 4;
	mode->vtotal = mode->vsync_end + 6;
	mode->width_mm = EA8076_WIDTH_MM;
	mode->height_mm = EA8076_HEIGHT_MM;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = EA8076_WIDTH_MM;
	connector->display_info.height_mm = EA8076_HEIGHT_MM;

	return 1;
}

static const struct drm_panel_funcs ea8076_panel_funcs = {
	.prepare = ea8076_prepare,
	.enable = ea8076_enable,
	.disable = ea8076_disable,
	.unprepare = ea8076_unprepare,
	.get_modes = ea8076_get_modes,
};

static int ea8076_bl_update_status(struct backlight_device *bl)
{
	struct ea8076 *ctx = bl_get_data(bl);
	unsigned int index = bl->props.brightness;

	if (index > EA8076_BRIGHTNESS_MAX)
		index = EA8076_BRIGHTNESS_MAX;

	ctx->brightness = index;

	if (!ctx->prepared)
		return 0;

	return ea8076_write_brightness(ctx, index);
}

static const struct backlight_ops ea8076_bl_ops = {
	.update_status = ea8076_bl_update_status,
};

static int ea8076_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct backlight_properties props;
	struct ea8076 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dsi = dsi;
	ctx->brightness = EA8076_BRIGHTNESS_DEFAULT;

	ctx->enable_gpio[0] = devm_gpiod_get_index_optional(dev, "enable", 0,
							    GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->enable_gpio[0]))
		return dev_err_probe(dev, PTR_ERR(ctx->enable_gpio[0]),
				     "failed to get the 1.8 V enable GPIO\n");

	ctx->enable_gpio[1] = devm_gpiod_get_index_optional(dev, "enable", 1,
							    GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->enable_gpio[1]))
		return dev_err_probe(dev, PTR_ERR(ctx->enable_gpio[1]),
				     "failed to get the 3.0 V enable GPIO\n");

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get the reset GPIO\n");

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/*
	 * Command mode with the clock and the link configuration inherited from
	 * the boot chain - the driver only speaks DCS on the running link.
	 */
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	drm_panel_init(&ctx->panel, dev, &ea8076_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = EA8076_BRIGHTNESS_UI_MAX;
	props.brightness = ctx->brightness;
	ctx->backlight = devm_backlight_device_register(dev, "ea8076", dev,
							ctx, &ea8076_bl_ops,
							&props);
	if (IS_ERR(ctx->backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->backlight),
				     "failed to register backlight\n");

	drm_panel_add(&ctx->panel);
	mipi_dsi_set_drvdata(dsi, ctx);

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach DSI device\n");
	}

	dev_info(dev, "EA8076 panel on %u lanes\n", dsi->lanes);

	return 0;
}

static void ea8076_remove(struct mipi_dsi_device *dsi)
{
	struct ea8076 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ea8076_of_match[] = {
	{ .compatible = "samsung,ea8076" },
	{ }
};
MODULE_DEVICE_TABLE(of, ea8076_of_match);

static struct mipi_dsi_driver ea8076_driver = {
	.probe = ea8076_probe,
	.remove = ea8076_remove,
	.driver = {
		.name = "panel-samsung-ea8076",
		.of_match_table = ea8076_of_match,
	},
};
module_mipi_dsi_driver(ea8076_driver);

MODULE_AUTHOR("Exynos 9611 mainline port");
MODULE_DESCRIPTION("Samsung EA8076 AMOLED panel (Galaxy A51)");
MODULE_LICENSE("GPL");
