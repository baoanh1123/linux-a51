// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal PID 1 for validating Exynos9611 CMU clock registration.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <unistd.h>

#define CLOCK_SUMMARY "/debug/clk/clk_summary"
#define LINE_SIZE 512
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

static int console_fd = -1;
static int kmsg_fd = -1;
static int pmsg_fd = -1;
static int fb_fd = -1;
static uint32_t *framebuffer;
static struct fb_fix_screeninfo fb_fix;
static struct fb_var_screeninfo fb_var;

struct clock_result {
	const char *name;
	unsigned long rate;
	bool passed;
};

struct rgb {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
};

static const uint8_t font_5x7[][7] = {
	[0] = { 0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11 }, /* A */
	[1] = { 0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e }, /* B */
	[2] = { 0x0f, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0f }, /* C */
	[3] = { 0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e }, /* D */
	[4] = { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f }, /* E */
	[5] = { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 }, /* F */
	[6] = { 0x0f, 0x10, 0x10, 0x13, 0x11, 0x11, 0x0f }, /* G */
	[7] = { 0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11 }, /* H */
	[8] = { 0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1f }, /* I */
	[9] = { 0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0c }, /* J */
	[10] = { 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 }, /* K */
	[11] = { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f }, /* L */
	[12] = { 0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11 }, /* M */
	[13] = { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 }, /* N */
	[14] = { 0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e }, /* O */
	[15] = { 0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10 }, /* P */
	[16] = { 0x0e, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0d }, /* Q */
	[17] = { 0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11 }, /* R */
	[18] = { 0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e }, /* S */
	[19] = { 0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 }, /* T */
	[20] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e }, /* U */
	[21] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04 }, /* V */
	[22] = { 0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0a }, /* W */
	[23] = { 0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11 }, /* X */
	[24] = { 0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04 }, /* Y */
	[25] = { 0x1f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1f }, /* Z */
	[26] = { 0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e }, /* 0 */
	[27] = { 0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e }, /* 1 */
	[28] = { 0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f }, /* 2 */
	[29] = { 0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e }, /* 3 */
	[30] = { 0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02 }, /* 4 */
	[31] = { 0x1f, 0x10, 0x10, 0x1e, 0x01, 0x01, 0x1e }, /* 5 */
	[32] = { 0x0e, 0x10, 0x10, 0x1e, 0x11, 0x11, 0x0e }, /* 6 */
	[33] = { 0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 }, /* 7 */
	[34] = { 0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e }, /* 8 */
	[35] = { 0x0e, 0x11, 0x11, 0x0f, 0x01, 0x01, 0x0e }, /* 9 */
};

static const struct rgb color_background = { 8, 15, 28 };
static const struct rgb color_header = { 18, 35, 60 };
static const struct rgb color_row = { 15, 25, 42 };
static const struct rgb color_row_alt = { 11, 20, 35 };
static const struct rgb color_text = { 232, 240, 250 };
static const struct rgb color_muted = { 140, 165, 190 };
static const struct rgb color_pass = { 55, 220, 135 };
static const struct rgb color_fail = { 255, 90, 90 };
static const struct rgb color_rate = { 100, 205, 255 };

static uint32_t pack_color(struct rgb color)
{
	uint32_t pixel = 0;
	unsigned int max;

	max = (1U << fb_var.red.length) - 1;
	pixel |= ((color.red * max + 127) / 255) << fb_var.red.offset;
	max = (1U << fb_var.green.length) - 1;
	pixel |= ((color.green * max + 127) / 255) << fb_var.green.offset;
	max = (1U << fb_var.blue.length) - 1;
	pixel |= ((color.blue * max + 127) / 255) << fb_var.blue.offset;
	if (fb_var.transp.length) {
		max = (1U << fb_var.transp.length) - 1;
		pixel |= max << fb_var.transp.offset;
	}

	return pixel;
}

static int init_framebuffer(void)
{
	uint64_t required_size;

	fb_fd = open("/dev/fb0", O_RDWR | O_CLOEXEC);
	if (fb_fd < 0)
		return -errno;

	if (ioctl(fb_fd, FBIOGET_FSCREENINFO, &fb_fix) < 0 ||
	    ioctl(fb_fd, FBIOGET_VSCREENINFO, &fb_var) < 0)
		goto ioctl_error;

	required_size = (uint64_t)fb_fix.line_length * fb_var.yres;
	if (fb_var.bits_per_pixel != 32 ||
	    fb_fix.type != FB_TYPE_PACKED_PIXELS ||
	    fb_fix.visual != FB_VISUAL_TRUECOLOR ||
	    fb_var.xres < 640 || fb_var.yres < 400 ||
	    fb_fix.line_length < fb_var.xres * sizeof(uint32_t) ||
	    fb_fix.smem_len < required_size ||
	    !fb_var.red.length || !fb_var.green.length ||
	    !fb_var.blue.length || fb_var.red.length > 8 ||
	    fb_var.green.length > 8 || fb_var.blue.length > 8 ||
	    fb_var.transp.length > 8 ||
	    fb_var.red.offset + fb_var.red.length > 32 ||
	    fb_var.green.offset + fb_var.green.length > 32 ||
	    fb_var.blue.offset + fb_var.blue.length > 32 ||
	    fb_var.transp.offset + fb_var.transp.length > 32 ||
	    fb_var.red.msb_right || fb_var.green.msb_right ||
	    fb_var.blue.msb_right || fb_var.transp.msb_right) {
		close(fb_fd);
		fb_fd = -1;
		return -EINVAL;
	}

	framebuffer = mmap(NULL, fb_fix.smem_len, PROT_READ | PROT_WRITE,
			   MAP_SHARED, fb_fd, 0);
	if (framebuffer == MAP_FAILED) {
		framebuffer = NULL;
		goto mmap_error;
	}

	return 0;

ioctl_error:
	{
		int error = -errno;

		close(fb_fd);
		fb_fd = -1;
		return error;
	}

mmap_error:
	{
		int error = -errno;

		close(fb_fd);
		fb_fd = -1;
		return error;
	}
}

static void fill_rect(unsigned int x, unsigned int y, unsigned int width,
		      unsigned int height, struct rgb color)
{
	uint32_t pixel = pack_color(color);
	unsigned int row;
	unsigned int col;
	unsigned int stride = fb_fix.line_length / sizeof(uint32_t);

	if (x >= fb_var.xres || y >= fb_var.yres)
		return;
	if (width > fb_var.xres - x)
		width = fb_var.xres - x;
	if (height > fb_var.yres - y)
		height = fb_var.yres - y;

	for (row = y; row < y + height; row++)
		for (col = x; col < x + width; col++)
			framebuffer[row * stride + col] = pixel;
}

static const uint8_t *glyph_rows(char ch)
{
	static const uint8_t space[7];
	static const uint8_t dash[7] = { 0, 0, 0, 0x1f, 0, 0, 0 };
	static const uint8_t underscore[7] = { 0, 0, 0, 0, 0, 0, 0x1f };
	static const uint8_t period[7] = { 0, 0, 0, 0, 0, 0x0c, 0x0c };
	static const uint8_t colon[7] = { 0, 0x0c, 0x0c, 0, 0x0c, 0x0c, 0 };
	static const uint8_t comma[7] = { 0, 0, 0, 0, 0x0c, 0x0c, 0x08 };
	static const uint8_t slash[7] = { 0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10 };
	static const uint8_t equals[7] = { 0, 0x1f, 0, 0x1f, 0, 0, 0 };
	static const uint8_t plus[7] = { 0, 0x04, 0x04, 0x1f, 0x04, 0x04, 0 };
	static const uint8_t left_bracket[7] = { 0x0e, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0e };
	static const uint8_t right_bracket[7] = { 0x0e, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0e };
	static const uint8_t question[7] = { 0x0e, 0x11, 0x01, 0x02, 0x04, 0, 0x04 };

	if (ch >= 'a' && ch <= 'z')
		ch = toupper((unsigned char)ch);
	if (ch >= 'A' && ch <= 'Z')
		return font_5x7[ch - 'A'];
	if (ch >= '0' && ch <= '9')
		return font_5x7[26 + ch - '0'];

	switch (ch) {
	case ' ':
		return space;
	case '-':
		return dash;
	case '_':
		return underscore;
	case '.':
		return period;
	case ':':
		return colon;
	case ',':
		return comma;
	case '/':
		return slash;
	case '=':
		return equals;
	case '+':
		return plus;
	case '[':
		return left_bracket;
	case ']':
		return right_bracket;
	default:
		return question;
	}
}

static void draw_text(unsigned int x, unsigned int y, unsigned int scale,
		      const char *text, struct rgb color)
{
	unsigned int stride = fb_fix.line_length / sizeof(uint32_t);
	uint32_t pixel = pack_color(color);
	unsigned int index;

	for (index = 0; text[index] && x < fb_var.xres; index++) {
		const uint8_t *glyph = glyph_rows(text[index]);
		unsigned int row;

		for (row = 0; row < 7; row++) {
			unsigned int col;

			for (col = 0; col < 5; col++) {
				unsigned int repeat_x;
				unsigned int repeat_y;

				if (!(glyph[row] & (1U << (4 - col))) ||
				    x + (col + 1) * scale > fb_var.xres ||
				    y + (row + 1) * scale > fb_var.yres)
					continue;

				for (repeat_y = 0; repeat_y < scale; repeat_y++)
					for (repeat_x = 0; repeat_x < scale;
					     repeat_x++)
						framebuffer[(y + row * scale +
						     repeat_y) * stride + x +
						    col * scale + repeat_x] = pixel;
			}
		}
		x += 6 * scale;
	}
}

static void format_rate(unsigned long rate, char *buffer, size_t size)
{
	char digits[32];
	size_t digits_len;
	size_t out = 0;
	size_t i;

	snprintf(digits, sizeof(digits), "%lu", rate);
	digits_len = strlen(digits);
	for (i = 0; i < digits_len && out + 1 < size; i++) {
		if (i && (digits_len - i) % 3 == 0)
			buffer[out++] = ',';
		buffer[out++] = digits[i];
	}
	snprintf(buffer + out, size - out, " HZ");
}

static void draw_debug_screen(const struct clock_result *results,
			      unsigned int count, unsigned int failures)
{
	const unsigned int left = 24;
	const unsigned int top = 140;
	unsigned int scale = 3;
	unsigned int row_height;
	unsigned int i;
	char summary[96];
	char rate[32];

	if (!framebuffer)
		return;

	if (count * 27 > fb_var.yres - top - 24)
		scale = 2;
	row_height = 9 * scale;

	fill_rect(0, 0, fb_var.xres, fb_var.yres, color_background);
	fill_rect(0, 0, fb_var.xres, 118, color_header);
	draw_text(left, 22, 4, "EXYNOS9611 CMU TEST", color_text);
	snprintf(summary, sizeof(summary), "%u CLOCKS  PASS %u  FAIL %u",
		 count, count - failures, failures);
	draw_text(left, 78, 2, summary, failures ? color_fail : color_muted);
	fill_rect(left, 124, fb_var.xres - left * 2, 4, color_rate);

	for (i = 0; i < count; i++) {
		unsigned int y = top + i * row_height;
		unsigned int rate_width = 15 * 6 * scale;
		unsigned int rate_x = fb_var.xres > rate_width + left ?
			fb_var.xres - rate_width - left : left;
		struct rgb background = i & 1 ? color_row_alt : color_row;

		fill_rect(left, y, fb_var.xres - left * 2, row_height - 2,
			  background);
		draw_text(left + 8, y + scale, scale,
			  results[i].passed ? "PASS" : "FAIL",
			  results[i].passed ? color_pass : color_fail);
		draw_text(left + 8 + 5 * 6 * scale, y + scale, scale,
			  results[i].name, color_text);
		if (results[i].passed) {
			format_rate(results[i].rate, rate, sizeof(rate));
			draw_text(rate_x, y + scale, scale, rate, color_rate);
		} else {
			draw_text(rate_x, y + scale, scale, "NO RATE",
				  color_fail);
		}
	}

	if (failures)
		snprintf(summary, sizeof(summary), "CHECK FAILED  %u CLOCKS",
			 failures);
	else
		snprintf(summary, sizeof(summary), "ALL CLOCK CHECKS PASSED");
	draw_text(left, fb_var.yres - 34, 3, summary,
		  failures ? color_fail : color_pass);
	__sync_synchronize();
}

static void draw_error_screen(const char *title)
{
	unsigned int scale;

	if (!framebuffer)
		return;

	fill_rect(0, 0, fb_var.xres, fb_var.yres, color_background);
	fill_rect(0, 0, fb_var.xres, 118, color_header);
	draw_text(24, 22, 4, "EXYNOS9611 CMU TEST", color_text);
	fill_rect(24, 124, fb_var.xres - 48, 4, color_fail);

	scale = fb_var.xres >= 720 ? 4 : 2;
	draw_text(24, fb_var.yres / 3, scale, title, color_fail);
	draw_text(24, fb_var.yres / 3 + 7 * scale + 24, 2,
		  "SEE CONSOLE LOG", color_muted);
	draw_text(24, fb_var.yres - 34, 3, "TEST RESULT FAIL", color_fail);
	__sync_synchronize();
}

static void write_all(int fd, const char *buf, size_t len)
{
	while (len) {
		ssize_t written = write(fd, buf, len);

		if (written < 0) {
			if (errno == EINTR)
				continue;
			return;
		}
		if (!written)
			return;

		buf += written;
		len -= written;
	}
}

static void report(const char *format, ...)
{
	char message[LINE_SIZE];
	char kmsg[LINE_SIZE + 4];
	va_list args;
	int len;

	va_start(args, format);
	len = vsnprintf(message, sizeof(message), format, args);
	va_end(args);
	if (len < 0)
		return;

	len = strnlen(message, sizeof(message));
	if (console_fd >= 0) {
		write_all(console_fd, message, len);
		write_all(console_fd, "\n", 1);
	} else {
		write_all(STDOUT_FILENO, message, len);
		write_all(STDOUT_FILENO, "\n", 1);
	}

	if (kmsg_fd >= 0) {
		int kmsg_len = snprintf(kmsg, sizeof(kmsg), "<6>%.*s\n",
					 (int)len, message);

		if (kmsg_len > 0)
			write_all(kmsg_fd, kmsg,
				  strnlen(kmsg, sizeof(kmsg)));
	}

	if (pmsg_fd >= 0) {
		write_all(pmsg_fd, message, len);
		write_all(pmsg_fd, "\n", 1);
	}
}

static int mount_fs(const char *source, const char *target,
		    const char *type, unsigned long flags)
{
	if (!mkdir(target, 0755) || errno == EEXIST) {
		if (!mount(source, target, type, flags, NULL) || errno == EBUSY)
			return 0;
	}

	report("FAIL: mount %s on %s: %s", type, target, strerror(errno));
	return -1;
}

static bool check_clock(FILE *summary, struct clock_result *result)
{
	char line[LINE_SIZE];

	rewind(summary);
	while (fgets(line, sizeof(line), summary)) {
		char *saveptr;
		char *token;
		unsigned int column;
		unsigned long rate;
		char *end;

		token = strtok_r(line, " \t\r\n", &saveptr);
		if (!token || strcmp(token, result->name))
			continue;

		for (column = 1; column <= 4; column++) {
			token = strtok_r(NULL, " \t\r\n", &saveptr);
			if (!token)
				break;
		}
		if (!token)
			break;

		errno = 0;
		rate = strtoul(token, &end, 10);
		if (errno || end == token || *end || !rate)
			break;

		result->rate = rate;
		result->passed = true;
		report("PASS: %s reports %lu Hz", result->name, rate);
		return true;
	}

	report("FAIL: %s is missing or reports a zero/invalid rate",
	       result->name);
	return false;
}

static void wait_forever(void)
{
	for (;;)
		pause();
}

int main(void)
{
	static const char * const diagnostic_markers[] = {
		"RAMOOPS-DIAG-BEGIN-7F3A",
		"RAMOOPS-DIAG-ASCII-000: 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz",
		"RAMOOPS-DIAG-ASCII-001: abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ9876543210",
		"RAMOOPS-DIAG-END-7F3A",
	};
	static const char * const clocks[] = {
		"fout_shared0_pll",
		"dout_shared0_div2",
		"dout_shared0_div4",
		"fout_shared1_pll",
		"dout_shared1_div2",
		"dout_shared1_div4",
		"mout_apm_bus",
		"gout_apm_bus",
		"dout_apm_bus",
		"clk_dll_dco",
		"mout_apm_bus_user",
		"mout_dll_user",
		"mout_clk_apm_bus",
		"div_clk_apm_bus",
		"gout_gpio_alive_pclk",
		"gout_apm_bus_rstsync",
		"gout_clkcmu_cmgp_bus",
		"gout_cmgp_cmu_pclk",
		"gout_rtc_pclk",
		"gout_top_rtc_pclk",
		"gout_fsys_mmc_card_aclk",
		"gout_fsys_mmc_card_sdclkin",
		"gout_fsys_mmc_card_qch",
	};
	FILE *summary;
	struct clock_result results[ARRAY_SIZE(clocks)] = {};
	unsigned int i;
	unsigned int failures = 0;
	int ret;

	report("Exynos9611 CMU initramfs test started");
	if (mount_fs("devtmpfs", "/dev", "devtmpfs", MS_NOSUID)) {
		report("TEST RESULT: FAIL");
		wait_forever();
	}

	console_fd = open("/dev/console", O_WRONLY | O_NOCTTY);
	if (console_fd < 0)
		report("WARN: open /dev/console: %s", strerror(errno));
	kmsg_fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
	if (kmsg_fd < 0)
		report("WARN: open /dev/kmsg: %s", strerror(errno));
	pmsg_fd = open("/dev/pmsg0", O_WRONLY | O_CLOEXEC);
	if (pmsg_fd < 0)
		report("WARN: open /dev/pmsg0: %s", strerror(errno));

	ret = init_framebuffer();
	if (ret)
		report("WARN: framebuffer UI unavailable: %s",
		       strerror(-ret));

	for (i = 0; i < ARRAY_SIZE(diagnostic_markers); i++)
		report("%s", diagnostic_markers[i]);

	if (mount_fs("debugfs", "/debug", "debugfs", MS_NOSUID)) {
		report("TEST RESULT: FAIL");
		draw_error_screen("DEBUGFS MOUNT FAILED");
		wait_forever();
	}

	summary = fopen(CLOCK_SUMMARY, "r");
	if (!summary) {
		report("FAIL: open %s: %s", CLOCK_SUMMARY, strerror(errno));
		report("TEST RESULT: FAIL");
		draw_error_screen("CLOCK SUMMARY MISSING");
		wait_forever();
	}

	for (i = 0; i < ARRAY_SIZE(clocks); i++) {
		results[i].name = clocks[i];
		if (!check_clock(summary, &results[i]))
			failures++;
	}

	fclose(summary);
	report("TEST RESULT: %s", failures ? "FAIL" : "PASS");
	draw_debug_screen(results, ARRAY_SIZE(results), failures);
	wait_forever();
}
