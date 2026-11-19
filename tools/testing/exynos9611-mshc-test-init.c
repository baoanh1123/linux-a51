// SPDX-License-Identifier: GPL-2.0-only
/*
 * Framebuffer-only Exynos9611 A51 MSHC bring-up test init.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <unistd.h>

#define LINE_SIZE	256
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#define TEST_ROW_COUNT		8
#define TEST_ROW_TOP		220
#define TEST_ROW_MAX_HEIGHT	100

struct rgb {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
};

static const uint8_t font_5x7[36][7] = {
	{ 0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11 },
	{ 0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e },
	{ 0x0f, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0f },
	{ 0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e },
	{ 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f },
	{ 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
	{ 0x0f, 0x10, 0x10, 0x13, 0x11, 0x11, 0x0f },
	{ 0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11 },
	{ 0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1f },
	{ 0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0c },
	{ 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 },
	{ 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f },
	{ 0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11 },
	{ 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 },
	{ 0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e },
	{ 0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10 },
	{ 0x0e, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0d },
	{ 0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11 },
	{ 0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e },
	{ 0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 },
	{ 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e },
	{ 0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04 },
	{ 0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0a },
	{ 0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11 },
	{ 0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04 },
	{ 0x1f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1f },
	{ 0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e },
	{ 0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e },
	{ 0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f },
	{ 0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e },
	{ 0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02 },
	{ 0x1f, 0x10, 0x10, 0x1e, 0x01, 0x01, 0x1e },
	{ 0x0e, 0x10, 0x10, 0x1e, 0x11, 0x11, 0x0e },
	{ 0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 },
	{ 0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e },
	{ 0x0e, 0x11, 0x11, 0x0f, 0x01, 0x01, 0x0e },
};

static const struct rgb color_background = { 8, 15, 28 };
static const struct rgb color_header = { 18, 35, 60 };
static const struct rgb color_text = { 232, 240, 250 };
static const struct rgb color_muted = { 140, 165, 190 };
static const struct rgb color_pass = { 55, 220, 135 };
static const struct rgb color_fail = { 255, 90, 90 };
static const struct rgb color_value = { 100, 205, 255 };

static int kmsg_fd = -1;
static int pmsg_fd = -1;
static int fb_fd = -1;
static uint32_t *framebuffer;
static struct fb_fix_screeninfo fb_fix;
static struct fb_var_screeninfo fb_var;

static int write_all(int fd, const char *buf, size_t len)
{
	while (len) {
		ssize_t written = write(fd, buf, len);

		if (written < 0) {
			if (errno == EINTR)
				continue;
			return -1;
		}
		if (!written)
			return -1;

		buf += written;
		len -= written;
	}

	return 0;
}

static void report(const char *message)
{
	char kmsg[LINE_SIZE];
	size_t len = strlen(message);

	if (kmsg_fd >= 0) {
		int kmsg_len = snprintf(kmsg, sizeof(kmsg),
					"<6>Exynos9611-MSHC-test: %s\n",
					message);

		if (kmsg_len < 0 || (size_t)kmsg_len >= sizeof(kmsg) ||
		    write_all(kmsg_fd, kmsg, kmsg_len)) {
			close(kmsg_fd);
			kmsg_fd = -1;
		}
	}

	if (pmsg_fd >= 0 &&
	    (write_all(pmsg_fd, message, len) || write_all(pmsg_fd, "\n", 1))) {
		close(pmsg_fd);
		pmsg_fd = -1;
	}
}

static void dump_mmc_kmsg(void)
{
	char buf[65536];
	char line[LINE_SIZE];
	size_t line_len = 0;
	int fd;
	ssize_t n;

	if (pmsg_fd < 0)
		return;

	fd = open("/dev/kmsg", O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0)
		return;

	write_all(pmsg_fd, "=== MMC KMSG ===\n", 17);

	while ((n = read(fd, buf, sizeof(buf))) > 0) {
		ssize_t i;

		for (i = 0; i < n; i++) {
			char c = buf[i];

			if (c == '\n') {
				line[line_len] = '\0';
				if (strstr(line, "mmc") || strstr(line, "MMC") ||
				    strstr(line, "dwmmc") || strstr(line, "SD ") ||
				    strstr(line, "card")) {
					write_all(pmsg_fd, line, line_len);
					write_all(pmsg_fd, "\n", 1);
				}
				line_len = 0;
			} else if (line_len + 1 < sizeof(line)) {
				line[line_len++] = c;
			}
		}
	}

	close(fd);
	write_all(pmsg_fd, "=== END MMC KMSG ===\n", 21);
}

static int mount_devtmpfs(void)
{
	if (mkdir("/dev", 0755) && errno != EEXIST) {
		char message[LINE_SIZE];

		snprintf(message, sizeof(message), "FAIL: mkdir /dev: %s",
			 strerror(errno));
		report(message);
		return -1;
	}

	if (!mount("devtmpfs", "/dev", "devtmpfs", MS_NOSUID, NULL) ||
	    errno == EBUSY)
		return 0;

	{
		char message[LINE_SIZE];

		snprintf(message, sizeof(message), "FAIL: mount devtmpfs: %s",
			 strerror(errno));
		report(message);
	}
	return -1;
}

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
		goto ioctl_error;
	}

	return 0;

ioctl_error:
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
	unsigned int stride = fb_fix.line_length / sizeof(uint32_t);
	unsigned int row, col;

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
	static const uint8_t colon[7] = { 0, 0x0c, 0x0c, 0, 0x0c, 0x0c, 0 };
	static const uint8_t underscore[7] = { 0, 0, 0, 0, 0, 0, 0x1f };

	if (ch >= 'a' && ch <= 'z')
		ch -= 'a' - 'A';
	if (ch >= 'A' && ch <= 'Z')
		return font_5x7[ch - 'A'];
	if (ch >= '0' && ch <= '9')
		return font_5x7[26 + ch - '0'];
	if (ch == '-')
		return dash;
	if (ch == ':')
		return colon;
	if (ch == '_')
		return underscore;
	return space;
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
				unsigned int repeat_x, repeat_y;

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

static void draw_status(const char *result, bool passed)
{
	fill_rect(32, 148, fb_var.xres - 64, 58, color_header);
	draw_text(40, 160, 3, result,
		  passed ? color_pass : color_fail);
	__sync_synchronize();
}

static void draw_screen(const char *subtitle, const char *result, bool passed)
{
	if (!framebuffer)
		return;

	fill_rect(0, 0, fb_var.xres, fb_var.yres, color_background);
	fill_rect(0, 0, fb_var.xres, 210, color_header);
	draw_text(32, 30, 5, "EXYNOS9611 MSHC", color_text);
	draw_text(36, 112, 3, subtitle, color_muted);
	draw_status(result, passed);
	fill_rect(32, 210, fb_var.xres - 64, 5,
		  passed ? color_pass : color_fail);
	__sync_synchronize();
}

static bool draw_test_row(unsigned int row, const char *text)
{
	unsigned int available_height = fb_var.yres - TEST_ROW_TOP;
	unsigned int row_height = available_height / TEST_ROW_COUNT;
	unsigned int y, scale, text_height;

	if (!framebuffer || row >= TEST_ROW_COUNT || !row_height)
		return false;
	if (row_height > TEST_ROW_MAX_HEIGHT)
		row_height = TEST_ROW_MAX_HEIGHT;
	y = TEST_ROW_TOP + row * row_height;
	if (y + row_height > fb_var.yres)
		return false;

	scale = row_height >= 70 ? 4 : row_height >= 45 ? 3 : 2;
	text_height = 7 * scale;
	fill_rect(32, y, fb_var.xres - 64, row_height,
		  row & 1 ? color_header : color_background);
	draw_text(56, y + (row_height - text_height) / 2, scale, text,
		  color_value);
	__sync_synchronize();
	return true;
}

#ifndef EXYNOS9611_MMC_ENUMERATION_TEST
static int snapshot_registers(void)
{
	char output[LINE_SIZE];
	size_t offset = 0;
	int snapshot_fd;

	snapshot_fd = open("/dev/exynos9611-mshc-snapshot",
			   O_RDONLY | O_CLOEXEC);
	if (snapshot_fd < 0) {
		char message[LINE_SIZE];

		snprintf(message, sizeof(message),
			 "FAIL: open snapshot endpoint: %s",
			 strerror(errno));
		report(message);
		return -1;
	}

	while (offset < sizeof(output) - 1) {
		ssize_t bytes_read;

		do {
			bytes_read = read(snapshot_fd, output + offset,
					  sizeof(output) - 1 - offset);
		} while (bytes_read < 0 && errno == EINTR);

		if (bytes_read < 0) {
			char message[LINE_SIZE];

			snprintf(message, sizeof(message),
				 "FAIL: read snapshot endpoint: %s",
				 strerror(errno));
			report(message);
			close(snapshot_fd);
			return -1;
		}
		if (!bytes_read)
			break;
		offset += bytes_read;
	}
	close(snapshot_fd);
	output[offset] = '\0';
	if (!offset) {
		report("FAIL: empty snapshot endpoint");
		return -1;
	}

	{
		char *line = output;
		unsigned int row = 0;

		while (*line && row < TEST_ROW_COUNT) {
			char *end = strchr(line, '\n');

			if (end)
				*end = '\0';
			if (*line) {
				report(line);
				if (!draw_test_row(row, line)) {
					report("FAIL: register row does not fit framebuffer");
					return -1;
				}
				row++;
			}
			if (!end)
				break;
			line = end + 1;
		}
		if (row != TEST_ROW_COUNT) {
			report("FAIL: snapshot did not return eight registers");
			return -1;
		}
	}

	return 0;
}

int main(void)
{
	if (mount_devtmpfs()) {
		report("TEST RESULT: FAIL");
		for (;;)
			pause();
	}

	kmsg_fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
	pmsg_fd = open("/dev/pmsg0", O_WRONLY | O_CLOEXEC);

	if (init_framebuffer()) {
		report("FAIL: required framebuffer unavailable");
		report("TEST RESULT: FAIL");
		for (;;)
			pause();
	}

	draw_screen("READ ONLY REGISTER SNAPSHOT", "SNAPSHOT RUNNING", true);
	report("Read-only MSHC snapshot; no reset or card I/O");
	if (snapshot_registers()) {
		draw_status("SNAPSHOT FAILED", false);
		report("TEST RESULT: FAIL");
	} else {
		draw_status("SNAPSHOT COMPLETE", true);
		report("TEST RESULT: SNAPSHOT COMPLETE");
	}

	for (;;)
		pause();
}
#else
static int mount_sysfs(void)
{
	if (mkdir("/sys", 0755) && errno != EEXIST) {
		char message[LINE_SIZE];

		snprintf(message, sizeof(message), "FAIL: mkdir sys: %s",
			 strerror(errno));
		report(message);
		return -1;
	}

	if (!mount("sysfs", "/sys", "sysfs",
		   MS_NOSUID | MS_NODEV | MS_NOEXEC, NULL) || errno == EBUSY)
		return 0;

	{
		char message[LINE_SIZE];

		snprintf(message, sizeof(message), "FAIL: mount sysfs: %s",
			 strerror(errno));
		report(message);
	}
	return -1;
}

static int mount_debugfs(void)
{
	if (mkdir("/sys/kernel/debug", 0755) && errno != EEXIST) {
		char message[LINE_SIZE];

		snprintf(message, sizeof(message), "FAIL: mkdir debugfs: %s",
			 strerror(errno));
		report(message);
		return -1;
	}

	if (!mount("debugfs", "/sys/kernel/debug", "debugfs",
		   MS_RDONLY | MS_NOSUID | MS_NODEV | MS_NOEXEC, NULL) ||
	    errno == EBUSY)
		return 0;

	{
		char message[LINE_SIZE];

		snprintf(message, sizeof(message), "FAIL: mount debugfs: %s",
			 strerror(errno));
		report(message);
	}
	return -1;
}

static void sanitize_debug_line(char *line)
{
	char *ch;

	for (ch = line; *ch; ch++) {
		unsigned char value = *ch;

		if (!((value >= 'a' && value <= 'z') ||
		      (value >= 'A' && value <= 'Z') ||
		      (value >= '0' && value <= '9') ||
		      value == ' ' || value == '-' || value == '_' ||
		      value == ':' || value == '.'))
			*ch = ' ';
	}
}

static bool read_matching_line(const char *path, const char *needle,
			       char *output, size_t output_size)
{
	char line[LINE_SIZE];
	FILE *file;

	file = fopen(path, "r");
	if (!file) {
		snprintf(output, output_size, "DEBUG DATA UNAVAILABLE");
		return false;
	}

	while (fgets(line, sizeof(line), file)) {
		char *end = strpbrk(line, "\r\n");
		int length;

		if (end)
			*end = '\0';
		if (!strstr(line, needle))
			continue;

		sanitize_debug_line(line);
		length = snprintf(output, output_size, "%s", line);
		fclose(file);
		return length >= 0 && (size_t)length < output_size;
	}

	fclose(file);
	snprintf(output, output_size, "MATCHING DATA NOT FOUND");
	return false;
}

static void read_gpio_debug_state(char *chip_output, size_t chip_output_size,
				  char *cd_output, size_t cd_output_size)
{
	char line[LINE_SIZE];
	FILE *file;
	bool chip_found = false;
	bool cd_found = false;

	file = fopen("/sys/kernel/debug/gpio", "r");
	if (!file) {
		snprintf(chip_output, chip_output_size, "GPIO DEBUG UNAVAILABLE");
		snprintf(cd_output, cd_output_size, "CD GPIO STATE UNKNOWN");
		return;
	}

	while (fgets(line, sizeof(line), file)) {
		char *end = strpbrk(line, "\r\n");

		if (end)
			*end = '\0';
		if (!chip_found && strstr(line, "gpa2")) {
			snprintf(chip_output, chip_output_size,
				 "GPIO CHIP: GPA2 AVAILABLE");
			chip_found = true;
		}
		if (strstr(line, "13550000")) {
			snprintf(cd_output, cd_output_size,
				 "CD GPIO: CLAIMED BY MMC HOST");
			cd_found = true;
		}
	}

	fclose(file);

	if (!chip_found)
		snprintf(chip_output, chip_output_size,
			 "GPIO CHIP: GPA2 NOT FOUND");
	if (!cd_found)
		snprintf(cd_output, cd_output_size,
			 "CD GPIO: NOT CLAIMED BY MMC HOST");
}

static bool read_cd_pinmux_line(char *output, size_t output_size)
{
	DIR *pinctrl;
	struct dirent *entry;
	char path[LINE_SIZE];

	pinctrl = opendir("/sys/kernel/debug/pinctrl");
	if (!pinctrl) {
		snprintf(output, output_size, "PINMUX DEBUG UNAVAILABLE");
		return false;
	}

	while ((entry = readdir(pinctrl))) {
		int length;

		if (entry->d_name[0] == '.')
			continue;
		length = snprintf(path, sizeof(path),
				  "/sys/kernel/debug/pinctrl/%s/pinmux-pins",
				  entry->d_name);
		if (length < 0 || (size_t)length >= sizeof(path))
			continue;
		if (read_matching_line(path, "gpa2-4", output, output_size)) {
			closedir(pinctrl);
			return true;
		}
	}

	closedir(pinctrl);
	snprintf(output, output_size, "CD PINMUX ENTRY NOT FOUND");
	return false;
}

static bool is_mmc_host(const char *name)
{
	const char *digit;

	if (strncmp(name, "mmc", 3))
		return false;

	digit = name + 3;
	if (*digit < '0' || *digit > '9')
		return false;
	while (*digit >= '0' && *digit <= '9')
		digit++;

	return !*digit;
}

static bool find_card(char *host, size_t host_size, char *card_path,
		      size_t card_path_size, bool *host_found)
{
	DIR *hosts;
	struct dirent *host_entry;

	hosts = opendir("/sys/class/mmc_host");
	if (!hosts)
		return false;

	while ((host_entry = readdir(hosts))) {
		DIR *host_dir;
		struct dirent *card_entry;
		char host_path[LINE_SIZE];
		int path_length;

		if (!is_mmc_host(host_entry->d_name))
			continue;
		path_length = snprintf(host_path, sizeof(host_path),
				       "/sys/class/mmc_host/%s",
				       host_entry->d_name);
		if (path_length < 0 ||
		    (size_t)path_length >= sizeof(host_path))
			continue;

		host_dir = opendir(host_path);
		if (!host_dir)
			continue;
		*host_found = true;

		if (!host[0]) {
			path_length = snprintf(host, host_size, "%s",
					       host_entry->d_name);
			if (path_length < 0 ||
			    (size_t)path_length >= host_size) {
				closedir(host_dir);
				continue;
			}
		}

		while ((card_entry = readdir(host_dir))) {
			if (strncmp(card_entry->d_name, host_entry->d_name,
				    strlen(host_entry->d_name)) ||
			    card_entry->d_name[strlen(host_entry->d_name)] != ':')
				continue;

			path_length = snprintf(card_path, card_path_size, "%s/%s",
					       host_path, card_entry->d_name);
			if (path_length < 0 ||
			    (size_t)path_length >= card_path_size)
				continue;
			closedir(host_dir);
			closedir(hosts);
			return true;
		}
		closedir(host_dir);
	}

	closedir(hosts);
	return false;
}

static void read_card_attribute(const char *card_path, const char *attribute,
				char *value, size_t value_size)
{
	char path[LINE_SIZE];
	ssize_t bytes_read;
	int fd;
	int path_length;

	path_length = snprintf(path, sizeof(path), "%s/%s", card_path,
			       attribute);
	if (path_length < 0 || (size_t)path_length >= sizeof(path)) {
		snprintf(value, value_size, "UNKNOWN");
		return;
	}

	fd = open(path, O_RDONLY | O_CLOEXEC);
	if (fd < 0) {
		snprintf(value, value_size, "UNKNOWN");
		return;
	}

	do {
		bytes_read = read(fd, value, value_size - 1);
	} while (bytes_read < 0 && errno == EINTR);
	close(fd);
	if (bytes_read <= 0) {
		snprintf(value, value_size, "UNKNOWN");
		return;
	}

	value[bytes_read] = '\0';
	value[strcspn(value, "\r\n")] = '\0';
}

static void show_card_info(const char *host, const char *card_path)
{
	static const char * const attributes[] = {
		"type", "name", "manfid", "oemid", "serial",
	};
	static const char * const labels[] = {
		"CARD TYPE: ", "CARD NAME: ", "MANUFACTURER ID: ",
		"OEM ID: ", "SERIAL: ",
	};
	char value[LINE_SIZE];
	char line[LINE_SIZE];
	unsigned int i;

	snprintf(line, sizeof(line), "HOST: %.248s", host);
	draw_test_row(0, line);
	report(line);

	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		size_t j;

		read_card_attribute(card_path, attributes[i], value,
				    sizeof(value));
		for (j = 0; value[j]; j++) {
			unsigned char ch = value[j];

			if (!((ch >= 'a' && ch <= 'z') ||
			      (ch >= 'A' && ch <= 'Z') ||
			      (ch >= '0' && ch <= '9') ||
			      ch == ' ' || ch == '-' || ch == '_' ||
			      ch == ':' || ch == '.'))
				value[j] = ' ';
		}
		snprintf(line, sizeof(line), "%s%.200s", labels[i], value);
		draw_test_row(i + 1, line);
		report(line);
	}

	draw_test_row(6, "BLOCK IO DISABLED");
	draw_test_row(7, "NO SECTOR ACCESS");
}

static void show_card_failure_diagnostics(const char *host)
{
	static const char * const attributes[] = {
		"clock:", "power mode:", "bus width:",
		"Command Timeout", "Command CRC",
	};
	char path[LINE_SIZE];
	char line[LINE_SIZE];
	char gpio_line[LINE_SIZE];
	unsigned int i;

	snprintf(line, sizeof(line), "HOST REGISTERED %.230s", host);
	draw_test_row(0, line);
	report(line);

	read_gpio_debug_state(line, sizeof(line), gpio_line,
			      sizeof(gpio_line));
	draw_test_row(1, line);
	report(line);
	draw_test_row(2, gpio_line);
	report(gpio_line);

	read_cd_pinmux_line(line, sizeof(line));
	draw_test_row(3, line);
	report(line);

	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		const char *file = i < 3 ? "ios" : "err_stats";
		int length;

		length = snprintf(path, sizeof(path),
				  "/sys/kernel/debug/%s/%s", host, file);
		if (length < 0 || (size_t)length >= sizeof(path) ||
		    !read_matching_line(path, attributes[i], line,
					sizeof(line)))
			snprintf(line, sizeof(line), "%s UNAVAILABLE",
				 i < 3 ? "HOST IOS" : "MMC ERROR COUNTER");

		draw_test_row(i + 4, line);
		report(line);
	}

	report("BLOCK IO DISABLED");
}

int main(void)
{
	unsigned int elapsed;
	bool host_found = false;
	char host[LINE_SIZE] = "";
	char card_path[LINE_SIZE] = "";

	if (mount_devtmpfs()) {
		draw_status("DEV SETUP FAILED", false);
		for (;;)
			pause();
	}

	kmsg_fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
	pmsg_fd = open("/dev/pmsg0", O_WRONLY | O_CLOEXEC);

	if (init_framebuffer()) {
		report("FAIL: required framebuffer unavailable");
		for (;;)
			pause();
	}

	draw_screen("FORCED SD IDENTIFICATION", "STARTING", true);
	draw_test_row(0, "MMC BLOCK IO DISABLED");
	draw_test_row(1, "NO SECTOR ACCESS");
	report("MMC identification only; no block or sector I/O");

	if (mount_sysfs()) {
		draw_test_row(0, "SYSFS SETUP FAILED");
		draw_status("TEST RESULT FAIL", false);
		report("TEST RESULT: FAIL");
		for (;;)
			pause();
	}

	for (elapsed = 0; elapsed < 30; elapsed++) {
		if (find_card(host, sizeof(host), card_path,
			      sizeof(card_path), &host_found))
			break;

		draw_status(host_found ? "WAITING FOR CARD" :
			    "WAITING FOR HOST", true);
		sleep(1);
	}

	if (elapsed == 30) {
		draw_test_row(0, host_found ? "CARD NOT FOUND" :
			       "HOST NOT PROBED");
		draw_status("TEST RESULT FAIL", false);
		report(host_found ? "FAIL: card was not enumerated" :
		       "FAIL: MMC host was not registered");
		if (host_found) {
			if (mount_debugfs())
				report("FAIL: unable to mount debugfs for diagnostics");
			show_card_failure_diagnostics(host);
		}
		dump_mmc_kmsg();
		report("TEST RESULT: FAIL");
		for (;;)
			pause();
	}

	show_card_info(host, card_path);
	draw_status("TEST RESULT PASS", true);
	dump_mmc_kmsg();
	report("TEST RESULT: PASS");

	for (;;)
		pause();
}
#endif
