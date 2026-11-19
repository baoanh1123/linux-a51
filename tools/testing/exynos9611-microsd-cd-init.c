// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only Exynos9611 A51 microSD tray card-detect GPIO test.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define GPIOCHIP_COUNT 64
#define GPIO_OFFSET 4
#define SAMPLE_COUNT 45
#define LINE_SIZE 256

static int console_fd = -1;
static int kmsg_fd = -1;
static int pmsg_fd = -1;

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
	int output_fd = console_fd >= 0 ? console_fd : STDOUT_FILENO;

	write_all(output_fd, message, len);
	write_all(output_fd, "\n", 1);

	if (kmsg_fd >= 0) {
		int kmsg_len = snprintf(kmsg, sizeof(kmsg),
					"<6>Exynos9611-microSD-CD: %s\n",
					message);

		if (kmsg_len < 0 || (size_t)kmsg_len >= sizeof(kmsg) ||
		    write_all(kmsg_fd, kmsg, kmsg_len)) {
			static const char warning[] =
				"WARN: /dev/kmsg write failed\n";

			write_all(output_fd, warning, sizeof(warning) - 1);
			close(kmsg_fd);
			kmsg_fd = -1;
		}
	}

	if (pmsg_fd >= 0 &&
	    (write_all(pmsg_fd, message, len) || write_all(pmsg_fd, "\n", 1))) {
		static const char warning[] =
			"WARN: /dev/pmsg0 write failed\n";

		write_all(output_fd, warning, sizeof(warning) - 1);
		close(pmsg_fd);
		pmsg_fd = -1;
	}
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

static int open_gpa2_chip(void)
{
	unsigned int i;

	for (i = 0; i < GPIOCHIP_COUNT; i++) {
		struct gpiochip_info info = {};
		char path[LINE_SIZE];
		int fd;

		snprintf(path, sizeof(path), "/dev/gpiochip%u", i);
		fd = open(path, O_RDONLY | O_CLOEXEC);
		if (fd < 0) {
			if (errno == ENOENT)
				continue;
			{
				char message[LINE_SIZE];

				snprintf(message, sizeof(message),
					 "FAIL: open gpiochip%u: %.160s",
					 i, strerror(errno));
				report(message);
			}
			return -1;
		}

		if (!ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &info) &&
		    !strcmp(info.label, "gpa2")) {
			if (info.lines <= GPIO_OFFSET) {
				report("FAIL: gpa2 GPIO bank has no line 4");
				close(fd);
				return -1;
			}

			report("PASS: found gpa2 GPIO bank");
			return fd;
		}

		close(fd);
	}

	report("FAIL: gpa2 GPIO chip not found");
	return -1;
}

static int request_cd_line(int chip_fd)
{
	struct gpio_v2_line_request request = {
		.offsets = { GPIO_OFFSET },
		.consumer = "a51-microsd-cd-test",
		.num_lines = 1,
		.config = {
			.flags = GPIO_V2_LINE_FLAG_INPUT,
		},
	};

	if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request)) {
		char message[LINE_SIZE];

		snprintf(message, sizeof(message),
			 "FAIL: request gpa2-4 input: %s", strerror(errno));
		report(message);
		return -1;
	}

	return request.fd;
}

static int read_cd_line(int line_fd, unsigned int *value)
{
	struct gpio_v2_line_values values = {
		.mask = 1,
	};

	if (ioctl(line_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &values))
		return -1;

	*value = values.bits & 1;
	return 0;
}

static void wait_one_second(void)
{
	struct timespec delay = {
		.tv_sec = 1,
	};

	while (nanosleep(&delay, &delay) && errno == EINTR)
		;
}

int main(void)
{
	unsigned int previous, current, transitions = 0, i;
	int chip_fd, line_fd;

	if (mount_devtmpfs()) {
		report("TEST RESULT: FAIL");
		for (;;)
			pause();
	}

	console_fd = open("/dev/console", O_WRONLY | O_NOCTTY);
	if (console_fd < 0)
		report("WARN: cannot open /dev/console");

	kmsg_fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
	if (kmsg_fd < 0)
		report("WARN: cannot open /dev/kmsg");

	pmsg_fd = open("/dev/pmsg0", O_WRONLY | O_CLOEXEC);
	if (pmsg_fd < 0)
		report("WARN: cannot open /dev/pmsg0");

	report("A51 microSD card-detect GPIO test; no MMC host or card I/O");
	report("Remove and reinsert the card during the next 45 seconds");

	chip_fd = open_gpa2_chip();
	if (chip_fd < 0)
		goto failed;

	line_fd = request_cd_line(chip_fd);
	close(chip_fd);
	if (line_fd < 0)
		goto failed;

	if (read_cd_line(line_fd, &previous)) {
		report("FAIL: cannot read gpa2-4");
		close(line_fd);
		goto failed;
	}

	{
		char message[LINE_SIZE];

		snprintf(message, sizeof(message),
			 "Initial gpa2-4 raw level: %u", previous);
		report(message);
	}

	for (i = 0; i < SAMPLE_COUNT; i++) {
		wait_one_second();
		if (read_cd_line(line_fd, &current)) {
			report("FAIL: cannot sample gpa2-4");
			close(line_fd);
			goto failed;
		}

		if (current == previous)
			continue;

		transitions++;
		previous = current;
		{
			char message[LINE_SIZE];

			snprintf(message, sizeof(message),
				 "gpa2-4 changed to raw level %u", current);
			report(message);
		}
	}

	close(line_fd);
	if (transitions) {
		report("PASS: card-detect input changed state");
		report("TEST RESULT: PASS");
	} else {
		report("INCONCLUSIVE: no card-detect transition observed");
		report("TEST RESULT: INCONCLUSIVE");
	}

	for (;;)
		pause();

failed:
	report("TEST RESULT: FAIL");
	for (;;)
		pause();
}
