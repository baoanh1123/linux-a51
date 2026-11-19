// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal PID 1 for validating Exynos9611 I2C0 controller probe.
 */

#define _GNU_SOURCE

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <unistd.h>

#define I2C_DEVICES "/sys/bus/i2c/devices"
#define I2C_PLATFORM_DEVICE "/sys/bus/platform/devices/13830000.i2c"
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
					"<6>Exynos9611-I2C: %s\n", message);

		if (kmsg_len < 0 || (size_t)kmsg_len >= sizeof(kmsg) ||
		    write_all(kmsg_fd, kmsg, kmsg_len)) {
			static const char warning[] =
				"WARN: /dev/kmsg failed; pstore console logging stopped\n";

			write_all(output_fd, warning, sizeof(warning) - 1);
			close(kmsg_fd);
			kmsg_fd = -1;
		}
	}

	if (pmsg_fd >= 0 &&
	    (write_all(pmsg_fd, message, len) || write_all(pmsg_fd, "\n", 1))) {
		static const char warning[] =
			"WARN: pmsg write failed; test output is no longer saved\n";

		write_all(output_fd, warning, sizeof(warning) - 1);
		close(pmsg_fd);
		pmsg_fd = -1;
	}
}

static int mount_fs(const char *source, const char *target, const char *type)
{
	if (mkdir(target, 0755) && errno != EEXIST) {
		char message[LINE_SIZE];

		snprintf(message, sizeof(message), "FAIL: mkdir %s: %s",
			 target, strerror(errno));
		report(message);
		return -1;
	}

	if (!mount(source, target, type, MS_NOSUID, NULL) || errno == EBUSY)
		return 0;

	{
		char message[LINE_SIZE];

		snprintf(message, sizeof(message), "FAIL: mount %s: %s",
			 type, strerror(errno));
		report(message);
	}
	return -1;
}

static bool is_i2c_adapter_name(const char *name)
{
	const char *digit;

	if (strncmp(name, "i2c-", 4))
		return false;
	digit = name + 4;
	if (!*digit)
		return false;

	while (*digit) {
		if (*digit < '0' || *digit > '9')
			return false;
		digit++;
	}

	return true;
}

static bool check_i2c_adapter(void)
{
	struct dirent *entry;
	DIR *devices;
	char path[LINE_SIZE];
	char name[LINE_SIZE];
	FILE *name_file;

	devices = opendir(I2C_DEVICES);
	if (!devices)
		return false;

	while ((entry = readdir(devices))) {
		int path_len;

		if (!is_i2c_adapter_name(entry->d_name))
			continue;

		path_len = snprintf(path, sizeof(path), "%s/%s/name",
				    I2C_DEVICES, entry->d_name);
		if (path_len < 0 || (size_t)path_len >= sizeof(path))
			continue;

		name_file = fopen(path, "r");
		if (!name_file)
			continue;

		if (!fgets(name, sizeof(name), name_file)) {
			fclose(name_file);
			continue;
		}
		fclose(name_file);
		name[strcspn(name, "\r\n")] = '\0';

		if (strstr(name, "s3c2410-i2c")) {
			closedir(devices);
			return true;
		}
	}

	closedir(devices);
	return false;
}

int main(void)
{
	struct stat status;
	bool driver_bound = false;
	bool adapter_registered = false;
	unsigned int attempt;

	if (mount_fs("devtmpfs", "/dev", "devtmpfs")) {
		report("TEST RESULT: FAIL");
		for (;;)
			pause();
	}

	console_fd = open("/dev/console", O_WRONLY | O_NOCTTY);
	if (console_fd < 0)
		report("WARN: cannot open /dev/console");

	kmsg_fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
	if (kmsg_fd < 0) {
		char message[LINE_SIZE];

		snprintf(message, sizeof(message), "WARN: cannot open /dev/kmsg: %s",
			 strerror(errno));
		report(message);
	}

	pmsg_fd = open("/dev/pmsg0", O_WRONLY | O_CLOEXEC);
	if (pmsg_fd < 0) {
		char message[LINE_SIZE];

		snprintf(message, sizeof(message), "WARN: cannot open /dev/pmsg0: %s",
			 strerror(errno));
		report(message);
	}

	report("Exynos9611 I2C0 probe test started; no bus transactions");
	if (mount_fs("sysfs", "/sys", "sysfs")) {
		report("TEST RESULT: FAIL");
		for (;;)
			pause();
	}

	for (attempt = 0; attempt < 10; attempt++) {
		driver_bound = !stat(I2C_PLATFORM_DEVICE "/driver", &status);
		adapter_registered = check_i2c_adapter();
		if (driver_bound && adapter_registered)
			break;
		sleep(1);
	}

	if (driver_bound)
		report("PASS: I2C0 platform driver is bound");
	else
		report("FAIL: I2C0 platform driver is not bound");

	if (adapter_registered)
		report("PASS: S3C I2C adapter registered");
	else
		report("FAIL: no S3C I2C adapter registered");

	report(driver_bound && adapter_registered ?
	       "TEST RESULT: PASS" : "TEST RESULT: FAIL");

	for (;;)
		pause();
}
