// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal PID 1 for a read-only SM5713 I2C0 transfer test.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define I2C_ADAPTER_NAME "/sys/bus/i2c/devices/i2c-0/name"
#define I2C_PLATFORM_DRIVER "/sys/bus/platform/devices/13830000.i2c/driver"
#define I2C_DEVICE "/dev/i2c-0"
#define SM5713_ADDR 0x33
#define SM5713_REG_PROBE0 0xd0
#define LINE_SIZE 256

static int console_fd = -1;
static int kmsg_fd = -1;

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

	if (kmsg_fd < 0)
		return;

	len = snprintf(kmsg, sizeof(kmsg), "<6>Exynos9611-I2C: %s\n", message);
	if (len >= sizeof(kmsg) || write_all(kmsg_fd, kmsg, len)) {
		static const char warning[] =
			"WARN: /dev/kmsg write failed; pstore console logging stopped\n";

		write_all(output_fd, warning, sizeof(warning) - 1);
		close(kmsg_fd);
		kmsg_fd = -1;
	}
}

static int mount_fs(const char *source, const char *target,
		    const char *type)
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

static int wait_for_i2c0(void)
{
	struct stat status;
	char name[LINE_SIZE];
	unsigned int attempt;

	for (attempt = 0; attempt < 10; attempt++) {
		FILE *name_file = fopen(I2C_ADAPTER_NAME, "r");

		if (name_file) {
			if (fgets(name, sizeof(name), name_file)) {
				name[strcspn(name, "\r\n")] = '\0';
				fclose(name_file);
				if (!strcmp(name, "s3c2410-i2c") &&
				    !stat(I2C_PLATFORM_DRIVER, &status)) {
					report("PASS: I2C0 adapter and driver are ready");
					return 0;
				}
			} else {
				fclose(name_file);
			}
		}

		sleep(1);
	}

	report("FAIL: expected S3C I2C0 adapter or /dev/i2c-0 is unavailable");
	return -1;
}

static int read_sm5713_probe_status(void)
{
	struct i2c_smbus_ioctl_data transfer = {
		.read_write = I2C_SMBUS_READ,
		.command = SM5713_REG_PROBE0,
		.size = I2C_SMBUS_BYTE_DATA,
	};
	union i2c_smbus_data data = {};
	unsigned long funcs;
	char message[LINE_SIZE];
	int fd;

	fd = open(I2C_DEVICE, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		snprintf(message, sizeof(message), "FAIL: open %s: %s",
			 I2C_DEVICE, strerror(errno));
		report(message);
		return -1;
	}

	if (ioctl(fd, I2C_FUNCS, &funcs) < 0) {
		snprintf(message, sizeof(message), "FAIL: query I2C0 functions: %s",
			 strerror(errno));
		report(message);
		close(fd);
		return -1;
	}

	if (!(funcs & I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
		report("FAIL: I2C0 lacks SMBus byte-data read support");
		close(fd);
		return -1;
	}

	if (ioctl(fd, I2C_SLAVE, SM5713_ADDR) < 0) {
		snprintf(message, sizeof(message),
			 "FAIL: select SM5713 address 0x%02x: %s",
			 SM5713_ADDR, strerror(errno));
		report(message);
		close(fd);
		return -1;
	}

	transfer.data = &data;
	if (ioctl(fd, I2C_SMBUS, &transfer) < 0) {
		snprintf(message, sizeof(message),
			 "FAIL: read SM5713 register 0x%02x: %s",
			 SM5713_REG_PROBE0, strerror(errno));
		report(message);
		close(fd);
		return -1;
	}

	snprintf(message, sizeof(message),
		 "PASS: SM5713 read-only register 0x%02x returned 0x%02x",
		 SM5713_REG_PROBE0, data.byte);
	report(message);
	close(fd);
	return 0;
}

static void wait_forever(void)
{
	for (;;)
		pause();
}

int main(void)
{
	int result;

	report("Exynos9611 I2C0 read-only transfer test started");

	if (mount_fs("devtmpfs", "/dev", "devtmpfs") ||
	    mount_fs("sysfs", "/sys", "sysfs")) {
		report("TEST RESULT: FAIL");
		wait_forever();
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

	if (wait_for_i2c0()) {
		report("TEST RESULT: FAIL");
		wait_forever();
	}

	result = read_sm5713_probe_status();
	report(result ? "TEST RESULT: FAIL" : "TEST RESULT: PASS");
	wait_forever();
}
