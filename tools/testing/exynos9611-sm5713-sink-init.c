// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal PID 1 for validating the SM5713 sink-only CC configuration.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <unistd.h>

#define I2C_ADAPTER_NAME "/sys/bus/i2c/devices/i2c-0/name"
#define I2C_PLATFORM_DRIVER "/sys/bus/platform/devices/13830000.i2c/driver"
#define SM5713_I2C_DEVICE "/dev/i2c-0"
#define SM5713_CLIENT_PATH "/sys/bus/i2c/devices/0-0033"
#define SM5713_ADDR 0x33
#define SM5713_REG_STATUS1 0x0b
#define SM5713_REG_CC_CNTL1 0x29
#define SM5713_REG_CC_CNTL3 0x2b
#define SM5713_REG_PD_STATE0 0xd5
#define SM5713_CC_ATTACH (1U << 3)
#define SM5713_CC_SINK_ONLY 0x84
#define SM5713_CC_UNATTACHED_SINK 0x81
#define SM5713_CC_OPERATE 0x80
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

static void report(const char *format, ...)
{
	char message[LINE_SIZE];
	char kmsg[LINE_SIZE + 32];
	va_list args;
	int output_fd = console_fd >= 0 ? console_fd : STDOUT_FILENO;
	int len;

	va_start(args, format);
	len = vsnprintf(message, sizeof(message), format, args);
	va_end(args);
	if (len < 0)
		return;

	len = strnlen(message, sizeof(message));
	write_all(output_fd, message, len);
	write_all(output_fd, "\n", 1);

	if (kmsg_fd < 0)
		return;

	len = snprintf(kmsg, sizeof(kmsg), "<6>SM5713-sink: %.*s\n",
		       (int)strnlen(message, sizeof(message)), message);
	if (len < 0 || (size_t)len >= sizeof(kmsg) ||
	    write_all(kmsg_fd, kmsg, len)) {
		static const char warning[] =
			"WARN: /dev/kmsg write failed; pstore logging stopped\n";

		write_all(output_fd, warning, sizeof(warning) - 1);
		close(kmsg_fd);
		kmsg_fd = -1;
	}
}

static int mount_fs(const char *source, const char *target,
		    const char *type)
{
	if (mkdir(target, 0755) && errno != EEXIST) {
		report("FAIL: mkdir %s: %s", target, strerror(errno));
		return -1;
	}

	if (!mount(source, target, type, MS_NOSUID, NULL) || errno == EBUSY)
		return 0;

	report("FAIL: mount %s: %s", type, strerror(errno));
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

	report("FAIL: expected S3C I2C0 adapter or driver is unavailable");
	return -1;
}

static int sm5713_smbus(int fd, bool read, unsigned char reg,
			unsigned char *value)
{
	struct i2c_smbus_ioctl_data transfer = {
		.read_write = read ? I2C_SMBUS_READ : I2C_SMBUS_WRITE,
		.command = reg,
		.size = I2C_SMBUS_BYTE_DATA,
	};
	union i2c_smbus_data data = {};
	int ret;

	if (!read)
		data.byte = *value;
	transfer.data = &data;

	ret = ioctl(fd, I2C_SMBUS, &transfer);
	if (ret < 0)
		return -1;

	if (read)
		*value = data.byte;

	return 0;
}

static int read_sm5713_reg(int fd, unsigned char reg, unsigned char *value)
{
	if (sm5713_smbus(fd, true, reg, value)) {
		report("FAIL: read SM5713 register 0x%02x: %s",
		       reg, strerror(errno));
		return -1;
	}

	return 0;
}

static int write_sm5713_reg(int fd, unsigned char reg, unsigned char value)
{
	if (sm5713_smbus(fd, false, reg, &value)) {
		report("FAIL: write SM5713 register 0x%02x: %s",
		       reg, strerror(errno));
		return -1;
	}

	return 0;
}

static int configure_sink_only(void)
{
	unsigned char status1, pd_state0, cc_cntl1, cc_cntl3;
	unsigned char expected_cc3;
	unsigned long functions;
	int fd;

	if (!access(SM5713_CLIENT_PATH, F_OK)) {
		report("FAIL: SM5713 has a kernel I2C client; refusing raw register writes");
		return -1;
	}
	if (errno != ENOENT) {
		report("FAIL: inspect SM5713 I2C client path: %s", strerror(errno));
		return -1;
	}

	fd = open(SM5713_I2C_DEVICE, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		report("FAIL: open %s: %s", SM5713_I2C_DEVICE, strerror(errno));
		return -1;
	}

	if (ioctl(fd, I2C_FUNCS, &functions) < 0) {
		report("FAIL: query I2C0 functions: %s", strerror(errno));
		close(fd);
		return -1;
	}
	if (!(functions & I2C_FUNC_SMBUS_READ_BYTE_DATA) ||
	    !(functions & I2C_FUNC_SMBUS_WRITE_BYTE_DATA)) {
		report("FAIL: I2C0 lacks SMBus byte-data read/write support");
		close(fd);
		return -1;
	}
	if (ioctl(fd, I2C_SLAVE, SM5713_ADDR) < 0) {
		report("FAIL: select SM5713 address 0x%02x: %s",
		       SM5713_ADDR, strerror(errno));
		close(fd);
		return -1;
	}

	if (read_sm5713_reg(fd, SM5713_REG_CC_CNTL1, &cc_cntl1) ||
	    read_sm5713_reg(fd, SM5713_REG_CC_CNTL3, &cc_cntl3) ||
	    read_sm5713_reg(fd, SM5713_REG_STATUS1, &status1) ||
	    read_sm5713_reg(fd, SM5713_REG_PD_STATE0, &pd_state0)) {
		close(fd);
		return -1;
	}

	report("INFO: initial CC_CNTL1=0x%02x CC_CNTL3=0x%02x STATUS1=0x%02x PD_STATE0=0x%02x",
	       cc_cntl1, cc_cntl3, status1, pd_state0);

	if (write_sm5713_reg(fd, SM5713_REG_CC_CNTL1, SM5713_CC_SINK_ONLY)) {
		close(fd);
		return -1;
	}

	if (!(status1 & SM5713_CC_ATTACH)) {
		expected_cc3 = (pd_state0 & 0x0f) == 0x01 ?
			SM5713_CC_UNATTACHED_SINK : SM5713_CC_OPERATE;
		if (write_sm5713_reg(fd, SM5713_REG_CC_CNTL3, expected_cc3)) {
			close(fd);
			return -1;
		}
	} else {
		expected_cc3 = cc_cntl3;
		report("INFO: cable already attached; preserving CC_CNTL3");
	}

	if (read_sm5713_reg(fd, SM5713_REG_CC_CNTL1, &cc_cntl1) ||
	    read_sm5713_reg(fd, SM5713_REG_CC_CNTL3, &cc_cntl3)) {
		close(fd);
		return -1;
	}
	close(fd);

	if (cc_cntl1 != SM5713_CC_SINK_ONLY ||
	    cc_cntl3 != expected_cc3) {
		report("FAIL: CC readback mismatch (CC_CNTL1=0x%02x CC_CNTL3=0x%02x)",
		       cc_cntl1, cc_cntl3);
		return -1;
	}

	report("PASS: SM5713 sink-only CC configuration read back (CC_CNTL1=0x%02x CC_CNTL3=0x%02x)",
	       cc_cntl1, cc_cntl3);
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

	report("Exynos9611 SM5713 sink-only test started");

	if (mount_fs("devtmpfs", "/dev", "devtmpfs") ||
	    mount_fs("sysfs", "/sys", "sysfs")) {
		report("TEST RESULT: FAIL");
		wait_forever();
	}

	console_fd = open("/dev/console", O_WRONLY | O_NOCTTY);
	if (console_fd < 0)
		report("WARN: cannot open /dev/console: %s", strerror(errno));

	kmsg_fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
	if (kmsg_fd < 0)
		report("WARN: cannot open /dev/kmsg: %s", strerror(errno));

	if (wait_for_i2c0()) {
		report("TEST RESULT: FAIL");
		wait_forever();
	}

	result = configure_sink_only();
	report("TEST RESULT: %s", result ? "FAIL" : "PASS");
	wait_forever();
}
