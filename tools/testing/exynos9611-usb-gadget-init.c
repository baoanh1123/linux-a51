// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos9611 A51 USB CDC ACM gadget test init.
 *
 * Mounts the SD card read-write, configures a minimal CDC ACM gadget on the
 * first available UDC, waits for host enumeration, and leaves /dev/ttyGS0
 * available for the host. Boot diagnostics are saved under /mnt/pi-debug/.
 */

#define _GNU_SOURCE

#include <dirent.h>
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
#include <sys/statfs.h>
#include <unistd.h>

#define LINE_SIZE	512
#define COPY_BUF_SIZE	(64 * 1024)
#define KMSG_MAX	(4 * 1024 * 1024)

static int kmsg_fd = -1;
static int pmsg_fd = -1;
static char logdir[LINE_SIZE] = "/mnt/pi-debug";

static int save_one(const char *src, const char *name, const char *dir,
		    bool fatal);
static int read_text_value(const char *path, char *buf, size_t len);

static void report(const char *fmt, ...)
{
	char message[LINE_SIZE];
	char kmsg[LINE_SIZE + 8];
	va_list args;
	int len;

	va_start(args, fmt);
	vsnprintf(message, sizeof(message), fmt, args);
	va_end(args);

	if (kmsg_fd >= 0) {
		len = snprintf(kmsg, sizeof(kmsg), "<6>Exynos9611-SD-log: %s\n",
			       message);
		if (len > 0 && write(kmsg_fd, kmsg, len) < 0) {
			close(kmsg_fd);
			kmsg_fd = -1;
		}
	}

	if (pmsg_fd >= 0) {
		len = strlen(message);
		if (write(pmsg_fd, message, len) < 0 ||
		    write(pmsg_fd, "\n", 1) < 0) {
			close(pmsg_fd);
			pmsg_fd = -1;
		}
	}
}

static void make_dirs(void)
{
	mkdir("/dev", 0755);
	mkdir("/proc", 0755);
	mkdir("/sys", 0755);
	mkdir("/sys/fs", 0755);
	mkdir("/sys/fs/pstore", 0755);
	mkdir("/sys/kernel", 0755);
	mkdir("/sys/kernel/debug", 0755);
	mkdir("/mnt", 0755);
}

static void mount_devtmpfs(void)
{
	if (mount("devtmpfs", "/dev", "devtmpfs", MS_NOSUID, NULL) &&
	    errno != EBUSY)
		printf("devtmpfs mount failed: %s\n", strerror(errno));
}

static void mount_procfs(void)
{
	if (mount("proc", "/proc", "proc", MS_NOSUID | MS_NODEV | MS_NOEXEC,
		  NULL) && errno != EBUSY)
		printf("procfs mount failed: %s\n", strerror(errno));
}

static void mount_sysfs(void)
{
	if (mount("sysfs", "/sys", "sysfs", MS_NOSUID | MS_NODEV | MS_NOEXEC,
		  NULL) && errno != EBUSY)
		printf("sysfs mount failed: %s\n", strerror(errno));
}

static void mount_debugfs(void)
{
	if (mount("debugfs", "/sys/kernel/debug", "debugfs",
		  MS_NOSUID | MS_NODEV | MS_NOEXEC, NULL) && errno != EBUSY)
		printf("debugfs mount failed: %s\n", strerror(errno));
}

static void mount_configfs(void)
{
	mkdir("/sys/kernel/config", 0755);
	if (mount("configfs", "/sys/kernel/config", "configfs",
		  MS_NOSUID | MS_NODEV | MS_NOEXEC, NULL) && errno != EBUSY)
		printf("configfs mount failed: %s\n", strerror(errno));
}

static int write_attr(const char *path, const char *value)
{
	int fd;
	ssize_t len = strlen(value);
	ssize_t n;

	fd = open(path, O_WRONLY | O_CLOEXEC);
	if (fd < 0)
		return -1;
	n = write(fd, value, len);
	close(fd);
	return n == len ? 0 : -1;
}

static int make_dir(const char *path)
{
	if (mkdir(path, 0755) && errno != EEXIST)
		return -1;
	return 0;
}

static void usb_step(const char *dir, const char *fmt, ...)
{
	char message[LINE_SIZE];
	char path[LINE_SIZE * 2];
	va_list args;
	int fd;

	va_start(args, fmt);
	vsnprintf(message, sizeof(message), fmt, args);
	va_end(args);

	report("%s", message);
	snprintf(path, sizeof(path), "%s/usb-setup.log", dir);
	fd = open(path, O_CREAT | O_APPEND | O_WRONLY | O_CLOEXEC, 0644);
	if (fd >= 0) {
		dprintf(fd, "%s\n", message);
		fsync(fd);
		close(fd);
	}
}

#define SM5713_I2C_DEVICE	"/dev/i2c-0"
#define SM5713_CLIENT_PATH	"/sys/bus/i2c/devices/0-0033"
#define SM5713_ADDR		0x33
#define SM5713_REG_STATUS1	0x0b
#define SM5713_REG_CC_CNTL1	0x29
#define SM5713_REG_CC_CNTL2	0x2a
#define SM5713_REG_CC_CNTL3	0x2b
#define SM5713_REG_PD_STATE0	0xd5
#define SM5713_REG_PD_CNTL1	0x38
#define SM5713_REG_PD_CNTL2	0x39
#define SM5713_CC_ATTACH	(1U << 3)
#define SM5713_CC_SINK_ONLY	0x84
#define SM5713_CC_UNATTACHED_SINK	0x81
#define SM5713_CC_OPERATE	0x80

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

static int configure_sm5713_sink(const char *dir)
{
	unsigned char status1, pd_state0, cc_cntl1, cc_cntl2, cc_cntl3;
	unsigned long functions;
	int fd;

	if (!access(SM5713_CLIENT_PATH, F_OK)) {
		usb_step(dir, "USB: SM5713 kernel client present; skipping raw access");
		return 0;
	}
	if (errno != ENOENT) {
		usb_step(dir, "USB: cannot inspect SM5713 client path: %s",
			 strerror(errno));
		return -1;
	}

	fd = open(SM5713_I2C_DEVICE, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		usb_step(dir, "USB: open %s failed: %s", SM5713_I2C_DEVICE,
			 strerror(errno));
		return -1;
	}

	if (ioctl(fd, I2C_FUNCS, &functions) < 0 ||
	    !(functions & I2C_FUNC_SMBUS_READ_BYTE_DATA) ||
	    ioctl(fd, I2C_SLAVE, SM5713_ADDR) < 0) {
		usb_step(dir, "USB: I2C0 SM5713 setup failed: %s",
			 strerror(errno));
		close(fd);
		return -1;
	}

	if (sm5713_smbus(fd, true, SM5713_REG_CC_CNTL1, &cc_cntl1) ||
	    sm5713_smbus(fd, true, SM5713_REG_CC_CNTL2, &cc_cntl2) ||
	    sm5713_smbus(fd, true, SM5713_REG_CC_CNTL3, &cc_cntl3) ||
	    sm5713_smbus(fd, true, SM5713_REG_STATUS1, &status1) ||
	    sm5713_smbus(fd, true, SM5713_REG_PD_STATE0, &pd_state0)) {
		usb_step(dir, "USB: SM5713 register read failed: %s",
			 strerror(errno));
		close(fd);
		return -1;
	}
	close(fd);

	usb_step(dir,
		 "USB: SM5713 left in default state CC1=0x%02x CC2=0x%02x CC3=0x%02x STATUS1=0x%02x PD=0x%02x",
		 cc_cntl1, cc_cntl2, cc_cntl3, status1, pd_state0);
	return 0;
}

static void log_udc_state(const char *dir, const char *udc, const char *tag)
{
	char path[LINE_SIZE * 2];
	char state[LINE_SIZE];

	snprintf(path, sizeof(path), "/sys/class/udc/%s/state", udc);
	if (read_text_value(path, state, sizeof(state))) {
		usb_step(dir, "USB: %s udc state read failed: %s", tag,
			 strerror(errno));
		return;
	}
	usb_step(dir, "USB: %s udc state=%s", tag, state);
}

static void log_sm5713_status(const char *dir, const char *tag)
{
	unsigned char status1, pd_state0, cc_cntl1, cc_cntl3;
	int fd;

	fd = open(SM5713_I2C_DEVICE, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		usb_step(dir, "USB: %s SM5713 open failed: %s", tag,
			 strerror(errno));
		return;
	}
	if (ioctl(fd, I2C_SLAVE, SM5713_ADDR) < 0 ||
	    sm5713_smbus(fd, true, SM5713_REG_STATUS1, &status1) ||
	    sm5713_smbus(fd, true, SM5713_REG_PD_STATE0, &pd_state0) ||
	    sm5713_smbus(fd, true, SM5713_REG_CC_CNTL1, &cc_cntl1) ||
	    sm5713_smbus(fd, true, SM5713_REG_CC_CNTL3, &cc_cntl3)) {
		usb_step(dir, "USB: %s SM5713 read failed: %s", tag,
			 strerror(errno));
		close(fd);
		return;
	}
	close(fd);
	usb_step(dir,
		 "USB: %s SM5713 CC1=0x%02x CC3=0x%02x STATUS1=0x%02x PD=0x%02x",
		 tag, cc_cntl1, cc_cntl3, status1, pd_state0);
}

static void rebind_dwc3(const char *dir, const char *udc)
{
	usb_step(dir, "USB: unbinding %s to re-run PHY init", udc);
	if (write_attr("/sys/bus/platform/drivers/dwc3/unbind", udc)) {
		usb_step(dir, "USB: unbind failed: %s", strerror(errno));
		return;
	}
	usleep(300000);

	usb_step(dir, "USB: rebinding %s", udc);
	if (write_attr("/sys/bus/platform/drivers/dwc3/bind", udc)) {
		usb_step(dir, "USB: bind failed: %s", strerror(errno));
		return;
	}
	usleep(300000);
}

static int find_udc(char *name, size_t len)
{
	struct dirent *de;
	DIR *d;

	d = opendir("/sys/class/udc");
	if (!d)
		return -1;
	while ((de = readdir(d))) {
		if (de->d_name[0] == '.')
			continue;
		snprintf(name, len, "%s", de->d_name);
		closedir(d);
		return 0;
	}
	closedir(d);
	return -1;
}

static int setup_acm_gadget(const char *dir)
{
	char udc[LINE_SIZE];
	char path[LINE_SIZE * 2];
	char link[LINE_SIZE * 2];
	unsigned int i;

	if (find_udc(udc, sizeof(udc))) {
		report("USB: no UDC found");
		return -1;
	}
	usb_step(dir, "USB: using UDC %s", udc);

	/*
	 * The USB PHY probed before the ACPM PMIC regulator provider, so the
	 * LDO12/LDO13/LDO14 rails may only have been enabled afterwards.
	 * Rebinding dwc3 re-runs phy_init/power_on with those rails on.
	 */
	log_udc_state(dir, udc, "before rebind");
	rebind_dwc3(dir, udc);
	sleep(1);
	log_udc_state(dir, udc, "after rebind");

	if (make_dir("/sys/kernel/config/usb_gadget/g1") ||
	    make_dir("/sys/kernel/config/usb_gadget/g1/strings/0x409") ||
	    make_dir("/sys/kernel/config/usb_gadget/g1/configs/c.1") ||
	    make_dir("/sys/kernel/config/usb_gadget/g1/configs/c.1/strings/0x409") ||
	    make_dir("/sys/kernel/config/usb_gadget/g1/functions/acm.gs0")) {
		usb_step(dir, "USB: configfs directory setup failed: %s",
			 strerror(errno));
		return -1;
	}

	if (write_attr("/sys/kernel/config/usb_gadget/g1/idVendor", "0x1d6b") ||
	    write_attr("/sys/kernel/config/usb_gadget/g1/idProduct", "0x0104") ||
	    write_attr("/sys/kernel/config/usb_gadget/g1/bcdUSB", "0x0200") ||
	    write_attr("/sys/kernel/config/usb_gadget/g1/bcdDevice", "0x0100") ||
	    write_attr("/sys/kernel/config/usb_gadget/g1/strings/0x409/serialnumber",
		       "A51MAINLINE") ||
	    write_attr("/sys/kernel/config/usb_gadget/g1/strings/0x409/manufacturer",
		       "Samsung") ||
	    write_attr("/sys/kernel/config/usb_gadget/g1/strings/0x409/product",
		       "A51 mainline ACM") ||
	    write_attr("/sys/kernel/config/usb_gadget/g1/configs/c.1/strings/0x409/configuration",
		       "ACM") ||
	    write_attr("/sys/kernel/config/usb_gadget/g1/configs/c.1/MaxPower",
		       "250")) {
		usb_step(dir, "USB: configfs attribute write failed: %s",
			 strerror(errno));
		return -1;
	}

	snprintf(link, sizeof(link),
		 "/sys/kernel/config/usb_gadget/g1/configs/c.1/acm.gs0");
	snprintf(path, sizeof(path),
		 "/sys/kernel/config/usb_gadget/g1/functions/acm.gs0");
	if (symlink(path, link) && errno != EEXIST) {
		usb_step(dir, "USB: function symlink failed: %s", strerror(errno));
		return -1;
	}

	usb_step(dir, "USB: configfs attributes written; binding UDC");
	sync();
	if (write_attr("/sys/kernel/config/usb_gadget/g1/UDC", udc)) {
		usb_step(dir, "USB: UDC bind failed: %s", strerror(errno));
		return -1;
	}
	usb_step(dir, "USB: UDC bind returned successfully");

	usb_step(dir, "USB: sleeping 10s after UDC bind");
	for (i = 0; i < 10; i++) {
		sleep(1);
		log_udc_state(dir, udc, "after bind");
	}
	usb_step(dir, "USB: survived 10s after UDC bind");
	log_sm5713_status(dir, "after bind");
	save_one("/proc/interrupts", "interrupts-after-usb.txt", dir, true);
	save_one("/sys/kernel/debug/regulator/regulator_summary",
		 "regulator-summary-after-usb.txt", dir, false);

	return 0;
}

static const char *wait_for_partition(void)
{
	static const char *candidates[] = {
		"/dev/mmcblk0p1",
		"/dev/mmcblk0",
	};
	unsigned int i, attempt;

	for (attempt = 0; attempt < 60; attempt++) {
		for (i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++)
			if (access(candidates[i], F_OK) == 0)
				return candidates[i];
		sleep(1);
	}

	return NULL;
}

static int mount_card(const char *device, const char *mountpoint,
		      const char **fstype)
{
	static const char *types[] = { "ext4", "exfat" };
	unsigned int i;
	int saved_errno = 0;

	for (i = 0; i < sizeof(types) / sizeof(types[0]); i++) {
		if (mount(device, mountpoint, types[i], 0, "") == 0) {
			*fstype = types[i];
			return 0;
		}
		saved_errno = errno;
	}

	errno = saved_errno;
	return -1;
}

static int make_boot_dir(void)
{
	char base[] = "/mnt/pi-debug";
	struct dirent *de;
	int max = 0;
	DIR *dir;

	mkdir(base, 0755);
	dir = opendir(base);
	if (dir) {
		while ((de = readdir(dir))) {
			int n;

			if (sscanf(de->d_name, "boot-%d", &n) == 1 && n > max)
				max = n;
		}
		closedir(dir);
	}

	snprintf(logdir, sizeof(logdir), "%s/boot-%04d", base, max + 1);
	if (mkdir(logdir, 0755) && errno != EEXIST)
		return -1;
	return 0;
}

static int copy_file(const char *src, const char *dst)
{
	char buf[COPY_BUF_SIZE];
	ssize_t n;
	int in, out;
	long long total = 0;

	in = open(src, O_RDONLY | O_CLOEXEC);
	if (in < 0)
		return -1;

	out = open(dst, O_CREAT | O_TRUNC | O_WRONLY | O_CLOEXEC, 0644);
	if (out < 0) {
		close(in);
		return -1;
	}

	while ((n = read(in, buf, sizeof(buf))) > 0) {
		ssize_t off = 0;

		while (off < n) {
			ssize_t w = write(out, buf + off, n - off);

			if (w < 0) {
				close(out);
				close(in);
				return -1;
			}
			off += w;
		}
		total += n;
	}

	close(out);
	close(in);

	if (n < 0)
		return -1;

	return (int)(total > 0x7fffffff ? 0x7fffffff : total);
}

static int dump_kmsg(const char *dst)
{
	char buf[COPY_BUF_SIZE];
	int in, out;
	long long total = 0;

	in = open("/dev/kmsg", O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (in < 0)
		return -1;

	out = open(dst, O_CREAT | O_TRUNC | O_WRONLY | O_CLOEXEC, 0644);
	if (out < 0) {
		close(in);
		return -1;
	}

	while (total < KMSG_MAX) {
		ssize_t n = read(in, buf, sizeof(buf));
		ssize_t off = 0;

		if (n <= 0) {
			if (n < 0 && errno == EAGAIN)
				break;
			break;
		}
		while (off < n) {
			ssize_t w = write(out, buf + off, n - off);

			if (w < 0) {
				close(out);
				close(in);
				return -1;
			}
			off += w;
		}
		total += n;
	}

	close(out);
	close(in);
	return (int)(total > 0x7fffffff ? 0x7fffffff : total);
}

static int dump_kmsg_messages(const char *dst)
{
	char buf[COPY_BUF_SIZE];
	int in, out;
	long long total = 0;

	in = open("/dev/kmsg", O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (in < 0)
		return -1;

	out = open(dst, O_CREAT | O_TRUNC | O_WRONLY | O_CLOEXEC, 0644);
	if (out < 0) {
		close(in);
		return -1;
	}

	while (total < KMSG_MAX) {
		ssize_t n = read(in, buf, sizeof(buf));
		char *semi;
		ssize_t off;
		ssize_t payload_len;

		if (n <= 0)
			break;
		semi = memchr(buf, ';', n);
		if (semi) {
			off = semi - buf + 1;
			payload_len = n - off;
		} else {
			off = 0;
			payload_len = n;
		}
		if (payload_len > 0) {
			ssize_t written = 0;

			while (written < payload_len) {
				ssize_t w = write(out, buf + off + written,
						  payload_len - written);

				if (w < 0) {
					close(out);
					close(in);
					return -1;
				}
				written += w;
			}
			total += payload_len;
		}
	}

	close(out);
	close(in);
	return (int)(total > 0x7fffffff ? 0x7fffffff : total);
}

static int read_text_value(const char *path, char *buf, size_t len)
{
	int fd;
	ssize_t n;

	fd = open(path, O_RDONLY | O_CLOEXEC);
	if (fd < 0)
		return -1;
	n = read(fd, buf, len - 1);
	close(fd);
	if (n <= 0)
		return -1;
	buf[n] = '\0';
	while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r'))
		buf[--n] = '\0';
	return 0;
}

static int save_dir_listing(const char *src, const char *name,
			    const char *dir)
{
	char dst[LINE_SIZE * 2];
	struct dirent *de;
	DIR *d;
	int fd;

	d = opendir(src);
	if (!d)
		return -1;

	snprintf(dst, sizeof(dst), "%s/%s", dir, name);
	fd = open(dst, O_CREAT | O_TRUNC | O_WRONLY | O_CLOEXEC, 0644);
	if (fd < 0) {
		closedir(d);
		return -1;
	}

	while ((de = readdir(d)))
		dprintf(fd, "%s\n", de->d_name);

	close(fd);
	closedir(d);
	return 0;
}

static int save_clocks(const char *dir)
{
	static const char * const clocks[] = {
		"mout_fsys_mmc_card", "gout_fsys_mmc_card",
		"dout_fsys_mmc_card", "mout_fsys_mmc_card_user",
		"mout_fsys_bus", "mout_fsys_bus_user", "gout_fsys_bus",
		"dout_fsys_bus",
		"mout_peri_bus", "mout_peri_ip", "mout_peri_uart",
		"dout_peri_bus", "dout_peri_ip", "dout_peri_uart",
		"dout_peri_i2c", "gout_peri_i2c",
		"dout_peri_spi0", "dout_peri_spi1", "dout_peri_spi2",
		"gout_peri_spi0", "gout_peri_spi1", "gout_peri_spi2",
		"dout_peri_usi_i2c", "gout_peri_usi_i2c",
		"dout_peri_usi_usi", "gout_peri_usi_usi",
		"gout_i2c_0_pclk", "gout_i2c_1_pclk", "gout_i2c_2_pclk",
		"gout_i2c_3_pclk", "gout_i2c_4_pclk", "gout_i2c_5_pclk",
		"gout_i2c_6_pclk",
		"gout_spi0_pclk", "gout_spi1_pclk", "gout_spi2_pclk",
		"gout_uart_pclk", "gout_usi00_i2c_pclk",
		"gout_usi00_usi_pclk",
		"mout_core_bus", "mout_core_bus_user", "gout_core_bus",
		"dout_core_bus", "div_clk_core_busp", "gout_core_cmu_pclk",
		"mout_apm_bus", "mout_apm_bus_user", "gout_apm_bus",
		"dout_apm_bus", "div_clk_apm_bus",
		"gout_clkcmu_cmgp_bus", "gout_cmgp_cmu_pclk",
		"gout_sysreg_cmgp_pclk", "gout_gpio_cmgp_pclk",
		"mux_cmgp_i2c", "div_clk_cmgp_i2c",
		"mux_cmgp_usi00", "div_clk_cmgp_usi00",
		"div_clk_cmgp_adc",
		"oscclk_rco_cmgp", "oscclk_cmgp",
		"gout_peri_cmu_pclk", "gout_peri_bus_rstsync",
		"gout_peri_i2c_rstsync", "gout_peri_spi0_rstsync",
		"gout_peri_spi1_rstsync", "gout_peri_spi2_rstsync",
		"gout_peri_uart_rstsync", "gout_peri_usi_i2c_rstsync",
		"gout_peri_usi_usi_rstsync",
		"mout_usb_usb30drd", "dout_usb_usb30drd",
		"gout_usb_usb30drd",
		"fout_mmc_pll", "fout_shared0_pll", "fout_shared1_pll",
		"oscclk_fsys", "oscclk_rco_apm", "fin_pll",
	};
	char path[LINE_SIZE * 2];
	char dst[LINE_SIZE * 2];
	char rate[64], enable[64], prepare[64];
	unsigned int i;
	int fd, total = 0;

	snprintf(dst, sizeof(dst), "%s/clocks.txt", dir);
	fd = open(dst, O_CREAT | O_TRUNC | O_WRONLY | O_CLOEXEC, 0644);
	if (fd < 0)
		return -1;

	for (i = 0; i < sizeof(clocks) / sizeof(clocks[0]); i++) {
		char line[LINE_SIZE];
		int len;

		snprintf(path, sizeof(path),
			 "/sys/kernel/debug/clk/%s/clk_rate", clocks[i]);
		if (read_text_value(path, rate, sizeof(rate)))
			snprintf(rate, sizeof(rate), "NA");
		snprintf(path, sizeof(path),
			 "/sys/kernel/debug/clk/%s/clk_enable_count", clocks[i]);
		if (read_text_value(path, enable, sizeof(enable)))
			snprintf(enable, sizeof(enable), "NA");
		snprintf(path, sizeof(path),
			 "/sys/kernel/debug/clk/%s/clk_prepare_count", clocks[i]);
		if (read_text_value(path, prepare, sizeof(prepare)))
			snprintf(prepare, sizeof(prepare), "NA");

		len = snprintf(line, sizeof(line), "%-32s rate=%-12s enable=%-3s prepare=%-3s\n",
			       clocks[i], rate, enable, prepare);
		if (len > 0 && write(fd, line, len) > 0)
			total += len;
	}

	fsync(fd);
	close(fd);
	report("SAVED: clocks.txt bytes=%d entries=%u", total,
	       (unsigned int)(sizeof(clocks) / sizeof(clocks[0])));
	return 0;
}

static int save_one(const char *src, const char *name, const char *dir,
		    bool report_missing)
{
	char dst[LINE_SIZE * 2];
	int ret;

	snprintf(dst, sizeof(dst), "%s/%s", dir, name);
	ret = copy_file(src, dst);
	if (ret < 0) {
		if (report_missing)
			report("copy %s failed: %s", src, strerror(errno));
		return -1;
	}
	report("SAVED: %s bytes=%d", name, ret);
	return 0;
}

int main(void)
{
	const char *device;
	const char *fstype = NULL;
	struct statfs sfs;
	int ret;

	make_dirs();
	mount_devtmpfs();
	mount_procfs();
	mount_sysfs();
	mount_debugfs();
	mount_configfs();

	kmsg_fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
	pmsg_fd = open("/dev/pmsg0", O_WRONLY | O_CLOEXEC);

	report("waiting for microSD partition");
	device = wait_for_partition();
	if (!device) {
		report("FAIL: no mmcblk device found");
		report("TEST RESULT: FAIL");
		goto out;
	}
	report("device: %s", device);

	ret = mount_card(device, "/mnt", &fstype);
	if (ret) {
		report("FAIL: mount %s rw ext4/exfat: %s", device,
		       strerror(errno));
		report("TEST RESULT: FAIL");
		goto out;
	}
	report("mounted %s as %s (read-write)", device, fstype);

	if (statfs("/mnt", &sfs) == 0)
		report("FS: type=%#lx blocks=%llu bfree=%llu bsize=%u",
		       (unsigned long)sfs.f_type,
		       (unsigned long long)sfs.f_blocks,
		       (unsigned long long)sfs.f_bfree, sfs.f_bsize);

	if (make_boot_dir()) {
		report("FAIL: mkdir boot dir: %s", strerror(errno));
		report("TEST RESULT: FAIL");
		goto out;
	}

	report("saving debug logs to %s", logdir);

	save_one("/proc/cmdline", "cmdline.txt", logdir, true);
	save_one("/proc/version", "version.txt", logdir, true);
	save_one("/proc/interrupts", "interrupts.txt", logdir, true);
	save_one("/proc/partitions", "partitions.txt", logdir, true);

	save_one("/sys/kernel/debug/mmc0/ios", "mmc0-ios.txt", logdir, false);
	save_one("/sys/kernel/debug/mmc0/regs", "mmc0-regs.txt", logdir, false);
	save_one("/sys/kernel/debug/mmc0/clock", "mmc0-clock.txt", logdir, false);
	save_one("/sys/kernel/debug/mmc0/state", "mmc0-state.txt", logdir, false);
	save_one("/proc/uptime", "uptime.txt", logdir, true);
	save_dir_listing("/sys/bus/platform/devices",
			 "platform-devices.txt", logdir);
	save_dir_listing("/sys/bus/platform/drivers/dwc3",
			 "dwc3-driver.txt", logdir);
	save_dir_listing("/sys/class/udc", "udc.txt", logdir);
	save_one("/sys/kernel/debug/devices_deferred",
		 "devices-deferred.txt", logdir, false);
	save_clocks(logdir);
	save_one("/sys/kernel/debug/regulator/regulator_summary",
		 "regulator-summary.txt", logdir, false);

	report("USB: unplug the cable now; SM5713 config in 15s");
	sleep(15);
	configure_sm5713_sink(logdir);

	if (setup_acm_gadget(logdir))
		report("USB: ACM gadget setup failed");

	{
		char messages_path[LINE_SIZE * 2];
		int messages;

		snprintf(messages_path, sizeof(messages_path),
			 "%s/kmsg-messages.txt", logdir);
		messages = dump_kmsg_messages(messages_path);
		if (messages >= 0)
			report("SAVED: kmsg-messages.txt bytes=%d", messages);
		else
			report("copy /dev/kmsg messages failed: %s",
			       strerror(errno));
	}

	{
		char kmsg_path[LINE_SIZE * 2];

		snprintf(kmsg_path, sizeof(kmsg_path), "%s/kmsg-current.txt",
			 logdir);
		ret = dump_kmsg(kmsg_path);
	}
	if (ret >= 0)
		report("SAVED: kmsg-current.txt bytes=%d", ret);
	else
		report("copy /dev/kmsg failed: %s", strerror(errno));

	sync();
	report("log dump complete; files are under %s", logdir);
	report("TEST RESULT: PASS");

out:
	for (;;)
		pause();
}
