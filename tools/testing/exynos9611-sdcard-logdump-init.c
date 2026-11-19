// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos9611 A51 microSD debug-log dump init.
 *
 * Mounts the ext4 card read-write and saves the current kernel log, pstore
 * records, /proc state and mmc0 debugfs state into /mnt/pi-debug/.
 */

#define _GNU_SOURCE

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
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
