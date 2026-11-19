// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos9611 A51 microSD captured-log check init.
 *
 * Mounts the ext4 card and checks the /mnt/pi-debug files: non-text byte
 * counts and markers for ramoops/kmsg logs, byte-exact matches for the
 * static /proc and mmc0 debugfs copies.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <unistd.h>

#define LINE_SIZE	512
#define KMSG_MAX	(4 * 1024 * 1024)

static int kmsg_fd = -1;
static int pmsg_fd = -1;
static const char *logdir = "/mnt/pi-debug";

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
		len = snprintf(kmsg, sizeof(kmsg), "<6>Exynos9611-SD-check: %s\n",
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

static void mount_if_needed(const char *src, const char *target,
			    const char *type)
{
	if (mount(src, target, type, MS_NOSUID | MS_NODEV | MS_NOEXEC, NULL) &&
	    errno != EBUSY)
		printf("%s mount failed: %s\n", type, strerror(errno));
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

static int read_whole(const char *path, unsigned char **buf, size_t *len)
{
	struct stat st;
	unsigned char *data;
	size_t done = 0;
	int fd;

	fd = open(path, O_RDONLY | O_CLOEXEC);
	if (fd < 0)
		return -1;
	if (fstat(fd, &st) < 0 || st.st_size <= 0) {
		close(fd);
		return -1;
	}
	data = malloc(st.st_size + 1);
	if (!data) {
		close(fd);
		return -1;
	}
	while (done < (size_t)st.st_size) {
		ssize_t n = read(fd, data + done, st.st_size - done);

		if (n <= 0) {
			free(data);
			close(fd);
			return -1;
		}
		done += n;
	}
	data[done] = 0;
	close(fd);
	*buf = data;
	*len = done;
	return 0;
}

static unsigned int count_marker(const unsigned char *buf, size_t len,
				 const char *marker)
{
	size_t mlen = strlen(marker);
	size_t i;
	unsigned int count = 0;

	if (mlen > len)
		return 0;
	for (i = 0; i + mlen <= len; i++)
		if (!memcmp(buf + i, marker, mlen))
			count++;
	return count;
}

static void check_log(const char *name)
{
	char path[LINE_SIZE];
	unsigned char *buf;
	size_t len, i;
	unsigned int nontext = 0, nul = 0, high = 0;

	snprintf(path, sizeof(path), "%s/%s", logdir, name);
	if (read_whole(path, &buf, &len)) {
		report("CHECK: %s unreadable: %s", name, strerror(errno));
		return;
	}

	for (i = 0; i < len; i++) {
		unsigned char c = buf[i];

		if (!c)
			nul++;
		else if (c < 0x20 && c != '\n' && c != '\r' && c != '\t')
			nontext++;
		else if (c >= 0x80)
			high++;
	}

	report("CHECK: %s bytes=%zu nontext=%u nul=%u high=%u",
	       name, len, nontext, nul, high);
	report("CHECK: %s markers SD-log=%u result=%u mmcblk=%u ext4=%u",
	       name,
	       count_marker(buf, len, "Exynos9611-SD"),
	       count_marker(buf, len, "TEST RESULT"),
	       count_marker(buf, len, "mmcblk0p1"),
	       count_marker(buf, len, "EXT4-fs"));
	free(buf);
}

static int compare_files(const char *src, const char *dst, const char *name)
{
	unsigned char *a = NULL, *b = NULL;
	size_t alen = 0, blen = 0;
	int ret = -1;

	if (read_whole(src, &a, &alen) || read_whole(dst, &b, &blen))
		goto out;
	if (alen != blen) {
		report("MISMATCH: %s size src=%zu card=%zu", name, alen, blen);
		goto out;
	}
	if (memcmp(a, b, alen)) {
		size_t i;

		for (i = 0; i < alen; i++)
			if (a[i] != b[i])
				break;
		report("MISMATCH: %s at offset %zu", name, i);
		goto out;
	}
	report("MATCH: %s bytes=%zu", name, alen);
	ret = 0;
out:
	free(a);
	free(b);
	return ret;
}

int main(void)
{
	const char *device;
	struct {
		const char *src;
		const char *card;
		const char *name;
	} static_files[] = {
		{ "/proc/cmdline", "cmdline.txt", "cmdline" },
		{ "/proc/version", "version.txt", "version" },
		{ "/proc/interrupts", "interrupts.txt", "interrupts" },
		{ "/proc/partitions", "partitions.txt", "partitions" },
		{ "/sys/kernel/debug/mmc0/ios", "mmc0-ios.txt", "mmc0-ios" },
		{ "/sys/kernel/debug/mmc0/regs", "mmc0-regs.txt", "mmc0-regs" },
		{ "/sys/kernel/debug/mmc0/clock", "mmc0-clock.txt", "mmc0-clock" },
		{ "/sys/kernel/debug/mmc0/state", "mmc0-state.txt", "mmc0-state" },
	};
	unsigned int i;
	int ret;

	make_dirs();
	mount_if_needed("devtmpfs", "/dev", "devtmpfs");
	mount_if_needed("proc", "/proc", "proc");
	mount_if_needed("sysfs", "/sys", "sysfs");
	mount_if_needed("pstore", "/sys/fs/pstore", "pstore");
	mount_if_needed("debugfs", "/sys/kernel/debug", "debugfs");

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

	ret = mount(device, "/mnt", "ext4", MS_RDONLY, "");
	if (ret) {
		report("FAIL: mount %s ro ext4: %s", device, strerror(errno));
		report("TEST RESULT: FAIL");
		goto out;
	}
	report("mounted %s as ext4 (read-only)", device);

	check_log("console-ramoops-0.raw");
	check_log("pmsg-ramoops-0.raw");
	check_log("dmesg-ramoops-0.raw");
	check_log("kmsg-current.txt");

	for (i = 0; i < sizeof(static_files) / sizeof(static_files[0]); i++) {
		char dst[LINE_SIZE];

		snprintf(dst, sizeof(dst), "%s/%s", logdir,
			 static_files[i].card);
		compare_files(static_files[i].src, dst, static_files[i].name);
	}

	sync();
	report("log check complete");
	report("TEST RESULT: PASS");

out:
	for (;;)
		pause();
}
