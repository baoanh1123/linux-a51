// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal Exynos9611 A51 microSD data test init.
 *
 * Mounts the first partition as exFAT read-only, lists the root directory,
 * reads a file and reports PASS/FAIL to kmsg and pmsg.
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
#define MAX_ENTRIES	24

static int kmsg_fd = -1;
static int pmsg_fd = -1;

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
		len = snprintf(kmsg, sizeof(kmsg), "<6>Exynos9611-SD-test: %s\n",
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

static int mount_devtmpfs(void)
{
	if (mkdir("/dev", 0755) && errno != EEXIST)
		return -1;
	if (mount("devtmpfs", "/dev", "devtmpfs", MS_NOSUID, NULL) &&
	    errno != EBUSY)
		return -1;
	return 0;
}

static int mount_card(const char *device, const char *mountpoint,
		      const char **fstype)
{
	static const char *types[] = { "ext4", "exfat" };
	unsigned int i;
	int saved_errno = 0;

	for (i = 0; i < sizeof(types) / sizeof(types[0]); i++) {
		if (mount(device, mountpoint, types[i], MS_RDONLY, "") == 0) {
			*fstype = types[i];
			return 0;
		}
		saved_errno = errno;
	}

	errno = saved_errno;
	return -1;
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

static bool printable_name(const char *name)
{
	const unsigned char *p;

	for (p = (const unsigned char *)name; *p; p++)
		if (*p < 0x20 || *p == 0x7f)
			return false;
	return true;
}

static int list_and_read(const char *mountpoint)
{
	char first_file[LINE_SIZE] = "";
	char path[LINE_SIZE];
	struct statfs sfs;
	struct dirent *de;
	struct stat st;
	DIR *dir;
	int entries = 0;
	int fd;

	if (statfs(mountpoint, &sfs) == 0)
		report("FS: type=%#lx blocks=%llu bfree=%llu bsize=%u",
		       (unsigned long)sfs.f_type,
		       (unsigned long long)sfs.f_blocks,
		       (unsigned long long)sfs.f_bfree, sfs.f_bsize);
	else
		report("FS: statfs failed: %s", strerror(errno));

	dir = opendir(mountpoint);
	if (!dir) {
		report("FAIL: opendir %s: %s", mountpoint, strerror(errno));
		return -1;
	}

	while ((de = readdir(dir)) && entries < MAX_ENTRIES) {
		if (!strcmp(de->d_name, ".") || !strcmp(de->d_name, ".."))
			continue;
		snprintf(path, sizeof(path), "%s/%s", mountpoint, de->d_name);
		if (stat(path, &st) < 0)
			continue;
		report("ENTRY: %s%s size=%lld",
		       printable_name(de->d_name) ? de->d_name : "<nonprintable>",
		       S_ISDIR(st.st_mode) ? "/" : "",
		       (long long)st.st_size);
		if (S_ISREG(st.st_mode) && !first_file[0])
			snprintf(first_file, sizeof(first_file), "%s", path);
		entries++;
	}
	closedir(dir);

	report("DIR: %d entries listed", entries);
	if (!first_file[0]) {
		report("empty exFAT root (no regular files); mount/statfs succeeded");
		return 0;
	}

	fd = open(first_file, O_RDONLY);
	if (fd < 0) {
		report("FAIL: open %s: %s", first_file, strerror(errno));
		return -1;
	}

	{
		unsigned char buf[4096];
		ssize_t n = read(fd, buf, sizeof(buf));
		unsigned int sum = 0;
		ssize_t i;

		close(fd);
		if (n < 0) {
			report("FAIL: read %s: %s", first_file, strerror(errno));
			return -1;
		}
		for (i = 0; i < n; i++)
			sum += buf[i];
		report("READ: %s bytes=%zd sum=%#x", first_file, n, sum);
	}

	return 0;
}

int main(void)
{
	const char *device;
	const char *fstype = NULL;
	char mountpoint[] = "/mnt";
	int ret;

	if (mount_devtmpfs()) {
		/* console fallback: /dev/kmsg is not available yet */
		printf("FAIL: devtmpfs mount: %s\n", strerror(errno));
		goto out;
	}

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

	if (mkdir(mountpoint, 0755) && errno != EEXIST) {
		report("FAIL: mkdir %s: %s", mountpoint, strerror(errno));
		report("TEST RESULT: FAIL");
		goto out;
	}

	ret = mount_card(device, mountpoint, &fstype);
	if (ret) {
		report("FAIL: mount %s as ext4/exfat: %s", device,
		       strerror(errno));
		report("TEST RESULT: FAIL");
		goto out;
	}
	report("mounted %s as %s (read-only)", device, fstype);

	if (list_and_read(mountpoint)) {
		report("TEST RESULT: FAIL");
		goto out;
	}

	report("TEST RESULT: PASS");

out:
	for (;;)
		pause();
}
