// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos9611 A51 microSD read-write test init.
 *
 * Mounts the first partition read-write, creates a binary file and a text
 * file, fsyncs, reads both back, and reports PASS/FAIL to kmsg and pmsg.
 */

#define _GNU_SOURCE

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <unistd.h>

#define LINE_SIZE	512
#define CHUNK_SIZE	(64 * 1024)
#define CHUNK_COUNT	16
#define BIN_SIZE	(CHUNK_SIZE * CHUNK_COUNT)
#define TEXT_FILE	"/mnt/pi-rw-test.txt"
#define BIN_FILE	"/mnt/pi-rw-test.bin"
#define TEXT_DATA	"Exynos9611 microSD read-write test\n"

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
		len = snprintf(kmsg, sizeof(kmsg), "<6>Exynos9611-SD-rw: %s\n",
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

static void fill_pattern(unsigned char *buf, unsigned int seed)
{
	unsigned int i;

	for (i = 0; i < CHUNK_SIZE; i++)
		buf[i] = (unsigned char)(i * 37 + seed + 11);
}

static int write_binary(void)
{
	unsigned char *buf;
	unsigned int chunk;
	int fd, ret = -1;

	buf = malloc(CHUNK_SIZE);
	if (!buf)
		return -1;

	fd = open(BIN_FILE, O_CREAT | O_TRUNC | O_WRONLY, 0644);
	if (fd < 0) {
		report("FAIL: open %s: %s", BIN_FILE, strerror(errno));
		goto out;
	}

	for (chunk = 0; chunk < CHUNK_COUNT; chunk++) {
		ssize_t written;
		size_t offset = 0;

		fill_pattern(buf, chunk * 17);
		while (offset < CHUNK_SIZE) {
			written = write(fd, buf + offset, CHUNK_SIZE - offset);
			if (written < 0) {
				report("FAIL: write %s: %s", BIN_FILE,
				       strerror(errno));
				goto close_out;
			}
			offset += written;
		}
	}

	if (fsync(fd) < 0) {
		report("FAIL: fsync %s: %s", BIN_FILE, strerror(errno));
		goto close_out;
	}
	report("WRITE: %s bytes=%u", BIN_FILE, BIN_SIZE);
	ret = 0;

close_out:
	close(fd);
out:
	free(buf);
	return ret;
}

static int verify_binary(void)
{
	unsigned char *expect, *got;
	unsigned int chunk;
	int fd, ret = -1;

	expect = malloc(CHUNK_SIZE);
	got = malloc(CHUNK_SIZE);
	if (!expect || !got)
		goto out;

	fd = open(BIN_FILE, O_RDONLY);
	if (fd < 0) {
		report("FAIL: reopen %s: %s", BIN_FILE, strerror(errno));
		goto out;
	}

	for (chunk = 0; chunk < CHUNK_COUNT; chunk++) {
		size_t offset = 0;

		fill_pattern(expect, chunk * 17);
		while (offset < CHUNK_SIZE) {
			ssize_t n = read(fd, got + offset, CHUNK_SIZE - offset);

			if (n < 0) {
				report("FAIL: read %s: %s", BIN_FILE,
				       strerror(errno));
				goto close_out;
			}
			if (n == 0) {
				report("FAIL: short read at chunk %u offset %zu",
				       chunk, offset);
				goto close_out;
			}
			offset += n;
		}
		if (memcmp(expect, got, CHUNK_SIZE)) {
			report("FAIL: data mismatch at chunk %u", chunk);
			goto close_out;
		}
	}

	report("READ: %s bytes=%u match=yes", BIN_FILE, BIN_SIZE);
	ret = 0;

close_out:
	close(fd);
out:
	free(expect);
	free(got);
	return ret;
}

static int text_test(void)
{
	char buf[128];
	size_t len = strlen(TEXT_DATA);
	int fd, ret = -1;
	ssize_t n;

	fd = open(TEXT_FILE, O_CREAT | O_TRUNC | O_WRONLY, 0644);
	if (fd < 0) {
		report("FAIL: open %s: %s", TEXT_FILE, strerror(errno));
		return -1;
	}
	if (write(fd, TEXT_DATA, len) != (ssize_t)len || fsync(fd) < 0) {
		report("FAIL: write %s: %s", TEXT_FILE, strerror(errno));
		close(fd);
		return -1;
	}
	close(fd);

	fd = open(TEXT_FILE, O_RDONLY);
	if (fd < 0) {
		report("FAIL: reopen %s: %s", TEXT_FILE, strerror(errno));
		return -1;
	}
	n = read(fd, buf, sizeof(buf));
	close(fd);
	if (n != (ssize_t)len || memcmp(buf, TEXT_DATA, len)) {
		report("FAIL: text verify %s (n=%zd)", TEXT_FILE, n);
		return -1;
	}

	buf[n] = '\0';
	report("TEXT: %s content=%s", TEXT_FILE, buf);
	ret = 0;
	return ret;
}

int main(void)
{
	const char *device;
	char mountpoint[] = "/mnt";
	struct statfs sfs;
	int ret;

	if (mount_devtmpfs()) {
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

	ret = mount(device, mountpoint, "ext4", 0, "");
	if (ret) {
		report("FAIL: mount %s rw ext4: %s", device, strerror(errno));
		report("TEST RESULT: FAIL");
		goto out;
	}
	report("mounted %s as ext4 (read-write)", device);

	if (statfs(mountpoint, &sfs) == 0)
		report("FS: type=%#lx blocks=%llu bfree=%llu bsize=%u",
		       (unsigned long)sfs.f_type,
		       (unsigned long long)sfs.f_blocks,
		       (unsigned long long)sfs.f_bfree, sfs.f_bsize);

	if (write_binary() || verify_binary() || text_test()) {
		report("TEST RESULT: FAIL");
		goto out;
	}

	sync();
	report("TEST RESULT: PASS");

out:
	for (;;)
		pause();
}
