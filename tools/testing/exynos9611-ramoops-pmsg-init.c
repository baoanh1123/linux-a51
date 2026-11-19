// SPDX-License-Identifier: GPL-2.0-only
/*
 * PID 1 that writes fixed markers only through pstore's pmsg interface.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <unistd.h>

static int mount_devtmpfs(void)
{
	if (mkdir("/dev", 0755) && errno != EEXIST)
		return -1;

	if (mount("devtmpfs", "/dev", "devtmpfs", MS_NOSUID, NULL) &&
	    errno != EBUSY)
		return -1;

	return 0;
}

static int write_record(int fd, const char *record)
{
	size_t len = strlen(record);
	ssize_t written;

	written = write(fd, record, len);
	if (written != (ssize_t)len)
		return -1;

	return 0;
}

static void report_result(const char *result, unsigned int written,
			  unsigned int failures)
{
	char message[128];
	int fd;
	int len;

	len = snprintf(message, sizeof(message),
		       "<6>RAMOOPS-PMSG-ONLY-RESULT: %s writes=%u failures=%u\n",
		       result, written, failures);
	if (len < 0 || (size_t)len >= sizeof(message))
		return;

	fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
	if (fd < 0)
		return;

	write(fd, message, (size_t)len);
	close(fd);
}

int main(void)
{
	static const char * const records[] = {
		"RAMOOPS-PMSG-ONLY-BEGIN-51B4\n",
		"RAMOOPS-PMSG-ONLY-ASCII-0:0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ\n",
		"RAMOOPS-PMSG-ONLY-ASCII-1:abcdefghijklmnopqrstuvwxyz9876543210\n",
		"RAMOOPS-PMSG-ONLY-END-51B4\n",
		NULL,
	};
	unsigned int failures = 0;
	unsigned int i, written = 0;
	const char *result = "ok";
	int fd;

	if (mount_devtmpfs()) {
		report_result("devtmpfs_failed", 0, 1);
		goto wait;
	}

	fd = open("/dev/pmsg0", O_WRONLY | O_CLOEXEC);
	if (fd < 0) {
		report_result("open_failed", 0, 1);
		goto wait;
	}

	for (i = 0; records[i]; i++) {
		if (write_record(fd, records[i]))
			failures++;
		else
			written++;
	}

	close(fd);
	if (failures)
		result = "write_failed";
	report_result(result, written, failures);

wait:
	for (;;)
		pause();
}
