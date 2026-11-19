// SPDX-License-Identifier: GPL-2.0-only
/*
 * PID 1 that writes deterministic, checksummed patterns only through pmsg.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <unistd.h>

#define MAX_PATTERN_SIZE 1024
#define MAX_RECORD_SIZE (2 * MAX_PATTERN_SIZE + 128)

static int mount_devtmpfs(void)
{
	if (mkdir("/dev", 0755) && errno != EEXIST)
		return -1;

	if (mount("devtmpfs", "/dev", "devtmpfs", MS_NOSUID, NULL) &&
	    errno != EBUSY)
		return -1;

	return 0;
}

static int write_record(int fd, const char *record, size_t len)
{
	ssize_t written;

	written = write(fd, record, len);
	if (written != (ssize_t)len)
		return -1;

	return 0;
}

static uint32_t crc32_ieee(const unsigned char *data, size_t len)
{
	uint32_t crc = 0xffffffff;
	size_t i;
	unsigned int bit;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (bit = 0; bit < 8; bit++)
			crc = (crc >> 1) ^ (0xedb88320 &
						    (uint32_t)-(int32_t)(crc & 1));
	}

	return crc ^ 0xffffffff;
}

static void make_pattern(unsigned int block, unsigned char *data, size_t len)
{
	uint32_t state = 0x9e3779b9U ^ (block * 0x85ebca6bU);
	size_t i;

	for (i = 0; i < len; i++) {
		state ^= state << 13;
		state ^= state >> 17;
		state ^= state << 5;
		data[i] = (unsigned char)state;
	}
}

static int make_record(unsigned int block, size_t pattern_len, char *record,
		       size_t record_size)
{
	static const char hex[] = "0123456789abcdef";
	unsigned char pattern[MAX_PATTERN_SIZE];
	uint32_t crc;
	int prefix_len;
	size_t i, record_len;

	if (pattern_len > sizeof(pattern))
		return -1;

	make_pattern(block, pattern, pattern_len);
	crc = crc32_ieee(pattern, pattern_len);
	prefix_len = snprintf(record, record_size,
			      "PMSGCRC-V2|block=%u|len=%zu|crc32=%08x|data=",
			      block, pattern_len, crc);
	if (prefix_len < 0 || (size_t)prefix_len + 2 * pattern_len + 1 >
	    record_size)
		return -1;

	record_len = (size_t)prefix_len;
	for (i = 0; i < pattern_len; i++) {
		record[record_len++] = hex[pattern[i] >> 4];
		record[record_len++] = hex[pattern[i] & 0x0f];
	}
	record[record_len++] = '\n';

	return (int)record_len;
}

static void report_result(const char *result, unsigned int written,
			  unsigned int failures)
{
	char message[128];
	int fd, len;

	len = snprintf(message, sizeof(message),
		       "<6>RAMOOPS-PMSG-CRC-RESULT: %s writes=%u failures=%u\n",
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
	static const size_t pattern_lengths[] = { 16, 64, 256, 1024 };
	static const char * const markers[] = {
		"PMSGCRC-V2-BEGIN|seed=9e3779b9\n",
		"PMSGCRC-V2-END\n",
	};
	unsigned int failures = 0, written = 0;
	unsigned int block;
	const char *result = "ok";
	char record[MAX_RECORD_SIZE];
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

	if (write_record(fd, markers[0], strlen(markers[0])))
		failures++;
	else
		written++;

	for (block = 0; block < sizeof(pattern_lengths) /
				      sizeof(pattern_lengths[0]); block++) {
		int len = make_record(block, pattern_lengths[block], record,
				      sizeof(record));

		if (len < 0 || write_record(fd, record, (size_t)len))
			failures++;
		else
			written++;
	}

	if (write_record(fd, markers[1], strlen(markers[1])))
		failures++;
	else
		written++;

	close(fd);
	if (failures)
		result = "write_failed";
	report_result(result, written, failures);

wait:
	for (;;)
		pause();
}
