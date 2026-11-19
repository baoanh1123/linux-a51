// SPDX-License-Identifier: GPL-2.0-only
/*
 * PID 1 that writes deterministic printable patterns through printk (ramoops
 * console zone) and through pmsg (ramoops pmsg zone), so the physical
 * persistent-memory region can be checked for data-integrity errors.
 *
 * The pattern generators below must stay in sync with the offline verifier.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/reboot.h>
#include <sys/stat.h>
#include <unistd.h>

#define PATTERN_COUNT	4
#define PATTERN_DATA	880
#define RECORD_SIZE	1024

/* The patterns walk the 95 printable ASCII characters in code-point order. */
#define PATTERN_FIRST_CHAR	' '
#define PATTERN_CHARS		95

/*
 * The same 128 KiB pattern is written twice, back to back, so the two copies
 * sit at different physical offsets inside the pmsg zone. Comparing their
 * corruption maps tells an address-dependent fault apart from a write-path or
 * data-dependent one. 256 KiB stays below the 512 KiB zone, so no wrap.
 */
#define PMSG_COPY_BYTES		(128 * 1024)
#define PMSG_COPIES		2
#define PMSG_PATTERN_BYTES	(PMSG_COPY_BYTES * PMSG_COPIES)
#define PMSG_CHUNK_BYTES	4096

/*
 * Ramoops is mapped cacheable here, so force the written lines out to DRAM
 * before the board is reset: touch well beyond the CPU cache size.
 */
#define EVICT_BYTES		(8 * 1024 * 1024)

static unsigned char evict_buffer[EVICT_BYTES];

/* Filled in by self_check() for the reboot decision below. */
static int pmsg_pattern_seen;
static int stage1_seen;

static void report_failure(const char *message);

static void evict_cache(void)
{
	unsigned int checksum = 0;
	size_t i;

	memset(evict_buffer, 0xa5, sizeof(evict_buffer));

	/* Consume one byte per cache line so the stores cannot be dropped. */
	for (i = 0; i < sizeof(evict_buffer); i += 64)
		checksum += evict_buffer[i];

	if (checksum == 0xdeadbeef)
		report_failure("RAMOOPS-CONSOLE-ONLY: cache eviction checksum\n");
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

static void enable_devkmsg(void)
{
	int fd = open("/proc/sys/kernel/printk_devkmsg", O_WRONLY | O_CLOEXEC);

	if (fd < 0)
		return;

	/* Best effort: disable the userspace printk ratelimit and off switch. */
	write(fd, "on", 2);
	close(fd);
}

static int write_all(int fd, const void *buffer, size_t len)
{
	const char *cursor = buffer;
	size_t done = 0;

	while (done < len) {
		ssize_t written = write(fd, cursor + done, len - done);

		if (written <= 0)
			return -1;
		done += (size_t)written;
	}

	return 0;
}

static int write_marker(int fd, const char *marker)
{
	char record[RECORD_SIZE];
	int formatted = snprintf(record, sizeof(record), "<6>%s\n", marker);

	if (formatted < 0 || (size_t)formatted >= sizeof(record))
		return -1;

	return write_all(fd, record, (size_t)formatted);
}

static int write_pattern(int fd, unsigned int index)
{
	char record[RECORD_SIZE];
	size_t pos;
	unsigned int i;
	int header;

	header = snprintf(record, sizeof(record), "<6>CONSOLEPAT-%u-LEN%u: ",
			  index, PATTERN_DATA);
	if (header < 0 || (size_t)header >= sizeof(record))
		return -1;

	pos = (size_t)header;
	for (i = 0; i < PATTERN_DATA; i++)
		record[pos++] = PATTERN_FIRST_CHAR +
			(i * 37 + index * 53) % PATTERN_CHARS;

	record[pos++] = '\n';

	return write_all(fd, record, pos);
}

static unsigned char pmsg_expected_byte(size_t pos)
{
	return PATTERN_FIRST_CHAR +
		(pos * 37 + (pos / 1024) * 53) % PATTERN_CHARS;
}

static int write_pmsg_pattern(int fd)
{
	unsigned char chunk[PMSG_CHUNK_BYTES];
	unsigned int copy;

	for (copy = 0; copy < PMSG_COPIES; copy++) {
		size_t offset;

		for (offset = 0; offset < PMSG_COPY_BYTES;
		     offset += sizeof(chunk)) {
			size_t chunk_len = PMSG_COPY_BYTES - offset;
			size_t i;

			if (chunk_len > sizeof(chunk))
				chunk_len = sizeof(chunk);

			for (i = 0; i < chunk_len; i++)
				chunk[i] = pmsg_expected_byte(offset + i);

			if (write_all(fd, chunk, chunk_len))
				return -1;
		}
	}

	return 0;
}

static void report_failure(const char *message)
{
	int fd = open("/dev/console", O_WRONLY | O_CLOEXEC);

	if (fd < 0)
		return;

	write(fd, message, strlen(message));
	close(fd);
}

#define STAGE1_MARKER	"SELFCHECK-STAGE1-7A51"

static int read_file(const char *path, unsigned char *buffer, size_t size)
{
	int fd = open(path, O_RDONLY | O_CLOEXEC);
	ssize_t got;

	if (fd < 0)
		return -1;

	got = read(fd, buffer, size);
	close(fd);

	return got < 0 ? -1 : (int)got;
}

/* Counts per copy plus how the two copies' corruption maps relate. */
struct pmsg_check {
	size_t bad[PMSG_COPIES];
	size_t both;
	size_t only_first;
	size_t only_second;
};

static void verify_pmsg_pattern(const unsigned char *data, struct pmsg_check *out)
{
	size_t i;

	memset(out, 0, sizeof(*out));

	for (i = 0; i < PMSG_COPY_BYTES; i++) {
		unsigned char expected = pmsg_expected_byte(i);
		int first = data[i] != expected;
		int second = data[PMSG_COPY_BYTES + i] != expected;

		if (first)
			out->bad[0]++;
		if (second)
			out->bad[1]++;

		if (first && second)
			out->both++;
		else if (first)
			out->only_first++;
		else if (second)
			out->only_second++;
	}
}

static size_t verify_console_patterns(const unsigned char *data, size_t len)
{
	size_t bad = 0;
	unsigned int index;

	for (index = 0; index < PATTERN_COUNT; index++) {
		char header[32];
		const unsigned char *found;
		int header_len;
		size_t i;

		header_len = snprintf(header, sizeof(header),
				      "CONSOLEPAT-%u-LEN%u: ", index,
				      PATTERN_DATA);
		if (header_len < 0 || (size_t)header_len >= sizeof(header))
			continue;

		found = memmem(data, len, header, (size_t)header_len);
		if (!found)
			continue;

		found += header_len;
		for (i = 0; i < PATTERN_DATA; i++) {
			unsigned char expected = PATTERN_FIRST_CHAR +
				(i * 37 + index * 53) % PATTERN_CHARS;

			if (found + i >= data + len || found[i] != expected)
				bad++;
		}
	}

	return bad;
}

/*
 * Verify the records the previous boot left behind, using this kernel's own
 * read path, and report the result through printk so recovery can read it.
 */
static void self_check(int kmsg_fd)
{
	static unsigned char record[PMSG_PATTERN_BYTES];
	struct pmsg_check check;
	size_t con_bad = 0;
	int con_len = 0;
	int pmsg_len = 0;
	char line[192];
	int len;

	memset(&check, 0, sizeof(check));

	if (mkdir("/pstore", 0755) && errno != EEXIST)
		return;

	if (mount("pstore", "/pstore", "pstore", 0, NULL) && errno != EBUSY)
		return;

	pmsg_len = read_file("/pstore/pmsg-ramoops-0", record, sizeof(record));
	if (pmsg_len == PMSG_PATTERN_BYTES)
		verify_pmsg_pattern(record, &check);

	pmsg_pattern_seen = pmsg_len == PMSG_PATTERN_BYTES;

	con_len = read_file("/pstore/console-ramoops-0", record, sizeof(record));
	if (con_len > 0) {
		con_bad = verify_console_patterns(record, (size_t)con_len);
		stage1_seen = !!memmem(record, (size_t)con_len, STAGE1_MARKER,
				       sizeof(STAGE1_MARKER) - 1);
	}

	len = snprintf(line, sizeof(line),
		       "<6>SELFCHECK pmsg_len=%d bad0=%zu bad1=%zu both=%zu only0=%zu only1=%zu con_len=%d con_bad=%zu stage1=%d\n",
		       pmsg_len, check.bad[0], check.bad[1], check.both,
		       check.only_first, check.only_second, con_len, con_bad,
		       stage1_seen);
	if (len < 0 || (size_t)len >= sizeof(line))
		return;

	if (write_all(kmsg_fd, line, (size_t)len))
		report_failure("RAMOOPS-CONSOLE-ONLY: selfcheck report failed\n");
}

int main(void)
{
	static const char *const markers[] = {
		"RAMOOPS-CONSOLE-ONLY-BEGIN-9C21",
		"RAMOOPS-CONSOLE-ONLY-ASCII-000: 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz",
		"RAMOOPS-CONSOLE-ONLY-ASCII-001: abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ9876543210",
		NULL,
	};
	unsigned int index;
	unsigned int i;
	int fd;

	if (mount_devtmpfs()) {
		report_failure("RAMOOPS-CONSOLE-ONLY: devtmpfs mount failed\n");
		goto wait;
	}

	enable_devkmsg();

	fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
	if (fd < 0) {
		report_failure("RAMOOPS-CONSOLE-ONLY: cannot open /dev/kmsg\n");
		goto pmsg;
	}

	self_check(fd);

	for (i = 0; markers[i]; i++) {
		if (write_marker(fd, markers[i])) {
			report_failure("RAMOOPS-CONSOLE-ONLY: printk marker write failed\n");
			goto out;
		}
	}

	for (index = 0; index < PATTERN_COUNT; index++) {
		if (write_pattern(fd, index)) {
			report_failure("RAMOOPS-CONSOLE-ONLY: pattern write failed\n");
			goto out;
		}
	}

	if (write_marker(fd, "RAMOOPS-CONSOLE-ONLY-END-9C21"))
		report_failure("RAMOOPS-CONSOLE-ONLY: end marker write failed\n");

out:
	close(fd);

pmsg:
	fd = open("/dev/pmsg0", O_WRONLY | O_CLOEXEC);
	if (fd < 0) {
		report_failure("RAMOOPS-CONSOLE-ONLY: cannot open /dev/pmsg0\n");
		goto wait;
	}

	if (write_pmsg_pattern(fd))
		report_failure("RAMOOPS-CONSOLE-ONLY: pmsg pattern write failed\n");

	close(fd);
	evict_cache();

	/*
	 * First boot of a pair: tell the next boot where to look, then reboot into
	 * this same image so MAINLINE itself verifies the records it just wrote.
	 * The marker keeps a device that never retains the data from looping.
	 */
	if (!pmsg_pattern_seen && !stage1_seen) {
		fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
		if (fd >= 0) {
			write_marker(fd, STAGE1_MARKER);
			close(fd);
		}

		sync();
		sleep(3);
		reboot(RB_AUTOBOOT);
	}

wait:
	for (;;)
		pause();
}
