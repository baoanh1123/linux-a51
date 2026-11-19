// SPDX-License-Identifier: GPL-2.0-only
/*
 * PID 1 for a bounded Volume Up suspend-to-idle wake test.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <unistd.h>

#define INPUT_DEVICE_NAME "a51-volume-up-eint-test"
#define INPUT_DEVICE_PATH "/dev/input/event"
#define INPUT_DEVICE_COUNT 32
#define INPUT_DEVICE_RETRIES 10
#define LINE_SIZE 256
#define WAKE_TIMEOUT_MS 30000

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

	len = snprintf(kmsg, sizeof(kmsg), "<6>Exynos9611-EINT-WAKE: %s\n",
		       message);
	if (len >= sizeof(kmsg) || write_all(kmsg_fd, kmsg, len)) {
		static const char warning[] =
			"WARN: /dev/kmsg write failed; further status is console-only\n";

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

static int open_volume_up_event(void)
{
	char path[LINE_SIZE];
	char name[LINE_SIZE];
	unsigned int attempt;
	unsigned int index;

	for (attempt = 0; attempt < INPUT_DEVICE_RETRIES; attempt++) {
		for (index = 0; index < INPUT_DEVICE_COUNT; index++) {
			int fd;

			snprintf(path, sizeof(path), "%s%u", INPUT_DEVICE_PATH,
				 index);
			fd = open(path, O_RDONLY | O_CLOEXEC | O_NONBLOCK);
			if (fd < 0)
				continue;

			if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) >= 0 &&
			    !strcmp(name, INPUT_DEVICE_NAME))
				return fd;

			close(fd);
		}

		sleep(1);
	}

	report("FAIL: could not find the Volume Up input device");
	return -1;
}

static int enter_freeze(void)
{
	static const char state[] = "freeze\n";
	int fd;

	fd = open("/sys/power/state", O_WRONLY | O_CLOEXEC);
	if (fd < 0) {
		char message[LINE_SIZE];

		snprintf(message, sizeof(message), "FAIL: open /sys/power/state: %s",
			 strerror(errno));
		report(message);
		return -1;
	}

	report("READY: entering suspend-to-idle; press and release Volume Up to wake");
	if (write_all(fd, state, sizeof(state) - 1)) {
		char message[LINE_SIZE];

		snprintf(message, sizeof(message), "FAIL: request freeze: %s",
			 strerror(errno));
		report(message);
		close(fd);
		return -1;
	}

	close(fd);
	report("RESUMED: kernel returned from suspend-to-idle");
	return 0;
}

static int wait_for_press_release(int fd)
{
	struct pollfd poll_fd = {
		.fd = fd,
		.events = POLLIN,
	};
	bool pressed = false;

	for (;;) {
		struct input_event event;
		char message[LINE_SIZE];
		int ret;
		ssize_t bytes;

		ret = poll(&poll_fd, 1, WAKE_TIMEOUT_MS);
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			snprintf(message, sizeof(message), "FAIL: input poll: %s",
				 strerror(errno));
			report(message);
			return -1;
		}
		if (!ret) {
			report("FAIL: timed out waiting for Volume Up press/release after resume");
			return -1;
		}
		if (poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
			report("FAIL: Volume Up input device stopped");
			return -1;
		}

		bytes = read(fd, &event, sizeof(event));
		if (bytes < 0) {
			if (errno == EINTR || errno == EAGAIN)
				continue;
			snprintf(message, sizeof(message), "FAIL: input read: %s",
				 strerror(errno));
			report(message);
			return -1;
		}
		if (bytes != sizeof(event)) {
			report("FAIL: incomplete input event");
			return -1;
		}
		if (event.type != EV_KEY || event.code != KEY_VOLUMEUP)
			continue;

		if (event.value == 1) {
			pressed = true;
			report("EVENT: Volume Up press delivered after resume");
		} else if (event.value == 0 && pressed) {
			report("EVENT: Volume Up release delivered after resume");
			return 0;
		}
	}
}

static void wait_forever(void)
{
	for (;;)
		pause();
}

int main(void)
{
	int event_fd;

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

	report("Exynos9611 Volume Up suspend-to-idle wake test started");
	event_fd = open_volume_up_event();
	if (event_fd < 0) {
		report("TEST RESULT: FAIL");
		wait_forever();
	}

	if (enter_freeze()) {
		report("TEST RESULT: FAIL");
		close(event_fd);
		wait_forever();
	}

	report(wait_for_press_release(event_fd) ?
	       "TEST RESULT: FAIL" :
	       "TEST RESULT: PASS (Volume Up woke suspend-to-idle)");
	close(event_fd);
	wait_forever();
}
