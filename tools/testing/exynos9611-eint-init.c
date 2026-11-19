// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal PID 1 for validating physical Volume Up EINT delivery.
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
#define EDGE_TIMEOUT_MS 60000

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

	len = snprintf(kmsg, sizeof(kmsg), "<6>Exynos9611-EINT: %s\n", message);
	if (len >= sizeof(kmsg) || write_all(kmsg_fd, kmsg, len)) {
		static const char warning[] =
			"WARN: /dev/kmsg write failed; pstore console logging stopped\n";

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

static int wait_for_press_release(int fd)
{
	struct pollfd poll_fd = {
		.fd = fd,
		.events = POLLIN,
	};
	bool pressed = false;

	report("Press and release Volume Up within 60 seconds");

	for (;;) {
		struct input_event event;
		char message[LINE_SIZE];
		int ret;
		ssize_t bytes;

		ret = poll(&poll_fd, 1, EDGE_TIMEOUT_MS);
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			snprintf(message, sizeof(message), "FAIL: input poll: %s",
				 strerror(errno));
			report(message);
			return -1;
		}
		if (!ret) {
			report("FAIL: timed out waiting for Volume Up press/release");
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
			report("EVENT: Volume Up pressed");
		} else if (event.value == 0 && pressed) {
			report("EVENT: Volume Up released");
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

	report("Exynos9611 Volume Up EINT test started");

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

	event_fd = open_volume_up_event();
	if (event_fd < 0) {
		report("TEST RESULT: FAIL");
		wait_forever();
	}

	report(wait_for_press_release(event_fd) ?
	       "TEST RESULT: FAIL" : "TEST RESULT: PASS");
	close(event_fd);
	wait_forever();
}
