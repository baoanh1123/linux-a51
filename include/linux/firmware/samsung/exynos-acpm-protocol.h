/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020 Samsung Electronics Co., Ltd.
 * Copyright 2020 Google LLC.
 * Copyright 2024 Linaro Ltd.
 */

#ifndef __EXYNOS_ACPM_PROTOCOL_H
#define __EXYNOS_ACPM_PROTOCOL_H

#include <linux/types.h>

struct acpm_handle;
struct device_node;

struct acpm_pmic_ops {
	int (*read_reg)(struct acpm_handle *handle, unsigned int acpm_chan_id,
			u8 type, u8 reg, u8 chan, u8 *buf);
	int (*bulk_read)(struct acpm_handle *handle, unsigned int acpm_chan_id,
			 u8 type, u8 reg, u8 chan, u8 count, u8 *buf);
	int (*write_reg)(struct acpm_handle *handle, unsigned int acpm_chan_id,
			 u8 type, u8 reg, u8 chan, u8 value);
	int (*bulk_write)(struct acpm_handle *handle, unsigned int acpm_chan_id,
			  u8 type, u8 reg, u8 chan, u8 count, const u8 *buf);
	int (*update_reg)(struct acpm_handle *handle, unsigned int acpm_chan_id,
			  u8 type, u8 reg, u8 chan, u8 value, u8 mask);
};

struct acpm_ops {
	struct acpm_pmic_ops pmic_ops;
};

/**
 * struct acpm_handle - Reference to an initialized protocol instance
 * @ops:
 */
struct acpm_handle {
	struct acpm_ops ops;
};

struct device;

struct acpm_handle *devm_acpm_get_by_node(struct device *dev,
					  struct device_node *np);

/**
 * enum acpm_front_mode - how a TYPE_BUFFER channel advertises a new message
 * @ACPM_FRONT_REAR_NEXT: publish rear + 1, a raw counter the firmware compares
 * @ACPM_FRONT_VENDOR: replicate the vendor driver, which writes
 *		      (front + 1) % q_len and therefore leaves a single-slot
 *		      buffer's front at zero and relies on the doorbell alone
 */
enum acpm_front_mode {
	ACPM_FRONT_REAR_NEXT = 0,
	ACPM_FRONT_VENDOR,
};

/**
 * struct acpm_xfer - one ACPM message exchange
 * @txd: request words
 * @rxd: buffer for the response words
 * @txlen: request length in bytes
 * @rxlen: response buffer length in bytes
 * @acpm_chan_id: ACPM channel id
 * @no_wait: post the request and return instead of waiting for a response
 *
 * Some firmware channels are fire-and-forget: the request is answered by the
 * firmware acting on it, and waiting for a response word would only time out
 * on a message that was delivered.  DVFS is one of those.
 */
struct acpm_xfer {
	const u32 *txd;
	u32 *rxd;
	size_t txlen;
	size_t rxlen;
	unsigned int acpm_chan_id;
	bool no_wait;
	enum acpm_front_mode front_mode;
};

int acpm_do_xfer(struct acpm_handle *handle,
		 const struct acpm_xfer *xfer);

/**
 * acpm_do_cmd() - exchange one raw message with the ACPM firmware
 * @handle: ACPM handle
 * @acpm_chan_id: ACPM channel id
 * @cmd: message words, also used for the response in place
 * @words: number of 32-bit words in @cmd
 *
 * Sends @cmd to the firmware and waits for the reply, which is written back
 * into @cmd.  The firmware expects the request words to start with a
 * command-specific payload ({id, value, command, arg} for the DVFS channel).
 */
int acpm_do_cmd(struct acpm_handle *handle, unsigned int acpm_chan_id,
		u32 *cmd, unsigned int words);

#endif /* __EXYNOS_ACPM_PROTOCOL_H */
