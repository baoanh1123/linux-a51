/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos mailbox message.
 *
 * Copyright 2024 Linaro Ltd.
 */

#ifndef _LINUX_EXYNOS_MESSAGE_H_
#define _LINUX_EXYNOS_MESSAGE_H_

#define EXYNOS_MBOX_CHAN_TYPE_DOORBELL		0
#define EXYNOS_MBOX_CHAN_TYPE_DATA		1

struct exynos_mbox_msg {
	unsigned int chan_id;
	unsigned int chan_type;
};

#endif /* _LINUX_EXYNOS_MESSAGE_H_ */

struct mbox_chan;

#if IS_ENABLED(CONFIG_EXYNOS_MBOX)
u32 exynos_mbox_int_status(struct mbox_chan *chan);
#else
static inline u32 exynos_mbox_int_status(struct mbox_chan *chan)
{
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_EXYNOS_MBOX)
void exynos_mbox_chan_ack(struct mbox_chan *chan, unsigned int chan_id);
#else
static inline void exynos_mbox_chan_ack(struct mbox_chan *chan,
					unsigned int chan_id)
{
}
#endif
