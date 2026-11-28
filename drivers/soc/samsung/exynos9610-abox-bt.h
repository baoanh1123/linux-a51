/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Shared-memory interface between the WLBT Bluetooth firmware and the ABOX
 * audio DSP.  The layout is the one the vendor's scsc bt_audio.h defines; the
 * Bluetooth firmware gets the MIF-relative offset, the ABOX gets the physical
 * address of the same buffer.
 */
#ifndef __EXYNOS9610_ABOX_BT_H
#define __EXYNOS9610_ABOX_BT_H

#include <linux/errno.h>
#include <linux/types.h>

#define SCSC_BT_AUDIO_ABOX_VERSION_MAJOR	1
#define SCSC_BT_AUDIO_ABOX_VERSION_MINOR	1
#define SCSC_BT_AUDIO_ABOX_DCACHE_LINE_WIDTH	64
#define SCSC_BT_AUDIO_ABOX_DATA_SIZE		(128 * SCSC_BT_AUDIO_ABOX_DCACHE_LINE_WIDTH)
#define SCSC_BT_AUDIO_ABOX_IF_0_SIZE		(10 * SCSC_BT_AUDIO_ABOX_DCACHE_LINE_WIDTH)
#define SCSC_BT_AUDIO_ABOX_IF_1_SIZE		(10 * SCSC_BT_AUDIO_ABOX_DCACHE_LINE_WIDTH)

struct scsc_bt_audio_abox {
	u32 magic_value;
	u16 version_major;
	u16 version_minor;
	u8 reserved1[0x18];

	u32 abox_to_bt_streaming_if_0_size;
	u32 abox_to_bt_streaming_if_0_offset;
	u32 bt_to_abox_streaming_if_0_size;
	u32 bt_to_abox_streaming_if_0_offset;
	u32 abox_to_bt_streaming_if_1_size;
	u32 abox_to_bt_streaming_if_1_offset;
	u32 bt_to_abox_streaming_if_1_size;
	u32 bt_to_abox_streaming_if_1_offset;
	u8 reserved2[0x40];

	u32 abox_fw_features;
	u32 bt_to_abox_streaming_if_0_current_index;
	u32 abox_to_bt_streaming_if_0_current_index;
	u32 bt_to_abox_streaming_if_1_current_index;
	u32 abox_to_bt_streaming_if_1_current_index;
	u8 reserved3[0x2c];

	u32 bt_fw_features;
	u32 streaming_if_0_sample_rate;
	u32 streaming_if_1_sample_rate;
	u8 reserved4[0x34];

	u8 abox_to_bt_streaming_if_data[SCSC_BT_AUDIO_ABOX_DATA_SIZE];
	u8 bt_to_abox_streaming_if_data[SCSC_BT_AUDIO_ABOX_DATA_SIZE];
};

#define SCSC_BT_AUDIO_ABOX_MAGIC_VALUE \
	(((offsetof(struct scsc_bt_audio_abox, abox_to_bt_streaming_if_0_size) << 20) | \
	  (offsetof(struct scsc_bt_audio_abox, bt_to_abox_streaming_if_0_current_index) << 10) | \
	  offsetof(struct scsc_bt_audio_abox, abox_to_bt_streaming_if_data)) ^ 0xBA12EF82)

#define ABOX_IOVA_BLUETOOTH	0xb0000000

#if IS_ENABLED(CONFIG_EXYNOS9610_ABOX)
int exynos9610_abox_bt_register(phys_addr_t paddr, size_t size, u32 ref);
void exynos9610_abox_bt_unregister(void);
#else
static inline int exynos9610_abox_bt_register(phys_addr_t paddr, size_t size,
					      u32 ref) { return -ENODEV; }
static inline void exynos9610_abox_bt_unregister(void) { }
#endif

#endif /* __EXYNOS9610_ABOX_BT_H */
