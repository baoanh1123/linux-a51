// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFS PHY driver data for Samsung Exynos9610
 *
 * The Exynos9610 MPHY uses the vendor UFS CAL sequence.  The PHY register
 * layout differs from Exynos7: PMA registers use direct byte offsets and the
 * second lane starts at +0x140.  Keep the calibration data here and let the
 * host driver's pre-link path supply the UIC/DME attributes.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "phy-samsung-ufs.h"

#define EXYNOS9610_EMBEDDED_COMBO_PHY_ISOL	0x724
#define EXYNOS9610_PHY_TRSV_CH_OFFSET		0x140

#define EXYNOS9610_COMN_CFG(o, v) {		\
	.off_0 = (o),				\
	.off_1 = 0,				\
	.val = (v),				\
	.desc = PWR_MODE_ANY,			\
	.id = PHY_COMN_BLK,			\
}

#define EXYNOS9610_TRSV_CFG(o, v) {		\
	.off_0 = (o),				\
	.off_1 = (o) + EXYNOS9610_PHY_TRSV_CH_OFFSET, \
	.val = (v),				\
	.desc = PWR_MODE_ANY,			\
	.id = PHY_TRSV_BLK,			\
}

/*
 * Direct PMA writes from the vendor init_cfg table.  UIC/PCS/DME entries are
 * programmed by the host pre-link path; this table is only the PHY half.
 * DOWNSTREAM_READONLY/drivers/scsi/ufs/ufs-cal-9610.c:110-168.
 */
static const struct samsung_ufs_phy_cfg exynos9610_pre_init_cfg[] = {
	EXYNOS9610_COMN_CFG(0x08c, 0x80),
	EXYNOS9610_COMN_CFG(0x074, 0x10),
	EXYNOS9610_TRSV_CFG(0x110, 0xb5),
	EXYNOS9610_TRSV_CFG(0x134, 0x43),
	EXYNOS9610_TRSV_CFG(0x16c, 0x20),
	EXYNOS9610_TRSV_CFG(0x178, 0xc0),
	EXYNOS9610_TRSV_CFG(0x1b0, 0x94),
	EXYNOS9610_TRSV_CFG(0x0e0, 0x12),
	EXYNOS9610_TRSV_CFG(0x164, 0x58),
	EXYNOS9610_COMN_CFG(0x08c, 0xc0),
	EXYNOS9610_COMN_CFG(0x08c, 0x00),
	END_UFS_PHY_CFG,
};

/*
 * HS-B Gear3 MPHY tuning from calib_of_hs_rate_b (vendor board = common).
 * Only the PMA entries are listed here; the UNIPRO/DBG MIB entries of
 * that table are programmed by the host pre/post power-mode-change
 * paths. 0x12c is 0x00: the 0x10 value is __BRD_SMDK-only and the
 * common entry is the G2 one (0x00). Mainline applies a whole tag
 * table unconditionally, so this table must hold exactly one mode.
 */
static const struct samsung_ufs_phy_cfg exynos9610_pre_pwr_hs_cfg[] = {
	EXYNOS9610_TRSV_CFG(0x0c8, 0xbc),
	EXYNOS9610_TRSV_CFG(0x0f0, 0x7f),
	EXYNOS9610_TRSV_CFG(0x120, 0xc0),
	EXYNOS9610_TRSV_CFG(0x128, 0x00),
	EXYNOS9610_TRSV_CFG(0x12c, 0x00),
	EXYNOS9610_TRSV_CFG(0x134, 0x63),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg *exynos9610_ufs_phy_cfgs[CFG_TAG_MAX] = {
	[CFG_PRE_INIT] = exynos9610_pre_init_cfg,
	[CFG_PRE_PWR_HS] = exynos9610_pre_pwr_hs_cfg,
};

static const char * const exynos9610_ufs_phy_clks[] = {
	"ref_clk",
};

static int exynos9610_ufs_phy_wait(struct phy *phy, u8 lane)
{
	/* Vendor init_cfg ends with a 0xc8 microsecond common wait. */
	udelay(300);

	return 0;
}

/*
 * Vendor post_calib_of_hs_rate_b waits for the CDR AFC lock bit
 * (PMA 0x1fc, mask 0x40). The generic mainline wait addresses the
 * status register as a word index (off << 2), which is wrong for
 * this byte-addressed PHY, so poll the byte offset directly. A
 * timeout is reported but not fatal: the host ignores the
 * phy_calibrate() result, and a missed lock must not fail probe.
 */
static int exynos9610_ufs_phy_wait_cdr(struct phy *phy, u8 lane)
{
	struct samsung_ufs_phy *ufs_phy = get_samsung_ufs_phy(phy);
	unsigned int val;
	int err;

	err = readl_poll_timeout(ufs_phy->reg_pma + 0x1fc, val,
				 (val & 0x40) == 0x40, 10, 100000);
	if (err)
		dev_err(ufs_phy->dev,
			"failed to get phy cdr afc lock acquisition %d\n", err);

	return err;
}

const struct samsung_ufs_phy_drvdata exynos9610_ufs_phy = {
	.cfgs = exynos9610_ufs_phy_cfgs,
	.isol = {
		.offset = EXYNOS9610_EMBEDDED_COMBO_PHY_ISOL,
		.mask = 0x1,
		.en = 0x1,
	},
	.clk_list = exynos9610_ufs_phy_clks,
	.num_clks = ARRAY_SIZE(exynos9610_ufs_phy_clks),
	.cdr_lock_status_offset = 0x1fc,
	.wait_for_cal = exynos9610_ufs_phy_wait,
	.wait_for_cdr = exynos9610_ufs_phy_wait_cdr,
};
