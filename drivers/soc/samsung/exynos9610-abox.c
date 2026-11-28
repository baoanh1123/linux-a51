// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 9610 ABOX bring-up.
 *
 * Powers DISPAUD through the genpd provider, programs PLL_AUD and the ABOX
 * bus/CPU clocks, brings up the ABOX GIC and IPC mailbox, maps the firmware
 * DRAM, private, SFR and VSS windows through the ABOX SysMMU, and starts the
 * Cortex-A7 firmware.  WLBT can then register its BT SCO shared buffer.
 *
 * This is still a bring-up driver: there is no PCM/RDMA/WDMA path and no
 * ALSA sound card yet.
 *
 * The domain is requested through runtime PM, so a machine that never probes
 * ABOX never touches the DISPAUD PMU registers.  That avoids hanging the
 * whole boot before display init if DISPAUD is left in a bad state by a
 * previous experiment.
 */

#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/memremap.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/string.h>

#define EXYNOS9610_ABOX_BUILD
#include "exynos9610-abox-bt.h"

#define ABOX_VERSION			0x0004
#define ABOX_SYSPOWER_CTRL		0x0010
#define ABOX_SYSPOWER_STATUS		0x0014

#define ABOX_IPC_TX_OFFSET		0x22000
#define ABOX_IPC_TX_ACK_OFFSET		0x222fc
#define ABOX_IPC_RX_OFFSET		0x22300
#define ABOX_IPC_RX_ACK_OFFSET		0x225fc
#define ABOX_BOOTARGS_OFFSET		0x22e00
#define ABOX_SRAM_SIZE			0x35000
#define ABOX_DRAM_SIZE			(SZ_16M + SZ_2M)
#define ABOX_FW_PHYS			0x80000000

#define ABOX_PMU_BASE			0x11860000
#define ABOX_CPU_CONFIGURATION		0x2520
#define ABOX_CPU_STATUS			0x2524
#define ABOX_CPU_OPTION			0x2528
#define ABOX_CPU_LOCAL_PWR_CFG		BIT(0)
#define ABOX_CPU_OPTION_ENABLE_CPU	BIT(15)

/* ABOX GIC (Cortex-A7 GIC-400, non-secure) */
#define GIC_DIST_CTRL			0x000
#define GIC_DIST_ENABLE_SET		0x100
#define GIC_DIST_ISPEND			0x200
#define GIC_DIST_IACTIVE		0x300
#define GIC_DIST_PRI			0x400
#define GIC_CPU_CTRL			0x000
#define GIC_CPU_PRIMASK			0x004
#define GIC_CPU_INTACK			0x00c
#define GIC_CPU_EOI			0x010
#define GIC_CPU_DEACTIVATE		0x1000

#define GIC_DIST_SOFTINT		0x0f00

#define ABOX_QCH_CPU			0x3000

/* One IPC mailbox slot is 0x2fc bytes; TX and RX are back to back. */
#define ABOX_IPC_MSG_SIZE		0x2fc
#define IPC_SYSTEM			1
#define ABOX_BT_SCO_ENABLE		0xd0
#define ABOX_IPC_TIMEOUT_US		10000

#define ABOX_IOVA_DRAM			0x80000000
#define ABOX_IOVA_PRIVATE		0xe0000000
#define ABOX_IOVA_SFR			0x10000000
#define ABOX_PRIVATE_SIZE		SZ_8M

#define ABOX_IOVA_VSS			0xa0000000
#define ABOX_VSS_PHYS			0xf7900000
#define ABOX_VSS_SIZE			(SZ_4M + SZ_2M + SZ_1M)

/* CMU_TOP */
#define TOP_GATE_DISPAUD_CPU		0x203c
#define TOP_GATE_DISPAUD_AUD		0x2038

/* CMU_DISPAUD */
#define AUD_PLL_LOCKTIME		0x0000
#define AUD_PLL_CON0			0x0160
#define AUD_PLL_CON3			0x016c
#define AUD_MUX_BUS			0x1000
#define AUD_MUX_CPU			0x1004
#define AUD_MUX_CPU_HCH		0x1008
#define AUD_DIV_AUDIF			0x1800
#define AUD_DIV_BUS			0x1808
#define AUD_DIV_CPU			0x180c
#define AUD_DIV_CPU_ACLK		0x1810
#define AUD_DIV_CPU_PCLKDBG		0x1814
#define AUD_GATE_DISPAUD_OSCCLK		0x201c
#define AUD_GATE_ABOX_ACLK		0x2020
#define AUD_GATE_CCLK_ASB		0x202c
#define AUD_GATE_CCLK_CA7		0x2030
#define AUD_GATE_LHM_AXI		0x2070
#define AUD_GATE_PERI_ACLKM		0x207c
#define AUD_GATE_PERI_PCLK		0x2080
#define AUD_GATE_RSTNSYNC_CPU_ACLK	0x2094
#define AUD_GATE_RSTNSYNC_CPU_CLKIN	0x2098
#define AUD_GATE_RSTNSYNC_CPU_PCLKDBG	0x209c
#define AUD_GATE_RSTNSYNC_AUD		0x20a4
#define AUD_GATE_SYSREG_PCLK		0x20b8
#define AUD_GATE_SMMU_ABOX		0x20b0
#define AUD_GATE_AXI_US		0x203c

#define GATE_ON				BIT(21)

#define AUD_PLL_ENABLE			BIT(31)
#define AUD_PLL_STABLE			BIT(29)
#define AUD_PLL_M			45
#define AUD_PLL_P			1
#define AUD_PLL_S			0
#define AUD_PLL_K			24319

struct exynos9610_abox_ipc_system {
	u32 msgtype;
	s32 param1;
	s32 param2;
	s32 param3;
	u8 bundle[740];
};

struct exynos9610_abox_ipc {
	u32 ipcid;
	u32 task_id;
	struct exynos9610_abox_ipc_system system;
};

static_assert(sizeof(struct exynos9610_abox_ipc) == ABOX_IPC_MSG_SIZE);
static_assert(sizeof(struct scsc_bt_audio_abox) == 0x4100);

struct exynos9610_abox {
	struct device *dev;
	void __iomem *sfr;
	void __iomem *cmu;
	void __iomem *top;
	void __iomem *sram;
	void __iomem *gicd;
	void __iomem *gicc;
	void __iomem *pmu;
	void *dram;
	u32 bootargs_offset;
	const char *bootargs;
	int irq;
	bool irq_requested;
	bool booted;
	struct mutex ipc_lock;
	struct iommu_domain *iommu_domain;
	struct page **priv_pages;
	size_t priv_nr_pages;
	size_t priv_size;
	phys_addr_t bt_paddr;
	dma_addr_t bt_iova;
	size_t bt_size;
	u32 bt_ref;
	int bt_active[2];
};

static struct exynos9610_abox *abox_priv;

static bool abox_boot;
module_param(abox_boot, bool, 0644);
MODULE_PARM_DESC(abox_boot, "Download firmware and start the ABOX CPU");

static bool abox_verbose;
module_param(abox_verbose, bool, 0644);
MODULE_PARM_DESC(abox_verbose, "Log every ABOX IPC message");

static void abox_gate(void __iomem *base, u32 off)
{
	writel(GATE_ON, base + off);
}

static void abox_set_div(void __iomem *base, u32 off, unsigned int div,
			 unsigned int width)
{
	u32 val = readl(base + off);

	val &= ~GENMASK(width - 1, 0);
	val |= (div - 1) & GENMASK(width - 1, 0);
	writel(val, base + off);
}

static void abox_set_mux(void __iomem *base, u32 off, unsigned int parent)
{
	u32 val = readl(base + off);

	val &= ~BIT(0);
	val |= parent & BIT(0);
	writel(val, base + off);
}

static int abox_setup_pll(struct exynos9610_abox *abox)
{
	u32 con0;
	int i;

	con0 = readl(abox->cmu + AUD_PLL_CON0);
	if ((con0 & AUD_PLL_ENABLE) && (con0 & AUD_PLL_STABLE) &&
	    ((con0 >> 16) & 0x3ff) == AUD_PLL_M &&
	    ((con0 >> 8) & 0x7) == AUD_PLL_P &&
	    (con0 & 0xf) == AUD_PLL_S) {
		dev_info(abox->dev, "PLL_AUD already locked, CON0=%#x\n", con0);
		return 0;
	}

	writel(150, abox->cmu + AUD_PLL_LOCKTIME);
	writel(AUD_PLL_K, abox->cmu + AUD_PLL_CON3);
	writel((AUD_PLL_M << 16) | (AUD_PLL_P << 8) | AUD_PLL_S,
	       abox->cmu + AUD_PLL_CON0);
	writel((AUD_PLL_M << 16) | (AUD_PLL_P << 8) | AUD_PLL_S |
	       AUD_PLL_ENABLE, abox->cmu + AUD_PLL_CON0);

	for (i = 0; i < 100; i++) {
		con0 = readl(abox->cmu + AUD_PLL_CON0);
		if (con0 & AUD_PLL_STABLE) {
			dev_info(abox->dev, "PLL_AUD locked, CON0=%#x\n", con0);
			return 0;
		}
		udelay(10);
	}

	dev_err(abox->dev, "PLL_AUD did not lock, CON0=%#x\n", con0);
	return -ETIMEDOUT;
}

static void abox_setup_rates(struct exynos9610_abox *abox)
{
	/* 1179.648 MHz PLL -> 1.179 GHz CPU, 294.9 MHz bus/ACLK. */
	abox_set_div(abox->cmu, AUD_DIV_CPU, 1, 4);
	abox_set_div(abox->cmu, AUD_DIV_BUS, 4, 3);
	abox_set_div(abox->cmu, AUD_DIV_AUDIF, 48, 9);
	abox_set_div(abox->cmu, AUD_DIV_CPU_ACLK, 4, 3);
	abox_set_div(abox->cmu, AUD_DIV_CPU_PCLKDBG, 8, 3);

	abox_set_mux(abox->cmu, AUD_MUX_BUS, 0);
	abox_set_mux(abox->cmu, AUD_MUX_CPU, 0);
	abox_set_mux(abox->cmu, AUD_MUX_CPU_HCH, 0);
}

static void abox_setup_clocks(struct exynos9610_abox *abox)
{
	/* Feed CMU_DISPAUD from CMU_TOP before touching its registers. */
	abox_gate(abox->top, TOP_GATE_DISPAUD_AUD);
	abox_gate(abox->top, TOP_GATE_DISPAUD_CPU);

	/* APB/AXI path to the ABOX register block. */
	abox_gate(abox->cmu, AUD_GATE_DISPAUD_OSCCLK);
	abox_gate(abox->cmu, AUD_GATE_SYSREG_PCLK);
	abox_gate(abox->cmu, AUD_GATE_SMMU_ABOX);
	abox_gate(abox->cmu, AUD_GATE_LHM_AXI);
	abox_gate(abox->cmu, AUD_GATE_AXI_US);
	abox_gate(abox->cmu, AUD_GATE_PERI_ACLKM);
	abox_gate(abox->cmu, AUD_GATE_PERI_PCLK);
	abox_gate(abox->cmu, AUD_GATE_ABOX_ACLK);
	abox_gate(abox->cmu, AUD_GATE_CCLK_ASB);
	abox_gate(abox->cmu, AUD_GATE_CCLK_CA7);
	abox_gate(abox->cmu, AUD_GATE_RSTNSYNC_CPU_ACLK);
	abox_gate(abox->cmu, AUD_GATE_RSTNSYNC_CPU_CLKIN);
	abox_gate(abox->cmu, AUD_GATE_RSTNSYNC_CPU_PCLKDBG);
	abox_gate(abox->cmu, AUD_GATE_RSTNSYNC_AUD);
}

static void abox_gic_init(struct exynos9610_abox *abox)
{
	int i;

	writel(0x000000ff, abox->gicc + GIC_CPU_PRIMASK);
	/* SGIs are enabled through the same ISENABLER0 bank as the PPIs. */
	writel(1, abox->gicd + GIC_DIST_ENABLE_SET);
	writel(0x3, abox->gicd + GIC_DIST_CTRL);
	for (i = 0; i < 40; i++)
		writel(0x10101010, abox->gicd + GIC_DIST_PRI + i * 4);
	writel(0x3, abox->gicc + GIC_CPU_CTRL);
}

static int abox_send_ipc(struct exynos9610_abox *abox,
			 struct exynos9610_abox_ipc *msg)
{
	void __iomem *tx = abox->sram + ABOX_IPC_TX_OFFSET;
	void __iomem *ack = abox->sram + ABOX_IPC_TX_ACK_OFFSET;
	int ret = 0, i;

	mutex_lock(&abox->ipc_lock);
	if (abox_verbose)
		dev_info(abox->dev,
			 "IPC send type=%#x ack=%#x gicd_ctrl=%#x gicc_ctrl=%#x primask=%#x isen=%#x pend=%#x act=%#x\n",
			 msg->system.msgtype, readl(ack),
			 readl(abox->gicd + GIC_DIST_CTRL),
			 readl(abox->gicc + GIC_CPU_CTRL),
			 readl(abox->gicc + GIC_CPU_PRIMASK),
			 readl(abox->gicd + GIC_DIST_ENABLE_SET),
			 readl(abox->gicd + GIC_DIST_ISPEND),
			 readl(abox->gicd + GIC_DIST_IACTIVE));
	memcpy_toio(tx, msg, sizeof(*msg));
	/* Publish the whole message before the doorbell ack. */
	wmb();
	writel(1, ack);
	/* The vendor passes msg.ipcid as the GIC interrupt number; for a
	 * system message that is IPC_SYSTEM (1), i.e. SGI 1, not SGI 0.
	 */
	writel((1 << 16) | (msg->ipcid & 0xf),
	       abox->gicd + GIC_DIST_SOFTINT);

	for (i = 0; i < ABOX_IPC_TIMEOUT_US; i++) {
		if (!readl(ack))
			break;
		udelay(1);
	}
	if (readl(ack)) {
		dev_warn(abox->dev, "IPC %#x timed out after %d us\n",
			 msg->system.msgtype, i);
		writel(0, ack);
		ret = -ETIMEDOUT;
	}
	mutex_unlock(&abox->ipc_lock);
	return ret;
}

static int abox_bt_sco_set(int dir, bool enable)
{
	struct exynos9610_abox *abox = abox_priv;
	struct exynos9610_abox_ipc msg = { };
	int ret;

	if (!abox || !abox->booted)
		return -ENODEV;
	if (dir < 0 || dir > 1)
		return -EINVAL;
	if (!abox->bt_paddr) {
		dev_warn(abox->dev, "BT SCO buffer is not registered\n");
		return -EIO;
	}

	msg.ipcid = IPC_SYSTEM;
	msg.system.msgtype = ABOX_BT_SCO_ENABLE;
	msg.system.param1 = enable;
	msg.system.param2 = (s32)abox->bt_iova;
	msg.system.param3 = dir;
	ret = abox_send_ipc(abox, &msg);
	if (!ret)
		abox->bt_active[dir] = enable;
	dev_info(abox->dev, "BT SCO %s %s: iova=%#llx ret=%d\n",
		 dir ? "mic" : "spk", enable ? "on" : "off",
		 (unsigned long long)abox->bt_iova, ret);
	return ret;
}

int exynos9610_abox_bt_register(phys_addr_t paddr, size_t size, u32 ref)
{
	struct exynos9610_abox *abox = abox_priv;
	size_t map_size;
	int ret;

	if (!abox || !abox->booted)
		return -ENODEV;
	if (!paddr || size < sizeof(struct scsc_bt_audio_abox))
		return -EINVAL;

	/* HCI open may register the same persistent buffer again. */
	if (abox->bt_size == size && abox->bt_paddr == paddr &&
	    abox->bt_iova == ABOX_IOVA_BLUETOOTH)
		return 0;

	if (abox->bt_size && abox->iommu_domain &&
	    abox->bt_iova == ABOX_IOVA_BLUETOOTH)
		iommu_unmap(abox->iommu_domain, ABOX_IOVA_BLUETOOTH,
			    PAGE_ALIGN(abox->bt_size));

	map_size = PAGE_ALIGN(size);
	if (abox->iommu_domain) {
		ret = iommu_map(abox->iommu_domain, ABOX_IOVA_BLUETOOTH, paddr,
				map_size, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		if (ret) {
			dev_err(abox->dev, "cannot map BT SCO buffer: %d\n", ret);
			abox->bt_paddr = 0;
			abox->bt_iova = 0;
			abox->bt_size = 0;
			return ret;
		}
		abox->bt_iova = ABOX_IOVA_BLUETOOTH;
	} else {
		abox->bt_iova = paddr;
	}

	abox->bt_paddr = paddr;
	abox->bt_size = size;
	abox->bt_ref = ref;
	dev_info(abox->dev, "BT SCO buffer: paddr=%pa iova=%#llx size=%zu ref=%#x\n",
		 &paddr, (unsigned long long)abox->bt_iova, size, ref);
	return 0;
}
EXPORT_SYMBOL_GPL(exynos9610_abox_bt_register);

void exynos9610_abox_bt_unregister(void)
{
	struct exynos9610_abox *abox = abox_priv;

	if (!abox)
		return;
	if (abox->iommu_domain && abox->bt_size &&
	    abox->bt_iova == ABOX_IOVA_BLUETOOTH)
		iommu_unmap(abox->iommu_domain, ABOX_IOVA_BLUETOOTH,
			    PAGE_ALIGN(abox->bt_size));
	abox->bt_paddr = 0;
	abox->bt_iova = 0;
	abox->bt_size = 0;
	abox->bt_ref = 0;
	abox->bt_active[0] = 0;
	abox->bt_active[1] = 0;
}
EXPORT_SYMBOL_GPL(exynos9610_abox_bt_unregister);

static int abox_bt_sco_param_set(const char *val,
				 const struct kernel_param *kp)
{
	int dir, enable;

	if (sscanf(val, "%d %d", &dir, &enable) != 2)
		return -EINVAL;
	if (dir < 0 || dir > 1 || enable < 0 || enable > 1)
		return -EINVAL;
	return abox_bt_sco_set(dir, enable);
}

static int abox_bt_sco_param_get(char *buf, const struct kernel_param *kp)
{
	struct exynos9610_abox *abox = abox_priv;

	if (!abox)
		return sysfs_emit(buf, "0 0\n");
	return sysfs_emit(buf, "%d %d\n", abox->bt_active[0],
			  abox->bt_active[1]);
}

static const struct kernel_param_ops abox_bt_sco_param_ops = {
	.set = abox_bt_sco_param_set,
	.get = abox_bt_sco_param_get,
};
module_param_cb(bt_sco, &abox_bt_sco_param_ops, NULL, 0644);
MODULE_PARM_DESC(bt_sco, "Enable ABOX BT SCO: <dir 0=spk 1=mic> <0|1>");
static irqreturn_t abox_irq(int irq, void *data)
{
	struct exynos9610_abox *abox = data;
	u32 iar, id;
	int n;

	for (n = 0; n < 32; n++) {
		iar = readl(abox->gicc + GIC_CPU_INTACK);
		id = iar & 0x3ff;
		if (id >= 1020)
			break;

		writel(iar, abox->gicc + GIC_CPU_EOI);
		writel(iar, abox->gicc + GIC_CPU_DEACTIVATE);

		if (id < 16) {
			u32 ipcid = readl(abox->sram + ABOX_IPC_RX_OFFSET);
			u32 msgtype = readl(abox->sram + ABOX_IPC_RX_OFFSET + 8);
			u32 p1 = readl(abox->sram + ABOX_IPC_RX_OFFSET + 12);
			u32 p2 = readl(abox->sram + ABOX_IPC_RX_OFFSET + 16);
			u32 p3 = readl(abox->sram + ABOX_IPC_RX_OFFSET + 20);

			if (msgtype == 0x3)
				dev_info(abox->dev,
					 "ABOX boot done: version=%#x string=%#x\n",
					 p1, p3);
			else if (abox_verbose)
				dev_info(abox->dev,
					 "ABOX IPC irq=%u id=%u ipcid=%u type=%#x p=%#x,%#x,%#x\n",
					 irq, id, ipcid, msgtype, p1, p2, p3);
			writel(0, abox->sram + ABOX_IPC_RX_ACK_OFFSET);
		} else {
			dev_info(abox->dev, "ABOX GIC irq=%u id=%u\n", irq, id);
		}
	}

	return IRQ_HANDLED;
}

static int abox_iommu_setup(struct exynos9610_abox *abox)
{
	struct iommu_domain *domain;
	size_t i;
	int ret;

	domain = iommu_get_domain_for_dev(abox->dev);
	if (!domain) {
		dev_warn(abox->dev, "no IOMMU domain; ABOX DMA stays physical\n");
		return 0;
	}
	abox->iommu_domain = domain;

	ret = iommu_map(domain, ABOX_IOVA_DRAM, ABOX_FW_PHYS, ABOX_DRAM_SIZE,
			IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret) {
		dev_err(abox->dev, "cannot map ABOX firmware DRAM: %d\n", ret);
		return ret;
	}

	abox->priv_size = ABOX_PRIVATE_SIZE;
	abox->priv_nr_pages = abox->priv_size / PAGE_SIZE;
	abox->priv_pages = kcalloc(abox->priv_nr_pages,
				   sizeof(*abox->priv_pages), GFP_KERNEL);
	if (!abox->priv_pages) {
		ret = -ENOMEM;
		goto err_dram;
	}

	for (i = 0; i < abox->priv_nr_pages; i++) {
		struct page *page;

		page = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!page) {
			dev_err(abox->dev, "cannot allocate ABOX private page %zu\n", i);
			ret = -ENOMEM;
			goto err_private;
		}
		abox->priv_pages[i] = page;
		ret = iommu_map(domain, ABOX_IOVA_PRIVATE + i * PAGE_SIZE,
				page_to_phys(page), PAGE_SIZE,
				IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		if (ret) {
			dev_err(abox->dev,
				"cannot map ABOX private page %zu: %d\n", i, ret);
			__free_page(page);
			abox->priv_pages[i] = NULL;
			goto err_private;
		}
	}

	ret = iommu_map(domain, ABOX_IOVA_SFR, ABOX_IOVA_SFR, PAGE_SIZE,
			IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret) {
		dev_err(abox->dev, "cannot map ABOX SFR window: %d\n", ret);
		goto err_private;
	}

	ret = iommu_map(domain, ABOX_IOVA_VSS, ABOX_VSS_PHYS, ABOX_VSS_SIZE,
			IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret) {
		dev_err(abox->dev, "cannot map ABOX VSS memory: %d\n", ret);
		goto err_sfr;
	}

	dev_info(abox->dev,
		 "IOMMU: firmware %#x -> %#x, private %#x (%zu pages)\n",
		 ABOX_IOVA_DRAM, ABOX_FW_PHYS, ABOX_IOVA_PRIVATE,
		 abox->priv_nr_pages);
	return 0;

err_sfr:
	iommu_unmap(domain, ABOX_IOVA_SFR, PAGE_SIZE);
err_private:
	while (i-- > 0) {
		iommu_unmap(domain, ABOX_IOVA_PRIVATE + i * PAGE_SIZE, PAGE_SIZE);
		__free_page(abox->priv_pages[i]);
		abox->priv_pages[i] = NULL;
	}
	kfree(abox->priv_pages);
	abox->priv_pages = NULL;
	abox->priv_nr_pages = 0;
err_dram:
	iommu_unmap(domain, ABOX_IOVA_DRAM, ABOX_DRAM_SIZE);
	return ret;
}

static void abox_iommu_teardown(struct exynos9610_abox *abox)
{
	size_t i;

	if (!abox->iommu_domain)
		return;
	if (abox->bt_size && abox->bt_iova == ABOX_IOVA_BLUETOOTH)
		iommu_unmap(abox->iommu_domain, ABOX_IOVA_BLUETOOTH,
			    PAGE_ALIGN(abox->bt_size));
	iommu_unmap(abox->iommu_domain, ABOX_IOVA_VSS, ABOX_VSS_SIZE);
	iommu_unmap(abox->iommu_domain, ABOX_IOVA_SFR, PAGE_SIZE);
	for (i = 0; i < abox->priv_nr_pages; i++) {
		if (!abox->priv_pages[i])
			continue;
		iommu_unmap(abox->iommu_domain,
			    ABOX_IOVA_PRIVATE + i * PAGE_SIZE, PAGE_SIZE);
		__free_page(abox->priv_pages[i]);
	}
	kfree(abox->priv_pages);
	abox->priv_pages = NULL;
	abox->priv_nr_pages = 0;
	iommu_unmap(abox->iommu_domain, ABOX_IOVA_DRAM, ABOX_DRAM_SIZE);
	abox->iommu_domain = NULL;
}

static int abox_boot_firmware(struct exynos9610_abox *abox)
{
	const struct firmware *sram_fw, *dram_fw;
	char bootargs[SZ_512];
	u32 val, status = 0;
	int i, ret;

	ret = request_firmware(&sram_fw, "calliope_sram.bin", abox->dev);
	if (ret) {
		dev_err(abox->dev, "cannot load calliope_sram.bin: %d\n", ret);
		return ret;
	}
	ret = request_firmware(&dram_fw, "calliope_dram.bin", abox->dev);
	if (ret) {
		dev_err(abox->dev, "cannot load calliope_dram.bin: %d\n", ret);
		release_firmware(sram_fw);
		return ret;
	}

	dev_info(abox->dev, "firmware: sram %zu bytes, dram %zu bytes\n",
		 sram_fw->size, dram_fw->size);
	if (sram_fw->size > ABOX_SRAM_SIZE || dram_fw->size > ABOX_DRAM_SIZE) {
		ret = -EINVAL;
		goto out;
	}

	/*
	 * A previous load may have left the ABOX CPU running.  Power it down
	 * before overwriting SRAM so the new firmware starts from reset.
	 */
	val = readl(abox->pmu + ABOX_CPU_CONFIGURATION);
	writel(val & ~ABOX_CPU_LOCAL_PWR_CFG,
	       abox->pmu + ABOX_CPU_CONFIGURATION);
	val = readl(abox->pmu + ABOX_CPU_OPTION);
	writel(val & ~ABOX_CPU_OPTION_ENABLE_CPU,
	       abox->pmu + ABOX_CPU_OPTION);
	for (i = 0; i < 100; i++) {
		status = readl(abox->pmu + ABOX_CPU_STATUS);
		if (!(status & BIT(0)))
			break;
		mdelay(1);
	}
	dev_info(abox->dev, "CPU powered down, STATUS=%#x after %d ms\n",
		 status, i);

	memcpy_toio(abox->sram, sram_fw->data, sram_fw->size);
	memset_io(abox->sram + sram_fw->size, 0,
		  ABOX_SRAM_SIZE - sram_fw->size);
	memcpy(abox->dram, dram_fw->data, dram_fw->size);
	memset(abox->dram + dram_fw->size, 0, ABOX_DRAM_SIZE - dram_fw->size);

	if (abox->bootargs && abox->bootargs_offset) {
		memset(bootargs, 0, sizeof(bootargs));
		strscpy(bootargs, abox->bootargs, sizeof(bootargs));
		memcpy_toio(abox->sram + abox->bootargs_offset, bootargs,
			    sizeof(bootargs));
		dev_info(abox->dev, "bootargs at SRAM+%#x: %s\n",
			 abox->bootargs_offset, bootargs);
	}

	abox_gic_init(abox);

	if (abox->irq > 0) {
		ret = request_irq(abox->irq, abox_irq, IRQF_TRIGGER_RISING,
				  "exynos9610-abox", abox);
		if (ret) {
			dev_warn(abox->dev, "cannot request ABOX IRQ %d: %d\n",
				 abox->irq, ret);
		} else {
			abox->irq_requested = true;
			dev_info(abox->dev, "ABOX IRQ %d requested\n",
				 abox->irq);
		}
	} else {
		dev_warn(abox->dev, "no ABOX IRQ in DT\n");
	}

	writel(0x3, abox->cmu + ABOX_QCH_CPU);

	writel(1, abox->sfr + ABOX_SYSPOWER_CTRL);
	for (i = 0; i < 100; i++) {
		status = readl(abox->sfr + ABOX_SYSPOWER_STATUS);
		if (status & BIT(0))
			break;
		udelay(10);
	}
	dev_info(abox->dev, "SYSPOWER_STATUS=%#x\n", status);

	val = readl(abox->pmu + ABOX_CPU_CONFIGURATION);
	writel(val | ABOX_CPU_LOCAL_PWR_CFG,
	       abox->pmu + ABOX_CPU_CONFIGURATION);
	val = readl(abox->pmu + ABOX_CPU_OPTION);
	writel(val | ABOX_CPU_OPTION_ENABLE_CPU,
	       abox->pmu + ABOX_CPU_OPTION);

	status = 0;
	for (i = 0; i < 100; i++) {
		status = readl(abox->pmu + ABOX_CPU_STATUS);
		if (status & BIT(0))
			break;
		mdelay(1);
	}
	dev_info(abox->dev, "CPU_STATUS=%#x after %d ms\n", status, i);

	mdelay(100);
	if (abox_verbose)
		for (i = 0; i < 12; i++)
			dev_info(abox->dev, "IPC RX[%02d]=%#010x\n", i,
				 readl(abox->sram + ABOX_IPC_RX_OFFSET + i * 4));

	ret = (status & BIT(0)) ? 0 : -ETIMEDOUT;
out:
	release_firmware(dram_fw);
	release_firmware(sram_fw);
	return ret;
}

static int exynos9610_abox_probe(struct platform_device *pdev)
{
	struct exynos9610_abox *abox;
	u32 version;
	int ret;

	abox = devm_kzalloc(&pdev->dev, sizeof(*abox), GFP_KERNEL);
	if (!abox)
		return -ENOMEM;

	abox->dev = &pdev->dev;
	mutex_init(&abox->ipc_lock);
	abox->sfr = devm_platform_ioremap_resource_byname(pdev, "sfr");
	if (IS_ERR(abox->sfr))
		return PTR_ERR(abox->sfr);
	abox->cmu = devm_platform_ioremap_resource_byname(pdev, "cmu");
	if (IS_ERR(abox->cmu))
		return PTR_ERR(abox->cmu);
	abox->top = devm_platform_ioremap_resource_byname(pdev, "top");
	if (IS_ERR(abox->top))
		return PTR_ERR(abox->top);
	abox->sram = devm_platform_ioremap_resource_byname(pdev, "sram");
	if (IS_ERR(abox->sram))
		return PTR_ERR(abox->sram);
	abox->gicd = devm_platform_ioremap_resource_byname(pdev, "gicd");
	if (IS_ERR(abox->gicd))
		return PTR_ERR(abox->gicd);
	abox->gicc = devm_platform_ioremap_resource_byname(pdev, "gicc");
	if (IS_ERR(abox->gicc))
		return PTR_ERR(abox->gicc);

	abox->pmu = devm_ioremap(&pdev->dev, ABOX_PMU_BASE, 0x10000);
	if (!abox->pmu)
		return -ENOMEM;

	abox->irq = platform_get_irq(pdev, 0);
	if (abox->irq < 0)
		dev_warn(&pdev->dev, "no ABOX IRQ: %d\n", abox->irq);

	of_property_read_u32(pdev->dev.of_node, "samsung,abox-bootargs-offset",
			      &abox->bootargs_offset);
	of_property_read_string(pdev->dev.of_node, "samsung,abox-bootargs",
				&abox->bootargs);

	platform_set_drvdata(pdev, abox);
	abox_priv = abox;

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot power DISPAUD: %d\n", ret);
		goto err_rpm;
	}

	abox_setup_clocks(abox);

	ret = abox_setup_pll(abox);
	if (ret)
		goto err_put;

	abox_setup_rates(abox);

	version = readl(abox->sfr + ABOX_VERSION);
	dev_info(&pdev->dev, "ABOX version %#08x\n", version);
	{
		void __iomem *smmu = devm_ioremap(&pdev->dev, 0x14920000, 0x9000);

		if (smmu)
			dev_info(&pdev->dev, "SMMU_ABOX version=%#x ctrl=%#x\n",
				 readl(smmu + 0x34), readl(smmu));
	}
	if (!version || version == 0xffffffff) {
		dev_err(&pdev->dev, "ABOX did not answer\n");
		ret = -ENODEV;
		goto err_put;
	}

	dev_info(&pdev->dev, "SRAM words %#x %#x %#x %#x, GICD=%#x GICC=%#x\n",
		 readl(abox->sram), readl(abox->sram + 4),
		 readl(abox->sram + 8), readl(abox->sram + 12),
		 readl(abox->gicd + GIC_DIST_CTRL),
		 readl(abox->gicc + GIC_CPU_CTRL));

	ret = abox_iommu_setup(abox);
	if (ret)
		goto err_put;

	if (abox_boot) {
		abox->dram = memremap(ABOX_FW_PHYS, ABOX_DRAM_SIZE,
				      MEMREMAP_WC);
		if (!abox->dram) {
			dev_err(&pdev->dev, "cannot map ABOX firmware DRAM\n");
			goto err_put;
		}

		ret = abox_boot_firmware(abox);
		if (ret)
			dev_err(&pdev->dev,
				"ABOX boot did not complete: %d; hardware left running\n",
				ret);
		else
			abox->booted = true;
		ret = 0;
	}

	pm_runtime_put(&pdev->dev);
	return 0;

err_put:
	pm_runtime_put(&pdev->dev);
err_rpm:
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static void exynos9610_abox_remove(struct platform_device *pdev)
{
	struct exynos9610_abox *abox = platform_get_drvdata(pdev);

	if (abox->irq_requested)
		free_irq(abox->irq, abox);

	abox->booted = false;
	if (abox_priv == abox)
		abox_priv = NULL;

	abox_iommu_teardown(abox);
	pm_runtime_disable(&pdev->dev);
}

static const struct of_device_id exynos9610_abox_of_match[] = {
	{ .compatible = "samsung,exynos9610-abox" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos9610_abox_of_match);

static struct platform_driver exynos9610_abox_driver = {
	.probe = exynos9610_abox_probe,
	.remove = exynos9610_abox_remove,
	.driver = {
		.name = "exynos9610-abox",
		.of_match_table = exynos9610_abox_of_match,
	},
};
module_platform_driver(exynos9610_abox_driver);

MODULE_DESCRIPTION("Exynos 9610 ABOX bring-up");
MODULE_LICENSE("GPL");
