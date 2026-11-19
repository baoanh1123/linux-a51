// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM/KMS driver for the Samsung Exynos 9610/9611 DPU 2.0 display controller
 * ("exynos9-decon"), handoff mode.
 *
 * The Samsung boot chain (through uniLoader) leaves the whole display pipe
 * running: DECON0 and its IDMA/DPP fetch a framebuffer, DSIM0 forwards the
 * composed frames in MIPI command mode and the panel keeps its own timing from
 * its init table.  uniLoader only unlocks the DECON shadow registers and hands
 * the framebuffer over through /chosen/framebuffer.
 *
 * This driver therefore does not re-initialise DSIM, the DSI PHY or the panel.
 * It takes over the parts that select what is displayed - the DECON window and
 * blender state, and the IDMA channel that reads the framebuffer - so that the
 * kernel owns modesetting while the boot chain keeps the link alive.
 *
 * State of this file: probe maps the blocks, reports the inherited state and
 * registers a KMS device with the fixed panel mode.  Every atomic commit
 * programs window 0 (IDMA address/format/size and the blender window) and
 * requests a shadow update, so the kernel framebuffer is what the panel shows.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_vblank.h>
#include <drm/drm_vblank_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "exynos9610_dpu.h"
#include "exynos9610_dpu_regs.h"

#define DPU_DEFAULT_WIDTH	1080
#define DPU_DEFAULT_HEIGHT	2400
#define DPU_DEFAULT_REFRESH	60

/**
 * struct exynos9610_dpu - DPU 2.0 handoff driver state
 * @drm: DRM device
 * @decon: DECON + BLENDER register window
 * @idma: IDMA (DPU_DMA) channel that feeds window 0
 * @pipe: CRTC + primary plane of the single display pipe
 * @mode: fixed panel mode taken from the display timings
 * @irq: DECON interrupt line, used for vblank events
 */
struct exynos9610_dpu {
	struct drm_device drm;
	void __iomem *decon;
	void __iomem *idma;
	struct drm_simple_display_pipe pipe;
	struct drm_display_mode mode;
	struct dentry *debugfs;
	int irq;
	int te_irq;
	bool active;
	atomic64_t te_count;
	atomic64_t trigger_count;
	bool sw_trigger;
};

static inline struct exynos9610_dpu *drm_to_dpu(struct drm_device *drm)
{
	return container_of(drm, struct exynos9610_dpu, drm);
}

static inline struct exynos9610_dpu *
pipe_to_dpu(struct drm_simple_display_pipe *pipe)
{
	return container_of(pipe, struct exynos9610_dpu, pipe);
}

static inline struct exynos9610_dpu *
crtc_to_dpu(struct drm_crtc *crtc)
{
	return container_of(crtc, struct exynos9610_dpu, pipe.crtc);
}

static inline u32 dpu_read(struct exynos9610_dpu *dpu, void __iomem *base,
			   u32 reg)
{
	return readl(base + reg);
}

/*
 * The boot chain owns the pipeline until this driver learns to re-initialise
 * it, so report what was inherited instead of guessing.  Every value printed
 * here has been observed live on an A51.
 */
static void exynos9610_dpu_report_live_state(struct exynos9610_dpu *dpu)
{
	struct device *dev = dpu->drm.dev;
	u32 global = dpu_read(dpu, dpu->decon, DECON_GLOBAL_CONTROL);
	u32 trigger = dpu_read(dpu, dpu->decon, DECON_HW_SW_TRIG_CONTROL);
	u32 data_path = dpu_read(dpu, dpu->decon, DECON_DATA_PATH_CONTROL_0);
	u32 out_size = dpu_read(dpu, dpu->decon, DECON_OUTFIFO_SIZE_CONTROL);
	u32 win_start = dpu_read(dpu, dpu->decon, DECON_WIN_START_POSITION(0));
	u32 win_end = dpu_read(dpu, dpu->decon, DECON_WIN_END_POSITION(0));
	u32 win_control = dpu_read(dpu, dpu->decon, DECON_WIN_CONTROL(0));
	u32 dsim_conn = dpu_read(dpu, dpu->decon, DECON_DSIM_CONNECTION_CONTROL);
	u32 in_con = dpu_read(dpu, dpu->idma, IDMA_IN_CON);
	u32 src_size = dpu_read(dpu, dpu->idma, IDMA_SRC_SIZE);
	u32 img_size = dpu_read(dpu, dpu->idma, IDMA_IMG_SIZE);
	u32 base = dpu_read(dpu, dpu->idma, IDMA_IN_BASE_ADDR_Y);
	u32 in_format = (in_con & IDMA_IN_CON_IMG_FORMAT_MASK) >> 11;
	u32 win0_en = data_path & DECON_DATA_PATH_WIN_EN(0) ? 1 : 0;

	dev_info(dev, "DECON: global=0x%08x (enabled=%u, %s mode), trigger=0x%08x (%s trigger)\n",
		 global,
		 !!(global & DECON_GLOBAL_CONTROL_DECON_EN),
		 (global & DECON_GLOBAL_CONTROL_OPERATION_MODE) ==
			DECON_GLOBAL_CONTROL_OPERATION_MODE_CMD ? "command" : "video",
		 trigger,
		 (trigger & DECON_HW_TRIG_EN) ? "hardware" : "software");
	dev_info(dev, "DECON: out=%ux%u win0 %u,%u-%u,%u control=0x%08x data_path=0x%08x dsim_conn=0x%08x\n",
		 out_size & 0x3fff, (out_size >> 16) & 0x3fff,
		 win_start & 0x3fff, (win_start >> 16) & 0x3fff,
		 win_end & 0x3fff, (win_end >> 16) & 0x3fff,
		 win_control, data_path, dsim_conn);
	dev_info(dev, "IDMA: base=0x%08x src=%ux%u img=%ux%u in_con=0x%08x (format %u), win0_enabled=%u\n",
		 base, src_size & 0xffff, (src_size >> 16) & 0xffff,
		 img_size & 0xffff, (img_size >> 16) & 0xffff, in_con,
		 in_format, win0_en);

	if (!(global & DECON_GLOBAL_CONTROL_DECON_EN) ||
	    (global & DECON_GLOBAL_CONTROL_OPERATION_MODE) !=
		DECON_GLOBAL_CONTROL_OPERATION_MODE_CMD)
		dev_warn(dev, "boot chain did not leave a command-mode pipe running; output stays dark\n");
}

/*
 * Window 0 of this pipe is fed by IDMA channel G0 ("G0" in the vendor mapping),
 * the same channel the boot chain used, so the inherited DPP and blender
 * settings stay valid and only the fetch side has to be reprogrammed.
 */
#define DPU_WIN				0
#define DPU_UPDATE_TIMEOUT_US		100000

/* IDMA encodes the pixel format; U32_MAX means "not representable" */
static u32 exynos9610_dpu_idma_format(const struct drm_format_info *info)
{
	switch (info->format) {
	case DRM_FORMAT_ARGB8888:
		return IDMA_IMG_FORMAT_ARGB8888;
	case DRM_FORMAT_ABGR8888:
		return IDMA_IMG_FORMAT_ABGR8888;
	case DRM_FORMAT_XRGB8888:
		return IDMA_IMG_FORMAT_XRGB8888;
	case DRM_FORMAT_XBGR8888:
		return IDMA_IMG_FORMAT_XBGR8888;
	case DRM_FORMAT_RGB565:
		return IDMA_IMG_FORMAT_RGB565;
	default:
		return U32_MAX;
	}
}

/* Latch the shadow registers into the running pipeline, on the next panel TE */
/*
 * A shadow update is only consumed when DECON produces a frame, and in command
 * mode that happens on the panel's tearing effect.  While the pipe is stopped -
 * the disable half of a modeset, DPMS off, or a panel that is asleep and emits
 * no TE - nothing drains the request.  Polling then always times out, and the
 * request correctly stays pending until the next enable drains it.
 */
/* Update request with the usual "wait for the pipe to drain it" behaviour */
static int exynos9610_dpu_update_wait(struct exynos9610_dpu *dpu, bool wait);
static int exynos9610_dpu_update(struct exynos9610_dpu *dpu)
{
	return exynos9610_dpu_update_wait(dpu, true);
}

static int exynos9610_dpu_update_wait(struct exynos9610_dpu *dpu, bool wait)
{
	u32 request = DECON_UPDATE_REQ_GLOBAL | DECON_UPDATE_REQ_WIN(DPU_WIN);
	unsigned int timeout = DPU_UPDATE_TIMEOUT_US;

	writel(request, dpu->decon + DECON_UPDATE_REQ);

	if (!wait || !dpu->active)
		return 0;

	while (dpu_read(dpu, dpu->decon, DECON_UPDATE_REQ)) {
		if (!timeout) {
			dev_warn(dpu->drm.dev, "shadow update timed out\n");
			return -ETIMEDOUT;
		}

		usleep_range(500, 1000);
		timeout -= 1000;
	}

	return 0;
}

static void exynos9610_dpu_commit(struct exynos9610_dpu *dpu)
{
	struct drm_plane_state *state = dpu->pipe.plane.state;
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_dma_object *gem;
	u32 format, size, data_path;

	/*
	 * Leave the window alone when the plane is momentarily empty.  A mode set
	 * arrives before the first framebuffer, and compositors also drop their
	 * plane for a commit or two; clearing the window here blanked a scanout
	 * that was already running, so the panel went black between frames.  Only
	 * an explicit pipe disable stops the window.
	 */
	if (!fb)
		return;

	gem = drm_fb_dma_get_gem_obj(fb, 0);
	if (!gem)
		return;

	format = exynos9610_dpu_idma_format(fb->format);
	if (WARN_ON(format == U32_MAX))
		return;

	size = IDMA_SIZE(fb->width, fb->height);

	writel(gem->dma_addr + fb->offsets[0],
	       dpu->idma + IDMA_IN_BASE_ADDR_Y);
	/* SRC_SIZE carries the row stride in pixels, IMG_SIZE the visible area */
	writel(IDMA_SIZE(fb->pitches[0] / fb->format->cpp[0], fb->height),
	       dpu->idma + IDMA_SRC_SIZE);
	writel(size, dpu->idma + IDMA_IMG_SIZE);

	format |= dpu_read(dpu, dpu->idma, IDMA_IN_CON) &
		  ~IDMA_IN_CON_IMG_FORMAT_MASK;
	writel(format, dpu->idma + IDMA_IN_CON);

	writel(DECON_WIN_XY(0, 0),
	       dpu->decon + DECON_WIN_START_POSITION(DPU_WIN));
	writel(DECON_WIN_XY(fb->width - 1, fb->height - 1),
	       dpu->decon + DECON_WIN_END_POSITION(DPU_WIN));

	data_path = dpu_read(dpu, dpu->decon, DECON_DATA_PATH_CONTROL_0);
	writel(data_path | DECON_DATA_PATH_WIN_EN(DPU_WIN),
	       dpu->decon + DECON_DATA_PATH_CONTROL_0);

	/*
	 * Post the request without waiting for the pipe to drain it.  The
	 * update is consumed at the next frame boundary, and only *then* does
	 * DRM arm the flip event, so blocking here means the event is always
	 * one frame late: the compositor then repaints, misses the following
	 * vblank and settles at exactly half the panel rate (30 fps on a 60 Hz
	 * panel).  Armed early, the same registers are latched in the very
	 * next frame and the flip event goes out on that frame's vblank.
	 */
	exynos9610_dpu_update_wait(dpu, false);
}

/*
 * The inherited pipe keeps running whether or not anybody services its
 * interrupts, so the line is only enabled while DRM asks for vblank events:
 * an unserviced level-sensitive interrupt would otherwise stay asserted.
 */
static void exynos9610_dpu_irq_set(struct exynos9610_dpu *dpu, bool enable)
{
	u32 value;

	/* Drain whatever the boot chain left latched, it is stale by now */
	writel(DECON_FRAME_DONE_INT_PEND | DECON_FRAME_START_INT_PEND |
	       DECON_EXTRA_INT_PEND, dpu->decon + DECON_INTERRUPT_PENDING);

	value = dpu_read(dpu, dpu->decon, DECON_INTERRUPT_ENABLE);
	value &= ~(DECON_FRAME_START_INT_EN | DECON_FRAME_DONE_INT_EN);

	if (enable)
		value |= DECON_FRAME_DONE_INT_EN | DECON_INT_EN;

	writel(value, dpu->decon + DECON_INTERRUPT_ENABLE);
}

/*
 * Command mode frame trigger.
 *
 * The panel paces itself: it raises TE when it is ready for the next frame.
 * The boot chain wires that signal to DECON's hardware trigger, so hardware
 * mode needs no CPU involvement and is what runs by default.  Software mode
 * keeps the same signal as an interrupt source and asks DECON for one frame
 * per TE instead, which is what the vendor driver does when the kernel needs
 * to see every vsync.
 */
static void exynos9610_dpu_set_trigger_mode(struct exynos9610_dpu *dpu,
					    bool sw)
{
	dpu->sw_trigger = sw;

	if (sw) {
		/* Frames now come one per TE interrupt */
		writel(DECON_SW_TRIG_EN,
		       dpu->decon + DECON_HW_SW_TRIG_CONTROL);
	} else {
		writel(DECON_HW_TRIG_EN | DECON_HW_TRIG_EDGE_POLARITY,
		       dpu->decon + DECON_HW_SW_TRIG_CONTROL);
	}

	dev_info(dpu->drm.dev, "frame trigger: %s (HW_SW_TRIG_CONTROL=0x%08x)\n",
		 sw ? "software (TE interrupt)" : "hardware (TE signal)",
		 dpu_read(dpu, dpu->decon, DECON_HW_SW_TRIG_CONTROL));
}

static irqreturn_t exynos9610_dpu_te_irq(int irq, void *data)
{
	struct exynos9610_dpu *dpu = data;

	atomic64_inc(&dpu->te_count);

	if (dpu->sw_trigger) {
		/* One frame per tearing effect */
		writel(DECON_SW_TRIG_EN,
		       dpu->decon + DECON_HW_SW_TRIG_CONTROL);
		atomic64_inc(&dpu->trigger_count);
	}

	return IRQ_HANDLED;
}

static int exynos9610_dpu_trigger_show(struct seq_file *seq, void *unused)
{
	struct exynos9610_dpu *dpu = seq->private;

	seq_printf(seq, "mode: %s\n",
		   dpu->sw_trigger ? "software (TE interrupt)" :
				     "hardware (TE signal)");
	seq_printf(seq, "te interrupts: %llu\n",
		   (u64)atomic64_read(&dpu->te_count));
	seq_printf(seq, "frames triggered by us: %llu\n",
		   (u64)atomic64_read(&dpu->trigger_count));
	seq_printf(seq, "hw_sw_trig_control: 0x%08x\n",
		   dpu_read(dpu, dpu->decon, DECON_HW_SW_TRIG_CONTROL));

	return 0;
}

static ssize_t exynos9610_dpu_trigger_write(struct file *file,
					    const char __user *ubuf,
					    size_t len, loff_t *ppos)
{
	struct seq_file *seq = file->private_data;
	struct exynos9610_dpu *dpu = seq->private;
	char buf[8];

	if (len >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;

	buf[len] = '\0';

	if (!strncmp(buf, "sw", 2))
		exynos9610_dpu_set_trigger_mode(dpu, true);
	else if (!strncmp(buf, "hw", 2))
		exynos9610_dpu_set_trigger_mode(dpu, false);
	else
		return -EINVAL;

	return len;
}

static int exynos9610_dpu_trigger_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos9610_dpu_trigger_show, inode->i_private);
}

static const struct file_operations exynos9610_dpu_trigger_fops = {
	.owner = THIS_MODULE,
	.open = exynos9610_dpu_trigger_open,
	.read = seq_read,
	.write = exynos9610_dpu_trigger_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static irqreturn_t exynos9610_dpu_irq(int irq, void *data)
{
	struct exynos9610_dpu *dpu = data;
	u32 pending, extra;

	pending = dpu_read(dpu, dpu->decon, DECON_INTERRUPT_PENDING);
	if (!pending)
		return IRQ_NONE;

	/* Pending bits are write-1-to-clear; ack before reporting the vblank */
	writel(pending, dpu->decon + DECON_INTERRUPT_PENDING);

	if (pending & DECON_EXTRA_INT_PEND) {
		extra = dpu_read(dpu, dpu->decon, DECON_EXTRA_INTERRUPT_PENDING);
		writel(extra, dpu->decon + DECON_EXTRA_INTERRUPT_PENDING);
		if (extra)
			dev_warn_ratelimited(dpu->drm.dev,
					     "DPU error interrupt: 0x%08x (%s%s)\n",
					     extra,
					     extra & DECON_RESOURCE_CONFLICT_INT_PEND ?
						"resource conflict " : "",
					     extra & DECON_TIME_OUT_INT_PEND ?
						"timeout " : "");
	}

	if (pending & DECON_FRAME_DONE_INT_PEND) {
		/*
		 * In software trigger mode keep the pipe free-running off our own
		 * frame-done instead of the panel's tearing effect.  Experiment: the
		 * panel here has never been seen to emit TE (decon-te stays at 0), so
		 * with the vendor default (trigger on TE) the pipe scans exactly one
		 * frame per commit and the serialiser has nothing to send afterwards.
		 */
		if (dpu->sw_trigger) {
			writel(DECON_SW_TRIG_EN,
			       dpu->decon + DECON_HW_SW_TRIG_CONTROL);
			atomic64_inc(&dpu->trigger_count);
		}

		drm_crtc_handle_vblank(&dpu->pipe.crtc);
	}

	return IRQ_HANDLED;
}

/*
 * The panel is the connector.
 *
 * It lives below the DSI host and exposes a DRM panel, which becomes a bridge
 * on our simple encoder.  That way DPMS reaches the panel: display off/on and
 * sleep in/out are DCS writes issued by the panel driver through the DSI host
 * this driver's sibling implements.
 */
static struct drm_bridge *exynos9610_dpu_find_panel_bridge(struct exynos9610_dpu *dpu)
{
	struct device *dev = dpu->drm.dev;
	struct device_node *dsi_node, *panel_node;
	struct drm_panel *panel;
	struct drm_bridge *bridge;

	dsi_node = of_parse_phandle(dev->of_node, "samsung,dsi-controller", 0);
	if (!dsi_node) {
		dev_err(dev, "no DSI controller phandle\n");
		return ERR_PTR(-ENODEV);
	}

	panel_node = of_get_child_by_name(dsi_node, "panel");
	of_node_put(dsi_node);
	if (!panel_node) {
		dev_err(dev, "no panel below the DSI controller\n");
		return ERR_PTR(-ENODEV);
	}

	panel = of_drm_find_panel(panel_node);
	of_node_put(panel_node);
	if (IS_ERR(panel))
		return ERR_CAST(panel);

	bridge = drm_panel_bridge_add_typed(panel, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(bridge))
		return bridge;

	return bridge;
}

static enum drm_mode_status
exynos9610_dpu_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
			       const struct drm_display_mode *mode)
{
	struct exynos9610_dpu *dpu = pipe_to_dpu(pipe);

	/*
	 * The panel is already running the mode the boot chain programmed; the
	 * driver cannot change it yet, so only that mode is accepted.
	 */
	if (mode->hdisplay != dpu->mode.hdisplay ||
	    mode->vdisplay != dpu->mode.vdisplay)
		return MODE_BAD;

	return MODE_OK;
}

/*
 * DECON start/stop for command mode, mirroring the vendor decon_reg_start() and
 * decon_reg_stop_perframe(): mask the hardware trigger, drop the per-frame
 * enable, wait until the block reports idle - and the other way round on the
 * way back up.  DSIM has to be re-initialised in between.
 */
static struct exynos9610_dpu *exynos9610_dpu_instance;

int exynos9610_dpu_decon_stop(void)
{
	struct exynos9610_dpu *dpu = exynos9610_dpu_instance;
	unsigned int timeout = 1000;
	u32 val;

	if (!dpu)
		return -ENODEV;

	val = dpu_read(dpu, dpu->decon, DECON_HW_SW_TRIG_CONTROL);
	writel((val & ~DECON_HW_TRIG_EN) | DECON_HW_TRIG_MASK_DECON,
	       dpu->decon + DECON_HW_SW_TRIG_CONTROL);

	/*
	 * Per-frame stop first, like the vendor DPMS off: clear the frame-synced
	 * enable and let the next frame retire it.  If the pipe is wedged, fall
	 * back to the instant stop, which clears the request bit as well.
	 */
	val = dpu_read(dpu, dpu->decon, DECON_GLOBAL_CONTROL);
	writel(val & ~DECON_GLOBAL_CONTROL_DECON_EN_F,
	       dpu->decon + DECON_GLOBAL_CONTROL);
	exynos9610_dpu_update_wait(dpu, false);

	while ((dpu_read(dpu, dpu->decon, DECON_GLOBAL_CONTROL) &
		DECON_GLOBAL_CONTROL_RUN_STATUS) && timeout) {
		usleep_range(500, 1000);
		timeout--;
	}

	if (!timeout) {
		dev_dbg(dpu->drm.dev, "per-frame stop timed out, stopping instantly\n");

		val = dpu_read(dpu, dpu->decon, DECON_GLOBAL_CONTROL);
		writel(val & ~(DECON_GLOBAL_CONTROL_DECON_EN |
			       DECON_GLOBAL_CONTROL_DECON_EN_F),
		       dpu->decon + DECON_GLOBAL_CONTROL);
		exynos9610_dpu_update_wait(dpu, false);

		timeout = 1000;
		while ((dpu_read(dpu, dpu->decon, DECON_GLOBAL_CONTROL) &
			DECON_GLOBAL_CONTROL_RUN_STATUS) && timeout) {
			usleep_range(500, 1000);
			timeout--;
		}

		if (!timeout) {
			dev_warn(dpu->drm.dev, "DECON did not go idle\n");
			return -ETIMEDOUT;
		}
	}

	dev_info(dpu->drm.dev, "DECON stopped (pipe idle, trigger masked)\n");

	return 0;
}
EXPORT_SYMBOL_GPL(exynos9610_dpu_decon_stop);

int exynos9610_dpu_decon_start(void)
{
	struct exynos9610_dpu *dpu = exynos9610_dpu_instance;
	unsigned int timeout = 20;
	u32 val;

	if (!dpu)
		return -ENODEV;

	val = dpu_read(dpu, dpu->decon, DECON_GLOBAL_CONTROL);
	writel(val | DECON_GLOBAL_CONTROL_DECON_EN |
	       DECON_GLOBAL_CONTROL_DECON_EN_F,
	       dpu->decon + DECON_GLOBAL_CONTROL);

	exynos9610_dpu_update_wait(dpu, false);

	/* Normal case: the panel is already tearing and the pipe just runs */
	while (!(dpu_read(dpu, dpu->decon, DECON_GLOBAL_CONTROL) &
		 DECON_GLOBAL_CONTROL_RUN_STATUS) && timeout) {
		usleep_range(500, 1000);
		timeout--;
	}

	if (!timeout) {
		/*
		 * After a link power cycle the panel may only start tearing once
		 * it has been sent a frame, so kick exactly one through with a
		 * software trigger and hand the pipe back to the TE trigger.
		 */
		writel(DECON_SW_TRIG_EN,
		       dpu->decon + DECON_HW_SW_TRIG_CONTROL);
		usleep_range(2000, 3000);
		writel(DECON_HW_TRIG_EN | DECON_HW_TRIG_EDGE_POLARITY,
		       dpu->decon + DECON_HW_SW_TRIG_CONTROL);

		timeout = 20;
		while (!(dpu_read(dpu, dpu->decon, DECON_GLOBAL_CONTROL) &
			 DECON_GLOBAL_CONTROL_RUN_STATUS) && timeout) {
			usleep_range(500, 1000);
			timeout--;
		}

		if (!timeout)
			dev_warn(dpu->drm.dev, "DECON did not start running\n");
	} else {
		val = dpu_read(dpu, dpu->decon, DECON_HW_SW_TRIG_CONTROL);
		writel((val | DECON_HW_TRIG_EN) & ~DECON_HW_TRIG_MASK_DECON,
		       dpu->decon + DECON_HW_SW_TRIG_CONTROL);
	}

	dev_info(dpu->drm.dev, "DECON running again (trigger enabled)\n");

	return 0;
}
EXPORT_SYMBOL_GPL(exynos9610_dpu_decon_start);

static void exynos9610_dpu_check_reject(struct exynos9610_dpu *dpu,
					const char *why,
					struct drm_framebuffer *fb)
{
	static int n;

	if (n++ < 10)
		dev_info(dpu->drm.dev,
			 "pipe_check REJECT (%s): fb %ux%u pitch=%u cpp=%u %p4cc, mode %ux%u\n",
			 why, fb->width, fb->height, fb->pitches[0],
			 fb->format->cpp[0], &fb->format->format,
			 dpu->mode.hdisplay, dpu->mode.vdisplay);
}

static int exynos9610_dpu_pipe_check(struct drm_simple_display_pipe *pipe,
				     struct drm_plane_state *plane_state,
				     struct drm_crtc_state *crtc_state)
{
	struct exynos9610_dpu *dpu = pipe_to_dpu(pipe);
	struct drm_framebuffer *fb = plane_state->fb;

	if (!fb)
		return 0;

	/*
	 * The window covers the whole panel, so the visible area must match the
	 * mode exactly.  The pitch may be padded: IDMA takes the stride from the
	 * width field of IMG_SIZE, and clients such as GBM align pitches to 64
	 * bytes (1080x4 becomes 4352), which an equality check used to reject.
	 */
	if (fb->width != dpu->mode.hdisplay || fb->height != dpu->mode.vdisplay) {
		exynos9610_dpu_check_reject(dpu, "size", fb);
		return -EINVAL;
	}

	if (fb->pitches[0] < fb->width * fb->format->cpp[0] ||
	    fb->pitches[0] % fb->format->cpp[0] ||
	    fb->pitches[0] / fb->format->cpp[0] > 0x1fff) {
		exynos9610_dpu_check_reject(dpu, "pitch", fb);
		return -EINVAL;
	}

	if (exynos9610_dpu_idma_format(fb->format) == U32_MAX) {
		exynos9610_dpu_check_reject(dpu, "format", fb);
		return -EINVAL;
	}

	return 0;
}

static void exynos9610_dpu_pipe_enable(struct drm_simple_display_pipe *pipe,
				       struct drm_crtc_state *crtc_state,
				       struct drm_plane_state *plane_state)
{
	struct exynos9610_dpu *dpu = pipe_to_dpu(pipe);

	dev_info(dpu->drm.dev, "modeset %ux%u at %u Hz on the inherited panel pipe\n",
		 crtc_state->mode.hdisplay, crtc_state->mode.vdisplay,
		 drm_mode_vrefresh(&crtc_state->mode));

	/*
	 * Start from an idle pipe: the update requested here is drained by the
	 * first tearing effect, so there is nothing to wait for yet.  From the
	 * next commit on the pipe is live and updates are confirmed.
	 */
	dpu->active = false;
	exynos9610_dpu_update(dpu);
	dpu->active = true;

	drm_crtc_vblank_on(&pipe->crtc);
}

static void exynos9610_dpu_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct exynos9610_dpu *dpu = pipe_to_dpu(pipe);

	drm_crtc_vblank_off(&pipe->crtc);

	/* Nothing will drain this update: the panel is about to go to sleep */
	writel(dpu_read(dpu, dpu->decon, DECON_DATA_PATH_CONTROL_0) &
	       ~DECON_DATA_PATH_WIN_EN(DPU_WIN),
	       dpu->decon + DECON_DATA_PATH_CONTROL_0);
	dpu->active = false;
	exynos9610_dpu_update(dpu);
}

static int exynos9610_dpu_pipe_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct exynos9610_dpu *dpu = pipe_to_dpu(pipe);

	/*
	 * DECON free-runs on the panel's tearing effect as soon as the link is up,
	 * so frame-done - and therefore a real vblank - exists even before any
	 * framebuffer is attached; reporting -EINVAL whenever the plane was empty
	 * made every flip complete with a stale timestamp, which is what drove
	 * compositors into "time until next presentation is abnormal" loops.
	 *
	 * Only a DECON that is enabled but not yet running has no vblank, and then
	 * a commit must complete immediately instead of waiting for an interrupt
	 * that would never come.
	 */
	if (!(dpu_read(dpu, dpu->decon, DECON_GLOBAL_CONTROL) &
	      DECON_GLOBAL_CONTROL_RUN_STATUS))
		return -EINVAL;

	if (!dpu->irq)
		return -ENODEV;

	exynos9610_dpu_irq_set(dpu, true);

	return 0;
}

static void exynos9610_dpu_pipe_disable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct exynos9610_dpu *dpu = pipe_to_dpu(pipe);

	if (dpu->irq)
		exynos9610_dpu_irq_set(dpu, false);
}

static void exynos9610_dpu_pipe_update(struct drm_simple_display_pipe *pipe,
				       struct drm_plane_state *old_state)
{
	struct exynos9610_dpu *dpu = pipe_to_dpu(pipe);

	exynos9610_dpu_commit(dpu);
}

static const struct drm_simple_display_pipe_funcs exynos9610_dpu_pipe_funcs = {
	.check = exynos9610_dpu_pipe_check,
	.mode_valid = exynos9610_dpu_pipe_mode_valid,
	.enable = exynos9610_dpu_pipe_enable,
	.disable = exynos9610_dpu_pipe_disable,
	.enable_vblank = exynos9610_dpu_pipe_enable_vblank,
	.disable_vblank = exynos9610_dpu_pipe_disable_vblank,
	.update = exynos9610_dpu_pipe_update,
};

/*
 * These mirror the helpers drm_simple_display_pipe_init() installs, which only
 * delegate to the pipe callbacks, plus a flush hook: an atomic commit completes
 * when its vblank event has been sent, and once vblank support exists the core
 * no longer fakes that event for us.  drm_crtc_vblank_atomic_flush() arms it,
 * the frame-done interrupt then delivers it through drm_crtc_handle_vblank().
 *
 * Replacing crtc->helper_private wholesale would drop the simple-pipe enable and
 * disable hooks, and the pipe would never be programmed or unprogrammed.
 */
static enum drm_mode_status
exynos9610_dpu_crtc_mode_valid(struct drm_crtc *crtc,
			       const struct drm_display_mode *mode)
{
	return exynos9610_dpu_pipe_mode_valid(&crtc_to_dpu(crtc)->pipe, mode);
}

static int exynos9610_dpu_crtc_check(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	/*
	 * Deliberately no drm_atomic_helper_check_crtc_primary_plane() here: DECON
	 * runs fine with no window programmed, and compositors such as weston set
	 * the mode first and attach the framebuffer in a later commit.  The commit
	 * in between has nothing to scan out, which enable_vblank() reports as
	 * -EINVAL so that its flip completes immediately instead of waiting for a
	 * frame-done interrupt that would never arrive.
	 */
	return drm_atomic_add_affected_planes(state, crtc);
}

static void exynos9610_dpu_crtc_atomic_enable(struct drm_crtc *crtc,
					      struct drm_atomic_state *state)
{
	struct exynos9610_dpu *dpu = crtc_to_dpu(crtc);
	struct drm_plane *plane = &dpu->pipe.plane;

	exynos9610_dpu_pipe_enable(&dpu->pipe, crtc->state, plane->state);
}

static void exynos9610_dpu_crtc_atomic_disable(struct drm_crtc *crtc,
					       struct drm_atomic_state *state)
{
	exynos9610_dpu_pipe_disable(&crtc_to_dpu(crtc)->pipe);
}

static const struct drm_crtc_helper_funcs exynos9610_dpu_crtc_helper_funcs = {
	.mode_valid = exynos9610_dpu_crtc_mode_valid,
	.atomic_check = exynos9610_dpu_crtc_check,
	.atomic_enable = exynos9610_dpu_crtc_atomic_enable,
	.atomic_disable = exynos9610_dpu_crtc_atomic_disable,
	.atomic_flush = drm_crtc_vblank_atomic_flush,
};

/*
 * DECON has no compressed (AFBC) scanout implemented here, so only linear
 * layouts are valid.  The property matters: without IN_FORMATS wlroots builds an
 * empty scanout format set and fails with EGL_BAD_NATIVE_WINDOW.
 */
static const u64 exynos9610_dpu_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static const u32 exynos9610_dpu_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGB565,
};

static const struct drm_mode_config_funcs exynos9610_dpu_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(exynos9610_dpu_fops);

static const struct drm_driver exynos9610_dpu_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &exynos9610_dpu_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name = "exynos9610-dpu",
	.desc = "Samsung Exynos 9610/9611 DPU 2.0",
	.major = 1,
	.minor = 0,
};

/* A51 panel outline, used when the DT carries no display timings */
static const struct drm_display_mode exynos9610_dpu_default_mode = {
	DRM_SIMPLE_MODE(DPU_DEFAULT_WIDTH, DPU_DEFAULT_HEIGHT, 68, 152),
};

static int exynos9610_dpu_mode_init(struct exynos9610_dpu *dpu)
{
	struct device *dev = dpu->drm.dev;
	int ret;

	ret = of_get_drm_display_mode(dev->of_node, &dpu->mode, NULL, 0);
	if (ret) {
		dev_warn(dev, "no display timings in DT (%d), assuming %ux%u\n",
			 ret, DPU_DEFAULT_WIDTH, DPU_DEFAULT_HEIGHT);
		dpu->mode = exynos9610_dpu_default_mode;
	}

	dpu->mode.type |= DRM_MODE_TYPE_DRIVER;
	drm_mode_set_name(&dpu->mode);

	return 0;
}

static int exynos9610_dpu_probe(struct platform_device *pdev)
{
	struct device_node *dsi_node;
	struct drm_bridge *bridge, *link;
	struct exynos9610_dpu *dpu;
	struct drm_device *drm;
	int ret;

	dpu = devm_drm_dev_alloc(&pdev->dev, &exynos9610_dpu_driver,
				 struct exynos9610_dpu, drm);
	if (IS_ERR(dpu))
		return PTR_ERR(dpu);

	drm = &dpu->drm;
	platform_set_drvdata(pdev, dpu);
	exynos9610_dpu_instance = dpu;

	dpu->decon = devm_platform_ioremap_resource_byname(pdev, "decon");
	if (IS_ERR(dpu->decon))
		return dev_err_probe(drm->dev, PTR_ERR(dpu->decon),
				     "failed to map DECON\n");

	dpu->idma = devm_platform_ioremap_resource_byname(pdev, "idma");
	if (IS_ERR(dpu->idma))
		return dev_err_probe(drm->dev, PTR_ERR(dpu->idma),
				     "failed to map IDMA\n");

	ret = dma_set_mask_and_coherent(drm->dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(drm->dev, ret, "no 32-bit DMA mask\n");

	/*
	 * Bind the shared contiguous pool from the "memory-region" phandle.  The
	 * IDMA channel needs a physically contiguous, 32-bit addressable buffer
	 * for every scanout frame.
	 */
	ret = of_reserved_mem_device_init(drm->dev);
	if (ret)
		return dev_err_probe(drm->dev, ret, "failed to get the scanout memory region\n");

	dpu->irq = platform_get_irq_optional(pdev, 0);
	if (dpu->irq == -ENXIO || dpu->irq == -ENODEV)
		dpu->irq = 0;
	else if (dpu->irq < 0)
		return dpu->irq;

	ret = exynos9610_dpu_mode_init(dpu);
	if (ret)
		return ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = 16;
	drm->mode_config.min_height = 16;
	/*
	 * Scanning out anything but the panel mode is not supported yet, so the
	 * mode limits are the panel limits.  This also matters for the fbdev
	 * client: it sizes its buffer from max_width/max_height when no mode is
	 * resolved yet, and a too-large buffer fails to allocate.
	 */
	drm->mode_config.max_width = dpu->mode.hdisplay;
	drm->mode_config.max_height = dpu->mode.vdisplay;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.funcs = &exynos9610_dpu_mode_config_funcs;

	bridge = exynos9610_dpu_find_panel_bridge(dpu);
	if (IS_ERR(bridge))
		return dev_err_probe(drm->dev, PTR_ERR(bridge),
				     "failed to find the panel\n");

	ret = drm_simple_display_pipe_init(drm, &dpu->pipe,
					   &exynos9610_dpu_pipe_funcs,
					   exynos9610_dpu_formats,
					   ARRAY_SIZE(exynos9610_dpu_formats),
					   exynos9610_dpu_modifiers, NULL);
	if (ret)
		return ret;

	/*
	 * The DSIM link bridge powers the D-PHY up before the panel init sequence
	 * runs and down again once the pipe has been stopped.  Bridges are chained
	 * in attachment order, so the link goes first and the panel bridge after
	 * it: pre_enable then goes link -> panel, post_disable panel -> link.
	 */
	link = NULL;
	dsi_node = of_parse_phandle(drm->dev->of_node, "samsung,dsi-controller", 0);
	if (dsi_node) {
		struct mipi_dsi_host *host = of_find_mipi_dsi_host_by_node(dsi_node);

		of_node_put(dsi_node);

		if (host)
			link = exynos9610_dsim_link_bridge(host);

		if (!link)
			dev_warn(drm->dev, "no DSIM link bridge, link power stays inherited\n");
	}

	if (link) {
		ret = drm_bridge_attach(&dpu->pipe.encoder, link, NULL, 0);
		if (ret)
			return dev_err_probe(drm->dev, ret,
					     "failed to attach the DSIM link\n");
	}

	ret = drm_bridge_attach(&dpu->pipe.encoder, bridge, link, 0);
	if (ret)
		return dev_err_probe(drm->dev, ret, "failed to attach the panel\n");

	dpu->pipe.crtc.helper_private = &exynos9610_dpu_crtc_helper_funcs;

	/*
	 * The frame-done interrupt is the vblank source.  Without this call the
	 * core refuses every vblank ioctl (dev->num_crtcs stays 0), even though
	 * drm_crtc_handle_vblank() below would already work.
	 */
	ret = drm_vblank_init(drm, drm->mode_config.num_crtc);
	if (ret)
		return dev_err_probe(drm->dev, ret, "failed to initialise vblank\n");

	drm_mode_config_reset(drm);

	exynos9610_dpu_report_live_state(dpu);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	if (dpu->irq) {
		ret = devm_request_irq(drm->dev, dpu->irq, exynos9610_dpu_irq, 0,
				       dev_name(drm->dev), dpu);
		if (ret) {
			dev_warn(drm->dev, "no vblank events without IRQ: %d\n",
				 ret);
			dpu->irq = 0;
		}
	}

	/*
	 * The panel's tearing effect reports when it is ready for a frame.  It
	 * is optional: hardware trigger mode already paces the pipe without it.
	 */
	dpu->te_irq = platform_get_irq_byname_optional(pdev, "te");
	if (dpu->te_irq == -ENXIO || dpu->te_irq == -ENODEV)
		dpu->te_irq = 0;
	else if (dpu->te_irq < 0)
		return dpu->te_irq;

	if (dpu->te_irq) {
		ret = devm_request_irq(drm->dev, dpu->te_irq,
				       exynos9610_dpu_te_irq, 0, "decon-te", dpu);
		if (ret) {
			dev_warn(drm->dev, "no TE events: %d\n", ret);
			dpu->te_irq = 0;
		} else {
			dev_info(drm->dev, "panel TE on irq %d, %s trigger\n",
				 dpu->te_irq, dpu->sw_trigger ? "software" :
							       "hardware");
		}
	}

	dpu->debugfs = debugfs_create_dir(dev_name(drm->dev), NULL);
	debugfs_create_file("trigger", 0600, dpu->debugfs, dpu,
			    &exynos9610_dpu_trigger_fops);

	drm_client_setup_with_fourcc(drm, DRM_FORMAT_XRGB8888);

	return 0;
}

static void exynos9610_dpu_remove(struct platform_device *pdev)
{
	struct exynos9610_dpu *dpu = platform_get_drvdata(pdev);

	debugfs_remove_recursive(dpu->debugfs);

	drm_dev_unplug(&dpu->drm);
}

static void exynos9610_dpu_shutdown(struct platform_device *pdev)
{
	struct exynos9610_dpu *dpu = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&dpu->drm);
}

static const struct of_device_id exynos9610_dpu_of_match[] = {
	{ .compatible = "samsung,exynos9610-decon" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos9610_dpu_of_match);

/*
 * System sleep.  Suspending the DRM state turns the pipe off through the normal
 * atomic path, which is what powers the DSI link down (the DSIM bridge's
 * post_disable) and puts the panel to sleep; resume rebuilds the state and
 * powers everything back up.
 */
static int exynos9610_dpu_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	int ret;

	ret = drm_mode_config_helper_suspend(drm);
	dev_info(dev, "display suspended (%d)\n", ret);

	return ret;
}

static int exynos9610_dpu_resume(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	int ret;

	ret = drm_mode_config_helper_resume(drm);
	dev_info(dev, "display resumed (%d)\n", ret);

	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(exynos9610_dpu_pm_ops, exynos9610_dpu_suspend,
				exynos9610_dpu_resume);

static struct platform_driver exynos9610_dpu_platform_driver = {
	.probe = exynos9610_dpu_probe,
	.remove = exynos9610_dpu_remove,
	.shutdown = exynos9610_dpu_shutdown,
	.driver = {
		.name = "exynos9610-dpu",
		.of_match_table = exynos9610_dpu_of_match,
		.pm = pm_ptr(&exynos9610_dpu_pm_ops),
	},
};
module_platform_driver(exynos9610_dpu_platform_driver);

MODULE_AUTHOR("Exynos 9611 mainline port");
MODULE_DESCRIPTION("Samsung Exynos 9610/9611 DPU 2.0 DRM/KMS driver");
MODULE_LICENSE("GPL");
