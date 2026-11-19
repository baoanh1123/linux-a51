/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Shared entry points between the DPU and DSIM halves of the Exynos 9610/9611
 * display port.
 *
 * Command mode couples the two blocks: DECON stops generating frames once its
 * hardware trigger is masked, and DSIM must be re-initialised while DECON is
 * idle - otherwise DECON waits for a frame-done that the reset DSIM will never
 * send, and the pipeline deadlocks.  Until the DRM DPMS path owns this, the
 * DSIM bring-up helpers sequence it explicitly.
 */
#ifndef __EXYNOS9610_DPU_H__
#define __EXYNOS9610_DPU_H__

struct drm_bridge;
struct mipi_dsi_host;

#if IS_ENABLED(CONFIG_DRM_EXYNOS9610)
int exynos9610_dpu_decon_stop(void);
int exynos9610_dpu_decon_start(void);
#else
static inline int exynos9610_dpu_decon_stop(void) { return -ENODEV; }
static inline int exynos9610_dpu_decon_start(void) { return -ENODEV; }
#endif

/*
 * The DSIM link bridge powers the D-PHY up before the panel is prepared and
 * down after the pipe has been stopped, so DPMS really cuts the link.
 */
#if IS_ENABLED(CONFIG_DRM_EXYNOS9610_DSIM)
struct drm_bridge *exynos9610_dsim_link_bridge(struct mipi_dsi_host *host);
#else
static inline struct drm_bridge *
exynos9610_dsim_link_bridge(struct mipi_dsi_host *host)
{
	return NULL;
}
#endif

#endif /* __EXYNOS9610_DPU_H__ */
