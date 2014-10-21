/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:Mark Yao <mark.yao@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drm.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_rect.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/component.h>

#include <linux/reset.h>
#include <linux/iommu.h>
#include <linux/delay.h>

#include <video/of_display_timing.h>
#include <video/of_videomode.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_fbdev.h"
#include "rockchip_drm_gem.h"
#include "rockchip_drm_fb.h"
#include "rockchip_drm_vop.h"

#define VOP_DEFAULT_FRAMERATE	60
#define VOP_MAX_WIN_SUPPORT	5
#define VOP_DEFAULT_PRIMARY	0
#define VOP_DEFAULT_CURSOR	1
#define VOP_REG(off, _mask, s) \
		{.offset = off, \
		 .mask = _mask, \
		 .shift = s,}

#define __REG_SET_RELAXED(x, off, mask, shift, v) \
		vop_mask_write_relaxed(x, off, (mask) << shift, (v) << shift)
#define __REG_SET_NORMAL(x, off, mask, shift, v) \
		vop_mask_write(x, off, (mask) << shift, (v) << shift)

#define REG_SET(x, base, reg, v, mode) \
		__REG_SET_##mode(x, base + reg.offset, reg.mask, reg.shift, v)

#define VOP_WIN_SET(x, win, name, v) \
		REG_SET(x, win->base, win->phy->name, v, RELAXED)
#define VOP_CTRL_SET(x, name, v) \
		REG_SET(x, 0, (x)->data->ctrl->name, v, NORMAL)

#define VOP_WIN_GET_YRGBADDR(ctx, win) \
		vop_readl(ctx, win->base + win->phy->yrgb_mst.offset)

#define to_vop_ctx(x) container_of(x, struct vop_context, crtc)
#define to_rockchip_plane(x) container_of(x, struct rockchip_plane, base)

struct rockchip_plane {
	int id;
	struct drm_plane base;
	const struct vop_win *win;

	uint32_t pending_yrgb_mst;
	struct drm_framebuffer *front_fb;
	struct drm_framebuffer *pending_fb;
	bool enabled;
};

struct vop_context {
	struct device *dev;
	struct drm_device *drm_dev;
	struct drm_crtc crtc;
	struct drm_pending_vblank_event *event;
	struct vop_driver *drv;
	unsigned int dpms;

	int connector_type;
	int connector_out_mode;
	wait_queue_head_t wait_vsync_queue;
	atomic_t wait_vsync_event;

	struct workqueue_struct *vsync_wq;
	struct work_struct vsync_work;

	/* mutex vsync_ work */
	struct mutex vsync_mutex;
	bool vsync_work_pending;

	struct vop_driver_data *data;

	uint32_t *regsbak;
	void __iomem *regs;

	/* physical map length of vop register */
	uint32_t len;

	/* one time only one process allowed to config the register */
	spinlock_t reg_lock;
	/* lock vop irq reg */
	spinlock_t irq_lock;

	unsigned int irq;

	/* vop AHP clk */
	struct clk *hclk;
	/* vop dclk */
	struct clk *dclk;
	/* vop share memory frequency */
	struct clk *aclk;

	/* vop dclk reset */
	struct reset_control *dclk_rst;

	int pipe;
	bool clk_on;
	bool is_iommu_attach;
};

enum vop_data_format {
	VOP_FMT_ARGB8888 = 0,
	VOP_FMT_RGB888,
	VOP_FMT_RGB565,
	VOP_FMT_YUV420SP = 4,
	VOP_FMT_YUV422SP,
	VOP_FMT_YUV444SP,
};

struct vop_reg_data {
	uint32_t offset;
	uint32_t value;
};

struct vop_reg {
	uint32_t offset;
	uint32_t shift;
	uint32_t mask;
};

struct vop_ctrl {
	struct vop_reg standby;
	struct vop_reg data_blank;
	struct vop_reg gate_en;
	struct vop_reg mmu_en;
	struct vop_reg rgb_en;
	struct vop_reg edp_en;
	struct vop_reg hdmi_en;
	struct vop_reg mipi_en;
	struct vop_reg out_mode;
	struct vop_reg dither_down;
	struct vop_reg dither_up;
	struct vop_reg pin_pol;

	struct vop_reg htotal_pw;
	struct vop_reg hact_st_end;
	struct vop_reg vtotal_pw;
	struct vop_reg vact_st_end;
	struct vop_reg hpost_st_end;
	struct vop_reg vpost_st_end;
};

struct vop_win_phy {
	const uint32_t *data_formats;
	uint32_t nformats;

	struct vop_reg enable;
	struct vop_reg format;
	struct vop_reg act_info;
	struct vop_reg dsp_info;
	struct vop_reg dsp_st;
	struct vop_reg yrgb_mst;
	struct vop_reg uv_mst;
	struct vop_reg yrgb_vir;
	struct vop_reg uv_vir;

	struct vop_reg dst_alpha_ctl;
	struct vop_reg src_alpha_ctl;
};

struct vop_win {
	uint32_t base;
	const struct vop_win_phy *phy;
};

struct vop_driver_data {
	const void *init_table;
	int table_size;
	const struct vop_ctrl *ctrl;
	const struct vop_win *win[VOP_MAX_WIN_SUPPORT];
};

static const uint32_t formats_01[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
};

static const uint32_t formats_234[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
};

static const struct vop_win_phy win01_data = {
	.data_formats = formats_01,
	.nformats = ARRAY_SIZE(formats_01),
	.enable = VOP_REG(WIN0_CTRL0, 0x1, 0),
	.format = VOP_REG(WIN0_CTRL0, 0x7, 1),
	.act_info = VOP_REG(WIN0_ACT_INFO, 0x1fff1fff, 0),
	.dsp_info = VOP_REG(WIN0_DSP_INFO, 0x0fff0fff, 0),
	.dsp_st = VOP_REG(WIN0_DSP_ST, 0x1fff1fff, 0),
	.yrgb_mst = VOP_REG(WIN0_YRGB_MST, 0xffffffff, 0),
	.uv_mst = VOP_REG(WIN0_CBR_MST, 0xffffffff, 0),
	.yrgb_vir = VOP_REG(WIN0_VIR, 0x3fff, 0),
	.uv_vir = VOP_REG(WIN0_VIR, 0x3fff, 16),
	.src_alpha_ctl = VOP_REG(WIN0_SRC_ALPHA_CTRL, 0xff, 0),
	.dst_alpha_ctl = VOP_REG(WIN0_DST_ALPHA_CTRL, 0xff, 0),
};

static const struct vop_win_phy win23_data = {
	.data_formats = formats_234,
	.nformats = ARRAY_SIZE(formats_234),
	.enable = VOP_REG(WIN2_CTRL0, 0x1, 0),
	.format = VOP_REG(WIN2_CTRL0, 0x7, 1),
	.dsp_info = VOP_REG(WIN2_DSP_INFO0, 0x0fff0fff, 0),
	.dsp_st = VOP_REG(WIN2_DSP_ST0, 0x1fff1fff, 0),
	.yrgb_mst = VOP_REG(WIN2_MST0, 0xffffffff, 0),
	.yrgb_vir = VOP_REG(WIN2_VIR0_1, 0x1fff, 0),
	.src_alpha_ctl = VOP_REG(WIN2_SRC_ALPHA_CTRL, 0xff, 0),
	.dst_alpha_ctl = VOP_REG(WIN2_DST_ALPHA_CTRL, 0xff, 0),
};

static const struct vop_win_phy cursor_data = {
	.data_formats = formats_234,
	.nformats = ARRAY_SIZE(formats_234),
	.enable = VOP_REG(HWC_CTRL0, 0x1, 0),
	.format = VOP_REG(HWC_CTRL0, 0x7, 1),
	.dsp_st = VOP_REG(HWC_DSP_ST, 0x1fff1fff, 0),
	.yrgb_mst = VOP_REG(HWC_MST, 0xffffffff, 0),
};

static const struct vop_win win0 = {
	.base = 0,
	.phy = &win01_data,
};

static const struct vop_win win1 = {
	.base = 0x40,
	.phy = &win01_data,
};

static const struct vop_win win2 = {
	.base = 0,
	.phy = &win23_data,
};

static const struct vop_win win3 = {
	.base = 0x50,
	.phy = &win23_data,
};

static const struct vop_win win_cursor = {
	.base = 0,
	.phy = &cursor_data,
};

static const struct vop_ctrl ctrl_data = {
	.standby = VOP_REG(SYS_CTRL, 0x1, 22),
	.gate_en = VOP_REG(SYS_CTRL, 0x1, 23),
	.mmu_en = VOP_REG(SYS_CTRL, 0x1, 20),
	.rgb_en = VOP_REG(SYS_CTRL, 0x1, 12),
	.hdmi_en = VOP_REG(SYS_CTRL, 0x1, 13),
	.edp_en = VOP_REG(SYS_CTRL, 0x1, 14),
	.mipi_en = VOP_REG(SYS_CTRL, 0x1, 15),
	.dither_down = VOP_REG(DSP_CTRL1, 0xf, 1),
	.dither_up = VOP_REG(DSP_CTRL1, 0x1, 6),
	.data_blank = VOP_REG(DSP_CTRL0, 0x1, 19),
	.out_mode = VOP_REG(DSP_CTRL0, 0xf, 0),
	.pin_pol = VOP_REG(DSP_CTRL0, 0xf, 4),
	.htotal_pw = VOP_REG(DSP_HTOTAL_HS_END, 0x1fff1fff, 0),
	.hact_st_end = VOP_REG(DSP_HACT_ST_END, 0x1fff1fff, 0),
	.vtotal_pw = VOP_REG(DSP_VTOTAL_VS_END, 0x1fff1fff, 0),
	.vact_st_end = VOP_REG(DSP_VACT_ST_END, 0x1fff1fff, 0),
	.hpost_st_end = VOP_REG(POST_DSP_HACT_INFO, 0x1fff1fff, 0),
	.vpost_st_end = VOP_REG(POST_DSP_VACT_INFO, 0x1fff1fff, 0),
};

static const struct vop_reg_data vop_init_reg_table[] = {
	{SYS_CTRL, 0x00c00000},
	{DSP_CTRL0, 0x00000000},
	{WIN0_CTRL0, 0x00000080},
	{WIN1_CTRL0, 0x00000080},
};

static const struct vop_driver_data rockchip_rk3288_vop = {
	.init_table = vop_init_reg_table,
	.table_size = ARRAY_SIZE(vop_init_reg_table),
	.ctrl = &ctrl_data,
	.win[0] = &win0,
	.win[1] = &win1,
	.win[2] = &win2,
	.win[3] = &win3,
	.win[4] = &win_cursor,
};

static const struct of_device_id vop_driver_dt_match[] = {
	{ .compatible = "rockchip,rk3288-vop",
	  .data = (void *)&rockchip_rk3288_vop },
	{},
};

static inline void vop_writel(struct vop_context *ctx,
			      uint32_t offset, uint32_t v)
{
	writel(v, ctx->regs + offset);
	ctx->regsbak[offset >> 2] = v;
}

static inline uint32_t vop_readl(struct vop_context *ctx, uint32_t offset)
{
	return readl(ctx->regs + offset);
}

static inline void vop_cfg_done(struct vop_context *ctx)
{
	writel(0x01, ctx->regs + REG_CFG_DONE);
}

static inline void vop_mask_write(struct vop_context *ctx,
				  uint32_t offset, uint32_t mask, uint32_t v)
{
	if (mask) {
		uint32_t cached_val = ctx->regsbak[offset >> 2];

		cached_val = (cached_val & ~mask) | v;
		writel(cached_val, ctx->regs + offset);
		ctx->regsbak[offset >> 2] = cached_val;
	}
}

static inline void vop_mask_write_relaxed(struct vop_context *ctx,
					  uint32_t offset, uint32_t mask,
					  uint32_t v)
{
	if (mask) {
		uint32_t cached_val = ctx->regsbak[offset >> 2];

		cached_val = (cached_val & ~mask) | v;
		writel_relaxed(cached_val, ctx->regs + offset);
		ctx->regsbak[offset >> 2] = cached_val;
	}
}

static inline struct vop_driver_data *vop_get_driver_data(struct device *dev)
{
	const struct of_device_id *of_id =
			of_match_device(vop_driver_dt_match, dev);

	return (struct vop_driver_data *)of_id->data;
}

static enum vop_data_format vop_convert_format(uint32_t format)
{
	switch (format) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		return VOP_FMT_ARGB8888;
	case DRM_FORMAT_RGB888:
		return VOP_FMT_RGB888;
	case DRM_FORMAT_RGB565:
		return VOP_FMT_RGB565;
	case DRM_FORMAT_NV12:
		return VOP_FMT_YUV420SP;
	case DRM_FORMAT_NV16:
		return VOP_FMT_YUV422SP;
	case DRM_FORMAT_NV24:
		return VOP_FMT_YUV444SP;
	default:
		DRM_ERROR("unsupport format[%08x]\n", format);
		return -EINVAL;
	}
}

static bool is_alpha_support(uint32_t format)
{
	switch (format) {
	case DRM_FORMAT_ARGB8888:
		return true;
	default:
		return false;
	}
}

/* TODO(djkurtz): move generic 'setup slave rk_iommu' code somewhere common */
static int vop_iommu_init(struct vop_context *ctx)
{
	struct device *dev = ctx->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *pd;
	int count;
	int ret;
	struct of_phandle_args args;

	/* Each VOP must have exactly one iommu node, with no args */
	count = of_count_phandle_with_args(np, "iommus", "#iommu-cells");
	if (count != 1) {
		dev_err(dev, "of_count_phandle_with_args(%s) => %d\n",
			np->full_name, count);
		return -EINVAL;
	}

	ret = of_parse_phandle_with_args(np, "iommus", "#iommu-cells", 0,
					 &args);
	if (ret) {
		dev_err(dev, "of_parse_phandle_with_args(%s) => %d\n",
			np->full_name, ret);
		return ret;
	}
	if (args.args_count != 0) {
		dev_err(dev, "incorrect number of iommu params found for %s (found %d, expected 0)\n",
			args.np->full_name, args.args_count);
		return -EINVAL;
	}

	pd = of_find_device_by_node(args.np);
	of_node_put(args.np);
	if (!pd) {
		dev_err(dev, "iommu %s not found\n", args.np->full_name);
		return -EPROBE_DEFER;
	}

	/* TODO(djkurtz): handle multiple slave iommus for a single master */
	dev->archdata.iommu = &pd->dev;

	return 0;
}

static void rockchip_power_on(struct drm_crtc *crtc)
{
	struct vop_context *ctx = to_vop_ctx(crtc);
	int ret;

	ret = clk_enable(ctx->hclk);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to enable hclk - %d\n", ret);
		return;
	}

	ret = clk_enable(ctx->dclk);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to enable dclk - %d\n", ret);
		goto err_disable_hclk;
	}

	ret = clk_enable(ctx->aclk);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to enable aclk - %d\n", ret);
		goto err_disable_dclk;
	}
	if (!ctx->is_iommu_attach) {
		/*
		 * when we attach iommu device, we sould sure the vop scan at
		 * correct address.
		 */
		ret = rockchip_drm_dma_attach_device(ctx->drm_dev, ctx->dev);
		if (ret) {
			dev_err(ctx->dev, "failed to attach dma mapping, %d\n",
				ret);
			goto err_disable_aclk;
		}
		ctx->is_iommu_attach = true;
	}

	spin_lock(&ctx->reg_lock);

	VOP_CTRL_SET(ctx, standby, 0);

	spin_unlock(&ctx->reg_lock);

	return;

err_disable_aclk:
	clk_disable(ctx->aclk);
err_disable_dclk:
	clk_disable(ctx->dclk);
err_disable_hclk:
	clk_disable(ctx->hclk);
}

static void rockchip_power_off(struct drm_crtc *crtc)
{
	struct vop_context *ctx = to_vop_ctx(crtc);

	drm_vblank_off(crtc->dev, ctx->pipe);

	spin_lock(&ctx->reg_lock);

	VOP_CTRL_SET(ctx, standby, 1);

	spin_unlock(&ctx->reg_lock);
	/*
	 * disable dclk to stop frame scan, so we can safely detach iommu,
	 */
	clk_disable(ctx->dclk);

	if (ctx->is_iommu_attach) {
		rockchip_drm_dma_detach_device(ctx->drm_dev, ctx->dev);
		ctx->is_iommu_attach = false;
	}

	clk_disable(ctx->aclk);
	clk_disable(ctx->hclk);
}

static int rockchip_update_plane(struct drm_plane *plane, struct drm_crtc *crtc,
				 struct drm_framebuffer *fb, int crtc_x,
				 int crtc_y, unsigned int crtc_w,
				 unsigned int crtc_h, uint32_t src_x,
				 uint32_t src_y, uint32_t src_w, uint32_t src_h)
{
	struct rockchip_plane *rockchip_plane = to_rockchip_plane(plane);
	const struct vop_win *win = rockchip_plane->win;
	struct vop_context *ctx = to_vop_ctx(crtc);
	struct drm_gem_object *obj;
	struct rockchip_gem_object *rk_obj;
	unsigned long offset;
	unsigned int actual_w;
	unsigned int actual_h;
	unsigned int dsp_stx;
	unsigned int dsp_sty;
	unsigned int y_vir_stride;
	dma_addr_t yrgb_mst;
	enum vop_data_format format;
	uint32_t val;
	bool is_alpha;
	bool visible;
	int ret;
	struct drm_rect dest = {
		.x1 = crtc_x,
		.y1 = crtc_y,
		.x2 = crtc_x + crtc_w,
		.y2 = crtc_y + crtc_h,
	};
	struct drm_rect src = {
		/* 16.16 fixed point */
		.x1 = src_x,
		.y1 = src_y,
		.x2 = src_x + src_w,
		.y2 = src_y + src_h,
	};
	const struct drm_rect clip = {
		.x2 = crtc->mode.hdisplay,
		.y2 = crtc->mode.vdisplay,
	};
	bool can_position = plane->type != DRM_PLANE_TYPE_PRIMARY;

	ret = drm_plane_helper_check_update(plane, crtc, fb,
					    &src, &dest, &clip,
					    DRM_PLANE_HELPER_NO_SCALING,
					    DRM_PLANE_HELPER_NO_SCALING,
					    can_position, false, &visible);
	if (ret)
		return ret;

	if (!visible)
		return 0;

	is_alpha = is_alpha_support(fb->pixel_format);
	format = vop_convert_format(fb->pixel_format);
	if (format < 0)
		return format;

	obj = rockchip_fb_get_gem_obj(fb, 0);
	if (!obj) {
		DRM_ERROR("fail to get rockchip gem object from framebuffer\n");
		return -EINVAL;
	}

	rk_obj = to_rockchip_obj(obj);

	yrgb_mst = rk_obj->dma_addr;
	actual_w = (src.x2 - src.x1) >> 16;
	actual_h = (src.y2 - src.y1) >> 16;
	crtc_x = max(0, crtc_x);
	crtc_y = max(0, crtc_y);

	dsp_stx = crtc_x + crtc->mode.htotal - crtc->mode.hsync_start;
	dsp_sty = crtc_y + crtc->mode.vtotal - crtc->mode.vsync_start;

	offset = (src.x1 >> 16) * (fb->bits_per_pixel >> 3);
	offset += (src.y1 >> 16) * fb->pitches[0];

	y_vir_stride = fb->pitches[0] / (fb->bits_per_pixel >> 3);

	spin_lock(&ctx->reg_lock);

	VOP_WIN_SET(ctx, win, format, format);
	VOP_WIN_SET(ctx, win, yrgb_vir, y_vir_stride);
	yrgb_mst += offset;
	VOP_WIN_SET(ctx, win, yrgb_mst, yrgb_mst);
	val = (actual_h - 1) << 16;
	val |= (actual_w - 1) & 0xffff;
	VOP_WIN_SET(ctx, win, act_info, val);
	VOP_WIN_SET(ctx, win, dsp_info, val);
	val = (dsp_sty - 1) << 16;
	val |= (dsp_stx - 1) & 0xffff;
	VOP_WIN_SET(ctx, win, dsp_st, val);

	if (is_alpha) {
		VOP_WIN_SET(ctx, win, dst_alpha_ctl,
			    DST_FACTOR_M0(ALPHA_SRC_INVERSE));
		val = SRC_ALPHA_EN(1) | SRC_COLOR_M0(ALPHA_SRC_PRE_MUL) |
			SRC_ALPHA_M0(ALPHA_STRAIGHT) |
			SRC_BLEND_M0(ALPHA_PER_PIX) |
			SRC_ALPHA_CAL_M0(ALPHA_NO_SATURATION) |
			SRC_FACTOR_M0(ALPHA_ONE);
		VOP_WIN_SET(ctx, win, src_alpha_ctl, val);
	} else {
		VOP_WIN_SET(ctx, win, src_alpha_ctl, SRC_ALPHA_EN(0));
	}

	VOP_WIN_SET(ctx, win, enable, 1);

	spin_unlock(&ctx->reg_lock);

	mutex_lock(&ctx->vsync_mutex);

	/*
	 * Because the buffer set to vop take effect at frame start time,
	 * we need make sure old buffer is not in use before we release
	 * it.
	 * reference the framebuffer, and unference it when it swap out of vop.
	 */
	if (fb != rockchip_plane->front_fb) {
		drm_framebuffer_reference(fb);
		if (rockchip_plane->pending_fb)
			drm_framebuffer_unreference(rockchip_plane->pending_fb);
		rockchip_plane->pending_fb = fb;
		rockchip_plane->pending_yrgb_mst = yrgb_mst;
		ctx->vsync_work_pending = true;
	}
	rockchip_plane->enabled = true;

	mutex_unlock(&ctx->vsync_mutex);

	spin_lock(&ctx->reg_lock);
	vop_cfg_done(ctx);
	spin_unlock(&ctx->reg_lock);

	return 0;
}

static inline int rockchip_update_primary_plane(struct drm_crtc *crtc)
{
	unsigned int crtc_w, crtc_h;

	crtc_w = crtc->primary->fb->width - crtc->x;
	crtc_h = crtc->primary->fb->height - crtc->y;

	return rockchip_update_plane(crtc->primary, crtc, crtc->primary->fb,
				     0, 0, crtc_w, crtc_h, crtc->x << 16,
				     crtc->y << 16, crtc_w << 16, crtc_h << 16);
}

static int rockchip_disable_plane(struct drm_plane *plane)
{
	struct rockchip_plane *rockchip_plane = to_rockchip_plane(plane);
	const struct vop_win *win = rockchip_plane->win;
	struct vop_context *ctx;

	if (!plane->crtc || !rockchip_plane->enabled)
		return 0;

	ctx = to_vop_ctx(plane->crtc);
	spin_lock(&ctx->reg_lock);

	VOP_WIN_SET(ctx, win, enable, 0);
	vop_cfg_done(ctx);

	spin_unlock(&ctx->reg_lock);

	mutex_lock(&ctx->vsync_mutex);

	/*
	* clear the pending framebuffer and set vsync_work_pending true,
	* so that the framebuffer will unref at the next vblank.
	*/
	if (rockchip_plane->pending_fb) {
		drm_framebuffer_unreference(rockchip_plane->pending_fb);
		rockchip_plane->pending_fb = NULL;
	}

	rockchip_plane->enabled = false;
	ctx->vsync_work_pending = true;

	mutex_unlock(&ctx->vsync_mutex);

	return 0;
}

static void rockchip_plane_destroy(struct drm_plane *plane)
{
	struct rockchip_plane *rockchip_plane = to_rockchip_plane(plane);

	rockchip_disable_plane(plane);
	drm_plane_cleanup(plane);
	kfree(rockchip_plane);
}

static const struct drm_plane_funcs rockchip_plane_funcs = {
	.update_plane = rockchip_update_plane,
	.disable_plane = rockchip_disable_plane,
	.destroy = rockchip_plane_destroy,
};

static struct drm_plane *rockchip_plane_init(struct vop_context *ctx,
					     unsigned long possible_crtcs,
					     enum drm_plane_type type,
					     int index)
{
	struct rockchip_plane *rockchip_plane;
	struct vop_driver_data *vop_data = ctx->data;
	const struct vop_win *win;
	int err;

	if (index >= VOP_MAX_WIN_SUPPORT)
		return ERR_PTR(-EINVAL);

	rockchip_plane = kzalloc(sizeof(*rockchip_plane), GFP_KERNEL);
	if (!rockchip_plane)
		return ERR_PTR(-ENOMEM);

	win = vop_data->win[index];
	rockchip_plane->id = index;
	rockchip_plane->win = win;

	err = drm_universal_plane_init(ctx->drm_dev, &rockchip_plane->base,
				       possible_crtcs, &rockchip_plane_funcs,
				       win->phy->data_formats,
				       win->phy->nformats, type);
	if (err) {
		DRM_ERROR("failed to initialize plane\n");
		kfree(rockchip_plane);
		return ERR_PTR(err);
	}

	return &rockchip_plane->base;
}

int rockchip_drm_crtc_mode_config(struct drm_crtc *crtc,
				  int connector_type,
				  int out_mode)
{
	struct vop_context *ctx = to_vop_ctx(crtc);

	ctx->connector_type = connector_type;
	ctx->connector_out_mode = out_mode;

	return 0;
}

static struct drm_crtc *rockchip_drm_find_crtc(struct drm_device *drm, int pipe)
{
	struct drm_crtc *c, *crtc = NULL;
	int i;

	list_for_each_entry(c, &drm->mode_config.crtc_list, head)
		if (i++ == pipe) {
			crtc = c;
			break;
		}

	return crtc;
}

int rockchip_drm_crtc_enable_vblank(struct drm_device *dev, int pipe)
{
	struct vop_context *ctx = to_vop_ctx(rockchip_drm_find_crtc(dev, pipe));
	unsigned long flags;

	if (ctx->dpms != DRM_MODE_DPMS_ON)
		return -EPERM;

	spin_lock_irqsave(&ctx->irq_lock, flags);

	vop_mask_write(ctx, INTR_CTRL0, FS_INTR_MASK, FS_INTR_EN(1));

	spin_unlock_irqrestore(&ctx->irq_lock, flags);

	return 0;
}

void rockchip_drm_crtc_disable_vblank(struct drm_device *dev, int pipe)
{
	struct vop_context *ctx = to_vop_ctx(rockchip_drm_find_crtc(dev, pipe));
	unsigned long flags;

	if (ctx->dpms != DRM_MODE_DPMS_ON)
		return;
	spin_lock_irqsave(&ctx->irq_lock, flags);
	vop_mask_write(ctx, INTR_CTRL0, FS_INTR_MASK, FS_INTR_EN(0));
	spin_unlock_irqrestore(&ctx->irq_lock, flags);
}

static void rockchip_drm_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct vop_context *ctx = to_vop_ctx(crtc);

	DRM_DEBUG_KMS("crtc[%d] mode[%d]\n", crtc->base.id, mode);

	if (ctx->dpms == mode) {
		DRM_DEBUG_KMS("desired dpms mode is same as previous one.\n");
		return;
	}

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		rockchip_power_on(crtc);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		rockchip_power_off(crtc);
		break;
	default:
		DRM_DEBUG_KMS("unspecified mode %d\n", mode);
		break;
	}

	ctx->dpms = mode;
}

static void rockchip_drm_crtc_prepare(struct drm_crtc *crtc)
{
	rockchip_drm_crtc_dpms(crtc, DRM_MODE_DPMS_ON);
}

static bool rockchip_drm_crtc_mode_fixup(struct drm_crtc *crtc,
					 const struct drm_display_mode *mode,
					 struct drm_display_mode *adjusted_mode)
{
	if (adjusted_mode->htotal == 0 || adjusted_mode->vtotal == 0)
		return false;

	return true;
}

static int rockchip_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
					   struct drm_framebuffer *old_fb)
{
	int ret;

	crtc->x = x;
	crtc->y = y;

	ret = rockchip_update_primary_plane(crtc);
	if (ret < 0) {
		DRM_ERROR("fail to update plane\n");
		return ret;
	}

	return 0;
}

static int rockchip_drm_crtc_mode_set(struct drm_crtc *crtc,
				      struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode,
				      int x, int y,
				      struct drm_framebuffer *fb)
{
	struct vop_context *ctx = to_vop_ctx(crtc);
	u16 hsync_len = adjusted_mode->hsync_end - adjusted_mode->hsync_start;
	u16 hdisplay = adjusted_mode->hdisplay;
	u16 htotal = adjusted_mode->htotal;
	u16 hact_st = adjusted_mode->htotal - adjusted_mode->hsync_start;
	u16 hact_end = hact_st + hdisplay;
	u16 vdisplay = adjusted_mode->vdisplay;
	u16 vtotal = adjusted_mode->vtotal;
	u16 vsync_len = adjusted_mode->vsync_end - adjusted_mode->vsync_start;
	u16 vact_st = adjusted_mode->vtotal - adjusted_mode->vsync_start;
	u16 vact_end = vact_st + vdisplay;
	int ret;
	uint32_t val;

	/*
	 * disable dclk to stop frame scan, so that we can safe config mode and
	 * enable iommu.
	 */
	clk_disable(ctx->dclk);

	ret = rockchip_drm_crtc_mode_set_base(crtc, x, y, fb);
	if (ret)
		return ret;

	switch (ctx->connector_type) {
	case DRM_MODE_CONNECTOR_LVDS:
		VOP_CTRL_SET(ctx, rgb_en, 1);
		break;
	case DRM_MODE_CONNECTOR_eDP:
		VOP_CTRL_SET(ctx, edp_en, 1);
		break;
	case DRM_MODE_CONNECTOR_HDMIA:
		VOP_CTRL_SET(ctx, hdmi_en, 1);
		break;
	default:
		DRM_ERROR("unsupport connector_type[%d]\n",
			  ctx->connector_type);
		return -EINVAL;
	};
	VOP_CTRL_SET(ctx, out_mode, ctx->connector_out_mode);

	val = 0x8;
	val |= (adjusted_mode->flags & DRM_MODE_FLAG_NHSYNC) ? 1 : 0;
	val |= (adjusted_mode->flags & DRM_MODE_FLAG_NVSYNC) ? (1 << 1) : 0;
	VOP_CTRL_SET(ctx, pin_pol, val);

	VOP_CTRL_SET(ctx, htotal_pw, (htotal << 16) | hsync_len);
	val = hact_st << 16;
	val |= hact_end;
	VOP_CTRL_SET(ctx, hact_st_end, val);
	VOP_CTRL_SET(ctx, hpost_st_end, val);

	VOP_CTRL_SET(ctx, vtotal_pw, (vtotal << 16) | vsync_len);
	val = vact_st << 16;
	val |= vact_end;
	VOP_CTRL_SET(ctx, vact_st_end, val);
	VOP_CTRL_SET(ctx, vpost_st_end, val);
	/*
	 * reset dclk, take all mode config affect, so the clk would run in
	 * correct frame.
	 */
	reset_control_assert(ctx->dclk_rst);
	usleep_range(10, 20);
	reset_control_deassert(ctx->dclk_rst);

	clk_set_rate(ctx->dclk, adjusted_mode->clock * 1000);
	ret = clk_enable(ctx->dclk);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to enable dclk - %d\n", ret);
		return ret;
	}

	return 0;
}

static void rockchip_drm_crtc_commit(struct drm_crtc *crtc)
{
}

static const struct drm_crtc_helper_funcs rockchip_crtc_helper_funcs = {
	.dpms = rockchip_drm_crtc_dpms,
	.prepare = rockchip_drm_crtc_prepare,
	.mode_fixup = rockchip_drm_crtc_mode_fixup,
	.mode_set = rockchip_drm_crtc_mode_set,
	.mode_set_base = rockchip_drm_crtc_mode_set_base,
	.commit = rockchip_drm_crtc_commit,
};

static int rockchip_drm_crtc_page_flip(struct drm_crtc *crtc,
				       struct drm_framebuffer *fb,
				       struct drm_pending_vblank_event *event,
				       uint32_t page_flip_flags)
{
	struct drm_device *dev = crtc->dev;
	struct vop_context *ctx = to_vop_ctx(crtc);
	struct drm_framebuffer *old_fb = crtc->primary->fb;
	int pipe = ctx->pipe;
	int ret;

	/* when the page flip is requested, crtc's dpms should be on */
	if (ctx->dpms > DRM_MODE_DPMS_ON) {
		DRM_DEBUG("failed page flip request at dpms[%d].\n", ctx->dpms);
		return 0;
	}

	ret = drm_vblank_get(dev, pipe);
	if (ret) {
		DRM_DEBUG("failed to acquire vblank counter\n");
		return ret;
	}

	spin_lock_irq(&dev->event_lock);
	if (ctx->event) {
		spin_unlock_irq(&dev->event_lock);
		DRM_ERROR("already pending flip!\n");
		return -EBUSY;
	}
	ctx->event = event;
	atomic_set(&ctx->wait_vsync_event, 1);
	spin_unlock_irq(&dev->event_lock);

	crtc->primary->fb = fb;

	ret = rockchip_update_primary_plane(crtc);
	if (ret) {
		crtc->primary->fb = old_fb;

		spin_lock_irq(&dev->event_lock);
		drm_vblank_put(dev, pipe);
		atomic_set(&ctx->wait_vsync_event, 0);
		ctx->event = NULL;
		spin_unlock_irq(&dev->event_lock);
	}

	return ret;
}

static void rockchip_drm_crtc_finish_pageflip(struct drm_crtc *crtc)
{
	struct vop_context *ctx = to_vop_ctx(crtc);
	struct drm_device *drm = ctx->drm_dev;
	unsigned long flags;

	spin_lock_irqsave(&drm->event_lock, flags);

	if (ctx->event) {
		drm_send_vblank_event(drm, -1, ctx->event);
		drm_vblank_put(drm, ctx->pipe);
		atomic_set(&ctx->wait_vsync_event, 0);
		wake_up(&ctx->wait_vsync_queue);
		ctx->event = NULL;
	}

	spin_unlock_irqrestore(&drm->event_lock, flags);
}

static void rockchip_drm_crtc_destroy(struct drm_crtc *crtc)
{
	drm_crtc_cleanup(crtc);
}

static const struct drm_crtc_funcs rockchip_crtc_funcs = {
	.set_config = drm_crtc_helper_set_config,
	.page_flip = rockchip_drm_crtc_page_flip,
	.destroy = rockchip_drm_crtc_destroy,
};

static void rockchip_vsync_worker(struct work_struct *work)
{
	struct vop_context *ctx = container_of(work, struct vop_context,
					       vsync_work);
	struct drm_device *drm = ctx->drm_dev;
	struct drm_crtc *crtc = &ctx->crtc;
	struct rockchip_plane *rockchip_plane;
	struct drm_plane *plane;
	uint32_t yrgb_mst;

	mutex_lock(&ctx->vsync_mutex);

	ctx->vsync_work_pending = false;

	list_for_each_entry(plane, &drm->mode_config.plane_list, head) {
		rockchip_plane = to_rockchip_plane(plane);

		if (plane->crtc != crtc)
			continue;
		if (rockchip_plane->enabled && !rockchip_plane->pending_fb)
			continue;
		if (!rockchip_plane->enabled && !rockchip_plane->front_fb)
			continue;
		/*
		 * make sure the yrgb_mst take effect, so that
		 * we can unreference the old framebuffer.
		 */
		yrgb_mst = VOP_WIN_GET_YRGBADDR(ctx, rockchip_plane->win);
		if (rockchip_plane->pending_yrgb_mst != yrgb_mst) {
			/*
			 * some plane no complete, unref at next vblank
			 */
			ctx->vsync_work_pending = true;
			continue;
		}

		/*
		 * drm_framebuffer_unreference maybe call iommu unmap,
		 * and iommu not allow unmap buffer at irq context,
		 * so we do drm_framebuffer_unreference at queue_work.
		 */
		if (rockchip_plane->front_fb)
			drm_framebuffer_unreference(rockchip_plane->front_fb);

		rockchip_plane->front_fb = rockchip_plane->pending_fb;
		rockchip_plane->pending_fb = NULL;

		/*
		 * if primary plane flip complete, sending the event to
		 * userspace
		 */
		if (&rockchip_plane->base == crtc->primary)
			rockchip_drm_crtc_finish_pageflip(crtc);
	}

	mutex_unlock(&ctx->vsync_mutex);
}

static irqreturn_t rockchip_vop_isr(int irq, void *data)
{
	struct vop_context *ctx = data;
	uint32_t intr0_reg;
	unsigned long flags;

	intr0_reg = vop_readl(ctx, INTR_CTRL0);
	if (intr0_reg & FS_INTR) {
		spin_lock_irqsave(&ctx->irq_lock, flags);
		vop_writel(ctx, INTR_CTRL0, intr0_reg | FS_INTR_CLR);
		spin_unlock_irqrestore(&ctx->irq_lock, flags);
	} else {
		return IRQ_NONE;
	}

	drm_handle_vblank(ctx->drm_dev, ctx->pipe);
	if (ctx->vsync_work_pending)
		queue_work(ctx->vsync_wq, &ctx->vsync_work);

	return IRQ_HANDLED;
}

static int vop_create_crtc(struct vop_context *ctx)
{
	struct device *dev = ctx->dev;
	struct drm_device *drm_dev = ctx->drm_dev;
	struct drm_plane *primary, *cursor, *plane;
	enum drm_plane_type plane_type;
	struct drm_crtc *crtc = &ctx->crtc;
	struct device_node *port;
	int ret;
	int nr;

	for (nr = 0; nr < VOP_MAX_WIN_SUPPORT; nr++) {
		if (nr == VOP_DEFAULT_PRIMARY)
			plane_type = DRM_PLANE_TYPE_PRIMARY;
		else if (nr == VOP_DEFAULT_CURSOR)
			plane_type = DRM_PLANE_TYPE_CURSOR;
		else
			plane_type = DRM_PLANE_TYPE_OVERLAY;

		plane = rockchip_plane_init(ctx, 0xff, plane_type, nr);
		if (IS_ERR(plane)) {
			ret = PTR_ERR(plane);
			DRM_ERROR("fail to init overlay plane - %d\n", ret);
			goto err_destroy_plane;
		}

		if (plane_type == DRM_PLANE_TYPE_PRIMARY)
			primary = plane;
		else if (plane_type == DRM_PLANE_TYPE_CURSOR)
			cursor = plane;
	}

	ret = drm_crtc_init_with_planes(drm_dev, crtc, primary, cursor,
					&rockchip_crtc_funcs);
	if (ret)
		goto err_destroy_plane;

	drm_crtc_helper_add(crtc, &rockchip_crtc_helper_funcs);

	port = of_get_child_by_name(dev->of_node, "port");
	if (!port) {
		DRM_ERROR("no port node found in %s\n",
			  dev->of_node->full_name);
		goto err_cleanup_crtc;
	}

	drm_modeset_lock_all(drm_dev);
	crtc->port = port;
	ctx->pipe = drm_crtc_index(crtc);
	drm_modeset_unlock_all(drm_dev);

	return 0;

err_cleanup_crtc:
	drm_crtc_cleanup(crtc);
err_destroy_plane:
	mutex_lock(&drm_dev->mode_config.mutex);
	list_for_each_entry(plane, &drm_dev->mode_config.plane_list, head)
		plane->funcs->destroy(plane);
	mutex_unlock(&drm_dev->mode_config.mutex);
	return ret;
}

static int rockchip_vop_initial(struct vop_context *ctx)
{
	struct vop_driver_data *vop_data = ctx->data;
	const struct vop_reg_data *init_table = vop_data->init_table;
	struct reset_control *ahb_rst;
	int i, ret;

	ctx->hclk = devm_clk_get(ctx->dev, "hclk_vop");
	if (IS_ERR(ctx->hclk)) {
		dev_err(ctx->dev, "failed to get hclk source\n");
		return PTR_ERR(ctx->hclk);
	}
	ctx->aclk = devm_clk_get(ctx->dev, "aclk_vop");
	if (IS_ERR(ctx->aclk)) {
		dev_err(ctx->dev, "failed to get aclk source\n");
		return PTR_ERR(ctx->aclk);
	}
	ctx->dclk = devm_clk_get(ctx->dev, "dclk_vop");
	if (IS_ERR(ctx->dclk)) {
		dev_err(ctx->dev, "failed to get dclk source\n");
		return PTR_ERR(ctx->dclk);
	}

	ret = clk_prepare(ctx->hclk);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to prepare hclk\n");
		return ret;
	}

	ret = clk_prepare(ctx->dclk);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to prepare dclk\n");
		goto err_unprepare_hclk;
	}

	ret = clk_prepare(ctx->aclk);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to prepare aclk\n");
		goto err_unprepare_dclk;
	}

	/*
	 * enable hclk, so that we can config vop register.
	 */
	ret = clk_enable(ctx->hclk);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to prepare aclk\n");
		goto err_unprepare_aclk;
	}
	/*
	 * do hclk_reset, reset all vop registers.
	 */
	ahb_rst = devm_reset_control_get(ctx->dev, "ahb");
	if (IS_ERR(ahb_rst)) {
		dev_err(ctx->dev, "failed to get ahb reset\n");
		ret = PTR_ERR(ahb_rst);
		goto err_disable_hclk;
	}
	reset_control_assert(ahb_rst);
	usleep_range(10, 20);
	reset_control_deassert(ahb_rst);

	memcpy(ctx->regsbak, ctx->regs, ctx->len);

	for (i = 0; i < vop_data->table_size; i++)
		vop_writel(ctx, init_table[i].offset, init_table[i].value);

	for (i = 0; i < VOP_MAX_WIN_SUPPORT; i++)
		VOP_WIN_SET(ctx, vop_data->win[i], enable, 0);

	vop_cfg_done(ctx);

	/*
	 * do dclk_reset, let all config take affect.
	 */
	ctx->dclk_rst = devm_reset_control_get(ctx->dev, "dclk");
	if (IS_ERR(ctx->dclk_rst)) {
		dev_err(ctx->dev, "failed to get dclk reset\n");
		ret = PTR_ERR(ctx->dclk_rst);
		goto err_unprepare_aclk;
	}
	reset_control_assert(ctx->dclk_rst);
	usleep_range(10, 20);
	reset_control_deassert(ctx->dclk_rst);

	clk_disable(ctx->hclk);

	ctx->dpms = DRM_MODE_DPMS_OFF;

	return 0;

err_disable_hclk:
	clk_disable(ctx->hclk);
err_unprepare_aclk:
	clk_unprepare(ctx->aclk);
err_unprepare_dclk:
	clk_unprepare(ctx->dclk);
err_unprepare_hclk:
	clk_unprepare(ctx->hclk);
	return ret;
}

static int vop_bind(struct device *dev, struct device *master, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct vop_driver_data *vop_data = vop_get_driver_data(dev);
	struct drm_device *drm_dev = data;
	struct vop_context *ctx;
	struct resource *res;
	int ret;

	if (!vop_data)
		return -ENODEV;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	ctx->data = vop_data;
	ctx->drm_dev = drm_dev;
	dev_set_drvdata(dev, ctx);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctx->len = resource_size(res);
	ctx->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(ctx->regs))
		return PTR_ERR(ctx->regs);

	ctx->regsbak = devm_kzalloc(dev, ctx->len, GFP_KERNEL);
	if (!ctx->regsbak)
		return -ENOMEM;

	ret = rockchip_vop_initial(ctx);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot initial vop dev - err %d\n", ret);
		return ret;
	}

	ctx->irq = platform_get_irq(pdev, 0);
	if (ctx->irq < 0) {
		dev_err(dev, "cannot find irq for vop\n");
		return ctx->irq;
	}

	spin_lock_init(&ctx->reg_lock);
	spin_lock_init(&ctx->irq_lock);

	init_waitqueue_head(&ctx->wait_vsync_queue);
	atomic_set(&ctx->wait_vsync_event, 0);

	ret = vop_iommu_init(ctx);
	if (ret) {
		DRM_ERROR("Failed to setup iommu, %d\n", ret);
		return ret;
	}

	ctx->vsync_wq = create_singlethread_workqueue("vsync");
	if (!ctx->vsync_wq) {
		dev_err(dev, "failed to create workqueue\n");
		return -EINVAL;
	}
	INIT_WORK(&ctx->vsync_work, rockchip_vsync_worker);

	mutex_init(&ctx->vsync_mutex);
	pm_runtime_enable(&pdev->dev);

	ret = devm_request_irq(dev, ctx->irq, rockchip_vop_isr,
			       IRQF_SHARED, dev_name(dev), ctx);
	if (ret) {
		dev_err(dev, "cannot requeset irq%d - err %d\n", ctx->irq, ret);
		return ret;
	}

	return vop_create_crtc(ctx);
}

static void vop_unbind(struct device *dev, struct device *master,
		       void *data)
{
	struct vop_context *ctx = dev_get_drvdata(dev);
	struct drm_crtc *crtc = &ctx->crtc;

	of_node_put(crtc->port);
	drm_crtc_cleanup(crtc);
	pm_runtime_disable(dev);
}

static const struct component_ops vop_component_ops = {
	.bind = vop_bind,
	.unbind = vop_unbind,
};

static int vop_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vop_context *ctx;

	if (!dev->of_node) {
		dev_err(dev, "can't find vop devices\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, ctx);

	return component_add(dev, &vop_component_ops);
}

static int vop_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &vop_component_ops);

	return 0;
}

struct platform_driver rockchip_vop_platform_driver = {
	.probe = vop_probe,
	.remove = vop_remove,
	.driver = {
		.name = "rockchip-vop",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(vop_driver_dt_match),
	},
};

module_platform_driver(rockchip_vop_platform_driver);

MODULE_AUTHOR("Mark Yao <mark.yao@rock-chips.com>");
MODULE_DESCRIPTION("ROCKCHIP VOP Driver");
MODULE_LICENSE("GPL v2");
