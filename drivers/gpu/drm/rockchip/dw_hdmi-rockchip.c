/*
 * Copyright (c) 2014, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <drm/drm_of.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>
#include <drm/bridge/dw_hdmi.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_vop.h"

#define GRF_SOC_CON6                    0x025c
#define HDMI_SEL_VOP_LIT                (1 << 4)

struct rockchip_hdmi {
	struct device *dev;
	struct regmap *regmap;
	struct drm_encoder encoder;
};

#define to_rockchip_hdmi(x)	container_of(x, struct rockchip_hdmi, x)

static const int dw_hdmi_rates[] = {
	25175000,
	27000000,
	28320000,
	30240000,
	31500000,
	36000000,
	40000000,
	49500000,
	50000000,
	54000000,
	57280000,
	65000000,
	66000000,
	71000000,
	73250000,
	74250000,
	75000000,
	78750000,
	83500000,
	85500000,
	88750000,
	106500000,
	108000000,
	115500000,
	119000000,
	135000000,
	136750000,
	146250000,
	148500000,
	154000000,
	162000000,
	172800000,
};

static const struct dw_hdmi_mpll_config rockchip_mpll_cfg[] = {
	{
		40000000, {
			{ 0x00b3, 0x0000},
			{ 0x2153, 0x0000},
			{ 0x40f3, 0x0000}
		},
	}, {
		65000000, {
			{ 0x0072, 0x0001},
			{ 0x2142, 0x0001},
			{ 0x40a2, 0x0001},
		},
	}, {
		66000000, {
			{ 0x013e, 0x0003},
			{ 0x217e, 0x0002},
			{ 0x4061, 0x0002}
		},
	}, {
		88750000, {
			{ 0x0072, 0x0001},
			{ 0x2145, 0x0002},
			{ 0x4061, 0x0002}
		},
	}, {
		146250000, {
			{ 0x0051, 0x0002},
			{ 0x2145, 0x0002},
			{ 0x4061, 0x0002}
		},
	}, {
		148500000, {
			{ 0x0051, 0x0003},
			{ 0x214c, 0x0003},
			{ 0x4064, 0x0003}
		},
	}, {
		172800000, {
			{ 0x0051, 0x0002},
		},
	}, {
		~0UL, {
			{ 0x00a0, 0x000a },
			{ 0x2001, 0x000f },
			{ 0x4002, 0x000f },
		},
	}
};

static const struct dw_hdmi_curr_ctrl rockchip_cur_ctr[] = {
	/*      pixelclk    bpp8    bpp10   bpp12 */
	{
		40000000,  { 0x0018, 0x0018, 0x0018 },
	}, {
		65000000,  { 0x0028, 0x0028, 0x0028 },
	}, {
		66000000,  { 0x0038, 0x0038, 0x0038 },
	}, {
		88750000,  { 0x0028, 0x0038, 0x0038 },
	}, {
		172800000, { 0x0038, 0x0038, 0x0038 },
	}, {
		~0UL,      { 0x0000, 0x0000, 0x0000},
	}
};

static const struct dw_hdmi_phy_config rockchip_phy_config[] = {
	/*pixelclk   symbol   term   vlev*/
	{ 74250000,  0x8009, 0x0004, 0x0272},
	{ 148500000, 0x802b, 0x0004, 0x028d},
	{ 297000000, 0x8039, 0x0005, 0x028d},
	{ ~0UL,	     0x0000, 0x0000, 0x0000}
};

static int rockchip_hdmi_parse_dt(struct rockchip_hdmi *hdmi)
{
	struct device_node *np = hdmi->dev->of_node;

	hdmi->regmap = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(hdmi->regmap)) {
		dev_err(hdmi->dev, "Unable to get rockchip,grf\n");
		return PTR_ERR(hdmi->regmap);
	}

	return 0;
}

static enum drm_mode_status
dw_hdmi_rockchip_mode_valid(struct drm_connector *connector,
			    struct drm_display_mode *mode)
{
	int pclk = mode->clock * 1000;
	int i;

	for (i = 0; i < ARRAY_SIZE(dw_hdmi_rates); i++)
		if (pclk == dw_hdmi_rates[i])
			return MODE_OK;

	return MODE_BAD;
}

static struct drm_encoder_funcs dw_hdmi_rockchip_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static void dw_hdmi_rockchip_encoder_disable(struct drm_encoder *encoder)
{
}

static bool
dw_hdmi_rockchip_encoder_mode_fixup(struct drm_encoder *encoder,
				    const struct drm_display_mode *mode,
				    struct drm_display_mode *adj_mode)
{
	/*
	 * Hardware: Because we can not find an NPLL rate which have
	 * generate both accurate 136.75MHz and good jitter performance,
	 * we change dclk_vop0 to 136.80MHz, so dclk_vop0 could have good jitter
	 * performance(118.790ps).  When we pick 136.80MHz clk driver could
	 * choose the 1368MHz for us.  Normally the clock driver can't pick
	 * clocks that are higher that we request.
	 *
	 * We adjust 146.25MHz for a similar reason.  If we don't adjust
	 * then the clock framework will pick 585MHz which has bad jitter.
	 * We instead specify 146.200MHz and will pick 1608MHz
	 */
	if (adj_mode->clock == 136750)
		adj_mode->clock = 136800;
	if (adj_mode->clock == 146250)
		adj_mode->clock = 146200;

	return true;
}

static void dw_hdmi_rockchip_encoder_mode_set(struct drm_encoder *encoder,
					      struct drm_display_mode *mode,
					      struct drm_display_mode *adj_mode)
{
}

static void dw_hdmi_rockchip_encoder_commit(struct drm_encoder *encoder)
{
	struct rockchip_hdmi *hdmi = to_rockchip_hdmi(encoder);
	u32 val;
	int mux;

	mux = rockchip_drm_encoder_get_mux_id(hdmi->dev->of_node, encoder);
	if (mux)
		val = HDMI_SEL_VOP_LIT | (HDMI_SEL_VOP_LIT << 16);
	else
		val = HDMI_SEL_VOP_LIT << 16;

	regmap_write(hdmi->regmap, GRF_SOC_CON6, val);
	dev_dbg(hdmi->dev, "vop %s output to hdmi\n",
		(mux) ? "LIT" : "BIG");
}

static void dw_hdmi_rockchip_encoder_prepare(struct drm_encoder *encoder)
{
	rockchip_drm_crtc_mode_config(encoder->crtc, DRM_MODE_CONNECTOR_HDMIA,
				      ROCKCHIP_OUT_MODE_AAAA);
}

static struct drm_encoder_helper_funcs dw_hdmi_rockchip_encoder_helper_funcs = {
	.mode_fixup = dw_hdmi_rockchip_encoder_mode_fixup,
	.mode_set   = dw_hdmi_rockchip_encoder_mode_set,
	.prepare    = dw_hdmi_rockchip_encoder_prepare,
	.commit     = dw_hdmi_rockchip_encoder_commit,
	.disable    = dw_hdmi_rockchip_encoder_disable,
};

static const struct dw_hdmi_plat_data rockchip_hdmi_drv_data = {
	.mode_valid = dw_hdmi_rockchip_mode_valid,
	.mpll_cfg   = rockchip_mpll_cfg,
	.cur_ctr    = rockchip_cur_ctr,
	.phy_config = rockchip_phy_config,
	.dev_type   = RK3288_HDMI,
};

static const struct of_device_id dw_hdmi_rockchip_ids[] = {
	{ .compatible = "rockchip,rk3288-dw-hdmi",
	  .data = &rockchip_hdmi_drv_data
	},
	{},
};
MODULE_DEVICE_TABLE(of, dw_hdmi_rockchip_dt_ids);

static int dw_hdmi_rockchip_bind(struct device *dev, struct device *master,
				 void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct dw_hdmi_plat_data *plat_data;
	const struct of_device_id *match;
	struct drm_device *drm = data;
	struct drm_encoder *encoder;
	struct rockchip_hdmi *hdmi;
	struct resource *iores;
	int irq;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	hdmi = devm_kzalloc(&pdev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	match = of_match_node(dw_hdmi_rockchip_ids, pdev->dev.of_node);
	plat_data = match->data;
	hdmi->dev = &pdev->dev;
	encoder = &hdmi->encoder;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iores)
		return -ENXIO;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);
	/*
	 * If we failed to find the CRTC(s) which this encoder is
	 * supposed to be connected to, it's because the CRTC has
	 * not been registered yet.  Defer probing, and hope that
	 * the required CRTC is added later.
	 */
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	ret = rockchip_hdmi_parse_dt(hdmi);
	if (ret) {
		dev_err(hdmi->dev, "Unable to parse OF data\n");
		return ret;
	}

	drm_encoder_helper_add(encoder, &dw_hdmi_rockchip_encoder_helper_funcs);
	drm_encoder_init(drm, encoder, &dw_hdmi_rockchip_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS);

	ret = dw_hdmi_bind(dev, master, data, encoder, iores, irq, plat_data);

	/*
	 * If dw_hdmi_bind() fails we'll never call dw_hdmi_unbind(),
	 * which would have called the encoder cleanup.  Do it manually.
	 */
	if (ret)
		drm_encoder_cleanup(encoder);

	return ret;
}

static void dw_hdmi_rockchip_unbind(struct device *dev, struct device *master,
				    void *data)
{
	return dw_hdmi_unbind(dev, master, data);
}

static const struct component_ops dw_hdmi_rockchip_ops = {
	.bind	= dw_hdmi_rockchip_bind,
	.unbind	= dw_hdmi_rockchip_unbind,
};

static int dw_hdmi_rockchip_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &dw_hdmi_rockchip_ops);
}

static int dw_hdmi_rockchip_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dw_hdmi_rockchip_ops);

	return 0;
}

static int dw_hdmi_rockchip_suspend(struct device *dev)
{
	return dw_hdmi_suspend(dev);
}

static int dw_hdmi_rockchip_resume(struct device *dev)
{
	return dw_hdmi_resume(dev);
}

static const struct dev_pm_ops dw_hdmi_rockchip_pm = {
	.resume = dw_hdmi_rockchip_resume,
	.suspend = dw_hdmi_rockchip_suspend,
};

static struct platform_driver dw_hdmi_rockchip_pltfm_driver = {
	.probe  = dw_hdmi_rockchip_probe,
	.remove = dw_hdmi_rockchip_remove,
	.driver = {
		.name = "dwhdmi-rockchip",
		.pm = &dw_hdmi_rockchip_pm,
		.of_match_table = dw_hdmi_rockchip_ids,
	},
};

module_platform_driver(dw_hdmi_rockchip_pltfm_driver);

MODULE_AUTHOR("Andy Yan <andy.yan@rock-chips.com>");
MODULE_AUTHOR("Yakir Yang <ykk@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip Specific DW-HDMI Driver Extension");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dwhdmi-rockchip");
