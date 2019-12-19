// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 Rockchip Electronics Co. Ltd.
 *
 * Author: Guochun Huang <hero.huang@rock-chips.com
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/mfd/rk628.h>
#include <linux/reset.h>
#include <linux/phy/phy.h>

#include <drm/drmP.h>
#include <drm/drm_of.h>
#include <drm/drm_atomic.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>

#include <video/of_display_timing.h>
#include <video/videomode.h>

struct rk628_bt1120 {
	struct drm_bridge base;
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	struct drm_connector connector;
	struct device *dev;
	struct regmap *grf;
	struct rk628 *parent;
	struct clk *decclk;
	struct reset_control *rstc;
	struct drm_display_mode mode;
	bool decoder;
	bool dual_edge;
};

static inline struct rk628_bt1120 *bridge_to_bt1120(struct drm_bridge *b)
{
	return container_of(b, struct rk628_bt1120, base);
}

static inline struct rk628_bt1120 *connector_to_bt1120(struct drm_connector *c)
{
	return container_of(c, struct rk628_bt1120, connector);
}

static int rk628_bt1120_connector_get_modes(struct drm_connector *connector)
{
	struct rk628_bt1120 *bt1120 = connector_to_bt1120(connector);
	int num = 0;

	num = drm_add_modes_noedid(&bt1120->connector, 1920, 1080);
	drm_set_preferred_mode(&bt1120->connector, 1920, 1080);

	return num;
}

static const struct drm_connector_helper_funcs
rk628_bt1120_connector_helper_funcs = {
	.get_modes = rk628_bt1120_connector_get_modes,
};

static enum drm_connector_status
rk628_bt1120_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void rk628_bt1120_connector_destroy(struct drm_connector *connector)
{
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs rk628_bt1120_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.detect = rk628_bt1120_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = rk628_bt1120_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static void bt1120_bridge_mode_set(struct drm_bridge *bridge,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adj)
{
	struct rk628_bt1120 *bt1120 = bridge_to_bt1120(bridge);

	drm_mode_copy(&bt1120->mode, adj);
}

static void rk628_bt1120_bridge_enable(struct drm_bridge *bridge)
{
	struct rk628_bt1120 *bt1120 = bridge_to_bt1120(bridge);
	const struct drm_display_mode *mode = &bt1120->mode;
	u32 val = 0;

	if (bt1120->decoder) {
		reset_control_assert(bt1120->rstc);
		udelay(10);
		reset_control_deassert(bt1120->rstc);
		udelay(10);

		clk_set_rate(bt1120->decclk, mode->clock * 1000);
		clk_prepare_enable(bt1120->decclk);

		if (bt1120->dual_edge) {

			regmap_update_bits(bt1120->grf, GRF_RGB_DEC_CON0,
					   DEC_DUALEDGE_EN, DEC_DUALEDGE_EN);

			regmap_write(bt1120->grf,
				     GRF_BT1120_DCLK_DELAY_CON0, 0x10000000);
			regmap_write(bt1120->grf,
				     GRF_BT1120_DCLK_DELAY_CON1, 0);
		} else
			regmap_update_bits(bt1120->grf, GRF_RGB_DEC_CON0,
					   DEC_DUALEDGE_EN, 0);

		regmap_update_bits(bt1120->grf, GRF_RGB_DEC_CON1,
				   SW_SET_X_MASK, SW_SET_X(mode->hdisplay));
		regmap_update_bits(bt1120->grf, GRF_RGB_DEC_CON2,
				   SW_SET_Y_MASK, SW_SET_Y(mode->vdisplay));

		regmap_update_bits(bt1120->grf, GRF_SYSTEM_CON0,
			SW_BT_DATA_OEN_MASK | SW_INPUT_MODE_MASK,
			SW_BT_DATA_OEN | SW_INPUT_MODE(INPUT_MODE_BT1120));

		regmap_write(bt1120->grf, GRF_CSC_CTRL_CON, SW_Y2R_EN(1));

		regmap_update_bits(bt1120->grf, GRF_RGB_DEC_CON0,
				   SW_CAP_EN_PSYNC | SW_CAP_EN_ASYNC | SW_PROGRESS_EN,
				   SW_CAP_EN_PSYNC |  SW_CAP_EN_ASYNC | SW_PROGRESS_EN);
	} else {
		regmap_update_bits(bt1120->grf, GRF_SYSTEM_CON0,
				   SW_BT_DATA_OEN_MASK | SW_OUTPUT_MODE_MASK,
				   SW_OUTPUT_MODE(OUTPUT_MODE_BT1120));

		regmap_write(bt1120->grf, GRF_CSC_CTRL_CON, SW_R2Y_EN(1));

		regmap_update_bits(bt1120->grf, GRF_POST_PROC_CON,
				   SW_DCLK_OUT_INV_EN, SW_DCLK_OUT_INV_EN);

		if (bt1120->dual_edge) {
			val |= ENC_DUALEDGE_EN(1);

			regmap_write(bt1120->grf,
				     GRF_BT1120_DCLK_DELAY_CON0, 0x10000000);
			regmap_write(bt1120->grf,
				     GRF_BT1120_DCLK_DELAY_CON1, 0);
		}

		val |= BT1120_UV_SWAP(1);
		regmap_write(bt1120->grf, GRF_RGB_ENC_CON, val);
	}
}

static void rk628_bt1120_bridge_disable(struct drm_bridge *bridge)
{
	struct rk628_bt1120 *bt1120 = bridge_to_bt1120(bridge);

	if (bt1120->decclk)
		clk_disable_unprepare(bt1120->decclk);

	if (bt1120->rstc)
		reset_control_assert(bt1120->rstc);
}

static int rk628_bt1120_bridge_attach(struct drm_bridge *bridge)
{
	struct rk628_bt1120 *bt1120 = bridge_to_bt1120(bridge);
	struct drm_connector *connector = &bt1120->connector;
	struct drm_device *drm = bridge->dev;
	struct device *dev = bt1120->dev;
	int ret;

	ret = drm_of_find_panel_or_bridge(dev->of_node, 1, -1,
					  &bt1120->panel, &bt1120->bridge);
	if (ret) {
		bridge->dev = NULL;
		dev_err(dev, "failed to find bridge\n");
		return ret;
	}

	if (bt1120->decoder) {
		if (!bt1120->bridge) {
			dev_err(dev, "bt1120 decoder failed to find bridge\n");
			return -EPROBE_DEFER;
		}

		bt1120->bridge->encoder = bridge->encoder;
		ret = drm_bridge_attach(bridge->dev, bt1120->bridge);
		if (ret) {
			dev_err(dev, "failed to attach bridge\n");
			return ret;
		}

		bridge->next = bt1120->bridge;

	} else {
		if (bt1120->bridge) {
			bt1120->bridge->encoder = bridge->encoder;
			ret = drm_bridge_attach(bridge->dev, bt1120->bridge);
			if (ret) {
				dev_err(dev, "failed to attach bridge\n");
				return ret;
			}

			bridge->next = bt1120->bridge;
		}

		if (bt1120->panel) {
			ret = drm_connector_init(drm, connector,
						 &rk628_bt1120_connector_funcs,
						 DRM_MODE_CONNECTOR_HDMIA);
			if (ret) {
				dev_err(bt1120->dev,
					"Failed to initialize connector with drm\n");
				return ret;
			}

			drm_connector_helper_add(connector,
						 &rk628_bt1120_connector_helper_funcs);
			drm_mode_connector_attach_encoder(connector, bridge->encoder);

		}
	}

	return 0;
}

static const struct drm_bridge_funcs rk628_bt1120_bridge_funcs = {
	.attach = rk628_bt1120_bridge_attach,
	.mode_set = bt1120_bridge_mode_set,
	.enable = rk628_bt1120_bridge_enable,
	.disable = rk628_bt1120_bridge_disable,
};

static int rk628_bt1120_probe(struct platform_device *pdev)
{
	struct rk628 *rk628 = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct rk628_bt1120 *bt1120;
	int ret;

	bt1120 = devm_kzalloc(dev, sizeof(*bt1120), GFP_KERNEL);
	if (!bt1120)
		return -ENOMEM;

	bt1120->dev = dev;
	bt1120->parent = rk628;
	platform_set_drvdata(pdev, bt1120);

	bt1120->grf = rk628->grf;
	bt1120->dual_edge = of_property_read_bool(dev->of_node, "dual-edge");
	bt1120->decoder = of_property_read_bool(dev->of_node, "bt1120-decoder");

	if (bt1120->decoder) {
		bt1120->decclk = devm_clk_get(dev, "bt1120dec");
		if (IS_ERR(bt1120->decclk)) {
			ret = PTR_ERR(bt1120->decclk);
			dev_err(dev, "failed to get dec clk: %d\n", ret);
			return ret;
		}

		bt1120->rstc = of_reset_control_get(dev->of_node, NULL);
		if (IS_ERR(bt1120->rstc)) {
			ret = PTR_ERR(bt1120->rstc);
			dev_err(dev, "failed to get reset control: %d\n", ret);
			return ret;
		}
	}

	bt1120->base.funcs = &rk628_bt1120_bridge_funcs;
	bt1120->base.of_node = dev->of_node;
	ret = drm_bridge_add(&bt1120->base);
	if (ret) {
		dev_err(dev, "failed to add drm_bridge: %d\n", ret);
		return ret;
	}

	return 0;
}

static int rk628_bt1120_remove(struct platform_device *pdev)
{
	struct rk628_bt1120 *bt1120 = platform_get_drvdata(pdev);

	drm_bridge_remove(&bt1120->base);

	return 0;
}

static const struct of_device_id rk628_bt1120_of_match[] = {
	{ .compatible = "rockchip,rk628-bt1120", },
	{},
};
MODULE_DEVICE_TABLE(of, rk628_bt1120_of_match);

static struct platform_driver rk628_bt1120_driver = {
	.driver = {
		.name = "rk628-bt1120",
		.of_match_table = of_match_ptr(rk628_bt1120_of_match),
	},
	.probe = rk628_bt1120_probe,
	.remove = rk628_bt1120_remove,
};
module_platform_driver(rk628_bt1120_driver);

MODULE_AUTHOR("Guochun Huang <hero.huang@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip RK628 bt1120 driver");
MODULE_LICENSE("GPL v2");
