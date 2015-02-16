/*
* Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
* Author:
*      Andy yan <andy.yan@rock-chips.com>
*      Jeff chen <jeff.chen@rock-chips.com>
*
* based on exynos_dp_core.c
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
*/

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_of.h>
#include <drm/drm_dp_helper.h>

#include <linux/component.h>
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <video/of_videomode.h>
#include <video/videomode.h>

#include "rockchip_drm_vop.h"
#include "rockchip_edp_reg.h"
#include "rockchip_edp_core.h"

#define connector_to_edp(c) \
		container_of(c, struct rockchip_edp_device, connector)

#define encoder_to_edp(c) \
		container_of(c, struct rockchip_edp_device, encoder)

static char *voltage_names[] = {
	"0.4V", "0.6V", "0.8V", "1.2V"
};
static char *pre_emph_names[] = {
	"0dB", "3.5dB", "6dB", "9.5dB"
};

/* edp grf register offset */
#define EDP_VOP_SEL	0x025c /* grf_soc_con6 */
#define EDP_REF_CLK_SEL	0x0274 /* grf_soc_con12 */

#define GRF_EDP_REF_CLK_SEL_INTER		(1 << 4)
#define EDP_SEL_VOP_LIT				(1 << 5)

/***** general DP utility functions *****/
#define DP_VOLTAGE_MAX         DP_TRAIN_VOLTAGE_SWING_1200
#define DP_PRE_EMPHASIS_MAX    DP_TRAIN_PRE_EMPHASIS_9_5

/* edp hpd detect time */
#define EDP_HPD_TIMEOUT_MS	250

static const struct of_device_id rockchip_edp_dt_ids[] = {
	{.compatible = "rockchip,rk3288-edp",},
	{}
};

MODULE_DEVICE_TABLE(of, rockchip_edp_dt_ids);

static int rockchip_edp_clk_enable(struct rockchip_edp_device *edp)
{
	int ret = 0;

	ret = clk_prepare_enable(edp->pclk);
	if (ret < 0) {
		dev_err(edp->dev, "cannot enable edp pclk %d\n", ret);
		goto err_pclk;
	}

	ret = clk_prepare_enable(edp->clk_edp);
	if (ret < 0) {
		dev_err(edp->dev, "cannot enable clk_edp %d\n", ret);
		goto err_clk_edp;
	}

	ret = clk_set_rate(edp->clk_24m, 24000000);
	if (ret < 0) {
		dev_err(edp->dev, "cannot set edp clk_24m %d\n",
			ret);
		goto err_clk_24m;
	}

	ret = clk_prepare_enable(edp->clk_24m);
	if (ret < 0) {
		dev_err(edp->dev, "cannot enable edp clk_24m %d\n",
			ret);
		goto err_clk_24m;
	}

	return 0;

err_clk_24m:
	clk_disable_unprepare(edp->clk_edp);
err_clk_edp:
	clk_disable_unprepare(edp->pclk);
err_pclk:
	return ret;
}

static int rockchip_edp_clk_disable(struct rockchip_edp_device *edp)
{
	clk_disable_unprepare(edp->pclk);
	clk_disable_unprepare(edp->clk_edp);
	clk_disable_unprepare(edp->clk_24m);
	return 0;
}

static int rockchip_edp_pre_init(struct rockchip_edp_device *edp)
{
	u32 val;
	int ret;

	val = GRF_EDP_REF_CLK_SEL_INTER | (GRF_EDP_REF_CLK_SEL_INTER << 16);
	ret = regmap_write(edp->grf, EDP_REF_CLK_SEL, val);
	if (ret != 0) {
		dev_err(edp->dev, "Could not write to GRF: %d\n", ret);
		return ret;
	}

	reset_control_assert(edp->rst);
	usleep_range(10, 20);
	reset_control_deassert(edp->rst);

	return 0;
}

static void rockchip_edp_lt_init(struct rockchip_edp_device *edp)
{
	rockchip_edp_reset(edp);
	rockchip_edp_init_refclk(edp);
	rockchip_edp_init_interrupt(edp);
	rockchip_edp_enable_sw_function(edp);
	rockchip_edp_init_analog_func(edp);
	rockchip_edp_init_aux(edp);
}

static ssize_t rockchip_dpaux_transfer(struct drm_dp_aux *aux,
				    struct drm_dp_aux_msg *msg)

{
	int retval;

	retval = rockchip_edp_transfer(aux, msg);
	return retval;
}

static int rockchip_edp_config_video(struct rockchip_edp_device *edp,
				     struct video_info *video_info)
{
	int retval = 0;
	int done_count = 0;
	unsigned long timeout = 0;

	rockchip_edp_config_video_slave_mode(edp, video_info);

	rockchip_edp_set_video_color_format(edp, video_info->color_depth,
					    video_info->color_space,
					    video_info->dynamic_range,
					    video_info->ycbcr_coeff);

	if (rockchip_edp_get_pll_lock_status(edp) == DP_PLL_UNLOCKED) {
		dev_err(edp->dev, "PLL is not locked yet.\n");
		return -EINVAL;
	}

	timeout = jiffies + usecs_to_jiffies(100);
	while (rockchip_edp_check_video_stream_clock_on(edp)) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
	}

	/* Set to use the register calculated M/N video */
	rockchip_edp_set_video_cr_mn(edp, CALCULATED_M, 0, 0);

	/* Disable video mute */
	rockchip_edp_enable_video_mute(edp, 0);

	/* Enable video */
	rockchip_edp_start_video(edp);

	timeout = jiffies + msecs_to_jiffies(100);
	while (time_before(jiffies, timeout) && (done_count < 11)) {
		if (rockchip_edp_is_video_stream_on(edp))
			done_count = 0;
		else
			done_count++;
		udelay(10);
	}
	if (done_count < 10)
		return -ETIMEDOUT;

	return retval;
}

static void dp_get_adjust_train(u8 link_status[DP_LINK_STATUS_SIZE],
				int lane_count,
				u8 train_set[4])
{
	u8 v = 0;
	u8 p = 0;
	int lane;

	for (lane = 0; lane < lane_count; lane++) {
		u8 this_v =
			drm_dp_get_adjust_request_voltage(link_status, lane);
		u8 this_p =
			drm_dp_get_adjust_request_pre_emphasis(link_status,
								lane);

		DRM_DEBUG_KMS("requested signal parameters: lane %d "
				"voltage %s pre_emph %s\n", lane,
			 voltage_names[this_v >> DP_TRAIN_VOLTAGE_SWING_SHIFT],
			 pre_emph_names[this_p >> DP_TRAIN_PRE_EMPHASIS_SHIFT]);

		if (this_v > v)
			v = this_v;
		if (this_p > p)
			p = this_p;
	}

	if (v >= DP_VOLTAGE_MAX)
		v |= DP_TRAIN_MAX_SWING_REACHED;

	if (p >= DP_PRE_EMPHASIS_MAX)
		p |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;

	DRM_DEBUG_KMS("using signal parameters: voltage %s pre_emph %s\n",
		  voltage_names[(v & DP_TRAIN_VOLTAGE_SWING_MASK)
		  >> DP_TRAIN_VOLTAGE_SWING_SHIFT],
		  pre_emph_names[(p & DP_TRAIN_PRE_EMPHASIS_MASK)
		  >> DP_TRAIN_PRE_EMPHASIS_SHIFT]);

	for (lane = 0; lane < 4; lane++)
		train_set[lane] = v | p;
}

static int rockchip_dp_link_train_cr(struct drm_dp_aux *aux,
			struct drm_dp_link *link)
{
	struct rockchip_edp_device *edp =
		container_of(aux, struct rockchip_edp_device, aux);
	bool clock_recovery;
	u8 voltage;
	u8 status[DP_LINK_STATUS_SIZE];
	int i;

	writel(DP_TRAINING_PATTERN_1, edp->regs + TRAINING_PTN_SET);
	drm_dp_dpcd_writeb(aux, DP_TRAINING_PATTERN_SET, DP_TRAINING_PATTERN_1);
	memset(edp->train_set, 0, 4);

	/* clock recovery loop */
	clock_recovery = false;
	edp->tries = 0;
	voltage = 0xff;
	while (1) {
		rockchip_edp_set_link_training(edp, edp->train_set);
		drm_dp_dpcd_write(aux, DP_TRAINING_LANE0_SET, edp->train_set,
				link->num_lanes);

		usleep_range(500, 1000);

		if (drm_dp_dpcd_read_link_status(aux, status) < 0) {
			DRM_ERROR("displayport link status failed\n");
			break;
		}

		if (drm_dp_clock_recovery_ok(status, link->num_lanes)) {
			clock_recovery = true;
			break;
		}

		for (i = 0; i < link->num_lanes; i++) {
			if ((edp->train_set[i] &
				DP_TRAIN_MAX_SWING_REACHED) == 0)
				break;
		}
		if (i == link->num_lanes) {
			DRM_ERROR("clock recovery reached max voltage\n");
			break;
		}

		if ((edp->train_set[0] &
			DP_TRAIN_VOLTAGE_SWING_MASK) == voltage) {
			++edp->tries;
			if (edp->tries == MAX_CR_LOOP) {
				DRM_ERROR("clock recovery tried 5 times\n");
				break;
			}
		} else
			edp->tries = 0;

		voltage = edp->train_set[0] & DP_TRAIN_VOLTAGE_SWING_MASK;

		/* Compute new train_set as requested by sink */
		dp_get_adjust_train(status, link->num_lanes, edp->train_set);
	}
	if (!clock_recovery) {
		DRM_ERROR("clock recovery failed\n");
		return -1;
	} else {
		DRM_DEBUG_KMS("clock recovery at voltage %d pre-emphasis %d\n",
			  edp->train_set[0] & DP_TRAIN_VOLTAGE_SWING_MASK,
			  (edp->train_set[0] & DP_TRAIN_PRE_EMPHASIS_MASK) >>
			  DP_TRAIN_PRE_EMPHASIS_SHIFT);
		return 0;
	}
}

static int rockchip_dp_link_train_ce(struct drm_dp_aux *aux,
			struct drm_dp_link *link)
{
	struct rockchip_edp_device *edp =
		container_of(aux, struct rockchip_edp_device, aux);
	bool channel_eq;
	u8 status[DP_LINK_STATUS_SIZE];

	writel(DP_TRAINING_PATTERN_2, edp->regs + TRAINING_PTN_SET);
	drm_dp_dpcd_writeb(aux, DP_TRAINING_PATTERN_SET, DP_TRAINING_PATTERN_2);

	/* channel equalization loop */
	edp->tries = 0;
	channel_eq = false;
	while (1) {
		usleep_range(500, 1000);

		if (drm_dp_dpcd_read_link_status(aux, status) < 0) {
			DRM_ERROR("displayport link status failed\n");
			break;
		}

		if (drm_dp_channel_eq_ok(status, link->num_lanes)) {
			channel_eq = true;
			break;
		}
		/* Try 5 times */
		if (edp->tries > MAX_EQ_LOOP) {
			DRM_ERROR("channel eq failed: 5 tries\n");
			break;
		}

		/* Compute new train_set as requested by sink */
		dp_get_adjust_train(status, link->num_lanes, edp->train_set);
		rockchip_edp_set_link_training(edp, edp->train_set);
		edp->tries++;
	}

	if (!channel_eq) {
		DRM_ERROR("channel eq failed\n");
		return -1;
	} else {
		DRM_DEBUG_KMS("channel eq at voltage %d pre-emphasis %d\n",
			  edp->train_set[0] & DP_TRAIN_VOLTAGE_SWING_MASK,
			  (edp->train_set[0] & DP_TRAIN_PRE_EMPHASIS_MASK)
			  >> DP_TRAIN_PRE_EMPHASIS_SHIFT);
		return 0;
	}
}

static int rockchip_edp_commit(struct drm_encoder *encoder)
{
	struct rockchip_edp_device *edp = encoder_to_edp(encoder);
	unsigned long timeout = 0;
	u32 val;
	int ret;

	ret = drm_dp_link_probe(&edp->aux, &edp->link);
	if (ret < 0) {
		dev_err(edp->dev, "failed to probe eDP link: %d\n",
			ret);
		return ret;
	}

	ret = drm_dp_link_power_up(&edp->aux, &edp->link);
	if (ret < 0) {
		dev_err(edp->dev, "failed to power up eDP link: %d\n",
			ret);
		return ret;
	}

	ret = drm_dp_link_configure(&edp->aux, &edp->link);
	if (ret < 0) {
		dev_err(edp->dev, "failed to configure eDP link: %d\n",
			ret);
		return ret;
	}

	val = drm_dp_link_rate_to_bw_code(edp->link.rate);
	rockchip_edp_set_link_bandwidth(edp, val);
	rockchip_edp_set_lane_count(edp, edp->link.num_lanes);

	rockchip_dp_link_train_cr(&edp->aux, &edp->link);
	rockchip_dp_link_train_ce(&edp->aux, &edp->link);
	rockchip_edp_hw_link_training_en(edp);
	timeout = jiffies + msecs_to_jiffies(50);
	while (rockchip_edp_wait_hw_lt_done(edp)) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
	}
	ret = rockchip_edp_get_hw_lt_status(edp);
	if (ret)
		dev_err(edp->dev, "hw lt err:%d\n", val);

	rockchip_edp_init_video(edp);
	ret = rockchip_edp_config_video(edp, &edp->video_info);
	if (ret)
		dev_err(edp->dev, "unable to config video\n");
	return ret;
}

/*
 * Wait up to EDP_HPD_TIMEOUT_MS for eDP HPD signal.
 * On boards where HPD is not hooked up, or for panels whose HPD voltage is too
 * low to be detected (< 3.3 * 0.7 = 2.31 V), there is no way to detect hotplug
 * so we will timeout.
 * In this case, we force the eDP HPD to enable the AUX channel.
 * Note: At present, there is no way to reliably detect that a panel is not
 * present.
 */
static void rockchip_edp_wait_hpd(struct rockchip_edp_device *edp)
{
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(EDP_HPD_TIMEOUT_MS);
	do {
		if (rockchip_edp_get_plug_in_status(edp))
			return;
		usleep_range(10000, 20000);
	} while (!time_after(jiffies, timeout));

	dev_dbg(edp->dev, "Timed out waiting for eDP HPD, forcing...\n");

	rockchip_edp_force_hpd(edp);

	if (!rockchip_edp_get_plug_in_status(edp))
		dev_warn(edp->dev, "eDP HPD forced but still not detected!\n");
}

static void rockchip_edp_poweron(struct drm_encoder *encoder)
{
	struct rockchip_edp_device *edp = encoder_to_edp(encoder);
	int ret;

	if (edp->dpms_mode == DRM_MODE_DPMS_ON)
		return;

	if (edp->panel)
		drm_panel_prepare(edp->panel);

	ret = rockchip_edp_clk_enable(edp);
	if (ret < 0) {
		dev_err(edp->dev, "cannot enable edp clk %d\n", ret);
		return;
	}

	ret = rockchip_edp_pre_init(edp);
	if (ret < 0) {
		dev_err(edp->dev, "edp pre init fail %d\n", ret);
		return;
	}

	rockchip_edp_wait_hpd(edp);
	rockchip_edp_lt_init(edp);
	rockchip_edp_commit(encoder);

	if (edp->panel)
		drm_panel_enable(edp->panel);
}

static void rockchip_edp_poweroff(struct drm_encoder *encoder)
{
	struct rockchip_edp_device *edp = encoder_to_edp(encoder);

	if (edp->dpms_mode == DRM_MODE_DPMS_OFF)
		return;

	if (edp->panel)
		drm_panel_disable(edp->panel);

	rockchip_edp_reset(edp);
	rockchip_edp_analog_power_ctr(edp, 0);
	rockchip_edp_clk_disable(edp);

	if (edp->panel)
		drm_panel_unprepare(edp->panel);
}

static enum drm_connector_status
rockchip_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void rockchip_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs rockchip_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = rockchip_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = rockchip_connector_destroy,
};

static int rockchip_connector_get_modes(struct drm_connector *connector)
{
	struct rockchip_edp_device *edp = connector_to_edp(connector);
	struct drm_panel *panel = edp->panel;

	return panel->funcs->get_modes(panel);
}

static struct drm_encoder *
	rockchip_connector_best_encoder(struct drm_connector *connector)
{
	struct rockchip_edp_device *edp = connector_to_edp(connector);

	return &edp->encoder;
}

static struct drm_connector_helper_funcs rockchip_connector_helper_funcs = {
	.get_modes = rockchip_connector_get_modes,
	.best_encoder = rockchip_connector_best_encoder,
};

static void rockchip_drm_encoder_dpms(struct drm_encoder *encoder,
								int mode)
{
	struct rockchip_edp_device *edp = encoder_to_edp(encoder);

	if (edp->dpms_mode == mode)
		return;

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		rockchip_edp_poweron(encoder);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		rockchip_edp_poweroff(encoder);
		break;
	default:
		break;
	}

	edp->dpms_mode = mode;
}

static bool
rockchip_drm_encoder_mode_fixup(struct drm_encoder *encoder,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void rockchip_drm_encoder_mode_set(struct drm_encoder *encoder,
					  struct drm_display_mode *mode,
					  struct drm_display_mode *adjusted)
{
	struct rockchip_edp_device *edp = encoder_to_edp(encoder);

	memcpy(&edp->mode, adjusted, sizeof(*mode));
}

static void rockchip_drm_encoder_prepare(struct drm_encoder *encoder)
{
	struct rockchip_edp_device *edp = encoder_to_edp(encoder);
	u32 val;
	int ret;

	ret = rockchip_drm_crtc_mode_config(encoder->crtc,
					    edp->connector.connector_type,
					    ROCKCHIP_OUT_MODE_AAAA);
	if (ret < 0) {
		dev_err(edp->dev, "Could not set crtc mode config: %d.\n", ret);
		return;
	}

	ret = rockchip_drm_encoder_get_mux_id(edp->dev->of_node, encoder);
	if (ret < 0)
		return;

	if (ret)
		val = EDP_SEL_VOP_LIT | (EDP_SEL_VOP_LIT << 16);
	else
		val = EDP_SEL_VOP_LIT << 16;

	dev_info(edp->dev, "vop %s output to edp\n",
		 (ret) ? "LIT" : "BIG");

	ret = regmap_write(edp->grf, EDP_VOP_SEL, val);
	if (ret != 0) {
		dev_err(edp->dev, "Could not write to GRF: %d\n", ret);
		return;
	}
}

static void rockchip_drm_encoder_commit(struct drm_encoder *encoder)
{
	rockchip_drm_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void rockchip_drm_encoder_disable(struct drm_encoder *encoder)
{
	struct drm_plane *plane;
	struct drm_device *dev = encoder->dev;

	rockchip_drm_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);

	/* all planes connected to this encoder should be also disabled. */
	list_for_each_entry(plane, &dev->mode_config.plane_list, head) {
		if (plane->crtc && (plane->crtc == encoder->crtc))
			plane->funcs->disable_plane(plane);
	}
}

static struct drm_encoder_helper_funcs rockchip_encoder_helper_funcs = {
	.dpms = rockchip_drm_encoder_dpms,
	.mode_fixup = rockchip_drm_encoder_mode_fixup,
	.mode_set = rockchip_drm_encoder_mode_set,
	.prepare = rockchip_drm_encoder_prepare,
	.commit = rockchip_drm_encoder_commit,
	.disable = rockchip_drm_encoder_disable,
};

static void rockchip_drm_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static struct drm_encoder_funcs rockchip_encoder_funcs = {
	.destroy = rockchip_drm_encoder_destroy,
};

static int rockchip_edp_init(struct rockchip_edp_device *edp)
{
	struct device *dev = edp->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *res;
	int ret;

	edp->grf = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(edp->grf)) {
		dev_err(dev,
			"rk3288-edp needs rockchip,grf property\n");
		return PTR_ERR(edp->grf);
	}

	edp->video_info.h_sync_polarity = 0;
	edp->video_info.v_sync_polarity = 0;
	edp->video_info.interlaced = 0;
	edp->video_info.color_space = CS_RGB;
	edp->video_info.dynamic_range = VESA;
	edp->video_info.ycbcr_coeff = COLOR_YCBCR601;
	edp->video_info.color_depth = COLOR_8;

	edp->video_info.link_rate = DP_LINK_BW_1_62;
	edp->video_info.lane_count = LANE_CNT4;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	edp->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(edp->regs)) {
		dev_err(dev, "ioremap reg failed\n");
		return PTR_ERR(edp->regs);
	}

	edp->clk_edp = devm_clk_get(dev, "clk_edp");
	if (IS_ERR(edp->clk_edp)) {
		dev_err(dev, "cannot get clk_edp\n");
		return PTR_ERR(edp->clk_edp);
	}

	edp->clk_24m = devm_clk_get(dev, "clk_edp_24m");
	if (IS_ERR(edp->clk_24m)) {
		dev_err(dev, "cannot get clk_edp_24m\n");
		return PTR_ERR(edp->clk_24m);
	}

	edp->pclk = devm_clk_get(dev, "pclk_edp");
	if (IS_ERR(edp->pclk)) {
		dev_err(dev, "cannot get pclk\n");
		return PTR_ERR(edp->pclk);
	}

	edp->rst = devm_reset_control_get(dev, "edp");
	if (IS_ERR(edp->rst)) {
		dev_err(dev, "failed to get reset\n");
		return PTR_ERR(edp->rst);
	}

	ret = rockchip_edp_clk_enable(edp);
	if (ret < 0) {
		dev_err(edp->dev, "cannot enable edp clk %d\n", ret);
		return ret;
	}

	ret = rockchip_edp_pre_init(edp);
	if (ret < 0) {
		dev_err(edp->dev, "failed to pre init %d\n", ret);
		return ret;
	}

	edp->dpms_mode = DRM_MODE_DPMS_OFF;

	dev_set_name(edp->dev, "rockchip-edp");

	return 0;
}

static int rockchip_edp_bind(struct device *dev, struct device *master,
			     void *data)
{
	struct rockchip_edp_device *edp = dev_get_drvdata(dev);
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct drm_device *drm_dev = data;
	int ret;

	ret = rockchip_edp_init(edp);
	if (ret < 0)
		return ret;

	edp->drm_dev = drm_dev;

	encoder = &edp->encoder;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm_dev,
							     dev->of_node);
	DRM_DEBUG_KMS("possible_crtcs = 0x%x\n", encoder->possible_crtcs);

	ret = drm_encoder_init(drm_dev, encoder, &rockchip_encoder_funcs,
			       DRM_MODE_ENCODER_TMDS);
	if (ret) {
		DRM_ERROR("failed to initialize encoder with drm\n");
		return ret;
	}

	drm_encoder_helper_add(encoder, &rockchip_encoder_helper_funcs);

	connector = &edp->connector;
	connector->polled = DRM_CONNECTOR_POLL_HPD;
	connector->dpms = DRM_MODE_DPMS_OFF;

	ret = drm_connector_init(drm_dev, connector,
				 &rockchip_connector_funcs,
				 DRM_MODE_CONNECTOR_eDP);
	if (ret) {
		DRM_ERROR("failed to initialize connector with drm\n");
		goto err_free_encoder;
	}

	drm_connector_helper_add(connector,
				 &rockchip_connector_helper_funcs);

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret) {
		DRM_ERROR("failed to attach connector and encoder\n");
		goto err_free_connector;
	}

	ret = drm_panel_attach(edp->panel, connector);
	if (ret) {
		DRM_ERROR("failed to attach connector and encoder\n");
		goto err_free_connector;
	}

	ret = drm_dp_aux_register_i2c_bus(&edp->aux);
	if (ret) {
		DRM_ERROR("failed to register i2c\n");
		goto err_panel_detach;
	}

	return 0;

err_panel_detach:
	drm_panel_detach(edp->panel);
err_free_connector:
	drm_connector_cleanup(connector);
err_free_encoder:
	drm_encoder_cleanup(encoder);
	return ret;
}

static void rockchip_edp_unbind(struct device *dev, struct device *master,
				void *data)
{
	struct rockchip_edp_device *edp = dev_get_drvdata(dev);
	struct drm_encoder *encoder;

	encoder = &edp->encoder;

	if (edp->panel)
		drm_panel_detach(edp->panel);

	rockchip_drm_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
	encoder->funcs->destroy(encoder);
	drm_connector_unregister(&edp->connector);
	drm_connector_cleanup(&edp->connector);
	drm_encoder_cleanup(encoder);
}

static const struct component_ops rockchip_edp_component_ops = {
	.bind = rockchip_edp_bind,
	.unbind = rockchip_edp_unbind,
};

static int rockchip_edp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_panel *panel;
	struct device_node *panel_node;
	struct rockchip_edp_device *edp;

	if (!dev->of_node) {
		dev_err(dev, "can't find eDP devices\n");
		return -ENODEV;
	}

	panel_node = of_parse_phandle(dev->of_node, "rockchip,panel", 0);
	if (!panel_node) {
		DRM_ERROR("failed to find rockchip,panel dt node\n");
		return -ENODEV;
	}

	panel = of_drm_find_panel(panel_node);
	if (!panel) {
		DRM_ERROR("failed to find panel\n");
		of_node_put(panel_node);
		return -EPROBE_DEFER;
	}

	of_node_put(panel_node);

	edp = devm_kzalloc(dev, sizeof(*edp), GFP_KERNEL);
	if (!edp)
		return -ENOMEM;
	edp->dev = dev;
	edp->panel = panel;
	edp->aux.transfer = rockchip_dpaux_transfer;
	edp->aux.dev = dev;

	platform_set_drvdata(pdev, edp);

	return component_add(dev, &rockchip_edp_component_ops);
}

static int rockchip_edp_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &rockchip_edp_component_ops);

	return 0;
}

static struct platform_driver rockchip_edp_driver = {
	.probe = rockchip_edp_probe,
	.remove = rockchip_edp_remove,
	.driver = {
		   .name = "rockchip-edp",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(rockchip_edp_dt_ids),
	},
};

module_platform_driver(rockchip_edp_driver);

MODULE_AUTHOR("Jeff chen <jeff.chen@rock-chips.com>");
MODULE_DESCRIPTION("ROCKCHIP EDP Driver");
MODULE_LICENSE("GPL v2");
