// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:
 *      Sandy Huang <hjc@rock-chips.com>
 */

#include <linux/component.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/phy/phy.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio/consumer.h>

#include <video/of_display_timing.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include <uapi/linux/videodev2.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_vop.h"

#define HIWORD_UPDATE(v, l, h)	(((v) << (l)) | (GENMASK(h, l) << 16))

#define PX30_GRF_PD_VO_CON1		0x0438
#define PX30_RGB_DATA_SYNC_BYPASS(v)	HIWORD_UPDATE(v, 3, 3)
#define PX30_RGB_VOP_SEL(v)		HIWORD_UPDATE(v, 2, 2)

#define RK1808_GRF_PD_VO_CON1		0x0444
#define RK1808_RGB_DATA_SYNC_BYPASS(v)	HIWORD_UPDATE(v, 3, 3)

#define RV1106_VENC_GRF_VOP_IO_WRAPPER	0x1000c
#define RV1106_IO_BYPASS_SEL(v)		HIWORD_UPDATE(v, 0, 1)
#define RV1106_VOGRF_VOP_PIPE_BYPASS	0x60034
#define RV1106_VOP_PIPE_BYPASS(v)	HIWORD_UPDATE(v, 0, 1)

#define RV1126_GRF_IOFUNC_CON3		0x1026c
#define RV1126_LCDC_IO_BYPASS(v)	HIWORD_UPDATE(v, 0, 0)

#define RK3288_GRF_SOC_CON6		0x025c
#define RK3288_LVDS_LCDC_SEL(x)		HIWORD_UPDATE(x,  3,  3)
#define RK3288_GRF_SOC_CON7		0x0260
#define RK3288_LVDS_PWRDWN(x)		HIWORD_UPDATE(x, 15, 15)
#define RK3288_LVDS_CON_ENABLE_2(x)	HIWORD_UPDATE(x, 12, 12)
#define RK3288_LVDS_CON_ENABLE_1(x)	HIWORD_UPDATE(x, 11, 11)
#define RK3288_LVDS_CON_CLKINV(x)	HIWORD_UPDATE(x,  8,  8)
#define RK3288_LVDS_CON_TTL_EN(x)	HIWORD_UPDATE(x,  6,  6)

#define RK3562_GRF_IOC_VO_IO_CON	0x10500
#define RK3562_RGB_DATA_BYPASS(v)	HIWORD_UPDATE(v, 6, 6)

#define RK3568_GRF_VO_CON1		0X0364
#define RK3568_RGB_DATA_BYPASS(v)	HIWORD_UPDATE(v, 6, 6)

struct rockchip_rgb;

struct rockchip_rgb_funcs {
	void (*enable)(struct rockchip_rgb *rgb);
	void (*disable)(struct rockchip_rgb *rgb);
};

struct rockchip_rgb_data {
	u32 rgb_max_dclk_rate;
	u32 mcu_max_dclk_rate;
	const struct rockchip_rgb_funcs *funcs;
};

struct mcu_cmd_header {
	u8 data_type;
	u8 delay;
	u8 payload_length;
} __packed;

struct mcu_cmd_desc {
	struct mcu_cmd_header header;
	u8 *payload;
};

struct mcu_cmd_seq {
	struct mcu_cmd_desc *cmds;
	unsigned int cmd_cnt;
};

struct rockchip_mcu_panel_desc {
	struct drm_display_mode *mode;
	struct mcu_cmd_seq *init_seq;
	struct mcu_cmd_seq *exit_seq;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
		unsigned int reset;
		unsigned int init;
	} delay;

	unsigned int bpc;
	u32 bus_format;
	u32 bus_flags;
};

struct rockchip_mcu_panel {
	struct drm_panel base;
	struct drm_device *drm_dev;
	struct rockchip_mcu_panel_desc *desc;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;

	struct device_node *np_crtc;

	bool prepared;
	bool enabled;
};

struct rockchip_rgb {
	u8 id;
	u32 max_dclk_rate;
	u32 mcu_pix_total;
	struct device *dev;
	struct device_node *np_mcu_panel;
	struct drm_panel *panel;
	struct drm_bridge *bridge;
	struct drm_connector connector;
	struct drm_encoder encoder;
	struct phy *phy;
	struct regmap *grf;
	bool data_sync_bypass;
	bool phy_enabled;
	const struct rockchip_rgb_funcs *funcs;
	struct rockchip_drm_sub_dev sub_dev;
};

static inline struct rockchip_rgb *connector_to_rgb(struct drm_connector *c)
{
	return container_of(c, struct rockchip_rgb, connector);
}

static inline struct rockchip_rgb *encoder_to_rgb(struct drm_encoder *e)
{
	return container_of(e, struct rockchip_rgb, encoder);
}

static inline struct rockchip_mcu_panel *to_rockchip_mcu_panel(struct drm_panel *panel)
{
	return container_of(panel, struct rockchip_mcu_panel, base);
}

static enum drm_connector_status
rockchip_rgb_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static int
rockchip_rgb_atomic_connector_get_property(struct drm_connector *connector,
					   const struct drm_connector_state *state,
					   struct drm_property *property,
					   uint64_t *val)
{
	struct rockchip_rgb *rgb = connector_to_rgb(connector);
	struct rockchip_drm_private *private = connector->dev->dev_private;

	if (property == private->connector_id_prop) {
		*val = rgb->id;
		return 0;
	}

	DRM_ERROR("failed to get rockchip RGB property\n");
	return -EINVAL;
}

static const struct drm_connector_funcs rockchip_rgb_connector_funcs = {
	.detect = rockchip_rgb_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.atomic_get_property = rockchip_rgb_atomic_connector_get_property,
};

static int rockchip_rgb_connector_get_modes(struct drm_connector *connector)
{
	struct rockchip_rgb *rgb = connector_to_rgb(connector);
	struct drm_panel *panel = rgb->panel;

	return drm_panel_get_modes(panel, connector);
}

static struct drm_encoder *
rockchip_rgb_connector_best_encoder(struct drm_connector *connector)
{
	struct rockchip_rgb *rgb = connector_to_rgb(connector);

	return &rgb->encoder;
}

static const
struct drm_connector_helper_funcs rockchip_rgb_connector_helper_funcs = {
	.get_modes = rockchip_rgb_connector_get_modes,
	.best_encoder = rockchip_rgb_connector_best_encoder,
};

static void rockchip_rgb_encoder_enable(struct drm_encoder *encoder)
{
	struct rockchip_rgb *rgb = encoder_to_rgb(encoder);

	pinctrl_pm_select_default_state(rgb->dev);

	if (rgb->funcs && rgb->funcs->enable)
		rgb->funcs->enable(rgb);

	if (rgb->phy && !rgb->phy_enabled) {
		phy_power_on(rgb->phy);
		rgb->phy_enabled = true;
	}

	if (rgb->panel) {
		drm_panel_prepare(rgb->panel);
		drm_panel_enable(rgb->panel);
	}
}

static void rockchip_rgb_encoder_disable(struct drm_encoder *encoder)
{
	struct rockchip_rgb *rgb = encoder_to_rgb(encoder);

	if (rgb->panel) {
		drm_panel_disable(rgb->panel);
		drm_panel_unprepare(rgb->panel);
	}

	if (rgb->phy && rgb->phy_enabled) {
		phy_power_off(rgb->phy);
		rgb->phy_enabled = false;
	}

	if (rgb->funcs && rgb->funcs->disable)
		rgb->funcs->disable(rgb);

	pinctrl_pm_select_sleep_state(rgb->dev);
}

static int
rockchip_rgb_encoder_atomic_check(struct drm_encoder *encoder,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	struct rockchip_crtc_state *s = to_rockchip_crtc_state(crtc_state);
	struct drm_connector *connector = conn_state->connector;
	struct drm_display_info *info = &connector->display_info;

	if (info->num_bus_formats)
		s->bus_format = info->bus_formats[0];
	else
		s->bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	switch (s->bus_format) {
	case MEDIA_BUS_FMT_RGB666_1X18:
		s->output_mode = ROCKCHIP_OUT_MODE_P666;
		s->output_if = VOP_OUTPUT_IF_RGB;
		break;
	case MEDIA_BUS_FMT_RGB565_1X16:
		s->output_mode = ROCKCHIP_OUT_MODE_P565;
		s->output_if = VOP_OUTPUT_IF_RGB;
		break;
	case MEDIA_BUS_FMT_RGB565_2X8_LE:
	case MEDIA_BUS_FMT_BGR565_2X8_LE:
		s->output_mode = ROCKCHIP_OUT_MODE_S565;
		s->output_if = VOP_OUTPUT_IF_RGB;
		break;
	case MEDIA_BUS_FMT_RGB666_3X6:
		s->output_mode = ROCKCHIP_OUT_MODE_S666;
		s->output_if = VOP_OUTPUT_IF_RGB;
		break;
	case MEDIA_BUS_FMT_RGB888_3X8:
	case MEDIA_BUS_FMT_BGR888_3X8:
		s->output_mode = ROCKCHIP_OUT_MODE_S888;
		s->output_if = VOP_OUTPUT_IF_RGB;
		break;
	case MEDIA_BUS_FMT_RGB888_DUMMY_4X8:
	case MEDIA_BUS_FMT_BGR888_DUMMY_4X8:
		s->output_mode = ROCKCHIP_OUT_MODE_S888_DUMMY;
		s->output_if = VOP_OUTPUT_IF_RGB;
		break;
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_YVYU8_2X8:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		s->output_mode = ROCKCHIP_OUT_MODE_BT656;
		s->output_if = VOP_OUTPUT_IF_BT656;
		break;
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YVYU8_1X16:
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_1X16:
		s->output_mode = ROCKCHIP_OUT_MODE_BT1120;
		s->output_if = VOP_OUTPUT_IF_BT1120;
		break;
	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_RGB666_1X24_CPADHI:
	default:
		s->output_mode = ROCKCHIP_OUT_MODE_P888;
		s->output_if = VOP_OUTPUT_IF_RGB;
		break;
	}

	s->output_type = DRM_MODE_CONNECTOR_DPI;
	s->bus_flags = info->bus_flags;
	s->tv_state = &conn_state->tv;
	s->eotf = HDMI_EOTF_TRADITIONAL_GAMMA_SDR;
	s->color_space = V4L2_COLORSPACE_DEFAULT;

	return 0;
}

static int rockchip_rgb_encoder_loader_protect(struct drm_encoder *encoder,
					       bool on)
{
	struct rockchip_rgb *rgb = encoder_to_rgb(encoder);

	if (rgb->np_mcu_panel) {
		struct rockchip_mcu_panel *mcu_panel = to_rockchip_mcu_panel(rgb->panel);

		mcu_panel->prepared = true;
		mcu_panel->enabled = true;

		return 0;
	}

	if (rgb->panel)
		panel_simple_loader_protect(rgb->panel);

	if (on) {
		phy_init(rgb->phy);
		if (rgb->phy) {
			rgb->phy->power_count++;
			rgb->phy_enabled = true;
		}
	} else {
		phy_exit(rgb->phy);
		if (rgb->phy) {
			rgb->phy->power_count--;
			rgb->phy_enabled = false;
		}
	}

	return 0;
}

static enum drm_mode_status
rockchip_rgb_encoder_mode_valid(struct drm_encoder *encoder,
				 const struct drm_display_mode *mode)
{
	struct rockchip_rgb *rgb = encoder_to_rgb(encoder);
	struct device *dev = rgb->dev;
	struct drm_display_info *info = &rgb->connector.display_info;
	u32 request_clock = mode->clock;
	u32 max_clock = rgb->max_dclk_rate;
	u32 bus_format;

	if (info->num_bus_formats)
		bus_format = info->bus_formats[0];
	else
		bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	if (mode->flags & DRM_MODE_FLAG_DBLCLK)
		request_clock *= 2;

	if (rgb->np_mcu_panel)
		request_clock *= rockchip_drm_get_cycles_per_pixel(bus_format) *
				 (rgb->mcu_pix_total + 1);

	if (max_clock != 0 && request_clock > max_clock) {
		DRM_DEV_ERROR(dev, "mode [%dx%d] clock %d is higher than max_clock %d\n",
			      mode->hdisplay, mode->vdisplay, request_clock, max_clock);
		return MODE_CLOCK_HIGH;
	}

	return MODE_OK;
}

static const
struct drm_encoder_helper_funcs rockchip_rgb_encoder_helper_funcs = {
	.enable = rockchip_rgb_encoder_enable,
	.disable = rockchip_rgb_encoder_disable,
	.atomic_check = rockchip_rgb_encoder_atomic_check,
	.mode_valid = rockchip_rgb_encoder_mode_valid,
};

static const struct drm_encoder_funcs rockchip_rgb_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int rockchip_mcu_panel_parse_cmd_seq(struct device *dev,
					    const u8 *data, int length,
					    struct mcu_cmd_seq *seq)
{
	struct mcu_cmd_header *header;
	struct mcu_cmd_desc *desc;
	char *buf, *d;
	unsigned int i, cnt, len;

	if (!seq)
		return -EINVAL;

	buf = devm_kmemdup(dev, data, length, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	d = buf;
	len = length;
	cnt = 0;
	while (len > sizeof(*header)) {
		header = (struct mcu_cmd_header *)d;

		d += sizeof(*header);
		len -= sizeof(*header);

		if (header->payload_length > len)
			return -EINVAL;

		d += header->payload_length;
		len -= header->payload_length;
		cnt++;
	}

	if (len)
		return -EINVAL;

	seq->cmd_cnt = cnt;
	seq->cmds = devm_kcalloc(dev, cnt, sizeof(*desc), GFP_KERNEL);
	if (!seq->cmds)
		return -ENOMEM;

	d = buf;
	len = length;
	for (i = 0; i < cnt; i++) {
		header = (struct mcu_cmd_header *)d;
		len -= sizeof(*header);
		d += sizeof(*header);

		desc = &seq->cmds[i];
		desc->header = *header;
		desc->payload = d;

		d += header->payload_length;
		len -= header->payload_length;
	}

	return 0;
}

static int rockchip_mcu_panel_init(struct rockchip_rgb *rgb)
{
	struct device *dev = rgb->dev;
	struct device_node *np_mcu_panel = rgb->np_mcu_panel;
	struct device_node *port, *endpoint, *np_crtc, *remote, *np_mcu_timing;
	struct rockchip_mcu_panel *mcu_panel = to_rockchip_mcu_panel(rgb->panel);
	struct drm_display_mode *mode;
	const void *data;
	int len;
	int ret;
	u32 bus_flags;
	u32 val;

	mcu_panel->enable_gpio = devm_fwnode_gpiod_get_index(dev, &np_mcu_panel->fwnode,
							     "enable", 0, GPIOD_ASIS,
							     fwnode_get_name(&np_mcu_panel->fwnode));
	if (IS_ERR(mcu_panel->enable_gpio)) {
		DRM_DEV_ERROR(dev, "failed to find mcu panel enable GPIO\n");
		return PTR_ERR(mcu_panel->enable_gpio);
	}

	mcu_panel->reset_gpio = devm_fwnode_gpiod_get_index(dev, &np_mcu_panel->fwnode,
							    "reset", 0, GPIOD_ASIS,
							    fwnode_get_name(&np_mcu_panel->fwnode));
	if (IS_ERR(mcu_panel->reset_gpio)) {
		DRM_DEV_ERROR(dev, "failed to find mcu panel reset GPIO\n");
		return PTR_ERR(mcu_panel->reset_gpio);
	}

	mcu_panel->desc = devm_kzalloc(dev, sizeof(*mcu_panel->desc), GFP_KERNEL);
	if (!mcu_panel->desc)
		return -ENOMEM;

	mode = devm_kzalloc(dev, sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return -ENOMEM;

	if (!of_get_drm_display_mode(np_mcu_panel, mode, &bus_flags,
				     OF_USE_NATIVE_MODE)) {
		mcu_panel->desc->mode = mode;
		mcu_panel->desc->bus_flags = bus_flags;
	} else {
		DRM_DEV_ERROR(dev, "failed to parse display mode\n");
		return -EINVAL;
	}

	of_property_read_u32(np_mcu_panel, "bpc", &mcu_panel->desc->bpc);
	of_property_read_u32(np_mcu_panel, "bus-format", &mcu_panel->desc->bus_format);
	of_property_read_u32(np_mcu_panel, "width-mm", &mcu_panel->desc->size.width);
	of_property_read_u32(np_mcu_panel, "height-mm", &mcu_panel->desc->size.height);

	of_property_read_u32(np_mcu_panel, "prepare-delay-ms", &mcu_panel->desc->delay.prepare);
	of_property_read_u32(np_mcu_panel, "enable-delay-ms", &mcu_panel->desc->delay.enable);
	of_property_read_u32(np_mcu_panel, "disable-delay-ms", &mcu_panel->desc->delay.disable);
	of_property_read_u32(np_mcu_panel, "unprepare-delay-ms",
			     &mcu_panel->desc->delay.unprepare);
	of_property_read_u32(np_mcu_panel, "reset-delay-ms", &mcu_panel->desc->delay.reset);
	of_property_read_u32(np_mcu_panel, "init-delay-ms", &mcu_panel->desc->delay.init);

	data = of_get_property(np_mcu_panel, "panel-init-sequence", &len);
	if (data) {
		mcu_panel->desc->init_seq = devm_kzalloc(dev, sizeof(*mcu_panel->desc->init_seq),
							 GFP_KERNEL);
		if (!mcu_panel->desc->init_seq)
			return -ENOMEM;

		ret = rockchip_mcu_panel_parse_cmd_seq(dev, data, len,
						       mcu_panel->desc->init_seq);
		if (ret < 0) {
			DRM_DEV_ERROR(dev, "failed to parse init sequence\n");
			return ret;
		}
	}

	data = of_get_property(np_mcu_panel, "panel-exit-sequence", &len);
	if (data) {
		mcu_panel->desc->exit_seq = devm_kzalloc(dev, sizeof(*mcu_panel->desc->exit_seq),
					      GFP_KERNEL);
		if (!mcu_panel->desc->exit_seq)
			return -ENOMEM;

		ret = rockchip_mcu_panel_parse_cmd_seq(dev, data, len,
						       mcu_panel->desc->exit_seq);
		if (ret < 0) {
			DRM_DEV_ERROR(dev, "failed to parse exit sequence\n");
			return ret;
		}
	}

	/*
	 * Support to find crtc device for both vop and vop3:
	 * vopl/vopb       -> rgb
	 * vop2/vop3 -> vp -> rgb
	 */
	port = of_graph_get_port_by_id(dev->of_node, 0);
	if (port) {
		for_each_child_of_node(port, endpoint) {
			if (of_device_is_available(endpoint)) {
				remote = of_graph_get_remote_endpoint(endpoint);
				if (remote) {
					np_crtc = of_get_next_parent(remote);
					mcu_panel->np_crtc = np_crtc;

					of_node_put(np_crtc);
					break;
				}
			}
		}

		if (!mcu_panel->np_crtc) {
			DRM_DEV_ERROR(dev, "failed to find available crtc for mcu panel\n");
			return -EINVAL;
		}

		np_mcu_timing = of_get_child_by_name(mcu_panel->np_crtc, "mcu-timing");
		if (!np_mcu_timing) {
			np_crtc = of_get_parent(mcu_panel->np_crtc);
			if (np_crtc)
				np_mcu_timing = of_get_child_by_name(np_crtc, "mcu-timing");

			if (!np_mcu_timing) {
				DRM_DEV_ERROR(dev, "failed to find timing config for mcu panel\n");
				of_node_put(np_crtc);
				return -EINVAL;
			}

			of_node_put(np_crtc);
		}

		ret = of_property_read_u32(np_mcu_timing, "mcu-pix-total", &val);
		if (ret || val == 0) {
			DRM_DEV_ERROR(dev, "failed to parse mcu_pix_total config\n");
			of_node_put(np_mcu_timing);
			return -EINVAL;
		}
		rgb->mcu_pix_total = val;

		of_node_put(np_mcu_timing);
	}

	return 0;
}

static void rockchip_mcu_panel_sleep(unsigned int msec)
{
	if (msec > 20)
		msleep(msec);
	else
		usleep_range(msec * 1000, (msec + 1) * 1000);
}

static int rockchip_mcu_panel_xfer_mcu_cmd_seq(struct rockchip_mcu_panel *mcu_panel,
					       struct mcu_cmd_seq *cmds)
{
	struct drm_device *drm_dev = mcu_panel->drm_dev;
	struct drm_panel *panel = &mcu_panel->base;
	struct device_node *np_crtc = mcu_panel->np_crtc;
	struct drm_crtc *crtc;
	struct mcu_cmd_desc *cmd;
	struct rockchip_drm_private *priv;
	int i;
	int pipe = 0;
	u32 value;

	if (!cmds)
		return -EINVAL;

	drm_for_each_crtc(crtc, drm_dev) {
		if (crtc->port == np_crtc)
			break;
	}

	pipe = drm_crtc_index(crtc);
	priv = crtc->dev->dev_private;
	if (!priv->crtc_funcs[pipe]->crtc_send_mcu_cmd) {
		DRM_DEV_ERROR(panel->dev, "crtc not supported to send mcu cmds\n");
		return -EINVAL;
	}

	priv->crtc_funcs[pipe]->crtc_send_mcu_cmd(crtc, MCU_SETBYPASS, 1);
	for (i = 0; i < cmds->cmd_cnt; i++) {
		cmd = &cmds->cmds[i];
		value = cmd->payload[0];
		priv->crtc_funcs[pipe]->crtc_send_mcu_cmd(crtc, cmd->header.data_type, value);
		if (cmd->header.delay)
			rockchip_mcu_panel_sleep(cmd->header.delay);
	}
	priv->crtc_funcs[pipe]->crtc_send_mcu_cmd(crtc, MCU_SETBYPASS, 0);

	return 0;
}

static int rockchip_mcu_panel_disable(struct drm_panel *panel)
{
	struct rockchip_mcu_panel *mcu_panel = to_rockchip_mcu_panel(panel);
	int ret = 0;

	if (!mcu_panel->enabled)
		return 0;

	if (mcu_panel->desc->delay.disable)
		msleep(mcu_panel->desc->delay.disable);

	ret = rockchip_mcu_panel_xfer_mcu_cmd_seq(mcu_panel, mcu_panel->desc->exit_seq);
	if (ret)
		DRM_DEV_ERROR(panel->dev, "failed to send exit cmds seq\n");

	mcu_panel->enabled = false;

	return 0;
}

static int rockchip_mcu_panel_unprepare(struct drm_panel *panel)
{
	struct rockchip_mcu_panel *mcu_panel = to_rockchip_mcu_panel(panel);

	if (!mcu_panel->prepared)
		return 0;

	gpiod_direction_output(mcu_panel->reset_gpio, 1);
	gpiod_direction_output(mcu_panel->enable_gpio, 0);

	if (mcu_panel->desc->delay.unprepare)
		msleep(mcu_panel->desc->delay.unprepare);

	mcu_panel->prepared = false;

	return 0;
}

static int rockchip_mcu_panel_prepare(struct drm_panel *panel)
{
	struct rockchip_mcu_panel *mcu_panel = to_rockchip_mcu_panel(panel);
	unsigned int delay;

	if (mcu_panel->prepared)
		return 0;

	gpiod_direction_output(mcu_panel->enable_gpio, 1);

	delay = mcu_panel->desc->delay.prepare;
	if (delay)
		msleep(delay);

	gpiod_direction_output(mcu_panel->reset_gpio, 1);

	if (mcu_panel->desc->delay.reset)
		msleep(mcu_panel->desc->delay.reset);

	gpiod_direction_output(mcu_panel->reset_gpio, 0);

	if (mcu_panel->desc->delay.init)
		msleep(mcu_panel->desc->delay.init);

	mcu_panel->prepared = true;

	return 0;
}

static int rockchip_mcu_panel_enable(struct drm_panel *panel)
{
	struct rockchip_mcu_panel *mcu_panel = to_rockchip_mcu_panel(panel);
	int ret = 0;

	if (mcu_panel->enabled)
		return 0;

	ret = rockchip_mcu_panel_xfer_mcu_cmd_seq(mcu_panel, mcu_panel->desc->init_seq);
	if (ret)
		DRM_DEV_ERROR(panel->dev, "failed to send init cmds seq\n");

	if (mcu_panel->desc->delay.enable)
		msleep(mcu_panel->desc->delay.enable);

	mcu_panel->enabled = true;

	return 0;
}

static int rockchip_mcu_panel_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct rockchip_mcu_panel *mcu_panel = to_rockchip_mcu_panel(panel);
	struct drm_display_mode *m, *mode;

	if (!mcu_panel->desc)
		return 0;

	m = mcu_panel->desc->mode;
	mode = drm_mode_duplicate(connector->dev, m);
	if (!mode) {
		DRM_DEV_ERROR(mcu_panel->base.dev, "failed to add mode %ux%u@%u\n",
			      m->hdisplay, m->vdisplay,
			      drm_mode_vrefresh(m));
		return 0;
	}

	mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);

	drm_mode_probed_add(connector, mode);

	if (mcu_panel->desc->bpc)
		connector->display_info.bpc = mcu_panel->desc->bpc;
	if (mcu_panel->desc->size.width)
		connector->display_info.width_mm = mcu_panel->desc->size.width;
	if (mcu_panel->desc->size.height)
		connector->display_info.height_mm = mcu_panel->desc->size.height;
	if (mcu_panel->desc->bus_format)
		drm_display_info_set_bus_formats(&connector->display_info,
						 &mcu_panel->desc->bus_format, 1);
	if (mcu_panel->desc->bus_flags)
		connector->display_info.bus_flags = mcu_panel->desc->bus_flags;

	return 1;
}

static const struct drm_panel_funcs rockchip_mcu_panel_funcs = {
	.disable = rockchip_mcu_panel_disable,
	.unprepare = rockchip_mcu_panel_unprepare,
	.prepare = rockchip_mcu_panel_prepare,
	.enable = rockchip_mcu_panel_enable,
	.get_modes = rockchip_mcu_panel_get_modes,
};

static struct backlight_device *rockchip_mcu_panel_find_backlight(struct rockchip_rgb *rgb)
{
	struct backlight_device *bd = NULL;
	struct device_node *np_mcu_panel = rgb->np_mcu_panel;
	struct device_node *np = NULL;

	np = of_parse_phandle(np_mcu_panel, "backlight", 0);
	if (np) {
		bd = of_find_backlight_by_node(np);
		if (IS_ERR_OR_NULL(bd))
			return NULL;

		of_node_put(np);

		if (!bd->props.brightness)
			bd->props.brightness = bd->props.max_brightness;
	}

	return bd;
}

static int rockchip_rgb_bind(struct device *dev, struct device *master,
			     void *data)
{
	struct rockchip_rgb *rgb = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct drm_encoder *encoder = &rgb->encoder;
	struct drm_connector *connector;
	int ret;

	if (rgb->np_mcu_panel) {
		struct rockchip_mcu_panel *mcu_panel;

		mcu_panel = devm_kzalloc(dev, sizeof(*mcu_panel), GFP_KERNEL);
		if (!mcu_panel) {
			return -ENOMEM;
		}
		mcu_panel->drm_dev = drm_dev;

		rgb->panel = &mcu_panel->base;

		ret = rockchip_mcu_panel_init(rgb);
		if (ret < 0) {
			DRM_DEV_ERROR(dev, "failed to init mcu panel: %d\n", ret);
			return ret;
		}

		rgb->panel->backlight = rockchip_mcu_panel_find_backlight(rgb);
		if (!rgb->panel->backlight) {
			DRM_DEV_ERROR(dev, "failed to find backlight device");
			return -EINVAL;
		}

		drm_panel_init(&mcu_panel->base, dev, &rockchip_mcu_panel_funcs,
			       DRM_MODE_CONNECTOR_DPI);

		drm_panel_add(&mcu_panel->base);
	} else {
		ret = drm_of_find_panel_or_bridge(dev->of_node, 1, -1,
						  &rgb->panel, &rgb->bridge);
		if (ret) {
			DRM_DEV_ERROR(dev, "failed to find panel or bridge: %d\n", ret);
			return ret;
		}
	}

	encoder->possible_crtcs = rockchip_drm_of_find_possible_crtcs(drm_dev,
								      dev->of_node);

	ret = drm_encoder_init(drm_dev, encoder, &rockchip_rgb_encoder_funcs,
			       DRM_MODE_ENCODER_DPI, NULL);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to initialize encoder: %d\n", ret);
		return ret;
	}

	drm_encoder_helper_add(encoder, &rockchip_rgb_encoder_helper_funcs);

	if (rgb->panel) {
		struct rockchip_drm_private *private = drm_dev->dev_private;

		connector = &rgb->connector;
		connector->interlace_allowed = true;
		ret = drm_connector_init(drm_dev, connector,
					 &rockchip_rgb_connector_funcs,
					 DRM_MODE_CONNECTOR_DPI);
		if (ret < 0) {
			DRM_DEV_ERROR(dev,
				      "failed to initialize connector: %d\n",
				      ret);
			goto err_free_encoder;
		}

		drm_connector_helper_add(connector,
					 &rockchip_rgb_connector_helper_funcs);

		ret = drm_connector_attach_encoder(connector, encoder);
		if (ret < 0) {
			DRM_DEV_ERROR(dev,
				      "failed to attach encoder: %d\n", ret);
			goto err_free_connector;
		}
		rgb->sub_dev.connector = &rgb->connector;
		rgb->sub_dev.of_node = rgb->dev->of_node;
		rgb->sub_dev.loader_protect = rockchip_rgb_encoder_loader_protect;
		drm_object_attach_property(&connector->base, private->connector_id_prop, 0);
		rockchip_drm_register_sub_dev(&rgb->sub_dev);
	} else {
		rgb->bridge->encoder = encoder;
		ret = drm_bridge_attach(encoder, rgb->bridge, NULL, 0);
		if (ret) {
			DRM_DEV_ERROR(dev,
				      "failed to attach bridge: %d\n", ret);
			goto err_free_encoder;
		}
	}

	return 0;

err_free_connector:
	drm_connector_cleanup(connector);
err_free_encoder:
	drm_encoder_cleanup(encoder);
	return ret;
}

static void rockchip_rgb_unbind(struct device *dev, struct device *master,
				void *data)
{
	struct rockchip_rgb *rgb = dev_get_drvdata(dev);

	if (rgb->sub_dev.connector)
		rockchip_drm_unregister_sub_dev(&rgb->sub_dev);
	if (rgb->panel)
		drm_connector_cleanup(&rgb->connector);

	drm_encoder_cleanup(&rgb->encoder);
}

static const struct component_ops rockchip_rgb_component_ops = {
	.bind = rockchip_rgb_bind,
	.unbind = rockchip_rgb_unbind,
};

static int rockchip_rgb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rockchip_rgb *rgb;
	const struct rockchip_rgb_data *rgb_data;
	struct fwnode_handle *fwnode_mcu_panel;
	int ret, id;

	rgb = devm_kzalloc(&pdev->dev, sizeof(*rgb), GFP_KERNEL);
	if (!rgb)
		return -ENOMEM;

	id = of_alias_get_id(dev->of_node, "rgb");
	if (id < 0)
		id = 0;

	rgb->data_sync_bypass = of_property_read_bool(dev->of_node, "rockchip,data-sync-bypass");

	fwnode_mcu_panel = device_get_named_child_node(dev, "mcu-panel");
	if (fwnode_mcu_panel)
		rgb->np_mcu_panel = to_of_node(fwnode_mcu_panel);

	rgb_data = of_device_get_match_data(dev);
	if (rgb_data) {
		rgb->funcs = rgb_data->funcs;
		if (rgb->np_mcu_panel)
			rgb->max_dclk_rate = rgb_data->mcu_max_dclk_rate;
		else
			rgb->max_dclk_rate = rgb_data->rgb_max_dclk_rate;
	}
	rgb->id = id;
	rgb->dev = dev;
	platform_set_drvdata(pdev, rgb);

	if (dev->parent && dev->parent->of_node) {
		rgb->grf = syscon_node_to_regmap(dev->parent->of_node);
		if (IS_ERR(rgb->grf)) {
			ret = PTR_ERR(rgb->grf);
			dev_err(dev, "Unable to get grf: %d\n", ret);
			return ret;
		}
	}

	rgb->phy = devm_phy_optional_get(dev, "phy");
	if (IS_ERR(rgb->phy)) {
		ret = PTR_ERR(rgb->phy);
		dev_err(dev, "failed to get phy: %d\n", ret);
		return ret;
	}

	return component_add(dev, &rockchip_rgb_component_ops);
}

static int rockchip_rgb_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &rockchip_rgb_component_ops);

	return 0;
}

static void px30_rgb_enable(struct rockchip_rgb *rgb)
{
	int pipe = drm_of_encoder_active_endpoint_id(rgb->dev->of_node,
						     &rgb->encoder);

	regmap_write(rgb->grf, PX30_GRF_PD_VO_CON1, PX30_RGB_VOP_SEL(pipe) |
		     PX30_RGB_DATA_SYNC_BYPASS(rgb->data_sync_bypass));
}

static const struct rockchip_rgb_funcs px30_rgb_funcs = {
	.enable = px30_rgb_enable,
};

static const struct rockchip_rgb_data px30_rgb = {
	.funcs = &px30_rgb_funcs,
};

static void rk1808_rgb_enable(struct rockchip_rgb *rgb)
{
	regmap_write(rgb->grf, RK1808_GRF_PD_VO_CON1,
		     RK1808_RGB_DATA_SYNC_BYPASS(rgb->data_sync_bypass));
}

static const struct rockchip_rgb_funcs rk1808_rgb_funcs = {
	.enable = rk1808_rgb_enable,
};

static const struct rockchip_rgb_data rk1808_rgb = {
	.funcs = &rk1808_rgb_funcs,
};

static void rk3288_rgb_enable(struct rockchip_rgb *rgb)
{
	int pipe = drm_of_encoder_active_endpoint_id(rgb->dev->of_node,
						     &rgb->encoder);

	regmap_write(rgb->grf, RK3288_GRF_SOC_CON6, RK3288_LVDS_LCDC_SEL(pipe));
	regmap_write(rgb->grf, RK3288_GRF_SOC_CON7,
		     RK3288_LVDS_PWRDWN(0) | RK3288_LVDS_CON_ENABLE_2(1) |
		     RK3288_LVDS_CON_ENABLE_1(1) | RK3288_LVDS_CON_CLKINV(0) |
		     RK3288_LVDS_CON_TTL_EN(1));
}

static void rk3288_rgb_disable(struct rockchip_rgb *rgb)
{
	regmap_write(rgb->grf, RK3288_GRF_SOC_CON7,
		     RK3288_LVDS_PWRDWN(1) | RK3288_LVDS_CON_ENABLE_2(0) |
		     RK3288_LVDS_CON_ENABLE_1(0) | RK3288_LVDS_CON_TTL_EN(0));
}

static const struct rockchip_rgb_funcs rk3288_rgb_funcs = {
	.enable = rk3288_rgb_enable,
	.disable = rk3288_rgb_disable,
};

static const struct rockchip_rgb_data rk3288_rgb = {
	.funcs = &rk3288_rgb_funcs,
};

static void rk3562_rgb_enable(struct rockchip_rgb *rgb)
{
	regmap_write(rgb->grf, RK3562_GRF_IOC_VO_IO_CON,
		     RK3562_RGB_DATA_BYPASS(rgb->data_sync_bypass));
}

static const struct rockchip_rgb_funcs rk3562_rgb_funcs = {
	.enable = rk3562_rgb_enable,
};

static const struct rockchip_rgb_data rk3562_rgb = {
	.funcs = &rk3562_rgb_funcs,
};

static void rk3568_rgb_enable(struct rockchip_rgb *rgb)
{
	regmap_write(rgb->grf, RK3568_GRF_VO_CON1,
		     RK3568_RGB_DATA_BYPASS(rgb->data_sync_bypass));
}

static const struct rockchip_rgb_funcs rk3568_rgb_funcs = {
	.enable = rk3568_rgb_enable,
};

static const struct rockchip_rgb_data rk3568_rgb = {
	.funcs = &rk3568_rgb_funcs,
};

static void rv1126_rgb_enable(struct rockchip_rgb *rgb)
{
	regmap_write(rgb->grf, RV1126_GRF_IOFUNC_CON3,
		     RV1126_LCDC_IO_BYPASS(rgb->data_sync_bypass));
}

static const struct rockchip_rgb_funcs rv1126_rgb_funcs = {
	.enable = rv1126_rgb_enable,
};

static const struct rockchip_rgb_data rv1126_rgb = {
	.funcs = &rv1126_rgb_funcs,
};

static void rv1106_rgb_enable(struct rockchip_rgb *rgb)
{
	regmap_write(rgb->grf, RV1106_VENC_GRF_VOP_IO_WRAPPER,
		     RV1106_IO_BYPASS_SEL(rgb->data_sync_bypass ? 0x3 : 0x0));
	regmap_write(rgb->grf, RV1106_VOGRF_VOP_PIPE_BYPASS,
		     RV1106_VOP_PIPE_BYPASS(rgb->data_sync_bypass ? 0x3 : 0x0));
}

static const struct rockchip_rgb_funcs rv1106_rgb_funcs = {
	.enable = rv1106_rgb_enable,
};

static const struct rockchip_rgb_data rv1106_rgb = {
	.rgb_max_dclk_rate = 74250,
	.mcu_max_dclk_rate = 150000,
	.funcs = &rv1106_rgb_funcs,
};

static const struct of_device_id rockchip_rgb_dt_ids[] = {
	{ .compatible = "rockchip,px30-rgb", .data = &px30_rgb },
	{ .compatible = "rockchip,rk1808-rgb", .data = &rk1808_rgb },
	{ .compatible = "rockchip,rk3066-rgb", },
	{ .compatible = "rockchip,rk3128-rgb", },
	{ .compatible = "rockchip,rk3288-rgb", .data = &rk3288_rgb },
	{ .compatible = "rockchip,rk3308-rgb", },
	{ .compatible = "rockchip,rk3368-rgb", },
	{ .compatible = "rockchip,rk3562-rgb", .data = &rk3562_rgb },
	{ .compatible = "rockchip,rk3568-rgb", .data = &rk3568_rgb },
	{ .compatible = "rockchip,rk3588-rgb", },
	{ .compatible = "rockchip,rv1106-rgb", .data = &rv1106_rgb},
	{ .compatible = "rockchip,rv1108-rgb", },
	{ .compatible = "rockchip,rv1126-rgb", .data = &rv1126_rgb},
	{}
};
MODULE_DEVICE_TABLE(of, rockchip_rgb_dt_ids);

struct platform_driver rockchip_rgb_driver = {
	.probe = rockchip_rgb_probe,
	.remove = rockchip_rgb_remove,
	.driver = {
		.name = "rockchip-rgb",
		.of_match_table = of_match_ptr(rockchip_rgb_dt_ids),
	},
};
