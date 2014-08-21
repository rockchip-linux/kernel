/*
 * Copyright (C) 2014 Google, Inc.
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

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <drm/panel/panel-jdi-lpm102a188a.h>

#include <video/mipi_display.h>

struct panel_jdi {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;
	struct mipi_dsi_device *slave;

	struct regulator *supply;
	struct regulator *ddi_supply;
	int enable_gpio;
	unsigned long enable_gpio_flags;
	int reset_gpio;
	unsigned long reset_gpio_flags;

	const struct drm_display_mode *mode;

	bool enabled;
};

static inline struct panel_jdi *to_panel_jdi(struct drm_panel *panel)
{
	return container_of(panel, struct panel_jdi, base);
}

static int panel_jdi_write_display_brightness(struct panel_jdi *jdi)
{
	int ret;
	u8 data;

	data = RSP_WRITE_DISPLAY_BRIGHTNESS(0xFF);

	ret = mipi_dsi_dcs_write(jdi->dsi,
			MIPI_DCS_RSP_WRITE_DISPLAY_BRIGHTNESS, &data, 1);
	if (ret < 1) {
		DRM_ERROR("failed to write display brightness: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_write(jdi->slave,
			MIPI_DCS_RSP_WRITE_DISPLAY_BRIGHTNESS, &data, 1);
	if (ret < 1) {
		DRM_ERROR("failed to write display brightness: %d\n", ret);
		return ret;
	}

	return 0;
}

static int panel_jdi_write_control_display(struct panel_jdi *jdi)
{
	int ret;
	u8 data;

	data = RSP_WRITE_CONTROL_DISPLAY_BL_ON |
			RSP_WRITE_CONTROL_DISPLAY_BCTRL_LEDPWM;

	ret = mipi_dsi_dcs_write(jdi->dsi, MIPI_DCS_RSP_WRITE_CONTROL_DISPLAY,
			&data, 1);
	if (ret < 1) {
		DRM_ERROR("failed to write control display: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_write(jdi->slave, MIPI_DCS_RSP_WRITE_CONTROL_DISPLAY,
			&data, 1);
	if (ret < 1) {
		DRM_ERROR("failed to write control display: %d\n", ret);
		return ret;
	}

	return 0;
}

static int panel_jdi_write_adaptive_brightness_control(struct panel_jdi *jdi)
{
	int ret;
	u8 data;

	data = 0;

	ret = mipi_dsi_dcs_write(jdi->dsi,
			MIPI_DCS_RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL, &data,
			1);
	if (ret < 1) {
		DRM_ERROR("failed to set adaptive brightness ctrl: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_write(jdi->slave,
			MIPI_DCS_RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL, &data,
			1);
	if (ret < 1) {
		DRM_ERROR("failed to set adaptive brightness ctrl: %d\n", ret);
		return ret;
	}

	return 0;
}

static int panel_jdi_disable(struct drm_panel *panel)
{
	struct panel_jdi *jdi = to_panel_jdi(panel);
	int ret;

	if (!jdi->enabled)
		return 0;

	return ret;
}

static int panel_jdi_unprepare(struct drm_panel *panel)
{
	struct panel_jdi *jdi = to_panel_jdi(panel);
	int ret;

	if (!jdi->enabled)
		return 0;

	ret = mipi_dsi_dcs_set_display_off(jdi->dsi);
	if (ret < 0)
		DRM_ERROR("failed to set display off: %d\n", ret);

	/* Specified by JDI @ 50ms, subject to change */
	msleep(50);

	ret = mipi_dsi_dcs_enter_sleep_mode(jdi->dsi);
	if (ret < 0)
		DRM_ERROR("failed to enter sleep mode: %d\n", ret);

	/* Specified by JDI @ 150ms, subject to change */
	msleep(150);

	gpio_set_value(jdi->reset_gpio,
		(jdi->reset_gpio_flags & GPIO_ACTIVE_LOW) ? 0 : 1);

	/* T4 = 1ms */
	usleep_range(1000, 3000);

	gpio_set_value(jdi->enable_gpio,
		(jdi->enable_gpio_flags & GPIO_ACTIVE_LOW) ? 1 : 0);

	/* T5 = 2ms */
	usleep_range(2000, 4000);

	jdi->enabled = false;

	return 0;
}

static int panel_jdi_prepare(struct drm_panel *panel)
{
	struct panel_jdi *jdi = to_panel_jdi(panel);
	int ret;

	if (jdi->enabled)
		return 0;

	gpio_set_value(jdi->enable_gpio,
		(jdi->enable_gpio_flags & GPIO_ACTIVE_LOW) ? 0 : 1);

	/* T2 = 10ms */
	usleep_range(10000, 15000);

	gpio_set_value(jdi->reset_gpio,
		(jdi->reset_gpio_flags & GPIO_ACTIVE_LOW) ? 1 : 0);

	/* Specified by JDI @ 3ms, subject to change */
	usleep_range(3000, 5000);

	ret = mipi_dsi_dcs_set_column_address(jdi->dsi, 0,
				jdi->mode->hdisplay / 2 - 1);
	if (ret < 0)
		DRM_ERROR("failed to set column address: %d\n", ret);

	ret = mipi_dsi_dcs_set_column_address(jdi->slave, 0,
				jdi->mode->hdisplay / 2 - 1);
	if (ret < 0)
		DRM_ERROR("failed to set column address: %d\n", ret);

	ret = mipi_dsi_dcs_set_page_address(jdi->dsi, 0,
				jdi->mode->vdisplay - 1);
	if (ret < 0)
		DRM_ERROR("failed to set page address: %d\n", ret);

	ret = mipi_dsi_dcs_set_page_address(jdi->slave, 0,
				jdi->mode->vdisplay - 1);
	if (ret < 0)
		DRM_ERROR("failed to set page address: %d\n", ret);

	ret = mipi_dsi_dcs_exit_sleep_mode(jdi->dsi);
	if (ret < 0)
		DRM_ERROR("failed to exit sleep mode: %d\n", ret);

	ret = mipi_dsi_dcs_exit_sleep_mode(jdi->slave);
	if (ret < 0)
		DRM_ERROR("failed to exit sleep mode: %d\n", ret);

	ret = mipi_dsi_dcs_set_tear_on(jdi->dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0)
		DRM_ERROR("failed to set tear on: %d\n", ret);

	ret = mipi_dsi_dcs_set_tear_on(jdi->slave,
			MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0)
		DRM_ERROR("failed to set tear on: %d\n", ret);

	ret = mipi_dsi_dcs_set_address_mode(jdi->dsi, false, false, false,
			false, false, false, false, false);
	if (ret < 0)
		DRM_ERROR("failed to set address mode: %d\n", ret);

	ret = mipi_dsi_dcs_set_address_mode(jdi->slave, false, false,
			false, false, false, false, false, false);
	if (ret < 0)
		DRM_ERROR("failed to set address mode: %d\n", ret);

	ret = mipi_dsi_dcs_set_pixel_format(jdi->dsi, MIPI_DCS_PIXEL_FMT_24BIT);
	if (ret < 0)
		DRM_ERROR("failed to set pixel format: %d\n", ret);

	ret = mipi_dsi_dcs_set_pixel_format(jdi->slave,
				MIPI_DCS_PIXEL_FMT_24BIT);
	if (ret < 0)
		DRM_ERROR("failed to set pixel format: %d\n", ret);

	ret = panel_jdi_write_display_brightness(jdi);
	if (ret < 0)
		DRM_ERROR("failed to write display brightness: %d\n", ret);

	ret = panel_jdi_write_control_display(jdi);
	if (ret < 0)
		DRM_ERROR("failed to write control display: %d\n", ret);

	ret = panel_jdi_write_adaptive_brightness_control(jdi);
	if (ret < 0)
		DRM_ERROR("failed to set adaptive brightness ctrl: %d\n", ret);

	ret = mipi_dsi_dcs_set_display_on(jdi->dsi);
	if (ret < 0)
		DRM_ERROR("failed to set display on: %d\n", ret);

	ret = mipi_dsi_dcs_set_display_on(jdi->slave);
	if (ret < 0)
		DRM_ERROR("failed to set display on: %d\n", ret);

	jdi->enabled = true;

	return ret;
}

static int panel_jdi_enable(struct drm_panel *panel)
{
	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 301620,
	.hdisplay = 2560,
	.hsync_start = 2560 + 80,
	.hsync_end = 2560 + 80 + 80,
	.htotal = 2560 + 80 + 80 + 80,
	.vdisplay = 1800,
	.vsync_start = 1800 + 4,
	.vsync_end = 1800 + 4 + 4,
	.vtotal = 1800 + 4 + 4 + 4,

	.vrefresh = 60,
};

static int panel_jdi_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		DRM_ERROR("failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = 211;
	panel->connector->display_info.height_mm = 148;

	return 1;
}

static const struct drm_panel_funcs panel_jdi_funcs = {
	.prepare = panel_jdi_prepare,
	.enable = panel_jdi_enable,
	.disable = panel_jdi_disable,
	.unprepare = panel_jdi_unprepare,
	.get_modes = panel_jdi_get_modes,
};

static const struct of_device_id jdi_of_match[] = {
	{ .compatible = "jdi,lpm102a188a", },
	{ }
};
MODULE_DEVICE_TABLE(of, jdi_of_match);

static int panel_jdi_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct panel_jdi *jdi;
	struct device_node *np;
	struct mipi_dsi_device *slave;
	enum of_gpio_flags gpio_flags;
	int ret;
	unsigned int value;

	/* if this device is the secondary link, there's nothing more to do */
	np = of_parse_phandle(dsi->dev.of_node, "slave", 0);
	if (!np)
		return 0;

	/* this device is the primary interface */
	slave = of_find_mipi_dsi_device_by_node(np);
	of_node_put(np);

	if (!slave)
		return -EPROBE_DEFER;


	jdi = devm_kzalloc(&dsi->dev, sizeof(*jdi), GFP_KERNEL);
	if (!jdi)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, jdi);
	jdi->mode = &default_mode;
	jdi->slave = slave;
	jdi->dsi = dsi;

	dsi->lanes = 8;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = 0;

	jdi->supply = devm_regulator_get(&dsi->dev, "power");
	if (IS_ERR(jdi->supply))
		return PTR_ERR(jdi->supply);

	jdi->ddi_supply = devm_regulator_get(&dsi->dev, "ddi");
	if (IS_ERR(jdi->ddi_supply))
		return PTR_ERR(jdi->ddi_supply);

	jdi->enable_gpio = of_get_named_gpio_flags(dsi->dev.of_node,
				"enable-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(jdi->enable_gpio)) {
		DRM_ERROR("enable gpio not found: %d\n", ret);
		return -ENODEV;
	}

	if (gpio_flags & OF_GPIO_ACTIVE_LOW)
		jdi->enable_gpio_flags |= GPIO_ACTIVE_LOW;

	ret = devm_gpio_request(&dsi->dev, jdi->enable_gpio, "jdi-enable");
	if (ret < 0) {
		DRM_ERROR("Request enable gpio failed: %d\n", ret);
		return ret;
	}

	value = (jdi->enable_gpio_flags & GPIO_ACTIVE_LOW) ? 1 : 0;
	ret = gpio_direction_output(jdi->enable_gpio, value);
	if (ret < 0) {
		DRM_ERROR("Set enable gpio direction failed: %d\n", ret);
		return ret;
	}

	jdi->reset_gpio = of_get_named_gpio_flags(dsi->dev.of_node,
				"reset-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(jdi->reset_gpio)) {
		DRM_ERROR("reset gpio not found: %d\n", ret);
		return -ENODEV;
	}

	if (gpio_flags & OF_GPIO_ACTIVE_LOW)
		jdi->reset_gpio_flags |= GPIO_ACTIVE_LOW;

	ret = devm_gpio_request(&dsi->dev, jdi->reset_gpio, "jdi-reset");
	if (ret < 0) {
		DRM_ERROR("Request reset gpio failed: %d\n", ret);
		return ret;
	}

	value = (jdi->reset_gpio_flags & GPIO_ACTIVE_LOW) ? 0 : 1;
	ret = gpio_direction_output(jdi->reset_gpio, value);
	if (ret < 0) {
		DRM_ERROR("Set enable gpio direction failed: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(jdi->supply);
	if (ret < 0) {
		DRM_ERROR("failed to enable supply: %d\n", ret);
		return ret;
	}

	/* T1 = 2ms */
	usleep_range(2000, 4000);

	ret = regulator_enable(jdi->ddi_supply);
	if (ret < 0) {
		DRM_ERROR("failed to enable ddi_supply: %d\n", ret);
		return ret;
	}

	/* T2 = 1ms */
	usleep_range(1000, 3000);

	drm_panel_init(&jdi->base);
	jdi->base.dev = &dsi->dev;
	jdi->base.funcs = &panel_jdi_funcs;

	ret = drm_panel_add(&jdi->base);
	if (ret < 0) {
		DRM_ERROR("drm_panel_add failed: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		return ret;

	return 0;
}

static int panel_jdi_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct panel_jdi *jdi = mipi_dsi_get_drvdata(dsi);
	int ret;

	/* nothing to do for the secondary interface */
	if (!jdi->slave)
		return 0;

	panel_jdi_disable(&jdi->base);

	regulator_disable(jdi->supply);

	/* T6 = 2ms */
	usleep_range(2000, 4000);

	regulator_disable(jdi->ddi_supply);

	/* Specified by JDI @ 50ms, subject to change */
	msleep(50);

	drm_panel_detach(&jdi->base);
	drm_panel_remove(&jdi->base);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		DRM_ERROR("failed to detach from DSI host: %d\n", ret);

	put_device(&jdi->slave->dev);

	return 0;
}

static void panel_jdi_dsi_shutdown(struct mipi_dsi_device *dsi)
{
	struct panel_jdi *jdi = mipi_dsi_get_drvdata(dsi);

	panel_jdi_disable(&jdi->base);
}

static struct mipi_dsi_driver panel_jdi_dsi_driver = {
	.driver = {
		.name = "panel-jdi-lpm102a188a-dsi",
		.of_match_table = jdi_of_match,
	},
	.probe = panel_jdi_dsi_probe,
	.remove = panel_jdi_dsi_remove,
	.shutdown = panel_jdi_dsi_shutdown,
};
module_mipi_dsi_driver(panel_jdi_dsi_driver);
MODULE_AUTHOR("Sean Paul <seanpaul@chromium.org>");
MODULE_DESCRIPTION("DRM Driver for JDI LPM102A188A");
MODULE_LICENSE("GPL and additional rights");
