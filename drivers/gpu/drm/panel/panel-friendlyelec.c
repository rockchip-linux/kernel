/*
 * Copyright (C) Guangzhou FriendlyARM Computer Tech. Co., Ltd.
 * (http://www.friendlyarm.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_panel.h>

#include <linux/platform_data/ctouch.h>

/* -------------------------------------------------------------- */
#include "panel-friendlyelec.h"

static struct lcd_desc hd700 = {
	.width = 800,
	.height = 1280,
	.p_width = 94,
	.p_height = 151,
	.bpp = 24,
	.freq = 60,

	.timing = {
		.h_fp = 20,
		.h_bp = 20,
		.h_sw = 24,
		.v_fp = 4,
		.v_fpe = 1,
		.v_bp = 4,
		.v_bpe = 1,
		.v_sw = 8,
	},
	.polarity = {
		.rise_vclk = 0,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};

static struct lcd_desc s70 = {
	.width = 800,
	.height = 480,
	.p_width = 155,
	.p_height = 93,
	.bpp = 24,
	.freq = 61,

	.timing = {
		.h_fp = 48,
		.h_bp = 36,
		.h_sw = 10,
		.v_fp = 22,
		.v_fpe = 1,
		.v_bp = 15,
		.v_bpe = 1,
		.v_sw = 8,
	},
	.polarity = {
		.rise_vclk = 0,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};

static struct lcd_desc hd101 = {
	.width = 1280,
	.height = 800,
	.p_width = 218,
	.p_height = 136,
	.bpp = 24,
	.freq = 60,

	.timing = {
		.h_fp = 16,
		.h_bp = 16,
		.h_sw = 30,
		.v_fp = 8,
		.v_fpe = 1,
		.v_bp = 8,
		.v_bpe = 1,
		.v_sw = 12,
	},
	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 0,
		.inv_vsync = 0,
		.inv_vden = 0,
	},
};

static struct lcd_desc k116e = {
	.width = 1920,
	.height = 1080,
	.p_width = 256,
	.p_height = 144,
	.bpp = 24,
	.freq = 60,

	.timing = {
		.h_fp = 48,
		.h_bp = 80,
		.h_sw = 32,
		.v_fp = 3,
		.v_fpe = 1,
		.v_bp = 23,
		.v_bpe = 1,
		.v_sw = 5,
	},
	.polarity = {
		.rise_vclk = 0,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};

static struct lcd_desc k101d = {
	.width = 1200,
	.height = 1920,
	.p_width = 135,
	.p_height = 216,
	.bpp = 24,
	.freq = 60,

	.timing = {
		.h_fp = 60,
		.h_bp = 80,
		.h_sw = 24,
		.v_fp = 14,
		.v_fpe = 1,
		.v_bp = 10,
		.v_bpe = 1,
		.v_sw = 8,
	},
	.polarity = {
		.rise_vclk = 0,
		.inv_hsync = 0,
		.inv_vsync = 0,
		.inv_vden = 0,
	},
};

/* HDMI */
static struct lcd_desc hdmi_def = {
	.width = 1920,
	.height = 1080,
	.p_width = 508,
	.p_height = 286,
	.bpp = 24,
	.freq = 60,

	.timing = {
		.h_fp = 12,
		.h_bp = 12,
		.h_sw = 4,
		.v_fp = 8,
		.v_fpe = 1,
		.v_bp = 8,
		.v_bpe = 1,
		.v_sw = 4,
	},
	.polarity = {
		.rise_vclk = 0,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};

/* -------------------------------------------------------------- */

static struct hdmi_config {
	char *name;
	int width;
	int height;
} panel_hdmi_modes[] = {
	{ "HDMI1080P60",	1920, 1080 },
	{ "HDMI720P60",		1280,  720 },

	/* Non-CEA modes */
	{ "HDMI1280x800",	1280,  800 },
	{ "HDMI1024x768",	1024,  768 },
};

/* Try to identify LCD panel by kernel command line, or
 * choose *HD702* as default */

static struct {
	char *name;
	struct lcd_desc *lcd;
	int ctp;
} panel_lcd_list[] = {
	{ "HDMI",	&hdmi_def,       0  },

	{ "HD702E",	&hd700,	CTP_GOODIX  },
	{ "HD101B",	&hd101,	CTP_GOODIX  },
	{ "S701",	&s70,	CTP_GOODIX  },

	{ "LC116LF1L01",&k116e,	CTP_FT5526_KR },
	{ "LC116LF1L02",&k116e,	CTP_FT5526_KR },
	{ "TST101WUIH", &k101d,	CTP_FT5X06 },

	{ "EDID",	NULL,	CTP_AUTO    },
};

static int lcd_idx = 0;
static int lcd_connected = false;

static struct lcd_desc *panel_get_lcd_desc(void)
{
	return panel_lcd_list[lcd_idx].lcd;
}

static int panel_setup_lcd(char *str)
{
	char *delim;
	int i;

	delim = strchr(str, ',');
	if (delim)
		*delim++ = '\0';

	if (!strncasecmp("HDMI", str, 4)) {
		struct hdmi_config *cfg = &panel_hdmi_modes[0];
		struct lcd_desc *lcd;
		int w = 0, h = 0;

		lcd_idx = 0;
		lcd = panel_lcd_list[lcd_idx].lcd;

		for (i = 0; i < ARRAY_SIZE(panel_hdmi_modes); i++, cfg++) {
			if (!strcasecmp(cfg->name, str)) {
				lcd->width = cfg->width;
				lcd->height = cfg->height;
				break;
			}
		}

		/* parse <xres>x<yres> */
		if (i >=ARRAY_SIZE(panel_hdmi_modes) &&
			sscanf(str + 4, "%dx%d", &w, &h) == 2) {
			if ((w > 1920 && h > 1280) || (w > 1280 && h > 1920)) {
				lcd->width = w / 2;
				lcd->height = h / 2;
			} else if (w > 640 && h > 480) {
				lcd->width = w;
				lcd->height = h;
			}
		}

		printk("Panel: using mode %dx%d for %s\n",
				lcd->width, lcd->height, str);
		goto __ret;
	}

	for (i = 1; i < ARRAY_SIZE(panel_lcd_list); i++) {
		if (!strcasecmp(panel_lcd_list[i].name, str) ||
			(panel_lcd_list[i].lcd == NULL)) {
			lcd_idx = i;
			lcd_connected = true;
			break;
		}
	}

	printk("Panel: %s selected\n", str);

__ret:
	panel_set_touch_id(panel_lcd_list[lcd_idx].ctp);
	return 0;
}
early_param("lcd", panel_setup_lcd);

int panel_is_lcd_connected(void)
{
	return lcd_connected;
}

void panel_init_display_mode(struct drm_display_mode *dmode)
{
	struct lcd_desc *lcd = panel_lcd_list[lcd_idx].lcd;

	if (!lcd_connected && lcd) {
		dmode->hdisplay = lcd->width;
		dmode->vdisplay = lcd->height;
	}

	dmode->flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC;

	return;
}

void panel_get_display_size(int *w, int *h)
{
	struct lcd_desc *lcd = panel_lcd_list[lcd_idx].lcd;

	if (lcd) {
		if (w)
			*w = lcd->width;
		if (h)
			*h = lcd->height;
	}
}
EXPORT_SYMBOL(panel_get_display_size);

/* -------------------------------------------------------------- */

/* Touch panel type */
static unsigned int ctp_type = CTP_AUTO;

static int __init panel_setup_touch_id(char *str)
{
	unsigned int val;
	char *p = str, *end;

	val = simple_strtoul(p, &end, 10);
	if (end <= p) {
		return 1;
	}

	if (val < CTP_MAX && panel_lcd_list[lcd_idx].ctp) {
		ctp_type = val;
	} else if (val == CTP_NONE) {
		ctp_type = CTP_NONE;
	}

	return 1;
}
__setup("ctp=", panel_setup_touch_id);

unsigned int panel_get_touch_id(void)
{
	if (panel_lcd_list[lcd_idx].ctp)
		return ctp_type;
	else
		return CTP_NONE;
}
EXPORT_SYMBOL(panel_get_touch_id);

void panel_set_touch_id(int type)
{
	if (ctp_type == CTP_AUTO && type < CTP_MAX) {
		ctp_type = type;
	}
}
EXPORT_SYMBOL(panel_set_touch_id);

/* -------------------------------------------------------------- */
/* Derived from drivers/gpu/drm/panel/panel-simple.c */

struct panel_desc {
	struct drm_panel base;
	bool prepared;
	bool enabled;

	const struct drm_display_mode *mode;

	unsigned int bpc;
	u32 width_mm;
	u32 height_mm;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;

	u32 bus_format;

	struct backlight_device *backlight;
	struct regulator *supply;

	struct gpio_desc *enable_gpio;
};

static inline struct panel_desc *to_panel_desc(struct drm_panel *panel)
{
	return container_of(panel, struct panel_desc, base);
}

static int panel_disable(struct drm_panel *panel)
{
	struct panel_desc *p = to_panel_desc(panel);

	if (!p->enabled)
		return 0;

	if (p->backlight) {
		p->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(p->backlight);
	}

	if (p->delay.disable)
		msleep(p->delay.disable);

	p->enabled = false;

	return 0;
}

static int panel_unprepare(struct drm_panel *panel)
{
	struct panel_desc *p = to_panel_desc(panel);

	if (!p->prepared)
		return 0;

	if (p->enable_gpio)
		gpiod_set_value_cansleep(p->enable_gpio, 0);

	regulator_disable(p->supply);

	if (p->delay.unprepare)
		msleep(p->delay.unprepare);

	p->prepared = false;

	return 0;
}

static int panel_prepare(struct drm_panel *panel)
{
	struct panel_desc *p = to_panel_desc(panel);
	int err;

	if (!panel_is_lcd_connected()) {
		dev_err_once(panel->dev, "No panel connected\n");
		return -ENODEV;
	}

	if (p->prepared)
		return 0;

	err = regulator_enable(p->supply);
	if (err < 0) {
		dev_err(panel->dev, "failed to enable supply: %d\n", err);
		return err;
	}

	if (p->enable_gpio)
		gpiod_set_value_cansleep(p->enable_gpio, 1);

	if (p->delay.prepare)
		msleep(p->delay.prepare);

	p->prepared = true;

	return 0;
}

static int panel_enable(struct drm_panel *panel)
{
	struct panel_desc *p = to_panel_desc(panel);

	if (p->enabled)
		return 0;

	if (p->delay.enable)
		msleep(p->delay.enable);

	if (p->backlight) {
		p->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(p->backlight);
	}

	p->enabled = true;

	return 0;
}

static int panel_get_modes(struct drm_panel *panel,
			   struct drm_connector *connector)
{
	struct panel_desc *ctx = to_panel_desc(panel);
	struct drm_display_mode *mode;

	if (!ctx->mode) {
		dev_warn_once(panel->dev,
			"unable to get mode from builtin timings\n");
		return 0;
	}

	mode = drm_mode_duplicate(connector->dev, ctx->mode);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			ctx->mode->hdisplay, ctx->mode->vdisplay,
			drm_mode_vrefresh(ctx->mode));
		return 0;
	}

	drm_mode_set_name(mode);
	mode->width_mm = ctx->width_mm;
	mode->height_mm = ctx->height_mm;

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bpc = ctx->bpc;

	if (ctx->bus_format)
		drm_display_info_set_bus_formats(&connector->display_info,
				&ctx->bus_format, 1);

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs panel_funcs = {
	.disable = panel_disable,
	.unprepare = panel_unprepare,
	.prepare = panel_prepare,
	.enable = panel_enable,
	.get_modes = panel_get_modes,
};

/* -------------------------------------------------------------- */

static struct drm_display_mode def_mode = {
	.clock = 67185,
	.hdisplay = 800,
	.hsync_start = 800 + 20,
	.hsync_end = 800 + 20 + 24,
	.htotal = 800 + 20 + 24 + 20,
	.vdisplay = 1280,
	.vsync_start = 1280 + 4,
	.vsync_end = 1280 + 4 + 8,
	.vtotal = 1280 + 4 + 8 + 4,
};

static int panel_display_mode_init(struct panel_desc *ctx)
{
	struct drm_display_mode *dmode = &def_mode;
	struct lcd_desc *lcd;

	lcd = panel_get_lcd_desc();
	if (!lcd)
		return -ENODEV;

	ctx->mode = dmode;
	ctx->bpc = 8;
	ctx->width_mm = lcd->p_width;
	ctx->height_mm = lcd->p_height;

	dmode->hdisplay = lcd->width;
	dmode->hsync_start = dmode->hdisplay + lcd->timing.h_fp;
	dmode->hsync_end = dmode->hsync_start + lcd->timing.h_sw;
	dmode->htotal = dmode->hsync_end + lcd->timing.h_bp;

	dmode->vdisplay = lcd->height;
	dmode->vsync_start = dmode->vdisplay + lcd->timing.v_fp;
	dmode->vsync_end = dmode->vsync_start + lcd->timing.v_sw;
	dmode->vtotal = dmode->vsync_end + lcd->timing.v_bp;

	dmode->clock = DIV_ROUND_CLOSEST_ULL(
			dmode->htotal * dmode->vtotal * lcd->freq, 1000);

	dmode->flags = 0;

	/*
	 * ugly converion:
	 * [LCD] polarity.inv_hsync (0)
	 *   --> DRM_MODE_FLAG_PHSYNC --> DISPLAY_FLAGS_HSYNC_HIGH
	 */
	if (lcd->polarity.inv_hsync)
		dmode->flags |= DRM_MODE_FLAG_NHSYNC;
	else
		dmode->flags |= DRM_MODE_FLAG_PHSYNC;

	if (lcd->polarity.inv_vsync)
		dmode->flags |= DRM_MODE_FLAG_NVSYNC;
	else
		dmode->flags |= DRM_MODE_FLAG_PVSYNC;

	return 0;
}

static int panel_probe(struct device *dev)
{
	struct device_node *backlight;
	struct panel_desc *panel;
	const char *str;
	int err;

	err = of_property_read_string(dev->of_node, "lcd", &str);
	if (!err) {
		char lcd[64];
		strlcpy(lcd, str, sizeof(lcd));
		panel_setup_lcd(lcd);
	}

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	panel->enabled = false;
	panel->prepared = false;

	panel_display_mode_init(panel);

	panel->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(panel->supply))
		return PTR_ERR(panel->supply);

	panel->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						     GPIOD_OUT_LOW);
	if (IS_ERR(panel->enable_gpio)) {
		err = PTR_ERR(panel->enable_gpio);
		dev_err(dev, "failed to request GPIO: %d\n", err);
		return err;
	}

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		panel->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!panel->backlight)
			return -EPROBE_DEFER;
	}

	drm_panel_init(&panel->base, dev, &panel_funcs, DRM_MODE_CONNECTOR_eDP);
	drm_panel_add(&panel->base);

	dev_set_drvdata(dev, panel);

	return 0;
}

static int panel_remove(struct device *dev)
{
	struct panel_desc *panel = dev_get_drvdata(dev);

	drm_panel_remove(&panel->base);
	drm_panel_disable(&panel->base);
	drm_panel_unprepare(&panel->base);

	if (panel->backlight)
		put_device(&panel->backlight->dev);

	return 0;
}

static void panel_shutdown(struct device *dev)
{
	struct panel_desc *panel = dev_get_drvdata(dev);

	drm_panel_disable(&panel->base);
	drm_panel_unprepare(&panel->base);
}

/* -------------------------------------------------------------- */

static const struct of_device_id platform_of_match[] = {
	{
		.compatible = "lcds",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, platform_of_match);

static int panel_platform_probe(struct platform_device *pdev)
{
	return panel_probe(&pdev->dev);
}

static int panel_platform_remove(struct platform_device *pdev)
{
	return panel_remove(&pdev->dev);
}

static void panel_platform_shutdown(struct platform_device *pdev)
{
	panel_shutdown(&pdev->dev);
}

static struct platform_driver panel_platform_driver = {
	.driver = {
		.name = "panel-friendlyelec",
		.of_match_table = platform_of_match,
	},
	.probe = panel_platform_probe,
	.remove = panel_platform_remove,
	.shutdown = panel_platform_shutdown,
};

static int __init panel_init(void)
{
	return platform_driver_register(&panel_platform_driver);
}
module_init(panel_init);

static void __exit panel_exit(void)
{
	platform_driver_unregister(&panel_platform_driver);
}
module_exit(panel_exit);

MODULE_AUTHOR("support@friendlyarm.com");
MODULE_DESCRIPTION("DRM Driver for FriendlyElec Panels");
MODULE_LICENSE("GPL v2");
