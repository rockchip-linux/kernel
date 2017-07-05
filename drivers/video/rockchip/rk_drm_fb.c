/*
 * drivers/video/rockchip/rk_fb.c
 *
 * Copyright (C) ROCKCHIP, Inc.
 * Author:yzq<yxj@rock-chips.com>
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <asm/div64.h>
#include <linux/uaccess.h>
#include <linux/rk_fb.h>
#include <linux/linux_logo.h>
#include <linux/dma-mapping.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <video/of_display_timing.h>
#include <video/display_timing.h>
#include <dt-bindings/rkfb/rk_fb.h>
#endif

#include <linux/display-sys.h>
#include "rk_drm_fb.h"
__weak int support_uboot_display(void)
{
	return 0;
}
static struct platform_device *drm_fb_pdev;
static struct rk_fb_trsm_ops *trsm_lvds_ops;
static struct rk_fb_trsm_ops *trsm_edp_ops;
static struct rk_fb_trsm_ops *trsm_mipi_ops;
static struct rk_display_device *disp_hdmi_devices;
static struct rk_lcdc_driver *prm_dev_drv;
static struct rk_lcdc_driver *ext_dev_drv;

static ssize_t show_disp_info(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	struct rk_lcdc_driver *dev_drv =
			rk_drm_priv->screen_priv[0].lcdc_dev_drv;

	if (dev_drv->ops->get_disp_info)
		return dev_drv->ops->get_disp_info(dev_drv, buf, 0);

	return 0;
}

static ssize_t show_dsp_bcsh(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	struct rk_lcdc_driver *dev_drv =
			rk_drm_priv->screen_priv[0].lcdc_dev_drv;
	int brightness = 0, contrast = 0, sat_con = 0, sin_hue = 0, cos_hue = 0;

	if (dev_drv->ops->get_dsp_bcsh_bcs) {
		brightness = dev_drv->ops->get_dsp_bcsh_bcs(dev_drv,
							    BRIGHTNESS);
		contrast = dev_drv->ops->get_dsp_bcsh_bcs(dev_drv, CONTRAST);
		sat_con = dev_drv->ops->get_dsp_bcsh_bcs(dev_drv, SAT_CON);
	}
	if (dev_drv->ops->get_dsp_bcsh_hue) {
		sin_hue = dev_drv->ops->get_dsp_bcsh_hue(dev_drv, H_SIN);
		cos_hue = dev_drv->ops->get_dsp_bcsh_hue(dev_drv, H_COS);
	}
	return snprintf(buf, PAGE_SIZE,
			"brightness:%4d,contrast:%4d,sat_con:%4d,"
			"sin_hue:%4d,cos_hue:%4d\n",
			brightness, contrast, sat_con, sin_hue, cos_hue);
}

static ssize_t set_dsp_bcsh(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	struct rk_lcdc_driver *dev_drv =
			rk_drm_priv->screen_priv[0].lcdc_dev_drv;
	int brightness, contrast, sat_con, ret = 0, sin_hue, cos_hue;

	if (!strncmp(buf, "open", 4)) {
		if (dev_drv->ops->open_bcsh)
			ret = dev_drv->ops->open_bcsh(dev_drv, 1);
		else
			ret = -1;
	} else if (!strncmp(buf, "close", 5)) {
		if (dev_drv->ops->open_bcsh)
			ret = dev_drv->ops->open_bcsh(dev_drv, 0);
		else
			ret = -1;
	} else if (!strncmp(buf, "brightness", 10)) {
		if (!sscanf(buf, "brightness %d", &brightness))
			return -EINVAL;
		if (unlikely(brightness > 255)) {
			dev_err(dev_drv->dev,
				"brightness should be [0:255],now=%d\n\n",
				brightness);
			brightness = 255;
		}
		if (dev_drv->ops->set_dsp_bcsh_bcs)
			ret = dev_drv->ops->set_dsp_bcsh_bcs(dev_drv,
							     BRIGHTNESS,
							     brightness);
		else
			ret = -1;
	} else if (!strncmp(buf, "contrast", 8)) {
		if (!sscanf(buf, "contrast %d", &contrast))
			return -EINVAL;
		if (unlikely(contrast > 510)) {
			dev_err(dev_drv->dev,
				"contrast should be [0:510],now=%d\n",
				contrast);
			contrast = 510;
		}
		if (dev_drv->ops->set_dsp_bcsh_bcs)
			ret = dev_drv->ops->set_dsp_bcsh_bcs(dev_drv,
							     CONTRAST,
							     contrast);
		else
			ret = -1;
	} else if (!strncmp(buf, "sat_con", 7)) {
		if (!sscanf(buf, "sat_con %d", &sat_con))
			return -EINVAL;
		if (unlikely(sat_con > 1015)) {
			dev_err(dev_drv->dev,
				"sat_con should be [0:1015],now=%d\n",
				sat_con);
			sat_con = 1015;
		}
		if (dev_drv->ops->set_dsp_bcsh_bcs)
			ret = dev_drv->ops->set_dsp_bcsh_bcs(dev_drv,
							     SAT_CON,
							     sat_con);
		else
			ret = -1;
	} else if (!strncmp(buf, "hue", 3)) {
		if (!sscanf(buf, "hue %d %d", &sin_hue, &cos_hue))
			return -EINVAL;
		if (unlikely(sin_hue > 511 || cos_hue > 511)) {
			dev_err(dev_drv->dev, "sin_hue=%d,cos_hue=%d\n",
				sin_hue, cos_hue);
		}
		if (dev_drv->ops->set_dsp_bcsh_hue)
			ret = dev_drv->ops->set_dsp_bcsh_hue(dev_drv,
							     sin_hue,
							     cos_hue);
		else
			ret = -1;
	} else {
		dev_info(dev, "format error\n");
	}

	if (ret < 0)
		return ret;

	return count;
}

static ssize_t set_dsp_lut(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	struct rk_lcdc_driver *dev_drv =
			rk_drm_priv->screen_priv[0].lcdc_dev_drv;
	int *dsp_lut = NULL;
	const char *start = buf;
	int i = 256, temp;
	int space_max = 10;

	dsp_lut = kzalloc(256 * 4, GFP_KERNEL);
	if (!dsp_lut)
		return -ENOMEM;
	for (i = 0; i < 256; i++) {
		temp = i;
		/*init by default value*/
		dsp_lut[i] = temp + (temp << 8) + (temp << 16);
	}
	/*printk("count:%d\n>>%s\n\n",count,start);*/
	for (i = 0; i < 256; i++) {
		space_max = 10;	/*max space number 10*/
		temp = simple_strtoul(start, NULL, 10);
		dsp_lut[i] = temp;
		do {
			start++;
			space_max--;
		} while ((*start != ' ') && space_max);

		if (!space_max)
			break;
		else
			start++;
	}
	if (dev_drv->ops->set_dsp_lut)
		dev_drv->ops->set_dsp_lut(dev_drv, dsp_lut);

	kfree(dsp_lut);
	return count;
}

static ssize_t show_fps(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	struct rk_lcdc_driver *dev_drv =
			rk_drm_priv->screen_priv[0].lcdc_dev_drv;
	int fps = 0;

	if (dev_drv->ops->fps_mgr)
		fps = dev_drv->ops->fps_mgr(dev_drv, 0, 0);
	if (fps < 0)
		return fps;

	return snprintf(buf, PAGE_SIZE, "fps:%d\n", fps);
}

static ssize_t set_fps(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	struct rk_lcdc_driver *dev_drv =
			rk_drm_priv->screen_priv[0].lcdc_dev_drv;
	u32 fps;
	int ret;

	ret = kstrtou32(buf, 0, &fps);
	if (ret)
		return ret;

	if (fps == 0 || fps > 60) {
		dev_info(dev, "unsupport fps value,pelase set 1~60\n");
		return count;
	}

	if (dev_drv->ops->fps_mgr)
		ret = dev_drv->ops->fps_mgr(dev_drv, fps, 1);
	if (ret < 0)
		return ret;

	return count;
}

static struct device_attribute rk_drm_fb_attrs[] = {
	__ATTR(disp_info, S_IRUGO, show_disp_info, NULL),
	__ATTR(bcsh, S_IRUGO | S_IWUSR, show_dsp_bcsh, set_dsp_bcsh),
	__ATTR(dsp_lut, S_IWUSR, NULL, set_dsp_lut),
	__ATTR(fps, S_IRUGO | S_IWUSR, show_fps, set_fps),
};

static int rk_drm_fb_create_sysfs(struct rk_lcdc_driver *dev_drv)
{
	int r, t;

	for (t = 0; t < ARRAY_SIZE(rk_drm_fb_attrs); t++) {
		r = device_create_file(dev_drv->dev, &rk_drm_fb_attrs[t]);
		if (r) {
			dev_err(dev_drv->dev, "failed to create sysfs file\n");
			return r;
		}
	}

	return 0;
}

void rk_drm_display_register(struct rk_display_ops *extend_ops,
			     void *displaydata, int type)
{
	switch (type) {
	case SCREEN_HDMI:
		disp_hdmi_devices = kzalloc(sizeof(struct rk_display_device),
					    GFP_KERNEL);
		disp_hdmi_devices->priv_data = displaydata;
		disp_hdmi_devices->ops = extend_ops;
		break;
	default:
		pr_info("%s:un supported extend display:%d!\n",
			__func__, type);
		break;
	}
}
int rk_fb_trsm_ops_register(struct rk_fb_trsm_ops *ops, int type)
{
	switch (type) {
	case SCREEN_RGB:
	case SCREEN_LVDS:
	case SCREEN_DUAL_LVDS:
		trsm_lvds_ops = ops;
		break;
	case SCREEN_EDP:
		trsm_edp_ops = ops;
		break;
	case SCREEN_MIPI:
	case SCREEN_DUAL_MIPI:
		trsm_mipi_ops = ops;
		break;
	default:
		pr_info("%s:un supported transmitter:%d!\n",
			__func__, type);
		break;
	}
	return 0;
}

struct rk_display_device *rk_drm_extend_display_get(int type)
{
	struct rk_display_device *extend_display = NULL;
	switch (type) {
	case SCREEN_HDMI:
		if (disp_hdmi_devices)
			extend_display = disp_hdmi_devices;
		else
			pr_info("%s:screen hdmi ops is NULL!\n", __func__);
		break;
	default:
		pr_info("%s:un supported extend display:%d!\n",
			__func__, type);
		break;
	}
	return extend_display;
}
/*
 * this two function is for other module that in the kernel which
 * need show image directly through fb
 * fb_id:we have 4 fb here,default we use fb0 for ui display
 */
struct fb_info *rk_get_fb(int fb_id)
{
	struct rk_fb *inf = platform_get_drvdata(drm_fb_pdev);
	struct fb_info *fb = inf->fb[fb_id];
	return fb;
}
EXPORT_SYMBOL(rk_get_fb);

int rk_fb_get_display_policy(void)
{
	struct rk_fb *rk_fb;

	if (drm_fb_pdev) {
		rk_fb = platform_get_drvdata(drm_fb_pdev);
		return rk_fb->disp_policy;
	} else {
		return DISPLAY_POLICY_SDK;
	}
}

struct rk_fb_trsm_ops *rk_fb_trsm_ops_get(int type)
{
	struct rk_fb_trsm_ops *ops;

	switch (type) {
	case SCREEN_RGB:
	case SCREEN_LVDS:
	case SCREEN_DUAL_LVDS:
		ops = trsm_lvds_ops;
		break;
	case SCREEN_EDP:
		ops = trsm_edp_ops;
		break;
	case SCREEN_MIPI:
	case SCREEN_DUAL_MIPI:
		ops = trsm_mipi_ops;
		break;
	default:
		ops = NULL;
		pr_info("%s:un supported transmitter:%d!\n",
			__func__, type);
		break;
	}
	return ops;
}
/* rk display power control parse from dts
 *
 */
int rk_disp_pwr_ctr_parse_dt(struct rk_lcdc_driver *dev_drv)
{
	struct device_node *root  = of_get_child_by_name(dev_drv->dev->of_node,
				"power_ctr");
	struct device_node *child;
	struct rk_disp_pwr_ctr_list *pwr_ctr;
	struct list_head *pos;
	enum of_gpio_flags flags;
	u32 val = 0;
	u32 debug = 0;
	u32 mirror = 0;
	int ret;

	INIT_LIST_HEAD(&dev_drv->pwrlist_head);
	if (!root) {
		dev_err(dev_drv->dev, "can't find power_ctr node for lcdc%d\n",
			dev_drv->id);
		return -ENODEV;
	}

	for_each_child_of_node(root, child) {
		pwr_ctr = kmalloc(sizeof(struct rk_disp_pwr_ctr_list), GFP_KERNEL);
		strcpy(pwr_ctr->pwr_ctr.name, child->name);
		if (!of_property_read_u32(child, "rockchip,power_type", &val)) {
			if (val == GPIO) {
				pwr_ctr->pwr_ctr.type = GPIO;
				pwr_ctr->pwr_ctr.gpio = of_get_gpio_flags(child, 0, &flags);
				if (!gpio_is_valid(pwr_ctr->pwr_ctr.gpio)) {
					dev_err(dev_drv->dev, "%s ivalid gpio\n", child->name);
					return -EINVAL;
				}
				pwr_ctr->pwr_ctr.atv_val = !(flags & OF_GPIO_ACTIVE_LOW);
				ret = gpio_request(pwr_ctr->pwr_ctr.gpio, child->name);
				if (ret) {
					dev_err(dev_drv->dev, "request %s gpio fail:%d\n",
						child->name, ret);
				}

			} else {
				pwr_ctr->pwr_ctr.type = REGULATOR;
			}
		};
		of_property_read_u32(child, "rockchip,delay", &val);
		pwr_ctr->pwr_ctr.delay = val;
		list_add_tail(&pwr_ctr->list, &dev_drv->pwrlist_head);
	}

	of_property_read_u32(root, "rockchip,mirror", &mirror);

	if (mirror == NO_MIRROR) {
		dev_drv->screen0->x_mirror = 0;
		dev_drv->screen0->y_mirror = 0;
	} else if (mirror == X_MIRROR) {
		dev_drv->screen0->x_mirror = 1;
		dev_drv->screen0->y_mirror = 0;
	} else if (mirror == Y_MIRROR) {
		dev_drv->screen0->x_mirror = 0;
		dev_drv->screen0->y_mirror = 1;
	} else if (mirror == X_Y_MIRROR) {
		dev_drv->screen0->x_mirror = 1;
		dev_drv->screen0->y_mirror = 1;
	}

	of_property_read_u32(root, "rockchip,debug", &debug);

	if (debug) {
		list_for_each(pos, &dev_drv->pwrlist_head) {
			pwr_ctr = list_entry(pos, struct rk_disp_pwr_ctr_list, list);
			printk(KERN_INFO "pwr_ctr_name:%s\n"
					 "pwr_type:%s\n"
					 "gpio:%d\n"
					 "atv_val:%d\n"
					 "delay:%d\n\n",
					 pwr_ctr->pwr_ctr.name,
					 (pwr_ctr->pwr_ctr.type == GPIO) ? "gpio" : "regulator",
					 pwr_ctr->pwr_ctr.gpio,
					 pwr_ctr->pwr_ctr.atv_val,
					 pwr_ctr->pwr_ctr.delay);
		}
	}

	return 0;
}

int rk_fb_video_mode_from_timing(const struct display_timing *dt,
				 struct rk_screen *screen)
{
	screen->mode.pixclock = dt->pixelclock.typ;
	screen->mode.left_margin = dt->hback_porch.typ;
	screen->mode.right_margin = dt->hfront_porch.typ;
	screen->mode.xres = dt->hactive.typ;
	screen->mode.hsync_len = dt->hsync_len.typ;
	screen->mode.upper_margin = dt->vback_porch.typ;
	screen->mode.lower_margin = dt->vfront_porch.typ;
	screen->mode.yres = dt->vactive.typ;
	screen->mode.vsync_len = dt->vsync_len.typ;
	screen->type = dt->screen_type;
	screen->lvds_format = dt->lvds_format;
	screen->face = dt->face;

	if (dt->flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
		screen->pin_dclk = 1;
	else
		screen->pin_dclk = 0;
	if (dt->flags & DISPLAY_FLAGS_HSYNC_HIGH)
		screen->pin_hsync = 1;
	else
		screen->pin_hsync = 0;
	if (dt->flags & DISPLAY_FLAGS_VSYNC_HIGH)
		screen->pin_vsync = 1;
	else
		screen->pin_vsync = 0;
	if (dt->flags & DISPLAY_FLAGS_DE_HIGH)
		screen->pin_den = 1;
	else
		screen->pin_den = 0;

	return 0;
}

int rk_fb_prase_timing_dt(struct device_node *np, struct rk_screen *screen)
{
	struct display_timings *disp_timing;
	struct display_timing *dt;
	disp_timing = of_get_display_timings(np);

	if (!disp_timing) {
		pr_err("parse display timing err\n");
		return -EINVAL;
	}
	dt = display_timings_get(disp_timing, 0);
	rk_fb_video_mode_from_timing(dt, screen);
	printk(KERN_ERR "dclk:%d\n"
			 "hactive:%d\n"
			 "hback_porch:%d\n"
			 "hfront_porch:%d\n"
			 "hsync_len:%d\n"
			 "vactive:%d\n"
			 "vback_porch:%d\n"
			 "vfront_porch:%d\n"
			 "vsync_len:%d\n"
			 "screen_type:%d\n"
			 "lvds_format:%d\n"
			 "face:%d\n",
			dt->pixelclock.typ,
			dt->hactive.typ,
			dt->hback_porch.typ,
			dt->hfront_porch.typ,
			dt->hsync_len.typ,
			dt->vactive.typ,
			dt->vback_porch.typ,
			dt->vfront_porch.typ,
			dt->vsync_len.typ,
			dt->screen_type,
			dt->lvds_format,
			dt->face);
	return 0;
}

static int init_lcdc_win(struct rk_lcdc_driver *dev_drv,
			 struct rk_lcdc_win *def_win)
{
	int i;
	int lcdc_win_num = dev_drv->lcdc_win_num;

	for (i = 0; i < lcdc_win_num; i++) {
		struct rk_lcdc_win *win = NULL;

		win = kzalloc(sizeof(*win), GFP_KERNEL);
		if (!win) {
			dev_err(dev_drv->dev, "kzmalloc for win fail!");
			return   -ENOMEM;
		}

		strcpy(win->name, def_win[i].name);
		win->id = def_win[i].id;
		win->support_3d = def_win[i].support_3d;
		dev_drv->win[i] = win;
	}

	return 0;
}

static int init_lcdc_device_driver(struct rk_drm_screen_private *screen_priv,
				   struct rk_lcdc_win *def_win, int index)
{
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	struct rk_lcdc_driver *dev_drv = screen_priv->lcdc_dev_drv;
	struct rk_screen *screen1 = NULL;
	struct rk_screen *screen = devm_kzalloc(dev_drv->dev,
				sizeof(struct rk_screen), GFP_KERNEL);

	if (!screen) {
		dev_err(dev_drv->dev, "malloc screen for lcdc%d fail!",
			dev_drv->id);
		goto fail_screen;
	}
	screen->screen_id = 0;
	screen->lcdc_id = dev_drv->id;
	screen->overscan.left = 100;
	screen->overscan.top = 100;
	screen->overscan.right = 100;
	screen->overscan.bottom = 100;
	dev_drv->screen0 = screen;
	dev_drv->cur_screen = screen;
	/* devie use one lcdc + rk61x scaler for dual display*/
	if (rk_drm_priv->disp_mode == ONE_DUAL) {
		screen1 = devm_kzalloc(dev_drv->dev,
				       sizeof(struct rk_screen), GFP_KERNEL);
		if (!screen1) {
			dev_err(dev_drv->dev, "malloc screen1 for lcdc%d fail!",
				dev_drv->id);
			goto fail_screen1;
		}
		screen1->screen_id = 1;
		screen1->lcdc_id = 1;
		dev_drv->screen1 = screen1;
	}
	sprintf(dev_drv->name, "lcdc%d", dev_drv->id);
	init_lcdc_win(dev_drv, def_win);
	init_completion(&dev_drv->frame_done);
	spin_lock_init(&dev_drv->cpl_lock);
	mutex_init(&dev_drv->fb_win_id_mutex);
	dev_drv->ops->fb_win_remap(dev_drv, FB_DEFAULT_ORDER);
	dev_drv->first_frame = 1;
	rk_disp_pwr_ctr_parse_dt(dev_drv);
	if (dev_drv->prop == PRMRY) {
		rk_fb_set_prmry_screen(screen);
		rk_fb_get_prmry_screen(screen);
		dev_drv->trsm_ops = rk_fb_trsm_ops_get(screen->type);
	}

	return 0;

fail_screen1:
	devm_kfree(dev_drv->dev, screen);
fail_screen:
	return -ENOMEM;
}
int rk_disp_pwr_enable(struct rk_lcdc_driver *dev_drv)
{
	struct list_head *pos;
	struct rk_disp_pwr_ctr_list *pwr_ctr_list;
	struct pwr_ctr *pwr_ctr;

	if (list_empty(&dev_drv->pwrlist_head))
		return 0;
	list_for_each(pos, &dev_drv->pwrlist_head) {
		pwr_ctr_list = list_entry(pos,
					  struct rk_disp_pwr_ctr_list, list);
		pwr_ctr = &pwr_ctr_list->pwr_ctr;
		if (pwr_ctr->type == GPIO) {
			gpio_direction_output(pwr_ctr->gpio, pwr_ctr->atv_val);
			mdelay(pwr_ctr->delay);
		}
	}

	return 0;
}
int rk_fb_calc_fps(struct rk_screen *screen, u32 pixclock)
{
	int x, y;
	unsigned long long hz;

	if (!screen) {
		pr_err("%s:null screen!\n", __func__);
		return 0;
	}
	x = screen->mode.xres + screen->mode.left_margin +
		screen->mode.right_margin + screen->mode.hsync_len;
	y = screen->mode.yres + screen->mode.upper_margin +
		screen->mode.lower_margin + screen->mode.vsync_len;

	hz = 1000000000000ULL;	/* 1e12 picoseconds per second */

	hz += (x * y) / 2;
	do_div(hz, x * y);	/* divide by x * y with rounding */

	hz += pixclock / 2;
	do_div(hz, pixclock);	/* divide by pixclock with rounding */

	return hz;
}

char *get_format_string(enum data_format format, char *fmt)
{
	if (!fmt)
		return NULL;
	switch (format) {
	case ARGB888:
		strcpy(fmt, "ARGB888");
		break;
	case RGB888:
		strcpy(fmt, "RGB888");
		break;
	case RGB565:
		strcpy(fmt, "RGB565");
		break;
	case YUV420:
		strcpy(fmt, "YUV420");
		break;
	case YUV422:
		strcpy(fmt, "YUV422");
		break;
	case YUV444:
		strcpy(fmt, "YUV444");
		break;
	case XRGB888:
		strcpy(fmt, "XRGB888");
		break;
	case XBGR888:
		strcpy(fmt, "XBGR888");
		break;
	case ABGR888:
		strcpy(fmt, "XBGR888");
		break;
	default:
		strcpy(fmt, "invalid");
		break;
	}

	return fmt;
}

/*
 * this is for hdmi
 * name: lcdc device name ,lcdc0 , lcdc1
 */
struct rk_lcdc_driver *rk_get_lcdc_drv(char *name)
{
	struct rk_drm_private  *rk_drm_priv = NULL;
	struct rk_lcdc_driver *dev_drv = NULL;
	int i = 0;

	if (likely(drm_fb_pdev))
		rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	else
		return NULL;

	for (i = 0; i < rk_drm_priv->num_lcdc; i++) {
		if (!strcmp(rk_drm_priv->lcdc_dev_drv[i]->name, name)) {
			dev_drv = rk_drm_priv->lcdc_dev_drv[i];
			break;
		}
	}

	return dev_drv;
}

static struct rk_lcdc_driver *rk_get_prmry_lcdc_drv(void)
{
	struct rk_fb *inf = NULL;

	if (likely(drm_fb_pdev))
		inf = platform_get_drvdata(drm_fb_pdev);
	else
		return NULL;

	return prm_dev_drv;
}

static __maybe_unused struct rk_lcdc_driver *rk_get_extend_lcdc_drv(void)
{
	struct rk_fb *inf = NULL;

	if (likely(drm_fb_pdev))
		inf = platform_get_drvdata(drm_fb_pdev);
	else
		return NULL;

	return ext_dev_drv;
}

/*
 * get one frame time of the prmry screen, unit: us
 */
u32 rk_fb_get_prmry_screen_ft(void)
{
	struct rk_lcdc_driver *dev_drv = rk_get_prmry_lcdc_drv();
	u32 htotal, vtotal, pixclock_ps;
	u64 pix_total, ft_us;

	if (unlikely(!dev_drv))
		return 0;

	pixclock_ps = dev_drv->pixclock;

	vtotal = dev_drv->cur_screen->mode.upper_margin +
		 dev_drv->cur_screen->mode.lower_margin +
		 dev_drv->cur_screen->mode.yres +
		 dev_drv->cur_screen->mode.vsync_len;
	htotal = dev_drv->cur_screen->mode.left_margin +
		 dev_drv->cur_screen->mode.right_margin +
		 dev_drv->cur_screen->mode.xres +
		 dev_drv->cur_screen->mode.hsync_len;
	pix_total = htotal * vtotal;
	ft_us = pix_total * pixclock_ps;
	do_div(ft_us, 1000000);
	if (dev_drv->frame_time.ft == 0)
		dev_drv->frame_time.ft = ft_us;

	ft_us = dev_drv->frame_time.framedone_t -
			dev_drv->frame_time.last_framedone_t;
	do_div(ft_us, 1000);
	ft_us = min(dev_drv->frame_time.ft, (u32)ft_us);
	if (ft_us != 0)
		dev_drv->frame_time.ft = ft_us;

	return dev_drv->frame_time.ft;
}

/*
 * get the vblanking time of the prmry screen, unit: us
 */
u32 rk_fb_get_prmry_screen_vbt(void)
{
	struct rk_lcdc_driver *dev_drv = rk_get_prmry_lcdc_drv();
	u32 htotal, vblank, pixclock_ps;
	u64 pix_blank, vbt_us;

	if (unlikely(!dev_drv))
		return 0;

	pixclock_ps = dev_drv->pixclock;

	htotal = (dev_drv->cur_screen->mode.left_margin +
		  dev_drv->cur_screen->mode.right_margin +
		  dev_drv->cur_screen->mode.xres +
		  dev_drv->cur_screen->mode.hsync_len);
	vblank = (dev_drv->cur_screen->mode.upper_margin +
		  dev_drv->cur_screen->mode.lower_margin +
		  dev_drv->cur_screen->mode.vsync_len);
	pix_blank = htotal * vblank;
	vbt_us = pix_blank * pixclock_ps;
	do_div(vbt_us, 1000000);
	return (u32)vbt_us;
}

/*
 * get the frame done time of the prmry screen, unit: us
 */
u64 rk_fb_get_prmry_screen_framedone_t(void)
{
	struct rk_lcdc_driver *dev_drv = rk_get_prmry_lcdc_drv();

	if (unlikely(!dev_drv))
		return 0;
	else
		return dev_drv->frame_time.framedone_t;
}

/*
 * set prmry screen status
 */
int rk_fb_set_prmry_screen_status(int status)
{
	struct rk_lcdc_driver *dev_drv = rk_get_prmry_lcdc_drv();
	struct rk_screen *screen;

	if (unlikely(!dev_drv))
		return 0;

	screen = dev_drv->cur_screen;
	switch (status) {
	case SCREEN_PREPARE_DDR_CHANGE:
		if (screen->type == SCREEN_MIPI ||
		    screen->type == SCREEN_DUAL_MIPI) {
			if (dev_drv->trsm_ops->dsp_pwr_off)
				dev_drv->trsm_ops->dsp_pwr_off();
		}
		break;
	case SCREEN_UNPREPARE_DDR_CHANGE:
		if (screen->type == SCREEN_MIPI ||
		    screen->type == SCREEN_DUAL_MIPI) {
			if (dev_drv->trsm_ops->dsp_pwr_on)
				dev_drv->trsm_ops->dsp_pwr_on();
		}
		break;
	default:
		break;
	}

	return 0;
}

u32 rk_fb_get_prmry_screen_pixclock(void)
{
	struct rk_lcdc_driver *dev_drv = rk_get_prmry_lcdc_drv();

	if (unlikely(!dev_drv))
		return 0;
	else
		return dev_drv->pixclock;
}

int rk_fb_poll_prmry_screen_vblank(void)
{
	struct rk_lcdc_driver *dev_drv = rk_get_prmry_lcdc_drv();

	if (likely(dev_drv)) {
		if (dev_drv->ops->poll_vblank)
			return dev_drv->ops->poll_vblank(dev_drv);
		else
			return RK_LF_STATUS_NC;
	} else {
		return RK_LF_STATUS_NC;
	}
}

bool rk_fb_poll_wait_frame_complete(void)
{
	uint32_t timeout = RK_LF_MAX_TIMEOUT;
	struct rk_lcdc_driver *dev_drv = rk_get_prmry_lcdc_drv();

	if (likely(dev_drv)) {
		if (dev_drv->ops->set_irq_to_cpu)
			dev_drv->ops->set_irq_to_cpu(dev_drv, 0);
	}

	if (rk_fb_poll_prmry_screen_vblank() == RK_LF_STATUS_NC) {
		if (likely(dev_drv)) {
			if (dev_drv->ops->set_irq_to_cpu)
				dev_drv->ops->set_irq_to_cpu(dev_drv, 1);
		}
		return false;
	}
	while (!(rk_fb_poll_prmry_screen_vblank() == RK_LF_STATUS_FR) && --timeout)
		;
	while (!(rk_fb_poll_prmry_screen_vblank() == RK_LF_STATUS_FC) && --timeout)
		;
	if (likely(dev_drv)) {
		if (dev_drv->ops->set_irq_to_cpu)
			dev_drv->ops->set_irq_to_cpu(dev_drv, 1);
	}

	return true;
}

/* rk_fb_get_sysmmu_device_by_compatible()
 * @compt: dts device compatible name
 * return value: success: pointer to the device inside of platform device
 *               fail: NULL
 */
struct device *rk_fb_get_sysmmu_device_by_compatible(const char *compt)
{
	struct device_node *dn = NULL;
	struct platform_device *pd = NULL;
	struct device *ret = NULL;

	dn = of_find_compatible_node(NULL, NULL, compt);
	if (!dn) {
		pr_info("can't find device node %s \r\n", compt);
		return NULL;
	}

	pd = of_find_device_by_node(dn);
	if (!pd) {
		pr_info("can't find platform device node %s \r\n", compt);
		return  NULL;
	}
	ret = &pd->dev;

	return ret;
}

#ifdef CONFIG_IOMMU_API
void rk_fb_platform_set_sysmmu(struct device *sysmmu, struct device *dev)
{
	dev->archdata.iommu = sysmmu;
}
#else
void rk_fb_platform_set_sysmmu(struct device *sysmmu, struct device *dev)
{
}
#endif

int rk_disp_pwr_disable(struct rk_lcdc_driver *dev_drv)
{
	struct list_head *pos;
	struct rk_disp_pwr_ctr_list *pwr_ctr_list;
	struct pwr_ctr *pwr_ctr;

	if (list_empty(&dev_drv->pwrlist_head))
		return 0;
	list_for_each(pos, &dev_drv->pwrlist_head) {
		pwr_ctr_list = list_entry(pos,
					  struct rk_disp_pwr_ctr_list, list);
		pwr_ctr = &pwr_ctr_list->pwr_ctr;
		if (pwr_ctr->type == GPIO)
			gpio_set_value(pwr_ctr->gpio, pwr_ctr->atv_val);
	}

	return 0;
}
/********************************
*check if the primary lcdc has registerd,
the primary lcdc mas register first
*********************************/
bool is_prmry_rk_lcdc_registered(void)
{
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);

	if (rk_drm_priv->screen_priv[0].lcdc_dev_drv)
		return  true;
	else
		return false;
}
static void rk_fb_update_regs_handler(struct kthread_work *work)
{
}
static int rk_fb_wait_for_vsync_thread(void *data)
{
	struct rk_lcdc_driver  *dev_drv = data;
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	struct rk_drm_screen_private *drm_screen_priv = NULL;
	struct rk_drm_display *drm_display = NULL;

	if (dev_drv->prop == PRMRY)
		drm_screen_priv = &rk_drm_priv->screen_priv[0];
	else if (dev_drv->prop == EXTEND)
		drm_screen_priv = &rk_drm_priv->screen_priv[1];
	if (drm_screen_priv == NULL)
		return -1;
	drm_display = &drm_screen_priv->drm_disp;

	while (!kthread_should_stop()) {
		ktime_t timestamp = dev_drv->vsync_info.timestamp;
		int ret = wait_event_interruptible(dev_drv->vsync_info.wait,
			!ktime_equal(timestamp, dev_drv->vsync_info.timestamp) &&
			(dev_drv->vsync_info.active || dev_drv->vsync_info.irq_stop));
#if 1
		if (atomic_read(&drm_screen_priv->wait_vsync_done)) {
			atomic_set(&drm_screen_priv->wait_vsync_done, 0);
			wake_up(&drm_screen_priv->wait_vsync_queue);
		}
		if (!ret && drm_display->event_call_back)
			drm_display->event_call_back(drm_display, 0,
						     RK_DRM_CALLBACK_VSYNC);
#endif
	}

	return 0;
}

static void rk_drm_irq_handle(struct rk_lcdc_driver *dev_drv)
{
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	struct rk_drm_screen_private *drm_screen_priv = NULL;
	struct rk_drm_display *drm_display = NULL;

	if (dev_drv->prop == PRMRY)
		drm_screen_priv = &rk_drm_priv->screen_priv[0];
	else if (dev_drv->prop == EXTEND)
		drm_screen_priv = &rk_drm_priv->screen_priv[1];
	if (drm_screen_priv == NULL)
		return;
	drm_display = &drm_screen_priv->drm_disp;
	if (atomic_read(&drm_screen_priv->wait_vsync_done)) {
		atomic_set(&drm_screen_priv->wait_vsync_done, 0);
		wake_up(&drm_screen_priv->wait_vsync_queue);
	}

	if (drm_display->event_call_back)
		drm_display->event_call_back(drm_display, 0,
					     RK_DRM_CALLBACK_VSYNC);
}
int rk_fb_register(struct rk_lcdc_driver *dev_drv,
		   struct rk_lcdc_win *win, int id)
{
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	struct rk_drm_display *drm_display = NULL;
	struct rk_drm_screen_private *drm_screen_priv = NULL;
	int i = 0;

	if (rk_drm_priv->num_screen == RK30_MAX_LCDC_SUPPORT)
		return -ENXIO;
	for (i = 0; i < RK_DRM_MAX_SCREEN_NUM; i++) {
		if (!rk_drm_priv->screen_priv[i].lcdc_dev_drv) {
			rk_drm_priv->lcdc_dev_drv[i] = dev_drv;
			rk_drm_priv->lcdc_dev_drv[i]->id = id;
			rk_drm_priv->num_lcdc++;
			break;
		}
	}

	rk_drm_priv->num_screen++;
	drm_screen_priv = &rk_drm_priv->screen_priv[i];
	drm_screen_priv->lcdc_dev_drv = dev_drv;
	drm_screen_priv->lcdc_dev_drv->id = id;

	init_lcdc_device_driver(drm_screen_priv, win, i);
	dev_drv->irq_call_back = rk_drm_irq_handle;

	drm_display = &drm_screen_priv->drm_disp;
	drm_display->num_win = dev_drv->lcdc_win_num;
	atomic_set(&drm_screen_priv->wait_vsync_done, 1);
	init_waitqueue_head(&drm_screen_priv->wait_vsync_queue);
	drm_display->vop_dev = dev_drv->dev;
	drm_display->iommu_en = dev_drv->iommu_enabled;
	if (dev_drv->prop == PRMRY) {
		struct fb_modelist *modelist_new;
		struct fb_modelist *modelist;
		struct fb_videomode *mode;

		prm_dev_drv = dev_drv;
		drm_display->modelist = kmalloc(sizeof(struct list_head),
						GFP_KERNEL);
		INIT_LIST_HEAD(drm_display->modelist);
		modelist_new = kmalloc(sizeof(struct fb_modelist),
				       GFP_KERNEL);
		drm_display->screen_type = RK_DRM_PRIMARY_SCREEN;
		drm_display->num_videomode = 1;
		drm_display->best_mode = 0;
		drm_display->is_connected = 1;
		memcpy(&modelist_new->mode, &dev_drv->cur_screen->mode,
		       sizeof(struct fb_videomode));
		list_add_tail(&modelist_new->list, drm_display->modelist);
		modelist = list_first_entry(drm_display->modelist,
					    struct fb_modelist, list);
		mode = &modelist->mode;

		rk_drm_fb_create_sysfs(dev_drv);
		if (dev_drv->ops->open)
			dev_drv->ops->open(dev_drv, 0, 1);
		if (dev_drv->ops->mmu_en)
			dev_drv->ops->mmu_en(dev_drv);
	} else if (dev_drv->prop == EXTEND) {
		/* struct list_head *modelist;*/
		ext_dev_drv = dev_drv;
		drm_screen_priv->ex_display = rk_drm_extend_display_get(SCREEN_HDMI);
		drm_display->screen_type = RK_DRM_EXTEND_SCREEN;
		drm_display->is_connected = 0;
#if 0
		drm_screen_priv->ex_display->ops->getmodelist(drm_screen_priv->ex_display, &modelist);
		memcpy(&drm_display->modelist, modelist, sizeof(struct list_head));
		drm_display->is_connected = drm_screen_priv->ex_display->ops->getstatus(drm_screen_priv->ex_display);
#endif
	}
	if (1) {
		/*dev_drv->prop == PRMRY) {*/
		init_waitqueue_head(&dev_drv->vsync_info.wait);
		dev_drv->vsync_info.thread =
				kthread_run(rk_fb_wait_for_vsync_thread,
					    dev_drv, "fb-vsync");
		if (dev_drv->vsync_info.thread == ERR_PTR(-ENOMEM)) {
			pr_info("failed to run vsync thread\n");
			dev_drv->vsync_info.thread = NULL;
		}
		dev_drv->vsync_info.active = 1;

		mutex_init(&dev_drv->output_lock);

		INIT_LIST_HEAD(&dev_drv->update_regs_list);
		mutex_init(&dev_drv->update_regs_list_lock);
		init_kthread_worker(&dev_drv->update_regs_worker);

		dev_drv->update_regs_thread = kthread_run(kthread_worker_fn,
				&dev_drv->update_regs_worker, "rk-fb");
		if (IS_ERR(dev_drv->update_regs_thread)) {
			int err = PTR_ERR(dev_drv->update_regs_thread);

			dev_drv->update_regs_thread = NULL;
			pr_info("failed to run update_regs thread\n");
			return err;
		}
		init_kthread_work(&dev_drv->update_regs_work,
				  rk_fb_update_regs_handler);
		dev_drv->timeline = sw_sync_timeline_create("fb-timeline");
		dev_drv->timeline_max = 1;
	}
	return 0;
}
int rk_fb_unregister(struct rk_lcdc_driver *dev_drv)

{
	/*
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	int i = 0;
	*/
	return 0;
}

struct rk_drm_display *rk_drm_get_info(int screen_type)
{
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	int i = 0;

	for (i = 0; i < rk_drm_priv->num_screen; i++) {
		if (rk_drm_priv->screen_priv[i].drm_disp.screen_type == screen_type)
			break;
	}
	if (i == rk_drm_priv->num_screen) {
		pr_info("%s can not find match DISPLAY_TYPE %d\n",
			__func__, screen_type);
		return NULL;
	}

	return &rk_drm_priv->screen_priv[i].drm_disp;
}

static int rk_drm_screen_blank(struct rk_drm_display *drm_disp)
{
	struct rk_drm_screen_private *drm_screen_priv =
		container_of(drm_disp, struct rk_drm_screen_private, drm_disp);
	struct rk_lcdc_driver *dev_drv = drm_screen_priv->lcdc_dev_drv;

	dev_drv->ops->blank(dev_drv, 0, drm_disp->enable ? FB_BLANK_UNBLANK : FB_BLANK_NORMAL);

	return 0;
}

int rk_fb_disp_scale(u8 scale_x, u8 scale_y, u8 lcdc_id)
{
	return 0;
}

int rk_fb_switch_screen(struct rk_screen *screen , int enable, int lcdc_id)
{
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);
	struct rk_lcdc_driver *dev_drv = NULL;
	struct rk_drm_display *drm_disp = NULL;
	char name[6];
	int i;

	sprintf(name, "lcdc%d", lcdc_id);

	for (i = 0; i < rk_drm_priv->num_screen; i++) {
		if (rk_drm_priv->screen_priv[i].lcdc_dev_drv->prop == EXTEND) {
			drm_disp = &rk_drm_priv->screen_priv[i].drm_disp;
			dev_drv = rk_drm_priv->screen_priv[i].lcdc_dev_drv;
			break;
		}
	}

	if (i == rk_drm_priv->num_screen) {
		pr_err("%s driver not found!", name);
		return -ENODEV;
	}

	pr_info("hdmi %s lcdc%d\n", enable ? "connect to" : "remove from",
		dev_drv->id);

	if (enable && !drm_disp->is_connected) {
		struct list_head *modelist;
		struct rk_display_device *ex_display =
				rk_drm_priv->screen_priv[i].ex_display;

		memcpy(dev_drv->cur_screen, screen, sizeof(struct rk_screen));
		if (ex_display == NULL)
			ex_display = rk_drm_extend_display_get(SCREEN_HDMI);
		rk_drm_priv->screen_priv[i].ex_display = ex_display;
		ex_display->ops->getmodelist(ex_display, &modelist);
		drm_disp->modelist = modelist;

		drm_disp->is_connected = true;
		drm_disp->event_call_back(drm_disp, 0, RK_DRM_CALLBACK_HOTPLUG);
	} else {
		drm_disp->is_connected = false;
		drm_disp->event_call_back(drm_disp, 0, RK_DRM_CALLBACK_HOTPLUG);
	}

	return 0;
}
static int rk_drm_screen_videomode_set(struct rk_drm_display *drm_disp)
{
	struct rk_drm_screen_private *drm_screen_priv =
		container_of(drm_disp, struct rk_drm_screen_private, drm_disp);
	struct rk_lcdc_driver *dev_drv = drm_screen_priv->lcdc_dev_drv;
	struct fb_videomode *mode = drm_disp->mode;
	if (!mode) {
		pr_err("%s fb_video mode is NULL", __func__);
		return -1;
	}

	if (dev_drv->prop == PRMRY) {
		if (mode != &dev_drv->cur_screen->mode)
			memcpy(&dev_drv->cur_screen->mode, mode,
			       sizeof(struct fb_videomode));

	} else {
		struct rk_display_device *ex_display =
						drm_screen_priv->ex_display;
		if (ex_display == NULL)
			ex_display = rk_drm_extend_display_get(SCREEN_HDMI);

		if (ex_display == NULL) {
			pr_err("%s can't find extend display ops\n", __func__);
			return -1;
		}
		ex_display->ops->setmode(ex_display, mode);
	}
	if (!dev_drv->atv_layer_cnt) {
		pr_info("%s[%d],atv_layer_cnt:%d close vop%d\n",
			__func__, __LINE__, dev_drv->atv_layer_cnt, dev_drv->id);
		dev_drv->ops->open(dev_drv, 0, true);
	}

	dev_drv->ops->ovl_mgr(dev_drv, 3210, 1);
	dev_drv->ops->load_screen(dev_drv, 1);

	return 0;
}

static int rk_drm_win_commit(struct rk_drm_display *drm_disp,
			     unsigned int win_id)
{
	struct rk_drm_screen_private *drm_screen_priv =
		container_of(drm_disp, struct rk_drm_screen_private, drm_disp);
	struct rk_lcdc_driver *dev_drv = drm_screen_priv->lcdc_dev_drv;
	unsigned int i = 0, j = 0;

	for (i = 1; i < RK_DRM_WIN_MASK; i = i << 1) {
		if (i & win_id) {
			struct rk_lcdc_win *lcdc_win = dev_drv->win[j];
			struct rk_win_data *drm_win = &drm_disp->win[j];
			if (!lcdc_win && !drm_win) {
				pr_err("---->%s can not find display win%d\n",
				       __func__, j);
				return -1;
			}

			lcdc_win->area[0].format = drm_win->format;

			lcdc_win->area[0].xpos = drm_win->xpos;
			lcdc_win->area[0].ypos = drm_win->ypos;
			lcdc_win->area[0].xsize = drm_win->xsize;
			lcdc_win->area[0].ysize = drm_win->ysize;

			lcdc_win->area[0].xact = drm_win->xact;
			lcdc_win->area[0].yact = drm_win->yact;
			lcdc_win->area[0].xvir = drm_win->xvir;
			lcdc_win->area[0].y_vir_stride = drm_win->xvir;
			lcdc_win->area[0].uv_vir_stride = drm_win->uv_vir;
			lcdc_win->area[0].smem_start = drm_win->yrgb_addr;
			lcdc_win->area[0].cbr_start = drm_win->uv_addr;
			lcdc_win->alpha_en = 1;
			if (lcdc_win->state != drm_win->enabled) {
				pr_info("%s[%d], win:%d, drm_win->enabled:%d, lcdc_win->state:%d\n",
					__func__, __LINE__, j, drm_win->enabled, lcdc_win->state);
				dev_drv->ops->open(dev_drv, j, drm_win->enabled ? true : false);
			}
			dev_drv->ops->set_par(dev_drv, j);
			dev_drv->ops->pan_display(dev_drv, j);
		}
		j++;
	}
	return 0;
}

static int rk_drm_display_commit(struct rk_drm_display *drm_disp)
{
	struct rk_drm_screen_private *drm_screen_priv =
		container_of(drm_disp, struct rk_drm_screen_private, drm_disp);
	struct rk_lcdc_driver *dev_drv = drm_screen_priv->lcdc_dev_drv;

	if (dev_drv->ops->lcdc_reg_update)
		dev_drv->ops->lcdc_reg_update(dev_drv);
	if (dev_drv->ops->cfg_done)
		dev_drv->ops->cfg_done(dev_drv);
	return 0;
}

int rk_drm_disp_handle(struct rk_drm_display *drm_disp, unsigned int win_id,
		       unsigned int cmd_id)
{
	struct rk_drm_screen_private *drm_screen_priv =
		container_of(drm_disp, struct rk_drm_screen_private, drm_disp);
	int i = 0;

	for (i = 1; i < RK_DRM_CMD_MASK; i = i << 1) {
		switch (i&cmd_id) {
		case RK_DRM_SCREEN_SET:
			rk_drm_screen_videomode_set(drm_disp);
			break;
		case RK_DRM_SCREEN_BLANK:
			rk_drm_screen_blank(drm_disp);
			break;
		case RK_DRM_WIN_COMMIT:
			rk_drm_win_commit(drm_disp, win_id);
			break;
		case RK_DRM_DISPLAY_COMMIT:
			if (win_id & i) {
				if (!wait_event_timeout(drm_screen_priv->wait_vsync_queue,
							!atomic_read(&drm_screen_priv->wait_vsync_done),
							HZ/20))
				pr_info("wait frame timed out.\n");
			}

			rk_drm_display_commit(drm_disp);
			if (win_id & i)
				atomic_set(&drm_screen_priv->wait_vsync_done,
					   1);
			break;
		}
	}
	return 0;
}

struct rk_drm_display *rk_drm_get_diplay(int screen_type)
{
	struct rk_drm_private *rk_drm_priv = platform_get_drvdata(drm_fb_pdev);

	int i = 0;
	for (i = 0; i < rk_drm_priv->num_screen; i++) {
		if (rk_drm_priv->screen_priv[i].drm_disp.screen_type == screen_type)
			return &rk_drm_priv->screen_priv[i].drm_disp;
	}

	return NULL;
}

int rk_fb_set_car_reverse_status(struct rk_lcdc_driver *dev_drv,
				 int status)
{
	return 0;
}

static int rk_drm_fb_probe(struct platform_device *pdev)
{
	struct rk_drm_private  *rk_drm_priv = NULL;
	struct device_node *np = pdev->dev.of_node;
	u32 mode;

	if (!np) {
		dev_err(&pdev->dev, "Missing device tree node.\n");
		return -EINVAL;
	}

	rk_drm_priv = devm_kzalloc(&pdev->dev,
				   sizeof(struct rk_drm_private), GFP_KERNEL);
	if (!rk_drm_priv) {
		dev_err(&pdev->dev, "kmalloc for rk_drm_priv fail!");
		return  -ENOMEM;
	}
	platform_set_drvdata(pdev, rk_drm_priv);

	if (!of_property_read_u32(np, "rockchip,disp-mode", &mode)) {
		rk_drm_priv->disp_mode = mode;
	} else {
		dev_err(&pdev->dev, "no disp-mode node found!");
		return -ENODEV;
	}
	dev_set_name(&pdev->dev, "rockchip-drmfb");

	drm_fb_pdev = pdev;
	dev_info(&pdev->dev, "rockchip drm framebuffer driver probe\n");
	return 0;
}

static int rk_drm_fb_remove(struct platform_device *pdev)
{
	struct rk_drm_private  *rk_drm_priv = platform_get_drvdata(pdev);

	kfree(rk_drm_priv);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void rk_drm_fb_shutdown(struct platform_device *pdev)
{
}


static const struct of_device_id rk_drm_fb_dt_ids[] = {
	{.compatible = "rockchip,rk-fb", },
	{}
};

static struct platform_driver rk_drm_fb_driver = {
	.probe		= rk_drm_fb_probe,
	.remove		= rk_drm_fb_remove,
	.driver		= {
		.name	= "rk-fb",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(rk_drm_fb_dt_ids),
	},
	.shutdown   = rk_drm_fb_shutdown,
};

static int __init rk_drm_fb_init(void)
{
	return platform_driver_register(&rk_drm_fb_driver);
}

static void __exit rk_drm_fb_exit(void)
{
	platform_driver_unregister(&rk_drm_fb_driver);
}

fs_initcall(rk_drm_fb_init);
module_exit(rk_drm_fb_exit);
