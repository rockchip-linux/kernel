/*
**************************************************************************
 * Rockchip driver for CIF CIF 1.0
 * (Based on Intel driver for sofiaxxx)
 *
 * Copyright (C) 2015 Intel Mobile Communications GmbH
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
**************************************************************************
 */

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include <linux/platform_data/rk_cif10_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include "cif_cvbsin.h"

#define CVBSIN_CTRL2    (0x01 * 4)
#define CVBSIN_CTRL3    (0x02 * 4)
#define CVBSIN_CTRL4    (0x03 * 4)
#define CVBSIN_STATUS   (0x1f * 4)
#define CVBSIN_OUTCTRL  (0x20 * 4)

#define FCOMB_BYPASS    (0x1 << 6)
#define FCOMB_DISABLE   (0x1 << 5)
#define COMB_MODE_SE    (0x4)
#define CTRL2_VAL       (FCOMB_BYPASS | FCOMB_DISABLE | COMB_MODE_SE)
#define TWOS_COMPLEMENT (0x1 << 1)
#define CTRL3_VAL       (TWOS_COMPLEMENT)
#define AGC		(0x1 << 4)
#define ABL		(0x1 << 3)
#define CTRL4_VAL       (AGC | ABL | 0x3)

#define RST_DISABLE     (0x0)
#define RST_ENABLE      (0x1)

#define OF_CAMERA_MODULE_DEFRECT0 "rockchip,camera-module-defrect0"
#define OF_CAMERA_MODULE_DEFRECT1 "rockchip,camera-module-defrect1"
#define OF_CAMERA_MODULE_DEFRECT2 "rockchip,camera-module-defrect2"
#define OF_CAMERA_MODULE_DEFRECT3 "rockchip,camera-module-defrect3"

struct cvbsin_module_config {
	const char *name;
	struct v4l2_mbus_framefmt frm_fmt;
	struct v4l2_subdev_frame_interval frm_intrvl;

	struct pltfrm_cam_itf itf_cfg;
};

struct cif_cvbsin_module {
	void __iomem *base_addr;
	struct device *dev;
	struct v4l2_subdev sd;
	struct v4l2_mbus_framefmt frm_fmt;
	struct v4l2_subdev_frame_interval frm_intrvl;

	struct pltfrm_cvbsin_cfg *soc_cfg;

	struct pltfrm_cam_defrect defrects[4];
};
static struct cif_cvbsin_module *cvbsin;

#define write_cvbsin_reg(addr, val)	    \
	__raw_writel(val, addr + cvbsin->base_addr)
#define read_cvbsin_reg(addr)			\
	__raw_readl(addr + cvbsin->base_addr)

static struct cvbsin_module_config cvbsin_configs[] = {
	{
		.name = "720x480_60fps", /*NTSC*/
		.frm_fmt = {
			.width = 720,
			.height = 480,
			.code = V4L2_MBUS_FMT_VYUY8_2X8
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 60
			}
		}
	},
	{
		.name = "720x576_50fps", /*PAL*/
		.frm_fmt = {
			.width = 720,
			.height = 576,
			.code = V4L2_MBUS_FMT_UYVY8_2X8
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 50
			}
		}
	}
};

static int cvbsin_module_g_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	return 0;
};

static  int cvbsin_module_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	return 0;
}

static  int cvbsin_module_s_ext_ctrls(
	struct v4l2_subdev *sd,
	struct v4l2_ext_controls *ctrls)
{
	return 0;
}

static int cvbsin_module_s_power(struct v4l2_subdev *sd, int on)
{
	struct pltfrm_cvbsin_cfg_para cfg_para;
	if (on) {
		cfg_para.cmd = PLTFRM_CVBSIN_POWERON;
		(cvbsin->soc_cfg->soc_cfg)(&cfg_para);

		write_cvbsin_reg(CVBSIN_OUTCTRL, RST_ENABLE);
		mdelay(5);
		write_cvbsin_reg(CVBSIN_OUTCTRL, RST_DISABLE);
		mdelay(5);
		write_cvbsin_reg(CVBSIN_CTRL4, CTRL4_VAL);
		write_cvbsin_reg(CVBSIN_CTRL3, CTRL3_VAL);
		write_cvbsin_reg(CVBSIN_CTRL2, CTRL2_VAL);
	} else {
		cfg_para.cmd = PLTFRM_CVBSIN_POWEROFF;
		(cvbsin->soc_cfg->soc_cfg)(&cfg_para);
	}

	return 0;
}

static long cvbsin_module_ioctl(
	struct v4l2_subdev *sd,
	unsigned int cmd,
	void *arg)
{
	int ret = 0;
	struct pltfrm_cam_itf *itf_cfg = NULL;
	struct pltfrm_cvbsin_cfg_para cfg_para;

	if (!cvbsin || !cvbsin->soc_cfg)
		return -1;

	if (cmd == PLTFRM_CIFCAM_G_ITF_CFG) {
		itf_cfg = (struct pltfrm_cam_itf *)arg;
		itf_cfg->type = PLTFRM_CAM_ITF_BT601_8_FIELD;
		return ret;
	} else if (cmd == PLTFRM_CIFCAM_ATTACH) {
		return ret;
	} else if (cmd == PLTFRM_CIFCAM_G_DEFRECT) {
		struct pltfrm_cam_defrect *defrect =
			(struct pltfrm_cam_defrect *)arg;
		unsigned int i;

		for (i = 0; i < 4; i++) {
			if ((cvbsin->defrects[i].width == defrect->width) &&
			    (cvbsin->defrects[i].height == defrect->height))
				defrect->defrect = cvbsin->defrects[i].defrect;
		}
		return 0;
	} else if (cmd == PLTFRM_CIFCAM_RESET) {
		dev_info(cvbsin->dev, "cvbsin softreset\n");
		cfg_para.cmd = PLTFRM_CVBSIN_RST;
		(cvbsin->soc_cfg->soc_cfg)(&cfg_para);

		write_cvbsin_reg(CVBSIN_OUTCTRL, RST_ENABLE);
		mdelay(5);
		write_cvbsin_reg(CVBSIN_OUTCTRL, RST_DISABLE);
		mdelay(5);
		write_cvbsin_reg(CVBSIN_CTRL4, CTRL4_VAL);
		write_cvbsin_reg(CVBSIN_CTRL3, CTRL3_VAL);
		write_cvbsin_reg(CVBSIN_CTRL2, CTRL2_VAL);
		return ret;
	} else {
		dev_warn(cvbsin->dev, "cvbsin not support this ioctl\n");
		return ret;
	}
}
static struct v4l2_subdev_core_ops cvbsin_module_core_ops = {
	.g_ctrl = cvbsin_module_g_ctrl,
	.s_ctrl = cvbsin_module_s_ctrl,
	.s_ext_ctrls = cvbsin_module_s_ext_ctrls,
	.s_power = cvbsin_module_s_power,
	.ioctl = cvbsin_module_ioctl
};

static int cvbsin_module_s_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_frame_interval *interval)
{
	return 0;
}

static int cvbsin_module_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (!cvbsin) {
		pr_err("cvbsin not init\n");
		return -1;
	}
	if (enable)
		write_cvbsin_reg(CVBSIN_OUTCTRL, 0xfc);
	else
		write_cvbsin_reg(CVBSIN_OUTCTRL, RST_DISABLE);

	return 0;
}

static struct v4l2_subdev_video_ops cvbsin_module_video_ops = {
	.s_frame_interval = cvbsin_module_s_frame_interval,
	.s_stream = cvbsin_module_s_stream
};

static int cvbsin_module_enum_frameintervals(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	int index = 0;

	if (fie->index >= 1)
		return -EINVAL;

	if (!cvbsin || !cvbsin->soc_cfg)
		return -1;

	cvbsin_module_s_power(sd, 1);
	msleep(1000);

	index = read_cvbsin_reg(CVBSIN_STATUS) & 0x1;

	fie->code = cvbsin_configs[index].frm_fmt.code;
	fie->width = cvbsin_configs[index].frm_fmt.width;
	fie->height = cvbsin_configs[index].frm_fmt.height;
	fie->interval.numerator =
		cvbsin_configs[index].frm_intrvl.interval.numerator;
	fie->interval.denominator =
		cvbsin_configs[index].frm_intrvl.interval.denominator;

	return 0;
}

static int cvbsin_module_g_fmt(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	int index = 0;

	if (cvbsin == NULL)
		return -1;

	index = read_cvbsin_reg(CVBSIN_STATUS) & 0x1;
	fmt->code = cvbsin_configs[index].frm_fmt.code;
	fmt->width = cvbsin_configs[index].frm_fmt.width;
	fmt->height = cvbsin_configs[index].frm_fmt.height;

	pr_debug("no active config\n");

	return 0;
}

static int cvbsin_module_s_fmt(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_format *format)
{
	return 0;
}
static struct v4l2_subdev_pad_ops cvbsin_module_pad_ops = {
	.enum_frame_interval = cvbsin_module_enum_frameintervals,
	.get_fmt = cvbsin_module_g_fmt,
	.set_fmt = cvbsin_module_s_fmt,
};

static struct v4l2_subdev_ops cvbsin_module_ops = {
	.core = &cvbsin_module_core_ops,
	.video = &cvbsin_module_video_ops,
	.pad = &cvbsin_module_pad_ops
};

static struct pltfrm_cvbsin_cfg rk1108_cvbsin_cfg = {
	.name = CIF_CIF10_SOC_RK1108,
	.soc_cfg = pltfrm_rk1108_cvbsin_cfg,
};

static const struct of_device_id cif_cvbsin_of_match[] = {
	{.compatible = "rockchip,cvbsin",
	 .data = (void *)&rk1108_cvbsin_cfg
	},
	{},
};

static int cif_cvbsin_drv_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res = NULL;
	void __iomem *base_addr = NULL;
	const struct of_device_id *match = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct pltfrm_cvbsin_cfg_para cfg_para;
	struct pltfrm_cvbsin_init_para init_para;
	struct pltfrm_cvbsin_cfg *soc_cfg = NULL;

	match = of_match_node(cif_cvbsin_of_match, np);
	if (match == NULL) {
		dev_err(&pdev->dev, "no match\n");
		return -ENODEV;
	}
	soc_cfg = (struct pltfrm_cvbsin_cfg *)match->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev,
			"platform_get_resource_byname failed\n");
		return -ENODEV;
	}
	base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(base_addr)) {
		dev_err(&pdev->dev,
			"devm_ioremap_resource failed\n");
		if (IS_ERR(base_addr))
			ret = PTR_ERR(base_addr);
		else
			ret = -ENODEV;
		return ret;
	}

	cvbsin = (struct cif_cvbsin_module *)devm_kzalloc(
				&pdev->dev,
				sizeof(struct cif_cvbsin_module),
				GFP_KERNEL);
	if (!cvbsin)
		return -ENOMEM;

	v4l2_subdev_init(&cvbsin->sd, &cvbsin_module_ops);

	/* the owner is the same as the i2c_client's driver owner */
	cvbsin->sd.owner = pdev->dev.driver->owner;
	v4l2_set_subdevdata(&cvbsin->sd, pdev);
	platform_set_drvdata(pdev, &cvbsin->sd);
	cvbsin->dev = &pdev->dev;
	/* initialize name */
	snprintf(
		cvbsin->sd.name,
		sizeof(cvbsin->sd.name),
		"%s",
		pdev->dev.driver->name);

	cvbsin->base_addr = base_addr;

	if (!IS_ERR_OR_NULL(soc_cfg) &&
	    !IS_ERR_OR_NULL(soc_cfg->soc_cfg)) {
		cfg_para.cmd = PLTFRM_CVBSIN_INIT;
		cfg_para.cfg_para = &init_para;
		init_para.pdev = pdev;
		ret = (soc_cfg->soc_cfg)(&cfg_para);
		if (ret == 0)
			cvbsin->soc_cfg = soc_cfg;
	}

	of_property_read_u32_array(
		np,
		OF_CAMERA_MODULE_DEFRECT0,
		(unsigned int *)&cvbsin->defrects[0],
		6);
	of_property_read_u32_array(
		np,
		OF_CAMERA_MODULE_DEFRECT1,
		(unsigned int *)&cvbsin->defrects[1],
		6);
	of_property_read_u32_array(
		np,
		OF_CAMERA_MODULE_DEFRECT2,
		(unsigned int *)&cvbsin->defrects[2],
		6);
	of_property_read_u32_array(
		np,
		OF_CAMERA_MODULE_DEFRECT3,
		(unsigned int *)&cvbsin->defrects[3],
		6);

	/* config tvd */
	write_cvbsin_reg(CVBSIN_CTRL4, CTRL4_VAL);
	write_cvbsin_reg(CVBSIN_CTRL3, CTRL3_VAL);
	write_cvbsin_reg(CVBSIN_CTRL2, CTRL2_VAL);
	write_cvbsin_reg(CVBSIN_OUTCTRL, RST_DISABLE);

	pr_info("cvbsin probe success\n");
	return ret;
}

/* ======================================================================== */

static int cif_cvbsin_drv_remove(struct platform_device *pdev)
{
	v4l2_device_unregister_subdev(&cvbsin->sd);
	return 0;
}

static struct platform_driver cif_cvbsin_plat_drv = {
	.driver = {
		.name = "cvbs-in",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cif_cvbsin_of_match),
	 },
	.probe = cif_cvbsin_drv_probe,
	.remove = cif_cvbsin_drv_remove,
};

/* ======================================================================== */
static int cif_cvbsin_init(void)
{
	int ret;

	ret = platform_driver_register(&cif_cvbsin_plat_drv);
	if (ret) {
		pr_err("register driver failed.%d\n", ret);
		return -ENODEV;
	}

	return ret;
}

/* ======================================================================== */
static void __exit cif_cvbsin_exit(void)
{
	platform_driver_unregister(&cif_cvbsin_plat_drv);
}

module_init(cif_cvbsin_init);
module_exit(cif_cvbsin_exit);

MODULE_DESCRIPTION("V4L2 interface for CIF CVBS-IN driver");
MODULE_AUTHOR("George");
MODULE_LICENSE("GPL");
