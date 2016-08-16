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

#define RK1108_GRF_SOC_CON4  (0x0410)
#define RK1108_GRF_SOC_CON10 (0x0428)
#define RK1108_GRF_SOC_CON11 (0x042c)
#define VADC_PD_CLMP         (0x1 << 5)
#define VADC_LPF_BW_BYPASS   (0x3 << 12)
#define VADC_ICLMP_CTL_400MA (0x8 << 8)
#define VADC_GAIN            (0xa << 4)
#define CIF_DATA_FROM_CVBSIN (0x0 << 16 | 0x3)

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

	struct pltfrm_soc_cfg *soc_cfg;
	struct regmap *regmap_grf;
	struct clk *cvbs_clk_in;
	struct clk *cvbs_clk;
	struct clk *cvbs_clk_parent;
	struct clk *cif_clk;
	struct clk *cif_clk_parent;

	struct reset_control *cvbs_prst;
	struct reset_control *cvbs_hrst;
	struct reset_control *cvbs_clk_rst;
};
static struct cif_cvbsin_module *cvbsin;

#define write_cvbsin_reg(addr, val)	    \
	__raw_writel(val, addr + cvbsin->base_addr)
#define read_cvbsin_reg(addr)			\
	__raw_readl(addr + cvbsin->base_addr)

#define write_grf_reg(addr, val)	\
	regmap_write(cvbsin->regmap_grf, addr, val)
#define read_grf_reg(addr)	        \
	regmap_read(cvbsin->regmap_grf, addr)

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
	if (on) {
		clk_set_parent(cvbsin->cif_clk,
			       cvbsin->cif_clk_parent);
		write_grf_reg(RK1108_GRF_SOC_CON4,
			      CIF_DATA_FROM_CVBSIN);
	}

	return 0;
}

static long cvbsin_module_ioctl(
	struct v4l2_subdev *sd,
	unsigned int cmd,
	void *arg)
{
	int index = 0, ret = 0;
	struct pltfrm_cam_itf *itf_cfg = NULL;
	struct pltfrm_cam_defrect *defrect = NULL;

	if (!cvbsin)
		return -1;

	if (cmd == PLTFRM_CIFCAM_G_ITF_CFG) {
		itf_cfg = (struct pltfrm_cam_itf *)arg;
		itf_cfg->base_addr = cvbsin->base_addr;
		index = read_cvbsin_reg(CVBSIN_STATUS) & 0x1;
		if (index == 0)
			itf_cfg->type = PLTFRM_CAM_ITF_CVBS_NTSC;
		else
			itf_cfg->type = PLTFRM_CAM_ITF_CVBS_PAL;
		return ret;
	} else if (cmd == PLTFRM_CIFCAM_ATTACH) {
		cvbsin->soc_cfg = (struct pltfrm_soc_cfg *)arg;
		return ret;
	} else if (cmd == PLTFRM_CIFCAM_G_DEFRECT) {
		index = read_cvbsin_reg(CVBSIN_STATUS) & 0x1;
		defrect = (struct pltfrm_cam_defrect *)arg;
		defrect->defrect.top = 0;
		defrect->defrect.left = 0;
		defrect->defrect.width =
			cvbsin_configs[index].frm_fmt.width;
		defrect->defrect.height =
			cvbsin_configs[index].frm_fmt.height;
		return ret;
	} else if (cmd == PLTFRM_CIFCAM_RESET) {
		dev_info(cvbsin->dev, "cvbsin softreset\n");
		/* rst */
		reset_control_assert(cvbsin->cvbs_prst);
		reset_control_assert(cvbsin->cvbs_hrst);
		reset_control_assert(cvbsin->cvbs_clk_rst);
		udelay(5);
		reset_control_deassert(cvbsin->cvbs_prst);
		reset_control_deassert(cvbsin->cvbs_hrst);
		reset_control_deassert(cvbsin->cvbs_clk_rst);
		udelay(5);

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

	if (cvbsin == NULL)
		return -1;

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

static const struct of_device_id cif_cvbsin_of_match[] = {
	{.compatible = "rockchip,cvbsin"},
	{},
};

static int cif_cvbsin_drv_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *base_addr;
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		pr_err("platform_get_resource_byname failed\n");
		return -ENODEV;
	}
	base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(base_addr)) {
		pr_err("devm_ioremap_resource failed\n");
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

	cvbsin->regmap_grf =
		syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(cvbsin->regmap_grf)) {
		dev_err(&pdev->dev,
			"Can't regmap cvbsin rk1108 grf\n");
		return PTR_ERR(cvbsin->regmap_grf);
	}

	/* get clks resouce */
	cvbsin->cvbs_clk_in =
		devm_clk_get(&pdev->dev, "clk_cvbs_host");
	if (IS_ERR_OR_NULL(cvbsin->cvbs_clk_in)) {
		dev_err(&pdev->dev,
			"Get clk_cvbs_host clock resouce failed !\n");
		return -EINVAL;
	}
	cvbsin->cvbs_clk =
		devm_clk_get(&pdev->dev, "clk_cvbs");
	if (IS_ERR_OR_NULL(cvbsin->cvbs_clk)) {
		dev_err(&pdev->dev,
			"Get clk_cvbs clock resouce failed !\n");
		return -EINVAL;
	}
	cvbsin->cvbs_clk_parent =
		devm_clk_get(&pdev->dev, "clk_cvbs_parent");
	if (IS_ERR_OR_NULL(cvbsin->cvbs_clk_parent)) {
		dev_err(&pdev->dev,
			"Get clk_cvbsin_parent clock resouce failed !\n");
		return -EINVAL;
	}
	cvbsin->cif_clk =
		devm_clk_get(&pdev->dev, "cif_clk");
	if (IS_ERR_OR_NULL(cvbsin->cif_clk)) {
		dev_err(&pdev->dev,
			"Get cif_clk clock resouce failed !\n");
		return -EINVAL;
	}
	cvbsin->cif_clk_parent =
		devm_clk_get(&pdev->dev, "cif_clk_parent");
	if (IS_ERR_OR_NULL(cvbsin->cif_clk_parent)) {
		dev_err(&pdev->dev,
			"Get cif_clk_parent clock resouce failed !\n");
		return -EINVAL;
	}

	/* get rsts resouce */
	cvbsin->cvbs_prst =
		devm_reset_control_get(&pdev->dev, "cvbs_prst");
	if (IS_ERR_OR_NULL(cvbsin->cvbs_prst)) {
		dev_err(&pdev->dev,
			"Get cvbsin prst resouce failed !\n");
		return -EINVAL;
	}
	cvbsin->cvbs_hrst =
		devm_reset_control_get(&pdev->dev, "cvbs_hrst");
	if (IS_ERR_OR_NULL(cvbsin->cvbs_hrst)) {
		dev_err(&pdev->dev,
			"Get cvbsin hrst resouce failed !\n");
		return -EINVAL;
	}
	cvbsin->cvbs_clk_rst =
		devm_reset_control_get(&pdev->dev, "cvbs_clk_rst");
	if (IS_ERR_OR_NULL(cvbsin->cvbs_clk_rst)) {
		dev_err(&pdev->dev,
			"Get cvbsin clk rst resouce failed !\n");
		return -EINVAL;
	}

	/* config clk */
	clk_set_parent(cvbsin->cvbs_clk,
		       cvbsin->cvbs_clk_parent);
	clk_set_rate(cvbsin->cvbs_clk_in, 54000000);
	clk_prepare_enable(cvbsin->cvbs_clk_in);

	/* config adc */
	write_grf_reg(RK1108_GRF_SOC_CON10,
		      0xFFFF0000 |
		      VADC_LPF_BW_BYPASS |
		      VADC_ICLMP_CTL_400MA |
		      VADC_GAIN);
	write_grf_reg(RK1108_GRF_SOC_CON11, 0xFFFF0000 | VADC_PD_CLMP);

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
