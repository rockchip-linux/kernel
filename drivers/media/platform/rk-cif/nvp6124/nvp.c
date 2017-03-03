/*
**************************************************************************
 * Rockchip driver for NVP6124b
 *
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
**************************************************************************
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/videobuf-core.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/pinctrl-state.h>
#include "video.h"

#define NVP_I2C_NAME "nvp"
#define GPIO_HIGH 1
#define GPIO_LOW 0

#define OF_NVP_MODULE_CHANNELS "channels"
#define OF_NVP_MODULE_APIO_VOL "apio_vol"
#define OF_NVP_MODULE_NVP_MODE "nvp_mode"
#define OF_NVP_MODULE_CVBS_MODE "cvbs_mode"
#define OF_CAMERA_MODULE_DEFRECT0 "rockchip,camera-module-defrect0"
#define OF_CAMERA_MODULE_DEFRECT1 "rockchip,camera-module-defrect1"
#define OF_CAMERA_MODULE_DEFRECT2 "rockchip,camera-module-defrect2"
#define OF_CAMERA_MODULE_DEFRECT3 "rockchip,camera-module-defrect3"

#define GRF_SOC_CON1 0x0404
#define APIO5_33V   (0 << 14 | 1 << 30)
#define APIO5_18V   (1 << 14 | 1 << 30)

#define WRITE_MASK  (0xFFFF << 16)
#define ENABLE_MASK (0x8000 << 16)
#define CIF1TO4_EN  (0x1 << 15)
#define CIF1TO4_DIS (0x0 << 15)
#define CH_NUM_1    (0x0 << 12)
#define CH_NUM_2    (0x1 << 12)
#define CH_NUM_4    (0x2 << 12)

#define CHATCIF0    (0x0 << 10)
#define CHBTCIF0    (0x1 << 10)
#define CHCTCIF0    (0x2 << 10)
#define CHDTCIF0    (0x3 << 10)

#define CHATCIF1    (0x0 << 8)
#define CHBTCIF1    (0x1 << 8)
#define CHCTCIF1    (0x2 << 8)
#define CHDTCIF1    (0x3 << 8)

#define CHATCIF2    (0x0 << 6)
#define CHBTCIF2    (0x1 << 6)
#define CHCTCIF2    (0x2 << 6)
#define CHDTCIF2    (0x3 << 6)

#define CHATCIF3    (0x0 << 4)
#define CHBTCIF3    (0x1 << 4)
#define CHCTCIF3    (0x2 << 4)
#define CHDTCIF3    (0x3 << 4)

static struct nvp *nvp;

static struct nvp_module_config nvp_configs[] = {
	{
		.name = "720x480_30fps",	/* NTSC */
		.frm_fmt = {
			.width = 720,
			.height = 480,
			.code = V4L2_MBUS_FMT_VYUY8_2X8
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 30
			}
		},
		PLTFRM_CAM_ITF_DVP_CFG(
			PLTFRM_CAM_ITF_BT601_8_FIELD,
			PLTFRM_CAM_SIGNAL_LOW_LEVEL,
			PLTFRM_CAM_SIGNAL_HIGH_LEVEL,
			PLTFRM_CAM_SDR_NEG_EDG)
	},
	{
		.name = "720x576_25fps",	/* PAL */
		.frm_fmt = {
			.width = 720,
			.height = 576,
			.code = V4L2_MBUS_FMT_UYVY8_2X8
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 25
			}
		},
		PLTFRM_CAM_ITF_DVP_CFG(
			PLTFRM_CAM_ITF_BT601_8_FIELD,
			PLTFRM_CAM_SIGNAL_LOW_LEVEL,
			PLTFRM_CAM_SIGNAL_HIGH_LEVEL,
			PLTFRM_CAM_SDR_NEG_EDG)
	},
	{
		.name = "1280x720_30fps",	/* 720p@30 */
		.frm_fmt = {
			.width = 1280,
			.height = 720,
			.code = V4L2_MBUS_FMT_VYUY8_2X8
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 30
			}
		},
		PLTFRM_CAM_ITF_DVP_CFG(
			PLTFRM_CAM_ITF_BT601_8_FIELD,
			PLTFRM_CAM_SIGNAL_LOW_LEVEL,
			PLTFRM_CAM_SIGNAL_HIGH_LEVEL,
			PLTFRM_CAM_SDR_NEG_EDG)
	},
	{
		.name = "1280x720_25fps",	/* 720P@25 */
		.frm_fmt = {
			.width = 1280,
			.height = 720,
			.code = V4L2_MBUS_FMT_UYVY8_2X8
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 25
			}
		},
		PLTFRM_CAM_ITF_DVP_CFG(
			PLTFRM_CAM_ITF_BT601_8_FIELD,
			PLTFRM_CAM_SIGNAL_LOW_LEVEL,
			PLTFRM_CAM_SIGNAL_HIGH_LEVEL,
			PLTFRM_CAM_SDR_NEG_EDG)
	},
	{
		.name = "1920x1080_30fps",	/* 1080p@30 */
		.frm_fmt = {
			.width = 1920,
			.height = 1080,
			.code = V4L2_MBUS_FMT_VYUY8_2X8
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 30
			}
		},
		PLTFRM_CAM_ITF_DVP_CFG(
			PLTFRM_CAM_ITF_BT601_8_FIELD,
			PLTFRM_CAM_SIGNAL_LOW_LEVEL,
			PLTFRM_CAM_SIGNAL_HIGH_LEVEL,
			PLTFRM_CAM_SDR_NEG_EDG)
	},
	{
		.name = "1920x1080_25fps",	/* 1080P@25 */
		.frm_fmt = {
			.width = 1920,
			.height = 1080,
			.code = V4L2_MBUS_FMT_UYVY8_2X8
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 25
			}
		},
		PLTFRM_CAM_ITF_DVP_CFG(
			PLTFRM_CAM_ITF_BT601_8_FIELD,
			PLTFRM_CAM_SIGNAL_LOW_LEVEL,
			PLTFRM_CAM_SIGNAL_HIGH_LEVEL,
			PLTFRM_CAM_SDR_NEG_EDG)
	}
};

static int nvp_module_g_ctrl(
		struct v4l2_subdev *sd,
		struct v4l2_control *ctrl)
{
	return 0;
};

static int nvp_module_s_ctrl(
		struct v4l2_subdev *sd,
		struct v4l2_control *ctrl)
{
	return 0;
}

static int
nvp_module_s_ext_ctrls(
	struct v4l2_subdev *sd,
	struct v4l2_ext_controls *ctrls)
{
	return 0;
}

static int
nvp_module_s_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;

	if (!nvp)
		return -1;

	if (on)
		ret = pinctrl_pm_select_default_state(&nvp->client->dev);
	else
		ret = pinctrl_pm_select_sleep_state(&nvp->client->dev);

	return ret;
}

static long
nvp_module_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct pltfrm_cam_itf *itf_cfg = NULL;

	if (!nvp)
		return -1;
	if (cmd == PLTFRM_CIFCAM_G_ITF_CFG) {
		itf_cfg = (struct pltfrm_cam_itf *)arg;
		memcpy(itf_cfg,
		       &nvp_configs[0].itf_cfg,
		       sizeof(struct pltfrm_cam_itf));
		if (nvp->channels == 1)
			itf_cfg->type = PLTFRM_CAM_ITF_BT656_8_1;
		else if (nvp->channels == 2)
			itf_cfg->type = PLTFRM_CAM_ITF_BT656_8_2;
		else if (nvp->channels == 4)
			itf_cfg->type = PLTFRM_CAM_ITF_BT656_8_4;

		if (nvp->apio_vol == 1800)
			itf_cfg->cfg.dvp.io_vol = PLTFRM_CAM_IO_1800;
		else if (nvp->apio_vol == 3300)
			itf_cfg->cfg.dvp.io_vol = PLTFRM_CAM_IO_3300;
		else
			pr_err("Don't support %d voltage\n",
			       nvp->apio_vol);
		return ret;
	} else if (cmd == PLTFRM_CIFCAM_ATTACH) {
		nvp->soc_cfg = (struct pltfrm_soc_cfg *)arg;
		return ret;
	} else if (cmd == PLTFRM_CIFCAM_G_DEFRECT) {
		struct pltfrm_cam_defrect *defrect =
			(struct pltfrm_cam_defrect *)arg;
		unsigned int i;

		for (i = 0; i < 4; i++) {
			if ((nvp->defrects[i].width == defrect->width) &&
			    (nvp->defrects[i].height == defrect->height))
				defrect->defrect = nvp->defrects[i].defrect;
		}
		return 0;
	} else if (cmd == PLTFRM_CIFCAM_RESET) {
		dev_info(&nvp->client->dev,
			 "nvp softreset\n");
		/* rst */
		return ret;
	} else {
		dev_warn(&nvp->client->dev,
			 "nvp not support this ioctl\n");
		return ret;
	}
}

static struct v4l2_subdev_core_ops nvp_module_core_ops = {
	.g_ctrl = nvp_module_g_ctrl,
	.s_ctrl = nvp_module_s_ctrl,
	.s_ext_ctrls = nvp_module_s_ext_ctrls,
	.s_power = nvp_module_s_power,
	.ioctl = nvp_module_ioctl
};

static int
nvp_module_s_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_frame_interval *interval)
{
	return 0;
}

static int
nvp_module_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (!nvp) {
		pr_err("nvp not init\n");
		return -1;
	}

	return 0;
}

static int
nvp_module_enum_frameintervals(
	struct v4l2_subdev *sd,
	struct v4l2_frmivalenum *fie)
{
	int index = 0;

	if (fie->index > 0)
		return -EINVAL;
	if (nvp == NULL)
		return -1;
	if (!strcasecmp(nvp->cvbs_mode, "pal"))
		index = 1;
	else
		index = 0;
	if (nvp->nvp_mode &&
	    !strcasecmp(nvp->nvp_mode, "720p_2530"))
		index += 2;
	if (nvp->nvp_mode &&
	    !strcasecmp(nvp->nvp_mode, "1080p_2530"))
		index += 4;
	fie->pixel_format = nvp_configs[index].frm_fmt.code;
	fie->width = nvp_configs[index].frm_fmt.width;
	fie->height = nvp_configs[index].frm_fmt.height;
	fie->discrete.numerator =
		nvp_configs[index].frm_intrvl.interval.numerator;
	fie->discrete.denominator =
		nvp_configs[index].frm_intrvl.interval.denominator;
	return 0;
}

static int nvp_module_g_fmt(
	struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt)
{
	int index = 0;

	if (nvp == NULL)
		return -1;
	if (!strcasecmp(nvp->cvbs_mode, "pal"))
		index = 1;
	else
		index = 0;
	if (nvp->nvp_mode &&
	    !strcasecmp(nvp->nvp_mode, "720p_2530"))
		index += 2;
	if (nvp->nvp_mode &&
	    !strcasecmp(nvp->nvp_mode, "1080p_2530"))
		index += 4;
	fmt->code = nvp_configs[index].frm_fmt.code;
	fmt->width = nvp_configs[index].frm_fmt.width;
	fmt->height = nvp_configs[index].frm_fmt.height;
	pr_debug("no active config\n");
	return 0;
}

static int nvp_module_s_fmt(
	struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt)
{
	return 0;
}

static struct v4l2_subdev_video_ops nvp_module_video_ops = {
	.enum_frameintervals = nvp_module_enum_frameintervals,
	.g_mbus_fmt = nvp_module_g_fmt,
	.s_mbus_fmt = nvp_module_s_fmt,
	.s_frame_interval = nvp_module_s_frame_interval,
	.s_stream = nvp_module_s_stream
};

static struct v4l2_subdev_ops nvp_module_ops = {
	.core = &nvp_module_core_ops,
	.video = &nvp_module_video_ops,
};

static int nvp_set_channels(void)
{
	int chs;
	unsigned char vformat;

	if (!nvp)
		return -1;

	chs = nvp->channels;
	if (!strcasecmp(nvp->cvbs_mode, "pal"))
		vformat = PAL;
	else
		vformat = NTSC;

	switch (chs) {
	case 1:
	case 5:
		/* mode 1 to 1: channal 0 */
		if (nvp->nvp_mode &&
		    !strcasecmp(nvp->nvp_mode, "1080p_2530")) {
			nvp6124_each_mode_setting(nvp,
						  0,
						  vformat,
						  NVP6124_VI_1080P_2530);
			nvp6124b_set_portmode(nvp,
					      0,
					      NVP6124_OUTMODE_1MUX_FHD,
					      0);
		} else if (nvp->nvp_mode &&
			  !strcasecmp(nvp->nvp_mode, "720p_2530")) {
			nvp6124_each_mode_setting(nvp,
						  0,
						  vformat,
						  NVP6124_VI_720P_2530);
			nvp6124b_set_portmode(nvp,
					      0,
					      NVP6124_OUTMODE_1MUX_HD,
					      0);
		} else {
			nvp6124_each_mode_setting(nvp,
						  0,
						  vformat,
						  NVP6124_VI_720H);
			nvp6124b_set_portmode(nvp,
					      0,
					      NVP6124_OUTMODE_1MUX_SD,
					      0);
		}
		break;
	case 6:
		/* mode 1 to 1: channal 1 */
		nvp6124_each_mode_setting(nvp, 1, vformat, NVP6124_VI_720H);
		nvp6124b_set_portmode(nvp, 0, NVP6124_OUTMODE_1MUX_SD, 1);
		break;
	case 7:
		/* mode 1 to 1: channal 2 */
		nvp6124_each_mode_setting(nvp, 2, vformat, NVP6124_VI_720H);
		nvp6124b_set_portmode(nvp, 0, NVP6124_OUTMODE_1MUX_SD, 2);
		break;
	case 8:
		/* mode 1 to 1: channal 3 */
		nvp6124_each_mode_setting(nvp, 3, vformat, NVP6124_VI_720H);
		nvp6124b_set_portmode(nvp, 0, NVP6124_OUTMODE_1MUX_SD, 3);
		break;
	case 2:
		/* mode 1 to 2: channal 0 1 */
		if (nvp->nvp_mode &&
		    !strcasecmp(nvp->nvp_mode, "720p_2530")) {
			nvp6124_each_mode_setting(nvp,
						  0,
						  vformat,
						  NVP6124_VI_720P_2530);
			nvp6124_each_mode_setting(nvp,
						  1,
						  vformat,
						  NVP6124_VI_720P_2530);
			nvp6124b_set_portmode(nvp,
					      0,
					      NVP6124_OUTMODE_2MUX_MIX,
					      0);
		} else {
			nvp6124_each_mode_setting(nvp,
						  0,
						  vformat,
						  NVP6124_VI_720H);
			nvp6124_each_mode_setting(nvp,
						  1,
						  vformat,
						  NVP6124_VI_720H);
			nvp6124b_set_portmode(nvp,
					      0,
					      NVP6124_OUTMODE_2MUX_SD,
					      0);
		}
		break;
	case 3:
		/* mode 1 to 2: channal 2 3 */
		nvp6124_each_mode_setting(nvp, 2, vformat, NVP6124_VI_720H);
		nvp6124_each_mode_setting(nvp, 3, vformat, NVP6124_VI_720H);
		nvp6124b_set_portmode(nvp, 0, NVP6124_OUTMODE_2MUX_SD, 1);
		break;
	case 4:
		/* mode 1 to 4 */
		nvp6124_each_mode_setting(nvp, 0, vformat, NVP6124_VI_720H);
		nvp6124_each_mode_setting(nvp, 1, vformat, NVP6124_VI_720H);
		nvp6124_each_mode_setting(nvp, 2, vformat, NVP6124_VI_720H);
		nvp6124_each_mode_setting(nvp, 3, vformat, NVP6124_VI_720H);
		nvp6124b_set_portmode(nvp, 0, NVP6124_OUTMODE_4MUX_SD, 0);
		break;
	default:
		pr_info("wrong channal\n");
		break;
	}

	return 0;
}

static int nvp_hw_config(void)
{
	struct device_node *np;
	enum of_gpio_flags rst_flags;
	unsigned long irq_flags;
	unsigned int irq_gpio;
	struct device *dev = &nvp->client->dev;

	np = dev->of_node;
	irq_gpio = of_get_named_gpio_flags(np,
					   "irq-gpio",
					   0,
					   (enum of_gpio_flags *)&irq_flags);
	nvp->irq = gpio_to_irq(irq_gpio);
	nvp->reset_gpio =
		of_get_named_gpio_flags(np,
					"reset-gpio",
					0,
					&rst_flags);
	if (devm_gpio_request(dev, nvp->reset_gpio, NULL) != 0)	{
		gpio_free(nvp->reset_gpio);
		pr_err("nvp reset gpio_request error\n");
		return -EIO;
	}
	if (devm_gpio_request(dev, irq_gpio, NULL) != 0)	{
		gpio_free(irq_gpio);
		pr_err("nvp irq gpio_request error\n");
		return -EIO;
	}

	gpio_direction_output(nvp->reset_gpio, GPIO_HIGH);
	mdelay(10);
	gpio_set_value(nvp->reset_gpio, GPIO_LOW);
	mdelay(10);
	gpio_set_value(nvp->reset_gpio, GPIO_HIGH);
	mdelay(20);

	return 0;
}

static int nvp_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_I2C)) {
		dev_err(&client->dev, "not supported\n");
		return -ENODEV;
	}
	nvp = devm_kzalloc(&client->dev,
			   sizeof(*nvp),
			   GFP_KERNEL);
	if (!nvp) {
		dev_err(&client->dev, "nvp kzalloc failed\n");
		return -ENOMEM;
	}

	nvp->client = client;
	nvp->device_id = id->driver_data;
	v4l2_i2c_subdev_init(&nvp->sd, client, &nvp_module_ops);
	/* initialize name */
	snprintf(
		nvp->sd.name,
		sizeof(nvp->sd.name),
		"%s",
		client->dev.driver->name);

	if (of_property_read_u32(np,
				 OF_NVP_MODULE_CHANNELS,
				 &nvp->channels)) {
		nvp->channels = 0;
		dev_err(&client->dev,
			"get %s from dts failed!\n",
			OF_NVP_MODULE_CHANNELS);
		return -EINVAL;
	}

	if (of_property_read_u32(np,
				 OF_NVP_MODULE_APIO_VOL,
				 &nvp->apio_vol)) {
		dev_err(&client->dev,
			"get %s from dts failed!\n",
			OF_NVP_MODULE_APIO_VOL);
		return -EINVAL;
	}

	if (of_property_read_string(np,
				    OF_NVP_MODULE_CVBS_MODE,
				    &nvp->cvbs_mode)) {
		dev_err(&client->dev,
			"get %s from dts failed!\n",
			OF_NVP_MODULE_CVBS_MODE);
		return -EINVAL;
	}

	if (of_property_read_string(np,
				    OF_NVP_MODULE_NVP_MODE,
				    &nvp->nvp_mode)) {
		dev_warn(&client->dev,
			 "get %s from dts failed!\n",
			 OF_NVP_MODULE_NVP_MODE);
		nvp->nvp_mode = NULL;
	}

	of_property_read_u32_array(
		np,
		OF_CAMERA_MODULE_DEFRECT0,
		(unsigned int *)&nvp->defrects[0],
		6);
	of_property_read_u32_array(
		np,
		OF_CAMERA_MODULE_DEFRECT1,
		(unsigned int *)&nvp->defrects[1],
		6);
	of_property_read_u32_array(
		np,
		OF_CAMERA_MODULE_DEFRECT2,
		(unsigned int *)&nvp->defrects[2],
		6);
	of_property_read_u32_array(
		np,
		OF_CAMERA_MODULE_DEFRECT3,
		(unsigned int *)&nvp->defrects[3],
		6);

	nvp_hw_config();
	if (nvp6124b_check_id(nvp) < 0)
		return -ENXIO;
	nvp6124b_common_init(nvp);
	nvp_set_channels();
	pr_info("%s: success!\n", __func__);

	return 0;
}

static const struct i2c_device_id nvp_id[] = {
	{NVP_I2C_NAME, 0},
	{}
};

static struct of_device_id nvp_dt_ids[] = {
	{.compatible = "rockchip,nvp"},
	{}
};

static struct i2c_driver nvp_driver = {
	.driver = {
		.name = NVP_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(nvp_dt_ids),
	},
	.probe = nvp_probe,
	.id_table = nvp_id,
};

module_i2c_driver(nvp_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("nvp controller driver");
MODULE_AUTHOR("linjh");
MODULE_ALIAS("platform:rk");
