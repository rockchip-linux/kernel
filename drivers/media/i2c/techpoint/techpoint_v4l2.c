// SPDX-License-Identifier: GPL-2.0
/*
 * techpoint v4l2 driver
 *
 * Copyright (C) 2021 Rockchip Electronics Co., Ltd.
 *
 */

#include "techpoint_dev.h"

#define TECHPOINT_NAME  "techpoint"

#define OF_CAMERA_PINCTRL_STATE_DEFAULT		"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP		"rockchip,camera_sleep"

static const char *const techpoint_supply_names[] = {
	"dovdd",		/* Digital I/O power */
	"avdd",			/* Analog power */
	"dvdd",			/* Digital power */
};

#define TECHPOINT_NUM_SUPPLIES ARRAY_SIZE(techpoint_supply_names)

#define to_techpoint(sd) container_of(sd, struct techpoint, subdev)

static int techpoint_get_regulators(struct techpoint *techpoint)
{
	unsigned int i;
	struct i2c_client *client = techpoint->client;
	struct device *dev = &techpoint->client->dev;

	if (!techpoint->supplies)
		techpoint->supplies = devm_kzalloc(dev,
						   sizeof(struct
							  regulator_bulk_data) *
						   TECHPOINT_NUM_SUPPLIES,
						   GFP_KERNEL);

	for (i = 0; i < TECHPOINT_NUM_SUPPLIES; i++)
		techpoint->supplies[i].supply = techpoint_supply_names[i];
	return devm_regulator_bulk_get(&client->dev,
				       TECHPOINT_NUM_SUPPLIES,
				       techpoint->supplies);
}

static int techpoint_analyze_dts(struct techpoint *techpoint)
{
	int ret;
	struct i2c_client *client = techpoint->client;
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &techpoint->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &techpoint->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &techpoint->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &techpoint->len_name);
	if (ret) {
		dev_err(dev, "could not get %s!\n", RKMODULE_CAMERA_LENS_NAME);
		return -EINVAL;
	}

	ret = of_property_read_u32(node, TECHPOINT_CAMERA_XVCLK_FREQ,
				   &techpoint->xvclk_freq_value);
	if (ret)
		techpoint->xvclk_freq_value = 27000000;

	ret = of_property_read_u32(node, TECHPOINT_CHANNEL_NUMS,
				   &techpoint->channel_nums);
	if (ret)
		techpoint->channel_nums = 4;

	techpoint->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(techpoint->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	techpoint->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_HIGH);
	if (IS_ERR(techpoint->power_gpio))
		dev_warn(dev, "Failed to get power-gpios\n");
	else
		gpiod_set_value_cansleep(techpoint->power_gpio, 1);

	techpoint_get_regulators(techpoint);
	ret =
	    regulator_bulk_enable(TECHPOINT_NUM_SUPPLIES, techpoint->supplies);
	if (ret < 0)
		dev_err(dev, "Failed to enable regulators\n");

	techpoint->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(techpoint->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");
	else
		gpiod_set_value_cansleep(techpoint->reset_gpio, 0);

	techpoint->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(techpoint->pinctrl)) {
		techpoint->pins_default =
		    pinctrl_lookup_state(techpoint->pinctrl,
					 OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(techpoint->pins_default))
			dev_info(dev, "could not get default pinstate\n");

		techpoint->pins_sleep =
		    pinctrl_lookup_state(techpoint->pinctrl,
					 OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(techpoint->pins_sleep))
			dev_info(dev, "could not get sleep pinstate\n");
	} else {
		dev_info(dev, "no pinctrl\n");
	}

	return 0;
}

static int techpoint_initialize_controls(struct techpoint *techpoint)
{
	int ret;
	u64 pixel_rate;
	struct v4l2_ctrl_handler *handler;
	const struct techpoint_video_modes *mode;
	struct device *dev = &techpoint->client->dev;

	handler = &techpoint->ctrl_handler;
	mode = techpoint->cur_video_mode;

	if (techpoint->input_type == TECHPOINT_DVP_BT1120) {
		ret = v4l2_ctrl_handler_init(handler, 1);
		if (ret)
			return ret;
		handler->lock = &techpoint->mutex;
		pixel_rate = mode->link_freq_value;
		techpoint->pixel_rate_ctrl = v4l2_ctrl_new_std(handler, NULL,
							       V4L2_CID_PIXEL_RATE,
							       0, pixel_rate, 1,
							       pixel_rate);
		dev_dbg(dev, "initialize pixel_rate %lld\n", pixel_rate);
	} else if (techpoint->input_type == TECHPOINT_MIPI) {
		ret = v4l2_ctrl_handler_init(handler, 2);
		if (ret)
			return ret;
		handler->lock = &techpoint->mutex;
		techpoint->link_freq_ctrl =
		    v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ, 0,
					   0, &mode->link_freq_value);
		__v4l2_ctrl_s_ctrl(techpoint->link_freq_ctrl, 0);
		dev_dbg(dev, "initialize link_freq %lld\n",
			mode->link_freq_value);

		pixel_rate =
		    (u32) mode->link_freq_value / mode->bpp * 2 * mode->lane;
		techpoint->pixel_rate_ctrl =
		    v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE, 0,
				      pixel_rate, 1, pixel_rate);
		dev_dbg(dev, "initialize pixel_rate %lld\n", pixel_rate);
	}

	if (handler->error) {
		ret = handler->error;
		dev_err(dev, "Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	techpoint->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int __techpoint_power_on(struct techpoint *techpoint)
{
	int ret;
	struct device *dev = &techpoint->client->dev;

	dev_dbg(dev, "%s\n", __func__);

	if (!IS_ERR_OR_NULL(techpoint->pins_default)) {
		ret = pinctrl_select_state(techpoint->pinctrl,
					   techpoint->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins. ret=%d\n", ret);
	}

	if (!IS_ERR(techpoint->power_gpio)) {
		gpiod_set_value_cansleep(techpoint->power_gpio, 1);
		usleep_range(25 * 1000, 30 * 1000);
	}

	usleep_range(1500, 2000);

	if (!IS_ERR(techpoint->xvclk)) {
		ret =
		    clk_set_rate(techpoint->xvclk, techpoint->xvclk_freq_value);
		if (ret < 0)
			dev_warn(dev, "Failed to set xvclk rate\n");
		if (clk_get_rate(techpoint->xvclk) !=
		    techpoint->xvclk_freq_value)
			dev_warn(dev, "xvclk mismatched\n");
		ret = clk_prepare_enable(techpoint->xvclk);
		if (ret < 0) {
			dev_err(dev, "Failed to enable xvclk\n");
			goto err_clk;
		}
	}

	if (!IS_ERR(techpoint->reset_gpio)) {
		gpiod_set_value_cansleep(techpoint->reset_gpio, 0);
		usleep_range(10 * 1000, 20 * 1000);
		gpiod_set_value_cansleep(techpoint->reset_gpio, 1);
		usleep_range(10 * 1000, 20 * 1000);
		gpiod_set_value_cansleep(techpoint->reset_gpio, 0);
	}

	usleep_range(10 * 1000, 20 * 1000);

	return 0;

err_clk:
	if (!IS_ERR(techpoint->reset_gpio))
		gpiod_set_value_cansleep(techpoint->reset_gpio, 0);

	if (!IS_ERR_OR_NULL(techpoint->pins_sleep))
		pinctrl_select_state(techpoint->pinctrl, techpoint->pins_sleep);

	if (!IS_ERR(techpoint->power_gpio))
		gpiod_set_value_cansleep(techpoint->power_gpio, 0);

	return ret;
}

static void __techpoint_power_off(struct techpoint *techpoint)
{
	int ret;
	struct device *dev = &techpoint->client->dev;

#if TECHPOINT_SHARING_POWER
	return;
#endif

	dev_dbg(dev, "%s\n", __func__);

	if (!IS_ERR(techpoint->reset_gpio))
		gpiod_set_value_cansleep(techpoint->reset_gpio, 1);

	if (IS_ERR(techpoint->xvclk))
		clk_disable_unprepare(techpoint->xvclk);

	if (!IS_ERR_OR_NULL(techpoint->pins_sleep)) {
		ret = pinctrl_select_state(techpoint->pinctrl,
					   techpoint->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}

	if (!IS_ERR(techpoint->power_gpio))
		gpiod_set_value_cansleep(techpoint->power_gpio, 0);
}

static int techpoint_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct techpoint *techpoint = to_techpoint(sd);

	return __techpoint_power_on(techpoint);
}

static int techpoint_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct techpoint *techpoint = to_techpoint(sd);

	__techpoint_power_off(techpoint);

	return 0;
}

static int techpoint_power(struct v4l2_subdev *sd, int on)
{
	struct techpoint *techpoint = to_techpoint(sd);
	struct i2c_client *client = techpoint->client;
	int ret = 0;

	mutex_lock(&techpoint->mutex);

	/* If the power state is not modified - no work to do. */
	if (techpoint->power_on == !!on)
		goto exit;

	dev_dbg(&client->dev, "%s: on %d\n", __func__, on);

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto exit;
		}
		techpoint->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		techpoint->power_on = false;
	}

exit:
	mutex_unlock(&techpoint->mutex);

	return ret;
}

static int techpoint_get_reso_dist(const struct techpoint_video_modes *mode,
				   struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct techpoint_video_modes *
techpoint_find_best_fit(struct techpoint *techpoint,
			struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < techpoint->video_modes_num; i++) {
		dist =
		    techpoint_get_reso_dist(&techpoint->video_modes[i],
					    framefmt);
		if ((cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) &&
		    techpoint->video_modes[i].bus_fmt == framefmt->code) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &techpoint->video_modes[cur_best_fit];
}

static int techpoint_set_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
{
	struct techpoint *techpoint = to_techpoint(sd);
	const struct techpoint_video_modes *mode;

	mutex_lock(&techpoint->mutex);

	mode = techpoint_find_best_fit(techpoint, fmt);
	techpoint->cur_video_mode = mode;
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_SRGB;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&techpoint->mutex);
		return -ENOTTY;
#endif
	} else {
		if (techpoint->streaming) {
			mutex_unlock(&techpoint->mutex);
			return -EBUSY;
		}
	}

	mutex_unlock(&techpoint->mutex);
	return 0;
}

static int techpoint_get_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
{
	struct techpoint *techpoint = to_techpoint(sd);
	struct i2c_client *client = techpoint->client;
	const struct techpoint_video_modes *mode = techpoint->cur_video_mode;

	mutex_lock(&techpoint->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&techpoint->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		if (fmt->pad < PAD_MAX && fmt->pad >= PAD0)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&techpoint->mutex);

	dev_dbg(&client->dev, "%s: %x %dx%d\n",
		__func__, fmt->format.code,
		fmt->format.width, fmt->format.height);

	return 0;
}

static int techpoint_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	struct techpoint *techpoint = to_techpoint(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = techpoint->cur_video_mode->bus_fmt;

	return 0;
}

static int techpoint_enum_frame_sizes(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_size_enum *fse)
{
	struct techpoint *techpoint = to_techpoint(sd);
	struct i2c_client *client = techpoint->client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	if (fse->index >= techpoint->video_modes_num)
		return -EINVAL;

	if (fse->code != techpoint->video_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width = techpoint->video_modes[fse->index].width;
	fse->max_width = techpoint->video_modes[fse->index].width;
	fse->max_height = techpoint->video_modes[fse->index].height;
	fse->min_height = techpoint->video_modes[fse->index].height;

	return 0;
}

static int techpoint_g_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_frame_interval *fi)
{
	struct techpoint *techpoint = to_techpoint(sd);

	mutex_lock(&techpoint->mutex);
	fi->interval = techpoint->cur_video_mode->max_fps;
	mutex_unlock(&techpoint->mutex);

	return 0;
}

static int techpoint_g_mbus_config(struct v4l2_subdev *sd,
				   struct v4l2_mbus_config *cfg)
{
	struct techpoint *techpoint = to_techpoint(sd);

	if (techpoint->input_type == TECHPOINT_DVP_BT1120) {
		cfg->type = V4L2_MBUS_BT656;
		cfg->flags = RKMODULE_CAMERA_BT656_CHANNELS |
			     V4L2_MBUS_PCLK_SAMPLE_RISING |
			     V4L2_MBUS_PCLK_SAMPLE_FALLING;
	} else if (techpoint->input_type == TECHPOINT_MIPI) {
		cfg->type = V4L2_MBUS_CSI2;
		cfg->flags = V4L2_MBUS_CSI2_4_LANE | V4L2_MBUS_CSI2_CHANNELS;
	}

	return 0;
}

static int techpoint_querystd(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	struct techpoint *techpoint = to_techpoint(sd);

	if (techpoint->input_type == TECHPOINT_DVP_BT1120)
		*std = V4L2_STD_ATSC;

	return 0;
}

static __maybe_unused void techpoint_get_module_inf(struct techpoint *techpoint,
						    struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, TECHPOINT_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, techpoint->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, techpoint->len_name, sizeof(inf->base.lens));
}

static __maybe_unused void
techpoint_get_bt656_module_inf(struct techpoint *techpoint,
			       struct rkmodule_bt656_mbus_info *inf)
{
	memset(inf, 0, sizeof(*inf));
	if (techpoint->input_type == TECHPOINT_DVP_BT1120) {
		inf->id_en_bits = RKMODULE_CAMERA_BT656_ID_EN_BITS_2;
		inf->flags = RKMODULE_CAMERA_BT656_PARSE_ID_LSB |
			     RKMODULE_CAMERA_BT656_CHANNELS;
	}
}

static void techpoint_get_vicap_rst_inf(struct techpoint *techpoint,
					struct rkmodule_vicap_reset_info *rst_info)
{
	rst_info->is_reset = techpoint->do_reset;
	rst_info->src = RKCIF_RESET_SRC_ERR_HOTPLUG;
}

static void techpoint_set_vicap_rst_inf(struct techpoint *techpoint,
					struct rkmodule_vicap_reset_info *rst_info)
{
	techpoint->do_reset = rst_info->is_reset;
}

static long techpoint_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct techpoint *techpoint = to_techpoint(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		techpoint_get_module_inf(techpoint, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_BT656_MBUS_INFO:
		techpoint_get_bt656_module_inf(techpoint,
					       (struct rkmodule_bt656_mbus_info
						*)arg);
		break;
	case RKMODULE_GET_VC_FMT_INFO:
		if (!techpoint->streaming)
			techpoint_get_vc_fmt_inf(techpoint,
						 (struct rkmodule_vc_fmt_info *)
						 arg);
		else
			__techpoint_get_vc_fmt_inf(techpoint,
						   (struct rkmodule_vc_fmt_info
						    *)arg);
		break;
	case RKMODULE_GET_VC_HOTPLUG_INFO:
		techpoint_get_vc_hotplug_inf(techpoint,
					     (struct rkmodule_vc_hotplug_info *)
					     arg);
		break;
	case RKMODULE_GET_START_STREAM_SEQ:
		*(int *)arg = RKMODULE_START_STREAM_FRONT;
		break;
	case RKMODULE_GET_VICAP_RST_INFO:
		techpoint_get_vicap_rst_inf(techpoint,
					    (struct rkmodule_vicap_reset_info *)
					    arg);
		break;
	case RKMODULE_SET_VICAP_RST_INFO:
		techpoint_set_vicap_rst_inf(techpoint,
					    (struct rkmodule_vicap_reset_info *)
					    arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		techpoint_set_quick_stream(techpoint, *((u32 *)arg));
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long techpoint_compat_ioctl32(struct v4l2_subdev *sd,
				     unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_bt656_mbus_info *bt565_inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_vc_fmt_info *vc_fmt_inf;
	struct rkmodule_vc_hotplug_info *vc_hp_inf;
	struct rkmodule_vicap_reset_info *vicap_rst_inf;
	int *stream_seq;
	u32 stream;
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = techpoint_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = techpoint_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_VC_FMT_INFO:
		vc_fmt_inf = kzalloc(sizeof(*vc_fmt_inf), GFP_KERNEL);
		if (!vc_fmt_inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = techpoint_ioctl(sd, cmd, vc_fmt_inf);
		if (!ret)
			ret = copy_to_user(up, vc_fmt_inf, sizeof(*vc_fmt_inf));
		kfree(vc_fmt_inf);
		break;
	case RKMODULE_GET_VC_HOTPLUG_INFO:
		vc_hp_inf = kzalloc(sizeof(*vc_hp_inf), GFP_KERNEL);
		if (!vc_hp_inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = techpoint_ioctl(sd, cmd, vc_hp_inf);
		if (!ret)
			ret = copy_to_user(up, vc_hp_inf, sizeof(*vc_hp_inf));
		kfree(vc_hp_inf);
		break;
	case RKMODULE_GET_BT656_MBUS_INFO:
		bt565_inf = kzalloc(sizeof(*bt565_inf), GFP_KERNEL);
		if (!bt565_inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = techpoint_ioctl(sd, cmd, bt565_inf);
		if (!ret)
			ret = copy_to_user(up, bt565_inf, sizeof(*bt565_inf));
		kfree(bt565_inf);
		break;
	case RKMODULE_GET_VICAP_RST_INFO:
		vicap_rst_inf = kzalloc(sizeof(*vicap_rst_inf), GFP_KERNEL);
		if (!vicap_rst_inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = techpoint_ioctl(sd, cmd, vicap_rst_inf);
		if (!ret)
			ret =
			    copy_to_user(up, vicap_rst_inf,
					 sizeof(*vicap_rst_inf));
		kfree(vicap_rst_inf);
		break;
	case RKMODULE_SET_VICAP_RST_INFO:
		vicap_rst_inf = kzalloc(sizeof(*vicap_rst_inf), GFP_KERNEL);
		if (!vicap_rst_inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(vicap_rst_inf, up, sizeof(*vicap_rst_inf));
		if (!ret)
			ret = techpoint_ioctl(sd, cmd, vicap_rst_inf);
		kfree(vicap_rst_inf);
		break;
	case RKMODULE_GET_START_STREAM_SEQ:
		stream_seq = kzalloc(sizeof(*stream_seq), GFP_KERNEL);
		if (!stream_seq) {
			ret = -ENOMEM;
			return ret;
		}

		ret = techpoint_ioctl(sd, cmd, stream_seq);
		if (!ret)
			ret = copy_to_user(up, stream_seq, sizeof(*stream_seq));
		kfree(stream_seq);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = techpoint_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static __maybe_unused int __techpoint_start_stream(struct techpoint *techpoint)
{
	techpoint_start_video_stream(techpoint);
	return 0;
}

static __maybe_unused int __techpoint_stop_stream(struct techpoint *techpoint)
{
	techpoint_stop_video_stream(techpoint);
	return 0;
}

static int techpoint_stream(struct v4l2_subdev *sd, int on)
{
	struct techpoint *techpoint = to_techpoint(sd);
	struct i2c_client *client = techpoint->client;

	dev_dbg(&client->dev, "s_stream: %d. %dx%d\n", on,
		techpoint->cur_video_mode->width,
		techpoint->cur_video_mode->height);

	mutex_lock(&techpoint->mutex);
	on = !!on;
	if (techpoint->streaming == on)
		goto unlock;

	if (on)
		__techpoint_start_stream(techpoint);
	else
		__techpoint_stop_stream(techpoint);

	techpoint->streaming = on;

unlock:
	mutex_unlock(&techpoint->mutex);
	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int techpoint_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}
#endif

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops techpoint_internal_ops = {
	.open = techpoint_open,
};
#endif

static const struct v4l2_subdev_video_ops techpoint_video_ops = {
	.s_stream = techpoint_stream,
	.g_mbus_config = techpoint_g_mbus_config,
	.g_frame_interval = techpoint_g_frame_interval,
	.querystd = techpoint_querystd,
};

static const struct v4l2_subdev_pad_ops techpoint_subdev_pad_ops = {
	.enum_mbus_code = techpoint_enum_mbus_code,
	.enum_frame_size = techpoint_enum_frame_sizes,
	.get_fmt = techpoint_get_fmt,
	.set_fmt = techpoint_set_fmt,
};

static const struct v4l2_subdev_core_ops techpoint_core_ops = {
	.s_power = techpoint_power,
	.ioctl = techpoint_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = techpoint_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_ops techpoint_subdev_ops = {
	.core = &techpoint_core_ops,
	.video = &techpoint_video_ops,
	.pad = &techpoint_subdev_pad_ops,
};

static const struct dev_pm_ops techpoint_pm_ops = {
	SET_RUNTIME_PM_OPS(techpoint_runtime_suspend,
			   techpoint_runtime_resume, NULL)
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id techpoint_of_match[] = {
	{ .compatible = "techpoint,tp2855" },
	{ .compatible = "techpoint,tp2815" },
	{ .compatible = "techpoint,tp9930" },
	{ },
};

MODULE_DEVICE_TABLE(of, techpoint_of_match);
#endif

static const struct i2c_device_id techpoint_match_id[] = {
	{ "techpoint", 0 },
	{ },
};

static int techpoint_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct techpoint *techpoint;
	struct v4l2_subdev *sd;
	__maybe_unused char facing[2];
	int ret, index;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8, DRIVER_VERSION & 0x00ff);

	techpoint = devm_kzalloc(dev, sizeof(*techpoint), GFP_KERNEL);
	if (!techpoint)
		return -ENOMEM;

	techpoint->client = client;
	techpoint->supplies = NULL;

	mutex_init(&techpoint->mutex);

	sd = &techpoint->subdev;
	v4l2_i2c_subdev_init(sd, client, &techpoint_subdev_ops);

	techpoint_analyze_dts(techpoint);

	ret = __techpoint_power_on(techpoint);
	if (ret) {
		dev_err(dev, "Failed to power on techpoint\n");
		goto err_destroy_mutex;
	}

	ret = techpoint_initialize_devices(techpoint);
	if (ret) {
		dev_err(dev, "Failed to initialize techpoint device\n");
		goto err_power_off;
	}

	ret = techpoint_initialize_controls(techpoint);
	if (ret) {
		dev_err(dev, "Failed to initialize controls techpoint\n");
		goto err_free_handler;
	}
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &techpoint_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif

#if defined(CONFIG_MEDIA_CONTROLLER)
	for (index = 0; index < PAD_MAX; index++)
		techpoint->pad[index].flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, PAD_MAX, techpoint->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(techpoint->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 techpoint->module_index, facing,
		 TECHPOINT_NAME, dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_free_handler:
	v4l2_ctrl_handler_free(&techpoint->ctrl_handler);
err_power_off:
	__techpoint_power_off(techpoint);
err_destroy_mutex:
	mutex_destroy(&techpoint->mutex);

	return ret;
}

static int techpoint_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct techpoint *techpoint = to_techpoint(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&techpoint->ctrl_handler);
	mutex_destroy(&techpoint->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__techpoint_power_off(techpoint);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static struct i2c_driver techpoint_i2c_driver = {
	.driver = {
		   .name = TECHPOINT_NAME,
		   .pm = &techpoint_pm_ops,
		   .of_match_table = of_match_ptr(techpoint_of_match),
		    },
	.probe = &techpoint_probe,
	.remove = &techpoint_remove,
	.id_table = techpoint_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&techpoint_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&techpoint_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_AUTHOR("Vicent Chi <vicent.chi@rock-chips.com>");
MODULE_DESCRIPTION("Techpoint decoder driver");
MODULE_LICENSE("GPL v2");
