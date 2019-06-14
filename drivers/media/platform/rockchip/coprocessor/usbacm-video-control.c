// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip Rkcoproc-usbacm-video Driver
 *
 * Copyright (C) 2019 Rockchip Electronics Co., Ltd.
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/rk-usbacm-msg.h>
#include <linux/usb/npu-acm.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define RKMODULE_CAMERA_MODULE_INDEX	"rockchip,camera-module-index"
#define RKMODULE_CAMERA_MODULE_FACING	"rockchip,camera-module-facing"
#define RKMODULE_CAMERA_MODULE_NAME	"rockchip,camera-module-name"
#define RKMODULE_CAMERA_LENS_NAME	"rockchip,camera-module-lens-name"

#define RKCOPROC_SINK_PAD       0
#define RKCOPROC_NUM_SINK_PADS  1
#define RKCOPROC_NUM_SRC_PADS   1
#define RKCOPROC_NUM_PADS       2
#define MAX_RKCOPROC_SENSORS	2

#define RKCOPROC_NUM_CAM	4
#define DEVICE_NAME "usbacm-video-control"

enum rkcoproc_pads {
	RKCOPROC_PAD_SINK = 0,
	RKCOPROC_PAD_SOURCE0
};

struct rkcoproc_sensor {
	struct v4l2_subdev *sd;
	struct v4l2_mbus_config mbus;
	int lanes;
};

struct rkcoproc_ctrl_ops {
	int (*npu_xfer)(unsigned char *write_buf,
			int write_length,
			unsigned char *read_buf,
			int read_size,
			int *read_count);
};

struct rkcoproc_dev {
	struct device	*dev;
	struct v4l2_subdev	sd;
	struct media_pad	pad;
	struct v4l2_async_notifier	notifier;
	struct v4l2_fwnode_bus_mipi_csi2	bus;

	/* lock to protect all members below */
	struct mutex	lock;

	struct v4l2_subdev	*src_sd;
	struct v4l2_mbus_framefmt	format_mbus;
	u32	module_index;
	const char	*module_facing;
	const char	*module_name;
	const char	*len_name;
	struct rkcoproc_sensor	sensors[MAX_RKCOPROC_SENSORS];
	int	num_sensors;

	struct rkcoproc_ctrl_ops	ctrl_ops;
};

static inline struct rkcoproc_dev *sd_to_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct rkcoproc_dev, sd);
}

static int rkcoproc_get_remote_node_dev(struct rkcoproc_dev *rkcoproc)
{
	struct i2c_client *sensor_pdev = NULL;
	struct device *dev = rkcoproc->dev;
	struct device_node *parent = dev->of_node;
	struct device_node *remote = NULL;
	int ret = 0;

	remote = of_graph_get_remote_node(parent, 0, 0);
	if (!remote) {
		dev_err(dev, "Failed to get (%s) remote node\n",
			of_node_full_name(parent));
		return -EINVAL;
	}

	sensor_pdev = of_find_i2c_device_by_node(remote);
	of_node_put(remote);
	if (!sensor_pdev) {
		dev_err(dev, "Failed to get sensor device(%s)\n",
			of_node_full_name(remote));
		ret = -EINVAL;
	} else {
		dev_info(dev, "Get i2c client(%s)!\n", sensor_pdev->name);
		rkcoproc->src_sd =
			i2c_get_clientdata(sensor_pdev);
		if (rkcoproc->src_sd) {
			dev_info(dev, "Get sensor device(%s)!\n",
				 rkcoproc->src_sd->name);
		} else {
			dev_err(dev, "Failed to get sensor drvdata\n");
			ret = -EINVAL;
		}
	}

	return ret;
}

static struct v4l2_subdev *get_remote_sensor(struct v4l2_subdev *sd)
{
	struct rkcoproc_dev *rkcoproc = sd_to_dev(sd);
	int ret = 0;

	if (!rkcoproc->src_sd) {
		ret = rkcoproc_get_remote_node_dev(rkcoproc);
		if (ret || !rkcoproc->src_sd) {
			dev_info(rkcoproc->dev, "remote node dev is NULL\n");
			return NULL;
		}
		rkcoproc->sd.ctrl_handler = rkcoproc->src_sd->ctrl_handler;
	}

	return rkcoproc->src_sd;
}

/*
 * V4L2 subdev operations.
 */

static int rkcoproc_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rkcoproc_dev *rkcoproc = sd_to_dev(sd);
	int ret = 0;
	int i, size, ret_size = 0;
	struct cam_msg_s *msg;
	char *ret_msg;

	mutex_lock(&rkcoproc->lock);

	dev_dbg(rkcoproc->dev, "stream %s, src_sd: %p\n",
		enable ? "ON" : "OFF",
		rkcoproc->src_sd);

	ret = v4l2_subdev_call(rkcoproc->src_sd, video, s_stream, enable);
	ret = (ret && ret != -ENOIOCTLCMD) ? ret : 0;
	if (ret)
		goto out;

	size = sizeof(*msg) + _IOC_SIZE(VIDIOC_STREAMON);
	msg = kmalloc(size, GFP_KERNEL);
	if (!msg) {
		v4l2_err(&rkcoproc->sd, "kmalloc msg failed. (wanted %d bytes)\n",
			 size);
		ret = -ENOMEM;
		goto out;
	}
	ret_msg = kmalloc(size, GFP_KERNEL);
	if (!ret_msg) {
		v4l2_err(&rkcoproc->sd, "kmalloc ret_msg failed. (wanted %d bytes)\n",
			 size);
		ret = -ENOMEM;
		goto free_msg;
	}

	if (enable)
		msg->type = VIDIOC_STREAMON;
	else
		msg->type = VIDIOC_STREAMOFF;

	memcpy(msg->msg_entity, &enable, sizeof(enable));
	for (i = 0; i < RKCOPROC_NUM_CAM; i++) {
		msg->cam_id = i;
		ret = rkcoproc->ctrl_ops.npu_xfer(
				(unsigned char *)msg, size,
				ret_msg, size, &ret_size);
		if (ret < 0)
			v4l2_err(&rkcoproc->sd,
				 "npu_xfer stream %s failed(0x%x) in cam[%d]\n",
				 enable ? "ON" : "OFF", ret, i);
	}
	kfree(ret_msg);
free_msg:
	kfree(msg);
out:
	mutex_unlock(&rkcoproc->lock);
	return ret;
}

static int rkcoproc_media_init(struct v4l2_subdev *sd)
{
	struct rkcoproc_dev *rkcoproc = sd_to_dev(sd);
	int ret = 0;

	/* set a default mbus format  */
	rkcoproc->format_mbus.code =  MEDIA_BUS_FMT_UYVY8_2X8;
	rkcoproc->format_mbus.field = V4L2_FIELD_NONE;
	rkcoproc->format_mbus.width = 1920;
	rkcoproc->format_mbus.height = 1080;

#if defined(CONFIG_MEDIA_CONTROLLER)
	rkcoproc->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &rkcoproc->pad, 0);
#endif
	return ret;
}

static int rkcoproc_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	struct v4l2_subdev *sensor = get_remote_sensor(sd);

	return v4l2_subdev_call(sensor, pad, enum_mbus_code, cfg, code);
}

static int rkcoproc_enum_frame_sizes(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct v4l2_subdev *sensor = get_remote_sensor(sd);

	return v4l2_subdev_call(sensor, pad, enum_frame_size, cfg, fse);
}

/* rkcoproc accepts all fmt/size from sensor */
static int rkcoproc_get_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *fmt)
{
	struct v4l2_subdev *sensor = get_remote_sensor(sd);

	return v4l2_subdev_call(sensor, pad, get_fmt, NULL, fmt);
}

static int rkcoproc_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *fmt)
{
	struct v4l2_subdev *sensor = get_remote_sensor(sd);
	struct rkcoproc_dev *rkcoproc = sd_to_dev(sd);
	int ret;
	int i, size, ret_size = 0;
	struct cam_msg_s *msg;
	char *ret_msg;
	struct v4l2_format usr_fmt;

	ret = v4l2_subdev_call(sensor, pad, set_fmt, NULL, fmt);
	if (ret)
		return ret;

	size = sizeof(*msg) + _IOC_SIZE(VIDIOC_S_FMT);
	msg = kmalloc(size, GFP_KERNEL);
	if (!msg) {
		v4l2_err(&rkcoproc->sd, "kmalloc msg failed. (wanted %d bytes)\n",
			 size);
		return -ENOMEM;
	}

	ret_msg = kmalloc(size, GFP_KERNEL);
	if (!ret_msg) {
		v4l2_err(&rkcoproc->sd, "kmalloc ret_msg failed. (wanted %d bytes)\n",
			 size);
		ret = -ENOMEM;
		goto free_msg;
	}

	msg->type = VIDIOC_S_FMT;
	usr_fmt.fmt.pix.width = fmt->format.width;
	usr_fmt.fmt.pix.height = fmt->format.height;
	usr_fmt.fmt.pix.colorspace = fmt->format.colorspace;
	usr_fmt.fmt.pix.field = fmt->format.field;
	usr_fmt.fmt.pix.pixelformat = fmt->format.code;
	memcpy(msg->msg_entity, &usr_fmt, sizeof(usr_fmt));
	for (i = 0; i < RKCOPROC_NUM_CAM; i++) {
		msg->cam_id = i;
		ret = rkcoproc->ctrl_ops.npu_xfer(
				(unsigned char *)msg, size,
				ret_msg, size, &ret_size);
		if (ret < 0)
			v4l2_err(&rkcoproc->sd,
				 "npu_xfer s_fmt failed(0x%x) in cam[%d]\n",
				 ret, i);
	}
	kfree(ret_msg);
free_msg:
	kfree(msg);

	return ret;
}

static int rkcoproc_g_mbus_config(struct v4l2_subdev *sd,
				  struct v4l2_mbus_config *mbus)
{
	struct v4l2_subdev *sensor_sd = get_remote_sensor(sd);

	return v4l2_subdev_call(sensor_sd, video, g_mbus_config, mbus);
}

static int rkcoproc_power(struct v4l2_subdev *sd, int on)
{
	struct v4l2_subdev *sensor = get_remote_sensor(sd);

	return v4l2_subdev_call(sensor, core, s_power, on);
}

static long rkcoproc_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct v4l2_subdev *sensor = get_remote_sensor(sd);

	return v4l2_subdev_call(sensor, core, ioctl, cmd, arg);
}

static long rkcoproc_compat_ioctl32(struct v4l2_subdev *sd,
				    unsigned int cmd,
				    unsigned long arg)
{
	struct v4l2_subdev *sensor = get_remote_sensor(sd);

	return v4l2_subdev_call(sensor, core, compat_ioctl32, cmd, arg);
}

static const struct media_entity_operations rkcoproc_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_video_ops rkcoproc_video_ops = {
	.g_mbus_config = rkcoproc_g_mbus_config,
	.s_stream = rkcoproc_s_stream,
};

static const struct v4l2_subdev_pad_ops rkcoproc_pad_ops = {
	.enum_mbus_code = rkcoproc_enum_mbus_code,
	.enum_frame_size = rkcoproc_enum_frame_sizes,
	.get_fmt = rkcoproc_get_fmt,
	.set_fmt = rkcoproc_set_fmt,
};

static const struct v4l2_subdev_core_ops rkcoproc_core_ops = {
	.s_power = rkcoproc_power,
	.ioctl = rkcoproc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = rkcoproc_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_ops rkcoproc_subdev_ops = {
	.core = &rkcoproc_core_ops,
	.video = &rkcoproc_video_ops,
	.pad = &rkcoproc_pad_ops,
};

static int rkcoproc_probe(struct platform_device *pdev)
{
	struct rkcoproc_dev *rkcoproc = NULL;
	char facing[2];
	struct device_node *node = pdev->dev.of_node;
	int ret;

	rkcoproc = devm_kzalloc(&pdev->dev, sizeof(*rkcoproc), GFP_KERNEL);
	if (!rkcoproc)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &rkcoproc->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &rkcoproc->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &rkcoproc->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &rkcoproc->len_name);
	if (ret) {
		dev_err(&pdev->dev, "could not get %s!\n",
			RKMODULE_CAMERA_LENS_NAME);
		return -EINVAL;
	}

	rkcoproc->dev = &pdev->dev;
	platform_set_drvdata(pdev, rkcoproc);
	rkcoproc->sd.dev = &pdev->dev;
	v4l2_subdev_init(&rkcoproc->sd, &rkcoproc_subdev_ops);
	rkcoproc->sd.entity.ops = &rkcoproc_entity_ops;
	rkcoproc->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	mutex_init(&rkcoproc->lock);

	memset(facing, 0, sizeof(facing));
	if (strcmp(rkcoproc->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(rkcoproc->sd.name, sizeof(rkcoproc->sd.name),
		 "m%02d_%s_%s %s",
		 rkcoproc->module_index, facing,
		 DEVICE_NAME,
		 dev_name(rkcoproc->dev));

	ret = rkcoproc_media_init(&rkcoproc->sd);
	if (ret < 0)
		goto rmmutex;

	ret = v4l2_async_register_subdev_sensor_common(&rkcoproc->sd);
	if (ret)
		v4l2_err(&rkcoproc->sd, "v4l2 async register subdev failed\n");

	rkcoproc->ctrl_ops.npu_xfer = npu_acm_transfer;
	v4l2_info(&rkcoproc->sd, "probe success!\n");

	return 0;

rmmutex:
	mutex_destroy(&rkcoproc->lock);
	return ret;
}

static int rkcoproc_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct rkcoproc_dev *rkcoproc = sd_to_dev(sd);

	v4l2_async_unregister_subdev(sd);
	mutex_destroy(&rkcoproc->lock);
	media_entity_cleanup(&sd->entity);

	return 0;
}

static const struct of_device_id rkcoproc_dt_ids[] = {
	{
		.compatible = "rockchip,usbacm-video-control",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rkcoproc_dt_ids);

static struct platform_driver rkcoproc_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = rkcoproc_dt_ids,
	},
	.probe = rkcoproc_probe,
	.remove = rkcoproc_remove,
};

static int __init rkcoproc_mod_init(void)
{
	return  platform_driver_register(&rkcoproc_driver);
}

static void __exit rkcoproc_mod_exit(void)
{
	platform_driver_unregister(&rkcoproc_driver);
}

late_initcall(rkcoproc_mod_init);
module_exit(rkcoproc_mod_exit);

MODULE_DESCRIPTION("Rockchip rkcoproc-usbacm-video driver");
MODULE_AUTHOR("Macrofly.xu <xuhf@rock-chips.com>");
MODULE_LICENSE("GPL");
