// SPDX-License-Identifier: GPL-2.0
/*
 * dw9718 vcm driver
 *
 * Copyright (C) 2019 Fuzhou Rockchip Electronics Co., Ltd.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/rk-camera-module.h>
#include <linux/version.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/rk_vcm_head.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x0)
#define DW9718_NAME			"dw9718"

#define DW9718T_PD_REG 0X00
#define DW9718T_CONTROL 0X01
#define DW9718T_DATAM_REG 0X02
#define DW9718T_DATAL_REG 0X03
#define DW9718T_SW_REG 0X04
#define DW9718T_SACT_REG 0X05
#define DW9718T_FLAG_REG 0X10

#define DW9718_MAX_CURRENT		100U
#define DW9718_MAX_REG			1023U

#define DW9718_DEFAULT_START_CURRENT	0
#define DW9718_DEFAULT_RATED_CURRENT	100
#define DW9718_DEFAULT_STEP_MODE	0xd
#define REG_NULL			0xFF

#define OF_CAMERA_VCMDRV_CONTROL_MODE     "rockchip,vcm-control-mode"
#define OF_CAMERA_VCMDRV_SACDIV_MODE      "rockchip,vcm-sacdiv-mode"
#define VCMDRV_DEFAULT_CONTROL_MODE       4
#define VCMDRV_DEFAULT_SACDIV_MODE        1

/* dw9718 device structure */
struct dw9718_device {
	struct v4l2_ctrl_handler ctrls_vcm;
	struct v4l2_subdev sd;
	struct v4l2_device vdev;
	u16 current_val;

	unsigned short current_related_pos;
	unsigned short current_lens_pos;
	unsigned int start_current;
	unsigned int rated_current;
	unsigned int step;
	unsigned int step_mode;
	unsigned int control_mode;
	unsigned int sacdiv_mode;
	unsigned long mv_time_per_pos;

	struct timeval start_move_tv;
	struct timeval end_move_tv;
	unsigned long move_us;

	u32 module_index;
	const char *module_facing;
};

static inline struct dw9718_device *to_dw9718_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct dw9718_device, ctrls_vcm);
}

static inline struct dw9718_device *sd_to_dw9718_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct dw9718_device, sd);
}

static int dw9718_read_reg(struct i2c_client *client,
	u8 addr, u32 *val, u8 len)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = (u8 *)&addr;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int dw9718_write_reg(struct i2c_client *client,
	u8 addr, u32 val, u8 len)
{
	u32 buf_i, val_i;
	u8 buf[5];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = addr & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 1;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 1) != len + 1)
		return -EIO;

	return 0;
}

static int dw9718_get_pos(struct dw9718_device *dev_vcm,
	unsigned int *cur_pos)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev_vcm->sd);
	int ret;
	u32 val;
	unsigned int abs_step;

	ret = dw9718_read_reg(client, DW9718T_DATAM_REG, &val, 2);
	if (ret != 0)
		goto err;

	abs_step = val & 0x3ff;
	if (abs_step <= dev_vcm->start_current)
		abs_step = VCMDRV_MAX_LOG;
	else if ((abs_step > dev_vcm->start_current) &&
		 (abs_step <= dev_vcm->rated_current))
		abs_step = (dev_vcm->rated_current - abs_step) / dev_vcm->step;
	else
		abs_step = 0;

	*cur_pos = abs_step;
	dev_dbg(&client->dev, "%s: get position %d\n", __func__, *cur_pos);
	return 0;

err:
	dev_err(&client->dev,
		"%s: failed with error %d\n", __func__, ret);
	return ret;
}

static int dw9718_set_pos(struct dw9718_device *dev_vcm,
	unsigned int dest_pos)
{
	int ret;
	unsigned int position = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dev_vcm->sd);

	if (dest_pos >= VCMDRV_MAX_LOG)
		position = dev_vcm->start_current;
	else
		position = dev_vcm->start_current +
			   (dev_vcm->step * (VCMDRV_MAX_LOG - dest_pos));

	if (position > DW9718_MAX_REG)
		position = DW9718_MAX_REG;

	dev_vcm->current_lens_pos = position;
	dev_vcm->current_related_pos = dest_pos;
	ret = dw9718_write_reg(client, DW9718T_DATAM_REG, position & 0x3ff, 2);
	if (ret != 0)
		goto err;

	return ret;
err:
	dev_err(&client->dev,
		"%s: failed with error %d\n", __func__, ret);
	return ret;
}

static int dw9718_get_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dw9718_device *dev_vcm = to_dw9718_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE)
		return dw9718_get_pos(dev_vcm, &ctrl->val);

	return -EINVAL;
}

static int dw9718_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dw9718_device *dev_vcm = to_dw9718_vcm(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(&dev_vcm->sd);
	unsigned int dest_pos = ctrl->val;
	int move_pos;
	long mv_us;
	int ret = 0;

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		if (dest_pos > VCMDRV_MAX_LOG) {
			dev_info(&client->dev,
				"%s dest_pos is error. %d > %d\n",
				__func__, dest_pos, VCMDRV_MAX_LOG);
			return -EINVAL;
		}
		/* calculate move time */
		move_pos = dev_vcm->current_related_pos - dest_pos;
		if (move_pos < 0)
			move_pos = -move_pos;

		ret = dw9718_set_pos(dev_vcm, dest_pos);

		if (dev_vcm->control_mode == 1)
			dev_vcm->move_us = dev_vcm->mv_time_per_pos * move_pos;
		else
			dev_vcm->move_us = dev_vcm->mv_time_per_pos;
		dev_dbg(&client->dev,
			"dest_pos %d, move_us %ld\n",
			dest_pos, dev_vcm->move_us);

		dev_vcm->start_move_tv = ns_to_timeval(ktime_get_ns());
		mv_us = dev_vcm->start_move_tv.tv_usec + dev_vcm->move_us;
		if (mv_us >= 1000000) {
			dev_vcm->end_move_tv.tv_sec =
				dev_vcm->start_move_tv.tv_sec + 1;
			dev_vcm->end_move_tv.tv_usec = mv_us - 1000000;
		} else {
			dev_vcm->end_move_tv.tv_sec =
					dev_vcm->start_move_tv.tv_sec;
			dev_vcm->end_move_tv.tv_usec = mv_us;
		}
	}

	return ret;
}

static const struct v4l2_ctrl_ops dw9718_vcm_ctrl_ops = {
	.g_volatile_ctrl = dw9718_get_ctrl,
	.s_ctrl = dw9718_set_ctrl,
};

static int dw9718t_init(struct dw9718_device *dev)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	u32 control_mode = 0;
	u32 step_mode = 0;

	/*There is need a sleep after power on for write i2c*/
	usleep_range(10000, 11000);
	dev_info(&client->dev, "enter vcm driver init\n");

	ret = dw9718_write_reg(client, DW9718T_PD_REG, 0, 1);
	if (ret < 0)
		goto err;
	/*There is need a sleep after out of standby status*/
	msleep(100);
	step_mode = (dev->sacdiv_mode << 6) | (dev->step_mode & 0x3f);
	ret = dw9718_write_reg(client, DW9718T_SACT_REG, step_mode, 1);
	if (ret < 0)
		goto err;
	control_mode = 0x31;
	control_mode |= (dev->control_mode & 0x07) << 1;
	ret = dw9718_write_reg(client, DW9718T_CONTROL, control_mode, 1);
	if (ret < 0)
		goto err;

	return 0;
err:
	dev_err(&client->dev, "failed with error %d\n", ret);
	return ret;
}

static int dw9718_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int rval;
	struct dw9718_device *dev_vcm = sd_to_dw9718_vcm(sd);

	rval = pm_runtime_get_sync(sd->dev);
	if (rval < 0) {
		pm_runtime_put_noidle(sd->dev);
		return rval;
	}

	rval = dw9718t_init(dev_vcm);
	if (rval < 0) {
		pm_runtime_put_noidle(sd->dev);
		return rval;
	}
	return 0;
}

static int dw9718_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops dw9718_int_ops = {
	.open = dw9718_open,
	.close = dw9718_close,
};

static long dw9718_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct rk_cam_vcm_tim *vcm_tim;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct dw9718_device *dw9718_dev = sd_to_dw9718_vcm(sd);

	if (cmd == RK_VIDIOC_VCM_TIMEINFO) {
		vcm_tim = (struct rk_cam_vcm_tim *)arg;

		vcm_tim->vcm_start_t.tv_sec = dw9718_dev->start_move_tv.tv_sec;
		vcm_tim->vcm_start_t.tv_usec =
				dw9718_dev->start_move_tv.tv_usec;
		vcm_tim->vcm_end_t.tv_sec = dw9718_dev->end_move_tv.tv_sec;
		vcm_tim->vcm_end_t.tv_usec = dw9718_dev->end_move_tv.tv_usec;

		dev_dbg(&client->dev, "dw9718_get_move_res 0x%lx, 0x%lx, 0x%lx, 0x%lx\n",
			vcm_tim->vcm_start_t.tv_sec,
			vcm_tim->vcm_start_t.tv_usec,
			vcm_tim->vcm_end_t.tv_sec,
			vcm_tim->vcm_end_t.tv_usec);
	} else {
		dev_err(&client->dev,
			"cmd 0x%x not supported\n", cmd);
		return -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long dw9718_compat_ioctl32(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct rk_cam_vcm_tim vcm_tim;
	struct rk_cam_compat_vcm_tim compat_vcm_tim;
	struct rk_cam_compat_vcm_tim __user *p32 = compat_ptr(arg);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	long ret;

	if (cmd == RK_VIDIOC_COMPAT_VCM_TIMEINFO) {
		ret = dw9718_ioctl(sd, RK_VIDIOC_VCM_TIMEINFO, &vcm_tim);
		compat_vcm_tim.vcm_start_t.tv_sec = vcm_tim.vcm_start_t.tv_sec;
		compat_vcm_tim.vcm_start_t.tv_usec =
				vcm_tim.vcm_start_t.tv_usec;
		compat_vcm_tim.vcm_end_t.tv_sec = vcm_tim.vcm_end_t.tv_sec;
		compat_vcm_tim.vcm_end_t.tv_usec = vcm_tim.vcm_end_t.tv_usec;

		put_user(compat_vcm_tim.vcm_start_t.tv_sec,
			&p32->vcm_start_t.tv_sec);
		put_user(compat_vcm_tim.vcm_start_t.tv_usec,
			&p32->vcm_start_t.tv_usec);
		put_user(compat_vcm_tim.vcm_end_t.tv_sec,
			&p32->vcm_end_t.tv_sec);
		put_user(compat_vcm_tim.vcm_end_t.tv_usec,
			&p32->vcm_end_t.tv_usec);
	} else {
		dev_err(&client->dev,
			"cmd 0x%x not supported\n", cmd);
		return -EINVAL;
	}

	return ret;
}
#endif

static const struct v4l2_subdev_core_ops dw9718_core_ops = {
	.ioctl = dw9718_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = dw9718_compat_ioctl32
#endif
};

static const struct v4l2_subdev_ops dw9718_ops = {
	.core = &dw9718_core_ops,
};

static void dw9718_subdev_cleanup(struct dw9718_device *dw9718_dev)
{
	v4l2_device_unregister_subdev(&dw9718_dev->sd);
	v4l2_device_unregister(&dw9718_dev->vdev);
	v4l2_ctrl_handler_free(&dw9718_dev->ctrls_vcm);
	media_entity_cleanup(&dw9718_dev->sd.entity);
}

static int dw9718_init_controls(struct dw9718_device *dev_vcm)
{
	struct v4l2_ctrl_handler *hdl = &dev_vcm->ctrls_vcm;
	const struct v4l2_ctrl_ops *ops = &dw9718_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, VCMDRV_MAX_LOG, 1, VCMDRV_MAX_LOG);

	if (hdl->error)
		dev_err(dev_vcm->sd.dev, "%s fail error: 0x%x\n",
			__func__, hdl->error);
	dev_vcm->sd.ctrl_handler = hdl;
	return hdl->error;
}

static int dw9718_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device_node *np = of_node_get(client->dev.of_node);
	struct dw9718_device *dw9718_dev;
	int ret;
	int current_distance;
	unsigned int start_current;
	unsigned int rated_current;
	unsigned int step_mode;
	unsigned int control_mode;
	unsigned int sacdiv_mode;
	struct v4l2_subdev *sd;
	char facing[2];

	dev_info(&client->dev, "probing...\n");
	if (of_property_read_u32(np,
		OF_CAMERA_VCMDRV_START_CURRENT,
		(unsigned int *)&start_current)) {
		start_current = DW9718_DEFAULT_START_CURRENT;
		dev_info(&client->dev,
			"could not get module %s from dts!\n",
			OF_CAMERA_VCMDRV_START_CURRENT);
	}
	if (of_property_read_u32(np,
		OF_CAMERA_VCMDRV_RATED_CURRENT,
		(unsigned int *)&rated_current)) {
		rated_current = DW9718_DEFAULT_RATED_CURRENT;
		dev_info(&client->dev,
			"could not get module %s from dts!\n",
			OF_CAMERA_VCMDRV_RATED_CURRENT);
	}
	if (of_property_read_u32(np,
		OF_CAMERA_VCMDRV_STEP_MODE,
		(unsigned int *)&step_mode)) {
		step_mode = DW9718_DEFAULT_STEP_MODE;
		dev_info(&client->dev,
			"could not get module %s from dts!\n",
			OF_CAMERA_VCMDRV_STEP_MODE);
	}
	if (of_property_read_u32(np,
		OF_CAMERA_VCMDRV_CONTROL_MODE,
		(unsigned int *)&control_mode)) {
		control_mode = VCMDRV_DEFAULT_CONTROL_MODE;
		dev_info(&client->dev,
			"could not get module %s from dts!\n",
			OF_CAMERA_VCMDRV_CONTROL_MODE);
	}
	if (of_property_read_u32(np,
		OF_CAMERA_VCMDRV_SACDIV_MODE,
		(unsigned int *)&sacdiv_mode)) {
		sacdiv_mode = VCMDRV_DEFAULT_SACDIV_MODE;
		dev_info(&client->dev,
			"could not get module %s from dts!\n",
			OF_CAMERA_VCMDRV_SACDIV_MODE);
	}

	dw9718_dev = devm_kzalloc(&client->dev, sizeof(*dw9718_dev),
				  GFP_KERNEL);
	if (dw9718_dev == NULL)
		return -ENOMEM;

	ret = of_property_read_u32(np, RKMODULE_CAMERA_MODULE_INDEX,
				   &dw9718_dev->module_index);
	ret |= of_property_read_string(np, RKMODULE_CAMERA_MODULE_FACING,
				       &dw9718_dev->module_facing);
	if (ret) {
		dev_err(&client->dev,
			"could not get module information!\n");
		return -EINVAL;
	}

	v4l2_i2c_subdev_init(&dw9718_dev->sd, client, &dw9718_ops);
	dw9718_dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9718_dev->sd.internal_ops = &dw9718_int_ops;

	ret = dw9718_init_controls(dw9718_dev);
	if (ret)
		goto err_cleanup;

	ret = media_entity_pads_init(&dw9718_dev->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	sd = &dw9718_dev->sd;
	sd->entity.function = MEDIA_ENT_F_LENS;

	memset(facing, 0, sizeof(facing));
	if (strcmp(dw9718_dev->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 dw9718_dev->module_index, facing,
		 DW9718_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev(sd);
	if (ret)
		dev_err(&client->dev, "v4l2 async register subdev failed\n");

	current_distance = rated_current - start_current;
	current_distance = current_distance * DW9718_MAX_REG /
						DW9718_MAX_CURRENT;
	dw9718_dev->step = (current_distance + (VCMDRV_MAX_LOG - 1)) /
						VCMDRV_MAX_LOG;
	dw9718_dev->start_current = start_current * DW9718_MAX_REG /
						DW9718_MAX_CURRENT;
	dw9718_dev->rated_current = dw9718_dev->start_current +
						VCMDRV_MAX_LOG *
						dw9718_dev->step;
	dw9718_dev->step_mode     = step_mode;
	dw9718_dev->move_us       = 0;
	dw9718_dev->current_related_pos = VCMDRV_MAX_LOG;
	dw9718_dev->start_move_tv = ns_to_timeval(ktime_get_ns());
	dw9718_dev->end_move_tv = ns_to_timeval(ktime_get_ns());

	switch (control_mode) {
	case 0:
		dev_err(&client->dev, "control_mode is derect mode, not support\n");
		return -EINVAL;
	case 1:
		dw9718_dev->mv_time_per_pos = (126 + step_mode * 2) * dw9718_dev->step;
		dev_dbg(&client->dev, "control_mode is LSC mode\n");
		break;
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
		dw9718_dev->mv_time_per_pos = (6300 + step_mode * 100);
		dev_dbg(&client->dev, "control_mode is LSC mode\n");
		break;
	default:
		dev_err(&client->dev, "set unknown control_mode\n");
		return -EINVAL;
	}

	switch (sacdiv_mode) {
	case 0:
		dw9718_dev->mv_time_per_pos *= 2;
		dev_dbg(&client->dev, "sacdiv_mode is %d\n", sacdiv_mode);
		break;
	case 1:
		dev_dbg(&client->dev, "sacdiv_mode is %d\n", sacdiv_mode);
		break;
	case 2:
		dw9718_dev->mv_time_per_pos /= 2;
		dev_dbg(&client->dev, "sacdiv_mode is %d\n", sacdiv_mode);
		break;
	case 3:
		dw9718_dev->mv_time_per_pos /= 4;
		dev_dbg(&client->dev, "sacdiv_mode is %d\n", sacdiv_mode);
		break;
	default:
		dev_err(&client->dev, "set unknown control_mode\n");
		return -EINVAL;
	}

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	dev_info(&client->dev, "probing successful\n");

	return 0;

err_cleanup:
	dw9718_subdev_cleanup(dw9718_dev);
	dev_err(&client->dev, "Probe failed: %d\n", ret);
	return ret;
}

static int dw9718_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9718_device *dw9718_dev = sd_to_dw9718_vcm(sd);

	pm_runtime_disable(&client->dev);
	dw9718_subdev_cleanup(dw9718_dev);

	return 0;
}

static int __maybe_unused dw9718_vcm_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused dw9718_vcm_resume(struct device *dev)
{
	return 0;
}

static const struct i2c_device_id dw9718_id_table[] = {
	{ DW9718_NAME, 0 },
	{ { 0 } }
};
MODULE_DEVICE_TABLE(i2c, dw9718_id_table);

static const struct of_device_id dw9718_of_table[] = {
	{ .compatible = "dongwoon,dw9718" },
	{ { 0 } }
};
MODULE_DEVICE_TABLE(of, dw9718_of_table);

static const struct dev_pm_ops dw9718_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dw9718_vcm_suspend, dw9718_vcm_resume)
	SET_RUNTIME_PM_OPS(dw9718_vcm_suspend, dw9718_vcm_resume, NULL)
};

static struct i2c_driver dw9718_i2c_driver = {
	.driver = {
		.name = DW9718_NAME,
		.pm = &dw9718_pm_ops,
		.of_match_table = dw9718_of_table,
	},
	.probe = &dw9718_probe,
	.remove = &dw9718_remove,
	.id_table = dw9718_id_table,
};

module_i2c_driver(dw9718_i2c_driver);

MODULE_DESCRIPTION("DW9718 VCM driver");
MODULE_LICENSE("GPL v2");
