/*
 * drivers/media/i2c/soc_camera/xgold/gt9760s.c
 *
 * gt9760s auto focus controller driver
 *
 * Copyright (C) 2014
 *
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Note:
 *    09/04/2014: initial version
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>
#include <linux/platform_data/rk_isp11_platform_camera_module.h>
#include <linux/platform_data/rk_isp11_platform.h>
#include <media/rk-isp11-config.h>
#include <linux/time.h>

#define GT9760S_DRIVER_NAME             "gt9760s"

#define GT9760S_SEL_ON_BYTE1 0xEC
#define GT9760S_SEL_ON_BYTE2 0xA3
#define GT9760S_DVO_DLC_BYTE1 0xA1
#define GT9760S_DVO_DLC_BYTE2 0xD
#define GT9760S_T_SRC_BYTE1 0xF2
#define GT9760S_T_SRC_BYTE2 0xF8
#define GT9760S_SEL_OFF_BYTE1 0xDC
#define GT9760S_SEL_OFF_BYTE2 0x51

/*
* Time to move the motor, This is fixed in the DLC specific setting
*/
#define GT9760S_DLC_MOVE_MS 33

/*
 * Focus position values:
 * 65 logical positions ( 0 - 64 )
 * where 64 is the setting for infinity and 0 for macro
 * corresponding to
 * 1024 register settings (0 - 1023)
 * where 0 is the setting for infinity and 1023 for macro
 */
#define MAX_LOG                        64U
#define MAX_REG                        1023U
#define MAX_VCMDRV_CURRENT             120U
#define MAX_VCMDRV_REG                 1023U

#define VCMDRV_DEFAULT_START_CURRENT   0
#define VCMDRV_DEFAULT_RATED_CURRENT   120
#define VCMDRV_DEFAULT_STEP_MODE       4
#define OF_CAMERA_VCMDRV_START_CURRENT "rockchip,vcm-start-current"
#define OF_CAMERA_VCMDRV_RATED_CURRENT "rockchip,vcm-rated-current"
#define OF_CAMERA_VCMDRV_STEP_MODE     "rockchip,vcm-step-mode"

/* ======================================================================== */
struct gt9760s_dev {
	unsigned short current_related_pos;
	unsigned short current_lens_pos;
	unsigned int start_current;
	unsigned int rated_current;
	unsigned int step;
	unsigned int step_mode;
	unsigned int vcm_movefull_t;

	struct timeval start_move_tv;
	struct timeval end_move_tv;
	unsigned long move_ms;

	struct v4l2_subdev sd;
};

/* ======================================================================== */
static int gt9760s_read_msg(
	struct i2c_client *client,
	unsigned char *msb, unsigned char *lsb)
{
	int ret = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retries;

	if (!client->adapter) {
		dev_err(&client->dev, "client->adapter NULL\n");
		return -ENODEV;
	}

	for (retries = 0; retries < 5; retries++) {
		msg->addr = client->addr;
		msg->flags = 1;
		msg->len = 2;
		msg->buf = data;

		ret = i2c_transfer(client->adapter, msg, 1);
		if (ret == 1) {
			dev_dbg(&client->dev,
				"%s: vcm i2c ok, addr 0x%x, data 0x%x, 0x%x\n",
				__func__, msg->addr, data[0], data[1]);

			*msb = data[0];
			*lsb = data[1];
			return 0;
		}

		dev_info(&client->dev,
			"retrying I2C... %d\n", retries);
		retries++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
	}
	dev_err(&client->dev,
		"%s: i2c write to failed with error %d\n", __func__, ret);
	return ret;
}

static int gt9760s_write_msg(
	struct i2c_client *client,
	unsigned char msb, unsigned char lsb)
{
	int ret = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retries;

	if (!client->adapter) {
		dev_err(&client->dev, "client->adapter NULL\n");
		return -ENODEV;
	}

	for (retries = 0; retries < 5; retries++) {
		msg->addr = client->addr;
		msg->flags = 0;
		msg->len = 2;
		msg->buf = data;

		data[0] = msb;
		data[1] = lsb;

		ret = i2c_transfer(client->adapter, msg, 1);
		if (ret == 1) {
			dev_dbg(&client->dev,
				"%s: vcm i2c ok, addr 0x%x, data 0x%x, 0x%x\n",
				__func__, msg->addr, data[0], data[1]);
			return 0;
		}

		dev_info(&client->dev,
			"retrying I2C... %d\n", retries);
		retries++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20.0));
	}
	dev_err(&client->dev,
		"%s: i2c write to failed with error %d\n", __func__, ret);
	return ret;
}

static int gt9760s_init(struct v4l2_subdev *sd, u32 val)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	msleep(7);

	ret = gt9760s_write_msg(client, GT9760S_SEL_ON_BYTE1,
				GT9760S_SEL_ON_BYTE2);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = gt9760s_write_msg(client, GT9760S_DVO_DLC_BYTE1,
				GT9760S_DVO_DLC_BYTE2);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = gt9760s_write_msg(client, GT9760S_T_SRC_BYTE1,
				GT9760S_T_SRC_BYTE2);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = gt9760s_write_msg(client, GT9760S_SEL_OFF_BYTE1,
				GT9760S_SEL_OFF_BYTE2);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	dev_err(&client->dev, "failed with error %d\n", ret);
	return -1;
}

static int gt9760s_get_pos(
	struct v4l2_subdev *sd,
	unsigned int *cur_pos)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gt9760s_dev *dev = container_of(sd, struct gt9760s_dev, sd);
	int ret;
	unsigned char lsb;
	unsigned char msb;
	unsigned int abs_step;

	ret = gt9760s_read_msg(client, &msb, &lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	abs_step = (((unsigned int)(msb & 0x3FU)) << 4U) | (((unsigned int)lsb) >> 4U);
	if (abs_step <= dev->start_current)
		abs_step = MAX_LOG;
	else if ((abs_step > dev->start_current) && (abs_step <= dev->rated_current))
		abs_step = (dev->rated_current - abs_step) / dev->step;
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

static int gt9760s_set_pos(
	struct v4l2_subdev *sd,
	unsigned int dest_pos)
{
	int ret;
	unsigned char lsb;
	unsigned char msb;
	unsigned int position;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gt9760s_dev *dev = container_of(sd, struct gt9760s_dev, sd);

	if (dest_pos > MAX_LOG) {
		dev_err(&client->dev,
			"value out of range, must be in [0..%d]\n", MAX_LOG);
		ret = -ERANGE;
		goto err;
	}
	dev_dbg(&client->dev, "%s: set position %d\n", __func__, dest_pos);

	if (dest_pos >= MAX_LOG)
		position = dev->start_current;
	else
		position = dev->start_current + (dev->step * (MAX_LOG - dest_pos));

	if (position > MAX_VCMDRV_REG)
		position = MAX_VCMDRV_REG;

	dev->current_lens_pos = position;
	dev->current_related_pos = dest_pos;
	msb = (0x00U | ((dev->current_lens_pos & 0x3F0U) >> 4U));
	lsb = (((dev->current_lens_pos & 0x0FU) << 4U) | dev->step_mode);
	ret = gt9760s_write_msg(client, msb, lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	dev_err(&client->dev,
		"%s: failed with error %d\n", __func__, ret);
	return ret;
}

static int gt9760s_g_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = -EINVAL;

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		ret = gt9760s_get_pos(sd, &ctrl->value);
		if (IS_ERR_VALUE(ret))
			goto err;

		dev_dbg(&client->dev,
			"%s V4L2_CID_FOCUS_ABSOLUTE %d\n", __func__, ctrl->value);
		return 0;
	}

err:
	dev_err(&client->dev,
		"%s: failed with error %d\n", __func__, ret);
	return ret;
}

static int gt9760s_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		dev_dbg(&client->dev,
			"%s V4L2_CID_FOCUS_ABSOLUTE %d\n", __func__, ctrl->value);

		ret = gt9760s_set_pos(sd, ctrl->value);
		if (IS_ERR_VALUE(ret))
			goto err;
	} else {
		dev_info(&client->dev,
			"ctrl ID %d not supported\n", ctrl->id);
		return -EINVAL;
	}

	return 0;
err:
	dev_err(&client->dev,
		"%s: failed with error %d\n", __func__, ret);
	return ret;
}

static long gt9760s_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd,
	void *arg)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gt9760s_dev *dev = container_of(sd, struct gt9760s_dev, sd);

	if (cmd == PLTFRM_CIFCAM_SET_VCM_POS) {
		unsigned int *dest_pos = (unsigned int *)arg;
		long int mv_us;

		if (*dest_pos > MAX_LOG) {
			return -EINVAL;
		} else {

			ret = gt9760s_set_pos(sd, *dest_pos);

			dev->move_ms = GT9760S_DLC_MOVE_MS;
			do_gettimeofday(&dev->start_move_tv);
			mv_us = dev->start_move_tv.tv_usec + dev->move_ms * 1000;
			if (mv_us >= 1000000) {
				dev->end_move_tv.tv_sec = dev->start_move_tv.tv_sec + 1;
				dev->end_move_tv.tv_usec = mv_us - 1000000;
			} else {
				dev->end_move_tv.tv_sec = dev->start_move_tv.tv_sec;
				dev->end_move_tv.tv_usec = mv_us;
			}

			dev_dbg(&client->dev, "gt9760s_set_vcm_pos 0x%lx, 0x%lx, 0x%lx\n",
				dev->move_ms,
				dev->start_move_tv.tv_sec * 1000 + dev->start_move_tv.tv_usec / 1000,
				dev->end_move_tv.tv_sec * 1000 + dev->end_move_tv.tv_usec / 1000);
		}
	} else if (cmd == PLTFRM_CIFCAM_GET_VCM_POS) {
		unsigned int *cur_pos = (unsigned int *)arg;

		ret = gt9760s_get_pos(sd, cur_pos);
	} else if (cmd == PLTFRM_CIFCAM_GET_VCM_MOVE_RES) {
		struct pltfrm_cam_vcm_tim *vcm_tim = (struct pltfrm_cam_vcm_tim *)arg;

		vcm_tim->vcm_start_t = dev->start_move_tv;
		vcm_tim->vcm_end_t = dev->end_move_tv;

		dev_dbg(&client->dev, "gt9760s_get_move_res 0x%lx, 0x%lx\n",
			vcm_tim->vcm_start_t.tv_sec * 1000 + vcm_tim->vcm_start_t.tv_usec / 1000,
			vcm_tim->vcm_end_t.tv_sec * 1000 + vcm_tim->vcm_end_t.tv_usec / 1000);
	} else {
		dev_info(&client->dev,
			"cmd %d not supported\n", cmd);
		return -EINVAL;
	}
	return ret;
}

/* ======================================================================== */
static struct v4l2_subdev_core_ops gt9760s_core_ops = {
	.g_ctrl = gt9760s_g_ctrl,
	.s_ctrl = gt9760s_s_ctrl,
	.ioctl = gt9760s_ioctl,
	.init = gt9760s_init
};

static struct v4l2_subdev_ops gt9760s_ops = {
	.core = &gt9760s_core_ops,
};

static int gt9760s_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct gt9760s_dev *dev;
	int current_distance;
	struct device_node *np = of_node_get(client->dev.of_node);
	unsigned int start_current;
	unsigned int rated_current;
	unsigned int step_mode;

	dev_info(&client->dev, "probing...\n");
	if (of_property_read_u32(
		np,
		OF_CAMERA_VCMDRV_START_CURRENT,
		(unsigned int *)&start_current)) {
		start_current = VCMDRV_DEFAULT_START_CURRENT;
		dev_info(&client->dev,
			"could not get module %s from dts!\n",
			OF_CAMERA_VCMDRV_START_CURRENT);
	}
	if (of_property_read_u32(
		np,
		OF_CAMERA_VCMDRV_RATED_CURRENT,
		(unsigned int *)&rated_current)) {
		rated_current = VCMDRV_DEFAULT_RATED_CURRENT;
		dev_info(&client->dev,
			"could not get module %s from dts!\n",
			OF_CAMERA_VCMDRV_RATED_CURRENT);
	}
	if (of_property_read_u32(
		np,
		OF_CAMERA_VCMDRV_STEP_MODE,
		(unsigned int *)&step_mode)) {
		step_mode = VCMDRV_DEFAULT_STEP_MODE;
		dev_info(&client->dev,
			"could not get module %s from dts!\n",
			OF_CAMERA_VCMDRV_STEP_MODE);
	}

	dev = devm_kzalloc(&client->dev, sizeof(struct gt9760s_dev), GFP_KERNEL);
	if (NULL == dev)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&dev->sd, client, &gt9760s_ops);

	current_distance = rated_current - start_current;
	current_distance = current_distance * MAX_VCMDRV_REG / MAX_VCMDRV_CURRENT;
	dev->step = (current_distance + (MAX_LOG - 1)) / MAX_LOG;
	dev->start_current = start_current * MAX_VCMDRV_REG / MAX_VCMDRV_CURRENT;
	dev->rated_current = dev->start_current + MAX_LOG * dev->step;
	dev->step_mode     = step_mode;
	dev->move_ms       = 0;
	dev->current_related_pos = MAX_LOG;
	do_gettimeofday(&dev->start_move_tv);
	do_gettimeofday(&dev->end_move_tv);

	dev->vcm_movefull_t = GT9760S_DLC_MOVE_MS;

	dev_info(&client->dev, "probing successful\n");
	return 0;
}

static int __exit gt9760s_remove(
	struct i2c_client *client)
{
	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id gt9760s_id[] = {
	{ GT9760S_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id gt9760s_of_match[] = {
	{.compatible = "silicon touch,gt9760s-v4l2-i2c-subdev"},
	{ }
};

MODULE_DEVICE_TABLE(i2c, gt9760s_id);

static struct i2c_driver gt9760s_i2c_driver = {
	.driver = {
		.name = GT9760S_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gt9760s_of_match
	},
	.probe = gt9760s_probe,
	.remove = __exit_p(gt9760s_remove),
	.id_table = gt9760s_id,
};

module_i2c_driver(gt9760s_i2c_driver);

MODULE_DESCRIPTION("gt9760s auto focus controller driver");
MODULE_AUTHOR("George");
MODULE_LICENSE("GPL");
