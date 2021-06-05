// SPDX-License-Identifier: GPL-2.0
/*
 * motor  driver
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 *
 */
//#define DEBUG
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/hrtimer.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <linux/completion.h>
#include <linux/rk_vcm_head.h>

#define DRIVER_VERSION	KERNEL_VERSION(0, 0x01, 0x00)

#define DRIVER_NAME "fp5501"

#define MAX_START_UP_HZ			(500)

#define FOCUS_MAX_STEP_DEF		7500
#define ZOOM_MAX_STEP_DEF		7500

#define FOCUS_MAX_BACK_DELAY		4
#define ZOOM_MAX_BACK_DELAY		4

#define MOTOR_PHASE_PINS (4)
#define PHASE_TYPES (4)

static u8 phases_cw[PHASE_TYPES] = {0x09, 0x0c, 0x06, 0x03};

enum {
	MOTOR_STATUS_STOPPED = 0,
	MOTOR_STATUS_CW = 1,
	MOTOR_STATUS_CCW = 2,
};

enum ext_dev_type {
	TYPE_FOCUS = 1,
	TYPE_ZOOM = 2,
};

struct ext_dev {
	u8 type;
	u32 step_max;
	u32 last_pos;
	u32 step_cnt;
	u32 cur_step_cnt;
	u32 move_status;
	u32 last_status;
	u32 reback;
	u32 reback_status;
	u32 phase_index;
	u32 cur_back_delay;
	u32 max_back_delay;
	bool is_need_reback;
	bool is_need_update_tim;
	bool is_running;
	bool is_dir_opp;
	bool reback_ctrl;
	struct rk_cam_vcm_tim mv_tim;
	struct completion comp_in;
	struct completion comp_out;
	struct gpio_desc *phase_gpios[MOTOR_PHASE_PINS];
};

struct motor_dev {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *focus_ctrl;
	struct v4l2_ctrl *zoom_ctrl;
	struct device *dev;
	struct hrtimer timer;
	u64 phase_interval_ns;
	u32 start_up_speed;
	u32 module_index;
	bool is_timer_restart;
	const char *module_facing;
	spinlock_t spin_lock;
	struct ext_dev focus;
	struct ext_dev zoom;
};

static int set_motor_running_status(struct motor_dev *motor,
				    struct ext_dev *ext_dev,
				    u32 pos, bool is_need_reback)
{
	int ret = 0;
	u32 mv_cnt = 0;
	int status = 0;
	unsigned long lock_flags = 0;

	if (ext_dev->move_status != MOTOR_STATUS_STOPPED)
		wait_for_completion(&ext_dev->comp_in);
	mv_cnt = abs(pos - ext_dev->last_pos);
	if (is_need_reback)
		mv_cnt += ext_dev->reback;
	if (mv_cnt == 0)
		return 0;
	reinit_completion(&ext_dev->comp_in);
	if (ext_dev->is_dir_opp) {
		if (pos > ext_dev->last_pos)
			status = MOTOR_STATUS_CCW;
		else
			status = MOTOR_STATUS_CW;
	} else {
		if (pos > ext_dev->last_pos)
			status = MOTOR_STATUS_CW;
		else
			status = MOTOR_STATUS_CCW;
	}
	if (ext_dev->last_status != MOTOR_STATUS_STOPPED &&
	    ext_dev->last_status != status)
		msleep(130);
	ext_dev->last_status = status;
	ext_dev->last_pos = pos;
	spin_lock_irqsave(&motor->spin_lock, lock_flags);
	ext_dev->move_status = status;
	ext_dev->is_need_reback = is_need_reback;
	ext_dev->step_cnt = mv_cnt + 1;
	ext_dev->cur_step_cnt = ext_dev->step_cnt;
	ext_dev->is_need_update_tim = true;

	if (motor->is_timer_restart == false) {
		motor->is_timer_restart = true;
		spin_unlock_irqrestore(&motor->spin_lock, lock_flags);
		hrtimer_start(&motor->timer, ktime_set(0, 0), HRTIMER_MODE_REL_PINNED);
	} else {
		spin_unlock_irqrestore(&motor->spin_lock, lock_flags);
	}
	ext_dev->is_running = true;
	dev_dbg(motor->dev,
		"ext tpye %d, mv_cnt %d, status %d, is_need_reback %d, reback %d, reback status %d\n",
		ext_dev->type, ext_dev->cur_step_cnt, status,
		is_need_reback, ext_dev->reback, ext_dev->reback_status);

	return ret;
}

static void motor_gpio_set_phase(struct ext_dev *dev, int phase_index)
{
	int i;
	int ctrl = phases_cw[phase_index];

	for (i = 0; i < MOTOR_PHASE_PINS; i++)
		gpiod_set_value(dev->phase_gpios[i], (ctrl >> i) & 0x1);
}

static void motor_gpio_set_stop(struct ext_dev *dev)
{
	int i = 0;

	if (dev->is_running) {
		for (i = 0; i < MOTOR_PHASE_PINS; i++)
			gpiod_set_value(dev->phase_gpios[i], 0);
	}
}

static void motor_set_config(struct ext_dev *dev)
{
	int phase_types = PHASE_TYPES;

	if (dev->cur_step_cnt != 0) {
		if (dev->step_cnt == dev->cur_step_cnt) {
			if (dev->is_need_update_tim) {
				dev->mv_tim.vcm_start_t = ns_to_timeval(ktime_get_ns());
				dev->is_need_update_tim  = false;
			}
		} else {
			if (dev->move_status == MOTOR_STATUS_CW)
				dev->phase_index = (dev->phase_index + 1) % phase_types;
			else
				dev->phase_index = (dev->phase_index - 1) % phase_types;
		}
		motor_gpio_set_phase(dev, dev->phase_index);
		dev->cur_step_cnt--;
	} else {
		if (dev->is_need_reback) {
			if (dev->cur_back_delay < dev->max_back_delay) {
				if (dev->cur_back_delay == 0)
					motor_gpio_set_stop(dev);
				dev->cur_back_delay++;
			} else {
				dev->cur_step_cnt = dev->reback + 1;
				dev->step_cnt = dev->reback + 1;
				dev->move_status = dev->reback_status;
				dev->last_status = dev->reback_status;
				dev->cur_back_delay = 0;
				dev->is_need_reback = false;
			}
		} else {
			motor_gpio_set_stop(dev);
			dev->move_status = MOTOR_STATUS_STOPPED;
			dev->mv_tim.vcm_end_t = ns_to_timeval(ktime_get_ns());
			dev->is_running = false;
			complete(&dev->comp_out);
			complete(&dev->comp_in);
		}
	}
}
static enum hrtimer_restart motor_timer_func(struct hrtimer *timer)
{
	struct motor_dev *motor;
	unsigned long lock_flags = 0;
	static struct timeval tv_last = {0};
	struct timeval tv = {0};
	u64 time_dist = 0;

	motor = container_of(timer, struct motor_dev, timer);

	do_gettimeofday(&tv);
	time_dist = tv.tv_sec * 1000000 + tv.tv_usec - (tv_last.tv_sec * 1000000 + tv_last.tv_usec);
	tv_last = tv;
	if (abs(time_dist * 1000 - motor->phase_interval_ns) > 250000)
		dev_dbg(motor->dev,
			 "focus cnt %d, zoom cnt %d, Current interrupt interval %llu\n",
			 motor->focus.cur_step_cnt, motor->zoom.cur_step_cnt, time_dist);
	spin_lock_irqsave(&motor->spin_lock, lock_flags);
	if (motor->focus.move_status != MOTOR_STATUS_STOPPED)
		motor_set_config(&motor->focus);
	if (motor->zoom.move_status != MOTOR_STATUS_STOPPED)
		motor_set_config(&motor->zoom);
	if (motor->focus.move_status == MOTOR_STATUS_STOPPED &&
	    motor->zoom.move_status == MOTOR_STATUS_STOPPED)
		motor->is_timer_restart = false;

	if (motor->is_timer_restart) {
		spin_unlock_irqrestore(&motor->spin_lock, lock_flags);
		do_gettimeofday(&tv);
		time_dist = tv.tv_sec * 1000000 + tv.tv_usec - (tv_last.tv_sec * 1000000 + tv_last.tv_usec);
		hrtimer_forward_now(timer,
				    ns_to_ktime(motor->phase_interval_ns + (time_dist * 1000)));
		return HRTIMER_RESTART;
	} else {
		spin_unlock_irqrestore(&motor->spin_lock, lock_flags);
	}
	return HRTIMER_NORESTART;
}

static void wait_for_motor_stop(struct motor_dev *motor, struct ext_dev *dev)
{
	unsigned long ret = 0;

	if (dev->is_running) {
		reinit_completion(&dev->comp_out);
		ret = wait_for_completion_timeout(&dev->comp_out, 16 * HZ);
		if (ret == 0)
			dev_info(motor->dev,
				 "wait for complete timeout\n");
	}
}

static int motor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct motor_dev *motor = container_of(ctrl->handler,
					     struct motor_dev, ctrl_handler);
	bool is_need_reback = false;

	switch (ctrl->id) {
	case V4L2_CID_FOCUS_ABSOLUTE:
		if (motor->focus.reback_ctrl) {
			if (ctrl->val >= motor->focus.last_pos)
				is_need_reback = false;
			else
				is_need_reback = true;
		}
		ret = set_motor_running_status(motor,
					       &motor->focus,
					       ctrl->val,
					       is_need_reback);
		wait_for_motor_stop(motor, &motor->focus);
		dev_dbg(motor->dev, "set focus pos %d\n", ctrl->val);
		break;
	case V4L2_CID_ZOOM_ABSOLUTE:
		if (motor->zoom.reback_ctrl) {
			if (ctrl->val >= motor->zoom.last_pos)
				is_need_reback = false;
			else
				is_need_reback = true;
		}
		ret = set_motor_running_status(motor,
					       &motor->zoom,
					       ctrl->val,
					       is_need_reback);
		wait_for_motor_stop(motor, &motor->zoom);
		dev_dbg(motor->dev, "set zoom pos %d\n", ctrl->val);
		break;
	default:
		dev_err(motor->dev, "not support cmd %d\n", ctrl->id);
		break;
	}
	return ret;
}

static int motor_set_zoom_follow(struct motor_dev *motor, struct rk_cam_set_zoom *mv_param)
{
	int i = 0;
	bool is_need_zoom_reback = mv_param->is_need_zoom_reback;
	bool is_need_focus_reback = mv_param->is_need_focus_reback;

	for (i = 0; i < mv_param->setzoom_cnt; i++) {
		if (i == (mv_param->setzoom_cnt - 1)) {
			set_motor_running_status(motor,
						 &motor->focus,
						 mv_param->zoom_pos[i].focus_pos,
						 is_need_focus_reback);
			set_motor_running_status(motor,
						 &motor->zoom,
						 mv_param->zoom_pos[i].zoom_pos,
						 is_need_zoom_reback);
		} else {
			set_motor_running_status(motor,
						 &motor->focus,
						 mv_param->zoom_pos[i].focus_pos,
						 false);
			set_motor_running_status(motor,
						 &motor->zoom,
						 mv_param->zoom_pos[i].zoom_pos,
						 false);
		}
		dev_dbg(motor->dev,
			"%s zoom %d, focus %d, i %d\n",
			__func__,
			mv_param->zoom_pos[i].zoom_pos,
			mv_param->zoom_pos[i].focus_pos,
			i);
	}
	wait_for_motor_stop(motor, &motor->focus);
	wait_for_motor_stop(motor, &motor->zoom);
	return 0;
}

static int motor_set_focus(struct motor_dev *motor, struct rk_cam_set_focus *mv_param)
{
	int ret = 0;
	bool is_need_reback = mv_param->is_need_reback;

#ifdef REBACK_CTRL_BY_DRV
	if (mv_param->focus_pos >= motor->focus.last_pos)
		is_need_reback = false;
	else
		is_need_reback = true;
#endif
	ret = set_motor_running_status(motor,
				       &motor->focus,
				       mv_param->focus_pos,
				       is_need_reback);
	wait_for_motor_stop(motor, &motor->focus);

	return ret;
}

static int motor_reinit_focus(struct motor_dev *motor)
{
	int ret = 0;

	motor->focus.last_pos = motor->focus.step_max;
	ret = set_motor_running_status(motor,
				       &motor->focus,
				       0,
				       true);
	wait_for_motor_stop(motor, &motor->focus);

	return ret;
}

static void motor_reinit_focus_pos(struct motor_dev *motor)
{
	motor_reinit_focus(motor);
	motor->focus.last_pos = 0;
	__v4l2_ctrl_modify_range(motor->focus_ctrl, 0,
				 motor->focus.step_max - motor->focus.reback,
				 1, 0);
}

static int  motor_reinit_zoom(struct motor_dev *motor)
{
	int ret = 0;

	motor->zoom.last_pos = motor->zoom.step_max;
	ret = set_motor_running_status(motor,
				       &motor->zoom,
				       0,
				       true);
	wait_for_motor_stop(motor, &motor->zoom);
	return ret;
}

static void motor_reinit_zoom_pos(struct motor_dev *motor)
{
	motor_reinit_zoom(motor);
	motor->zoom.last_pos = 0;
	__v4l2_ctrl_modify_range(motor->zoom_ctrl, 0,
				 motor->zoom.step_max - motor->zoom.reback,
				 1, 0);
}

static long motor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct rk_cam_vcm_tim *mv_tim;
	struct motor_dev *motor = container_of(sd, struct motor_dev, sd);
	struct rk_cam_set_zoom *mv_param;
	struct rk_cam_set_focus *focus_param;
	struct rk_cam_modify_pos *pos;

	switch (cmd) {
	case RK_VIDIOC_VCM_TIMEINFO:
		mv_tim = (struct rk_cam_vcm_tim *)arg;
		memcpy(mv_tim, &motor->focus.mv_tim, sizeof(*mv_tim));

		dev_dbg(motor->dev, "get_focus_move_tim 0x%lx, 0x%lx, 0x%lx, 0x%lx\n",
			mv_tim->vcm_start_t.tv_sec,
			mv_tim->vcm_start_t.tv_usec,
			mv_tim->vcm_end_t.tv_sec,
			mv_tim->vcm_end_t.tv_usec);
		break;
	case RK_VIDIOC_ZOOM_TIMEINFO:
		mv_tim = (struct rk_cam_vcm_tim *)arg;
		memcpy(mv_tim, &motor->zoom.mv_tim, sizeof(*mv_tim));

		dev_dbg(motor->dev, "get_zoom_move_tim 0x%lx, 0x%lx, 0x%lx, 0x%lx\n",
			mv_tim->vcm_start_t.tv_sec,
			mv_tim->vcm_start_t.tv_usec,
			mv_tim->vcm_end_t.tv_sec,
			mv_tim->vcm_end_t.tv_usec);
		break;
	case RK_VIDIOC_FOCUS_CORRECTION:
		motor_reinit_focus_pos(motor);
		break;
	case RK_VIDIOC_ZOOM_CORRECTION:
		motor_reinit_zoom_pos(motor);
		break;
	case RK_VIDIOC_ZOOM_SET_POSITION:
		mv_param = (struct rk_cam_set_zoom *)arg;
		motor_set_zoom_follow(motor, mv_param);
		break;
	case RK_VIDIOC_FOCUS_SET_POSITION:
		focus_param = (struct rk_cam_set_focus *)arg;
		motor_set_focus(motor, focus_param);
		break;
	case RK_VIDIOC_MODIFY_POSITION:
		pos = (struct rk_cam_modify_pos *)arg;
		motor->focus.last_pos = pos->focus_pos;
		motor->zoom.last_pos = pos->zoom_pos;
		break;
	default:
		break;
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static long motor_compat_ioctl32(struct v4l2_subdev *sd, unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rk_cam_vcm_tim *mv_tim;
	int ret = 0;
	u32 val = 0;
	struct rk_cam_set_zoom *mv_param;
	struct rk_cam_set_focus *focus_param;
	struct rk_cam_modify_pos *pos;

	switch (cmd) {
	case RK_VIDIOC_VCM_TIMEINFO:
	case RK_VIDIOC_ZOOM_TIMEINFO:
		mv_tim = kzalloc(sizeof(*mv_tim), GFP_KERNEL);
		if (!mv_tim) {
			ret = -ENOMEM;
			return ret;
		}
		ret = motor_ioctl(sd, cmd, mv_tim);
		if (!ret) {
			if (copy_to_user(up, mv_tim, sizeof(*mv_tim))) {
				kfree(mv_tim);
				return -EFAULT;
			}
		}
		kfree(mv_tim);
		break;
	case RK_VIDIOC_FOCUS_CORRECTION:
	case RK_VIDIOC_ZOOM_CORRECTION:
		if (copy_from_user(&val, up, sizeof(val)))
			return -EFAULT;
		ret = motor_ioctl(sd, cmd, &val);
		break;
	case RK_VIDIOC_ZOOM_SET_POSITION:
		mv_param = kzalloc(sizeof(*mv_param), GFP_KERNEL);
		if (!mv_param) {
			ret = -ENOMEM;
			return ret;
		}
		if (copy_from_user(mv_param, up, sizeof(*mv_param))) {
			kfree(mv_param);
			return -EFAULT;
		}
		ret = motor_ioctl(sd, cmd, mv_param);
		kfree(mv_param);
		break;
	case RK_VIDIOC_FOCUS_SET_POSITION:
		focus_param = kzalloc(sizeof(*focus_param), GFP_KERNEL);
		if (!focus_param) {
			ret = -ENOMEM;
			return ret;
		}
		if (copy_from_user(focus_param, up, sizeof(*focus_param))) {
			kfree(focus_param);
			return -EFAULT;
		}
		ret = motor_ioctl(sd, cmd, focus_param);
		kfree(focus_param);
		break;
	case RK_VIDIOC_MODIFY_POSITION:
		pos = kzalloc(sizeof(*pos), GFP_KERNEL);
		if (!pos) {
			ret = -ENOMEM;
			return ret;
		}
		if (copy_from_user(pos, up, sizeof(*pos))) {
			kfree(pos);
			return -EFAULT;
		}
		ret = motor_ioctl(sd, cmd, pos);
		kfree(pos);
		break;
	default:
		break;
	}
	return ret;
}
#endif

static int motor_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct motor_dev *motor = container_of(ctrl->handler,
					     struct motor_dev, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_FOCUS_ABSOLUTE:
		ctrl->val = motor->focus.last_pos;
		return 0;
	case V4L2_CID_ZOOM_ABSOLUTE:
		ctrl->val = motor->zoom.last_pos;
		return 0;
	}
	return 0;
}

static const struct v4l2_subdev_core_ops motor_core_ops = {
	.ioctl = motor_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = motor_compat_ioctl32
#endif
};

static const struct v4l2_subdev_ops motor_subdev_ops = {
	.core	= &motor_core_ops,
};

static const struct v4l2_ctrl_ops motor_ctrl_ops = {
	.g_volatile_ctrl = motor_g_volatile_ctrl,
	.s_ctrl = motor_s_ctrl,
};

static int motor_initialize_controls(struct motor_dev *motor)
{
	struct v4l2_ctrl_handler *handler;
	int ret = 0;
	unsigned long flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE | V4L2_CTRL_FLAG_VOLATILE;

	handler = &motor->ctrl_handler;
	ret = v4l2_ctrl_handler_init(handler, 2);
	if (ret)
		return ret;
	#ifdef REINIT_BOOT
	ret = motor_reinit_focus(motor);
	if (ret < 0)
		return -EINVAL;
	#endif
	motor->focus.last_pos = 0;
	motor->focus_ctrl = v4l2_ctrl_new_std(handler, &motor_ctrl_ops,
			    V4L2_CID_FOCUS_ABSOLUTE, 0,
			    motor->focus.step_max - motor->focus.reback,
			    1, 0);
	if (motor->focus_ctrl)
		motor->focus_ctrl->flags |= flags;
	#ifdef REINIT_BOOT
	ret = motor_reinit_zoom(motor);
	if (ret < 0)
		return -EINVAL;
	#endif
	motor->zoom.last_pos = 0;
	motor->zoom_ctrl = v4l2_ctrl_new_std(handler, &motor_ctrl_ops,
			    V4L2_CID_ZOOM_ABSOLUTE,
			    0,
			    motor->zoom.step_max - motor->zoom.reback,
			    1, 0);
	if (motor->zoom_ctrl)
		motor->zoom_ctrl->flags |= flags;
	if (handler->error) {
		ret = handler->error;
		dev_err(motor->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	motor->sd.ctrl_handler = handler;
	return ret;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}
#define USED_SYS_DEBUG
#ifdef USED_SYS_DEBUG

static ssize_t reinit_focus_pos(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = container_of(sd, struct motor_dev, sd);
	int val = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (val == 1)
			motor_reinit_focus_pos(motor);
	}
	return count;
}

static ssize_t reinit_zoom_pos(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = container_of(sd, struct motor_dev, sd);
	int val = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (val == 1)
			motor_reinit_zoom_pos(motor);
	}
	return count;
}

static ssize_t set_focus_reback_ctrl(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = container_of(sd, struct motor_dev, sd);
	int val = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (val == 1)
			motor->focus.reback_ctrl = true;
		else
			motor->focus.reback_ctrl = false;
	}
	return count;
}

static ssize_t set_zoom_reback_ctrl(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = container_of(sd, struct motor_dev, sd);
	int val = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (val == 1)
			motor->zoom.reback_ctrl = true;
		else
			motor->zoom.reback_ctrl = false;
	}
	return count;
}

static ssize_t set_motor_speed(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = container_of(sd, struct motor_dev, sd);
	int val = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (val < 100 || val > 600)
			dev_info(motor->dev,
				"min speed 100pps, max speed 600pps, cur speed %d pps\n",
				val);
		else
			motor->phase_interval_ns =
				div_u64(NSEC_PER_SEC, val);
	}
	return count;
}

static struct device_attribute attributes[] = {
	__ATTR(reinit_focus, S_IWUSR, NULL, reinit_focus_pos),
	__ATTR(reinit_zoom, S_IWUSR, NULL, reinit_zoom_pos),
	__ATTR(focus_reback_ctrl, S_IWUSR, NULL, set_focus_reback_ctrl),
	__ATTR(zoom_reback_ctrl, S_IWUSR, NULL, set_zoom_reback_ctrl),
	__ATTR(speed, S_IWUSR, NULL, set_motor_speed),
};

static int add_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto undo;
	return 0;
undo:
	for (i--; i >= 0 ; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}
#endif

static void dev_param_init(struct motor_dev *motor)
{
	motor->focus.type = TYPE_FOCUS;
	motor->zoom.type = TYPE_ZOOM;
	motor->phase_interval_ns =
		div_u64(NSEC_PER_SEC, motor->start_up_speed);
	motor->focus.mv_tim.vcm_start_t = ns_to_timeval(ktime_get_ns());
	motor->focus.mv_tim.vcm_end_t = ns_to_timeval(ktime_get_ns());
	motor->focus.move_status = MOTOR_STATUS_STOPPED;
	motor->focus.last_status = MOTOR_STATUS_STOPPED;
	init_completion(&motor->focus.comp_in);
	init_completion(&motor->focus.comp_out);
	motor->focus.is_running = false;
	motor->focus.reback_ctrl = false;
	motor->focus.phase_index = 0;
	if (motor->focus.reback != 0) {
		motor->focus.cur_back_delay = 0;
		if (motor->focus.is_dir_opp)
			motor->focus.reback_status = MOTOR_STATUS_CCW;
		else
			motor->focus.reback_status = MOTOR_STATUS_CW;
	}

	motor->zoom.mv_tim.vcm_start_t = ns_to_timeval(ktime_get_ns());
	motor->zoom.mv_tim.vcm_end_t = ns_to_timeval(ktime_get_ns());
	motor->zoom.move_status = MOTOR_STATUS_STOPPED;
	motor->zoom.last_status = MOTOR_STATUS_STOPPED;
	init_completion(&motor->zoom.comp_in);
	init_completion(&motor->zoom.comp_out);
	motor->zoom.is_running = false;
	motor->zoom.reback_ctrl = false;
	motor->zoom.phase_index = 0;
	if (motor->zoom.reback != 0) {
		motor->zoom.cur_back_delay = 0;
		if (motor->zoom.is_dir_opp)
			motor->zoom.reback_status = MOTOR_STATUS_CCW;
		else
			motor->zoom.reback_status = MOTOR_STATUS_CW;
	}
	spin_lock_init(&motor->spin_lock);
	motor->is_timer_restart = false;
}

static int motor_dev_parse_dt(struct motor_dev *motor)
{
	struct device_node *node = motor->dev->of_node;
	int ret = 0;
	int i = 0;

	motor->focus.phase_gpios[i] = devm_gpiod_get(motor->dev,
						     "a1", GPIOD_OUT_LOW);
	if (IS_ERR(motor->focus.phase_gpios[i])) {
		dev_err(motor->dev, "Failed to get focus control gpios\n");
		return -EINVAL;
	}
	i++;
	motor->focus.phase_gpios[i] = devm_gpiod_get(motor->dev,
						     "b1", GPIOD_OUT_LOW);
	if (IS_ERR(motor->focus.phase_gpios[i])) {
		dev_err(motor->dev, "Failed to get focus control gpios\n");
		return -EINVAL;
	}
	i++;
	motor->focus.phase_gpios[i] = devm_gpiod_get(motor->dev,
						     "a2", GPIOD_OUT_LOW);
	if (IS_ERR(motor->focus.phase_gpios[i])) {
		dev_err(motor->dev, "Failed to get focus control gpios\n");
		return -EINVAL;
	}
	i++;
	motor->focus.phase_gpios[i] = devm_gpiod_get(motor->dev,
						     "b2", GPIOD_OUT_LOW);
	if (IS_ERR(motor->focus.phase_gpios[i])) {
		dev_err(motor->dev, "Failed to get focus control gpios\n");
		return -EINVAL;
	}
	i = 0;
	motor->zoom.phase_gpios[i] = devm_gpiod_get(motor->dev,
						     "c1", GPIOD_OUT_LOW);
	if (IS_ERR(motor->focus.phase_gpios[i])) {
		dev_err(motor->dev, "Failed to get focus control gpios\n");
		return -EINVAL;
	}
	i++;
	motor->zoom.phase_gpios[i] = devm_gpiod_get(motor->dev,
						     "d1", GPIOD_OUT_LOW);
	if (IS_ERR(motor->focus.phase_gpios[i])) {
		dev_err(motor->dev, "Failed to get focus control gpios\n");
		return -EINVAL;
	}
	i++;
	motor->zoom.phase_gpios[i] = devm_gpiod_get(motor->dev,
						     "c2", GPIOD_OUT_LOW);
	if (IS_ERR(motor->focus.phase_gpios[i])) {
		dev_err(motor->dev, "Failed to get focus control gpios\n");
		return -EINVAL;
	}
	i++;
	motor->zoom.phase_gpios[i] = devm_gpiod_get(motor->dev,
						     "d2", GPIOD_OUT_LOW);
	if (IS_ERR(motor->focus.phase_gpios[i])) {
		dev_err(motor->dev, "Failed to get focus control gpios\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node,
				   "focus-step-max",
				   &motor->focus.step_max);
	if (ret != 0) {
		motor->focus.step_max = FOCUS_MAX_STEP_DEF;
		dev_err(motor->dev,
			"failed get iris focus_pos_max,use dafult value 7500\n");
	}

	ret = of_property_read_u32(node,
				   "zoom-step-max",
				   &motor->zoom.step_max);
	if (ret != 0) {
		motor->zoom.step_max = ZOOM_MAX_STEP_DEF;
		dev_err(motor->dev,
			"failed get iris zoom_pos_max,use dafult value 7500\n");
	}

	ret = of_property_read_u32(node,
				   "motor-start-up-speed",
				   &motor->start_up_speed);
	if (ret != 0) {
		motor->start_up_speed = MAX_START_UP_HZ;
		dev_err(motor->dev,
			"failed get motor start up speed,use dafult value\n");
	}

	ret = of_property_read_u32(node,
				   "focus-reback-distance",
				   &motor->focus.reback);
	if (ret != 0) {
		dev_err(motor->dev,
			"failed get focus reback distance, return\n");
		return -EINVAL;
	}
	ret = of_property_read_u32(node,
				   "focus-reback-delay",
				   &motor->focus.max_back_delay);
	if (ret != 0) {
		motor->focus.max_back_delay = FOCUS_MAX_BACK_DELAY;
		dev_err(motor->dev,
			"failed get focus reback delay, used default\n");
	}
	ret = of_property_read_u32(node,
				   "zoom-reback-distance",
				   &motor->zoom.reback);
	if (ret != 0) {
		dev_err(motor->dev,
			"failed get zoom reback distance, return\n");
		return -EINVAL;
	}
	ret = of_property_read_u32(node,
				   "zoom-reback-delay",
				   &motor->zoom.max_back_delay);
	if (ret != 0) {
		motor->zoom.max_back_delay = ZOOM_MAX_BACK_DELAY;
		dev_err(motor->dev,
			"failed get zoom reback delay, used default\n");
	}

	motor->focus.is_dir_opp =
		device_property_read_bool(motor->dev, "focus-dir-opposite");
	motor->zoom.is_dir_opp =
		device_property_read_bool(motor->dev, "zoom-dir-opposite");
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &motor->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &motor->module_facing);
	if (ret) {
		dev_err(motor->dev,
			"could not get module information!\n");
		return -EINVAL;
	}
	return 0;
}

static int motor_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct motor_dev *motor;
	struct v4l2_subdev *sd;
	char facing[2];

	dev_info(&pdev->dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);
	motor = devm_kzalloc(&pdev->dev, sizeof(*motor), GFP_KERNEL);
	if (!motor)
		return -ENOMEM;
	motor->dev = &pdev->dev;
	dev_set_name(motor->dev, "motor");
	dev_set_drvdata(motor->dev, motor);
	if (motor_dev_parse_dt(motor)) {
		dev_err(motor->dev, "parse dt error\n");
		return -EINVAL;
	}
	dev_param_init(motor);
	hrtimer_init(&motor->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
	motor->timer.function = motor_timer_func;
	v4l2_subdev_init(&motor->sd, &motor_subdev_ops);
	motor->sd.owner = pdev->dev.driver->owner;
	motor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	motor->sd.dev = &pdev->dev;
	v4l2_set_subdevdata(&motor->sd, pdev);
	platform_set_drvdata(pdev, &motor->sd);
	motor_initialize_controls(motor);
	if (ret)
		goto err_free;
	ret = media_entity_pads_init(&motor->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_free;
	sd = &motor->sd;
	sd->entity.function = MEDIA_ENT_F_LENS;
	sd->entity.flags = 0;

	memset(facing, 0, sizeof(facing));
	if (strcmp(motor->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';
	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s",
		 motor->module_index, facing,
		 DRIVER_NAME);
	ret = v4l2_async_register_subdev(sd);
	if (ret)
		dev_err(&pdev->dev, "v4l2 async register subdev failed\n");
#ifdef USED_SYS_DEBUG
	add_sysfs_interfaces(&pdev->dev);
#endif
	dev_info(motor->dev, "gpio motor driver probe success\n");
	return 0;
err_free:
	v4l2_ctrl_handler_free(&motor->ctrl_handler);
	v4l2_device_unregister_subdev(&motor->sd);
	media_entity_cleanup(&motor->sd.entity);
	return ret;
}

static int motor_dev_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct motor_dev *motor;

	if (sd)
		motor = v4l2_get_subdevdata(sd);
	else
		return -ENODEV;
	hrtimer_cancel(&motor->timer);
	if (sd)
		v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&motor->ctrl_handler);
	media_entity_cleanup(&motor->sd.entity);
#ifdef USED_SYS_DEBUG
	remove_sysfs_interfaces(motor->dev);
#endif
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id motor_dev_of_match[] = {
	{ .compatible = "fitipower,fp5501", },
	{},
};
#endif

static struct platform_driver motor_dev_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(motor_dev_of_match),
	},
	.probe = motor_dev_probe,
	.remove = motor_dev_remove,
};

module_platform_driver(motor_dev_driver);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:motor");
MODULE_AUTHOR("ROCKCHIP");
