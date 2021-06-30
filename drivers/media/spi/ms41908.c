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
#include <linux/mutex.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <linux/completion.h>
#include "linux/rk_vcm_head.h"
#include <linux/time.h>
#include <linux/semaphore.h>

#define DRIVER_VERSION	KERNEL_VERSION(0, 0x01, 0x00)

#define DRIVER_NAME "ms41908"

#define PSUMAB	0X24
#define INTCTAB	0X25
#define PSUMCD	0X29
#define INTCTCD	0X2A

#define START_UP_HZ_DEF			(800)
#define PIRIS_MAX_STEP_DEF		(80)
#define FOCUS_MAX_STEP_DEF		(3060)
#define ZOOM_MAX_STEP_DEF		(1520)

#define DCIRIS_MAX_LOG			1023

#define VD_FZ_US			10000

#define PPW_DEF				0xff
#define MICRO_DEF			64
#define PHMODE_DEF			0
#define PPW_STOP			0x00

#define FOCUS_MAX_BACK_DELAY		4
#define ZOOM_MAX_BACK_DELAY		4
#define ZOOM1_MAX_BACK_DELAY		4

#define to_motor_dev(sd) container_of(sd, struct motor_dev, subdev)

enum {
	MOTOR_STATUS_STOPPED = 0,
	MOTOR_STATUS_CCW = 1,
	MOTOR_STATUS_CW = 2,
};

enum ext_dev_type {
	TYPE_IRIS = 0,
	TYPE_FOCUS = 1,
	TYPE_ZOOM = 2,
	TYPE_ZOOM1 = 3,
};

struct motor_reg_s {
	u16 dt2_phmod;
	u16 ppw;
	u16 psum;
	u16 intct;
};

struct reg_op_s {
	struct motor_reg_s reg;
	u16 tmp_psum;
	bool is_used;
};

struct run_data_s {
	u32 count;
	u32 cur_count;
	u32 psum;
	u32 psum_last;
	u32 intct;
	u32 ppw;
	u32 ppw_stop;
	u32 phmode;
	u32 micro;
};

struct ext_dev {
	u8 type;
	u32 step_max;
	int last_pos;
	u32 start_up_speed;
	u32 move_status;
	u32 reback_status;
	u32 move_time_us;
	u32 reback_move_time_us;
	u32 backlash;
	int reback;
	u32 last_dir;
	int min_pos;
	int max_pos;
	bool is_half_step_mode;
	bool is_mv_tim_update;
	bool is_need_update_tim;
	bool is_dir_opp;
	bool is_need_reback;
	struct rk_cam_vcm_tim mv_tim;
	struct run_data_s run_data;
	struct run_data_s reback_data;
	struct completion complete;
	struct reg_op_s *reg_op;
	struct gpio_desc *pic_gpio;
	struct gpio_desc *pia_gpio;
	struct gpio_desc *pie_gpio;
	int cur_back_delay;
	int max_back_delay;
	struct completion complete_out;
	bool is_running;
};

struct dciris_dev {
	u32 last_log;
	u32 max_log;
	bool is_reversed_polarity;
	struct gpio_desc *vd_iris_gpio;
};

struct motor_dev {
	struct spi_device *spi;
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *iris_ctrl;
	struct v4l2_ctrl *focus_ctrl;
	struct v4l2_ctrl *zoom_ctrl;
	struct v4l2_ctrl *zoom1_ctrl;
	struct hrtimer timer;
	struct mutex mutex;
	u32 module_index;
	const char *module_facing;
	struct ext_dev *piris;
	struct ext_dev *focus;
	struct ext_dev *zoom;
	struct ext_dev *zoom1;
	struct ext_dev *dev0;
	struct ext_dev *dev1;
	bool is_use_dc_iris;
	bool is_use_p_iris;
	bool is_use_focus;
	bool is_use_zoom;
	bool is_use_zoom1;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vd_fz_gpio;
	bool is_timer_restart;
	bool is_should_wait;
	struct motor_work_s *wk;
	u32 vd_fz_period_us;
	struct reg_op_s motor_op[2];
	struct dciris_dev *dciris;
	int id;
	int wait_cnt;
};

struct motor_work_s {
	struct work_struct work;
	struct motor_dev *dev;
};

static const struct reg_op_s g_motor_op[2] = {{{0x22, 0x23, 0x24, 0x25}, 0, 0},
					   {{0x27, 0x28, 0x29, 0x2a}, 0, 0}};

static int spi_write_reg(struct spi_device *spi, u8 reg, u16 val)
{
	int ret = 0;
	u8 buf_reg = reg;
	u16 buf_val = val;

	struct spi_message msg;
	struct spi_transfer tx[] = {
		{
			.tx_buf = &buf_reg,
			.len = 1,
			.delay_usecs = 1,
		}, {
			.tx_buf = &buf_val,
			.len = 2,
			.delay_usecs = 1,
		},
	};
	spi_message_init(&msg);
	spi_message_add_tail(&tx[0], &msg);
	spi_message_add_tail(&tx[1], &msg);
	ret = spi_sync(spi, &msg);
	return ret;
}

static __maybe_unused int spi_read_reg(struct spi_device *spi, u8 reg, u16 *val)
{
	int ret = 0;
	u8 buf_reg = reg | 0x40;
	u16 buf_val = 0;

	struct spi_message msg;
	struct spi_transfer tx[] = {
		{
			.tx_buf = &buf_reg,
			.len = 1,
			.delay_usecs = 1,
		}, {
			.rx_buf = &buf_val,
			.len = 2,
			.delay_usecs = 1,
		},
	};
	spi_message_init(&msg);
	spi_message_add_tail(&tx[0], &msg);
	spi_message_add_tail(&tx[1], &msg);
	ret = spi_sync(spi, &msg);
	*val = buf_val;

	return ret;
}

static int set_motor_running_status(struct motor_dev *motor,
				    struct ext_dev *ext_dev,
				    s32 pos,
				    bool is_need_update_tim,
				    bool is_should_wait,
				    bool is_need_reback)
{
	int ret = 0;
	u32 step = 0;
	u16 psum = 0;
	struct run_data_s run_data = ext_dev->run_data;
	u32 micro = 0;
	u32 mv_cnt = 0;
	int status = 0;

	if (ext_dev->move_status != MOTOR_STATUS_STOPPED)
		wait_for_completion(&ext_dev->complete);
	ext_dev->is_mv_tim_update = false;

	ext_dev->move_time_us = 0;
	mv_cnt = abs(pos - ext_dev->last_pos);
	if (is_need_reback)
		mv_cnt += ext_dev->reback;
	dev_dbg(&motor->spi->dev,
		"dev type %d pos %d, last_pos %d, mv_cnt %d, status %d is_need_reback %d\n",
		ext_dev->type, pos, ext_dev->last_pos, mv_cnt, status, is_need_reback);
	if (mv_cnt == 0) {
		mutex_lock(&motor->mutex);
		if (is_need_update_tim) {
			ext_dev->mv_tim.vcm_start_t = ns_to_timeval(ktime_get_ns());
			ext_dev->mv_tim.vcm_end_t = ext_dev->mv_tim.vcm_start_t;
			ext_dev->is_mv_tim_update = true;
		}
		if (is_should_wait) {
			motor->wait_cnt++;
		} else if (motor->is_timer_restart == false && motor->wait_cnt) {
			hrtimer_start(&motor->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
			motor->wait_cnt = 0;
		} else {
			motor->wait_cnt = 0;
		}
		mutex_unlock(&motor->mutex);
		return 0;
	}
	ext_dev->is_running = true;
	reinit_completion(&ext_dev->complete);

	if (ext_dev->is_dir_opp) {
		if (pos > ext_dev->last_pos) {
			if (ext_dev->last_dir == MOTOR_STATUS_CCW)
				mv_cnt += ext_dev->backlash;
			status = MOTOR_STATUS_CW;
		} else {
			if (ext_dev->last_dir == MOTOR_STATUS_CW)
				mv_cnt += ext_dev->backlash;
			status = MOTOR_STATUS_CCW;
		}
	} else {
		if (pos > ext_dev->last_pos) {
			if (ext_dev->last_dir == MOTOR_STATUS_CW)
				mv_cnt += ext_dev->backlash;
			status = MOTOR_STATUS_CCW;
		} else {
			if (ext_dev->last_dir == MOTOR_STATUS_CCW)
				mv_cnt += ext_dev->backlash;
			status = MOTOR_STATUS_CW;
		}
	}

	if (ext_dev->is_half_step_mode)
		step = mv_cnt * 4;
	else
		step = mv_cnt * 8;

	run_data.count = (step + run_data.psum - 1) / run_data.psum;
	run_data.cur_count = run_data.count;
	run_data.psum_last = step % run_data.psum;
	if (run_data.psum_last == 0)
		run_data.psum_last = run_data.psum;

	switch (run_data.micro) {
	case 64:
		micro = 0x03;
		break;
	case 128:
		micro = 0x02;
		break;
	case 256:
		micro = 0x00;
		break;
	default:
		micro = 0x00;
		break;
	};
	if (run_data.count == 1)
		psum = ((status - 1) << 8) |
		       (1 << 10) |
		       (micro << 12) |
		       (run_data.psum_last);
	else
		psum = ((status - 1) << 8) |
		       (1 << 10) |
		       (micro << 12) |
		       (run_data.psum);
	mutex_lock(&motor->mutex);
	ext_dev->is_need_update_tim = is_need_update_tim;
	ext_dev->is_need_reback = is_need_reback;
	ext_dev->move_time_us = (run_data.count + 1) * (motor->vd_fz_period_us + 500);
	if (is_need_reback)
		ext_dev->move_time_us += ext_dev->reback_move_time_us;

	ext_dev->last_pos = pos;
	ext_dev->run_data = run_data;
	ext_dev->move_status = status;
	spi_write_reg(motor->spi, 0x20, 0x1a01);
	spi_write_reg(motor->spi, ext_dev->reg_op->reg.ppw, run_data.ppw | (run_data.ppw << 8));
	spi_write_reg(motor->spi, ext_dev->reg_op->reg.psum, psum);
	spi_write_reg(motor->spi, ext_dev->reg_op->reg.intct, ext_dev->run_data.intct);
	ext_dev->reg_op->tmp_psum = psum;

	ext_dev->last_dir = status;
	if (is_should_wait) {
		motor->wait_cnt++;
	} else if (motor->is_timer_restart == false) {
		hrtimer_start(&motor->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
		motor->wait_cnt = 0;
	} else {
		motor->wait_cnt = 0;
	}
	mutex_unlock(&motor->mutex);
	dev_dbg(&motor->spi->dev,
		 "ext_dev type %d move count %d, psum %d, psum_last %d, move_time_us %u!\n",
		 ext_dev->type,
		 ext_dev->run_data.count,
		 ext_dev->run_data.psum,
		 ext_dev->run_data.psum_last,
		 ext_dev->move_time_us);

	return ret;
}

static int motor_dev_parse_dt(struct motor_dev *motor)
{
	struct device_node *node = motor->spi->dev.of_node;
	int ret = 0;
	const char *str;
	int step_motor_cnt = 0;

	motor->is_use_dc_iris =
		device_property_read_bool(&motor->spi->dev, "use-dc-iris");
	motor->is_use_p_iris =
		device_property_read_bool(&motor->spi->dev, "use-p-iris");
	motor->is_use_focus =
		device_property_read_bool(&motor->spi->dev, "use-focus");
	motor->is_use_zoom =
		device_property_read_bool(&motor->spi->dev, "use-zoom");
	motor->is_use_zoom1 =
		device_property_read_bool(&motor->spi->dev, "use-zoom1");

	/* get reset gpio */
	motor->reset_gpio = devm_gpiod_get(&motor->spi->dev,
					     "reset", GPIOD_OUT_LOW);
	if (IS_ERR(motor->reset_gpio))
		dev_err(&motor->spi->dev, "Failed to get reset-gpios\n");

	/* get vd_fz gpio */
	motor->vd_fz_gpio = devm_gpiod_get(&motor->spi->dev,
					      "vd_fz", GPIOD_OUT_LOW);
	if (IS_ERR(motor->vd_fz_gpio))
		dev_info(&motor->spi->dev, "Failed to get vd_fz-gpios\n");

	ret = of_property_read_u32(node,
				   "vd_fz-period-us",
				   &motor->vd_fz_period_us);
	if (ret != 0) {
		motor->vd_fz_period_us = VD_FZ_US;
		dev_err(&motor->spi->dev,
			"failed get vd_fz-period-us,use dafult value\n");
	}

	ret = of_property_read_u32(node,
				   "id",
				   &motor->id);
	if (ret != 0) {
		motor->id = 0;
		dev_err(&motor->spi->dev,
			"failed get driver id,use dafult value\n");
	}

	if (motor->is_use_dc_iris) {
		motor->dciris = devm_kzalloc(&motor->spi->dev, sizeof(*motor->dciris), GFP_KERNEL);
		if (!motor->dciris) {
			dev_err(&motor->spi->dev,
				"__line__ %d, devm_kzalloc return fail!\n", __LINE__);
			return -ENOMEM;
		}
			/* get vd_iris gpio */
		motor->dciris->vd_iris_gpio = devm_gpiod_get(&motor->spi->dev,
					     "vd_iris", GPIOD_OUT_LOW);
		if (IS_ERR(motor->dciris->vd_iris_gpio))
			dev_info(&motor->spi->dev, "Failed to get vd_iris-gpios\n");
		motor->dciris->is_reversed_polarity =
			device_property_read_bool(&motor->spi->dev, "dc-iris-reserved-polarity");
		ret = of_property_read_u32(node,
					   "dc-iris-max-log",
					   &motor->dciris->max_log);
		if (ret != 0) {
			motor->dciris->max_log = DCIRIS_MAX_LOG;
			dev_err(&motor->spi->dev,
				"failed get dc-iris max log,use dafult value\n");
		}
	}

	if (motor->is_use_p_iris) {
		if (motor->is_use_dc_iris) {
			dev_err(&motor->spi->dev,
				"Does not support p-iris and dc-iris on the same module\n");
			return -EINVAL;
		}
		step_motor_cnt++;
		motor->piris = devm_kzalloc(&motor->spi->dev, sizeof(*motor->piris), GFP_KERNEL);
		if (!motor->piris) {
			dev_err(&motor->spi->dev,
				"__line__ %d, devm_kzalloc return fail!\n", __LINE__);
			return -ENOMEM;
		}
		ret = of_property_read_string(node, "piris-used-pin",
					       &str);
		if (ret != 0) {
			dev_err(&motor->spi->dev,
				"get piris-used-pin fail, please check it!\n");
			return -EINVAL;
		}
		if (strcmp(str, "ab") == 0) {
			motor->piris->reg_op = &motor->motor_op[0];
			if (motor->piris->reg_op->is_used) {
				dev_err(&motor->spi->dev,
					"__line__ %d, pin already been used\n", __LINE__);
				return -EINVAL;
			}
			motor->piris->reg_op->is_used = true;
		} else if (strcmp(str, "cd") == 0) {
			motor->piris->reg_op = &motor->motor_op[1];
			if (motor->piris->reg_op->is_used) {
				dev_err(&motor->spi->dev,
					"__line__ %d, pin already been used\n", __LINE__);
				return -EINVAL;
			}
			motor->piris->reg_op->is_used = true;
		} else {
			dev_err(&motor->spi->dev,
				"__line__ %d, pin require error\n", __LINE__);
			return -EINVAL;
		}
		ret = of_property_read_u32(node,
					   "piris-backlash",
					   &motor->piris->backlash);
		if (ret != 0) {
			motor->piris->backlash = 0;
			dev_err(&motor->spi->dev,
				"failed get motor backlash,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "piris-start-up-speed",
					   &motor->piris->start_up_speed);
		if (ret != 0) {
			motor->piris->start_up_speed = START_UP_HZ_DEF;
			dev_err(&motor->spi->dev,
				"failed get motor start up speed,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "piris-step-max",
					   &motor->piris->step_max);
		if (ret != 0) {
			motor->piris->step_max = PIRIS_MAX_STEP_DEF;
			dev_err(&motor->spi->dev,
				"failed get piris pos_max,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "piris-ppw",
					   &motor->piris->run_data.ppw);
		if (ret != 0 || (motor->piris->run_data.ppw > 0xff)) {
			motor->piris->run_data.ppw = PPW_DEF;
			dev_err(&motor->spi->dev,
				"failed get piris ppw,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "piris-ppw-stop",
					   &motor->piris->run_data.ppw_stop);
		if (ret != 0 || (motor->piris->run_data.ppw_stop > 0xff)) {
			motor->piris->run_data.ppw_stop = PPW_STOP;
			dev_err(&motor->spi->dev,
				"failed get piris ppw_stop,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "piris-phmode",
					   &motor->piris->run_data.phmode);
		if (ret != 0 || (motor->piris->run_data.phmode > 0x3f)) {
			motor->piris->run_data.phmode = PHMODE_DEF;
			dev_err(&motor->spi->dev,
				"failed get piris phmode,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "piris-micro",
					   &motor->piris->run_data.micro);
		if (ret != 0) {
			motor->piris->run_data.micro = MICRO_DEF;
			dev_err(&motor->spi->dev,
				"failed get piris micro,use dafult value\n");
		}
		/* get piris pi gpio */
		motor->piris->pic_gpio = devm_gpiod_get(&motor->spi->dev,
							"piris_pic", GPIOD_IN);
		if (IS_ERR(motor->piris->pic_gpio))
			dev_err(&motor->spi->dev, "Failed to get piris-pi-c-gpios\n");
		motor->piris->pia_gpio = devm_gpiod_get(&motor->spi->dev,
							"piris_pia", GPIOD_OUT_LOW);
		if (IS_ERR(motor->piris->pia_gpio))
			dev_err(&motor->spi->dev, "Failed to get piris-pi-a-gpios\n");
		motor->piris->pie_gpio = devm_gpiod_get(&motor->spi->dev,
							"piris_pie", GPIOD_OUT_LOW);
		if (IS_ERR(motor->piris->pie_gpio))
			dev_err(&motor->spi->dev, "Failed to get piris-pi-e-gpios\n");
		motor->piris->is_half_step_mode =
			device_property_read_bool(&motor->spi->dev, "piris-1-2phase-excitation");
		motor->piris->is_dir_opp =
			device_property_read_bool(&motor->spi->dev, "piris-dir-opposite");
		ret = of_property_read_s32(node,
					   "piris-min-pos",
					   &motor->piris->min_pos);
		if (ret != 0) {
			motor->piris->min_pos = 0;
			dev_err(&motor->spi->dev,
				"failed get piris min pos,use dafult value\n");
		}
		ret = of_property_read_s32(node,
					   "piris-max-pos",
					   &motor->piris->max_pos);
		if (ret != 0) {
			motor->piris->max_pos = motor->piris->step_max;
			dev_err(&motor->spi->dev,
				"failed get piris max_pos pos,use dafult value\n");
		}
		if (step_motor_cnt == 1)
			motor->dev0 = motor->piris;
		else if (step_motor_cnt == 2)
			motor->dev1 = motor->piris;
		ret = of_property_read_u32(node,
					   "piris-reback-distance",
					   &motor->piris->reback);
		if (ret != 0) {
			dev_err(&motor->spi->dev,
				"failed get piris reback distance, return\n");
			return -EINVAL;
		}
	}

	if (motor->is_use_focus) {
		step_motor_cnt++;
		motor->focus = devm_kzalloc(&motor->spi->dev, sizeof(*motor->focus), GFP_KERNEL);
		if (!motor->focus) {
			dev_err(&motor->spi->dev,
				"__line__ %d, devm_kzalloc return fail!\n", __LINE__);
			return -ENOMEM;
		}
		ret = of_property_read_string(node, "focus-used-pin",
					       &str);
		if (ret != 0) {
			dev_err(&motor->spi->dev,
				"get focus-used-pin fail, please check it!\n");
			return -EINVAL;
		}
		if (strcmp(str, "ab") == 0) {
			motor->focus->reg_op = &motor->motor_op[0];
			if (motor->focus->reg_op->is_used) {
				dev_err(&motor->spi->dev,
					"__line__ %d, pin already been used\n", __LINE__);
				return -EINVAL;
			}
			motor->focus->reg_op->is_used = true;
		} else if (strcmp(str, "cd") == 0) {
			motor->focus->reg_op = &motor->motor_op[1];
			if (motor->focus->reg_op->is_used) {
				dev_err(&motor->spi->dev,
					"__line__ %d, pin already been used\n", __LINE__);
				return -EINVAL;
			}
			motor->focus->reg_op->is_used = true;
		} else {
			dev_err(&motor->spi->dev,
				"__line__ %d, pin require error\n", __LINE__);
			return -EINVAL;
		}
		ret = of_property_read_u32(node,
					   "focus-backlash",
					   &motor->focus->backlash);
		if (ret != 0) {
			motor->focus->backlash = 0;
			dev_err(&motor->spi->dev,
				"failed get motor backlash,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "focus-start-up-speed",
					   &motor->focus->start_up_speed);
		if (ret != 0) {
			motor->focus->start_up_speed = START_UP_HZ_DEF;
			dev_err(&motor->spi->dev,
				"failed get motor start up speed,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "focus-step-max",
					   &motor->focus->step_max);
		if (ret != 0) {
			motor->focus->step_max = FOCUS_MAX_STEP_DEF;
			dev_err(&motor->spi->dev,
				"failed get focus_pos_max,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "focus-ppw",
					   &motor->focus->run_data.ppw);
		if (ret != 0 || (motor->focus->run_data.ppw > 0xff)) {
			motor->focus->run_data.ppw = PPW_DEF;
			dev_err(&motor->spi->dev,
				"failed get focus ppw,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "focus-ppw-stop",
					   &motor->focus->run_data.ppw_stop);
		if (ret != 0 || (motor->focus->run_data.ppw_stop > 0xff)) {
			motor->focus->run_data.ppw_stop = PPW_STOP;
			dev_err(&motor->spi->dev,
				"failed get focus ppw_stop,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "focus-phmode",
					   &motor->focus->run_data.phmode);
		if (ret != 0 || (motor->focus->run_data.phmode > 0x3f)) {
			motor->focus->run_data.phmode = PHMODE_DEF;
			dev_err(&motor->spi->dev,
				"failed get focus phmode,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "focus-micro",
					   &motor->focus->run_data.micro);
		if (ret != 0) {
			motor->focus->run_data.micro = MICRO_DEF;
			dev_err(&motor->spi->dev,
				"failed get focus micro,use dafult value\n");
		}
		if (step_motor_cnt == 1)
			motor->dev0 = motor->focus;
		else if (step_motor_cnt == 2)
			motor->dev1 = motor->focus;
		/* get focus pi gpio */
		motor->focus->pic_gpio = devm_gpiod_get(&motor->spi->dev,
							"focus_pic", GPIOD_IN);
		if (IS_ERR(motor->focus->pic_gpio))
			dev_err(&motor->spi->dev, "Failed to get focus-pi-c-gpios\n");
		motor->focus->pia_gpio = devm_gpiod_get(&motor->spi->dev,
							"focus_pia", GPIOD_OUT_LOW);
		if (IS_ERR(motor->focus->pia_gpio))
			dev_err(&motor->spi->dev, "Failed to get focus-pi-a-gpios\n");
		motor->focus->pie_gpio = devm_gpiod_get(&motor->spi->dev,
							"focus_pie", GPIOD_OUT_LOW);
		if (IS_ERR(motor->focus->pie_gpio))
			dev_err(&motor->spi->dev, "Failed to get focus-pi-e-gpios\n");
		ret = of_property_read_u32(node,
					   "focus-reback-distance",
					   &motor->focus->reback);
		if (ret != 0) {
			dev_err(&motor->spi->dev,
				"failed get focus reback distance, return\n");
			return -EINVAL;
		}
		motor->focus->is_half_step_mode =
			device_property_read_bool(&motor->spi->dev, "focus-1-2phase-excitation");
		motor->focus->is_dir_opp =
			device_property_read_bool(&motor->spi->dev, "focus-dir-opposite");
		ret = of_property_read_s32(node,
					   "focus-min-pos",
					   &motor->focus->min_pos);
		if (ret != 0) {
			motor->focus->min_pos = 0;
			dev_err(&motor->spi->dev,
				"failed get focus min pos,use dafult value\n");
		}
		ret = of_property_read_s32(node,
					   "focus-max-pos",
					   &motor->focus->max_pos);
		if (ret != 0) {
			motor->focus->max_pos = motor->focus->step_max;
			dev_err(&motor->spi->dev,
				"failed get focus max_pos pos,use dafult value\n");
		}
	}

	if (motor->is_use_zoom) {
		if (step_motor_cnt >= 2) {
			dev_err(&motor->spi->dev,
				"The driver support step-motor max num is 2\n");
			return -EINVAL;
		}
		step_motor_cnt++;
		motor->zoom = devm_kzalloc(&motor->spi->dev, sizeof(*motor->zoom), GFP_KERNEL);
		if (!motor->zoom) {
			dev_err(&motor->spi->dev,
				"__line__ %d, devm_kzalloc return fail!\n", __LINE__);
			return -ENOMEM;
		}
		ret = of_property_read_string(node, "zoom-used-pin",
					       &str);
		if (ret != 0) {
			dev_err(&motor->spi->dev,
				"get zoom-used-pin fail, please check it!\n");
			return -EINVAL;
		}
		if (strcmp(str, "ab") == 0) {
			motor->zoom->reg_op = &motor->motor_op[0];
			if (motor->zoom->reg_op->is_used) {
				dev_err(&motor->spi->dev,
					"__line__ %d, pin already been used\n", __LINE__);
				return -EINVAL;
			}
			motor->zoom->reg_op->is_used = true;
		} else if (strcmp(str, "cd") == 0) {
			motor->zoom->reg_op = &motor->motor_op[1];
			if (motor->zoom->reg_op->is_used) {
				dev_err(&motor->spi->dev,
					"__line__ %d, pin already been used\n", __LINE__);
				return -EINVAL;
			}
			motor->zoom->reg_op->is_used = true;
		} else {
			dev_err(&motor->spi->dev,
				"__line__ %d, pin require error\n", __LINE__);
			return -EINVAL;
		}
		ret = of_property_read_u32(node,
					   "zoom-backlash",
					   &motor->zoom->backlash);
		if (ret != 0) {
			motor->zoom->backlash = 0;
			dev_err(&motor->spi->dev,
				"failed get motor backlash,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "zoom-step-max",
					   &motor->zoom->step_max);
		if (ret != 0) {
			motor->zoom->step_max = ZOOM_MAX_STEP_DEF;
			dev_err(&motor->spi->dev,
				"failed get iris zoom_pos_max,use dafult value\n");
		}

		ret = of_property_read_u32(node,
					   "zoom-start-up-speed",
					   &motor->zoom->start_up_speed);
		if (ret != 0) {
			motor->zoom->start_up_speed = START_UP_HZ_DEF;
			dev_err(&motor->spi->dev,
				"failed get motor start up speed,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "zoom-ppw",
					   &motor->zoom->run_data.ppw);
		if (ret != 0 || (motor->zoom->run_data.ppw > 0xff)) {
			motor->zoom->run_data.ppw = PPW_DEF;
			dev_err(&motor->spi->dev,
				"failed get zoom ppw,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "zoom-ppw-stop",
					   &motor->zoom->run_data.ppw_stop);
		if (ret != 0 || (motor->zoom->run_data.ppw_stop > 0xff)) {
			motor->zoom->run_data.ppw_stop = PPW_STOP;
			dev_err(&motor->spi->dev,
				"failed get zoom ppw_stop,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "zoom-phmode",
					   &motor->zoom->run_data.phmode);
		if (ret != 0 || (motor->zoom->run_data.phmode > 0x3ff)) {
			motor->zoom->run_data.phmode = PHMODE_DEF;
			dev_err(&motor->spi->dev,
				"failed get zoom phmode,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "zoom-micro",
					   &motor->zoom->run_data.micro);
		if (ret != 0) {
			motor->zoom->run_data.micro = MICRO_DEF;
			dev_err(&motor->spi->dev,
				"failed get zoom micro,use dafult value\n");
		}
		motor->zoom->pic_gpio = devm_gpiod_get(&motor->spi->dev,
						       "zoom_pic", GPIOD_IN);
		if (IS_ERR(motor->zoom->pic_gpio))
			dev_err(&motor->spi->dev, "Failed to get zoom-pi-c-gpios\n");
		motor->zoom->is_half_step_mode =
			device_property_read_bool(&motor->spi->dev, "zoom-1-2phase-excitation");
		motor->zoom->is_dir_opp =
			device_property_read_bool(&motor->spi->dev, "zoom-dir-opposite");
		if (step_motor_cnt == 1)
			motor->dev0 = motor->zoom;
		else if (step_motor_cnt == 2)
			motor->dev1 = motor->zoom;
		ret = of_property_read_s32(node,
					   "zoom-min-pos",
					   &motor->zoom->min_pos);
		if (ret != 0) {
			motor->zoom->min_pos = 0;
			dev_err(&motor->spi->dev,
				"failed get zoom min pos,use dafult value\n");
		}
		ret = of_property_read_s32(node,
					   "zoom-max-pos",
					   &motor->zoom->max_pos);
		if (ret != 0) {
			motor->zoom->max_pos = motor->zoom->step_max;
			dev_err(&motor->spi->dev,
				"failed get zoom max_pos pos,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "zoom-reback-distance",
					   &motor->zoom->reback);
		if (ret != 0) {
			dev_err(&motor->spi->dev,
				"failed get zoom reback distance, return\n");
			return -EINVAL;
		}
	}

	if (motor->is_use_zoom1) {
		if (step_motor_cnt >= 2) {
			dev_err(&motor->spi->dev,
				"The driver support step-motor max num is 2\n");
			return -EINVAL;
		}
		step_motor_cnt++;
		motor->zoom1 = devm_kzalloc(&motor->spi->dev, sizeof(*motor->zoom1), GFP_KERNEL);
		if (!motor->zoom1) {
			dev_err(&motor->spi->dev,
				"__line__ %d, devm_kzalloc return fail!\n", __LINE__);
			return -ENOMEM;
		}
		ret = of_property_read_string(node, "zoom1-used-pin",
					       &str);
		if (ret != 0) {
			dev_err(&motor->spi->dev,
				"get zoom1-used-pin fail, please check it!\n");
			return -EINVAL;
		}
		if (strcmp(str, "ab") == 0) {
			motor->zoom1->reg_op = &motor->motor_op[0];
			if (motor->zoom1->reg_op->is_used) {
				dev_err(&motor->spi->dev,
					"__line__ %d, pin already been used\n", __LINE__);
				return -EINVAL;
			}
			motor->zoom1->reg_op->is_used = true;
		} else if (strcmp(str, "cd") == 0) {
			motor->zoom1->reg_op = &motor->motor_op[1];
			if (motor->zoom1->reg_op->is_used) {
				dev_err(&motor->spi->dev,
					"__line__ %d, pin already been used\n", __LINE__);
				return -EINVAL;
			}
			motor->zoom1->reg_op->is_used = true;
		} else {
			dev_err(&motor->spi->dev,
				"__line__ %d, pin require error\n", __LINE__);
			return -EINVAL;
		}
		ret = of_property_read_u32(node,
					   "zoom1-backlash",
					   &motor->zoom1->backlash);
		if (ret != 0) {
			motor->zoom1->backlash = 0;
			dev_err(&motor->spi->dev,
				"failed get motor backlash,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "zoom1-step-max",
					   &motor->zoom1->step_max);
		if (ret != 0) {
			motor->zoom1->step_max = ZOOM_MAX_STEP_DEF;
			dev_err(&motor->spi->dev,
				"failed get zoom_pos_max,use dafult value\n");
		}

		ret = of_property_read_u32(node,
					   "zoom1-start-up-speed",
					   &motor->zoom1->start_up_speed);
		if (ret != 0) {
			motor->zoom1->start_up_speed = START_UP_HZ_DEF;
			dev_err(&motor->spi->dev,
				"failed get motor start up speed,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "zoom1-ppw",
					   &motor->zoom1->run_data.ppw);
		if (ret != 0 || (motor->zoom1->run_data.ppw > 0xff)) {
			motor->zoom1->run_data.ppw = PPW_DEF;
			dev_err(&motor->spi->dev,
				"failed get zoom1 ppw,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "zoom1-ppw-stop",
					   &motor->zoom1->run_data.ppw_stop);
		if (ret != 0 || (motor->zoom1->run_data.ppw_stop > 0xff)) {
			motor->zoom1->run_data.ppw_stop = PPW_STOP;
			dev_err(&motor->spi->dev,
				"failed get zoom1 ppw_stop,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "zoom1-phmode",
					   &motor->zoom1->run_data.phmode);
		if (ret != 0 || (motor->zoom1->run_data.phmode > 0x3f)) {
			motor->zoom1->run_data.phmode = PHMODE_DEF;
			dev_err(&motor->spi->dev,
				"failed get zoom1 phmode,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "zoom1-micro",
					   &motor->zoom1->run_data.micro);
		if (ret != 0) {
			motor->zoom1->run_data.micro = MICRO_DEF;
			dev_err(&motor->spi->dev,
				"failed get zoom1 micro,use dafult value\n");
		}
		/* get zoom1 pi gpio */
		motor->zoom1->pic_gpio = devm_gpiod_get(&motor->spi->dev,
							"zoom1_pic", GPIOD_IN);
		if (IS_ERR(motor->zoom1->pic_gpio))
			dev_err(&motor->spi->dev, "Failed to get zoom1-pi-c-gpios\n");
		motor->zoom1->pia_gpio = devm_gpiod_get(&motor->spi->dev,
							"zoom1_pia", GPIOD_OUT_LOW);
		if (IS_ERR(motor->zoom1->pia_gpio))
			dev_err(&motor->spi->dev, "Failed to get zoom1-pi-a-gpios\n");
		motor->zoom1->pie_gpio = devm_gpiod_get(&motor->spi->dev,
							"zoom1_pie", GPIOD_OUT_LOW);
		if (IS_ERR(motor->zoom1->pie_gpio))
			dev_err(&motor->spi->dev, "Failed to get zoom1-pi-e-gpios\n");
		motor->zoom1->is_half_step_mode =
			device_property_read_bool(&motor->spi->dev, "zoom1-1-2phase-excitation");
		motor->zoom1->is_dir_opp =
			device_property_read_bool(&motor->spi->dev, "zoom1-dir-opposite");
		if (step_motor_cnt == 1)
			motor->dev0 = motor->zoom1;
		else if (step_motor_cnt == 2)
			motor->dev1 = motor->zoom1;
		ret = of_property_read_s32(node,
					   "zoom1-min-pos",
					   &motor->zoom1->min_pos);
		if (ret != 0) {
			motor->zoom1->min_pos = 0;
			dev_err(&motor->spi->dev,
				"failed get zoom1 min pos,use dafult value\n");
		}
		ret = of_property_read_s32(node,
					   "zoom1-max-pos",
					   &motor->zoom1->max_pos);
		if (ret != 0) {
			motor->zoom1->max_pos = motor->zoom1->step_max;
			dev_err(&motor->spi->dev,
				"failed get zoom1 max_pos pos,use dafult value\n");
		}
		ret = of_property_read_u32(node,
					   "zoom1-reback-distance",
					   &motor->zoom1->reback);
		if (ret != 0) {
			dev_err(&motor->spi->dev,
				"failed get zoom1 reback distance, return\n");
			return -EINVAL;
		}
	}

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &motor->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &motor->module_facing);
	if (ret) {
		dev_err(&motor->spi->dev,
			"could not get module information!\n");
		return -EINVAL;
	}
	return 0;
}

static void motor_config_dev_next_status(struct motor_dev *motor, struct ext_dev *dev)
{
	u16 ppw = 0;
	u16 psum = 0;
	u16 micro = 0;
	struct spi_device *spi = motor->spi;
#ifdef DEBUG
	u16 intct = 0;
	int i = 0;
	u16 val = 0;

	if (dev->move_status != MOTOR_STATUS_STOPPED) {
		dev_dbg(&spi->dev,
			"__line__ %d dev type %d, cur_count %d !\n", __LINE__,
			dev->type,
			dev->run_data.cur_count);
		dev_dbg(&spi->dev,
			"__line__ %d, motor reg table: 0x%02x 0x%02x 0x%02x!\n", __LINE__,
			dev->reg_op->reg.ppw,
			dev->reg_op->reg.psum,
			dev->reg_op->reg.intct);
		for (i = 0; i < 11; i++) {
			spi_read_reg(spi, 0x20 + i, &val);
			dev_dbg(&spi->dev,
				"========reg,val= 0x%02x, 0x%04x========\n",
				0x20 + i,
				val);
		}
	}
#endif

#ifdef DEBUG
	spi_read_reg(spi, dev->reg_op->reg.ppw, &ppw);
	spi_read_reg(spi, dev->reg_op->reg.psum, &psum);
	spi_read_reg(spi, dev->reg_op->reg.intct, &intct);
	dev_info(&spi->dev,
		 "__line__ %d dev type %d, cur_count %d , status %d! ppw 0x%x, psum 0x%x intct 0x%x\n", __LINE__,
		 dev->type,
		 dev->run_data.cur_count,
		 dev->move_status, ppw, psum, intct);
#endif
	if (dev->run_data.cur_count != 0) {
		if (dev->run_data.cur_count == dev->run_data.count &&
		    dev->is_need_update_tim) {
			dev->mv_tim.vcm_start_t = ns_to_timeval(ktime_get_ns());
			dev->is_need_update_tim = false;
		}
		dev->run_data.cur_count--;
		ppw = (dev->run_data.ppw << 8) | dev->run_data.ppw;
		switch (dev->run_data.micro) {
		case 64:
			micro = 0x03;
			break;
		case 128:
			micro = 0x02;
			break;
		case 256:
			micro = 0x00;
			break;
		default:
			micro = 0x00;
			break;
		};
		switch (dev->run_data.cur_count) {
		case 0:
			psum = ((dev->move_status - 1) << 8) |
			       (micro << 12) |
			       (1 << 10);
			ppw = (dev->run_data.ppw_stop << 8) | dev->run_data.ppw_stop;
			break;
		case 1:
			psum = ((dev->move_status - 1) << 8) |
			       (1 << 10) |
			       (micro << 12) |
			       (dev->run_data.psum_last);
			break;
		default:
			psum = ((dev->move_status - 1) << 8) |
			       (1 << 10) |
			       (micro << 12) |
			       (dev->run_data.psum);
			break;
		};
		spi_write_reg(motor->spi, 0x20, 0x1a01);
		spi_write_reg(spi, dev->reg_op->reg.ppw, ppw);
		spi_write_reg(spi, dev->reg_op->reg.psum, psum);
		spi_write_reg(spi, dev->reg_op->reg.intct,
			      dev->run_data.intct);
		dev->reg_op->tmp_psum = psum;
	} else if (dev->move_status != MOTOR_STATUS_STOPPED) {
		dev->mv_tim.vcm_end_t = ns_to_timeval(ktime_get_ns());
		dev->is_mv_tim_update = true;
		dev->move_status = MOTOR_STATUS_STOPPED;
		dev->reg_op->tmp_psum = 0;
		dev->is_running = false;
		complete(&dev->complete_out);
		complete(&dev->complete);
	}
}

static void motor_op_work(struct work_struct *work)
{
	struct motor_work_s *wk =
		container_of(work, struct motor_work_s, work);
	struct motor_dev *motor = wk->dev;
	static struct timeval tv_last = {0};
	struct timeval tv = {0};
	u64 time_dist = 0;

	do_gettimeofday(&tv);
	time_dist = tv.tv_sec * 1000000 + tv.tv_usec - (tv_last.tv_sec * 1000000 + tv_last.tv_usec);
	tv_last = tv;
	if (time_dist < motor->vd_fz_period_us && motor->is_timer_restart)
		dev_info(&motor->spi->dev,
			 "Timer error, Current interrupt interval %llu\n", time_dist);
	mutex_lock(&motor->mutex);
	gpiod_set_value(motor->vd_fz_gpio, 1);
	usleep_range(30, 60);
	gpiod_set_value(motor->vd_fz_gpio, 0);
	if (motor->dev0 && motor->dev0->run_data.cur_count == 0 &&
	   motor->dev0->is_need_reback) {
		if (motor->dev0->cur_back_delay < motor->dev0->max_back_delay) {
			motor->dev0->cur_back_delay++;
			motor->dev0->run_data.cur_count = 1;
		} else {
			motor->dev0->run_data = motor->dev0->reback_data;
			motor->dev0->is_need_reback = false;
			motor->dev0->move_status = motor->dev0->reback_status;
			motor->dev0->last_dir = motor->dev1->reback_status;
			motor->dev0->cur_back_delay = 0;
		}
	}
	if (motor->dev1 && motor->dev1->run_data.cur_count == 0 &&
	   motor->dev1->is_need_reback) {
		if (motor->dev1->cur_back_delay < motor->dev1->max_back_delay) {
			motor->dev1->cur_back_delay++;
			motor->dev1->run_data.cur_count = 1;
		} else {
			motor->dev1->run_data = motor->dev1->reback_data;
			motor->dev1->is_need_reback = false;
			motor->dev1->move_status = motor->dev1->reback_status;
			motor->dev1->last_dir = motor->dev1->reback_status;
			motor->dev1->cur_back_delay = 0;
		}
	}
	if ((motor->dev0 && motor->dev0->run_data.cur_count > 0) ||
	   (motor->dev1 && motor->dev1->run_data.cur_count > 0)) {
		motor->is_timer_restart = true;
		hrtimer_start(&motor->timer,
			      motor->vd_fz_period_us * 1000,
			      HRTIMER_MODE_REL);
	} else {
		motor->is_timer_restart = false;
	}
	usleep_range(660, 700);//delay more than DT1

	if (motor->dev0 && motor->dev0->move_status != MOTOR_STATUS_STOPPED)
		motor_config_dev_next_status(motor, motor->dev0);
	if (motor->dev1 && motor->dev1->move_status != MOTOR_STATUS_STOPPED)
		motor_config_dev_next_status(motor, motor->dev1);
	mutex_unlock(&motor->mutex);
	motor->is_should_wait = false;
}

static enum hrtimer_restart motor_timer_func(struct hrtimer *timer)
{
	struct motor_dev *motor = container_of(timer, struct motor_dev, timer);

	motor->is_should_wait = true;
	schedule_work_on(smp_processor_id(), &motor->wk->work);
	return HRTIMER_NORESTART;
}

static int motor_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct motor_dev *motor = container_of(ctrl->handler,
					     struct motor_dev, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_IRIS_ABSOLUTE:
		if (motor->is_use_dc_iris)
			ctrl->val = motor->dciris->last_log;
		else if (motor->is_use_p_iris)
			ctrl->val = motor->piris->last_pos;
		return 0;
	case V4L2_CID_FOCUS_ABSOLUTE:
		ctrl->val = motor->focus->last_pos;
		return 0;
	case V4L2_CID_ZOOM_ABSOLUTE:
		ctrl->val = motor->zoom->last_pos;
		return 0;
	case V4L2_CID_ZOOM_CONTINUOUS:
		ctrl->val = motor->zoom1->last_pos;
		return 0;
	}
	return 0;
}

static void wait_for_motor_stop(struct motor_dev *motor, struct ext_dev *dev)
{
	unsigned long ret = 0;

	if (dev->is_running) {
		reinit_completion(&dev->complete_out);
		ret = wait_for_completion_timeout(&dev->complete_out, 10 * HZ);
		if (ret == 0)
			dev_info(&motor->spi->dev,
				 "wait for complete timeout\n");
	}
}

static int motor_s_ctrl(struct v4l2_ctrl *ctrl)
{
#ifdef DEBUG
	int i = 0;
	u16 val = 0;
#endif
	int ret = 0;
	struct motor_dev *motor = container_of(ctrl->handler,
					     struct motor_dev, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_IRIS_ABSOLUTE:
		if (motor->is_use_dc_iris) {
			if (motor->dciris->is_reversed_polarity)
				spi_write_reg(motor->spi, 0x00,
					      motor->dciris->max_log - ctrl->val);
			else
				spi_write_reg(motor->spi, 0x00, ctrl->val);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 1);
			usleep_range(200, 400);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 0);
			motor->dciris->last_log = ctrl->val;
			dev_dbg(&motor->spi->dev, "set iris pos %d\n", ctrl->val);
#ifdef DEBUG
			for (i = 0; i < 16; i++) {
				spi_read_reg(motor->spi, i, &val);
				dev_dbg(&motor->spi->dev, "reg,val=0x%02x,0x%04x\n", i, val);
			}
#endif
		} else if (motor->is_use_p_iris) {
			ret = set_motor_running_status(motor,
						       motor->piris,
						       ctrl->val,
						       true,
						       false,
						       false);
			wait_for_motor_stop(motor, motor->piris);
			dev_dbg(&motor->spi->dev, "set piris pos %d\n", ctrl->val);
		}
		break;
	case V4L2_CID_FOCUS_ABSOLUTE:
		ret = set_motor_running_status(motor,
					       motor->focus,
					       ctrl->val,
					       true,
					       false,
					       false);
		wait_for_motor_stop(motor, motor->focus);
		dev_dbg(&motor->spi->dev, "set focus pos %d\n", ctrl->val);
		break;
	case V4L2_CID_ZOOM_ABSOLUTE:
		ret = set_motor_running_status(motor,
					       motor->zoom,
					       ctrl->val,
					       true,
					       false,
					       false);
		wait_for_motor_stop(motor, motor->zoom);
		dev_dbg(&motor->spi->dev, "set zoom pos %d\n", ctrl->val);
		break;
	case V4L2_CID_ZOOM_CONTINUOUS:
		ret = set_motor_running_status(motor,
					       motor->zoom1,
					       ctrl->val,
					       true,
					       false,
					       false);
		wait_for_motor_stop(motor, motor->zoom1);
		dev_dbg(&motor->spi->dev, "set zoom1 pos %d\n", ctrl->val);
		break;
	default:
		dev_err(&motor->spi->dev, "not support cmd %d\n", ctrl->id);
		break;
	}
	return ret;
}

static int motor_set_zoom_follow(struct motor_dev *motor, struct rk_cam_set_zoom *mv_param)
{
	int i = 0;
	int ret = 0;
	bool is_need_zoom_reback = mv_param->is_need_zoom_reback;
	bool is_need_focus_reback = mv_param->is_need_focus_reback;

	for (i = 0; i < mv_param->setzoom_cnt; i++) {
		if (i == (mv_param->setzoom_cnt - 1)) {
			ret = set_motor_running_status(motor,
						       motor->focus,
						       mv_param->zoom_pos[i].focus_pos,
						       true,
						       true,
						       is_need_zoom_reback);
			ret = set_motor_running_status(motor,
						       motor->zoom,
						       mv_param->zoom_pos[i].zoom_pos,
						       true,
						       false,
						       is_need_focus_reback);
		} else {
			set_motor_running_status(motor,
						 motor->focus,
						 mv_param->zoom_pos[i].focus_pos,
						 false,
						 true,
						 false);
			set_motor_running_status(motor,
						 motor->zoom,
						 mv_param->zoom_pos[i].zoom_pos,
						 false,
						 false,
						 false);
		}
		wait_for_motor_stop(motor, motor->focus);
		wait_for_motor_stop(motor, motor->zoom);
		dev_dbg(&motor->spi->dev,
			"%s zoom %d, focus %d, i %d\n",
			__func__,
			mv_param->zoom_pos[i].zoom_pos,
			mv_param->zoom_pos[i].focus_pos,
			i);
	}
	return ret;
}

static int motor_find_pi_binarysearch(struct motor_dev *motor,
			 struct ext_dev *ext_dev,
			 int min, int max)
{
	int gpio_val = 0;
	int tmp_val = 0;
	int mid = 0;
	int last_pos = 0;
	int new_min = 0;
	int new_max = 0;

	if (min > max)
		return -EINVAL;
	tmp_val = gpiod_get_value(ext_dev->pic_gpio);
	mid = (min + max) / 2;
	if (mid == min) {
		dev_dbg(&motor->spi->dev,
			"ext dev %d find pi %d\n", ext_dev->type, mid);
		if (ext_dev->last_pos < mid)
			set_motor_running_status(motor,
						 ext_dev,
						 mid,
						 false,
						 false,
						 false);
		else
			set_motor_running_status(motor,
						 ext_dev,
						 mid,
						 false,
						 false,
						 true);
		wait_for_motor_stop(motor, ext_dev);
		return mid;
	}
	last_pos = ext_dev->last_pos;
	if (last_pos < mid)
		set_motor_running_status(motor,
					 ext_dev,
					 mid,
					 false,
					 false,
					 false);
	else
		set_motor_running_status(motor,
				       ext_dev,
				       mid,
				       false,
				       false,
				       true);
	wait_for_motor_stop(motor, ext_dev);
	gpio_val = gpiod_get_value(ext_dev->pic_gpio);
	if (tmp_val != gpio_val) {
		usleep_range(10, 20);
		gpio_val = gpiod_get_value(ext_dev->pic_gpio);
	}

	dev_dbg(&motor->spi->dev,
		"__line__ %d ext_dev type %d, get pi value %d, tmp_val %d, min %d, max %d\n",
		__LINE__, ext_dev->type, gpio_val, tmp_val, min, max);
	if (tmp_val != gpio_val) {
		if (last_pos == min) {
			new_min = min;
			new_max = mid;
		} else {
			new_min = mid;
			new_max = max;
		}
	} else {
		if (last_pos == min) {
			new_min = mid;
			new_max = max;
		} else {
			new_min = min;
			new_max = mid;
		}
	}
	return motor_find_pi_binarysearch(motor, ext_dev, new_min, new_max);
}

static int motor_find_pi(struct motor_dev *motor,
		     struct ext_dev *ext_dev, int step)
{
	int i = 0;
	int idx_max = ext_dev->step_max + step - 1;
	int tmp_val = 0;
	int gpio_val = 0;
	int min = 0;
	int max = 0;
	bool is_find_pi = false;

	tmp_val = gpiod_get_value(ext_dev->pic_gpio);
	for (i = ext_dev->last_pos + step; i < idx_max; i += step) {
		set_motor_running_status(motor,
					 ext_dev,
					 i,
					 false,
					 false,
					 false);
		wait_for_motor_stop(motor, ext_dev);
		gpio_val = gpiod_get_value(ext_dev->pic_gpio);
		if (tmp_val != gpio_val) {
			usleep_range(10, 20);
			gpio_val = gpiod_get_value(ext_dev->pic_gpio);
		}
		dev_dbg(&motor->spi->dev,
			"__line__ %d ext_dev type %d, get pi value %d, i %d, tmp_val %d\n",
			__LINE__, ext_dev->type, gpio_val, i, tmp_val);
		if (tmp_val != gpio_val) {
			min = i - step;
			max = i;
			is_find_pi = true;
			break;
		}
	}
	if (i > idx_max) {
		for (i = ext_dev->last_pos - step; i > 0; i -= step) {
			set_motor_running_status(motor,
					       ext_dev,
					       i,
					       false,
					       false,
					       true);
			wait_for_motor_stop(motor, ext_dev);
			gpio_val = gpiod_get_value(ext_dev->pic_gpio);
			if (tmp_val != gpio_val) {
				usleep_range(10, 20);
				gpio_val = gpiod_get_value(ext_dev->pic_gpio);
			}
			dev_dbg(&motor->spi->dev,
				"__line__ %d ext_dev type %d, get pi value %d, i %d, tmp_val %d\n",
				__LINE__, ext_dev->type, gpio_val, i, tmp_val);
			if (tmp_val != gpio_val) {
				min = i;
				max = i + step;
				is_find_pi = true;
				break;
			}
		}
	}
	if (is_find_pi) {
		if (abs(step) == 1)
			return ext_dev->last_pos;
		else
			return motor_find_pi_binarysearch(motor, ext_dev,
							  min,
							  max);
	} else {
		return -EINVAL;
	}
}

static int motor_reinit_piris(struct motor_dev *motor)
{
	int ret = 0;

	if (!IS_ERR(motor->piris->pic_gpio)) {
		if (!IS_ERR(motor->piris->pia_gpio))
			gpiod_set_value(motor->piris->pia_gpio, 1);
		if (!IS_ERR(motor->piris->pie_gpio))
			gpiod_set_value(motor->piris->pie_gpio, 0);
		msleep(250);
		#ifdef PI_TEST
		motor->piris->last_pos = motor->piris->step_max;
		ret = set_motor_running_status(motor,
					       motor->piris,
					       0,
					       false,
					       false,
					       false);
		wait_for_motor_stop(motor, motor->piris);
		#else
		motor->piris->last_pos = 0;
		#endif
		ret = motor_find_pi(motor, motor->piris, 10);
		if (ret < 0) {
			dev_err(&motor->spi->dev,
				"get piris pi fail, pls check it\n");
			return -EINVAL;
		}
		#ifdef PI_TEST
		min = -ret;
		max = motor->piris->step_max + min;
		motor->piris->min_pos = min;
		motor->piris->max_pos = max;
		#endif
		if (!IS_ERR(motor->piris->pia_gpio))
			gpiod_set_value(motor->piris->pia_gpio, 0);
		if (!IS_ERR(motor->piris->pie_gpio))
			gpiod_set_value(motor->piris->pie_gpio, 0);
		motor->piris->last_pos = 0;
	} else {
		motor->piris->last_pos = motor->piris->step_max;
		ret = set_motor_running_status(motor,
					       motor->piris,
					       0,
					       false,
					       false,
					       false);
		wait_for_motor_stop(motor, motor->piris);
	}
	return 0;
}

static void motor_reinit_piris_pos(struct motor_dev *motor)
{
	if (!motor->piris) {
		dev_err(&motor->spi->dev,
			"not support piris\n");
		return;
	}
	motor_reinit_piris(motor);
	motor->piris->last_pos = 0;
	__v4l2_ctrl_modify_range(motor->iris_ctrl, motor->piris->min_pos,
				 motor->piris->max_pos - motor->piris->reback,
				 1, 0);
}

static int motor_reinit_focus(struct motor_dev *motor)
{
	int ret = 0;

	if (!IS_ERR(motor->focus->pic_gpio)) {
		if (!IS_ERR(motor->focus->pia_gpio))
			gpiod_set_value(motor->focus->pia_gpio, 1);
		if (!IS_ERR(motor->focus->pie_gpio))
			gpiod_set_value(motor->focus->pie_gpio, 0);
		msleep(250);
		#ifdef PI_TEST
		motor->focus->last_pos = motor->focus->step_max;
		ret = set_motor_running_status(motor,
					       motor->focus,
					       0,
					       false,
					       false,
					       false);
		wait_for_motor_stop(motor, motor->focus);
		#else
		motor->focus->last_pos = 0;
		#endif
		ret = motor_find_pi(motor, motor->focus, 200);
		if (ret < 0) {
			dev_info(&motor->spi->dev,
				 "get focus pi fail, pls check it\n");
			return -EINVAL;
		}
		#ifdef PI_TEST
		min = -ret;
		max = motor->focus->step_max + min;
		motor->focus->min_pos = min;
		motor->focus->max_pos = max;
		#endif
		if (!IS_ERR(motor->focus->pia_gpio))
			gpiod_set_value(motor->focus->pia_gpio, 0);
		if (!IS_ERR(motor->focus->pie_gpio))
			gpiod_set_value(motor->focus->pie_gpio, 0);
	} else {
		motor->focus->last_pos = motor->focus->step_max;
		ret = set_motor_running_status(motor,
					       motor->focus,
					       0,
					       false,
					       false,
					       true);
		wait_for_motor_stop(motor, motor->focus);
	}
	return 0;
}

static void motor_reinit_focus_pos(struct motor_dev *motor)
{
	if (!motor->focus) {
		dev_err(&motor->spi->dev,
			"not support focus\n");
		return;
	}
	motor_reinit_focus(motor);
	motor->focus->last_pos = 0;
	__v4l2_ctrl_modify_range(motor->focus_ctrl, motor->focus->min_pos,
				 motor->focus->max_pos - motor->focus->reback,
				 1, 0);
}

static int  motor_reinit_zoom(struct motor_dev *motor)
{
	int ret = 0;

	if (!IS_ERR(motor->zoom->pic_gpio)) {
		if (!IS_ERR(motor->focus->pia_gpio))
			gpiod_set_value(motor->focus->pia_gpio, 1);
		if (!IS_ERR(motor->focus->pie_gpio))
			gpiod_set_value(motor->focus->pie_gpio, 0);
		msleep(250);
		#ifdef PI_TEST
		motor->zoom->last_pos = motor->zoom->step_max;
		ret = set_motor_running_status(motor,
					       motor->zoom,
					       0,
					       false,
					       false,
					       false);
		wait_for_motor_stop(motor, motor->zoom);
		#else
		motor->zoom->last_pos = 0;
		#endif
		ret = motor_find_pi(motor, motor->zoom, 200);
		if (ret < 0) {
			dev_err(&motor->spi->dev,
				"get zoom pi fail, pls check it\n");
			return -EINVAL;
		}
		#ifdef PI_TEST
		min = -ret;
		max = motor->zoom->step_max + min;
		motor->zoom->min_pos = min;
		motor->zoom->max_pos = max;
		#endif
		if (!IS_ERR(motor->focus->pia_gpio))
			gpiod_set_value(motor->focus->pia_gpio, 0);
		if (!IS_ERR(motor->focus->pie_gpio))
			gpiod_set_value(motor->focus->pie_gpio, 0);
	} else {
		motor->zoom->last_pos = motor->zoom->step_max;
		ret = set_motor_running_status(motor,
					       motor->zoom,
					       0,
					       false,
					       false,
					       true);
		wait_for_motor_stop(motor, motor->zoom);
	}
	return 0;
}

static void motor_reinit_zoom_pos(struct motor_dev *motor)
{
	if (!motor->zoom) {
		dev_err(&motor->spi->dev,
			"not support zoom\n");
		return;
	}
	motor_reinit_zoom(motor);
	motor->zoom->last_pos = 0;
	__v4l2_ctrl_modify_range(motor->zoom_ctrl, motor->zoom->min_pos,
				 motor->zoom->max_pos - motor->zoom->reback,
				 1, 0);
}

static int motor_reinit_zoom1(struct motor_dev *motor)
{
	int ret = 0;

	if (!IS_ERR(motor->zoom1->pic_gpio)) {
		if (!IS_ERR(motor->zoom1->pia_gpio))
			gpiod_set_value(motor->zoom1->pia_gpio, 1);
		if (!IS_ERR(motor->zoom1->pie_gpio))
			gpiod_set_value(motor->zoom1->pie_gpio, 0);
		msleep(250);
		#ifdef PI_TEST
		motor->zoom1->last_pos = motor->zoom1->step_max;
		ret = set_motor_running_status(motor,
					       motor->zoom1,
					       0,
					       false,
					       false,
					       false);
		wait_for_motor_stop(motor, motor->zoom1);
		#else
		motor->zoom1->last_pos = 0;
		#endif
		ret = motor_find_pi(motor, motor->zoom1, 200);
		if (ret < 0) {
			dev_err(&motor->spi->dev,
				"get zoom1 pi fail, pls check it\n");
			return -EINVAL;
		}
		#ifdef PI_TEST
		min = -ret;
		max = motor->zoom1->step_max + min;
		motor->zoom1->min_pos = min;
		motor->zoom1->max_pos = max;
		#endif
		if (!IS_ERR(motor->zoom1->pia_gpio))
			gpiod_set_value(motor->zoom1->pia_gpio, 0);
		if (!IS_ERR(motor->zoom1->pie_gpio))
			gpiod_set_value(motor->zoom1->pie_gpio, 0);
	}  else {
		motor->zoom1->last_pos = motor->zoom1->step_max;
		ret = set_motor_running_status(motor,
					       motor->zoom1,
					       0,
					       false,
					       false,
					       true);
		wait_for_motor_stop(motor, motor->zoom1);
	}
	return 0;
}

static void motor_reinit_zoom1_pos(struct motor_dev *motor)
{
	if (!motor->zoom1) {
		dev_err(&motor->spi->dev,
			"not support zoom1\n");
		return;
	}
	motor_reinit_zoom1(motor);
	motor->zoom1->last_pos = 0;
	__v4l2_ctrl_modify_range(motor->zoom1_ctrl, motor->zoom1->min_pos,
				 motor->zoom1->max_pos - motor->zoom1->reback,
				 1, 0);
}

//#define REBACK_CTRL_BY_DRV
static int motor_set_focus(struct motor_dev *motor, struct rk_cam_set_focus *mv_param)
{
	int ret = 0;
	bool is_need_reback = mv_param->is_need_reback;

#ifdef REBACK_CTRL_BY_DRV
	if (mv_param->focus_pos > motor->focus->last_pos)
		is_need_reback = false;
	else
		is_need_reback = true;
#endif
	ret = set_motor_running_status(motor,
				       motor->focus,
				       mv_param->focus_pos,
				       true,
				       false,
				       is_need_reback);
	wait_for_motor_stop(motor, motor->focus);

	return ret;
}

static long motor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct rk_cam_vcm_tim *mv_tim;
	struct motor_dev *motor = to_motor_dev(sd);
	u32 *pbacklash = 0;
	struct rk_cam_set_zoom *mv_param;
	struct rk_cam_set_focus *focus_param;
	int ret = 0;
	struct rk_cam_modify_pos *pos;

	switch (cmd) {
	case RK_VIDIOC_IRIS_TIMEINFO:
		mv_tim = (struct rk_cam_vcm_tim *)arg;
		if (!motor->piris->is_mv_tim_update)
			usleep_range(motor->piris->move_time_us,
				     motor->piris->move_time_us + 1000);
		if (motor->piris->is_mv_tim_update) {
			memcpy(mv_tim, &motor->piris->mv_tim, sizeof(*mv_tim));

			dev_dbg(&motor->spi->dev,
				"get_piris_move_tim 0x%lx, 0x%lx, 0x%lx, 0x%lx\n",
				mv_tim->vcm_start_t.tv_sec,
				mv_tim->vcm_start_t.tv_usec,
				mv_tim->vcm_end_t.tv_sec,
				mv_tim->vcm_end_t.tv_usec);
		} else {
			dev_err(&motor->spi->dev, "get_piris_move_tim failed\n");
			return -EINVAL;
		}
		break;
	case RK_VIDIOC_VCM_TIMEINFO:
		mv_tim = (struct rk_cam_vcm_tim *)arg;
		if (!motor->focus->is_mv_tim_update)
			usleep_range(motor->focus->move_time_us,
				     motor->focus->move_time_us + 1000);
		if (motor->focus->is_mv_tim_update) {
			memcpy(mv_tim, &motor->focus->mv_tim, sizeof(*mv_tim));

			dev_dbg(&motor->spi->dev,
				"get_focus_move_tim 0x%lx, 0x%lx, 0x%lx, 0x%lx\n",
				mv_tim->vcm_start_t.tv_sec,
				mv_tim->vcm_start_t.tv_usec,
				mv_tim->vcm_end_t.tv_sec,
				mv_tim->vcm_end_t.tv_usec);
		} else {
			dev_err(&motor->spi->dev, "get_focus_move_tim failed\n");
			return -EINVAL;
		}
		break;
	case RK_VIDIOC_ZOOM_TIMEINFO:
		mv_tim = (struct rk_cam_vcm_tim *)arg;
		if (!motor->zoom->is_mv_tim_update)
			usleep_range(motor->zoom->move_time_us,
				     motor->zoom->move_time_us + 1000);
		if (motor->zoom->is_mv_tim_update) {
			memcpy(mv_tim, &motor->zoom->mv_tim, sizeof(*mv_tim));

			dev_dbg(&motor->spi->dev,
				"get_zoom_move_tim 0x%lx, 0x%lx, 0x%lx, 0x%lx\n",
				mv_tim->vcm_start_t.tv_sec,
				mv_tim->vcm_start_t.tv_usec,
				mv_tim->vcm_end_t.tv_sec,
				mv_tim->vcm_end_t.tv_usec);
		} else {
			dev_err(&motor->spi->dev, "get_zoom_move_tim failed\n");
			return -EINVAL;
		}
		break;
	case RK_VIDIOC_ZOOM1_TIMEINFO:
		mv_tim = (struct rk_cam_vcm_tim *)arg;
		if (!motor->zoom1->is_mv_tim_update)
			usleep_range(motor->zoom1->move_time_us,
				     motor->zoom1->move_time_us + 1000);
		if (motor->zoom1->is_mv_tim_update) {
			memcpy(mv_tim, &motor->zoom1->mv_tim, sizeof(*mv_tim));

			dev_dbg(&motor->spi->dev,
				"get_zoom_move_tim 0x%lx, 0x%lx, 0x%lx, 0x%lx\n",
				mv_tim->vcm_start_t.tv_sec,
				mv_tim->vcm_start_t.tv_usec,
				mv_tim->vcm_end_t.tv_sec,
				mv_tim->vcm_end_t.tv_usec);
		} else {
			dev_err(&motor->spi->dev, "get_zoom_move_tim failed\n");
			return -EINVAL;
		}
		break;
	case RK_VIDIOC_IRIS_SET_BACKLASH:
		pbacklash = (u32 *)arg;
		motor->piris->backlash = *pbacklash;
		break;
	case RK_VIDIOC_FOCUS_SET_BACKLASH:
		pbacklash = (u32 *)arg;
		motor->focus->backlash = *pbacklash;
		break;
	case RK_VIDIOC_ZOOM_SET_BACKLASH:
		pbacklash = (u32 *)arg;
		motor->zoom->backlash = *pbacklash;
		break;
	case RK_VIDIOC_ZOOM1_SET_BACKLASH:
		pbacklash = (u32 *)arg;
		motor->zoom1->backlash = *pbacklash;
		break;
	case RK_VIDIOC_IRIS_CORRECTION:
		motor_reinit_piris_pos(motor);
		break;
	case RK_VIDIOC_FOCUS_CORRECTION:
		motor_reinit_focus_pos(motor);
		break;
	case RK_VIDIOC_ZOOM_CORRECTION:
		motor_reinit_zoom_pos(motor);
		break;
	case RK_VIDIOC_ZOOM1_CORRECTION:
		motor_reinit_zoom1_pos(motor);
		break;
	case RK_VIDIOC_ZOOM_SET_POSITION:
		mv_param = (struct rk_cam_set_zoom *)arg;
		ret = motor_set_zoom_follow(motor, mv_param);
		break;
	case RK_VIDIOC_FOCUS_SET_POSITION:
		focus_param = (struct rk_cam_set_focus *)arg;
		ret = motor_set_focus(motor, focus_param);
		break;
	case RK_VIDIOC_MODIFY_POSITION:
		pos = (struct rk_cam_modify_pos *)arg;
		if (motor->focus)
			motor->focus->last_pos = pos->focus_pos;
		if (motor->zoom)
			motor->zoom->last_pos = pos->zoom_pos;
		if (motor->zoom1)
			motor->zoom1->last_pos = pos->zoom1_pos;
		break;
	default:
		break;
	}
	return ret;
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

	switch (cmd) {
	case RK_VIDIOC_IRIS_TIMEINFO:
	case RK_VIDIOC_VCM_TIMEINFO:
	case RK_VIDIOC_ZOOM_TIMEINFO:
	case RK_VIDIOC_ZOOM1_TIMEINFO:
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
	case RK_VIDIOC_IRIS_SET_BACKLASH:
	case RK_VIDIOC_FOCUS_SET_BACKLASH:
	case RK_VIDIOC_ZOOM_SET_BACKLASH:
	case RK_VIDIOC_ZOOM1_SET_BACKLASH:
		if (copy_from_user(&val, up, sizeof(val)))
			return -EFAULT;
		ret = motor_ioctl(sd, cmd, &val);
		break;
	case RK_VIDIOC_IRIS_CORRECTION:
	case RK_VIDIOC_FOCUS_CORRECTION:
	case RK_VIDIOC_ZOOM_CORRECTION:
	case RK_VIDIOC_ZOOM1_CORRECTION:
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
	default:
		break;
	}
	return ret;
}
#endif

#define USED_SYS_DEBUG
#ifdef USED_SYS_DEBUG
static ssize_t set_pid_dgain(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = to_motor_dev(sd);
	int val = 0;
	int ret = 0;
	u16 reg_val = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (motor->is_use_dc_iris) {
			spi_read_reg(motor->spi, 0x01, &reg_val);
			reg_val &= 0x01ff;
			reg_val |= (val & 0x7f) << 9;
			spi_write_reg(motor->spi, 0x01, reg_val);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 1);
			usleep_range(200, 400);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 0);
			dev_info(dev, "set pid dgain %d, reg val 0x%x\n", val, reg_val);
			spi_read_reg(motor->spi, 0x01, &reg_val);
			dev_info(dev, "pid dgain reg val 0x%x, read from register\n", reg_val);
		} else {
			dev_err(dev, "not support dc-iris, do nothing\n");
		}
	}
	return count;
}

static ssize_t set_pid_zero(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = to_motor_dev(sd);
	int val = 0;
	int ret = 0;
	u16 reg_val = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (motor->is_use_dc_iris) {
			spi_read_reg(motor->spi, 0x02, &reg_val);
			reg_val &= 0xf0ff;
			reg_val |= (val & 0xf) << 8;
			spi_write_reg(motor->spi, 0x02, reg_val);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 1);
			usleep_range(200, 400);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 0);
			dev_info(dev, "set pid zero %d, reg val 0x%x\n", val, reg_val);
			spi_read_reg(motor->spi, 0x02, &reg_val);
			dev_info(dev, "pid zero reg val 0x%x, read from register\n", reg_val);
		} else {
			dev_err(dev, "not support dc-iris, do nothing\n");
		}
	}
	return count;
}

static ssize_t set_pid_pole(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = to_motor_dev(sd);
	int val = 0;
	int ret = 0;
	u16 reg_val = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (motor->is_use_dc_iris) {
			spi_read_reg(motor->spi, 0x02, &reg_val);
			reg_val &= 0x0fff;
			reg_val |= (val & 0xf) << 12;
			spi_write_reg(motor->spi, 0x02, reg_val);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 1);
			usleep_range(200, 400);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 0);
			dev_info(dev, "set pid pole %d, reg val 0x%x\n", val, reg_val);
			spi_read_reg(motor->spi, 0x02, &reg_val);
			dev_info(dev, "pid pole reg val 0x%x, read from register\n", reg_val);
		} else {
			dev_err(dev, "not support dc-iris, do nothing\n");
		}
	}
	return count;
}

static ssize_t set_hall_bias(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = to_motor_dev(sd);
	int val = 0;
	int ret = 0;
	u16 reg_val = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (motor->is_use_dc_iris) {
			spi_read_reg(motor->spi, 0x04, &reg_val);
			reg_val &= 0xff00;
			reg_val |= val & 0xff;
			spi_write_reg(motor->spi, 0x04, reg_val);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 1);
			usleep_range(200, 400);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 0);
			dev_info(dev, "set hall_bias %d, reg val 0x%x\n", val, reg_val);
			spi_read_reg(motor->spi, 0x04, &reg_val);
			dev_info(dev, "hall bias reg val 0x%x, read from register\n", reg_val);
		} else {
			dev_err(dev, "not support dc-iris, do nothing\n");
		}
	}
	return count;
}

static ssize_t set_hall_offset(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = to_motor_dev(sd);
	int val = 0;
	int ret = 0;
	u16 reg_val = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (motor->is_use_dc_iris) {
			spi_read_reg(motor->spi, 0x04, &reg_val);
			reg_val &= 0x00ff;
			reg_val |= (val & 0xff) << 8;
			spi_write_reg(motor->spi, 0x04, reg_val);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 1);
			usleep_range(200, 400);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 0);
			dev_info(dev, "set hall_offset %d, reg val 0x%x\n", val, reg_val);
			spi_read_reg(motor->spi, 0x04, &reg_val);
			dev_info(dev, "hall offset reg val 0x%x, read from register\n", reg_val);
		} else {
			dev_err(dev, "not support dc-iris, do nothing\n");
		}
	}
	return count;
}

static ssize_t set_hall_gain(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = to_motor_dev(sd);
	int val = 0;
	int ret = 0;
	u16 reg_val = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (motor->is_use_dc_iris) {
			spi_read_reg(motor->spi, 0x05, &reg_val);
			reg_val &= 0xf0ff;
			reg_val |= (val & 0xf) << 8;
			spi_write_reg(motor->spi, 0x05, reg_val);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 1);
			usleep_range(200, 400);
			gpiod_set_value(motor->dciris->vd_iris_gpio, 0);
			dev_info(dev, "set hall_offset %d, reg val 0x%04x\n", val, reg_val);
			spi_read_reg(motor->spi, 0x05, &reg_val);
			dev_info(dev, "hall gain reg val 0x%04x, read from register\n", reg_val);
		} else {
			dev_err(dev, "not support dc-iris, do nothing\n");
		}
	}
	return count;
}

static ssize_t reinit_piris_pos(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = to_motor_dev(sd);
	int val = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (val == 1)
			motor_reinit_piris_pos(motor);
	}
	return count;
}

static ssize_t reinit_focus_pos(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = to_motor_dev(sd);
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
	struct motor_dev *motor = to_motor_dev(sd);
	int val = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (val == 1)
			motor_reinit_zoom_pos(motor);
	}
	return count;
}

static ssize_t reinit_zoom1_pos(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct motor_dev *motor = to_motor_dev(sd);
	int val = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		if (val == 1)
			motor_reinit_zoom1_pos(motor);
	}
	return count;
}

static struct device_attribute attributes[] = {
	__ATTR(pid_dgain, S_IWUSR, NULL, set_pid_dgain),
	__ATTR(pid_zero, S_IWUSR, NULL, set_pid_zero),
	__ATTR(pid_pole, S_IWUSR, NULL, set_pid_pole),
	__ATTR(hall_bias, S_IWUSR, NULL, set_hall_bias),
	__ATTR(hall_offset, S_IWUSR, NULL, set_hall_offset),
	__ATTR(hall_gain, S_IWUSR, NULL, set_hall_gain),
	__ATTR(reinit_piris, S_IWUSR, NULL, reinit_piris_pos),
	__ATTR(reinit_focus, S_IWUSR, NULL, reinit_focus_pos),
	__ATTR(reinit_zoom, S_IWUSR, NULL, reinit_zoom_pos),
	__ATTR(reinit_zoom1, S_IWUSR, NULL, reinit_zoom1_pos),
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
	#ifdef PI_TEST
	int min = 0;
	int max = 0;
	#endif
	unsigned long flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE | V4L2_CTRL_FLAG_VOLATILE;

	handler = &motor->ctrl_handler;
	ret = v4l2_ctrl_handler_init(handler, 3);
	if (ret)
		return ret;
	if (motor->is_use_dc_iris) {
		motor->iris_ctrl = v4l2_ctrl_new_std(handler, &motor_ctrl_ops,
			V4L2_CID_IRIS_ABSOLUTE, 0, motor->dciris->max_log, 1, 0);
		if (motor->iris_ctrl)
			motor->iris_ctrl->flags |= flags;

	} else if (motor->is_use_p_iris) {
		#ifdef REINIT_BOOT
		ret = motor_reinit_piris(motor);
		if (ret < 0)
			return -EINVAL;
		#endif
		motor->piris->last_pos = motor->piris->min_pos;
		motor->iris_ctrl = v4l2_ctrl_new_std(handler, &motor_ctrl_ops,
						     V4L2_CID_IRIS_ABSOLUTE,
						     motor->piris->min_pos,
						     motor->piris->max_pos,
						     1, motor->piris->min_pos);
		if (motor->iris_ctrl)
			motor->iris_ctrl->flags |= flags;
	}
	if (motor->is_use_focus) {
		#ifdef REINIT_BOOT
		ret = motor_reinit_focus(motor);
		if (ret < 0)
			return -EINVAL;
		#endif
		motor->focus->last_pos = motor->focus->min_pos;
		motor->focus_ctrl = v4l2_ctrl_new_std(handler, &motor_ctrl_ops,
				    V4L2_CID_FOCUS_ABSOLUTE, motor->focus->min_pos,
				    motor->focus->max_pos - motor->focus->reback,
				    1, motor->focus->min_pos);
		if (motor->focus_ctrl)
			motor->focus_ctrl->flags |= flags;
	}
	if (motor->is_use_zoom) {
		#ifdef REINIT_BOOT
		ret = motor_reinit_zoom(motor);
		if (ret < 0)
			return -EINVAL;
		#endif
		motor->zoom->last_pos = motor->zoom->min_pos;
		motor->zoom_ctrl = v4l2_ctrl_new_std(handler, &motor_ctrl_ops,
				    V4L2_CID_ZOOM_ABSOLUTE,
				    motor->zoom->min_pos,
				    motor->zoom->max_pos - motor->zoom->reback,
				    1, motor->zoom->min_pos);
		if (motor->zoom_ctrl)
			motor->zoom_ctrl->flags |= flags;
	}
	if (motor->is_use_zoom1) {
		#ifdef REINIT_BOOT
		ret = motor_reinit_zoom1(motor);
		if (ret < 0)
			return -EINVAL;
		#endif
		motor->zoom1->last_pos = motor->zoom1->min_pos;
		motor->zoom1_ctrl = v4l2_ctrl_new_std(handler, &motor_ctrl_ops,
				    V4L2_CID_ZOOM_CONTINUOUS,
				    motor->zoom1->min_pos,
				    motor->zoom1->max_pos,
				    1, motor->zoom1->min_pos);
		if (motor->zoom1_ctrl)
			motor->zoom1_ctrl->flags |= flags;
	}
	if (handler->error) {
		ret = handler->error;
		dev_err(&motor->spi->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	motor->subdev.ctrl_handler = handler;
	return ret;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static void dev_param_init(struct motor_dev *motor)
{
	int step = 0;
	u32 mv_cnt = 0;
	u32 status = 0;
	u32 reback_vd_cnt = 0;

	if (motor->is_use_dc_iris)
		motor->dciris->last_log = 0;
	if (motor->is_use_p_iris) {
		motor->piris->is_mv_tim_update = false;
		motor->piris->is_need_update_tim = false;
		motor->piris->move_status = MOTOR_STATUS_STOPPED;
		motor->piris->type = TYPE_IRIS;
		motor->piris->mv_tim.vcm_start_t = ns_to_timeval(ktime_get_ns());
		motor->piris->mv_tim.vcm_end_t = ns_to_timeval(ktime_get_ns());
		init_completion(&motor->piris->complete);
		init_completion(&motor->piris->complete_out);
		motor->piris->run_data.psum = motor->vd_fz_period_us *
					      motor->piris->start_up_speed * 8 / 1000000;
		motor->piris->run_data.intct = 27 * motor->vd_fz_period_us /
					       (motor->piris->run_data.psum * 24);
		motor->piris->is_running = false;
		dev_info(&motor->spi->dev,
			 "piris vd_fz_period_us %u, inict %d\n",
			 motor->vd_fz_period_us,
			 motor->piris->run_data.intct);
	}
	if (motor->is_use_focus) {
		motor->focus->is_mv_tim_update = false;
		motor->focus->is_need_update_tim = false;
		motor->focus->move_status = MOTOR_STATUS_STOPPED;
		motor->focus->type = TYPE_FOCUS;
		motor->focus->mv_tim.vcm_start_t = ns_to_timeval(ktime_get_ns());
		motor->focus->mv_tim.vcm_end_t = ns_to_timeval(ktime_get_ns());
		init_completion(&motor->focus->complete);
		init_completion(&motor->focus->complete_out);
		motor->focus->run_data.psum = motor->vd_fz_period_us *
					      motor->focus->start_up_speed * 8 / 1000000;
		motor->focus->run_data.intct = 27 * motor->vd_fz_period_us /
					       (motor->focus->run_data.psum * 24);
		motor->focus->is_running = false;
		dev_info(&motor->spi->dev,
			 "focus vd_fz_period_us %u, inict %d\n",
			 motor->vd_fz_period_us,
			 motor->focus->run_data.intct);
		if (motor->focus->reback != 0) {
			motor->focus->cur_back_delay = 0;
			motor->focus->max_back_delay = FOCUS_MAX_BACK_DELAY;
			motor->focus->reback_data = motor->focus->run_data;
			mv_cnt = motor->focus->reback;
			if (motor->focus->is_dir_opp) {
				mv_cnt += motor->focus->backlash;
				status = MOTOR_STATUS_CW;
			} else {
				mv_cnt += motor->focus->backlash;
				status = MOTOR_STATUS_CCW;
			}
			motor->focus->reback_status = status;
			if (motor->focus->is_half_step_mode)
				step = mv_cnt * 4;
			else
				step = mv_cnt * 8;
			motor->focus->reback_data.count = (step + motor->focus->reback_data.psum - 1) /
							   motor->focus->reback_data.psum + 1;
			motor->focus->reback_data.cur_count = motor->focus->reback_data.count;
			motor->focus->reback_data.psum_last = step % motor->focus->reback_data.psum;
			if (motor->focus->reback_data.psum_last == 0)
				motor->focus->reback_data.psum_last = motor->focus->reback_data.psum;
			reback_vd_cnt = motor->focus->reback_data.count + motor->focus->max_back_delay;
			motor->focus->reback_move_time_us = reback_vd_cnt * (motor->vd_fz_period_us + 500);
		}
	}
	if (motor->is_use_zoom) {
		motor->zoom->is_mv_tim_update = false;
		motor->zoom->is_need_update_tim = false;
		motor->zoom->move_status = MOTOR_STATUS_STOPPED;
		motor->zoom->type = TYPE_ZOOM;
		motor->zoom->mv_tim.vcm_start_t = ns_to_timeval(ktime_get_ns());
		motor->zoom->mv_tim.vcm_end_t = ns_to_timeval(ktime_get_ns());
		init_completion(&motor->zoom->complete);
		init_completion(&motor->zoom->complete_out);
		motor->vd_fz_period_us = VD_FZ_US;
		motor->zoom->run_data.psum = motor->vd_fz_period_us *
					     motor->zoom->start_up_speed * 8 / 1000000;
		motor->zoom->run_data.intct = 27 * motor->vd_fz_period_us /
					      (motor->zoom->run_data.psum * 24);
		motor->zoom->is_running = false;
		if (motor->zoom->reback != 0) {
			motor->zoom->cur_back_delay = 0;
			motor->zoom->max_back_delay = ZOOM_MAX_BACK_DELAY;
			motor->zoom->reback_data = motor->zoom->run_data;
			mv_cnt = motor->zoom->reback;
			if (motor->zoom->is_dir_opp) {
				mv_cnt += motor->zoom->backlash;
				status = MOTOR_STATUS_CW;
			} else {
				mv_cnt += motor->zoom->backlash;
				status = MOTOR_STATUS_CCW;
			}
			motor->zoom->reback_status = status;
			if (motor->zoom->is_half_step_mode)
				step = mv_cnt * 4;
			else
				step = mv_cnt * 8;
			motor->zoom->reback_data.count = (step + motor->zoom->reback_data.psum - 1) /
							   motor->zoom->reback_data.psum + 1;
			motor->zoom->reback_data.cur_count = motor->zoom->reback_data.count;
			motor->zoom->reback_data.psum_last = step % motor->zoom->reback_data.psum;
			if (motor->zoom->reback_data.psum_last == 0)
				motor->zoom->reback_data.psum_last = motor->zoom->reback_data.psum;
			reback_vd_cnt = motor->zoom->reback_data.count + motor->zoom->max_back_delay;
			motor->zoom->reback_move_time_us = reback_vd_cnt * (motor->vd_fz_period_us + 500);
		}
		dev_info(&motor->spi->dev,
			 "zoom vd_fz_period_us %u, inict %d\n",
			 motor->vd_fz_period_us,
			 motor->zoom->run_data.intct);
	}
	if (motor->is_use_zoom1) {
		motor->zoom1->is_mv_tim_update = false;
		motor->zoom1->is_need_update_tim = false;
		motor->zoom1->move_status = MOTOR_STATUS_STOPPED;
		motor->zoom1->type = TYPE_ZOOM1;
		motor->zoom1->mv_tim.vcm_start_t = ns_to_timeval(ktime_get_ns());
		motor->zoom1->mv_tim.vcm_end_t = ns_to_timeval(ktime_get_ns());
		init_completion(&motor->zoom1->complete);
		init_completion(&motor->zoom1->complete_out);
		motor->vd_fz_period_us = VD_FZ_US;
		motor->zoom1->run_data.psum = motor->vd_fz_period_us *
					      motor->zoom1->start_up_speed * 8 / 1000000;
		motor->zoom1->run_data.intct = 27 * motor->vd_fz_period_us /
					       (motor->zoom1->run_data.psum * 24);
		motor->zoom1->is_running = false;
		dev_info(&motor->spi->dev,
			 "zoom1 vd_fz_period_us %u, inict %d\n",
			 motor->vd_fz_period_us,
			 motor->zoom1->run_data.intct);
	}

	motor->is_should_wait = false;
	motor->is_timer_restart = false;
	motor->wait_cnt = 0;
}

static void dev_reg_init(struct motor_dev *motor)
{
	spi_write_reg(motor->spi, 0x20, 0x1a01);//27M/(30*2^3*2^0)
	spi_write_reg(motor->spi, 0x21, 0x0085);
	spi_write_reg(motor->spi, 0x23, PPW_STOP);
	spi_write_reg(motor->spi, 0x28, PPW_STOP);
	if (motor->dev0)
		spi_write_reg(motor->spi,
			      motor->dev0->reg_op->reg.dt2_phmod,
			     (motor->dev0->run_data.phmode << 8) | 0x0001);
	if (motor->dev1)
		spi_write_reg(motor->spi,
			      motor->dev1->reg_op->reg.dt2_phmod,
			     (motor->dev1->run_data.phmode << 8) | 0x0001);

	spi_write_reg(motor->spi, 0x0b, 0x0480);
	if (motor->is_use_dc_iris) {
		//DC-IRIS reg init
		if (motor->dciris->is_reversed_polarity)
			spi_write_reg(motor->spi, 0x00,
				      motor->dciris->max_log - motor->dciris->last_log);
		else
			spi_write_reg(motor->spi, 0x00, motor->dciris->last_log);
		spi_write_reg(motor->spi, 0x01, 0x6000);
		spi_write_reg(motor->spi, 0x02, 0x66f0);
		spi_write_reg(motor->spi, 0x03, 0x0e10);
		spi_write_reg(motor->spi, 0x04, 0xd640);
		spi_write_reg(motor->spi, 0x05, 0x0004);
		spi_write_reg(motor->spi, 0x0b, 0x0480);
		spi_write_reg(motor->spi, 0x0a, 0x0000);
		spi_write_reg(motor->spi, 0x0e, 0x0300);

	}
	if (!IS_ERR(motor->vd_fz_gpio))
		gpiod_set_value(motor->vd_fz_gpio, 1);
	if (motor->is_use_dc_iris && (!IS_ERR(motor->dciris->vd_iris_gpio)))
		gpiod_set_value(motor->dciris->vd_iris_gpio, 1);
	usleep_range(100, 200);
	if (!IS_ERR(motor->vd_fz_gpio))
		gpiod_set_value(motor->vd_fz_gpio, 0);
	if (motor->is_use_dc_iris && (!IS_ERR(motor->dciris->vd_iris_gpio)))
		gpiod_set_value(motor->dciris->vd_iris_gpio, 0);
}


static int motor_check_id(struct motor_dev *motor)
{
	u16 val = 0xffff;
	int i = 0;

	for (i = 0; i < 0x20; i++)
		spi_read_reg(motor->spi, i, &val);
	spi_read_reg(motor->spi, 0x20, &val);
	if (val == 0xffff) {
		dev_err(&motor->spi->dev,
			"check id fail, spi transfer err or driver not connect, val 0x%x\n",
			val);
		return -EINVAL;
	}
	return 0;
}

static int dev_init(struct motor_dev *motor)
{
	int ret = 0;

	if (!IS_ERR(motor->reset_gpio)) {
		gpiod_set_value_cansleep(motor->reset_gpio, 0);
		usleep_range(100, 200);
		gpiod_set_value_cansleep(motor->reset_gpio, 1);
	}
	ret = motor_check_id(motor);
	if (ret < 0)
		return -EINVAL;
	dev_param_init(motor);
	dev_reg_init(motor);

	motor->wk = devm_kzalloc(&motor->spi->dev, sizeof(*motor->wk), GFP_KERNEL);
	if (!motor->wk) {
		dev_err(&motor->spi->dev, "failed to alloc work struct\n");
		return -ENOMEM;
	}
	motor->wk->dev = motor;
	INIT_WORK(&motor->wk->work, motor_op_work);

	return 0;
}

static int motor_dev_probe(struct spi_device *spi)
{
	int ret = 0;
	struct device *dev = &spi->dev;
	struct motor_dev *motor;
	struct v4l2_subdev *sd;
	char facing[2];

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);
	motor = devm_kzalloc(dev, sizeof(*motor), GFP_KERNEL);
	if (!motor)
		return -ENOMEM;
	spi->mode = SPI_MODE_3 | SPI_LSB_FIRST | SPI_CS_HIGH;
	spi->irq = -1;
	spi->max_speed_hz = 5000000;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(dev, "could not setup spi!\n");
		return -EINVAL;
	}
	motor->spi = spi;
	motor->motor_op[0] = g_motor_op[0];
	motor->motor_op[1] = g_motor_op[1];

	if (motor_dev_parse_dt(motor)) {
		dev_err(&motor->spi->dev, "parse dt error\n");
		return -EINVAL;
	}
	ret = dev_init(motor);
	if (ret)
		goto err_free;

	mutex_init(&motor->mutex);
	hrtimer_init(&motor->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	motor->timer.function = motor_timer_func;

	sd = &motor->subdev;
	v4l2_spi_subdev_init(sd, spi, &motor_subdev_ops);
	sd->entity.function = MEDIA_ENT_F_LENS;
	sd->entity.flags = 0;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	motor_initialize_controls(motor);
	ret = media_entity_pads_init(&motor->subdev.entity, 0, NULL);
	if (ret < 0)
		goto err_free;
	memset(facing, 0, sizeof(facing));
	if (strcmp(motor->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';
	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s_%d",
		 motor->module_index, facing,
		 DRIVER_NAME,
		 motor->id);
	ret = v4l2_async_register_subdev(sd);
	if (ret)
		dev_err(&spi->dev, "v4l2 async register subdev failed\n");
#ifdef USED_SYS_DEBUG
	add_sysfs_interfaces(dev);
#endif
	dev_info(&motor->spi->dev, "gpio motor driver probe success\n");
	return 0;
err_free:
	v4l2_ctrl_handler_free(&motor->ctrl_handler);
	v4l2_device_unregister_subdev(&motor->subdev);
	media_entity_cleanup(&motor->subdev.entity);
	return ret;
}

static int motor_dev_remove(struct spi_device *spi)
{
	struct v4l2_subdev *sd = spi_get_drvdata(spi);
	struct motor_dev *motor = to_motor_dev(sd);

	hrtimer_cancel(&motor->timer);
	if (sd)
		v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&motor->ctrl_handler);
	media_entity_cleanup(&motor->subdev.entity);
#ifdef USED_SYS_DEBUG
	remove_sysfs_interfaces(&spi->dev);
#endif
	return 0;
}

static const struct spi_device_id motor_match_id[] = {
	{"relmon,ms41908", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, motor_match_id);

#if defined(CONFIG_OF)
static const struct of_device_id motor_dev_of_match[] = {
	{.compatible = "relmon,ms41908", },
	{},
};
#endif

static struct spi_driver motor_dev_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(motor_dev_of_match),
	},
	.probe		= &motor_dev_probe,
	.remove		= &motor_dev_remove,
	.id_table	= motor_match_id,
};

static int __init motor_mod_init(void)
{
	return spi_register_driver(&motor_dev_driver);
}

static void __exit motor_mod_exit(void)
{
	spi_unregister_driver(&motor_dev_driver);
}

device_initcall_sync(motor_mod_init);
module_exit(motor_mod_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:motor");
MODULE_AUTHOR("zefa.chen@rock-chips.com");
