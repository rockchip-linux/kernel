/*
 * gpio motor  driver
 *
 * Copyright (C) 2016 Rockchip Electronics Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more motorails.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/hrtimer.h>

static struct class *motor_class;
static char *motor_id_label = "rockchip,motor-id";
static char *motor_status_desc[] = {
	"stopped",
	"cw",
	"ccw",
};

#define MAX_START_UP_HZ (1150)
#define MAX_RUNNING_HZ (1200)
#define MOTOR_MAX_HZ MAX_START_UP_HZ
#define MOTOR_MIN_HZ (550)

#define HZ_TO_SPEED(hz) (5625 * (hz) / 64000)
#define SPEED_TO_HZ(sp) ((sp) * 64000 / 5625)
#define ANGLE_PER_PHASE (5625 / 64000)
#define ANGLE_TO_PHASE_COUNT(ag) ((ag) * 64000 / 5625)

#define MOTOR_PHASE_PINS (4)
#define PHASE_TYPES (8)
/* 1-2phases mode */
static u8 phases_cw[PHASE_TYPES] = {0x1, 0x3, 0x2, 0x6, 0x4, 0xc, 0x8, 0x9};

enum {
	MOTOR_STATUS_STOPPED = 0,
	MOTOR_STATUS_CW = 1,
	MOTOR_STATUS_CCW = 2,
};

struct motor_gpio {
	struct device *dev;
	struct device *child_dev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct wake_lock wake_lock;
	struct hrtimer timer;
	int phase_gpios[MOTOR_PHASE_PINS];
	int type;
	unsigned int status;
	int speed;
	int angle;
	int id;
	u64 phase_interval_ns;
	u32 step;
	u32 remain_step;
	int phase_index;
};

static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct motor_gpio *motor;

	motor = dev_get_drvdata(dev);
	if (motor->status > MOTOR_STATUS_CCW)
		return -1;
	return sprintf(buf, "%s\n", motor_status_desc[motor->status]);
}

/* example: echo "cw 70" > status */
static ssize_t status_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct motor_gpio *motor = dev_get_drvdata(dev);
	int status = MOTOR_STATUS_STOPPED;
	int angle;
	int ret;
	char *cmd = kzalloc(count, GFP_KERNEL);

	if (!cmd)
		return -ENOMEM;
	ret = sscanf(buf, "%s %d", cmd, &angle);
	if (ret != 2)
		goto ERR_EXIT;
	if (!strncmp(cmd, "cw", 2))
		status = MOTOR_STATUS_CW;
	else if (!strncmp(cmd, "ccw", 3))
		status = MOTOR_STATUS_CCW;
	else if (!strncmp(cmd, "stop", 4))
		status = MOTOR_STATUS_STOPPED;
	else
		goto ERR_EXIT;
	kfree(cmd);
	if (status != MOTOR_STATUS_STOPPED &&
	    motor->status != MOTOR_STATUS_STOPPED) {
		dev_err(dev, "motor already working\n");
		return -EBUSY;
	}

	if (status == MOTOR_STATUS_STOPPED &&
	    motor->status != MOTOR_STATUS_STOPPED) {
		dev_info(dev, "motor try to stop working\n");
		motor->status = MOTOR_STATUS_STOPPED;
		/* motor stop work */
	}

	if (status != MOTOR_STATUS_STOPPED) {
		motor->status = status;
		motor->angle = angle;
		/* motor start work */
		motor->phase_index = 0;
		motor->step = ANGLE_TO_PHASE_COUNT(motor->angle);
		motor->phase_index = 0;
		hrtimer_start(&motor->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
	}
	dev_info(dev, "status %d, step %d, intval %lld ns\n",
		 motor->status, motor->step, motor->phase_interval_ns);
	return count;
ERR_EXIT:
	kfree(cmd);
	dev_info(dev, "input cmd error\n");
	dev_info(dev, "echo [cmd] [angle] > status\n");
	dev_info(dev, "cmd: cw,ccw,stop\n");
	dev_info(dev, "angle: 0 - 180\n");
	dev_info(dev, "example:echo \"cw 70\" > status\n");
	return -EIO;
}

static ssize_t speed_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct motor_gpio *motor;

	motor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", motor->speed);
}

static ssize_t speed_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct motor_gpio *motor;
	int hz;
	int val;
	int ret;

	motor = dev_get_drvdata(dev);
	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val <= 0)
		return val;

	hz = SPEED_TO_HZ(val);
	motor->speed = val;

	if (hz < MOTOR_MIN_HZ) {
		hz = MOTOR_MIN_HZ;
		motor->speed = HZ_TO_SPEED(MOTOR_MIN_HZ);
		dev_info(dev, "speed too large, use %d\n",
			 motor->speed);
	}
	if (hz > MOTOR_MAX_HZ) {
		hz = MOTOR_MAX_HZ;
		motor->speed = HZ_TO_SPEED(MOTOR_MAX_HZ);
		dev_info(dev, "speed too large, use %d\n",
			 motor->speed);
	}

	motor->phase_interval_ns = NSEC_PER_SEC / hz;
	dev_info(dev, "speed %d\n", motor->speed);
	return count;
}

static ssize_t angle_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct motor_gpio *motor;

	motor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", motor->angle);
}
static DEVICE_ATTR_RW(status);
static DEVICE_ATTR_RW(speed);
static DEVICE_ATTR_RO(angle);

static struct attribute *motor_gpio_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_speed.attr,
	&dev_attr_angle.attr,
	NULL,
};
ATTRIBUTE_GROUPS(motor_gpio);

static int motor_gpio_parse_dt(struct motor_gpio *motor)
{
	int i;
	int ret;
	int gpio;
	struct device_node *node = motor->dev->of_node;

	for (i = 0; i < MOTOR_PHASE_PINS; i++) {
		gpio = of_get_gpio(node, i);
		if (!gpio_is_valid(gpio)) {
			dev_err(motor->dev, "failed to parse gpio\n");
			return -1;
		}
		ret = devm_gpio_request(motor->dev, gpio, dev_name(motor->dev));
		if (ret < 0) {
			dev_err(motor->dev, "failed to request %d gpio %d\n",
				gpio, ret);
			return -1;
		}
		gpio_direction_output(gpio, 0);
		motor->phase_gpios[i] = gpio;
	}
	ret = of_property_read_u32(node, motor_id_label, &motor->id);
	if (ret != 0) {
		dev_err(motor->dev, "failed getting id from dts\n");
		return -EIO;
	}
	return 0;
}

static void motor_gpio_set_phase(struct motor_gpio *motor, int phase_index)
{
	int i;
	int ctrl = phases_cw[phase_index];

	for (i = 0; i < MOTOR_PHASE_PINS; i++)
		gpio_direction_output(motor->phase_gpios[i], (ctrl >> i) & 0x1);
}

static enum hrtimer_restart motor_timer_func(struct hrtimer *timer)
{
	int i;
	struct motor_gpio *motor;
	bool resched = false;
	int phase_types;

	motor = container_of(timer, struct motor_gpio, timer);
	/* dev_info(motor->dev, "%s:step %d\n", __func__, motor->step); */

	if (motor->step < 1 || motor->status == MOTOR_STATUS_STOPPED) {
		motor->status = MOTOR_STATUS_STOPPED;
		resched = false;
		for (i = 0; i < MOTOR_PHASE_PINS; i++)
			gpio_direction_output(motor->phase_gpios[i], 0);

	} else {
		/* do phase change */
		phase_types = PHASE_TYPES;
		motor_gpio_set_phase(motor, motor->phase_index);
		if (motor->status == MOTOR_STATUS_CW)
			motor->phase_index =
				(motor->phase_index + 1) % phase_types;
		else
			motor->phase_index =
				(motor->phase_index + phase_types - 1) %
				phase_types;
		resched = true;
		motor->step--;
	}

	if (resched) {
		hrtimer_forward_now(timer,
				    ns_to_ktime(motor->phase_interval_ns));
		return HRTIMER_RESTART;
	}
	return HRTIMER_NORESTART;
}

static int motor_gpio_probe(struct platform_device *pdev)
{
	int ret;
	struct motor_gpio *motor;

	motor = devm_kzalloc(&pdev->dev, sizeof(*motor), GFP_KERNEL);
	if (!motor)
		return -ENOMEM;
	motor->dev = &pdev->dev;
	dev_set_name(motor->dev, "motor");
	dev_set_drvdata(motor->dev, motor);
	if (!pdev->dev.of_node)
		return -EINVAL;
	wake_lock_init(&motor->wake_lock, WAKE_LOCK_SUSPEND, "motor_gpio");
	if (motor_gpio_parse_dt(motor)) {
		dev_err(motor->dev, "parse dt error\n");
		return -EINVAL;
	}
	motor->phase_interval_ns = NSEC_PER_SEC / MOTOR_MAX_HZ;
	motor->speed = HZ_TO_SPEED(MOTOR_MAX_HZ);
	hrtimer_init(&motor->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	motor->timer.function = motor_timer_func;

	motor->child_dev = device_create(motor_class, &pdev->dev, MKDEV(0, 0),
				motor, "%s%d", "motor_", motor->id);
	ret = sysfs_create_groups(&motor->child_dev->kobj, motor_gpio_groups);
	if (ret) {
		dev_err(motor->dev, "sysfs failed, ret=%d\n", ret);
		return -1;
	}
	dev_info(motor->dev, "gpio motor driver probe success\n");
	return 0;
}

static int motor_gpio_remove(struct platform_device *pdev)
{
	struct motor_gpio *motor;

	motor = platform_get_drvdata(pdev);
	hrtimer_cancel(&motor->timer);
	sysfs_remove_groups(&motor->child_dev->kobj, motor_gpio_groups);
	device_unregister(motor->child_dev);
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id motor_gpio_of_match[] = {
	{ .compatible = "motor-gpio", },
	{},
};
#endif

static struct platform_driver motor_gpio_driver = {
	.driver = {
		.name = "motor-gpio",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(motor_gpio_of_match),
	},
	.probe = motor_gpio_probe,
	.remove = motor_gpio_remove,
};

static int __init motor_gpio_init(void)
{
	motor_class = class_create(THIS_MODULE, "motor");
	if (IS_ERR(motor_class))
		return PTR_ERR(motor_class);
	return platform_driver_register(&motor_gpio_driver);
}

fs_initcall_sync(motor_gpio_init);

static void __exit motor_gpio_exit(void)
{
	platform_driver_unregister(&motor_gpio_driver);
	class_destroy(motor_class);
}

module_exit(motor_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:motor-gpio");
MODULE_AUTHOR("ROCKCHIP");
