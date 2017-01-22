/*
 **************************************************************************
 * Rockchip driver for IR-Cuter
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
/*
 *v0.1.0:
 *1. Initialize version;
 */
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>

#define IRCUT_GPIO_COUNT 2
#define IRCUT_GPIO_OPEN  0
#define IRCUT_GPIO_CLOSE 1

enum IRCUT_STATE_e {
	IRCUT_STATE_CLOSED,
	IRCUT_STATE_CLOSING,
	IRCUT_STATE_OPENING,
	IRCUT_STATE_OPENED,
};

struct ircut_op_work {
	struct work_struct work;
	int op_cmd;
	struct ircut_dev *dev;
};

struct ircut_dev {
	struct v4l2_subdev sd;
	struct device *dev;
	struct completion complete;
	/* state mutex */
	struct mutex mut_state;
	enum IRCUT_STATE_e state;
	struct workqueue_struct *wq;
	int pulse_width;
	int val;
	int gpio[IRCUT_GPIO_COUNT];
	int gpio_led;
};

static char *ircut_gpio_open_label = "rockchip,ircut-gpio-open";
static char *ircut_gpio_close_label = "rockchip,ircut-gpio-close";
static char *gpio_led_lable = "rockchip,led";

#define IRCUT_STATE_EQ(expected) \
	((ircut->state == (expected)) ? true : false)

static int ircut_gpio_parse_dt(
		struct ircut_dev *ircut,
		struct device_node *node)
{
	int i;
	int ret;
	int gpio;

	dev_dbg(ircut->dev, "ircut_gpio_parse_dt...");
	ret = of_property_read_u32(
		node,
		"rockchip,pulse-width",
		&ircut->pulse_width);
	if (ret != 0) {
		ircut->pulse_width = 100;
		dev_err(ircut->dev, "failed get  pulse-width from dts,use dafult value 100\n");
	}
	if (ircut->pulse_width > 2000) {
		ircut->pulse_width = 300;
		dev_info(ircut->dev, "pulse width to long,use default dafult 300");
	}
	dev_dbg(
		ircut->dev, "pulse-width value from dts %d\n",
		ircut->pulse_width);
	/* get ircut open gpio */
	gpio = of_get_named_gpio_flags(node, ircut_gpio_open_label, 0, NULL);
	dev_dbg(ircut->dev, "of_get_named_gpio_flags %d", gpio);
	if (!gpio_is_valid(gpio)) {
		dev_err(ircut->dev, "failed to parse open gpio\n");
		goto err_request_gpio;
	}
	ret = devm_gpio_request(ircut->dev, gpio, dev_name(ircut->dev));
	if (ret < 0) {
		dev_err(ircut->dev, "failed to request open gpio %d\n", ret);
		goto err_request_gpio;
	}
	gpio_direction_output(gpio, 0);
	ircut->gpio[IRCUT_GPIO_OPEN] = gpio;

	/* get ircut close gpio */
	gpio = of_get_named_gpio_flags(node, ircut_gpio_close_label, 0, NULL);
	dev_dbg(ircut->dev, "of_get_named_gpio_flags %d", gpio);
	if (!gpio_is_valid(gpio)) {
		dev_err(ircut->dev, "failed to parse close gpio\n");
		goto err_request_gpio;
	}
	ret = devm_gpio_request(ircut->dev, gpio, dev_name(ircut->dev));
	if (ret < 0) {
		dev_err(ircut->dev, "failed to request close gpio %d\n", ret);
		goto err_request_gpio;
	}
	gpio_direction_output(gpio, 0);
	ircut->gpio[IRCUT_GPIO_CLOSE] = gpio;

	/* get led gpio */
	gpio = of_get_named_gpio_flags(node, gpio_led_lable, 0, NULL);
	dev_dbg(ircut->dev, "of_get_named_gpio_flags %d", gpio);
	if (!gpio_is_valid(gpio)) {
		dev_err(ircut->dev, "failed to parse led gpio\n");
		goto err_request_gpio;
	}
	ret = devm_gpio_request(ircut->dev, gpio, dev_name(ircut->dev));
	if (ret < 0) {
		dev_err(ircut->dev, "failed to request led gpio %d\n", ret);
		goto err_request_gpio;
	}
	gpio_direction_output(gpio, 0);
	ircut->gpio_led = gpio;
	return 0;
err_request_gpio:
	for (i = 0; i < IRCUT_GPIO_COUNT; i++) {
		if (gpio_is_valid(ircut->gpio[i]))
			devm_gpio_free(ircut->dev, ircut->gpio[i]);
	}
	return -1;
}

static void ircut_op_work(struct work_struct *work)
{
	struct ircut_op_work *wk =
		container_of(work, struct ircut_op_work, work);
	struct ircut_dev *ircut = wk->dev;

	if (wk->op_cmd > 0) {
		gpio_set_value(ircut->gpio[IRCUT_GPIO_OPEN], 1);
		msleep(ircut->pulse_width);
		gpio_set_value(ircut->gpio[IRCUT_GPIO_OPEN], 0);
		gpio_direction_output(ircut->gpio_led, 0);
		mutex_lock(&ircut->mut_state);
		ircut->state = IRCUT_STATE_OPENED;
		mutex_unlock(&ircut->mut_state);
	} else {
		gpio_set_value(ircut->gpio[IRCUT_GPIO_CLOSE], 1);
		msleep(ircut->pulse_width);
		gpio_set_value(ircut->gpio[IRCUT_GPIO_CLOSE], 0);
		gpio_direction_output(ircut->gpio_led, 1);
		mutex_lock(&ircut->mut_state);
		ircut->state = IRCUT_STATE_CLOSED;
		mutex_unlock(&ircut->mut_state);
	}
	complete(&ircut->complete);
	kfree(wk);
	wk = NULL;
}

static int ircut_operation(struct ircut_dev *ircut, int op)
{
	struct ircut_op_work *wk = NULL;
	bool should_wait = false;
	bool do_nothing = false;
	enum IRCUT_STATE_e old_state;

	mutex_lock(&ircut->mut_state);
	old_state = ircut->state;
	if (op > 0) {
		/* check state */
		if (
			IRCUT_STATE_EQ(IRCUT_STATE_OPENING) ||
			IRCUT_STATE_EQ(IRCUT_STATE_OPENED)) {
			/* already in opening or opened state, do nothing */
			do_nothing = true;
		} else if (IRCUT_STATE_EQ(IRCUT_STATE_CLOSING)) {
			/* in closing state,should wait */
			should_wait = true;
		} else {
			/* in closed state,queue work */
		}
	} else {
		/* check state */
		if (
			IRCUT_STATE_EQ(IRCUT_STATE_CLOSING) ||
			IRCUT_STATE_EQ(IRCUT_STATE_CLOSED)) {
			/* already in closing or closed state, do nothing */
			do_nothing = true;
		} else if (IRCUT_STATE_EQ(IRCUT_STATE_OPENING)) {
			/* in opening state,should wait */
			should_wait = true;
		} else {
			/* in opened state,queue work */
		}
	}
	mutex_unlock(&ircut->mut_state);
	if (do_nothing)
		goto op_done;
	if (should_wait)
		wait_for_completion(&ircut->complete);
	wk = kmalloc(
		sizeof(struct ircut_op_work),
		GFP_ATOMIC);
	if (!wk) {
		dev_err(ircut->dev, "failed to allock ircut work struct\n");
		goto err_ircut_operation;
	}
	wk->op_cmd = op;
	wk->dev = ircut;
	/* change state, lock is unnecessary here,cause no flying work now */
	if (op > 0)
		ircut->state = IRCUT_STATE_OPENING;
	else
		ircut->state = IRCUT_STATE_CLOSING;
	init_completion(&ircut->complete);
	/* queue work */
	INIT_WORK(
		(struct work_struct *)&wk->work,
		ircut_op_work);
	if (!queue_work
		(ircut->wq, (struct work_struct *)&wk->work)
		) {
		dev_err(ircut->dev, "queue work failed\n");
		kfree(wk);
		ircut->state = old_state;
		goto err_ircut_operation;
	}
op_done:
	return 0;
err_ircut_operation:
	return -1;
}

static int ircut_g_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	struct ircut_dev *ircut = NULL;

	ircut = v4l2_get_subdevdata(sd);

	if (ctrl->id == V4L2_CID_BAND_STOP_FILTER) {
		ctrl->value = ircut->val;
		return 0;
	} else {
		return -EINVAL;
	}
}

/* ======================================================================== */

static int ircut_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	int ret = -EINVAL;
	struct ircut_dev *ircut = NULL;

	ircut = v4l2_get_subdevdata(sd);

	if (ctrl->id == V4L2_CID_BAND_STOP_FILTER) {
		ret = ircut_operation(ircut, ctrl->value);
		if (ret == 0)
			ircut->val = ctrl->value;
	}
	return ret;
}

/* ======================================================================== */

static struct v4l2_subdev_core_ops ircut_core_ops = {
	.g_ctrl = ircut_g_ctrl,
	.s_ctrl = ircut_s_ctrl,
};

static struct v4l2_subdev_ops ircut_ops = {
	.core = &ircut_core_ops,
};

static int ircut_probe(struct platform_device *pdev)
{
	struct ircut_dev *ircut = NULL;
	struct device_node *node = pdev->dev.of_node;

	dev_info(&pdev->dev, "probing...");
	ircut = devm_kzalloc(&pdev->dev, sizeof(*ircut), GFP_KERNEL);
	if (!ircut) {
		dev_err(&pdev->dev, "alloc ircut failed\n");
		return -ENOMEM;
	}

	ircut->dev = &pdev->dev;
	mutex_init(&ircut->mut_state);
	init_completion(&ircut->complete);
	ircut->wq = alloc_workqueue("ircut wq",
			WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	v4l2_subdev_init(&ircut->sd, &ircut_ops);
	ircut->sd.owner = pdev->dev.driver->owner;
	v4l2_set_subdevdata(&ircut->sd, ircut);
	platform_set_drvdata(pdev, &ircut->sd);

	if (ircut_gpio_parse_dt(ircut, node) < 0)
		return -EBUSY;
	/* set defalut state to open */
	ircut_operation(ircut, 1);
	ircut->val = 1;
	dev_info(&pdev->dev, "probe successful!");
	return 0;
}

static int ircut_drv_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct ircut_dev *ircut = NULL;

	if (sd)
		ircut = v4l2_get_subdevdata(sd);
	if (ircut && ircut->wq)
		drain_workqueue(ircut->wq);
	if (sd)
		v4l2_device_unregister_subdev(sd);
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id ircut_of_match[] = {
	{ .compatible = "rockchip,ircut", },
	{},
};
#endif

static struct platform_driver ircut_driver = {
		.driver         = {
		.name   = "rockchip-ircut",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ircut_of_match),
	},
	.probe          = ircut_probe,
	.remove         = ircut_drv_remove,
};

static int __init ircut_init(void)
{
	return platform_driver_register(&ircut_driver);
}

static void __exit ircut_exit(void)
{
	platform_driver_unregister(&ircut_driver);
}

module_init(ircut_init);
module_exit(ircut_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rockchip-ircut");
MODULE_AUTHOR("ROCKCHIP");
