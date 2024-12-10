// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim Quad GMSL2/GMSL1 to CSI-2 Deserializer driver
 *
 * Copyright (C) 2023 Rockchip Electronics Co., Ltd.
 *
 * Author: Cai Wenzhong <cwz@rock-chips.com>
 *
 * V2.00.00 maxim serdes quad GMSL2/GMSL1 driver framework.
 *     1. local deserializer support: max96712/max96722
 *     2. remote serializer support: max9295/max96715/max96717
 *     3. support deserializer and serializer auto adaptive
 *     4. support deserializer output test pattern
 *     5. support remote serializer I2c address mapping
 *     6. support remote serializer hot plug detection and recovery
 *
 * V2.01.00
 *     1. remote device and local link are bound through link id
 *     2. support local and remote port chain check
 *     3. drivers/media/i2c/maxim4c/Kconfig support menu select
 *     4. optimize delay time and error messages
 *     5. power control: local by pwdn gpio, remote by pocen gpio
 *     6. local pwdn on/off enable depend on MAXIM4C_LOCAL_DES_ON_OFF_EN
 *
 * V2.02.00
 *     1. Force all MIPI clocks running Setting in csi out enable.
 *     2. Pattern mode force_clock_out_en default enable.
 *
 * V2.03.00
 *     1. remote device add the maxim4c prefix to driver name.
 *
 * V2.04.04
 *     1. Add regulator supplier dependencies.
 *     2. Add config ssc-ratio property
 *     3. Add debugfs entry to change MIPI timing
 *     4. Use PM runtime autosuspend feature
 *     5. Fix unbalanced disabling for PoC regulator
 *     6. MIPI VC count does not affected by data lane count
 *
 * V2.05.00
 *     1. local device power on add some delay for i2c normal access.
 *     2. enable hot plug detect for partial links are locked.
 *     3. remote device hot plug init disable lock irq.
 *
 */
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/compat.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/rk-camera-module.h>
#include <linux/gpio/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "maxim4c_api.h"

#define DRIVER_VERSION			KERNEL_VERSION(2, 0x05, 0x00)

#define MAXIM4C_XVCLK_FREQ		25000000

static const char *const maxim4c_supply_names[MAXIM4C_NUM_SUPPLIES] = {
	"vcc1v2",
	"vcc1v8",
};

static int maxim4c_check_local_chipid(maxim4c_t *maxim4c)
{
	struct i2c_client *client = maxim4c->client;
	struct device *dev = &client->dev;
	int ret = 0, loop = 0;
	u8 chipid = 0;

	for (loop = 0; loop < 5; loop++) {
		if (loop != 0) {
			dev_info(dev, "check local chipid retry (%d)", loop);
			msleep(10);
		}

		ret = maxim4c_i2c_read_byte(client,
				MAXIM4C_REG_CHIP_ID, MAXIM4C_I2C_REG_ADDR_16BITS,
				&chipid);
		if (ret == 0) {
			if (chipid == maxim4c->chipid) {
				if (chipid == MAX96712_CHIP_ID) {
					dev_info(dev, "MAX96712 is Detected\n");
					return 0;
				}

				if (chipid == MAX96722_CHIP_ID) {
					dev_info(dev, "MAX96722 is Detected\n");
					return 0;
				}
			} else {
				// if chipid is unexpected, retry
				dev_err(dev, "Unexpected maxim chipid = %02x\n", chipid);
			}
		}
	}

	dev_err(dev, "maxim check chipid error, ret(%d)\n", ret);

	return -ENODEV;
}

static irqreturn_t maxim4c_hot_plug_detect_irq_handler(int irq, void *dev_id)
{
	struct maxim4c *maxim4c = dev_id;
	struct device *dev = &maxim4c->client->dev;
	int lock_gpio_level = 0;

	mutex_lock(&maxim4c->mutex);
	if (maxim4c->streaming) {
		lock_gpio_level = gpiod_get_value_cansleep(maxim4c->lock_gpio);
		if (lock_gpio_level == 0) {
			dev_info(dev, "serializer hot plug out\n");

			maxim4c->hot_plug_state = MAXIM4C_HOT_PLUG_OUT;
		} else {
			dev_info(dev, "serializer hot plug in\n");

			maxim4c->hot_plug_state = MAXIM4C_HOT_PLUG_IN;
		}

		queue_delayed_work(maxim4c->hot_plug_work.state_check_wq,
					&maxim4c->hot_plug_work.state_d_work,
					msecs_to_jiffies(100));
	}
	mutex_unlock(&maxim4c->mutex);

	return IRQ_HANDLED;
}

static void maxim4c_lock_irq_init(maxim4c_t *maxim4c)
{
	struct device *dev = &maxim4c->client->dev;
	int ret = 0;

	if (!IS_ERR(maxim4c->lock_gpio)) {
		maxim4c->hot_plug_irq = gpiod_to_irq(maxim4c->lock_gpio);
		if (maxim4c->hot_plug_irq < 0) {
			dev_err(dev, "failed to get hot plug irq\n");
		} else {
			ret = devm_request_threaded_irq(dev,
					maxim4c->hot_plug_irq,
					NULL,
					maxim4c_hot_plug_detect_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"maxim4c_hot_plug",
					maxim4c);
			if (ret) {
				dev_err(dev, "failed to request hot plug irq (%d)\n", ret);
				maxim4c->hot_plug_irq = -1;
			} else {
				disable_irq(maxim4c->hot_plug_irq);
			}
		}
	}
}

static void maxim4c_hot_plug_state_check_work(struct work_struct *work)
{
	struct maxim4c_hot_plug_work *hot_plug_work =
		container_of(work, struct maxim4c_hot_plug_work, state_d_work.work);
	struct maxim4c *maxim4c =
		container_of(hot_plug_work, struct maxim4c, hot_plug_work);
	struct device *dev = &maxim4c->client->dev;
	u8 curr_lock_state = 0, last_lock_state = 0, link_lock_change = 0;
	u8 link_enable_mask = 0, link_id = 0;

	dev_dbg(dev, "%s\n", __func__);

	mutex_lock(&maxim4c->mutex);
	if (maxim4c->streaming == 0) {
		mutex_unlock(&maxim4c->mutex);
		return;
	}

	link_enable_mask = maxim4c->gmsl_link.link_enable_mask;
	last_lock_state = maxim4c->link_lock_state;
	if ((maxim4c->hot_plug_state == MAXIM4C_HOT_PLUG_OUT)
			&& (last_lock_state == link_enable_mask)) {
		maxim4c_link_select_remote_control(maxim4c, 0);
	}

	curr_lock_state = maxim4c_link_get_lock_state(maxim4c, link_enable_mask);
	link_lock_change = (last_lock_state ^ curr_lock_state);
	if (link_lock_change) {
		dev_dbg(dev, "lock state: current = 0x%02x, last = 0x%02x\n",
			curr_lock_state, last_lock_state);

		maxim4c->link_lock_state = curr_lock_state;
	}

	if (link_lock_change & MAXIM4C_LINK_MASK_A) {
		link_id = MAXIM4C_LINK_ID_A;

		if (curr_lock_state & MAXIM4C_LINK_MASK_A) {
			dev_info(dev, "Link A plug in\n");

			if (maxim4c->hot_plug_irq > 0)
				disable_irq(maxim4c->hot_plug_irq);

			maxim4c_remote_devices_init(maxim4c, MAXIM4C_LINK_MASK_A);

			if (maxim4c->hot_plug_irq > 0)
				enable_irq(maxim4c->hot_plug_irq);

			maxim4c_video_pipe_linkid_enable(maxim4c, link_id, true);
		} else {
			dev_info(dev, "Link A plug out\n");

			maxim4c_video_pipe_linkid_enable(maxim4c, link_id, false);
		}
	}

	if (link_lock_change & MAXIM4C_LINK_MASK_B) {
		link_id = MAXIM4C_LINK_ID_B;

		if (curr_lock_state & MAXIM4C_LINK_MASK_B) {
			dev_info(dev, "Link B plug in\n");

			if (maxim4c->hot_plug_irq > 0)
				disable_irq(maxim4c->hot_plug_irq);

			maxim4c_remote_devices_init(maxim4c, MAXIM4C_LINK_MASK_B);

			if (maxim4c->hot_plug_irq > 0)
				enable_irq(maxim4c->hot_plug_irq);

			maxim4c_video_pipe_linkid_enable(maxim4c, link_id, true);
		} else {
			dev_info(dev, "Link B plug out\n");

			maxim4c_video_pipe_linkid_enable(maxim4c, link_id, false);
		}
	}

	if (link_lock_change & MAXIM4C_LINK_MASK_C) {
		link_id = MAXIM4C_LINK_ID_C;

		if (curr_lock_state & MAXIM4C_LINK_MASK_C) {
			dev_info(dev, "Link C plug in\n");

			if (maxim4c->hot_plug_irq > 0)
				disable_irq(maxim4c->hot_plug_irq);

			maxim4c_remote_devices_init(maxim4c, MAXIM4C_LINK_MASK_C);

			if (maxim4c->hot_plug_irq > 0)
				enable_irq(maxim4c->hot_plug_irq);

			maxim4c_video_pipe_linkid_enable(maxim4c, link_id, true);
		} else {
			dev_info(dev, "Link C plug out\n");

			maxim4c_video_pipe_linkid_enable(maxim4c, link_id, false);
		}
	}

	if (link_lock_change & MAXIM4C_LINK_MASK_D) {
		link_id = MAXIM4C_LINK_ID_D;

		if (curr_lock_state & MAXIM4C_LINK_MASK_D) {
			dev_info(dev, "Link D plug in\n");

			if (maxim4c->hot_plug_irq > 0)
				disable_irq(maxim4c->hot_plug_irq);

			maxim4c_remote_devices_init(maxim4c, MAXIM4C_LINK_MASK_D);

			if (maxim4c->hot_plug_irq > 0)
				enable_irq(maxim4c->hot_plug_irq);

			maxim4c_video_pipe_linkid_enable(maxim4c, link_id, true);
		} else {
			dev_info(dev, "Link D plug out\n");

			maxim4c_video_pipe_linkid_enable(maxim4c, link_id, false);
		}
	}

	if (curr_lock_state == link_enable_mask) {
		// remote control mask enable
		maxim4c_link_select_remote_control(maxim4c, link_enable_mask);
	} else {
		queue_delayed_work(maxim4c->hot_plug_work.state_check_wq,
				&maxim4c->hot_plug_work.state_d_work,
				msecs_to_jiffies(200));
	}

	mutex_unlock(&maxim4c->mutex);
}

int maxim4c_hot_plug_detect_work_start(maxim4c_t *maxim4c)
{
	struct device *dev = &maxim4c->client->dev;
	u8 link_lock_state = 0, link_enable_mask = 0;

	link_lock_state = maxim4c->link_lock_state;
	link_enable_mask = maxim4c->gmsl_link.link_enable_mask;

	if (link_lock_state != link_enable_mask) {
		dev_info(dev, "%s: link_lock = 0x%02x, link_mask = 0x%02x\n",
			__func__, link_lock_state, link_enable_mask);

		maxim4c->hot_plug_state = MAXIM4C_HOT_PLUG_OUT;

		queue_delayed_work(maxim4c->hot_plug_work.state_check_wq,
				&maxim4c->hot_plug_work.state_d_work,
				msecs_to_jiffies(200));
	}

	return 0;
}

static int maxim4c_lock_state_work_init(maxim4c_t *maxim4c)
{
	struct device *dev = &maxim4c->client->dev;

	INIT_DELAYED_WORK(&maxim4c->hot_plug_work.state_d_work,
			maxim4c_hot_plug_state_check_work);
	maxim4c->hot_plug_work.state_check_wq =
		create_singlethread_workqueue("maxim4c work queue");
	if (maxim4c->hot_plug_work.state_check_wq == NULL) {
		dev_err(dev, "failed to create hot plug work queue\n");
		return -ENOMEM;
	}

	return 0;
}

static int maxim4c_lock_state_work_deinit(maxim4c_t *maxim4c)
{
	if (maxim4c->hot_plug_work.state_check_wq) {
		cancel_delayed_work_sync(&maxim4c->hot_plug_work.state_d_work);
		destroy_workqueue(maxim4c->hot_plug_work.state_check_wq);
		maxim4c->hot_plug_work.state_check_wq = NULL;
	}

	return 0;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 maxim4c_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, MAXIM4C_XVCLK_FREQ / 1000 / 1000);
}

static int maxim4c_local_device_power_on(maxim4c_t *maxim4c)
{
	struct device *dev = &maxim4c->client->dev;
	int ret;

	ret = regulator_bulk_enable(MAXIM4C_NUM_SUPPLIES, maxim4c->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		return -EINVAL;
	}

	if (!IS_ERR(maxim4c->pwdn_gpio)) {
		dev_info(dev, "local device pwdn gpio on\n");

		gpiod_set_value_cansleep(maxim4c->pwdn_gpio, 1);

		usleep_range(10000, 11000);
	}

	return 0;
}

static void maxim4c_local_device_power_off(maxim4c_t *maxim4c)
{
	struct device *dev = &maxim4c->client->dev;
	int ret;

	if (!IS_ERR(maxim4c->pwdn_gpio)) {
		dev_info(dev, "local device pwdn gpio off\n");

		gpiod_set_value_cansleep(maxim4c->pwdn_gpio, 0);
	}

	ret = regulator_bulk_disable(MAXIM4C_NUM_SUPPLIES, maxim4c->supplies);
	if (ret < 0) {
		dev_warn(dev, "Failed to disable regulators\n");
	}
}

static int maxim4c_remote_device_power_on(maxim4c_t *maxim4c)
{
	struct device *dev = &maxim4c->client->dev;
	int ret;

	dev_dbg(dev, "Turn PoC on\n");
	ret = regulator_enable(maxim4c->poc_regulator);
	if (ret < 0) {
		dev_err(dev, "Unable to turn PoC on\n");
		return ret;
	}

	return 0;
}

static int maxim4c_remote_device_power_off(maxim4c_t *maxim4c)
{
	struct device *dev = &maxim4c->client->dev;
	int ret;

	dev_dbg(dev, "Turn PoC off\n");
	ret = regulator_disable(maxim4c->poc_regulator);
	if (ret < 0)
		dev_warn(dev, "Unable to turn PoC off\n");

	return 0;
}

static int maxim4c_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct maxim4c *maxim4c = v4l2_get_subdevdata(sd);
	int ret = 0;

#if MAXIM4C_LOCAL_DES_ON_OFF_EN
	ret |= maxim4c_local_device_power_on(maxim4c);
#endif /* MAXIM4C_LOCAL_DES_ON_OFF_EN */

	ret |= maxim4c_remote_device_power_on(maxim4c);

	return ret;
}

static int maxim4c_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct maxim4c *maxim4c = v4l2_get_subdevdata(sd);
	int ret = 0;

	ret |= maxim4c_remote_device_power_off(maxim4c);

#if MAXIM4C_LOCAL_DES_ON_OFF_EN
	maxim4c_local_device_power_off(maxim4c);
#endif /* MAXIM4C_LOCAL_DES_ON_OFF_EN */

	return ret;
}

static const struct dev_pm_ops maxim4c_pm_ops = {
	SET_RUNTIME_PM_OPS(
		maxim4c_runtime_suspend, maxim4c_runtime_resume, NULL)
};

static void maxim4c_module_data_init(maxim4c_t *maxim4c)
{
	maxim4c_link_data_init(maxim4c);
	maxim4c_video_pipe_data_init(maxim4c);
	maxim4c_mipi_txphy_data_init(maxim4c);
}

static int maxim4c_extra_init_seq_parse(maxim4c_t *maxim4c, struct device_node *node)
{
	struct device *dev = &maxim4c->client->dev;
	struct device_node *init_seq_node = NULL;
	struct maxim4c_i2c_init_seq *init_seq = NULL;

	init_seq_node = of_get_child_by_name(node, "extra-init-sequence");
	if (IS_ERR_OR_NULL(init_seq_node)) {
		dev_dbg(dev, "%pOF no child node extra-init-sequence\n", node);
		return 0;
	}

	if (!of_device_is_available(init_seq_node)) {
		dev_dbg(dev, "%pOF is disabled\n", init_seq_node);

		of_node_put(init_seq_node);
		return 0;
	}

	dev_info(dev, "load extra-init-sequence\n");

	init_seq = &maxim4c->extra_init_seq;
	maxim4c_i2c_load_init_seq(dev,
			init_seq_node, init_seq);

	of_node_put(init_seq_node);

	return 0;
}

static int maxim4c_module_parse_dt(maxim4c_t *maxim4c)
{
	struct device *dev = &maxim4c->client->dev;
	struct device_node *node = NULL;

	// maxim serdes local
	node = of_get_child_by_name(dev->of_node, "serdes-local-device");
	if (IS_ERR_OR_NULL(node)) {
		dev_err(dev, "%pOF has no child node: serdes-local-device\n",
				dev->of_node);

		return -ENODEV;
	}

	if (!of_device_is_available(node)) {
		dev_info(dev, "%pOF is disabled\n", node);

		of_node_put(node);
		return -ENODEV;
	}

	/* gmsl link parse dt */
	maxim4c_link_parse_dt(maxim4c, node);

	/* video pipe parse dt */
	maxim4c_video_pipe_parse_dt(maxim4c, node);

	/* mipi txphy parse dt */
	maxim4c_mipi_txphy_parse_dt(maxim4c, node);

	/* extra init seq parse dt */
	maxim4c_extra_init_seq_parse(maxim4c, node);

	of_node_put(node);

	return 0;
}

static int maxim4c_run_extra_init_seq(maxim4c_t *maxim4c)
{
	struct i2c_client *client = maxim4c->client;
	struct device *dev = &client->dev;
	int ret = 0;

	ret = maxim4c_i2c_run_init_seq(client,
			&maxim4c->extra_init_seq);
	if (ret) {
		dev_err(dev, "extra init sequence error\n");
		return ret;
	}

	return 0;
}

static int maxim4c_module_hw_previnit(maxim4c_t *maxim4c)
{
	struct i2c_client *client = maxim4c->client;
	int ret = 0;

	// All links disable at beginning.
	ret = maxim4c_i2c_write_byte(client,
			0x0006, MAXIM4C_I2C_REG_ADDR_16BITS,
			0xF0);
	if (ret)
		return ret;

	// MIPI CSI output disable.
	ret = maxim4c_i2c_write_byte(client,
			0x040B, MAXIM4C_I2C_REG_ADDR_16BITS,
			0x00);
	if (ret)
		return ret;

	// MIPI TXPHY standby
	ret = maxim4c_i2c_update_byte(client,
			0x08A2, MAXIM4C_I2C_REG_ADDR_16BITS,
			0xF0, 0x00);
	if (ret)
		return ret;

	return 0;
}

static int maxim4c_module_hw_postinit(maxim4c_t *maxim4c)
{
	struct i2c_client *client = maxim4c->client;
	int ret = 0;

	// video pipe disable all
	ret |= maxim4c_i2c_write_byte(client,
			0x00F4, MAXIM4C_I2C_REG_ADDR_16BITS,
			0);

	// remote control disable all
	ret |= maxim4c_link_select_remote_control(maxim4c, 0);

	return ret;
}

int maxim4c_module_hw_init(maxim4c_t *maxim4c)
{
	struct device *dev = &maxim4c->client->dev;
	int ret = 0;

	ret = maxim4c_module_hw_previnit(maxim4c);
	if (ret) {
		dev_err(dev, "%s: hw prev init error\n", __func__);

		return ret;
	}

	ret = maxim4c_link_hw_init(maxim4c);
	if (ret) {
		dev_err(dev, "%s: hw link init error\n", __func__);
		return ret;
	}

	ret = maxim4c_video_pipe_hw_init(maxim4c);
	if (ret) {
		dev_err(dev, "%s: hw pipe init error\n", __func__);
		return ret;
	}

	ret = maxim4c_mipi_txphy_hw_init(maxim4c);
	if (ret) {
		dev_err(dev, "%s: hw txphy init error\n", __func__);
		return ret;
	}

	ret = maxim4c_run_extra_init_seq(maxim4c);
	if (ret) {
		dev_err(dev, "%s: run extra init seq error\n", __func__);
		return ret;
	}

	ret = maxim4c_module_hw_postinit(maxim4c);
	if (ret) {
		dev_err(dev, "%s: hw post init error\n", __func__);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(maxim4c_module_hw_init);

static int maxim4c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	maxim4c_t *maxim4c = NULL;
	u32 chip_id;
	int ret = 0;
	unsigned int i;

	dev_info(dev, "driver version: %02x.%02x.%02x", DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8, DRIVER_VERSION & 0x00ff);

	chip_id = (uintptr_t)of_device_get_match_data(dev);
	if (chip_id == MAX96712_CHIP_ID) {
		dev_info(dev, "maxim4c driver for max96712\n");
	} else if (chip_id == MAX96722_CHIP_ID) {
		dev_info(dev, "maxim4c driver for max96722\n");
	} else {
		dev_err(dev, "maxim4c driver unknown chip\n");
		return -EINVAL;
	}

	maxim4c = devm_kzalloc(dev, sizeof(*maxim4c), GFP_KERNEL);
	if (!maxim4c) {
		dev_err(dev, "maxim4c probe no memory error\n");
		return -ENOMEM;
	}

	maxim4c->client = client;
	maxim4c->chipid = chip_id;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &maxim4c->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &maxim4c->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &maxim4c->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &maxim4c->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	maxim4c->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(maxim4c->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios, maybe no use\n");
	else
		usleep_range(1000, 1100);

	maxim4c->lock_gpio = devm_gpiod_get(dev, "lock", GPIOD_IN);
	if (IS_ERR(maxim4c->lock_gpio))
		dev_warn(dev, "Failed to get lock-gpios\n");

	for (i = 0; i < MAXIM4C_NUM_SUPPLIES; i++)
		maxim4c->supplies[i].supply = maxim4c_supply_names[i];

	ret = devm_regulator_bulk_get(dev, MAXIM4C_NUM_SUPPLIES,
				      maxim4c->supplies);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Unable to get supply regulators\n");
		else
			dev_warn(dev, "Get PoC regulator deferred\n");
		return ret;
	}

	maxim4c->poc_regulator = devm_regulator_get(dev, "poc");
	if (IS_ERR(maxim4c->poc_regulator)) {
		if (PTR_ERR(maxim4c->poc_regulator) != -EPROBE_DEFER)
			dev_err(dev, "Unable to get PoC regulator (%ld)\n",
				PTR_ERR(maxim4c->poc_regulator));
		else
			dev_err(dev, "Get PoC regulator deferred\n");

		ret = PTR_ERR(maxim4c->poc_regulator);
#if !MAXIM4C_TEST_PATTERN
		return ret;
#endif
	}

	mutex_init(&maxim4c->mutex);

	ret = maxim4c_local_device_power_on(maxim4c);
	if (ret)
		goto err_destroy_mutex;

	ret = maxim4c_remote_device_power_on(maxim4c);
	if (ret)
		dev_warn(dev, "Power on PoC regulator failed\n");

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);

	ret = maxim4c_check_local_chipid(maxim4c);
	if (ret)
		goto err_power_off;

	// client->dev->driver_data = subdev
	// subdev->dev->driver_data = maxim4c
	ret = maxim4c_v4l2_subdev_init(maxim4c);
	if (ret) {
		dev_err(dev, "maxim4c probe v4l2 subdev init error\n");
		goto err_power_off;
	}

#if MAXIM4C_TEST_PATTERN
	ret = maxim4c_pattern_data_init(maxim4c);
	if (ret)
		goto err_power_off;

#if (MAXIM4C_LOCAL_DES_ON_OFF_EN == 0)
	ret = maxim4c_pattern_hw_init(maxim4c);
	if (ret)
		goto err_power_off;
#endif /* MAXIM4C_LOCAL_DES_ON_OFF_EN */

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;
#endif /* MAXIM4C_TEST_PATTERN */

	maxim4c_module_data_init(maxim4c);
	maxim4c_module_parse_dt(maxim4c);

	ret = maxim4c_dbgfs_init(maxim4c);
	if (ret)
		goto err_subdev_deinit;

#if (MAXIM4C_LOCAL_DES_ON_OFF_EN == 0)
	ret = maxim4c_module_hw_init(maxim4c);
	if (ret)
		goto err_dbgfs_deinit;
#endif /* MAXIM4C_LOCAL_DES_ON_OFF_EN */

	ret = maxim4c_remote_mfd_add_devices(maxim4c);
	if (ret)
		goto err_dbgfs_deinit;

	maxim4c_lock_irq_init(maxim4c);
	maxim4c_lock_state_work_init(maxim4c);

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

err_dbgfs_deinit:
	maxim4c_dbgfs_deinit(maxim4c);
err_subdev_deinit:
	maxim4c_v4l2_subdev_deinit(maxim4c);
err_power_off:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	maxim4c_remote_device_power_off(maxim4c);
	maxim4c_local_device_power_off(maxim4c);
err_destroy_mutex:
	mutex_destroy(&maxim4c->mutex);

	return ret;
}

static int maxim4c_remove(struct i2c_client *client)
{
	maxim4c_t *maxim4c = i2c_get_clientdata(client);

	maxim4c_lock_state_work_deinit(maxim4c);

	maxim4c_dbgfs_deinit(maxim4c);

	maxim4c_v4l2_subdev_deinit(maxim4c);

	mutex_destroy(&maxim4c->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		maxim4c_local_device_power_off(maxim4c);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct of_device_id maxim4c_of_match[] = {
	{
		.compatible = "maxim4c,max96712",
		.data = (const void *)MAX96712_CHIP_ID
	}, {
		.compatible = "maxim4c,max96722",
		.data = (const void *)MAX96722_CHIP_ID
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, maxim4c_of_match);

static struct i2c_driver maxim4c_i2c_driver = {
	.driver = {
		.name = MAXIM4C_NAME,
		.pm = &maxim4c_pm_ops,
		.of_match_table = of_match_ptr(maxim4c_of_match),
	},
	.probe		= &maxim4c_probe,
	.remove		= &maxim4c_remove,
};

module_i2c_driver(maxim4c_i2c_driver);

MODULE_AUTHOR("Cai Wenzhong <cwz@rock-chips.com>");
MODULE_DESCRIPTION("Maxim quad gmsl deserializer driver");
MODULE_LICENSE("GPL");
