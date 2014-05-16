/*
 * ChromeOS EC multi-function device
 *
 * Copyright (C) 2012 Google, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * The ChromeOS EC multi function device is used to mux all the requests
 * to the EC device for its multiple features: keyboard controller,
 * battery charging and regulator control, firmware update.
 */

#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mfd/core.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/delay.h>

#define EC_COMMAND_RETRIES	50
#define EC_RETRY_DELAY_MS	10

int cros_ec_prepare_tx(struct cros_ec_device *ec_dev,
		       struct cros_ec_command *msg)
{
	uint8_t *out;
	int csum, i;

	BUG_ON(msg->outsize > EC_PROTO2_MAX_PARAM_SIZE);
	out = ec_dev->dout;
	out[0] = EC_CMD_VERSION0 + msg->version;
	out[1] = msg->command;
	out[2] = msg->outsize;
	csum = out[0] + out[1] + out[2];
	for (i = 0; i < msg->outsize; i++)
		csum += out[EC_MSG_TX_HEADER_BYTES + i] = msg->outdata[i];
	out[EC_MSG_TX_HEADER_BYTES + msg->outsize] = (uint8_t)(csum & 0xff);

	return EC_MSG_TX_PROTO_BYTES + msg->outsize;
}
EXPORT_SYMBOL(cros_ec_prepare_tx);

int cros_ec_cmd_xfer(struct cros_ec_device *ec_dev,
		     struct cros_ec_command *msg)
{
	int ret, i;

	mutex_lock(&ec_dev->lock);
	ret = ec_dev->cmd_xfer(ec_dev, msg);
	if (ret == -EAGAIN && msg->result == EC_RES_IN_PROGRESS) {
		/*
		 * Query the EC's status until it's no longer busy or
		 * we encounter an error.
		 */
		for (i = 0; i < EC_COMMAND_RETRIES; i++) {
			struct cros_ec_command status_msg;
			struct ec_response_get_comms_status status;

			msleep(EC_RETRY_DELAY_MS);

			status_msg.version = 0;
			status_msg.command = EC_CMD_GET_COMMS_STATUS;
			status_msg.outdata = NULL;
			status_msg.outsize = 0;
			status_msg.indata = (uint8_t *)&status;
			status_msg.insize = sizeof(status);

			ret = ec_dev->cmd_xfer(ec_dev, &status_msg);
			if (ret < 0)
				break;

			msg->result = status_msg.result;
			if (status_msg.result != EC_RES_SUCCESS)
				break;
			if (!(status.flags & EC_COMMS_STATUS_PROCESSING))
				break;
		}
	}
	mutex_unlock(&ec_dev->lock);

	return ret;
}
EXPORT_SYMBOL(cros_ec_cmd_xfer);

static const struct mfd_cell cros_accel_devs[] = {
	{
		.name = "cros-ec-accel",
	},
	{
		.name = "iio-trig-sysfs",
	},
};

/**
 * cros_ec_accel_register - register EC accelerometer devices
 *
 * @ec_dev Pointer to cros_ec_device
 * @return 0 if ok, -ve on error
 *
 * Check if EC has accelerometers and register related drivers if it does.
 * Return 0 if successfully registered EC drivers. Return -ENODEV if there
 * are no accelerometers attached or we can't read from the EC.
 */
static int cros_ec_accel_register(struct cros_ec_device *ec_dev)
{
	u8 status;

	/*
	 * Try to read the accel status byte from EC shared memory. If the
	 * read is successful and the status byte has the physical presence bit
	 * set, then this machine has accelerometers. If it has accelerometers,
	 * then register the cros_accel_devs.
	 */
	if (!ec_dev->cmd_read_u8)
		return -ENODEV;

	if (ec_dev->cmd_read_u8(ec_dev, EC_MEMMAP_ACC_STATUS, &status) < 0)
		return -ENODEV;

	if (status & EC_MEMMAP_ACC_STATUS_PRESENCE_BIT)
		return mfd_add_devices(ec_dev->dev, 0, cros_accel_devs,
				      ARRAY_SIZE(cros_accel_devs),
				      NULL, ec_dev->irq, NULL);
	else
		return -ENODEV;
}

static const struct mfd_cell cros_devs[] = {
	{
		.name = "cros-ec-dev",
		.id = 0,
	},
};

int cros_ec_register(struct cros_ec_device *ec_dev)
{
	struct device *dev = ec_dev->dev;
	int err = 0;
#ifdef CONFIG_OF
	struct device_node *node;
	int id = ARRAY_SIZE(cros_devs);
#endif

	if (ec_dev->din_size) {
		ec_dev->din = devm_kzalloc(dev, ec_dev->din_size, GFP_KERNEL);
		if (!ec_dev->din)
			return -ENOMEM;
	}
	if (ec_dev->dout_size) {
		ec_dev->dout = devm_kzalloc(dev, ec_dev->dout_size, GFP_KERNEL);
		if (!ec_dev->dout)
			return -ENOMEM;
	}

	mutex_init(&ec_dev->lock);

	err = mfd_add_devices(dev, 0, cros_devs,
			      ARRAY_SIZE(cros_devs),
			      NULL, ec_dev->irq, NULL);
	if (err) {
		dev_err(dev, "failed to add mfd devices\n");
		return err;
	}

	err = cros_ec_accel_register(ec_dev);
	if (err && err != -ENODEV)
		dev_err(dev, "failed to add cros-ec-accel mfd devices\n");

#ifdef CONFIG_OF
	/*
	 * Add sub-devices declared in the device tree.  NOTE they should NOT be
	 * declared in cros_devs
	 */
	for_each_child_of_node(dev->of_node, node) {
		char name[128];
		struct mfd_cell cell = {
			.id = 0,
			.name = name,
		};

		if (of_modalias_node(node, name, sizeof(name)) < 0) {
			dev_err(dev, "modalias failure on %s\n",
				node->full_name);
			continue;
		}
		dev_dbg(dev, "adding MFD sub-device %s\n", node->name);
		cell.of_compatible = of_get_property(node, "compatible", NULL);
		err = mfd_add_devices(dev, ++id, &cell, 1, NULL, ec_dev->irq,
				      NULL);
		if (err)
			dev_err(dev, "fail to add %s\n", node->full_name);
	}
#endif

	dev_info(dev, "Chrome EC device registered\n");

	return 0;
}
EXPORT_SYMBOL(cros_ec_register);

int cros_ec_remove(struct cros_ec_device *ec_dev)
{
	mfd_remove_devices(ec_dev->dev);
	return 0;
}
EXPORT_SYMBOL(cros_ec_remove);

#ifdef CONFIG_PM_SLEEP
int cros_ec_suspend(struct cros_ec_device *ec_dev)
{
	struct device *dev = ec_dev->dev;

	if (device_may_wakeup(dev))
		ec_dev->wake_enabled = !enable_irq_wake(ec_dev->irq);

	disable_irq(ec_dev->irq);
	ec_dev->was_wake_device = ec_dev->wake_enabled;

	return 0;
}
EXPORT_SYMBOL(cros_ec_suspend);

int cros_ec_resume(struct cros_ec_device *ec_dev)
{
	enable_irq(ec_dev->irq);

	if (ec_dev->wake_enabled) {
		disable_irq_wake(ec_dev->irq);
		ec_dev->wake_enabled = 0;
	}

	return 0;
}
EXPORT_SYMBOL(cros_ec_resume);

#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ChromeOS EC core driver");
