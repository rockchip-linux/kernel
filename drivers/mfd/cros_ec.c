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

static const struct mfd_cell cros_devs[] = {
	{
		.name = "cros-ec-keyb",
		.id = 1,
		.of_compatible = "google,cros-ec-keyb",
	},
	{
		.name = "cros-ec-i2c-tunnel",
		.id = 2,
		.of_compatible = "google,cros-ec-i2c-tunnel",
	},
	{
		.name = "cros-ec-dev",
		.id = 3,
	},
	{
		.name = "cros-ec-vbc",
		.id = 4,
		.of_compatible = "google,cros-ec-vbc",
	},
	{
		.name = "cros-ec-tps65090",
		.id = 5,
		.of_compatible = "ti,cros-ec-tps65090",
	},
	{
		.name = "cros-ec-charger",
		.id = 6,
		.of_compatible = "ti,cros-ec-charger",
	},
};

int cros_ec_register(struct cros_ec_device *ec_dev)
{
	struct device *dev = ec_dev->dev;
	int err = 0;

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
