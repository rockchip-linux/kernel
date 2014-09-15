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

#include "cros_ec_dev.h"

#define EC_COMMAND_RETRIES	50
#define EC_RETRY_DELAY_MS	10

static int dev_id;

static int prepare_packet(struct cros_ec_device *ec_dev,
			  struct cros_ec_command *msg)
{
	struct ec_host_request *request;
	u8 *out;
	int i;
	u8 csum = 0;

	BUG_ON(ec_dev->proto_version != EC_HOST_REQUEST_VERSION);
	BUG_ON(msg->outsize + sizeof(*request) > ec_dev->dout_size);

	out = ec_dev->dout;
	request = (struct ec_host_request *)out;
	request->struct_version = EC_HOST_REQUEST_VERSION;
	request->checksum = 0;
	request->command = msg->command;
	request->command_version = msg->version;
	request->reserved = 0;
	request->data_len = msg->outsize;

	for (i = 0; i < sizeof(*request); i++)
		csum += out[i];

	/* Copy data and update checksum */
	memcpy(out + sizeof(*request), msg->outdata, msg->outsize);
	for (i = 0; i < msg->outsize; i++)
		csum += msg->outdata[i];

	request->checksum = -csum;

	return sizeof(*request) + msg->outsize;
}

static int send_command(struct cros_ec_device *ec_dev,
			struct cros_ec_command *msg)
{
	int ret, i;

	if (ec_dev->proto_version > 2)
		ret = ec_dev->pkt_xfer(ec_dev, msg);
	else
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
			status_msg.indata = (u8 *)&status;
			status_msg.insize = sizeof(status);

			if (ec_dev->proto_version > 2)
				ret = ec_dev->pkt_xfer(ec_dev, &status_msg);
			else
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

	return ret;
}

static int cros_ec_host_command_proto_probe(struct cros_ec_device *ec_dev,
	int devidx,
	struct ec_response_get_protocol_info *info)
{
	/*
	 * Try using v3+ to query for supported protocols. If this
	 * command fails, fall back to v2. Returns the highest protocol
	 * supported by the EC.
	 * Also sets the max request/response/passthru size.
	 */

	struct cros_ec_command msg;
	int ret;

	if (!ec_dev->pkt_xfer)
		return -EPROTONOSUPPORT;

	memset(&msg, 0, sizeof(msg));
	msg.command = EC_CMD_PASSTHRU_OFFSET(devidx) | EC_CMD_GET_PROTOCOL_INFO;
	msg.indata = (u8 *)info;
	msg.insize = sizeof(*info);

	ret = send_command(ec_dev, &msg);

	if (ret < 0) {
		dev_dbg(ec_dev->dev,
			"failed to probe for EC[%d] protocol version: %d\n",
			devidx, ret);
		return ret;
	}

	if (devidx > 0 && msg.result == EC_RES_INVALID_COMMAND)
		return -ENODEV;
	else if (msg.result != EC_RES_SUCCESS)
		return msg.result;

	return 0;
}

static int cros_ec_host_command_proto_probe_v2(struct cros_ec_device *ec_dev)
{
	struct cros_ec_command msg;
	struct ec_params_hello hello_params;
	struct ec_response_hello hello_response;
	int ret;

	hello_params.in_data = 0xa0b0c0d0;

	memset(&msg, 0, sizeof(msg));
	msg.command = EC_CMD_HELLO;
	msg.outdata = (u8 *)&hello_params;
	msg.outsize = sizeof(hello_params);
	msg.indata = (u8 *)&hello_response;
	msg.insize = sizeof(hello_response);

	ret = send_command(ec_dev, &msg);

	if (ret < 0) {
		dev_dbg(ec_dev->dev,
			"EC failed to respond to v2 hello: %d\n",
			ret);
		return ret;
	} else if (msg.result != EC_RES_SUCCESS) {
		dev_err(ec_dev->dev,
			"EC responded to v2 hello with error: %d\n",
			msg.result);
		return msg.result;
	} else if (hello_response.out_data != 0xa1b2c3d4) {
		dev_err(ec_dev->dev,
			"EC responded to v2 hello with bad result: %u\n",
			hello_response.out_data);
		return -EBADMSG;
	}

	return 0;
}

static int cros_ec_probe_all(struct cros_ec_device *ec_dev)
{
	struct device *dev = ec_dev->dev;
	struct ec_response_get_protocol_info proto_info;
	int ret;

	/* First try sending with proto v3. */
	ec_dev->proto_version = 3;
	ret = cros_ec_host_command_proto_probe(ec_dev, 0, &proto_info);

	if (ret == 0) {
		ec_dev->max_request = proto_info.max_request_packet_size -
			sizeof(struct ec_host_request);
		ec_dev->max_response = proto_info.max_response_packet_size -
			sizeof(struct ec_host_response);
		ec_dev->proto_version =
			min(EC_HOST_REQUEST_VERSION,
					fls(proto_info.protocol_versions) - 1);
		dev_dbg(ec_dev->dev,
			"using proto v%u\n",
			ec_dev->proto_version);

		ec_dev->din_size = ec_dev->max_response +
			sizeof(struct ec_host_response) +
			EC_MAX_RESPONSE_OVERHEAD;
		ec_dev->dout_size = ec_dev->max_request +
			sizeof(struct ec_host_request) +
			EC_MAX_REQUEST_OVERHEAD;

		/*
		 * Check for PD
		 * TODO(gwendal):crbug/31456: add specific driver for samus PD
		 */
		ret = cros_ec_host_command_proto_probe(ec_dev, 1, &proto_info);

		if (ret) {
			dev_dbg(ec_dev->dev, "no PD chip found: %d\n", ret);
			ec_dev->max_passthru = 0;
		} else {
			dev_dbg(ec_dev->dev, "found PD chip\n");
			ec_dev->max_passthru =
				proto_info.max_request_packet_size -
				sizeof(struct ec_host_request);
		}
	} else {
		/* Try probing with a v2 hello message. */
		ec_dev->proto_version = 2;
		ret = cros_ec_host_command_proto_probe_v2(ec_dev);

		if (ret == 0) {
			/* V2 hello succeeded. */
			dev_dbg(ec_dev->dev, "falling back to proto v2\n");

			ec_dev->max_request = EC_PROTO2_MAX_PARAM_SIZE;
			ec_dev->max_response = EC_PROTO2_MAX_PARAM_SIZE;
			ec_dev->max_passthru = 0;
			ec_dev->pkt_xfer = NULL;
			ec_dev->din_size = EC_MSG_BYTES;
			ec_dev->dout_size = EC_MSG_BYTES;
		} else {
			/*
			 * It's possible for a probe to occur too early when
			 * the EC isn't listening. If this happens, we'll
			 * probe later when the first command is run.
			 */
			ec_dev->proto_version = EC_PROTO_VERSION_UNKNOWN;
			dev_dbg(ec_dev->dev, "EC probe failed: %d\n", ret);
			return ret;
		}
	}

	devm_kfree(dev, ec_dev->din);
	devm_kfree(dev, ec_dev->dout);

	ec_dev->din = devm_kzalloc(dev, ec_dev->din_size, GFP_KERNEL);
	if (!ec_dev->din)
		return -ENOMEM;
	ec_dev->dout = devm_kzalloc(dev, ec_dev->dout_size, GFP_KERNEL);
	if (!ec_dev->dout) {
		devm_kfree(dev, ec_dev->din);
		return -ENOMEM;
	}

	return 0;
}

int cros_ec_prepare_tx(struct cros_ec_device *ec_dev,
		       struct cros_ec_command *msg)
{
	u8 *out;
	u8 csum;
	int i;

	if (ec_dev->proto_version > 2)
		return prepare_packet(ec_dev, msg);

	BUG_ON(msg->outsize > EC_PROTO2_MAX_PARAM_SIZE);
	out = ec_dev->dout;
	out[0] = EC_CMD_VERSION0 + msg->version;
	out[1] = msg->command;
	out[2] = msg->outsize;
	csum = out[0] + out[1] + out[2];
	for (i = 0; i < msg->outsize; i++)
		csum += out[EC_MSG_TX_HEADER_BYTES + i] = msg->outdata[i];
	out[EC_MSG_TX_HEADER_BYTES + msg->outsize] = csum;

	return EC_MSG_TX_PROTO_BYTES + msg->outsize;
}
EXPORT_SYMBOL(cros_ec_prepare_tx);

int cros_ec_cmd_xfer(struct cros_ec_device *ec_dev,
		     struct cros_ec_command *msg)
{
	int ret;

	mutex_lock(&ec_dev->lock);

	if (ec_dev->proto_version == EC_PROTO_VERSION_UNKNOWN) {
		ret = cros_ec_probe_all(ec_dev);
		if (ret) {
			dev_err(ec_dev->dev,
				"EC version unknown and probe failed; aborting command\n");
			mutex_unlock(&ec_dev->lock);
			return ret;
		}
	}

	if (msg->insize > ec_dev->max_response) {
		dev_dbg(ec_dev->dev, "clamping message receive buffer\n");
		msg->insize = ec_dev->max_response;
	}

	if (msg->command < EC_CMD_PASSTHRU_OFFSET(1)) {
		if (msg->outsize > ec_dev->max_request) {
			dev_err(ec_dev->dev,
				"request of size %u is too big (max: %u)\n",
				msg->outsize,
				ec_dev->max_request);
			mutex_unlock(&ec_dev->lock);
			return -EMSGSIZE;
		}
	} else {
		if (msg->outsize > ec_dev->max_passthru) {
			dev_err(ec_dev->dev,
				"passthru request of size %u is too big (max: %u)\n",
				msg->outsize,
				ec_dev->max_passthru);
			mutex_unlock(&ec_dev->lock);
			return -EMSGSIZE;
		}
	}

	ret = send_command(ec_dev, msg);
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

static int cros_ec_dev_register(struct cros_ec_device *ec_dev,
				int dev_id, int devidx)
{
	struct device *dev = ec_dev->dev;
	struct cros_ec_platform ec_p = {
		.cmd_offset = 0,
	};
	struct mfd_cell ec_cell = {
		.name = "cros-ec-dev",
		.id = 0,
		.platform_data = &ec_p,
		.pdata_size = sizeof(ec_p),
	};
	switch (devidx) {
	case 0:
#ifdef CONFIG_OF
		ec_p.ec_name = of_get_property(dev->of_node, "devname", NULL);
		if (ec_p.ec_name == NULL) {
			dev_dbg(dev, "Name of device not found, using default");
			ec_p.ec_name = CROS_EC_DEV_NAME;
		}
#else
		ec_p.ec_name = CROS_EC_DEV_NAME;
#endif
		break;
	case 1:
		ec_p.ec_name = CROS_EC_DEV_PD_NAME;
		break;
	default:
		return -EINVAL;
	}
	ec_p.cmd_offset = EC_CMD_PASSTHRU_OFFSET(devidx);
	return mfd_add_devices(dev, dev_id, &ec_cell, 1,
			       NULL, ec_dev->irq, NULL);
}

int cros_ec_register(struct cros_ec_device *ec_dev)
{
	struct device *dev = ec_dev->dev;
	int err = 0;
#ifdef CONFIG_OF
	struct device_node *node;
	char name[128];
	struct mfd_cell cell = {
		.name = name,
		.id = 0,
	};
#endif

	ec_dev->max_request = sizeof(struct ec_params_hello);
	ec_dev->max_response = sizeof(struct ec_response_get_protocol_info);
	ec_dev->max_passthru = 0;

	ec_dev->din = devm_kzalloc(dev, ec_dev->din_size, GFP_KERNEL);
	if (!ec_dev->din)
		return -ENOMEM;
	ec_dev->dout = devm_kzalloc(dev, ec_dev->dout_size, GFP_KERNEL);
	if (!ec_dev->dout) {
		devm_kfree(dev, ec_dev->din);
		return -ENOMEM;
	}

	mutex_init(&ec_dev->lock);

	cros_ec_probe_all(ec_dev);
	err = cros_ec_dev_register(ec_dev, dev_id++, 0);
	if (err) {
		dev_err(dev, "failed to add ec\n");
		return err;
	}

	if (ec_dev->max_passthru) {
		/*
		 * Register a PD device as well on top of this device.
		 * We make the following assumptions:
		 * - behind an EC, we have a pd
		 * - only one device added.
		 * - the EC is responsive at init time (it is not true for a
		 *   sensor hub.
		 */
		err = cros_ec_dev_register(ec_dev, dev_id++, 1);
		if (err) {
			dev_err(dev, "failed to add additional ec\n");
			return err;
		}
	}

#ifdef CONFIG_OF
	/*
	 * Add sub-devices declared in the device tree.  NOTE they should NOT be
	 * declared in cros_devs
	 */
	for_each_child_of_node(dev->of_node, node) {
		if (of_modalias_node(node, name, sizeof(name)) < 0) {
			dev_err(dev, "modalias failure on %s\n",
				node->full_name);
			continue;
		}
		dev_dbg(dev, "adding MFD sub-device %s\n", node->name);
		cell.of_compatible = of_get_property(node, "compatible", NULL);
		err = mfd_add_devices(dev, dev_id++, &cell, 1, NULL,
				ec_dev->irq, NULL);
		if (err)
			dev_err(dev, "fail to add %s\n", node->full_name);
	}
#endif

	err = cros_ec_accel_register(ec_dev);
	if (err && err != -ENODEV)
		dev_err(dev, "failed to add cros-ec-accel mfd devices\n");

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
