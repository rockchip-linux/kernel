/*
 *  Copyright (C) 2013 Google, Inc
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 * Expose an I2C passthrough to the ChromeOS EC.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/of_i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/**
 * struct ec_i2c_device - Driver data for I2C tunnel
 *
 * @dev: Device node
 * @adap: I2C adapter
 * @ec: Pointer to EC device
 * @remote_bus: The EC bus number we tunnel to on the other side.
 * @request_buf: Buffer for transmitting data; we expect most transfers to fit.
 * @response_buf: Buffer for receiving data; we expect most transfers to fit.
 */

struct ec_i2c_device {
	struct device *dev;
	struct i2c_adapter adap;
	struct cros_ec_device *ec;

	u16 remote_bus;

	u8 request_buf[256];
	u8 response_buf[256];
};

/**
 * ec_i2c_construct_message - construct a message to go to the EC
 *
 * This function effectively stuffs the standard i2c_msg format of Linux into
 * a format that the EC understands.
 *
 * @buf: The buffer to fill.  Can pass NULL to count how many bytes the message
 *       would be.
 * @i2c_msgs: The i2c messages to read.
 * @num: The number of i2c messages.
 * @bus_num: The remote bus number we want to talk to.
 *
 * Returns the number of bytes that the message would take up or a negative
 * error number.
 */
static int ec_i2c_construct_message(u8 *buf, const struct i2c_msg i2c_msgs[],
				    int num, u16 bus_num)
{
	struct ec_params_i2c_passthru *params;
	u8 *out_data;
	int i;
	int size;

	size = sizeof(struct ec_params_i2c_passthru);
	size += num * sizeof(struct ec_params_i2c_passthru_msg);
	out_data = buf + size;
	for (i = 0; i < num; i++)
		if (!(i2c_msgs[i].flags & I2C_M_RD))
			size += i2c_msgs[i].len;

	/* If there is no buffer, we can't build the message */
	if (!buf)
		return size;

	params = (struct ec_params_i2c_passthru *)buf;
	params->port = bus_num;
	params->num_msgs = num;
	for (i = 0; i < num; i++) {
		const struct i2c_msg *i2c_msg = &i2c_msgs[i];
		struct ec_params_i2c_passthru_msg *msg = &params->msg[i];

		msg->len = i2c_msg->len;
		msg->addr_flags = i2c_msg->addr;

		if (i2c_msg->flags & I2C_M_TEN)
			msg->addr_flags |= EC_I2C_FLAG_10BIT;

		if (i2c_msg->flags & I2C_M_RD) {
			msg->addr_flags |= EC_I2C_FLAG_READ;
		} else {
			memcpy(out_data, i2c_msg->buf, msg->len);
			out_data += msg->len;
		}
	}

	return size;
}

/**
 * ec_i2c_parse_response - Parse a response from the EC
 *
 * We'll take the EC's response and copy it back into msgs.
 *
 * @buf: The buffer to parse.  Can pass NULL to count how many bytes we expect
 *	 the response to be. Otherwise we assume that the right number of
 *	 bytes are available.
 * @i2c_msgs: The i2c messages to to fill up.
 * @num: The number of i2c messages; will be modified to include the actual
 *	 number received.
 *
 * Returns the number of response bytes or a negative error number.
 */
static int ec_i2c_parse_response(const u8 *buf, struct i2c_msg i2c_msgs[],
				 int *num)
{
	const struct ec_response_i2c_passthru *resp;
	const u8 *in_data;
	int size;
	int i;

	size = sizeof(struct ec_response_i2c_passthru);
	in_data = buf + size;
	for (i = 0; i < *num; i++)
		if (i2c_msgs[i].flags & I2C_M_RD)
			size += i2c_msgs[i].len;

	if (buf == NULL)
		return size;

	resp = (const struct ec_response_i2c_passthru *)buf;
	if (resp->i2c_status & EC_I2C_STATUS_TIMEOUT)
		return -ETIMEDOUT;
	else if (resp->i2c_status & EC_I2C_STATUS_ERROR)
		return -EREMOTEIO;

	/* Other side could send us back fewer messages, but not more */
	if (resp->num_msgs > *num)
		return -EPROTO;
	*num = resp->num_msgs;

	for (i = 0; i < *num; i++) {
		struct i2c_msg *i2c_msg = &i2c_msgs[i];

		if (i2c_msgs[i].flags & I2C_M_RD) {
			memcpy(i2c_msg->buf, in_data, i2c_msg->len);
			in_data += i2c_msg->len;
		}
	}

	return size;
}

static int ec_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg i2c_msgs[],
		       int num)
{
	struct ec_i2c_device *bus = adap->algo_data;
	struct device *dev = bus->dev;
	const u16 bus_num = bus->remote_bus;
	int request_len;
	int response_len;
	u8 *request = NULL;
	u8 *response = NULL;
	int result;

	request_len = ec_i2c_construct_message(NULL, i2c_msgs, num, bus_num);
	if (request_len < 0) {
		dev_warn(dev, "Error constructing message %d\n", request_len);
		result = request_len;
		goto exit;
	}
	response_len = ec_i2c_parse_response(NULL, i2c_msgs, &num);
	if (response_len < 0) {
		/* Unexpected; no errors should come when NULL response */
		dev_warn(dev, "Error preparing response %d\n", response_len);
		result = response_len;
		goto exit;
	}

	if (request_len <= ARRAY_SIZE(bus->request_buf)) {
		request = bus->request_buf;
	} else {
		request = kzalloc(request_len, GFP_KERNEL);
		if (request == NULL) {
			result = -ENOMEM;
			goto exit;
		}
	}
	if (response_len <= ARRAY_SIZE(bus->response_buf)) {
		response = bus->response_buf;
	} else {
		response = kzalloc(response_len, GFP_KERNEL);
		if (response == NULL) {
			result = -ENOMEM;
			goto exit;
		}
	}

	ec_i2c_construct_message(request, i2c_msgs, num, bus_num);
	result = bus->ec->command_sendrecv(bus->ec, EC_CMD_I2C_PASSTHRU,
					   request, request_len,
					   response, response_len);
	if (result)
		goto exit;

	result = ec_i2c_parse_response(response, i2c_msgs, &num);
	if (result < 0)
		goto exit;

	/* Indicate success by saying how many messages were sent */
	result = num;
exit:
	if (request != bus->request_buf)
		kfree(request);
	if (response != bus->response_buf)
		kfree(response);

	return result;
}

static u32 ec_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm ec_i2c_algorithm = {
	.master_xfer	= ec_i2c_xfer,
	.functionality	= ec_i2c_functionality,
};

static int ec_i2c_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct cros_ec_device *ec = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct ec_i2c_device *bus = NULL;
	u32 remote_bus;
	int err;

	dev_dbg(dev, "Probing\n");

	if (!np) {
		dev_err(dev, "no device node\n");
		return -ENOENT;
	}

	bus = devm_kzalloc(dev, sizeof(*bus), GFP_KERNEL);
	if (bus == NULL) {
		dev_err(dev, "cannot allocate bus device\n");
		return -ENOMEM;
	}

	err = of_property_read_u32(np, "google,remote-bus", &remote_bus);
	if (err) {
		dev_err(dev, "Couldn't read remote-bus property\n");
		return err;
	}
	bus->remote_bus = remote_bus;

	bus->ec = ec;
	bus->dev = dev;

	bus->adap.owner = THIS_MODULE;
	strlcpy(bus->adap.name, "cros-ec-i2c-tunnel", sizeof(bus->adap.name));
	bus->adap.algo = &ec_i2c_algorithm;
	bus->adap.algo_data = bus;
	bus->adap.dev.parent = &pdev->dev;
	bus->adap.dev.of_node = np;

	err = i2c_add_adapter(&bus->adap);
	if (err) {
		dev_err(dev, "cannot register i2c adapter\n");
		return err;
	}
	platform_set_drvdata(pdev, bus);

	dev_dbg(dev, "ChromeOS EC I2C tunnel adapter\n");

	of_i2c_register_devices(&bus->adap);

	return err;
}

static int ec_i2c_remove(struct platform_device *dev)
{
	struct ec_i2c_device *bus = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	i2c_del_adapter(&bus->adap);

	return 0;
}

static struct platform_driver ec_i2c_tunnel_driver = {
	.probe = ec_i2c_probe,
	.remove = ec_i2c_remove,
	.driver = {
		.name = "cros-ec-i2c-tunnel",
	},
};

static int __init ec_i2c_init(void)
{
	return platform_driver_register(&ec_i2c_tunnel_driver);
}
subsys_initcall(ec_i2c_init);

static void __exit ec_i2c_exit(void)
{
	platform_driver_unregister(&ec_i2c_tunnel_driver);
}
module_exit(ec_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("EC I2C tunnel driver");
MODULE_ALIAS("platform:cros-ec-i2c-tunnel");
