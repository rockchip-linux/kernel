/*
 * cros_ec_lpc - LPC access to the Chrome OS Embedded Controller
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
 * This driver uses the Chrome OS EC byte-level message-based protocol for
 * communicating the keyboard state (which keys are pressed) from a keyboard EC
 * to the AP over some bus (such as i2c, lpc, spi).  The EC does debouncing,
 * but everything else (including deghosting) is done here.  The main
 * motivation for this is to keep the EC firmware as simple as possible, since
 * it cannot be easily upgraded and EC flash/IRAM space is relatively
 * expensive.
 */

#include <linux/delay.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>

#define MYNAME "cros_ec_lpc"

/**
 * cros_ec_lpc_read_bytes - Read bytes from a given LPC-mapped address.
 * Returns 8-bit checksum of all bytes read.
 *
 * @offset: Base read address
 * @length: Number of bytes to read
 * @dest: Destination buffer
 */
static u8 cros_ec_lpc_read_bytes(unsigned int offset,
				 unsigned int length,
				 u8 *dest)
{
	int i;
	int sum = 0;

	for (i = 0; i < length; ++i) {
		dest[i] = inb(offset + i);
		sum += dest[i];
	}

	/* Return checksum of all bytes read */
	return sum;
}

/**
 * cros_ec_lpc_write_bytes - Write bytes to a given LPC-mapped address.
 * Returns 8-bit checksum of all bytes written.
 *
 * @offset: Base write address
 * @length: Number of bytes to write
 * @msg: Write data buffer
 */
static u8 cros_ec_lpc_write_bytes(unsigned int offset,
				  unsigned int length,
				  u8 *msg)
{
	int i;
	int sum = 0;

	for (i = 0; i < length; ++i) {
		outb(msg[i], offset + i);
		sum += msg[i];
	}

	/* Return checksum of all bytes written */
	return sum;
}

static int ec_response_timed_out(void)
{
	unsigned long one_second = jiffies + HZ;
	u8 data;

	usleep_range(200, 300);
	do {
		if (!(cros_ec_lpc_read_bytes(EC_LPC_ADDR_HOST_CMD, 1, &data) &
		    EC_LPC_STATUS_BUSY_MASK))
			return 0;
		usleep_range(100, 200);
	} while (time_before(jiffies, one_second));

	return 1;
}

static int cros_ec_pkt_xfer_lpc(struct cros_ec_device *ec,
				struct cros_ec_command *msg)
{
	struct ec_host_request *request;
	struct ec_host_response response;
	u8 sum;
	int ret = 0;
	u8 *dout;

	ret = cros_ec_prepare_tx(ec, msg);

	/* Write buffer */
	cros_ec_lpc_write_bytes(EC_LPC_ADDR_HOST_PACKET, ret, ec->dout);

	request = (struct ec_host_request *)ec->dout;

	/* Here we go */
	sum = EC_COMMAND_PROTOCOL_3;
	cros_ec_lpc_write_bytes(EC_LPC_ADDR_HOST_CMD, 1, &sum);

	if (ec_response_timed_out()) {
		dev_warn(ec->dev, "EC responsed timed out\n");
		ret = -EIO;
		goto done;
	}

	/* Check result */
	msg->result = cros_ec_lpc_read_bytes(EC_LPC_ADDR_HOST_DATA, 1, &sum);
	switch (msg->result) {
	case EC_RES_SUCCESS:
		break;
	case EC_RES_IN_PROGRESS:
		ret = -EAGAIN;
		dev_dbg(ec->dev, "command 0x%02x in progress\n",
			msg->command);
		goto done;
	default:
		dev_dbg(ec->dev, "command 0x%02x returned %d\n",
			msg->command, msg->result);
	}

	/* Read back response */
	dout = (u8 *)&response;
	sum = cros_ec_lpc_read_bytes(
		EC_LPC_ADDR_HOST_PACKET, sizeof(response), dout);

	msg->result = response.result;

	if (response.data_len > msg->insize) {
		dev_err(ec->dev,
			"packet too long (%d bytes, expected %d)",
			response.data_len, msg->insize);
		ret = -EMSGSIZE;
		goto done;
	}

	/* Read response and process checksum */
	sum += cros_ec_lpc_read_bytes(
			EC_LPC_ADDR_HOST_PACKET + sizeof(response),
			response.data_len,
			msg->indata);

	if (sum) {
		dev_err(ec->dev,
			"bad packet checksum %02x\n",
			response.checksum);
		ret = -EBADMSG;
		goto done;
	}

	/* Return actual amount of data received */
	ret = response.data_len;
done:
	return ret;
}

static int cros_ec_cmd_xfer_lpc(struct cros_ec_device *ec,
				struct cros_ec_command *msg)
{
	struct ec_lpc_host_args args;
	u8 sum;
	int ret = 0;

	if (msg->outsize > EC_PROTO2_MAX_PARAM_SIZE ||
	    msg->insize > EC_PROTO2_MAX_PARAM_SIZE) {
		dev_err(ec->dev,
			"invalid buffer sizes (out %d, in %d)\n",
			msg->outsize, msg->insize);
		return -EINVAL;
	}

	/* Now actually send the command to the EC and get the result */
	args.flags = EC_HOST_ARGS_FLAG_FROM_HOST;
	args.command_version = msg->version;
	args.data_size = msg->outsize;

	/* Initialize checksum */
	sum = msg->command + args.flags +
		args.command_version + args.data_size;

	/* Copy data and update checksum */
	sum += cros_ec_lpc_write_bytes(
		EC_LPC_ADDR_HOST_PARAM, msg->outsize, msg->outdata);

	/* Finalize checksum and write args */
	args.checksum = sum;
	cros_ec_lpc_write_bytes(
		EC_LPC_ADDR_HOST_ARGS, sizeof(args), (u8 *)&args);

	/* Here we go */
	sum = msg->command;
	cros_ec_lpc_write_bytes(EC_LPC_ADDR_HOST_CMD, 1, &sum);

	if (ec_response_timed_out()) {
		dev_warn(ec->dev, "EC responsed timed out\n");
		ret = -EIO;
		goto done;
	}

	/* Check result */
	msg->result = cros_ec_lpc_read_bytes(EC_LPC_ADDR_HOST_DATA, 1, &sum);
	switch (msg->result) {
	case EC_RES_SUCCESS:
		break;
	case EC_RES_IN_PROGRESS:
		ret = -EAGAIN;
		dev_dbg(ec->dev, "command 0x%02x in progress\n",
			msg->command);
		goto done;
	default:
		dev_dbg(ec->dev, "command 0x%02x returned %d\n",
			msg->command, msg->result);
	}

	/* Read back args */
	cros_ec_lpc_read_bytes(
		EC_LPC_ADDR_HOST_ARGS, sizeof(args), (u8 *)&args);

	if (args.data_size > msg->insize) {
		dev_err(ec->dev,
			"packet too long (%d bytes, expected %d)",
			args.data_size, msg->insize);
		ret = -ENOSPC;
		goto done;
	}

	/* Start calculating response checksum */
	sum = msg->command + args.flags +
		args.command_version + args.data_size;

	/* Read response and update checksum */
	sum += cros_ec_lpc_read_bytes(
		EC_LPC_ADDR_HOST_PARAM, args.data_size, msg->indata);

	/* Verify checksum */
	if (args.checksum != sum) {
		dev_err(ec->dev,
			"bad packet checksum, expected %02x, got %02x\n",
			args.checksum, sum);
		ret = -EBADMSG;
		goto done;
	}

	/* Return actual amount of data received */
	ret = args.data_size;
done:
	return ret;
}

/* Returns num bytes read, or negative on error. Doesn't need locking. */
static int cros_ec_lpc_readmem(struct cros_ec_device *ec, unsigned int offset,
			       unsigned int bytes, void *dest)
{
	int i = offset;
	char *s = dest;
	int cnt = 0;

	if (offset >= EC_MEMMAP_SIZE - bytes)
		return -EINVAL;

	/* fixed length */
	if (bytes) {
		cros_ec_lpc_read_bytes(EC_LPC_ADDR_MEMMAP + offset, bytes, s);
		return bytes;
	}

	/* string */
	for (; i < EC_MEMMAP_SIZE; i++, s++) {
		cros_ec_lpc_read_bytes(EC_LPC_ADDR_MEMMAP + i, 1, s);
		cnt++;
		if (!*s)
			break;
	}

	return cnt;
}

static int cros_ec_lpc_read_u32(struct cros_ec_device *ec,
				unsigned int offset,
				u32 *dest)
{
	u32 tmp;
	int ret = cros_ec_lpc_readmem(ec, offset, 4, &tmp);

	*dest = le32_to_cpu(tmp);

	return ret;
}

static int cros_ec_lpc_read_u16(struct cros_ec_device *ec,
				unsigned int offset,
				u16 *dest)
{
	u16 tmp;
	int ret = cros_ec_lpc_readmem(ec, offset, 2, &tmp);

	*dest = le16_to_cpu(tmp);

	return ret;
}

static int cros_ec_lpc_read_u8(struct cros_ec_device *ec,
			       unsigned int offset,
			       u8 *dest)
{
	return cros_ec_lpc_readmem(ec, offset, 1, dest);
}

static int cros_ec_lpc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cros_ec_device *ec_dev;
	int err = -ENOTTY;
	u8 buf[2];

	if (!request_region(EC_LPC_ADDR_MEMMAP, EC_MEMMAP_SIZE, MYNAME)) {
		dev_warn(dev, "couldn't reserve memmap region\n");
		goto failed_memmap;
	}

	cros_ec_lpc_read_bytes(EC_LPC_ADDR_MEMMAP + EC_MEMMAP_ID, 2, buf);
	if (buf[0] != 'E' || buf[1] != 'C') {
		dev_warn(dev, "EC ID not detected\n");
		goto failed_ec_probe;
	}

	if (!request_region(EC_HOST_CMD_REGION0, EC_HOST_CMD_REGION_SIZE,
			    MYNAME)) {
		dev_warn(dev, "couldn't reserve region0\n");
		goto failed_region0;
	}
	if (!request_region(EC_HOST_CMD_REGION1, EC_HOST_CMD_REGION_SIZE,
			    MYNAME)) {
		dev_warn(dev, "couldn't reserve region1\n");
		goto failed_region1;
	}

	ec_dev = devm_kzalloc(dev, sizeof(*ec_dev), GFP_KERNEL);
	if (!ec_dev) {
		err = -ENOMEM;
		goto failed_ec_dev;
	}

	platform_set_drvdata(pdev, ec_dev);
	ec_dev->dev = dev;
	ec_dev->phys_name = dev_name(dev);
	ec_dev->cmd_xfer = cros_ec_cmd_xfer_lpc;
	ec_dev->pkt_xfer = cros_ec_pkt_xfer_lpc;
	ec_dev->cmd_readmem = cros_ec_lpc_readmem;
	ec_dev->cmd_read_u32 = cros_ec_lpc_read_u32;
	ec_dev->cmd_read_u16 = cros_ec_lpc_read_u16;
	ec_dev->cmd_read_u8 = cros_ec_lpc_read_u8;
	ec_dev->din_size = sizeof(struct ec_host_response) +
			   sizeof(struct ec_response_get_protocol_info);
	ec_dev->dout_size = sizeof(struct ec_host_request);

	err = cros_ec_register(ec_dev);
	if (err) {
		dev_warn(dev, "couldn't register ec_dev\n");
		goto failed_ec_dev;
	}

	return 0;

failed_ec_dev:
	release_region(EC_HOST_CMD_REGION1, EC_HOST_CMD_REGION_SIZE);
failed_region1:
	release_region(EC_HOST_CMD_REGION0, EC_HOST_CMD_REGION_SIZE);
failed_region0:
failed_ec_probe:
	release_region(EC_LPC_ADDR_MEMMAP, EC_MEMMAP_SIZE);
failed_memmap:
	return err;
}

static int cros_ec_lpc_remove(struct platform_device *pdev)
{
	struct cros_ec_device *ec_dev;

	ec_dev = platform_get_drvdata(pdev);
	cros_ec_remove(ec_dev);

	release_region(EC_HOST_CMD_REGION1, EC_HOST_CMD_REGION_SIZE);
	release_region(EC_HOST_CMD_REGION0, EC_HOST_CMD_REGION_SIZE);
	release_region(EC_LPC_ADDR_MEMMAP, EC_MEMMAP_SIZE);

	return 0;
}

static struct platform_driver cros_ec_lpc_driver = {
	.driver = {
		.name = MYNAME,
		.owner = THIS_MODULE,
	},
	.probe = cros_ec_lpc_probe,
	.remove = cros_ec_lpc_remove,
};

static void do_nothing(struct device *dev)
{
	/* not a physical device */
}

static struct platform_device cros_ec_lpc_device = {
	.name = MYNAME,
	.dev = {
		.release = do_nothing,
	},
};

static int __init cros_ec_lpc_init(void)
{
	int ret;

	/* Register the driver */
	ret = platform_driver_register(&cros_ec_lpc_driver);
	if (ret < 0) {
		pr_warn(MYNAME ": can't register driver: %d\n", ret);
		return ret;
	}

	/* Register the device, and it'll get hooked up automaticaly */
	ret = platform_device_register(&cros_ec_lpc_device);
	if (ret < 0) {
		pr_warn(MYNAME ": can't register device: %d\n", ret);
		platform_driver_unregister(&cros_ec_lpc_driver);
		return ret;
	}

	return 0;
}

static void __exit cros_ec_lpc_exit(void)
{
	platform_device_unregister(&cros_ec_lpc_device);
	platform_driver_unregister(&cros_ec_lpc_driver);
}

module_init(cros_ec_lpc_init);
module_exit(cros_ec_lpc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ChromeOS EC LPC driver");
