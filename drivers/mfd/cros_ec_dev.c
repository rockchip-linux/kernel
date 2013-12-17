/*
 * cros_ec_dev - expose the Chrome OS Embedded Controller to userspace
 *
 * Copyright (C) 2013 The Chromium OS Authors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#define pr_fmt(fmt) "cros_ec_dev: " fmt

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/mfd/cros_ec_dev.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "cros_ec_dev.h"

/* Device variables */
#define CROS_CLASS_NAME "chromeos"
static struct cros_ec_device *ec;
static struct class *cros_class;
static int ec_major;

/* Basic communication */
static int ec_get_version(struct cros_ec_device *ec, char *str, int maxlen)
{
	struct ec_response_get_version resp;
	static const char * const current_image_name[] = {
		"unknown", "read-only", "read-write", "invalid",
	};
	struct cros_ec_command msg = {
		.version = 0,
		.command = EC_CMD_GET_VERSION,
		.outdata = NULL,
		.outsize = 0,
		.indata = (uint8_t *)&resp,
		.insize = sizeof(resp),
	};
	int ret;

	ret = cros_ec_cmd_xfer(ec, &msg);
	if (ret < 0)
		return ret;
	if (msg.result != EC_RES_SUCCESS) {
		snprintf(str, maxlen,
			 "%s\nUnknown EC version: EC returned %d\n",
			 CROS_EC_DEV_VERSION, msg.result);
		return 0;
	}
	if (resp.current_image >= ARRAY_SIZE(current_image_name))
		resp.current_image = 3; /* invalid */
	snprintf(str, maxlen, "%s\n%s\n%s\n\%s\n", CROS_EC_DEV_VERSION,
		 resp.version_string_ro, resp.version_string_rw,
		 current_image_name[resp.current_image]);

	return 0;
}

/* Device file ops */
static int ec_device_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ec_device_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t ec_device_read(struct file *filp, char __user *buffer,
			      size_t length, loff_t *offset)
{
	char msg[sizeof(struct ec_response_get_version) +
		 sizeof(CROS_EC_DEV_VERSION)];
	size_t count;
	int ret;

	if (*offset != 0)
		return 0;

	ret = ec_get_version(ec, msg, sizeof(msg));
	if (ret)
		return ret;
	count = min(length, strlen(msg));

	if (copy_to_user(buffer, msg, count))
		return -EFAULT;

	*offset += count;
	return count;
}


/* Ioctls */
static long ec_device_ioctl_xcmd(void __user *argp)
{
	long ret;
	struct cros_ec_command s_cmd;
	uint8_t *user_indata;
	uint8_t buf[EC_PROTO2_MAX_PARAM_SIZE]; /* FIXME: crosbug.com/p/20820 */

	if (copy_from_user(&s_cmd, argp, sizeof(s_cmd)))
		return -EFAULT;
	if (s_cmd.outsize &&
	    copy_from_user(&buf, (void __user *)s_cmd.outdata, sizeof(buf)))
		return -EFAULT;

	user_indata = s_cmd.indata;
	s_cmd.indata = buf;
	s_cmd.outdata = buf;
	ret = cros_ec_cmd_xfer(ec, &s_cmd);
	s_cmd.indata = user_indata;

	/* Only copy data to userland if data was received. */
	if (ret > 0 && s_cmd.insize) {
		unsigned size = ret;

		size = min(size, s_cmd.insize);
		if (copy_to_user((void __user *)s_cmd.indata, buf, size))
			return -EFAULT;
	}
	if (copy_to_user(argp, &s_cmd, sizeof(s_cmd)))
		return -EFAULT;

	return ret;
}

static long ec_device_ioctl_readmem(void __user *argp)
{
	struct cros_ec_readmem s_mem;
	char buf[EC_MEMMAP_SIZE];
	long num;

	/* Not every platform supports direct reads */
	if (!ec->cmd_readmem)
		return -ENOTTY;

	if (copy_from_user(&s_mem, argp, sizeof(s_mem)))
		return -EFAULT;
	num = ec->cmd_readmem(ec, s_mem.offset, s_mem.bytes, buf);
	if (num <= 0)
		return num;
	if (copy_to_user((void __user *)s_mem.buffer, buf, num))
		return -EFAULT;
	return num;
}

static long ec_device_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	if (_IOC_TYPE(cmd) != CROS_EC_DEV_IOC)
		return -ENOTTY;

	switch (cmd) {
	case CROS_EC_DEV_IOCXCMD:
		return ec_device_ioctl_xcmd(argp);
		break;
	case CROS_EC_DEV_IOCRDMEM:
		return ec_device_ioctl_readmem(argp);
		break;
	}

	return -ENOTTY;
}

/* FIXME: Handle 32-bit apps on 64-bit kernels */
static long ec_device_compat_ioctl(struct file *filp, unsigned int cmd,
				   unsigned long arg)
{
	return -ENOTTY;
}

/* Module initialization */
static const struct file_operations fops = {
	.open = ec_device_open,
	.release = ec_device_release,
	.read = ec_device_read,
	.unlocked_ioctl = ec_device_ioctl,
	.compat_ioctl = ec_device_compat_ioctl,
};

static int ec_device_probe(struct platform_device *pdev)
{
	int retval = -ENOTTY;

	ec = dev_get_drvdata(pdev->dev.parent);

	cros_class = class_create(THIS_MODULE, CROS_CLASS_NAME);
	if (IS_ERR(cros_class)) {
		pr_err("failed to register device class\n");
		retval = PTR_ERR(cros_class);
		goto failed_class;
	}

	/* Let the kernel pick the major num for us */
	ec_major = register_chrdev(0, CROS_EC_DEV_NAME, &fops);
	if (ec_major < 0) {
		pr_err("failed to register device\n");
		retval = ec_major;
		goto failed_chrdevreg;
	}

	/* Instantiate it (and remember the EC) */
	ec->vdev = device_create(cros_class, NULL, MKDEV(ec_major, 0),
				 ec, CROS_EC_DEV_NAME);
	if (IS_ERR(ec->vdev)) {
		pr_err("failed to create device\n");
		retval = PTR_ERR(ec->vdev);
		goto failed_devreg;
	}

	/* Initialize extra interfaces */
	ec_dev_sysfs_init(ec);

	return 0;

failed_devreg:
	unregister_chrdev(ec_major, CROS_EC_DEV_NAME);
failed_chrdevreg:
	class_destroy(cros_class);
failed_class:
	return retval;
}

static int ec_device_remove(struct platform_device *pdev)
{
	ec_dev_sysfs_remove(ec);
	device_destroy(cros_class, MKDEV(ec_major, 0));
	unregister_chrdev(ec_major, CROS_EC_DEV_NAME);
	class_destroy(cros_class);
	return 0;
}

static struct platform_driver cros_ec_dev_driver = {
	.driver = {
		.name = "cros-ec-dev",
		.owner = THIS_MODULE,
	},
	.probe = ec_device_probe,
	.remove = ec_device_remove,
};

module_platform_driver(cros_ec_dev_driver);

MODULE_AUTHOR("Bill Richardson <wfrichar@chromium.org>");
MODULE_DESCRIPTION("Userspace interface to the Chrome OS Embedded Controller");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
