/*
 *  Copyright (C) 2012 The Chromium OS Authors
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
 */

#define pr_fmt(fmt) "chromeos_vbc_ec: " fmt

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/mfd/cros_ec.h>

#include "chromeos.h"

static phandle ec_phandle;

static int match_of_node(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static struct device *get_ec_dev(phandle phandle)
{
	struct device_node *dn;
	struct device *dev;

	if (!phandle)
		return ERR_PTR(-ENODEV);

	dn = of_find_node_by_phandle(phandle);
	if (!dn)
		return ERR_PTR(-ENODEV);

	dev = bus_find_device(&platform_bus_type, NULL, dn, match_of_node);
	of_node_put(dn);
	if (!dev)
		return ERR_PTR(-ENODEV);

	return dev;
}

static int vbc_read(struct device *dev, void *buf, size_t count)
{
	struct cros_ec_device *ec;
	struct ec_params_vbnvcontext param;
	struct ec_response_vbnvcontext resp;
	struct cros_ec_command msg = {
		.version = EC_VER_VBNV_CONTEXT,
		.command = EC_CMD_VBNV_CONTEXT,
		.outdata = (uint8_t *)&param,
		.outsize = sizeof(param.op),
		.indata = (uint8_t *)&resp,
		.insize = sizeof(resp),
	};
	int err;

	ec = dev_get_drvdata(dev->parent);

	param.op = EC_VBNV_CONTEXT_OP_READ;
	err = cros_ec_cmd_xfer(ec, &msg);
	if (err < 0)
		return err;
	/* FIXME: This assumes msg.result == EC_RES_SUCCESS */
	count = min(count, sizeof(resp.block));
	memcpy(buf, resp.block, count);

	return count;
}

static int vbc_write(struct device *dev, const void *buf, size_t count)
{
	struct cros_ec_device *ec;
	struct ec_params_vbnvcontext param;
	struct cros_ec_command msg = {
		.version = EC_VER_VBNV_CONTEXT,
		.command = EC_CMD_VBNV_CONTEXT,
		.outdata = (uint8_t *)&param,
		.outsize = sizeof(param),
		.indata = NULL,
		.insize = 0,
	};
	int err;

	ec = dev_get_drvdata(dev->parent);

	count = min(count, sizeof(param.block));

	param.op = EC_VBNV_CONTEXT_OP_WRITE;
	memcpy(param.block, buf, count);
	err = cros_ec_cmd_xfer(ec, &msg);
	if (err < 0)
		return err;
	/* FIXME: This assumes msg.result == EC_RES_SUCCESS */
	return count;
}

static ssize_t dev_attr_vbc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ec_params_vbnvcontext param;
	int i, j, count;

	count = vbc_read(dev, param.block, sizeof(param.block));
	if (count < 0)
		return count;

	for (i = j = 0; i < count && j < PAGE_SIZE - 2; i++) {
		buf[j++] = hex_asc_hi(param.block[i]);
		buf[j++] = hex_asc_lo(param.block[i]);
	}
	buf[j++] = '\0';

	return j;
}

static ssize_t dev_attr_vbc_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ec_params_vbnvcontext param;
	int err;

	memset(param.block, 0x0, sizeof(param.block));
	err = hex2bin(param.block, buf, min(count / 2, sizeof(param.block)));
	if (err < 0)
		return err;

	err = vbc_write(dev, param.block, sizeof(param.block));
	if (err < 0)
		return err;

	return count;
}

static DEVICE_ATTR(vboot_context, S_IRUSR | S_IWUSR,
		dev_attr_vbc_show, dev_attr_vbc_store);

static struct attribute *vbc_attrs[] = {
	&dev_attr_vboot_context.attr,
	NULL
};

static const struct attribute_group vbc_attr_group = {
	.attrs = vbc_attrs,
};

static int chromeos_vbc_ec_read(void *buf, size_t count)
{
	struct device *dev;
	int err;

	dev = get_ec_dev(ec_phandle);
	if (IS_ERR(dev))
		return PTR_ERR(dev);

	err = vbc_read(dev, buf, count);
	put_device(dev);

	return err;
}

static int chromeos_vbc_ec_write(const void *buf, size_t count)
{
	struct device *dev;
	int err;

	dev = get_ec_dev(ec_phandle);
	if (IS_ERR(dev))
		return PTR_ERR(dev);

	err = vbc_write(dev, buf, count);
	put_device(dev);

	return err;
}

static int chromeos_vbc_ec_probe(struct platform_device *pdev)
{
	pdev->dev.of_node = of_find_node_by_phandle(ec_phandle);
	return 0;
}

static struct chromeos_vbc chromeos_vbc_ec = {
	.name = "cros-ec-vbc",
	.read = chromeos_vbc_ec_read,
	.write = chromeos_vbc_ec_write,
};

static struct platform_driver chromeos_vbc_ec_driver = {
	.probe = chromeos_vbc_ec_probe,
	.driver = {
		.name = "cros-ec-vbc",
	},
};

static int __init chromeos_vbc_ec_init(void)
{
	struct device_node *of_node;
	struct device *dev;
	const char *vbc_type;
	int err;

	of_node = of_find_compatible_node(NULL, NULL, "chromeos-firmware");
	if (!of_node)
		return -ENODEV;

	err = of_property_read_string(of_node, "nonvolatile-context-storage",
			&vbc_type);
	if (err)
		goto exit;

	if (strcmp(vbc_type, "mkbp")) {
		err = 0;  /* not configured to use vbc_ec, exit normally. */
		goto exit;
	}

	err = of_property_read_u32(of_node, "chromeos-vbc-ec", &ec_phandle);
	if (err) {
		pr_err("Could not find chromeos-vbc-ec: err=%d\n", err);
		goto exit;
	}

	err = chromeos_vbc_register(&chromeos_vbc_ec);
	if (err < 0)
		goto exit;

	err = platform_driver_register(&chromeos_vbc_ec_driver);
	if (err < 0)
		goto exit;

	/* Create a sysfs entry for manual debugging/testing. */
	dev = get_ec_dev(ec_phandle);
	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		pr_err("Could not find ec device: err=%d\n", err);
		goto exit;
	}
	err = sysfs_create_group(&dev->kobj, &vbc_attr_group);
	put_device(dev);
	if (err) {
		pr_err("Could not create sysfs entries: err=%d\n", err);
		goto exit;
	}

	err = 0;
exit:
	of_node_put(of_node);
	return err;
}
module_init(chromeos_vbc_ec_init);

static void __exit chromeos_vbc_ec_exit(void)
{
	struct device *dev = get_ec_dev(ec_phandle);

	if (IS_ERR(dev)) {
		pr_warn("Could not find ec device: err=%ld\n", PTR_ERR(dev));
	} else {
		sysfs_remove_group(&dev->kobj, &vbc_attr_group);
		put_device(dev);
	}

	platform_driver_unregister(&chromeos_vbc_ec_driver);
}
module_exit(chromeos_vbc_ec_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ChromeOS vboot context on EC accessor");
