/*
 * Copyright (C) Guangzhou FriendlyELEC Computer Tech. Co., Ltd.
 * (http://www.friendlyarm.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/crc32.h>
#include <linux/nvmem-consumer.h>

#define BOARD_MANF "FriendlyELEC Computer Tech. Co., Ltd."

static const char *board_mach;
static const char *board_name;
static u32 board_rev;
static u32 board_serial_high, board_serial_low;

static ssize_t board_sys_info_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	char *s = buf;

	s += sprintf(s, "Hardware\t: %s\n", board_mach);
	s += sprintf(s, "Revision\t: %04x\n", board_rev);
	s += sprintf(s, "Serial\t\t: %08x%08x\n",
			board_serial_high, board_serial_low);
	s += sprintf(s, "\nModel\t\t: %s\n", board_name);
	s += sprintf(s, "Manufacturer\t: %s\n", BOARD_MANF);

	return (s - buf);
}

static struct device_attribute board_attr_info =
	__ATTR(info, S_IRUGO, board_sys_info_show, NULL);

static int rockchip_cpuinfo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvmem_cell *cell;
	unsigned char *efuse_buf, buf[16];
	size_t len;
	int i;

	cell = nvmem_cell_get(dev, "id");
	if (IS_ERR(cell)) {
		dev_err(dev, "failed to get id cell: %ld\n", PTR_ERR(cell));
		return PTR_ERR(cell);
	}

	efuse_buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);

	if (len != 16) {
		kfree(efuse_buf);
		dev_err(dev, "invalid id len: %zu\n", len);
		return -EINVAL;
	}

	for (i = 0; i < 8; i++) {
		buf[i] = efuse_buf[1 + (i << 1)];
		buf[i + 8] = efuse_buf[i << 1];
	}

	kfree(efuse_buf);

	board_serial_low = crc32(0, buf, 8);
	board_serial_high = crc32(board_serial_low, buf + 8, 8);

	dev_info(dev, "Serial\t\t: %08x%08x\n",
		 board_serial_high, board_serial_low);

	return 0;
}

static int board_sys_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *root;

	root = of_find_node_by_path("/");

	of_property_read_u32(np, "hwrev", &board_rev);

	if (of_property_read_string(np, "machine", &board_mach))
		of_property_read_string(root, "compatible", &board_mach);

	if (of_property_read_string(np, "model", &board_name))
		of_property_read_string(root, "model", &board_name);

	of_node_put(root);

	rockchip_cpuinfo_probe(pdev);

	device_create_file(&pdev->dev, &board_attr_info);

	return 0;
}

static const struct of_device_id board_sys_of_match[] = {
	{ .compatible = "friendlyelec,board" },
	{}
};
MODULE_DEVICE_TABLE(of, board_sys_of_match);

static struct platform_driver board_sys_driver = {
	.probe = board_sys_probe,
	.driver = {
		.name = "friendlyelec-board",
		.of_match_table = board_sys_of_match,
	},
};

static int __init board_sys_init(void)
{
	return platform_driver_register(&board_sys_driver);
}
late_initcall(board_sys_init);

MODULE_AUTHOR("support@friendlyarm.com");
MODULE_DESCRIPTION("FriendlyElec NanoPi Series Machine Driver");
MODULE_LICENSE("GPL v2");
