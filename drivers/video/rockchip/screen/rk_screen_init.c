/*
 * drivers/video/rockchip/screen/rk_screen_init.c
 *
 * Copyright (C) 2017 ROCKCHIP, Inc.
 * Author: hjc <hjc@rock-chips.com>
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/rk_fb.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/device.h>

struct screen_init_cmd {
	u8 cmd;
	u8 *value;
	u32 value_size;
	int delay;
};

struct screen_init_cmd_list {
	struct list_head list;
	struct screen_init_cmd cmd;
};

struct _spi_data {
	struct rk_screen *screen;
	int spi_cs;
	int spi_sdi;
	int spi_scl;
	int spi_rst;
	u8  cmd_type;
	struct list_head cmd_list_head;
};

struct _spi_data *spi_data;

static void spi_sentdata(unsigned char value, bool dcx)
{
	int i;

	if (spi_data->cmd_type == SCREEN_INIT_SPI_9BIT) {
		gpio_set_value(spi_data->spi_scl, 0);
		gpio_set_value(spi_data->spi_sdi, dcx);
		udelay(1);
		gpio_set_value(spi_data->spi_scl, 1);
	}

	for (i = 0; i < 8; i++) {
		udelay(1);
		if (value & 0x80)
			gpio_set_value(spi_data->spi_sdi, 1);
		else
			gpio_set_value(spi_data->spi_sdi, 0);

		gpio_set_value(spi_data->spi_scl, 0);
		udelay(1);
		gpio_set_value(spi_data->spi_scl, 1);
		value <<= 1;
	}
}

void spi_write_cmd(unsigned char c, u8 *d, u32 size)
{
	int i;

	gpio_set_value(spi_data->spi_cs, 0);
	spi_sentdata(c, 0);

	for (i = 0; i < size; i++)
		spi_sentdata(*d++, 1);

	gpio_set_value(spi_data->spi_cs, 1);
}

static int spi_screen_init(void)
{
	struct list_head *pos;
	struct screen_init_cmd_list *init_cmd_list;

	gpio_direction_output(spi_data->spi_cs, 0);
	gpio_direction_output(spi_data->spi_rst, 1);
	mdelay(2);
	gpio_direction_output(spi_data->spi_rst, 0);
	mdelay(3);
	gpio_direction_output(spi_data->spi_rst, 1);
	mdelay(3);
	gpio_direction_output(spi_data->spi_cs, 1);
	gpio_direction_output(spi_data->spi_sdi, 1);
	gpio_direction_output(spi_data->spi_scl, 1);
	mdelay(2);

	list_for_each(pos, &spi_data->cmd_list_head) {
		init_cmd_list =
			list_entry(pos, struct screen_init_cmd_list, list);
		spi_write_cmd(init_cmd_list->cmd.cmd, init_cmd_list->cmd.value,
			      init_cmd_list->cmd.value_size);
		mdelay(init_cmd_list->cmd.delay);
	}

	return 0;
}

static int rk_fb_parse_init_cmd(void)
{
	struct device_node *childnode, *root;
	struct list_head *pos;
	struct screen_init_cmd_list *init_cmd_list;
	const struct property *prop;
	const __be32 *val;
	u8 *data;
	int nr;
	int ret = 0;
	u32 value = 0;
	int debug = 0;

	root = of_find_node_by_name(NULL, "screen-init-cmds");

	if (!root) {
		pr_err("can't find screen-init-cmds node\n");
	} else {
		for_each_child_of_node(root, childnode) {
			init_cmd_list =
				kmalloc(sizeof(struct screen_init_cmd_list),
					GFP_KERNEL);
			if (!init_cmd_list) {
				ret = -ENOMEM;
				goto err_free_mem;
			}

			ret = of_property_read_u32(childnode,
						   "rockchip,init-cmd",
						   &value);
			if (ret)
				pr_err("Can't get init cmd: %d\n", ret);
			else
				init_cmd_list->cmd.cmd = value;

			prop = of_find_property(childnode,
						"rockchip,init-cmd-value",
						NULL);
			if (!prop || !prop->value) {
				pr_debug("Can't get init cmd value\n");
				init_cmd_list->cmd.value_size = 0;
			} else {
				nr = prop->length / sizeof(u32);
				init_cmd_list->cmd.value_size = nr;
				data = kmalloc(nr, GFP_KERNEL);
				if (!data) {
					pr_err("Can't kmalloc buf\n");
					ret = -ENOMEM;
					goto err_free_value;
				}

				init_cmd_list->cmd.value = data;

				val = prop->value;
				while (nr--)
					*data++ = be32_to_cpup(val++);
			}

			ret = of_property_read_u32(childnode,
						   "rockchip,init-cmd-delay",
						   &value);
			if (ret)
				pr_err("Can't get init cmd delay: %d\n", ret);
			else
				init_cmd_list->cmd.delay = value;

			list_add_tail(&init_cmd_list->list,
				      &spi_data->cmd_list_head);
		}
	}

	of_property_read_u32(root, "rockchip,cmd_debug", &debug);
	if (debug) {
		list_for_each(pos, &spi_data->cmd_list_head) {
			init_cmd_list = list_entry(pos,
						   struct screen_init_cmd_list,
						   list);
			pr_info("cmd: 0x%x, val size: %d, delay: %d\n",
				init_cmd_list->cmd.cmd,
				init_cmd_list->cmd.value_size,
				init_cmd_list->cmd.delay);
		}
	}

	return  0;

err_free_value:
	kfree(init_cmd_list);
err_free_mem:
	while (!list_empty(&spi_data->cmd_list_head)) {
		init_cmd_list = list_entry(spi_data->cmd_list_head.next,
					   struct screen_init_cmd_list, list);
		list_del(&init_cmd_list->list);
		kfree(init_cmd_list->cmd.value);
		kfree(init_cmd_list);
	}

	return ret;
}

static int rk_fb_parse_screen_init(struct device_node *np,
				   struct rk_screen *screen)
{
	u32 init_type = 0;

	struct device_node *root = NULL;

	root = of_get_child_by_name(np, "screen_init");
	if (!root) {
		pr_err("can't find screen_init node\n");
		screen->init = NULL;
		return -ENODEV;
	}

	of_property_read_u32(root, "screen-init-type", &init_type);
	if ((init_type != SCREEN_INIT_SPI) &&
	    (init_type != SCREEN_INIT_SPI_9BIT)) {
		pr_err("now unsupport screen init type: %d\n", init_type);
		return -1;
	}

	spi_data->cmd_type = init_type;
	spi_data->spi_cs = of_get_named_gpio(root, "cs-gpio", 0);

	if (gpio_request(spi_data->spi_cs, "cs_gpio") != 0) {
		gpio_free(spi_data->spi_cs);
		pr_err("spi_data spi_cs request failed\n");
	}
	spi_data->spi_sdi = of_get_named_gpio(root, "sdi-gpio", 0);
	if (gpio_request(spi_data->spi_sdi, "sdi_gpio") != 0) {
		gpio_free(spi_data->spi_sdi);
		pr_err("spi_data spi_sdi request failed\n");
	}
	spi_data->spi_scl = of_get_named_gpio(root, "scl-gpio", 0);
	if (gpio_request(spi_data->spi_scl, "scl_gpio") != 0) {
		gpio_free(spi_data->spi_scl);
		pr_err("spi_data spi_scl request failed\n");
	}
	spi_data->spi_rst = of_get_named_gpio(root, "rst-gpio", 0);
	if (gpio_request(spi_data->spi_rst, "rst_gpio") != 0) {
		gpio_free(spi_data->spi_rst);
		pr_err("spi_data spi_rst request failed\n");
	}

	rk_fb_parse_init_cmd();

	return 0;
}

int rk_fb_get_screen_init(struct device_node *np, struct rk_screen *screen)
{
	int ret = 0;

	if (unlikely(!screen))
		return -1;

	spi_data = kzalloc(sizeof(*spi_data), GFP_KERNEL);
	if (!spi_data)
		return -ENOMEM;
	INIT_LIST_HEAD(&spi_data->cmd_list_head);
	ret = rk_fb_parse_screen_init(np, screen);

	if (ret) {
		screen->init = NULL;
		kfree(spi_data);
		return ret;
	}
	screen->init = spi_screen_init;
	spi_data->screen = screen;

	return 0;
}
