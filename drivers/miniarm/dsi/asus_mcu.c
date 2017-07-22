/*
 *
 * RPI Touchscreen MCU driver.
 *
 * Copyright (c) 2016 ASUSTek Computer Inc.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include "asus_mcu.h"

struct ts_mcu_data *mdata;

static int is_hex(char num)
{
	//0-9, a-f, A-F
	if ((47 < num && num < 58) || (64 < num && num < 71) || (96 < num && num < 103))
		return 1;
	return 0;
}

static int string_to_byte(const char *source, unsigned char *destination, int size)
{
	int i = 0, counter = 0;
	char c[3] = {0};
	unsigned char bytes;

	if (size%2 == 1)
		return -EINVAL;

	for(i = 0; i < size; i++){
		if(!is_hex(source[i])) {
			return -EINVAL;
		}
		if(0 == i%2){
			c[0] = source[i];
			c[1] = source[i+1];
			sscanf(c, "%hhx", &bytes);
			destination[counter] = bytes;
			counter++;
		}
	}
	return 0;
}

static int send_cmds(struct i2c_client *client, const char *buf)
{
	int ret, size = strlen(buf);
	unsigned char byte_cmd[size/2];

	if ((size%2) != 0) {
		LOG_ERR("size should be even\n");
		return -EINVAL;
	}

	LOG_INFO("%s\n", buf);

	string_to_byte(buf, byte_cmd, size);

	ret = i2c_master_send(client, byte_cmd, size/2);
	if (ret < 0) {
		LOG_ERR("send command failed, ret = %d\n", ret);
		return ret;
	}

	msleep(20);
	return 0;
}

static int recv_cmds(struct i2c_client *client, char *buf, int size)
{
	int ret;

	ret = i2c_master_recv(client, buf, size);
	if (ret < 0) {
		LOG_ERR("receive commands failed, %d\n", ret);
		return ret;
	}
	msleep(20);
	return 0;
}

static int init_cmd_check(struct ts_mcu_data *mcu_data)
{
	int ret;
	char recv_buf[1] = {0};

	ret = send_cmds(mcu_data->client, "80");
	if (ret < 0)
		goto error;

	recv_cmds(mcu_data->client, recv_buf, 1);
	if (ret < 0)
		goto error;

	LOG_INFO("recv_cmds: 0x%X\n", recv_buf[0]);
	if (recv_buf[0] != 0xC3) {
		LOG_ERR("read wrong info\n");
		ret = -EINVAL;
		goto error;

	}
	return 0;

error:
	return ret;
}

void mcu_power_up(void)
{
	send_cmds(mdata->client, "8500");
	msleep(800);
	send_cmds(mdata->client, "8501");
	send_cmds(mdata->client, "86FF");
	send_cmds(mdata->client, "8104");
}
EXPORT_SYMBOL_GPL(mcu_power_up);

static int rpi_ts_mcu_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;

	LOG_INFO("address = 0x%x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		LOG_ERR("I2C check functionality failed\n");
		return -ENODEV;
	}

	mdata = kzalloc(sizeof(struct ts_mcu_data), GFP_KERNEL);
	if (mdata == NULL) {
		LOG_ERR("no memory for device\n");
		return -ENOMEM;
	}

	mdata->client = client;
	i2c_set_clientdata(client, mdata);

	ret = init_cmd_check(mdata);
	if (ret < 0) {
		LOG_ERR("init_cmd_check failed, %d\n", ret);
		goto error;
	}

	return 0;

error:
	kfree(mdata);
	return ret;
}

static int rpi_ts_mcu_remove(struct i2c_client *client)
{
	struct ts_mcu_data *mcu_data = i2c_get_clientdata(client);

	kfree(mcu_data);
	return 0;
}

static const struct i2c_device_id rpi_ts_mcu_id[] = {
	{"rpi_ts_mcu", 0},
	{},
};

static struct i2c_driver rpi_ts_mcu_driver = {
	.driver = {
		.name = "rpi_ts_mcu",
	},
	.probe = rpi_ts_mcu_probe,
	.remove = rpi_ts_mcu_remove,
	.id_table = rpi_ts_mcu_id,
};
module_i2c_driver(rpi_ts_mcu_driver);

MODULE_DESCRIPTION("RPI TouchScreen MCU driver");
MODULE_LICENSE("GPL v2");
