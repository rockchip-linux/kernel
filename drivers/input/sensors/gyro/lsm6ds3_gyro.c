/* drivers/input/sensors/gyro/lsm6ds3_gyro.c
 *
 * Copyright (C) 2018 Fuzhou Rockchip Electronics Co., Ltd.
 * Author: Dayao Ji<jdy@rock-chips.com>
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
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>

#define LSM6DS3_GYRO_FS_ADDR		0x11
#define LSM6DS3_RESET_ADDR			0x12
#define LSM6DS3_INT2_ON_INT1_ADDR	0x13

#define LSM6DS3_LIR_ADDR			0x58
#define LSM6DS3_FIFO_ODR_ADDR		0x0a
#define LSM6DS3_ROUNDING_ADDR		0x16

#define LSM6DS3_GYRO_XOUT_H			0x22
#define LSM6DS3_WHOAMI				0x0F
#define LSM6DS3_DEVICE_ID			0x69
#define LSM6DS3_PRECISION			16
#define LSM6DS3_PWR_ACCEL			0x10
#define LSM6DS3_PWR_GYRO			0x11
#define LSM6DS3_INT1_CTRL			0x0D

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);
	int result = 0;

	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	if (!enable)
		sensor->ops->ctrl_data &= 0x0F;	/*power down*/
	else
		sensor->ops->ctrl_data |= 0x40;	/*104HZ*/
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if (result)
		printk(KERN_ERR "%s:fail to active sensor\n", __func__);

	return result;
}

static int sensor_init(struct i2c_client *client)
{
	int res = 0;
	unsigned char gyro_reg_data = 0;
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);

	res = sensor->ops->active(client, 0, 0);                        /*Disable accel sensor*/
	if (res) {
		printk(KERN_ERR "%s:line=%d,error\n", __func__, __LINE__);
		return res;
	}
	sensor->status_cur = SENSOR_OFF;

	gyro_reg_data = sensor_read_reg(client, LSM6DS3_GYRO_FS_ADDR);
	gyro_reg_data |= 0x0C;   /*2000dps*/
	/*gyro_reg_data |= 0x40; */  /*104HZ*/
	res = sensor_write_reg(client, LSM6DS3_GYRO_FS_ADDR, gyro_reg_data);
	if (res)
		return res;

	gyro_reg_data = sensor_read_reg(client, LSM6DS3_PWR_ACCEL);
	DBG("gyro_reg_data0 = 0x%2x\n", gyro_reg_data);
	if ((gyro_reg_data & 0xF0) == 0) {
		gyro_reg_data = sensor_read_reg(client, LSM6DS3_RESET_ADDR);
		gyro_reg_data |= 0x41;
		DBG("gyro_reg_data1 = %2x\n", gyro_reg_data);
	} else {
		DBG("gyro_reg_data3 = 0x%2x\n", gyro_reg_data);
		gyro_reg_data = sensor_read_reg(client, LSM6DS3_RESET_ADDR);
		gyro_reg_data |= 0x40;
		DBG("gyro_reg_data2 = %2x\n", gyro_reg_data);
	}
	res = sensor_write_reg(client, LSM6DS3_RESET_ADDR, gyro_reg_data);
	if (res)
		return res;

	res = sensor_write_reg(client, LSM6DS3_LIR_ADDR, 0x81);
	if (res)
		return res;

	res = sensor_write_reg(client, LSM6DS3_ROUNDING_ADDR, 0x04);
	if (res)
		return res;

	res = sensor_write_reg(client, LSM6DS3_INT2_ON_INT1_ADDR, 0x20);
	if (res)
		return res;

	if (sensor->pdata->irq_enable) {
		gyro_reg_data = sensor_read_reg(client, LSM6DS3_INT1_CTRL);
		gyro_reg_data |= 0x02;
		res = sensor_write_reg(client, LSM6DS3_INT1_CTRL, gyro_reg_data);
		if (res)
			return res;
	}

	return res;
}

static int gyro_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *)i2c_get_clientdata(client);

	/* Report acceleration sensor information */
	input_report_rel(sensor->input_dev, ABS_RX, axis->x);
	input_report_rel(sensor->input_dev, ABS_RY, axis->y);
	input_report_rel(sensor->input_dev, ABS_RZ, axis->z);
	input_sync(sensor->input_dev);
	DBG("Gyro sensor x==%d  y==%d z==%d\n", axis->x, axis->y, axis->z);

	return 0;
}

static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);
	struct sensor_platform_data *pdata = sensor->pdata;
	int ret = 0;
	short x, y, z;
	struct sensor_axis axis;
	u8 buffer[6] = {0};
	char value = 0;

	if (sensor->ops->read_len < 6) {
		printk("%s:len is error,len=%d\n", __func__, sensor->ops->read_len);
		return -1;
	}
	memset(buffer, 0, 6);
	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
			return ret;
	} while (0);

	x = ((buffer[1] << 8) & 0xFF00) + (buffer[0] & 0xFF);
	y = ((buffer[3] << 8) & 0xFF00) + (buffer[2] & 0xFF);
	z = ((buffer[5] << 8) & 0xFF00) + (buffer[4] & 0xFF);

	axis.x = (pdata->orientation[0]) * x + (pdata->orientation[1]) * y + (pdata->orientation[2]) * z;
	axis.y = (pdata->orientation[3]) * x + (pdata->orientation[4]) * y + (pdata->orientation[5]) * z;
	axis.z = (pdata->orientation[6]) * x + (pdata->orientation[7]) * y + (pdata->orientation[8]) * z;

	/*filter gyro data*/
	if ((abs(axis.x) > pdata->x_min) || (abs(axis.y) > pdata->y_min) || (abs(axis.z) > pdata->z_min)) {
		gyro_report_value(client, &axis);

		mutex_lock(&sensor->data_mutex);
		sensor->axis = axis;
		mutex_unlock(&sensor->data_mutex);
	}
	/*read sensor intterupt status register*/
	if ((sensor->pdata->irq_enable) && (sensor->ops->int_status_reg >= 0)) {
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		DBG("%s:gyro sensor int status :0x%x\n", __func__, value);
	}

	return ret;
}

struct sensor_operate gyro_lsm6ds3_ops = {
	.name				= "lsm6ds3_gyro",
	.type				= SENSOR_TYPE_GYROSCOPE,		/*sensor type and it should be correct*/
	.id_i2c				= GYRO_ID_LSM6DS3,				/*i2c id number*/
	.read_reg			= LSM6DS3_GYRO_XOUT_H,			/*read data*/
	.read_len			= 6,							/*data length*/
	.id_reg				= LSM6DS3_WHOAMI,				/*read device id from this register*/
	.id_data			= LSM6DS3_DEVICE_ID,			/*device id*/
	.precision			= LSM6DS3_PRECISION,			/*16 bit*/
	.ctrl_reg			= LSM6DS3_PWR_GYRO,				/*enable or disable*/
	.int_status_reg		= SENSOR_UNKNOW_DATA,			/*intterupt status register*/
	.range				= {-2000000, 2000000},			/*range*/
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
	.active				= sensor_active,
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/

/*function name should not be changed*/
static struct sensor_operate *gyro_get_ops(void)
{
	return &gyro_lsm6ds3_ops;
}

static int __init gyro_lsm6ds3_init(void)
{
	struct sensor_operate *ops = gyro_get_ops();
	int result = 0;
	int type = ops->type;

	result = sensor_register_slave(type, NULL, NULL, gyro_get_ops);

	return result;
}

static void __exit gyro_lsm6ds3_exit(void)
{
	struct sensor_operate *ops = gyro_get_ops();
	int type = ops->type;

	sensor_unregister_slave(type, NULL, NULL, gyro_get_ops);
}

module_init(gyro_lsm6ds3_init);
module_exit(gyro_lsm6ds3_exit);
