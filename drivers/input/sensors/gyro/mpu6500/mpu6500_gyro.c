/* drivers/input/sensors/gyro/mpu6500_gyro.c
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd
 * Author: Wang RuoMing <wrm@rock-chips.com>
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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/mpu6500.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/sensor-dev.h>
#include <linux/timer.h>
#include <linux/timex.h>

#define GYRO_CONFIG_FSR_SHIFT		3
#define ACCEL_CONFIG_FSR_SHIFT		3

#define GYRO_MAX_DELAY_MS		1000
#define GYRO_MIN_DELAY_MS		1

#define GYRO_CALIBRATION_COUNT		1
#define GYRO_CALIBRATION_SAMPLE_COUNT	1000

#define MPU6500_SMP_RATE_1000HZ		0x00
#define MPU6500_SMP_RATE_500HZ		0x01
#define MPU6500_SMP_RATE_200HZ		0x04
#define MPU6500_SMP_RATE_100HZ		0x09

#define FIFO_ACCEL			0x01
#define FIFO_GYRO			0x02
#define FIFO_TEMP			0x04

#define BUF_DATA_SIZE			200

#define GYRO_SLEEP			1

struct mpu6500_data {
	struct i2c_client *client;
	struct delayed_work work;
	struct sensor_axis gyro_axis;
	struct sensor_axis accel_axis;
	long long timestamp_usec;
	long long timestamp_nsec;
	long long data_delay_ns;
	long long cur_timestamp;

	int mpu_fifo_enable;
	int mpu_sample_rate;
	int mpu_irq;
	int mpu_irq_handle;
	int mpu_static_judge;
	int gyro_calibration;
	int gyro_new_calibration;

	u8 gyro_xl;
	u8 gyro_xh;
	u8 gyro_yl;
	u8 gyro_yh;
	u8 gyro_zl;
	u8 gyro_zh;

	u8 fifo_feature;
	atomic_t gyro_enable;

	int ave_time;
	int fifo_max;
	int cur_group_count;
	int total_group_count;

	/* use irq */
	unsigned int mpu_irq_gpio;
	int use_buf1;
	int tbuf1_count;
	long long timestamp_buf1[BUF_DATA_SIZE];
	int tbuf2_count;
	long long timestamp_buf2[BUF_DATA_SIZE];
};

enum mpu_fsr {
	MPU_FSR_250DPS = 0,
	MPU_FSR_500DPS,
	MPU_FSR_1000DPS,
	MPU_FSR_2000DPS,
	NUM_FSR
};

/* Filter configurations. */
enum lpf_e {
	INV_FILTER_250HZ_NOLPF2 = 0,
	INV_FILTER_184HZ,
	INV_FILTER_92HZ,
	INV_FILTER_41HZ,
	INV_FILTER_20HZ,
	INV_FILTER_10HZ,
	INV_FILTER_5HZ,
	INV_FILTER_3600HZ_NOLPF,
	NUM_FILTER
};

/* Full scale ranges. */
enum accel_fsr_e {
	INV_FSR_2G = 0,
	INV_FSR_4G,
	INV_FSR_8G,
	INV_FSR_16G,
	NUM_ACCEL_FSR
};

static int sensor_report_gyro_value(struct i2c_client *client,
				    struct sensor_axis *gyro_axis);
static int sensor_report_accel_value(struct i2c_client *client,
				     struct sensor_axis *accel_axis);
static int mpu6500_gyro_setoffset(int config);

static struct mpu6500_data *mpu6500_dev_data;
static struct raw_data_set *fifodata;
static unsigned char *fifo_block;
#ifdef MPU6500_TEST
static long long pre_time;
#endif

static int mpu_i2c_read(
	char slave_addr,
	unsigned char reg_addr,
	unsigned short length,
	unsigned char *rxdata)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	int i = 0, ret = 0;
	unsigned char rdata[800] = {0};

	rdata[0] = reg_addr;
	ret = sensor_rx_data(gdata->client, rdata, length);
	if (ret < 0) {
		dev_dbg(&gdata->client->dev,
			"mpu6500 dmp read failure\n");
		return -1;
	}
	for (i = 0; i < length; i++)
		rxdata[i] = rdata[i];

	return 0;
}

static int mpu6500_read_fifo_block(unsigned char *buf, int len)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;

	if (mpu_i2c_read(0x68, 0x74, len, buf)) {
		dev_err(&gdata->client->dev, "%s error\n", __func__);
		return -1;
	}

	return 0;
}

static int mpu6500_set_fifo_feature(u8 fifo_feature)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	struct i2c_client *client = gdata->client;
	u8 data = 0;

	data = sensor_read_reg(client, MPU6500_FIFO_EN);
	/* fifo data enable gyro and accel temp */
	if (fifo_feature & FIFO_ACCEL)
		data |= 0x08;
	if (fifo_feature & FIFO_GYRO)
		data |= 0x70;
	if (fifo_feature & FIFO_TEMP)
		data |= 0x80;

	sensor_write_reg(client, MPU6500_FIFO_EN, data);
	mdelay(20);
	data = sensor_read_reg(client, MPU6500_USER_CTRL);
	data |= 0x40;
	sensor_write_reg(client, MPU6500_USER_CTRL, data);
	mdelay(20);

	return 0;
}

static int mpu6500_fifo_reset(void)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	unsigned char data = 0;
	struct i2c_client *client = gdata->client;
	unsigned char fifo_en = 0;
	unsigned char user_ctrl = 0;

	gdata->use_buf1 = 1;
	gdata->tbuf1_count = 0;
	gdata->tbuf2_count = 0;

	fifo_en = sensor_read_reg(client, MPU6500_FIFO_EN);
	user_ctrl = sensor_read_reg(client, MPU6500_USER_CTRL);

	sensor_write_reg(client, MPU6500_FIFO_EN, 0);
	sensor_write_reg(client, MPU6500_USER_CTRL, 0);

	/* reset fifo */
	data = BIT_FIFO_RST;
	sensor_write_reg(client, MPU6500_USER_CTRL, data);

	/* fifo enable */
	mdelay(50);
	sensor_write_reg(client, MPU6500_FIFO_EN, fifo_en);
	sensor_write_reg(client, MPU6500_USER_CTRL, user_ctrl);
	gdata->tbuf1_count = 0;
	gdata->tbuf2_count = 0;
	return 0;
}

static int gyro_report_value(struct i2c_client *client,
			     struct sensor_axis *axis)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);

	/* Report acceleration sensor information */
	input_report_rel(sensor->input_dev, ABS_RX, axis->x);
	input_report_rel(sensor->input_dev, ABS_RY, axis->y);
	input_report_rel(sensor->input_dev, ABS_RZ, axis->z);
	input_sync(sensor->input_dev);
	dev_dbg(&gdata->client->dev,
		"Gyro x==%d  y==%d z==%d\n", axis->x, axis->y, axis->z);

	return 0;
}

static int sensor_report_value(struct i2c_client *client)
{
	int ret = 0;
	short x, y, z;
	struct sensor_axis axis;
	u8 buffer[6] = {0};
	struct mpu6500_data *gdata = mpu6500_dev_data;
	struct sensor_private_data *sensor = (struct sensor_private_data *)
					     i2c_get_clientdata(client);

	if (gdata->mpu_fifo_enable == 1)
		return 0;

	if (sensor->ops->read_len < 6) {
		dev_err(&gdata->client->dev,
			"%s:length is error,len=%d\n",
			__func__,
			sensor->ops->read_len);
		return -1;
	}

	memset(buffer, 0, 6);

	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
			return ret;
	} while (0);

	x = ((buffer[0] << 8) & 0xFF00) + (buffer[1] & 0xFF);
	y = ((buffer[2] << 8) & 0xFF00) + (buffer[3] & 0xFF);
	z = ((buffer[4] << 8) & 0xFF00) + (buffer[5] & 0xFF);

	/* save raw data */
	gdata->gyro_axis.x = x;
	gdata->gyro_axis.y = y;
	gdata->gyro_axis.z = z;

	axis.x = x;
	axis.y = y;
	axis.z = z;

	gyro_report_value(client, &axis);

	return ret;
}

static int sensor_report_accel_value(
	struct i2c_client *client,
	struct sensor_axis *accel_axis)
{
	int ret = 0;
	short x, y, z;
	u8 buffer[6] = {0};
	struct mpu6500_data *gdata = mpu6500_dev_data;
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);

	/* sensor->ops->read_len = 6 */
	if (sensor->ops->read_len < 6) {
		dev_err(&gdata->client->dev,
			"%s:length is error,len=%d\n",
			__func__,
			sensor->ops->read_len);
		return -1;
	}

	memset(buffer, 0, 6);

	do {
		*buffer = MPU6500_ACCEL_XOUT_H;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
			return ret;
	} while (0);

	x = ((buffer[0] << 8) & 0xFF00) + (buffer[1] & 0xFF);
	y = ((buffer[2] << 8) & 0xFF00) + (buffer[3] & 0xFF);
	z = ((buffer[4] << 8) & 0xFF00) + (buffer[5] & 0xFF);

	/* save raw data */
	gdata->accel_axis.x = x;
	gdata->accel_axis.y = y;
	gdata->accel_axis.z = z;

	accel_axis->x = x;
	accel_axis->y = y;
	accel_axis->z = z;
	dev_dbg(&gdata->client->dev, "accel: x=%d y=%d z=%d\n", x, y, z);

	return ret;
}

static int sensor_report_gyro_value(struct i2c_client *client,
				    struct sensor_axis *gyro_axis)
{
	int ret = 0;
	short x, y, z;
	u8 buffer[6] = {0};
	struct mpu6500_data *gdata = mpu6500_dev_data;
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);

	if (sensor->ops->read_len < 6) {
		dev_err(&gdata->client->dev,
			"%s:length is error,len=%d\n",
			__func__,
			sensor->ops->read_len);
		return -1;
	}

	memset(buffer, 0, 6);

	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
			return ret;
	} while (0);

	x = ((buffer[0] << 8) & 0xFF00) + (buffer[1] & 0xFF);
	y = ((buffer[2] << 8) & 0xFF00) + (buffer[3] & 0xFF);
	z = ((buffer[4] << 8) & 0xFF00) + (buffer[5] & 0xFF);

	/* save raw data */
	gdata->gyro_axis.x = x;
	gdata->gyro_axis.y = y;
	gdata->gyro_axis.z = z;

	gyro_axis->x = x;
	gyro_axis->y = y;
	gyro_axis->z = z;

	dev_dbg(&gdata->client->dev, "gyro: x=%d y=%d z=%d\n", x, y, z);

	return ret;
}

static int mpu6500_gyro_setoffset(int config)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	struct i2c_client *client = gdata->client;
	u8 tmp;

	dev_dbg(&gdata->client->dev,
		"kernel_save: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		gdata->gyro_xl,
		gdata->gyro_xh,
		gdata->gyro_yl,
		gdata->gyro_yh,
		gdata->gyro_zl,
		gdata->gyro_zh);
	if (config) {
		tmp = gdata->gyro_xh;
		sensor_write_reg(client, MPU6500_GYRO_OFFSET_XH, tmp);
		tmp = gdata->gyro_xl;
		sensor_write_reg(client, MPU6500_GYRO_OFFSET_XL, tmp);
		tmp = gdata->gyro_yh;
		sensor_write_reg(client, MPU6500_GYRO_OFFSET_YH, tmp);
		tmp = gdata->gyro_yl;
		sensor_write_reg(client, MPU6500_GYRO_OFFSET_YL, tmp);
		tmp = gdata->gyro_zh;
		sensor_write_reg(client, MPU6500_GYRO_OFFSET_ZH, tmp);
		tmp = gdata->gyro_zl;
		sensor_write_reg(client, MPU6500_GYRO_OFFSET_ZL, tmp);

		gdata->gyro_calibration = 1;
	}

	return 0;
}

static int mpu6500_parsing_block(
	u8 feature,
	unsigned char *fifo_data,
	short *accel,
	short *gyro,
	short *temp)
{
	int i = 0;

	if (feature & 0x01) {
		accel[0] = fifo_data[i + 0];
		accel[0] = (accel[0] << 8) | fifo_data[i + 1];
		accel[1] = fifo_data[i + 2];
		accel[1] = (accel[1] << 8) | fifo_data[i + 3];
		accel[2] = fifo_data[i + 4];
		accel[2] = (accel[2] << 8) | fifo_data[i + 5];
		i += 6;
	}

	if (feature & 0x04) {
		*temp = (short)(fifo_data[i + 0] << 8) | fifo_data[i + 1];
		i += 2;
	}

	if (feature & 0x02) {
		gyro[0] = fifo_data[i + 0];
		gyro[0] = (gyro[0] << 8) | fifo_data[i + 1];
		gyro[1] = fifo_data[i + 2];
		gyro[1] = (gyro[1] << 8) | fifo_data[i + 3];
		gyro[2] = fifo_data[i + 4];
		gyro[2] = (gyro[2] << 8) | fifo_data[i + 5];
		i += 6;
	}

	return 0;
}

static int mpu6500_set_fifodata_block(
	u8 feature,
	unsigned char *buf,
	short byte_len)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	int i = 0, j = 0, k = 0;
	int read_size = 0;
	short accel[3] = {0};
	short gyro[3] = {0};
	short temperature = 0;
	unsigned char *buf_tmp = buf;

	/* read fifo temperature data */
	if (feature & 0x04)
		read_size += 2;
	/* read fifo gyro data */
	if (feature & 0x02)
		read_size += 6;
	/* read fifo accel data */
	if (feature & 0x01)
		read_size += 6;

	j = byte_len / read_size;

	for (k = 0; k < j; k++) {
		i = fifodata->avail_num;
		if (i >= 200) {
			i = 200 - 1;
			fifodata->avail_num = i;
		}
		mpu6500_parsing_block(feature,
				      buf_tmp,
				      accel,
				      gyro,
				      &temperature);

		fifodata->sensordata[i].accel_axis.x = accel[0];
		fifodata->sensordata[i].accel_axis.y = accel[1];
		fifodata->sensordata[i].accel_axis.z = accel[2];
		fifodata->sensordata[i].gyro_axis.x = gyro[0];
		fifodata->sensordata[i].gyro_axis.y = gyro[1];
		fifodata->sensordata[i].gyro_axis.z = gyro[2];
		fifodata->sensordata[i].temperature = temperature;
		fifodata->sensordata[i].quat_flag = 0;
		fifodata->sensordata[i].use_irq = 0;
		fifodata->sensordata[i].timestamp_usec =
			gdata->timestamp_nsec - gdata->ave_time *
			(gdata->total_group_count - gdata->cur_group_count) -
			gdata->data_delay_ns;

		buf_tmp += read_size;
		fifodata->avail_num++;
		gdata->cur_group_count++;
	}

	return 0;
}

static int mpu6500_parse_dts(struct mpu6500_data *gdata,
			     struct device_node *np)
{
	int rv = -1;
	enum of_gpio_flags flags;

	rv = of_property_read_u32(np,
				  "mpu-irq",
				  &gdata->mpu_irq);
	if (rv) {
		dev_err(&gdata->client->dev,
			"MPU6500 get mpu_irq failure\n");
		gdata->mpu_irq = 0;
	}

	rv = of_property_read_u32(np,
				  "mpu-sample-rate",
				  &gdata->mpu_sample_rate);
	if (rv) {
		dev_err(&gdata->client->dev,
			"MPU6500 get mpu-sample-rate failure\n");
		gdata->mpu_sample_rate = 100;
	}

	rv = of_property_read_u32(np,
				  "mpu-fifo-enable",
				  &gdata->mpu_fifo_enable);
	if (rv) {
		dev_err(&gdata->client->dev,
			"MPU6500 get mpu-fifo-enable failure\n");
		gdata->mpu_fifo_enable = 0;
	}

	rv = of_property_read_u32(np,
				  "mpu-static-judge",
				  &gdata->mpu_static_judge);
	if (rv) {
		dev_err(&gdata->client->dev,
			"MPU6500 get mpu-static-judge failure\n");
		gdata->mpu_static_judge = 0;
	}

	if (gdata->mpu_irq == 1) {
		gdata->mpu_irq_gpio = of_get_named_gpio_flags(np,
							      "mpu-irq-gpio",
							      0,
							      &flags);
		if (gpio_is_valid(gdata->mpu_irq_gpio)) {
			rv = gpio_request(gdata->mpu_irq_gpio, "mpu_irq_state");
			if (rv) {
				dev_err(&gdata->client->dev,
					"mpu6500 dmp gpio request failure!\n");
				return -1;
			}
			gpio_direction_input(gdata->mpu_irq_gpio);
		} else {
			dev_err(&gdata->client->dev,
				"mpu6500 irq gpio invalid!\n");
		}
	}

	return rv;
}

static ssize_t gyro_enable_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;

	return  sprintf(buf, "%d\n", atomic_read(&gdata->gyro_enable));
}

static void gyro_set_enable(struct device *dev, int enable)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	int pre_enable;

	pre_enable = atomic_read(&gdata->gyro_enable);

	if ((pre_enable == 0) && (enable == 1)) {
		/* deal with gyroscope enable */
		atomic_set(&gdata->gyro_enable, 1);
	}

	if ((pre_enable == 1) && (enable == 0)) {
		/* deal with gyroscope disable */
		atomic_set(&gdata->gyro_enable, 0);
	}
}

static ssize_t gyro_enable_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	unsigned long data;
	int err;

	err = kstrtoul(buf, 10, &data);
	if (err) {
		dev_err(&gdata->client->dev,
			"read user space enable node failure!\n");
		return err;
	}

	if ((data == 1) || (data == 0))
		gyro_set_enable(dev, data);

	return count;
}

static int mpu6500_calibration(int type)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	struct i2c_client *client = gdata->client;
	struct sensor_axis gyro_axis;
	struct sensor_axis accel_ax1;
	struct sensor_axis accel_ax2;
	short gyro_offset_x = 0;
	short gyro_offset_y = 0;
	short gyro_offset_z = 0;
	int i = 0;
	int gyro_x = 0, gyro_y = 0, gyro_z = 0;
	int calibration_count = 0;
	u8 offset_h = 0;
	u8 offset_l = 0;

	/* 1.mpu6500 gyro 0:mpu6500 accel */
	if (type) {
		dev_dbg(&client->dev, "In horizontal and keep still\n");

		while (calibration_count < GYRO_CALIBRATION_COUNT) {
			dev_dbg(&client->dev, "--------------------\n");
			/* get accel data for judge static */
			if (gdata->mpu_static_judge == 1)
				sensor_report_accel_value(client,
							  &accel_ax1);
			/* calibration */
			for (i = 0; i < GYRO_CALIBRATION_SAMPLE_COUNT; i++) {
				sensor_report_gyro_value(client, &gyro_axis);
				dev_dbg(&client->dev,
					"gyro_x=%d gyro_y=%d gyro_z=%d\n",
					gyro_axis.x,
					gyro_axis.y,
					gyro_axis.z);
				gyro_x += (gyro_axis.x * 2);
				gyro_y += (gyro_axis.y * 2);
				gyro_z += (gyro_axis.z * 2);
				/* msleep(20); */
			}
			/* get accel data for judge static */
			if (gdata->mpu_static_judge == 1) {
				sensor_report_accel_value(client,
							  &accel_ax2);
				if ((abs(accel_ax1.x - accel_ax2.x) > 200) ||
				    (abs(accel_ax1.y - accel_ax2.y) > 200) ||
				    (abs(accel_ax1.z - accel_ax2.z) > 200)) {
					dev_dbg(&client->dev,
						"stop mpu6500 gyro calibration\n");
					if (calibration_count >= 1) {
						dev_dbg(&client->dev,
							"calibration success\n");
						goto JUMP;
					} else {
						dev_dbg(&client->dev,
							"calibration failure\n");
						goto JUMP;
					}
				}
			}
			gyro_x = gyro_x / GYRO_CALIBRATION_SAMPLE_COUNT;
			gyro_y = gyro_y / GYRO_CALIBRATION_SAMPLE_COUNT;
			gyro_z = gyro_z / GYRO_CALIBRATION_SAMPLE_COUNT;
			dev_dbg(&client->dev,
				"%d calibration: g_x_ave=%d g_y_ave=%d g_z_ave=%d\n",
				calibration_count + 1,
				gyro_x,
				gyro_y,
				gyro_z);

			offset_l = sensor_read_reg(client,
						   MPU6500_GYRO_OFFSET_XL);
			offset_h = sensor_read_reg(client,
						   MPU6500_GYRO_OFFSET_XH);
			gyro_offset_x = offset_h;
			gyro_offset_x = (gyro_offset_x << 8) | offset_l;
			offset_l = sensor_read_reg(client,
						   MPU6500_GYRO_OFFSET_YL);
			offset_h = sensor_read_reg(client,
						   MPU6500_GYRO_OFFSET_YH);
			gyro_offset_y = offset_h;
			gyro_offset_y = (gyro_offset_y << 8) | offset_l;
			offset_l = sensor_read_reg(client,
						   MPU6500_GYRO_OFFSET_ZL);
			offset_h = sensor_read_reg(client,
						   MPU6500_GYRO_OFFSET_ZH);
			gyro_offset_z = offset_h;
			gyro_offset_z = (gyro_offset_z << 8) | offset_l;

			gyro_offset_x -= gyro_x;
			gyro_offset_y -= gyro_y;
			gyro_offset_z -= gyro_z;

			dev_dbg(&client->dev,
				"%d calibration offset: gf_x=0x%x gf_y=0x%x gf_z=0x%x\n",
				calibration_count + 1,
				gyro_offset_x,
				gyro_offset_y,
				gyro_offset_z);
			offset_h = (gyro_offset_x >> 8);
			gdata->gyro_xh = offset_h;
			offset_l = (gyro_offset_x & 0x00ff);
			gdata->gyro_xl = offset_l;
			sensor_write_reg(client,
					 MPU6500_GYRO_OFFSET_XH,
					 offset_h);
			sensor_write_reg(client,
					 MPU6500_GYRO_OFFSET_XL,
					 offset_l);

			offset_h = (gyro_offset_y >> 8);
			gdata->gyro_yh = offset_h;
			offset_l = (gyro_offset_y & 0x00ff);
			gdata->gyro_yl = offset_l;
			sensor_write_reg(client,
					 MPU6500_GYRO_OFFSET_YH,
					 offset_h);
			sensor_write_reg(client,
					 MPU6500_GYRO_OFFSET_YL,
					 offset_l);

			offset_h = (gyro_offset_z >> 8);
			gdata->gyro_zh = offset_h;
			offset_l = (gyro_offset_z & 0x00ff);
			gdata->gyro_zl = offset_l;
			sensor_write_reg(client,
					 MPU6500_GYRO_OFFSET_ZH,
					 offset_h);
			sensor_write_reg(client,
					 MPU6500_GYRO_OFFSET_ZL,
					 offset_l);

			calibration_count++;
		}
JUMP:
		dev_dbg(&client->dev, "gyro calibration complete and exit\n");
		if (calibration_count == 0)
			gdata->gyro_new_calibration = 0;
		else
			gdata->gyro_new_calibration = 1;
	} else {
		dev_dbg(&client->dev, "accel calibration\n");
	}

	return 0;
}

static ssize_t gyro_calibration_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;

	return sprintf(buf, "%d\n", gdata->gyro_calibration);
}

static ssize_t gyro_calibration_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	unsigned long data;
	int err;

	err = kstrtoul(buf, 10, &data);
	if (err) {
		dev_err(&gdata->client->dev,
			"read user space enable node failure!\n");
		return err;
	}

	if (data == 1)
		mpu6500_calibration(1);

	return count;
}

static ssize_t gyro_tempearture_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	char rdata[2] = {0};
	short temperature = 0;

	rdata[0] = MPU6500_TEMP_OUT_H;
	sensor_rx_data(gdata->client, rdata, 2);
	temperature = rdata[0];
	temperature = (temperature << 8) | rdata[1];

	return sprintf(buf, "%d\n", temperature);
}

static DEVICE_ATTR(gyro_enable, 0664,
		   gyro_enable_show, gyro_enable_store);

static DEVICE_ATTR(gyro_calibration,
		   0664,
		   gyro_calibration_show,
		   gyro_calibration_store);

static DEVICE_ATTR(gyro_temperature,
		   0444,
		   gyro_tempearture_show,
		   NULL);

static struct attribute *mpu6500_attributes[] = {
	&dev_attr_gyro_enable.attr,
	&dev_attr_gyro_calibration.attr,
	&dev_attr_gyro_temperature.attr,
	NULL
};

static struct attribute_group mpu6500_attribute_group = {
	.attrs = mpu6500_attributes
};

static int mpu6500_sensor_configure(struct i2c_client *client)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	int ret = 0;
	u8 wdata = 0, rdata = 0;

	/* Sample Div */
	switch (gdata->mpu_sample_rate) {
	case 1000:
		rdata = MPU6500_SMP_RATE_1000HZ;
		gdata->ave_time = 1001301;/* ns */
		break;
	case 500:
		rdata = MPU6500_SMP_RATE_500HZ;
		gdata->ave_time = 2000000;/* ns */
		break;
	case 200:
		rdata = MPU6500_SMP_RATE_200HZ;
		gdata->ave_time = 5000000;/* ns */
		break;
	default:
		rdata = MPU6500_SMP_RATE_100HZ;
		gdata->ave_time = 10000000;/* ns */
		break;
	}
	sensor_write_reg(client, MPU6500_SMPLRT_DIV, rdata);

	/* Configure MPU6500 gyro low pass filter */
	wdata = INV_FILTER_184HZ;
	gdata->data_delay_ns = 29 * 1000 * 100;
	ret = sensor_write_reg(client, MPU6500_CONFIG, wdata);
	if (ret < 0)
		return ret;

	/* Gyro fsr configure and lpf(FCHOICE_B<bit1:0 bit2 : 0>) */
	ret = sensor_write_reg(
		      client,
		      MPU6500_GYRO_CONFIG,
		      MPU_FSR_2000DPS << GYRO_CONFIG_FSR_SHIFT);
	if (ret < 0)
		return ret;

	/* Configure MPU6500 FIFO SIZE and ACCEL LPF */
	wdata = BIT_FIFO_SIZE_2K | A_DLPF_CFG_460HZ;
	ret = sensor_write_reg(client, MPU6500_ACCEL_CONFIG2, wdata);
	if (ret < 0)
		return ret;
	if ((wdata & BIT_FIFO_SIZE_1K) == BIT_FIFO_SIZE_1K)
		gdata->fifo_max = 1024;
	if ((wdata & BIT_FIFO_SIZE_2K) == BIT_FIFO_SIZE_2K)
		gdata->fifo_max = 2048;
	if ((wdata & BIT_FIFO_SIZE_4K) == BIT_FIFO_SIZE_4K)
		gdata->fifo_max = 4096;

	/* Configure MPU6500 accel FULL SCALE RANGE */
	ret = sensor_write_reg(
		      client,
		      MPU6500_ACCEL_CONFIG,
		      INV_FSR_2G << ACCEL_CONFIG_FSR_SHIFT);
	if (ret < 0)
		return ret;

	ret = sensor_write_reg(client, MPU6500_INT_ENABLE, 0);
	if (ret < 0)
		return ret;

	return ret;
}

static int sensor_calibration(struct i2c_client *client, char *data)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	int gyro_enable;

	gyro_enable = atomic_read(&gdata->gyro_enable);

	if (data[0] == 1) {
		if (gdata->gyro_new_calibration == 1) {
			if (gdata->gyro_calibration == 0)
				gdata->gyro_calibration = 1;
			data[7] = sensor_read_reg(gdata->client,
						  MPU6500_TEMP_OUT_H);
			data[8] = sensor_read_reg(gdata->client,
						  MPU6500_TEMP_OUT_L);
			/* get calibration data */
			data[1] = gdata->gyro_xl;
			data[2] = gdata->gyro_xh;
			data[3] = gdata->gyro_yl;
			data[4] = gdata->gyro_yh;
			data[5] = gdata->gyro_zl;
			data[6] = gdata->gyro_zh;
		} else {
			return -1;
		}

	} else if (data[0] == 2) {
		/* set calibration data */
		gdata->gyro_xl = data[1];
		gdata->gyro_xh = data[2];
		gdata->gyro_yl = data[3];
		gdata->gyro_yh = data[4];
		gdata->gyro_zl = data[5];
		gdata->gyro_zh = data[6];
		if (gyro_enable == 1) {
			sensor_write_reg(gdata->client,
					 MPU6500_GYRO_OFFSET_XH,
					 gdata->gyro_xh);
			sensor_write_reg(gdata->client,
					 MPU6500_GYRO_OFFSET_XL,
					 gdata->gyro_xl);
			sensor_write_reg(gdata->client,
					 MPU6500_GYRO_OFFSET_YH,
					 gdata->gyro_yh);
			sensor_write_reg(gdata->client,
					 MPU6500_GYRO_OFFSET_YL,
					 gdata->gyro_yl);
			sensor_write_reg(gdata->client,
					 MPU6500_GYRO_OFFSET_ZH,
					 gdata->gyro_zh);
			sensor_write_reg(gdata->client,
					 MPU6500_GYRO_OFFSET_ZL,
					 gdata->gyro_zl);
		}
	}

	return 0;
}

static int sensor_direct_getdata_from_fifo(
	struct i2c_client *client,
	void *data)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	raw_data_set_t **rdata = (raw_data_set_t **)data;
	struct timex txc;
	short fifo_count;
	int read_size = 0;
	unsigned char read_data[2] = {0};
	short getdata_max = 0;

	fifodata->avail_num = 0;
	if (gdata->fifo_feature & FIFO_ACCEL)
		read_size += 6;
	if (gdata->fifo_feature & FIFO_GYRO)
		read_size += 6;
	if (gdata->fifo_feature & FIFO_TEMP)
		read_size += 2;
	getdata_max = 80 / read_size;
	getdata_max = getdata_max * read_size;

	if (gdata->mpu_fifo_enable) {
		read_data[0] = MPU6500_FIFO_COUNTH;
		sensor_rx_data(gdata->client, read_data, 2);
		fifo_count = read_data[0];
		fifo_count = (fifo_count << 8) | read_data[1];

		do_gettimeofday(&txc.time);
		gdata->cur_timestamp = (long long)txc.time.tv_sec * 1000 * 1000
				+ (long long)txc.time.tv_usec;
		gdata->timestamp_nsec = gdata->cur_timestamp * 1000;
		gdata->total_group_count = fifo_count / read_size;
		gdata->cur_group_count = 0;
		/* read data */
		if (fifo_count >= (gdata->fifo_max)) {
			/* clear fifo */
			dev_dbg(&gdata->client->dev,
				"mpu6500_fifo_reset count=%d\n", fifo_count);
			mpu6500_fifo_reset();

			return -1;
		}

		while (fifo_count >=  getdata_max) {
			mpu6500_read_fifo_block(fifo_block, getdata_max);
			fifo_count -= getdata_max;
			mpu6500_set_fifodata_block(gdata->fifo_feature,
						   fifo_block,
						   getdata_max);
		}
		fifo_count = fifo_count / read_size;
		fifo_count = fifo_count * read_size;
		if (fifo_count > 0) {
			mpu6500_read_fifo_block(fifo_block, fifo_count);
			mpu6500_set_fifodata_block(gdata->fifo_feature,
						   fifo_block,
						   fifo_count);
		}
		*rdata = fifodata;
	}

	return 0;
}

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	int result = 0;
	u8 data = 0;

	if (enable == 0) {
		sensor_write_reg(client, MPU6500_INT_ENABLE, 0);
		data = sensor_read_reg(client, MPU6500_INT_PIN_CFG);
		data |= 0x20;
		sensor_write_reg(client, MPU6500_INT_PIN_CFG, data);
		data = BIT_GYRO_STANDBY;
		sensor_write_reg(client, MPU6500_PWR_MGMT_1, data);
		atomic_set(&gdata->gyro_enable, 0);
	} else {
		/* Enable sensor Select Clk Source */
		data = BIT_CLKSEL;
		sensor_write_reg(client, MPU6500_PWR_MGMT_1, data);

		if (gdata->mpu_fifo_enable == 1) {
			msleep(200);
			mpu6500_fifo_reset();
			if (gdata->mpu_irq == 1) {
				sensor_write_reg(client,
						 MPU6500_INT_PIN_CFG,
						 0x80);
				sensor_write_reg(client,
						 MPU6500_INT_ENABLE,
						 BIT_RAW_RDY);
				pr_info("%s: use BIT_RAW_RDY\n", __func__);
			}
		}

		mpu6500_gyro_setoffset(1);
		atomic_set(&gdata->gyro_enable, 1);
	}

	return result;
}

static irqreturn_t irq_handle_fifo_func(int irq, void *data)
{
	struct mpu6500_data *gdata = (struct mpu6500_data *)data;
	struct timex txc;
#ifdef MPU6500_TEST
	long long time_interval = 0;
	long long cur_time = 0;
#endif
	do_gettimeofday(&txc.time);
#ifdef MPU6500_TEST
	cur_time = (long long)txc.time.tv_sec * 1000 * 1000 +
		   (long long)txc.time.tv_usec;
	time_interval = cur_time - pre_time;
	pr_info("irq: time_interval=%lld\n", time_interval);
	pre_time = cur_time;
#endif

	if (gdata->use_buf1) {
		gdata->tbuf1_count++;
		gdata->timestamp_buf1[gdata->tbuf1_count - 1] =
			(long long)txc.time.tv_sec * 1000 * 1000 +
			(long long)txc.time.tv_usec;
	} else {
		gdata->tbuf2_count++;
		gdata->timestamp_buf2[gdata->tbuf2_count - 1] =
			(long long)txc.time.tv_sec * 1000 * 1000 +
			(long long)txc.time.tv_usec;
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t mpu6500_irq_work_func(int irq, void *data)
{
	struct mpu6500_data *gdata = (struct mpu6500_data *)data;

	if ((gdata->tbuf1_count >= BUF_DATA_SIZE) ||
	    (gdata->tbuf2_count >= BUF_DATA_SIZE)) {
#ifdef MPU6500_TEST
		pr_info("mpu6500 timestamp buffer overflow\n");
#endif
		mpu6500_fifo_reset();
	}

	return IRQ_HANDLED;
}

static int mpu6500_irq_mode_parsing_fifoblock(
	u8 feature,
	unsigned char *buf,
	short byte_len)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	short accel[3] = {0};
	short gyro[3] = {0};
	short temperature = 0;
	unsigned char *buf_tmp = buf;
	int read_size = 0;
	int i = 0, k = 0, total_group_count = 0;

	/* read fifo temperature data */
	if (feature & 0x04)
		read_size += 2;
	/* read fifo gyro data */
	if (feature & 0x02)
		read_size += 6;
	/* read fifo accel data */
	if (feature & 0x01)
		read_size += 6;

	total_group_count = byte_len / read_size;

	for (k = 0; k < total_group_count; k++) {
		i = fifodata->avail_num;
		if (i >= 200) {
			i = 200 - 1;
			fifodata->avail_num = i;
		}
		mpu6500_parsing_block(feature,
				      buf_tmp,
				      accel,
				      gyro,
				      &temperature);

		fifodata->sensordata[i].accel_axis.x = accel[0];
		fifodata->sensordata[i].accel_axis.y = accel[1];
		fifodata->sensordata[i].accel_axis.z = accel[2];
		fifodata->sensordata[i].gyro_axis.x = gyro[0];
		fifodata->sensordata[i].gyro_axis.y = gyro[1];
		fifodata->sensordata[i].gyro_axis.z = gyro[2];
		fifodata->sensordata[i].temperature = temperature;
		fifodata->sensordata[i].quat_flag = 0;
		fifodata->sensordata[i].use_irq = 1;
		if (gdata->use_buf1 == 0) {
			fifodata->sensordata[i].timestamp_usec =
				gdata->timestamp_buf1[i] - 5900; /* 5.9ms */
		} else {
			fifodata->sensordata[i].timestamp_usec =
				gdata->timestamp_buf2[i] - 5900;
		}
		buf_tmp += read_size;
		fifodata->avail_num++;
	}

	return 0;
}

static int sensor_use_irq_getdata(struct i2c_client *client, void *data)
{
	struct mpu6500_data *gdata = mpu6500_dev_data;
	raw_data_set_t **rdata = (raw_data_set_t **)data;
	int getdata_max_byte = 0;
	int feature_byte = 0;
	int read_byte = 0;
	int data_group_count = 0;

	fifodata->avail_num = 0;
	if (gdata->fifo_feature & FIFO_ACCEL)
		feature_byte += 6;
	if (gdata->fifo_feature & FIFO_GYRO)
		feature_byte += 6;
	if (gdata->fifo_feature & FIFO_TEMP)
		feature_byte += 2;

	read_byte = feature_byte;
	getdata_max_byte = read_byte * 50;

	if (gdata->use_buf1) {
		gdata->use_buf1 = 0;
		data_group_count = gdata->tbuf1_count;
		read_byte = data_group_count * read_byte;
		while (read_byte >= getdata_max_byte) {
			mpu6500_read_fifo_block(fifo_block, getdata_max_byte);
			read_byte -= getdata_max_byte;
			mpu6500_irq_mode_parsing_fifoblock(gdata->fifo_feature,
							   fifo_block,
							   getdata_max_byte);
		}

		if (gdata->tbuf1_count > data_group_count) {
			read_byte += (gdata->tbuf1_count - data_group_count)
				* feature_byte;
			pr_info("polishing buf1 data: %d read_byte=%d\n",
				gdata->tbuf1_count - data_group_count,
				read_byte);
		}

		if (read_byte > 0) {
			mpu6500_read_fifo_block(fifo_block, read_byte);
			mpu6500_irq_mode_parsing_fifoblock(gdata->fifo_feature,
							   fifo_block,
							   read_byte);
		}
		gdata->tbuf1_count = 0;
	} else {
		gdata->use_buf1 = 1;
		data_group_count = gdata->tbuf2_count;
		read_byte = data_group_count * read_byte;
		while (read_byte >=  getdata_max_byte) {
			mpu6500_read_fifo_block(fifo_block, getdata_max_byte);
			read_byte -= getdata_max_byte;
			mpu6500_irq_mode_parsing_fifoblock(gdata->fifo_feature,
							   fifo_block,
							   getdata_max_byte);
		}

		if (gdata->tbuf2_count > data_group_count) {
			read_byte += (gdata->tbuf2_count - data_group_count)
				* feature_byte;
			pr_info("polishing buf2 data: %d read_byte=%d\n",
				gdata->tbuf2_count - data_group_count,
				read_byte);
		}

		if (read_byte > 0) {
			mpu6500_read_fifo_block(fifo_block, read_byte);
			mpu6500_irq_mode_parsing_fifoblock(gdata->fifo_feature,
							   fifo_block,
							   read_byte);
		}
		gdata->tbuf2_count = 0;
	}

	*rdata = fifodata;

	return 0;
}

static int sensor_init(struct i2c_client *client)
{
	int ret;
	u8 rdata;
	struct mpu6500_data *gdata;
	struct device_node *np = client->dev.of_node;

	gdata = kzalloc(sizeof(*gdata), GFP_KERNEL);
	if (!gdata) {
		ret = -ENOMEM;
		return ret;
	}

	/* reset device */
	ret = sensor_write_reg(client, MPU6500_PWR_MGMT_1, BIT_H_RESET);
	if (ret < 0)
		goto error;
	do {
		msleep(20);
		/* check reset complete */
		ret = sensor_read_reg(client, MPU6500_PWR_MGMT_1);
		if (ret < 0) {
			dev_err(&client->dev,
				"Failed to read reset status ret =%d\n",
				ret);
			goto error;
		}
		rdata = (u8)ret;
	} while (rdata & BIT_H_RESET);

	gdata->client = client;
	ret = mpu6500_parse_dts(gdata, np);
	if (ret)
		dev_err(&client->dev, "mpu6500_parse_dts failure\n");

	gdata->use_buf1 = 1;
	mpu6500_dev_data = gdata;

	atomic_set(&gdata->gyro_enable, 0);

	/* configure gyro and accel */
	mpu6500_sensor_configure(client);

	/* create /sys/class/misc/gyrosensor/device/... */
	ret = sysfs_create_group(&client->dev.kobj, &mpu6500_attribute_group);
	if (ret < 0) {
		dev_err(&client->dev, "create system file node failure!\n");
		goto error;
	}

	if (gdata->mpu_fifo_enable == 1) {
		fifodata = devm_kzalloc(&client->dev,
					sizeof(*fifodata),
					GFP_KERNEL);
		if (!fifodata) {
			dev_err(&client->dev, "fifodata kalloc failure\n");
			ret = -ENOMEM;
			goto error1;
		}
		fifodata->avail_num = 0;
		fifo_block = devm_kzalloc(&client->dev,
					  gdata->fifo_max,
					  GFP_KERNEL);
		if (!fifo_block) {
			dev_err(&client->dev, "fifo_block kalloc failure\n");
			ret = -ENOMEM;
			goto error1;
		}
		if (gdata->mpu_irq == 1) {
			gdata->mpu_irq_handle =
				gpio_to_irq(gdata->mpu_irq_gpio);
			ret = request_threaded_irq(gdata->mpu_irq_handle,
						   irq_handle_fifo_func,
						   mpu6500_irq_work_func,
						   IRQ_TYPE_EDGE_FALLING,
						   "mpu6500-irq",
						   gdata);
			if (ret) {
				pr_info("mpu6500 irq failure!\n");
				goto error1;
			}
		}

		gdata->fifo_feature = FIFO_GYRO | FIFO_TEMP;
		mpu6500_set_fifo_feature(gdata->fifo_feature);
	}

	return 0;

error1:
	sysfs_remove_group(&client->dev.kobj, &mpu6500_attribute_group);
error:
	kfree(gdata);
	return -1;
}

struct sensor_operate gyro_mpu6500_ops = {
	.name				= "mpu6500_gyro",
	.type				= SENSOR_TYPE_GYROSCOPE,
	.id_i2c				= GYRO_ID_MPU6500,
	.read_reg			= MPU6500_GYRO_XOUT_H,
	.read_len			= 6,
	.id_reg				= MPU6500_WHOAMI,
	.id_data			= MPU6500_DEVICE_ID,
	.precision			= MPU6500_PRECISION,
	.ctrl_reg			= MPU6500_PWR_MGMT_2,
	.int_status_reg			= MPU6500_INT_STATUS,
	.range				= { -MPU6500_RANGE, MPU6500_RANGE},
	.trig				= IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
	.active				= sensor_active,
	.init				= sensor_init,
	.report				= sensor_report_value,
	.getdata                        = sensor_use_irq_getdata,
	.get_fifo_data			= sensor_direct_getdata_from_fifo,
	.calibration			= sensor_calibration,
};

/****************operate according to sensor chip:end************/

static struct sensor_operate *gyro_get_ops(void)
{
	return &gyro_mpu6500_ops;
}

static int __init gyro_mpu6500_init(void)
{
	struct sensor_operate *ops = gyro_get_ops();
	int result = 0;
	int type = ops->type;

	result = sensor_register_slave(type, NULL, NULL, gyro_get_ops);

	return result;
}

static void __exit gyro_mpu6500_exit(void)
{
	struct sensor_operate *ops = gyro_get_ops();
	int type = ops->type;

	sensor_unregister_slave(type, NULL, NULL, gyro_get_ops);
}

device_initcall_sync(gyro_mpu6500_init);
module_exit(gyro_mpu6500_exit);
