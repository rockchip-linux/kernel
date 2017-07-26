/*
 *  drivers/misc/mpu6500/spi-mpu6500.h
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
#ifndef SPI_MPU6500_H
#define SPI_MPU6500_H

struct sensor_axis {
	int x;
	int y;
	int z;
};

struct raw_data {
	struct sensor_axis gyro_axis;
	struct sensor_axis accel_axis;
	long long timestamp_usec;
	short temperature;
	int quaternion[4];
	int quat_flag;
	int use_irq;
};

struct raw_data_set {
	struct raw_data sensordata[200];
	int avail_num;
};

#define GYROSENSOR_IOCTL_MAGIC		'f'
#define GYROSENSOR_IOCTL_CLOSE		_IO(GYROSENSOR_IOCTL_MAGIC, 0x00)
#define GYROSENSOR_IOCTL_START		_IO(GYROSENSOR_IOCTL_MAGIC, 0x01)
#define GYROSENSOR_IOCTL_RESET		_IO(GYROSENSOR_IOCTL_MAGIC, 0x02)
#define GYROSENSOR_IOCTL_GETDATA \
_IOWR(GYROSENSOR_IOCTL_MAGIC, 0x03, struct raw_data_set)
#define GYROSENSOR_IOCTL_CALIBRATION \
_IOWR(GYROSENSOR_IOCTL_MAGIC, 0x04, char *)
#define GYROSENSOR_IOCTL_GETDATA_FIFO \
_IOWR(GYROSENSOR_IOCTL_MAGIC, 0x05, struct raw_data_set)

#define SENSOR_ON               1
#define SENSOR_OFF              0

#endif
