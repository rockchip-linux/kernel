// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2021 Rockchip Electronics Co., Ltd */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/acpi.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include "icm40605.h"

#define ICM40605_CHANNEL(_type, _axis, _index) {		\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = _axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |  \
		BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_BE,				\
	},							\
}

#define ICM40605_TEMP_CHANNEL(_type, _index) {			\
	.type = _type,						\
	.channel = -1,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 8,					\
		.storagebits = 8,				\
		.endianness = IIO_BE,				\
	},							\
}

const struct regmap_config icm40605_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};
EXPORT_SYMBOL(icm40605_regmap_config);

struct icm40605_regs {
	u8 data;
	u8 config;
	u8 config_odr_mask;
	u8 config_fsr_mask;
};

static struct icm40605_regs icm40605_regs[] = {
	[ICM40605_ACCEL] = {
		.data	= MPUREG_ACCEL_DATA_X0_UI,
		.config	= MPUREG_ACCEL_CONFIG0_REG,
		.config_odr_mask = BIT_ACCEL_ODR_NONFLAME_MASK,
		.config_fsr_mask = BIT_ACCEL_UI_FS_SEL_MASK,
	},
	[ICM40605_GYRO] = {
		.data	= MPUREG_GYRO_DATA_X0_UI,
		.config	= MPUREG_GYRO_CONFIG0_REG,
		.config_odr_mask = BIT_GYRO_ODR_NONFLAME_MASK,
		.config_fsr_mask = BIT_GYRO_UI_FS_SEL_MASK,
	},
	[ICM40605_TEMP] = {
		.data = MPUREG_TEMP_DATA0_UI,
		.config = MPUREG_GYRO_CONFIG1_REG,
		.config_odr_mask = 0,
		.config_fsr_mask = 0,
	},
};

struct icm40605_scale {
	u8 bits;
	int scale;
	int uscale;
};

struct icm40605_odr {
	u8 bits;
	int odr;
	int divider;
};

static const struct icm40605_scale icm40605_accel_scale[] = {
	{ ICM40605_ACCEL_FSR_2G, 0, 598},
	{ ICM40605_ACCEL_FSR_4G, 4, 1196},
	{ ICM40605_ACCEL_FSR_8G, 8, 2393},
	{ ICM40605_ACCEL_FSR_16G, 16, 4785},
};

static const struct icm40605_scale icm40605_gyro_scale[] = {
	{ ICM40605_GYRO_FSR_2000DPS, 0, 1064225},
	{ ICM40605_GYRO_FSR_1000DPS, 0, 532113},
	{ ICM40605_GYRO_FSR_500DPS, 0, 266462},
	{ ICM40605_GYRO_FSR_250DPS, 0, 133231},
	{ ICM40605_GYRO_FSR_125DPS, 0, 66615},
	{ ICM40605_GYRO_FSR_62_5DPS, 0, 33289},
	{ ICM40605_GYRO_FSR_31_25DPS, 0, 16644},
	{ ICM40605_GYRO_FSR_15_625DPS, 0, 8322},
};

struct icm40605_scale_item {
	const struct icm40605_scale *tbl;
	int num;
	int format;
};

static const struct  icm40605_scale_item icm40605_scale_table[] = {
	[ICM40605_ACCEL] = {
		.tbl	= icm40605_accel_scale,
		.num	= ARRAY_SIZE(icm40605_accel_scale),
		.format = IIO_VAL_INT_PLUS_MICRO,
	},
	[ICM40605_GYRO] = {
		.tbl	= icm40605_gyro_scale,
		.num	= ARRAY_SIZE(icm40605_gyro_scale),
		.format = IIO_VAL_INT_PLUS_NANO,
	},
};

static const struct icm40605_odr icm40605_accel_odr[] = {
	{ICM40605_ACCEL_ODR_8KHZ, 8000, 8},
	{ICM40605_ACCEL_ODR_4KHZ, 4000, 4},
	{ICM40605_ACCEL_ODR_2KHZ, 2000, 2},
	{ICM40605_ACCEL_ODR_1KHZ, 1000, 1},
	{ICM40605_ACCEL_ODR_200HZ, 200, -5},
	{ICM40605_ACCEL_ODR_100HZ, 100, -10},
	{ICM40605_ACCEL_ODR_50HZ, 50, -20},
	{ICM40605_ACCEL_ODR_25HZ, 25, -40},
};

static const struct icm40605_odr icm40605_gyro_odr[] = {
	{ICM40605_GYRO_ODR_8KHZ, 8000, 8},
	{ICM40605_GYRO_ODR_4KHZ, 4000, 4},
	{ICM40605_GYRO_ODR_2KHZ, 2000, 2},
	{ICM40605_GYRO_ODR_1KHZ, 1000, 1},
	{ICM40605_GYRO_ODR_200HZ, 200, -5},
	{ICM40605_GYRO_ODR_100HZ, 100, -10},
	{ICM40605_GYRO_ODR_50HZ, 50, -20},
	{ICM40605_GYRO_ODR_25HZ, 25, -40},
};

struct icm40605_odr_item {
	const struct icm40605_odr *tbl;
	int num;
};

static const struct  icm40605_odr_item icm40605_odr_table[] = {
	[ICM40605_ACCEL] = {
		.tbl	= icm40605_accel_odr,
		.num	= ARRAY_SIZE(icm40605_accel_odr),
	},
	[ICM40605_GYRO] = {
		.tbl	= icm40605_gyro_odr,
		.num	= ARRAY_SIZE(icm40605_gyro_odr),
	},
};

static const struct iio_chan_spec icm40605_channels[] = {
	ICM40605_CHANNEL(IIO_ACCEL, IIO_MOD_X, ICM40605_SCAN_ACCEL_X),
	ICM40605_CHANNEL(IIO_ACCEL, IIO_MOD_Y, ICM40605_SCAN_ACCEL_Y),
	ICM40605_CHANNEL(IIO_ACCEL, IIO_MOD_Z, ICM40605_SCAN_ACCEL_Z),
	ICM40605_CHANNEL(IIO_ANGL_VEL, IIO_MOD_X, ICM40605_SCAN_GYRO_X),
	ICM40605_CHANNEL(IIO_ANGL_VEL, IIO_MOD_Y, ICM40605_SCAN_GYRO_Y),
	ICM40605_CHANNEL(IIO_ANGL_VEL, IIO_MOD_Z, ICM40605_SCAN_GYRO_Z),
	ICM40605_TEMP_CHANNEL(IIO_TEMP, ICM40605_SCAN_TEMP),
	IIO_CHAN_SOFT_TIMESTAMP(ICM40605_SCAN_TIMESTAMP),
};

static const unsigned long icm40605_scan_masks[] = {
	/* 3-axis accel + temp */
	BIT(ICM40605_SCAN_ACCEL_X)
		| BIT(ICM40605_SCAN_ACCEL_Y)
		| BIT(ICM40605_SCAN_ACCEL_Z)
		| BIT(ICM40605_SCAN_TEMP),
	/* 3-axis gyro + temp */
	BIT(ICM40605_SCAN_GYRO_X)
		| BIT(ICM40605_SCAN_GYRO_Y)
		| BIT(ICM40605_SCAN_GYRO_Z)
		| BIT(ICM40605_SCAN_TEMP),
	/* 6-axis accel + gyro + temp */
	BIT(ICM40605_SCAN_ACCEL_X)
		| BIT(ICM40605_SCAN_ACCEL_Y)
		| BIT(ICM40605_SCAN_ACCEL_Z)
		| BIT(ICM40605_SCAN_GYRO_X)
		| BIT(ICM40605_SCAN_GYRO_Y)
		| BIT(ICM40605_SCAN_GYRO_Z)
		| BIT(ICM40605_SCAN_TEMP),
	0,
};

static int icm40605_chip_init(struct icm40605_data *data, bool use_spi);

static enum icm40605_sensor_type icm40605_to_sensor(enum iio_chan_type iio_type)
{
	switch (iio_type) {
	case IIO_ACCEL:
		return ICM40605_ACCEL;
	case IIO_ANGL_VEL:
		return ICM40605_GYRO;
	case IIO_TEMP:
		return ICM40605_TEMP;
	default:
		return -EINVAL;
	}
}

static
int icm40605_gyro_on(struct icm40605_data *data)
{
	int ret;
	unsigned int cmd;

	// IGNORE:Gyro Workaround 9396
	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, 0x04);
	if (ret < 0)
		return ret;
	ret = regmap_write(data->regmap, 0x10, 0x0d);
	if (ret < 0)
		return ret;
	ret = regmap_write(data->regmap, 0x12, 0x01);
	if (ret < 0)
		return ret;

	// Enable Gyro
	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, ICM40605_BANK0);
	if (ret < 0)
		return ret;
	ret = regmap_read(data->regmap, MPUREG_PWR_MGMT_0_REG, &cmd);
	if (ret < 0)
		return ret;
	cmd |= (BIT_GYRO_MODE_LN);  // Gyro on in LNM
	usleep_range(50, 51);
	ret = regmap_write(data->regmap, MPUREG_PWR_MGMT_0_REG, cmd);
	if (ret < 0)
		return ret;
	usleep_range(200, 201); //workaround 9136
	ret = regmap_read(data->regmap, MPUREG_PWR_MGMT_0_REG, &cmd);
	if (ret < 0)
		return ret;
	// IGNORE: Gyro Workaround 9396
	msleep(20);
	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, 0x04);
	if (ret < 0)
		return ret;
	ret = regmap_write(data->regmap, 0x10, 0x2a); //set 42 to reg 0x10
	if (ret < 0)
		return ret;
	msleep(100);
	return ret;
}

static
int icm40605_gyro_off(struct icm40605_data *data)
{
	int ret;
	unsigned int cmd;

	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, ICM40605_BANK0);
	if (ret < 0)
		return ret;
	ret = regmap_read(data->regmap, MPUREG_PWR_MGMT_0_REG, &cmd);
	if (ret < 0)
		return ret;
	cmd &= (~BIT_GYRO_MODE_LN);
	msleep(80);
	ret = regmap_write(data->regmap, MPUREG_PWR_MGMT_0_REG, cmd);
	if (ret < 0)
		return ret;
	msleep(200);
	return ret;
}

static
int icm40605_accel_on(struct icm40605_data *data)
{
	int ret;
	unsigned int cmd;

	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, ICM40605_BANK0);
	if (ret < 0)
		return ret;
	ret = regmap_read(data->regmap, MPUREG_PWR_MGMT_0_REG, &cmd);
	if (ret < 0)
		return ret;
	cmd |= (BIT_ACCEL_MODE_LN);      // Accel on in LNM
	ret = regmap_write(data->regmap, MPUREG_PWR_MGMT_0_REG, cmd);
	if (ret < 0)
		return ret;
	usleep_range(200, 201);

	return ret;
}

static
int icm40605_accel_off(struct icm40605_data *data)
{
	int ret;
	unsigned int cmd;

	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, ICM40605_BANK0);
	if (ret < 0)
		return ret;
	ret = regmap_read(data->regmap, MPUREG_PWR_MGMT_0_REG, &cmd);
	if (ret < 0)
		return ret;
	cmd &= (~BIT_ACCEL_MODE_LN);      // Accel on in LNM
	usleep_range(50, 51);
	ret = regmap_write(data->regmap, MPUREG_PWR_MGMT_0_REG, cmd);
	if (ret < 0)
		return ret;
	usleep_range(200, 201);
	return ret;
}

static
int icm40605_temp_on(struct icm40605_data *data)
{
	int ret;
	unsigned int cmd;

	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, ICM40605_BANK0);
	if (ret)
		return ret;
	ret = regmap_read(data->regmap, MPUREG_PWR_MGMT_0_REG, &cmd);
	if (ret)
		return ret;
	cmd |= (BIT_TEMP_DIS);
	ret = regmap_write(data->regmap, MPUREG_PWR_MGMT_0_REG, cmd);
	if (ret < 0)
		return ret;
	usleep_range(200, 250);

	return ret;
}

static
int icm40605_temp_off(struct icm40605_data *data)
{
	int ret;
	unsigned int cmd;

	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, ICM40605_BANK0);
	if (ret < 0)
		return ret;
	ret = regmap_read(data->regmap, MPUREG_PWR_MGMT_0_REG, &cmd);
	if (ret < 0)
		return ret;
	cmd &= (~BIT_TEMP_DIS);
	ret = regmap_write(data->regmap, MPUREG_PWR_MGMT_0_REG, cmd);
	if (ret < 0)
		return ret;
	usleep_range(200, 250);
	return ret;
}

int icm40605_set_mode(struct icm40605_data *data,
		      enum icm40605_sensor_type t,
		      bool mode)
{
	int ret;

	if (t == ICM40605_GYRO) {
		if (mode)
			ret = icm40605_gyro_on(data);
		else
			ret = icm40605_gyro_off(data);
	} else if (t == ICM40605_ACCEL) {
		if (mode)
			ret = icm40605_accel_on(data);
		else
			ret = icm40605_accel_off(data);
	} else if (t == ICM40605_TEMP) {
		if (mode)
			ret = icm40605_temp_on(data);
		else
			ret = icm40605_temp_off(data);
	} else {
		return -EINVAL;
	}

	return ret;
}

static
int __icm40605_set_odr(struct icm40605_data *data,
		       enum icm40605_sensor_type t,
		       int sel, int divider)
{
	int ret;
	unsigned int cmd;

	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, ICM40605_BANK0);
	if (ret < 0)
		return ret;
	if (t == ICM40605_ACCEL) {
		ret = regmap_read(data->regmap, MPUREG_ACCEL_CONFIG0_REG, &cmd);
		if (ret < 0)
			return ret;
		if (!((cmd & BIT_GYRO_ODR_NONFLAME_MASK) & (~sel)))
			return 0;

		cmd &= (~BIT_GYRO_ODR_NONFLAME_MASK);
		cmd |= sel;
		ret = regmap_write(data->regmap, MPUREG_ACCEL_CONFIG0_REG, cmd);
		if (!ret) {
			data->chip_config.accel_odr = sel;
			data->period_divider = (data->period_divider &&
			data->period_divider > divider) ?
			data->period_divider : divider;
		}
	} else if (t == ICM40605_GYRO) {
		ret = regmap_read(data->regmap, MPUREG_GYRO_CONFIG0_REG, &cmd);
		if (ret < 0)
			return ret;
		if (!((cmd & BIT_ACCEL_ODR_NONFLAME_MASK) & (~sel)))
			return 0;
		cmd &= (~BIT_ACCEL_ODR_NONFLAME_MASK);
		cmd |= sel;
		ret = regmap_write(data->regmap, MPUREG_GYRO_CONFIG0_REG, cmd);
		if (!ret) {
			data->chip_config.gyro_odr = sel;
			data->period_divider = (data->period_divider &&
			data->period_divider > divider) ?
			data->period_divider : divider;
		}
	} else {
		return -EINVAL;
	}

	return ret;
}

static
int __icm40605_set_fsr(struct icm40605_data *data, enum icm40605_sensor_type t, int sel)
{
	int ret;
	unsigned int cmd;

	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, ICM40605_BANK0);
	if (ret < 0)
		return ret;
	if (t == ICM40605_ACCEL) {
		ret = regmap_read(data->regmap, MPUREG_ACCEL_CONFIG0_REG, &cmd);
		if (ret < 0)
			return ret;
		cmd &= (~BIT_ACCEL_UI_FS_SEL_MASK);
		cmd |= sel;
		ret = regmap_write(data->regmap, MPUREG_ACCEL_CONFIG0_REG, cmd);
	} else if (t == ICM40605_GYRO) {
		ret = regmap_read(data->regmap, MPUREG_GYRO_CONFIG0_REG, &cmd);
		if (ret < 0)
			return ret;
		cmd &= (~BIT_GYRO_UI_FS_SEL_MASK);
		cmd |= sel;
		ret = regmap_write(data->regmap, MPUREG_GYRO_CONFIG0_REG, cmd);
	} else {
		return -EINVAL;
	}

	return ret;
}

static
int icm40605_set_scale(struct icm40605_data *data,
		       enum icm40605_sensor_type t,
		       int scale, int uscale)
{
	int i;

	if (t >= ARRAY_SIZE(icm40605_scale_table))
		return 0;

	for (i = 0; i < icm40605_scale_table[t].num; i++)
		if (icm40605_scale_table[t].tbl[i].uscale == uscale &&
		    icm40605_scale_table[t].tbl[i].scale == scale)
			break;

	if (i == icm40605_scale_table[t].num)
		return -EINVAL;

	dev_info(regmap_get_device(data->regmap), "set scale t: %d, bit: %x\n",
		 t, icm40605_scale_table[t].tbl[i].bits);
	return __icm40605_set_fsr(data, t, icm40605_scale_table[t].tbl[i].bits);
}

static
int icm40605_get_scale(struct icm40605_data *data,
		       enum icm40605_sensor_type t,
		       int *scale, int *uscale)
{
	int i, ret, val;

	if (t < 0 || (t >= ARRAY_SIZE(icm40605_scale_table)))
		return -EINVAL;

	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, ICM40605_BANK0);
	if (ret < 0)
		return ret;

	ret = regmap_read(data->regmap, icm40605_regs[t].config, &val);
	if (ret < 0)
		return ret;

	val &= icm40605_regs[t].config_fsr_mask;
	for (i = 0; i < icm40605_scale_table[t].num; i++)
		if (icm40605_scale_table[t].tbl[i].bits == val) {

			*scale = icm40605_scale_table[t].tbl[i].scale;
			*uscale = icm40605_scale_table[t].tbl[i].uscale;

			return icm40605_scale_table[t].format;
		}

	return -EINVAL;
}

static
int icm40605_get_data(struct icm40605_data *data,
		      int chan_type,
		      int axis, int *val)
{
	u8 reg;
	int ret;
	__be16 sample;
	int t = icm40605_to_sensor(chan_type);

	// TODO: check type
	if (t < 0) {
		*val = -1;
		return 0;
	}

	if (t == ICM40605_TEMP)
		reg = icm40605_regs[t].data;
	else
		reg = icm40605_regs[t].data + (axis - IIO_MOD_X) * sizeof(sample);

	do {
		ret = regmap_bulk_read(data->regmap, reg, (u8 *)&sample, 2);
		if (ret) {
			ret = -EINVAL;
			break;
		}

		*val = (short)be16_to_cpup(&sample);
		dev_info(regmap_get_device(data->regmap),
			 "icm40605get_data: ch: %d, aix: %d, reg: %x, val: %x\n",
			 chan_type, axis, sample, *val);
	} while (0);

	return ret;
}

static
int icm40605_set_odr(struct icm40605_data *data,
		     enum icm40605_sensor_type t,
		     int odr)
{
	int i;

	if (t == ICM40605_ACCEL)
		data->accel_frequency = odr;

	if (t == ICM40605_GYRO)
		data->gyro_frequency = odr;

	if (t >= ARRAY_SIZE(icm40605_odr_table))
		return 0;

	for (i = 0; i < icm40605_odr_table[t].num; i++)
		if (icm40605_odr_table[t].tbl[i].odr == odr)
			break;

	if (i >= icm40605_odr_table[t].num)
		return -EINVAL;

	dev_info(regmap_get_device(data->regmap), "set odr t: %d, bit: %x\n", t,
		 icm40605_odr_table[t].tbl[i].bits);
	return __icm40605_set_odr(data, t, icm40605_odr_table[t].tbl[i].bits,
				  icm40605_odr_table[t].tbl[i].divider);
}

static int icm40605_get_odr(struct icm40605_data *data,
			    enum icm40605_sensor_type t,
			    int *odr)
{
	int i, val, ret;

	if (t < 0 || t >= ARRAY_SIZE(icm40605_odr_table))
		return 0;

	ret = regmap_read(data->regmap, icm40605_regs[t].config, &val);
	if (ret < 0)
		return ret;

	val &= icm40605_regs[t].config_odr_mask;

	for (i = 0; i < icm40605_odr_table[t].num; i++)
		if (val == icm40605_odr_table[t].tbl[i].bits)
			break;

	if (i >= icm40605_odr_table[t].num)
		return -EINVAL;

	*odr = icm40605_odr_table[t].tbl[i].odr;

	return 0;
}

static int icm40605_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	int ret;
	struct icm40605_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		mutex_lock(&data->lock);
		icm40605_get_data(data, chan->type, chan->channel2, val);
		mutex_unlock(&data->lock);
		iio_device_release_direct_mode(indio_dev);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&data->lock);
		ret = icm40605_get_scale(data,
					icm40605_to_sensor(chan->type),
					val,
					val2);
		mutex_unlock(&data->lock);
		return ret;
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&data->lock);
		ret = icm40605_get_odr(data,
				       icm40605_to_sensor(chan->type),
				       val);
		mutex_unlock(&data->lock);
		return ret < 0 ? ret : IIO_VAL_INT;
	default:
		return -EINVAL;
	}

	return 0;
}

static int icm40605_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	int ret = 0;
	struct icm40605_data *data = iio_priv(indio_dev);

	mutex_lock(&data->lock);
	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		goto error_write_raw_unlock;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		ret = icm40605_set_scale(data,
					 icm40605_to_sensor(chan->type),
					 val,
					 val2);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = icm40605_set_odr(data,
				       icm40605_to_sensor(chan->type),
				       val);
		break;
	default:
		ret = -EINVAL;
	}

error_write_raw_unlock:
	iio_device_release_direct_mode(indio_dev);
	mutex_unlock(&data->lock);
	return ret;
}

static int icm40605_write_raw_get_fmt(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			return IIO_VAL_INT_PLUS_NANO;
		default:
			return IIO_VAL_INT_PLUS_MICRO;
		}
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

static
IIO_CONST_ATTR(in_accel_sampling_frequency_available,
	       "25 50 100 200 1000 2000 4000 8000");
static
IIO_CONST_ATTR(in_anglvel_sampling_frequency_available,
	       "25 50 100 200 1000 2000 4000 8000");
static
IIO_CONST_ATTR(in_accel_scale_available,
	       "0.000598 0.001196 0.002393 0.004785");
static
IIO_CONST_ATTR(in_anglvel_scale_available,
	       "0.001064225 0.000532113 0.000266462 0.000133231 0.000066615, 0.000033289, 0.000016644, 0.000008322");

static struct attribute *icm40605_attrs[] = {
	&iio_const_attr_in_accel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_accel_scale_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group icm40605_attrs_group = {
	.attrs = icm40605_attrs,
};

static const struct iio_info icm40605_info = {
	.read_raw = icm40605_read_raw,
	.write_raw = icm40605_write_raw,
	.write_raw_get_fmt = &icm40605_write_raw_get_fmt,
	.attrs = &icm40605_attrs_group,
};

static const char *icm40605_match_acpi_device(struct device *dev)
{
	const struct acpi_device_id *id;

	id = acpi_match_device(dev->driver->acpi_match_table, dev);
	if (!id)
		return NULL;

	return dev_name(dev);
}

static void icm40605_disable_vdd_reg(void *_data)
{
	struct icm40605_data *st = _data;
	const struct device *dev = regmap_get_device(st->regmap);
	int ret;

	ret = regulator_disable(st->vdd_supply);
	if (ret)
		dev_err(dev, "failed to disable vdd error %d\n", ret);
}

static int icm40605_chip_init(struct icm40605_data *data, bool use_spi)
{
	int ret;
	unsigned int regval;
	struct device *dev = regmap_get_device(data->regmap);
	// who am i
	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, ICM40605_BANK0);
	if (ret)
		return ret;
	ret = regmap_read(data->regmap, MPUREG_WHO_AM_I, &regval);
	if (ret)
		return ret;
	dev_info(dev, "i am: %x, 40605: %x\n", regval, BIT_I_AM_ICM40605);
	if (regval != BIT_I_AM_ICM40605)
		return -EINVAL;

	ret = gpio_direction_output(data->int1_gpio, 0);
	if (ret < 0)
		dev_err(dev, "can't set gpio!\r\n");
	// reset
	ret = regmap_write(data->regmap, MPUREG_CHIP_CONFIG_REG, BIT_SOFT_RESET_CHIP_CONFIG);
	if (ret < 0)
		return ret;

	usleep_range(20000, 21000);

	ret = gpio_direction_input(data->int1_gpio);
	if (ret < 0)
		dev_err(dev, "gpio_direction_input  failed!\r\n");

	if (use_spi) {
		regval &= (~BIT_SPI_I2C_SEL_MASK);
		regval |= BIT_SEL_I2C_DISABLE;
		regval |= BIT_FIFO_COUNT_REC;
		ret = regmap_write(data->regmap, MPUREG_INTF_CONFIG0_REG, regval);
		if (ret < 0)
			return ret;
	} else {
		regval &= (~BIT_SPI_I2C_SEL_MASK);
		regval |= BIT_SEL_SPI_DISABLE;
		ret = regmap_write(data->regmap, MPUREG_INTF_CONFIG0_REG, regval);
		if (ret < 0)
			return ret;
	}

	ret = regmap_write(data->regmap, MPUREG_DRIVE_CONFIG_REG, BIT_SPI_SPEED_5M);
	if (ret)
		return ret;
	// TODO: test spi link
	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, ICM40605_BANK0);
	if (ret < 0)
		return ret;

	ret = regmap_read(data->regmap, MPUREG_WHO_AM_I, &regval);
	if (ret)
		return ret;
	dev_info(dev, "after reset i am: %x, 40605: %x\n", regval, BIT_I_AM_ICM40605);
	if (regval != BIT_I_AM_ICM40605)
		return -EINVAL;

	ret = icm40605_set_mode(data, ICM40605_ACCEL, true);
	if (ret < 0)
		return ret;

	ret = icm40605_set_mode(data, ICM40605_GYRO, true);
	if (ret < 0)
		return ret;

	// init gyro and accel para
	icm40605_set_odr(data, ICM40605_ACCEL, 1000);
	icm40605_set_odr(data, ICM40605_GYRO, 1000);
	__icm40605_set_fsr(data, ICM40605_ACCEL, ICM40605_ACCEL_FSR_2G);
	__icm40605_set_fsr(data, ICM40605_GYRO, ICM40605_GYRO_FSR_2000DPS);

	ret = regmap_write(data->regmap, MPUREG_INT_SOURCE0_REG, 0x0);
	if (ret < 0)
		return ret;
	// set int mode
	ret = regmap_write(data->regmap, MPUREG_INT_CONFIG_REG, data->irq_mask);
	if (ret < 0)
		return ret;

	dev_info(dev, "icm40605chip_init out\n");

	return ret;
}

static void icm40605_chip_uninit(struct icm40605_data *data)
{
	icm40605_set_mode(data, ICM40605_GYRO, false);
	icm40605_set_mode(data, ICM40605_ACCEL, false);
}

int icm40605_core_probe(struct regmap *regmap,
			int irq, const char *name,
			int chip_type, bool use_spi)
{
	struct iio_dev *indio_dev;
	struct icm40605_data *data;
	struct device *dev = regmap_get_device(regmap);
	int ret;
	struct irq_data *desc;
	int irq_type;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	memset(data, 0, sizeof(struct icm40605_data));
	mutex_init(&data->lock);
	data->chip_type = chip_type;
	data->powerup_count = 0;
	data->irq = irq;
	data->regmap = regmap;
	/* get the node */
	data->node = of_find_node_by_name(NULL, "icm40605");
	if (data->node == NULL)
		dev_err(dev, "spi node not find!\n");

	data->int1_gpio = of_get_named_gpio(data->node, "int1-gpio", 0);
	if (data->int1_gpio < 0) {
		dev_err(dev, "Could not get int1_gpio!\n");
		return -EINVAL;
	}

	desc = irq_get_irq_data(irq);
	if (!desc) {
		dev_err(dev, "Could not find IRQ %d\n", irq);
		return -EINVAL;
	}

	irq_type = irqd_get_trigger_type(desc);
	if (!irq_type)
		irq_type = IRQF_TRIGGER_RISING;
	if (irq_type == IRQF_TRIGGER_RISING)
		data->irq_mask = BIT_ONLY_INT1_ACTIVE_HIGH;
	else if (irq_type == IRQF_TRIGGER_FALLING)
		data->irq_mask = BIT_ONLY_INT1_ACTIVE_LOW;
	else {
		dev_err(dev, "Invalid interrupt type 0x%x specified\n",
			irq_type);
		return -EINVAL;
	}

	data->vdd_supply = devm_regulator_get(dev, "vcc_avdd");
	if (IS_ERR(data->vdd_supply))
		return PTR_ERR(data->vdd_supply);

	ret = regulator_enable(data->vdd_supply);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, icm40605_disable_vdd_reg, data);
	if (ret)
		return ret;

	ret = icm40605_chip_init(data, use_spi);
	if (ret < 0) {
		dev_err(dev, "icm40605_chip_init fail\n");
		return ret;
	}

	dev_set_drvdata(dev, indio_dev);
	if (!name && ACPI_HANDLE(dev))
		name = icm40605_match_acpi_device(dev);

	indio_dev->dev.parent = dev;
	indio_dev->channels = icm40605_channels;
	indio_dev->num_channels = ARRAY_SIZE(icm40605_channels);
	indio_dev->available_scan_masks = icm40605_scan_masks;
	indio_dev->name = name;
	indio_dev->info = &icm40605_info;
	indio_dev->modes = INDIO_BUFFER_TRIGGERED;

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev, iio_pollfunc_store_time,
					      icm40605_read_fifo, NULL);
	if (ret < 0)
		goto uninit;

	ret = icm40605_probe_trigger(indio_dev, irq_type);
	if (ret) {
		dev_err(dev, "trigger probe fail %d\n", ret);
		return ret;
	}
	ret = devm_iio_device_register(dev, indio_dev);

	if (ret < 0) {
		dev_err(dev, "devm_iio_device_register fail\n");
		goto uninit;
	}

	ret = pm_runtime_set_active(dev);

	if (ret)
		return ret;

	return 0;
uninit:
	icm40605_chip_uninit(data);
	return ret;
}
EXPORT_SYMBOL_GPL(icm40605_core_probe);
/*
 * System resume gets the system back on and restores the sensors state.
 * Manually put runtime power management in system active state.
 */
static int __maybe_unused sleep_icm40605_resume(struct device *dev)
{
	struct icm40605_data *st =  iio_priv(dev_get_drvdata(dev));
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&st->lock);

	ret = regulator_enable(st->vdd_supply);
	if (ret)
		goto out_unlock;

	usleep_range(3000, 4000);

	pm_runtime_disable(dev);

	pm_runtime_set_active(dev);

	pm_runtime_enable(dev);

	icm40605_set_odr(st, ICM40605_GYRO, st->gyro_frequency_buff);

	icm40605_set_odr(st, ICM40605_ACCEL, st->accel_frequency_buff);

	ret = icm40605_set_enable(indio_dev, 1);

	if (ret)
		goto out_unlock;

	devm_regulator_put(st->vdd_supply);
out_unlock:
	mutex_unlock(&st->lock);
	return ret;
}

/*
 * Suspend saves sensors state and turns everything off.
 * Check first if runtime suspend has not already done the job.
 */
static int __maybe_unused sleep_icm40605_suspend(struct device *dev)
{
	struct icm40605_data *st =  iio_priv(dev_get_drvdata(dev));
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&st->lock);

	st->gyro_frequency_buff = st->gyro_frequency;

	st->accel_frequency_buff = st->accel_frequency;

	if (pm_runtime_suspended(dev)) {
		ret = 0;
		goto out_unlock;
	}

	ret = icm40605_set_enable(indio_dev, 0);
	if (ret)
		goto out_unlock;

	regulator_disable(st->vdd_supply);

	devm_regulator_put(st->vdd_supply);

out_unlock:
	mutex_unlock(&st->lock);
	return ret;

}

void icm40605_core_remove(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct icm40605_data *data = iio_priv(indio_dev);

	icm40605_chip_uninit(data);
}
EXPORT_SYMBOL_GPL(icm40605_core_remove);

const struct dev_pm_ops inv_icm42600_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sleep_icm40605_suspend, sleep_icm40605_resume)
};
EXPORT_SYMBOL_GPL(inv_icm42600_pm_ops);

MODULE_AUTHOR("ALLEN CHEN");
MODULE_DESCRIPTION("Invensense ICM40605 SPI driver");
MODULE_LICENSE("GPL v2");
