// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2021 Rockchip Electronics Co., Ltd */
#include "icm40605.h"
#include <linux/math64.h>
/**
 *  icm40605_set_enable() - enable chip functions.
 *  @indio_dev:	Device driver instance.
 *  @enable: enable/disable
 */
int icm40605_set_enable(struct iio_dev *indio_dev, bool enable)
{
	int ret;
	struct icm40605_data *data = iio_priv(indio_dev);

	ret = regmap_write(data->regmap, MPUREG_REG_BANK_SEL, ICM40605_BANK0);
	if (ret) {
		dev_err(regmap_get_device(data->regmap),
			"sel_bank0 fail: %d\n",
			ret);
		goto error_off;
	}
	if (enable) {
		ret = icm40605_set_mode(data, ICM40605_ACCEL, true);
		if (ret)
			goto error_off;

		ret = icm40605_set_mode(data, ICM40605_GYRO, true);
		if (ret)
			goto error_accl_off;

		ret = icm40605_reset_fifo(indio_dev);
		if (ret)
			goto error_gyro_off;

		data->standard_period = data->period_divider > 0 ?
			div_s64(NSEC_PER_MSEC, data->period_divider) :
			NSEC_PER_MSEC * data->period_divider * (-1);
		data->interrupt_period = data->standard_period;
		data->period_min = div_s64((data->standard_period * (100 -
					    INV_MPU6050_TS_PERIOD_JITTER)),
					    100);
		data->period_max = div_s64((data->standard_period * (100 +
					    INV_MPU6050_TS_PERIOD_JITTER)),
					    100);
	} else {
		ret = regmap_write(data->regmap, MPUREG_FIFO_CONFIG_REG, BIT_FIFO_MODE_CTRL_BYPASS);
		if (ret) {
			dev_err(regmap_get_device(data->regmap), "set MPUREG_FIFO_CONFIG_REG fail: %d\n",
				ret);
			goto error_gyro_off;
		}
		ret = icm40605_set_mode(data, ICM40605_GYRO, false);
		if (ret)
			goto error_gyro_off;

		ret = icm40605_set_mode(data, ICM40605_ACCEL, false);
		if (ret)
			goto error_accl_off;
	}
	dev_info(regmap_get_device(data->regmap), "set fifo mode end\n");
	return ret;

error_gyro_off:
	icm40605_set_mode(data, ICM40605_GYRO, false);
error_accl_off:
	icm40605_set_mode(data, ICM40605_ACCEL, false);
error_off:
	return ret;
}

/**
 * inv_mpu_data_rdy_trigger_set_state() - set data ready interrupt state
 * @trig: Trigger instance
 * @state: Desired trigger state
 */
static int inv_mpu_data_rdy_trigger_set_state(struct iio_trigger *trig,
					      bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct icm40605_data *data = iio_priv(indio_dev);
	int result;

	mutex_lock(&data->lock);
	dev_info(regmap_get_device(data->regmap), "in data_rdy_trigger_set_state, %d\n", state);
	result = icm40605_set_enable(indio_dev, state);
	mutex_unlock(&data->lock);

	return result;
}

static const struct iio_trigger_ops inv_mpu_trigger_ops = {
	.set_trigger_state = &inv_mpu_data_rdy_trigger_set_state,
};

int icm40605_probe_trigger(struct iio_dev *indio_dev, int irq_type)
{
	int ret;
	struct icm40605_data *data = iio_priv(indio_dev);

	data->trig = devm_iio_trigger_alloc(&indio_dev->dev,
					    "%s-dev%d",
					    indio_dev->name,
					    indio_dev->id);
	if (!data->trig)
		return -ENOMEM;

	ret = devm_request_irq(&indio_dev->dev, data->irq,
			       &iio_trigger_generic_data_rdy_poll,
			       irq_type,
			       "inv_mpu",
			       data->trig);
	if (ret)
		return ret;

	data->trig->dev.parent = regmap_get_device(data->regmap);
	data->trig->ops = &inv_mpu_trigger_ops;
	iio_trigger_set_drvdata(data->trig, indio_dev);

	ret = devm_iio_trigger_register(&indio_dev->dev, data->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(data->trig);

	return 0;
}
