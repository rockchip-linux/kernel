// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2021 Rockchip Electronics Co., Ltd */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/math64.h>
#include <asm/unaligned.h>
#include "icm40605.h"

/**
 *  icm40605_update_period() - Update chip internal period estimation
 *
 *  @st:		driver state
 *  @timestamp:		the interrupt timestamp
 *  @nb:		number of data set in the fifo
 *
 *  This function uses interrupt timestamps to estimate the chip period and
 *  to choose the data timestamp to come.
 */
static void icm40605_update_period(struct icm40605_data *st,
				   s64 timestamp, size_t nb)
{
	s64 interval;

	if (st->it_timestamp != 0)
		st->interrupt_period = div_s64((timestamp - st->it_timestamp), nb);

	interval = (nb - 1) * st->interrupt_period;
	st->data_timestamp = timestamp - interval;
	/* save it timestamp */
	st->it_timestamp = timestamp;
}

/**
 *  icm40605_get_timestamp() - Return the current data timestamp
 *
 *  @st:		driver state
 *  @return:		current data timestamp
 *
 *  This function returns the current data timestamp and prepares for next one.
 */
static s64 icm40605_get_timestamp(struct icm40605_data *st)
{
	s64 ts;

	/* return current data timestamp and increment */
	ts = st->data_timestamp;
	st->data_timestamp += st->interrupt_period;

	return ts;
}

int icm40605_reset_fifo(struct iio_dev *indio_dev)
{
	int ret;
	struct icm40605_data  *st = iio_priv(indio_dev);

	/* reset it timestamp validation */
	st->it_timestamp = 0;
	ret = regmap_write(st->regmap, MPUREG_REG_BANK_SEL, ICM40605_BANK0);
	if (ret) {
		dev_err(regmap_get_device(st->regmap), "sel_bank0 fail: %d\n",
			ret);
		return ret;
	}
	/* disable interrupt */
	ret = regmap_write(st->regmap, MPUREG_INT_SOURCE0_REG, 0x0);
	if (ret) {
		dev_err(regmap_get_device(st->regmap), "int_enable failed %d\n",
			ret);
		goto reset_fifo_fail;
	}
	/* disable the sensor output to FIFO */
	ret = regmap_write(st->regmap, MPUREG_FIFO_CONFIG_REG, 0x0);
	if (ret)
		goto reset_fifo_fail;

	/* reset FIFO*/
	ret = regmap_write(st->regmap, MPUREG_SIGNAL_PATH_RESET_REG, BIT_FIFO_FLUSH);
	if (ret)
		goto reset_fifo_fail;

	/* enable sensor output to FIFO */
	ret = regmap_write(st->regmap, MPUREG_FIFO_CONFIG_REG, 0x80);
	if (ret)
		goto reset_fifo_fail;

	ret = regmap_write(st->regmap, MPUREG_TMST_CONFIG_REG, 0x31);
	if (ret) {
		dev_err(regmap_get_device(st->regmap), "set MPUREG_TMST_CONFIG_REG fail: %d, val: 0x39\n",
			ret);
		goto reset_fifo_fail;
	}

	ret = regmap_write(st->regmap, MPUREG_FIFO_CONFIG2_REG, BIT_FIFO_WM5);
	if (ret)
		goto reset_fifo_fail;

	/* enable interrupt */
	ret = regmap_write(st->regmap, MPUREG_INT_SOURCE0_REG,
			   BIT_INT_RESET_DONE_INT1_EN|BIT_INT_FIFO_THS_INT1_EN);
	if (ret)
		goto reset_fifo_fail;

	ret = regmap_write(st->regmap, MPUREG_FIFO_CONFIG1_REG, 0xf);
	if (ret)
		goto reset_fifo_fail;

	ret = regmap_write(st->regmap, MPUREG_INT_CONFIG1_REG, 0x10);
	if (ret)
		goto reset_fifo_fail;

	st->interrupt_period = st->standard_period;
	return 0;

reset_fifo_fail:
	dev_err(regmap_get_device(st->regmap), "%s :reset fifo failed %d\n", __func__, ret);
	ret = regmap_write(st->regmap, MPUREG_INT_SOURCE0_REG,
			   BIT_INT_RESET_DONE_INT1_EN|
			   BIT_INT_FIFO_THS_INT1_EN);

	return ret;
}


/**
 * icm40605_read_fifo() - Transfer data from hardware FIFO to KFIFO.
 */
irqreturn_t icm40605_read_fifo(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct icm40605_data *st = iio_priv(indio_dev);
	size_t bytes_per_datum;
	int result, int_status, interval_time;
	u8 data[ICM40605_OUTPUT_DATA_SIZE_PULS_ONE], i;
	u16 fifo_count;
	u32 data_len;
	s64 timestamp = 0;

	mutex_lock(&st->lock);

	result = regmap_read(st->regmap, MPUREG_GYRO_CONFIG0_REG, &interval_time);
	if (result) {
		dev_err(regmap_get_device(st->regmap),
			"failed get interval_time\n");
		goto flush_fifo;
	}

	switch (interval_time & 0x0f) {
	case 0xA:
		interval_time = 40000;
		break;
	case 0x9:
		interval_time = 20000;
		break;
	case 0x8:
		interval_time = 10000;
		break;
	case 0x7:
		interval_time = 5000;
		break;
	case 0x6:
		interval_time = 1000;
		break;
	case 0x5:
		interval_time = 500;
		break;
	case 0x4:
		interval_time = 250;
		break;
	case 0x3:
		interval_time = 125;
		break;
	default:
		interval_time = 5000;
	}

	/* ack interrupt and check status */
	result = regmap_read(st->regmap, MPUREG_INT_STATUS, &int_status);
	if (result) {
		dev_err(regmap_get_device(st->regmap),
			"failed to ack interrupt\n");
		goto flush_fifo;
	}

	if (!(int_status & BIT_STATUS_FIFO_THS)) {
		dev_warn(regmap_get_device(st->regmap),
			 "spurious interrupt with status 0x%x\n",
			 int_status);
		if (!(int_status & (BIT_STATUS_DRDY | BIT_STATUS_FIFO_THS)))
			goto flush_fifo;
	}

	if ((int_status & BIT_STATUS_FIFO_FULL)) {
		dev_warn(regmap_get_device(st->regmap), "the fifo is full\n");
		goto flush_fifo;
	}

	bytes_per_datum = ICM40605_FIFO_DATUM;
	result = regmap_bulk_read(st->regmap, MPUREG_FIFO_BYTE_COUNT1_REG,
				  data, ICM40605_FIFO_COUNT_BYTE);
	if (result) {
		dev_err(regmap_get_device(st->regmap), "read fifo count fail: %d\n", result);
		goto end_session;
	}

	fifo_count = (data[0] << 8) | data[1];
	if (fifo_count > ICM40605_FIFO_COUNT_LIMIT) {
		dev_warn(regmap_get_device(st->regmap),
			 "fifo overflow reset, cnt: %u\n",
			 fifo_count);
		goto flush_fifo;
	}


	icm40605_update_period(st, pf->timestamp, fifo_count);

	data_len = bytes_per_datum * fifo_count;
	if (data_len > ICM40605_FIFO_COUNT_LIMIT)
		data_len = ICM40605_FIFO_COUNT_LIMIT;

	result = regmap_bulk_read(st->regmap, MPUREG_FIFO_DATA_REG,
				  st->data_buff, data_len);
	if (result) {
		dev_err(regmap_get_device(st->regmap),
			"regmap_bulk_read failed\n");
		goto flush_fifo;
	}

	for (i = 0; i < fifo_count; ++i) {
		/* skip first samples if needed */
		if (st->skip_samples) {
			st->skip_samples--;
			continue;
		}
		memcpy(data, st->data_buff+i*16, bytes_per_datum);
		timestamp = icm40605_get_timestamp(st);
		iio_push_to_buffers_with_timestamp(indio_dev, &(data[1]), timestamp);
	}

end_session:
	mutex_unlock(&st->lock);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;

flush_fifo:
	/* Flush HW and SW FIFOs. */
	dev_info(regmap_get_device(st->regmap), "flush info\n");
	icm40605_reset_fifo(indio_dev);
	mutex_unlock(&st->lock);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}
