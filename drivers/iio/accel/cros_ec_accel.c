/*
 * cros_ec_accel - Driver for Chrome OS Embedded Controller accelerometer
 *
 * Copyright (C) 2014 Google, Inc
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
 * This driver uses the cros-ec interface to communicate with the Chrome OS
 * EC about accelerometer data. Accelerometer access is presented through
 * iio sysfs.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/kernel.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>

/* Indices for EC sensor values. */
enum sensor_index {
	ACC_BASE_X,
	ACC_BASE_Y,
	ACC_BASE_Z,
	ACC_LID_X,
	ACC_LID_Y,
	ACC_LID_Z,
	LID_ANGLE,
	GYRO_X,
	GYRO_Y,
	GYRO_Z,

	NUM_EC_INPUTS
};

/**
 * host_cmd_sensor_num - convert sensor index into host command sensor number.
 *
 * @idx sensor index (should be element of enum sensor_index)
 * @return host command sensor number
 */
static inline int host_cmd_sensor_num(int idx)
{
	if ((idx >= GYRO_X) && (idx <= GYRO_Z))
		return EC_MOTION_SENSOR_GYRO;
	else if ((idx >= ACC_BASE_X) && (idx <= ACC_BASE_Z))
		return EC_MOTION_SENSOR_ACCEL_BASE;
	else
		return EC_MOTION_SENSOR_ACCEL_LID;
}

/* Register addresses for EC accelerometer data. */
static const unsigned int ec_regs[NUM_EC_INPUTS] = {
	[ACC_BASE_X] = (EC_MEMMAP_ACC_DATA+2),
	[ACC_BASE_Y] = (EC_MEMMAP_ACC_DATA+4),
	[ACC_BASE_Z] = (EC_MEMMAP_ACC_DATA+6),
	[ACC_LID_X] =  (EC_MEMMAP_ACC_DATA+8),
	[ACC_LID_Y] =  (EC_MEMMAP_ACC_DATA+10),
	[ACC_LID_Z] =  (EC_MEMMAP_ACC_DATA+12),
	[LID_ANGLE] =  (EC_MEMMAP_ACC_DATA),
	[GYRO_X] =  (EC_MEMMAP_GYRO_DATA),
	[GYRO_Y] =  (EC_MEMMAP_GYRO_DATA+2),
	[GYRO_Z] =  (EC_MEMMAP_GYRO_DATA+4)
};

/* ADC counts per 1G. */
#define ACCEL_G 1024

enum accel_data_format {
	RAW,
	CALIBRATED,
};

/*
 * Scalar to use for the calibration scale. Typically the calibration
 * scale is a float near 1.0, but to avoid floating point, we will multiply
 * the calibration scale by this scalar. Using a power of 2 is more efficient.
 */
#define CALIB_SCALE_SCALAR 1024

#define EC_CHAN_COMMON						\
		.modified = 1,					        \
		.info_mask_separate =					\
			BIT(IIO_CHAN_INFO_RAW) |			\
			BIT(IIO_CHAN_INFO_PROCESSED) |			\
			BIT(IIO_CHAN_INFO_CALIBSCALE) |			\
			BIT(IIO_CHAN_INFO_CALIBBIAS),			\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
			BIT(IIO_CHAN_INFO_PEAK_SCALE) |			\
			BIT(IIO_CHAN_INFO_FREQUENCY) |			\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 16,					\
			.storagebits = 16,				\
			.shift = 0,					\
		}							\

#define EC_ACCEL_CHAN_COMMON						\
		.type = IIO_ACCEL,					\
		EC_CHAN_COMMON

#define EC_GYRO_CHAN_COMMON						\
		.type = IIO_ANGL_VEL,					\
		EC_CHAN_COMMON

static const struct iio_chan_spec ec_accel_channels[] = {
	{
		EC_ACCEL_CHAN_COMMON,
		.extend_name = "base",
		.channel2 = IIO_MOD_X,
		.scan_index = ACC_BASE_X,
	},
	{
		EC_ACCEL_CHAN_COMMON,
		.extend_name = "base",
		.channel2 = IIO_MOD_Y,
		.scan_index = ACC_BASE_Y,
	},
	{
		EC_ACCEL_CHAN_COMMON,
		.extend_name = "base",
		.channel2 = IIO_MOD_Z,
		.scan_index = ACC_BASE_Z,
	},
	{
		EC_ACCEL_CHAN_COMMON,
		.extend_name = "lid",
		.channel2 = IIO_MOD_X,
		.scan_index = ACC_LID_X,
	},
	{
		EC_ACCEL_CHAN_COMMON,
		.extend_name = "lid",
		.channel2 = IIO_MOD_Y,
		.scan_index = ACC_LID_Y,
	},
	{
		EC_ACCEL_CHAN_COMMON,
		.extend_name = "lid",
		.channel2 = IIO_MOD_Z,
		.scan_index = ACC_LID_Z,
	},
	{
		.type = IIO_ANGL,
		.channel = 0,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_PROCESSED) |
			BIT(IIO_CHAN_INFO_OFFSET),
		.address = LID_ANGLE,
		.scan_index = LID_ANGLE,
		.scan_type = {
			.sign = 's',
			.realbits = 9,
			.storagebits = 16,
			.shift = 0,
		},
	},
	{
		EC_GYRO_CHAN_COMMON,
		.extend_name = "gyro",
		.channel2 = IIO_MOD_X,
		.scan_index = GYRO_X,
	},
	{
		EC_GYRO_CHAN_COMMON,
		.extend_name = "gyro",
		.channel2 = IIO_MOD_Y,
		.scan_index = GYRO_Y,
	},
	{
		EC_GYRO_CHAN_COMMON,
		.extend_name = "gyro",
		.channel2 = IIO_MOD_Z,
		.scan_index = GYRO_Z,
	},
	IIO_CHAN_SOFT_TIMESTAMP(NUM_EC_INPUTS)
};


/* State data for ec_accel iio driver. */
struct cros_ec_accel_state {
	struct cros_ec_device *ec;

	/*
	 * Static array to hold data from a single capture. For each
	 * channel we need 2 bytes, except for the timestamp. The timestamp
	 * is always last and is always 8-byte aligned.
	 */
	union {
		u16 samples[NUM_EC_INPUTS];
		u64 ts[DIV_ROUND_UP(ARRAY_SIZE(ec_accel_channels),
					sizeof(u64)/sizeof(u16)) + 1];
	} capture_data;

	/*
	 * Calibration parameters. Note that trigger captured data will always
	 * provide the calibrated values.
	 */
	int calib_scale[NUM_EC_INPUTS];
	int calib_offset[NUM_EC_INPUTS];
};

/**
 * apply_calibration - apply calibration to raw data from a sensor
 *
 * @st Pointer to state information for device.
 * @data Raw data to convert.
 * @sensor_id The sensor id that the data belongs to.
 * @return calibrated value
 *
 * The processed value can be calculated as:
 * processed = (raw * calib_scale/CALIB_SCALE_SCALAR) + calib_offset.
 */
static inline s16 apply_calibration(struct cros_ec_accel_state *st,
				    s16 data, int sensor_id)
{
	return (data * st->calib_scale[sensor_id] / CALIB_SCALE_SCALAR) +
		st->calib_offset[sensor_id];
}

/**
 * read_ec_until_not_busy - read from EC status byte until it reads not busy.
 *
 * @st Pointer to state information for device.
 * @return 8-bit status if ok, -ve on error
 */
static int read_ec_until_not_busy(struct cros_ec_accel_state *st)
{
	struct cros_ec_device *ec = st->ec;
	u8 status;
	int attempts = 0;

	ec->cmd_read_u8(st->ec, EC_MEMMAP_ACC_STATUS, &status);
	while (status & EC_MEMMAP_ACC_STATUS_BUSY_BIT) {
		/* Give up after enough attempts, return error. */
		if (attempts++ >= 50)
			return -EIO;

		/* Small delay every so often. */
		if (attempts % 5 == 0)
			msleep(25);

		ec->cmd_read_u8(st->ec, EC_MEMMAP_ACC_STATUS, &status);
	}

	return status;
}

/**
 * read_ec_accel_data_unsafe - read acceleration data from EC shared memory.
 *
 * @st Pointer to state information for device.
 * @scan_mask Bitmap of the sensor indices to scan.
 * @data Location to store data.
 * @ret_format Return data format (RAW or CALIBRATED)
 *
 * Note this is the unsafe function for reading the EC data. It does not
 * guarantee that the EC will not modify the data as it is being read in.
 */
static void read_ec_accel_data_unsafe(struct cros_ec_accel_state *st,
			 long unsigned int scan_mask, u16 *data,
			 enum accel_data_format ret_format)
{
	struct cros_ec_device *ec = st->ec;
	int i = 0;
	int num_enabled = bitmap_weight(&scan_mask, NUM_EC_INPUTS);

	/*
	 * Read all sensors enabled in scan_mask. Each value is 2
	 * bytes.
	 */
	while (num_enabled--) {
		i = find_next_bit(&scan_mask, NUM_EC_INPUTS, i);
		ec->cmd_read_u16(st->ec, ec_regs[i], data);

		/* Calibrate the data if desired. */
		if (ret_format == CALIBRATED)
			*data = apply_calibration(st, (s16)*data, i);

		i++;
		data++;
	}
}

/**
 * read_ec_accel_data - read acceleration data from EC shared memory.
 *
 * @st Pointer to state information for device.
 * @scan_mask Bitmap of the sensor indices to scan.
 * @data Location to store data.
 * @ret_format Return data format (RAW or CALIBRATED)
 * @return 0 if ok, -ve on error
 *
 * Note: this is the safe function for reading the EC data. It guarantees
 * that the data sampled was not modified by the EC while being read.
 */
static int read_ec_accel_data(struct cros_ec_accel_state *st,
			      long unsigned int scan_mask, u16 *data,
			      enum accel_data_format ret_format)
{
	struct cros_ec_device *ec = st->ec;
	u8 samp_id = 0xff, status = 0;
	int attempts = 0;

	/*
	 * Continually read all data from EC until the status byte after
	 * all reads reflects that the EC is not busy and the sample id
	 * matches the sample id from before all reads. This guarantees
	 * that data read in was not modified by the EC while reading.
	 */
	while ((status & (EC_MEMMAP_ACC_STATUS_BUSY_BIT |
			  EC_MEMMAP_ACC_STATUS_SAMPLE_ID_MASK)) != samp_id) {
		/* If we have tried to read too many times, return error. */
		if (attempts++ >= 5)
			return -EIO;

		/* Read status byte until EC is not busy. */
		status = read_ec_until_not_busy(st);
		if (status < 0)
			return status;

		/*
		 * Store the current sample id so that we can compare to the
		 * sample id after reading the data.
		 */
		samp_id = status & EC_MEMMAP_ACC_STATUS_SAMPLE_ID_MASK;

		/* Read all EC data, format it, and store it into data. */
		read_ec_accel_data_unsafe(st, scan_mask, data, ret_format);

		/* Read status byte. */
		ec->cmd_read_u8(st->ec, EC_MEMMAP_ACC_STATUS, &status);
	}

	return 0;
}

/**
 * send_motion_host_cmd - send motion sense host command
 *
 * @st Pointer to state information for device.
 * @param Pointer to motion sense host command parameter struct.
 * @resp Pointer to motion sense host command response struct.
 * @return 0 if ok, -ve on error.
 *
 * Note, when called, the sub-command is assumed to be set in param->cmd.
 */
static int send_motion_host_cmd(struct cros_ec_accel_state *st,
				struct ec_params_motion_sense *param,
				struct ec_response_motion_sense *resp)
{
	struct cros_ec_command msg;

	/* Set up the host command structure. */
	msg.version = 0;
	msg.command = EC_CMD_MOTION_SENSE_CMD;
	msg.outdata = (uint8_t *)param;
	msg.outsize = sizeof(struct ec_params_motion_sense);
	msg.indata = (uint8_t *)resp;
	msg.insize = sizeof(struct ec_response_motion_sense);

	/* Send host command. */
	if (cros_ec_cmd_xfer(st->ec, &msg) > 0)
		return 0;
	else
		return -EIO;
}

static int ec_accel_read(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  int *val, int *val2, long mask)
{
	struct cros_ec_accel_state *st = iio_priv(indio_dev);
	struct ec_params_motion_sense param;
	struct ec_response_motion_sense resp;
	u16 data = 0;
	int ret = IIO_VAL_INT;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (read_ec_accel_data(st, (1 << chan->scan_index),
					&data, RAW) < 0)
			ret = -EIO;
		*val = (s16)data;
		break;
	case IIO_CHAN_INFO_PROCESSED:
		if (read_ec_accel_data(st, (1 << chan->scan_index),
					&data, CALIBRATED) < 0)
			ret = -EIO;
		*val = (s16)data;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = ACCEL_G;
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		*val = st->calib_scale[chan->scan_index];
		break;
	case IIO_CHAN_INFO_CALIBBIAS:
		*val = st->calib_offset[chan->scan_index];
		break;
	case IIO_CHAN_INFO_OFFSET:
		/* Only lid angle supports offset field. */
		if (chan->scan_index != LID_ANGLE)
			return -EIO;

		param.cmd = MOTIONSENSE_CMD_KB_WAKE_ANGLE;
		param.kb_wake_angle.data = EC_MOTION_SENSE_NO_VALUE;

		if (send_motion_host_cmd(st, &param, &resp))
			return -EIO;
		else
			*val = resp.kb_wake_angle.ret;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		param.cmd = MOTIONSENSE_CMD_EC_RATE;
		param.ec_rate.data = EC_MOTION_SENSE_NO_VALUE;

		if (send_motion_host_cmd(st, &param, &resp))
			return -EIO;
		else
			*val = resp.ec_rate.ret;

		break;
	case IIO_CHAN_INFO_PEAK_SCALE:
		param.cmd = MOTIONSENSE_CMD_SENSOR_RANGE;
		param.sensor_range.data = EC_MOTION_SENSE_NO_VALUE;
		param.sensor_range.sensor_num =
			host_cmd_sensor_num(chan->scan_index);

		if (send_motion_host_cmd(st, &param, &resp))
			return -EIO;
		else
			*val = resp.sensor_range.ret;

		break;
	case IIO_CHAN_INFO_FREQUENCY:
		param.cmd = MOTIONSENSE_CMD_SENSOR_ODR;
		param.sensor_odr.data = EC_MOTION_SENSE_NO_VALUE;
		param.sensor_range.sensor_num =
			host_cmd_sensor_num(chan->scan_index);

		if (send_motion_host_cmd(st, &param, &resp))
			ret = -EIO;
		else
			*val = resp.sensor_odr.ret;

		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ec_accel_write(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct cros_ec_accel_state *st = iio_priv(indio_dev);
	struct ec_params_motion_sense param;
	struct ec_response_motion_sense resp;
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		st->calib_scale[chan->scan_index] = val;
		break;
	case IIO_CHAN_INFO_CALIBBIAS:
		st->calib_offset[chan->scan_index] = val;
		break;
	case IIO_CHAN_INFO_OFFSET:
		/* Only lid angle supports offset field. */
		if (chan->scan_index != LID_ANGLE)
			return -EIO;

		param.cmd = MOTIONSENSE_CMD_KB_WAKE_ANGLE;
		param.kb_wake_angle.data = val;

		if (send_motion_host_cmd(st, &param, &resp))
			return -EIO;

		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		param.cmd = MOTIONSENSE_CMD_EC_RATE;
		param.ec_rate.data = val;

		if (send_motion_host_cmd(st, &param, &resp))
			return -EIO;

		break;

	case IIO_CHAN_INFO_PEAK_SCALE:
		param.cmd = MOTIONSENSE_CMD_SENSOR_RANGE;
		param.sensor_range.data = val;
		param.sensor_range.sensor_num =
			host_cmd_sensor_num(chan->scan_index);

		/* Always roundup, so caller gets at least what it asks for. */
		param.sensor_range.roundup = 1;

		if (send_motion_host_cmd(st, &param, &resp))
			return -EIO;

		break;
	case IIO_CHAN_INFO_FREQUENCY:
		param.cmd = MOTIONSENSE_CMD_SENSOR_ODR;
		param.sensor_odr.data = val;
		param.sensor_range.sensor_num =
			host_cmd_sensor_num(chan->scan_index);

		/* Always roundup, so caller gets at least what it asks for. */
		param.sensor_odr.roundup = 1;

		if (send_motion_host_cmd(st, &param, &resp))
			ret = -EIO;

		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct iio_info ec_accel_info = {
	.read_raw = &ec_accel_read,
	.write_raw = &ec_accel_write,
	.driver_module = THIS_MODULE,
};

/**
 * accel_capture - the trigger handler function
 *
 * @irq: the interrupt number
 * @p: private data - always a pointer to the poll func.
 *
 * On a trigger event occurring, if the pollfunc is attached then this
 * handler is called as a threaded interrupt (and hence may sleep). It
 * is responsible for grabbing data from the device and pushing it into
 * the associated buffer.
 */
static irqreturn_t accel_capture(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct cros_ec_accel_state *st = iio_priv(indio_dev);

	/* Clear capture data. */
	memset(st->capture_data.samples, 0, sizeof(st->capture_data));

	/*
	 * Read data based on which channels are enabled in scan mask. Note
	 * that on a capture we are always reading the calibrated data.
	 */
	read_ec_accel_data(st, *(indio_dev->active_scan_mask),
			   st->capture_data.samples, CALIBRATED);

	/* Store the timestamp last 8 bytes of data. */
	if (indio_dev->scan_timestamp)
		st->capture_data.ts[(indio_dev->scan_bytes - 1) / sizeof(s64)] =
			iio_get_time_ns();

	iio_push_to_buffers(indio_dev, (u8 *)st->capture_data.samples);

	/*
	 * Tell the core we are done with this trigger and ready for the
	 * next one.
	 */
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops iio_simple_dummy_buffer_setup_ops = {
	/* Generic function that attaches the pollfunc to the trigger. */
	.postenable = &iio_triggered_buffer_postenable,

	/* Generic function that detaches the pollfunc from the trigger. */
	.predisable = &iio_triggered_buffer_predisable,
};

static int configure_buffer(struct iio_dev *indio_dev,
	const struct iio_chan_spec *channels, const unsigned int num_channels)
{
	int ret;
	struct iio_buffer *buffer;

	/* Allocate a buffer to use - here a kfifo. */
	buffer = iio_kfifo_allocate(indio_dev);
	if (buffer == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}

	/* Enable timestamps by default. */
	buffer->scan_timestamp = true;

	indio_dev->buffer = buffer;
	indio_dev->setup_ops = &iio_simple_dummy_buffer_setup_ops;
	indio_dev->pollfunc =
		iio_alloc_pollfunc(NULL, &accel_capture, IRQF_ONESHOT,
				   indio_dev, "");

	if (indio_dev->pollfunc == NULL) {
		ret = -ENOMEM;
		goto error_free_buffer;
	}

	/* This device uses buffered captures driven by trigger. */
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;

	ret = iio_buffer_register(indio_dev, channels, num_channels);
	if (ret)
		goto error_dealloc_pollfunc;

	return 0;

error_dealloc_pollfunc:
	iio_dealloc_pollfunc(indio_dev->pollfunc);
error_free_buffer:
	iio_kfifo_free(indio_dev->buffer);
error_ret:
	return ret;
}

static int ec_accel_probe(struct platform_device *pdev)
{
	struct cros_ec_device *ec = dev_get_drvdata(pdev->dev.parent);
	struct iio_dev *indio_dev;
	struct cros_ec_accel_state *state;
	int ret, i;
	u8 status;

	if (!ec) {
		dev_warn(&pdev->dev, "No CROS EC device found.\n");
		ret = -EINVAL;
		goto error_ret;
	}

	/*
	 * Check if EC supports direct memory reads and if EC has
	 * accelerometers.
	 */
	if (!ec->cmd_read_u8 || !ec->cmd_read_u16) {
		dev_info(&pdev->dev, "EC does not support direct reads.\n");
		ret = -ENODEV;
		goto error_ret;
	}
	ret = ec->cmd_read_u8(ec, EC_MEMMAP_ACC_STATUS, &status);
	if (ret < 0) {
		dev_info(&pdev->dev, "EC does not support direct reads.\n");
		goto error_ret;
	}

	/* Check if EC has accelerometers. */
	if (!(status & EC_MEMMAP_ACC_STATUS_PRESENCE_BIT)) {
		dev_info(&pdev->dev, "EC does not have accelerometers.\n");
		ret = -ENOENT;
		goto error_ret;
	}

	indio_dev = iio_device_alloc(sizeof(*state));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}

	platform_set_drvdata(pdev, indio_dev);

	state = iio_priv(indio_dev);
	state->ec = ec;

	/* Set nominal calibration offset and scale. */
	for (i = 0; i < NUM_EC_INPUTS; i++) {
		state->calib_offset[i] = 0;
		state->calib_scale[i] = CALIB_SCALE_SCALAR;
	}

	indio_dev->channels = ec_accel_channels;
	indio_dev->num_channels = ARRAY_SIZE(ec_accel_channels);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &ec_accel_info;
	indio_dev->name = "cros-ec-accel";
	indio_dev->modes = INDIO_DIRECT_MODE;

	/* Configure buffered capture support. */
	ret = configure_buffer(indio_dev, ec_accel_channels,
			       ARRAY_SIZE(ec_accel_channels));
	if (ret)
		goto error_free_device;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_free_device;

	return 0;

error_free_device:
	iio_device_free(indio_dev);
error_ret:
	return ret;
}

static int ec_accel_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);

	/* Unconfigure and free buffer. */
	iio_buffer_unregister(indio_dev);
	iio_dealloc_pollfunc(indio_dev->pollfunc);
	iio_kfifo_free(indio_dev->buffer);

	iio_device_free(indio_dev);

	return 0;
}

static const struct platform_device_id cros_ec_accel_ids[] = {
	{
		.name = "cros-ec-accel",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, cros_ec_accel_ids);

static struct platform_driver cros_ec_accel_platform_driver = {
	.driver = {
		.name	= "cros-ec-accel",
		.owner	= THIS_MODULE,
	},
	.probe		= ec_accel_probe,
	.remove		= ec_accel_remove,
};
module_platform_driver(cros_ec_accel_platform_driver);

MODULE_DESCRIPTION("ChromeOS EC accelerometer driver");
MODULE_LICENSE("GPL");
