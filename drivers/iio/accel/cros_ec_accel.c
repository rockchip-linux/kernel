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

static char *cros_ec_loc[] = {
	"base",		/* MOTIONSENSE_LOC_BASE */
	"lid",		/* MOTIONSENSE_LOC_LID */
	"unknown",
};

enum {
	X,
	Y,
	Z,
	MAX_AXIS,
};

/* ADC counts per 1G. */
#define ACCEL_G 1024
#define UNKNOWN_SENSOR_NUM -1

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

typedef int (*read_ec_accel_data_t)(struct iio_dev *indio_dev,
		long unsigned int scan_mask, s16 *data,
		enum accel_data_format ret_format);
/* State data for ec_accel iio driver. */
struct cros_ec_accel_state {
	struct cros_ec_device *ec;

	/* Number of sensors (accel + gyro) */
	unsigned sensor_num;

	/*
	 * Number of accel sensors.
	 * The EC presents the accel sensors values first,
	 * then the gyros values.
	 */
	unsigned accel_num;

	/* index of the channel used for reporting the lid angle */
	unsigned lid_angle_idx;

	/*
	 * Calibration parameters. Note that trigger captured data will always
	 * provide the calibrated values.
	 */
	struct calib_data {
		int scale;
		int offset;
	} *calib;

	/*
	 * Static array to hold data from a single capture. For each
	 * channel we need 2 bytes, except for the timestamp. The timestamp
	 * is always last and is always 8-byte aligned.
	 */
	u8 *samples;

	/*
	 *  Location to store command and response to the EC.
	 */
	struct mutex cmd_lock;

	/*
	 * Statically allocated command structure that holds parameters
	 * and response. Response is dynamically allocated, to match the number
	 * of sensors.
	 */
	struct cros_ec_command msg;
	struct ec_params_motion_sense param;
	struct ec_response_motion_sense *resp;

	/* Pointer to function used for accessing sensors values. */
	read_ec_accel_data_t read_ec_accel_data;
};

/*
 * idx_to_sensor_num - convert sensor index into host command sensor number.
 *
 * @st: private data
 * @idx: sensor index
 * @return host command sensor number
 */
static int idx_to_sensor_num(struct cros_ec_accel_state *st,
			     unsigned idx)
{
	if (idx < st->lid_angle_idx)
		return idx / MAX_AXIS;
	else
		return UNKNOWN_SENSOR_NUM;
}

/*
 * idx_to_reg - convert sensor index into offset in shared memory region.
 *
 * @st: private data
 * @idx: sensor index (should be element of enum sensor_index)
 * @return address to read at.
 */
static unsigned idx_to_reg(struct cros_ec_accel_state *st,
			   unsigned idx)
{
	if (idx == st->lid_angle_idx)
		return EC_MEMMAP_ACC_DATA;
	/*
	 * At this point, idx belongs to a valid sensor.
	 * lid_angle channel is always the last channel.
	 */
	if (idx_to_sensor_num(st, idx) < st->accel_num)
		return EC_MEMMAP_ACC_DATA + sizeof(u16) * (idx + 1);
	else
		return EC_MEMMAP_GYRO_DATA +
			sizeof(u16) * (idx - st->accel_num * MAX_AXIS);
}

/*
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
static s16 apply_calibration(struct cros_ec_accel_state *st,
			     s16 data, unsigned sensor_id)
{
	return (data * st->calib[sensor_id].scale / CALIB_SCALE_SCALAR) +
		st->calib[sensor_id].offset;
}

/*
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

/*
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
static void read_ec_accel_data_unsafe(struct iio_dev *indio_dev,
			 long unsigned int scan_mask, s16 *data,
			 enum accel_data_format ret_format)
{
	struct cros_ec_accel_state *st = iio_priv(indio_dev);
	struct cros_ec_device *ec = st->ec;
	unsigned i = 0;

	/*
	 * Read all sensors enabled in scan_mask. Each value is 2
	 * bytes.
	 */
	for_each_set_bit(i, &scan_mask, indio_dev->masklength) {
		ec->cmd_read_u16(st->ec, idx_to_reg(st, i), data);

		/* Calibrate the data if desired. */
		if (ret_format == CALIBRATED)
			*data = apply_calibration(st, *data, i);

		data++;
	}
}

/*
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
static int read_ec_accel_data_lpc(struct iio_dev *indio_dev,
			      long unsigned int scan_mask, s16 *data,
			      enum accel_data_format ret_format)
{
	struct cros_ec_accel_state *st = iio_priv(indio_dev);
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
		read_ec_accel_data_unsafe(indio_dev, scan_mask, data,
					  ret_format);

		/* Read status byte. */
		ec->cmd_read_u8(st->ec, EC_MEMMAP_ACC_STATUS, &status);
	}

	return 0;
}

/*
 * send_motion_host_cmd - send motion sense host command
 *
 * @st Pointer to state information for device.
 * @resp Pointer to motion sense host command response struct.
 * @extra_resp_size request more data after the usual response structure.
 * @return 0 if ok, -ve on error.
 *
 * Note, when called, the sub-command is assumed to be set in param->cmd.
 */
static int send_motion_host_cmd(struct cros_ec_accel_state *st,
				struct ec_response_motion_sense *resp,
				int extra_resp_size)
{
	/* Set up the host command structure. */
	st->msg.indata = (u8 *)resp;
	st->msg.insize = extra_resp_size +
		sizeof(struct ec_response_motion_sense);

	/* Send host command. */
	if (cros_ec_cmd_xfer_status(st->ec, &st->msg) > 0)
		return 0;
	else
		return -EIO;
}

static int read_ec_accel_data_cmd(struct iio_dev *indio_dev,
			      long unsigned int scan_mask, s16 *data,
			      enum accel_data_format ret_format)
{
	struct cros_ec_accel_state *st = iio_priv(indio_dev);
	int ret, sensor_num;
	unsigned i = 0;

	/*
	 * read all sensor data through a command.
	 */
	st->param.cmd = MOTIONSENSE_CMD_DUMP;
	st->param.dump.max_sensor_count = st->sensor_num;
	ret = send_motion_host_cmd(st, st->resp, st->sensor_num *
			sizeof(struct ec_response_motion_sensor_data));
	if (ret != 0) {
		dev_warn(&indio_dev->dev, "Unable to read sensor data\n");
		return ret;
	}

	for_each_set_bit(i, &scan_mask, indio_dev->masklength) {
		sensor_num = idx_to_sensor_num(st, i);
		if (sensor_num == UNKNOWN_SENSOR_NUM)
			*data = 0;
		else
			*data = st->resp->dump.sensor[
				sensor_num].data[i % MAX_AXIS];
		/* Calibrate the data if desired. */
		if (ret_format == CALIBRATED)
			*data = apply_calibration(st, *data, i);

		data++;
	}
	return 0;
}


static int ec_accel_read(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  int *val, int *val2, long mask)
{
	struct cros_ec_accel_state *st = iio_priv(indio_dev);
	s16 data = 0;
	int ret = IIO_VAL_INT;
	int sensor_num = idx_to_sensor_num(st, chan->scan_index);
	mutex_lock(&st->cmd_lock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (st->read_ec_accel_data(indio_dev, (1 << chan->scan_index),
					&data, RAW) < 0)
			ret = -EIO;
		*val = (s16)data;
		break;
	case IIO_CHAN_INFO_PROCESSED:
		if (st->read_ec_accel_data(indio_dev, (1 << chan->scan_index),
					&data, CALIBRATED) < 0)
			ret = -EIO;
		*val = (s16)data;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = ACCEL_G;
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		*val = st->calib[chan->scan_index].scale;
		break;
	case IIO_CHAN_INFO_CALIBBIAS:
		*val = st->calib[chan->scan_index].offset;
		break;
	case IIO_CHAN_INFO_OFFSET:
		/* Only lid angle supports offset field. */
		if (sensor_num != UNKNOWN_SENSOR_NUM) {
			dev_err(&indio_dev->dev,
				"offset only for angle - not %d - channel\n",
				chan->scan_index);
			return -EIO;
		}

		st->param.cmd = MOTIONSENSE_CMD_KB_WAKE_ANGLE;
		st->param.kb_wake_angle.data = EC_MOTION_SENSE_NO_VALUE;

		if (send_motion_host_cmd(st, st->resp, 0))
			return -EIO;
		else
			*val = st->resp->kb_wake_angle.ret;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		st->param.cmd = MOTIONSENSE_CMD_EC_RATE;
		st->param.ec_rate.data = EC_MOTION_SENSE_NO_VALUE;

		if (send_motion_host_cmd(st, st->resp, 0))
			return -EIO;
		else
			*val = st->resp->ec_rate.ret;
		break;
	case IIO_CHAN_INFO_PEAK_SCALE:
		if (sensor_num == UNKNOWN_SENSOR_NUM) {
			dev_err(&indio_dev->dev,
				"peak scale: index must not channel angle\n");
			return -EIO;
		}

		st->param.cmd = MOTIONSENSE_CMD_SENSOR_RANGE;
		st->param.sensor_range.data = EC_MOTION_SENSE_NO_VALUE;
		st->param.sensor_range.sensor_num = sensor_num;

		if (send_motion_host_cmd(st, st->resp, 0))
			return -EIO;
		else
			*val = st->resp->sensor_range.ret;

		break;
	case IIO_CHAN_INFO_FREQUENCY:
		if (sensor_num == UNKNOWN_SENSOR_NUM) {
			dev_err(&indio_dev->dev,
				"frequency: index must not channel angle\n");
			return -EIO;
		}

		st->param.cmd = MOTIONSENSE_CMD_SENSOR_ODR;
		st->param.sensor_odr.data = EC_MOTION_SENSE_NO_VALUE;
		st->param.sensor_range.sensor_num = sensor_num;

		if (send_motion_host_cmd(st, st->resp, 0))
			ret = -EIO;
		else
			*val = st->resp->sensor_odr.ret;

		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&st->cmd_lock);
	return ret;
}

static int ec_accel_write(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct cros_ec_accel_state *st = iio_priv(indio_dev);
	int ret = 0, sensor_num = idx_to_sensor_num(st, chan->scan_index);
	mutex_lock(&st->cmd_lock);

	switch (mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		st->calib[chan->scan_index].scale = val;
		break;
	case IIO_CHAN_INFO_CALIBBIAS:
		st->calib[chan->scan_index].offset = val;
		break;
	case IIO_CHAN_INFO_OFFSET:
		/* Only lid angle supports offset field. */
		if (sensor_num != UNKNOWN_SENSOR_NUM) {
			dev_err(&indio_dev->dev,
				"offset only for angle - not %d - channel\n",
				chan->scan_index);
			return -EIO;
		}


		st->param.cmd = MOTIONSENSE_CMD_KB_WAKE_ANGLE;
		st->param.kb_wake_angle.data = val;

		if (send_motion_host_cmd(st, st->resp, 0))
			return -EIO;

		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		st->param.cmd = MOTIONSENSE_CMD_EC_RATE;
		st->param.ec_rate.data = val;

		if (send_motion_host_cmd(st, st->resp, 0))
			return -EIO;

		break;

	case IIO_CHAN_INFO_PEAK_SCALE:
		if (sensor_num == UNKNOWN_SENSOR_NUM) {
			dev_err(&indio_dev->dev,
				"peak scale: index must not channel angle\n");
			return -EIO;
		}

		st->param.cmd = MOTIONSENSE_CMD_SENSOR_RANGE;
		st->param.sensor_range.data = val;
		st->param.sensor_range.sensor_num = sensor_num;

		/* Always roundup, so caller gets at least what it asks for. */
		st->param.sensor_range.roundup = 1;

		if (send_motion_host_cmd(st, st->resp, 0))
			return -EIO;

		break;
	case IIO_CHAN_INFO_FREQUENCY:
		if (sensor_num == UNKNOWN_SENSOR_NUM) {
			dev_err(&indio_dev->dev,
				"frequency: index must not channel angle\n");
			return -EIO;
		}

		st->param.cmd = MOTIONSENSE_CMD_SENSOR_ODR;
		st->param.sensor_odr.data = val;
		st->param.sensor_range.sensor_num = sensor_num;

		/* Always roundup, so caller gets at least what it asks for. */
		st->param.sensor_odr.roundup = 1;

		if (send_motion_host_cmd(st, st->resp, 0))
			ret = -EIO;

		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&st->cmd_lock);
	return ret;
}

static const struct iio_info ec_accel_info = {
	.read_raw = &ec_accel_read,
	.write_raw = &ec_accel_write,
	.driver_module = THIS_MODULE,
};

/*
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

	mutex_lock(&st->cmd_lock);
	/* Clear capture data. */
	memset(st->samples, 0, indio_dev->scan_bytes);

	/*
	 * Read data based on which channels are enabled in scan mask. Note
	 * that on a capture we are always reading the calibrated data.
	 */
	st->read_ec_accel_data(indio_dev, *(indio_dev->active_scan_mask),
			   (s16 *)st->samples, CALIBRATED);

	/* Store the timestamp last 8 bytes of data. */
	if (indio_dev->scan_timestamp)
		*(s64 *)&st->samples[round_down(indio_dev->scan_bytes -
						sizeof(s64),
				     sizeof(s64))] =
			iio_get_time_ns();

	iio_push_to_buffers(indio_dev, st->samples);

	/*
	 * Tell the core we are done with this trigger and ready for the
	 * next one.
	 */
	iio_trigger_notify_done(indio_dev->trig);
	mutex_unlock(&st->cmd_lock);

	return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops iio_simple_dummy_buffer_setup_ops = {
	/* Generic function that attaches the pollfunc to the trigger. */
	.postenable = &iio_triggered_buffer_postenable,

	/* Generic function that detaches the pollfunc from the trigger. */
	.predisable = &iio_triggered_buffer_predisable,
};

static int configure_buffer(struct iio_dev *indio_dev)
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

	ret = iio_buffer_register(indio_dev,
			indio_dev->channels,
			indio_dev->num_channels);
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
	struct ec_response_motion_sense resp;
	struct iio_chan_spec *channel, *channels;
	int ret, i, j, samples_size, idx, channel_num;

	if (!ec) {
		dev_warn(&pdev->dev, "No CROS EC device found.\n");
		return -EINVAL;
	}

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*state));
	if (indio_dev == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, indio_dev);

	state = iio_priv(indio_dev);
	state->ec = ec;
	mutex_init(&state->cmd_lock);
	/* Set up the host command structure. */
	state->msg.version = 1;
	state->msg.command = EC_CMD_MOTION_SENSE_CMD;
	state->msg.outdata = (u8 *)&state->param;
	state->msg.outsize = sizeof(struct ec_params_motion_sense);

	/* Check how many accel sensors */
	state->param.cmd = MOTIONSENSE_CMD_DUMP;
	state->param.dump.max_sensor_count = 0;
	if ((send_motion_host_cmd(state, &resp, 0)) ||
	    (resp.dump.sensor_count == 0))
		return -ENODEV;

	state->sensor_num = resp.dump.sensor_count;
	state->accel_num = 0;

	channel_num = state->sensor_num * MAX_AXIS +
			1;  /* for the lid angle if needed */
	channels = devm_kcalloc(&pdev->dev,
			channel_num + 1,  /* for the timestamp */
			sizeof(struct iio_chan_spec),
			GFP_KERNEL);
	if (channels == NULL)
		return -ENOMEM;

	/* For each retrieve type and location */
	for (i = 0, idx = 0, channel = channels; i < state->sensor_num; i++) {
		state->param.cmd = MOTIONSENSE_CMD_INFO;
		state->param.sensor_odr.sensor_num = i;
		if (send_motion_host_cmd(state, &resp, 0)) {
			dev_warn(&pdev->dev,
				 "Can not access sensor %d info\n", i);
			return -EIO;
		}
		if (resp.info.type == MOTIONSENSE_TYPE_ACCEL)
			state->accel_num++;
		for (j = X; j <= Z; j++, channel++, idx++) {
			switch (resp.info.type) {
			case MOTIONSENSE_TYPE_ACCEL:
				channel->type = IIO_ACCEL;
				break;
			case MOTIONSENSE_TYPE_GYRO:
				channel->type = IIO_ANGL_VEL;
				break;
			default:
				dev_warn(&pdev->dev, "unknown\n");
			}
			channel->modified = 1;
			channel->info_mask_separate =
				BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_PROCESSED) |
				BIT(IIO_CHAN_INFO_CALIBSCALE) |
				BIT(IIO_CHAN_INFO_CALIBBIAS);
			channel->info_mask_shared_by_type =
				BIT(IIO_CHAN_INFO_SCALE) |
				BIT(IIO_CHAN_INFO_PEAK_SCALE) |
				BIT(IIO_CHAN_INFO_FREQUENCY) |
				BIT(IIO_CHAN_INFO_SAMP_FREQ);
			channel->scan_type.sign = 's';
			channel->scan_type.realbits = 16;
			channel->scan_type.storagebits = 16;
			channel->scan_type.shift = 0;
			channel->channel2 = IIO_MOD_X + j;
			channel->extend_name =
				cros_ec_loc[resp.info.location];
			channel->scan_index = idx;
		}
	}
	/* Hack to display the lid angle. Not all firmware has it. */
	if (state->accel_num >= 2) {
		state->lid_angle_idx = idx;
		channel->type = IIO_ANGL;
		channel->channel = 0;
		channel->info_mask_separate =
			BIT(IIO_CHAN_INFO_PROCESSED) |
			BIT(IIO_CHAN_INFO_OFFSET);
		channel->scan_index = idx++;
		channel->scan_type.sign = 's';
		channel->scan_type.realbits = 9;
		channel->scan_type.storagebits = 16;
		channel->scan_type.shift = 0;
		channel++;
	} else {
		/* No lid angle sensor */
		state->lid_angle_idx = INT_MAX;
		channel_num--;
	}

	/* Timestamp */
	channel->type = IIO_TIMESTAMP;
	channel->channel = -1;
	channel->scan_index = idx++;
	channel->scan_type.sign = 's';
	channel->scan_type.realbits = 64;
	channel->scan_type.storagebits = 64;

	indio_dev->channels = channels;
	indio_dev->num_channels = idx;

	/* Set nominal calibration offset and scale. */
	state->calib = devm_kcalloc(&pdev->dev, channel_num,
			sizeof(struct calib_data), GFP_KERNEL);
	if (state->calib == NULL)
		return -ENOMEM;

	for (i = 0; i < channel_num; i++) {
		state->calib[i].offset = 0;
		state->calib[i].scale = CALIB_SCALE_SCALAR;
	}

	/*
	 * Samples:
	 * We need 2 bytes per samples, and 8 bytes for the timestamp.
	 * Moreover, the time stamp has to be aligned on 64 bit boundary.
	 */
	samples_size = round_up(sizeof(s16) * channel_num, sizeof(s64)) +
		sizeof(s64);
	state->samples = devm_kzalloc(&pdev->dev, samples_size, GFP_KERNEL);
	if (state->samples == NULL)
		return -ENOMEM;

	if (ec->cmd_read_u8 != NULL) {
		state->read_ec_accel_data = read_ec_accel_data_lpc;
		state->resp = devm_kzalloc(&pdev->dev,
				sizeof(struct ec_params_motion_sense),
				GFP_KERNEL);
	} else {
		state->read_ec_accel_data = read_ec_accel_data_cmd;
		state->resp = devm_kzalloc(&pdev->dev,
				sizeof(struct ec_params_motion_sense) +
				state->sensor_num *
				sizeof(struct ec_response_motion_sensor_data),
				GFP_KERNEL);
	}
	if (state->resp == NULL)
		return -ENOMEM;

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &ec_accel_info;
	indio_dev->name = "cros-ec-accel";
	indio_dev->modes = INDIO_DIRECT_MODE;

	/* Configure buffered capture support. */
	ret = configure_buffer(indio_dev);
	if (ret)
		return ret;

	ret = iio_device_register(indio_dev);
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
