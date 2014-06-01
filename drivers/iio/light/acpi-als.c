/*
 * ACPI Ambient Light Sensor Driver
 *
 * Based on ALS driver:
 * Copyright (C) 2009 Zhang Rui <rui.zhang@intel.com>
 *
 * Rework for IIO subsystem:
 * Copyright (C) 2012-2013 Martin Liska <marxin.liska@gmail.com>
 *
 * Final cleanup and debugging:
 * Copyright (C) 2013-2014 Marek Vasut <marex@denx.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/acpi.h>
#include <linux/err.h>
#include <linux/mutex.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define ACPI_ALS_CLASS			"als"
#define ACPI_ALS_DEVICE_NAME		"acpi-als"
#define ACPI_ALS_NOTIFY_ILLUMINANCE	0x80

ACPI_MODULE_NAME("acpi-als");

/*
 * So far, there's only one channel in here, but the specification for
 * ACPI0008 says there can be more to what the block can report. Like
 * chromaticity and such. We are ready for incoming additions!
 */
static const struct iio_chan_spec acpi_als_channels[] = {
	{
		.type		= IIO_LIGHT,
		.indexed	= 0,
		.channel	= 0,
		.scan_index	= 0,
		.scan_type	= {
			.sign		= 'u',
			.realbits	= 10,
			.storagebits	= 16,
		},
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW),
	},
};

/*
 * The event buffer contains timestamp and all the data from
 * the ACPI0008 block. There are multiple, but so far we only
 * support _ALI (illuminance). Once someone adds new channels
 * to acpi_als_channels[], the evt_buffer below will grow
 * automatically.
 */
#define EVT_NR_SOURCES		ARRAY_SIZE(acpi_als_channels)
#define EVT_BUFFER_SIZE		\
	(sizeof(int64_t) + (EVT_NR_SOURCES * sizeof(uint16_t)))

struct acpi_als {
	struct acpi_device	*device;
	struct iio_trigger	*trig;
	struct mutex		lock;

	uint16_t		evt_buffer[EVT_BUFFER_SIZE];
};

/*
 * All types of properties the ACPI0008 block can report. The ALI, ALC, ALT
 * and ALP can all be handled by als_read_value() below, while the ALR is
 * special.
 *
 * The _ALR property returns tables that can be used to fine-tune the values
 * reported by the other props based on the particular hardware type and it's
 * location (it contains tables for "rainy", "bright inhouse lighting" etc.).
 *
 * So far, we support only ALI (illuminance).
 */
#define ACPI_ALS_ILLUMINANCE	"_ALI"
#define ACPI_ALS_CHROMATICITY	"_ALC"
#define ACPI_ALS_COLOR_TEMP	"_ALT"
#define ACPI_ALS_POLLING	"_ALP"
#define ACPI_ALS_TABLES		"_ALR"

static int32_t als_read_value(struct acpi_als *als, char *prop)
{
	unsigned long long illuminance;
	acpi_status status;

	status = acpi_evaluate_integer(als->device->handle,
				prop, NULL, &illuminance);

	if (ACPI_FAILURE(status)) {
		ACPI_EXCEPTION((AE_INFO, status,
				"Error reading ALS illuminance"));
		/* Something went wrong, it's pitch black outside! */
		illuminance = 0;
	}

	return illuminance;
}

static void acpi_als_notify(struct acpi_device *device, u32 event)
{
	struct iio_dev *iio = acpi_driver_data(device);
	struct acpi_als *als = iio_priv(iio);
	uint16_t *buffer = als->evt_buffer;
	int64_t time_ns = iio_get_time_ns();

	mutex_lock(&als->lock);

	memset(buffer, 0, EVT_BUFFER_SIZE);

	switch (event) {
	case ACPI_ALS_NOTIFY_ILLUMINANCE:
		*buffer++ = als_read_value(als, ACPI_ALS_ILLUMINANCE);
		break;
	default:
		/* Unhandled event */
		dev_dbg(&device->dev, "Unhandled ACPI ALS event (%08x)!\n",
			event);
		return;
	}

	iio_push_to_buffers_with_timestamp(iio, (uint8_t *)als->evt_buffer,
					   time_ns);

	mutex_unlock(&als->lock);
}

static int acpi_als_read_raw(struct iio_dev *iio,
			struct iio_chan_spec const *chan, int *val,
			int *val2, long mask)
{
	struct acpi_als *als = iio_priv(iio);

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	/* we support only illumination (_ALI) so far. */
	if (chan->type != IIO_LIGHT)
		return -EINVAL;

	*val = als_read_value(als, ACPI_ALS_ILLUMINANCE);
	return IIO_VAL_INT;
}

static int acpi_als_validate_trigger(struct iio_dev *iio,
				     struct iio_trigger *trig)
{
	if (iio->dev.parent != trig->dev.parent)
		return -EINVAL;

	return 0;
}

static const struct iio_info acpi_als_info = {
	.driver_module		= THIS_MODULE,
	.read_raw		= acpi_als_read_raw,
	.validate_trigger	= acpi_als_validate_trigger,
};

static irqreturn_t acpi_als_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *iio = pf->indio_dev;
	struct acpi_als *als = iio_priv(iio);

	iio_trigger_notify_done(als->trig);

	return IRQ_HANDLED;
}

static int acpi_als_validate_device(struct iio_trigger *trig,
				    struct iio_dev *iio)
{
	if (iio->dev.parent != trig->dev.parent)
		return -EINVAL;

	return 0;
}

static const struct iio_trigger_ops acpi_als_trigger_ops = {
	.owner			= THIS_MODULE,
	.validate_device	= acpi_als_validate_device,
};

static int acpi_als_trigger_init(struct iio_dev *iio)
{
	struct acpi_als *als = iio_priv(iio);
	struct device *dev = &iio->dev;
	struct iio_trigger *trig;
	int ret;

	trig = devm_iio_trigger_alloc(dev, "%s-dev%i", iio->name, iio->id);
	if (!trig)
		return -ENOMEM;

	trig->dev.parent = dev->parent;
	trig->ops = &acpi_als_trigger_ops;
	iio_trigger_set_drvdata(trig, iio);

	ret = iio_trigger_register(trig);
	if (ret) {
		iio_trigger_free(trig);
		return ret;
	}

	als->trig = trig;

	return 0;
}

static void acpi_als_trigger_remove(struct iio_dev *iio)
{
	struct acpi_als *als = iio_priv(iio);
	iio_trigger_unregister(als->trig);
}

static int acpi_als_add(struct acpi_device *device)
{
	struct acpi_als *als;
	struct iio_dev *iio;
	int ret;

	iio = devm_iio_device_alloc(&device->dev, sizeof(*als));
	if (!iio)
		return -ENOMEM;

	als = iio_priv(iio);

	device->driver_data = iio;
	als->device = device;
	mutex_init(&als->lock);

	iio->name = ACPI_ALS_DEVICE_NAME;
	iio->dev.parent = &device->dev;
	iio->info = &acpi_als_info;
	iio->modes = INDIO_DIRECT_MODE;
	iio->channels = acpi_als_channels;
	iio->num_channels = ARRAY_SIZE(acpi_als_channels);

	ret = iio_triggered_buffer_setup(iio, &iio_pollfunc_store_time,
					&acpi_als_trigger_handler, NULL);
	if (ret)
		return ret;

	ret = acpi_als_trigger_init(iio);
	if (ret)
		goto err_trig;

	ret = iio_device_register(iio);
	if (ret < 0)
		goto err_dev;

	return 0;

err_dev:
	acpi_als_trigger_remove(iio);
err_trig:
	iio_triggered_buffer_cleanup(iio);
	return ret;
}

static int acpi_als_remove(struct acpi_device *device)
{
	struct iio_dev *iio = acpi_driver_data(device);

	iio_device_unregister(iio);
	iio_triggered_buffer_cleanup(iio);
	acpi_als_trigger_remove(iio);

	return 0;
}

static const struct acpi_device_id acpi_als_device_ids[] = {
	{"ACPI0008", 0},
	{"", 0},
};

MODULE_DEVICE_TABLE(acpi, acpi_als_device_ids);

static struct acpi_driver acpi_als_driver = {
	.name	= "acpi_als",
	.class	= ACPI_ALS_CLASS,
	.ids	= acpi_als_device_ids,
	.ops = {
		.add	= acpi_als_add,
		.remove	= acpi_als_remove,
		.notify	= acpi_als_notify,
	},
};

module_acpi_driver(acpi_als_driver);

MODULE_AUTHOR("Zhang Rui <rui.zhang@...>");
MODULE_AUTHOR("Martin Liska <marxin.liska@...>");
MODULE_AUTHOR("Marek Vasut <marex@...>");
MODULE_DESCRIPTION("ACPI Ambient Light Sensor Driver");
MODULE_LICENSE("GPL");
