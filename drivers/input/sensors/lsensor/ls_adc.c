/* drivers/input/sensors/lsensor/ls_adc.c
 *
 * light sensor driver for the rv1108
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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/vermagic.h>
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/power_supply.h>
#include <linux/of_platform.h>

#define LIGHTSENSOR_STABLE_COUNT	10

static struct class *lightsensor_class;

struct lightsensor_data {
	struct platform_device *pdev;
	struct device *child_dev;
	struct delayed_work work;
	struct iio_channel *chan;
	int id;
	int max_voltage;
	int threshold;
};

static ssize_t value_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lightsensor_data *gdata;
	struct iio_channel *channel;
	int i, result, adcv = 0;
	int value = 0;

	gdata = dev_get_drvdata(dev);
	channel = gdata->chan;
	/* Read ADC value */
	for (i = 0; i < LIGHTSENSOR_STABLE_COUNT; i++) {
		result = iio_read_channel_raw(channel, &value);
		if (result < 0) {
			dev_err(dev, "read adc channel 2 error:%d\n", result);
			goto out;
		}
		adcv = adcv + value;
	}
	adcv = adcv / LIGHTSENSOR_STABLE_COUNT;
	return sprintf(buf, "%d\n", adcv);
out:
	return -1;
}

int lightsensor_vol_r(struct device *dev,
	int *light_vol)
{
	struct lightsensor_data *gdata;
	struct iio_channel *channel;
	int i, result;
	int value = 0;

	gdata = dev_get_drvdata(dev);
	channel = gdata->chan;
	/* Read ADC value */
	for (i = 0; i < LIGHTSENSOR_STABLE_COUNT; i++) {
		result = iio_read_channel_raw(channel, &value);
			if (result < 0) {
			dev_err(dev, "read adc channel 2 error:%d\n", result);
			goto out;
		}
		*light_vol = *light_vol + value;
	}
	*light_vol = *light_vol / LIGHTSENSOR_STABLE_COUNT;
	return 0;
out:
	return -1;
}
EXPORT_SYMBOL(lightsensor_vol_r);

static DEVICE_ATTR_RO(value);
static struct attribute *lightsensor_attrs[] = {
	&dev_attr_value.attr,
	NULL,
};
ATTRIBUTE_GROUPS(lightsensor);

static int lightsensor_dt_parse(
	struct device *dev,
	struct lightsensor_data *gdata)
{
	struct iio_channel *chan;

	chan = iio_channel_get(dev, NULL);
	if (IS_ERR(chan)) {
		dev_err(dev, "no io-channnels defined for lightsensor\n");
		return PTR_ERR(chan);
	}
	gdata->chan = chan;
	return 0;
}

static int lightsensor_probe(struct platform_device *pdev)
{
	int result;
	struct lightsensor_data *gdata;
	struct device *dev = &pdev->dev;

	gdata = devm_kzalloc(dev, sizeof(*gdata), GFP_KERNEL);
	if (gdata == NULL) {
		dev_err(dev, "lightsensor_data malloc failure!\n");
		result = -ENOMEM;
		goto err1;
	}
	gdata->pdev = pdev;
	platform_set_drvdata(pdev, gdata);
	result = lightsensor_dt_parse(dev, gdata);
	if (result) {
		dev_err(dev, "light sensor analysis dts failure!\n");
		goto err1;
	}
	gdata->child_dev = device_create(lightsensor_class, &pdev->dev, MKDEV(0, 0), gdata, "%s%d", "lsr_", 0);
	if (IS_ERR(gdata->child_dev)) {
		dev_err(dev, "class device create failed!\n");
		goto err1;
	}
	result = sysfs_create_groups(&gdata->child_dev->kobj, lightsensor_groups);
	if (result) {
		dev_err(&pdev->dev, "sysfs create files failed, result=%d\n", result);
		goto err2;
	}

	dev_info(&pdev->dev, "light sensor driver probe success\n");
	return 0;
err2:
	device_unregister(gdata->child_dev);
err1:
	return result;
}

static int lightsensor_remove(struct platform_device *pdev)
{
	struct lightsensor_data *gdata;

	gdata = platform_get_drvdata(pdev);
	device_unregister(gdata->child_dev);
	dev_info(&pdev->dev, "light sensor driver remove success\n");
	return 0;
}

static struct of_device_id lightsensor_match_table[] = {
	{.compatible = "rockchip,light_sensor"},
	{},
};

static struct platform_driver lightsensor_driver = {
	.probe  = lightsensor_probe,
	.remove = lightsensor_remove,
	.driver = {
		.name  = "adc-lightsensor",
		.owner = THIS_MODULE,
		.of_match_table = lightsensor_match_table,
	},
};

static int __init lightsensor_driver_init(void)
{
	lightsensor_class = class_create(THIS_MODULE, "lightsensor");
	if (IS_ERR(lightsensor_class))
		return PTR_ERR(lightsensor_class);
	return platform_driver_register(&lightsensor_driver);
}
module_init(lightsensor_driver_init);

static void __exit lightsensor_driver_exit(void)
{
	platform_driver_unregister(&lightsensor_driver);
	class_destroy(lightsensor_class);
}
module_exit(lightsensor_driver_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:adc-lightsensor");
MODULE_AUTHOR("ROCKCHIP");
