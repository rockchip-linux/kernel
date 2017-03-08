/*
 * Texas Instruments TMP103 SMBus temperature sensor driver
 * Copyright (C) 2017 WangJianhui <wjh@rock-chips.com>
 *
 * Based on:
 * Texas Instruments TMP102 SMBus temperature sensor driver
 *
 * Copyright (C) 2010 Steven King <sfking@fdwdc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/thermal.h>
#include <linux/of.h>

#define	DRIVER_NAME		"tmp103"

#define TMP103_TEMP_REG		0x00
#define TMP103_CONF_REG		0x01
#define TMP103_TLOW_REG		0x02
#define TMP103_THIGH_REG	0x03

#define TMP103_CONF_M0		0x01
#define TMP103_CONF_M1		0x02
#define TMP103_CONF_LC		0x04
#define TMP103_CONF_FL		0x08
#define TMP103_CONF_FH		0x10
#define TMP103_CONF_CR0		0x20
#define TMP103_CONF_CR1		0x40
#define TMP103_CONF_ID		0x80
#define TMP103_CONF_SD		(TMP103_CONF_M1)
#define TMP103_CONF_SD_MASK	(TMP103_CONF_M0 | TMP103_CONF_M1)

#define TMP103_CONFIG		(TMP103_CONF_CR1 | TMP103_CONF_M1)
#define TMP103_CONFIG_MASK	(TMP103_CONF_CR0 | TMP103_CONF_CR1 | \
				 TMP103_CONF_M0 | TMP103_CONF_M1)

struct tmp103 {
	struct device *hwmon_dev;
	struct thermal_zone_device *tz;
	struct mutex lock;
	u8 config_orig;
	unsigned long last_update;
	int temp[3];
	u32 compensation_value;
};

static inline int tmp103_reg_to_mc(s8 val)
{
	return val * 1000;
}

static inline u8 tmp103_mc_to_reg(int val)
{
	return DIV_ROUND_CLOSEST(val, 1000);
}

static const u8 tmp103_reg[] = {
	TMP103_TEMP_REG,
	TMP103_TLOW_REG,
	TMP103_THIGH_REG,
};

static struct tmp103 *tmp103_update_device(struct i2c_client *client)
{
	struct tmp103 *tmp103 = i2c_get_clientdata(client);

	mutex_lock(&tmp103->lock);
	if (time_after(jiffies, tmp103->last_update + HZ / 3)) {
		int i;

		for (i = 0; i < ARRAY_SIZE(tmp103->temp); ++i) {
			int status = i2c_smbus_read_byte_data(client,
								 tmp103_reg[i]);
			if (status > -1)
				tmp103->temp[i] = tmp103_reg_to_mc(status);
		}
		tmp103->last_update = jiffies;
	}
	mutex_unlock(&tmp103->lock);
	return tmp103;
}

static int tmp103_read_temp(void *dev, long *temp)
{
	struct tmp103 *tmp103 = tmp103_update_device(to_i2c_client(dev));

	*temp = tmp103->temp[0] + tmp103->compensation_value;

	return 0;
}

static ssize_t tmp103_show_temp(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct sensor_device_attribute *sda = to_sensor_dev_attr(attr);
	struct tmp103 *tmp103 = tmp103_update_device(to_i2c_client(dev));

	return sprintf(buf, "%d\n",
		tmp103->temp[sda->index] + tmp103->compensation_value);
}

static ssize_t tmp103_set_temp(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct sensor_device_attribute *sda = to_sensor_dev_attr(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct tmp103 *tmp103 = i2c_get_clientdata(client);
	long val;
	int ret;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	val -= tmp103->compensation_value;
	val = clamp_val(val, -55000, 127000);
	mutex_lock(&tmp103->lock);
	tmp103->temp[sda->index] = val;
	ret = i2c_smbus_write_byte_data(client, tmp103_reg[sda->index],
					tmp103_mc_to_reg(val));
	mutex_unlock(&tmp103->lock);

	return ret ? ret : count;
}

static SENSOR_DEVICE_ATTR(temp1_input, 0444, tmp103_show_temp, NULL,
			  TMP103_TEMP_REG);

static SENSOR_DEVICE_ATTR(temp1_min, 0644, tmp103_show_temp,
			  tmp103_set_temp, TMP103_TLOW_REG);

static SENSOR_DEVICE_ATTR(temp1_max, 0644, tmp103_show_temp,
			  tmp103_set_temp, TMP103_THIGH_REG);

static struct attribute *tmp103_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_min.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	NULL
};

static const struct attribute_group tmp103_attr_group = {
	.attrs = tmp103_attrs,
};

static int tmp103_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct tmp103 *tmp103;
	int status;
	struct device_node *np = client->dev.of_node;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev,
			"adapter doesn't support SMBus word transactions\n");
		return -ENODEV;
	}

	tmp103 = devm_kzalloc(&client->dev, sizeof(*tmp103), GFP_KERNEL);
	if (!tmp103)
		return -ENOMEM;

	of_property_read_u32(np, "compensation_value",
			     &tmp103->compensation_value);

	i2c_set_clientdata(client, tmp103);

	status = i2c_smbus_read_byte_data(client, TMP103_CONF_REG);
	if (status < 0) {
		dev_err(&client->dev, "error reading config register status\n");
		return status;
	}

	tmp103->config_orig = status;
	status = i2c_smbus_write_byte_data(client,
					   TMP103_CONF_REG, TMP103_CONFIG);
	if (status < 0) {
		dev_err(&client->dev, "error writing config register status=%d\n",
			status);
		goto fail_restore_config;
	}

	status = i2c_smbus_read_byte_data(client, TMP103_CONF_REG);
	if (status < 0) {
		dev_err(&client->dev, "error reading config register\n");
		goto fail_restore_config;
	}
	status &= TMP103_CONFIG_MASK;
	if (status != TMP103_CONFIG) {
		dev_err(&client->dev, "config settings did not stick\n");
		status = -ENODEV;
		goto fail_restore_config;
	}

	tmp103->last_update = jiffies - HZ;
	mutex_init(&tmp103->lock);

	status = sysfs_create_group(&client->dev.kobj, &tmp103_attr_group);
	if (status) {
		dev_dbg(&client->dev, "could not create sysfs files\n");
		goto fail_restore_config;
	}
	tmp103->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(tmp103->hwmon_dev)) {
		dev_dbg(&client->dev, "unable to register hwmon device\n");
		status = PTR_ERR(tmp103->hwmon_dev);
		goto fail_remove_sysfs;
	}

	tmp103->tz = thermal_zone_of_sensor_register(&client->dev, 0,
							 &client->dev,
							 tmp103_read_temp,
							 NULL);
	if (IS_ERR(tmp103->tz))
		tmp103->tz = NULL;

	dev_info(&client->dev, "initialized\n");

	return 0;

fail_remove_sysfs:
	sysfs_remove_group(&client->dev.kobj, &tmp103_attr_group);
fail_restore_config:
	i2c_smbus_write_byte_data(client, TMP103_CONF_REG,
				  tmp103->config_orig);
	return status;
}

#ifdef CONFIG_PM
static int tmp103_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int config;

	config = i2c_smbus_read_byte_data(client, TMP103_CONF_REG);
	if (config < 0)
		return config;

	config |= TMP103_CONF_SD;
	return i2c_smbus_write_byte_data(client, TMP103_CONF_REG, config);
}

static int tmp103_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int config;

	config = i2c_smbus_read_byte_data(client, TMP103_CONF_REG);
	if (config < 0)
		return config;

	config &= ~TMP103_CONF_SD;
	return i2c_smbus_write_byte_data(client, TMP103_CONF_REG, config);
}

static const struct dev_pm_ops tmp103_dev_pm_ops = {
	.suspend	= tmp103_suspend,
	.resume		= tmp103_resume,
};

#define TMP103_DEV_PM_OPS (&tmp103_dev_pm_ops)
#else
#define	TMP103_DEV_PM_OPS NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id tmp103_id[] = {
	{ "tmp103", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tmp103_id);

static struct i2c_driver tmp103_driver = {
	.driver = {
		.name	= "tmp103",
		.pm	= TMP103_DEV_PM_OPS,
	},
	.probe		= tmp103_probe,
	.id_table	= tmp103_id,
};

module_i2c_driver(tmp103_driver);

MODULE_LICENSE("GPL");
