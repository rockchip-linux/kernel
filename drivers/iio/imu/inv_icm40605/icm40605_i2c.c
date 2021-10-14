// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2021 Rockchip Electronics Co., Ltd
 * Author:Daniel Baluta <daniel.baluta@intel.com>
 */
#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include "icm40605.h"

static int icm40605_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct regmap *regmap;
	const char *name = NULL;
	int chip_type = 0;

	if (id)
		chip_type = id->driver_data;
	regmap = devm_regmap_init_i2c(client, &icm40605_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	if (id)
		name = id->name;

	return icm40605_core_probe(regmap, client->irq, name, chip_type, true);
}

static int icm40605_i2c_remove(struct i2c_client *client)
{
	icm40605_core_remove(&client->dev);

	return 0;
}

static const struct i2c_device_id icm40605_i2c_id[] = {
	{"icm40605", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, icm40605_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id icm40605_of_match[] = {
	{ .compatible = "Invensense,icm40605" },
	{ },
};
MODULE_DEVICE_TABLE(of, icm40605_of_match);
#endif

static struct i2c_driver icm40605_i2c_driver = {
	.driver = {
		.name			= "icm40605_i2c",
		.acpi_match_table	= ACPI_PTR(icm40605_acpi_match),
		.of_match_table		= of_match_ptr(icm40605_of_match),
	},
	.probe		= icm40605_i2c_probe,
	.remove		= icm40605_i2c_remove,
	.id_table	= icm40605_i2c_id,
};
module_i2c_driver(icm40605_i2c_driver);

MODULE_AUTHOR("Daniel Baluta <daniel.baluta@intel.com>");
MODULE_DESCRIPTION("ICM40605 I2C driver");
MODULE_LICENSE("GPL v2");
