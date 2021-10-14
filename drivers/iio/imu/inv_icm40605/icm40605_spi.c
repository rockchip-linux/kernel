// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2021 Rockchip Electronics Co., Ltd */

#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/pm_runtime.h>

#include "icm40605.h"

static int icm40605_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;
	const char *name = NULL;
	int chip_type;
	const struct spi_device_id *id = spi_get_device_id(spi);

	chip_type = id->driver_data;
	name = id->name;

	regmap = devm_regmap_init_spi(spi, &icm40605_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
		       (int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return icm40605_core_probe(regmap, spi->irq, name, chip_type, true);
}

static int icm40605_spi_remove(struct spi_device *spi)
{
	icm40605_core_remove(&spi->dev);

	return 0;
}

static const struct spi_device_id icm40605_spi_id[] = {
	{"icm40605", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, icm40605_spi_id);

#ifdef CONFIG_OF
static const struct of_device_id icm40605_of_match[] = {
	{ .compatible = "invensense,icm40605" },
	{ },
};
MODULE_DEVICE_TABLE(of, icm40605_of_match);
#endif

static struct spi_driver icm40605_spi_driver = {
	.probe		= icm40605_spi_probe,
	.remove		= icm40605_spi_remove,
	.id_table	= icm40605_spi_id,
	.driver = {
		.acpi_match_table	= ACPI_PTR(icm40605_acpi_match),
		.of_match_table		= of_match_ptr(icm40605_of_match),
		.name			= "icm40605_spi",
		.pm			= &inv_icm42600_pm_ops,
	},
};
module_spi_driver(icm40605_spi_driver);

MODULE_AUTHOR("AABB");
MODULE_DESCRIPTION("Invensense ICM40605 SPI driver");
MODULE_LICENSE("GPL v2");
