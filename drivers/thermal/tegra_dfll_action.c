/*
 * tegra-dfll-action.c - connect Tegra DFLL clock driver to thermal framework
 *
 * Copyright (C) 2012-2014 NVIDIA Corporation.  All rights reserved.
 *
 * Aleksandr Frid <afrid@nvidia.com>
 * Paul Walmsley <pwalmsley@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk-provider.h>

#include <soc/tegra/tegra-dfll.h>

#define DFLL_CDEV_TYPE_FLOOR	"DFLL-floor"
#define DFLL_CDEV_TYPE_CAP	"DFLL-cap"
/**
 * struct tegra_dfll_cdev_data - DFLL cooling device info
 * @cdev_floor: pointer to floor cooling device, after thermal registration
 * @cdev_cap: pointer to cap cooling device, after thermal registration
 * @dfll_pdev: pointer to the DFLL instance associated with this cooling
 * device
 */
struct tegra_dfll_cdev_data {
	struct thermal_cooling_device *cdev_floor;
	struct thermal_cooling_device *cdev_cap;
	struct platform_device *dfll_pdev;
};

static struct tegra_dfll_cdev_data tegra_dfll_cdev_data;

/*
 * Thermal cooling device interface
 */

static int
tegra_dfll_cdev_floor_get_max_state(struct thermal_cooling_device *cdev,
				       unsigned long *max_state)
{
	struct platform_device *pdev = (struct platform_device *)cdev->devdata;
	int num;

	num = tegra_dfll_count_thermal_states(pdev, TEGRA_DFLL_THERMAL_FLOOR);
	if (num <= 0)
		*max_state = 0;
	else
		*max_state = num - 1;

	return 0;
}

static int
tegra_dfll_cdev_floor_get_cur_state(struct thermal_cooling_device *cdev,
				       unsigned long *cur_state)
{
	struct platform_device *pdev = (struct platform_device *)cdev->devdata;
	*cur_state = tegra_dfll_get_thermal_index(pdev,
						     TEGRA_DFLL_THERMAL_FLOOR);
	return 0;
}

static int
tegra_dfll_cdev_floor_set_state(struct thermal_cooling_device *cdev,
				   unsigned long cur_state)
{
	struct platform_device *pdev = (struct platform_device *)cdev->devdata;

	return tegra_dfll_update_thermal_index(pdev,
						  TEGRA_DFLL_THERMAL_FLOOR,
						  cur_state);
}

static struct thermal_cooling_device_ops tegra_dfll_floor_cooling_ops = {
	.get_max_state = tegra_dfll_cdev_floor_get_max_state,
	.get_cur_state = tegra_dfll_cdev_floor_get_cur_state,
	.set_cur_state = tegra_dfll_cdev_floor_set_state,
};

static int tegra_dfll_register_therm_floor(struct platform_device *pdev,
					      struct platform_device *dfll_pdev)
{
	struct thermal_cooling_device *tcd;
	struct device *dev = &pdev->dev;
	int ret = 0;

	tcd = thermal_of_cooling_device_register(dev->of_node,
					DFLL_CDEV_TYPE_FLOOR,
					(void *)dfll_pdev,
					&tegra_dfll_floor_cooling_ops);
	if (IS_ERR(tcd)) {
		dev_err(dev,
			"Tegra DFLL thermal reaction: failed to register\n");
		ret = PTR_ERR(tcd);
		goto error;
	}

	tegra_dfll_cdev_data.cdev_floor = tcd;
	tegra_dfll_cdev_data.dfll_pdev = dfll_pdev;

	dev_info(dev, "Tegra DFLL 'floor cooling device' registered\n");

error:
	return ret;
}

static int
tegra_dfll_cdev_cap_get_max_state(struct thermal_cooling_device *cdev,
				     unsigned long *max_state)
{
	struct platform_device *pdev = (struct platform_device *)cdev->devdata;
	int num;

	num = tegra_dfll_count_thermal_states(pdev, TEGRA_DFLL_THERMAL_CAP);
	if (num <= 0)
		*max_state = 0;
	else
		*max_state = num - 1;

	return 0;
}

static int
tegra_dfll_cdev_cap_get_cur_state(struct thermal_cooling_device *cdev,
				     unsigned long *cur_state)
{
	struct platform_device *pdev = (struct platform_device *)cdev->devdata;
	*cur_state = tegra_dfll_get_thermal_index(pdev,
						     TEGRA_DFLL_THERMAL_CAP);
	return 0;
}

static int
tegra_dfll_cdev_cap_set_state(struct thermal_cooling_device *cdev,
				 unsigned long cur_state)
{
	struct platform_device *pdev = (struct platform_device *)cdev->devdata;

	return tegra_dfll_update_thermal_index(pdev,
						  TEGRA_DFLL_THERMAL_CAP,
						  cur_state);
}

static struct thermal_cooling_device_ops tegra_dfll_cap_cooling_ops = {
	.get_max_state = tegra_dfll_cdev_cap_get_max_state,
	.get_cur_state = tegra_dfll_cdev_cap_get_cur_state,
	.set_cur_state = tegra_dfll_cdev_cap_set_state,
};

static int tegra_dfll_register_therm_cap(struct platform_device *pdev,
					    struct platform_device *dfll_pdev)
{
	struct thermal_cooling_device *tcd;
	struct device *dev = &pdev->dev;
	int ret = 0;

	tcd = thermal_of_cooling_device_register(dev->of_node,
					DFLL_CDEV_TYPE_CAP,
					(void *)dfll_pdev,
					&tegra_dfll_cap_cooling_ops);
	if (IS_ERR(tcd)) {
		dev_err(dev,
			"Tegra DFLL thermal reaction: failed to register\n");
		ret = PTR_ERR(tcd);
		goto error;
	}

	tegra_dfll_cdev_data.cdev_cap = tcd;
	tegra_dfll_cdev_data.dfll_pdev = dfll_pdev;

	dev_info(dev, "Tegra DFLL 'cap cooling device' registered\n");

error:
	return ret;
}

static int tegra_dfll_cdev_probe(struct platform_device *pdev)
{
	struct device_node *dn;
	const __be32 *prop;
	struct platform_device *dfll_pdev;
	struct device *dev = &pdev->dev;
	const char *cdev_type;
	struct clk *cpu_dfll_clk;
	int ret;

	prop = of_get_property(dev->of_node, "act-dev", NULL);
	if (!prop) {
		dev_err(dev,
			"Tegra DFLL thermal reaction: missing dfll node\n");
		return -ENOENT;
	}
	dn = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!dn) {
		dev_err(dev,
			"Tegra DFLL thermal reaction: missing dfll node\n");
		return -ENOENT;
	}

	cdev_type = of_get_property(dev->of_node, "cdev-type", NULL);
	if (IS_ERR_OR_NULL(cdev_type)) {
		dev_err(dev,
			"Tegra DFLL thermal reaction: missing cdev type\n");
		ret = -ENOENT;
		goto error;
	}

	cpu_dfll_clk = devm_clk_get(dev, "dfllCPU_out");
	if (IS_ERR(cpu_dfll_clk)) {
		dev_info(dev,
			 "Tegra DFLL thermal reaction: DFLL not ready\n");
		ret = -EPROBE_DEFER;
		goto error;
	}

	dfll_pdev = of_find_device_by_node(dn);
	if (!dfll_pdev) {
		dev_err(dev,
			"Tegra DFLL thermal reaction: missing dfll dev\n");
		ret = -ENOENT;
		goto error;
	}

	if (!strcmp(cdev_type, DFLL_CDEV_TYPE_FLOOR))
		ret = tegra_dfll_register_therm_floor(pdev, dfll_pdev);
	else if (!strcmp(cdev_type, DFLL_CDEV_TYPE_CAP))
		ret = tegra_dfll_register_therm_cap(pdev, dfll_pdev);
	else {
		ret = -ENOENT;
		dev_err(dev, "Tegra DFLL thermal reaction: incorrect cdev type\n");
	}

error:
	of_node_put(dn);
	return ret;
}

static int tegra_dfll_cdev_remove(struct platform_device *pdev)
{
	if (tegra_dfll_cdev_data.cdev_floor)
		thermal_cooling_device_unregister(
				tegra_dfll_cdev_data.cdev_floor);
	if (tegra_dfll_cdev_data.cdev_cap)
		thermal_cooling_device_unregister(
				tegra_dfll_cdev_data.cdev_cap);

	return 0;
}

static const struct of_device_id tegra_dfll_cdev_match[] = {
	{ .compatible = "nvidia,tegra-dfll-cdev-action", },
	{},
};

static struct platform_driver tegra_dfll_cdev_driver = {
	.driver = {
		.name   = "tegra_dfll_action",
		.owner  = THIS_MODULE,
		.of_match_table = tegra_dfll_cdev_match,
	},
	.probe = tegra_dfll_cdev_probe,
	.remove = tegra_dfll_cdev_remove,
};

module_platform_driver(tegra_dfll_cdev_driver);

MODULE_DESCRIPTION("Tegra DFLL thermal reaction driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aleksandr Frid <afrid@nvidia.com>");
MODULE_AUTHOR("Paul Walmsley <pwalmsley@nvidia.com>");
