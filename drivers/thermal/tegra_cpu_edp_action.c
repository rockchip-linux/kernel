/*
 * Copyright (C) 2012-2013 NVIDIA Corporation.  All rights reserved.
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
#include <soc/tegra/tegra-edp.h>

static struct thermal_cooling_device *cpu_edp_cdev;

static int tegra_edp_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *max_state)
{
	struct platform_device *pdev = (struct platform_device *)cdev->devdata;
	*max_state = tegra_cpu_edp_count_therm_floors(pdev);
	return 0;
}

static int tegra_edp_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *cur_state)
{
	struct platform_device *pdev = (struct platform_device *)cdev->devdata;
	*cur_state = tegra_cpu_edp_get_thermal_index(pdev);
	return 0;
}

static int tegra_edp_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long new_state)
{
	struct platform_device *pdev = (struct platform_device *)cdev->devdata;
	tegra_cpu_edp_update_thermal_index(pdev, new_state);
	dev_dbg(&pdev->dev, "Update CPU EDP to new state %ld\n", new_state);
	return 0;
}

static struct thermal_cooling_device_ops tegra_edp_cooling_ops = {
	.get_max_state = tegra_edp_get_max_state,
	.get_cur_state = tegra_edp_get_cur_state,
	.set_cur_state = tegra_edp_set_cur_state,
};

static int tegra_cpu_edp_cdev_probe(struct platform_device *pdev)
{
	struct thermal_cooling_device *tcd;
	struct device_node *dn;
	const __be32 *prop;
	struct platform_device *cpu_edp_pdev;
	struct device *dev = &pdev->dev;
	const char *cdev_type;
	int ret = 0;

	if (!tegra_cpu_edp_ready())
		return -EPROBE_DEFER;

	prop = of_get_property(dev->of_node, "act-dev", NULL);
	if (!prop) {
		dev_err(dev,
			"Missing cpu-edp node\n");
		return -ENOENT;
	}
	dn = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!dn) {
		dev_err(dev,
			"Can't dereference cpu-edp phandle\n");
		return -ENOENT;
	}

	cpu_edp_pdev = of_find_device_by_node(dn);
	if (!cpu_edp_pdev) {
		dev_err(dev,
			"Can't find cpu-edp platform device\n");
		ret = -ENOENT;
		goto error;
	}

	cdev_type = of_get_property(dev->of_node, "cdev-type", NULL);
	if (!cdev_type) {
		dev_err(dev,
			"Failed to read cpu-edp cap cdev type\n");
		ret = -EINVAL;
		goto error;
	}

	tcd = thermal_of_cooling_device_register(pdev->dev.of_node,
						 (char *)cdev_type,
						 (void *)cpu_edp_pdev,
						 &tegra_edp_cooling_ops);
	if (IS_ERR(tcd)) {
		dev_err(dev,
			"Tegra CPU EDP thermal reaction driver failed to register\n");
		ret = PTR_ERR(tcd);
		goto error;
	}

	cpu_edp_cdev = tcd;

	dev_info(dev, "Tegra CPU EDP cooling device registered\n");

error:
	of_node_put(dn);
	return ret;
}

static int tegra_cpu_edp_cdev_remove(struct platform_device *pdev)
{
	thermal_cooling_device_unregister(cpu_edp_cdev);

	return 0;
}

static const struct of_device_id tegra_cpu_edp_cdev_match[] = {
	{ .compatible = "nvidia,tegra124-cpu-edp-cdev-action", },
	{ .compatible = "nvidia,tegra132-cpu-edp-cdev-action", },
	{},
};

static struct platform_driver tegra_cpu_edp_cdev_driver = {
	.driver = {
		.name   = "tegra_cpu_edp_action",
		.owner  = THIS_MODULE,
		.of_match_table = tegra_cpu_edp_cdev_match,
	},
	.probe = tegra_cpu_edp_cdev_probe,
	.remove = tegra_cpu_edp_cdev_remove,
};

module_platform_driver(tegra_cpu_edp_cdev_driver);

MODULE_DESCRIPTION("Tegra CPU EDP thermal reaction driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Wei Ni <wni@nvidia.com>");
