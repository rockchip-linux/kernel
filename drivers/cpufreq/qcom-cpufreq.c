/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/err.h>

static int qcom_cpufreq_driver_init(void)
{
	struct platform_device_info devinfo = { .name = "cpufreq-krait", };
	struct device *cpu_dev;
	struct device_node *np;
	struct platform_device *pdev;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev)
		return -ENODEV;

	np = of_node_get(cpu_dev->of_node);
	if (!np)
		return -ENOENT;

	if (!of_device_is_compatible(np, "qcom,krait")) {
		of_node_put(np);
		return -ENODEV;
	}
	of_node_put(np);

	pdev = platform_device_register_full(&devinfo);

	return PTR_ERR_OR_ZERO(pdev);
}
module_init(qcom_cpufreq_driver_init);

MODULE_DESCRIPTION("Qualcomm CPUfreq driver");
MODULE_LICENSE("GPL v2");
