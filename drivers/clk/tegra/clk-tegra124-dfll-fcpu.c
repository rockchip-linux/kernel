/*
 * Tegra124 DFLL FCPU clock source driver
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
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <soc/tegra/fuse.h>

#include "clk.h"
#include "clk-dfll.h"
#include "cvb.h"

/* Maximum CPU frequency, indexed by CPU speedo id */
static const unsigned long tegra124_max_freq_table[] = {
	[0] = 2014500000UL,
	[1] = 2320500000UL,
	[2] = 2116500000UL,
	[3] = 2524500000UL,
};

static const struct cvb_table tegra124_cpu_cvb_tables[] = {
	{
		.speedo_id = -1,
		.process_id = -1,
		.min_millivolts = 900,
		.max_millivolts = 1260,
		.alignment = {
			.step_uv = 10000, /* 10mV */
		},
		.speedo_scale = 100,
		.voltage_scale = 1000,
		.cvb_table = {
			{204000000UL,   {1112619, -29295, 402} },
			{306000000UL,	{1150460, -30585, 402} },
			{408000000UL,	{1190122, -31865, 402} },
			{510000000UL,	{1231606, -33155, 402} },
			{612000000UL,	{1274912, -34435, 402} },
			{714000000UL,	{1320040, -35725, 402} },
			{816000000UL,	{1366990, -37005, 402} },
			{918000000UL,	{1415762, -38295, 402} },
			{1020000000UL,	{1466355, -39575, 402} },
			{1122000000UL,	{1518771, -40865, 402} },
			{1224000000UL,	{1573009, -42145, 402} },
			{1326000000UL,	{1629068, -43435, 402} },
			{1428000000UL,	{1686950, -44715, 402} },
			{1530000000UL,	{1746653, -46005, 402} },
			{1632000000UL,	{1808179, -47285, 402} },
			{1734000000UL,	{1871526, -48575, 402} },
			{1836000000UL,	{1936696, -49855, 402} },
			{1938000000UL,	{2003687, -51145, 402} },
			{2014500000UL,	{2054787, -52095, 402} },
			{2116500000UL,	{2124957, -53385, 402} },
			{2218500000UL,	{2196950, -54665, 402} },
			{2320500000UL,	{2270765, -55955, 402} },
			{2422500000UL,	{2346401, -57235, 402} },
			{2524500000UL,	{2437299, -58535, 402} },
			{0,		{      0,      0,   0} },
		},
		.cpu_dfll_data = {
			.tune0_low = 0x005020ff,
			.tune0_high = 0x005040ff,
			.tune1 = 0x00000060,
		}
	},
};

/* Maximum CPU frequency, indexed by CPU speedo id */
static const unsigned long tegra132_max_freq_table[] = {
	/* We don't support A01 (speedo id == 1) */

	/* Artifically limit max CPU frequency without thermal throttling */
#if 0
	[1] = 2499000000UL,
#else
	[1] = 1938000000UL,
#endif
};

static const struct cvb_table tegra132_cpu_cvb_tables[] = {
	{
		.speedo_id = 1,
		.process_id = -1,
		.min_millivolts = 890,
		.max_millivolts = 1260,
		.alignment = {
			.step_uv = 10000, /* 10mV */
		},
		.speedo_scale = 100,
		.voltage_scale = 1000,
		.cvb_table = {
			{204000000UL,	{1225091, -39915,  743} },
			{306000000UL,	{1263591, -41215,  743} },
			{408000000UL,	{1303202, -42515,  743} },
			{510000000UL,	{1343922, -43815,  743} },
			{612000000UL,	{1385753, -45115,  743} },
			{714000000UL,	{1428693, -46415,  743} },
			{816000000UL,	{1472743, -47715,  743} },
			{918000000UL,	{1517903, -49015,  743} },
			{1020000000UL,	{1564174, -50315,  743} },
			{1122000000UL,	{1611553, -51615,  743} },
			{1224000000UL,	{1660043, -52915,  743} },
			{1326000000UL,	{1709643, -54215,  743} },
			{1428000000UL,	{1760353, -55515,  743} },
			{1530000000UL,	{1812172, -56815,  743} },
			{1632000000UL,	{1865102, -58115,  743} },
			{1734000000UL,	{1919141, -59425,  743} },
			{1836000000UL,	{1974291, -60725,  743} },
			{1938000000UL,	{2030550, -62025,  743} },
			{2014500000UL,	{2073190, -62985,  743} },
			{2091000000UL,	{2117020, -63975,  743} },
			{2193000000UL,	{2176054, -65275,  743} },
			{2295000000UL,	{2236198, -66575,  743} },
			{2397000000UL,	{2297452, -67875,  743} },
			{2499000000UL,	{2359816, -69175,  743} },
			{0,		{      0,      0,   0} },
		},
		.cpu_dfll_data = {
			/* Default values for speedo value <= 2180 */
			.tune0_low = 0x008a15ff,
			.tune0_high = 0x008a40ff,
			.tune1 = 0x00000095,
			.tune_high_min_millivolts = 950,
		},
	},
};

static struct tegra_dfll_soc_data soc;
static const struct cvb_table *tegra_cpu_cvb_tables;
static const unsigned long *cpu_max_freq_table;

static int tegra124_dfll_fcpu_probe(struct platform_device *pdev)
{
	return tegra_dfll_register(pdev, &soc);
}

static struct of_device_id tegra124_dfll_fcpu_of_match[] = {
	{ .compatible = "nvidia,tegra124-dfll", },
	{ .compatible = "nvidia,tegra132-dfll", },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra124_dfll_fcpu_of_match);

static const struct dev_pm_ops tegra124_dfll_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra_dfll_runtime_suspend,
			   tegra_dfll_runtime_resume, NULL)
};

static struct platform_driver tegra124_dfll_fcpu_driver = {
	.probe		= tegra124_dfll_fcpu_probe,
	.remove		= tegra_dfll_unregister,
	.driver		= {
		.name		= "tegra124-dfll",
		.owner		= THIS_MODULE,
		.of_match_table = tegra124_dfll_fcpu_of_match,
		.pm		= &tegra124_dfll_pm_ops,
	},
};

static int __init tegra124_dfll_fcpu_init(void)
{
	int process_id, speedo_id, speedo_value, chip_id;
	const struct cvb_table *cvb;
	int cvb_table_size, max_freq_table_size;

	chip_id = tegra_get_chip_id();

	if (chip_id == TEGRA124) {
		tegra_cpu_cvb_tables = tegra124_cpu_cvb_tables;
		cvb_table_size = ARRAY_SIZE(tegra124_cpu_cvb_tables);
		cpu_max_freq_table = tegra124_max_freq_table;
		max_freq_table_size = ARRAY_SIZE(tegra124_max_freq_table);
	} else if (chip_id == TEGRA132) {
		tegra_cpu_cvb_tables = tegra132_cpu_cvb_tables;
		cvb_table_size = ARRAY_SIZE(tegra132_cpu_cvb_tables);
		cpu_max_freq_table = tegra132_max_freq_table;
		max_freq_table_size = ARRAY_SIZE(tegra132_max_freq_table);
	} else {
		BUG();
	}

	process_id = tegra_sku_info.cpu_process_id;
	speedo_id = tegra_sku_info.cpu_speedo_id;
	speedo_value = tegra_sku_info.cpu_speedo_value;

	if (speedo_id >= max_freq_table_size) {
		pr_err("unknown max CPU freq for speedo_id=%d\n", speedo_id);
		return -ENODEV;
	}

	soc.opp_dev = get_cpu_device(0);
	if (!soc.opp_dev) {
		pr_err("no CPU0 device\n");
		return -ENODEV;
	}

	cvb = tegra_cvb_build_opp_table(tegra_cpu_cvb_tables,
					cvb_table_size,
					process_id, speedo_id, speedo_value,
					cpu_max_freq_table[speedo_id],
					soc.opp_dev);
	if (IS_ERR(cvb)) {
		pr_err("couldn't build OPP table: %ld\n", PTR_ERR(cvb));
		return PTR_ERR(cvb);
	}

	soc.assert_dvco_reset = (chip_id == TEGRA124) ?
		tegra124_clock_assert_dfll_dvco_reset :
		tegra132_clock_assert_dfll_dvco_reset;
	soc.deassert_dvco_reset = (chip_id == TEGRA124) ?
		tegra124_clock_deassert_dfll_dvco_reset :
		tegra132_clock_deassert_dfll_dvco_reset;
	soc.min_millivolts = cvb->min_millivolts;
	soc.tune0_low = cvb->cpu_dfll_data.tune0_low;
	soc.tune0_high = cvb->cpu_dfll_data.tune0_high;
	soc.tune1 = cvb->cpu_dfll_data.tune1;
	soc.tune_high_min_millivolts =
		cvb->cpu_dfll_data.tune_high_min_millivolts;
	/* Tegra132 has different tunning data based on speedo values */
	if (chip_id == TEGRA132) {
		if (speedo_value >= 2336) {
			soc.tune0_low = 0x008315ff;
			soc.tune0_high = 0x008340ff;
			soc.tune_high_min_millivolts = 900;
		} else if (speedo_value  > 2180) {
			soc.tune0_low = 0x008715ff;
			soc.tune0_high = 0x008740ff;
			soc.tune_high_min_millivolts = 900;
		}
	}

	return platform_driver_register(&tegra124_dfll_fcpu_driver);
}
module_init(tegra124_dfll_fcpu_init);

static void __exit tegra124_dfll_fcpu_exit(void)
{
	platform_driver_unregister(&tegra124_dfll_fcpu_driver);
}
module_exit(tegra124_dfll_fcpu_exit);

MODULE_DESCRIPTION("Tegra124 DFLL clock source driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksandr Frid <afrid@nvidia.com>");
MODULE_AUTHOR("Paul Walmsley <pwalmsley@nvidia.com>");
