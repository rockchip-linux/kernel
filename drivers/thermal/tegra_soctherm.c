/*
 * drivers/thermal/tegra_soctherm.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Mikko Perttunen <mperttunen@nvidia.com>
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

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/thermal.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <soc/tegra/fuse.h>

#define SENSOR_CONFIG0				0
#define		SENSOR_CONFIG0_STOP		BIT(0)
#define		SENSOR_CONFIG0_TALL_SHIFT	8
#define		SENSOR_CONFIG0_TCALC_OVER	BIT(4)
#define		SENSOR_CONFIG0_OVER		BIT(3)
#define		SENSOR_CONFIG0_CPTR_OVER	BIT(2)
#define SENSOR_CONFIG1				4
#define		SENSOR_CONFIG1_TSAMPLE_SHIFT	0
#define		SENSOR_CONFIG1_TIDDQ_EN_SHIFT	15
#define		SENSOR_CONFIG1_TEN_COUNT_SHIFT	24
#define		SENSOR_CONFIG1_TEMP_ENABLE	BIT(31)
#define SENSOR_CONFIG2				8
#define		SENSOR_CONFIG2_THERMA_SHIFT	16
#define		SENSOR_CONFIG2_THERMB_SHIFT	0

#define SENSOR_PDIV				0x1c0
#define		SENSOR_PDIV_T124		0x8888
#define SENSOR_HOTSPOT_OFF			0x1c4
#define		SENSOR_HOTSPOT_OFF_T124		0x00060600
#define SENSOR_TEMP1				0x1c8
#define SENSOR_TEMP2				0x1cc

#define FUSE_TSENSOR8_CALIB			0x180
#define FUSE_SPARE_REALIGNMENT_REG_0		0x1fc

#define NOMINAL_CALIB_FT_T124			105
#define NOMINAL_CALIB_CP_T124			25

struct tegra_tsensor_configuration {
	u32 tall, tsample, tiddq_en, ten_count;
	u32 pdiv, tsample_ate, pdiv_ate;
};

struct tegra_tsensor {
	u32 base;
	u32 calib_fuse_offset;
	/* Correction values used to modify values read from calibration fuses */
	s32 fuse_corr_alpha, fuse_corr_beta;
};

struct tegra_thermctl_zone {
	void __iomem *temp_reg;
	int temp_shift;
};

static const struct tegra_tsensor_configuration t124_tsensor_config = {
	.tall = 16300,
	.tsample = 120,
	.tiddq_en = 1,
	.ten_count = 1,
	.pdiv = 8,
	.tsample_ate = 481,
	.pdiv_ate = 8
};

static struct tegra_tsensor t124_tsensors[] = {
	{
		.base = 0xc0,
		.calib_fuse_offset = 0x098,
		.fuse_corr_alpha = 1135400,
		.fuse_corr_beta = -6266900,
	},
	{
		.base = 0xe0,
		.calib_fuse_offset = 0x084,
		.fuse_corr_alpha = 1122220,
		.fuse_corr_beta = -5700700,
	},
	{
		.base = 0x100,
		.calib_fuse_offset = 0x088,
		.fuse_corr_alpha = 1127000,
		.fuse_corr_beta = -6768200,
	},
	{
		.base = 0x120,
		.calib_fuse_offset = 0x12c,
		.fuse_corr_alpha = 1110900,
		.fuse_corr_beta = -6232000,
	},
	{
		.base = 0x140,
		.calib_fuse_offset = 0x158,
		.fuse_corr_alpha = 1122300,
		.fuse_corr_beta = -5936400,
	},
	{
		.base = 0x160,
		.calib_fuse_offset = 0x15c,
		.fuse_corr_alpha = 1145700,
		.fuse_corr_beta = -7124600,
	},
	{
		.base = 0x180,
		.calib_fuse_offset = 0x154,
		.fuse_corr_alpha = 1120100,
		.fuse_corr_beta = -6000500,
	},
	{
		.base = 0x1a0,
		.calib_fuse_offset = 0x160,
		.fuse_corr_alpha = 1106500,
		.fuse_corr_beta = -6729300,
	},
};

struct tegra_soctherm {
	struct reset_control *reset;
	struct clk *clock_tsensor;
	struct clk *clock_soctherm;
	void __iomem *regs;

	struct thermal_zone_device *thermctl_tzs[4];
};

struct tsensor_shared_calibration {
	u32 base_cp, base_ft;
	u32 actual_temp_cp, actual_temp_ft;
};

static int calculate_shared_calibration(struct tsensor_shared_calibration *r)
{
	u32 val;
	u32 shifted_cp, shifted_ft;
	int err;

	err = tegra_fuse_readl(FUSE_TSENSOR8_CALIB, &val);
	if (err)
		return err;
	r->base_cp = val & 0x3ff;
	r->base_ft = (val & (0x7ff << 10)) >> 10;

	err = tegra_fuse_readl(FUSE_SPARE_REALIGNMENT_REG_0, &val);
	if (err)
		return err;
	shifted_cp = sign_extend32(val, 5);
	val = ((val & (0x1f << 21)) >> 21);
	shifted_ft = sign_extend32(val, 4);

	r->actual_temp_cp = 2 * NOMINAL_CALIB_CP_T124 + shifted_cp;
	r->actual_temp_ft = 2 * NOMINAL_CALIB_FT_T124 + shifted_ft;

	return 0;
}

static int calculate_tsensor_calibration(
	struct tegra_tsensor *sensor,
	struct tsensor_shared_calibration shared,
	u32 *calib
)
{
	u32 val;
	s32 actual_tsensor_ft, actual_tsensor_cp;
	s32 delta_sens, delta_temp;
	s32 mult, div;
	s16 therma, thermb;
	int err;

	err = tegra_fuse_readl(sensor->calib_fuse_offset, &val);
	if (err)
		return err;

	actual_tsensor_cp = (shared.base_cp * 64) + sign_extend32(val, 12);
	val = (val & (0x1fff << 13)) >> 13;
	actual_tsensor_ft = (shared.base_ft * 32) + sign_extend32(val, 12);

	delta_sens = actual_tsensor_ft - actual_tsensor_cp;
	delta_temp = shared.actual_temp_ft - shared.actual_temp_cp;

	mult = t124_tsensor_config.pdiv * t124_tsensor_config.tsample_ate;
	div = t124_tsensor_config.tsample * t124_tsensor_config.pdiv_ate;

	therma = div_s64((s64) delta_temp * (1LL << 13) * mult,
		(s64) delta_sens * div);
	thermb = div_s64(((s64) actual_tsensor_ft * shared.actual_temp_cp) -
		((s64) actual_tsensor_cp * shared.actual_temp_ft),
		(s64) delta_sens);

	therma = div_s64((s64) therma * sensor->fuse_corr_alpha,
		(s64) 1000000LL);
	thermb = div_s64((s64) thermb * sensor->fuse_corr_alpha +
		sensor->fuse_corr_beta,
		(s64) 1000000LL);

	*calib = ((u16)(therma) << SENSOR_CONFIG2_THERMA_SHIFT) |
		((u16)thermb << SENSOR_CONFIG2_THERMB_SHIFT);

	return 0;
}

static int enable_tsensor(struct tegra_soctherm *tegra,
			  struct tegra_tsensor *sensor,
			  struct tsensor_shared_calibration shared)
{
	void * __iomem base = tegra->regs + sensor->base;
	unsigned int val;
	u32 calib;
	int err;

	err = calculate_tsensor_calibration(sensor, shared, &calib);
	if (err)
		return err;

	val = 0;
	val |= t124_tsensor_config.tall << SENSOR_CONFIG0_TALL_SHIFT;
	writel(val, base + SENSOR_CONFIG0);

	val = 0;
	val |= (t124_tsensor_config.tsample - 1) <<
		SENSOR_CONFIG1_TSAMPLE_SHIFT;
	val |= t124_tsensor_config.tiddq_en << SENSOR_CONFIG1_TIDDQ_EN_SHIFT;
	val |= t124_tsensor_config.ten_count << SENSOR_CONFIG1_TEN_COUNT_SHIFT;
	val |= SENSOR_CONFIG1_TEMP_ENABLE;
	writel(val, base + SENSOR_CONFIG1);

	writel(calib, base + SENSOR_CONFIG2);

	return 0;
}

static inline long translate_temp(u32 val)
{
	long t;

	t = ((val & 0xff00) >> 8) * 1000;
	if (val & 0x80)
		t += 500;
	if (val & 0x01)
		t *= -1;

	return t;
}

static int tegra_thermctl_get_temp(void *data, long *out_temp)
{
	struct tegra_thermctl_zone *zone = data;
	u32 val;

	val = (readl(zone->temp_reg) >> zone->temp_shift) & 0xffff;
	*out_temp = translate_temp(val);

	return 0;
}

static struct of_device_id tegra_soctherm_of_match[] = {
	{ .compatible = "nvidia,tegra124-soctherm" },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_soctherm_of_match);

static int thermctl_temp_offsets[] = {
	SENSOR_TEMP1, SENSOR_TEMP2, SENSOR_TEMP1, SENSOR_TEMP2
};

static int thermctl_temp_shifts[] = {
	16, 16, 0, 0
};

static int tegra_soctherm_probe(struct platform_device *pdev)
{
	struct tegra_soctherm *tegra;
	struct thermal_zone_device *tz;
	struct tsensor_shared_calibration shared_calib;
	int i;
	int err = 0;

	struct tegra_tsensor *tsensors = t124_tsensors;

	tegra = devm_kzalloc(&pdev->dev, sizeof(*tegra), GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;

	tegra->regs = devm_ioremap_resource(&pdev->dev,
		platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(tegra->regs)) {
		dev_err(&pdev->dev, "can't get registers");
		return PTR_ERR(tegra->regs);
	}

	tegra->reset = devm_reset_control_get(&pdev->dev, "soctherm");
	if (IS_ERR(tegra->reset)) {
		dev_err(&pdev->dev, "can't get soctherm reset\n");
		return PTR_ERR(tegra->reset);
	}

	tegra->clock_tsensor = devm_clk_get(&pdev->dev, "tsensor");
	if (IS_ERR(tegra->clock_tsensor)) {
		dev_err(&pdev->dev, "can't get clock tsensor\n");
		return PTR_ERR(tegra->clock_tsensor);
	}

	tegra->clock_soctherm = devm_clk_get(&pdev->dev, "soctherm");
	if (IS_ERR(tegra->clock_soctherm)) {
		dev_err(&pdev->dev, "can't get clock soctherm\n");
		return PTR_ERR(tegra->clock_soctherm);
	}

	reset_control_assert(tegra->reset);

	err = clk_prepare_enable(tegra->clock_soctherm);
	if (err) {
		reset_control_deassert(tegra->reset);
		return err;
	}

	err = clk_prepare_enable(tegra->clock_tsensor);
	if (err) {
		clk_disable_unprepare(tegra->clock_soctherm);
		reset_control_deassert(tegra->reset);
		return err;
	}

	reset_control_deassert(tegra->reset);

	/* Initialize raw sensors */

	err = calculate_shared_calibration(&shared_calib);
	if (err)
		goto disable_clocks;

	for (i = 0; i < ARRAY_SIZE(t124_tsensors); ++i) {
		err = enable_tsensor(tegra, tsensors + i, shared_calib);
		if (err)
			goto disable_clocks;
	}

	writel(SENSOR_PDIV_T124, tegra->regs + SENSOR_PDIV);
	writel(SENSOR_HOTSPOT_OFF_T124, tegra->regs + SENSOR_HOTSPOT_OFF);

	/* Initialize thermctl sensors */

	for (i = 0; i < ARRAY_SIZE(tegra->thermctl_tzs); ++i) {
		struct tegra_thermctl_zone *zone =
			devm_kzalloc(&pdev->dev, sizeof(*zone), GFP_KERNEL);
		if (!zone) {
			err = -ENOMEM;
			goto unregister_tzs;
		}

		zone->temp_reg = tegra->regs + thermctl_temp_offsets[i];
		zone->temp_shift = thermctl_temp_shifts[i];

		tz = thermal_zone_of_sensor_register(
			&pdev->dev, i, zone, tegra_thermctl_get_temp, NULL);
		if (IS_ERR(tz)) {
			err = PTR_ERR(tz);
			dev_err(&pdev->dev, "failed to register sensor: %d\n",
				err);
			--i;
			goto unregister_tzs;
		}

		tegra->thermctl_tzs[i] = tz;
	}

	return 0;

unregister_tzs:
	for (; i >= 0; i--)
		thermal_zone_of_sensor_unregister(&pdev->dev,
						  tegra->thermctl_tzs[i]);

disable_clocks:
	clk_disable_unprepare(tegra->clock_tsensor);
	clk_disable_unprepare(tegra->clock_soctherm);

	return err;
}

static int tegra_soctherm_remove(struct platform_device *pdev)
{
	struct tegra_soctherm *tegra = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ARRAY_SIZE(tegra->thermctl_tzs); ++i) {
		thermal_zone_of_sensor_unregister(&pdev->dev,
						  tegra->thermctl_tzs[i]);
	}

	clk_disable_unprepare(tegra->clock_tsensor);
	clk_disable_unprepare(tegra->clock_soctherm);

	return 0;
}

static struct platform_driver tegra_soctherm_driver = {
	.probe = tegra_soctherm_probe,
	.remove = tegra_soctherm_remove,
	.driver = {
		.name = "tegra_soctherm",
		.of_match_table = tegra_soctherm_of_match,
	},
};
module_platform_driver(tegra_soctherm_driver);

MODULE_AUTHOR("Mikko Perttunen <mperttunen@nvidia.com>");
MODULE_DESCRIPTION("Tegra SOCTHERM thermal management driver");
MODULE_LICENSE("GPL v2");
