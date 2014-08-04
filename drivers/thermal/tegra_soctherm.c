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

#include <dt-bindings/thermal/tegra124-soctherm.h>

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

#define THERMCTL_LEVEL0_GROUP_CPU		0x0
#define THERMCTL_LEVEL0_GROUP_GPU		0x4
#define THERMCTL_LEVEL0_GROUP_MEM		0x8
#define THERMCTL_LEVEL0_GROUP_TSENSE		0xc
#define		THERMCTL_LEVEL0_GROUP_EN	BIT(8)
#define		THERMCTL_LEVEL0_GROUP_DN_THRESH_SHIFT 9
#define		THERMCTL_LEVEL0_GROUP_UP_THRESH_SHIFT 17

#define THERMCTL_INTR_STATUS			0x84
#define THERMCTL_INTR_EN			0x88

#define SENSOR_PDIV				0x1c0
#define		SENSOR_PDIV_T124		0x8888
#define SENSOR_HOTSPOT_OFF			0x1c4
#define		SENSOR_HOTSPOT_OFF_T124		0x00060600
#define SENSOR_TEMP1				0x1c8
#define		SENSOR_TEMP1_CPU_TEMP_MASK	(0xffff << 16)
#define		SENSOR_TEMP1_GPU_TEMP_MASK	0xffff
#define SENSOR_TEMP2				0x1cc
#define		SENSOR_TEMP2_MEM_TEMP_MASK	(0xffff << 16)
#define		SENSOR_TEMP2_PLLX_TEMP_MASK	0xffff

#define FUSE_TSENSOR8_CALIB			0x180
#define FUSE_SPARE_REALIGNMENT_REG_0		0x1fc

#define NOMINAL_CALIB_FT_T124			105
#define NOMINAL_CALIB_CP_T124			25

/**
 * struct tegra_tsensor_group - SOC_THERM sensor group data
 * @name: short name of the temperature sensor group
 * @id: numeric ID of the temperature sensor group
 * @thermctl_isr_shift: bit shift for interrupt status/enable register
 * @thermctl_level0_offset: offset of the THERMCTL_LEVEL0_GROUP_* reg
 * @sensor_temp_offset: offset of the SENSOR_TEMP* register
 * @sensor_temp_mask: bit mask for this sensor group in SENSOR_TEMP* register
 */
struct tegra_tsensor_group {
	const char	*name;
	u8		id;
	u8		thermctl_isr_shift;
	u16		thermctl_level0_offset;
	u16		sensor_temp_offset;
	u32		sensor_temp_mask;
};

struct tegra_tsensor_configuration {
	u32 tall, tsample, tiddq_en, ten_count;
	u32 pdiv, tsample_ate, pdiv_ate;
};

struct tegra_tsensor {
	const char *name;
	u32 base;
	struct tegra_tsensor_configuration *config;
	u32 calib_fuse_offset;
	/* Correction values used to modify values read from calibration fuses */
	s32 fuse_corr_alpha, fuse_corr_beta;
	struct tegra_tsensor_group *group;
};

struct tegra_thermctl_zone {
	struct tegra_soctherm *tegra;
	struct tegra_tsensor_group *sensor_group;
	struct thermal_zone_device *tz;
};

static struct tegra_tsensor_configuration t124_tsensor_config = {
	.tall = 16300,
	.tsample = 120,
	.tiddq_en = 1,
	.ten_count = 1,
	.pdiv = 8,
	.tsample_ate = 481,
	.pdiv_ate = 8
};

static struct tegra_tsensor_group tegra124_tsensor_group_cpu = {
	.id				= TEGRA124_SOCTHERM_SENSOR_CPU,
	.name				= "cpu",
	.thermctl_isr_shift		= 8,
	.thermctl_level0_offset		= THERMCTL_LEVEL0_GROUP_CPU,
	.sensor_temp_offset		= SENSOR_TEMP1,
	.sensor_temp_mask		= SENSOR_TEMP1_CPU_TEMP_MASK,
};

static struct tegra_tsensor_group tegra124_tsensor_group_gpu = {
	.id				= TEGRA124_SOCTHERM_SENSOR_GPU,
	.name				= "gpu",
	.thermctl_isr_shift		= 16,
	.thermctl_level0_offset		= THERMCTL_LEVEL0_GROUP_GPU,
	.sensor_temp_offset		= SENSOR_TEMP1,
	.sensor_temp_mask		= SENSOR_TEMP1_GPU_TEMP_MASK,
};

static struct tegra_tsensor_group tegra124_tsensor_group_pll = {
	.id				= TEGRA124_SOCTHERM_SENSOR_PLLX,
	.name				= "pll",
	.thermctl_isr_shift		= 0,
	.thermctl_level0_offset		= THERMCTL_LEVEL0_GROUP_TSENSE,
	.sensor_temp_offset		= SENSOR_TEMP2,
	.sensor_temp_mask		= SENSOR_TEMP2_PLLX_TEMP_MASK,
};

static struct tegra_tsensor_group tegra124_tsensor_group_mem = {
	.id				= TEGRA124_SOCTHERM_SENSOR_MEM,
	.name				= "mem",
	.thermctl_isr_shift		= 24,
	.thermctl_level0_offset		= THERMCTL_LEVEL0_GROUP_MEM,
	.sensor_temp_offset		= SENSOR_TEMP2,
	.sensor_temp_mask		= SENSOR_TEMP2_MEM_TEMP_MASK,
};

static struct tegra_tsensor_group *tegra124_tsensor_groups[] = {
	&tegra124_tsensor_group_cpu,
	&tegra124_tsensor_group_gpu,
	&tegra124_tsensor_group_pll,
	&tegra124_tsensor_group_mem,
	NULL,
};

static struct tegra_tsensor t124_tsensors[] = {
	{
		.base = 0xc0,
		.name = "cpu0",
		.config = &t124_tsensor_config,
		.calib_fuse_offset = 0x098,
		.fuse_corr_alpha = 1135400,
		.fuse_corr_beta = -6266900,
		.group = &tegra124_tsensor_group_cpu,
	},
	{
		.base = 0xe0,
		.name = "cpu1",
		.config = &t124_tsensor_config,
		.calib_fuse_offset = 0x084,
		.fuse_corr_alpha = 1122220,
		.fuse_corr_beta = -5700700,
		.group = &tegra124_tsensor_group_cpu,
	},
	{
		.base = 0x100,
		.name = "cpu2",
		.config = &t124_tsensor_config,
		.calib_fuse_offset = 0x088,
		.fuse_corr_alpha = 1127000,
		.fuse_corr_beta = -6768200,
		.group = &tegra124_tsensor_group_cpu,
	},
	{
		.base = 0x120,
		.name = "cpu3",
		.config = &t124_tsensor_config,
		.calib_fuse_offset = 0x12c,
		.fuse_corr_alpha = 1110900,
		.fuse_corr_beta = -6232000,
		.group = &tegra124_tsensor_group_cpu,
	},
	{
		.base = 0x140,
		.name = "mem0",
		.config = &t124_tsensor_config,
		.calib_fuse_offset = 0x158,
		.fuse_corr_alpha = 1122300,
		.fuse_corr_beta = -5936400,
		.group = &tegra124_tsensor_group_mem,
	},
	{
		.base = 0x160,
		.name = "mem1",
		.config = &t124_tsensor_config,
		.calib_fuse_offset = 0x15c,
		.fuse_corr_alpha = 1145700,
		.fuse_corr_beta = -7124600,
		.group = &tegra124_tsensor_group_mem,
	},
	{
		.base = 0x180,
		.name = "gpu",
		.config = &t124_tsensor_config,
		.calib_fuse_offset = 0x154,
		.fuse_corr_alpha = 1120100,
		.fuse_corr_beta = -6000500,
		.group = &tegra124_tsensor_group_gpu,
	},
	{
		.base = 0x1a0,
		.name = "pllx",
		.config = &t124_tsensor_config,
		.calib_fuse_offset = 0x160,
		.fuse_corr_alpha = 1106500,
		.fuse_corr_beta = -6729300,
		.group = &tegra124_tsensor_group_pll,
	},
	{ .name = NULL },
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

/**
 * soctherm_writel() - writes a value to a SOC_THERM register
 * @ts: pointer to a struct tegra_soctherm
 * @v: the value to write
 * @reg: the register offset
 *
 * Writes @v to @reg.  No return value.
 */
static void soctherm_writel(struct tegra_soctherm *ts, u32 v, u16 reg)
{
	writel(v, (void __iomem *)(ts->regs + reg));
}

/**
 * soctherm_readl() - reads specified register from SOC_THERM IP block
 * @ts: pointer to a struct tegra_soctherm
 * @reg: register address to be read
 *
 * Return: the value of the register
 */
static u32 soctherm_readl(struct tegra_soctherm *ts, u16 reg)
{
	return readl(ts->regs + reg);
}

/**
 * soctherm_barrier() - ensure previous writes to SOC_THERM have completed
 * @ts: pointer to a struct tegra_soctherm
 *
 * Ensures that any previous writes to the SOC_THERM IP block have reached
 * the IP block before continuing.
 */
static void soctherm_barrier(struct tegra_soctherm *ts)
{
	soctherm_readl(ts, THERMCTL_LEVEL0_GROUP_CPU);
}

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

	mult = sensor->config->pdiv * sensor->config->tsample_ate;
	div = sensor->config->tsample * sensor->config->pdiv_ate;

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
	unsigned int val;
	u32 calib;
	int err;

	err = calculate_tsensor_calibration(sensor, shared, &calib);
	if (err)
		return err;

	val = 0;
	val |= sensor->config->tall << SENSOR_CONFIG0_TALL_SHIFT;
	soctherm_writel(tegra, val, sensor->base + SENSOR_CONFIG0);

	val = 0;
	val |= (sensor->config->tsample - 1) << SENSOR_CONFIG1_TSAMPLE_SHIFT;
	val |= sensor->config->tiddq_en << SENSOR_CONFIG1_TIDDQ_EN_SHIFT;
	val |= sensor->config->ten_count << SENSOR_CONFIG1_TEN_COUNT_SHIFT;
	val |= SENSOR_CONFIG1_TEMP_ENABLE;
	soctherm_writel(tegra, val, sensor->base + SENSOR_CONFIG1);

	soctherm_writel(tegra, calib, sensor->base + SENSOR_CONFIG2);

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

	val = soctherm_readl(zone->tegra,
			     zone->sensor_group->sensor_temp_offset);
	val &= zone->sensor_group->sensor_temp_mask;
	val >>= ffs(zone->sensor_group->sensor_temp_mask) - 1;

	*out_temp = translate_temp(val);

	return 0;
}

static int tegra_thermctl_set_trips(void *data, long low, long high)
{
	struct tegra_thermctl_zone *zone = data;
	u32 val;

	low /= 1000;
	high /= 1000;

	low = clamp_val(low, -127, 127);
	high = clamp_val(high, -127, 127);

	val = 0;
	val |= ((u8) low) << THERMCTL_LEVEL0_GROUP_DN_THRESH_SHIFT;
	val |= ((u8) high) << THERMCTL_LEVEL0_GROUP_UP_THRESH_SHIFT;
	val |= THERMCTL_LEVEL0_GROUP_EN;

	soctherm_writel(zone->tegra, val,
	       zone->sensor_group->thermctl_level0_offset);

	return 0;
}

static irqreturn_t soctherm_isr(int irq, void *dev_id)
{
	struct tegra_thermctl_zone *zone = dev_id;
	u32 val;
	u32 intr_mask = 0x03 << zone->sensor_group->thermctl_isr_shift;

	val = soctherm_readl(zone->tegra, THERMCTL_INTR_STATUS);

	if ((val & intr_mask) == 0)
		return IRQ_NONE;

	soctherm_writel(zone->tegra, val & intr_mask, THERMCTL_INTR_STATUS);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t soctherm_isr_thread(int irq, void *dev_id)
{
	struct tegra_thermctl_zone *zone = dev_id;

	thermal_zone_device_update(zone->tz);

	return IRQ_HANDLED;
}

static struct of_device_id tegra_soctherm_of_match[] = {
	{ .compatible = "nvidia,tegra124-soctherm" },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_soctherm_of_match);

static int tegra_soctherm_probe(struct platform_device *pdev)
{
	struct tegra_soctherm *tegra;
	struct thermal_zone_device *tz;
	struct tsensor_shared_calibration shared_calib;
	int i;
	int err = 0;
	int irq;

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

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev, "can't get interrupt\n");
		return -EINVAL;
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

	for (i = 0; tsensors[i].name; ++i) {
		err = enable_tsensor(tegra, tsensors + i, shared_calib);
		if (err)
			goto disable_clocks;
	}

	soctherm_writel(tegra, SENSOR_PDIV_T124, SENSOR_PDIV);
	soctherm_writel(tegra, SENSOR_HOTSPOT_OFF_T124, SENSOR_HOTSPOT_OFF);

	/* Initialize thermctl sensors */

	for (i = 0; tegra124_tsensor_groups[i]; ++i) {
		struct tegra_thermctl_zone *zone =
			devm_kzalloc(&pdev->dev, sizeof(*zone), GFP_KERNEL);
		if (!zone) {
			err = -ENOMEM;
			goto unregister_tzs;
		}

		zone->sensor_group = tegra124_tsensor_groups[i];
		zone->tegra = tegra;

		tz = thermal_zone_of_sensor_register(
			&pdev->dev, i, zone, tegra_thermctl_get_temp, NULL,
			tegra_thermctl_set_trips);
		if (IS_ERR(tz)) {
			err = PTR_ERR(tz);
			dev_err(&pdev->dev, "failed to register sensor: %d\n",
				err);
			--i;
			goto unregister_tzs;
		}

		zone->tz = tz;

		err = devm_request_threaded_irq(&pdev->dev, irq, soctherm_isr,
						soctherm_isr_thread,
						IRQF_SHARED, "tegra_soctherm",
						zone);
		if (err) {
			dev_err(&pdev->dev, "unable to register isr: %d\n",
				err);
			goto unregister_tzs;
		}

		soctherm_writel(tegra,
				0x3 << zone->sensor_group->thermctl_isr_shift,
				THERMCTL_INTR_EN);
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
		.owner = THIS_MODULE,
		.of_match_table = tegra_soctherm_of_match,
	},
};
module_platform_driver(tegra_soctherm_driver);

MODULE_AUTHOR("Mikko Perttunen <mperttunen@nvidia.com>");
MODULE_DESCRIPTION("Tegra SOCTHERM thermal management driver");
MODULE_LICENSE("GPL v2");
