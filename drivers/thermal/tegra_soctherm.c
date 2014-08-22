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

#include <linux/debugfs.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/thermal.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <soc/tegra/fuse.h>

#include <dt-bindings/thermal/tegra124-soctherm.h>

#include "tegra_soctherm.h"

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

#define		THERMCTL_LEVEL0_GROUP_EN	BIT(8)
#define		THERMCTL_LEVEL0_GROUP_DN_THRESH_SHIFT 9
#define		THERMCTL_LEVEL0_GROUP_UP_THRESH_SHIFT 17

#define CTL_LVL0_CPU0				0x0
#define CTL_LVL0_CPU0_UP_THRESH_SHIFT		17
#define CTL_LVL0_CPU0_UP_THRESH_MASK		0xff
#define CTL_LVL0_CPU0_DN_THRESH_SHIFT		9
#define CTL_LVL0_CPU0_DN_THRESH_MASK		0xff
#define CTL_LVL0_CPU0_EN_SHIFT			8
#define CTL_LVL0_CPU0_EN_MASK			0x1
#define CTL_LVL0_CPU0_CPU_THROT_SHIFT		5
#define CTL_LVL0_CPU0_CPU_THROT_MASK		0x3
#define CTL_LVL0_CPU0_CPU_THROT_LIGHT		0x1
#define CTL_LVL0_CPU0_CPU_THROT_HEAVY		0x2
#define CTL_LVL0_CPU0_GPU_THROT_SHIFT		3
#define CTL_LVL0_CPU0_GPU_THROT_MASK		0x3
#define CTL_LVL0_CPU0_GPU_THROT_LIGHT		0x1
#define CTL_LVL0_CPU0_GPU_THROT_HEAVY		0x2
#define CTL_LVL0_CPU0_MEM_THROT_SHIFT		2
#define CTL_LVL0_CPU0_MEM_THROT_MASK		0x1
#define CTL_LVL0_CPU0_STATUS_SHIFT		0
#define CTL_LVL0_CPU0_STATUS_MASK		0x3

#define THERMCTL_INTR_STATUS			0x84
#define THERMCTL_INTR_EN			0x88
#define THERMCTL_INTR_DISABLE			0x8c

#define TH_INTR_POS_GD0_SHIFT		17
#define TH_INTR_POS_GD0_MASK		0x1
#define TH_INTR_POS_GU0_SHIFT		16
#define TH_INTR_POS_GU0_MASK		0x1
#define TH_INTR_POS_CD0_SHIFT		9
#define TH_INTR_POS_CD0_MASK		0x1
#define TH_INTR_POS_CU0_SHIFT		8
#define TH_INTR_POS_CU0_MASK		0x1
#define TH_INTR_POS_PD0_SHIFT		1
#define TH_INTR_POS_PD0_MASK		0x1
#define TH_INTR_POS_PU0_SHIFT		0
#define TH_INTR_POS_PU0_MASK		0x1

#define TH_INTR_POS_IGNORE_MASK		0xfffbfbfb

#define STATS_CTL				0x94
#define STATS_CTL_CLR_DN			0x8
#define STATS_CTL_EN_DN				0x4
#define STATS_CTL_CLR_UP			0x2
#define STATS_CTL_EN_UP				0x1

#define TS_CPU0_CONFIG0				0xc0
#define TS_CPU0_CONFIG0_TALL_SHIFT		8
#define TS_CPU0_CONFIG0_TALL_MASK		0xfffff
#define TS_CPU0_CONFIG0_TCALC_OVER_SHIFT	4
#define TS_CPU0_CONFIG0_TCALC_OVER_MASK		0x1
#define TS_CPU0_CONFIG0_OVER_SHIFT		3
#define TS_CPU0_CONFIG0_OVER_MASK		0x1
#define TS_CPU0_CONFIG0_CPTR_OVER_SHIFT		2
#define TS_CPU0_CONFIG0_CPTR_OVER_MASK		0x1
#define TS_CPU0_CONFIG0_STOP_SHIFT		0
#define TS_CPU0_CONFIG0_STOP_MASK		0x1

#define TS_CPU0_CONFIG1				0xc4
#define TS_CPU0_CONFIG1_EN_SHIFT		31
#define TS_CPU0_CONFIG1_EN_MASK			0x1
#define TS_CPU0_CONFIG1_TIDDQ_SHIFT		15
#define TS_CPU0_CONFIG1_TIDDQ_MASK		0x3f
#define TS_CPU0_CONFIG1_TEN_COUNT_SHIFT		24
#define TS_CPU0_CONFIG1_TEN_COUNT_MASK		0x3f
#define TS_CPU0_CONFIG1_TSAMPLE_SHIFT		0
#define TS_CPU0_CONFIG1_TSAMPLE_MASK		0x3ff

#define TS_CPU0_CONFIG2				0xc8
#define TS_CPU0_CONFIG2_THERM_A_SHIFT		16
#define TS_CPU0_CONFIG2_THERM_A_MASK		0xffff
#define TS_CPU0_CONFIG2_THERM_B_SHIFT		0
#define TS_CPU0_CONFIG2_THERM_B_MASK		0xffff

#define TS_CPU0_STATUS0				0xcc
#define TS_CPU0_STATUS0_VALID_SHIFT		31
#define TS_CPU0_STATUS0_VALID_MASK		0x1
#define TS_CPU0_STATUS0_CAPTURE_SHIFT		0
#define TS_CPU0_STATUS0_CAPTURE_MASK		0xffff

#define TS_CPU0_STATUS1				0xd0
#define TS_CPU0_STATUS1_TEMP_VALID_SHIFT	31
#define TS_CPU0_STATUS1_TEMP_VALID_MASK		0x1
#define TS_CPU0_STATUS1_TEMP_SHIFT		0
#define TS_CPU0_STATUS1_TEMP_MASK		0xffff

#define TS_PDIV					0x1c0
#define TS_PDIV_CPU_SHIFT			12
#define TS_PDIV_CPU_MASK			0xf
#define TS_PDIV_GPU_SHIFT			8
#define TS_PDIV_GPU_MASK			0xf
#define TS_PDIV_MEM_SHIFT			4
#define TS_PDIV_MEM_MASK			0xf
#define TS_PDIV_PLLX_SHIFT			0
#define TS_PDIV_PLLX_MASK			0xf

#define TS_HOTSPOT_OFF				0x1c4
#define TS_HOTSPOT_OFF_CPU_SHIFT		16
#define TS_HOTSPOT_OFF_CPU_MASK			0xff
#define TS_HOTSPOT_OFF_GPU_SHIFT		8
#define TS_HOTSPOT_OFF_GPU_MASK			0xff
#define TS_HOTSPOT_OFF_MEM_SHIFT		0
#define TS_HOTSPOT_OFF_MEM_MASK			0xff

#define TS_TEMP1				0x1c8
#define TS_TEMP1_CPU_TEMP_SHIFT			16
#define TS_TEMP1_CPU_TEMP_MASK			0xffff
#define TS_TEMP1_GPU_TEMP_SHIFT			0
#define TS_TEMP1_GPU_TEMP_MASK			0xffff

#define TS_TEMP2				0x1cc
#define TS_TEMP2_MEM_TEMP_SHIFT			16
#define TS_TEMP2_MEM_TEMP_MASK			0xffff
#define TS_TEMP2_PLLX_TEMP_SHIFT		0
#define TS_TEMP2_PLLX_TEMP_MASK			0xffff

#define OC1_CFG				0x310
#define OC1_CFG_LONG_LATENCY_SHIFT	6
#define OC1_CFG_LONG_LATENCY_MASK	0x1
#define OC1_CFG_HW_RESTORE_SHIFT	5
#define OC1_CFG_HW_RESTORE_MASK		0x1
#define OC1_CFG_PWR_GOOD_MASK_SHIFT	4
#define OC1_CFG_PWR_GOOD_MASK_MASK	0x1
#define OC1_CFG_THROTTLE_MODE_SHIFT	2
#define OC1_CFG_THROTTLE_MODE_MASK	0x3
#define OC1_CFG_ALARM_POLARITY_SHIFT	1
#define OC1_CFG_ALARM_POLARITY_MASK	0x1
#define OC1_CFG_EN_THROTTLE_SHIFT	0
#define OC1_CFG_EN_THROTTLE_MASK	0x1

#define OC1_CNT_THRESHOLD		0x314
#define OC1_THROTTLE_PERIOD		0x318
#define OC1_ALARM_COUNT			0x31c
#define OC1_FILTER			0x320

#define OC1_STATS			0x3a8

#define OC_INTR_STATUS			0x39c
#define OC_INTR_ENABLE			0x3a0
#define OC_INTR_DISABLE			0x3a4
#define OC_INTR_POS_OC1_SHIFT		0
#define OC_INTR_POS_OC1_MASK		0x1
#define OC_INTR_POS_OC2_SHIFT		1
#define OC_INTR_POS_OC2_MASK		0x1
#define OC_INTR_POS_OC3_SHIFT		2
#define OC_INTR_POS_OC3_MASK		0x1
#define OC_INTR_POS_OC4_SHIFT		3
#define OC_INTR_POS_OC4_MASK		0x1
#define OC_INTR_POS_OC5_SHIFT		4
#define OC_INTR_POS_OC5_MASK		0x1

#define OC_STATS_CTL			0x3c4
#define OC_STATS_CTL_CLR_ALL		0x2
#define OC_STATS_CTL_EN_ALL		0x1

#define THROT_GLOBAL_CFG			0x400
#define THROT_GLOBAL_ENB_SHIFT			0
#define THROT_GLOBAL_ENB_MASK			0x1

#define CPU_PSKIP_STATUS			0x418
#define GPU_PSKIP_STATUS			0x41c
#define XPU_PSKIP_STATUS_M_SHIFT		12
#define XPU_PSKIP_STATUS_M_MASK			0xff
#define XPU_PSKIP_STATUS_N_SHIFT		4
#define XPU_PSKIP_STATUS_N_MASK			0xff
#define XPU_PSKIP_STATUS_SW_OVERRIDE_SHIFT	1
#define XPU_PSKIP_STATUS_SW_OVERRIDE_MASK	0x1
#define XPU_PSKIP_STATUS_ENABLED_SHIFT		0
#define XPU_PSKIP_STATUS_ENABLED_MASK		0x1

#define THROT_PRIORITY_LOCK			0x424
#define THROT_PRIORITY_LOCK_PRIORITY_SHIFT	0
#define THROT_PRIORITY_LOCK_PRIORITY_MASK	0xff

#define THROT_STATUS				0x428
#define THROT_STATUS_BREACH_SHIFT		12
#define THROT_STATUS_BREACH_MASK		0x1
#define THROT_STATUS_STATE_SHIFT		4
#define THROT_STATUS_STATE_MASK			0xff
#define THROT_STATUS_ENABLED_SHIFT		0
#define THROT_STATUS_ENABLED_MASK		0x1

#define THROT_PSKIP_CTRL_LITE_CPU		0x430
#define THROT_PSKIP_CTRL_ENABLE_SHIFT		31
#define THROT_PSKIP_CTRL_ENABLE_MASK		0x1
#define THROT_PSKIP_CTRL_DIVIDEND_SHIFT		8
#define THROT_PSKIP_CTRL_DIVIDEND_MASK		0xff
#define THROT_PSKIP_CTRL_DIVISOR_SHIFT		0
#define THROT_PSKIP_CTRL_DIVISOR_MASK		0xff
#define THROT_PSKIP_CTRL_VECT_GPU_SHIFT		16
#define THROT_PSKIP_CTRL_VECT_GPU_MASK		0x7
#define THROT_PSKIP_CTRL_VECT_CPU_SHIFT		8
#define THROT_PSKIP_CTRL_VECT_CPU_MASK		0x7
#define THROT_PSKIP_CTRL_VECT2_CPU_SHIFT	0
#define THROT_PSKIP_CTRL_VECT2_CPU_MASK		0x7

#define THROT_PSKIP_RAMP_LITE_CPU		0x434
#define THROT_PSKIP_RAMP_SEQ_BYPASS_MODE_SHIFT	31
#define THROT_PSKIP_RAMP_SEQ_BYPASS_MODE_MASK	0x1
#define THROT_PSKIP_RAMP_DURATION_SHIFT		8
#define THROT_PSKIP_RAMP_DURATION_MASK		0xffff
#define THROT_PSKIP_RAMP_STEP_SHIFT		0
#define THROT_PSKIP_RAMP_STEP_MASK		0xff

#define THROT_PRIORITY_LITE			0x444
#define THROT_PRIORITY_LITE_PRIO_SHIFT		0
#define THROT_PRIORITY_LITE_PRIO_MASK		0xff

#define THROT_DELAY_LITE			0x448
#define THROT_DELAY_LITE_DELAY_SHIFT		0
#define THROT_DELAY_LITE_DELAY_MASK		0xff

#define CCROC_GLOBAL_CFG			0x148

#define CCROC_THROT_PSKIP_CTRL_CPU		0x154
#define CCROC_THROT_PSKIP_CTRL_ENB_SHIFT	31
#define CCROC_THROT_PSKIP_CTRL_ENB_MASK		0x1
#define CCROC_THROT_PSKIP_CTRL_DIVIDEND_SHIFT	8
#define CCROC_THROT_PSKIP_CTRL_DIVIDEND_MASK	0xff
#define CCROC_THROT_PSKIP_CTRL_DIVISOR_SHIFT	0
#define CCROC_THROT_PSKIP_CTRL_DIVISOR_MASK	0xff

#define CCROC_THROT_PSKIP_RAMP_CPU		0x150
#define CCROC_THROT_PSKIP_RAMP_SEQ_BYPASS_MODE_SHIFT	31
#define CCROC_THROT_PSKIP_RAMP_SEQ_BYPASS_MODE_MASK	0x1
#define CCROC_THROT_PSKIP_RAMP_DURATION_SHIFT	8
#define CCROC_THROT_PSKIP_RAMP_DURATION_MASK	0xffff
#define CCROC_THROT_PSKIP_RAMP_STEP_SHIFT	0
#define CCROC_THROT_PSKIP_RAMP_STEP_MASK	0xff

/* car register offsets needed for enabling HW throttling */
#define CAR_SUPER_CCLKG_DIVIDER			0x36c
#define CDIVG_ENABLE_SHIFT			31
#define CDIVG_ENABLE_MASK			0x1
#define CDIVG_USE_THERM_CONTROLS_SHIFT		30
#define CDIVG_USE_THERM_CONTROLS_MASK		0x1
#define CDIVG_DIVIDEND_MASK			0xff
#define CDIVG_DIVIDEND_SHIFT			8
#define CDIVG_DIVISOR_MASK			0xff
#define CDIVG_DIVISOR_SHIFT			0

#define CCROC_SUPER_CCLKG_DIVIDER		0x024

#define UP_STATS_L0				0x10
#define DN_STATS_L0				0x14

#define THROT_VECT_NONE				0x0 /* 3'b000 */
#define THROT_VECT_LOW				0x1 /* 3'b001 */
#define THROT_VECT_MED				0x3 /* 3'b011 */
#define THROT_VECT_HIGH				0x7 /* 3'b111 */

#define THROT_OFFSET				0x30
#define CCROC_THROT_OFFSET			0x0c
#define ALARM_OFFSET				0x14

#define THROT_PSKIP_CTRL(throt, dev)		(THROT_PSKIP_CTRL_LITE_CPU + \
						(THROT_OFFSET * throt) + \
						(8 * dev))
#define THROT_PSKIP_RAMP(throt, dev)		(THROT_PSKIP_RAMP_LITE_CPU + \
						(THROT_OFFSET * throt) + \
						(8 * dev))
#define CCROC_THROT_PSKIP_CTRL_CPU_REG(vect)	(CCROC_THROT_PSKIP_CTRL_CPU + \
						(CCROC_THROT_OFFSET * vect))
#define CCROC_THROT_PSKIP_RAMP_CPU_REG(vect)	(CCROC_THROT_PSKIP_RAMP_CPU + \
						(CCROC_THROT_OFFSET * vect))

#define THROT_PRIORITY_CTRL(throt)		(THROT_PRIORITY_LITE + \
						(THROT_OFFSET * throt))
#define THROT_DELAY_CTRL(throt)			(THROT_DELAY_LITE + \
						(THROT_OFFSET * throt))

#define ALARM_CFG(throt)			(OC1_CFG + \
						(ALARM_OFFSET * (throt - \
								THROTTLE_OC1)))
#define ALARM_CNT_THRESHOLD(throt)		(OC1_CNT_THRESHOLD + \
						(ALARM_OFFSET * (throt - \
								THROTTLE_OC1)))
#define ALARM_THROTTLE_PERIOD(throt)		(OC1_THROTTLE_PERIOD + \
						(ALARM_OFFSET * (throt - \
								THROTTLE_OC1)))
#define ALARM_ALARM_COUNT(throt)		(OC1_ALARM_COUNT + \
						(ALARM_OFFSET * (throt - \
								THROTTLE_OC1)))
#define ALARM_FILTER(throt)			(OC1_FILTER + \
						(ALARM_OFFSET * (throt - \
								THROTTLE_OC1)))
#define ALARM_STATS(throt)			(OC1_STATS + \
						(4 * (throt - THROTTLE_OC1)))

#define REG_SET(r, _name, val)	(((r) & ~(_name##_MASK << _name##_SHIFT)) | \
				(((val) & _name##_MASK) << _name##_SHIFT))
#define REG_GET_BIT(r, _name)	((r) & (_name##_MASK << _name##_SHIFT))
#define REG_GET(r, _name)	(REG_GET_BIT(r, _name) >> _name##_SHIFT)

#define TS_TSENSE_REGS_SIZE		0x20
#define TS_TSENSE_REG_OFFSET(reg, ts)	((reg) + ((ts) * TS_TSENSE_REGS_SIZE))

#define TS_THERM_LVL_REGS_SIZE		0x20
#define TS_THERM_GRP_REGS_SIZE		0x04
#define TS_THERM_REG_OFFSET(rg, lv, gr)	((rg) + ((lv) * TS_THERM_LVL_REGS_SIZE)\
					+ ((gr) * TS_THERM_GRP_REGS_SIZE))

#define THROT_DEPTH_DIVIDEND(depth)	((256 * (100 - (depth)) / 100) - 1)

enum soctherm_throttle_id {
	THROTTLE_LIGHT = 0,
	THROTTLE_HEAVY,
	THROTTLE_OC1,
	THROTTLE_OC2,
	THROTTLE_OC3,
	THROTTLE_OC4,
	THROTTLE_OC5,
	THROTTLE_SIZE,
};

enum soctherm_throttle_level {
	THROTTLE_LEVEL_LOW = 0,
	THROTTLE_LEVEL_MED,
	THROTTLE_LEVEL_HIGH,
	THROTTLE_LEVEL_SIZE,
	THROTTLE_LEVEL_NONE = -1,
};

enum soctherm_throttle_dev_id {
	THROTTLE_DEV_CPU = 0,
	THROTTLE_DEV_GPU,
	THROTTLE_DEV_SIZE,
	THROTTLE_DEV_NONE,
};

enum soctherm_oc_irq_id {
	TEGRA_SOC_OC_IRQ_1,
	TEGRA_SOC_OC_IRQ_2,
	TEGRA_SOC_OC_IRQ_3,
	TEGRA_SOC_OC_IRQ_4,
	TEGRA_SOC_OC_IRQ_5,
	TEGRA_SOC_OC_IRQ_NUM,
};

enum throt_mode {
	DISABLED = 0,
	STICKY,
	BRIEF,
	RESERVED,
};

static const char *const throt_names[] = {
	[THROTTLE_LIGHT]   = "light",
	[THROTTLE_HEAVY]   = "heavy",
	[THROTTLE_OC1]     = "oc1",
	[THROTTLE_OC2]     = "oc2",
	[THROTTLE_OC3]     = "oc3",
	[THROTTLE_OC4]     = "oc4",
	[THROTTLE_OC5]     = "oc5", /* reserved */
};

static const char *const throt_dev_names[] = {
	[THROTTLE_DEV_CPU] = "CPU",
	[THROTTLE_DEV_GPU] = "GPU",
};

/* SOC- OCx to theirt GPIO which is wakeup capable */
static int soctherm_ocx_to_wake_gpio[TEGRA_SOC_OC_IRQ_NUM] = {
	243,		/* TEGRA_GPIO_PEE3,	TEGRA_SOC_OC_IRQ_1 */
	-1,		/* TEGRA_GPIO_INVALID,	TEGRA_SOC_OC_IRQ_2 */
	-1,		/* TEGRA_GPIO_INVALID,	TEGRA_SOC_OC_IRQ_3 */
	74,		/* TEGRA_GPIO_PJ2,	TEGRA_SOC_OC_IRQ_4 */
	-1,		/* TEGRA_GPIO_INVALID,	TEGRA_SOC_OC_IRQ_5 */
};

static const int min_low_temp = -127000;
static const int max_high_temp = 127000;

struct soctherm_oc_irq_chip_data {
	struct mutex		irq_lock; /* serialize OC IRQs */
	struct irq_chip		irq_chip;
	struct irq_domain	*domain;
	int			irq_enable;
};

struct tegra_soctherm {
	struct platform_device *pdev;
	struct reset_control *reset;
	struct clk *clock_tsensor;
	struct clk *clock_soctherm;

	unsigned int thermal_irq;

	void __iomem *regs;
	void __iomem *clk_regs;
	void __iomem *ccroc_regs;

	struct thermal_zone_device *thermctl_tzs[4];
	struct tegra_tsensor_group **sensor_groups;
	struct tegra_tsensor *tsensors;
	struct tsensor_shared_calibration *shared_calib;
	struct soctherm_oc_irq_chip_data *soc_irq_cdata;

	bool is_ccroc;
};

struct tegra_thermctl_zone {
	struct tegra_soctherm *tegra;
	struct tegra_tsensor_group *sensor_group;
	struct thermal_zone_device *tz;
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

/**
 * clk_writel() - writes a value to a CAR register
 * @ts: pointer to a struct tegra_soctherm
 * @v: the value to write
 * @reg: the register offset
 *
 * Writes @v to @reg.  No return value.
 */
static inline void clk_writel(struct tegra_soctherm *ts, u32 value, u32 reg)
{
	__raw_writel(value, (ts->clk_regs + reg));
}

/**
 * clk_readl() - reads specified register from CAR IP block
 * @ts: pointer to a struct tegra_soctherm
 * @reg: register address to be read
 *
 * Return: the value of the register
 */
static inline u32 clk_readl(struct tegra_soctherm *ts, u32 reg)
{
	return __raw_readl(ts->clk_regs + reg);
}

/**
 * ccroc_writel() - writes a value to a CCROC register
 * @ts: pointer to a struct tegra_soctherm
 * @v: the value to write
 * @reg: the register offset
 *
 * Writes @v to @reg.  No return value.
 */
static inline void ccroc_writel(struct tegra_soctherm *ts, u32 value, u32 reg)
{
	__raw_writel(value, (ts->ccroc_regs + reg));
}

/**
 * ccroc_readl() - reads specified register from CCROC IP block
 * @ts: pointer to a struct tegra_soctherm
 * @reg: register address to be read
 *
 * Return: the value of the register
 */
static inline u32 ccroc_readl(struct tegra_soctherm *ts, u32 reg)
{
	return __raw_readl(ts->ccroc_regs + reg);
}

static void enable_tsensor(struct tegra_soctherm *tegra,
			   struct tegra_tsensor *sensor)
{
	unsigned int val;

	val = 0;
	val |= sensor->config->tall << SENSOR_CONFIG0_TALL_SHIFT;
	soctherm_writel(tegra, val, sensor->base + SENSOR_CONFIG0);

	val = 0;
	val |= (sensor->config->tsample - 1) << SENSOR_CONFIG1_TSAMPLE_SHIFT;
	val |= sensor->config->tiddq_en << SENSOR_CONFIG1_TIDDQ_EN_SHIFT;
	val |= sensor->config->ten_count << SENSOR_CONFIG1_TEN_COUNT_SHIFT;
	val |= SENSOR_CONFIG1_TEMP_ENABLE;
	soctherm_writel(tegra, val, sensor->base + SENSOR_CONFIG1);

	soctherm_writel(tegra, sensor->calib, sensor->base + SENSOR_CONFIG2);
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

/**
 * soctherm_thermal_isr() - thermal interrupt request handler
 * @irq:	Interrupt request number
 * @arg:	Not used.
 *
 * Reads the thermal interrupt status and then disables any asserted
 * interrupts. The thread woken by this isr services the asserted
 * interrupts and re-enables them.
 *
 * Return: %IRQ_WAKE_THREAD
 */
static irqreturn_t soctherm_thermal_isr(int irq, void *dev_id)
{
	struct tegra_soctherm *ts = dev_id;
	u32 r;

	r = soctherm_readl(ts, THERMCTL_INTR_STATUS);

	soctherm_writel(ts, r, THERMCTL_INTR_DISABLE);

	return IRQ_WAKE_THREAD;
}

/**
 * soctherm_thermal_isr_thread() - Handles a thermal interrupt request
 * @irq:	The interrupt number being requested; not used
 * @arg:	Opaque pointer to an argument; not used
 *
 * Clears the interrupt status register if there are expected
 * interrupt bits set.
 * The interrupt(s) are then handled by updating the corresponding
 * thermal zones.
 *
 * An error is logged if any unexpected interrupt bits are set.
 *
 * Disabled interrupts are re-enabled.
 *
 * Return: %IRQ_HANDLED. Interrupt was handled and no further processing
 * is needed.
 */
static irqreturn_t soctherm_thermal_isr_thread(int irq, void *dev_id)
{
	struct tegra_soctherm *ts = dev_id;
	struct thermal_zone_device *tz;
	u32 st, ex = 0, cp = 0, gp = 0, pl = 0;

	st = soctherm_readl(ts, THERMCTL_INTR_STATUS);

	/* deliberately clear expected interrupts handled in SW */
	cp |= REG_GET_BIT(st, TH_INTR_POS_CD0);
	cp |= REG_GET_BIT(st, TH_INTR_POS_CU0);
	ex |= cp;

	gp |= REG_GET_BIT(st, TH_INTR_POS_GD0);
	gp |= REG_GET_BIT(st, TH_INTR_POS_GU0);
	ex |= gp;

	pl |= REG_GET_BIT(st, TH_INTR_POS_PD0);
	pl |= REG_GET_BIT(st, TH_INTR_POS_PU0);
	ex |= pl;

	if (ex) {
		soctherm_writel(ts, ex, THERMCTL_INTR_STATUS);
		st &= ~ex;
		if (cp) {
			tz = ts->thermctl_tzs[TEGRA124_SOCTHERM_SENSOR_CPU];
			thermal_zone_device_update(tz);
		}
		if (gp) {
			tz = ts->thermctl_tzs[TEGRA124_SOCTHERM_SENSOR_GPU];
			thermal_zone_device_update(tz);
		}
		if (pl) {
			tz = ts->thermctl_tzs[TEGRA124_SOCTHERM_SENSOR_PLLX];
			thermal_zone_device_update(tz);
		}
	}

	/* deliberately ignore expected interrupts NOT handled in SW */
	ex |= TH_INTR_POS_IGNORE_MASK;
	st &= ~ex;

	if (st) {
		/* Whine about any other unexpected INTR bits still set */
		pr_err("soctherm: Ignored unexpected INTRs 0x%08x\n", st);
		soctherm_writel(ts, st, THERMCTL_INTR_STATUS);
	}

	return IRQ_HANDLED;
}

/**
 * soctherm_edp_isr() - Disables any active interrupts
 * @irq:	The interrupt request number
 * @arg:	Opaque pointer to an argument
 *
 * Writes to the OC_INTR_DISABLE register the over current interrupt status,
 * masking any asserted interrupts. Doing this prevents the same interrupts
 * from triggering this isr repeatedly. The thread woken by this isr will
 * handle asserted interrupts and subsequently unmask/re-enable them.
 *
 * The OC_INTR_DISABLE register indicates which OC interrupts
 * have been disabled.
 *
 * Return: %IRQ_WAKE_THREAD, handler requests to wake the handler thread
 */
static irqreturn_t soctherm_edp_isr(int irq, void *dev_id)
{
	struct tegra_soctherm *ts = dev_id;
	u32 r;

	r = soctherm_readl(ts, OC_INTR_STATUS);
	soctherm_writel(ts, r, OC_INTR_DISABLE);

	return IRQ_WAKE_THREAD;
}

/**
 * soctherm_oc_intr_enable() - Enables the soctherm over-current interrupt
 * @alarm:		The soctherm throttle id
 * @enable:		Flag indicating enable the soctherm over-current
 *			interrupt or disable it
 *
 * Enables a specific over-current pins @alarm to raise an interrupt if the flag
 * is set and the alarm corresponds to OC1, OC2, OC3, or OC4.
 */
static void soctherm_oc_intr_enable(struct tegra_soctherm *ts,
				    enum soctherm_throttle_id alarm,
				    bool enable)
{
	u32 r;

	if (!enable)
		return;

	r = soctherm_readl(ts, OC_INTR_ENABLE);
	switch (alarm) {
	case THROTTLE_OC1:
		r = REG_SET(r, OC_INTR_POS_OC1, 1);
		break;
	case THROTTLE_OC2:
		r = REG_SET(r, OC_INTR_POS_OC2, 1);
		break;
	case THROTTLE_OC3:
		r = REG_SET(r, OC_INTR_POS_OC3, 1);
		break;
	case THROTTLE_OC4:
		r = REG_SET(r, OC_INTR_POS_OC4, 1);
		break;
	default:
		r = 0;
		break;
	}
	soctherm_writel(ts, r, OC_INTR_ENABLE);
}

/**
 * soctherm_handle_alarm() - Handles soctherm alarms
 * @alarm:		The soctherm throttle id
 *
 * "Handles" over-current alarms (OC1, OC2, OC3, and OC4) by printing
 * a warning or informative message.
 *
 * Return: -EINVAL for @alarm = THROTTLE_OC3, otherwise 0 (success).
 */
static int soctherm_handle_alarm(struct tegra_soctherm *ts,
				 enum soctherm_throttle_id alarm)
{
	struct platform_device *pdev = ts->pdev;
	int rv = -EINVAL;

	switch (alarm) {
	case THROTTLE_OC1:
		dev_dbg(&pdev->dev,
			"soctherm: Successfully handled OC1 alarm\n");
		/* add OC1 alarm handling code here */
		rv = 0;
		break;

	case THROTTLE_OC2:
		dev_dbg(&pdev->dev,
			"soctherm: Successfully handled OC2 alarm\n");
		/* TODO: add OC2 alarm handling code here */
		rv = 0;
		break;

	case THROTTLE_OC3:
		dev_dbg(&pdev->dev,
			"soctherm: Unexpected OC3 alarm\n");
		/* add OC3 alarm handling code here */
		break;

	case THROTTLE_OC4:
		dev_dbg(&pdev->dev,
			"soctherm: Successfully handled OC4 alarm\n");
		/* TODO: add OC4 alarm handling code here */
		rv = 0;
		break;

	default:
		break;
	}

	if (rv)
		dev_err(&pdev->dev,
			"soctherm: ERROR in handling %s alarm\n",
			throt_names[alarm]);

	return rv;
}

/**
 * soctherm_edp_isr_thread() - log an over-current interrupt request
 * @irq:	OC irq number. Currently not being used. See description
 * @arg:	a void pointer for callback, currently not being used
 *
 * Over-current events are handled in hardware. This function is called to log
 * and handle any OC events that happened. Additionally, it checks every
 * over-current interrupt registers for registers are set but
 * was not expected (i.e. any discrepancy in interrupt status) by the function,
 * the discrepancy will logged.
 *
 * Return: %IRQ_HANDLED
 */
static irqreturn_t soctherm_edp_isr_thread(int irq, void *dev_id)
{
	struct tegra_soctherm *ts = dev_id;
	struct soctherm_oc_irq_chip_data *soc_irq_cdata = ts->soc_irq_cdata;
	u32 st, ex, oc1, oc2, oc3, oc4;

	st = soctherm_readl(ts, OC_INTR_STATUS);

	/* deliberately clear expected interrupts handled in SW */
	oc1 = REG_GET_BIT(st, OC_INTR_POS_OC1);
	oc2 = REG_GET_BIT(st, OC_INTR_POS_OC2);
	oc3 = REG_GET_BIT(st, OC_INTR_POS_OC3);
	oc4 = REG_GET_BIT(st, OC_INTR_POS_OC4);
	ex = oc1 | oc2 | oc3 | oc4;

	if (ex) {
		soctherm_writel(ts, st, OC_INTR_STATUS);
		st &= ~ex;

		if (oc1 && !soctherm_handle_alarm(ts, THROTTLE_OC1))
			soctherm_oc_intr_enable(ts, THROTTLE_OC1, true);

		if (oc2 && !soctherm_handle_alarm(ts, THROTTLE_OC2))
			soctherm_oc_intr_enable(ts, THROTTLE_OC2, true);

		if (oc3 && !soctherm_handle_alarm(ts, THROTTLE_OC3))
			soctherm_oc_intr_enable(ts, THROTTLE_OC3, true);

		if (oc4 && !soctherm_handle_alarm(ts, THROTTLE_OC4))
			soctherm_oc_intr_enable(ts, THROTTLE_OC4, true);

		if (oc1 && soc_irq_cdata->irq_enable & BIT(0))
			handle_nested_irq(
				irq_find_mapping(soc_irq_cdata->domain, 0));

		if (oc2 && soc_irq_cdata->irq_enable & BIT(1))
			handle_nested_irq(
				irq_find_mapping(soc_irq_cdata->domain, 1));

		if (oc3 && soc_irq_cdata->irq_enable & BIT(2))
			handle_nested_irq(
				irq_find_mapping(soc_irq_cdata->domain, 2));

		if (oc4 && soc_irq_cdata->irq_enable & BIT(3))
			handle_nested_irq(
				irq_find_mapping(soc_irq_cdata->domain, 3));
	}

	if (st) {
		dev_err(&ts->pdev->dev,
			"soctherm: Ignored unexpected OC ALARM 0x%08x\n", st);
		soctherm_writel(ts, st, OC_INTR_STATUS);
	}

	return IRQ_HANDLED;
}

/*
 * Thermtrip
 */


/**
 * enforce_temp_range() - check and enforce temperature range [min, max]
 * @trip_temp: the trip temperature to check
 *
 * Checks and enforces the permitted temperature range that SOC_THERM
 * HW can support with 8-bit registers to specify temperature. This is
 * done while taking care of precision.
 *
 * Return: The precision adjusted capped temperature in millicelsius.
 */
static int enforce_temp_range(struct device *dev, long trip_temp)
{
	long temp = trip_temp;

	if (temp < min_low_temp) {
		dev_info(dev, "soctherm: trip_point temp %ld forced to %d\n",
			 trip_temp, min_low_temp);
		temp = min_low_temp;
	} else if (temp > max_high_temp) {
		dev_info(dev, "soctherm: trip_point temp %ld forced to %d\n",
			 trip_temp, max_high_temp);
		temp = max_high_temp;
	}

	return temp;
}

/**
 * thermtrip_clear() - disable thermtrip for a sensor
 * @dev: ptr to the struct device for the SOC_THERM IP block
 * @sg: pointer to the sensor group to disable thermtrip for
 *
 * Disables thermtrip for the sensor group @sg on SOC_THERM device @dev.
 * Intended to be used when THERMTRIP is not explicitly configured for
 * a sensor, and the sensor's calibration is bad or not supplied.
 *
 * Return: 0 upon success, or %-EINVAL upon failure.
 */
static int thermtrip_clear(struct device *dev, struct tegra_tsensor_group *sg)
{
	struct tegra_soctherm *ts = dev_get_drvdata(dev);
	u32 r;

	if (!dev || !sg)
		return -EINVAL;

	if (!sg->thermtrip_threshold_mask)
		return -EINVAL;

	r = soctherm_readl(ts, THERMTRIP);

	r &= ~sg->thermtrip_threshold_mask;
	r |= 0 << sg->thermtrip_enable_shift;

	dev_err(dev, "thermtrip mask %08x\n", sg->thermtrip_threshold_mask);
	dev_err(dev, "thermtrip enable shift %d\n", sg->thermtrip_enable_shift);

	r = REG_SET(r, THERMTRIP_ANY_EN, 0);

	dev_err(dev, "write %08x to thermtrip\n", r);
	soctherm_writel(ts, r, THERMTRIP);
	soctherm_barrier(ts);

	return 0;
}

/**
 * thermtrip_program() - Configures the hardware to shut down the
 * system if a given sensor group reaches a given temperature
 * @dev: ptr to the struct device for the SOC_THERM IP block
 * @sg: pointer to the sensor group to set the thermtrip temperature for
 * @trip_temp: the temperature in millicelsius to trigger the thermal trip at
 *
 * Sets the thermal trip threshold of the given sensor group to be the
 * @trip_temp.  If this threshold is crossed, the hardware will shut
 * down.
 *
 * Note that, although @trip_temp is specified in millicelsius, the
 * hardware is programmed in degrees Celsius.
 *
 * Return: 0 upon success, or %-EINVAL upon failure.
 */
static int thermtrip_program(struct device *dev, struct tegra_tsensor_group *sg,
			     long trip_temp)
{
	struct tegra_soctherm *ts = dev_get_drvdata(dev);
	u32 r;
	int temp;

	if (!dev || !sg)
		return -EINVAL;

	if (!sg->thermtrip_threshold_mask)
		return -EINVAL;

	temp = enforce_temp_range(dev, trip_temp) / 1000;

	/* XXX Do some sanity-checking here */

	r = soctherm_readl(ts, THERMTRIP);

	r &= ~sg->thermtrip_threshold_mask;
	r |= temp << (ffs(sg->thermtrip_threshold_mask) - 1);

	r |= 1 << sg->thermtrip_enable_shift;
	r = REG_SET(r, THERMTRIP_ANY_EN, 0);

	soctherm_writel(ts, r, THERMTRIP);
	soctherm_barrier(ts);

	return 0;
}

/**
 * find_sensor_group_by_name() - look up a thermal sensor group by name
 * @name: name of the thermal sensor group to look up
 *
 * Look up a SOC_THERM thermal sensor group by its name @name, and
 * return a pointer to that thermal sensor group's data record if
 * found.
 *
 * Return: a pointer to a struct tegra_tsensor_group upon success, or
 * NULL upon failure.
 */
static struct tegra_tsensor_group *find_sensor_group_by_name(
						struct tegra_soctherm *ts,
						const char *name)
{
	int i;

	for (i = 0; ts->sensor_groups[i]->name; i++)
		if (!strcmp(ts->sensor_groups[i]->name, name))
			return ts->sensor_groups[i];

	return NULL;
}

/**
 * thermtrip_configure_limits_from_dt() - configure thermal shutdown limits
 * @dev: struct device * of the SOC_THERM instance
 * @ttn: struct device_node * of the "thermtrip" node in DT
 *
 * Read the maximum thermal limits that the SoC has been configured to
 * operate at from DT data, and configure the SOC_THERM IP block @dev
 * to reset the SoC and turn off the PMIC when the internal sensor
 * group temperatures cross those limits.
 *
 * Return: 0 upon success or a negative error code upon failure.
 */
static int thermtrip_configure_limits_from_dt(struct device *dev,
					      struct device_node *ttn)
{
	struct tegra_soctherm *ts = dev_get_drvdata(dev);
	struct tegra_tsensor_group *sg;
	struct device_node *sgn, *sgsn;
	const char *name;
	u32 temperature;
	int r;

	/* Read the limits */
	sgsn = of_find_node_by_name(dev->of_node, "sensor-groups");
	if (!sgsn) {
		dev_info(dev, "thermtrip: no sensor-groups node - not enabling\n");
		return 0;
	}
	for_each_child_of_node(sgsn, sgn) {
		name = sgn->name;
		sg = find_sensor_group_by_name(ts, name);
		if (!sg) {
			dev_err(dev, "thermtrip: %s: could not find sensor group - could not enable\n",
				name);
			continue;
		}

		if (sg->flags & SKIP_THERMTRIP_REGISTRATION) {
			dev_info(dev, "thermtrip: %s: skipping due to chip revision\n",
				 name);
			thermtrip_clear(dev, sg);
			continue;
		}

		r = of_property_read_u32(sgn, "therm-temp", &temperature);
		if (r) {
			dev_err(dev, "thermtrip: %s: missing temperature property - could not enable\n",
				name);
			continue;
		}

		r = thermtrip_program(dev, sg, temperature);
		if (r) {
			dev_err(dev, "thermtrip: %s: error during enable\n",
				name);
			continue;
		}

		dev_info(dev, "thermtrip: will shut down when %s sensor group reaches %d degrees millicelsius\n",
			 name, temperature);
	}

	return 0;
}

/**
 * thermtrip_configure_from_dt() - configure thermal shutdown from DT data
 * @dev: struct device * of the SOC_THERM instance
 *
 * Configure the SOC_THERM "THERMTRIP" feature, using data from DT.
 * After it's been configured, THERMTRIP will take action when the
 * configured SoC thermal sensor group reaches a certain temperature.
 * It will assert an internal SoC reset line, and will signal the
 * boot-ROM to tell the PMIC to turn off (if PMIC information has been
 * provided).
 *
 * SOC_THERM registers are in the VDD_SOC voltage domain.  This means
 * that SOC_THERM THERMTRIP programming does not survive an LP0/SC7
 * transition, unless this driver has been modified to save those
 * registers before entering SC7 and restore them upon exiting SC7.
 *
 * Return: 0 upon success, or a negative error code on failure.
 * "Success" does not mean that thermtrip was enabled; it could also
 * mean that no "thermtrip" node was found in DT.  THERMTRIP has been
 * enabled successfully when a message similar to this one appears on
 * the serial console: "thermtrip: will shut down when sensor group
 * XXX reaches YYYYYY millidegrees C"
 */
static int thermtrip_configure_from_dt(struct device *dev)
{
	struct device_node *ttn;
	int r;

	ttn = of_find_node_by_name(dev->of_node, "hw-trips");
	if (!ttn) {
		dev_info(dev, "thermtrip: no DT node - not enabling\n");
		return 0;
	}

	r = thermtrip_configure_limits_from_dt(dev, ttn);
	if (r)
		return r;

	return 0;
}

static inline void prog_hw_threshold(struct device *dev,
				     long trip_temp,
				     struct tegra_tsensor_group *sg,
				     int throt)
{
	struct tegra_soctherm *ts = dev_get_drvdata(dev);
	int temp, cpu_throt, gpu_throt;
	u32 r, reg_off;

	temp = enforce_temp_range(dev, trip_temp) / 1000;

	/* Hardcode LITE on level-1 and HEAVY on level-2 */
	reg_off = TS_THERM_REG_OFFSET(CTL_LVL0_CPU0, throt + 1, sg->id);

	if (throt == THROTTLE_LIGHT) {
		cpu_throt = CTL_LVL0_CPU0_CPU_THROT_LIGHT;
		gpu_throt = CTL_LVL0_CPU0_GPU_THROT_LIGHT;
	} else {
		cpu_throt = CTL_LVL0_CPU0_CPU_THROT_HEAVY;
		gpu_throt = CTL_LVL0_CPU0_GPU_THROT_HEAVY;
		if (throt != THROTTLE_HEAVY)
			pr_warn("soctherm: invalid throt %d - assuming HEAVY",
				throt);
	}

	r = soctherm_readl(ts, reg_off);
	r = REG_SET(r, CTL_LVL0_CPU0_UP_THRESH, temp);
	r = REG_SET(r, CTL_LVL0_CPU0_DN_THRESH, temp);
	r = REG_SET(r, CTL_LVL0_CPU0_CPU_THROT, cpu_throt);
	r = REG_SET(r, CTL_LVL0_CPU0_GPU_THROT, gpu_throt);
	r = REG_SET(r, CTL_LVL0_CPU0_EN, 1);
	soctherm_writel(ts, r, reg_off);
}

static int throttrip_program(struct device *dev, struct tegra_tsensor_group *sg,
			     long trip_temp)
{
	if (!dev || !sg)
		return -EINVAL;

	prog_hw_threshold(dev, trip_temp, sg, THROTTLE_HEAVY);

	return 0;
}

static int throttrip_configure_limits_from_dt(struct device *dev,
					      struct device_node *ttn)
{
	struct tegra_soctherm *ts = dev_get_drvdata(dev);
	struct tegra_tsensor_group *sg;
	struct device_node *sgn, *sgsn;
	const char *name;
	u32 temperature;
	int r;

	/* Read the limits */
	sgsn = of_find_node_by_name(dev->of_node, "sensor-groups");
	if (!sgsn) {
		dev_info(dev,
			"throttle-trip: no sensor-groups node - not enabling\n");
		return 0;
	}
	for_each_child_of_node(sgsn, sgn) {
		name = sgn->name;
		sg = find_sensor_group_by_name(ts, name);
		if (!sg) {
			dev_err(dev,
				"throtlte-trip: %s: could not find sensor group - could not enable\n",
				name);
			continue;
		}

		r = of_property_read_u32(sgn, "throt-temp", &temperature);
		if (r) {
			dev_info(dev,
				"throttle-trip: %s: missing temperature property - could not enable\n",
				name);
			continue;
		}

		r = throttrip_program(dev, sg, temperature);
		if (r) {
			dev_err(dev, "throttle-trip: %s: error during enable\n",
				name);
			continue;
		}

		dev_info(dev,
			"throttle-trip: will hw throttle when %s sensor group reaches %d degrees millicelsius\n",
			name, temperature);
	}

	return 0;
}

static int throttrip_configure_from_dt(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *ttn;
	int r;

	ttn = of_find_node_by_name(dev->of_node, "hw-trips");
	if (!ttn) {
		dev_info(dev, "throttle-trip: no DT node - not enabling\n");
		return 0;
	}

	r = throttrip_configure_limits_from_dt(dev, ttn);
	if (r)
		return r;

	return 0;
}

static int soctherm_clk_enable(struct platform_device *pdev, bool enable)
{
	struct tegra_soctherm *tegra = platform_get_drvdata(pdev);
	int err;

	if (tegra->clock_soctherm == NULL || tegra->clock_tsensor == NULL)
		return -EINVAL;

	reset_control_assert(tegra->reset);

	if (enable) {
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
	} else {
		clk_disable_unprepare(tegra->clock_tsensor);
		clk_disable_unprepare(tegra->clock_soctherm);
	}

	reset_control_deassert(tegra->reset);

	return 0;
}

/**
 * throttlectl_cpu_level_cfg() - programs CCROC NV_THERM level config
 * @throt	soctherm_throttle_id describing the level of throttling
 *
 * It's necessary to set up the CPU-local CCROC NV_THERM instance with
 * the M/N values desired for each level. This function does this.
 *
 * This function pre-programs the CCROC NV_THERM levels in terms of
 * pre-configured "Low", "Medium" or "Heavy" throttle levels which are
 * mapped to THROT_LEVEL_LOW, THROT_LEVEL_MED and THROT_LEVEL_HVY.
 *
 * Return: boolean true if HW was programmed
 */
static void throttlectl_cpu_level_cfg(struct tegra_soctherm *ts, int level)
{
	u8 depth, dividend;
	u32 r;

	switch (level) {
	case THROTTLE_LEVEL_LOW:
		depth = 50;
		break;
	case THROTTLE_LEVEL_MED:
		depth = 75;
		break;
	case THROTTLE_LEVEL_HIGH:
		depth = 80;
		break;
	case THROTTLE_LEVEL_NONE:
		return;
	default:
		return;
	}

	dividend = THROT_DEPTH_DIVIDEND(depth);

	/* setup PSKIP in ccroc nv_therm registers */
	r = ccroc_readl(ts, CCROC_THROT_PSKIP_RAMP_CPU_REG(level));
	r = REG_SET(r, CCROC_THROT_PSKIP_RAMP_DURATION, 0xff);
	r = REG_SET(r, CCROC_THROT_PSKIP_RAMP_STEP, 0xf);
	ccroc_writel(ts, r, CCROC_THROT_PSKIP_RAMP_CPU_REG(level));

	r = ccroc_readl(ts, CCROC_THROT_PSKIP_CTRL_CPU_REG(level));
	r = REG_SET(r, CCROC_THROT_PSKIP_CTRL_ENB, 1);
	r = REG_SET(r, CCROC_THROT_PSKIP_CTRL_DIVIDEND, dividend);
	r = REG_SET(r, CCROC_THROT_PSKIP_CTRL_DIVISOR, 0xff);
	ccroc_writel(ts, r, CCROC_THROT_PSKIP_CTRL_CPU_REG(level));
}

/**
 * throttlectl_cpu_level_select() - program CPU pulse skipper config
 * @throt: soctherm_throttle_id describing the level of throttling
 *
 * Pulse skippers are used to throttle clock frequencies.  This
 * function programs the pulse skippers based on @throt and platform
 * data.  This function is used on SoCs which have CPU-local pulse
 * skipper control, such as T13x. It programs soctherm's interface to
 * Denver:CCROC NV_THERM in terms of Low, Medium and Heavy throttling
 * vectors. PSKIP_BYPASS mode is set as required per HW spec.
 *
 * Return: boolean true if HW was programmed, or false if the desired
 * configuration is not supported.
 */
static bool throttlectl_cpu_level_select(struct tegra_soctherm *ts,
					 enum soctherm_throttle_id throt)
{
	u32 r, throt_vect;

	/* Denver:CCROC NV_THERM interface N:3 Mapping */
	switch (throt) {
	case THROTTLE_LIGHT:
		throt_vect = THROT_VECT_LOW;
		break;
	case THROTTLE_HEAVY:
		throt_vect = THROT_VECT_HIGH;
		break;
	case THROTTLE_OC1:
		throt_vect = THROT_VECT_MED;
		break;
	case THROTTLE_OC4:
		throt_vect = THROT_VECT_LOW;
		break;
	default:
		throt_vect = THROT_VECT_NONE;
		break;
	}

	r = soctherm_readl(ts, THROT_PSKIP_CTRL(throt, THROTTLE_DEV_CPU));
	r = REG_SET(r, THROT_PSKIP_CTRL_ENABLE, 1);
	r = REG_SET(r, THROT_PSKIP_CTRL_VECT_CPU, throt_vect);
	r = REG_SET(r, THROT_PSKIP_CTRL_VECT2_CPU, throt_vect);
	soctherm_writel(ts, r, THROT_PSKIP_CTRL(throt, THROTTLE_DEV_CPU));

	/* bypass sequencer in soc_therm as it is programmed in ccroc */
	r = REG_SET(0, THROT_PSKIP_RAMP_SEQ_BYPASS_MODE, 1);
	soctherm_writel(ts, r, THROT_PSKIP_RAMP(throt, THROTTLE_DEV_CPU));

	return true;
}

/**
 * throttlectl_cpu_mn() - program CPU pulse skipper configuration
 * @throt: soctherm_throttle_id describing the level of throttling
 *
 * Pulse skippers are used to throttle clock frequencies.  This
 * function programs the pulse skippers based on @throt and platform
 * data.  This function is used for CPUs that have "remote" pulse
 * skipper control, e.g., the CPU pulse skipper is controlled by the
 * SOC_THERM IP block.  (SOC_THERM is located outside the CPU
 * complex.)
 *
 * Return: boolean true if HW was programmed, or false if the desired
 * configuration is not supported.
 */
static bool throttlectl_cpu_mn(struct tegra_soctherm *ts,
			       enum soctherm_throttle_id throt)
{
	u32 r;
	u8 dividend;

	switch (throt) {
	case THROTTLE_LIGHT:
		dividend = 229;
		break;
	case THROTTLE_HEAVY:
		dividend = 51;
		break;
	default:
		return false;
	}

	r = soctherm_readl(ts, THROT_PSKIP_CTRL(throt, THROTTLE_DEV_CPU));
	r = REG_SET(r, THROT_PSKIP_CTRL_ENABLE, 1);
	r = REG_SET(r, THROT_PSKIP_CTRL_DIVIDEND, dividend);
	r = REG_SET(r, THROT_PSKIP_CTRL_DIVISOR, 0xff);
	soctherm_writel(ts, r, THROT_PSKIP_CTRL(throt, THROTTLE_DEV_CPU));

	r = soctherm_readl(ts, THROT_PSKIP_RAMP(throt, THROTTLE_DEV_CPU));
	r = REG_SET(r, THROT_PSKIP_RAMP_DURATION, 0xff);
	r = REG_SET(r, THROT_PSKIP_RAMP_STEP, 0xf);
	soctherm_writel(ts, r, THROT_PSKIP_RAMP(throt, THROTTLE_DEV_CPU));

	return true;
}

/**
 * soctherm_throttle_program() - programs pulse skippers' configuration
 * @throt	soctherm_throttle_id describing the level of throttling
 *
 * Pulse skippers are used to throttle clock frequencies.
 * This function programs the pulse skippers based on @throt and platform data.
 *
 * Return: Nothing is returned (void).
 */
static void soctherm_throttle_program(struct tegra_soctherm *ts,
				      enum soctherm_throttle_id throt,
				      u8 priority,
				      u32 alarm_cnt_threshold)
{
	u32 r;

	/* Setup PSKIP parameters */
	if (ts->is_ccroc)
		throttlectl_cpu_level_select(ts, throt);
	else
		throttlectl_cpu_mn(ts, throt);

	r = REG_SET(0, THROT_PRIORITY_LITE_PRIO, priority);
	soctherm_writel(ts, r, THROT_PRIORITY_CTRL(throt));

	r = REG_SET(0, THROT_DELAY_LITE_DELAY, 0);
	soctherm_writel(ts, r, THROT_DELAY_CTRL(throt));

	r = soctherm_readl(ts, THROT_PRIORITY_LOCK);
	if (r < priority) {
		r = REG_SET(0, THROT_PRIORITY_LOCK_PRIORITY, priority);
		soctherm_writel(ts, r, THROT_PRIORITY_LOCK);
	}

	/* ----- configure reserved OC5 alarm ----- */
	if (throt == THROTTLE_OC5) {
		r = soctherm_readl(ts, ALARM_CFG(throt));
		r = REG_SET(r, OC1_CFG_THROTTLE_MODE, BRIEF);
		r = REG_SET(r, OC1_CFG_ALARM_POLARITY, 0);
		r = REG_SET(r, OC1_CFG_EN_THROTTLE, 1);
		soctherm_writel(ts, r, ALARM_CFG(throt));

		r = REG_SET(r, OC1_CFG_ALARM_POLARITY, 1);
		soctherm_writel(ts, r, ALARM_CFG(throt));

		r = REG_SET(r, OC1_CFG_ALARM_POLARITY, 0);
		soctherm_writel(ts, r, ALARM_CFG(throt));

		r = REG_SET(r, THROT_PSKIP_CTRL_DIVIDEND, 0);
		r = REG_SET(r, THROT_PSKIP_CTRL_DIVISOR, 0);
		r = REG_SET(r, THROT_PSKIP_CTRL_ENABLE, 1);
		soctherm_writel(ts, r,
				THROT_PSKIP_CTRL(throt, THROTTLE_DEV_CPU));

		r = soctherm_readl(ts,
				THROT_PSKIP_RAMP(throt, THROTTLE_DEV_CPU));
		r = REG_SET(r, THROT_PSKIP_RAMP_DURATION, 0xff);
		r = REG_SET(r, THROT_PSKIP_RAMP_STEP, 0xf);
		soctherm_writel(ts, r,
				THROT_PSKIP_RAMP(throt, THROTTLE_DEV_CPU));

		r = soctherm_readl(ts,
				THROT_PSKIP_CTRL(throt, THROTTLE_DEV_GPU));
		r = REG_SET(r, THROT_PSKIP_CTRL_ENABLE, 1);
		soctherm_writel(ts, r,
				THROT_PSKIP_CTRL(throt, THROTTLE_DEV_GPU));

		r = REG_SET(0, THROT_PRIORITY_LITE_PRIO, 1);
		soctherm_writel(ts, r,
				THROT_PRIORITY_CTRL(throt));
		return;
	}

	if (throt < THROTTLE_OC1)
		return;

	/* ----- configure other OC alarms ----- */

	r = soctherm_readl(ts, ALARM_CFG(throt));
	r = REG_SET(r, OC1_CFG_HW_RESTORE, 1);
	r = REG_SET(r, OC1_CFG_PWR_GOOD_MASK, 0);
	r = REG_SET(r, OC1_CFG_THROTTLE_MODE, BRIEF);
	r = REG_SET(r, OC1_CFG_ALARM_POLARITY, 50);
	r = REG_SET(r, OC1_CFG_EN_THROTTLE, 1);
	soctherm_writel(ts, r, ALARM_CFG(throt));

	soctherm_oc_intr_enable(ts, throt, true);

	soctherm_writel(ts, 0, ALARM_THROTTLE_PERIOD(throt)); /* usec */
	soctherm_writel(ts, alarm_cnt_threshold, ALARM_CNT_THRESHOLD(throt));
	soctherm_writel(ts, 5100000, ALARM_FILTER(throt));
}

static int tegra_soctherm_hw_throttle(struct platform_device *pdev)
{
	struct tegra_soctherm *ts = platform_get_drvdata(pdev);
	bool is_ccroc;

	if (!ts)
		return -EINVAL;

	is_ccroc = ts->is_ccroc;

	/* configure low, med and heavy levels for CCROC NV_THERM */
	if (is_ccroc) {
		throttlectl_cpu_level_cfg(ts, THROTTLE_LEVEL_LOW);
		throttlectl_cpu_level_cfg(ts, THROTTLE_LEVEL_MED);
		throttlectl_cpu_level_cfg(ts, THROTTLE_LEVEL_HIGH);
	}

	/* Thermal HW throttle programming */
	soctherm_throttle_program(ts, THROTTLE_HEAVY, THROTTLE_HEAVY + 0xE, 0);
	soctherm_throttle_program(ts, THROTTLE_OC1, 50, 100);
	soctherm_throttle_program(ts, THROTTLE_OC4, 50, 15);

	throttrip_configure_from_dt(pdev);

	return 0;
}

static int soctherm_init_platform_data(struct platform_device *pdev)
{
	struct tegra_soctherm *tegra = platform_get_drvdata(pdev);
	struct tegra_tsensor *tsensors = tegra->tsensors;
	struct tegra_tsensor_group **tegra_tsensor_groups;
	int i;
	u32 v;

	tegra_tsensor_groups = tegra->sensor_groups;

	/* Enable thermal clocks */
	if (soctherm_clk_enable(pdev, true) < 0) {
		dev_err(&pdev->dev, "enable clocks failed\n");
		return -EINVAL;
	}

	/* Initialize raw sensors */
	for (i = 0; tsensors[i].name; ++i)
		enable_tsensor(tegra, tsensors + i);

	/* Wait for sensor data to be ready */
	usleep_range(1000, 5000);

	/* Initialize thermctl sensors */
	for (i = 0; tegra_tsensor_groups[i]; ++i) {
		struct tegra_tsensor_group *ttg;

		ttg = tegra_tsensor_groups[i];

		v = soctherm_readl(tegra, TS_PDIV);
		v &= ~ttg->pdiv_mask;
		v |= ttg->pdiv << (ffs(ttg->pdiv_mask) - 1);
		soctherm_writel(tegra, v, TS_PDIV);

		v = soctherm_readl(tegra, TS_HOTSPOT_OFF);
		v &= ~ttg->pllx_hotspot_mask;
		v |= (ttg->pllx_hotspot_diff / 1000) <<
			(ffs(ttg->pllx_hotspot_mask) - 1);
		soctherm_writel(tegra, v, TS_HOTSPOT_OFF);

		if (!(ttg->flags & SKIP_THERMAL_FW_REGISTRATION)) {
			soctherm_writel(tegra,
			     0x3 << tegra_tsensor_groups[i]->thermctl_isr_shift,
			     THERMCTL_INTR_EN);
		}
	}

	/* Set up hardware thermal limits */
	if (thermtrip_configure_from_dt(&pdev->dev)) {
		dev_err(&pdev->dev, "configure thermtrip failed\n");
		return -EINVAL;
	}

	/* Set up hardware throttle */
	if (tegra_soctherm_hw_throttle(pdev)) {
		dev_err(&pdev->dev, "configure HW throttle trip failed\n");
		return -EINVAL;
	}

	v = REG_SET(0, THROT_GLOBAL_ENB, 1);
	if (tegra->is_ccroc)
		ccroc_writel(tegra, v, CCROC_GLOBAL_CFG);
	else
		soctherm_writel(tegra, v, THROT_GLOBAL_CFG);

	if (tegra->is_ccroc) {
		v = ccroc_readl(tegra, CCROC_SUPER_CCLKG_DIVIDER);
		v = REG_SET(v, CDIVG_USE_THERM_CONTROLS, 1);
		ccroc_writel(tegra, v, CCROC_SUPER_CCLKG_DIVIDER);
	} else {
		v = clk_readl(tegra, CAR_SUPER_CCLKG_DIVIDER);
		v = REG_SET(v, CDIVG_USE_THERM_CONTROLS, 1);
		clk_writel(tegra, v, CAR_SUPER_CCLKG_DIVIDER);
	}

	/* initialize stats collection */
	v = STATS_CTL_CLR_DN | STATS_CTL_EN_DN |
		STATS_CTL_CLR_UP | STATS_CTL_EN_UP;
	soctherm_writel(tegra, v, STATS_CTL);

	return 0;
}

/**
 * soctherm_oc_irq_lock() - locks the over-current interrupt request
 * @data:	Interrupt request data
 *
 * Looks up the chip data from @data and locks the mutex associated with
 * a particular over-current interrupt request.
 */
static void soctherm_oc_irq_lock(struct irq_data *data)
{
	struct soctherm_oc_irq_chip_data *d = irq_data_get_irq_chip_data(data);

	mutex_lock(&d->irq_lock);
}

/**
 * soctherm_oc_irq_sync_unlock() - Unlocks the OC interrupt request
 * @data:		Interrupt request data
 *
 * Looks up the interrupt request data @data and unlocks the mutex associated
 * with a particular over-current interrupt request.
 */
static void soctherm_oc_irq_sync_unlock(struct irq_data *data)
{
	struct soctherm_oc_irq_chip_data *d = irq_data_get_irq_chip_data(data);

	mutex_unlock(&d->irq_lock);
}

/**
 * soctherm_oc_irq_enable() - Enables the SOC_THERM over-current interrupt queue
 * @data:       irq_data structure of the chip
 *
 * Sets the irq_enable bit of SOC_THERM allowing SOC_THERM
 * to respond to over-current interrupts.
 *
 */
static void soctherm_oc_irq_enable(struct irq_data *data)
{
	struct soctherm_oc_irq_chip_data *d = irq_data_get_irq_chip_data(data);

	d->irq_enable |= BIT(data->hwirq);
}

/**
 * soctherm_oc_irq_disable() - Disables overcurrent interrupt requests
 * @irq_data:	The interrupt request information
 *
 * Clears the interrupt request enable bit of the overcurrent
 * interrupt request chip data.
 *
 * Return: Nothing is returned (void)
 */
static void soctherm_oc_irq_disable(struct irq_data *data)
{
	struct soctherm_oc_irq_chip_data *d = irq_data_get_irq_chip_data(data);

	d->irq_enable &= ~BIT(data->hwirq);
}

static int soctherm_oc_irq_set_type(struct irq_data *data, unsigned int type)
{
	return 0;
}

/**
 * soctherm_oc_irq_set_wake() - Set the overcurrent interrupt request
 * to "wake"
 * @irq_data:	Interrupt request information
 * @on:		Whether to enable or disable power management wakeup
 *
 * Configure the GPIO associated with a SOC_THERM over-current
 * interrupt to wake the system from sleep
 *
 * It may be necessary to wake the system from sleep mode so that
 * SOC_THERM can provide proper over-current throttling.
 *
 * Return: 0 on success, -EINVAL if there is no wakeup support
 * for that given hardware irq, or the gpio number if there is
 * no gpio_to_irq for that gpio.
 */
static int soctherm_oc_irq_set_wake(struct irq_data *data, unsigned int on)
{
	int gpio;
	int gpio_irq;

	gpio = soctherm_ocx_to_wake_gpio[data->hwirq];
	if (!gpio_is_valid(gpio)) {
		pr_err("No wakeup supported for irq %lu\n", data->hwirq);
		return -EINVAL;
	}

	gpio_irq = gpio_to_irq(gpio);
	if (gpio_irq < 0) {
		pr_err("No gpio_to_irq for gpio %d\n", gpio);
		return gpio;
	}

	irq_set_irq_wake(gpio_irq, on);
	return 0;
}

/**
 * soctherm_oc_irq_map() - SOC_THERM interrupt request domain mapper
 * @h:		Interrupt request domain
 * @virq:	Virtual interrupt request number
 * @hw:		Hardware interrupt request number
 *
 * Mapping callback function for SOC_THERM's irq_domain. When a SOC_THERM
 * interrupt request is called, the irq_domain takes the request's virtual
 * request number (much like a virtual memory address) and maps it to a
 * physical hardware request number.
 *
 * When a mapping doesn't already exist for a virtual request number, the
 * irq_domain calls this function to associate the virtual request number with
 * a hardware request number.
 *
 * Return: 0
 */
static int soctherm_oc_irq_map(struct irq_domain *h, unsigned int virq,
		irq_hw_number_t hw)
{
	struct soctherm_oc_irq_chip_data *data = h->host_data;

	irq_set_chip_data(virq, data);
	irq_set_chip(virq, &data->irq_chip);
	irq_set_nested_thread(virq, 1);
	set_irq_flags(virq, IRQF_VALID);
	return 0;
}

/**
 * soctherm_irq_domain_xlate_twocell() - xlate for soctherm interrupts
 * @d:      Interrupt request domain
 * @intspec:    Array of u32s from DTs "interrupt" property
 * @intsize:    Number of values inside the intspec array
 * @out_hwirq:  HW IRQ value associated with this interrupt
 * @out_type:   The IRQ SENSE type for this interrupt.
 *
 * This Device Tree IRQ specifier translation function will translate a
 * specific "interrupt" as defined by 2 DT values where the cell values map
 * the hwirq number + 1 and linux irq flags. Since the output is the hwirq
 * number, this function will subtract 1 from the value listed in DT.
 *
 * Return: 0
 */
static int soctherm_irq_domain_xlate_twocell(struct irq_domain *d,
	struct device_node *ctrlr, const u32 *intspec, unsigned int intsize,
	irq_hw_number_t *out_hwirq, unsigned int *out_type)
{
	if (WARN_ON(intsize < 2))
		return -EINVAL;

	/*
	 * The HW value is 1 index less than the DT IRQ values.
	 * i.e. OC4 goes to HW index 3.
	 */
	*out_hwirq = intspec[0] - 1;
	*out_type = intspec[1] & IRQ_TYPE_SENSE_MASK;
	return 0;
}

static struct irq_domain_ops soctherm_oc_domain_ops = {
	.map	= soctherm_oc_irq_map,
	.xlate	= soctherm_irq_domain_xlate_twocell,
};

/**
 * soctherm_oc_int_init() - Initial enabling of the over
 * current interrupts
 * @pdev:	platform device
 * @num_irqs:	The number of new interrupt requests

 *
 * Sets the over current interrupt request chip data
 *
 * Return: 0 on success or if overcurrent interrupts are not enabled,
 * -ENOMEM (out of memory), or irq_base if the function failed to
 * allocate the irqs
 */
static int soctherm_oc_int_init(struct platform_device *pdev, int num_irqs)
{
	struct device_node *np = pdev->dev.of_node;
	struct tegra_soctherm *ts = platform_get_drvdata(pdev);
	struct soctherm_oc_irq_chip_data *soc_irq_cdata;

	if (!num_irqs) {
		pr_info("%s(): OC interrupts are not enabled\n", __func__);
		return 0;
	}

	soc_irq_cdata = devm_kzalloc(&pdev->dev, sizeof(*soc_irq_cdata),
				     GFP_KERNEL);
	if (!soc_irq_cdata)
		return -ENOMEM;

	ts->soc_irq_cdata = soc_irq_cdata;

	mutex_init(&soc_irq_cdata->irq_lock);
	soc_irq_cdata->irq_enable = 0;

	soc_irq_cdata->irq_chip.name = "soc_therm_oc";
	soc_irq_cdata->irq_chip.irq_bus_lock = soctherm_oc_irq_lock;
	soc_irq_cdata->irq_chip.irq_bus_sync_unlock =
		soctherm_oc_irq_sync_unlock;
	soc_irq_cdata->irq_chip.irq_disable = soctherm_oc_irq_disable;
	soc_irq_cdata->irq_chip.irq_enable = soctherm_oc_irq_enable;
	soc_irq_cdata->irq_chip.irq_set_type = soctherm_oc_irq_set_type;
	soc_irq_cdata->irq_chip.irq_set_wake = soctherm_oc_irq_set_wake;

	soc_irq_cdata->domain = irq_domain_add_linear(np, num_irqs,
				&soctherm_oc_domain_ops, soc_irq_cdata);

	if (!soc_irq_cdata->domain) {
		dev_err(&pdev->dev,
			"%s: Failed to create IRQ domain\n", __func__);
		return -ENOMEM;
	}

	dev_dbg(&pdev->dev,
		"%s(): OC interrupts enabled successful\n", __func__);
	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int regs_show(struct seq_file *s, void *data)
{
	struct platform_device *pdev = s->private;
	struct tegra_soctherm *ts = platform_get_drvdata(pdev);
	struct tegra_tsensor *tsensors = ts->tsensors;
	struct tegra_tsensor_group **tsensor_groups = ts->sensor_groups;
	u32 r;
	u32 state;
	int i, j, level;
	uint m, n, q;
	char *depth;

	seq_puts(s, "-----TSENSE (convert HW)-----\n");

	for (i = 0; tsensors[i].name; i++) {
		r = soctherm_readl(ts,
				TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG1, i));
		state = REG_GET(r, TS_CPU0_CONFIG1_EN);
		if (!state)
			continue;

		seq_printf(s, "%s: ", tsensors[i].name);

		seq_printf(s, "En(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TIDDQ);
		seq_printf(s, "tiddq(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TEN_COUNT);
		seq_printf(s, "ten_count(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TSAMPLE);
		seq_printf(s, "tsample(%d) ", state + 1);

		r = soctherm_readl(ts,
				TS_TSENSE_REG_OFFSET(TS_CPU0_STATUS1, i));
		state = REG_GET(r, TS_CPU0_STATUS1_TEMP_VALID);
		seq_printf(s, "Temp(%d/", state);
		state = REG_GET(r, TS_CPU0_STATUS1_TEMP);
		seq_printf(s, "%ld) ", translate_temp(state));

		r = soctherm_readl(ts,
				TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG0, i));
		state = REG_GET(r, TS_CPU0_CONFIG0_STOP);
		seq_printf(s, "Stop(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_TALL);
		seq_printf(s, "Tall(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_TCALC_OVER);
		seq_printf(s, "Over(%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_OVER);
		seq_printf(s, "%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_CPTR_OVER);
		seq_printf(s, "%d) ", state);

		r = soctherm_readl(ts,
				TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG2, i));
		state = REG_GET(r, TS_CPU0_CONFIG2_THERM_A);
		seq_printf(s, "Therm_A/B(%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG2_THERM_B);
		seq_printf(s, "%d)\n", (s16)state);
	}

	r = soctherm_readl(ts, TS_PDIV);
	seq_printf(s, "PDIV: 0x%x\n", r);

	seq_puts(s, "\n");
	seq_puts(s, "-----SOC_THERM-----\n");

	r = soctherm_readl(ts, TS_TEMP1);
	state = REG_GET(r, TS_TEMP1_CPU_TEMP);
	seq_printf(s, "Temperatures: CPU(%ld) ", translate_temp(state));
	state = REG_GET(r, TS_TEMP1_GPU_TEMP);
	seq_printf(s, " GPU(%ld) ", translate_temp(state));
	r = soctherm_readl(ts, TS_TEMP2);
	state = REG_GET(r, TS_TEMP2_PLLX_TEMP);
	seq_printf(s, " PLLX(%ld) ", translate_temp(state));
	state = REG_GET(r, TS_TEMP2_MEM_TEMP);
	seq_printf(s, " MEM(%ld)\n", translate_temp(state));

	for (i = 0; tsensor_groups[i]; i++) {
		seq_printf(s, "%s:\n", tsensor_groups[i]->name);
		for (level = 0; level < 4; level++) {
			r = soctherm_readl(ts,
					TS_THERM_REG_OFFSET(CTL_LVL0_CPU0,
					level, i));
			state = REG_GET(r, CTL_LVL0_CPU0_UP_THRESH);
			seq_printf(s, "   %d: Up/Dn(%d/", level, state);
			state = REG_GET(r, CTL_LVL0_CPU0_DN_THRESH);
			seq_printf(s, "%d) ", (s8)state);
			state = REG_GET(r, CTL_LVL0_CPU0_EN);
			seq_printf(s, "En(%d) ", state);

			state = REG_GET(r, CTL_LVL0_CPU0_CPU_THROT);
			seq_puts(s, "CPU Throt");
			seq_printf(s, "(%s) ", state ?
			state == CTL_LVL0_CPU0_CPU_THROT_LIGHT ? "L" :
			state == CTL_LVL0_CPU0_CPU_THROT_HEAVY ? "H" :
				"H+L" : "none");

			state = REG_GET(r, CTL_LVL0_CPU0_GPU_THROT);
			seq_puts(s, "GPU Throt");
			seq_printf(s, "(%s) ", state ?
			state == CTL_LVL0_CPU0_GPU_THROT_LIGHT ? "L" :
			state == CTL_LVL0_CPU0_GPU_THROT_HEAVY ? "H" :
				"H+L" : "none");

			state = REG_GET(r, CTL_LVL0_CPU0_STATUS);
			seq_printf(s, "Status(%s)\n",
				   state == 0 ? "LO" :
				   state == 1 ? "in" :
				   state == 2 ? "??" : "HI");
		}
	}

	r = soctherm_readl(ts, STATS_CTL);
	seq_printf(s, "STATS: Up(%s) Dn(%s)\n",
		   r & STATS_CTL_EN_UP ? "En" : "--",
		   r & STATS_CTL_EN_DN ? "En" : "--");
	for (level = 0; level < 4; level++) {
		r = soctherm_readl(ts,
				TS_TSENSE_REG_OFFSET(UP_STATS_L0, level));
		seq_printf(s, "  Level_%d Up(%d) ", level, r);
		r = soctherm_readl(ts,
				TS_TSENSE_REG_OFFSET(DN_STATS_L0, level));
		seq_printf(s, "Dn(%d)\n", r);
	}

	r = soctherm_readl(ts, THERMTRIP);
	state = REG_GET(r, THERMTRIP_ANY_EN);
	seq_printf(s, "ThermTRIP ANY En(%d)\n", state);

	state = REG_GET(r, THERMTRIP_CPU_EN);
	seq_printf(s, "     CPU En(%d) ", state);
	state = REG_GET(r, THERMTRIP_CPU_THRESH);
	seq_printf(s, "Thresh(%d)\n", state);

	state = REG_GET(r, THERMTRIP_GPU_EN);
	seq_printf(s, "     GPU En(%d) ", state);
	state = REG_GET(r, THERMTRIP_GPUMEM_THRESH);
	seq_printf(s, "Thresh(%d)\n", state);

	state = REG_GET(r, THERMTRIP_MEM_EN);
	seq_printf(s, "     MEM En(%d) ", state);
	state = REG_GET(r, THERMTRIP_GPUMEM_THRESH);
	seq_printf(s, "Thresh(%d)\n", state);

	state = REG_GET(r, THERMTRIP_TSENSE_EN);
	seq_printf(s, "    PLLX En(%d) ", state);
	state = REG_GET(r, THERMTRIP_TSENSE_THRESH);
	seq_printf(s, "Thresh(%d)\n", state);

	r = soctherm_readl(ts, THROT_GLOBAL_CFG);
	seq_printf(s, "GLOBAL THROTTLE CONFIG: 0x%08x\n", r);

	seq_puts(s, "---------------------------------------------------\n");
	r = soctherm_readl(ts, THROT_STATUS);
	state = REG_GET(r, THROT_STATUS_BREACH);
	seq_printf(s, "THROT STATUS: breach(%d) ", state);
	state = REG_GET(r, THROT_STATUS_STATE);
	seq_printf(s, "state(%d) ", state);
	state = REG_GET(r, THROT_STATUS_ENABLED);
	seq_printf(s, "enabled(%d)\n", state);

	r = soctherm_readl(ts, CPU_PSKIP_STATUS);
	if (ts->is_ccroc) {
		state = REG_GET(r, XPU_PSKIP_STATUS_ENABLED);
		seq_printf(s, "%s PSKIP STATUS: ",
			   throt_dev_names[THROTTLE_DEV_CPU]);
		seq_printf(s, "enabled(%d)\n", state);
	} else {
		state = REG_GET(r, XPU_PSKIP_STATUS_M);
		seq_printf(s, "%s PSKIP STATUS: M(%d) ",
			   throt_dev_names[THROTTLE_DEV_CPU], state);
		state = REG_GET(r, XPU_PSKIP_STATUS_N);
		seq_printf(s, "N(%d) ", state);
		state = REG_GET(r, XPU_PSKIP_STATUS_ENABLED);
		seq_printf(s, "enabled(%d)\n", state);
	}

	r = soctherm_readl(ts, GPU_PSKIP_STATUS);
	state = REG_GET(r, XPU_PSKIP_STATUS_ENABLED);
	seq_printf(s, "%s PSKIP STATUS: ",
		   throt_dev_names[THROTTLE_DEV_GPU]);
	seq_printf(s, "enabled(%d)\n", state);

	seq_puts(s, "---------------------------------------------------\n");
	seq_puts(s, "THROTTLE control and PSKIP configuration:\n");
	seq_printf(s, "%5s  %3s  %2s  %7s  %8s  %7s  %8s  %4s  %4s  %5s  ",
		   "throt", "dev", "en", " depth ", "dividend", "divisor",
		   "duration", "step", "prio", "delay");
	seq_printf(s, "%2s  %2s  %2s  %2s  %2s  %2s  ",
		   "LL", "HW", "PG", "MD", "01", "EN");
	seq_printf(s, "%8s  %8s  %8s  %8s  %8s\n",
		   "thresh", "period", "count", "filter", "stats");

	/* display throttle_cfg's of all alarms including OC5 */
	for (i = 0; i < THROTTLE_SIZE; i++) {
		for (j = 0; j < THROTTLE_DEV_SIZE; j++) {
			r = soctherm_readl(ts, THROT_PSKIP_CTRL(i, j));
			state = REG_GET(r, THROT_PSKIP_CTRL_ENABLE);
			seq_printf(s, "%5s  %3s  %2d  ",
				   j ? "" : throt_names[i],
				   throt_dev_names[j], state);
			if (!state) {
				seq_puts(s, "\n");
				continue;
			}

			level = THROTTLE_LEVEL_NONE; /* invalid */
			depth = "";
			q = 0;
			if (ts->is_ccroc && j == THROTTLE_DEV_CPU) {
				state = REG_GET(r, THROT_PSKIP_CTRL_VECT_CPU);
				if (state == THROT_VECT_HIGH) {
					level = THROTTLE_LEVEL_HIGH;
					depth = "hi";
				} else if (state == THROT_VECT_MED) {
					level = THROTTLE_LEVEL_MED;
					depth = "med";
				} else if (state == THROT_VECT_LOW) {
					level = THROTTLE_LEVEL_LOW;
					depth = "low";
				}
			}
			if (j == THROTTLE_DEV_GPU) {
				state = REG_GET(r, THROT_PSKIP_CTRL_VECT_GPU);
				/* Mapping is hard-coded in gk20a:nv_therm */
				if (state == THROT_VECT_HIGH) {
					q = 87;
					depth = "hi";
				} else if (state == THROT_VECT_MED) {
					q = 75;
					depth = "med";
				} else if (state == THROT_VECT_LOW) {
					q = 50;
					depth = "low";
				}
			}

			if (level == THROTTLE_LEVEL_NONE)
				r = 0;
			else if (ts->is_ccroc && j == THROTTLE_DEV_CPU)
				r = ccroc_readl(ts,
					CCROC_THROT_PSKIP_CTRL_CPU_REG(level));
			else
				r = soctherm_readl(ts, THROT_PSKIP_CTRL(i, j));

			m = REG_GET(r, THROT_PSKIP_CTRL_DIVIDEND);
			n = REG_GET(r, THROT_PSKIP_CTRL_DIVISOR);
			q = q ?: 100 - (((100 * (m+1)) + ((n+1) / 2)) / (n+1));
			seq_printf(s, "%2u%% %3s  ", q, depth);
			seq_printf(s, "%8u  ", m);
			seq_printf(s, "%7u  ", n);

			if (ts->is_ccroc && j == THROTTLE_DEV_CPU)
				r = ccroc_readl(ts,
					CCROC_THROT_PSKIP_RAMP_CPU_REG(level));
			else
				r = soctherm_readl(ts, THROT_PSKIP_RAMP(i, j));

			state = REG_GET(r, THROT_PSKIP_RAMP_DURATION);
			seq_printf(s, "%8d  ", state);
			state = REG_GET(r, THROT_PSKIP_RAMP_STEP);
			seq_printf(s, "%4d  ", state);

			r = soctherm_readl(ts, THROT_PRIORITY_CTRL(i));
			state = REG_GET(r, THROT_PRIORITY_LITE_PRIO);
			seq_printf(s, "%4d  ", state);

			r = soctherm_readl(ts, THROT_DELAY_CTRL(i));
			state = REG_GET(r, THROT_DELAY_LITE_DELAY);
			seq_printf(s, "%5d  ", state);

			if (i >= THROTTLE_OC1) {
				r = soctherm_readl(ts, ALARM_CFG(i));
				state = REG_GET(r, OC1_CFG_LONG_LATENCY);
				seq_printf(s, "%2d  ", state);
				state = REG_GET(r, OC1_CFG_HW_RESTORE);
				seq_printf(s, "%2d  ", state);
				state = REG_GET(r, OC1_CFG_PWR_GOOD_MASK);
				seq_printf(s, "%2d  ", state);
				state = REG_GET(r, OC1_CFG_THROTTLE_MODE);
				seq_printf(s, "%2d  ", state);
				state = REG_GET(r, OC1_CFG_ALARM_POLARITY);
				seq_printf(s, "%2d  ", state);
				state = REG_GET(r, OC1_CFG_EN_THROTTLE);
				seq_printf(s, "%2d  ", state);

				r = soctherm_readl(ts, ALARM_CNT_THRESHOLD(i));
				seq_printf(s, "%8d  ", r);
				r = soctherm_readl(ts, ALARM_THROTTLE_PERIOD(i));
				seq_printf(s, "%8d  ", r);
				r = soctherm_readl(ts, ALARM_ALARM_COUNT(i));
				seq_printf(s, "%8d  ", r);
				r = soctherm_readl(ts, ALARM_FILTER(i));
				seq_printf(s, "%8d  ", r);
				r = soctherm_readl(ts, ALARM_STATS(i));
				seq_printf(s, "%8d  ", r);
			}
			seq_puts(s, "\n");
		}
	}
	return 0;
}

static int temp_log_show(struct seq_file *s, void *data)
{
	struct platform_device *pdev = s->private;
	struct tegra_soctherm *tegra = platform_get_drvdata(pdev);
	struct tegra_tsensor *tsensors = tegra->tsensors;
	int i;
	u32 r, state;
	u64 ts;
	u_long ns;

	ts = cpu_clock(0);
	ns = do_div(ts, 1000000000);
	seq_printf(s, "%6lu.%06lu", (u_long) ts, ns / 1000);

	for (i = 0; tsensors[i].name; i++) {
		r = soctherm_readl(tegra,
				TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG1, i));
		state = REG_GET(r, TS_CPU0_CONFIG1_EN);
		if (!state)
			continue;

		r = soctherm_readl(tegra,
				TS_TSENSE_REG_OFFSET(TS_CPU0_STATUS1, i));
		if (!REG_GET(r, TS_CPU0_STATUS1_TEMP_VALID)) {
			seq_puts(s, "\tINVALID");
			continue;
		}

		state = REG_GET(r, TS_CPU0_STATUS1_TEMP);
		seq_printf(s, "\t%ld", translate_temp(state));
	}
	seq_puts(s, "\n");

	return 0;
}

static int regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, regs_show, inode->i_private);
}

static const struct file_operations regs_fops = {
	.open		= regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int temp_log_open(struct inode *inode, struct file *file)
{
	return single_open(file, temp_log_show, inode->i_private);
}
static const struct file_operations temp_log_fops = {
	.open		= temp_log_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int soctherm_debug_init(struct platform_device *pdev)
{
	struct dentry *tegra_soctherm_root;

	tegra_soctherm_root = debugfs_create_dir("tegra_soctherm", NULL);
	debugfs_create_file("regs", 0644, tegra_soctherm_root,
			    pdev, &regs_fops);
	debugfs_create_file("temp_log", 0644, tegra_soctherm_root,
			    pdev, &temp_log_fops);
	return 0;
}
#else
static inline int soctherm_debug_init(struct platform_device *pdev)
{ return 0; }
#endif

int tegra_soctherm_probe(struct platform_device *pdev,
		struct tegra_tsensor *tsensors,
		struct tegra_tsensor_group **tegra_tsensor_groups,
		u8 nominal_calib_cp, u8 nominal_calib_ft,
		bool is_ccroc)
{
	struct tegra_soctherm *tegra;
	struct thermal_zone_device *tz;
	struct tegra_tsensor_group *ttg;
	struct tsensor_shared_calibration *shared_calib;
	struct resource *reg_res;
	int i, irq_num;
	int err = 0;

	tegra = devm_kzalloc(&pdev->dev, sizeof(*tegra), GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, tegra);
	tegra->pdev = pdev;
	tegra->sensor_groups = tegra_tsensor_groups;
	tegra->tsensors = tsensors;
	tegra->is_ccroc = is_ccroc;

	reg_res = platform_get_resource_byname(pdev,
						IORESOURCE_MEM,
						"soctherm-reg");
	tegra->regs = devm_ioremap_resource(&pdev->dev, reg_res);
	if (IS_ERR(tegra->regs)) {
		dev_err(&pdev->dev, "can't get registers");
		return PTR_ERR(tegra->regs);
	}

	reg_res = platform_get_resource_byname(pdev,
						IORESOURCE_MEM,
						"car-reg");
	tegra->clk_regs = devm_ioremap_resource(&pdev->dev, reg_res);
	if (IS_ERR(tegra->clk_regs)) {
		dev_err(&pdev->dev, "can't get clk registers");
		return PTR_ERR(tegra->clk_regs);
	}

	if (is_ccroc) {
		reg_res = platform_get_resource_byname(pdev,
							IORESOURCE_MEM,
							"ccroc-reg");
		tegra->ccroc_regs = devm_ioremap_resource(&pdev->dev, reg_res);
		if (IS_ERR(tegra->ccroc_regs)) {
			dev_err(&pdev->dev, "can't get ccroc registers");
			return PTR_ERR(tegra->ccroc_regs);
		}
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

	tegra->thermal_irq = platform_get_irq(pdev, 0);
	if (tegra->thermal_irq <= 0) {
		dev_err(&pdev->dev, "can't get interrupt\n");
		return -EINVAL;
	}

	/* calculate shared calibration data */
	shared_calib = devm_kzalloc(&pdev->dev,
				    sizeof(*shared_calib), GFP_KERNEL);
	if (!shared_calib)
		return -ENOMEM;
	tegra->shared_calib = shared_calib;
	err = tegra_soctherm_calculate_shared_calibration(shared_calib,
							  nominal_calib_cp,
							  nominal_calib_ft);
	if (err)
		goto disable_clocks;

	/* calculate tsensor calibaration data */
	for (i = 0; tsensors[i].name; ++i)
		err = tegra_soctherm_calculate_tsensor_calibration(tsensors + i,
							   shared_calib);
	if (err)
		goto disable_clocks;

	err = soctherm_init_platform_data(pdev);
	if (err) {
		dev_err(&pdev->dev, "Initialize platform data failed\n");
		goto disable_clocks;
	}

	/* Initialize thermctl sensors */
	for (i = 0; tegra_tsensor_groups[i]; ++i) {
		struct tegra_thermctl_zone *zone =
			devm_kzalloc(&pdev->dev, sizeof(*zone), GFP_KERNEL);
		if (!zone) {
			err = -ENOMEM;
			goto unregister_tzs;
		}

		zone->sensor_group = tegra_tsensor_groups[i];
		zone->tegra = tegra;

		ttg = tegra_tsensor_groups[i];
		if (!(ttg->flags & SKIP_THERMAL_FW_REGISTRATION)) {
			tz = thermal_zone_of_sensor_register(
						&pdev->dev, ttg->id,
						zone,
						tegra_thermctl_get_temp,
						NULL,
						tegra_thermctl_set_trips);
			if (IS_ERR(tz)) {
				err = PTR_ERR(tz);
				dev_err(&pdev->dev, "failed to register sensor: %d\n",
					err);
				--i;
				goto unregister_tzs;
			}

			zone->tz = tz;
			tegra->thermctl_tzs[i] = tz;
		}
	}

	irq_num = platform_get_irq(pdev, 0);
	if (irq_num < 0) {
		dev_err(&pdev->dev, "get 'thermal irq' failed.\n");
		goto unregister_tzs;
	}
	err = devm_request_threaded_irq(&pdev->dev,
					irq_num,
					soctherm_thermal_isr,
					soctherm_thermal_isr_thread,
					IRQF_ONESHOT,
					"soctherm_thermal",
					tegra);
	if (err < 0) {
		dev_err(&pdev->dev, "request_irq 'thermal_irq' failed.\n");
		goto unregister_tzs;
	}

	err = soctherm_oc_int_init(pdev, TEGRA_SOC_OC_IRQ_NUM);
	if (err < 0) {
		dev_err(&pdev->dev,
			"soctherm_oc_int_init failed\n");
		goto unregister_tzs;
	}

	irq_num = platform_get_irq(pdev, 1);
	if (irq_num < 0) {
		dev_err(&pdev->dev, "get 'edp irq' failed.\n");
		goto unregister_tzs;
	}
	err = devm_request_threaded_irq(&pdev->dev,
					irq_num,
					soctherm_edp_isr,
					soctherm_edp_isr_thread,
					IRQF_ONESHOT,
					"soctherm_edp",
					tegra);
	if (err < 0) {
		dev_err(&pdev->dev, "request_irq 'edp_irq' failed.\n");
		goto unregister_tzs;
	}

	soctherm_debug_init(pdev);

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

int tegra_soctherm_remove(struct platform_device *pdev)
{
	struct tegra_soctherm *tegra = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ARRAY_SIZE(tegra->thermctl_tzs); ++i)
		thermal_zone_of_sensor_unregister(&pdev->dev,
						  tegra->thermctl_tzs[i]);

	clk_disable_unprepare(tegra->clock_tsensor);
	clk_disable_unprepare(tegra->clock_soctherm);

	return 0;
}

MODULE_AUTHOR("Mikko Perttunen <mperttunen@nvidia.com>");
MODULE_DESCRIPTION("Tegra SOCTHERM thermal management driver");
MODULE_LICENSE("GPL v2");
