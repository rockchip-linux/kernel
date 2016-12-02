/*
 * Copyright (c) 2014, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/thermal.h>
#include <linux/timer.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/rockchip/common.h>
#include <linux/reboot.h>
#include <linux/scpi_protocol.h>
#include "../../arch/arm/mach-rockchip/efuse.h"

#if 0
#define thermal_dbg(dev, format, arg...)		\
	dev_printk(KERN_INFO , dev , format , ## arg)
#else
#define thermal_dbg(dev, format, arg...)
#endif


/**
 * If the temperature over a period of time High,
 * the resulting TSHUT gave CRU module,let it reset the entire chip,
 * or via GPIO give PMIC.
 */
enum tshut_mode {
	TSHUT_MODE_CRU = 0,
	TSHUT_MODE_GPIO,
};

enum tsadc_mode {
	TSADC_AUTO_MODE = 0,
	TSHUT_USER_MODE,
};

/**
 * the system Temperature Sensors tshut(tshut) polarity
 * the bit 8 is tshut polarity.
 * 0: low active, 1: high active
 */
enum tshut_polarity {
	TSHUT_LOW_ACTIVE = 0,
	TSHUT_HIGH_ACTIVE,
};

/**
 * The system has three Temperature Sensors.  channel 0 is reserved,
 * channel 1 is for CPU, and channel 2 is for GPU.
 */
 /*
enum sensor_id {
	SENSOR_CPU = 1,
	SENSOR_GPU,
};
*/
#define NUM_SENSORS	2

struct rockchip_tsadc_chip {
	long hw_shut_temp;
	enum tshut_mode tshut_mode;
	enum tshut_polarity tshut_polarity;
	enum tsadc_mode mode;
	int chn_id[NUM_SENSORS];
	int chn_num;
	const struct chip_tsadc_table *temp_table;
	const struct command_table *init_cmd_table;
	const struct command_table *enable_cmd_table;
	const struct command_table *disable_cmd_table;
	const struct command_table *ack_cmd_table;

	/* Chip-wide methods */
	void (*init)(void __iomem *reg,
		     const struct command_table *init_cmd_table,
		     enum tshut_polarity p);
	void (*irq_ack)(void __iomem *reg,
			const struct command_table *ack_cmd_table);
	void (*control)(void __iomem *reg,
			const struct command_table *ctrl_cmd_table);

	/* Per-sensor methods */
	int (*get_temp)(const struct chip_tsadc_table *table,
			int chn, void __iomem *reg, long *temp);
	void (*set_alarm_temp)(const struct chip_tsadc_table *table,
			       int chn, void __iomem *reg, long temp);
	void (*set_low_alarm_temp)(const struct chip_tsadc_table *table,
				   int chn, void __iomem *reg, long temp);
	void (*set_tshut_temp)(const struct chip_tsadc_table *table,
			       int chn, void __iomem *reg, long temp);
	void (*set_tshut_mode)(int chn, void __iomem *reg, enum tshut_mode m);
};

struct rockchip_thermal_sensor {
	struct rockchip_thermal_data *thermal;
	struct thermal_zone_device *tzd;
	int id;
};

struct rockchip_thermal_data {
	const struct rockchip_tsadc_chip *chip;
	struct kobject *rockchip_thermal_kobj;
	struct platform_device *pdev;
	struct reset_control *reset;

	struct rockchip_thermal_sensor sensors[NUM_SENSORS];

	struct clk *clk;
	struct clk *pclk;

	int cpu_temp_adjust;
	int gpu_temp_adjust;
	int cpu_temp;
	bool logout;
	bool b_suspend;
	struct mutex suspend_lock;
	int shuttemp_count;

	void __iomem *regs;

	long hw_shut_temp;
	enum tshut_mode tshut_mode;
	enum tshut_polarity tshut_polarity;
};

/* TSADC V2 Sensor info define: */
#define TSADCV2_USER_CON			0x00
#define TSADCV2_AUTO_CON			0x04
#define TSADCV2_INT_EN				0x08
#define TSADCV2_INT_PD				0x0c
#define TSADCV2_DATA(chn)			(0x20 + (chn) * 0x04)
#define TSADCV2_COMP_INT(chn)		        (0x30 + (chn) * 0x04)
#define TSADCV2_COMP_SHUT(chn)		        (0x40 + (chn) * 0x04)
#define TSADCV2_HIGHT_INT_DEBOUNCE		0x60
#define TSADCV2_HIGHT_TSHUT_DEBOUNCE		0x64
#define TSADCV2_AUTO_PERIOD			0x68
#define TSADCV2_AUTO_PERIOD_HT			0x6c
#define TSADCV4_LOW_COMP_INT			0x80

#define TSADCV2_AUTO_EN				BIT(0)
#define TSADCV2_AUTO_DISABLE			~BIT(0)
#define TSADCV2_AUTO_SRC_EN(chn)		BIT(4 + (chn))
#define TSADCV2_AUTO_TSHUT_POLARITY_HIGH	BIT(8)
#define TSADCV2_AUTO_TSHUT_POLARITY_LOW		~BIT(8)

/**
 * TSADCV3_AUTO_Q_SEL_EN:
 * whether select (4096 - tsadc_q) as output
 * 1'b0:use tsadc_q as output(temperature-code is rising sequence)
 * 1'b1:use(4096 - tsadc_q) as output (temperature-code is falling sequence)
 */
#define TSADCV3_AUTO_Q_SEL_EN			BIT(1)

#define TSADCV2_INT_SRC_EN(chn)			BIT(chn)
#define TSADCV2_SHUT_2GPIO_SRC_EN(chn)		BIT(4 + (chn))
#define TSADCV2_SHUT_2CRU_SRC_EN(chn)		BIT(8 + (chn))

#define TSADCV2_INT_PD_CLEAR			~BIT(8)
#define TSADCV4_INT_PD_CLEAR			~BIT(16)

#define TSADCV4_LOW_TEMP_INT_SRC_EN		BIT(12)

#define TSADCV2_DATA_MASK			0xfff
#define TSADCV3_DATA_MASK			0x3ff

#define TSADCV2_HIGHT_INT_DEBOUNCE_COUNT	4
#define TSADCV2_HIGHT_TSHUT_DEBOUNCE_COUNT	4

#define TSADCV2_AUTO_PERIOD_TIME		250 /* msec */
#define TSADCV2_AUTO_PERIOD_HT_TIME		50  /* msec */
#define TSADCV3_AUTO_PERIOD_TIME		1500 /* msec */
#define TSADCV3_AUTO_PERIOD_HT_TIME		1000 /* msec */

#define TSADC_TEST
#define TSADC_TEST_SAMPLE_TIME			200/* msec */

#define TSADC_MAX_HW_SHUT_TEMP_COUNT            3

/**
 * The conversion table has the adc value and temperature.
 * ADC_DECREMENT: the adc value is of diminishing.(e.g. rk3288_code_table)
 * ADC_INCREMENT: the adc value is incremental.(e.g. rk3368_code_table)
 */
enum adc_sort_mode {
	ADC_DECREMENT = 0,
	ADC_INCREMENT,
};

/**
 * struct chip_tsadc_table - hold information about chip-specific differences
 * @id: conversion table
 * @length: size of conversion table
 * @data_mask: mask to apply on data inputs
 * @mode: sort mode of this adc variant (incrementing or decrementing)
 */
struct chip_tsadc_table {
	const struct tsadc_table *id;
	unsigned int length;
	u32 data_mask;
	enum adc_sort_mode mode;
};

struct tsadc_table {
	unsigned long code;
	long temp;
};
static struct rockchip_thermal_data *s_thermal = NULL;
static long soc_is_rk3368;

#define TSADC_DATA_MASK		0xfff

static const struct tsadc_table v1_code_table[] = {
	{0, -40000},
	{296, -40000},
	{304, -35000},
	{313, -30000},
	{331, -20000},
	{340, -15000},
	{349, -10000},
	{359, -5000},
	{368, 0},
	{378, 5000},
	{388, 10000},
	{398, 15000},
	{408, 20000},
	{418, 25000},
	{429, 30000},
	{440, 35000},
	{451, 40000},
	{462, 45000},
	{473, 50000},
	{485, 55000},
	{496, 60000},
	{508, 65000},
	{521, 70000},
	{533, 75000},
	{546, 80000},
	{559, 85000},
	{572, 90000},
	{586, 95000},
	{600, 100000},
	{614, 105000},
	{629, 110000},
	{644, 115000},
	{659, 120000},
	{675, 125000},

	{TSADC_DATA_MASK, 125000},
};

static const struct chip_tsadc_table v1_tsadc = {
	.id = v1_code_table,
	.length = ARRAY_SIZE(v1_code_table),
	.data_mask = 0xfff,
	.mode = ADC_INCREMENT,
};

static const struct tsadc_table v2_code_table[] = {
	{TSADCV2_DATA_MASK, -40000},
	{3800, -40000},
	{3792, -35000},
	{3783, -30000},
	{3774, -25000},
	{3765, -20000},
	{3756, -15000},
	{3747, -10000},
	{3737, -5000},
	{3728, 0},
	{3718, 5000},
	{3708, 10000},
	{3698, 15000},
	{3688, 20000},
	{3678, 25000},
	{3667, 30000},
	{3656, 35000},
	{3645, 40000},
	{3634, 45000},
	{3623, 50000},
	{3611, 55000},
	{3600, 60000},
	{3588, 65000},
	{3575, 70000},
	{3563, 75000},
	{3550, 80000},
	{3537, 85000},
	{3524, 90000},
	{3510, 95000},
	{3496, 100000},
	{3482, 105000},
	{3467, 110000},
	{3452, 115000},
	{3437, 120000},
	{3421, 125000},
	{0, 125000},
};

static const struct chip_tsadc_table v2_tsadc = {
	.id = v2_code_table,
	.length = ARRAY_SIZE(v2_code_table),
	.data_mask = 0xfff,
	.mode = ADC_DECREMENT,
};

static const struct tsadc_table v3_code_table[] = {
	{0, -40000},
	{106, -40000},
	{108, -35000},
	{110, -30000},
	{112, -25000},
	{114, -20000},
	{116, -15000},
	{118, -10000},
	{120, -5000},
	{122, 0},
	{124, 5000},
	{126, 10000},
	{128, 15000},
	{130, 20000},
	{132, 25000},
	{134, 30000},
	{136, 35000},
	{138, 40000},
	{140, 45000},
	{142, 50000},
	{144, 55000},
	{146, 60000},
	{148, 65000},
	{150, 70000},
	{152, 75000},
	{154, 80000},
	{156, 85000},
	{158, 90000},
	{160, 95000},
	{162, 100000},
	{163, 105000},
	{165, 110000},
	{167, 115000},
	{169, 120000},
	{171, 125000},
	{TSADCV3_DATA_MASK, 125000},
};

static const struct chip_tsadc_table v3_tsadc = {
	.id = v3_code_table,
	.length = ARRAY_SIZE(v3_code_table),
	.data_mask = 0xfff,
	.mode = ADC_INCREMENT,
};

struct tsadc_command {
	u32 reg_value;
	u32 offset;
	u32 mask;
	u32 delay;
	u32 pre_read;
	u32 and;
};

struct command_table {
	const struct tsadc_command *commands;
	unsigned int length;
};

static const struct tsadc_command v2_init_commands[] = {
	{TSADCV2_AUTO_PERIOD_TIME, TSADCV2_AUTO_PERIOD, 0xffff, 0, 0, 0},
	{TSADCV2_HIGHT_INT_DEBOUNCE_COUNT, TSADCV2_HIGHT_INT_DEBOUNCE, 0xffff,
	 0, 0, 0},
	{TSADCV2_AUTO_PERIOD_HT_TIME, TSADCV2_AUTO_PERIOD_HT, 0xffff, 0, 0, 0},
	{TSADCV2_HIGHT_TSHUT_DEBOUNCE_COUNT, TSADCV2_HIGHT_TSHUT_DEBOUNCE,
	 0xffff, 0, 0, 0},
};

static const struct command_table v2_init_cmd_table = {
	.commands = v2_init_commands,
	.length = ARRAY_SIZE(v2_init_commands),
};

static void rk_config_common(void __iomem *regs,
			     const struct command_table *cmd_table)
{
	int i;
	u32 val = 0;
	const struct tsadc_command *commands = cmd_table->commands;

	for (i = 0; i < cmd_table->length; i++) {

		val = 0;
		if (commands[i].pre_read)
			val = readl_relaxed(regs + commands[i].offset);

		if (commands[i].and)
			val = val & commands[i].reg_value;
		else
			val = val | commands[i].reg_value;

		writel_relaxed(val & commands[i].mask,
			       regs + commands[i].offset);

		if (commands[i].delay > 0)
			udelay(commands[i].delay);
	}

}

static void rk_tsadc_init(void __iomem *regs,
			  const struct command_table *init_cmd_table,
			  enum tshut_polarity tshut_polarity)
{
	if (tshut_polarity == TSHUT_HIGH_ACTIVE)
		writel_relaxed(0 | (TSADCV2_AUTO_TSHUT_POLARITY_HIGH),
			       regs + TSADCV2_AUTO_CON);

	rk_config_common(regs, init_cmd_table);
}

static const struct tsadc_command v3_init_commands[] = {
	{TSADCV3_AUTO_PERIOD_TIME, TSADCV2_AUTO_PERIOD, 0xffff, 0, 0, 0},
	{TSADCV2_HIGHT_INT_DEBOUNCE_COUNT, TSADCV2_HIGHT_INT_DEBOUNCE, 0xffff,
	 0, 0, 0},
	{TSADCV3_AUTO_PERIOD_HT_TIME, TSADCV2_AUTO_PERIOD_HT, 0xffff, 0, 0, 0},
	{TSADCV3_AUTO_PERIOD_TIME, TSADCV2_AUTO_PERIOD, 0xffff, 0, 0, 0},
	{TSADCV2_HIGHT_TSHUT_DEBOUNCE_COUNT, TSADCV2_HIGHT_TSHUT_DEBOUNCE,
	 0xffff, 0, 0, 0},
};

static const struct command_table v3_init_cmd_table = {
	.commands = v3_init_commands,
	.length = ARRAY_SIZE(v3_init_commands),
};

static void rk_tsadc_irq_ack(void __iomem *regs,
			     const struct command_table *ack_cmd_table)
{
	rk_config_common(regs, ack_cmd_table);
}

static const struct tsadc_command v2_ack_commands[] = {
	{TSADCV2_INT_PD, TSADCV2_INT_PD, 0xffff, 0, 1, 1}
};

static const struct command_table v2_ack_cmd_table = {
	.commands = v2_ack_commands,
	.length = ARRAY_SIZE(v2_ack_commands),
};

static const struct tsadc_command v4_ack_commands[] = {
	{(u32)TSADCV4_INT_PD_CLEAR, TSADCV2_INT_PD, 0xffff, 0, 1, 1}
};

static const struct command_table v4_ack_cmd_table = {
	.commands = v4_ack_commands,
	.length = ARRAY_SIZE(v4_ack_commands),
};

static void rk_tsadc_control(void __iomem *regs,
			     const struct command_table *ctrl_cmd_table)
{
	rk_config_common(regs, ctrl_cmd_table);
}

static const struct tsadc_command v2_enable_commands[] = {
	{TSADCV2_AUTO_EN, TSADCV2_AUTO_CON, 0xffff, 0, 1, 0},
};

static const struct command_table v2_enable_cmd_table = {
	.commands = v2_enable_commands,
	.length = ARRAY_SIZE(v2_enable_commands),
};

static const struct tsadc_command v2_disable_commands[] = {
	{(u32)~TSADCV2_AUTO_EN, TSADCV2_AUTO_CON, 0xffff, 0, 1, 1},
};

static const struct command_table v2_disable_cmd_table = {
	.commands = v2_disable_commands,
	.length = ARRAY_SIZE(v2_disable_commands),
};

static const struct tsadc_command v3_enable_commands[] = {
	{TSADCV2_AUTO_EN | TSADCV3_AUTO_Q_SEL_EN, TSADCV2_AUTO_CON, 0xffff, 0,
	 1, 0},
};

static const struct command_table v3_enable_cmd_table = {
	.commands = v3_enable_commands,
	.length = ARRAY_SIZE(v3_enable_commands),
};

static struct rockchip_thermal_data *rockchip_thermal_get_data(void)
{
	BUG_ON(!s_thermal);
	return s_thermal;
}

static int rockchip_temp_to_code(const struct chip_tsadc_table *tmp_table,
				 long temp, u32 *code)
{
	unsigned int low = 1;
	unsigned int high = tmp_table->length - 1;
	unsigned int mid = (low + high) / 2;
	unsigned int num;
	unsigned long denom;
	*code = tmp_table->data_mask;

	WARN_ON(tmp_table->length < 2);

	if (temp < tmp_table->id[low].temp)
		return -EAGAIN;	/* Incorrect reading */

	while (low <= high) {
		if (temp == tmp_table->id[mid].temp) {
			*code = tmp_table->id[mid].code;
			break;
		} else if (temp > tmp_table->id[mid].temp) {
			low = mid + 1;
		} else {
			high = mid - 1;
		}

		mid = (low + high) / 2;
	}
	/*
	 * The 5C granularity provided by the table is too much. Let's
	 * assume that the relationship between sensor readings and
	 * temperature between 2 table entries is linear and interpolate
	 * to produce less granular result.
	 */
	if (*code == tmp_table->data_mask) {
		num = abs(tmp_table->id[low].code - tmp_table->id[high].code);
		num *= abs(tmp_table->id[high].temp - temp);
		denom = abs(tmp_table->id[high].temp - tmp_table->id[low].temp);
		*code = tmp_table->id[high].code + (num / denom);
	}

	return 0;
}

static int rockchip_code_to_temp(const struct chip_tsadc_table *tmp_table,
				 u32 code, long *temp)
{
	unsigned int low = 1;
	unsigned int high = tmp_table->length - 1;
	unsigned int mid = (low + high) / 2;
	unsigned int num;
	unsigned long denom;
	*temp = INVALID_TEMP;

	WARN_ON(tmp_table->length < 2);

	switch (tmp_table->mode) {
	case ADC_DECREMENT:
		code &= tmp_table->data_mask;
		if (code < tmp_table->id[high].code)
			return -EAGAIN;	/* Incorrect reading */

		while (low <= high) {
			if (code == tmp_table->id[mid].code) {
				*temp = tmp_table->id[mid].temp;
				break;
			} else if (code < tmp_table->id[mid].code) {
				low = mid + 1;
			} else {
				high = mid - 1;
			}

			mid = (low + high) / 2;
		}
		break;
	case ADC_INCREMENT:
		code &= tmp_table->data_mask;
		if (code < tmp_table->id[low].code)
			return -EAGAIN;	/* Incorrect reading */

		while (low <= high) {
			if (code == tmp_table->id[mid].code) {
				*temp = tmp_table->id[mid].temp;
				break;
			} else if (code > tmp_table->id[mid].code) {
				low = mid + 1;
			} else {
				high = mid - 1;
			}

			mid = (low + high) / 2;
		}
		break;
	default:
		pr_err("Invalid the conversion table\n");
	}

	/*
	 * The 5C granularity provided by the table is too much. Let's
	 * assume that the relationship between sensor readings and
	 * temperature between 2 table entries is linear and interpolate
	 * to produce less granular result.
	 */
	if (*temp == INVALID_TEMP) {
		num = abs(tmp_table->id[low].temp - tmp_table->id[high].temp);
		num *= abs(tmp_table->id[high].code - code);
		denom = abs(tmp_table->id[high].code - tmp_table->id[low].code);
		*temp = tmp_table->id[high].temp + (num / denom);
	}

	return 0;
}

static int rk_tsadc_get_temp(const struct chip_tsadc_table *tmp_table, int chn,
			     void __iomem *regs, long *temp)
{
	u32 val;

	val = readl_relaxed(regs + TSADCV2_DATA(chn));
	if (val == 0)
		return -EAGAIN;

	rockchip_code_to_temp(tmp_table, val, temp);
	return 0;
}

static void rk_tsadc_alarm_temp(const struct chip_tsadc_table *tmp_table,
				int chn, void __iomem *regs, long temp)
{
	u32 alarm_value, int_en;

	rockchip_temp_to_code(tmp_table, temp, &alarm_value);
	writel_relaxed(alarm_value & TSADCV2_DATA_MASK,
		       regs + TSADCV2_COMP_INT(chn));

	int_en = readl_relaxed(regs + TSADCV2_INT_EN);
	int_en |= TSADCV2_INT_SRC_EN(chn);
	writel_relaxed(int_en, regs + TSADCV2_INT_EN);
}

static void rk_tsadc_tshut_temp(const struct chip_tsadc_table *tmp_table,
				int chn, void __iomem *regs, long temp)
{
	u32 tshut_value, val;

	rockchip_temp_to_code(tmp_table, temp, &tshut_value);
	writel_relaxed(tshut_value, regs + TSADCV2_COMP_SHUT(chn));

	/* TSHUT will be valid */
	val = readl_relaxed(regs + TSADCV2_AUTO_CON);
	writel_relaxed(val | TSADCV2_AUTO_SRC_EN(chn), regs + TSADCV2_AUTO_CON);
}

static void rk_tsadc_low_alarm_temp(const struct chip_tsadc_table *tmp_table,
				    int chn, void __iomem *regs, long temp)
{
	u32 alarm_value, int_en;

	rockchip_temp_to_code(tmp_table, temp, &alarm_value);
	writel_relaxed(alarm_value & TSADCV2_DATA_MASK,
		       regs + TSADCV4_LOW_COMP_INT);

	int_en = readl_relaxed(regs + TSADCV2_INT_EN);
	int_en |= TSADCV4_LOW_TEMP_INT_SRC_EN;
	writel_relaxed(int_en, regs + TSADCV2_INT_EN);
}

static void rk_tsadc_tshut_mode(int chn, void __iomem *regs,
				enum tshut_mode mode)
{
	u32 val;

	val = readl_relaxed(regs + TSADCV2_INT_EN);
	if (mode == TSHUT_MODE_GPIO) {
		val &= ~TSADCV2_SHUT_2CRU_SRC_EN(chn);
		val |= TSADCV2_SHUT_2GPIO_SRC_EN(chn);
	} else {
		val &= ~TSADCV2_SHUT_2GPIO_SRC_EN(chn);
		val |= TSADCV2_SHUT_2CRU_SRC_EN(chn);
	}

	writel_relaxed(val, regs + TSADCV2_INT_EN);
}

static const struct rockchip_tsadc_chip rk3288_tsadc_data = {
	.tshut_mode = TSHUT_MODE_GPIO, /* default TSHUT via GPIO give PMIC */
	.tshut_polarity = TSHUT_LOW_ACTIVE, /* default TSHUT LOW ACTIVE */
	.hw_shut_temp = 125000,
	.mode = TSADC_AUTO_MODE,
	.chn_num = 2,
	.chn_id[0] = 1,
	.chn_id[1] = 2,
	.temp_table = &v2_tsadc,

	.init_cmd_table = &v2_init_cmd_table,
	.enable_cmd_table = &v2_enable_cmd_table,
	.disable_cmd_table = &v2_disable_cmd_table,
	.ack_cmd_table = &v2_ack_cmd_table,

	.init = rk_tsadc_init,
	.irq_ack = rk_tsadc_irq_ack,
	.control = rk_tsadc_control,
	.get_temp = rk_tsadc_get_temp,

	.set_alarm_temp = rk_tsadc_alarm_temp,
	.set_tshut_temp = rk_tsadc_tshut_temp,
	.set_tshut_mode = rk_tsadc_tshut_mode,
};

static const struct rockchip_tsadc_chip rk3368_tsadc_data = {
	.tshut_mode = TSHUT_MODE_GPIO, /* default TSHUT via GPIO give PMIC */
	.tshut_polarity = TSHUT_LOW_ACTIVE, /* default TSHUT LOW ACTIVE */
	.hw_shut_temp = 125000,
	.mode = TSHUT_USER_MODE,
	.chn_num = 2,
	.chn_id[0] = 0,
	.chn_id[1] = 1,
	.temp_table = &v3_tsadc,

	.init_cmd_table = &v3_init_cmd_table,
	.enable_cmd_table = &v2_enable_cmd_table,
	.disable_cmd_table = &v2_disable_cmd_table,
	.ack_cmd_table = &v2_ack_cmd_table,

	.init = rk_tsadc_init,
	.irq_ack = rk_tsadc_irq_ack,
	.control = rk_tsadc_control,
	.get_temp = rk_tsadc_get_temp,

	.set_alarm_temp = rk_tsadc_alarm_temp,
	.set_tshut_temp = rk_tsadc_tshut_temp,
	.set_tshut_mode = rk_tsadc_tshut_mode,
};

static const struct rockchip_tsadc_chip rk3228_tsadc_data = {
	.tshut_mode = TSHUT_MODE_GPIO, /* default TSHUT via GPIO give PMIC */
	.tshut_polarity = TSHUT_LOW_ACTIVE, /* default TSHUT LOW ACTIVE */
	.hw_shut_temp = 125000,
	.mode = TSADC_AUTO_MODE,
	.chn_num = 1,
	.chn_id[0] = 0,
	.temp_table = &v2_tsadc,

	.init_cmd_table = &v2_init_cmd_table,
	.enable_cmd_table = &v2_enable_cmd_table,
	.disable_cmd_table = &v2_disable_cmd_table,
	.ack_cmd_table = &v4_ack_cmd_table,

	.init = rk_tsadc_init,
	.irq_ack = rk_tsadc_irq_ack,
	.control = rk_tsadc_control,
	.get_temp = rk_tsadc_get_temp,

	.set_alarm_temp = rk_tsadc_alarm_temp,
	.set_low_alarm_temp = rk_tsadc_low_alarm_temp,
	.set_tshut_temp = rk_tsadc_tshut_temp,
	.set_tshut_mode = rk_tsadc_tshut_mode,
};

static const struct rockchip_tsadc_chip rk322xh_tsadc_data = {
	.tshut_mode = TSHUT_MODE_CRU,	/* default TSHUT via GPIO give PMIC */
	.tshut_polarity = TSHUT_LOW_ACTIVE,	/* default TSHUT LOW ACTIVE */
	.hw_shut_temp = 120000,
	.mode = TSADC_AUTO_MODE,
	.chn_num = 1,
	.chn_id[0] = 0,
	.temp_table = &v1_tsadc,

	.init_cmd_table = &v2_init_cmd_table,
	.enable_cmd_table = &v3_enable_cmd_table,
	.disable_cmd_table = &v2_disable_cmd_table,
	.ack_cmd_table = &v4_ack_cmd_table,

	.init = rk_tsadc_init,
	.irq_ack = rk_tsadc_irq_ack,
	.control = rk_tsadc_control,
	.get_temp = rk_tsadc_get_temp,
	.set_alarm_temp = rk_tsadc_alarm_temp,
	.set_tshut_temp = rk_tsadc_tshut_temp,
	.set_tshut_mode = rk_tsadc_tshut_mode,
};

static const struct of_device_id of_rockchip_thermal_match[] = {
	{
		.compatible = "rockchip,rk3288-tsadc",
		.data = (void *)&rk3288_tsadc_data,
	},
	{
		.compatible = "rockchip,rk3368-tsadc",
		.data = (void *)&rk3368_tsadc_data,
	},
	{
		.compatible = "rockchip,rk3228-tsadc",
		.data = (void *)&rk3228_tsadc_data,
	},
	{
		.compatible = "rockchip,rk322xh-tsadc",
		.data = (void *)&rk322xh_tsadc_data,
	},
	{ /* end */ },
};
MODULE_DEVICE_TABLE(of, of_rockchip_thermal_match);

static void rockchip_thermal_toggle_sensor(struct rockchip_thermal_sensor *sensor
	, bool on)
{
	struct thermal_zone_device *tzd = sensor->tzd;

	tzd->ops->set_mode(tzd,
		on ? THERMAL_DEVICE_ENABLED : THERMAL_DEVICE_DISABLED);
}

static irqreturn_t rockchip_thermal_alarm_irq_thread(int irq, void *dev)
{
	struct rockchip_thermal_data *thermal = dev;
	int i;

	dev_dbg(&thermal->pdev->dev, "thermal alarm\n");

	thermal->chip->irq_ack(thermal->regs, thermal->chip->ack_cmd_table);

	for (i = 0; i < thermal->chip->chn_num; i++)
		thermal_zone_device_update(thermal->sensors[i].tzd);

	return IRQ_HANDLED;
}

/*
static int rockchip_thermal_set_trips(void *_sensor, long low, long high)
{
	struct rockchip_thermal_sensor *sensor = _sensor;
	struct rockchip_thermal_data *thermal = sensor->thermal;
	const struct rockchip_tsadc_chip *tsadc = thermal->chip;

	dev_dbg(&thermal->pdev->dev, "%s: sensor %d: low: %ld, high %ld\n",
		__func__, sensor->id, low, high);

	tsadc->set_alarm_temp(sensor->id, thermal->regs, high);

	return 0;
}
*/

static int rockchip_thermal_get_temp(void *_sensor, long *out_temp)
{
	struct rockchip_thermal_sensor *sensor = _sensor;
	struct rockchip_thermal_data *thermal = sensor->thermal;
	const struct rockchip_tsadc_chip *tsadc = sensor->thermal->chip;
	int retval;

	retval = tsadc->get_temp(tsadc->temp_table, sensor->id, thermal->regs,
				 out_temp);
	dev_dbg(&thermal->pdev->dev, "sensor %d - temp: %ld, retval: %d\n",
		sensor->id, *out_temp, retval);

	return retval;
}

static int rockchip_configure_from_dt(struct device *dev,
				      struct device_node *np,
				      struct rockchip_thermal_data *thermal)
{
	u32 shut_temp, tshut_mode, tshut_polarity;
	u32 rate, cycle;

	if(of_property_read_u32(np, "clock-frequency", &rate)) {
		dev_err(dev, "Missing clock-frequency property in the DT.\n");
		return -EINVAL;
	}
	clk_set_rate(thermal->clk, rate);
	if (thermal->chip->mode == TSHUT_USER_MODE) {
		cycle = DIV_ROUND_UP(1000000000, rate) / 1000;
		if (scpi_thermal_set_clk_cycle(cycle)) {
			dev_err(dev, "scpi_thermal_set_clk_cycle error.\n");
			return -EINVAL;
		}
	}

	if (of_property_read_u32(np, "hw-shut-temp", &shut_temp)) {
		dev_warn(dev,
			 "Missing tshut temp property, using default %ld\n",
			 thermal->chip->hw_shut_temp);
		thermal->hw_shut_temp = thermal->chip->hw_shut_temp;
	} else {
		thermal->hw_shut_temp = shut_temp;
	}

	if (thermal->hw_shut_temp > INT_MAX) {
		dev_err(dev, "Invalid tshut temperature specified: %ld\n",
			thermal->hw_shut_temp);
		return -ERANGE;
	}

	if (of_property_read_u32(np, "tsadc-tshut-mode", &tshut_mode)) {
		dev_warn(dev,
			 "Missing tshut mode property, using default (%s)\n",
			 thermal->chip->tshut_mode == TSHUT_MODE_GPIO ?
				"gpio" : "cru");
		thermal->tshut_mode = thermal->chip->tshut_mode;
	} else {
		thermal->tshut_mode = tshut_mode;
	}

	if (thermal->tshut_mode > 1) {
		dev_err(dev, "Invalid tshut mode specified: %d\n",
			thermal->tshut_mode);
		return -EINVAL;
	}

	if (of_property_read_u32(np, "tsadc-tshut-polarity", &tshut_polarity)) {
		dev_warn(dev,
			 "Missing tshut-polarity property, using default (%s)\n",
			 thermal->chip->tshut_polarity == TSHUT_LOW_ACTIVE ?
				"low" : "high");
		thermal->tshut_polarity = thermal->chip->tshut_polarity;
	} else {
		thermal->tshut_polarity = tshut_polarity;
	}

	if (thermal->tshut_polarity > 1) {
		dev_err(dev, "Invalid tshut-polarity specified: %d\n",
			thermal->tshut_polarity);
		return -EINVAL;
	}

	/* init the cpu_temp to be invalid value */
	thermal->cpu_temp = INVALID_TEMP;

	return 0;
}

static int
rockchip_thermal_register_sensor(struct platform_device *pdev,
				 struct rockchip_thermal_data *thermal,
				 struct rockchip_thermal_sensor *sensor,
				 int id)
{
	const struct rockchip_tsadc_chip *tsadc = thermal->chip;
	int error;

	tsadc->set_tshut_mode(id, thermal->regs, thermal->tshut_mode);
	tsadc->set_tshut_temp(tsadc->temp_table, id, thermal->regs,
			      thermal->hw_shut_temp);

	sensor->thermal = thermal;
	sensor->id = id;
	sensor->tzd = thermal_zone_of_sensor_register(&pdev->dev, id, sensor,
						rockchip_thermal_get_temp,
						NULL);
	if (IS_ERR(sensor->tzd)) {
		error = PTR_ERR(sensor->tzd);
		dev_err(&pdev->dev, "failed to register sensor %d: %d\n",
			id, error);
		return error;
	}

	return 0;
}

/*
 * Reset TSADC Controller, reset all tsadc registers.
 */
static void rockchip_thermal_reset_controller(struct reset_control *reset)
{
	reset_control_assert(reset);
	usleep_range(10, 20);
	reset_control_deassert(reset);
}

static int rk3368_rockchip_tsadc_get_temp(int chn, int voltage)
{
	struct rockchip_thermal_data *thermal = rockchip_thermal_get_data();
	const struct rockchip_tsadc_chip *tsadc = thermal->chip;
	long out_temp;
	int temp;
	int tsadc_data, data_adjust;
	u32 code_temp;
	u32 tmp_code1;
	u32 tmp_code2;

	if (thermal->chip->mode == TSADC_AUTO_MODE) {
		thermal->chip->get_temp(thermal->chip->temp_table, chn,
					thermal->regs, &out_temp);
		temp = (int)out_temp / 1000;
	} else {
		tsadc_data = scpi_thermal_get_temperature();
		rockchip_temp_to_code(tsadc->temp_table,
				      thermal->cpu_temp_adjust * 1000,
				      &tmp_code1);
		rockchip_temp_to_code(tsadc->temp_table, 0, &tmp_code2);
		data_adjust = tmp_code1 - tmp_code2;
		code_temp =
		    ((tsadc_data * voltage - data_adjust * 1000000) +
		     500000) / 1000000;
		rockchip_code_to_temp(tsadc->temp_table, code_temp, &out_temp);
		temp = (int)out_temp / 1000;
		thermal->cpu_temp = temp;
		if (thermal->logout)
			printk("cpu code temp:[%d, %d], voltage: %d\n",
			       tsadc_data, temp, voltage);

		if (temp > thermal->hw_shut_temp / 1000) {
			thermal->shuttemp_count++;
			dev_err(&thermal->pdev->dev,
				"cpu code temp:[%d, %d], voltage: %d\n",
				 tsadc_data, temp, voltage);
		}
		else
			thermal->shuttemp_count = 0;
		if (thermal->shuttemp_count >= TSADC_MAX_HW_SHUT_TEMP_COUNT) {
			dev_err(&thermal->pdev->dev,
				"critical temperature reached(%ld C),shutting down\n",
				 thermal->hw_shut_temp / 1000);
			orderly_poweroff(true);
		}
	}

	return temp;
}

int rockchip_tsadc_get_temp(int chn, int voltage)
{
	struct rockchip_thermal_data *thermal = rockchip_thermal_get_data();
	int temp;

	mutex_lock(&thermal->suspend_lock);

	if (thermal->b_suspend) {
		temp = INVALID_TEMP;
		mutex_unlock(&thermal->suspend_lock);
		return temp;
	}

	if (soc_is_rk3368) {
		temp = rk3368_rockchip_tsadc_get_temp(chn, voltage);
	} else {
		thermal->chip->get_temp(thermal->chip->temp_table, chn,
					thermal->regs, (long *)&temp);
		/* Convert milli degree Celsius to degree Celsius. */
		temp /= 1000;
		/* back up the temp for easy access when panic dump */
		thermal->cpu_temp = temp;
	}
	mutex_unlock(&thermal->suspend_lock);

	return temp;
}
EXPORT_SYMBOL(rockchip_tsadc_get_temp);

static ssize_t rockchip_thermal_temp_adjust_test_store(struct kobject *kobj
	, struct kobj_attribute *attr, const char *buf, size_t n)
{
	struct rockchip_thermal_data *thermal = rockchip_thermal_get_data();
	int getdata;
	char cmd;
	const char *buftmp = buf;

	sscanf(buftmp, "%c ", &cmd);
	switch (cmd) {
	case 'c':
		sscanf(buftmp, "%c %d", &cmd, &getdata);
		thermal->cpu_temp_adjust = getdata;
		printk("get cpu_temp_adjust value = %d\n", getdata);

		break;
	case 'g':
		sscanf(buftmp, "%c %d", &cmd, &getdata);
		thermal->gpu_temp_adjust = getdata;
		printk("get gpu_temp_adjust value = %d\n", getdata);

		break;
	default:
		printk("Unknown command\n");
		break;
	}

	return n;
}

static ssize_t rockchip_thermal_temp_adjust_test_show(struct kobject *kobj
	, struct kobj_attribute *attr, char *buf)
{
	struct rockchip_thermal_data *thermal = rockchip_thermal_get_data();
	char *str = buf;

	str += sprintf(str, "rockchip_thermal: cpu:%d, gpu:%d\n"
		, thermal->cpu_temp_adjust, thermal->gpu_temp_adjust);
	return (str - buf);
}

static ssize_t rockchip_thermal_temp_test_store(struct kobject *kobj
	, struct kobj_attribute *attr, const char *buf, size_t n)
{
	struct rockchip_thermal_data *thermal = rockchip_thermal_get_data();
	char cmd;
	const char *buftmp = buf;

	sscanf(buftmp, "%c", &cmd);
	switch (cmd) {
	case 't':
		thermal->logout = true;
		break;
	case 'f':
		thermal->logout = false;
		break;
	default:
		printk("Unknown command\n");
		break;
	}

	return n;
}

static ssize_t rockchip_thermal_temp_test_show(struct kobject *kobj
	, struct kobj_attribute *attr, char *buf)
{
	struct rockchip_thermal_data *thermal = rockchip_thermal_get_data();
	char *str = buf;

	str += sprintf(str, "current cpu_temp:%d\n"
		, thermal->cpu_temp);
	return (str - buf);
}

struct rockchip_thermal_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf);
	ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n);
};

static struct rockchip_thermal_attribute rockchip_thermal_attrs[] = {
	/*node_name	permision show_func store_func*/
	__ATTR(temp_adjust, S_IRUGO | S_IWUSR, rockchip_thermal_temp_adjust_test_show
		, rockchip_thermal_temp_adjust_test_store),
	__ATTR(temp, S_IRUGO | S_IWUSR, rockchip_thermal_temp_test_show
		, rockchip_thermal_temp_test_store),
};

static void rockchip_dump_temperature(void)
{
	struct rockchip_thermal_data *thermal = rockchip_thermal_get_data();
	struct platform_device *pdev;

	if (!thermal)
		return;

	pdev = thermal->pdev;

	if (thermal->cpu_temp != INVALID_TEMP)
		dev_warn(&pdev->dev, "cpu channal temperature(%d C)\n",
			 thermal->cpu_temp);

	if (thermal->regs) {
		pr_warn("THERMAL REGS:\n");
		print_hex_dump(KERN_WARNING, "", DUMP_PREFIX_OFFSET,
			       32, 4, thermal->regs, 0x88, false);
	}
}

static int rockchip_thermal_panic(struct notifier_block *this,
				  unsigned long ev, void *ptr)
{
	rockchip_dump_temperature();
	return NOTIFY_DONE;
}

static struct notifier_block rockchip_thermal_panic_block = {
	.notifier_call = rockchip_thermal_panic,
};

static int rockchip_thermal_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct rockchip_thermal_data *thermal;
	const struct of_device_id *match;
	struct resource *res;
	int irq;
	int i, j;
	int error;

	match = of_match_node(of_rockchip_thermal_match, np);
	if (!match)
		return -ENXIO;

	if (match->data == (void *)&rk3368_tsadc_data)
		soc_is_rk3368 = 1;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -EINVAL;
	}

	thermal = devm_kzalloc(&pdev->dev, sizeof(struct rockchip_thermal_data),
			       GFP_KERNEL);
	if (!thermal)
		return -ENOMEM;

	thermal->pdev = pdev;

	thermal->chip = (const struct rockchip_tsadc_chip *)match->data;
	if (!thermal->chip)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	thermal->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(thermal->regs))
		return PTR_ERR(thermal->regs);

	thermal->reset = devm_reset_control_get(&pdev->dev, "tsadc-apb");
	if (IS_ERR(thermal->reset)) {
		error = PTR_ERR(thermal->reset);
		dev_err(&pdev->dev, "failed to get tsadc reset: %d\n", error);
		return error;
	}

	thermal->clk = devm_clk_get(&pdev->dev, "tsadc");
	if (IS_ERR(thermal->clk)) {
		error = PTR_ERR(thermal->clk);
		dev_err(&pdev->dev, "failed to get tsadc clock: %d\n", error);
		return error;
	}

	thermal->pclk = devm_clk_get(&pdev->dev, "apb_pclk");
	if (IS_ERR(thermal->pclk)) {
		error = PTR_ERR(thermal->pclk);
		dev_err(&pdev->dev, "failed to get apb_pclk clock: %d\n",
			error);
		return error;
	}

	error = clk_prepare_enable(thermal->clk);
	if (error) {
		dev_err(&pdev->dev, "failed to enable converter clock: %d\n"
			, error);
		return error;
	}

	error = clk_prepare_enable(thermal->pclk);
	if (error) {
		dev_err(&pdev->dev, "failed to enable pclk: %d\n", error);
		goto err_disable_clk;
	}

	rockchip_thermal_reset_controller(thermal->reset);

	error = rockchip_configure_from_dt(&pdev->dev, np, thermal);
	if (error) {
		dev_err(&pdev->dev, "failed to parse device tree data: %d\n",
			error);
		goto err_disable_pclk;
	}

	mutex_init(&thermal->suspend_lock);
	s_thermal = thermal;

	if (soc_is_rk3368)
		thermal->cpu_temp_adjust = rockchip_efuse_get_temp_adjust(0);

	if (thermal->chip->mode == TSADC_AUTO_MODE)
	{
		thermal->chip->init(thermal->regs,
				    thermal->chip->init_cmd_table,
				    thermal->tshut_polarity);
		for (i = 0; i < thermal->chip->chn_num; i++) {
			error = rockchip_thermal_register_sensor(pdev, thermal,
							 &thermal->sensors[i],
							 thermal->chip->chn_id[i]);
			if (error) {
				dev_err(&pdev->dev,
					"failed to register thermal sensor %d : error= %d\n", i, error);
				for (j = 0; j < i; j++)
					thermal_zone_of_sensor_unregister(&pdev->dev, thermal->sensors[j].tzd);
				goto err_disable_pclk;
			}
		}

		error = devm_request_threaded_irq(&pdev->dev, irq, NULL,
						  &rockchip_thermal_alarm_irq_thread,
						  IRQF_ONESHOT,
						  "rockchip_thermal", thermal);
		if (error) {
			dev_err(&pdev->dev,
				"failed to request tsadc irq: %d\n", error);
			goto err_unregister_sensor;
		}

		thermal->chip->control(thermal->regs,
				       thermal->chip->enable_cmd_table);

		for (i = 0; i < thermal->chip->chn_num; i++)
			rockchip_thermal_toggle_sensor(&thermal->sensors[i], true);
	}

	if (soc_is_rk3368) {
		thermal->rockchip_thermal_kobj =
		    kobject_create_and_add("rockchip_thermal", NULL);
		if (!thermal->rockchip_thermal_kobj)
			return -ENOMEM;
		for (i = 0; i < ARRAY_SIZE(rockchip_thermal_attrs); i++) {
			error =
			    sysfs_create_file(thermal->rockchip_thermal_kobj,
					      &rockchip_thermal_attrs[i].attr);
			if (error != 0) {
				printk("create index %d error\n", i);
				return error;
			}
		}
	}

	platform_set_drvdata(pdev, thermal);

	atomic_notifier_chain_register(&panic_notifier_list,
				       &rockchip_thermal_panic_block);

	return 0;

err_unregister_sensor:
	if (thermal->chip->mode == TSADC_AUTO_MODE) {
		for (i = 0; i < thermal->chip->chn_num; i++) {
			thermal_zone_of_sensor_unregister(&pdev->dev, thermal->sensors[i].tzd);
		}
	}
err_disable_pclk:
	clk_disable_unprepare(thermal->pclk);
err_disable_clk:
	clk_disable_unprepare(thermal->clk);

	return error;
}

static int rockchip_thermal_remove(struct platform_device *pdev)
{
	struct rockchip_thermal_data *thermal = platform_get_drvdata(pdev);
	int i;

	if (thermal->chip->mode == TSADC_AUTO_MODE)
	{
		for (i = 0; i < thermal->chip->chn_num; i++) {
			struct rockchip_thermal_sensor *sensor = &thermal->sensors[i];

			rockchip_thermal_toggle_sensor(sensor, false);
			thermal_zone_of_sensor_unregister(&pdev->dev, sensor->tzd);
		}

		thermal->chip->control(thermal->regs,
				       thermal->chip->disable_cmd_table);
	}
	clk_disable_unprepare(thermal->pclk);
	clk_disable_unprepare(thermal->clk);

	return 0;
}

static int __maybe_unused rockchip_thermal_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rockchip_thermal_data *thermal = platform_get_drvdata(pdev);
	int i;

	mutex_lock(&thermal->suspend_lock);
	thermal->b_suspend = true;
	if (thermal->chip->mode == TSADC_AUTO_MODE)
	{
		for (i = 0; i < thermal->chip->chn_num; i++)
			rockchip_thermal_toggle_sensor(&thermal->sensors[i], false);

		thermal->chip->control(thermal->regs,
				       thermal->chip->disable_cmd_table);
	}
	clk_disable(thermal->pclk);
	clk_disable(thermal->clk);
	mutex_unlock(&thermal->suspend_lock);

	return 0;
}

static int __maybe_unused rockchip_thermal_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rockchip_thermal_data *thermal = platform_get_drvdata(pdev);
	int i;
	int error;

	mutex_lock(&thermal->suspend_lock);
	error = clk_enable(thermal->clk);
	if (error) {
		mutex_unlock(&thermal->suspend_lock);
		return error;
	}

	error = clk_enable(thermal->pclk);
	if (error) {
		mutex_unlock(&thermal->suspend_lock);
		return error;
	}

	rockchip_thermal_reset_controller(thermal->reset);
	if (thermal->chip->mode == TSADC_AUTO_MODE)
	{
		thermal->chip->init(thermal->regs,
				    thermal->chip->init_cmd_table,
				    thermal->tshut_polarity);

		for (i = 0; i < thermal->chip->chn_num; i++) {
			int id = thermal->sensors[i].id;

			thermal->chip->set_tshut_mode(id, thermal->regs,
						      thermal->tshut_mode);
			thermal->chip->set_tshut_temp(thermal->chip->temp_table,
						      id, thermal->regs,
						      thermal->hw_shut_temp);
		}

		thermal->chip->control(thermal->regs,
				       thermal->chip->enable_cmd_table);

		for (i = 0; i < thermal->chip->chn_num; i++)
			rockchip_thermal_toggle_sensor(&thermal->sensors[i], true);
	}

	thermal->b_suspend = false;
	mutex_unlock(&thermal->suspend_lock);

	return 0;
}

static SIMPLE_DEV_PM_OPS(rockchip_thermal_pm_ops,
			 rockchip_thermal_suspend, rockchip_thermal_resume);

static struct platform_driver rockchip_thermal_driver = {
	.driver = {
		.name = "rockchip-thermal",
		.owner = THIS_MODULE,
		.pm = &rockchip_thermal_pm_ops,
		.of_match_table = of_rockchip_thermal_match,
	},
	.probe = rockchip_thermal_probe,
	.remove = rockchip_thermal_remove,
};

module_platform_driver(rockchip_thermal_driver);

MODULE_DESCRIPTION("ROCKCHIP THERMAL Driver");
MODULE_AUTHOR("Rockchip, Inc.");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:rockchip-thermal");
