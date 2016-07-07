/*
 * Copyright (C) ST-Ericsson 2010 - 2013
 * Author: Martin Persson <martin.persson@stericsson.com>
 *         Hongbo Zhang <hongbo.zhang@linaro.org>
 * License Terms: GNU General Public License v2
 *
 * When the AB8500 thermal warning temperature is reached (threshold cannot
 * be changed by SW), an interrupt is set, and if no further action is taken
 * within a certain time frame, pm_power off will be called.
 *
 * When AB8500 thermal shutdown temperature is reached a hardware shutdown of
 * the AB8500 will occur.
 */

 #include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/ioport.h>

#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <linux/timer.h>
#include <linux/completion.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/rockchip/common.h>
#include "hwmon-rockchip.h"


#define DEFAULT_POWER_OFF_DELAY	(HZ * 10)
/* Number of monitored sensors should not greater than NUM_SENSORS */
#define NUM_MONITORED_SENSORS	4

#define TSADC_USER_CON  0x00
#define TSADC_AUTO_CON  0x04

#define TSADC_CTRL_CH(ch)	((ch) << 0)
#define TSADC_CTRL_POWER_UP	(1 << 3)
#define TSADC_CTRL_START	(1 << 4)

#define TSADC_STAS_BUSY		(1 << 12)
#define TSADC_STAS_BUSY_MASK	(1 << 12)
#define TSADC_AUTO_STAS_BUSY		(1 << 16)
#define TSADC_AUTO_STAS_BUSY_MASK	(1 << 16)
#define TSADC_SAMPLE_DLY_SEL  (1 << 17)
#define TSADC_SAMPLE_DLY_SEL_MASK  (1 << 17)

#define TSADC_INT_EN  0x08
#define TSADC_INT_PD  0x0c

#define TSADC_DATA0  0x20
#define TSADC_DATA1  0x24
#define TSADC_DATA2  0x28
#define TSADC_DATA3  0x2c
#define TSADC_DATA_MASK		0xfff

#define TSADC_COMP0_INT  0x30
#define TSADC_COMP1_INT  0x34
#define TSADC_COMP2_INT  0x38
#define TSADC_COMP3_INT  0x3c

#define TSADC_COMP0_SHUT  0x40
#define TSADC_COMP1_SHUT  0x44
#define TSADC_COMP2_SHUT  0x48
#define TSADC_COMP3_SHUT  0x4c

#define TSADC_HIGHT_INT_DEBOUNCE  0x60
#define TSADC_HIGHT_TSHUT_DEBOUNCE  0x64
#define TSADC_HIGHT_INT_DEBOUNCE_TIME 0x0a
#define TSADC_HIGHT_TSHUT_DEBOUNCE_TIME 0x0a

#define TSADC_AUTO_PERIOD  0x68
#define TSADC_AUTO_PERIOD_HT  0x6c
#define TSADC_AUTO_PERIOD_TIME	0x03e8
#define TSADC_AUTO_PERIOD_HT_TIME  0x64

#define TSADC_COMP0_LOW_INT		0x80
#define TSADC_SRC_LT_EN(ch)		(1 << (12 + ch))
#define TSADC_LT_INTEN_SRC(ch)		(1 << (12 + ch))
#define TSADC_AUTO_MODE_EN		(1 << 0)
#define TSADC_Q_SEL_EN			(1 << 1)

#define TSADC_AUTO_EVENT_NAME		"tsadc"

#define TSADC_COMP_INT_DATA		80
#define TSADC_COMP_INT_DATA_MASK		0xfff
#define TSADC_COMP_SHUT_DATA_MASK		0xfff
#define TSADC_TEMP_INT_EN 0
#define TSADC_TEMP_SHUT_EN 1
static int tsadc_ht_temp;
static int tsadc_low_temp;
static int tsadc_ht_reset_cru;
static int tsadc_ht_pull_gpio;

struct tsadc_port {
	struct pinctrl		*pctl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_tsadc_int;
};

struct rockchip_tsadc_temp {
	struct delayed_work power_off_work;
	struct rockchip_temp *rockchip_data;
	void __iomem		*regs;
	struct clk		*clk;
	struct clk		*pclk;
	int irq;
	struct resource		*ioarea;
	struct tsadc_host		*tsadc;
	struct work_struct 	auto_ht_irq_work;
	struct workqueue_struct  *workqueue;
	struct workqueue_struct  *tsadc_workqueue;
};
struct tsadc_table
{
	int code;
	int temp;
};

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

#define TSADCV3_DATA_MASK			0x3ff

static struct chip_tsadc_table *this_table;

static const struct tsadc_table rk1108_table[] = {
	{0, -40},

	{374, -40},
	{382, -35},
	{389, -30},
	{397, -25},
	{405, -20},
	{413, -15},
	{421, -10},
	{429, -5},
	{436, 0},
	{444, 5},
	{452, 10},
	{460, 15},
	{468, 20},
	{476, 25},
	{483, 30},
	{491, 35},
	{499, 40},
	{507, 45},
	{515, 50},
	{523, 55},
	{531, 60},
	{539, 65},
	{547, 70},
	{555, 75},
	{562, 80},
	{570, 85},
	{578, 90},
	{586, 95},
	{594, 100},
	{602, 105},
	{610, 110},
	{618, 115},
	{626, 120},
	{634, 125},

	{TSADC_DATA_MASK, 125},
};

static const struct tsadc_table rk322x_table[] =
{
	{0, -40},

	{588, -40},
	{593, -35},
	{598, -30},
	{603, -25},
	{608, -20},
	{613, -15},
	{618, -10},
	{623, -5},
	{629, 0},
	{634, 5},
	{639, 10},
	{644, 15},
	{649, 20},
	{654, 25},
	{660, 30},
	{665, 35},
	{670, 40},
	{675, 45},
	{681, 50},
	{686, 55},
	{691, 60},
	{696, 65},
	{702, 70},
	{707, 75},
	{712, 80},
	{717, 85},
	{723, 90},
	{728, 95},
	{733, 100},
	{738, 105},
	{744, 110},
	{749, 115},
	{754, 120},
	{760, 125},

	{TSADC_DATA_MASK, 125},
};

static const struct tsadc_table table[] = {
	{TSADC_DATA_MASK, -40},

	{3800, -40},
	{3792, -35},
	{3783, -30},
	{3774, -25},
	{3765, -20},
	{3756, -15},
	{3747, -10},
	{3737, -5},
	{3728, 0},
	{3718, 5},

	{3708, 10},
	{3698, 15},
	{3688, 20},
	{3678, 25},
	{3667, 30},
	{3656, 35},
	{3645, 40},
	{3634, 45},
	{3623, 50},
	{3611, 55},

	{3600, 60},
	{3588, 65},
	{3575, 70},
	{3563, 75},
	{3550, 80},
	{3537, 85},
	{3524, 90},
	{3510, 95},
	{3496, 100},
	{3482, 105},

	{3467, 110},
	{3452, 115},
	{3437, 120},
	{3421, 125},

	{0, 125},
};

static struct rockchip_tsadc_temp *g_dev;
static struct rockchip_temp *g_data;

static DEFINE_MUTEX(tsadc_mutex);

static u32 tsadc_readl(u32 offset)
{
	return readl_relaxed(g_dev->regs + offset);
}

static void tsadc_writel(u32 val, u32 offset)
{
	writel_relaxed(val, g_dev->regs + offset);
}

void rockchip_tsadc_auto_ht_work(struct work_struct *work)
{
	int ret, val;

	if (!g_data)
		return;

	mutex_lock(&tsadc_mutex);

	val = tsadc_readl(TSADC_INT_PD);
	tsadc_writel(val &(~ (1 <<8) ), TSADC_INT_PD);
	ret = tsadc_readl(TSADC_INT_PD);
	tsadc_writel(ret | 0xff, TSADC_INT_PD);       //clr irq status
	if (g_data->tsadc_type == RK322X_TSADC ||
	    g_data->tsadc_type == RK1108_TSADC) {
		if ((val & 0x1000) != 0) {
			dev_info(&g_data->pdev->dev, "rk322x tsadc is low temp\n");
		} else if ((val & 0x1) != 0) {
			dev_info(&g_data->pdev->dev, "rk322x tsadc is over temp\n");
			pm_power_off();
		}
	} else if ((g_data->tsadc_type == RK3288_TSADC) &&
		   ((val & 0x0f) != 0)) {
		printk("rockchip tsadc is over temp . %s,line=%d\n", __func__,__LINE__);
		pm_power_off();					//power_off
	}
	mutex_unlock(&tsadc_mutex);
}

static irqreturn_t rockchip_tsadc_auto_ht_interrupt(int irq, void *data)
{
	struct rockchip_tsadc_temp *dev = data;

	printk("%s,line=%d\n", __func__,__LINE__);

	queue_work(dev->workqueue, &dev->auto_ht_irq_work);

	return IRQ_HANDLED;
}

static int rockchip_temp_to_code(int temp, u32 *code)
{
	unsigned int low = 1;
	unsigned int high = this_table->length - 1;
	unsigned int mid = (low + high) / 2;
	unsigned int num;
	unsigned long denom;
	*code = TSADC_DATA_MASK;

	WARN_ON(this_table->length < 2);

	temp &= this_table->data_mask;
	if (temp < this_table->id[low].temp)
		return -EAGAIN;	/* Incorrect reading */

	while (low <= high) {
		if (temp == this_table->id[mid].temp) {
			*code = this_table->id[mid].code;
			break;
		} else if (temp > this_table->id[mid].temp) {
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
	if (*code == TSADC_DATA_MASK) {
		num = abs(this_table->id[low].code - this_table->id[high].code);
		num *= abs(this_table->id[high].temp - temp);
		denom =
		    abs(this_table->id[high].temp - this_table->id[low].temp);
		*code = this_table->id[high].code + (num / denom);
	}

	return 0;
}

static int rockchip_code_to_temp(u32 code, int *temp)
{
	unsigned int low = 1;
	unsigned int high = this_table->length - 1;
	unsigned int mid = (low + high) / 2;
	unsigned int num;
	unsigned long denom;
	*temp = INVALID_TEMP;

	WARN_ON(this_table->length < 2);

	switch (this_table->mode) {
	case ADC_DECREMENT:
		code &= this_table->data_mask;
		if (code < this_table->id[high].code)
			return -EAGAIN;	/* Incorrect reading */

		while (low <= high) {
			if (code == this_table->id[mid].code) {
				*temp = this_table->id[mid].temp;
				break;
			} else if (code < this_table->id[mid].code) {
				low = mid + 1;
			} else {
				high = mid - 1;
			}

			mid = (low + high) / 2;
		}
		break;
	case ADC_INCREMENT:
		code &= this_table->data_mask;
		if (code < this_table->id[low].code)
			return -EAGAIN;	/* Incorrect reading */

		while (low <= high) {
			if (code == this_table->id[mid].code) {
				*temp = this_table->id[mid].temp;
				break;
			} else if (code > this_table->id[mid].code) {
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
		num = abs(this_table->id[low].temp - this_table->id[high].temp);
		num *= abs(this_table->id[high].code - code);
		denom =
		    abs(this_table->id[high].code - this_table->id[low].code);
		*temp = this_table->id[high].temp + (num / denom);
	}

	return 0;
}

static void rockchip_v1_tsadc_set_cmpn_int_code(int chn)
{
	u32 code = 0;

	rockchip_temp_to_code(tsadc_ht_temp - 10, &code);
	tsadc_writel((code & TSADC_COMP_INT_DATA_MASK),
		     (TSADC_COMP0_INT + chn * 4));

	rockchip_temp_to_code(tsadc_ht_temp, &code);
	tsadc_writel((code & TSADC_COMP_INT_DATA_MASK),
		     (TSADC_COMP0_SHUT + chn * 4));

	rockchip_temp_to_code(tsadc_low_temp, &code);
	tsadc_writel((code & TSADC_COMP_INT_DATA_MASK),
		     (TSADC_COMP0_LOW_INT + chn * 4));
}

static void rockchip_tsadc_set_cmpn_int_vale( int chn, int temp)
{
	u32 code = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(table) - 1; i++) {
		if (temp <= table[i].temp && temp > table[i -1].temp) {
			code = table[i].code;
		}
	}
	tsadc_writel((code & TSADC_COMP_INT_DATA_MASK), (TSADC_COMP0_INT + chn*4));

}

static void rockchip_tsadc_set_cmpn_shut_vale( int chn, int temp)
{
	u32 code=0;
	int i;

	for (i = 0; i < ARRAY_SIZE(table) - 1; i++) {
		if (temp <= table[i].temp && temp > table[i -1].temp) {
			code = table[i].code;
		}
	}

	tsadc_writel((code & TSADC_COMP_SHUT_DATA_MASK), (TSADC_COMP0_SHUT + chn*4));
}

static void rockchip_tsadc_set_auto_int_en( int chn, int ht_int_en,int tshut_en)
{
	u32 ret;
	tsadc_writel(0, TSADC_INT_EN);
	if (ht_int_en){
		ret = tsadc_readl(TSADC_INT_EN);
		tsadc_writel( ret | (1 << chn), TSADC_INT_EN);
	}
	if (tshut_en){
		ret = tsadc_readl(TSADC_INT_EN);
		if (tsadc_ht_pull_gpio)
			tsadc_writel(ret | (0xf << (chn + 4)), TSADC_INT_EN);
		else if (tsadc_ht_reset_cru)
			tsadc_writel(ret | (0xf << (chn + 8)), TSADC_INT_EN);
	}

}
static void rockchip_tsadc_auto_mode_set(int chn, int int_temp,
	int shut_temp, int int_en, int shut_en)
{
	u32 ret;

	if (!g_dev || chn > 4)
		return;

	mutex_lock(&tsadc_mutex);

	clk_enable(g_dev->pclk);
	clk_enable(g_dev->clk);

	msleep(10);
	tsadc_writel(0, TSADC_AUTO_CON);
	tsadc_writel(3 << (4+chn), TSADC_AUTO_CON);
	msleep(10);
	if ((tsadc_readl(TSADC_AUTO_CON) & TSADC_AUTO_STAS_BUSY_MASK) != TSADC_AUTO_STAS_BUSY) {
		rockchip_tsadc_set_cmpn_int_vale(chn,int_temp);
		rockchip_tsadc_set_cmpn_shut_vale(chn,shut_temp),

		tsadc_writel(TSADC_AUTO_PERIOD_TIME, TSADC_AUTO_PERIOD);
		tsadc_writel(TSADC_AUTO_PERIOD_HT_TIME, TSADC_AUTO_PERIOD_HT);

		tsadc_writel(TSADC_HIGHT_INT_DEBOUNCE_TIME,
			TSADC_HIGHT_INT_DEBOUNCE);
		tsadc_writel(TSADC_HIGHT_TSHUT_DEBOUNCE_TIME,
			TSADC_HIGHT_TSHUT_DEBOUNCE);

		rockchip_tsadc_set_auto_int_en(chn, int_en, shut_en);
	}

	msleep(10);

	ret = tsadc_readl(TSADC_AUTO_CON);
	tsadc_writel(ret | (1 <<0) , TSADC_AUTO_CON);

	mutex_unlock(&tsadc_mutex);

}

int rockchip_tsadc_set_auto_temp(int chn)
{
	rockchip_tsadc_auto_mode_set(chn, TSADC_COMP_INT_DATA,
		tsadc_ht_temp, TSADC_TEMP_INT_EN, TSADC_TEMP_SHUT_EN);
	return 0;
}
EXPORT_SYMBOL(rockchip_tsadc_set_auto_temp);

int rockchip_rk3288_tsadc_get_temp(int chn, int voltage)
{
	int i, val = 0, reg = 0;

	if (!g_dev || chn > 4){
		val = INVALID_TEMP;
		return val;
	}

	reg = tsadc_readl((TSADC_DATA0 + chn*4)) & TSADC_DATA_MASK;
	for (i = 0; i < ARRAY_SIZE(table) - 1; i++) {
		if ((reg) <= table[i].code && (reg) > table[i + 1].code)
			val = table[i].temp + (table[i + 1].temp
			- table[i].temp) * (table[i].code - (reg))
			/ (table[i].code - table[i + 1].code);
	}

	return val;
}
EXPORT_SYMBOL(rockchip_rk3288_tsadc_get_temp);

static void rockchip_rk322x_tsadc_set_cmpn_int_vale(int chn)
{
	u32 code = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(rk322x_table) - 1; i++) {
		if ((tsadc_ht_temp - 10) >= rk322x_table[i].temp &&
		    (tsadc_ht_temp - 10) < rk322x_table[i + 1].temp)
			code = rk322x_table[i].code;
	}
	tsadc_writel((code & TSADC_COMP_INT_DATA_MASK),
		     (TSADC_COMP0_INT + chn * 4));

	for (i = 0; i < ARRAY_SIZE(rk322x_table) - 1; i++) {
		if (tsadc_ht_temp >= rk322x_table[i].temp &&
		    tsadc_ht_temp <  rk322x_table[i + 1].temp)
			code = rk322x_table[i].code;
	}
	tsadc_writel((code & TSADC_COMP_INT_DATA_MASK),
		     (TSADC_COMP0_SHUT + chn * 4));

	for (i = 0; i < ARRAY_SIZE(rk322x_table) - 1; i++) {
		if (tsadc_low_temp >= rk322x_table[i].temp &&
		    tsadc_low_temp < rk322x_table[i + 1].temp)
			code = rk322x_table[i].code;
	}
	tsadc_writel((code & TSADC_COMP_INT_DATA_MASK),
		     (TSADC_COMP0_LOW_INT + chn * 4));
}

static void rockchip_rk322x_tsadc_set_auto_int_en(int chn,
						  int ht_int_en,
						  int tshut_en)
{
	u32 ret;

	tsadc_writel(0, TSADC_INT_EN);

	if (ht_int_en) {
		ret = tsadc_readl(TSADC_INT_EN);
		tsadc_writel(ret | (1 << chn), TSADC_INT_EN);
	}
	if (tshut_en) {
		ret = tsadc_readl(TSADC_INT_EN);
		if (tsadc_ht_pull_gpio)
			tsadc_writel(ret | (1 << (chn + 4)), TSADC_INT_EN);
		else if (tsadc_ht_reset_cru)
			tsadc_writel(ret | (1 << (chn + 8)), TSADC_INT_EN);
	}
	if (tsadc_low_temp > -40)
		tsadc_writel(tsadc_readl(TSADC_INT_EN)
			| TSADC_LT_INTEN_SRC(chn), TSADC_INT_EN);
}

struct chip_tsadc_table rk1108_tsadc_data = {
	.id = rk1108_table,
	.length = ARRAY_SIZE(rk1108_table),
	.data_mask = TSADCV3_DATA_MASK,
	.mode = ADC_INCREMENT,
};

static void rockchip_v1_tsadc_auto_mode_set(int chn, int int_en, int shut_en)
{
	u32 ret;

	if (!g_dev || chn > 4)
		return;

	mutex_lock(&tsadc_mutex);

	clk_enable(g_dev->pclk);
	clk_enable(g_dev->clk);

	usleep_range(9, 10);
	tsadc_writel(0, TSADC_AUTO_CON);
	tsadc_writel(1 << (4 + chn), TSADC_AUTO_CON);
	usleep_range(9, 10);
	if ((tsadc_readl(TSADC_AUTO_CON) & TSADC_AUTO_STAS_BUSY_MASK) !=
	    TSADC_AUTO_STAS_BUSY) {
		rockchip_v1_tsadc_set_cmpn_int_code(chn);

		tsadc_writel(TSADC_AUTO_PERIOD_TIME, TSADC_AUTO_PERIOD);
		tsadc_writel(TSADC_AUTO_PERIOD_HT_TIME, TSADC_AUTO_PERIOD_HT);

		tsadc_writel(TSADC_HIGHT_INT_DEBOUNCE_TIME,
			     TSADC_HIGHT_INT_DEBOUNCE);
		tsadc_writel(TSADC_HIGHT_TSHUT_DEBOUNCE_TIME,
			     TSADC_HIGHT_TSHUT_DEBOUNCE);

		rockchip_rk322x_tsadc_set_auto_int_en(chn, int_en, shut_en);
	}

	usleep_range(9, 10);

	ret = tsadc_readl(TSADC_AUTO_CON);
	tsadc_writel(ret | TSADC_AUTO_MODE_EN | TSADC_SRC_LT_EN(chn) |
		     TSADC_Q_SEL_EN, TSADC_AUTO_CON);

	mutex_unlock(&tsadc_mutex);
}

static void rockchip_rk322x_tsadc_auto_mode_set(int chn,
						int int_en,
						int shut_en)
{
	u32 ret;

	if (!g_dev || chn > 4)
		return;

	mutex_lock(&tsadc_mutex);

	clk_enable(g_dev->pclk);
	clk_enable(g_dev->clk);

	msleep(10);
	tsadc_writel(0, TSADC_AUTO_CON);
	tsadc_writel(1 << (4 + chn), TSADC_AUTO_CON);
	msleep(10);
	if ((tsadc_readl(TSADC_AUTO_CON) & TSADC_AUTO_STAS_BUSY_MASK) !=
	    TSADC_AUTO_STAS_BUSY) {
		rockchip_rk322x_tsadc_set_cmpn_int_vale(chn);

		tsadc_writel(TSADC_AUTO_PERIOD_TIME,
			     TSADC_AUTO_PERIOD);
		tsadc_writel(TSADC_AUTO_PERIOD_HT_TIME,
			     TSADC_AUTO_PERIOD_HT);

		tsadc_writel(TSADC_HIGHT_INT_DEBOUNCE_TIME,
			     TSADC_HIGHT_INT_DEBOUNCE);
		tsadc_writel(TSADC_HIGHT_TSHUT_DEBOUNCE_TIME,
			     TSADC_HIGHT_TSHUT_DEBOUNCE);

		rockchip_rk322x_tsadc_set_auto_int_en(chn, int_en, shut_en);
	}

	msleep(10);

	ret = tsadc_readl(TSADC_AUTO_CON);
	tsadc_writel(ret | TSADC_AUTO_MODE_EN | TSADC_SRC_LT_EN(chn) |
		     TSADC_Q_SEL_EN, TSADC_AUTO_CON);

	mutex_unlock(&tsadc_mutex);
}

static int rockchip_v1_tsadc_set_auto_temp(int chn)
{
	rockchip_v1_tsadc_auto_mode_set(chn, TSADC_TEMP_INT_EN,
					TSADC_TEMP_SHUT_EN);
	return 0;
}

int rockchip_rk322x_tsadc_set_auto_temp(int chn)
{
	rockchip_rk322x_tsadc_auto_mode_set(chn,
					    TSADC_TEMP_INT_EN,
					    TSADC_TEMP_SHUT_EN);
	return 0;
}
EXPORT_SYMBOL(rockchip_rk322x_tsadc_set_auto_temp);

static int rockchip_v1_tsadc_get_temp(int chn, int voltage)
{
	int temp = INVALID_TEMP, code = 0;

	if (!g_dev || chn > 4)
		return temp;

	code = tsadc_readl((TSADC_DATA0 + chn * 4)) & TSADC_DATA_MASK;
	rockchip_code_to_temp(code, &temp);
	return temp;
}

int rockchip_rk322x_tsadc_get_temp(int chn, int voltage)
{
	int i, val = 0, reg = 0;

	if (!g_dev || chn > 4) {
		val = INVALID_TEMP;
		return val;
	}

	reg = tsadc_readl((TSADC_DATA0 + chn * 4)) & TSADC_DATA_MASK;
	for (i = 0; i < ARRAY_SIZE(rk322x_table) - 1; i++) {
		if ((reg) <= rk322x_table[i + 1].code &&
		    (reg) > rk322x_table[i].code)
			val = rk322x_table[i].temp + (rk322x_table[i + 1].temp
			- rk322x_table[i].temp) * ((reg) - rk322x_table[i].code)
			/ (rk322x_table[i + 1].code - rk322x_table[i].code);
	}

	return val;
}
EXPORT_SYMBOL(rockchip_rk322x_tsadc_get_temp);

int rockchip_tsadc_get_temp(int chn, int voltage)
{
	int temp;

	if (!g_data || chn > 4) {
		temp = INVALID_TEMP;
		return temp;
	}
	temp = g_data->ops.read_sensor(chn, voltage);

	return temp;
}
EXPORT_SYMBOL(rockchip_tsadc_get_temp);

static ssize_t rockchip_show_name(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "rockchip-tsadc\n");
}

static ssize_t rockchip_show_label(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	char *label;
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	int index = attr->index;

	switch (index) {
	case 0:
		label = "tsadc0";
		break;
	case 1:
		label = "tsadc1";
		break;
	case 2:
		label = "tsadc2";
		break;
	case 3:
		label = "tsadc3";
		break;
	default:
		return -EINVAL;
	}

	return sprintf(buf, "%s\n", label);
}

int rockchip_hwmon_init(struct rockchip_temp *data)
{
	struct rockchip_tsadc_temp *rockchip_tsadc_data;
	struct resource *res;
	struct device_node *np = data->pdev->dev.of_node;
	int ret,irq;
	u32 rate;
	struct tsadc_port *uap;

	rockchip_tsadc_data = devm_kzalloc(&data->pdev->dev, sizeof(*rockchip_tsadc_data),
		GFP_KERNEL);
	if (!rockchip_tsadc_data)
		return -ENOMEM;

	res = platform_get_resource(data->pdev, IORESOURCE_MEM, 0);
	rockchip_tsadc_data->regs = devm_request_and_ioremap(&data->pdev->dev, res);
	if (!rockchip_tsadc_data->regs) {
		dev_err(&data->pdev->dev, "cannot map IO\n");
		return -ENXIO;
	}

	irq = platform_get_irq(data->pdev, 0);
	if (irq < 0) {
		dev_err(&data->pdev->dev, "no irq resource?\n");
		return -EPERM;
	}
	rockchip_tsadc_data->irq = irq;
	ret = request_threaded_irq(rockchip_tsadc_data->irq, NULL, rockchip_tsadc_auto_ht_interrupt, IRQF_ONESHOT, TSADC_AUTO_EVENT_NAME, rockchip_tsadc_data);
	if (ret < 0) {
		dev_err(&data->pdev->dev, "failed to attach tsadc irq\n");
		return -EPERM;
	}

	rockchip_tsadc_data->workqueue = create_singlethread_workqueue("rockchip_tsadc");
	INIT_WORK(&rockchip_tsadc_data->auto_ht_irq_work, rockchip_tsadc_auto_ht_work);

	rockchip_tsadc_data->clk = devm_clk_get(&data->pdev->dev, "tsadc");
	if (IS_ERR(rockchip_tsadc_data->clk)) {
	    dev_err(&data->pdev->dev, "failed to get tsadc clock\n");
	    ret = PTR_ERR(rockchip_tsadc_data->clk);
	    return -EPERM;
	}

	if(of_property_read_u32(np, "clock-frequency", &rate)) {
          dev_err(&data->pdev->dev, "Missing clock-frequency property in the DT.\n");
	  return -EPERM;
	}

	ret = clk_set_rate(rockchip_tsadc_data->clk, rate);
	    if(ret < 0) {
	    dev_err(&data->pdev->dev, "failed to set adc clk\n");
	    return -EPERM;
	}
	clk_prepare_enable(rockchip_tsadc_data->clk);

	rockchip_tsadc_data->pclk = devm_clk_get(&data->pdev->dev, "pclk_tsadc");
	if (IS_ERR(rockchip_tsadc_data->pclk)) {
	    dev_err(&data->pdev->dev, "failed to get tsadc pclk\n");
	    ret = PTR_ERR(rockchip_tsadc_data->pclk);
	    return -EPERM;
	}
	clk_prepare_enable(rockchip_tsadc_data->pclk);

	platform_set_drvdata(data->pdev, rockchip_tsadc_data);
	g_dev = rockchip_tsadc_data;
	data->plat_data = rockchip_tsadc_data;

	if (of_property_read_u32(np, "tsadc-ht-temp",
		&tsadc_ht_temp)) {
		dev_err(&data->pdev->dev, "Missing  tsadc_ht_temp in the DT.\n");
		return -EPERM;
	}
	if (of_property_read_u32(np, "tsadc-low-temp", &tsadc_low_temp)) {
		dev_err(&data->pdev->dev, "Missing tsadc_low_temp in the DT.\n");
		tsadc_low_temp = -40;
	}
	if (of_property_read_u32(np, "tsadc-ht-reset-cru",
		&tsadc_ht_reset_cru)) {
		dev_err(&data->pdev->dev, "Missing tsadc_ht_reset_cru in the DT.\n");
		return -EPERM;
	}
	if (of_property_read_u32(np, "tsadc-ht-pull-gpio",
		&tsadc_ht_pull_gpio)) {
		dev_err(&data->pdev->dev, "Missing tsadc_ht_pull_gpio in the DT.\n");
		return -EPERM;
	}

	if (tsadc_ht_pull_gpio){
		/*bit8=1 gpio0_b2 = 1 shutdown else gpio0_b2 =1 shutdown*/
		/*
		ret = tsadc_readl(TSADC_AUTO_CON);
		tsadc_writel(ret | (1 << 8) , TSADC_AUTO_CON);
		*/
		uap = devm_kzalloc(&data->pdev->dev, sizeof(struct tsadc_port),
		GFP_KERNEL);
		if (uap == NULL)
			dev_err(&data->pdev->dev,
			"uap is not set %s,line=%d\n", __func__, __LINE__);
		uap->pctl = devm_pinctrl_get(&data->pdev->dev);
		uap->pins_default = pinctrl_lookup_state(uap->pctl, "default");
		uap->pins_tsadc_int = pinctrl_lookup_state(uap->pctl, "tsadc_int");
		pinctrl_select_state(uap->pctl, uap->pins_tsadc_int);
	}

	switch (data->tsadc_type) {
	case RK1108_TSADC:
		this_table = &rk1108_tsadc_data;
		rockchip_v1_tsadc_set_auto_temp(0);
		data->ops.read_sensor = rockchip_v1_tsadc_get_temp;
		break;
	case RK3288_TSADC:
		rockchip_tsadc_set_auto_temp(1);
		data->ops.read_sensor = rockchip_rk3288_tsadc_get_temp;
		break;
	case RK322X_TSADC:
		rockchip_rk322x_tsadc_set_auto_temp(0);
		data->ops.read_sensor = rockchip_rk322x_tsadc_get_temp;
		break;
	}

	data->monitored_sensors = NUM_MONITORED_SENSORS;
	data->ops.show_name = rockchip_show_name;
	data->ops.show_label = rockchip_show_label;
	data->ops.is_visible = NULL;
	g_data = data;

	dev_info(&data->pdev->dev, "initialized\n");
	return 0;
}
EXPORT_SYMBOL(rockchip_hwmon_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhangqing <zhangqing@rock-chips.com>");
MODULE_DESCRIPTION("Driver for TSADC");
