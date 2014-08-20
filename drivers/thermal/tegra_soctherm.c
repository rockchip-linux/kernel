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
#include <linux/string.h>
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

#define CTL_LVL0_CPU0			0x0
#define CTL_LVL0_CPU0_UP_THRESH_SHIFT	17
#define CTL_LVL0_CPU0_UP_THRESH_MASK	0xff
#define CTL_LVL0_CPU0_DN_THRESH_SHIFT	9
#define CTL_LVL0_CPU0_DN_THRESH_MASK	0xff
#define CTL_LVL0_CPU0_EN_SHIFT		8
#define CTL_LVL0_CPU0_EN_MASK		0x1
#define CTL_LVL0_CPU0_DEV_THROT_LIGHT	0x1
#define CTL_LVL0_CPU0_DEV_THROT_HEAVY	0x2
#define CTL_LVL0_CPU0_CPU_THROT_SHIFT	5
#define CTL_LVL0_CPU0_CPU_THROT_MASK	0x3
#define CTL_LVL0_CPU0_GPU_THROT_SHIFT	3
#define CTL_LVL0_CPU0_GPU_THROT_MASK	0x3
#define CTL_LVL0_CPU0_STATUS_SHIFT	0
#define CTL_LVL0_CPU0_STATUS_MASK	0x3

#define THERMCTL_INTR_STATUS			0x84
#define THERMCTL_INTR_EN			0x88

#define STATS_CTL		0x94
#define STATS_CTL_CLR_DN	0x8
#define STATS_CTL_EN_DN		0x4
#define STATS_CTL_CLR_UP	0x2
#define STATS_CTL_EN_UP		0x1

#define SENSOR_PDIV				0x1c0
#define SENSOR_HOTSPOT_OFF			0x1c4

#define FUSE_TSENSOR8_CALIB			0x180
#define FUSE_SPARE_REALIGNMENT_REG_0		0x1fc

#define THROT_GLOBAL_CFG	0x400
#define THROT_GLOBAL_ENB_SHIFT	0
#define THROT_GLOBAL_ENB_MASK	0x1

#define THROT_PRIORITY_LOCK			0x424
#define THROT_PRIORITY_LOCK_PRIORITY_SHIFT	0
#define THROT_PRIORITY_LOCK_PRIORITY_MASK	0xff

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

#define CCROC_GLOBAL_CFG	0x148

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

#define CCROC_THROT_PSKIP_CTRL_CPU_REG(vect)	(CCROC_THROT_PSKIP_CTRL_CPU + \
						(CCROC_THROT_OFFSET * vect))
#define CCROC_THROT_PSKIP_RAMP_CPU_REG(vect)	(CCROC_THROT_PSKIP_RAMP_CPU + \
						(CCROC_THROT_OFFSET * vect))

#define THROT_PSKIP_CTRL(throt, dev)		(THROT_PSKIP_CTRL_LITE_CPU + \
						(THROT_OFFSET * throt) + \
						(8 * dev))
#define THROT_PSKIP_RAMP(throt, dev)		(THROT_PSKIP_RAMP_LITE_CPU + \
						(THROT_OFFSET * throt) + \
						(8 * dev))

#define THROT_PRIORITY_CTRL(throt)		(THROT_PRIORITY_LITE + \
						(THROT_OFFSET * throt))
#define THROT_DELAY_CTRL(throt)			(THROT_DELAY_LITE + \
						(THROT_OFFSET * throt))

#define REG_SET(r, _name, val)	(((r) & ~(_name##_MASK << _name##_SHIFT)) | \
				(((val) & _name##_MASK) << _name##_SHIFT))

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

static const int min_low_temp = -127000;
static const int max_high_temp = 127000;

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

	r &= ~THERMTRIP_ANY_EN_MASK;

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

	r &= ~THERMTRIP_ANY_EN_MASK;

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
	int temp;
	u32 r, reg_off;

	temp = enforce_temp_range(dev, trip_temp) / 1000;

	/* Hardcode LITE on level-1 and HEAVY on level-2 */
	reg_off = TS_THERM_REG_OFFSET(CTL_LVL0_CPU0, throt + 1, sg->id);

	r = soctherm_readl(ts, reg_off);
	r = REG_SET(r, CTL_LVL0_CPU0_UP_THRESH, temp);
	r = REG_SET(r, CTL_LVL0_CPU0_DN_THRESH, temp);
	r = REG_SET(r, CTL_LVL0_CPU0_EN, 1);
	r = REG_SET(r, CTL_LVL0_CPU0_CPU_THROT,
		    throt == THROTTLE_HEAVY ?
		    CTL_LVL0_CPU0_DEV_THROT_HEAVY :
		    CTL_LVL0_CPU0_DEV_THROT_LIGHT);
	r = REG_SET(r, CTL_LVL0_CPU0_GPU_THROT,
		    throt == THROTTLE_HEAVY ?
		    CTL_LVL0_CPU0_DEV_THROT_HEAVY :
		    CTL_LVL0_CPU0_DEV_THROT_LIGHT);

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
				      enum soctherm_throttle_id throt)
{
	u32 r;
	u8 priority;

	/* Setup PSKIP parameters */
	if (ts->is_ccroc)
		throttlectl_cpu_level_select(ts, throt);
	else
		throttlectl_cpu_mn(ts, throt);

	priority = 0xE + throt;
	r = REG_SET(0, THROT_PRIORITY_LITE_PRIO, priority);
	soctherm_writel(ts, r, THROT_PRIORITY_CTRL(throt));

	r = REG_SET(0, THROT_DELAY_LITE_DELAY, 0);
	soctherm_writel(ts, r, THROT_DELAY_CTRL(throt));

	r = soctherm_readl(ts, THROT_PRIORITY_LOCK);
	if (r < priority) {
		r = REG_SET(0, THROT_PRIORITY_LOCK_PRIORITY, priority);
		soctherm_writel(ts, r, THROT_PRIORITY_LOCK);
	}
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
	soctherm_throttle_program(ts, THROTTLE_HEAVY);

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

		v = soctherm_readl(tegra, SENSOR_PDIV);
		v &= ~ttg->pdiv_mask;
		v |= ttg->pdiv << (ffs(ttg->pdiv_mask) - 1);
		soctherm_writel(tegra, v, SENSOR_PDIV);

		v = soctherm_readl(tegra, SENSOR_HOTSPOT_OFF);
		v &= ~ttg->pllx_hotspot_mask;
		v |= (ttg->pllx_hotspot_diff / 1000) <<
			(ffs(ttg->pllx_hotspot_mask) - 1);
		soctherm_writel(tegra, v, SENSOR_HOTSPOT_OFF);

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
	int i;
	int err = 0;

	tegra = devm_kzalloc(&pdev->dev, sizeof(*tegra), GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, tegra);
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
