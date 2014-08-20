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

#define THERMCTL_INTR_STATUS			0x84
#define THERMCTL_INTR_EN			0x88

#define SENSOR_PDIV				0x1c0
#define SENSOR_HOTSPOT_OFF			0x1c4

#define FUSE_TSENSOR8_CALIB			0x180
#define FUSE_SPARE_REALIGNMENT_REG_0		0x1fc

static const int min_low_temp = -127000;
static const int max_high_temp = 127000;

struct tegra_soctherm {
	struct platform_device *pdev;
	struct reset_control *reset;
	struct clk *clock_tsensor;
	struct clk *clock_soctherm;
	void __iomem *regs;

	struct thermal_zone_device *thermctl_tzs[4];
	struct tegra_tsensor_group **sensor_groups;
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

static int enable_tsensor(struct tegra_soctherm *tegra,
			  struct tegra_tsensor *sensor,
			  struct tsensor_shared_calibration shared)
{
	unsigned int val;
	u32 calib;
	int err;

	err = tegra_soctherm_calculate_tsensor_calibration(sensor,
							   sensor->group,
							   shared, &calib);
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

		r = of_property_read_u32(sgn, "temperature", &temperature);
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

	ttn = of_find_node_by_name(dev->of_node, "thermtrip");
	if (!ttn) {
		dev_info(dev, "thermtrip: no DT node - not enabling\n");
		return 0;
	}

	r = thermtrip_configure_limits_from_dt(dev, ttn);
	if (r)
		return r;

	return 0;
}


int tegra_soctherm_probe(struct platform_device *pdev,
		struct tegra_tsensor *tsensors,
		struct tegra_tsensor_group **tegra_tsensor_groups,
		u8 nominal_calib_cp, u8 nominal_calib_ft)
{
	struct tegra_soctherm *tegra;
	struct thermal_zone_device *tz;
	struct tegra_tsensor_group *ttg;
	struct tsensor_shared_calibration shared_calib;
	int i;
	int err = 0;
	int irq;
	u32 v;

	tegra = devm_kzalloc(&pdev->dev, sizeof(*tegra), GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, tegra);
	tegra->sensor_groups = tegra_tsensor_groups;

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

	err = tegra_soctherm_calculate_shared_calibration(&shared_calib,
							  nominal_calib_cp,
							  nominal_calib_ft);
	if (err)
		goto disable_clocks;

	for (i = 0; tsensors[i].name; ++i) {
		err = enable_tsensor(tegra, tsensors + i, shared_calib);
		if (err)
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

		zone->sensor_group = tegra_tsensor_groups[i];
		zone->tegra = tegra;

		if (!(ttg->flags & SKIP_THERMAL_FW_REGISTRATION)) {

			tz = thermal_zone_of_sensor_register(
						&pdev->dev, i,
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

			soctherm_writel(
				tegra,
				0x3 << zone->sensor_group->thermctl_isr_shift,
				THERMCTL_INTR_EN);
		}
	}

	/* Set up hardware thermal limits */
	thermtrip_configure_from_dt(&pdev->dev);

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
