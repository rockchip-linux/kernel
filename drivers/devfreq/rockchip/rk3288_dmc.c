/*
 * Copyright (C) 2014 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>

#include <soc/rockchip/dmc-sync.h>

#define RK3288_DMC_NUM_CH	2

#define DMC_UPTHRESHOLD		15
#define DMC_DOWNDIFFERENTIAL	10

#define DDR_PCTL_TOGCNT1U	0x00c0

#define GRF_SOC_CON4		0x0254
#define GRF_SOC_STATUS11	0x02ac
#define GRF_SOC_STATUS12	0x02b0
#define GRF_SOC_STATUS13	0x02b4
#define GRF_SOC_STATUS14	0x02b8

#define PMU_SYS_REG2		0x009c

#define NOC_DDRCONF		0x0008
#define NOC_DDRTIMING		0x000c

#define DDR_SEL_DDR3		0x20000000
#define DDR_SEL_MOBILEDDR	0x20002000

struct dmc_usage {
	u32 write;
	u32 read;
	u32 active;
	u32 time;
};

struct rk3288_dmcfreq {
	struct device *clk_dev;
	struct devfreq *devfreq;
	struct devfreq_simple_ondemand_data ondemand_data;
	struct clk *dmc_clk;
	struct regulator *vdd_logic;
	struct dmc_usage ch_usage[RK3288_DMC_NUM_CH];

	struct regmap *grf;
	struct regmap *pmu;
	struct regmap *noc;
	struct regmap *dmc;

	unsigned long rate;
	bool enabled;
};

/*
 * We can only have one since devfreq has to be added for the parent device (the
 * clk device) which has the operating points defined. We assume only one or
 * zero devices will be probed for this driver.
 */
static struct rk3288_dmcfreq dmcfreq;

static int rk3288_dmcfreq_target(struct device *dev, unsigned long *freq,
				 u32 flags)
{
	struct dev_pm_opp *opp;
	unsigned long rate, old_rate;
	unsigned long volt, old_volt;
	unsigned long old_clk_rate;
	int err = 0;

	rcu_read_lock();
	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		return PTR_ERR(opp);
	}
	rate = dev_pm_opp_get_freq(opp);
	volt = dev_pm_opp_get_voltage(opp);
	old_rate = dmcfreq.rate;
	opp = devfreq_recommended_opp(dev, &old_rate, flags);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		return PTR_ERR(opp);
	}
	old_volt = dev_pm_opp_get_voltage(opp);
	rcu_read_unlock();

	*freq = old_rate;

	if (old_rate == rate)
		return 0;

	if (old_rate < rate)
		err = regulator_set_voltage(dmcfreq.vdd_logic, volt, volt);
	if (err) {
		dev_err(dev, "Unable to set voltage %lu\n", volt);
		return err;
	}

	old_clk_rate = clk_get_rate(dmcfreq.dmc_clk);
	err = clk_set_rate(dmcfreq.dmc_clk, rate);
	if (err || old_clk_rate == clk_get_rate(dmcfreq.dmc_clk)) {
		dev_err(dev,
			"Unable to set freq %lu. Current freq %lu. Error %d\n",
			rate,
			old_clk_rate,
			err);
		goto rate;
	}

	if (old_rate > rate)
		err = regulator_set_voltage(dmcfreq.vdd_logic, volt, volt);
	if (err) {
		dev_err(dev, "Unable to set voltage %lu\n", volt);
		goto volt;
	}

	dmcfreq.rate = rate;
	*freq = rate;

	return 0;
volt:
	clk_set_rate(dmcfreq.dmc_clk, old_rate);
rate:
	regulator_set_voltage(dmcfreq.vdd_logic, old_volt, old_volt);

	return err;
}

static void rk3288_dmc_start_hardware_counter(void)
{
	int i;

	for (i = 1; i < 4; i++) {
		regmap_write(dmcfreq.noc, 0x400*i+NOC_DDRCONF, 0x08);
		regmap_write(dmcfreq.noc, 0x400*i+NOC_DDRTIMING, 0x01);
		/* TODO: What are these registers? */
		regmap_write(dmcfreq.noc, 0x400*i+0x138, 0x06);
		regmap_write(dmcfreq.noc, 0x400*i+0x14c, 0x10);
		regmap_write(dmcfreq.noc, 0x400*i+0x160, 0x08);
		regmap_write(dmcfreq.noc, 0x400*i+0x174, 0x10);
	}

	/* Enables hardware usage counters */
	regmap_write(dmcfreq.grf, GRF_SOC_CON4, 0xc000c000);
	/* TODO: What is this register? */
	for (i = 1; i < 4; i++)
		regmap_write(dmcfreq.noc, 0x400*i+0x28, 0x1);
}

static void rk3288_dmc_stop_hardware_counter(void)
{
	/* Disables hardware usage counters */
	regmap_write(dmcfreq.grf, GRF_SOC_CON4, 0xc0000000);
}

static int rk3288_dmc_get_busier_ch(void)
{
	u64 tmp, max = 0;
	int i, busier_ch;

	rk3288_dmc_stop_hardware_counter();
	/* Find out which channel is busier */
	for (i = 0; i < RK3288_DMC_NUM_CH; i++) {
		regmap_read(dmcfreq.grf, GRF_SOC_STATUS11+i*16,
			    &dmcfreq.ch_usage[i].write);
		regmap_read(dmcfreq.grf, GRF_SOC_STATUS12+i*16,
			    &dmcfreq.ch_usage[i].read);
		regmap_read(dmcfreq.grf, GRF_SOC_STATUS13+i*16,
			    &dmcfreq.ch_usage[i].active);
		regmap_read(dmcfreq.grf, GRF_SOC_STATUS14+i*16,
			    &dmcfreq.ch_usage[i].time);
		tmp = (u64)dmcfreq.ch_usage[i].write + dmcfreq.ch_usage[i].read;
		if (tmp > max) {
			busier_ch = i;
			max = tmp;
		}
	}
	rk3288_dmc_start_hardware_counter();

	return busier_ch;
}

static int rk3288_dmcfreq_get_dev_status(struct device *dev,
					 struct devfreq_dev_status *stat)
{
	int busier_ch;

	busier_ch = rk3288_dmc_get_busier_ch();
	stat->current_frequency = dmcfreq.rate;
	stat->busy_time = (dmcfreq.ch_usage[busier_ch].write +
		dmcfreq.ch_usage[busier_ch].read) * 4;
	stat->total_time = dmcfreq.ch_usage[busier_ch].time;

	return 0;
}

static void rk3288_dmcfreq_exit(struct device *dev)
{
	devfreq_unregister_opp_notifier(dmcfreq.clk_dev, dmcfreq.devfreq);
}

static unsigned int rk3288_dmc_rates[] = {
	200000000,
	266000000,
	333000000,
	400000000,
	533000000,
	666000000,
};

static struct devfreq_dev_profile rk3288_devfreq_dmc_profile = {
	.polling_ms	= 200,
	.target		= rk3288_dmcfreq_target,
	.get_dev_status	= rk3288_dmcfreq_get_dev_status,
	.exit		= rk3288_dmcfreq_exit,
	.freq_table	= rk3288_dmc_rates,
	.max_state	= ARRAY_SIZE(rk3288_dmc_rates),
};

/*
 * This puts the frequency at max and suspends devfreq when there are too many
 * things to sync with (DMC_DISABLE). It resumes devfreq when there are few
 * enough things to sync with (DMC_ENABLE).
 */
static int rk3288_dmc_sync_notify(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	unsigned long freq = ULONG_MAX;

	if (action == DMC_ENABLE) {
		dev_info(dmcfreq.clk_dev, "resuming DVFS\n");
		rk3288_dmc_start_hardware_counter();
		devfreq_resume_device(dmcfreq.devfreq);
		return NOTIFY_OK;
	} else if (action == DMC_DISABLE) {
		dev_info(dmcfreq.clk_dev, "suspending DVFS and going to max freq\n");
		devfreq_suspend_device(dmcfreq.devfreq);
		rk3288_dmc_stop_hardware_counter();
		rk3288_dmcfreq_target(dmcfreq.clk_dev, &freq, 0);
		return NOTIFY_OK;
	}

	return NOTIFY_BAD;
}

static struct notifier_block dmc_sync_nb = {
	.notifier_call = rk3288_dmc_sync_notify,
};

static __maybe_unused int rk3288_dmcfreq_suspend(struct device *dev)
{
	int ret = devfreq_suspend_device(dmcfreq.devfreq);

	rk3288_dmc_stop_hardware_counter();

	return ret;
}

static __maybe_unused int rk3288_dmcfreq_resume(struct device *dev)
{
	rk3288_dmc_start_hardware_counter();

	return devfreq_resume_device(dmcfreq.devfreq);
}

static SIMPLE_DEV_PM_OPS(rk3288_dmcfreq_pm, rk3288_dmcfreq_suspend,
			 rk3288_dmcfreq_resume);

static int rk3288_dmcfreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	u32 tmp;

	dmcfreq.clk_dev = dev->parent;
	dmcfreq.dmc = syscon_regmap_lookup_by_compatible("rockchip,rk3288-dmc");
	if (IS_ERR(dmcfreq.dmc))
		return PTR_ERR(dmcfreq.dmc);
	dmcfreq.grf = syscon_regmap_lookup_by_compatible("rockchip,rk3288-grf");
	if (IS_ERR(dmcfreq.grf))
		return PTR_ERR(dmcfreq.grf);
	dmcfreq.pmu = syscon_regmap_lookup_by_compatible("rockchip,rk3288-pmu");
	if (IS_ERR(dmcfreq.pmu))
		return PTR_ERR(dmcfreq.pmu);
	dmcfreq.noc = syscon_regmap_lookup_by_compatible("rockchip,rk3288-noc");
	if (IS_ERR(dmcfreq.noc))
		return PTR_ERR(dmcfreq.noc);

	dmcfreq.vdd_logic = regulator_get(dmcfreq.clk_dev, "logic");
	if (IS_ERR(dmcfreq.vdd_logic)) {
		dev_err(dev, "Cannot get the regulator \"vdd_logic\"\n");
		return PTR_ERR(dmcfreq.vdd_logic);
	}

	dmcfreq.dmc_clk = devm_clk_get(dev, "dmc_clk");
	if (IS_ERR(dmcfreq.dmc_clk)) {
		dev_err(dev, "Cannot get the clk dmc_clk\n");
		return PTR_ERR(dmcfreq.dmc_clk);
	};

	/*
	 * We add a devfreq driver to our parent since it has a device tree node
	 * with operating points.
	 */
	if (of_init_opp_table(dmcfreq.clk_dev)) {
		dev_err(dev, "Invalid operating-points in device tree.\n");
		return -EINVAL;
	}

	/* Check if we are using LPDDR */
	regmap_read(dmcfreq.pmu, PMU_SYS_REG2, &tmp);
	tmp = (tmp >> 13) & 7;
	regmap_write(dmcfreq.grf, GRF_SOC_CON4,
		     (tmp == 3) ? DDR_SEL_DDR3 : DDR_SEL_MOBILEDDR);
	rk3288_dmc_start_hardware_counter();

	regmap_read(dmcfreq.dmc, DDR_PCTL_TOGCNT1U, &tmp);
	dmcfreq.rate = tmp * 1000000;
	rk3288_devfreq_dmc_profile.initial_freq = dmcfreq.rate;

	dmcfreq.ondemand_data.upthreshold = DMC_UPTHRESHOLD;
	dmcfreq.ondemand_data.downdifferential = DMC_DOWNDIFFERENTIAL;
	dmcfreq.devfreq = devfreq_add_device(dmcfreq.clk_dev,
					     &rk3288_devfreq_dmc_profile,
					     "simple_ondemand",
					     &dmcfreq.ondemand_data);
	if (IS_ERR(dmcfreq.devfreq))
		return PTR_ERR(dmcfreq.devfreq);

	devfreq_register_opp_notifier(dmcfreq.clk_dev, dmcfreq.devfreq);
	rockchip_dmc_register_enable_notifier(&dmc_sync_nb);

	return 0;
}

static int rk3288_dmcfreq_remove(struct platform_device *pdev)
{
	devfreq_remove_device(dmcfreq.devfreq);
	regulator_put(dmcfreq.vdd_logic);

	return 0;
}

static struct platform_driver rk3288_dmcfreq_driver = {
	.probe	= rk3288_dmcfreq_probe,
	.remove	= rk3288_dmcfreq_remove,
	.driver = {
		.name	= "rk3288-dmc-freq",
		.pm	= &rk3288_dmcfreq_pm,
	},
};
module_platform_driver(rk3288_dmcfreq_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Rockchip dmcfreq driver with devfreq framework");
MODULE_AUTHOR("Derek Basehore <dbasehore@chromium.org>");
