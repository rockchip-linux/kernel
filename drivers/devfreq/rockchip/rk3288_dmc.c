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
#include <linux/cpu.h>
#include <linux/cpufreq.h>
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
#include <linux/rwsem.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

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

#define DMC_NUM_RETRIES		3

#define DMC_MIN_CPU_KHZ		1200000

struct dmc_usage {
	u32 write;
	u32 read;
	u32 active;
	u32 time;
};

struct rk3288_dmcfreq {
	struct device *clk_dev;
	struct work_struct work;
	struct devfreq *devfreq;
	struct devfreq_simple_ondemand_data ondemand_data;
	struct clk *dmc_clk;
	struct regulator *vdd_logic;
	struct dmc_usage ch_usage[RK3288_DMC_NUM_CH];

	struct regmap *grf;
	struct regmap *pmu;
	struct regmap *noc;
	struct regmap *dmc;

	unsigned long rate, target_rate;
	unsigned long volt, target_volt;
	int err;

	struct notifier_block thermal_change_nb;
	unsigned long *freq_table;
	unsigned long num_levels;
	unsigned int thermal_throttling_level;
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
	unsigned long thermal_throttling_rate;

	rcu_read_lock();
	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		return PTR_ERR(opp);
	}

	if (dmcfreq.num_levels) {
		thermal_throttling_rate =
			dmcfreq.freq_table[dmcfreq.thermal_throttling_level];

		if (*freq > thermal_throttling_rate) {
			dev_dbg(dev, "Thermal throttling %lu to %lu\n", *freq,
				thermal_throttling_rate);

			*freq = thermal_throttling_rate;
			opp = devfreq_recommended_opp(
				dev, freq, DEVFREQ_FLAG_LEAST_UPPER_BOUND);
		}
	}

	dmcfreq.target_rate = dev_pm_opp_get_freq(opp);
	dmcfreq.target_volt = dev_pm_opp_get_voltage(opp);
	opp = devfreq_recommended_opp(dev, &dmcfreq.rate, flags);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		return PTR_ERR(opp);
	}
	dmcfreq.volt = dev_pm_opp_get_voltage(opp);
	rcu_read_unlock();

	if (dmcfreq.rate == dmcfreq.target_rate) {
		*freq = dmcfreq.rate;
		return 0;
	}

	queue_work(system_highpri_wq, &dmcfreq.work);
	flush_work(&dmcfreq.work);
	*freq = dmcfreq.rate;

	return dmcfreq.err;
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
	433000000,
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

static void rk3288_dmcfreq_work(struct work_struct *work)
{
	struct device *dev = dmcfreq.clk_dev;
	struct cpufreq_policy *policy;
	unsigned long rate = dmcfreq.rate;
	unsigned long target_rate = dmcfreq.target_rate;
	unsigned long volt = dmcfreq.volt;
	unsigned long target_volt = dmcfreq.target_volt;
	unsigned long old_clk_rate;
	unsigned int cpufreq_cur;
	int retries, err = 0;

	/*
	 * We need to prevent cpu hotplug from happening while a dmc freq rate
	 * change is happening. The rate change needs to synchronize all of the
	 * cpus. This has to be done here since the lock for preventing cpu
	 * hotplug needs to be grabbed before the clk prepare lock.
	 *
	 * Do this before taking the policy rwsem to avoid deadlocks between the
	 * mutex that is locked/unlocked in both get/put_online_cpus.
	 */
	get_online_cpus();
	/*
	 * Go to max cpufreq and block other cpufreq changes since set_rate
	 * needs to complete during vblank.
	 */
	policy = cpufreq_cpu_get(0);
	if (err) {
		dev_err(dev, "Error adjusting cpufreq %d\n", err);
		goto cpufreq;
	}
	down_write(&policy->rwsem);
	cpufreq_cur = cpufreq_quick_get(0);

	/* If we're thermally throttled; can't change; it won't work */
	if (policy->max < DMC_MIN_CPU_KHZ) {
		dev_warn(dev, "CPU too slow for DMC (%d MHz)\n", policy->max);
		goto out;
	}

	__cpufreq_driver_target(policy, policy->max, CPUFREQ_RELATION_L);

	if (rate < target_rate)
		err = regulator_set_voltage(dmcfreq.vdd_logic, target_volt,
					    target_volt);
	if (err) {
		dev_err(dev, "Unable to set voltage %lu\n", target_volt);
		goto out;
	}

	/* Get exact clk rate instead of cached value. */
	old_clk_rate = clk_get_rate(dmcfreq.dmc_clk);
	for (retries = 0; retries < DMC_NUM_RETRIES; retries++) {
		err = clk_set_rate(dmcfreq.dmc_clk, target_rate);
		/* Break if we see an error or the rate changes. */
		if (err || old_clk_rate != clk_get_rate(dmcfreq.dmc_clk))
			break;
	}

	if (err || old_clk_rate == clk_get_rate(dmcfreq.dmc_clk)) {
		dev_err(dev,
			"Unable to set freq %lu. Current freq %lu. Error %d\n",
			target_rate, old_clk_rate, err);
		regulator_set_voltage(dmcfreq.vdd_logic, volt, volt);
		goto out;
	}

	dmcfreq.rate = target_rate;

	if (rate > target_rate)
		err = regulator_set_voltage(dmcfreq.vdd_logic, target_volt,
					    target_volt);
	if (err)
		dev_err(dev, "Unable to set voltage %lu\n", target_volt);

out:
	/* Restore cpufreq and allow frequency changes. */
	__cpufreq_driver_target(policy, cpufreq_cur, CPUFREQ_RELATION_L);
	up_write(&policy->rwsem);
cpufreq:
	put_online_cpus();
	dmcfreq.err = err;
}

/*
 * This puts the frequency at max and suspends devfreq when there are too many
 * things to sync with (DMC_DISABLE). It resumes devfreq when there are few
 * enough things to sync with (DMC_ENABLE).
 */
static int rk3288_dmc_enable_notify(struct notifier_block *nb,
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

static int dmc_set_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long state)
{
	WARN_ON(state >= dmcfreq.num_levels);
	dmcfreq.thermal_throttling_level = state;
	dsb();

	return 0;
}

static int dmc_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	*state = dmcfreq.num_levels - 1;

	return 0;
}

static int dmc_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	*state = dmcfreq.thermal_throttling_level;

	return 0;
}

static struct thermal_cooling_device_ops const dmcfreq_cooling_ops = {
	.get_max_state = dmc_get_max_state,
	.get_cur_state = dmc_get_cur_state,
	.set_cur_state = dmc_set_cur_state,
};

static struct notifier_block dmc_enable_nb = {
	.notifier_call = rk3288_dmc_enable_notify,
};

static __maybe_unused int rk3288_dmcfreq_suspend(struct device *dev)
{
	rockchip_dmc_disable();

	return 0;
}

static __maybe_unused int rk3288_dmcfreq_resume(struct device *dev)
{
	rockchip_dmc_enable();

	return 0;
}

static SIMPLE_DEV_PM_OPS(rk3288_dmcfreq_pm, rk3288_dmcfreq_suspend,
			 rk3288_dmcfreq_resume);

static void rk3288_dmcfreq_shutdown(void)
{
	devfreq_suspend_device(dmcfreq.devfreq);
}

static struct syscore_ops rk3288_dmcfreq_syscore_ops = {
	.shutdown = rk3288_dmcfreq_shutdown,
};

static int rk3288_dmcfreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned long freq;
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

	INIT_WORK(&dmcfreq.work, rk3288_dmcfreq_work);
	dmcfreq.ondemand_data.upthreshold = DMC_UPTHRESHOLD;
	dmcfreq.ondemand_data.downdifferential = DMC_DOWNDIFFERENTIAL;
	dmcfreq.devfreq = devfreq_add_device(dmcfreq.clk_dev,
					     &rk3288_devfreq_dmc_profile,
					     "simple_ondemand",
					     &dmcfreq.ondemand_data);
	if (IS_ERR(dmcfreq.devfreq))
		return PTR_ERR(dmcfreq.devfreq);

	devfreq_register_opp_notifier(dmcfreq.clk_dev, dmcfreq.devfreq);
	rockchip_dmc_en_lock();
	if (!rockchip_dmc_enabled()) {
		devfreq_suspend_device(dmcfreq.devfreq);
		rk3288_dmc_stop_hardware_counter();
		freq = ULONG_MAX;
		rk3288_dmcfreq_target(dmcfreq.clk_dev, &freq, 0);
		dev_info(dev, "DVFS disabled at probe\n");
	}
	rockchip_dmc_register_enable_notifier(&dmc_enable_nb);
	rockchip_dmc_en_unlock();

	register_syscore_ops(&rk3288_dmcfreq_syscore_ops);

	if (of_find_property(dmcfreq.clk_dev->of_node, "#cooling-cells",
			     NULL)) {
		struct thermal_cooling_device *cdev;
		unsigned long freq;
		int length;
		int i;

		/*
		 * Make a table of frequencies as of probe time, just like
		 * everyone else using OPP.  Kinda defeats the purpose of
		 * OPP (doesn't handle someone else adding to the list), but
		 * oh well.
		 */
		rcu_read_lock();
		length = dev_pm_opp_get_opp_count(dmcfreq.clk_dev);
		dmcfreq.freq_table =
			devm_kzalloc(dmcfreq.clk_dev,
				     length * sizeof(*dmcfreq.freq_table),
				     GFP_KERNEL);
		if (!dmcfreq.freq_table) {
			dev_err(dmcfreq.clk_dev, "No memory for freq table\n");
			length = 0;
		}

		freq = ULONG_MAX;
		for (i = 0; i < length; i++) {
			struct dev_pm_opp *opp;

			opp = dev_pm_opp_find_freq_floor(dmcfreq.clk_dev,
							 &freq);
			if (IS_ERR(opp)) {
				dev_err(dmcfreq.clk_dev,
					"Error getting opp %d\n", i);
				length = i;
				break;
			}
			dmcfreq.freq_table[i] = freq;

			freq--;
		}
		rcu_read_unlock();
		dmcfreq.num_levels = length;

		if (length) {
			cdev = thermal_of_cooling_device_register(
				dmcfreq.clk_dev->of_node, "dmcthermal",
				&dmcfreq, &dmcfreq_cooling_ops);

			if (IS_ERR(cdev))
				pr_err("dmc w/out cooling device: %ld\n",
				PTR_ERR(cdev));
		}
	}

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
