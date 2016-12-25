/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Copyright (C) 2014 Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * The OPP code in function set_target() is reused from
 * drivers/cpufreq/omap-cpufreq.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/cpufreq-dt.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#define MAX_CLUSTERS		2
#define MAX_PROP_NAME_LEN	6
#define VERSION_ELEMENTS	1
#define INVALID_VALUE		0xff
#define SAFE_FREQ	(24 * 67 * 1000000)

struct private_data {
	struct device *cpu_dev;
	struct thermal_cooling_device *cdev;
	const char *reg_name;
};

static struct freq_attr *cpufreq_dt_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,   /* Extra space for boost-attr if required */
	NULL,
};

static unsigned char package_info = INVALID_VALUE;
extern unsigned long apll_safefreq;
static int set_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct private_data *priv = policy->driver_data;

	return dev_pm_opp_set_rate(priv->cpu_dev,
				   policy->freq_table[index].frequency * 1000);
}

static int fetch_package_info(struct device *dev, unsigned char *package_info)
{
	struct nvmem_cell *cell;
	unsigned char *buf;
	size_t len;

	cell = nvmem_cell_get(dev, "package_info");
	if (IS_ERR(cell))
		return PTR_ERR(cell);

	buf = (unsigned char *)nvmem_cell_read(cell, &len);

	nvmem_cell_put(cell);

	if (IS_ERR(buf))
		return PTR_ERR(buf);

	if (buf[0] == INVALID_VALUE)
		return -EINVAL;

	*package_info = buf[0];
	kfree(buf);

	return 0;
}

static int set_opp_info(struct device *dev)
{
	struct device_node *opp_np, *np;
	unsigned char chip_vesion;
	char name[MAX_PROP_NAME_LEN];
	unsigned int version;
	unsigned int count = 0;
	int ret;

	if (package_info == INVALID_VALUE) {
		dev_info(dev, "package_info NULL\n");
		return 0;
	}

	opp_np = of_parse_phandle(dev->of_node, "operating-points-v2", 0);
	if (!opp_np)
		return 0;
	for_each_available_child_of_node(opp_np, np) {
		ret = of_property_read_u32_index(np, "opp-supported-hw",
						 VERSION_ELEMENTS - 1,
						 &version);
		if (!ret) {
			count++;
			break;
		}
	}
	if (count == 0)
		return 0;

	if ((package_info & 0xc0) == 0xc0) {
		chip_vesion = 1;
		apll_safefreq = ULONG_MAX;
	} else {
		chip_vesion = 0;
		apll_safefreq = SAFE_FREQ;
	}
	snprintf(name, MAX_PROP_NAME_LEN, "v%d", chip_vesion);
	ret = dev_pm_opp_set_prop_name(dev, name);
	if (ret) {
		dev_err(dev, "Failed to set prop name\n");
		return ret;
	}

	version = BIT(chip_vesion);
	ret = dev_pm_opp_set_supported_hw(dev, &version, VERSION_ELEMENTS);
	if (ret) {
		dev_err(dev, "Failed to set supported hardware\n");
		return ret;
	}

	dev_info(dev, "chip_version: %d 0x%x\n", chip_vesion, version);

	return 0;
}

/*
 * An earlier version of opp-v1 bindings used to name the regulator
 * "cpu0-supply", we still need to handle that for backwards compatibility.
 */
static const char *find_supply_name(struct device *dev)
{
	struct device_node *np;
	struct property *pp;
	int cpu = dev->id;
	const char *name = NULL;

	np = of_node_get(dev->of_node);

	/* This must be valid for sure */
	if (WARN_ON(!np))
		return NULL;

	/* Try "cpu0" for older DTs */
	if (!cpu) {
		pp = of_find_property(np, "cpu0-supply", NULL);
		if (pp) {
			name = "cpu0";
			goto node_put;
		}
	}

	pp = of_find_property(np, "cpu-supply", NULL);
	if (pp) {
		name = "cpu";
		goto node_put;
	}

	dev_dbg(dev, "no regulator for cpu%d\n", cpu);
node_put:
	of_node_put(np);
	return name;
}

static int resources_available(void)
{
	struct device *cpu_dev;
	struct regulator *cpu_reg;
	struct clk *cpu_clk;
	int ret = 0;
	const char *name;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev) {
		pr_err("failed to get cpu0 device\n");
		return -ENODEV;
	}

	cpu_clk = clk_get(cpu_dev, NULL);
	ret = PTR_ERR_OR_ZERO(cpu_clk);
	if (ret) {
		/*
		 * If cpu's clk node is present, but clock is not yet
		 * registered, we should try defering probe.
		 */
		if (ret == -EPROBE_DEFER)
			dev_dbg(cpu_dev, "clock not ready, retry\n");
		else
			dev_err(cpu_dev, "failed to get clock: %d\n", ret);

		return ret;
	}

	clk_put(cpu_clk);

	name = find_supply_name(cpu_dev);
	/* Platform doesn't require regulator */
	if (!name)
		return 0;

	cpu_reg = regulator_get_optional(cpu_dev, name);
	ret = PTR_ERR_OR_ZERO(cpu_reg);
	if (ret) {
		/*
		 * If cpu's regulator supply node is present, but regulator is
		 * not yet registered, we should try defering probe.
		 */
		if (ret == -EPROBE_DEFER)
			dev_dbg(cpu_dev, "cpu0 regulator not ready, retry\n");
		else
			dev_dbg(cpu_dev, "no regulator for cpu0: %d\n", ret);

		return ret;
	}

	regulator_put(cpu_reg);

	ret = fetch_package_info(cpu_dev, &package_info);
	if (ret)
		dev_dbg(cpu_dev, "Failed to fetch wafer_info\n");

	return 0;
}

static int cpufreq_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *freq_table;
	struct private_data *priv;
	struct device *cpu_dev;
	struct clk *cpu_clk;
	struct dev_pm_opp *suspend_opp;
	struct cpumask cpus;

	unsigned int transition_latency;
	unsigned long cur_freq;
	bool opp_v1 = false;
	const char *name;
	int ret;
	static int check_init;

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev) {
		pr_err("failed to get cpu%d device\n", policy->cpu);
		return -ENODEV;
	}

	cpu_clk = clk_get(cpu_dev, NULL);
	if (IS_ERR(cpu_clk)) {
		ret = PTR_ERR(cpu_clk);
		dev_err(cpu_dev, "%s: failed to get clk: %d\n", __func__, ret);
		return ret;
	}

	/* Get OPP-sharing information from "operating-points-v2" bindings */
	ret = dev_pm_opp_of_get_sharing_cpus(cpu_dev, policy->cpus);
	if (ret) {
		/*
		 * operating-points-v2 not supported, fallback to old method of
		 * finding shared-OPPs for backward compatibility.
		 */
		if (ret == -ENOENT)
			opp_v1 = true;
		else
			goto out_put_clk;
	}

	if (!opp_v1) {
		ret = set_opp_info(cpu_dev);
		if (ret)
			dev_err(cpu_dev, "Failed to set_opp_info: %d\n", ret);
	}

	/*
	 * OPP layer will be taking care of regulators now, but it needs to know
	 * the name of the regulator first.
	 */
	name = find_supply_name(cpu_dev);
	if (name) {
		ret = dev_pm_opp_set_regulator(cpu_dev, name);
		if (ret) {
			dev_err(cpu_dev, "Failed to set regulator for cpu%d: %d\n",
				policy->cpu, ret);
			goto out_put_clk;
		}
	}

	/*
	 * Initialize OPP tables for all policy->cpus. They will be shared by
	 * all CPUs which have marked their CPUs shared with OPP bindings.
	 *
	 * For platforms not using operating-points-v2 bindings, we do this
	 * before updating policy->cpus. Otherwise, we will end up creating
	 * duplicate OPPs for policy->cpus.
	 *
	 * OPPs might be populated at runtime, don't check for error here
	 */
	ret = dev_pm_opp_of_add_table(cpu_dev);
	if (ret) {
		dev_err(cpu_dev, "couldn't find opp table for cpu:%d, %d\n",
			policy->cpu, ret);
	} else {
		cpumask_copy(&cpus, policy->cpus);
		cpumask_clear_cpu(policy->cpu, &cpus);
		if (!cpumask_empty(&cpus)) {
			if (dev_pm_opp_of_cpumask_add_table(&cpus))
				dev_pm_opp_of_remove_table(cpu_dev);
		}
	}

	/*
	 * But we need OPP table to function so if it is not there let's
	 * give platform code chance to provide it for us.
	 */
	ret = dev_pm_opp_get_opp_count(cpu_dev);
	if (ret <= 0) {
		dev_dbg(cpu_dev, "OPP table is not ready, deferring probe\n");
		ret = -EPROBE_DEFER;
		goto out_free_opp;
	}

	if (opp_v1) {
		struct cpufreq_dt_platform_data *pd = cpufreq_get_driver_data();

		if (!pd || !pd->independent_clocks)
			cpumask_setall(policy->cpus);

		/*
		 * OPP tables are initialized only for policy->cpu, do it for
		 * others as well.
		 */
		ret = dev_pm_opp_set_sharing_cpus(cpu_dev, policy->cpus);
		if (ret)
			dev_err(cpu_dev, "%s: failed to mark OPPs as shared: %d\n",
				__func__, ret);
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto out_free_opp;
	}

	priv->reg_name = name;

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
		goto out_free_priv;
	}

	priv->cpu_dev = cpu_dev;
	policy->driver_data = priv;
	policy->clk = cpu_clk;

	rcu_read_lock();
	suspend_opp = dev_pm_opp_get_suspend_opp(cpu_dev);
	if (suspend_opp)
		policy->suspend_freq = dev_pm_opp_get_freq(suspend_opp) / 1000;
	rcu_read_unlock();

	ret = cpufreq_table_validate_and_show(policy, freq_table);
	if (ret) {
		dev_err(cpu_dev, "%s: invalid frequency table: %d\n", __func__,
			ret);
		goto out_free_cpufreq_table;
	}

	/* Support turbo/boost mode */
	if (policy_has_boost_freq(policy)) {
		/* This gets disabled by core on driver unregister */
		ret = cpufreq_enable_boost_support();
		if (ret)
			goto out_free_cpufreq_table;
		cpufreq_dt_attr[1] = &cpufreq_freq_attr_scaling_boost_freqs;
	}

	transition_latency = dev_pm_opp_get_max_transition_latency(cpu_dev);
	if (!transition_latency)
		transition_latency = CPUFREQ_ETERNAL;

	policy->cpuinfo.transition_latency = transition_latency;

	if (check_init < MAX_CLUSTERS) {
		ret = dev_pm_opp_check_initial_rate(cpu_dev, &cur_freq);
		if (!ret)
			policy->cur = cur_freq / 1000;
		check_init++;
	}

	return 0;

out_free_cpufreq_table:
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);
out_free_priv:
	kfree(priv);
out_free_opp:
	dev_pm_opp_of_cpumask_remove_table(policy->cpus);
	if (name)
		dev_pm_opp_put_regulator(cpu_dev);
out_put_clk:
	clk_put(cpu_clk);

	return ret;
}

static int cpufreq_exit(struct cpufreq_policy *policy)
{
	struct cpumask cpus;
	struct private_data *priv = policy->driver_data;

	cpumask_set_cpu(policy->cpu, policy->cpus);
	if (cpufreq_generic_suspend(policy))
		pr_err("%s: Failed to suspend driver: %p\n", __func__, policy);
	cpumask_clear_cpu(policy->cpu, policy->cpus);

	priv->cpu_dev = get_cpu_device(policy->cpu);
	cpufreq_cooling_unregister(priv->cdev);
	dev_pm_opp_free_cpufreq_table(priv->cpu_dev, &policy->freq_table);
	cpumask_copy(&cpus, policy->related_cpus);
	cpumask_clear_cpu(policy->cpu, &cpus);
	dev_pm_opp_of_cpumask_remove_table(&cpus);
	dev_pm_opp_of_remove_table(priv->cpu_dev);
	if (priv->reg_name)
		dev_pm_opp_put_regulator(priv->cpu_dev);

	clk_put(policy->clk);
	kfree(priv);

	return 0;
}

static void cpufreq_ready(struct cpufreq_policy *policy)
{
	struct private_data *priv = policy->driver_data;
	struct device_node *np = of_node_get(priv->cpu_dev->of_node);

	if (WARN_ON(!np))
		return;

	/*
	 * For now, just loading the cooling device;
	 * thermal DT code takes care of matching them.
	 */
	if (of_find_property(np, "#cooling-cells", NULL)) {
		u32 power_coefficient = 0;

		of_property_read_u32(np, "dynamic-power-coefficient",
				     &power_coefficient);

		priv->cdev = of_cpufreq_power_cooling_register(np,
				policy->related_cpus, power_coefficient, NULL);
		if (IS_ERR(priv->cdev)) {
			dev_err(priv->cpu_dev,
				"running cpufreq without cooling device: %ld\n",
				PTR_ERR(priv->cdev));

			priv->cdev = NULL;
		}
	}

	of_node_put(np);
}

static struct cpufreq_driver dt_cpufreq_driver = {
	.flags = CPUFREQ_STICKY | CPUFREQ_NEED_INITIAL_FREQ_CHECK |
			 CPUFREQ_HAVE_GOVERNOR_PER_POLICY,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = set_target,
	.get = cpufreq_generic_get,
	.init = cpufreq_init,
	.exit = cpufreq_exit,
	.ready = cpufreq_ready,
	.name = "cpufreq-dt",
	.attr = cpufreq_dt_attr,
	.suspend = cpufreq_generic_suspend,
};

static int dt_cpufreq_probe(struct platform_device *pdev)
{
	int ret;

	/*
	 * All per-cluster (CPUs sharing clock/voltages) initialization is done
	 * from ->init(). In probe(), we just need to make sure that clk and
	 * regulators are available. Else defer probe and retry.
	 *
	 * FIXME: Is checking this only for CPU0 sufficient ?
	 */
	ret = resources_available();
	if (ret)
		return ret;

	dt_cpufreq_driver.driver_data = dev_get_platdata(&pdev->dev);

	ret = cpufreq_register_driver(&dt_cpufreq_driver);
	if (ret)
		dev_err(&pdev->dev, "failed register driver: %d\n", ret);

	return ret;
}

static int dt_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&dt_cpufreq_driver);
	return 0;
}

static struct platform_driver dt_cpufreq_platdrv = {
	.driver = {
		.name	= "cpufreq-dt",
	},
	.probe		= dt_cpufreq_probe,
	.remove		= dt_cpufreq_remove,
};
module_platform_driver(dt_cpufreq_platdrv);

MODULE_ALIAS("platform:cpufreq-dt");
MODULE_AUTHOR("Viresh Kumar <viresh.kumar@linaro.org>");
MODULE_AUTHOR("Shawn Guo <shawn.guo@linaro.org>");
MODULE_DESCRIPTION("Generic cpufreq driver");
MODULE_LICENSE("GPL");
