/* Copyright (c) 2010-2013 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/bug.h>
#include <linux/clk-provider.h>

#include <dt-bindings/mfd/qcom-rpm.h>

#include "clk-rpm.h"

static DEFINE_MUTEX(rpm_clk_lock);

static int clk_rpm_set_active_rate(struct rpm_clk *r, u32 value)
{
	int ret = 0;
	ret = qcom_rpm_write(r->rpm, QCOM_RPM_ACTIVE_STATE,
				r->rpm_clk_id, &value, 1);

	/* Upon success save newly set rate in Hz*/
	if (ret == 0)
		r->rate = value;

	return ret;
}

static int clk_rpm_set_sleep_rate(struct rpm_clk *r, u32 value)
{
	return qcom_rpm_write(r->rpm, QCOM_RPM_SLEEP_STATE,
				r->rpm_clk_id, &value, 1);
}

static void to_active_sleep_khz(struct rpm_clk *r, u32 rate,
			u32 *active_khz, u32 *sleep_khz)
{
	/* Convert the rate (hz) to khz */
	*active_khz = DIV_ROUND_UP(rate, 1000);

	/*
	 * Active-only clocks don't care what the rate is during sleep. So,
	 * they vote for zero.
	 */
	if (r->active_only)
		*sleep_khz = 0;
	else
		*sleep_khz = *active_khz;
}

static int rpm_clk_prepare(struct clk_hw *hw)
{
	struct rpm_clk *r = to_rpm_clk(hw);
	int rc = 0;
	u32 this_khz, this_sleep_khz;

	mutex_lock(&rpm_clk_lock);

	/* Assume the clock is enabled if rate is not specified in DT */
	if (r->rate == 0) {
		r->enabled = true;
		goto out;
	}

	to_active_sleep_khz(r, r->rate, &this_khz, &this_sleep_khz);

	if (r->branch) {
		this_khz = !!this_khz;
		this_sleep_khz = !!this_sleep_khz;
	}
	rc = clk_rpm_set_active_rate(r, this_khz);
	if (rc)
		goto out;

	rc = clk_rpm_set_sleep_rate(r, this_sleep_khz);
	if (!rc)
		r->enabled = true;
out:
	mutex_unlock(&rpm_clk_lock);
	return rc;
}

static void rpm_clk_unprepare(struct clk_hw *hw)
{
	struct rpm_clk *r = to_rpm_clk(hw);

	r->enabled = false;

	return;
}

static int rpm_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct rpm_clk *r = to_rpm_clk(hw);
	u32 this_khz, this_sleep_khz;
	int rc = -EPERM;

	mutex_lock(&rpm_clk_lock);

	to_active_sleep_khz(r, rate, &this_khz, &this_sleep_khz);

	if (r->enabled) {
		rc = clk_rpm_set_active_rate(r, this_khz);
		if (rc)
			goto out;

		rc = clk_rpm_set_sleep_rate(r, this_sleep_khz);
	}
out:
	mutex_unlock(&rpm_clk_lock);
	return rc;
}

static unsigned long
rpm_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct rpm_clk *r = to_rpm_clk(hw);

	return r->rate;
}

static long rpm_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long *p_rate)
{
	struct rpm_clk *r = to_rpm_clk(hw);

	return r->rate;
}

const struct clk_ops clk_rpm_ops = {
	.prepare = rpm_clk_prepare,
	.unprepare = rpm_clk_unprepare,
	.set_rate = rpm_clk_set_rate,
	.recalc_rate = rpm_clk_recalc_rate,
	.round_rate = rpm_clk_round_rate,
};

const struct clk_ops clk_rpm_branch_ops = {
	.prepare = rpm_clk_prepare,
	.unprepare = rpm_clk_unprepare,
	.recalc_rate = rpm_clk_recalc_rate,
	.round_rate = rpm_clk_round_rate,
};

static const struct of_device_id clk_rpm_of_match[] = {
	{ .compatible = "qcom,rpm-clk", },
	{ },
};

static int rpm_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct rpm_clk *clk;
	struct clk *clock;
	u32 val;
	int ret;

	struct clk_init_data init;

	match = of_match_device(clk_rpm_of_match, &pdev->dev);
	if (!match)
		return -ENODEV;

	clk = devm_kzalloc(dev, sizeof(*clk), GFP_KERNEL);
	if (!clk) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	clk->dev = &pdev->dev;
	clk->rpm = dev_get_drvdata(pdev->dev.parent);
	if (!clk->rpm) {
		dev_err(&pdev->dev, "unable to retrieve handle to rpm\n");
		return -ENODEV;
	}

	/* load clock info from dts */
	ret = of_property_read_u32(pdev->dev.of_node, "reg", &val);
	if (ret) {
		dev_err(&pdev->dev, "failed to read reg.\n");
		return ret;
	}
	clk->rpm_clk_id = val;

	ret = of_property_read_string(pdev->dev.of_node,
				"qcom,rpm-clk-name", &init.name);
	if (ret) {
		dev_err(&pdev->dev, "failed to read qcom,rpm-clk-name\n");
		return ret;
	}
	init.ops = &clk_rpm_ops;
	init.flags = CLK_IS_ROOT;
	init.parent_names = NULL;
	init.num_parents =  0;
	clk->hw.init = &init;

	ret = of_property_read_u32(pdev->dev.of_node,
			"qcom,rpm-clk-freq", &val);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to read qcom,rpm-clk-freq\n");
		return ret;
	}
	clk->rate = val;

	clk->branch = of_property_read_bool(pdev->dev.of_node,
				"qcom,rpm-clk-branch");

	clk->active_only = of_property_read_bool(pdev->dev.of_node,
				"qcom,rpm-clk-active-only");

	clock = clk_register(dev, &clk->hw);
	if (IS_ERR(clock))
		return PTR_ERR(clock);

	ret = rpm_clk_prepare(&clk->hw);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to prepare %d\n", clk->rpm_clk_id);
		return ret;
	}

	return 0;
}

static struct platform_driver rpm_clk_driver = {
	.probe		= rpm_clk_probe,
	.driver		= {
		.name	= "qcom-rpm-clk",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(clk_rpm_of_match),
	},
};

static int __init rpm_clk_init(void)
{
	return platform_driver_register(&rpm_clk_driver);
}
core_initcall(rpm_clk_init);

static void __exit rpm_clk_exit(void)
{
	platform_driver_unregister(&rpm_clk_driver);
}
module_exit(rpm_clk_exit);

MODULE_DESCRIPTION("QCOM RPM CLOCK Driver");
MODULE_LICENSE("GPL v2");
