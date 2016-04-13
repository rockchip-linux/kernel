/*
 * Regulator driver for Rockchip RK816
 *
 * Copyright (c) 2014, Fuzhou Rockchip Electronics Co., Ltd
 *
 * Author: Zhang Qing <zhangqing@rock-chips.com>
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/mfd/rk816.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

/* Field Definitions */
#define RK816_BUCK_VSEL_MASK	0x3f
#define RK816_BUCK4_VSEL_MASK	0x1f
#define RK816_BUCK5_VSEL_MASK	0x7
#define RK816_LDO_VSEL_MASK	0x1f

static int buck_set_vol_base_addr[] = {
	RK816_BUCK1_ON_VSEL_REG,
	RK816_BUCK2_ON_VSEL_REG,
	RK816_BUCK3_CONFIG_REG,
	RK816_BUCK4_ON_VSEL_REG,
	RK816_BUCK5_ON_VSEL_REG,
};
#define rk816_BUCK_SET_VOL_REG(x) (buck_set_vol_base_addr[x])

static int ldo_voltage_map[] = {
	800000, 900000, 1000000, 1100000, 1200000, 1300000, 1400000, 1500000,
	1600000, 1700000, 1800000, 1900000, 2000000, 2100000, 2200000, 2300000,
	2400000, 2500000, 2600000, 2700000, 2800000, 2900000, 3000000, 3100000,
	3200000, 3300000, 3400000,
};

/* Offset from XXX_ON_VSEL to XXX_SLP_VSEL */
#define RK816_SLP_REG_OFFSET 1

static int rk816_ldo_list_voltage(struct regulator_dev *dev, unsigned index)
{
	int volt;

	if (index < 0x0 || index > RK816_LDO_VSEL_MASK)
		return -EINVAL;

	volt = 800000 + index * 100000;

	return volt;
}

static int rk816_is_enabled(struct regulator_dev *dev)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	u16 val;

	val = rk816_reg_read(rk816, (u8)(dev->desc->enable_reg));
	if (val < 0)
		return val;

	if (val & dev->desc->enable_mask)
		return 1;
	else
		return 0;
}

static int rk816_enable(struct regulator_dev *dev)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	u16 enable_val, enable_mask;

	enable_mask = dev->desc->enable_mask | (dev->desc->enable_mask << 4);
	enable_val = dev->desc->enable_mask | (dev->desc->enable_mask << 4);

	return rk816_set_bits(rk816, dev->desc->enable_reg,
			      enable_mask, enable_val);
}

static int rk816_disable(struct regulator_dev *dev)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	u16 enable_val, enable_mask;

	enable_mask = dev->desc->enable_mask | (dev->desc->enable_mask << 4);
	enable_val = dev->desc->enable_mask << 4;

	return rk816_set_bits(rk816, dev->desc->enable_reg,
			      enable_mask, enable_val);
}

static int rk816_ldo_suspend_enable(struct regulator_dev *dev)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	int ldo = rdev_get_id(dev) - RK816_LDO1;
	u16 enable_val, enable_mask;

	enable_mask = (1 << ldo);
	enable_val = (1 << ldo);

	return rk816_set_bits(rk816, RK816_SLP_LDO_EN_REG,
			      enable_mask, enable_val);
}

static int rk816_ldo_suspend_disable(struct regulator_dev *dev)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	int ldo = rdev_get_id(dev) - RK816_LDO1;
	u16 enable_val, enable_mask;

	enable_mask = (1 << ldo);
	enable_val = (0 << ldo);

	return rk816_set_bits(rk816, RK816_SLP_LDO_EN_REG,
			      enable_mask, enable_val);
}

static int rk816_ldo_select_min_voltage(struct regulator_dev *dev,
					int min_uV, int max_uV)
{
	u16 vsel = 0;

	if (min_uV < 800000)
		vsel = 0;
	else if (min_uV <= 3400000)
		vsel = ((min_uV - 800000) / 100000);
	else
		return -EINVAL;

	if (rk816_ldo_list_voltage(dev, vsel) > max_uV)
		return -EINVAL;

	return vsel;
}

static int rk816_ldo_get_voltage(struct regulator_dev *dev)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	u16 reg = 0;
	int val;

	reg = rk816_reg_read(rk816, (u8)dev->desc->vsel_reg);
	reg &= dev->desc->vsel_mask;
	val = ldo_voltage_map[reg];

	return val;
}

static int rk816_ldo_set_sleep_voltage(struct regulator_dev *dev, int uV)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	u16 val;
	int ret = 0;

	val = rk816_ldo_select_min_voltage(dev, uV, uV);
	ret = rk816_set_bits(rk816, dev->desc->vsel_reg + RK816_SLP_REG_OFFSET,
			     dev->desc->vsel_mask, val);

	return ret;
}

static int rk816_ldo_set_voltage(struct regulator_dev *dev,
				 int min_uV, int max_uV, unsigned *selector)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	u16 val;
	int ret = 0;

	val = rk816_ldo_select_min_voltage(dev, min_uV, min_uV);
	ret = rk816_set_bits(rk816, dev->desc->vsel_reg,
			     dev->desc->vsel_mask, val);

	return ret;
}

static int rk816_dcdc_list_voltage(struct regulator_dev *dev, unsigned selector)
{
	int volt;
	int buck = rdev_get_id(dev) - RK816_DCDC1;

	switch (buck) {
	case 0:
	case 1:
		if (selector <= 0x3b)
			volt = 712500 + selector * 12500;
		else if (selector <= 0x3e)
			volt = 1800000 + (selector - 0x3c) * 200000;
		else
			volt = 2300000;
		break;
	case 2:
		volt = 1200000;
		break;
	case 3:
		if (selector >= 0x1b)
			selector = 0x1b;
		volt = 800000 + selector * 100000;
		break;
	default:
		BUG();
		return -EINVAL;
	}

	return  volt;
}

static int rk816_dcdc_suspend_enable(struct regulator_dev *dev)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev) - RK816_DCDC1;
	u16 enable_val, enable_mask;

	enable_mask = (1 << dcdc);
	enable_val = (1 << dcdc);

	return rk816_set_bits(rk816, RK816_SLP_DCDC_EN_REG,
			      enable_mask, enable_val);
}

static int rk816_dcdc_suspend_disable(struct regulator_dev *dev)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev) - RK816_DCDC1;
	u16 enable_val, enable_mask;

	enable_mask = (1 << dcdc);
	enable_val = (0 << dcdc);

	return rk816_set_bits(rk816, RK816_SLP_DCDC_EN_REG,
			      enable_mask, enable_val);
}

static int rk816_dcdc_get_voltage(struct regulator_dev *dev)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	u16 reg = 0;
	int val;

	reg = rk816_reg_read(rk816, (u8)dev->desc->vsel_reg);
	reg &= dev->desc->vsel_mask;
	val = rk816_dcdc_list_voltage(dev, reg);

	return val;
}

static int rk816_dcdc_select_min_voltage(struct regulator_dev *dev,
					 int min_uV, int max_uV, int buck)
{
	u16 vsel = 0;

	if (buck == 0 || buck == 1) {
		if (min_uV < 712500)
			vsel = 0;
		else if (min_uV <= 1450000)
			vsel = (min_uV - 712500) / 12500;
		else if (min_uV < 1800000)
			vsel = 0x3b;
		else if (min_uV < 2300000)
			vsel = 0x3c + (min_uV - 1800000) / 200000;
		else if (min_uV == 2300000)
			vsel = 0x3f;
		else
			return -EINVAL;
	} else if (buck == 3) {
		if (min_uV < 800000)
			vsel = 0;
		else if (min_uV <= 3500000)
			vsel = ((min_uV - 800000) / 100000);
		else
			return -EINVAL;
	}

	if (rk816_dcdc_list_voltage(dev, vsel) > max_uV)
		return -EINVAL;

	return vsel;
}

static int rk816_dcdc_set_voltage(struct regulator_dev *dev,
				  int min_uV, int max_uV, unsigned *selector)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	int buck = rdev_get_id(dev) - RK816_DCDC1;
	u16 val;
	int ret = 0;

	if (buck == 2) {
		return 0;
	} else {
		val = rk816_dcdc_select_min_voltage(dev, min_uV, max_uV, buck);
		ret = rk816_set_bits(rk816, dev->desc->vsel_reg,
				     dev->desc->vsel_mask, val);
	}

	if (ret < 0)
		pr_err("WARN: set voltage error! voltage set is %d mv %d\n",
		       min_uV, ret);

	return ret;
}

static int rk816_dcdc_set_sleep_voltage(struct regulator_dev *dev, int uV)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	int buck = rdev_get_id(dev) - RK816_DCDC1;
	u16 val;
	int ret = 0;

	if (buck == 2) {
		return 0;
	} else {
		val = rk816_dcdc_select_min_voltage(dev, uV, uV, buck);
		ret = rk816_set_bits(rk816,
				     dev->desc->vsel_reg + RK816_SLP_REG_OFFSET,
				     dev->desc->vsel_mask, val);
	}

	return ret;
}

static unsigned int rk816_dcdc_get_mode(struct regulator_dev *dev)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	int buck = rdev_get_id(dev) - RK816_DCDC1;
	u16 mask = 0x80;
	u16 val;

	val = rk816_reg_read(rk816, rk816_BUCK_SET_VOL_REG(buck));
	if (val < 0)
		return val;

	if (val & mask)
		return REGULATOR_MODE_FAST;
	else
		return REGULATOR_MODE_NORMAL;
}

static int rk816_dcdc_set_mode(struct regulator_dev *dev, unsigned int mode)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	int buck = rdev_get_id(dev) - RK816_DCDC1;
	u16 mask = 0x80;

	switch (mode) {
	case REGULATOR_MODE_FAST:
		return rk816_set_bits(rk816, rk816_BUCK_SET_VOL_REG(buck),
				      mask, mask);
	case REGULATOR_MODE_NORMAL:
		return rk816_set_bits(rk816, rk816_BUCK_SET_VOL_REG(buck),
				      mask, 0);
	default:
		pr_err("error:pmu_rk816 only powersave and pwm mode\n");
		return -EINVAL;
	}
}

static int rk816_dcdc_set_suspend_mode(struct regulator_dev *dev,
				       unsigned int mode)
{
	struct rk816 *rk816 = rdev_get_drvdata(dev);
	int buck = rdev_get_id(dev) - RK816_DCDC1;
	u16 mask = 0x80;
	int ret;

	switch (mode) {
	case REGULATOR_MODE_FAST:
		ret = rk816_set_bits(rk816,
				     (rk816_BUCK_SET_VOL_REG(buck) +
				     RK816_SLP_REG_OFFSET),
				     mask, mask);
		break;
	case REGULATOR_MODE_NORMAL:
		ret = rk816_set_bits(rk816,
				     (rk816_BUCK_SET_VOL_REG(buck) +
				     RK816_SLP_REG_OFFSET),
				     mask, 0);
		break;
	default:
		pr_err("error:pmu_rk816 only powersave and pwm mode\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rk816_dcdc_set_voltage_time_sel(struct regulator_dev *dev,
					   unsigned int old_selector,
					   unsigned int new_selector)
{
	int old_volt, new_volt;

	old_volt = rk816_dcdc_list_voltage(dev, old_selector);
	if (old_volt < 0)
		return old_volt;

	new_volt = rk816_dcdc_list_voltage(dev, new_selector);
	if (new_volt < 0)
		return new_volt;

	return DIV_ROUND_UP(abs(old_volt - new_volt) * 2, 12500);
}

static struct regulator_ops rk816_ldo_ops = {
	.set_voltage = rk816_ldo_set_voltage,
	.get_voltage = rk816_ldo_get_voltage,
	.list_voltage = rk816_ldo_list_voltage,
	.is_enabled = rk816_is_enabled,
	.enable = rk816_enable,
	.disable = rk816_disable,
	.set_suspend_enable = rk816_ldo_suspend_enable,
	.set_suspend_disable = rk816_ldo_suspend_disable,
	.set_suspend_voltage = rk816_ldo_set_sleep_voltage,
};

static struct regulator_ops rk816_dcdc_ops = {
	.set_voltage = rk816_dcdc_set_voltage,
	.get_voltage = rk816_dcdc_get_voltage,
	.list_voltage = rk816_dcdc_list_voltage,
	.is_enabled = rk816_is_enabled,
	.enable = rk816_enable,
	.disable = rk816_disable,
	.set_suspend_enable = rk816_dcdc_suspend_enable,
	.set_suspend_disable = rk816_dcdc_suspend_disable,
	.get_mode = rk816_dcdc_get_mode,
	.set_mode = rk816_dcdc_set_mode,
	.set_suspend_mode = rk816_dcdc_set_suspend_mode,
	.set_suspend_voltage = rk816_dcdc_set_sleep_voltage,
	.set_voltage_time_sel = rk816_dcdc_set_voltage_time_sel,
};

static const struct regulator_desc rk816_reg[] = {
	{
		.name = "RK816_DCDC1",
		.supply_name = "vcc1",
		.id = RK816_ID_DCDC1,
		.ops = &rk816_dcdc_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 64,
		.vsel_reg = RK816_BUCK1_ON_VSEL_REG,
		.vsel_mask = RK816_BUCK_VSEL_MASK,
		.enable_reg = RK816_DCDC_EN_REG1,
		.enable_mask = BIT(0),
		.owner = THIS_MODULE,
	}, {
		.name = "RK816_DCDC2",
		.supply_name = "vcc2",
		.id = RK816_ID_DCDC2,
		.ops = &rk816_dcdc_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 64,
		.vsel_reg = RK816_BUCK2_ON_VSEL_REG,
		.vsel_mask = RK816_BUCK_VSEL_MASK,
		.enable_reg = RK816_DCDC_EN_REG1,
		.enable_mask = BIT(1),
		.owner = THIS_MODULE,
	}, {
		.name = "RK816_DCDC3",
		.supply_name = "vcc3",
		.id = RK816_ID_DCDC3,
		.ops = &rk816_dcdc_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 1,
		.enable_reg = RK816_DCDC_EN_REG1,
		.enable_mask = BIT(2),
		.owner = THIS_MODULE,
	}, {
		.name = "RK816_DCDC4",
		.supply_name = "vcc4",
		.id = RK816_ID_DCDC4,
		.ops = &rk816_dcdc_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 32,
		.vsel_reg = RK816_BUCK4_ON_VSEL_REG,
		.vsel_mask = RK816_BUCK4_VSEL_MASK,
		.enable_reg = RK816_DCDC_EN_REG1,
		.enable_mask = BIT(3),
		.owner = THIS_MODULE,
	}, {
		.name = "RK816_LDO1",
		.supply_name = "vcc5",
		.id = RK816_ID_LDO1,
		.ops = &rk816_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = ARRAY_SIZE(ldo_voltage_map),
		.vsel_reg = RK816_LDO1_ON_VSEL_REG,
		.vsel_mask = RK816_LDO_VSEL_MASK,
		.enable_reg = RK816_LDO_EN_REG1,
		.enable_mask = BIT(0),
		.enable_time = 400,
		.owner = THIS_MODULE,
	}, {
		.name = "RK816_LDO2",
		.supply_name = "vcc6",
		.id = RK816_ID_LDO2,
		.ops = &rk816_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = ARRAY_SIZE(ldo_voltage_map),
		.vsel_reg = RK816_LDO2_ON_VSEL_REG,
		.vsel_mask = RK816_LDO_VSEL_MASK,
		.enable_reg = RK816_LDO_EN_REG1,
		.enable_mask = BIT(1),
		.enable_time = 400,
		.owner = THIS_MODULE,
	}, {
		.name = "RK816_LDO3",
		.supply_name = "vcc7",
		.id = RK816_ID_LDO3,
		.ops = &rk816_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = ARRAY_SIZE(ldo_voltage_map),
		.vsel_reg = RK816_LDO3_ON_VSEL_REG,
		.vsel_mask = RK816_LDO_VSEL_MASK,
		.enable_reg = RK816_LDO_EN_REG1,
		.enable_mask = BIT(2),
		.enable_time = 400,
		.owner = THIS_MODULE,
	}, {
		.name = "RK816_LDO4",
		.supply_name = "vcc8",
		.id = RK816_ID_LDO4,
		.ops = &rk816_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = ARRAY_SIZE(ldo_voltage_map),
		.vsel_reg = RK816_LDO4_ON_VSEL_REG,
		.vsel_mask = RK816_LDO_VSEL_MASK,
		.enable_reg = RK816_LDO_EN_REG1,
		.enable_mask = BIT(3),
		.enable_time = 400,
		.owner = THIS_MODULE,
	}, {
		.name = "RK816_LDO5",
		.supply_name = "vcc9",
		.id = RK816_ID_LDO5,
		.ops = &rk816_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = ARRAY_SIZE(ldo_voltage_map),
		.vsel_reg = RK816_LDO5_ON_VSEL_REG,
		.vsel_mask = RK816_LDO_VSEL_MASK,
		.enable_reg = RK816_LDO_EN_REG2,
		.enable_mask = BIT(0),
		.enable_time = 400,
		.owner = THIS_MODULE,
	}, {
		.name = "RK816_LDO6",
		.supply_name = "vcc10",
		.id = RK816_ID_LDO6,
		.ops = &rk816_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = ARRAY_SIZE(ldo_voltage_map),
		.vsel_reg = RK816_LDO6_ON_VSEL_REG,
		.vsel_mask = RK816_LDO_VSEL_MASK,
		.enable_reg = RK816_LDO_EN_REG2,
		.enable_mask = BIT(1),
		.enable_time = 400,
		.owner = THIS_MODULE,
	}
};

static struct of_regulator_match rk816_reg_matches[] = {
	[RK816_ID_DCDC1]	= { .name = "RK816_DCDC1" },
	[RK816_ID_DCDC2]	= { .name = "RK816_DCDC2" },
	[RK816_ID_DCDC3]	= { .name = "RK816_DCDC3" },
	[RK816_ID_DCDC4]	= { .name = "RK816_DCDC4" },
	[RK816_ID_LDO1]		= { .name = "RK816_LDO1" },
	[RK816_ID_LDO2]		= { .name = "RK816_LDO2" },
	[RK816_ID_LDO3]		= { .name = "RK816_LDO3" },
	[RK816_ID_LDO4]		= { .name = "RK816_LDO4" },
	[RK816_ID_LDO5]		= { .name = "RK816_LDO5" },
	[RK816_ID_LDO6]		= { .name = "RK816_LDO6" },
};

int rk816_regulator_dt_parse_pdata(struct device *dev,
				   struct device *client_dev)
{
	struct device_node *np;
	int ret;

	np = of_get_child_by_name(client_dev->of_node, "regulators");
	if (!np)
		return -ENXIO;

	ret = of_regulator_match(dev, np, rk816_reg_matches,
				 RK816_NUM_REGULATORS);
	if (ret < 0)
		goto dt_parse_end;

dt_parse_end:
	of_node_put(np);
	return ret;
}

static int rk816_regulator_probe(struct platform_device *pdev)
{
	struct rk816 *rk816 = dev_get_drvdata(pdev->dev.parent);
	struct i2c_client *client = rk816->i2c;
	struct regulator_config config = {};
	struct regulator_dev *rk816_rdev;
	int ret, i;

	ret = rk816_regulator_dt_parse_pdata(&pdev->dev, &client->dev);
	if (ret < 0)
		return ret;

	/* Instantiate the regulators */
	rk816->rdev = devm_kcalloc(&pdev->dev, RK816_NUM_REGULATORS,
				   sizeof(struct regulator_dev *), GFP_KERNEL);
	if (!rk816->rdev)
		return -ENOMEM;

	for (i = 0; i < RK816_NUM_REGULATORS; i++) {
		if (!rk816_reg_matches[i].init_data ||
		    !rk816_reg_matches[i].of_node)
			continue;

		config.dev = &client->dev;
		config.driver_data = rk816;
		config.regmap = rk816->regmap;
		config.of_node = rk816_reg_matches[i].of_node;
		config.init_data = rk816_reg_matches[i].init_data;

		rk816_rdev = regulator_register(&rk816_reg[i], &config);
		if (IS_ERR(rk816_rdev)) {
			dev_err(&client->dev,
				"failed to register %d regulator\n", i);
			ret = PTR_ERR(rk816_rdev);
			goto err_unregister_regulator;
		}

		rk816->rdev[i] = rk816_rdev;
	}

	return 0;
err_unregister_regulator:
	while (--i >= 0)
		regulator_unregister(rk816->rdev[i]);

	return ret;
}

static int rk816_regulator_remove(struct platform_device *pdev)
{
	struct rk816 *rk816 = dev_get_drvdata(pdev->dev.parent);
	int i;

	for (i = 0; i < RK816_NUM_REGULATORS; i++)
		if (rk816->rdev[i])
			regulator_unregister(rk816->rdev[i]);
	return 0;
}

static struct platform_driver rk816_regulator_driver = {
	.probe = rk816_regulator_probe,
	.remove = rk816_regulator_remove,
	.driver = {
		.name = "rk816-regulator",
		.owner = THIS_MODULE,
	},
};

static int __init rk816_regulator_init(void)
{
	return platform_driver_register(&rk816_regulator_driver);
}
subsys_initcall_sync(rk816_regulator_init);

static void __exit rk816_regulator_cleanup(void)
{
	platform_driver_unregister(&rk816_regulator_driver);
}
module_exit(rk816_regulator_cleanup);

MODULE_DESCRIPTION("regulator driver for the rk816 series PMICs");
MODULE_AUTHOR("Zhang Qing<zhangqing@rock-chips.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rk816-regulator");
