/*
 * Driver for tps549b22 I2C DCDC
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bug.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define TPS549b22_DEBUG			0

static int tps549b22_bebug;
module_param(tps549b22_bebug, int, 0644);

#define TPS549b22_INFO(fmt, ...) \
do { \
	if (tps549b22_bebug) \
		pr_info("TPS549b22 DBG %s[%d]:"fmt, \
			__func__, \
			__LINE__, \
			##__VA_ARGS__); \
} while (0)

#define TPS549b22_REG_OPERATION		0x01
#define TPS549b22_REG_ON_OFF_CFG	0x02
#define TPS549b22_REG_WRITE_PROTECT	0x10
#define TPS549b22_REG_COMMAND		0x21
#define TPS549b22_REG_MRG_H		0x25
#define TPS549b22_REG_MRG_L		0x26
#define TPS549b22_REG_ST_BYTE		0x78
#define TPS549b22_REG_MFR_SPC_44	0xfc

#define TPS549b22_ID			0x0200

#define VOL_MSK				0x3ff
#define VOL_OFF_MSK			0x40
#define OPERATION_ON_MSK		0x80
#define OPERATION_MRG_MSK		0x3c
#define ON_OFF_CFG_OPT_MSK		0x0c
#define VOL_MIN_IDX			0x133
#define VOL_MAX_IDX			0x266
#define VOL_STEP			2500

#define VOL2REG(vol_sel) \
		(((vol_sel) / VOL_STEP) & VOL_MSK)
#define REG2VOL(val) \
		(VOL_STEP * ((val) & VOL_MSK))

#define VOL_MAX				1500000
#define VOL_MIN				780000

#define TPS549b22_SPEED			(400 * 1000)
#define TPS549b22_NUM_REGULATORS	1

struct tps549b22 {
	struct device *dev;
	struct mutex io_lock; /* exclusion in i2c read/write */
	struct i2c_client *i2c;
	int num_regulators;
	struct regulator_dev **rdev;
	struct regmap *regmap;
};

static struct tps549b22 *g_tps549b22;

struct tps549b22_board {
	struct regulator_init_data
		*tps549b22_init_data[TPS549b22_NUM_REGULATORS];
	struct device_node *of_node[TPS549b22_NUM_REGULATORS];
};

struct tps549b22_regulator_subdev {
	int id;
	struct regulator_init_data *initdata;
	struct device_node *reg_node;
};

static int tps549b22_dcdc_list_voltage(struct regulator_dev *rdev,
				       unsigned int index)
{
	if (index + VOL_MIN_IDX > VOL_MAX_IDX)
		return -EINVAL;

	return REG2VOL(index + VOL_MIN_IDX);
}

static int tps549b22_i2c_read(struct i2c_client *i2c,
			      u8 reg,
			      int count,
			      u16 *dest)

{
	int ret = -1;
	struct i2c_adapter *adap;
	struct i2c_msg msgs[2];

	if (!i2c || count < 1)
		return ret;

	*dest = 0;

	adap = i2c->adapter;

	msgs[0].addr = i2c->addr;
	msgs[0].buf = &reg;
	msgs[0].flags = i2c->flags;
	msgs[0].len = 1;
	msgs[0].scl_rate = TPS549b22_SPEED;

	msgs[1].buf = (u8 *)dest;
	msgs[1].addr = i2c->addr;
	msgs[1].flags = i2c->flags | I2C_M_RD;
	msgs[1].len = count;
	msgs[1].scl_rate = TPS549b22_SPEED;
	ret = i2c_transfer(adap, msgs, 2);

	return ret;
}

static int tps549b22_i2c_write(struct i2c_client *i2c,
			       u8 reg,
			       int count,
			       const u16 src)
{
	int ret = -1;
	struct i2c_adapter *adap;
	struct i2c_msg msg;
	char tx_buf[3];

	if (!i2c)
		return ret;

	if (count < 1)
		return ret;

	adap = i2c->adapter;

	tx_buf[0] = reg;
	tx_buf[1] = src & 0xff;
	tx_buf[2] = (src >> 8) & 0xff;

	msg.addr = i2c->addr;
	msg.buf = &tx_buf[0];
	msg.len = count + 1;
	msg.flags = i2c->flags;
	msg.scl_rate = TPS549b22_SPEED;

	ret = i2c_transfer(adap, &msg, 1);

	return ret;
}

static int tps549b22_reg_read(struct tps549b22 *tps549b22,
			      u8 reg,
			      int count,
			      u16 *dest)
{
	int ret;

	mutex_lock(&tps549b22->io_lock);

	ret = tps549b22_i2c_read(tps549b22->i2c, reg, count, dest);

	mutex_unlock(&tps549b22->io_lock);

	return ret;
}

static int tps549b22_set_bits(struct tps549b22 *tps549b22,
			      u8 reg,
			      u16 mask,
			      u16 val,
			      int count)
{
	u16 tmp = 0;
	int ret;

	mutex_lock(&tps549b22->io_lock);
	ret = tps549b22_i2c_read(tps549b22->i2c, reg, count, &tmp);
	TPS549b22_INFO("1 reg read 0x%x -> 0x%x\n",
		       (int)reg,
		       (unsigned int)tmp);
	if (ret < 0)
		goto out;

	tmp = (tmp & ~mask) | (val & mask);
	ret = tps549b22_i2c_write(tps549b22->i2c, reg, count, tmp);
	TPS549b22_INFO("2 reg write 0x%x -> 0x%x\n",
		       (int)reg,
		       (unsigned int)tmp);
	if (ret < 0)
		goto out;

	ret = tps549b22_i2c_read(tps549b22->i2c, reg, count, &tmp);
	TPS549b22_INFO("3 reg read 0x%x -> 0x%x\n",
		       (int)reg,
		       (unsigned int)tmp);
	if (ret < 0)
		goto out;

	ret = 0;
out:
	mutex_unlock(&tps549b22->io_lock);
	return ret;
}

static int tps549b22_reg_init(struct tps549b22 *tps549b22)
{
	if (tps549b22_set_bits(tps549b22,
			       TPS549b22_REG_OPERATION,
			       OPERATION_ON_MSK,
			       0x80,
			       1) == 0)
		return tps549b22_set_bits(tps549b22,
					  TPS549b22_REG_ON_OFF_CFG,
					  ON_OFF_CFG_OPT_MSK,
					  0x0c,
					  1);

	return -1;
}

static int tps549b22dcdc_is_enabled(struct regulator_dev *rdev)
{
	struct tps549b22 *tps549b22 = rdev_get_drvdata(rdev);
	int ret;
	u16 val;

	ret = tps549b22_reg_read(tps549b22, TPS549b22_REG_ST_BYTE, 1, &val);
	if (ret < 0)
		return 0;

	return !(val & VOL_OFF_MSK);
}

static int tps549b22dcdc_enable(struct regulator_dev *rdev)
{
	struct tps549b22 *tps549b22 = rdev_get_drvdata(rdev);

	return tps549b22_set_bits(tps549b22,
				  TPS549b22_REG_OPERATION,
				  OPERATION_ON_MSK,
				  0x80,
				  1);
}

static int tps549b22dcdc_disable(struct regulator_dev *rdev)
{
	struct tps549b22 *tps549b22 = rdev_get_drvdata(rdev);

	return tps549b22_set_bits(tps549b22,
				  TPS549b22_REG_OPERATION,
				  OPERATION_ON_MSK,
				  0,
				  1);
}

static int tps549b22dcdc_get_voltage(struct regulator_dev *rdev)
{
	struct tps549b22 *tps549b22 = rdev_get_drvdata(rdev);
	int ret;
	u16 val = 0;

	ret = tps549b22_reg_read(tps549b22, TPS549b22_REG_COMMAND, 2, &val);

	if (ret >= 0)
		return REG2VOL(val);

	return -1;
}

static int tps549b22dcdc_set_voltage(struct regulator_dev *rdev,
				     int min_uV,
				     int max_uV,
				     unsigned int *selector)
{
	struct tps549b22 *tps549b22 = rdev_get_drvdata(rdev);
	u16 val;
	int ret = 0;

	if (min_uV < REG2VOL(VOL_MIN_IDX) ||
	    min_uV > REG2VOL(VOL_MAX_IDX)) {
		dev_warn(rdev_get_dev(rdev),
			 "this voltage is out of limit! voltage set is %d mv\n",
			 REG2VOL(min_uV));
		return -EINVAL;
	}

	for (val = VOL_MIN_IDX; val <= VOL_MAX_IDX; val++)
		if (REG2VOL(val) >= min_uV)
			break;

	if (REG2VOL(val) > max_uV)
		dev_warn(rdev_get_dev(rdev),
			 "this voltage is not support! voltage set is %d mv\n",
			 REG2VOL(val));

	ret = tps549b22_set_bits(tps549b22,
				 TPS549b22_REG_COMMAND,
				 VOL_MSK,
				 val,
				 2);
	if (ret < 0)
		dev_err(rdev_get_dev(rdev),
			"set voltage is error! voltage set is %d mv %d\n",
			REG2VOL(val), ret);

	return ret;
}

static struct regulator_ops tps549b22dcdc_ops = {
	.set_voltage = tps549b22dcdc_set_voltage,
	.get_voltage = tps549b22dcdc_get_voltage,
	.is_enabled = tps549b22dcdc_is_enabled,
	.enable = tps549b22dcdc_enable,
	.disable = tps549b22dcdc_disable,
	.list_voltage = tps549b22_dcdc_list_voltage,
};

static struct regulator_desc regulators[] = {
	{
		.name = " tps549b22_DCDC1",
		.id = 0,
		.ops = &tps549b22dcdc_ops,
		.n_voltages = VOL_MAX_IDX - VOL_MIN_IDX + 1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

#ifdef CONFIG_OF
static struct of_regulator_match tps549b22_reg_matches[] = {
	{ .name = "tps549b22_dcdc1", .driver_data = (void *)0},
};

static struct tps549b22_board *tps549b22_parse_dt(struct tps549b22 *tps549b22)
{
	struct tps549b22_board *pdata;
	struct device_node *regs;
	struct device_node *tps549b22_np;
	int count;

	tps549b22_np = of_node_get(tps549b22->dev->of_node);
	if (!tps549b22_np) {
		pr_err("could not find pmic sub-node\n");
		goto err;
	}

	regs = of_find_node_by_name(tps549b22_np, "regulators");
	if (!regs)
		goto err;

	count = of_regulator_match(tps549b22->dev, regs, tps549b22_reg_matches,
				   TPS549b22_NUM_REGULATORS);
	of_node_put(regs);
	of_node_put(tps549b22_np);

	if (count <= 0)
		goto err;

	pdata = devm_kzalloc(tps549b22->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		goto err;

	pdata->tps549b22_init_data[0] = tps549b22_reg_matches[0].init_data;
	pdata->of_node[0] = tps549b22_reg_matches[0].of_node;

	return pdata;

err:
	return NULL;
}

#else
static struct tps549b22_board *tps549b22_parse_dt(struct i2c_client *i2c)
{
	return NULL;
}
#endif

#if TPS549b22_DEBUG
static struct dentry *debugfs_rootdir;

static ssize_t tps549b22_read(struct file *file,
			      char __user *userbuf,
			      size_t count,
			      loff_t *ppos)
{
	char buf[20] = {0};
	struct tps549b22 *t = (struct tps549b22 *)file->f_inode->i_private;

	sprintf(buf, "%d\n", tps549b22dcdc_get_voltage(t->rdev[0]));

	return simple_read_from_buffer(userbuf, count, ppos, buf, strlen(buf));
}

static ssize_t tps549b22_write(struct file *file,
			       const char __user *userbuf,
			       size_t count,
			       loff_t *ppos)
{
	char buf[20] = {0};
	unsigned int voltage;
	int ret;
	struct tps549b22 *t = (struct tps549b22 *)file->f_inode->i_private;

	if (copy_from_user(buf + *ppos, userbuf, count))
		return -EFAULT;

	ret = kstrtouint(buf, 10, &voltage);
	if (ret) {
		pr_err("The input string isn't a num\n");
		return 0;
	}

	TPS549b22_INFO("The supply will be set to %duV\n", voltage);
	tps549b22dcdc_set_voltage(t->rdev[0],
				  voltage,
				  voltage + VOL_STEP,
				  NULL);
	TPS549b22_INFO("Now the voltage is %duV\n",
		       tps549b22dcdc_get_voltage(t->rdev[0]));

	return count;
}

static const struct file_operations tps549b22_fops = {
	.read = tps549b22_read,
	.write = tps549b22_write,
};

static void tps549b22_add_dbg_node(const char *name)
{
	static struct dentry *parent;

	if (!debugfs_rootdir)
		return;

	parent = debugfs_create_dir(name, debugfs_rootdir);
	if (parent)
		debugfs_create_file("voltage",
				    0664,
				    parent,
				    g_tps549b22,
				    &tps549b22_fops);
	else
		dev_warn(g_tps549b22->dev, "create dir %s failed\n", name);
}

static void tps549b22_del_debugfs(void)
{
	debugfs_remove_recursive(debugfs_rootdir);
}

static void tps549b22_add_debugfs(void)
{
	debugfs_rootdir = debugfs_create_dir("tps549b22", NULL);
	if (!debugfs_rootdir)
		pr_warn("create dir tps549b22 failed\n");
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id tps549b22_of_match[] = {
	{.compatible = "ti,tps549b22"},
	{ },
};
MODULE_DEVICE_TABLE(of, tps549b22_of_match);
#endif

static int tps549b22_i2c_probe(struct i2c_client *i2c,
			       const struct i2c_device_id *id)
{
	struct tps549b22 *tps549b22;
	struct tps549b22_board *pdev = NULL;
	const struct of_device_id *match;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct regulator_init_data *reg_data;
	const char *rail_name = NULL;
	int ret;
	u16 val;

	if (i2c->dev.of_node) {
		match = of_match_device(tps549b22_of_match, &i2c->dev);
		if (!match) {
			pr_err("Failed to find matching dt id\n");
			return -EINVAL;
		}
	}

	tps549b22 = devm_kzalloc(&i2c->dev,
				 sizeof(struct tps549b22),
				 GFP_KERNEL);
	if (!tps549b22) {
		ret = -ENOMEM;
		goto err;
	}

	tps549b22->i2c = i2c;
	tps549b22->dev = &i2c->dev;
	i2c_set_clientdata(i2c, tps549b22);
	g_tps549b22 = tps549b22;

	mutex_init(&tps549b22->io_lock);

	ret = tps549b22_reg_read(tps549b22, TPS549b22_REG_MFR_SPC_44, 2, &val);
	if (ret >= 0) {
		if (val != TPS549b22_ID)
			dev_warn(tps549b22->dev,
				 "The device is not tps549b22 0x%x\n",
				 val);
	} else {
		dev_err(tps549b22->dev,
			"Tps549b22_reg_read err, ret = %d\n",
			ret);
		return -EINVAL;
	}

	tps549b22_reg_init(tps549b22);

	if (tps549b22->dev->of_node)
		pdev = tps549b22_parse_dt(tps549b22);

	if (pdev) {
		tps549b22->num_regulators = TPS549b22_NUM_REGULATORS;
		tps549b22->rdev =
			devm_kmalloc_array(tps549b22->dev,
					   TPS549b22_NUM_REGULATORS,
					   sizeof(struct regulator_dev *),
					   GFP_KERNEL);
		if (!tps549b22->rdev)
			return -ENOMEM;

		/* Instantiate the regulators */
		reg_data = pdev->tps549b22_init_data[0];
		config.dev = tps549b22->dev;
		config.driver_data = tps549b22;
		if (tps549b22->dev->of_node)
			config.of_node = pdev->of_node[0];

		if (reg_data->constraints.name)
			rail_name = reg_data->constraints.name;
		else
			rail_name = regulators[0].name;

		reg_data->supply_regulator = rail_name;

		config.init_data = reg_data;
		rdev = regulator_register(&regulators[0], &config);
		if (IS_ERR(rdev)) {
			pr_err("failed to register regulator\n");
			goto err;
		}

		tps549b22->rdev[0] = rdev;
#if TPS549b22_DEBUG
		tps549b22_add_dbg_node(rail_name);
#endif
	}

	return 0;
err:
	return ret;
}

static int tps549b22_i2c_remove(struct i2c_client *i2c)
{
	struct tps549b22 *tps549b22 = i2c_get_clientdata(i2c);

#if TPS549b22_DEBUG
	tps549b22_del_debugfs();
#endif

	if (tps549b22->rdev[0])
		regulator_unregister(tps549b22->rdev[0]);

	i2c_set_clientdata(i2c, NULL);

	return 0;
}

static const struct i2c_device_id tps549b22_i2c_id[] = {
	{"tps549b22", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, tps549b22_i2c_id);

static struct i2c_driver tps549b22_i2c_driver = {
	.driver = {
		.name = "tps549b22",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tps549b22_of_match),
	},
	.probe = tps549b22_i2c_probe,
	.remove = tps549b22_i2c_remove,
	.id_table = tps549b22_i2c_id,
};

static int __init tps549b22_module_init(void)
{
	int ret;

#if TPS549b22_DEBUG
	tps549b22_add_debugfs();
#endif

	ret = i2c_add_driver(&tps549b22_i2c_driver);

	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);

	return ret;
}
subsys_initcall_sync(tps549b22_module_init);

static void __exit tps549b22_module_exit(void)
{
	i2c_del_driver(&tps549b22_i2c_driver);
}
module_exit(tps549b22_module_exit);

MODULE_LICENSE("GPL");

MODULE_AUTHOR("derrick.huang@rock-chips.com");
MODULE_DESCRIPTION("   tps549b22 dcdc driver");
