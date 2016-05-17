/*
 * Regulator driver for rk816 PMIC chip for rk31xx
 *
 * Based on rk816.c that is work by zhangqing<zhangqing@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/rk816.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>
#include <linux/syscore_ops.h>
#include <linux/irq.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>

#if 0
#define RK816_DBG(x...)	printk(KERN_INFO x)
#else
#define RK816_DBG(x...)
#endif

struct rk816 *g_rk816;

static const struct regmap_config rk816_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = DATA18_REG,
	.cache_type = REGCACHE_RBTREE,
};

static struct mfd_cell rk816s[] = {
	{
		.name = "rk816-regulator",
	},
	{
		.name = "rk816-rtc",
	},
	{
		.name = "rk816-pwrkey",
	},
	{
		.name = "rk816-battery",
	},
	{
		.name = "rk816-gpio",
	},
};

int rk816_i2c_read(struct rk816 *rk816, char reg, int count, u8 *dest)
{
	struct i2c_client *i2c = rk816->i2c;
	int ret;
	struct i2c_adapter *adap;
	struct i2c_msg msgs[2];

	if (!i2c)
		return ret;

	if (count != 1)
		return -EIO;

	adap = i2c->adapter;

	msgs[0].addr = i2c->addr;
	msgs[0].buf = &reg;
	msgs[0].flags = i2c->flags;
	msgs[0].len = 1;
	msgs[0].scl_rate = RK816_I2C_SPEED;

	msgs[1].buf = (u8 *)dest;
	msgs[1].addr = i2c->addr;
	msgs[1].flags =  i2c->flags | I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].scl_rate = RK816_I2C_SPEED;

	ret = i2c_transfer(adap, msgs, 2);

	RK816_DBG("***run in %s %x  %x\n", __func__, i2c->addr, *(msgs[1].buf));
	return ret;
}

int rk816_i2c_write(struct rk816 *rk816, char reg, int count,  const u8 src)
{
	int ret = -1;
	struct i2c_client *i2c = rk816->i2c;
	struct i2c_adapter *adap;
	struct i2c_msg msg;
	char tx_buf[2];

	if (!i2c)
		return ret;
	if (count != 1)
		return -EIO;

	adap = i2c->adapter;
	tx_buf[0] = reg;
	tx_buf[1] = src;

	msg.addr = i2c->addr;
	msg.buf = &tx_buf[0];
	msg.len = 1 + 1;
	msg.flags = i2c->flags;
	msg.scl_rate = RK816_I2C_SPEED;

	ret = i2c_transfer(adap, &msg, 1);
	return ret;
}

int rk816_reg_read(struct rk816 *rk816, u8 reg)
{
	u8 val = 0;
	int ret;

	mutex_lock(&rk816->io_lock);

	ret = rk816_i2c_read(rk816, reg, 1, &val);
	RK816_DBG("reg read 0x%02x -> 0x%02x\n",
		  (int)reg, (unsigned)val & 0xff);
	if (ret < 0) {
		mutex_unlock(&rk816->io_lock);
		return ret;
	}
	mutex_unlock(&rk816->io_lock);

	return val & 0xff;
}
EXPORT_SYMBOL_GPL(rk816_reg_read);

int rk816_reg_write(struct rk816 *rk816, u8 reg, u8 val)
{
	int err = 0;

	mutex_lock(&rk816->io_lock);

	err = rk816_i2c_write(rk816, reg, 1, val);
	if (err < 0)
		dev_err(rk816->dev, "Write for reg 0x%x failed\n", reg);

	mutex_unlock(&rk816->io_lock);
	return err;
}
EXPORT_SYMBOL_GPL(rk816_reg_write);

int rk816_set_bits(struct rk816 *rk816, u8 reg, u8 mask, u8 val)
{
	u8 tmp;
	int ret = 0;

	mutex_lock(&rk816->io_lock);

	ret = rk816_i2c_read(rk816, reg, 1, &tmp);
	RK816_DBG("1 reg read 0x%02x -> 0x%02x\n",
		  (int)reg, (unsigned)tmp & 0xff);
	if (ret < 0) {
		mutex_unlock(&rk816->io_lock);
		return ret;
	}
	tmp = (tmp & ~mask) | val;
	ret = rk816_i2c_write(rk816, reg, 1, tmp);
	RK816_DBG("reg write 0x%02x -> 0x%02x\n",
		  (int)reg, (unsigned)val & 0xff);
	if (ret < 0) {
		mutex_unlock(&rk816->io_lock);
		return ret;
	}
	ret = rk816_i2c_read(rk816, reg, 1, &tmp);
	if (ret < 0) {
		mutex_unlock(&rk816->io_lock);
		return ret;
	}
	RK816_DBG("2 reg read 0x%02x -> 0x%02x\n",
		  (int)reg, (unsigned)tmp & 0xff);
	mutex_unlock(&rk816->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(rk816_set_bits);

int rk816_clear_bits(struct rk816 *rk816, u8 reg, u8 mask)
{
	u8 data;
	int err;

	mutex_lock(&rk816->io_lock);
	err = rk816_i2c_read(rk816, reg, 1, &data);
	if (err < 0) {
		dev_err(rk816->dev, "read from reg %x failed\n", reg);
		goto out;
	}

	data &= ~mask;
	err = rk816_i2c_write(rk816, reg, 1, data);
	if (err < 0)
		dev_err(rk816->dev, "write to reg %x failed\n", reg);

out:
	mutex_unlock(&rk816->io_lock);
	return err;
}
EXPORT_SYMBOL_GPL(rk816_clear_bits);
/*
int rk816_bulk_read(struct rk816 *rk816, u8 reg,
		     int count, u8 *buf)
{
	int ret;

	mutex_lock(&rk816->io_lock);
	ret = rk816_i2c_read(rk816, reg, count, buf);
	mutex_unlock(&rk816->io_lock);

	return ret;

}
EXPORT_SYMBOL_GPL(rk816_bulk_read);

int rk816_bulk_write(struct rk816 *rk816, u8 reg,
		     int count, u8 *buf)
{
	int ret;

	mutex_lock(&rk816->io_lock);
	ret = rk816_i2c_write(rk816, reg, count, buf);
	mutex_unlock(&rk816->io_lock);

	return ret;

}
EXPORT_SYMBOL_GPL(rk816_bulk_write);
*/

static ssize_t rk816_test_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	u32 getdata[8];
	u8 regAddr;
	u8 data;
	char cmd;
	const char *buftmp = buf;
	struct rk816 *rk816 = g_rk816;
	int ret;

	ret = sscanf(buftmp, "%c ", &cmd);
	pr_info("------zhangqing: get cmd = %c\n", cmd);
	switch (cmd) {
	case 'w':
		ret = sscanf(buftmp, "%c %x %x ", &cmd, &getdata[0],
			     &getdata[1]);
		regAddr = (u8)(getdata[0] & 0xff);
		data = (u8)(getdata[1] & 0xff);
		pr_info("get value = %x\n", data);

		rk816_i2c_write(rk816, regAddr, 1, data);
		rk816_i2c_read(rk816, regAddr, 1, &data);
		pr_info("%x   %x\n", getdata[1], data);
		break;
	case 'r':
		ret = sscanf(buftmp, "%c %x ", &cmd, &getdata[0]);
		pr_info("CMD : %c %x\n", cmd, getdata[0]);

		regAddr = (u8)(getdata[0] & 0xff);
		rk816_i2c_read(rk816, regAddr, 1, &data);
		pr_info("\n%x %x\n", getdata[0], data);
		break;
	default:
		pr_err("Unknown command\n");
		break;
	}
	return n;
}
static ssize_t rk816_test_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	buf = "hello";
	return sprintf(s, "%s\n", buf);
}

static struct kobject *rk816_kobj;
struct rk816_attribute {
	struct attribute	attr;

	ssize_t (*show)(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf);
	ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t n);
};

static struct rk816_attribute rk816_attrs[] = {
	/*node_name permision show_func	store_func */
	__ATTR(rk816_test, S_IRUGO | S_IWUSR,
	       rk816_test_show, rk816_test_store),
};

#ifdef CONFIG_OF
static struct of_device_id rk816_of_match[] = {
	{ .compatible = "rockchip,rk816" },
	{ },
};
MODULE_DEVICE_TABLE(of, rk816_of_match);
#endif

#ifdef CONFIG_OF
static struct rk816_board *rk816_parse_dt(struct rk816 *rk816)
{
	struct rk816_board *pdata;
	struct device_node *rk816_pmic_np;

	rk816_pmic_np = of_node_get(rk816->dev->of_node);
	if (!rk816_pmic_np) {
		pr_err("could not find pmic sub-node\n");
		return NULL;
	}

	pdata = devm_kzalloc(rk816->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->irq = rk816->chip_irq;
	pdata->irq_base = -1;

	pdata->irq_gpio = of_get_named_gpio(rk816_pmic_np, "gpios", 0);
	if (!gpio_is_valid(pdata->irq_gpio)) {
		pr_err("invalid gpio: %d\n",  pdata->irq_gpio);
		return NULL;
	}

	pdata->pmic_sleep_gpio = of_get_named_gpio(rk816_pmic_np, "gpios", 1);
	if (!gpio_is_valid(pdata->pmic_sleep_gpio))
		pr_err("invalid gpio: %d\n", pdata->pmic_sleep_gpio);
	pdata->pmic_sleep = true;

	pdata->pm_off = of_property_read_bool(rk816_pmic_np,
					      "rk816,system-power-controller");

	return pdata;
}

#else
static struct rk816_board *rk816_parse_dt(struct i2c_client *i2c)
{
	return NULL;
}
#endif
static void rk816_shutdown(void)
{
	int ret;
	struct rk816 *rk816 = g_rk816;

	pr_info("%s\n", __func__);

	ret = rk816_set_bits(rk816, RK816_INT_STS_MSK_REG2,
			     (0x3 << 5), (0x3 << 5));
	/*close rtc int when power off*/
	ret = rk816_clear_bits(rk816, RK816_INT_STS_REG1,
			       (0x3 << 5));
	mutex_lock(&rk816->io_lock);
	mdelay(100);
}

static struct syscore_ops rk816_syscore_ops = {
	.shutdown = rk816_shutdown,
};

static void rk816_device_shutdown(void)
{
	int ret, i;
	u8 reg = 0;
	struct rk816 *rk816 = g_rk816;

	for (i = 0; i < 10; i++) {
		pr_info("%s\n", __func__);
		ret = rk816_i2c_read(rk816, RK816_DEV_CTRL_REG,
				     1, &reg);
		if (ret < 0)
			continue;
		ret = rk816_i2c_write(rk816, RK816_DEV_CTRL_REG,
				      1, (reg | (0x1 << 0)));
		if (ret < 0) {
			pr_err("rk816 power off error!\n");
			continue;
		}
	}
	while (1)
		wfi();
}
EXPORT_SYMBOL_GPL(rk816_device_shutdown);

static const struct regmap_irq rk816_irqs[] = {
	/* INT_STS */
	[RK816_IRQ_PWRON_FALL] = {
		.mask = RK816_IRQ_PWRON_FALL_MSK,
		.reg_offset = 0,
	},
	[RK816_IRQ_PWRON_RISE] = {
		.mask = RK816_IRQ_PWRON_RISE_MSK,
		.reg_offset = 0,
	},
	[RK816_IRQ_VB_LOW] = {
		.mask = RK816_IRQ_VB_LOW_MSK,
		.reg_offset = 1,
	},
	[RK816_IRQ_PWRON] = {
		.mask = RK816_IRQ_PWRON_MSK,
		.reg_offset = 1,
	},
	[RK816_IRQ_PWRON_LP] = {
		.mask = RK816_IRQ_PWRON_LP_MSK,
		.reg_offset = 1,
	},
	[RK816_IRQ_HOTDIE] = {
		.mask = RK816_IRQ_HOTDIE_MSK,
		.reg_offset = 1,
	},
	[RK816_IRQ_RTC_ALARM] = {
		.mask = RK816_IRQ_RTC_ALARM_MSK,
		.reg_offset = 1,
	},
	[RK816_IRQ_RTC_PERIOD] = {
		.mask = RK816_IRQ_RTC_PERIOD_MSK,
		.reg_offset = 1,
	},
	[RK816_IRQ_USB_OV] = {
		.mask = RK816_IRQ_USB_OV_MSK,
		.reg_offset = 1,
	},
};

static struct regmap_irq_chip rk816_irq_chip = {
	.name = "rk816",
	.irqs = rk816_irqs,
	.num_irqs = ARRAY_SIZE(rk816_irqs),
	.num_regs = 2,
	.irq_reg_stride = 3,
	.status_base = RK816_INT_STS_REG1,
	.mask_base = RK816_INT_STS_MSK_REG1,
	.ack_base = RK816_INT_STS_REG1,
};

static const struct regmap_irq rk816_battery_irqs[] = {
	/* INT_STS */
	[RK816_IRQ_PLUG_IN] = {
		.mask = RK816_IRQ_PLUG_IN_MSK,
		.reg_offset = 0,
	},
	[RK816_IRQ_PLUG_OUT] = {
		.mask = RK816_IRQ_PLUG_OUT_MSK,
		.reg_offset = 0,
	},
	[RK816_IRQ_CHG_OK] = {
		.mask = RK816_IRQ_CHG_OK_MSK,
		.reg_offset = 0,
	},
	[RK816_IRQ_CHG_TE] = {
		.mask = RK816_IRQ_CHG_TE_MSK,
		.reg_offset = 0,
	},
	[RK816_IRQ_CHG_TS] = {
		.mask = RK816_IRQ_CHG_TS_MSK,
		.reg_offset = 0,
	},
	[RK816_IRQ_CHG_CVTLIM] = {
		.mask = RK816_IRQ_CHG_CVTLIM_MSK,
		.reg_offset = 0,
	},
	[RK816_IRQ_DISCHG_ILIM] = {
		.mask = RK816_IRQ_DISCHG_ILIM_MSK,
		.reg_offset = 0,
	},
};
static struct regmap_irq_chip rk816_battery_irq_chip = {
	.name = "rk816_battery",
	.irqs = rk816_battery_irqs,
	.num_irqs = ARRAY_SIZE(rk816_battery_irqs),
	.num_regs = 1,
	.status_base = RK816_INT_STS_REG3,
	.mask_base = RK816_INT_STS_MSK_REG3,
	.ack_base = RK816_INT_STS_REG3,
};

static int rk816_pre_init(struct rk816 *rk816)
{
	int ret, val;

	pr_info("pmic power on/off source: on=0x%x, off=0x%x\n",
		rk816_reg_read(rk816, RK816_ON_SOURCE_REG),
		rk816_reg_read(rk816, RK816_OFF_SOURCE_REG));

	/***********set buck 12.5mv/us ************/
	val = rk816_reg_read(rk816, RK816_BUCK1_CONFIG_REG);
	val &= (~(0x3 << 3));
	val |= (0x2 << 3);
	ret = rk816_reg_write(rk816, RK816_BUCK1_CONFIG_REG, val);
	if (ret < 0) {
		pr_err("Unable to write RK816_BUCK1_CONFIG_REG reg\n");
		return ret;
	}

	val = rk816_reg_read(rk816, RK816_BUCK2_CONFIG_REG);
	val &= (~(0x3 << 3));
	val |= (0x2 << 3);
	ret = rk816_reg_write(rk816, RK816_BUCK2_CONFIG_REG, val);
	if (ret < 0) {
		pr_err("Unable to write RK816_BUCK2_CONFIG_REG reg\n");
		return ret;
	}
	/*****************************************/

	/****************set vbat low **********/
	val = rk816_reg_read(rk816, RK816_VB_MON_REG);
	val &= (~(VBAT_LOW_VOL_MASK | VBAT_LOW_ACT_MASK));
	val |= (RK816_VBAT_LOW_3V0 | EN_VABT_LOW_SHUT_DOWN);
	ret = rk816_reg_write(rk816, RK816_VB_MON_REG, val);
	if (ret < 0) {
		pr_err("Unable to write RK816_VB_MON_REG reg\n");
		return ret;
	}
	/**************************************/

	ret = rk816_clear_bits(rk816, RK816_INT_STS_MSK_REG2, (0x3 << 5));
	ret = rk816_set_bits(rk816, RK816_INT_STS_REG2, (0x1<<3), (0x1 << 3));

	ret = rk816_reg_write(rk816, RK816_BUCK4_CONFIG_REG, 0x2c);
	ret = rk816_reg_write(rk816, RK816_CHRG_CTRL_REG1, 0x87);
	ret = rk816_reg_write(rk816, RK816_USB_CTRL_REG, 0xc7);
	ret = rk816_reg_write(rk816, RK816_THERMAL_REG, 0x08);

	ret = rk816_reg_write(rk816, RK816_INT_STS_MSK_REG1, 0x9f);
	ret = rk816_reg_write(rk816, RK816_INT_STS_REG1, 0xff);
	ret = rk816_reg_write(rk816, RK816_INT_STS_MSK_REG2, 0xdf);
	ret = rk816_reg_write(rk816, RK816_INT_STS_MSK_REG3, 0xfc);

	return ret;
}

static int rk816_irq_init(struct rk816 *rk816)
{
	int ret = 0;

	if (gpio_is_valid(rk816->irq_gpio)) {
		rk816->chip_irq = gpio_to_irq(rk816->irq_gpio);
		ret = gpio_request(rk816->irq_gpio, "rk816_pmic_irq");
		if (ret < 0) {
			dev_err(rk816->dev,
				"Failed request gpio %d with ret=%d\n",
				rk816->irq_gpio, ret);
			return ret;
		}
		gpio_direction_input(rk816->irq_gpio);
		gpio_free(rk816->irq_gpio);
	}

	ret = regmap_add_irq_chip(rk816->regmap, rk816->chip_irq,
				  IRQF_TRIGGER_FALLING | IRQF_ONESHOT |
				  IRQF_SHARED, -1,
				  &rk816_irq_chip, &rk816->irq_data);

	if (ret < 0) {
		dev_err(rk816->dev, "Failed to add irq_chip %d\n", ret);
		return ret;
	}

	ret = regmap_add_irq_chip(rk816->regmap, rk816->chip_irq,
				  IRQF_TRIGGER_FALLING | IRQF_ONESHOT |
				  IRQF_SHARED, -1,
				  &rk816_battery_irq_chip,
				  &rk816->battery_irq_data);

	if (ret < 0) {
		dev_err(rk816->dev, "Failed to add battery_irq_chip %d\n", ret);
		regmap_del_irq_chip(rk816->chip_irq, rk816->irq_data);
		return ret;
	}

	irq_set_irq_type(rk816->chip_irq, IRQ_TYPE_LEVEL_LOW);
	enable_irq_wake(rk816->chip_irq);

	return ret;
}

static int rk816_i2c_probe(struct i2c_client *i2c,
			   const struct i2c_device_id *id)
{
	struct rk816 *rk816;
	struct rk816_board *pdev;
	const struct of_device_id *match;
	int ret, i;
	u16 chip_version = 0;

	pr_info("%s,line=%d\n", __func__, __LINE__);

	if (i2c->dev.of_node) {
		match = of_match_device(rk816_of_match, &i2c->dev);
		if (!match) {
			dev_err(&i2c->dev, "Failed to find matching dt id\n");
			return -EINVAL;
		}
	}
	rk816 = devm_kzalloc(&i2c->dev, sizeof(struct rk816), GFP_KERNEL);
	if (rk816 == NULL)
		return -ENOMEM;

	rk816->i2c = i2c;
	rk816->dev = &i2c->dev;
	i2c_set_clientdata(i2c, rk816);
	printk("pmic: addr=%p\n", (void *)rk816);

	rk816->regmap = devm_regmap_init_i2c(i2c, &rk816_regmap_config);
	if (IS_ERR(rk816->regmap)) {
		dev_err(&i2c->dev, "regmap initialization failed\n");
		return PTR_ERR(rk816->regmap);
	}

	mutex_init(&rk816->io_lock);

	ret = rk816_reg_read(rk816, RK816_CHIP_NAME_REG);
	chip_version |= ret << 8;
	ret = rk816_reg_read(rk816, RK816_CHIP_VER_REG);
	chip_version |= ret;

	if ((chip_version & 0x8160) == 0x8160) {
		pr_info("The device is rk816.The chip version is%x\n",
			chip_version);
	} else {
		pr_err("The device is not rk816.The chip is %x.error = %d\n",
		       chip_version, ret);
		return -EINVAL;
	}

	ret = rk816_pre_init(rk816);
	if (ret < 0) {
		pr_err("The rk816_pre_init failed %d\n", ret);
		return ret;
	}

	if (rk816->dev->of_node)
		pdev = rk816_parse_dt(rk816);

	/***set sleep vol & dcdc mode***/
	#ifdef CONFIG_OF
	rk816->pmic_sleep_gpio = pdev->pmic_sleep_gpio;
	if (gpio_is_valid(rk816->pmic_sleep_gpio)) {
		ret = gpio_request(rk816->pmic_sleep_gpio,
				   "rk816_pmic_sleep");
		if (ret < 0) {
			dev_err(rk816->dev, "Failed req gpio %d with ret=%d\n",
				rk816->pmic_sleep_gpio, ret);
			return ret;
		}
		gpio_direction_output(rk816->pmic_sleep_gpio, 0);
		ret = gpio_get_value(rk816->pmic_sleep_gpio);
		gpio_free(rk816->pmic_sleep_gpio);
		pr_info("%s: rk816_pmic_sleep=%x\n", __func__, ret);
	}
	#endif
	/**********************************************************/

	/*****************rk816 init irq******************************/
	rk816->irq_gpio = pdev->irq_gpio;
	ret = rk816_irq_init(rk816);
	/**********************************************************/

	ret = mfd_add_devices(rk816->dev, -1,
			      rk816s, ARRAY_SIZE(rk816s),
			      NULL, 0, NULL);

	if (ret) {
		dev_err(&i2c->dev, "failed to add MFD devices %d\n", ret);
		goto err_irq;
	}

	g_rk816 = rk816;

	if (pdev->pm_off && !pm_power_off)
		pm_power_off = rk816_device_shutdown;

	rk816_kobj = kobject_create_and_add("rk816", NULL);
	if (!rk816_kobj) {
		ret = -ENOMEM;
		goto err_irq;
	}

	for (i = 0; i < ARRAY_SIZE(rk816_attrs); i++) {
		ret = sysfs_create_file(rk816_kobj, &rk816_attrs[i].attr);
		if (ret != 0) {
			pr_err("create index %d error\n", i);
			goto err_irq;
		}
	}

	register_syscore_ops(&rk816_syscore_ops);

	return 0;

err_irq:
	regmap_del_irq_chip(rk816->chip_irq, rk816->irq_data);
	regmap_del_irq_chip(rk816->chip_irq, rk816->battery_irq_data);
	return ret;
}

static int rk816_i2c_remove(struct i2c_client *i2c)
{
	struct rk816 *rk816 = i2c_get_clientdata(i2c);

	unregister_syscore_ops(&rk816_syscore_ops);
	regmap_del_irq_chip(rk816->chip_irq, rk816->irq_data);
	regmap_del_irq_chip(rk816->chip_irq, rk816->battery_irq_data);
	mfd_remove_devices(&i2c->dev);
	pm_power_off = NULL;
	i2c_set_clientdata(i2c, NULL);

	return 0;
}

static const struct i2c_device_id rk816_i2c_id[] = {
	{"rk816", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, rk816_i2c_id);

#ifdef CONFIG_PM
static int rk816_suspend(struct i2c_client *i2c, pm_message_t mesg)
{
	int ret, val;
	struct rk816 *rk816 = g_rk816;

	/************set vbat low 3v4 to irq**********/
	val = rk816_reg_read(rk816, RK816_VB_MON_REG);
	val &= (~(VBAT_LOW_VOL_MASK | VBAT_LOW_ACT_MASK));
	val |= (RK816_VBAT_LOW_3V4 | EN_VBAT_LOW_IRQ);
	ret = rk816_reg_write(rk816, RK816_VB_MON_REG, val);
	if (ret < 0) {
		pr_err("Unable to write RK818_VB_MON_REG reg\n");
		return ret;
	}
	/*enable low irq*/
	rk816_set_bits(rk816, RK816_INT_STS_MSK_REG2, (0x1 << 1), (0x0 << 1));
	return 0;
}

static int rk816_resume(struct i2c_client *i2c)
{
	int ret, val;
	struct rk816 *rk816 = g_rk816;

	/********set vbat low 3v0 to shutdown**********/
	val = rk816_reg_read(rk816, RK816_VB_MON_REG);
	val &= (~(VBAT_LOW_VOL_MASK | VBAT_LOW_ACT_MASK));
	val |= (RK816_VBAT_LOW_3V0 | EN_VABT_LOW_SHUT_DOWN);
	ret = rk816_reg_write(rk816, RK816_VB_MON_REG, val);
	if (ret < 0) {
		pr_err("Unable to write RK818_VB_MON_REG reg\n");
		return ret;
	}
	/*disable low irq*/
	rk816_set_bits(rk816, RK816_INT_STS_MSK_REG2, (0x1 << 1), (0x1 << 1));

	return 0;
}
#else
static int rk816_suspend(struct i2c_client *i2c, pm_message_t mesg)
{
	return 0;
}

static int rk816_resume(struct i2c_client *i2c)
{
	return 0;
}
#endif

static struct i2c_driver rk816_i2c_driver = {
	.driver = {
		.name = "rk816",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rk816_of_match),
	},
	.probe    = rk816_i2c_probe,
	.remove   = rk816_i2c_remove,
	#ifdef CONFIG_PM
	.suspend	= rk816_suspend,
	.resume		= rk816_resume,
	#endif

	.id_table = rk816_i2c_id,
};

static int __init rk816_module_init(void)
{
	int ret;

	ret = i2c_add_driver(&rk816_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);
	return ret;
}
subsys_initcall_sync(rk816_module_init);

static void __exit rk816_module_exit(void)
{
	i2c_del_driver(&rk816_i2c_driver);
}
module_exit(rk816_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhangqing <zhangqing@rock-chips.com>");
MODULE_DESCRIPTION("rk816 PMIC driver");

