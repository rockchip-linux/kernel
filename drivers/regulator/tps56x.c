/*
 * Regulator driver for tps56x DCDC chip for rk32xx
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
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>

#define TPS56X

#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif
#if 1
#define DBG_INFO(x...)	printk(KERN_INFO x)
#else
#define DBG_INFO(x...)
#endif

/* EN_PIN control the enable of this Buck, so clr the enable bit of register */
#define EN_PIN_CONTROL 1

#define TPS56X_SPEED 200*1000
#define tps56x_NUM_REGULATORS 1

struct tps56x_board {
	int irq;
	int irq_base;
	struct regulator_init_data *tps56x_init_data[tps56x_NUM_REGULATORS];
	struct device_node *of_node[tps56x_NUM_REGULATORS];
	int sleep_gpio; /* */
	unsigned int dcdc_slp_voltage[3]; /* buckx_voltage in uV */
	bool sleep;
};

struct tps56x {
	struct device *dev;
	struct mutex io_lock;
	struct i2c_client *i2c;
	int num_regulators;
	struct regulator_dev **rdev;
	int irq_base;
	int chip_irq;
	int sleep_gpio; /* */
	unsigned int dcdc_slp_voltage[3]; /* buckx_voltage in uV */
	bool pmic_sleep;
	struct regmap *regmap;
};

/*
struct tps56x_regulator_subdev {
	int id;
	struct regulator_init_data *initdata;
	struct device_node *reg_node;
};

struct tps56x_platform_data {
	int ono;
	int num_regulators;
	struct tps56x_regulator_subdev *regulators;
	
	int sleep_gpio; 
	unsigned int dcdc_slp_voltage[3]; // buckx_voltage in uV
	bool sleep;
};
struct tps56x *g_tps56x;

struct tps56x_regulator {
	struct device		*dev;
	struct regulator_desc	*desc;
	struct regulator_dev	*rdev;
};
*/
static int tps56x_i2c_read(struct i2c_client *i2c, char reg, int count,	u16 *dest)
{
	int ret;

	if(!i2c)
		return ret;

	if (count != 1)
		return -EIO;  

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0)
		return ret;
	*dest = ret;

	DBG("***run in %s %d reg %x = %x\n",__FUNCTION__,__LINE__,reg, ret);

	return 0;
}

static int tps56x_i2c_write(struct i2c_client *i2c, char reg, int count, const u16 src)
{
	int ret=-1;

	if(!i2c)
		return ret;
	if (count != 1)
		return -EIO;

	ret = i2c_smbus_write_byte_data(i2c, reg, src);
	if (ret < 0)
		return ret;

	return ret;
}

static int tps56x_reg_read(struct tps56x *tps56x, u8 reg)
{
	u16 val = 0;
	int ret;

	mutex_lock(&tps56x->io_lock);

	ret = tps56x_i2c_read(tps56x->i2c, reg, 1, &val);

	DBG("reg read 0x%02x -> 0x%02x\n", (int)reg, (unsigned)val&0xff);
	if (ret < 0){
		mutex_unlock(&tps56x->io_lock);
		return ret;
	}
	mutex_unlock(&tps56x->io_lock);

	return val & 0xff;	
}

static int tps56x_set_bits(struct tps56x *tps56x, u8 reg, u16 mask, u16 val)
{
	u16 tmp;
	int ret;


	mutex_lock(&tps56x->io_lock);


	ret = tps56x_i2c_read(tps56x->i2c, reg, 1, &tmp);
	DBG("1 reg read 0x%02x -> 0x%02x\n", (int)reg, (unsigned)tmp&0xff);
	if (ret < 0){
                mutex_unlock(&tps56x->io_lock);
                return ret;
        }

	
	tmp = (tmp & ~mask) | val;
	ret = tps56x_i2c_write(tps56x->i2c, reg, 1, tmp);
	DBG("reg write 0x%02x -> 0x%02x\n", (int)reg, (unsigned)val&0xff);
	if (ret < 0){
                mutex_unlock(&tps56x->io_lock);
                return ret;
        }

	
	ret = tps56x_i2c_read(tps56x->i2c, reg, 1, &tmp);
	DBG("2 reg read 0x%02x -> 0x%02x\n", (int)reg, (unsigned)tmp&0xff);
	if (ret < 0){
                mutex_unlock(&tps56x->io_lock);
                return ret;
        }
	mutex_unlock(&tps56x->io_lock);

	return 0;
}



#define TPS56X_BUCK1_SET_VOL_BASE 0x00
#define TPS56X_BUCK1_SLP_VOL_BASE 0x01
#define TPS56X_CONTR_REG1 0x02
#define TPS56X_ID1_REG 0x03
#define TPS56X_ID2_REG 0x04
#define TPS56X_CONTR_REG2 0x05

#define BUCK_VOL_MASK 0x3f

#ifdef TPS56X
#define TPS56X_REG_VOUT 	0
#define TPS56X_REG_CTR_A 	8
#define TPS56X_REG_CTR_B 	9
#define TPS56X_REG_STATUS 	24

#define TPS56X_FAST_MODE 	2
#define TPS56X_NORMAL_MODE 	3

#define VOL_MIN_IDX 0x00
#define VOL_MAX_IDX 0x7f

static int buck_voltage_map[VOL_MAX_IDX + 1] = { 0, };
#else
#define VOL_MIN_IDX 0x00
#define VOL_MAX_IDX 0x3f

const static int buck_voltage_map[] = {
	 712500, 725000, 737500,750000, 762500,775000,787500,800000,
	 812500, 825000, 837500,850000, 862500,875000,887500,900000,
	 912500, 925000, 937500,950000, 962500,975000,987500,1000000,
	 1012500, 1025000, 1037500,1050000, 1062500,1075000,1087500,1100000,
	 1112500, 1125000, 1137500,1150000, 1162500,1175000,1187500,1200000,
	 1212500, 1225000, 1237500,1250000, 1262500,1275000,1287500,1300000,
	 1312500, 1325000, 1337500,1350000, 1362500,1375000,1387500,1400000,
	 1412500, 1425000, 1437500,1450000, 1462500,1475000,1487500,1500000,
};
#endif

static int g_ntps56x_dcdc_list_voltage = 0;

static int tps56x_dcdc_list_voltage(struct regulator_dev *dev, unsigned index)
{

	if (g_ntps56x_dcdc_list_voltage % 20 == 0)
	{
		g_ntps56x_dcdc_list_voltage++;
	}

	if (index >= ARRAY_SIZE(buck_voltage_map))
		return -EINVAL;
	return  buck_voltage_map[index];
}
static int tps56x_dcdc_is_enabled(struct regulator_dev *dev)
{
	struct tps56x *tps56x = rdev_get_drvdata(dev);
	u16 val;
	u16 mask=0x80;	


#ifdef TPS56X
	#ifdef EN_PIN_CONTROL
	return 1;
	#endif
	val = tps56x_reg_read(tps56x, TPS56X_REG_CTR_B);
	if (val < 0)
		return val;
 	val = val & mask;
#else
	val = tps56x_reg_read(tps56x, TPS56X_BUCK1_SET_VOL_BASE);
	if (val < 0)
		return val;
	 val=val&~0x7f;
#endif

	if (val & mask)
	{
		return 1;
	}
	else
	{
		return 0; 	
	}
}
static int tps56x_dcdc_enable(struct regulator_dev *dev)
{
	struct tps56x *tps56x = rdev_get_drvdata(dev);
	u16 mask=0x80;	

#ifdef TPS56X
	#ifdef EN_PIN_CONTROL
	return tps56x_set_bits(tps56x, TPS56X_REG_CTR_B, mask, 0);
	#endif
	return tps56x_set_bits(tps56x, TPS56X_REG_CTR_B, mask, 0x80);
#else
	return tps56x_set_bits(tps56x, TPS56X_BUCK1_SET_VOL_BASE, mask, 0x80);
#endif
}
static int tps56x_dcdc_disable(struct regulator_dev *dev)
{
	struct tps56x *tps56x = rdev_get_drvdata(dev);
	u16 mask=0x80;

#ifdef TPS56X
	return tps56x_set_bits(tps56x, TPS56X_REG_CTR_B, mask, 0);
#else
	return tps56x_set_bits(tps56x, TPS56X_BUCK1_SET_VOL_BASE, mask, 0);
#endif
}
static int tps56x_dcdc_get_voltage(struct regulator_dev *dev)
{
	struct tps56x *tps56x = rdev_get_drvdata(dev);
	u16 reg = 0;
	int val;


#ifdef TPS56X
	reg = tps56x_reg_read(tps56x, TPS56X_REG_VOUT);
	reg &= 0x7F;
	val = buck_voltage_map[reg];	
#else
	reg = tps56x_reg_read(tps56x,TPS56X_BUCK1_SET_VOL_BASE);
	reg &= BUCK_VOL_MASK;
	val = buck_voltage_map[reg];	
#endif
	
	return val;
}
static int tps56x_dcdc_set_voltage(struct regulator_dev *dev,
				  int min_uV, int max_uV,unsigned *selector)
{
	struct tps56x *tps56x = rdev_get_drvdata(dev);
	const int *vol_map = buck_voltage_map;
	u16 val, reg_val = 0;
	int n, odd_num = 0, ret = 0;

	
	if (min_uV < vol_map[VOL_MIN_IDX] ||
	    min_uV > vol_map[VOL_MAX_IDX])
		return -EINVAL;

	for (val = VOL_MIN_IDX; val <= VOL_MAX_IDX; val++){
		if (vol_map[val] >= min_uV)
			break;
        }

	if (vol_map[val] > max_uV)
		printk("WARNING:this voltage is not support!voltage set is %d mv\n",vol_map[val]);

#ifdef TPS56X
	reg_val |= val;
	for (n = 0; n < 7; n++)
		if (((val >> n) & 0x01) > 0) odd_num++;
	if ((odd_num % 2) == 0) reg_val |= 0x80;

	//reg_val = 0x80 | 40;
	ret = tps56x_set_bits(tps56x, TPS56X_REG_VOUT, 0xFF, reg_val);

	*selector = val;

#else
	ret = tps56x_set_bits(tps56x, TPS56X_BUCK1_SET_VOL_BASE ,BUCK_VOL_MASK, val);
#endif
	if(ret < 0)
		printk("###################WARNING:set voltage is error!voltage set is %d mv %d\n",vol_map[val],ret);

	
	return ret;
}

static unsigned int tps56x_dcdc_get_mode(struct regulator_dev *dev)
{
	struct tps56x *tps56x = rdev_get_drvdata(dev);
	u16 mask = 0x40;
	u16 val;
	

#ifdef TPS56X
	val = tps56x_reg_read(tps56x, TPS56X_REG_CTR_A);
	if (val < 0) {
			return val;
	}
	val = val & 0x03;
	if (val == TPS56X_NORMAL_MODE)
		return REGULATOR_MODE_NORMAL;
	else
		return REGULATOR_MODE_FAST;
#else
	val = tps56x_reg_read(tps56x, TPS56X_BUCK1_SET_VOL_BASE);
        if (val < 0) {
                return val;
        }
	val=val & mask;
	if (val== mask)
		return REGULATOR_MODE_FAST;
	else
		return REGULATOR_MODE_NORMAL;
#endif
}
static int tps56x_dcdc_set_mode(struct regulator_dev *dev, unsigned int mode)
{
	struct tps56x *tps56x = rdev_get_drvdata(dev);
	u16 mask = 0x40;


#ifdef TPS56X
	switch(mode)
	{
	case REGULATOR_MODE_FAST:
		return tps56x_set_bits(tps56x, TPS56X_REG_CTR_A, 0x03, TPS56X_FAST_MODE);
	case REGULATOR_MODE_NORMAL:
		return tps56x_set_bits(tps56x, TPS56X_REG_CTR_A, 0x03, TPS56X_NORMAL_MODE);
	default:
		printk("error:dcdc_tps56x only auto and pwm mode\n");
		return -EINVAL;
	}
#else
	switch(mode)
	{
	case REGULATOR_MODE_FAST:
		return tps56x_set_bits(tps56x,TPS56X_BUCK1_SET_VOL_BASE, mask, mask);
	case REGULATOR_MODE_NORMAL:
		return tps56x_set_bits(tps56x, TPS56X_BUCK1_SET_VOL_BASE, mask, 0);
	default:
		printk("error:dcdc_tps56x only auto and pwm mode\n");
		return -EINVAL;
	}
#endif
}
static int tps56x_dcdc_set_voltage_time_sel(struct regulator_dev *dev,   unsigned int old_selector,
				     unsigned int new_selector)
{
	int old_volt, new_volt;

	
	old_volt = tps56x_dcdc_list_voltage(dev, old_selector);
	if (old_volt < 0)
		return old_volt;
	
	new_volt = tps56x_dcdc_list_voltage(dev, new_selector);
	if (new_volt < 0)
		return new_volt;

	
	return DIV_ROUND_UP(abs(old_volt - new_volt)*4, 10000);
}
static int tps56x_dcdc_suspend_enable(struct regulator_dev *dev)
{
	struct tps56x *tps56x = rdev_get_drvdata(dev);
	u16 mask=0x80;	

#ifdef TPS56X
	return 0;
#else
	return tps56x_set_bits(tps56x, TPS56X_BUCK1_SLP_VOL_BASE, mask, 0x80);
#endif
}
static int tps56x_dcdc_suspend_disable(struct regulator_dev *dev)
{
	struct tps56x *tps56x = rdev_get_drvdata(dev);
	u16 mask=0x80;


#ifdef TPS56X
	return 0;
#else
	 return tps56x_set_bits(tps56x, TPS56X_BUCK1_SLP_VOL_BASE, mask, 0);
#endif
}
static int tps56x_dcdc_set_sleep_voltage(struct regulator_dev *dev,
					    int uV)
{
	struct tps56x *tps56x = rdev_get_drvdata(dev);
	const int *vol_map = buck_voltage_map;
	u16 val;
	int ret = 0;

	
	if (uV < vol_map[VOL_MIN_IDX] ||
	    uV > vol_map[VOL_MAX_IDX])
		return -EINVAL;

	for (val = VOL_MIN_IDX; val <= VOL_MAX_IDX; val++){
		if (vol_map[val] >= uV)
			break;
        }

	if (vol_map[val] > uV)
		printk("WARNING:this voltage is not support!voltage set is %d mv\n",vol_map[val]);

#ifdef TPS56X
	return 0;
#else	
	ret = tps56x_set_bits(tps56x,TPS56X_BUCK1_SLP_VOL_BASE ,BUCK_VOL_MASK, val);	
	return ret;
#endif
}


static int tps56x_dcdc_set_suspend_mode(struct regulator_dev *dev, unsigned int mode)
{
	struct tps56x *tps56x = rdev_get_drvdata(dev);
	u16 mask = 0x40;


#ifdef TPS56X
		return 0;
#else
	switch(mode)
	{
	case REGULATOR_MODE_FAST:
		return tps56x_set_bits(tps56x,TPS56X_BUCK1_SLP_VOL_BASE, mask, mask);
	case REGULATOR_MODE_NORMAL:
		return tps56x_set_bits(tps56x, TPS56X_BUCK1_SLP_VOL_BASE, mask, 0);
	default:
		printk("error:dcdc_tps56x only auto and pwm mode\n");
		return -EINVAL;
	}
#endif
}

static struct regulator_ops tps56x_dcdc_ops = { 
	.set_voltage = tps56x_dcdc_set_voltage,
	.get_voltage = tps56x_dcdc_get_voltage,
	.list_voltage= tps56x_dcdc_list_voltage,
	.is_enabled = tps56x_dcdc_is_enabled,
	.enable = tps56x_dcdc_enable,
	.disable = tps56x_dcdc_disable,
	.get_mode = tps56x_dcdc_get_mode,
	.set_mode = tps56x_dcdc_set_mode,
	.set_suspend_voltage = tps56x_dcdc_set_sleep_voltage,
	.set_suspend_enable = tps56x_dcdc_suspend_enable,
	.set_suspend_disable = tps56x_dcdc_suspend_disable,
	.set_suspend_mode = tps56x_dcdc_set_suspend_mode,
	.set_voltage_time_sel = tps56x_dcdc_set_voltage_time_sel,
};
static struct regulator_desc regulators[] = {

        {
		.name = "SY_DCDC1",
		.id = 0,
		.ops = &tps56x_dcdc_ops,
		.n_voltages = ARRAY_SIZE(buck_voltage_map),
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

#ifdef CONFIG_OF
static struct of_device_id tps56x_of_match[] = {
	{ .compatible = "ti,tps56x"},
	{ },
};
MODULE_DEVICE_TABLE(of, tps56x_of_match);
#endif
#ifdef CONFIG_OF
static struct of_regulator_match tps56x_reg_matches[] = {
	{ .name = "tps56x_dcdc1" ,.driver_data = (void *)0},
};

static struct tps56x_board *tps56x_parse_dt(struct tps56x *tps56x)
{
	struct tps56x_board *pdata;
	struct device_node *regs;
	struct device_node *tps56x_np;
	int count;
	DBG("%s,line=%d\n", __func__,__LINE__);	

	
	tps56x_np = of_node_get(tps56x->dev->of_node);
	if (!tps56x_np) {
		printk("could not find pmic sub-node\n");
		return NULL;
	}
	
	regs = of_find_node_by_name(tps56x_np, "regulators");
	if (!regs)
		return NULL;
	count = of_regulator_match(tps56x->dev, regs, tps56x_reg_matches,
				   tps56x_NUM_REGULATORS);
	of_node_put(regs);
	pdata = devm_kzalloc(tps56x->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;
	pdata->tps56x_init_data[0] = tps56x_reg_matches[0].init_data;
	pdata->of_node[0] = tps56x_reg_matches[0].of_node;
	
	return pdata;
}

#else
static struct tps56x_board *tps56x_parse_dt(struct i2c_client *i2c)
{

	return NULL;
}
#endif

static int tps56x_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct tps56x *tps56x;	
	struct tps56x_board *pdev ;
	const struct of_device_id *match;
	struct regulator_config config = { };
	struct regulator_dev *sy_rdev;
	struct regulator_init_data *reg_data;
	const char *rail_name = NULL;
	int ret;
	int n;
	
	
	DBG("%s,line=%d\n", __func__,__LINE__);	

#ifdef TPS56X
	for (n = VOL_MIN_IDX; n <= VOL_MAX_IDX; n++)
	{
		buck_voltage_map[n] = 600000 + (10000 * n);
	}
#endif

	if (i2c->dev.of_node) {
		match = of_match_device(tps56x_of_match, &i2c->dev);
		if (!match) {
			printk("Failed to find matching dt id\n");
			return -EINVAL;
		}
	}

	tps56x = devm_kzalloc(&i2c->dev,sizeof(struct tps56x), GFP_KERNEL);
	if (tps56x == NULL) {
		ret = -ENOMEM;		
		goto err;
	}
	tps56x->i2c = i2c;
	tps56x->dev = &i2c->dev;
	i2c_set_clientdata(i2c, tps56x);
	//g_tps56x = tps56x;
		
	mutex_init(&tps56x->io_lock);	

#ifdef TPS56X
#else
	ret = tps56x_reg_read(tps56x,TPS56X_ID1_REG);
	if ((ret <0) ||(ret ==0xff) ||(ret ==0)){
		printk("The device is not tps56x %x \n",ret);
		goto err;
	}
#endif

#ifdef TPS56X
	ret = tps56x_set_bits(tps56x, TPS56X_REG_CTR_A, 0x80, 0x80);				// internal mode.
	ret = tps56x_set_bits(tps56x, TPS56X_REG_CTR_A, 0x04, 0x04);	
	ret = tps56x_set_bits(tps56x, TPS56X_REG_CTR_A, 0x03, TPS56X_NORMAL_MODE);	
	ret = tps56x_set_bits(tps56x, TPS56X_REG_VOUT, 0xFF, (0x80 | 40));	
#else
	ret = tps56x_set_bits(tps56x,TPS56X_CONTR_REG1,(1 << 6),(1<<6));  //10mv/2.4us
#endif

	if (tps56x->dev->of_node)
		pdev = tps56x_parse_dt(tps56x);
	
	if (pdev) {
		tps56x->num_regulators = tps56x_NUM_REGULATORS;
		tps56x->rdev = kcalloc(tps56x_NUM_REGULATORS,sizeof(struct regulator_dev *), GFP_KERNEL);
		if (!tps56x->rdev) {
			return -ENOMEM;
		}
		/* Instantiate the regulators */
		reg_data = pdev->tps56x_init_data[0];
		config.dev = tps56x->dev;
		config.driver_data = tps56x;
		if (tps56x->dev->of_node)
			config.of_node = pdev->of_node[0];
			if (reg_data && reg_data->constraints.name)
				rail_name = reg_data->constraints.name;
			else
				rail_name = regulators[0].name;
			reg_data->supply_regulator = rail_name;

		config.init_data =reg_data;
		sy_rdev = regulator_register(&regulators[0],&config);
		if (IS_ERR(sy_rdev)) {
			printk("failed to register regulator\n");
		goto err;
		}
		tps56x->rdev[0] = sy_rdev;
	}
	return 0;
err:
	return ret;	

}

static int  tps56x_i2c_remove(struct i2c_client *i2c)
{
	struct tps56x *tps56x = i2c_get_clientdata(i2c);

	if (tps56x->rdev[0])
		regulator_unregister(tps56x->rdev[0]);
	i2c_set_clientdata(i2c, NULL);
	return 0;
}

static const struct i2c_device_id tps56x_i2c_id[] = {
       { "tps56x", 0 },
       { }
};

MODULE_DEVICE_TABLE(i2c, tps56x_i2c_id);

static struct i2c_driver tps56x_i2c_driver = {
	.driver = {
		.name = "tps56x",
		.owner = THIS_MODULE,
		.of_match_table =of_match_ptr(tps56x_of_match),
	},
	.probe    = tps56x_i2c_probe,
	.remove   = tps56x_i2c_remove,
	.id_table = tps56x_i2c_id,
};

static int __init tps56x_module_init(void)
{
	int ret;

	
	ret = i2c_add_driver(&tps56x_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);
	return ret;
}
subsys_initcall_sync(tps56x_module_init);

static void __exit tps56x_module_exit(void)
{
	i2c_del_driver(&tps56x_i2c_driver);
}
module_exit(tps56x_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("mts <mts@micro-tech.co.kr>");
MODULE_DESCRIPTION("tps56x PMIC driver");

