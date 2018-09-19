#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/power_supply.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define MP8859_MAX_VSEL			0x7ff
#define VOL_MIN_IDX			0x00
#define VOL_MAX_IDX			0x7ff
#define MP8859_ENABLE_TIME		150

/* Register definitions */
#define MP8859_VOUT_L_REG		0    //低3位
#define MP8859_VOUT_H_REG		1    //低8位
#define MP8859_VOUT_GO_REG		2
#define MP8859_IOUT_LIM_REG		3
#define MP8859_CTL1_REG			4
#define MP8859_CTL2_REG			5
#define MP8859_READ_VOUT_L		6
#define MP8859_READ_VOUT_H		7
#define MP8859_READ_IOUT		8
#define MP8859_STATUS_REG		9
#define MP8859_MAX_REG			10

#define MP8859_VOUT_MASK		0x7ff
#define MP8859_VBOOT_MASK		0x80

#define MP8859_GO_BIT			0x01  //GO_BIT 置1 更新电压变化
#define MP8859_SLEW_RATE_MASK		0x30
#define MP8859_SWITCH_FRE_MASK		0x0c
#define MP8859_PFM_MODE			0x10
#define MP8859_ENABLE			0x80
#define MP8859_GO_BIT_MASK		0x01
#define MP8859_VOUT_H_MASK		0xff
#define MP8859_VOUT_L_MASK		0x07

/* mp8859 chip information */
struct mp8859_chip {
	struct device *dev;
	struct regulator_desc desc;
	struct regulator_dev  *rdev;
	struct regmap *regmap;
	struct power_supply	*supply_charger;
	struct extcon_dev *ext_dev;
	struct notifier_block		cable_pd_nb;
	struct workqueue_struct		*pd_wq;
	struct delayed_work		pd_work;
	//struct gpio_desc *gpio_charge_en;
	u32 max_input_vol;
	u32 max_input_current;
};

struct mp8859_platform_data {
	struct regulator_init_data *mp8859_init_data;
	struct device_node *of_node;
};


static const struct regulator_linear_range mp8859_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(500000, VOL_MIN_IDX, VOL_MAX_IDX, 10000),
};

static int mp8859_set_voltage(struct regulator_dev *rdev, unsigned sel)
{
	int ret;
	//sel <<= ffs(rdev->desc->vsel_mask) - 1;

	ret = regmap_write(rdev->regmap,MP8859_VOUT_L_REG,sel & 0x7);

	if (ret)
		return ret;

	ret = regmap_write(rdev->regmap,MP8859_VOUT_H_REG,sel >> 3);

	if (ret)
		return ret;

	ret = regmap_update_bits(rdev->regmap, MP8859_VOUT_GO_REG,
				 MP8859_GO_BIT, MP8859_GO_BIT);
	if (ret)
		return ret;

	return ret;
	//return regmap_write(rdev->regmap, MP8859_VOUT_REG, sel);
}


static int mp8859_get_voltage_sel(struct regulator_dev *rdev)
{
	unsigned int val_tmp;
	unsigned int val;
	int ret;

	ret = regmap_read(rdev->regmap,MP8859_VOUT_H_REG, &val_tmp);
	if (ret != 0)
		return ret;

	val = val_tmp << 3 ;

	ret = regmap_read(rdev->regmap,MP8859_VOUT_L_REG, &val_tmp);
	if (ret != 0)
		return ret;

	val |= val_tmp & 0x07;

	return val;
}

static bool is_write_reg(struct device *dev, unsigned int reg)
{
	return (reg < MP8859_READ_VOUT_L) ? true : false;
}

static bool is_volatile_reg(struct device *dev, unsigned int reg)
{
	return (reg == MP8859_STATUS_REG) ? true : false;
}

static const struct regmap_config mp8859_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = is_write_reg,
	.volatile_reg = is_volatile_reg,
	.max_register = MP8859_MAX_REG,
	.cache_type = REGCACHE_RBTREE,
};

static struct regulator_ops mp8859_dcdc_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	//.get_voltage_sel = regulator_get_voltage_sel_regmap,//********
	.get_voltage_sel = mp8859_get_voltage_sel,
	.set_voltage_sel = mp8859_set_voltage,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct of_device_id mp8859_of_match[] = {
	{.compatible =  "mps,mp8859", .data = (void *)0},
	{},
};
MODULE_DEVICE_TABLE(of, mp8859_of_match);

static struct of_regulator_match mp8859_matches = {
	.name = "mp8859_dcdc1",
	.driver_data = (void *)0,
};

static struct mp8859_platform_data *
	mp8859_parse_dt(struct i2c_client *client,
			struct of_regulator_match **mp8859_reg_matches)
{
	struct mp8859_platform_data *pdata;
	struct device_node *np, *regulators;
	int ret;
	struct mp8859_chip *mp8859 = i2c_get_clientdata(client);

	pdata = devm_kzalloc(&client->dev, sizeof(struct mp8859_platform_data),
			     GFP_KERNEL);
	if (!pdata)
		return NULL;
	np = of_node_get(client->dev.of_node);
	regulators = of_find_node_by_name(np, "regulators");
	if (!regulators) {
		dev_err(&client->dev, "regulator node not found\n");
		return NULL;
	}

	ret = of_regulator_match(&client->dev, regulators, &mp8859_matches, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Error parsing regulators init data\n");
		return NULL;
	}
	*mp8859_reg_matches = &mp8859_matches;
	of_node_put(regulators);

	ret = of_property_read_u32(np, "mp,max-input-voltage",
				   &mp8859->max_input_vol);
	if (ret < 0) {
		dev_info(&client->dev, "Error parsing max-input-voltage data\n");
	}

	ret = of_property_read_u32(np, "mp,max-input-current",
				   &mp8859->max_input_current);
	if (ret < 0) {
		dev_info(&client->dev, "Error parsing max-input-current data\n");
	}

	pdata->mp8859_init_data = mp8859_matches.init_data;
	pdata->of_node = mp8859_matches.of_node;

	return pdata;
}


static enum power_supply_property mp8859_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int mp8859_power_supply_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct mp8859_chip *mp8859_chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "firefly";
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = true;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = mp8859_chip->max_input_vol;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = mp8859_chip->max_input_current;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static char *mp8859_charger_supplied_to[] = {
	"charger",
};

static const struct power_supply_desc mp8859_power_supply_desc = {
	.name = "mp8859-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = mp8859_power_supply_props,
	.num_properties = ARRAY_SIZE(mp8859_power_supply_props),
	.get_property = mp8859_power_supply_get_property,
};

static int mp8859_power_supply_init(struct mp8859_chip *charger)
{
	struct power_supply_config psy_cfg = { .drv_data = charger, };

	psy_cfg.supplied_to = mp8859_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(mp8859_charger_supplied_to);

	charger->supply_charger =
		power_supply_register(charger->dev,
				      &mp8859_power_supply_desc,
				      &psy_cfg);

	return PTR_ERR_OR_ZERO(charger->supply_charger);
}

static int mp8859_init(struct mp8859_chip *mp8859)
{
	int ret;

	/* set slew_rate 300us */
	ret = regmap_update_bits(mp8859->regmap, MP8859_CTL2_REG,
				 MP8859_SLEW_RATE_MASK, 0x00);
	//ret = regmap_write(mp8859->regmap, MP8859_CTL2_REG, 0x40);

	if (ret) {
		dev_err(mp8859->dev, "failed to set slew_rate\n");
		return ret;
	}

	/* set switch_frequency 500Khz */
	ret = regmap_update_bits(mp8859->regmap, MP8859_CTL1_REG,
				 MP8859_SWITCH_FRE_MASK, 0x00);
	if (ret) {
		dev_err(mp8859->dev, "failed to set switch_frequency\n");
		return ret;
	}

	/* set vout_reg 12v*/
	ret = regmap_write(mp8859->regmap, MP8859_VOUT_H_REG, 0x99);
	if (ret) {
		dev_err(mp8859->dev, "failed to set VOUT_H_REG\n");
		return ret;
	}

	ret = regmap_write(mp8859->regmap, MP8859_VOUT_L_REG, 0x06);
	if (ret) {
		dev_err(mp8859->dev, "failed to set VOUT_L_REG\n");
		return ret;
	}

	ret = regmap_write(mp8859->regmap, MP8859_VOUT_GO_REG, 0x01);
	if (ret) {
		dev_err(mp8859->dev, "failed to set VOUT_GO_REG\n");
		return ret;
	}

	dev_info(mp8859->dev, "set mp8859 out 12v ok\n");
	return ret;
}

static int mp8859_pd_evt_notifier(struct notifier_block *nb,
				   unsigned long event, void *ptr)
{

	struct mp8859_chip *mp8859 =
		container_of(nb, struct mp8859_chip, cable_pd_nb);

	queue_delayed_work(mp8859->pd_wq, &mp8859->pd_work,
		   msecs_to_jiffies(100));

	return NOTIFY_DONE;
}

static void mp8859_pd_evt_worker(struct work_struct *work)
{
	struct mp8859_chip *mp8859 = container_of(work,
				struct mp8859_chip, pd_work.work);

	bool ufp;
	bool dp;

	dev_info(mp8859->dev, "mp8859 get notify\n");
	ufp = extcon_get_state(mp8859->ext_dev, EXTCON_USB);
	dp = extcon_get_state(mp8859->ext_dev, EXTCON_CHG_USB_FAST);
	dev_info(mp8859->dev, "mp8859 get ufp %d dp %d \n", ufp, dp);

	if (ufp) {
		mp8859_init(mp8859);
	}

}

static int mp8859_probe(struct i2c_client *client,const struct i2c_device_id *id)//探测
{
	struct mp8859_chip *mp8859;
	struct mp8859_platform_data *pdata;
	struct of_regulator_match *mp8859_reg_matches = NULL;
	//struct regulator_dev *sy_rdev;
	struct regulator_config config;
	struct regulator_desc *rdesc;
	struct extcon_dev *edev;
	int ret;

	mp8859 = devm_kzalloc(&client->dev, sizeof(struct mp8859_chip),
			      GFP_KERNEL);
	if (!mp8859)
		return -ENOMEM;

	i2c_set_clientdata(client, mp8859);//(void *)driver_data = mp8859

	pdata = dev_get_platdata(&client->dev);//兼容老式无dts，data接收

	if (!pdata)
		pdata = mp8859_parse_dt(client, &mp8859_reg_matches);//新式

	if (!pdata || !pdata->mp8859_init_data) {
		dev_err(&client->dev, "Platform data not found!\n");
		return -ENODEV;
	}

	mp8859->dev = &client->dev;
	mp8859->regmap = devm_regmap_init_i2c(client, &mp8859_regmap_config);
	if (IS_ERR(mp8859->regmap)) {
		dev_err(&client->dev, "Failed to allocate regmap!\n");
		return PTR_ERR(mp8859->regmap);
	}

	config.dev = mp8859->dev;
	config.driver_data = mp8859;
	config.init_data = pdata->mp8859_init_data;
	config.of_node = pdata->of_node;
	config.regmap = mp8859->regmap;

	rdesc = &mp8859->desc;
	rdesc->name = "mp8859_dcdc1",
	rdesc->id = 0,
	rdesc->ops = &mp8859_dcdc_ops,
	rdesc->n_voltages = MP8859_MAX_VSEL + 1,
	rdesc->linear_ranges = mp8859_voltage_ranges,
	rdesc->n_linear_ranges = ARRAY_SIZE(mp8859_voltage_ranges),
	//rdesc->vsel_reg = MP8859_VOUT_REG;
	rdesc->vsel_reg = MP8859_VOUT_H_REG;
	rdesc->vsel_mask = MP8859_VOUT_MASK;
	rdesc->enable_reg = MP8859_CTL1_REG;
	rdesc->enable_mask = MP8859_ENABLE;
	rdesc->enable_time = MP8859_ENABLE_TIME;
	rdesc->type = REGULATOR_VOLTAGE,
	rdesc->owner = THIS_MODULE;

	ret = mp8859_power_supply_init(mp8859);
	if (ret) {
		dev_err(mp8859->dev, "failed to register power supply device\n");
		return ret;
	}

/* type-C */
	edev = extcon_get_edev_by_phandle(&client->dev, 0);
	if (IS_ERR(edev)) {
		if (PTR_ERR(edev) != -EPROBE_DEFER)
			dev_err(mp8859->dev, "Invalid or missing extcon dev0\n");
		mp8859->ext_dev = NULL;
		dev_err(mp8859->dev, "Invalid or missing extcon dev0\n");
	} else {
		mp8859->ext_dev = edev;
	}

	mp8859_init(mp8859);

	if (mp8859->ext_dev != NULL) {
		dev_info(mp8859->dev, "register ext0 dev0\n");
		mp8859->pd_wq = alloc_ordered_workqueue("%s",
							  WQ_MEM_RECLAIM |
							  WQ_FREEZABLE,
							  "mp8859-wq");

		INIT_DELAYED_WORK(&mp8859->pd_work, mp8859_pd_evt_worker);
		mp8859->cable_pd_nb.notifier_call = mp8859_pd_evt_notifier;

		extcon_register_notifier(mp8859->ext_dev,
					 EXTCON_USB,
					 &mp8859->cable_pd_nb);
/*
		extcon_register_notifier(mp8859->ext_dev,
					 EXTCON_CHG_USB_FAST,
					 &mp8859->cable_pd_nb);
*/
	}

	dev_info(mp8859->dev, "mp8859 register successful\n");

		return 0;
}

static const struct i2c_device_id mp8859_id[] = {
	{ .name = "mps,mp8859", },
	{  },
};
MODULE_DEVICE_TABLE(i2c, mp8859_id);
static struct i2c_driver mp8859_driver = {
	.driver = {
		.name = "mp8859",
		.of_match_table = of_match_ptr(mp8859_of_match),
		.owner = THIS_MODULE,
	},
	.probe    = mp8859_probe,
	.id_table = mp8859_id,
};
module_i2c_driver(mp8859_driver);
/*
static int __init mp8859_regulator_init(void)
{
	return i2c_add_driver(&mp8859_driver);
}
subsys_initcall(mp8859_regulator_init);


static void __exit mp8859_regulator_exit(void)
{
	return i2c_del_driver(&mp8859_driver);
}
module_exit(mp8859_regulator_exit);
*/
MODULE_AUTHOR("daijh");
MODULE_DESCRIPTION("mp8859 voltage regulator driver");
MODULE_LICENSE("GPL v2");
