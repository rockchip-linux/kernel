/* drivers/power/rv1108_power.c
 *
 * battery detect driver for the rv1108
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
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/vermagic.h>
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/power_supply.h>
#include <linux/of_platform.h>

#define BATTERY_STABLE_COUNT	10
#define DIVIDER_RESISTANCE_A	200
#define DIVIDER_RESISTANCE_B	120

static int ac_online;
static int usb_online			= 1;
static int battery_status		= POWER_SUPPLY_STATUS_CHARGING;
static int battery_health		= POWER_SUPPLY_HEALTH_GOOD;
static int battery_present		= 1;
static int battery_technology	= POWER_SUPPLY_TECHNOLOGY_LION;
static int battery_capacity		= 50;
static int battery_voltage		= 3300;
static int pre_battery_capacity	= 100;

struct rv1108_battery_data {
	struct platform_device *pdev;
	struct delayed_work work;
	struct iio_channel *chan;

	int max_voltage;
	int *levels;
};

extern int dwc_vbus_status(void);

static int rv1108_power_get_ac_property(
	struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = ac_online;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int rv1108_power_get_usb_property(
	struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = usb_online;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int rv1108_power_get_battery_property(
	struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "RV1108 battery";
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Rock-Chip";
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = UTS_RELEASE;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = battery_status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = battery_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battery_present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = battery_technology;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = battery_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = 100;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		val->intval = 3600;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 26;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battery_voltage;
		break;
	default:
		pr_info("%s: some properties deliberately report errors.\n",
			__func__);
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property rv1108_power_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property rv1108_power_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property rv1108_power_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static char *rv1108_power_ac_supplied_to[] = {
	"rk_battery",
};

static char *rv1108_power_usb_supplied_to[] = {
	"rk_battery",
};

static struct power_supply rv1108_power_supplies[] = {
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = rv1108_power_ac_supplied_to,
		.num_supplicants = ARRAY_SIZE(rv1108_power_ac_supplied_to),
		.properties = rv1108_power_ac_props,
		.num_properties = ARRAY_SIZE(rv1108_power_ac_props),
		.get_property = rv1108_power_get_ac_property,
	}, {
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = rv1108_power_battery_props,
		.num_properties = ARRAY_SIZE(rv1108_power_battery_props),
		.get_property = rv1108_power_get_battery_property,
	}, {
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = rv1108_power_usb_supplied_to,
		.num_supplicants = ARRAY_SIZE(rv1108_power_usb_supplied_to),
		.properties = rv1108_power_usb_props,
		.num_properties = ARRAY_SIZE(rv1108_power_usb_props),
		.get_property = rv1108_power_get_usb_property,
	},
};

static int rv1108_battery_dt_parse(
	struct device *dev,
	struct rv1108_battery_data *gdata)
{
	int rv = 0;
	int length;
	struct iio_channel *chan;
	struct property *prop;

	chan = iio_channel_get(dev, NULL);
	if (IS_ERR(chan)) {
		dev_err(dev, "no io-channnels defined\n");
		chan = NULL;
		rv = PTR_ERR(chan);
		return rv;
	}
	gdata->chan = chan;

	prop = of_find_property(dev->of_node, "battery-levels", &length);
	if (!prop)
		return -EINVAL;

	gdata->max_voltage = length / sizeof(u32);
	/* read brightness levels from DT property */
	if (gdata->max_voltage > 0) {
		size_t size = sizeof(*gdata->levels) * gdata->max_voltage;

		gdata->levels = devm_kzalloc(dev, size, GFP_KERNEL);
		if (!gdata->levels)
			return -ENOMEM;

		rv = of_property_read_u32_array(
							dev->of_node,
							"battery-levels",
							gdata->levels,
							gdata->max_voltage);
		if (rv < 0)
			return rv;

		gdata->max_voltage--;
	}

	return 0;
}

static int rv1108_adc_to_voltage(int adc_value)
{
	int voltage;
	int ra = DIVIDER_RESISTANCE_A;
	int rb = DIVIDER_RESISTANCE_B;

	voltage = ((adc_value * 11) / 6) * (ra + rb) / rb;

	return voltage;
}

static int rv1108_battery_capacity_change(
	int adc_value,
	struct rv1108_battery_data *gdata)
{
	int i;
	int voltage;

	voltage = rv1108_adc_to_voltage(adc_value);
	battery_voltage = voltage;
	for (i = 0; i <= gdata->max_voltage; i++) {
		if (voltage < gdata->levels[i]) {
			battery_capacity = i;
			break;
		}
	}

	if (i == 100)
		battery_capacity = 100;

	if (battery_capacity != pre_battery_capacity) {
		pre_battery_capacity = battery_capacity;
		return 1;
	}

	return 0;
}

static int rv1108_battery_checkvbus(void)
{
	int vbus_connect;

	vbus_connect = dwc_vbus_status();
	if (vbus_connect >= 1)
		return 1;
	else
		return 0;
}

static void rv1108_battery_work_func(struct work_struct *work)
{
	struct rv1108_battery_data *gdata;
	struct iio_channel *channel;
	struct device *dev;
	int result = 0, i = 0, batv = 0;
	int value = 0;
	int changed_vbus = 0, changed_bat = 0;

	gdata = container_of(
			(struct delayed_work *)work,
			struct rv1108_battery_data,
			work);

	channel = gdata->chan;
	dev = &gdata->pdev->dev;

	/* Check vbus status */
	result = rv1108_battery_checkvbus();
	if (result != usb_online)
		changed_vbus = 1;
	if (result == 1) {
		usb_online = 1;
		battery_status = POWER_SUPPLY_STATUS_CHARGING;
	}
	if (result == 0) {
		usb_online = 0;
		battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	/* Read ADC value */
	for (i = 0; i < BATTERY_STABLE_COUNT; i++) {
		result = iio_read_channel_raw(channel, &value);
		if (result < 0) {
			dev_err(dev, "read adc channel 2 error:%d\n", result);
			goto out;
		}
		batv = batv + value;
	}
	batv = batv / BATTERY_STABLE_COUNT;

	changed_bat = rv1108_battery_capacity_change(batv, gdata);
	if (changed_bat)
		power_supply_changed(&rv1108_power_supplies[1]);
	if (changed_vbus)
		power_supply_changed(&rv1108_power_supplies[2]);

out:
	/* start work queue */
	schedule_delayed_work(&gdata->work, msecs_to_jiffies(500));
}

static int rv1108_battery_probe(struct platform_device *pdev)
{
	int i;
	int result;
	struct rv1108_battery_data *gdata;
	struct device *dev = &pdev->dev;

	for (i = 0; i < ARRAY_SIZE(rv1108_power_supplies); i++) {
		result = power_supply_register(NULL, &rv1108_power_supplies[i]);
		if (result) {
			dev_err(dev, "%s: failed to register\n", __func__);
			goto err1;
		}
	}

	gdata = devm_kzalloc(dev, sizeof(*gdata), GFP_KERNEL);
	if (gdata == NULL) {
		dev_err(dev, "rv1108 battery malloc failure!\n");
		result = -ENOMEM;
		goto err1;
	}
	gdata->pdev = pdev;

	result = rv1108_battery_dt_parse(dev, gdata);
	if (result) {
		dev_err(dev, "rv1108 analysis DTS failure!\n");
		goto err1;
	}

	INIT_DELAYED_WORK(&gdata->work, rv1108_battery_work_func);
	schedule_delayed_work(&gdata->work, msecs_to_jiffies(500));

	return 0;
err1:
	while (--i >= 0)
		power_supply_unregister(&rv1108_power_supplies[i]);
	return result;
}

static int rv1108_battery_remove(struct platform_device *pdev)
{
	int i;
	/* Let's see how we handle changes... */
	ac_online = 0;
	usb_online = 0;
	battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
	for (i = 0; i < ARRAY_SIZE(rv1108_power_supplies); i++)
		power_supply_changed(&rv1108_power_supplies[i]);
	pr_info("%s: 'changed' event sent, sleeping for 10 seconds...\n",
		__func__);
	ssleep(10);

	for (i = 0; i < ARRAY_SIZE(rv1108_power_supplies); i++)
		power_supply_unregister(&rv1108_power_supplies[i]);

	return 0;
}

static struct of_device_id battery_match_table[] = {
	{.compatible = "rockchip,rv1108_battery"},
	{},
};

static struct platform_driver rv1108_battery_driver = {
	.probe  = rv1108_battery_probe,
	.remove = rv1108_battery_remove,
	.driver = {
		.name  = "rv1108-battery",
		.owner = THIS_MODULE,
		.of_match_table = battery_match_table,
	},
};

module_platform_driver(rv1108_battery_driver);

MODULE_DESCRIPTION("RV1108 Power supply driver");
MODULE_AUTHOR("wangruoming <wrm@rock-chips.com>");
MODULE_LICENSE("GPL");
