/*
 * Copyright (c) 2013 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Power supply consumer driver for ChromeOS EC Charger
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

/* Device Type 1 Reg */
#define TSU6721_TYPE_NONE		0x000000
#define TSU6721_TYPE_USB_HOST		0x000004
#define TSU6721_TYPE_CHG12		0x000010
#define TSU6721_TYPE_CDP		0x000020
#define TSU6721_TYPE_DCP		0x000040

/* Device Type 2 Reg */
#define TSU6721_TYPE_JIG_UART_ON	0x000400
#define TSU6721_TYPE_AUDIO3		0x008000

/* Device Type 3 Reg */
#define TSU6721_TYPE_APPLE_CHG		0x200000
#define TSU6721_TYPE_U200_CHG		0x400000
#define TSU6721_TYPE_NON_STD_CHG	0x040000

/* VBUS_DEBOUNCED might show up together with other type */
#define TSU6721_TYPE_VBUS_DEBOUNCED	0x020000

#define CHARGING_MASK (TSU6721_TYPE_USB_HOST | TSU6721_TYPE_CHG12 | \
		       TSU6721_TYPE_CDP | TSU6721_TYPE_DCP | \
		       TSU6721_TYPE_APPLE_CHG | TSU6721_TYPE_U200_CHG | \
		       TSU6721_TYPE_NON_STD_CHG | TSU6721_TYPE_JIG_UART_ON)

struct charger_data {
	struct device *dev;
	struct power_supply charger;
};

static enum power_supply_property cros_ec_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE, /* charger is active or not */
	POWER_SUPPLY_PROP_CURRENT_NOW, /* current flowing out of charger */
	POWER_SUPPLY_PROP_VOLTAGE_NOW, /* voltage at charger */
	POWER_SUPPLY_PROP_POWER_NOW, /* product of voltage & current props */
};

static int is_debounced(struct ec_response_power_info *ec_data)
{
	return !!(ec_data->usb_dev_type & TSU6721_TYPE_VBUS_DEBOUNCED);
}

static void update_psu_type(struct power_supply *psy,
			    struct ec_response_power_info *ec_data)
{
	dev_dbg(psy->dev, "dev_type = 0x%06x cur_limit = %d\n",
		ec_data->usb_dev_type & CHARGING_MASK,
		ec_data->usb_current_limit);

	psy->type = POWER_SUPPLY_TYPE_UNKNOWN;
	if (is_debounced(ec_data)) {
		switch (ec_data->usb_dev_type & CHARGING_MASK) {
		case TSU6721_TYPE_USB_HOST:
		case TSU6721_TYPE_JIG_UART_ON:
			psy->type = POWER_SUPPLY_TYPE_USB;
			break;
		case TSU6721_TYPE_CDP:
			psy->type = POWER_SUPPLY_TYPE_USB_CDP;
			break;
		case TSU6721_TYPE_DCP:
		case TSU6721_TYPE_APPLE_CHG:
			psy->type = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		case TSU6721_TYPE_CHG12:
			psy->type = POWER_SUPPLY_TYPE_MAINS;
			break;
		}
	}
}

static int get_ec_power_info(struct power_supply *psy,
			     struct ec_response_power_info *ec_info)
{
	struct charger_data *charger_data = container_of(psy,
							 struct charger_data,
							 charger);
	struct cros_ec_command msg;
	struct device *dev = charger_data->dev;
	struct cros_ec_device *ec = dev_get_drvdata(dev->parent);
	int ret;

	msg.version = 0;
	msg.command = EC_CMD_POWER_INFO;
	msg.outdata = NULL;
	msg.outsize = 0;
	msg.indata = (u8 *)ec_info;
	msg.insize = sizeof(struct ec_response_power_info);
	ret = cros_ec_cmd_xfer(ec, &msg);
	if (ret < 0) {
		dev_err(dev, "Unable to query EC power info (err:%d)\n",
			ret);
		return ret;
	}
	return 0;
}

static void cros_ec_charger_power_changed(struct power_supply *psy)
{
	struct ec_response_power_info ec_info;

	get_ec_power_info(psy, &ec_info);
	update_psu_type(psy, &ec_info);
}

static int cros_ec_charger_get_prop(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct ec_response_power_info ec_info;
	int ret = get_ec_power_info(psy, &ec_info);
	if (ret)
		return -EINVAL;

	/* Zero properties unless we've detected presence of AC */
	if (!is_debounced(&ec_info)) {
		val->intval = 0;
		return 0;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = ec_info.current_system * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = ec_info.voltage_system * 1000;
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = ec_info.voltage_system *
			ec_info.current_system;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static char *charger_supplied_to[] = {"cros-ec-charger"};

static int cros_ec_charger_probe(struct platform_device *pd)
{
	struct cros_ec_device *ec = dev_get_drvdata(pd->dev.parent);
	struct charger_data *charger_data;
	struct power_supply *psy;
	struct device *dev = &pd->dev;
	struct device_node *mfd_np, *charger_np;
	int ret = 0;

	if (!ec) {
		dev_err(dev, "no EC device found\n");
		return -EINVAL;
	}

	mfd_np = dev->parent->of_node;
	if (!mfd_np) {
		dev_err(dev, "no device tree data available\n");
		return -EINVAL;
	}

	charger_np = of_find_node_by_name(mfd_np, "charger");
	if (!charger_np) {
		dev_err(dev, "no OF charger data found at %s\n",
			mfd_np->full_name);
		return -EINVAL;
	}

	charger_data = devm_kzalloc(dev, sizeof(struct charger_data),
				    GFP_KERNEL);
	if (!charger_data) {
		dev_err(dev, "Failed to alloc driver structure\n");
		return -ENOMEM;
	}

	psy = &charger_data->charger;
	charger_data->dev = dev;
	psy->name = "cros-ec-charger";

	psy->supplied_to = charger_supplied_to;
	psy->num_supplicants = ARRAY_SIZE(charger_supplied_to);
	psy->type = POWER_SUPPLY_TYPE_UNKNOWN;
	psy->get_property = cros_ec_charger_get_prop;
	psy->external_power_changed = cros_ec_charger_power_changed;
	psy->properties = cros_ec_charger_props;
	psy->num_properties = ARRAY_SIZE(cros_ec_charger_props);

	platform_set_drvdata(pd, charger_data);

	ret = power_supply_register(dev, psy);
	if (ret) {
		dev_err(dev, "failed: power supply register\n");
		return ret;
	}
	ec->charger = psy;

	return 0;
}

static int cros_ec_charger_remove(struct platform_device *pd)
{
	struct charger_data *charger_data = platform_get_drvdata(pd);

	power_supply_unregister(&charger_data->charger);
	platform_set_drvdata(pd, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cros_ec_charger_resume(struct device *dev)
{
	struct charger_data *charger_data = dev_get_drvdata(dev);

	power_supply_changed(&charger_data->charger);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(cros_ec_charger_pm_ops, NULL,
			 cros_ec_charger_resume);

static struct platform_driver cros_ec_charger_driver = {
	.driver = {
		.name = "cros-ec-charger",
		.owner = THIS_MODULE,
		.pm = &cros_ec_charger_pm_ops,
	},
	.probe = cros_ec_charger_probe,
	.remove = cros_ec_charger_remove,
};

module_platform_driver(cros_ec_charger_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Chrome EC charger");
MODULE_ALIAS("power_supply:cros-ec-charger");
