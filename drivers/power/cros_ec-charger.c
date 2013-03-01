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

#define CHARGING_MASK (TSU6721_TYPE_USB_HOST | TSU6721_TYPE_CHG12 | \
		       TSU6721_TYPE_CDP | TSU6721_TYPE_DCP | \
		       TSU6721_TYPE_APPLE_CHG | TSU6721_TYPE_U200_CHG | \
		       TSU6721_TYPE_NON_STD_CHG)

struct charger_data {
	struct device *dev;
	struct power_supply charger;
};

static enum power_supply_property cros_ec_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE, /* charger is active or not */
};

static int cros_ec_charger_get_prop(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct charger_data *charger_data = container_of(psy,
							 struct charger_data,
							 charger);
	struct device *dev = charger_data->dev;
	struct cros_ec_device *ec = dev_get_drvdata(dev->parent);
	struct cros_ec_command msg;
	struct ec_response_power_info ec_info;
	int ret;

	val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
	msg.version = 0;
	msg.command = EC_CMD_POWER_INFO;
	msg.outdata = NULL;
	msg.outsize = 0;
	msg.indata = (u8 *)&ec_info;
	msg.insize = sizeof(struct ec_response_power_info);
	ret = cros_ec_cmd_xfer(ec, &msg);
	if (ret < 0) {
		dev_err(dev, "Unable to query EC power info (err:%d)\n",
			ret);
		return ret;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !!(ec_info.usb_dev_type & CHARGING_MASK);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int cros_ec_charger_probe(struct platform_device *pd)
{
	struct cros_ec_device *ec = dev_get_drvdata(pd->dev.parent);
	struct charger_data *charger_data;
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

	charger_data->dev = dev;

	charger_data->charger.name = "cros-ec-charger";
	charger_data->charger.type = POWER_SUPPLY_TYPE_MAINS;
	charger_data->charger.get_property = cros_ec_charger_get_prop;
	charger_data->charger.properties = cros_ec_charger_props;
	charger_data->charger.num_properties =
		ARRAY_SIZE(cros_ec_charger_props);

	platform_set_drvdata(pd, charger_data);

	ret = power_supply_register(dev, &charger_data->charger);
	if (ret) {
		dev_err(dev, "failed: power supply register\n");
		return ret;
	}

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
