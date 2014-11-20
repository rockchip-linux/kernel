/*
 * Copyright (c) 2014 Google, Inc
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
 * Power supply driver for ChromeOS EC based USB PD Charger.
 */

#include <linux/module.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

/*
 * TODO: hack alert! Move cros_ec_dev out of here to
 * include/mfd/cros_ec_dev.h.
 */
#include "../mfd/cros_ec_dev.h"

#define CHARGER_DIR_NAME		"CROS_USB_PD_CHARGER%d"
#define CHARGER_DIR_NAME_LENGTH		sizeof(CHARGER_DIR_NAME)

#define CROS_USB_PD_MAX_PORTS		8

struct port_data {
	int port_number;
	char name[CHARGER_DIR_NAME_LENGTH];
	struct power_supply psy;
	int psy_type;
	int psy_online;
	int psy_status;
	int psy_current_max;
	int psy_voltage_max_design;
	int psy_voltage_now;
	int psy_power_max;
	struct charger_data *charger;
};

struct charger_data {
	struct device *dev;
	struct cros_ec_dev *ec_dev;
	struct cros_ec_device *ec_device;
	int num_charger_ports;
	int num_registered_psy;
	struct port_data *ports[CROS_USB_PD_MAX_PORTS];
};

#define EC_MAX_IN_SIZE EC_PROTO2_MAX_REQUEST_SIZE
#define EC_MAX_OUT_SIZE EC_PROTO2_MAX_RESPONSE_SIZE
uint8_t ec_inbuf[EC_MAX_IN_SIZE];
uint8_t ec_outbuf[EC_MAX_OUT_SIZE];

static enum power_supply_property cros_usb_pd_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

int ec_command(struct charger_data *charger, int command,
	       uint8_t *outdata, int outsize, uint8_t *indata, int insize)
{
	struct cros_ec_device *ec_device = charger->ec_device;
	struct cros_ec_dev *ec_dev = charger->ec_dev;
	struct cros_ec_command msg = {0};

	msg.command = ec_dev->cmd_offset + command;
	msg.outdata = outdata;
	msg.outsize = outsize;
	msg.indata = indata;
	msg.insize = insize;
	return cros_ec_cmd_xfer(ec_device, &msg);
}

static int get_ec_num_ports(struct charger_data *charger, int *num_ports)
{
	struct device *dev = charger->dev;
	struct ec_response_usb_pd_ports *resp =
	    (struct ec_response_usb_pd_ports *)ec_inbuf;
	int ret;

	*num_ports = 0;
	ret = ec_command(charger, EC_CMD_USB_PD_PORTS,
			 NULL, 0, ec_inbuf, EC_MAX_IN_SIZE);
	if (ret < 0) {
		dev_err(dev, "Unable to query PD ports (err:0x%x)\n", ret);
		return ret;
	}
	*num_ports = resp->num_ports;
	dev_dbg(dev, "Num ports = %d\n", *num_ports);

	return 0;
}

static int get_ec_port_status(struct port_data *port)
{
	struct charger_data *charger = port->charger;
	struct device *dev = charger->dev;
	struct ec_params_usb_pd_power_info req;
	struct ec_response_usb_pd_power_info resp;
	int ret;

	req.port = port->port_number;
	ret = ec_command(charger, EC_CMD_USB_PD_POWER_INFO,
			 (uint8_t *)&req, sizeof(req),
			 (uint8_t *)&resp, sizeof(resp));
	if (!ret) {
		WARN(1, "%s: Unable to query PD power info (err:0x%x)\n",
		     dev_name(dev), ret);
		return -EINVAL;
	}

	switch (resp.role) {
	case USB_PD_PORT_POWER_DISCONNECTED:
		dev_dbg(dev, "Port %d: DISCONNECTED", port->port_number);
		port->psy_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		port->psy_online = 0;
		break;
	case USB_PD_PORT_POWER_SOURCE:
		dev_dbg(dev, "Port %d: SOURCE", port->port_number);
		port->psy_status = POWER_SUPPLY_STATUS_DISCHARGING;
		port->psy_online = 1;
		break;
	case USB_PD_PORT_POWER_SINK:
		dev_dbg(dev, "Port %d: SINK", port->port_number);
		port->psy_status = POWER_SUPPLY_STATUS_CHARGING;
		port->psy_online = 1;
		break;
	case USB_PD_PORT_POWER_SINK_NOT_CHARGING:
		dev_dbg(dev, "Port %d: NOT_CHARGING", port->port_number);
		port->psy_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		port->psy_online = 0;
		break;
	default:
		dev_err(dev, "Unknown role %d\n", resp.role);
	}

	switch (resp.type) {
	case USB_CHG_TYPE_NONE:
		dev_dbg(dev, "Port %d: Charger type: None\n",
			port->port_number);
		port->psy_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	case USB_CHG_TYPE_PD:
	case USB_CHG_TYPE_PROPRIETARY:
	case USB_CHG_TYPE_C:
	case USB_CHG_TYPE_BC12_DCP:
	case USB_CHG_TYPE_BC12_CDP:
	case USB_CHG_TYPE_BC12_SDP:
		dev_dbg(dev, "Port %d: Charger type: %d\n",
			port->port_number, resp.type);
		/*
		 * TODO: Once the EC supports dual role property, report:
		 *   USB_CHG_TYPE_PD && !dual_role: Mains
		 *   USB_CHG_TYPE_PD && dual_role: USB (or better yet, USB_PD)
		 *   other types: None
		 */
		port->psy_type = POWER_SUPPLY_TYPE_MAINS;
		break;
	default:
		dev_err(dev, "Port %d: default case!\n",
			port->port_number);
		port->psy_type = POWER_SUPPLY_TYPE_UNKNOWN;
	}

	port->psy.type = port->psy_type;

	dev_dbg(dev, "Port %d: Charging voltage: %dmV\n",
		port->port_number, resp.voltage_now);
	port->psy_voltage_now = resp.voltage_now;
	dev_dbg(dev, "Port %d: Max input current: %dmA\n",
		port->port_number, resp.current_max);
	port->psy_current_max = resp.current_max;
	dev_dbg(dev, "Port %d: Max input power: %dmW\n",
		port->port_number, resp.max_power);
	port->psy_power_max = resp.max_power;


	return 0;
}

static void cros_usb_pd_charger_power_changed(struct power_supply *psy)
{
	struct port_data *port = container_of(psy, struct port_data, psy);
	struct charger_data *charger = port->charger;
	struct device *dev = charger->dev;
	int i;

	dev_dbg(dev, "cros_usb_pd_charger_power_changed\n");
	for (i = 0; i < charger->num_registered_psy; i++)
		power_supply_changed(&charger->ports[i]->psy);
}

static int cros_usb_pd_charger_get_prop(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct port_data *port = container_of(psy, struct port_data, psy);
	struct charger_data *charger = port->charger;
	struct device *dev = charger->dev;
	int ret;

	/* TODO: use cached values instead? */
	ret = get_ec_port_status(port);
	if (ret) {
		dev_err(dev, "Failed to get port status (err:0x%x)\n", ret);
		return -EINVAL;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = port->psy_online;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = port->psy_status;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = port->psy_current_max * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = port->psy_voltage_max_design * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* TODO: change this to voltage_now. */
		val->intval = port->psy_voltage_now * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		/* TODO: send a TBD host command to the EC. */
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = NULL;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = NULL;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int cros_usb_pd_charger_set_prop(struct power_supply *psy,
				    enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		/*
		 * TODO: Send a TBD host command to the EC to change the
		 * charging role of the port associated with this psy.
		 */
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		/*
		 * TODO: Send a TBD host command to the EC with port number
		 * set to -1.
		 */
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int cros_usb_pd_charger_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

static char *charger_supplied_to[] = {"cros-usb_pd-charger"};

static int cros_usb_pd_charger_probe(struct platform_device *pd)
{
	struct device *dev = &pd->dev;
	struct cros_ec_dev *ec_dev = dev_get_drvdata(pd->dev.parent);
	struct cros_ec_device *ec_device;
	struct charger_data *charger;
	struct port_data *port;
	struct power_supply *psy;
	int i;
	int ret = -EINVAL;

	dev_dbg(dev, "cros_usb_pd_charger_probe\n");
	if (!ec_dev) {
		WARN(1, "%s: No EC dev found\n", dev_name(dev));
		return -EINVAL;
	}

	ec_device = ec_dev->ec_dev;
	if (!ec_device) {
		WARN(1, "%s: No EC device found\n", dev_name(dev));
		return -EINVAL;
	}

	charger = devm_kzalloc(dev, sizeof(struct charger_data),
				    GFP_KERNEL);
	if (!charger) {
		WARN(1, "%s: Failed to alloc charger\n", dev_name(dev));
		return -ENOMEM;
	}

	charger->dev = dev;
	charger->ec_dev = ec_dev;
	charger->ec_device = ec_device;

	platform_set_drvdata(pd, charger);

	if (get_ec_num_ports(charger, &charger->num_charger_ports)) {
		/*
		 * This can happen on a system that doesn't supprt USB PD.
		 * Log a message, but no need to warn.
		 */
		dev_info(dev, "No charging ports found\n");
		ret = -ENODEV;
		goto fail_nowarn;
	}

	for (i = 0; i < charger->num_charger_ports; i++) {
		port = devm_kzalloc(dev, sizeof(struct port_data), GFP_KERNEL);
		if (!port) {
			dev_err(dev, "Failed to alloc port structure\n");
			ret = -ENOMEM;
			goto fail;
		}

		port->charger = charger;
		port->port_number = i;
		sprintf(port->name, CHARGER_DIR_NAME, i);

		psy = &port->psy;
		psy->name = port->name;
		psy->supplied_to = charger_supplied_to;
		psy->num_supplicants = ARRAY_SIZE(charger_supplied_to);
		psy->type = POWER_SUPPLY_TYPE_UNKNOWN;
		psy->get_property = cros_usb_pd_charger_get_prop;
		psy->set_property = cros_usb_pd_charger_set_prop;
		psy->property_is_writeable = cros_usb_pd_charger_is_writeable;
		psy->external_power_changed = cros_usb_pd_charger_power_changed;
		psy->properties = cros_usb_pd_charger_props;
		psy->num_properties = ARRAY_SIZE(cros_usb_pd_charger_props);

		ret = power_supply_register(dev, psy);
		if (ret) {
			dev_err(dev, "Failed to register power supply\n");
			continue;
		}

		charger->ports[charger->num_registered_psy++] = port;
		ec_device->charger = psy;
	}

	if (!charger->num_registered_psy) {
		ret = -ENODEV;
		WARN(1, "%s: No power supplies registered\n", dev_name(dev));
		goto fail;
	}

	return 0;

fail:
	WARN(1, "%s: Failing probe (err:0x%x)\n", dev_name(dev), ret);

fail_nowarn:
	if (charger) {
		ec_device->charger = NULL;
		for (i = 0; i < charger->num_registered_psy; i++) {
			port = charger->ports[i];
			power_supply_unregister(&port->psy);
			devm_kfree(dev, port);
		}
		platform_set_drvdata(pd, NULL);
		devm_kfree(dev, charger);
	}

	return ret;
}

static int cros_usb_pd_charger_remove(struct platform_device *pd)
{
	struct charger_data *charger = platform_get_drvdata(pd);
	struct cros_ec_device *ec_device = charger->ec_device;
	struct device *dev = charger->dev;
	struct port_data *port;
	int i;

	if (charger) {
		ec_device->charger = NULL;
		for (i = 0; i < charger->num_registered_psy; i++) {
			port = charger->ports[i];
			power_supply_unregister(&port->psy);
			devm_kfree(dev, port);
		}
		platform_set_drvdata(pd, NULL);
		devm_kfree(dev, charger);
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cros_usb_pd_charger_resume(struct device *dev)
{
	struct charger_data *charger = dev_get_drvdata(dev);
	int i;

	dev_dbg(dev, "cros_usb_pd_charger_resume: updating power supplies\n");
	for (i = 0; i < charger->num_registered_psy; i++)
		power_supply_changed(&charger->ports[i]->psy);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(cros_usb_pd_charger_pm_ops, NULL,
			 cros_usb_pd_charger_resume);

static struct platform_driver cros_usb_pd_charger_driver = {
	.driver = {
		.name = "cros-usb-pd-charger",
		.owner = THIS_MODULE,
		.pm = &cros_usb_pd_charger_pm_ops,
	},
	.probe = cros_usb_pd_charger_probe,
	.remove = cros_usb_pd_charger_remove,
};

module_platform_driver(cros_usb_pd_charger_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Chrome USB PD charger");
MODULE_ALIAS("power_supply:cros-usb-pd-charger");
