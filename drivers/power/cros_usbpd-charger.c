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

#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/rtc.h>

/*
 * TODO: hack alert! Move cros_ec_dev out of here to
 * include/mfd/cros_ec_dev.h.
 */
#include "../mfd/cros_ec_dev.h"

#define CROS_USB_PD_MAX_PORTS		8
#define CROS_USB_PD_MAX_LOG_ENTRIES	30

#define CROS_USB_PD_LOG_UPDATE_DELAY msecs_to_jiffies(60000)

/* Buffer + macro for building PDLOG string */
#define BUF_SIZE 80
#define APPEND_STRING(buf, len, str, ...) ((len) += \
	snprintf((buf) + (len), max(BUF_SIZE - (len), 0), (str), ##__VA_ARGS__))

#define CHARGER_DIR_NAME		"CROS_USB_PD_CHARGER%d"
#define CHARGER_DIR_NAME_LENGTH		sizeof(CHARGER_DIR_NAME)

#define MANUFACTURER_MODEL_LENGTH	32

struct port_data {
	int port_number;
	char name[CHARGER_DIR_NAME_LENGTH];
	char manufacturer[MANUFACTURER_MODEL_LENGTH];
	char model_name[MANUFACTURER_MODEL_LENGTH];
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
	struct delayed_work log_work;
	struct workqueue_struct *log_workqueue;
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
	return cros_ec_cmd_xfer_status(ec_device, &msg);
}

static int set_ec_usb_pd_override_ports(struct charger_data *charger,
					int port_num)
{
	struct device *dev = charger->dev;
	struct ec_params_charge_port_override req;
	int ret;

	req.override_port = port_num;

	ret = ec_command(charger, EC_CMD_PD_CHARGE_PORT_OVERRIDE,
			 (uint8_t *)&req, sizeof(req),
			 NULL, 0);
	if (ret < 0) {
		dev_info(dev, "Port Override command returned 0x%x\n", ret);
		return -EINVAL;
	}

	return 0;
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

static int get_ec_usb_pd_discovery_info(struct port_data *port)
{
	struct charger_data *charger = port->charger;
	struct device *dev = charger->dev;
	struct ec_params_usb_pd_info_request req;
	struct ec_params_usb_pd_discovery_entry resp;
	int ret;

	req.port = port->port_number;

	ret = ec_command(charger, EC_CMD_USB_PD_DISCOVERY,
			 (uint8_t *)&req, sizeof(req),
			 (uint8_t *)&resp, sizeof(resp));
	if (ret < 0) {
		dev_err(dev, "Unable to query Discovery info (err:0x%x)\n",
			 ret);
		return -EINVAL;
	}

	dev_dbg(dev, "Port %d: VID = 0x%x, PID=0x%x, PTYPE=0x%x\n",
		port->port_number, resp.vid, resp.pid, resp.ptype);

	snprintf(port->manufacturer, MANUFACTURER_MODEL_LENGTH, "%x", resp.vid);
	snprintf(port->model_name, MANUFACTURER_MODEL_LENGTH, "%x", resp.pid);

	return 0;
}

static int get_ec_usb_pd_power_info(struct port_data *port)
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
	if (ret < 0) {
		dev_err(dev, "Unable to query PD power info (err:0x%x)\n", ret);
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
		port->psy_online = 0;
		break;
	case USB_PD_PORT_POWER_SINK:
		dev_dbg(dev, "Port %d: SINK", port->port_number);
		port->psy_status = POWER_SUPPLY_STATUS_CHARGING;
		port->psy_online = 1;
		break;
	case USB_PD_PORT_POWER_SINK_NOT_CHARGING:
		dev_dbg(dev, "Port %d: NOT_CHARGING", port->port_number);
		port->psy_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		port->psy_online = 1;
		break;
	default:
		dev_err(dev, "Unknown role %d\n", resp.role);
		break;
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
	case USB_CHG_TYPE_VBUS:
		dev_dbg(dev, "Port %d: Charger type: %d\n",
			port->port_number, resp.type);
		/*
		 * Report all type of USB chargers as POWER_SUPPLY_TYPE_USB to
		 * keep userland low power charger detection logic simpler.
		 */
		port->psy_type = POWER_SUPPLY_TYPE_USB;
		break;
	default:
		dev_err(dev, "Port %d: default case!\n",
			port->port_number);
		port->psy_type = POWER_SUPPLY_TYPE_UNKNOWN;
	}

	port->psy.type = port->psy_type;

	dev_dbg(dev, "Port %d: Voltage max: %dmV\n",
		port->port_number, resp.meas.voltage_max);
	port->psy_voltage_max_design = resp.meas.voltage_max;
	dev_dbg(dev, "Port %d: Voltage now: %dmV\n",
		port->port_number, resp.meas.voltage_now);
	port->psy_voltage_now = resp.meas.voltage_now;
	dev_dbg(dev, "Port %d: Current max: %dmA\n",
		port->port_number, resp.meas.current_max);
	port->psy_current_max = resp.meas.current_max;
	dev_dbg(dev, "Port %d: Power max: %dmW\n",
		port->port_number, resp.max_power);
	port->psy_power_max = resp.max_power;

	return 0;
}

static int get_ec_port_status(struct port_data *port)
{
	int ret;

	ret = get_ec_usb_pd_power_info(port);
	if (ret < 0)
		return ret;

	return get_ec_usb_pd_discovery_info(port);
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
	if (ret < 0) {
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
		val->strval = port->model_name;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = port->manufacturer;
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
	struct port_data *port = container_of(psy, struct port_data, psy);
	struct charger_data *charger = port->charger;
	int port_number;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		/*
		 * A value of -1 implies switching to battery as the power
		 * source. Any other value implies using this port as the
		 * power source.
		 */
		port_number = val->intval;
		if (port_number != -1)
			port_number = port->port_number;
		return set_ec_usb_pd_override_ports(charger, port_number);
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
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

static void cros_usb_pd_print_log_entry(struct ec_response_pd_log *r,
					ktime_t tstamp)
{
	static const char * const fault_names[] = {
		"---", "OCP", "fast OCP", "OVP", "Discharge"
	};
	static const char * const role_names[] = {
		"Disconnected", "SRC", "SNK", "SNK (not charging)"
	};
	static const char * const chg_type_names[] = {
		"None", "PD", "Type-C", "Proprietary",
		"DCP", "CDP", "SDP", "Other", "VBUS"
	};
	int i;
	int role_idx, type_idx;
	const char *fault, *role, *chg_type;
	struct usb_chg_measures *meas;
	struct mcdp_info *minfo;
	struct rtc_time rt;
	int len = 0;
	char buf[BUF_SIZE + 1];

	/* the timestamp is the number of 1024th of seconds in the past */
	tstamp = ktime_sub_us(tstamp,
		 (uint64_t)r->timestamp << PD_LOG_TIMESTAMP_SHIFT);
	rt = rtc_ktime_to_tm(tstamp);

	switch (r->type) {
	case PD_EVENT_MCU_CHARGE:
		if (r->data & CHARGE_FLAGS_OVERRIDE)
			APPEND_STRING(buf, len, "override ");
		if (r->data & CHARGE_FLAGS_DELAYED_OVERRIDE)
			APPEND_STRING(buf, len, "pending_override ");
		role_idx = r->data & CHARGE_FLAGS_ROLE_MASK;
		role = role_idx < ARRAY_SIZE(role_names) ?
			role_names[role_idx] : "Unknown";
		type_idx = (r->data & CHARGE_FLAGS_TYPE_MASK)
			 >> CHARGE_FLAGS_TYPE_SHIFT;
		chg_type = type_idx < ARRAY_SIZE(chg_type_names) ?
			chg_type_names[type_idx] : "???";

		if ((role_idx == USB_PD_PORT_POWER_DISCONNECTED) ||
		    (role_idx == USB_PD_PORT_POWER_SOURCE)) {
			APPEND_STRING(buf, len, "%s", role);
			break;
		}

		meas = (struct usb_chg_measures *)r->payload;
		APPEND_STRING(buf, len, "%s %s %s %dmV max %dmV / %dmA", role,
			r->data & CHARGE_FLAGS_DUAL_ROLE ? "DRP" : "Charger",
			chg_type,
			meas->voltage_now,
			meas->voltage_max,
			meas->current_max);
		break;
	case PD_EVENT_ACC_RW_FAIL:
		APPEND_STRING(buf, len, "RW signature check failed");
		break;
	case PD_EVENT_PS_FAULT:
		fault = r->data < ARRAY_SIZE(fault_names) ? fault_names[r->data]
							  : "???";
		APPEND_STRING(buf, len, "Power supply fault: %s", fault);
		break;
	case PD_EVENT_VIDEO_DP_MODE:
		APPEND_STRING(buf, len, "DP mode %sabled",
			      (r->data == 1) ? "en" : "dis");
		break;
	case PD_EVENT_VIDEO_CODEC:
		minfo = (struct mcdp_info *)r->payload;
		APPEND_STRING(buf, len,
			      "HDMI info: family:%04x chipid:%04x "
			      "irom:%d.%d.%d fw:%d.%d.%d",
			      MCDP_FAMILY(minfo->family),
			      MCDP_CHIPID(minfo->chipid),
			      minfo->irom.major, minfo->irom.minor,
			      minfo->irom.build, minfo->fw.major,
			      minfo->fw.minor, minfo->fw.build);
		break;
	default:
		APPEND_STRING(buf, len,
			"Event %02x (%04x) [", r->type, r->data);
		for (i = 0; i < PD_LOG_SIZE(r->size_port); i++)
			APPEND_STRING(buf, len, "%02x ", r->payload[i]);
		APPEND_STRING(buf, len, "]");
		break;
	}

	pr_info("PDLOG %d/%02d/%02d %02d:%02d:%02d.%03d P%d %s\n",
		rt.tm_year + 1900, rt.tm_mon + 1, rt.tm_mday,
		rt.tm_hour, rt.tm_min, rt.tm_sec,
		(int)(ktime_to_ms(tstamp) % MSEC_PER_SEC),
		PD_LOG_PORT(r->size_port), buf);
}

static void cros_usb_pd_log_check(struct work_struct *work)
{
	struct charger_data *charger = container_of(to_delayed_work(work),
		struct charger_data, log_work);
	struct device *dev = charger->dev;
	union {
		struct ec_response_pd_log r;
		uint32_t words[8]; /* space for the payload */
	} u;
	int ret;
	int entries = 0;
	ktime_t now;

	while (entries++ < CROS_USB_PD_MAX_LOG_ENTRIES) {
		ret = ec_command(charger, EC_CMD_PD_GET_LOG_ENTRY,
				 NULL, 0, (uint8_t *)&u, sizeof(u));
		now = ktime_get_real();
		if (ret < 0) {
			dev_dbg(dev, "Cannot get PD log %d\n", ret);
			break;
		}
		if (u.r.type == PD_EVENT_NO_ENTRY)
			break;

		cros_usb_pd_print_log_entry(&u.r, now);
	}

	queue_delayed_work(charger->log_workqueue, &charger->log_work,
		CROS_USB_PD_LOG_UPDATE_DELAY);
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
		dev_err(dev, "Failed to alloc charger. Failing probe.\n");
		return -ENOMEM;
	}

	charger->dev = dev;
	charger->ec_dev = ec_dev;
	charger->ec_device = ec_device;

	platform_set_drvdata(pd, charger);

	if ((get_ec_num_ports(charger, &charger->num_charger_ports) < 0) ||
	    !charger->num_charger_ports) {
		/*
		 * This can happen on a system that doesn't support USB PD.
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

		ret = power_supply_register_no_ws(dev, psy);
		if (ret) {
			dev_err(dev, "Failed to register power supply\n");
			continue;
		}

		charger->ports[charger->num_registered_psy++] = port;
		ec_device->charger = psy;
	}

	if (!charger->num_registered_psy) {
		ret = -ENODEV;
		dev_err(dev, "No power supplies registered\n");
		goto fail;
	}

	/* Retrieve PD event logs periodically */
	INIT_DELAYED_WORK(&charger->log_work, cros_usb_pd_log_check);
	charger->log_workqueue =
		create_singlethread_workqueue("cros_usb_pd_log");
	queue_delayed_work(charger->log_workqueue, &charger->log_work,
		CROS_USB_PD_LOG_UPDATE_DELAY);

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

	dev_info(dev, "Failing probe (err:0x%x)\n", ret);
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
		flush_delayed_work(&charger->log_work);
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

	if (!charger)
		return 0;

	dev_dbg(dev, "cros_usb_pd_charger_resume: updating power supplies\n");
	for (i = 0; i < charger->num_registered_psy; i++)
		power_supply_changed(&charger->ports[i]->psy);
	queue_delayed_work(charger->log_workqueue, &charger->log_work,
		CROS_USB_PD_LOG_UPDATE_DELAY);

	return 0;
}

static int cros_usb_pd_charger_suspend(struct device *dev)
{
	struct charger_data *charger = dev_get_drvdata(dev);

	if (charger)
		flush_delayed_work(&charger->log_work);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(cros_usb_pd_charger_pm_ops,
	cros_usb_pd_charger_suspend, cros_usb_pd_charger_resume);

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
