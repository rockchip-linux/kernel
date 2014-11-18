/*
 * cros_ec_pd_update - Chrome OS EC Power Delivery Device FW Update Driver
 *
 * Copyright (C) 2014 Google, Inc
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
 * This driver communicates with a Chrome OS PD device and performs tasks
 * related to auto-updating its firmware.
 */

#include <linux/acpi.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_pd_update.h>
#include <linux/module.h>

#include "cros_ec_dev.h"

/* Store our PD device pointer so we can send update-related commands. */
static struct cros_ec_dev *pd_ec;

/**
 * firmware_images - Keep this updated with the latest RW FW + hash for each
 * PD device. Entries should be primary sorted by id_major and secondary
 * sorted by id_minor.
 */
static const struct cros_ec_pd_firmware_image firmware_images[] = {
	/* PD_DEVICE_TYPE_ZINGER */
	{
		.id_major = PD_DEVICE_TYPE_ZINGER,
		.id_minor = 1,
		.usb_vid = USB_VID_GOOGLE,
		.usb_pid = USB_PID_ZINGER,
		.filename = "cros-pd/zinger_000003.bin",
		.hash = { 0xc3, 0x81, 0x0e, 0x73, 0xfc,
			  0xb2, 0x39, 0xf9, 0x8a, 0xa3,
			  0xca, 0x62, 0x01, 0x43, 0xc0,
			  0x55, 0x7f, 0xed, 0x23, 0xed },
	},
};

static const int firmware_image_count = ARRAY_SIZE(firmware_images);

/**
 * cros_ec_pd_command - Send a command to the EC. Returns 0 on success,
 * <0 on failure.
 *
 * @dev: PD device
 * @pd_dev: EC PD device
 * @command: EC command
 * @outdata: EC command output data
 * @outsize: Size of outdata
 * @indata: EC command input data
 * @insize: Size of indata
 */
static int cros_ec_pd_command(struct device *dev,
			      struct cros_ec_dev *pd_dev,
			      int command,
			      uint8_t *outdata,
			      int outsize,
			      uint8_t *indata,
			      int insize)
{
	struct cros_ec_command msg = {0};
	int ret;

	if (!pd_dev) {
		dev_err(dev, "No ec_dev device\n");
		return -1;
	}

	msg.command = pd_dev->cmd_offset + command;
	msg.outdata = outdata;
	msg.outsize = outsize;
	msg.indata = indata;
	msg.insize = insize;

	ret = cros_ec_cmd_xfer(pd_dev->ec_dev, &msg);
	if (ret < 0) {
		dev_err(dev, "Command xfer error (err:%d)\n", ret);
		return ret;
	} else if (msg.result)
		return -EECRESULT - msg.result;
	else
		return EC_RES_SUCCESS;
}

/**
 * cros_ec_pd_get_status - Get info about a possible PD device attached to a
 * given port. Returns 0 on success, <0 on failure.
 *
 * @dev: PD device
 * @pd_dev: EC PD device
 * @port: Port # on device
 * @hash_entry: Stores received PD device RW FW info, on success
 * @discovery_entry: Stores received PD device USB info, if device present
 */
static int cros_ec_pd_get_status(struct device *dev,
				 struct cros_ec_dev *pd_dev,
				 int port,
				 struct ec_params_usb_pd_rw_hash_entry
					*hash_entry,
				 struct ec_params_usb_pd_discovery_entry
					*discovery_entry)
{
	struct ec_params_usb_pd_info_request info_request;
	int ret;

	info_request.port = port;
	ret = cros_ec_pd_command(dev, pd_dev, EC_CMD_USB_PD_DEV_INFO,
				 (uint8_t *)&info_request, sizeof(info_request),
				 (uint8_t *)hash_entry, sizeof(*hash_entry));
	/* Skip getting USB discovery data if no device present on port */
	if (ret < 0 || hash_entry->dev_id == PD_DEVICE_TYPE_NONE)
		return ret;

	return cros_ec_pd_command(dev, pd_dev, EC_CMD_USB_PD_DISCOVERY,
				  (uint8_t *)&info_request,
				  sizeof(info_request),
				  (uint8_t *)discovery_entry,
				  sizeof(*discovery_entry));
}

/**
 * cros_ec_pd_send_hash_entry - Inform the EC of a PD devices for which we
 * have firmware available. EC typically will not store more than four hashes.
 * Returns 0 on success, <0 on failure.
 *
 * @dev: PD device
 * @pd_dev: EC PD device
 * @fw: FW update image to inform the EC of
 */
static int cros_ec_pd_send_hash_entry(struct device *dev,
				      struct cros_ec_dev *pd_dev,
				      const struct cros_ec_pd_firmware_image
						   *fw)
{
	struct ec_params_usb_pd_rw_hash_entry hash_entry;

	hash_entry.dev_id = MAJOR_MINOR_TO_DEV_ID(fw->id_major, fw->id_minor);
	memcpy(hash_entry.dev_rw_hash, fw->hash, PD_RW_HASH_SIZE);

	return cros_ec_pd_command(dev, pd_dev, EC_CMD_USB_PD_RW_HASH_ENTRY,
				  (uint8_t *)&hash_entry, sizeof(hash_entry),
				  NULL, 0);
}

/**
 * cros_ec_pd_send_fw_update_cmd - Send update-related EC command.
 * Returns 0 on success, <0 on failure.
 *
 * @dev: PD device
 * @pd_dev: EC PD device
 * @pd_cmd: fw_update command
 */
static int cros_ec_pd_send_fw_update_cmd(struct device *dev,
					 struct cros_ec_dev *pd_dev,
					 struct ec_params_usb_pd_fw_update
						*pd_cmd)
{
	return cros_ec_pd_command(dev, pd_dev, EC_CMD_USB_PD_FW_UPDATE,
				  (uint8_t *)pd_cmd,
				  pd_cmd->size + sizeof(*pd_cmd),
				  NULL, 0);
}

/**
 * cros_ec_pd_get_num_ports - Get number of EC charge ports.
 * Returns 0 on success, <0 on failure.
 *
 * @dev: PD device
 * @pd_dev: EC PD device
 * @num_ports: Holds number of ports, on command success
 */
static int cros_ec_pd_get_num_ports(struct device *dev,
				    struct cros_ec_dev *pd_dev,
				    int *num_ports)
{
	struct ec_response_usb_pd_ports resp;
	int ret;

	ret = cros_ec_pd_command(dev, pd_dev, EC_CMD_USB_PD_PORTS,
				 NULL, 0,
				 (uint8_t *)&resp, sizeof(resp));
	if (ret == EC_RES_SUCCESS)
		*num_ports = resp.num_ports;
	return ret;
}


/**
 * cros_ec_pd_fw_update - Send EC_CMD_USB_PD_FW_UPDATE command to perform
 * update-related operation.
 * Returns 0 on success, <0 on failure.
 *
 * @dev: PD device
 * @pd_dev: EC PD device
 * @fw: RW FW update file
 * @port: Port# to which update device is attached
 */
static int cros_ec_pd_fw_update(struct device *dev,
				struct cros_ec_dev *pd_dev,
				const struct firmware *fw,
				uint8_t port)
{
	int i, ret;

	uint8_t cmd_buf[sizeof(struct ec_params_usb_pd_fw_update) +
			PD_FLASH_WRITE_STEP];
	struct ec_params_usb_pd_fw_update *pd_cmd =
		(struct ec_params_usb_pd_fw_update *)cmd_buf;
	uint8_t *pd_cmd_data = cmd_buf + sizeof(*pd_cmd);

	/* Common port */
	pd_cmd->port = port;

	/* Erase signature */
	pd_cmd->cmd = USB_PD_FW_ERASE_SIG;
	pd_cmd->size = 0;
	ret = cros_ec_pd_send_fw_update_cmd(dev, pd_dev, pd_cmd);
	if (ret < 0) {
		dev_err(dev, "Unable to clear PD signature (err:%d)\n", ret);
		return ret;
	}

	/* Reboot PD */
	pd_cmd->cmd = USB_PD_FW_REBOOT;
	pd_cmd->size = 0;
	ret = cros_ec_pd_send_fw_update_cmd(dev, pd_dev, pd_cmd);
	if (ret < 0) {
		dev_err(dev, "Unable to clear PD signature (err:%d)\n", ret);
		return ret;
	}

	/* Erase RW flash */
	pd_cmd->cmd = USB_PD_FW_FLASH_ERASE;
	pd_cmd->size = 0;
	ret = cros_ec_pd_send_fw_update_cmd(dev, pd_dev, pd_cmd);
	if (ret < 0) {
		dev_err(dev, "Unable to clear PD signature (err:%d)\n", ret);
		return ret;
	}

	/* Write RW flash */
	pd_cmd->cmd = USB_PD_FW_FLASH_WRITE;
	for (i = 0; i < fw->size; i += PD_FLASH_WRITE_STEP) {
		pd_cmd->size = min(fw->size - i, (size_t)PD_FLASH_WRITE_STEP);
		memcpy(pd_cmd_data, fw->data + i, pd_cmd->size);
		ret = cros_ec_pd_send_fw_update_cmd(dev, pd_dev, pd_cmd);
		if (ret < 0) {
			dev_err(dev, "Unable to write PD RW flash (err:%d)\n",
				ret);
			return ret;
		}
	}

	/* Reboot PD into new RW */
	pd_cmd->cmd = USB_PD_FW_REBOOT;
	pd_cmd->size = 0;
	ret = cros_ec_pd_send_fw_update_cmd(dev, pd_dev, pd_cmd);
	if (ret < 0) {
		dev_err(dev, "Unable to clear PD signature (err:%d)\n", ret);
		return ret;
	}

	return 0;
}

/**
 * find_firmware_image - Search firmware image table for an image matching
 * the passed attributes. Returns matching index if id is found and
 * PD_NO_IMAGE if not found in table.
 *
 * @dev_id: Target PD device id
 * @vid: Target USB VID
 * @pid: Target USB PID
 */
static int find_firmware_image(uint16_t dev_id, uint16_t vid, uint16_t pid)
{
	/*
	 * TODO(shawnn): Replace sequential table search with modified binary
	 * search on major / minor.
	 */
	int i;

	for (i = 0; i < firmware_image_count; ++i)
		if (MAJOR_MINOR_TO_DEV_ID(firmware_images[i].id_major,
					  firmware_images[i].id_minor)
					  == dev_id &&
		    firmware_images[i].usb_vid == vid &&
		    firmware_images[i].usb_pid == pid)
			return i;

	return PD_NO_IMAGE;
}

/**
 * cros_ec_pd_update_check - Probe the status of attached PD devices and kick
 * off an RW firmware update if needed. This is run as a deferred task on
 * module load, resume, and when an ACPI event is received (typically on
 * PD device insertion).
 *
 * @work: Delayed work pointer
 */
static void cros_ec_pd_update_check(struct work_struct *work)
{
	const struct firmware *fw;
	struct ec_params_usb_pd_rw_hash_entry hash_entry;
	struct ec_params_usb_pd_discovery_entry discovery_entry;
	char *file;
	int ret, port, i;
	struct cros_ec_pd_update_data *drv_data =
		container_of(to_delayed_work(work),
		struct cros_ec_pd_update_data, work);
	struct device *dev = drv_data->dev;

	if (!pd_ec) {
		dev_err(dev, "No pd_ec device found\n");
		return;
	}

	/*
	 * If there is an EC based charger, send a notification to it to
	 * trigger a refresh of the power supply state.
	 */
	if (pd_ec->ec_dev->charger) {
		power_supply_changed(pd_ec->ec_dev->charger);
	}

	/* Received notification, send command to check on PD status. */
	for (port = 0; port < drv_data->num_ports; ++port) {
		ret = cros_ec_pd_get_status(dev,
					    pd_ec,
					    port,
					    &hash_entry,
					    &discovery_entry);
		if (ret < 0) {
			dev_err(dev, "Can't get device status (err:%d)\n",
				ret);
			return;
		} else  {
			if (hash_entry.dev_id == PD_DEVICE_TYPE_NONE)
				i = PD_NO_IMAGE;
			else
				i = find_firmware_image(hash_entry.dev_id,
					discovery_entry.vid,
					discovery_entry.pid);

			/* Device found, should we update firmware? */
			if (i != PD_NO_IMAGE &&
			    memcmp(hash_entry.dev_rw_hash,
				   firmware_images[i].hash,
				   PD_RW_HASH_SIZE) != 0) {
				file = firmware_images[i].filename;
				ret = request_firmware(&fw, file, dev);
				if (ret) {
					dev_err(dev, "Error, can't load file %s\n",
						file);
					continue;
				}

				if (fw->size > PD_RW_IMAGE_SIZE) {
					dev_err(dev, "Firmware file %s is too large\n",
						file);
					goto done;
				}

				/* Update firmware */
				cros_ec_pd_fw_update(dev, pd_ec, fw, port);
done:
				release_firmware(fw);
			} else if (i != PD_NO_IMAGE) {
				/**
				 * Device already has latest firmare. Send
				 * hash entry to EC so we don't get subsequent
				 * FW update requests.
				 */
				cros_ec_pd_send_hash_entry(dev,
							   pd_ec,
							   &firmware_images[i]);
			} else {
				/* Unknown PD device -- don't update FW */
			}
		}
	}
}

/**
 * acpi_cros_ec_pd_notify - Called upon receiving an ACPI event (typically
 * due to PD device insertion). Queue a delayed task to check if a PD
 * device FW update is necessary.
 */
static void acpi_cros_ec_pd_notify(struct acpi_device *acpi_device, u32 event)
{
	struct cros_ec_pd_update_data *drv_data =
		(struct cros_ec_pd_update_data *)
		dev_get_drvdata(&acpi_device->dev);

	if (drv_data)
		queue_delayed_work(drv_data->workqueue, &drv_data->work,
			PD_UPDATE_CHECK_DELAY);
	else
		dev_warn(&acpi_device->dev,
			"ACPI notification skipped due to missing drv_data\n");
}

static int acpi_cros_ec_pd_add(struct acpi_device *acpi_device)
{
	struct cros_ec_pd_update_data *drv_data;
	int ret;

	drv_data =
		devm_kzalloc(&acpi_device->dev, sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data) {
		ret = -ENOMEM;
		goto fail;
	}

	drv_data->dev = &acpi_device->dev;
	INIT_DELAYED_WORK(&drv_data->work, cros_ec_pd_update_check);
	drv_data->workqueue =
		create_singlethread_workqueue("cros_ec_pd_update");
	if (cros_ec_pd_get_num_ports(drv_data->dev,
				     pd_ec,
				     &drv_data->num_ports) < 0) {
		dev_err(drv_data->dev, "Can't get num_ports\n");
		ret = -EINVAL;
		goto fail;
	}
	dev_set_drvdata(&acpi_device->dev, drv_data);

	queue_delayed_work(drv_data->workqueue, &drv_data->work,
		PD_UPDATE_CHECK_DELAY);
	return 0;

fail:
	if (drv_data) {
		dev_set_drvdata(&acpi_device->dev, NULL);
		devm_kfree(&acpi_device->dev, drv_data);
	}
	return ret;
}

static int acpi_cros_ec_pd_resume(struct device *dev)
{
	struct cros_ec_pd_update_data *drv_data =
		(struct cros_ec_pd_update_data *)dev_get_drvdata(dev);

	if (drv_data)
		queue_delayed_work(drv_data->workqueue, &drv_data->work,
			PD_UPDATE_CHECK_DELAY);
	return 0;
}

static int acpi_cros_ec_pd_remove(struct acpi_device *acpi_device)
{
	struct cros_ec_pd_update_data *drv_data =
		(struct cros_ec_pd_update_data *)
		dev_get_drvdata(&acpi_device->dev);

	if (drv_data)
		flush_delayed_work(&drv_data->work);
	return 0;
}

static int acpi_cros_ec_pd_suspend(struct device *dev)
{
	struct cros_ec_pd_update_data *drv_data =
		(struct cros_ec_pd_update_data *)dev_get_drvdata(dev);

	if (drv_data)
		flush_delayed_work(&drv_data->work);
	return 0;
}

static umode_t cros_ec_pd_attrs_are_visible(struct kobject *kobj,
					    struct attribute *a, int n)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cros_ec_dev *ec = container_of(dev, struct cros_ec_dev,
					      class_dev);
	struct ec_params_usb_pd_rw_hash_entry hash_entry;
	struct ec_params_usb_pd_discovery_entry discovery_entry;

	/* Check if a PD MCU is present */
	if (cros_ec_pd_get_status(dev,
				  ec,
				  0,
				  &hash_entry,
				  &discovery_entry) == EC_RES_SUCCESS) {
		/*
		 * Save our ec pointer so we can conduct transactions.
		 * TODO(shawnn): Find a better way to access the ec pointer.
		 */
		if (!pd_ec)
			pd_ec = ec;
		return a->mode;
	}

	return 0;
}

static ssize_t show_firmware_images(struct device *dev,
				    struct device_attribute *attr, char *buf) {
	int size = 0;
	int i;

	for (i = 0; i < firmware_image_count; ++i) {
		if (firmware_images[i].filename == NULL)
			size += scnprintf(buf + size, PAGE_SIZE, "%d: NONE\n",
					  i);
		else
			size += scnprintf(buf + size, PAGE_SIZE, "%d: %s\n", i,
					  firmware_images[i].filename);
	}

	return size;
}


static DEVICE_ATTR(firmware_images, S_IRUGO, show_firmware_images, NULL);

static struct attribute *__pd_attrs[] = {
	&dev_attr_firmware_images.attr,
	NULL,
};

struct attribute_group cros_ec_pd_attr_group = {
	.name = "pd_update",
	.attrs = __pd_attrs,
	.is_visible = cros_ec_pd_attrs_are_visible,
};

/**
 * TODO(shawnn): Find an alternative notification method for devices which
 * don't use ACPI.
 */
static const struct acpi_device_id pd_device_ids[] = {
	{ "GOOG0003", 0 },
	{ }
};

MODULE_DEVICE_TABLE(acpi, pd_device_ids);

static SIMPLE_DEV_PM_OPS(acpi_cros_ec_pd_pm,
	acpi_cros_ec_pd_suspend, acpi_cros_ec_pd_resume);

static struct acpi_driver acpi_cros_ec_pd_driver = {
	.name = "cros_ec_pd_update",
	.class = "cros_ec_pd_update",
	.ids = pd_device_ids,
	.ops = {
		.add = acpi_cros_ec_pd_add,
		.remove = acpi_cros_ec_pd_remove,
		.notify = acpi_cros_ec_pd_notify,
	},
	.drv.pm = &acpi_cros_ec_pd_pm,
};

module_acpi_driver(acpi_cros_ec_pd_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ChromeOS power device FW update driver");
