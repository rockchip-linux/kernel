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
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_pd_update.h>
#include <linux/module.h>

#include "cros_ec_dev.h"

/* Store our PD device pointer so we can send update-related commands. */
static struct cros_ec_dev *pd_ec;

/*
 * $DEVICE_known_update_hashes - A list of old known RW hashes from which we
 * wish to upgrade. When firmware_images is updated, the old hash should
 * probably be added here. The latest hash currently in firmware_images should
 * NOT appear here.
 */
static uint8_t zinger_known_update_hashes[][PD_RW_HASH_SIZE] = {
	/* zinger_v1.7.262-9a5b8f4.bin */
	{ 0x05, 0x94, 0xb8, 0x97, 0x8a,
	  0x9a, 0xa0, 0x0a, 0x71, 0x07,
	  0x37, 0xba, 0x8f, 0x4c, 0x01,
	  0xe6, 0x45, 0x6d, 0xb0, 0x01 },
};

/*
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
		.filename = "cros-pd/zinger_v1.7.509-e5bffd3.bin",
		.hash = { 0x02, 0xad, 0x4c, 0x95, 0x25,
			  0x89, 0xe5, 0xe7, 0x1e, 0xc6,
			  0xaf, 0x9c, 0x0e, 0xaa, 0xbb,
			  0x6c, 0xa7, 0x52, 0x8c, 0x3a },
		.update_hashes = &zinger_known_update_hashes,
		.update_hash_count = ARRAY_SIZE(zinger_known_update_hashes),
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

	ret = cros_ec_cmd_xfer_status(pd_dev->ec_dev, &msg);
	return ret >= 0 ? EC_RES_SUCCESS : ret;
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
		dev_err(dev, "Unable to reboot PD (err:%d)\n", ret);
		return ret;
	}

	/*
	 * Wait for the charger to reboot.
	 * TODO(shawnn): Instead of waiting for a fixed period of time, wait
	 * to recieve an interrupt that signals the charger is back online.
	 */
	msleep(4000);

	/* Erase RW flash */
	pd_cmd->cmd = USB_PD_FW_FLASH_ERASE;
	pd_cmd->size = 0;
	ret = cros_ec_pd_send_fw_update_cmd(dev, pd_dev, pd_cmd);
	if (ret < 0) {
		dev_err(dev, "Unable to erase PD RW flash (err:%d)\n", ret);
		return ret;
	}

	/* Wait 3 seconds for the PD peripheral to finalize RW erase */
	msleep(3000);

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
		/*
		 * TODO(crosbug.com/p/33905): Throttle the update process so
		 * that the EC doesn't trip its WDT. Remove this delay once
		 * the root cause is resolved.
		 */
		usleep_range(10000, 10500);
	}

	/* Wait 100ms to guarantee that writes finish */
	msleep(100);

	/* Reboot PD into new RW */
	pd_cmd->cmd = USB_PD_FW_REBOOT;
	pd_cmd->size = 0;
	ret = cros_ec_pd_send_fw_update_cmd(dev, pd_dev, pd_cmd);
	if (ret < 0) {
		dev_err(dev, "Unable to reboot PD post-flash (err:%d)\n", ret);
		return ret;
	}

	return 0;
}

/**
 * cros_ec_find_update_firmware - Search firmware image table for an image
 * matching the passed attributes, then decide whether an update should
 * be performed.
 * Returns PD_DO_UPDATE if an update should be performed, and writes the
 * firmware_image pointer to update_image.
 * Returns reason for not updating otherwise.
 *
 * @dev: PD device
 * @hash_entry: Pre-filled hash entry struct for matching
 * @discovery_entry: Pre-filled discovery entry struct for matching
 * @update_image: Stores update firmware image on success
 */
static enum cros_ec_pd_find_update_firmware_result cros_ec_find_update_firmware(
	struct device *dev,
	struct ec_params_usb_pd_rw_hash_entry *hash_entry,
	struct ec_params_usb_pd_discovery_entry *discovery_entry,
	const struct cros_ec_pd_firmware_image **update_image)
{
	const struct cros_ec_pd_firmware_image *img;
	int i;

	if (hash_entry->dev_id == PD_DEVICE_TYPE_NONE)
		return PD_UNKNOWN_DEVICE;

	/*
	 * Search for a matching firmware update image.
	 * TODO(shawnn): Replace sequential table search with modified binary
	 * search on major / minor.
	 */
	for (i = 0; i < firmware_image_count; ++i) {
		img = &firmware_images[i];
		if (MAJOR_MINOR_TO_DEV_ID(img->id_major, img->id_minor)
					  == hash_entry->dev_id &&
		    img->usb_vid == discovery_entry->vid &&
		    img->usb_pid == discovery_entry->pid)
			break;
	}

	if (i == firmware_image_count)
		return PD_UNKNOWN_DEVICE;
	else if (memcmp(hash_entry->dev_rw_hash, img->hash, PD_RW_HASH_SIZE)
			== 0)
		return PD_ALREADY_HAVE_LATEST;

	/* Always update if PD device is stuck in RO. */
	if (hash_entry->current_image != EC_IMAGE_RW) {
		*update_image = img;
		dev_info(dev, "Updating FW since PD dev is in RO\n");
		return PD_DO_UPDATE;
	}

	dev_info(dev, "Considering upgrade from existing RW: %x %x %x %x\n",
		 hash_entry->dev_rw_hash[0],
		 hash_entry->dev_rw_hash[1],
		 hash_entry->dev_rw_hash[2],
		 hash_entry->dev_rw_hash[3]);

	/* Verify RW is a known update image so we don't roll-back. */
	for (i = 0; i < img->update_hash_count; ++i)
		if (memcmp(hash_entry->dev_rw_hash,
			   (*img->update_hashes)[i],
			   PD_RW_HASH_SIZE) == 0) {
			*update_image = img;
			dev_info(dev, "Updating FW since RW is known\n");
			return PD_DO_UPDATE;
		}

	dev_info(dev, "Skipping FW update since RW is unknown\n");
	return PD_UNKNOWN_RW;
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
	const struct cros_ec_pd_firmware_image *img;
	const struct firmware *fw;
	struct ec_response_host_event_status host_event_status;
	struct ec_params_usb_pd_rw_hash_entry hash_entry;
	struct ec_params_usb_pd_discovery_entry discovery_entry;
	struct cros_ec_pd_update_data *drv_data =
		container_of(to_delayed_work(work),
		struct cros_ec_pd_update_data, work);
	struct device *dev = drv_data->dev;
	enum cros_ec_pd_find_update_firmware_result result;
	int ret, port;

	dev_dbg(dev, "Checking for updates\n");

	if (!pd_ec) {
		dev_err(dev, "No pd_ec device found\n");
		return;
	}

	/* Check for host events on EC. */
	ret = cros_ec_pd_command(dev, pd_ec, EC_CMD_PD_HOST_EVENT_STATUS,
				 NULL, 0,
				 (uint8_t *)&host_event_status,
				 sizeof(host_event_status));
	if (ret) {
		dev_err(dev, "Can't get host event status (err: %d)\n", ret);
		return;
	}
	dev_dbg(dev, "Got host event status %x\n", host_event_status.status);

	/*
	 * Override status received from EC if update is forced, such as
	 * after power-on or after resume.
	 */
	if (drv_data->force_update) {
		host_event_status.status = PD_EVENT_POWER_CHANGE |
					   PD_EVENT_UPDATE_DEVICE;
		drv_data->force_update = 0;
	}

	/*
	 * If there is an EC based charger, send a notification to it to
	 * trigger a refresh of the power supply state.
	 */
	if ((host_event_status.status & PD_EVENT_POWER_CHANGE) &&
	    pd_ec->ec_dev->charger)
		power_supply_changed(pd_ec->ec_dev->charger);

	if (!(host_event_status.status & PD_EVENT_UPDATE_DEVICE))
		return;

	/* Received notification, send command to check on PD status. */
	for (port = 0; port < drv_data->num_ports; ++port) {
		ret = cros_ec_pd_get_status(dev, pd_ec, port, &hash_entry,
					    &discovery_entry);
		if (ret < 0) {
			dev_err(dev, "Can't get device status (err:%d)\n",
				ret);
			return;
		}

		result = cros_ec_find_update_firmware(dev,
						      &hash_entry,
						      &discovery_entry,
						      &img);
		dev_dbg(dev, "Find FW result: %d\n", result);

		switch (result) {
		case PD_DO_UPDATE:
			if (request_firmware(&fw, img->filename, dev)) {
				dev_err(dev, "Error, can't load file %s\n",
					img->filename);
				break;
			}

			if (fw->size > PD_RW_IMAGE_SIZE) {
				dev_err(dev, "Firmware file %s is too large\n",
					img->filename);
				goto done;
			}

			/* Update firmware */
			dev_info(dev, "Updating RW to %s\n", img->filename);
			cros_ec_pd_fw_update(dev, pd_ec, fw, port);
			dev_info(dev, "FW update completed\n");
done:
			release_firmware(fw);
			break;
		case PD_ALREADY_HAVE_LATEST:
			/*
			 * Device already has latest firmare. Send hash entry
			 * to EC so we don't get subsequent FW update requests.
			 */
			cros_ec_pd_send_hash_entry(dev, pd_ec, img);
			break;
		case PD_UNKNOWN_DEVICE:
		case PD_UNKNOWN_RW:
			/* Unknown PD device or RW -- don't update FW */
			break;
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
	int ret, i;

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
	drv_data->force_update = 1;
	dev_set_drvdata(&acpi_device->dev, drv_data);

	/*
	 * Send list of update FW hashes to PD MCU.
	 * TODO(crosbug.com/p/35510): This won't scale past four update
	 * devices. Find a better solution once we get there.
	 */
	for (i = 0; i < firmware_image_count; ++i)
		cros_ec_pd_send_hash_entry(drv_data->dev,
					   pd_ec,
					   &firmware_images[i]);

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

	if (drv_data) {
		drv_data->force_update = 1;
		queue_delayed_work(drv_data->workqueue, &drv_data->work,
			PD_UPDATE_CHECK_DELAY);
	}
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
