/*
 * wakeup.c - support wakeup devices
 * Copyright (C) 2004 Li Shaohua <shaohua.li@intel.com>
 */

#include <linux/init.h>
#include <linux/acpi.h>
#include <linux/kernel.h>
#include <linux/pm.h>
#include <linux/types.h>

#include "internal.h"
#include "sleep.h"

/*
 * We didn't lock acpi_device_lock in the file, because it invokes oops in
 * suspend/resume and isn't really required as this is called in S-state. At
 * that time, there is no device hotplug
 **/
#define _COMPONENT		ACPI_SYSTEM_COMPONENT
ACPI_MODULE_NAME("wakeup_devices")

/**
 * acpi_enable_wakeup_devices - Enable wake-up device GPEs.
 * @sleep_state: ACPI system sleep state.
 *
 * Enable wakeup device power of devices with the state.enable flag set and set
 * the wakeup enable mask bits in the GPE registers that correspond to wakeup
 * devices.
 */
void acpi_enable_wakeup_devices(u8 sleep_state)
{
	struct list_head *node, *next;

	list_for_each_safe(node, next, &acpi_wakeup_device_list) {
		struct acpi_device *dev =
			container_of(node, struct acpi_device, wakeup_list);

		if (!dev->wakeup.flags.valid
		    || sleep_state > (u32) dev->wakeup.sleep_state
		    || !(device_may_wakeup(&dev->dev)
		        || dev->wakeup.prepare_count))
			continue;

		if (device_may_wakeup(&dev->dev))
			acpi_enable_wakeup_device_power(dev, sleep_state);

		/* The wake-up power should have been enabled already. */
		acpi_set_gpe_wake_mask(dev->wakeup.gpe_device,
				dev->wakeup.wake_numbers.gpe,
				ACPI_GPE_ENABLE);
	}
}

/**
 * acpi_disable_wakeup_devices - Disable devices' wakeup capability.
 * @sleep_state: ACPI system sleep state.
 */
void acpi_disable_wakeup_devices(u8 sleep_state)
{
	struct list_head *node, *next;

	list_for_each_safe(node, next, &acpi_wakeup_device_list) {
		struct acpi_device *dev =
			container_of(node, struct acpi_device, wakeup_list);

		if (!dev->wakeup.flags.valid
		    || sleep_state > (u32) dev->wakeup.sleep_state
		    || !(device_may_wakeup(&dev->dev)
		        || dev->wakeup.prepare_count))
			continue;

		acpi_set_gpe_wake_mask(dev->wakeup.gpe_device,
				dev->wakeup.wake_numbers.gpe,
				ACPI_GPE_DISABLE);

		if (device_may_wakeup(&dev->dev))
			acpi_disable_wakeup_device_power(dev);
	}
}

/**
 * apci_get_wakeup_data - returns the wakeup data.
 *
 * The wakeup data includes the _SWS indices for the _SB and the _GPE scope.
 * These are used to figure out which wakeup source woke the system.
 */
static void *acpi_get_wakeup_data(void)
{
	struct acpi_wakeup_data *wakeup_data;
	acpi_status status;
	u64 sb, gpe;

	status = acpi_evaluate_integer(NULL, "\\_SB._SWS", NULL, &sb);
	if (ACPI_FAILURE(status))
		return NULL;

	status = acpi_evaluate_integer(NULL, "\\_GPE._SWS", NULL, &gpe);
	if (ACPI_FAILURE(status))
		return NULL;

	wakeup_data = kmalloc(sizeof(*wakeup_data), GFP_KERNEL);
	if (!wakeup_data)
		return NULL;

	wakeup_data->sb = sb;
	wakeup_data->gpe = gpe;

	return wakeup_data;
}

/**
 * acpi_put_wakeup_data - frees the acpi wakeup data from acpi_get_wakeup_data
 * @data: acpi wakeup data to free
 */
static void acpi_put_wakeup_data(void *data)
{
	kfree(data);
}

/**
 * acpi_match_wakeup_data - Figures out if the wakeup source woke the system
 * @platform_data: pointer to acpi wakeup data returned by acpi_get_wakeup_data
 * @wakeup_data: acpi wakeup data for a wakeup source
 *
 * Checks if wakeup_data matches the platform_data. This does not do exact
 * matches. If the _SB scope is set, there only has to be a match with that
 * index.
 */
static bool acpi_match_wakeup_data(void *platform_data, void *wakeup_data)
{
	struct acpi_wakeup_data *wake_data = wakeup_data;
	struct acpi_wakeup_data *plat_data = platform_data;
	if (!wake_data || !plat_data)
		return false;

	/* The SB scope takes precedence over the GPE scope. This is kind of
	 * arbitrary, but the power button (included in SB scope) should
	 * take precedence over everything. This covers the case where two
	 * devices such as the power button and some gpe device are both set as
	 * wakeup devices. */
	return wake_data->sb == plat_data->sb &&
	       (plat_data->sb != (u64)-1 || wake_data->gpe == plat_data->gpe);
}

static struct platform_wakeup_source_ops acpi_wakeup_ops = {
	.get = acpi_get_wakeup_data,
	.put = acpi_put_wakeup_data,
	.match = acpi_match_wakeup_data,
};

int __init acpi_wakeup_device_init(void)
{
	struct list_head *node, *next;

	mutex_lock(&acpi_device_lock);
	list_for_each_safe(node, next, &acpi_wakeup_device_list) {
		struct acpi_device *dev = container_of(node,
						       struct acpi_device,
						       wakeup_list);
		if (device_can_wakeup(&dev->dev)) {
			/* Button GPEs are supposed to be always enabled. */
			acpi_enable_gpe(dev->wakeup.gpe_device,
					dev->wakeup.wake_numbers.gpe);
			device_set_wakeup_enable(&dev->dev, true);
		}
	}
	mutex_unlock(&acpi_device_lock);

	wakeup_register_platform_ops(&acpi_wakeup_ops);

	return 0;
}
