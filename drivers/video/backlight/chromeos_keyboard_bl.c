/*
 *  chromeos_keyboard_bl.c - Keyboard backlight driver for Chrome OS.
 *
 *  Copyright (C) 2012 The Chromium OS Authors
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/acpi.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* Keyboard Backlight ACPI Device must be defined in firmware */
#define ACPI_KEYBOARD_BACKLIGHT_DEVICE	"\\_SB.KBLT"
#define ACPI_KEYBOARD_BACKLIGHT_READ	ACPI_KEYBOARD_BACKLIGHT_DEVICE ".KBQC"
#define ACPI_KEYBOARD_BACKLIGHT_WRITE	ACPI_KEYBOARD_BACKLIGHT_DEVICE ".KBCM"

static int keyboard_backlight_update_status(struct backlight_device *device)
{
	union acpi_object param;
	struct acpi_object_list input;
	acpi_status status;

	param.type = ACPI_TYPE_INTEGER;
	param.integer.value = (u8)device->props.brightness;
	input.count = 1;
	input.pointer = &param;

	status = acpi_evaluate_object(NULL, ACPI_KEYBOARD_BACKLIGHT_WRITE,
				      &input, NULL);
	if (ACPI_FAILURE(status)) {
		dev_err(&device->dev, "Error setting keyboard backlight value");
		return -1;
	}
	return 0;
}

static int keyboard_backlight_get_brightness(struct backlight_device *device)
{
	unsigned long long brightness;
	acpi_status status;

	status = acpi_evaluate_integer(NULL, ACPI_KEYBOARD_BACKLIGHT_READ,
				       NULL, &brightness);

	if (ACPI_FAILURE(status)) {
		dev_err(&device->dev, "Error reading keyboard backlight value");
		return -1;
	}
	return (int)brightness;
}

static const struct backlight_ops keyboard_backlight_ops = {
	.update_status	= keyboard_backlight_update_status,
	.get_brightness	= keyboard_backlight_get_brightness,
};

static int keyboard_backlight_probe(struct platform_device *pdev)
{
	struct backlight_device *bl;
	struct backlight_properties props = {
		.type = BACKLIGHT_FIRMWARE, .max_brightness = 100
	};
	acpi_handle handle;
	acpi_status status;

	/* Look for the keyboard backlight ACPI Device */
	status = acpi_get_handle(ACPI_ROOT_OBJECT,
				 ACPI_KEYBOARD_BACKLIGHT_DEVICE,
				 &handle);
	if (ACPI_FAILURE(status)) {
		dev_err(&pdev->dev, "Unable fo find ACPI device %s\n",
			ACPI_KEYBOARD_BACKLIGHT_DEVICE);
		return -ENODEV;
	}

	bl = backlight_device_register("keyboard_backlight", &pdev->dev, NULL,
				       &keyboard_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		return PTR_ERR(bl);
	}

	bl->props.brightness = bl->props.max_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;
}

static int keyboard_backlight_remove(struct platform_device *pdev)
{
	backlight_device_unregister(platform_get_drvdata(pdev));
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int keyboard_backlight_suspend(struct device *pdev)
{
	struct backlight_device *bl = dev_get_drvdata(pdev);
	int saved_brightness = bl->props.brightness;

	bl->props.brightness = 0;
	backlight_update_status(bl);
	bl->props.brightness = saved_brightness;
	return 0;
}

static int keyboard_backlight_resume(struct device *pdev)
{
	struct backlight_device *bl = dev_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#endif  /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(keyboard_backlight_pm, keyboard_backlight_suspend,
			 keyboard_backlight_resume);

static struct platform_driver keyboard_backlight_driver = {
	.driver		= {
		.name	= "chromeos-keyboard-backlight",
		.owner	= THIS_MODULE,
		.pm	= &keyboard_backlight_pm,
	},
	.probe		= keyboard_backlight_probe,
	.remove		= keyboard_backlight_remove,
};

module_platform_driver(keyboard_backlight_driver);

MODULE_AUTHOR("Simon Que <sque@chromium.org>");
MODULE_DESCRIPTION("ChromeOS Keyboard Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:chromeos-keyboard-backlight");
