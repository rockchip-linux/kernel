/*
 *  leds-chromeos-keyboard.c - Keyboard backlight LED driver for Chrome OS.
 *
 *  Copyright (C) 2012 Google, Inc.
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
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* Keyboard LED ACPI Device must be defined in firmware */
#define ACPI_KEYBOARD_BACKLIGHT_DEVICE	"\\_SB.KBLT"
#define ACPI_KEYBOARD_BACKLIGHT_READ	ACPI_KEYBOARD_BACKLIGHT_DEVICE ".KBQC"
#define ACPI_KEYBOARD_BACKLIGHT_WRITE	ACPI_KEYBOARD_BACKLIGHT_DEVICE ".KBCM"

#define ACPI_KEYBOARD_BACKLIGHT_MAX		100

static void keyboard_led_set_brightness(struct led_classdev *cdev,
	enum led_brightness brightness)
{
	union acpi_object param;
	struct acpi_object_list input;
	acpi_status status;

	if (!(cdev->flags & LED_SUSPENDED))
		cdev->brightness = brightness;

	param.type = ACPI_TYPE_INTEGER;
	param.integer.value = brightness;
	input.count = 1;
	input.pointer = &param;

	status = acpi_evaluate_object(NULL, ACPI_KEYBOARD_BACKLIGHT_WRITE,
					  &input, NULL);
	if (ACPI_FAILURE(status))
		dev_err(cdev->dev, "Error setting keyboard LED value");
}

static int keyboard_led_probe(struct platform_device *pdev)
{
	struct led_classdev *cdev;
	acpi_handle handle;
	acpi_status status;
	int ret;

	/* Look for the keyboard LED ACPI Device */
	status = acpi_get_handle(ACPI_ROOT_OBJECT,
				 ACPI_KEYBOARD_BACKLIGHT_DEVICE,
				 &handle);
	if (ACPI_FAILURE(status)) {
		dev_err(&pdev->dev, "Unable fo find ACPI device %s\n",
			ACPI_KEYBOARD_BACKLIGHT_DEVICE);
		return -ENODEV;
	}

	cdev = kzalloc(sizeof(struct led_classdev), GFP_KERNEL);
	if (!cdev)
		return -ENOMEM;
	cdev->name = "chromeos::kbd_backlight";
	cdev->brightness_set = keyboard_led_set_brightness;
	cdev->max_brightness = ACPI_KEYBOARD_BACKLIGHT_MAX;
	cdev->brightness = cdev->max_brightness;
	cdev->flags |= LED_CORE_SUSPENDRESUME;

	ret = led_classdev_register(&pdev->dev, cdev);
	if (ret)
		goto err;

	platform_set_drvdata(pdev, cdev);
	return 0;
err:
	kfree(cdev);
	return ret;
}

static int keyboard_led_remove(struct platform_device *pdev)
{
	struct led_classdev *cdev = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	kfree(cdev);
	return 0;
}

static struct platform_driver keyboard_led_driver = {
	.driver		= {
		.name	= "chromeos-keyboard-leds",
		.owner	= THIS_MODULE,
	},
	.probe		= keyboard_led_probe,
	.remove		= keyboard_led_remove,
};

module_platform_driver(keyboard_led_driver);

MODULE_AUTHOR("Simon Que <sque@chromium.org>");
MODULE_DESCRIPTION("ChromeOS Keyboard LED Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:chromeos-keyboard-leds");
