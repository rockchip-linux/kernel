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

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/slab.h>

/* I/O addresses for LPC commands */
#define EC_ADDR_DATA          0x62
#define EC_ADDR_CMD           0x66
#define EC_ADDR_PARAM        0x800
#define EC_PARAM_SIZE          128  /* Size of each param area in bytes */

/*
 * LPC command status byte masks
 * Host has written a command/data byte and the EC hasn't read it yet
 */
#define EC_LPC_STATUS_FROM_HOST   0x02
/* EC is processing a command */
#define EC_LPC_STATUS_PROCESSING  0x04

/* EC is busy.  This covers both the EC processing a command, and the host has
 * written a new command but the EC hasn't picked it up yet. */
#define EC_LPC_STATUS_BUSY_MASK \
	(EC_LPC_STATUS_FROM_HOST | EC_LPC_STATUS_PROCESSING)

/* EC PWM commands */
#define EC_CMD_PWM_GET_KEYBOARD_BACKLIGHT 0x22
#define EC_CMD_PWM_SET_KEYBOARD_BACKLIGHT 0x23

/* Waits for the EC to be unbusy.  Returns 0 if unbusy, non-zero if
 * timeout. */
static int wait_for_ec(void)
{
	int i;
	for (i = 0; i < 1000000; i += 10) {
		udelay(10);  /* Delay first, in case we just sent a command */
		if (!(inb(EC_ADDR_CMD) & EC_LPC_STATUS_BUSY_MASK))
			return 0;
	}
	return -1;  /* Timeout */
}

/* Sends a command to the EC.  Returns the command status code, or
 * -1 if other error. */
static int ec_command(int command, const u8 *indata, int insize,
		      u8 *outdata, int outsize)
{
	int i;

	if (insize > EC_PARAM_SIZE || outsize > EC_PARAM_SIZE) {
		printk(KERN_ERR "Data size too big\n");
		return -1;
	}

	if (wait_for_ec()) {
		printk(KERN_ERR "Timeout waiting for EC ready\n");
		return -1;
	}

	/* Write data, if any */
	/* TODO: optimized copy using outl() */
	for (i = 0; i < insize; i++)
		outb(indata[i], EC_ADDR_PARAM + i);

	outb(command, EC_ADDR_CMD);

	if (wait_for_ec()) {
		printk(KERN_ERR "Timeout waiting for EC response\n");
		return -1;
	}

	/* Check result */
	i = inb(EC_ADDR_DATA);
	if (i) {
		printk(KERN_ERR "EC returned error result code %d\n", i);
		return i;
	}

	/* Read data, if any */
	/* TODO: optimized copy using outl() */
	for (i = 0; i < outsize; i++)
		outdata[i] = inb(EC_ADDR_PARAM + i);

	return 0;
}

static int keyboard_backlight_update_status(struct backlight_device *device)
{
	u8 brightness = device->props.brightness;
	int ret = ec_command(EC_CMD_PWM_SET_KEYBOARD_BACKLIGHT, &brightness,
			     sizeof(brightness), NULL, 0);
	if (ret) {
		printk(KERN_ERR "Error setting keyboard backlight value.");
		return ret;
	}
	return 0;
}

static int keyboard_backlight_get_brightness(struct backlight_device *device)
{
	u8 brightness;
	int ret = ec_command(EC_CMD_PWM_GET_KEYBOARD_BACKLIGHT, NULL, 0,
			     &brightness, sizeof(brightness));
	if (ret) {
		printk(KERN_ERR "Error reading keyboard backlight value.");
		return -1;
	}
	return brightness;
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
