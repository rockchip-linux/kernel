/*
 * LED MULTI-CONTROL
 *
 * Copyright 2017 Allen Zhang <zwp@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LEDS_MULTI_H_INCLUDED
#define __LEDS_MULTI_H_INCLUDED

enum {
	TRIGGER_NONE = 0,
	TRIGGER_DEFAULT_ON,
	TRIGGER_TIMER,
	TRIGGER_ONESHOT,
	TRIGGER_MAX,
};

struct led_ctrl_data {
	unsigned int trigger;
	/* the delay time(ms) of triggering a trigger */
	unsigned int delayed_trigger_ms;
	unsigned int brightness;
	unsigned int delay_on;
	unsigned int delay_off;
} __packed;

#define MAX_LEDS_NUMBER	128

#define LEDS_MULTI_CTRL_IOCTL_MAGIC	'z'

#define LEDS_MULTI_CTRL_IOCTL_MULTI_SET	\
		_IOW(LEDS_MULTI_CTRL_IOCTL_MAGIC, 0x01, struct led_ctrl_data*)
#define LEDS_MULTI_CTRL_IOCTL_GET_LED_NUMBER	\
		_IOR(LEDS_MULTI_CTRL_IOCTL_MAGIC, 0x02, int)

int led_multi_control_register(struct led_classdev *led_cdev);
int led_multi_control_unregister(struct led_classdev *led_cdev);
int led_multi_control_init(struct device *dev);
int led_multi_control_exit(struct device *dev);

#endif	/* __LEDS_MULTI_H_INCLUDED */
