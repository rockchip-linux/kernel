/*
* Copyright (C) 2012 Texas Instruments
*
* License Terms: GNU General Public License v2
*
* Simple driver for Texas Instruments LM3648 LED driver chip
*
* Author: G.Shark Jeong <gshark.jeong@gmail.com>
*         Daniel Jeong <daniel.jeong@ti.com>
*/

#ifndef __LINUX_LM3648_H
#define __LINUX_LM3648_H

#include <linux/of_gpio.h>

#define LM3648_NAME "leds-lm3648"

#define OF_LM3648_GPIO_STROBE "lm3648,strobe-gpio"
#define OF_LM3648_GPIO_HWEN "lm3648,hwen-gpio"

enum lm3648_torch_pin_enable {
	LM3648_TORCH_PIN_DISABLE = 0x00,
	LM3648_TORCH_PIN_ENABLE = 0x10,
};

enum lm3648_strobe_pin_enable {
	LM3648_STROBE_PIN_DISABLE = 0x00,
	LM3648_STROBE_PIN_ENABLE = 0x20,
};

enum lm3648_tx_pin_enable {
	LM3648_TX_PIN_DISABLE = 0x00,
	LM3648_TX_PIN_ENABLE = 0x40,
};

enum lm3648_platform_pin_state {
	PLTFRM_PIN_STATE_INACTIVE = 0,
	PLTFRM_PIN_STATE_ACTIVE = 1
};

struct lm3648_platform_goio {
	int pltfrm_gpio;
	const char *label;
	enum of_gpio_flags active_low;
};
#endif /* __LINUX_LM3648_H */
