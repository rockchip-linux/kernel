/*
 * IMG pinctrl driver header
 *
 * Copyright (C) 2014 Imagination Technologies Ltd.
 *
 * Author: Damien Horsley <Damien.Horsley@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#ifndef __PINCTRL_IMG_H__
#define __PINCTRL_IMG_H__

enum img_pinconf_param {
	IMG_PINCONF_PARAM_PULL = 0,
	IMG_PINCONF_PARAM_DRIVE_STRENGTH,
	IMG_PINCONF_PARAM_SLEW,
	IMG_PINCONF_PARAM_SCHMITT,
	IMG_PINCONF_PARAM_FORCE_IO,
	IMG_PINCONF_NUM_CONFIGS
};

enum img_pinconf_pull {
	IMG_PINCONFIG_PULL_HIGH_IMPEDANCE = 0,
	IMG_PINCONFIG_PULL_UP,
	IMG_PINCONFIG_PULL_DOWN,
	IMG_PINCONFIG_PULL_REPEATER
};

enum img_pinconf_drive_strength {
	IMG_PINCONFIG_DRIVE_2MA = 0,
	IMG_PINCONFIG_DRIVE_4MA,
	IMG_PINCONFIG_DRIVE_8MA,
	IMG_PINCONFIG_DRIVE_12MA
};

enum img_pinconf_slew {
	IMG_PINCONF_SLEW_SLOW = 0,
	IMG_PINCONF_SLEW_FAST
};

enum img_pinconf_schmitt {
	IMG_PINCONF_SCHMITT_NONE = 0,
	IMG_PINCONF_SCHMITT_ACTIVE
};

#define IMG_PINCONF_PACK(param, arg)	((param) << 16 | (arg))
#define IMG_PINCONF_UNPACK_PARAM(conf)	((conf) >> 16)
#define IMG_PINCONF_UNPACK_ARG(conf)	((conf) & 0xffff)

struct img_pinctrl_function {
	const char *name;
	const char * const *groups;
	unsigned int ngroups;
};

#define IMG_MFIO_REG_OFFSET(mfio_reg_data)	((mfio_reg_data) & 0xFFFF)
#define IMG_MFIO_REG_SHIFT(mfio_reg_data)	((mfio_reg_data >> 16) & 0xFF)
#define IMG_MFIO_REG_BITS(mfio_reg_data)	((mfio_reg_data >> 24) & 0xFF)

struct img_pinctrl_group {
	const char *name;
	const unsigned int *pins;
	const unsigned int npins;
	const unsigned int *selects;
	const unsigned int scenario_mask;
	const unsigned int scenario_val;
};

struct img_gpio_irqdata {
	struct img_pinctrl *pc;
	int bank;
	int irq;
};

struct img_pinctrl_soc_data {
	const struct pinctrl_pin_desc *pins;
	unsigned int num_pins;
	unsigned int num_gpios;
	unsigned int num_mfios;
	const struct img_pinctrl_function *functions;
	unsigned int num_functions;
	const struct img_pinctrl_group *groups;
	unsigned int num_groups;
	struct img_gpio_irqdata *gpio_irq_data;
	unsigned int num_gpio_banks;
	unsigned int *mfio_reg_ctrl;
};

int img_pinctrl_probe(struct platform_device *pdev,
		const struct img_pinctrl_soc_data *soc_data);

int img_pinctrl_remove(struct platform_device *pdev);

#endif
