/*
 * IMG pinctrl driver
 *
 * Copyright (C) 2014 Imagination Technologies Ltd.
 *
 * Author: Damien Horsley <Damien.Horsley@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/bitmap.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#include "pinctrl-img.h"
#include "core.h"

#define IMG_PINCONF_SCHMITT_EN		(0x0000)
#define IMG_PINCONF_IE			(0x0020)
#define IMG_PINCONF_PU_PD		(0x0040)
#define IMG_PINCONF_OE			(0x0080)
#define IMG_PINCONF_OE_STS		(0x00A0)
#define IMG_PINCONF_FUNCTION_SELECT	(0x00C0)
#define IMG_PINCONF_SCENARIO		(0x00F8)
#define IMG_PINCONF_SLEW_RATE		(0x0100)
#define IMG_PINCONF_DRIVE_STRENGTH	(0x0120)

#define IMG_GPIO_BIT_EN			(0x0200)
#define IMG_GPIO_OUTPUT_EN		(0x0204)
#define IMG_GPIO_OUTPUT			(0x0208)
#define IMG_GPIO_INPUT			(0x020C)
#define IMG_GPIO_INPUT_POLARITY		(0x0210)
#define IMG_GPIO_INTERRUPT_STATUS_TYPE	(0x0214)
#define IMG_GPIO_INTERRUPT_EDGE_SEL	(0x0218)
#define IMG_GPIO_INTERRUPT_EN		(0x021C)
#define IMG_GPIO_INTERRUPT_STATUS	(0x0220)
#define IMG_GPIO_STRIDE			(0x24)

struct img_pinctrl {
	struct device *dev;
	void __iomem *base;
	struct pinctrl_dev *pctl_dev;
	struct irq_domain *irq_domain;
	struct gpio_chip *gpio_chip;
	struct img_gpio_irqdata *gpio_irq_data;
	const struct img_pinctrl_soc_data *soc_data;
	unsigned int num_gpio_banks;
	spinlock_t lock;
};

static inline unsigned int img_pinconf_read(struct img_pinctrl *pc,
	unsigned int pin, unsigned int reg_offset, unsigned int confbits)
{
	u32 reg, tmp;
	void __iomem *base;
	unsigned int ret;

	tmp = (32 / confbits);
	base = pc->base + reg_offset + ((pin / tmp) * 4);

	reg = readl(base);
	ret = ((reg >> ((pin % tmp) * confbits)) & ((1UL << confbits) - 1));

	return ret;
}

static inline void img_pinconf_write(struct img_pinctrl *pc, unsigned int pin,
	unsigned int reg_offset, unsigned int confbits, unsigned int val)
{
	u32 tmp, shift, mask, reg;
	void __iomem *base;

	tmp = (32 / confbits);
	base = pc->base + reg_offset + ((pin / tmp) * 4);
	shift = (pin % tmp) * confbits;
	mask = ((1UL << confbits) - 1) << shift;

	reg = readl(base);
	reg = (reg & ~mask) | ((val << shift) & mask);
	writel(reg, base);
}

static inline int img_gpio_read(struct img_pinctrl *pc, unsigned int pin,
				unsigned int reg_offset)
{
	u32 reg;
	void __iomem *base;
	int ret;

	base = pc->base + reg_offset + ((pin / 16) * IMG_GPIO_STRIDE);

	reg = readl(base);
	ret = (reg >> (pin % 16)) & 0x1;

	return ret;
}

static inline void img_gpio_write(struct img_pinctrl *pc, unsigned int pin,
				unsigned int reg_offset, unsigned int val)
{
	void __iomem *base;
	u32 reg;

	base = pc->base + reg_offset + ((pin / 16) * IMG_GPIO_STRIDE);
	reg = (0x10000 | val) << (pin % 16);

	writel(reg, base);
}

static inline unsigned int img_mfio_read(struct img_pinctrl *pc,
					unsigned int mfio_reg_data)
{
	u32 mask, reg;
	void __iomem *base;
	unsigned int ret;

	base = pc->base + IMG_MFIO_REG_OFFSET(mfio_reg_data);
	mask = (1UL << IMG_MFIO_REG_BITS(mfio_reg_data)) - 1;

	reg = readl(base);
	ret = (reg >> IMG_MFIO_REG_SHIFT(mfio_reg_data)) & mask;

	return ret;
}

static inline void img_mfio_write(struct img_pinctrl *pc,
		unsigned int mfio_reg_data, unsigned int select)
{
	u32 shift, mask, reg;
	void __iomem *base;

	base = pc->base + IMG_MFIO_REG_OFFSET(mfio_reg_data);
	shift = IMG_MFIO_REG_SHIFT(mfio_reg_data);
	mask = ((1UL << IMG_MFIO_REG_BITS(mfio_reg_data)) - 1) << shift;

	reg = readl(base);
	reg = (reg & ~mask) | ((select << shift) & mask);
	writel(reg, base);
}

static int img_gpio_request(struct gpio_chip *chip, unsigned int pin)
{
	return pinctrl_request_gpio(pin);
}

static void img_gpio_free(struct gpio_chip *chip, unsigned int pin)
{
	pinctrl_free_gpio(pin);
}

static int img_gpio_dir_input(struct gpio_chip *chip, unsigned int pin)
{
	return pinctrl_gpio_direction_input(pin);
}

static int img_gpio_get(struct gpio_chip *chip, unsigned int pin)
{
	struct img_pinctrl *pc = dev_get_drvdata(chip->dev);

	return img_gpio_read(pc, pin, IMG_GPIO_INPUT);
}

static void img_gpio_set(struct gpio_chip *chip, unsigned int pin, int value)
{
	struct img_pinctrl *pc = dev_get_drvdata(chip->dev);

	img_gpio_write(pc, pin, IMG_GPIO_OUTPUT, value);
}

static int img_gpio_dir_output(struct gpio_chip *chip,
		unsigned int pin, int value)
{
	struct img_pinctrl *pc = dev_get_drvdata(chip->dev);

	img_gpio_write(pc, pin, IMG_GPIO_OUTPUT, value);

	return pinctrl_gpio_direction_output(chip->base + pin);
}

static int img_gpio_to_irq(struct gpio_chip *chip, unsigned int pin)
{
	struct img_pinctrl *pc = dev_get_drvdata(chip->dev);

	return irq_linear_revmap(pc->irq_domain, pin);
}

static struct gpio_chip img_gpio_chip = {
	.label = "pinctrl-img-gpio",
	.owner = THIS_MODULE,
	.request = img_gpio_request,
	.free = img_gpio_free,
	.direction_input = img_gpio_dir_input,
	.direction_output = img_gpio_dir_output,
	.get = img_gpio_get,
	.set = img_gpio_set,
	.to_irq = img_gpio_to_irq,
	.base = 0,
	.can_sleep = 0
};

static irqreturn_t img_gpio_irq_handler(int irq, void *dev_id)
{
	struct img_gpio_irqdata *irqdata = dev_id;
	struct img_pinctrl *pc = irqdata->pc;
	unsigned int pin, bank = irqdata->bank;
	u32 edge, bit;
	unsigned long events;
	void __iomem *base = pc->base + (bank * IMG_GPIO_STRIDE);

	events = readl(base + IMG_GPIO_INTERRUPT_STATUS) & 0xFFFF;
	events &= readl(base + IMG_GPIO_INTERRUPT_EN);
	edge = readl(base + IMG_GPIO_INTERRUPT_STATUS_TYPE) & 0xFFFF;

	for_each_set_bit(bit, &events, 32) {

		pin = bit + (bank * 16);

		/* ack edge triggered IRQs immediately */
		if (edge & (1UL << bit))
			img_gpio_write(pc, pin, IMG_GPIO_INTERRUPT_STATUS, 0);

		generic_handle_irq(irq_linear_revmap(pc->irq_domain, pin));
	}

	return events ? IRQ_HANDLED : IRQ_NONE;
}

static void img_gpio_irq_enable(struct irq_data *data)
{
	struct img_pinctrl *pc = irq_data_get_irq_chip_data(data);
	unsigned int gpio = irqd_to_hwirq(data);

	img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_EN, 1);
}

static void img_gpio_irq_disable(struct irq_data *data)
{
	struct img_pinctrl *pc = irq_data_get_irq_chip_data(data);
	unsigned int gpio = irqd_to_hwirq(data);

	img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_EN, 0);
}

static unsigned int img_gpio_irq_startup(struct irq_data *data)
{
	struct img_pinctrl *pc = irq_data_get_irq_chip_data(data);
	unsigned int gpio = irqd_to_hwirq(data);
	int ret;

	ret = pinctrl_request_gpio(gpio);
	if (ret)
		return ret;

	ret = pinctrl_gpio_direction_input(gpio);
	if (ret)
		pinctrl_free_gpio(gpio);
	else
		img_gpio_irq_enable(data);

	return ret;
}

static void img_gpio_irq_shutdown(struct irq_data *data)
{
	struct img_pinctrl *pc = irq_data_get_irq_chip_data(data);
	unsigned int gpio = irqd_to_hwirq(data);

	img_gpio_irq_disable(data);

	pinctrl_free_gpio(gpio);
}

static int img_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct img_pinctrl *pc = irq_data_get_irq_chip_data(data);
	unsigned int gpio, bank;
	int ret = 0;

	gpio = irqd_to_hwirq(data);
	bank = gpio / 16;

	img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_EN, 0);

	switch (type) {
	case IRQ_TYPE_NONE:
		break;
	case IRQ_TYPE_EDGE_RISING:
		img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_STATUS_TYPE, 1);
		img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_EDGE_SEL, 0);
		img_gpio_write(pc, gpio, IMG_GPIO_INPUT_POLARITY, 1);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_STATUS_TYPE, 1);
		img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_EDGE_SEL, 0);
		img_gpio_write(pc, gpio, IMG_GPIO_INPUT_POLARITY, 0);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_STATUS_TYPE, 1);
		img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_EDGE_SEL, 1);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_STATUS_TYPE, 0);
		img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_EDGE_SEL, 0);
		img_gpio_write(pc, gpio, IMG_GPIO_INPUT_POLARITY, 1);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_STATUS_TYPE, 0);
		img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_EDGE_SEL, 0);
		img_gpio_write(pc, gpio, IMG_GPIO_INPUT_POLARITY, 0);
		break;
	default:
		ret = -EINVAL;
	}

	/* Clear potential spurious interrupt */
	img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_STATUS, 0);

	if (!ret)
		img_gpio_write(pc, gpio, IMG_GPIO_INTERRUPT_EN, 1);

	return ret;
}

static struct irq_chip img_gpio_irq_chip = {
	.name = "pinctrl-img-gpio-irq",
	.irq_enable = img_gpio_irq_enable,
	.irq_disable = img_gpio_irq_disable,
	.irq_set_type = img_gpio_irq_set_type,
	.irq_startup = img_gpio_irq_startup,
	.irq_shutdown = img_gpio_irq_shutdown
};

static int img_pctl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	return pc->soc_data->num_groups;
}

static const char *img_pctl_get_group_name(struct pinctrl_dev *pctldev,
		unsigned int selector)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	return pc->soc_data->groups[selector].name;
}

static int img_pctl_get_group_pins(struct pinctrl_dev *pctldev,
		unsigned int selector, const unsigned int **pins,
		unsigned int *num_pins)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	*pins = pc->soc_data->groups[selector].pins;
	*num_pins = pc->soc_data->groups[selector].npins;

	return 0;
}

static char *_img_pctl_get_mfio_group(struct img_pinctrl *pc,
		unsigned int pin, unsigned int mfio_sel)
{
	int i, j;
	const struct img_pinctrl_group *g;

	for (i = 0; i < pc->soc_data->num_groups; i++) {
		g = &pc->soc_data->groups[i];
		for (j = 0; j < g->npins; j++)
			if ((g->pins[j] == pin) && (g->selects[j] == mfio_sel))
				return (char *)pc->soc_data->groups[i].name;
	}

	return "";
}

static char *_img_pctl_get_mfio_function(struct img_pinctrl *pc,
						const char *group_name)
{
	int i, j;
	const struct img_pinctrl_function *f;

	for (i = 0; i < pc->soc_data->num_functions; i++) {
		f = &pc->soc_data->functions[i];
		for (j = 0; j < f->ngroups; j++)
			if (!strcmp(group_name, f->groups[j]))
				return (char *)f->name;
	}

	return "";
}

static int _img_pctl_get_group(struct img_pinctrl *pc, const char *group_name)
{
	int i;

	for (i = 0; i < pc->soc_data->num_groups; i++)
		if (!strcmp(group_name, pc->soc_data->groups[i].name))
			return i;

	return -1;
}

static char *_img_pctl_dbg_get_irq_info(struct img_pinctrl *pc,
					unsigned int pin)
{
	char *int_type;

	if (img_gpio_read(pc, pin, IMG_GPIO_INTERRUPT_STATUS_TYPE))
		if (img_gpio_read(pc, pin, IMG_GPIO_INTERRUPT_EDGE_SEL))
			int_type = "Edge Both";
		else if (img_gpio_read(pc, pin, IMG_GPIO_INPUT_POLARITY))
			int_type = "Edge Rising";
		else
			int_type = "Edge Falling";
	else
		if (img_gpio_read(pc, pin, IMG_GPIO_INPUT_POLARITY))
			int_type = "Level High";
		else
			int_type = "Level Low";

	return int_type;
}

static void img_pctl_pin_dbg_show(struct pinctrl_dev *pctldev,
		struct seq_file *s, unsigned int pin)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	int irq;
	unsigned int mfio_sel, gpio_dir_output, is_gpio = 0;
	char *int_type, *group, *function;

	if (pin < pc->soc_data->num_gpios)
		is_gpio = img_gpio_read(pc, pin, IMG_GPIO_BIT_EN);

	if (is_gpio) {
		gpio_dir_output = img_gpio_read(pc, pin, IMG_GPIO_OUTPUT_EN);
		if (gpio_dir_output) {
			seq_printf(s, "GPIO output value %u",
			img_gpio_read(pc, pin, IMG_GPIO_OUTPUT));
		} else {
			irq = irq_find_mapping(pc->irq_domain, pin);

			if (img_gpio_read(pc, pin, IMG_GPIO_INTERRUPT_EN))
				int_type = _img_pctl_dbg_get_irq_info(pc, pin);
			else
				int_type = "Disabled";

			seq_printf(s, "GPIO input value %u irq %d (type %s)",
				img_gpio_read(pc, pin, IMG_GPIO_INPUT),
				irq, int_type);
		}
	} else if (pin < pc->soc_data->num_mfios) {

		if (pc->soc_data->mfio_reg_ctrl[pin])
			mfio_sel = img_mfio_read(pc,
					pc->soc_data->mfio_reg_ctrl[pin]);
		else
			mfio_sel = 0;

		group = _img_pctl_get_mfio_group(pc, pin, mfio_sel);
		function = _img_pctl_get_mfio_function(pc, group);

		seq_printf(s, "MFIO group %s function %s",
				group, function);
	} else {
		seq_printf(s, "Standard pin %s", pc->soc_data->pins[pin].name);
	}
}

static void img_pctl_dt_free_map(struct pinctrl_dev *pctldev,
		struct pinctrl_map *maps, unsigned int num_maps)
{
	int i;

	for (i = 0; i < num_maps; i++)
		if (maps[i].type == PIN_MAP_TYPE_CONFIGS_PIN)
			kfree(maps[i].data.configs.configs);

	kfree(maps);
}

const char *img_pctl_cfgs[IMG_PINCONF_NUM_CONFIGS] = {
	[IMG_PINCONF_PARAM_PULL] = "img,pull",
	[IMG_PINCONF_PARAM_DRIVE_STRENGTH] = "img,drive",
	[IMG_PINCONF_PARAM_SLEW] = "img,slew",
	[IMG_PINCONF_PARAM_SCHMITT] = "img,schmitt",
	[IMG_PINCONF_PARAM_FORCE_IO] = "img,forceio"
};

static int _img_pctl_dt_node_to_map_config_single(struct img_pinctrl *pc,
		struct device_node *np, u32 dt_index, int config_type,
		unsigned long *config, unsigned int num_cfgs)
{
	int err;
	u32 val;
	const char *name;

	name = img_pctl_cfgs[config_type];

	if (num_cfgs == 1) {
		err = of_property_read_u32(np, name, &val);
		if (err)
			return err;
	} else {
		err = of_property_read_u32_index(np, name, dt_index, &val);
		if (err)
			return err;
	}

	switch (config_type) {
	case IMG_PINCONF_PARAM_PULL:
	case IMG_PINCONF_PARAM_DRIVE_STRENGTH:
	case IMG_PINCONF_PARAM_FORCE_IO:
		if (val > 3) {
			dev_err(pc->dev, "%s: invalid %s: %u\n",
				of_node_full_name(np), name, val);
			return -EINVAL;
		}
		break;
	case IMG_PINCONF_PARAM_SLEW:
	case IMG_PINCONF_PARAM_SCHMITT:
		if (val > 1) {
			dev_err(pc->dev, "%s: invalid %s: %u\n",
				of_node_full_name(np), name, val);
			return -EINVAL;
		}
		break;
	default:
		dev_err(pc->dev, "%s: invalid configuration type: %d\n",
			of_node_full_name(np), config_type);
		return -EINVAL;
	}

	*config = IMG_PINCONF_PACK(config_type, val);

	return 0;
}

static int _img_pctl_dt_node_to_map_config(struct img_pinctrl *pc,
		struct device_node *np, u32 pin, int dt_index,
		unsigned int  *num_cfgs, unsigned int num_unique_cfg,
		struct pinctrl_map **maps)
{
	struct pinctrl_map *map = *maps;
	unsigned long *configs;
	int i, j, err;

	configs = kzalloc(sizeof(*configs) * num_unique_cfg, GFP_KERNEL);
	if (!configs)
		return -ENOMEM;

	for (i = 0, j = 0; i < IMG_PINCONF_NUM_CONFIGS; i++) {
		if (num_cfgs[i]) {
			err = _img_pctl_dt_node_to_map_config_single(pc, np,
				dt_index, i, &configs[j], num_cfgs[i]);
			if (err)
				goto out;
			j++;
		}
	}

	map->type = PIN_MAP_TYPE_CONFIGS_PIN;
	map->data.configs.group_or_pin = pc->soc_data->pins[pin].name;
	map->data.configs.configs = configs;
	map->data.configs.num_configs = j;
	(*maps)++;

	return 0;

out:
	kfree(configs);
	return err;
}

static int img_pctl_dt_node_to_map(struct pinctrl_dev *pctldev,
	struct device_node *np, struct pinctrl_map **map,
	unsigned int *ptr_num_maps)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	struct property *pins, *funcs, *groups, *configs;
	int err, i, k;
	struct pinctrl_map *maps, *cur_map;
	unsigned int num_pins, num_groups, num_funcs, num_maps, num_unique_cfg;
	unsigned int num_cfgs[IMG_PINCONF_NUM_CONFIGS];
	const unsigned int *pin_indexes = NULL;
	const char *group_name, *func_name;
	u32 pin;

	pins = of_find_property(np, "pins", NULL);
	num_pins = pins ? (pins->length / 4) : 0;
	groups = of_find_property(np, "group", NULL);
	num_groups = groups ? of_property_count_strings(np, "group") : 0;
	funcs = of_find_property(np, "function", NULL);
	num_funcs = funcs ? of_property_count_strings(np, "function") : 0;

	if (funcs && ((num_funcs != 1))) {
		dev_err(pc->dev, "%s: Invalid function specifier in node\n",
			of_node_full_name(np));
		return -EINVAL;
	}

	if ((!pins || num_pins < 1) && (!groups || num_groups != 1)) {
		dev_err(pc->dev,
			"%s: No valid pins or group present in node\n",
			of_node_full_name(np));
		return -EINVAL;
	}

	if (num_pins && num_groups) {
		dev_err(pc->dev,
			"%s: Cannot specify pins and group in same node\n",
			of_node_full_name(np));
		return -EINVAL;
	}

	if (!num_pins) {
		if (of_property_read_string(np, "group", &group_name)) {
			dev_err(pc->dev, "%s: Failed to read group string\n",
				of_node_full_name(np));
			return -EINVAL;
		}
		k = _img_pctl_get_group(pc, group_name);
		if (k == -1) {
			dev_err(pc->dev, "%s: Invalid group name\n",
				of_node_full_name(np));
			return -EINVAL;
		}
		pin_indexes = pc->soc_data->groups[k].pins;
		num_pins = pc->soc_data->groups[k].npins;
	}

	num_maps = 0;
	num_unique_cfg = 0;

	for (i = 0; i < IMG_PINCONF_NUM_CONFIGS; i++) {
		configs = of_find_property(np, img_pctl_cfgs[i], NULL);
		if (configs) {
			num_cfgs[i] = configs->length / 4;
			if ((num_cfgs[i] != 1) && (num_cfgs[i] != num_pins)) {
				dev_err(pc->dev,
					"%s: Invalid number of configs\n",
					of_node_full_name(np));
				return -EINVAL;
			}
			num_unique_cfg++;
		} else {
			num_cfgs[i] = 0;
		}
	}

	if (num_funcs)
		num_maps++;

	/* One map per pin which contains array of config values */
	if (num_unique_cfg)
		num_maps += num_pins;

	cur_map = maps = kzalloc(num_maps * sizeof(*maps), GFP_KERNEL);
	if (!maps)
		return -ENOMEM;

	if (num_funcs) {
		if (of_property_read_string(np, "function", &func_name)) {
			dev_err(pc->dev,
				"%s: Failed to read function string\n",
				of_node_full_name(np));
			err = -EINVAL;
			goto out;
		}
		cur_map->type = PIN_MAP_TYPE_MUX_GROUP;
		cur_map->data.mux.group = group_name;
		cur_map->data.mux.function = func_name;
		cur_map++;
	}

	if (num_unique_cfg) {
		for (i = 0; i < num_pins; i++) {
			if (pin_indexes) {
				pin = pin_indexes[i];
			} else {
				err = of_property_read_u32_index(np, "pins",
					i, &pin);
				if (err)
					goto out;
				if (pin >= pc->soc_data->num_pins) {
					dev_err(pc->dev,
						"%s: invalid pins value %u\n",
						of_node_full_name(np), pin);
					err = -EINVAL;
					goto out;
				}
			}

			err = _img_pctl_dt_node_to_map_config(pc, np, pin,
					i, num_cfgs, num_unique_cfg, &cur_map);
			if (err)
				goto out;
		}
	}

	*map = maps;
	*ptr_num_maps = num_maps;

	return 0;

out:
	kfree(maps);
	return err;
}

static const struct pinctrl_ops img_pctl_ops = {
	.get_groups_count = img_pctl_get_groups_count,
	.get_group_name = img_pctl_get_group_name,
	.get_group_pins = img_pctl_get_group_pins,
	.pin_dbg_show = img_pctl_pin_dbg_show,
	.dt_node_to_map = img_pctl_dt_node_to_map,
	.dt_free_map = img_pctl_dt_free_map
};

static int img_pmx_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	return pc->soc_data->num_functions;
}

static const char *img_pmx_get_function_name(struct pinctrl_dev *pctldev,
		unsigned int selector)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	return pc->soc_data->functions[selector].name;
}

static int img_pmx_get_function_groups(struct pinctrl_dev *pctldev,
		unsigned int selector,
		const char * const **groups,
		unsigned int * const num_groups)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	*groups = pc->soc_data->functions[selector].groups;
	*num_groups = pc->soc_data->functions[selector].ngroups;

	return 0;
}

static int img_pmx_enable(struct pinctrl_dev *pctldev,
		unsigned int func_selector, unsigned int group_selector)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	unsigned int i, npins;
	const unsigned int *pins, *selects;
	unsigned long flags;
	u32 smask, reg_mfio_data, reg, sval;

	npins = pc->soc_data->groups[group_selector].npins;
	pins = pc->soc_data->groups[group_selector].pins;
	selects = pc->soc_data->groups[group_selector].selects;
	smask = pc->soc_data->groups[group_selector].scenario_mask;
	sval = pc->soc_data->groups[group_selector].scenario_val;

	spin_lock_irqsave(&pc->lock, flags);

	if (smask) {
		reg = readl(pc->base + IMG_PINCONF_SCENARIO);
		reg = (reg & ~smask) | (sval & smask);
		writel(reg, pc->base + IMG_PINCONF_SCENARIO);
	}

	for (i = 0; i < npins; i++) {
		reg_mfio_data = pc->soc_data->mfio_reg_ctrl[pins[i]];
		if (reg_mfio_data)
			img_mfio_write(pc, reg_mfio_data, selects[i]);
		img_gpio_write(pc, pins[i], IMG_GPIO_BIT_EN, 0);
	}

	spin_unlock_irqrestore(&pc->lock, flags);

	dev_dbg(pctldev->dev, "Function %s group %s muxed",
			pc->soc_data->functions[func_selector].name,
			pc->soc_data->groups[group_selector].name);

	return 0;
}

static void img_pmx_disable(struct pinctrl_dev *pctldev,
		unsigned int func_selector, unsigned int group_selector)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	unsigned int i;
	unsigned int npins = pc->soc_data->groups[group_selector].npins;
	const unsigned int *pins = pc->soc_data->groups[group_selector].pins;

	/* Disable by setting to GPIO IN */
	for (i = 0; i < npins; i++) {
		img_gpio_write(pc, pins[i], IMG_GPIO_OUTPUT_EN, 0);
		img_gpio_write(pc, pins[i], IMG_GPIO_BIT_EN, 1);
	}

	dev_dbg(pctldev->dev, "Function %s group %s demuxed",
			pc->soc_data->functions[func_selector].name,
			pc->soc_data->groups[group_selector].name);
}

static int img_pmx_gpio_request_enable(struct pinctrl_dev *pctldev,
			struct pinctrl_gpio_range *range, unsigned int pin)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	/*
	 * Set to GPIO IN to avoid sending erroneous signals when the GPIO
	 * enable is asserted
	 */
	img_gpio_write(pc, pin, IMG_GPIO_OUTPUT_EN, 0);
	img_gpio_write(pc, pin, IMG_GPIO_BIT_EN, 1);

	dev_dbg(pctldev->dev, "Pin %u pmx GPIO alloc", pin);

	return 0;
}

static void img_pmx_gpio_disable_free(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range, unsigned int pin)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	img_gpio_write(pc, pin, IMG_GPIO_OUTPUT_EN, 0);
	img_gpio_write(pc, pin, IMG_GPIO_BIT_EN, 0);

	dev_dbg(pctldev->dev, "Pin %u pmx GPIO free", pin);
}

static int img_pmx_gpio_set_dir(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range,
		unsigned int pin, bool input)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	img_gpio_write(pc, pin, IMG_GPIO_OUTPUT_EN, input ? 0 : 1);

	return 0;
}

static const struct pinmux_ops img_pmx_ops = {
	.get_functions_count = img_pmx_get_functions_count,
	.get_function_name = img_pmx_get_function_name,
	.get_function_groups = img_pmx_get_function_groups,
	.enable = img_pmx_enable,
	.disable = img_pmx_disable,
	.gpio_request_enable = img_pmx_gpio_request_enable,
	.gpio_disable_free = img_pmx_gpio_disable_free,
	.gpio_set_direction = img_pmx_gpio_set_dir,
};

static int img_pinconf_get(struct pinctrl_dev *pctldev,
			unsigned int pin, unsigned long *config)
{
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	enum img_pinconf_param param = IMG_PINCONF_UNPACK_PARAM(*config);
	unsigned int i_en, o_en, o_sts, val;

	switch (param) {
	case IMG_PINCONF_PARAM_PULL:
		val = img_pinconf_read(pc, pin, IMG_PINCONF_PU_PD, 2);
		break;
	case IMG_PINCONF_PARAM_DRIVE_STRENGTH:
		val = img_pinconf_read(pc, pin, IMG_PINCONF_DRIVE_STRENGTH, 2);
		break;
	case IMG_PINCONF_PARAM_SLEW:
		val = img_pinconf_read(pc, pin, IMG_PINCONF_SLEW_RATE, 1);
		break;
	case IMG_PINCONF_PARAM_SCHMITT:
		val = img_pinconf_read(pc, pin, IMG_PINCONF_SCHMITT_EN, 1);
		break;
	case IMG_PINCONF_PARAM_FORCE_IO:
		i_en = img_pinconf_read(pc, pin, IMG_PINCONF_IE, 1);
		o_en = img_pinconf_read(pc, pin, IMG_PINCONF_OE, 1);
		o_sts = img_pinconf_read(pc, pin, IMG_PINCONF_OE_STS, 1);
		if (i_en) {
			if (o_en) {
				if (o_sts)
					val = 1;
				else
					val = 3;
			} else {
				dev_err(pc->dev, "Bad value in force io regs");
				return -EINVAL;
			}
		} else if (o_en) {
			if (o_sts) {
				dev_err(pc->dev, "Bad value in force io regs");
				return -EINVAL;
			} else {
				val = 2;
			}
		} else {
			val = 0;
		}

		break;
	default:
		return -ENOTSUPP;
	}

	*config = IMG_PINCONF_PACK(param, val);

	return 0;
}

static int _img_pinconf_set_single(struct img_pinctrl *pc, unsigned int pin,
		enum img_pinconf_param param, unsigned int val)
{

	switch (param) {
	case IMG_PINCONF_PARAM_PULL:
		if (val > 3) {
			dev_err(pc->dev, "Bad pull value: %u\n", val);
			return -EINVAL;
		}
		img_pinconf_write(pc, pin, IMG_PINCONF_PU_PD, 2, val);
		break;
	case IMG_PINCONF_PARAM_DRIVE_STRENGTH:
		if (val > 3) {
			dev_err(pc->dev, "Bad drive value: %u\n", val);
			return -EINVAL;
		}
		img_pinconf_write(pc, pin, IMG_PINCONF_DRIVE_STRENGTH, 2, val);
		break;
	case IMG_PINCONF_PARAM_SLEW:
		if (val > 1) {
			dev_err(pc->dev, "Bad slew value: %u\n", val);
			return -EINVAL;
		}
		img_pinconf_write(pc, pin, IMG_PINCONF_SLEW_RATE, 1, val);
		break;
	case IMG_PINCONF_PARAM_SCHMITT:
		if (val > 1) {
			dev_err(pc->dev, "Bad schmitt value: %u\n", val);
			return -EINVAL;
		}
		img_pinconf_write(pc, pin, IMG_PINCONF_SCHMITT_EN, 1, val);
		break;
	case IMG_PINCONF_PARAM_FORCE_IO:
		if (val > 3) {
			dev_err(pc->dev, "Bad force io value: %u\n", val);
			return -EINVAL;
		}
		switch (val) {
		case 0:
			img_pinconf_write(pc, pin, IMG_PINCONF_OE, 1, 0);
			img_pinconf_write(pc, pin, IMG_PINCONF_IE, 1, 0);
			break;
		case 1:
			img_pinconf_write(pc, pin, IMG_PINCONF_OE_STS, 1, 1);
			img_pinconf_write(pc, pin, IMG_PINCONF_OE, 1, 1);
			img_pinconf_write(pc, pin, IMG_PINCONF_IE, 1, 1);
			break;
		case 2:
			img_pinconf_write(pc, pin, IMG_PINCONF_OE_STS, 1, 0);
			img_pinconf_write(pc, pin, IMG_PINCONF_OE, 1, 1);
			img_pinconf_write(pc, pin, IMG_PINCONF_IE, 1, 0);
			break;
		case 3:
			img_pinconf_write(pc, pin, IMG_PINCONF_OE_STS, 1, 0);
			img_pinconf_write(pc, pin, IMG_PINCONF_OE, 1, 1);
			img_pinconf_write(pc, pin, IMG_PINCONF_IE, 1, 1);
			break;
		}
		break;
	default:
		return -ENOTSUPP;
	}

	return 0;
}

static int img_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
		unsigned long *configs, unsigned int num_configs)
{
	int i, ret;
	struct img_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	enum img_pinconf_param param;
	unsigned int val;
	unsigned long flags;

	for (i = 0; i < num_configs; i++) {

		param = IMG_PINCONF_UNPACK_PARAM(configs[i]);
		val = IMG_PINCONF_UNPACK_ARG(configs[i]);

		spin_lock_irqsave(&pc->lock, flags);

		ret = _img_pinconf_set_single(pc, pin, param, val);

		spin_unlock_irqrestore(&pc->lock, flags);

		if (ret)
			return ret;
	}

	return 0;
}

static const struct pinconf_ops img_pinconf_ops = {
	.pin_config_get = img_pinconf_get,
	.pin_config_set = img_pinconf_set,
};

static struct pinctrl_desc img_pinctrl_desc = {
	.name = "pinctrl-img",
	.pctlops = &img_pctl_ops,
	.pmxops = &img_pmx_ops,
	.confops = &img_pinconf_ops,
	.owner = THIS_MODULE,
};

int img_pinctrl_probe(struct platform_device *pdev,
		const struct img_pinctrl_soc_data *soc_data)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct img_pinctrl *pc;
	struct resource iomem;
	int err, i;
	u32 num_banks, len;
	char *name;

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	platform_set_drvdata(pdev, pc);
	pc->dev = dev;
	pc->soc_data = soc_data;
	pc->gpio_irq_data = soc_data->gpio_irq_data;

	spin_lock_init(&pc->lock);

	err = of_address_to_resource(np, 0, &iomem);
	if (err) {
		dev_err(dev, "could not get IO memory (pinctrl)\n");
		return err;
	}

	pc->base = devm_ioremap_resource(dev, &iomem);
	if (IS_ERR(pc->base))
		return PTR_ERR(pc->base);

	img_gpio_chip.ngpio = soc_data->num_gpios;

	img_pinctrl_desc.pins = soc_data->pins;
	img_pinctrl_desc.npins = soc_data->num_pins;

	pc->gpio_chip = &img_gpio_chip;
	pc->gpio_chip->dev = dev;
	pc->gpio_chip->of_node = np;

	num_banks = DIV_ROUND_UP(soc_data->num_gpios, 16);

	pc->pctl_dev = pinctrl_register(&img_pinctrl_desc, dev, pc);
	if (!pc->pctl_dev)
		return -EINVAL;

	err = gpiochip_add(pc->gpio_chip);
	if (err) {
		dev_err(dev, "could not add GPIO chip\n");
		goto err_pctl;
	}

	err = gpiochip_add_pin_range(pc->gpio_chip, dev_name(&pdev->dev),
			0, 0, pc->soc_data->num_gpios);
	if (err)
		goto err_gpio;

	pc->irq_domain = irq_domain_add_linear(np, soc_data->num_gpios,
			&irq_domain_simple_ops, NULL);
	if (!pc->irq_domain) {
		dev_err(dev, "could not create IRQ domain\n");
		err = -EINVAL;
		goto err_gpio;
	}

	for (i = 0; i < soc_data->num_gpios; i++) {
		int irq = irq_create_mapping(pc->irq_domain, i);
		irq_set_chip_and_handler(irq, &img_gpio_irq_chip,
				handle_simple_irq);
		irq_set_chip_data(irq, pc);
	}

	for (i = 0; i < num_banks; i++) {
		pc->gpio_irq_data[i].irq = irq_of_parse_and_map(np, i);
		pc->gpio_irq_data[i].pc = pc;
		pc->gpio_irq_data[i].bank = i;

		len = strlen(dev_name(pc->dev)) + 16;
		name = devm_kzalloc(pc->dev, len, GFP_KERNEL);
		if (!name) {
			err = -ENOMEM;
			goto err_irqdom;
		}
		snprintf(name, len, "%s:bank%d", dev_name(pc->dev), i);

		err = devm_request_irq(dev, pc->gpio_irq_data[i].irq,
			img_gpio_irq_handler, IRQF_SHARED,
			name, &pc->gpio_irq_data[i]);
		if (err) {
			dev_err(dev, "unable to request IRQ %d\n",
					pc->gpio_irq_data[i].irq);
			goto err_irqdom;
		}
	}

	dev_info(dev, "Probe successful. %u pins, %u mfios, %u gpios",
			soc_data->num_pins, soc_data->num_mfios,
			soc_data->num_gpios);

	return 0;

err_irqdom:
	irq_domain_remove(pc->irq_domain);
err_gpio:
	err = gpiochip_remove(pc->gpio_chip);
	if (err)
		return err;
err_pctl:
	pinctrl_unregister(pc->pctl_dev);

	return err;
}

int img_pinctrl_remove(struct platform_device *pdev)
{
	int err;
	struct img_pinctrl *pc = platform_get_drvdata(pdev);

	irq_domain_remove(pc->irq_domain);

	err = gpiochip_remove(pc->gpio_chip);
	if (err)
		return err;

	pinctrl_unregister(pc->pctl_dev);

	return 0;
}
