// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim max96755f pin control driver.
 *
 * Copyright (c) 2022 Rockchip Electronics Co. Ltd.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/max96755f.h>

#include "core.h"
#include "pinconf.h"
#include "pinmux.h"

struct max96755f_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctl;
	struct regmap *regmap;
};

struct config_desc {
	u16 reg;
	u8 mask;
	u8 val;
};

struct max96755f_group_data {
	const struct config_desc *configs;
	int num_configs;
};

struct max96755f_function_data {
	u8 gpio_out_dis:1;
	u8 gpio_tx_en:1;
	u8 gpio_rx_en:1;
	u8 gpio_tx_id;
	u8 gpio_rx_id;
};

static int max96755f_pinmux_set_mux(struct pinctrl_dev *pctldev,
				    unsigned int function, unsigned int group)
{
	struct max96755f_pinctrl *mpctl = pinctrl_dev_get_drvdata(pctldev);
	struct function_desc *func;
	struct group_desc *grp;
	int i;

	func = pinmux_generic_get_function(pctldev, function);
	if (!func)
		return -EINVAL;

	grp = pinctrl_generic_get_group(pctldev, group);
	if (!grp)
		return -EINVAL;

	if (func->data) {
		struct max96755f_function_data *fdata = func->data;

		for (i = 0; i < grp->num_pins; i++) {
			regmap_update_bits(mpctl->regmap, GPIO_A_REG(grp->pins[i]),
					   GPIO_OUT_DIS | GPIO_RX_EN | GPIO_TX_EN,
					   FIELD_PREP(GPIO_OUT_DIS, fdata->gpio_out_dis) |
					   FIELD_PREP(GPIO_RX_EN, fdata->gpio_rx_en) |
					   FIELD_PREP(GPIO_TX_EN, fdata->gpio_tx_en));

			if (fdata->gpio_tx_en)
				regmap_update_bits(mpctl->regmap, GPIO_B_REG(grp->pins[i]),
						   GPIO_TX_ID,
						   FIELD_PREP(GPIO_TX_ID, fdata->gpio_tx_id));

			if (fdata->gpio_rx_en)
				regmap_update_bits(mpctl->regmap, GPIO_C_REG(grp->pins[i]),
						   GPIO_RX_ID,
						   FIELD_PREP(GPIO_RX_ID, fdata->gpio_rx_id));
		}
	}

	if (grp->data) {
		struct max96755f_group_data *gdata = grp->data;

		for (i = 0; i < gdata->num_configs; i++) {
			const struct config_desc *config = &gdata->configs[i];

			regmap_update_bits(mpctl->regmap, config->reg,
					   config->mask, config->val);
		}
	}

	return 0;
}

static int max96755f_pinconf_get(struct pinctrl_dev *pctldev,
				 unsigned int pin, unsigned long *config)
{
	struct max96755f_pinctrl *mpctl = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	unsigned int gpio_a_reg, gpio_b_reg;
	u16 arg = 0;

	regmap_read(mpctl->regmap, GPIO_A_REG(pin), &gpio_a_reg);
	regmap_read(mpctl->regmap, GPIO_B_REG(pin), &gpio_b_reg);

	switch (param) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		if (FIELD_GET(OUT_TYPE, gpio_b_reg))
			return -EINVAL;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		if (!FIELD_GET(OUT_TYPE, gpio_b_reg))
			return -EINVAL;
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		if (FIELD_GET(PULL_UPDN_SEL, gpio_b_reg) != 0)
			return -EINVAL;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (FIELD_GET(PULL_UPDN_SEL, gpio_b_reg) != 1)
			return -EINVAL;
		switch (FIELD_GET(RES_CFG, gpio_a_reg)) {
		case 0:
			arg = 40000;
			break;
		case 1:
			arg = 10000;
			break;
		}
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (FIELD_GET(PULL_UPDN_SEL, gpio_b_reg) != 2)
			return -EINVAL;
		switch (FIELD_GET(RES_CFG, gpio_a_reg)) {
		case 0:
			arg = 40000;
			break;
		case 1:
			arg = 10000;
			break;
		}
		break;
	case PIN_CONFIG_OUTPUT:
		if (FIELD_GET(GPIO_OUT_DIS, gpio_a_reg))
			return -EINVAL;

		arg = FIELD_GET(GPIO_OUT, gpio_a_reg);
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int max96755f_pinconf_set(struct pinctrl_dev *pctldev,
				 unsigned int pin, unsigned long *configs,
				 unsigned int num_configs)
{
	struct max96755f_pinctrl *mpctl = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param;
	u32 arg;
	u8 res_cfg;
	int i;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			regmap_update_bits(mpctl->regmap, GPIO_B_REG(pin),
					   OUT_TYPE, FIELD_PREP(OUT_TYPE, 0));
			break;
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			regmap_update_bits(mpctl->regmap, GPIO_B_REG(pin),
					   OUT_TYPE, FIELD_PREP(OUT_TYPE, 1));
			break;
		case PIN_CONFIG_BIAS_DISABLE:
			regmap_update_bits(mpctl->regmap, GPIO_C_REG(pin),
					   PULL_UPDN_SEL,
					   FIELD_PREP(PULL_UPDN_SEL, 0));
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			switch (arg) {
			case 40000:
				res_cfg = 0;
				break;
			case 1000000:
				res_cfg = 1;
				break;
			default:
				return -EINVAL;
			}

			regmap_update_bits(mpctl->regmap, GPIO_A_REG(pin),
					   RES_CFG, FIELD_PREP(RES_CFG, res_cfg));
			regmap_update_bits(mpctl->regmap, GPIO_C_REG(pin),
					   PULL_UPDN_SEL,
					   FIELD_PREP(PULL_UPDN_SEL, 1));
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			switch (arg) {
			case 40000:
				res_cfg = 0;
				break;
			case 1000000:
				res_cfg = 1;
				break;
			default:
				return -EINVAL;
			}

			regmap_update_bits(mpctl->regmap, GPIO_A_REG(pin),
					   RES_CFG, FIELD_PREP(RES_CFG, res_cfg));
			regmap_update_bits(mpctl->regmap, GPIO_C_REG(pin),
					   PULL_UPDN_SEL,
					   FIELD_PREP(PULL_UPDN_SEL, 2));
			break;
		case PIN_CONFIG_OUTPUT:
			regmap_update_bits(mpctl->regmap, GPIO_A_REG(pin),
					   GPIO_OUT_DIS | GPIO_OUT,
					   FIELD_PREP(GPIO_OUT_DIS, 0) |
					   FIELD_PREP(GPIO_OUT, arg));
			break;
		default:
			return -ENOTSUPP;
		}
	}

	return 0;
}

static const struct pinconf_ops max96755f_pinconf_ops = {
	.pin_config_get = max96755f_pinconf_get,
	.pin_config_set = max96755f_pinconf_set,
};

static const struct pinmux_ops max96755f_pinmux_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = max96755f_pinmux_set_mux,
};

static const struct pinctrl_ops max96755f_pinctrl_ops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name = pinctrl_generic_get_group_name,
	.get_group_pins = pinctrl_generic_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static const struct pinctrl_pin_desc max96755f_pins_desc[] = {
	PINCTRL_PIN(0, "MFP0"),
	PINCTRL_PIN(1, "MFP1"),
	PINCTRL_PIN(2, "MFP2"),
	PINCTRL_PIN(3, "MFP3"),
	PINCTRL_PIN(4, "MFP4"),
	PINCTRL_PIN(5, "MFP5"),
	PINCTRL_PIN(6, "MFP6"),
	PINCTRL_PIN(7, "MFP7"),
	PINCTRL_PIN(8, "MFP8"),
	PINCTRL_PIN(9, "MFP9"),
	PINCTRL_PIN(10, "MFP10"),
	PINCTRL_PIN(11, "MFP11"),
	PINCTRL_PIN(12, "MFP12"),
	PINCTRL_PIN(13, "MFP13"),
	PINCTRL_PIN(14, "MFP14"),
	PINCTRL_PIN(15, "MFP15"),
	PINCTRL_PIN(16, "MFP16"),
	PINCTRL_PIN(17, "MFP17"),
	PINCTRL_PIN(18, "MFP18"),
	PINCTRL_PIN(19, "MFP19"),
	PINCTRL_PIN(20, "MFP20"),
};

static int MFP0_pins[] = {0};
static int MFP1_pins[] = {1};
static int MFP2_pins[] = {2};
static int MFP3_pins[] = {3};
static int MFP4_pins[] = {4};
static int MFP5_pins[] = {5};
static int MFP6_pins[] = {6};
static int MFP7_pins[] = {7};
static int MFP8_pins[] = {8};
static int MFP9_pins[] = {9};
static int MFP10_pins[] = {10};
static int MFP11_pins[] = {11};
static int MFP12_pins[] = {12};
static int MFP13_pins[] = {13};
static int MFP14_pins[] = {14};
static int MFP15_pins[] = {15};
static int MFP16_pins[] = {16};
static int MFP17_pins[] = {17};
static int MFP18_pins[] = {18};
static int MFP19_pins[] = {19};
static int MFP20_pins[] = {20};
static int I2C_pins[] = {19, 20};
static int UART_pins[] = {19, 20};

#define GROUP_DESC(nm) \
{ \
	.name = #nm, \
	.pins = nm ## _pins, \
	.num_pins = ARRAY_SIZE(nm ## _pins), \
}

#define GROUP_DESC_CONFIG(nm) \
{ \
	.name = #nm, \
	.pins = nm ## _pins, \
	.num_pins = ARRAY_SIZE(nm ## _pins), \
	.data = (void *)(const struct max96755f_group_data []) { \
		{ \
			.configs = nm ## _configs, \
			.num_configs = ARRAY_SIZE(nm ## _configs), \
		} \
	}, \
}

static const struct config_desc MFP0_configs[] = {
	{ 0x0005, LOCK_EN, 0 },
	{ 0x0048, LOC_MS_EN, 0},
};

static const struct config_desc MFP1_configs[] = {
	{ 0x0005, ERRB_EN, 0 },
};

static const struct config_desc MFP4_configs[] = {
	{ 0x070, SPI_EN, 0 },
};

static const struct config_desc MFP5_configs[] = {
	{ 0x006, RCLKEN, 0 },
};

static const struct config_desc MFP7_configs[] = {
	{ 0x0002, AUD_TX_EN_X, 0 },
	{ 0x0002, AUD_TX_EN_Y, 0 }
};

static const struct config_desc MFP8_configs[] = {
	{ 0x0002, AUD_TX_EN_X, 0 },
	{ 0x0002, AUD_TX_EN_Y, 0 }
};

static const struct config_desc MFP9_configs[] = {
	{ 0x0002, AUD_TX_EN_X, 0 },
	{ 0x0002, AUD_TX_EN_Y, 0 }
};

static const struct config_desc MFP10_configs[] = {
	{ 0x0001, IIC_2_EN, 0 },
	{ 0x0003, UART_2_EN, 0 },
	{ 0x0140, AUD_RX_EN, 0},
};

static const struct config_desc MFP11_configs[] = {
	{ 0x0001, IIC_2_EN, 0 },
	{ 0x0003, UART_2_EN, 0 },
	{ 0x0140, AUD_RX_EN, 0},
};

static const struct config_desc MFP12_configs[] = {
	{ 0x0140, AUD_RX_EN, 0 },
};

static const struct config_desc MFP13_configs[] = {
	{ 0x0005, PU_LF0, 0 },
};

static const struct config_desc MFP14_configs[] = {
	{ 0x0005, PU_LF1, 0 },
};

static const struct config_desc MFP15_configs[] = {
	{ 0x0005, PU_LF2, 0 },
};

static const struct config_desc MFP16_configs[] = {
	{ 0x0005, PU_LF3, 0 },
};

static const struct config_desc MFP17_configs[] = {
	{ 0x0001, IIC_1_EN, 0 },
	{ 0x0003, UART_1_EN, 0 },
};

static const struct config_desc MFP18_configs[] = {
	{ 0x0001, IIC_1_EN, 0 },
	{ 0x0003, UART_1_EN, 0 },
};

static const struct group_desc max96755f_groups[] = {
	GROUP_DESC_CONFIG(MFP0),
	GROUP_DESC_CONFIG(MFP1),
	GROUP_DESC(MFP2),
	GROUP_DESC(MFP3),
	GROUP_DESC_CONFIG(MFP4),
	GROUP_DESC_CONFIG(MFP5),
	GROUP_DESC(MFP6),
	GROUP_DESC_CONFIG(MFP7),
	GROUP_DESC_CONFIG(MFP8),
	GROUP_DESC_CONFIG(MFP9),
	GROUP_DESC_CONFIG(MFP10),
	GROUP_DESC_CONFIG(MFP11),
	GROUP_DESC_CONFIG(MFP12),
	GROUP_DESC_CONFIG(MFP13),
	GROUP_DESC_CONFIG(MFP14),
	GROUP_DESC_CONFIG(MFP15),
	GROUP_DESC_CONFIG(MFP16),
	GROUP_DESC_CONFIG(MFP17),
	GROUP_DESC_CONFIG(MFP18),
	GROUP_DESC(MFP19),
	GROUP_DESC(MFP20),
	GROUP_DESC(I2C),
	GROUP_DESC(UART),
};

static const char *MFP_groups[] = {
	"MFP0", "MFP1", "MFP2", "MFP3", "MFP4", "MFP5",
	"MFP6", "MFP7", "MFP8", "MFP9", "MFP10",
	"MFP11", "MFP12", "MFP13", "MFP14", "MFP15",
	"MFP16", "MFP17", "MFP18", "MFP19", "MFP20",
};
static const char *I2C_groups[] = { "I2C" };
static const char *UART_groups[] = { "UART" };

#define FUNCTION_DESC(nm) \
{ \
	.name = #nm, \
	.group_names = nm##_groups, \
	.num_group_names = ARRAY_SIZE(nm##_groups), \
} \

#define FUNCTION_DESC_GPIO() \
{ \
	.name = "GPIO", \
	.group_names = MFP_groups, \
	.num_group_names = ARRAY_SIZE(MFP_groups), \
	.data = (void *)(const struct max96755f_function_data []) { \
		{ } \
	}, \
} \

#define FUNCTION_DESC_GPIO_RX(id) \
{ \
	.name = "GPIO_RX_"#id, \
	.group_names = MFP_groups, \
	.num_group_names = ARRAY_SIZE(MFP_groups), \
	.data = (void *)(const struct max96755f_function_data []) { \
		{ .gpio_rx_en = 1, .gpio_rx_id = id } \
	}, \
} \

#define FUNCTION_DESC_GPIO_TX(id) \
{ \
	.name = "GPIO_TX_"#id, \
	.group_names = MFP_groups, \
	.num_group_names = ARRAY_SIZE(MFP_groups), \
	.data = (void *)(const struct max96755f_function_data []) { \
		{ .gpio_out_dis = 1, .gpio_tx_en = 1, .gpio_tx_id = id } \
	}, \
} \

static const struct function_desc max96755f_functions[] = {
	FUNCTION_DESC_GPIO_TX(0),
	FUNCTION_DESC_GPIO_TX(1),
	FUNCTION_DESC_GPIO_TX(2),
	FUNCTION_DESC_GPIO_TX(3),
	FUNCTION_DESC_GPIO_TX(4),
	FUNCTION_DESC_GPIO_TX(5),
	FUNCTION_DESC_GPIO_TX(6),
	FUNCTION_DESC_GPIO_TX(7),
	FUNCTION_DESC_GPIO_TX(8),
	FUNCTION_DESC_GPIO_TX(9),
	FUNCTION_DESC_GPIO_TX(10),
	FUNCTION_DESC_GPIO_TX(11),
	FUNCTION_DESC_GPIO_TX(12),
	FUNCTION_DESC_GPIO_TX(13),
	FUNCTION_DESC_GPIO_TX(14),
	FUNCTION_DESC_GPIO_TX(15),
	FUNCTION_DESC_GPIO_TX(16),
	FUNCTION_DESC_GPIO_TX(17),
	FUNCTION_DESC_GPIO_TX(18),
	FUNCTION_DESC_GPIO_TX(19),
	FUNCTION_DESC_GPIO_TX(20),
	FUNCTION_DESC_GPIO_RX(0),
	FUNCTION_DESC_GPIO_RX(1),
	FUNCTION_DESC_GPIO_RX(2),
	FUNCTION_DESC_GPIO_RX(3),
	FUNCTION_DESC_GPIO_RX(4),
	FUNCTION_DESC_GPIO_RX(5),
	FUNCTION_DESC_GPIO_RX(6),
	FUNCTION_DESC_GPIO_RX(7),
	FUNCTION_DESC_GPIO_RX(8),
	FUNCTION_DESC_GPIO_RX(9),
	FUNCTION_DESC_GPIO_RX(10),
	FUNCTION_DESC_GPIO_RX(11),
	FUNCTION_DESC_GPIO_RX(12),
	FUNCTION_DESC_GPIO_RX(13),
	FUNCTION_DESC_GPIO_RX(14),
	FUNCTION_DESC_GPIO_RX(15),
	FUNCTION_DESC_GPIO_RX(16),
	FUNCTION_DESC_GPIO_RX(17),
	FUNCTION_DESC_GPIO_RX(18),
	FUNCTION_DESC_GPIO_RX(19),
	FUNCTION_DESC_GPIO_RX(20),
	FUNCTION_DESC_GPIO(),
	FUNCTION_DESC(I2C),
	FUNCTION_DESC(UART),
};

static int max96755f_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max96755f_pinctrl *mpctl;
	struct pinctrl_desc *pctl_desc;
	int i, ret;

	mpctl = devm_kzalloc(dev, sizeof(*mpctl), GFP_KERNEL);
	if (!mpctl)
		return -ENOMEM;

	mpctl->dev = dev;
	platform_set_drvdata(pdev, mpctl);

	mpctl->regmap = dev_get_regmap(dev->parent, NULL);
	if (!mpctl->regmap)
		return dev_err_probe(dev, -ENODEV, "failed to get regmap\n");

	pctl_desc = devm_kzalloc(dev, sizeof(*pctl_desc), GFP_KERNEL);
	if (!pctl_desc)
		return -ENOMEM;

	pctl_desc->name = dev_name(dev);
	pctl_desc->owner = THIS_MODULE;
	pctl_desc->pctlops = &max96755f_pinctrl_ops;
	pctl_desc->pmxops = &max96755f_pinmux_ops;
	pctl_desc->confops = &max96755f_pinconf_ops;
	pctl_desc->pins = max96755f_pins_desc;
	pctl_desc->npins = ARRAY_SIZE(max96755f_pins_desc);

	ret = devm_pinctrl_register_and_init(dev, pctl_desc, mpctl,
					     &mpctl->pctl);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register pinctrl\n");

	for (i = 0; i < ARRAY_SIZE(max96755f_groups); i++) {
		const struct group_desc *group = &max96755f_groups[i];

		ret = pinctrl_generic_add_group(mpctl->pctl, group->name,
						group->pins, group->num_pins,
						group->data);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "failed to register group %s\n",
					     group->name);
	}

	for (i = 0; i < ARRAY_SIZE(max96755f_functions); i++) {
		const struct function_desc *func = &max96755f_functions[i];

		ret = pinmux_generic_add_function(mpctl->pctl, func->name,
						  func->group_names,
						  func->num_group_names,
						  func->data);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "failed to register function %s\n",
					     func->name);
	}

	return pinctrl_enable(mpctl->pctl);
}

static const struct of_device_id max96755f_pinctrl_of_match[] = {
	{ .compatible = "maxim,max96755f-pinctrl" },
	{}
};
MODULE_DEVICE_TABLE(of, max96755f_pinctrl_of_match);

static struct platform_driver max96755f_pinctrl_driver = {
	.driver = {
		.name = "max96755f-pinctrl",
		.of_match_table = max96755f_pinctrl_of_match,
	},
	.probe = max96755f_pinctrl_probe,
};

module_platform_driver(max96755f_pinctrl_driver);

MODULE_AUTHOR("Guochun Huang <hero.huang@rock-chips.com>");
MODULE_DESCRIPTION("Maxim max96755f pin control driver");
MODULE_LICENSE("GPL");
