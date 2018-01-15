/*
* Simple driver for Texas Instruments LM3648 LED Flash driver chip
* Copyright (C) 2012 Texas Instruments
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <linux/platform_data/leds-lm3648.h>

#define	REG_TOUT			(0x8)
#define	REG_BOOST_CON			(0x6)
#define	REG_FLASH_BRIGHT		(0x3)
#define	REG_TORCH_BRIGHT		(0x5)
#define	REG_ENABLE			(0x1)
#define	REG_FLAG			(0xA)
#define	REG_MAX				(0xD)

#define	FLASH_TOUT_TIME_SHIFT		(0)
#define	FLASH_BRIGHT_SHIFT		(0)
#define	TORCH_BRIGHT_SHIFT		(0)
#define	IVFM_SHIFT			(7)
#define	TX_PIN_EN_SHIFT			(6)
#define	STROBE_PIN_EN_SHIFT		(5)
#define	TORCH_PIN_EN_SHIFT		(4)
#define	MODE_BITS_SHIFT			(0)

#define	FLASH_TOUT_TIME_MASK		(0x7)
#define	TORCH_BRIGHT_MASK		(0x7F)
#define	FLASH_BRIGHT_MASK		(0xBF)
#define	TX_PIN_EN_MASK			(0x1)
#define	STROBE_PIN_EN_MASK		(0x1)
#define	TORCH_PIN_EN_MASK		(0x1)
#define	MODE_BITS_MASK			(0xFF)

#define TORCH_BRIGHT_MAX		(0x7F)
#define FLASH_BRIGHT_MAX		(0x3F)

const char *PLTFRM_PIN_STROBE = OF_LM3648_GPIO_STROBE;
const char *PLTFRM_PIN_HWEN = OF_LM3648_GPIO_HWEN;

enum lm3648_mode {
	MODES_STASNDBY = 0,
	MODES_TORCH = 0xb,
	MODES_FLASH = 0xf
};

struct lm3648_platform_data {
	struct lm3648_platform_goio gpios[2];
};

struct lm3648_chip_data {
	struct device *dev;

	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;

	struct work_struct work_flash;
	struct work_struct work_torch;

	u8 br_flash;
	u8 br_torch;

	enum lm3648_torch_pin_enable torch_pin;
	enum lm3648_strobe_pin_enable strobe_pin;
	enum lm3648_tx_pin_enable tx_pin;

	struct lm3648_platform_data *pdata;
	struct regmap *regmap;
	struct mutex lock;

	unsigned int last_flag;
};

static int lm3648_set_pin_state(struct lm3648_chip_data *chip,
				const char *pin,
				enum lm3648_platform_pin_state state)
{
	struct lm3648_platform_data *pdata = chip->pdata;
	int gpio_val;
	int i;

	for (i = 0; i < ARRAY_SIZE(pdata->gpios); i++) {
		if (pin == pdata->gpios[i].label) {
			if (!gpio_is_valid(pdata->gpios[i].pltfrm_gpio))
				return 0;
			if (state == PLTFRM_PIN_STATE_INACTIVE)
				gpio_val = (pdata->gpios[i].active_low ==
					OF_GPIO_ACTIVE_LOW) ? 1 : 0;
			else
				gpio_val = (pdata->gpios[i].active_low ==
					OF_GPIO_ACTIVE_LOW) ? 0 : 1;
			gpio_set_value(pdata->gpios[i].pltfrm_gpio, gpio_val);
			dev_dbg(chip->dev,
				"set GPIO #%d ('%s') to %s\n",
				pdata->gpios[i].pltfrm_gpio,
				pdata->gpios[i].label,
				gpio_val ? "HIGH" : "LOW");

			return 0;
		}
	}

	dev_err(chip->dev, "unknown pin '%s'\n", pin);
	return -EINVAL;
}

static int lm3648_init_gpio(struct lm3648_chip_data *chip)
{
	int ret = 0;
	struct lm3648_platform_data *pdata = chip->pdata;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(pdata->gpios); i++) {
		if (gpio_is_valid(pdata->gpios[i].pltfrm_gpio)) {
			ret = gpio_request_one(
				pdata->gpios[i].pltfrm_gpio,
				GPIOF_DIR_OUT,
				pdata->gpios[i].label);
			if (IS_ERR_VALUE(ret)) {
				dev_err(chip->dev,
					"failed to request GPIO #%d ('%s')\n",
					pdata->gpios[i].pltfrm_gpio,
					pdata->gpios[i].label);
				goto err;
			}

			if (pdata->gpios[i].label == PLTFRM_PIN_STROBE)
				ret = lm3648_set_pin_state(chip,
							   pdata->gpios[i].label,
							   PLTFRM_PIN_STATE_ACTIVE);
			else if (pdata->gpios[i].label == PLTFRM_PIN_HWEN)
				ret = lm3648_set_pin_state(chip,
							   pdata->gpios[i].label,
							   PLTFRM_PIN_STATE_ACTIVE);
			else
				ret = lm3648_set_pin_state(chip,
							   pdata->gpios[i].label,
							   PLTFRM_PIN_STATE_INACTIVE);
		}
	}
	return ret;
err:
	dev_err(chip->dev, "failed with error %d\n", ret);
	for (; i < ARRAY_SIZE(pdata->gpios); i++)
		pdata->gpios[i].pltfrm_gpio = -1;
	return ret;
}

/* chip initialize */
static int lm3648_chip_init(struct lm3648_chip_data *chip)
{
	return lm3648_init_gpio(chip);
}

/* chip control */
static int lm3648_control(struct lm3648_chip_data *chip,
			  u8 brightness, enum lm3648_mode opmode)
{
	int ret;

	ret = regmap_read(chip->regmap, REG_FLAG, &chip->last_flag);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read REG_FLAG Register\n");
		goto out;
	}

	if (chip->last_flag)
		dev_info(chip->dev, "Last FLAG is 0x%x\n", chip->last_flag);

	/* brightness 0 means off-state */
	if (!brightness)
		opmode = MODES_STASNDBY;

	switch (opmode) {
	case MODES_TORCH:
		ret = regmap_update_bits(chip->regmap, REG_TORCH_BRIGHT,
					 TORCH_BRIGHT_MASK << TORCH_BRIGHT_SHIFT,
					 (brightness - 1) << TORCH_BRIGHT_SHIFT);

		if (chip->torch_pin)
			opmode |= (TORCH_PIN_EN_MASK << TORCH_PIN_EN_SHIFT);
		break;

	case MODES_FLASH:
		ret = regmap_update_bits(chip->regmap, REG_FLASH_BRIGHT,
					 FLASH_BRIGHT_MASK << FLASH_BRIGHT_SHIFT,
					 (brightness - 1) << FLASH_BRIGHT_SHIFT);

		if (chip->strobe_pin)
			opmode |= (STROBE_PIN_EN_MASK << STROBE_PIN_EN_SHIFT);
		break;

	case MODES_STASNDBY:

		break;

	default:
		return ret;
	}
	if (ret < 0) {
		dev_err(chip->dev, "Failed to write REG_I_CTRL Register\n");
		goto out;
	}

	if (chip->tx_pin)
		opmode |= (TX_PIN_EN_MASK << TX_PIN_EN_SHIFT);

	ret = regmap_update_bits(chip->regmap, REG_ENABLE,
				 MODE_BITS_MASK << MODE_BITS_SHIFT,
				 opmode << MODE_BITS_SHIFT);
out:
	return ret;
}

/* torch */

/* torch pin config for lm3648*/
static ssize_t lm3648_torch_pin_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	ssize_t ret;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3648_chip_data *chip =
	    container_of(led_cdev, struct lm3648_chip_data, cdev_torch);
	unsigned int state;

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		goto out_strtoint;
	if (state != 0)
		state = 0x01 << TORCH_PIN_EN_SHIFT;

	chip->torch_pin = state;
	ret = regmap_update_bits(chip->regmap, REG_ENABLE,
				 TORCH_PIN_EN_MASK << TORCH_PIN_EN_SHIFT,
				 state);
	if (ret < 0)
		goto out;

	return size;
out:
	dev_err(chip->dev, "%s:i2c access fail to register\n", __func__);
	return ret;
out_strtoint:
	dev_err(chip->dev, "%s: fail to change str to int\n", __func__);
	return ret;
}

static DEVICE_ATTR(torch_pin, S_IWUSR, NULL, lm3648_torch_pin_store);

static void lm3648_deferred_torch_brightness_set(struct work_struct *work)
{
	struct lm3648_chip_data *chip =
	    container_of(work, struct lm3648_chip_data, work_torch);

	mutex_lock(&chip->lock);
	lm3648_control(chip, chip->br_torch, MODES_TORCH);
	mutex_unlock(&chip->lock);
}

static void lm3648_torch_brightness_set(struct led_classdev *cdev,
					enum led_brightness brightness)
{
	struct lm3648_chip_data *chip =
	    container_of(cdev, struct lm3648_chip_data, cdev_torch);

	chip->br_torch = brightness;
	schedule_work(&chip->work_torch);
}

/* flash */

/* strobe pin config for lm3648*/
static ssize_t lm3648_strobe_pin_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	ssize_t ret;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3648_chip_data *chip =
	    container_of(led_cdev, struct lm3648_chip_data, cdev_torch);
	unsigned int state;

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		goto out_strtoint;
	if (state != 0)
		state = 0x01 << STROBE_PIN_EN_SHIFT;

	chip->strobe_pin = state;
	ret = regmap_update_bits(chip->regmap, REG_ENABLE,
				 STROBE_PIN_EN_MASK << STROBE_PIN_EN_SHIFT,
				 state);
	if (ret < 0)
		goto out;

	return size;
out:
	dev_err(chip->dev, "%s:i2c access fail to register\n", __func__);
	return ret;
out_strtoint:
	dev_err(chip->dev, "%s: fail to change str to int\n", __func__);
	return ret;
}

static DEVICE_ATTR(strobe_pin, S_IWUSR, NULL, lm3648_strobe_pin_store);

static void lm3648_deferred_strobe_brightness_set(struct work_struct *work)
{
	struct lm3648_chip_data *chip =
	    container_of(work, struct lm3648_chip_data, work_flash);

	mutex_lock(&chip->lock);
	lm3648_control(chip, chip->br_flash, MODES_FLASH);
	mutex_unlock(&chip->lock);
}

static void lm3648_strobe_brightness_set(struct led_classdev *cdev,
					 enum led_brightness brightness)
{
	struct lm3648_chip_data *chip =
	    container_of(cdev, struct lm3648_chip_data, cdev_flash);

	chip->br_flash = brightness;
	schedule_work(&chip->work_flash);
}

static const struct regmap_config lm3648_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static int lm3648_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct lm3648_platform_data *pdata;
	struct lm3648_chip_data *chip;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c functionality check fail.\n");
		return -EOPNOTSUPP;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (IS_ERR_OR_NULL(pdata))
		return -ENOMEM;

	pdata->gpios[0].label = PLTFRM_PIN_STROBE;
	pdata->gpios[0].pltfrm_gpio = of_get_named_gpio_flags(
		np,
		pdata->gpios[0].label,
		0,
		&pdata->gpios[0].active_low);

	pdata->gpios[1].label = PLTFRM_PIN_HWEN;
	pdata->gpios[1].pltfrm_gpio = of_get_named_gpio_flags(
		np,
		pdata->gpios[1].label,
		0,
		&pdata->gpios[1].active_low);

	chip = devm_kzalloc(&client->dev,
			    sizeof(struct lm3648_chip_data), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &client->dev;
	chip->pdata = pdata;

	chip->regmap = devm_regmap_init_i2c(client, &lm3648_regmap);
	if (IS_ERR(chip->regmap)) {
		err = PTR_ERR(chip->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			err);
		return err;
	}

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	err = lm3648_chip_init(chip);
	if (err < 0)
		goto err_out;

	/* flash */
	INIT_WORK(&chip->work_flash, lm3648_deferred_strobe_brightness_set);
	chip->cdev_flash.name = "flash";
	chip->cdev_flash.max_brightness = FLASH_BRIGHT_MAX;
	chip->cdev_flash.brightness_set = lm3648_strobe_brightness_set;
	chip->cdev_flash.default_trigger = "flash";
	err = led_classdev_register((struct device *)
				    &client->dev, &chip->cdev_flash);
	if (err < 0) {
		dev_err(chip->dev, "failed to register flash\n");
		goto err_out;
	}
	err = device_create_file(chip->cdev_flash.dev, &dev_attr_strobe_pin);
	if (err < 0) {
		dev_err(chip->dev, "failed to create strobe-pin file\n");
		goto err_create_flash_pin_file;
	}

	/* torch */
	INIT_WORK(&chip->work_torch, lm3648_deferred_torch_brightness_set);
	chip->cdev_torch.name = "torch";
	chip->cdev_torch.max_brightness = TORCH_BRIGHT_MAX;
	chip->cdev_torch.brightness_set = lm3648_torch_brightness_set;
	chip->cdev_torch.default_trigger = "torch";
	err = led_classdev_register((struct device *)
				    &client->dev, &chip->cdev_torch);
	if (err < 0) {
		dev_err(chip->dev, "failed to register torch\n");
		goto err_create_torch_file;
	}
	err = device_create_file(chip->cdev_torch.dev, &dev_attr_torch_pin);
	if (err < 0) {
		dev_err(chip->dev, "failed to create torch-pin file\n");
		goto err_create_torch_pin_file;
	}

	dev_info(&client->dev, "LM3648 is initialized\n");
	return 0;

err_create_torch_pin_file:
	led_classdev_unregister(&chip->cdev_torch);
err_create_torch_file:
	device_remove_file(chip->cdev_flash.dev, &dev_attr_strobe_pin);
err_create_flash_pin_file:
	led_classdev_unregister(&chip->cdev_flash);
err_out:
	return err;
}

static int lm3648_remove(struct i2c_client *client)
{
	struct lm3648_chip_data *chip = i2c_get_clientdata(client);

	device_remove_file(chip->cdev_torch.dev, &dev_attr_torch_pin);
	led_classdev_unregister(&chip->cdev_torch);
	flush_work(&chip->work_torch);
	device_remove_file(chip->cdev_flash.dev, &dev_attr_strobe_pin);
	led_classdev_unregister(&chip->cdev_flash);
	flush_work(&chip->work_flash);
	regmap_write(chip->regmap, REG_ENABLE, 0);
	return 0;
}

static const struct i2c_device_id lm3648_id[] = {
	{LM3648_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lm3648_id);

static struct i2c_driver lm3648_i2c_driver = {
	.driver = {
		   .name = LM3648_NAME,
		   .owner = THIS_MODULE,
		   .pm = NULL,
		   },
	.probe = lm3648_probe,
	.remove = lm3648_remove,
	.id_table = lm3648_id,
};

module_i2c_driver(lm3648_i2c_driver);

MODULE_DESCRIPTION("Texas Instruments Flash Lighting driver for LM3648");
MODULE_AUTHOR("Daniel Jeong <daniel.jeong@ti.com>");
MODULE_AUTHOR("G.Shark Jeong <gshark.jeong@gmail.com>");
MODULE_LICENSE("GPL v2");
