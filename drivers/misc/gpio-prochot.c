/*
 * Toggles a GPIO pin to protect a device
 *
 * Copyright (C) 2014 Google, Inc.
 * Copyright (C) 2015 Rockchip, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of_platform.h>
#include <linux/module.h>

struct gpio_prochot {
	struct gpio_desc *prochot_gpio;
	struct notifier_block cpufreq_clamp_handler;
	bool clamp_active;
};

static int gpio_prochot_notify(struct notifier_block *this,
				unsigned long mode, void *data)
{
	struct cpufreq_policy *p = data;
	unsigned long clamp_freq;

	struct gpio_prochot *gpio_prochot =
		container_of(this, struct gpio_prochot, cpufreq_clamp_handler);

	if (gpio_prochot->clamp_active)  {
		clamp_freq = p->cpuinfo.min_freq;
		cpufreq_verify_within_limits(p, 0, clamp_freq);
	}

	return NOTIFY_DONE;
}

static irqreturn_t gpio_prochot_irq_thread(int irq, void *dev)
{
	struct gpio_prochot *gpio_prochot = dev;

	gpio_prochot->clamp_active =
			gpiod_get_value_cansleep(gpio_prochot->prochot_gpio);

	return IRQ_HANDLED;
}

static int gpio_prochot_probe(struct platform_device *pdev)
{
	struct gpio_prochot *gpio_prochot;
	int irq;
	int ret;

	gpio_prochot = devm_kzalloc(&pdev->dev, sizeof(*gpio_prochot),
			GFP_KERNEL);
	if (!gpio_prochot)
		return -ENOMEM;

	gpio_prochot->prochot_gpio = devm_gpiod_get(&pdev->dev, "prochot");
	if (IS_ERR(gpio_prochot->prochot_gpio))
		return PTR_ERR(gpio_prochot->prochot_gpio);

	gpio_prochot->cpufreq_clamp_handler.notifier_call = gpio_prochot_notify;
	gpio_prochot->clamp_active =
			gpiod_get_value_cansleep(gpio_prochot->prochot_gpio);

	platform_set_drvdata(pdev, gpio_prochot);

	irq = gpiod_to_irq(gpio_prochot->prochot_gpio);

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
				  &gpio_prochot_irq_thread,
				  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING |
				  IRQF_ONESHOT,  "prochot", gpio_prochot);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to request prochot irq: %d\n", ret);
		return ret;
	}

	ret = cpufreq_register_notifier(&gpio_prochot->cpufreq_clamp_handler,
				  CPUFREQ_POLICY_NOTIFIER);
	if (ret) {
		dev_err(&pdev->dev, "%s: cannot register cpufreq clamp handler, %d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int gpio_prochot_remove(struct platform_device *pdev)
{
	struct gpio_prochot *gpio_prochot = platform_get_drvdata(pdev);
	int ret;

	ret = cpufreq_unregister_notifier(&gpio_prochot->cpufreq_clamp_handler,
					  CPUFREQ_POLICY_NOTIFIER);
	if (ret) {
		dev_err(&pdev->dev,
			"%s: cannot unregister cpufreq clamp handler, %d\n",
			__func__, ret);
		BUG();
	}

	return 0;
}

static const struct of_device_id of_gpio_prochot_match[] = {
	{ .compatible = "gpio-prochot", },
	{},
};

static struct platform_driver gpio_prochot_driver = {
	.probe = gpio_prochot_probe,
	.remove = gpio_prochot_remove,
	.driver = {
		.name = "prochot-gpio",
		.owner = THIS_MODULE,
		.of_match_table = of_gpio_prochot_match,
	},
};

module_platform_driver(gpio_prochot_driver);

MODULE_AUTHOR("Caesar Wang <wxt@rock-chips.com>");
MODULE_DESCRIPTION("GPIO prochot driver");
MODULE_LICENSE("GPL");
