#include <linux/init.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

static int gpio_reset_probe(struct platform_device *pdev){
	int ret = 0, gpio_num, time, i;
	struct device_node *sg_node;
	struct gpio_desc **reset_gpio;

	sg_node = pdev->dev.of_node;

	dev_dbg(&pdev->dev, "%s %d\n", __FUNCTION__, __LINE__);

	gpio_num = of_gpio_named_count(pdev->dev.of_node, "reset-gpios");
	if (gpio_num < 1)
		return -EINVAL;

	reset_gpio = devm_kzalloc(&pdev->dev, sizeof(*reset_gpio) * gpio_num,
				  GFP_KERNEL);

	for (i = 0; i < gpio_num; i++) {
		reset_gpio[i] = gpiod_get_index(&pdev->dev, "reset", i);
		if (IS_ERR(reset_gpio[i])) {
			dev_warn(&pdev->dev, "there is no reset_gpio gpio\n");
			return -ENODEV;
		}
		gpiod_direction_output(reset_gpio[i], 0);
	}

	if (!of_property_read_u32(pdev->dev.of_node, "delay-time", &time))
		msleep(time);

	for (i = 0; i < gpio_num; i++) {
		gpiod_direction_output(reset_gpio[i], 1);
	}

	return ret;
}

static const struct of_device_id gpio_reset_dt_ids[] = {
	{.compatible = "rockchip,gpio-reset"},
	{}
};

static struct platform_driver gpio_reset = {
	.probe	= gpio_reset_probe,
	.driver	= {
		.name	= "gpio_reset",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(gpio_reset_dt_ids),
	},
};

#if 0
static int __init gpio_reset_control_init(void){
	int ret = 0;
	dev_dbg(&pdev->dev,"%s %d\n", __FUNCTION__, __LINE__);
	ret = platform_driver_register(&gpio_reset);

	printk("%s ret = %d\n", __FUNCTION__, ret);
	return ret;
}

static void __exit gpio_reset_control_exit(void){
	platform_driver_unregister(&gpio_reset);
}

late_initcall(gpio_reset_control_init);
module_exit(gpio_reset_control_exit);
#else
module_platform_driver(gpio_reset);
#endif
MODULE_DESCRIPTION ("gpio reset driver");
MODULE_LICENSE("GPL");
