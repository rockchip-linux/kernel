#include <linux/init.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

static int rk_wifi_reset_probe(struct platform_device *pdev){
	int ret = 0;
	struct device_node *sg_node;
	struct gpio_desc *wifi_reset;

	sg_node = pdev->dev.of_node;

	dev_dbg(&pdev->dev, "%s %d\n", __FUNCTION__, __LINE__);

	wifi_reset = gpiod_get(&pdev->dev, "wifi-reset");
	if (IS_ERR(wifi_reset)) {
		dev_warn(&pdev->dev, "there is no wifi-reset gpio\n");
		return -ENODEV;
	}

	gpiod_direction_output(wifi_reset, 1);
	msleep(50);
	gpiod_direction_output(wifi_reset, 0);
	msleep(150);
	gpiod_direction_output(wifi_reset, 1);

	return ret;
}

static const struct of_device_id rk_wifi_reset_gpio_dt_ids[] = {
	{.compatible = "rockchip,wifi-reset-wdt"},
	{}
};

static struct platform_driver rk_wifi_reset_gpio = {
	.probe	= rk_wifi_reset_probe,
	.driver	= {
		.name	= "rk_wifi_reset",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(rk_wifi_reset_gpio_dt_ids),
	},
};

#if 0
static int __init rk_wifi_reset_control_init(void){
	int ret = 0;
	dev_dbg(&pdev->dev,"%s %d\n", __FUNCTION__, __LINE__);
	ret = platform_driver_register(&rk_wifi_reset_gpio);

	printk("%s ret = %d\n", __FUNCTION__, ret);
	return ret;
}

static void __exit rk_wifi_reset_control_exit(void){
	platform_driver_unregister(&rk_wifi_reset_gpio);
}

late_initcall(rk_wifi_reset_control_init);
module_exit(rk_wifi_reset_control_exit);
#else
module_platform_driver(rk_wifi_reset_gpio);
#endif
MODULE_DESCRIPTION ("sofia sgpio driver");
MODULE_LICENSE("GPL");
