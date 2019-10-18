#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/leds.h>
#include <linux/pwm.h>

DEFINE_LED_TRIGGER(ledtrig_ir_click);
static int rk_led_test_probe(struct platform_device *pdev)
{
    led_trigger_register_simple("ir-user-click", &ledtrig_ir_click);
    led_trigger_event(ledtrig_ir_click,LED_FULL);
    printk("\r\n\r\nrk_led_test_probe\r\n\r\n");
    return 0;
}

static int rk_led_test_remove(struct platform_device *pdev)
{
    led_trigger_unregister_simple(ledtrig_ir_click);
    return 0;
}

static const struct of_device_id rk_led_test_match[] = {
    { .compatible = "firefly,rk3399-led" },
    {},
};

MODULE_DEVICE_TABLE(of, rk_led_test_match);

static struct platform_driver rk_led_test_driver = {
    .probe      = rk_led_test_probe,
    .remove     = rk_led_test_remove,
    .driver = {
        .name = "rk-led-test",
        .owner  = THIS_MODULE,
        .of_match_table = rk_led_test_match,
    }
};

module_platform_driver(rk_led_test_driver);
MODULE_AUTHOR("zjy <service@t-firefly.com>");
MODULE_DESCRIPTION("Firefly LED demo driver");
MODULE_ALIAS("platform:firefly-led");
MODULE_LICENSE("GPL");
