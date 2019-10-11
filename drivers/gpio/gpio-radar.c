#include <linux/module.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/device.h>
#include <linux/semaphore.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/rk_keys.h>
#include <linux/of_platform.h>
#include <linux/input.h>
//#define Delay_50us(x) usleep(x*50)
#define	Delay_1ms(x)	msleep(x)
#define	Delay_50us(x)	usleep_range(50*x,50*x+20)

#define RADAR_NAME				"firefly_radar"
#define RADAR_LED_TEST 			0
/*
refer to /include/dt-bindings/input/rk-input.h
key event range 0 - 0x2ff
*/
#define KEY_RADAR_IN			0x2fe
#define KEY_RADAR_OUT			0x2fd

#define RADAR_KEYEVENT_PRESS			1
#define RADAR_KEYEVENT_RELAESE			0

static int radar_value;
struct input_dev *radar_input_temp = NULL;
struct platform_device *cur_pdev = NULL;
static struct work_struct work;
static bool radar_work_status = false;

#if RADAR_LED_TEST
static int test_led_gpio;
#endif

struct firefly_gpio_info 
{
	int irq_gpio_num;
	int irq_num;
	int irq_mode;
};

static struct firefly_gpio_info *gpio_info;

static void work_handler(struct work_struct *data)
{
	int ret;
	// input_set_capability(radar_input_temp, EV_KEY, KEY_OK);
	// input_set_capability(radar_input_temp, EV_KEY, KEY_RADAR_IN);
	// input_set_capability(radar_input_temp, EV_KEY, KEY_RADAR_OUT);
	ret = gpio_get_value(gpio_info->irq_gpio_num);
	if(ret)
	{
#if RADAR_LED_TEST
		gpio_direction_output(test_led_gpio, 1);
#endif
		//printk("\r\n\r\n==== Enter radar gpio irq test program! high ====\r\n\r\n");
		input_report_key(radar_input_temp, KEY_RADAR_IN, RADAR_KEYEVENT_PRESS);
		input_report_key(radar_input_temp, KEY_RADAR_IN, RADAR_KEYEVENT_RELAESE);
		input_sync(radar_input_temp);
	}
	else
	{
#if RADAR_LED_TEST
		gpio_direction_output(test_led_gpio, 0);
#endif
		//printk("\r\n\r\n =====Enter radar gpio irq test program!low ====\r\n\r\n");
		input_report_key(radar_input_temp, KEY_RADAR_OUT, RADAR_KEYEVENT_PRESS);
		input_report_key(radar_input_temp, KEY_RADAR_OUT, RADAR_KEYEVENT_RELAESE);
		input_sync(radar_input_temp);
		//printk("Enter radar gpio irq test program! low\r\n");
	}
}

static irqreturn_t radar_gpio_irq(int irq, void *dev_id)                                                                                 
{
	INIT_WORK(&work,work_handler);
	schedule_work(&work);
	radar_work_status = true;
	return IRQ_HANDLED;
}

static ssize_t radargpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	
	unsigned int value = 0;
	if(radar_work_status)
	{
		value = gpio_get_value(gpio_info->irq_gpio_num);
	}
	return sprintf(buf,"%d\n", value);
}

static ssize_t radargpio_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	return count;
}



static DEVICE_ATTR(radargpio, 0644, radargpio_show, radargpio_store);

static struct attribute *radargpio_attributes[] = {
	&dev_attr_radargpio.attr,
	NULL
};


static struct attribute_group radargpio_attribute_group = {
	.attrs =  radargpio_attributes
};

static struct of_device_id radar_of_match[] = {
	{ .compatible = "firefly,radar" },
	{ }
};
MODULE_DEVICE_TABLE(of, radar_of_match);

static int radar_probe(struct platform_device *pdev){
	int ret =0;
	int gpio,error;
	enum of_gpio_flags flag;
	struct input_dev *radar_input = NULL;

	radar_input = input_allocate_device();
	if (!radar_input) {
		printk("input_allocate_device failed!\n");
		return -ENOMEM;
	}


	radar_input->name = RADAR_NAME;
	radar_input->id.bustype = BUS_HOST;

	radar_input->phys = "gpio-keys/radar";
	radar_input->id.vendor = 0x0001;
	radar_input->id.product = 0x0001;
	radar_input->id.version = 0x0100;

	radar_input->dev.parent = &pdev->dev;
	input_set_drvdata(radar_input, &radar_value);
	radar_input_temp = radar_input;

	input_set_capability(radar_input_temp, EV_KEY, KEY_OK);
	input_set_capability(radar_input_temp, EV_KEY, KEY_RADAR_IN);
	input_set_capability(radar_input_temp, EV_KEY, KEY_RADAR_OUT);
	error = input_register_device(radar_input);

	if (error)
		pr_err("radar: register input device err, error: %d\n", error);

	gpio_info = devm_kzalloc(&pdev->dev,sizeof(struct firefly_gpio_info *), GFP_KERNEL);
	if (!gpio_info) {
		dev_err(&pdev->dev, "devm_kzalloc failed!\n");
		return -ENOMEM;
	}



#if RADAR_LED_TEST	
	test_led_gpio = of_get_named_gpio_flags(pdev->dev.of_node, "radar-led-gpio", 0, &flag);
	printk("test_led_gpio gpio:%d\r\n",gpio);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "firefly-irq-gpio: %d is invalid\n", gpio);
		return -ENODEV;
	}
#endif

	gpio = of_get_named_gpio_flags(pdev->dev.of_node, "radar-irq-gpio", 0, &flag);
	printk("radar gpio:%d\r\n",gpio);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "firefly-irq-gpio: %d is invalid\n", gpio);
		return -ENODEV;
	}

	gpio_info->irq_gpio_num = gpio;
	gpio_info->irq_mode = flag;
	gpio_info->irq_num = gpio_to_irq(gpio_info->irq_gpio_num);
	if (gpio_info->irq_num) 
	{
		if (gpio_request(gpio, "radar-irq-gpio"))
		{
			dev_err(&pdev->dev, "radar-irq-gpio: %d request failed!\n", gpio);
			gpio_free(gpio);
			return IRQ_NONE;
		}

		ret = request_irq(gpio_info->irq_num, radar_gpio_irq,
			flag, "radar-irq-gpio", gpio_info);
		if (ret != 0) 
		{
			free_irq(gpio_info->irq_num, gpio_info);
				dev_err(&pdev->dev, "Failed to request IRQ: %d\n", ret);
		}
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &radargpio_attribute_group);

	printk("radar_probe, end ret=%d \n",ret);
	return ret;
}

static int radar_remove(struct platform_device *pdev){
	printk("radar_remove, \n");
	free_irq(gpio_info->irq_num, gpio_info);
	gpio_free(gpio_info->irq_gpio_num);
	devm_kfree(&pdev->dev, gpio_info);
	// 删除设备节点
	//device_unregister(dev);
	// 销毁类
	//class_destroy(cls);
	// 取消注册设备驱动
	//unregister_chrdev(major, "wiegandOut");
	// 取消内存映射
	//gpio_reset();
	return 0;
}

struct platform_driver radar_drv = {
	.probe	= radar_probe,//匹配到dev之后调用probe
	.remove = radar_remove,
	.driver = {
		.name = "radar",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(radar_of_match),
	},
};

static int __init radar_init(void)
{
	int ret =0;
	//printk(KERN_ALERT"radar_init function.\n");
	ret=platform_driver_register(&radar_drv);
	printk(KERN_ALERT"radar end.ret=%d \n",ret);

	return 0;
}

static void __exit radar_exit(void)
{
	platform_driver_unregister(&radar_drv);
	//destroy_workqueue(queue);
}
MODULE_LICENSE("GPL");
module_init(radar_init);
module_exit(radar_exit);
