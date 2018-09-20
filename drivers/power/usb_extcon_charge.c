#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/workqueue.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/reboot.h>

static int debug_enable=1;
module_param(debug_enable, int, 0644);

enum USB_STATUS {
	NOT_CONNECT,
	UFP,
	PD
};

#define TIMER_MS_COUNTS			1000
#define DEFAULT_MONITOR_SEC		2

struct usb_ext_charge {
	struct device *dev;
	struct workqueue_struct *battery_workqueue;
	struct iio_channel *channel;
	struct extcon_dev *ext_dev;
	struct notifier_block		cable_pd_nb;
	struct delayed_work		pd_work;
	struct delayed_work		battery_delay_work;
	struct gpio_desc *gpio_charge_en;
	struct gpio_desc *gpio_charge_current_ctl;
	struct gpio_desc *gpio_poe_state;
	struct gpio_desc *gpio_led;
	bool battery_online;
	bool battery_low;
	bool battery_off;
	bool battery_adc_check;
	bool charge_online;
	enum USB_STATUS usb_status;
	//bool usb_online;
	bool poe_online;
	int irq_pin;
	int irq;
	int monitor_sec;
	int bat_det_count;
	int bat_one_min;
	int bat_one_min_status;
	int barttery_adc;
};

#define uec_printk(args...) \
	do { \
		if (debug_enable) { \
			dev_info(uec->dev, args); \
		} \
	} while (0)

static void uec_update_battery_charge(struct usb_ext_charge *uec);

static int uec_parse_dt(struct usb_ext_charge *uec)
{
	struct device *dev = uec->dev;
	struct device_node *node = dev->of_node;
	struct iio_channel *chan;
	struct extcon_dev *edev;

	if (!node)
		return -ENODEV;

/* type-C */
	edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR(edev)) {
		if (PTR_ERR(edev) != -EPROBE_DEFER)
			dev_err(uec->dev, "Invalid or missing extcon dev0\n");
		uec->ext_dev = NULL;
		dev_err(uec->dev, "Invalid or missing extcon dev0\n");
	} else {
		uec->ext_dev = edev;
	}

	chan = iio_channel_get(dev, NULL);
	if (IS_ERR(chan)) {
		dev_info(dev, "no io-channels defined\n");
		chan = NULL;
	}
	uec->channel = chan;

	uec->gpio_charge_en = devm_gpiod_get_optional(uec->dev, "charge-en",
						       GPIOD_OUT_LOW);
	if (IS_ERR(uec->gpio_charge_en)) {
		dev_warn(uec->dev,
			 "Could not get named GPIO for charge enable!\n");
		uec->gpio_charge_en = NULL;
	}

	uec->gpio_charge_current_ctl = devm_gpiod_get_optional(uec->dev, "cur-ctl",
						       GPIOD_OUT_LOW);
	if (IS_ERR(uec->gpio_charge_current_ctl)) {
		dev_warn(uec->dev,
			 "Could not get named GPIO for charge enable!\n");
		uec->gpio_charge_current_ctl = NULL;
	}

	uec->gpio_poe_state = devm_gpiod_get_optional(uec->dev, "poe-state", GPIOD_IN);
	if (IS_ERR(uec->gpio_poe_state)) {
		dev_warn(uec->dev, "Could not get poe detect GPIO\n");
		uec->gpio_poe_state = NULL;
	}

	uec->gpio_led = devm_gpiod_get_optional(uec->dev, "cap-led", GPIOD_OUT_LOW);
	if (IS_ERR(uec->gpio_poe_state)) {
		dev_warn(uec->dev, "Could not get cap-led detect GPIO\n");
		uec->gpio_led = NULL;
	}

	return 0;
}

static int uec_adc_iio_read(struct usb_ext_charge *uec)
{
	struct iio_channel *channel = uec->channel;
	int val, ret;

	if (!channel)
		return -1;
	ret = iio_read_channel_raw(channel, &val);
	if (ret < 0) {
		pr_err("read channel() error: %d\n", ret);
		return ret;
	}
	return val;
}


static void uec_update_battery_charge(struct usb_ext_charge *uec)
{
	bool enable_charge = false;
	bool high_current_charge = false;
	bool power_off = false;
	bool enable_led = false;

	if (uec->battery_online)
	{
		if (uec->poe_online) {
			enable_charge = true;
			high_current_charge = true;
		} else if (uec->usb_status > NOT_CONNECT) {
			enable_charge = true;
			if (uec->usb_status == PD)
				high_current_charge = true;
			else
				high_current_charge = false;

			if( (uec->usb_status == UFP) && (uec->bat_one_min_status == 0) )
			{
				enable_charge = false;
			}

		} else {
			if (uec->battery_low == true)
			{
				enable_led = true;
			}
			else if(uec->battery_off == true)
			{
				power_off = true;
			}
			else
			{
				enable_led = false;
				power_off = false;
			}
			enable_charge = false;
			high_current_charge = false;
		}
	} else {
		enable_charge = false;
		high_current_charge = false;
	}

	uec_printk(" usb status %d battery_online %d poe_online %d enable_led %d power_off %d\n ",
				uec->usb_status, uec->battery_online, uec->poe_online, enable_led, power_off);

	if (enable_charge) {
		if (high_current_charge) {
			if (uec->gpio_charge_current_ctl != NULL)
				gpiod_set_value(uec->gpio_charge_current_ctl, 1);
		} else {
			if (uec->gpio_charge_current_ctl != NULL)
				gpiod_set_value(uec->gpio_charge_current_ctl, 0);
		}

		if (uec->gpio_charge_en != NULL)
			gpiod_set_value(uec->gpio_charge_en, 1);

	} else {
		if (uec->gpio_charge_en != NULL)
			gpiod_set_value(uec->gpio_charge_en, 0);
		if (uec->gpio_charge_current_ctl != NULL)
			gpiod_set_value(uec->gpio_charge_current_ctl, 0);
	}
	if(enable_led == true)
	{
		if (uec->gpio_led != NULL)
			gpiod_set_value(uec->gpio_led, 1);
	}
	else
	{
		if (uec->gpio_led != NULL)
			gpiod_set_value(uec->gpio_led, 0);
	}
	if(power_off == true)
	{
			kernel_power_off();
	}
}

static void uec_get_extcon_status(struct usb_ext_charge *uec)
{
	bool ufp;
	bool pd;
	enum USB_STATUS usb_status;

	dev_info(uec->dev, "uec get notify\n");
	ufp = extcon_get_state(uec->ext_dev, EXTCON_USB);
	pd = extcon_get_state(uec->ext_dev, EXTCON_CHG_USB_FAST);
	dev_info(uec->dev, "uec get ufp %d pd %d \n", ufp, pd);

	if (pd)
		usb_status = PD;
	else if (ufp)
		usb_status = UFP;
	else
		usb_status = NOT_CONNECT;

	if (uec->usb_status != usb_status) {
		uec->usb_status = usb_status;
		uec_update_battery_charge(uec);
	}
}

static void uec_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct usb_ext_charge *uec;
	bool battery_online = false;
	bool poe_online = false;
	bool battery_off =false;
	bool battery_low = false;
	int adc_val = 0;

	delay_work = container_of(work, struct delayed_work, work);
	uec = container_of(delay_work, struct usb_ext_charge, battery_delay_work);

	adc_val = uec_adc_iio_read(uec);
	printk("adc_val,%d\r\n",adc_val);
	if (unlikely(adc_val < 0)) {
		dev_err(uec->dev, "get battery adc value fail\n");
	} else if (adc_val > 100) {
		if (!uec->battery_online) {
			if ( ++uec->bat_det_count > 1) {
				battery_online = true;
				uec->bat_det_count = 0;
			}
		} else
			battery_online = true;

			if (adc_val < 600){
				battery_off = true;
			}
			else if (adc_val < 700){
				battery_low = true;
			}
			else{
				battery_low = false;
				battery_off = false;
			}
	} else {
		battery_online = false;
		uec->bat_det_count = 0;
	}

	if (uec->gpio_poe_state != NULL)
		poe_online = (gpiod_get_raw_value(uec->gpio_poe_state) == 1);

	if ( uec->bat_one_min_status == 0)
	{
		if ( (++uec->bat_one_min < 20) && (uec->bat_one_min > 2))//20秒
		{
			battery_online = true;
		}
		else if( uec->bat_one_min >= 10 )
		{
			uec->bat_one_min_status = 1;
		}
	}

	if ((battery_online != uec->battery_online) ||
		(poe_online != uec->poe_online) ||
		(battery_off != uec->battery_off) ||
		(battery_low != uec->battery_low)) {
		uec_printk("adc value = %d\n", adc_val);
		uec->battery_online = battery_online;
		uec->poe_online = poe_online;
		uec->battery_off = battery_off;
		uec->battery_low = battery_low;
		uec_update_battery_charge(uec);
	}

	queue_delayed_work(uec->battery_workqueue,
			   &uec->battery_delay_work,
			   msecs_to_jiffies(uec->monitor_sec));
}


static void uec_pd_evt_worker(struct work_struct *work)
{
	struct usb_ext_charge* uec = container_of(work,
				struct usb_ext_charge, pd_work.work);

	uec_get_extcon_status(uec);
}

static int uec_pd_evt_notifier(struct notifier_block *nb,
				   unsigned long event, void *ptr)
{

	struct usb_ext_charge *uec =
		container_of(nb, struct usb_ext_charge, cable_pd_nb);

	queue_delayed_work(uec->battery_workqueue, &uec->pd_work,
		   msecs_to_jiffies(100));

	return NOTIFY_DONE;
}

static irqreturn_t uec_charge_irq(int irq, void *dev_id)
{
	struct usb_ext_charge *uec = (struct usb_ext_charge *)(dev_id);

	dev_err(uec->dev, "interrupt happend\n");

	if (!uec->battery_online) {
		uec->battery_online = true;
		uec_update_battery_charge(uec);
	}
	return IRQ_HANDLED;
}



static int usb_ext_charge_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct usb_ext_charge *uec;
	int ret = 0;
	unsigned long irq_flags;

	uec = devm_kzalloc(dev, sizeof(*uec), GFP_KERNEL);
	if (!uec) {
		dev_err(dev,
			"fail to allocate memory for usb ext charge\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, uec);
	dev_set_drvdata(&pdev->dev, uec);
	uec->dev =  dev;

	ret = uec_parse_dt(uec);
	if (ret) {
		dev_err(dev, "parse usb ext charge device fail %d\n", ret);
		return -1;
	}

	uec->irq_pin = of_get_named_gpio_flags(np, "bat-int", 0, (enum of_gpio_flags *)&irq_flags);
    if (gpio_is_valid(uec->irq_pin)) {
		uec->irq = gpio_to_irq(uec->irq_pin);		/*If not defined in client*/
		if (uec->irq) {
			ret = devm_request_irq(dev, uec->irq,
			uec_charge_irq, irq_flags | IRQF_ONESHOT, "uec", uec);
			if (ret != 0) {
				dev_err(uec->dev, "Cannot allocate ts INT!ERRNO:%d\n", ret);
			}
		}
	} else {
		dev_err(uec->dev, "get INT!ERRNO:\n");
	}

	disable_irq(uec->irq);

	uec->monitor_sec = DEFAULT_MONITOR_SEC * TIMER_MS_COUNTS;

	uec->battery_workqueue = create_singlethread_workqueue("usb_ext_charge");
	INIT_DELAYED_WORK(&uec->battery_delay_work, uec_work);

	uec->charge_online = false;
	uec->battery_online = false;
	uec->poe_online = false;
	uec->battery_off = false;
	uec->battery_low = false;
	//uec->usb_online = false;
	uec->usb_status = NOT_CONNECT;
	if (uec->ext_dev != NULL) {
		dev_info(dev, "register ext0 dev0\n");

		INIT_DELAYED_WORK(&uec->pd_work, uec_pd_evt_worker);
		uec->cable_pd_nb.notifier_call = uec_pd_evt_notifier;

		extcon_register_notifier(uec->ext_dev,
					 EXTCON_USB,
					 &uec->cable_pd_nb);

		extcon_register_notifier(uec->ext_dev,
					 EXTCON_CHG_USB_FAST,
					 &uec->cable_pd_nb);

		uec_get_extcon_status(uec);
	} else {
		dev_err(dev, "no ext0 find\n");
	}

	queue_delayed_work(uec->battery_workqueue,
			   &uec->battery_delay_work, msecs_to_jiffies(10));

	dev_info(dev, "probe success\n");
	return 0;
}

static int usb_ext_charge_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct usb_ext_charge *uec = dev_get_drvdata(dev);

	cancel_delayed_work(&uec->battery_delay_work);
	return 0;
}


static const struct of_device_id usb_ext_charge_match[] = {
	{ .compatible = "usb-ext-charge", .data = NULL},
	{},
};

static struct platform_driver usb_ext_charge_driver = {
	.probe		= usb_ext_charge_probe,
	.remove		= usb_ext_charge_remove,
	.driver		= {
		.name	= "usb-ext-charge",
		.owner	= THIS_MODULE,
		.of_match_table = usb_ext_charge_match,
	}
};

//module_platform_driver(usb_ext_charge_driver);

static int __init usb_ext_charge_driver_init(void)
{
	return platform_driver_register(&usb_ext_charge_driver);
}

static void __exit usb_ext_charge_driver_exit(void)
{
	platform_driver_unregister(&usb_ext_charge_driver);
}

late_initcall_sync(usb_ext_charge_driver_init);
module_exit(usb_ext_charge_driver_exit);
