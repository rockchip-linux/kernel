/*
 *	Real Time Clock driver for  rk816
 *
 *  Author: zhangqing <zhangqing@rock-chips.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/bcd.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/completion.h>
#include <linux/mfd/rk816.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/regmap.h>

/* RTC Definitions */
/* RTC_CTRL_REG bitfields */
#define BIT_RTC_CTRL_REG_STOP_RTC_M		0x01
#define BIT_RTC_CTRL_REG_ROUND_30S_M		0x02
#define BIT_RTC_CTRL_REG_AUTO_COMP_M		0x04
#define BIT_RTC_CTRL_REG_MODE_12_24_M		0x08
#define BIT_RTC_CTRL_REG_TEST_MODE_M		0x10
#define BIT_RTC_CTRL_REG_SET_32_COUNTER_M	0x20
#define BIT_RTC_CTRL_REG_GET_TIME_M		0x40
#define BIT_RTC_CTRL_REG_RTC_V_OPT_M		0x80

/* RTC_STATUS_REG bitfields */
#define BIT_RTC_STATUS_REG_RUN_M		0x02
#define BIT_RTC_STATUS_REG_1S_EVENT_M		0x04
#define BIT_RTC_STATUS_REG_1M_EVENT_M		0x08
#define BIT_RTC_STATUS_REG_1H_EVENT_M		0x10
#define BIT_RTC_STATUS_REG_1D_EVENT_M		0x20
#define BIT_RTC_STATUS_REG_ALARM_M		0x40
#define BIT_RTC_STATUS_REG_POWER_UP_M		0x80

/* RTC_INTERRUPTS_REG bitfields */
#define BIT_RTC_INTERRUPTS_REG_EVERY_M		0x03
#define BIT_RTC_INTERRUPTS_REG_IT_TIMER_M	0x04
#define BIT_RTC_INTERRUPTS_REG_IT_ALARM_M	0x08

/* REG_SECONDS_REG through REG_YEARS_REG is how many registers? */
#define ALL_TIME_REGS				7
#define ALL_ALM_REGS				6

#define RTC_SET_TIME_RETRIES			5
#define RTC_GET_TIME_RETRIES			5

struct rk816_rtc {
	struct rk816 *rk816;
	struct rtc_device *rtc;
	unsigned int alarm_enabled:1;
};

/*
 * Read current time and date in RTC
 */
static int rk816_rtc_readtime(struct device *dev, struct rtc_time *tm)
{
	struct rk816_rtc *rk816_rtc = dev_get_drvdata(dev);
	struct rk816 *rk816 = rk816_rtc->rk816;
	int ret;
	/*int count = 0;*/
	unsigned char rtc_data[ALL_TIME_REGS + 1];
	u8 rtc_ctl;

	/* Has the RTC been programmed? */
	ret = rk816_reg_read(rk816, RK816_RTC_CTRL_REG);
	if (ret < 0) {
		dev_err(dev, "Failed to read RTC control: %d\n", ret);
		return ret;
	}

	rtc_ctl = ret & (~BIT_RTC_CTRL_REG_RTC_V_OPT_M);
	ret = rk816_reg_write(rk816, RK816_RTC_CTRL_REG, rtc_ctl);
	if (ret < 0) {
		dev_err(dev, "Failed to write RTC control: %d\n", ret);
		return ret;
	}

	rtc_data[0] = rk816_reg_read(rk816, RK816_SECONDS_REG);
	rtc_data[1] = rk816_reg_read(rk816, RK816_MINUTES_REG);
	rtc_data[2] = rk816_reg_read(rk816, RK816_HOURS_REG);
	rtc_data[3] = rk816_reg_read(rk816, RK816_DAYS_REG);
	rtc_data[4] = rk816_reg_read(rk816, RK816_MONTHS_REG);
	rtc_data[5] = rk816_reg_read(rk816, RK816_YEARS_REG);
	rtc_data[6] = rk816_reg_read(rk816, RK816_WEEKS_REG);

	tm->tm_sec = bcd2bin(rtc_data[0]);
	tm->tm_min = bcd2bin(rtc_data[1]);
	tm->tm_hour = bcd2bin(rtc_data[2]);
	tm->tm_mday = bcd2bin(rtc_data[3]);
	tm->tm_mon = bcd2bin(rtc_data[4]) - 1;
	tm->tm_year = bcd2bin(rtc_data[5]) + 100;
	tm->tm_wday = bcd2bin(rtc_data[6]);

	dev_dbg(dev, "RTC date/time %4d-%02d-%02d(%d) %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday,
		tm->tm_wday, tm->tm_hour, tm->tm_min, tm->tm_sec);

	return 0;
}

/*
 * Set current time and date in RTC
 */
static int rk816_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	struct rk816_rtc *rk816_rtc = dev_get_drvdata(dev);
	struct rk816 *rk816 = rk816_rtc->rk816;
	int ret;
	u8 rtc_ctl;
	unsigned char rtc_data[ALL_TIME_REGS + 1];

	rtc_data[0] = bin2bcd(tm->tm_sec);
	rtc_data[1] = bin2bcd(tm->tm_min);
	rtc_data[2] = bin2bcd(tm->tm_hour);
	rtc_data[3] = bin2bcd(tm->tm_mday);
	rtc_data[4] = bin2bcd(tm->tm_mon + 1);
	rtc_data[5] = bin2bcd(tm->tm_year - 100);
	rtc_data[6] = bin2bcd(tm->tm_wday);

	dev_dbg(dev, "set RTC date/time %4d-%02d-%02d(%d) %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday,
		tm->tm_wday, tm->tm_hour, tm->tm_min, tm->tm_sec);

	/* Stop RTC while updating the TC registers */
	ret = rk816_reg_read(rk816, RK816_RTC_CTRL_REG);
	if (ret < 0) {
		dev_err(dev, "Failed to read RTC control: %d\n", ret);
		return ret;
	}

	rtc_ctl = ret | (BIT_RTC_CTRL_REG_STOP_RTC_M);
	ret = rk816_reg_write(rk816, RK816_RTC_CTRL_REG, rtc_ctl);
	if (ret < 0) {
		dev_err(dev, "Failed to write RTC control: %d\n", ret);
		return ret;
	}

	rk816_reg_write(rk816, RK816_SECONDS_REG, rtc_data[0]);
	rk816_reg_write(rk816, RK816_MINUTES_REG, rtc_data[1]);
	rk816_reg_write(rk816, RK816_HOURS_REG, rtc_data[2]);
	rk816_reg_write(rk816, RK816_DAYS_REG, rtc_data[3]);
	rk816_reg_write(rk816, RK816_MONTHS_REG, rtc_data[4]);
	rk816_reg_write(rk816, RK816_YEARS_REG, rtc_data[5]);
	rk816_reg_write(rk816, RK816_WEEKS_REG, rtc_data[6]);

	/* Start RTC again */
	ret = rk816_reg_read(rk816, RK816_RTC_CTRL_REG);
	if (ret < 0) {
		dev_err(dev, "Failed to read RTC control: %d\n", ret);
		return ret;
	}

	rtc_ctl = ret & (~BIT_RTC_CTRL_REG_STOP_RTC_M);
	ret = rk816_reg_write(rk816, RK816_RTC_CTRL_REG, rtc_ctl);
	if (ret < 0) {
		dev_err(dev, "Failed to write RTC control: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * Read alarm time and date in RTC
 */
static int rk816_rtc_readalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rk816_rtc *rk816_rtc = dev_get_drvdata(dev);
	int ret;
	unsigned char alrm_data[ALL_ALM_REGS + 1];

	alrm_data[0] = rk816_reg_read(rk816_rtc->rk816,
				      RK816_ALARM_SECONDS_REG);
	alrm_data[1] = rk816_reg_read(rk816_rtc->rk816,
				      RK816_ALARM_MINUTES_REG);
	alrm_data[2] = rk816_reg_read(rk816_rtc->rk816,
				      RK816_ALARM_HOURS_REG);
	alrm_data[3] = rk816_reg_read(rk816_rtc->rk816,
				      RK816_ALARM_DAYS_REG);
	alrm_data[4] = rk816_reg_read(rk816_rtc->rk816,
				      RK816_ALARM_MONTHS_REG);
	alrm_data[5] = rk816_reg_read(rk816_rtc->rk816,
				      RK816_ALARM_YEARS_REG);

	/* some of these fields may be wildcard/"match all" */
	alrm->time.tm_sec = bcd2bin(alrm_data[0]);
	alrm->time.tm_min = bcd2bin(alrm_data[1]);
	alrm->time.tm_hour = bcd2bin(alrm_data[2]);
	alrm->time.tm_mday = bcd2bin(alrm_data[3]);
	alrm->time.tm_mon = bcd2bin(alrm_data[4]) - 1;
	alrm->time.tm_year = bcd2bin(alrm_data[5]) + 100;

	ret = rk816_reg_read(rk816_rtc->rk816, RK816_RTC_INT_REG);
	if (ret < 0) {
		dev_err(dev, "Failed to read RTC control: %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "alrm read RTC date/time %4d-%02d-%02d(%d) %02d:%02d:%02d\n",
		1900 + alrm->time.tm_year, alrm->time.tm_mon + 1,
		alrm->time.tm_mday, alrm->time.tm_wday, alrm->time.tm_hour,
		alrm->time.tm_min, alrm->time.tm_sec);

	if (ret & BIT_RTC_INTERRUPTS_REG_IT_ALARM_M)
		alrm->enabled = 1;
	else
		alrm->enabled = 0;

	return 0;
}

static int rk816_rtc_stop_alarm(struct rk816_rtc *rk816_rtc)
{
	rk816_rtc->alarm_enabled = 0;

	return rk816_clear_bits(rk816_rtc->rk816, RK816_RTC_INT_REG,
			       BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);
}

static int rk816_rtc_start_alarm(struct rk816_rtc *rk816_rtc)
{
	rk816_rtc->alarm_enabled = 1;

	return rk816_set_bits(rk816_rtc->rk816, RK816_RTC_INT_REG,
			      BIT_RTC_INTERRUPTS_REG_IT_ALARM_M,
			      BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);
}

static int rk816_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	int ret;
	struct rk816_rtc *rk816_rtc = dev_get_drvdata(dev);
	unsigned char alrm_data[ALL_TIME_REGS + 1];

	ret = rk816_rtc_stop_alarm(rk816_rtc);
	if (ret < 0) {
		dev_err(dev, "Failed to stop alarm: %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "alrm set RTC date/time %4d-%02d-%02d(%d) %02d:%02d:%02d\n",
		1900 + alrm->time.tm_year, alrm->time.tm_mon + 1,
		alrm->time.tm_mday, alrm->time.tm_wday, alrm->time.tm_hour,
		alrm->time.tm_min, alrm->time.tm_sec);

	alrm_data[0] = bin2bcd(alrm->time.tm_sec);
	alrm_data[1] = bin2bcd(alrm->time.tm_min);
	alrm_data[2] = bin2bcd(alrm->time.tm_hour);
	alrm_data[3] = bin2bcd(alrm->time.tm_mday);
	alrm_data[4] = bin2bcd(alrm->time.tm_mon + 1);
	alrm_data[5] = bin2bcd(alrm->time.tm_year - 100);

	rk816_reg_write(rk816_rtc->rk816, RK816_ALARM_SECONDS_REG,
			alrm_data[0]);
	rk816_reg_write(rk816_rtc->rk816, RK816_ALARM_MINUTES_REG,
			alrm_data[1]);
	rk816_reg_write(rk816_rtc->rk816, RK816_ALARM_HOURS_REG,
			alrm_data[2]);
	rk816_reg_write(rk816_rtc->rk816, RK816_ALARM_DAYS_REG,
			alrm_data[3]);
	rk816_reg_write(rk816_rtc->rk816, RK816_ALARM_MONTHS_REG,
			alrm_data[4]);
	rk816_reg_write(rk816_rtc->rk816, RK816_ALARM_YEARS_REG,
			alrm_data[5]);

	if (alrm->enabled) {
		ret = rk816_rtc_start_alarm(rk816_rtc);
		if (ret < 0) {
			dev_err(dev, "Failed to start alarm: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int rk816_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct rk816_rtc *rk816_rtc = dev_get_drvdata(dev);

	if (enabled)
		return rk816_rtc_start_alarm(rk816_rtc);
	else
		return rk816_rtc_stop_alarm(rk816_rtc);
}

/*
 * We will just handle setting the frequency and make use the framework for
 * reading the periodic interupts.
 *
 * @freq: Current periodic IRQ freq:
 * bit 0: every second
 * bit 1: every minute
 * bit 2: every hour
 * bit 3: every day
 */
static irqreturn_t rk816_alm_irq(int irq, void *data)
{
	struct rk816_rtc *rk816_rtc = data;
	int ret;
	u8 rtc_ctl;

	ret = rk816_reg_read(rk816_rtc->rk816, RK816_RTC_STATUS_REG);
	if (ret < 0) {
		pr_err("%s:Failed to read RTC status: %d\n", __func__, ret);
		return ret;
	}
	rtc_ctl = ret & 0xff;

	/* The alarm interrupt keeps its low level,
	 * until the micro-controller write 1 in the
	 * ALARM bit of the RTC_STATUS_REG register.
	 */
	ret = rk816_reg_write(rk816_rtc->rk816, RK816_RTC_STATUS_REG, rtc_ctl);
	if (ret < 0) {
		pr_err("%s:Failed to read RTC status: %d\n", __func__, ret);
		return ret;
	}

	ret = rk816_set_bits(rk816_rtc->rk816, RK816_INT_STS_REG2,
			     RK816_ALARM_INT_STATUS, RK816_ALARM_INT_STATUS);
	if (ret < 0) {
		pr_err("%s:Failed to read RTC status: %d\n", __func__, ret);
		return ret;
	}

	rtc_update_irq(rk816_rtc->rtc, 1, RTC_IRQF | RTC_AF);
	pr_info("%s:irq=%d,rtc_ctl=0x%x\n", __func__, irq, rtc_ctl);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops rk816_rtc_ops = {
	.read_time = rk816_rtc_readtime,
	.set_time = rk816_rtc_settime,
	.read_alarm = rk816_rtc_readalarm,
	.set_alarm = rk816_rtc_setalarm,
	.alarm_irq_enable = rk816_rtc_alarm_irq_enable,
};

#ifdef CONFIG_PM
/* Turn off the alarm if it should not be a wake source. */
static int rk816_rtc_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rk816_rtc *rk816_rtc = dev_get_drvdata(&pdev->dev);
	int ret = 0;

	if (rk816_rtc->alarm_enabled && device_may_wakeup(&pdev->dev))
		ret = rk816_rtc_start_alarm(rk816_rtc);
	else
		ret = rk816_rtc_stop_alarm(rk816_rtc);

	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to update RTC alarm: %d\n", ret);
		return ret;
	}

	return 0;
}

/* Enable the alarm if it should be enabled (in case it was disabled to
 * prevent use as a wake source).
 */
static int rk816_rtc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rk816_rtc *rk816_rtc = dev_get_drvdata(&pdev->dev);
	int ret = 0;

	if (rk816_rtc->alarm_enabled) {
		ret = rk816_rtc_start_alarm(rk816_rtc);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to restart RTC alarm: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

/* Unconditionally disable the alarm */
static int rk816_rtc_freeze(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rk816_rtc *rk816_rtc = dev_get_drvdata(&pdev->dev);
	int ret = 0;

	ret = rk816_rtc_stop_alarm(rk816_rtc);
	if (ret < 0)
		dev_err(&pdev->dev, "Failed to stop RTC alarm: %d\n", ret);

	return ret;
}
#else
#define rk816_rtc_suspend NULL
#define rk816_rtc_resume NULL
#define rk816_rtc_freeze NULL
#endif

struct platform_device *rk816_pdev;
struct rtc_time rk816_tm_def = {
		.tm_wday = 6,
		.tm_year = 112,
		.tm_mon = 0,
		.tm_mday = 1,
		.tm_hour = 12,
		.tm_min = 0,
		.tm_sec = 0,
};

static int rk816_rtc_probe(struct platform_device *pdev)
{
	struct rk816 *rk816 = dev_get_drvdata(pdev->dev.parent);
	struct rk816_rtc *rk816_rtc;
	struct rtc_time tm;
	int alm_irq;
	int ret = 0;
	u8 rtc_ctl;

	pr_info("%s,line=%d\n", __func__, __LINE__);

	rk816_rtc = devm_kzalloc(&pdev->dev, sizeof(*rk816_rtc), GFP_KERNEL);
	if (rk816_rtc == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, rk816_rtc);
	rk816_rtc->rk816 = rk816;

	/*start rtc default*/
	ret = rk816_reg_read(rk816, RK816_RTC_CTRL_REG);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to read RTC control: %d\n", ret);
		return ret;
	}

	rtc_ctl = ret & (~BIT_RTC_CTRL_REG_STOP_RTC_M);
	ret = rk816_reg_write(rk816, RK816_RTC_CTRL_REG, rtc_ctl);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to write RTC control: %d\n", ret);
		return ret;
	}

	ret = rk816_reg_read(rk816, RK816_RTC_STATUS_REG);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to read RTC status: %d\n", ret);
		return ret;
	}
	rk816_reg_write(rk816, RK816_RTC_STATUS_REG, 0xfe);
	/*set init time*/

	ret = rk816_rtc_readtime(&pdev->dev, &tm);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to read RTC time\n");
		return ret;
	}

	ret = rtc_valid_tm(&tm);
	if (ret) {
		dev_err(&pdev->dev, "invalid date/time and init time\n");
		rk816_rtc_settime(&pdev->dev, &rk816_tm_def);
	}

	device_init_wakeup(&pdev->dev, 1);

	rk816_rtc->rtc = devm_rtc_device_register(&pdev->dev, "rk816",
						  &rk816_rtc_ops, THIS_MODULE);
	if (IS_ERR(rk816_rtc->rtc)) {
		ret = PTR_ERR(rk816_rtc->rtc);
		return ret;
	}

	alm_irq = regmap_irq_get_virq(rk816->irq_data, RK816_IRQ_RTC_ALARM);
	if (alm_irq < 0) {
		if (alm_irq != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Wake up is not possible as irq = %d\n",
				alm_irq);
		return alm_irq;
	}

	ret = devm_request_threaded_irq(rk816->dev, alm_irq,
					NULL, rk816_alm_irq,
					IRQF_TRIGGER_RISING, "RTC alarm",
					rk816_rtc);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to request alarm IRQ %d: %d\n",
			alm_irq, ret);
	}

	rk816_pdev = pdev;

	pr_info("%s:ok\n", __func__);

	return ret;
}

static const struct dev_pm_ops rk816_rtc_pm_ops = {
	.suspend = rk816_rtc_suspend,
	.resume = rk816_rtc_resume,
	.freeze = rk816_rtc_freeze,
	.thaw = rk816_rtc_resume,
	.restore = rk816_rtc_resume,
	.poweroff = rk816_rtc_suspend,
};

static struct platform_driver rk816_rtc_driver = {
	.probe = rk816_rtc_probe,
	.driver = {
		.name = "rk816-rtc",
		.pm = &rk816_rtc_pm_ops,
	},
};

static int __init rk816_rtc_init(void)
{
	return platform_driver_register(&rk816_rtc_driver);
}
subsys_initcall_sync(rk816_rtc_init);

static void __exit rk816_rtc_exit(void)
{
	platform_driver_unregister(&rk816_rtc_driver);
}
module_exit(rk816_rtc_exit);

MODULE_DESCRIPTION("RTC driver for the rk816 series PMICs");
MODULE_AUTHOR("ZHANGQING <zhanqging@rock-chips.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rk816-rtc");
