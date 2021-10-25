// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Fuzhou Rockchip Electronics Co., Ltd
 */
#include <linux/bcd.h>
#include <linux/kernel.h>
#include <linux/mfd/rk630.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

/* RTC_CTRL_REG bitfields */
#define BIT_RTC_CTRL_REG_STOP_RTC_M		BIT(0)

/* RK630 has a shadowed register for saving a "frozen" RTC time.
 * When user setting "GET_TIME" to 1, the time will save in this shadowed
 * register. If set "READSEL" to 1, user read rtc time register, actually
 * get the time of that moment. If we need the real time, clr this bit.
 */
#define BIT_RTC_CTRL_REG_RTC_GET_TIME		BIT(6)
#define BIT_RTC_CTRL_REG_RTC_READSEL_M		BIT(7)
#define BIT_RTC_INTERRUPTS_REG_IT_ALARM_M	BIT(7)
#define RTC_STATUS_MASK		0xFF

#define SECONDS_REG_MSK				0x7F
#define MINUTES_REG_MAK				0x7F
#define HOURS_REG_MSK				0x3F
#define DAYS_REG_MSK				0x3F
#define MONTHS_REG_MSK				0x1F
#define YEARS_REG_MSK				0xFF
#define WEEKS_REG_MSK				0x7

/* REG_SECONDS_REG through REG_YEARS_REG is how many registers? */

#define NUM_TIME_REGS				8
#define NUM_ALARM_REGS				7

#define ENABLE_ALARM_INT			0xBF
#define ALARM_INT_STATUS			BIT(4)

#define CLK32K_TEST_EN				BIT(0)
#define CLK32K_TEST_START			BIT(0)
#define CLK32K_TEST_STATUS			BIT(1)
#define CLK32K_TEST_DONE			BIT(2)

/* Units are seconds */
#define CLK32K_TEST_LEN				10

#define CLK32K_COMP_DIR_ADD			BIT(7)
#define CLK32K_COMP_EN				BIT(2)
#define CLK32K_NO_COMP				0x1

struct rk630_rtc {
	struct rk630 *rk630;
	struct rtc_device *rtc;
	int irq;
	unsigned int flag;
};

/* Read current time and date in RTC */
static int rk630_rtc_readtime(struct device *dev, struct rtc_time *tm)
{
	struct rk630_rtc *rk630_rtc = dev_get_drvdata(dev);
	struct rk630 *rk630 = rk630_rtc->rk630;
	u8 rtc_data[NUM_TIME_REGS];
	int ret;

	/* Force an update of the shadowed registers right now */
	ret = regmap_update_bits(rk630->rtc, RTC_CTRL,
				 BIT_RTC_CTRL_REG_RTC_GET_TIME,
				 BIT_RTC_CTRL_REG_RTC_GET_TIME);
	if (ret) {
		dev_err(dev, "Failed to update bits rtc_ctrl: %d\n", ret);
		return ret;
	}

	/*
	 * After we set the GET_TIME bit, the rtc time can't be read
	 * immediately. So we should wait up to 31.25 us, about one cycle of
	 * 32khz. If we clear the GET_TIME bit here, the time of i2c transfer
	 * certainly more than 31.25us: 16 * 2.5us at 400kHz bus frequency.
	 */
	ret = regmap_update_bits(rk630->rtc, RTC_CTRL,
				 BIT_RTC_CTRL_REG_RTC_GET_TIME,
				 0);
	if (ret) {
		dev_err(dev, "Failed to update bits rtc_ctrl: %d\n", ret);
		return ret;
	}

	ret = regmap_bulk_read(rk630->rtc, RTC_SET_SECONDS,
			       rtc_data, NUM_TIME_REGS);
	if (ret) {
		dev_err(dev, "Failed to bulk read rtc_data: %d\n", ret);
		return ret;
	}

	tm->tm_sec = bcd2bin(rtc_data[0] & SECONDS_REG_MSK);
	tm->tm_min = bcd2bin(rtc_data[1] & MINUTES_REG_MAK);
	tm->tm_hour = bcd2bin(rtc_data[2] & HOURS_REG_MSK);
	tm->tm_mday = bcd2bin(rtc_data[3] & DAYS_REG_MSK);
	tm->tm_mon = (bcd2bin(rtc_data[4] & MONTHS_REG_MSK)) - 1;
	tm->tm_year = (bcd2bin(rtc_data[5] & YEARS_REG_MSK)) + 100;
	tm->tm_wday = bcd2bin(rtc_data[7] & WEEKS_REG_MSK);

	dev_dbg(dev, "RTC date/time %4d-%02d-%02d(%d) %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday,
		tm->tm_wday, tm->tm_hour, tm->tm_min, tm->tm_sec);

	return ret;
}

/* Set current time and date in RTC */
static int rk630_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct rk630_rtc *rk630_rtc = dev_get_drvdata(dev);
	struct rk630 *rk630 = rk630_rtc->rk630;
	u8 rtc_data[NUM_TIME_REGS];
	int ret;

	dev_dbg(dev, "set RTC date/time %4d-%02d-%02d(%d) %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday,
		tm->tm_wday, tm->tm_hour, tm->tm_min, tm->tm_sec);

	rtc_data[0] = bin2bcd(tm->tm_sec);
	rtc_data[1] = bin2bcd(tm->tm_min);
	rtc_data[2] = bin2bcd(tm->tm_hour);
	rtc_data[3] = bin2bcd(tm->tm_mday);
	rtc_data[4] = bin2bcd(tm->tm_mon + 1);
	rtc_data[5] = bin2bcd(tm->tm_year - 100);
	rtc_data[6] = 0;
	rtc_data[7] = bin2bcd(tm->tm_wday);

	/* Stop RTC while updating the RTC registers */
	ret = regmap_update_bits(rk630->rtc, RTC_CTRL,
				 BIT_RTC_CTRL_REG_STOP_RTC_M,
				 BIT_RTC_CTRL_REG_STOP_RTC_M);
	if (ret) {
		dev_err(dev, "Failed to update RTC control: %d\n", ret);
		return ret;
	}

	ret = regmap_bulk_write(rk630->rtc, RTC_SET_SECONDS,
				rtc_data, NUM_TIME_REGS);
	if (ret) {
		dev_err(dev, "Failed to bull write rtc_data: %d\n", ret);
		return ret;
	}
	/* Start RTC again */
	ret = regmap_update_bits(rk630->rtc, RTC_CTRL,
				 BIT_RTC_CTRL_REG_STOP_RTC_M, 0);
	if (ret) {
		dev_err(dev, "Failed to update RTC control: %d\n", ret);
		return ret;
	}
	return 0;
}

/* Read alarm time and date in RTC */
static int rk630_rtc_readalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rk630_rtc *rk630_rtc = dev_get_drvdata(dev);
	struct rk630 *rk630 = rk630_rtc->rk630;
	u8 alrm_data[NUM_ALARM_REGS];
	u32 int_reg;
	int ret;

	ret = regmap_bulk_read(rk630->rtc,
			       RTC_ALARM_SECONDS,
			       alrm_data, NUM_ALARM_REGS);
	if (ret) {
		dev_err(dev, "Failed to read RTC alarm date REG: %d\n", ret);
		return ret;
	}

	alrm->time.tm_sec = bcd2bin(alrm_data[0] & SECONDS_REG_MSK);
	alrm->time.tm_min = bcd2bin(alrm_data[1] & MINUTES_REG_MAK);
	alrm->time.tm_hour = bcd2bin(alrm_data[2] & HOURS_REG_MSK);
	alrm->time.tm_mday = bcd2bin(alrm_data[3] & DAYS_REG_MSK);
	alrm->time.tm_mon = (bcd2bin(alrm_data[4] & MONTHS_REG_MSK)) - 1;
	alrm->time.tm_year = (bcd2bin(alrm_data[5] & YEARS_REG_MSK)) + 100;

	ret = regmap_read(rk630->rtc, RTC_INT0_EN, &int_reg);
	if (ret) {
		dev_err(dev, "Failed to read RTC INT REG: %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "alrm read RTC date/time %4d-%02d-%02d(%d) %02d:%02d:%02d\n",
		1900 + alrm->time.tm_year, alrm->time.tm_mon + 1,
		alrm->time.tm_mday, alrm->time.tm_wday, alrm->time.tm_hour,
		alrm->time.tm_min, alrm->time.tm_sec);

	alrm->enabled = (int_reg & BIT_RTC_INTERRUPTS_REG_IT_ALARM_M) ? 1 : 0;

	return 0;
}

static int rk630_rtc_stop_alarm(struct rk630_rtc *rk630_rtc)
{
	struct rk630 *rk630 = rk630_rtc->rk630;
	int ret;

	ret = regmap_write(rk630->rtc, RTC_INT0_EN, 0);

	return ret;
}

static int rk630_rtc_start_alarm(struct rk630_rtc *rk630_rtc)
{
	struct rk630 *rk630 = rk630_rtc->rk630;
	int ret;

	ret = regmap_write(rk630->rtc, RTC_INT0_EN, ENABLE_ALARM_INT);

	return ret;
}

static int rk630_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rk630_rtc *rk630_rtc = dev_get_drvdata(dev);
	struct rk630 *rk630 = rk630_rtc->rk630;
	u8 alrm_data[NUM_ALARM_REGS];
	int ret;

	ret = rk630_rtc_stop_alarm(rk630_rtc);
	if (ret) {
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

	ret = regmap_bulk_write(rk630->rtc,
				RTC_ALARM_SECONDS,
				alrm_data, NUM_ALARM_REGS);
	if (ret) {
		dev_err(dev, "Failed to bulk write: %d\n", ret);
		return ret;
	}
	if (alrm->enabled) {
		ret = rk630_rtc_start_alarm(rk630_rtc);
		if (ret) {
			dev_err(dev, "Failed to start alarm: %d\n", ret);
			return ret;
		}
	}
	return 0;
}

static int rk630_rtc_alarm_irq_enable(struct device *dev,
				      unsigned int enabled)
{
	struct rk630_rtc *rk630_rtc = dev_get_drvdata(dev);

	if (enabled)
		return rk630_rtc_start_alarm(rk630_rtc);

	return rk630_rtc_stop_alarm(rk630_rtc);
}

/*
 * We will just handle setting the frequency and make use the framework for
 * reading the periodic interrupts.
 *
 */
static irqreturn_t rk630_alarm_irq(int irq, void *data)
{
	struct rk630_rtc *rk630_rtc = data;
	struct rk630 *rk630 = rk630_rtc->rk630;
	int ret, status;

	ret = regmap_read(rk630->rtc, RTC_STATUS0, &status);
	if (ret) {
		pr_err("Failed to read RTC INT REG: %d\n", ret);
		return ret;
	}

	ret = regmap_write(rk630->rtc, RTC_STATUS0, status);
	if (ret) {
		pr_err("%s:Failed to update RTC status: %d\n", __func__, ret);
		return ret;
	}
	if (status & ALARM_INT_STATUS)
		rtc_update_irq(rk630_rtc->rtc, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops rk630_rtc_ops = {
	.read_time = rk630_rtc_readtime,
	.set_time = rk630_rtc_set_time,
	.read_alarm = rk630_rtc_readalarm,
	.set_alarm = rk630_rtc_setalarm,
	.alarm_irq_enable = rk630_rtc_alarm_irq_enable,
};

/*
 * Due to the analog generator 32k clock affected by
 * temperature, voltage, clock precision need test
 * with the environment change. In rtc test,
 * use 24M clock as reference clock to measure the 32k clock.
 * Before start test 32k clock, we should enable clk32k test(0x80),
 * and configure test length, when rtc test done(0x84[2]),
 * latch the 24M clock domain counter,
 * and read out the counter from rtc_test
 * registers(0x8c~0x98) via apb bus.
 * In RTC digital design, we set three level compensation,
 * the compensation value due to the
 * RTC 32k clock test result, and if we need compensation,
 * we need configure the compensation enable bit.
 * Comp every hour, compensation at last minute every hour,
 * and support add time and sub time by the MSB bit.
 * Comp every day, compensation at last minute in last hour every day,
 * and support add time and sub time by the MSB bit.
 * Comp every month, compensation at last minute
 * in last hour in last day every month,
 * and support add time and sub time by the MSB bit.
 */
static int rk630_rtc_compensation(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rk630_rtc *rk630_rtc = dev_get_drvdata(&pdev->dev);
	struct rk630 *rk630 = rk630_rtc->rk630;
	u32 counts, g_ref;
	u8 count[4];
	int ret, done = 0, tcamp, trim_dir, c_hour,
	    c_day, c_det_day, c_mon, c_det_mon;

	ret = regmap_update_bits(rk630->rtc, RTC_CLK32K_TEST,
				 CLK32K_TEST_EN, CLK32K_TEST_EN);
	if (ret) {
		dev_err(dev,
			"%s:Failed to update RTC CLK32K TEST: %d\n",
			__func__, ret);
		return ret;
	}

	ret = regmap_write(rk630->rtc, RTC_TEST_LEN,
			   CLK32K_TEST_LEN);
	if (ret) {
		dev_err(dev,
			"%s:Failed to update RTC CLK32K TEST LEN: %d\n",
			__func__, ret);
		return ret;
	}

	ret = regmap_update_bits(rk630->rtc, RTC_TEST_ST,
				 CLK32K_TEST_START, CLK32K_TEST_START);
	if (ret) {
		dev_err(dev,
			"%s:Failed to update RTC CLK32K TEST STATUS : %d\n",
			__func__, ret);
		return ret;
	}

	while (!done) {
		ret = regmap_read(rk630->rtc, RTC_TEST_ST, &done);
		if (ret) {
			dev_err(dev, "Failed to read RTC CLK32K TEST STATUS: %d\n",
				ret);
			return ret;
		}
		done = (done & CLK32K_TEST_DONE) >> 2;
		udelay(1);
	}

	ret = regmap_bulk_read(rk630->rtc,
			       RTC_CNT_0,
			       count, 4);
	if (ret) {
		dev_err(dev, "Failed to read RTC count REG: %d\n", ret);
		return ret;
	}

	counts = count[0] | (count[1] << 8) |
		 (count[2] << 16) | (count[3] << 24);
	g_ref = 24000000 * CLK32K_TEST_LEN;

	if (counts > g_ref) {
		trim_dir = 1;
		tcamp = 3600 * ((counts - g_ref) * 10) / g_ref;
	} else {
		trim_dir = 0;
		tcamp = 3600 * ((g_ref - counts) * 10) / g_ref;
	}
	c_hour = tcamp / 10;
	c_day = (24 * tcamp) / 10;
	c_mon = (30 * 24 * tcamp) / 10;
	if (trim_dir) {
		c_det_day = c_day - c_hour * 23;
		c_det_mon = c_mon - 29 * c_day - 23 * c_hour;
		if (c_hour > 1)
			regmap_write(rk630->rtc, RTC_COMP_H, c_hour - 1);
		else
			regmap_write(rk630->rtc, RTC_COMP_H, CLK32K_NO_COMP);
		if (c_det_day > 1)
			regmap_write(rk630->rtc, RTC_COMP_D, c_det_day - 1);
		else
			regmap_write(rk630->rtc, RTC_COMP_D, CLK32K_NO_COMP);
		if (c_det_mon)
			regmap_write(rk630->rtc, RTC_COMP_M, c_det_mon - 1);
		else
			regmap_write(rk630->rtc, RTC_COMP_M, CLK32K_NO_COMP);
	} else {
		c_det_day = c_day - c_hour * 23;
		c_det_mon = c_mon - 29 * c_day - 23 * c_hour;
		if (c_hour > 1)
			regmap_write(rk630->rtc, RTC_COMP_H,
				     (c_hour - 1) | CLK32K_COMP_DIR_ADD);
		else
			regmap_write(rk630->rtc, RTC_COMP_H, CLK32K_NO_COMP);
		if (c_det_day > 1)
			regmap_write(rk630->rtc, RTC_COMP_D,
				     (c_det_day - 1) | CLK32K_COMP_DIR_ADD);
		else
			regmap_write(rk630->rtc, RTC_COMP_D, CLK32K_NO_COMP);
		if (c_det_mon)
			regmap_write(rk630->rtc, RTC_COMP_M,
				     (c_det_mon - 1) | CLK32K_COMP_DIR_ADD);
		else
			regmap_write(rk630->rtc, RTC_COMP_M, CLK32K_NO_COMP);
	}

	ret = regmap_update_bits(rk630->rtc, RTC_CTRL,
				 CLK32K_COMP_EN, CLK32K_COMP_EN);
	if (ret) {
		dev_err(dev,
			"%s:Failed to update RTC CTRL : %d\n", __func__, ret);
		return ret;
	}
	return 0;
}

/* Enable the alarm if it should be enabled (in case it was disabled to
 * prevent use as a wake source).
 */
#ifdef CONFIG_PM_SLEEP
/* Turn off the alarm if it should not be a wake source. */
static int rk630_rtc_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rk630_rtc *rk630_rtc = dev_get_drvdata(&pdev->dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(rk630_rtc->irq);

	return 0;
}

/* Enable the alarm if it should be enabled (in case it was disabled to
 * prevent use as a wake source).
 */
static int rk630_rtc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rk630_rtc *rk630_rtc = dev_get_drvdata(&pdev->dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(rk630_rtc->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(rk630_rtc_pm_ops,
	rk630_rtc_suspend, rk630_rtc_resume);

static int rk630_rtc_probe(struct platform_device *pdev)
{
	struct rk630 *rk630 = dev_get_drvdata(pdev->dev.parent);
	struct rk630_rtc *rk630_rtc;
	int ret;

	rk630_rtc = devm_kzalloc(&pdev->dev, sizeof(*rk630_rtc), GFP_KERNEL);
	if (!rk630_rtc)
		return -ENOMEM;

	platform_set_drvdata(pdev, rk630_rtc);
	rk630_rtc->rk630 = rk630;

	rk630_rtc_compensation(&pdev->dev);

	/* start rtc running by default, and use shadowed timer. */
	ret = regmap_update_bits(rk630->rtc, RTC_CTRL,
				 BIT_RTC_CTRL_REG_STOP_RTC_M |
				 BIT_RTC_CTRL_REG_RTC_READSEL_M,
				 BIT_RTC_CTRL_REG_RTC_READSEL_M);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to update RTC control: %d\n", ret);
		return ret;
	}

	ret = regmap_write(rk630->rtc, RTC_STATUS0,
			   RTC_STATUS_MASK);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to write RTC status0: %d\n", ret);
			return ret;
	}

	ret = regmap_write(rk630->rtc, RTC_STATUS1,
			   RTC_STATUS_MASK);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to write RTC status1: %d\n", ret);
			return ret;
	}

	device_init_wakeup(&pdev->dev, 1);

	rk630_rtc->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(rk630_rtc->rtc))
		return PTR_ERR(rk630_rtc->rtc);

	rk630_rtc->rtc->ops = &rk630_rtc_ops;

	rk630_rtc->irq = platform_get_irq(pdev, 0);
	if (rk630_rtc->irq < 0) {
		if (rk630_rtc->irq != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Wake up is not possible as irq = %d\n",
				rk630_rtc->irq);
		return rk630_rtc->irq;
	}

	/* request alarm irq of rk630 */
	ret = devm_request_threaded_irq(&pdev->dev, rk630_rtc->irq, NULL,
					rk630_alarm_irq, 0,
					"RTC alarm", rk630_rtc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request alarm IRQ %d: %d\n",
			rk630_rtc->irq, ret);
		return ret;
	}

	return rtc_register_device(rk630_rtc->rtc);
}

static struct platform_driver rk630_rtc_driver = {
	.probe = rk630_rtc_probe,
	.driver = {
		.name = "rk630-rtc",
		.pm = &rk630_rtc_pm_ops,
	},
};

module_platform_driver(rk630_rtc_driver);

MODULE_DESCRIPTION("RTC driver for the rk630");
MODULE_AUTHOR("Zhang Qing <zhangqing@rock-chips.com>");
MODULE_LICENSE("GPL");
