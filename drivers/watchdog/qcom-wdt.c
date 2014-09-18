/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/watchdog.h>

#define WDT_RST		0x0
#define WDT_EN		0x8
#define WDT_BITE_TIME	0x24

struct qcom_wdt {
	struct watchdog_device	wdd;
	unsigned long		freq;
	struct notifier_block	restart_nb;
	void __iomem		*base;
};

static inline
struct qcom_wdt *to_qcom_wdt(struct watchdog_device *wdd)
{
	return container_of(wdd, struct qcom_wdt, wdd);
}

static int qcom_wdt_start(struct watchdog_device *wdd)
{
	struct qcom_wdt *wdt = to_qcom_wdt(wdd);

	writel(0, wdt->base + WDT_EN);
	writel(1, wdt->base + WDT_RST);
	writel(wdd->timeout * wdt->freq, wdt->base + WDT_BITE_TIME);
	writel(1, wdt->base + WDT_EN);
	return 0;
}

static int qcom_wdt_stop(struct watchdog_device *wdd)
{
	struct qcom_wdt *wdt = to_qcom_wdt(wdd);

	writel(0, wdt->base + WDT_EN);
	return 0;
}

static int qcom_wdt_ping(struct watchdog_device *wdd)
{
	struct qcom_wdt *wdt = to_qcom_wdt(wdd);

	writel(1, wdt->base + WDT_RST);
	return 0;
}

static int qcom_wdt_set_timeout(struct watchdog_device *wdd,
				unsigned int timeout)
{
	wdd->timeout = timeout;
	return qcom_wdt_start(wdd);
}

static const struct watchdog_ops qcom_wdt_ops = {
	.start		= qcom_wdt_start,
	.stop		= qcom_wdt_stop,
	.ping		= qcom_wdt_ping,
	.set_timeout	= qcom_wdt_set_timeout,
	.owner		= THIS_MODULE,
};

static const struct watchdog_info qcom_wdt_info = {
	.options	= WDIOF_KEEPALIVEPING
			| WDIOF_MAGICCLOSE
			| WDIOF_SETTIMEOUT,
	.identity	= KBUILD_MODNAME,
};

static int qcom_wdt_restart(struct notifier_block *nb, unsigned long action,
			    void *data)
{
	struct qcom_wdt *wdt = container_of(nb, struct qcom_wdt, restart_nb);

	/*
	 * Trigger watchdog bite:
	 *    Setup BITE_TIME to be very low, and enable WDT.
	 */
	mutex_lock(&wdt->wdd.lock);
	writel(0, wdt->base + WDT_EN);
	writel(1, wdt->base + WDT_RST);
	writel(0x31F3, wdt->base + WDT_BITE_TIME);
	writel(1, wdt->base + WDT_EN);
	mutex_unlock(&wdt->wdd.lock);
	return NOTIFY_DONE;
}

static int qcom_watchdog_probe(struct platform_device *pdev)
{
	struct qcom_wdt *wdt;
	struct resource *res;
	u32 tmp;
	int ret;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	platform_set_drvdata(pdev, wdt);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	wdt->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);

	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency", &tmp);
	if (ret) {
		dev_err(&pdev->dev, "unable to get clock-frequency\n");
		return ret;
	}

	wdt->freq = tmp;

	wdt->wdd.dev = &pdev->dev;
	wdt->wdd.info = &qcom_wdt_info;
	wdt->wdd.ops = &qcom_wdt_ops;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = 0x10000000U / wdt->freq;
	watchdog_init_timeout(&wdt->wdd, 0, &pdev->dev);

	ret = watchdog_register_device(&wdt->wdd);
	if (ret) {
		dev_err(&pdev->dev, "failed to register watchdog\n");
		return ret;
	}

	/*
	 * WDT restart notifier has priority 0 (use as a last resort)
	 */
	wdt->restart_nb.notifier_call = qcom_wdt_restart;
	ret = register_restart_handler(&wdt->restart_nb);
	if (ret) {
		dev_err(&pdev->dev, "failed to setup restart handler\n");
		watchdog_unregister_device(&wdt->wdd);
		return ret;
	}

	return 0;
}

static const struct of_device_id qcom_wdt_of_table[] = {
	{ .compatible = "qcom,kpss-wdt-msm8960", },
	{ .compatible = "qcom,kpss-wdt-apq8064", },
	{ .compatible = "qcom,kpss-wdt-ipq8064", },
	{ },
};
MODULE_DEVICE_TABLE(of, qcom_wdt_of_table);

static struct platform_driver qcom_watchdog_driver = {
	.probe	= qcom_watchdog_probe,
	.driver	= {
		.name		= KBUILD_MODNAME,
		.of_match_table	= qcom_wdt_of_table,
	},
};
module_platform_driver(qcom_watchdog_driver);

MODULE_DESCRIPTION("QCOM KPSS Watchdog Driver");
MODULE_LICENSE("GPL v2");
