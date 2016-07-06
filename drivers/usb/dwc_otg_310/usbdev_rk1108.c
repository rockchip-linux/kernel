#ifdef CONFIG_ARM
#include "usbdev_rk.h"
#include "dwc_otg_regs.h"

static struct dwc_otg_control_usb *control_usb;

#ifdef CONFIG_USB20_OTG
static void usb20otg_hw_init(void)
{
	/* Turn off differential receiver in suspend mode */
	regmap_write(control_usb->usb_grf, 0x018, UOC_HIWORD_UPDATE(0, 1, 2));

	/* Set disconnect detection trigger point to 625mv */
	regmap_write(control_usb->usb_grf, 0x01c,
		     UOC_HIWORD_UPDATE(0x9, 0xf, 11));
}

static void usb20otg_phy_suspend(void *pdata, int suspend)
{
	struct dwc_otg_platform_data *usbpdata = pdata;

	if (suspend) {
		regmap_write(control_usb->usb_grf, 0x100,
			     UOC_HIWORD_UPDATE(0x1d1, 0x1ff, 0));
		usbpdata->phy_status = 1;
	} else {
		regmap_write(control_usb->usb_grf, 0x100,
			     UOC_HIWORD_UPDATE(0, 1, 0));
		usbpdata->phy_status = 0;
	}
}

static void usb20otg_soft_reset(void *pdata, enum rkusb_rst_flag rst_type)
{
	struct dwc_otg_platform_data *usbpdata = pdata;
	struct reset_control *rst_otg_h, *rst_otg_p, *rst_otg_c;

	rst_otg_h = devm_reset_control_get(usbpdata->dev, "otg_ahb");
	rst_otg_p = devm_reset_control_get(usbpdata->dev, "otg_phy");
	rst_otg_c = devm_reset_control_get(usbpdata->dev, "otg_controller");
	if (IS_ERR(rst_otg_h) || IS_ERR(rst_otg_p) || IS_ERR(rst_otg_c)) {
		dev_info(usbpdata->dev, "no reset control specified\n");
		return;
	}

	switch (rst_type) {
	case RST_POR:
		/* PHY reset */
		regmap_write(control_usb->usb_grf, 0x100,
			     UOC_HIWORD_UPDATE(0x1, 0x3, 0));
		reset_control_assert(rst_otg_p);
		usleep_range(10, 15);

		regmap_write(control_usb->usb_grf, 0x100,
			     UOC_HIWORD_UPDATE(0x2, 0x3, 0));
		usleep_range(1000, 1500);

		reset_control_deassert(rst_otg_p);
		udelay(2);

		/* Controller reset */
		reset_control_assert(rst_otg_c);
		reset_control_assert(rst_otg_h);
		udelay(2);

		reset_control_deassert(rst_otg_c);
		reset_control_deassert(rst_otg_h);
		break;
	case RST_CHN_HALT:
		/* PHY reset */
		regmap_write(control_usb->usb_grf, 0x100,
			     UOC_HIWORD_UPDATE(0x1, 0x3, 0));
		reset_control_assert(rst_otg_p);
		usleep_range(10, 15);

		regmap_write(control_usb->usb_grf, 0x100,
			     UOC_HIWORD_UPDATE(0x2, 0x3, 0));
		usleep_range(1000, 1500);

		reset_control_deassert(rst_otg_p);
		udelay(2);
		break;
	default:
		break;
	}
}

static void usb20otg_clock_init(void *pdata)
{
	struct dwc_otg_platform_data *usbpdata = pdata;
	struct clk *ahbclk, *ahbclk_otg_pmu;

	ahbclk = devm_clk_get(usbpdata->dev, "hclk_otg");
	if (IS_ERR(ahbclk)) {
		dev_err(usbpdata->dev, "Failed to get hclk_otg\n");
		return;
	}

	ahbclk_otg_pmu = devm_clk_get(usbpdata->dev, "hclk_otg_pmu");
	if (IS_ERR(ahbclk_otg_pmu)) {
		dev_err(usbpdata->dev, "Failed to get hclk_otg_pmu\n");
		return;
	}

	usbpdata->ahbclk = ahbclk;
	usbpdata->ahbclk_otg_pmu = ahbclk_otg_pmu;
}

static void usb20otg_clock_enable(void *pdata, int enable)
{
	struct dwc_otg_platform_data *usbpdata = pdata;

	if (enable) {
		clk_prepare_enable(usbpdata->ahbclk);
		clk_prepare_enable(usbpdata->ahbclk_otg_pmu);
		clk_prepare_enable(usbpdata->phyclk);
	} else {
		clk_disable_unprepare(usbpdata->ahbclk);
		clk_prepare_enable(usbpdata->ahbclk_otg_pmu);
		clk_disable_unprepare(usbpdata->phyclk);
	}
}

static int usb20otg_get_status(int id)
{
	int ret;
	u32 soc_st0;

	ret = regmap_read(control_usb->grf, 0x480, &soc_st0);
	if (ret)
		return ret;

	switch (id) {
	case USB_STATUS_BVABLID:
		/* bvalid in grf */
		ret = soc_st0 & BIT(10);
		break;
	case USB_STATUS_DPDM:
		/* dpdm in grf */
		ret = soc_st0 & (0x3 << 11);
		break;
	case USB_STATUS_ID:
		/* id in grf */
		ret = soc_st0 & BIT(13);
		break;
	case USB_CHIP_ID:
		ret = control_usb->chip_id;
		break;
	case USB_REMOTE_WAKEUP:
		ret = control_usb->remote_wakeup;
		break;
	case USB_IRQ_WAKEUP:
		ret = control_usb->usb_irq_wakeup;
		break;
	default:
		break;
	}

	return ret;
}

static void dwc_otg_uart_mode(void *pdata, int enter_usb_uart_mode)
{
}

struct dwc_otg_platform_data usb20otg_pdata_rk1108 = {
	.phyclk = NULL,
	.ahbclk = NULL,
	.busclk = NULL,
	.ahbclk_otg_pmu = NULL,
	.phy_status = 0,
	.hw_init = usb20otg_hw_init,
	.phy_suspend = usb20otg_phy_suspend,
	.soft_reset = usb20otg_soft_reset,
	.clock_init = usb20otg_clock_init,
	.clock_enable = usb20otg_clock_enable,
	.get_status = usb20otg_get_status,
	.dwc_otg_uart_mode = dwc_otg_uart_mode,
};
#endif

#define WAKE_LOCK_TIMEOUT (HZ * 10)
static inline void do_wakeup(struct work_struct *work)
{
	/* wake up the system */
	rk_send_wakeup_key();
}

static irqreturn_t bvalid_irq_handler(int irq, void *dev_id)
{
	/* clear irq */
	regmap_write(control_usb->grf, 0x6a0,
		     UOC_HIWORD_UPDATE(0x1, 0x1, 3));

	if (control_usb->usb_irq_wakeup) {
		wake_lock_timeout(&control_usb->usb_wakelock,
				  WAKE_LOCK_TIMEOUT);
		schedule_delayed_work(&control_usb->usb_det_wakeup_work,
				      HZ / 10);
	}

	return IRQ_HANDLED;
}

static irqreturn_t otg_irq_handler(int irq, void *data)
{
	u32 sig_det_st;
	int ret = IRQ_HANDLED;

	regmap_read(control_usb->grf, 0x690, &sig_det_st);

	/* For rk1108, otg_bvalid_irq, otg_id_irq and otg_linestate_irq are
	 * shared with the same irq number, so we use _grf_sig_detect_status_
	 * bits to distinguish them.
	 */
	if (sig_det_st & BIT(3))
		ret = bvalid_irq_handler(irq, data);

	return ret;
}

static int otg_irq_detect_init(struct platform_device *pdev)
{
	int ret, irq;

	wake_lock_init(&control_usb->usb_wakelock, WAKE_LOCK_SUSPEND,
		       "usb_detect");
	INIT_DELAYED_WORK(&control_usb->usb_det_wakeup_work, do_wakeup);

	if (!control_usb->usb_irq_wakeup && !control_usb->linestate_wakeup)
		return 0;

	irq = platform_get_irq_byname(pdev, "otg-irq");
	if (irq <= 0) {
		dev_err(&pdev->dev, "no otg-irq property provided.\n");
		return irq;
	}

	ret = request_threaded_irq(irq, NULL, otg_irq_handler,
				   IRQF_ONESHOT, "otg_irq", NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "request otg_irq failed.\n");
		return ret;
	}

	if (control_usb->usb_irq_wakeup) {
		/* clear and enable otg_bvalid irq */
		regmap_write(control_usb->grf, 0x6a0,
			     UOC_HIWORD_UPDATE(0x1, 0x1, 3));
		regmap_write(control_usb->grf, 0x680,
			     UOC_HIWORD_UPDATE(0x1, 0x1, 3));
	}

	if (control_usb->linestate_wakeup) {
		/* clear and enable otg_linestate irq */
		regmap_write(control_usb->grf, 0x6a0,
			     UOC_HIWORD_UPDATE(0x1, 0x1, 2));
		regmap_write(control_usb->grf, 0x680,
			     UOC_HIWORD_UPDATE(0x1, 0x1, 2));
	}

	return 0;
}

static int dwc_otg_control_usb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;

	control_usb = devm_kzalloc(dev, sizeof(*control_usb), GFP_KERNEL);
	if (!control_usb)
		return -ENOMEM;

	/* General Register Files regmap */
	control_usb->grf = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(control_usb->grf)) {
		dev_err(dev, "Missing rockchip,grf property\n");
		return PTR_ERR(control_usb->grf);
	}

	/* USB General Register Files regmap */
	control_usb->usb_grf =
		syscon_regmap_lookup_by_phandle(np, "rockchip,usbgrf");
	if (IS_ERR(control_usb->usb_grf)) {
		dev_err(dev, "Missing rockchip,usbgrf property\n");
		return PTR_ERR(control_usb->usb_grf);
	}

	control_usb->remote_wakeup =
		of_property_read_bool(np, "rockchip,remote_wakeup");
	control_usb->usb_irq_wakeup =
		of_property_read_bool(np, "rockchip,usb_irq_wakeup");
	control_usb->linestate_wakeup =
		of_property_read_bool(np, "rockchip,linestate_wakeup");

	control_usb->hclk_usb_peri = devm_clk_get(&pdev->dev, "hclk_usb_peri");
	if (IS_ERR(control_usb->hclk_usb_peri)) {
		dev_info(&pdev->dev, "no hclk_usb_peri clk specified\n");
		control_usb->hclk_usb_peri = NULL;
	}
	clk_prepare_enable(control_usb->hclk_usb_peri);

	ret = otg_irq_detect_init(pdev);
	if (ret < 0)
		goto err;

	return 0;

err:
	clk_disable_unprepare(control_usb->hclk_usb_peri);
	return ret;
}

static int dwc_otg_control_usb_remove(struct platform_device *pdev)
{
	clk_disable_unprepare(control_usb->hclk_usb_peri);
	return 0;
}

static const struct of_device_id dwc_otg_control_dt_match[] = {
	{ .compatible = "rockchip,rk1108-dwc-control-usb", },
	{},
};
MODULE_DEVICE_TABLE(of, dwc_otg_control_dt_match);

static struct platform_driver dwc_otg_control_usb_driver = {
	.probe		= dwc_otg_control_usb_probe,
	.remove		= dwc_otg_control_usb_remove,
	.driver		= {
		.name	= "rk1108-dwc-control-usb",
		.owner = THIS_MODULE,
		.of_match_table = dwc_otg_control_dt_match,
	},
};

static int __init dwc_otg_control_usb_init(void)
{
	return platform_driver_register(&dwc_otg_control_usb_driver);
}

subsys_initcall(dwc_otg_control_usb_init);

static void __exit dwc_otg_control_usb_exit(void)
{
	platform_driver_unregister(&dwc_otg_control_usb_driver);
}

module_exit(dwc_otg_control_usb_exit);

MODULE_AUTHOR("Frank Wang <frank.wang@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip USB OTG Control Driver");
MODULE_LICENSE("GPL v2");
#endif
