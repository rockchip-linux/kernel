/**
 * core.c - DesignWare USB3 DRD Controller Core file
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>

#include <linux/usb/otg.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/of.h>
#include <linux/usb/otg.h>

#include "platform_data.h"
#include "core.h"
#include "gadget.h"
#include "io.h"

#include "debug.h"

/* -------------------------------------------------------------------------- */

void dwc3_set_mode(struct dwc3 *dwc, u32 mode)
{
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~(DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG));
	reg |= DWC3_GCTL_PRTCAPDIR(mode);
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

/**
 * dwc3_core_soft_reset - Issues core soft reset and PHY reset
 * @dwc: pointer to our context structure
 */
static int dwc3_core_soft_reset(struct dwc3 *dwc)
{
	u32		reg;
	int		retries = 1000;
	int		ret;

	usb_phy_init(dwc->usb2_phy);
	usb_phy_init(dwc->usb3_phy);
	ret = phy_init(dwc->usb2_generic_phy);
	if (ret < 0)
		return ret;

	ret = phy_init(dwc->usb3_generic_phy);
	if (ret < 0) {
		phy_exit(dwc->usb2_generic_phy);
		return ret;
	}

	/*
	 * We're resetting only the device side because, if we're in host mode,
	 * XHCI driver will reset the host block. If dwc3 was configured for
	 * host-only mode, then we can return early.
	 */
	if (dwc->dr_mode == USB_DR_MODE_HOST)
		return 0;

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg |= DWC3_DCTL_CSFTRST;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	do {
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		if (!(reg & DWC3_DCTL_CSFTRST))
			return 0;

		udelay(1);
	} while (--retries);

	return -ETIMEDOUT;
}

/**
 * dwc3_free_one_event_buffer - Frees one event buffer
 * @dwc: Pointer to our controller context structure
 * @evt: Pointer to event buffer to be freed
 */
static void dwc3_free_one_event_buffer(struct dwc3 *dwc,
		struct dwc3_event_buffer *evt)
{
	dma_free_coherent(dwc->dev, evt->length, evt->buf, evt->dma);
}

/**
 * dwc3_alloc_one_event_buffer - Allocates one event buffer structure
 * @dwc: Pointer to our controller context structure
 * @length: size of the event buffer
 *
 * Returns a pointer to the allocated event buffer structure on success
 * otherwise ERR_PTR(errno).
 */
static struct dwc3_event_buffer *dwc3_alloc_one_event_buffer(struct dwc3 *dwc,
		unsigned length)
{
	struct dwc3_event_buffer	*evt;

	evt = devm_kzalloc(dwc->dev, sizeof(*evt), GFP_KERNEL);
	if (!evt)
		return ERR_PTR(-ENOMEM);

	evt->dwc	= dwc;
	evt->length	= length;
	evt->buf	= dma_alloc_coherent(dwc->dev, length,
			&evt->dma, GFP_KERNEL);
	if (!evt->buf)
		return ERR_PTR(-ENOMEM);

	return evt;
}

/**
 * dwc3_free_event_buffers - frees all allocated event buffers
 * @dwc: Pointer to our controller context structure
 */
static void dwc3_free_event_buffers(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int i;

	for (i = 0; i < dwc->num_event_buffers; i++) {
		evt = dwc->ev_buffs[i];
		if (evt)
			dwc3_free_one_event_buffer(dwc, evt);
	}
}

/**
 * dwc3_alloc_event_buffers - Allocates @num event buffers of size @length
 * @dwc: pointer to our controller context structure
 * @length: size of event buffer
 *
 * Returns 0 on success otherwise negative errno. In the error case, dwc
 * may contain some buffers allocated but not all which were requested.
 */
static int dwc3_alloc_event_buffers(struct dwc3 *dwc, unsigned length)
{
	int			num;
	int			i;

	num = DWC3_NUM_INT(dwc->hwparams.hwparams1);
	dwc->num_event_buffers = num;

	dwc->ev_buffs = devm_kzalloc(dwc->dev, sizeof(*dwc->ev_buffs) * num,
			GFP_KERNEL);
	if (!dwc->ev_buffs) {
		dev_err(dwc->dev, "can't allocate event buffers array\n");
		return -ENOMEM;
	}

	for (i = 0; i < num; i++) {
		struct dwc3_event_buffer	*evt;

		evt = dwc3_alloc_one_event_buffer(dwc, length);
		if (IS_ERR(evt)) {
			dev_err(dwc->dev, "can't allocate event buffer\n");
			return PTR_ERR(evt);
		}
		dwc->ev_buffs[i] = evt;
	}

	return 0;
}

/**
 * dwc3_event_buffers_setup - setup our allocated event buffers
 * @dwc: pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int dwc3_event_buffers_setup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int				n;

	for (n = 0; n < dwc->num_event_buffers; n++) {
		evt = dwc->ev_buffs[n];
		dev_dbg(dwc->dev, "Event buf %p dma %08llx length %d\n",
				evt->buf, (unsigned long long) evt->dma,
				evt->length);

		evt->lpos = 0;

		dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(n),
				lower_32_bits(evt->dma));
		dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(n),
				upper_32_bits(evt->dma));
		dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(n),
				evt->length & 0xffff);
		dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(n), 0);
	}

	return 0;
}

static void dwc3_event_buffers_cleanup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int				n;

	for (n = 0; n < dwc->num_event_buffers; n++) {
		evt = dwc->ev_buffs[n];

		evt->lpos = 0;

		dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(n), 0);
		dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(n), 0);
		dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(n), 0);
		dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(n), 0);
	}
}

static void dwc3_core_num_eps(struct dwc3 *dwc)
{
	struct dwc3_hwparams	*parms = &dwc->hwparams;

	dwc->num_in_eps = DWC3_NUM_IN_EPS(parms);
	dwc->num_out_eps = DWC3_NUM_EPS(parms) - dwc->num_in_eps;

	dev_vdbg(dwc->dev, "found %d IN and %d OUT endpoints\n",
			dwc->num_in_eps, dwc->num_out_eps);
}

static void dwc3_cache_hwparams(struct dwc3 *dwc)
{
	struct dwc3_hwparams	*parms = &dwc->hwparams;

	parms->hwparams0 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS0);
	parms->hwparams1 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS1);
	parms->hwparams2 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS2);
	parms->hwparams3 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS3);
	parms->hwparams4 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS4);
	parms->hwparams5 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS5);
	parms->hwparams6 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS6);
	parms->hwparams7 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS7);
	parms->hwparams8 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS8);
}

/**
 * dwc3_phy_setup - Configure USB PHY Interface of DWC3 Core
 * @dwc: Pointer to our controller context structure
 */
static void dwc3_phy_setup(struct dwc3 *dwc)
{
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));

	/*
	 * Above 1.94a, it is recommended to set DWC3_GUSB3PIPECTL_SUSPHY
	 * to '0' during coreConsultant configuration. So default value
	 * will be '0' when the core is reset. Application needs to set it
	 * to '1' after the core initialization is completed.
	 */
	if (dwc->revision > DWC3_REVISION_194A)
		reg |= DWC3_GUSB3PIPECTL_SUSPHY;

	if (dwc->u2ss_inp3_quirk)
		reg |= DWC3_GUSB3PIPECTL_U2SSINP3OK;

	if (dwc->dis_u3_susphy_quirk)
		reg &= ~DWC3_GUSB3PIPECTL_SUSPHY;

	if (dwc->del_phy_power_chg_quirk)
		reg |= DWC3_GUSB3PIPECTL_DEPOCHANGE;

	if (dwc->dis_del_phy_power_chg_quirk)
		reg &= ~DWC3_GUSB3PIPECTL_DEPOCHANGE;

	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));

	switch (dwc->hsphy_mode) {
	case USBPHY_INTERFACE_MODE_UTMI:
		reg &= ~(DWC3_GUSB2PHYCFG_PHYIF_MASK |
		       DWC3_GUSB2PHYCFG_USBTRDTIM_MASK);
		reg |= DWC3_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_8_BIT) |
		       DWC3_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_8_BIT);
		break;
	case USBPHY_INTERFACE_MODE_UTMIW:
		reg &= ~(DWC3_GUSB2PHYCFG_PHYIF_MASK |
		       DWC3_GUSB2PHYCFG_USBTRDTIM_MASK);
		reg |= DWC3_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_16_BIT) |
		       DWC3_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_16_BIT);
		break;
	default:
		break;
	}

	/*
	 * Above 1.94a, it is recommended to set DWC3_GUSB2PHYCFG_SUSPHY to
	 * '0' during coreConsultant configuration. So default value will
	 * be '0' when the core is reset. Application needs to set it to
	 * '1' after the core initialization is completed.
	 */
	if (dwc->revision > DWC3_REVISION_194A)
		reg |= DWC3_GUSB2PHYCFG_SUSPHY;

	if (dwc->dis_enblslpm_quirk)
		reg &= ~DWC3_GUSB2PHYCFG_ENBLSLPM;

	if (dwc->dis_u2_susphy_quirk)
		reg &= ~DWC3_GUSB2PHYCFG_SUSPHY;

	if (dwc->dis_u2_freeclk_exists_quirk)
		reg &= ~DWC3_GUSB2PHYCFG_U2_FREECLK_EXISTS;

	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
}

/**
 * dwc3_core_init - Low-level initialization of DWC3 Core
 * @dwc: Pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int dwc3_core_init(struct dwc3 *dwc)
{
	unsigned long		timeout;
	u32			reg;
	int			ret;

	reg = dwc3_readl(dwc->regs, DWC3_GSNPSID);
	/* This should read as U3 followed by revision number */
	if ((reg & DWC3_GSNPSID_MASK) != 0x55330000) {
		dev_err(dwc->dev, "this is not a DesignWare USB3 DRD Core\n");
		ret = -ENODEV;
		goto err0;
	}
	dwc->revision = reg;

	/* issue device SoftReset too */
	timeout = jiffies + msecs_to_jiffies(500);
	dwc3_writel(dwc->regs, DWC3_DCTL, DWC3_DCTL_CSFTRST);
	do {
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		if (!(reg & DWC3_DCTL_CSFTRST))
			break;

		if (time_after(jiffies, timeout)) {
			dev_err(dwc->dev, "Reset Timed Out\n");
			ret = -ETIMEDOUT;
			goto err0;
		}

		cpu_relax();
	} while (true);

	ret = dwc3_core_soft_reset(dwc);
	if (ret)
		goto err0;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~DWC3_GCTL_SCALEDOWN_MASK;
	reg &= ~DWC3_GCTL_DISSCRAMBLE;

	switch (DWC3_GHWPARAMS1_EN_PWROPT(dwc->hwparams.hwparams1)) {
	case DWC3_GHWPARAMS1_EN_PWROPT_CLK:
		reg &= ~DWC3_GCTL_DSBLCLKGTNG;
		break;
	default:
		dev_dbg(dwc->dev, "No power optimization available\n");
	}

	/* check if current dwc3 is on simulation board */
	if (dwc->hwparams.hwparams6 & DWC3_GHWPARAMS6_EN_FPGA) {
		dev_dbg(dwc->dev, "it is on FPGA board\n");
		dwc->is_fpga = true;
	}

	/*
	 * WORKAROUND: DWC3 revisions <1.90a have a bug
	 * where the device can fail to connect at SuperSpeed
	 * and falls back to high-speed mode which causes
	 * the device to enter a Connect/Disconnect loop
	 */
	if (dwc->revision < DWC3_REVISION_190A)
		reg |= DWC3_GCTL_U2RSTECN;

	dwc3_core_num_eps(dwc);

	dwc3_writel(dwc->regs, DWC3_GCTL, reg);

	reg = dwc3_readl(dwc->regs, DWC3_GUCTL1);

	/*
	 * Enable hardware control of sending remote wakeup in HS when
	 * the device is in the L1 state.
	 */
	if (dwc->revision >= DWC3_REVISION_290A)
		reg |= DWC3_GUCTL1_DEV_L1_EXIT_BY_HW;

	if (dwc->tx_ipgap_linecheck_dis_quirk)
		reg |= DWC3_GUCTL1_TX_IPGAP_LINECHECK_DIS;

	dwc3_writel(dwc->regs, DWC3_GUCTL1, reg);

	return 0;

err0:
	return ret;
}

static void dwc3_core_exit(struct dwc3 *dwc)
{
	usb_phy_shutdown(dwc->usb2_phy);
	usb_phy_shutdown(dwc->usb3_phy);
	phy_exit(dwc->usb2_generic_phy);
	phy_exit(dwc->usb3_generic_phy);
}

static int dwc3_core_get_phy(struct dwc3 *dwc)
{
	struct device		*dev = dwc->dev;
	struct device_node	*node = dev->of_node;
	int ret;

	if (node) {
		dwc->usb2_phy = devm_usb_get_phy_by_phandle(dev, "usb-phy", 0);
		dwc->usb3_phy = devm_usb_get_phy_by_phandle(dev, "usb-phy", 1);
	} else {
		dwc->usb2_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
		dwc->usb3_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB3);
	}

	if (IS_ERR(dwc->usb2_phy)) {
		ret = PTR_ERR(dwc->usb2_phy);
		if (ret == -ENXIO || ret == -ENODEV) {
			dwc->usb2_phy = NULL;
		} else if (ret == -EPROBE_DEFER) {
			return ret;
		} else {
			dev_err(dev, "no usb2 phy configured\n");
			return ret;
		}
	}

	if (IS_ERR(dwc->usb3_phy)) {
		ret = PTR_ERR(dwc->usb3_phy);
		if (ret == -ENXIO || ret == -ENODEV) {
			dwc->usb3_phy = NULL;
		} else if (ret == -EPROBE_DEFER) {
			return ret;
		} else {
			dev_err(dev, "no usb3 phy configured\n");
			return ret;
		}
	}

	dwc->usb2_generic_phy = devm_phy_get(dev, "usb2-phy");
	if (IS_ERR(dwc->usb2_generic_phy)) {
		ret = PTR_ERR(dwc->usb2_generic_phy);
		if (ret == -ENOSYS || ret == -ENODEV) {
			dwc->usb2_generic_phy = NULL;
		} else if (ret == -EPROBE_DEFER) {
			return ret;
		} else {
			dev_err(dev, "no usb2 phy configured\n");
			return ret;
		}
	}

	dwc->usb3_generic_phy = devm_phy_get(dev, "usb3-phy");
	if (IS_ERR(dwc->usb3_generic_phy)) {
		ret = PTR_ERR(dwc->usb3_generic_phy);
		if (ret == -ENOSYS || ret == -ENODEV) {
			dwc->usb3_generic_phy = NULL;
		} else if (ret == -EPROBE_DEFER) {
			return ret;
		} else {
			dev_err(dev, "no usb3 phy configured\n");
			return ret;
		}
	}

	return 0;
}

#define DWC3_ALIGN_MASK		(16 - 1)

static int dwc3_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct dwc3_platform_data *pdata = dev_get_platdata(dev);
	struct device_node	*node = dev->of_node;
	struct resource		*res;
	struct dwc3		*dwc;

	int			ret = -ENOMEM;

	void __iomem		*regs;
	void			*mem;

	mem = devm_kzalloc(dev, sizeof(*dwc) + DWC3_ALIGN_MASK, GFP_KERNEL);
	if (!mem) {
		dev_err(dev, "not enough memory\n");
		return -ENOMEM;
	}
	dwc = PTR_ALIGN(mem, DWC3_ALIGN_MASK + 1);
	dwc->mem = mem;
	dwc->dev = dev;

	/* Try to set 64-bit DMA first */
	if (!pdev->dev.dma_mask)
		/* Platform did not initialize dma_mask */
		ret = dma_coerce_mask_and_coherent(&pdev->dev,
						   DMA_BIT_MASK(64));
	else
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));

	/* If seting 64-bit DMA mask fails, fall back to 32-bit DMA mask */
	if (ret) {
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (ret)
			return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "missing IRQ\n");
		return -ENODEV;
	}
	dwc->xhci_resources[1].start = res->start;
	dwc->xhci_resources[1].end = res->end;
	dwc->xhci_resources[1].flags = res->flags;
	dwc->xhci_resources[1].name = res->name;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "missing memory resource\n");
		return -ENODEV;
	}

	if (node) {
		dwc->maximum_speed = of_usb_get_maximum_speed(node);

		dwc->needs_fifo_resize = of_property_read_bool(node, "tx-fifo-resize");
		dwc->usb3_lpm_capable = of_property_read_bool(node,
				"snps,usb3_lpm_capable");
		dwc->dr_mode = of_usb_get_dr_mode(node);
		dwc->hsphy_mode = of_usb_get_phy_mode(node);

		dwc->u2ss_inp3_quirk = of_property_read_bool(node,
				"snps,u2ss_inp3_quirk");
		dwc->dis_enblslpm_quirk = of_property_read_bool(node,
				"snps,dis_enblslpm_quirk");
		dwc->dis_u3_autosuspend_quirk = of_property_read_bool(node,
				"snps,dis-u3-autosuspend-quirk");
		dwc->dis_u3_susphy_quirk = of_property_read_bool(node,
				"snps,dis_u3_susphy_quirk");
		dwc->dis_u2_susphy_quirk = of_property_read_bool(node,
				"snps,dis_u2_susphy_quirk");
		dwc->dis_u2_freeclk_exists_quirk = of_property_read_bool(node,
				"snps,dis-u2-freeclk-exists-quirk");
		dwc->del_phy_power_chg_quirk = of_property_read_bool(node,
				"snps,del_phy_power_chg_quirk");
		dwc->dis_del_phy_power_chg_quirk = of_property_read_bool(node,
				"snps,dis-del-phy-power-chg-quirk");
		dwc->tx_ipgap_linecheck_dis_quirk = of_property_read_bool(node,
				"snps,tx-ipgap-linecheck-dis-quirk");
	} else if (pdata) {
		dwc->maximum_speed = pdata->maximum_speed;

		dwc->needs_fifo_resize = pdata->tx_fifo_resize;
		dwc->usb3_lpm_capable = pdata->usb3_lpm_capable;
		dwc->dr_mode = pdata->dr_mode;
		dwc->hsphy_mode = pdata->hsphy_mode;

		dwc->u2ss_inp3_quirk = pdata->u2ss_inp3_quirk;
		dwc->dis_enblslpm_quirk = pdata->dis_enblslpm_quirk;
		dwc->dis_u3_autosuspend_quirk = pdata->dis_u3_autosuspend_quirk;
		dwc->dis_u3_susphy_quirk = pdata->dis_u3_susphy_quirk;
		dwc->dis_u2_susphy_quirk = pdata->dis_u2_susphy_quirk;
		dwc->dis_u2_freeclk_exists_quirk =
			pdata->dis_u2_freeclk_exists_quirk;
		dwc->del_phy_power_chg_quirk = pdata->del_phy_power_chg_quirk;
		dwc->dis_del_phy_power_chg_quirk =
			pdata->dis_del_phy_power_chg_quirk;
	}

	/* default to superspeed if no maximum_speed passed */
	if (dwc->maximum_speed == USB_SPEED_UNKNOWN)
		dwc->maximum_speed = USB_SPEED_SUPER;

	ret = dwc3_core_get_phy(dwc);
	if (ret)
		return ret;

	dwc->xhci_resources[0].start = res->start;
	dwc->xhci_resources[0].end = dwc->xhci_resources[0].start +
					DWC3_XHCI_REGS_END;
	dwc->xhci_resources[0].flags = res->flags;
	dwc->xhci_resources[0].name = res->name;

	res->start += DWC3_GLOBALS_REGS_START;

	/*
	 * Request memory region but exclude xHCI regs,
	 * since it will be requested by the xhci-plat driver.
	 */
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	spin_lock_init(&dwc->lock);
	platform_set_drvdata(pdev, dwc);

	dwc->regs	= regs;
	dwc->regs_size	= resource_size(res);

	dwc3_phy_setup(dwc);

	if (!dev->dma_mask) {
		dev->dma_mask	= dev->parent->dma_mask;
		dev->dma_parms	= dev->parent->dma_parms;
		dma_set_coherent_mask(dev, dev->parent->coherent_dma_mask);
	}

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);
	pm_runtime_forbid(dev);

	dwc3_cache_hwparams(dwc);

	ret = dwc3_alloc_event_buffers(dwc, DWC3_EVENT_BUFFERS_SIZE);
	if (ret) {
		dev_err(dwc->dev, "failed to allocate event buffers\n");
		ret = -ENOMEM;
		goto err0;
	}

	ret = dwc3_core_init(dwc);
	if (ret) {
		dev_err(dev, "failed to initialize core\n");
		goto err0;
	}

	usb_phy_set_suspend(dwc->usb2_phy, 0);
	usb_phy_set_suspend(dwc->usb3_phy, 0);
	ret = phy_power_on(dwc->usb2_generic_phy);
	if (ret < 0)
		goto err1;

	ret = phy_power_on(dwc->usb3_generic_phy);
	if (ret < 0)
		goto err_usb2phy_power;

	ret = dwc3_event_buffers_setup(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to setup event buffers\n");
		goto err_usb3phy_power;
	}

	if (IS_ENABLED(CONFIG_USB_DWC3_HOST))
		dwc->dr_mode = USB_DR_MODE_HOST;
	else if (IS_ENABLED(CONFIG_USB_DWC3_GADGET))
		dwc->dr_mode = USB_DR_MODE_PERIPHERAL;

	if (dwc->dr_mode == USB_DR_MODE_UNKNOWN)
		dwc->dr_mode = USB_DR_MODE_OTG;

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);
		ret = dwc3_gadget_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize gadget\n");
			goto err2;
		}
		break;
	case USB_DR_MODE_HOST:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);
		ret = dwc3_host_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize host\n");
			goto err2;
		}
		break;
	case USB_DR_MODE_OTG:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_OTG);
		ret = dwc3_host_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize host\n");
			goto err2;
		}

		ret = dwc3_gadget_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize gadget\n");
			goto err2;
		}
		break;
	default:
		dev_err(dev, "Unsupported mode of operation %d\n", dwc->dr_mode);
		goto err2;
	}

	ret = dwc3_debugfs_init(dwc);
	if (ret) {
		dev_err(dev, "failed to initialize debugfs\n");
		goto err3;
	}

	pm_runtime_allow(dev);

	return 0;

err3:
	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		dwc3_gadget_exit(dwc);
		break;
	case USB_DR_MODE_HOST:
		dwc3_host_exit(dwc);
		break;
	case USB_DR_MODE_OTG:
		dwc3_host_exit(dwc);
		dwc3_gadget_exit(dwc);
		break;
	default:
		/* do nothing */
		break;
	}

err2:
	dwc3_event_buffers_cleanup(dwc);

err_usb3phy_power:
	phy_power_off(dwc->usb3_generic_phy);

err_usb2phy_power:
	phy_power_off(dwc->usb2_generic_phy);

err1:
	usb_phy_set_suspend(dwc->usb2_phy, 1);
	usb_phy_set_suspend(dwc->usb3_phy, 1);
	dwc3_core_exit(dwc);

err0:
	dwc3_free_event_buffers(dwc);

	return ret;
}

static int dwc3_remove(struct platform_device *pdev)
{
	struct dwc3	*dwc = platform_get_drvdata(pdev);

	dwc3_debugfs_exit(dwc);

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		dwc3_gadget_exit(dwc);
		break;
	case USB_DR_MODE_HOST:
		dwc3_host_exit(dwc);
		break;
	case USB_DR_MODE_OTG:
		dwc3_host_exit(dwc);
		dwc3_gadget_exit(dwc);
		break;
	default:
		/* do nothing */
		break;
	}

	dwc3_event_buffers_cleanup(dwc);
	dwc3_free_event_buffers(dwc);

	usb_phy_set_suspend(dwc->usb2_phy, 1);
	usb_phy_set_suspend(dwc->usb3_phy, 1);
	phy_power_off(dwc->usb2_generic_phy);
	phy_power_off(dwc->usb3_generic_phy);

	dwc3_core_exit(dwc);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dwc3_prepare(struct device *dev)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	unsigned long	flags;

	spin_lock_irqsave(&dwc->lock, flags);

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
	case USB_DR_MODE_OTG:
		dwc3_gadget_prepare(dwc);
		/* FALLTHROUGH */
	case USB_DR_MODE_HOST:
	default:
		dwc3_event_buffers_cleanup(dwc);
		break;
	}

	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}

static void dwc3_complete(struct device *dev)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	unsigned long	flags;

	spin_lock_irqsave(&dwc->lock, flags);

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
	case USB_DR_MODE_OTG:
		dwc3_gadget_complete(dwc);
		/* FALLTHROUGH */
	case USB_DR_MODE_HOST:
	default:
		dwc3_event_buffers_setup(dwc);
		break;
	}

	spin_unlock_irqrestore(&dwc->lock, flags);
}

static int dwc3_suspend(struct device *dev)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	unsigned long	flags;

	spin_lock_irqsave(&dwc->lock, flags);

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
	case USB_DR_MODE_OTG:
		dwc3_gadget_suspend(dwc);
		/* FALLTHROUGH */
	case USB_DR_MODE_HOST:
	default:
		/* do nothing */
		break;
	}

	dwc->gctl = dwc3_readl(dwc->regs, DWC3_GCTL);
	spin_unlock_irqrestore(&dwc->lock, flags);

	usb_phy_shutdown(dwc->usb3_phy);
	usb_phy_shutdown(dwc->usb2_phy);
	phy_exit(dwc->usb2_generic_phy);
	phy_exit(dwc->usb3_generic_phy);

	usb_phy_set_suspend(dwc->usb2_phy, 1);
	usb_phy_set_suspend(dwc->usb3_phy, 1);
	WARN_ON(phy_power_off(dwc->usb2_generic_phy) < 0);
	WARN_ON(phy_power_off(dwc->usb3_generic_phy) < 0);

	return 0;
}

static int dwc3_resume(struct device *dev)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	unsigned long	flags;
	int		ret;

	usb_phy_set_suspend(dwc->usb2_phy, 0);
	usb_phy_set_suspend(dwc->usb3_phy, 0);
	ret = phy_power_on(dwc->usb2_generic_phy);
	if (ret < 0)
		return ret;

	ret = phy_power_on(dwc->usb3_generic_phy);
	if (ret < 0)
		goto err_usb2phy_power;

	usb_phy_init(dwc->usb3_phy);
	usb_phy_init(dwc->usb2_phy);
	ret = phy_init(dwc->usb2_generic_phy);
	if (ret < 0)
		goto err_usb3phy_power;

	ret = phy_init(dwc->usb3_generic_phy);
	if (ret < 0)
		goto err_usb2phy_init;

	spin_lock_irqsave(&dwc->lock, flags);

	dwc3_writel(dwc->regs, DWC3_GCTL, dwc->gctl);

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
	case USB_DR_MODE_OTG:
		dwc3_gadget_resume(dwc);
		/* FALLTHROUGH */
	case USB_DR_MODE_HOST:
	default:
		/* do nothing */
		break;
	}

	spin_unlock_irqrestore(&dwc->lock, flags);

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;

err_usb2phy_init:
	phy_exit(dwc->usb2_generic_phy);

err_usb3phy_power:
	phy_power_off(dwc->usb3_generic_phy);

err_usb2phy_power:
	phy_power_off(dwc->usb2_generic_phy);

	return ret;
}

static const struct dev_pm_ops dwc3_dev_pm_ops = {
	.prepare	= dwc3_prepare,
	.complete	= dwc3_complete,

	SET_SYSTEM_SLEEP_PM_OPS(dwc3_suspend, dwc3_resume)
};

#define DWC3_PM_OPS	&(dwc3_dev_pm_ops)
#else
#define DWC3_PM_OPS	NULL
#endif

#ifdef CONFIG_OF
static const struct of_device_id of_dwc3_match[] = {
	{
		.compatible = "snps,dwc3"
	},
	{
		.compatible = "synopsys,dwc3"
	},
	{ },
};
MODULE_DEVICE_TABLE(of, of_dwc3_match);
#endif

static struct platform_driver dwc3_driver = {
	.probe		= dwc3_probe,
	.remove		= dwc3_remove,
	.driver		= {
		.name	= "dwc3",
		.of_match_table	= of_match_ptr(of_dwc3_match),
		.pm	= DWC3_PM_OPS,
	},
};

module_platform_driver(dwc3_driver);

MODULE_ALIAS("platform:dwc3");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("DesignWare USB3 DRD Controller Driver");
