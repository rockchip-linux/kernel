/*
**************************************************************************
 * Rockchip driver for CIF CIF 1.0
 * (Based on Intel driver for sofiaxxx)
 *
 * Copyright (C) 2015 Intel Mobile Communications GmbH
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
**************************************************************************
 */

#ifndef CONFIG_OF
#error "this driver requires a kernel with device tree support"
#endif
#include <linux/regmap.h>

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include "cif_cif10.h"
#include "cif_cif10_regs.h"
#include <linux/platform_data/rk_cif10_platform.h>
#include <media/videobuf-core.h>

#define CIF_F0_READY (0x01<<0)
#define CIF_F1_READY (0x01<<1)

#define CIF_CLS_F0_STAT     (0xFFFFFFFE)
#define CIF_CLS_F1_STAT     (0xFFFFFFFD)
#define CIF_CLS_F0F1_STAT   (0xFFFFFFFC)

#define CIF_RESET_WORK
#if defined(CIF_RESET_WORK)
static void cif_cif10_cifrest(struct work_struct *work)
{
	struct cif_cif10_device *cif_cif10_dev =
		  container_of(work, struct cif_cif10_device, work);
#else
static void cif_cif10_cifrest(struct cif_cif10_device *cif_cif10_dev)
{
#endif
	struct device *dev = cif_cif10_dev->dev;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct pltfrm_soc_cfg *soc_cfg = cif_cif10_dev->soc_cfg;
	struct pltfrm_soc_cfg_para cfg_para;
	struct pltfrm_soc_init_para init_para;

	cfg_para.cmd = PLTFRM_CLKRST;
	cfg_para.cfg_para = &init_para;
	init_para.pdev = pdev;
	init_para.cif_base = cif_cif10_dev->config.base_addr;
	(soc_cfg->soc_cfg)(&cfg_para);

	cif_iowrite32(
			0x200 | 0x001,
			cif_cif10_dev->config.base_addr + CIF_CIF_INTEN);
	cif_iowrite32OR(ENABLE_CAPTURE,
			cif_cif10_dev->config.base_addr + CIF_CIF_CTRL);
	mdelay(5);
	cif_cif10_img_src_s_streaming(cif_cif10_dev->img_src, true);
}

static inline irqreturn_t cif_cif10_cifirq(int irq, void *data)
{
	short reset = 0, cif_err = 0, frm_flag;
	int ret;
	unsigned long y_addr, uv_addr;
	struct timeval tv;
	unsigned long long now, interval;
	unsigned int cifctrl, lastpix, lastline;
	unsigned long tmp_cif_frmst, reg_intstat;
	struct cif_cif10_device *cif_cif10_dev = data;
	struct cif_cif10_frm_fmt *frm_fmt = NULL;
	struct videobuf_buffer *curr_buf = NULL, *next_buf = NULL;

	do_gettimeofday(&tv);
	now = tv.tv_sec*1000000 + tv.tv_usec;
	interval = now -
		cif_cif10_dev->irqinfo.cifirq_interval;
	cif_cif10_dev->irqinfo.cifirq_interval = now;

	frm_fmt = &cif_cif10_dev->config.output;

	reg_intstat = cif_ioread32(
			cif_cif10_dev->config.base_addr + CIF_CIF_INTSTAT);

	if ((reg_intstat & 0x200) && (reg_intstat & 0x001)) {
		cif_iowrite32(
				0x3FF,
				cif_cif10_dev->config.base_addr +
				CIF_CIF_INTSTAT);
		cifctrl  = cif_ioread32(
				cif_cif10_dev->config.base_addr +
				CIF_CIF_CTRL);
		tmp_cif_frmst = cif_ioread32(
				cif_cif10_dev->config.base_addr +
				CIF_CIF_FRAME_STATUS);
		lastpix  = cif_ioread32(
				cif_cif10_dev->config.base_addr +
				CIF_CIF_LAST_PIX);
		lastline = cif_ioread32(
				cif_cif10_dev->config.base_addr +
				CIF_CIF_LAST_LINE);

		if (
			(tmp_cif_frmst & CIF_F0_READY) &&
				(tmp_cif_frmst & CIF_F1_READY)) {
			cif_cif10_pltfrm_pr_err(
				cif_cif10_dev->dev,
				"cif frm0&frm1 now reset it (frm_stat reg_val = %#lx)\n",
				tmp_cif_frmst);
			cif_err = 1;
			goto cif_rst;
		}
		if (tmp_cif_frmst & CIF_F0_READY)
			frm_flag = 0;
		else if (tmp_cif_frmst & CIF_F1_READY)
			frm_flag = 1;

		if (PLTFRM_CAM_ITF_IS_CVBS_NTSC(
				cif_cif10_dev->config.cam_itf.type)) {
			if (interval > 17000) {
				if (cif_cif10_dev->irqinfo.plug < 3)
					cif_cif10_dev->irqinfo.plug++;
			} else {
				if (cif_cif10_dev->irqinfo.plug == 3) {
					cif_cif10_dev->irqinfo.plug = 0;
					reset = 1;
					goto cif_rst;
				} else {
					cif_cif10_dev->irqinfo.plug = 0;
				}
			}
		} else if (PLTFRM_CAM_ITF_IS_CVBS_PAL(
				cif_cif10_dev->config.cam_itf.type)) {
			if (interval > 20500) {
				if (cif_cif10_dev->irqinfo.plug < 3)
					cif_cif10_dev->irqinfo.plug++;
			} else {
				if (cif_cif10_dev->irqinfo.plug == 3) {
					cif_cif10_dev->irqinfo.plug = 0;
					reset = 1;
					goto cif_rst;
				} else {
					cif_cif10_dev->irqinfo.plug = 0;
				}
			}
		}

		curr_buf = cif_cif10_dev->stream.curr_buf;

		if (
			!list_empty(&cif_cif10_dev->stream.buf_queue) &&
						!cif_cif10_dev->stream.stop) {
			next_buf = list_entry(
					cif_cif10_dev->stream.buf_queue.next,
					struct videobuf_buffer,
					queue);
			WARN_ON(next_buf->state != VIDEOBUF_QUEUED);
			if (frm_flag == 0) {
				y_addr = next_buf->baddr;
				uv_addr = y_addr +
					next_buf->width * next_buf->height;
				cif_iowrite32(
					y_addr,
					cif_cif10_dev->config.base_addr +
						CIF_CIF_FRM0_ADDR_Y);
				cif_iowrite32(
					uv_addr,
					cif_cif10_dev->config.base_addr +
						CIF_CIF_FRM0_ADDR_UV);
				 cif_cif10_dev->irqinfo.cif_frm0_ok = 1;
			} else if (frm_flag == 1 &&
					cif_cif10_dev->irqinfo.cif_frm0_ok){
				cif_cif10_dev->irqinfo.cif_frm1_ok = 1;

				y_addr = next_buf->baddr + next_buf->width;
				uv_addr = y_addr +
					next_buf->width * next_buf->height;

				cif_iowrite32(
					y_addr,
					cif_cif10_dev->config.base_addr +
						CIF_CIF_FRM1_ADDR_Y);
				cif_iowrite32(
					uv_addr,
					cif_cif10_dev->config.base_addr +
						CIF_CIF_FRM1_ADDR_UV);

				cif_cif10_dev->stream.curr_buf = next_buf;
				list_del_init(&next_buf->queue);
			}

			if (
				frm_flag == 1 &&
					cif_cif10_dev->irqinfo.cif_frm0_ok &&
					cif_cif10_dev->irqinfo.cif_frm0_ok) {
				do_gettimeofday(&curr_buf->ts);
				cif_cif10_dev->irqinfo.cif_frm0_ok = 0;
				cif_cif10_dev->irqinfo.cif_frm1_ok = 0;
				if (
					(curr_buf->state == VIDEOBUF_QUEUED) ||
					(curr_buf->state == VIDEOBUF_ACTIVE)) {
					curr_buf->state =
						VIDEOBUF_DONE;
					curr_buf->field_count++;
				}
				wake_up(&curr_buf->done);
			}
		} else {
			cif_cif10_dev->irqinfo.cif_frm0_ok = 0;
			cif_cif10_dev->irqinfo.cif_frm1_ok = 0;
			pr_info("video_buf queue is empty!\n");
			goto end;
		}
	} else {
		cif_cif10_pltfrm_pr_err(
			cif_cif10_dev->dev,
			"cif devices error now reset it (intsat reg_val = %#lx)\n",
			reg_intstat);
		cif_iowrite32(
				0x3FF,
				cif_cif10_dev->config.base_addr +
				CIF_CIF_INTSTAT);
		cif_err = 1;
		goto cif_rst;
	}

cif_rst:
	if (
		(cifctrl & ENABLE_CAPTURE) &&
		((cifctrl >> 1) & 0x3) == MODE_ONEFRAME) {
			cif_iowrite32(
				(cifctrl & ~ENABLE_CAPTURE),
				cif_cif10_dev->config.base_addr +
				CIF_CIF_CTRL);
	}

	if (reset || cif_err) {
		cif_iowrite32AND(
				~ENABLE_CAPTURE,
				cif_cif10_dev->config.base_addr +
				CIF_CIF_CTRL);
		cif_iowrite32(
				0x0,
				cif_cif10_dev->config.base_addr +
				CIF_CIF_INTEN);

		/*reset cvbsin*/
		ret = cif_cif10_img_src_ioctl(
				cif_cif10_dev->img_src,
				PLTFRM_CIFCAM_RESET,
				NULL);
		if (IS_ERR_VALUE(ret)) {
			cif_cif10_pltfrm_pr_err(
				dev,
				"cif_cif10_img_src_ioctl PLTFRM_CIFCAM_RESET failed!\n");
			return ret;
		}
#if defined(CIF_RESET_WORK)
		INIT_WORK(&(cif_cif10_dev->work), cif_cif10_cifrest);
		queue_work(cif_cif10_dev->wq, &cif_cif10_dev->work);
#else
		cif_cif10_cifrest(cif_cif10_dev);
#endif
	}

end:
	if (
			cif_cif10_dev->stream.stop &&
			(cif_cif10_dev->stream.state ==
				CIF_CIF10_STATE_STREAMING)) {
		cif_cif10_dev->stream.state = CIF_CIF10_STATE_READY;
		cif_iowrite32AND(
				~ENABLE_CAPTURE,
				cif_cif10_dev->config.base_addr + CIF_CIF_CTRL);
		cif_iowrite32(
				0x0,
				cif_cif10_dev->config.base_addr +
				CIF_CIF_INTEN);
		cif_iowrite32(
				0xffffffff,
				cif_cif10_dev->config.base_addr +
				CIF_CIF_INTSTAT);
		cif_iowrite32(
				0x00000000,
				cif_cif10_dev->config.base_addr +
				CIF_CIF_FRAME_STATUS);
		cif_cif10_pltfrm_event_signal(
				cif_cif10_dev->dev,
				&cif_cif10_dev->stream.done);
	}

	return IRQ_HANDLED;
}

static irqreturn_t cif_cif10_pltfrm_irq_handler(int irq, void *cntxt)
{
	struct device *dev = cntxt;
	struct cif_cif10_device *cif_cif10_dev = NULL;

	if (!dev)
		return IRQ_NONE;
	cif_cif10_dev = dev_get_drvdata(dev);

	cif_cif10_cifirq(irq, cif_cif10_dev);

	return IRQ_HANDLED;
}

const char *cif_cif10_pltfrm_pm_state_string(
	enum cif_cif10_pm_state pm_state)
{
	switch (pm_state) {
	case CIF_CIF10_PM_STATE_OFF:
		return "CIF_CIF10_PM_STATE_OFF";
	case CIF_CIF10_PM_STATE_SUSPENDED:
		return "CIF_CIF10_PM_STATE_SUSPENDED";
	case CIF_CIF10_PM_STATE_SW_STNDBY:
		return "CIF_CIF10_PM_STATE_SW_STNDBY";
	case CIF_CIF10_PM_STATE_STREAMING:
		return "CIF_CIF10_PM_STATE_STREAMING";
	default:
		return "PM_STATE_UNKNOWN";
	}
}

inline void cif_cif10_pltfrm_write_reg(
	struct device *dev,
	u32 data,
	CIF_CIF10_PLTFRM_MEM_IO_ADDR addr)
{
	iowrite32(data, addr);
}

inline void cif_cif10_pltfrm_write_reg_OR(
	struct device *dev,
	u32 data,
	CIF_CIF10_PLTFRM_MEM_IO_ADDR addr)
{
	cif_cif10_pltfrm_write_reg(
			dev,
			(ioread32(addr) | data),
			addr);
}

inline void cif_cif10_pltfrm_write_reg_AND(
	struct device *dev,
	u32 data,
	CIF_CIF10_PLTFRM_MEM_IO_ADDR addr)
{
	cif_cif10_pltfrm_write_reg(
			dev,
			(ioread32(addr) & data),
			addr);
}

inline u32 cif_cif10_pltfrm_read_reg(
	struct device *dev,
	CIF_CIF10_PLTFRM_MEM_IO_ADDR addr)
{
	return ioread32(addr);
}

int cif_cif10_pltfrm_dev_init(
	struct cif_cif10_device *cif_cif10_dev,
	struct device **_dev,
	void __iomem **reg_base_addr)
{
	int ret;
	struct cif_cif10_pltfrm_data *pdata;
	struct device *dev = *_dev;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct resource *res;
	void __iomem *base_addr;
	unsigned int  irq;

	dev_set_drvdata(dev, cif_cif10_dev);
	cif_cif10_dev->dev = dev;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (NULL == pdata) {
		cif_cif10_pltfrm_pr_err(
			dev,
			"could not allocate memory for platform data\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "cif-id", &pdev->id);
	if (ret < 0) {
			dev_err(&pdev->dev, "Property 'cif-id' missing or invalid\n");
			ret = -EINVAL;
			goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		cif_cif10_pltfrm_pr_err(
			NULL,
			"platform_get_resource_byname failed\n");
		ret = -ENODEV;
		goto err;
	}
	base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(base_addr)) {
		cif_cif10_pltfrm_pr_err(NULL, "devm_ioremap_resource failed\n");
		if (IS_ERR(base_addr))
			ret = PTR_ERR(base_addr);
		else
			ret = -ENODEV;
	}
	*reg_base_addr = base_addr;
	pdata->base_addr = base_addr;

	irq = platform_get_irq_byname(pdev, "irq");
	if (IS_ERR_VALUE(irq)) {
		ret = irq;
		goto err;
	}

	ret = devm_request_threaded_irq(
			dev,
			irq,
			cif_cif10_pltfrm_irq_handler,
			NULL,
			0,
			dev_driver_string(dev),
			dev);
	if (IS_ERR_VALUE(ret))
		goto err;
	pdata->irq = irq;

	pdata->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR_OR_NULL(pdata->pinctrl)) {
		pdata->pins_default = pinctrl_lookup_state(pdata->pinctrl,
			PINCTRL_STATE_DEFAULT);
		if (IS_ERR(pdata->pins_default))
			cif_cif10_pltfrm_pr_warn(
						dev,
						"could not get default pinstate\n");

		pdata->pins_sleep = pinctrl_lookup_state(pdata->pinctrl,
			PINCTRL_STATE_SLEEP);
		if (IS_ERR(pdata->pins_sleep))
			cif_cif10_pltfrm_pr_warn(
					dev,
					"could not get pins_sleep pinstate\n");

		pdata->pins_inactive =
				pinctrl_lookup_state(
					pdata->pinctrl,
					"inactive");
		if (IS_ERR(pdata->pins_inactive))
			cif_cif10_pltfrm_pr_warn(
						dev,
						"could not get pins_inactive pinstate\n");

		if (!IS_ERR_OR_NULL(pdata->pins_default))
			pinctrl_select_state(
					pdata->pinctrl,
					pdata->pins_default);
	}


	dev->platform_data = pdata;

	return 0;
err:
	cif_cif10_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	if (!IS_ERR_OR_NULL(pdata))
		devm_kfree(dev, pdata);
	return ret;
}

int cif_cif10_pltfrm_soc_init(
	struct cif_cif10_device *cif_cif10_dev,
	struct pltfrm_soc_cfg *soc_cfg)
{
	struct pltfrm_soc_cfg_para cfg_para;
	struct device *dev = cif_cif10_dev->dev;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct pltfrm_soc_init_para init_para;
	int ret = 0;

	if (!IS_ERR_OR_NULL(soc_cfg) && !IS_ERR_OR_NULL(soc_cfg->soc_cfg)) {
		cfg_para.cmd = PLTFRM_SOC_INIT;
		cfg_para.cfg_para = &init_para;
		init_para.pdev = pdev;
		init_para.cif_base = cif_cif10_dev->config.base_addr;
		ret = (soc_cfg->soc_cfg)(&cfg_para);
		if (ret == 0)
			cif_cif10_dev->soc_cfg = soc_cfg;
	}

	return ret;
}

int cif_cif10_pltfrm_mipi_dphy_config(
	struct cif_cif10_device *cif_cif10_dev)
{
	struct pltfrm_soc_cfg_para cfg_para;
	struct pltfrm_soc_cfg *soc_cfg;
	int ret = 0;

	soc_cfg = cif_cif10_dev->soc_cfg;
	if (
		!IS_ERR_OR_NULL(soc_cfg) &&
		!IS_ERR_OR_NULL(soc_cfg->soc_cfg)) {
		cfg_para.cmd =
			PLTFRM_MIPI_DPHY_CFG;
		cfg_para.cfg_para =
			(void *)(&cif_cif10_dev->config.cam_itf.cfg.mipi);
		ret = (soc_cfg->soc_cfg)(&cfg_para);
	}

	return ret;
}

int cif_cif10_pltfrm_pm_set_state(
	struct device *dev,
	enum cif_cif10_pm_state pm_state)
{
	int ret;
	struct cif_cif10_device *cif_cif10_dev = dev_get_drvdata(dev);
	struct platform_device *pdev =
				container_of(dev, struct platform_device, dev);
	struct pltfrm_soc_cfg *soc_cfg = cif_cif10_dev->soc_cfg;
	struct pltfrm_soc_cfg_para cfg_para;
	struct pltfrm_soc_init_para init_para;

	switch (pm_state) {
	case CIF_CIF10_PM_STATE_OFF:
	case CIF_CIF10_PM_STATE_SUSPENDED:
		cfg_para.cmd = PLTFRM_CLKDIS;

		init_para.pdev = pdev;
		init_para.cif_base = cif_cif10_dev->config.base_addr;
		cfg_para.cfg_para = &init_para;
		ret = (soc_cfg->soc_cfg)(&cfg_para);
		break;
	case CIF_CIF10_PM_STATE_SW_STNDBY:
	case CIF_CIF10_PM_STATE_STREAMING:
		cfg_para.cmd = PLTFRM_CLKEN;

		init_para.pdev = pdev;
		init_para.cif_base = cif_cif10_dev->config.base_addr;
		cfg_para.cfg_para = &init_para;
		ret = (soc_cfg->soc_cfg)(&cfg_para);
		break;
	default:
		cif_cif10_pltfrm_pr_err(
				dev,
				"unknown or unsupported PM state %d\n",
				pm_state);
		return -EINVAL;
	}

	if (IS_ERR_VALUE(ret))
		cif_cif10_pltfrm_pr_err(
			dev,
			"setting pm state to %s failed with error %d\n",
			cif_cif10_pltfrm_pm_state_string(pm_state), ret);
	else
		cif_cif10_pltfrm_pr_dbg(
			dev,
			"successfully changed pm state to %s\n",
			cif_cif10_pltfrm_pm_state_string(pm_state));
	return ret;
}

int cif_cif10_pltfrm_g_interface_config(
	struct cif_cif10_img_src *img_src,
	struct pltfrm_cam_itf *cam_itf)
{
	int ret = 0;

	ret = cif_cif10_img_src_ioctl(
			img_src,
			PLTFRM_CIFCAM_G_ITF_CFG,
			(void *)cam_itf);
	if (IS_ERR_VALUE(ret)) {
		cif_cif10_pltfrm_pr_err(
			dev,
			"cif_cif10_img_src_ioctl PLTFRM_CIFCAM_G_ITF_CFG failed!\n");
		return ret;
	}
	return 0;
}

int cif_cif10_pltfrm_pinctrl_set_state(
	struct device *dev,
	enum cif_cif10_pinctrl_state pinctrl_state)
{
	int ret = 0;
	struct cif_cif10_pltfrm_data *pdata = dev_get_platdata(dev);

	cif_cif10_pltfrm_pr_dbg(
			dev,
			"set pinctrl state to %d\n",
			pinctrl_state);

	if (NULL == pdata) {
		cif_cif10_pltfrm_pr_err(
				dev,
				"unable to retrieve CIF platform data\n");
		ret = -EINVAL;
		goto err;
	}
	if (IS_ERR_OR_NULL(pdata->pinctrl))
		return 0;


	switch (pinctrl_state) {
	case CIF_CIF10_PINCTRL_STATE_SLEEP:
		if (!IS_ERR_OR_NULL(pdata->pins_sleep))
			ret = pinctrl_select_state(
					pdata->pinctrl,
					pdata->pins_sleep);
		break;
	case CIF_CIF10_PINCTRL_STATE_ACTIVE:
	case CIF_CIF10_PINCTRL_STATE_DEFAULT:
		if (!IS_ERR_OR_NULL(pdata->pins_default))
			ret = pinctrl_select_state(
					pdata->pinctrl,
					pdata->pins_default);
		break;
	case CIF_CIF10_PINCTRL_STATE_INACTIVE:
		if (!IS_ERR_OR_NULL(pdata->pins_inactive))
			ret = pinctrl_select_state(
					pdata->pinctrl,
					pdata->pins_inactive);
		break;
	default:
		cif_cif10_pltfrm_pr_err(
			dev,
			"unknown or unsupported pinctrl state %d\n",
			pinctrl_state);
		ret = -EINVAL;
		goto err;
	}

	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_cif10_pltfrm_pr_err(dev, "failed with error %d\n", ret);
	return ret;
}

const char *cif_cif10_pltfrm_get_device_type(
	struct device *dev)
{
	return dev->of_node->type;
}

const char *cif_cif10_pltfrm_dev_string(
	struct device *dev)
{
	return dev_driver_string(dev);
}

static int of_dev_node_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

/* must call put_device() when done with returned i2c_client device */
struct platform_device *of_find_pltfrm_device_by_node(struct device_node *node)
{
	struct device *dev;

	dev = bus_find_device(
			&platform_bus_type,
			NULL,
			node,
			of_dev_node_match);
	if (!dev)
		return NULL;

	return to_platform_device(dev);
}

int cif_cif10_pltfrm_get_img_src_device(
	struct device *dev,
	struct cif_cif10_img_src **img_src_array,
	unsigned int array_len)
{
	struct device_node *node = NULL;
	struct device_node *camera_list_node = NULL;
	struct i2c_client *client = NULL;
	struct platform_device *pltfrm_dev = NULL;
	int ret = 0;
	int index, size = 0;
	const __be32 *phandle;
	int num_cameras = 0;
	struct cif_cif10_device *cif_cif10_dev = dev_get_drvdata(dev);

	node = of_node_get(dev->of_node);
	if (IS_ERR_OR_NULL(node)) {
		dev_err(dev, "Unable to obtain CIF device node\n");
		ret = -EEXIST;
		goto err;
	}

	phandle = of_get_property(
			node,
			"rockchip,camera-modules-attached",
			&size);
	if (IS_ERR_OR_NULL(phandle)) {
		cif_cif10_pltfrm_pr_err(
			dev,
			"no camera-modules-attached'\n");
			ret = -EINVAL;
			goto err;
	}

	for (index = 0; index < size/sizeof(*phandle); index++) {
		camera_list_node = of_parse_phandle(
			node,
			"rockchip,camera-modules-attached",
			index);
		of_node_put(node);
		if (IS_ERR_OR_NULL(camera_list_node)) {
			cif_cif10_pltfrm_pr_err(
				dev,
				"invalid index %d for property\n",
				index);
				ret = -EINVAL;
				goto err;
		}

		if (!strcmp(
					camera_list_node->type,
					"v4l2-i2c-subdev")) {
			client = of_find_i2c_device_by_node(
				camera_list_node);
			of_node_put(camera_list_node);
			if (IS_ERR_OR_NULL(client)) {
				cif_cif10_pltfrm_pr_err(
					dev,
					"could not get camera i2c client\n");
				continue;
			}
		} else if (!strcmp(camera_list_node->type,
					"v4l2-pltfrm-subdev")) {
			pltfrm_dev =
				of_find_pltfrm_device_by_node(camera_list_node);
			if (IS_ERR_OR_NULL(pltfrm_dev)) {
				cif_cif10_pltfrm_pr_err(
					dev,
					"could not get cvbsin platform device\n");
				continue;
			}
		} else {
			cif_cif10_pltfrm_pr_dbg(
				dev,
				"device of type %s not supported\n",
				camera_list_node->type);
			of_node_put(camera_list_node);
			continue;
		}

		if (client != NULL)
			img_src_array[num_cameras] =
				cif_cif10_img_src_to_img_src(
					&client->dev,
					cif_cif10_dev->soc_cfg);
		else if (pltfrm_dev != NULL)
			img_src_array[num_cameras] =
				cif_cif10_img_src_to_img_src(
					&pltfrm_dev->dev,
					cif_cif10_dev->soc_cfg);

		if (!IS_ERR_OR_NULL(img_src_array[num_cameras])) {
			cif_cif10_pltfrm_pr_info(
				dev,
				"%s attach to cif cif10 img_src_array[%d]\n",
				cif_cif10_img_src_g_name(
					img_src_array[num_cameras]),
				num_cameras);
			num_cameras++;
			if (num_cameras >= array_len) {
				cif_cif10_pltfrm_pr_err(
					dev,
					"cif cif10 isn't support > %d\n",
					array_len);
				break;
			}
		} else {
		    continue;
		}
	}

	return num_cameras;
err:
	if (!IS_ERR_OR_NULL(client))
		put_device(&client->dev);
	if (!IS_ERR_OR_NULL(camera_list_node))
		of_node_put(camera_list_node);
	return ret;
}

void cif_cif10_pltfrm_dev_release(
	struct device *dev)
{
}

