/**
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 * author: ZhiChao Yu zhichao.yu@rock-chips.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/of_address.h>
#include "dsp_dbg.h"
#include "dsp_ioctl.h"
#include "dsp_dev.h"

/*
 * Trace buffer max size must be smaller than dma_pool buffer size
 * defined in dsp_service.c. Trace buffer size must be an integer
 * multiple of trace slot size.
 */
#define DSP_TRACE_BUFFER_SIZE       (1 * 1024)
#define DSP_TRACE_SLOT_SIZE         (128)
#define DSP_TRACE_SLOT_COUNT        (DSP_TRACE_BUFFER_SIZE / \
				     DSP_TRACE_SLOT_SIZE)

#define DSP_FIRMWARE_NAME    "rkdsp.bin"
#define DSP_CMP_OFFSET       0x400000

#define DSP_GRF_CON0         0x0000
#define DSP_GLOBAL_RSTN_REQ  BIT(13)

#define DSP_DEV_SESSION      0

#define MHZ                  (1000 * 1000)
#define DSP_RATE             (600 * MHZ)

static void dsp_set_div_clk(struct clk *clock, int divide)
{
	struct clk *parent = clk_get_parent(clock);
	unsigned long rate = clk_get_rate(parent);

	clk_set_rate(clock, (rate / divide) + 1);
}

static int dsp_dev_trace(struct dsp_dev *dev, u32 index)
{
	int ret = 0;
	u32 trace_end = index;
	u32 trace_start = dev->trace_index;

	if (trace_end < trace_start ||
	    trace_end - trace_start > DSP_TRACE_SLOT_COUNT) {
		dsp_err("trace slot overflow\n");
		ret = -EFAULT;
		goto out;
	}

	while (trace_start <= trace_end) {
		int slot_index = trace_start % DSP_TRACE_SLOT_COUNT;
		char *trace_slot = dev->trace_buffer +
				DSP_TRACE_SLOT_SIZE * slot_index;

		if (*trace_slot)
			pr_info("##dsp##: %s\n", trace_slot);

		*trace_slot = 0;
		trace_start++;
	}
	dev->trace_index = trace_start;

out:
	return ret;
}

static int dsp_dev_work_done(struct dsp_work *work, void *data)
{
	int ret = 0;
	struct dsp_dev *dev = data;

	dsp_debug_enter();

	/*
	 * If a work is not belone to a session,
	 * this work is created by DSP device for config purpose.
	 * so the work must be destroy here.
	 */
	if (work->session == DSP_DEV_SESSION)
		dsp_work_destroy(dev->dma_pool, work);
	else
		dev->client->work_done(dev->client, work);

	mutex_unlock(&dev->lock);

	dsp_debug_leave();
	return ret;
}

static int dsp_dev_work(struct dsp_dev *dev, struct dsp_work *work)
{
	int ret = 0;

	dsp_debug_enter();

	mutex_lock(&dev->lock);

	work->done = dsp_dev_work_done;
	dev->mbox->send_data(dev->mbox, MBOX_CHAN_0, DSP_CMD_WORK,
			     work->dma_addr);
	dsp_debug_leave();
	return ret;
}

static int dsp_dev_receive_data(struct dsp_mbox_client *client,
				u32 chan, u32 cmd, u32 data)
{
	int ret = 0;
	struct dsp_dev *dev = container_of(client, struct dsp_dev, mbox_client);

	dsp_debug_enter();

	dsp_debug(DEBUG_DEVICE,
		  "received mbox data, chan=%d, cmd=0x%08x, data=0x%08x\n",
		  chan, cmd, data);

	switch (chan) {
	case MBOX_CHAN_0: {
		if (cmd == DSP_CMD_READY) {
			dev->client->device_ready(dev->client);
		} else if (cmd == DSP_CMD_WORK) {
			struct dsp_work *work;

			work = (struct dsp_work *)phys_to_virt(data);
			if (!work) {
				dsp_err("invalid work from dsp\n");
				ret = -EFAULT;
				goto out;
			}
			work->done(work, dev);
		}
	} break;

	case MBOX_CHAN_3: {
		if (cmd == DSP_CMD_TRACE) {
			ret = dsp_dev_trace(dev, data);
			dev->mbox->send_data(dev->mbox, chan, cmd,
					     dev->trace_index);
		}
	} break;
	}
out:
	dsp_debug_leave();
	return ret;
}

static int dsp_dev_config(struct dsp_dev *dev)
{
	int ret = 0;
	struct dsp_work *work;
	struct dsp_config_params config_params;
	struct list_head *pos, *n;

	dsp_debug_enter();

	memset(&config_params, 0, sizeof(config_params));
	config_params.type = DSP_CONFIG_INIT;
	config_params.trace_buffer = dev->trace_dma;
	config_params.trace_slot_size = DSP_TRACE_SLOT_SIZE;
	if (dev->trace_buffer)
		config_params.trace_buffer_size = DSP_TRACE_BUFFER_SIZE;
	else
		config_params.trace_buffer_size = 0;

	dsp_debug(DEBUG_DEVICE, "dsp trace start 0x%08x\n", dev->trace_dma);

	/* Config DSP image information */
	list_for_each_safe(pos, n, &dev->loader->images) {
		struct dsp_image *image;
		int idx = config_params.image_count;

		image = container_of(pos, struct dsp_image, list_node);
		config_params.image_phys[idx] = virt_to_phys(image);
		config_params.image_count++;
	}

	ret = dsp_work_create(dev->dma_pool, DSP_DEV_SESSION, &work);
	if (ret) {
		dsp_err("cannot create config work\n");
		goto out;
	}

	dsp_work_set_type(work, DSP_CONFIG_WORK);
	dsp_work_set_params(work, (void *)&config_params);
	dsp_dev_work(dev, work);
out:
	dsp_debug_leave();
	return ret;
}

static int dsp_dev_suspend(struct dsp_dev *dev)
{
	int ret = 0;

	dsp_debug_enter();
	/* TODO wait DSP idle and config DSP enter sleep mode */
	dev->status = DSP_SLEEP;
	dsp_debug_leave();

	return ret;
}

static int dsp_dev_resume(struct dsp_dev *dev)
{
	int ret = 0;

	dsp_debug_enter();
	/* TODO config DSP enter work state */
	dev->status = DSP_ON;
	dsp_debug_leave();

	return ret;
}

static int dsp_dev_power_on(struct dsp_dev *dev)
{
	int ret = 0;

	if (!(dev->status == DSP_OFF))
		return ret;

	dsp_debug_enter();

	clk_prepare_enable(dev->clk_dsp);
	/* clk_set_rate(dev->clk_dsp, DSP_RATE); */

	dsp_set_div_clk(dev->clk_iop, 4);
	dsp_set_div_clk(dev->clk_epp, 2);
	dsp_set_div_clk(dev->clk_edp, 2);
	dsp_set_div_clk(dev->clk_edap, 2);

	writel_relaxed((DSP_GLOBAL_RSTN_REQ << 16) | DSP_GLOBAL_RSTN_REQ,
		       dev->dsp_grf + DSP_GRF_CON0);

	reset_control_deassert(dev->sys_rst);
	reset_control_deassert(dev->global_rst);
	reset_control_deassert(dev->oecm_rst);
	usleep_range(500, 1000);

	ret = dev->loader->load_image(dev->loader, DSP_IMAGE_MAIN);
	if (ret) {
		dsp_err("load dsp os image failed\n");
		goto out;
	}

	/* Revoke reset of core, DSP will run right after this code */
	reset_control_deassert(dev->core_rst);

	dev->status = DSP_ON;
out:
	if (ret) {
		reset_control_assert(dev->sys_rst);
		reset_control_assert(dev->global_rst);
		reset_control_assert(dev->oecm_rst);

		writel_relaxed(DSP_GLOBAL_RSTN_REQ << 16,
			       dev->dsp_grf + DSP_GRF_CON0);
	}
	dsp_debug_leave();
	return ret;
}

static int dsp_dev_power_off(struct dsp_dev *dev)
{
	int ret = 0;

	if (dev->status == DSP_OFF)
		return ret;

	dsp_debug_enter();
	dsp_dev_trace(dev, dev->trace_index + DSP_TRACE_SLOT_COUNT);

	reset_control_assert(dev->core_rst);
	reset_control_assert(dev->sys_rst);
	reset_control_assert(dev->global_rst);
	reset_control_assert(dev->oecm_rst);

	writel_relaxed(DSP_GLOBAL_RSTN_REQ << 16, dev->dsp_grf + DSP_GRF_CON0);

	if (dev->clk_dsp)
		clk_disable_unprepare(dev->clk_dsp);

	dev->status = DSP_OFF;

	dsp_debug_leave();
	return ret;
}

static int dsp_dev_clk_init(struct platform_device *pdev, struct dsp_dev *dev)
{
	int ret = 0;

	dsp_debug_enter();

	dev->clk_dsp = devm_clk_get(&pdev->dev, "clk_dsp");
	if (IS_ERR(dev->clk_dsp)) {
		dsp_err("failed to get clk_dsp\n");
		ret = -EFAULT;
		goto out;
	}

	dev->clk_iop = devm_clk_get(&pdev->dev, "clk_dsp_iop");
	if (IS_ERR(dev->clk_iop)) {
		dsp_err("failed to get clk_iop\n");
		ret = -EFAULT;
		goto out;
	}

	dev->clk_epp = devm_clk_get(&pdev->dev, "clk_dsp_epp");
	if (IS_ERR(dev->clk_epp)) {
		dsp_err("failed to get clk_epp\n");
		ret = -EFAULT;
		goto out;
	}

	dev->clk_edp = devm_clk_get(&pdev->dev, "clk_dsp_edp");
	if (IS_ERR(dev->clk_edp)) {
		dsp_err("failed to get clk_edp\n");
		ret = -EFAULT;
		goto out;
	}

	dev->clk_edap = devm_clk_get(&pdev->dev, "clk_dsp_edap");
	if (IS_ERR(dev->clk_edap)) {
		dsp_err("failed to get clk_edap\n");
		ret = -EFAULT;
		goto out;
	}
out:
	dsp_debug_leave();
	return ret;
}

static int dsp_dev_resource_init(struct platform_device *pdev,
				 struct dsp_dev *dev)
{
	int ret = 0;
	struct resource *res;

	dsp_debug_enter();

	dev->core_rst = devm_reset_control_get(&pdev->dev, "core_rst");
	dev->sys_rst = devm_reset_control_get(&pdev->dev, "sys_rst");
	dev->global_rst = devm_reset_control_get(&pdev->dev, "global_rst");
	dev->oecm_rst = devm_reset_control_get(&pdev->dev, "oecm_rst");
	if (IS_ERR(dev->core_rst) || IS_ERR(dev->sys_rst) ||
	    IS_ERR(dev->global_rst) || IS_ERR(dev->oecm_rst)) {
		dsp_err("cannot get reset control\n");
		ret = -EINVAL;
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->dsp_grf = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->dsp_grf)) {
		dsp_err("cannot get dsp grf base address\n");
		ret = PTR_ERR(dev->dsp_grf);
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	dev->dsp_axi = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->dsp_axi)) {
		dsp_err("cannot get dsp base address\n");
		ret = PTR_ERR(dev->dsp_axi);
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	dev->mbox_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->mbox_base)) {
		dsp_err("cannot get mbox base address\n");
		ret = PTR_ERR(dev->mbox_base);
		goto out;
	}
out:
	dsp_debug_leave();
	return ret;
}

int dsp_dev_register_client(struct dsp_dev *dev, struct dsp_dev_client *client)
{
	dev->client = client;
	return 0;
}

int dsp_dev_create(struct platform_device *pdev, struct dma_pool *dma_pool,
		   struct dsp_dev **dev_out)
{
	int ret = 0;
	dma_addr_t dma_addr = 0;
	struct dsp_dev *dev;

	dsp_debug_enter();

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dsp_err("cannot alloc mem for dsp device\n");
		ret = -ENOMEM;
		goto out;
	}

	dev->on = dsp_dev_power_on;
	dev->off = dsp_dev_power_off;
	dev->suspend = dsp_dev_suspend;
	dev->resume = dsp_dev_resume;
	dev->config = dsp_dev_config;
	dev->work = dsp_dev_work;
	dev->dma_pool = dma_pool;
	dev->mbox_client.receive_data = dsp_dev_receive_data;
	mutex_init(&dev->lock);

	ret = dsp_dev_resource_init(pdev, dev);
	if (ret) {
		dsp_err("cannot initialize dsp hardware resource\n");
		goto out;
	}

	ret = dsp_dev_clk_init(pdev, dev);
	if (ret) {
		dsp_err("cannot initialize clock of dsp\n");
		goto out;
	}

	ret = dsp_dma_create(dev->dsp_axi + DSP_CMP_OFFSET, &dev->dma);
	if (ret) {
		dsp_err("create dsp dma fail\n");
		goto out;
	}

	ret = dsp_mbox_create(pdev, dev->mbox_base, &dev->mbox);
	if (ret) {
		dsp_err("cannot create dsp mbox\n");
		goto out;
	}
	dsp_mbox_register_client(dev->mbox, &dev->mbox_client);

	ret = dsp_loader_create(dev->dma, &dev->loader);
	if (ret) {
		dsp_err("cannot create dsp image loader\n");
		goto out;
	}
	request_firmware_nowait(THIS_MODULE, 0, DSP_FIRMWARE_NAME, &pdev->dev,
				GFP_KERNEL, dev->loader,
				dsp_loader_request_firmware);

	dev->trace_buffer = dma_pool_alloc(dma_pool, GFP_KERNEL, &dma_addr);
	if (dev->trace_buffer) {
		memset(dev->trace_buffer, 0, DSP_TRACE_BUFFER_SIZE);
		dev->trace_dma = dma_addr;
	}

	(*dev_out) = dev;
out:
	if (ret) {
		dsp_loader_destroy(dev->loader);
		dsp_dma_destroy(dev->dma);

		(*dev_out) = NULL;
	}
	dsp_debug_leave();
	return ret;
}

int dsp_dev_destroy(struct dsp_dev *dev)
{
	int ret = 0;

	dsp_debug_enter();
	dsp_mbox_destroy(dev->mbox);
	dsp_dma_destroy(dev->dma);
	dsp_loader_destroy(dev->loader);
	dma_pool_free(dev->dma_pool, dev->trace_buffer, dev->trace_dma);
	dsp_debug_leave();

	return ret;
}
