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
#include <linux/rockchip/pmu.h>
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

#define DSP_CMP_OFFSET       0x400000

#define DSP_GRF_CON0                0x0000
#	define DSP_GLOBAL_RSTN_REQ  BIT(13)
#define DSP_GRF_STAT1               0x0024
#	define DSP_PSU_CORE_IDLE    BIT(5)
#	define DSP_PSU_DSP_IDLE     BIT(4)

#define DSP_DEV_SESSION      0

static int dsp_dev_power_off(struct dsp_dev *dev);

static void dsp_set_div_clk(struct clk *clock, int divide)
{
	struct clk *parent = clk_get_parent(clock);
	unsigned long rate = clk_get_rate(parent);

	clk_set_rate(clock, (rate / divide) + 1);
}

static void dsp_dev_clk_enable(struct dsp_dev *dev)
{

	if (dev->dsp_dvfs_node) {
		clk_enable_dvfs(dev->dsp_dvfs_node);
		dvfs_clk_prepare_enable(dev->dsp_dvfs_node);
	} else {
		clk_prepare_enable(dev->clk_dsp);
	}

	clk_prepare_enable(dev->clk_dsp_free);
	clk_prepare_enable(dev->clk_iop);
	clk_prepare_enable(dev->clk_epp);
	clk_prepare_enable(dev->clk_edp);
	clk_prepare_enable(dev->clk_edap);

	dsp_set_div_clk(dev->clk_iop, 4);
	dsp_set_div_clk(dev->clk_epp, 2);
	dsp_set_div_clk(dev->clk_edp, 2);
	dsp_set_div_clk(dev->clk_edap, 2);
}

static void dsp_dev_clk_disable(struct dsp_dev *dev)
{
	clk_disable_unprepare(dev->clk_dsp_free);
	clk_disable_unprepare(dev->clk_iop);
	clk_disable_unprepare(dev->clk_epp);
	clk_disable_unprepare(dev->clk_edp);
	clk_disable_unprepare(dev->clk_edap);

	if (dev->dsp_dvfs_node) {
		dvfs_clk_disable_unprepare(dev->dsp_dvfs_node);
		clk_disable_dvfs(dev->dsp_dvfs_node);
	} else {
		clk_disable_unprepare(dev->clk_dsp);
	}
}

static void dsp_grf_global_reset_assert(struct dsp_dev *dev)
{
	writel_relaxed(DSP_GLOBAL_RSTN_REQ << 16, dev->dsp_grf + DSP_GRF_CON0);
}

static void dsp_grf_global_reset_deassert(struct dsp_dev *dev)
{
	writel_relaxed((DSP_GLOBAL_RSTN_REQ << 16) | DSP_GLOBAL_RSTN_REQ,
		       dev->dsp_grf + DSP_GRF_CON0);
}

static int dsp_dev_trace(struct dsp_dev *dev, u32 index)
{
	int ret = 0;
	u32 trace_end = index;
	u32 trace_start = dev->trace_index;

	if (trace_end < trace_start ||
	    trace_end - trace_start > DSP_TRACE_SLOT_COUNT) {
		dsp_err("trace slot overflow, start=%d, end=%d\n",
			trace_start, trace_end);
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

static int dsp_dev_work_done(struct dsp_dev *dev, struct dsp_work *work)
{
	dsp_debug_enter();

	dsp_debug(DEBUG_DEVICE,
		  "Work cost cycles, id=0x%08x, cycles=%d, dsp_rate=%ld\n",
		  work->id, work->cycles,
		  dvfs_clk_get_rate(dev->dsp_dvfs_node));

	/*
	 * Algorithms can request its satisfying DSP
	 * rate respectively
	 */
	if (dev->dsp_dvfs_node && work->rate) {
		dvfs_clk_set_rate(dev->dsp_dvfs_node, work->rate);
		dsp_debug(DEBUG_DEVICE, "request DSP rate=%d\n", work->rate);
	}

	/* We should not cancel work in timeout callback */
	if (work->result != DSP_WORK_ETIMEOUT)
		cancel_delayed_work_sync(&dev->guard_work);

	dsp_work_set_status(work, DSP_WORK_DONE);
	dev->client->work_done(dev->client, work);
	mutex_unlock(&dev->lock);

	dsp_debug_leave();
	return 0;
}

static int dsp_dev_work(struct dsp_dev *dev, struct dsp_work *work)
{
	int ret = 0;

	dsp_debug_enter();

	if (!mutex_trylock(&dev->lock)) {
		ret = -EBUSY;
		dsp_debug(DEBUG_DEVICE, "DSP device is busy\n");
		goto out;
	}

	schedule_delayed_work(&dev->guard_work, HZ);

	if (dev->dsp_dvfs_node)
		work->rate = dvfs_clk_get_rate(dev->dsp_dvfs_node);
	else
		work->rate = 0;
	dsp_work_set_status(work, DSP_WORK_RUNNING);

	dev->running_work = work;
	dev->mbox->send_data(dev->mbox, MBOX_CHAN_0, DSP_CMD_WORK,
			     work->dma_addr);
out:
	dsp_debug_leave();
	return ret;
}

/*
 * dsp_dev_work_timeout - Work guard work timeout callback
 *
 * Inevitably, in some cases, DSP hangs up by an unexpected bug, and no
 * interrupt comes from DSP. For stability, a work guard is used to watch
 * over work processing. If a work is processed by DSP more than 1 second,
 * this callback will be called to recover DSP from hang up status.
 */
static void dsp_dev_work_timeout(struct work_struct *work)
{
	struct dsp_dev *dev =
		container_of(work, struct dsp_dev, guard_work.work);
	struct dsp_work *timeout_work = dev->running_work;

	dsp_debug_enter();

	dev->client->device_pause(dev->client);

	dsp_err("start to recover DSP from hang up status\n");
	timeout_work->result = DSP_WORK_ETIMEOUT;
	dsp_dev_work_done(dev, timeout_work);

	/*
	 * Reset DSP core. Before reset DSP, we should request NoC idle
	 * to prevent DSP access system bus which maybe cause system halt.
	 */
	rockchip_pmu_ops.set_idle_request(IDLE_REQ_DSP, true);
	reset_control_assert(dev->core_rst);
	udelay(1);
	rockchip_pmu_ops.set_idle_request(IDLE_REQ_DSP, false);
	reset_control_deassert(dev->core_rst);

	dsp_debug_leave();
}

static int dsp_dev_config_done(struct dsp_dev *dev, struct dsp_work *work)
{
	dsp_debug_enter();

	dsp_work_destroy(dev->dma_pool, work);
	dev->client->device_ready(dev->client);

	dsp_debug_leave();
	return 0;
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
	if (dev->trace_buffer) {
		memset(dev->trace_buffer, 0, DSP_TRACE_BUFFER_SIZE);
		dev->trace_index = 0;
		config_params.trace_buffer_size = DSP_TRACE_BUFFER_SIZE;
	} else {
		config_params.trace_buffer_size = 0;
	}

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
	dsp_work_set_status(work, DSP_WORK_RUNNING);
	dev->mbox->send_data(dev->mbox, MBOX_CHAN_1, DSP_CMD_CONFIG,
			     work->dma_addr);
out:
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
		if (cmd == DSP_CMD_WORK_DONE) {
			struct dsp_work *work;

			work = (struct dsp_work *)phys_to_virt(data);
			if (!work) {
				dsp_err("invalid work from dsp\n");
				ret = -EFAULT;
				goto out;
			}
			dsp_dev_work_done(dev, work);
		}
	} break;

	case MBOX_CHAN_1: {
		if (cmd == DSP_CMD_READY) {
			dsp_dev_config(dev);
		} else if (cmd == DSP_CMD_CONFIG_DONE) {
			struct dsp_work *work;

			work = (struct dsp_work *)phys_to_virt(data);
			if (!work) {
				dsp_err("invalid config work from dsp\n");
				ret = -EFAULT;
				goto out;
			}
			dsp_dev_config_done(dev, work);
		}
	} break;

	case MBOX_CHAN_3: {
		if (cmd == DSP_CMD_TRACE) {
			ret = dsp_dev_trace(dev, data);
			dev->mbox->send_data(dev->mbox, chan,
					     DSP_CMD_TRACE_DONE,
					     dev->trace_index);
		}
	} break;
	}
out:
	dsp_debug_leave();
	return ret;
}

static int dsp_dev_suspend(struct dsp_dev *dev)
{
	u32 status = 0;
	int ret = 0;

	dsp_debug_enter();

	if (dev->status != DSP_ON)
		goto out;

	/*
	 * Only if DSP is in standby mode, we can disable DSP external clock.
	 * After that DSP is in DSP_SLEEP status.
	 */
	status = readl_relaxed(dev->dsp_grf + DSP_GRF_STAT1);
	if (status & DSP_PSU_DSP_IDLE) {
		dsp_debug(DEBUG_DEVICE, "Disbale DSP clk when standby\n");
		dsp_dev_clk_disable(dev);
		dev->status = DSP_SLEEP;
	} else {
		dsp_err("DSP is working, cannot enter suspend status\n");
		ret = -EFAULT;
	}

out:
	dsp_debug_leave();
	return ret;
}

static int dsp_dev_resume(struct dsp_dev *dev)
{
	int ret = 0;

	dsp_debug_enter();

	if (dev->status != DSP_SLEEP)
		goto out;

	/* Restore DSP external clock when DSP is in standby mode */
	dsp_debug(DEBUG_DEVICE, "Enable DSP clk when standby\n");
	dsp_dev_clk_enable(dev);
	dev->status = DSP_ON;

out:
	dsp_debug_leave();
	return ret;
}

static int dsp_dev_power_on(struct dsp_dev *dev)
{
	int ret = 0;

	dsp_debug_enter();

	if (!(dev->status == DSP_OFF))
		goto out;

	dsp_dev_clk_enable(dev);
	udelay(1);

	dsp_grf_global_reset_deassert(dev);

	reset_control_deassert(dev->sys_rst);
	reset_control_deassert(dev->global_rst);
	reset_control_deassert(dev->oecm_rst);
	udelay(1);

	dsp_mbox_enable(dev->mbox);

	ret = dsp_loader_load_image(dev->device, dev->loader, "MAIN");
	if (ret) {
		dev->status = DSP_ON;
		dsp_err("load dsp os image failed\n");
		goto out;
	}

	/* Revoke reset of core, DSP will run right after this code */
	reset_control_deassert(dev->core_rst);

	dev->status = DSP_ON;
	pr_info("DSP power on\n");
out:
	if (ret)
		dsp_dev_power_off(dev);

	dsp_debug_leave();
	return ret;
}

static int dsp_dev_power_off(struct dsp_dev *dev)
{
	int ret = 0;

	dsp_debug_enter();

	if (dev->status == DSP_OFF)
		goto out;

	dev->resume(dev);

	/*
	 * Before DSP device power off, we must make sure that there is not
	 * coming work request from device client.
	 */
	mutex_lock(&dev->lock);
	dev->client->device_pause(dev->client);

	dsp_dev_trace(dev, dev->trace_index + DSP_TRACE_SLOT_COUNT);

	dsp_mbox_disable(dev->mbox);

	reset_control_assert(dev->core_rst);
	reset_control_assert(dev->sys_rst);
	reset_control_assert(dev->global_rst);
	reset_control_assert(dev->oecm_rst);

	dsp_grf_global_reset_assert(dev);
	dsp_dev_clk_disable(dev);

	dev->status = DSP_OFF;
	pr_info("DSP power off\n");
	mutex_unlock(&dev->lock);
out:
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

	dev->clk_dsp_free = devm_clk_get(&pdev->dev, "clk_dsp_free");
	if (IS_ERR(dev->clk_dsp_free)) {
		dsp_err("failed to get clk_dsp_free\n");
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

	dev->trace_buffer = dma_pool_alloc(dma_pool, GFP_KERNEL, &dma_addr);
	if (dev->trace_buffer) {
		memset(dev->trace_buffer, 0, DSP_TRACE_BUFFER_SIZE);
		dev->trace_dma = dma_addr;
	}

	dev->dsp_dvfs_node = clk_get_dvfs_node("clk_dsp");
	INIT_DELAYED_WORK(&dev->guard_work, dsp_dev_work_timeout);

	dev->device = &pdev->dev;
	(*dev_out) = dev;
out:
	if (ret) {
		if (dev) {
			dsp_loader_destroy(dev->loader);
			dsp_dma_destroy(dev->dma);
		}

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
