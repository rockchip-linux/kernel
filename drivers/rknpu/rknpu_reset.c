// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 * Author: Felix Zeng <felix.zeng@rock-chips.com>
 */

#include <linux/delay.h>
#include <linux/iommu.h>

#include "rknpu_reset.h"
#include "rknpu_iommu.h"

#ifndef FPGA_PLATFORM
static inline struct reset_control *rknpu_reset_control_get(struct device *dev,
							    const char *name)
{
	struct reset_control *rst = NULL;

	rst = devm_reset_control_get(dev, name);
	if (IS_ERR(rst))
		LOG_DEV_ERROR(dev,
			      "failed to get rknpu reset control: %s, %ld\n",
			      name, PTR_ERR(rst));

	return rst;
}
#endif

int rknpu_reset_get(struct rknpu_device *rknpu_dev)
{
#ifndef FPGA_PLATFORM
	int i = 0;
	int num_srsts = 0;

	num_srsts = of_count_phandle_with_args(rknpu_dev->dev->of_node,
					       "resets", "#reset-cells");
	if (num_srsts <= 0) {
		LOG_DEV_ERROR(rknpu_dev->dev,
			      "failed to get rknpu resets from dtb\n");
		return num_srsts;
	}

	rknpu_dev->srsts = devm_kcalloc(rknpu_dev->dev, num_srsts,
					sizeof(*rknpu_dev->srsts), GFP_KERNEL);
	if (!rknpu_dev->srsts)
		return -ENOMEM;

	for (i = 0; i < num_srsts; ++i) {
		rknpu_dev->srsts[i] = devm_reset_control_get_exclusive_by_index(
			rknpu_dev->dev, i);
		if (IS_ERR(rknpu_dev->srsts[i])) {
			rknpu_dev->num_srsts = i;
			return PTR_ERR(rknpu_dev->srsts[i]);
		}
	}

	rknpu_dev->num_srsts = num_srsts;

	return num_srsts;
#endif

	return 0;
}

#ifndef FPGA_PLATFORM
static int rknpu_reset_assert(struct reset_control *rst)
{
	int ret = -EINVAL;

	if (!rst)
		return -EINVAL;

	ret = reset_control_assert(rst);
	if (ret < 0) {
		LOG_ERROR("failed to assert rknpu reset: %d\n", ret);
		return ret;
	}

	return 0;
}

static int rknpu_reset_deassert(struct reset_control *rst)
{
	int ret = -EINVAL;

	if (!rst)
		return -EINVAL;

	ret = reset_control_deassert(rst);
	if (ret < 0) {
		LOG_ERROR("failed to deassert rknpu reset: %d\n", ret);
		return ret;
	}

	return 0;
}
#endif

int rknpu_soft_reset(struct rknpu_device *rknpu_dev)
{
#ifndef FPGA_PLATFORM
	struct iommu_domain *domain = NULL;
	struct rknpu_subcore_data *subcore_data = NULL;
	int ret = 0, i = 0;

	if (rknpu_dev->bypass_soft_reset) {
		LOG_WARN("bypass soft reset\n");
		return 0;
	}

	if (!mutex_trylock(&rknpu_dev->reset_lock))
		return 0;

	rknpu_dev->soft_reseting = true;

	msleep(100);

	for (i = 0; i < rknpu_dev->config->num_irqs; ++i) {
		subcore_data = &rknpu_dev->subcore_datas[i];
		wake_up(&subcore_data->job_done_wq);
	}

	LOG_INFO("soft reset, num: %d\n", rknpu_dev->num_srsts);

	for (i = 0; i < rknpu_dev->num_srsts; ++i)
		ret |= rknpu_reset_assert(rknpu_dev->srsts[i]);

	udelay(10);

	for (i = 0; i < rknpu_dev->num_srsts; ++i)
		ret |= rknpu_reset_deassert(rknpu_dev->srsts[i]);

	udelay(10);

	if (ret) {
		LOG_DEV_ERROR(rknpu_dev->dev,
			      "failed to soft reset for rknpu: %d\n", ret);
		mutex_unlock(&rknpu_dev->reset_lock);
		return ret;
	}

	/*
	 * A soft reset wipes the MMU, so the page table has to be reprogrammed. This
	 * used to be a detach/attach of iommu_get_domain_for_dev(), which is the
	 * core's default domain -- correct only while the driver overwrote that to
	 * follow its switches. It no longer does, so re-establish the live domain
	 * explicitly instead; otherwise the reset silently leaves the NPU pointed at
	 * domain 0 while the driver believes domain N is live, and the next job is
	 * committed against a page table where its IOVAs do not exist.
	 */
	if (rknpu_dev->iommu_en) {
		int rp = rk_iommu_reprogram(rknpu_dev->dev);

		if (rp)
			LOG_DEV_ERROR(rknpu_dev->dev,
				      "failed to reprogram iommu after reset: %d\n",
				      rp);
	}
	(void)domain;

	rknpu_dev->soft_reseting = false;

	if (rknpu_dev->config->state_init != NULL)
		rknpu_dev->config->state_init(rknpu_dev);

	mutex_unlock(&rknpu_dev->reset_lock);
#endif

	return 0;
}
