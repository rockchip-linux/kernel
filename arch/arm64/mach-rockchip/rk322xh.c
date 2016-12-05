/*
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/rockchip/common.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/rockchip/cru.h>
#include <linux/rockchip/grf.h>
#include <linux/rockchip/pmu.h>

static void __iomem *grf;

static DEFINE_SPINLOCK(pmu_idle_lock);

static const u8 rk322xh_pmu_idle_map[] = {
	[IDLE_REQ_CORE] = 0,
	[IDLE_REQ_GPU] = 1,
	[IDLE_REQ_BUS] = 2,
	[IDLE_REQ_MSCH] = 3,
	[IDLE_REQ_PERI] = 4,
	[IDLE_REQ_VIDEO] = 5,
	[IDLE_REQ_HEVC] = 6,
	[IDLE_REQ_SYS] = 7,
	[IDLE_REQ_VIO] = 8,
	[IDLE_REQ_VPU] = 9,
};

static int rk322xh_set_idle_request(enum pmu_idle_req req, bool idle)
{
	u32 bit = rk322xh_pmu_idle_map[req];
	u32 idle_mask, target, status;
	u32 mask = BIT(bit);
	u32 val;
	unsigned long flags;

	idle_mask = BIT(bit + 16);
	target = (idle << bit) | (idle << (bit + 10));
	status = mask | (1 << (bit + 10));

	spin_lock_irqsave(&pmu_idle_lock, flags);
	val = readl_relaxed(grf + RK322XH_PMU_IDLE_REQ);
	if (idle)
		val |=  mask;
	else
		val &= ~mask;
	writel_relaxed(val | idle_mask, grf + RK322XH_PMU_IDLE_REQ);

	while ((readl_relaxed(grf + RK322XH_PMU_IDLE_ST) & status) != target)
		;
	spin_unlock_irqrestore(&pmu_idle_lock, flags);

	return 0;
}

static __init int rk322xh_dt_init(void)
{
	struct device_node *node;
	int avs_delta = -5;

	node = of_find_compatible_node(NULL, NULL, "rockchip,rk322xh-grf");
	if (node) {
		grf = of_iomap(node, 0);
		if (!grf) {
			pr_err("%s: could not map grf registers\n", __func__);
			return -ENXIO;
		}
	} else {
		pr_err("%s: could not find grf dt node\n", __func__);
		return -ENODEV;
	}
	rockchip_pmu_ops.set_idle_request = rk322xh_set_idle_request;

	node = of_find_compatible_node(NULL, NULL, "rockchip,avs");
	if (node)
		of_property_read_u32(node, "avs-delta", &avs_delta);

	rockchip_avs_delta = avs_delta;

	return 0;
}

arch_initcall(rk322xh_dt_init);
