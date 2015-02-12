/*
 * Copyright (C) 2014 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __SOC_ROCKCHIP_DMC_SYNC_H
#define __SOC_ROCKCHIP_DMC_SYNC_H

#include <linux/notifier.h>

enum dmc_enable_op {
	DMC_ENABLE = 0,
	DMC_DISABLE,
};

extern void rockchip_dmc_lock(void);
extern void rockchip_dmc_wait(ktime_t *timeout);
extern void rockchip_dmc_unlock(void);
extern void rockchip_dmc_enable(void);
extern void rockchip_dmc_disable(void);
extern int rockchip_dmc_get(struct notifier_block *nb);
extern int rockchip_dmc_put(struct notifier_block *nb);
extern int rockchip_dmc_register_enable_notifier(struct notifier_block *nb);
extern int rockchip_dmc_unregister_enable_notifier(struct notifier_block *nb);

#endif /* __SOC_ROCKCHIP_RK3288_DMC_SYNC_H */
