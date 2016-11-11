/*
 *
 * Copyright (C) 2015, Fuzhou Rockchip Electronics Co., Ltd
 * Author: Tony.Xie
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm/compiler.h>
#include <asm/suspend.h>
#include <linux/psci.h>
#include <linux/rockchip/psci.h>

#define PSCI_SIP_SUSPEND_CTRBITS		(0x82000003)

#define SEC_REG_RW_SHT				(0x0)
#define SEC_REG_RD				(0x0)
#define SEC_REG_WR				(0x1)
#define SEC_REG_BITS_SHT				(0x1)
#define SEC_REG_32				(0x0)
#define SEC_REG_64				(0x2)
#define SEC_REG_RD_32				(SEC_REG_RD | SEC_REG_32)
#define SEC_REG_RD_64				(SEC_REG_RD | SEC_REG_64)
#define SEC_REG_WR_32				(SEC_REG_WR | SEC_REG_32)
#define SEC_REG_WR_64				(SEC_REG_WR | SEC_REG_64)

#define PSCI_FEATURES				0x8400000A
#define PSCI_SYSTEM_SUSPEND_AARCH64		0xc400000E

extern void set_psci_cpu_suspend(int (*fn)(unsigned long));

static int psci_sys_suspend_finisher(unsigned long unuse)
{
	struct arm_smccc_res res;

	res = rockchip_psci_smc_read(PSCI_SYSTEM_SUSPEND_AARCH64,
				     virt_to_phys(cpu_resume), 0, 0);
	return res.a0;
}

static int cpu_psci_sys_suspend(unsigned long index)
{
	return __cpu_suspend(index, psci_sys_suspend_finisher);
}

static int __init  rk3688_suspend_init(void)
{
	struct device_node *parent;
	u32 pm_ctrbits;
	struct arm_smccc_res res;

	parent = of_find_node_by_name(NULL, "rockchip_suspend");

	if (IS_ERR_OR_NULL(parent)) {
		pr_warn("%s: getting dev node error\n", __func__);
		goto sys_suspend;
	}

	if (of_property_read_u32_array(parent, "rockchip,ctrbits",
				       &pm_ctrbits, 1)) {
		pr_err("%s:get pm ctr error\n", __func__);
		goto sys_suspend;
	}

	rockchip_psci_smc_write(PSCI_SIP_SUSPEND_CTRBITS,
				pm_ctrbits, 0, SEC_REG_WR);
	res = rockchip_psci_smc_read(PSCI_SIP_SUSPEND_CTRBITS,
				     0, 0, SEC_REG_RD);

	if (res.a1 != pm_ctrbits) {
		pr_err("%s read val error\n", __func__);
		goto sys_suspend;
	}

	printk(KERN_INFO "%s: pm_ctrbits =0x%x\n", __func__, pm_ctrbits);

sys_suspend:
	res = rockchip_psci_smc_read(PSCI_FEATURES,
				     PSCI_SYSTEM_SUSPEND_AARCH64, 0, 0);

	if (!res.a0)
		set_psci_cpu_suspend(cpu_psci_sys_suspend);

	return 0;
}

late_initcall_sync(rk3688_suspend_init);
