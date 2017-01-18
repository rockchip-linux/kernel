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
#include <linux/rockchip/cpu_axi.h>

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

static void __iomem *cpu_qos_base;
static u32 cpu_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *gpu0_qos_base;
static u32 gpu0_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *gpu1_qos_base;
static u32 gpu1_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *emmc_qos_base;
static u32 emmc_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *gmac2io_qos_base;
static u32 gmac2io_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *sdio_qos_base;
static u32 sdio_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *sdmmc_qos_base;
static u32 sdmmc_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *usbhost0_qos_base;
static u32 usbhost0_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *usb3otg_qos_base;
static u32 usb3otg_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *usbotg_qos_base;
static u32 usbotg_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *gmac2phy_qos_base;
static u32 gmac2phy_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *sdmmc_ext_qos_base;
static u32 sdmmc_ext_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *dma_qos_base;
static u32 dma_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *crypto_qos_base;
static u32 crypto_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *tsp_qos_base;
static u32 tsp_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *rkvdec_r_qos_base;
static u32 rkvdec_r_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *rkvdec_w_qos_base;
static u32 rkvdec_w_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *hdcp_qos_base;
static u32 hdcp_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *vop_qos_base;
static u32 vop_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *iep_qos_base;
static u32 iep_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *vip_qos_base;
static u32 vip_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *rga_r_qos_base;
static u32 rga_r_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *rga_w_qos_base;
static u32 rga_w_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *h265_qos_base;
static u32 h265_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *h264_qos_base;
static u32 h264_qos[CPU_AXI_QOS_NUM_REGS];
static void __iomem *vpu_qos_base;
static u32 vpu_qos[CPU_AXI_QOS_NUM_REGS];

#define PD_SAVE_QOS(name) CPU_AXI_SAVE_QOS(name##_qos, name##_qos_base)
#define PD_RESTORE_QOS(name) CPU_AXI_RESTORE_QOS(name##_qos, name##_qos_base)

static int rk322xh_do_idle_request(enum pmu_idle_req req, bool idle)
{
	u32 bit = rk322xh_pmu_idle_map[req];
	u32 idle_mask, target, status;
	u32 mask = BIT(bit);
	u32 val;

	idle_mask = BIT(bit + 16);
	target = (idle << bit) | (idle << (bit + 10));
	status = mask | (1 << (bit + 10));

	val = readl_relaxed(grf + RK322XH_PMU_IDLE_REQ);
	if (idle)
		val |=  mask;
	else
		val &= ~mask;
	writel_relaxed(val | idle_mask, grf + RK322XH_PMU_IDLE_REQ);

	while ((readl_relaxed(grf + RK322XH_PMU_IDLE_ST) & status) != target)
		;

	return 0;
}

static int rk322xh_set_idle_request(enum pmu_idle_req req, bool idle)
{
	unsigned long flags;

	spin_lock_irqsave(&pmu_idle_lock, flags);

	if (idle) {
		if (req == IDLE_REQ_VIO) {
			PD_SAVE_QOS(hdcp);
			PD_SAVE_QOS(vop);
			PD_SAVE_QOS(iep);
			PD_SAVE_QOS(vip);
			PD_SAVE_QOS(rga_r);
			PD_SAVE_QOS(rga_w);
		} else if (req == IDLE_REQ_VIDEO) {
			PD_SAVE_QOS(rkvdec_r);
			PD_SAVE_QOS(rkvdec_w);
		} else if (req == IDLE_REQ_HEVC) {
			PD_SAVE_QOS(h265);
			PD_SAVE_QOS(h264);
		} else if (req == IDLE_REQ_VPU) {
			PD_SAVE_QOS(vpu);
		} else if (req == IDLE_REQ_GPU) {
			PD_SAVE_QOS(gpu0);
			PD_SAVE_QOS(gpu1);
		} else if (req == IDLE_REQ_PERI) {
			PD_SAVE_QOS(emmc);
			PD_SAVE_QOS(gmac2io);
			PD_SAVE_QOS(sdio);
			PD_SAVE_QOS(sdmmc);
			PD_SAVE_QOS(usbhost0);
			PD_SAVE_QOS(usb3otg);
			PD_SAVE_QOS(usbotg);
			PD_SAVE_QOS(gmac2phy);
			PD_SAVE_QOS(sdmmc_ext);
		} else if (req == IDLE_REQ_BUS) {
			PD_SAVE_QOS(dma);
			PD_SAVE_QOS(crypto);
			PD_SAVE_QOS(tsp);
		} else if (req == IDLE_REQ_CORE) {
			PD_SAVE_QOS(cpu);
		}
	}

	rk322xh_do_idle_request(req, idle);

	if (!idle) {
		if (req == IDLE_REQ_VIO) {
			PD_RESTORE_QOS(hdcp);
			PD_RESTORE_QOS(vop);
			PD_RESTORE_QOS(iep);
			PD_RESTORE_QOS(vip);
			PD_RESTORE_QOS(rga_r);
			PD_RESTORE_QOS(rga_w);
		} else if (req == IDLE_REQ_VIDEO) {
			PD_RESTORE_QOS(rkvdec_r);
			PD_RESTORE_QOS(rkvdec_w);
		} else if (req == IDLE_REQ_HEVC) {
			PD_RESTORE_QOS(h265);
			PD_RESTORE_QOS(h264);
		} else if (req == IDLE_REQ_VPU) {
			PD_RESTORE_QOS(vpu);
		} else if (req == IDLE_REQ_GPU) {
			PD_RESTORE_QOS(gpu0);
			PD_RESTORE_QOS(gpu1);
		} else if (req == IDLE_REQ_PERI) {
			PD_RESTORE_QOS(emmc);
			PD_RESTORE_QOS(gmac2io);
			PD_RESTORE_QOS(sdio);
			PD_RESTORE_QOS(sdmmc);
			PD_RESTORE_QOS(usbhost0);
			PD_RESTORE_QOS(usb3otg);
			PD_RESTORE_QOS(usbotg);
			PD_RESTORE_QOS(gmac2phy);
			PD_RESTORE_QOS(sdmmc_ext);
		} else if (req == IDLE_REQ_BUS) {
			PD_RESTORE_QOS(dma);
			PD_RESTORE_QOS(crypto);
			PD_RESTORE_QOS(tsp);
		} else if (req == IDLE_REQ_CORE) {
			PD_RESTORE_QOS(cpu);
		}
	}
	spin_unlock_irqrestore(&pmu_idle_lock, flags);
	return 0;
}

static __init int rk322xh_dt_init(void)
{
	struct device_node *node, *gp, *cp;
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

	node = of_find_compatible_node(NULL, NULL, "rockchip,cpu_axi_bus");
	if (!node)
		return -ENODEV;

#define MAP(name)							\
	do {								\
		cp = of_get_child_by_name(gp, #name);			\
		if (cp)							\
			name##_qos_base = of_iomap(cp, 0);		\
		if (!name##_qos_base)					\
			pr_err("%s: could not map qos %s register\n",	\
			       __func__, #name);			\
	} while (0)

	gp = of_get_child_by_name(node, "qos");
	if (gp) {
		MAP(cpu);
		MAP(gpu0);
		MAP(gpu1);
		MAP(emmc);
		MAP(gmac2io);
		MAP(sdio);
		MAP(sdmmc);
		MAP(usbhost0);
		MAP(usb3otg);
		MAP(usbotg);
		MAP(gmac2phy);
		MAP(sdmmc_ext);
		MAP(dma);
		MAP(crypto);
		MAP(tsp);
		MAP(rkvdec_r);
		MAP(rkvdec_w);
		MAP(hdcp);
		MAP(vop);
		MAP(iep);
		MAP(vip);
		MAP(rga_r);
		MAP(rga_w);
		MAP(h265);
		MAP(h264);
		MAP(vpu);
	}

#undef MAP

	return 0;
}

arch_initcall(rk322xh_dt_init);
