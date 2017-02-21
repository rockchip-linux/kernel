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

#include <linux/init.h>
#include <linux/io.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/rockchip/cpu.h>
#include <linux/rockchip/grf.h>
#include <linux/rockchip/iomap.h>
#include <linux/suspend.h>
#include <linux/wakeup_reason.h>
#include "pm.h"

#define GPIO_INTEN		0x30
#define GPIO_INTMASK		0x34
#define GPIO_INT_STATUS		0x40
#define GIC_DIST_PENDING_SET	0x200
#define DUMP_GPIO_INTEN(ID) \
do { \
	u32 en = readl_relaxed(RK_GPIO_VIRT(ID) + GPIO_INTEN); \
	if (en) {\
		rkpm_ddr_printascii("GPIO" #ID "_INTEN: "); \
		rkpm_ddr_printhex(en); \
		rkpm_ddr_printch('\n'); \
	} \
} while (0)
#define RV1108_CRU_UNGATING_OPS(id) cru_writel( \
	CRU_W_MSK_SETBITS(0,  (id) % 16, 0x1), RV1108_CRU_GATEID_CONS((id)))
#define RV1108_CRU_GATING_OPS(id) cru_writel( \
	CRU_W_MSK_SETBITS(1, (id) % 16, 0x1), RV1108_CRU_GATEID_CONS((id)))

enum rk_plls_id {
	APLL_ID = 0,
	DPLL_ID,
	GPLL_ID,
	END_PLL_ID
};

#define RV1108_EVB_V11

static inline void  uart_printch(char bbyte)
{
	u32 reg_save[2];
#ifdef RV1108_EVB_V11
	u32 u_clk_id = (RV1108_CLKGATE_UART2_SRC + 0 * 2);
	u32 u_pclk_id = (RV1108_CLKGATE_PCLK_UART2 + 0);
#else
	u32 u_clk_id = (RV1108_CLKGATE_UART0_SRC + 0 * 2);
	u32 u_pclk_id = (RV1108_CLKGATE_PCLK_UART0 + 0);
#endif
	reg_save[0] = cru_readl(RV1108_CRU_GATEID_CONS(u_clk_id));
	reg_save[1] = cru_readl(RV1108_CRU_GATEID_CONS(u_pclk_id));
	RV1108_CRU_UNGATING_OPS(u_clk_id);
	RV1108_CRU_UNGATING_OPS(u_pclk_id);

	rkpm_udelay(1);

write_uart:
	writel_relaxed(bbyte, RK_DEBUG_UART_VIRT);
	dsb();
	while (!(readl_relaxed(RK_DEBUG_UART_VIRT + 0x14) & 0x40))
		barrier();

	if (bbyte == '\n') {
		bbyte = '\r';
		goto write_uart;
	}

	cru_writel(reg_save[0] | CRU_W_MSK(u_clk_id % 16, 0x1),
		   RV1108_CRU_GATEID_CONS(u_clk_id));
	cru_writel(reg_save[1] | CRU_W_MSK(u_pclk_id % 16, 0x1),
		   RV1108_CRU_GATEID_CONS(u_pclk_id));
}

void PIE_FUNC(sram_printch)(char byte)
{
	uart_printch(byte);
}

static void pll_udelay(u32 udelay)
{
#if 0
	u32 mode;

	mode = cru_readl(RV1108_CRU_MODE_CON);
	cru_writel(RV1108_PLL_MODE_SLOW(APLL_ID), RV1108_CRU_MODE_CON);
	rkpm_udelay(udelay * 5);
	cru_writel(mode | (RV1108_PLL_MODE_MSK(APLL_ID) << 16),
		   RV1108_CRU_MODE_CON);
#endif
}

#define RV1108_PLL_MODE_SLOW	((0x0 << 8) | \
		(0x1 << (16 + RV1108_PLLS_MODE_OFFSET)))

#define RV1108_PLL_MODE_NORM	((0x1 << 8) | \
		(0x1 << (16 + RV1108_PLLS_MODE_OFFSET)))

#define RV1108_PLL_POWERDOWN	((0x1 << 0) | \
		(0x1 << (16 + 0)))

#define RV1108_PLL_POWERON	((0x0 << 0) | \
		(0x1 << (16 + 0)))

static void pm_pll_wait_lock(u32 pll_idx)
{
	u32 delay = 600000U;

	dsb();
	dsb();
	dsb();
	dsb();
	dsb();
	dsb();
	while (delay > 0) {
		if ((cru_readl(RV1108_PLL_CONS(pll_idx, 2)) & (0x1 << 31))) {
			rkpm_ddr_printascii("lock-pll-ok:");
			rkpm_ddr_printhex(pll_idx);
			rkpm_ddr_printch('\n');
			break;
		}
		delay--;
	}
	if (delay == 0) {
		rkpm_ddr_printascii("unlock-pll:");
		rkpm_ddr_printhex(pll_idx);
		rkpm_ddr_printch('\n');
	}
}

/* #define RV1108_SOFT_PD_PLL */
#define RV1108_SUSPEND_DEBUG
static u32 cru_plls_con_save[END_PLL_ID][6];

static inline void plls_suspend(u32 pll_id)
{
	int i;

	cru_writel(RV1108_PLL_MODE_SLOW, RV1108_PLL_CONS(pll_id, 3));

#ifdef RV1108_SOFT_PD_PLL
	cru_writel(RV1108_PLL_POWERDOWN, RV1108_PLL_CONS((pll_id), 3));
#endif

	for (i = 0; i < 6; i++)
		cru_plls_con_save[pll_id][i] = cru_readl(
				RV1108_PLL_CONS(pll_id, i));
}

static inline void plls_resume(u32 pll_id)
{
	cru_writel(cru_plls_con_save[pll_id][0] | 0xffff0000,
		   RV1108_PLL_CONS(pll_id, 0));
	cru_writel(cru_plls_con_save[pll_id][1] | 0xffff0000,
		   RV1108_PLL_CONS(pll_id, 1));
	cru_writel(cru_plls_con_save[pll_id][2],
		   RV1108_PLL_CONS(pll_id, 2));
	cru_writel(cru_plls_con_save[pll_id][3] | 0xffff0000,
		   RV1108_PLL_CONS(pll_id, 3));
	cru_writel(cru_plls_con_save[pll_id][4] | 0xffff0000,
		   RV1108_PLL_CONS(pll_id, 4));
	cru_writel(cru_plls_con_save[pll_id][5] | 0xffff0000,
		   RV1108_PLL_CONS(pll_id, 5));

#ifdef RV1108_SOFT_PD_PLL
	cru_writel(RV1108_PLL_POWERON, RV1108_PLL_CONS((pll_id), 3));
#endif
	pm_pll_wait_lock(pll_id);

	cru_writel(RV1108_PLL_MODE_NORM, RV1108_PLL_CONS(pll_id, 3));
}

static u32 clk_sel0, clk_sel1, clk_sel11;

static void pm_plls_suspend(void)
{
	clk_sel0 = cru_readl(RV1108_CRU_CLKSELS_CON(0));
	clk_sel1 = cru_readl(RV1108_CRU_CLKSELS_CON(1));
	clk_sel11 = cru_readl(RV1108_CRU_CLKSELS_CON(11));

	plls_suspend(GPLL_ID);
	plls_suspend(APLL_ID);

#if 0
	/* core */
	cru_writel(CRU_W_MSK_SETBITS(0, 0, 0x1f),
		   RV1108_CRU_CLKSELS_CON(0));

	/* pclk_dbg */
	cru_writel(CRU_W_MSK_SETBITS(3, 4, 0xf),
		   RV1108_CRU_CLKSELS_CON(1));

	/* crypto */
	cru_writel(CRU_W_MSK_SETBITS(0, 0, 0x1f),
		   RV1108_CRU_CLKSELS_CON(11));
#endif
#ifdef RV1108_SUSPEND_DEBUG
{
	int i;
	enum rk_plls_id plls_id;

	rkpm_ddr_printascii("\npll_sus_dump\n");
	for (plls_id = APLL_ID; plls_id < END_PLL_ID; plls_id++) {
		for (i = 0; i < 6; i++) {
			rkpm_ddr_printhex(
				cru_readl(RV1108_PLL_CONS(plls_id, i))
			);
			rkpm_ddr_printch(' ');
		}
		rkpm_ddr_printch('\n');
	}
	rkpm_ddr_printascii("pllsus1\n");
}
#endif
}

static void pm_plls_resume(void)
{
#if 0
	/* crypto */
	cru_writel(clk_sel11 | CRU_W_MSK(0, 0x1f),
		   RV1108_CRU_CLKSELS_CON(11));

	/* pclk_dbg */
	cru_writel(clk_sel1 | CRU_W_MSK(4, 0xf),
		   RV1108_CRU_CLKSELS_CON(1));

	/* core */
	cru_writel(clk_sel0 | CRU_W_MSK(0, 0x1f),
		   RV1108_CRU_CLKSELS_CON(0));
#endif

	plls_resume(APLL_ID);
	plls_resume(GPLL_ID);

#ifdef RV1108_SUSPEND_DEBUG
{
	int i;
	enum rk_plls_id plls_id;

	rkpm_ddr_printascii("pllres_dump:\n");
	for (plls_id = APLL_ID; plls_id < END_PLL_ID; plls_id++) {
		for (i = 0; i < 6; i++) {
			rkpm_ddr_printhex(
				cru_readl(RV1108_PLL_CONS(plls_id, i))
			);
			rkpm_ddr_printch(' ');
		}
		rkpm_ddr_printch('\n');
	}
}
#endif
}

#ifdef CONFIG_RK_LAST_LOG
extern void rk_last_log_text(char *text, size_t size);
#endif

static void  ddr_printch(char byte)
{
	uart_printch(byte);
#ifdef CONFIG_RK_LAST_LOG
	rk_last_log_text(&byte, 1);

	if (byte == '\n') {
		byte = '\r';
		rk_last_log_text(&byte, 1);
	}
#endif
	pll_udelay(2);
}

static noinline void rv1108_pm_dump_inten(void)
{
	DUMP_GPIO_INTEN(0);
	DUMP_GPIO_INTEN(1);
	DUMP_GPIO_INTEN(2);
	DUMP_GPIO_INTEN(3);
	rkpm_ddr_printascii("gpioMask:");
	rkpm_ddr_printhex(readl_relaxed(RK_GPIO_VIRT(0) + GPIO_INTMASK));
	rkpm_ddr_printch('\n');
}

static noinline void rv1108_pm_dump_irq(void)
{
	u32 irq[4];
	int i;

	u32 irq_gpio = (readl_relaxed(RK_GIC_VIRT +
			GIC_DIST_PENDING_SET + 8) >> 8) & 0x0F;
	rkpm_ddr_printascii("irq_gpio:");
	rkpm_ddr_printhex(irq_gpio);
	rkpm_ddr_printch('\n');

	for (i = 0; i < ARRAY_SIZE(irq); i++)
		irq[i] = readl_relaxed(RK_GIC_VIRT +
				       GIC_DIST_PENDING_SET + (1 + i) * 4);
	for (i = 0; i < ARRAY_SIZE(irq); i++) {
		if (irq[i])
			log_wakeup_reason(32 * (i + 1) + fls(irq[i]) - 1);
	}
	for (i = 0; i <= 3; i++) {
		if (irq_gpio & (1 << i)) {
			pr_debug("wakeup gpio%d: %08x\n", i,
				 readl_relaxed(RK_GPIO_VIRT(i) +
				 GPIO_INT_STATUS));
			rkpm_ddr_printascii("wakeup gpio");
			rkpm_ddr_printhex(i);
			rkpm_ddr_printascii(":");
			rkpm_ddr_printhex(readl_relaxed(RK_GPIO_VIRT(i) +
					  GPIO_INT_STATUS));
			rkpm_ddr_printch('\n');
		}
	}
}

static void rkpm_prepare(void)
{
	rv1108_pm_dump_inten();
}

static void rkpm_finish(void)
{
	rv1108_pm_dump_irq();
}

enum pmu_wakeup_cfg2 {
	wakeup_int_cluster = 0,
	wakeup_gpio_int_en = 2,
	wakeup_sdio_en = 3,
	wakeup_sdmmc_en = 4,
	wakeup_timer_en = 6,
	wakeup_usbdev_en = 7,
	wakeup_timeout_en = 10
};

enum rv1108_pwr_mode_core_con {
	pmu_global_int_disable = 0,
	pmu_clr_core = 5,
	pmu_scu_pd_en = 6,
	pmu_apll_pd_en = 12,
	pmu_dpll_pd_en = 13,
	pmu_gpll_pd_en = 14
};

enum rv1108_pwr_mode_common_con {
	pmu_power_mode_en = 0,
	pmu_wakeup_reset_en = 3,
	pmu_pll_pd_en = 4,
	pmu_power_off_req_cfg = 5,
	pmu_pmu_use_lf = 6,
	pmu_ddrphy_gating_en = 7,
	pmu_osc_24m_dis = 8,
	pmu_input_clamp_en = 9,
	pmu_sref_enter_en = 10,
	pmu_ddrc_gating_en = 11,
	pmu_ddr_ret_en = 12,
	pmu_ddr_ret_de_req = 13,
	pmu_clr_pmu = 14,
	pmu_clr_bus = 16,
	pmu_clr_dsp = 17,
	pmu_clr_msch = 18,
	pmu_clr_venc = 19,
	pmu_clr_peri = 20,
	pmu_clr_vdec = 21,
	pmu_clr_vio = 22,
	pmu_ddrphy_gating_enable = 25,
	pmu_wait_wakeup_begin_cfg = 28,
	pmu_core_clk_src_gate_en = 29,
	pmu_peri_clk_src_gate_en = 30,
	pmu_bus_clk_src_gate_en = 31
};

#define RK_PMU_GRF_VIRT			(RK_PMU_VIRT + SZ_4K)

#define RK_PMU_MEM_VIRT			(RK_BOOTRAM_VIRT + SZ_32K)
#define RKPM_BOOTRAM_PHYS		(RV1108_PMU_MEM_PHYS)
#define RKPM_BOOTRAM_BASE		(RK_PMU_MEM_VIRT)
#define RKPM_BOOTRAM_SIZE		(RV1108_PMU_MEM_SIZE)

#define RKPM_BOOT_CODE_OFFSET		(0x0)
#define RV1108PM_BOOT_CODE_SIZE		(0x700)

#define RV1108PM_BOOT_DATA_OFFSET	(RKPM_BOOT_CODE_OFFSET + \
					 RV1108PM_BOOT_CODE_SIZE)

#define RV1108PM_BOOT_DDRCODE_OFFSET	(RV1108PM_BOOT_DATA_OFFSET + \
					 RKPM_BOOT_DATA_SIZE)

#define  RKPM_BOOT_CODE_PHY	(RKPM_BOOTRAM_PHYS + RKPM_BOOT_CODE_OFFSET)
#define  RKPM_BOOT_CODE_BASE	(RKPM_BOOTRAM_BASE + RKPM_BOOT_CODE_OFFSET)

#define  RKPM_BOOT_DATA_PHY	(RKPM_BOOTRAM_PHYS + \
				 RV1108PM_BOOT_DATA_OFFSET)

#define  RKPM_BOOT_DATA_BASE	(RKPM_BOOTRAM_BASE + \
				 RV1108PM_BOOT_DATA_OFFSET)

/* ddr resume data in boot ram */
#define  RKPM_BOOT_DDRCODE_PHY	(RKPM_BOOTRAM_PHYS + \
				 RV1108PM_BOOT_DDRCODE_OFFSET)

#define  RKPM_BOOT_DDRCODE_BASE	(RKPM_BOOTRAM_BASE + \
				 RV1108PM_BOOT_DDRCODE_OFFSET)

#define RKPM_BOOT_CPUSP_PHY	(RKPM_BOOTRAM_PHYS + \
				((RKPM_BOOTRAM_SIZE - 1) & (~(0x7))))

/* the value is used to control cpu resume flow */
static u32 sleep_resume_data[RKPM_BOOTDATA_ARR_SIZE];
static char *resume_data_base = (char *)(RKPM_BOOT_DATA_BASE);

/*****save boot sram**********************/
#define BOOT_RAM_SAVE_SIZE	(RKPM_BOOTRAM_SIZE + 4 * 10)
#define INT_RAM_SIZE		(64 * 1024)
static char boot_ram_data[BOOT_RAM_SAVE_SIZE];/* 8K + 40byte */
static char int_ram_data[INT_RAM_SIZE];

extern void rv1108_pm_slp_cpu_resume(void);

static void sram_data_for_sleep(char *boot_save, char *int_save, u32 flag)
{
	char *addr_base, *data_src, *data_dst;
	u32 sr_size, data_size;

	/**********save boot sarm***********************************/
	addr_base = (char *)RKPM_BOOTRAM_BASE;
	sr_size = RKPM_BOOTRAM_SIZE;
	if (boot_save)
		memcpy(boot_save, addr_base, sr_size);

	/**********move  resume code and data to boot sram*************/
	data_dst = (char *)RKPM_BOOT_CODE_BASE;
	data_src = (char *)rv1108_pm_slp_cpu_resume;
	data_size = RV1108PM_BOOT_CODE_SIZE;
	memcpy(data_dst, data_src, data_size);

	data_dst = (char *)resume_data_base;
	data_src = (char *)sleep_resume_data;
	data_size = sizeof(sleep_resume_data);
	memcpy((char *)data_dst, (char *)data_src, data_size);
}

#define RV1108_PMUGRF_DLL_CON0			(0x0180)
#define RV1108_PMUGRF_DLL_CON1			(0x0184)
#define RV1108_PMUGRF_DLL_STATUS0		(0x0190)
#define RV1108_PMUGRF_DLL_STATUS1		(0x0194)
#define RV1108_PMUGRF_SOC_CON0			(0x0100)
#define RV1108_PMUGRF_FAST_BOOT_ADDR		(0x0300)

static u32 pmu_grf_dll_con0;
static u32 pmu_grf_dll_con1;
static u32 cru_clk_gate8;
static u32 pmu_grf_soc_con0;
static u32 pmu_grf_fast_boot_addr;
static u32 cru_clk_gate12;

static u32 rv1108_core_powermode;
static u32 rv1108_common_powermode;
static u32 pmu_pwrmode_core_con;
static u32 pmu_pwrmode_common_con;
static u32 pmu_wakeup_conf0;
static u32 pmu_wakeup_conf1;
static u32 pmu_wakeup_conf2;
static u32 gpio_pmic_sleep_mode;
static u32 pmic_sleep_gpio;
static u32 pmu_sft_con;

#define RV1108_24MOSC_CNT	(60)
/* #define RV1108_SOFT_USE_LF */
/* #define RV1108_TIMEOUT_WACKUP */
/* #define RV1108_PMU_DEBUG */
#ifdef RV1108_PMU_DEBUG
static u32 pmu_grf_gpio0a_iomux;
static u32 pmu_grf_gpio0c_iomux;
#endif

static u32 rkpm_slp_mode_set(u32 ctrbits)
{
	int i;
	u32 wakeup_conf2 = 0;
	u32 pwr_mode_core_config = 0;
	u32 pwr_mode_common_config = 0;
	u32 pmugrf_soc_con0 = 0;

	if (((RKPM_CTR_ARMOFF_LPMD |
	      RKPM_CTR_IDLESRAM_MD |
	      RKPM_CTR_ARMLOGDP_LPMD) & ctrbits) == 0)
		return 0;

#ifdef RV1108_PMU_DEBUG
	/* set pmu debug */
	pmu_grf_gpio0a_iomux = pmu_grf_readl(0x0);
	pmu_grf_gpio0c_iomux = pmu_grf_readl(0x8);
	pmu_grf_writel(CRU_W_MSK_SETBITS(0x1, 12, 0x3), 0x0);
	pmu_grf_writel(CRU_W_MSK_SETBITS(0x1, 4, 0x3) |
			CRU_W_MSK_SETBITS(0x1, 6, 0x3) |
			CRU_W_MSK_SETBITS(0x2, 10, 0x3) |
			CRU_W_MSK_SETBITS(0x2, 12, 0x3), 0x8);
#endif
#if 0
	cru_misc_con = cru_readl(RV1108_CRU_MISC_CON);
	pmu_grf_writel(0 | CRU_W_MSK_SETBITS(0x2, 12, 0x3), 0x0);
	cru_writel(CRU_W_MSK_SETBITS(0x0, 8, 0xf), RV1108_CRU_MISC_CON);
	cru_writel(0x02000000, RV1108_CRU_CLKGATES_CON(9));
#endif
	/* config wakeup source */
	pmu_wakeup_conf0 = pmu_readl(RV1108_PMU_WAKEUP_CFG0);
	pmu_wakeup_conf1 = pmu_readl(RV1108_PMU_WAKEUP_CFG1);
	pmu_wakeup_conf2 = pmu_readl(RV1108_PMU_WAKEUP_CFG2);

	pmu_writel(0x0, RV1108_PMU_WAKEUP_CFG0);
	pmu_writel(0x0, RV1108_PMU_WAKEUP_CFG1);
	wakeup_conf2 = 0
		     | BIT(wakeup_gpio_int_en)
#ifdef RV1108_TIMEOUT_WACKUP
		     | BIT(wakeup_timeout_en)
#endif
		     ;
	pmu_writel(wakeup_conf2, RV1108_PMU_WAKEUP_CFG2);

	/* set sleep gpio, GPIO0_B5, [10, 11]=01:pmu_sleep*/
	gpio_pmic_sleep_mode = pmu_grf_readl(0x04);
	pmu_grf_writel(CRU_W_MSK_SETBITS(0x1, 10, 0x3), 0x04);

	/* set pmu mode */
	pmu_pwrmode_core_con = pmu_readl(RV1108_PMU_PWRMODE_CORE_CON);
	pmu_pwrmode_common_con = pmu_readl(RV1108_PMU_PWRMODE_COMMON_CON);

	pwr_mode_core_config = 0
				| BIT(pmu_global_int_disable)
				| BIT(pmu_clr_core)
				/* | BIT(pmu_scu_pd_en) */
				| BIT(pmu_dpll_pd_en)
#ifndef RV1108_SOFT_PD_PLL
				| BIT(pmu_apll_pd_en)
				| BIT(pmu_gpll_pd_en)
#endif
				;

	pwr_mode_common_config = 0
				| BIT(pmu_power_mode_en)
				| BIT(pmu_wait_wakeup_begin_cfg)
				/* | BIT(pmu_wakeup_reset_en) */
				| BIT(pmu_pll_pd_en)
				/* | BIT(pmu_power_off_req_cfg) */
				/* | BIT(pmu_pmu_use_lf) */
				| BIT(pmu_osc_24m_dis)
				/* | BIT(pmu_input_clamp_en) */

				| BIT(pmu_ddrphy_gating_en)
				| BIT(pmu_sref_enter_en)
				| BIT(pmu_ddrc_gating_en)
				| BIT(pmu_ddr_ret_en)
				/* | BIT(pmu_ddr_ret_de_req) */
				| BIT(pmu_ddrphy_gating_enable)

				| BIT(pmu_core_clk_src_gate_en)
#if 0
				| BIT(pmu_clr_dsp)

				| BIT(pmu_clr_vio)

				| BIT(pmu_clr_vdec)
				| BIT(pmu_clr_venc)

				| BIT(pmu_clr_msch)

				| BIT(pmu_clr_peri)
				| BIT(pmu_peri_clk_src_gate_en)

				| BIT(pmu_clr_pmu)

				| BIT(pmu_clr_bus)
				| BIT(pmu_bus_clk_src_gate_en)
#endif
				;

	if (ctrbits & (RKPM_CTR_ARMOFF_LPMD | RKPM_CTR_ARMLOGDP_LPMD)) {
		pwr_mode_core_config |= 0
				     | BIT(pmu_scu_pd_en)
				     ;
	}

	if (ctrbits & RKPM_CTR_ARMOFF_LPMD) {
		pwr_mode_common_config |= 0
				       | BIT(pmu_wakeup_reset_en)
				       | BIT(pmu_input_clamp_en)
				       ;
	}

	pmu_grf_dll_con0 = pmu_grf_readl(RV1108_PMUGRF_DLL_CON0);
	pmu_grf_dll_con1 = pmu_grf_readl(RV1108_PMUGRF_DLL_CON1);
	cru_clk_gate8 = cru_readl(RV1108_CRU_CLKGATES_CON(8));
	/* set 24m osc disable */
	if (pwr_mode_common_config & BIT(pmu_osc_24m_dis)) {
		pwr_mode_common_config |= BIT(pmu_pmu_use_lf);
		pmu_writel(RV1108_24MOSC_CNT * 32, RV1108_PMU_OSC_CNT);
	}
	/* set pvtm */
	if (pwr_mode_common_config & BIT(pmu_pmu_use_lf)) {
		/* enable pvtm clk */
		cru_writel(0x20000000, RV1108_CRU_CLKGATES_CON(8));
		/* enable pvtm */
		pmu_grf_writel(0x00020002, RV1108_PMUGRF_DLL_CON0);
		for (i = 0; i < 10; i++)
			asm("nop;");

		pmu_grf_writel(0x01000, RV1108_PMUGRF_DLL_CON1);
		pmu_grf_writel(CRU_W_MSK_SETBITS(511, 2, 0xfff),
			       RV1108_PMUGRF_DLL_CON0);
		for (i = 0; i < 10; i++)
			asm("nop;");

		pmu_grf_writel(0x00010001, RV1108_PMUGRF_DLL_CON0);
		while ((pmu_grf_readl(RV1108_PMUGRF_DLL_STATUS0) & BIT(0)) == 0)
			;
#ifdef RV1108_TIMEOUT_WACKUP
		pmu_writel(300 * 32, RV1108_PMU_TIMEOUT_CNT);
#endif

#ifdef RV1108_SOFT_USE_LF
		pwr_mode_common_config &= (~BIT(pmu_pmu_use_lf));
#endif
	}

	/* enable ddr retention */
	pmu_sft_con = pmu_readl(RV1108_PMU_SFT_CON);
	pmu_grf_soc_con0 = pmu_grf_readl(RV1108_PMUGRF_SOC_CON0);
	if (pwr_mode_common_config & BIT(pmu_ddr_ret_en)) {
		/* firstly, set pmu_sft retention disable */
		pmu_writel(pmu_sft_con & ~(1 << 11), RV1108_PMU_SFT_CON);

		/* and then, set grf_soc_con0 retention enable */
		pmugrf_soc_con0 |= CRU_W_MSK_SETBITS(0x0, 2, 0x1);
	}

	/* set pmu reset hold */
	if (pwr_mode_common_config & BIT(pmu_wakeup_reset_en)) {
		pmugrf_soc_con0 |= 0
				| CRU_W_MSK_SETBITS(0xb, 3, 0xf)/* problem */
				| CRU_W_MSK_SETBITS(0x1, 8, 0x1)
				| CRU_W_MSK_SETBITS(0x1, 10, 0x1)
				| CRU_W_MSK_SETBITS(0x3, 12, 0x3)
				;
	}
	pmu_grf_writel(pmugrf_soc_con0, RV1108_PMUGRF_SOC_CON0);

	/* set fast boot addr */
	pmu_grf_fast_boot_addr = pmu_grf_readl(RV1108_PMUGRF_FAST_BOOT_ADDR);
	cru_clk_gate12 = cru_readl(RV1108_CRU_CLKGATES_CON(12));
	if (ctrbits & (RKPM_CTR_ARMOFF_LPMD | RKPM_CTR_ARMLOGDP_LPMD)) {
		pmu_grf_writel(RV1108_PMU_MEM_PHYS,
			       RV1108_PMUGRF_FAST_BOOT_ADDR);
		cru_writel(0x000a0000, RV1108_CRU_CLKGATES_CON(12));
		cru_writel(0x0, RV1108_CRU_GLB_RST_ST);
	}

	/* set pmu mode */
	pmu_writel(pwr_mode_core_config, RV1108_PMU_PWRMODE_CORE_CON);
	pmu_writel(pwr_mode_common_config, RV1108_PMU_PWRMODE_COMMON_CON);
	rv1108_core_powermode = pwr_mode_core_config;
	rv1108_common_powermode = pwr_mode_common_config;
#ifdef RV1108_SUSPEND_DEBUG
	rkpm_ddr_printascii("pwr_mode_core_config:");
	rkpm_ddr_printhex(pmu_readl(RV1108_PMU_PWRMODE_CORE_CON));
	rkpm_ddr_printch('\n');
	rkpm_ddr_printascii("pwr_mode_common_config:");
	rkpm_ddr_printhex(pmu_readl(RV1108_PMU_PWRMODE_COMMON_CON));
	rkpm_ddr_printch('\n');
	rkpm_ddr_printascii("pmu_sft_con:");
	rkpm_ddr_printhex(pmu_readl(RV1108_PMU_SFT_CON));
	rkpm_ddr_printascii("\n");
	rkpm_ddr_printascii("grf_soc_con0:");
	rkpm_ddr_printhex(pmu_grf_readl(RV1108_PMUGRF_SOC_CON0));
	rkpm_ddr_printascii("\n");
	rkpm_ddr_printascii("\nPMUGRF_GPIO_BMUX_:");
	rkpm_ddr_printhex(pmu_grf_readl(0x04));
	rkpm_ddr_printascii(" ");
	rkpm_ddr_printhex(gpio_pmic_sleep_mode);
	rkpm_ddr_printascii("\n");
	rkpm_ddr_printascii("\nPMUGRF_DLL_CON0:");
	rkpm_ddr_printhex(pmu_grf_readl(RV1108_PMUGRF_DLL_CON0));
	rkpm_ddr_printascii("\n");
	rkpm_ddr_printascii("PMUGRF_DLL_CON1:");
	rkpm_ddr_printhex(pmu_grf_readl(RV1108_PMUGRF_DLL_CON1));
	rkpm_ddr_printascii("\n");
	rkpm_ddr_printascii("PMUGRF_DLL__STATUS0:");
	rkpm_ddr_printhex(pmu_grf_readl(RV1108_PMUGRF_DLL_STATUS0));
	rkpm_ddr_printascii("\n");
	rkpm_ddr_printascii("PMUGRF_DLL__STATUS1:");
	rkpm_ddr_printhex(pmu_grf_readl(RV1108_PMUGRF_DLL_STATUS1));
	rkpm_ddr_printascii("\n");
	rkpm_ddr_printascii("fast_boot_addr:");
	rkpm_ddr_printhex(pmu_grf_readl(RV1108_PMUGRF_FAST_BOOT_ADDR));
	rkpm_ddr_printch('\n');
	rkpm_ddr_printascii("PMU_STABLE_CNT:");
	rkpm_ddr_printhex(pmu_readl(RV1108_PMU_STABLE_CNT));
	rkpm_ddr_printch('\n');
	rkpm_ddr_printascii("RV1108_CRU_MISC_CON:");
	rkpm_ddr_printhex(cru_readl(RV1108_CRU_MISC_CON));
	rkpm_ddr_printascii("\n");
	rkpm_ddr_printascii("sleep set end\n");
#endif
	return 1;
}

static void sram_code_data_save(u32 core_power_mode, u32 common_power_mode)
{
	sleep_resume_data[RKPM_BOOTDATA_L2LTY_F] = 0;
	if (rkpm_chk_ctrbits(RKPM_CTR_VOL_PWM0))
		sleep_resume_data[RKPM_BOOTDATA_ARM_ERRATA_818325_F] |= 0x01;
	if (rkpm_chk_ctrbits(RKPM_CTR_VOL_PWM1))
		sleep_resume_data[RKPM_BOOTDATA_ARM_ERRATA_818325_F] |= 0x02;
	if (rkpm_chk_ctrbits(RKPM_CTR_VOL_PWM2))
		sleep_resume_data[RKPM_BOOTDATA_ARM_ERRATA_818325_F] |= 0x04;
	sleep_resume_data[RKPM_BOOTDATA_DDR_F] = 0;
	sleep_resume_data[RKPM_BOOTDATA_CPUSP] = RKPM_BOOT_CPUSP_PHY;
	/* in sys resume ,ddr is need resume */
	sleep_resume_data[RKPM_BOOTDATA_CPUCODE] = virt_to_phys(cpu_resume);

	sram_data_for_sleep(boot_ram_data,
			    int_ram_data,
			    sleep_resume_data[RKPM_BOOTDATA_DDR_F]);

	flush_cache_all();
	outer_flush_all();
	local_flush_tlb_all();
}

#define RK_GICD_BASE		(RK_GIC_VIRT)
#define RK_GICC_BASE		(RK_GIC_VIRT + RV1108_GIC_DIST_SIZE)
#define PM_IRQN_START		(32)
#define PM_IRQN_END		(107)
#define gic_reg_dump(a, b, c)	{}
static u32 slp_gic_save[260 + 50];

static void rkpm_gic_dist_save(u32 *context)
{
	int i = 0, j, irqstart = 0;
	unsigned int gic_irqs;

	gic_irqs = readl_relaxed(RK_GICD_BASE + GIC_DIST_CTR) & 0x1f;
	gic_irqs = (gic_irqs + 1) * 32;
	if (gic_irqs > 1020)
		gic_irqs = 1020;

	irqstart = PM_IRQN_START;

	i = 0;

	for (j = irqstart; j < gic_irqs; j += 16)
		context[i++] = readl_relaxed(RK_GICD_BASE +
					     GIC_DIST_CONFIG +
					     (j * 4) / 16);
	gic_reg_dump("gic level", j, RK_GICD_BASE + GIC_DIST_CONFIG);

	/* Set all global interrupts to this CPU only. */
	for (j = 0; j < gic_irqs; j += 4)
		context[i++] = readl_relaxed(RK_GICD_BASE +
					     GIC_DIST_TARGET +
					     (j * 4) / 4);
	gic_reg_dump("gic trig", j, RK_GICD_BASE + GIC_DIST_TARGET);

	for (j = 0; j < gic_irqs; j += 4)
		context[i++] = readl_relaxed(RK_GICD_BASE +
					     GIC_DIST_PRI +
					     (j * 4) / 4);
	gic_reg_dump("gic pri", j, RK_GICD_BASE + GIC_DIST_PRI);

	for (j = 0; j < gic_irqs; j += 32)
		context[i++] = readl_relaxed(RK_GICD_BASE +
					     GIC_DIST_IGROUP +
					     (j * 4) / 32);
	gic_reg_dump("gic secure", j, RK_GICD_BASE + 0x80);

	for (j = irqstart; j < gic_irqs; j += 32)
		context[i++] = readl_relaxed(RK_GICD_BASE +
					     GIC_DIST_PENDING_SET +
					     (j * 4) / 32);
	gic_reg_dump("gic PENDING", j, RK_GICD_BASE +
		     GIC_DIST_PENDING_SET);

	for (j = 0; j < gic_irqs; j += 32)
		context[i++] = readl_relaxed(RK_GICD_BASE +
					     GIC_DIST_ENABLE_SET +
					     (j * 4) / 32);
	gic_reg_dump("gic en", j, RK_GICD_BASE + GIC_DIST_ENABLE_SET);
	gic_reg_dump("gicc", 0x1c, RK_GICC_BASE);
	gic_reg_dump("giccfc", 0, RK_GICC_BASE + 0xfc);

	context[i++] = readl_relaxed(RK_GICC_BASE + GIC_CPU_PRIMASK);
	context[i++] = readl_relaxed(RK_GICD_BASE + GIC_DIST_CTRL);
	context[i++] = readl_relaxed(RK_GICC_BASE + GIC_CPU_CTRL);

	for (j = irqstart; j < gic_irqs; j += 32) {
		writel_relaxed(0xffffffff, RK_GICD_BASE +
			       GIC_DIST_ENABLE_CLEAR + j * 4 / 32);
		dsb();
	}
	writel_relaxed(0xffff0000, RK_GICD_BASE + GIC_DIST_ENABLE_CLEAR);
	writel_relaxed(0x0000ffff, RK_GICD_BASE + GIC_DIST_ENABLE_SET);

	writel_relaxed(0, RK_GICC_BASE + GIC_CPU_CTRL);
	writel_relaxed(0, RK_GICD_BASE + GIC_DIST_CTRL);
}

static void  rkpm_peri_save(u32 core_power_mode, u32 common_power_mode)
{
	rkpm_gic_dist_save(&slp_gic_save[0]);
}

static u32 rv1108_ctrbits;

static void rkpm_save_setting(u32 ctrbits)
{
	rv1108_ctrbits = ctrbits;
	if (((RKPM_CTR_ARMOFF_LPMD |
	      RKPM_CTR_IDLESRAM_MD |
	      RKPM_CTR_ARMLOGDP_LPMD) & ctrbits) == 0) {
		return;
	}
	rkpm_slp_mode_set(ctrbits);
	if (rv1108_common_powermode & BIT(pmu_power_mode_en)) {
		ddr_printch('v');
		if (ctrbits & (RKPM_CTR_ARMOFF_LPMD | RKPM_CTR_ARMLOGDP_LPMD)) {
			sram_code_data_save(rv1108_core_powermode,
					    rv1108_common_powermode);
			ddr_printch('e');
			rkpm_peri_save(rv1108_core_powermode,
				       rv1108_common_powermode);
		}
	}
}

#if 0
#define UART_DLL	(0)	/* Out: Divisor Latch Low */
#define UART_DLM	(1)	/* Out: Divisor Latch High */
#define UART_IER	(1)
#define UART_FCR	(2)
#define UART_LCR	(3)	/* Out: Line Control Register */
#define UART_MCR	(4)

static void slp1108_uartdbg_resume(void)
{
	void __iomem *b_addr = RK_DEBUG_UART_VIRT;
	u32 pclk_id = RV1108_CLKGATE_PCLK_UART2;
	u32 clk_id = (RV1108_CLKGATE_UART0_SRC + 2 * 2);
	u32 gate_reg[2];
	u32 rfl_reg, lsr_reg;

	gate_reg[0] = cru_readl(RV1108_CRU_GATEID_CONS(pclk_id));
	gate_reg[1] = cru_readl(RV1108_CRU_GATEID_CONS(clk_id));

	RV1108_CRU_UNGATING_OPS(pclk_id);
	grf_writel(0x00f00000, 0x00c0);

	do {
		cru_writel(CRU_W_MSK_SETBITS(0x2, 8, 0x3),
			   RV1108_CRU_CLKSELS_CON(16));
		cru_writel(0 | CRU_W_MSK_SETBITS(1, 9, 0x1),
			   RV1108_CRU_SOFTRSTS_CON(2));
		dsb();
		dsb();
		rkpm_udelay(10);
		cru_writel(0 | CRU_W_MSK_SETBITS(0, 9, 0x1),
			   RV1108_CRU_SOFTRSTS_CON(2));

		reg_writel(0x83, b_addr + UART_LCR * 4);

		reg_writel(0xd, b_addr + UART_DLL * 4);
		reg_writel(0x0, b_addr + UART_DLM * 4);

		reg_writel(0x3, b_addr + UART_LCR * 4);

		reg_writel(0x5, b_addr + UART_IER * 4);
		reg_writel(0xc1, b_addr + UART_FCR * 4);

		rfl_reg = readl_relaxed(b_addr + 0x84);
		lsr_reg = readl_relaxed(b_addr + 0x14);
	} while ((rfl_reg & 0x1f) || (lsr_reg & 0xf));

	cru_writel(CRU_W_MSK_SETBITS(0x2, 8, 0x3), RV1108_CRU_CLKSELS_CON(16));

	grf_writel(0x00f000a0, 0x00c0);

	cru_writel(gate_reg[0] | CRU_W_MSK(pclk_id % 16, 0x1),
		   RV1108_CRU_GATEID_CONS(pclk_id));
	cru_writel(gate_reg[1] | CRU_W_MSK(clk_id % 16, 0x1),
		   RV1108_CRU_GATEID_CONS(clk_id));
}
#endif

static void rkpm_gic_dist_resume(u32 *context)
{
	int i = 0, j, irqstart = 0;
	unsigned int gic_irqs;

	gic_irqs = readl_relaxed(RK_GICD_BASE + GIC_DIST_CTR) & 0x1f;
	gic_irqs = (gic_irqs + 1) * 32;
	if (gic_irqs > 1020)
		gic_irqs = 1020;

	irqstart = PM_IRQN_START;

	writel_relaxed(0, RK_GICC_BASE + GIC_CPU_CTRL);
	dsb();
	writel_relaxed(0, RK_GICD_BASE + GIC_DIST_CTRL);
	dsb();
	for (j = irqstart; j < gic_irqs; j += 32) {
		writel_relaxed(0xffffffff, RK_GICD_BASE +
			       GIC_DIST_ENABLE_CLEAR + j * 4 / 32);
		dsb();
	}

	i = 0;

	for (j = irqstart; j < gic_irqs; j += 16) {
		writel_relaxed(context[i++], RK_GICD_BASE +
			       GIC_DIST_CONFIG + j * 4 / 16);
		dsb();
	}
	gic_reg_dump("gic level", j, RK_GICD_BASE + GIC_DIST_CONFIG);

	/* Set all global interrupts to this CPU only. */
	for (j = 0; j < gic_irqs; j += 4) {
		writel_relaxed(context[i++], RK_GICD_BASE +
			       GIC_DIST_TARGET +  (j * 4) / 4);
		dsb();
	}
	gic_reg_dump("gic target", j, RK_GICD_BASE + GIC_DIST_TARGET);

	for (j = 0; j < gic_irqs; j += 4) {
		writel_relaxed(context[i++], RK_GICD_BASE +
			       GIC_DIST_PRI + (j * 4) / 4);
		dsb();
	}
	gic_reg_dump("gic pri", j, RK_GICD_BASE + GIC_DIST_PRI);

	for (j = 0; j < gic_irqs; j += 32) {
		writel_relaxed(context[i++], RK_GICD_BASE +
			       GIC_DIST_IGROUP + (j * 4) / 32);
		dsb();
	}
	gic_reg_dump("gic secu", j, RK_GICD_BASE + 0x80);

	for (j = irqstart; j < gic_irqs; j += 32) {
		i++;
		dsb();
	}

	gic_reg_dump("gic pending", j, RK_GICD_BASE + GIC_DIST_PENDING_SET);

	if (0) {
		for (j = 0; j < gic_irqs; j += 32) {
			writel_relaxed(context[i++], RK_GICD_BASE +
				       GIC_DIST_ENABLE_CLEAR + j * 4 / 32);
			dsb();
		}
		gic_reg_dump("gic disable", j, RK_GICD_BASE +
			     GIC_DIST_ENABLE_CLEAR);
	} else {
		for (j = irqstart; j < gic_irqs; j += 32)
			writel_relaxed(0xffffffff, RK_GICD_BASE +
				       GIC_DIST_ENABLE_CLEAR + j * 4 / 32);
		writel_relaxed(0xffff0000, RK_GICD_BASE +
			       GIC_DIST_ENABLE_CLEAR);
		writel_relaxed(0x0000ffff, RK_GICD_BASE + GIC_DIST_ENABLE_SET);
	}

	/* enable */
	for (j = 0; j < gic_irqs; j += 32) {
		writel_relaxed(context[i++], RK_GICD_BASE +
			       GIC_DIST_ENABLE_SET + (j * 4) / 32);
		dsb();
	}

	gic_reg_dump("gic enable", j, RK_GICD_BASE + GIC_DIST_ENABLE_SET);

	writel_relaxed(context[i++], RK_GICC_BASE + GIC_CPU_PRIMASK);
	writel_relaxed(context[i++], RK_GICD_BASE + GIC_DIST_CTRL);
	writel_relaxed(context[i++], RK_GICC_BASE + GIC_CPU_CTRL);

	gic_reg_dump("gicc", 0x1c, RK_GICC_BASE);
	gic_reg_dump("giccfc", 0, RK_GICC_BASE + 0xfc);
}

static void sram_data_resume(char *boot_save, char *int_save, u32 flag)
{
	char *addr_base;
	u32 sr_size;

	addr_base = (char *)RKPM_BOOTRAM_BASE;
	sr_size = RKPM_BOOTRAM_SIZE;
	/* save boot sram */
	if (boot_save)
		memcpy(addr_base, boot_save, sr_size);

	flush_icache_range((unsigned long)addr_base,
			   (unsigned long)addr_base + sr_size);
}

static inline void sram_code_data_resume(u32 core_power_mode,
					u32 common_power_mode)
{
	sram_data_resume(boot_ram_data,
			 int_ram_data,
			 sleep_resume_data[RKPM_BOOTDATA_DDR_F]);
}

static inline void  rkpm_slp_mode_set_resume(void)
{
#ifdef RV1108_PMU_DEBUG
	/* pmu debug */
	pmu_grf_writel(pmu_grf_gpio0a_iomux | CRU_W_MSK(12, 0x3), 0x0);
	pmu_grf_writel(pmu_grf_gpio0c_iomux |
			CRU_W_MSK(4, 0x3) |
			CRU_W_MSK(6, 0x3) |
			CRU_W_MSK(10, 0x3) |
			CRU_W_MSK(12, 0x3), 0x8);
#endif
	pmu_grf_writel(pmu_grf_soc_con0 | 0xffff0000, RV1108_PMUGRF_SOC_CON0);
	pmu_writel(pmu_sft_con, RV1108_PMU_SFT_CON);
	/* pmu wakeup config */
	pmu_writel(pmu_wakeup_conf0, RV1108_PMU_WAKEUP_CFG0);
	pmu_writel(pmu_wakeup_conf1, RV1108_PMU_WAKEUP_CFG1);
	pmu_writel(pmu_wakeup_conf2, RV1108_PMU_WAKEUP_CFG2);
	/* sleep gpio */
	pmu_grf_writel(gpio_pmic_sleep_mode | CRU_W_MSK(10, 0x3), 0x04);
	/* pmu mode */
	pmu_writel(pmu_pwrmode_core_con, RV1108_PMU_PWRMODE_CORE_CON);
	pmu_writel(pmu_pwrmode_common_con, RV1108_PMU_PWRMODE_COMMON_CON);
	/* fast boot */
	pmu_grf_writel(pmu_grf_fast_boot_addr, RV1108_PMUGRF_FAST_BOOT_ADDR);
	cru_writel(cru_clk_gate12 | 0xffff0000, RV1108_CRU_CLKGATES_CON(12));
	/* pvtm */
	pmu_grf_writel(pmu_grf_dll_con0, RV1108_PMUGRF_DLL_CON0);
	pmu_grf_writel(pmu_grf_dll_con1, RV1108_PMUGRF_DLL_CON1);
	cru_writel(cru_clk_gate8 | 0xffff0000, RV1108_CRU_CLKGATES_CON(8));
}

void fiq_glue_resume(void);

static inline void  rkpm_peri_resume(u32 core_power_mode,
					u32 common_power_mode)
{
	rkpm_gic_dist_resume(&slp_gic_save[0]);
#ifndef CONFIG_ARM_TRUSTZONE
	fiq_glue_resume();
#endif
}

static void rkpm_save_setting_resume(void)
{
	if (rv1108_common_powermode == 0) {
		rkpm_ddr_printascii("powermode==0\n");
		return;
	}

	rkpm_slp_mode_set_resume();
	if (rv1108_common_powermode & BIT(pmu_power_mode_en)) {
		if (rv1108_ctrbits &
		    (RKPM_CTR_ARMOFF_LPMD | RKPM_CTR_ARMLOGDP_LPMD)) {
			rkpm_peri_resume(rv1108_core_powermode,
					 rv1108_common_powermode);
			sram_code_data_resume(rv1108_core_powermode,
					      rv1108_common_powermode);
		}
	}
}

#if 0
static inline void rkpm_peri_resume_first(u32 power_mode)
{
	slp1108_uartdbg_resume();
}

extern void rk_sram_suspend(void);

static void rkpm_slp_setting(void)
{
	rk_usb_power_down();
	rk_sram_suspend();
}

static void rkpm_save_setting_resume_first(void)
{
	rk_usb_power_up();
	rkpm_peri_resume_first(pmu_pwrmode_con);
}
#endif

static u32 clk_ungt_msk[RV1108_CRU_CLKGATES_CON_CNT];

static u32 clk_ungt_msk_1[RV1108_CRU_CLKGATES_CON_CNT];

static u32 clk_ungt_save[RV1108_CRU_CLKGATES_CON_CNT];

static u32 *p_rkpm_clkgt_last_set;
#define CLK_MSK_GATING(msk, con)	cru_writel(((msk) << 16) | 0xffff, con)
#define CLK_MSK_UNGATING(msk, con)	cru_writel(((~(msk)) << 16) | 0xffff, \
						      con)

static void gtclks_suspend(void)
{
	int i;

	for (i = 0; i < RV1108_CRU_CLKGATES_CON_CNT; i++) {
		clk_ungt_save[i] = cru_readl(RV1108_CRU_CLKGATES_CON(i));
#ifdef RV1108_SUSPEND_DEBUG
		rkpm_ddr_printhex(i);
		rkpm_ddr_printascii(" mask: ");
		rkpm_ddr_printhex(clk_ungt_msk[i]);
		rkpm_ddr_printascii(" save: ");
		rkpm_ddr_printhex(cru_readl(RV1108_CRU_CLKGATES_CON(i)));
#endif
		CLK_MSK_UNGATING(clk_ungt_msk[i], RV1108_CRU_CLKGATES_CON(i));

		/* cru_writel(0xffff0000, RV1108_CRU_CLKGATES_CON(i)); */
#ifdef RV1108_SUSPEND_DEBUG
		rkpm_ddr_printascii(" ~msk: ");
		rkpm_ddr_printhex(((~clk_ungt_msk[i]) << 16) | 0xffff);
		rkpm_ddr_printascii(" after gating: ");
		rkpm_ddr_printhex(cru_readl(RV1108_CRU_CLKGATES_CON(i)));
		rkpm_ddr_printch('\n');
#endif
	}
#if 0
	CLK_MSK_UNGATING(0x1400, RV1108_CRU_CLKGATES_CON(9));
	CLK_MSK_UNGATING(0x183e, RV1108_CRU_CLKGATES_CON(14));
	CLK_MSK_UNGATING(0x7b, RV1108_CRU_CLKGATES_CON(6));
	CLK_MSK_UNGATING(0x1e4c, RV1108_CRU_CLKGATES_CON(7));
#endif
}

static void gtclks_resume(void)
{
	int i;

	for (i = 0; i < RV1108_CRU_CLKGATES_CON_CNT; i++)
		cru_writel(clk_ungt_save[i] | 0xffff0000,
			   RV1108_CRU_CLKGATES_CON(i));
#if 0
	rkpm_ddr_printascii("resume gating:\n");
	for (i = 0; i < RV1108_CRU_CLKGATES_CON_CNT; i++) {
		rkpm_ddr_printhex(i);
		rkpm_ddr_printch(':');
		rkpm_ddr_printhex(cru_readl(RV1108_CRU_CLKGATES_CON(i)));
		rkpm_ddr_printch('\n');
	}
#endif
}

static void clks_gating_suspend_init(void)
{
	p_rkpm_clkgt_last_set = &clk_ungt_msk_1[0];

	if (clk_suspend_clkgt_info_get(clk_ungt_msk,
				       p_rkpm_clkgt_last_set,
				       RV1108_CRU_CLKGATES_CON_CNT) ==
				       RV1108_CRU_CLKGATES_CON(0)) {
		rkpm_set_ops_gtclks(gtclks_suspend, gtclks_resume);
		pr_info("%s:init ok\n", __func__);
	}
#ifdef RV1108_SUSPEND_DEBUG
{
	int i;

	pr_info("%s:ungt_mask:\n", __func__);
	for (i = 0; i < RV1108_CRU_CLKGATES_CON_CNT; i++)
		pr_info("%d:0x%x\n", i, clk_ungt_msk[i]);
}
#endif
}

static void pmic_sleep_gpio_get_dts_info(struct device_node *parent)
{
	struct property *prop;

	prop = of_find_property(parent, "rockchip,pmic-suspend_gpios", NULL);
	if (!prop)
		return;
	if (!prop->value)
		return;

	of_property_read_u32_array(parent, "rockchip,pmic-suspend_gpios",
				   &pmic_sleep_gpio, 1);
}

static void reg_pread(void)
{
	u32 n;

	flush_cache_all();
	outer_flush_all();
	local_flush_tlb_all();

	n = readl_relaxed(RK_GPIO_VIRT(0));
	n = readl_relaxed(RK_GPIO_VIRT(1));
	n = readl_relaxed(RK_GPIO_VIRT(1) + 4);

	n = readl_relaxed(RK_GPIO_VIRT(2));
	n = readl_relaxed(RK_GPIO_VIRT(3));

	n = readl_relaxed(RK_DEBUG_UART_VIRT);
	n = readl_relaxed(RK_CPU_AXI_BUS_VIRT);
	n = readl_relaxed(RK_DDR_VIRT);
	n = readl_relaxed(RK_GRF_VIRT);
	n = readl_relaxed(RK_CRU_VIRT);
	n = readl_relaxed(RK_PMU_VIRT);
	n = readl_relaxed(RK_PWM_VIRT);
}

static void __init rv1108_suspend_init(void)
{
	struct device_node *parent;
	u32 pm_ctrbits = 0;

	parent = of_find_node_by_name(NULL, "rockchip_suspend");
	if (IS_ERR_OR_NULL(parent)) {
		PM_ERR("%s dev node err\n", __func__);
		return;
	}

	if (of_property_read_u32_array(parent, "rockchip,ctrbits",
				       &pm_ctrbits, 1)) {
		PM_ERR("%s: read rockchip ctrbits error\n", __func__);
		return;
	}

	/* TODO some suspend code should be done */
	PM_LOG("%s: pm_ctrbits = 0x%x\n", __func__, pm_ctrbits);

	pmic_sleep_gpio_get_dts_info(parent);
	rkpm_set_ctrbits(pm_ctrbits);
	clks_gating_suspend_init();
	rkpm_set_ops_prepare_finish(rkpm_prepare, rkpm_finish);
	rkpm_set_ops_plls(pm_plls_suspend, pm_plls_resume);
	rkpm_set_ops_save_setting(rkpm_save_setting,
				  rkpm_save_setting_resume);
#if 0
	rkpm_set_ops_regs_sleep(rkpm_slp_setting,
				rkpm_save_setting_resume_first);
#endif
	rkpm_set_ops_regs_pread(reg_pread);
	rkpm_set_ops_printch(ddr_printch);
}
