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

#include <dt-bindings/clock/ddr.h>
#include <linux/rockchip/cru.h>
#include <linux/rk_fb.h>
#include <linux/rockchip/iomap.h>
#include <linux/io.h>
#include <linux/wait.h>
#include "rv1108_ddrpctl_phy.h"
#include <linux/interrupt.h>
#include <media/rk-isp11-ioctl.h>
#include <linux/workqueue.h>
#include <asm/arch_timer.h>

/* DDR3 define */
/* mr0 for ddr3 */
#define DDR3_BL8		(0)
#define DDR3_BC4_8		(1)
#define DDR3_BC4		(2)
#define DDR3_CL(n)		(((((n) - 4) & 0x7) << 4) |\
				 ((((n) - 4) & 0x8) >> 1))
#define DDR3_WR(n)		(((n) & 0x7) << 9)
#define DDR3_DLL_RESET		(1 << 8)
#define DDR3_DLL_DERESET	(0 << 8)

/* mr1 for ddr3 */
#define DDR3_DLL_ENABLE		(0)
#define DDR3_DLL_DISABLE	(1)
#define DDR3_MR1_AL(n)		(((n) & 0x7) << 3)

#define DDR3_DS_40		(0)
#define DDR3_DS_34		(1 << 1)
#define DDR3_RTT_NOM_DIS	(0)
#define DDR3_RTT_NOM_60		(1 << 2)
#define DDR3_RTT_NOM_120	(1 << 6)
#define DDR3_RTT_NOM_40		((1 << 2) | (1 << 6))

/* mr2 for ddr3 */
#define DDR3_MR2_CWL(n)		((((n) - 5) & 0x7) << 3)
#define DDR3_RTT_WR_DIS		(0)
#define DDR3_RTT_WR_60		(1 << 9)
#define DDR3_RTT_WR_120		(2 << 9)

#define DDR3_FAST_EXIT			(1)
#define ACTIVE_POWER_DOWN		(1)
/* GRF */
#define GRF_SOC_CON0			(0x0400)
#define GRF_SOC_CON1			(0x0404)
#define GRF_SOC_CON3			(0x040c)
#define GRF_SOC_STATUS			(0x0480)
#define GRF_OS_REG2			(0x0588)
#define PMU_GRF_SOC_CON0		(0X0100)
#define PMU_GRF_DDRPHY_BUFFEREN_CORE_EN	(((0x1 << 2) << 16) | (0x0 << 2))
#define PMU_GRF_DDRPHY_BUFFEREN_CORE_DIS (((0x1 << 2) << 16) | (0x1 << 2))
#define IDLE_REQ_MSCH_EN		((0x1 << (11 + 16)) | (0x1 << 11))
#define IDLE_REQ_MSCH_DIS		((0x1 << (11 + 16)) | (0x0 << 11))
#define IDLE_MSCH_ST			(0x1 << 27)
#define C_ACTIVE_IN_EN			((1 << (16 + 3)) | (1 << 3))
#define C_ACTIVE_IN_DISABLE		((1 << (16 + 3)) | (0 << 3))

/* VOP */
#define VOP_SYS_CTRL			(8)
#define VOP_STANDBY_EN			(22)

#define VOP_INTR_CLEAR			(0x284)
#define VOP_INTR_STATUS			(0x288)

#define VOP_CLEAR_FLAG1			((1 << (4 + 16)) | (1 << 4))
#define VOP_FLAG1_STATUS		(1 << 4)

/* service msch */
#define MSCH_DDR_TIMING			(0x000c)
#define MSCH_READ_LATENCY		(0x0014)
#define MSCH_ACTIVATE			(0x0038)
#define MSCH_DEVTODEV			(0x003c)

#define cru_read32(offset)		readl_relaxed(RK_CRU_VIRT + offset)
#define cru_write32(v, offset)		writel_relaxed(v, RK_CRU_VIRT + offset)
#define grf_read32(offset)		readl_relaxed(RK_GRF_VIRT + offset)
#define grf_write32(v, offset)		writel_relaxed(v, RK_GRF_VIRT + offset)
#define pmu_read32(offset)		readl_relaxed(RK_PMU_VIRT + offset)
#define pmu_write32(v, offset)		writel_relaxed(v, RK_PMU_VIRT + offset)
#define ddrpctl_read32(offset)		readl_relaxed(RK_DDR_VIRT + offset)
#define ddrpctl_write32(v, offset)	writel_relaxed(v, RK_DDR_VIRT + offset)
#define ddrphy_read32(offset)		readl_relaxed((RK_DDR_VIRT + RV1108_DDR_PCTL_SIZE) + offset)
#define ddrphy_write32(v, offset)	writel_relaxed(v, (RK_DDR_VIRT + RV1108_DDR_PCTL_SIZE) + offset)
#define msch_read32(offset)		readl_relaxed(RK_SERVICE_MSCH_VIRT + \
					(offset))
#define msch_write32(v, offset)		writel_relaxed(v, \
					RK_SERVICE_MSCH_VIRT + (offset))

#define READ_CS_INFO()		(((grf_read32(GRF_OS_REG2) >> 11) & 0x1) + 1)
#define READ_COL_INFO()		(9 + ((grf_read32(GRF_OS_REG2) >> 9) & 0x3))
#define READ_BK_INFO()		(3 - ((grf_read32(GRF_OS_REG2) >> 8) & 0x1))
#define READ_CS0_ROW_INFO()	(13 + ((grf_read32(GRF_OS_REG2) >> 6) & 0x3))
#define READ_CS1_ROW_INFO()	(13 + ((grf_read32(GRF_OS_REG2) >> 4) & 0x3))
#define READ_BW_INFO()		(2 >> ((grf_read32(GRF_OS_REG2) & 0xc) >> 2))
#define READ_DIE_BW_INFO()	(2 >> (grf_read32(GRF_OS_REG2) & 0x3))
#define READ_DRAM_TYPE_INFO()	((grf_read32(GRF_OS_REG2) >> 13) & 0x7)

#define SRAM_CODE_OFFSET        rockchip_sram_virt
#define SRAM_SIZE               rockchip_sram_size


/* pll config */
static __sramdata u32 clkFbDiv;
static __sramdata u32 clkPostDiv1;
static __sramdata u32 clkPostDiv2;

#define DDR_PLL_REFDIV  (1)
#define FBDIV(n)        ((0xFFF << 16) | (n & 0xfff))
#define REFDIV(n)       ((0x3F << 16) | (n & 0x3f))
#define POSTDIV1(n)     ((0x7 << (8 + 16)) | ((n & 0x7)<<8))
#define POSTDIV2(n)     ((0x7 << (12 + 16)) | ((n & 0x7)<<12))
#define PLL_LOCK_STATUS	(0x1 << 31)
#define PLL_MODE_SLOW 	((1 << (8 + 16)) | (0 << 8))
#define PLL_MODE_NORM 	((1 << (8 + 16)) | (1 << 8))

typedef enum PLL_ID_Tag {
	APLL = 0,
	DPLL,
	CPLL,
	GPLL,
	PLL_MAX
} PLL_ID;

typedef enum dram_type_tag {
	LPDDR = 0,
	DDR,
	DDR2,
	DDR3,
	LPDDR2S2,
	LPDDR2,
	LPDDR3,
	DRAM_MAX
} DRAM_TYPE;

typedef struct PCTRL_TIMING_Tag {
	u32 ddrFreq;
	/* Memory Timing Registers */
	u32 togcnt1u;		/* Toggle Counter 1U Register */
	u32 tinit;		/* t_init Timing Register */
	u32 trsth;		/* Reset High Time Register */
	u32 togcnt100n;		/* Toggle Counter 100N Register */
	u32 trefi;		/* t_refi Timing Register */
	u32 tmrd;		/* t_mrd Timing Register */
	u32 trfc;		/* t_rfc Timing Register */
	u32 trp;		/* t_rp Timing Register */
	u32 trtw;		/* t_rtw Timing Register */
	u32 tal;		/* AL Latency Register */
	u32 tcl;		/* CL Timing Register */
	u32 tcwl;		/* CWL Register */
	u32 tras;		/* t_ras Timing Register */
	u32 trc;		/* t_rc Timing Register */
	u32 trcd;		/* t_rcd Timing Register */
	u32 trrd;		/* t_rrd Timing Register */
	u32 trtp;		/* t_rtp Timing Register */
	u32 twr;		/* t_wr Timing Register */
	u32 twtr;		/* t_wtr Timing Register */
	u32 texsr;		/* t_exsr Timing Register */
	u32 txp;		/* t_xp Timing Register */
	u32 txpdll;		/* t_xpdll Timing Register */
	u32 tzqcs;		/* t_zqcs Timing Register */
	u32 tzqcsi;		/* t_zqcsi Timing Register */
	u32 tdqs;		/* t_dqs Timing Register */
	u32 tcksre;		/* t_cksre Timing Register */
	u32 tcksrx;		/* t_cksrx Timing Register */
	u32 tcke;		/* t_cke Timing Register */
	u32 tmod;		/* t_mod Timing Register */
	u32 trstl;		/* Reset Low Timing Register */
	u32 tzqcl;		/* t_zqcl Timing Register */
	u32 tmrr;		/* t_mrr Timing Register */
	u32 tckesr;		/* t_ckesr Timing Register */
	u32 tdpd;		/* t_dpd Timing Register */
	u32 trefi_mem_ddr3;
} PCTL_TIMING_T;

typedef union NOC_TIMING_Tag {
	u32 d32;
	struct {
		unsigned ActToAct:6;
		unsigned RdToMiss:6;
		unsigned WrToMiss:6;
		unsigned BurstLen:3;
		unsigned RdToWr:5;
		unsigned WrToRd:5;
		unsigned BwRatio:1;
	} b;
} NOC_TIMING_T;

typedef union NOC_ACTIVATE_Tag {
	u32 d32;
	struct {
		unsigned Rrd : 4;	/* bit[0:3] */
		unsigned Faw : 6;	/* bit[4:9] */
		unsigned Fawbank : 1;	/* bit 10 */
		unsigned reserved : 21;
	} b;
} NOC_ACTIVATE_T;

struct ddr_timing {
	u32 dram_spd_bin;
	u32 sr_idle;
	u32 pd_idle;
	u32 dram_dll_dis_freq;
	u32 phy_dll_dis_freq;
	u32 dram_odt_dis_freq;
	u32 phy_odt_dis_freq;
	u32 ddr3_drv;
	u32 ddr3_odt;
	u32 lpddr3_drv;
	u32 lpddr3_odt;
	u32 lpddr2_drv;
	u32 phy_ddr3_clk_drv;
	u32 phy_ddr3_cmd_drv;
	u32 phy_ddr3_dqs_drv;
	u32 phy_ddr3_odt;
	u32 phy_lp23_clk_drv;
	u32 phy_lp23_cmd_drv;
	u32 phy_lp23_dqs_drv;
	u32 phy_lp3_odt;
};

typedef struct BACKUP_REG_Tag {
	PCTL_TIMING_T pctl_timing;
	NOC_TIMING_T noc_timing;
	u32 ddrMR[4];
	u32 ddrMR11;
	u32 mem_type;
	u32 ddr_speed_bin;
	u32 ddr_capability_per_die;
	u32 ddr_dll_status;
	u32 ddr_sr_idle;
	struct ddr_timing dram_timing;
	u32 readlatency;
	NOC_ACTIVATE_T noc_activate;
	u32 devtodev;
} BACKUP_REG_T;

BACKUP_REG_T DEFINE_PIE_DATA(ddr_reg);
static BACKUP_REG_T *p_ddr_reg;

u32 DEFINE_PIE_DATA(ddr_freq);
static u32 *p_ddr_freq;


static __sramdata u32 clk_gate[RV1108_CRU_CLKGATES_CON_CNT];

static int __ddr_change_freq(u32 nmhz);

struct master_request {
	int id;
	unsigned long long start;     /* us*/
	unsigned long long end;     /* us*/
	unsigned long timeout;   /* us*/
	unsigned long frame_time;  /* us*/
};

static int ddr_freq_current = 600;	/* MHz*/
static int ddr_freq_new = 600;		/* MHz*/
static int ddr_freq_period = 300;	/* us*/
static struct master_request gmr[4];
static struct tasklet_struct ddr_freq_ts[4];
unsigned long long trace_time[16];
static wait_queue_head_t wq;
#ifdef CONFIG_VIDEO_RK_CIF_ISP11
static unsigned int numerator = 1;
static unsigned int denominator = 30;
static struct work_struct ws;
#endif

#define DDR3_CL_CWL(d1, d2, d3, d4, d5, d6, d7) \
	{((d1) << 4) | 5, ((d2) << 4) | 5, ((d3) << 4) | 6, ((d4) << 4) | 7, \
	 ((d5) << 4) | 8, ((d6) << 4) | 9, ((d7) << 4) | 10}
#define DDR3_TRC_TFAW(trc, tfaw)	(((trc) << 8) | tfaw)

static const uint8_t ddr3_cl_cwl[22][7] = {
	/*
	 * speed 0~330 331~400 401~533 534~666 667~800 801~933 934~1066
	 * tCK	>3 2.5~3 1.875~2.5 1.5~1.875 1.25~1.5 1.07~1.25 0.938~1.07
	 * cl<<4, cwl  cl<<4, cwl  cl<<4, cwl
	 */
	DDR3_CL_CWL(5, 5, 0, 0, 0, 0, 0),	/* DDR3_800D (5-5-5) */
	DDR3_CL_CWL(5, 6, 0, 0, 0, 0, 0),	/* DDR3_800E (6-6-6) */
	DDR3_CL_CWL(5, 5, 6, 0, 0, 0, 0),	/* DDR3_1066E (6-6-6) */
	DDR3_CL_CWL(5, 6, 7, 0, 0, 0, 0),	/* DDR3_1066F (7-7-7) */
	DDR3_CL_CWL(5, 6, 8, 0, 0, 0, 0),	/* DDR3_1066G (8-8-8) */
	DDR3_CL_CWL(5, 5, 6, 7, 0, 0, 0),	/* DDR3_1333F (7-7-7) */
	DDR3_CL_CWL(5, 5, 7, 8, 0, 0, 0),	/* DDR3_1333G (8-8-8) */
	DDR3_CL_CWL(5, 6, 8, 9, 0, 0, 0),	/* DDR3_1333H (9-9-9) */
	DDR3_CL_CWL(5, 6, 8, 10, 0, 0, 0),	/* DDR3_1333J (10-10-10) */
	DDR3_CL_CWL(5, 5, 6, 7, 8, 0, 0),	/* DDR3_1600G (8-8-8) */
	DDR3_CL_CWL(5, 5, 6, 8, 9, 0, 0),	/* DDR3_1600H (9-9-9) */
	DDR3_CL_CWL(5, 5, 7, 9, 10, 0, 0),	/* DDR3_1600J (10-10-10) */
	DDR3_CL_CWL(5, 6, 8, 10, 11, 0, 0),	/* DDR3_1600K (11-11-11) */
	DDR3_CL_CWL(5, 5, 6, 8, 9, 11, 0),	/* DDR3_1866J (10-10-10) */
	DDR3_CL_CWL(5, 5, 7, 8, 10, 11, 0),	/* DDR3_1866K (11-11-11) */
	DDR3_CL_CWL(6, 6, 7, 9, 11, 12, 0),	/* DDR3_1866L (12-12-12) */
	DDR3_CL_CWL(6, 6, 8, 10, 11, 13, 0),	/* DDR3_1866M (13-13-13) */
	DDR3_CL_CWL(5, 5, 6, 7, 9, 10, 11),	/* DDR3_2133K (11-11-11) */
	DDR3_CL_CWL(5, 5, 6, 8, 9, 11, 12),	/* DDR3_2133L (12-12-12) */
	DDR3_CL_CWL(5, 5, 7, 9, 10, 12, 13),	/* DDR3_2133M (13-13-13) */
	DDR3_CL_CWL(6, 6, 7, 9, 11, 13, 14),	/* DDR3_2133N (14-14-14) */
	DDR3_CL_CWL(6, 6, 8, 10, 11, 13, 14),	/* DDR3_DEFAULT */
};

static const uint16_t ddr3_tRC_tFAW[22] = {
	/* tRC      tFAW */
	DDR3_TRC_TFAW(50, 50),	/* DDR3_800D (5-5-5) */
	DDR3_TRC_TFAW(53, 50),	/* DDR3_800E (6-6-6) */
	DDR3_TRC_TFAW(49, 50),	/* DDR3_1066E (6-6-6) */
	DDR3_TRC_TFAW(51, 50),	/* DDR3_1066F (7-7-7) */
	DDR3_TRC_TFAW(53, 50),	/* DDR3_1066G (8-8-8) */
	DDR3_TRC_TFAW(47, 45),	/* DDR3_1333F (7-7-7) */
	DDR3_TRC_TFAW(48, 45),	/* DDR3_1333G (8-8-8) */
	DDR3_TRC_TFAW(50, 45),	/* DDR3_1333H (9-9-9) */
	DDR3_TRC_TFAW(51, 45),	/* DDR3_1333J (10-10-10) */
	DDR3_TRC_TFAW(45, 40),	/* DDR3_1600G (8-8-8) */
	DDR3_TRC_TFAW(47, 40),	/* DDR3_1600H (9-9-9) */
	DDR3_TRC_TFAW(48, 40),	/* DDR3_1600J (10-10-10) */
	DDR3_TRC_TFAW(49, 40),	/* DDR3_1600K (11-11-11) */
	DDR3_TRC_TFAW(45, 35),	/* DDR3_1866J (10-10-10) */
	DDR3_TRC_TFAW(46, 35),	/* DDR3_1866K (11-11-11) */
	DDR3_TRC_TFAW(47, 35),	/* DDR3_1866L (12-12-12) */
	DDR3_TRC_TFAW(48, 35),	/* DDR3_1866M (13-13-13) */
	DDR3_TRC_TFAW(44, 35),	/* DDR3_2133K (11-11-11) */
	DDR3_TRC_TFAW(45, 35),	/* DDR3_2133L (12-12-12) */
	DDR3_TRC_TFAW(46, 35),	/* DDR3_2133M (13-13-13) */
	DDR3_TRC_TFAW(47, 35),	/* DDR3_2133N (14-14-14) */
	DDR3_TRC_TFAW(53, 50),	/* DDR3_DEFAULT */
};

/****************************************************************************
*Internal sram us delay function
*Cpu highest frequency is 1.6 GHz
*1 cycle = 1/1.6 ns
*1 us = 1000 ns = 1000 * 1.6 cycles = 1600 cycles
******************************************************************************/
volatile u32 DEFINE_PIE_DATA(loops_per_us);
#define LPJ_100MHZ  999456UL

static void __sramfunc ddr_delayus(u32 us)
{
	u32 start, end;

	us *= 24;

	start = arch_counter_get_cntpct();
	do {
		end = arch_counter_get_cntpct();
		if ((end - start) > us)
			break;
	} while (1);
}

static __sramfunc void ddr_copy(u32 *pDest, u32 *pSrc,
				u32 words)
{
	u32 i;

	for (i = 0; i < words; i++)
		pDest[i] = pSrc[i];
}

static __sramfunc void ddr_move_to_Lowpower_state(void)
{
	volatile u32 value;

	grf_write32(C_ACTIVE_IN_EN, GRF_SOC_CON0);

	while (1) {
		value = GET_CTL_STAT(ddrpctl_read32(DDR_PCTL_STAT));
		if (value == LOW_POWER)
			break;
		switch (value) {
		case INIT_MEM:
			ddrpctl_write32(CFG_STATE, DDR_PCTL_SCTL);
			dsb();
			while (GET_CTL_STAT(ddrpctl_read32(DDR_PCTL_STAT)) !=
			       CONFIG)
				continue;
		case CONFIG:
			ddrpctl_write32(GO_STATE, DDR_PCTL_SCTL);
			dsb();
			while (GET_CTL_STAT(ddrpctl_read32(DDR_PCTL_STAT)) !=
			       ACCESS)
				continue;
		case ACCESS:
			ddrpctl_write32(SLEEP_STATE, DDR_PCTL_SCTL);
			dsb();
			while (GET_CTL_STAT(ddrpctl_read32(DDR_PCTL_STAT)) !=
			       LOW_POWER)
				continue;
			break;
		}
	}
}

/* set sr_idle, pd_idle and dfi_lp */
static __sramfunc void ddr_set_dfi_lp(void)
{
	u32 value;

	/*
	 * when ddr3 mode ddr input clk can't stop during power-down
	 * and lpddr2, lpddr3 mode can stop clk during power-down.
	 */
	if (DATA(ddr_reg).mem_type == DDR3)
		ddrpctl_write32(0x51100, DDR_PCTL_DFILPCFG0);
	else
		ddrpctl_write32(0x51111, DDR_PCTL_DFILPCFG0);

	value = ddrpctl_read32(DDR_PCTL_MCFG1);
	value = (value & 0xffffff00) | DATA(ddr_reg).ddr_sr_idle | (1 << 31);
	ddrpctl_write32(value, DDR_PCTL_MCFG1);

	value = ddrpctl_read32(DDR_PCTL_MCFG);
	value = (value & 0xffff00ff) | (DATA(ddr_reg).dram_timing.pd_idle << 8);
	value = (value & 0xfffcffff) | ((ACTIVE_POWER_DOWN << 16) |
					(DDR3_FAST_EXIT << 17));
	ddrpctl_write32(value, DDR_PCTL_MCFG);
}

static __sramfunc void ddr_move_to_Access_state(void)
{
	volatile u32 value;

	while (1) {
		value = GET_CTL_STAT(ddrpctl_read32(DDR_PCTL_STAT));
		if ((value == ACCESS) ||
		    ((GET_LP_TRIG(ddrpctl_read32(DDR_PCTL_STAT)) == 1) &&
			(value == LOW_POWER))) {
			break;
		}
		switch (value) {
		case LOW_POWER:
			ddrpctl_write32(WAKEUP_STATE, DDR_PCTL_SCTL);
			dsb();
			while (GET_CTL_STAT(ddrpctl_read32(DDR_PCTL_STAT)) !=
			       ACCESS)
				continue;
			break;
		case INIT_MEM:
			ddrpctl_write32(CFG_STATE, DDR_PCTL_SCTL);
			dsb();
			while (GET_CTL_STAT(ddrpctl_read32(DDR_PCTL_STAT)) !=
			       CONFIG)
				continue;
		case CONFIG:
			ddrpctl_write32(GO_STATE, DDR_PCTL_SCTL);
			dsb();
			while (!
			       ((GET_CTL_STAT(ddrpctl_read32(DDR_PCTL_STAT)) ==
				 ACCESS) ||
				((GET_LP_TRIG(ddrpctl_read32(DDR_PCTL_STAT)) ==
				  1) &&
				 (GET_CTL_STAT(ddrpctl_read32(DDR_PCTL_STAT))
				     == LOW_POWER))))
				continue;
			break;
		}
	}
	grf_write32(C_ACTIVE_IN_DISABLE, GRF_SOC_CON0);
}

static __sramfunc void ddr_move_to_Config_state(void)
{
	volatile u32 value;

	grf_write32(C_ACTIVE_IN_EN, GRF_SOC_CON0);

	/* hw_wakeup :disable auto sr */
	while (1) {
		value = GET_CTL_STAT(ddrpctl_read32(DDR_PCTL_STAT));
		if (value == CONFIG)
			break;
		switch (value) {
		case LOW_POWER:
			ddrpctl_write32(WAKEUP_STATE, DDR_PCTL_SCTL);
			dsb();
		case ACCESS:
		case INIT_MEM:
			ddrpctl_write32(CFG_STATE, DDR_PCTL_SCTL);
			dsb();
			break;
		}
	}
}

static __sramfunc void ddr_send_command(u32 rank, u32 cmd,
					     u32 arg)
{
	u32 value;

	while (ddrpctl_read32(DDR_PCTL_MCMD) & start_cmd)
		continue;
	value = (start_cmd | (rank << 20) | arg | cmd);
	ddrpctl_write32(value, DDR_PCTL_MCMD);
	dsb();
}

static __sramfunc u32 ddr_data_training(void)
{
	u32 value, dram_bw, phyreg2_val;

	value = ddrpctl_read32(DDR_PCTL_TREFI);
	ddrpctl_write32(0x1 << 31, DDR_PCTL_TREFI);
	dram_bw = GET_PHY_BW(ddrphy_read32(PHYREG00));
	phyreg2_val = ddrphy_read32(PHYREG02);
	phyreg2_val &= 0xCC;
	ddrphy_write32(0x21 | phyreg2_val, PHYREG02);
	/* wait echo byte DTDONE */
	ddr_delayus(1);
	/* stop DTT */
	while ((ddrphy_read32(PHYREGFF) & 0xf) != dram_bw)
		continue;

	ddrphy_write32(0x20 | phyreg2_val, PHYREG02);
	/* send some auto refresh to complement the lost while DTT */
	ddr_send_command(3, PREA_cmd, 0);
	ddr_send_command(3, REF_cmd, 0);
	ddr_send_command(3, REF_cmd, 0);
	ddr_send_command(3, REF_cmd, 0);
	ddr_send_command(3, REF_cmd, 0);

	ddrpctl_write32(value | (0x1 << 31), DDR_PCTL_TREFI);
	return 0;
}

static __sramfunc void ddr_set_dll_bypass(u32 freq)
{
	u32 phase;

	if (freq < 680)
		phase = 2;
	else
		phase = 1;

	ddrphy_write32(phase, PHYREG28);
	ddrphy_write32(phase, PHYREG38);

	if (freq <= DATA(ddr_reg).dram_timing.phy_dll_dis_freq)
		ddrphy_write32((ddrphy_read32(PHYREGDLL) | 0x1F), PHYREGDLL);
	else
		ddrphy_write32((ddrphy_read32(PHYREGDLL) & (~0x1F)), PHYREGDLL);

	dsb();
}

static noinline u32 ddr_get_pll_freq(PLL_ID pll_id)
{
	u32 ret = 0, val = 0;
	u32 fbdiv, postdiv1, postdiv2, refdiv;

	fbdiv = cru_read32(RV1108_PLL_CONS(pll_id, 0)) & 0xfff;
	val = cru_read32(RV1108_PLL_CONS(pll_id, 1));
	postdiv1 = (val >> 8) & 0x7;
	postdiv2 = (val >> 12) & 0x7;
	refdiv = val & 0x3f;
	val = (cru_read32(RV1108_PLL_CONS(pll_id, 3)) >> 8) & 1;

	if (val == 1)
		ret = 24 * fbdiv / (refdiv * postdiv1 * postdiv2);
	else
		ret = 24;

	return ret;
}

static __sramfunc u32 ddr_set_pll(u32 nmhz, u32 set)
{
	u32 ret, delay;

	if (!set) {
		if (nmhz <= 150)
			clkPostDiv1 = 8;
		else if (nmhz <= 200)
			clkPostDiv1 = 6;
		else if (nmhz <= 300)
			clkPostDiv1 = 4;
		else if (nmhz <= 800)
			clkPostDiv1 = 1;
		else
			clkPostDiv1 = 1;
		clkPostDiv2 = 1;
		clkFbDiv = (nmhz * 2 * DDR_PLL_REFDIV * clkPostDiv1 * clkPostDiv2) / 24;
	} else {
		cru_write32(PLL_MODE_SLOW, RV1108_PLL_CONS(DPLL, 3));
		cru_write32(FBDIV(clkFbDiv), RV1108_PLL_CONS(DPLL, 0));
		cru_write32(REFDIV(DDR_PLL_REFDIV) | POSTDIV1(clkPostDiv1) |
				POSTDIV2(clkPostDiv2), RV1108_PLL_CONS(DPLL, 1));
		ddr_delayus(1);
		delay = 1000;
		while (delay > 0) {
			if (cru_read32(RV1108_PLL_CONS(DPLL, 2)) &
			    PLL_LOCK_STATUS)
				break;
			ddr_delayus(1);
			delay--;
		}
		cru_write32(PLL_MODE_NORM, RV1108_PLL_CONS(DPLL, 3));
	}

	ret = (24 * clkFbDiv) / (2 * DDR_PLL_REFDIV * clkPostDiv1 *
				 clkPostDiv2);
	return ret;
}

u32 PIE_FUNC(ddr_set_pll)(u32 nMHz, u32 set)
{
	return ddr_set_pll(nMHz, set);
}

EXPORT_PIE_SYMBOL(FUNC(ddr_set_pll));
static u32 (*p_ddr_set_pll)(u32 nMHz, u32 set);

static inline u32 ddr_get_dram_freq(void)
{
	u32 ret;

	ret = ddrpctl_read32(DDR_PCTL_TOGCNT1U) * 2;

	return ret;
}

static u32 ddr_get_parameter_ddr3(u32 nmhz)
{
	u32 tmp;
	u32 ret = 0;
	u32 al;
	u32 bl;
	u32 cl;
	u32 cwl;

	PCTL_TIMING_T *p_pctl_timing = &p_ddr_reg->pctl_timing;
	NOC_TIMING_T *p_noc_timing = &p_ddr_reg->noc_timing;
	NOC_ACTIVATE_T  *p_noc_activate = &p_ddr_reg->noc_activate;
	u32 *p_readlatency = &p_ddr_reg->readlatency;
	u32 *p_devtodev = &p_ddr_reg->devtodev;

	p_pctl_timing->togcnt1u = nmhz / 2;
	p_pctl_timing->togcnt100n = nmhz / 20;
	p_pctl_timing->tinit = 200;
	if (p_ddr_reg->ddr_speed_bin > DDR3_DEFAULT) {
		ret = -1;
		goto OUT;
	}
#define DDR3_tREFI_7_8_us	(78)
#define DDR3_tMRD		(4)
#define DDR3_tRFC_512Mb		(90)
#define DDR3_tRFC_1Gb		(110)
#define DDR3_tRFC_2Gb		(160)
#define DDR3_tRFC_4Gb		(260)
#define DDR3_tRFC_8Gb		(350)
#define DDR3_tRTW		(2)	/* register min valid value */
#define DDR3_tRAS		(37)
#define DDR3_tRRD		(7)
#define DDR3_tRTP		(7)
#define DDR3_tWR		(15)
#define DDR3_tWTR		(7)
#define DDR3_tXP		(7)
#define DDR3_tXPDLL		(24)
#define DDR3_tZQCS		(80)
#define DDR3_tZQCSI		(0)
#define DDR3_tDQS		(1)
#define DDR3_tCKSRE		(10)
#define DDR3_tCKE_400MHz	(7)
#define DDR3_tCKE_533MHz	(6)
#define DDR3_tMOD		(15)
#define DDR3_tRSTL		(100)
#define DDR3_tZQCL		(320)
#define DDR3_tDLLK		(512)
	p_pctl_timing->trsth = 500;
	al = 0;
	bl = 8;
	if (nmhz <= 330)
		tmp = 0;
	else if (nmhz <= 400)
		tmp = 1;
	else if (nmhz <= 533)
		tmp = 2;
	else if (nmhz <= 666)
		tmp = 3;
	else if (nmhz <= 800)
		tmp = 4;
	else if (nmhz <= 933)
		tmp = 5;
	else
		tmp = 6;

	if (nmhz <= p_ddr_reg->dram_timing.dram_dll_dis_freq) {
		/* when dll bypss cl = cwl = 6 */
		cl = 6;
		cwl = 6;
	} else {
		cl = ddr3_cl_cwl[p_ddr_reg->ddr_speed_bin][tmp] >> 4;
		cwl = ddr3_cl_cwl[p_ddr_reg->ddr_speed_bin][tmp] & 0xf;
	}

	if (cl == 0)
		ret = -4;

	if (nmhz <= p_ddr_reg->dram_timing.dram_odt_dis_freq)
		p_ddr_reg->ddrMR[1] =
		    p_ddr_reg->dram_timing.ddr3_drv | DDR3_RTT_NOM_DIS;
	else
		p_ddr_reg->ddrMR[1] = p_ddr_reg->dram_timing.ddr3_drv |
		    p_ddr_reg->dram_timing.ddr3_odt;
	p_ddr_reg->ddrMR[2] = DDR3_MR2_CWL(cwl);
	p_ddr_reg->ddrMR[3] = 0;
	/**************************************************
	* PCTL Timing
	**************************************************/
	/*
	 * tREFI, average periodic refresh interval, 7.8us
	 */
	p_pctl_timing->trefi = (1 << 31) | DDR3_tREFI_7_8_us;
	p_pctl_timing->trefi_mem_ddr3 =
	    DDR3_tREFI_7_8_us * p_pctl_timing->togcnt100n;
	/*
	 * tMRD, 4 tCK
	 */
	p_pctl_timing->tmrd = DDR3_tMRD & 0x7;
	/*
	 * tRFC, 90ns(512Mb),110ns(1Gb),160ns(2Gb),300ns(4Gb),350ns(8Gb)
	 */
	if (p_ddr_reg->ddr_capability_per_die <= 0x4000000)  /*512Mb 90ns */
		tmp = DDR3_tRFC_512Mb;
	else if (p_ddr_reg->ddr_capability_per_die <= 0x8000000)/*1Gb 110ns */
		tmp = DDR3_tRFC_1Gb;
	else if (p_ddr_reg->ddr_capability_per_die <= 0x10000000)/*2Gb 160ns */
		tmp = DDR3_tRFC_2Gb;
	else if (p_ddr_reg->ddr_capability_per_die <= 0x20000000)/*4Gb 300ns */
		tmp = DDR3_tRFC_4Gb;
	else							/*8Gb  350ns */
		tmp = DDR3_tRFC_8Gb;
	p_pctl_timing->trfc = (tmp * nmhz + 999) / 1000;
	/*
	 * tXSR, =tDLLK=512 tCK
	 */
	p_pctl_timing->texsr = DDR3_tDLLK;
	/*
	 * tRP=CL
	 */
	p_pctl_timing->trp = cl;
	/*
	 * WrToMiss=WL*tCK + tWR + tRP + tRCD
	 */
	p_noc_timing->b.WrToMiss =
	    (((cwl + ((DDR3_tWR * nmhz + 999) / 1000) + cl + cl) / 2) & 0x3F);
	/*
	 * tRAS, 37.5ns(400MHz)     37.5ns(533MHz)
	 */
	p_pctl_timing->tras =
	    (((DDR3_tRAS * nmhz + (nmhz >> 1) + 999) / 1000) & 0x3F);

	/*
	 * tRC=tRAS+tRP
	 */
	p_pctl_timing->trc = p_pctl_timing->tras + p_pctl_timing->trp;

	p_noc_timing->b.ActToAct =
	    (((((ddr3_tRC_tFAW[p_ddr_reg->ddr_speed_bin] >> 8) * nmhz +
	       999) / 1000) / 2) & 0x3F);

	p_pctl_timing->trtw = (cl + 2 - cwl);	/*DDR3_tRTW */
	p_noc_timing->b.RdToWr = (((cl + 2 - cwl) / 2) & 0x1F);
	p_pctl_timing->tal = al;
	p_pctl_timing->tcl = cl;
	p_pctl_timing->tcwl = cwl;
	/*
	 * tRAS, 37.5ns(400MHz)     37.5ns(533MHz)
	 */
	p_pctl_timing->tras =
	    (((DDR3_tRAS * nmhz + (nmhz >> 1) + 999) / 1000) & 0x3F);
	/*
	 * tRCD=CL
	 */
	p_pctl_timing->trcd = cl;
	/*
	 * tRRD = max(4nCK, 7.5ns), DDR3-1066(1K), DDR3-1333(2K), DDR3-1600(2K)
	 *        max(4nCK, 10ns), DDR3-800(1K,2K), DDR3-1066(2K)
	 *        max(4nCK, 6ns), DDR3-1333(1K), DDR3-1600(1K)
	 *
	 */
	tmp = ((DDR3_tRRD * nmhz + (nmhz >> 1) + 999) / 1000);
	if (tmp < 4)
		tmp = 4;
	p_pctl_timing->trrd = (tmp & 0xF);
	p_noc_activate->b.Rrd = tmp / 2;
	/*
	 * tRTP, max(4 tCK,7.5ns)
	 */
	tmp = ((DDR3_tRTP * nmhz + (nmhz >> 1) + 999) / 1000);
	if (tmp < 4)
		tmp = 4;
	p_pctl_timing->trtp = tmp & 0xF;
	/*
	 * RdToMiss=tRTP+tRP + tRCD - (BL/2 * tCK)
	 */
	p_noc_timing->b.RdToMiss = (((tmp + cl + cl - (bl >> 1)) / 2) & 0x3F);
	/*
	 * tWR, 15ns
	 */
	tmp = ((DDR3_tWR * nmhz + 999) / 1000);
	p_pctl_timing->twr = tmp & 0x1F;
	if (tmp < 9) {
		tmp = tmp - 4;
	} else {
		tmp += (tmp & 0x1) ? 1 : 0;
		tmp = tmp >> 1;
	}
	p_ddr_reg->ddrMR[0] = DDR3_BL8 | DDR3_CL(cl) | DDR3_WR(tmp) |
			      (DDR3_FAST_EXIT<<12);

	/*
	 * tWTR, max(4 tCK,7.5ns)
	 */
	tmp = ((DDR3_tWTR * nmhz + (nmhz >> 1) + 999) / 1000);
	if (tmp < 4)
		tmp = 4;
	p_pctl_timing->twtr = tmp & 0xF;
	p_noc_timing->b.WrToRd = (((tmp + cwl) / 2) & 0x1F);
	/*
	 * tXP, max(3 tCK, 7.5ns)(<933MHz)
	 */
	tmp = ((DDR3_tXP * nmhz + (nmhz >> 1) + 999) / 1000);
	if (tmp < 3)
		tmp = 3;
	p_pctl_timing->txp = tmp & 0x7;
	/*
	 * tXPDLL, max(10 tCK,24ns)
	 */
	tmp = ((DDR3_tXPDLL * nmhz + 999) / 1000);
	if (tmp < 10)
		tmp = 10;
	p_pctl_timing->txpdll = tmp & 0x3F;
	/*
	 * tZQCS, max(64 tCK, 80ns)
	 */
	tmp = ((DDR3_tZQCS * nmhz + 999) / 1000);
	if (tmp < 64)
		tmp = 64;
	p_pctl_timing->tzqcs = tmp & 0x7F;
	/*
	 * tZQCSI,
	 */
	p_pctl_timing->tzqcsi = DDR3_tZQCSI;
	/*
	 * tDQS,
	 */
	p_pctl_timing->tdqs = DDR3_tDQS;
	/*
	 * tCKSRE, max(5 tCK, 10ns)
	 */
	tmp = ((DDR3_tCKSRE * nmhz + 999) / 1000);
	if (tmp < 5)
		tmp = 5;
	p_pctl_timing->tcksre = tmp & 0x1F;
	/*
	 * tCKSRX, max(5 tCK, 10ns)
	 */
	p_pctl_timing->tcksrx = tmp & 0x1F;
	/*
	 * tCKE, max(3 tCK,7.5ns)(400MHz) max(3 tCK,5.625ns)(533MHz)
	 */
	if (nmhz >= 533)
		tmp = ((DDR3_tCKE_533MHz * nmhz + 999) / 1000);
	else
		tmp = ((DDR3_tCKE_400MHz * nmhz + (nmhz >> 1) + 999) / 1000);
	if (tmp < 3)
		tmp = 3;
	p_pctl_timing->tcke = tmp & 0x7;
	/*
	 * tCKESR, =tCKE + 1tCK
	 */
	p_pctl_timing->tckesr = (tmp + 1) & 0xF;
	/*
	 * tMOD, max(12 tCK,15ns)
	 */
	tmp = ((DDR3_tMOD * nmhz + 999) / 1000);
	if (tmp < 12)
		tmp = 12;
	p_pctl_timing->tmod = tmp & 0x1F;
	/*
	 * tRSTL, 100ns
	 */
	p_pctl_timing->trstl = ((DDR3_tRSTL * nmhz + 999) / 1000) & 0x7F;
	/*
	 * tZQCL, max(256 tCK, 320ns)
	 */
	tmp = ((DDR3_tZQCL * nmhz + 999) / 1000);
	if (tmp < 256)
		tmp = 256;
	p_pctl_timing->tzqcl = tmp & 0x3FF;
	/*
	 * tMRR, 0 tCK
	 */
	p_pctl_timing->tmrr = 0;
	/*
	 * tDPD, 0
	 */
	p_pctl_timing->tdpd = ddrpctl_read32(DDR_PCTL_TDPD);

	/**************************************************
	*NOC Timing
	**************************************************/
	/*
	* tFAW,40ns(400MHz 1KB page) 37.5ns(533MHz 1KB page)
	* 50ns(400MHz 2KB page)   50ns(533MHz 2KB page)
	*/
	tmp = (((ddr3_tRC_tFAW[0] & 0x0ff) * nmhz + 999) / 1000);
	p_noc_activate->b.Fawbank = 1;
	p_noc_activate->b.Faw = tmp / 2;
	p_noc_timing->b.BurstLen = (((bl >> 1) / 2) & 0x7);
	/*
	* readlatency = trp + trcd + cl + bl + few cycles
	*/
	*p_readlatency = (cl + cl + cl + (bl / 2) + 26) / 2;
	*p_devtodev = (1 << 4) | (1 << 2) | 1;
OUT:
	return ret;
}

static __sramfunc u32 ddr_update_timing(void)
{
	u32 ret = 0, val;

	PCTL_TIMING_T *p_pctl_timing = &(DATA(ddr_reg).pctl_timing);

	ddr_copy((u32 *)(RK_DDR_VIRT + DDR_PCTL_TOGCNT1U),
		 (u32 *)&(p_pctl_timing->togcnt1u), 35);
	ddrphy_write32((p_pctl_timing->tcl << 4) | (p_pctl_timing->tal),
		       PHYREG0b);
	ddrphy_write32(p_pctl_timing->tcwl, PHYREG0c);
	/* Update PCTL BL */
	if (DATA(ddr_reg).mem_type == DDR3) {
		val = ddrpctl_read32(DDR_PCTL_MCFG);
		val = (val & (~(0x1 | (0x3 << 18) | (0x1 << 17)
				| (0x1 << 16)))) | ddr2_ddr3_bl_8 | tfaw_cfg(5)
		    | pd_exit_slow | pd_type(1);
		ddrpctl_write32(val, DDR_PCTL_MCFG);
		val = (p_pctl_timing->tcl - 1) / 2 - 1;
		ddrpctl_write32(val, DDR_PCTL_DFITRDDATAEN);
		val = (p_pctl_timing->tcwl - 1) / 2 - 1;
		ddrpctl_write32(val, DDR_PCTL_DFITPHYWRLAT);
	}

	return ret;
}

static __sramfunc u32 ddr_update_mr(void)
{
	if (DATA(ddr_reg).mem_type == DDR3) {
		if (DATA(ddr_freq) > DATA(ddr_reg).dram_timing.dram_dll_dis_freq) {
			if ((DATA(ddr_reg).ddr_dll_status) == DDR3_DLL_DISABLE) {
				ddr_send_command(3, MRS_cmd, bank_addr(0x1) |
						 cmd_addr((DATA(ddr_reg).ddrMR[1])));
				ddr_send_command(3, MRS_cmd, bank_addr(0x0) |
						 cmd_addr(((DATA(ddr_reg).ddrMR[0]))
						 | DDR3_DLL_RESET));
				ddr_delayus(2);	/* at least 200 DDR cycle */
				ddr_send_command(3, MRS_cmd, bank_addr(0x0) |
						 cmd_addr((DATA(ddr_reg).ddrMR[0])));
				DATA(ddr_reg).ddr_dll_status = DDR3_DLL_ENABLE;
			} else {	/* on -> on */
				ddr_send_command(3, MRS_cmd, bank_addr(0x1) |
						 cmd_addr((DATA(ddr_reg).ddrMR[1])));
				ddr_send_command(3, MRS_cmd, bank_addr(0x0) |
						 cmd_addr((DATA(ddr_reg).ddrMR[0])));
			}
		} else {
			ddr_send_command(3, MRS_cmd, bank_addr(0x1)
					 | cmd_addr(((DATA(ddr_reg).ddrMR[1]))
					 | DDR3_DLL_DISABLE));/* DLL disable */
			ddr_send_command(3, MRS_cmd, bank_addr(0x0) |
					 cmd_addr((DATA(ddr_reg).ddrMR[0])));
			DATA(ddr_reg).ddr_dll_status = DDR3_DLL_DISABLE;
		}
		ddr_send_command(3, MRS_cmd, bank_addr(0x2) |
				 cmd_addr((DATA(ddr_reg).ddrMR[2])));
	}
	return 0;
}

static __sramfunc void ddr_update_odt(void)
{
	/* adjust DRV and ODT */
	u32 dqs_odt, cmd_drv, clk_drv, dqs_drv;

	if (DATA(ddr_reg).mem_type == DDR3) {
		if (DATA(ddr_freq) <= DATA(ddr_reg).dram_timing.phy_odt_dis_freq)
			dqs_odt = PHY_DDR3_RON_RTT_DISABLE;
		else
			dqs_odt = DATA(ddr_reg).dram_timing.phy_ddr3_odt;
		cmd_drv = DATA(ddr_reg).dram_timing.phy_ddr3_cmd_drv;
		clk_drv = DATA(ddr_reg).dram_timing.phy_ddr3_clk_drv;
		dqs_drv = DATA(ddr_reg).dram_timing.phy_ddr3_dqs_drv;
	}

	ddrphy_write32(cmd_drv, PHYREG11);
	ddrphy_write32((cmd_drv << 3) | 0x2, PHYREG12);
	ddrphy_write32(clk_drv, PHYREG16);
	ddrphy_write32(clk_drv, PHYREG18);
	ddrphy_write32(dqs_drv, PHYREG20);
	ddrphy_write32(dqs_drv, PHYREG2f);
	ddrphy_write32(dqs_drv, PHYREG30);
	ddrphy_write32(dqs_drv, PHYREG3f);
	ddrphy_write32(dqs_drv, PHYREG40);
	ddrphy_write32(dqs_drv, PHYREG4f);
	ddrphy_write32(dqs_drv, PHYREG50);
	ddrphy_write32(dqs_drv, PHYREG5f);
	ddrphy_write32(dqs_odt, PHYREG21);
	ddrphy_write32(dqs_odt, PHYREG2e);
	ddrphy_write32(dqs_odt, PHYREG31);
	ddrphy_write32(dqs_odt, PHYREG3e);
	ddrphy_write32(dqs_odt, PHYREG41);
	ddrphy_write32(dqs_odt, PHYREG4e);
	ddrphy_write32(dqs_odt, PHYREG51);
	ddrphy_write32(dqs_odt, PHYREG5e);
}

static __sramfunc void idle_port(void)
{
	u32 idle_stus, i;

	ddr_copy(&(clk_gate[0]),
		 (u32 *)(RK_CRU_VIRT + RV1108_CRU_CLKGATE_CON),
		 RV1108_CRU_CLKGATES_CON_CNT);

	for (i = 0; i < RV1108_CRU_CLKGATES_CON_CNT; i++)
		cru_write32(0xffff0000, RV1108_CRU_CLKGATES_CON(i));

	/* Set all masters to stall state when ddr access is forbidden */
	grf_write32(0x00ff00ff, GRF_SOC_CON1);

	idle_stus = IDLE_MSCH_ST;
	pmu_write32(IDLE_REQ_MSCH_EN, 0x3c);
	dsb();

	while ((pmu_read32(0x40) & idle_stus) != idle_stus)
		;
}

static __sramfunc void deidle_port(void)
{
	u32 idle_stus;

	idle_stus = IDLE_MSCH_ST;
	pmu_write32(IDLE_REQ_MSCH_DIS, 0x3c);
	dsb();

	while ((pmu_read32(0x40) & idle_stus) != 0)
		;

	ddr_copy((u32 *)(RK_CRU_VIRT + RV1108_CRU_CLKGATE_CON),
		 &(clk_gate[0]), RV1108_CRU_CLKGATES_CON_CNT);
}

static void __sramfunc ddr_SRE_2_SRX(u32 freq)
{
	u32 val;
	NOC_TIMING_T *p_noc_timing = &(DATA(ddr_reg).noc_timing);
	NOC_ACTIVATE_T  *p_noc_activate = &(DATA(ddr_reg).noc_activate);
	u32 *p_readlatency = &(DATA(ddr_reg).readlatency);
	u32 *p_devtodev = &(DATA(ddr_reg).devtodev);

	grf_write32(C_ACTIVE_IN_EN, GRF_SOC_CON0);

	idle_port();
	ddr_move_to_Lowpower_state();
	DATA(ddr_freq) = freq;
	pmu_write32(PMU_GRF_DDRPHY_BUFFEREN_CORE_EN, PMU_GRF_SOC_CON0);

	val = ddrphy_read32(PHYREG00) &
	    (~(SOFT_DERESET_DIGITAL | SOFT_DERESET_ANALOG));
	ddrphy_write32(val, PHYREG00);
	dsb();
	FUNC(ddr_set_pll)(freq, 1);
	ddr_set_dll_bypass(freq);
	ddr_update_timing();
	val = ddrphy_read32(PHYREG00) | SOFT_DERESET_ANALOG;
	ddrphy_write32(val, PHYREG00);
	ddr_delayus(5);
	ddr_update_odt();
	val = ddrphy_read32(PHYREG00) | SOFT_DERESET_DIGITAL;
	ddrphy_write32(val, PHYREG00);
	pmu_write32(PMU_GRF_DDRPHY_BUFFEREN_CORE_DIS, PMU_GRF_SOC_CON0);
	dsb();
	ddr_move_to_Config_state();
	ddr_update_mr();
	ddr_data_training();
	ddr_set_dfi_lp();
	ddr_move_to_Access_state();
	grf_write32(C_ACTIVE_IN_DISABLE, GRF_SOC_CON0);
	deidle_port();

	/* During idle port, cpu accesses service msch registers would fail*/
	val = (msch_read32(MSCH_DDR_TIMING) & (1u << 31)) | p_noc_timing->d32;
	msch_write32(val, MSCH_DDR_TIMING);
	msch_write32(*p_readlatency, MSCH_READ_LATENCY);
	msch_write32(p_noc_activate->d32, MSCH_ACTIVATE);
	msch_write32(*p_devtodev, MSCH_DEVTODEV);
}

void PIE_FUNC(ddr_change_freq_sram)(void *arg)
{
	u32 *freq = (u32 *)arg;

	ddr_SRE_2_SRX(*freq);
}

EXPORT_PIE_SYMBOL(FUNC(ddr_change_freq_sram));

static bool ddr_freq_is_event_active(int id, unsigned long long tmp)
{
	if ((gmr[id].start + 200000) < tmp)
		return false;
	else
		return true;
}

static int _ddr_change_freq(u32 nmhz)
{
	u32 ret = -1;
	unsigned long long tmp = cpu_clock(0);

	do_div(tmp, 1000);
	ret = p_ddr_set_pll(nmhz, 0);

	if (ret == ddr_freq_new)
		return 0;

	switch (p_ddr_reg->mem_type) {
	case DDR3:
		ddr_get_parameter_ddr3(ret);
		break;
	default:
		break;
	}

	ddr_freq_new = ret;
	ddr_freq_scale_send_event(0, 10000);

#ifdef CONFIG_VIDEO_RK_CIF_ISP11
	if (ddr_freq_is_event_active(ISP_FE_EVENT, tmp)) {

		cif_isp11_v4l2_g_frame_interval(&numerator, &denominator);

		if (numerator * 15 < denominator)
			cif_isp11_v4l2_s_frame_interval(1, 15);
	}
#endif
	wait_event_timeout(wq, ddr_freq_new == ddr_freq_current, (HZ / 2));
	return ddr_freq_current;
}

#ifdef CONFIG_VIDEO_RK_CIF_ISP11
static void ddr_freq_change_isp_fps(struct work_struct *work)
{
	if (numerator * 15 < denominator)
		cif_isp11_v4l2_s_frame_interval(numerator, denominator);
}
#endif

static void ddr_freq_scale_tasklet(unsigned long data)
{
	int id;
	struct master_request *mr = (struct master_request *)data;
	unsigned long long tmp = cpu_clock(0), end;

	do_div(tmp, 1000);
	id = mr->id;
	end = tmp + ddr_freq_period;

	if (ddr_freq_is_event_active(ISP_FE_EVENT, tmp)) {
		if ((gmr[ISP_VS_EVENT].start < gmr[ISP_FE_EVENT].start) &&
			(gmr[VOP_EVENT].end > end)) {
			if (ddr_freq_new != ddr_freq_current) {
				__ddr_change_freq(ddr_freq_new);
				ddr_freq_current = ddr_freq_new;
#ifdef CONFIG_VIDEO_RK_CIF_ISP11
				schedule_work(&ws);
#endif
				wake_up(&wq);
			}
		}
	} else if (ddr_freq_is_event_active(VOP_EVENT, tmp)) {
		if (gmr[VOP_EVENT].end > end) {
			if (ddr_freq_new != ddr_freq_current) {
				__ddr_change_freq(ddr_freq_new);
				ddr_freq_current = ddr_freq_new;
				wake_up(&wq);
			}
		}
	} else {
		if (ddr_freq_new != ddr_freq_current) {
			__ddr_change_freq(ddr_freq_new);
			ddr_freq_current = ddr_freq_new;
		}
	}
	wake_up(&wq);
}

static int _ddr_freq_scale_send_event(int id, unsigned long timeout)
{
	unsigned long long tmp = 0;

	tmp = cpu_clock(0);
	do_div(tmp, 1000);

	gmr[id].frame_time = tmp - gmr[id].start;
	gmr[id].start = tmp;

	if (ddr_freq_new == ddr_freq_current)
		return 0;

	gmr[id].id = id;
	gmr[id].timeout = timeout;
	gmr[id].end = tmp + gmr[id].timeout;

	if (id <= VOP_EVENT) {
		if (timeout < ddr_freq_period) {
			pr_warn("%s vop blank time %luus should larger than %dus\n",
				 __func__, timeout, ddr_freq_period * 10 / 8);
			pr_warn("%s please adjust vback-porch vfront-porch vsync-len of lcd dts\n",
				__func__);
		}
		tasklet_hi_schedule(&ddr_freq_ts[id]);
	}

	return 0;
}

static int __ddr_change_freq(u32 nmhz)
{
	u32 i;
	volatile u32 n;
	unsigned long flags;
	volatile unsigned int *temp = (volatile unsigned int *)SRAM_CODE_OFFSET;

	/*
	 * 1. Make sure there is no host access
	 */
	flags = 0;
	local_irq_save(flags);
	local_fiq_disable();
	flush_cache_all();
	outer_flush_all();
	flush_tlb_all();

	for (i = 0; i < 2; i++) {
		n = temp[1024 * i];
		barrier();
	}

	asm volatile ("ldr %0,[%1]\n" : "=r" (n) : "r"(RK_DDR_VIRT));
	asm volatile ("ldr %0,[%1]\n" : "=r" (n) : "r"(RK_DDR_VIRT + RV1108_DDR_PCTL_SIZE));
	asm volatile ("ldr %0,[%1]\n" : "=r" (n) : "r"(RK_CRU_VIRT));
	asm volatile ("ldr %0,[%1]\n" : "=r" (n) : "r"(RK_GRF_VIRT));
	asm volatile ("ldr %0,[%1]\n" : "=r" (n) : "r"(RK_SERVICE_MSCH_VIRT));

	dsb();
	call_with_stack(fn_to_pie
			(rockchip_pie_chunk, &FUNC(ddr_change_freq_sram)),
			&nmhz,
			rockchip_sram_stack - (NR_CPUS -
					       1) * PAUSE_CPU_STACK_SIZE);
	local_fiq_enable();
	local_irq_restore(flags);
	return 0;
}

static u32 ddr_get_cap(u32 cs_cap)
{
	u32 cs, bank, row, col, row1, bw;
	u32 cap;

	bank = READ_BK_INFO();
	row = READ_CS0_ROW_INFO();
	col = READ_COL_INFO();
	cs = READ_CS_INFO();
	bw = READ_BW_INFO();
	if ((cs > 1) && (cs_cap == 1)) {
		row1 = READ_CS1_ROW_INFO();
		cap = ((1 << (row + col + bank + bw)) +
			(1 << (row1 + col + bank + bw)));
	} else {
		cap = (1 << (row + col + bank + bw));
	}

	return cap;
}

static int of_do_get_timings(struct device_node *np, struct ddr_timing *tim)
{
	struct device_node *np_tim = np;
	int ret;

	ret = 0;
	ret |= of_property_read_u32(np_tim, "dram_spd_bin",
				    &tim->dram_spd_bin);
	ret |= of_property_read_u32(np_tim, "sr_idle", &tim->sr_idle);
	ret |= of_property_read_u32(np_tim, "pd_idle", &tim->pd_idle);
	ret |= of_property_read_u32(np_tim, "dram_dll_disb_freq",
				    &tim->dram_dll_dis_freq);
	ret |= of_property_read_u32(np_tim, "phy_dll_disb_freq",
				    &tim->phy_dll_dis_freq);
	ret |= of_property_read_u32(np_tim, "dram_odt_disb_freq",
				    &tim->dram_odt_dis_freq);
	ret |= of_property_read_u32(np_tim, "phy_odt_disb_freq",
				    &tim->phy_odt_dis_freq);
	ret |= of_property_read_u32(np_tim, "ddr3_drv", &tim->ddr3_drv);
	ret |= of_property_read_u32(np_tim, "ddr3_odt", &tim->ddr3_odt);
	ret |= of_property_read_u32(np_tim, "lpddr3_drv", &tim->lpddr3_drv);
	ret |= of_property_read_u32(np_tim, "lpddr3_odt", &tim->lpddr3_odt);
	ret |= of_property_read_u32(np_tim, "lpddr2_drv", &tim->lpddr2_drv);

	ret |= of_property_read_u32(np_tim, "phy_ddr3_clk_drv", &tim->phy_ddr3_clk_drv);
	ret |= of_property_read_u32(np_tim, "phy_ddr3_cmd_drv", &tim->phy_ddr3_cmd_drv);
	ret |= of_property_read_u32(np_tim, "phy_ddr3_dqs_drv", &tim->phy_ddr3_dqs_drv);
	ret |= of_property_read_u32(np_tim, "phy_ddr3_odt", &tim->phy_ddr3_odt);

	ret |= of_property_read_u32(np_tim, "phy_lp23_clk_drv", &tim->phy_lp23_clk_drv);
	ret |= of_property_read_u32(np_tim, "phy_lp23_cmd_drv", &tim->phy_lp23_cmd_drv);
	ret |= of_property_read_u32(np_tim, "phy_lp23_dqs_drv", &tim->phy_lp23_dqs_drv);
	ret |= of_property_read_u32(np_tim, "phy_lp3_odt", &tim->phy_lp3_odt);
	return ret;
}

static void rv1108_ddr_timing_init(void *arg)
{
	if (arg) {
		of_do_get_timings(arg, &p_ddr_reg->dram_timing);
	} else {
		p_ddr_reg->dram_timing.dram_spd_bin = DDR3_DEFAULT;
		p_ddr_reg->dram_timing.sr_idle = 0x1;
		p_ddr_reg->dram_timing.pd_idle = 0x40;
		p_ddr_reg->dram_timing.dram_dll_dis_freq = 300;
		p_ddr_reg->dram_timing.phy_dll_dis_freq = 400;
		p_ddr_reg->dram_timing.dram_odt_dis_freq = 333;
		p_ddr_reg->dram_timing.phy_odt_dis_freq = 333;
		p_ddr_reg->dram_timing.ddr3_drv = DDR3_DS_40;
		p_ddr_reg->dram_timing.ddr3_odt = DDR3_RTT_NOM_120;
		p_ddr_reg->dram_timing.phy_ddr3_clk_drv = PHY_DDR3_RON_RTT_45ohm;
		p_ddr_reg->dram_timing.phy_ddr3_cmd_drv = PHY_DDR3_RON_RTT_34ohm;
		p_ddr_reg->dram_timing.phy_ddr3_dqs_drv = PHY_DDR3_RON_RTT_34ohm;
		p_ddr_reg->dram_timing.phy_ddr3_odt = PHY_DDR3_RON_RTT_225ohm;
		p_ddr_reg->dram_timing.phy_lp23_clk_drv = PHY_LP23_RON_RTT_43ohm;
		p_ddr_reg->dram_timing.phy_lp23_cmd_drv = PHY_LP23_RON_RTT_34ohm;
		p_ddr_reg->dram_timing.phy_lp23_dqs_drv = PHY_LP23_RON_RTT_34ohm;
		p_ddr_reg->dram_timing.phy_lp3_odt = PHY_LP23_RON_RTT_240ohm;
	}
}

static long _ddr_round_rate(unsigned int nMHz)
{
	return p_ddr_set_pll(nMHz, 0);
}

static void _ddr_set_auto_self_refresh(bool en)
{
	return;
}

int ddr_init(u32 freq, void *arg)
{
	u32 die = 1;

	p_ddr_reg = kern_to_pie(rockchip_pie_chunk, &DATA(ddr_reg));
	p_ddr_freq = kern_to_pie(rockchip_pie_chunk, &DATA(ddr_freq));
	p_ddr_set_pll = fn_to_pie(rockchip_pie_chunk, &FUNC(ddr_set_pll));

	rv1108_ddr_timing_init(arg);

	*p_ddr_freq = 0;
	p_ddr_reg->ddr_sr_idle = p_ddr_reg->dram_timing.sr_idle;
	p_ddr_reg->ddr_dll_status = DDR3_DLL_DISABLE;
	p_ddr_reg->mem_type = READ_DRAM_TYPE_INFO();

	die = 1 << (READ_BW_INFO() - READ_DIE_BW_INFO());
	p_ddr_reg->ddr_capability_per_die = ddr_get_cap(0) / die;
	p_ddr_reg->ddr_speed_bin = p_ddr_reg->dram_timing.dram_spd_bin;

	ddr_freq_current = ddr_get_pll_freq(DPLL) / 2;
	ddr_freq_new = ddr_freq_current;
#ifdef CONFIG_VIDEO_RK_CIF_ISP11
	INIT_WORK(&ws, ddr_freq_change_isp_fps);
#endif
	init_waitqueue_head(&wq);

	tasklet_init(&ddr_freq_ts[0], ddr_freq_scale_tasklet,
			(unsigned long)&gmr[0]);
	tasklet_init(&ddr_freq_ts[VOP_EVENT], ddr_freq_scale_tasklet,
			(unsigned long)&gmr[VOP_EVENT]);

	if (freq == 0)
		_ddr_change_freq(ddr_freq_current);
	else
		_ddr_change_freq(freq);

	return 0;
}
