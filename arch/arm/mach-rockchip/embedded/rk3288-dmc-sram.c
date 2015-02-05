/*
* Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
*/

#include <linux/io.h>
#include <soc/rockchip/rk3288-dmc-sram.h>

#include "rk3288_ddr.h"
#include "sram_delay.h"

#define dmc_io_and(and_data, addr)	\
	__raw_writel(__raw_readl(addr) & (and_data), addr)

#define dmc_io_or(or_data, addr)	\
	__raw_writel(__raw_readl(addr) | (or_data), addr)

#define CH_MAX	2

/* PMU Registers Define */
#define PMU_WAKEUP_CFG0	0x0000
#define PMU_IDLE_ST	0x0014
#define PMU_PWRDN_ST	0x000c
#define PMU_IDLE_REQ	0x0010

/* PMU_PWRDN_ST */
#define PD_HEVC_PWR_ST		(1 << 10)
#define PD_GPU_PWR_ST		(1 << 9)
#define PD_VIDEO_PWR_ST		(1 << 8)
#define PD_VIO_PWR_ST		(1 << 7)
#define PD_PERI_PWR_ST		(1 << 6)

/* PMU_IDLE_REQ */
#define IDLE_REQ_HEVC_CFG	(1 << 9)
#define IDLE_REQ_DMA_CFG	(1 << 7)
#define IDLE_REQ_CORE_CFG	(1 << 5)
#define IDLE_REQ_VIO_CFG	(1 << 4)
#define IDLE_REQ_VIDEO_CFG	(1 << 3)
#define IDLE_REQ_GPU_CFG	(1 << 2)
#define IDLE_REQ_PERI_CFG	(1 << 1)

/* PMU_IDLE_ST */
#define IDLE_HEVC_ST		(1 << 9)
#define IDLE_DMA_ST		(1 << 7)
#define IDLE_CORE_ST		(1 << 5)
#define IDLE_VIO_ST		(1 << 4)
#define IDLE_VIDEO_ST		(1 << 3)
#define IDLE_GPU_ST		(1 << 2)
#define IDLE_PERI_ST		(1 << 1)

/* CRU Registers Define */
#define CRU_PLL_CON(x, y)	((x) * 0x10 + (y) * 0x4)
#define CRU_MODE_CON		0x0050
#define CRU_CLKSEL_CON(x)	((x) * 0x4 + 0x60)
#define CRU_CLKGATE_CON(x)	((x) * 0x4 + 0x160)
#define CLKGATE_MAX_NUM		19

/* CRU_PLL_CON */
#define PLL_RESET	(((0x1 << 5) << 16) | (0x1 << 5))
#define PLL_DE_RESET	(((0x1 << 5) << 16) | (0x0 << 5))
#define NR(n)		((0x3f << (8 + 16)) | (((n) - 1) << 8))
#define NO(n)		((0xf << 16) | ((n) - 1))
#define NF(n)		((0x1fff << 16) | ((n) - 1))
#define NB(n)		((0xfff << 16) | ((n) - 1))

/* REG FILE Registers Define */
#define GRF_SOC_CON0	0x0244
#define GRF_SOC_STATUS1	0x0284
#define DPLL_LOCK	(1 << 5)

/* GRF_SOC_CON0 */
#define UPCTL_C_ACTIVE_IN(en, ch)	\
	((1 << (16 + 5 + (ch))) | ((en) << (5 + (ch))))

/* DFI Control Registers */
#define DDR_PCTL_DFITPHYWRLAT		0x0254
#define DDR_PCTL_DFITRDDATAEN		0x0260

#define GET_CTL_STAT(x)	(((x) >> 0) & 0x7)
#define GET_LP_TRIG(x)	(((x) >> 4) & 0x7)

/* DDR_PCTL_MCFG */
#define MDDR_LPDDR2_BL_4	(1 << 20)
#define MDDR_LPDDR2_BL_8	(2 << 20)
#define MDDR_LPDDR2_BL_16	(3 << 20)
#define DDR2_DDR3_BL_4		0
#define DDR2_DDR3_BL_8		1

#define tfaw_cfg(n)	(((n) - 4) << 18)
#define PD_EXIT_SLOW	0
#define PD_EXIT_FAST	(1 << 17)
#define pd_type(n)	((n) << 16)

/* MCMD */
#define DESELECT_CMD	0
#define PREA_CMD	1
#define REF_CMD		2
#define MRS_CMD		3
#define ZQCS_CMD	4
#define ZQCL_CMD	5
#define RSTL_CMD	6
#define MRR_CMD		8
#define DPDE_CMD	9

#define lpddr2_op(n)	((n) << 12)
#define lpddr2_ma(n)	((n) << 4)
#define bank_addr(n)	((n) << 17)
#define cmd_addr(n)	((n) << 4)

#define START_CMD	(1u << 31)

/* DDR_PCTL_SCFG */
#define HW_LOW_POWER_EN(x)	((x) & 0x1)

/* DDR_PUBL_MR0 */
#define DDR3_DLL_RESET	(1 << 8)
#define DDR3_BL8	0

/* DDR_PUBL_MR1 */
#define DDR3_DLL_DISABLE	0x1
#define LPDDR2_BL4		0x2
#define LPDDR2_BL8		0x3

/* MSCH Register Define */
#define MSCH_DDRCONFIG	0x0008
#define MSCH_DDRTIMING	0x000c
#define MSCH_ACTIVATE	0x0038

/* MSCH_DDRTIMING */
#define GET_MSCH_BWRATION(x)	((x) & (0x1 << 31))

#define DPLL_MODE_MSK	(0x3 << 20)
#define DPLL_MODE_SLOW	(0 << 4)
#define DPLL_MODE_NORM	(1 << 4)

static struct dmc_regtiming g_ddr_timing;

struct channel_info_tag {
	void __iomem *p_ddr_reg;
	void __iomem *p_phy_reg;
	void __iomem *p_msch_reg;
	enum dram_type_tag mem_type;
};

struct dmc_sram {
	u32 cur_freq;
	u32 target_freq;
	u32 freq_slew;
	u32 dqstr_value;

	u32 channel_num;
	u32 dtar[NUM_MC_CHANNEL_MAX];

	void __iomem *sram;

	u32 ddr_type;
	u32 odt_disable_freq;
	u32 dll_disable_freq;
	u32 sr_enable_freq;
	u32 pd_enable_freq;
	u32 sr_cnt;
	u32 pd_cnt;

	struct dmc_regtiming *regtiming;
};

static void __iomem *p_cru_reg;
static void __iomem *p_grf_reg;
static void __iomem *p_pmu_reg;
static struct channel_info_tag ddr_ch[CH_MAX];

static struct dmc_sram g_dmc_sram;
static u32 clkr;
static u32 clkf;
static u32 clkod;

static void ddr_copy(u64 *p_dest, u64 *p_src, u32 w_word)
{
	u32 i;

	for (i = 0; i < w_word; i++)
		p_dest[i] = p_src[i];
}

static void rk3288_dmc_set_dpll(u32 nmhz)
{
	u32 val;
	int delay;

	/* PLL slow-mode */
	val = DPLL_MODE_MSK | DPLL_MODE_SLOW;
	__raw_writel(val, p_cru_reg + CRU_MODE_CON);

	if (nmhz == 24)
		return;

	__raw_writel(PLL_RESET, p_cru_reg + CRU_PLL_CON(1, 3));
	sram_udelay(1);
	__raw_writel(NR(clkr) | NO(clkod),
		     p_cru_reg + CRU_PLL_CON(1, 0));
	__raw_writel(NF(clkf), p_cru_reg + CRU_PLL_CON(1, 1));
	__raw_writel(NB(clkf >> 1), p_cru_reg + CRU_PLL_CON(1, 2));
	sram_udelay(5);
	__raw_writel(PLL_DE_RESET, p_cru_reg + CRU_PLL_CON(1, 3));

	delay = 1000;
	while (delay > 0) {
		/* get dpll lock status */
		if (__raw_readl(p_grf_reg + GRF_SOC_STATUS1) & DPLL_LOCK)
			break;
		sram_udelay(1);
		delay--;
	}

	/* clk_ddr_src = DDR PLL,clk_ddr_src:clk_ddrphy = 1:1 */
	val = ((0x3 | (0x1 << 2)) << 16) | (0 << 2) | 0;
	__raw_writel(val, p_cru_reg + CRU_CLKSEL_CON(26));

	/* PLL normal */
	val = DPLL_MODE_MSK | DPLL_MODE_NORM;
	__raw_writel(val, p_cru_reg + CRU_MODE_CON);
}

static u32 idle_port(u32 clk_gate[])
{
	u32 i, deidle_req, idle_req;

	/* save clock gate status and enable all clock gate for request idle */
	for (i = 0; i < CLKGATE_MAX_NUM; i++) {
		clk_gate[i] = __raw_readl(p_cru_reg + CRU_CLKGATE_CON(i));
		__raw_writel(0xffff0000, p_cru_reg + CRU_CLKGATE_CON(i));
	}

	/*
	 * Save current idle_req.  In deidle_port we deidle all
	 * nius that were either active or transitioning to active.
	 */
	deidle_req = __raw_readl(p_pmu_reg + PMU_IDLE_REQ);
	idle_req = IDLE_REQ_HEVC_CFG | IDLE_REQ_DMA_CFG |
		   IDLE_REQ_CORE_CFG | IDLE_REQ_VIO_CFG |
		   IDLE_REQ_VIDEO_CFG | IDLE_REQ_GPU_CFG |
		   IDLE_REQ_PERI_CFG;
	dmc_io_or(idle_req, p_pmu_reg + PMU_IDLE_REQ);

	while ((__raw_readl(p_pmu_reg + PMU_IDLE_ST) & idle_req) != idle_req)
		continue;

	return deidle_req;
}

static void deidle_port(const u32 clk_gate[], u32 deidle_req)
{
	u32 i, val;

	__raw_writel(deidle_req, p_pmu_reg + PMU_IDLE_REQ);

	while ((__raw_readl(p_pmu_reg + PMU_IDLE_ST) & deidle_req) !=
	       deidle_req)
		continue;

	/* restore clock gate status */
	for (i = 0; i < CLKGATE_MAX_NUM; i++) {
		val = clk_gate[i] | 0xffff0000;
		__raw_writel(val, p_cru_reg + CRU_CLKGATE_CON(i));
	}
}

static void ddr_reset_dll(u32 ch)
{
	void __iomem *p_ddr_reg = ddr_ch[ch].p_ddr_reg;
	void __iomem *p_phy_reg = ddr_ch[ch].p_phy_reg;

	dmc_io_and(~0x40000000, p_phy_reg + DDR_PUBL_ACDLLCR);
	dmc_io_and(~0x40000000, p_phy_reg + DDR_PUBL_DX0DLLCR);
	dmc_io_and(~0x40000000, p_phy_reg + DDR_PUBL_DX1DLLCR);
	if (!(__raw_readl(p_ddr_reg + DDR_PCTL_PPCFG) & 1)) {
		dmc_io_and(~0x40000000, p_phy_reg + DDR_PUBL_DX2DLLCR);
		dmc_io_and(~0x40000000, p_phy_reg + DDR_PUBL_DX3DLLCR);
	}
	sram_udelay(1);
	dmc_io_or(0x40000000, p_phy_reg + DDR_PUBL_ACDLLCR);
	dmc_io_or(0x40000000, p_phy_reg + DDR_PUBL_DX0DLLCR);
	dmc_io_or(0x40000000, p_phy_reg + DDR_PUBL_DX1DLLCR);
	if (!(__raw_readl(p_ddr_reg + DDR_PCTL_PPCFG) & 1)) {
		dmc_io_or(0x40000000, p_phy_reg + DDR_PUBL_DX2DLLCR);
		dmc_io_or(0x40000000, p_phy_reg + DDR_PUBL_DX3DLLCR);
	}
	sram_udelay(1);
}

static void ddr_move_to_lowpower_state(u32 ch)
{
	u32 value;
	void __iomem *p_ddr_reg = ddr_ch[ch].p_ddr_reg;

	while (1) {
		value = GET_CTL_STAT(__raw_readl(p_ddr_reg + DDR_PCTL_STAT));
		if (value == LOW_POWER)
			break;

		switch (value) {
		case INIT_MEM:
			/* INIT_MEM->Config->ACCESS->LOW_POWER */
			__raw_writel(CFG_STATE, p_ddr_reg + DDR_PCTL_SCTL);
			while (GET_CTL_STAT(__raw_readl(p_ddr_reg +
			       DDR_PCTL_STAT)) != CONFIG)
				continue;
			__raw_writel(GO_STATE, p_ddr_reg + DDR_PCTL_SCTL);
			while (GET_CTL_STAT(__raw_readl(p_ddr_reg +
			       DDR_PCTL_STAT)) != ACCESS)
				continue;
			__raw_writel(SLEEP_STATE,
				     p_ddr_reg + DDR_PCTL_SCTL);
			while (GET_CTL_STAT(__raw_readl(p_ddr_reg +
			       DDR_PCTL_STAT)) != LOW_POWER)
				continue;
			break;
		case CONFIG:
			/* CONFIG->ACCESS->LOW_POWER */
			__raw_writel(GO_STATE, p_ddr_reg + DDR_PCTL_SCTL);
			while (GET_CTL_STAT(__raw_readl(p_ddr_reg +
			       DDR_PCTL_STAT)) != ACCESS)
				continue;
			__raw_writel(SLEEP_STATE,
				     p_ddr_reg + DDR_PCTL_SCTL);
			while (GET_CTL_STAT(__raw_readl(p_ddr_reg +
			       DDR_PCTL_STAT)) != LOW_POWER)
				continue;
			break;
		case ACCESS:
			__raw_writel(SLEEP_STATE,
				     p_ddr_reg + DDR_PCTL_SCTL);
			while (GET_CTL_STAT(__raw_readl(p_ddr_reg +
			       DDR_PCTL_STAT)) != LOW_POWER)
				continue;
			break;
		default:
			break;
		}
	}
}

static void ddr_move_to_access_state(u32 ch)
{
	u32 value;
	void __iomem *p_ddr_reg = ddr_ch[ch].p_ddr_reg;
	void __iomem *p_phy_reg = ddr_ch[ch].p_phy_reg;

	/* set auto self-refresh idle */
	value = __raw_readl(p_ddr_reg + DDR_PCTL_MCFG1);
	value = (value & 0xffffff00) | g_dmc_sram.sr_cnt | (1 << 31);
	__raw_writel(value, p_ddr_reg + DDR_PCTL_MCFG1);

	while (1) {
		value = GET_CTL_STAT(__raw_readl(p_ddr_reg + DDR_PCTL_STAT));
		if ((value == ACCESS) ||
		    ((GET_LP_TRIG(__raw_readl(p_ddr_reg +
		      DDR_PCTL_STAT)) == 1) && (value == LOW_POWER))) {
			break;
		}
		switch (value) {
		case LOW_POWER:
			__raw_writel(WAKEUP_STATE,
				     p_ddr_reg + DDR_PCTL_SCTL);
			while (GET_CTL_STAT(__raw_readl(p_ddr_reg +
			       DDR_PCTL_STAT)) != ACCESS)
				continue;
			while ((__raw_readl(p_phy_reg + DDR_PUBL_PGSR) &
				PGSR_DLDONE) != PGSR_DLDONE)
				continue;
			break;
		case INIT_MEM:
			/* INIT_MEM->CONFIG->ACCESS */
			__raw_writel(CFG_STATE, p_ddr_reg + DDR_PCTL_SCTL);
			while (GET_CTL_STAT(__raw_readl(p_ddr_reg +
			       DDR_PCTL_STAT)) != CONFIG)
				continue;
			__raw_writel(GO_STATE, p_ddr_reg + DDR_PCTL_SCTL);
			while (!((GET_CTL_STAT(__raw_readl(p_ddr_reg +
				  DDR_PCTL_STAT)) == ACCESS) ||
				 ((GET_LP_TRIG(__raw_readl(p_ddr_reg +
				   DDR_PCTL_STAT)) == 1) &&
				  (GET_CTL_STAT(__raw_readl(p_ddr_reg +
				   DDR_PCTL_STAT)) == LOW_POWER))))
				continue;
			break;
		case CONFIG:
			__raw_writel(GO_STATE, p_ddr_reg + DDR_PCTL_SCTL);
			while (!((GET_CTL_STAT(__raw_readl(p_ddr_reg +
				  DDR_PCTL_STAT)) == ACCESS) ||
				 ((GET_LP_TRIG(__raw_readl(p_ddr_reg +
				   DDR_PCTL_STAT)) == 1) &&
				  (GET_CTL_STAT(__raw_readl(p_ddr_reg +
				   DDR_PCTL_STAT)) == LOW_POWER))))
				continue;
			break;
		default:
			break;
		}
	}
	/* de_hw_wakeup :enable auto sr if sr_idle != 0 */
	__raw_writel(UPCTL_C_ACTIVE_IN(0, ch), p_grf_reg + GRF_SOC_CON0);
}

static void ddr_move_to_config_state(u32 ch)
{
	u32 value;
	void __iomem *p_ddr_reg = ddr_ch[ch].p_ddr_reg;
	void __iomem *p_phy_reg = ddr_ch[ch].p_phy_reg;

	/* hw_wakeup :disable auto sr */
	__raw_writel(UPCTL_C_ACTIVE_IN(1, ch), p_grf_reg + GRF_SOC_CON0);

	while (1) {
		value = GET_CTL_STAT(__raw_readl(p_ddr_reg + DDR_PCTL_STAT));
		if (value == CONFIG)
			break;

		switch (value) {
		case LOW_POWER:
			/* LOW_POWER->ACCESS->CONFIG */
			__raw_writel(WAKEUP_STATE,
				     p_ddr_reg + DDR_PCTL_SCTL);
			while (GET_CTL_STAT(__raw_readl(p_ddr_reg +
			       DDR_PCTL_STAT)) != ACCESS)
				continue;
			while ((__raw_readl(p_phy_reg +
			       DDR_PUBL_PGSR) & PGSR_DLDONE) != PGSR_DLDONE)
				continue;
			__raw_writel(CFG_STATE, p_ddr_reg + DDR_PCTL_SCTL);
			while (GET_CTL_STAT(__raw_readl(p_ddr_reg +
			       DDR_PCTL_STAT)) != CONFIG)
				continue;
			break;
		case ACCESS:
		case INIT_MEM:
			__raw_writel(CFG_STATE, p_ddr_reg + DDR_PCTL_SCTL);
			while (GET_CTL_STAT(__raw_readl(p_ddr_reg +
			       DDR_PCTL_STAT)) != CONFIG)
				continue;
			break;
		default:
			break;
		}
	}
}

static void ddr_send_command(u32 ch, u32 rank, u32 cmd, u32 arg)
{
	void __iomem *p_ddr_reg = ddr_ch[ch].p_ddr_reg;
	u32 val;

	val = (START_CMD | (rank << 20) | arg | cmd);
	__raw_writel(val, p_ddr_reg + DDR_PCTL_MCMD);
	while (__raw_readl(p_ddr_reg + DDR_PCTL_MCMD) & START_CMD)
		continue;
}

/*
 * ddr data training trigger
 * 0  DTT succeed
 * !0 DTT fail
 */
static u32 ddr_data_training_trigger(u32 ch)
{
	u32 val, cs;
	void __iomem *p_ddr_reg = ddr_ch[ch].p_ddr_reg;
	void __iomem *p_phy_reg = ddr_ch[ch].p_phy_reg;

	/* disable auto refresh */
	__raw_writel(0x0, p_ddr_reg + DDR_PCTL_TREFI);
	if ((ddr_ch[ch].mem_type != LPDDR2) &&
	    (ddr_ch[ch].mem_type != LPDDR3)) {
		/* passive window */
		dmc_io_or((1 << 1), p_phy_reg + DDR_PUBL_PGCR);
	}
	/* clear DTDONE status */
	dmc_io_or(PIR_CLRSR, p_phy_reg + DDR_PUBL_PIR);
	cs = ((__raw_readl(p_phy_reg + DDR_PUBL_PGCR) >> 18) & 0xf);
	/* use cs0 dtt */
	val = __raw_readl(p_phy_reg + DDR_PUBL_PGCR);
	val = (val & (~(0xf << 18))) | (1 << 18);
	__raw_writel(val, p_phy_reg + DDR_PUBL_PGCR);
	/* trigger DTT */
	val = PIR_INIT | PIR_QSTRN | PIR_LOCKBYP | PIR_ZCALBYP | PIR_CLRSR |
	      PIR_ICPC;
	dmc_io_or(val, p_phy_reg + DDR_PUBL_PIR);
	return cs;
}

static bool ddr_data_training(u32 ch, u32 cs)
{
	u32 val, i, byte;
	void __iomem *p_ddr_reg = ddr_ch[ch].p_ddr_reg;
	void __iomem *p_phy_reg = ddr_ch[ch].p_phy_reg;

	/* wait echo byte DTDONE */
	while ((__raw_readl(p_phy_reg + DDR_PUBL_DX0GSR0) & 1) != 1)
		continue;
	while ((__raw_readl(p_phy_reg + DDR_PUBL_DX1GSR0) & 1) != 1)
		continue;
	if (!(__raw_readl(p_ddr_reg + DDR_PCTL_PPCFG) & 1)) {
		while ((__raw_readl(p_phy_reg + DDR_PUBL_DX2GSR0) & 1) != 1)
			continue;
		while ((__raw_readl(p_phy_reg + DDR_PUBL_DX3GSR0) & 1) != 1)
			continue;
		byte = 4;
	}

	/* restore cs */
	val = __raw_readl(p_phy_reg + DDR_PUBL_PGCR);
	val = (val & (~(0xf << 18))) | (cs << 18);
	__raw_writel(val, p_phy_reg + DDR_PUBL_PGCR);

	for (i = 0; i < byte; i++) {
		val = __raw_readl(p_phy_reg + DDR_PUBL_DX0DQSTR + i * 0x40);
		val = (val & (~((0x7 << 3) | (0x3 << 14)))) |
			((val & 0x7) << 3) | (((val >> 12) & 0x3) << 14);
		__raw_writel(val, p_phy_reg + DDR_PUBL_DX0DQSTR + i * 0x40);
	}

	/* send some auto refresh to complement the lost while DTT */
	if (cs > 1) {
		ddr_send_command(ch, cs, REF_CMD, 0);
		ddr_send_command(ch, cs, REF_CMD, 0);
		ddr_send_command(ch, cs, REF_CMD, 0);
		ddr_send_command(ch, cs, REF_CMD, 0);
	} else {
		ddr_send_command(ch, cs, REF_CMD, 0);
		ddr_send_command(ch, cs, REF_CMD, 0);
	}

	if ((ddr_ch[ch].mem_type != LPDDR2) &&
	    (ddr_ch[ch].mem_type != LPDDR3)) {
		/* active window */
		dmc_io_and(~(1 << 1), p_phy_reg + DDR_PUBL_PGCR);
	}

	/* resume auto refresh */
	__raw_writel(g_dmc_sram.regtiming->ctl_timing.trefi,
		     p_ddr_reg + DDR_PCTL_TREFI);

	if (__raw_readl(p_phy_reg + DDR_PUBL_PGSR) & PGSR_DTERR)
		return false;
	else
		return true;
}

static void ddr_set_dll_bypass(u32 ch, u32 freq)
{
	void __iomem *p_ddr_reg = ddr_ch[ch].p_ddr_reg;
	void __iomem *p_phy_reg = ddr_ch[ch].p_phy_reg;

	if (freq <= 150) {
		dmc_io_and(~(1 << 23), p_phy_reg + DDR_PUBL_DLLGCR);
		dmc_io_or(0x80000000, p_phy_reg + DDR_PUBL_ACDLLCR);
		dmc_io_or(0x80000000, p_phy_reg + DDR_PUBL_DX0DLLCR);
		dmc_io_or(0x80000000, p_phy_reg + DDR_PUBL_DX1DLLCR);
		dmc_io_or(0x80000000, p_phy_reg + DDR_PUBL_DX2DLLCR);
		dmc_io_or(0x80000000, p_phy_reg + DDR_PUBL_DX3DLLCR);
		dmc_io_or(PIR_DLLBYP, p_phy_reg + DDR_PUBL_PIR);
	} else if (freq <= 250) {
		dmc_io_or((1 << 23), p_phy_reg + DDR_PUBL_DLLGCR);
		dmc_io_or(0x80000000, p_phy_reg + DDR_PUBL_ACDLLCR);
		dmc_io_or(0x80000000, p_phy_reg + DDR_PUBL_DX0DLLCR);
		dmc_io_or(0x80000000, p_phy_reg + DDR_PUBL_DX1DLLCR);
		dmc_io_or(0x80000000, p_phy_reg + DDR_PUBL_DX2DLLCR);
		dmc_io_or(0x80000000, p_phy_reg + DDR_PUBL_DX3DLLCR);
		dmc_io_or(PIR_DLLBYP, p_phy_reg + DDR_PUBL_PIR);
	} else {
		dmc_io_and(~(1 << 23), p_phy_reg + DDR_PUBL_DLLGCR);
		dmc_io_and(~0x80000000, p_phy_reg + DDR_PUBL_ACDLLCR);
		dmc_io_and(~0x80000000, p_phy_reg + DDR_PUBL_DX0DLLCR);
		dmc_io_and(~0x80000000, p_phy_reg + DDR_PUBL_DX1DLLCR);
		if (!(__raw_readl(p_ddr_reg + DDR_PCTL_PPCFG) & 1)) {
			dmc_io_and(~0x80000000,  p_phy_reg + DDR_PUBL_DX2DLLCR);
			dmc_io_and(~0x80000000,  p_phy_reg + DDR_PUBL_DX3DLLCR);
		}
		dmc_io_and(~PIR_DLLBYP,  p_phy_reg + DDR_PUBL_PIR);
	}
}

static u32 ddr_update_timing(u32 ch)
{
	u32 val, i, bl_tmp = 0;
	struct ctl_timing_tag *p_ctl_timing =
		&(g_dmc_sram.regtiming->ctl_timing);
	struct phy_timing_tag *p_phy_timing =
		&(g_dmc_sram.regtiming->phy_timing);
	union noc_timing_tag *p_noc_timing =
	    &(g_dmc_sram.regtiming->noc_timing);
	union noc_activate_tag *p_noc_activate =
	    &(g_dmc_sram.regtiming->noc_activate);
	void __iomem *p_ddr_reg = ddr_ch[ch].p_ddr_reg;
	void __iomem *p_phy_reg = ddr_ch[ch].p_phy_reg;
	void __iomem *p_msch_reg = ddr_ch[ch].p_msch_reg;

	ddr_copy((u64 *)(p_ddr_reg + DDR_PCTL_TOGCNT1U),
		 (u64 *)&(p_ctl_timing->togcnt1u), 17);
	__raw_writel(p_phy_timing->dtpr0.d32, p_phy_reg + DDR_PUBL_DTPR0);
	__raw_writel(p_phy_timing->dtpr1.d32, p_phy_reg + DDR_PUBL_DTPR1);
	__raw_writel(p_phy_timing->dtpr2.d32, p_phy_reg + DDR_PUBL_DTPR2);

	val = __raw_readl(p_msch_reg + MSCH_DDRTIMING);
	val = GET_MSCH_BWRATION(val) | p_noc_timing->d32;
	__raw_writel(val, p_msch_reg + MSCH_DDRTIMING);
	__raw_writel(p_noc_activate->d32, p_msch_reg + MSCH_ACTIVATE);
	/* Update PCTL BL */
	if (ddr_ch[ch].mem_type == DDR3) {
		bl_tmp =
		    ((p_phy_timing->mr[0] & 0x3) ==
		     DDR3_BL8) ? DDR2_DDR3_BL_8 : DDR2_DDR3_BL_4;
		val = __raw_readl(p_ddr_reg + DDR_PCTL_MCFG);
		val = (val & (~(0x1 | (0x3 << 18) | (0x1 << 17) |
		       (0x1 << 16)))) | bl_tmp | tfaw_cfg(5) |
		      PD_EXIT_SLOW | pd_type(1);
		__raw_writel(val, p_ddr_reg + DDR_PCTL_MCFG);

		if (g_dmc_sram.target_freq <= g_dmc_sram.dll_disable_freq) {
			val = __raw_readl(p_ddr_reg + DDR_PCTL_TCL)  - 3;
			__raw_writel(val, p_ddr_reg + DDR_PCTL_DFITRDDATAEN);
		} else {
			val = __raw_readl(p_ddr_reg + DDR_PCTL_TCL)  - 2;
			__raw_writel(val, p_ddr_reg + DDR_PCTL_DFITRDDATAEN);
		}
		val = __raw_readl(p_ddr_reg + DDR_PCTL_TCWL)  - 1;
		__raw_writel(val, p_ddr_reg + DDR_PCTL_DFITPHYWRLAT);
	} else if ((ddr_ch[ch].mem_type == LPDDR2) ||
		   (ddr_ch[ch].mem_type == LPDDR3)) {
		if (((p_phy_timing->mr[1]) & 0x7) == LPDDR2_BL8)
			bl_tmp = MDDR_LPDDR2_BL_8;
		else if (((p_phy_timing->mr[1]) & 0x7) == LPDDR2_BL4)
			bl_tmp = MDDR_LPDDR2_BL_4;
		else
			bl_tmp = MDDR_LPDDR2_BL_16;

		if ((g_dmc_sram.target_freq >= 200) ||
		    (ddr_ch[ch].mem_type == LPDDR3)) {
			val = __raw_readl(p_ddr_reg + DDR_PCTL_MCFG);
			val = (val & (~((0x3 << 20) | (0x3 << 18) |
			       (0x1 << 17) | (0x1 << 16)))) | bl_tmp |
			      tfaw_cfg(5) | PD_EXIT_FAST | pd_type(1);
			__raw_writel(val, p_ddr_reg + DDR_PCTL_MCFG);
		} else {
			val = __raw_readl(p_ddr_reg + DDR_PCTL_MCFG);
			val = (val & (~((0x3 << 20) | (0x3 << 18) |
			       (0x1 << 17) | (0x1 << 16)))) | bl_tmp |
			      tfaw_cfg(6) | PD_EXIT_FAST | pd_type(1);
			__raw_writel(val, p_ddr_reg + DDR_PCTL_MCFG);
		}

		i = ((__raw_readl(p_phy_reg + DDR_PUBL_DTPR1) >> 27) & 0x7) -
		    ((__raw_readl(p_phy_reg + DDR_PUBL_DTPR1) >> 24) & 0x7);
		/* tDQSCKmax-tDQSCK */
		val = __raw_readl(p_phy_reg + DDR_PUBL_DSGCR);
		val = (val & (~(0x3f << 5))) | (i << 5) | (i << 8);
		__raw_writel(val, p_phy_reg + DDR_PUBL_DSGCR);
		val = __raw_readl(p_ddr_reg + DDR_PCTL_TCL)  - 1;
		__raw_writel(val, p_ddr_reg + DDR_PCTL_DFITRDDATAEN);
		val = __raw_readl(p_ddr_reg + DDR_PCTL_TCWL);
		__raw_writel(val, p_ddr_reg + DDR_PCTL_DFITPHYWRLAT);
	}

	return 0;
}

static u32 ddr_update_mr(u32 ch)
{
	struct phy_timing_tag *p_phy_timing =
				&(g_dmc_sram.regtiming->phy_timing);
	u32 val, cs, dll_off;
	void __iomem *p_phy_reg = ddr_ch[ch].p_phy_reg;

	val = __raw_readl(p_phy_reg + DDR_PUBL_PGCR);
	cs = ((val >> 18) & 0xf);
	val = __raw_readl(p_phy_reg + DDR_PUBL_MR1);
	dll_off = (val & DDR3_DLL_DISABLE) ? 1 : 0;
	ddr_copy((u64 *)(p_phy_reg + DDR_PUBL_MR0),
		 (u64 *)&(p_phy_timing->mr[0]), 2);
	if (ddr_ch[ch].mem_type == DDR3) {
		ddr_send_command(ch, cs, MRS_CMD,
				 bank_addr(0x2) |
				 cmd_addr((p_phy_timing->mr[2])));
		if (g_dmc_sram.target_freq > g_dmc_sram.dll_disable_freq) {
			if (dll_off) {
				/* off -> on */
				/* DLL enable */
				ddr_send_command(ch, cs, MRS_CMD,
						 bank_addr(0x1) |
					cmd_addr((p_phy_timing->mr[1])));
				/* DLL reset */
				ddr_send_command(ch, cs, MRS_CMD,
						 bank_addr(0x0) |
					cmd_addr(((p_phy_timing->mr[0])) |
					DDR3_DLL_RESET));
				/* at least 200 DDR cycle */
				sram_udelay(1);
				ddr_send_command(ch, cs, MRS_CMD,
						 bank_addr(0x0) |
					cmd_addr((p_phy_timing->mr[0])));
			} else {
				/* on -> on */
				ddr_send_command(ch, cs, MRS_CMD,
						 bank_addr(0x1) |
						 cmd_addr((p_phy_timing->
							   mr[1])));
				ddr_send_command(ch, cs, MRS_CMD,
						 bank_addr(0x0) |
						 cmd_addr((p_phy_timing->
							   mr[0])));
			}
		} else {
			val = ((p_phy_timing->mr[1])) | DDR3_DLL_DISABLE;
			__raw_writel(val, p_phy_reg + DDR_PUBL_MR1);
			/* DLL disable */
			ddr_send_command(ch, cs, MRS_CMD, bank_addr(0x1) |
					 cmd_addr(((p_phy_timing->mr[1])) |
					 DDR3_DLL_DISABLE));
			ddr_send_command(ch, cs, MRS_CMD,
					 bank_addr(0x0) |
					 cmd_addr((p_phy_timing->mr[0])));
		}
	} else if ((ddr_ch[ch].mem_type == LPDDR2) ||
		   (ddr_ch[ch].mem_type == LPDDR3)) {
		ddr_send_command(ch, cs, MRS_CMD,
				 lpddr2_ma(0x1) |
				 lpddr2_op((p_phy_timing->mr[1])));
		ddr_send_command(ch, cs, MRS_CMD,
				 lpddr2_ma(0x2) |
				 lpddr2_op((p_phy_timing->mr[2])));
		ddr_send_command(ch, cs, MRS_CMD,
				 lpddr2_ma(0x3) |
				 lpddr2_op((p_phy_timing->mr[3])));
		if (ddr_ch[ch].mem_type == LPDDR3) {
			ddr_send_command(ch, cs, MRS_CMD,
					 lpddr2_ma(11) |
					 lpddr2_op((p_phy_timing->mr11)));
		}
	} else {
		/* mDDR */
		ddr_send_command(ch, cs, MRS_CMD,
				 bank_addr(0x0) |
				 cmd_addr((p_phy_timing->mr[0])));
		ddr_send_command(ch, cs, MRS_CMD, bank_addr(0x1) |
				 cmd_addr((p_phy_timing->mr[2])));
	}
	return 0;
}

static void ddr_update_odt(u32 ch)
{
	void __iomem *p_ddr_reg = ddr_ch[ch].p_ddr_reg;
	void __iomem *p_phy_reg = ddr_ch[ch].p_phy_reg;

	/* adjust DRV and ODT */
	if ((ddr_ch[ch].mem_type == DDR3) || (ddr_ch[ch].mem_type == LPDDR3)) {
		if (g_dmc_sram.target_freq <= g_dmc_sram.odt_disable_freq) {
			/* dynamic RTT disable */
			dmc_io_and(~(0x3 << 9), p_phy_reg + DDR_PUBL_DX0GCR);
			dmc_io_and(~(0x3 << 9), p_phy_reg + DDR_PUBL_DX1GCR);
			if (!(__raw_readl(p_ddr_reg + DDR_PCTL_PPCFG) & 1)) {
				dmc_io_and(~(0x3 << 9),
					   p_phy_reg + DDR_PUBL_DX2GCR);
				dmc_io_and(~(0x3 << 9),
					   p_phy_reg + DDR_PUBL_DX3GCR);
			}
		} else {
			/* dynamic RTT enable */
			dmc_io_or((0x3 << 9), p_phy_reg + DDR_PUBL_DX0GCR);
			dmc_io_or((0x3 << 9), p_phy_reg + DDR_PUBL_DX1GCR);
			if (!(__raw_readl(p_ddr_reg + DDR_PCTL_PPCFG) & 1)) {
				dmc_io_or((0x3 << 9),
					  p_phy_reg + DDR_PUBL_DX2GCR);
				dmc_io_or((0x3 << 9),
					  p_phy_reg + DDR_PUBL_DX3GCR);
			}
		}
	} else {
		/* dynamic RTT disable */
		dmc_io_and(~(0x3 << 9), p_phy_reg + DDR_PUBL_DX0GCR);
		dmc_io_and(~(0x3 << 9), p_phy_reg + DDR_PUBL_DX1GCR);
		if (!(__raw_readl(p_ddr_reg + DDR_PCTL_PPCFG) & 1)) {
			dmc_io_and(~(0x3 << 9), p_phy_reg + DDR_PUBL_DX2GCR);
			dmc_io_and(~(0x3 << 9), p_phy_reg + DDR_PUBL_DX3GCR);
		}
	}
}

static void ddr_selfrefresh_enter(u32 nmhz)
{
	u32 ch;
	void __iomem *p_ddr_reg;

	for (ch = 0; ch < CH_MAX; ch++) {
		p_ddr_reg = ddr_ch[ch].p_ddr_reg;

		if (ddr_ch[ch].mem_type != DRAM_MAX) {
			ddr_move_to_lowpower_state(ch);
			__raw_writel(0x0, p_ddr_reg + DDR_PCTL_TZQCSI);
		}
	}
}

static bool cpu_pause[NR_CPUS];
static u32 major_cpu;

static bool is_cpu0_paused(unsigned int cpu)
{
	/* make sure cpu_pause[major_cpu] */
	smp_rmb();
	return cpu_pause[major_cpu];
}

static void set_cpux_paused(unsigned int cpu, bool pause)
{
	cpu_pause[cpu] = pause;
	/* Make sure cpu_pause[cpu] is written before next time use. */
	smp_wmb();
}

static bool is_cpux_paused(unsigned int cpu)
{
	/* make sure cpu_pause[cpu] */
	smp_rmb();
	return cpu_pause[cpu];
}

static void set_major_cpu(unsigned int cpu)
{
	major_cpu = cpu;
	/* Make sure major_cpu is written before next time use. */
	smp_wmb();
}

static u32 get_major_cpu(void)
{
	return major_cpu;
}

static void set_major_cpu_paused(unsigned int cpu, bool pause)
{
	cpu_pause[cpu] = pause;
	/* Make sure cpu_pause[cpu] is written before next time use. */
	smp_wmb();
}

static void pause_cpu_in_sram(void *arg)
{
	unsigned int cpu = (unsigned int)arg;

	set_cpux_paused(cpu, true);
	while (is_cpu0_paused(cpu))
		continue;
	set_cpux_paused(cpu, false);
}

static void ddr_adjust_config(u32 ch)
{
	void __iomem *p_ddr_reg = ddr_ch[ch].p_ddr_reg;
	void __iomem *p_phy_reg = ddr_ch[ch].p_phy_reg;
	u32 val;

	/* set data training address */
	__raw_writel(g_dmc_sram.dtar[ch], p_phy_reg + DDR_PUBL_DTAR);
	/* set auto power down idle */
	val = __raw_readl(p_ddr_reg + DDR_PCTL_MCFG);
	val = (val & 0xffff00ff) | (g_dmc_sram.pd_cnt << 8);
	__raw_writel(val, p_ddr_reg + DDR_PCTL_MCFG);
	/* CKDV=00 */
	dmc_io_and(~(0x3 << 12), p_phy_reg + DDR_PUBL_PGCR);
	/* enable the hardware low-power interface */
	dmc_io_or(HW_LOW_POWER_EN(1), p_ddr_reg + DDR_PCTL_SCFG);

	if (__raw_readl(p_ddr_reg + DDR_PCTL_PPCFG) & 1) {
		dmc_io_and(~(1), p_phy_reg + DDR_PUBL_DX2GCR);
		dmc_io_and(~(1), p_phy_reg + DDR_PUBL_DX3GCR);
		dmc_io_or(0x80000000, p_phy_reg + DDR_PUBL_DX2DLLCR);
		dmc_io_or(0x80000000, p_phy_reg + DDR_PUBL_DX3DLLCR);
	}
}

static void dmc_sram_init(struct rk3288_dmcclk *dmc)
{
	g_dmc_sram.cur_freq = dmc->cur_freq;
	g_dmc_sram.target_freq = dmc->target_freq;
	g_dmc_sram.freq_slew = dmc->freq_slew;
	g_dmc_sram.dqstr_value = dmc->dqstr_value;

	g_dmc_sram.channel_num = dmc->channel_num;
	g_dmc_sram.dtar[0] = dmc->dtar[0];
	g_dmc_sram.dtar[1] = dmc->dtar[1];
	g_dmc_sram.sram = dmc->sram;
	g_dmc_sram.ddr_type = dmc->ddr_type;
	g_dmc_sram.odt_disable_freq = dmc->oftimings.odt_disable_freq;
	g_dmc_sram.dll_disable_freq = dmc->oftimings.dll_disable_freq;

	if (dmc->target_freq <= dmc->oftimings.sr_enable_freq)
		g_dmc_sram.sr_cnt = dmc->oftimings.sr_cnt;
	else
		g_dmc_sram.sr_cnt = 0;

	if (dmc->target_freq <= dmc->oftimings.pd_enable_freq)
		g_dmc_sram.pd_cnt = dmc->oftimings.pd_cnt;
	else
		g_dmc_sram.pd_cnt = 0;

	g_dmc_sram.regtiming = dmc->regtiming;

	p_cru_reg = dmc->cru;
	p_grf_reg = dmc->grf;
	p_pmu_reg = dmc->pmu;
	ddr_ch[0].p_ddr_reg = dmc->ddr_regs[0];
	ddr_ch[1].p_ddr_reg = dmc->ddr_regs[1];
	ddr_ch[0].p_phy_reg = dmc->phy_regs[0];
	ddr_ch[1].p_phy_reg = dmc->phy_regs[1];
	ddr_ch[0].p_msch_reg = dmc->noc;
	ddr_ch[1].p_msch_reg = dmc->noc + 0x80;
	if (dmc->channel_num > 1) {
		ddr_ch[0].mem_type = dmc->ddr_type;
		ddr_ch[1].mem_type = dmc->ddr_type;
	} else {
		ddr_ch[0].mem_type = dmc->ddr_type;
		ddr_ch[1].mem_type = DRAM_MAX;
	}

	clkod = dmc->clkod;
	clkr = dmc->clkr;
	clkf = dmc->clkf;
}

static void dmc_set_rate_in_sram(void *arg)
{
	struct rk3288_dmcclk *dmc = (struct rk3288_dmcclk *)arg;
	u32 tmp, n, cs[CH_MAX], ch, deidle_req, clk_gate[CLKGATE_MAX_NUM];
	int retries = 0;
	bool success = false;

	dmc_sram_init(dmc);
	/* read the need register to confirm TLB cached */
	for (n = 0; n < (64 * 1024) / 4096; n++) {
		n = readl(g_dmc_sram.sram + n * 4096);
		barrier();
	}
	for (n = 0; n < g_dmc_sram.channel_num; n++) {
		tmp = __raw_readl(ddr_ch[n].p_ddr_reg + DDR_PCTL_SCFG);
		tmp = __raw_readl(ddr_ch[n].p_phy_reg + DDR_PUBL_RIDR);
		tmp = __raw_readl(ddr_ch[n].p_msch_reg + MSCH_DDRCONFIG);
	}
	n = __raw_readl(p_cru_reg + CRU_PLL_CON(0, 0));
	n = __raw_readl(p_pmu_reg + PMU_WAKEUP_CFG0);
	n = __raw_readl(p_grf_reg + GRF_SOC_CON0);

	/* ddr enter self-refresh mode or precharge power-down mode */
	deidle_req = idle_port(clk_gate);
	ddr_selfrefresh_enter(g_dmc_sram.target_freq);

	/* change frequence */
	rk3288_dmc_set_dpll(g_dmc_sram.target_freq);
	g_dmc_sram.cur_freq = g_dmc_sram.target_freq;

	while (!success) {
		/* Issues a Mode Exit command */
		for (ch = 0; ch < CH_MAX; ch++) {
			if (ddr_ch[ch].mem_type != DRAM_MAX) {
				ddr_set_dll_bypass(ch, g_dmc_sram.target_freq);
				ddr_reset_dll(ch);
				ddr_move_to_config_state(ch);
				ddr_update_timing(ch);
				ddr_update_mr(ch);
				ddr_update_odt(ch);
				ddr_adjust_config(ch);
				cs[ch] = ddr_data_training_trigger(ch);
			}
		}

		for (ch = 0; ch < CH_MAX; ch++) {
			if (ddr_ch[ch].mem_type != DRAM_MAX) {
				success = ddr_data_training(ch, cs[ch]);
				ddr_move_to_access_state(ch);
				if (!success) {
					retries++;
					break;
				}
			}
		}
	}

	deidle_port(clk_gate, deidle_req);
	dmc->cur_freq = g_dmc_sram.cur_freq;
	dmc->training_retries = retries;
}

static struct dmc_regtiming *dmc_get_regtiming_addr(void)
{
	return &g_ddr_timing;
}

static struct rockchip_dmc_sram_params dmc_sram_params;

struct rockchip_dmc_sram_params *__attribute__((section(".init")))init(void)
{
	dmc_sram_params.pause_cpu_in_sram = pause_cpu_in_sram;
	dmc_sram_params.set_major_cpu = set_major_cpu;
	dmc_sram_params.get_major_cpu = get_major_cpu;
	dmc_sram_params.set_major_cpu_paused = set_major_cpu_paused;
	dmc_sram_params.is_cpux_paused = is_cpux_paused;
	dmc_sram_params.dmc_set_rate_in_sram = dmc_set_rate_in_sram;
	dmc_sram_params.dmc_get_regtiming_addr = dmc_get_regtiming_addr;

	return &dmc_sram_params;
}
