/*
 **************************************************************************
 * Copyright (c) 2013,2015, The Linux Foundation. All rights reserved.
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 **************************************************************************
 */

/**
 * nss_hal_pvt.c
 *	NSS HAL private APIs.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/version.h>

#if (NSS_DT_SUPPORT != 1)
#include <mach/gpiomux.h>
#endif

#include "nss_hal_pvt.h"
#include "nss_clocks.h"
#include "nss_core.h"

/*
 * Global declarations
 */
extern struct nss_top_instance nss_top_main;

#if (NSS_FW_DBG_SUPPORT == 1)
/*
 * NSS debug pins configuration
 */

/*
 * Core 0, Data
 * No pull up, Function 2
 */
static struct gpiomux_setting nss_spi_data_0 = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

/*
 * Core 0, CLK, CS
 * Pull up high, Function 2
 */
static struct gpiomux_setting nss_spi_cs_clk_0 = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

/*
 * Core 1, CS
 * Pull up high, Function 4
 */
static struct gpiomux_setting nss_spi_cs_1 = {
	.func = GPIOMUX_FUNC_4,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

/*
 * Core 1, CLK
 * Pull up high, Function 5
 */
static struct gpiomux_setting nss_spi_clk_1 = {
	.func = GPIOMUX_FUNC_5,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

/*
 * Core 1, Data
 * Pull up none, Function 5
 */
static struct gpiomux_setting nss_spi_data_1 = {
	.func = GPIOMUX_FUNC_5,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config nss_spi_gpiomux[] = {
	{
		.gpio = 14,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_data_0,
			[GPIOMUX_SUSPENDED] = &nss_spi_data_0,
		},
	},
	{
		.gpio = 15,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_data_0,
			[GPIOMUX_SUSPENDED] = &nss_spi_data_0,
		},
	},
	{
		.gpio = 16,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_cs_clk_0,
			[GPIOMUX_SUSPENDED] = &nss_spi_cs_clk_0,
		},
	},
	{
		.gpio = 17,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_cs_clk_0,
			[GPIOMUX_SUSPENDED] = &nss_spi_cs_clk_0,
		},
	},
	{
		.gpio = 55,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_data_1,
			[GPIOMUX_SUSPENDED] = &nss_spi_data_1,
		},
	},
	{
		.gpio = 56,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_data_1,
			[GPIOMUX_SUSPENDED] = &nss_spi_data_1,
		},
	},
	{
		.gpio = 57,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_cs_1,
			[GPIOMUX_SUSPENDED] = &nss_spi_cs_1,
		},
	},
	{
		.gpio = 58,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_clk_1,
			[GPIOMUX_SUSPENDED] = &nss_spi_clk_1,
		},
	},
};
#endif /* NSS_FW_DBG_SUPPORT */

/*
 * __nss_hal_debug_enable()
 *	Enable NSS debug
 */
void __nss_hal_debug_enable(void)
{
#if (NSS_FW_DBG_SUPPORT == 1)
	msm_gpiomux_install(nss_spi_gpiomux,
				ARRAY_SIZE(nss_spi_gpiomux));
#endif
}

#if (NSS_DT_SUPPORT != 1)
/*
 * nss_hal_pvt_pll_change
 *	Change the Pll between 11(400mhz) or 18(1066 or 1466)
 */
void nss_hal_pvt_pll_change(uint32_t pll)
{
	uint32_t ctl_reg0;
	uint32_t ctl_reg1;

	uint32_t pll11_mask = 0x1;
	uint32_t pll18_mask = 0x0;

	uint32_t pll_cl_mask = 0x1;


	nss_trace("Picking PLL%d\n", pll);

	if (pll == 11) {

		ctl_reg0 = readl(UBI32_COREn_CLK_CTL(0));
		ctl_reg1 = readl(UBI32_COREn_CLK_CTL(1));

		ctl_reg0 &= ~pll_cl_mask;
		ctl_reg1 &= ~pll_cl_mask;

		ctl_reg0 |= pll11_mask;
		ctl_reg1 |= pll11_mask;

		writel(ctl_reg0, UBI32_COREn_CLK_CTL(0));
		writel(ctl_reg1, UBI32_COREn_CLK_CTL(1));

	} else if (pll == 18) {

		ctl_reg0 = readl(UBI32_COREn_CLK_CTL(0));
		ctl_reg1 = readl(UBI32_COREn_CLK_CTL(1));

		ctl_reg0 &= ~pll_cl_mask;
		ctl_reg1 &= ~pll_cl_mask;

		ctl_reg0 |= pll18_mask;
		ctl_reg1 |= pll18_mask;

		writel(ctl_reg0, UBI32_COREn_CLK_CTL(0));
		writel(ctl_reg1, UBI32_COREn_CLK_CTL(1));
	} else {
		BUG_ON(1);
	}

	return;
}

/*
 * nss_hal_pvt_register_dump
 *	Dump Registers Regarding NSS
 */
void nss_hal_pvt_register_dump(void) {
	nss_trace("NSSFB0_CLK_SRC_CTL	: %x\n", readl(NSSFB0_CLK_SRC_CTL));
	nss_trace("NSSFB1_CLK_SRC_CTL	: %x\n", readl(NSSFB1_CLK_SRC_CTL));
	nss_trace("NSSFB0_CLK_SRC0_NS	: %x\n", readl(NSSFB0_CLK_SRC0_NS));
	nss_trace("NSSFB0_CLK_SRC1_NS	: %x\n", readl(NSSFB0_CLK_SRC1_NS));
	nss_trace("NSSFB1_CLK_SRC0_NS	: %x\n", readl(NSSFB1_CLK_SRC0_NS));
	nss_trace("NSSFB1_CLK_SRC1_NS	: %x\n", readl(NSSFB1_CLK_SRC1_NS));
	nss_trace("\n");
	nss_trace("PLL_ENA_NSS	: %x\n", readl(PLL_ENA_NSS));
	nss_trace("PLL18_L_VAL	: %x\n", readl(PLL18_L_VAL));
	nss_trace("PLL18_M_VAL	: %x\n", readl(PLL18_M_VAL));
	nss_trace("PLL18_N_VAL	: %x\n", readl(PLL18_N_VAL));
	nss_trace("PLL18_CONFIG	: %x\n", readl(PLL18_CONFIG));
	nss_trace("PLL18_TEST_CTL: %x\n", readl(PLL18_TEST_CTL));
	nss_trace("\n");
	nss_trace("UBI32_COREn_CLK_SRC0_CTL Core 0: %x\n", readl(UBI32_COREn_CLK_SRC_CTL(0)));
	nss_trace("UBI32_COREn_CLK_SRC0_CTL Core 1: %x\n", readl(UBI32_COREn_CLK_SRC_CTL(1)));
	nss_trace("UBI32_COREn_CLK_SRC0_NS Core 0: %x\n", readl(UBI32_COREn_CLK_SRC0_NS(0)));
	nss_trace("UBI32_COREn_CLK_SRC0_NS Core 1: %x\n", readl(UBI32_COREn_CLK_SRC0_NS(1)));
	nss_trace("UBI32_COREn_CLK_SRC0_MD Core 0: %x\n", readl(UBI32_COREn_CLK_SRC0_MD(0)));
	nss_trace("UBI32_COREn_CLK_SRC0_MD Core 1: %x\n", readl(UBI32_COREn_CLK_SRC0_MD(1)));
	nss_trace("\n\n\n");
}

/*
 * nss_hal_pvt_divide_pll
 *	Divide PLL by int val
 */
uint32_t nss_hal_pvt_divide_pll18(uint32_t core_id, uint32_t divider)
{
	uint32_t ns_mask 	= 0x00ff01ff;
	uint32_t ns_mask_1	= 0x00ff0001;
	uint32_t ns_mask_2	= 0x00fe0141;
	uint32_t ns_mask_5 	= 0x00fb0141;
	uint32_t ns_reg0;
	uint32_t ns_reg1;

	uint32_t md_mask 	= 0x00ff00ff;
	uint32_t md_mask_2	= 0x000100fd;
	uint32_t md_mask_5 	= 0x000100fa;
	uint32_t md_reg0;
	uint32_t md_reg1;

	nss_trace("NSSFB0_CLK_SRC_CTL  : %x\n", readl(NSSFB0_CLK_SRC_CTL));
	nss_trace("NSSFB1_CLK_SRC_CTL  : %x\n", readl(NSSFB1_CLK_SRC_CTL));
	nss_trace("NSSFB0_CLK_SRC0_NS  : %x\n", readl(NSSFB0_CLK_SRC0_NS));
	nss_trace("NSSFB0_CLK_SRC1_NS  : %x\n", readl(NSSFB0_CLK_SRC1_NS));
	nss_trace("NSSFB1_CLK_SRC0_NS  : %x\n", readl(NSSFB1_CLK_SRC0_NS));
	nss_trace("NSSFB1_CLK_SRC1_NS  : %x\n", readl(NSSFB1_CLK_SRC1_NS));
	nss_trace("\n");
	nss_trace("PLL_ENA_NSS	: %x\n", readl(PLL_ENA_NSS));
	nss_trace("PLL18_L_VAL  : %x\n", readl(PLL18_L_VAL));
	nss_trace("PLL18_M_VAL  : %x\n", readl(PLL18_M_VAL));
	nss_trace("PLL18_N_VAL  : %x\n", readl(PLL18_N_VAL));
	nss_trace("PLL18_CONFIG : %x\n", readl(PLL18_CONFIG));
	nss_trace("PLL18_TEST_CTL: %x\n", readl(PLL18_TEST_CTL));
	nss_trace("\n");
	nss_trace("UBI32_COREn_CLK_SRC0_CTL Core 0: %x\n", readl(UBI32_COREn_CLK_SRC_CTL(0)));
	nss_trace("UBI32_COREn_CLK_SRC0_CTL Core 1: %x\n", readl(UBI32_COREn_CLK_SRC_CTL(1)));
	nss_trace("UBI32_COREn_CLK_SRC0_NS Core 0: %x\n", readl(UBI32_COREn_CLK_SRC0_NS(0)));
	nss_trace("UBI32_COREn_CLK_SRC0_NS Core 1: %x\n", readl(UBI32_COREn_CLK_SRC0_NS(1)));
	nss_trace("UBI32_COREn_CLK_SRC0_MD Core 0: %x\n", readl(UBI32_COREn_CLK_SRC0_MD(0)));
	nss_trace("UBI32_COREn_CLK_SRC0_MD Core 1: %x\n", readl(UBI32_COREn_CLK_SRC0_MD(1)));
	nss_trace("\n\n\n");


	md_reg0 = readl(UBI32_COREn_CLK_SRC0_MD(0));
	md_reg1 = readl(UBI32_COREn_CLK_SRC0_MD(1));
	ns_reg0 = readl(UBI32_COREn_CLK_SRC0_NS(0));
	ns_reg1 = readl(UBI32_COREn_CLK_SRC0_NS(1));

	/*
	 * Bypass
	 */
	if (divider == 1) {
		nss_trace("Bypass PLL Output\n");

		/*
		 * Clear M and D ( Not2*D ) and Set Bits
		 */

		md_reg0 &= ~md_mask;
		md_reg1 &= ~md_mask;

		/*
		 * PLL Source/ Pre Divide/ Counter Mode/ Counter Reset/ Counter Enable/ N Value
		 */

		ns_reg0 &= ~ns_mask;
		ns_reg1 &= ~ns_mask;

		ns_reg0 |= ns_mask_1;
		ns_reg1 |= ns_mask_1;
	} else if (divider == 2) {
		nss_trace("Divide PLL Output by 2\n");

		/*
		 * Clear M and D ( Not2*D ) and Set Bits
		 */

		md_reg0 &= ~md_mask;
		md_reg1 &= ~md_mask;

		md_reg0 |= md_mask_2;
		md_reg1 |= md_mask_2;

		/*
		 * PLL Source/ Pre Divide/ Counter Mode/ Counter Reset/ Counter Enable/ N Value
		 */

		ns_reg0 &= ~ns_mask;
		ns_reg1 &= ~ns_mask;

		ns_reg0 |= ns_mask_2;
		ns_reg1 |= ns_mask_2;
	} else if (divider == 5) {
		nss_trace("Divide PLL Output by 5\n");

		/*
		 * Clear M and D ( Not2*D ) and Set Bits
		 */

		md_reg0 &= ~md_mask;
		md_reg1 &= ~md_mask;

		md_reg0 |= md_mask_5;
		md_reg1 |= md_mask_5;

		/*
		 * PLL Source/ Pre Divide/ Counter Mode/ Counter Reset/ Counter Enable/ N Value
		 */

		ns_reg0 &= ~ns_mask;
		ns_reg1 &= ~ns_mask;

		ns_reg0 |= ns_mask_5;
		ns_reg1 |= ns_mask_5;
	} else {
		return 0;
	}

	writel(md_reg0, UBI32_COREn_CLK_SRC0_MD(0));
	writel(md_reg1, UBI32_COREn_CLK_SRC0_MD(1));
	writel(ns_reg0, UBI32_COREn_CLK_SRC0_NS(0));
	writel(ns_reg1, UBI32_COREn_CLK_SRC0_NS(1));

	nss_trace("UBI32_COREn_CLK_SRC0_CTL Core 0: %x\n", readl(UBI32_COREn_CLK_SRC_CTL(0)));
	nss_trace("UBI32_COREn_CLK_SRC0_CTL Core 1: %x\n", readl(UBI32_COREn_CLK_SRC_CTL(1)));
	nss_trace("UBI32_COREn_CLK_SRC0_NS Core 0: %x\n", readl(UBI32_COREn_CLK_SRC0_NS(0)));
	nss_trace("UBI32_COREn_CLK_SRC0_NS Core 1: %x\n", readl(UBI32_COREn_CLK_SRC0_NS(1)));
	nss_trace("UBI32_COREn_CLK_SRC0_MD Core 0: %x\n", readl(UBI32_COREn_CLK_SRC0_MD(0)));
	nss_trace("UBI32_COREn_CLK_SRC0_MD Core 1: %x\n", readl(UBI32_COREn_CLK_SRC0_MD(1)));

	return 1;
}

/*
 * nss_hal_pvt_enable_pll18()
 *	Enable PLL18
 */
uint32_t nss_hal_pvt_enable_pll18(uint32_t speed)
{
	uint32_t retries = 100;

	/*
	 * Prevent Compiler from commenting out the loop.
	 */
	uint32_t value;
	uint32_t mask = (1 << 2);

	/*
	 * Start with clean slate
	 */
	writel(0, PLL18_MODE);

	/*
	 * Effective VCO Frequency = 1100 MHz Post Divide 2
	 */
	if (speed == 1100) {
		writel(0x4000042C, PLL18_L_VAL);
		writel(0x0, PLL18_M_VAL);
		writel(0x1, PLL18_N_VAL);

		/*
		 * PLL configuration (as provided by HW team)
		 */
		writel(0x01495625, PLL18_CONFIG);
		writel(0x00003080, PLL18_TEST_CTL);
	} else if (speed == 1466) {
		/*
		 * Effective VCO Frequency = 1466 MHz Post Divide 2
		 */

		writel(0x4000043A, PLL18_L_VAL);
		writel(0x10, PLL18_M_VAL);
		writel(0x19, PLL18_N_VAL);

		/*
		 * PLL configuration (as provided by HW team)
		 */
		writel(0x014B5625, PLL18_CONFIG);
		writel(0x00003080, PLL18_TEST_CTL);
	} else {
		BUG_ON(1);
	}

	/*
	 * Enable PLL18 output (sequence provided by HW team)
	 */
	writel(0x2, PLL18_MODE);
	mdelay(1);
	writel(0x6, PLL18_MODE);
	writel(0x7, PLL18_MODE);

	/*
	 * Enable NSS Vote for PLL18.
	 */
	writel(mask, PLL_ENA_NSS);
	do {
		value = readl(PLL_LOCK_DET_STATUS);
		if (value & mask) {
			return PLL_LOCKED;
		}

		mdelay(1);
	} while (retries-- > 0);

	return PLL_NOT_LOCKED;
}


/*
 * __nss_hal_common_reset
 *	Do reset/clock configuration common to all cores
 */
void __nss_hal_common_reset(uint32_t *clk_src)
{
	uint32_t i;
	uint32_t value;
	uint32_t status_mask = 0x1;
	uint32_t wait_cycles = 100;

#if defined(NSS_ENABLE_CLK)

	/*
	 * NSS FPB CLOCK
	 */

	/*
	 * Enable clock root and Divider 0
	 * NOTE: Default value is good so no work here
	 */

	/*
	 * PLL0 (800 MHZ). SRC_SEL is 2 (3'b010)
	 * src_div selected is Div-6 (4'b0101).
	 *
	 * Effective frequency (Divider 0) = 133 MHz
	 */
	writel(0x2a, NSSFPB_CLK_SRC0_NS);

	/*
	 * Enable clock branch
	 */
	writel(0x50, NSSFPB_CLK_CTL);

	/*
	 * NSS FABRIC0 CLOCK
	 */

	/*
	 * Enable clock root and Divider 0
	 * NOTE: Default value is good so no work here
	 */

	/*
	 * PLL0 (800 MHZ) and div is set to 2.
	 * Effective frequency = 400 MHZ.
	 */
	writel(0x0a, NSSFB0_CLK_SRC0_NS);

	/*
	 * NSS Fabric0 Branch and dynamic clock gating enabled.
	 */
	writel(0x50, NSSFB0_CLK_CTL);

	/*
	 * Enable clock root and Divider 0
	 * NOTE: Default value is good so no work here
	 */

	/*
	 * PLL0 (800 MHZ) and div is set to 4.
	 * Effective frequency = 200 MHZ.
	 */
	writel(0x1a, NSSFB1_CLK_SRC0_NS);

	/*
	 * NSS Fabric1 Branch enable and fabric clock gating enabled.
	 */
	writel(0x50, NSSFB1_CLK_CTL);

	/*
	 * NSS TCM CLOCK
	 */

	/*
	 * Enable NSS TCM clock root source and select divider 0.
	 *
	 * NOTE: Default value is not good here
	 */
	writel(0x2, NSSTCM_CLK_SRC_CTL);

	/*
	 * PLL0 (800 MHZ) and div is set to 2.
	 * Effective frequency = 400 MHZ
	 */
	writel(0xa, NSSTCM_CLK_SRC0_NS);

	/*
	 * NSS TCM Branch enable and fabric clock gating enabled.
	 */
	writel(0x50, NSSTCM_CLK_CTL);

	/*
	 * Enable global NSS clock branches.
	 * NSS global Fab Branch enable and fabric clock gating enabled.
	 */
	writel(0xf, NSSFAB_GLOBAL_BUS_NS);

	/*
	 * Send reset interrupt to NSS
	 */
	writel(0x0, NSS_RESET);

	/*
	 * Enable PLL18
	 */
	pll18_status = nss_hal_pvt_enable_pll18();
	if (!pll18_status) {
		/*
		 * Select alternate good source (Src1/pll0)
		 */
		*clk_src = NSS_REGS_CLK_SRC_ALTERNATE;
		return;
	}

	/*
	 * Select default source (Src0/pll18)
	 */
	*clk_src = NSS_REGS_CLK_SRC_DEFAULT;
#endif

	/*
	 * Attach debug interface to TLMM
	 */
	nss_write_32((uint32_t)MSM_NSS_FPB_BASE, NSS_REGS_FPB_CSR_CFG_OFFSET, 0x360);

	/*
	 * NSS TCM CLOCK
	 */

	/*
	 * Enable NSS TCM clock root source - SRC1.
	 *
	 */
	writel(0x3, NSSTCM_CLK_SRC_CTL);

	/* Enable PLL Voting for 0 */
	writel((readl(PLL_ENA_NSS) | 0x1), PLL_ENA_NSS);
	do {
		value = readl(PLL_LOCK_DET_STATUS);
		if (value & status_mask) {
			break;
		}
		mdelay(1);
	} while (wait_cycles-- > 0);

	/*
	 * PLL0 (800 MHZ) and div is set to 3/4.
	 * Effective frequency = 266/400 Mhz for SRC0/1
	 */
	writel(0x12, NSSTCM_CLK_SRC0_NS);
	writel(0xa, NSSTCM_CLK_SRC1_NS);

	/*
	 * NSS TCM Branch enable and fabric clock gating enabled.
	 */
	writel(0x50, NSSTCM_CLK_CTL);

	/*
	 * Clear TCM memory
	 */
	for (i = 0; i < IPQ806X_NSS_TCM_SIZE; i += 4) {
		nss_write_32((uint32_t)MSM_NSS_TCM_BASE, i, 0);
	}

	return;
}
#endif /* NSS_DT_SUPPORT */

/*
 * __nss_hal_core_reset
 */
#if (NSS_DT_SUPPORT == 1)
void __nss_hal_core_reset(uint32_t map, uint32_t addr)
#else
void __nss_hal_core_reset(uint32_t core_id, uint32_t map, uint32_t addr, uint32_t clk_src)
#endif
{
#if (NSS_DT_SUPPORT != 1)
#if defined(NSS_ENABLE_CLOCK)
	/*
	 * Enable mpt clock
	 */
	writel(0x10, UBI32_MPT0_CLK_CTL);

	/*
	 * UBI coren clock root enable
	 */
	if (clk_src == NSS_REGS_CLK_SRC_DEFAULT) {
		/* select Src0 */
		writel(0x02, UBI32_COREn_CLK_SRC_CTL(core_id));
	} else {
		/* select Src1 */
		writel(0x03, UBI32_COREn_CLK_SRC_CTL(core_id));
	}

	/*
	 * Src0: Bypass M value configuration.
	 */

	/*
	 * Src1: M val is 0x01 and NOT_2D value is 0xfd, 400 MHz with PLL0.
	 */
	writel(0x100fd, UBI32_COREn_CLK_SRC1_MD(core_id));

	/*
	 * Bypass, pll18
	 * Effective frequency = 550 MHz
	 */
	writel(0x00000001, UBI32_COREn_CLK_SRC0_NS(core_id));

	/*
	 * Dual edge, pll0, NOT(N_M) = 0xfe.
	 * Effective frequency = 400 MHz
	 */
	writel(0x00fe0142, UBI32_COREn_CLK_SRC1_NS(core_id));

	/*
	 * UBI32 coren clock control branch.
	 */
	writel(0x4f, UBI32_COREn_CLK_FS(core_id));

	/*
	 * UBI32 coren clock control branch.
	 */
	writel(0x10, UBI32_COREn_CLK_CTL(core_id));
#endif
	/*
	 * Remove UBI32 reset clamp
	 */
	writel(0xB, UBI32_COREn_RESET_CLAMP(core_id));

	/*
	 * Busy wait for few cycles
	 */
	mdelay(1);

	/*
	 * Remove UBI32 core clamp
	 */
	writel(0x3, UBI32_COREn_RESET_CLAMP(core_id));

	mdelay(1);

	/*
	 * Remove UBI32 AHB reset
	 */
	writel(0x1, UBI32_COREn_RESET_CLAMP(core_id));

	mdelay(1);

	/*
	 * Remove UBI32 AXI reset
	 */
	writel(0x0, UBI32_COREn_RESET_CLAMP(core_id));

	mdelay(1);
#endif /* NSS_DT_SUPPORT */

	/*
	* Apply ubi32 core reset
	*/
	nss_write_32(map, NSS_REGS_RESET_CTRL_OFFSET, 1);

	/*
	 * Program address configuration
	 */
	nss_write_32(map, NSS_REGS_CORE_AMC_OFFSET, 1);
	nss_write_32(map, NSS_REGS_CORE_BAR_OFFSET, 0x3c000000);
	nss_write_32(map, NSS_REGS_CORE_BOOT_ADDR_OFFSET, addr);

	/*
	 * C2C interrupts are level sensitive
	 */
	nss_write_32(map, NSS_REGS_CORE_INT_STAT2_TYPE_OFFSET, 0xFFFF);

	/*
	 * Set IF check value
	 */
	nss_write_32(map, NSS_REGS_CORE_IFETCH_RANGE_OFFSET, 0xBF004001);

	/*
	 * De-assert ubi32 core reset
	 */
	nss_write_32(map, NSS_REGS_RESET_CTRL_OFFSET, 0);
}
