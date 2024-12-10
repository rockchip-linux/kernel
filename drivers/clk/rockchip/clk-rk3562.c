// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 Rockchip Electronics Co. Ltd.
 * Author: Elaine Zhang <zhangqing@rock-chips.com>
 * Author: Finley Xiao <finley.xiao@rock-chips.com>
 */

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#include <dt-bindings/clock/rk3562-cru.h>
#include "clk.h"

#define RK3562_GRF_SOC_STATUS0		0x430

enum rk3562_plls {
	apll, gpll, vpll, hpll, cpll, dpll,
};

static struct rockchip_pll_rate_table rk3562_pll_rates[] = {
	/* _mhz, _refdiv, _fbdiv, _postdiv1, _postdiv2, _dsmpd, _frac */
	RK3036_PLL_RATE(2208000000, 1, 92, 1, 1, 1, 0),
	RK3036_PLL_RATE(2184000000, 1, 91, 1, 1, 1, 0),
	RK3036_PLL_RATE(2160000000, 1, 90, 1, 1, 1, 0),
	RK3036_PLL_RATE(2088000000, 1, 87, 1, 1, 1, 0),
	RK3036_PLL_RATE(2064000000, 1, 86, 1, 1, 1, 0),
	RK3036_PLL_RATE(2040000000, 1, 85, 1, 1, 1, 0),
	RK3036_PLL_RATE(2016000000, 1, 84, 1, 1, 1, 0),
	RK3036_PLL_RATE(1992000000, 1, 83, 1, 1, 1, 0),
	RK3036_PLL_RATE(1920000000, 1, 80, 1, 1, 1, 0),
	RK3036_PLL_RATE(1896000000, 1, 79, 1, 1, 1, 0),
	RK3036_PLL_RATE(1800000000, 1, 75, 1, 1, 1, 0),
	RK3036_PLL_RATE(1704000000, 1, 71, 1, 1, 1, 0),
	RK3036_PLL_RATE(1608000000, 1, 67, 1, 1, 1, 0),
	RK3036_PLL_RATE(1600000000, 3, 200, 1, 1, 1, 0),
	RK3036_PLL_RATE(1584000000, 1, 132, 2, 1, 1, 0),
	RK3036_PLL_RATE(1560000000, 1, 130, 2, 1, 1, 0),
	RK3036_PLL_RATE(1536000000, 1, 128, 2, 1, 1, 0),
	RK3036_PLL_RATE(1512000000, 1, 126, 2, 1, 1, 0),
	RK3036_PLL_RATE(1488000000, 1, 124, 2, 1, 1, 0),
	RK3036_PLL_RATE(1464000000, 1, 122, 2, 1, 1, 0),
	RK3036_PLL_RATE(1440000000, 1, 120, 2, 1, 1, 0),
	RK3036_PLL_RATE(1416000000, 1, 118, 2, 1, 1, 0),
	RK3036_PLL_RATE(1400000000, 3, 350, 2, 1, 1, 0),
	RK3036_PLL_RATE(1392000000, 1, 116, 2, 1, 1, 0),
	RK3036_PLL_RATE(1368000000, 1, 114, 2, 1, 1, 0),
	RK3036_PLL_RATE(1344000000, 1, 112, 2, 1, 1, 0),
	RK3036_PLL_RATE(1320000000, 1, 110, 2, 1, 1, 0),
	RK3036_PLL_RATE(1296000000, 1, 108, 2, 1, 1, 0),
	RK3036_PLL_RATE(1272000000, 1, 106, 2, 1, 1, 0),
	RK3036_PLL_RATE(1248000000, 1, 104, 2, 1, 1, 0),
	RK3036_PLL_RATE(1200000000, 1, 100, 2, 1, 1, 0),
	RK3036_PLL_RATE(1188000000, 1, 99, 2, 1, 1, 0),
	RK3036_PLL_RATE(1104000000, 1, 92, 2, 1, 1, 0),
	RK3036_PLL_RATE(1100000000, 3, 275, 2, 1, 1, 0),
	RK3036_PLL_RATE(1008000000, 1, 84, 2, 1, 1, 0),
	RK3036_PLL_RATE(1000000000, 3, 250, 2, 1, 1, 0),
	RK3036_PLL_RATE(912000000, 1, 76, 2, 1, 1, 0),
	RK3036_PLL_RATE(816000000, 1, 68, 2, 1, 1, 0),
	RK3036_PLL_RATE(800000000, 3, 200, 2, 1, 1, 0),
	RK3036_PLL_RATE(700000000, 3, 350, 4, 1, 1, 0),
	RK3036_PLL_RATE(696000000, 1, 116, 4, 1, 1, 0),
	RK3036_PLL_RATE(600000000, 1, 100, 4, 1, 1, 0),
	RK3036_PLL_RATE(594000000, 1, 99, 4, 1, 1, 0),
	RK3036_PLL_RATE(500000000, 1, 125, 6, 1, 1, 0),
	RK3036_PLL_RATE(408000000, 1, 68, 2, 2, 1, 0),
	RK3036_PLL_RATE(312000000, 1, 78, 6, 1, 1, 0),
	RK3036_PLL_RATE(216000000, 1, 72, 4, 2, 1, 0),
	RK3036_PLL_RATE(200000000, 1, 100, 3, 4, 1, 0),
	RK3036_PLL_RATE(148500000, 1, 99, 4, 4, 1, 0),
	RK3036_PLL_RATE(100000000, 1, 150, 6, 6, 1, 0),
	RK3036_PLL_RATE(96000000, 1, 96, 6, 4, 1, 0),
	RK3036_PLL_RATE(74250000, 2, 99, 4, 4, 1, 0),
	{ /* sentinel */ },
};

PNAME(mux_pll_p)			= { "xin24m" };
PNAME(gpll_cpll_p)			= { "gpll", "cpll" };
PNAME(gpll_cpll_hpll_p)			= { "gpll", "cpll", "hpll" };
PNAME(gpll_cpll_pvtpll_dmyapll_p)	= { "gpll", "cpll", "log_pvtpll", "dummy_apll" };
PNAME(gpll_cpll_hpll_xin24m_p)		= { "gpll", "cpll", "hpll", "xin24m" };
PNAME(gpll_cpll_vpll_dmyhpll_p)		= { "gpll", "cpll", "vpll", "dummy_hpll" };
PNAME(gpll_dmyhpll_vpll_apll_p)		= { "gpll", "dummy_hpll", "vpll", "apll" };
PNAME(gpll_cpll_xin24m_p)		= { "gpll", "cpll", "xin24m" };
PNAME(gpll_cpll_xin24m_dmyapll_p)	= { "gpll", "cpll", "xin24m", "dummy_apll" };
PNAME(gpll_cpll_xin24m_dmyhpll_p)	= { "gpll", "cpll", "xin24m", "dummy_hpll" };
PNAME(vpll_dmyhpll_gpll_cpll_p)		= { "vpll", "dummy_hpll", "gpll", "cpll" };
PNAME(mux_xin24m_32k_p)			= { "xin24m", "clk_rtc_32k" };
PNAME(mux_50m_xin24m_p)			= { "clk_matrix_50m_src", "xin24m" };
PNAME(mux_100m_50m_xin24m_p)		= { "clk_matrix_100m_src", "clk_matrix_50m_src", "xin24m" };
PNAME(mux_125m_xin24m_p)		= { "clk_matrix_125m_src", "xin24m" };
PNAME(mux_200m_xin24m_32k_p)		= { "clk_200m_pmu", "xin24m", "clk_rtc_32k" };
PNAME(mux_200m_100m_p)			= { "clk_matrix_200m_src", "clk_matrix_100m_src" };
PNAME(mux_200m_100m_50m_xin24m_p)	= { "clk_matrix_200m_src", "clk_matrix_100m_src", "clk_matrix_50m_src", "xin24m" };
PNAME(clk_sai0_p)			= { "clk_sai0_src", "clk_sai0_frac", "xin_osc0_half", "mclk_sai0_from_io" };
PNAME(mclk_sai0_out2io_p)		= { "mclk_sai0", "xin_osc0_half" };
PNAME(clk_sai1_p)			= { "clk_sai1_src", "clk_sai1_frac", "xin_osc0_half", "mclk_sai1_from_io" };
PNAME(mclk_sai1_out2io_p)		= { "mclk_sai1", "xin_osc0_half" };
PNAME(clk_sai2_p)			= { "clk_sai2_src", "clk_sai2_frac", "xin_osc0_half", "mclk_sai2_from_io" };
PNAME(mclk_sai2_out2io_p)		= { "mclk_sai2", "xin_osc0_half" };
PNAME(clk_spdif_p)			= { "clk_spdif_src", "clk_spdif_frac", "xin_osc0_half" };
PNAME(clk_uart1_p)			= { "clk_uart1_src", "clk_uart1_frac", "xin24m" };
PNAME(clk_uart2_p)			= { "clk_uart2_src", "clk_uart2_frac", "xin24m" };
PNAME(clk_uart3_p)			= { "clk_uart3_src", "clk_uart3_frac", "xin24m" };
PNAME(clk_uart4_p)			= { "clk_uart4_src", "clk_uart4_frac", "xin24m" };
PNAME(clk_uart5_p)			= { "clk_uart5_src", "clk_uart5_frac", "xin24m" };
PNAME(clk_uart6_p)			= { "clk_uart6_src", "clk_uart6_frac", "xin24m" };
PNAME(clk_uart7_p)			= { "clk_uart7_src", "clk_uart7_frac", "xin24m" };
PNAME(clk_uart8_p)			= { "clk_uart8_src", "clk_uart8_frac", "xin24m" };
PNAME(clk_uart9_p)			= { "clk_uart9_src", "clk_uart9_frac", "xin24m" };
PNAME(clk_rtc32k_pmu_p)			= { "clk_rtc32k_frac", "xin32k", "clk_32k_pvtm" };
PNAME(clk_pmu1_uart0_p)			= { "clk_pmu1_uart0_src", "clk_pmu1_uart0_frac", "xin24m" };
PNAME(clk_pipephy_ref_p)		= { "clk_pipephy_div", "clk_pipephy_xin24m" };
PNAME(clk_usbphy_ref_p)			= { "clk_usb2phy_xin24m", "clk_24m_sscsrc" };
PNAME(clk_mipidsi_ref_p)		= { "clk_mipidsiphy_xin24m", "clk_24m_sscsrc" };

static struct rockchip_pll_clock rk3562_pll_clks[] __initdata = {
	[apll] = PLL(pll_rk3328, PLL_APLL, "apll", mux_pll_p,
		     0, RK3562_PLL_CON(0),
		     RK3562_MODE_CON, 0, 0,
		     ROCKCHIP_PLL_ALLOW_POWER_DOWN, rk3562_pll_rates),
	[gpll] = PLL(pll_rk3328, PLL_GPLL, "gpll", mux_pll_p,
		     0, RK3562_PLL_CON(24),
		     RK3562_MODE_CON, 2, 3, 0, rk3562_pll_rates),
	[vpll] = PLL(pll_rk3328, PLL_VPLL, "vpll", mux_pll_p,
		     0, RK3562_PLL_CON(32),
		     RK3562_MODE_CON, 6, 4,
		     ROCKCHIP_PLL_ALLOW_POWER_DOWN, rk3562_pll_rates),
	[hpll] = PLL(pll_rk3328, PLL_HPLL, "hpll", mux_pll_p,
		     0, RK3562_PLL_CON(40),
		     RK3562_MODE_CON, 8, 5,
		     ROCKCHIP_PLL_ALLOW_POWER_DOWN, rk3562_pll_rates),
	[cpll] = PLL(pll_rk3328, PLL_CPLL, "cpll", mux_pll_p,
		     0, RK3562_PMU1_PLL_CON(0),
		     RK3562_PMU1_MODE_CON, 0, 2, 0, rk3562_pll_rates),
	[dpll] = PLL(pll_rk3328, PLL_DPLL, "dpll", mux_pll_p,
		     CLK_IS_CRITICAL, RK3562_SUBDDR_PLL_CON(0),
		     RK3562_SUBDDR_MODE_CON, 0, 1, 0, NULL),
};

#define MFLAGS CLK_MUX_HIWORD_MASK
#define DFLAGS CLK_DIVIDER_HIWORD_MASK
#define GFLAGS (CLK_GATE_HIWORD_MASK | CLK_GATE_SET_TO_DISABLE)

static struct rockchip_clk_branch rk3562_clk_sai0_fracmux __initdata =
	MUX(CLK_SAI0, "clk_sai0", clk_sai0_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(3), 6, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_sai1_fracmux __initdata =
	MUX(CLK_SAI1, "clk_sai1", clk_sai1_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(5), 6, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_sai2_fracmux __initdata =
	MUX(CLK_SAI2, "clk_sai2", clk_sai2_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(8), 6, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_spdif_fracmux __initdata =
	MUX(CLK_SPDIF, "clk_spdif", clk_spdif_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(15), 6, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_uart1_fracmux __initdata =
	MUX(CLK_UART1, "clk_uart1", clk_uart1_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(21), 14, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_uart2_fracmux __initdata =
	MUX(CLK_UART2, "clk_uart2", clk_uart2_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(23), 14, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_uart3_fracmux __initdata =
	MUX(CLK_UART3, "clk_uart3", clk_uart3_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(25), 14, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_uart4_fracmux __initdata =
	MUX(CLK_UART4, "clk_uart4", clk_uart4_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(27), 14, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_uart5_fracmux __initdata =
	MUX(CLK_UART5, "clk_uart5", clk_uart5_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(29), 14, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_uart6_fracmux __initdata =
	MUX(CLK_UART6, "clk_uart6", clk_uart6_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(31), 14, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_uart7_fracmux __initdata =
	MUX(CLK_UART7, "clk_uart7", clk_uart7_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(33), 14, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_uart8_fracmux __initdata =
	MUX(CLK_UART8, "clk_uart8", clk_uart8_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(35), 14, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_uart9_fracmux __initdata =
	MUX(CLK_UART9, "clk_uart9", clk_uart9_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(37), 14, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_rtc32k_pmu_fracmux __initdata =
	MUX(CLK_RTC_32K, "clk_rtc_32k", clk_rtc32k_pmu_p, CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT,
			RK3562_PMU0_CLKSEL_CON(1), 0, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_pmu1_uart0_fracmux __initdata =
	MUX(CLK_PMU1_UART0, "clk_pmu1_uart0", clk_pmu1_uart0_p, CLK_SET_RATE_PARENT,
			RK3562_PMU1_CLKSEL_CON(2), 6, 2, MFLAGS);

static struct rockchip_clk_branch rk3562_clk_branches[] __initdata = {
	/*
	 * CRU Clock-Architecture
	 */
	/* PD_TOP */
	COMPOSITE(CLK_MATRIX_50M_SRC, "clk_matrix_50m_src", gpll_cpll_p, 0,
			RK3562_CLKSEL_CON(0), 7, 1, MFLAGS, 0, 5, DFLAGS,
			RK3562_CLKGATE_CON(0), 0, GFLAGS),
	COMPOSITE(CLK_MATRIX_100M_SRC, "clk_matrix_100m_src", gpll_cpll_p, CLK_IS_CRITICAL,
			RK3562_CLKSEL_CON(0), 15, 1, MFLAGS, 8, 4, DFLAGS,
			RK3562_CLKGATE_CON(0), 1, GFLAGS),
	COMPOSITE(CLK_MATRIX_125M_SRC, "clk_matrix_125m_src", gpll_cpll_p, 0,
			RK3562_CLKSEL_CON(1), 7, 1, MFLAGS, 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(0), 2, GFLAGS),
	COMPOSITE(CLK_MATRIX_200M_SRC, "clk_matrix_200m_src", gpll_cpll_p, CLK_IS_CRITICAL,
			RK3562_CLKSEL_CON(2), 7, 1, MFLAGS, 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(0), 4, GFLAGS),
	COMPOSITE(CLK_MATRIX_300M_SRC, "clk_matrix_300m_src", gpll_cpll_p, CLK_IS_CRITICAL,
			RK3562_CLKSEL_CON(3), 7, 1, MFLAGS, 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(0), 6, GFLAGS),
	COMPOSITE(ACLK_TOP, "aclk_top", gpll_cpll_p, CLK_IS_CRITICAL,
			RK3562_CLKSEL_CON(5), 7, 1, MFLAGS, 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(1), 0, GFLAGS),
	COMPOSITE(ACLK_TOP_VIO, "aclk_top_vio", gpll_cpll_p, 0,
			RK3562_CLKSEL_CON(5), 15, 1, MFLAGS, 8, 4, DFLAGS,
			RK3562_CLKGATE_CON(1), 1, GFLAGS),
	COMPOSITE(CLK_24M_SSCSRC, "clk_24m_sscsrc", vpll_dmyhpll_gpll_cpll_p, 0,
			RK3562_CLKSEL_CON(6), 6, 2, MFLAGS, 0, 6, DFLAGS,
			RK3562_CLKGATE_CON(1), 9, GFLAGS),
	COMPOSITE(CLK_CAM0_OUT2IO, "clk_cam0_out2io", gpll_cpll_xin24m_dmyapll_p, 0,
			RK3562_CLKSEL_CON(8), 6, 2, MFLAGS, 0, 6, DFLAGS,
			RK3562_CLKGATE_CON(1), 12, GFLAGS),
	COMPOSITE(CLK_CAM1_OUT2IO, "clk_cam1_out2io", gpll_cpll_xin24m_dmyapll_p, 0,
			RK3562_CLKSEL_CON(8), 14, 2, MFLAGS, 8, 6, DFLAGS,
			RK3562_CLKGATE_CON(1), 13, GFLAGS),
	COMPOSITE(CLK_CAM2_OUT2IO, "clk_cam2_out2io", gpll_cpll_xin24m_dmyapll_p, 0,
			RK3562_CLKSEL_CON(9), 6, 2, MFLAGS, 0, 6, DFLAGS,
			RK3562_CLKGATE_CON(1), 14, GFLAGS),
	COMPOSITE(CLK_CAM3_OUT2IO, "clk_cam3_out2io", gpll_cpll_xin24m_dmyapll_p, 0,
			RK3562_CLKSEL_CON(9), 14, 2, MFLAGS, 8, 6, DFLAGS,
			RK3562_CLKGATE_CON(1), 15, GFLAGS),
	FACTOR(0, "xin_osc0_half", "xin24m", 0, 1, 2),

	/* PD_BUS */
	COMPOSITE(ACLK_BUS, "aclk_bus", gpll_cpll_p, CLK_IS_CRITICAL,
			RK3562_CLKSEL_CON(40), 7, 1, MFLAGS, 0, 5, DFLAGS,
			RK3562_CLKGATE_CON(18), 0, GFLAGS),
	COMPOSITE(HCLK_BUS, "hclk_bus", gpll_cpll_p, CLK_IS_CRITICAL,
			RK3562_CLKSEL_CON(40), 15, 1, MFLAGS, 8, 6, DFLAGS,
			RK3562_CLKGATE_CON(18), 1, GFLAGS),
	COMPOSITE(PCLK_BUS, "pclk_bus", gpll_cpll_p, CLK_IS_CRITICAL,
			RK3562_CLKSEL_CON(41), 7, 1, MFLAGS, 0, 5, DFLAGS,
			RK3562_CLKGATE_CON(18), 2, GFLAGS),
	GATE(PCLK_I2C1, "pclk_i2c1", "pclk_bus", 0,
			RK3562_CLKGATE_CON(19), 0, GFLAGS),
	GATE(PCLK_I2C2, "pclk_i2c2", "pclk_bus", 0,
			RK3562_CLKGATE_CON(19), 1, GFLAGS),
	GATE(PCLK_I2C3, "pclk_i2c3", "pclk_bus", 0,
			RK3562_CLKGATE_CON(19), 2, GFLAGS),
	GATE(PCLK_I2C4, "pclk_i2c4", "pclk_bus", 0,
			RK3562_CLKGATE_CON(19), 3, GFLAGS),
	GATE(PCLK_I2C5, "pclk_i2c5", "pclk_bus", 0,
			RK3562_CLKGATE_CON(19), 4, GFLAGS),
	COMPOSITE_NODIV(CLK_I2C, "clk_i2c", mux_200m_100m_50m_xin24m_p, 0,
			RK3562_CLKSEL_CON(41), 8, 2, MFLAGS,
			RK3562_CLKGATE_CON(19), 5, GFLAGS),
	GATE(CLK_I2C1, "clk_i2c1", "clk_i2c", 0,
			RK3562_CLKGATE_CON(19), 6, GFLAGS),
	GATE(CLK_I2C2, "clk_i2c2", "clk_i2c", 0,
			RK3562_CLKGATE_CON(19), 7, GFLAGS),
	GATE(CLK_I2C3, "clk_i2c3", "clk_i2c", 0,
			RK3562_CLKGATE_CON(19), 8, GFLAGS),
	GATE(CLK_I2C4, "clk_i2c4", "clk_i2c", 0,
			RK3562_CLKGATE_CON(19), 9, GFLAGS),
	GATE(CLK_I2C5, "clk_i2c5", "clk_i2c", 0,
			RK3562_CLKGATE_CON(19), 10, GFLAGS),
	COMPOSITE_NODIV(DCLK_BUS_GPIO, "dclk_bus_gpio", mux_xin24m_32k_p, 0,
			RK3562_CLKSEL_CON(41), 15, 1, MFLAGS,
			RK3562_CLKGATE_CON(20), 4, GFLAGS),
	GATE(DCLK_BUS_GPIO3, "dclk_bus_gpio3", "dclk_bus_gpio", 0,
			RK3562_CLKGATE_CON(20), 5, GFLAGS),
	GATE(DCLK_BUS_GPIO4, "dclk_bus_gpio4", "dclk_bus_gpio", 0,
			RK3562_CLKGATE_CON(20), 6, GFLAGS),
	GATE(PCLK_TIMER, "pclk_timer", "pclk_bus", 0,
			RK3562_CLKGATE_CON(21), 0, GFLAGS),
	GATE(CLK_TIMER0, "clk_timer0", "xin24m", 0,
			RK3562_CLKGATE_CON(21), 1, GFLAGS),
	GATE(CLK_TIMER1, "clk_timer1", "xin24m", 0,
			RK3562_CLKGATE_CON(21), 2, GFLAGS),
	GATE(CLK_TIMER2, "clk_timer2", "xin24m", 0,
			RK3562_CLKGATE_CON(21), 3, GFLAGS),
	GATE(CLK_TIMER3, "clk_timer3", "xin24m", 0,
			RK3562_CLKGATE_CON(21), 4, GFLAGS),
	GATE(CLK_TIMER4, "clk_timer4", "xin24m", 0,
			RK3562_CLKGATE_CON(21), 5, GFLAGS),
	GATE(CLK_TIMER5, "clk_timer5", "xin24m", 0,
			RK3562_CLKGATE_CON(21), 6, GFLAGS),
	GATE(PCLK_STIMER, "pclk_stimer", "pclk_bus", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(21), 7, GFLAGS),
	GATE(CLK_STIMER0, "clk_stimer0", "xin24m", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(21), 8, GFLAGS),
	GATE(CLK_STIMER1, "clk_stimer1", "xin24m", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(21), 9, GFLAGS),
	GATE(PCLK_WDTNS, "pclk_wdtns", "pclk_bus", 0,
			RK3562_CLKGATE_CON(22), 0, GFLAGS),
	GATE(CLK_WDTNS, "clk_wdtns", "xin24m", 0,
			RK3562_CLKGATE_CON(22), 1, GFLAGS),
	GATE(PCLK_GRF, "pclk_grf", "pclk_bus", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(22), 2, GFLAGS),
	GATE(PCLK_SGRF, "pclk_sgrf", "pclk_bus", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(22), 3, GFLAGS),
	GATE(PCLK_MAILBOX, "pclk_mailbox", "pclk_bus", 0,
			RK3562_CLKGATE_CON(22), 4, GFLAGS),
	GATE(PCLK_INTC, "pclk_intc", "pclk_bus", 0,
			RK3562_CLKGATE_CON(22), 5, GFLAGS),
	GATE(ACLK_BUS_GIC400, "aclk_bus_gic400", "aclk_bus", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(22), 6, GFLAGS),
	GATE(ACLK_BUS_SPINLOCK, "aclk_bus_spinlock", "aclk_bus", 0,
			RK3562_CLKGATE_CON(23), 0, GFLAGS),
	GATE(ACLK_DCF, "aclk_dcf", "aclk_bus", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(23), 1, GFLAGS),
	GATE(PCLK_DCF, "pclk_dcf", "pclk_bus", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(23), 2, GFLAGS),
	GATE(FCLK_BUS_CM0_CORE, "fclk_bus_cm0_core", "hclk_bus", 0,
			RK3562_CLKGATE_CON(23), 3, GFLAGS),
	GATE(CLK_BUS_CM0_RTC, "clk_bus_cm0_rtc", "clk_rtc_32k", 0,
			RK3562_CLKGATE_CON(23), 4, GFLAGS),
	GATE(HCLK_ICACHE, "hclk_icache", "hclk_bus", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(23), 8, GFLAGS),
	GATE(HCLK_DCACHE, "hclk_dcache", "hclk_bus", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(23), 9, GFLAGS),
	GATE(PCLK_TSADC, "pclk_tsadc", "pclk_bus", 0,
			RK3562_CLKGATE_CON(24), 0, GFLAGS),
	COMPOSITE_NOMUX(CLK_TSADC, "clk_tsadc", "xin24m", 0,
			RK3562_CLKSEL_CON(43), 0, 11, DFLAGS,
			RK3562_CLKGATE_CON(24), 1, GFLAGS),
	COMPOSITE_NOMUX(CLK_TSADC_TSEN, "clk_tsadc_tsen", "xin24m", 0,
			RK3562_CLKSEL_CON(43), 11, 5, DFLAGS,
			RK3562_CLKGATE_CON(24), 3, GFLAGS),
	GATE(PCLK_DFT2APB, "pclk_dft2apb", "pclk_bus", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(24), 4, GFLAGS),
	COMPOSITE_NOMUX(CLK_SARADC_VCCIO156, "clk_saradc_vccio156", "xin24m", 0,
			RK3562_CLKSEL_CON(44), 0, 12, DFLAGS,
			RK3562_CLKGATE_CON(24), 9, GFLAGS),
	GATE(PCLK_GMAC, "pclk_gmac", "pclk_bus", 0,
			RK3562_CLKGATE_CON(25), 0, GFLAGS),
	GATE(ACLK_GMAC, "aclk_gmac", "aclk_bus", 0,
			RK3562_CLKGATE_CON(25), 1, GFLAGS),
	COMPOSITE_NODIV(CLK_GMAC_125M_CRU_I, "clk_gmac_125m_cru_i", mux_125m_xin24m_p, 0,
			RK3562_CLKSEL_CON(45), 8, 1, MFLAGS,
			RK3562_CLKGATE_CON(25), 2, GFLAGS),
	COMPOSITE_NODIV(CLK_GMAC_50M_CRU_I, "clk_gmac_50m_cru_i", mux_50m_xin24m_p, 0,
			RK3562_CLKSEL_CON(45), 7, 1, MFLAGS,
			RK3562_CLKGATE_CON(25), 3, GFLAGS),
	COMPOSITE(CLK_GMAC_ETH_OUT2IO, "clk_gmac_eth_out2io", gpll_cpll_p, 0,
			RK3562_CLKSEL_CON(46), 7, 1, MFLAGS, 0, 7, DFLAGS,
			RK3562_CLKGATE_CON(25), 4, GFLAGS),
	GATE(PCLK_APB2ASB_VCCIO156, "pclk_apb2asb_vccio156", "pclk_bus", CLK_IS_CRITICAL,
			RK3562_CLKGATE_CON(25), 5, GFLAGS),
	GATE(PCLK_TO_VCCIO156, "pclk_to_vccio156", "pclk_bus", CLK_IS_CRITICAL,
			RK3562_CLKGATE_CON(25), 6, GFLAGS),
	GATE(PCLK_DSIPHY, "pclk_dsiphy", "pclk_bus", 0,
			RK3562_CLKGATE_CON(25), 8, GFLAGS),
	GATE(PCLK_DSITX, "pclk_dsitx", "pclk_bus", 0,
			RK3562_CLKGATE_CON(25), 9, GFLAGS),
	GATE(PCLK_CPU_EMA_DET, "pclk_cpu_ema_det", "pclk_bus", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(25), 10, GFLAGS),
	GATE(PCLK_HASH, "pclk_hash", "pclk_bus", 0,
			RK3562_CLKGATE_CON(25), 11, GFLAGS),
	GATE(PCLK_TOPCRU, "pclk_topcru", "pclk_bus", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(25), 15, GFLAGS),
	GATE(PCLK_ASB2APB_VCCIO156, "pclk_asb2apb_vccio156", "pclk_to_vccio156", CLK_IS_CRITICAL,
			RK3562_CLKGATE_CON(26), 0, GFLAGS),
	GATE(PCLK_IOC_VCCIO156, "pclk_ioc_vccio156", "pclk_to_vccio156", CLK_IS_CRITICAL,
			RK3562_CLKGATE_CON(26), 1, GFLAGS),
	GATE(PCLK_GPIO3_VCCIO156, "pclk_gpio3_vccio156", "pclk_to_vccio156", 0,
			RK3562_CLKGATE_CON(26), 2, GFLAGS),
	GATE(PCLK_GPIO4_VCCIO156, "pclk_gpio4_vccio156", "pclk_to_vccio156", 0,
			RK3562_CLKGATE_CON(26), 3, GFLAGS),
	GATE(PCLK_SARADC_VCCIO156, "pclk_saradc_vccio156", "pclk_to_vccio156", 0,
			RK3562_CLKGATE_CON(26), 4, GFLAGS),
	GATE(PCLK_MAC100, "pclk_mac100", "pclk_bus", 0,
			RK3562_CLKGATE_CON(27), 0, GFLAGS),
	GATE(ACLK_MAC100, "aclk_mac100", "aclk_bus", 0,
			RK3562_CLKGATE_CON(27), 1, GFLAGS),
	COMPOSITE_NODIV(CLK_MAC100_50M_MATRIX, "clk_mac100_50m_matrix", mux_50m_xin24m_p, 0,
			RK3562_CLKSEL_CON(47), 7, 1, MFLAGS,
			RK3562_CLKGATE_CON(27), 2, GFLAGS),

	/* PD_CORE */
	COMPOSITE_NOMUX(0, "aclk_core_pre", "scmi_clk_cpu", CLK_IGNORE_UNUSED,
			RK3562_CLKSEL_CON(11), 0, 3, DFLAGS | CLK_DIVIDER_READ_ONLY,
			RK3562_CLKGATE_CON(4), 3, GFLAGS),
	COMPOSITE_NOMUX(0, "pclk_dbg_pre", "scmi_clk_cpu", CLK_IGNORE_UNUSED,
			RK3562_CLKSEL_CON(12), 0, 4, DFLAGS | CLK_DIVIDER_READ_ONLY,
			RK3562_CLKGATE_CON(4), 5, GFLAGS),
	COMPOSITE_NOMUX(HCLK_CORE, "hclk_core", "gpll", CLK_IS_CRITICAL,
			RK3562_CLKSEL_CON(13), 0, 6, DFLAGS,
			RK3562_CLKGATE_CON(5), 2, GFLAGS),
	GATE(0, "pclk_dbg_daplite", "pclk_dbg_pre", CLK_IGNORE_UNUSED,
			RK3562_CLKGATE_CON(4), 10, GFLAGS),

	/* PD_DDR */
	FACTOR_GATE(0, "clk_gpll_mux_to_ddr", "gpll", 0, 1, 4,
			RK3328_CLKGATE_CON(1), 6, GFLAGS),
	COMPOSITE_NOMUX(PCLK_DDR, "pclk_ddr", "clk_gpll_mux_to_ddr", CLK_IS_CRITICAL,
			RK3562_DDR_CLKSEL_CON(1), 8, 5, DFLAGS,
			RK3562_DDR_CLKGATE_CON(0), 3, GFLAGS),
	COMPOSITE_NOMUX(CLK_MSCH_BRG_BIU, "clk_msch_brg_biu", "clk_gpll_mux_to_ddr", CLK_IS_CRITICAL,
			RK3562_DDR_CLKSEL_CON(1), 0, 4, DFLAGS,
			RK3562_DDR_CLKGATE_CON(0), 4, GFLAGS),
	GATE(PCLK_DDR_HWLP, "pclk_ddr_hwlp", "pclk_ddr", CLK_IGNORE_UNUSED,
			RK3562_DDR_CLKGATE_CON(0), 6, GFLAGS),
	GATE(PCLK_DDR_UPCTL, "pclk_ddr_upctl", "pclk_ddr", CLK_IGNORE_UNUSED,
			RK3562_DDR_CLKGATE_CON(0), 7, GFLAGS),
	GATE(PCLK_DDR_PHY, "pclk_ddr_phy", "pclk_ddr", CLK_IGNORE_UNUSED,
			RK3562_DDR_CLKGATE_CON(0), 8, GFLAGS),
	GATE(PCLK_DDR_DFICTL, "pclk_ddr_dfictl", "pclk_ddr", CLK_IGNORE_UNUSED,
			RK3562_DDR_CLKGATE_CON(0), 9, GFLAGS),
	GATE(PCLK_DDR_DMA2DDR, "pclk_ddr_dma2ddr", "pclk_ddr", CLK_IGNORE_UNUSED,
			RK3562_DDR_CLKGATE_CON(0), 10, GFLAGS),
	GATE(PCLK_DDR_MON, "pclk_ddr_mon", "pclk_ddr", CLK_IGNORE_UNUSED,
			RK3562_DDR_CLKGATE_CON(1), 0, GFLAGS),
	GATE(TMCLK_DDR_MON, "tmclk_ddr_mon", "xin24m", CLK_IGNORE_UNUSED,
			RK3562_DDR_CLKGATE_CON(1), 1, GFLAGS),
	GATE(PCLK_DDR_GRF, "pclk_ddr_grf", "pclk_ddr", CLK_IGNORE_UNUSED,
			RK3562_DDR_CLKGATE_CON(1), 2, GFLAGS),
	GATE(PCLK_DDR_CRU, "pclk_ddr_cru", "pclk_ddr", CLK_IGNORE_UNUSED,
			RK3562_DDR_CLKGATE_CON(1), 3, GFLAGS),
	GATE(PCLK_SUBDDR_CRU, "pclk_subddr_cru", "pclk_ddr", CLK_IGNORE_UNUSED,
			RK3562_DDR_CLKGATE_CON(1), 4, GFLAGS),

	/* PD_GPU */
	COMPOSITE(CLK_GPU_PRE, "clk_gpu_pre", gpll_cpll_p, 0,
			RK3562_CLKSEL_CON(18), 7, 1, MFLAGS, 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(8), 0, GFLAGS),
	COMPOSITE_NOMUX(ACLK_GPU_PRE, "aclk_gpu_pre", "clk_gpu_pre", 0,
			RK3562_CLKSEL_CON(19), 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(8), 2, GFLAGS),
	GATE(CLK_GPU, "clk_gpu", "clk_gpu_pre", 0,
			RK3562_CLKGATE_CON(8), 4, GFLAGS),
	COMPOSITE_NODIV(CLK_GPU_BRG, "clk_gpu_brg", mux_200m_100m_p, 0,
			RK3562_CLKSEL_CON(19), 15, 1, MFLAGS,
			RK3562_CLKGATE_CON(8), 8, GFLAGS),

	/* PD_NPU */
	COMPOSITE(CLK_NPU_PRE, "clk_npu_pre", gpll_cpll_p, 0,
			RK3562_CLKSEL_CON(15), 7, 1, MFLAGS, 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(6), 0, GFLAGS),
	COMPOSITE_NOMUX(HCLK_NPU_PRE, "hclk_npu_pre", "clk_npu_pre", 0,
			RK3562_CLKSEL_CON(16), 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(6), 1, GFLAGS),
	GATE(ACLK_RKNN, "aclk_rknn", "clk_npu_pre", 0,
			RK3562_CLKGATE_CON(6), 4, GFLAGS),
	GATE(HCLK_RKNN, "hclk_rknn", "hclk_npu_pre", 0,
			RK3562_CLKGATE_CON(6), 5, GFLAGS),

	/* PD_PERI */
	COMPOSITE(ACLK_PERI, "aclk_peri", gpll_cpll_p, CLK_IS_CRITICAL,
			RK3562_PERI_CLKSEL_CON(0), 7, 1, MFLAGS, 0, 5, DFLAGS,
			RK3562_PERI_CLKGATE_CON(1), 0, GFLAGS),
	COMPOSITE(HCLK_PERI, "hclk_peri", gpll_cpll_p, CLK_IS_CRITICAL,
			RK3562_PERI_CLKSEL_CON(0), 15, 1, MFLAGS, 8, 6, DFLAGS,
			RK3562_PERI_CLKGATE_CON(1), 1, GFLAGS),
	COMPOSITE(PCLK_PERI, "pclk_peri", gpll_cpll_p, CLK_IS_CRITICAL,
			RK3562_PERI_CLKSEL_CON(1), 7, 1, MFLAGS, 0, 5, DFLAGS,
			RK3562_PERI_CLKGATE_CON(1), 2, GFLAGS),
	GATE(PCLK_PERICRU, "pclk_pericru", "pclk_peri", CLK_IGNORE_UNUSED,
			RK3562_PERI_CLKGATE_CON(1), 6, GFLAGS),
	GATE(HCLK_SAI0, "hclk_sai0", "hclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(2), 0, GFLAGS),
	COMPOSITE(CLK_SAI0_SRC, "clk_sai0_src", gpll_cpll_hpll_p, 0,
			RK3562_PERI_CLKSEL_CON(1), 14, 2, MFLAGS, 8, 6, DFLAGS,
			RK3562_PERI_CLKGATE_CON(2), 1, GFLAGS),
	COMPOSITE_FRACMUX(CLK_SAI0_FRAC, "clk_sai0_frac", "clk_sai0_src", CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(2), 0,
			RK3562_PERI_CLKGATE_CON(2), 2, GFLAGS,
			&rk3562_clk_sai0_fracmux),
	GATE(MCLK_SAI0, "mclk_sai0", "clk_sai0", 0,
			RK3562_PERI_CLKGATE_CON(2), 3, GFLAGS),
	COMPOSITE_NODIV(MCLK_SAI0_OUT2IO, "mclk_sai0_out2io", mclk_sai0_out2io_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(3), 5, 1, MFLAGS,
			RK3562_PERI_CLKGATE_CON(2), 4, GFLAGS),
	GATE(HCLK_SAI1, "hclk_sai1", "hclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(2), 5, GFLAGS),
	COMPOSITE(CLK_SAI1_SRC, "clk_sai1_src", gpll_cpll_hpll_p, 0,
			RK3562_PERI_CLKSEL_CON(3), 14, 2, MFLAGS, 8, 6, DFLAGS,
			RK3562_PERI_CLKGATE_CON(2), 6, GFLAGS),
	COMPOSITE_FRACMUX(CLK_SAI1_FRAC, "clk_sai1_frac", "clk_sai1_src", CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(4), 0,
			RK3562_PERI_CLKGATE_CON(2), 7, GFLAGS,
			&rk3562_clk_sai1_fracmux),
	GATE(MCLK_SAI1, "mclk_sai1", "clk_sai1", 0,
			RK3562_PERI_CLKGATE_CON(2), 8, GFLAGS),
	COMPOSITE_NODIV(MCLK_SAI1_OUT2IO, "mclk_sai1_out2io", mclk_sai1_out2io_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(5), 5, 1, MFLAGS,
			RK3562_PERI_CLKGATE_CON(2), 9, GFLAGS),
	GATE(HCLK_SAI2, "hclk_sai2", "hclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(2), 10, GFLAGS),
	COMPOSITE(CLK_SAI2_SRC, "clk_sai2_src", gpll_cpll_hpll_p, 0,
			RK3562_PERI_CLKSEL_CON(6), 14, 2, MFLAGS, 8, 6, DFLAGS,
			RK3562_PERI_CLKGATE_CON(2), 11, GFLAGS),
	COMPOSITE_FRACMUX(CLK_SAI2_FRAC, "clk_sai2_frac", "clk_sai2_src", CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(7), 0,
			RK3562_PERI_CLKGATE_CON(2), 12, GFLAGS,
			&rk3562_clk_sai2_fracmux),
	GATE(MCLK_SAI2, "mclk_sai2", "clk_sai2", 0,
			RK3562_PERI_CLKGATE_CON(2), 13, GFLAGS),
	COMPOSITE_NODIV(MCLK_SAI2_OUT2IO, "mclk_sai2_out2io", mclk_sai2_out2io_p, CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(8), 5, 1, MFLAGS,
			RK3562_PERI_CLKGATE_CON(2), 14, GFLAGS),
	GATE(HCLK_DSM, "hclk_dsm", "hclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(3), 1, GFLAGS),
	GATE(CLK_DSM, "clk_dsm", "mclk_sai1", 0,
			RK3562_PERI_CLKGATE_CON(3), 2, GFLAGS),
	GATE(HCLK_PDM, "hclk_pdm", "hclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(3), 4, GFLAGS),
	COMPOSITE(MCLK_PDM, "mclk_pdm", gpll_cpll_hpll_xin24m_p, 0,
			RK3562_PERI_CLKSEL_CON(12), 6, 2, MFLAGS, 0, 5, DFLAGS,
			RK3562_PERI_CLKGATE_CON(3), 5, GFLAGS),
	GATE(HCLK_SPDIF, "hclk_spdif", "hclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(3), 8, GFLAGS),
	COMPOSITE(CLK_SPDIF_SRC, "clk_spdif_src", gpll_cpll_hpll_p, 0,
			RK3562_PERI_CLKSEL_CON(13), 14, 2, MFLAGS, 8, 6, DFLAGS,
			RK3562_PERI_CLKGATE_CON(3), 9, GFLAGS),
	COMPOSITE_FRACMUX(CLK_SPDIF_FRAC, "clk_spdif_frac", "clk_spdif_src", CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(14), 0,
			RK3562_PERI_CLKGATE_CON(3), 10, GFLAGS,
			&rk3562_clk_spdif_fracmux),
	GATE(MCLK_SPDIF, "mclk_spdif", "clk_spdif", 0,
			RK3562_PERI_CLKGATE_CON(3), 11, GFLAGS),
	GATE(HCLK_SDMMC0, "hclk_sdmmc0", "hclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(4), 0, GFLAGS),
	COMPOSITE(CCLK_SDMMC0, "cclk_sdmmc0", gpll_cpll_xin24m_dmyhpll_p, 0,
			RK3562_PERI_CLKSEL_CON(16), 14, 2, MFLAGS, 0, 8, DFLAGS,
			RK3562_PERI_CLKGATE_CON(4), 1, GFLAGS),
	MMC(SCLK_SDMMC0_DRV, "sdmmc0_drv", "cclk_sdmmc0", RK3562_SDMMC0_CON0, 1),
	MMC(SCLK_SDMMC0_SAMPLE, "sdmmc0_sample", "cclk_sdmmc0", RK3562_SDMMC0_CON1, 1),
	GATE(HCLK_SDMMC1, "hclk_sdmmc1", "hclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(4), 2, GFLAGS),
	COMPOSITE(CCLK_SDMMC1, "cclk_sdmmc1", gpll_cpll_xin24m_dmyhpll_p, 0,
			RK3562_PERI_CLKSEL_CON(17), 14, 2, MFLAGS, 0, 8, DFLAGS,
			RK3562_PERI_CLKGATE_CON(4), 3, GFLAGS),
	MMC(SCLK_SDMMC1_DRV, "sdmmc1_drv", "cclk_sdmmc1", RK3562_SDMMC1_CON0, 1),
	MMC(SCLK_SDMMC1_SAMPLE, "sdmmc1_sample", "cclk_sdmmc1", RK3562_SDMMC1_CON1, 1),
	GATE(HCLK_EMMC, "hclk_emmc", "hclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(4), 8, GFLAGS),
	GATE(ACLK_EMMC, "aclk_emmc", "aclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(4), 9, GFLAGS),
	COMPOSITE(CCLK_EMMC, "cclk_emmc", gpll_cpll_xin24m_dmyhpll_p, 0,
			RK3562_PERI_CLKSEL_CON(18), 14, 2, MFLAGS, 0, 8, DFLAGS,
			RK3562_PERI_CLKGATE_CON(4), 10, GFLAGS),
	COMPOSITE(BCLK_EMMC, "bclk_emmc", gpll_cpll_p, 0,
			RK3562_PERI_CLKSEL_CON(19), 15, 1, MFLAGS, 8, 7, DFLAGS,
			RK3562_PERI_CLKGATE_CON(4), 11, GFLAGS),
	GATE(TMCLK_EMMC, "tmclk_emmc", "xin24m", 0,
			RK3562_PERI_CLKGATE_CON(4), 12, GFLAGS),
	COMPOSITE(SCLK_SFC, "sclk_sfc", gpll_cpll_xin24m_p, 0,
			RK3562_PERI_CLKSEL_CON(20), 8, 2, MFLAGS, 0, 8, DFLAGS,
			RK3562_PERI_CLKGATE_CON(4), 13, GFLAGS),
	GATE(HCLK_SFC, "hclk_sfc", "hclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(4), 14, GFLAGS),
	GATE(HCLK_USB2HOST, "hclk_usb2host", "hclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(5), 0, GFLAGS),
	GATE(HCLK_USB2HOST_ARB, "hclk_usb2host_arb", "hclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(5), 1, GFLAGS),
	GATE(PCLK_SPI1, "pclk_spi1", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(6), 0, GFLAGS),
	COMPOSITE_NODIV(CLK_SPI1, "clk_spi1", mux_200m_100m_50m_xin24m_p, 0,
			RK3562_PERI_CLKSEL_CON(20), 12, 2, MFLAGS,
			RK3562_PERI_CLKGATE_CON(6), 1, GFLAGS),
	GATE(SCLK_IN_SPI1, "sclk_in_spi1", "sclk_in_spi1_io", 0,
			RK3562_PERI_CLKGATE_CON(6), 2, GFLAGS),
	GATE(PCLK_SPI2, "pclk_spi2", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(6), 3, GFLAGS),
	COMPOSITE_NODIV(CLK_SPI2, "clk_spi2", mux_200m_100m_50m_xin24m_p, 0,
			RK3562_PERI_CLKSEL_CON(20), 14, 2, MFLAGS,
			RK3562_PERI_CLKGATE_CON(6), 4, GFLAGS),
	GATE(SCLK_IN_SPI2, "sclk_in_spi2", "sclk_in_spi2_io", 0,
			RK3562_PERI_CLKGATE_CON(6), 5, GFLAGS),
	GATE(PCLK_UART1, "pclk_uart1", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(7), 0, GFLAGS),
	GATE(PCLK_UART2, "pclk_uart2", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(7), 1, GFLAGS),
	GATE(PCLK_UART3, "pclk_uart3", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(7), 2, GFLAGS),
	GATE(PCLK_UART4, "pclk_uart4", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(7), 3, GFLAGS),
	GATE(PCLK_UART5, "pclk_uart5", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(7), 4, GFLAGS),
	GATE(PCLK_UART6, "pclk_uart6", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(7), 5, GFLAGS),
	GATE(PCLK_UART7, "pclk_uart7", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(7), 6, GFLAGS),
	GATE(PCLK_UART8, "pclk_uart8", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(7), 7, GFLAGS),
	GATE(PCLK_UART9, "pclk_uart9", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(7), 8, GFLAGS),
	COMPOSITE(CLK_UART1_SRC, "clk_uart1_src", gpll_cpll_p, 0,
			RK3562_PERI_CLKSEL_CON(21), 8, 1, MFLAGS, 0, 7, DFLAGS,
			RK3562_PERI_CLKGATE_CON(7), 9, GFLAGS),
	COMPOSITE_FRACMUX(CLK_UART1_FRAC, "clk_uart1_frac", "clk_uart1_src", CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(22), 0,
			RK3562_PERI_CLKGATE_CON(7), 10, GFLAGS,
			&rk3562_clk_uart1_fracmux),
	GATE(SCLK_UART1, "sclk_uart1", "clk_uart1", 0,
			RK3562_PERI_CLKGATE_CON(7), 11, GFLAGS),
	COMPOSITE(CLK_UART2_SRC, "clk_uart2_src", gpll_cpll_p, 0,
			RK3562_PERI_CLKSEL_CON(23), 8, 1, MFLAGS, 0, 7, DFLAGS,
			RK3562_PERI_CLKGATE_CON(7), 12, GFLAGS),
	COMPOSITE_FRACMUX(CLK_UART2_FRAC, "clk_uart2_frac", "clk_uart2_src", CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(24), 0,
			RK3562_PERI_CLKGATE_CON(7), 13, GFLAGS,
			&rk3562_clk_uart2_fracmux),
	GATE(SCLK_UART2, "sclk_uart2", "clk_uart2", 0,
			RK3562_PERI_CLKGATE_CON(7), 14, GFLAGS),
	COMPOSITE(CLK_UART3_SRC, "clk_uart3_src", gpll_cpll_p, 0,
			RK3562_PERI_CLKSEL_CON(25), 8, 1, MFLAGS, 0, 7, DFLAGS,
			RK3562_PERI_CLKGATE_CON(7), 15, GFLAGS),
	COMPOSITE_FRACMUX(CLK_UART3_FRAC, "clk_uart3_frac", "clk_uart3_src", CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(26), 0,
			RK3562_PERI_CLKGATE_CON(8), 0, GFLAGS,
			&rk3562_clk_uart3_fracmux),
	GATE(SCLK_UART3, "sclk_uart3", "clk_uart3", 0,
			RK3562_PERI_CLKGATE_CON(8), 1, GFLAGS),
	COMPOSITE(CLK_UART4_SRC, "clk_uart4_src", gpll_cpll_p, 0,
			RK3562_PERI_CLKSEL_CON(27), 8, 1, MFLAGS, 0, 7, DFLAGS,
			RK3562_PERI_CLKGATE_CON(8), 2, GFLAGS),
	COMPOSITE_FRACMUX(CLK_UART4_FRAC, "clk_uart4_frac", "clk_uart4_src", CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(28), 0,
			RK3562_PERI_CLKGATE_CON(8), 3, GFLAGS,
			&rk3562_clk_uart4_fracmux),
	GATE(SCLK_UART4, "sclk_uart4", "clk_uart4", 0,
			RK3562_PERI_CLKGATE_CON(8), 4, GFLAGS),
	COMPOSITE(CLK_UART5_SRC, "clk_uart5_src", gpll_cpll_p, 0,
			RK3562_PERI_CLKSEL_CON(29), 8, 1, MFLAGS, 0, 7, DFLAGS,
			RK3562_PERI_CLKGATE_CON(8), 5, GFLAGS),
	COMPOSITE_FRACMUX(CLK_UART5_FRAC, "clk_uart5_frac", "clk_uart5_src", CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(30), 0,
			RK3562_PERI_CLKGATE_CON(8), 6, GFLAGS,
			&rk3562_clk_uart5_fracmux),
	GATE(SCLK_UART5, "sclk_uart5", "clk_uart5", 0,
			RK3562_PERI_CLKGATE_CON(8), 7, GFLAGS),
	COMPOSITE(CLK_UART6_SRC, "clk_uart6_src", gpll_cpll_p, 0,
			RK3562_PERI_CLKSEL_CON(31), 8, 1, MFLAGS, 0, 7, DFLAGS,
			RK3562_PERI_CLKGATE_CON(8), 8, GFLAGS),
	COMPOSITE_FRACMUX(CLK_UART6_FRAC, "clk_uart6_frac", "clk_uart6_src", CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(32), 0,
			RK3562_PERI_CLKGATE_CON(8), 9, GFLAGS,
			&rk3562_clk_uart6_fracmux),
	GATE(SCLK_UART6, "sclk_uart6", "clk_uart6", 0,
			RK3562_PERI_CLKGATE_CON(8), 10, GFLAGS),
	COMPOSITE(CLK_UART7_SRC, "clk_uart7_src", gpll_cpll_p, 0,
			RK3562_PERI_CLKSEL_CON(33), 8, 1, MFLAGS, 0, 7, DFLAGS,
			RK3562_PERI_CLKGATE_CON(8), 11, GFLAGS),
	COMPOSITE_FRACMUX(CLK_UART7_FRAC, "clk_uart7_frac", "clk_uart7_src", CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(34), 0,
			RK3562_PERI_CLKGATE_CON(8), 12, GFLAGS,
			&rk3562_clk_uart7_fracmux),
	GATE(SCLK_UART7, "sclk_uart7", "clk_uart7", 0,
			RK3562_PERI_CLKGATE_CON(8), 13, GFLAGS),
	COMPOSITE(CLK_UART8_SRC, "clk_uart8_src", gpll_cpll_p, 0,
			RK3562_PERI_CLKSEL_CON(35), 8, 1, MFLAGS, 0, 7, DFLAGS,
			RK3562_PERI_CLKGATE_CON(8), 14, GFLAGS),
	COMPOSITE_FRACMUX(CLK_UART8_FRAC, "clk_uart8_frac", "clk_uart8_src", CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(36), 0,
			RK3562_PERI_CLKGATE_CON(8), 15, GFLAGS,
			&rk3562_clk_uart8_fracmux),
	GATE(SCLK_UART8, "sclk_uart8", "clk_uart8", 0,
			RK3562_PERI_CLKGATE_CON(9), 0, GFLAGS),
	COMPOSITE(CLK_UART9_SRC, "clk_uart9_src", gpll_cpll_p, 0,
			RK3562_PERI_CLKSEL_CON(37), 8, 1, MFLAGS, 0, 7, DFLAGS,
			RK3562_PERI_CLKGATE_CON(9), 1, GFLAGS),
	COMPOSITE_FRACMUX(CLK_UART9_FRAC, "clk_uart9_frac", "clk_uart9_src", CLK_SET_RATE_PARENT,
			RK3562_PERI_CLKSEL_CON(38), 0,
			RK3562_PERI_CLKGATE_CON(9), 2, GFLAGS,
			&rk3562_clk_uart9_fracmux),
	GATE(SCLK_UART9, "sclk_uart9", "clk_uart9", 0,
			RK3562_PERI_CLKGATE_CON(9), 3, GFLAGS),
	GATE(PCLK_PWM1_PERI, "pclk_pwm1_peri", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(10), 0, GFLAGS),
	COMPOSITE_NODIV(CLK_PWM1_PERI, "clk_pwm1_peri", mux_100m_50m_xin24m_p, 0,
			RK3562_PERI_CLKSEL_CON(40), 0, 2, MFLAGS,
			RK3562_PERI_CLKGATE_CON(10), 1, GFLAGS),
	GATE(CLK_CAPTURE_PWM1_PERI, "clk_capture_pwm1_peri", "xin24m", 0,
			RK3562_PERI_CLKGATE_CON(10), 2, GFLAGS),
	GATE(PCLK_PWM2_PERI, "pclk_pwm2_peri", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(10), 3, GFLAGS),
	COMPOSITE_NODIV(CLK_PWM2_PERI, "clk_pwm2_peri", mux_100m_50m_xin24m_p, 0,
			RK3562_PERI_CLKSEL_CON(40), 6, 2, MFLAGS,
			RK3562_PERI_CLKGATE_CON(10), 4, GFLAGS),
	GATE(CLK_CAPTURE_PWM2_PERI, "clk_capture_pwm2_peri", "xin24m", 0,
			RK3562_PERI_CLKGATE_CON(10), 5, GFLAGS),
	GATE(PCLK_PWM3_PERI, "pclk_pwm3_peri", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(10), 6, GFLAGS),
	COMPOSITE_NODIV(CLK_PWM3_PERI, "clk_pwm3_peri", mux_100m_50m_xin24m_p, 0,
			RK3562_PERI_CLKSEL_CON(40), 8, 2, MFLAGS,
			RK3562_PERI_CLKGATE_CON(10), 7, GFLAGS),
	GATE(CLK_CAPTURE_PWM3_PERI, "clk_capture_pwm3_peri", "xin24m", 0,
			RK3562_PERI_CLKGATE_CON(10), 8, GFLAGS),
	GATE(PCLK_CAN0, "pclk_can0", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(11), 0, GFLAGS),
	COMPOSITE(CLK_CAN0, "clk_can0", gpll_cpll_p, 0,
			RK3562_PERI_CLKSEL_CON(41), 7, 1, MFLAGS, 0, 5, DFLAGS,
			RK3562_PERI_CLKGATE_CON(11), 1, GFLAGS),
	GATE(PCLK_CAN1, "pclk_can1", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(11), 2, GFLAGS),
	COMPOSITE(CLK_CAN1, "clk_can1", gpll_cpll_p, 0,
			RK3562_PERI_CLKSEL_CON(41), 15, 1, MFLAGS, 8, 5, DFLAGS,
			RK3562_PERI_CLKGATE_CON(11), 3, GFLAGS),
	GATE(PCLK_PERI_WDT, "pclk_peri_wdt", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(13), 0, GFLAGS),
	COMPOSITE_NODIV(TCLK_PERI_WDT, "tclk_peri_wdt", mux_xin24m_32k_p, 0,
			RK3562_PERI_CLKSEL_CON(43), 15, 1, MFLAGS,
			RK3562_PERI_CLKGATE_CON(13), 1, GFLAGS),
	GATE(ACLK_SYSMEM, "aclk_sysmem", "aclk_peri", CLK_IGNORE_UNUSED,
			RK3562_PERI_CLKGATE_CON(13), 2, GFLAGS),
	GATE(HCLK_BOOTROM, "hclk_bootrom", "hclk_peri", CLK_IGNORE_UNUSED,
			RK3562_PERI_CLKGATE_CON(13), 3, GFLAGS),
	GATE(PCLK_PERI_GRF, "pclk_peri_grf", "pclk_peri", CLK_IGNORE_UNUSED,
			RK3562_PERI_CLKGATE_CON(13), 4, GFLAGS),
	GATE(ACLK_DMAC, "aclk_dmac", "aclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(13), 5, GFLAGS),
	GATE(ACLK_RKDMAC, "aclk_rkdmac", "aclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(13), 6, GFLAGS),
	GATE(PCLK_OTPC_NS, "pclk_otpc_ns", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(14), 0, GFLAGS),
	GATE(CLK_SBPI_OTPC_NS, "clk_sbpi_otpc_ns", "xin24m", 0,
			RK3562_PERI_CLKGATE_CON(14), 1, GFLAGS),
	COMPOSITE_NOMUX(CLK_USER_OTPC_NS, "clk_user_otpc_ns", "xin24m", 0,
			RK3562_PERI_CLKSEL_CON(44), 0, 8, DFLAGS,
			RK3562_PERI_CLKGATE_CON(14), 2, GFLAGS),
	GATE(PCLK_OTPC_S, "pclk_otpc_s", "pclk_peri", CLK_IGNORE_UNUSED,
			RK3562_PERI_CLKGATE_CON(14), 3, GFLAGS),
	GATE(CLK_SBPI_OTPC_S, "clk_sbpi_otpc_s", "xin24m", CLK_IGNORE_UNUSED,
			RK3562_PERI_CLKGATE_CON(14), 4, GFLAGS),
	COMPOSITE_NOMUX(CLK_USER_OTPC_S, "clk_user_otpc_s", "xin24m", CLK_IGNORE_UNUSED,
			RK3562_PERI_CLKSEL_CON(44), 8, 8, DFLAGS,
			RK3562_PERI_CLKGATE_CON(14), 5, GFLAGS),
	GATE(CLK_OTPC_ARB, "clk_otpc_arb", "xin24m", 0,
			RK3562_PERI_CLKGATE_CON(14), 6, GFLAGS),
	GATE(PCLK_OTPPHY, "pclk_otpphy", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(14), 7, GFLAGS),
	GATE(PCLK_USB2PHY, "pclk_usb2phy", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(15), 0, GFLAGS),
	GATE(PCLK_PIPEPHY, "pclk_pipephy", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(15), 7, GFLAGS),
	GATE(PCLK_SARADC, "pclk_saradc", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(16), 4, GFLAGS),
	COMPOSITE_NOMUX(CLK_SARADC, "clk_saradc", "xin24m", 0,
			RK3562_PERI_CLKSEL_CON(46), 0, 12, DFLAGS,
			RK3562_PERI_CLKGATE_CON(16), 5, GFLAGS),
	GATE(PCLK_IOC_VCCIO234, "pclk_ioc_vccio234", "pclk_peri", CLK_IS_CRITICAL,
			RK3562_PERI_CLKGATE_CON(16), 12, GFLAGS),
	GATE(PCLK_PERI_GPIO1, "pclk_peri_gpio1", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(17), 0, GFLAGS),
	GATE(PCLK_PERI_GPIO2, "pclk_peri_gpio2", "pclk_peri", 0,
			RK3562_PERI_CLKGATE_CON(17), 1, GFLAGS),
	COMPOSITE_NODIV(DCLK_PERI_GPIO, "dclk_peri_gpio", mux_xin24m_32k_p, 0,
			RK3562_PERI_CLKSEL_CON(47), 8, 1, MFLAGS,
			RK3562_PERI_CLKGATE_CON(17), 4, GFLAGS),
	GATE(DCLK_PERI_GPIO1, "dclk_peri_gpio1", "dclk_peri_gpio", 0,
			RK3562_PERI_CLKGATE_CON(17), 2, GFLAGS),
	GATE(DCLK_PERI_GPIO2, "dclk_peri_gpio2", "dclk_peri_gpio", 0,
			RK3562_PERI_CLKGATE_CON(17), 3, GFLAGS),

	/* PD_PHP */
	COMPOSITE(ACLK_PHP, "aclk_php", gpll_cpll_p, 0,
			RK3562_CLKSEL_CON(36), 7, 1, MFLAGS, 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(16), 0, GFLAGS),
	COMPOSITE_NOMUX(PCLK_PHP, "pclk_php", "aclk_php", 0,
			RK3562_CLKSEL_CON(36), 8, 4, DFLAGS,
			RK3562_CLKGATE_CON(16), 1, GFLAGS),
	GATE(ACLK_PCIE20_MST, "aclk_pcie20_mst", "aclk_php", 0,
			RK3562_CLKGATE_CON(16), 4, GFLAGS),
	GATE(ACLK_PCIE20_SLV, "aclk_pcie20_slv", "aclk_php", 0,
			RK3562_CLKGATE_CON(16), 5, GFLAGS),
	GATE(ACLK_PCIE20_DBI, "aclk_pcie20_dbi", "aclk_php", 0,
			RK3562_CLKGATE_CON(16), 6, GFLAGS),
	GATE(PCLK_PCIE20, "pclk_pcie20", "pclk_php", 0,
			RK3562_CLKGATE_CON(16), 7, GFLAGS),
	GATE(CLK_PCIE20_AUX, "clk_pcie20_aux", "xin24m", 0,
			RK3562_CLKGATE_CON(16), 8, GFLAGS),
	GATE(ACLK_USB3OTG, "aclk_usb3otg", "aclk_php", 0,
			RK3562_CLKGATE_CON(16), 10, GFLAGS),
	COMPOSITE_NODIV(CLK_USB3OTG_SUSPEND, "clk_usb3otg_suspend", mux_xin24m_32k_p, 0,
			RK3562_CLKSEL_CON(36), 15, 1, MFLAGS,
			RK3562_CLKGATE_CON(16), 11, GFLAGS),
	GATE(CLK_USB3OTG_REF, "clk_usb3otg_ref", "xin24m", 0,
			RK3562_CLKGATE_CON(16), 12, GFLAGS),
	GATE(CLK_PIPEPHY_REF_FUNC, "clk_pipephy_ref_func", "pclk_pcie20", 0,
			RK3562_CLKGATE_CON(17), 3, GFLAGS),

	/* PD_PMU1 */
	COMPOSITE_NOMUX(CLK_200M_PMU, "clk_200m_pmu", "cpll", CLK_IS_CRITICAL,
			RK3562_PMU1_CLKSEL_CON(0), 0, 5, DFLAGS,
			RK3562_PMU1_CLKGATE_CON(0), 1, GFLAGS),
	/* PD_PMU0 */
	COMPOSITE_FRACMUX(CLK_RTC32K_FRAC, "clk_rtc32k_frac", "xin24m", CLK_IS_CRITICAL,
			RK3562_PMU0_CLKSEL_CON(0), 0,
			RK3562_PMU0_CLKGATE_CON(0), 15, GFLAGS,
			&rk3562_rtc32k_pmu_fracmux),
	COMPOSITE_NOMUX(BUSCLK_PDPMU0, "busclk_pdpmu0", "clk_200m_pmu", CLK_IS_CRITICAL,
			RK3562_PMU0_CLKSEL_CON(1), 3, 2, DFLAGS,
			RK3562_PMU0_CLKGATE_CON(0), 14, GFLAGS),
	GATE(PCLK_PMU0_CRU, "pclk_pmu0_cru", "busclk_pdpmu0", CLK_IGNORE_UNUSED,
			RK3562_PMU0_CLKGATE_CON(0), 0, GFLAGS),
	GATE(PCLK_PMU0_PMU, "pclk_pmu0_pmu", "busclk_pdpmu0", CLK_IGNORE_UNUSED,
			RK3562_PMU0_CLKGATE_CON(0), 1, GFLAGS),
	GATE(CLK_PMU0_PMU, "clk_pmu0_pmu", "xin24m", CLK_IGNORE_UNUSED,
			RK3562_PMU0_CLKGATE_CON(0), 2, GFLAGS),
	GATE(PCLK_PMU0_HP_TIMER, "pclk_pmu0_hp_timer", "busclk_pdpmu0", CLK_IGNORE_UNUSED,
			RK3562_PMU0_CLKGATE_CON(0), 3, GFLAGS),
	GATE(CLK_PMU0_HP_TIMER, "clk_pmu0_hp_timer", "xin24m", CLK_IGNORE_UNUSED,
			RK3562_PMU0_CLKGATE_CON(0), 4, GFLAGS),
	GATE(CLK_PMU0_32K_HP_TIMER, "clk_pmu0_32k_hp_timer", "clk_rtc_32k", CLK_IGNORE_UNUSED,
			RK3562_PMU0_CLKGATE_CON(0), 5, GFLAGS),
	GATE(PCLK_PMU0_PVTM, "pclk_pmu0_pvtm", "busclk_pdpmu0", 0,
			RK3562_PMU0_CLKGATE_CON(0), 6, GFLAGS),
	GATE(CLK_PMU0_PVTM, "clk_pmu0_pvtm", "xin24m", 0,
			RK3562_PMU0_CLKGATE_CON(0), 7, GFLAGS),
	GATE(PCLK_IOC_PMUIO, "pclk_ioc_pmuio", "busclk_pdpmu0", CLK_IS_CRITICAL,
			RK3562_PMU0_CLKGATE_CON(0), 8, GFLAGS),
	GATE(PCLK_PMU0_GPIO0, "pclk_pmu0_gpio0", "busclk_pdpmu0", 0,
			RK3562_PMU0_CLKGATE_CON(0), 9, GFLAGS),
	GATE(DBCLK_PMU0_GPIO0, "dbclk_pmu0_gpio0", "xin24m", 0,
			RK3562_PMU0_CLKGATE_CON(0), 10, GFLAGS),
	GATE(PCLK_PMU0_GRF, "pclk_pmu0_grf", "busclk_pdpmu0", CLK_IGNORE_UNUSED,
			RK3562_PMU0_CLKGATE_CON(0), 11, GFLAGS),
	GATE(PCLK_PMU0_SGRF, "pclk_pmu0_sgrf", "busclk_pdpmu0", CLK_IGNORE_UNUSED,
			RK3562_PMU0_CLKGATE_CON(0), 12, GFLAGS),
	GATE(CLK_DDR_FAIL_SAFE, "clk_ddr_fail_safe", "xin24m", CLK_IGNORE_UNUSED,
			RK3562_PMU0_CLKGATE_CON(1), 0, GFLAGS),
	GATE(PCLK_PMU0_SCRKEYGEN, "pclk_pmu0_scrkeygen", "busclk_pdpmu0", CLK_IGNORE_UNUSED,
			RK3562_PMU0_CLKGATE_CON(1), 1, GFLAGS),
	COMPOSITE_NOMUX(CLK_PIPEPHY_DIV, "clk_pipephy_div", "cpll", 0,
			RK3562_PMU0_CLKSEL_CON(2), 0, 6, DFLAGS,
			RK3562_PMU0_CLKGATE_CON(2), 0, GFLAGS),
	GATE(CLK_PIPEPHY_XIN24M, "clk_pipephy_xin24m", "xin24m", 0,
			RK3562_PMU0_CLKGATE_CON(2), 1, GFLAGS),
	COMPOSITE_NODIV(CLK_PIPEPHY_REF, "clk_pipephy_ref", clk_pipephy_ref_p, 0,
			RK3562_PMU0_CLKSEL_CON(2), 7, 1, MFLAGS,
			RK3562_PMU0_CLKGATE_CON(2), 2, GFLAGS),
	GATE(CLK_USB2PHY_XIN24M, "clk_usb2phy_xin24m", "xin24m", 0,
			RK3562_PMU0_CLKGATE_CON(2), 4, GFLAGS),
	COMPOSITE_NODIV(CLK_USB2PHY_REF, "clk_usb2phy_ref", clk_usbphy_ref_p, 0,
			RK3562_PMU0_CLKSEL_CON(2), 8, 1, MFLAGS,
			RK3562_PMU0_CLKGATE_CON(2), 5, GFLAGS),
	GATE(CLK_MIPIDSIPHY_XIN24M, "clk_mipidsiphy_xin24m", "xin24m", 0,
			RK3562_PMU0_CLKGATE_CON(2), 6, GFLAGS),
	COMPOSITE_NODIV(CLK_MIPIDSIPHY_REF, "clk_mipidsiphy_ref", clk_mipidsi_ref_p, 0,
			RK3562_PMU0_CLKSEL_CON(2), 15, 1, MFLAGS,
			RK3562_PMU0_CLKGATE_CON(2), 7, GFLAGS),
	GATE(PCLK_PMU0_I2C0, "pclk_pmu0_i2c0", "busclk_pdpmu0", 0,
			RK3562_PMU0_CLKGATE_CON(2), 8, GFLAGS),
	COMPOSITE(CLK_PMU0_I2C0, "clk_pmu0_i2c0", mux_200m_xin24m_32k_p, 0,
			RK3562_PMU0_CLKSEL_CON(3), 14, 2, MFLAGS, 8, 5, DFLAGS,
			RK3562_PMU0_CLKGATE_CON(2), 9, GFLAGS),
	/* PD_PMU1 */
	GATE(PCLK_PMU1_CRU, "pclk_pmu1_cru", "busclk_pdpmu0", CLK_IGNORE_UNUSED,
			RK3562_PMU1_CLKGATE_CON(0), 0, GFLAGS),
	GATE(HCLK_PMU1_MEM, "hclk_pmu1_mem", "busclk_pdpmu0", CLK_IGNORE_UNUSED,
			RK3562_PMU1_CLKGATE_CON(0), 2, GFLAGS),
	GATE(PCLK_PMU1_UART0, "pclk_pmu1_uart0", "busclk_pdpmu0", 0,
			RK3562_PMU1_CLKGATE_CON(0), 7, GFLAGS),
	COMPOSITE_NOMUX(CLK_PMU1_UART0_SRC, "clk_pmu1_uart0_src", "cpll", 0,
			RK3562_PMU1_CLKSEL_CON(2), 0, 4, DFLAGS,
			RK3562_PMU1_CLKGATE_CON(0), 8, GFLAGS),
	COMPOSITE_FRACMUX(CLK_PMU1_UART0_FRAC, "clk_pmu1_uart0_frac", "clk_pmu1_uart0_src", CLK_SET_RATE_PARENT,
			RK3562_PMU1_CLKSEL_CON(3), 0,
			RK3562_PMU1_CLKGATE_CON(0), 9, GFLAGS,
			&rk3562_clk_pmu1_uart0_fracmux),
	GATE(SCLK_PMU1_UART0, "sclk_pmu1_uart0", "clk_pmu1_uart0", 0,
			RK3562_PMU1_CLKGATE_CON(0), 10, GFLAGS),
	GATE(PCLK_PMU1_SPI0, "pclk_pmu1_spi0", "busclk_pdpmu0", 0,
			RK3562_PMU1_CLKGATE_CON(1), 0, GFLAGS),
	COMPOSITE(CLK_PMU1_SPI0, "clk_pmu1_spi0", mux_200m_xin24m_32k_p, 0,
			RK3562_PMU1_CLKSEL_CON(4), 6, 2, MFLAGS, 0, 2, DFLAGS,
			RK3562_PMU1_CLKGATE_CON(1), 1, GFLAGS),
	GATE(SCLK_IN_PMU1_SPI0, "sclk_in_pmu1_spi0", "sclk_in_pmu1_spi0_io", 0,
			RK3562_PMU1_CLKGATE_CON(1), 2, GFLAGS),
	GATE(PCLK_PMU1_PWM0, "pclk_pmu1_pwm0", "busclk_pdpmu0", 0,
			RK3562_PMU1_CLKGATE_CON(1), 3, GFLAGS),
	COMPOSITE(CLK_PMU1_PWM0, "clk_pmu1_pwm0", mux_200m_xin24m_32k_p, 0,
			RK3562_PMU1_CLKSEL_CON(4), 14, 2, MFLAGS, 8, 2, DFLAGS,
			RK3562_PMU1_CLKGATE_CON(1), 4, GFLAGS),
	GATE(CLK_CAPTURE_PMU1_PWM0, "clk_capture_pmu1_pwm0", "xin24m", 0,
			RK3562_PMU1_CLKGATE_CON(1), 5, GFLAGS),
	GATE(CLK_PMU1_WIFI, "clk_pmu1_wifi", "xin24m", 0,
			RK3562_PMU1_CLKGATE_CON(1), 6, GFLAGS),
	GATE(FCLK_PMU1_CM0_CORE, "fclk_pmu1_cm0_core", "busclk_pdpmu0", 0,
			RK3562_PMU1_CLKGATE_CON(2), 0, GFLAGS),
	GATE(CLK_PMU1_CM0_RTC, "clk_pmu1_cm0_rtc", "clk_rtc_32k", 0,
			RK3562_PMU1_CLKGATE_CON(2), 1, GFLAGS),
	GATE(PCLK_PMU1_WDTNS, "pclk_pmu1_wdtns", "busclk_pdpmu0", 0,
			RK3562_PMU1_CLKGATE_CON(2), 3, GFLAGS),
	GATE(CLK_PMU1_WDTNS, "clk_pmu1_wdtns", "xin24m", 0,
			RK3562_PMU1_CLKGATE_CON(2), 4, GFLAGS),
	GATE(PCLK_PMU1_MAILBOX, "pclk_pmu1_mailbox", "busclk_pdpmu0", 0,
			RK3562_PMU1_CLKGATE_CON(3), 8, GFLAGS),

	/* PD_RGA */
	COMPOSITE(ACLK_RGA_PRE, "aclk_rga_pre", gpll_cpll_pvtpll_dmyapll_p, 0,
			RK3562_CLKSEL_CON(32), 6, 2, MFLAGS, 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(14), 0, GFLAGS),
	COMPOSITE_NOMUX(HCLK_RGA_PRE, "hclk_rga_pre", "aclk_rga_jdec", 0,
			RK3562_CLKSEL_CON(32), 8, 3, DFLAGS,
			RK3562_CLKGATE_CON(14), 1, GFLAGS),
	GATE(ACLK_RGA, "aclk_rga", "aclk_rga_jdec", 0,
			RK3562_CLKGATE_CON(14), 6, GFLAGS),
	GATE(HCLK_RGA, "hclk_rga", "hclk_rga_pre", 0,
			RK3562_CLKGATE_CON(14), 7, GFLAGS),
	COMPOSITE(CLK_RGA_CORE, "clk_rga_core", gpll_cpll_pvtpll_dmyapll_p, 0,
			RK3562_CLKSEL_CON(33), 6, 2, MFLAGS, 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(14), 8, GFLAGS),
	GATE(ACLK_JDEC, "aclk_jdec", "aclk_rga_jdec", 0,
			RK3562_CLKGATE_CON(14), 9, GFLAGS),
	GATE(HCLK_JDEC, "hclk_jdec", "hclk_rga_pre", 0,
			RK3562_CLKGATE_CON(14), 10, GFLAGS),

	/* PD_VDPU */
	COMPOSITE(ACLK_VDPU_PRE, "aclk_vdpu_pre", gpll_cpll_pvtpll_dmyapll_p, 0,
			RK3562_CLKSEL_CON(22), 6, 2, MFLAGS, 0, 5, DFLAGS,
			RK3562_CLKGATE_CON(10), 0, GFLAGS),
	COMPOSITE(CLK_RKVDEC_HEVC_CA, "clk_rkvdec_hevc_ca", gpll_cpll_pvtpll_dmyapll_p, 0,
			RK3562_CLKSEL_CON(23), 14, 2, MFLAGS, 8, 5, DFLAGS,
			RK3562_CLKGATE_CON(10), 3, GFLAGS),
	COMPOSITE_NOMUX(HCLK_VDPU_PRE, "hclk_vdpu_pre", "aclk_vdpu", 0,
			RK3562_CLKSEL_CON(24), 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(10), 4, GFLAGS),
	GATE(ACLK_RKVDEC, "aclk_rkvdec", "aclk_vdpu", 0,
			RK3562_CLKGATE_CON(10), 7, GFLAGS),
	GATE(HCLK_RKVDEC, "hclk_rkvdec", "hclk_vdpu_pre", 0,
			RK3562_CLKGATE_CON(10), 8, GFLAGS),

	/* PD_VEPU */
	COMPOSITE(CLK_RKVENC_CORE, "clk_rkvenc_core", gpll_cpll_pvtpll_dmyapll_p, 0,
			RK3562_CLKSEL_CON(20), 6, 2, MFLAGS, 0, 5, DFLAGS,
			RK3562_CLKGATE_CON(9), 0, GFLAGS),
	COMPOSITE(ACLK_VEPU_PRE, "aclk_vepu_pre", gpll_cpll_pvtpll_dmyapll_p, 0,
			RK3562_CLKSEL_CON(20), 14, 2, MFLAGS, 8, 5, DFLAGS,
			RK3562_CLKGATE_CON(9), 1, GFLAGS),
	COMPOSITE_NOMUX(HCLK_VEPU_PRE, "hclk_vepu_pre", "aclk_vepu", 0,
			RK3562_CLKSEL_CON(21), 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(9), 2, GFLAGS),
	GATE(ACLK_RKVENC, "aclk_rkvenc", "aclk_vepu", 0,
			RK3562_CLKGATE_CON(9), 5, GFLAGS),
	GATE(HCLK_RKVENC, "hclk_rkvenc", "hclk_vepu", 0,
			RK3562_CLKGATE_CON(9), 6, GFLAGS),

	/* PD_VI */
	COMPOSITE(ACLK_VI, "aclk_vi", gpll_cpll_pvtpll_dmyapll_p, 0,
			RK3562_CLKSEL_CON(25), 6, 2, MFLAGS, 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(11), 0, GFLAGS),
	COMPOSITE_NOMUX(HCLK_VI, "hclk_vi", "aclk_vi_isp", 0,
			RK3562_CLKSEL_CON(26), 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(11), 1, GFLAGS),
	COMPOSITE_NOMUX(PCLK_VI, "pclk_vi", "aclk_vi_isp", 0,
			RK3562_CLKSEL_CON(26), 8, 4, DFLAGS,
			RK3562_CLKGATE_CON(11), 2, GFLAGS),
	GATE(ACLK_ISP, "aclk_isp", "aclk_vi_isp", 0,
			RK3562_CLKGATE_CON(11), 6, GFLAGS),
	GATE(HCLK_ISP, "hclk_isp", "hclk_vi", 0,
			RK3562_CLKGATE_CON(11), 7, GFLAGS),
	COMPOSITE(CLK_ISP, "clk_isp", gpll_cpll_pvtpll_dmyapll_p, 0,
			RK3562_CLKSEL_CON(27), 6, 2, MFLAGS, 0, 4, DFLAGS,
			RK3562_CLKGATE_CON(11), 8, GFLAGS),
	GATE(ACLK_VICAP, "aclk_vicap", "aclk_vi_isp", 0,
			RK3562_CLKGATE_CON(11), 9, GFLAGS),
	GATE(HCLK_VICAP, "hclk_vicap", "hclk_vi", 0,
			RK3562_CLKGATE_CON(11), 10, GFLAGS),
	COMPOSITE(DCLK_VICAP, "dclk_vicap", gpll_cpll_pvtpll_dmyapll_p, 0,
			RK3562_CLKSEL_CON(27), 14, 2, MFLAGS, 8, 4, DFLAGS,
			RK3562_CLKGATE_CON(11), 11, GFLAGS),
	GATE(CSIRX0_CLK_DATA, "csirx0_clk_data", "csirx0_clk_data_io", 0,
			RK3562_CLKGATE_CON(11), 12, GFLAGS),
	GATE(CSIRX1_CLK_DATA, "csirx1_clk_data", "csirx1_clk_data_io", 0,
			RK3562_CLKGATE_CON(11), 13, GFLAGS),
	GATE(CSIRX2_CLK_DATA, "csirx2_clk_data", "csirx2_clk_data_io", 0,
			RK3562_CLKGATE_CON(11), 14, GFLAGS),
	GATE(CSIRX3_CLK_DATA, "csirx3_clk_data", "csirx3_clk_data_io", 0,
			RK3562_CLKGATE_CON(11), 15, GFLAGS),
	GATE(PCLK_CSIHOST0, "pclk_csihost0", "pclk_vi", 0,
			RK3562_CLKGATE_CON(12), 0, GFLAGS),
	GATE(PCLK_CSIHOST1, "pclk_csihost1", "pclk_vi", 0,
			RK3562_CLKGATE_CON(12), 1, GFLAGS),
	GATE(PCLK_CSIHOST2, "pclk_csihost2", "pclk_vi", 0,
			RK3562_CLKGATE_CON(12), 2, GFLAGS),
	GATE(PCLK_CSIHOST3, "pclk_csihost3", "pclk_vi", 0,
			RK3562_CLKGATE_CON(12), 3, GFLAGS),
	GATE(PCLK_CSIPHY0, "pclk_csiphy0", "pclk_vi", 0,
			RK3562_CLKGATE_CON(12), 4, GFLAGS),
	GATE(PCLK_CSIPHY1, "pclk_csiphy1", "pclk_vi", 0,
			RK3562_CLKGATE_CON(12), 5, GFLAGS),

	/* PD_VO */
	COMPOSITE(ACLK_VO_PRE, "aclk_vo_pre", gpll_cpll_vpll_dmyhpll_p, 0,
			RK3562_CLKSEL_CON(28), 6, 2, MFLAGS, 0, 5, DFLAGS,
			RK3562_CLKGATE_CON(13), 0, GFLAGS),
	COMPOSITE_NOMUX(HCLK_VO_PRE, "hclk_vo_pre", "aclk_vo", 0,
			RK3562_CLKSEL_CON(29), 0, 5, DFLAGS,
			RK3562_CLKGATE_CON(13), 1, GFLAGS),
	GATE(ACLK_VOP, "aclk_vop", "aclk_vo", 0,
			RK3562_CLKGATE_CON(13), 6, GFLAGS),
	GATE(HCLK_VOP, "hclk_vop", "hclk_vo_pre", 0,
			RK3562_CLKGATE_CON(13), 7, GFLAGS),
	COMPOSITE(DCLK_VOP, "dclk_vop", gpll_dmyhpll_vpll_apll_p, CLK_SET_RATE_NO_REPARENT,
			RK3562_CLKSEL_CON(30), 14, 2, MFLAGS, 0, 8, DFLAGS,
			RK3562_CLKGATE_CON(13), 8, GFLAGS),
	COMPOSITE(DCLK_VOP1, "dclk_vop1", gpll_dmyhpll_vpll_apll_p, CLK_SET_RATE_NO_REPARENT,
			RK3562_CLKSEL_CON(31), 14, 2, MFLAGS, 0, 8, DFLAGS,
			RK3562_CLKGATE_CON(13), 9, GFLAGS),
};

static void __iomem *rk3562_cru_base;

static void rk3562_dump_cru(void)
{
	if (rk3562_cru_base) {
		pr_warn("CRU:\n");
		print_hex_dump(KERN_WARNING, "", DUMP_PREFIX_OFFSET,
			       32, 4, rk3562_cru_base,
			       0x600, false);
	}
}

static int protect_clocks[] = {
	ACLK_VO_PRE,
	HCLK_VO_PRE,
	ACLK_VOP,
	HCLK_VOP,
	DCLK_VOP,
	DCLK_VOP1,
};

static void __init rk3562_clk_init(struct device_node *np)
{
	struct rockchip_clk_provider *ctx;
	void __iomem *reg_base;

	reg_base = of_iomap(np, 0);
	if (!reg_base) {
		pr_err("%s: could not map cru region\n", __func__);
		return;
	}

	rk3562_cru_base = reg_base;

	ctx = rockchip_clk_init(np, reg_base, CLK_NR_CLKS);
	if (IS_ERR(ctx)) {
		pr_err("%s: rockchip clk init failed\n", __func__);
		iounmap(reg_base);
		return;
	}

	rockchip_clk_register_plls(ctx, rk3562_pll_clks,
				   ARRAY_SIZE(rk3562_pll_clks),
				   RK3562_GRF_SOC_STATUS0);

	rockchip_clk_register_branches(ctx, rk3562_clk_branches,
				       ARRAY_SIZE(rk3562_clk_branches));

	/* (0x30444 - 0x400) / 4 + 1 = 49170 */
	rockchip_register_softrst(np, 49170, reg_base + RK3562_SOFTRST_CON(0),
				  ROCKCHIP_SOFTRST_HIWORD_MASK);

	rockchip_register_restart_notifier(ctx, RK3562_GLB_SRST_FST, NULL);

	rockchip_clk_of_add_provider(np, ctx);

	if (!rk_dump_cru)
		rk_dump_cru = rk3562_dump_cru;

	rockchip_clk_protect(ctx, protect_clocks, ARRAY_SIZE(protect_clocks));
}

CLK_OF_DECLARE(rk3562_cru, "rockchip,rk3562-cru", rk3562_clk_init);

#ifdef MODULE
struct clk_rk3562_inits {
	void (*inits)(struct device_node *np);
};

static const struct clk_rk3562_inits clk_3562_cru_init = {
	.inits = rk3562_clk_init,
};

static const struct of_device_id clk_rk3562_match_table[] = {
	{
		.compatible = "rockchip,rk3562-cru",
		.data = &clk_3562_cru_init,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, clk_rk3562_match_table);

static int clk_rk3562_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	const struct clk_rk3562_inits *init_data;

	match = of_match_device(clk_rk3562_match_table, &pdev->dev);
	if (!match || !match->data)
		return -EINVAL;

	init_data = match->data;
	if (init_data->inits)
		init_data->inits(np);

	return 0;
}

static struct platform_driver clk_rk3562_driver = {
	.probe		= clk_rk3562_probe,
	.driver		= {
		.name	= "clk-rk3562",
		.of_match_table = clk_rk3562_match_table,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(clk_rk3562_driver);

MODULE_DESCRIPTION("Rockchip RK3562 Clock Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:clk-rk3562");
#endif /* MODULE */
