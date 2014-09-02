/*
 * Copyright (c) 2014 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/clk/tegra.h>
#include <soc/tegra/fuse.h>
#include <dt-bindings/clock/tegra132-ccplex.h>

#include "clk.h"

#define PLLX_BASE 0x0
#define PLLX_MISC 0x4
#define PLLX_MISC2 0xc
#define PLLX_MISC3 0x10
#define PLLX_HW_CTRL_CFG 0x14
#define PLLX_SW_RAMP_CFG 0x18
#define PLLX_HW_CTRL_STATUS 0x1c
#define CCLK_BURST_POLICY 0x20

#define PLL_BASE_LOCK BIT(27)
#define PLL_MISC_LOCK_ENABLE 18

#define RST_DFLL_DVCO		0x80
#define DVFS_DFLL_RESET_SHIFT	0

static struct div_nmp pllxc_nmp = {
	.divm_shift = 0,
	.divm_width = 8,
	.divn_shift = 8,
	.divn_width = 8,
	.divp_shift = 20,
	.divp_width = 4,
};

static struct pdiv_map pllxc_p[] = {
	{ .pdiv = 1, .hw_val = 0 },
	{ .pdiv = 2, .hw_val = 1 },
	{ .pdiv = 3, .hw_val = 2 },
	{ .pdiv = 4, .hw_val = 3 },
	{ .pdiv = 5, .hw_val = 4 },
	{ .pdiv = 6, .hw_val = 5 },
	{ .pdiv = 8, .hw_val = 6 },
	{ .pdiv = 10, .hw_val = 7 },
	{ .pdiv = 12, .hw_val = 8 },
	{ .pdiv = 16, .hw_val = 9 },
	{ .pdiv = 12, .hw_val = 10 },
	{ .pdiv = 16, .hw_val = 11 },
	{ .pdiv = 20, .hw_val = 12 },
	{ .pdiv = 24, .hw_val = 13 },
	{ .pdiv = 32, .hw_val = 14 },
	{ .pdiv = 0, .hw_val = 0 },
};

static struct tegra_clk_pll_freq_table pll_x_freq_table[] = {
	/* 1 GHz */
	{12000000, 1000000000, 83, 0, 1},	/* actual: 996.0 MHz */
	{13000000, 1000000000, 76, 0, 1},	/* actual: 988.0 MHz */
	{16800000, 1000000000, 59, 0, 1},	/* actual: 991.2 MHz */
	{19200000, 1000000000, 52, 0, 1},	/* actual: 998.4 MHz */
	{26000000, 1000000000, 76, 1, 1},	/* actual: 988.0 MHz */
	{0, 0, 0, 0, 0, 0},
};

static struct tegra_clk_pll_params pll_x_params = {
	.input_min = 12000000,
	.input_max = 800000000,
	.cf_min = 12000000,
	.cf_max = 19200000,	/* s/w policy, h/w capability 50 MHz */
	.vco_min = 700000000,
	.vco_max = 3000000000UL,
	.base_reg = PLLX_BASE,
	.misc_reg = PLLX_MISC,
	.lock_mask = PLL_BASE_LOCK,
	.lock_enable_bit_idx = PLL_MISC_LOCK_ENABLE,
	.lock_delay = 300,
	.iddq_reg = PLLX_MISC3,
	.iddq_bit_idx = 3,
	.max_p = 6,
	.dyn_ramp_reg = PLLX_MISC2,
	.stepa_shift = 16,
	.stepb_shift = 24,
	.pdiv_tohw = pllxc_p,
	.div_nmp = &pllxc_nmp,
	.freq_table = pll_x_freq_table,
	.flags = TEGRA_PLL_USE_LOCK,
};

static void __iomem *clk_base;
static void __iomem *pmc_base;
static struct clk *clks[TEGRA132_CCPLEX_CLK_MAX];
static struct clk_onecell_data clk_data = {
	.clks  = clks,
	.clk_num = ARRAY_SIZE(clks)
};

static const struct of_device_id pmc_match[] __initconst = {
	{ .compatible = "nvidia,tegra124-pmc" },
	{},
};

static const char *cclk_g_parents[] = { "clk_m", "unused", "unused", "pll_ref",
					"pll_m", "pll_p", "sclk", "unused",
					"pll_x", "unused", "unused", "unused",
					"unused", "unused", "unused",
					"dfllCPU_out", };


/**
 * tegra132_clock_assert_dfll_dvco_reset - assert the DFLL's DVCO reset
 *
 * Assert the reset line of the DFLL's DVCO.  No return value.
 */
void tegra132_clock_assert_dfll_dvco_reset(void)
{
	u32 v;

	v = readl_relaxed(clk_base + RST_DFLL_DVCO);
	v |= (1 << DVFS_DFLL_RESET_SHIFT);
	writel_relaxed(v, clk_base + RST_DFLL_DVCO);
	readl_relaxed(clk_base + RST_DFLL_DVCO);
}
EXPORT_SYMBOL(tegra132_clock_assert_dfll_dvco_reset);

/**
 * tegra132_clock_deassert_dfll_dvco_reset - deassert the DFLL's DVCO reset
 *
 * Deassert the reset line of the DFLL's DVCO, allowing the DVCO to
 * operate.  No return value.
 */
void tegra132_clock_deassert_dfll_dvco_reset(void)
{
	u32 v;

	v = readl_relaxed(clk_base + RST_DFLL_DVCO);
	v &= ~(1 << DVFS_DFLL_RESET_SHIFT);
	writel_relaxed(v, clk_base + RST_DFLL_DVCO);
	readl_relaxed(clk_base + RST_DFLL_DVCO);
}
EXPORT_SYMBOL(tegra132_clock_deassert_dfll_dvco_reset);

static struct tegra_clk_init_table init_table[] __initdata = {
	{TEGRA132_CCPLEX_CCLK_G, TEGRA132_CCPLEX_CLK_MAX, 0, 1},
	/* This MUST be the last entry. */
	{TEGRA132_CCPLEX_CLK_MAX, TEGRA132_CCPLEX_CLK_MAX, 0, 0},
};

static void __init tegra132_clock_apply_init_table(void)
{
	tegra_init_from_table(init_table, clks, TEGRA132_CCPLEX_CLK_MAX);
}

static void __init tegra132_ccplex(struct device_node *np)
{
	struct clk *clk;
	struct device_node *node;

	clk_base = of_iomap(np, 0);
	if (!clk_base) {
		pr_err("ioremap tegra132 CCPLEX clk failed\n");
		return;
	}

	node = of_find_matching_node(NULL, pmc_match);
	if (!node) {
		pr_err("Failed to find pmc node\n");
		WARN_ON(1);
		return;
	}

	pmc_base = of_iomap(node, 0);
	if (!pmc_base) {
		pr_err("Can't map pmc registers\n");
		WARN_ON(1);
		return;
	}

	clk = clk_register_mux(NULL, "cclk_g", cclk_g_parents,
			       ARRAY_SIZE(cclk_g_parents),
				   CLK_SET_RATE_NO_REPARENT |
				   CLK_SET_RATE_PARENT,
			       clk_base + CCLK_BURST_POLICY, 28, 4, 0, NULL);
	clks[TEGRA132_CCPLEX_CCLK_G] = clk;
	clk_register_clkdev(clk, "cclk_g", NULL);

	clk = tegra_clk_register_pllxc("pll_x", "pll_ref", clk_base,
			pmc_base, CLK_IGNORE_UNUSED, &pll_x_params, NULL);
	clks[TEGRA132_PLL_X] = clk;

	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);

	tegra132_clock_apply_init_table();
}

CLK_OF_DECLARE(tegra132_ccplex, "nvidia,tegra132-ccplex-clk", tegra132_ccplex);

