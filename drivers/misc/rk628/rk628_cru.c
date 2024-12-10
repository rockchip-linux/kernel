// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Rockchip Electronics Co. Ltd.
 *
 * Author: Wyon Bi <bivvy.bi@rock-chips.com>
 */

#include "rk628.h"
#include "rk628_cru.h"

#define REFCLK_RATE		24000000UL
#define MIN_FREF_RATE		10000000UL
#define MAX_FREF_RATE		800000000UL
#define MIN_FREFDIV_RATE	1000000UL
#define MAX_FREFDIV_RATE	100000000UL
#define MIN_FVCO_RATE		600000000UL
#define MAX_FVCO_RATE		1600000000UL
#define MIN_FOUTPOSTDIV_RATE	12000000UL
#define MAX_FOUTPOSTDIV_RATE	1600000000UL

static void rational_best_approximation(unsigned long given_numerator,
					unsigned long given_denominator,
					unsigned long max_numerator,
					unsigned long max_denominator,
					unsigned long *best_numerator,
					unsigned long *best_denominator)
{
	unsigned long n, d, n0, d0, n1, d1;

	n = given_numerator;
	d = given_denominator;
	n0 = d1 = 0;
	n1 = d0 = 1;
	for (;;) {
		unsigned long t, a;

		if ((n1 > max_numerator) || (d1 > max_denominator)) {
			n1 = n0;
			d1 = d0;
			break;
		}
		if (d == 0)
			break;
		t = d;
		a = n / d;
		d = n % d;
		n = t;
		t = n0 + a * n1;
		n0 = n1;
		n1 = t;
		t = d0 + a * d1;
		d0 = d1;
		d1 = t;
	}
	*best_numerator = n1;
	*best_denominator = d1;
}

static unsigned long rk628_cru_clk_get_rate_pll(struct rk628 *rk628,
						unsigned int id)
{
	unsigned long parent_rate = REFCLK_RATE;
	u32 postdiv1, fbdiv, dsmpd, postdiv2, refdiv, frac, bypass;
	u32 con0, con1, con2;
	u64 foutvco, foutpostdiv;
	u32 offset, val;

	rk628_i2c_read(rk628, CRU_MODE_CON00, &val);
	if (id == CGU_CLK_CPLL) {
		val &= CLK_CPLL_MODE_MASK;
		val >>= CLK_CPLL_MODE_SHIFT;
		if (val == CLK_CPLL_MODE_OSC)
			return parent_rate;

		offset = 0x00;
	} else {
		val &= CLK_GPLL_MODE_MASK;
		val >>= CLK_GPLL_MODE_SHIFT;
		if (val == CLK_GPLL_MODE_OSC)
			return parent_rate;

		offset = 0x20;
	}

	rk628_i2c_read(rk628, offset + CRU_CPLL_CON0, &con0);
	rk628_i2c_read(rk628, offset + CRU_CPLL_CON1, &con1);
	rk628_i2c_read(rk628, offset + CRU_CPLL_CON2, &con2);

	bypass = (con0 & PLL_BYPASS_MASK) >> PLL_BYPASS_SHIFT;
	postdiv1 = (con0 & PLL_POSTDIV1_MASK) >> PLL_POSTDIV1_SHIFT;
	fbdiv = (con0 & PLL_FBDIV_MASK) >> PLL_FBDIV_SHIFT;
	dsmpd = (con1 & PLL_DSMPD_MASK) >> PLL_DSMPD_SHIFT;
	postdiv2 = (con1 & PLL_POSTDIV2_MASK) >> PLL_POSTDIV2_SHIFT;
	refdiv = (con1 & PLL_REFDIV_MASK) >> PLL_REFDIV_SHIFT;
	frac = (con2 & PLL_FRAC_MASK) >> PLL_FRAC_SHIFT;

	if (bypass)
		return parent_rate;

	foutvco = parent_rate * fbdiv;
	do_div(foutvco, refdiv);

	if (!dsmpd) {
		u64 frac_rate = (u64)parent_rate * frac;

		do_div(frac_rate, refdiv);
		foutvco += frac_rate >> 24;
	}

	foutpostdiv = foutvco;
	do_div(foutpostdiv, postdiv1);
	do_div(foutpostdiv, postdiv2);

	return foutpostdiv;
}

static unsigned long rk628_cru_clk_set_rate_pll(struct rk628 *rk628,
						unsigned int id,
						unsigned long rate)
{
	unsigned long fin = REFCLK_RATE, fout = rate;
	u8 min_refdiv, max_refdiv, postdiv;
	u8 dsmpd = 1, postdiv1 = 0, postdiv2 = 0, refdiv = 0;
	u16 fbdiv = 0;
	u32 frac = 0;
	u64 foutvco, foutpostdiv;
	u32 offset, val;

	/*
	 * FREF : 10MHz ~ 800MHz
	 * FREFDIV : 1MHz ~ 40MHz
	 * FOUTVCO : 400MHz ~ 1.6GHz
	 * FOUTPOSTDIV : 8MHz ~ 1.6GHz
	 */
	if (fin < MIN_FREF_RATE || fin > MAX_FREF_RATE)
		return 0;

	if (fout < MIN_FOUTPOSTDIV_RATE || fout > MAX_FOUTPOSTDIV_RATE)
		return 0;

	if (id == CGU_CLK_CPLL)
		offset = 0x00;
	else
		offset = 0x20;

	rk628_i2c_write(rk628, offset + CRU_CPLL_CON1, PLL_PD(1));

	if (fin == fout) {
		rk628_i2c_write(rk628, offset + CRU_CPLL_CON0, PLL_BYPASS(1));
		rk628_i2c_write(rk628, offset + CRU_CPLL_CON1, PLL_PD(0));
		while (1) {
			rk628_i2c_read(rk628, offset + CRU_CPLL_CON1, &val);
			if (val & PLL_LOCK)
				break;
		}
		return fin;
	}

	min_refdiv = fin / MAX_FREFDIV_RATE + 1;
	max_refdiv = fin / MIN_FREFDIV_RATE;
	if (max_refdiv > 64)
		max_refdiv = 64;

	if (fout < MIN_FVCO_RATE) {
		postdiv = MIN_FVCO_RATE / fout + 1;

		for (postdiv2 = 1; postdiv2 < 8; postdiv2++) {
			if (postdiv % postdiv2)
				continue;

			postdiv1 = postdiv / postdiv2;

			if (postdiv1 > 0 && postdiv1 < 8)
				break;
		}

		if (postdiv2 > 7)
			return 0;

		fout *= postdiv1 * postdiv2;
	} else {
		postdiv1 = 1;
		postdiv2 = 1;
	}

	for (refdiv = min_refdiv; refdiv <= max_refdiv; refdiv++) {
		u64 tmp, frac_rate;

		if (fin % refdiv)
			continue;

		tmp = (u64)fout * refdiv;
		do_div(tmp, fin);
		fbdiv = tmp;
		if (fbdiv < 10 || fbdiv > 1600)
			continue;

		tmp = (u64)fbdiv * fin;
		do_div(tmp, refdiv);
		if (fout < MIN_FVCO_RATE || fout > MAX_FVCO_RATE)
			continue;

		frac_rate = fout - tmp;

		if (frac_rate) {
			tmp = (u64)frac_rate * refdiv;
			tmp <<= 24;
			do_div(tmp, fin);
			frac = tmp;
			dsmpd = 0;
		}

		break;
	}

	/*
	 * If DSMPD = 1 (DSM is disabled, "integer mode")
	 * FOUTVCO = FREF / REFDIV * FBDIV
	 * FOUTPOSTDIV = FOUTVCO / POSTDIV1 / POSTDIV2
	 *
	 * If DSMPD = 0 (DSM is enabled, "fractional mode")
	 * FOUTVCO = FREF / REFDIV * (FBDIV + FRAC / 2^24)
	 * FOUTPOSTDIV = FOUTVCO / POSTDIV1 / POSTDIV2
	 */
	foutvco = fin * fbdiv;
	do_div(foutvco, refdiv);

	if (!dsmpd) {
		u64 frac_rate = (u64)fin * frac;

		do_div(frac_rate, refdiv);
		foutvco += frac_rate >> 24;
	}

	foutpostdiv = foutvco;
	do_div(foutpostdiv, postdiv1);
	do_div(foutpostdiv, postdiv2);

	rk628_i2c_write(rk628, offset + CRU_CPLL_CON0,
			PLL_BYPASS(0) | PLL_POSTDIV1(postdiv1) |
			PLL_FBDIV(fbdiv));
	rk628_i2c_write(rk628, offset + CRU_CPLL_CON1,
			PLL_DSMPD(dsmpd) | PLL_POSTDIV2(postdiv2) |
			PLL_REFDIV(refdiv));
	rk628_i2c_write(rk628, offset + CRU_CPLL_CON2, PLL_FRAC(frac));

	rk628_i2c_write(rk628, offset + CRU_CPLL_CON1, PLL_PD(0));
	while (1) {
		rk628_i2c_read(rk628, offset + CRU_CPLL_CON1, &val);
		if (val & PLL_LOCK)
			break;
	}

	return (unsigned long)foutpostdiv;
}

static unsigned long rk628_cru_clk_set_rate_sclk_vop(struct rk628 *rk628,
						     unsigned long rate)
{
	unsigned long m, n, parent_rate;
	u32 val;

	rk628_i2c_read(rk628, CRU_CLKSEL_CON02, &val);
	val &= SCLK_VOP_SEL_MASK;
	val >>= SCLK_VOP_SEL_SHIFT;
	if (val == SCLK_VOP_SEL_GPLL)
		parent_rate = rk628_cru_clk_get_rate_pll(rk628, CGU_CLK_GPLL);
	else
		parent_rate = rk628_cru_clk_get_rate_pll(rk628, CGU_CLK_CPLL);

	rational_best_approximation(rate, parent_rate,
				    GENMASK(15, 0), GENMASK(15, 0),
				    &m, &n);
	rk628_i2c_write(rk628, CRU_CLKSEL_CON13, m << 16 | n);

	return rate;
}

static unsigned long rk628_cru_clk_get_rate_sclk_vop(struct rk628 *rk628)
{
	unsigned long rate, parent_rate, m, n;
	u32 mux, div;

	rk628_i2c_read(rk628, CRU_CLKSEL_CON02, &mux);
	mux &= CLK_UART_SRC_SEL_MASK;
	mux >>= SCLK_VOP_SEL_SHIFT;
	if (mux == SCLK_VOP_SEL_GPLL)
		parent_rate = rk628_cru_clk_get_rate_pll(rk628, CGU_CLK_GPLL);
	else
		parent_rate = rk628_cru_clk_get_rate_pll(rk628, CGU_CLK_CPLL);

	rk628_i2c_read(rk628, CRU_CLKSEL_CON13, &div);
	m = div >> 16 & 0xffff;
	n = div & 0xffff;
	rate = parent_rate * m / n;

	return rate;
}

static unsigned long rk628_cru_clk_set_rate_rx_read(struct rk628 *rk628,
						    unsigned long rate)
{
	unsigned long m, n, parent_rate;
	u32 val;

	rk628_i2c_read(rk628, CRU_CLKSEL_CON02, &val);
	val &= CLK_RX_READ_SEL_MASK;
	val >>= CLK_RX_READ_SEL_SHIFT;
	if (val == CLK_RX_READ_SEL_GPLL)
		parent_rate = rk628_cru_clk_get_rate_pll(rk628, CGU_CLK_GPLL);
	else
		parent_rate = rk628_cru_clk_get_rate_pll(rk628, CGU_CLK_CPLL);

	rational_best_approximation(rate, parent_rate,
				    GENMASK(15, 0), GENMASK(15, 0),
				    &m, &n);
	rk628_i2c_write(rk628, CRU_CLKSEL_CON14, m << 16 | n);

	return rate;
}

static unsigned long rk628_cru_clk_get_rate_uart_src(struct rk628 *rk628)
{
	unsigned long rate, parent_rate;
	u32 mux, div;

	rk628_i2c_read(rk628, CRU_CLKSEL_CON21, &mux);
	mux &= SCLK_VOP_SEL_MASK;
	if (mux == CLK_UART_SRC_SEL_GPLL)
		parent_rate = rk628_cru_clk_get_rate_pll(rk628, CGU_CLK_GPLL);
	else
		parent_rate = rk628_cru_clk_get_rate_pll(rk628, CGU_CLK_CPLL);

	rk628_i2c_read(rk628, CRU_CLKSEL_CON21, &div);
	div &= CLK_UART_SRC_DIV_MASK;
	div >>= CLK_UART_SRC_DIV_SHIFT;
	rate = parent_rate / (div + 1);

	return rate;
}

static unsigned long rk628_cru_clk_set_rate_sclk_uart(struct rk628 *rk628,
						      unsigned long rate)
{
	unsigned long m, n, parent_rate;

	parent_rate = rk628_cru_clk_get_rate_uart_src(rk628);

	if (rate == REFCLK_RATE) {
		rk628_i2c_write(rk628, CRU_CLKSEL_CON06,
				SCLK_UART_SEL(SCLK_UART_SEL_OSC));
		return rate;
	} else if (rate == parent_rate) {
		rk628_i2c_write(rk628, CRU_CLKSEL_CON06,
				SCLK_UART_SEL(SCLK_UART_SEL_UART_SRC));
		return rate;
	}

	rk628_i2c_write(rk628, CRU_CLKSEL_CON06,
			SCLK_UART_SEL(SCLK_UART_SEL_UART_FRAC));

	rational_best_approximation(rate, parent_rate,
				    GENMASK(15, 0), GENMASK(15, 0),
				    &m, &n);
	rk628_i2c_write(rk628, CRU_CLKSEL_CON20, m << 16 | n);

	return rate;
}

static unsigned long
rk628_cru_clk_get_rate_bt1120_dec_parent(struct rk628 *rk628)
{
	unsigned long parent_rate;
	u32 mux;

	rk628_i2c_read(rk628, CRU_CLKSEL_CON02, &mux);
	mux &= CLK_BT1120DEC_SEL_MASK;
	if (mux == CLK_BT1120DEC_SEL_GPLL)
		parent_rate = rk628_cru_clk_get_rate_pll(rk628, CGU_CLK_GPLL);
	else
		parent_rate = rk628_cru_clk_get_rate_pll(rk628, CGU_CLK_CPLL);

	return parent_rate;
}

static unsigned long rk628_cru_clk_set_rate_bt1120_dec(struct rk628 *rk628,
						       unsigned long rate)
{
	unsigned long parent_rate;
	u32 div;

	parent_rate = rk628_cru_clk_get_rate_bt1120_dec_parent(rk628);
	div = DIV_ROUND_UP(parent_rate, rate);
	rk628_i2c_write(rk628, CRU_CLKSEL_CON02, CLK_BT1120DEC_DIV(div-1));

	return parent_rate / div;
}

int rk628_cru_clk_set_rate(struct rk628 *rk628, unsigned int id,
			   unsigned long rate)
{
	switch (id) {
	case CGU_CLK_CPLL:
	case CGU_CLK_GPLL:
		rk628_cru_clk_set_rate_pll(rk628, id, rate);
		break;
	case CGU_CLK_RX_READ:
		rk628_cru_clk_set_rate_rx_read(rk628, rate);
		break;
	case CGU_SCLK_VOP:
		rk628_cru_clk_set_rate_sclk_vop(rk628, rate);
		break;
	case CGU_SCLK_UART:
		rk628_cru_clk_set_rate_sclk_uart(rk628, rate);
		break;
	case CGU_BT1120DEC:
		rk628_cru_clk_set_rate_bt1120_dec(rk628, rate);
		break;
	default:
		return -1;
	}

	return 0;
}

unsigned long rk628_cru_clk_get_rate(struct rk628 *rk628, unsigned int id)
{
	unsigned long rate;

	switch (id) {
	case CGU_CLK_CPLL:
	case CGU_CLK_GPLL:
		rate = rk628_cru_clk_get_rate_pll(rk628, id);
		break;
	case CGU_SCLK_VOP:
		rate = rk628_cru_clk_get_rate_sclk_vop(rk628);
		break;
	default:
		return 0;
	}

	return rate;
}

void rk628_cru_init(struct rk628 *rk628)
{
	u32 val;
	u8 mcu_mode;

	rk628_i2c_read(rk628, GRF_SYSTEM_STATUS0, &val);
	mcu_mode = (val & I2C_ONLY_FLAG) ? 0 : 1;
	if (mcu_mode)
		return;

	rk628_i2c_write(rk628, CRU_GPLL_CON0, 0xffff701d);
	mdelay(1);
	rk628_i2c_write(rk628, CRU_MODE_CON00, 0xffff0004);
	mdelay(1);
	rk628_i2c_write(rk628, CRU_CLKSEL_CON00, 0x00ff0080);
	rk628_i2c_write(rk628, CRU_CLKSEL_CON00, 0x00ff0083);
	rk628_i2c_write(rk628, CRU_CPLL_CON0, 0xffff3063);
	mdelay(1);
	rk628_i2c_write(rk628, CRU_MODE_CON00, 0xffff0005);
	rk628_i2c_write(rk628, CRU_CLKSEL_CON00, 0x00ff0003);
	rk628_i2c_write(rk628, CRU_CLKSEL_CON00, 0x00ff000b);
	rk628_i2c_write(rk628, CRU_GPLL_CON0, 0xffff1028);
	mdelay(1);
	rk628_i2c_write(rk628, CRU_CLKSEL_CON00, 0x00ff008b);
	rk628_i2c_write(rk628, CRU_CPLL_CON0, 0xffff1063);
	mdelay(1);
	rk628_i2c_write(rk628, CRU_CLKSEL_CON00, 0x00ff000b);
}
