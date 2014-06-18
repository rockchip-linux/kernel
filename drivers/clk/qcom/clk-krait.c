/*
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk-provider.h>
#include <linux/spinlock.h>

#include <asm/krait-l2-accessors.h>

#include "clk-krait.h"

/* Secondary and primary muxes share the same cp15 register */
static DEFINE_SPINLOCK(kpss_clock_reg_lock);

#define LPL_SHIFT	8
static void __kpss_mux_set_sel(struct mux_clk *mux, int sel)
{
	unsigned long flags;
	u32 regval;

	spin_lock_irqsave(&kpss_clock_reg_lock, flags);
	regval = krait_get_l2_indirect_reg(mux->offset);
	regval &= ~(mux->mask << mux->shift);
	regval |= (sel & mux->mask) << mux->shift;
	if (mux->priv) {
		regval &= ~(mux->mask << (mux->shift + LPL_SHIFT));
		regval |= (sel & mux->mask) << (mux->shift + LPL_SHIFT);
	}
	krait_set_l2_indirect_reg(mux->offset, regval);
	spin_unlock_irqrestore(&kpss_clock_reg_lock, flags);

	/* Wait for switch to complete. */
	mb();
	udelay(1);
}

static int kpss_mux_set_sel(struct mux_clk *mux, int sel)
{
	mux->en_mask = sel;
	/* Don't touch mux if CPU is off as it won't work */
	if (__clk_is_enabled(mux->hw.clk))
		__kpss_mux_set_sel(mux, sel);
	return 0;
}

static int kpss_mux_get_sel(struct mux_clk *mux)
{
	u32 sel;

	sel = krait_get_l2_indirect_reg(mux->offset);
	sel >>= mux->shift;
	sel &= mux->mask;
	mux->en_mask = sel;

	return sel;
}

static int kpss_mux_enable(struct mux_clk *mux)
{
	__kpss_mux_set_sel(mux, mux->en_mask);
	return 0;
}

static void kpss_mux_disable(struct mux_clk *mux)
{
	__kpss_mux_set_sel(mux, mux->safe_sel);
}

const struct clk_mux_ops clk_mux_ops_kpss = {
	.enable = kpss_mux_enable,
	.disable = kpss_mux_disable,
	.set_mux_sel = kpss_mux_set_sel,
	.get_mux_sel = kpss_mux_get_sel,
};
EXPORT_SYMBOL_GPL(clk_mux_ops_kpss);

/*
 * The divider can divide by 2, 4, 6 and 8. But we only really need div-2. So
 * force it to div-2 during handoff and treat it like a fixed div-2 clock.
 */
static int kpss_div2_get_div(struct div_clk *div)
{
	unsigned long flags;
	u32 regval;
	int val;

	spin_lock_irqsave(&kpss_clock_reg_lock, flags);
	regval = krait_get_l2_indirect_reg(div->offset);
	val = (regval >> div->shift) & div->mask;
	regval &= ~(div->mask << div->shift);
	if (div->priv)
		regval &= ~(div->mask << (div->shift + LPL_SHIFT));
	krait_set_l2_indirect_reg(div->offset, regval);
	spin_unlock_irqrestore(&kpss_clock_reg_lock, flags);

	val = (val + 1) * 2;
	WARN(val != 2, "Divider %s was configured to div-%d instead of 2!\n",
		__clk_get_name(div->hw.clk), val);

	return 2;
}

const struct clk_div_ops clk_div_ops_kpss_div2 = {
	.get_div = kpss_div2_get_div,
};
EXPORT_SYMBOL_GPL(clk_div_ops_kpss_div2);
