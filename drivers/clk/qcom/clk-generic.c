/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/bug.h>
#include <linux/err.h>
#include <linux/clk-provider.h>
#include <linux/clk/msm-clk-generic.h>


/* ==================== Mux clock ==================== */

static int mux_set_parent(struct clk_hw *hw, u8 sel)
{
	struct mux_clk *mux = to_mux_clk(hw);

	if (mux->parent_map)
		sel = mux->parent_map[sel];

	return mux->ops->set_mux_sel(mux, sel);
}

static u8 mux_get_parent(struct clk_hw *hw)
{
	struct mux_clk *mux = to_mux_clk(hw);
	int num_parents = __clk_get_num_parents(hw->clk);
	int i;
	u8 sel;

	sel = mux->ops->get_mux_sel(mux);
	if (mux->parent_map) {
		for (i = 0; i < num_parents; i++)
			if (sel == mux->parent_map[i])
				return i;
		WARN(1, "Can't find parent\n");
		return -EINVAL;
	}

	return sel;
}

static int mux_enable(struct clk_hw *hw)
{
	struct mux_clk *mux = to_mux_clk(hw);
	if (mux->ops->enable)
		return mux->ops->enable(mux);
	return 0;
}

static void mux_disable(struct clk_hw *hw)
{
	struct mux_clk *mux = to_mux_clk(hw);
	if (mux->ops->disable)
		return mux->ops->disable(mux);
}

static struct clk *mux_get_safe_parent(struct clk_hw *hw)
{
	int i;
	struct mux_clk *mux = to_mux_clk(hw);
	int num_parents = __clk_get_num_parents(hw->clk);

	if (!mux->has_safe_parent)
		return NULL;

	i = mux->safe_sel;
	if (mux->parent_map)
		for (i = 0; i < num_parents; i++)
			if (mux->safe_sel == mux->parent_map[i])
				break;

	return clk_get_parent_by_index(hw->clk, i);
}

const struct clk_ops clk_ops_gen_mux = {
	.enable = mux_enable,
	.disable = mux_disable,
	.set_parent = mux_set_parent,
	.get_parent = mux_get_parent,
	.determine_rate = __clk_mux_determine_rate,
	.get_safe_parent = mux_get_safe_parent,
};
EXPORT_SYMBOL_GPL(clk_ops_gen_mux);

/* ==================== Divider clock ==================== */

static long __div_round_rate(struct div_data *data, unsigned long rate,
	struct clk *parent, unsigned int *best_div, unsigned long *best_prate,
	bool set_parent)
{
	unsigned int div, min_div, max_div, _best_div = 1;
	unsigned long prate, _best_prate = 0, rrate = 0, req_prate, actual_rate;
	unsigned int numer;

	rate = max(rate, 1UL);

	min_div = max(data->min_div, 1U);
	max_div = min(data->max_div, (unsigned int) (ULONG_MAX / rate));

	/*
	 * div values are doubled for half dividers.
	 * Adjust for that by picking a numer of 2.
	 */
	numer = data->is_half_divider ? 2 : 1;

	if (!set_parent) {
		prate = *best_prate * numer;
		div = DIV_ROUND_UP(prate, rate);
		div = clamp(1U, div, max_div);
		if (best_div)
			*best_div = div;
		return mult_frac(*best_prate, numer, div);
	}

	for (div = min_div; div <= max_div; div++) {
		req_prate = mult_frac(rate, div, numer);
		prate = __clk_round_rate(parent, req_prate);
		if (IS_ERR_VALUE(prate))
			break;

		actual_rate = mult_frac(prate, numer, div);
		if (is_better_rate(rate, rrate, actual_rate)) {
			rrate = actual_rate;
			_best_div = div;
			_best_prate = prate;
		}

		/*
		 * Trying higher dividers is only going to ask the parent for
		 * a higher rate. If it can't even output a rate higher than
		 * the one we request for this divider, the parent is not
		 * going to be able to output an even higher rate required
		 * for a higher divider. So, stop trying higher dividers.
		 */
		if (actual_rate < rate)
			break;

		if (rrate <= rate)
			break;
	}

	if (!rrate)
		return -EINVAL;
	if (best_div)
		*best_div = _best_div;
	if (best_prate)
		*best_prate = _best_prate;

	return rrate;
}

static long div_round_rate(struct clk_hw *hw, unsigned long rate,
			   unsigned long *parent_rate)
{
	struct div_clk *d = to_div_clk(hw);
	bool set_parent = __clk_get_flags(hw->clk) & CLK_SET_RATE_PARENT;

	return __div_round_rate(&d->data, rate, __clk_get_parent(hw->clk),
				NULL, parent_rate, set_parent);
}

static int div_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long
			parent_rate)
{
	struct div_clk *d = to_div_clk(hw);
	int div, rc = 0;
	struct div_data *data = &d->data;

	div = parent_rate / rate;
	if (div != data->div)
		rc = d->ops->set_div(d, div);
	data->div = div;

	return rc;
}

static int div_enable(struct clk_hw *hw)
{
	struct div_clk *d = to_div_clk(hw);
	if (d->ops && d->ops->enable)
		return d->ops->enable(d);
	return 0;
}

static void div_disable(struct clk_hw *hw)
{
	struct div_clk *d = to_div_clk(hw);
	if (d->ops && d->ops->disable)
		return d->ops->disable(d);
}

static unsigned long div_recalc_rate(struct clk_hw *hw, unsigned long prate)
{
	struct div_clk *d = to_div_clk(hw);
	unsigned int div = d->data.div;

	if (d->ops && d->ops->get_div)
		div = max(d->ops->get_div(d), 1);
	div = max(div, 1U);

	if (!d->ops || !d->ops->set_div)
		d->data.min_div = d->data.max_div = div;
	d->data.div = div;

	return prate / div;
}

const struct clk_ops clk_ops_div = {
	.enable = div_enable,
	.disable = div_disable,
	.round_rate = div_round_rate,
	.set_rate = div_set_rate,
	.recalc_rate = div_recalc_rate,
};
EXPORT_SYMBOL_GPL(clk_ops_div);

/* ==================== Mux_div clock ==================== */

static int mux_div_clk_enable(struct clk_hw *hw)
{
	struct mux_div_clk *md = to_mux_div_clk(hw);

	if (md->ops->enable)
		return md->ops->enable(md);
	return 0;
}

static void mux_div_clk_disable(struct clk_hw *hw)
{
	struct mux_div_clk *md = to_mux_div_clk(hw);

	if (md->ops->disable)
		return md->ops->disable(md);
}

static long __mux_div_round_rate(struct clk_hw *hw, unsigned long rate,
	struct clk **best_parent, int *best_div, unsigned long *best_prate)
{
	struct mux_div_clk *md = to_mux_div_clk(hw);
	unsigned int i;
	unsigned long rrate, best = 0, _best_div = 0, _best_prate = 0;
	struct clk *_best_parent = 0;
	int num_parents = __clk_get_num_parents(hw->clk);
	bool set_parent = __clk_get_flags(hw->clk) & CLK_SET_RATE_PARENT;

	for (i = 0; i < num_parents; i++) {
		int div;
		unsigned long prate;
		struct clk *p = clk_get_parent_by_index(hw->clk, i);

		rrate = __div_round_rate(&md->data, rate, p, &div, &prate,
				set_parent);

		if (is_better_rate(rate, best, rrate)) {
			best = rrate;
			_best_div = div;
			_best_prate = prate;
			_best_parent = p;
		}

		if (rate <= rrate)
			break;
	}

	if (best_div)
		*best_div = _best_div;
	if (best_prate)
		*best_prate = _best_prate;
	if (best_parent)
		*best_parent = _best_parent;

	if (best)
		return best;
	return -EINVAL;
}

static long mux_div_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long *parent_rate)
{
	return __mux_div_round_rate(hw, rate, NULL, NULL, parent_rate);
}

/* requires enable lock to be held */
static int __set_src_div(struct mux_div_clk *md, u8 src_sel, u32 div)
{
	int rc;

	rc = md->ops->set_src_div(md, src_sel, div);
	if (!rc) {
		md->data.div = div;
		md->src_sel = src_sel;
	}

	return rc;
}

/* Must be called after handoff to ensure parent clock rates are initialized */
static int safe_parent_init_once(struct clk_hw *hw)
{
	unsigned long rrate;
	u32 best_div;
	struct clk *best_parent;
	struct mux_div_clk *md = to_mux_div_clk(hw);

	if (IS_ERR(md->safe_parent))
		return -EINVAL;
	if (!md->safe_freq || md->safe_parent)
		return 0;

	rrate = __mux_div_round_rate(hw, md->safe_freq, &best_parent,
			&best_div, NULL);

	if (rrate == md->safe_freq) {
		md->safe_div = best_div;
		md->safe_parent = best_parent;
	} else {
		md->safe_parent = ERR_PTR(-EINVAL);
		return -EINVAL;
	}
	return 0;
}

static int
__mux_div_clk_set_rate_and_parent(struct clk_hw *hw, u8 index, u32 div)
{
	struct mux_div_clk *md = to_mux_div_clk(hw);
	int rc;

	rc = safe_parent_init_once(hw);
	if (rc)
		return rc;

	return __set_src_div(md, index, div);
}

static int mux_div_clk_set_rate_and_parent(struct clk_hw *hw,
		unsigned long rate, unsigned long parent_rate, u8 index)
{
	return __mux_div_clk_set_rate_and_parent(hw, index, parent_rate / rate);
}

static int mux_div_clk_set_rate(struct clk_hw *hw,
		unsigned long rate, unsigned long parent_rate)
{
	struct mux_div_clk *md = to_mux_div_clk(hw);
	return __mux_div_clk_set_rate_and_parent(hw, md->src_sel,
			parent_rate / rate);
}

static int mux_div_clk_set_parent(struct clk_hw *hw, u8 index)
{
	struct mux_div_clk *md = to_mux_div_clk(hw);
	return __mux_div_clk_set_rate_and_parent(hw, md->parent_map[index],
			md->data.div);
}

static u8 mux_div_clk_get_parent(struct clk_hw *hw)
{
	struct mux_div_clk *md = to_mux_div_clk(hw);
	int num_parents = __clk_get_num_parents(hw->clk);
	u32 i, div, sel;

	md->ops->get_src_div(md, &sel, &div);
	md->src_sel = sel;

	for (i = 0; i < num_parents; i++)
		if (sel == md->parent_map[i])
			return i;
	WARN(1, "Can't find parent\n");
	return -EINVAL;
}

static unsigned long
mux_div_clk_recalc_rate(struct clk_hw *hw, unsigned long prate)
{
	struct mux_div_clk *md = to_mux_div_clk(hw);
	u32 div, sel;

	md->ops->get_src_div(md, &sel, &div);

	return prate / div;
}

const struct clk_ops clk_ops_mux_div_clk = {
	.enable = mux_div_clk_enable,
	.disable = mux_div_clk_disable,
	.set_rate_and_parent = mux_div_clk_set_rate_and_parent,
	.set_rate = mux_div_clk_set_rate,
	.set_parent = mux_div_clk_set_parent,
	.round_rate = mux_div_clk_round_rate,
	.get_parent = mux_div_clk_get_parent,
	.recalc_rate = mux_div_clk_recalc_rate,
};
EXPORT_SYMBOL_GPL(clk_ops_mux_div_clk);
