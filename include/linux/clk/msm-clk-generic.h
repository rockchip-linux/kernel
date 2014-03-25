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

#ifndef __QCOM_CLK_GENERIC_H__
#define __QCOM_CLK_GENERIC_H__

#include <linux/err.h>
#include <linux/clk-provider.h>

static inline bool is_better_rate(unsigned long req, unsigned long best,
				  unsigned long new)
{
	if (IS_ERR_VALUE(new))
		return false;

	return (req <= new && new < best) || (best < req && best < new);
}

/* ==================== Mux clock ==================== */

struct mux_clk;

struct clk_mux_ops {
	int (*set_mux_sel)(struct mux_clk *clk, int sel);
	int (*get_mux_sel)(struct mux_clk *clk);

	/* Optional */
	bool (*is_enabled)(struct mux_clk *clk);
	int (*enable)(struct mux_clk *clk);
	void (*disable)(struct mux_clk *clk);
};

struct mux_clk {
	/* Parents in decreasing order of preference for obtaining rates. */
	u8 		*parent_map;
	bool		has_safe_parent;
	u8		safe_sel;
	const struct clk_mux_ops *ops;

	/* Fields not used by helper function. */
	void __iomem 	*base;
	u32		offset;
	u32		en_offset;
	int		en_reg;
	u32		mask;
	u32		shift;
	u32		en_mask;
	void		*priv;

	struct clk_hw	hw;
};

static inline struct mux_clk *to_mux_clk(struct clk_hw *hw)
{
	return container_of(hw, struct mux_clk, hw);
}

extern const struct clk_ops clk_ops_gen_mux;

/* ==================== Divider clock ==================== */

struct div_clk;

struct clk_div_ops {
	int (*set_div)(struct div_clk *clk, int div);
	int (*get_div)(struct div_clk *clk);
	bool (*is_enabled)(struct div_clk *clk);
	int (*enable)(struct div_clk *clk);
	void (*disable)(struct div_clk *clk);
};

struct div_data {
	unsigned int div;
	unsigned int min_div;
	unsigned int max_div;
	/*
	 * Indicate whether this divider clock supports half-interger divider.
	 * If it is, all the min_div and max_div have been doubled. It means
	 * they are 2*N.
	 */
	bool is_half_divider;
};

struct div_clk {
	struct div_data data;

	/* Optional */
	const struct clk_div_ops *ops;

	/* Fields not used by helper function. */
	void __iomem 	*base;
	u32		offset;
	u32		mask;
	u32		shift;
	u32		en_mask;
	void		*priv;
	struct clk_hw	hw;
};

static inline struct div_clk *to_div_clk(struct clk_hw *hw)
{
	return container_of(hw, struct div_clk, hw);
}

extern const struct clk_ops clk_ops_div;

#define DEFINE_FIXED_DIV_CLK(clk_name, _div, _parent) \
static struct div_clk clk_name = {	\
	.data = {				\
		.max_div = _div,		\
		.min_div = _div,		\
		.div = _div,			\
	},					\
	.hw.init = &(struct clk_init_data){ \
		.parent_names = (const char *[]){ _parent }, \
		.num_parents = 1,		\
		.name = #clk_name,		\
		.ops = &clk_ops_div,		\
		.flags = CLK_SET_RATE_PARENT,	\
	}					\
}

/* ==================== Mux Div clock ==================== */

struct mux_div_clk;

/*
 * struct mux_div_ops
 * the enable and disable ops are optional.
 */

struct mux_div_ops {
	int (*set_src_div)(struct mux_div_clk *, u32 src_sel, u32 div);
	void (*get_src_div)(struct mux_div_clk *, u32 *src_sel, u32 *div);
	int (*enable)(struct mux_div_clk *);
	void (*disable)(struct mux_div_clk *);
	bool (*is_enabled)(struct mux_div_clk *);
};

/*
 * struct mux_div_clk - combined mux/divider clock
 * @priv
		parameters needed by ops
 * @safe_freq
		when switching rates from A to B, the mux div clock will
		instead switch from A -> safe_freq -> B. This allows the
		mux_div clock to change rates while enabled, even if this
		behavior is not supported by the parent clocks.

		If changing the rate of parent A also causes the rate of
		parent B to change, then safe_freq must be defined.

		safe_freq is expected to have a source clock which is always
		on and runs at only one rate.
 * @parents
		list of parents and mux indicies
 * @ops
		function pointers for hw specific operations
 * @src_sel
		the mux index which will be used if the clock is enabled.
 */

struct mux_div_clk {
	/* Required parameters */
	const struct mux_div_ops	*ops;
	struct div_data			data;
	u8				*parent_map;

	struct clk_hw			hw;

	/* Internal */
	u32				src_sel;

	/* Optional parameters */
	void				*priv;
	void __iomem			*base;
	u32				div_mask;
	u32				div_offset;
	u32				div_shift;
	u32				src_mask;
	u32				src_offset;
	u32				src_shift;
	u32				en_mask;
	u32				en_offset;

	u32				safe_div;
	struct clk			*safe_parent;
	unsigned long			safe_freq;
};

static inline struct mux_div_clk *to_mux_div_clk(struct clk_hw *hw)
{
	return container_of(hw, struct mux_div_clk, hw);
}

extern const struct clk_ops clk_ops_mux_div_clk;

#endif
