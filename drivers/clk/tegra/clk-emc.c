/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_data/tegra_emc.h>

#include "clk.h"

static u8 clk_emc_get_parent(struct clk_hw *hw)
{
	struct tegra_clk_emc *emc = to_clk_emc(hw);
	const struct clk_ops *mux_ops = emc->periph->mux_ops;
	struct clk_hw *mux_hw = &emc->periph->mux.hw;

	mux_hw->clk = hw->clk;
	return mux_ops->get_parent(mux_hw);
}

static unsigned long clk_emc_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct tegra_clk_emc *emc = to_clk_emc(hw);
	const struct clk_ops *div_ops = emc->periph->div_ops;
	struct clk_hw *div_hw = &emc->periph->divider.hw;
	unsigned long new_rate = __clk_get_rate(clk_get_parent(hw->clk));

	div_hw->clk = hw->clk;
	return div_ops->recalc_rate(div_hw, new_rate);
}

static long clk_emc_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *prate)
{
	struct tegra_clk_emc *emc = to_clk_emc(hw);
	struct clk *parent_clk = __clk_get_parent(hw->clk);
	unsigned long parent_rate = __clk_get_rate(parent_clk);
	unsigned long ret;

	if (!emc->emc_ops)
		return parent_rate;

	ret = emc->emc_ops->emc_round_rate(rate);
	if (!ret)
		return parent_rate;

	return ret;
}

static int clk_emc_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct tegra_clk_emc *emc = to_clk_emc(hw);
	struct clk *old_parent, *new_parent, *backup_parent;
	unsigned long backup_rate, old_rate, new_parent_rate;
	int ret = -EINVAL;

	if (!emc->emc_ops)
		goto out;

	new_parent = emc->emc_ops->emc_predict_parent(rate, &new_parent_rate);
	if (IS_ERR(new_parent))
		goto out;

	old_parent = __clk_get_parent(hw->clk);
	old_rate = emc->emc_ops->emc_get_rate();
	if (old_rate == rate)
		return 0;

	if (new_parent_rate != clk_get_rate(new_parent)) {
		if (emc->emc_ops->emc_get_backup_parent) {
			emc->emc_ops->emc_get_backup_parent(&backup_parent,
							    &backup_rate);
			if (IS_ERR(backup_parent))
				goto out;
			clk_prepare_enable(backup_parent);
			ret = emc->emc_ops->emc_set_rate(backup_rate);
			if (ret) {
				clk_disable_unprepare(backup_parent);
				goto out;
			}
			__clk_reparent(hw->clk, backup_parent);
			clk_disable_unprepare(old_parent);
			old_parent = backup_parent;
			ret = clk_set_rate(new_parent, new_parent_rate);
			if (ret)
				goto out;
		} else {
			goto out;
		}
	}

	if (new_parent != old_parent)
		clk_prepare_enable(new_parent);

	ret = emc->emc_ops->emc_set_rate(rate);
	if (ret) {
		if (new_parent != old_parent)
			clk_disable_unprepare(new_parent);
		goto out;
	}

	if (new_parent != old_parent) {
		__clk_reparent(hw->clk, new_parent);
		clk_disable_unprepare(old_parent);
	}

out:
	return ret;
}

static int clk_emc_is_enabled(struct clk_hw *hw)
{
	struct tegra_clk_emc *emc = to_clk_emc(hw);
	const struct clk_ops *gate_ops = emc->periph->gate_ops;
	struct clk_hw *gate_hw = &emc->periph->gate.hw;

	gate_hw->clk = hw->clk;

	return gate_ops->is_enabled(gate_hw);
}

static int clk_emc_enable(struct clk_hw *hw)
{
	struct tegra_clk_emc *emc = to_clk_emc(hw);
	const struct clk_ops *gate_ops = emc->periph->gate_ops;
	struct clk_hw *gate_hw = &emc->periph->gate.hw;

	gate_hw->clk = hw->clk;

	return gate_ops->enable(gate_hw);
}

static void clk_emc_disable(struct clk_hw *hw)
{
	struct tegra_clk_emc *emc = to_clk_emc(hw);
	const struct clk_ops *gate_ops = emc->periph->gate_ops;
	struct clk_hw *gate_hw = &emc->periph->gate.hw;

	gate_ops->disable(gate_hw);
}

static const struct clk_ops tegra_clk_emc_ops = {
	.get_parent = clk_emc_get_parent,
	.recalc_rate = clk_emc_recalc_rate,
	.round_rate = clk_emc_round_rate,
	.set_rate = clk_emc_set_rate,
	.is_enabled = clk_emc_is_enabled,
	.enable = clk_emc_enable,
	.disable = clk_emc_disable,
};

struct clk *tegra_clk_register_emc(const char *name, const char **parent_names,
	int num_parents, struct tegra_clk_periph *periph,
	void __iomem *clk_base, u32 offset, unsigned long flags,
	const struct emc_clk_ops *emc_ops)
{
	struct tegra_clk_emc *emc;
	struct clk *clk;
	struct clk_init_data init;
	struct tegra_clk_periph_regs *bank;

	emc = kzalloc(sizeof(*emc), GFP_KERNEL);
	if (!emc) {
		pr_err("%s: could not allocate emc clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &tegra_clk_emc_ops;
	init.flags = flags;
	init.parent_names = parent_names;
	init.num_parents = num_parents;

	bank = get_reg_bank(periph->gate.clk_num);
	if (!bank)
		return ERR_PTR(-EINVAL);

	/* Data in .init is copied by clk_register(), so stack variable OK */
	periph->hw.init = &init;
	periph->magic = TEGRA_CLK_PERIPH_MAGIC;
	periph->mux.reg = clk_base + offset;
	periph->divider.reg = clk_base + offset;
	periph->gate.clk_base = clk_base;
	periph->gate.regs = bank;
	periph->gate.enable_refcnt = periph_clk_enb_refcnt;

	emc->hw.init = &init;
	emc->periph = periph;
	emc->emc_ops = emc_ops;
	clk = clk_register(NULL, &emc->hw);
	if (IS_ERR(clk)) {
		kfree(emc);
		return clk;
	}

	emc->periph->mux.hw.clk = clk;
	emc->periph->divider.hw.clk = clk;
	emc->periph->gate.hw.clk = clk;

	return clk;
}
