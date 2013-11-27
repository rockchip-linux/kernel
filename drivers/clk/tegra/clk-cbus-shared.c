/*
 * Copyright (c) 2012, 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>

#include "clk.h"

static int cbus_switch_one(struct clk *client, struct clk *p)
{
	int ret = 0;
	unsigned long old_parent_rate, new_parent_rate, current_rate;

	current_rate = clk_get_rate(client);
	old_parent_rate = clk_get_rate(clk_get_parent(client));
	new_parent_rate = clk_get_rate(p);

	if (new_parent_rate > old_parent_rate) {
		u64 temp_rate;

		/*
		 * In order to not overclocking the IP block when changing the
		 * parent, we set the divider to a value which will give us an
		 * allowed rate when the new parent is selected.
		 */
		temp_rate = DIV_ROUND_UP_ULL((u64)clk_get_rate(client) *
			(u64)old_parent_rate, new_parent_rate);
		ret = clk_set_rate(client, temp_rate);
		if (ret) {
			pr_err("failed to set %s rate to %llu: %d\n",
				__clk_get_name(client), temp_rate, ret);
			return ret;
		}
	}

	ret = clk_set_parent(client, p);
	if (ret) {
		pr_err("failed to set %s parent to %s: %d\n",
			__clk_get_name(client),
			__clk_get_name(p), ret);
		return ret;
	}

	clk_set_rate(client, current_rate);

	return ret;
}

static int cbus_backup(struct clk_hw *hw)
{
	int ret = 0;
	struct tegra_clk_cbus_shared *cbus = to_clk_cbus_shared(hw);
	struct tegra_clk_cbus_shared *user;

	list_for_each_entry(user, &cbus->shared_bus_list,
				 u.shared_bus_user.node) {
		struct clk *client = user->u.shared_bus_user.client;

		if (client && __clk_is_enabled(client) &&
		 (clk_get_parent(client) == clk_get_parent(hw->clk))) {
			ret = cbus_switch_one(client, cbus->shared_bus_backup);
			if (ret)
				break;
		}
	}

	return ret;
}

static void cbus_restore(struct clk_hw *hw)
{
	struct tegra_clk_cbus_shared *user;
	struct tegra_clk_cbus_shared *cbus = to_clk_cbus_shared(hw);

	list_for_each_entry(user, &cbus->shared_bus_list,
				 u.shared_bus_user.node) {
		struct clk *client = user->u.shared_bus_user.client;

		if (client) {
			cbus_switch_one(client, clk_get_parent(hw->clk));
			clk_set_rate(client, user->u.shared_bus_user.rate);
		}
	}
}

static int clk_cbus_set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	int ret;
	struct clk *parent;
	struct tegra_clk_cbus_shared *cbus = to_clk_cbus_shared(hw);

	if (cbus->rate_updating)
		return 0;

	if (rate == 0)
		return 0;

	cbus->rate_updating = true;

	parent = clk_get_parent(hw->clk);
	if (!parent) {
		cbus->rate_updating = false;
		return -EINVAL;
	}

	ret = clk_prepare_enable(parent);
	if (ret) {
		cbus->rate_updating = false;
		pr_err("%s: failed to enable %s clock: %d\n",
		       __func__, __clk_get_name(hw->clk), ret);
		return ret;
	}

	ret = cbus_backup(hw);
	if (ret)
		goto out;

	ret = clk_set_rate(parent, rate);
	if (ret) {
		pr_err("%s: failed to set %s clock rate %lu: %d\n",
		       __func__, __clk_get_name(hw->clk), rate, ret);
		goto out;
	}

	cbus_restore(hw);

out:
	cbus->rate_updating = false;
	clk_disable_unprepare(parent);
	return ret;
}

static long clk_cbus_round_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long *parent_rate)
{
	struct clk *parent;
	long new_rate;

	parent = clk_get_parent(hw->clk);
	if (IS_ERR(parent)) {
		pr_err("no parent for %s\n", __clk_get_name(hw->clk));
		return *parent_rate;
	}

	new_rate = clk_round_rate(parent, rate);
	if (new_rate < 0)
		return *parent_rate;

	return new_rate;
}

static unsigned long clk_cbus_recalc_rate(struct clk_hw *hw,
				unsigned long parent_rate)
{
	return clk_get_rate(clk_get_parent(hw->clk));
}

static unsigned long _clk_cap_shared_bus(struct clk *c, unsigned long rate,
						unsigned long ceiling)
{
	ceiling = clk_round_rate(c, ceiling);

	rate = min(rate, ceiling);

	return rate;
}

static int _clk_shared_bus_update(struct clk *bus)
{
	struct tegra_clk_cbus_shared *cbus =
			to_clk_cbus_shared(__clk_get_hw(bus));
	struct tegra_clk_cbus_shared *c;
	unsigned long override_rate = 0;
	unsigned long top_rate = 0;
	unsigned long rate = cbus->min_rate;
	unsigned long bw = 0;
	unsigned long ceiling = cbus->max_rate;

	list_for_each_entry(c, &cbus->shared_bus_list,
			u.shared_bus_user.node) {
		/*
		 * Ignore requests from disabled floor, bw users, and
		 * auto-users riding the bus. Always check the ceiling users
		 * so we don't need to enable it for capping the bus rate.
		 */
		if (c->u.shared_bus_user.enabled ||
		    (c->u.shared_bus_user.mode == SHARED_CEILING)) {
			unsigned long request_rate = c->u.shared_bus_user.rate;

			switch (c->u.shared_bus_user.mode) {
			case SHARED_BW:
				bw += request_rate;
				if (bw > cbus->max_rate)
					bw = cbus->max_rate;
				break;
			case SHARED_CEILING:
				if (request_rate)
					ceiling = min(request_rate, ceiling);
				break;
			case SHARED_OVERRIDE:
				if (override_rate == 0)
					override_rate = request_rate;
				break;
			case SHARED_AUTO:
				break;
			case SHARED_FLOOR:
			default:
				top_rate = max(request_rate, top_rate);
				rate = max(top_rate, rate);
			}
		}
	}
	/*
	 * Keep the bus rate as its default rate when there is no SHARED_FLOOR
	 * users enabled so we won't underrun the bus.
	 */
	if (!top_rate)
		rate = clk_get_rate(bus);
	rate = override_rate ? : max(rate, bw);
	ceiling = override_rate ? cbus->max_rate : ceiling;

	rate = _clk_cap_shared_bus(bus, rate, ceiling);

	return clk_set_rate(bus, rate);
}

static int clk_shared_prepare(struct clk_hw *hw)
{
	int err = 0;
	struct tegra_clk_cbus_shared *shared = to_clk_cbus_shared(hw);

	shared->u.shared_bus_user.enabled = true;
	err = _clk_shared_bus_update(clk_get_parent(hw->clk));
	if (!err && shared->u.shared_bus_user.client)
		err = clk_prepare_enable(shared->u.shared_bus_user.client);

	return err;
}

static void clk_shared_unprepare(struct clk_hw *hw)
{
	struct tegra_clk_cbus_shared *shared = to_clk_cbus_shared(hw);

	if (shared->u.shared_bus_user.client)
		clk_disable_unprepare(shared->u.shared_bus_user.client);

	shared->u.shared_bus_user.enabled = false;
	_clk_shared_bus_update(clk_get_parent(hw->clk));
}

static bool shared_clk_set_rate;

static int clk_shared_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct tegra_clk_cbus_shared *shared = to_clk_cbus_shared(hw);
	int err;

	if (shared_clk_set_rate)
		return 0;

	shared_clk_set_rate = true;
	shared->u.shared_bus_user.rate = rate;
	err = _clk_shared_bus_update(clk_get_parent(hw->clk));
	shared_clk_set_rate = false;

	return err;
}

static long clk_shared_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	struct tegra_clk_cbus_shared *shared = to_clk_cbus_shared(hw);
	struct tegra_clk_cbus_shared *parent_cbus;
	struct clk *parent;
	int ret;

	parent = clk_get_parent(hw->clk);
	parent_cbus = to_clk_cbus_shared(__clk_get_hw(parent));

	/*
	 * Defer rounding requests until aggregated. BW users must not be
	 * rounded at all, others just clipped to bus range (some clients
	 * may use round api to find limits)
	 */
	if (shared->u.shared_bus_user.mode != SHARED_BW) {
		if (!parent_cbus->max_rate) {
			ret = clk_round_rate(parent, ULONG_MAX);
			if (!IS_ERR_VALUE(ret))
				parent_cbus->max_rate = ret;
		}

		if (rate > parent_cbus->max_rate)
			rate = parent_cbus->max_rate;
		else if (rate < parent_cbus->min_rate)
			rate = parent_cbus->min_rate;
	}
	return rate;
}

static unsigned long clk_shared_recalc_rate(struct clk_hw *hw,
				unsigned long parent_rate)
{
	struct tegra_clk_cbus_shared *shared = to_clk_cbus_shared(hw);

	return shared->u.shared_bus_user.rate;
}

static int clk_system_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	int err = 0;
	struct tegra_clk_cbus_shared *system = to_clk_cbus_shared(hw);
	struct clk *pclk = system->u.system.pclk->clk, *parent;

	if (system->rate_updating)
		return 0;

	system->rate_updating = true;

	if (rate >= clk_round_rate(pclk, 0) * 2) {
		unsigned long hclk_rate;

		hclk_rate = clk_get_rate(system->u.system.hclk->clk);

		err = clk_set_rate(pclk, DIV_ROUND_UP(hclk_rate, 2));
		if (err) {
			pr_err("failed to set %s rate to %lu: %d\n",
				__clk_get_name(pclk),
				DIV_ROUND_UP(hclk_rate, 2), err);
			system->rate_updating = false;
			return err;
		}
	}

	if (rate <= system->u.system.threshold)
		parent = system->u.system.sclk_low->clk;
	else
		parent = system->u.system.sclk_high->clk;

	err = clk_set_rate(parent, rate);
	if (err) {
		pr_err("Failed to set sclk source %s to %lu\n",
			__clk_get_name(parent), rate);
		system->rate_updating = false;
		return err;
	}

	err = clk_set_parent(clk_get_parent(hw->clk), parent);
	if (err) {
		pr_err("Failed to switch sclk source to %s\n",
			__clk_get_name(parent));
		system->rate_updating = false;
		return err;
	}

	if (rate < clk_round_rate(pclk, 0) * 2) {
		unsigned long hclk_rate;

		hclk_rate = clk_get_rate(system->u.system.hclk->clk);

		err = clk_set_rate(pclk, hclk_rate);
		if (err)
			pr_err("failed to set %s rate to %lu: %d\n",
				__clk_get_name(pclk), hclk_rate, err);
	}

	system->rate_updating = false;
	return err;
}

static long clk_system_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
	struct clk *new_parent;
	unsigned long round_rate;
	struct tegra_clk_cbus_shared *system = to_clk_cbus_shared(hw);

	rate = max(rate, system->min_rate);

	if (rate <= system->u.system.threshold)
		new_parent = system->u.system.sclk_low->clk;
	else
		new_parent = system->u.system.sclk_high->clk;

	round_rate = clk_round_rate(new_parent, rate);

	if (new_parent == system->u.system.sclk_high->clk) {
		if (round_rate <= system->u.system.threshold)
			round_rate = system->u.system.threshold;
	}

	return round_rate;
}

static unsigned long clk_system_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	return parent_rate;
}

static const struct clk_ops tegra_clk_system_ops = {
	.recalc_rate = clk_system_recalc_rate,
	.round_rate = clk_system_round_rate,
	.set_rate = clk_system_set_rate,
};

static const struct clk_ops tegra_clk_cbus_ops = {
	.recalc_rate = clk_cbus_recalc_rate,
	.round_rate = clk_cbus_round_rate,
	.set_rate = clk_cbus_set_rate,
};

static const struct clk_ops tegra_clk_shared_ops = {
	.prepare = clk_shared_prepare,
	.unprepare = clk_shared_unprepare,
	.set_rate = clk_shared_set_rate,
	.round_rate = clk_shared_round_rate,
	.recalc_rate = clk_shared_recalc_rate,
};

static const struct clk_ops tegra_clk_shared_master_ops;

struct clk *tegra_clk_register_sbus_cmplx(const char *name,
		const char *parent, unsigned long flags,
		const char *pclk, const char *hclk,
		const char *sclk_low, const char *sclk_high,
		unsigned long threshold, unsigned long min_rate,
		unsigned long max_rate)
{
	struct clk *parent_clk, *c;
	struct clk_init_data init;
	struct tegra_clk_cbus_shared *system;

	system = kzalloc(sizeof(*system), GFP_KERNEL);
	if (!system)
		return ERR_PTR(-ENOMEM);

	parent_clk = __clk_lookup(parent);

	if (IS_ERR(parent_clk)) {
		kfree(system);
		return parent_clk;
	}

	c = __clk_lookup(pclk);
	if (IS_ERR(c)) {
		kfree(system);
		return c;
	}
	system->u.system.pclk = __clk_get_hw(c);
	c = __clk_lookup(hclk);
	if (IS_ERR(c)) {
		kfree(system);
		return c;
	}
	system->u.system.hclk = __clk_get_hw(c);
	c = __clk_lookup(sclk_low);
	if (IS_ERR(c)) {
		kfree(system);
		return c;
	}
	system->u.system.sclk_low = __clk_get_hw(c);
	c = __clk_lookup(sclk_high);
	if (IS_ERR(c)) {
		kfree(system);
		return c;
	}
	system->u.system.sclk_high = __clk_get_hw(c);

	system->u.system.threshold = threshold;

	system->min_rate = min_rate;
	system->max_rate = max_rate;

	INIT_LIST_HEAD(&system->shared_bus_list);

	flags |= CLK_GET_RATE_NOCACHE;

	init.name = name;
	init.ops = &tegra_clk_system_ops;
	init.flags = flags;
	init.parent_names = &parent;
	init.num_parents = 1;

	system->hw.init = &init;

	c = clk_register(NULL, &system->hw);
	if (!c)
		kfree(system);

	return c;
}

struct clk *tegra_clk_register_cbus(const char *name,
		const char *parent, unsigned long flags,
		const char *backup, unsigned long min_rate,
		unsigned long max_rate)
{
	struct tegra_clk_cbus_shared *cbus;
	struct clk_init_data init;
	struct clk *backup_clk, *c;

	cbus = kzalloc(sizeof(*cbus), GFP_KERNEL);
	if (!cbus)
		return ERR_PTR(-ENOMEM);

	backup_clk = __clk_lookup(backup);
	if (IS_ERR(backup_clk)) {
		kfree(cbus);
		return backup_clk;
	}

	cbus->shared_bus_backup = backup_clk;
	cbus->min_rate = min_rate;
	cbus->max_rate = max_rate;

	INIT_LIST_HEAD(&cbus->shared_bus_list);

	flags |= CLK_GET_RATE_NOCACHE;

	init.name = name;
	init.ops = &tegra_clk_cbus_ops;
	init.flags = flags;
	init.parent_names = &parent;
	init.num_parents = 1;

	/* Data in .init is copied by clk_register(), so stack variable OK */
	cbus->hw.init = &init;

	c = clk_register(NULL, &cbus->hw);
	if (!c)
		kfree(cbus);

	return c;
}

struct clk *tegra_clk_register_shared(const char *name,
		const char **parent, u8 num_parents, unsigned long flags,
		enum shared_bus_users_mode mode, const char *client)
{
	struct tegra_clk_cbus_shared *shared;
	struct clk_init_data init;
	struct tegra_clk_cbus_shared *parent_cbus;
	struct clk *client_clk, *parent_clk, *c;

	if (num_parents > 2)
		return ERR_PTR(-EINVAL);

	parent_clk = __clk_lookup(parent[0]);
	if (IS_ERR(parent_clk))
		return parent_clk;

	parent_cbus = to_clk_cbus_shared(__clk_get_hw(parent_clk));

	shared = kzalloc(sizeof(*shared), GFP_KERNEL);
	if (!shared)
		return ERR_PTR(-ENOMEM);

	if (client) {
		client_clk = __clk_lookup(client);
		if (IS_ERR(client_clk)) {
			kfree(shared);
			return client_clk;
		}
		shared->u.shared_bus_user.client = client_clk;
		shared->magic = TEGRA_CLK_SHARED_MAGIC;
	}

	shared->u.shared_bus_user.mode = mode;
	if (mode == SHARED_CEILING)
		shared->u.shared_bus_user.rate = parent_cbus->max_rate;
	else
		shared->u.shared_bus_user.rate = clk_get_rate(parent_clk);

	shared->flags = flags;

	if (num_parents > 1) {
		struct clk *c = __clk_lookup(parent[1]);

		if (IS_ERR(c)) {
			kfree(shared);
			return c;
		}

		shared->u.shared_bus_user.inputs[0] = parent_clk;
		shared->u.shared_bus_user.inputs[1] = c;
	}
	shared->max_rate = parent_cbus->max_rate;

	INIT_LIST_HEAD(&shared->u.shared_bus_user.node);

	list_add_tail(&shared->u.shared_bus_user.node,
			&parent_cbus->shared_bus_list);

	flags |= CLK_GET_RATE_NOCACHE;

	init.name = name;
	init.ops = &tegra_clk_shared_ops;
	init.flags = flags;
	init.parent_names = parent;
	init.num_parents = 1;

	/* Data in .init is copied by clk_register(), so stack variable OK */
	shared->hw.init = &init;

	c = clk_register(NULL, &shared->hw);
	if (!c)
		kfree(shared);

	return c;
}

/*
 * Not all shared clocks have a cbus clock as parent. The parent clock however
 * provides the head of the shared clock list. This clock provides a placeholder
 * for the head.
*/
struct clk *tegra_clk_register_shared_master(const char *name,
		const char *parent, unsigned long flags,
		unsigned long min_rate, unsigned long max_rate)
{
	struct tegra_clk_cbus_shared *master;
	struct clk_init_data init;
	struct clk *c;

	master = kzalloc(sizeof(*master), GFP_KERNEL);
	if (!master)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&master->shared_bus_list);

	flags |= CLK_SET_RATE_PARENT | CLK_GET_RATE_NOCACHE;

	init.name = name;
	init.ops = &tegra_clk_shared_master_ops;
	init.flags = flags;
	init.parent_names = &parent;
	init.num_parents = 1;

	master->min_rate = min_rate;
	master->max_rate = max_rate;

	/* Data in .init is copied by clk_register(), so stack variable OK */
	master->hw.init = &init;

	c = clk_register(NULL, &master->hw);
	if (!c)
		kfree(master);

	return c;
}
