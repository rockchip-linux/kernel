/*
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/list_sort.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/cpu.h>

#include <soc/tegra/tegra-dvfs.h>

struct dvfs_rail *tegra_cpu_rail;
struct dvfs_rail *tegra_core_rail;

bool core_dvfs_started;
int core_dvfs_lock_cnt;

static LIST_HEAD(dvfs_rail_list);
static DEFINE_MUTEX(dvfs_lock);

static int dvfs_rail_update(struct dvfs_rail *rail);

static inline int tegra_dvfs_rail_get_disable_level(struct dvfs_rail *rail)
{
	return rail->disable_millivolts ? : rail->nominal_millivolts;
}

static inline int tegra_dvfs_rail_get_suspend_level(struct dvfs_rail *rail)
{
	return rail->suspend_millivolts ? : rail->nominal_millivolts;
}

void tegra_dvfs_add_relationships(struct dvfs_relationship *rels, int n)
{
	int i;
	struct dvfs_relationship *rel;

	mutex_lock(&dvfs_lock);

	for (i = 0; i < n; i++) {
		rel = &rels[i];
		list_add_tail(&rel->from_node, &rel->to->relationships_from);
		list_add_tail(&rel->to_node, &rel->from->relationships_to);
	}

	mutex_unlock(&dvfs_lock);
}

int tegra_dvfs_init_rails(struct dvfs_rail *rails[], int n)
{
	int i, mv;

	mutex_lock(&dvfs_lock);

	for (i = 0; i < n; i++) {
		INIT_LIST_HEAD(&rails[i]->dvfs);
		INIT_LIST_HEAD(&rails[i]->relationships_from);
		INIT_LIST_HEAD(&rails[i]->relationships_to);

		mv = rails[i]->nominal_millivolts;
		if (rails[i]->disable_millivolts > mv)
			rails[i]->disable_millivolts = mv;
		if (rails[i]->suspend_millivolts > mv)
			rails[i]->suspend_millivolts = mv;

		rails[i]->millivolts = mv;
		rails[i]->new_millivolts = mv;
		if (!rails[i]->step)
			rails[i]->step = rails[i]->max_millivolts;
		if (!rails[i]->step_up)
			rails[i]->step_up = rails[i]->step;

		list_add_tail(&rails[i]->node, &dvfs_rail_list);

		if (!strcmp("vdd_cpu", rails[i]->reg_id))
			tegra_cpu_rail = rails[i];
		else if (!strcmp("vdd_core", rails[i]->reg_id))
			tegra_core_rail = rails[i];
	}

	mutex_unlock(&dvfs_lock);

	return 0;
};

static int dvfs_solve_relationship(struct dvfs_relationship *rel)
{
	return rel->solve(rel->from, rel->to);
}

static void dvfs_rail_stats_init(struct dvfs_rail *rail, int millivolts)
{
	int dvfs_rail_stats_range;

	if (!rail->stats.bin_uv)
		rail->stats.bin_uv = DVFS_RAIL_STATS_BIN;

	dvfs_rail_stats_range =
		(DVFS_RAIL_STATS_TOP_BIN - 1) * rail->stats.bin_uv / 1000;

	rail->stats.last_update = ktime_get();
	if (millivolts >= rail->min_millivolts) {
		int i = 1 + (2 * (millivolts - rail->min_millivolts) * 1000 +
			     rail->stats.bin_uv) / (2 * rail->stats.bin_uv);
		rail->stats.last_index = min(i, DVFS_RAIL_STATS_TOP_BIN);
	}

	if (rail->max_millivolts >
	    rail->min_millivolts + dvfs_rail_stats_range)
		pr_warn("tegra_dvfs: %s: stats above %d mV will be squashed\n",
			rail->reg_id,
			rail->min_millivolts + dvfs_rail_stats_range);
}

static void dvfs_rail_stats_update(
	struct dvfs_rail *rail, int millivolts, ktime_t now)
{
	rail->stats.time_at_mv[rail->stats.last_index] = ktime_add(
		rail->stats.time_at_mv[rail->stats.last_index], ktime_sub(
			now, rail->stats.last_update));
	rail->stats.last_update = now;

	if (rail->stats.off)
		return;

	if (millivolts >= rail->min_millivolts) {
		int i = 1 + (2 * (millivolts - rail->min_millivolts) * 1000 +
			     rail->stats.bin_uv) / (2 * rail->stats.bin_uv);
		rail->stats.last_index = min(i, DVFS_RAIL_STATS_TOP_BIN);
	} else if (millivolts == 0)
			rail->stats.last_index = 0;
}

static int dvfs_rail_set_voltage_reg(struct dvfs_rail *rail, int millivolts)
{
	int ret;

	ret = regulator_set_voltage(rail->reg,
		millivolts * 1000,
		rail->max_millivolts * 1000);
	if (!ret && rail == tegra_cpu_rail)
		ret = regulator_sync_voltage(rail->reg);

	return ret;
}

/**
 * dvfs_rail_set_voltage - set voltage in millivolts to specific rail
 *
 * @rail: struct dvfs_rail * power rail context
 * @millivolts: voltage in millivolts to be set to regulator
 *
 * Sets the voltage on a dvfs rail to a specific value, and updates any
 * rails that depend on this rail.
 */
static int dvfs_rail_set_voltage(struct dvfs_rail *rail, int millivolts)
{
	int ret = 0;
	struct dvfs_relationship *rel;
	int step, offset;
	int i;
	int steps;
	bool jmp_to_zero;

	if (!rail->reg) {
		if (millivolts == rail->millivolts)
			return 0;
		else
			return -EINVAL;
	}

	if (millivolts > rail->millivolts) {
		step = rail->step_up;
		offset = step;
	} else {
		step = rail->step;
		offset = -step;
	}

	if (rail->dfll_mode) {
		rail->millivolts = millivolts;
		rail->new_millivolts = millivolts;
		dvfs_rail_stats_update(rail, millivolts, ktime_get());
		return 0;
	}

	if (rail->disabled)
		return 0;

	rail->resolving_to = true;
	jmp_to_zero = rail->jmp_to_zero &&
			((millivolts == 0) || (rail->millivolts == 0));
	steps = jmp_to_zero ? 1 :
		DIV_ROUND_UP(abs(millivolts - rail->millivolts), step);

	for (i = 0; i < steps; i++) {
		if (!jmp_to_zero &&
		    (abs(millivolts - rail->millivolts) > step))
			rail->new_millivolts = rail->millivolts + offset;
		else
			rail->new_millivolts = millivolts;

		/*
		 * Before changing the voltage, tell each rail that depends
		 * on this rail that the voltage will change.
		 * This rail will be the "from" rail in the relationship,
		 * the rail that depends on this rail will be the "to" rail.
		 * from->millivolts will be the old voltage
		 * from->new_millivolts will be the new voltage
		 */
		list_for_each_entry(rel, &rail->relationships_to, to_node) {
			ret = dvfs_rail_update(rel->to);
			if (ret)
				goto out;
		}

		ret = dvfs_rail_set_voltage_reg(rail, rail->new_millivolts);
		if (ret) {
			pr_err("Failed to set dvfs regulator %s\n",
					rail->reg_id);
			goto out;
		}

		rail->millivolts = rail->new_millivolts;
		dvfs_rail_stats_update(rail, rail->millivolts, ktime_get());

		/*
		 * After changing the voltage, tell each rail that depends
		 * on this rail that the voltage has changed.
		 * from->millivolts and from->new_millivolts will be the
		 * new voltage
		 */
		list_for_each_entry(rel, &rail->relationships_to, to_node) {
			ret = dvfs_rail_update(rel->to);
			if (ret)
				goto out;
		}
	}

	if (unlikely(rail->millivolts != millivolts)) {
		pr_err("%s: rail didn't reach target %d in %d steps (%d)\n",
			__func__, millivolts, steps, rail->millivolts);
		ret = -EINVAL;
	}

out:
	rail->resolving_to = false;
	return ret;
}

static inline int dvfs_rail_apply_limits(struct dvfs_rail *rail, int millivolts)
{
	if (rail->override_millivolts)
		millivolts = rail->override_millivolts;

	return millivolts < rail->min_millivolts ?
		rail->min_millivolts : millivolts;
}

/**
 * dvfs_rail_update - update rail voltage
 *
 * @rail: struct dvfs_rail * power rail context
 *
 * Determine the minimum valid voltage for a rail, taking into account
 * the dvfs clocks and any rails that this rail depends on.  Calls
 * dvfs_rail_set_voltage with the new voltage, which will call
 * dvfs_rail_update on any rails that depend on this rail.
 */
static int dvfs_rail_update(struct dvfs_rail *rail)
{
	int millivolts = 0;
	struct dvfs *d;
	struct dvfs_relationship *rel;
	int ret = 0;
	int steps;

	if (rail->disabled)
		return 0;

	/* if dvfs is suspended, return and handle it during resume */
	if (rail->suspended)
		return 0;

	/* if regulators are not connected yet, return and handle it later */
	if (!rail->reg)
		return 0;

	/* if rail update is entered while resolving circular dependencies,
	   abort recursion */
	if (rail->resolving_to)
		return 0;

	/* Find the maximum voltage requested by any clock */
	list_for_each_entry(d, &rail->dvfs, reg_node)
		millivolts = max(d->cur_millivolts, millivolts);

	/* Apply offset and min/max limits if any clock is requesting voltage */
	if (millivolts)
		millivolts = dvfs_rail_apply_limits(rail, millivolts);
	/* Keep current voltage if regulator must not be disabled at run time */
	else if (!rail->jmp_to_zero) {
		WARN(1, "%s cannot be turned off by dvfs\n", rail->reg_id);
		return 0;
	}

	/*
	 * retry update if limited by from-relationship to account for
	 * circular dependencies
	 */
	steps = DIV_ROUND_UP(abs(millivolts - rail->millivolts), rail->step);
	for (; steps >= 0; steps--) {
		rail->new_millivolts = millivolts;

		/* Check any rails that this rail depends on */
		list_for_each_entry(rel, &rail->relationships_from, from_node)
			rail->new_millivolts = dvfs_solve_relationship(rel);

		if (rail->new_millivolts == rail->millivolts)
			break;

		ret = dvfs_rail_set_voltage(rail, rail->new_millivolts);
	}

	return ret;
}

static int dvfs_rail_connect_to_regulator(struct dvfs_rail *rail)
{
	struct regulator *reg;
	int v;

	if (!rail->reg) {
		reg = regulator_get(NULL, rail->reg_id);
		if (IS_ERR(reg)) {
			pr_err("tegra_dvfs: failed to connect %s rail\n",
			       rail->reg_id);
			return -EINVAL;
		}
		rail->reg = reg;
	}

	v = regulator_enable(rail->reg);
	if (v < 0) {
		pr_err("tegra_dvfs: failed on enabling regulator %s\n, err %d",
			rail->reg_id, v);
		return v;
	}

	v = regulator_get_voltage(rail->reg);
	if (v < 0) {
		pr_err("tegra_dvfs: failed initial get %s voltage\n",
		       rail->reg_id);
		return v;
	}

	rail->millivolts = v / 1000;
	rail->new_millivolts = rail->millivolts;
	dvfs_rail_stats_init(rail, rail->millivolts);

	return 0;
}

static inline const int *dvfs_get_millivolts(struct dvfs *d, unsigned long rate)
{
	if (tegra_dvfs_is_dfll_scale(d, rate))
		return d->dfll_millivolts;

	return d->millivolts;
}

static int __tegra_dvfs_set_rate(struct dvfs *d, unsigned long rate)
{
	int i = 0;
	int ret, mv;
	unsigned long *freqs = &d->freqs[0];
	const int *millivolts = dvfs_get_millivolts(d, rate);

	if (freqs == NULL || millivolts == NULL)
		return -ENODEV;

	/*
	 * On entry to dfll range limit 1st step to range bottom (full ramp of
	 * voltage/rate is completed automatically in dfll mode)
	 */
	if (tegra_dvfs_is_dfll_range_entry(d, rate))
		rate = d->use_dfll_rate_min;

	if (rate > freqs[d->num_freqs - 1]) {
		pr_warn("tegra-dvfs: rate %lu too high for dvfs on %s\n", rate,
			d->clk_name);
		return -EINVAL;
	}

	if (rate == 0) {
		d->cur_millivolts = 0;
	} else {
		while (i < d->num_freqs && rate > freqs[i])
			i++;

		if ((d->max_millivolts) &&
		    (millivolts[i] > d->max_millivolts)) {
			pr_warn("tegra-dvfs: voltage %d too high for dvfs on %s\n",
					millivolts[i], d->clk_name);
			return -EINVAL;
		}

		mv = millivolts[i];
		d->cur_millivolts = millivolts[i];
	}

	d->cur_rate = rate;

	ret = dvfs_rail_update(d->dvfs_rail);
	if (ret)
		pr_err("Failed to set regulator %s for clock %s to %d mV\n",
			d->dvfs_rail->reg_id, d->clk_name, d->cur_millivolts);

	return ret;
}

void tegra_dvfs_core_lock(void)
{
	int ret;
	struct dvfs_rail *rail = tegra_core_rail;

	if (!rail)
		return;

	mutex_lock(&dvfs_lock);
	if (core_dvfs_lock_cnt++ == 0) {
		rail->override_millivolts = rail->nominal_millivolts;
		if (!rail->disabled && !rail->suspended) {
			ret = dvfs_rail_update(rail);
			if (ret)
				pr_err("%s: failed to set override level: %d\n",
				       __func__, ret);
		}
	}
	mutex_unlock(&dvfs_lock);
}
EXPORT_SYMBOL(tegra_dvfs_core_lock);

void tegra_dvfs_core_unlock(void)
{
	int ret;
	struct dvfs_rail *rail = tegra_core_rail;

	if (!rail)
		return;

	mutex_lock(&dvfs_lock);
	if (--core_dvfs_lock_cnt == 0) {
		rail->override_millivolts = 0;
		if (!rail->disabled && !rail->suspended) {
			ret = dvfs_rail_update(rail);
			if (ret)
				pr_err("%s: failed to clear override level: %d\n",
				       __func__, ret);
		}
	}
	mutex_unlock(&dvfs_lock);
}
EXPORT_SYMBOL(tegra_dvfs_core_unlock);

int tegra_dvfs_get_core_nominal_millivolts(void)
{
	if (tegra_core_rail)
		return tegra_core_rail->nominal_millivolts;
	return -ENOENT;
}
EXPORT_SYMBOL(tegra_dvfs_get_core_nominal_millivolts);

static struct dvfs *tegra_clk_to_dvfs(struct clk *c)
{
	struct dvfs *d;
	struct dvfs_rail *rail;

	list_for_each_entry(rail, &dvfs_rail_list, node) {
		list_for_each_entry(d, &rail->dvfs, reg_node) {
			if (c == d->clk)
				return d;
		}
	}
	return NULL;
}

static int predict_millivolts(struct dvfs *d, const int *millivolts,
			      unsigned long rate)
{
	int i;

	if (!millivolts)
		return -ENODEV;

	for (i = 0; i < d->num_freqs; i++) {
		if (rate <= d->freqs[i])
			break;
	}

	if (i == d->num_freqs)
		return -EINVAL;

	return millivolts[i];
}

int opp_millivolts[MAX_DVFS_FREQS];
unsigned long opp_frequencies[MAX_DVFS_FREQS];

/**
 * tegra_get_cpu_fv_table - get CPU frequencies/voltages table
 *
 * @num_freqs: number of frequencies
 * @freqs: the array of frequencies
 * @mvs: the array of voltages
 *
 * Get the frequency and voltage table using CPU OPP which were built by the
 * DFLL driver.
 */
int tegra_get_cpu_fv_table(int *num_freqs, unsigned long **freqs, int **mvs)
{
	struct device *cpu_dev;
	struct dev_pm_opp *opp;
	unsigned long rate;
	int i, ret = 0;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev)
		return -EINVAL;

	mutex_lock(&dvfs_lock);
	for (i = 0, rate = 0;; rate++) {
		rcu_read_lock();
		opp = dev_pm_opp_find_freq_ceil(cpu_dev, &rate);
		if (IS_ERR(opp)) {
			rcu_read_unlock();
			break;
		}
		opp_frequencies[i] = rate;
		opp_millivolts[i++] = dev_pm_opp_get_voltage(opp);
		rcu_read_unlock();
	}
	if (!i) {
		ret = -EINVAL;
		goto out;
	}

	*num_freqs = i;
	*freqs = opp_frequencies;
	*mvs = opp_millivolts;
out:
	mutex_unlock(&dvfs_lock);
	return ret;
}
EXPORT_SYMBOL(tegra_get_cpu_fv_table);

/**
 * tegra_dvfs_predict_millivolts - return the safe voltage for running
 *					the clock at one sepcific rate
 *
 * @c: struct clk * the clock which needs the voltage info
 * @rate: the rate being predicted
 *
 * Extract the voltage table associated with the clock and return the safe
 * voltage for ticking the clock at the specified rate
 */
int tegra_dvfs_predict_millivolts(struct clk *c, unsigned long rate)
{
	int ret;
	const int *millivolts;
	struct dvfs *d;

	mutex_lock(&dvfs_lock);

	d = tegra_clk_to_dvfs(c);
	if (d == NULL) {
		mutex_unlock(&dvfs_lock);
		return -EINVAL;
	}

	if (!rate) {
		mutex_unlock(&dvfs_lock);
		return 0;
	}

	millivolts = dvfs_is_dfll_range(d, rate) ?
			d->dfll_millivolts :
			d->millivolts;

	ret = predict_millivolts(d, millivolts, rate);

	mutex_unlock(&dvfs_lock);

	return ret;
}
EXPORT_SYMBOL(tegra_dvfs_predict_millivolts);

int tegra_dvfs_set_fmax_at_vmin(struct clk *c, unsigned long f_max, int v_min)
{
	int i, ret = 0;
	struct dvfs *d;
	unsigned long f_min = 1000;	/* 1kHz min rate in DVFS tables */

	d = tegra_clk_to_dvfs(c);

	mutex_lock(&dvfs_lock);

	if (v_min >= d->max_millivolts) {
		pr_err("%s: new vmin %dmV is at/above max voltage %dmV\n",
		       __func__, v_min, d->max_millivolts);
		ret = -EINVAL;
		goto out;
	}

	/*
	 * dvfs table update:
	 * - for voltages below new v_min the respective frequencies are shifted
	 * below new f_max to the levels already present in the table; if the
	 * 1st table entry has frequency above new fmax, all entries below v_min
	 * are filled in with 1kHz (min rate used in DVFS tables).
	 * - for voltages above new v_min, the respective frequencies are
	 * increased to at least new f_max
	 * - if new v_min is already in the table set the respective frequency
	 * to new f_max
	 */
	for (i = 0; i < d->num_freqs; i++) {
		int mv = d->millivolts[i];
		unsigned long f = d->freqs[i];

		if (mv < v_min) {
			if (f >= f_max)
				d->freqs[i] = i ? d->freqs[i-1] : f_min;
		} else if (mv > v_min) {
			d->freqs[i] = max(f, f_max);
		} else {
			d->freqs[i] = f_max;
		}
	}
	ret = __tegra_dvfs_set_rate(d, d->cur_rate);

out:
	mutex_unlock(&dvfs_lock);

	return ret;
}
EXPORT_SYMBOL(tegra_dvfs_set_fmax_at_vmin);

/**
 * tegra_dvfs_set_rate - update rail voltage due to the clock rate change
 *
 * @c: struct clk * the clock which has changed rate
 * @rate: the changed rate
 *
 * Check if the voltage of the power rail need to be updated due to the clock
 * rate change.
 */
int tegra_dvfs_set_rate(struct clk *c, unsigned long rate)
{
	int ret = 0;
	struct dvfs *d;

	if (!core_dvfs_started)
		return ret;

	mutex_lock(&dvfs_lock);

	d = tegra_clk_to_dvfs(c);
	if (d)
		ret = __tegra_dvfs_set_rate(d, rate);

	mutex_unlock(&dvfs_lock);

	return ret;
}
EXPORT_SYMBOL(tegra_dvfs_set_rate);

/**
 * tegra_dvfs_get_rate - get current rate used for determining rail voltage
 *
 * @c: struct clk * clock we want to know the rate of used for determining
 *		    rail voltage
 *
 * Returns 0 if there is no dvfs for the clock.
 */
unsigned long tegra_dvfs_get_rate(struct clk *c)
{
	unsigned long rate = 0;
	struct dvfs *d;

	if (!core_dvfs_started)
		return rate;

	mutex_lock(&dvfs_lock);

	d = tegra_clk_to_dvfs(c);
	if (d)
		rate = d->cur_rate;

	mutex_unlock(&dvfs_lock);

	return rate;
}
EXPORT_SYMBOL(tegra_dvfs_get_rate);

/**
 * tegra_dvfs_get_freqs - export dvfs frequency array associated with the clock
 *
 * @c: struct clk * the clock which needs the frequency table
 * @freqs: the array of the frequencies
 * @num_freqs: number of the frequencies
 *
 * Check if the voltage of the power rail need to be updated due to the clock
 * rate change.
 */
int tegra_dvfs_get_freqs(struct clk *c, unsigned long **freqs, int *num_freqs)
{
	struct dvfs *d;

	d = tegra_clk_to_dvfs(c);
	if (d == NULL) {
		pr_err("Failed to get dvfs structure\n");
		return -ENOSYS;
	}

	*num_freqs = d->num_freqs;
	*freqs = d->freqs;

	return 0;
}
EXPORT_SYMBOL(tegra_dvfs_get_freqs);

static int tegra_dvfs_clk_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct clk_notifier_data *cnd = ptr;
	struct dvfs *d;

	d = tegra_clk_to_dvfs(cnd->clk);
	if (d == NULL)
		return NOTIFY_DONE;

	if (d->dvfs_rail == tegra_core_rail && !core_dvfs_started)
		return NOTIFY_DONE;

	if (!__clk_is_enabled(cnd->clk) && !__clk_is_prepared(cnd->clk))
		return NOTIFY_DONE;

	switch (event) {
	case PRE_RATE_CHANGE:
		if (cnd->old_rate < cnd->new_rate)
			tegra_dvfs_set_rate(cnd->clk, cnd->new_rate);
		break;
	case POST_RATE_CHANGE:
		if (cnd->old_rate > cnd->new_rate)
			tegra_dvfs_set_rate(cnd->clk, cnd->new_rate);
		break;
	case ABORT_RATE_CHANGE:
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block tegra_dvfs_nb = {
	.notifier_call = tegra_dvfs_clk_event,
	.priority = 1,
};

int tegra_setup_dvfs(struct clk *c, struct dvfs *d)
{
	int i;

	for (i = 0; i < MAX_DVFS_FREQS; i++) {
		if (d->millivolts[i] == 0)
			break;

		if (d->freqs_mult)
			d->freqs[i] *= d->freqs_mult;

		/* If final frequencies are 0, pad with previous frequency */
		if (d->freqs[i] == 0 && i > 1)
			d->freqs[i] = d->freqs[i - 1];
	}
	d->num_freqs = i;
	d->clk = c;

	mutex_lock(&dvfs_lock);
	list_add_tail(&d->reg_node, &d->dvfs_rail->dvfs);
	mutex_unlock(&dvfs_lock);

	return 0;
}

static bool tegra_dvfs_all_rails_suspended(void)
{
	struct dvfs_rail *rail;
	bool all_suspended = true;

	list_for_each_entry(rail, &dvfs_rail_list, node)
		if (!rail->suspended && !rail->disabled)
			all_suspended = false;

	return all_suspended;
}

static bool tegra_dvfs_from_rails_suspended_or_solved(struct dvfs_rail *to)
{
	struct dvfs_relationship *rel;
	bool all_suspended = true;

	list_for_each_entry(rel, &to->relationships_from, from_node)
		if (!rel->from->suspended && !rel->from->disabled &&
			!rel->solved_at_nominal)
			all_suspended = false;

	return all_suspended;
}

static int tegra_dvfs_suspend_one(void)
{
	struct dvfs_rail *rail;
	int mv;
	int ret = 0;

	list_for_each_entry(rail, &dvfs_rail_list, node) {
		if (!rail->suspended && !rail->disabled &&
		    tegra_dvfs_from_rails_suspended_or_solved(rail)) {
			mv = tegra_dvfs_rail_get_suspend_level(rail);
			mv = dvfs_rail_apply_limits(rail, mv);
			/* apply suspend limit only if it is above current mv */
			if (mv >= rail->millivolts)
				ret = dvfs_rail_set_voltage(rail, mv);
			if (ret) {
				pr_err("tegra_dvfs: failed %s suspend at %d\n",
					rail->reg_id, rail->millivolts);
				return ret;
			}

			rail->suspended = true;
			return 0;
		}
	}
	return -EINVAL;
}

static void tegra_dvfs_resume(void)
{
	struct dvfs *d;
	struct dvfs_rail *rail;

	mutex_lock(&dvfs_lock);

	list_for_each_entry(d, &tegra_core_rail->dvfs, reg_node) {
		if (!__clk_is_enabled(d->clk) && d->cur_rate == 0)
			continue;
		d->cur_rate = __clk_get_rate(d->clk);
		d->cur_millivolts = predict_millivolts(d, d->millivolts,
							d->cur_rate);
		if (d->cur_millivolts < 0)
			d->cur_millivolts = d->max_millivolts;
	}

	list_for_each_entry(rail, &dvfs_rail_list, node)
		rail->suspended = false;

	list_for_each_entry(rail, &dvfs_rail_list, node)
		dvfs_rail_update(rail);

	mutex_unlock(&dvfs_lock);
}

static int tegra_dvfs_suspend(void)
{
	int ret = 0;

	mutex_lock(&dvfs_lock);

	while (!tegra_dvfs_all_rails_suspended()) {
		ret = tegra_dvfs_suspend_one();
		if (ret)
			break;
	}

	mutex_unlock(&dvfs_lock);

	if (ret)
		tegra_dvfs_resume();

	return ret;
}

static int tegra_dvfs_pm_notifier_event(struct notifier_block *nb,
				unsigned long event, void *data)
{
	if (event == PM_SUSPEND_PREPARE) {
		if (tegra_dvfs_suspend())
			return NOTIFY_STOP;
		pr_info("tegra_dvfs: suspended\n");
	} else if (event == PM_POST_SUSPEND) {
		tegra_dvfs_resume();
		pr_info("tegra_dvfs: resumed\n");
	}
	return NOTIFY_OK;
};

static struct notifier_block tegra_dvfs_pm_nb = {
	.notifier_call = tegra_dvfs_pm_notifier_event,
	.priority = -1,
};

static int tegra_dvfs_reboot_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{
	switch (event) {
	case SYS_RESTART:
	case SYS_HALT:
	case SYS_POWER_OFF:
		tegra_dvfs_suspend();
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block tegra_dvfs_reboot_nb = {
	.notifier_call = tegra_dvfs_reboot_notify,
};

static void __tegra_dvfs_rail_disable(struct dvfs_rail *rail)
{
	int ret = -EPERM;
	int mv;

	if (rail->dfll_mode) {
		rail->disabled = true;
		return;
	}

	mv = tegra_dvfs_rail_get_disable_level(rail);
	mv = dvfs_rail_apply_limits(rail, mv);

	if (mv >= rail->millivolts)
		ret = dvfs_rail_set_voltage(rail, mv);
	if (ret) {
		pr_err("tegra_dvfs: failed to disable %s at %d\n",
		       rail->reg_id, rail->millivolts);
		return;
	}
	rail->disabled = true;
}

static void __tegra_dvfs_rail_enable(struct dvfs_rail *rail)
{
	rail->disabled = false;
	dvfs_rail_update(rail);
}

void tegra_dvfs_rail_enable(struct dvfs_rail *rail)
{
	if (!rail)
		return;

	mutex_lock(&dvfs_lock);

	if (rail->disabled)
		__tegra_dvfs_rail_enable(rail);

	mutex_unlock(&dvfs_lock);
}

void tegra_dvfs_rail_disable(struct dvfs_rail *rail)
{
	if (!rail)
		return;

	mutex_lock(&dvfs_lock);
	if (rail->disabled)
		goto out;

	__tegra_dvfs_rail_disable(rail);
out:
	mutex_unlock(&dvfs_lock);
}

bool tegra_dvfs_is_dfll_range(struct clk *c, unsigned long rate)
{
	struct dvfs *d;

	d = tegra_clk_to_dvfs(c);
	if (d == NULL) {
		pr_err("Failed to get dvfs structure\n");
		return false;
	}

	return dvfs_is_dfll_range(d, rate);
}
EXPORT_SYMBOL(tegra_dvfs_is_dfll_range);

int tegra_dvfs_set_dfll_range(struct clk *c, int range)
{
	struct dvfs *d;
	int ret = -EINVAL;

	mutex_lock(&dvfs_lock);
	d = tegra_clk_to_dvfs(c);
	if (d == NULL) {
		pr_err("Failed to get dvfs structure\n");
		goto out;
	}

	if (!d->dfll_millivolts)
		goto out;

	if ((range < DFLL_RANGE_NONE) || (range > DFLL_RANGE_HIGH_RATES))
		goto out;

	d->range = range;
	ret = 0;
out:
	mutex_unlock(&dvfs_lock);
	return ret;
}
EXPORT_SYMBOL(tegra_dvfs_set_dfll_range);

int tegra_dvfs_dfll_mode_set(struct clk *c, unsigned long rate)
{
	struct dvfs *d;

	mutex_lock(&dvfs_lock);

	d = tegra_clk_to_dvfs(c);
	if (d == NULL) {
		pr_err("Failed to get dvfs structure\n");
		mutex_unlock(&dvfs_lock);
		return -EINVAL;
	}

	if (!d->dvfs_rail->dfll_mode) {
		d->dvfs_rail->dfll_mode = true;
		__tegra_dvfs_set_rate(d, rate);
	}

	mutex_unlock(&dvfs_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_dvfs_dfll_mode_set);

int tegra_dvfs_dfll_mode_clear(struct clk *c, unsigned long rate)
{
	int ret = 0;
	struct dvfs *d;

	mutex_lock(&dvfs_lock);

	d = tegra_clk_to_dvfs(c);
	if (d == NULL) {
		pr_err("Failed to get dvfs structure\n");
		mutex_unlock(&dvfs_lock);
		return -EINVAL;
	}

	if (d->dvfs_rail->dfll_mode) {
		d->dvfs_rail->dfll_mode = false;
		d->dvfs_rail->millivolts = regulator_get_voltage(
				d->dvfs_rail->reg) / 1000;
		if (d->dvfs_rail->disabled) {
			d->dvfs_rail->disabled = false;
			__tegra_dvfs_rail_disable(d->dvfs_rail);
		}
		ret = __tegra_dvfs_set_rate(d, rate);
	}

	mutex_unlock(&dvfs_lock);

	return ret;
}
EXPORT_SYMBOL(tegra_dvfs_dfll_mode_clear);

int tegra_dvfs_get_dfll_threshold(struct clk *c, unsigned long *rate)
{
	struct dvfs *d;

	d = tegra_clk_to_dvfs(c);
	if (d == NULL) {
		pr_err("Failed to get dvfs structure\n");
		return -EINVAL;
	}

	if (d->dvfs_rail && d->use_dfll_rate_min)
		*rate = d->use_dfll_rate_min;

	return 0;
}
EXPORT_SYMBOL(tegra_dvfs_get_dfll_threshold);

static int tegra_config_dvfs(struct dvfs_rail *rail)
{
	int i;
	struct dvfs *d;

	list_for_each_entry(d, &rail->dvfs, reg_node) {
		if (__clk_is_enabled(d->clk) || __clk_is_prepared(d->clk)) {
			d->cur_rate = __clk_get_rate(d->clk);
			d->cur_millivolts = d->max_millivolts;

			for (i = 0; i < d->num_freqs; i++)
				if (d->cur_rate <= d->freqs[i])
					break;

			if (i != d->num_freqs)
				d->cur_millivolts = d->millivolts[i];
		}

		mutex_unlock(&dvfs_lock);
		clk_notifier_register(d->clk, &tegra_dvfs_nb);
		mutex_lock(&dvfs_lock);
	}

	return 0;
}

static int tegra_dvfs_regulator_init(void)
{
	bool connected = true;
	struct dvfs_rail *rail;

	mutex_lock(&dvfs_lock);

	list_for_each_entry(rail, &dvfs_rail_list, node) {
		if (dvfs_rail_connect_to_regulator(rail)) {
			connected = false;
			if (!rail->disabled)
				__tegra_dvfs_rail_disable(rail);
		}
	}

	if (!connected) {
		mutex_unlock(&dvfs_lock);
		return -EPROBE_DEFER;
	}

	list_for_each_entry(rail, &dvfs_rail_list, node) {
		tegra_config_dvfs(rail);
		__tegra_dvfs_rail_enable(rail);
	}

	core_dvfs_started = true;

	mutex_unlock(&dvfs_lock);

	register_pm_notifier(&tegra_dvfs_pm_nb);
	register_reboot_notifier(&tegra_dvfs_reboot_nb);

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int dvfs_tree_sort_cmp(void *p, struct list_head *a, struct list_head *b)
{
	struct dvfs *da = list_entry(a, struct dvfs, reg_node);
	struct dvfs *db = list_entry(b, struct dvfs, reg_node);
	int ret;

	ret = strcmp(da->dvfs_rail->reg_id, db->dvfs_rail->reg_id);
	if (ret != 0)
		return ret;

	if (da->cur_millivolts < db->cur_millivolts)
		return 1;
	if (da->cur_millivolts > db->cur_millivolts)
		return -1;

	return strcmp(da->clk_name, db->clk_name);
}

/* To emulate and show rail relations with 0 mV on dependent rail-to */
static struct dvfs_rail show_to;
static struct dvfs_relationship show_rel;

static int dvfs_tree_show(struct seq_file *s, void *data)
{
	struct dvfs *d;
	struct dvfs_rail *rail;
	struct dvfs_relationship *rel;

	seq_puts(s, "   clock           rate       mV\n");
	seq_puts(s, "-------------------------------------\n");

	mutex_lock(&dvfs_lock);

	list_for_each_entry(rail, &dvfs_rail_list, node) {
		seq_printf(s, "%s %d mV%s:\n", rail->reg_id,
			   rail->stats.off ? 0 : rail->millivolts,
			   rail->dfll_mode ? " dfll mode" :
				rail->disabled ? " disabled" : "");
		list_for_each_entry(rel, &rail->relationships_from, from_node) {
			show_rel = *rel;
			show_rel.to = &show_to;
			show_to = *rel->to;
			show_to.millivolts = show_to.new_millivolts = 0;
			seq_printf(s, "   %-10s %-7d mV %-4d mV .. %-4d mV\n",
				rel->from->reg_id, rel->from->millivolts,
				dvfs_solve_relationship(&show_rel),
				dvfs_solve_relationship(rel));
		}
		seq_printf(s, "   nominal    %-7d mV\n",
			   rail->nominal_millivolts);

		list_sort(NULL, &rail->dvfs, dvfs_tree_sort_cmp);

		list_for_each_entry(d, &rail->dvfs, reg_node) {
			seq_printf(s, "   %-15s %-10lu %-4d mV\n", d->clk_name,
				d->cur_rate, d->cur_millivolts);
		}
	}

	mutex_unlock(&dvfs_lock);

	return 0;
}

static int dvfs_tree_open(struct inode *inode, struct file *file)
{
	return single_open(file, dvfs_tree_show, inode->i_private);
}

static const struct file_operations dvfs_tree_fops = {
	.open		= dvfs_tree_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dvfs_table_show(struct seq_file *s, void *data)
{
	int i;
	struct dvfs *d;
	struct dvfs_rail *rail;
	const int *v_pll, *last_v_pll = NULL;
	const int *v_dfll, *last_v_dfll = NULL;

	seq_puts(s, "DVFS tables: units mV/MHz\n");

	mutex_lock(&dvfs_lock);

	list_for_each_entry(rail, &dvfs_rail_list, node) {
		list_for_each_entry(d, &rail->dvfs, reg_node) {
			bool mv_done = false;
			v_pll = d->millivolts;
			v_dfll = d->dfll_millivolts;

			if (v_pll && (last_v_pll != v_pll)) {
				if (!mv_done) {
					seq_puts(s, "\n");
					mv_done = true;
				}
				last_v_pll = v_pll;
				seq_printf(s, "%-16s", rail->reg_id);
				for (i = 0; i < d->num_freqs; i++)
					seq_printf(s, "%7d", v_pll[i]);
				seq_puts(s, "\n");
			}

			if (v_dfll && (last_v_dfll != v_dfll)) {
				if (!mv_done) {
					seq_puts(s, "\n");
					mv_done = true;
				}
				last_v_dfll = v_dfll;
				seq_printf(s, "%-8s (dfll) ", rail->reg_id);
				for (i = 0; i < d->num_freqs; i++)
					seq_printf(s, "%7d", v_dfll[i]);
				seq_puts(s, "\n");
			}

			seq_printf(s, "%-16s", d->clk_name);
			for (i = 0; i < d->num_freqs; i++) {
				unsigned int f = d->freqs[i]/100000;
				seq_printf(s, " %4u.%u", f/10, f%10);
			}
			seq_puts(s, "\n");
		}
	}

	mutex_unlock(&dvfs_lock);

	return 0;
}

static int dvfs_table_open(struct inode *inode, struct file *file)
{
	return single_open(file, dvfs_table_show, inode->i_private);
}

static const struct file_operations dvfs_table_fops = {
	.open		= dvfs_table_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int rail_stats_save_to_buf(char *buf, int len)
{
	int i;
	struct dvfs_rail *rail;
	char *str = buf;
	char *end = buf + len;

	str += scnprintf(str, end - str, "%-12s %-10s\n", "millivolts", "time");

	mutex_lock(&dvfs_lock);

	list_for_each_entry(rail, &dvfs_rail_list, node) {
		str += scnprintf(str, end - str, "%s (bin: %d.%dmV)\n",
			   rail->reg_id,
			   rail->stats.bin_uv / 1000,
			   (rail->stats.bin_uv / 10) % 100);

		dvfs_rail_stats_update(rail, -1, ktime_get());

		str += scnprintf(str, end - str, "%-12d %-10llu\n", 0,
			cputime64_to_clock_t(msecs_to_jiffies(
				ktime_to_ms(rail->stats.time_at_mv[0]))));

		for (i = 1; i <= DVFS_RAIL_STATS_TOP_BIN; i++) {
			ktime_t ktime_zero = ktime_set(0, 0);
			if (ktime_equal(rail->stats.time_at_mv[i], ktime_zero))
				continue;
			str += scnprintf(str, end - str, "%-12d %-10llu\n",
				rail->min_millivolts +
				(i - 1) * rail->stats.bin_uv / 1000,
				cputime64_to_clock_t(msecs_to_jiffies(
					ktime_to_ms(rail->stats.time_at_mv[i])))
			);
		}
	}
	mutex_unlock(&dvfs_lock);
	return str - buf;
}

static int rail_stats_show(struct seq_file *s, void *data)
{
	char *buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	int size = 0;

	if (!buf)
		return -ENOMEM;

	size = rail_stats_save_to_buf(buf, PAGE_SIZE);
	seq_write(s, buf, size);
	kfree(buf);
	return 0;
}

static int rail_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, rail_stats_show, inode->i_private);
}

static const struct file_operations rail_stats_fops = {
	.open		= rail_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dvfs_debugfs_init(void)
{
	struct dentry *d_root, *d;

	d_root = debugfs_create_dir("tegra_dvfs", NULL);
	if (!d_root)
		return -ENOMEM;

	d = debugfs_create_file("dvfs", S_IRUGO, d_root, NULL,
		&dvfs_tree_fops);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_file("dvfs_table", S_IRUGO, d_root, NULL,
		&dvfs_table_fops);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_file("rails", S_IRUGO, d_root, NULL,
		&rail_stats_fops);
	if (!d)
		return -ENOMEM;

	return 0;
}

#endif

static struct {
	char *compat;
	int (*init)(void);
} dvfs_init_funcs[] = {
	{ "nvidia,tegra124", tegra124_init_dvfs },
};

static int tegra_dvfs_probe(struct platform_device *pdev)
{
	int i;
	int ret = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(dvfs_init_funcs); i++) {
		if (of_machine_is_compatible(dvfs_init_funcs[i].compat)) {
			ret = dvfs_init_funcs[i].init();
			if (!ret)
				break;
			else
				goto out;
		}
	}
	if (i == ARRAY_SIZE(dvfs_init_funcs))
		goto out;

	ret = tegra_dvfs_regulator_init();
	if (ret)
		goto out;

#ifdef CONFIG_DEBUG_FS
	dvfs_debugfs_init();
#endif
	return 0;
out:
	return ret;
}

static int tegra_dvfs_remove(struct platform_device *pdev)
{
	struct dvfs *d;

	core_dvfs_started = false;

	unregister_pm_notifier(&tegra_dvfs_reboot_nb);
	unregister_pm_notifier(&tegra_dvfs_pm_nb);

	list_for_each_entry(d, &tegra_core_rail->dvfs, reg_node) {
		clk_notifier_unregister(d->clk, &tegra_dvfs_nb);
	}

	return 0;
}

static struct platform_driver tegra_dvfs_platdrv = {
	.driver = {
		.name	= "tegra-dvfs",
		.owner	= THIS_MODULE,
	},
	.probe		= tegra_dvfs_probe,
	.remove		= tegra_dvfs_remove,
};
module_platform_driver(tegra_dvfs_platdrv);

int __init tegra_dvfs_init(void)
{
	struct platform_device_info devinfo = { .name = "tegra-dvfs", };

	platform_device_register_full(&devinfo);

	return 0;
}
device_initcall(tegra_dvfs_init);
