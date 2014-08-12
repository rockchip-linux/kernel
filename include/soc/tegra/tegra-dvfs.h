/*
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
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

#ifndef _TEGRA_DVFS_H_
#define _TEGRA_DVFS_H_

#define MAX_DVFS_FREQS		40
#define DVFS_RAIL_STATS_TOP_BIN	100
#define DVFS_RAIL_STATS_BIN	10000

/*
 * dvfs_relationship between to rails, "from" and "to"
 * when the rail changes, it will call dvfs_rail_update on the rails
 * in the relationship_to list.
 * when determining the voltage to set a rail to, it will consider each
 * rail in the relationship_from list.
 */
struct dvfs_relationship {
	struct dvfs_rail *to;
	struct dvfs_rail *from;
	int (*solve)(struct dvfs_rail *, struct dvfs_rail *);

	struct list_head to_node; /* node in relationship_to list */
	struct list_head from_node; /* node in relationship_from list */
	bool solved_at_nominal;
};

struct cpu_pll_fv_table {
	int freq;
	int volt;
};

struct cpu_dvfs {
	int speedo_id;
	int process_id;

	int max_mv;
	int min_mv;
	struct cpu_pll_fv_table fv_table[MAX_DVFS_FREQS];
};

struct rail_stats {
	ktime_t time_at_mv[DVFS_RAIL_STATS_TOP_BIN + 1];
	ktime_t last_update;
	int last_index;
	bool off;
	int bin_uv;
};

struct rail_alignment {
	int offset_uv;
	int step_uv;
};

struct dvfs_rail {
	const char *reg_id;
	int min_millivolts;
	int max_millivolts;
	int nominal_millivolts;
	int override_millivolts;

	int step;
	int step_up;
	bool jmp_to_zero;
	bool disabled;
	bool resolving_to;

	struct list_head node;  /* node in dvfs_rail_list */
	struct list_head dvfs;  /* list head of attached dvfs clocks */
	struct list_head relationships_to;
	struct list_head relationships_from;
	struct regulator *reg;
	int millivolts;
	int new_millivolts;
	int disable_millivolts;
	int suspend_millivolts;

	bool suspended;
	bool dfll_mode;

	struct rail_alignment alignment;
	struct rail_stats stats;
};

enum dfll_range {
	DFLL_RANGE_NONE = 0,
	DFLL_RANGE_ALL_RATES,
	DFLL_RANGE_HIGH_RATES,
};

struct dvfs {
	const char *clk_name;
	struct clk *clk;
	int speedo_id;
	int process_id;

	int freqs_mult;
	unsigned long freqs[MAX_DVFS_FREQS];
	const int *millivolts;
	const int *dfll_millivolts;
	struct dvfs_rail *dvfs_rail;
	bool auto_dvfs;

	int max_millivolts;
	int num_freqs;

	enum dfll_range	range;
	unsigned long use_dfll_rate_min;

	int cur_millivolts;
	unsigned long cur_rate;
	struct list_head node;
	struct list_head reg_node;
};

static inline bool tegra_dvfs_rail_is_dfll_mode(struct dvfs_rail *rail)
{
	return rail ? rail->dfll_mode : false;
}

static inline bool tegra_dvfs_is_dfll_range_entry(struct dvfs *d,
						  unsigned long rate)
{
	return  d->cur_rate && d->dvfs_rail && (!d->dvfs_rail->dfll_mode) &&
		(d->range == DFLL_RANGE_HIGH_RATES) &&
		(rate >= d->use_dfll_rate_min) &&
		(d->cur_rate < d->use_dfll_rate_min);
}

static inline bool tegra_dvfs_is_dfll_scale(struct dvfs *d, unsigned long rate)
{
	return tegra_dvfs_rail_is_dfll_mode(d->dvfs_rail) ||
		tegra_dvfs_is_dfll_range_entry(d, rate);
}

static inline bool dvfs_is_dfll_range(struct dvfs *d, unsigned long rate)
{
	return (d->range == DFLL_RANGE_ALL_RATES) ||
		((d->range == DFLL_RANGE_HIGH_RATES) &&
		(rate >= d->use_dfll_rate_min));
}

#ifdef CONFIG_TEGRA_DVFS
int tegra_dvfs_init(void);
int tegra_dvfs_dfll_mode_set(struct clk *c, unsigned long rate);
int tegra_dvfs_dfll_mode_clear(struct clk *c, unsigned long rate);
int tegra_dvfs_get_dfll_threshold(struct clk *c, unsigned long *rate);
int tegra_dvfs_set_rate(struct clk *c, unsigned long rate);
unsigned long tegra_dvfs_get_rate(struct clk *c);
int tegra_dvfs_get_freqs(struct clk *c, unsigned long **freqs, int *num_freqs);
int tegra_setup_dvfs(struct clk *c, struct dvfs *d);
int tegra_dvfs_init_rails(struct dvfs_rail *dvfs_rails[], int n);
void tegra_dvfs_add_relationships(struct dvfs_relationship *rels, int n);
void tegra_dvfs_rail_enable(struct dvfs_rail *rail);
void tegra_dvfs_rail_disable(struct dvfs_rail *rail);
int tegra_dvfs_predict_millivolts(struct clk *c, unsigned long rate);
bool tegra_dvfs_is_dfll_range(struct clk *c, unsigned long rate);
int tegra_dvfs_set_dfll_range(struct clk *c, int range);
int tegra_get_cpu_fv_table(int *num_freqs, unsigned long **freqs, int **mvs);
void tegra_dvfs_core_lock(void);
void tegra_dvfs_core_unlock(void);
int tegra_dvfs_set_fmax_at_vmin(struct clk *c, unsigned long f_max, int v_min);
int tegra_dvfs_get_core_nominal_millivolts(void);
#else
static inline int tegra_dvfs_init(void)
{ return 0; }
static inline int tegra_dvfs_dfll_mode_set(struct clk *c, unsigned long rate)
{ return -EINVAL; }
static inline int tegra_dvfs_dfll_mode_clear(struct clk *c, unsigned long rate)
{ return -EINVAL; }
static inline int tegra_dvfs_get_dfll_threshold(
		struct clk *c, unsigned long *rate)
{ return -EINVAL; }
static inline int tegra_dvfs_set_rate(struct clk *c, unsigned long rate)
{ return -EINVAL; }
static inline unsigned long tegra_dvfs_get_rate(struct clk *c)
{ return 0; }
static inline int tegra_dvfs_get_freqs(
		struct clk *c, unsigned long **freqs, int *num_freqs)
{ return -EINVAL; }
static inline int tegra_setup_dvfs(struct clk *c, struct dvfs *d)
{ return -EINVAL; }
static inline int tegra_dvfs_init_rails(struct dvfs_rail *dvfs_rails[], int n)
{ return -EINVAL; }
static inline void tegra_dvfs_add_relationships(
		struct dvfs_relationship *rels, int n)
{ return; }
static inline void tegra_dvfs_rail_enable(struct dvfs_rail *rail)
{ return; }
static inline void tegra_dvfs_rail_disable(struct dvfs_rail *rail)
{ return; }
static inline int tegra_dvfs_predict_millivolts(
		struct clk *c, unsigned long rate)
{ return -EINVAL; }
static inline bool tegra_dvfs_is_dfll_range(struct clk *c, unsigned long rate)
{ return false; }
static inline int tegra_dvfs_set_dfll_range(struct clk *c, int range)
{ return -EINVAL; }
static inline int tegra_get_cpu_fv_table(
		int *num_freqs, unsigned long **freqs, int **mvs)
{ return -EINVAL; }
static inline void tegra_dvfs_core_lock(void)
{ return; }
static inline void tegra_dvfs_core_unlock(void)
{ return; }
static inline int tegra_dvfs_set_fmax_at_vmin(struct clk *c,
		unsigned long f_max, int v_min)
{ return -EINVAL; }
static inline int tegra_dvfs_get_core_nominal_millivolts(void)
{ return -ENOENT; }
#endif

#ifdef CONFIG_TEGRA_124_DVFS
int tegra124_init_dvfs(void);
#else
static inline int tegra124_init_dvfs(void)
{ return -EINVAL; }
#endif

#endif
