/*
 * Copyright (c) 2014, NVIDIA CORPORATION. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/suspend.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/tegra-dvfs.h>
#include <soc/tegra/tegra-edp.h>
#include <dt-bindings/thermal/tegra-cpu-edp-trips.h>

#include "tegra_edp_private.h"

#define KHZ	1000
#define FREQ_STEP_KHZ		12750

/*
 * "Safe entry" to be used when no match for speedo_id or
 * regulator_cur is found; must be the last one
 */
static struct tegra_edp_limits edp_default_limits[] = {
	{TEGRA_CPU_EDP_THERMAL_CAP_0, {1000000, 1000000, 1000000, 1000000} },
	{TEGRA_CPU_EDP_THERMAL_CAP_1, {1000000, 1000000, 1000000, 1000000} },
	{TEGRA_CPU_EDP_THERMAL_CAP_2, {1000000, 1000000, 1000000, 1000000} },
	{TEGRA_CPU_EDP_THERMAL_CAP_3, {1000000, 1000000, 1000000, 1000000} },
	{TEGRA_CPU_EDP_THERMAL_CAP_4, {1000000, 1000000, 1000000, 1000000} },
	{TEGRA_CPU_EDP_THERMAL_CAP_5, {1000000, 1000000, 1000000, 1000000} },
	{TEGRA_CPU_EDP_THERMAL_CAP_6, {1000000, 1000000, 1000000, 1000000} },
	{TEGRA_CPU_EDP_THERMAL_CAP_7, {1000000, 1000000, 1000000, 1000000} },
	{TEGRA_CPU_EDP_THERMAL_CAP_8, {1000000, 1000000, 1000000, 1000000} },
	{TEGRA_CPU_EDP_THERMAL_CAP_9, {1000000, 1000000, 1000000, 1000000} },
	{TEGRA_CPU_EDP_THERMAL_CAP_10, {1000000, 1000000, 1000000, 1000000} },
	{TEGRA_CPU_EDP_THERMAL_CAP_11, {1000000, 1000000, 1000000, 1000000} },
	{TEGRA_CPU_EDP_THERMAL_CAP_12, {1000000, 1000000, 1000000, 1000000} },
};

/* Constants for EDP calculations */

/*
 * Temperatures which are used to calculate EDP limits in
 * different temperature rang
 */
static const int temperatures[] = { /* degree celcius (C) */
	TEGRA_CPU_EDP_THERMAL_CAP_0,
	TEGRA_CPU_EDP_THERMAL_CAP_1,
	TEGRA_CPU_EDP_THERMAL_CAP_2,
	TEGRA_CPU_EDP_THERMAL_CAP_3,
	TEGRA_CPU_EDP_THERMAL_CAP_4,
	TEGRA_CPU_EDP_THERMAL_CAP_5,
	TEGRA_CPU_EDP_THERMAL_CAP_6,
	TEGRA_CPU_EDP_THERMAL_CAP_7,
	TEGRA_CPU_EDP_THERMAL_CAP_8,
	TEGRA_CPU_EDP_THERMAL_CAP_9,
	TEGRA_CPU_EDP_THERMAL_CAP_10,
	TEGRA_CPU_EDP_THERMAL_CAP_11,
	TEGRA_CPU_EDP_THERMAL_CAP_12,
};

/* Leakage calculation data, used for EDP table calculation */
static const int tegra124_leakage_ijk_common[4][4][4] = {
	/* i = 0 */
	{ {  -309609464,    197786326,    -40763150,    1613941, },
	  {   964716269,   -569081375,    115781607,   -4206296, },
	  {  -994324790,    529664031,   -106360108,    3454033, },
	  {   343209442,   -160577505,     31928605,    -895157, },
	},
	/* i = 1 */
	{ {   616319664,   -637007187,    137759592,    -7194133,  },
	  { -1853817283,   1896032851,   -407407611,    20868220,  },
	  {  1824097131,  -1831611624,    390753403,   -19530122,  },
	  {  -589155245,    578838526,   -122655676,     5985577,  },
	},
	/* i = 2 */
	{ {  -439994037,    455845250,   -104097013,     6191899, },
	  {  1354650774,  -1395561938,    318665647,   -18886906, },
	  { -1361677255,   1390149678,   -317474532,    18728266, },
	  {   447877887,   -451382027,    103201434,    -6046692, },
	},
	/* i = 3 */
	{ {    56797556,    -59779544,     13810295,     -848290, },
	  {  -175867301,    184753957,    -42708242,     2621537, },
	  {   177626357,   -185996541,     43029384,    -2638283, },
	  {   -58587547,     61075322,    -14145853,      865351, },
	},
};

#define TEGRA124_EDP_PARAMS_COMMON_PART					\
	.temp_scaled      = 10,						\
	.dyn_scaled       = 1000,					\
	.dyn_consts_n     = { 950,  1399, 2166, 3041 },	\
	.consts_scaled    = 100,					\
	.leakage_consts_n = { 45, 67, 87, 100 },			\
	.ijk_scaled       = 100000,					\
	.leakage_min      = 30,						\
	.volt_temp_cap    = { 70, 1240 },				\
	.leakage_consts_ijk = tegra124_leakage_ijk_common

static const struct tegra_edp_cpu_leakage_params tegra124_leakage_params[] = {
	{
		/*
		 * this leakage params can be used for all
		 * t124 cpu speedo id
		 */
		.cpu_speedo_id      = -1,
		TEGRA124_EDP_PARAMS_COMMON_PART,
	},
};

static const int tegra132_leakage_ijk_common[4][4][4] = {
	/* i = 0 */
	{ {     37941815,   -34693402,    7820012,   -441775, },
	  {   -120688312,   110469778,  -24921798,   1406135, },
	  {    125400868,  -114967356,   25961411,  -1462457, },
	  {    -42517654,    39125172,   -8844968,    497219, },
	},
	/* i = 1 */
	{ {    -50381367,    44032432,   -9833725,    555725, },
	  {    160275950,  -139793882,   31257629,  -1764975, },
	  {   -166543984,   144936902,  -32453803,   1830681, },
	  {     56516540,   -49076991,   11008971,   -619802, },
	},
	/* i = 2 */
	{ {     17384788,   -15152169,    3354364,   -189265, },
	  {    -55185216,    48061420,  -10652770,    601257, },
	  {     57179883,   -49766548,   11046206,   -623597, },
	  {    -19334275,    16829382,   -3741212,    211201, },
	},
	/* i = 3 */
	{ {     -1678751,     1460723,    -322310,     18193, },
	  {      5323806,    -4629524,    1022876,    -57779, },
	  {     -5509910,     4788629,   -1059681,     59896, },
	  {      1861103,    -1617204,     358518,    -20274, },
	},

};

#define TEGRA132_EDP_PARAMS_COMMON_PART					\
	.temp_scaled      = 10,						\
	.dyn_scaled       = 1000,					\
	.dyn_consts_n     = { 3900, 5900 }, /* { save: 2700, 5900 } */	\
	.consts_scaled    = 100,					\
	.leakage_consts_n = { 100, 100 }, /* { save: 60, 100 } */	\
	.ijk_scaled       = 1000,					\
	.leakage_min      = 30,						\
	.leakage_consts_ijk = tegra132_leakage_ijk_common

static const struct tegra_edp_cpu_leakage_params tegra132_leakage_params[] = {
	{
		/*
		 * this leakage params can be used for all
		 * t132 cpu speedo id
		 */
		.cpu_speedo_id      = -1,
		TEGRA132_EDP_PARAMS_COMMON_PART,
		.safety_cap = { 2500000, 2300000, },
	},
};

/* Locked when update edp limit */
static DEFINE_MUTEX(tegra_cpu_lock);
static struct tegra_edp cpu_edp;

/* val to the power of pwr */
static inline s64 edp_pow(s64 val, int pwr)
{
	s64 retval = 1;

	while (pwr) {
		if (pwr & 1)
			retval *= val;
		pwr >>= 1;
		if (pwr)
			val *= val;
	}

	return retval;
}

static s64 cal_leakage_ma(const struct tegra_edp_cpu_leakage_params *params,
			  int n_cores_idx, int iddq_ma,
			  unsigned int voltage_mv, int temp_c)
{
	int i, j, k;
	s64 leakage_ma, leakage_calc_step;

	/*
	 * Calculate leakage current
	 * Use the values in leakage_ijk_common table and iddq_ma, voltage_mv,
	 * temp_c to calculate the leakage current.
	 * leakage_ma += ijk * iddq_ma^i * voltage_mv^j* temp_c^k
	 */
	leakage_ma = 0;
	for (i = 0; i <= 3; i++) {
		for (j = 0; j <= 3; j++) {
			for (k = 0; k <= 3; k++) {
				unsigned int scaled;
				int ijk;

				ijk = params->leakage_consts_ijk[i][j][k];
				leakage_calc_step = ijk * edp_pow(iddq_ma, i);
				/* Convert (mA)^i to (A)^i */
				leakage_calc_step = div64_s64(leakage_calc_step,
							      edp_pow(1000, i));
				leakage_calc_step *= edp_pow(voltage_mv, j);
				/* Convert (mV)^j to (V)^j */
				leakage_calc_step = div64_s64(leakage_calc_step,
							      edp_pow(1000, j));
				leakage_calc_step *= edp_pow(temp_c, k);
				/* Convert (C)^k to (scaled_C)^k */
				scaled = params->temp_scaled;
				leakage_calc_step = div64_s64(leakage_calc_step,
							edp_pow(scaled, k));
				/* leakage_consts_ijk was scaled */
				leakage_calc_step = div64_s64(leakage_calc_step,
							params->ijk_scaled);
				leakage_ma += leakage_calc_step;
			}
		}
	}

	leakage_ma *= params->leakage_consts_n[n_cores_idx];

	/* leakage_const_n was scaled */
	leakage_ma = div64_s64(leakage_ma, params->consts_scaled);

	/* if specified, set floor for leakage current */
	if (params->leakage_min && leakage_ma <= params->leakage_min)
		leakage_ma = params->leakage_min;

	return leakage_ma;
}

static s64 cal_dynamic_ma(const struct tegra_edp_cpu_leakage_params *params,
			  int n_cores_idx, unsigned int voltage_mv,
			  unsigned int freq_khz)
{
	s64 dyn_ma;

	/* Calculate dynamic current */
	dyn_ma = voltage_mv * freq_khz / 1000;
	/* Convert mV to V */
	dyn_ma = div64_s64(dyn_ma, 1000);
	dyn_ma *= params->dyn_consts_n[n_cores_idx];
	/* dyn_const_n was scaled */
	dyn_ma = div64_s64(dyn_ma, params->dyn_scaled);

	return dyn_ma;
}

/**
 * edp_calculate_maxf
 * @dev: struct device of cpu edp.
 * @n_cores_idx: index of cpu cores.
 * @temp_idx: index of array of temperature.
 * @power_mw: valid or -1 (infinite) or -EINVAL
 * @iddq_ma: cpu iddq value.
 * @freq_voltage_lut: pointer of the freq voltage look up table.
 * @freq_voltage_lut_size: size of the freq voltage look up table.
 *
 * Find the maximum frequency that results in dynamic and leakage current that
 * is less than the regulator current limit.
 *
 * Return: cpu frequecy in khz (success), or -EINVAL (fail)
 */
static int edp_calculate_maxf(struct device *dev, int n_cores_idx, int temp_idx,
			      int power_mw, int iddq_ma,
			      struct tegra_edp_freq_vol_table *freq_voltage_lut,
			      unsigned int freq_voltage_lut_size)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);
	const struct tegra_edp_cpu_leakage_params *params;
	int temp_c = edp->temps[temp_idx] / 1000;
	unsigned int cur_effective;
	int f;

	params = &edp->params[edp->cpu_speedo_idx];
	cur_effective = edp->reg_edp_ma;

	for (f = freq_voltage_lut_size - 1; f >= 0; f--) {
		s64 leakage_ma, dyn_ma;
		unsigned int voltage_mv = freq_voltage_lut[f].voltage_mv;
		unsigned int freq_khz = freq_voltage_lut[f].freq / KHZ;

		/* Constrain Volt-Temp */
		if (params->volt_temp_cap.temperature &&
		    temp_c > params->volt_temp_cap.temperature &&
		    params->volt_temp_cap.voltage_limit_mv &&
		    voltage_mv > params->volt_temp_cap.voltage_limit_mv)
			continue;

		/* Calculate leakage current */
		leakage_ma = cal_leakage_ma(params, n_cores_idx,
					    iddq_ma, voltage_mv, temp_c);

		/* leakage cannot be negative => leakage model has error */
		if (leakage_ma <= 0) {
			dev_err(dev,
				"VDD_CPU EDP failed: IDDQ too high (%d mA)\n",
				iddq_ma);
			return -EINVAL;
		}

		/* Calculate dynamic current */
		dyn_ma = cal_dynamic_ma(params, n_cores_idx,
					voltage_mv, freq_khz);

		if (power_mw != -1) {
			s64 leakage_mw = leakage_ma * voltage_mv;
			s64 dyn_mw = dyn_ma * voltage_mv;
			if (div64_s64(leakage_mw + dyn_mw, 1000) <= power_mw)
				return freq_khz;
		} else if ((leakage_ma + dyn_ma) <= cur_effective) {
			return freq_khz;
		}
	}

	return -EINVAL;
}

/**
 * edp_relate_freq_voltage
 * @dev: struct device of cpu edp.
 * @clk_cpu_g: clock of cpu.
 * @freq_voltage_lut: pointer of the freq voltage look up table.
 * @freq_voltage_lut_size: size of the freq voltage look up table.
 *
 * Get the predict voltage for all cpu clock frequencies with
 * FREQ_STEP_KHZ step, and  fill  them to the cpu frequency and
 * voltage look up table.
 *
 * Return: 0 (success), or -EINVAL (fail)
 */
static int edp_relate_freq_voltage(struct device *dev, struct clk *clk_cpu_g,
				   struct tegra_edp_freq_vol_table *lut,
				   unsigned int lut_size)
{
	unsigned int i, freq;
	int voltage_mv;

	for (i = 0, freq = 0; i < lut_size; i++, freq += FREQ_STEP_KHZ) {
		/* Predict voltages */
		voltage_mv = tegra_dvfs_predict_millivolts(clk_cpu_g,
							   freq * KHZ);
		if (voltage_mv < 0) {
			dev_err(dev,
				"Couldn't predict voltage: freq %u; err %d\n",
				freq, voltage_mv);
			return -EINVAL;
		}

		/* Cache frequency / voltage / voltage constant relationship */
		lut[i].freq = freq * KHZ;
		lut[i].voltage_mv = voltage_mv;
	}

	return 0;
}

static int edp_find_speedo_idx(struct device *dev, int cpu_speedo_id)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < edp->params_size; i++) {
		if (edp->params[i].cpu_speedo_id == -1)
			return i;

		if (cpu_speedo_id == edp->params[i].cpu_speedo_id)
			return i;
	}

	dev_err(dev, "Couldn't find cpu speedo id %d in freq/voltage LUT\n",
		cpu_speedo_id);

	return -EINVAL;
}

static int tegra_cpu_edp_cal_limits(struct device *dev)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);
	struct clk *clk_cpu_g = clk_get_sys(NULL, "cclk_g");
	unsigned int temp_idx, n_cores_idx;
	unsigned int cpu_g_minf, cpu_g_maxf;
	struct tegra_edp_limits *edp_calculated_limits;
	const struct tegra_edp_cpu_leakage_params *params;
	struct tegra_edp_freq_vol_table *freq_voltage_lut;
	unsigned int freq_voltage_lut_size;
	int iddq_ma;
	size_t size;
	int ret;

	/* Determine all inputs to EDP formula */
	iddq_ma = tegra_sku_info.cpu_iddq_value;
	params = &edp->params[edp->cpu_speedo_idx];

	/* Allocate freq_voltage look up table */
	cpu_g_minf = 0;
	cpu_g_maxf = edp->max_cpu_freq;
	freq_voltage_lut_size = (cpu_g_maxf - cpu_g_minf) / FREQ_STEP_KHZ + 1;
	size = sizeof(struct tegra_edp_freq_vol_table) * freq_voltage_lut_size;
	freq_voltage_lut = kzalloc(size, GFP_KERNEL);
	if (!freq_voltage_lut) {
		dev_err(dev, "Failed to alloc mem for freq/voltage LUT\n");
		return -ENOMEM;
	}

	/* Fill the freq_voltage look up table */
	ret = edp_relate_freq_voltage(dev, clk_cpu_g,
				      freq_voltage_lut,
				      freq_voltage_lut_size);
	if (ret)
		goto err_free_lut;

	/* Allocate EDP limits table */
	size = sizeof(struct tegra_edp_limits) * edp->temps_size;
	edp_calculated_limits = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!edp_calculated_limits) {
		dev_err(dev, "Failed to alloc mem for edp calculatd limits\n");
		ret = -ENOMEM;
		goto err_free_lut;
	}

	/* Calculate EDP table */
	for (n_cores_idx = 0;
	     n_cores_idx < num_possible_cpus(); n_cores_idx++) {
		if (n_cores_idx >= MAX_CPU_CORES) {
			dev_warn(dev,
				"The number of cpu cores is greater than supported (%d)\n",
				MAX_CPU_CORES);
			break;
		}

		for (temp_idx = 0;
		     temp_idx < edp->temps_size; temp_idx++) {
			unsigned int cap, limit;

			edp_calculated_limits[temp_idx].temperature =
						edp->temps[temp_idx];
			limit = edp_calculate_maxf(dev,
						   n_cores_idx,
						   temp_idx,
						   -1,
						   iddq_ma,
						   freq_voltage_lut,
						   freq_voltage_lut_size);
			if (limit == -EINVAL) {
				ret = -EINVAL;
				goto err_free_limits;
			}

			/* apply safety cap if it is specified */
			cap = params->safety_cap[n_cores_idx];
			if (cap && cap < limit)
				limit = cap;
			edp_calculated_limits[temp_idx].
				freq_limits[n_cores_idx] = limit;
		}
	}

	/*
	 * If this is an EDP table update, need to overwrite old table.
	 * The old table's address must remain valid.
	 */
	mutex_lock(&tegra_cpu_lock);
	if (edp->edp_limits != edp->def_edp_limits) {
		memcpy(edp->edp_limits, edp_calculated_limits, size);
		devm_kfree(dev, edp_calculated_limits);
	} else {
		edp->edp_limits = edp_calculated_limits;
		edp->edp_limits_size = edp->temps_size;
	}
	mutex_unlock(&tegra_cpu_lock);

	kfree(freq_voltage_lut);

	return 0;

err_free_limits:
	devm_kfree(dev, edp_calculated_limits);
err_free_lut:
	kfree(freq_voltage_lut);
	return ret;
}

/* Get the freq limit according to the edp_thermal_index */
static unsigned int edp_predict_limit(struct tegra_edp *edp, unsigned int cpus)
{
	unsigned int limit = 0;

	BUG_ON(cpus == 0);
	if (edp->edp_limits) {
		int i = edp->edp_thermal_index;
		int size = edp->edp_limits_size;

		BUG_ON(i >= size);
		limit = edp->edp_limits[i].freq_limits[cpus - 1];
	}

	return limit;
}

/* Must be called while holding tegra_cpu_lock */
static void tegra_edp_update_limit(struct device *dev)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);
	unsigned int cpus, limit;

	if (!edp->edp_limits)
		return;

	BUG_ON(!mutex_is_locked(&tegra_cpu_lock));

	cpus = cpumask_weight(&edp->edp_cpumask);
	limit = edp_predict_limit(edp, cpus);
	edp->edp_freq_limit = limit;
}

/* Governor requested frequency, not higher than edp limits */
static unsigned int tegra_edp_governor_speed(struct device *dev,
				      unsigned int requested_speed)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);

	if ((!edp->edp_freq_limit) ||
	    (requested_speed <= edp->edp_freq_limit))
		return requested_speed;
	else
		return edp->edp_freq_limit;
}

static int tegra_cpu_edp_init_data(struct device *dev,
				   unsigned int reg_edp_ma)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);
	unsigned int cpu_speedo_idx;

	edp->reg_edp_ma = reg_edp_ma;
	edp->def_edp_limits = edp_default_limits;
	edp->def_edp_limits_size = ARRAY_SIZE(edp_default_limits);
	edp->temps = temperatures;
	edp->temps_size = ARRAY_SIZE(temperatures);
	edp->edp_limits = edp_default_limits;
	edp->edp_limits_size = ARRAY_SIZE(edp_default_limits);

	cpu_speedo_idx = edp_find_speedo_idx(dev, tegra_sku_info.cpu_speedo_id);
	if (cpu_speedo_idx < 0)
		return -EINVAL;
	edp->cpu_speedo_idx = cpu_speedo_idx;

	tegra_cpu_edp_cal_limits(dev);

	mutex_lock(&tegra_cpu_lock);
	edp->edp_cpumask = *cpu_online_mask;
	tegra_edp_update_limit(dev);
	mutex_unlock(&tegra_cpu_lock);

	dev_info(dev, "Init EDP limit: %u MHz\n", edp->edp_freq_limit/1000);

	return 0;
}

/*
 * The CPU EDP driver worked with thermal framework, provided callbacks
 * for the cpu edp recation driver.
 */
int tegra_cpu_edp_get_thermal_index(struct platform_device *pdev)
{
	struct tegra_edp *edp = dev_get_drvdata(&pdev->dev);
	return edp->edp_thermal_index;
}
EXPORT_SYMBOL(tegra_cpu_edp_get_thermal_index);

int tegra_cpu_edp_count_therm_floors(struct platform_device *pdev)
{
	struct tegra_edp *edp = dev_get_drvdata(&pdev->dev);
	return edp->edp_limits_size - 1;
}
EXPORT_SYMBOL(tegra_cpu_edp_count_therm_floors);

int tegra_cpu_edp_update_thermal_index(struct platform_device *pdev,
				       unsigned long new_idx)
{
	struct tegra_edp *edp = dev_get_drvdata(&pdev->dev);

	if (new_idx > (edp->edp_limits_size - 1))
		return -ERANGE;

	if (!cpufreq_get(0))
		return 0;

	mutex_lock(&tegra_cpu_lock);

	edp->edp_thermal_index = new_idx;
	edp->edp_cpumask = *cpu_online_mask;
	tegra_edp_update_limit(&pdev->dev);

	mutex_unlock(&tegra_cpu_lock);

	cpufreq_update_policy(0);

	return 0;
}
EXPORT_SYMBOL(tegra_cpu_edp_update_thermal_index);

bool tegra_cpu_edp_ready(void)
{
	return cpu_edp.edp_init_done;
}
EXPORT_SYMBOL(tegra_cpu_edp_ready);

/* Get the supported maximum cpu frequency */
static unsigned int tegra_edp_get_max_cpu_freq(void)
{
	struct cpufreq_frequency_table *table = cpufreq_frequency_get_table(0);
	unsigned int freq = 0;
	int i;

	for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (table[i].frequency == CPUFREQ_ENTRY_INVALID)
			continue;

		if (table[i].frequency > freq)
			freq = table[i].frequency;
	}

	return freq;
}

#ifdef CONFIG_DEBUG_FS

static int edp_limit_debugfs_show(struct seq_file *s, void *data)
{
	struct device *dev = s->private;
	struct tegra_edp *edp = dev_get_drvdata(dev);

	seq_printf(s, "%u\n", edp->edp_freq_limit);
	return 0;
}

static int edp_debugfs_show(struct seq_file *s, void *data)
{
	struct device *dev = s->private;
	struct tegra_edp *edp = dev_get_drvdata(dev);
	int i, th_idx;

	th_idx = edp->edp_thermal_index;
	seq_printf(s, "-- VDD_CPU %sEDP table (%umA) --\n",
		   edp->edp_limits == edp->def_edp_limits ? "**default** " : "",
		   edp->reg_edp_ma);
	seq_printf(s, "%6s %10s %10s %10s %10s\n",
		   " Temp.", "1-core", "2-cores", "3-cores", "4-cores");
	for (i = 0; i < edp->edp_limits_size; i++) {
		seq_printf(s, "%c%3dC: %10u %10u %10u %10u\n",
			   i == th_idx ? '>' : ' ',
			   edp->edp_limits[i].temperature / 1000,
			   edp->edp_limits[i].freq_limits[0],
			   edp->edp_limits[i].freq_limits[1],
			   edp->edp_limits[i].freq_limits[2],
			   edp->edp_limits[i].freq_limits[3]);
	}

	return 0;
}

static int edp_reg_edp_ma_show(struct seq_file *s, void *data)
{
	struct device *dev = s->private;
	struct tegra_edp *edp = dev_get_drvdata(dev);
	seq_printf(s, "Effective limit: %u mA\n", edp->reg_edp_ma);
	return 0;
}

static ssize_t edp_reg_edp_ma_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct device *dev = file->f_path.dentry->d_inode->i_private;
	struct tegra_edp *edp = dev_get_drvdata(dev);
	char buf[32];
	unsigned long res;
	unsigned int reg_edp_ma_temp;
	unsigned int reg_edp_ma_prev = edp->reg_edp_ma;

	if (sizeof(buf) <= count)
		goto edp_ma_err;

	if (copy_from_user(buf, userbuf, count))
		goto edp_ma_err;

	/* terminate buffer and trim - white spaces may be appended
	 *  at the end when invoked from shell command line */
	buf[count] = '\0';
	strim(buf);

	if (!kstrtoul(buf, 10, &res))
		reg_edp_ma_temp = (unsigned int)res;
	else
		goto edp_ma_err;

	if (reg_edp_ma_temp == 0)
		goto edp_ma_err;

	if (edp->reg_edp_ma == reg_edp_ma_temp)
		return count;

	edp->reg_edp_ma = reg_edp_ma_temp;
	if (tegra_cpu_edp_cal_limits(dev)) {
		/* Revert to previous mA value if new value fails */
		edp->reg_edp_ma = reg_edp_ma_prev;
		goto edp_ma_err;
	}

	if (cpufreq_update_policy(0)) {
		dev_err(dev,
			"FAILED: Set CPU freq cap with new VDD_CPU EDP table\n");
		goto edp_ma_out;
	}

	dev_info(dev,
		 "Reinitialized VDD_CPU EDP table with regulator current limit %u mA\n",
		 edp->reg_edp_ma);

	return count;

edp_ma_err:
	dev_err(dev,
		"FAILED: Reinitialize VDD_CPU EDP table with limit current \"%s\"",
		buf);
edp_ma_out:
	return -EINVAL;
}

static int edp_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, edp_debugfs_show, inode->i_private);
}

static int edp_limit_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, edp_limit_debugfs_show, inode->i_private);
}

static int edp_reg_edp_ma_open(struct inode *inode, struct file *file)
{
	return single_open(file, edp_reg_edp_ma_show, inode->i_private);
}

static const struct file_operations edp_debugfs_fops = {
	.open		= edp_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations edp_limit_debugfs_fops = {
	.open		= edp_limit_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations edp_reg_edp_ma_debugfs_fops = {
	.open		= edp_reg_edp_ma_open,
	.read		= seq_read,
	.write		= edp_reg_edp_ma_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int tegra_cpu_edp_debugfs_init(struct device *dev)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);
	struct dentry *d_edp;
	struct dentry *d_edp_limit;
	struct dentry *d_edp_reg_edp_ma;
	struct dentry *vdd_cpu_dir;

	vdd_cpu_dir = debugfs_create_dir("cpu_edp", NULL);

	if (!vdd_cpu_dir)
		goto vdd_cpu_dir_err;

	d_edp = debugfs_create_file("edp", S_IRUGO, vdd_cpu_dir, dev,
				&edp_debugfs_fops);

	if (!d_edp)
		goto vdd_cpu_dir_err;

	d_edp_limit = debugfs_create_file("edp_limit", S_IRUGO, vdd_cpu_dir,
				dev, &edp_limit_debugfs_fops);

	if (!d_edp_limit)
		goto vdd_cpu_dir_err;

	d_edp_reg_edp_ma = debugfs_create_file("reg_edp_ma",
				S_IRUGO | S_IWUSR, vdd_cpu_dir, dev,
				&edp_reg_edp_ma_debugfs_fops);

	if (!d_edp_reg_edp_ma)
		goto vdd_cpu_dir_err;

	edp->dir = vdd_cpu_dir;

	return 0;

vdd_cpu_dir_err:
	debugfs_remove_recursive(vdd_cpu_dir);
	return -ENOMEM;
}

#endif /* CONFIG_DEBUG_FS */

/**
 * tegra_cpu_edp_notify - Notifier callback for cpu hotplug.
 * @nb:	struct notifier_block * with callback info.
 * @event: value showing cpufreq event for which this function invoked.
 * @data: callback-specific data
 *
 * Callback to highjack the notification on cpu hotplug.
 * Every time there is a event of cpu hotplug, we will intercept and
 * update the cpu edp limit and update cpu policy if needed.
 *
 * Return: 0 (success)
 */
static int tegra_cpu_edp_notify(
	struct notifier_block *nb, unsigned long event, void *hcpu)
{
	struct platform_device *pdev = cpu_edp.pdev;
	struct device *dev = &pdev->dev;
	int ret = 0;
	unsigned int cpu_speed, new_speed;
	int cpu = (long)hcpu;

	switch (event) {
	case CPU_UP_PREPARE:
		mutex_lock(&tegra_cpu_lock);
		cpu_set(cpu, cpu_edp.edp_cpumask);
		tegra_edp_update_limit(dev);

		cpu_speed = cpufreq_get(0);
		new_speed = tegra_edp_governor_speed(dev, cpu_speed);
		if (new_speed < cpu_speed) {
			ret = cpufreq_update_policy(0);
			dev_dbg(dev, "cpu-tegra:%sforce EDP limit %u kHz\n",
				ret ? " failed to " : " ", new_speed);
		}
		if (ret) {
			cpu_clear(cpu, cpu_edp.edp_cpumask);
			tegra_edp_update_limit(dev);
		}
		mutex_unlock(&tegra_cpu_lock);
		break;
	case CPU_DEAD:
		mutex_lock(&tegra_cpu_lock);
		cpu_clear(cpu, cpu_edp.edp_cpumask);
		tegra_edp_update_limit(dev);
		mutex_unlock(&tegra_cpu_lock);
		cpufreq_update_policy(0);
		break;
	}

	return notifier_from_errno(ret);
}

static struct notifier_block tegra_cpu_edp_notifier = {
	.notifier_call = tegra_cpu_edp_notify,
};

/**
 * edp_cpufreq_policy_notifier - Notifier callback for cpufreq policy change.
 * @nb:	struct notifier_block * with callback info.
 * @event: value showing cpufreq event for which this function invoked.
 * @data: callback-specific data
 *
 * Callback to highjack the notification on cpufreq policy transition.
 * Every time there is a change in policy, we will intercept and
 * update the cpufreq policy with edp constraints.
 *
 * Return: 0 (success)
 */
static int edp_cpufreq_policy_notifier(struct notifier_block *nb,
				       unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;

	if (event != CPUFREQ_ADJUST)
		return 0;

	/* Limit max freq to be within edp limit. */

	if (policy->max != cpu_edp.edp_freq_limit)
		cpufreq_verify_within_limits(policy, 0, cpu_edp.edp_freq_limit);

	return 0;
}

/* Notifier for cpufreq policy change */
static struct notifier_block edp_cpufreq_notifier_block = {
	.notifier_call = edp_cpufreq_policy_notifier,
};

struct edp_params {
	const struct tegra_edp_cpu_leakage_params *params;
	int params_size;
};

static const struct edp_params tegra124_edp_params = {
	.params = tegra124_leakage_params,
	.params_size = ARRAY_SIZE(tegra124_leakage_params),
};

static const struct edp_params tegra132_edp_params = {
	.params = tegra132_leakage_params,
	.params_size = ARRAY_SIZE(tegra132_leakage_params),
};

static const struct of_device_id tegra_cpu_edp_match[] = {
	{ .compatible = "nvidia,tegra124-cpu-edp",
	  .data = &tegra124_edp_params },
	{ .compatible = "nvidia,tegra132-cpu-edp",
	  .data = &tegra132_edp_params },
	{},
};

static int tegra_cpu_edp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	const struct edp_params *params;
	unsigned int reg_edp_ma;
	u32 val;
	int err;
	struct device_node *np = pdev->dev.of_node;

	if (!cpufreq_get(0)) {
		dev_warn(dev, "CPU clocks are not ready.\n");
		return -EPROBE_DEFER;
	}

	err = of_property_read_u32(np, "nvidia,edp-limit", &val);
	if (!err)
		reg_edp_ma = val;
	else {
		dev_err(dev, "Failed to read edp-limit");
		return -EINVAL;
	}

	dev_info(dev, "CPU regulator EDP limit %d mA\n", reg_edp_ma);

	match = of_match_node(tegra_cpu_edp_match, np);
	params = match->data;
	cpu_edp.params = params->params;
	cpu_edp.params_size = params->params_size;

	cpu_edp.max_cpu_freq = tegra_edp_get_max_cpu_freq();
	cpu_edp.pdev = pdev;

	platform_set_drvdata(pdev, &cpu_edp);

	err = tegra_cpu_edp_init_data(dev, reg_edp_ma);
	if (err) {
		dev_err(dev, "Failed to initialize cpu edp data\n");
		return err;
	}

	cpufreq_register_notifier(&edp_cpufreq_notifier_block,
				  CPUFREQ_POLICY_NOTIFIER);

	register_hotcpu_notifier(&tegra_cpu_edp_notifier);

#ifdef CONFIG_DEBUG_FS
	tegra_cpu_edp_debugfs_init(dev);
#endif

	cpu_edp.edp_init_done = true;

	return 0;
}

static int tegra_cpu_edp_remove(struct platform_device *pdev)
{
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(cpu_edp.dir);
#endif
	unregister_hotcpu_notifier(&tegra_cpu_edp_notifier);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_cpu_edp_suspend(struct device *dev)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);
	int size;

	mutex_lock(&tegra_cpu_lock);

	if (edp->edp_limits) {
		size = edp->edp_limits_size - 1;
		edp->edp_freq_limit = edp->edp_limits[size].freq_limits[3];
	} else {
		size = edp->def_edp_limits_size - 1;
		edp->edp_freq_limit = edp->def_edp_limits[size].freq_limits[3];
	}

	mutex_unlock(&tegra_cpu_lock);

	cpufreq_update_policy(0);

	return 0;
}

static int tegra_cpu_edp_resume(struct device *dev)
{
	struct tegra_edp *edp = dev_get_drvdata(dev);

	mutex_lock(&tegra_cpu_lock);
	edp->edp_cpumask = *cpu_online_mask;
	tegra_edp_update_limit(dev);
	mutex_unlock(&tegra_cpu_lock);

	cpufreq_update_policy(0);

	return 0;
}

static const struct dev_pm_ops tegra_cpu_edp_pm_ops = {
	.suspend = tegra_cpu_edp_suspend,
	.resume = tegra_cpu_edp_resume,
};
#endif

static struct platform_driver tegra_cpu_edp_driver = {
	.probe  = tegra_cpu_edp_probe,
	.remove = tegra_cpu_edp_remove,
	.driver = {
		.name   = "tegra-cpu-edp",
		.owner  = THIS_MODULE,
		.of_match_table = tegra_cpu_edp_match,
#ifdef CONFIG_PM_SLEEP
		.pm	= &tegra_cpu_edp_pm_ops,
#endif
	},
};

static int __init tegra_cpu_edp_init(void)
{
	return platform_driver_register(&tegra_cpu_edp_driver);
}

static void __exit tegra_cpu_edp_exit(void)
{
	platform_driver_unregister(&tegra_cpu_edp_driver);
}

module_init(tegra_cpu_edp_init);
module_exit(tegra_cpu_edp_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Tegra VDD_CPU EDP management");
