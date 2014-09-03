/*
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
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#include <soc/tegra/tegra-dvfs.h>
#include <soc/tegra/fuse.h>

#define KHZ		1000
#define MHZ		1000000
#define VDD_SAFE_STEP	100

static bool tegra_dvfs_cpu_disabled;
static bool tegra_dvfs_core_disabled;
static int cpu_millivolts[MAX_DVFS_FREQS];
static int cpu_dfll_millivolts[MAX_DVFS_FREQS];

static const char *tegra_soc_name;
static struct cpu_dvfs *cpu_fv_dvfs_table;
static struct dvfs *core_dvfs_table;
static struct dvfs_rail *tegra_dvfs_cpu_rail;
static struct dvfs_rail *tegra_dvfs_core_rail;
static unsigned long *cpu_max_freq_tegra;
static struct dvfs_rail **tegra_dvfs_rails;

static struct dvfs_rail tegra124_dvfs_rail_vdd_cpu = {
	.reg_id = "vdd_cpu",
	.max_millivolts = 1300,
	.min_millivolts = 700,
	.step = VDD_SAFE_STEP,
	.jmp_to_zero = true,
	.alignment = {
		.step_uv = 10000, /* 10mV */
	},
};

static struct dvfs_rail tegra132_dvfs_rail_vdd_cpu = {
	.reg_id = "vdd_cpu",
	.max_millivolts = 1300,
	.min_millivolts = 800,
	.step = VDD_SAFE_STEP,
	.jmp_to_zero = true,
	.alignment = {
		.step_uv = 10000, /* 10mV */
	},
};

static struct dvfs_rail tegra124_dvfs_rail_vdd_core = {
	.reg_id = "vdd_core",
	.max_millivolts = 1400,
	.min_millivolts = 800,
	.step = VDD_SAFE_STEP,
	.step_up = 1400,
};

static struct dvfs_rail tegra132_dvfs_rail_vdd_core = {
	.reg_id = "vdd_core",
	.max_millivolts = 1400,
	.min_millivolts = 800,
	.step = VDD_SAFE_STEP,
	.step_up = 1400,
};

static struct dvfs_rail *tegra124_dvfs_rails[] = {
	&tegra124_dvfs_rail_vdd_cpu,
	&tegra124_dvfs_rail_vdd_core,
};

static struct dvfs_rail *tegra132_dvfs_rails[] = {
	&tegra132_dvfs_rail_vdd_cpu,
	&tegra132_dvfs_rail_vdd_core,
};

static struct dvfs cpu_dvfs = {
	.clk_name	= "cclk_g",
	.millivolts	= cpu_millivolts,
	.dfll_millivolts = cpu_dfll_millivolts,
	.auto_dvfs	= true,
};

/* Tegra124 CPU DVFS tables */
static unsigned long cpu_max_freq_tegra124[] = {
/* speedo_id	0	 1	  2	   3      */
		2014500, 2320500, 2116500, 2524500,
};

/* Tegra132 CPU DVFS tables */
static unsigned long cpu_max_freq_tegra132[] = {
/* speedo_id	0	 1	  2	   3      */
		2499000, 2499000,
};

static struct cpu_dvfs cpu_fv_dvfs_table_tegra124[] = {
	{
		.speedo_id = -1,
		.process_id = -1,
		.min_mv = 720,
		.max_mv = 1260,
		.fv_table = {
			{204000,  800},
			{306000,  800},
			{408000,  800},
			{510000,  800},
			{612000,  800},
			{714000,  800},
			{816000,  820},
			{918000,  840},
			{1020000, 880},
			{1122000, 900},
			{1224000, 930},
			{1326000, 960},
			{1428000, 990},
			{1530000, 1020},
			{1632000, 1070},
			{1734000, 1100},
			{1836000, 1140},
			{1938000, 1180},
			{2014500, 1220},
			{2116500, 1260},
			{2218500, 1310},
			{2320500, 1360},
			{2422500, 1400},
			{2524500, 1400},
		},
	},
};

static struct cpu_dvfs cpu_fv_dvfs_table_tegra132[] = {
	{
		.speedo_id = -1,
		.process_id = -1,
		.min_mv = 800,
		.max_mv = 1260,
		.fv_table = {
			{204000,  800},
			{306000,  800},
			{408000,  840},
			{510000,  880},
			{612000,  920},
			{714000,  960},
			{816000,  1000},
			{918000,  1050},
			{1020000, 1090},
			{1122000, 1130},
			{1224000, 1170},
			{1326000, 1210},
			{1428000, 1260},
			{1530000, 1300},
			{1632000, 1300},
			{1734000, 1300},
			{1836000, 1300},
			{1938000, 1300},
			{2014500, 1300},
			{2091000, 1300},
			{2193000, 1300},
			{2295000, 1300},
			{2397000, 1300},
			{2499000, 1300},
		},
	},
};

/* Core DVFS tables */
static const int core_millivolts[MAX_DVFS_FREQS] = {
	800, 850, 900, 950, 1000, 1050, 1100, 1150};

#define CORE_DVFS(_clk_name, _speedo_id, _process_id, _auto, _mult, _freqs...) \
	{							\
		.clk_name	= _clk_name,			\
		.speedo_id	= _speedo_id,			\
		.process_id	= _process_id,			\
		.freqs		= {_freqs},			\
		.freqs_mult	= _mult,			\
		.millivolts	= core_millivolts,		\
		.auto_dvfs	= _auto,			\
	}

static struct dvfs core_dvfs_table_tegra124[] = {
	/* Core voltages (mV):		         800,    850,    900,	 950,    1000,	1050,    1100,	 1150 */
	/* Clock limits for internal blocks, PLLs */
	CORE_DVFS("emc",	-1, -1, 1, KHZ, 264000, 348000, 384000, 384000, 528000, 528000, 1200000, 1200000),
	CORE_DVFS("cclk_lp",	0, 0, 1, KHZ,   312000, 528000, 660000, 804000, 912000, 1044000, 1044000, 1044000),
	CORE_DVFS("cclk_lp",	0, 1, 1, KHZ,   312000, 564000, 696000, 828000, 960000, 1044000, 1044000, 1044000),
	CORE_DVFS("cclk_lp",	1, -1, 1, KHZ,  312000, 564000, 696000, 828000, 960000, 1092000, 1092000, 1092000),
	CORE_DVFS("vic03",	0, 0, 1, KHZ,   180000, 324000, 408000, 492000, 588000, 660000, 708000, 756000),
	CORE_DVFS("vic03",	0, 1, 1, KHZ,   180000, 336000, 420000, 504000, 600000, 684000, 756000, 756000),
	CORE_DVFS("vic03",	1, -1, 1, KHZ,  180000, 336000, 420000, 504000, 600000, 684000, 756000, 828000),
	CORE_DVFS("tsec",	0, 0, 1, KHZ,   180000, 324000, 408000, 492000, 588000, 660000, 708000, 756000),
	CORE_DVFS("tsec",	0, 1, 1, KHZ,   180000, 336000, 420000, 504000, 600000, 684000, 756000, 756000),
	CORE_DVFS("tsec",	1, -1, 1, KHZ,  180000, 336000, 420000, 504000, 600000, 684000, 756000, 828000),
	CORE_DVFS("msenc",	0, 0, 1, KHZ,   120000, 216000, 288000, 336000, 384000, 432000, 456000, 480000),
	CORE_DVFS("msenc",	0, 1, 1, KHZ,   120000, 228000, 276000, 348000, 396000, 444000, 480000, 480000),
	CORE_DVFS("msenc",	1, -1, 1, KHZ,  120000, 228000, 276000, 348000, 396000, 444000, 480000, 528000),
	CORE_DVFS("se",		0, 0, 1, KHZ,   120000, 216000, 288000, 336000, 384000, 432000, 456000, 480000),
	CORE_DVFS("se",		0, 1, 1, KHZ,   120000, 228000, 276000, 348000, 396000, 444000, 480000, 480000),
	CORE_DVFS("se",		1, -1, 1, KHZ,  120000, 228000, 276000, 348000, 396000, 444000, 480000, 528000),
	CORE_DVFS("vde",	0, 0, 1, KHZ,   120000, 216000, 288000, 336000, 384000, 432000, 456000, 480000),
	CORE_DVFS("vde",	0, 1, 1, KHZ,   120000, 228000, 276000, 348000, 396000, 444000, 480000, 480000),
	CORE_DVFS("vde",	1, -1, 1, KHZ,  120000, 228000, 276000, 348000, 396000, 444000, 480000, 528000),
	CORE_DVFS("host1x",	0, 0, 1, KHZ,   108000, 156000, 204000, 240000, 348000, 372000, 408000, 408000),
	CORE_DVFS("host1x",	0, 1, 1, KHZ,   108000, 156000, 204000, 252000, 348000, 384000, 408000, 408000),
	CORE_DVFS("host1x",	1, -1, 1, KHZ,  108000, 156000, 204000, 252000, 348000, 384000, 444000, 444000),
	CORE_DVFS("vi",		0, 0, 1, KHZ,   228000, 408000, 480000, 600000, 600000, 600000, 600000, 600000),
	CORE_DVFS("vi",		0, 1, 1, KHZ,   228000, 420000, 480000, 600000, 600000, 600000, 600000, 600000),
	CORE_DVFS("vi",		1, -1, 1, KHZ,  228000, 420000, 480000, 600000, 600000, 600000, 600000, 600000),
	CORE_DVFS("isp",	0, 0, 1, KHZ,   228000, 408000, 480000, 600000, 600000, 600000, 600000, 600000),
	CORE_DVFS("isp",	0, 1, 1, KHZ,   228000, 420000, 480000, 600000, 600000, 600000, 600000, 600000),
	CORE_DVFS("isp",	1, -1, 1, KHZ,  228000, 420000, 480000, 600000, 600000, 600000, 600000, 600000),
	CORE_DVFS("pll_m",  -1, -1, 1, KHZ,   800000,  800000, 1066000, 1066000, 1066000, 1066000, 1200000, 1200000),
	CORE_DVFS("pll_c",  -1, -1, 1, KHZ,   800000,  800000, 1066000, 1066000, 1066000, 1066000, 1066000, 1066000),
	CORE_DVFS("pll_c2", -1, -1, 1, KHZ,   800000,  800000, 1066000, 1066000, 1066000, 1066000, 1066000, 1066000),
	CORE_DVFS("pll_c3", -1, -1, 1, KHZ,   800000,  800000, 1066000, 1066000, 1066000, 1066000, 1066000, 1066000),
	/* Clock limits for I/O peripherals */
	CORE_DVFS("sbc1",   -1, -1, 1, KHZ,    33000,  33000,  33000,  33000,   33000,  33000,  51000,  51000),
	CORE_DVFS("sbc2",   -1, -1, 1, KHZ,    33000,  33000,  33000,  33000,   33000,  33000,  51000,  51000),
	CORE_DVFS("sbc3",   -1, -1, 1, KHZ,    33000,  33000,  33000,  33000,   33000,  33000,  51000,  51000),
	CORE_DVFS("sbc4",   -1, -1, 1, KHZ,    33000,  33000,  33000,  33000,   33000,  33000,  51000,  51000),
	CORE_DVFS("sbc5",   -1, -1, 1, KHZ,    33000,  33000,  33000,  33000,   33000,  33000,  51000,  51000),
	CORE_DVFS("sbc6",   -1, -1, 1, KHZ,    33000,  33000,  33000,  33000,   33000,  33000,  51000,  51000),
	CORE_DVFS("sdmmc1", -1, -1, 1, KHZ,  1, 1, 82000, 82000,  136000, 136000, 136000, 204000),
	CORE_DVFS("sdmmc3", -1, -1, 1, KHZ,  1, 1, 82000, 82000,  136000, 136000, 136000, 204000),
	CORE_DVFS("sdmmc4", -1, -1, 1, KHZ,  1, 1, 82000, 82000,  136000, 136000, 136000, 204000),
	CORE_DVFS("hdmi",   -1, -1, 1, KHZ,  1, 148500, 148500, 297000,  297000, 297000, 297000, 297000),
	CORE_DVFS("nor",    -1, -1, 1, KHZ,   102000, 102000, 102000, 102000,  102000, 102000, 102000, 102000),
	CORE_DVFS("pciex",  -1,  -1, 1, KHZ,	1, 250000, 250000, 500000,  500000, 500000, 500000, 500000),
	CORE_DVFS("mselect", -1, -1, 1, KHZ,  102000, 102000, 204000, 204000, 204000, 204000, 408000, 408000),
	CORE_DVFS("xusb_falcon_src", -1, -1, 1, KHZ, 1, 336000, 336000, 336000, 336000, 336000, 336000, 336000),
	CORE_DVFS("xusb_host_src", -1, -1, 1, KHZ, 1, 112000, 112000, 112000, 112000, 112000, 112000, 112000),
	CORE_DVFS("xusb_dev_src", -1, -1, 1, KHZ, 1, 58300, 58300, 58300, 112000, 112000, 112000, 112000),
	CORE_DVFS("xusb_ss_src", -1, -1, 1, KHZ, 1, 120000, 120000, 120000, 120000, 120000, 120000, 120000),
	CORE_DVFS("xusb_fs_src", -1, -1, 1, KHZ, 1, 48000, 48000, 48000, 48000, 48000, 48000, 48000),
	CORE_DVFS("xusb_hs_src", -1, -1, 1, KHZ, 1, 60000, 60000, 60000, 60000, 60000, 60000, 60000),
	CORE_DVFS("hda", -1, -1, 1, KHZ, 1, 108000, 108000, 108000, 108000, 108000, 108000, 108000),
	CORE_DVFS("hda2codec_2x", -1, -1, 1, KHZ, 1, 48000, 48000, 48000, 48000, 48000, 48000, 48000),
	CORE_DVFS("sor0", -1, -1, 1, KHZ, 162000, 270000, 540000, 540000, 540000, 540000, 540000, 540000),
	/*
	 * The clock rate for the display controllers that determines the
	 * necessary core voltage depends on a divider that is internal
	 * to the display block.  Disable auto-dvfs on the display clocks,
	 * and let the display driver call tegra_dvfs_set_rate manually
	 */
	CORE_DVFS("disp1",	0, 0, 0, KHZ,   148500, 241000, 297000, 297000, 297000, 474000, 474000, 474000),
	CORE_DVFS("disp1",	0, 1, 0, KHZ,   148500, 241000, 297000, 297000, 474000, 474000, 474000, 474000),
	CORE_DVFS("disp1",	1, -1, 0, KHZ,  148500, 241000, 297000, 297000, 474000, 474000, 474000, 533000),
	CORE_DVFS("disp2",	0, 0, 0, KHZ,   148500, 241000, 297000, 297000, 297000, 474000, 474000, 474000),
	CORE_DVFS("disp2",	0, 1, 0, KHZ,   148500, 241000, 297000, 297000, 474000, 474000, 474000, 474000),
	CORE_DVFS("disp2",	1, -1, 0, KHZ,  148500, 241000, 297000, 297000, 474000, 474000, 474000, 533000),
};

static struct dvfs core_dvfs_table_tegra132[] = {
	/* Core voltages (mV):		         800,    850,    900,	 950,    1000,	1050,    1100,	 1150 */
	/* Clock limits for internal blocks, PLLs */
	CORE_DVFS("emc",	-1, -1, 1, KHZ, 264000, 348000, 384000, 384000, 528000, 528000, 1066000, 1200000),
	CORE_DVFS("vic03",	0, 0, 1, KHZ,   180000, 240000, 324000, 420000, 492000, 576000, 648000, 720000),
	CORE_DVFS("vic03",	0, 1, 1, KHZ,   180000, 336000, 420000, 504000, 600000, 684000, 720000, 720000),
	CORE_DVFS("tsec",	0, 0, 1, KHZ,   180000, 240000, 324000, 420000, 492000, 576000, 648000, 720000),
	CORE_DVFS("tsec",	0, 1, 1, KHZ,   180000, 336000, 420000, 504000, 600000, 684000, 720000, 720000),
	CORE_DVFS("msenc",	0, 0, 1, KHZ,   84000, 168000, 216000, 276000, 324000, 372000, 420000, 456000),
	CORE_DVFS("msenc",	0, 1, 1, KHZ,   120000, 228000, 276000, 348000, 396000, 444000, 456000, 456000),
	CORE_DVFS("se",		0, 0, 1, KHZ,   84000, 168000, 216000, 276000, 324000, 372000, 420000, 456000),
	CORE_DVFS("se",		0, 1, 1, KHZ,   120000, 228000, 276000, 348000, 396000, 444000, 456000, 456000),
	CORE_DVFS("vde",	0, 0, 1, KHZ,   84000, 168000, 216000, 276000, 324000, 372000, 420000, 456000),
	CORE_DVFS("vde",	0, 1, 1, KHZ,   120000, 228000, 276000, 348000, 396000, 444000, 456000, 456000),
	CORE_DVFS("host1x",	0, 0, 1, KHZ,   108000, 156000, 204000, 240000, 348000, 372000, 408000, 408000),
	CORE_DVFS("host1x",	0, 1, 1, KHZ,   108000, 156000, 204000, 252000, 348000, 384000, 408000, 408000),
	CORE_DVFS("vi",		0, 0, 1, KHZ,   1, 324000, 420000, 516000, 600000, 600000, 600000, 600000),
	CORE_DVFS("vi",		0, 1, 1, KHZ,   1, 420000, 480000, 600000, 600000, 600000, 600000, 600000),
	CORE_DVFS("isp",	0, 0, 1, KHZ,   1, 324000, 420000, 516000, 600000, 600000, 600000, 600000),
	CORE_DVFS("isp",	0, 1, 1, KHZ,   1, 420000, 480000, 600000, 600000, 600000, 600000, 600000),
	CORE_DVFS("pll_m",  -1, -1, 1, KHZ,   800000,  800000, 1066000, 1066000, 1066000, 1066000, 1200000, 1200000),
	CORE_DVFS("pll_c",  -1, -1, 1, KHZ,   800000,  800000, 1066000, 1066000, 1066000, 1066000, 1066000, 1066000),
	CORE_DVFS("pll_c2", -1, -1, 1, KHZ,   800000,  800000, 1066000, 1066000, 1066000, 1066000, 1066000, 1066000),
	CORE_DVFS("pll_c3", -1, -1, 1, KHZ,   800000,  800000, 1066000, 1066000, 1066000, 1066000, 1066000, 1066000),
	/* Clock limits for I/O peripherals */
	CORE_DVFS("sdmmc1", -1, -1, 1, KHZ,  1, 1, 82000, 82000,  136000, 136000, 136000, 204000),
	CORE_DVFS("sdmmc3", -1, -1, 1, KHZ,  1, 1, 82000, 82000,  136000, 136000, 136000, 204000),
	CORE_DVFS("sdmmc4", -1, -1, 1, KHZ,  1, 1, 82000, 82000,  136000, 136000, 136000, 200000),
	CORE_DVFS("hdmi",   -1, -1, 1, KHZ,  1, 148500, 148500, 297000,  297000, 297000, 297000, 297000),
	CORE_DVFS("nor",    -1, -1, 1, KHZ,   102000, 102000, 102000, 102000,  102000, 102000, 102000, 102000),
	CORE_DVFS("pciex",  -1,  -1, 1, KHZ,	1, 250000, 250000, 500000,  500000, 500000, 500000, 500000),
	CORE_DVFS("mselect", -1, -1, 1, KHZ,  102000, 102000, 204000, 204000, 204000, 204000, 408000, 408000),
	CORE_DVFS("xusb_falcon_src", -1, -1, 1, KHZ, 1, 336000, 336000, 336000, 336000, 336000, 336000, 336000),
	CORE_DVFS("xusb_host_src", -1, -1, 1, KHZ, 1, 112000, 112000, 112000, 112000, 112000, 112000, 112000),
	CORE_DVFS("xusb_dev_src", -1, -1, 1, KHZ, 1, 58300, 58300, 58300, 112000, 112000, 112000, 112000),
	CORE_DVFS("xusb_ss_src", -1, -1, 1, KHZ, 1, 120000, 120000, 120000, 120000, 120000, 120000, 120000),
	CORE_DVFS("xusb_fs_src", -1, -1, 1, KHZ, 1, 48000, 48000, 48000, 48000, 48000, 48000, 48000),
	CORE_DVFS("xusb_hs_src", -1, -1, 1, KHZ, 1, 60000, 60000, 60000, 60000, 60000, 60000, 60000),
	CORE_DVFS("hda", -1, -1, 1, KHZ, 1, 108000, 108000, 108000, 108000, 108000, 108000, 108000),
	CORE_DVFS("hda2codec_2x", -1, -1, 1, KHZ, 1, 48000, 48000, 48000, 48000, 48000, 48000, 48000),
	CORE_DVFS("sor0", -1, -1, 1, KHZ, 162500, 270000, 540000, 540000, 540000, 540000, 540000, 540000),
	/*
	 * The clock rate for the display controllers that determines the
	 * necessary core voltage depends on a divider that is internal
	 * to the display block.  Disable auto-dvfs on the display clocks,
	 * and let the display driver call tegra_dvfs_set_rate manually
	 */
	CORE_DVFS("disp1",	0, 0, 0, KHZ,   1, 240000, 282000, 330000, 388000, 408000, 456000, 490000),
	CORE_DVFS("disp1",	0, 1, 0, KHZ,   192000, 247000, 306000, 342000, 400000, 432000, 474000, 535000),
	CORE_DVFS("disp2",	0, 0, 0, KHZ,   1, 240000, 282000, 330000, 388000, 408000, 456000, 490000),
	CORE_DVFS("disp2",	0, 1, 0, KHZ,   192000, 247000, 306000, 342000, 400000, 432000, 474000, 535000),
};

int tegra_dvfs_disable_core_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_core_disabled)
		tegra_dvfs_rail_disable(tegra_dvfs_core_rail);
	else
		tegra_dvfs_rail_enable(tegra_dvfs_core_rail);

	return 0;
}

int tegra_dvfs_disable_cpu_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_cpu_disabled)
		tegra_dvfs_rail_disable(tegra_dvfs_cpu_rail);
	else
		tegra_dvfs_rail_enable(tegra_dvfs_cpu_rail);

	return 0;
}

int tegra_dvfs_disable_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_bool(buffer, kp);
}

static struct kernel_param_ops tegra_dvfs_disable_core_ops = {
	.set = tegra_dvfs_disable_core_set,
	.get = tegra_dvfs_disable_get,
};

static struct kernel_param_ops tegra_dvfs_disable_cpu_ops = {
	.set = tegra_dvfs_disable_cpu_set,
	.get = tegra_dvfs_disable_get,
};

module_param_cb(disable_core, &tegra_dvfs_disable_core_ops,
	&tegra_dvfs_core_disabled, 0644);
module_param_cb(disable_cpu, &tegra_dvfs_disable_cpu_ops,
	&tegra_dvfs_cpu_disabled, 0644);

static void init_dvfs_one(struct dvfs *d, int max_freq_index)
{
	int ret;
	struct clk *c = clk_get_sys(d->clk_name, d->clk_name);

	if (IS_ERR(c)) {
		pr_debug("%s_dvfs: no clock found for %s\n",
			tegra_soc_name, d->clk_name);
		return;
	}

	d->max_millivolts = d->dvfs_rail->nominal_millivolts;

	ret = tegra_setup_dvfs(c, d);
	if (ret)
		pr_err("%s_dvfs: failed to enable dvfs on %s\n",
			tegra_soc_name,	__clk_get_name(c));
}

static bool match_dvfs_one(const char *name,
	int dvfs_speedo_id, int dvfs_process_id,
	int speedo_id, int process_id)
{
	if ((dvfs_process_id != -1 && dvfs_process_id != process_id) ||
		(dvfs_speedo_id != -1 && dvfs_speedo_id != speedo_id)) {
		pr_debug("%s_dvfs: rejected %s speedo %d, process %d\n",
			 tegra_soc_name, name, dvfs_speedo_id, dvfs_process_id);
		return false;
	}
	return true;
}

static int round_voltage(int mv, struct rail_alignment *align, bool up)
{
	if (align->step_uv) {
		int uv = max(mv * 1000, align->offset_uv) - align->offset_uv;
		uv = (uv + (up ? align->step_uv - 1 : 0)) / align->step_uv;
		return (uv * align->step_uv + align->offset_uv) / 1000;
	}
	return mv;
}

static int set_cpu_dvfs_data(unsigned long max_freq,
	struct cpu_dvfs *d, struct dvfs *cpu_dvfs, int *max_freq_index)
{
	int i, mv, dfll_mv, min_dfll_mv, num_freqs;
	unsigned long fmax_at_vmin = 0;
	unsigned long fmin_use_dfll = 0;
	unsigned long *freqs;
	int *dfll_millivolts;
	struct rail_alignment *align =
		&tegra_dvfs_cpu_rail->alignment;

	min_dfll_mv = d->min_mv;
	min_dfll_mv = round_voltage(min_dfll_mv, align, true);
	d->max_mv = round_voltage(d->max_mv, align, false);
	BUG_ON(min_dfll_mv < tegra_dvfs_cpu_rail->min_millivolts);

	if (tegra_get_cpu_fv_table(&num_freqs, &freqs, &dfll_millivolts))
		return -EPROBE_DEFER;

	for (i = 0; i < num_freqs; i++) {
		if (freqs[i] / 1000 != d->fv_table[i].freq) {
			pr_err("Err: DFLL freq ladder does not match PLL's\n");
			return -EINVAL;
		}

		if (d->fv_table[i].freq > max_freq)
			break;

		mv = d->fv_table[i].volt;
		/*
		 * Check maximum frequency at minimum voltage for dfll source;
		 * round down unless all table entries are above Vmin, then use
		 * the 1st entry as is.
		 */
		dfll_mv = max(dfll_millivolts[i] / 1000, min_dfll_mv);
		if (dfll_mv > min_dfll_mv) {
			if (!i)
				fmax_at_vmin = freqs[i];
			if (!fmax_at_vmin)
				fmax_at_vmin = freqs[i - 1];
		}

		/* Clip maximum frequency at maximum voltage for pll source */
		if ((mv > d->max_mv) && !i) {
			pr_err("Err: volt of 1st entry is higher than Vmax\n");
			return -EINVAL;
		}

		/* Minimum rate with pll source voltage above dfll Vmin */
		if ((mv >= min_dfll_mv) && !fmin_use_dfll)
			fmin_use_dfll = freqs[i];

		/* fill in dvfs tables */
		cpu_dvfs->freqs[i] = freqs[i];
		cpu_millivolts[i] = mv;
		cpu_dfll_millivolts[i] = min(dfll_mv, d->max_mv);
	}

	/*
	 * In the dfll operating range dfll voltage at any rate should be
	 * better (below) than pll voltage
	 */
	if (!fmin_use_dfll || (fmin_use_dfll > fmax_at_vmin))
		fmin_use_dfll = fmax_at_vmin;

	/* dvfs tables are successfully populated - fill in the rest */
	cpu_dvfs->speedo_id = d->speedo_id;
	cpu_dvfs->process_id = d->process_id;
	cpu_dvfs->dvfs_rail->nominal_millivolts = min(d->max_mv,
		max(cpu_millivolts[i - 1], cpu_dfll_millivolts[i - 1]));
	*max_freq_index = i - 1;

	cpu_dvfs->use_dfll_rate_min = fmin_use_dfll;

	return 0;
}

static int get_core_nominal_mv_index(int speedo_id)
{
	int i, mv;

	switch (tegra_sku_info.soc_speedo_id) {
	case 0:
	case 1:
		mv = 1150;
		break;
	default:
		pr_err("Un-supported %s speedo %d\n",
			tegra_soc_name, tegra_sku_info.soc_speedo_id);
		return -EINVAL;
	}

	/* Round nominal level down to the nearest core scaling step */
	for (i = 0; i < MAX_DVFS_FREQS; i++) {
		if ((core_millivolts[i] == 0) || (mv < core_millivolts[i]))
			break;
	}
	if (i == 0) {
		pr_err("tegra124-dvfs: failed to get nominal idx at volt %d\n",
				mv);
		return -ENOSYS;
	}

	return i - 1;
}

int tegra124_init_dvfs(void)
{
	int cpu_speedo_id = tegra_sku_info.cpu_speedo_id;
	int cpu_process_id = tegra_sku_info.cpu_process_id;
	int soc_speedo_id = tegra_sku_info.soc_speedo_id;
	int core_process_id = tegra_sku_info.core_process_id;
	int i, ret;
	int core_nominal_mv_index;
	int cpu_max_freq_index = 0;
	int cpu_max_freq_size, cpu_fv_dvfs_table_size;
	int core_dvfs_table_size, num_rails;
	int chip_id = tegra_get_chip_id();

	if (chip_id == TEGRA124) {
		cpu_max_freq_tegra = cpu_max_freq_tegra124;
		cpu_max_freq_size = ARRAY_SIZE(cpu_max_freq_tegra124);
		cpu_fv_dvfs_table = cpu_fv_dvfs_table_tegra124;
		cpu_fv_dvfs_table_size = ARRAY_SIZE(cpu_fv_dvfs_table_tegra124);
		core_dvfs_table = core_dvfs_table_tegra124;
		core_dvfs_table_size = ARRAY_SIZE(core_dvfs_table_tegra124);
		tegra_dvfs_cpu_rail = &tegra124_dvfs_rail_vdd_cpu;
		tegra_dvfs_core_rail = &tegra124_dvfs_rail_vdd_core;
		tegra_dvfs_rails = tegra124_dvfs_rails;
		num_rails = ARRAY_SIZE(tegra124_dvfs_rails);
	} else if (chip_id == TEGRA132) {
		cpu_max_freq_tegra = cpu_max_freq_tegra132;
		cpu_max_freq_size = ARRAY_SIZE(cpu_max_freq_tegra132);
		cpu_fv_dvfs_table = cpu_fv_dvfs_table_tegra132;
		cpu_fv_dvfs_table_size = ARRAY_SIZE(cpu_fv_dvfs_table_tegra132);
		core_dvfs_table = core_dvfs_table_tegra132;
		core_dvfs_table_size = ARRAY_SIZE(core_dvfs_table_tegra132);
		tegra_dvfs_cpu_rail = &tegra132_dvfs_rail_vdd_cpu;
		tegra_dvfs_core_rail = &tegra132_dvfs_rail_vdd_core;
		tegra_dvfs_rails = tegra132_dvfs_rails;
		num_rails = ARRAY_SIZE(tegra132_dvfs_rails);
	} else {
		BUG();
	}
	tegra_soc_name = (chip_id == TEGRA124) ? "tegra124" : "tegra132";
	cpu_dvfs.dvfs_rail = tegra_dvfs_cpu_rail;

	/*
	 * Find nominal voltages for core (1st) and cpu rails before rail
	 * init. Nominal voltage index in core scaling ladder can also be
	 * used to determine max dvfs frequencies for all core clocks. In
	 * case of error disable core scaling and set index to 0, so that
	 * core clocks would not exceed rates allowed at minimum voltage.
	 */
	core_nominal_mv_index = get_core_nominal_mv_index(soc_speedo_id);
	if (core_nominal_mv_index < 0) {
		tegra_dvfs_core_rail->disabled = true;
		tegra_dvfs_core_disabled = true;
		core_nominal_mv_index = 0;
	}
	tegra_dvfs_core_rail->nominal_millivolts =
		core_millivolts[core_nominal_mv_index];

	/*
	 * Setup cpu dvfs and dfll tables from cvb data, determine nominal
	 * voltage for cpu rail, and cpu maximum frequency. Note that entire
	 * frequency range is guaranteed only when dfll is used as cpu clock
	 * source. Reaching maximum frequency with pll as cpu clock source
	 * may not be possible within nominal voltage range (dvfs mechanism
	 * would automatically fail frequency request in this case, so that
	 * voltage limit is not violated). Error when cpu dvfs table can not
	 * be constructed must never happen.
	 */
	BUG_ON(cpu_speedo_id >= cpu_max_freq_size);
	for (ret = 0, i = 0; i <  cpu_fv_dvfs_table_size; i++) {
		struct cpu_dvfs *d = &cpu_fv_dvfs_table[i];
		unsigned long max_freq = cpu_max_freq_tegra[cpu_speedo_id];
		if (match_dvfs_one("cpu dvfs", d->speedo_id, d->process_id,
				   cpu_speedo_id, cpu_process_id)) {
			ret = set_cpu_dvfs_data(max_freq,
				d, &cpu_dvfs, &cpu_max_freq_index);
			if (ret)
				goto out;
			break;
		}
	}
	BUG_ON(i == cpu_fv_dvfs_table_size);

	/* Init rail structures and dependencies */
	tegra_dvfs_init_rails(tegra_dvfs_rails, num_rails);

	/*
	 * Search core dvfs table for speedo/process matching entries and
	 * initialize dvfs-ed clocks
	 */
	for (i = 0; i < core_dvfs_table_size; i++) {
		struct dvfs *d = &core_dvfs_table[i];
		d->dvfs_rail = tegra_dvfs_core_rail;
		if (!match_dvfs_one(d->clk_name, d->speedo_id,
			d->process_id, soc_speedo_id, core_process_id))
			continue;
		init_dvfs_one(d, core_nominal_mv_index);
	}

	/*
	 * Initialize matching cpu dvfs entry already found when nominal
	 * voltage was determined
	 */
	init_dvfs_one(&cpu_dvfs, cpu_max_freq_index);

	pr_info("%s_dvfs: VDD_CPU nominal %dmV, scaling %s\n", tegra_soc_name,
		tegra_dvfs_cpu_rail->nominal_millivolts,
		tegra_dvfs_cpu_disabled ? "disabled" : "enabled");
	pr_info("%s_dvfs: VDD_CORE nominal %dmV, scaling %s\n", tegra_soc_name,
		tegra_dvfs_core_rail->nominal_millivolts,
		tegra_dvfs_core_disabled ? "disabled" : "enabled");

	return 0;
out:
	return ret;
}
