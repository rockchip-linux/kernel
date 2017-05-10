/*
 * Copyright (c) 2017, Fuzhou Rockchip Electronics Co., Ltd
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
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 */
#include <linux/mfd/syscon.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rk_fb.h>
#include <linux/rockchip/common.h>
#include <linux/rockchip/psci.h>
#include <linux/rockchip/psci_ddr.h>
#include <asm/psci.h>

#define DDR_VERSION	"V1.00 20170426"

#define MHZ	(1000 * 1000)

/* GRF registers */
#define GRF_SOC_CON0		0x140
#define GRF_OS_REG1		0x1cc
#define GRF_DFI_WRNUM		0x220
#define GRF_DFI_RDNUM		0x224
#define GRF_DFI_ACTNUM		0x228
#define GRF_DFI_TIMERVAL	0x22c

/* GRF_SOC_CON0 */
#define DDR_MONITOR_EN		((1 << (16 + 6)) + (1 << 6))
#define DDR_MONITOR_DISB	((1 << (16 + 6)) + (0 << 6))

enum ddr_bandwidth_id {
	ddrbw_wr_num = 0,
	ddrbw_rd_num,
	ddrbw_act_num,
	ddrbw_time_num,
	ddrbw_id_end
};

struct share_params {
	u32 hz;
	u32 lcdc_type;
	u32 vop;
	u32 vop_dclk_mode;
	u32 sr_idle_en;
	/* if need, add parameter after */
};

struct ddr_dts_config_timing {
	u32 ddr3_speed_bin;
	u32 pd_idle;
	u32 sr_idle;
	u32 auto_pd_dis_freq;
	u32 auto_sr_dis_freq;
	u32 ddr3_dll_dis_freq;
	u32 lpddr2_dll_dis_freq;
	u32 phy_dll_dis_freq;
	u32 ddr3_odt_dis_freq;
	u32 phy_ddr3_odt_disb_freq;
	u32 ddr3_drv;
	u32 ddr3_odt;
	u32 phy_ddr3_clk_drv;
	u32 phy_ddr3_cmd_drv;
	u32 phy_ddr3_dqs_drv;
	u32 phy_ddr3_odt;
	u32 lpddr2_drv;
	u32 phy_lpddr2_clk_drv;
	u32 phy_lpddr2_cmd_drv;
	u32 phy_lpddr2_dqs_drv;
	u32 available;
};

struct rockchip_ddr {
	u32 enable;
	struct device *dev;
	void __iomem *share_memory;
	struct regmap *grf_regs;
	struct ddr_dts_config_timing *ddr_timing;
};

static struct rockchip_ddr *ddr_data;

static int of_get_ddr_timings(struct device_node *np,
			      struct  ddr_dts_config_timing *timing)
{
	struct device_node *np_tim;
	int ret = 0;

	np_tim = of_parse_phandle(np, "rockchip,ddr_timing", 0);

	if (!np_tim) {
		ret = -EINVAL;
		goto end;
	}

	ret |= of_property_read_u32(np_tim, "ddr3_speed_bin",
				    &timing->ddr3_speed_bin);
	ret |= of_property_read_u32(np_tim, "pd_idle", &timing->pd_idle);
	ret |= of_property_read_u32(np_tim, "sr_idle", &timing->sr_idle);
	ret |= of_property_read_u32(np_tim, "auto_pd_dis_freq",
				    &timing->auto_pd_dis_freq);
	ret |= of_property_read_u32(np_tim, "auto_sr_dis_freq",
				    &timing->auto_sr_dis_freq);
	ret |= of_property_read_u32(np_tim, "ddr3_dll_dis_freq",
				    &timing->ddr3_dll_dis_freq);
	ret |= of_property_read_u32(np_tim, "lpddr2_dll_dis_freq",
				    &timing->lpddr2_dll_dis_freq);
	ret |= of_property_read_u32(np_tim, "phy_dll_dis_freq",
				    &timing->phy_dll_dis_freq);
	ret |= of_property_read_u32(np_tim, "ddr3_odt_dis_freq",
				    &timing->ddr3_odt_dis_freq);
	ret |= of_property_read_u32(np_tim, "phy_ddr3_odt_disb_freq",
				    &timing->phy_ddr3_odt_disb_freq);
	ret |= of_property_read_u32(np_tim, "ddr3_drv", &timing->ddr3_drv);
	ret |= of_property_read_u32(np_tim, "ddr3_odt", &timing->ddr3_odt);
	ret |= of_property_read_u32(np_tim, "phy_ddr3_clk_drv",
				    &timing->phy_ddr3_clk_drv);
	ret |= of_property_read_u32(np_tim, "phy_ddr3_cmd_drv",
				    &timing->phy_ddr3_cmd_drv);
	ret |= of_property_read_u32(np_tim, "phy_ddr3_dqs_drv",
				    &timing->phy_ddr3_dqs_drv);
	ret |= of_property_read_u32(np_tim, "phy_ddr3_odt",
				    &timing->phy_ddr3_odt);
	ret |= of_property_read_u32(np_tim, "lpddr2_drv", &timing->lpddr2_drv);
	ret |= of_property_read_u32(np_tim, "phy_lpddr2_clk_drv",
				    &timing->phy_lpddr2_clk_drv);
	ret |= of_property_read_u32(np_tim, "phy_lpddr2_cmd_drv",
				    &timing->phy_lpddr2_cmd_drv);
	ret |= of_property_read_u32(np_tim, "phy_lpddr2_dqs_drv",
				    &timing->phy_lpddr2_dqs_drv);

	if (ret) {
		ret = -EINVAL;
		goto end;
	}

end:
	if (np_tim)
		of_node_put(np_tim);

	return ret;
}

static int _ddr_recalc_rate(void)
{
	struct arm_smccc_res res;

	if (!ddr_data->enable)
		return 0;

	res = sip_smc_dram(0, 0, DRAM_FREQ_CONFIG_DRAM_GET_RATE);
	if (res.a0)
		return 0;
	else
		return (res.a1 / MHZ);
}

static long _ddr_round_rate(u32 n_mhz)
{
	struct arm_smccc_res res;
	struct share_params *p =
		(struct share_params *)ddr_data->share_memory;

	if (!ddr_data->enable)
		return 0;

	p->hz = n_mhz * MHZ;
	res = sip_smc_dram(SHARE_PAGE_TYPE_DDR,
			   0,
			   DRAM_FREQ_CONFIG_DRAM_ROUND_RATE);

	if (res.a0)
		return 0;
	else
		return (res.a1 / MHZ);
}

static void _ddr_set_auto_self_refresh(bool en)
{
	struct share_params *p =
		(struct share_params *)ddr_data->share_memory;

	if (!ddr_data->enable)
		return;

	p->sr_idle_en = en;
	sip_smc_dram(SHARE_PAGE_TYPE_DDR,
		     0,
		     DRAM_FREQ_CONFIG_DRAM_SET_AT_SR);
}

static int _ddr_change_freq(u32 n_mhz)
{
	struct share_params *p =
		(struct share_params *)ddr_data->share_memory;
	struct arm_smccc_res res;

	if (!ddr_data->enable)
		return 0;

	p->hz = n_mhz * MHZ;

	res = sip_smc_dram(SHARE_PAGE_TYPE_DDR, 0,
			   DRAM_FREQ_CONFIG_DRAM_FREQ_CHANGE);

	if (res.a0) {
		pr_info("_ddr_change_freq_ error:%lx\n",
			res.a0);
		return -ENOMEM;
	}

	return (res.a1 / MHZ);
}

static int ddr_monitor_init(struct platform_device *pdev,
			    struct rockchip_ddr *ddr_data)
{
	struct device_node *np = pdev->dev.of_node;

	ddr_data->grf_regs =
	    syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(ddr_data->grf_regs)) {
		dev_err(&pdev->dev, "%s: could not find grf dt node\n",
			__func__);
		return -ENXIO;
	}

	return 0;
}

static void ddr_monitor_start(void)
{
	regmap_write(ddr_data->grf_regs, GRF_SOC_CON0,
		     DDR_MONITOR_EN);
}

static void ddr_monitor_stop(void)
{
	regmap_write(ddr_data->grf_regs, GRF_SOC_CON0,
		     DDR_MONITOR_DISB);
}

static u32 ddr_read_bw_info(void)
{
	u32 tmp, ret;

	regmap_read(ddr_data->grf_regs, GRF_OS_REG1,
		    &tmp);

	/* 0->8bit 1->16bit 2->32bit*/
	ret = 2 >> ((tmp & 0xc) >> 2);

	return ret;
}

static void _ddr_bandwidth_get(struct ddr_bw_info *ddr_bw_ch0,
			       struct ddr_bw_info *ddr_bw_ch1)
{
	u32 monitor_rd, monitor_wr;
	u32 monitor_act, monitor_time;
	u64 time;
	u64 ddr_freq_hz = 0;
	u64 bw;
	u64 tmp64;

	if (!ddr_data)
		return;

	ddr_monitor_stop();

	regmap_read(ddr_data->grf_regs, GRF_DFI_WRNUM,
		    &monitor_wr);
	regmap_read(ddr_data->grf_regs, GRF_DFI_RDNUM,
		    &monitor_rd);
	regmap_read(ddr_data->grf_regs, GRF_DFI_ACTNUM,
		    &monitor_act);
	regmap_read(ddr_data->grf_regs, GRF_DFI_TIMERVAL,
		    &monitor_time);

	bw = ddr_read_bw_info();

	ddr_freq_hz = _ddr_recalc_rate() * MHZ;

	if (!ddr_freq_hz)
		return;

	/* us */
	tmp64 = (u64)monitor_time * (u64)1000000;
	do_div(tmp64, ddr_freq_hz);
	time = tmp64;

	/*
	 * 4: all ddr use burst 8, so each read/write
	 * command occupy 4 cycle
	 */
	tmp64 = ((u64)monitor_wr + (u64)monitor_rd) *
			(u64)4 *
			(u64)100;
	do_div(tmp64, monitor_time);
	ddr_bw_ch0->ddr_percent = tmp64;
	/* ms */
	tmp64 = time;
	do_div(tmp64, (u64)1000);
	ddr_bw_ch0->ddr_time = tmp64;
	/* 8: ddr use burst 8 */
	/*unit:MB/s */
	tmp64 = (u64)monitor_wr * (u64)8 * bw * 2;
	do_div(tmp64, time);
	ddr_bw_ch0->ddr_wr = tmp64;
	tmp64 = (u64)monitor_rd * (u64)8 * bw * 2;
	do_div(tmp64, time);
	ddr_bw_ch0->ddr_rd = tmp64;
	ddr_bw_ch0->ddr_act = monitor_act;
	tmp64 = ddr_freq_hz * (u64)2 * bw * 2;
	do_div(tmp64, 1000 * 1000);
	ddr_bw_ch0->ddr_total = tmp64;

	ddr_monitor_start();
}

static void ddr_init(struct platform_device *pdev, u32 page_type)
{
	struct share_params *p =
		(struct share_params *)ddr_data->share_memory;
	struct arm_smccc_res res;
	struct rk_lcdc_driver *lcdc_dev = NULL;
	struct device_node *np = pdev->dev.of_node;

	if (of_property_read_u32(np, "vop-dclk-mode", &p->vop_dclk_mode))
		p->vop_dclk_mode = 0;

	lcdc_dev = rk_get_lcdc_drv("lcdc0");
	if (!lcdc_dev)
		p->lcdc_type = 0;
	else
		p->lcdc_type = (u32)lcdc_dev->cur_screen->type;

	res = sip_smc_dram(page_type,
			   0,
			   DRAM_FREQ_CONFIG_DRAM_INIT);
	if (res.a0)
		pr_info("ddr init error\n");
}

static int __init rockchip_atf_ver_check(void)
{
	struct arm_smccc_res res;

	res = sip_smc_dram(SHARE_PAGE_TYPE_DDR, 0,
			   DRAM_FREQ_CONFIG_DRAM_GET_VERSION);
	pr_info("current DDR ATF driver version 0x%lx\n", res.a1);
	if ((!res.a0) && (res.a1 >= 0x100))
		return 0;

	pr_err("read tf version 0x%lx\n", res.a1);

	do {
		mdelay(1000);
		pr_err("trusted firmware need to update or is invalid!\n");
	} while (1);

	return -EINVAL;
}

static int __init rockchip_ddr_probe(struct platform_device *pdev)
{
	struct device_node *np;
	struct arm_smccc_res res;

	pr_info("Rockchip DDR Initialize, verision: " DDR_VERSION "\n");

	np = pdev->dev.of_node;
	ddr_data =
	    devm_kzalloc(&pdev->dev, sizeof(struct rockchip_ddr), GFP_KERNEL);
	if (!ddr_data) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	if (psci_smp_available() && (rockchip_atf_ver_check() == 0)) {
		ddr_data->enable = 1;
	} else {
		ddr_data->enable = 0;
		return -ENXIO;
	}

	/*
	 * first 4KB is used for interface parameters
	 * after 4KB * N is dts parameters
	 */
	res = sip_smc_request_share_mem(SHARE_PAGE_TYPE_DDR,
		DIV_ROUND_UP(sizeof(struct ddr_dts_config_timing),
			     4096) + 1);
	if (res.a0 != 0) {
		dev_err(&pdev->dev, "no ATF memory for init\n");
		return -ENOMEM;
	}

	/* ioremap share memory */
	ddr_data->share_memory = ioremap(res.a1,
		(DIV_ROUND_UP(sizeof(struct ddr_dts_config_timing),
			      4096) + 1) * 4096);
	if (!ddr_data->share_memory) {
		dev_err(&pdev->dev, "ioremap share memory fail\n");
		return -ENOMEM;
	}

	/* fill dts dram timing to share memory */
	ddr_data->ddr_timing =
		(struct ddr_dts_config_timing *)(ddr_data->share_memory +
		4096);

	if (!of_get_ddr_timings(np, ddr_data->ddr_timing)) {
		ddr_data->ddr_timing->available = 1;
	} else {
		dev_err(&pdev->dev, "of_get_ddr_timings: fail\n");
		ddr_data->ddr_timing->available = 0;
	}

	platform_set_drvdata(pdev, ddr_data);
	ddr_monitor_init(pdev, ddr_data);
	ddr_init(pdev, SHARE_PAGE_TYPE_DDR);

	ddr_round_rate = _ddr_round_rate;
	ddr_set_auto_self_refresh = _ddr_set_auto_self_refresh;
	ddr_bandwidth_get = _ddr_bandwidth_get;
	ddr_change_freq = _ddr_change_freq;

	return 0;
}

static const struct of_device_id rockchip_ddr_of_match[] __refdata = {
	{.compatible = "rockchip,rk312x-ddr", .data = NULL,},
	{},
};

static struct platform_driver rockchip_ddr_driver = {
	.driver = {
		   .name = "rockchip_ddr",
		   .of_match_table = rockchip_ddr_of_match,
	},
};

static int __init rockchip_ddr_init(void)
{
	return platform_driver_probe(&rockchip_ddr_driver, rockchip_ddr_probe);
}

device_initcall(rockchip_ddr_init);

