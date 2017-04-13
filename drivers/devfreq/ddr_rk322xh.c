/*
 * Copyright (c) 2015, Fuzhou Rockchip Electronics Co., Ltd
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
#include <dt-bindings/clock/ddr.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rk_fb.h>
#include <linux/rockchip/common.h>
#include <linux/rockchip/psci.h>
#include <linux/rockchip/psci_ddr.h>
#include <linux/compiler.h>
#include <linux/cpu.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>

#define DDR_VERSION			"V1.00 20161215"

#define DDR_PCTL2_MSTR			0x0
#define DDRMON_CTRL			0x4
#define DDRMON_CH0_DFI_ACT_NUM		0x1c
#define DDRMON_CH0_DFI_WR_NUM		0x20
#define DDRMON_CH0_DFI_RD_NUM		0x24
#define DDRMON_CH0_COUNT_NUM		0x28
#define DDRMON_CH0_DFI_ACCESS_NUM	0x2C

/* PCTL2_MRSTAT */
#define PCTL2_DDR3			BIT(0)
#define PCTL2_DDR4			BIT(4)
#define PCTL2_DATA_BUS_WIDTH_MASK	(3 << 12)
#define PCTL2_DATA_BUS_32BIT		(0 << 12)
#define PCTL2_DATA_BUS_16BIT		BIT(12)
#define PCTL2_DATA_BUS_8BIT		BIT(13)

/* DDRMON_CTRL */
#define MON_DDR3_EN			((0x34 << 16) | 0)
#define MON_DDR4_EN			((0x34 << 16) | (1 << 5))
#define MON_LPDDR4_EN			((0x34 << 16) |	(1 << 4))
#define MON_LPDDR2_3_EN			((0x34 << 16) | (1 << 2))
#define MON_HARDWARE_EN			((1 << (3 + 16)) |\
					(1 << 3))
#define MON_HARDWARE_DIS		((1 << (3 + 16)) |\
					(0 << 3))
#define MON_SOFTWARE_EN_MASK		((1 << (1 + 16)) |\
					(1 << 1))
#define MON_SOFTWARE_EN			((1 << (1 + 16)) |\
					(1 << 1))
#define MON_SOFTWARE_DIS		((1 << (1 + 16)) |\
					(0 << 1))
#define MON_TIMER_CNT_EN		((1 << (0 + 16)) |\
					(1 << 0))

enum ddr_bandwidth_id {
	ddrbw_wr_num = 0,
	ddrbw_rd_num,
	ddrbw_act_num,
	ddrbw_time_num,
	ddrbw_eff,
	ddrbw_id_end
};

struct  ddr_dts_config_timing {
	unsigned int ddr3_speed_bin;
	unsigned int ddr4_speed_bin;
	unsigned int pd_idle;
	unsigned int sr_idle;
	unsigned int sr_mc_gate_idle;
	unsigned int srpd_lite_idle;
	unsigned int standby_idle;

	unsigned int auto_pd_dis_freq;
	unsigned int auto_sr_dis_freq;
	/* for ddr3 only */
	unsigned int ddr3_dll_dis_freq;
	/* for ddr4 only */
	unsigned int ddr4_dll_dis_freq;
	unsigned int phy_dll_dis_freq;

	unsigned int ddr3_odt_dis_freq;
	unsigned int ddr3_drv;
	unsigned int ddr3_odt;
	unsigned int phy_ddr3_ca_drv;
	unsigned int phy_ddr3_ck_drv;
	unsigned int phy_ddr3_dq_drv;
	unsigned int phy_ddr3_odt;

	unsigned int lpddr3_odt_dis_freq;
	unsigned int lpddr3_drv;
	unsigned int lpddr3_odt;
	unsigned int phy_lpddr3_ca_drv;
	unsigned int phy_lpddr3_ck_drv;
	unsigned int phy_lpddr3_dq_drv;
	unsigned int phy_lpddr3_odt;

	unsigned int lpddr4_odt_dis_freq;
	unsigned int lpddr4_drv;
	unsigned int lpddr4_dq_odt;
	unsigned int lpddr4_ca_odt;
	unsigned int phy_lpddr4_ca_drv;
	unsigned int phy_lpddr4_ck_cs_drv;
	unsigned int phy_lpddr4_dq_drv;
	unsigned int phy_lpddr4_odt;

	unsigned int ddr4_odt_dis_freq;
	unsigned int ddr4_drv;
	unsigned int ddr4_odt;
	unsigned int phy_ddr4_ca_drv;
	unsigned int phy_ddr4_ck_drv;
	unsigned int phy_ddr4_dq_drv;
	unsigned int phy_ddr4_odt;

	unsigned int ca_skew[15];
	unsigned int cs0_skew[44];
	unsigned int cs1_skew[44];

	u32 available;
};

struct	ddr_de_skew_setting {
	unsigned int ca_de_skew[30];
	unsigned int cs0_de_skew[84];
	unsigned int cs1_de_skew[84];
};

struct init_params {
	/* these parameters, not use in RK322xh */
	u32 hz;
	u32 lcdc_type;
	u32 vop;
	u32 addr_mcu_el3;
	/* if need, add parameter after */
};

struct set_rate_params {
	u32 hz;
	/*
	 * 1: need to wait flag1
	 * 0: never wait flag1
	 */
	u32 wait_flag1;
	/*
	 * 1: need to wait flag1
	 * 0: never wait flag1
	 */
	u32 wait_flag0;
	/* these parameters, not use in RK322xh */
	u32 lcdc_type;
	u32 vop;
	/* if need, add parameter after */
};

struct round_rate_params {
	u32 hz;
	/* if need, add parameter after */
};

struct set_at_sr_params {
	u32 en;
	/* if need, add parameter after */
};

struct rockchip_ddr {
	u32 enable;
	void __iomem *share_memory;
	struct ddr_dts_config_timing *dram_timing;

	struct regmap *ddrpctl_regs;
	struct regmap *msch_regs;
	struct regmap *ddrmonitor_regs;
};

static struct rockchip_ddr *ddr_data;

struct probe {
	char *name;
	unsigned long offset;
	struct regmap *regs;
	u64 packets;
	u64 bytes;
};

struct probe probes[] = {
	{
		.name = "core",
		.offset = 0x1800,
	},
	{
		.name = "gpu",
		.offset = 0x0400,
	},
	{
		.name = "peri",
		.offset = 0x0800,
	},
	{
		.name = "vio0",
		.offset = 0x0C00,
	},
	{
		.name = "vio1",
		.offset = 0x1000,
	},
	{
		.name = "vpu",
		.offset = 0x1400,
	},
};

struct probe probes_pctl = {
	.name = "pctl",
	.offset = 0,
};

struct probe probes_monitor = {
	.name = "monitor",
	.offset = 0,
};

#define for_each_probe(_p) for (_p = probes;\
				_p <= &probes[ARRAY_SIZE(probes) - 1];\
				_p++)

/*
 * function: packaging de-skew setting to ddr_dts_config_timing,
 *           ddr_dts_config_timing will pass to trust firmware, and
 *           used direct to set register.
 * input: de_skew
 * output: tim
 */
static void de_skew_setting_2_register(struct ddr_de_skew_setting *de_skew,
				       struct  ddr_dts_config_timing *tim)
{
	u32 n;
	u32 offset;
	u32 shift;

	memset_io(tim->ca_skew, 0, sizeof(tim->ca_skew));
	memset_io(tim->cs0_skew, 0, sizeof(tim->cs0_skew));
	memset_io(tim->cs1_skew, 0, sizeof(tim->cs1_skew));

	/* CA de-skew */
	for (n = 0; n < ARRAY_SIZE(de_skew->ca_de_skew); n++) {
		offset = n / 2;
		shift = n % 2;
		/* 0 => 4; 1 => 0 */
		shift = (shift == 0) ? 4 : 0;
		tim->ca_skew[offset] &= ~(0xf << shift);
		tim->ca_skew[offset] |= (de_skew->ca_de_skew[n] << shift);
	}

	/* CS0 data de-skew */
	for (n = 0; n < ARRAY_SIZE(de_skew->cs0_de_skew); n++) {
		offset = ((n / 21) * 11) + ((n % 21) / 2);
		shift = ((n % 21) % 2);
		if ((n % 21) == 20)
			shift = 0;
		else
			/* 0 => 4; 1 => 0 */
			shift = (shift == 0) ? 4 : 0;
		tim->cs0_skew[offset] &= ~(0xf << shift);
		tim->cs0_skew[offset] |= (de_skew->cs0_de_skew[n] << shift);
	}

	/* CS1 data de-skew */
	for (n = 0; n < ARRAY_SIZE(de_skew->cs1_de_skew); n++) {
		offset = ((n / 21) * 11) + ((n % 21) / 2);
		shift = ((n % 21) % 2);
		if ((n % 21) == 20)
			shift = 0;
		else
			/* 0 => 4; 1 => 0 */
			shift = (shift == 0) ? 4 : 0;
		tim->cs1_skew[offset] &= ~(0xf << shift);
		tim->cs1_skew[offset] |= (de_skew->cs1_de_skew[n] << shift);
	}
}

static int of_get_ca_deskew(struct device_node *np_tim,
			    struct ddr_de_skew_setting *de_skew)
{
	u32 i;
	int ret = 0;

	i = 0;
	ret |= of_property_read_u32(np_tim, "ddr3a1_ddr4a9_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a0_ddr4a10_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a3_ddr4a6_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a2_ddr4a4_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a5_ddr4a8_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a4_ddr4a5_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a7_ddr4a11_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a6_ddr4a7_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a9_ddr4a0_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a8_ddr4a13_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a11_ddr4a3_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a10_ddr4cs0_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a13_ddr4a2_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a12_ddr4ba1_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a15_ddr4odt0_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3a14_ddr4a1_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3ba1_ddr4a15_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3ba0_ddr4bg0_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3ras_ddr4cke_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3ba2_ddr4ba0_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3we_ddr4bg1_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3cas_ddr4a12_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3ckn_ddr4ckn_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3ckp_ddr4ckp_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3cke_ddr4a16_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3odt0_ddr4a14_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3cs0_ddr4act_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3reset_ddr4reset_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3cs1_ddr4cs1_de-skew",
				    &de_skew->ca_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "ddr3odt1_ddr4odt1_de-skew",
				    &de_skew->ca_de_skew[i++]);

	return ret;
}

static int of_get_cs0_deskew(struct device_node	*np_tim,
			     struct ddr_de_skew_setting *de_skew)
{
	u32 i;
	int ret = 0;

	i = 0;
	ret |= of_property_read_u32(np_tim, "cs0_dm0_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dm0_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq0_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq0_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq1_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq1_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq2_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq2_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq3_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq3_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq4_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq4_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq5_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq5_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq6_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq6_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq7_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq7_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dqs0_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dqs0p_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dqs0n_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);

	ret |= of_property_read_u32(np_tim, "cs0_dm1_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dm1_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq8_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq8_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq9_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq9_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq10_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq10_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq11_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq11_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq12_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq12_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq13_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq13_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq14_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq14_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq15_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq15_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dqs1_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dqs1p_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dqs1n_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);

	ret |= of_property_read_u32(np_tim, "cs0_dm2_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dm2_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq16_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq16_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq17_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq17_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq18_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq18_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq19_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq19_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq20_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq20_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq21_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq21_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq22_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq22_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq23_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq23_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dqs2_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dqs2p_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dqs2n_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);

	ret |= of_property_read_u32(np_tim, "cs0_dm3_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dm3_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq24_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq24_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq25_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq25_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq26_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq26_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq27_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq27_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq28_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq28_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq29_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq29_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq30_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq30_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq31_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dq31_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dqs3_rx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dqs3p_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs0_dqs3n_tx_de-skew",
				    &de_skew->cs0_de_skew[i++]);

	return ret;
}

static int of_get_cs1_deskew(struct device_node *np_tim,
			     struct ddr_de_skew_setting *de_skew)
{
	u32 i;
	int ret = 0;

	i = 0;
	ret |= of_property_read_u32(np_tim, "cs1_dm0_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dm0_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq0_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq0_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq1_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq1_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq2_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq2_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq3_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq3_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq4_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq4_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq5_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq5_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq6_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq6_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq7_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq7_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dqs0_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dqs0p_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dqs0n_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);

	ret |= of_property_read_u32(np_tim, "cs1_dm1_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dm1_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq8_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq8_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq9_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq9_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq10_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq10_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq11_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq11_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq12_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq12_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq13_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq13_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq14_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq14_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq15_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq15_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dqs1_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dqs1p_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dqs1n_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);

	ret |= of_property_read_u32(np_tim, "cs1_dm2_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dm2_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq16_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq16_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq17_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq17_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq18_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq18_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq19_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq19_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq20_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq20_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq21_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq21_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq22_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq22_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq23_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq23_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dqs2_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dqs2p_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dqs2n_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);

	ret |= of_property_read_u32(np_tim, "cs1_dm3_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dm3_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq24_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq24_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq25_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq25_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq26_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq26_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq27_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq27_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq28_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq28_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq29_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq29_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq30_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq30_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq31_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dq31_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dqs3_rx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dqs3p_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);
	ret |= of_property_read_u32(np_tim, "cs1_dqs3n_tx_de-skew",
				    &de_skew->cs1_de_skew[i++]);

	return ret;
}

static int of_get_ddr_timings(struct device_node *np,
			      struct  ddr_dts_config_timing *tim)
{
	struct device_node	*np_tim;
	int ret = 0;
	struct ddr_de_skew_setting *de_skew;

	np_tim = of_parse_phandle(np, "rockchip,ddr_timing", 0);
	if (!np_tim) {
		ret = -EINVAL;
		goto end;
	}
	de_skew = kmalloc(sizeof(*de_skew), GFP_KERNEL);
	if (!de_skew) {
		ret = -ENOMEM;
		goto end;
	}

	ret |= of_property_read_u32(np_tim, "ddr3_speed_bin",
				   &tim->ddr3_speed_bin);
	ret |= of_property_read_u32(np_tim, "ddr4_speed_bin",
				   &tim->ddr4_speed_bin);
	ret |= of_property_read_u32(np_tim, "pd_idle",
				    &tim->pd_idle);
	ret |= of_property_read_u32(np_tim, "sr_idle",
				    &tim->sr_idle);
	ret |= of_property_read_u32(np_tim, "sr_mc_gate_idle",
				    &tim->sr_mc_gate_idle);
	ret |= of_property_read_u32(np_tim, "srpd_lite_idle",
				    &tim->srpd_lite_idle);
	ret |= of_property_read_u32(np_tim, "standby_idle",
				    &tim->standby_idle);
	ret |= of_property_read_u32(np_tim, "auto_pd_dis_freq",
				    &tim->auto_pd_dis_freq);
	ret |= of_property_read_u32(np_tim, "auto_sr_dis_freq",
				    &tim->auto_sr_dis_freq);
	ret |= of_property_read_u32(np_tim, "ddr3_dll_dis_freq",
				    &tim->ddr3_dll_dis_freq);
	ret |= of_property_read_u32(np_tim, "ddr4_dll_dis_freq",
				    &tim->ddr4_dll_dis_freq);
	ret |= of_property_read_u32(np_tim, "phy_dll_dis_freq",
				    &tim->phy_dll_dis_freq);
	ret |= of_property_read_u32(np_tim, "ddr3_odt_dis_freq",
				    &tim->ddr3_odt_dis_freq);
	ret |= of_property_read_u32(np_tim, "ddr3_drv",
				    &tim->ddr3_drv);
	ret |= of_property_read_u32(np_tim, "ddr3_odt",
				    &tim->ddr3_odt);
	ret |= of_property_read_u32(np_tim, "phy_ddr3_ca_drv",
				    &tim->phy_ddr3_ca_drv);
	ret |= of_property_read_u32(np_tim, "phy_ddr3_ck_drv",
				    &tim->phy_ddr3_ck_drv);
	ret |= of_property_read_u32(np_tim, "phy_ddr3_dq_drv",
				    &tim->phy_ddr3_dq_drv);
	ret |= of_property_read_u32(np_tim, "phy_ddr3_odt",
				    &tim->phy_ddr3_odt);
	ret |= of_property_read_u32(np_tim, "lpddr3_odt_dis_freq",
				    &tim->lpddr3_odt_dis_freq);
	ret |= of_property_read_u32(np_tim, "lpddr3_drv",
				    &tim->lpddr3_drv);
	ret |= of_property_read_u32(np_tim, "lpddr3_odt",
				    &tim->lpddr3_odt);
	ret |= of_property_read_u32(np_tim, "phy_lpddr3_ca_drv",
				    &tim->phy_lpddr3_ca_drv);
	ret |= of_property_read_u32(np_tim, "phy_lpddr3_ck_drv",
				    &tim->phy_lpddr3_ck_drv);
	ret |= of_property_read_u32(np_tim, "phy_lpddr3_dq_drv",
				    &tim->phy_lpddr3_dq_drv);
	ret |= of_property_read_u32(np_tim, "phy_lpddr3_odt",
				    &tim->phy_lpddr3_odt);
	ret |= of_property_read_u32(np_tim, "lpddr4_odt_dis_freq",
				    &tim->lpddr4_odt_dis_freq);
	ret |= of_property_read_u32(np_tim, "lpddr4_drv",
				    &tim->lpddr4_drv);
	ret |= of_property_read_u32(np_tim, "lpddr4_dq_odt",
				    &tim->lpddr4_dq_odt);
	ret |= of_property_read_u32(np_tim, "lpddr4_ca_odt",
				    &tim->lpddr4_ca_odt);
	ret |= of_property_read_u32(np_tim, "phy_lpddr4_ca_drv",
				    &tim->phy_lpddr4_ca_drv);
	ret |= of_property_read_u32(np_tim, "phy_lpddr4_ck_cs_drv",
				    &tim->phy_lpddr4_ck_cs_drv);
	ret |= of_property_read_u32(np_tim, "phy_lpddr4_dq_drv",
				    &tim->phy_lpddr4_dq_drv);
	ret |= of_property_read_u32(np_tim, "phy_lpddr4_odt",
				    &tim->phy_lpddr4_odt);
	ret |= of_property_read_u32(np_tim, "ddr4_odt_dis_freq",
				    &tim->ddr4_odt_dis_freq);
	ret |= of_property_read_u32(np_tim, "ddr4_drv",
				    &tim->ddr4_drv);
	ret |= of_property_read_u32(np_tim, "ddr4_odt",
				    &tim->ddr4_odt);
	ret |= of_property_read_u32(np_tim, "lpddr4_ca_odt",
				    &tim->lpddr4_ca_odt);
	ret |= of_property_read_u32(np_tim, "phy_ddr4_ca_drv",
				    &tim->phy_ddr4_ca_drv);
	ret |= of_property_read_u32(np_tim, "phy_ddr4_ck_drv",
				    &tim->phy_ddr4_ck_drv);
	ret |= of_property_read_u32(np_tim, "phy_ddr4_dq_drv",
				    &tim->phy_ddr4_dq_drv);
	ret |= of_property_read_u32(np_tim, "phy_ddr4_odt",
				    &tim->phy_ddr4_odt);

	ret |= of_get_ca_deskew(np_tim, de_skew);
	ret |= of_get_cs0_deskew(np_tim, de_skew);
	ret |= of_get_cs1_deskew(np_tim, de_skew);

	if (!ret)
		de_skew_setting_2_register(de_skew, tim);
	of_node_put(np_tim);
	kfree(de_skew);
end:
	return ret;
}

static int _ddr_recalc_rate(void)
{
	struct arm_smccc_res res;

	if (!ddr_data->enable)
		return 0;

	pr_debug("In func %s\n", __func__);
	res = rockchip_psci_smc_read(PSCI_SIP_DRAM_FREQ_CONFIG, 0, 0,
				     DRAM_FREQ_CONFIG_DRAM_GET_RATE);
	if (res.a0)
		return 0;
	else
		return res.a1;
}

static int _ddr_change_freq(u32 hz)
{
	u32 ret;
	struct arm_smccc_res res;
	struct set_rate_params *p =
		(struct set_rate_params *)ddr_data->share_memory;
	unsigned int cpu = raw_smp_processor_id();

	if (!ddr_data->enable)
		return 0;

	p->hz = hz;
	p->wait_flag1 = 1;
	p->wait_flag0 = 1;
	pr_debug("In func %s,freq=%dMHz\n", __func__, hz / 1000000);
	pr_err("current cpu %x\n", cpu);
	pr_err("vbank = %dus\n", rk_fb_get_prmry_screen_vbt());
	/* lock cpu to avoid cpu on/off */
	cpu_maps_update_begin();
	res = rockchip_psci_smc_read(PSCI_SIP_DRAM_FREQ_CONFIG,
				     SHARE_PAGE_TYPE_DDR,
				     0,
				     DRAM_FREQ_CONFIG_DRAM_FREQ_CHANGE);
	cpu_maps_update_done();
	if (!res.a1)
		pr_info("set ddr freq timeout\n");
	ret = _ddr_recalc_rate();
	pr_debug("Func %s out,freq=%dMHz\n", __func__, ret / 1000000);
	return ret;
}

static long _ddr_round_rate(u32 hz)
{
	struct arm_smccc_res res;
	struct round_rate_params *p =
		(struct round_rate_params *)ddr_data->share_memory;

	if (!ddr_data->enable)
		return 0;

	p->hz = hz;
	pr_debug("In func %s,freq=%dMHz\n", __func__, hz / 1000000);
	res = rockchip_psci_smc_read(PSCI_SIP_DRAM_FREQ_CONFIG,
				     SHARE_PAGE_TYPE_DDR,
				     0,
				     DRAM_FREQ_CONFIG_DRAM_ROUND_RATE);
	if (res.a0)
		return 0;
	else
		return res.a1;
}

static void _ddr_set_auto_self_refresh(bool en)
{
	struct set_at_sr_params *p =
		(struct set_at_sr_params *)ddr_data->share_memory;

	if (!ddr_data->enable)
		return;

	p->en = en;
	pr_debug("In func %s\n", __func__);
	rockchip_psci_smc_read(PSCI_SIP_DRAM_FREQ_CONFIG,
			       SHARE_PAGE_TYPE_DDR,
			       0,
			       DRAM_FREQ_CONFIG_DRAM_SET_AT_SR);
}

static int ddr_monitor_init(struct platform_device *pdev,
			    struct rockchip_ddr *ddr_data)
{
	struct device_node *np;
	u32 dram_type;

	np = pdev->dev.of_node;
	/* ddrpctl */
	ddr_data->ddrpctl_regs =
	    syscon_regmap_lookup_by_phandle(np, "rockchip,ddrpctl");
	if (IS_ERR(ddr_data->ddrpctl_regs)) {
		dev_err(&pdev->dev, "%s: could not find ddrpctl dt node\n",
			__func__);
		return -ENXIO;
	}

	/* msch */
	ddr_data->msch_regs =
	    syscon_regmap_lookup_by_phandle(np, "rockchip,msch");
	if (IS_ERR(ddr_data->msch_regs)) {
		dev_err(&pdev->dev, "%s: could not find msch dt node\n",
			__func__);
		return -ENXIO;
	}

	/* ddr monitor */
	ddr_data->ddrmonitor_regs =
	    syscon_regmap_lookup_by_phandle(np, "rockchip,ddrmonitor");
	if (IS_ERR(ddr_data->ddrmonitor_regs)) {
		dev_err(&pdev->dev, "%s: could not find msch dt node\n",
			__func__);
		return -ENXIO;
	}

	regmap_read(ddr_data->ddrpctl_regs,
		    DDR_PCTL2_MSTR,
		    &dram_type);
	if (dram_type & PCTL2_DDR3)
		regmap_write(ddr_data->ddrmonitor_regs, DDRMON_CTRL,
			     MON_DDR3_EN | MON_HARDWARE_DIS);
	else if (dram_type & PCTL2_DDR4)
		regmap_write(ddr_data->ddrmonitor_regs, DDRMON_CTRL,
			     MON_DDR4_EN | MON_HARDWARE_DIS);
	else
		regmap_write(ddr_data->ddrmonitor_regs, DDRMON_CTRL,
			     MON_LPDDR2_3_EN | MON_HARDWARE_DIS);

	return 0;
}

static void ddr_monitor_start(void)
{
	struct probe *p;

	for_each_probe(p) {
		regmap_write(ddr_data->msch_regs, p->offset + 0x00c, 0x0);
		regmap_write(ddr_data->msch_regs, p->offset + 0x008, 0x8);
		regmap_write(ddr_data->msch_regs, p->offset + 0x138, 0x6);
		regmap_write(ddr_data->msch_regs, p->offset + 0x14c, 0x10);
		regmap_write(ddr_data->msch_regs, p->offset + 0x160, 0x8);
		regmap_write(ddr_data->msch_regs, p->offset + 0x174, 0x10);
		regmap_write(ddr_data->msch_regs, p->offset + 0x00c, 0x1);
	}
	/* dfi eff start */
	regmap_update_bits(ddr_data->ddrmonitor_regs, DDRMON_CTRL,
			   MON_SOFTWARE_EN_MASK,
			   MON_SOFTWARE_EN);

	/*flash data */
	wmb();
	/* trigger statistic */
	for_each_probe(p)
		regmap_write(ddr_data->msch_regs, p->offset + 0x28, 0x1);
}

static void ddr_monitor_stop(void)
{
	/* dfi eff stop */
	regmap_update_bits(ddr_data->ddrmonitor_regs, DDRMON_CTRL,
			   MON_SOFTWARE_EN_MASK,
			   MON_SOFTWARE_DIS);
}

static void _ddr_bandwidth_get(struct ddr_bw_info *ddr_bw_ch0,
			       struct ddr_bw_info *ddr_bw_ch1)
{
	u32 monitor_total, monitor_rd, monitor_wr;
	u32 monitor_act, monitor_time;
	u64 time;
	u64 ddr_freq_hz = 0;
	/* byte */
	u64 bw;
	u64 tmp64;
	u32 tmp32;
	struct probe *p;

	if (!ddr_data)
		return;

	ddr_monitor_stop();
	regmap_read(ddr_data->ddrmonitor_regs,
		    DDRMON_CH0_DFI_ACT_NUM,
		    &monitor_act);
	regmap_read(ddr_data->ddrmonitor_regs,
		    DDRMON_CH0_DFI_WR_NUM,
		    &monitor_wr);
	regmap_read(ddr_data->ddrmonitor_regs,
		    DDRMON_CH0_DFI_RD_NUM,
		    &monitor_rd);
	regmap_read(ddr_data->ddrmonitor_regs,
		    DDRMON_CH0_COUNT_NUM,
		    &monitor_time);
	regmap_read(ddr_data->ddrmonitor_regs,
		    DDRMON_CH0_DFI_ACCESS_NUM,
		    &monitor_total);

	for_each_probe(p) {
		regmap_read(ddr_data->msch_regs,
			    p->offset + 0x178,
			    &tmp32);
		regmap_read(ddr_data->msch_regs,
			    p->offset + 0x164,
			    (u32 *)&p->bytes);
		p->bytes += (tmp32 << 16);
	}

	regmap_read(ddr_data->ddrpctl_regs,
		    DDR_PCTL2_MSTR,
		    &tmp32);
	if ((tmp32 & PCTL2_DATA_BUS_WIDTH_MASK) ==
		PCTL2_DATA_BUS_32BIT)
		bw = 4;
	else if ((tmp32 & PCTL2_DATA_BUS_WIDTH_MASK) ==
		PCTL2_DATA_BUS_16BIT)
		bw = 2;
	else
		bw = 1;
	ddr_freq_hz = _ddr_recalc_rate();
	if (!ddr_freq_hz)
		return;
	/* us */
	tmp64 = (u64)monitor_time * (u64)1000000;
	do_div(tmp64, ddr_freq_hz);
	time = tmp64;

	if (ddr_bw_ch0) {
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
		tmp64 = (u64)monitor_wr * (u64)8 * bw;
		do_div(tmp64, time);
		ddr_bw_ch0->ddr_wr = tmp64;
		tmp64 = (u64)monitor_rd * (u64)8 * bw;
		do_div(tmp64, time);
		ddr_bw_ch0->ddr_rd = tmp64;
		ddr_bw_ch0->ddr_act = monitor_act;
		tmp64 = ddr_freq_hz * (u64)2 * bw;
		do_div(tmp64, 1024 * 1024);
		ddr_bw_ch0->ddr_total = tmp64;

		for_each_probe(p)
			do_div(p->bytes, time);
		/* noc unit:bype */
		ddr_bw_ch0->cpum = probes[0].bytes;
		ddr_bw_ch0->gpu = probes[1].bytes;
		ddr_bw_ch0->peri = probes[2].bytes;
		ddr_bw_ch0->vio0 = probes[3].bytes;
		ddr_bw_ch0->vio1 = probes[4].bytes;
		ddr_bw_ch0->video = probes[5].bytes;
		ddr_bw_ch0->vio2 = 0;
	}
	ddr_monitor_start();
}

static void ddr_init(u32 page_type)
{
	struct arm_smccc_res res;

	pr_debug("In Func:%s\n", __func__);
	res = rockchip_psci_smc_read(PSCI_SIP_DRAM_FREQ_CONFIG, page_type,
				     0,
				     DRAM_FREQ_CONFIG_DRAM_INIT);
	if (res.a0)
		pr_info("ddr init error\n");
	else
		pr_debug("%s out\n", __func__);
}

static int __init rockchip_atf_ver_check(void)
{
	struct arm_smccc_res res;

	res = rockchip_psci_smc_read(PSCI_SIP_DRAM_FREQ_CONFIG, 0, 0,
				     DRAM_FREQ_CONFIG_DRAM_GET_VERSION);
	pr_info("current ATF version 0x%lx!\n", res.a1);
	if ((!res.a0) && (res.a1 >= 0x100))
		return 0;

	pr_err("read tf version 0x%lx!\n", res.a1);

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

	if (rockchip_atf_ver_check() == 0) {
		ddr_data->enable = 1;
	} else {
		ddr_data->enable = 0;
		return -ENXIO;
	}

	/*
	 * first 4KB is used for interface parameters
	 * after 4KB * N is dts parameters
	 */
	res = rockchip_request_share_memory(
					    SHARE_PAGE_TYPE_DDR,
			DIV_ROUND_UP(sizeof(struct ddr_dts_config_timing),
				     4096) + 1);
	if (res.a0 != 0) {
		dev_err(&pdev->dev, "no ATF memory for init\n");
		return -ENOMEM;
	}
	dev_err(&pdev->dev, "share memory addr=0x%lx\n", res.a1);
	ddr_data->share_memory = ioremap(res.a1,
		(DIV_ROUND_UP(sizeof(struct ddr_dts_config_timing),
			      4096) + 1) * 4096);
	if (!ddr_data->share_memory) {
		dev_err(&pdev->dev, "ioremap share memory fail\n");
		return -ENOMEM;
	}

	ddr_data->dram_timing =
		(struct ddr_dts_config_timing *)(ddr_data->share_memory +
					4096);
	if (!of_get_ddr_timings(np, ddr_data->dram_timing)) {
		ddr_data->dram_timing->available = 1;
	} else {
		pr_info("of_get_ddr_timings: fail\n");
		ddr_data->dram_timing->available = 0;
	}

	platform_set_drvdata(pdev, ddr_data);

	ddr_monitor_init(pdev, ddr_data);
	ddr_init(SHARE_PAGE_TYPE_DDR);
	ddr_change_freq = _ddr_change_freq;
	ddr_round_rate = _ddr_round_rate;
	ddr_set_auto_self_refresh = _ddr_set_auto_self_refresh;
	ddr_bandwidth_get = _ddr_bandwidth_get;
	ddr_recalc_rate = _ddr_recalc_rate;
	pr_info("%s: success\n", __func__);
	return 0;
}

static const struct of_device_id rockchip_ddr_of_match[] __refdata = {
	{ .compatible = "rockchip,rk322xh-ddr", .data = NULL, },
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
