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
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/workqueue.h>

#include <soc/tegra/fuse.h>
#include <soc/tegra/xusb.h>

#include <dt-bindings/pinctrl/pinctrl-tegra-xusb.h>

#include "core.h"
#include "pinctrl-utils.h"

#define FUSE_SKU_CALIB_HS_CURR_LEVEL_PADX_SHIFT(x) ((x) ? 15 : 0)
#define FUSE_SKU_CALIB_HS_CURR_LEVEL_PAD_MASK 0x3f
#define FUSE_SKU_CALIB_HS_IREF_CAP_SHIFT 13
#define FUSE_SKU_CALIB_HS_IREF_CAP_MASK 0x3
#define FUSE_SKU_CALIB_HS_SQUELCH_LEVEL_SHIFT 11
#define FUSE_SKU_CALIB_HS_SQUELCH_LEVEL_MASK 0x3
#define FUSE_SKU_CALIB_HS_TERM_RANGE_ADJ_SHIFT 7
#define FUSE_SKU_CALIB_HS_TERM_RANGE_ADJ_MASK 0xf

#define XUSB_PADCTL_USB2_PORT_CAP 0x008
#define XUSB_PADCTL_USB2_PORT_CAP_PORTX_CAP_SHIFT(x) ((x) * 4)
#define XUSB_PADCTL_USB2_PORT_CAP_PORT_CAP_MASK 0x3
#define XUSB_PADCTL_USB2_PORT_CAP_DISABLED 0x0
#define XUSB_PADCTL_USB2_PORT_CAP_HOST 0x1
#define XUSB_PADCTL_USB2_PORT_CAP_DEVICE 0x2
#define XUSB_PADCTL_USB2_PORT_CAP_OTG 0x3

#define XUSB_PADCTL_SS_PORT_MAP 0x014
#define XUSB_PADCTL_SS_PORT_MAP_PORTX_SHIFT(x) ((x) * 4)
#define XUSB_PADCTL_SS_PORT_MAP_PORT_MASK 0x7

#define XUSB_PADCTL_ELPG_PROGRAM 0x01c
#define XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_VCORE_DOWN (1 << 26)
#define XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN_EARLY (1 << 25)
#define XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN (1 << 24)
#define XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_VCORE_DOWN(x) (1 << (18 + (x) * 4))
#define XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_CLAMP_EN_EARLY(x) \
							(1 << (17 + (x) * 4))
#define XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_CLAMP_EN(x) (1 << (16 + (x) * 4))

#define XUSB_PADCTL_IOPHY_PLL_P0_CTL1 0x040
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL0_LOCKDET (1 << 19)
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL1_REFCLK_SEL_MASK (0xf << 12)
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL_RST (1 << 1)

#define XUSB_PADCTL_IOPHY_PLL_P0_CTL2 0x044
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL2_REFCLKBUF_EN (1 << 6)
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL2_TXCLKREF_EN (1 << 5)
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL2_TXCLKREF_SEL (1 << 4)

#define XUSB_PADCTL_IOPHY_USB3_PADX_CTL2(x) (0x058 + (x) * 4)
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_CDR_CNTL_SHIFT 24
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_CDR_CNTL_MASK 0xff
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_SHIFT 16
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_MASK 0x3f
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_SHIFT 8
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_MASK 0x3f
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_SHIFT 8
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_MASK 0xffff
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_WANDER_SHIFT 4
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_WANDER_MASK 0x7

#define XUSB_PADCTL_IOPHY_USB3_PADX_CTL4(x) (0x068 + (x) * 4)
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_SHIFT 24
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_MASK 0x1f
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_SHIFT 16
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_MASK 0x7f

#define XUSB_PADCTL_IOPHY_MISC_PAD_PX_CTL2(x) ((x) < 2 ? 0x078 + (x) * 4 : \
					       0x0f8 + (x) * 4)
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL2_SPARE_IN_SHIFT 28
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL2_SPARE_IN_MASK 0x3

#define XUSB_PADCTL_IOPHY_MISC_PAD_PX_CTL5(x) ((x) < 2 ? 0x090 + (x) * 4 : \
					       0x11c + (x) * 4)
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL5_RX_QEYE_EN (1 << 8)

#define XUSB_PADCTL_IOPHY_MISC_PAD_PX_CTL6(x) ((x) < 2 ? 0x098 + (x) * 4 : \
					       0x128 + (x) * 4)
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SHIFT 24
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_G_Z_MASK 0x3f
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_TAP_MASK 0x1f
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_AMP_MASK 0x7f
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT 16
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_MASK 0xff
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_G_Z 0x21
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_TAP 0x32
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_AMP 0x33
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_CTLE_Z 0x48
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_LATCH_G_Z 0xa1

#define XUSB_PADCTL_USB2_OTG_PADX_CTL0(x) (0x0a0 + (x) * 4)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD_ZI (1 << 21)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD2 (1 << 20)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD (1 << 19)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_LS_RSLEW_SHIFT 14
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_LS_RSLEW_MASK 0x3
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_SLEW_SHIFT 6
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_SLEW_MASK 0x3f
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_SHIFT 0
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_MASK 0x3f

#define XUSB_PADCTL_USB2_OTG_PADX_CTL1(x) (0x0ac + (x) * 4)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_HS_IREF_CAP_SHIFT 9
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_HS_IREF_CAP_MASK 0x3
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_SHIFT 3
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_MASK 0x7
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_DR (1 << 2)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_DISC_FORCE_POWERUP (1 << 1)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_CHRP_FORCE_POWERUP (1 << 0)

#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0 0x0b8
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_PD (1 << 12)
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_SHIFT 2
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_MASK 0x7
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_SHIFT 0
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_MASK 0x3

#define XUSB_PADCTL_HSIC_PADX_CTL0(x) (0x0c0 + (x) * 4)
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWN_SHIFT 12
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWN_MASK 0x7
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWP_SHIFT 8
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWP_MASK 0x7
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEN_SHIFT 4
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEN_MASK 0x7
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEP_SHIFT 0
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEP_MASK 0x7

#define XUSB_PADCTL_HSIC_PADX_CTL1(x) (0x0c8 + (x) * 4)
#define XUSB_PADCTL_HSIC_PAD_CTL1_RPU_STROBE (1 << 10)
#define XUSB_PADCTL_HSIC_PAD_CTL1_RPU_DATA (1 << 9)
#define XUSB_PADCTL_HSIC_PAD_CTL1_RPD_STROBE (1 << 8)
#define XUSB_PADCTL_HSIC_PAD_CTL1_RPD_DATA (1 << 7)
#define XUSB_PADCTL_HSIC_PAD_CTL1_PD_ZI (1 << 5)
#define XUSB_PADCTL_HSIC_PAD_CTL1_PD_RX (1 << 4)
#define XUSB_PADCTL_HSIC_PAD_CTL1_PD_TRX (1 << 3)
#define XUSB_PADCTL_HSIC_PAD_CTL1_PD_TX (1 << 2)
#define XUSB_PADCTL_HSIC_PAD_CTL1_AUTO_TERM_EN (1 << 0)

#define XUSB_PADCTL_HSIC_PADX_CTL2(x) (0x0d0 + (x) * 4)
#define XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_SHIFT 4
#define XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_MASK 0x7
#define XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_SHIFT 0
#define XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_MASK 0x7

#define XUSB_PADCTL_HSIC_STRB_TRIM_CONTROL 0x0e0
#define XUSB_PADCTL_HSIC_STRB_TRIM_CONTROL_STRB_TRIM_MASK 0x1f

#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1 0x138
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL1_LOCKDET (1 << 27)
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL1_MODE (1 << 24)
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL0_REFCLK_NDIV_SHIFT 20
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL0_REFCLK_NDIV_MASK 0x3
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_PWR_OVRD (1 << 3)
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_RST (1 << 1)
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_IDDQ (1 << 0)

#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2 0x13c
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL1_CP_CNTL_SHIFT 20
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL1_CP_CNTL_MASK 0xf
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL0_CP_CNTL_SHIFT 16
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL0_CP_CNTL_MASK 0xf
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_TCLKOUT_EN (1 << 12)
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_TXCLKREF_SEL (1 << 4)
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_XDIGCLK_SEL_SHIFT 0
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_XDIGCLK_SEL_MASK 0x7

#define XUSB_PADCTL_IOPHY_PLL_S0_CTL3 0x140
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL3_RCAL_BYPASS (1 << 7)

#define XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1 0x148
#define XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1_IDDQ_OVRD (1 << 1)
#define XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1_IDDQ (1 << 0)

#define XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL2 0x14c

#define XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL5 0x158

#define XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL6 0x15c

struct tegra_xusb_padctl_function {
	const char *name;
	const char * const *groups;
	unsigned int num_groups;
};

struct tegra_xusb_padctl_group {
	const unsigned int *funcs;
	unsigned int num_funcs;
};

struct tegra_xusb_padctl_soc {
	const struct pinctrl_pin_desc *pins;
	unsigned int num_pins;

	const struct tegra_xusb_padctl_function *functions;
	unsigned int num_functions;

	const struct tegra_xusb_padctl_lane *lanes;
	unsigned int num_lanes;

	u32 rx_wander;
	u32 rx_eq;
	u32 cdr_cntl;
	u32 dfe_cntl;
	u32 hs_slew;
	u32 ls_rslew[TEGRA_XUSB_UTMI_PHYS];
	u32 hs_discon_level;
	u32 spare_in;
	int hsic_port_offset;
};

struct tegra_xusb_padctl_lane {
	const char *name;

	unsigned int offset;
	unsigned int shift;
	unsigned int mask;
	unsigned int iddq;

	const unsigned int *funcs;
	unsigned int num_funcs;
};

struct tegra_xusb_fuse_calibration {
	u32 hs_curr_level[TEGRA_XUSB_UTMI_PHYS];
	u32 hs_iref_cap;
	u32 hs_term_range_adj;
	u32 hs_squelch_level;
};

struct tegra_xusb_usb3_port {
	int lane;
	bool context_saved;
	u32 tap1_val;
	u32 amp_val;
	u32 ctle_z_val;
	u32 ctle_g_val;
};

struct tegra_xusb_padctl {
	struct device *dev;
	void __iomem *regs;
	struct mutex lock;
	struct reset_control *rst;

	const struct tegra_xusb_padctl_soc *soc;
	struct tegra_xusb_fuse_calibration calib;
	struct pinctrl_dev *pinctrl;
	struct pinctrl_desc desc;

	struct phy_provider *provider;
	struct phy *phys[TEGRA_XUSB_NUM_PHYS];

	unsigned int enable;

	struct work_struct mbox_req_work;
	struct tegra_xusb_mbox_msg mbox_req;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;

	struct tegra_xusb_usb3_port usb3_ports[TEGRA_XUSB_USB3_PHYS];
	unsigned int utmi_enable;
	unsigned int hs_curr_level_offset[TEGRA_XUSB_UTMI_PHYS];
	struct regulator *vbus[TEGRA_XUSB_UTMI_PHYS];
	struct regulator *vddio_hsic;
};

static inline void padctl_writel(struct tegra_xusb_padctl *padctl, u32 value,
				 unsigned long offset)
{
	writel(value, padctl->regs + offset);
}

static inline u32 padctl_readl(struct tegra_xusb_padctl *padctl,
			       unsigned long offset)
{
	return readl(padctl->regs + offset);
}

static inline bool is_otg_lane(unsigned int lane)
{
	return lane >= TEGRA_XUSB_PADCTL_PIN_OTG_0 &&
		lane <= TEGRA_XUSB_PADCTL_PIN_OTG_2;
}

static inline bool is_hsic_lane(unsigned int lane)
{
	return lane >= TEGRA_XUSB_PADCTL_PIN_HSIC_0 &&
		lane <= TEGRA_XUSB_PADCTL_PIN_HSIC_1;
}

static inline bool is_pcie_or_sata_lane(unsigned int lane)
{
	return lane >= TEGRA_XUSB_PADCTL_PIN_PCIE_0 &&
		lane <= TEGRA_XUSB_PADCTL_PIN_SATA_0;
}

static int lane_to_usb3_port(struct tegra_xusb_padctl *padctl,
			     unsigned int lane)
{
	int i;

	for (i = 0; i < TEGRA_XUSB_USB3_PHYS; i++) {
		if (padctl->usb3_ports[i].lane == lane)
			return i;
	}

	return -1;
}

static int tegra_xusb_padctl_get_groups_count(struct pinctrl_dev *pinctrl)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);

	return padctl->soc->num_pins;
}

static const char *tegra_xusb_padctl_get_group_name(struct pinctrl_dev *pinctrl,
						    unsigned int group)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);

	return padctl->soc->pins[group].name;
}

enum tegra_xusb_padctl_param {
	TEGRA_XUSB_PADCTL_IDDQ,
	TEGRA_XUSB_PADCTL_USB2_PORT_NUM,
	TEGRA_XUSB_PADCTL_HSIC_STROBE_TRIM,
	TEGRA_XUSB_PADCTL_HSIC_RX_STROBE_TRIM,
	TEGRA_XUSB_PADCTL_HSIC_RX_DATA_TRIM,
	TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEN,
	TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEP,
	TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWN,
	TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWP,
	TEGRA_XUSB_PADCTL_HSIC_AUTO_TERM,
	TEGRA_XUSB_PADCTL_OTG_HS_CURR_LEVEL_OFFSET,
};

static const struct tegra_xusb_padctl_property {
	const char *name;
	enum tegra_xusb_padctl_param param;
} properties[] = {
	{ "nvidia,iddq", TEGRA_XUSB_PADCTL_IDDQ },
	{ "nvidia,usb2-port-num", TEGRA_XUSB_PADCTL_USB2_PORT_NUM },
	{ "nvidia,hsic-strobe-trim", TEGRA_XUSB_PADCTL_HSIC_STROBE_TRIM },
	{ "nvidia,hsic-rx-strobe-trim", TEGRA_XUSB_PADCTL_HSIC_RX_STROBE_TRIM },
	{ "nvidia,hsic-rx-data-trim", TEGRA_XUSB_PADCTL_HSIC_RX_DATA_TRIM },
	{ "nvidia,hsic-tx-rtune-n", TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEN },
	{ "nvidia,hsic-tx-rtune-p", TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEP },
	{ "nvidia,hsic-tx-rslew-n", TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWN },
	{ "nvidia,hsic-tx-rslew-p", TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWP },
	{ "nvidia,hsic-auto-term", TEGRA_XUSB_PADCTL_HSIC_AUTO_TERM },
	{ "nvidia,otg-hs-curr-level-offset",
	  TEGRA_XUSB_PADCTL_OTG_HS_CURR_LEVEL_OFFSET },
};

#define TEGRA_XUSB_PADCTL_PACK(param, value) ((param) << 16 | (value))
#define TEGRA_XUSB_PADCTL_UNPACK_PARAM(config) ((config) >> 16)
#define TEGRA_XUSB_PADCTL_UNPACK_VALUE(config) ((config) & 0xffff)

static int tegra_xusb_padctl_parse_subnode(struct tegra_xusb_padctl *padctl,
					   struct device_node *np,
					   struct pinctrl_map **maps,
					   unsigned int *reserved_maps,
					   unsigned int *num_maps)
{
	unsigned int i, reserve = 0, num_configs = 0;
	unsigned long config, *configs = NULL;
	const char *function, *group;
	struct property *prop;
	int err = 0;
	u32 value;

	err = of_property_read_string(np, "nvidia,function", &function);
	if (err < 0) {
		if (err != -EINVAL)
			return err;

		function = NULL;
	}

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		err = of_property_read_u32(np, properties[i].name, &value);
		if (err < 0) {
			if (err == -EINVAL)
				continue;

			return err;
		}

		config = TEGRA_XUSB_PADCTL_PACK(properties[i].param, value);

		err = pinctrl_utils_add_config(padctl->pinctrl, &configs,
					       &num_configs, config);
		if (err < 0)
			return err;
	}

	if (function)
		reserve++;

	if (num_configs)
		reserve++;

	err = of_property_count_strings(np, "nvidia,lanes");
	if (err < 0)
		return err;

	reserve *= err;

	err = pinctrl_utils_reserve_map(padctl->pinctrl, maps, reserved_maps,
					num_maps, reserve);
	if (err < 0)
		return err;

	of_property_for_each_string(np, "nvidia,lanes", prop, group) {
		if (function) {
			err = pinctrl_utils_add_map_mux(padctl->pinctrl, maps,
					reserved_maps, num_maps, group,
					function);
			if (err < 0)
				return err;
		}

		if (num_configs) {
			err = pinctrl_utils_add_map_configs(padctl->pinctrl,
					maps, reserved_maps, num_maps, group,
					configs, num_configs,
					PIN_MAP_TYPE_CONFIGS_GROUP);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

static int tegra_xusb_padctl_dt_node_to_map(struct pinctrl_dev *pinctrl,
					    struct device_node *parent,
					    struct pinctrl_map **maps,
					    unsigned int *num_maps)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);
	unsigned int reserved_maps = 0;
	struct device_node *np;
	int err;

	*num_maps = 0;
	*maps = NULL;

	for_each_child_of_node(parent, np) {
		err = tegra_xusb_padctl_parse_subnode(padctl, np, maps,
						      &reserved_maps,
						      num_maps);
		if (err < 0)
			return err;
	}

	return 0;
}

static const struct pinctrl_ops tegra_xusb_padctl_pinctrl_ops = {
	.get_groups_count = tegra_xusb_padctl_get_groups_count,
	.get_group_name = tegra_xusb_padctl_get_group_name,
	.dt_node_to_map = tegra_xusb_padctl_dt_node_to_map,
	.dt_free_map = pinctrl_utils_dt_free_map,
};

static int tegra_xusb_padctl_get_functions_count(struct pinctrl_dev *pinctrl)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);

	return padctl->soc->num_functions;
}

static const char *
tegra_xusb_padctl_get_function_name(struct pinctrl_dev *pinctrl,
				    unsigned int function)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);

	return padctl->soc->functions[function].name;
}

static int tegra_xusb_padctl_get_function_groups(struct pinctrl_dev *pinctrl,
						 unsigned int function,
						 const char * const **groups,
						 unsigned * const num_groups)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);

	*num_groups = padctl->soc->functions[function].num_groups;
	*groups = padctl->soc->functions[function].groups;

	return 0;
}

static int tegra_xusb_padctl_pinmux_enable(struct pinctrl_dev *pinctrl,
					   unsigned int function,
					   unsigned int group)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);
	const struct tegra_xusb_padctl_lane *lane;
	unsigned int i;
	u32 value;

	lane = &padctl->soc->lanes[group];

	for (i = 0; i < lane->num_funcs; i++)
		if (lane->funcs[i] == function)
			break;

	if (i >= lane->num_funcs)
		return -EINVAL;

	value = padctl_readl(padctl, lane->offset);
	value &= ~(lane->mask << lane->shift);
	value |= i << lane->shift;
	padctl_writel(padctl, value, lane->offset);

	return 0;
}

static const struct pinmux_ops tegra_xusb_padctl_pinmux_ops = {
	.get_functions_count = tegra_xusb_padctl_get_functions_count,
	.get_function_name = tegra_xusb_padctl_get_function_name,
	.get_function_groups = tegra_xusb_padctl_get_function_groups,
	.enable = tegra_xusb_padctl_pinmux_enable,
};

static int tegra_xusb_padctl_pinconf_group_get(struct pinctrl_dev *pinctrl,
					       unsigned int group,
					       unsigned long *config)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);
	const struct tegra_xusb_padctl_lane *lane;
	enum tegra_xusb_padctl_param param;
	int port;
	u32 value;

	param = TEGRA_XUSB_PADCTL_UNPACK_PARAM(*config);
	lane = &padctl->soc->lanes[group];

	switch (param) {
	case TEGRA_XUSB_PADCTL_IDDQ:
		/* lanes with iddq == 0 don't support this parameter */
		if (lane->iddq == 0)
			return -EINVAL;

		value = padctl_readl(padctl, lane->offset);

		if (value & BIT(lane->iddq))
			value = 0;
		else
			value = 1;
		break;

	case TEGRA_XUSB_PADCTL_USB2_PORT_NUM:
		port = lane_to_usb3_port(padctl, group);
		if (port < 0) {
			dev_err(padctl->dev,
				"Pin %d not mapped to USB3 port\n", group);
			return -EINVAL;
		}

		value = padctl_readl(padctl, XUSB_PADCTL_SS_PORT_MAP) >>
			XUSB_PADCTL_SS_PORT_MAP_PORTX_SHIFT(port);
		value &= XUSB_PADCTL_SS_PORT_MAP_PORT_MASK;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_STROBE_TRIM:
		if (!is_hsic_lane(group)) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		value = padctl_readl(padctl,
				     XUSB_PADCTL_HSIC_STRB_TRIM_CONTROL);
		value &= XUSB_PADCTL_HSIC_STRB_TRIM_CONTROL_STRB_TRIM_MASK;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_RX_STROBE_TRIM:
		if (!is_hsic_lane(group)) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
		value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL2(port)) >>
			XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_SHIFT;
		value &= XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_MASK;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_RX_DATA_TRIM:
		if (!is_hsic_lane(group)) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
		value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL2(port)) >>
			XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_SHIFT;
		value &= XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_MASK;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEN:
		if (!is_hsic_lane(group)) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
		value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL0(port)) >>
			XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEN_SHIFT;
		value &= XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEN_MASK;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEP:
		if (!is_hsic_lane(group)) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
		value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL0(port)) >>
			XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEP_SHIFT;
		value &= XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEP_MASK;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWN:
		if (!is_hsic_lane(group)) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
		value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL0(port)) >>
			XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWN_SHIFT;
		value &= XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWN_MASK;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWP:
		if (!is_hsic_lane(group)) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
		value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL0(port)) >>
			XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWP_SHIFT;
		value &= XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWP_MASK;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_AUTO_TERM:
		if (!is_hsic_lane(group)) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
		value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL1(port));
		if (value & XUSB_PADCTL_HSIC_PAD_CTL1_AUTO_TERM_EN)
			value = 1;
		else
			value = 0;
		break;

	case TEGRA_XUSB_PADCTL_OTG_HS_CURR_LEVEL_OFFSET:
		if (!is_otg_lane(group)) {
			dev_err(padctl->dev, "Pin %d is not an OTG pad\n",
				group);
			return -EINVAL;
		}

		port = group - TEGRA_XUSB_PADCTL_PIN_OTG_0;
		value = padctl->hs_curr_level_offset[port];
		break;

	default:
		dev_err(padctl->dev, "invalid configuration parameter: %04x\n",
			param);
		return -ENOTSUPP;
	}

	*config = TEGRA_XUSB_PADCTL_PACK(param, value);
	return 0;
}

static int tegra_xusb_padctl_pinconf_group_set(struct pinctrl_dev *pinctrl,
					       unsigned int group,
					       unsigned long *configs,
					       unsigned int num_configs)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);
	const struct tegra_xusb_padctl_lane *lane;
	enum tegra_xusb_padctl_param param;
	unsigned long value;
	unsigned int i;
	u32 regval;
	int port;

	lane = &padctl->soc->lanes[group];

	for (i = 0; i < num_configs; i++) {
		param = TEGRA_XUSB_PADCTL_UNPACK_PARAM(configs[i]);
		value = TEGRA_XUSB_PADCTL_UNPACK_VALUE(configs[i]);

		switch (param) {
		case TEGRA_XUSB_PADCTL_IDDQ:
			/* lanes with iddq == 0 don't support this parameter */
			if (lane->iddq == 0)
				return -EINVAL;

			regval = padctl_readl(padctl, lane->offset);

			if (value)
				regval &= ~BIT(lane->iddq);
			else
				regval |= BIT(lane->iddq);

			padctl_writel(padctl, regval, lane->offset);
			break;

		case TEGRA_XUSB_PADCTL_USB2_PORT_NUM:
			if (value >= TEGRA_XUSB_UTMI_PHYS) {
				dev_err(padctl->dev, "Invalid USB2 port: %lu\n",
					value);
				return -EINVAL;
			}
			if (!is_pcie_or_sata_lane(group)) {
				dev_err(padctl->dev,
					"USB2 port not applicable for pin %d\n",
					group);
				return -EINVAL;
			}
			port = lane_to_usb3_port(padctl, group);
			if (port < 0) {
				dev_err(padctl->dev,
					"Pin %d not mapped to USB3 port\n",
					group);
				return -EINVAL;
			}

			regval = padctl_readl(padctl, XUSB_PADCTL_SS_PORT_MAP);
			regval &= ~(XUSB_PADCTL_SS_PORT_MAP_PORT_MASK <<
				    XUSB_PADCTL_SS_PORT_MAP_PORTX_SHIFT(port));
			regval |= value <<
				XUSB_PADCTL_SS_PORT_MAP_PORTX_SHIFT(port);
			padctl_writel(padctl, regval, XUSB_PADCTL_SS_PORT_MAP);
			break;

		case TEGRA_XUSB_PADCTL_HSIC_STROBE_TRIM:
			if (!is_hsic_lane(group)) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			value &= XUSB_PADCTL_HSIC_STRB_TRIM_CONTROL_STRB_TRIM_MASK;
			padctl_writel(padctl, value,
				      XUSB_PADCTL_HSIC_STRB_TRIM_CONTROL);
			break;

		case TEGRA_XUSB_PADCTL_HSIC_RX_STROBE_TRIM:
			if (!is_hsic_lane(group)) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
			value &= XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_MASK;
			regval = padctl_readl(padctl,
					      XUSB_PADCTL_HSIC_PADX_CTL2(port));
			regval &= ~(XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_MASK <<
				    XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_SHIFT);
			regval |= value <<
				XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_SHIFT;
			padctl_writel(padctl, regval,
				      XUSB_PADCTL_HSIC_PADX_CTL2(port));
			break;

		case TEGRA_XUSB_PADCTL_HSIC_RX_DATA_TRIM:
			if (!is_hsic_lane(group)) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
			value &= XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_MASK;
			regval = padctl_readl(padctl,
					      XUSB_PADCTL_HSIC_PADX_CTL2(port));
			regval &= ~(XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_MASK <<
				    XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_SHIFT);
			regval |= value <<
				XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_SHIFT;
			padctl_writel(padctl, regval,
				      XUSB_PADCTL_HSIC_PADX_CTL2(port));
			break;

		case TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEN:
			if (!is_hsic_lane(group)) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
			value &= XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEN_MASK;
			regval = padctl_readl(padctl,
					      XUSB_PADCTL_HSIC_PADX_CTL0(port));
			regval &= ~(XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEN_MASK <<
				    XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEN_SHIFT);
			regval |= value <<
				XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEN_SHIFT;
			padctl_writel(padctl, regval,
				      XUSB_PADCTL_HSIC_PADX_CTL0(port));
			break;

		case TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEP:
			if (!is_hsic_lane(group)) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
			value &= XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEP_MASK;
			regval = padctl_readl(padctl,
					      XUSB_PADCTL_HSIC_PADX_CTL0(port));
			regval &= ~(XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEP_MASK <<
				    XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEP_SHIFT);
			regval |= value <<
				XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEP_SHIFT;
			padctl_writel(padctl, regval,
				      XUSB_PADCTL_HSIC_PADX_CTL0(port));
			break;

		case TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWN:
			if (!is_hsic_lane(group)) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
			value &= XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWN_MASK;
			regval = padctl_readl(padctl,
					      XUSB_PADCTL_HSIC_PADX_CTL0(port));
			regval &= ~(XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWN_MASK <<
				    XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWN_SHIFT);
			regval |= value <<
				XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWN_SHIFT;
			padctl_writel(padctl, regval,
				      XUSB_PADCTL_HSIC_PADX_CTL0(port));
			break;

		case TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWP:
			if (!is_hsic_lane(group)) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
			value &= XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWP_MASK;
			regval = padctl_readl(padctl,
					      XUSB_PADCTL_HSIC_PADX_CTL0(port));
			regval &= ~(XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWP_MASK <<
				    XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWP_SHIFT);
			regval |= value <<
				XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWP_SHIFT;
			padctl_writel(padctl, regval,
				      XUSB_PADCTL_HSIC_PADX_CTL0(port));
			break;

		case TEGRA_XUSB_PADCTL_HSIC_AUTO_TERM:
			if (!is_hsic_lane(group)) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			port = group - TEGRA_XUSB_PADCTL_PIN_HSIC_0;
			regval = padctl_readl(padctl,
					      XUSB_PADCTL_HSIC_PADX_CTL1(port));
			if (!value)
				regval &= ~XUSB_PADCTL_HSIC_PAD_CTL1_AUTO_TERM_EN;
			else
				regval |= XUSB_PADCTL_HSIC_PAD_CTL1_AUTO_TERM_EN;
			padctl_writel(padctl, regval,
				      XUSB_PADCTL_HSIC_PADX_CTL1(port));
			break;

		case TEGRA_XUSB_PADCTL_OTG_HS_CURR_LEVEL_OFFSET:
			if (!is_otg_lane(group)) {
				dev_err(padctl->dev,
					"Pin %d is not an OTG pad\n", group);
				return -EINVAL;
			}

			port = group - TEGRA_XUSB_PADCTL_PIN_OTG_0;
			value &= XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_MASK;
			padctl->hs_curr_level_offset[port] = value;
			break;

		default:
			dev_err(padctl->dev,
				"invalid configuration parameter: %04x\n",
				param);
			return -ENOTSUPP;
		}
	}

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static const char *strip_prefix(const char *s)
{
	const char *comma = strchr(s, ',');
	if (!comma)
		return s;

	return comma + 1;
}

static void
tegra_xusb_padctl_pinconf_group_dbg_show(struct pinctrl_dev *pinctrl,
					 struct seq_file *s,
					 unsigned int group)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		unsigned long config, value;
		int err;

		config = TEGRA_XUSB_PADCTL_PACK(properties[i].param, 0);

		err = tegra_xusb_padctl_pinconf_group_get(pinctrl, group,
							  &config);
		if (err < 0)
			continue;

		value = TEGRA_XUSB_PADCTL_UNPACK_VALUE(config);

		seq_printf(s, "\n\t%s=%lu\n", strip_prefix(properties[i].name),
			   value);
	}
}

static void
tegra_xusb_padctl_pinconf_config_dbg_show(struct pinctrl_dev *pinctrl,
					  struct seq_file *s,
					  unsigned long config)
{
	enum tegra_xusb_padctl_param param;
	const char *name = "unknown";
	unsigned long value;
	unsigned int i;

	param = TEGRA_XUSB_PADCTL_UNPACK_PARAM(config);
	value = TEGRA_XUSB_PADCTL_UNPACK_VALUE(config);

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		if (properties[i].param == param) {
			name = properties[i].name;
			break;
		}
	}

	seq_printf(s, "%s=%lu", strip_prefix(name), value);
}
#endif

static const struct pinconf_ops tegra_xusb_padctl_pinconf_ops = {
	.pin_config_group_get = tegra_xusb_padctl_pinconf_group_get,
	.pin_config_group_set = tegra_xusb_padctl_pinconf_group_set,
#ifdef CONFIG_DEBUG_FS
	.pin_config_group_dbg_show = tegra_xusb_padctl_pinconf_group_dbg_show,
	.pin_config_config_dbg_show = tegra_xusb_padctl_pinconf_config_dbg_show,
#endif
};

static int tegra_xusb_padctl_enable(struct tegra_xusb_padctl *padctl)
{
	u32 value;

	mutex_lock(&padctl->lock);

	if (padctl->enable++ > 0)
		goto out;

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN_EARLY;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_VCORE_DOWN;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

out:
	mutex_unlock(&padctl->lock);
	return 0;
}

static int tegra_xusb_padctl_disable(struct tegra_xusb_padctl *padctl)
{
	u32 value;

	mutex_lock(&padctl->lock);

	if (WARN_ON(padctl->enable == 0))
		goto out;

	if (--padctl->enable > 0)
		goto out;

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value |= XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_VCORE_DOWN;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value |= XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN_EARLY;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value |= XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

out:
	mutex_unlock(&padctl->lock);
	return 0;
}

static int tegra_xusb_phy_init(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);

	return tegra_xusb_padctl_enable(padctl);
}

static int tegra_xusb_phy_exit(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);

	return tegra_xusb_padctl_disable(padctl);
}

static int pcie_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	unsigned long timeout;
	int err = -ETIMEDOUT;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);
	value &= ~XUSB_PADCTL_IOPHY_PLL_P0_CTL1_REFCLK_SEL_MASK;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_P0_CTL2);
	value |= XUSB_PADCTL_IOPHY_PLL_P0_CTL2_REFCLKBUF_EN |
		 XUSB_PADCTL_IOPHY_PLL_P0_CTL2_TXCLKREF_EN |
		 XUSB_PADCTL_IOPHY_PLL_P0_CTL2_TXCLKREF_SEL;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_P0_CTL2);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);
	value |= XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL_RST;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);

	timeout = jiffies + msecs_to_jiffies(50);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);
		if (value & XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL0_LOCKDET) {
			err = 0;
			break;
		}

		usleep_range(100, 200);
	}

	return err;
}

static int pcie_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);
	value &= ~XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL_RST;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);

	return 0;
}

static const struct phy_ops pcie_phy_ops = {
	.init = tegra_xusb_phy_init,
	.exit = tegra_xusb_phy_exit,
	.power_on = pcie_phy_power_on,
	.power_off = pcie_phy_power_off,
	.owner = THIS_MODULE,
};

static int sata_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	unsigned long timeout;
	int err = -ETIMEDOUT;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1);
	value &= ~XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1_IDDQ_OVRD;
	value &= ~XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1_IDDQ;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
	value &= ~XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_PWR_OVRD;
	value &= ~XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_IDDQ;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
	value |= XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL1_MODE;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
	value |= XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_RST;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

	timeout = jiffies + msecs_to_jiffies(50);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
		if (value & XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL1_LOCKDET) {
			err = 0;
			break;
		}

		usleep_range(100, 200);
	}

	return err;
}

static int sata_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
	value &= ~XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_RST;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
	value &= ~XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL1_MODE;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
	value |= XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_PWR_OVRD;
	value |= XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_IDDQ;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1);
	value |= ~XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1_IDDQ_OVRD;
	value |= ~XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1_IDDQ;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1);

	return 0;
}

static const struct phy_ops sata_phy_ops = {
	.init = tegra_xusb_phy_init,
	.exit = tegra_xusb_phy_exit,
	.power_on = sata_phy_power_on,
	.power_off = sata_phy_power_off,
	.owner = THIS_MODULE,
};

static int usb3_phy_to_port(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	int i;

	for (i = 0; i < TEGRA_XUSB_USB3_PHYS; i++) {
		if (phy == padctl->phys[TEGRA_XUSB_PADCTL_USB3_P0 + i])
			break;
	}
	BUG_ON(i == TEGRA_XUSB_USB3_PHYS);

	return i;
}

static int usb3_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	int port = usb3_phy_to_port(phy);
	int lane = padctl->usb3_ports[port].lane;
	u32 value, offset;

	if (!is_pcie_or_sata_lane(lane)) {
		dev_err(padctl->dev, "USB3 PHY %d mapped to invalid lane: %d\n",
			port, lane);
		return -EINVAL;
	}

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_USB3_PADX_CTL2(port));
	value &= ~((XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_WANDER_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_WANDER_SHIFT) |
		   (XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_SHIFT) |
		   (XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_CDR_CNTL_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_CDR_CNTL_SHIFT));
	value |= (padctl->soc->rx_wander <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_WANDER_SHIFT) |
		 (padctl->soc->cdr_cntl <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_CDR_CNTL_SHIFT) |
		 (padctl->soc->rx_eq <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_SHIFT);
	if (padctl->usb3_ports[port].context_saved) {
		value &= ~((XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_MASK <<
			    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_SHIFT) |
			   (XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_MASK <<
			    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_SHIFT));
		value |= (padctl->usb3_ports[port].ctle_g_val <<
			  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_SHIFT) |
			 (padctl->usb3_ports[port].ctle_z_val <<
			  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_SHIFT);
	}
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_USB3_PADX_CTL2(port));

	value = padctl->soc->dfe_cntl;
	if (padctl->usb3_ports[port].context_saved) {
		value &= ~((XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_MASK <<
			    XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_SHIFT) |
			   (XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_MASK <<
			    XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_SHIFT));
		value |= (padctl->usb3_ports[port].tap1_val <<
			  XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_SHIFT) |
			 (padctl->usb3_ports[port].amp_val <<
			  XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_SHIFT);
	}
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_USB3_PADX_CTL4(port));

	offset = (lane == TEGRA_XUSB_PADCTL_PIN_SATA_0) ?
		XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL2 :
		XUSB_PADCTL_IOPHY_MISC_PAD_PX_CTL2(lane -
						TEGRA_XUSB_PADCTL_PIN_PCIE_0);
	value = padctl_readl(padctl, offset);
	value &= ~(XUSB_PADCTL_IOPHY_MISC_PAD_CTL2_SPARE_IN_MASK <<
		   XUSB_PADCTL_IOPHY_MISC_PAD_CTL2_SPARE_IN_SHIFT);
	value |= padctl->soc->spare_in <<
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL2_SPARE_IN_SHIFT;
	padctl_writel(padctl, value, offset);

	offset = (lane == TEGRA_XUSB_PADCTL_PIN_SATA_0) ?
		XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL5 :
		XUSB_PADCTL_IOPHY_MISC_PAD_PX_CTL5(lane -
						TEGRA_XUSB_PADCTL_PIN_PCIE_0);
	value = padctl_readl(padctl, offset);
	value |= XUSB_PADCTL_IOPHY_MISC_PAD_CTL5_RX_QEYE_EN;
	padctl_writel(padctl, value, offset);

	/* Enable SATA PHY when SATA lane is used */
	if (lane == TEGRA_XUSB_PADCTL_PIN_SATA_0) {
		value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
		value &= ~(XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL0_REFCLK_NDIV_MASK <<
			   XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL0_REFCLK_NDIV_SHIFT);
		value |= 0x2 <<
			XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL0_REFCLK_NDIV_SHIFT;
		padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

		value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL2);
		value &= ~((XUSB_PADCTL_IOPHY_PLL_S0_CTL2_XDIGCLK_SEL_MASK <<
			    XUSB_PADCTL_IOPHY_PLL_S0_CTL2_XDIGCLK_SEL_SHIFT) |
			   (XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL1_CP_CNTL_MASK <<
			    XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL1_CP_CNTL_SHIFT) |
			   (XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL0_CP_CNTL_MASK <<
			    XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL0_CP_CNTL_SHIFT) |
			   XUSB_PADCTL_IOPHY_PLL_S0_CTL2_TCLKOUT_EN);
		value |= (0x7 <<
			  XUSB_PADCTL_IOPHY_PLL_S0_CTL2_XDIGCLK_SEL_SHIFT) |
			 (0x8 <<
			  XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL1_CP_CNTL_SHIFT) |
			 (0x8 <<
			  XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL0_CP_CNTL_SHIFT) |
			 XUSB_PADCTL_IOPHY_PLL_S0_CTL2_TXCLKREF_SEL;
		padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL2);

		value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL3);
		value &= ~XUSB_PADCTL_IOPHY_PLL_S0_CTL3_RCAL_BYPASS;
		padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL3);
	}

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_VCORE_DOWN(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(150, 250);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_CLAMP_EN(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	return 0;
}

static int usb3_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	int port = usb3_phy_to_port(phy);
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value |= XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value |= XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_CLAMP_EN(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(250, 350);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value |= XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_VCORE_DOWN(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	return 0;
}

static void usb3_phy_save_context(struct tegra_xusb_padctl *padctl, int port)
{
	int lane = padctl->usb3_ports[port].lane;
	u32 value, offset;

	padctl->usb3_ports[port].context_saved = true;

	offset = (lane == TEGRA_XUSB_PADCTL_PIN_SATA_0) ?
		XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL6 :
		XUSB_PADCTL_IOPHY_MISC_PAD_PX_CTL6(lane -
						TEGRA_XUSB_PADCTL_PIN_PCIE_0);

	value = padctl_readl(padctl, offset);
	value &= ~(XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_MASK <<
		   XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT);
	value |= XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_TAP <<
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT;
	padctl_writel(padctl, value, offset);

	value = padctl_readl(padctl, offset) >>
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SHIFT;
	padctl->usb3_ports[port].tap1_val = value &
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_TAP_MASK;

	value = padctl_readl(padctl, offset);
	value &= ~(XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_MASK <<
		   XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT);
	value |= XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_AMP <<
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT;
	padctl_writel(padctl, value, offset);

	value = padctl_readl(padctl, offset) >>
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SHIFT;
	padctl->usb3_ports[port].amp_val = value &
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_AMP_MASK;

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_USB3_PADX_CTL4(port));
	value &= ~((XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_SHIFT) |
		   (XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_SHIFT));
	value |= (padctl->usb3_ports[port].tap1_val <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_SHIFT) |
		 (padctl->usb3_ports[port].amp_val <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_USB3_PADX_CTL4(port));

	value = padctl_readl(padctl, offset);
	value &= ~(XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_MASK <<
		   XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT);
	value |= XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_LATCH_G_Z <<
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT;
	padctl_writel(padctl, value, offset);

	value = padctl_readl(padctl, offset);
	value &= ~(XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_MASK <<
		   XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT);
	value |= XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_G_Z <<
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT;
	padctl_writel(padctl, value, offset);

	value = padctl_readl(padctl, offset) >>
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SHIFT;
	padctl->usb3_ports[port].ctle_g_val = value &
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_G_Z_MASK;

	value = padctl_readl(padctl, offset);
	value &= ~(XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_MASK <<
		   XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT);
	value |= XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_CTLE_Z <<
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT;
	padctl_writel(padctl, value, offset);

	value = padctl_readl(padctl, offset) >>
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SHIFT;
	padctl->usb3_ports[port].ctle_z_val = value &
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_G_Z_MASK;

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_USB3_PADX_CTL2(port));
	value &= ~((XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_SHIFT) |
		   (XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_SHIFT));
	value |= (padctl->usb3_ports[port].ctle_g_val <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_SHIFT) |
		 (padctl->usb3_ports[port].ctle_z_val <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_USB3_PADX_CTL2(port));
}

static const struct phy_ops usb3_phy_ops = {
	.init = tegra_xusb_phy_init,
	.exit = tegra_xusb_phy_exit,
	.power_on = usb3_phy_power_on,
	.power_off = usb3_phy_power_off,
	.owner = THIS_MODULE,
};

static int utmi_phy_to_port(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	int i;

	for (i = 0; i < TEGRA_XUSB_UTMI_PHYS; i++) {
		if (phy == padctl->phys[TEGRA_XUSB_PADCTL_UTMI_P0 + i])
			break;
	}
	BUG_ON(i == TEGRA_XUSB_UTMI_PHYS);

	return i;
}

static int utmi_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	int port = utmi_phy_to_port(phy);
	int ret;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);
	value &= ~((XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_MASK <<
		    XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_SHIFT) |
		   (XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_MASK <<
		    XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_SHIFT));
	value |= (padctl->calib.hs_squelch_level <<
		  XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_SHIFT) |
		 (padctl->soc->hs_discon_level <<
		  XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_PORT_CAP);
	value &= ~(XUSB_PADCTL_USB2_PORT_CAP_PORT_CAP_MASK <<
		   XUSB_PADCTL_USB2_PORT_CAP_PORTX_CAP_SHIFT(port));
	value |= XUSB_PADCTL_USB2_PORT_CAP_HOST <<
		XUSB_PADCTL_USB2_PORT_CAP_PORTX_CAP_SHIFT(port);
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_PORT_CAP);

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));
	value &= ~((XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_MASK <<
		    XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_SHIFT) |
		   (XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_SLEW_MASK <<
		    XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_SLEW_SHIFT) |
		   (XUSB_PADCTL_USB2_OTG_PAD_CTL0_LS_RSLEW_MASK <<
		    XUSB_PADCTL_USB2_OTG_PAD_CTL0_LS_RSLEW_SHIFT) |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD2 |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD_ZI);
	value |= (padctl->calib.hs_curr_level[port] +
		  padctl->hs_curr_level_offset[port]) <<
		XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_SHIFT;
	value |= padctl->soc->hs_slew <<
		XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_SLEW_SHIFT;
	value |= padctl->soc->ls_rslew[port] <<
		XUSB_PADCTL_USB2_OTG_PAD_CTL0_LS_RSLEW_SHIFT;
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));
	value &= ~((XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_MASK <<
		    XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_SHIFT) |
		   (XUSB_PADCTL_USB2_OTG_PAD_CTL1_HS_IREF_CAP_MASK <<
		    XUSB_PADCTL_USB2_OTG_PAD_CTL1_HS_IREF_CAP_SHIFT) |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_DR |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_CHRP_FORCE_POWERUP |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_DISC_FORCE_POWERUP);
	value |= (padctl->calib.hs_term_range_adj <<
		  XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_SHIFT) |
		 (padctl->calib.hs_iref_cap <<
		  XUSB_PADCTL_USB2_OTG_PAD_CTL1_HS_IREF_CAP_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));

	ret = regulator_enable(padctl->vbus[port]);
	if (ret)
		return ret;

	mutex_lock(&padctl->lock);

	if (padctl->utmi_enable++ > 0)
		goto out;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);
	value &= ~XUSB_PADCTL_USB2_BIAS_PAD_CTL0_PD;
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);

out:
	mutex_unlock(&padctl->lock);
	return 0;
}

static int utmi_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	int port = utmi_phy_to_port(phy);
	u32 value;

	regulator_disable(padctl->vbus[port]);

	mutex_lock(&padctl->lock);

	if (WARN_ON(padctl->utmi_enable == 0))
		goto out;

	if (--padctl->utmi_enable > 0)
		goto out;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);
	value |= XUSB_PADCTL_USB2_BIAS_PAD_CTL0_PD;
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);

out:
	mutex_unlock(&padctl->lock);
	return 0;
}

static const struct phy_ops utmi_phy_ops = {
	.init = tegra_xusb_phy_init,
	.exit = tegra_xusb_phy_exit,
	.power_on = utmi_phy_power_on,
	.power_off = utmi_phy_power_off,
	.owner = THIS_MODULE,
};

static int hsic_phy_to_port(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	int i;

	for (i = 0; i < TEGRA_XUSB_HSIC_PHYS; i++) {
		if (phy == padctl->phys[TEGRA_XUSB_PADCTL_HSIC_P0 + i])
			break;
	}
	BUG_ON(i == TEGRA_XUSB_HSIC_PHYS);

	return i;
}

static int hsic_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	int port = hsic_phy_to_port(phy);
	int ret;
	u32 value;

	ret = regulator_enable(padctl->vddio_hsic);
	if (ret)
		return ret;

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL1(port));
	value &= ~(XUSB_PADCTL_HSIC_PAD_CTL1_RPD_STROBE |
		   XUSB_PADCTL_HSIC_PAD_CTL1_RPU_DATA |
		   XUSB_PADCTL_HSIC_PAD_CTL1_PD_RX |
		   XUSB_PADCTL_HSIC_PAD_CTL1_PD_ZI |
		   XUSB_PADCTL_HSIC_PAD_CTL1_PD_TRX |
		   XUSB_PADCTL_HSIC_PAD_CTL1_PD_TX);
	value |= XUSB_PADCTL_HSIC_PAD_CTL1_RPD_DATA |
		 XUSB_PADCTL_HSIC_PAD_CTL1_RPU_STROBE;
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL1(port));

	return 0;
}

static int hsic_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	int port = hsic_phy_to_port(phy);
	u32 value;

	regulator_disable(padctl->vddio_hsic);

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL1(port));
	value |= XUSB_PADCTL_HSIC_PAD_CTL1_PD_RX |
		 XUSB_PADCTL_HSIC_PAD_CTL1_PD_ZI |
		 XUSB_PADCTL_HSIC_PAD_CTL1_PD_TRX |
		 XUSB_PADCTL_HSIC_PAD_CTL1_PD_TX;
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL1(port));

	return 0;
}

static void hsic_phy_set_idle(struct tegra_xusb_padctl *padctl, int port,
			      bool set)
{
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL1(port));
	if (set)
		value |= XUSB_PADCTL_HSIC_PAD_CTL1_RPD_DATA |
			 XUSB_PADCTL_HSIC_PAD_CTL1_RPU_STROBE;
	else
		value &= ~(XUSB_PADCTL_HSIC_PAD_CTL1_RPD_DATA |
			   XUSB_PADCTL_HSIC_PAD_CTL1_RPU_STROBE);
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL1(port));
}

static const struct phy_ops hsic_phy_ops = {
	.init = tegra_xusb_phy_init,
	.exit = tegra_xusb_phy_exit,
	.power_on = hsic_phy_power_on,
	.power_off = hsic_phy_power_off,
	.owner = THIS_MODULE,
};

static bool is_phy_mbox_message(u32 cmd)
{
	switch (cmd) {
	case MBOX_CMD_SAVE_DFE_CTLE_CTX:
	case MBOX_CMD_START_HSIC_IDLE:
	case MBOX_CMD_STOP_HSIC_IDLE:
		return true;
	default:
		return false;
	}
}

static void tegra_xusb_phy_mbox_work(struct work_struct *work)
{
	struct tegra_xusb_padctl *padctl = container_of(work,
				struct tegra_xusb_padctl, mbox_req_work);
	struct tegra_xusb_mbox_msg *msg = &padctl->mbox_req;
	struct tegra_xusb_mbox_msg resp;
	u32 ports;
	int i;

	resp.cmd = 0;
	switch (msg->cmd) {
	case MBOX_CMD_SAVE_DFE_CTLE_CTX:
		resp.data = msg->data;
		if (msg->data > TEGRA_XUSB_USB3_PHYS) {
			resp.cmd = MBOX_CMD_NAK;
		} else {
			usb3_phy_save_context(padctl, msg->data);
			resp.cmd = MBOX_CMD_ACK;
		}
		break;
	case MBOX_CMD_START_HSIC_IDLE:
	case MBOX_CMD_STOP_HSIC_IDLE:
		ports = msg->data >> (padctl->soc->hsic_port_offset + 1);
		resp.data = msg->data;
		resp.cmd = MBOX_CMD_ACK;
		for (i = 0; i < TEGRA_XUSB_HSIC_PHYS; i++) {
			if (!(ports & BIT(i)))
				continue;
			if (msg->cmd == MBOX_CMD_START_HSIC_IDLE)
				hsic_phy_set_idle(padctl, i, true);
			else
				hsic_phy_set_idle(padctl, i, false);
		}
		break;
	default:
		break;
	}

	if (resp.cmd)
		mbox_send_message(padctl->mbox_chan, &resp);
}

static void tegra_xusb_phy_mbox_rx(struct mbox_client *cl, void *data)
{
	struct tegra_xusb_padctl *padctl = dev_get_drvdata(cl->dev);
	struct tegra_xusb_mbox_msg *msg = data;

	if (is_phy_mbox_message(msg->cmd)) {
		padctl->mbox_req = *msg;
		schedule_work(&padctl->mbox_req_work);
	}
}

static struct phy *tegra_xusb_padctl_xlate(struct device *dev,
					   struct of_phandle_args *args)
{
	struct tegra_xusb_padctl *padctl = dev_get_drvdata(dev);
	unsigned int index = args->args[0];

	if (args->args_count <= 0)
		return ERR_PTR(-EINVAL);

	if (index >= ARRAY_SIZE(padctl->phys))
		return ERR_PTR(-EINVAL);

	return padctl->phys[index];
}

static const struct pinctrl_pin_desc tegra124_pins[] = {
	PINCTRL_PIN(TEGRA_XUSB_PADCTL_PIN_OTG_0,  "otg-0"),
	PINCTRL_PIN(TEGRA_XUSB_PADCTL_PIN_OTG_1,  "otg-1"),
	PINCTRL_PIN(TEGRA_XUSB_PADCTL_PIN_OTG_2,  "otg-2"),
	PINCTRL_PIN(TEGRA_XUSB_PADCTL_PIN_ULPI_0, "ulpi-0"),
	PINCTRL_PIN(TEGRA_XUSB_PADCTL_PIN_HSIC_0, "hsic-0"),
	PINCTRL_PIN(TEGRA_XUSB_PADCTL_PIN_HSIC_1, "hsic-1"),
	PINCTRL_PIN(TEGRA_XUSB_PADCTL_PIN_PCIE_0, "pcie-0"),
	PINCTRL_PIN(TEGRA_XUSB_PADCTL_PIN_PCIE_1, "pcie-1"),
	PINCTRL_PIN(TEGRA_XUSB_PADCTL_PIN_PCIE_2, "pcie-2"),
	PINCTRL_PIN(TEGRA_XUSB_PADCTL_PIN_PCIE_3, "pcie-3"),
	PINCTRL_PIN(TEGRA_XUSB_PADCTL_PIN_PCIE_4, "pcie-4"),
	PINCTRL_PIN(TEGRA_XUSB_PADCTL_PIN_SATA_0, "sata-0"),
};

static const char * const tegra124_snps_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
	"ulpi-0",
	"hsic-0",
	"hsic-1",
};

static const char * const tegra124_xusb_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
	"ulpi-0",
	"hsic-0",
	"hsic-1",
};

static const char * const tegra124_uart_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
};

static const char * const tegra124_pcie_groups[] = {
	"pcie-0",
	"pcie-1",
	"pcie-2",
	"pcie-3",
	"pcie-4",
	"sata-0",
};

static const char * const tegra124_usb3_groups[] = {
	"pcie-0",
	"pcie-1",
	"pcie-2",
	"pcie-3",
	"pcie-4",
	"sata-0",
};

static const char * const tegra124_sata_groups[] = {
	"pcie-0",
	"pcie-1",
	"pcie-2",
	"pcie-3",
	"pcie-4",
	"sata-0",
};

static const char * const tegra124_rsvd_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
	"pcie-0",
	"pcie-1",
	"pcie-2",
	"pcie-3",
	"pcie-4",
	"sata-0",
};

#define TEGRA124_FUNCTION(_name)					\
	{								\
		.name = #_name,						\
		.num_groups = ARRAY_SIZE(tegra124_##_name##_groups),	\
		.groups = tegra124_##_name##_groups,			\
	}

static struct tegra_xusb_padctl_function tegra124_functions[] = {
	TEGRA124_FUNCTION(snps),
	TEGRA124_FUNCTION(xusb),
	TEGRA124_FUNCTION(uart),
	TEGRA124_FUNCTION(pcie),
	TEGRA124_FUNCTION(usb3),
	TEGRA124_FUNCTION(sata),
	TEGRA124_FUNCTION(rsvd),
};

enum tegra124_function {
	TEGRA124_FUNC_SNPS,
	TEGRA124_FUNC_XUSB,
	TEGRA124_FUNC_UART,
	TEGRA124_FUNC_PCIE,
	TEGRA124_FUNC_USB3,
	TEGRA124_FUNC_SATA,
	TEGRA124_FUNC_RSVD,
};

static const unsigned int tegra124_otg_functions[] = {
	TEGRA124_FUNC_SNPS,
	TEGRA124_FUNC_XUSB,
	TEGRA124_FUNC_UART,
	TEGRA124_FUNC_RSVD,
};

static const unsigned int tegra124_usb_functions[] = {
	TEGRA124_FUNC_SNPS,
	TEGRA124_FUNC_XUSB,
};

static const unsigned int tegra124_pci_functions[] = {
	TEGRA124_FUNC_PCIE,
	TEGRA124_FUNC_USB3,
	TEGRA124_FUNC_SATA,
	TEGRA124_FUNC_RSVD,
};

#define TEGRA124_LANE(_name, _offset, _shift, _mask, _iddq, _funcs)	\
	{								\
		.name = _name,						\
		.offset = _offset,					\
		.shift = _shift,					\
		.mask = _mask,						\
		.iddq = _iddq,						\
		.num_funcs = ARRAY_SIZE(tegra124_##_funcs##_functions),	\
		.funcs = tegra124_##_funcs##_functions,			\
	}

static const struct tegra_xusb_padctl_lane tegra124_lanes[] = {
	TEGRA124_LANE("otg-0",  0x004,  0, 0x3, 0, otg),
	TEGRA124_LANE("otg-1",  0x004,  2, 0x3, 0, otg),
	TEGRA124_LANE("otg-2",  0x004,  4, 0x3, 0, otg),
	TEGRA124_LANE("ulpi-0", 0x004, 12, 0x1, 0, usb),
	TEGRA124_LANE("hsic-0", 0x004, 14, 0x1, 0, usb),
	TEGRA124_LANE("hsic-1", 0x004, 15, 0x1, 0, usb),
	TEGRA124_LANE("pcie-0", 0x134, 16, 0x3, 1, pci),
	TEGRA124_LANE("pcie-1", 0x134, 18, 0x3, 2, pci),
	TEGRA124_LANE("pcie-2", 0x134, 20, 0x3, 3, pci),
	TEGRA124_LANE("pcie-3", 0x134, 22, 0x3, 4, pci),
	TEGRA124_LANE("pcie-4", 0x134, 24, 0x3, 5, pci),
	TEGRA124_LANE("sata-0", 0x134, 26, 0x3, 6, pci),
};

static const struct tegra_xusb_padctl_soc tegra124_soc = {
	.num_pins = ARRAY_SIZE(tegra124_pins),
	.pins = tegra124_pins,
	.num_functions = ARRAY_SIZE(tegra124_functions),
	.functions = tegra124_functions,
	.num_lanes = ARRAY_SIZE(tegra124_lanes),
	.lanes = tegra124_lanes,
	.rx_wander = 0xf,
	.rx_eq = 0xf070,
	.cdr_cntl = 0x24,
	.dfe_cntl = 0x002008ee,
	.hs_slew = 0xe,
	.ls_rslew = {0x3, 0x0, 0x0},
	.hs_discon_level = 0x5,
	.spare_in = 0x1,
	.hsic_port_offset = 6,
};

static const struct of_device_id tegra_xusb_padctl_of_match[] = {
	{ .compatible = "nvidia,tegra124-xusb-padctl", .data = &tegra124_soc },
	{ }
};
MODULE_DEVICE_TABLE(of, tegra_xusb_padctl_of_match);

static int tegra_xusb_read_fuse_calibration(struct tegra_xusb_padctl *padctl)
{
	int i, ret;
	u32 value;

	ret = tegra_fuse_readl(TEGRA_FUSE_SKU_CALIB_0, &value);
	if (ret < 0)
		return ret;

	for (i = 0; i < TEGRA_XUSB_UTMI_PHYS; i++) {
		padctl->calib.hs_curr_level[i] =
			(value >> FUSE_SKU_CALIB_HS_CURR_LEVEL_PADX_SHIFT(i)) &
			FUSE_SKU_CALIB_HS_CURR_LEVEL_PAD_MASK;
	}
	padctl->calib.hs_iref_cap =
		(value >> FUSE_SKU_CALIB_HS_IREF_CAP_SHIFT) &
		FUSE_SKU_CALIB_HS_IREF_CAP_MASK;
	padctl->calib.hs_term_range_adj =
		(value >> FUSE_SKU_CALIB_HS_TERM_RANGE_ADJ_SHIFT) &
		FUSE_SKU_CALIB_HS_TERM_RANGE_ADJ_MASK;
	padctl->calib.hs_squelch_level =
		(value >> FUSE_SKU_CALIB_HS_SQUELCH_LEVEL_SHIFT) &
		FUSE_SKU_CALIB_HS_SQUELCH_LEVEL_MASK;

	return 0;
}

static int tegra_xusb_padctl_probe(struct platform_device *pdev)
{
	struct tegra_xusb_padctl *padctl;
	const struct of_device_id *match;
	struct resource *res;
	struct phy *phy;
	int err, i;

	padctl = devm_kzalloc(&pdev->dev, sizeof(*padctl), GFP_KERNEL);
	if (!padctl)
		return -ENOMEM;

	platform_set_drvdata(pdev, padctl);
	mutex_init(&padctl->lock);
	padctl->dev = &pdev->dev;

	match = of_match_node(tegra_xusb_padctl_of_match, pdev->dev.of_node);
	padctl->soc = match->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	padctl->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(padctl->regs))
		return PTR_ERR(padctl->regs);

	err = tegra_xusb_read_fuse_calibration(padctl);
	if (err < 0)
		return err;

	padctl->rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(padctl->rst))
		return PTR_ERR(padctl->rst);

	err = reset_control_deassert(padctl->rst);
	if (err < 0)
		return err;

	for (i = 0; i < TEGRA_XUSB_USB3_PHYS; i++) {
		char prop[sizeof("nvidia,usb3-port-N-lane")];
		u32 lane;

		sprintf(prop, "nvidia,usb3-port-%d-lane", i);
		if (!of_property_read_u32(pdev->dev.of_node, prop, &lane)) {
			if (!is_pcie_or_sata_lane(lane)) {
				dev_err(&pdev->dev,
					"USB3 port mapped to invalid lane\n");
				err = -EINVAL;
				goto unregister;
			}
			padctl->usb3_ports[i].lane = lane;
		} else {
			padctl->usb3_ports[i].lane = -EINVAL;
		}
	}

	memset(&padctl->desc, 0, sizeof(padctl->desc));
	padctl->desc.name = dev_name(padctl->dev);
	padctl->desc.pctlops = &tegra_xusb_padctl_pinctrl_ops;
	padctl->desc.pmxops = &tegra_xusb_padctl_pinmux_ops;
	padctl->desc.confops = &tegra_xusb_padctl_pinconf_ops;
	padctl->desc.owner = THIS_MODULE;

	padctl->pinctrl = pinctrl_register(&padctl->desc, &pdev->dev, padctl);
	if (!padctl->pinctrl) {
		dev_err(&pdev->dev, "failed to register pincontrol\n");
		err = -ENODEV;
		goto reset;
	}

	phy = devm_phy_create(&pdev->dev, NULL, &pcie_phy_ops, NULL);
	if (IS_ERR(phy)) {
		err = PTR_ERR(phy);
		goto unregister;
	}

	padctl->phys[TEGRA_XUSB_PADCTL_PCIE] = phy;
	phy_set_drvdata(phy, padctl);

	phy = devm_phy_create(&pdev->dev, NULL, &sata_phy_ops, NULL);
	if (IS_ERR(phy)) {
		err = PTR_ERR(phy);
		goto unregister;
	}

	padctl->phys[TEGRA_XUSB_PADCTL_SATA] = phy;
	phy_set_drvdata(phy, padctl);

	for (i = 0; i < TEGRA_XUSB_USB3_PHYS; i++) {
		phy = devm_phy_create(&pdev->dev, NULL, &usb3_phy_ops, NULL);
		if (IS_ERR(phy)) {
			err = PTR_ERR(phy);
			goto unregister;
		}

		padctl->phys[TEGRA_XUSB_PADCTL_USB3_P0 + i] = phy;
		phy_set_drvdata(phy, padctl);
	}

	for (i = 0; i < TEGRA_XUSB_UTMI_PHYS; i++) {
		char reg_name[sizeof("vbus-N")];

		sprintf(reg_name, "vbus-%d", i);
		padctl->vbus[i] = devm_regulator_get(&pdev->dev, reg_name);
		if (IS_ERR(padctl->vbus[i])) {
			err = PTR_ERR(padctl->vbus[i]);
			goto unregister;
		}

		phy = devm_phy_create(&pdev->dev, NULL, &utmi_phy_ops, NULL);
		if (IS_ERR(phy)) {
			err = PTR_ERR(phy);
			goto unregister;
		}

		padctl->phys[TEGRA_XUSB_PADCTL_UTMI_P0 + i] = phy;
		phy_set_drvdata(phy, padctl);
	}

	padctl->vddio_hsic = devm_regulator_get(&pdev->dev, "vddio-hsic");
	if (IS_ERR(padctl->vddio_hsic)) {
		err = PTR_ERR(padctl->vddio_hsic);
		goto unregister;
	}

	for (i = 0; i < TEGRA_XUSB_HSIC_PHYS; i++) {
		phy = devm_phy_create(&pdev->dev, NULL, &hsic_phy_ops, NULL);
		if (IS_ERR(phy)) {
			err = PTR_ERR(phy);
			goto unregister;
		}

		padctl->phys[TEGRA_XUSB_PADCTL_HSIC_P0 + i] = phy;
		phy_set_drvdata(phy, padctl);
	}

	padctl->provider = devm_of_phy_provider_register(&pdev->dev,
							 tegra_xusb_padctl_xlate);
	if (IS_ERR(padctl->provider)) {
		err = PTR_ERR(padctl->provider);
		dev_err(&pdev->dev, "failed to register PHYs: %d\n", err);
		goto unregister;
	}

	INIT_WORK(&padctl->mbox_req_work, tegra_xusb_phy_mbox_work);
	padctl->mbox_client.dev = &pdev->dev;
	padctl->mbox_client.tx_block = true;
	padctl->mbox_client.tx_tout = 0;
	padctl->mbox_client.rx_callback = tegra_xusb_phy_mbox_rx;
	padctl->mbox_chan = mbox_request_channel(&padctl->mbox_client, 0);
	if (IS_ERR(padctl->mbox_chan)) {
		err = PTR_ERR(padctl->mbox_chan);
		dev_err(&pdev->dev, "failed to request mailbox: %d\n", err);
		goto unregister;
	}

	return 0;

unregister:
	pinctrl_unregister(padctl->pinctrl);
reset:
	reset_control_assert(padctl->rst);
	return err;
}

static int tegra_xusb_padctl_remove(struct platform_device *pdev)
{
	struct tegra_xusb_padctl *padctl = platform_get_drvdata(pdev);
	int err;

	cancel_work_sync(&padctl->mbox_req_work);
	mbox_free_channel(padctl->mbox_chan);

	pinctrl_unregister(padctl->pinctrl);

	err = reset_control_assert(padctl->rst);
	if (err < 0)
		dev_err(&pdev->dev, "failed to assert reset: %d\n", err);

	return err;
}

static struct platform_driver tegra_xusb_padctl_driver = {
	.driver = {
		.name = "tegra-xusb-padctl",
		.of_match_table = tegra_xusb_padctl_of_match,
	},
	.probe = tegra_xusb_padctl_probe,
	.remove = tegra_xusb_padctl_remove,
};
module_platform_driver(tegra_xusb_padctl_driver);

MODULE_AUTHOR("Thierry Reding <treding@nvidia.com>");
MODULE_DESCRIPTION("Tegra 124 XUSB Pad Control driver");
MODULE_LICENSE("GPL v2");
