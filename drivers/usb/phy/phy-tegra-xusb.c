/*
 * Copyright (C) 2014 Google, Inc.
 * Copyright (C) 2014 NVIDIA Corporation
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

#include <linux/slab.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/tegra-soc.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <linux/usb/phy.h>
#include <linux/usb/tegra_xusb_phy.h>
#include <linux/usb/tegra_usb_pmc.h>
#include <linux/usb/tegra_xusb.h>

struct tegra_xusb_hsic_config {
	u8 rx_strobe_trim;
	u8 rx_data_trim;
	u8 tx_rtune_n;
	u8 tx_rtune_p;
	u8 tx_slew_n;
	u8 tx_slew_p;
	bool auto_term_en;
	u8 strb_trim_val;
	bool pretend_connect;
};

struct tegra_xusb_phy_calib_data {
	u32 hs_curr_level_pad[TEGRA_XUSB_UTMI_COUNT];
	u32 hs_iref_cap;
	u32 hs_term_range_adj;
	u32 hs_squelch_level;
};

struct tegra_xusb_phy_board_data {
	unsigned long utmi_pads;
	unsigned long hsic_pads;
	unsigned long ss_pads;
	/*
	 * SS0 or SS1 port may be mapped either to USB2_P0 or USB2_P1
	 * ss_portmap[0:3] = SS0 map, ss_portmap[4:7] = SS1 map
	 */
	u32 ss_portmap;
	u32 lane_owner;
	struct tegra_xusb_hsic_config hsic[TEGRA_XUSB_HSIC_COUNT];
	u32 hs_xcvr_setup_offset;
};

struct tegra_xusb_phy_config {
	bool shared_ss_lanes;
	bool save_ctle_context;
	bool release_utmi_in_elpg;
	bool use_hs_src_clk2;
	bool recalc_tctrl_rctrl;
	int num_utmi_pads;
	u32 rx_wander;
	u32 rx_eq;
	u32 cdr_cntl;
	u32 dfe_cntl;
	u32 hs_slew;
	u32 ls_rslew_pad[TEGRA_XUSB_UTMI_COUNT];
	u32 hs_disc_lvl;
	u32 spare_in;
	u32 pmc_portmap[TEGRA_XUSB_UTMI_COUNT];
	int utmi_port_offset;
	int hsic_port_offset;
	const struct tegra_xusb_padctl_regs *padctl_offsets;
};

struct tegra_xusb_phy {
	struct device *dev;
	struct tegra_xhci_hcd *xhci;
	struct usb_phy u_phy;

	void __iomem *padctl_regs;
	void __iomem *pad_regs;
	struct regmap *clkrst_regs;
	struct regmap *pmc_regs;

	struct notifier_block mbox_nb;

	int padctl_irq;

	struct clk *ss_src_clk;
	struct clk *ss_clk;
	struct clk *pll_u_480M;
	struct clk *clk_m;
	struct clk *pad_clk;
	struct clk *plle;

	struct regulator *utmi_vbus[TEGRA_XUSB_UTMI_COUNT];
	struct regulator *vddio_hsic;

	/* DFE and CTLE context */
	u8 ss_ctx_saved;
	u8 tap1_val[TEGRA_XUSB_SS_COUNT];
	u8 amp_val[TEGRA_XUSB_SS_COUNT];
	u8 ctle_z_val[TEGRA_XUSB_SS_COUNT];
	u8 ctle_g_val[TEGRA_XUSB_SS_COUNT];

	/* UTMI context */
	u32 utmip_tctrl_val;
	u32 utmip_rctrl_val;

	struct tegra_xusb_phy_board_data board_data;
	struct tegra_xusb_phy_calib_data calib_data;
	const struct tegra_xusb_phy_config *soc_config;
};

enum hsic_pad_pupd {
	PUPD_DISABLE = 0,
	PUPD_IDLE,
	PUPD_RESET
};

static inline struct tegra_xusb_phy *phy_to_tegra(struct usb_phy *phy)
{
	return container_of(phy, struct tegra_xusb_phy, u_phy);
}

static inline u32 padctl_readl(struct tegra_xusb_phy *tegra, u32 reg)
{
	BUG_ON(reg == PADCTL_REG_NONE);
	return readl(tegra->padctl_regs + reg);
}

static inline void padctl_writel(struct tegra_xusb_phy *tegra, u32 val, u32 reg)
{
	BUG_ON(reg == PADCTL_REG_NONE);
	writel(val, tegra->padctl_regs + reg);
}

static inline u32 pad_readl(struct tegra_xusb_phy *tegra, u32 reg)
{
	return readl(tegra->pad_regs + reg);
}

static inline void pad_writel(struct tegra_xusb_phy *tegra, u32 val, u32 reg)
{
	writel(val, tegra->pad_regs + reg);
}

static inline u32 clkrst_readl(struct tegra_xusb_phy *tegra, u32 reg)
{
	u32 val;

	regmap_read(tegra->clkrst_regs, reg, &val);
	return val;
}

static inline void clkrst_writel(struct tegra_xusb_phy *tegra, u32 val, u32 reg)
{
	regmap_write(tegra->clkrst_regs, reg, val);
}

static inline u32 pmc_readl(struct tegra_xusb_phy *tegra, u32 reg)
{
	u32 val;

	regmap_read(tegra->pmc_regs, reg, &val);
	return val;
}

static inline void pmc_writel(struct tegra_xusb_phy *tegra, u32 val, u32 reg)
{
	regmap_write(tegra->pmc_regs, reg, val);
}

static void clear_wake_interrupts(struct tegra_xusb_phy *tegra)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 elpg_program0;

	padregs = tegra->soc_config->padctl_offsets;
	elpg_program0 = padctl_readl(tegra, padregs->elpg_program_0);
	elpg_program0 |= WAKEUP_EVENT_MASK;
	padctl_writel(tegra, elpg_program0, padregs->elpg_program_0);
}

static void disable_wake_interrupts(struct tegra_xusb_phy *tegra)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 elpg_program0;
	unsigned long ss_pads = tegra->board_data.ss_pads;
	unsigned long hsic_pads = tegra->board_data.hsic_pads;
	unsigned long utmi_pads = tegra->board_data.utmi_pads;
	int i;

	padregs = tegra->soc_config->padctl_offsets;
	clear_wake_interrupts(tegra);
	elpg_program0 = padctl_readl(tegra, padregs->elpg_program_0);
	for_each_set_bit(i, &ss_pads, TEGRA_XUSB_SS_COUNT)
		elpg_program0 &= ~SS_PORT_WAKE_INTERRUPT_ENABLE(i);
	for_each_set_bit(i, &hsic_pads, TEGRA_XUSB_HSIC_COUNT)
		elpg_program0 &= ~USB2_HSIC_PORT_WAKE_INTERRUPT_ENABLE(i);
	for_each_set_bit(i, &utmi_pads, TEGRA_XUSB_UTMI_COUNT)
		elpg_program0 &= ~USB2_PORT_WAKE_INTERRUPT_ENABLE(i);
	padctl_writel(tegra, elpg_program0, padregs->elpg_program_0);
}

static void enable_wake_interrupts(struct tegra_xusb_phy *tegra)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 elpg_program0;
	unsigned long ss_pads = tegra->board_data.ss_pads;
	unsigned long hsic_pads = tegra->board_data.hsic_pads;
	unsigned long utmi_pads = tegra->board_data.utmi_pads;
	int i;

	padregs = tegra->soc_config->padctl_offsets;
	clear_wake_interrupts(tegra);
	elpg_program0 = padctl_readl(tegra, padregs->elpg_program_0);
	for_each_set_bit(i, &ss_pads, TEGRA_XUSB_SS_COUNT)
		elpg_program0 |= SS_PORT_WAKE_INTERRUPT_ENABLE(i);
	for_each_set_bit(i, &hsic_pads, TEGRA_XUSB_HSIC_COUNT)
		elpg_program0 |= USB2_HSIC_PORT_WAKE_INTERRUPT_ENABLE(i);
	for_each_set_bit(i, &utmi_pads, TEGRA_XUSB_UTMI_COUNT)
		elpg_program0 |= USB2_PORT_WAKE_INTERRUPT_ENABLE(i);
	padctl_writel(tegra, elpg_program0, padregs->elpg_program_0);
}

static void hsic_pad_enable(struct tegra_xusb_phy *tegra, u8 pad)
{
	const struct tegra_xusb_padctl_regs *padregs;
	struct tegra_xusb_hsic_config *hsic = &tegra->board_data.hsic[pad];
	u32 reg;

	padregs = tegra->soc_config->padctl_offsets;

	reg = padctl_readl(tegra, padregs->usb2_hsic_padX_ctlY_0[pad][2]);
	reg &= ~(USB2_HSIC_RX_STROBE_TRIM(~0) | USB2_HSIC_RX_DATA_TRIM(~0));
	reg |= USB2_HSIC_RX_STROBE_TRIM(hsic->rx_strobe_trim);
	reg |= USB2_HSIC_RX_DATA_TRIM(hsic->rx_data_trim);
	padctl_writel(tegra, reg, padregs->usb2_hsic_padX_ctlY_0[pad][2]);

	reg = padctl_readl(tegra, padregs->usb2_hsic_padX_ctlY_0[pad][0]);
	reg &= ~(USB2_HSIC_TX_RTUNEP(~0) | USB2_HSIC_TX_RTUNEN(~0) |
		 USB2_HSIC_TX_SLEWP(~0) | USB2_HSIC_TX_SLEWN(~0));
	reg |= USB2_HSIC_TX_RTUNEP(hsic->tx_rtune_p);
	reg |= USB2_HSIC_TX_RTUNEN(hsic->tx_rtune_n);
	reg |= USB2_HSIC_TX_SLEWP(hsic->tx_slew_p);
	reg |= USB2_HSIC_TX_SLEWN(hsic->tx_slew_n);
	padctl_writel(tegra, reg, padregs->usb2_hsic_padX_ctlY_0[pad][0]);

	reg = padctl_readl(tegra, padregs->usb2_hsic_padX_ctlY_0[pad][1]);
	reg &= ~(USB2_HSIC_RPD_DATA | USB2_HSIC_RPD_STROBE |
		 USB2_HSIC_RPU_DATA | USB2_HSIC_RPU_STROBE);
	/* Keep HSIC in IDLE */
	reg |= (USB2_HSIC_RPD_DATA | USB2_HSIC_RPU_STROBE);
	if (hsic->auto_term_en)
		reg |= USB2_HSIC_AUTO_TERM_EN;
	else
		reg &= ~USB2_HSIC_AUTO_TERM_EN;
	reg &= ~(USB2_HSIC_PD_RX | USB2_HSIC_PD_ZI |
		 USB2_HSIC_PD_TRX | USB2_HSIC_PD_TX);
	padctl_writel(tegra, reg, padregs->usb2_hsic_padX_ctlY_0[pad][1]);

	reg = padctl_readl(tegra, padregs->hsic_strb_trim_ctl0);
	reg &= ~(HSIC_STRB_TRIM_VAL(~0));
	reg |= HSIC_STRB_TRIM_VAL(hsic->strb_trim_val);
	padctl_writel(tegra, reg, padregs->hsic_strb_trim_ctl0);

	reg = padctl_readl(tegra, padregs->usb2_pad_mux_0);
	reg |= USB2_HSIC_PAD_PORT(pad);
	padctl_writel(tegra, reg, padregs->usb2_pad_mux_0);
}

static void hsic_pad_disable(struct tegra_xusb_phy *tegra, u8 pad)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 reg;

	padregs = tegra->soc_config->padctl_offsets;

	reg = padctl_readl(tegra, padregs->usb2_pad_mux_0);
	reg &= ~USB2_HSIC_PAD_PORT(pad);
	padctl_writel(tegra, reg, padregs->usb2_pad_mux_0);

	reg = padctl_readl(tegra, padregs->usb2_hsic_padX_ctlY_0[pad][1]);
	reg |= (USB2_HSIC_PD_RX | USB2_HSIC_PD_ZI | USB2_HSIC_PD_TRX |
		USB2_HSIC_PD_TX);
	padctl_writel(tegra, reg, padregs->usb2_hsic_padX_ctlY_0[pad][1]);
}

static int hsic_pad_set_pupd(struct tegra_xusb_phy *tegra, u8 pad,
			     enum hsic_pad_pupd pupd)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 reg;

	padregs = tegra->soc_config->padctl_offsets;

	reg = padctl_readl(tegra, padregs->usb2_hsic_padX_ctlY_0[pad][1]);
	reg &= ~(USB2_HSIC_RPD_DATA | USB2_HSIC_RPD_STROBE |
		 USB2_HSIC_RPU_DATA | USB2_HSIC_RPU_STROBE);
	if (pupd == PUPD_IDLE)
		reg |= (USB2_HSIC_RPD_DATA | USB2_HSIC_RPU_STROBE);
	else if (pupd == PUPD_RESET)
		reg |= (USB2_HSIC_RPD_DATA | USB2_HSIC_RPD_STROBE);
	else if (pupd != PUPD_DISABLE) {
		dev_err(tegra->dev, "invalid hsic pupd %d\n", pupd);
		return -EINVAL;
	}
	padctl_writel(tegra, reg, padregs->usb2_hsic_padX_ctlY_0[pad][1]);

	return 0;
}

static void utmi_calc_tctrl_rctrl(struct tegra_xusb_phy *tegra)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 reg, utmi_pads, utmi_mask;

	padregs = tegra->soc_config->padctl_offsets;
	/* Use XUSB_PADCTL space only when XUSB owns all UTMIP port */
	utmi_pads = tegra->board_data.utmi_pads;
	utmi_mask = (1 << tegra->soc_config->num_utmi_pads) - 1;
	if ((utmi_pads & utmi_mask) == utmi_mask) {
		/*
		 * USB2_BIAS_PAD_CTL0_0_PD = 0
		 * USB2_BIAS_PAD_CTL0_0_PD_TRK = 0
		 */
		reg = padctl_readl(tegra, padregs->usb2_bias_pad_ctlY_0[0]);
		reg &= ~(USB2_BIAS_PD | USB2_BIAS_PD_TRK);
		padctl_writel(tegra, reg, padregs->usb2_bias_pad_ctlY_0[0]);

		/* Wait 20us */
		usleep_range(20, 30);

		/*
		 * Read USB2_BIAS_PAD_CTL1_0_{TCTRL,RCTRL}
		 */
		reg = padctl_readl(tegra, padregs->usb2_bias_pad_ctlY_0[1]);
		tegra->utmip_rctrl_val = 0xf + ffz(USB2_BIAS_RCTRL_VAL(reg));
		tegra->utmip_tctrl_val = 0xf + ffz(USB2_BIAS_TCTRL_VAL(reg));

		reg = padctl_readl(tegra, padregs->usb2_bias_pad_ctlY_0[0]);
		reg |= USB2_BIAS_PD_TRK;
		padctl_writel(tegra, reg, padregs->usb2_bias_pad_ctlY_0[0]);

		/*
		 * Program thermally encoded RCTRL_VAL, TCTRL_VAL into PMC
		 * space and set the PMC override.
		 */
		reg = PMC_TCTRL_VAL(tegra->utmip_tctrl_val) |
			PMC_RCTRL_VAL(tegra->utmip_rctrl_val);
		pmc_writel(tegra, reg, PMC_UTMIP_TERM_PAD_CFG);
		reg = pmc_readl(tegra, PMC_SLEEP_CFG);
		reg |= UTMIP_RCTRL_USE_PMC_P2 | UTMIP_TCTRL_USE_PMC_P2;
		pmc_writel(tegra, reg, PMC_SLEEP_CFG);
	} else {
		/* Use common PMC API to use SNPS register space */
		clk_prepare_enable(tegra->pad_clk);

		/* Bias pad MASTER_ENABLE=1 */
		reg = pmc_readl(tegra, PMC_UTMIP_BIAS_MASTER_CNTRL);
		reg |= BIAS_MASTER_PROG_VAL;
		pmc_writel(tegra, reg, PMC_UTMIP_BIAS_MASTER_CNTRL);

		/* Set the tracking length time */
		reg = pad_readl(tegra, UTMIP_BIAS_CFG1);
		reg &= ~UTMIP_BIAS_PDTRK_COUNT(~0);
		reg |= UTMIP_BIAS_PDTRK_COUNT(5);
		pad_writel(tegra, reg, UTMIP_BIAS_CFG1);

		/* Bias PDTRK is shared and MUST be done from USB1 ONLY */
		reg = pad_readl(tegra, UTMIP_BIAS_CFG1);
		reg &= ~UTMIP_BIAS_PDTRK_POWERDOWN;
		pad_writel(tegra, reg, UTMIP_BIAS_CFG1);

		reg = pad_readl(tegra, UTMIP_BIAS_CFG1);
		reg |= UTMIP_BIAS_PDTRK_POWERUP;
		pad_writel(tegra, reg, UTMIP_BIAS_CFG1);

		/* Wait for 25usec */
		udelay(25);

		/* Bias pad MASTER_ENABLE=0 */
		reg = pmc_readl(tegra, PMC_UTMIP_BIAS_MASTER_CNTRL);
		reg &= ~BIAS_MASTER_PROG_VAL;
		pmc_writel(tegra, reg, PMC_UTMIP_BIAS_MASTER_CNTRL);

		/* Wait for 1usec */
		udelay(1);

		/* Bias pad MASTER_ENABLE=1 */
		reg = pmc_readl(tegra, PMC_UTMIP_BIAS_MASTER_CNTRL);
		reg |= BIAS_MASTER_PROG_VAL;
		pmc_writel(tegra, reg, PMC_UTMIP_BIAS_MASTER_CNTRL);

		/* Read RCTRL and TCTRL from UTMIP space */
		reg = pad_readl(tegra, UTMIP_BIAS_STS0);
		tegra->utmip_rctrl_val = 0xf + ffz(UTMIP_RCTRL_VAL(reg));
		tegra->utmip_tctrl_val = 0xf + ffz(UTMIP_TCTRL_VAL(reg));

		/* PD_TRK=1 */
		reg = pad_readl(tegra, UTMIP_BIAS_CFG1);
		reg |= UTMIP_BIAS_PDTRK_POWERDOWN;
		pad_writel(tegra, reg, UTMIP_BIAS_CFG1);

		/*
		 * Program thermally encoded RCTRL_VAL, TCTRL_VAL into PMC
		 * space.
		 */
		reg = PMC_TCTRL_VAL(tegra->utmip_tctrl_val) |
			PMC_RCTRL_VAL(tegra->utmip_rctrl_val);
		pmc_writel(tegra, reg, PMC_UTMIP_TERM_PAD_CFG);
		clk_disable_unprepare(tegra->pad_clk);
	}
	dev_dbg(tegra->dev, "rctrl = 0x%x, tctrl = 0x%x\n",
		tegra->utmip_rctrl_val, tegra->utmip_tctrl_val);
}

static void utmi_phy_iddq_override(struct tegra_xusb_phy *tegra, bool set)
{
	u32 val;

	val = clkrst_readl(tegra, UTMIPLL_HW_PWRDN_CFG0);
	if (set)
		val |= UTMIPLL_IDDQ_OVERRIDE;
	else
		val &= ~UTMIPLL_IDDQ_OVERRIDE;
	val |= UTMIPLL_IDDQ_SWCTL;
	clkrst_writel(tegra, val, UTMIPLL_HW_PWRDN_CFG0);
}

static void utmi_pads_enable(struct tegra_xusb_phy *tegra)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 val;

	padregs = tegra->soc_config->padctl_offsets;
	clk_prepare_enable(tegra->pad_clk);

	val = pad_readl(tegra, UTMIP_BIAS_CFG0);
	val &= ~(UTMIP_OTGPD | UTMIP_BIASPD);
	val |= UTMIP_HSSQUELCH_LEVEL(0x2) | UTMIP_HSDISCON_LEVEL(0x1) |
		UTMIP_HSDISCON_LEVEL_MSB;
	pad_writel(tegra, val, UTMIP_BIAS_CFG0);

	val = padctl_readl(tegra, padregs->usb2_bias_pad_ctlY_0[0]);
	val &= ~USB2_BIAS_PD;
	padctl_writel(tegra, val, padregs->usb2_bias_pad_ctlY_0[0]);
	clk_disable_unprepare(tegra->pad_clk);

	utmi_phy_iddq_override(tegra, false);
}

static void utmi_pads_disable(struct tegra_xusb_phy *tegra)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 val;

	padregs = tegra->soc_config->padctl_offsets;
	clk_prepare_enable(tegra->pad_clk);

	val = pad_readl(tegra, UTMIP_BIAS_CFG0);
	val |= UTMIP_OTGPD | UTMIP_BIASPD;
	val &= ~(UTMIP_HSSQUELCH_LEVEL(~0) | UTMIP_HSDISCON_LEVEL(~0) |
		 UTMIP_HSDISCON_LEVEL_MSB);
	pad_writel(tegra, val, UTMIP_BIAS_CFG0);

	val = padctl_readl(tegra, padregs->usb2_bias_pad_ctlY_0[0]);
	val |= USB2_BIAS_PD;
	padctl_writel(tegra, val, padregs->usb2_bias_pad_ctlY_0[0]);
	clk_disable_unprepare(tegra->pad_clk);

	utmi_phy_iddq_override(tegra, true);
}

static void utmi_pad_init(struct tegra_xusb_phy *tegra, u8 port)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 reg;
	u32 ctl0_offset, ctl1_offset;
	struct tegra_xusb_phy_board_data *bdata = &tegra->board_data;
	u32 val;

	padregs = tegra->soc_config->padctl_offsets;

	reg = padctl_readl(tegra, padregs->usb2_pad_mux_0);
	reg &= ~USB2_OTG_PAD_PORT_MASK(port);
	reg |= USB2_OTG_PAD_PORT_OWNER_XUSB(port);
	padctl_writel(tegra, reg, padregs->usb2_pad_mux_0);

	reg = padctl_readl(tegra, padregs->usb2_port_cap_0);
	reg &= ~USB2_PORT_CAP_MASK(port);
	reg |= USB2_PORT_CAP_HOST(port);
	padctl_writel(tegra, reg, padregs->usb2_port_cap_0);

	ctl0_offset = padregs->usb2_otg_padX_ctlY_0[port][0];
	ctl1_offset = padregs->usb2_otg_padX_ctlY_0[port][1];

	reg = padctl_readl(tegra, ctl0_offset);
	reg &= ~(USB2_OTG_HS_CURR_LVL | USB2_OTG_HS_SLEW |
		USB2_OTG_FS_SLEW | USB2_OTG_LS_RSLEW |
		USB2_OTG_PD | USB2_OTG_PD2 | USB2_OTG_PD_ZI);
	reg |= tegra->soc_config->hs_slew;
	reg |= tegra->soc_config->ls_rslew_pad[port];
	val = (bdata->hs_xcvr_setup_offset >> (8 * port)) & 0xff;
	if ((tegra->calib_data.hs_curr_level_pad[port] + val) >
	    USB2_OTG_HS_CURR_LVL_MAX)
		dev_warn(tegra->dev,
			"0x%X in nvidia,xusb-hs-xcvr-setup-offset too large\n",
			val);
	reg |= min(tegra->calib_data.hs_curr_level_pad[port] + val,
		(u32) USB2_OTG_HS_CURR_LVL_MAX);
	padctl_writel(tegra, reg, ctl0_offset);

	reg = padctl_readl(tegra, ctl1_offset);
	reg &= ~(USB2_OTG_TERM_RANGE_AD | USB2_OTG_HS_IREF_CAP
		| USB2_OTG_PD_CHRP_FORCE_POWERUP
		| USB2_OTG_PD_DISC_FORCE_POWERUP
		| USB2_OTG_PD_DR);
	reg |= (tegra->calib_data.hs_iref_cap << 9) |
		(tegra->calib_data.hs_term_range_adj << 3);
	padctl_writel(tegra, reg, ctl1_offset);
}

static void utmi_pads_release(struct tegra_xusb_phy *tegra)
{
	const struct tegra_xusb_padctl_regs *padregs;
	unsigned long reg;

	if (!tegra->soc_config->release_utmi_in_elpg)
		return;

	padregs = tegra->soc_config->padctl_offsets;
	reg = padctl_readl(tegra, padregs->usb2_pad_mux_0);
	reg &= ~(USB2_OTG_PAD_PORT_MASK(0) | USB2_OTG_PAD_PORT_MASK(1) |
			USB2_OTG_PAD_PORT_MASK(2));
	padctl_writel(tegra, reg, padregs->usb2_pad_mux_0);
}

static void ss_save_context(struct tegra_xusb_phy *tegra, u8 port)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 offset;
	u32 reg;

	padregs = tegra->soc_config->padctl_offsets;
	tegra->ss_ctx_saved |= BIT(port);
	dev_dbg(tegra->dev, "Saving DFE context of port %d\n", port);

	/* If port1 is mapped to SATA lane then read from SATA register */
	if (port == 1 && tegra->soc_config->shared_ss_lanes &&
	    tegra->board_data.lane_owner & BIT(0))
		offset = padregs->iophy_misc_pad_s0_ctlY_0[5];
	else
		offset = padregs->iophy_misc_pad_pX_ctlY_0[port][5];

	reg = padctl_readl(tegra, offset);
	reg &= ~IOPHY_MISC_OUT_SEL(~0);
	reg |= IOPHY_MISC_OUT_SEL(IOPHY_MISC_OUT_SEL_TAP);
	padctl_writel(tegra, reg, offset);

	reg = padctl_readl(tegra, offset);
	tegra->tap1_val[port] = IOPHY_MISC_OUT_TAP_VAL(reg);

	reg = padctl_readl(tegra, offset);
	reg &= ~IOPHY_MISC_OUT_SEL(~0);
	reg |= IOPHY_MISC_OUT_SEL(IOPHY_MISC_OUT_SEL_AMP);
	padctl_writel(tegra, reg, offset);

	reg = padctl_readl(tegra, offset);
	tegra->amp_val[port] = IOPHY_MISC_OUT_AMP_VAL(reg);

	reg = padctl_readl(tegra, padregs->iophy_usb3_padX_ctlY_0[port][3]);
	reg &= ~IOPHY_USB3_DFE_CNTL_TAP_VAL(~0);
	reg |= IOPHY_USB3_DFE_CNTL_TAP_VAL(tegra->tap1_val[port]);
	padctl_writel(tegra, reg, padregs->iophy_usb3_padX_ctlY_0[port][3]);

	reg = padctl_readl(tegra, padregs->iophy_usb3_padX_ctlY_0[port][3]);
	reg &= ~IOPHY_USB3_DFE_CNTL_AMP_VAL(~0);
	reg |= IOPHY_USB3_DFE_CNTL_AMP_VAL(tegra->amp_val[port]);
	padctl_writel(tegra, reg, padregs->iophy_usb3_padX_ctlY_0[port][3]);

	if (!tegra->soc_config->save_ctle_context)
		return;

	dev_dbg(tegra->dev, "Saving restore CTLE context of port %d\n", port);

	reg = padctl_readl(tegra, offset);
	reg &= ~IOPHY_MISC_OUT_SEL(~0);
	reg |= IOPHY_MISC_OUT_SEL(IOPHY_MISC_OUT_SEL_LATCH_G_Z);
	padctl_writel(tegra, reg, offset);

	reg = padctl_readl(tegra, offset);
	reg &= ~IOPHY_MISC_OUT_SEL(~0);
	reg |= IOPHY_MISC_OUT_SEL(IOPHY_MISC_OUT_SEL_G_Z);
	padctl_writel(tegra, reg, offset);

	reg = padctl_readl(tegra, offset);
	tegra->ctle_g_val[port] = IOPHY_MISC_OUT_G_Z_VAL(reg);

	reg = padctl_readl(tegra, offset);
	reg &= ~IOPHY_MISC_OUT_SEL(~0);
	reg |= IOPHY_MISC_OUT_SEL(IOPHY_MISC_OUT_SEL_CTLE_Z);
	padctl_writel(tegra, reg, offset);

	reg = padctl_readl(tegra, offset);
	tegra->ctle_z_val[port] = IOPHY_MISC_OUT_G_Z_VAL(reg);

	reg = padctl_readl(tegra, padregs->iophy_usb3_padX_ctlY_0[port][1]);
	reg &= ~IOPHY_USB3_RX_EQ_Z_VAL(~0);
	reg |= IOPHY_USB3_RX_EQ_Z_VAL(tegra->ctle_z_val[port]);
	padctl_writel(tegra, reg, padregs->iophy_usb3_padX_ctlY_0[port][1]);

	reg = padctl_readl(tegra, padregs->iophy_usb3_padX_ctlY_0[port][1]);
	reg &= ~IOPHY_USB3_RX_EQ_G_VAL(~0);
	reg |= IOPHY_USB3_RX_EQ_G_VAL(tegra->ctle_g_val[port]);
	padctl_writel(tegra, reg, padregs->iophy_usb3_padX_ctlY_0[port][1]);
}

static void ss_restore_context(struct tegra_xusb_phy *tegra, u8 port)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 reg;

	/* Don't restore if not saved */
	if (!(tegra->ss_ctx_saved & BIT(port)))
		return;

	padregs = tegra->soc_config->padctl_offsets;

	dev_dbg(tegra->dev, "Restoring DFE context of port %d\n", port);
	reg = padctl_readl(tegra, padregs->iophy_usb3_padX_ctlY_0[port][3]);
	reg &= ~(IOPHY_USB3_DFE_CNTL_AMP_VAL(~0) |
		 IOPHY_USB3_DFE_CNTL_TAP_VAL(~0));
	reg |= IOPHY_USB3_DFE_CNTL_AMP_VAL(tegra->amp_val[port]) |
		IOPHY_USB3_DFE_CNTL_TAP_VAL(tegra->tap1_val[port]);
	padctl_writel(tegra, reg, padregs->iophy_usb3_padX_ctlY_0[port][3]);

	if (!tegra->soc_config->save_ctle_context)
		return;

	dev_dbg(tegra->dev, "Restoring CTLE context of port %d\n", port);
	reg = padctl_readl(tegra, padregs->iophy_usb3_padX_ctlY_0[port][1]);
	reg &= ~(IOPHY_USB3_RX_EQ_Z_VAL(~0) | IOPHY_USB3_RX_EQ_G_VAL(~0));
	reg |= (IOPHY_USB3_RX_EQ_Z_VAL(tegra->ctle_z_val[port]) |
		IOPHY_USB3_RX_EQ_G_VAL(tegra->ctle_g_val[port]));
	padctl_writel(tegra, reg, padregs->iophy_usb3_padX_ctlY_0[port][1]);
}

static inline bool use_sata_lane(struct tegra_xusb_phy *tegra)
{
	return tegra->soc_config->shared_ss_lanes &&
		(tegra->board_data.lane_owner & BIT(0)) &&
		(tegra->board_data.ss_pads & BIT(1));
}

static void ss_set_clamp(struct tegra_xusb_phy *tegra, bool on)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 elpg_program0;
	int i;

	padregs = tegra->soc_config->padctl_offsets;
	/* Assert/Deassert clamp_en_early signals to SSP0/1 */
	elpg_program0 = padctl_readl(tegra, padregs->elpg_program_0);
	for_each_set_bit(i, &tegra->board_data.ss_pads, TEGRA_XUSB_SS_COUNT) {
		if (on)
			elpg_program0 |= SSP_ELPG_CLAMP_EN_EARLY(i);
		else
			elpg_program0 &= ~SSP_ELPG_CLAMP_EN_EARLY(i);
	}
	padctl_writel(tegra, elpg_program0, padregs->elpg_program_0);

	/*
	 * Check the LP0 figure and leave gap bw writes to
	 * clamp_en_early and clamp_en
	 */
	udelay(100);

	/* Assert/Deassert clam_en signal */
	elpg_program0 = padctl_readl(tegra, padregs->elpg_program_0);
	for_each_set_bit(i, &tegra->board_data.ss_pads, TEGRA_XUSB_SS_COUNT) {
		if (on)
			elpg_program0 |= SSP_ELPG_CLAMP_EN(i);
		else
			elpg_program0 &= ~SSP_ELPG_CLAMP_EN(i);
	}
	padctl_writel(tegra, elpg_program0, padregs->elpg_program_0);

	/* Wait for 250us for the writes to propogate */
	if (on)
		udelay(250);
}

static void ss_set_vcore(struct tegra_xusb_phy *tegra, bool on)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 elpg_program0;
	int i;

	padregs = tegra->soc_config->padctl_offsets;
	/* Assert vcore_off signal */
	elpg_program0 = padctl_readl(tegra, padregs->elpg_program_0);
	for_each_set_bit(i, &tegra->board_data.ss_pads, TEGRA_XUSB_SS_COUNT) {
		if (on)
			elpg_program0 &= ~SSP_ELPG_VCORE_DOWN(i);
		else
			elpg_program0 |= SSP_ELPG_VCORE_DOWN(i);
	}
	padctl_writel(tegra, elpg_program0, padregs->elpg_program_0);
}

static void ss_rx_idle_mode_override(struct tegra_xusb_phy *tegra, bool enable)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 reg, offset;
	int i;

	if (tegra->soc_config->shared_ss_lanes)
		return;

	padregs = tegra->soc_config->padctl_offsets;
	for_each_set_bit(i, &tegra->board_data.ss_pads, TEGRA_XUSB_SS_COUNT) {
		offset = padregs->iophy_misc_pad_pX_ctlY_0[i][2];
		reg = padctl_readl(tegra, offset);
		if (enable) {
			reg &= ~IOPHY_RX_IDLE_MODE;
			reg |= IOPHY_RX_IDLE_MODE_OVRD;
		} else {
			reg |= IOPHY_RX_IDLE_MODE;
			reg &= ~IOPHY_RX_IDLE_MODE_OVRD;
		}
		padctl_writel(tegra, reg, offset);
	}
}

static int ss_set_clock_rate(struct tegra_xusb_phy *tegra, unsigned int rate)
{
	unsigned int new_parent_rate, old_parent_rate, div;
	int ret;
	struct clk *ss_clk = tegra->ss_src_clk;

	if (clk_get_rate(ss_clk) == rate)
		return 0;

	switch (rate) {
	case SS_CLK_HIGH_SPEED:
		/* Reparent to PLLU_480M. Set div first to avoid overclocking */
		old_parent_rate = clk_get_rate(clk_get_parent(ss_clk));
		new_parent_rate = clk_get_rate(tegra->pll_u_480M);
		div = new_parent_rate / rate;
		ret = clk_set_rate(ss_clk, old_parent_rate / div);
		if (ret) {
			dev_err(tegra->dev, "Failed to set SS rate: %d\n",
				ret);
			return ret;
		}
		ret = clk_set_parent(ss_clk, tegra->pll_u_480M);
		if (ret) {
			dev_err(tegra->dev, "Failed to set SS parent: %d\n",
				ret);
			return ret;
		}
		ss_rx_idle_mode_override(tegra, false);
		break;
	case SS_CLK_LOW_SPEED:
		/* Reparent to CLK_M */
		ret = clk_set_parent(ss_clk, tegra->clk_m);
		if (ret) {
			dev_err(tegra->dev, "Failed to set SS parent: %d\n",
				ret);
			return ret;
		}
		ret = clk_set_rate(ss_clk, rate);
		if (ret) {
			dev_err(tegra->dev, "Failed to set SS rate: %d\n",
				ret);
			return ret;
		}
		ss_rx_idle_mode_override(tegra, true);
		break;
	default:
		dev_err(tegra->dev, "Invalid SS rate: %u\n", rate);
		return -EINVAL;
	}

	if (clk_get_rate(ss_clk) != rate) {
		dev_err(tegra->dev, "SS clock doesn't match requested rate\n");
		return -EINVAL;
	}

	return 0;
}

static void ss_pad_init(struct tegra_xusb_phy *tegra, u8 port)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 ctl2_offset, ctl4_offset, misc_ctl5_offset, misc_ctl2_offset;
	u32 reg;

	padregs = tegra->soc_config->padctl_offsets;
	ctl2_offset = padregs->iophy_usb3_padX_ctlY_0[port][1];
	ctl4_offset = padregs->iophy_usb3_padX_ctlY_0[port][3];
	misc_ctl5_offset = padregs->iophy_misc_pad_pX_ctlY_0[port][4];
	misc_ctl2_offset = padregs->iophy_misc_pad_pX_ctlY_0[port][1];

	reg = padctl_readl(tegra, ctl2_offset);
	reg &= ~(IOPHY_USB3_RX_WANDER_VAL(~0) | IOPHY_USB3_RX_EQ_VAL(~0) |
		 IOPHY_USB3_CDR_CNTL_VAL(~0));
	reg |= tegra->soc_config->rx_wander | tegra->soc_config->rx_eq |
		tegra->soc_config->cdr_cntl;
	padctl_writel(tegra, reg, ctl2_offset);

	padctl_writel(tegra, tegra->soc_config->dfe_cntl, ctl4_offset);

	reg = padctl_readl(tegra, misc_ctl5_offset);
	reg |= IOPHY_RX_QEYE_EN;
	padctl_writel(tegra, reg, misc_ctl5_offset);

	reg = padctl_readl(tegra, misc_ctl2_offset);
	reg &= ~IOPHY_SPARE_IN(~0);
	reg |= IOPHY_SPARE_IN(tegra->soc_config->spare_in);
	padctl_writel(tegra, reg, misc_ctl2_offset);

	if (use_sata_lane(tegra)) {
		reg = padctl_readl(tegra, padregs->iophy_misc_pad_s0_ctlY_0[4]);
		reg |= IOPHY_RX_QEYE_EN;
		padctl_writel(tegra, reg, padregs->iophy_misc_pad_s0_ctlY_0[4]);

		reg = padctl_readl(tegra, padregs->iophy_misc_pad_s0_ctlY_0[1]);
		reg &= ~IOPHY_SPARE_IN(~0);
		reg |= IOPHY_SPARE_IN(tegra->soc_config->spare_in);
		padctl_writel(tegra, reg, padregs->iophy_misc_pad_s0_ctlY_0[1]);
	}

	reg = padctl_readl(tegra, padregs->ss_port_map_0);
	reg &= ~SS_PORT_MAP_P(port);
	reg |= tegra->board_data.ss_portmap & SS_PORT_MAP_P(port);
	padctl_writel(tegra, reg, padregs->ss_port_map_0);

	ss_restore_context(tegra, port);
}

static void ss_lanes_init(struct tegra_xusb_phy *tegra)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 lane_owner = tegra->board_data.lane_owner;
	u32 val;

	if (!tegra->soc_config->shared_ss_lanes)
		return;

	padregs = tegra->soc_config->padctl_offsets;
	/* Program SATA pad phy */
	if (lane_owner & BIT(0)) {
		val = padctl_readl(tegra, padregs->iophy_pll_s0_ctlY_0[0]);
		val &= ~IOPHY_PLL_PLL0_REFCLK_NDIV(~0);
		val |= IOPHY_PLL_PLL0_REFCLK_NDIV(0x2);
		padctl_writel(tegra, val, padregs->iophy_pll_s0_ctlY_0[0]);

		val = padctl_readl(tegra, padregs->iophy_pll_s0_ctlY_0[1]);
		val &= ~(IOPHY_PLL_XDIGCLK_SEL(~0) | IOPHY_PLL_TXCLKREF_SEL |
			 IOPHY_PLL_TCLKOUT_EN | IOPHY_PLL_PLL0_CP_CNTL(~0) |
			 IOPHY_PLL_PLL1_CP_CNTL(~0));
		val |= IOPHY_PLL_XDIGCLK_SEL(0x7) | IOPHY_PLL_TXCLKREF_SEL |
			IOPHY_PLL_PLL0_CP_CNTL(0x8) |
			IOPHY_PLL_PLL1_CP_CNTL(0x8);
		padctl_writel(tegra, val, padregs->iophy_pll_s0_ctlY_0[1]);

		val = padctl_readl(tegra, padregs->iophy_pll_s0_ctlY_0[2]);
		val &= ~IOPHY_PLL_RCAL_BYPASS;
		padctl_writel(tegra, val, padregs->iophy_pll_s0_ctlY_0[2]);

		/* Enable SATA PADPLL clocks */
		val = clkrst_readl(tegra, SATA_PLL_CFG0_0);
		val &= ~SATA_PADPLL_RESET_SWCTL;
		val |= SATA_PADPLL_USE_LOCKDET | SATA_SEQ_START_STATE;
		clkrst_writel(tegra, val, SATA_PLL_CFG0_0);

		udelay(1);

		val = clkrst_readl(tegra, SATA_PLL_CFG0_0);
		val |= SATA_SEQ_ENABLE;
		clkrst_writel(tegra, val, SATA_PLL_CFG0_0);
	}

	/*
	 * Program ownership of lanes owned by USB3 based on lane_owner[2:0]
	 * lane_owner[0] = 0 (SATA lane owner = SATA),
	 * lane_owner[0] = 1 (SATA lane owner = USB3_SS port1)
	 * lane_owner[1] = 0 (PCIe lane0 owner = PCIe),
	 * lane_owner[1] = 1 (PCIe lane0 owner = USB3_SS port0)
	 * lane_owner[2] = 0 (PCIe lane1 owner = PCIe),
	 * lane_owner[2] = 1 (PCIe lane1 owner = USB3_SS port1)
	 */
	val = padctl_readl(tegra, padregs->usb3_pad_mux_0);
	/* USB3_SS port1 can either be mapped to SATA lane or PCIe lane1 */
	if (lane_owner & BIT(0)) {
		val &= ~USB3_SATA_PAD_LANE_OWNER(~0);
		val |= USB3_SATA_PAD_LANE_OWNER(USB3_LANE_OWNER_USB3_SS);
	} else if (lane_owner & BIT(2)) {
		val &= ~USB3_PCIE_PAD_LANE_OWNER(1, ~0);
		val |= USB3_PCIE_PAD_LANE_OWNER(1, USB3_LANE_OWNER_USB3_SS);
	}
	/* USB3_SS port0 is always mapped to PCIe lane0 */
	if (lane_owner & BIT(1)) {
		val &= ~USB3_PCIE_PAD_LANE_OWNER(0, ~0);
		val |= USB3_PCIE_PAD_LANE_OWNER(0, USB3_LANE_OWNER_USB3_SS);
	}
	padctl_writel(tegra, val, padregs->usb3_pad_mux_0);

	/* Bring enabled lane out of IDDQ */
	val = padctl_readl(tegra, padregs->usb3_pad_mux_0);
	if (lane_owner & BIT(0))
		val |= USB3_FORCE_SATA_PAD_IDDQ_DISABLE_MASK;
	else if (lane_owner & BIT(2))
		val |= USB3_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK(1);
	if (lane_owner & BIT(1))
		val |= USB3_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK(0);
	padctl_writel(tegra, val, padregs->usb3_pad_mux_0);

	udelay(1);

	/* Clear AUX_MUX_LP0 related bits in ELPG_PROGRAM */
	val = padctl_readl(tegra, padregs->elpg_program_0);
	val &= ~AUX_MUX_LP0_CLAMP_EN;
	padctl_writel(tegra, val, padregs->elpg_program_0);

	udelay(100);

	val &= ~AUX_MUX_LP0_CLAMP_EN_EARLY;
	padctl_writel(tegra, val, padregs->elpg_program_0);

	udelay(100);

	val &= ~AUX_MUX_LP0_VCORE_DOWN;
	padctl_writel(tegra, val, padregs->elpg_program_0);
}

/*
 * Assign the USB ports to the controllers, then programs the port
 * capabilities and pad parameters.
 */
static void init_ports(struct tegra_xusb_phy *tegra)
{
	const struct tegra_xusb_padctl_regs *padregs;
	u32 reg, offset;
	unsigned long ss_pads = tegra->board_data.ss_pads;
	unsigned long hsic_pads = tegra->board_data.hsic_pads;
	unsigned long utmi_pads = tegra->board_data.utmi_pads;
	int i;

	padregs = tegra->soc_config->padctl_offsets;
	reg = padctl_readl(tegra, padregs->usb2_bias_pad_ctlY_0[0]);
	reg &= ~(USB2_BIAS_HS_SQUELCH_LEVEL | USB2_BIAS_HS_DISCON_LEVEL);
	reg |= tegra->calib_data.hs_squelch_level |
		tegra->soc_config->hs_disc_lvl;
	padctl_writel(tegra, reg, padregs->usb2_bias_pad_ctlY_0[0]);

	for_each_set_bit(i, &utmi_pads, TEGRA_XUSB_UTMI_COUNT)
		utmi_pad_init(tegra, i);

	for_each_set_bit(i, &hsic_pads, TEGRA_XUSB_HSIC_COUNT)
		hsic_pad_enable(tegra, i);

	for (i = 0; i < TEGRA_XUSB_SS_COUNT; i++) {
		if (ss_pads & BIT(i)) {
			ss_pad_init(tegra, i);
		} else {
			/*
			 * Set rx_idle_mode_ovrd for unused SS ports to
			 * save power
			 */
			offset = padregs->iophy_misc_pad_pX_ctlY_0[i][2];
			reg = padctl_readl(tegra, offset);
			reg &= ~IOPHY_RX_IDLE_MODE;
			reg |= IOPHY_RX_IDLE_MODE_OVRD;
			padctl_writel(tegra, reg, offset);
			/*
			 * SATA lane also if USB3_SS port1 mapped to it but
			 * unused
			 */
			if (i == 1 && tegra->soc_config->shared_ss_lanes &&
			    (tegra->board_data.lane_owner & BIT(0))) {
				offset = padregs->iophy_misc_pad_s0_ctlY_0[2];
				reg = padctl_readl(tegra, offset);
				reg &= ~IOPHY_RX_IDLE_MODE;
				reg |= IOPHY_RX_IDLE_MODE_OVRD;
				padctl_writel(tegra, reg, offset);
			}
		}
	}
	ss_lanes_init(tegra);
}

static void hsic_pmc_wake_enable(struct tegra_xusb_phy *tegra, unsigned int pad)
{
	u32 val, port;

	if (!tegra_xhci_port_connected(tegra->xhci, pad +
				       tegra->soc_config->hsic_port_offset))
		return;

	port = pad + 1;
	val = pmc_readl(tegra, PMC_UHSIC_SLEEP_CFG(port));
	if (val & UHSIC_MASTER_ENABLE(port)) {
		dev_info(tegra->dev, "HSIC wake already enabled on port %d\n",
			 port);
		return;
	}

	/*
	 * Set PMC MASTER bits to do the following:
	 * a. Take over the UHSIC drivers
	 * b. Take over resume if remote wakeup is detected
	 * c. Take over suspend-wake detect-drive resume until USB controller
	 *    ready.
	 */

	/* disable master enable in PMC */
	val = pmc_readl(tegra, PMC_UHSIC_SLEEP_CFG(port));
	val &= ~UHSIC_MASTER_ENABLE(port);
	pmc_writel(tegra, val, PMC_UHSIC_SLEEP_CFG(port));

	/* UTMIP_PWR_PX=1 for power savings mode */
	val = pmc_readl(tegra, PMC_UHSIC_MASTER_CONFIG(port));
	val |= UHSIC_PWR(port);
	pmc_writel(tegra, val, PMC_UHSIC_MASTER_CONFIG(port));

	/* config debouncer */
	val = pmc_readl(tegra, PMC_USB_DEBOUNCE);
	val |= PMC_USB_DEBOUNCE_VAL(2);
	pmc_writel(tegra, val, PMC_USB_DEBOUNCE);

	/* Make sure nothing is happening on the line with respect to PMC */
	val = pmc_readl(tegra, PMC_UHSIC_FAKE(port));
	val &= ~UHSIC_FAKE_STROBE_VAL(port);
	val &= ~UHSIC_FAKE_DATA_VAL(port);
	pmc_writel(tegra, val, PMC_UHSIC_FAKE(port));

	/* Clear walk enable */
	val = pmc_readl(tegra, PMC_UHSIC_SLEEPWALK_CFG(port));
	val &= ~UHSIC_LINEVAL_WALK_EN(port);
	pmc_writel(tegra, val, PMC_UHSIC_SLEEPWALK_CFG(port));

	/* Make sure wake value for line is none */
	val = pmc_readl(tegra, PMC_UHSIC_SLEEP_CFG(port));
	val &= ~UHSIC_WAKE_VAL(port, WAKE_VAL_ANY);
	val |= UHSIC_WAKE_VAL(port, WAKE_VAL_NONE);
	pmc_writel(tegra, val, PMC_UHSIC_SLEEP_CFG(port));

	/* turn on pad detectors */
	val = pmc_readl(tegra, PMC_USB_AO);
	val &= ~(STROBE_VAL_PD(port) | DATA_VAL_PD(port));
	pmc_writel(tegra, val, PMC_USB_AO);

	/* Add small delay before usb detectors provide stable line values */
	udelay(1);

	/*
	 * Enable which type of event can trigger a walk, in this case
	 * usb_line_wake
	 */
	val = pmc_readl(tegra, PMC_UHSIC_SLEEPWALK_CFG(port));
	val |= UHSIC_LINEVAL_WALK_EN(port);
	pmc_writel(tegra, val, PMC_UHSIC_SLEEPWALK_CFG(port));

	/*
	 * Program walk sequence: maintain a J, followed by a driven K
	 * to signal a resume once an wake event is detected
	 */
	val = pmc_readl(tegra, PMC_SLEEPWALK_UHSIC(port));
	val &= ~UHSIC_DATA_RPU_A;
	val |=  UHSIC_DATA_RPD_A;
	val &= ~UHSIC_STROBE_RPD_A;
	val |=  UHSIC_STROBE_RPU_A;

	val &= ~UHSIC_DATA_RPD_B;
	val |=  UHSIC_DATA_RPU_B;
	val &= ~UHSIC_STROBE_RPU_B;
	val |=  UHSIC_STROBE_RPD_B;

	val &= ~UHSIC_DATA_RPD_C;
	val |=  UHSIC_DATA_RPU_C;
	val &= ~UHSIC_STROBE_RPU_C;
	val |=  UHSIC_STROBE_RPD_C;

	val &= ~UHSIC_DATA_RPD_D;
	val |=  UHSIC_DATA_RPU_D;
	val &= ~UHSIC_STROBE_RPU_D;
	val |=  UHSIC_STROBE_RPD_D;
	pmc_writel(tegra, val, PMC_SLEEPWALK_UHSIC(port));

	/* Set wake event */
	val = pmc_readl(tegra, PMC_UHSIC_SLEEP_CFG(port));
	val &= ~UHSIC_WAKE_VAL(port, WAKE_VAL_ANY);
	val |= UHSIC_WAKE_VAL(port, WAKE_VAL_SD10);
	pmc_writel(tegra, val, PMC_UHSIC_SLEEP_CFG(port));

	/* Clear the walk pointers and wake alarm */
	val = pmc_readl(tegra, PMC_UHSIC_TRIGGERS(port));
	val |= UHSIC_CLR_WAKE_ALARM(port) | UHSIC_CLR_WALK_PTR(port);
	pmc_writel(tegra, val, PMC_UHSIC_TRIGGERS(port));

	/* Turn over pad configuration to PMC for line wake events*/
	val = pmc_readl(tegra, PMC_UHSIC_SLEEP_CFG(port));
	val |= UHSIC_MASTER_ENABLE(port);
	pmc_writel(tegra, val, PMC_UHSIC_SLEEP_CFG(port));
}

static void hsic_pmc_wake_disable(struct tegra_xusb_phy *tegra,
				  unsigned int pad)
{
	u32 val, port;

	port = pad + 1;
	val = pmc_readl(tegra, PMC_UHSIC_SLEEP_CFG(port));
	if (!(val & UHSIC_MASTER_ENABLE(port))) {
		dev_info(tegra->dev, "HSIC wake already disabled on port %d\n",
			 port);
		return;
	}

	val = pmc_readl(tegra, PMC_UHSIC_SLEEP_CFG(port));
	val &= ~UHSIC_WAKE_VAL(port, WAKE_VAL_ANY);
	val |= UHSIC_WAKE_VAL(port, WAKE_VAL_NONE);
	pmc_writel(tegra, val, PMC_UHSIC_SLEEP_CFG(port));

	/* Disable PMC master mode by clearing MASTER_EN */
	val = pmc_readl(tegra, PMC_UHSIC_SLEEP_CFG(port));
	val &= ~(UHSIC_MASTER_ENABLE(port));
	pmc_writel(tegra, val, PMC_UHSIC_SLEEP_CFG(port));

	/* Turn off pad detectors */
	val = pmc_readl(tegra, PMC_USB_AO);
	val |= (STROBE_VAL_PD(port) | DATA_VAL_PD(port));
	pmc_writel(tegra, val, PMC_USB_AO);

	val = pmc_readl(tegra, PMC_UHSIC_TRIGGERS(port));
	val |= (UHSIC_CLR_WALK_PTR(port) | UHSIC_CLR_WAKE_ALARM(port));
	pmc_writel(tegra, val, PMC_UHSIC_TRIGGERS(port));
}

static void utmi_pmc_wake_enable(struct tegra_xusb_phy *tegra, unsigned int pad)
{
	u32 val, pmc_pad_cfg_val, port;
	enum usb_device_speed port_speed;

	port = tegra->soc_config->pmc_portmap[pad];
	port_speed = tegra_xhci_port_speed(tegra->xhci, pad +
					   tegra->soc_config->utmi_port_offset);
	val = pmc_readl(tegra, PMC_SLEEP_CFG);
	if (val & UTMIP_MASTER_ENABLE(port)) {
		dev_info(tegra->dev, "UTMI wake already enabled on port %d\n",
			 port);
		return;
	}

	/*
	 * Set PMC MASTER bits to do the following:
	 * a. Take over the UTMI drivers
	 * b. Take over resume if remote wakeup is detected
	 * c. Take over suspend-wake detect-drive resume until USB controller
	 *    ready.
	 */

	/* Disable master enable in PMC */
	val = pmc_readl(tegra, PMC_SLEEP_CFG);
	val &= ~UTMIP_MASTER_ENABLE(port);
	pmc_writel(tegra, val, PMC_SLEEP_CFG);

	/* UTMIP_PWR_PX=1 for power savings mode */
	val = pmc_readl(tegra, PMC_UTMIP_MASTER_CONFIG);
	val |= UTMIP_PWR(port);
	pmc_writel(tegra, val, PMC_UTMIP_MASTER_CONFIG);

	/* Config debouncer */
	val = pmc_readl(tegra, PMC_USB_DEBOUNCE);
	val &= ~UTMIP_LINE_DEB_CNT(~0);
	val |= UTMIP_LINE_DEB_CNT(1);
	val |= PMC_USB_DEBOUNCE_VAL(2);
	pmc_writel(tegra, val, PMC_USB_DEBOUNCE);

	/* Make sure nothing is happening on the line with respect to PMC */
	val = pmc_readl(tegra, PMC_UTMIP_FAKE);
	val &= ~USBOP_VAL(port);
	val &= ~USBON_VAL(port);
	pmc_writel(tegra, val, PMC_UTMIP_FAKE);

	/* Make sure wake value for line is none */
	val = pmc_readl(tegra, PMC_SLEEPWALK_CFG);
	val &= ~UTMIP_LINEVAL_WALK_EN(port);
	pmc_writel(tegra, val, PMC_SLEEPWALK_CFG);
	val = pmc_readl(tegra, PMC_SLEEP_CFG);
	val &= ~UTMIP_WAKE_VAL(port, ~0);
	val |= UTMIP_WAKE_VAL(port, WAKE_VAL_NONE);
	pmc_writel(tegra, val, PMC_SLEEP_CFG);

	/* turn off pad detectors */
	val = pmc_readl(tegra, PMC_USB_AO);
	val |= (USBOP_VAL_PD(port) | USBON_VAL_PD(port));
	pmc_writel(tegra, val, PMC_USB_AO);

	/* Remove fake values and make synchronizers work a bit */
	val = pmc_readl(tegra, PMC_UTMIP_FAKE);
	val &= ~USBOP_VAL(port);
	val &= ~USBON_VAL(port);
	pmc_writel(tegra, val, PMC_UTMIP_FAKE);

	/*
	 * Enable which type of event can trigger a walk, in this case
	 * usb_line_wake
	 */
	val = pmc_readl(tegra, PMC_SLEEPWALK_CFG);
	val |= UTMIP_LINEVAL_WALK_EN(port);
	pmc_writel(tegra, val, PMC_SLEEPWALK_CFG);

	/* Capture FS/LS pad configurations */
	pmc_pad_cfg_val = pmc_readl(tegra, PMC_PAD_CFG);
	val = pmc_readl(tegra, PMC_TRIGGERS);
	val |= UTMIP_CAP_CFG(port);
	pmc_writel(tegra, val, PMC_TRIGGERS);
	udelay(1);
	pmc_pad_cfg_val = pmc_readl(tegra, PMC_PAD_CFG);

	/* BIAS MASTER_ENABLE=0 */
	val = pmc_readl(tegra, PMC_UTMIP_BIAS_MASTER_CNTRL);
	val &= ~BIAS_MASTER_PROG_VAL;
	pmc_writel(tegra, val, PMC_UTMIP_BIAS_MASTER_CNTRL);

	/* Program walk sequence for remote or hotplug wakeup */
	if (port_speed > USB_SPEED_UNKNOWN) {
		/*
		 * Program walk sequence: maintain a J, followed by a driven K
		 * to signal a resume once an wake event is detected
		 */
		val = pmc_readl(tegra, PMC_SLEEPWALK_REG(port));
		val &= ~UTMIP_AP_A;
		val |= UTMIP_USBOP_RPD_A | UTMIP_USBON_RPD_A | UTMIP_AN_A |
					UTMIP_HIGHZ_A |
			UTMIP_USBOP_RPD_B | UTMIP_USBON_RPD_B | UTMIP_AP_B |
					UTMIP_AN_B |
			UTMIP_USBOP_RPD_C | UTMIP_USBON_RPD_C | UTMIP_AP_C |
					UTMIP_AN_C |
			UTMIP_USBOP_RPD_D | UTMIP_USBON_RPD_D | UTMIP_AP_D |
					UTMIP_AN_D;
		pmc_writel(tegra, val, PMC_SLEEPWALK_REG(port));

		if (port_speed == USB_SPEED_LOW) {
			val = pmc_readl(tegra, PMC_SLEEPWALK_REG(port));
			val &= ~(UTMIP_AN_B | UTMIP_HIGHZ_B | UTMIP_AN_C |
				UTMIP_HIGHZ_C | UTMIP_AN_D | UTMIP_HIGHZ_D);
			pmc_writel(tegra, val, PMC_SLEEPWALK_REG(port));
		} else {
			val = pmc_readl(tegra, PMC_SLEEPWALK_REG(port));
			val &= ~(UTMIP_AP_B | UTMIP_HIGHZ_B | UTMIP_AP_C |
				UTMIP_HIGHZ_C | UTMIP_AP_D | UTMIP_HIGHZ_D |
				UTMIP_AN_A);
				val |= UTMIP_AP_A;
			pmc_writel(tegra, val, PMC_SLEEPWALK_REG(port));
		}
	} else {
		/*
		 * Program walk sequence: pull down both dp and dn lines,
		 * tristate lines once a hotplug-in wake event is detected
		 */
		val = pmc_readl(tegra, PMC_SLEEPWALK_REG(port));
		val |= UTMIP_USBOP_RPD_A | UTMIP_USBON_RPD_A | UTMIP_HIGHZ_A;
		val &= ~UTMIP_AP_A;
		val &= ~UTMIP_AN_A;
		val |= UTMIP_USBOP_RPD_B | UTMIP_USBON_RPD_B | UTMIP_HIGHZ_B;
		val &= ~UTMIP_AP_B;
		val &= ~UTMIP_AN_B;
		val |= UTMIP_USBOP_RPD_C | UTMIP_USBON_RPD_C | UTMIP_HIGHZ_C;
		val &= ~UTMIP_AP_C;
		val &= ~UTMIP_AN_C;
		val |= UTMIP_USBOP_RPD_D | UTMIP_USBON_RPD_D | UTMIP_HIGHZ_D;
		val &= ~UTMIP_AP_D;
		val &= ~UTMIP_AN_D;
		pmc_writel(tegra, val, PMC_SLEEPWALK_REG(port));
	}

	/* Turn on pad detectors */
	val = pmc_readl(tegra, PMC_USB_AO);
	val &= ~(USBOP_VAL_PD(port) | USBON_VAL_PD(port));
	pmc_writel(tegra, val, PMC_USB_AO);

	/* Add small delay before usb detectors provide stable line values */
	usleep_range(1000, 1100);

	/* Program thermally encoded RCTRL_VAL, TCTRL_VAL into PMC space */
	if (tegra->utmip_tctrl_val | tegra->utmip_rctrl_val) {
		val = pmc_readl(tegra, PMC_UTMIP_TERM_PAD_CFG);
		val = PMC_TCTRL_VAL(tegra->utmip_tctrl_val) |
			PMC_RCTRL_VAL(tegra->utmip_rctrl_val);
		pmc_writel(tegra, val, PMC_UTMIP_TERM_PAD_CFG);
	}

	/* Turn over pad configuration to PMC for line wake events */
	val = pmc_readl(tegra, PMC_SLEEP_CFG);
	val &= ~UTMIP_WAKE_VAL(port, ~0);
	if (device_may_wakeup(tegra->dev)) {
		val |= UTMIP_WAKE_VAL(port, WAKE_VAL_ANY);
	} else if (tegra_xhci_port_may_wakeup(tegra->xhci, pad)) {
		switch (port_speed) {
		case USB_SPEED_LOW:
			val |= UTMIP_WAKE_VAL(port, WAKE_VAL_FSJ);
			break;
		case USB_SPEED_FULL:
		case USB_SPEED_HIGH:
			val |= UTMIP_WAKE_VAL(port, WAKE_VAL_FSK);
			break;
		default:
			val |= UTMIP_WAKE_VAL(port, WAKE_VAL_NONE);
		}
	} else {
		val |= UTMIP_WAKE_VAL(port, WAKE_VAL_NONE);
	}
	pmc_writel(tegra, val, PMC_SLEEP_CFG);
	val |= UTMIP_RCTRL_USE_PMC(port) | UTMIP_TCTRL_USE_PMC(port);
	val |= UTMIP_MASTER_ENABLE(port) | UTMIP_FSLS_USE_PMC(port);
	pmc_writel(tegra, val, PMC_SLEEP_CFG);
}

static void utmi_pmc_wake_disable(struct tegra_xusb_phy *tegra,
				  unsigned int pad)
{
	u32 val, port;

	port = tegra->soc_config->pmc_portmap[pad];
	val = pmc_readl(tegra, PMC_SLEEP_CFG);
	if (!(val & UTMIP_MASTER_ENABLE(port))) {
		dev_info(tegra->dev, "UTMI wake already disabled on port %d\n",
			 port);
		return;
	}

	val = pmc_readl(tegra, PMC_SLEEP_CFG);
	val &= ~UTMIP_WAKE_VAL(port, 0xF);
	val |= UTMIP_WAKE_VAL(port, WAKE_VAL_NONE);
	pmc_writel(tegra, val, PMC_SLEEP_CFG);

	/* Disable PMC master mode by clearing MASTER_EN */
	val = pmc_readl(tegra, PMC_SLEEP_CFG);
	val |= UTMIP_RCTRL_USE_PMC(port) | UTMIP_TCTRL_USE_PMC(port);
	val &= ~(UTMIP_FSLS_USE_PMC(port) | UTMIP_MASTER_ENABLE(port));
	pmc_writel(tegra, val, PMC_SLEEP_CFG);

	val = pmc_readl(tegra, PMC_TRIGGERS);
	val &= ~UTMIP_CAP_CFG(port);
	pmc_writel(tegra, val, PMC_TRIGGERS);

	/* turn off pad detectors */
	val = pmc_readl(tegra, PMC_USB_AO);
	val |= (USBOP_VAL_PD(port) | USBON_VAL_PD(port));
	pmc_writel(tegra, val, PMC_USB_AO);

	val = pmc_readl(tegra, PMC_TRIGGERS);
	val |= UTMIP_CLR_WALK_PTR(port);
	val |= UTMIP_CLR_WAKE_ALARM(port);
	pmc_writel(tegra, val, PMC_TRIGGERS);
}

static void pmc_wake_enable(struct tegra_xusb_phy *tegra)
{
	unsigned long utmi_pads, hsic_pads;
	int pad;

	hsic_pads = tegra->board_data.hsic_pads;
	for_each_set_bit(pad, &hsic_pads, TEGRA_XUSB_HSIC_COUNT)
		hsic_pmc_wake_enable(tegra, pad);

	utmi_pads = tegra->board_data.utmi_pads;
	for_each_set_bit(pad, &utmi_pads, TEGRA_XUSB_UTMI_COUNT)
		utmi_pmc_wake_enable(tegra, pad);
}

static void pmc_wake_disable(struct tegra_xusb_phy *tegra)
{
	unsigned long utmi_pads, hsic_pads;
	int pad;

	hsic_pads = tegra->board_data.hsic_pads;
	for_each_set_bit(pad, &hsic_pads, TEGRA_XUSB_HSIC_COUNT)
		hsic_pmc_wake_disable(tegra, pad);

	utmi_pads = tegra->board_data.utmi_pads;
	for_each_set_bit(pad, &utmi_pads, TEGRA_XUSB_UTMI_COUNT)
		utmi_pmc_wake_disable(tegra, pad);
}

static int tegra_xusb_phy_mbox_notifier(struct notifier_block *nb,
					unsigned long event, void *p)
{
	struct tegra_xusb_phy *tegra = container_of(nb, struct tegra_xusb_phy,
						    mbox_nb);
	struct mbox_notifier_data *data = (struct mbox_notifier_data *)p;
	unsigned long ports;
	int i, pad, ret;

	switch (event) {
	case MBOX_CMD_INC_SSPI_CLOCK:
	case MBOX_CMD_DEC_SSPI_CLOCK:
		if (tegra->soc_config->use_hs_src_clk2) {
			data->resp_cmd = MBOX_CMD_ACK;
			data->resp_data = data->msg_data;
			return NOTIFY_STOP;
		}
		ret = ss_set_clock_rate(tegra, data->msg_data * 1000);
		data->resp_data = clk_get_rate(tegra->ss_src_clk) / 1000;
		if (ret)
			data->resp_cmd = MBOX_CMD_NACK;
		else
			data->resp_cmd = MBOX_CMD_ACK;
		return NOTIFY_STOP;
	case MBOX_CMD_SAVE_DFE_CTLE_CTX:
		data->resp_data = data->msg_data;
		if (data->msg_data > TEGRA_XUSB_SS_COUNT) {
			data->resp_cmd = MBOX_CMD_NACK;
		} else {
			ss_save_context(tegra, data->msg_data);
			data->resp_cmd = MBOX_CMD_ACK;
		}
		return NOTIFY_STOP;
	case MBOX_CMD_STAR_HSIC_IDLE:
	case MBOX_CMD_STOP_HSIC_IDLE:
		ports = data->msg_data;
		data->resp_data = ports;
		for_each_set_bit(i, &ports, BITS_PER_LONG) {
			pad = i - 1 - tegra->soc_config->hsic_port_offset;
			if (pad > TEGRA_XUSB_HSIC_COUNT) {
				ret = -EINVAL;
				break;
			}
			if (event == MBOX_CMD_STAR_HSIC_IDLE)
				ret = hsic_pad_set_pupd(tegra, pad, PUPD_IDLE);
			else
				ret = hsic_pad_set_pupd(tegra, pad,
							PUPD_DISABLE);
			if (ret)
				break;
		}
		if (ret)
			data->resp_cmd = MBOX_CMD_NACK;
		else
			data->resp_cmd = MBOX_CMD_ACK;
		return NOTIFY_STOP;
	default:
		return NOTIFY_DONE;
	}
}

static int tegra_xusb_phy_init(struct usb_phy *phy)
{
	struct tegra_xusb_phy *tegra = phy_to_tegra(phy);
	unsigned long pads;
	int i, ret;

	BUG_ON(!tegra->xhci);

	/* Register MBOX command handler */
	ret = tegra_xhci_register_mbox_notifier(tegra->xhci, &tegra->mbox_nb);
	if (ret) {
		dev_err(tegra->dev, "Failed to register handler\n");
		return ret;
	}

	/* Enable VBUS and HSIC supplies */
	pads = tegra->board_data.utmi_pads;
	for_each_set_bit(i, &pads, TEGRA_XUSB_UTMI_COUNT) {
		ret = regulator_enable(tegra->utmi_vbus[i]);
		if (ret) {
			dev_err(tegra->dev, "Failed to enable vbus%d\n", i);
			return ret;
		}
	}
	if (tegra->board_data.hsic_pads) {
		ret = regulator_enable(tegra->vddio_hsic);
		if (ret) {
			dev_err(tegra->dev, "Failed to enable vddio-hsic\n");
			return ret;
		}
	}

	/* Initialize clocks */
	ret = clk_prepare_enable(tegra->ss_clk);
	if (ret) {
		dev_err(tegra->dev, "Failed to enable xusb_ss clock\n");
		return ret;
	}
	ret = ss_set_clock_rate(tegra, SS_CLK_HIGH_SPEED);
	if (ret) {
		dev_err(tegra->dev, "Failed to set xusb_ss rate\n");
		return ret;
	}

	/* Initialize pads and map them to XUSB controller */
	if (tegra->soc_config->recalc_tctrl_rctrl)
		utmi_calc_tctrl_rctrl(tegra);
	init_ports(tegra);

	/* Release SS wake logic */
	ss_set_clamp(tegra, false);
	ss_set_vcore(tegra, true);

	/* Enable UTMI pads */
	utmi_pads_enable(tegra);

	/* Pull SS out of reset */
	tegra_periph_reset_deassert(tegra->ss_clk);

	enable_irq(tegra->padctl_irq);

	return 0;
}

static void tegra_xusb_phy_shutdown(struct usb_phy *phy)
{
	struct tegra_xusb_phy *tegra = phy_to_tegra(phy);
	unsigned long pads;
	int i;

	BUG_ON(!tegra->xhci);

	disable_irq(tegra->padctl_irq);

	tegra_xhci_unregister_mbox_notifier(tegra->xhci, &tegra->mbox_nb);

	pads = tegra->board_data.hsic_pads;
	for_each_set_bit(i, &pads, TEGRA_XUSB_HSIC_COUNT)
		hsic_pad_disable(tegra, i);

	clk_disable_unprepare(tegra->ss_clk);

	pads = tegra->board_data.utmi_pads;
	for_each_set_bit(i, &pads, TEGRA_XUSB_UTMI_COUNT)
		regulator_disable(tegra->utmi_vbus[i]);
	if (tegra->board_data.hsic_pads)
		regulator_disable(tegra->vddio_hsic);
}

/*
 * Begin suspend of the XUSB PHY.  This should be called before the SuperSpeed
 * partition has been gated.
 */
void tegra_xusb_phy_presuspend(struct usb_phy *phy)
{
	struct tegra_xusb_phy *tegra = phy_to_tegra(phy);

	/*
	 * Set up SS wakeup logic and prepare for SS partition to be
	 * power-gated.
	 */
	enable_wake_interrupts(tegra);
	ss_set_clamp(tegra, true);
	tegra_periph_reset_assert(tegra->ss_clk);
	clk_disable_unprepare(tegra->ss_clk);

	usleep_range(100, 200);
}
EXPORT_SYMBOL(tegra_xusb_phy_presuspend);

/*
 * Finish suspend of the XUSB PHY SuperSpeed partition and set up wake events
 * from the PMC.  This should be called after the SuperSpeed partition has
 * been gated.
 */
static void tegra_xusb_phy_suspend(struct usb_phy *phy)
{
	struct tegra_xusb_phy *tegra = phy_to_tegra(phy);

	/* Finalize SS partition suspend */
	ss_set_vcore(tegra, false);

	/* Enable PMC wakeup logic */
	utmi_calc_tctrl_rctrl(tegra);
	pmc_wake_enable(tegra);
}

/*
 * Finish suspend of the XUSB PHY.  This should be called after both the host
 * and SuperSpeed partitions have been gated.
 */
void tegra_xusb_phy_postsuspend(struct usb_phy *phy)
{
	struct tegra_xusb_phy *tegra = phy_to_tegra(phy);

	/* Finalize USB 2.0 suspend */
	utmi_pads_release(tegra);
	utmi_pads_disable(tegra);
}
EXPORT_SYMBOL(tegra_xusb_phy_postsuspend);

/*
 * Begin resume of the XUSB PHY.  This should be called before the host
 * controller has started its resume process.
 */
void tegra_xusb_phy_preresume(struct usb_phy *phy)
{
	struct tegra_xusb_phy *tegra = phy_to_tegra(phy);

	/* Clear wakeup events */
	disable_wake_interrupts(tegra);

	/* Re-enable UTMI pads */
	utmi_pads_enable(tegra);
	if (tegra->soc_config->recalc_tctrl_rctrl)
		utmi_calc_tctrl_rctrl(tegra);

	/* Re-program ports and assign them back to XUSB */
	init_ports(tegra);
}
EXPORT_SYMBOL(tegra_xusb_phy_preresume);

/*
 * Resume the XUSB PHY SuperSpeed partition.  This should be called after
 * the SS partition has been un-gated.
 */
static void tegra_xusb_phy_resume(struct usb_phy *phy)
{
	struct tegra_xusb_phy *tegra = phy_to_tegra(phy);

	/* Re-initialize SS partition */
	clk_prepare_enable(tegra->ss_clk);
	ss_set_vcore(tegra, true);
	usleep_range(150, 200);
	ss_set_clamp(tegra, false);
	tegra_periph_reset_deassert(tegra->ss_clk);
	ss_set_clock_rate(tegra, SS_CLK_HIGH_SPEED);
}

/*
 * Finalize resume for the XUSB PHY.  This should be called after the host
 * controller has finished loading its firmware.
 */
void tegra_xusb_phy_postresume(struct usb_phy *phy)
{
	struct tegra_xusb_phy *tegra = phy_to_tegra(phy);

	/* Disable PMC wake control */
	pmc_wake_disable(tegra);
}
EXPORT_SYMBOL(tegra_xusb_phy_postresume);

static int tegra_xusb_phy_set_suspend(struct usb_phy *phy, int suspend)
{
	if (suspend)
		tegra_xusb_phy_suspend(phy);
	else
		tegra_xusb_phy_resume(phy);

	return 0;
}

/*
 * Bind a Tegra XHCI host-controller instance with this PHY.  This will
 * be used later when calling into the Tegra XHCI host driver.
 *
 * This *must* be called before usb_phy_{init,shutdown,set_suspend}.
 */
void tegra_xusb_phy_bind_xhci_dev(struct usb_phy *phy,
				  struct tegra_xhci_hcd *xhci)
{
	struct tegra_xusb_phy *tegra = phy_to_tegra(phy);

	tegra->xhci = xhci;
}
EXPORT_SYMBOL(tegra_xusb_phy_bind_xhci_dev);

static irqreturn_t tegra_xusb_phy_padctl_irq(int irq, void *data)
{
	struct tegra_xusb_phy *tegra = (struct tegra_xusb_phy *)data;
	const struct tegra_xusb_padctl_regs *padregs;
	u32 elpg_program0 = 0;

	padregs = tegra->soc_config->padctl_offsets;
	elpg_program0 = padctl_readl(tegra, padregs->elpg_program_0);
	dev_dbg(tegra->dev, "PADCTL IRQ: elpg_program_0 = 0x%08x\n",
		elpg_program0);

	/* Clear and disable wake events */
	disable_wake_interrupts(tegra);

	BUG_ON(!tegra->xhci);

	if (elpg_program0 & WAKEUP_EVENT_MASK) {
		atomic_notifier_call_chain(&tegra->u_phy.notifier,
					   USB_EVENT_VBUS, NULL);
	} else {
		dev_err(tegra->dev, "Unexpected wake event\n");
		padctl_writel(tegra, ~0, padregs->elpg_program_0);
	}
	return IRQ_HANDLED;
}

static const struct tegra_xusb_padctl_regs tegra114_padctl_offsets = {
	.boot_media_0			= 0x0,
	.usb2_pad_mux_0			= 0x4,
	.usb2_port_cap_0		= 0x8,
	.snps_oc_map_0			= 0xc,
	.usb2_oc_map_0			= 0x10,
	.ss_port_map_0			= 0x14,
	.oc_det_0			= 0x18,
	.elpg_program_0			= 0x1c,
	.usb2_bchrg_otgpadX_ctlY_0	= {
		{0x20, PADCTL_REG_NONE},
		{0x24, PADCTL_REG_NONE},
		{PADCTL_REG_NONE, PADCTL_REG_NONE}
	},
	.usb2_bchrg_bias_pad_0		= 0x28,
	.usb2_bchrg_tdcd_dbnc_timer_0	= 0x2c,
	.iophy_pll_p0_ctlY_0		= {0x30, 0x34, 0x38, 0x3c},
	.iophy_usb3_padX_ctlY_0		= {
		{0x40, 0x48, 0x50, 0x58},
		{0x44, 0x4c, 0x54, 0x5c}
	},
	.iophy_misc_pad_pX_ctlY_0	= {
		{0x60, 0x68, 0x70, 0x78, 0x80, 0x88},
		{0x64, 0x6c, 0x74, 0x7c, 0x84, 0x8c},
		{PADCTL_REG_NONE, PADCTL_REG_NONE, PADCTL_REG_NONE,
		 PADCTL_REG_NONE, PADCTL_REG_NONE},
		{PADCTL_REG_NONE, PADCTL_REG_NONE, PADCTL_REG_NONE,
		 PADCTL_REG_NONE, PADCTL_REG_NONE},
		{PADCTL_REG_NONE, PADCTL_REG_NONE, PADCTL_REG_NONE,
		 PADCTL_REG_NONE, PADCTL_REG_NONE}
	},
	.usb2_otg_padX_ctlY_0		= {
		{0x90, 0x98},
		{0x94, 0x9c},
		{PADCTL_REG_NONE, PADCTL_REG_NONE}
	},
	.usb2_bias_pad_ctlY_0		= {0xa0, 0xa4},
	.usb2_hsic_padX_ctlY_0		= {
		{0xa8, 0xb0, 0xb8},
		{0xac, 0xb4, 0xbc}
	},
	.ulpi_link_trim_ctl0		= 0xc0,
	.ulpi_null_clk_trim_ctl0	= 0xc4,
	.hsic_strb_trim_ctl0		= 0xc8,
	.wake_ctl0			= 0xcc,
	.pm_spare0			= 0xd0,
	.usb3_pad_mux_0			= PADCTL_REG_NONE,
	.iophy_pll_s0_ctlY_0		= {PADCTL_REG_NONE, PADCTL_REG_NONE,
					   PADCTL_REG_NONE, PADCTL_REG_NONE},
	.iophy_misc_pad_s0_ctlY_0	= {PADCTL_REG_NONE, PADCTL_REG_NONE,
					   PADCTL_REG_NONE, PADCTL_REG_NONE,
					   PADCTL_REG_NONE, PADCTL_REG_NONE},
};

static const struct tegra_xusb_padctl_regs tegra124_padctl_offsets = {
	.boot_media_0			= 0x0,
	.usb2_pad_mux_0			= 0x4,
	.usb2_port_cap_0		= 0x8,
	.snps_oc_map_0			= 0xc,
	.usb2_oc_map_0			= 0x10,
	.ss_port_map_0			= 0x14,
	.oc_det_0			= 0x18,
	.elpg_program_0			= 0x1c,
	.usb2_bchrg_otgpadX_ctlY_0	= {
		{0x20, 0x24},
		{0x28, 0x2c},
		{0x30, 0x34}
	},
	.usb2_bchrg_bias_pad_0		= 0x38,
	.usb2_bchrg_tdcd_dbnc_timer_0	= 0x3c,
	.iophy_pll_p0_ctlY_0		= {0x40, 0x44, 0x48, 0x4c},
	.iophy_usb3_padX_ctlY_0		= {
		{0x50, 0x58, 0x60, 0x68},
		{0x54, 0x5c, 0x64, 0x6c}
	},
	.iophy_misc_pad_pX_ctlY_0	= {
		{0x70, 0x78, 0x80, 0x88, 0x90, 0x98},
		{0x74, 0x7c, 0x84, 0x8c, 0x94, 0x9c},
		{0xec, 0xf8, 0x104, 0x110, 0x11c, 0x128},
		{0xf0, 0xfc, 0x108, 0x114, 0x120, 0x12c},
		{0xf4, 0x100, 0x10c, 0x118, 0x124, 0x130}
	},
	.usb2_otg_padX_ctlY_0		= {
		{0xa0, 0xac},
		{0xa4, 0xb0},
		{0xa8, 0xb4}
	},
	.usb2_bias_pad_ctlY_0		= {0xb8, 0xbc},
	.usb2_hsic_padX_ctlY_0		= {
		{0xc0, 0xc8, 0xd0},
		{0xc4, 0xcc, 0xd4}
	},
	.ulpi_link_trim_ctl0		= 0xd8,
	.ulpi_null_clk_trim_ctl0	= 0xdc,
	.hsic_strb_trim_ctl0		= 0xe0,
	.wake_ctl0			= 0xe4,
	.pm_spare0			= 0xe8,
	.usb3_pad_mux_0			= 0x134,
	.iophy_pll_s0_ctlY_0		= {0x138, 0x13c, 0x140, 0x144},
	.iophy_misc_pad_s0_ctlY_0	= {0x148, 0x14c, 0x150, 0x154,
					   0x158, 0x15c},
};

static const struct tegra_xusb_phy_config tegra114_soc_config = {
	.shared_ss_lanes = false,
	.save_ctle_context = false,
	.release_utmi_in_elpg = true,
	.use_hs_src_clk2 = true,
	.recalc_tctrl_rctrl = true,
	.num_utmi_pads = 2,
	.pmc_portmap = {
		TEGRA_XUSB_UTMIP_PMC_PORT0,
		TEGRA_XUSB_UTMIP_PMC_PORT2,
	},
	.rx_wander = (0x3 << 4),
	.rx_eq = (0x3928 << 8),
	.cdr_cntl = (0x26 << 24),
	.dfe_cntl = 0x002008EE,
	.hs_slew = (0xE << 6),
	.ls_rslew_pad = {0x3 << 14, 0x0 << 14, 0x0},
	.hs_disc_lvl = (0x5 << 2),
	.spare_in = 0x0,
	.utmi_port_offset = 2,
	.hsic_port_offset = 5,
	.padctl_offsets = &tegra114_padctl_offsets,
};

static const struct tegra_xusb_phy_config tegra124_soc_config = {
	.shared_ss_lanes = true,
	.save_ctle_context = true,
	.release_utmi_in_elpg = false,
	.use_hs_src_clk2 = false,
	.recalc_tctrl_rctrl = false,
	.num_utmi_pads = 3,
	.pmc_portmap = {
		TEGRA_XUSB_UTMIP_PMC_PORT0,
		TEGRA_XUSB_UTMIP_PMC_PORT1,
		TEGRA_XUSB_UTMIP_PMC_PORT2,
	},
	.rx_wander = (0xF << 4),
	.rx_eq = (0xF070 << 8),
	.cdr_cntl = (0x26 << 24),
	.dfe_cntl = 0x002008EE,
	.hs_slew = (0xE << 6),
	.ls_rslew_pad = {0x3 << 14, 0x0 << 14, 0x0 << 14},
	.hs_disc_lvl = (0x5 << 2),
	.spare_in = 0x1,
	.utmi_port_offset = 2,
	.hsic_port_offset = 6,
	.padctl_offsets = &tegra124_padctl_offsets,
};

static struct of_device_id tegra_xusb_phy_id_table[] = {
	{
		.compatible = "nvidia,tegra114-xusb-phy",
		.data = &tegra114_soc_config,
	},
	{
		.compatible = "nvidia,tegra124-xusb-phy",
		.data = &tegra124_soc_config,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_xusb_phy_id_table);

static int tegra_xusb_phy_parse_dt(struct tegra_xusb_phy *tegra)
{
	struct tegra_xusb_phy_board_data *bdata = &tegra->board_data;
	struct device_node *np = tegra->dev->of_node;
	u32 val;
	int ret, i;

	if (of_property_read_u32(np, "nvidia,ss-pads", &val)) {
		dev_err(tegra->dev, "Missing SS pad map\n");
		return -EINVAL;
	}
	bdata->ss_pads = val;
	if (of_property_read_u32(np, "nvidia,hsic-pads", &val)) {
		dev_err(tegra->dev, "Missing HSIC pad map\n");
		return -EINVAL;
	}
	bdata->hsic_pads = val;
	for_each_set_bit(i, &bdata->hsic_pads, TEGRA_XUSB_HSIC_COUNT) {
		char prop[sizeof("nvidia,hsicN-config")];

		sprintf(prop, "nvidia,hsic%d-config", i);
		ret = of_property_read_u8_array(np, prop,
						(u8 *)&bdata->hsic[i],
						sizeof(bdata->hsic[i]));
		if (ret) {
			dev_err(tegra->dev, "Missing hsic %d config\n", i);
			return -EINVAL;
		}
	}
	if (of_property_read_u32(np, "nvidia,utmi-pads", &val)) {
		dev_err(tegra->dev, "Missing UTMI pad map\n");
		return -EINVAL;
	}
	bdata->utmi_pads = val;
	if (of_property_read_u32(np, "nvidia,ss-portmap", &bdata->ss_portmap)) {
		dev_err(tegra->dev, "Missing SS portmap\n");
		return -EINVAL;
	}
	if (of_property_read_u32(np, "nvidia,lane-owner", &bdata->lane_owner)) {
		dev_err(tegra->dev, "Missing lane owner\n");
		return -EINVAL;
	}
	of_property_read_u32(np, "nvidia,xusb-hs-xcvr-setup-offset",
			     &bdata->hs_xcvr_setup_offset);

	return 0;
}

static void tegra_xusb_phy_read_calib_data(struct tegra_xusb_phy *tegra)
{
	u32 usb_calib0;

	usb_calib0 = tegra30_fuse_readl(FUSE_SKU_USB_CALIB_0);
	dev_dbg(tegra->dev, "usb_calib0 = 0x%08x\n", usb_calib0);
	/*
	 * Read calibration data from fuse:
	 * set HS_CURR_LEVEL (PAD0)	= usb_calib0[5:0]
	 * set TERM_RANGE_ADJ		= usb_calib0[10:7]
	 * set HS_SQUELCH_LEVEL		= usb_calib0[12:11]
	 * set HS_IREF_CAP		= usb_calib0[14:13]
	 * set HS_CURR_LEVEL (PAD1/2)	= usb_calib0[20:15]
	 */
	tegra->calib_data.hs_curr_level_pad[0] = (usb_calib0 >> 0) & 0x3f;
	tegra->calib_data.hs_term_range_adj = (usb_calib0 >> 7) & 0xf;
	tegra->calib_data.hs_squelch_level = (usb_calib0 >> 11) & 0x3;
	tegra->calib_data.hs_iref_cap = (usb_calib0 >> 13) & 0x3;
	tegra->calib_data.hs_curr_level_pad[1] = (usb_calib0 >> 15) & 0x3f;
	tegra->calib_data.hs_curr_level_pad[2] = (usb_calib0 >> 15) & 0x3f;
}

static int tegra_xusb_phy_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct resource *res;
	struct tegra_xusb_phy *tegra;
	unsigned long utmi_pads;
	int err, i;

	tegra = devm_kzalloc(&pdev->dev, sizeof(*tegra), GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;
	tegra->dev = &pdev->dev;
	platform_set_drvdata(pdev, tegra);

	tegra->mbox_nb.notifier_call = tegra_xusb_phy_mbox_notifier;

	match = of_match_device(tegra_xusb_phy_id_table, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "No matching device found\n");
		return -ENODEV;
	}
	tegra->soc_config = match->data;

	tegra_xusb_phy_read_calib_data(tegra);
	err = tegra_xusb_phy_parse_dt(tegra);
	if (err)
		return err;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get padctl regs\n");
		return -ENODEV;
	}
	tegra->padctl_regs = devm_ioremap(&pdev->dev, res->start,
					  resource_size(res));
	if (!tegra->padctl_regs) {
		dev_err(&pdev->dev, "Failed to map padctl regs\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get UTMI pad regs\n");
		return -ENODEV;
	}
	tegra->pad_regs = devm_ioremap(&pdev->dev, res->start,
					  resource_size(res));
	if (!tegra->pad_regs) {
		dev_err(&pdev->dev, "Failed to map UTMI pad regs\n");
		return -ENOMEM;
	}

	tegra->pmc_regs = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							  "nvidia,pmc");
	if (IS_ERR(tegra->pmc_regs)) {
		dev_err(&pdev->dev, "Failed to get PMC regs\n");
		return PTR_ERR(tegra->pmc_regs);
	}

	tegra->clkrst_regs = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							     "nvidia,clkrst");
	if (IS_ERR(tegra->clkrst_regs)) {
		dev_err(&pdev->dev, "Failed to get CLKRST regs\n");
		return PTR_ERR(tegra->clkrst_regs);
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get padctl IRQ\n");
		return -ENODEV;
	}
	tegra->padctl_irq = res->start;
	err = devm_request_irq(&pdev->dev, tegra->padctl_irq,
			       tegra_xusb_phy_padctl_irq, IRQF_TRIGGER_HIGH,
			       dev_name(&pdev->dev), tegra);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to request padctl IRQ\n");
		return err;
	}
	disable_irq(tegra->padctl_irq);

	if (tegra->board_data.hsic_pads) {
		tegra->vddio_hsic = devm_regulator_get(&pdev->dev,
						       "vddio-hsic");
		if (IS_ERR(tegra->vddio_hsic)) {
			dev_err(&pdev->dev,
				"Failed to get vddio-hsic regulator\n");
			return PTR_ERR(tegra->vddio_hsic);
		}
	}

	utmi_pads = tegra->board_data.utmi_pads;
	for_each_set_bit(i, &utmi_pads, TEGRA_XUSB_UTMI_COUNT) {
		char reg_name[sizeof("vbusN")];

		sprintf(reg_name, "vbus%d", i + 1);
		tegra->utmi_vbus[i] = devm_regulator_get(&pdev->dev, reg_name);
		if (IS_ERR(tegra->utmi_vbus[i])) {
			dev_err(&pdev->dev, "Failed to get %s regulator\n",
				reg_name);
			return PTR_ERR(tegra->utmi_vbus[i]);
		}
	}

	tegra->ss_src_clk = devm_clk_get(&pdev->dev, "xusb_ss_src");
	if (IS_ERR(tegra->ss_src_clk)) {
		dev_err(&pdev->dev, "Failed to get SS source clock\n");
		return PTR_ERR(tegra->ss_src_clk);
	}
	tegra->ss_clk = devm_clk_get(&pdev->dev, "xusb_ss");
	if (IS_ERR(tegra->ss_clk)) {
		dev_err(&pdev->dev, "Failed to get SS clock\n");
		return PTR_ERR(tegra->ss_clk);
	}
	tegra->pll_u_480M = devm_clk_get(&pdev->dev, "pll_u_480M");
	if (IS_ERR(tegra->pll_u_480M)) {
		dev_err(&pdev->dev, "Failed to get PLL_U_480M\n");
		return PTR_ERR(tegra->pll_u_480M);
	}
	tegra->clk_m = devm_clk_get(&pdev->dev, "clk_m");
	if (IS_ERR(tegra->clk_m)) {
		dev_err(&pdev->dev, "Failed to get clk_m\n");
		return PTR_ERR(tegra->clk_m);
	}
	tegra->pad_clk = devm_clk_get(&pdev->dev, "pad_clk");
	if (IS_ERR(tegra->pad_clk)) {
		dev_err(&pdev->dev, "Failed to get UTMI pad clock\n");
		return PTR_ERR(tegra->pad_clk);
	}
	tegra->plle = devm_clk_get(&pdev->dev, "pll_e");
	if (IS_ERR(tegra->plle)) {
		dev_err(&pdev->dev, "Failed to get PLLE\n");
		return PTR_ERR(tegra->plle);
	}
	err = clk_prepare_enable(tegra->plle);
	if (err) {
		dev_err(&pdev->dev, "Failed to enable PLLE\n");
		return err;
	}

	tegra->u_phy.dev = &pdev->dev;
	tegra->u_phy.init = tegra_xusb_phy_init;
	tegra->u_phy.shutdown = tegra_xusb_phy_shutdown;
	tegra->u_phy.set_suspend = tegra_xusb_phy_set_suspend;
	ATOMIC_INIT_NOTIFIER_HEAD(&tegra->u_phy.notifier);

	err = usb_add_phy_dev(&tegra->u_phy);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to add PHY device\n");
		clk_disable_unprepare(tegra->plle);
		return err;
	}

	return 0;
}

static int tegra_xusb_phy_remove(struct platform_device *pdev)
{
	struct tegra_xusb_phy *tegra = platform_get_drvdata(pdev);

	usb_remove_phy(&tegra->u_phy);
	clk_disable_unprepare(tegra->plle);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_xusb_phy_pm_suspend(struct device *dev)
{
	struct tegra_xusb_phy *tegra = dev_get_drvdata(dev);

	clk_disable_unprepare(tegra->plle);

	return 0;
}

static int tegra_xusb_phy_pm_resume(struct device *dev)
{
	struct tegra_xusb_phy *tegra = dev_get_drvdata(dev);

	clk_prepare_enable(tegra->plle);

	return 0;
}
#endif

static const struct dev_pm_ops tegra_xusb_phy_pm_ops = {
#ifdef CONFIG_PM_SLEEP
	.suspend_late = tegra_xusb_phy_pm_suspend,
	.resume_early = tegra_xusb_phy_pm_resume,
#endif
};

static struct platform_driver tegra_xusb_phy_driver = {
	.probe		= tegra_xusb_phy_probe,
	.remove		= tegra_xusb_phy_remove,
	.driver		= {
		.name	= "tegra-xusb-phy",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_xusb_phy_id_table),
		.pm	= &tegra_xusb_phy_pm_ops,
	},
};
module_platform_driver(tegra_xusb_phy_driver);

MODULE_DESCRIPTION("Tegra XUSB PHY driver");
MODULE_LICENSE("GPL v2");
