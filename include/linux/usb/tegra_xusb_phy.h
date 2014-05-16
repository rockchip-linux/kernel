/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _TEGRA_XUSB_PHY_INTERFACE_H_
#define _TEGRA_XUSB_PHY_INTERFACE_H_

#include <linux/usb/tegra_xusb.h>

/* SS pads */
#define TEGRA_XUSB_SS_P0	BIT(0)
#define TEGRA_XUSB_SS_P1	BIT(1)
#define TEGRA_XUSB_SS_COUNT	2

/* UTMI pads */
#define TEGRA_XUSB_UTMI_P0	BIT(0)
#define TEGRA_XUSB_UTMI_P1	BIT(1)
#define TEGRA_XUSB_UTMI_P2	BIT(2)
#define TEGRA_XUSB_UTMI_COUNT	3

/* HSIC pads */
#define TEGRA_XUSB_HSIC_P0	BIT(0)
#define TEGRA_XUSB_HSIC_P1	BIT(1)
#define TEGRA_XUSB_HSIC_COUNT	2

/* PMC UTMI ports */
#define TEGRA_XUSB_UTMIP_PMC_PORT0	0x0
#define TEGRA_XUSB_UTMIP_PMC_PORT1	0x1
#define TEGRA_XUSB_UTMIP_PMC_PORT2	0x2

#define SS_CLK_HIGH_SPEED	120000000
#define SS_CLK_LOW_SPEED	12000000

/* FUSE bits */
#define FUSE_SKU_USB_CALIB_0			0xf0

/* CLKRST bits */
#define SATA_PLL_CFG0_0		0x490
#define SATA_PADPLL_USE_LOCKDET	BIT(2)
#define SATA_PADPLL_RESET_SWCTL	BIT(0)
#define SATA_SEQ_ENABLE		BIT(24)
#define SATA_SEQ_START_STATE	BIT(25)
#define UTMIPLL_HW_PWRDN_CFG0	0x52c
#define UTMIPLL_IDDQ_OVERRIDE	BIT(1)
#define UTMIPLL_IDDQ_SWCTL	BIT(0)

/* PAD bits */
#define UTMIP_BIAS_CFG0			0x80c
#define UTMIP_OTGPD			BIT(11)
#define UTMIP_BIASPD			BIT(10)
#define UTMIP_HSSQUELCH_LEVEL(x)	(((x) & 0x3) << 0)
#define UTMIP_HSDISCON_LEVEL(x)		(((x) & 0x3) << 2)
#define UTMIP_HSDISCON_LEVEL_MSB	BIT(24)
#define UTMIP_BIAS_CFG1			0x83c
#define UTMIP_BIAS_PDTRK_COUNT(x)	(((x) & 0x1f) << 3)
#define UTMIP_BIAS_PDTRK_POWERDOWN	BIT(0)
#define UTMIP_BIAS_PDTRK_POWERUP	BIT(1)
#define UTMIP_BIAS_STS0			0x840
#define UTMIP_RCTRL_VAL(x)		((x) & 0xffff)
#define UTMIP_TCTRL_VAL(x)		(((x) >> 16) & 0xffff)

/* XUSB_PADCTL bits */
/* USB2_PAD_MUX_0 */
#define USB2_OTG_PAD_PORT_MASK(x)	(0x3 << (2 * (x)))
#define USB2_OTG_PAD_PORT_OWNER_SNPS(x) (0x0 << (2 * (x)))
#define USB2_OTG_PAD_PORT_OWNER_XUSB(x)	(0x1 << (2 * (x)))
#define USB2_OTG_PAD_PORT_OWNER_UART(x)	(0x2 << (2 * (x)))
#define USB2_ULPI_PAD_PORT		(0x1 << 12)
#define USB2_ULPI_PAD_PORT_OWNER_SNPS	(0x0 << 12)
#define USB2_ULPI_PAD_PORT_OWNER_XUSB	(0x1 << 12)
#define USB2_HSIC_PAD_PORT(x)		(0x1 << (14 + (x)))
#define USB2_HSIC_PAD_PORT_OWNER_SNPS(x)	(0x0 << (14 + (x)))
#define USB2_HSIC_PAD_PORT_OWNER_XUSB(x)	(0x1 << (14 + (x)))
/* USB2_PORT_CAP_0 */
#define USB2_PORT_CAP_MASK(x)		(0x3 << (4 * x))
#define USB2_PORT_CAP_HOST(x)		(0x1 << (4 * x))
#define USB2_ULPI_PORT_CAP		(0x1 << 24)
/* USB2_OC_MAP_0 */
#define SNPS_OC_MAP_CTRL1		(0x7 << 0)
#define SNPS_OC_MAP_CTRL2		(0x7 << 3)
#define SNPS_OC_MAP_CTRL3		(0x7 << 6)
#define USB2_OC_MAP_PORT0		(0x7 << 0)
#define USB2_OC_MAP_PORT1		(0x7 << 3)
#define USB2_OC_MAP_PORT2		(0x7 << 6)
#define USB2_OC_MAP_PORT(n)		(0x7 << ((n) * 3))
#define USB2_OC_MAP_PORT0_OC_DETECTED_VBUS_PAD0 (0x4 << 0)
#define USB2_OC_MAP_PORT1_OC_DETECTED_VBUS_PAD1 (0x5 << 3)
/* OC_DET_0 */
#define OC_DET_VBUS_ENABLE0_OC_MAP (0x7 << 10)
#define OC_DET_VBUS_ENABLE1_OC_MAP (0x7 << 13)
#define OC_DET_VBUS_ENABLE2_OC_MAP (0x7 << 5)
#define OC_DET_VBUS_ENABLE_OC_MAP(n)					\
	((n) == 2 ? OC_DET_VBUS_ENABLE2_OC_MAP :			\
		(n) ? OC_DET_VBUS_ENABLE1_OC_MAP :			\
			OC_DET_VBUS_ENABLE0_OC_MAP)
#define OC_DET_VBUS_EN0_OC_DETECTED_VBUS_PAD0 (0x4 << 10)
#define OC_DET_VBUS_EN1_OC_DETECTED_VBUS_PAD1 (0x5 << 13)
#define OC_DET_VBUS_EN2_OC_DETECTED_VBUS_PAD2 (0x6 << 5)
#define OC_DET_VBUS_EN_OC_DETECTED_VBUS_PAD(n)				\
	((n) == 2 ? OC_DET_VBUS_EN2_OC_DETECTED_VBUS_PAD2 :		\
		(n) ? OC_DET_VBUS_EN1_OC_DETECTED_VBUS_PAD1 :		\
			OC_DET_VBUS_EN0_OC_DETECTED_VBUS_PAD0)
#define OC_DET_OC_DETECTED_VBUS_PAD0	BIT(20)
#define OC_DET_OC_DETECTED_VBUS_PAD1	BIT(21)
#define OC_DET_OC_DETECTED_VBUS_PAD2	BIT(22)
#define OC_DET_OC_DETECTED_VBUS_PAD(n)	BIT(20 + (n))
/* SS_PORT_MAP_0 */
#define SS_PORT_MAP_P0			(0xf << 0)
#define SS_PORT_MAP_P1			(0xf << 4)
#define SS_PORT_MAP_P(n)		(0xf << (4 * n))
#define SS_PORT_MAP_USB2_PORT0		0x0
#define SS_PORT_MAP_USB2_PORT1		0x1
#define SS_PORT_MAP_USB2_PORT2		0x2
/* USB2_OTG_PAD_CTL0_0 */
#define USB2_OTG_HS_CURR_LVL_MAX	0x3f
#define USB2_OTG_HS_CURR_LVL		(0x3f << 0)
#define USB2_OTG_HS_SLEW		(0x3f << 6)
#define USB2_OTG_FS_SLEW		(0x3 << 12)
#define USB2_OTG_LS_RSLEW		(0x3 << 14)
#define USB2_OTG_LS_FSLEW		(0x3 << 16)
#define USB2_OTG_PD			(0x1 << 19)
#define USB2_OTG_PD2			(0x1 << 20)
#define USB2_OTG_PD_ZI			(0x1 << 21)
/* USB2_OTG_PAD_CTL1_0 */
#define USB2_OTG_PD_CHRP_FORCE_POWERUP	(0x1 << 0)
#define USB2_OTG_PD_DISC_FORCE_POWERUP	(0x1 << 1)
#define USB2_OTG_PD_DR			(0x1 << 2)
#define USB2_OTG_TERM_RANGE_AD		(0xF << 3)
#define USB2_OTG_HS_IREF_CAP		(0x3 << 9)
/* USB2_BIAS_PAD_CTL0_0 */
#define USB2_BIAS_HS_SQUELCH_LEVEL	(0x3 << 0)
#define USB2_BIAS_HS_DISCON_LEVEL	(0x7 << 2)
#define USB2_BIAS_PD			(0x1 << 12)
#define USB2_BIAS_PD_TRK		(0x1 << 13)
/* USB2_BIAS_PAD_CTL1_0 */
#define USB2_BIAS_RCTRL_VAL(x)		((x) & 0xffff)
#define USB2_BIAS_TCTRL_VAL(x)		(((x) >> 16) & 0xffff)
/* USB2_HSIC_PAD_CTL0_0 */
#define USB2_HSIC_TX_RTUNEP(x)		(((x) & 0xf) << 0)
#define USB2_HSIC_TX_RTUNEN(x)		(((x) & 0xf) << 4)
#define USB2_HSIC_TX_SLEWP(x)		(((x) & 0xf) << 8)
#define USB2_HSIC_TX_SLEWN(x)		(((x) & 0xf) << 12)
#define USB2_HSIC_OPT(x)		(((x) & 0xf) << 16)
/* USB2_HSIC_PAD_CTL1_0 */
#define USB2_HSIC_AUTO_TERM_EN		BIT(0)
#define USB2_HSIC_IDDQ			BIT(1)
#define USB2_HSIC_PD_TX			BIT(2)
#define USB2_HSIC_PD_TRX		BIT(3)
#define USB2_HSIC_PD_RX			BIT(4)
#define USB2_HSIC_PD_ZI			BIT(5)
#define USB2_HISC_LPBK			BIT(6)
#define USB2_HSIC_RPD_DATA		BIT(7)
#define USB2_HSIC_RPD_STROBE		BIT(8)
#define USB2_HSIC_RPU_DATA		BIT(9)
#define USB2_HSIC_RPU_STROBE		BIT(10)
/* USB2_HSIC_PAD_CTL2_0 */
#define USB2_HSIC_RX_DATA_TRIM(x)	(((x) & 0xf) << 0)
#define USB2_HSIC_RX_STROBE_TRIM(x)	(((x) & 0xf) << 4)
#define USB2_HSIC_CALIOUT(x)		(((x) & 0xffff) << 16)
/* HSIC_STRB_TRIM_CONTROL_0 */
#define HSIC_STRB_TRIM_VAL(x)		((x) & 0x3f)
/* IOPHY_MISC_PAD_CTL2_0 */
#define IOPHY_SPARE_IN(x)		(((x) & 0x3) << 28)
/* IOPHY_MISC_PAD_CTL3_0 */
#define IOPHY_MISC_CNTL(x)		(((x) & 0xf) << 0)
#define IOPHY_TX_SEL_LOAD(x)		(((x) & 0xf) << 8)
#define IOPHY_TX_RDET_T(x)		(((x) & 0x3) << 12)
#define IOPHY_RX_IDLE_T(x)		(((x) & 0x3) << 14)
#define IOPHY_TX_RDET_BYP		BIT(16)
#define IOPHY_RX_IDLE_BYP		BIT(17)
#define IOPHY_RX_IDLE_MODE		BIT(18)
#define IOPHY_RX_IDLE_MODE_OVRD		BIT(19)
#define IOPHY_CDR_TEST(x)		(((x) & 0xfff) << 20)
/* IOPHY_MISC_PAD_CTL5_0 */
#define IOPHY_RX_QEYE_EN		BIT(8)
/* IOPHY_MISC_PAD_CTL6_0 */
#define IOPHY_MISC_OUT_SEL(x)		((x & 0xff) << 16)
#define IOPHY_MISC_OUT_SEL_TAP		0x32
#define IOPHY_MISC_OUT_SEL_AMP		0x33
#define IOPHY_MISC_OUT_SEL_LATCH_G_Z	0xa1
#define IOPHY_MISC_OUT_SEL_G_Z		0x21
#define IOPHY_MISC_OUT_SEL_CTLE_Z	0x48
#define IOPHY_MISC_OUT_TAP_VAL(reg)	((reg & (0x1f << 24)) >> 24)
#define IOPHY_MISC_OUT_AMP_VAL(reg)	((reg & (0x7f << 24)) >> 24)
#define IOPHY_MISC_OUT_G_Z_VAL(reg)	((reg & (0x3f << 24)) >> 24)
/* IOPHY_USB3_PAD_CTL2_0 */
#define IOPHY_USB3_RX_WANDER_VAL(x)	(((x) & 0xf) << 4)
#define IOPHY_USB3_RX_EQ_G_VAL(x)	(((x) & 0x3f) << 8)
#define IOPHY_USB3_RX_EQ_Z_VAL(x)	(((x) & 0x3f) << 16)
#define IOPHY_USB3_RX_EQ_VAL(x)		(((x) & 0xffff) << 8)
#define IOPHY_USB3_CDR_CNTL_VAL(x)	(((x) & 0xff) << 24)
/* IOPHY_USB3_PAD_CTL4_0 */
#define IOPHY_USB3_DFE_CNTL_TAP_VAL(x)	(((x) & 0x1f) << 24)
#define IOPHY_USB3_DFE_CNTL_AMP_VAL(x)	(((x) & 0x7f) << 16)
/* IOPHY_PLL_CTL1_0 */
#define IOPHY_PLL_PLL0_REFCLK_NDIV(x)	(((x) & 0x3) << 20)
/* IOPHY_PLL_CTL2_0 */
#define IOPHY_PLL_XDIGCLK_SEL(x)	((x) & 0x7)
#define IOPHY_PLL_TXCLKREF_SEL		BIT(4)
#define IOPHY_PLL_TCLKOUT_EN		BIT(12)
#define IOPHY_PLL_PLL0_CP_CNTL(x)	(((x) & 0xf) << 16)
#define IOPHY_PLL_PLL1_CP_CNTL(x)	(((x) & 0xf) << 20)
/* IOPHY_PLL_CTL3_0 */
#define IOPHY_PLL_RCAL_BYPASS		BIT(7)
/* USB3_PAD_MUX_0 */
#define USB3_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK(x)	BIT(1 + (x))
#define USB3_FORCE_SATA_PAD_IDDQ_DISABLE_MASK		BIT(6)
#define USB3_PCIE_PAD_LANE_OWNER(l, x)		(((x) & 0x3) << (16 + 2 * (l)))
#define USB3_SATA_PAD_LANE_OWNER(x)		(((x) & 0x3) << 26)
#define USB3_LANE_OWNER_PCIE			0x0
#define USB3_LANE_OWNER_USB3_SS			0x1
#define USB3_LANE_OWNER_SATA			0x2
/* ELPG_PROGRAM_0 */
#define USB2_PORT0_WAKE_INTERRUPT_ENABLE	BIT(0)
#define USB2_PORT1_WAKE_INTERRUPT_ENABLE	BIT(1)
#define USB2_PORT2_WAKE_INTERRUPT_ENABLE	BIT(2)
#define USB2_PORT_WAKE_INTERRUPT_ENABLE(n)	BIT(n)
#define USB2_HSIC_PORT0_WAKE_INTERRUPT_ENABLE	BIT(3)
#define USB2_HSIC_PORT1_WAKE_INTERRUPT_ENABLE	BIT(4)
#define USB2_HSIC_PORT_WAKE_INTERRUPT_ENABLE(n)	BIT(3 + (n))
#define SS_PORT0_WAKE_INTERRUPT_ENABLE		BIT(6)
#define SS_PORT1_WAKE_INTERRUPT_ENABLE		BIT(7)
#define SS_PORT_WAKE_INTERRUPT_ENABLE(n)	BIT(6 + (n))
#define USB2_PORT0_WAKEUP_EVENT		BIT(8)
#define USB2_PORT1_WAKEUP_EVENT		BIT(9)
#define USB2_PORT2_WAKEUP_EVENT		BIT(10)
#define USB2_PORT_WAKEUP_EVENT(n)	BIT(8 + (n))
#define USB2_HSIC_PORT0_WAKEUP_EVENT	BIT(11)
#define USB2_HSIC_PORT1_WAKEUP_EVENT	BIT(12)
#define USB2_HSIC_PORT_WAKEUP_EVENT(n)	BIT(11 + (n))
#define SS_PORT0_WAKEUP_EVENT		BIT(14)
#define SS_PORT1_WAKEUP_EVENT		BIT(15)
#define SS_PORT_WAKEUP_EVENT(n)		BIT(14 + (n))
#define WAKEUP_EVENT_MASK		(0xdf << 8)
#define SSP0_ELPG_CLAMP_EN		BIT(16)
#define SSP0_ELPG_CLAMP_EN_EARLY	BIT(17)
#define SSP0_ELPG_VCORE_DOWN		BIT(18)
#define SSP1_ELPG_CLAMP_EN		BIT(20)
#define SSP1_ELPG_CLAMP_EN_EARLY	BIT(21)
#define SSP1_ELPG_VCORE_DOWN		BIT(22)
#define SSP_ELPG_CLAMP_EN(n)		BIT(16 + 4 * (n))
#define SSP_ELPG_CLAMP_EN_EARLY(n)	BIT(17 + 4 * (n))
#define SSP_ELPG_VCORE_DOWN(n)		BIT(18 + 4 * (n))
#define AUX_MUX_LP0_CLAMP_EN		BIT(24)
#define AUX_MUX_LP0_CLAMP_EN_EARLY	BIT(25)
#define AUX_MUX_LP0_VCORE_DOWN		BIT(26)

#define PADCTL_REG_NONE			0xffff

struct tegra_xusb_padctl_regs {
	u16 boot_media_0;
	u16 usb2_pad_mux_0;
	u16 usb2_port_cap_0;
	u16 snps_oc_map_0;
	u16 usb2_oc_map_0;
	u16 ss_port_map_0;
	u16 oc_det_0;
	u16 elpg_program_0;
	u16 usb2_bchrg_otgpadX_ctlY_0[3][2];
	u16 usb2_bchrg_bias_pad_0;
	u16 usb2_bchrg_tdcd_dbnc_timer_0;
	u16 iophy_pll_p0_ctlY_0[4];
	u16 iophy_usb3_padX_ctlY_0[2][4];
	u16 iophy_misc_pad_pX_ctlY_0[5][6];
	u16 usb2_otg_padX_ctlY_0[3][2];
	u16 usb2_bias_pad_ctlY_0[2];
	u16 usb2_hsic_padX_ctlY_0[2][3];
	u16 ulpi_link_trim_ctl0;
	u16 ulpi_null_clk_trim_ctl0;
	u16 hsic_strb_trim_ctl0;
	u16 wake_ctl0;
	u16 pm_spare0;
	u16 usb3_pad_mux_0;
	u16 iophy_pll_s0_ctlY_0[4];
	u16 iophy_misc_pad_s0_ctlY_0[6];
};

struct usb_phy;
struct tegra_xhci_hcd;

extern void tegra_xusb_phy_bind_xhci_dev(struct usb_phy *phy,
					 struct tegra_xhci_hcd *tegra);
extern void tegra_xusb_phy_presuspend(struct usb_phy *phy);
extern void tegra_xusb_phy_postsuspend(struct usb_phy *phy);
extern void tegra_xusb_phy_preresume(struct usb_phy *phy);
extern void tegra_xusb_phy_postresume(struct usb_phy *phy);

#endif /* _TEGRA_XUSB_PHY_INTERFACE_H_ */
