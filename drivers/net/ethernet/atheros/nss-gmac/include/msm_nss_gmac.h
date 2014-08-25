/*
 * Copyright (c) 2014 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_NSS_GMAC_H
#define __ASM_NSS_GMAC_H

#include <linux/phy.h>

#include <msm_nss_macsec.h>

/* NSS GMAC Base Addresses */
#define NSS_GMAC0_BASE			0x37000000
#define NSS_GMAC1_BASE			0x37200000
#define NSS_GMAC2_BASE			0x37400000
#define NSS_GMAC3_BASE			0x37600000
#define NSS_GMAC_REG_LEN		0x00200000

/* NSS GMAC Specific defines */
#define NSS_REG_BASE			0x03000000
#define NSS_REG_LEN			0x0000FFFF


/* Offsets of NSS config and status registers within NSS_REG_BASE */
/* We start the GMAC numbering from 0 */
#define NSS_CSR_REV			0x0000
#define NSS_CSR_CFG			0x0004
#define NSS_ETH_CLK_GATE_CTL		0x0008
#define NSS_ETH_CLK_DIV0		0x000C
#define NSS_ETH_CLK_DIV1		0x0010
#define NSS_ETH_CLK_SRC_CTL		0x0014
#define NSS_ETH_CLK_INV_CTL		0x0018
#define NSS_MACSEC_CTL			0x0028
#define NSS_QSGMII_CLK_CTL		0x002C
#define NSS_GMAC0_CTL			0x0030
#define NSS_GMAC1_CTL			0x0034
#define NSS_GMAC2_CTL			0x0038
#define NSS_GMAC3_CTL			0x003C
#define NSS_ETH_CLK_ROOT_STAT		0x0040
#define NSS_QSGMII_STAT			0x0044
#define NSS_ETH_SPARE_CTL		0x0088
#define NSS_ETH_SPARE_STAT		0x008C


/* Macros to calculate register offsets */
#define NSS_GMACn_CTL(n)		(NSS_GMAC0_CTL +  (n * 4))
#define NSS_ETH_CLK_CTLn(x)		(NSS_ETH_CLK_CTL0 +  (x * 4))


/* NSS_ETH_CLK_GATE_CTL bits */
#define MACSEC3_CORE_CLK		(1 << 30)
#define MACSEC2_CORE_CLK		(1 << 29)
#define MACSEC1_CORE_CLK		(1 << 28)
#define MACSEC_CORE_CLKEN_VAL		(0x7 << 28)
#define MACSEC_GMII_RX_CLKEN_VAL	(0x7 << 24)
#define MACSEC_GMII_TX_CLKEN_VAL	(0x7 << 20)
#define GMAC0_PTP_CLK			(1 << 16)
#define GMAC0_RGMII_RX_CLK		(1 << 9)
#define GMAC0_RGMII_TX_CLK		(1 << 8)
#define GMAC0_GMII_RX_CLK		(1 << 4)
#define GMAC0_GMII_TX_CLK		(1 << 0)

#define GMAC0_RGMII_TX_CLK_SHIFT		8
#define GMAC0_RGMII_RX_CLK_SHIFT		9
#define GMAC0_GMII_RX_CLK_SHIFT			4
#define GMAC0_GMII_TX_CLK_SHIFT			0
#define GMAC0_PTP_CLK_SHIFT			16

/* Macros to calculate bit offsets in NSS_ETH_CLK_GATE_CTL register
 * MACSEC_CORE_CLK: x = 1,2,3
 * GMII_xx_CLK: x = 0,1,2,3
 * RGMII_xx_CLK: x = 0,1
 * PTP_CLK: x = 0,1,2,3
*/
#define MACSECn_CORE_CLK(x)		(1 << (MACSEC1_CORE_CLK + x))
#define GMACn_GMII_TX_CLK(x)		(1 << (GMAC0_GMII_TX_CLK_SHIFT + x))
#define GMACn_GMII_RX_CLK(x)		(1 << (GMAC0_GMII_RX_CLK_SHIFT + x))
#define GMACn_RGMII_TX_CLK(x)		(1 << (GMAC0_RGMII_TX_CLK_SHIFT + (x * 2)))
#define GMACn_RGMII_RX_CLK(x)		(1 << (GMAC0_RGMII_RX_CLK_SHIFT + (x * 2)))
#define GMACn_PTP_CLK(x)		(1 << (GMAC0_PTP_CLK_SHIFT + x))

/* NSS_ETH_CLK_DIV0 bits ; n = 0,1,2,3 */
/* PHY increments divider values by 1. Hence the values here are (x - 1) */
#define RGMII_CLK_DIV_1000			1
#define RGMII_CLK_DIV_100			9
#define RGMII_CLK_DIV_10			99
#define SGMII_CLK_DIV_1000			0
#define SGMII_CLK_DIV_100			4
#define SGMII_CLK_DIV_10			49
#define QSGMII_CLK_DIV_1000			1
#define QSGMII_CLK_DIV_100			9
#define QSGMII_CLK_DIV_10			99
#define GMACn_CLK_DIV_SIZE			0x7F
#define GMACn_CLK_DIV(n,val)			(val << (n * 8))

/* NSS_ETH_CLK_SRC_CTL bits */
#define GMAC0_GMII_CLK_RGMII			(1 << 0)
#define GMAC1_GMII_CLK_RGMII			(1 << 1)

/* NSS_MACSEC_CTL bits */
#define GMAC1_MACSEC_BYPASS			0x1
#define GMACn_MACSEC_BYPASS(n)			(GMAC1_MACSEC_BYPASS << (n - 1))	/* n = 1,2,3 */
#define MACSEC_EXT_BYPASS_EN_MASK		0x7
#define MACSEC_DP_RST_VAL			(0x7 << 4)

/* Macros to calculate bit offsets in NSS_ETH_CLK_CTL3 register */
#define GMACn_GMII_CLK_RGMII(x)			(1 << x)

/* NSS_QSGMII_CLK_CTL bits */
#define GMAC0_TX_CLK_HALT			(1 << 7)
#define GMAC0_RX_CLK_HALT			(1 << 8)
#define GMAC1_TX_CLK_HALT			(1 << 9)
#define GMAC1_RX_CLK_HALT			(1 << 10)
#define GMAC2_TX_CLK_HALT			(1 << 11)
#define GMAC2_RX_CLK_HALT			(1 << 12)
#define GMAC3_TX_CLK_HALT			(1 << 13)
#define GMAC3_RX_CLK_HALT			(1 << 14)

#define GMAC0_QSGMII_TX_CLK_SHIFT		7
#define GMAC0_QSGMII_RX_CLK_SHIFT		8

/* Macros to calculate bit offsets in NSS_QSGMII_CLK_CTL register */
#define GMACn_QSGMII_TX_CLK(n)			(1 << (GMAC0_QSGMII_TX_CLK_SHIFT + (n * 2)))
#define GMACn_QSGMII_RX_CLK(n)			(1 << (GMAC0_QSGMII_RX_CLK_SHIFT + (n * 2)))

/* NSS_GMACn_CTL bits */
#define GMAC_IFG_CTL(x)				(x)
#define GMAC_IFG_LIMIT(x)			(x << 8)
#define GMAC_PHY_RGMII				(1 << 16)
#define GMAC_PHY_QSGMII				(0 << 16)
#define GMAC_FLOW_CTL				(1 << 18)
#define GMAC_CSYS_REQ				(1 << 19)
#define GMAC_PTP_TRIG				(1 << 20)

/* GMAC min Inter Frame Gap values */
#define GMAC_IFG				12
#define MACSEC_IFG				(0x2D)
#define IFG_MASK				(0x3F)
#define GMAC_IFG_MIN_1000			10
#define GMAC_IFG_MIN_HALF_DUPLEX		8

/*
 * GMAC min Inter Frame Gap Limits.
 * In full duplex mode set to same value as IFG
*/
#define GMAC_IFG_LIMIT_HALF			12

/* QSGMII Specific defines */
#define QSGMII_REG_BASE				0x1bb00000
#define QSGMII_REG_LEN				0x0000FFFF

/* QSGMII Register offsets */
#define PCS_QSGMII_CTL				0x020
#define PCS_QSGMII_SGMII_MODE			0x064
#define PCS_MODE_CTL				0x068
#define PCS_QSGMII_MAC_STAT			0x074
#define PCS_ALL_CH_CTL				0x080
#define PCS_ALL_CH_STAT				0x084
#define PCS_CAL_LCKDT_CTL			0x120
#define PCS_CAL_LCKDT_CTL_STATUS		0x124
#define QSGMII_PHY_MODE_CTL			0x128
#define QSGMII_PHY_QSGMII_CTL			0x134
#define QSGMII_PHY_SGMII_1_CTL			0x13C
#define QSGMII_PHY_SGMII_2_CTL			0x140
#define QSGMII_PHY_SERDES_CTL			0x144

/* Bit definitions for PCS_QSGMII_CTL register */
#define PCS_CH0_SERDES_SN_DETECT		0x800
#define PCS_CHn_SERDES_SN_DETECT(n)		(PCS_CH0_SERDES_SN_DETECT << n)
#define PCS_CH0_SERDES_SN_DETECT_2		0x10000
#define PCS_CHn_SERDES_SN_DETECT_2(n)		(PCS_CH0_SERDES_SN_DETECT_2 << n)
#define PCS_QSGMII_DEPTH_THRESH_MASK		0x300
#define PCS_QSGMII_DEPTH_THRESH(n)		(n << 8)	/* Threshold for depth control */
#define PCS_QSGMII_SHORT_LATENCY		0x20
#define PCS_QSGMII_SHORT_THRESH			0x10
#define PCS_QSGMII_CUTTHROUGH_RX		0x8
#define PCS_QSGMII_CUTTHROUGH_TX		0x4
#define PCS_QSGMII_SW_VER_1_7			0x2
#define PCS_QSGMII_ATHR_CSCO_AUTONEG		0x1


/* Bit definitions for PCS_QSGMII_SGMII_MODE */
#define PCS_QSGMII_MODE_SGMII			(0x0 << 0)
#define PCS_QSGMII_MODE_QSGMII			(0x1 << 0)

/* Bit definitions for QSGMII_PHY_MODE_CTL */
#define QSGMII_PHY_MODE_SGMII			(0x0 << 0)
#define QSGMII_PHY_MODE_QSGMII			(0x1 << 0)

/* Bit definitions for PCS_MODE_CTL register */
#define PCS_MODE_CTL_BASE_X			0x00
#define PCS_MODE_CTL_SGMII_PHY			0x01
#define PCS_MODE_CTL_SGMII_MAC			0x02
#define PCS_MODE_CTL_CH0_PHY_RESET		0x10
#define PCS_MODE_CTL_CH0_PHY_LOOPBACK		0x20
#define PCS_MODE_CTL_CH0_AUTONEG_RESTART	0x40
#define PCS_MODE_CTL_CH0_AUTONEG_EN		0x80
#define PCS_MODE_CTL_CHn_PHY_RESET(n)		(PCS_MODE_CTL_CH0_PHY_RESET << (n * 8))
#define PCS_MODE_CTL_CHn_PHY_LOOPBACK(n)	(PCS_MODE_CTL_CH0_PHY_LOOPBACK << (n * 8))
#define PCS_MODE_CTL_CHn_AUTONEG_EN(n)		(PCS_MODE_CTL_CH0_AUTONEG_EN << (n * 8))
#define PCS_MODE_CTL_CHn_AUTONEG_RESTART(n)	(PCS_MODE_CTL_CH0_AUTONEG_RESTART << (n * 8))

/* Bit definitions for PCS_QSGMII_MAC_STAT register */
#define PCS_MAC_STAT_CH0_LINK				0x0001
#define PCS_MAC_STAT_CH0_DUPLEX				0x0002
#define PCS_MAC_STAT_CH0_SPEED_MASK			0x000C
#define PCS_MAC_STAT_CH0_PAUSE				0x0010
#define PCS_MAC_STAT_CH0_ASYM_PAUSE			0x0020
#define PCS_MAC_STAT_CH0_TX_PAUSE			0x0040
#define PCS_MAC_STAT_CH0_RX_PAUSE			0x0080
#define PCS_MAC_STAT_CHn_LINK(n)			(PCS_MAC_STAT_CH0_LINK << (n * 8))
#define PCS_MAC_STAT_CHn_DUPLEX(n)			(PCS_MAC_STAT_CH0_DUPLEX << (n * 8))
#define PCS_MAC_STAT_CHn_SPEED_MASK(n)			(PCS_MAC_STAT_CH0_SPEED_MASK << (n * 8))
#define PCS_MAC_STAT_CHn_SPEED(n, reg)			((reg & PCS_MAC_STAT_CHn_SPEED_MASK(n)) >> ((n * 8) + 2))
#define PCS_MAC_STAT_CHn_PAUSE				(PCS_MAC_STAT_CH0_PAUSE << (n * 8))
#define PCS_MAC_STAT_CHn_ASYM_PAUSE			(PCS_MAC_STAT_CH0_ASYM_PAUSE << (n * 8))
#define PCS_MAC_STAT_CHn_TX_PAUSE			(PCS_MAC_STAT_CH0_TX_PAUSE << (n * 8))
#define PCS_MAC_STAT_CHn_RX_PAUSE			(PCS_MAC_STAT_CH0_RX_PAUSE << (n * 8))

/* Bit definitions for PCS_ALL_CH_CTL register */
#define PCS_CH0_FORCE_SPEED			0x2
#define PCS_CHn_FORCE_SPEED(n)			(PCS_CH0_FORCE_SPEED << (n * 4))
#define PCS_CH0_SPEED_MASK			0xC
#define PCS_CHn_SPEED_MASK(n)			(PCS_CH0_SPEED_MASK << (n * 4))
#define PCS_CH_SPEED_10				0x0
#define PCS_CH_SPEED_100			0x4
#define PCS_CH_SPEED_1000			0x8
#define PCS_CHn_SPEED(ch, speed)		(speed << (ch * 4))

/* Bit definitions for PCS_ALL_CH_STAT register */
#define PCS_CH0_AUTONEG_COMPLETE		0x0040
#define PCS_CHn_AUTONEG_COMPLETE(n)		(PCS_CH0_AUTONEG_COMPLETE << (n * 8))


/* Bit definitions for PCS_CAL_LCKDT_CTL register */
#define PCS_LCKDT_RST				0x80000

/* Bit definitions for QSGMII_PHY_QSGMII_CTL register */
#define QSGMII_PHY_CDR_EN			0x00000001
#define QSGMII_PHY_RX_FRONT_EN			0x00000002
#define QSGMII_PHY_RX_SIGNAL_DETECT_EN		0x00000004
#define QSGMII_PHY_TX_DRIVER_EN			0x00000008
#define QSGMII_PHY_NEAR_END_LOOPBACK		0x00000020
#define QSGMII_PHY_FAR_END_LOOPBACK		0x00000040
#define QSGMII_PHY_QSGMII_EN			0x00000080
#define QSGMII_PHY_SLEW_RATE_CTL_MASK		0x00000300
#define QSGMII_PHY_SLEW_RATE_CTL(x)		(x << 8)
#define QSGMII_PHY_DEEMPHASIS_LVL_MASK		0x00000C00
#define QSGMII_PHY_DEEMPHASIS_LVL(x)		(x << 10)
#define QSGMII_PHY_PHASE_LOOP_GAIN_MASK		0x00007000
#define QSGMII_PHY_PHASE_LOOP_GAIN(x)		(x << 12)
#define QSGMII_PHY_RX_DC_BIAS_MASK		0x000C0000
#define QSGMII_PHY_RX_DC_BIAS(x)		(x << 18)
#define QSGMII_PHY_RX_INPUT_EQU_MASK		0x00300000
#define QSGMII_PHY_RX_INPUT_EQU(x)		(x << 20)
#define QSGMII_PHY_CDR_PI_SLEW_MASK		0x00C00000
#define QSGMII_PHY_CDR_PI_SLEW(x)		(x << 22)
#define QSGMII_PHY_SIG_DETECT_THRESH_MASK	0x03000000
#define QSGMII_PHY_SIG_DETECT_THRESH(x)		(x << 24)
#define QSGMII_PHY_TX_SLEW_MASK			0x0C000000
#define QSGMII_PHY_TX_SLEW(x)			(x << 26)
#define QSGMII_PHY_TX_DRV_AMP_MASK		0xF0000000
#define QSGMII_PHY_TX_DRV_AMP(x)		(x << 28)


/* Bit definitions for QSGMII_PHY_SERDES_CTL register */
#define SERDES_100MHZ_OSC_CLK			0x00000001
#define SERDES_LOCK_DETECT_EN			0x00000002
#define SERDES_PLL_EN				0x00000004
#define SERDES_VCO_MANUAL_CAL			0x00000008
#define SERDES_PLL_LOOP_FILTER_MASK		0x00000070
#define SERDES_PLL_LOOP_FILTER(x)		(x << 4)
#define SERDES_RSV_MASK				0x00FF0000
#define SERDES_RSV(x)				(x << 16)
#define SERDES_PLL_AMP_MASK			0x07000000
#define SERDES_PLL_AMP(x)			(x << 24)
#define SERDES_PLL_ICP_MASK			0x70000000
#define SERDES_PLL_ICP(x)			(x << 28)

/* Interface between GMAC and PHY */
#define GMAC_INTF_RGMII				0
#define GMAC_INTF_SGMII				1
#define GMAC_INTF_QSGMII			2

/* For MII<->MII Interfaces that do not use an Ethernet PHY */
#define NSS_GMAC_NO_MDIO_PHY			PHY_MAX_ADDR

/* GMAC phy interface profiles */
#define NSS_GMAC_PHY_PROFILE_2R_2S	0	/* 2 RGMII, 2 SGMII */
#define NSS_GMAC_PHY_PROFILE_1R_3S	1	/* 1 RGMII, 3 SGMII*/
#define NSS_GMAC_PHY_PROFILE_QS		2	/* 4 QSGMII */

struct msm_nss_gmac_platform_data {
	uint32_t phy_mdio_addr;			/* MDIO address of the connected PHY */
	uint32_t poll_required;			/* [0/1] Link status poll? */
	uint32_t rgmii_delay;
	uint32_t phy_mii_type;
	uint32_t emulation;			/* Running on emulation platform */
	uint8_t  mac_addr[6];
	int32_t forced_speed;			/* Forced speed. Values used from
						   ethtool.h. 0 = Speed not forced */
	int32_t forced_duplex;			/* Forced duplex. Values used from
						   ethtool.h. 0 = Duplex not forced. */
};

#define NSS_MAX_GMACS				4
#define IPQ806X_MDIO_BUS_NAME			"mdio-gpio"
#define IPQ806X_MDIO_BUS_NUM			0
#define IPQ806X_MDIO_BUS_MAX			1

#define IPQ806X_CLK_CTL_PHYS			0x00900000
#define IPQ806X_CLK_CTL_SIZE			SZ_16K
#define IPQ806X_TCSR_BASE			0x1A400000
#define IPQ806X_TCSR_SIZE			0xFFFF

#endif /*__ASM_NSS_GMAC_H */



