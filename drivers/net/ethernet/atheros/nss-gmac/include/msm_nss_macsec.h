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

#ifndef __ASM_NSS_MACSEC_H
#define __ASM_NSS_MACSEC_H

/* NSS MACSEC Base Addresses */
#define NSS_MACSEC1_BASE			0x37800000
#define NSS_MACSEC2_BASE			0x37A00000
#define NSS_MACSEC3_BASE			0x37C00000
#define NSS_MACSEC_REG_LEN			0x00200000

/* MACSEC REGS Offsets of NSS_CSR_REG_BASE */
#define NSS_MACSEC1_CORE_CLK_FS_CTL		0x001C
#define NSS_MACSEC2_CORE_CLK_FS_CTL		0x0020
#define NSS_MACSEC3_CORE_CLK_FS_CTL		0x0024
#define NSS_ETH_MACSEC_TEST_BUS_EN		0x0078
#define NSS_ETH_MACSEC_TEST_BUS_1		0x007c

/* MACSEC REG Offset of CLK_CTL_BASE */
#define MACSEC_CORE1_RESET			0x3E28
#define MACSEC_CORE2_RESET			0x3E2C
#define MACSEC_CORE3_RESET			0x3E30

/* NSSFB1 */
#define NSSFB1_CLK_CTL_ACR			0x1380
#define NSSFB1_PLL_ENA_APCS		0x34C0
#define NSSFB1_PLL14_MODE		0x31C0
#define NSSFB1_PLL14_L_VAL		0x31C4
#define NSSFB1_PLL14_M_VAL		0x31C8
#define NSSFB1_PLL14_N_VAL		0x31CC
#define NSSFB1_PLL14_TEST_CTL		0x31D0
#define NSSFB1_PLL14_CONFIG		0x31D4
#define NSSFB1_PLL14_STATUS		0x31D8
#define NSSFB1_PLL18_MODE		0x31A0
#define NSSFB1_PLL18_L_VAL		0x31A4
#define NSSFB1_PLL18_M_VAL		0x31A8
#define NSSFB1_PLL18_N_VAL		0x31AC
#define NSSFB1_PLL18_TEST_CTL		0x31B0
#define NSSFB1_PLL18_CONFIG		0x31B4
#define NSSFB1_PLL18_STATUS		0x31B8
#define NSSFB1_CLK_CTL_SRC_CTL		0x3BE0
#define NSSFB1_CLK_CTL_SRC0_NS		0x3BE4
#define NSSFB1_CLK_CTL_SRC1_NS		0x3BE8
#define NSSFB1_CLK_CTL			0x3C00

/* S_W_VAL in MACSEC_CORE_CLK_FS_CTL S_W_VAL */
#define MACSEC_CLK_FS_CTL_S_W_VAL		0x5
#define MACSEC_CLK_FS_CTL_S_W_VAL_MASK		0xF

/* MACSEC_CORE_RESET bit */
#define MACSEC_CORE_RESET_BIT			(1 << 0)

/* MACSEC COMMAND_CONFIG bit */
#define MACSEC_CMDCFG_ETH_SPEED_BIT		(1 << 3)
#define MACSEC_CMDCFG_ENA_10_BIT		(1 << 25)
#endif /*__ASM_NSS_MACSEC_H */

