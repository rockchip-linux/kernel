/*
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PLAT_ROCKCHIP_RV1108_DDRPCTL_PHY_H
#define __PLAT_ROCKCHIP_RV1108_DDRPCTL_PHY_H

/* DDR pctl register */
#define DDR_PCTL_SCFG			0x0000
#define DDR_PCTL_SCTL			0x0004
#define DDR_PCTL_STAT			0x0008
#define DDR_PCTL_MCMD			0x0040
#define DDR_PCTL_POWCTL			0x0044
#define DDR_PCTL_POWSTAT		0x0048
#define DDR_PCTL_CMDTSTATEN		0x0050
#define DDR_PCTL_MRRCFG0		0x0060
#define DDR_PCTL_MRRSTAT0		0x0064
#define DDR_PCTL_MRRSTAT1		0x0068
#define DDR_PCTL_MCFG1			0x007c
#define DDR_PCTL_MCFG			0x0080
#define DDR_PCTL_PPCFG			0x0084
#define DDR_PCTL_TOGCNT1U		0x00c0
#define DDR_PCTL_TINIT			0x00c4
#define DDR_PCTL_TRSTH			0x00c8
#define DDR_PCTL_TOGCNT100N		0x00cc
#define DDR_PCTL_TREFI			0x00d0
#define DDR_PCTL_TMRD			0x00d4
#define DDR_PCTL_TRFC			0x00d8
#define DDR_PCTL_TRP			0x00dc
#define DDR_PCTL_TRTW			0x00e0
#define DDR_PCTL_TAL			0x00e4
#define DDR_PCTL_TCL			0x00e8
#define DDR_PCTL_TCWL			0x00ec
#define DDR_PCTL_TRAS			0x00f0
#define DDR_PCTL_TRC			0x00f4
#define DDR_PCTL_TRCD			0x00f8
#define DDR_PCTL_TRRD			0x00fc
#define DDR_PCTL_TRTP			0x0100
#define DDR_PCTL_TWR			0x0104
#define DDR_PCTL_TWTR			0x0108
#define DDR_PCTL_TEXSR			0x010c
#define DDR_PCTL_TXP			0x0110
#define DDR_PCTL_TXPDLL			0x0114
#define DDR_PCTL_TZQCS			0x0118
#define DDR_PCTL_TZQCSI			0x011c
#define DDR_PCTL_TDQS			0x0120
#define DDR_PCTL_TCKSRE			0x0124
#define DDR_PCTL_TCKSRX			0x0128
#define DDR_PCTL_TCKE			0x012c
#define DDR_PCTL_TMOD			0x0130
#define DDR_PCTL_TRSTL			0x0134
#define DDR_PCTL_TZQCL			0x0138
#define DDR_PCTL_TMRR			0x013c
#define DDR_PCTL_TCKESR			0x0140
#define DDR_PCTL_TDPD			0x0144
#define DDR_PCTL_DFITCTRLDELAY		0x0240
#define DDR_PCTL_DFIODTCFG		0x0244
#define DDR_PCTL_DFIODTCFG1		0x0248
#define DDR_PCTL_DFIODTRANKMAP		0x024c
#define DDR_PCTL_DFITPHYWRDATA		0x0250
#define DDR_PCTL_DFITPHYWRLAT		0x0254
#define DDR_PCTL_DFITRDDATAEN		0x0260
#define DDR_PCTL_DFITPHYRDLAT           0x0264
#define DDR_PCTL_DFITPHYUPDTYPE0        0x0270
#define DDR_PCTL_DFITPHYUPDTYPE1        0x0274
#define DDR_PCTL_DFITPHYUPDTYPE2        0x0278
#define DDR_PCTL_DFITPHYUPDTYPE3        0x027c
#define DDR_PCTL_DFITCTRLUPDMIN         0x0280
#define DDR_PCTL_DFITCTRLUPDMAX         0x0284
#define DDR_PCTL_DFITCTRLUPDDLY         0x0288
#define DDR_PCTL_DFIUPDCFG		0x0290
#define DDR_PCTL_DFITREFMSKI            0x0294
#define DDR_PCTL_DFITCTRLUPDI           0x0298
#define DDR_PCTL_DFISTCFG0		0x02c4
#define DDR_PCTL_DFISTCFG1		0x02c8
#define DDR_PCTL_DFITDRAMCLKEN          0x02d0
#define DDR_PCTL_DFITDRAMCLKDIS         0x02d4
#define DDR_PCTL_DFISTCFG2		0x02d8
#define DDR_PCTL_DFILPCFG0		0x02f0

 /* SCTL */
#define INIT_STATE			(0)
#define CFG_STATE			(1)
#define GO_STATE			(2)
#define SLEEP_STATE			(3)
#define WAKEUP_STATE			(4)
/* STAT */
#define INIT_MEM			(0)
#define CONFIG				(1)
#define CONFIG_REG			(2)
#define ACCESS				(3)
#define ACCESS_REG			(4)
#define LOW_POWER			(5)
#define LOW_POWER_ENTRY_REQ		(6)
#define LOW_POWER_EXIT_REQ		(7)
#define GET_CTL_STAT(x)			(((x) >> 0) & 0x7)
#define GET_LP_TRIG(x)			(((x) >> 4) & 0x7)
/* MCFG */
#define mddr_lpddr2_clk_stop_idle(n)	((n) << 24)
#define pd_idle(n)			((n) << 8)
#define mddr_en				(2 << 22)
#define lpddr2_en			(3 << 22)
#define ddr2_en				(0 << 5)
#define ddr3_en				(1 << 5)
#define lpddr2_s2			(0 << 6)
#define lpddr2_s4			(1 << 6)
#define mddr_lpddr2_bl_2		(0 << 20)
#define mddr_lpddr2_bl_4		(1 << 20)
#define mddr_lpddr2_bl_8		(2 << 20)
#define mddr_lpddr2_bl_16		(3 << 20)
#define ddr2_ddr3_bl_4			(0)
#define ddr2_ddr3_bl_8			(1)
#define tfaw_cfg(n)			(((n) - 4) << 18)
#define pd_exit_slow			(0 << 17)
#define pd_exit_fast			(1 << 17)
#define pd_type(n)			((n) << 16)
#define two_t_en(n)			((n) << 3)
#define bl8int_en(n)			((n) << 2)
#define cke_or_en(n)			((n) << 1)
/* POWCTL */
#define power_up_start			(1 << 0)
/* POWSTAT */
#define power_up_done			(1 << 0)
/* DFISTSTAT0 */
#define dfi_init_complete		(1 << 0)
/* CMDTSTAT */
#define cmd_tstat			(1 << 0)
/* CMDTSTATEN */
#define cmd_tstat_en			(1 << 1)
/* MCMD */
#define DESELECT_CMD			(0)
#define PREA_cmd			(1)
#define REF_cmd				(2)
#define MRS_cmd				(3)
#define ZQCS_cmd			(4)
#define ZQCL_cmd			(5)
#define RSTL_cmd			(6)
#define MRR_cmd				(8)
#define DPDE_cmd			(9)
#define lpddr2_op(n)			((n) << 12)
#define lpddr2_ma(n)			((n) << 4)
#define bank_addr(n)			((n) << 17)
#define cmd_addr(n)			((n) << 4)
#define start_cmd			(1u << 31)
/* DDR phy register */
#define	PHYREG00	0x0
#define PHYREG01	0x4
#define PHYREG02	0x8
#define PHYREG03	0xc
#define PHYREG04	0x10
#define PHYREG05	0x14
#define PHYREG06	0x18
#define PHYREG09	0x24
#define PHYREG0b	0x2c
#define PHYREG0c	0x30
#define PHYREG11	0x44
#define PHYREG12	0x48
#define PHYREG13	0x4c
#define PHYREG14	0x50
#define PHYREG16	0x58
#define PHYREG17	0x5c
#define PHYREG18	0x60
#define PHYREG20	0x80
#define PHYREG21	0x84
#define PHYREG26	0x98
#define PHYREG27	0x9c
#define PHYREG28	0xa0
#define PHYREG2b	0xac
#define PHYREG2e	0xb8
#define PHYREG2f	0xbc
#define PHYREG30	0xc0
#define PHYREG31	0xc4
#define PHYREG36	0xd8
#define PHYREG37	0xdc
#define PHYREG38	0xe0
#define PHYREG3b	0xec
#define PHYREG3e	0xf8
#define PHYREG3f	0xfc
#define PHYREG40	0x100
#define PHYREG41	0x104
#define PHYREG46	0x118
#define PHYREG47	0x11c
#define PHYREG48	0x120
#define PHYREG4b	0x12c
#define PHYREG4e	0x138
#define PHYREG4f	0x13c
#define PHYREG50	0x140
#define PHYREG51	0x144
#define PHYREG56	0x158
#define PHYREG57	0x15c
#define PHYREG58	0x160
#define PHYREG5b	0x16c
#define PHYREG5e	0x178
#define PHYREG5f	0x17c
#define PHYREG70	0x1c0
#define PHYREG71	0x1c4
#define PHYREG72	0x1c8
#define PHYREG73	0x1cc
#define PHYREG74	0x1d0
#define PHYREG75	0x1d4
#define PHYREG76	0x1d8
#define PHYREG77	0x1dc
#define PHYREG78	0x1e0
#define PHYREG79	0x1e4
#define PHYREG7A	0x1e8
#define PHYREG7B	0x1ec
#define PHYREG7C	0x1f0
#define PHYREG7D	0x1f4
#define PHYREG7E	0x1f8
#define PHYREG7F	0x1fc
#define PHYREG80	0x200
#define PHYREG81	0x204
#define PHYREG82	0x208
#define PHYREG83	0x20c
#define PHYREG84	0x210
#define PHYREG85	0x214
#define PHYREG86	0x218
#define PHYREG87	0x21c
#define PHYREG88	0x220
#define PHYREG89	0x224
#define PHYREG8A	0x228
#define PHYREG8B	0x22c
#define PHYREG8C	0x230
#define PHYREG8D	0x234
#define PHYREG8E	0x238
#define PHYREG8F	0x23c
#define PHYREG90	0x240
#define PHYREG91	0x244
#define PHYREG92	0x248
#define PHYREG93	0x24c
#define PHYREG94	0x250
#define PHYREG95	0x254
#define PHYREG96	0x258
#define PHYREG97	0x25c
#define PHYREG98	0x260
#define PHYREG99	0x264
#define PHYREG9A	0x268
#define PHYREG9B	0x26c
#define PHYREGDLL	0x290
#define PHYREGB0	0x2c0
#define PHYREGB1	0x2c4
#define PHYREGB2	0x2c8
#define PHYREGB3	0x2cc
#define PHYREGB4	0x2d0
#define PHYREGB5	0x2d4
#define PHYREGB6	0x2d8
#define PHYREGB7	0x2dc
#define PHYREGB8	0x2e0
#define PHYREGB9	0x2e4
#define PHYREGBA	0x2e8
#define PHYREGBB	0x2ec
#define PHYREGBC	0x2f0
#define PHYREGBD	0x2f4
#define PHYREGBE	0x2f8
#define PHYREGC0	0x300
#define PHYREGC1	0x304
#define PHYREGC2	0x308
#define PHYREGC3	0x30c
#define PHYREGC4	0x310
#define PHYREGC5	0x314
#define PHYREGC6	0x318
#define PHYREGC7	0x31c
#define PHYREGC8	0x320
#define PHYREGC9	0x324
#define PHYREGCA	0x328
#define PHYREGCB	0x32c
#define PHYREGCC	0x330
#define PHYREGCD	0x334
#define PHYREGCE	0x338
#define PHYREGCF	0x33c
#define PHYREGD0	0x340
#define PHYREGD1	0x344
#define PHYREGD2	0x348
#define PHYREGD3	0x34c
#define PHYREGD4	0x350
#define PHYREGD5	0x354
#define PHYREGD6	0x358
#define PHYREGD7	0x35c
#define PHYREGD8	0x360
#define PHYREGD9	0x364
#define PHYREGDA	0x368
#define PHYREGDB	0x36c
#define PHYREGDC	0x370
#define PHYREGDD	0x374
#define PHYREGDE	0x378
#define PHYREGDF	0x37c
#define PHYREGE0	0x380
#define PHYREGE1	0x384
#define PHYREGE2	0x388
#define PHYREGE3	0x38c
#define PHYREGE4	0x390
#define PHYREGE5	0x394
#define PHYREGE6	0x398
#define PHYREGE7	0x39c
#define PHYREGE8	0x3a0
#define PHYREGE9	0x3a4
#define PHYREGEA	0x3a8
#define PHYREGEB	0x3ac
#define PHYREGF0	0x3c0
#define PHYREGF1	0x3c4
#define PHYREGF2	0x3c8
#define PHYREGFA	0x3e8
#define PHYREGFB	0x3ec
#define PHYREGFC	0x3f0
#define PHYREGFD	0x3f4
#define PHYREGFE	0x3f8
#define PHYREGFF	0x3fc
/* PHYREG0 */
#define GET_PHY_BW(x)			((x >> 4) & 0xf)
#define SOFT_DERESET_DIGITAL		(1<<3)
#define SOFT_DERESET_ANALOG		(1<<2)
/* PHY_REG1 */
#define PHY_DDR2			(1)
#define PHY_DDR3			(0)
#define PHY_LPDDR2_3			(2)
/* PHY_REG2 */
#define PHY_DTT_EN			(1<<0)
#define PHY_DTT_DISB			(0<<0)
#define PHY_WRITE_LEVELING_EN		(1<<2)
#define PHY_WRITE_LEVELING_DISB		(0<<2)
#define PHY_SELECT_CS0			(2)
#define PHY_SELECT_CS1			(1)
#define PHY_SELECT_CS0_1		(0)
#define PHY_WRITE_LEVELING_SELECTCS(n)	(n<<6)
#define PHY_DATA_TRAINING_SELECTCS(n)	(n<<4)
#define PHY_BL_4			(0<<2)
#define PHY_BL_8			(1<<2)
#define PHY_CL(n)			(((n)&0xF)<<4)
#define PHY_AL(n)			((n)&0xF)

/* PHY DRV ODT strength*/
#define PHY_DDR3_RON_RTT_DISABLE	(0)
#define PHY_DDR3_RON_RTT_451ohm		(1)
#define PHY_DDR3_RON_RTT_225ohm		(2)
#define PHY_DDR3_RON_RTT_150ohm		(3)
#define PHY_DDR3_RON_RTT_112ohm		(4)
#define PHY_DDR3_RON_RTT_90ohm		(5)
#define PHY_DDR3_RON_RTT_75ohm		(6)
#define PHY_DDR3_RON_RTT_64ohm		(7)
#define PHY_DDR3_RON_RTT_56ohm		(16)
#define PHY_DDR3_RON_RTT_50ohm		(17)
#define PHY_DDR3_RON_RTT_45ohm		(18)
#define PHY_DDR3_RON_RTT_41ohm		(19)
#define PHY_DDR3_RON_RTT_37ohm		(20)
#define PHY_DDR3_RON_RTT_34ohm		(21)
#define PHY_DDR3_RON_RTT_33ohm		(22)
#define PHY_DDR3_RON_RTT_30ohm		(23)
#define PHY_DDR3_RON_RTT_28ohm		(24)
#define PHY_DDR3_RON_RTT_26ohm		(25)
#define PHY_DDR3_RON_RTT_25ohm		(26)
#define PHY_DDR3_RON_RTT_23ohm		(27)
#define PHY_DDR3_RON_RTT_22ohm		(28)
#define PHY_DDR3_RON_RTT_21ohm		(29)
#define PHY_DDR3_RON_RTT_20ohm		(30)
#define PHY_DDR3_RON_RTT_19ohm		(31)

#define PHY_LP23_RON_RTT_DISABLE	(0)
#define PHY_LP23_RON_RTT_480ohm		(1)
#define PHY_LP23_RON_RTT_240ohm		(2)
#define PHY_3_RON_RTT_160ohm		(3)
#define PHY_LP23_RON_RTT_120ohm		(4)
#define PHY_LP23_RON_RTT_96ohm		(5)
#define PHY_LP23_RON_RTT_80ohm		(6)
#define PHY_LP23_RON_RTT_68ohm		(7)
#define PHY_LP23_RON_RTT_60ohm		(16)
#define PHY_LP23_RON_RTT_53ohm		(17)
#define PHY_LP23_RON_RTT_48ohm		(18)
#define PHY_LP23_RON_RTT_43ohm		(19)
#define PHY_LP23_RON_RTT_40ohm		(20)
#define PHY_LP23_RON_RTT_37ohm		(21)
#define PHY_LP23_RON_RTT_34ohm		(22)
#define PHY_LP23_RON_RTT_32ohm		(23)
#define PHY_LP23_RON_RTT_30ohm		(24)
#define PHY_LP23_RON_RTT_28ohm		(25)
#define PHY_LP23_RON_RTT_26ohm		(26)
#define PHY_LP23_RON_RTT_25ohm		(27)
#define PHY_LP23_RON_RTT_24ohm		(28)
#define PHY_LP23_RON_RTT_22ohm		(29)
#define PHY_LP23_RON_RTT_21ohm		(30)
#define PHY_LP23_RON_RTT_20ohm		(31)

/* DDR msch register */
#define DDR_MSCH_DDRCONF		0x0008
#define DDR_MSCH_DDRTIMING		0x000c
#define DDR_MSCH_DDRMODE		0x0010
#define DDR_MSCH_READLATENCY		0x0014
#define DDR_MSCH_ACTIVATE		0x0038
#define DDR_MSCH_DEVTODEV		0x003c
#endif /* __MACH_ROCKCHIP_RV1108_DDR_H */
