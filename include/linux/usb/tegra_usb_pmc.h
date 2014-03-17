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

#ifndef _TEGRA_USB_PMC_INTERFACE_H_
#define _TEGRA_USB_PMC_INTERFACE_H_

#define UHSIC_INST(inst, x, y)	((inst == 1) ? x : y)

#define PMC_UHSIC_SLEEP_CFG(inst)	UHSIC_INST(inst, 0x1fc, 0x284)
#define UHSIC_MASTER_ENABLE(inst)	(1 << UHSIC_INST(inst, 24, 0))
#define UHSIC_WAKE_VAL(inst, x)		(((x) & 0xf) << UHSIC_INST(inst, 28, 4))
#define WAKE_VAL_SD10			0x2
#define UTMIP_PMC_WAKEUP0		0x84c
#define UHSIC_PMC_WAKEUP0		0xc34
#define EVENT_INT_ENB			(1 << 0)

#define PMC_UHSIC_TRIGGERS(inst)	UHSIC_INST(inst, 0x1ec, 0x27c)
#define UHSIC_CLR_WALK_PTR(inst)	(1 << UHSIC_INST(inst, 3, 0))
#define UHSIC_CLR_WAKE_ALARM(inst)	(1 << UHSIC_INST(inst, 15, 3))

#define PMC_UHSIC_SLEEPWALK_CFG(inst)	UHSIC_INST(inst, 0x200, 0x288)
#define UHSIC_LINEVAL_WALK_EN(inst)	(1 << UHSIC_INST(inst, 31, 7))

#define PMC_UHSIC_MASTER_CONFIG(inst)	UHSIC_INST(inst, 0x274, 0x29c)
#define UHSIC_PWR(inst)			(1 << UHSIC_INST(inst, 3, 0))

#define PMC_UHSIC_FAKE(inst)		UHSIC_INST(inst, 0x218, 0x294)
#define UHSIC_FAKE_STROBE_VAL(inst)	(1 << UHSIC_INST(inst, 12, 0))
#define UHSIC_FAKE_DATA_VAL(inst)	(1 << UHSIC_INST(inst, 13, 1))

#define PMC_SLEEPWALK_UHSIC(inst)	UHSIC_INST(inst, 0x210, 0x28c)
#define UHSIC_STROBE_RPD_A		(1 << 0)
#define UHSIC_DATA_RPD_A		(1 << 1)
#define UHSIC_STROBE_RPU_A		(1 << 2)
#define UHSIC_DATA_RPU_A		(1 << 3)
#define UHSIC_STROBE_RPD_B		(1 << 8)
#define UHSIC_DATA_RPD_B		(1 << 9)
#define UHSIC_STROBE_RPU_B		(1 << 10)
#define UHSIC_DATA_RPU_B		(1 << 11)
#define UHSIC_STROBE_RPD_C		(1 << 16)
#define UHSIC_DATA_RPD_C		(1 << 17)
#define UHSIC_STROBE_RPU_C		(1 << 18)
#define UHSIC_DATA_RPU_C		(1 << 19)
#define UHSIC_STROBE_RPD_D		(1 << 24)
#define UHSIC_DATA_RPD_D		(1 << 25)
#define UHSIC_STROBE_RPU_D		(1 << 26)
#define UHSIC_DATA_RPU_D		(1 << 27)
#define UHSIC_LINE_DEB_CNT(x)		(((x) & 0xf) << 20)

#define PMC_USB_DEBOUNCE		0xec
#define UTMIP_LINE_DEB_CNT(x)		(((x) & 0xf) << 16)
#define PMC_USB_DEBOUNCE_VAL(x)		((x) & 0xffff)

#define PMC_USB_AO			0xf0
#define HSIC_RESERVED(inst)		(3 << UHSIC_INST(inst, 14, 18))
#define STROBE_VAL_PD(inst)		(1 << UHSIC_INST(inst, 12, 16))
#define DATA_VAL_PD(inst)		(1 << UHSIC_INST(inst, 13, 17))
#define PMC_POWER_DOWN_MASK		0xffff
#define USB_ID_PD(inst)			(1 << ((4 * (inst)) + 3))
#define VBUS_WAKEUP_PD(inst)		(1 << ((4 * (inst)) + 2))
#define USBON_VAL_PD(inst)		(1 << ((4 * (inst)) + 1))
#define USBON_VAL_PD_P2			(1 << 9)
#define USBON_VAL_PD_P1			(1 << 5)
#define USBON_VAL_PD_P0			(1 << 1)
#define USBOP_VAL_PD(inst)		(1 << (4 * (inst)))
#define USBOP_VAL_PD_P2			(1 << 8)
#define USBOP_VAL_PD_P1			(1 << 4)
#define USBOP_VAL_PD_P0			(1 << 0)
#define PMC_USB_AO_PD_P2		(0xf << 8)
#define PMC_USB_AO_ID_PD_P0		(1 << 3)
#define PMC_USB_AO_VBUS_WAKEUP_PD_P0	(1 << 2)

#define PMC_TRIGGERS			0x1ec
#define UTMIP_CLR_WALK_PTR(inst)	(1 << (inst))
#define UTMIP_CLR_WALK_PTR_P2		(1 << 2)
#define UTMIP_CLR_WALK_PTR_P1		(1 << 1)
#define UTMIP_CLR_WALK_PTR_P0		(1 << 0)
#define UTMIP_CAP_CFG(inst)		(1 << ((inst) + 4))
#define UTMIP_CAP_CFG_P2		(1 << 6)
#define UTMIP_CAP_CFG_P1		(1 << 5)
#define UTMIP_CAP_CFG_P0		(1 << 4)
#define UTMIP_CLR_WAKE_ALARM(inst)	(1 << ((inst) + 12))
#define UTMIP_CLR_WAKE_ALARM_P2		(1 << 14)

#define PMC_PAD_CFG			0x1f4

#define PMC_UTMIP_TERM_PAD_CFG		0x1f8
#define PMC_TCTRL_VAL(x)		(((x) & 0x1f) << 5)
#define PMC_RCTRL_VAL(x)		(((x) & 0x1f) << 0)

#define PMC_SLEEP_CFG			0x1fc
#define UTMIP_TCTRL_USE_PMC(inst)	(1 << ((8 * (inst)) + 3))
#define UTMIP_TCTRL_USE_PMC_P2		(1 << 19)
#define UTMIP_TCTRL_USE_PMC_P1		(1 << 11)
#define UTMIP_TCTRL_USE_PMC_P0		(1 << 3)
#define UTMIP_RCTRL_USE_PMC(inst)	(1 << ((8 * (inst)) + 2))
#define UTMIP_RCTRL_USE_PMC_P2		(1 << 18)
#define UTMIP_RCTRL_USE_PMC_P1		(1 << 10)
#define UTMIP_RCTRL_USE_PMC_P0		(1 << 2)
#define UTMIP_FSLS_USE_PMC(inst)	(1 << ((8 * (inst)) + 1))
#define UTMIP_FSLS_USE_PMC_P2		(1 << 17)
#define UTMIP_FSLS_USE_PMC_P1		(1 << 9)
#define UTMIP_FSLS_USE_PMC_P0		(1 << 1)
#define UTMIP_MASTER_ENABLE(inst)	(1 << (8 * (inst)))
#define UTMIP_MASTER_ENABLE_P2		(1 << 16)
#define UTMIP_MASTER_ENABLE_P1		(1 << 8)
#define UTMIP_MASTER_ENABLE_P0		(1 << 0)

#define PMC_SLEEPWALK_CFG		0x200
#define UTMIP_LINEVAL_WALK_EN(inst)	(1 << ((8 * (inst)) + 7))
#define UTMIP_LINEVAL_WALK_EN_P2	(1 << 23)
#define UTMIP_LINEVAL_WALK_EN_P1	(1 << 15)
#define UTMIP_LINEVAL_WALK_EN_P0	(1 << 7)
#define UTMIP_WAKE_VAL(inst, x)		(((x) & 0xf) << ((8 * (inst)) + 4))
#define UTMIP_WAKE_VAL_P2(x)		(((x) & 0xf) << 20)
#define UTMIP_WAKE_VAL_P1(x)		(((x) & 0xf) << 12)
#define UTMIP_WAKE_VAL_P0(x)		(((x) & 0xf) << 4)
#define WAKE_VAL_NONE			0xc
#define WAKE_VAL_ANY			0xF
#define WAKE_VAL_FSJ			0x2
#define WAKE_VAL_FSK			0x1
#define WAKE_VAL_SE0			0x0

#define PMC_SLEEPWALK_REG(inst)	(0x204 + (4 * (inst)))
#define UTMIP_USBOP_RPD_A	(1 << 0)
#define UTMIP_USBON_RPD_A	(1 << 1)
#define UTMIP_AP_A		(1 << 4)
#define UTMIP_AN_A		(1 << 5)
#define UTMIP_HIGHZ_A		(1 << 6)
#define UTMIP_USBOP_RPD_B	(1 << 8)
#define UTMIP_USBON_RPD_B	(1 << 9)
#define UTMIP_AP_B		(1 << 12)
#define UTMIP_AN_B		(1 << 13)
#define UTMIP_HIGHZ_B		(1 << 14)
#define UTMIP_USBOP_RPD_C	(1 << 16)
#define UTMIP_USBON_RPD_C	(1 << 17)
#define UTMIP_AP_C		(1 << 20)
#define UTMIP_AN_C		(1 << 21)
#define UTMIP_HIGHZ_C		(1 << 22)
#define UTMIP_USBOP_RPD_D	(1 << 24)
#define UTMIP_USBON_RPD_D	(1 << 25)
#define UTMIP_AP_D		(1 << 28)
#define UTMIP_AN_D		(1 << 29)
#define UTMIP_HIGHZ_D		(1 << 30)

#define PMC_UTMIP_FAKE		0x218
#define USBON_VAL(inst)		(1 << ((4 * (inst)) + 1))
#define USBON_VAL_P2		(1 << 9)
#define USBON_VAL_P1		(1 << 5)
#define USBON_VAL_P0		(1 << 1)
#define USBOP_VAL(inst)		(1 << (4 * (inst)))
#define USBOP_VAL_P2		(1 << 8)
#define USBOP_VAL_P1		(1 << 4)
#define USBOP_VAL_P0		(1 << 0)

#define PMC_UTMIP_BIAS_MASTER_CNTRL	0x270
#define BIAS_MASTER_PROG_VAL		(1 << 1)

#define PMC_UTMIP_MASTER_CONFIG		0x274
#define UTMIP_PWR(inst)			(1 << (inst))

#endif /* _TEGRA_USB_PMC_INTERFACE_H_ */
