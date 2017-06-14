/*
 * rk818_battery.h: fuel gauge driver structures
 *
 * Copyright (C) 2016 Rockchip Electronics Co., Ltd
 * Author: chenjh <chenjh@rock-chips.com>
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

#ifndef RK818_BATTERY
#define RK818_BATTERY

#define RK818_VB_MOD_REG			0x21
#define RK818_THERMAL_REG			0x22
#define RK818_DCDC_EN_REG			0x23
#define RK818_DCDC_ILMAX_REG			0x90
#define RK818_CHRG_COMP_REG1			0x99
#define RK818_CHRG_COMP_REG2			0x9A
#define RK818_SUP_STS_REG			0xA0
#define RK818_USB_CTRL_REG			0xA1
#define RK818_CHRG_CTRL_REG1			0xA3
#define RK818_CHRG_CTRL_REG2			0xA4
#define RK818_CHRG_CTRL_REG3			0xA5
#define RK818_BAT_CTRL_REG			0xA6
#define RK818_BAT_HTS_TS1_REG			0xA8
#define RK818_BAT_LTS_TS1_REG			0xA9
#define RK818_BAT_HTS_TS2_REG			0xAA
#define RK818_BAT_LTS_TS2_REG			0xAB
#define RK818_TS_CTRL_REG			0xAC
#define RK818_ADC_CTRL_REG			0xAD
#define RK818_ON_SOURCE_REG			0xAE
#define RK818_OFF_SOURCE_REG			0xAF
#define RK818_GGCON_REG				0xB0
#define RK818_GGSTS_REG				0xB1
#define RK818_FRAME_SMP_INTERV_REG		0xB2
#define RK818_AUTO_SLP_CUR_THR_REG		0xB3
#define RK818_GASCNT_CAL_REG3			0xB4
#define RK818_GASCNT_CAL_REG2			0xB5
#define RK818_GASCNT_CAL_REG1			0xB6
#define RK818_GASCNT_CAL_REG0			0xB7
#define RK818_GASCNT3_REG			0xB8
#define RK818_GASCNT2_REG			0xB9
#define RK818_GASCNT1_REG			0xBA
#define RK818_GASCNT0_REG			0xBB
#define RK818_BAT_CUR_AVG_REGH			0xBC
#define RK818_BAT_CUR_AVG_REGL			0xBD
#define RK818_TS1_ADC_REGH			0xBE
#define RK818_TS1_ADC_REGL			0xBF
#define RK818_TS2_ADC_REGH			0xC0
#define RK818_TS2_ADC_REGL			0xC1
#define RK818_BAT_OCV_REGH			0xC2
#define RK818_BAT_OCV_REGL			0xC3
#define RK818_BAT_VOL_REGH			0xC4
#define RK818_BAT_VOL_REGL			0xC5
#define RK818_RELAX_ENTRY_THRES_REGH		0xC6
#define RK818_RELAX_ENTRY_THRES_REGL		0xC7
#define RK818_RELAX_EXIT_THRES_REGH		0xC8
#define RK818_RELAX_EXIT_THRES_REGL		0xC9
#define RK818_RELAX_VOL1_REGH			0xCA
#define RK818_RELAX_VOL1_REGL			0xCB
#define RK818_RELAX_VOL2_REGH			0xCC
#define RK818_RELAX_VOL2_REGL			0xCD
#define RK818_BAT_CUR_R_CALC_REGH		0xCE
#define RK818_BAT_CUR_R_CALC_REGL		0xCF
#define RK818_BAT_VOL_R_CALC_REGH		0xD0
#define RK818_BAT_VOL_R_CALC_REGL		0xD1
#define RK818_CAL_OFFSET_REGH			0xD2
#define RK818_CAL_OFFSET_REGL			0xD3
#define RK818_NON_ACT_TIMER_CNT_REG		0xD4
#define RK818_VCALIB0_REGH			0xD5
#define RK818_VCALIB0_REGL			0xD6
#define RK818_VCALIB1_REGH			0xD7
#define RK818_VCALIB1_REGL			0xD8
#define RK818_IOFFSET_REGH			0xDD
#define RK818_IOFFSET_REGL			0xDE
#define RK818_SOC_REG				0xE0
#define RK818_REMAIN_CAP_REG3			0xE1
#define RK818_REMAIN_CAP_REG2			0xE2
#define RK818_REMAIN_CAP_REG1			0xE3
#define RK818_REMAIN_CAP_REG0			0xE4
#define RK818_UPDAT_LEVE_REG			0xE5
#define RK818_NEW_FCC_REG3			0xE6
#define RK818_NEW_FCC_REG2			0xE7
#define RK818_NEW_FCC_REG1			0xE8
#define RK818_NEW_FCC_REG0			0xE9
#define RK818_NON_ACT_TIMER_CNT_SAVE_REG	0xEA
#define RK818_OCV_VOL_VALID_REG			0xEB
#define RK818_REBOOT_CNT_REG			0xEC
#define RK818_POFFSET_REG			0xED
#define RK818_MISC_MARK_REG			0xEE
#define RK818_HALT_CNT_REG			0xEF
#define RK818_CALC_REST_REGH			0xF0
#define RK818_CALC_REST_REGL			0xF1

/* RK818_INT_STS_MSK_REG2 */
#define PLUG_IN_MSK		BIT(0)
#define PLUG_OUT_MSK		BIT(1)
#define CHRG_CVTLMT_INT_MSK	BIT(6)

/* RK818_TS_CTRL_REG */
#define GG_EN			BIT(7)
#define ADC_CUR_EN		BIT(6)
#define TS2_FUN_ADC		BIT(5)

/* RK818_ADC_CTRL_REG */
#define ADC_TS1_EN		BIT(5)
#define ADC_TS2_EN		BIT(4)

/* RK818_GGCON */
#define OCV_SAMP_MIN_MSK	0x0c
#define OCV_SAMP_8MIN		(0x00 << 2)
#define ADC_CAL_MIN_MSK		0x30
#define ADC_CAL_8MIN		(0x00 << 4)
#define ADC_CUR_MODE		BIT(1)

/* RK818_GGSTS */
#define BAT_CON			BIT(4)
#define RELAX_VOL1_UPD		BIT(3)
#define RELAX_VOL2_UPD		BIT(2)
#define RELAX_VOL12_UPD_MSK	(RELAX_VOL1_UPD | RELAX_VOL2_UPD)

/* RK818_SUP_STS_REG */
#define CHRG_STATUS_MSK		0x70
#define BAT_EXS			BIT(7)
#define CHARGE_OFF		(0x0 << 4)
#define DEAD_CHARGE		(0x1 << 4)
#define TRICKLE_CHARGE		(0x2 << 4)
#define CC_OR_CV		(0x3 << 4)
#define CHARGE_FINISH		(0x4 << 4)
#define USB_OVER_VOL		(0x5 << 4)
#define BAT_TMP_ERR		(0x6 << 4)
#define TIMER_ERR		(0x7 << 4)
#define USB_VLIMIT_EN		BIT(3)
#define USB_CLIMIT_EN		BIT(2)
#define USB_EXIST		BIT(1)
#define USB_EFF			BIT(0)

/* RK818_USB_CTRL_REG */
#define CHRG_CT_EN		BIT(7)
#define FINISH_CUR_MSK		0xc0
#define TEMP_105C		(0x02 << 2)
#define TEMP_115C		(0x03 << 2)
#define FINISH_100MA		(0x00 << 6)
#define FINISH_150MA		(0x01 << 6)
#define FINISH_200MA		(0x02 << 6)
#define FINISH_250MA		(0x03 << 6)
#define INPUT_CUR_MSK		(0x0f)

/* RK818_CHRG_CTRL_REG3 */
#define CHRG_TERM_MODE_MSK	BIT(5)
#define CHRG_TERM_ANA_SIGNAL	(0 << 5)
#define CHRG_TERM_DIG_SIGNAL	BIT(5)
#define CHRG_TIMER_CCCV_EN	BIT(2)

/* RK818_DCDC_EN_REG */
#define OTG_EN_MASK		(1 << 7)
#define CHRG_EN			BIT(7)

/* RK818_VB_MON_REG */
#define	RK818_VBAT_LOW_3V0      0x02
#define	RK818_VBAT_LOW_3V4      0x06
#define PLUG_IN_STS		BIT(6)

/* RK818_THERMAL_REG */
#define FB_TEMP_MSK		0x0c

/* RK818_INT_STS_MSK_REG1 */
#define VB_LOW_INT_EN		BIT(1)

/* RK818_MISC_MARK_REG */
#define FG_INIT			BIT(5)
#define FG_RESET_LATE		BIT(4)
#define FG_RESET_NOW		BIT(3)
#define ALGO_REST_MODE_MSK	(0xc0)
#define ALGO_REST_MODE_SHIFT	6

/* RK818_CHGR_CUR_INPUT */
#define INPUT_CUR450MA		(0x00)
#define INPUT_CUR80MA		(0x01)
#define INPUT_CUR850MA		(0x02)
#define INPUT_CUR2000MA		(0x07)
#define CHRG_CUR1400MA		(0x02)
#define CHRG_VOL4200MV		(0x03 << 4)

/* bit shift */
#define FB_TEMP_SHIFT		2
#define CHRG_VOL_SEL_SHIFT	4
#define CHRG_CRU_INPUT_SHIFT	0
#define CHRG_CRU_SEL_SHIFT	0

/* parse ocv table param */
#define TIMER_MS_COUNTS		1000
#define MAX_PERCENTAGE		100
#define MAX_INTERPOLATE		1000
#define MAX_INT			0x7FFF

#define DRIVER_VERSION		"7.0"

struct battery_platform_data {
	u32 *ocv_table;
	u32 *zero_table;
	u32 *ntc_table;
	u32 ocv_size;
	u32 ntc_size;
	int ntc_degree_from;
	u32 max_input_current;
	u32 max_chrg_current;
	u32 max_chrg_voltage;
	u32 lp_input_current;
	u32 lp_soc_min;
	u32 lp_soc_max;
	u32 pwroff_vol;
	u32 monitor_sec;
	u32 zero_algorithm_vol;
	u32 zero_reserve_dsoc;
	u32 bat_res;
	u32 design_capacity;
	u32 design_qmax;
	u32 sleep_enter_current;
	u32 sleep_exit_current;
	u32 power_dc2otg;
	u32 max_soc_offset;
	u32 bat_mode;
	u32 fb_temp;
	u32 energy_mode;
	u32 cccv_hour;
	int dc_det_pin;
	u8  dc_det_level;
	u32 ts2_vol_multi;
};

enum work_mode {
	MODE_ZERO = 0,
	MODE_FINISH,
	MODE_SMOOTH_CHRG,
	MODE_SMOOTH_DISCHRG,
	MODE_SMOOTH,
};

enum bat_mode {
	MODE_BATTARY = 0,
	MODE_VIRTUAL,
};

enum charger_type {
	UNKNOWN_CHARGER = 0,
	USB_DC_TYPE_NONE_CHARGER,
	USB_TYPE_NONE_CHARGER,
	USB_TYPE_USB_CHARGER,
	USB_TYPE_AC_CHARGER,
	DC_TYPE_DC_CHARGER,
	DC_TYPE_NONE_CHARGER,
};

enum charger_state {
	OFFLINE = 0,
	ONLINE
};

static const u16 FEED_BACK_TEMP[] = {
	85, 95, 105, 115
};

static const u16 CHRG_VOL_SEL[] = {
	4050, 4100, 4150, 4200, 4300, 4350
};

static const u16 CHRG_CUR_SEL[] = {
	1000, 1200, 1400, 1600, 1800, 2000, 2250, 2400, 2600, 2800, 3000
};

static const u16 CHRG_CUR_INPUT[] = {
	450, 80, 850, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000
};

void kernel_power_off(void);
void rk_send_wakeup_key(void);

#endif
