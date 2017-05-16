/*
 * rk818/rk819 battery driver
 *
 *  Copyright (C) 2014 Rockchip Electronics Co., Ltd
 *  Author: zhangqing <zhangqing@rock-chips.com>
 *	    chenjh    <chenjh@rock-chips.com>
 *          Andy Yan  <andy.yan@rock-chips.com>
 *
 *  Copyright (C) 2008-2009 Texas Instruments, Inc.
 *  Author: Texas Instruments, Inc.
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 * Author: Texas Instruments, Inc.
 * Copyright (c) 2014, Fuzhou Rockchip Electronics Co., Ltd
 * Author: zhangqing <zhangqing@rock-chips.com>
 * Copyright (C) 2014-2015 Intel Mobile Communications GmbH
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
 */
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/jiffies.h>
#include <linux/mfd/rk818.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/power/rk_usbbc.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include "rk818_battery.h"

static int dbg_enable = 0;
module_param_named(dbg_level, dbg_enable, int, 0644);

#define DBG(args...) \
	do { \
		if (dbg_enable) { \
			pr_info(args); \
		} \
	} while (0)

#define BAT_INFO(fmt, args...) pr_info("rk818-bat: "fmt, ##args)

/* default param */
#define DEFAULT_BAT_RES			135
#define DEFAULT_SLP_ENTER_CUR		300
#define DEFAULT_SLP_EXIT_CUR		300
#define DEFAULT_SLP_FILTER_CUR		100
#define DEFAULT_PWROFF_VOL_THRESD	3400
#define DEFAULT_MONITOR_SEC		5
#define DEFAULT_ALGR_VOL_THRESD1	3850
#define DEFAULT_ALGR_VOL_THRESD2	3950
#define DEFAULT_CHRG_VOL_SEL		CHRG_VOL4200MV
#define DEFAULT_CHRG_CUR_SEL		CHRG_CUR1400MA
#define DEFAULT_CHRG_CUR_INPUT		INPUT_CUR2000MA
#define DEFAULT_POFFSET			42
#define DEFAULT_MAX_SOC_OFFSET		60
#define DEFAULT_FB_TEMP			TEMP_115C
#define DEFAULT_ENERGY_MODE		0
#define DEFAULT_POFFSET			42
#define DEFAULT_COFFSET			0x832
#define INVALID_COFFSET_MIN		0x780
#define INVALID_COFFSET_MAX		0x980
#define INVALID_VOL_THRESD		2500
#define DEFAULT_ZERO_RESERVE_DSOC	10
#define DEFAULT_TS2_THRESHOLD_VOL	4350
#define DEFAULT_TS2_VALID_VOL		1000
#define DEFAULT_TS2_VOL_MULTI		0
#define DEFAULT_TS2_CHECK_CNT		5

/*MODE_VIRTUAL params*/
#define VIRTUAL_CURRENT			1000
#define VIRTUAL_VOLTAGE			3888
#define VIRTUAL_SOC			66
#define VIRTUAL_STATUS			POWER_SUPPLY_STATUS_CHARGING
#define VIRTUAL_PRESET			1
#define VIRTUAL_AC_ONLINE		1
#define VIRTUAL_USB_ONLINE		0
#define VIRTUAL_TEMPERATURE		188

/* dsoc calib param */
#define FINISH_CHRG_CUR1		1000
#define FINISH_CHRG_CUR2		1500
#define FINISH_MAX_SOC_DELAY		20
#define TERM_CHRG_DSOC			88
#define TERM_CHRG_CURR			600
#define TERM_CHRG_K			650

#define SIMULATE_CHRG_INTV		8
#define SIMULATE_CHRG_CURR		400
#define SIMULATE_CHRG_K			1500

#define FULL_CHRG_K			400

/* zero algorithm */
#define PWROFF_THRESD			3400
#define MIN_ZERO_DSOC_ACCURACY		10	/*0.01%*/
#define MIN_ZERO_OVERCNT		100
#define MIN_ACCURACY			1
#define DEF_PWRPATH_RES			50
#define	WAIT_DSOC_DROP_SEC		15
#define	WAIT_SHTD_DROP_SEC		30
#define MIN_ZERO_GAP_XSOC1		10
#define MIN_ZERO_GAP_XSOC2		5
#define MIN_ZERO_GAP_XSOC3		3
#define MIN_ZERO_GAP_CALIB		5

#define ADC_CALIB_THRESHOLD		4
#define ADC_CALIB_LMT_MIN		3
#define ADC_CALIB_CNT			5
#define NTC_CALC_FACTOR			7

/* time */
#define	POWER_ON_SEC_BASE		1
#define MINUTE(x)			((x) * 60)

/* sleep */
#define SLP_CURR_MAX			40
#define SLP_CURR_MIN			6
#define DISCHRG_TIME_STEP1		MINUTE(10)
#define DISCHRG_TIME_STEP2		MINUTE(60)
#define SLP_DSOC_VOL_THRESD		3600
#define REBOOT_PERIOD_SEC		180
#define REBOOT_MAX_CNT			80

#define ZERO_LOAD_LVL1			1400
#define ZERO_LOAD_LVL2			600

/* fcc */
#define MIN_FCC				500


static const char *bat_status[] = {
	"charge off", "dead charge", "trickle charge", "cc cv",
	"finish", "usb over vol", "bat temp error", "timer error",
};
struct rk818_battery {
	struct platform_device		*pdev;
	struct rk818			*rk818;
	struct device			*dev;
	struct power_supply		bat;
	struct power_supply		ac;
	struct power_supply		usb;
	struct battery_platform_data	*pdata;
	struct workqueue_struct		*bat_monitor_wq;
	struct workqueue_struct		*charger_wq;
	struct workqueue_struct		*ts2_wq;
	struct delayed_work		bat_delay_work;
	struct delayed_work		dc_delay_work;
	struct delayed_work		bc_delay_work;
	struct delayed_work		calib_delay_work;
	struct delayed_work		term_mode_work;
	struct delayed_work		irq_delay_work;
	struct delayed_work		ts2_vol_work;
	struct wake_lock		wake_lock;
	struct notifier_block		bc_detect_nb;
	struct notifier_block           fb_nb;
	struct timer_list		caltimer;
	enum bc_port_type		charge_otg;
	struct timeval			rtc_base;
	int				bat_res;
	int				chrg_status;
	bool				is_initialized;
	bool				bat_first_power_on;
	u8				ac_in;
	u8				usb_in;
	u8				otg_in;
	u8				dc_in;
	u8				prop_val;
	unsigned long			bc_event;
	int				current_avg;
	int				voltage_avg;
	int				voltage_ocv;
	int				voltage_relax;
	int				voltage_k;/* VCALIB0 VCALIB1 */
	int				voltage_b;
	u32				voltage_ts2;
	int				remain_cap;
	int				design_cap;
	int				nac;
	int				fcc;
	int				qmax;
	int				dsoc;
	int				rsoc;
	int				poffset;
	int				fake_offline;
	int				age_ocv_soc;
	bool				age_allow_update;
	int				age_level;
	int				age_ocv_cap;
	int				age_voltage;
	int				age_adjust_cap;
	unsigned long			age_keep_sec;
	int				zero_timeout_cnt;
	int				zero_remain_cap;
	int				zero_dsoc;
	int				zero_linek;
	u64				zero_drop_sec;
	u64				shtd_drop_sec;
	int				sm_remain_cap;
	int				sm_linek;
	int				sm_chrg_dsoc;
	int				sm_dischrg_dsoc;
	int				algo_rest_val;
	int				algo_rest_mode;
	int				sleep_sum_cap;
	int				sleep_remain_cap;
	unsigned long			sleep_dischrg_sec;
	unsigned long			sleep_sum_sec;
	bool				sleep_chrg_online;
	u8				sleep_chrg_status;
	bool				adc_allow_update;
	int                             fb_blank;
	bool				s2r; /*suspend to resume*/
	u32				work_mode;
	int				temperature;
	int				chrg_cur_lp_input;
	int				chrg_vol_sel;
	int				chrg_cur_input;
	int				chrg_cur_sel;
	u32				monitor_ms;
	u32				pwroff_min;
	u32				adc_calib_cnt;
	unsigned long			chrg_finish_base;
	unsigned long			boot_base;
	unsigned long			flat_match_sec;
	unsigned long			plug_in_base;
	unsigned long			plug_out_base;
	u8				int_sts_msk2;
	u8				halt_cnt;
	bool				is_halt;
	bool				is_max_soc_offset;
	bool				is_sw_reset;
	bool				is_ocv_calib;
	bool				is_first_on;
	bool				is_force_calib;
	int				last_dsoc;
	u8				plug_in_irq;
	u8				plug_out_irq;
	u8				vb_low_irq;
	int				ocv_pre_dsoc;
	int				ocv_new_dsoc;
	int				max_pre_dsoc;
	int				max_new_dsoc;
	int				force_pre_dsoc;
	int				force_new_dsoc;
	int				dbg_cap_low0;
	int				dbg_pwr_dsoc;
	int				dbg_pwr_rsoc;
	int				dbg_pwr_vol;
	int				dbg_chrg_min[10];
	int				dbg_meet_soc;
	int				dbg_calc_dsoc;
	int				dbg_calc_rsoc;
	u32				dbg_i2c_rd_err;
	u32				dbg_i2c_wr_err;
};

struct led_ops {
	void (*led_init)(struct rk818_battery *di);
	void (*led_charging)(struct rk818_battery *di);
	void (*led_discharging)(struct rk818_battery *di);
	void (*led_charging_full)(struct rk818_battery *di);
};

static struct led_ops *rk818_led_ops;
#define to_bat_device_info(x) container_of((x), struct rk818_battery, bat)
#define to_ac_device_info(x) container_of((x), struct rk818_battery, ac)
#define to_usb_device_info(x) container_of((x), struct rk818_battery, usb)

#define DIV(x)	((x) ? (x) : 1)

static u64 get_boot_sec(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);

	return ts.tv_sec;
}

static unsigned long base2sec(unsigned long x)
{
	if (x)
		return (get_boot_sec() > x) ? (get_boot_sec() - x) : 0;
	else
		return 0;
}

static unsigned long base2min(unsigned long x)
{
	return base2sec(x) / 60;
}

static u32 interpolate(int value, u32 *table, int size)
{
	u8 i;
	u16 d;

	for (i = 0; i < size; i++) {
		if (value < table[i])
			break;
	}

	if ((i > 0) && (i < size)) {
		d = (value - table[i - 1]) * (MAX_INTERPOLATE / (size - 1));
		d /= table[i] - table[i - 1];
		d = d + (i - 1) * (MAX_INTERPOLATE / (size - 1));
	} else {
		d = i * ((MAX_INTERPOLATE + size / 2) / size);
	}

	if (d > 1000)
		d = 1000;

	return d;
}

/* (a*b)/c */
static int32_t ab_div_c(u32 a, u32 b, u32 c)
{
	bool sign;
	u32 ans = MAX_INT;
	int32_t tmp;

	sign = ((((a ^ b) ^ c) & 0x80000000) != 0);
	if (c != 0) {
		if (sign)
			c = -c;
		tmp = (a * b + (c >> 1)) / c;
		if (tmp < MAX_INT)
			ans = tmp;
	}

	if (sign)
		ans = -ans;

	return ans;
}

static int rk818_bat_read(struct rk818_battery *di, u8 reg)
{
	int i, val = -1;

	for (i = 0; val < 0 && i < 3; i++) {
		val = rk818_reg_read(di->rk818, reg);
		if (val < 0) {
			di->dbg_i2c_rd_err++;
			dev_err(di->dev, "read reg:0x%x failed\n", reg);
		}
	}

	return val;
}

static int rk818_bat_write(struct rk818_battery *di, u8 reg, u8 buf)
{
	int i, ret = -1;

	for (i = 0; ret < 0 && i < 3; i++) {
		ret = rk818_reg_write(di->rk818, reg, buf);
		if (ret < 0) {
			di->dbg_i2c_wr_err++;
			dev_err(di->dev, "i2c write reg: 0x%2x error\n", reg);
		}
	}

	return ret;
}

static int rk818_bat_set_bits(struct rk818_battery *di, u8 reg, u8 mask, u8 buf)
{
	int ret;

	ret = rk818_set_bits(di->rk818, reg, mask, buf);
	if (ret < 0) {
		di->dbg_i2c_wr_err++;
		dev_err(di->dev, "write reg:0x%x failed\n", reg);
	}

	return ret;
}

static int rk818_bat_clr_bits(struct rk818_battery *di, u8 reg, u8 mask)
{
	int ret;

	ret = rk818_clear_bits(di->rk818, reg, mask);
	if (ret < 0) {
		di->dbg_i2c_wr_err++;
		dev_err(di->dev, "clr reg:0x%02x failed\n", reg);
	}

	return ret;
}

static void rk818_bat_dump_regs(struct rk818_battery *di, u8 start, u8 end)
{
	int i;

	if (!dbg_enable)
		return;

	DBG("dump regs from: 0x%x-->0x%x\n", start, end);
	for (i = start; i < end; i++)
		DBG("0x%x: 0x%0x\n", i, rk818_bat_read(di, i));
}

static bool rk818_bat_chrg_online(struct rk818_battery *di)
{
	return (di->usb_in || di->ac_in || di->dc_in) ? true : false;
}

static int rk818_bat_get_coulomb_cap(struct rk818_battery *di)
{
	int cap, i, val[3] = {0, 0, 0};

	for (i = 0; i < 3; i++) {
		val[i] |= rk818_bat_read(di, RK818_GASCNT3_REG) << 24;
		val[i] |= rk818_bat_read(di, RK818_GASCNT2_REG) << 16;
		val[i] |= rk818_bat_read(di, RK818_GASCNT1_REG) << 8;
		val[i] |= rk818_bat_read(di, RK818_GASCNT0_REG) << 0;
	}

	if (val[0] == val[1])
		cap = val[0];
	else
		cap = val[2];

	return (cap / 2390);
}

static int rk818_bat_get_rsoc(struct rk818_battery *di)
{
	int remain_cap;

	remain_cap = rk818_bat_get_coulomb_cap(di);
	return (remain_cap + di->fcc / 200) * 100 / DIV(di->fcc);
}

static ssize_t bat_info_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	char cmd;
	struct rk818_battery *di = to_bat_device_info(dev_get_drvdata(dev));

	sscanf(buf, "%c", &cmd);
	if (cmd == 'n')
		rk818_bat_set_bits(di, RK818_MISC_MARK_REG,
				   FG_RESET_NOW, FG_RESET_NOW);
	else if (cmd == 'm')
		rk818_set_bits(di->rk818, RK818_MISC_MARK_REG,
			       FG_RESET_LATE, FG_RESET_LATE);
	else if (cmd == 'c')
		rk818_clear_bits(di->rk818, RK818_MISC_MARK_REG,
				 FG_RESET_LATE | FG_RESET_NOW);
	else if (cmd == 'r')
		BAT_INFO("0x%2x\n", rk818_bat_read(di, RK818_MISC_MARK_REG));
	else
		BAT_INFO("command error\n");

	return count;
}

static struct device_attribute rk818_bat_attr[] = {
	__ATTR(bat, 0664, NULL, bat_info_store),
};

static void rk818_bat_enable_gauge(struct rk818_battery *di)
{
	u8 buf;

	buf = rk818_bat_read(di, RK818_TS_CTRL_REG);
	buf |= GG_EN;
	rk818_bat_write(di, RK818_TS_CTRL_REG, buf);
}

static void rk818_bat_save_age_level(struct rk818_battery *di, u8 level)
{
	rk818_bat_write(di, RK818_UPDAT_LEVE_REG, level);
}

static u8 rk818_bat_get_age_level(struct  rk818_battery *di)
{
	return rk818_bat_read(di, RK818_UPDAT_LEVE_REG);
}

static int rk818_bat_get_vcalib0(struct rk818_battery *di)
{
	int val = 0;

	val |= rk818_bat_read(di, RK818_VCALIB0_REGL) << 0;
	val |= rk818_bat_read(di, RK818_VCALIB0_REGH) << 8;

	DBG("<%s>. voffset0: 0x%x\n", __func__, val);
	return val;
}

static int rk818_bat_get_vcalib1(struct rk818_battery *di)
{
	int val = 0;

	val |= rk818_bat_read(di, RK818_VCALIB1_REGL) << 0;
	val |= rk818_bat_read(di, RK818_VCALIB1_REGH) << 8;

	DBG("<%s>. voffset1: 0x%x\n", __func__, val);
	return val;
}

static int rk818_bat_get_ioffset(struct rk818_battery *di)
{
	int val = 0;

	val |= rk818_bat_read(di, RK818_IOFFSET_REGL) << 0;
	val |= rk818_bat_read(di, RK818_IOFFSET_REGH) << 8;

	DBG("<%s>. ioffset: 0x%x\n", __func__, val);
	return val;
}

static int rk818_bat_get_coffset(struct rk818_battery *di)
{
	int val = 0;

	val |= rk818_bat_read(di, RK818_CAL_OFFSET_REGL) << 0;
	val |= rk818_bat_read(di, RK818_CAL_OFFSET_REGH) << 8;

	DBG("<%s>. coffset: 0x%x\n", __func__, val);
	return val;
}

static void rk818_bat_set_coffset(struct rk818_battery *di, int val)
{
	u8 buf;

	if ((val < INVALID_COFFSET_MIN) || (val > INVALID_COFFSET_MAX)) {
		BAT_INFO("set invalid coffset=0x%x\n", val);
		return;
	}

	buf = (val >> 8) & 0xff;
	rk818_bat_write(di, RK818_CAL_OFFSET_REGH, buf);
	buf = (val >> 0) & 0xff;
	rk818_bat_write(di, RK818_CAL_OFFSET_REGL, buf);
	DBG("<%s>. coffset: 0x%x\n", __func__, val);
}

static void rk818_bat_init_voltage_kb(struct rk818_battery *di)
{
	int vcalib0, vcalib1;

	vcalib0 = rk818_bat_get_vcalib0(di);
	vcalib1 = rk818_bat_get_vcalib1(di);
	di->voltage_k = (4200 - 3000) * 1000 / DIV(vcalib1 - vcalib0);
	di->voltage_b = 4200 - (di->voltage_k * vcalib1) / 1000;

	DBG("voltage_k=%d(*1000),voltage_b=%d\n", di->voltage_k, di->voltage_b);
}

static int rk818_bat_get_ocv_voltage(struct rk818_battery *di)
{
	int vol, i, temp, val[3] = {0, 0, 0};

	for (i = 0; i < 3; i++) {
		val[i] |= rk818_bat_read(di, RK818_BAT_OCV_REGL) << 0;
		val[i] |= rk818_bat_read(di, RK818_BAT_OCV_REGH) << 8;
	}

	if (val[0] == val[1])
		temp = val[0];
	else
		temp = val[2];

	vol = di->voltage_k * temp / 1000 + di->voltage_b;

	return vol;
}

static int rk818_bat_get_avg_voltage(struct rk818_battery *di)
{
	int vol, i, temp, val[3] = {0, 0, 0};

	for (i = 0; i < 3; i++) {
		val[i] |= rk818_bat_read(di, RK818_BAT_VOL_REGL) << 0;
		val[i] |= rk818_bat_read(di, RK818_BAT_VOL_REGH) << 8;
	}

	if (val[0] == val[1])
		temp = val[0];
	else
		temp = val[2];

	vol = di->voltage_k * temp / 1000 + di->voltage_b;

	return vol;
}

static bool is_rk818_bat_relax_mode(struct rk818_battery *di)
{
	u8 status;

	status = rk818_bat_read(di, RK818_GGSTS_REG);
	if (!(status & RELAX_VOL1_UPD) || !(status & RELAX_VOL2_UPD))
		return false;
	else
		return true;
}

static u16 rk818_bat_get_relax_vol1(struct rk818_battery *di)
{
	u16 vol, val = 0;

	val |= rk818_bat_read(di, RK818_RELAX_VOL1_REGL) << 0;
	val |= rk818_bat_read(di, RK818_RELAX_VOL1_REGH) << 8;
	vol = di->voltage_k * val / 1000 + di->voltage_b;

	return vol;
}

static u16 rk818_bat_get_relax_vol2(struct rk818_battery *di)
{
	u16 vol, val = 0;

	val |= rk818_bat_read(di, RK818_RELAX_VOL2_REGL) << 0;
	val |= rk818_bat_read(di, RK818_RELAX_VOL2_REGH) << 8;
	vol = di->voltage_k * val / 1000 + di->voltage_b;

	return vol;
}

static u16 rk818_bat_get_relax_voltage(struct rk818_battery *di)
{
	u16 relax_vol1, relax_vol2;

	if (!is_rk818_bat_relax_mode(di))
		return 0;

	relax_vol1 = rk818_bat_get_relax_vol1(di);
	relax_vol2 = rk818_bat_get_relax_vol2(di);

	return relax_vol1 > relax_vol2 ? relax_vol1 : relax_vol2;
}

static int rk818_bat_get_avg_current(struct rk818_battery *di)
{
	int i, temp, cur, val[3] = {0, 0, 0};

	for (i = 0; i < 3; i++) {
		val[i] |= rk818_bat_read(di, RK818_BAT_CUR_AVG_REGL) << 0;
		val[i] |= rk818_bat_read(di, RK818_BAT_CUR_AVG_REGH) << 8;
	}

	if (val[0] == val[1])
		temp = val[0];
	else
		temp = val[2];
	if (temp & 0x800)
		temp -= 4096;
	cur = temp * 1506 / 1000;

	return cur;
}

static int rk818_bat_vol_to_ocvsoc(struct rk818_battery *di, int voltage)
{
	u32 *ocv_table, temp;
	int ocv_size, ocv_soc;

	ocv_table = di->pdata->ocv_table;
	ocv_size = di->pdata->ocv_size;
	temp = interpolate(voltage, ocv_table, ocv_size);
	ocv_soc = ab_div_c(temp, MAX_PERCENTAGE, MAX_INTERPOLATE);

	return ocv_soc;
}

static int rk818_bat_vol_to_ocvcap(struct rk818_battery *di, int voltage)
{
	u32 *ocv_table, temp;
	int ocv_size, cap;

	ocv_table = di->pdata->ocv_table;
	ocv_size = di->pdata->ocv_size;
	temp = interpolate(voltage, ocv_table, ocv_size);
	cap = ab_div_c(temp, di->fcc, MAX_INTERPOLATE);

	return cap;
}

static int rk818_bat_vol_to_zerosoc(struct rk818_battery *di, int voltage)
{
	u32 *ocv_table, temp;
	int ocv_size, ocv_soc;

	ocv_table = di->pdata->zero_table;
	ocv_size = di->pdata->ocv_size;
	temp = interpolate(voltage, ocv_table, ocv_size);
	ocv_soc = ab_div_c(temp, MAX_PERCENTAGE, MAX_INTERPOLATE);

	return ocv_soc;
}

static int rk818_bat_vol_to_zerocap(struct rk818_battery *di, int voltage)
{
	u32 *ocv_table, temp;
	int ocv_size, cap;

	ocv_table = di->pdata->zero_table;
	ocv_size = di->pdata->ocv_size;
	temp = interpolate(voltage, ocv_table, ocv_size);
	cap = ab_div_c(temp, di->fcc, MAX_INTERPOLATE);

	return cap;
}

static int rk818_bat_get_iadc(struct rk818_battery *di)
{
	int val = 0;

	val |= rk818_bat_read(di, RK818_BAT_CUR_AVG_REGL) << 0;
	val |= rk818_bat_read(di, RK818_BAT_CUR_AVG_REGH) << 8;
	if (val > 2047)
		val -= 4096;

	return val;
}

static bool rk818_bat_adc_calib(struct rk818_battery *di)
{
	int i, ioffset, coffset, adc, save_coffset;

	if ((di->chrg_status != CHARGE_FINISH) ||
	    (di->adc_calib_cnt > ADC_CALIB_CNT) ||
	    (base2min(di->boot_base) < ADC_CALIB_LMT_MIN) ||
	    (abs(di->current_avg) < ADC_CALIB_THRESHOLD))
		return false;

	di->adc_calib_cnt++;
	save_coffset = rk818_bat_get_coffset(di);
	for (i = 0; i < 5; i++) {
		adc = rk818_bat_get_iadc(di);
		if (!rk818_bat_chrg_online(di)) {
			rk818_bat_set_coffset(di, save_coffset);
			BAT_INFO("quit, charger plugout when calib adc\n");
			return false;
		}
		coffset = rk818_bat_get_coffset(di);
		rk818_bat_set_coffset(di, coffset + adc);
		msleep(2000);
		adc = rk818_bat_get_iadc(di);
		if (abs(adc) < ADC_CALIB_THRESHOLD) {
			coffset = rk818_bat_get_coffset(di);
			ioffset = rk818_bat_get_ioffset(di);
			di->poffset = coffset - ioffset;
			rk818_bat_write(di, RK818_POFFSET_REG, di->poffset);
			BAT_INFO("new offset:c=0x%x, i=0x%x, p=0x%x\n",
				 coffset, ioffset, di->poffset);
			return true;
		} else {
			BAT_INFO("coffset calib again %d.., max_cnt=%d\n",
				 i, di->adc_calib_cnt);
			rk818_bat_set_coffset(di, coffset);
			msleep(2000);
		}
	}

	rk818_bat_set_coffset(di, save_coffset);

	return false;
}

static void rk818_bat_set_ioffset_sample(struct rk818_battery *di)
{
	u8 ggcon;

	ggcon = rk818_bat_read(di, RK818_GGCON_REG);
	ggcon &= ~ADC_CAL_MIN_MSK;
	ggcon |= ADC_CAL_8MIN;
	rk818_bat_write(di, RK818_GGCON_REG, ggcon);
}

static void rk818_bat_set_ocv_sample(struct rk818_battery *di)
{
	u8 ggcon;

	ggcon = rk818_bat_read(di, RK818_GGCON_REG);
	ggcon &= ~OCV_SAMP_MIN_MSK;
	ggcon |= OCV_SAMP_8MIN;
	rk818_bat_write(di, RK818_GGCON_REG, ggcon);
}

static void rk818_bat_restart_relax(struct rk818_battery *di)
{
	u8 ggsts;

	ggsts = rk818_bat_read(di, RK818_GGSTS_REG);
	ggsts &= ~RELAX_VOL12_UPD_MSK;
	rk818_bat_write(di, RK818_GGSTS_REG, ggsts);
}

static void rk818_bat_set_relax_sample(struct rk818_battery *di)
{
	u8 buf;
	int enter_thres, exit_thres;
	struct battery_platform_data *pdata = di->pdata;

	enter_thres = pdata->sleep_enter_current * 1000 / 1506;
	exit_thres = pdata->sleep_exit_current * 1000 / 1506;

	/* set relax enter and exit threshold */
	buf = enter_thres & 0xff;
	rk818_bat_write(di, RK818_RELAX_ENTRY_THRES_REGL, buf);
	buf = (enter_thres >> 8) & 0xff;
	rk818_bat_write(di, RK818_RELAX_ENTRY_THRES_REGH, buf);

	buf = exit_thres & 0xff;
	rk818_bat_write(di, RK818_RELAX_EXIT_THRES_REGL, buf);
	buf = (exit_thres >> 8) & 0xff;
	rk818_bat_write(di, RK818_RELAX_EXIT_THRES_REGH, buf);

	/* reset relax update state */
	rk818_bat_restart_relax(di);
	DBG("<%s>. sleep_enter_current = %d, sleep_exit_current = %d\n",
	    __func__, pdata->sleep_enter_current, pdata->sleep_exit_current);
}

/* high load: current < 0 with charger in.
 * System will not shutdown while dsoc=0% with charging state(ac_in),
 * which will cause over discharge, so oppose status before report states.
 */
static void rk818_bat_lowpwr_check(struct rk818_battery *di)
{
	static u64 time;
	int pwr_off_thresd = di->pdata->pwroff_vol;

	if (di->current_avg < 0 && di->voltage_avg < pwr_off_thresd) {
		if (!time)
			time = get_boot_sec();

		if ((base2sec(time) > MINUTE(1)) ||
		    (di->voltage_avg <= pwr_off_thresd - 50)) {
			di->fake_offline = 1;
			if (di->voltage_avg <= pwr_off_thresd - 50)
				di->dsoc--;
			BAT_INFO("low power, soc=%d, current=%d\n",
				 di->dsoc, di->current_avg);
		}
	} else {
		time = 0;
		di->fake_offline = 0;
	}

	DBG("<%s>. t=%lu, dsoc=%d, current=%d, fake_offline=%d\n",
	    __func__, base2sec(time), di->dsoc,
	    di->current_avg, di->fake_offline);
}

static bool is_rk818_bat_exist(struct rk818_battery *di)
{
	return (rk818_bat_read(di, RK818_SUP_STS_REG) & BAT_EXS) ? true : false;
}

static bool is_rk818_bat_first_pwron(struct rk818_battery *di)
{
	u8 buf;

	buf = rk818_bat_read(di, RK818_GGSTS_REG);
	if (buf & BAT_CON) {
		buf &= ~BAT_CON;
		rk818_bat_write(di, RK818_GGSTS_REG, buf);
		return true;
	}

	return false;
}

static u8 rk818_bat_get_pwroff_min(struct rk818_battery *di)
{
	u8 cur, last;

	cur = rk818_bat_read(di, RK818_NON_ACT_TIMER_CNT_REG);
	last = rk818_bat_read(di, RK818_NON_ACT_TIMER_CNT_SAVE_REG);
	rk818_bat_write(di, RK818_NON_ACT_TIMER_CNT_SAVE_REG, cur);

	return (cur != last) ? cur : 0;
}

static u8 is_rk818_bat_initialized(struct rk818_battery *di)
{
	u8 val = rk818_bat_read(di, RK818_MISC_MARK_REG);

	if (val & FG_INIT) {
		val &= ~FG_INIT;
		rk818_bat_write(di, RK818_MISC_MARK_REG, val);
		return true;
	} else {
		return false;
	}
}

static bool is_rk818_bat_ocv_valid(struct rk818_battery *di)
{
	return (!di->is_initialized && di->pwroff_min >= 30) ? true : false;
}

static void rk818_bat_init_age_algorithm(struct rk818_battery *di)
{
	int age_level, ocv_soc, ocv_cap, ocv_vol;

	if (di->bat_first_power_on || is_rk818_bat_ocv_valid(di)) {
		DBG("<%s> enter.\n", __func__);
		ocv_vol = rk818_bat_get_ocv_voltage(di);
		ocv_soc = rk818_bat_vol_to_ocvsoc(di, ocv_vol);
		ocv_cap = rk818_bat_vol_to_ocvcap(di, ocv_vol);
		if (ocv_soc < 20) {
			di->age_voltage = ocv_vol;
			di->age_ocv_cap = ocv_cap;
			di->age_ocv_soc = ocv_soc;
			di->age_adjust_cap = 0;

			if (ocv_soc <= 0)
				di->age_level = 100;
			else if (ocv_soc < 5)
				di->age_level = 95;
			else if (ocv_soc < 10)
				di->age_level = 90;
			else
				di->age_level = 80;

			age_level = rk818_bat_get_age_level(di);
			if (age_level > di->age_level) {
				di->age_allow_update = false;
				age_level -= 5;
				if (age_level <= 80)
					age_level = 80;
				rk818_bat_save_age_level(di, age_level);
			} else {
				di->age_allow_update = true;
				di->age_keep_sec = get_boot_sec();
			}

			BAT_INFO("init_age_algorithm: "
				 "age_vol:%d, age_ocv_cap:%d, "
				 "age_ocv_soc:%d, old_age_level:%d, "
				 "age_allow_update:%d, new_age_level:%d\n",
				 di->age_voltage, di->age_ocv_cap,
				 ocv_soc, age_level, di->age_allow_update,
				 di->age_level);
		}
	}
}

static enum power_supply_property rk818_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int rk818_battery_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct rk818_battery *di = to_bat_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_avg * 1000;/*uA*/
		if (di->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_CURRENT * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage_avg * 1000;/*uV*/
		if (di->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_VOLTAGE * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = is_rk818_bat_exist(di);
		if (di->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_PRESET;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = di->dsoc;
		if (di->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_SOC;
		DBG("<%s>. report dsoc: %d\n", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->prop_val;
		if (di->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_STATUS;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->temperature;
		if (di->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_TEMPERATURE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property rk818_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property rk818_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int rk818_bat_ac_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	int ret = 0;
	struct rk818_battery *di = to_ac_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (di->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_AC_ONLINE;
		else if (di->fake_offline)
			val->intval = 0;
		else
			val->intval = di->ac_in | di->dc_in;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rk818_bat_usb_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	int ret = 0;
	struct rk818_battery *di = to_usb_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (di->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_USB_ONLINE;
		else if (di->fake_offline)
			val->intval = 0;
		else
			val->intval = di->usb_in;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rk818_bat_init_power_supply(struct rk818_battery *di)
{
	int ret;

	di->bat.name = "battery";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = rk818_bat_props;
	di->bat.num_properties = ARRAY_SIZE(rk818_bat_props);
	di->bat.get_property = rk818_battery_get_property;

	di->ac.name = "ac";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.properties = rk818_ac_props;
	di->ac.num_properties = ARRAY_SIZE(rk818_ac_props);
	di->ac.get_property = rk818_bat_ac_get_property;

	di->usb.name = "usb";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = rk818_usb_props;
	di->usb.num_properties = ARRAY_SIZE(rk818_usb_props);
	di->usb.get_property = rk818_bat_usb_get_property;

	ret = power_supply_register(di->dev, &di->bat);
	if (ret) {
		dev_err(di->dev, "register bat power supply fail\n");
		goto bat_failed;
	}

	ret = power_supply_register(di->dev, &di->usb);
	if (ret) {
		dev_err(di->dev, "register usb power supply fail\n");
		goto usb_failed;
	}

	ret = power_supply_register(di->dev, &di->ac);
	if (ret) {
		dev_err(di->dev, "register ac power supply fail\n");
		goto ac_failed;
	}

	return ret;

ac_failed:
	power_supply_unregister(&di->ac);
usb_failed:
	power_supply_unregister(&di->usb);
bat_failed:
	power_supply_unregister(&di->bat);

	return ret;
}

static void rk818_bat_save_cap(struct rk818_battery *di, int capacity)
{
	u8 buf;
	static u32 old_cap;

	if (capacity >= di->qmax)
		capacity = di->qmax;
	if (capacity <= 0)
		capacity = 0;
	if (old_cap == capacity)
		return;

	old_cap = capacity;
	buf = (capacity >> 24) & 0xff;
	rk818_bat_write(di, RK818_REMAIN_CAP_REG3, buf);
	buf = (capacity >> 16) & 0xff;
	rk818_bat_write(di, RK818_REMAIN_CAP_REG2, buf);
	buf = (capacity >> 8) & 0xff;
	rk818_bat_write(di, RK818_REMAIN_CAP_REG1, buf);
	buf = (capacity >> 0) & 0xff;
	rk818_bat_write(di, RK818_REMAIN_CAP_REG0, buf);
}

static int rk818_bat_get_prev_cap(struct rk818_battery *di)
{
	int i, val[3] = {0, 0, 0};

	for (i = 0; i < 3; i++) {
		val[i] |= rk818_bat_read(di, RK818_REMAIN_CAP_REG3) << 24;
		val[i] |= rk818_bat_read(di, RK818_REMAIN_CAP_REG2) << 16;
		val[i] |= rk818_bat_read(di, RK818_REMAIN_CAP_REG1) << 8;
		val[i] |= rk818_bat_read(di, RK818_REMAIN_CAP_REG0) << 0;
	}

	return  (val[0] == val[1]) ? val[0] : val[2];
}

static void rk818_bat_save_fcc(struct rk818_battery *di, u32 fcc)
{
	u8 buf;

	buf = (fcc >> 24) & 0xff;
	rk818_bat_write(di, RK818_NEW_FCC_REG3, buf);
	buf = (fcc >> 16) & 0xff;
	rk818_bat_write(di, RK818_NEW_FCC_REG2, buf);
	buf = (fcc >> 8) & 0xff;
	rk818_bat_write(di, RK818_NEW_FCC_REG1, buf);
	buf = (fcc >> 0) & 0xff;
	rk818_bat_write(di, RK818_NEW_FCC_REG0, buf);

	BAT_INFO("save fcc: %d\n", fcc);
}

static int rk818_bat_get_fcc(struct rk818_battery *di)
{
	u32 fcc = 0;

	fcc |= rk818_bat_read(di, RK818_NEW_FCC_REG3) << 24;
	fcc |= rk818_bat_read(di, RK818_NEW_FCC_REG2) << 16;
	fcc |= rk818_bat_read(di, RK818_NEW_FCC_REG1) << 8;
	fcc |= rk818_bat_read(di, RK818_NEW_FCC_REG0) << 0;

	if (fcc < MIN_FCC) {
		BAT_INFO("invalid fcc(%d), use design cap", fcc);
		fcc = di->pdata->design_capacity;
		rk818_bat_save_fcc(di, fcc);
	} else if (fcc > di->pdata->design_qmax) {
		BAT_INFO("invalid fcc(%d), use qmax", fcc);
		fcc = di->pdata->design_qmax;
		rk818_bat_save_fcc(di, fcc);
	}

	return fcc;
}

static void rk818_bat_save_dsoc(struct rk818_battery *di, u8 save_soc)
{
	static int last_soc = -1;

	if (last_soc != save_soc) {
		rk818_bat_write(di, RK818_SOC_REG, save_soc);
		last_soc = save_soc;
	}
}

static int rk818_bat_get_prev_dsoc(struct rk818_battery *di)
{
	return rk818_bat_read(di, RK818_SOC_REG);
}

static void rk818_bat_save_reboot_cnt(struct rk818_battery *di, u8 save_cnt)
{
	rk818_bat_write(di, RK818_REBOOT_CNT_REG, save_cnt);
}

static void rk818_bat_init_leds(struct rk818_battery *di)
{
	if (rk818_led_ops && rk818_led_ops->led_init) {
		rk818_led_ops->led_init(di);
		BAT_INFO("leds initialized\n");
	}
}

static void rk818_bat_update_leds(struct rk818_battery *di, int prop)
{
	static int old_prop = -1;

	if (prop == old_prop)
		return;

	old_prop = prop;
	switch(prop) {
	case POWER_SUPPLY_STATUS_FULL:
		if (rk818_led_ops && rk818_led_ops->led_charging_full) {
			rk818_led_ops->led_charging_full(di);
			BAT_INFO("charging full led on\n");
		}
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
		if (rk818_led_ops && rk818_led_ops->led_charging) {
			rk818_led_ops->led_charging(di);
			BAT_INFO("charging led on\n");
		}
		break;
	case POWER_SUPPLY_STATUS_DISCHARGING:
		if (rk818_led_ops && rk818_led_ops->led_discharging) {
			rk818_led_ops->led_discharging(di);
			BAT_INFO("discharging led on\n");
		}
		break;
	default:
		BAT_INFO("Unknown led update\n");
		break;
	}
}
static void rk818_bat_set_current(struct rk818_battery *di, int charge_current)
{
	u8 usb_ctrl;

	if (di->pdata->bat_mode == MODE_VIRTUAL) {
		BAT_INFO("virtual power test mode, set max input current\n");
		charge_current = di->chrg_cur_input;
	}

	usb_ctrl = rk818_bat_read(di, RK818_USB_CTRL_REG);
	usb_ctrl &= ~INPUT_CUR_MSK;
	usb_ctrl |= (charge_current);
	rk818_bat_write(di, RK818_USB_CTRL_REG, usb_ctrl);
}

static int rk818_bat_get_ts2_voltage(struct rk818_battery *di)
{
	u32 val = 0;
	int voltage;

	val |= rk818_bat_read(di, RK818_TS2_ADC_REGL) << 0;
	val |= rk818_bat_read(di, RK818_TS2_ADC_REGH) << 8;

	/* refer voltage 2.2V, 12bit adc accuracy */
	voltage = val * 2200 * di->pdata->ts2_vol_multi / 4095;

	DBG("********* ts2 adc=%d, vol=%d\n", val, voltage);

	return voltage;
}

static void rk818_chrg_term_mode_set(struct rk818_battery *di, int mode)
{
	u8 buf, mask = CHRG_TERM_MODE_MSK;

	buf = rk818_bat_read(di, RK818_CHRG_CTRL_REG3);
	buf &= ~mask;
	buf |= mode;
	rk818_bat_write(di, RK818_CHRG_CTRL_REG3, buf);
}

static void rk818_term_mode_work(struct work_struct *work)
{
	struct rk818_battery *di;

	di = container_of(work, struct rk818_battery,
			  term_mode_work.work);

	if (rk818_bat_chrg_online(di))
		rk818_chrg_term_mode_set(di, CHRG_TERM_DIG_SIGNAL);
	else
		rk818_chrg_term_mode_set(di, CHRG_TERM_ANA_SIGNAL);
}

static void rk818_ts2_vol_work(struct work_struct *work)
{
	struct rk818_battery *di;
	int ts2_vol, input_current, invalid_cnt = 0, confirm_cnt = 0;

	di = container_of(work, struct rk818_battery, ts2_vol_work.work);

	input_current = INPUT_CUR80MA;
	while (input_current < di->chrg_cur_input) {
		msleep(100);
		ts2_vol = rk818_bat_get_ts2_voltage(di);

		/* filter invalid voltage */
		if (ts2_vol <= DEFAULT_TS2_VALID_VOL) {
			invalid_cnt++;
			DBG("%s: invalid ts2 voltage: %d\n, cnt=%d",
			    __func__, ts2_vol, invalid_cnt);
			if (invalid_cnt < DEFAULT_TS2_CHECK_CNT)
				continue;

			/* if fail, set max input current as default */
			input_current = di->chrg_cur_input;
			rk818_bat_set_current(di, input_current);
			break;
		}

		/* update input current */
		if (ts2_vol >= DEFAULT_TS2_THRESHOLD_VOL) {
			/* update input current */
			input_current++;
			rk818_bat_set_current(di, input_current);
			DBG("********* input=%d\n",
			    CHRG_CUR_INPUT[input_current & 0x0f]);
		} else {
			/* confirm lower threshold voltage */
			confirm_cnt++;
			if (confirm_cnt < DEFAULT_TS2_CHECK_CNT) {
				DBG("%s: confirm ts2 voltage: %d\n, cnt=%d",
				    __func__, ts2_vol, confirm_cnt);
				continue;
			}

			/* trigger threshold, so roll back 1 step */
			input_current--;
			if (input_current == INPUT_CUR80MA ||
			    input_current < 0)
				input_current = INPUT_CUR450MA;
			rk818_bat_set_current(di, input_current);
			break;
		}
	}

	if (input_current != di->chrg_cur_input)
		BAT_INFO("adjust input current: %dma\n",
			 CHRG_CUR_INPUT[input_current & 0x0f]);
}

static void rk818_bat_set_chrg_param(struct rk818_battery *di,
				     enum charger_type charger_type)
{
	u8 buf, usb_ctrl, chrg_ctrl1;
	static const char *const charge_name[] = {"UNKNOWN", "NONE_USB_DC",
						  "NONE_USB", "USB", "AC",
						  "DC_DC", "NONE_DC"};

	switch (charger_type) {
	case USB_DC_TYPE_NONE_CHARGER:
		di->usb_in = OFFLINE;
		di->ac_in = OFFLINE;
		di->dc_in = OFFLINE;
		di->prop_val = POWER_SUPPLY_STATUS_DISCHARGING;
		rk818_bat_set_current(di, INPUT_CUR450MA);
		power_supply_changed(&di->bat);
		power_supply_changed(&di->usb);
		power_supply_changed(&di->ac);
		break;
	case USB_TYPE_NONE_CHARGER:
		di->usb_in = OFFLINE;
		di->ac_in = OFFLINE;
		if (di->dc_in == OFFLINE) {
			di->prop_val = POWER_SUPPLY_STATUS_DISCHARGING;
			rk818_bat_set_current(di, INPUT_CUR450MA);
		}
		power_supply_changed(&di->usb);
		power_supply_changed(&di->ac);
		break;
	case USB_TYPE_USB_CHARGER:
		di->usb_in = ONLINE;
		di->ac_in = OFFLINE;
		di->prop_val = POWER_SUPPLY_STATUS_CHARGING;
		if (di->dc_in == OFFLINE)
			rk818_bat_set_current(di, INPUT_CUR450MA);
		power_supply_changed(&di->usb);
		break;
	case USB_TYPE_AC_CHARGER:
		di->ac_in = ONLINE;
		di->usb_in = OFFLINE;
		di->prop_val = POWER_SUPPLY_STATUS_CHARGING;
		if (di->pdata->ts2_vol_multi) {
			rk818_bat_set_current(di, INPUT_CUR450MA);
			queue_delayed_work(di->ts2_wq,
					   &di->ts2_vol_work,
					   msecs_to_jiffies(0));
		} else {
			if (di->pdata->lp_input_current &&
			    di->dsoc >= di->pdata->lp_soc_min &&
			    di->dsoc <= di->pdata->lp_soc_max)
				rk818_bat_set_current(di,
						      di->chrg_cur_lp_input);
			else
				rk818_bat_set_current(di, di->chrg_cur_input);
		}
		power_supply_changed(&di->ac);
		break;
	case DC_TYPE_DC_CHARGER:
		di->dc_in = ONLINE;
		di->prop_val = POWER_SUPPLY_STATUS_CHARGING;
		if (di->pdata->ts2_vol_multi) {
			rk818_bat_set_current(di, INPUT_CUR450MA);
			queue_delayed_work(di->ts2_wq,
					   &di->ts2_vol_work,
					   msecs_to_jiffies(0));
		} else {
			if (di->pdata->lp_input_current &&
			    di->dsoc >= di->pdata->lp_soc_min &&
			    di->dsoc <= di->pdata->lp_soc_max)
				rk818_bat_set_current(di,
						      di->chrg_cur_lp_input);
			else
				rk818_bat_set_current(di, di->chrg_cur_input);
		}
		power_supply_changed(&di->ac);
		break;
	case DC_TYPE_NONE_CHARGER:
		di->dc_in = OFFLINE;
		/*
		 * check by pmic int avoid usb error notify:
		 * when plug in dc, usb may error notify usb/ac plug in,
		 * while dc plug out, the "ac/usb_in" still hold
		 */
		buf = rk818_bat_read(di, RK818_VB_MON_REG);
		if ((buf & PLUG_IN_STS) == 0) {
			di->ac_in = OFFLINE;
			di->usb_in = OFFLINE;
			di->prop_val = POWER_SUPPLY_STATUS_DISCHARGING;
			rk818_bat_set_current(di, INPUT_CUR450MA);
		} else if (di->usb_in) {
			rk818_bat_set_current(di, INPUT_CUR450MA);
			di->prop_val = POWER_SUPPLY_STATUS_CHARGING;
		}
		power_supply_changed(&di->usb);
		power_supply_changed(&di->ac);
		break;
	default:
		di->prop_val = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	}

	usb_ctrl = rk818_bat_read(di, RK818_USB_CTRL_REG);
	chrg_ctrl1 = rk818_bat_read(di, RK818_CHRG_CTRL_REG1);
	BAT_INFO("set charger type: %s, current: input=%d, chrg=%d\n",
		 charge_name[charger_type],
		 CHRG_CUR_INPUT[usb_ctrl & 0x0f],
		 CHRG_CUR_SEL[chrg_ctrl1 & 0x0f]);

	if (di->dsoc == 100 && rk818_bat_chrg_online(di))
		di->prop_val = POWER_SUPPLY_STATUS_FULL;

	rk818_bat_update_leds(di, di->prop_val);

	queue_delayed_work(di->charger_wq,
			   &di->term_mode_work,
			   msecs_to_jiffies(1000));
}

static void rk818_bat_set_otg_state(struct rk818_battery *di, int state)
{
	switch (state) {
	case USB_OTG_POWER_ON:
		rk818_bat_set_bits(di, RK818_INT_STS_MSK_REG2,
				   PLUG_IN_MSK, PLUG_IN_MSK);
		rk818_bat_set_bits(di, RK818_INT_STS_MSK_REG2,
				   PLUG_OUT_MSK, PLUG_OUT_MSK);
		rk818_bat_set_bits(di, RK818_DCDC_EN_REG,
				   OTG_EN_MASK, OTG_EN_MASK);
		rk818_bat_clr_bits(di, RK818_SLEEP_SET_OFF_REG1,
				   OTG_EN_MASK);
		break;
	case USB_OTG_POWER_OFF:
		rk818_bat_clr_bits(di, RK818_INT_STS_MSK_REG2, PLUG_IN_MSK);
		rk818_bat_clr_bits(di, RK818_INT_STS_MSK_REG2, PLUG_OUT_MSK);
		rk818_bat_clr_bits(di, RK818_DCDC_EN_REG, OTG_EN_MASK);
		rk818_bat_set_bits(di, RK818_SLEEP_SET_OFF_REG1,
				   OTG_EN_MASK, OTG_EN_MASK);
		break;
	default:
		break;
	}
}

static enum charger_type rk818_bat_get_dc_state(struct rk818_battery *di)
{
	int level;

	if (!gpio_is_valid(di->pdata->dc_det_pin))
		return DC_TYPE_NONE_CHARGER;

	level = gpio_get_value(di->pdata->dc_det_pin);

	return (level == di->pdata->dc_det_level) ? DC_TYPE_DC_CHARGER : DC_TYPE_NONE_CHARGER;
}

static void rk818_bat_dc_delay_work(struct work_struct *work)
{
	enum charger_type type;
	static enum charger_type old_type = UNKNOWN_CHARGER;
	struct rk818_battery *di = container_of(work,
				struct rk818_battery, dc_delay_work.work);

	type = rk818_bat_get_dc_state(di);
	if (old_type == type)
		return;

	old_type = type;
	if (type == DC_TYPE_DC_CHARGER) {
		BAT_INFO("detect dc charger in..\n");
		rk818_bat_set_chrg_param(di, DC_TYPE_DC_CHARGER);
		/* check otg supply */
		if (di->otg_in && di->pdata->power_dc2otg) {
			BAT_INFO("otg power from dc adapter\n");
			rk818_bat_set_otg_state(di, USB_OTG_POWER_OFF);
		}
	} else {
		BAT_INFO("detect dc charger out..\n");
		rk818_bat_set_chrg_param(di, DC_TYPE_NONE_CHARGER);
		/* check otg supply, power on anyway */
		if (di->otg_in) {
			BAT_INFO("charge disable, enable otg\n");
			rk818_bat_set_otg_state(di, USB_OTG_POWER_ON);
		}
	}
}

static enum charger_type rk818_bat_get_acusb_state(struct rk818_battery *di)
{
	u8 buf;
	int usb_id;
	enum charger_type charger;

	usb_id = dwc_otg_check_dpdm(0);
	switch (usb_id) {
	case 0:
		buf = rk818_bat_read(di, RK818_VB_MON_REG);
		if ((buf & PLUG_IN_STS) != 0)
			charger = USB_TYPE_AC_CHARGER;
		else
			charger = USB_TYPE_NONE_CHARGER;
		break;
	case 1:
	case 3:
		charger = USB_TYPE_USB_CHARGER;
		break;
	case 2:
		charger = USB_TYPE_AC_CHARGER;
		break;
	default:
		charger = USB_TYPE_NONE_CHARGER;
		break;
	}

	return charger;
}

static int rk818_bat_fb_notifier(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	struct rk818_battery *di;
	struct fb_event *evdata = data;

	di = container_of(nb, struct rk818_battery, fb_nb);
	di->fb_blank = *(int *)evdata->data;

	return 0;
}

static int rk818_bat_register_fb_notify(struct rk818_battery *di)
{
	memset(&di->fb_nb, 0, sizeof(di->fb_nb));
	di->fb_nb.notifier_call = rk818_bat_fb_notifier;

	return fb_register_client(&di->fb_nb);
}

static int rk818_bat_unregister_fb_notify(struct rk818_battery *di)
{
	return fb_unregister_client(&di->fb_nb);
}

static void rk818_bat_init_coulomb_cap(struct rk818_battery *di, u32 capacity)
{
	u8 buf;
	u32 cap;

	cap = capacity * 2390;
	buf = (cap >> 24) & 0xff;
	rk818_bat_write(di, RK818_GASCNT_CAL_REG3, buf);
	buf = (cap >> 16) & 0xff;
	rk818_bat_write(di, RK818_GASCNT_CAL_REG2, buf);
	buf = (cap >> 8) & 0xff;
	rk818_bat_write(di, RK818_GASCNT_CAL_REG1, buf);
	buf = (cap >> 0) & 0xff;
	rk818_bat_write(di, RK818_GASCNT_CAL_REG0, buf);

	di->remain_cap = capacity;
	di->rsoc = rk818_bat_get_rsoc(di);
}

static u8 rk818_bat_get_halt_cnt(struct rk818_battery *di)
{
	return rk818_bat_read(di, RK818_HALT_CNT_REG);
}

static void rk818_bat_inc_halt_cnt(struct rk818_battery *di)
{
	u8 cnt;

	cnt = rk818_bat_read(di, RK818_HALT_CNT_REG);
	rk818_bat_write(di, RK818_HALT_CNT_REG, ++cnt);
}

static bool is_rk818_bat_last_halt(struct rk818_battery *di)
{
	int pre_cap = rk818_bat_get_prev_cap(di);
	int now_cap = rk818_bat_get_coulomb_cap(di);

	/* over 10%: system halt last time */
	if (abs(now_cap - pre_cap) > (di->fcc / 10)) {
		rk818_bat_inc_halt_cnt(di);
		return true;
	} else {
		return false;
	}
}

static void rk818_bat_first_pwron(struct rk818_battery *di)
{
	int ocv_vol;

	rk818_bat_save_fcc(di, di->design_cap);
	ocv_vol = rk818_bat_get_ocv_voltage(di);
	di->fcc = rk818_bat_get_fcc(di);
	di->nac = rk818_bat_vol_to_ocvcap(di, ocv_vol);
	di->rsoc = rk818_bat_vol_to_ocvsoc(di, ocv_vol);
	di->dsoc = di->rsoc;
	di->is_first_on = true;

	BAT_INFO("first on: dsoc=%d, rsoc=%d cap=%d, fcc=%d, ov=%d\n",
		 di->dsoc, di->rsoc, di->nac, di->fcc, ocv_vol);
}

static void rk818_bat_not_first_pwron(struct rk818_battery *di)
{
	int now_cap, pre_soc, pre_cap, ocv_cap, ocv_soc, ocv_vol;

	di->fcc = rk818_bat_get_fcc(di);
	pre_soc = rk818_bat_get_prev_dsoc(di);
	pre_cap = rk818_bat_get_prev_cap(di);
	now_cap = rk818_bat_get_coulomb_cap(di);
	di->is_halt = is_rk818_bat_last_halt(di);
	di->halt_cnt = rk818_bat_get_halt_cnt(di);
	di->is_initialized = is_rk818_bat_initialized(di);
	di->is_ocv_calib = is_rk818_bat_ocv_valid(di);

	if (di->is_halt) {
		BAT_INFO("system halt last time... cap: pre=%d, now=%d\n",
			 pre_cap, now_cap);
		if (now_cap < 0)
			now_cap = 0;
		rk818_bat_init_coulomb_cap(di, now_cap);
		pre_cap = now_cap;
		pre_soc = di->rsoc;
		goto finish;
	} else if (di->is_initialized) {
		BAT_INFO("initialized yet..\n");
		goto finish;
	} else if (di->is_ocv_calib) {
		ocv_vol = rk818_bat_get_ocv_voltage(di);
		ocv_soc = rk818_bat_vol_to_ocvsoc(di, ocv_vol);
		ocv_cap = rk818_bat_vol_to_ocvcap(di, ocv_vol);
		pre_cap = ocv_cap;
		di->ocv_pre_dsoc = pre_soc;
		di->ocv_new_dsoc = ocv_soc;
		if (abs(ocv_soc - pre_soc) >= di->pdata->max_soc_offset) {
			di->ocv_pre_dsoc = pre_soc;
			di->ocv_new_dsoc = ocv_soc;
			di->is_max_soc_offset = true;
			BAT_INFO("trigger max soc offset, dsoc: %d -> %d\n",
				 pre_soc, ocv_soc);
			pre_soc = ocv_soc;
		}
		BAT_INFO("OCV calib: cap=%d, rsoc=%d\n", ocv_cap, ocv_soc);
	} else if (di->pwroff_min > 0) {
		ocv_vol = rk818_bat_get_ocv_voltage(di);
		ocv_soc = rk818_bat_vol_to_ocvsoc(di, ocv_vol);
		ocv_cap = rk818_bat_vol_to_ocvcap(di, ocv_vol);
		di->force_pre_dsoc = pre_soc;
		di->force_new_dsoc = ocv_soc;
		if (abs(ocv_soc - pre_soc) >= 80) {
			di->is_force_calib = true;
			BAT_INFO("dsoc force calib: %d -> %d\n",
				 pre_soc, ocv_soc);
			pre_soc = ocv_soc;
			pre_cap = ocv_cap;
		}
	}

finish:
	di->dsoc = pre_soc;
	di->nac = pre_cap;
	if (di->nac < 0)
		di->nac = 0;

	BAT_INFO("dsoc=%d cap=%d v=%d ov=%d rv=%d min=%d psoc=%d pcap=%d\n",
		 di->dsoc, di->nac, rk818_bat_get_avg_voltage(di),
		 rk818_bat_get_ocv_voltage(di), rk818_bat_get_relax_voltage(di),
		 di->pwroff_min, rk818_bat_get_prev_dsoc(di),
		 rk818_bat_get_prev_cap(di));
}

static bool rk818_bat_ocv_sw_reset(struct rk818_battery *di)
{
	u8 buf;

	buf = rk818_bat_read(di, RK818_MISC_MARK_REG);
	if (((buf & FG_RESET_LATE) && di->pwroff_min >= 30) ||
	    (buf & FG_RESET_NOW)) {
		buf &= ~FG_RESET_LATE;
		buf &= ~FG_RESET_NOW;
		rk818_bat_write(di, RK818_MISC_MARK_REG, buf);
		BAT_INFO("manual reset fuel gauge\n");
		return true;
	} else {
		return false;
	}
}

static void rk818_bat_init_rsoc(struct rk818_battery *di)
{
	di->bat_first_power_on = is_rk818_bat_first_pwron(di);
	di->is_sw_reset = rk818_bat_ocv_sw_reset(di);
	di->pwroff_min = rk818_bat_get_pwroff_min(di);

	if (di->bat_first_power_on || di->is_sw_reset)
		rk818_bat_first_pwron(di);
	else
		rk818_bat_not_first_pwron(di);
}

static u8 rk818_bat_get_chrg_status(struct rk818_battery *di)
{
	u8 status;

	status = rk818_bat_read(di, RK818_SUP_STS_REG) & CHRG_STATUS_MSK;
	switch (status) {
	case CHARGE_OFF:
		DBG("CHARGE-OFF ...\n");
		break;
	case DEAD_CHARGE:
		BAT_INFO("DEAD CHARGE...\n");
		break;
	case TRICKLE_CHARGE:
		BAT_INFO("TRICKLE CHARGE...\n ");
		break;
	case CC_OR_CV:
		DBG("CC or CV...\n");
		break;
	case CHARGE_FINISH:
		DBG("CHARGE FINISH...\n");
		break;
	case USB_OVER_VOL:
		BAT_INFO("USB OVER VOL...\n");
		break;
	case BAT_TMP_ERR:
		BAT_INFO("BAT TMP ERROR...\n");
		break;
	case TIMER_ERR:
		BAT_INFO("TIMER ERROR...\n");
		break;
	case USB_EXIST:
		BAT_INFO("USB EXIST...\n");
		break;
	case USB_EFF:
		BAT_INFO("USB EFF...\n");
		break;
	default:
		return -EINVAL;
	}

	return status;
}

static u8 rk818_bat_fb_temp(struct rk818_battery *di)
{
	u8 reg;
	int index, fb_temp;

	reg = DEFAULT_FB_TEMP;
	fb_temp = di->pdata->fb_temp;
	for (index = 0; index < ARRAY_SIZE(FEED_BACK_TEMP); index++) {
		if (fb_temp < FEED_BACK_TEMP[index])
			break;
		reg = (index << FB_TEMP_SHIFT);
	}

	return reg;
}

static void rk818_bat_select_chrg_cv(struct rk818_battery *di)
{
	int index, chrg_vol_sel, chrg_cur_sel, chrg_cur_input;
	int chrg_cur_lp_input;

	di->chrg_vol_sel = DEFAULT_CHRG_VOL_SEL;
	di->chrg_cur_input = DEFAULT_CHRG_CUR_INPUT;
	di->chrg_cur_sel = DEFAULT_CHRG_CUR_SEL;
	di->chrg_cur_lp_input = 0;

	chrg_vol_sel = di->pdata->max_chrg_voltage;
	chrg_cur_sel = di->pdata->max_chrg_current;
	chrg_cur_input = di->pdata->max_input_current;
	chrg_cur_lp_input = di->pdata->lp_input_current;

	for (index = 0; index < ARRAY_SIZE(CHRG_CUR_INPUT); index++) {
		if (chrg_cur_lp_input < CHRG_CUR_INPUT[index])
			break;
		di->chrg_cur_lp_input = (index << CHRG_CRU_INPUT_SHIFT);
	}

	for (index = 0; index < ARRAY_SIZE(CHRG_VOL_SEL); index++) {
		if (chrg_vol_sel < CHRG_VOL_SEL[index])
			break;
		di->chrg_vol_sel = (index << CHRG_VOL_SEL_SHIFT);
	}

	for (index = 0; index < ARRAY_SIZE(CHRG_CUR_INPUT); index++) {
		if (chrg_cur_input < CHRG_CUR_INPUT[index])
			break;
		di->chrg_cur_input = (index << CHRG_CRU_INPUT_SHIFT);
	}

	for (index = 0; index < ARRAY_SIZE(CHRG_CUR_SEL); index++) {
		if (chrg_cur_sel < CHRG_CUR_SEL[index])
			break;
		di->chrg_cur_sel = (index << CHRG_CRU_SEL_SHIFT);
	}

	DBG("<%s>. vol = 0x%x, input = 0x%x, sel = 0x%x\n",
	    __func__, di->chrg_vol_sel, di->chrg_cur_input, di->chrg_cur_sel);
}

static u8 rk818_bat_finish_ma(int fcc)
{
	u8 ma;

	if (fcc > 5000)
		ma = FINISH_250MA;
	else if (fcc >= 4000)
		ma = FINISH_200MA;
	else if (fcc >= 3000)
		ma = FINISH_150MA;
	else
		ma = FINISH_100MA;

	return ma;
}

static void rk818_bat_init_chrg_config(struct rk818_battery *di)
{
	u8 chrg_ctrl1, usb_ctrl, chrg_ctrl2, chrg_ctrl3;
	u8 sup_sts, thermal, ggcon, finish_ma, fb_temp;

	rk818_bat_select_chrg_cv(di);
	finish_ma = rk818_bat_finish_ma(di->fcc);
	fb_temp = rk818_bat_fb_temp(di);

	ggcon = rk818_bat_read(di, RK818_GGCON_REG);
	sup_sts = rk818_bat_read(di, RK818_SUP_STS_REG);
	thermal = rk818_bat_read(di, RK818_THERMAL_REG);
	usb_ctrl = rk818_bat_read(di, RK818_USB_CTRL_REG);
	chrg_ctrl1 = rk818_bat_read(di, RK818_CHRG_CTRL_REG1);
	chrg_ctrl2 = rk818_bat_read(di, RK818_CHRG_CTRL_REG2);
	chrg_ctrl3 = rk818_bat_read(di, RK818_CHRG_CTRL_REG3);

	/* set charge current and voltage */
	usb_ctrl &= ~INPUT_CUR_MSK;
	usb_ctrl |= di->chrg_cur_input;
	chrg_ctrl1 = (CHRG_EN) | (di->chrg_vol_sel | di->chrg_cur_sel);

	/* set charge finish current */
	chrg_ctrl3 |= CHRG_TERM_DIG_SIGNAL;
	chrg_ctrl2 &= ~FINISH_CUR_MSK;
	chrg_ctrl2 |= finish_ma;

	/* disable cccv mode */
	chrg_ctrl3 &= ~CHRG_TIMER_CCCV_EN;

	/* disable voltage limit and enable input current limit */
	sup_sts &= ~USB_VLIMIT_EN;
	sup_sts |= USB_CLIMIT_EN;

	/* set feed back temperature */
	if (di->pdata->fb_temp)
		usb_ctrl |= CHRG_CT_EN;
	else
		usb_ctrl &= ~CHRG_CT_EN;
	thermal &= ~FB_TEMP_MSK;
	thermal |= fb_temp;

	/* adc current mode */
	ggcon |= ADC_CUR_MODE;

	rk818_bat_write(di, RK818_GGCON_REG, ggcon);
	rk818_bat_write(di, RK818_SUP_STS_REG, sup_sts);
	rk818_bat_write(di, RK818_THERMAL_REG, thermal);
	rk818_bat_write(di, RK818_USB_CTRL_REG, usb_ctrl);
	rk818_bat_write(di, RK818_CHRG_CTRL_REG1, chrg_ctrl1);
	rk818_bat_write(di, RK818_CHRG_CTRL_REG2, chrg_ctrl2);
	rk818_bat_write(di, RK818_CHRG_CTRL_REG3, chrg_ctrl3);
}

static void rk818_bat_init_coffset(struct rk818_battery *di)
{
	int coffset, ioffset;

	ioffset = rk818_bat_get_ioffset(di);
	di->poffset = rk818_bat_read(di, RK818_POFFSET_REG);
	if (!di->poffset)
		di->poffset = DEFAULT_POFFSET;

	coffset = di->poffset + ioffset;
	if (coffset < INVALID_COFFSET_MIN || coffset > INVALID_COFFSET_MAX)
		coffset = DEFAULT_COFFSET;

	rk818_bat_set_coffset(di, coffset);

	DBG("<%s>. offset: p=0x%x, i=0x%x, c=0x%x\n",
	    __func__, di->poffset, ioffset, rk818_bat_get_coffset(di));
}

static void rk818_bat_caltimer_isr(unsigned long data)
{
	struct rk818_battery *di = (struct rk818_battery *)data;

	mod_timer(&di->caltimer, jiffies + MINUTE(8) * HZ);
	queue_delayed_work(di->bat_monitor_wq, &di->calib_delay_work,
			   msecs_to_jiffies(10));
}

static void rk818_bat_internal_calib(struct work_struct *work)
{
	int ioffset, poffset;
	struct rk818_battery *di = container_of(work,
			struct rk818_battery, calib_delay_work.work);

	/* calib coffset */
	poffset = rk818_bat_read(di, RK818_POFFSET_REG);
	if (poffset)
		di->poffset = poffset;
	else
		di->poffset = DEFAULT_POFFSET;

	ioffset = rk818_bat_get_ioffset(di);
	rk818_bat_set_coffset(di, di->poffset + ioffset);
	rk818_bat_init_voltage_kb(di);
	BAT_INFO("caltimer: ioffset=0x%x, coffset=0x%x\n",
		 ioffset, rk818_bat_get_coffset(di));
}

static void rk818_bat_init_caltimer(struct rk818_battery *di)
{
	setup_timer(&di->caltimer, rk818_bat_caltimer_isr, (unsigned long)di);
	di->caltimer.expires = jiffies + MINUTE(8) * HZ;
	add_timer(&di->caltimer);
	INIT_DELAYED_WORK(&di->calib_delay_work, rk818_bat_internal_calib);
}

static void rk818_bat_init_zero_table(struct rk818_battery *di)
{
	int i, diff, min, max;
	size_t ocv_size, length;

	ocv_size = di->pdata->ocv_size;
	length = sizeof(di->pdata->zero_table) * ocv_size;
	di->pdata->zero_table =
			devm_kzalloc(di->dev, length, GFP_KERNEL);
	if (!di->pdata->zero_table) {
		di->pdata->zero_table = di->pdata->ocv_table;
		dev_err(di->dev, "malloc zero table fail\n");
		return;
	}

	min = di->pdata->pwroff_vol,
	max = di->pdata->ocv_table[ocv_size - 4];
	diff = (max - min) / DIV(ocv_size - 1);
	for (i = 0; i < ocv_size; i++)
		di->pdata->zero_table[i] = min + (i * diff);

	for (i = 0; i < ocv_size; i++)
		DBG("zero[%d] = %d\n", i, di->pdata->zero_table[i]);

	for (i = 0; i < ocv_size; i++)
		DBG("ocv[%d] = %d\n", i, di->pdata->ocv_table[i]);
}

static void rk818_bat_calc_sm_linek(struct rk818_battery *di)
{
	int linek, current_avg;
	u8 diff, delta;

	delta = abs(di->dsoc - di->rsoc);
	diff = delta * 3;/* speed:3/4 */
	current_avg = rk818_bat_get_avg_current(di);
	if (current_avg >= 0) {
		if (di->dsoc < di->rsoc)
			linek = 1000 * (delta + diff) / DIV(diff);
		else if (di->dsoc > di->rsoc)
			linek = 1000 * diff / DIV(delta + diff);
		else
			linek = 1000;
		di->dbg_meet_soc = (di->dsoc >= di->rsoc) ?
				   (di->dsoc + diff) : (di->rsoc + diff);
	} else {
		if (di->dsoc < di->rsoc)
			linek = -1000 * diff / DIV(delta + diff);
		else if (di->dsoc > di->rsoc)
			linek = -1000 * (delta + diff) / DIV(diff);
		else
			linek = -1000;
		di->dbg_meet_soc = (di->dsoc >= di->rsoc) ?
				   (di->dsoc - diff) : (di->rsoc - diff);
	}

	di->sm_linek = linek;
	di->sm_remain_cap = di->remain_cap;
	di->dbg_calc_dsoc = di->dsoc;
	di->dbg_calc_rsoc = di->rsoc;

	DBG("<%s>.diff=%d, k=%d, cur=%d\n", __func__, diff, linek, current_avg);
}

static void rk818_bat_calc_zero_linek(struct rk818_battery *di)
{
	int dead_voltage, ocv_voltage;
	int voltage_avg, current_avg, vsys;
	int ocv_cap, dead_cap, xsoc;
	int ocv_soc, dead_soc;
	int pwroff_vol;
	int i, cnt, vol_old, vol_now;
	int org_linek = 0, min_gap_xsoc;

	if ((abs(di->current_avg) < 500) && (di->dsoc > 10))
		pwroff_vol = di->pdata->pwroff_vol + 50;
	else
		pwroff_vol = di->pdata->pwroff_vol;

	do {
		vol_old = rk818_bat_get_avg_voltage(di);
		msleep(100);
		vol_now = rk818_bat_get_avg_voltage(di);
		cnt++;
	} while ((vol_old == vol_now) && (cnt < 11));

	voltage_avg = 0;
	for (i = 0; i < 10; i++) {
		voltage_avg += rk818_bat_get_avg_voltage(di);
		msleep(100);
	}

	/* calc estimate ocv voltage */
	voltage_avg /= 10;
	current_avg = rk818_bat_get_avg_current(di);
	vsys = voltage_avg + (current_avg * DEF_PWRPATH_RES) / 1000;

	DBG("ZERO0: shtd_vol: org = %d, now = %d, zero_reserve_dsoc = %d\n",
	    di->pdata->pwroff_vol, pwroff_vol, di->pdata->zero_reserve_dsoc);

	dead_voltage = pwroff_vol - current_avg *
				(di->bat_res + DEF_PWRPATH_RES) / 1000;
	ocv_voltage = voltage_avg - (current_avg * di->bat_res) / 1000;
	DBG("ZERO0: dead_voltage(shtd) = %d, ocv_voltage(now) = %d\n",
	    dead_voltage, ocv_voltage);

	/* calc estimate soc and cap */
	dead_soc = rk818_bat_vol_to_zerosoc(di, dead_voltage);
	dead_cap = rk818_bat_vol_to_zerocap(di, dead_voltage);
	DBG("ZERO0: dead_soc = %d, dead_cap = %d\n",
	    dead_soc, dead_cap);

	ocv_soc = rk818_bat_vol_to_zerosoc(di, ocv_voltage);
	ocv_cap = rk818_bat_vol_to_zerocap(di, ocv_voltage);
	DBG("ZERO0: ocv_soc = %d, ocv_cap = %d\n",
	    ocv_soc, ocv_cap);

	/* xsoc: available rsoc */
	xsoc = ocv_soc - dead_soc;

	/* min_gap_xsoc: reserve xsoc */
	if (abs(current_avg) > ZERO_LOAD_LVL1)
		min_gap_xsoc = MIN_ZERO_GAP_XSOC3;
	else if (abs(current_avg) > ZERO_LOAD_LVL2)
		min_gap_xsoc = MIN_ZERO_GAP_XSOC2;
	else
		min_gap_xsoc = MIN_ZERO_GAP_XSOC1;

	if ((xsoc <= 30) && (di->dsoc >= di->pdata->zero_reserve_dsoc))
		min_gap_xsoc = min_gap_xsoc + MIN_ZERO_GAP_CALIB;

	di->zero_remain_cap = di->remain_cap;
	di->zero_timeout_cnt = 0;
	if ((di->dsoc <= 1) && (xsoc > 0)) {
		di->zero_linek = 400;
		di->zero_drop_sec = 0;
	} else if (xsoc >= 0) {
		di->zero_drop_sec = 0;
		di->zero_linek = (di->zero_dsoc + xsoc / 2) / DIV(xsoc);
		org_linek = di->zero_linek;
		/* battery energy mode to use up voltage */
		if ((di->pdata->energy_mode) &&
		    (xsoc - di->dsoc >= MIN_ZERO_GAP_XSOC3) &&
		    (di->dsoc <= 10) && (di->zero_linek < 300)) {
			di->zero_linek = 300;
			DBG("ZERO-new: zero_linek adjust step0...\n");
		/* reserve enough power yet, slow down any way */
		} else if ((xsoc - di->dsoc >= min_gap_xsoc) ||
			   ((xsoc - di->dsoc >= MIN_ZERO_GAP_XSOC2) &&
			    (di->dsoc <= 10) && (xsoc > 15))) {
			if (xsoc <= 20 &&
			    di->dsoc >= di->pdata->zero_reserve_dsoc)
				di->zero_linek = 1200;
			else if (xsoc - di->dsoc >= 2 * min_gap_xsoc)
				di->zero_linek = 400;
			else if (xsoc - di->dsoc >= 3 + min_gap_xsoc)
				di->zero_linek = 600;
			else
				di->zero_linek = 800;
			DBG("ZERO-new: zero_linek adjust step1...\n");
		/* control zero mode beginning enter */
		} else if ((di->zero_linek > 1800) && (di->dsoc > 70)) {
			di->zero_linek = 1800;
			DBG("ZERO-new: zero_linek adjust step2...\n");
		/* dsoc close to xsoc: it must reserve power */
		} else if ((di->zero_linek > 1000) && (di->zero_linek < 1200)) {
			di->zero_linek = 1200;
			DBG("ZERO-new: zero_linek adjust step3...\n");
		/* dsoc[5~15], dsoc < xsoc */
		} else if ((di->dsoc <= 15 && di->dsoc > 5) &&
			   (di->zero_linek <= 1200)) {
			/* slow down */
			if ((xsoc - di->dsoc) >= min_gap_xsoc)
				di->zero_linek = 800;
			/* reserve power */
			else
				di->zero_linek = 1200;
			DBG("ZERO-new: zero_linek adjust step4...\n");
		/* dsoc[5, 100], dsoc < xsoc */
		} else if ((di->zero_linek < 1000) && (di->dsoc >= 5)) {
			if ((xsoc - di->dsoc) < min_gap_xsoc) {
				/* reserve power */
				di->zero_linek = 1200;
			} else {
				if (abs(di->current_avg) > 500)/* heavy */
					di->zero_linek = 900;
				else
					di->zero_linek = 1000;
			}
			DBG("ZERO-new: zero_linek adjust step5...\n");
		/* dsoc[0~5], dsoc < xsoc */
		} else if ((di->zero_linek < 1000) && (di->dsoc <= 5)) {
			if ((xsoc - di->dsoc) <= 3)
				di->zero_linek = 1200;
			else
				di->zero_linek = 800;
			DBG("ZERO-new: zero_linek adjust step6...\n");
		}
	} else {
		/* xsoc < 0 */
		di->zero_linek = 1000;
		if (!di->zero_drop_sec)
			di->zero_drop_sec = get_boot_sec();
		if (base2sec(di->zero_drop_sec) >= WAIT_DSOC_DROP_SEC) {
			DBG("ZERO0: t=%lu\n", base2sec(di->zero_drop_sec));
			di->zero_drop_sec = 0;
			di->dsoc--;
			di->zero_dsoc = (di->dsoc + 1) * 1000 -
						MIN_ACCURACY;
		}
	}

	if (voltage_avg < pwroff_vol - 70) {
		if (!di->shtd_drop_sec)
			di->shtd_drop_sec = get_boot_sec();
		if (base2sec(di->shtd_drop_sec) > WAIT_SHTD_DROP_SEC) {
			BAT_INFO("voltage extreme low...soc:%d->0\n", di->dsoc);
			di->shtd_drop_sec = 0;
			di->dsoc = 0;
		}
	} else {
		di->shtd_drop_sec = 0;
	}

	DBG("ZERO-new: org_linek=%d, zero_linek=%d, dsoc=%d, Xsoc=%d, "
	    "rsoc=%d, gap=%d, v=%d, vsys=%d\n"
	    "ZERO-new: di->zero_dsoc=%d, zero_remain_cap=%d, zero_drop=%ld, "
	    "sht_drop=%ld\n\n",
	    org_linek, di->zero_linek, di->dsoc, xsoc, di->rsoc,
	    min_gap_xsoc, voltage_avg, vsys, di->zero_dsoc, di->zero_remain_cap,
	    base2sec(di->zero_drop_sec), base2sec(di->shtd_drop_sec));
}

static void rk818_bat_finish_algo_prepare(struct rk818_battery *di)
{
	di->chrg_finish_base = get_boot_sec();
	if (!di->chrg_finish_base)
		di->chrg_finish_base = 1;
}

static void rk818_bat_smooth_algo_prepare(struct rk818_battery *di)
{
	int tmp_soc;

	tmp_soc = di->sm_chrg_dsoc / 1000;
	if (tmp_soc != di->dsoc)
		di->sm_chrg_dsoc = di->dsoc * 1000;

	tmp_soc = di->sm_dischrg_dsoc / 1000;
	if (tmp_soc != di->dsoc)
		di->sm_dischrg_dsoc =
		(di->dsoc + 1) * 1000 - MIN_ACCURACY;

	DBG("<%s>. tmp_soc=%d, dsoc=%d, dsoc:sm_dischrg=%d, sm_chrg=%d\n",
	    __func__, tmp_soc, di->dsoc, di->sm_dischrg_dsoc, di->sm_chrg_dsoc);

	rk818_bat_calc_sm_linek(di);
}

static void rk818_bat_zero_algo_prepare(struct rk818_battery *di)
{
	int tmp_dsoc;

	di->zero_timeout_cnt = 0;
	tmp_dsoc = di->zero_dsoc / 1000;
	if (tmp_dsoc != di->dsoc)
		di->zero_dsoc = (di->dsoc + 1) * 1000 - MIN_ACCURACY;

	DBG("<%s>. first calc, reinit linek\n", __func__);

	rk818_bat_calc_zero_linek(di);
}

static void rk818_bat_calc_zero_algorithm(struct rk818_battery *di)
{
	int tmp_soc = 0, sm_delta_dsoc = 0;

	tmp_soc = di->zero_dsoc / 1000;
	if (tmp_soc == di->dsoc)
		goto out;

	DBG("<%s>. enter: dsoc=%d, rsoc=%d\n", __func__, di->dsoc, di->rsoc);
	/* when discharge slow down, take sm chrg into calc */
	if (di->dsoc < di->rsoc) {
		/* take sm charge rest into calc */
		tmp_soc = di->sm_chrg_dsoc / 1000;
		if (tmp_soc == di->dsoc) {
			sm_delta_dsoc = di->sm_chrg_dsoc - di->dsoc * 1000;
			di->sm_chrg_dsoc = di->dsoc * 1000;
			di->zero_dsoc += sm_delta_dsoc;
			DBG("ZERO1: take sm chrg,delta=%d\n", sm_delta_dsoc);
		}
	}

	/* when discharge speed up, take sm dischrg into calc */
	if (di->dsoc > di->rsoc) {
		/* take sm discharge rest into calc */
		tmp_soc = di->sm_dischrg_dsoc / 1000;
		if (tmp_soc == di->dsoc) {
			sm_delta_dsoc = di->sm_dischrg_dsoc -
				((di->dsoc + 1) * 1000 - MIN_ACCURACY);
			di->sm_dischrg_dsoc = (di->dsoc + 1) * 1000 -
								MIN_ACCURACY;
			di->zero_dsoc += sm_delta_dsoc;
			DBG("ZERO1: take sm dischrg,delta=%d\n", sm_delta_dsoc);
		}
	}

	/* check overflow */
	if (di->zero_dsoc > (di->dsoc + 1) * 1000 - MIN_ACCURACY) {
		DBG("ZERO1: zero dsoc overflow: %d\n", di->zero_dsoc);
		di->zero_dsoc = (di->dsoc + 1) * 1000 - MIN_ACCURACY;
	}

	/* check new dsoc */
	tmp_soc = di->zero_dsoc / 1000;
	if (tmp_soc != di->dsoc) {
		/* avoid dsoc jump when heavy load */
		if ((di->dsoc - tmp_soc) > 1) {
			di->dsoc--;
			di->zero_dsoc = (di->dsoc + 1) * 1000 - MIN_ACCURACY;
			DBG("ZERO1: heavy load...\n");
		} else {
			di->dsoc = tmp_soc;
		}
		di->zero_drop_sec = 0;
	}

out:
	DBG("ZERO1: zero_dsoc(Y0)=%d, dsoc=%d, rsoc=%d, tmp_soc=%d\n",
	    di->zero_dsoc, di->dsoc, di->rsoc, tmp_soc);
	DBG("ZERO1: sm_dischrg_dsoc=%d, sm_chrg_dsoc=%d\n",
	    di->sm_dischrg_dsoc, di->sm_chrg_dsoc);
}

static void rk818_bat_zero_algorithm(struct rk818_battery *di)
{
	int delta_cap = 0, delta_soc = 0;

	di->zero_timeout_cnt++;
	delta_cap = di->zero_remain_cap - di->remain_cap;
	delta_soc = di->zero_linek * (delta_cap * 100) / DIV(di->fcc);

	DBG("ZERO1: zero_linek=%d, zero_dsoc(Y0)=%d, dsoc=%d, rsoc=%d\n"
	    "ZERO1: delta_soc(X0)=%d, delta_cap=%d, zero_remain_cap = %d\n"
	    "ZERO1: timeout_cnt=%d, sm_dischrg=%d, sm_chrg=%d\n\n",
	    di->zero_linek, di->zero_dsoc, di->dsoc, di->rsoc,
	    delta_soc, delta_cap, di->zero_remain_cap,
	    di->zero_timeout_cnt, di->sm_dischrg_dsoc, di->sm_chrg_dsoc);

	if ((delta_soc >= MIN_ZERO_DSOC_ACCURACY) ||
	    (di->zero_timeout_cnt > MIN_ZERO_OVERCNT) ||
	    (di->zero_linek == 0)) {
		DBG("ZERO1:--------- enter calc -----------\n");
		di->zero_timeout_cnt = 0;
		di->zero_dsoc -= delta_soc;
		rk818_bat_calc_zero_algorithm(di);
		rk818_bat_calc_zero_linek(di);
	}
}

static void rk818_bat_dump_time_table(struct rk818_battery *di)
{
	u8 i;
	static int old_index;
	static int old_min;
	u32 time;
	int mod = di->dsoc % 10;
	int index = di->dsoc / 10;

	if (rk818_bat_chrg_online(di))
		time = base2min(di->plug_in_base);
	else
		time = base2min(di->plug_out_base);

	if ((mod == 0) && (index > 0) && (old_index != index)) {
		di->dbg_chrg_min[index - 1] = time - old_min;
		old_min = time;
		old_index = index;
	}

	for (i = 1; i < 11; i++)
		DBG("Time[%d]=%d, ", (i * 10), di->dbg_chrg_min[i - 1]);
	DBG("\n");
}

static void rk818_bat_debug_info(struct rk818_battery *di)
{
	u8 sup_tst, ggcon, ggsts, vb_mod, ts_ctrl;
	u8 usb_ctrl, chrg_ctrl1, thermal, reboot_cnt;
	u8 int_sts1, int_sts2;
	u8 int_msk1, int_msk2;
	u8 chrg_ctrl2, chrg_ctrl3, rtc, misc, dcdc_en;
	const char *work_mode[] = {"ZERO", "FINISH", "UN", "UN", "SMOOTH"};
	const char *bat_mode[] = {"BAT", "VIRTUAL"};

	if (rk818_bat_chrg_online(di))
		di->plug_out_base = get_boot_sec();
	else
		di->plug_in_base = get_boot_sec();

	rk818_bat_dump_time_table(di);

	if (!dbg_enable)
		return;

	reboot_cnt = rk818_bat_read(di, RK818_REBOOT_CNT_REG);
	ts_ctrl = rk818_bat_read(di, RK818_TS_CTRL_REG);
	misc = rk818_bat_read(di, RK818_MISC_MARK_REG);
	ggcon = rk818_bat_read(di, RK818_GGCON_REG);
	ggsts = rk818_bat_read(di, RK818_GGSTS_REG);
	sup_tst = rk818_bat_read(di, RK818_SUP_STS_REG);
	vb_mod = rk818_bat_read(di, RK818_VB_MON_REG);
	usb_ctrl = rk818_bat_read(di, RK818_USB_CTRL_REG);
	chrg_ctrl1 = rk818_bat_read(di, RK818_CHRG_CTRL_REG1);
	chrg_ctrl2 = rk818_bat_read(di, RK818_CHRG_CTRL_REG2);
	chrg_ctrl3 = rk818_bat_read(di, RK818_CHRG_CTRL_REG3);
	rtc = rk818_bat_read(di, RK818_SECONDS_REG);
	thermal = rk818_bat_read(di, RK818_THERMAL_REG);
	int_sts1 = rk818_bat_read(di, RK818_INT_STS_REG1);
	int_sts2 = rk818_bat_read(di, RK818_INT_STS_REG2);
	int_msk1 = rk818_bat_read(di, RK818_INT_STS_MSK_REG1);
	int_msk2 = rk818_bat_read(di, RK818_INT_STS_MSK_REG2);
	dcdc_en = rk818_bat_read(di, RK818_DCDC_EN_REG);

	DBG("\n------- DEBUG REGS, [Ver: %s] -------------------\n"
	    "GGCON=0x%2x, GGSTS=0x%2x, RTC=0x%2x, DCDC_EN2=0x%2x\n"
	    "SUP_STS= 0x%2x, VB_MOD=0x%2x, USB_CTRL=0x%2x\n"
	    "THERMAL=0x%2x, MISC_MARK=0x%2x, TS_CTRL=0x%2x\n"
	    "CHRG_CTRL:REG1=0x%2x, REG2=0x%2x, REG3=0x%2x\n"
	    "INT_STS:  REG1=0x%2x, REG2=0x%2x\n"
	    "INT_MSK:  REG1=0x%2x, REG2=0x%2x\n",
	    DRIVER_VERSION, ggcon, ggsts, rtc, dcdc_en,
	    sup_tst, vb_mod, usb_ctrl,
	    thermal, misc, ts_ctrl,
	    chrg_ctrl1, chrg_ctrl2, chrg_ctrl3,
	    int_sts1, int_sts2, int_msk1, int_msk2
	   );

	DBG("###############################################################\n"
	    "Dsoc=%d, Rsoc=%d, Vavg=%d, Iavg=%d, Cap=%d, Fcc=%d, d=%d\n"
	    "K=%d, Mode=%s, Oldcap=%d, Is=%d, Ip=%d, Vs=%d\n"
	    "AC=%d, USB=%d, DC=%d, OTG=%d, PROP=%d, rd=%d, wr=%d, "
	    "Tfb=%d, Tbat=%d, ts2_vol=%d\n"
	    "off:i=0x%x, c=0x%x, p=%d, Rbat=%d, age_ocv_cap=%d, fb=%d\n"
	    "adp:in=%lu, out=%lu, finish=%lu, boot_min=%lu, sleep_min=%lu, adc=%d\n"
	    "bat:%s, meet: soc=%d, calc: dsoc=%d, rsoc=%d, Vocv=%d\n"
	    "pwr: dsoc=%d, rsoc=%d, vol=%d, halt: st=%d, cnt=%d, reboot=%d\n"
	    "ocv_c=%d: %d -> %d; max_c=%d: %d -> %d; force_c=%d: %d -> %d\n"
	    "min=%d, init=%d, sw=%d, below0=%d, first=%d, changed=%d\n"
	    "###############################################################\n",
	    di->dsoc, di->rsoc, di->voltage_avg, di->current_avg,
	    di->remain_cap, di->fcc, di->dsoc - di->rsoc,
	    di->sm_linek, work_mode[di->work_mode], di->sm_remain_cap,
	    CHRG_CUR_SEL[chrg_ctrl1 & 0x0f],
	    CHRG_CUR_INPUT[usb_ctrl & 0x0f],
	    CHRG_VOL_SEL[(chrg_ctrl1 & 0x70) >> 4],
	    di->ac_in, di->usb_in, di->dc_in, di->otg_in, di->prop_val,
	    di->dbg_i2c_rd_err, di->dbg_i2c_wr_err,
	    FEED_BACK_TEMP[(thermal & 0x0c) >> 2], di->temperature,
	    di->voltage_ts2,
	    rk818_bat_get_ioffset(di), rk818_bat_get_coffset(di),
	    di->poffset, di->bat_res, di->age_adjust_cap, di->fb_blank,
	    base2min(di->plug_in_base), base2min(di->plug_out_base),
	    base2min(di->chrg_finish_base),
	    base2min(di->boot_base), di->sleep_sum_sec / 60,
	    di->adc_allow_update,
	    bat_mode[di->pdata->bat_mode], di->dbg_meet_soc,
	    di->dbg_calc_dsoc, di->dbg_calc_rsoc, di->voltage_ocv,
	    di->dbg_pwr_dsoc, di->dbg_pwr_rsoc, di->dbg_pwr_vol, di->is_halt,
	    di->halt_cnt, reboot_cnt,
	    di->is_ocv_calib, di->ocv_pre_dsoc, di->ocv_new_dsoc,
	    di->is_max_soc_offset, di->max_pre_dsoc, di->max_new_dsoc,
	    di->is_force_calib, di->force_pre_dsoc, di->force_new_dsoc,
	    di->pwroff_min, di->is_initialized, di->is_sw_reset,
	    di->dbg_cap_low0, di->is_first_on, di->last_dsoc
	   );
}

static void rk818_bat_init_capacity(struct rk818_battery *di, u32 cap)
{
	int delta_cap;

	delta_cap = cap - di->remain_cap;
	if (!delta_cap)
		return;

	di->age_adjust_cap += delta_cap;
	rk818_bat_init_coulomb_cap(di, cap);
	rk818_bat_smooth_algo_prepare(di);
	rk818_bat_zero_algo_prepare(di);
}

static void rk818_bat_update_age_fcc(struct rk818_battery *di)
{
	int fcc;
	int remain_cap;
	int age_keep_min;
	int lock_fcc;

	lock_fcc = rk818_bat_get_coulomb_cap(di);
	fcc = lock_fcc;
	remain_cap = fcc - di->age_ocv_cap - di->age_adjust_cap;
	age_keep_min = base2min(di->age_keep_sec);

	DBG("%s: lock_fcc=%d, age_ocv_cap=%d, age_adjust_cap=%d, remain_cap=%d,"
	    "age_allow_update=%d, age_keep_min=%d\n",
	    __func__, fcc, di->age_ocv_cap, di->age_adjust_cap, remain_cap,
	    di->age_allow_update, age_keep_min);

	if ((di->chrg_status == CHARGE_FINISH) && (di->age_allow_update) &&
	    (age_keep_min < 1200)) {
		di->age_allow_update = false;
		fcc = remain_cap * 100 / DIV(100 - di->age_ocv_soc);
		BAT_INFO("lock_fcc=%d, calc_cap=%d, age: soc=%d, cap=%d, "
			 "level=%d, fcc:%d->%d?\n",
			 lock_fcc, remain_cap, di->age_ocv_soc,
			 di->age_ocv_cap, di->age_level, di->fcc, fcc);

		if ((fcc < di->qmax) && (fcc > MIN_FCC)) {
			BAT_INFO("fcc:%d->%d!\n", di->fcc, fcc);
			di->fcc = fcc;
			rk818_bat_init_capacity(di, di->fcc);
			rk818_bat_save_fcc(di, di->fcc);
			rk818_bat_save_age_level(di, di->age_level);
		}
	}
}

static void rk818_bat_wait_finish_sig(struct rk818_battery *di)
{
	int chrg_finish_vol = di->pdata->max_chrg_voltage;

	if (!rk818_bat_chrg_online(di))
		return;

	if ((di->chrg_status == CHARGE_FINISH) && (di->adc_allow_update) &&
	    (di->voltage_avg > chrg_finish_vol - 150)) {
		rk818_bat_update_age_fcc(di);
		if (rk818_bat_adc_calib(di))
			di->adc_allow_update = false;
	}
}

static void rk818_bat_finish_algorithm(struct rk818_battery *di)
{
	unsigned long finish_sec, soc_sec;
	int plus_soc, finish_current, rest = 0;

	/* rsoc */
	if ((di->remain_cap != di->fcc) &&
	    (rk818_bat_get_chrg_status(di) == CHARGE_FINISH)) {
		di->age_adjust_cap += (di->fcc - di->remain_cap);
		rk818_bat_init_coulomb_cap(di, di->fcc);
	}

	/* dsoc */
	if (di->dsoc < 100) {
		if (!di->chrg_finish_base)
			di->chrg_finish_base = get_boot_sec();

		finish_current = (di->rsoc - di->dsoc) >  FINISH_MAX_SOC_DELAY ?
					FINISH_CHRG_CUR2 : FINISH_CHRG_CUR1;
		finish_sec = base2sec(di->chrg_finish_base);
		soc_sec = di->fcc * 3600 / 100 / DIV(finish_current);
		plus_soc = finish_sec / DIV(soc_sec);
		if (finish_sec > soc_sec) {
			rest = finish_sec % soc_sec;
			di->dsoc += plus_soc;
			di->chrg_finish_base = get_boot_sec();
			if (di->chrg_finish_base > rest)
				di->chrg_finish_base = get_boot_sec() - rest;
		}
		DBG("<%s>.CHARGE_FINISH:dsoc<100,dsoc=%d\n"
		    "soc_time=%lu, sec_finish=%lu, plus_soc=%d, rest=%d\n",
		    __func__, di->dsoc, soc_sec, finish_sec, plus_soc, rest);
	}
}

static void rk818_bat_calc_smooth_dischrg(struct rk818_battery *di)
{
	int tmp_soc = 0, sm_delta_dsoc = 0, zero_delta_dsoc = 0;

	tmp_soc = di->sm_dischrg_dsoc / 1000;
	if (tmp_soc == di->dsoc)
		goto out;

	DBG("<%s>. enter: dsoc=%d, rsoc=%d\n", __func__, di->dsoc, di->rsoc);
	/* when dischrge slow down, take sm charge rest into calc */
	if (di->dsoc < di->rsoc) {
		tmp_soc = di->sm_chrg_dsoc / 1000;
		if (tmp_soc == di->dsoc) {
			sm_delta_dsoc = di->sm_chrg_dsoc - di->dsoc * 1000;
			di->sm_chrg_dsoc = di->dsoc * 1000;
			di->sm_dischrg_dsoc += sm_delta_dsoc;
			DBG("<%s>. take sm dischrg, delta=%d\n",
			    __func__, sm_delta_dsoc);
		}
	}

	/* when discharge speed up, take zero discharge rest into calc */
	if (di->dsoc > di->rsoc) {
		tmp_soc = di->zero_dsoc / 1000;
		if (tmp_soc == di->dsoc) {
			zero_delta_dsoc = di->zero_dsoc - ((di->dsoc + 1) *
						1000 - MIN_ACCURACY);
			di->zero_dsoc = (di->dsoc + 1) * 1000 - MIN_ACCURACY;
			di->sm_dischrg_dsoc += zero_delta_dsoc;
			DBG("<%s>. take zero schrg, delta=%d\n",
			    __func__, zero_delta_dsoc);
		}
	}

	/* check up overflow */
	if ((di->sm_dischrg_dsoc) > ((di->dsoc + 1) * 1000 - MIN_ACCURACY)) {
		DBG("<%s>. dischrg_dsoc up overflow\n", __func__);
		di->sm_dischrg_dsoc = (di->dsoc + 1) *
					1000 - MIN_ACCURACY;
	}

	/* check new dsoc */
	tmp_soc = di->sm_dischrg_dsoc / 1000;
	if (tmp_soc != di->dsoc) {
		di->dsoc = tmp_soc;
		di->sm_chrg_dsoc = di->dsoc * 1000;
	}
out:
	DBG("<%s>. dsoc=%d, rsoc=%d, dsoc:sm_dischrg=%d, sm_chrg=%d, zero=%d\n",
	    __func__, di->dsoc, di->rsoc, di->sm_dischrg_dsoc, di->sm_chrg_dsoc,
	    di->zero_dsoc);
}

static void rk818_bat_calc_smooth_chrg(struct rk818_battery *di)
{
	int tmp_soc = 0, sm_delta_dsoc = 0, zero_delta_dsoc = 0;

	tmp_soc = di->sm_chrg_dsoc / 1000;
	if (tmp_soc == di->dsoc)
		goto out;

	DBG("<%s>. enter: dsoc=%d, rsoc=%d\n", __func__, di->dsoc, di->rsoc);
	/* when charge slow down, take zero & sm dischrg into calc */
	if (di->dsoc > di->rsoc) {
		/* take sm discharge rest into calc */
		tmp_soc = di->sm_dischrg_dsoc / 1000;
		if (tmp_soc == di->dsoc) {
			sm_delta_dsoc = di->sm_dischrg_dsoc -
					((di->dsoc + 1) * 1000 - MIN_ACCURACY);
			di->sm_dischrg_dsoc = (di->dsoc + 1) * 1000 -
							MIN_ACCURACY;
			di->sm_chrg_dsoc += sm_delta_dsoc;
			DBG("<%s>. take sm dischrg, delta=%d\n",
			    __func__, sm_delta_dsoc);
		}

		/* take zero discharge rest into calc */
		tmp_soc = di->zero_dsoc / 1000;
		if (tmp_soc == di->dsoc) {
			zero_delta_dsoc = di->zero_dsoc -
			((di->dsoc + 1) * 1000 - MIN_ACCURACY);
			di->zero_dsoc = (di->dsoc + 1) * 1000 - MIN_ACCURACY;
			di->sm_chrg_dsoc += zero_delta_dsoc;
			DBG("<%s>. take zero dischrg, delta=%d\n",
			    __func__, zero_delta_dsoc);
		}
	}

	/* check down overflow */
	if (di->sm_chrg_dsoc < di->dsoc * 1000) {
		DBG("<%s>. chrg_dsoc down overflow\n", __func__);
		di->sm_chrg_dsoc = di->dsoc * 1000;
	}

	/* check new dsoc */
	tmp_soc = di->sm_chrg_dsoc / 1000;
	if (tmp_soc != di->dsoc) {
		di->dsoc = tmp_soc;
		di->sm_dischrg_dsoc = (di->dsoc + 1) * 1000 - MIN_ACCURACY;
	}
out:
	DBG("<%s>.dsoc=%d, rsoc=%d, dsoc: sm_dischrg=%d, sm_chrg=%d, zero=%d\n",
	    __func__, di->dsoc, di->rsoc, di->sm_dischrg_dsoc, di->sm_chrg_dsoc,
	    di->zero_dsoc);
}

static void rk818_bat_smooth_algorithm(struct rk818_battery *di)
{
	int ydsoc = 0, delta_cap = 0, old_cap = 0;
	unsigned long tgt_sec = 0;

	di->remain_cap = rk818_bat_get_coulomb_cap(di);

	/* full charge: slow down */
	if ((di->dsoc == 99) && (di->chrg_status == CC_OR_CV)) {
		di->sm_linek = FULL_CHRG_K;
	/* terminal charge, slow down */
	} else if ((di->current_avg >= TERM_CHRG_CURR) &&
	    (di->chrg_status == CC_OR_CV) && (di->dsoc >= TERM_CHRG_DSOC)) {
		di->sm_linek = TERM_CHRG_K;
		DBG("<%s>. terminal mode..\n", __func__);
	/* simulate charge, speed up */
	} else if ((di->current_avg <= SIMULATE_CHRG_CURR) &&
		   (di->current_avg > 0) && (di->chrg_status == CC_OR_CV) &&
		   (di->dsoc < TERM_CHRG_DSOC) &&
		   ((di->rsoc - di->dsoc) >= SIMULATE_CHRG_INTV)) {
		di->sm_linek = SIMULATE_CHRG_K;
		DBG("<%s>. simulate mode..\n", __func__);
	} else {
		/* charge and discharge switch */
		if ((di->sm_linek * di->current_avg <= 0) ||
		    (di->sm_linek == TERM_CHRG_K) ||
		    (di->sm_linek == FULL_CHRG_K) ||
		    (di->sm_linek == SIMULATE_CHRG_K)) {
			DBG("<%s>. linek mode, retinit sm linek..\n", __func__);
			rk818_bat_calc_sm_linek(di);
		}
	}

	old_cap = di->sm_remain_cap;
	/*
	 * when dsoc equal rsoc(not include full, term, simulate case),
	 * sm_linek should change to -1000/1000 smoothly to avoid dsoc+1/-1
	 * right away, so change it after flat seconds
	 */
	if ((di->dsoc == di->rsoc) && (abs(di->sm_linek) != 1000) &&
	    (di->sm_linek != FULL_CHRG_K && di->sm_linek != TERM_CHRG_K &&
	     di->sm_linek != SIMULATE_CHRG_K)) {
		if (!di->flat_match_sec)
			di->flat_match_sec = get_boot_sec();
		tgt_sec = di->fcc * 3600 / 100 / DIV(abs(di->current_avg)) / 3;
		if (base2sec(di->flat_match_sec) >= tgt_sec) {
			di->flat_match_sec = 0;
			di->sm_linek = (di->current_avg >= 0) ? 1000 : -1000;
		}
		DBG("<%s>. flat_sec=%ld, tgt_sec=%ld, sm_k=%d\n", __func__,
		    base2sec(di->flat_match_sec), tgt_sec, di->sm_linek);
	} else {
		di->flat_match_sec = 0;
	}

	/* abs(k)=1000 or dsoc=100, stop calc */
	if ((abs(di->sm_linek) == 1000) || (di->current_avg >= 0 &&
	     di->chrg_status == CC_OR_CV && di->dsoc >= 100)) {
		DBG("<%s>. sm_linek=%d\n", __func__, di->sm_linek);
		if (abs(di->sm_linek) == 1000) {
			di->dsoc = di->rsoc;
			di->sm_linek = (di->sm_linek > 0) ? 1000 : -1000;
			DBG("<%s>. dsoc == rsoc, sm_linek=%d\n",
			    __func__, di->sm_linek);
		}
		di->sm_remain_cap = di->remain_cap;
		di->sm_chrg_dsoc = di->dsoc * 1000;
		di->sm_dischrg_dsoc = (di->dsoc + 1) * 1000 - MIN_ACCURACY;
		DBG("<%s>. sm_dischrg_dsoc=%d, sm_chrg_dsoc=%d\n",
		    __func__, di->sm_dischrg_dsoc, di->sm_chrg_dsoc);
	} else {
		delta_cap = di->remain_cap - di->sm_remain_cap;
		if (delta_cap == 0) {
			DBG("<%s>. delta_cap = 0\n", __func__);
			return;
		}
		ydsoc = di->sm_linek * abs(delta_cap) * 100 / DIV(di->fcc);
		if (ydsoc == 0) {
			DBG("<%s>. ydsoc = 0\n", __func__);
			return;
		}
		di->sm_remain_cap = di->remain_cap;

		DBG("<%s>. k=%d, ydsoc=%d; cap:old=%d, new:%d; delta_cap=%d\n",
		    __func__, di->sm_linek, ydsoc, old_cap,
		    di->sm_remain_cap, delta_cap);

		/* discharge mode */
		if (ydsoc < 0) {
			di->sm_dischrg_dsoc += ydsoc;
			rk818_bat_calc_smooth_dischrg(di);
		/* charge mode */
		} else {
			di->sm_chrg_dsoc += ydsoc;
			rk818_bat_calc_smooth_chrg(di);
		}

		if (di->s2r) {
			di->s2r = false;
			rk818_bat_calc_sm_linek(di);
		}
	}
}

static bool rk818_bat_fake_finish_mode(struct rk818_battery *di)
{
	if ((di->rsoc == 100) && (rk818_bat_get_chrg_status(di) == CC_OR_CV) &&
	    (abs(di->current_avg) <= 100))
		return true;
	else
		return false;
}

static void rk818_bat_display_smooth(struct rk818_battery *di)
{
	/* discharge: reinit "zero & smooth" algorithm to avoid handling dsoc */
	if (di->s2r && !di->sleep_chrg_online) {
		DBG("s2r: discharge, reset algorithm...\n");
		di->s2r = false;
		rk818_bat_zero_algo_prepare(di);
		rk818_bat_smooth_algo_prepare(di);
		return;
	}

	if (di->work_mode == MODE_FINISH) {
		DBG("step1: charge finish...\n");
		rk818_bat_finish_algorithm(di);
		if ((rk818_bat_get_chrg_status(di) != CHARGE_FINISH) &&
		    !rk818_bat_fake_finish_mode(di)) {
			if ((di->current_avg < 0) &&
			    (di->voltage_avg < di->pdata->zero_algorithm_vol)) {
				DBG("step1: change to zero mode...\n");
				rk818_bat_zero_algo_prepare(di);
				di->work_mode = MODE_ZERO;
			} else {
				DBG("step1: change to smooth mode...\n");
				rk818_bat_smooth_algo_prepare(di);
				di->work_mode = MODE_SMOOTH;
			}
		}
	} else if (di->work_mode == MODE_ZERO) {
		DBG("step2: zero algorithm...\n");
		rk818_bat_zero_algorithm(di);
		if ((di->voltage_avg >= di->pdata->zero_algorithm_vol + 50) ||
		    (di->current_avg >= 0)) {
			DBG("step2: change to smooth mode...\n");
			rk818_bat_smooth_algo_prepare(di);
			di->work_mode = MODE_SMOOTH;
		} else if ((rk818_bat_get_chrg_status(di) == CHARGE_FINISH) ||
			   rk818_bat_fake_finish_mode(di)) {
			DBG("step2: change to finish mode...\n");
			rk818_bat_finish_algo_prepare(di);
			di->work_mode = MODE_FINISH;
		}
	} else {
		DBG("step3: smooth algorithm...\n");
		rk818_bat_smooth_algorithm(di);
		if ((di->current_avg < 0) &&
		    (di->voltage_avg < di->pdata->zero_algorithm_vol)) {
			DBG("step3: change to zero mode...\n");
			rk818_bat_zero_algo_prepare(di);
			di->work_mode = MODE_ZERO;
		} else if ((rk818_bat_get_chrg_status(di) == CHARGE_FINISH) ||
			   rk818_bat_fake_finish_mode(di)) {
			DBG("step3: change to finish mode...\n");
			rk818_bat_finish_algo_prepare(di);
			di->work_mode = MODE_FINISH;
		}
	}
}

static void rk818_bat_relax_vol_calib(struct rk818_battery *di)
{
	int soc, cap, vol;

	vol = di->voltage_relax;
	soc = rk818_bat_vol_to_ocvsoc(di, vol);
	cap = rk818_bat_vol_to_ocvcap(di, vol);
	rk818_bat_init_capacity(di, cap);
	BAT_INFO("sleep ocv calib: rsoc=%d, cap=%d\n", soc, cap);
}

static void rk818_bat_relife_age_flag(struct rk818_battery *di)
{
	u8 ocv_soc, ocv_cap, soc_level;

	if (di->voltage_relax <= 0)
		return;

	ocv_soc = rk818_bat_vol_to_ocvsoc(di, di->voltage_relax);
	ocv_cap = rk818_bat_vol_to_ocvcap(di, di->voltage_relax);
	DBG("<%s>. ocv_soc=%d, min=%lu, vol=%d\n", __func__,
	    ocv_soc, di->sleep_dischrg_sec / 60, di->voltage_relax);

	/* sleep enough time and ocv_soc enough low */
	if (!di->age_allow_update && ocv_soc <= 10) {
		di->age_voltage = di->voltage_relax;
		di->age_ocv_cap = ocv_cap;
		di->age_ocv_soc = ocv_soc;
		di->age_adjust_cap = 0;

		if (ocv_soc <= 1)
			di->age_level = 100;
		else if (ocv_soc < 5)
			di->age_level = 90;
		else
			di->age_level = 80;

		soc_level = rk818_bat_get_age_level(di);
		if (soc_level > di->age_level) {
			di->age_allow_update = false;
		} else {
			di->age_allow_update = true;
			di->age_keep_sec = get_boot_sec();
		}

		BAT_INFO("resume: age_vol:%d, age_ocv_cap:%d, age_ocv_soc:%d, "
			 "soc_level:%d, age_allow_update:%d, "
			 "age_level:%d\n",
			 di->age_voltage, di->age_ocv_cap, ocv_soc, soc_level,
			 di->age_allow_update, di->age_level);
	}
}

static int rk818_bat_sleep_dischrg(struct rk818_battery *di)
{
	bool ocv_soc_updated = false;
	int tgt_dsoc, gap_soc, sleep_soc = 0;
	int pwroff_vol = di->pdata->pwroff_vol;
	unsigned long sleep_sec = di->sleep_dischrg_sec;

	DBG("<%s>. enter: dsoc=%d, rsoc=%d, rv=%d, v=%d, sleep_min=%lu\n",
	    __func__, di->dsoc, di->rsoc, di->voltage_relax,
	    di->voltage_avg, sleep_sec / 60);

	if (di->voltage_relax >= di->voltage_avg) {
		rk818_bat_relax_vol_calib(di);
		rk818_bat_restart_relax(di);
		rk818_bat_relife_age_flag(di);
		ocv_soc_updated = true;
	}

	/* handle dsoc */
	if (di->dsoc <= di->rsoc) {
		di->sleep_sum_cap = (SLP_CURR_MIN * sleep_sec / 3600);
		sleep_soc = di->sleep_sum_cap * 100 / DIV(di->fcc);
		tgt_dsoc = di->dsoc - sleep_soc;
		if (sleep_soc > 0) {
			BAT_INFO("calib0: rl=%d, dl=%d, intval=%d\n",
				 di->rsoc, di->dsoc, sleep_soc);
			if (di->dsoc < 5) {
				di->dsoc--;
			} else if ((tgt_dsoc < 5) && (di->dsoc >= 5)) {
				if (di->dsoc == 5)
					di->dsoc--;
				else
					di->dsoc = 5;
			} else if (tgt_dsoc > 5) {
				di->dsoc = tgt_dsoc;
			}
		}

		DBG("%s: dsoc<=rsoc, sum_cap=%d==>sleep_soc=%d, tgt_dsoc=%d\n",
		    __func__, di->sleep_sum_cap, sleep_soc, tgt_dsoc);
	} else {
		/* di->dsoc > di->rsoc */
		di->sleep_sum_cap = (SLP_CURR_MAX * sleep_sec / 3600);
		sleep_soc = di->sleep_sum_cap / DIV(di->fcc / 100);
		gap_soc = di->dsoc - di->rsoc;

		BAT_INFO("calib1: rsoc=%d, dsoc=%d, intval=%d\n",
			 di->rsoc, di->dsoc, sleep_soc);
		if (gap_soc > sleep_soc) {
			if ((gap_soc - 5) > (sleep_soc * 2))
				di->dsoc -= (sleep_soc * 2);
			else
				di->dsoc -= sleep_soc;
		} else {
			di->dsoc = di->rsoc;
		}

		DBG("%s: dsoc>rsoc, sum_cap=%d=>sleep_soc=%d, gap_soc=%d\n",
		    __func__, di->sleep_sum_cap, sleep_soc, gap_soc);
	}

	if (di->voltage_avg <= pwroff_vol - 70) {
		di->dsoc = 0;
		rk_send_wakeup_key();
		BAT_INFO("low power sleeping, shutdown... %d\n", di->dsoc);
	}

	if (ocv_soc_updated && sleep_soc && (di->rsoc - di->dsoc) < 5 &&
	    di->dsoc < 40) {
		di->dsoc--;
		BAT_INFO("low power sleeping, reserved... %d\n", di->dsoc);
	}

	if (di->dsoc <= 0) {
		di->dsoc = 0;
		rk_send_wakeup_key();
		BAT_INFO("sleep dsoc is %d...\n", di->dsoc);
	}

	DBG("<%s>. out: dsoc=%d, rsoc=%d, sum_cap=%d\n",
	    __func__, di->dsoc, di->rsoc, di->sleep_sum_cap);

	return sleep_soc;
}

static void rk818_bat_power_supply_changed(struct rk818_battery *di)
{
	u8 status;
	static int old_soc = -1;

	/* check dsoc */
	if (di->dsoc > 100)
		di->dsoc = 100;
	else if (di->dsoc < 0)
		di->dsoc = 0;

	/* update prop and leds */
	if (rk818_bat_chrg_online(di)) {
		if (di->dsoc == 100)
			di->prop_val = POWER_SUPPLY_STATUS_FULL;
		else
			di->prop_val = POWER_SUPPLY_STATUS_CHARGING;
		rk818_bat_update_leds(di, di->prop_val);
	}

	if (di->dsoc == old_soc)
		return;

	status = rk818_bat_read(di, RK818_SUP_STS_REG);
	status = (status & CHRG_STATUS_MSK) >> 4;
	old_soc = di->dsoc;
	di->last_dsoc = di->dsoc;
	power_supply_changed(&di->bat);
	BAT_INFO("changed: dsoc=%d, rsoc=%d, v=%d, ov=%d c=%d, "
		 "cap=%d, f=%d, st=%s\n",
		 di->dsoc, di->rsoc, di->voltage_avg, di->voltage_ocv,
		 di->current_avg, di->remain_cap, di->fcc, bat_status[status]);

	BAT_INFO("dl=%d, rl=%d, v=%d, halt=%d, halt_n=%d, max=%d, "
		 "init=%d, sw=%d, calib=%d, below0=%d, force=%d\n",
	         di->dbg_pwr_dsoc, di->dbg_pwr_rsoc, di->dbg_pwr_vol,
		 di->is_halt, di->halt_cnt, di->is_max_soc_offset,
		 di->is_initialized, di->is_sw_reset, di->is_ocv_calib,
		 di->dbg_cap_low0, di->is_force_calib);
}

static u8 rk818_bat_check_reboot(struct rk818_battery *di)
{
	u8 cnt;

	cnt = rk818_bat_read(di, RK818_REBOOT_CNT_REG);
	cnt++;

	if (cnt >= REBOOT_MAX_CNT) {
		BAT_INFO("reboot: %d --> %d\n", di->dsoc, di->rsoc);
		di->dsoc = di->rsoc;
		if (di->dsoc > 100)
			di->dsoc = 100;
		else if (di->dsoc < 0)
			di->dsoc = 0;
		rk818_bat_save_dsoc(di, di->dsoc);
		cnt = REBOOT_MAX_CNT;
	}

	rk818_bat_save_reboot_cnt(di, cnt);
	DBG("reboot cnt: %d\n", cnt);

	return cnt;
}

static void rk818_bat_check_charger(struct rk818_battery *di)
{
	u8 buf;
	int usb_id;
	enum charger_type charger;

	buf = rk818_bat_read(di, RK818_VB_MON_REG);
	/* pmic detect plug in, but ac/usb/dc_in offline, do check */
	if ((buf & PLUG_IN_STS) != 0 && !rk818_bat_chrg_online(di)) {
		usb_id = dwc_otg_check_dpdm(0);
		switch (usb_id) {
		case 0:
			charger = USB_TYPE_AC_CHARGER;
			break;
		case 1:
		case 3:
			charger = USB_TYPE_USB_CHARGER;
			break;
		case 2:
			charger = USB_TYPE_AC_CHARGER;
			break;
		default:
			break;
		}

		rk818_bat_set_chrg_param(di, charger);
		BAT_INFO("pmic detect charger.. %s\n",
			 charger == USB_TYPE_AC_CHARGER ? "AC" : "USB");
	/* pmic not detect plug in, but one of ac/usb/dc_in online, reset */
	} else if ((buf & PLUG_IN_STS) == 0 && rk818_bat_chrg_online(di)) {
		rk818_bat_set_chrg_param(di, USB_DC_TYPE_NONE_CHARGER);
		BAT_INFO("pmic not detect charger..\n");
	}
}

static void rk818_bat_rsoc_daemon(struct rk818_battery *di)
{
	int est_vol, remain_cap;
	static unsigned long sec;

	if ((di->remain_cap < 0) && (di->fb_blank != 0)) {
		if (!sec)
			sec = get_boot_sec();
		wake_lock_timeout(&di->wake_lock,
				  (di->pdata->monitor_sec + 1) * HZ);
		DBG("sec=%ld, hold_sec=%ld\n", sec, base2sec(sec));
		if (base2sec(sec) >= 60) {
			sec = 0;
			di->dbg_cap_low0++;
			est_vol = di->voltage_avg -
					(di->bat_res * di->current_avg) / 1000;
			remain_cap = rk818_bat_vol_to_ocvcap(di, est_vol);
			rk818_bat_init_capacity(di, remain_cap);
			BAT_INFO("adjust cap below 0 --> %d, rsoc=%d\n",
				 di->remain_cap, di->rsoc);
			wake_unlock(&di->wake_lock);
		}
	} else {
		sec = 0;
	}
}

static void rk818_bat_update_info(struct rk818_battery *di)
{
	di->voltage_avg = rk818_bat_get_avg_voltage(di);
	di->current_avg = rk818_bat_get_avg_current(di);
	di->chrg_status = rk818_bat_get_chrg_status(di);
	di->voltage_relax = rk818_bat_get_relax_voltage(di);
	di->rsoc = rk818_bat_get_rsoc(di);
	di->remain_cap = rk818_bat_get_coulomb_cap(di);
	di->voltage_ts2 = rk818_bat_get_ts2_voltage(di);

	/* smooth charge */
	if (di->remain_cap > di->fcc) {
		di->sm_remain_cap -= (di->remain_cap - di->fcc);
		DBG("<%s>. cap: remain=%d, sm_remain=%d\n",
		    __func__, di->remain_cap, di->sm_remain_cap);
		rk818_bat_init_coulomb_cap(di, di->fcc);
	}

	if (di->chrg_status != CHARGE_FINISH)
		di->chrg_finish_base = get_boot_sec();

	/*
	 * we need update fcc in continuous charging state, if discharge state
	 * keep at least 2 hour, we decide not to update fcc, so clear the
	 * fcc update flag: age_allow_update.
	 */
	if (base2min(di->plug_out_base) > 120)
		di->age_allow_update = false;
	/* do adc calib: status must from cccv mode to finish mode */
	if (di->chrg_status == CC_OR_CV) {
		di->adc_allow_update = true;
		di->adc_calib_cnt = 0;
	}
}

static void rk818_bat_init_dsoc_algorithm(struct rk818_battery *di)
{
	u8 buf;
	int16_t rest = 0;
	unsigned long soc_sec;
	const char *mode_name[] = { "MODE_ZERO", "MODE_FINISH",
		"MODE_SMOOTH_CHRG", "MODE_SMOOTH_DISCHRG", "MODE_SMOOTH", };

	/* get rest */
	rest |= rk818_bat_read(di, RK818_CALC_REST_REGH) << 8;
	rest |= rk818_bat_read(di, RK818_CALC_REST_REGL) << 0;

	/* get mode */
	buf = rk818_bat_read(di, RK818_MISC_MARK_REG);
	di->algo_rest_mode = (buf & ALGO_REST_MODE_MSK) >> ALGO_REST_MODE_SHIFT;

	if (rk818_bat_get_chrg_status(di) == CHARGE_FINISH) {
		if (di->algo_rest_mode == MODE_FINISH) {
			soc_sec = di->fcc * 3600 / 100 / FINISH_CHRG_CUR1;
			if ((rest / DIV(soc_sec)) > 0) {
				if (di->dsoc < 100) {
					di->dsoc++;
					di->algo_rest_val = rest % soc_sec;
					BAT_INFO("algorithm rest(%d) dsoc "
						 "inc: %d\n",
						 rest, di->dsoc);
				} else {
					di->algo_rest_val = 0;
				}
			} else {
				di->algo_rest_val = rest;
			}
		} else {
			di->algo_rest_val = rest;
		}
	} else {
		buf = rk818_bat_read(di, RK818_VB_MON_REG);
		/* charge speed up */
		if ((rest / 1000) > 0 && (buf & PLUG_IN_STS)) {
			if (di->dsoc < di->rsoc) {
				di->dsoc++;
				di->algo_rest_val = rest % 1000;
				BAT_INFO("algorithm rest(%d) dsoc inc: %d\n",
					 rest, di->dsoc);
			} else {
				di->algo_rest_val = 0;
			}
		/* discharge speed up */
		} else if (((rest / 1000) < 0) && !(buf & PLUG_IN_STS)) {
			if (di->dsoc > di->rsoc) {
				di->dsoc--;
				di->algo_rest_val = rest % 1000;
				BAT_INFO("algorithm rest(%d) dsoc sub: %d\n",
					 rest, di->dsoc);
			} else {
				di->algo_rest_val = 0;
			}
		} else {
			di->algo_rest_val = rest;
		}
	}

	if (di->dsoc >= 100)
		di->dsoc = 100;
	else if (di->dsoc <= 0)
		di->dsoc = 0;

	/* init current mode */
	di->voltage_avg = rk818_bat_get_avg_voltage(di);
	di->current_avg = rk818_bat_get_avg_current(di);
	if (rk818_bat_get_chrg_status(di) == CHARGE_FINISH) {
		rk818_bat_finish_algo_prepare(di);
		di->work_mode = MODE_FINISH;
	} else {
		rk818_bat_smooth_algo_prepare(di);
		di->work_mode = MODE_SMOOTH;
	}

	DBG("<%s>. init: org_rest=%d, rest=%d, mode=%s; "
	    "doc(x1000): zero=%d, chrg=%d, dischrg=%d, finish=%lu\n",
	    __func__, rest, di->algo_rest_val, mode_name[di->algo_rest_mode],
	    di->zero_dsoc, di->sm_chrg_dsoc, di->sm_dischrg_dsoc,
	    di->chrg_finish_base);
}

static void rk818_bat_save_algo_rest(struct rk818_battery *di)
{
	u8 buf, mode;
	int16_t algo_rest = 0;
	int tmp_soc;
	int zero_rest = 0, sm_chrg_rest = 0;
	int sm_dischrg_rest = 0, finish_rest = 0;
	const char *mode_name[] = { "MODE_ZERO", "MODE_FINISH",
		"MODE_SMOOTH_CHRG", "MODE_SMOOTH_DISCHRG", "MODE_SMOOTH", };

	/* zero dischrg */
	tmp_soc = (di->zero_dsoc) / 1000;
	if (tmp_soc == di->dsoc)
		zero_rest = di->zero_dsoc - ((di->dsoc + 1) * 1000 -
				MIN_ACCURACY);

	/* sm chrg */
	tmp_soc = di->sm_chrg_dsoc / 1000;
	if (tmp_soc == di->dsoc)
		sm_chrg_rest = di->sm_chrg_dsoc - di->dsoc * 1000;

	/* sm dischrg */
	tmp_soc = (di->sm_dischrg_dsoc) / 1000;
	if (tmp_soc == di->dsoc)
		sm_dischrg_rest = di->sm_dischrg_dsoc - ((di->dsoc + 1) * 1000 -
				MIN_ACCURACY);

	/* last time is also finish chrg, then add last rest */
	if (di->algo_rest_mode == MODE_FINISH && di->algo_rest_val)
		finish_rest = base2sec(di->chrg_finish_base) +
				       di->algo_rest_val;
	else
		finish_rest = base2sec(di->chrg_finish_base);

	/* total calc */
	if ((rk818_bat_chrg_online(di) && (di->dsoc > di->rsoc)) ||
	    (!rk818_bat_chrg_online(di) && (di->dsoc < di->rsoc)) ||
	    (di->dsoc == di->rsoc)) {
		di->algo_rest_val = 0;
		algo_rest = 0;
		DBG("<%s>. step1..\n", __func__);
	} else if (di->work_mode == MODE_FINISH) {
		algo_rest = finish_rest;
		DBG("<%s>. step2..\n", __func__);
	} else if (di->algo_rest_mode == MODE_FINISH) {
		algo_rest = zero_rest + sm_dischrg_rest + sm_chrg_rest;
		DBG("<%s>. step3..\n", __func__);
	} else {
		if (rk818_bat_chrg_online(di) && (di->dsoc < di->rsoc))
			algo_rest = sm_chrg_rest + di->algo_rest_val;
		else if (!rk818_bat_chrg_online(di) && (di->dsoc > di->rsoc))
			algo_rest = zero_rest + sm_dischrg_rest +
				    di->algo_rest_val;
		else
			algo_rest = zero_rest + sm_dischrg_rest + sm_chrg_rest +
				    di->algo_rest_val;
		DBG("<%s>. step4..\n", __func__);
	}

	/* check mode */
	if ((di->work_mode == MODE_FINISH) || (di->work_mode == MODE_ZERO)) {
		mode = di->work_mode;
	} else {/* MODE_SMOOTH */
		if (di->sm_linek > 0)
			mode = MODE_SMOOTH_CHRG;
		else
			mode = MODE_SMOOTH_DISCHRG;
	}

	/* save mode */
	buf = rk818_bat_read(di, RK818_MISC_MARK_REG);
	buf &= ~ALGO_REST_MODE_MSK;
	buf |= (mode << ALGO_REST_MODE_SHIFT);
	rk818_bat_write(di, RK818_MISC_MARK_REG, buf);

	/* save rest */
	buf = (algo_rest >> 8) & 0xff;
	rk818_bat_write(di, RK818_CALC_REST_REGH, buf);
	buf = (algo_rest >> 0) & 0xff;
	rk818_bat_write(di, RK818_CALC_REST_REGL, buf);

	DBG("<%s>. rest: algo=%d, mode=%s, last_rest=%d; zero=%d, "
	    "chrg=%d, dischrg=%d, finish=%lu\n",
	    __func__, algo_rest, mode_name[mode], di->algo_rest_val, zero_rest,
	    sm_chrg_rest, sm_dischrg_rest, base2sec(di->chrg_finish_base));
}

static void rk818_bat_save_data(struct rk818_battery *di)
{
	rk818_bat_save_dsoc(di, di->dsoc);
	rk818_bat_save_cap(di, di->remain_cap);
	rk818_bat_save_algo_rest(di);
}

/*get ntc resistance*/
static int rk818_bat_get_ntc_res(struct rk818_battery *di)
{
	int val = 0;

	val |= rk818_bat_read(di, RK818_TS1_ADC_REGL) << 0;
	val |= rk818_bat_read(di, RK818_TS1_ADC_REGH) << 8;

	val = val * NTC_CALC_FACTOR; /*reference voltage 2.2V,current 80ua*/
	DBG("<%s>. ntc_res=%d\n", __func__, val);

	return val;
}

static void rk818_bat_update_temperature(struct rk818_battery *di)
{
	u32 ntc_size, *ntc_table;
	int i, res;

	ntc_table = di->pdata->ntc_table;
	ntc_size = di->pdata->ntc_size;
	di->temperature = VIRTUAL_TEMPERATURE;

	if (ntc_size) {
		res = rk818_bat_get_ntc_res(di);
		if (res < ntc_table[ntc_size - 1]) {
			BAT_INFO("bat ntc upper max degree: R=%d\n", res);
		} else if (res > ntc_table[0]) {
			BAT_INFO("bat ntc lower min degree: R=%d\n", res);
		} else {
			for (i = 0; i < ntc_size; i++) {
				if (res >= ntc_table[i])
					break;
			}
			di->temperature = (i + di->pdata->ntc_degree_from) * 10;
		}
	}
}

static void rk818_battery_work(struct work_struct *work)
{
	struct rk818_battery *di =
		container_of(work, struct rk818_battery, bat_delay_work.work);

	rk818_bat_update_info(di);
	rk818_bat_wait_finish_sig(di);
	rk818_bat_rsoc_daemon(di);
	rk818_bat_check_charger(di);
	rk818_bat_update_temperature(di);
	rk818_bat_lowpwr_check(di);
	rk818_bat_display_smooth(di);
	rk818_bat_power_supply_changed(di);
	rk818_bat_save_data(di);
	rk818_bat_debug_info(di);

	queue_delayed_work(di->bat_monitor_wq, &di->bat_delay_work,
			   msecs_to_jiffies(di->monitor_ms));
}

static void rk818_bat_bc_delay_work(struct work_struct *work)
{
	enum charger_type charger;
	struct rk818_battery *di = container_of(work,
			struct rk818_battery, bc_delay_work.work);
	const char *event_name[] = {"DISCNT", "USB", "AC", "AC",
				    "UNKNOWN", "OTG ON", "OTG OFF"};

	BAT_INFO("receive bc notifier event: %s..\n", event_name[di->bc_event]);
	switch (di->bc_event) {
	case USB_BC_TYPE_DISCNT:
		rk818_bat_set_chrg_param(di, USB_TYPE_NONE_CHARGER);
		break;
	case USB_BC_TYPE_SDP:
		charger = rk818_bat_get_acusb_state(di);
		DBG("%s: charger=%d\n", __func__, charger);
		if (charger != USB_TYPE_NONE_CHARGER)
			rk818_bat_set_chrg_param(di, USB_TYPE_USB_CHARGER);
		break;
	case USB_BC_TYPE_CDP:
	case USB_BC_TYPE_DCP:
		charger = rk818_bat_get_acusb_state(di);
		DBG("%s: charger=%d\n", __func__, charger);
		if (charger != USB_TYPE_NONE_CHARGER)
			rk818_bat_set_chrg_param(di, USB_TYPE_AC_CHARGER);
		break;
	case USB_OTG_POWER_ON:
		di->otg_in = ONLINE;
		if (di->pdata->power_dc2otg && di->dc_in) {
			BAT_INFO("otg power from dc adapter\n");
		} else {
			rk818_bat_set_otg_state(di, USB_OTG_POWER_ON);
			BAT_INFO("charge disable, enable otg\n");
		}
		break;
	case USB_OTG_POWER_OFF:
		di->otg_in = OFFLINE;
		rk818_bat_set_otg_state(di, USB_OTG_POWER_OFF);
		BAT_INFO("charge enable, disable otg\n");
		break;
	default:
		break;
	}
}

static int rk818_bat_usb_notifier_call(struct notifier_block *nb,
				       unsigned long event, void *data)
{
	struct rk818_battery *di =
	    container_of(nb, struct rk818_battery, bc_detect_nb);

	di->bc_event = event;
	queue_delayed_work(di->charger_wq,
		   &di->bc_delay_work, msecs_to_jiffies(10));

	return NOTIFY_OK;
}

static void rk818_bat_irq_delay_work(struct work_struct *work)
{
	struct rk818_battery *di = container_of(work,
			struct rk818_battery, irq_delay_work.work);

	if (di->vb_low_irq) {
		di->vb_low_irq = 0;
		BAT_INFO("lower power yet, power off system! v=%d\n",
			 di->voltage_avg);
		di->dsoc = 0;
		rk_send_wakeup_key();
		power_supply_changed(&di->bat);
	} else if (di->plug_in_irq) {
		di->plug_in_irq = 0;
		rk_send_wakeup_key();
		BAT_INFO("pmic: plug in\n");
	} else if (di->plug_out_irq) {
		di->plug_out_irq = 0;
		rk_send_wakeup_key();
		BAT_INFO("pmic: plug out\n");
	} else {
		BAT_INFO("pmic: unknown irq\n");
	}
}

static irqreturn_t rk818_vb_low_irq(int irq, void *bat)
{
	struct rk818_battery *di = (struct rk818_battery *)bat;

	di->vb_low_irq = 1;
	queue_delayed_work(di->charger_wq, &di->irq_delay_work, 0);

	return IRQ_HANDLED;
}

static irqreturn_t rk818_plug_in(int irq, void *bat)
{
	struct rk818_battery *di = (struct rk818_battery *)bat;

	di->plug_in_irq = 1;
	queue_delayed_work(di->charger_wq, &di->irq_delay_work, 0);

	return IRQ_HANDLED;
}

static irqreturn_t rk818_plug_out(int irq, void  *bat)
{
	struct rk818_battery *di = (struct rk818_battery *)bat;

	di->plug_out_irq = 1;
	queue_delayed_work(di->charger_wq, &di->irq_delay_work, 0);

	return IRQ_HANDLED;
}

static irqreturn_t rk818_vbat_dc_det(int irq, void *bat)
{
	struct rk818_battery *di = (struct rk818_battery *)bat;

	if (gpio_get_value(di->pdata->dc_det_pin))
		irq_set_irq_type(irq, IRQF_TRIGGER_LOW);
	else
		irq_set_irq_type(irq, IRQF_TRIGGER_HIGH);

	BAT_INFO("dc det in/out\n");
	queue_delayed_work(di->charger_wq,
			   &di->dc_delay_work, msecs_to_jiffies(10));
	rk_send_wakeup_key();

	return IRQ_HANDLED;
}

static void rk818_bat_init_sysfs(struct rk818_battery *di)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(rk818_bat_attr); i++) {
		ret = sysfs_create_file(&di->bat.dev->kobj,
					&rk818_bat_attr[i].attr);
		if (ret)
			dev_err(di->dev, "create bat node(%s) error\n",
				rk818_bat_attr[i].attr.name);
	}
}

static int rk818_bat_init_irqs(struct rk818_battery *di)
{
	int ret, plug_in_irq, plug_out_irq, vb_lo_irq;
	struct rk818 *rk818 = di->rk818;
	struct platform_device *pdev = di->pdev;

	vb_lo_irq = irq_create_mapping(rk818->irq_domain, RK818_IRQ_VB_LO);
	if (vb_lo_irq < 0) {
		dev_err(&pdev->dev, "vb_lo_irq request failed!\n");
		return vb_lo_irq;
	}

	plug_in_irq = irq_create_mapping(rk818->irq_domain, RK818_IRQ_PLUG_IN);
	if (plug_in_irq < 0) {
		dev_err(&pdev->dev, "vb_lo_irq request failed!\n");
		return vb_lo_irq;
	}

	plug_out_irq = irq_create_mapping(rk818->irq_domain,
					  RK818_IRQ_PLUG_OUT);
	if (plug_out_irq < 0) {
		dev_err(&pdev->dev, "vb_lo_irq request failed!\n");
		return vb_lo_irq;
	}

	ret = devm_request_threaded_irq(di->dev, vb_lo_irq, NULL,
					rk818_vb_low_irq, IRQF_TRIGGER_HIGH,
					"rk818_vb_low", di);
	if (ret != 0) {
		dev_err(&pdev->dev, "vb_lo_irq request failed!\n");
		return vb_lo_irq;
	}
	enable_irq_wake(vb_lo_irq);

	ret = devm_request_threaded_irq(di->dev, plug_in_irq, NULL,
					rk818_plug_in, IRQF_TRIGGER_RISING,
					"rk818_plug_in", di);
	if (ret != 0) {
		dev_err(&pdev->dev, "plug_in_irq request failed!\n");
		return plug_in_irq;
	}

	ret = devm_request_threaded_irq(di->dev, plug_out_irq, NULL,
					rk818_plug_out, IRQF_TRIGGER_FALLING,
					"rk818_plug_out", di);
	if (ret != 0) {
		dev_err(&pdev->dev, "plug_out_irq request failed!\n");
		return plug_out_irq;
	}

	return 0;
}

static void rk818_bat_init_info(struct rk818_battery *di)
{
	di->design_cap = di->pdata->design_capacity;
	di->qmax = di->pdata->design_qmax;
	di->bat_res = di->pdata->bat_res;
	di->sleep_chrg_status = rk818_bat_get_chrg_status(di);
	di->monitor_ms = di->pdata->monitor_sec * TIMER_MS_COUNTS;
	di->prop_val = POWER_SUPPLY_STATUS_DISCHARGING;
	di->boot_base = POWER_ON_SEC_BASE;
	di->chrg_finish_base = 0;
	di->plug_in_base = 0;
	di->plug_out_base = 0;
}

static enum charger_type rk818_bat_init_dc_det(struct rk818_battery *di)
{
	int ret, level;
	unsigned long irq_flags;
	unsigned int dc_det_irq;
	enum charger_type type = DC_TYPE_NONE_CHARGER;

	if (gpio_is_valid(di->pdata->dc_det_pin)) {
		ret = devm_gpio_request(di->dev, di->pdata->dc_det_pin,
					"rk818_dc_det");
		if (ret < 0) {
			dev_err(di->dev, "Failed to request gpio %d\n",
				di->pdata->dc_det_pin);
			goto out;
		}

		ret = gpio_direction_input(di->pdata->dc_det_pin);
		if (ret) {
			dev_err(di->dev, "failed to set gpio input\n");
			goto out;
		}

		level = gpio_get_value(di->pdata->dc_det_pin);
		if (level == di->pdata->dc_det_level)
			type = DC_TYPE_DC_CHARGER;
		else
			type = DC_TYPE_NONE_CHARGER;

		if (level)
			irq_flags = IRQF_TRIGGER_LOW;
		else
			irq_flags = IRQF_TRIGGER_HIGH;

		dc_det_irq = gpio_to_irq(di->pdata->dc_det_pin);
		ret = devm_request_irq(di->dev, dc_det_irq, rk818_vbat_dc_det,
				       irq_flags, "rk818_dc_det", di);
		if (ret != 0) {
			dev_err(di->dev, "rk818_dc_det_irq request failed!\n");
			goto out;
		}

		enable_irq_wake(dc_det_irq);
	}
out:
	return type;
}

static void rk818_bat_init_charger(struct rk818_battery *di)
{
	enum charger_type dc_charger, usb_charger;

	di->charger_wq = alloc_ordered_workqueue("%s",
				WQ_MEM_RECLAIM | WQ_FREEZABLE,
				"rk818-bat-charger-wq");

	di->ts2_wq = alloc_ordered_workqueue("%s",
				WQ_MEM_RECLAIM | WQ_FREEZABLE,
				"rk818-bat-ts2-wq");

	INIT_DELAYED_WORK(&di->dc_delay_work, rk818_bat_dc_delay_work);
	INIT_DELAYED_WORK(&di->bc_delay_work, rk818_bat_bc_delay_work);
	INIT_DELAYED_WORK(&di->term_mode_work, rk818_term_mode_work);
	INIT_DELAYED_WORK(&di->ts2_vol_work, rk818_ts2_vol_work);

	di->bc_detect_nb.notifier_call = rk818_bat_usb_notifier_call;
	rk_bc_detect_notifier_register(&di->bc_detect_nb, &di->charge_otg);

	dc_charger = rk818_bat_init_dc_det(di);
	usb_charger = rk818_bat_get_acusb_state(di);
	di->usb_in = (usb_charger == USB_TYPE_USB_CHARGER) ? ONLINE : OFFLINE;
	di->ac_in = (usb_charger == USB_TYPE_AC_CHARGER) ? ONLINE : OFFLINE;
	di->dc_in = (dc_charger == DC_TYPE_DC_CHARGER) ? ONLINE : OFFLINE;

	if (di->dc_in)
		rk818_bat_set_chrg_param(di, dc_charger);
	else
		rk818_bat_set_chrg_param(di, usb_charger);
}

static int rk818_bat_rtc_sleep_sec(struct rk818_battery *di)
{
	int err;
	int interval_sec = 0;
	struct rtc_time tm;
	struct timespec tv = { .tv_nsec = NSEC_PER_SEC >> 1, };
	struct rtc_device *rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);

	err = rtc_read_time(rtc, &tm);
	if (err) {
		dev_err(rtc->dev.parent, "hctosys: read hardware clk failed\n");
		return 0;
	}

	err = rtc_valid_tm(&tm);
	if (err) {
		dev_err(rtc->dev.parent, "hctosys: invalid date time\n");
		return 0;
	}

	rtc_tm_to_time(&tm, &tv.tv_sec);
	interval_sec = tv.tv_sec - di->rtc_base.tv_sec;

	return (interval_sec > 0) ? interval_sec : 0;
}

static void rk818_bat_init_ts1_detect(struct rk818_battery *di)
{
	u8 buf;

	if (!di->pdata->ntc_size)
		return;

	/* ADC_TS1_EN */
	buf = rk818_bat_read(di, RK818_ADC_CTRL_REG);
	buf |= ADC_TS1_EN;
	rk818_bat_write(di, RK818_ADC_CTRL_REG, buf);
}

static void rk818_bat_init_ts2_detect(struct rk818_battery *di)
{
	u8 buf;

	if (!di->pdata->ts2_vol_multi)
		return;

	/* TS2 adc mode */
	buf = rk818_bat_read(di, RK818_TS_CTRL_REG);
	buf |= TS2_FUN_ADC;
	rk818_bat_write(di, RK818_TS_CTRL_REG, buf);

	/* TS2 adc enable */
	buf = rk818_bat_read(di, RK818_ADC_CTRL_REG);
	buf |= ADC_TS2_EN;
	rk818_bat_write(di, RK818_ADC_CTRL_REG, buf);

	BAT_INFO("eanble ts2 voltage detect, multi=%d\n",
		 di->pdata->ts2_vol_multi);
}

static void rk818_bat_init_fg(struct rk818_battery *di)
{
	rk818_bat_enable_gauge(di);
	rk818_bat_init_voltage_kb(di);
	rk818_bat_init_coffset(di);
	rk818_bat_set_relax_sample(di);
	rk818_bat_set_ioffset_sample(di);
	rk818_bat_set_ocv_sample(di);
	rk818_bat_init_ts1_detect(di);
	rk818_bat_init_ts2_detect(di);
	rk818_bat_init_rsoc(di);
	rk818_bat_init_coulomb_cap(di, di->nac);
	rk818_bat_init_age_algorithm(di);
	rk818_bat_init_chrg_config(di);
	rk818_bat_init_zero_table(di);
	rk818_bat_init_caltimer(di);
	rk818_bat_init_dsoc_algorithm(di);

	di->voltage_avg = rk818_bat_get_avg_voltage(di);
	di->voltage_ocv = rk818_bat_get_ocv_voltage(di);
	di->voltage_relax = rk818_bat_get_relax_voltage(di);
	di->current_avg = rk818_bat_get_avg_current(di);
	di->remain_cap = rk818_bat_get_coulomb_cap(di);
	di->dbg_pwr_dsoc = di->dsoc;
	di->dbg_pwr_rsoc = di->rsoc;
	di->dbg_pwr_vol = di->voltage_avg;

	rk818_bat_dump_regs(di, 0x99, 0xee);
	DBG("nac=%d cap=%d ov=%d v=%d rv=%d dl=%d rl=%d c=%d\n",
	    di->nac, di->remain_cap, di->voltage_ocv, di->voltage_avg,
	    di->voltage_relax, di->dsoc, di->rsoc, di->current_avg);
}

#ifdef CONFIG_OF
static int rk818_bat_parse_dt(struct rk818_battery *di)
{
	u32 out_value;
	int length, ret;
	size_t size;
	struct device_node *np;
	struct battery_platform_data *pdata;
	struct device *dev = di->dev;
	enum of_gpio_flags flags;

	np = of_find_node_by_name(di->rk818->dev->of_node, "battery");
	if (!np) {
		dev_err(dev, "battery node not found!\n");
		return -ENODEV;
	}

	pdata = devm_kzalloc(di->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	di->pdata = pdata;
	/* init default param */
	pdata->bat_res = DEFAULT_BAT_RES;
	pdata->monitor_sec = DEFAULT_MONITOR_SEC;
	pdata->pwroff_vol = DEFAULT_PWROFF_VOL_THRESD;
	pdata->sleep_exit_current = DEFAULT_SLP_EXIT_CUR;
	pdata->sleep_enter_current = DEFAULT_SLP_ENTER_CUR;
	pdata->bat_mode = MODE_BATTARY;
	pdata->max_soc_offset = DEFAULT_MAX_SOC_OFFSET;
	pdata->fb_temp = DEFAULT_FB_TEMP;
	pdata->energy_mode = DEFAULT_ENERGY_MODE;
	pdata->zero_reserve_dsoc = DEFAULT_ZERO_RESERVE_DSOC;
	pdata->ts2_vol_multi = DEFAULT_TS2_VOL_MULTI;

	/* parse necessary param */
	if (!of_find_property(np, "ocv_table", &length)) {
		dev_err(dev, "ocv_table not found!\n");
		return -EINVAL;
	}

	pdata->ocv_size = length / sizeof(u32);
	if (pdata->ocv_size <= 0) {
		dev_err(dev, "invalid ocv table\n");
		return -EINVAL;
	}

	size = sizeof(*pdata->ocv_table) * pdata->ocv_size;
	pdata->ocv_table = devm_kzalloc(di->dev, size, GFP_KERNEL);
	if (!pdata->ocv_table)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "ocv_table", pdata->ocv_table,
					 pdata->ocv_size);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(np, "design_capacity", &out_value);
	if (ret < 0) {
		dev_err(dev, "design_capacity not found!\n");
		return ret;
	}
	pdata->design_capacity = out_value;

	ret = of_property_read_u32(np, "design_qmax", &out_value);
	if (ret < 0) {
		dev_err(dev, "design_qmax not found!\n");
		return ret;
	}
	pdata->design_qmax = out_value;

	ret = of_property_read_u32(np, "max_chrg_current", &out_value);
	if (ret < 0) {
		ret = of_property_read_u32(np, "max_chrg_currentmA",
					   &out_value);
		if (ret < 0) {
			dev_err(dev, "max_chrg_current missing!\n");
			return ret;
		}
	}
	pdata->max_chrg_current = out_value;

	ret = of_property_read_u32(np, "max_input_current", &out_value);
	if (ret < 0) {
		ret = of_property_read_u32(np, "max_input_currentmA",
					   &out_value);
		if (ret < 0) {
			dev_err(dev, "max_input_current missing!\n");
			return ret;
		}
	}
	pdata->max_input_current = out_value;

	ret = of_property_read_u32(np, "max_chrg_voltage", &out_value);
	if (ret < 0) {
		ret = of_property_read_u32(np, "max_charge_voltagemV",
					   &out_value);
		if (ret < 0) {
			dev_err(dev, "max_chrg_voltage missing!\n");
			return ret;
		}
	}
	pdata->max_chrg_voltage = out_value;
	if (out_value >= 4300)
		pdata->zero_algorithm_vol = DEFAULT_ALGR_VOL_THRESD2;
	else
		pdata->zero_algorithm_vol = DEFAULT_ALGR_VOL_THRESD1;

	/* parse unnecessary param */
	if (!of_find_property(np, "lp_input_current", &length)) {
		pdata->lp_input_current = 0;
	} else {
		of_property_read_u32_index(np, "lp_input_current", 0,
					   &pdata->lp_input_current);
		of_property_read_u32_index(np, "lp_input_current", 1,
					   &pdata->lp_soc_min);
		of_property_read_u32_index(np, "lp_input_current", 2,
					   &pdata->lp_soc_max);
		if (pdata->lp_soc_max <= pdata->lp_soc_min) {
			dev_err(dev, "lp input current set min max error\n");
			pdata->lp_input_current = 0;
		}
	}

	ret = of_property_read_u32(np, "fb_temperature", &pdata->fb_temp);
	if (ret < 0)
		dev_err(dev, "fb_temperature missing!\n");

	ret = of_property_read_u32(np, "energy_mode",
				   &pdata->energy_mode);
	if (ret < 0)
		dev_err(dev, "energy_mode missing!\n");

	ret = of_property_read_u32(np, "max_soc_offset",
				   &pdata->max_soc_offset);
	if (ret < 0)
		dev_err(dev, "max_soc_offset missing!\n");

	ret = of_property_read_u32(np, "monitor_sec", &pdata->monitor_sec);
	if (ret < 0)
		dev_err(dev, "monitor_sec missing!\n");

	ret = of_property_read_u32(np, "zero_algorithm_vol",
				   &pdata->zero_algorithm_vol);
	if (ret < 0)
		dev_err(dev, "zero_algorithm_vol missing!\n");

	ret = of_property_read_u32(np, "zero_reserve_dsoc",
				   &pdata->zero_reserve_dsoc);

	ret = of_property_read_u32(np, "virtual_power", &pdata->bat_mode);
	if (ret < 0)
		dev_err(dev, "virtual_power missing!\n");

	ret = of_property_read_u32(np, "power_dc2otg", &pdata->power_dc2otg);
	if (ret < 0)
		dev_err(dev, "power_dc2otg missing!\n");

	ret = of_property_read_u32(np, "bat_res", &pdata->bat_res);
	if (ret < 0)
		dev_err(dev, "bat_res missing!\n");

	ret = of_property_read_u32(np, "sleep_enter_current",
				   &pdata->sleep_enter_current);
	if (ret < 0)
		dev_err(dev, "sleep_enter_current missing!\n");

	ret = of_property_read_u32(np, "sleep_exit_current",
				   &pdata->sleep_exit_current);
	if (ret < 0)
		dev_err(dev, "sleep_exit_current missing!\n");

	ret = of_property_read_u32(np, "power_off_thresd", &pdata->pwroff_vol);
	if (ret < 0)
		dev_err(dev, "power_off_thresd missing!\n");

	ret = of_property_read_u32(np, "ts2_vol_multi",
				   &pdata->ts2_vol_multi);

	if (!of_find_property(np, "dc_det_gpio", &length)) {
		BAT_INFO("not support dc\n");
		pdata->dc_det_pin = -1;
	} else {
		BAT_INFO("support dc\n");
		pdata->dc_det_pin = of_get_named_gpio_flags(np, "dc_det_gpio",
							    0, &flags);
		if (gpio_is_valid(pdata->dc_det_pin))
			pdata->dc_det_level =
					(flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}

	if (!of_find_property(np, "ntc_table", &length)) {
		pdata->ntc_size = 0;
	} else {
		/* get ntc degree base value */
		ret = of_property_read_u32_index(np, "ntc_degree_from", 1,
						 &pdata->ntc_degree_from);
		if (ret) {
			dev_err(dev, "invalid ntc_degree_from\n");
			return -EINVAL;
		}

		of_property_read_u32_index(np, "ntc_degree_from", 0,
					   &out_value);
		if (out_value)
			pdata->ntc_degree_from = -pdata->ntc_degree_from;

		pdata->ntc_size = length / sizeof(u32);
	}

	if (pdata->ntc_size) {
		size = sizeof(*pdata->ntc_table) * pdata->ntc_size;
		pdata->ntc_table = devm_kzalloc(di->dev, size, GFP_KERNEL);
		if (!pdata->ntc_table)
			return -ENOMEM;

		ret = of_property_read_u32_array(np, "ntc_table",
						 pdata->ntc_table,
						 pdata->ntc_size);
		if (ret < 0)
			return ret;
	}

	DBG("the battery dts info dump:\n"
	    "bat_res:%d\n"
	    "max_input_currentmA:%d\n"
	    "max_chrg_current:%d\n"
	    "max_chrg_voltage:%d\n"
	    "design_capacity:%d\n"
	    "design_qmax :%d\n"
	    "sleep_enter_current:%d\n"
	    "sleep_exit_current:%d\n"
	    "zero_algorithm_vol:%d\n"
	    "zero_reserve_dsoc:%d\n"
	    "monitor_sec:%d\n"
	    "power_dc2otg:%d\n"
	    "max_soc_offset:%d\n"
	    "virtual_power:%d\n"
	    "pwroff_vol:%d\n"
	    "ts2_vol_multi:%d\n"
	    "ntc_size=%d\n"
	    "ntc_degree_from:%d\n"
	    "ntc_degree_to:%d\n",
	    pdata->bat_res, pdata->max_input_current,
	    pdata->max_chrg_current, pdata->max_chrg_voltage,
	    pdata->design_capacity, pdata->design_qmax,
	    pdata->sleep_enter_current, pdata->sleep_exit_current,
	    pdata->zero_algorithm_vol, pdata->zero_reserve_dsoc,
	    pdata->monitor_sec, pdata->power_dc2otg,
	    pdata->max_soc_offset, pdata->bat_mode, pdata->pwroff_vol,
	    pdata->ts2_vol_multi, pdata->ntc_size, pdata->ntc_degree_from,
	    pdata->ntc_degree_from + pdata->ntc_size - 1
	    );

	return 0;
}
#else
static int rk818_bat_parse_dt(struct rk818_battery *di)
{
	return -ENODEV;
}
#endif

static int rk818_battery_probe(struct platform_device *pdev)
{
	struct rk818_battery *di;
	struct rk818 *rk818 = dev_get_drvdata(pdev->dev.parent);
	int ret;

	di = devm_kzalloc(&pdev->dev, sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->rk818 = rk818;
	di->pdev = pdev;
	di->dev = &pdev->dev;
	platform_set_drvdata(pdev, di);

	ret = rk818_bat_parse_dt(di);
	if (ret < 0) {
		dev_err(&pdev->dev, "rk818 battery parse dt failed!\n");
		return ret;
	}

	if (!is_rk818_bat_exist(di)) {
		di->pdata->bat_mode = MODE_VIRTUAL;
		dev_err(&pdev->dev, "no battery, virtual power mode\n");
	}

	ret = rk818_bat_init_power_supply(di);
	if (ret) {
		dev_err(&pdev->dev, "rk818 power supply register failed!\n");
		return ret;
	}

	rk818_bat_init_info(di);
	rk818_bat_init_fg(di);
	rk818_bat_init_leds(di);
	rk818_bat_init_charger(di);
	rk818_bat_init_sysfs(di);
	rk818_bat_register_fb_notify(di);
	wake_lock_init(&di->wake_lock, WAKE_LOCK_SUSPEND, "rk818_bat_lock");
	di->bat_monitor_wq = alloc_ordered_workqueue("%s",
			WQ_MEM_RECLAIM | WQ_FREEZABLE, "rk818-bat-monitor-wq");
	INIT_DELAYED_WORK(&di->bat_delay_work, rk818_battery_work);
	INIT_DELAYED_WORK(&di->irq_delay_work, rk818_bat_irq_delay_work);

	ret = rk818_bat_init_irqs(di);
	if (ret) {
		dev_err(&pdev->dev, "rk818 bat irq init failed!\n");
		goto irq_fail;
	}

	queue_delayed_work(di->bat_monitor_wq, &di->bat_delay_work,
			   msecs_to_jiffies(TIMER_MS_COUNTS * 5));

	BAT_INFO("driver version %s\n", DRIVER_VERSION);

	return 0;

irq_fail:
	cancel_delayed_work(&di->dc_delay_work);
	cancel_delayed_work(&di->bc_delay_work);
	cancel_delayed_work(&di->bat_delay_work);
	cancel_delayed_work(&di->calib_delay_work);
	cancel_delayed_work(&di->irq_delay_work);
	cancel_delayed_work(&di->term_mode_work);
	cancel_delayed_work(&di->ts2_vol_work);
	destroy_workqueue(di->bat_monitor_wq);
	destroy_workqueue(di->charger_wq);
	destroy_workqueue(di->ts2_wq);
	rk818_bat_unregister_fb_notify(di);
	rk_bc_detect_notifier_unregister(&di->bc_detect_nb);
	del_timer(&di->caltimer);
	wake_lock_destroy(&di->wake_lock);
	power_supply_unregister(&di->ac);
	power_supply_unregister(&di->usb);
	power_supply_unregister(&di->bat);

	return ret;
}

static int rk818_battery_suspend(struct platform_device *dev,
				 pm_message_t state)
{
	struct rk818_battery *di = platform_get_drvdata(dev);
	u8 st;

	cancel_delayed_work_sync(&di->bat_delay_work);
	/* when otg and dc both plugin */
	if (di->dc_in && di->otg_in) {
		di->int_sts_msk2 = rk818_bat_read(di, RK818_INT_STS_MSK_REG2);
		rk818_bat_set_bits(di, RK818_INT_STS_MSK_REG2,
			   CHRG_CVTLMT_INT_MSK, CHRG_CVTLMT_INT_MSK);
	}

	di->s2r = false;
	di->sleep_chrg_online = rk818_bat_chrg_online(di);
	di->sleep_chrg_status = rk818_bat_get_chrg_status(di);
	di->current_avg = rk818_bat_get_avg_current(di);
	di->remain_cap = rk818_bat_get_coulomb_cap(di);
	di->rsoc = rk818_bat_get_rsoc(di);
	do_gettimeofday(&di->rtc_base);
	rk818_bat_save_data(di);
	st = (rk818_bat_read(di, RK818_SUP_STS_REG) & CHRG_STATUS_MSK) >> 4;

	/* if not CHARGE_FINISH, reinit chrg_finish_base.
	 * avoid sleep loop in suspend and resume all the time
	 */
	if (di->sleep_chrg_status != CHARGE_FINISH)
		di->chrg_finish_base = get_boot_sec();

	/* avoid: enter suspend from MODE_ZERO: load from heavy to light */
	if ((di->work_mode == MODE_ZERO) &&
	    (di->sleep_chrg_online) && (di->current_avg >= 0)) {
		DBG("suspend: MODE_ZERO exit...\n");
		/* it need't do prepare for mode finish and smooth, it will
		 * be done in display_smooth
		 */
		if (di->sleep_chrg_status == CHARGE_FINISH) {
			di->work_mode = MODE_FINISH;
			di->chrg_finish_base = get_boot_sec();
		} else {
			di->work_mode = MODE_SMOOTH;
			rk818_bat_smooth_algo_prepare(di);
		}
	}

	BAT_INFO("suspend: dl=%d rl=%d c=%d v=%d cap=%d at=%ld ch=%d st=%s\n",
		 di->dsoc, di->rsoc, di->current_avg,
		 rk818_bat_get_avg_voltage(di), rk818_bat_get_coulomb_cap(di),
		 di->sleep_dischrg_sec, di->sleep_chrg_online, bat_status[st]);

	return 0;
}

static int rk818_battery_resume(struct platform_device *dev)
{
	int interval_sec, time_step, pwroff_vol;
	struct rk818_battery *di = platform_get_drvdata(dev);
	u8 st;
	/* when otg and dc both plugin */
	if (di->otg_in && di->dc_in)
		rk818_bat_write(di, RK818_INT_STS_MSK_REG2, di->int_sts_msk2);

	di->s2r = true;
	di->voltage_avg = rk818_bat_get_avg_voltage(di);
	di->current_avg = rk818_bat_get_avg_current(di);
	di->voltage_relax = rk818_bat_get_relax_voltage(di);
	di->remain_cap = rk818_bat_get_coulomb_cap(di);
	di->rsoc = rk818_bat_get_rsoc(di);
	interval_sec = rk818_bat_rtc_sleep_sec(di);
	di->sleep_sum_sec += interval_sec;
	pwroff_vol = di->pdata->pwroff_vol;
	st = (rk818_bat_read(di, RK818_SUP_STS_REG) & CHRG_STATUS_MSK) >> 4;

	if (!di->sleep_chrg_online) {
		/* only add up discharge sleep seconds */
		di->sleep_dischrg_sec += interval_sec;
		if (di->voltage_avg <= pwroff_vol + 50)
			time_step = DISCHRG_TIME_STEP1;
		else
			time_step = DISCHRG_TIME_STEP2;
	}

	BAT_INFO("resume: dl=%d rl=%d c=%d v=%d rv=%d "
		 "cap=%d dt=%d at=%ld ch=%d st=%s\n",
		 di->dsoc, di->rsoc, di->current_avg, di->voltage_avg,
		 di->voltage_relax, rk818_bat_get_coulomb_cap(di), interval_sec,
		 di->sleep_dischrg_sec, di->sleep_chrg_online, bat_status[st]);

	/* sleep: enough time and discharge */
	if ((di->sleep_dischrg_sec > time_step) && (!di->sleep_chrg_online)) {
		if (rk818_bat_sleep_dischrg(di))
			di->sleep_dischrg_sec = 0;
	}

	rk818_bat_save_data(di);
	/* charge/lowpower lock: for battery work to update dsoc and rsoc */
	if ((di->sleep_chrg_online) ||
	    (!di->sleep_chrg_online && di->voltage_avg <= pwroff_vol))
		wake_lock_timeout(&di->wake_lock, msecs_to_jiffies(2000));

	queue_delayed_work(di->bat_monitor_wq, &di->bat_delay_work,
			   msecs_to_jiffies(1000));

	return 0;
}

static void rk818_battery_shutdown(struct platform_device *dev)
{
	u8 cnt = 0;
	struct rk818_battery *di = platform_get_drvdata(dev);

	cancel_delayed_work_sync(&di->term_mode_work);
	cancel_delayed_work_sync(&di->bc_delay_work);
	cancel_delayed_work_sync(&di->dc_delay_work);
	cancel_delayed_work_sync(&di->bat_delay_work);
	cancel_delayed_work_sync(&di->calib_delay_work);
	cancel_delayed_work_sync(&di->irq_delay_work);
	cancel_delayed_work_sync(&di->ts2_vol_work);

	rk818_bat_unregister_fb_notify(di);
	rk_bc_detect_notifier_unregister(&di->bc_detect_nb);
	del_timer(&di->caltimer);
	rk818_bat_set_otg_state(di, USB_OTG_POWER_OFF);
	rk818_chrg_term_mode_set(di, CHRG_TERM_ANA_SIGNAL);

	if (base2sec(di->boot_base) < REBOOT_PERIOD_SEC)
		cnt = rk818_bat_check_reboot(di);
	else
		rk818_bat_save_reboot_cnt(di, 0);

	BAT_INFO("shutdown: dl=%d rl=%d c=%d v=%d cap=%d f=%d ch=%d n=%d "
		 "mode=%d rest=%d\n",
		 di->dsoc, di->rsoc, di->current_avg, di->voltage_avg,
		 di->remain_cap, di->fcc, rk818_bat_chrg_online(di), cnt,
		 di->algo_rest_mode, di->algo_rest_val);
}

static struct platform_driver rk818_battery_driver = {
	.probe = rk818_battery_probe,
	.suspend = rk818_battery_suspend,
	.resume = rk818_battery_resume,
	.shutdown = rk818_battery_shutdown,
	.driver = {
		.name	= "rk818-battery",
		.owner	= THIS_MODULE,
	},
};

static int __init battery_init(void)
{
	return platform_driver_register(&rk818_battery_driver);
}
fs_initcall_sync(battery_init);

static void __exit battery_exit(void)
{
	platform_driver_unregister(&rk818_battery_driver);
}
module_exit(battery_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rk818-battery");
MODULE_AUTHOR("ROCKCHIP");
