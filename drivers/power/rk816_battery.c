/*
 * rk816 battery driver
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
 *
 */
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/mfd/rk816.h>
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
#include "rk816_battery.h"

static int dbg_enable = 0;
module_param_named(dbg_level, dbg_enable, int, 0644);

#define DBG(args...) \
	do { \
		if (dbg_enable) { \
			pr_info(args); \
		} \
	} while (0)

#define BAT_INFO(fmt, args...) pr_info("rk816-bat: "fmt, ##args)

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
#define DEFAULT_CCCV_HOUR_SEL		CHG_CCCV_6HOUR
#define DEFAULT_MAX_SOC_OFFSET		60
#define DEFAULT_FB_TEMP			TEMP_115C

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
#define DSOC_CHRG_FINISH_CUR		1000
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
#define MIN_ZERO_ROUND_ACCURACY		1
#define DEF_PWRPATH_RES			50
#define	WAIT_DSOC_DROP_SEC		15
#define	WAIT_SHTD_DROP_SEC		30
#define MIN_ZERO_GAP_XSOC1		10
#define MIN_ZERO_GAP_XSOC2		5
#define MIN_ZERO_GAP_XSOC3		3

#define ADC_CALIB_THRESHOLD		4
#define ADC_CALIB_LMT_MIN		3

/* time */
#define	POWER_ON_SEC_BASE		1
#define MINUTE				60

/* sleep */
#define SLP_CURR_MAX			40
#define SLP_CURR_MIN			6
#define DISCHRG_TIME_STEP1		(10 * 60)
#define DISCHRG_TIME_STEP2		(60 * 60)
#define SLP_DSOC_VOL_THRESD		3600
#define REBOOT_PERIOD_SEC		180
#define REBOOT_MAX_CNT			80

/* fcc */
#define MAX_FCC				10000
#define MIN_FCC				500
#define SEC_TO_MIN(x)			((x) / 60)
#define TIMER_CALIB_8MIN		480/*seconds*/

/* DC ADC */
#define DC_ADC_TRIGGER			150

struct rk816_battery {
	struct platform_device		*pdev;
	struct rk816			*rk816;
	struct device			*dev;
	struct power_supply		bat;
	struct power_supply		ac;
	struct power_supply		usb;
	struct battery_platform_data	*pdata;
	struct workqueue_struct		*bat_monitor_wq;
	struct workqueue_struct		*dc_monitor_wq;
	struct delayed_work		bat_delay_work;
	struct delayed_work		dc_delay_work;
	struct delayed_work		calib_delay_work;
	struct wake_lock		wake_lock;
	struct notifier_block		bc_detect_nb;
	struct notifier_block           fb_nb;
	struct timer_list		caltimer;
	enum bc_port_type		charge_otg;
	struct timeval			rtc_base;
	struct iio_channel		*iio_chan;
	int				bat_res;
	int				chrg_status;
	bool				is_initialized;
	bool				bat_first_power_on;
	u8				ac_in;
	u8				usb_in;
	u8				otg_in;
	u8				dc_in;
	u8				prop_val;
	int				current_avg;
	int				current_relax;
	int				voltage_avg;
	int				voltage_ocv;
	int				voltage_relax;
	int				voltage_k;/* VCALIB0 VCALIB1 */
	int				voltage_b;
	int				remain_cap;
	int				design_cap;
	int				nac;
	int				fcc;
	int				lock_fcc;
	int				qmax;
	int				dsoc;
	int				rsoc;
	int				poffset;
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
	int				sleep_sum_cap;
	int				sleep_remain_cap;
	unsigned long			sleep_dischrg_sec;
	bool				sleep_chrg_online;
	u8				sleep_chrg_status;
	bool				adc_allow_update;
	int                             fb_blank;
	bool				early_resume;
	bool				s2r; /*suspend to resume*/
	u32				work_mode;
	int				temperature;
	int				chrg_cur_lp_input;
	int				chrg_vol_sel;
	int				chrg_cur_input;
	int				chrg_cur_sel;
	u32				monitor_ms;
	u32				pwroff_min;
	unsigned long			chrg_finish_base;
	unsigned long			boot_base;
	unsigned long			flat_match_sec;
	unsigned long			plug_in_base;
	unsigned long			plug_out_base;
	int				dbg_chrg_min[10];
	int				dbg_meet_soc;
	int				dbg_calc_dsoc;
	int				dbg_calc_rsoc;
	u32				dbg_i2c_rd_err;
	u32				dbg_i2c_wr_err;
};

#define to_bat_device_info(x) container_of((x), struct rk816_battery, bat)
#define to_ac_device_info(x) container_of((x), struct rk816_battery, ac)
#define to_usb_device_info(x) container_of((x), struct rk816_battery, usb)

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

static int DIV(int val)
{
	return val ? val : 1;
}

static int rk816_bat_read(struct rk816_battery *di, u8 reg)
{
	int val;

	val = rk816_reg_read(di->rk816, reg);
	if (val < 0) {
		di->dbg_i2c_rd_err++;
		dev_err(di->dev, "read reg:0x%x failed\n", reg);
	}

	return val;
}

static int rk816_bat_write(struct rk816_battery *di, u8 reg, u8 buf)
{
	int ret;

	ret = rk816_reg_write(di->rk816, reg, buf);
	if (ret < 0) {
		di->dbg_i2c_wr_err++;
		dev_err(di->dev, "write reg:0x%x failed\n", reg);
	}

	return ret;
}

static int rk816_bat_set_bits(struct rk816_battery *di, u8 reg, u8 mask, u8 val)
{
	int ret;

	ret = rk816_set_bits(di->rk816, reg, mask, val);
	if (ret < 0) {
		di->dbg_i2c_wr_err++;
		dev_err(di->dev, "set reg:0x%02x failed\n", reg);
	}

	return ret;
}

static void rk816_bat_dump_regs(struct rk816_battery *di, u8 start, u8 end)
{
	int i;

	if (!dbg_enable)
		return;

	DBG("dump regs from: 0x%x-->0x%x\n", start, end);
	for (i = start; i < end; i++)
		DBG("0x%x: 0x%0x\n", i, rk816_bat_read(di, i));
}

static bool rk816_bat_chrg_online(struct rk816_battery *di)
{
	return (di->usb_in || di->ac_in || di->dc_in) ? true : false;
}

static int rk816_bat_get_coulomb_cap(struct rk816_battery *di)
{
	int cap, val = 0;

	val |= rk816_bat_read(di, RK816_GASCNT_REG3) << 24;
	val |= rk816_bat_read(di, RK816_GASCNT_REG2) << 16;
	val |= rk816_bat_read(di, RK816_GASCNT_REG1) << 8;
	val |= rk816_bat_read(di, RK816_GASCNT_REG0) << 0;
	cap = val / 2390;

	return cap;
}

static int rk816_bat_get_rsoc(struct rk816_battery *di)
{
	return (di->remain_cap + di->fcc / 200) * 100 / DIV(di->fcc);
}

static ssize_t bat_info_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	char cmd;
	struct rk816_battery *di = to_bat_device_info(dev_get_drvdata(dev));

	sscanf(buf, "%c", &cmd);
	if (cmd == 's')
		rk816_set_bits(di->rk816, RK816_MISC_MARK_REG, 0x2, 0x2);
	else if (cmd == 'c')
		rk816_clear_bits(di->rk816, RK816_MISC_MARK_REG, 0x2);
	else if (cmd == 'r')
		BAT_INFO("0x%2x\n", rk816_bat_read(di, RK816_MISC_MARK_REG));
	else
		BAT_INFO("command error\n");

	return count;
}

static struct device_attribute rk816_bat_attr[] = {
	__ATTR(bat, 0664, NULL, bat_info_store),
};

static void rk816_bat_enable_gauge(struct rk816_battery *di)
{
	u8 buf;

	buf = rk816_bat_read(di, RK816_TS_CTRL_REG);
	buf |= GG_EN;
	rk816_bat_write(di, RK816_TS_CTRL_REG, buf);
}

static void rk816_bat_save_age_level(struct rk816_battery *di, u8 level)
{
	rk816_bat_write(di, RK816_UPDAT_LEVE_REG, level);
}

static u8 rk816_bat_get_age_level(struct  rk816_battery *di)
{
	return rk816_bat_read(di, RK816_UPDAT_LEVE_REG);
}

static int rk816_bat_get_vcalib0(struct rk816_battery *di)
{
	int val = 0;

	val |= rk816_bat_read(di, RK816_VCALIB0_REGL) << 0;
	val |= rk816_bat_read(di, RK816_VCALIB0_REGH) << 8;

	DBG("<%s>. voffset0: 0x%x\n", __func__, val);
	return val;
}

static int rk816_bat_get_vcalib1(struct rk816_battery *di)
{
	int val = 0;

	val |= rk816_bat_read(di, RK816_VCALIB1_REGL) << 0;
	val |= rk816_bat_read(di, RK816_VCALIB1_REGH) << 8;

	DBG("<%s>. voffset1: 0x%x\n", __func__, val);
	return val;
}

static int rk816_bat_get_ioffset(struct rk816_battery *di)
{
	int val = 0;

	val |= rk816_bat_read(di, RK816_IOFFSET_REGL) << 0;
	val |= rk816_bat_read(di, RK816_IOFFSET_REGH) << 8;

	DBG("<%s>. ioffset: 0x%x\n", __func__, val);
	return val;
}

static int rk816_bat_get_coffset(struct rk816_battery *di)
{
	int val = 0;

	val |= rk816_bat_read(di, RK816_CAL_OFFSET_REGL) << 0;
	val |= rk816_bat_read(di, RK816_CAL_OFFSET_REGH) << 8;

	DBG("<%s>. coffset: 0x%x\n", __func__, val);
	return val;
}

static void rk816_bat_set_coffset(struct rk816_battery *di, int val)
{
	u8 buf;

	buf = (val >> 8) & 0xff;
	rk816_bat_write(di, RK816_CAL_OFFSET_REGH, buf);
	buf = (val >> 0) & 0xff;
	rk816_bat_write(di, RK816_CAL_OFFSET_REGL, buf);
	DBG("<%s>. coffset: 0x%x\n", __func__, val);
}

static void rk816_bat_init_voltage_kb(struct rk816_battery *di)
{
	int vcalib0, vcalib1;

	vcalib0 = rk816_bat_get_vcalib0(di);
	vcalib1 = rk816_bat_get_vcalib1(di);
	di->voltage_k = (4200 - 3000) * 1000 / DIV(vcalib1 - vcalib0);
	di->voltage_b = 4200 - (di->voltage_k * vcalib1) / 1000;

	DBG("voltage_k=%d(*1000),voltage_b=%d\n", di->voltage_k, di->voltage_b);
}

static int rk816_bat_get_ocv_voltage(struct rk816_battery *di)
{
	int vol, val = 0;

	val |= rk816_bat_read(di, RK816_BAT_OCV_REGL) << 0;
	val |= rk816_bat_read(di, RK816_BAT_OCV_REGH) << 8;
	vol = di->voltage_k * val / 1000 + di->voltage_b;

	return (vol * 1100 / 1000);
}

static int rk816_bat_get_avg_voltage(struct rk816_battery *di)
{
	int vol, val = 0;

	val |= rk816_bat_read(di, RK816_BAT_VOL_REGL) << 0;
	val |= rk816_bat_read(di, RK816_BAT_VOL_REGH) << 8;
	vol = di->voltage_k * val / 1000 + di->voltage_b;

	return (vol * 1100 / 1000);
}

static int rk816_bat_get_usb_voltage(struct rk816_battery *di)
{
	int vol, val = 0;

	val |= rk816_bat_read(di, RK816_USB_ADC_REGL) << 0;
	val |= rk816_bat_read(di, RK816_USB_ADC_REGH) << 8;
	vol = di->voltage_k * val / 1000 + di->voltage_b;

	return (vol * 1400 / 1100);
}

static bool is_rk816_bat_relax_mode(struct rk816_battery *di)
{
	u8 status;

	status = rk816_bat_read(di, RK816_GGSTS_REG);
	if (!(status & RELAX_VOL1_UPD) || !(status & RELAX_VOL2_UPD))
		return false;
	else
		return true;
}

static u16 rk816_bat_get_relax_vol1(struct rk816_battery *di)
{
	u16 vol, val = 0;

	val |= rk816_bat_read(di, RK816_RELAX_VOL1_REGL) << 0;
	val |= rk816_bat_read(di, RK816_RELAX_VOL1_REGH) << 8;
	vol = di->voltage_k * val / 1000 + di->voltage_b;

	return (vol * 1100 / 1000);
}

static u16 rk816_bat_get_relax_vol2(struct rk816_battery *di)
{
	u16 vol, val = 0;

	val |= rk816_bat_read(di, RK816_RELAX_VOL2_REGL) << 0;
	val |= rk816_bat_read(di, RK816_RELAX_VOL2_REGH) << 8;
	vol = di->voltage_k * val / 1000 + di->voltage_b;

	return (vol * 1100 / 1000);
}

static u16 rk816_bat_get_relax_voltage(struct rk816_battery *di)
{
	u16 relax_vol1, relax_vol2;

	if (!is_rk816_bat_relax_mode(di))
		return 0;

	relax_vol1 = rk816_bat_get_relax_vol1(di);
	relax_vol2 = rk816_bat_get_relax_vol2(di);

	return relax_vol1 > relax_vol2 ? relax_vol1 : relax_vol2;
}

static int rk816_bat_get_avg_current(struct rk816_battery *di)
{
	int cur, val = 0;

	val |= rk816_bat_read(di, RK816_BAT_CUR_AVG_REGL) << 0;
	val |= rk816_bat_read(di, RK816_BAT_CUR_AVG_REGH) << 8;
	if (val & 0x800)
		val -= 4096;
	cur = val * 1506 / 1000;

	return cur;
}

static int rk816_bat_get_relax_cur1(struct rk816_battery *di)
{
	int val = 0;

	val |= rk816_bat_read(di, RK816_RELAX_CUR1_REGL) << 0;
	val |= rk816_bat_read(di, RK816_RELAX_CUR1_REGH) << 8;
	if (val & 0x800)
		val -= 4096;

	return (val * 1506 / 1000);
}

static int rk816_bat_get_relax_cur2(struct rk816_battery *di)
{
	int val = 0;

	val |= rk816_bat_read(di, RK816_RELAX_CUR2_REGL) << 0;
	val |= rk816_bat_read(di, RK816_RELAX_CUR2_REGH) << 8;
	if (val & 0x800)
		val -= 4096;

	return (val * 1506 / 1000);
}

static int rk816_bat_get_relax_current(struct rk816_battery *di)
{
	int relax_cur1, relax_cur2;

	if (!is_rk816_bat_relax_mode(di))
		return 0;

	relax_cur1 = rk816_bat_get_relax_cur1(di);
	relax_cur2 = rk816_bat_get_relax_cur2(di);

	return (relax_cur1 < relax_cur2) ? relax_cur1 : relax_cur2;
}

static int rk816_bat_vol_to_ocvsoc(struct rk816_battery *di, int voltage)
{
	u32 *ocv_table, temp;
	int ocv_size, ocv_soc;

	ocv_table = di->pdata->ocv_table;
	ocv_size = di->pdata->ocv_size;
	temp = interpolate(voltage, ocv_table, ocv_size);
	ocv_soc = ab_div_c(temp, MAX_PERCENTAGE, MAX_INTERPOLATE);

	return ocv_soc;
}

static int rk816_bat_vol_to_ocvcap(struct rk816_battery *di, int voltage)
{
	u32 *ocv_table, temp;
	int ocv_size, cap;

	ocv_table = di->pdata->ocv_table;
	ocv_size = di->pdata->ocv_size;
	temp = interpolate(voltage, ocv_table, ocv_size);
	cap = ab_div_c(temp, di->fcc, MAX_INTERPOLATE);

	return cap;
}

static int rk816_bat_vol_to_zerosoc(struct rk816_battery *di, int voltage)
{
	u32 *ocv_table, temp;
	int ocv_size, ocv_soc;

	ocv_table = di->pdata->zero_table;
	ocv_size = di->pdata->ocv_size;
	temp = interpolate(voltage, ocv_table, ocv_size);
	ocv_soc = ab_div_c(temp, MAX_PERCENTAGE, MAX_INTERPOLATE);

	return ocv_soc;
}

static int rk816_bat_vol_to_zerocap(struct rk816_battery *di, int voltage)
{
	u32 *ocv_table, temp;
	int ocv_size, cap;

	ocv_table = di->pdata->zero_table;
	ocv_size = di->pdata->ocv_size;
	temp = interpolate(voltage, ocv_table, ocv_size);
	cap = ab_div_c(temp, di->fcc, MAX_INTERPOLATE);

	return cap;
}

static int rk816_bat_get_iadc(struct rk816_battery *di)
{
	int val = 0;

	val |= rk816_bat_read(di, RK816_BAT_CUR_AVG_REGL) << 0;
	val |= rk816_bat_read(di, RK816_BAT_CUR_AVG_REGH) << 8;
	if (val > 2047)
		val -= 4096;

	return val;
}

static bool is_rk816_bat_cvtlim(struct rk816_battery *di)
{
	return (rk816_bat_read(di, RK816_INT_STS_REG1) & 0x80) ? true : false;
}

static bool rk816_bat_adc_calib(struct rk816_battery *di)
{
	int i, ioffset, coffset, adc, save_coffset;

	if ((di->chrg_status != CHARGE_FINISH) ||
	    (base2min(di->boot_base) < ADC_CALIB_LMT_MIN) ||
	    (abs(di->current_avg) < ADC_CALIB_THRESHOLD) ||
	    (is_rk816_bat_cvtlim(di)))
		return false;

	save_coffset = rk816_bat_get_coffset(di);
	for (i = 0; i < 5; i++) {
		adc = rk816_bat_get_iadc(di);
		if (!rk816_bat_chrg_online(di)) {
			rk816_bat_set_coffset(di, save_coffset);
			BAT_INFO("quit, charger plugout when calib adc\n");
			return false;
		}
		coffset = rk816_bat_get_coffset(di);
		rk816_bat_set_coffset(di, coffset + adc);
		msleep(2000);
		if (is_rk816_bat_cvtlim(di)) {
			rk816_bat_set_coffset(di, save_coffset);
			BAT_INFO("cvtlmt when calib adc\n");
			return false;
		} else {
			adc = rk816_bat_get_iadc(di);
		}
		if (abs(adc) < ADC_CALIB_THRESHOLD) {
			coffset = rk816_bat_get_coffset(di);
			ioffset = rk816_bat_get_ioffset(di);
			di->poffset = coffset - ioffset;
			rk816_bat_write(di, RK816_PCB_IOFFSET_REG, di->poffset);
			BAT_INFO("new offset:c=0x%x, i=0x%x, p=0x%x\n",
				 coffset, ioffset, di->poffset);
			return true;
		} else {
			BAT_INFO("coffset calib again %d..\n", i);
			rk816_bat_set_coffset(di, coffset);
			msleep(2000);
		}
	}

	return false;
}

static void rk816_bat_set_ioffset_sample(struct rk816_battery *di)
{
	u8 ggcon;

	ggcon = rk816_bat_read(di, RK816_GGCON_REG);
	ggcon &= ~(0x30);
	ggcon |= ADC_SAMP_8MIN;
	rk816_bat_write(di, RK816_GGCON_REG, ggcon);
}

static void rk816_bat_set_ocv_sample(struct rk816_battery *di)
{
	u8 ggcon;

	ggcon = rk816_bat_read(di, RK816_GGCON_REG);
	ggcon &= ~0x0c;
	ggcon |= OCV_SAMP_8MIN;
	rk816_bat_write(di, RK816_GGCON_REG, ggcon);
}

static void rk816_bat_restart_relax(struct rk816_battery *di)
{
	u8 ggsts;

	ggsts = rk816_bat_read(di, RK816_GGSTS_REG);
	ggsts &= ~0x0c;
	rk816_bat_write(di, RK816_GGSTS_REG, ggsts);
}

static void rk816_bat_set_relax_sample(struct rk816_battery *di)
{
	u8 buf;
	int enter_thres, exit_thres, filter_thres;
	struct battery_platform_data *pdata = di->pdata;

	enter_thres = pdata->sleep_enter_current * 1000 / 1506;
	exit_thres = pdata->sleep_exit_current * 1000 / 1506;
	filter_thres = pdata->sleep_filter_current * 1000 / 1506;

	/* set relax enter and exit threshold */
	buf = enter_thres & 0xff;
	rk816_bat_write(di, RK816_RELAX_ENTRY_THRES_REGL, buf);
	buf = (enter_thres >> 8) & 0xff;
	rk816_bat_write(di, RK816_RELAX_ENTRY_THRES_REGH, buf);

	buf = exit_thres & 0xff;
	rk816_bat_write(di, RK816_RELAX_EXIT_THRES_REGL, buf);
	buf = (exit_thres >> 8) & 0xff;
	rk816_bat_write(di, RK816_RELAX_EXIT_THRES_REGH, buf);

	/* set sample current threshold */
	buf = filter_thres & 0xff;
	rk816_bat_write(di, RK816_SLEEP_CON_SAMP_CUR_REG, buf);

	/* reset relax update state */
	rk816_bat_restart_relax(di);
	DBG("<%s>. sleep_enter_current = %d, sleep_exit_current = %d\n",
	    __func__, pdata->sleep_enter_current, pdata->sleep_exit_current);
}

/* high load: current < 0 with charger in.
 * System will not shutdown while dsoc=0% with charging state(ac_in),
 * which will cause over discharge, so oppose status before report states.
 */
static void rk816_bat_lowpwr_check(struct rk816_battery *di)
{
	static u64 time;
	int pwr_off_thresd = di->pdata->pwroff_vol;

	if (di->current_avg < 0 && di->voltage_avg < pwr_off_thresd) {
		if (!time)
			time = get_boot_sec();

		if ((base2sec(time) > MINUTE) ||
		    (di->voltage_avg <= pwr_off_thresd - 50)) {
			/* report fake: none charger */
			di->dc_in = OFFLINE;
			di->usb_in = OFFLINE;
			di->ac_in = OFFLINE;
			di->prop_val = POWER_SUPPLY_STATUS_DISCHARGING;
			if (di->voltage_avg <= pwr_off_thresd - 50)
				di->dsoc--;
			BAT_INFO("low power....\n");
		}
	} else {
		time = 0;
	}
}

static bool is_rk816_bat_exist(struct rk816_battery *di)
{
	return (rk816_bat_read(di, RK816_SUP_STS_REG) & 0x80) ? true : false;
}

static bool is_rk816_bat_first_pwron(struct rk816_battery *di)
{
	u8 buf;

	buf = rk816_bat_read(di, RK816_GGSTS_REG);
	if (buf & BAT_CON) {
		buf &= ~(BAT_CON);
		rk816_bat_write(di, RK816_GGSTS_REG, buf);
		return true;
	}

	return false;
}

static u8 rk816_bat_get_pwroff_min(struct rk816_battery *di)
{
	return rk816_bat_read(di, RK816_NON_ACT_TIMER_CNT_REG);
}

static u8 is_rk816_bat_initialized(struct rk816_battery *di)
{
	u8 val = rk816_bat_read(di, RK816_MISC_MARK_REG);

	if (val & 0x08) {
		val &= ~0x08;
		rk816_bat_write(di, RK816_MISC_MARK_REG, val);
		return true;
	} else {
		return false;
	}
}

static bool is_rk816_bat_ocv_valid(struct rk816_battery *di)
{
	return (!di->is_initialized && di->pwroff_min >= 30) ? true : false;
}

static void rk816_bat_init_age_algorithm(struct rk816_battery *di)
{
	int age_level, ocv_soc, ocv_cap, ocv_vol;

	if (di->bat_first_power_on || is_rk816_bat_ocv_valid(di)) {
		DBG("<%s> enter.\n", __func__);
		ocv_vol = rk816_bat_get_ocv_voltage(di);
		ocv_soc = rk816_bat_vol_to_ocvsoc(di, ocv_vol);
		ocv_cap = rk816_bat_vol_to_ocvcap(di, ocv_vol);
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

			age_level = rk816_bat_get_age_level(di);
			if (age_level > di->age_level) {
				di->age_allow_update = false;
				age_level -= 5;
				if (age_level <= 80)
					age_level = 80;
				rk816_bat_save_age_level(di, age_level);
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

static enum power_supply_property rk816_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int rk816_battery_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct rk816_battery *di = to_bat_device_info(psy);

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
		val->intval = is_rk816_bat_exist(di);
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

static enum power_supply_property rk816_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property rk816_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int rk816_bat_ac_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	int ret = 0;
	struct rk816_battery *di = to_ac_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (rk816_bat_chrg_online(di))
			rk816_bat_lowpwr_check(di);
		val->intval = di->ac_in | di->dc_in;
		if (di->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_AC_ONLINE;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rk816_bat_usb_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	int ret = 0;
	struct rk816_battery *di = to_usb_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (rk816_bat_chrg_online(di))
			rk816_bat_lowpwr_check(di);
		val->intval = di->usb_in;
		if (di->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_USB_ONLINE;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rk816_bat_init_power_supply(struct rk816_battery *di)
{
	int ret;

	di->bat.name = "battery";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = rk816_bat_props;
	di->bat.num_properties = ARRAY_SIZE(rk816_bat_props);
	di->bat.get_property = rk816_battery_get_property;

	di->ac.name = "ac";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.properties = rk816_ac_props;
	di->ac.num_properties = ARRAY_SIZE(rk816_ac_props);
	di->ac.get_property = rk816_bat_ac_get_property;

	di->usb.name = "usb";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = rk816_usb_props;
	di->usb.num_properties = ARRAY_SIZE(rk816_usb_props);
	di->usb.get_property = rk816_bat_usb_get_property;

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

static void rk816_bat_save_cap(struct rk816_battery *di, int capacity)
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
	rk816_bat_write(di, RK816_REMAIN_CAP_REG3, buf);
	buf = (capacity >> 16) & 0xff;
	rk816_bat_write(di, RK816_REMAIN_CAP_REG2, buf);
	buf = (capacity >> 8) & 0xff;
	rk816_bat_write(di, RK816_REMAIN_CAP_REG1, buf);
	buf = (capacity >> 0) & 0xff;
	rk816_bat_write(di, RK816_REMAIN_CAP_REG0, buf);
}

static int rk816_bat_get_prev_cap(struct rk816_battery *di)
{
	int val = 0;

	val |= rk816_bat_read(di, RK816_REMAIN_CAP_REG3) << 24;
	val |= rk816_bat_read(di, RK816_REMAIN_CAP_REG2) << 16;
	val |= rk816_bat_read(di, RK816_REMAIN_CAP_REG1) << 8;
	val |= rk816_bat_read(di, RK816_REMAIN_CAP_REG0) << 0;

	return val;
}

static void rk816_bat_save_fcc(struct rk816_battery *di, u32 fcc)
{
	u8 buf;

	buf = (fcc >> 24) & 0xff;
	rk816_bat_write(di, RK816_NEW_FCC_REG3, buf);
	buf = (fcc >> 16) & 0xff;
	rk816_bat_write(di, RK816_NEW_FCC_REG2, buf);
	buf = (fcc >> 8) & 0xff;
	rk816_bat_write(di, RK816_NEW_FCC_REG1, buf);
	buf = (fcc >> 0) & 0xff;
	rk816_bat_write(di, RK816_NEW_FCC_REG0, buf);

	BAT_INFO("save fcc: %d\n", fcc);
}

static int rk816_bat_get_fcc(struct rk816_battery *di)
{
	u32 fcc = 0;

	fcc |= rk816_bat_read(di, RK816_NEW_FCC_REG3) << 24;
	fcc |= rk816_bat_read(di, RK816_NEW_FCC_REG2) << 16;
	fcc |= rk816_bat_read(di, RK816_NEW_FCC_REG1) << 8;
	fcc |= rk816_bat_read(di, RK816_NEW_FCC_REG0) << 0;

	if (fcc < MIN_FCC) {
		BAT_INFO("invalid fcc(%d), use design cap", fcc);
		fcc = di->pdata->design_capacity;
		rk816_bat_save_fcc(di, fcc);
	} else if (fcc > di->pdata->design_qmax) {
		BAT_INFO("invalid fcc(%d), use qmax", fcc);
		fcc = di->pdata->design_qmax;
		rk816_bat_save_fcc(di, fcc);
	}

	return fcc;
}

static int rk816_bat_get_lock_fcc(struct rk816_battery *di)
{
	u8 reg;
	int fcc, val = 0;

	/* check lock flag, 1: yes, 0: no */
	reg = rk816_bat_read(di, RK816_GGSTS_REG);
	if ((reg & 0x20) == 0)
		return 0;

	val |= rk816_bat_read(di, RK816_FCC_GASCNT_REG3) << 24;
	val |= rk816_bat_read(di, RK816_FCC_GASCNT_REG2) << 16;
	val |= rk816_bat_read(di, RK816_FCC_GASCNT_REG1) << 8;
	val |= rk816_bat_read(di, RK816_FCC_GASCNT_REG0) << 0;
	fcc = val / 2390;

	/* clear lock flag */
	reg &= ~0x20;
	rk816_bat_write(di, RK816_GGSTS_REG, reg);
	BAT_INFO("lock fcc = %d\n", fcc);

	return fcc;
}

static void rk816_bat_save_dsoc(struct rk816_battery *di, u8 save_soc)
{
	static int last_soc = -1;

	if (last_soc != save_soc) {
		rk816_bat_write(di, RK816_SOC_REG, save_soc);
		last_soc = save_soc;
	}
}

static int rk816_bat_get_prev_dsoc(struct rk816_battery *di)
{
	return rk816_bat_read(di, RK816_SOC_REG);
}

static void rk816_bat_save_reboot_cnt(struct rk816_battery *di, u8 save_cnt)
{
	rk816_bat_write(di, RK816_REBOOT_CNT_REG, save_cnt);
}

static void rk816_bat_set_current(struct rk816_battery *di, int charge_current)
{
	u8 usb_ctrl;

	usb_ctrl = rk816_bat_read(di, RK816_USB_CTRL_REG);
	usb_ctrl &= (~0x0f);
	usb_ctrl |= (charge_current);
	rk816_bat_write(di, RK816_USB_CTRL_REG, usb_ctrl);
}

static void rk816_bat_set_chrg_param(struct rk816_battery *di,
				     enum charger_type charger_type)
{
	u8 buf;

	switch (charger_type) {
	case NONE_CHARGER:
		di->usb_in = OFFLINE;
		di->ac_in = OFFLINE;
		di->dc_in = OFFLINE;
		di->prop_val = POWER_SUPPLY_STATUS_DISCHARGING;
		rk816_bat_set_current(di, INPUT_CUR450MA);
		power_supply_changed(&di->bat);
		power_supply_changed(&di->usb);
		power_supply_changed(&di->ac);
		break;
	case NO_ACUSB_CHARGER:
		di->usb_in = OFFLINE;
		di->ac_in = OFFLINE;
		if (di->dc_in == OFFLINE) {
			di->prop_val = POWER_SUPPLY_STATUS_DISCHARGING;
			rk816_bat_set_current(di, INPUT_CUR450MA);
		}
		power_supply_changed(&di->usb);
		power_supply_changed(&di->ac);
		break;
	case USB_CHARGER:
		di->usb_in = ONLINE;
		di->ac_in = OFFLINE;
		di->prop_val = POWER_SUPPLY_STATUS_CHARGING;
		if (di->dc_in == OFFLINE)
			rk816_bat_set_current(di, INPUT_CUR450MA);
		power_supply_changed(&di->usb);
		break;
	case AC_CHARGER:
		di->ac_in = ONLINE;
		di->usb_in = OFFLINE;
		di->prop_val = POWER_SUPPLY_STATUS_CHARGING;
		if (di->pdata->lp_input_current &&
		    di->dsoc >= di->pdata->lp_soc_min &&
		    di->dsoc <= di->pdata->lp_soc_max)
			rk816_bat_set_current(di, di->chrg_cur_lp_input);
		else
			rk816_bat_set_current(di, di->chrg_cur_input);
		power_supply_changed(&di->ac);
		break;
	case DC_CHARGER:
		di->dc_in = ONLINE;
		di->prop_val = POWER_SUPPLY_STATUS_CHARGING;
		if (di->pdata->lp_input_current &&
		    di->dsoc >= di->pdata->lp_soc_min &&
		    di->dsoc <= di->pdata->lp_soc_max)
			rk816_bat_set_current(di, di->chrg_cur_lp_input);
		else
			rk816_bat_set_current(di, di->chrg_cur_input);
		power_supply_changed(&di->ac);
		break;
	case NO_DC_CHARGER:
		di->dc_in = OFFLINE;
		/*
		 * check by pmic int avoid usb error notify:
		 * when plug in dc, usb may error notify usb/ac plug in,
		 * while dc plug out, the "ac/usb_in" still hold
		 */
		buf = rk816_bat_read(di, RK816_VB_MON_REG);
		if ((buf & PLUG_IN_STS) == 0) {
			di->ac_in = OFFLINE;
			di->usb_in = OFFLINE;
			di->prop_val = POWER_SUPPLY_STATUS_DISCHARGING;
			rk816_bat_set_current(di, INPUT_CUR450MA);
		} else if (di->usb_in) {
			rk816_bat_set_current(di, INPUT_CUR450MA);
			di->prop_val = POWER_SUPPLY_STATUS_CHARGING;
		}
		power_supply_changed(&di->usb);
		power_supply_changed(&di->ac);
		break;
	default:
		di->prop_val = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	}

	if (di->dsoc == 100 && rk816_bat_chrg_online(di))
		di->prop_val = POWER_SUPPLY_STATUS_FULL;
}

static void rk816_bat_set_otg_state(struct rk816_battery *di, int state)
{
	switch (state) {
	case USB_OTG_POWER_ON:
		rk816_bat_set_bits(di, RK816_DCDC_EN_REG2,
				   OTG_EN_ON_MASK, OTG_EN_ON_MASK);
		break;
	case USB_OTG_POWER_OFF:
		rk816_bat_set_bits(di, RK816_DCDC_EN_REG2,
				   OTG_EN_ON_MASK, OTG_EN_OFF_MASK);
		break;
	default:
		break;
	}
}

static enum charger_type rk816_bat_get_adc_dc_state(struct rk816_battery *di)
{
	int val = 0;

	if (!di->iio_chan) {
		di->iio_chan = iio_channel_get(di->rk816->dev, NULL);
		if (IS_ERR(di->iio_chan)) {
			di->iio_chan = NULL;
			return NO_DC_CHARGER;
		}
	}

	if (iio_read_channel_raw(di->iio_chan, &val) < 0) {
		pr_err("read channel error\n");
		return NO_DC_CHARGER;
	}

	return (val >= DC_ADC_TRIGGER) ? DC_CHARGER : NO_DC_CHARGER;
}

static enum charger_type rk816_bat_get_gpio_dc_state(struct rk816_battery *di)
{
	int level;

	if (!gpio_is_valid(di->pdata->dc_det_pin))
		return NO_DC_CHARGER;

	level = gpio_get_value(di->pdata->dc_det_pin);

	return (level == di->pdata->dc_det_level) ? DC_CHARGER : NO_DC_CHARGER;
}

static enum charger_type rk816_bat_get_dc_state(struct rk816_battery *di)
{
	enum charger_type type;

	if (di->pdata->bat_mode == MODE_VIRTUAL)
		return DC_CHARGER;

	if (di->pdata->dc_det_adc)
		type = rk816_bat_get_adc_dc_state(di);
	else
		type = rk816_bat_get_gpio_dc_state(di);

	return type;
}

static void rk816_bat_dc_delay_work(struct work_struct *work)
{
	enum charger_type type;
	static enum charger_type old_type = UNKNOWN_CHARGER;
	struct rk816_battery *di = container_of(work,
				struct rk816_battery, dc_delay_work.work);

	type = rk816_bat_get_dc_state(di);
	if (old_type == type)
		goto out;

	old_type = type;
	if (type == DC_CHARGER) {
		BAT_INFO("detect dc charger in..\n");
		rk816_bat_set_chrg_param(di, DC_CHARGER);
		/* check otg supply */
		if (di->otg_in && di->pdata->power_dc2otg) {
			BAT_INFO("otg power from dc adapter\n");
			rk816_bat_set_otg_state(di, USB_OTG_POWER_OFF);
		}
	} else {
		BAT_INFO("detect dc charger out..\n");
		rk816_bat_set_chrg_param(di, NO_DC_CHARGER);
		/* check otg supply, power on anyway */
		if (di->otg_in) {
			BAT_INFO("charge disable, enable otg\n");
			rk816_bat_set_otg_state(di, USB_OTG_POWER_ON);
		}
	}
out:
	/* adc need check all the time */
	if (di->pdata->dc_det_adc)
		queue_delayed_work(di->dc_monitor_wq,
				   &di->dc_delay_work,
				   msecs_to_jiffies(1000));
}

static enum charger_type rk816_bat_get_acusb_state(struct rk816_battery *di)
{
	u8 buf;
	int usb_id;
	enum charger_type charger;

	usb_id = dwc_otg_check_dpdm(0);
	switch (usb_id) {
	case 0:
		buf = rk816_bat_read(di, RK816_VB_MON_REG);
		if ((buf & PLUG_IN_STS) != 0)
			charger = AC_CHARGER;
		else
			charger = NO_ACUSB_CHARGER;
		break;
	case 1:
	case 3:
		charger = USB_CHARGER;
		break;
	case 2:
		charger = AC_CHARGER;
		break;
	default:
		charger = NO_ACUSB_CHARGER;
		break;
	}

	return charger;
}

static int rk816_bat_fb_notifier(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	struct rk816_battery *di;
	struct fb_event *evdata = data;
	int blank;

	di = container_of(nb, struct rk816_battery, fb_nb);

	if (event != FB_EVENT_BLANK && event != FB_EVENT_CONBLANK)
		return 0;

	blank = *(int *)evdata->data;
	if (di->fb_blank != blank)
		di->fb_blank = blank;
	else
		return 0;

	if (blank == FB_BLANK_UNBLANK)
		di->early_resume = true;

	return 0;
}

static int rk816_bat_register_fb_notify(struct rk816_battery *di)
{
	memset(&di->fb_nb, 0, sizeof(di->fb_nb));
	di->fb_nb.notifier_call = rk816_bat_fb_notifier;

	return fb_register_client(&di->fb_nb);
}

static int rk816_bat_unregister_fb_notify(struct rk816_battery *di)
{
	return fb_unregister_client(&di->fb_nb);
}

static void rk816_bat_first_pwron(struct rk816_battery *di)
{
	int ocv_vol;

	rk816_bat_save_fcc(di, di->design_cap);
	ocv_vol = rk816_bat_get_ocv_voltage(di);
	di->fcc = rk816_bat_get_fcc(di);
	di->nac = rk816_bat_vol_to_ocvcap(di, ocv_vol);
	di->rsoc = rk816_bat_vol_to_ocvsoc(di, ocv_vol);
	di->dsoc = di->rsoc;

	BAT_INFO("first on: soc=%d, cap=%d, fcc=%d, ov=%d\n",
		 di->dsoc, di->nac, di->fcc, ocv_vol);
}

static void rk816_bat_not_first_pwron(struct rk816_battery *di)
{
	int pre_soc, pre_cap, ocv_cap, ocv_soc, ocv_vol;

	pre_soc = rk816_bat_get_prev_dsoc(di);
	pre_cap = rk816_bat_get_prev_cap(di);

	if (is_rk816_bat_initialized(di)) {
		di->is_initialized = true;
		BAT_INFO("initialized yet..\n");
		goto finish;
	}

	if (is_rk816_bat_ocv_valid(di)) {
		ocv_vol = rk816_bat_get_ocv_voltage(di);
		ocv_soc = rk816_bat_vol_to_ocvsoc(di, ocv_vol);
		ocv_cap = rk816_bat_vol_to_ocvcap(di, ocv_vol);
		pre_cap = ocv_cap;
		if (abs(ocv_soc - pre_soc) >= di->pdata->max_soc_offset) {
			BAT_INFO("trigger max soc offset, dsoc: %d -> %d\n",
				 pre_soc, ocv_soc);
			pre_soc = ocv_soc;
		}
		BAT_INFO("OCV calib: cap=%d, rsoc=%d\n", ocv_cap, ocv_soc);
	}

finish:
	di->dsoc = pre_soc;
	di->nac = pre_cap;
	if (di->nac < 0)
		di->nac = 0;

	BAT_INFO("dsoc=%d cap=%d v=%d ov=%d min=%d psoc=%d pcap=%d\n",
		 di->dsoc, di->nac, rk816_bat_get_avg_voltage(di),
		 rk816_bat_get_ocv_voltage(di), di->pwroff_min,
		 rk816_bat_get_prev_dsoc(di), rk816_bat_get_prev_cap(di));
}

static bool rk816_bat_ocv_sw_reset(struct rk816_battery *di)
{
	u8 buf, sft_calib;

	buf = rk816_bat_read(di, RK816_MISC_MARK_REG);
	sft_calib = buf;
	buf &= ~(0x02);
	rk816_bat_write(di, RK816_MISC_MARK_REG, buf);

	if ((sft_calib & 0x02) && di->pwroff_min >= 30)
		return true;
	else
		return false;
}

static void rk816_bat_init_rsoc(struct rk816_battery *di)
{
	di->bat_first_power_on = is_rk816_bat_first_pwron(di);
	di->pwroff_min = rk816_bat_get_pwroff_min(di);

	if (di->bat_first_power_on || rk816_bat_ocv_sw_reset(di))
		rk816_bat_first_pwron(di);
	else
		rk816_bat_not_first_pwron(di);
}

static u8 rk816_bat_get_chrg_status(struct rk816_battery *di)
{
	u8 status;

	status = rk816_bat_read(di, RK816_SUP_STS_REG);
	status &= (0x70);
	switch (status) {
	case CHARGE_OFF:
		DBG("CHARGE-OFF ...\n");
		break;
	case DEAD_CHARGE:
		BAT_INFO("DEAD CHARGE...\n");
		break;
	case  TRICKLE_CHARGE:
		BAT_INFO("TRICKLE CHARGE...\n ");
		break;
	case  CC_OR_CV:
		DBG("CC or CV...\n");
		break;
	case  CHARGE_FINISH:
		DBG("CHARGE FINISH...\n");
		break;
	case  USB_OVER_VOL:
		BAT_INFO("USB OVER VOL...\n");
		break;
	case  BAT_TMP_ERR:
		BAT_INFO("BAT TMP ERROR...\n");
		break;
	case  TIMER_ERR:
		BAT_INFO("TIMER ERROR...\n");
		break;
	case  USB_EXIST:
		BAT_INFO("USB EXIST...\n");
		break;
	case  USB_EFF:
		BAT_INFO("USB EFF...\n");
		break;
	default:
		return -EINVAL;
	}

	return status;
}

static u8 rk816_bat_fb_temp(struct rk816_battery *di)
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

static void rk816_bat_select_chrg_cv(struct rk816_battery *di)
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

static u8 rk816_bat_finish_ma(int fcc)
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

static void rk816_bat_init_chrg_config(struct rk816_battery *di)
{
	u8 chrg_ctrl1, usb_ctrl, chrg_ctrl2, chrg_ctrl3;
	u8 sup_sts, thermal, ggcon, finish_ma, fb_temp;

	rk816_bat_select_chrg_cv(di);
	finish_ma = rk816_bat_finish_ma(di->fcc);
	fb_temp = rk816_bat_fb_temp(di);

	ggcon = rk816_bat_read(di, RK816_GGCON_REG);
	sup_sts = rk816_bat_read(di, RK816_SUP_STS_REG);
	thermal = rk816_bat_read(di, RK816_THERMAL_REG);
	usb_ctrl = rk816_bat_read(di, RK816_USB_CTRL_REG);
	chrg_ctrl1 = rk816_bat_read(di, RK816_CHRG_CTRL_REG1);
	chrg_ctrl2 = rk816_bat_read(di, RK816_CHRG_CTRL_REG2);
	chrg_ctrl3 = rk816_bat_read(di, RK816_CHRG_CTRL_REG3);

	/* set charge current and voltage */
	usb_ctrl &= (~0x0f);
	usb_ctrl |= di->chrg_cur_input;
	chrg_ctrl1 &= (0x00);
	chrg_ctrl1 |= (CHRG_EN) | (di->chrg_vol_sel | di->chrg_cur_sel);

	/* set charge finish current */
	chrg_ctrl3 |= CHRG_TERM_DIG_SIGNAL;
	chrg_ctrl2 &= ~(0xc7);
	chrg_ctrl2 |= finish_ma;

	/* disable cccv mode */
	chrg_ctrl3 &= ~CHRG_TIMER_CCCV_EN;

	/* enable voltage limit and enable input current limit */
	sup_sts |= (0x01 << 3);
	sup_sts |= (0x01 << 2);

	/* set feed back temperature */
	if (di->pdata->fb_temp)
		usb_ctrl |= CHRG_CT_EN;
	else
		usb_ctrl &= ~CHRG_CT_EN;
	thermal &= (~0x0c);
	thermal |= fb_temp;

	/* adc current mode */
	ggcon &= ~0x03;
	ggcon |= ADC_CUR_MODE;
	ggcon |= AVG_CUR_MODE;

	rk816_bat_write(di, RK816_GGCON_REG, ggcon);
	rk816_bat_write(di, RK816_SUP_STS_REG, sup_sts);
	rk816_bat_write(di, RK816_THERMAL_REG, thermal);
	rk816_bat_write(di, RK816_USB_CTRL_REG, usb_ctrl);
	rk816_bat_write(di, RK816_CHRG_CTRL_REG1, chrg_ctrl1);
	rk816_bat_write(di, RK816_CHRG_CTRL_REG2, chrg_ctrl2);
	rk816_bat_write(di, RK816_CHRG_CTRL_REG3, chrg_ctrl3);
}

static void rk816_bat_init_poffset(struct rk816_battery *di)
{
	int coffset, ioffset;

	coffset = rk816_bat_get_coffset(di);
	ioffset = rk816_bat_get_ioffset(di);
	di->poffset = coffset - ioffset;
}

static void rk816_bat_caltimer_isr(unsigned long data)
{
	struct rk816_battery *di = (struct rk816_battery *)data;

	mod_timer(&di->caltimer, jiffies + TIMER_CALIB_8MIN * HZ);
	queue_delayed_work(di->bat_monitor_wq, &di->calib_delay_work,
			   msecs_to_jiffies(10));
}

static void rk816_bat_internal_calib(struct work_struct *work)
{
	int ioffset;
	struct rk816_battery *di = container_of(work,
			struct rk816_battery, calib_delay_work.work);

	ioffset = rk816_bat_get_ioffset(di);
	rk816_bat_set_coffset(di, di->poffset + ioffset);
	rk816_bat_init_voltage_kb(di);
	BAT_INFO("caltimer: ioffset=0x%x, coffset=0x%x\n",
		 ioffset, rk816_bat_get_coffset(di));
}

static void rk816_bat_init_caltimer(struct rk816_battery *di)
{
	setup_timer(&di->caltimer, rk816_bat_caltimer_isr, (unsigned long)di);
	di->caltimer.expires = jiffies + TIMER_CALIB_8MIN * HZ;
	add_timer(&di->caltimer);
	INIT_DELAYED_WORK(&di->calib_delay_work, rk816_bat_internal_calib);
}

static void rk816_bat_init_zero_table(struct rk816_battery *di)
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
	diff = (max - min) / (ocv_size - 1);
	for (i = 0; i < ocv_size; i++)
		di->pdata->zero_table[i] = min + (i * diff);

	for (i = 0; i < ocv_size; i++)
		DBG("zero[%d] = %d\n", i, di->pdata->zero_table[i]);

	for (i = 0; i < ocv_size; i++)
		DBG("ocv[%d] = %d\n", i, di->pdata->ocv_table[i]);
}

static void rk816_bat_calc_sm_linek(struct rk816_battery *di)
{
	int linek, current_avg;
	u8 diff, delta;

	delta = abs(di->dsoc - di->rsoc);
	diff = delta * 3;/* speed:3/4 */
	current_avg = rk816_bat_get_avg_current(di);
	if (current_avg >= 0) {
		if (di->dsoc < di->rsoc)
			linek = 1000 * (delta + diff) / diff;
		else if (di->dsoc > di->rsoc)
			linek = 1000 * diff / (delta + diff);
		else
			linek = 1000;
		di->dbg_meet_soc = (di->dsoc >= di->rsoc) ?
				   (di->dsoc + diff) : (di->rsoc + diff);
	} else {
		if (di->dsoc < di->rsoc)
			linek = -1000 * diff / (delta + diff);
		else if (di->dsoc > di->rsoc)
			linek = -1000 * (delta + diff) / diff;
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

static void rk816_bat_calc_zero_linek(struct rk816_battery *di)
{
	int dead_voltage, ocv_voltage;
	int voltage_avg, current_avg;
	int ocv_cap, dead_cap, xsoc;
	int ocv_soc, dead_soc;
	int pwroff_vol, org_linek = 0;
	int min_gap_xsoc;

	if (abs(di->current_avg) < 400)
		pwroff_vol = di->pdata->pwroff_vol + 50;
	else
		pwroff_vol = di->pdata->pwroff_vol;

	/* calc estimate ocv voltage */
	voltage_avg = rk816_bat_get_avg_voltage(di);
	current_avg = rk816_bat_get_avg_current(di);

	DBG("ZERO0: shtd_vol: org = %d, now = %d\n",
	    di->pdata->pwroff_vol, pwroff_vol);
	dead_voltage = pwroff_vol - current_avg *
				(di->bat_res + DEF_PWRPATH_RES) / 1000;
	ocv_voltage = voltage_avg - (current_avg * di->bat_res) / 1000;
	DBG("ZERO0: dead_voltage(shtd) = %d, ocv_voltage(now) = %d\n",
	    dead_voltage, ocv_voltage);

	/* calc estimate soc and cap */
	dead_soc = rk816_bat_vol_to_zerosoc(di, dead_voltage);
	dead_cap = rk816_bat_vol_to_zerocap(di, dead_voltage);
	DBG("ZERO0: dead_soc = %d, dead_cap = %d\n",
	    dead_soc, dead_cap);

	ocv_soc = rk816_bat_vol_to_zerosoc(di, ocv_voltage);
	ocv_cap = rk816_bat_vol_to_zerocap(di, ocv_voltage);
	DBG("ZERO0: ocv_soc = %d, ocv_cap = %d\n",
	    ocv_soc, ocv_cap);

	if (abs(current_avg) > 1400)
		min_gap_xsoc = MIN_ZERO_GAP_XSOC3;
	else if (abs(current_avg) > 600)
		min_gap_xsoc = MIN_ZERO_GAP_XSOC2;
	else
		min_gap_xsoc = MIN_ZERO_GAP_XSOC1;

	/* xsoc: available rsoc */
	xsoc = ocv_soc - dead_soc;
	di->zero_remain_cap = di->remain_cap;
	if ((di->dsoc <= 1) && (xsoc > 0)) {
		di->zero_linek = 600;
		di->zero_drop_sec = 0;
	} else if (xsoc >= 0) {
		di->zero_drop_sec = 0;
		di->zero_linek = (di->zero_dsoc + xsoc / 2) / DIV(xsoc);
		org_linek = di->zero_linek;
		/* battery energy mode to use up voltage */
		if ((di->pdata->energy_mode) &&
		    (xsoc - di->dsoc >= MIN_ZERO_GAP_XSOC3) &&
		    (di->dsoc <= 10) && (di->zero_linek < 600)) {
			di->zero_linek = 600;
			DBG("ZERO-new: zero_linek adjust step0...\n");
		/* reserve enough power yet, slow down any way */
		} else if ((xsoc - di->dsoc >= min_gap_xsoc) ||
			   ((xsoc - di->dsoc >= MIN_ZERO_GAP_XSOC2) &&
			    (di->dsoc <= 10))) {
			if (xsoc - di->dsoc >= 2 + min_gap_xsoc)
				di->zero_linek = 700;
			else
				di->zero_linek = 800;
			DBG("ZERO-new: zero_linek adjust step1...\n");
		/* control zero mode beginning enter */
		} else if ((di->zero_linek > 1800) && (di->dsoc > 70)) {
			di->zero_linek = 1800;
			DBG("ZERO-new: zero_linek adjust step2...\n");
		/* dsoc close to xsoc: it must reserve power */
		} else if ((di->zero_linek > 1000) && (di->zero_linek < 1200)) {
			di->zero_linek = 1300;
			DBG("ZERO-new: zero_linek adjust step3...\n");
		/* dsoc[5~15], dsoc < xsoc */
		} else if ((di->dsoc <= 15 && di->dsoc > 5) &&
			   (di->zero_linek <= 1200)) {
				/* slow down */
				if ((xsoc - di->dsoc) >= min_gap_xsoc)
					di->zero_linek = 800;
				/* reserve power */
				else
					di->zero_linek = 1300;
			DBG("ZERO-new: zero_linek adjust step4...\n");
		/* dsoc[5, 100], dsoc < xsoc */
		} else if ((di->zero_linek < 1000) && (di->dsoc >= 5)) {
			if ((xsoc - di->dsoc) < min_gap_xsoc) {
				/* reserve power */
				di->zero_linek = 1300;
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
				di->zero_linek = 1300;
			else
				di->zero_linek = 800;
			DBG("ZERO-new: zero_linek adjust step5...\n");
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
						MIN_ZERO_ROUND_ACCURACY;
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
	    "rsoc=%d, gap=%d, v=%d\n"
	    "ZERO-new: di->zero_dsoc=%d, sm_remain_cap=%d, zero_drop=%ld, "
	    "sht_drop=%ld\n\n",
	    org_linek, di->zero_linek, di->dsoc, xsoc, di->rsoc,
	    min_gap_xsoc, voltage_avg, di->zero_dsoc, di->zero_remain_cap,
	    base2sec(di->zero_drop_sec), base2sec(di->shtd_drop_sec));
}

static void rk816_bat_smooth_algo_prepare(struct rk816_battery *di)
{
	int tmp_soc;

	tmp_soc = di->sm_chrg_dsoc / 1000;
	if (tmp_soc != di->dsoc)
		di->sm_chrg_dsoc = di->dsoc * 1000;

	tmp_soc = di->sm_dischrg_dsoc / 1000;
	if (tmp_soc != di->dsoc)
		di->sm_dischrg_dsoc =
		(di->dsoc + 1) * 1000 - MIN_ZERO_ROUND_ACCURACY;

	DBG("<%s>. tmp_soc=%d, dsoc=%d, dsoc:sm_dischrg=%d, sm_chrg=%d\n",
	    __func__, tmp_soc, di->dsoc, di->sm_dischrg_dsoc, di->sm_chrg_dsoc);

	rk816_bat_calc_sm_linek(di);
}

static void rk816_bat_zero_algo_prepare(struct rk816_battery *di)
{
	int tmp_dsoc;

	di->zero_timeout_cnt = 0;
	tmp_dsoc = (di->zero_dsoc + MIN_ZERO_ROUND_ACCURACY) / 1000;
	if (tmp_dsoc != di->dsoc) {
		di->zero_dsoc = (di->dsoc + 1) * 1000 - MIN_ZERO_ROUND_ACCURACY;
		rk816_bat_calc_zero_linek(di);
	}

	DBG("<%s>. first calc, reinit linek\n", __func__);
}

static void rk816_bat_init_coulomb_cap(struct rk816_battery *di, u32 capacity)
{
	u8 buf;
	u32 cap;

	cap = capacity * 2390;
	buf = (cap >> 24) & 0xff;
	rk816_bat_write(di, RK816_GASCNT_CAL_REG3, buf);
	buf = (cap >> 16) & 0xff;
	rk816_bat_write(di, RK816_GASCNT_CAL_REG2, buf);
	buf = (cap >> 8) & 0xff;
	rk816_bat_write(di, RK816_GASCNT_CAL_REG1, buf);
	buf = (cap >> 0) & 0xff;
	rk816_bat_write(di, RK816_GASCNT_CAL_REG0, buf);

	di->remain_cap = capacity;
	di->rsoc = rk816_bat_get_rsoc(di);
}

static void rk816_bat_zero_algorithm(struct rk816_battery *di)
{
	int delta_cap, delta_soc, tmp_soc = 0;

	di->zero_timeout_cnt++;
	delta_cap = di->zero_remain_cap - di->remain_cap;
	delta_soc = di->zero_linek * (delta_cap * 100) / DIV(di->fcc);

	DBG("ZERO1: zero_linek=%d, zero_dsoc(Y0)=%d, dsoc=%d, rsoc=%d\n"
	    "ZERO1: delta_soc(X0)=%d, delta_cap=%d, sm_remain_cap = %d\n"
	    "ZERO1: timeout_cnt=%d\n\n",
	    di->zero_linek, di->zero_dsoc, di->dsoc, di->rsoc,
	    delta_soc, delta_cap, di->zero_remain_cap,
	    di->zero_timeout_cnt);

	if ((delta_soc >= MIN_ZERO_DSOC_ACCURACY) ||
	    (di->zero_timeout_cnt > MIN_ZERO_OVERCNT) ||
	    (di->zero_linek == 0)) {
		DBG("ZERO1:--------- enter calc -----------\n");
		di->zero_timeout_cnt = 0;
		di->zero_dsoc -= delta_soc;
		tmp_soc = (di->zero_dsoc + MIN_ZERO_ROUND_ACCURACY) / 1000;
		if (tmp_soc != di->dsoc) {
			/* avoid dsoc jump when heavy load */
			if ((di->dsoc - tmp_soc) > 1) {
				di->dsoc--;
				di->zero_dsoc = (di->dsoc + 1) * 1000 -
						MIN_ZERO_ROUND_ACCURACY;
				DBG("ZERO1: heavy load...\n");
			} else {
				di->dsoc = tmp_soc;
			}
			di->zero_drop_sec = 0;
		}
		DBG("ZERO1: zero_dsoc(Y0)=%d, dsoc=%d, rsoc=%d, tmp_soc=%d",
		    di->zero_dsoc, di->dsoc, di->rsoc, tmp_soc);

		rk816_bat_calc_zero_linek(di);
	}
}

static void rk816_bat_dump_time_table(struct rk816_battery *di)
{
	u8 i;
	static int old_index;
	static int old_min;
	u32 time;
	int mod = di->dsoc % 10;
	int index = di->dsoc / 10;

	if (rk816_bat_chrg_online(di))
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

static void rk816_bat_debug_info(struct rk816_battery *di)
{
	u8 sup_tst, ggcon, ggsts, vb_mod, ts_ctrl;
	u8 usb_ctrl, chrg_ctrl1, thermal;
	u8 int_sts1, int_sts2, int_sts3;
	u8 int_msk1, int_msk2, int_msk3;
	u8 chrg_ctrl2, chrg_ctrl3, rtc, misc, dcdc_en2;
	char *work_mode[] = {"ZERO", "SMOOTH", "FINISH"};
	char *bat_mode[] = {"BAT", "VIRTUAL"};

	if (rk816_bat_chrg_online(di))
		di->plug_out_base = get_boot_sec();
	else
		di->plug_in_base = get_boot_sec();

	rk816_bat_dump_time_table(di);

	if (!dbg_enable)
		return;

	ts_ctrl = rk816_bat_read(di, RK816_TS_CTRL_REG);
	misc = rk816_bat_read(di, RK816_MISC_MARK_REG);
	ggcon = rk816_bat_read(di, RK816_GGCON_REG);
	ggsts = rk816_bat_read(di, RK816_GGSTS_REG);
	sup_tst = rk816_bat_read(di, RK816_SUP_STS_REG);
	vb_mod = rk816_bat_read(di, RK816_VB_MON_REG);
	usb_ctrl = rk816_bat_read(di, RK816_USB_CTRL_REG);
	chrg_ctrl1 = rk816_bat_read(di, RK816_CHRG_CTRL_REG1);
	chrg_ctrl2 = rk816_bat_read(di, RK816_CHRG_CTRL_REG2);
	chrg_ctrl3 = rk816_bat_read(di, RK816_CHRG_CTRL_REG3);
	rtc = rk816_bat_read(di, RK816_SECONDS_REG);
	thermal = rk816_bat_read(di, RK816_THERMAL_REG);
	int_sts1 = rk816_bat_read(di, RK816_INT_STS_REG1);
	int_sts2 = rk816_bat_read(di, RK816_INT_STS_REG2);
	int_sts3 = rk816_bat_read(di, RK816_INT_STS_REG3);
	int_msk1 = rk816_bat_read(di, RK816_INT_STS_MSK_REG1);
	int_msk2 = rk816_bat_read(di, RK816_INT_STS_MSK_REG2);
	int_msk3 = rk816_bat_read(di, RK816_INT_STS_MSK_REG3);
	dcdc_en2 = rk816_bat_read(di, RK816_DCDC_EN_REG2);

	DBG("\n------- DEBUG REGS, [Ver: %s] -------------------\n"
	    "GGCON=0x%2x, GGSTS=0x%2x, RTC=0x%2x, DCDC_EN2=0x%2x\n"
	    "SUP_STS= 0x%2x, VB_MOD=0x%2x, USB_CTRL=0x%2x\n"
	    "THERMAL=0x%2x, MISC_MARK=0x%2x, TS_CTRL=0x%2x\n"
	    "CHRG_CTRL:REG1=0x%2x, REG2=0x%2x, REG3=0x%2x\n"
	    "INT_STS:  REG1=0x%2x, REG2=0x%2x, REG3=0x%2x\n"
	    "INT_MSK:  REG1=0x%2x, REG2=0x%2x, REG3=0x%2x\n",
	    DRIVER_VERSION, ggcon, ggsts, rtc, dcdc_en2,
	    sup_tst, vb_mod, usb_ctrl,
	    thermal, misc, ts_ctrl,
	    chrg_ctrl1, chrg_ctrl2, chrg_ctrl3,
	    int_sts1, int_sts2, int_sts3,
	    int_msk1, int_msk2, int_msk3
	   );

	DBG("###############################################################\n"
	    "Dsoc=%d, Rsoc=%d, Vavg=%d, Iavg=%d, Cap=%d, Vusb=%d, Fcc=%d\n"
	    "K=%d, Mode=%s, Oldcap=%d, Is=%d, Ip=%d, Vs=%d\n"
	    "AC=%d, USB=%d, DC=%d, OTG=%d, PROP=%d, min=%d, rd=%d, wr=%d, "
	    "Tfb=%d, Tbat=%d\n"
	    "off:i=0x%x, c=0x%x, p=%d, Rbat=%d, age_ocv_cap=%d, fb=%d\n"
	    "adp:in=%lu, out=%lu, finish=%lu, LFcc=%d, boot_sec=%lu, h=%d, adc=%d\n"
	    "bat:%s, meet: soc=%d, calc: dsoc=%d, rsoc=%d, Vocv=%d\n"
	    "###############################################################\n",
	    di->dsoc, di->rsoc, di->voltage_avg, di->current_avg,
	    di->remain_cap, rk816_bat_get_usb_voltage(di), di->fcc,
	    di->sm_linek, work_mode[di->work_mode], di->sm_remain_cap,
	    CHRG_CUR_SEL[chrg_ctrl1 & 0x0f],
	    CHRG_CUR_INPUT[usb_ctrl & 0x0f],
	    CHRG_VOL_SEL[(chrg_ctrl1 & 0x70) >> 4],
	    di->ac_in, di->usb_in, di->dc_in, di->otg_in, di->prop_val,
	    di->pwroff_min, di->dbg_i2c_rd_err, di->dbg_i2c_wr_err,
	    FEED_BACK_TEMP[(thermal & 0x0c) >> 2], di->temperature,
	    rk816_bat_get_ioffset(di), rk816_bat_get_coffset(di),
	    di->poffset, di->bat_res, di->age_adjust_cap, di->fb_blank,
	    base2min(di->plug_in_base), base2min(di->plug_out_base),
	    base2min(di->chrg_finish_base), di->lock_fcc,
	    base2sec(di->boot_base), (int_sts1 & 0x1f), di->adc_allow_update,
	    bat_mode[di->pdata->bat_mode], di->dbg_meet_soc,
	    di->dbg_calc_dsoc, di->dbg_calc_rsoc, di->voltage_ocv
	   );
}

static void rk816_bat_init_capacity(struct rk816_battery *di, u32 cap)
{
	int delta_cap;

	delta_cap = cap - di->remain_cap;
	if (!delta_cap)
		return;

	di->age_adjust_cap += delta_cap;
	rk816_bat_init_coulomb_cap(di, cap);
	rk816_bat_smooth_algo_prepare(di);
	rk816_bat_zero_algo_prepare(di);
}

static void rk816_bat_update_fcc(struct rk816_battery *di)
{
	int fcc;
	int remain_cap;
	int age_keep_min;

	di->lock_fcc = rk816_bat_get_lock_fcc(di);
	if (di->lock_fcc == 0)
		return;

	fcc = di->lock_fcc;
	remain_cap = fcc - di->age_ocv_cap - di->age_adjust_cap;
	age_keep_min = base2min(di->age_keep_sec);

	DBG("%s: lock_fcc=%d, age_ocv_cap=%d, age_adjust_cap=%d, remain_cap=%d"
	    "age_allow_update=%d, age_keep_min:%d\n",
	    __func__, fcc, di->age_ocv_cap, di->age_adjust_cap, remain_cap,
	    di->age_allow_update, age_keep_min);

	if ((di->chrg_status == CHARGE_FINISH) && (di->age_allow_update) &&
	    (age_keep_min < 1200)) {
		di->age_allow_update = false;
		fcc = remain_cap * 100 / DIV(100 - di->age_ocv_soc);
		BAT_INFO("lock_fcc=%d, calc_cap=%d, age: soc=%d, cap=%d, "
			 "level=%d, fcc:%d->%d?\n",
			 di->lock_fcc, remain_cap, di->age_ocv_soc,
			 di->age_ocv_cap, di->age_level, di->fcc, fcc);

		if ((fcc < di->qmax) && (fcc > MIN_FCC)) {
			BAT_INFO("fcc:%d->%d!\n", di->fcc, fcc);
			di->fcc = fcc;
			rk816_bat_init_capacity(di, di->fcc);
			rk816_bat_save_fcc(di, di->fcc);
			rk816_bat_save_age_level(di, di->age_level);
		}
	}
}

static void rk816_bat_wait_finish_sig(struct rk816_battery *di)
{
	int chrg_finish_vol = di->pdata->max_chrg_voltage;

	if ((di->chrg_status == CHARGE_FINISH) && (!is_rk816_bat_cvtlim(di)) &&
	    (di->voltage_avg > chrg_finish_vol - 150) && di->adc_allow_update) {
		rk816_bat_update_fcc(di);/* save new fcc*/
		if (rk816_bat_adc_calib(di))
			di->adc_allow_update = false;
	}
}

static void rk816_bat_finish_chrg(struct rk816_battery *di)
{
	unsigned long finish_sec, soc_sec;
	int plus_soc, rest;

	if (di->dsoc < 100) {
		if (!di->chrg_finish_base)
			di->chrg_finish_base = get_boot_sec();

		finish_sec = base2sec(di->chrg_finish_base);
		soc_sec = di->fcc * 3600 / 100 / DSOC_CHRG_FINISH_CUR;
		plus_soc = finish_sec / soc_sec;
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

static void rk816_bat_smooth_algorithm(struct rk816_battery *di)
{
	int ydsoc = 0, delta_cap = 0, tmp_soc = 0, delta_dsoc = 0, old_cap = 0;
	unsigned long tgt_sec;

	di->remain_cap = rk816_bat_get_coulomb_cap(di);

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
			rk816_bat_calc_sm_linek(di);
		}
	}

	old_cap = di->sm_remain_cap;
	/*
	 * when dsoc equal rsoc, sm_linek should change to -1000/1000
	 * smooth to avoid dsoc+1/-1 right away
	 */
	if ((di->dsoc == di->rsoc) && (abs(di->sm_linek) != 1000)) {
		if (!di->flat_match_sec)
			di->flat_match_sec = get_boot_sec();
		tgt_sec = di->fcc * 3600 / 100 / abs(di->current_avg) / 3;
		if (base2sec(di->flat_match_sec) > tgt_sec) {
			di->flat_match_sec = 0;
			di->sm_linek = (di->current_avg >= 0) ? 1000 : -1000;
		}
		DBG("<%s>. flat_sec=%ld, tgt_sec=%ld\n", __func__,
		    base2sec(di->flat_match_sec), tgt_sec);
	} else {
		di->flat_match_sec = 0;
	}

	if (abs(di->sm_linek) == 1000) {
		di->sm_linek = (di->sm_linek > 0) ? 1000 : -1000;
		di->dsoc = di->rsoc;
		di->sm_remain_cap = di->remain_cap;
		di->sm_chrg_dsoc = di->dsoc * 1000;
		di->sm_dischrg_dsoc = (di->dsoc + 1) * 1000 -
						MIN_ZERO_ROUND_ACCURACY;
		DBG("<%s>. dsoc == rsoc: sm_dischrg_dsoc=%d, sm_chrg_dsoc=%d\n",
		    __func__, di->sm_dischrg_dsoc, di->sm_chrg_dsoc);
	} else {
		delta_cap = di->remain_cap - di->sm_remain_cap;
		if (delta_cap == 0)
			goto out;
		ydsoc = di->sm_linek * abs(delta_cap) * 100 / DIV(di->fcc);
		if (ydsoc == 0)
			goto out;
		di->sm_remain_cap = di->remain_cap;
		if (ydsoc < 0) {
			di->sm_dischrg_dsoc += ydsoc;
			tmp_soc = (di->sm_dischrg_dsoc +
					MIN_ZERO_ROUND_ACCURACY) / 1000;
			if (tmp_soc != di->dsoc) {
				/* count charge rest into calc */
				delta_dsoc = di->sm_chrg_dsoc - di->dsoc * 1000;
				di->sm_chrg_dsoc = di->dsoc * 1000;
				di->sm_dischrg_dsoc += delta_dsoc;
				tmp_soc = (di->sm_dischrg_dsoc +
						MIN_ZERO_ROUND_ACCURACY) / 1000;
				if (tmp_soc != di->dsoc) {
					di->dsoc = tmp_soc;
					di->sm_chrg_dsoc = di->dsoc * 1000;
				}
			}
		} else {
			di->sm_chrg_dsoc += ydsoc;
			tmp_soc = di->sm_chrg_dsoc / 1000;
			if (tmp_soc != di->dsoc) {
				/* count discharge rest into calc */
				delta_dsoc = ((di->dsoc + 1) * 1000 -
				MIN_ZERO_ROUND_ACCURACY) - di->sm_dischrg_dsoc;
				di->sm_dischrg_dsoc = (di->dsoc + 1) * 1000 -
					MIN_ZERO_ROUND_ACCURACY;
				di->sm_chrg_dsoc -= delta_dsoc;
				tmp_soc = di->sm_chrg_dsoc / 1000;
				if (tmp_soc != di->dsoc) {
					di->dsoc = tmp_soc;
					di->sm_dischrg_dsoc = (di->dsoc + 1) *
						1000 - MIN_ZERO_ROUND_ACCURACY;
				}
			}
		}
	}
out:
	DBG("<%s>.k=%d, ydsoc=%d, cap:old=%d, new:%d, delta=%d, delta_soc=%d\n",
	    __func__, di->sm_linek, ydsoc, old_cap, di->sm_remain_cap,
	    delta_cap, delta_dsoc);
	DBG("<%s>.dsoc=%d, rsoc=%d, dsoc: sm_dischrg=%d, sm_chrg=%d\n",
	    __func__, di->dsoc, di->rsoc,
	    di->sm_dischrg_dsoc, di->sm_chrg_dsoc);
}

static void rk816_bat_display_smooth(struct rk816_battery *di)
{
	/* discharge: reinit "zero & smooth" algorithm to avoid handling dsoc */
	if (di->s2r && !di->sleep_chrg_online) {
		DBG("s2r: discharge, reset algorithm...\n");
		rk816_bat_zero_algo_prepare(di);
		rk816_bat_smooth_algo_prepare(di);
		return;
	}

	if (di->work_mode == MODE_FINISH) {
		DBG("step1: charge finish...\n");
		rk816_bat_finish_chrg(di);
		if (rk816_bat_get_chrg_status(di) != CHARGE_FINISH) {
			if ((di->current_avg < 0) &&
			    (di->voltage_avg < di->pdata->zero_algorithm_vol)) {
				DBG("step1: change to zero mode...\n");
				rk816_bat_zero_algo_prepare(di);
				di->work_mode = MODE_ZERO;
			} else {
				DBG("step1: change to smooth mode...\n");
				rk816_bat_smooth_algo_prepare(di);
				di->work_mode = MODE_SMOOTH;
			}
		}
	} else if (di->work_mode == MODE_ZERO) {
		DBG("step2: zero algorithm...\n");
		rk816_bat_zero_algorithm(di);
		/*
		 * 1. exit zero_algorithm_vol
		 * 2. enter charge
		 */
		if ((di->voltage_avg >= di->pdata->zero_algorithm_vol + 50) ||
		    (di->current_avg >= 0)) {
			DBG("step2: change to smooth mode...\n");
			rk816_bat_smooth_algo_prepare(di);
			di->work_mode = MODE_SMOOTH;
		} else if (rk816_bat_get_chrg_status(di) == CHARGE_FINISH) {
			DBG("step2: change to finish mode...\n");
			di->chrg_finish_base = get_boot_sec();
			di->work_mode = MODE_FINISH;
		}
	} else {
		DBG("step3: smooth algorithm...\n");
		rk816_bat_smooth_algorithm(di);
		if ((di->current_avg < 0) &&
		    (di->voltage_avg < di->pdata->zero_algorithm_vol)) {
			DBG("step3: change to zero mode...\n");
			rk816_bat_zero_algo_prepare(di);
			di->work_mode = MODE_ZERO;
		} else if (rk816_bat_get_chrg_status(di) == CHARGE_FINISH) {
			DBG("step3: change to finish mode...\n");
			di->chrg_finish_base = get_boot_sec();
			di->work_mode = MODE_FINISH;
		}
	}
}

static void rk816_bat_relax_vol_calib(struct rk816_battery *di)
{
	int soc, cap, vol;

	vol = di->voltage_relax - (di->current_relax * di->bat_res) / 1000;
	soc = rk816_bat_vol_to_ocvsoc(di, vol);
	cap = rk816_bat_vol_to_ocvcap(di, vol);
	rk816_bat_init_capacity(di, cap);
	BAT_INFO("sleep ocv calib: rsoc=%d, cap=%d\n", soc, cap);
}

static void rk816_bat_check_fcc_flag(struct rk816_battery *di)
{
	u8 ocv_soc, ocv_cap, soc_level;

	if (di->voltage_relax <= 0)
		return;

	ocv_soc = rk816_bat_vol_to_ocvsoc(di, di->voltage_relax);
	ocv_cap = rk816_bat_vol_to_ocvcap(di, di->voltage_relax);
	DBG("<%s>. ocv_soc=%d, min=%lu, vol=%d\n", __func__,
	    ocv_soc, SEC_TO_MIN(di->sleep_dischrg_sec), di->voltage_relax);

	/* sleep enough time */
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

		soc_level = rk816_bat_get_age_level(di);
		if (soc_level > di->age_level) {
			di->age_allow_update = false;
		} else {
			di->age_allow_update = true;
			di->age_keep_sec = get_boot_sec();
		}

		BAT_INFO("resume: relax_vol:%d, dod0_cap:%d\n"
			 "age_ocv_soc:%d, soc_level:%d: age_allow_update:%d\n"
			 "age_level:%d",
			 di->age_voltage, di->age_ocv_cap,
			 ocv_soc, soc_level, di->age_allow_update,
			 di->age_level);
	}
}

static int rk816_bat_sleep_dischrg(struct rk816_battery *di)
{
	bool ocv_soc_updated = false;
	int tgt_dsoc, gap_soc, sleep_soc = 0;
	int pwroff_vol = di->pdata->pwroff_vol;
	unsigned long sleep_sec = di->sleep_dischrg_sec;

	DBG("<%s>. enter: dsoc=%d, rsoc=%d, rv=%d, v=%d, sleep_min=%lu\n",
	    __func__, di->dsoc, di->rsoc, di->voltage_relax,
	    di->voltage_avg, sleep_sec / 60);

	if (di->voltage_relax >= di->voltage_avg) {
		rk816_bat_relax_vol_calib(di);
		rk816_bat_restart_relax(di);
		rk816_bat_check_fcc_flag(di);
		ocv_soc_updated = true;
	}

	/*handle dsoc*/
	if (di->dsoc <= di->rsoc) {
		di->sleep_sum_cap = (SLP_CURR_MIN * sleep_sec / 3600);
		sleep_soc = di->sleep_sum_cap * 100 / di->fcc;
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
		/*di->dsoc > di->rsoc*/
		di->sleep_sum_cap = (SLP_CURR_MAX * sleep_sec / 3600);
		sleep_soc = di->sleep_sum_cap / (di->fcc / 100);
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
		BAT_INFO("low power sleeping, shutdown... %d\n", di->dsoc);
	}

	if (ocv_soc_updated && sleep_soc && (di->rsoc - di->dsoc) < 5 &&
	    di->dsoc < 40) {
		di->dsoc--;
		BAT_INFO("low power sleeping, reserved... %d\n", di->dsoc);
	}

	if (di->dsoc <= 0)
		di->dsoc = 0;

	DBG("<%s>. out: dsoc=%d, rsoc=%d, sum_cap=%d\n",
	    __func__, di->dsoc, di->rsoc, di->sleep_sum_cap);

	return sleep_soc;
}

static void rk816_bat_power_supply_changed(struct rk816_battery *di)
{
	static int old_soc = -1;

	if (di->dsoc > 100) {
		di->dsoc = 100;
		if (rk816_bat_chrg_online(di))
			di->prop_val = POWER_SUPPLY_STATUS_FULL;
	} else if (di->dsoc < 0) {
		di->dsoc = 0;
	}

	if (di->dsoc == old_soc)
		return;

	old_soc = di->dsoc;
	power_supply_changed(&di->bat);
	BAT_INFO("changed: dsoc=%d, rsoc=%d\n", di->dsoc, di->rsoc);
}

static u8 rk816_bat_check_reboot(struct rk816_battery *di)
{
	u8 cnt;

	cnt = rk816_bat_read(di, RK816_REBOOT_CNT_REG);
	cnt++;

	if (cnt >= REBOOT_MAX_CNT) {
		BAT_INFO("reboot: %d --> %d\n", di->dsoc, di->rsoc);
		di->dsoc = di->rsoc;
		if (di->dsoc > 100)
			di->dsoc = 100;
		else if (di->dsoc < 0)
			di->dsoc = 0;
		rk816_bat_save_dsoc(di, di->dsoc);
		cnt = REBOOT_MAX_CNT;
	}

	rk816_bat_save_reboot_cnt(di, cnt);
	DBG("reboot cnt: %d\n", cnt);

	return cnt;
}

static void rk816_bat_check_charger(struct rk816_battery *di)
{
	u8 buf;
	int usb_id;
	enum charger_type charger;

	buf = rk816_bat_read(di, RK816_VB_MON_REG);
	/* pmic detect plug in, but ac/usb/dc_in offline, do check */
	if ((buf & PLUG_IN_STS) != 0 && !rk816_bat_chrg_online(di)) {
		usb_id = dwc_otg_check_dpdm(0);
		switch (usb_id) {
		case 0:
			charger = AC_CHARGER;
			break;
		case 1:
		case 3:
			charger = USB_CHARGER;
			break;
		case 2:
			charger = AC_CHARGER;
			break;
		default:
			break;
		}

		rk816_bat_set_chrg_param(di, charger);
		BAT_INFO("pmic detect charger.. %s\n",
			 charger == AC_CHARGER ? "AC" : "USB");
	/* pmic not detect plug in, but one of ac/usb/dc_in online, reset */
	} else if ((buf & PLUG_IN_STS) == 0 && rk816_bat_chrg_online(di)) {
		rk816_bat_set_chrg_param(di, NONE_CHARGER);
		BAT_INFO("pmic not detect charger..\n");
	}
}

static void rk816_bat_update_info(struct rk816_battery *di)
{
	di->voltage_avg = rk816_bat_get_avg_voltage(di);
	di->current_avg = rk816_bat_get_avg_current(di);
	di->chrg_status = rk816_bat_get_chrg_status(di);
	di->voltage_relax = rk816_bat_get_relax_voltage(di);
	di->rsoc = rk816_bat_get_rsoc(di);
	di->remain_cap = rk816_bat_get_coulomb_cap(di);

	/* adjust finish fcc and sm remain cap */
	if ((di->remain_cap > di->fcc) ||
	    (di->chrg_status == CHARGE_FINISH)) {
		di->sm_remain_cap -= (di->remain_cap - di->fcc);
		DBG("<%s>. cap: remain=%d, sm_remain=%d\n",
		    __func__, di->remain_cap, di->sm_remain_cap);
		rk816_bat_init_coulomb_cap(di, di->fcc);
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
	if (di->chrg_status == CC_OR_CV)
		di->adc_allow_update = true;
}

static void rk816_bat_save_data(struct rk816_battery *di)
{
	rk816_bat_save_dsoc(di, di->dsoc);
	rk816_bat_save_cap(di, di->remain_cap);
}

/*get ntc resistance*/
static int rk816_bat_get_ntc_res(struct rk816_battery *di)
{
	int res, val = 0;

	val |= rk816_bat_read(di, RK816_TS_ADC_REGL) << 0;
	val |= rk816_bat_read(di, RK816_TS_ADC_REGH) << 8;

	res = ((di->voltage_k * val) / 1000 + di->voltage_b) * 1000 / 2200;
	res = res * 1000 / 80;

	DBG("<%s>. val = %d, ntc_res=%d\n", __func__, val, res);

	return res;
}

static void rk816_bat_update_temperature(struct rk816_battery *di)
{
	u32 ntc_size, *ntc_table;
	int i, res;

	ntc_table = di->pdata->ntc_table;
	ntc_size = di->pdata->ntc_size;
	di->temperature = VIRTUAL_TEMPERATURE;

	if (ntc_size) {
		res = rk816_bat_get_ntc_res(di);
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

static void rk816_battery_work(struct work_struct *work)
{
	int ms;
	struct rk816_battery *di;

	di = container_of(work, struct rk816_battery,
			  bat_delay_work.work);

	ms = di->monitor_ms;
	if (rk816_bat_chrg_online(di))
		rk816_bat_wait_finish_sig(di);

	rk816_bat_update_info(di);
	rk816_bat_check_charger(di);
	rk816_bat_update_temperature(di);
	rk816_bat_display_smooth(di);
	rk816_bat_power_supply_changed(di);
	rk816_bat_save_data(di);
	rk816_bat_debug_info(di);

	/* discharge: delay 30ms for next loop when android health wakeup */
	if (!di->early_resume && di->s2r && !di->sleep_chrg_online)
		ms = 30 * TIMER_MS_COUNTS;
	else
		di->early_resume = false;

	di->s2r = false;
	queue_delayed_work(di->bat_monitor_wq, &di->bat_delay_work,
			   msecs_to_jiffies(ms));
}

static int rk816_bat_usb_notifier_call(struct notifier_block *nb,
				       unsigned long event, void *data)
{
	enum charger_type charger;
	struct rk816_battery *di =
	    container_of(nb, struct rk816_battery, bc_detect_nb);
	const char *event_name[] = {"DISCNT", "USB", "AC", "AC",
				    "UNKNOWN", "OTG ON", "OTG OFF"};

	if (di->pdata->bat_mode == MODE_VIRTUAL)
		return NOTIFY_OK;

	BAT_INFO("receive usb notifier event: %s..\n", event_name[event]);
	switch (event) {
	case USB_BC_TYPE_DISCNT:
		rk816_bat_set_chrg_param(di, NO_ACUSB_CHARGER);
		break;
	case USB_BC_TYPE_SDP:
		charger = rk816_bat_get_acusb_state(di);
		DBG("%s: charger=%d\n", __func__, charger);
		if (charger != NO_ACUSB_CHARGER)
			rk816_bat_set_chrg_param(di, USB_CHARGER);
		break;
	case USB_BC_TYPE_CDP:
	case USB_BC_TYPE_DCP:
		charger = rk816_bat_get_acusb_state(di);
		DBG("%s: charger=%d\n", __func__, charger);
		if (charger != NO_ACUSB_CHARGER)
			rk816_bat_set_chrg_param(di, AC_CHARGER);
		break;
	case USB_OTG_POWER_ON:
		di->otg_in = ONLINE;
		if (di->pdata->power_dc2otg && di->dc_in) {
			BAT_INFO("otg power from dc adapter\n");
		} else {
			rk816_bat_set_otg_state(di, USB_OTG_POWER_ON);
			BAT_INFO("charge disable, enable otg\n");
		}
		break;
	case USB_OTG_POWER_OFF:
		di->otg_in = OFFLINE;
		rk816_bat_set_otg_state(di, USB_OTG_POWER_OFF);
		BAT_INFO("charge enable, disable otg\n");
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static irqreturn_t rk816_vb_low_irq(int irq, void *bat)
{
	struct rk816_battery *di = (struct rk816_battery *)bat;

	BAT_INFO("lower power yet, power off system! v=%d\n", di->voltage_avg);
	rk_send_wakeup_key();
	kernel_power_off();

	return IRQ_HANDLED;
}

static irqreturn_t rk816_plug_in(int irq, void *bat)
{
	struct rk816_battery *di = (struct rk816_battery *)bat;

	rk816_bat_write(di, RK816_INT_STS_REG3, BIT(0));
	rk_send_wakeup_key();
	BAT_INFO("pmic isr: plug in\n");

	return IRQ_HANDLED;
}

static irqreturn_t rk816_plug_out(int irq, void  *bat)
{
	struct rk816_battery *di = (struct rk816_battery *)bat;

	rk816_bat_write(di, RK816_INT_STS_REG3, BIT(1));
	rk_send_wakeup_key();
	BAT_INFO("pmic isr: plug out\n");

	return IRQ_HANDLED;
}

static irqreturn_t rk816_vbat_dc_det(int irq, void *bat)
{
	struct rk816_battery *di = (struct rk816_battery *)bat;

	if (gpio_get_value(di->pdata->dc_det_pin))
		irq_set_irq_type(irq, IRQF_TRIGGER_LOW);
	else
		irq_set_irq_type(irq, IRQF_TRIGGER_HIGH);

	BAT_INFO("dc det isr: in/out\n");
	queue_delayed_work(di->dc_monitor_wq,
			   &di->dc_delay_work, msecs_to_jiffies(10));
	rk_send_wakeup_key();

	return IRQ_HANDLED;
}

static int rk816_bat_init_sysfs(struct rk816_battery *di)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(rk816_bat_attr); i++) {
		ret = sysfs_create_file(&di->bat.dev->kobj,
					&rk816_bat_attr[i].attr);
		if (ret != 0)
			dev_err(di->dev, "create bat node(%s) error\n",
				rk816_bat_attr[i].attr.name);
	}

	return ret;
}

static int rk816_bat_init_irqs(struct rk816_battery *di)
{
	int ret;
	int plug_in_irq, plug_out_irq, vb_lo_irq;
	struct rk816 *rk816 = di->rk816;
	struct platform_device *pdev = di->pdev;

	vb_lo_irq = regmap_irq_get_virq(rk816->irq_data, RK816_IRQ_VB_LOW);
	if (vb_lo_irq < 0) {
		dev_err(&pdev->dev, "find vb_lo_irq error\n");
		return vb_lo_irq;
	}

	plug_in_irq = regmap_irq_get_virq(rk816->battery_irq_data,
					  RK816_IRQ_PLUG_IN);
	if (plug_in_irq < 0) {
		dev_err(&pdev->dev, "find plug_in_irq error\n");
		return plug_in_irq;
	}

	plug_out_irq = regmap_irq_get_virq(rk816->battery_irq_data,
					   RK816_IRQ_PLUG_OUT);
	if (plug_out_irq < 0) {
		dev_err(&pdev->dev, "find plug_out_irq error\n");
		return plug_out_irq;
	}

	/* low power */
	ret = devm_request_threaded_irq(di->dev, vb_lo_irq, NULL,
					rk816_vb_low_irq, IRQF_TRIGGER_HIGH,
					"rk816_vb_low", di);
	if (ret != 0) {
		dev_err(di->dev, "vb low irq request failed!\n");
		return ret;
	}

	enable_irq_wake(vb_lo_irq);

	/* plug in */
	ret = devm_request_threaded_irq(di->dev, plug_in_irq, NULL,
					rk816_plug_in, IRQF_TRIGGER_FALLING,
					"rk816_plug_in", di);
	if (ret != 0) {
		dev_err(di->dev, "plug in irq request failed!\n");
		return ret;
	}

	/* plug out */
	ret = devm_request_threaded_irq(di->dev, plug_out_irq, NULL,
					rk816_plug_out, IRQF_TRIGGER_FALLING,
					"rk816_plug_out", di);
	if (ret != 0) {
		dev_err(di->dev, "plug out irq request failed!\n");
		return ret;
	}

	return 0;
}

static void rk816_bat_init_info(struct rk816_battery *di)
{
	di->design_cap = di->pdata->design_capacity;
	di->qmax = di->pdata->design_qmax;
	di->bat_res = di->pdata->bat_res;
	di->fcc = rk816_bat_get_fcc(di);
	di->early_resume = true;
	di->sleep_chrg_status = rk816_bat_get_chrg_status(di);
	di->monitor_ms = di->pdata->monitor_sec * TIMER_MS_COUNTS;
	di->prop_val = POWER_SUPPLY_STATUS_DISCHARGING;
	di->work_mode = MODE_SMOOTH;
	di->boot_base = POWER_ON_SEC_BASE;
	di->chrg_finish_base = 0;
	di->plug_in_base = 0;
	di->plug_out_base = 0;
}

static enum charger_type rk816_bat_init_adc_dc_det(struct rk816_battery *di)
{
	return rk816_bat_get_adc_dc_state(di);
}

static enum charger_type rk816_bat_init_gpio_dc_det(struct rk816_battery *di)
{
	int ret, level;
	unsigned long irq_flags;
	unsigned int dc_det_irq;
	enum charger_type type = NO_DC_CHARGER;

	if (gpio_is_valid(di->pdata->dc_det_pin)) {
		ret = gpio_request(di->pdata->dc_det_pin, "rk816_dc_det");
		if (ret < 0) {
			dev_err(di->dev, "Failed to request gpio %d\n",
				di->pdata->dc_det_pin);
			goto out;
		}

		ret = gpio_direction_input(di->pdata->dc_det_pin);
		if (ret) {
			dev_err(di->dev, "failed to set gpio input\n");
			gpio_free(di->pdata->dc_det_pin);
			goto out;
		}

		level = gpio_get_value(di->pdata->dc_det_pin);
		if (level == di->pdata->dc_det_level)
			type = DC_CHARGER;
		else
			type = NO_DC_CHARGER;

		if (level)
			irq_flags = IRQF_TRIGGER_LOW;
		else
			irq_flags = IRQF_TRIGGER_HIGH;

		dc_det_irq = gpio_to_irq(di->pdata->dc_det_pin);
		ret = request_irq(dc_det_irq, rk816_vbat_dc_det,
				  irq_flags, "rk816_dc_det", di);
		if (ret != 0) {
			dev_err(di->dev, "rk816_dc_det_irq request failed!\n");
			gpio_free(di->pdata->dc_det_pin);
			goto out;
		}

		enable_irq_wake(dc_det_irq);
		di->pdata->dc_gpio_enable = true;
	}
out:
	return type;
}

static enum charger_type rk816_bat_init_dc_det(struct rk816_battery *di)
{
	enum charger_type type;

	if (di->pdata->dc_det_adc)
		type = rk816_bat_init_adc_dc_det(di);
	else
		type = rk816_bat_init_gpio_dc_det(di);

	if (di->pdata->dc_gpio_enable || di->pdata->dc_det_adc) {
		di->dc_monitor_wq = alloc_ordered_workqueue("%s",
					WQ_MEM_RECLAIM | WQ_FREEZABLE,
					"rk816-bat-charger-wq");
		INIT_DELAYED_WORK(&di->dc_delay_work,
				  rk816_bat_dc_delay_work);
		/* adc dc need poll every 1s */
		if (di->pdata->dc_det_adc)
			queue_delayed_work(di->dc_monitor_wq,
					   &di->dc_delay_work,
					   msecs_to_jiffies(1000));
	}

	return type;
}

static void rk816_bat_init_charger(struct rk816_battery *di)
{
	enum charger_type dc_charger, usb_charger;

	dc_charger = rk816_bat_init_dc_det(di);
	usb_charger = rk816_bat_get_acusb_state(di);
	di->usb_in = (usb_charger == USB_CHARGER) ? ONLINE : OFFLINE;
	di->ac_in = (usb_charger == AC_CHARGER) ? ONLINE : OFFLINE;
	di->dc_in = (dc_charger == DC_CHARGER) ? ONLINE : OFFLINE;

	if (di->dc_in)
		rk816_bat_set_chrg_param(di, dc_charger);
	else
		rk816_bat_set_chrg_param(di, usb_charger);

	if (di->pdata->bat_mode == MODE_VIRTUAL)
		rk816_bat_set_chrg_param(di, AC_CHARGER);
}

static int rk816_bat_rtc_sleep_sec(struct rk816_battery *di)
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

static void rk816_bat_init_ts_detect(struct rk816_battery *di)
{
	u8 buf;

	if (!di->pdata->ntc_size)
		return;

	/* Pin func: ts */
	buf = rk816_bat_read(di, RK816_GPIO_IO_POL_REG);
	buf &= ~BIT(2);
	rk816_bat_write(di, RK816_GPIO_IO_POL_REG, buf);

	/* External temperature monitoring */
	buf = rk816_bat_read(di, RK816_TS_CTRL_REG);
	buf &= ~BIT(4);
	rk816_bat_write(di, RK816_TS_CTRL_REG, buf);

	/* ADC_TS_EN */
	buf = rk816_bat_read(di, RK816_ADC_CTRL_REG);
	buf |= BIT(5);
	rk816_bat_write(di, RK816_ADC_CTRL_REG, buf);
}

static void rk816_bat_init_fg(struct rk816_battery *di)
{
	rk816_bat_enable_gauge(di);
	rk816_bat_init_voltage_kb(di);
	rk816_bat_init_poffset(di);
	rk816_bat_set_relax_sample(di);
	rk816_bat_set_ioffset_sample(di);
	rk816_bat_set_ocv_sample(di);
	rk816_bat_init_ts_detect(di);
	rk816_bat_init_rsoc(di);
	rk816_bat_init_coulomb_cap(di, di->nac);
	rk816_bat_init_age_algorithm(di);
	rk816_bat_init_chrg_config(di);
	rk816_bat_init_zero_table(di);
	rk816_bat_init_caltimer(di);
	rk816_bat_smooth_algo_prepare(di);

	di->voltage_avg = rk816_bat_get_avg_voltage(di);
	di->voltage_ocv = rk816_bat_get_ocv_voltage(di);
	di->voltage_relax = rk816_bat_get_relax_voltage(di);
	di->current_avg = rk816_bat_get_avg_current(di);
	di->current_relax = rk816_bat_get_relax_current(di);
	di->remain_cap = rk816_bat_get_coulomb_cap(di);

	rk816_bat_dump_regs(di, 0x99, 0xee);
	DBG("nac=%d cap=%d ov=%d v=%d rv=%d dl=%d rl=%d c=%d\n",
	    di->nac, di->remain_cap, di->voltage_ocv, di->voltage_avg,
	    di->voltage_relax, di->dsoc, di->rsoc, di->current_avg);
}

#ifdef CONFIG_OF
static int rk816_bat_parse_dt(struct rk816_battery *di)
{
	u32 out_value;
	int length, ret;
	size_t size;
	struct device_node *np;
	struct battery_platform_data *pdata;
	struct device *dev = di->dev;
	enum of_gpio_flags flags;

	np = of_find_node_by_name(di->rk816->dev->of_node, "battery");
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
	pdata->sleep_filter_current = DEFAULT_SLP_FILTER_CUR;
	pdata->bat_mode = MODE_BATTARY;
	pdata->max_soc_offset = DEFAULT_MAX_SOC_OFFSET;
	pdata->energy_mode = 0;

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
		dev_err(dev, "max_chrg_current missing!\n");
		return ret;
	}
	pdata->max_chrg_current = out_value;

	ret = of_property_read_u32(np, "max_input_current", &out_value);
	if (ret < 0) {
		dev_err(dev, "max_input_current missing!\n");
		return ret;
	}
	pdata->max_input_current = out_value;

	ret = of_property_read_u32(np, "max_chrg_voltage", &out_value);
	if (ret < 0) {
		dev_err(dev, "max_chrg_voltage missing!\n");
		return ret;
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

	if (!of_find_property(np, "fb_temperature", &length)) {
		pdata->fb_temp = DEFAULT_FB_TEMP;
		dev_err(dev, "fb_temperature missing!\n");
	} else {
		of_property_read_u32(np, "fb_temperature",
				     &pdata->fb_temp);
	}

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

	ret = of_property_read_u32(np, "sleep_filter_current",
				   &pdata->sleep_filter_current);
	if (ret < 0)
		dev_err(dev, "sleep_filter_current missing!\n");

	ret = of_property_read_u32(np, "power_off_thresd", &pdata->pwroff_vol);
	if (ret < 0)
		dev_err(dev, "power_off_thresd missing!\n");

	pdata->dc_det_pin = of_get_named_gpio_flags(np, "dc_det_gpio",
						    0, &flags);
	if (gpio_is_valid(pdata->dc_det_pin))
		pdata->dc_det_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	else
		of_property_read_u32(np, "dc_det_adc", &pdata->dc_det_adc);

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
	    "sleep_filter_current:%d\n"
	    "zero_algorithm_vol:%d\n"
	    "monitor_sec:%d\n"
	    "power_dc2otg:%d\n"
	    "max_soc_offset:%d\n"
	    "virtual_power:%d\n"
	    "pwroff_vol:%d\n"
	    "dc_det_adc:%d\n"
	    "ntc_size=%d\n"
	    "ntc_degree_from:%d\n"
	    "ntc_degree_to:%d\n",
	    pdata->bat_res, pdata->max_input_current,
	    pdata->max_chrg_current, pdata->max_chrg_voltage,
	    pdata->design_capacity, pdata->design_qmax,
	    pdata->sleep_enter_current, pdata->sleep_exit_current,
	    pdata->sleep_filter_current, pdata->zero_algorithm_vol,
	    pdata->monitor_sec, pdata->power_dc2otg,
	    pdata->max_soc_offset, pdata->bat_mode, pdata->pwroff_vol,
	    pdata->dc_det_adc, pdata->ntc_size, pdata->ntc_degree_from,
	    pdata->ntc_degree_from + pdata->ntc_size - 1
	    );

	return 0;
}
#else
static int rk816_bat_parse_dt(struct rk816_battery *di)
{
	return -ENODEV;
}
#endif

static int rk816_battery_probe(struct platform_device *pdev)
{
	struct rk816_battery *di;
	struct rk816 *rk816 = dev_get_drvdata(pdev->dev.parent);
	int ret;

	di = devm_kzalloc(&pdev->dev, sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->rk816 = rk816;
	di->pdev = pdev;
	di->dev = &pdev->dev;
	platform_set_drvdata(pdev, di);

	ret = rk816_bat_parse_dt(di);
	if (ret < 0) {
		dev_err(&pdev->dev, "rk816 battery parse dt failed!\n");
		return ret;
	}

	if (!is_rk816_bat_exist(di)) {
		di->pdata->bat_mode = MODE_VIRTUAL;
		dev_err(&pdev->dev, "no battery, virtual power mode\n");
	}

	ret = rk816_bat_init_power_supply(di);
	if (ret) {
		dev_err(&pdev->dev, "rk816 power supply register failed!\n");
		return ret;
	}

	ret = rk816_bat_init_irqs(di);
	if (ret != 0) {
		dev_err(&pdev->dev, "rk816 bat irq init failed!\n");
		return ret;
	}

	rk816_bat_init_info(di);
	rk816_bat_init_fg(di);
	rk816_bat_init_charger(di);
	rk816_bat_init_sysfs(di);
	rk816_bat_register_fb_notify(di);
	di->bc_detect_nb.notifier_call = rk816_bat_usb_notifier_call;
	rk_bc_detect_notifier_register(&di->bc_detect_nb, &di->charge_otg);
	wake_lock_init(&di->wake_lock, WAKE_LOCK_SUSPEND, "resume_charge");
	di->bat_monitor_wq = alloc_ordered_workqueue("%s",
			WQ_MEM_RECLAIM | WQ_FREEZABLE, "rk816-bat-monitor-wq");
	INIT_DELAYED_WORK(&di->bat_delay_work, rk816_battery_work);
	queue_delayed_work(di->bat_monitor_wq, &di->bat_delay_work,
			   msecs_to_jiffies(TIMER_MS_COUNTS * 5));

	BAT_INFO("driver version %s\n", DRIVER_VERSION);
	return ret;
}

static int rk816_battery_suspend(struct platform_device *dev,
				 pm_message_t state)
{
	struct rk816_battery *di = platform_get_drvdata(dev);

	di->s2r = false;
	di->sleep_chrg_online = rk816_bat_chrg_online(di);
	di->sleep_chrg_status = rk816_bat_get_chrg_status(di);
	di->current_avg = rk816_bat_get_avg_current(di);
	do_gettimeofday(&di->rtc_base);

	/* if not CHARGE_FINISH, reinit chrg_finish_base.
	 * avoid sleep loop in suspend and resume all the time
	 */
	if (di->sleep_chrg_status != CHARGE_FINISH)
		di->chrg_finish_base = get_boot_sec();

	/* avoid: enter suspend from MODE_ZERO: load from heavy to light */
	if ((di->work_mode == MODE_ZERO) &&
	    (di->sleep_chrg_online) && (di->current_avg >= 0)) {
		DBG("suspend: MODE_ZERO exit...\n");
		if (di->sleep_chrg_status == CHARGE_FINISH) {
			di->work_mode = MODE_FINISH;
			di->chrg_finish_base = get_boot_sec();
		} else {
			di->work_mode = MODE_SMOOTH;
			rk816_bat_smooth_algo_prepare(di);
		}
	}

	BAT_INFO("suspend: dl=%d rl=%d c=%d v=%d cap=%d at=%ld st=0x%x ch=%d\n",
		 di->dsoc, di->rsoc, di->current_avg,
		 rk816_bat_get_avg_voltage(di), rk816_bat_get_coulomb_cap(di),
		 di->sleep_dischrg_sec, di->sleep_chrg_status,
		 di->sleep_chrg_online);

	return 0;
}

static int rk816_battery_resume(struct platform_device *dev)
{
	int interval_sec, time_step, pwroff_vol;
	struct rk816_battery *di = platform_get_drvdata(dev);

	di->s2r = true;
	di->voltage_avg = rk816_bat_get_avg_voltage(di);
	di->current_avg = rk816_bat_get_avg_current(di);
	di->voltage_relax = rk816_bat_get_relax_voltage(di);
	di->current_relax = rk816_bat_get_relax_current(di);
	interval_sec = rk816_bat_rtc_sleep_sec(di);
	pwroff_vol = di->pdata->pwroff_vol;

	if (!di->sleep_chrg_online) {
		/* only add up discharge sleep seconds */
		di->sleep_dischrg_sec += interval_sec;
		if (di->voltage_avg <= pwroff_vol + 50)
			time_step = DISCHRG_TIME_STEP1;
		else
			time_step = DISCHRG_TIME_STEP2;
	}

	BAT_INFO("resume: dl=%d rl=%d c=%d v=%d rv=%d "
		 "cap=%d dt=%d at=%ld ch=%d\n",
		 di->dsoc, di->rsoc, di->current_avg, di->voltage_avg,
		 di->voltage_relax, rk816_bat_get_coulomb_cap(di), interval_sec,
		 di->sleep_dischrg_sec, di->sleep_chrg_online);

	/* sleep: enough time and discharge */
	if ((di->sleep_dischrg_sec > time_step) && (!di->sleep_chrg_online)) {
		if (rk816_bat_sleep_dischrg(di))
			di->sleep_dischrg_sec = 0;
	}

	if ((!rk816_bat_chrg_online(di) && di->voltage_avg <= pwroff_vol) ||
	    rk816_bat_chrg_online(di))
		wake_lock_timeout(&di->wake_lock, 5 * HZ);

	return 0;
}

static void rk816_battery_shutdown(struct platform_device *dev)
{
	u8 cnt = 0;
	struct rk816_battery *di = platform_get_drvdata(dev);

	if (di->pdata->dc_gpio_enable || di->pdata->dc_det_adc)
		cancel_delayed_work_sync(&di->dc_delay_work);
	cancel_delayed_work_sync(&di->bat_delay_work);
	rk816_bat_unregister_fb_notify(di);
	rk_bc_detect_notifier_unregister(&di->bc_detect_nb);
	del_timer(&di->caltimer);
	rk816_bat_set_otg_state(di, USB_OTG_POWER_OFF);

	if (base2sec(di->boot_base) < REBOOT_PERIOD_SEC)
		cnt = rk816_bat_check_reboot(di);
	else
		rk816_bat_save_reboot_cnt(di, 0);

	BAT_INFO("shutdown: dl=%d rl=%d c=%d v=%d cap=%d f=%d ch=%d n=%d\n",
		 di->dsoc, di->rsoc, di->current_avg, di->voltage_avg,
		 di->remain_cap, di->fcc, rk816_bat_chrg_online(di), cnt);
}

static struct platform_driver rk816_battery_driver = {
	.probe = rk816_battery_probe,
	.suspend = rk816_battery_suspend,
	.resume = rk816_battery_resume,
	.shutdown = rk816_battery_shutdown,
	.driver = {
		.name	= "rk816-battery",
		.owner	= THIS_MODULE,
	},
};

static int __init battery_init(void)
{
	return platform_driver_register(&rk816_battery_driver);
}
fs_initcall_sync(battery_init);

static void __exit battery_exit(void)
{
	platform_driver_unregister(&rk816_battery_driver);
}
module_exit(battery_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rk816-battery");
MODULE_AUTHOR("ROCKCHIP");
