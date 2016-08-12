/*
 *  drivers/input/touchscreen/rockchip_gslX680_px5.c
 *
 *  Copyright (c) 2012 Shanghai Basewin
 *  Guan Yuwei<guanyuwei@basewin.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/gpio.h>
#include <asm/irq.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>
#include "tp_suspend.h"
#include "rockchip_gslX680_px5.h"
#include "gsl_point_id.h"
#include <linux/of_gpio.h>
#include <linux/wakelock.h>

#include <dt-bindings/touchscreen_type.h>

#define REPORT_DATA_ANDROID_4_0
/*
//#define GSL_DEBUG
//#define HAVE_TOUCH_KEY
//#define SLEEP_CLEAR_POINT
//#define FILTER_POINT
*/

#ifdef FILTER_POINT
#define FILTER_MAX	9
#endif

#define GSLX680_I2C_NAME	"gslx680"
#define GSLX680_I2C_ADDR	0x40
#define GSL_DATA_REG		0x80
#define GSL_STATUS_REG		0xe0
#define GSL_PAGE_REG		0xf0

#define TPD_PROC_DEBUG
#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31

static u8 gsl_proc_flag;
#endif

#define GSL_MONITOR
#define PRESS_MAX		255
#define MAX_FINGERS		10
#define MAX_CONTACTS		10
#define DMA_TRANS_LEN		0x20
#ifdef GSL_MONITOR
static struct workqueue_struct *gsl_monitor_workqueue;
static u8 int_1st[4] = { 0 };
static u8 int_2nd[4] = { 0 };
static char b0_counter;
static char b4_counter;
static char bc_counter;
static char i2c_lock_flag;
#endif

#define WRITE_I2C_SPEED 350000
#define I2C_SPEED  200000
#define CLOSE_TP_POWER   0

#ifdef HAVE_TOUCH_KEY
static u16 key;
static int key_state_flag;
struct key_data {
	u16 key;
	u16 x_min;
	u16 x_max;
	u16 y_min;
	u16 y_max;
};

const u16 key_array[] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU,
	KEY_SEARCH,
};

#define MAX_KEY_NUM (sizeof(key_array)/sizeof(key_array[0]))

struct key_data gsl_key_data[MAX_KEY_NUM] = {
	{KEY_BACK, 550, 650, 1400, 1600},
	{KEY_HOME, 350, 450, 1400, 1600},
	{KEY_MENU, 150, 250, 1400, 1600},
	{KEY_SEARCH, 2048, 2048, 2048, 2048},
};
#endif

struct gsl_ts_data {
	u8 x_index;
	u8 y_index;
	u8 z_index;
	u8 id_index;
	u8 touch_index;
	u8 data_reg;
	u8 status_reg;
	u8 data_size;
	u8 touch_bytes;
	u8 update_data;
	u8 touch_meta_data;
	u8 finger_size;
};

static struct gsl_ts_data devices[] = {
	{
	 .x_index = 6,
	 .y_index = 4,
	 .z_index = 5,
	 .id_index = 7,
	 .data_reg = GSL_DATA_REG,
	 .status_reg = GSL_STATUS_REG,
	 .update_data = 0x4,
	 .touch_bytes = 4,
	 .touch_meta_data = 4,
	 .finger_size = 70,
	 },
};

struct gsl_ts {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	struct workqueue_struct *wq;
	struct gsl_ts_data *dd;
	u8 *touch_data;
	u8 device_id;
	int irq_gpio;
	int reset_gpio;
	int flag_irq_is_disable;
	int tp_type;
	spinlock_t irq_lock;	/* lock for the whole structure */
	struct delayed_work gsl_monitor_work;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

	struct tp_device tp;
};

#ifdef GSL_DEBUG
#define print_info(fmt, args...) pr_info(fmt, ##args)
#else
#define print_info(fmt, args...)
#endif

static u32 id_sign[MAX_CONTACTS + 1] = { 0 };
static u8 id_state_flag[MAX_CONTACTS + 1] = { 0 };
static u8 id_state_old_flag[MAX_CONTACTS + 1] = { 0 };
static u16 x_old[MAX_CONTACTS + 1] = { 0 };
static u16 y_old[MAX_CONTACTS + 1] = { 0 };

static u16 x_new;
static u16 y_new;

static const struct fw_data *ptr_fw = GSLX680_FW_INCH7;
u32 ptr_fw_len;
u16 SCREEN_MAX_X;
u16 SCREEN_MAX_Y;
unsigned int *gsl_config_data_id = gsl_config_data_id_inch7;

static int gslx680_init(struct gsl_ts *ts)
{
	struct device_node *np = ts->client->dev.of_node;

	ts->irq_gpio = of_get_named_gpio_flags(np, "touch-gpio", 0, NULL);
	ts->reset_gpio = of_get_named_gpio_flags(np, "reset-gpio", 0, NULL);
	of_property_read_u32(np, "touchscreen-type", &ts->tp_type);

	switch (ts->tp_type) {
	case TOUCHSCREEN_INCH_7:
		ptr_fw = GSLX680_FW_INCH7;
		ptr_fw_len = ARRAY_SIZE(GSLX680_FW_INCH7);
		gsl_config_data_id = gsl_config_data_id_inch7;
		SCREEN_MAX_X = SCREEN_MAX_X_INCH7;
		SCREEN_MAX_Y = SCREEN_MAX_Y_INCH7;

		break;

	case TOUCHSCREEN_INCH_8:
		ptr_fw = GSLX680_FW_INCH8;
		ptr_fw_len = ARRAY_SIZE(GSLX680_FW_INCH8);
		gsl_config_data_id = gsl_config_data_id_inch8;
		SCREEN_MAX_X = SCREEN_MAX_X_INCH8;
		SCREEN_MAX_Y = SCREEN_MAX_Y_INCH8;

		break;

	case TOUCHSCREEN_INCH_9:
		ptr_fw = GSLX680_FW_INCH9;
		ptr_fw_len = ARRAY_SIZE(GSLX680_FW_INCH9);
		gsl_config_data_id = gsl_config_data_id_inch9;
		SCREEN_MAX_X = SCREEN_MAX_X_INCH9;
		SCREEN_MAX_Y = SCREEN_MAX_Y_INCH9;

		break;

	case TOUCHSCREEN_INCH_10:
		ptr_fw = GSLX680_FW_INCH10;
		ptr_fw_len = ARRAY_SIZE(GSLX680_FW_INCH10);
		gsl_config_data_id = gsl_config_data_id_inch10;
		SCREEN_MAX_X = SCREEN_MAX_X_INCH10;
		SCREEN_MAX_Y = SCREEN_MAX_Y_INCH10;

		break;

	default:
		ptr_fw = GSLX680_FW_INCH7;
		ptr_fw_len = ARRAY_SIZE(GSLX680_FW_INCH7);
		gsl_config_data_id = gsl_config_data_id_inch7;
		SCREEN_MAX_X = SCREEN_MAX_X_INCH7;
		SCREEN_MAX_Y = SCREEN_MAX_Y_INCH7;

		break;
	}

	if (gpio_request(ts->reset_gpio, NULL) != 0) {
		gpio_free(ts->reset_gpio);
		dev_err(&ts->client->dev, "gslx680 request reset gpio error\n");
		return -EIO;
	}

	if (gpio_request(ts->irq_gpio, NULL) != 0) {
		gpio_free(ts->irq_gpio);
		dev_err(&ts->client->dev, "gslx680 request irq gpio error\n");
		return -EIO;
	}

	gpio_direction_output(ts->reset_gpio, 1);
	gpio_set_value(ts->reset_gpio, 1);

	return 0;
}

static int gslx680_shutdown_low(struct gsl_ts *ts)
{
	gpio_direction_output(ts->reset_gpio, 0);
	gpio_set_value(ts->reset_gpio, 0);

	return 0;
}

static int gslx680_shutdown_high(struct gsl_ts *ts)
{
	gpio_direction_output(ts->reset_gpio, 1);
	gpio_set_value(ts->reset_gpio, 1);

	return 0;
}

static inline u16 join_bytes(u8 a, u8 b)
{
	u16 ab = 0;

	ab = ab | a;
	ab = ab << 8 | b;
	return ab;
}

static u32 gsl_write_interface(struct i2c_client *client, const u8 reg,
			       u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];

	buf[0] = reg;

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = buf;
	xfer_msg[0].scl_rate = WRITE_I2C_SPEED;

	return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}

static int gsl_ts_write(struct i2c_client *client, u8 addr, u8 *pdata,
			int datalen)
{
	int ret = 0;
	u8 tmp_buf[128];
	unsigned int bytelen = 0;

	if (datalen > 125) {
		dev_err(&client->dev, "%s too big datalen = %d!\n",
			__func__, datalen);
		return -1;
	}

	tmp_buf[0] = addr;
	bytelen++;

	if (datalen != 0 && pdata != NULL) {
		memcpy(&tmp_buf[bytelen], pdata, datalen);
		bytelen += datalen;
	}

	ret = i2c_master_send(client, tmp_buf, bytelen);
	return ret;
}

static int gsl_ts_read(struct i2c_client *client, u8 addr, u8 *pdata,
		       unsigned int datalen)
{
	int ret = 0;

	if (datalen > 126) {
		dev_err(&client->dev, "%s too big datalen = %d!\n",
			__func__, datalen);
		return -1;
	}

	ret = gsl_ts_write(client, addr, NULL, 0);
	if (ret < 0) {
		dev_err(&client->dev, "%s set data address fail!\n", __func__);
		return ret;
	}

	return i2c_master_recv(client, pdata, datalen);
}

static inline void fw2buf(u8 *buf, const u32 *fw)
{
	u32 *u32_buf = (int *)buf;

	*u32_buf = *fw;
}

static void gsl_load_fw(struct i2c_client *client)
{
	u8 buf[DMA_TRANS_LEN * 4 + 1] = { 0 };
	u8 send_flag = 1;
	u8 *cur = buf + 1;
	u32 source_line = 0;
	u32 source_len;

	dev_info(&client->dev, "=============gsl_load_fw start==============\n");

	source_len = ptr_fw_len;

	for (source_line = 0; source_line < source_len; source_line++) {
		/* init page trans, set the page val */
		if (GSL_PAGE_REG == ptr_fw[source_line].offset) {
			fw2buf(cur, &ptr_fw[source_line].val);
			gsl_write_interface(client, GSL_PAGE_REG, buf, 4);
			send_flag = 1;
		} else {
			if (1 ==
			    send_flag % (DMA_TRANS_LEN <
					 0x20 ? DMA_TRANS_LEN : 0x20))
				buf[0] = (u8) ptr_fw[source_line].offset;

			fw2buf(cur, &ptr_fw[source_line].val);
			cur += 4;

			if (0 ==
			    send_flag % (DMA_TRANS_LEN <
					 0x20 ? DMA_TRANS_LEN : 0x20)) {
				gsl_write_interface(client, buf[0], buf,
						    cur - buf - 1);
				cur = buf + 1;
			}

			send_flag++;
		}
	}

	dev_info(&client->dev, "=============gsl_load_fw end==============\n");
}

static int test_i2c(struct i2c_client *client)
{
	u8 read_buf = 0;
	u8 write_buf = 0x12;
	int ret, rc = 1;

	ret = gsl_ts_read(client, 0xf0, &read_buf, sizeof(read_buf));
	if (ret < 0)
		rc--;
	else
		dev_err(&client->dev, "gsl I read reg 0xf0 is %x\n", read_buf);

	usleep_range(1500, 2000);
	ret = gsl_ts_write(client, 0xf0, &write_buf, sizeof(write_buf));
	if (ret >= 0)
		dev_err(&client->dev, "gsl I write reg 0xf0 0x12\n");

	usleep_range(1500, 2000);
	ret = gsl_ts_read(client, 0xf0, &read_buf, sizeof(read_buf));
	if (ret < 0)
		rc--;
	else
		dev_err(&client->dev, "gsl I read reg 0xf0 is 0x%x\n",
			read_buf);

	return rc;
}

static void startup_chip(struct i2c_client *client)
{
	u8 tmp = 0x00;

	dev_info(&client->dev, "gsl startup_chip\n");

#ifdef GSL_NOID_VERSION
	gsl_DataInit(gsl_config_data_id);
#endif
	gsl_ts_write(client, 0xe0, &tmp, 1);
	usleep_range(9500, 10000);
}

static void reset_chip(struct i2c_client *client)
{
	u8 tmp = 0x88;
	u8 buf[4] = { 0x00 };

	dev_err(&client->dev, "gsl reset_chip\n");

	gsl_ts_write(client, 0xe0, &tmp, sizeof(tmp));
	usleep_range(19500, 20000);
	tmp = 0x04;
	gsl_ts_write(client, 0xe4, &tmp, sizeof(tmp));
	usleep_range(9500, 10000);
	gsl_ts_write(client, 0xbc, buf, sizeof(buf));
	usleep_range(9500, 10000);
}

static void clr_reg(struct i2c_client *client)
{
	u8 write_buf[4] = { 0 };

	write_buf[0] = 0x88;
	gsl_ts_write(client, 0xe0, &write_buf[0], 1);
	usleep_range(19500, 20000);
	write_buf[0] = 0x03;
	gsl_ts_write(client, 0x80, &write_buf[0], 1);
	usleep_range(4500, 5000);
	write_buf[0] = 0x04;
	gsl_ts_write(client, 0xe4, &write_buf[0], 1);
	usleep_range(4500, 5000);
	write_buf[0] = 0x00;
	gsl_ts_write(client, 0xe0, &write_buf[0], 1);
	usleep_range(19500, 20000);
}

static void init_chip(struct i2c_client *client, struct gsl_ts *ts)
{
	int rc;

	dev_info(&client->dev, "gsl init_chip\n");

	gslx680_shutdown_low(ts);
	msleep(20);
	gslx680_shutdown_high(ts);
	msleep(20);
	rc = test_i2c(client);
	if (rc < 0) {
		dev_err(&client->dev, "------gslx680 test_i2c error------\n");
		return;
	}
	clr_reg(client);
	reset_chip(client);
	gsl_load_fw(client);
	startup_chip(client);
	reset_chip(client);
	startup_chip(client);
}

static void check_mem_data(struct i2c_client *client, struct gsl_ts *ts)
{
	u8 read_buf[4] = { 0 };

	msleep(30);
	gsl_ts_read(client, 0xb0, read_buf, sizeof(read_buf));

	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a ||
	    read_buf[1] != 0x5a || read_buf[0] != 0x5a) {
		init_chip(client, ts);
	}
}

#ifdef FILTER_POINT
static void filter_point(u16 x, u16 y, u8 id)
{
	u16 x_err = 0;
	u16 y_err = 0;
	u16 filter_step_x = 0, filter_step_y = 0;

	id_sign[id] = id_sign[id] + 1;
	if (id_sign[id] == 1) {
		x_old[id] = x;
		y_old[id] = y;
	}

	x_err = x > x_old[id] ? (x - x_old[id]) : (x_old[id] - x);
	y_err = y > y_old[id] ? (y - y_old[id]) : (y_old[id] - y);

	if ((x_err > FILTER_MAX && y_err > FILTER_MAX / 3) ||
	    (x_err > FILTER_MAX / 3 && y_err > FILTER_MAX)) {
		filter_step_x = x_err;
		filter_step_y = y_err;
	} else {
		if (x_err > FILTER_MAX)
			filter_step_x = x_err;
		if (y_err > FILTER_MAX)
			filter_step_y = y_err;
	}

	if (x_err <= 2 * FILTER_MAX && y_err <= 2 * FILTER_MAX) {
		filter_step_x >>= 2;
		filter_step_y >>= 2;
	} else if (x_err <= 3 * FILTER_MAX && y_err <= 3 * FILTER_MAX) {
		filter_step_x >>= 1;
		filter_step_y >>= 1;
	} else if (x_err <= 4 * FILTER_MAX && y_err <= 4 * FILTER_MAX) {
		filter_step_x = filter_step_x * 3 / 4;
		filter_step_y = filter_step_y * 3 / 4;
	}

	x_new =
	    x >
	    x_old[id] ? (x_old[id] + filter_step_x) : (x_old[id] -
						       filter_step_x);
	y_new =
	    y >
	    y_old[id] ? (y_old[id] + filter_step_y) : (y_old[id] -
						       filter_step_y);

	x_old[id] = x_new;
	y_old[id] = y_new;
}
#else
static void record_point(u16 x, u16 y, u8 id)
{
	u16 x_err = 0;
	u16 y_err = 0;

	id_sign[id] = id_sign[id] + 1;

	if (id_sign[id] == 1) {
		x_old[id] = x;
		y_old[id] = y;
	}

	x = (x_old[id] + x) / 2;
	y = (y_old[id] + y) / 2;

	if (x > x_old[id])
		x_err = x - x_old[id];
	else
		x_err = x_old[id] - x;

	if (y > y_old[id])
		y_err = y - y_old[id];
	else
		y_err = y_old[id] - y;

	if ((x_err > 3 && y_err > 1) || (x_err > 1 && y_err > 3)) {
		x_new = x;
		x_old[id] = x;
		y_new = y;
		y_old[id] = y;
	} else {
		if (x_err > 3) {
			x_new = x;
			x_old[id] = x;
		} else {
			x_new = x_old[id];
		}
		if (y_err > 3) {
			y_new = y;
			y_old[id] = y;
		} else {
			y_new = y_old[id];
		}
	}

	if (id_sign[id] == 1) {
		x_new = x_old[id];
		y_new = y_old[id];
	}
}
#endif

#ifdef HAVE_TOUCH_KEY
static void report_key(struct gsl_ts *ts, u16 x, u16 y)
{
	u16 i = 0;

	for (i = 0; i < MAX_KEY_NUM; i++) {
		if ((gsl_key_data[i].x_min < x) &&
		    (x < gsl_key_data[i].x_max) &&
		    (gsl_key_data[i].y_min < y) &&
		    (y < gsl_key_data[i].y_max)) {
			key = gsl_key_data[i].key;
			input_report_key(ts->input, key, 1);
			input_sync(ts->input);
			key_state_flag = 1;
			break;
		}
	}
}
#endif

static void report_data(struct gsl_ts *ts, u16 x, u16 y, u8 pressure, u8 id)
{
	swap(x, y);
	x = SCREEN_MAX_X - x;

	if (x > SCREEN_MAX_X || y > SCREEN_MAX_Y) {
#ifdef HAVE_TOUCH_KEY
		report_key(ts, x, y);
#endif
		return;
	}
#ifdef REPORT_DATA_ANDROID_4_0
	input_mt_slot(ts->input, id);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, 1);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, pressure);
#ifdef X_POL
	input_report_abs(ts->input, ABS_MT_POSITION_X, SCREEN_MAX_X - x);
#else
	if (ts->tp_type == TOUCHSCREEN_INCH_10)
		input_report_abs(ts->input, ABS_MT_POSITION_X,
				 SCREEN_MAX_X - x);
	else
		input_report_abs(ts->input, ABS_MT_POSITION_X, x);
#endif
#ifdef Y_POL
	input_report_abs(ts->input, ABS_MT_POSITION_Y, (SCREEN_MAX_Y - y));
#else
	if (ts->tp_type == TOUCHSCREEN_INCH_10)
		input_report_abs(ts->input, ABS_MT_POSITION_Y,
				 SCREEN_MAX_Y - y);
	else
		input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
#endif
	input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 1);
#else
	input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, pressure);
	input_report_abs(ts->input, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 1);
	input_mt_sync(ts->input);
#endif
}

void ts_irq_disable(struct gsl_ts *ts)
{
	unsigned long irqflags;

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->flag_irq_is_disable) {
		disable_irq_nosync(ts->client->irq);
		ts->flag_irq_is_disable = 1;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

void ts_irq_enable(struct gsl_ts *ts)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->flag_irq_is_disable) {
		enable_irq(ts->client->irq);
		ts->flag_irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

static void gslx680_ts_worker(struct work_struct *work)
{
	int rc, i;
	u8 id, touches;
	u16 x, y;

#ifdef GSL_NOID_VERSION
	u32 tmp1;
	u8 buf[4] = { 0 };
	struct gsl_touch_info cinfo;
#endif

	struct gsl_ts *ts = container_of(work, struct gsl_ts, work);

#ifdef TPD_PROC_DEBUG
	if (gsl_proc_flag == 1)
		goto schedule;
#endif

#ifdef GSL_MONITOR
	if (i2c_lock_flag != 0)
		goto i2c_lock_schedule;
	else
		i2c_lock_flag = 1;
#endif

	rc = gsl_ts_read(ts->client, 0x80, ts->touch_data, ts->dd->data_size);
	if (rc < 0) {
		dev_err(&ts->client->dev, "read failed\n");
		goto schedule;
	}

	touches = ts->touch_data[ts->dd->touch_index];
#ifdef GSL_NOID_VERSION
	cinfo.finger_num = touches;
	for (i = 0; i < (touches < MAX_CONTACTS ? touches : MAX_CONTACTS);
	     i++) {
		cinfo.x[i] =
		    join_bytes((ts->
				touch_data[ts->dd->x_index + 4 * i + 1] & 0xf),
			       ts->touch_data[ts->dd->x_index + 4 * i]);
		cinfo.y[i] =
		    join_bytes(ts->touch_data[ts->dd->y_index + 4 * i + 1],
			       ts->touch_data[ts->dd->y_index + 4 * i]);
		cinfo.id[i] =
		    ((ts->touch_data[ts->dd->x_index + 4 * i + 1] & 0xf0) >> 4);
	}
	cinfo.finger_num = (ts->touch_data[3] << 24) | (ts->touch_data[2] << 16)
	    | (ts->touch_data[1] << 8) | (ts->touch_data[0]);
	gsl_alg_id_main(&cinfo);
	tmp1 = gsl_mask_tiaoping();
	if (tmp1 > 0 && tmp1 < 0xffffffff) {
		buf[0] = 0xa;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		gsl_ts_write(ts->client, 0xf0, buf, 4);
		buf[0] = (u8) (tmp1 & 0xff);
		buf[1] = (u8) ((tmp1 >> 8) & 0xff);
		buf[2] = (u8) ((tmp1 >> 16) & 0xff);
		buf[3] = (u8) ((tmp1 >> 24) & 0xff);
		print_info("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",
			   tmp1, buf[0], buf[1], buf[2], buf[3]);
		gsl_ts_write(ts->client, 0x8, buf, 4);
	}
	touches = cinfo.finger_num;
#endif

	for (i = 1; i <= MAX_CONTACTS; i++) {
		if (touches == 0)
			id_sign[i] = 0;
		id_state_flag[i] = 0;
	}
	for (i = 0; i < (touches > MAX_FINGERS ? MAX_FINGERS : touches); i++) {
#ifdef GSL_NOID_VERSION
		id = cinfo.id[i];
		x = cinfo.x[i];
		y = cinfo.y[i];
#else
		x = join_bytes((ts->
				touch_data[ts->dd->x_index + 4 * i + 1] & 0xf),
			       ts->touch_data[ts->dd->x_index + 4 * i]);
		y = join_bytes(ts->touch_data[ts->dd->y_index + 4 * i + 1],
			       ts->touch_data[ts->dd->y_index + 4 * i]);
		id = ts->touch_data[ts->dd->id_index + 4 * i] >> 4;
#endif

		if (1 <= id && id <= MAX_CONTACTS) {
#ifdef FILTER_POINT
			filter_point(x, y, id);
#else
			record_point(x, y, id);
#endif
			report_data(ts, x_new, y_new, 10, id);
			id_state_flag[id] = 1;
		}
	}
	for (i = 1; i <= MAX_CONTACTS; i++) {
		if ((0 == touches) ||
		    ((0 != id_state_old_flag[i]) &&
		    (0 == id_state_flag[i]))) {
#ifdef REPORT_DATA_ANDROID_4_0
			input_mt_slot(ts->input, i);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER,
						   false);
#endif
			id_sign[i] = 0;
		}
		id_state_old_flag[i] = id_state_flag[i];
	}

	if (0 == touches) {
#ifndef REPORT_DATA_ANDROID_4_0
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(ts->input);
#endif
#ifdef HAVE_TOUCH_KEY
		if (key_state_flag) {
			input_report_key(ts->input, key, 0);
			input_sync(ts->input);
			key_state_flag = 0;
		}
#endif
	}

	input_sync(ts->input);

schedule:
#ifdef GSL_MONITOR
	i2c_lock_flag = 0;
i2c_lock_schedule:
#endif
	ts_irq_enable(ts);
}

#ifdef GSL_MONITOR
static void gsl_monitor_worker(struct work_struct *work)
{
	u8 read_buf[4] = { 0 };
	char init_chip_flag = 0;

	struct gsl_ts *ts =
	    container_of(work, struct gsl_ts, gsl_monitor_work.work);
	if (i2c_lock_flag != 0)
		goto queue_monitor_work;
	else
		i2c_lock_flag = 1;

	gsl_ts_read(ts->client, 0xb0, read_buf, 4);
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a ||
	    read_buf[1] != 0x5a || read_buf[0] != 0x5a)
		b0_counter++;
	else
		b0_counter = 0;

	if (b0_counter > 1) {
		init_chip_flag = 1;
		b0_counter = 0;
		goto queue_monitor_init_chip;
	}

	gsl_ts_read(ts->client, 0xb4, read_buf, 4);
	int_2nd[3] = int_1st[3];
	int_2nd[2] = int_1st[2];
	int_2nd[1] = int_1st[1];
	int_2nd[0] = int_1st[0];
	int_1st[3] = read_buf[3];
	int_1st[2] = read_buf[2];
	int_1st[1] = read_buf[1];
	int_1st[0] = read_buf[0];

	if (int_1st[3] == int_2nd[3] && int_1st[2] == int_2nd[2] &&
	    int_1st[1] == int_2nd[1] && int_1st[0] == int_2nd[0])
		b4_counter++;
	else
		b4_counter = 0;

	if (b4_counter > 2) {
		init_chip_flag = 1;
		b4_counter = 0;
		goto queue_monitor_init_chip;
	}

	gsl_ts_read(ts->client, 0xbc, read_buf, 4);
	if (read_buf[3] != 0 || read_buf[2] != 0 ||
	    read_buf[1] != 0 || read_buf[0] != 0)
		bc_counter++;
	else
		bc_counter = 0;
	if (bc_counter > 1) {
		init_chip_flag = 1;
		bc_counter = 0;
	}

queue_monitor_init_chip:
	if (init_chip_flag)
		init_chip(ts->client, ts);

	i2c_lock_flag = 0;

queue_monitor_work:
	queue_delayed_work(gsl_monitor_workqueue, &ts->gsl_monitor_work, 100);
}
#endif

static irqreturn_t gsl_ts_irq(int irq, void *dev_id)
{
	struct gsl_ts *ts = (struct gsl_ts *)dev_id;

	ts_irq_disable(ts);

	if (!work_pending(&ts->work))
		queue_work(ts->wq, &ts->work);

	return IRQ_HANDLED;
}

static int gslx680_ts_init(struct i2c_client *client, struct gsl_ts *ts)
{
	struct input_dev *input_device;
	int rc = 0;

	ts->dd = &devices[ts->device_id];

	if (ts->device_id == 0) {
		ts->dd->data_size =
		    MAX_FINGERS * ts->dd->touch_bytes + ts->dd->touch_meta_data;
		ts->dd->touch_index = 0;
	}

	ts->touch_data =
	    devm_kzalloc(&client->dev, ts->dd->data_size, GFP_KERNEL);
	if (!ts->touch_data) {
		dev_err(&client->dev, "%s: Unable to allocate memory\n",
			__func__);
		return -ENOMEM;
	}

	input_device = devm_input_allocate_device(&ts->client->dev);
	if (!input_device)
		return -ENOMEM;

	ts->input = input_device;
	input_device->name = GSLX680_I2C_NAME;
	input_device->id.bustype = BUS_I2C;
	input_device->dev.parent = &client->dev;
	input_set_drvdata(input_device, ts);

#ifdef REPORT_DATA_ANDROID_4_0
	__set_bit(EV_ABS, input_device->evbit);
	__set_bit(EV_KEY, input_device->evbit);
	__set_bit(EV_REP, input_device->evbit);
	__set_bit(EV_SYN, input_device->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_device->propbit);
	__set_bit(MT_TOOL_FINGER, input_device->keybit);
	input_mt_init_slots(input_device, (MAX_CONTACTS + 1), 0);
#else
	input_set_abs_params(input_device, ABS_MT_TRACKING_ID, 0,
			     (MAX_CONTACTS + 1), 0, 0);
	set_bit(EV_ABS, input_device->evbit);
	set_bit(EV_KEY, input_device->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_device->propbit);
	input_device->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

#ifdef HAVE_TOUCH_KEY
	int i = 0;

	for (i = 0; i < MAX_KEY_NUM; i++)
		set_bit(key_array[i], input_device->keybit);
#endif

	set_bit(ABS_MT_POSITION_X, input_device->absbit);
	set_bit(ABS_MT_POSITION_Y, input_device->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input_device->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_device->absbit);

	input_set_abs_params(input_device, ABS_MT_POSITION_X, 0, SCREEN_MAX_X,
			     0, 0);
	input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y,
			     0, 0);
	input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0,
			     0);
	input_set_abs_params(input_device, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);


	ts->wq = create_singlethread_workqueue("kworkqueue_ts");
	if (!ts->wq) {
		dev_err(&client->dev, "gsl Could not create workqueue\n");
		goto error_wq_create;
	}
	flush_workqueue(ts->wq);

	INIT_WORK(&ts->work, gslx680_ts_worker);

	rc = input_register_device(input_device);
	if (rc)
		goto error_unreg_device;

	return 0;

error_unreg_device:
	destroy_workqueue(ts->wq);
error_wq_create:
	input_free_device(input_device);
	return rc;
}

static int gsl_ts_suspend(struct device *dev)
{
	struct gsl_ts *ts = dev_get_drvdata(dev);

#ifdef GSL_MONITOR
	cancel_delayed_work_sync(&ts->gsl_monitor_work);
#endif
	ts_irq_disable(ts);

	cancel_work_sync(&ts->work);
	gslx680_shutdown_low(ts);

#ifdef SLEEP_CLEAR_POINT
	usleep_range(9500, 10000);
#ifdef REPORT_DATA_ANDROID_4_0
	for (i = 1; i <= MAX_CONTACTS; i++) {
		input_mt_slot(ts->input, i);
		input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	}
#else
	input_mt_sync(ts->input);
#endif
	input_sync(ts->input);
	usleep_range(9500, 10000);
	report_data(ts, 1, 1, 10, 1);
	input_sync(ts->input);
#endif

	return 0;
}

static int gsl_ts_resume(struct device *dev)
{
	struct gsl_ts *ts = dev_get_drvdata(dev);

	gslx680_shutdown_high(ts);
	msleep(20);
	reset_chip(ts->client);
	startup_chip(ts->client);
	check_mem_data(ts->client, ts);

#ifdef SLEEP_CLEAR_POINT
#ifdef REPORT_DATA_ANDROID_4_0
	for (i = 1; i <= MAX_CONTACTS; i++) {
		input_mt_slot(ts->input, i);
		input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	}
#else
	input_mt_sync(ts->input);
#endif
	input_sync(ts->input);
#endif
#ifdef GSL_MONITOR
	queue_delayed_work(gsl_monitor_workqueue, &ts->gsl_monitor_work, 300);
#endif
	ts_irq_enable(ts);

	return 0;
}

static int gsl_ts_early_suspend(struct tp_device *tp_d)
{
	struct gsl_ts *ts = container_of(tp_d, struct gsl_ts, tp);

	gsl_ts_suspend(&ts->client->dev);
	return 0;
}

static int gsl_ts_late_resume(struct tp_device *tp_d)
{
	struct gsl_ts *ts = container_of(tp_d, struct gsl_ts, tp);

	gsl_ts_resume(&ts->client->dev);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static void gsl_ts_early_suspend(struct early_suspend *h)
{
	struct gsl_ts *ts = container_of(h, struct gsl_ts, early_suspend);

	gsl_ts_suspend(&ts->client->dev);
}

static void gsl_ts_late_resume(struct early_suspend *h)
{
	struct gsl_ts *ts = container_of(h, struct gsl_ts, early_suspend);

	gsl_ts_resume(&ts->client->dev);
}
#endif

static int gsl_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct gsl_ts *ts;
	int rc;

	dev_info(&client->dev, "GSLX680 Enter %s\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "gsl I2C functionality not supported\n");
		return -ENODEV;
	}

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->tp.tp_suspend = gsl_ts_early_suspend;
	ts->tp.tp_resume = gsl_ts_late_resume;
	tp_register_fb(&ts->tp);

	ts->client = client;
	i2c_set_clientdata(client, ts);
	gslx680_init(ts);

	rc = gslx680_ts_init(client, ts);
	if (rc < 0) {
		dev_err(&client->dev, "gsl GSLX680 init failed\n");
		goto error_mutex_destroy;
	}
	init_chip(ts->client, ts);
	check_mem_data(ts->client, ts);
	spin_lock_init(&ts->irq_lock);

	client->irq = gpio_to_irq(ts->irq_gpio);
	rc = request_irq(client->irq, gsl_ts_irq, IRQF_TRIGGER_RISING,
			 client->name, ts);
	if (rc < 0) {
		dev_err(&client->dev, "gsl_probe: request irq failed\n");
		goto error_req_irq_fail;
	}

	dev_info(&client->dev, "%s:client->irq=%d, ts->irq=%d\n", __func__,
		 client->irq, ts->irq_gpio);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = gsl_ts_early_suspend;
	ts->early_suspend.resume = gsl_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef GSL_MONITOR
	INIT_DELAYED_WORK(&ts->gsl_monitor_work, gsl_monitor_worker);
	gsl_monitor_workqueue =
	    create_singlethread_workqueue("gsl_monitor_workqueue");
	queue_delayed_work(gsl_monitor_workqueue, &ts->gsl_monitor_work, 1000);
#endif

	return 0;

error_req_irq_fail:
	free_irq(ts->client->irq, ts);

error_mutex_destroy:
	input_free_device(ts->input);
	return rc;
}

static int gsl_ts_remove(struct i2c_client *client)
{
	struct gsl_ts *ts = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

#ifdef GSL_MONITOR
	cancel_delayed_work_sync(&ts->gsl_monitor_work);
	destroy_workqueue(gsl_monitor_workqueue);
#endif

	device_init_wakeup(&client->dev, 0);
	cancel_work_sync(&ts->work);
	free_irq(ts->client->irq, ts);
	destroy_workqueue(ts->wq);
	input_unregister_device(ts->input);

	return 0;
}

static struct of_device_id gsl_ts_ids[] = {
	{.compatible = "GSL,GSLX680"},
	{}
};

static const struct i2c_device_id gsl_ts_id[] = {
	{GSLX680_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, gsl_ts_id);

static struct i2c_driver gsl_ts_driver = {
	.driver = {
		   .name = GSLX680_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(gsl_ts_ids),
		   },
	.probe = gsl_ts_probe,
	.remove = gsl_ts_remove,
	.id_table = gsl_ts_id,
};

static int __init gsl_ts_init(void)
{
	int ret;

	ret = i2c_add_driver(&gsl_ts_driver);
	return ret;
}

static void __exit gsl_ts_exit(void)
{
	i2c_del_driver(&gsl_ts_driver);
}

module_init(gsl_ts_init);
module_exit(gsl_ts_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GSLX680 touchscreen controller driver");
MODULE_AUTHOR("Guan Yuwei, guanyuwei@basewin.com");
MODULE_ALIAS("platform:gsl_ts");
