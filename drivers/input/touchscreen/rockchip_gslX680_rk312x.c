/*
 * drivers/input/touchscreen/gslX680.c
 *
 * Copyright (c) 2012 Shanghai Basewin
 *	Guan Yuwei<guanyuwei@basewin.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>
#include "rockchip_gslX680_rk312x.h"
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/device.h>
#include "tp_suspend.h"

#define GPIO_HIGH 1
#define GPIO_LOW 0

/* #define GSL_DEBUG */
/* #define GSL_TIMER */
#define REPORT_DATA_ANDROID_4_0

#define HAVE_TOUCH_KEY

#define GSLX680_I2C_NAME "gslX680_312x"
#define GSLX680_I2C_ADDR 0x40

/* #define IRQ_PORT			RK2928_PIN1_PB0 */
/* #define WAKE_PORT			RK2928_PIN0_PD3 */

#define GSL_DATA_REG		0x80
#define GSL_STATUS_REG		0xe0
#define GSL_PAGE_REG		0xf0

#define PRESS_MAX	255
#define MAX_FINGERS	5
#define MAX_CONTACTS	10
#define DMA_TRANS_LEN	0x20
/* #define FILTER_POINT */
#ifdef FILTER_POINT
#define FILTER_MAX	6
#endif

#define WRITE_I2C_SPEED (350*1000)
#define I2C_SPEED  (200*1000)
static struct i2c_client *gsl_client = NULL;

//#define TPD_PROC_DEBUG
#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif


#define CLOSE_TP_POWER  0

#if CLOSE_TP_POWER
static void set_tp_power(bool flag);
#endif

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

static const u16 key_array[] = {
	KEY_BACK,
	KEY_HOMEPAGE,
	KEY_MENU,
	//KEY_SEARCH,
	};

#define MAX_KEY_NUM     (sizeof(key_array)/sizeof(key_array[0]))

static struct key_data gsl_key_data[MAX_KEY_NUM] = {
	{KEY_BACK, 1040, 1100, 50, 130}, //90, 1060
	{KEY_HOMEPAGE, 1040, 1100, 230, 310}, //270, 1060
	{KEY_MENU, 1040, 1100, 410, 490}, //450, 1060
	//{KEY_SEARCH, 2048, 2048, 2048, 2048},
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
	int irq;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
#ifdef GSL_TIMER
	struct timer_list gsl_timer;
#endif
	int reset_gpio;
};

static struct gsl_ts *g_gsl_ts;
//#define GSL_DEBUG
#ifdef GSL_DEBUG
#define print_info(fmt, args...) printk(fmt, ##args)
#else
#define print_info(fmt, args...)
#endif

//static int ts_global_reset_pin;

static u32 id_sign[MAX_CONTACTS+1] = {0};
static u8 id_state_flag[MAX_CONTACTS+1] = {0};
static u8 id_state_old_flag[MAX_CONTACTS+1] = {0};
static u16 x_old[MAX_CONTACTS+1] = {0};
static u16 y_old[MAX_CONTACTS+1] = {0};
static u16 x_new;
static u16 y_new;




static int gslX680_shutdown_low(void)
{
	//gpio_direction_output(ts_global_reset_pin, GPIO_LOW);
	//gpio_set_value(ts_global_reset_pin, GPIO_LOW);
	return 0;
}

static int gslX680_shutdown_high(void)
{
	//gpio_direction_output(ts_global_reset_pin, GPIO_HIGH);
	//gpio_set_value(ts_global_reset_pin, GPIO_HIGH);
	return 0;
}
static inline u16 join_bytes(u8 a, u8 b)
{
	u16 ab = 0;

	ab = ab | a;
	ab = ab << 8 | b;
	return ab;
}

static u32 gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
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

static int gsl_ts_write(struct i2c_client *client, u8 addr, u8 *pdata, int datalen)
{
	int ret = 0;
	u8 tmp_buf[128];
	unsigned int bytelen = 0;
	if (datalen > 125)
	{
		printk("%s too big datalen = %d!\n", __func__, datalen);
		return -1;
	}
	
	tmp_buf[0] = addr;
	bytelen++;
	
	if (datalen != 0 && pdata != NULL)
	{
		memcpy(&tmp_buf[bytelen], pdata, datalen);
		bytelen += datalen;
	}
	
	ret = i2c_master_send(client, tmp_buf, bytelen);
	return ret;
}

static int gsl_ts_read(struct i2c_client *client, u8 addr, u8 *pdata, unsigned int datalen)
{
	int ret = 0;

	if (datalen > 126)
	{
		printk("%s too big datalen = %d!\n", __func__, datalen);
		return -1;
	}

	ret = gsl_ts_write(client, addr, NULL, 0);
	if (ret < 0)
	{
		printk("%s set data address fail!\n", __func__);
		return ret;
	}
	
	return i2c_master_recv(client, pdata, datalen);
}



static __inline__ void fw2buf(u8 *buf, const u32 *fw)
{
	u32 *u32_buf = (int *)buf;
	*u32_buf = *fw;
}

static void gsl_load_fw(struct i2c_client *client)
{
	u8 buf[DMA_TRANS_LEN*4 + 1] = {0};
	u8 send_flag = 1;
	u8 *cur = buf + 1;
	u32 source_line = 0;
	u32 source_len;
	//u8 read_buf[4] = {0};
	const struct fw_data *ptr_fw;
	printk("=============gsl_load_fw start==============\n");

	{
		ptr_fw = GSLX680_FW;
		source_len = fw_len;
	}

	for (source_line = 0; source_line < source_len; source_line++) {
		/* init page trans, set the page val */
		if (GSL_PAGE_REG == ptr_fw[source_line].offset) {
			fw2buf(cur, &ptr_fw[source_line].val);
			gsl_write_interface(client, GSL_PAGE_REG, buf, 4);
			send_flag = 1;
		} else {
			if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
				buf[0] = (u8)ptr_fw[source_line].offset;

			fw2buf(cur, &ptr_fw[source_line].val);
			cur += 4;

			if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20)) {
				gsl_write_interface(client, buf[0], buf, cur - buf - 1);
				cur = buf + 1;
			}

			send_flag++;
		}
	}

	printk("=============gsl_load_fw end==============\n");
}


static void startup_chip(struct i2c_client *client)
{
	u8 tmp = 0x00;
#if 0	
	u8 buf[4] = {0x00};

	buf[3] = 0x01;
	buf[2] = 0xfe;
	buf[1] = 0x10;
	buf[0] = 0x00;
	gsl_ts_write(client, 0xf0, buf, sizeof(buf));
	buf[3] = 0x00;
	buf[2] = 0x00;
	buf[1] = 0x00;
	buf[0] = 0x0f;
#endif
#ifdef GSL_NOID_VERSION
	gsl_DataInit(gsl_config_data_id);
#endif
	gsl_ts_write(client, 0xe0, &tmp, 1);
	msleep(10);
}

static void reset_chip(struct i2c_client *client)
{
	u8 tmp = 0x88;
	u8 buf[4] = {0x00};

	gsl_ts_write(client, 0xe0, &tmp, sizeof(tmp));
	msleep(20);
	tmp = 0x04;
	gsl_ts_write(client, 0xe4, &tmp, sizeof(tmp));
	msleep(10);
	gsl_ts_write(client, 0xbc, buf, sizeof(buf));
	msleep(10);
}

static void clr_reg(struct i2c_client *client)
{
	u8 write_buf[4]	= {0};

	write_buf[0] = 0x88;
	gsl_ts_write(client, 0xe0, &write_buf[0], 1);
	msleep(20);
	write_buf[0] = 0x03;
	gsl_ts_write(client, 0x80, &write_buf[0], 1);
	msleep(5);
	write_buf[0] = 0x04;
	gsl_ts_write(client, 0xe4, &write_buf[0], 1);
	msleep(5);
	write_buf[0] = 0x00;
	gsl_ts_write(client, 0xe0, &write_buf[0], 1);
	msleep(20);
}

static void init_chip(struct i2c_client *client)
{
	//gslX680_shutdown_low();
	msleep(20);
	//gslX680_shutdown_high();
	msleep(20);
	clr_reg(client);
	reset_chip(client);
	gsl_load_fw(client);
	startup_chip(client);
	reset_chip(client);
	startup_chip(client);
}

static void check_mem_data(struct i2c_client *client)
{
	u8 read_buf[4]  = {0};

	msleep(50);
	gsl_ts_read(client, 0xb0, read_buf, sizeof(read_buf));

	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a) {
	 printk("#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
   #if CLOSE_TP_POWER
	  set_tp_power(false);
	  msleep(200);
	  set_tp_power(true);
	  msleep(100);
   #endif
	 init_chip(client);
	}
}

#ifdef TPD_PROC_DEBUG
static int char_to_int(char ch)
{
	if(ch>='0' && ch<='9')
		return (ch-'0');
	else
		return (ch-'a'+10);
}

//static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
static int gsl_config_read_proc(struct seq_file *m,void *v)
{
	char temp_data[5] = {0};
	unsigned int tmp=0;
	
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_NOID_VERSION
		tmp=gsl_version_id();
#else 
		tmp=0x20121215;
#endif
		seq_printf(m,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_NOID_VERSION 
			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			seq_printf(m,"gsl_config_data_id[%d] = ",tmp);
			if(tmp>=0&&tmp<id_len)
			{
					seq_printf(m,"%d\n",gsl_config_data_id[tmp]); 
			}
#endif
		}
		else 
		{
			gsl_ts_write(gsl_client,0Xf0,&gsl_data_proc[4],4);
			if(gsl_data_proc[0] < 0x80)
				gsl_ts_read(gsl_client,gsl_data_proc[0],temp_data,4);
			gsl_ts_read(gsl_client,gsl_data_proc[0],temp_data,4);

			seq_printf(m,"offset : {0x%02x,0x",gsl_data_proc[0]);
			seq_printf(m,"%02x",temp_data[3]);
			seq_printf(m,"%02x",temp_data[2]);
			seq_printf(m,"%02x",temp_data[1]);
			seq_printf(m,"%02x};\n",temp_data[0]);
		}
	}
//	*eof = 1;
	return (0);
}
static int gsl_config_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	print_info("[tp-gsl][%s] \n",__func__);
	if(count > 512)
	{
		print_info("size not match [%d:%ld]\n", CONFIG_LEN, count);
        return -EFAULT;
	}
	path_buf=kzalloc(count,GFP_KERNEL);
	if(!path_buf)
	{
		printk("alloc path_buf memory error \n");
		return -1;
	}
	if(copy_from_user(path_buf, buffer, count))
	{
		print_info("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf,path_buf,(count<CONFIG_LEN?count:CONFIG_LEN));
	print_info("[tp-gsl][%s][%s]\n",__func__,temp_buf);
	
	buf[3]=char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);	
	buf[2]=char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
	buf[1]=char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
	buf[0]=char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);
	
	buf[7]=char_to_int(temp_buf[5])<<4 | char_to_int(temp_buf[6]);
	buf[6]=char_to_int(temp_buf[7])<<4 | char_to_int(temp_buf[8]);
	buf[5]=char_to_int(temp_buf[9])<<4 | char_to_int(temp_buf[10]);
	buf[4]=char_to_int(temp_buf[11])<<4 | char_to_int(temp_buf[12]);
	if('v'==temp_buf[0]&& 's'==temp_buf[1])//version //vs
	{
		memcpy(gsl_read,temp_buf,4);
		printk("gsl version\n");
	}
	else if('s'==temp_buf[0]&& 't'==temp_buf[1])//start //st
	{
	#ifdef GSL_MONITOR
		cancel_delayed_work_sync(&gsl_monitor_work);
	#endif
		gsl_proc_flag = 1;
		reset_chip(gsl_client);
	}
	else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
	{
		msleep(20);
		reset_chip(gsl_client);
		startup_chip(gsl_client);
		gsl_proc_flag = 0;
	}
	else if('r'==temp_buf[0]&&'e'==temp_buf[1])//read buf //
	{
		memcpy(gsl_read,temp_buf,4);
		memcpy(gsl_data_proc,buf,8);
	}
	else if('w'==temp_buf[0]&&'r'==temp_buf[1])//write buf
	{
		gsl_ts_write(gsl_client,buf[4],buf,4);
	}
#ifdef GSL_NOID_VERSION
	else if('i'==temp_buf[0]&&'d'==temp_buf[1])//write id config //
	{
		tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<id_len)
		{
			gsl_config_data_id[tmp1] = tmp;
		}
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}
static int gsl_server_list_open(struct inode *inode,struct file *file)
{
	return single_open(file,gsl_config_read_proc,NULL);
}
static const struct file_operations gsl_seq_fops = {
	.open = gsl_server_list_open,
	.read = seq_read,
	.release = single_release,
	.write = gsl_config_write_proc,
	.owner = THIS_MODULE,
};
#endif

static void record_point(u16 x, u16 y , u8 id)
{
	u16 x_err = 0;
	u16 y_err = 0;

	id_sign[id] = id_sign[id] + 1;

	if (id_sign[id] == 1) {
		x_old[id] = x;
		y_old[id] = y;
	}

	x = (x_old[id] + x)/2;
	y = (y_old[id] + y)/2;

	if (x > x_old[id]) {
		x_err = x - x_old[id];
	} else {
		x_err = x_old[id]-x;
	}

	if (y > y_old[id]) {
		y_err = y - y_old[id];
	} else {
		y_err = y_old[id] - y;
	}

	if ((x_err > 3 && y_err > 1) || (x_err > 1 && y_err > 3)) {
		x_new = x;
		x_old[id] = x;
		y_new = y;
		y_old[id] = y;
	} else {
		if (x_err > 3) {
			x_new = x;
			x_old[id] = x;
		} else
			x_new = x_old[id];

		if (y_err > 3) {
			y_new = y;
			y_old[id] = y;
		} else
			y_new = y_old[id];
	}

	if (id_sign[id] == 1) {
		x_new = x_old[id];
		y_new = y_old[id];
	}
}

#ifdef HAVE_TOUCH_KEY
static void report_key(struct gsl_ts *ts, u16 x, u16 y)
{
	u16 i = 0;

	for(i = 0; i < MAX_KEY_NUM; i++) 
	{
		if((gsl_key_data[i].x_min < x) && (x < gsl_key_data[i].x_max)&&(gsl_key_data[i].y_min < y) && (y < gsl_key_data[i].y_max))
		{
			//printk("#####key=%d,x=%d,y=%d######\n", key, x, y);
			key = gsl_key_data[i].key;	
			input_report_key(ts->input, key, 1);
			//input_report_key(ts->input, key, 0);
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

	//printk("#####id=%d,x=%d,y=%d######\n", id, x, y);
	//x = 1024 - x;
	//y = 600 - y;

	if (x > SCREEN_MAX_X || y > SCREEN_MAX_Y) {
	#ifdef HAVE_TOUCH_KEY
		report_key(ts, x, y);
	#endif
		return;
	}

#ifdef REPORT_DATA_ANDROID_4_0
	input_mt_slot(ts->input, id);
	input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, pressure);
	input_report_abs(ts->input, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
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

static void process_gslX680_data(struct gsl_ts *ts)
{
	u8 id, touches;
	u16 x, y;
	int i = 0;
#ifdef GSL_NOID_VERSION
		u32 tmp1;
		u8 buf[4] = {0};
		struct gsl_touch_info cinfo = {{0}};
#endif

	touches = ts->touch_data[ts->dd->touch_index];

#ifdef GSL_NOID_VERSION
{
	cinfo.finger_num = touches;
	print_info("tp-gsl  finger_num = %d\n",cinfo.finger_num);
	for(i = 0; i < (touches < MAX_CONTACTS ? touches : MAX_CONTACTS); i ++)
	{
		cinfo.x[i] = join_bytes( ( ts->touch_data[ts->dd->x_index  + 4 * i + 1] & 0xf),
				ts->touch_data[ts->dd->x_index + 4 * i]);
		cinfo.y[i] = join_bytes(ts->touch_data[ts->dd->y_index + 4 * i + 1],
				ts->touch_data[ts->dd->y_index + 4 * i ]);
		print_info("tp-gsl  x = %d y = %d \n",cinfo.x[i],cinfo.y[i]);
	}
	cinfo.finger_num=(ts->touch_data[3]<<24)|(ts->touch_data[2]<<16)
		|(ts->touch_data[1]<<8)|(ts->touch_data[0]);
	gsl_alg_id_main(&cinfo);
	tmp1=gsl_mask_tiaoping();
	print_info("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;buf[1]=0;buf[2]=0;buf[3]=0;
		gsl_ts_write(gsl_client,0xf0,buf,4);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
		print_info("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",
			tmp1,buf[0],buf[1],buf[2],buf[3]);
		gsl_ts_write(gsl_client,0x8,buf,4);
	}
	touches = cinfo.finger_num;
}	
#endif
	for (i = 1; i <= MAX_CONTACTS; i++) {
		if (touches == 0)
			id_sign[i] = 0;
		id_state_flag[i] = 0;
	}

	for (i = 0; i < (touches > MAX_FINGERS ? MAX_FINGERS : touches); i++) {
		#ifdef GSL_NOID_VERSION
		{
		id = cinfo.id[i];
		x =  cinfo.x[i];
		y =  cinfo.y[i];	
		}
	#else
		{
		x = join_bytes( ( ts->touch_data[ts->dd->x_index  + 4 * i + 1] & 0xf),
				ts->touch_data[ts->dd->x_index + 4 * i]);
		y = join_bytes(ts->touch_data[ts->dd->y_index + 4 * i + 1],
				ts->touch_data[ts->dd->y_index + 4 * i ]);
		id = ts->touch_data[ts->dd->id_index + 4 * i] >> 4;
		}
	#endif
	#if 0
		x = join_bytes((ts->touch_data[ts->dd->x_index + 4*i + 1] & 0xf), ts->touch_data[ts->dd->x_index + 4*i]);
		y = join_bytes(ts->touch_data[ts->dd->y_index + 4*i + 1], ts->touch_data[ts->dd->y_index + 4*i]);
		id = ts->touch_data[ts->dd->id_index + 4*i] >> 4;
#endif
		if (1 <= id && id <= MAX_CONTACTS) {
		#ifdef FILTER_POINT
			filter_point(x, y, id);
		#else
			record_point(x, y, id);
		#endif
			report_data(ts, x_new, y_new, 50, id);
			id_state_flag[id] = 1;
		}
	}

	for (i = 1; i <= MAX_CONTACTS; i++) {
		if ((0 == touches) || ((0 != id_state_old_flag[i]) && (0 == id_state_flag[i]))) {
		#ifdef REPORT_DATA_ANDROID_4_0
			input_mt_slot(ts->input, i);
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		#endif
			id_sign[i] = 0;
		}
		id_state_old_flag[i] = id_state_flag[i];
	}
#ifdef REPORT_DATA_ANDROID_4_0
	if (0 == touches) {
		//input_mt_sync(ts->input);
	#ifdef HAVE_TOUCH_KEY
		if (key_state_flag) {
			//printk("key=%d\n", key);
			input_report_key(ts->input, key, 0);
			input_sync(ts->input);
			key_state_flag = 0;
		}
	#endif
	}
#endif
	input_sync(ts->input);
}


static void gsl_ts_xy_worker(struct work_struct *work)
{
	int rc;
	u8 read_buf[4] = {0};

	struct gsl_ts *ts = container_of(work, struct gsl_ts, work);
	print_info("---gsl_ts_xy_worker---\n");
#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1)
		goto schedule;
#endif

	/* read data from DATA_REG */
	rc = gsl_ts_read(ts->client, 0x80, ts->touch_data, ts->dd->data_size);
	print_info("---touches: %d ---\n", ts->touch_data[0]);

	if (rc < 0) {
		dev_err(&ts->client->dev, "read failed\n");
		goto schedule;
	}

	if (ts->touch_data[ts->dd->touch_index] == 0xff) {
		goto schedule;
	}

	rc = gsl_ts_read(ts->client, 0xbc, read_buf, sizeof(read_buf));
	if (rc < 0) {
		dev_err(&ts->client->dev, "read 0xbc failed\n");
		goto schedule;
	}
	print_info("//////// reg %x : %x %x %x %x\n", 0xbc, read_buf[3], read_buf[2], read_buf[1], read_buf[0]);

	if (read_buf[3] == 0 && read_buf[2] == 0 && read_buf[1] == 0 && read_buf[0] == 0) {
		process_gslX680_data(ts);
	} else {
		reset_chip(ts->client);
		startup_chip(ts->client);
	}
schedule:
	enable_irq(ts->irq);
}

static irqreturn_t gsl_ts_irq(int irq, void *dev_id)
{
	struct gsl_ts *ts = dev_id;

	//printk("==========GSLX680 Interrupt============\n");

	disable_irq_nosync(ts->irq);

	if (!work_pending(&ts->work)) {
		queue_work(ts->wq, &ts->work);
	}

	return IRQ_HANDLED;

}

#ifdef GSL_TIMER
static void gsl_timer_handle(unsigned long data)
{
	struct gsl_ts *ts = (struct gsl_ts *)data;

#ifdef GSL_DEBUG
	printk("----------------gsl_timer_handle-----------------\n");
#endif

	disable_irq_nosync(ts->irq);
	check_mem_data(ts->client);
	ts->gsl_timer.expires = jiffies + 3 * HZ;
	add_timer(&ts->gsl_timer);
	enable_irq(ts->irq);
}
#endif

static int gsl_ts_init_ts(struct i2c_client *client, struct gsl_ts *ts)
{
	struct input_dev *input_device;
#ifdef CONFIG_MACH_RK_FAC
	struct tp_platform_data *pdata = client->dev.platform_data;
#endif
	int rc = 0;
	int i = 0;

	printk("[GSLX680] Enter %s\n", __func__);

	ts->dd = &devices[ts->device_id];

	if (ts->device_id == 0) {
		ts->dd->data_size = MAX_FINGERS * ts->dd->touch_bytes + ts->dd->touch_meta_data;
		ts->dd->touch_index = 0;
	}

	ts->touch_data = kzalloc(ts->dd->data_size, GFP_KERNEL);
	if (!ts->touch_data) {
		pr_err("%s: Unable to allocate memory\n", __func__);
		return -ENOMEM;
	}


	input_device = input_allocate_device();
	if (!input_device) {
		rc = -ENOMEM;
		goto error_alloc_dev;
	}

	ts->input = input_device;
	input_device->name = GSLX680_I2C_NAME;
	input_device->id.bustype = BUS_I2C;
	input_device->dev.parent = &client->dev;
	input_set_drvdata(input_device, ts);

#ifdef REPORT_DATA_ANDROID_4_0
	__set_bit(EV_ABS, input_device->evbit);
	__set_bit(EV_KEY, input_device->evbit);
	__set_bit(EV_REP, input_device->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_device->propbit);
	input_mt_init_slots(input_device, (MAX_CONTACTS+1), 0);
#else
	input_set_abs_params(input_device, ABS_MT_TRACKING_ID, 0, (MAX_CONTACTS+1), 0, 0);
	set_bit(EV_ABS, input_device->evbit);
	set_bit(EV_KEY, input_device->evbit);
#endif

#ifdef HAVE_TOUCH_KEY
	//input_device->evbit[0] |= BIT_MASK(EV_KEY)| BIT_MASK(EV_SYN);
	//input_device->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_SYN)|BIT_MASK(EV_ABS);
	for (i = 0; i < MAX_KEY_NUM; i++)
		set_bit(key_array[i], input_device->keybit);
#endif

	set_bit(ABS_MT_POSITION_X, input_device->absbit);
	set_bit(ABS_MT_POSITION_Y, input_device->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input_device->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_device->absbit);
#ifdef CONFIG_MACH_RK_FAC
	input_set_abs_params(input_device, ABS_MT_POSITION_X, 0, pdata->x_max, 0, 0);
	input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0, pdata->y_max, 0, 0);
#else
	input_set_abs_params(input_device, ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
#endif
	input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_device, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	client->irq = ts->irq;

	ts->wq = create_singlethread_workqueue("kworkqueue_ts");
	if (!ts->wq) {
		dev_err(&client->dev, "Could not create workqueue\n");
		goto error_wq_create;
	}
	flush_workqueue(ts->wq);

	INIT_WORK(&ts->work, gsl_ts_xy_worker);

	rc = input_register_device(input_device);
	if (rc)
		goto error_unreg_device;

	return 0;

error_unreg_device:
	destroy_workqueue(ts->wq);
error_wq_create:
	input_free_device(input_device);
error_alloc_dev:
	kfree(ts->touch_data);
	return rc;
}


#if 1
static int gsl_ts_suspend(void)
{
	struct gsl_ts *ts = g_gsl_ts;
	printk("I'am in gsl_ts_suspend() start\n");
	flush_workqueue(ts->wq);
#ifdef GSL_TIMER
	printk("gsl_ts_suspend () : delete gsl_timer\n");
	del_timer(&ts->gsl_timer);
#endif
	disable_irq_nosync(ts->irq);
	gslX680_shutdown_low();
	return 0;
}

static int gsl_ts_resume(void)
{
	struct gsl_ts *ts = g_gsl_ts;
	int i = 0;

	//printk("I'am in gsl_ts_resume() start\n");

	gslX680_shutdown_high();
	msleep(20);
	reset_chip(ts->client);
	startup_chip(ts->client);
	check_mem_data(ts->client);

	msleep(100);
	reset_chip(ts->client);
	startup_chip(ts->client);
	msleep(100);
	reset_chip(ts->client);
	startup_chip(ts->client);
#ifdef GSL_TIMER
	printk("gsl_ts_resume () : add gsl_timer\n");
	init_timer(&ts->gsl_timer);
	ts->gsl_timer.expires = jiffies + 3 * HZ;
	ts->gsl_timer.function = &gsl_timer_handle;
	ts->gsl_timer.data = (unsigned long)ts;
	add_timer(&ts->gsl_timer);
#endif

  for (i = 1; i <= MAX_CONTACTS; i++) {
		if (0 != id_state_old_flag[i]) {
		#ifdef REPORT_DATA_ANDROID_4_0
			input_mt_slot(ts->input, i);
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
			input_sync(ts->input);
		#endif
			id_sign[i] = 0;
		}
		id_state_old_flag[i] = id_state_flag[i] = 0;
	}
	enable_irq(ts->irq);
	return 0;
}

static int gsl_ts_fb_event_notify(struct notifier_block *self,
				      unsigned long action, void *data)
{
	struct fb_event *event = data;
	int blank_mode = *((int *)event->data);

	if (action == FB_EARLY_EVENT_BLANK) {
		switch (blank_mode) {
		case FB_BLANK_UNBLANK:
			break;
		default:
			gsl_ts_suspend();
			break;
		}
	} else if (action == FB_EVENT_BLANK) {
		switch (blank_mode) {
		case FB_BLANK_UNBLANK:
			gsl_ts_resume();
			break;
		default:
			break;
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block gsl_ts_fb_notifier = {
	.notifier_call = gsl_ts_fb_event_notify,
};

#endif

int gslx680_config_hw(struct gsl_ts *ts)
{
	struct device_node *np;
	enum of_gpio_flags rst_flags;
	unsigned long irq_flags;
	unsigned int irq_gpio;
	struct device *dev;
	int ret;
	
	dev = &ts->client->dev;
	np = dev->of_node;

	irq_gpio = of_get_named_gpio_flags(np, "touch-gpio", 0, (enum of_gpio_flags *)&irq_flags);
	ts->irq = gpio_to_irq(irq_gpio);
	ts->reset_gpio = of_get_named_gpio_flags(np, "reset-gpio", 0, &rst_flags);

	//if (gpio_request(ts->reset_gpio, NULL) != 0) {
	//	gpio_free(ts->reset_gpio);
	//	printk("gslx680_init_platform_hw gpio_request error\n");
	//	return -EIO;
	//}
	printk("reset gpio\n");

	if (gpio_request(irq_gpio, "tp-gpio-irq") != 0) {
	     gpio_free(irq_gpio);
	     printk("gslx680_init_platform_hw  gpio_request error\n");
	     return -EIO;
	}
	ret = gpio_direction_input(irq_gpio);
	printk("tp-gpio-irq input: %d\n", ret);
	//gpio_direction_output(ts->reset_gpio, GPIO_HIGH);
	mdelay(10);
	//gpio_set_value(ts->reset_gpio, GPIO_LOW);
	mdelay(10);
	//gpio_set_value(ts->reset_gpio, GPIO_HIGH);
	msleep(300);
	return 0;

}


static int  gsl_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct gsl_ts *ts;
	int rc;
	u8 read_buf = 0;
	int ret;
	struct device_node * tp_node = client->dev.of_node;;

	of_property_read_u32_index(tp_node,"max-x",0,&SCREEN_MAX_X);
	of_property_read_u32_index(tp_node,"max-y",0,&SCREEN_MAX_Y);
	printk("-----TP:x=%d, y=%d\n",SCREEN_MAX_X,SCREEN_MAX_Y);
//	tp_node = of_find_node_by_name(NULL,"tp_1024");
	if(SCREEN_MAX_X==1024 && SCREEN_MAX_Y==600)
	{
		gsl_config_data_id = gsl_config_data_id_1024_600;
		GSLX680_FW =(struct fw_data *) GSLX680_FW_1024_600;
	//	SCREEN_MAX_X = 1024;
	//	SCREEN_MAX_Y = 600;
		id_len = ARRAY_SIZE(gsl_config_data_id_1024_600);
		fw_len = ARRAY_SIZE(GSLX680_FW_1024_600);
		printk("-------TP:1024x600\n");
	}
	else if(SCREEN_MAX_X==800 && SCREEN_MAX_Y== 480)
	{
		gsl_config_data_id = gsl_config_data_id_800_480;
		GSLX680_FW =(struct fw_data *) GSLX680_FW_800_480;
	//	SCREEN_MAX_X = 800;
	//	SCREEN_MAX_Y = 480;
		id_len = ARRAY_SIZE(gsl_config_data_id_800_480);
		fw_len = ARRAY_SIZE(GSLX680_FW_800_480);
		printk("-------TP:800x480\n");
	}
	else if(SCREEN_MAX_X==1280 && SCREEN_MAX_Y==800)
	{
		gsl_config_data_id = gsl_config_data_id_1280_800;
		GSLX680_FW =(struct fw_data *) GSLX680_FW_1280_800;
	//	SCREEN_MAX_X = 1024;
	//	SCREEN_MAX_Y = 600;
		id_len = ARRAY_SIZE(gsl_config_data_id_1280_800);
		fw_len = ARRAY_SIZE(GSLX680_FW_1280_800);
		printk("-------TP:1280x800\n");
	}
	printk("GSLX680 Enter %s\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C functionality not supported\n");
		return -ENODEV;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	printk("==kzalloc success=\n");
	g_gsl_ts = ts;

	ts->client = client;
	ts->device_id = id->driver_data;

	gslx680_config_hw(ts);
	//ts_global_reset_pin = ts->reset_gpio;
	i2c_set_clientdata(client, ts);

	rc = gsl_ts_init_ts(client, ts);
	if (rc < 0) {
		dev_err(&client->dev, "GSLX680 init failed\n");
		goto error_mutex_destroy;
	}
	gsl_client = client;

	init_chip(ts->client);
	check_mem_data(ts->client);

	rc = request_irq(client->irq, gsl_ts_irq, IRQF_TRIGGER_RISING, client->name, ts);
	if (rc < 0) {
		printk("gsl_probe: request irq failed\n");
		goto error_req_irq_fail;
	}

	disable_irq(client->irq);

	ret = gsl_ts_read(client, 0xf0, &read_buf, sizeof(read_buf));
	if (ret  < 0) {
		pr_info("gslx680  I2C transfer error!\n");
		goto error_req_irq_fail;
	}
#ifdef TPD_PROC_DEBUG
		proc_create(GSL_CONFIG_PROC_FILE,0666,NULL,&gsl_seq_fops);
		gsl_proc_flag = 0;
		//this_client = gsl_client;
#endif

#ifdef GSL_TIMER
	printk("gsl_ts_probe () : add gsl_timer\n");

	init_timer(&ts->gsl_timer);
	ts->gsl_timer.expires = jiffies + 3 * HZ;
	ts->gsl_timer.function = &gsl_timer_handle;
	ts->gsl_timer.data = (unsigned long)ts;
	add_timer(&ts->gsl_timer);
#endif
	fb_register_client(&gsl_ts_fb_notifier);
	enable_irq(ts->irq);
	printk("[GSLX680] End %s\n", __func__);

	return 0;

error_req_irq_fail:
    free_irq(ts->irq, ts);

error_mutex_destroy:
	input_free_device(ts->input);
	kfree(ts);
	return rc;
}

static int  gsl_ts_remove(struct i2c_client *client)
{
	struct gsl_ts *ts = i2c_get_clientdata(client);
	printk("==gsl_ts_remove=\n");
	device_init_wakeup(&client->dev, 0);
	cancel_work_sync(&ts->work);
	free_irq(ts->irq, ts);
	destroy_workqueue(ts->wq);
	input_unregister_device(ts->input);

	kfree(ts->touch_data);
	kfree(ts);

	return 0;
}

static const struct i2c_device_id gsl_ts_id[] = {
	{GSLX680_I2C_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, gsl_ts_id);

static struct of_device_id gslX680_dt_ids[] = {
	{ .compatible = "gslX680_312x" },
};

static struct i2c_driver gsl_ts_driver = {
	.driver = {
		.name = GSLX680_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gslX680_dt_ids),
	},
	.probe		= gsl_ts_probe,
	.remove		= gsl_ts_remove,
	.id_table	= gsl_ts_id,
};

static int __init gsl_ts_init(void)
{
    int ret;
	printk("==gsl_ts_init==\n");
	ret = i2c_add_driver(&gsl_ts_driver);
	printk("ret=%d\n", ret);
	return ret;
}
static void __exit gsl_ts_exit(void)
{
	printk("==gsl_ts_exit==\n");
	i2c_del_driver(&gsl_ts_driver);
	return;
}

module_init(gsl_ts_init);
module_exit(gsl_ts_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GSLX680 touchscreen controller driver");
MODULE_AUTHOR("Guan Yuwei, guanyuwei@basewin.com");
MODULE_ALIAS("platform:gsl_ts");
