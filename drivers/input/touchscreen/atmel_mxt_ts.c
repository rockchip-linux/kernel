/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

/* Version */
#define MXT_VER_20		20
#define MXT_VER_21		21
#define MXT_VER_22		22

/* Slave addresses */
#define MXT_APP_LOW		0x4a
#define MXT_APP_HIGH		0x4b
/*
 * MXT_BOOT_LOW disagrees with Atmel documentation, but has been
 * updated to support new touch hardware that pairs 0x26 boot with 0x4a app.
 */
#define MXT_BOOT_LOW		0x26
#define MXT_BOOT_HIGH		0x25

/* Firmware */
#define MXT_FW_NAME		"maxtouch.fw"

/* Registers */
#define MXT_INFO		0x00
#define MXT_FAMILY_ID		0x00
#define MXT_VARIANT_ID		0x01
#define MXT_VERSION		0x02
#define MXT_BUILD		0x03
#define MXT_MATRIX_X_SIZE	0x04
#define MXT_MATRIX_Y_SIZE	0x05
#define MXT_OBJECT_NUM		0x06
#define MXT_OBJECT_START	0x07

#define MXT_OBJECT_SIZE		6

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_COMMAND_T6		6
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_GEN_DATASOURCE_T53		53
#define MXT_TOUCH_MULTI_T9		9
#define MXT_TOUCH_KEYARRAY_T15		15
#define MXT_TOUCH_PROXIMITY_T23		23
#define MXT_TOUCH_PROXKEY_T52		52
#define MXT_PROCI_GRIPFACE_T20		20
#define MXT_PROCG_NOISE_T22		22
#define MXT_PROCI_ONETOUCH_T24		24
#define MXT_PROCI_TWOTOUCH_T27		27
#define MXT_PROCI_GRIP_T40		40
#define MXT_PROCI_PALM_T41		41
#define MXT_PROCI_TOUCHSUPPRESSION_T42	42
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCG_NOISESUPPRESSION_T48	48
#define MXT_PROCI_ADAPTIVETHRESHOLD_T55 55
#define MXT_PROCI_SHIELDLESS_T56	56
#define MXT_PROCI_EXTRATOUCHSCREENDATA_T57	57
#define MXT_PROCG_NOISESUPPRESSION_T62	62
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_GPIOPWM_T19		19
#define MXT_SPT_SELFTEST_T25		25
#define MXT_SPT_CTECONFIG_T28		28
#define MXT_SPT_USERDATA_T38		38
#define MXT_SPT_DIGITIZER_T43		43
#define MXT_SPT_MESSAGECOUNT_T44	44
#define MXT_SPT_CTECONFIG_T46		46
#define MXT_SPT_TIMER_T61		61

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

#define MXT_T6_CMD_PAGE_UP		0x01
#define MXT_T6_CMD_PAGE_DOWN		0x02
#define MXT_T6_CMD_DELTAS		0x10
#define MXT_T6_CMD_REFS			0x11
#define MXT_T6_CMD_DEVICE_ID		0x80
#define MXT_T6_CMD_TOUCH_THRESH		0xF4

/* MXT_GEN_POWER_T7 field */
#define MXT_POWER_IDLEACQINT	0
#define MXT_POWER_ACTVACQINT	1
#define MXT_POWER_ACTV2IDLETO	2

/* MXT_GEN_ACQUIRE_T8 field */
#define MXT_ACQUIRE_CHRGTIME	0
#define MXT_ACQUIRE_TCHDRIFT	2
#define MXT_ACQUIRE_DRIFTST	3
#define MXT_ACQUIRE_TCHAUTOCAL	4
#define MXT_ACQUIRE_SYNC	5
#define MXT_ACQUIRE_ATCHCALST	6
#define MXT_ACQUIRE_ATCHCALSTHR	7

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_TOUCH_CTRL		0
#define MXT_TOUCH_XORIGIN	1
#define MXT_TOUCH_YORIGIN	2
#define MXT_TOUCH_XSIZE		3
#define MXT_TOUCH_YSIZE		4
#define MXT_TOUCH_BLEN		6
#define MXT_TOUCH_TCHTHR	7
#define MXT_TOUCH_TCHDI		8
#define MXT_TOUCH_ORIENT	9
#define MXT_TOUCH_MOVHYSTI	11
#define MXT_TOUCH_MOVHYSTN	12
#define MXT_TOUCH_NUMTOUCH	14
#define MXT_TOUCH_MRGHYST	15
#define MXT_TOUCH_MRGTHR	16
#define MXT_TOUCH_AMPHYST	17
#define MXT_TOUCH_XRANGE_LSB	18
#define MXT_TOUCH_XRANGE_MSB	19
#define MXT_TOUCH_YRANGE_LSB	20
#define MXT_TOUCH_YRANGE_MSB	21
#define MXT_TOUCH_XLOCLIP	22
#define MXT_TOUCH_XHICLIP	23
#define MXT_TOUCH_YLOCLIP	24
#define MXT_TOUCH_YHICLIP	25
#define MXT_TOUCH_XEDGECTRL	26
#define MXT_TOUCH_XEDGEDIST	27
#define MXT_TOUCH_YEDGECTRL	28
#define MXT_TOUCH_YEDGEDIST	29
#define MXT_TOUCH_JUMPLIMIT	30

/* MXT_PROCI_GRIPFACE_T20 field */
#define MXT_GRIPFACE_CTRL	0
#define MXT_GRIPFACE_XLOGRIP	1
#define MXT_GRIPFACE_XHIGRIP	2
#define MXT_GRIPFACE_YLOGRIP	3
#define MXT_GRIPFACE_YHIGRIP	4
#define MXT_GRIPFACE_MAXTCHS	5
#define MXT_GRIPFACE_SZTHR1	7
#define MXT_GRIPFACE_SZTHR2	8
#define MXT_GRIPFACE_SHPTHR1	9
#define MXT_GRIPFACE_SHPTHR2	10
#define MXT_GRIPFACE_SUPEXTTO	11

/* MXT_PROCI_NOISE field */
#define MXT_NOISE_CTRL		0
#define MXT_NOISE_OUTFLEN	1
#define MXT_NOISE_GCAFUL_LSB	3
#define MXT_NOISE_GCAFUL_MSB	4
#define MXT_NOISE_GCAFLL_LSB	5
#define MXT_NOISE_GCAFLL_MSB	6
#define MXT_NOISE_ACTVGCAFVALID	7
#define MXT_NOISE_NOISETHR	8
#define MXT_NOISE_FREQHOPSCALE	10
#define MXT_NOISE_FREQ0		11
#define MXT_NOISE_FREQ1		12
#define MXT_NOISE_FREQ2		13
#define MXT_NOISE_FREQ3		14
#define MXT_NOISE_FREQ4		15
#define MXT_NOISE_IDLEGCAFVALID	16

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1

/* MXT_SPT_CTECONFIG_T28 field */
#define MXT_CTE_CTRL		0
#define MXT_CTE_CMD		1
#define MXT_CTE_MODE		2
#define MXT_CTE_IDLEGCAFDEPTH	3
#define MXT_CTE_ACTVGCAFDEPTH	4
#define MXT_CTE_VOLTAGE		5

#define MXT_VOLTAGE_DEFAULT	2700000
#define MXT_VOLTAGE_STEP	10000

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_BACKUP_VALUE	0x55
#define MXT_BACKUP_TIME		50	/* msec */
#define MXT_RESET_TIME		200	/* msec */
#define MXT_CAL_TIME		25	/* msec */

#define MXT_FWRESET_TIME	500	/* msec */

/* MXT_SPT_GPIOPWM_T19 field */
#define MXT_GPIO0_MASK		0x04
#define MXT_GPIO1_MASK		0x08
#define MXT_GPIO2_MASK		0x10
#define MXT_GPIO3_MASK		0x20

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f

/* Touch status */
#define MXT_UNGRIP		(1 << 0)
#define MXT_SUPPRESS		(1 << 1)
#define MXT_AMP			(1 << 2)
#define MXT_VECTOR		(1 << 3)
#define MXT_MOVE		(1 << 4)
#define MXT_RELEASE		(1 << 5)
#define MXT_PRESS		(1 << 6)
#define MXT_DETECT		(1 << 7)

/* Touch orient bits */
#define MXT_XY_SWITCH		(1 << 0)
#define MXT_X_INVERT		(1 << 1)
#define MXT_Y_INVERT		(1 << 2)

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

/* For CMT (must match XRANGE/YRANGE as defined in board config */
#define MXT_PIXELS_PER_MM	20

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size;		/* Size of each instance - 1 */
	u8 instances;		/* Number of instances - 1 */
	u8 num_report_ids;
} __packed;

struct mxt_message {
	u8 reportid;
	u8 message[7];
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	const struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info info;
	bool is_tp;

	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;

	/* max touchscreen area in terms of pixels and channels */
	unsigned int max_area_pixels;
	unsigned int max_area_channels;

	u32 info_csum;
	u32 config_csum;

	/* Cached parameters from object table */
	u16 T5_address;
	u8 T6_reportid;
	u8 T9_reportid_min;
	u8 T9_reportid_max;
	u8 T19_reportid;
	u16 T44_address;

	/* for fw update in bootloader */
	struct completion bl_completion;

	/* per-instance debugfs root */
	struct dentry *dentry_dev;
	struct dentry *dentry_deltas;
	struct dentry *dentry_refs;

	/* Protect access to the T37 object buffer, used by debugfs */
	struct mutex T37_buf_mutex;
	u8 *T37_buf;
	size_t T37_buf_size;

	/* Saved T7 configuration
	 * [0] = IDLEACQINT
	 * [1] = ACTVACQINT
	 * [2] = ACTV2IDLETO
	 */
	u8 T7_config[3];
	bool T7_config_valid;

	/* Saved T9 Ctrl field */
	u8 T9_ctrl;
	bool T9_ctrl_valid;

	bool irq_wake;  /* irq wake is enabled */
};

/* global root node of the atmel_mxt_ts debugfs directory. */
static struct dentry *mxt_debugfs_root;

static int mxt_calc_resolution(struct mxt_data *data);
static void mxt_free_object_table(struct mxt_data *data);
static int mxt_initialize(struct mxt_data *data);
static int mxt_input_dev_create(struct mxt_data *data);

static inline size_t mxt_obj_size(const struct mxt_object *obj)
{
	return obj->size + 1;
}

static inline size_t mxt_obj_instances(const struct mxt_object *obj)
{
	return obj->instances + 1;
}

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_GEN_DATASOURCE_T53:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_PROCI_ADAPTIVETHRESHOLD_T55:
	case MXT_PROCI_SHIELDLESS_T56:
	case MXT_PROCI_EXTRATOUCHSCREENDATA_T57:
	case MXT_PROCG_NOISESUPPRESSION_T62:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_SPT_TIMER_T61:
		return true;
	default:
		return false;
	}
}

static bool mxt_object_writable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_PROCI_ADAPTIVETHRESHOLD_T55:
	case MXT_PROCI_SHIELDLESS_T56:
	case MXT_PROCI_EXTRATOUCHSCREENDATA_T57:
	case MXT_PROCG_NOISESUPPRESSION_T62:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_SPT_TIMER_T61:
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct device *dev,
			     struct mxt_message *message)
{
	dev_dbg(dev, "reportid: %u\tmessage: %*ph\n",
		message->reportid, 7, message->message);
}

static bool mxt_in_bootloader(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	return (client->addr == MXT_BOOT_LOW || client->addr == MXT_BOOT_HIGH);
}

static int mxt_i2c_recv(struct i2c_client *client, u8 *buf, size_t count)
{
	int ret;

	ret = i2c_master_recv(client, buf, count);
	if (ret == count) {
		ret = 0;
	} else if (ret != count) {
		ret = (ret < 0) ? ret : -EIO;
		dev_err(&client->dev, "i2c recv failed (%d)\n", ret);
	}

	return ret;
}

static int mxt_i2c_send(struct i2c_client *client, const u8 *buf, size_t count)
{
	int ret;

	ret = i2c_master_send(client, buf, count);
	if (ret == count) {
		ret = 0;
	} else if (ret != count) {
		ret = (ret < 0) ? ret : -EIO;
		dev_err(&client->dev, "i2c send failed (%d)\n", ret);
	}

	return ret;
}

static int mxt_i2c_transfer(struct i2c_client *client, struct i2c_msg *msgs,
		size_t count)
{
	int ret;

	ret = i2c_transfer(client->adapter, msgs, count);
	if (ret == count) {
		ret = 0;
	} else {
		ret = (ret < 0) ? ret : -EIO;
		dev_err(&client->dev, "i2c transfer failed (%d)\n", ret);
	}

	return ret;
}

static int mxt_wait_for_chg(struct mxt_data *data, unsigned int timeout_ms)
{
	struct device *dev = &data->client->dev;
	struct completion *comp = &data->bl_completion;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		dev_err(dev, "Wait for completion interrupted.\n");
		/*
		 * TODO: handle -EINTR better by terminating fw update process
		 * before returning to userspace by writing length 0x000 to
		 * device (iff we are in WAITING_FRAME_DATA state).
		 */
		return -EINTR;
	} else if (ret == 0) {
		dev_err(dev, "Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int mxt_check_bootloader(struct mxt_data *data, unsigned int state)
{
	struct i2c_client *client = data->client;
	u8 val;
	int ret;

recheck:
	if (state != MXT_WAITING_BOOTLOAD_CMD) {
		/*
		 * In application update mode, the interrupt
		 * line signals state transitions. We must wait for the
		 * CHG assertion before reading the status byte.
		 * Once the status byte has been read, the line is deasserted.
		 */
		int ret = mxt_wait_for_chg(data, 300);
		if (ret) {
			dev_err(&client->dev, "Update wait error %d\n", ret);
			return ret;
		}
	}

	ret = mxt_i2c_recv(client, &val, 1);
	if (ret)
		return ret;

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
		dev_info(&client->dev, "bootloader version: %d\n",
			 val & MXT_BOOT_STATUS_MASK);
	case MXT_WAITING_FRAME_DATA:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(&client->dev, "Unvalid bootloader mode state\n");
		dev_err(&client->dev, "Invalid bootloader mode state %d, %d\n",
			val, state);
		return -EINVAL;
	}

	return 0;
}

static int mxt_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = MXT_UNLOCK_CMD_LSB;
	buf[1] = MXT_UNLOCK_CMD_MSB;

	return mxt_i2c_send(client, buf, 2);
}

static int mxt_fw_write(struct i2c_client *client,
			     const u8 *data, unsigned int frame_size)
{
	return mxt_i2c_send(client, data, frame_size);
}

#ifdef DEBUG
#define DUMP_LEN	16
static void mxt_dump_xfer(struct device *dev, const char *func, u16 reg,
			  u16 len, const u8 *val)
{
	/* Rough guess for string size */
	char str[DUMP_LEN * 3 + 2];
	int i;
	size_t n;

	for (i = 0, n = 0; i < len; i++) {
		n += snprintf(&str[n], sizeof(str) - n, "%02x ", val[i]);
		if ((i + 1) % DUMP_LEN == 0 || (i + 1) == len) {
			dev_dbg(dev,
				"%s(reg: %d len: %d offset: 0x%02x): %s\n",
				func, reg, len, (i / DUMP_LEN) * DUMP_LEN,
				str);
			n = 0;
		}
	}
}
#undef DUMP_LEN
#else
static void mxt_dump_xfer(struct device *dev, const char *func, u16 reg,
			  u16 len, const u8 *val) { }
#endif

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	ret = mxt_i2c_transfer(client, xfer, 2);
	if (ret == 0)
		mxt_dump_xfer(&client->dev, __func__, reg, len, val);

	return ret;
}

static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
			   const void *val)
{
	u8 *buf;
	size_t count;
	int ret;

	count = len + 2;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(&buf[2], val, len);

	mxt_dump_xfer(&client->dev, __func__, reg, len, val);
	ret = mxt_i2c_send(client, buf, count);
	kfree(buf);
	return ret;
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type\n");
	return NULL;
}

static int mxt_read_num_messages(struct mxt_data *data, u8 *count)
{
	/* TODO: Optimization: read first message along with message count */
	return __mxt_read_reg(data->client, data->T44_address, 1, count);
}

static int mxt_read_messages(struct mxt_data *data, u8 count,
			     struct mxt_message *messages)
{
	return __mxt_read_reg(data->client, data->T5_address,
			sizeof(struct mxt_message) * count, messages);
}

static int mxt_write_obj_instance(struct mxt_data *data, u8 type, u8 instance,
		u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object || offset >= mxt_obj_size(object) ||
	    instance >= mxt_obj_instances(object))
		return -EINVAL;

	reg = object->start_address + instance * mxt_obj_size(object) + offset;
	return mxt_write_reg(data->client, reg, val);
}

static int mxt_write_object(struct mxt_data *data, u8 type, u8 offset, u8 val)
{
	return mxt_write_obj_instance(data, type, 0, offset, val);
}

static void mxt_input_button(struct mxt_data *data, struct mxt_message *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input = data->input_dev;
	bool button;
	int i;

	if (!data->pdata) {
		/* Active-low switch */
		button = !(message->message[0] & MXT_GPIO3_MASK);
		input_report_key(input, BTN_LEFT, button);
		dev_dbg(dev, "Button state: %d\n", button);
		return;
	}

	/* Active-low switch */
	for (i = 0; i < MXT_NUM_GPIO; i++) {
		if (data->pdata->key_map[i] == KEY_RESERVED)
			continue;
		button = !(message->message[0] & MXT_GPIO0_MASK << i);
		input_report_key(input, data->pdata->key_map[i], button);
	}
}

/*
 * Assume a circle touch contact and use the diameter as the touch major.
 * touch_pixels = touch_channels * (max_area_pixels / max_area_channels)
 * touch_pixels = pi * (touch_major / 2) ^ 2;
 */
static int get_touch_major_pixels(struct mxt_data *data, int touch_channels)
{
	int touch_pixels;

	if (data->max_area_channels == 0)
		return 0;

	touch_pixels = DIV_ROUND_CLOSEST(touch_channels * data->max_area_pixels,
					 data->max_area_channels);
	return int_sqrt(DIV_ROUND_CLOSEST(touch_pixels * 100, 314)) * 2;
}

static void mxt_input_touchevent(struct mxt_data *data,
				      struct mxt_message *message, int id)
{
	struct device *dev = &data->client->dev;
	u8 status = message->message[0];
	struct input_dev *input_dev = data->input_dev;
	int x;
	int y;
	int area;
	int pressure;
	int touch_major;
	int vector1, vector2;

	x = (message->message[1] << 4) | ((message->message[3] >> 4) & 0xf);
	y = (message->message[2] << 4) | ((message->message[3] & 0xf));
	if (data->max_x < 1024)
		x = x >> 2;
	if (data->max_y < 1024)
		y = y >> 2;

	area = message->message[4];
	touch_major = get_touch_major_pixels(data, area);
	pressure = message->message[5];

	/* The two vector components are 4-bit signed ints (2s complement) */
	vector1 = (signed)((signed char)message->message[6]) >> 4;
	vector2 = (signed)((signed char)(message->message[6] << 4)) >> 4;

	dev_dbg(dev,
		"[%u] %c%c%c%c%c%c%c%c x: %5u y: %5u area: %3u amp: %3u vector: [%d,%d]\n",
		id,
		(status & MXT_DETECT) ? 'D' : '.',
		(status & MXT_PRESS) ? 'P' : '.',
		(status & MXT_RELEASE) ? 'R' : '.',
		(status & MXT_MOVE) ? 'M' : '.',
		(status & MXT_VECTOR) ? 'V' : '.',
		(status & MXT_AMP) ? 'A' : '.',
		(status & MXT_SUPPRESS) ? 'S' : '.',
		(status & MXT_UNGRIP) ? 'U' : '.',
		x, y, area, pressure, vector1, vector2);

	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER,
				   status & MXT_DETECT);

	if (status & MXT_DETECT) {
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, touch_major);
		/* TODO: Use vector to report ORIENTATION & TOUCH_MINOR */
	}
}

static unsigned mxt_extract_T6_csum(const u8 *csum)
{
	return csum[0] | (csum[1] << 8) | (csum[2] << 16);
}

static bool mxt_is_T9_message(struct mxt_data *data, struct mxt_message *msg)
{
	u8 id = msg->reportid;
	return (id >= data->T9_reportid_min && id <= data->T9_reportid_max);
}

static int mxt_proc_messages(struct mxt_data *data, u8 count)
{
	struct device *dev = &data->client->dev;
	u8 reportid;
	bool update_input = false;
	struct mxt_message *messages, *msg;
	int ret;

	messages = kcalloc(count, sizeof(*messages), GFP_KERNEL);
	if (!messages)
		return -ENOMEM;

	ret = mxt_read_messages(data, count, messages);
	if (ret) {
		dev_err(dev, "Failed to read %u messages (%d).\n", count, ret);
		goto out;
	}

	for (msg = messages; msg < &messages[count]; msg++) {
		mxt_dump_message(dev, msg);
		reportid = msg->reportid;

		if (reportid == data->T6_reportid) {
			const u8 *payload = &msg->message[0];
			u8 status = payload[0];
			data->config_csum = mxt_extract_T6_csum(&payload[1]);
			dev_dbg(dev, "Status: %02x Config Checksum: %06x\n",
				status, data->config_csum);
		} else if (mxt_is_T9_message(data, msg)) {
			int id = reportid - data->T9_reportid_min;
			mxt_input_touchevent(data, msg, id);
			update_input = true;
		} else if (msg->reportid == data->T19_reportid) {
			mxt_input_button(data, msg);
			update_input = true;
		}
	}

	if (update_input) {
		input_mt_report_pointer_emulation(data->input_dev, false);
		input_sync(data->input_dev);
	}

out:
	kfree(messages);
	return ret;
}

static int mxt_handle_messages(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 count;

	ret = mxt_read_num_messages(data, &count);
	if (ret) {
		dev_err(dev, "Failed to read message count (%d).\n", ret);
		return ret;
	}

	if (count > 0)
		ret = mxt_proc_messages(data, count);

	return ret;
}

static int mxt_enter_bl(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	int ret;

	if (mxt_in_bootloader(data))
		return 0;

	disable_irq(data->irq);

	/* Change to the bootloader mode */
	ret = mxt_write_object(data, MXT_GEN_COMMAND_T6,
			       MXT_COMMAND_RESET, MXT_BOOT_VALUE);
	if (ret) {
		enable_irq(data->irq);
		return ret;
	}

	/* Change to slave address of bootloader */
	if (client->addr == MXT_APP_LOW)
		client->addr = MXT_BOOT_LOW;
	else
		client->addr = MXT_BOOT_HIGH;

	/* Free any driver state. It will get reinitialized after fw update. */
	mxt_free_object_table(data);
	if (data->input_dev) {
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}

	init_completion(&data->bl_completion);
	enable_irq(data->irq);

	/* Wait for CHG assert to indicate successful reset into bootloader */
	ret = mxt_wait_for_chg(data, MXT_RESET_TIME);
	if (ret) {
		dev_err(dev, "Failed waiting for reset to bootloader.\n");
		if (client->addr == MXT_BOOT_LOW)
			client->addr = MXT_APP_LOW;
		else
			client->addr = MXT_APP_HIGH;
		return ret;
	}
	return 0;
}

static void mxt_exit_bl(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	int error;

	if (!mxt_in_bootloader(data))
		return;

	/* Wait for reset */
	mxt_wait_for_chg(data, MXT_FWRESET_TIME);

	disable_irq(data->irq);
	if (client->addr == MXT_BOOT_LOW)
		client->addr = MXT_APP_LOW;
	else
		client->addr = MXT_APP_HIGH;

	error = mxt_initialize(data);
	if (error) {
		dev_err(dev, "Failed to initialize on exit bl. error = %d\n",
			error);
		return;
	}

	error = mxt_input_dev_create(data);
	if (error) {
		dev_err(dev, "Create input dev failed after init. error = %d\n",
			error);
		return;
	}

	error = mxt_handle_messages(data);
	if (error)
		dev_err(dev, "Failed to clear CHG after init. error = %d\n",
			error);
	enable_irq(data->irq);
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;

	if (mxt_in_bootloader(data)) {
		/* bootloader state transition completion */
		complete(&data->bl_completion);
	} else {
		mxt_handle_messages(data);
	}
	return IRQ_HANDLED;
}

static int mxt_check_reg_init(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	int index = 0;
	int i, size;
	int ret;

	if (!pdata->config) {
		dev_dbg(dev, "No cfg data defined, skipping reg init\n");
		return 0;
	}

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_writable(object->type))
			continue;

		size = mxt_obj_size(object) * mxt_obj_instances(object);
		if (index + size > pdata->config_length) {
			dev_err(dev, "Not enough config data!\n");
			return -EINVAL;
		}

		ret = __mxt_write_reg(data->client, object->start_address,
				size, &pdata->config[index]);
		if (ret)
			return ret;
		index += size;
	}

	return 0;
}

static void mxt_handle_pdata(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	u8 voltage;

	/* Set touchscreen lines */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_XSIZE,
			pdata->x_line);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_YSIZE,
			pdata->y_line);

	/* Set touchscreen orient */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_ORIENT,
			pdata->orient);

	/* Set touchscreen burst length */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_BLEN, pdata->blen);

	/* Set touchscreen threshold */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_TCHTHR, pdata->threshold);

	/* Set touchscreen resolution */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_XRANGE_LSB, (pdata->x_size - 1) & 0xff);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_XRANGE_MSB, (pdata->x_size - 1) >> 8);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_YRANGE_LSB, (pdata->y_size - 1) & 0xff);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_YRANGE_MSB, (pdata->y_size - 1) >> 8);

	/* Set touchscreen voltage */
	if (pdata->voltage) {
		if (pdata->voltage < MXT_VOLTAGE_DEFAULT) {
			voltage = (MXT_VOLTAGE_DEFAULT - pdata->voltage) /
				MXT_VOLTAGE_STEP;
			voltage = 0xff - voltage + 1;
		} else
			voltage = (pdata->voltage - MXT_VOLTAGE_DEFAULT) /
				MXT_VOLTAGE_STEP;

		mxt_write_object(data, MXT_SPT_CTECONFIG_T28,
				MXT_CTE_VOLTAGE, voltage);
	}
}

/* Update 24-bit CRC with two new bytes of data */
static u32 crc24_step(u32 crc, u8 byte1, u8 byte2)
{
	const u32 crcpoly = 0x80001b;
	u16 data = byte1 | (byte2 << 8);
	u32 result = data ^ (crc << 1);

	/* XOR result with crcpoly if bit 25 is set (overflow occurred) */
	if (result & 0x01000000)
		result ^= crcpoly;

	return result & 0x00ffffff;
}

static u32 crc24(u32 crc, const u8 *data, size_t len)
{
	size_t i;

	for (i = 0; i < len - 1; i += 2)
		crc = crc24_step(crc, data[i], data[i + 1]);

	/* If there were an odd number of bytes pad with 0 */
	if (i < len)
		crc = crc24_step(crc, data[i], 0);

	return crc;
}

static int mxt_verify_info_block_csum(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	size_t object_table_size, info_block_size;
	u32 crc = 0;
	u8 *info_block;
	int ret = 0;

	object_table_size = data->info.object_num * MXT_OBJECT_SIZE;
	info_block_size = sizeof(data->info) + object_table_size;
	info_block = kmalloc(info_block_size, GFP_KERNEL);
	if (!info_block)
		return -ENOMEM;

	/*
	 * Information Block CRC is computed over both ID info and Object Table
	 * So concat them in a temporary buffer, before computing CRC.
	 * TODO: refactor how the info block is read from the device such
	 * that it ends up in a single buffer and this copy is not needed.
	 */
	memcpy(info_block, &data->info, sizeof(data->info));
	memcpy(&info_block[sizeof(data->info)], data->object_table,
			object_table_size);

	crc = crc24(crc, info_block, info_block_size);

	if (crc != data->info_csum) {
		dev_err(dev, "Information Block CRC mismatch: %06x != %06x\n",
			data->info_csum, crc);
		ret = -EINVAL;
	}

	kfree(info_block);
	return ret;
}

static int mxt_get_info(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;

	/* Read 7-byte info block starting at address 0 */
	error = __mxt_read_reg(client, MXT_INFO, sizeof(*info), info);
	if (error)
		return error;

	return 0;
}

static int mxt_get_object_table(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &data->client->dev;
	size_t table_size;
	int error;
	int i;
	u8 reportid;
	u8 csum[3];

	table_size = data->info.object_num * sizeof(struct mxt_object);
	error = __mxt_read_reg(client, MXT_OBJECT_START, table_size,
			data->object_table);
	if (error)
		return error;

	/*
	 * Read Information Block checksum from 3 bytes immediately following
	 * info block
	 */
	error = __mxt_read_reg(client, MXT_OBJECT_START + table_size,
			sizeof(csum), csum);
	if (error)
		return error;

	data->info_csum = csum[0] | (csum[1] << 8) | (csum[2] << 16);
	dev_info(dev, "Information Block Checksum = %06x\n", data->info_csum);

	error = mxt_verify_info_block_csum(data);
	if (error)
		return error;

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	for (i = 0; i < data->info.object_num; i++) {
		struct mxt_object *object = data->object_table + i;
		u8 min_id, max_id;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids *
					mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		dev_dbg(&data->client->dev,
			"Type %2d Start %3d Size %3zu Instances %2zu ReportIDs %3u : %3u\n",
			object->type, object->start_address,
			mxt_obj_size(object), mxt_obj_instances(object),
			min_id, max_id);

		switch (object->type) {
		case MXT_GEN_MESSAGE_T5:
			data->T5_address = object->start_address;
			break;
		case MXT_GEN_COMMAND_T6:
			data->T6_reportid = min_id;
			break;
		case MXT_TOUCH_MULTI_T9:
			data->T9_reportid_min = min_id;
			data->T9_reportid_max = max_id;
			break;
		case MXT_SPT_GPIOPWM_T19:
			data->T19_reportid = min_id;
			break;
		case MXT_SPT_MESSAGECOUNT_T44:
			data->T44_address = object->start_address;
			break;
		}
	}

	return 0;
}

static void mxt_free_object_table(struct mxt_data *data)
{
	kfree(data->object_table);
	data->object_table = NULL;
	data->T6_reportid = 0;
	data->T9_reportid_min = 0;
	data->T9_reportid_max = 0;
	data->T19_reportid = 0;
}

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;

	error = mxt_get_info(data);
	if (error)
		return error;

	data->object_table = kcalloc(info->object_num,
				     sizeof(struct mxt_object),
				     GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt_get_object_table(data);
	if (error)
		goto err_free_object_table;

	/* Check register init values */
	error = mxt_check_reg_init(data);
	if (error)
		goto err_free_object_table;

	mxt_handle_pdata(data);

	/* Backup to memory */
	error = mxt_write_object(data, MXT_GEN_COMMAND_T6,
				 MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);
	if (error)
		return error;
	msleep(MXT_BACKUP_TIME);

	/* Soft reset */
	error = mxt_write_object(data, MXT_GEN_COMMAND_T6,
				 MXT_COMMAND_RESET, 1);
	if (error)
		return error;
	msleep(MXT_RESET_TIME);

	dev_info(&client->dev,
			"Family ID: %u Variant ID: %u Major.Minor.Build: %u.%u.%02X\n",
			info->family_id, info->variant_id, info->version >> 4,
			info->version & 0xf, info->build);

	dev_info(&client->dev,
			"Matrix X Size: %u Matrix Y Size: %u Object Num: %u\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);

	error = mxt_calc_resolution(data);
	if (error)
		return error;

	return 0;

err_free_object_table:
	mxt_free_object_table(data);
	return error;
}

static int mxt_calc_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	u8 orient;
	__le16 xyrange[2];
	unsigned int max_x, max_y;
	u8 xylines[2];
	int ret;

	struct mxt_object *T9 = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	if (T9 == NULL)
		return -EINVAL;

	/* Get touchscreen resolution */
	ret = __mxt_read_reg(client, T9->start_address + MXT_TOUCH_XRANGE_LSB,
			4, xyrange);
	if (ret)
		return ret;

	ret = __mxt_read_reg(client, T9->start_address + MXT_TOUCH_ORIENT,
			1, &orient);
	if (ret)
		return ret;

	ret = __mxt_read_reg(client, T9->start_address + MXT_TOUCH_XSIZE,
			2, xylines);
	if (ret)
		return ret;

	max_x = le16_to_cpu(xyrange[0]);
	max_y = le16_to_cpu(xyrange[1]);

	if (orient & MXT_XY_SWITCH) {
		data->max_x = max_y;
		data->max_y = max_x;
	} else {
		data->max_x = max_x;
		data->max_y = max_y;
	}

	data->max_area_pixels = max_x * max_y;
	data->max_area_channels = xylines[0] * xylines[1];

	return 0;
}

/*
 * Helper function for performing a T6 diagnostic command
 */
static int mxt_T6_diag_cmd(struct mxt_data *data, struct mxt_object *T6,
			   u8 cmd)
{
	int ret;
	u16 addr = T6->start_address + MXT_COMMAND_DIAGNOSTIC;

	ret = mxt_write_reg(data->client, addr, cmd);
	if (ret)
		return ret;

	/*
	 * Poll T6.diag until it returns 0x00, which indicates command has
	 * completed.
	 */
	while (cmd != 0) {
		ret = __mxt_read_reg(data->client, addr, 1, &cmd);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * SysFS Helper function for reading DELTAS and REFERENCE values for T37 object
 *
 * For both modes, a T37_buf is allocated to stores matrix_xsize * matrix_ysize
 * 2-byte (little-endian) values, which are returned to userspace unmodified.
 *
 * It is left to userspace to parse the 2-byte values.
 * - deltas are signed 2's complement 2-byte little-endian values.
 *     s32 delta = (b[0] + (b[1] << 8));
 * - refs are signed 'offset binary' 2-byte little-endian values, with offset
 *   value 0x4000:
 *     s32 ref = (b[0] + (b[1] << 8)) - 0x4000;
 */
static ssize_t mxt_T37_fetch(struct mxt_data *data, u8 mode)
{
	struct mxt_object *T6, *T37;
	u8 *obuf;
	ssize_t ret = 0;
	size_t i;
	size_t T37_buf_size, num_pages;
	size_t pos;

	if (!data || !data->object_table)
		return -ENODEV;

	T6 = mxt_get_object(data, MXT_GEN_COMMAND_T6);
	T37 = mxt_get_object(data, MXT_DEBUG_DIAGNOSTIC_T37);
	if (!T6 || mxt_obj_size(T6) < 6 || !T37 || mxt_obj_size(T37) < 3) {
		dev_err(&data->client->dev, "Invalid T6 or T37 object\n");
		return -ENODEV;
	}

	/* Something has gone wrong if T37_buf is already allocated */
	if (data->T37_buf)
		return -EINVAL;

	T37_buf_size = data->info.matrix_xsize * data->info.matrix_ysize *
		       sizeof(__le16);
	data->T37_buf_size = T37_buf_size;
	data->T37_buf = kmalloc(data->T37_buf_size, GFP_KERNEL);
	if (!data->T37_buf)
		return -ENOMEM;

	/* Temporary buffer used to fetch one T37 page */
	obuf = kmalloc(mxt_obj_size(T37), GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	disable_irq(data->irq);
	num_pages = DIV_ROUND_UP(T37_buf_size, mxt_obj_size(T37) - 2);
	pos = 0;
	for (i = 0; i < num_pages; i++) {
		u8 cmd;
		size_t chunk_len;

		/* For first page, send mode as cmd, otherwise PageUp */
		cmd = (i == 0) ? mode : MXT_T6_CMD_PAGE_UP;
		ret = mxt_T6_diag_cmd(data, T6, cmd);
		if (ret)
			goto err_free_T37_buf;

		ret = __mxt_read_reg(data->client, T37->start_address,
				mxt_obj_size(T37), obuf);
		if (ret)
			goto err_free_T37_buf;

		/* Verify first two bytes are current mode and page # */
		if (obuf[0] != mode) {
			dev_err(&data->client->dev,
				"Unexpected mode (%u != %u)\n", obuf[0], mode);
			ret = -EIO;
			goto err_free_T37_buf;
		}

		if (obuf[1] != i) {
			dev_err(&data->client->dev,
				"Unexpected page (%u != %zu)\n", obuf[1], i);
			ret = -EIO;
			goto err_free_T37_buf;
		}

		/*
		 * Copy the data portion of the page, or however many bytes are
		 * left, whichever is less.
		 */
		chunk_len = min(mxt_obj_size(T37) - 2, T37_buf_size - pos);
		memcpy(&data->T37_buf[pos], &obuf[2], chunk_len);
		pos += chunk_len;
	}

	goto out;

err_free_T37_buf:
	kfree(data->T37_buf);
	data->T37_buf = NULL;
	data->T37_buf_size = 0;
out:
	kfree(obuf);
	enable_irq(data->irq);
	return ret ?: 0;
}

static ssize_t mxt_backupnv_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret;

	/* Backup non-volatile memory */
	ret = mxt_write_object(data, MXT_GEN_COMMAND_T6,
			       MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);
	if (ret)
		return ret;
	msleep(MXT_BACKUP_TIME);

	return count;
}

static ssize_t mxt_calibrate_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret;

	disable_irq(data->irq);

	/* Perform touch surface recalibration */
	ret = mxt_write_object(data, MXT_GEN_COMMAND_T6,
			MXT_COMMAND_CALIBRATE, 1);
	if (ret)
		goto out;
	msleep(MXT_CAL_TIME);

out:
	enable_irq(data->irq);
	return ret ?: count;
}

static ssize_t mxt_config_csum_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%06x\n", data->config_csum);
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_fw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_info *info = &data->info;
	return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X\n",
			 info->version >> 4, info->version & 0xf, info->build);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_info *info = &data->info;
	return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
			 info->family_id, info->variant_id);
}

static ssize_t mxt_info_csum_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%06x\n", data->info_csum);
}

/* Matrix Size is <MatrixSizeX> <MatrixSizeY> */
static ssize_t mxt_matrix_size_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_info *info = &data->info;
	return scnprintf(buf, PAGE_SIZE, "%u %u\n",
			 info->matrix_xsize, info->matrix_ysize);
}

static ssize_t mxt_show_instance(char *buf, int count,
				 struct mxt_object *object, int instance,
				 const u8 *val)
{
	int i;

	if (mxt_obj_instances(object) > 1)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "Instance %u\n", instance);

	for (i = 0; i < mxt_obj_size(object); i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"\t[%2u]: %02x (%d)\n", i, val[i], val[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t mxt_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 *obuf;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	error = 0;
	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_readable(object->type))
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"T%u:\n", object->type);

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = __mxt_read_reg(data->client, addr, size, obuf);
			if (error)
				goto done;

			count = mxt_show_instance(buf, count, object, j, obuf);
		}
	}

done:
	kfree(obuf);
	return error ?: count;
}

static ssize_t mxt_object_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret;
	u32 param;
	u8 type, instance, offset, val;

	ret = kstrtou32(buf, 16, &param);
	if (ret < 0)
		return -EINVAL;

	/*
	 * Byte Write Command is encoded in 32-bit word: TTIIOOVV:
	 * <Type> <Instance> <Offset> <Value>
	 */
	type = (param & 0xff000000) >> 24;
	instance = (param & 0x00ff0000) >> 16;
	offset = (param & 0x0000ff00) >> 8;
	val = param & 0x000000ff;

	ret = mxt_write_obj_instance(data, type, instance, offset, val);
	if (ret)
		return ret;

	return count;
}

static int mxt_load_fw(struct device *dev, const char *fn)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	int ret;

	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		return ret;
	}

	ret = mxt_enter_bl(data);
	if (ret) {
		dev_err(dev, "Failed to reset to bootloader.\n");
		goto out;
	}

	ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD);
	if (ret)
		goto out;

	init_completion(&data->bl_completion);
	/* Unlock bootloader */
	ret = mxt_unlock_bootloader(client);
	if (ret)
		goto out;

	while (pos < fw->size) {
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA);
		if (ret)
			goto out;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* We should add 2 at frame size as the the firmware data is not
		 * included the CRC bytes.
		 */
		frame_size += 2;

		/* Write one frame to device */
		ret = mxt_fw_write(client, fw->data + pos, frame_size);
		if (ret)
			goto out;

		ret = mxt_check_bootloader(data, MXT_FRAME_CRC_PASS);
		if (ret)
			goto out;

		pos += frame_size;

		dev_dbg(dev, "Updated %d bytes / %zd bytes\n", pos, fw->size);
	}

	/* Device exits bl mode to app mode only if successful */
	mxt_exit_bl(data);
out:
	release_firmware(fw);

	return ret;
}

static ssize_t mxt_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int error;

	error = mxt_load_fw(dev, MXT_FW_NAME);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_dbg(dev, "The firmware update succeeded\n");
	}

	return count;
}

static DEVICE_ATTR(backupnv, S_IWUSR, NULL, mxt_backupnv_store);
static DEVICE_ATTR(calibrate, S_IWUSR, NULL, mxt_calibrate_store);
static DEVICE_ATTR(config_csum, S_IRUGO, mxt_config_csum_show, NULL);
static DEVICE_ATTR(fw_version, S_IRUGO, mxt_fw_version_show, NULL);
static DEVICE_ATTR(hw_version, S_IRUGO, mxt_hw_version_show, NULL);
static DEVICE_ATTR(info_csum, S_IRUGO, mxt_info_csum_show, NULL);
static DEVICE_ATTR(matrix_size, S_IRUGO, mxt_matrix_size_show, NULL);
static DEVICE_ATTR(object, S_IRUGO | S_IWUSR, mxt_object_show,
		   mxt_object_store);
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, mxt_update_fw_store);

static struct attribute *mxt_attrs[] = {
	&dev_attr_backupnv.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_config_csum.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_info_csum.attr,
	&dev_attr_matrix_size.attr,
	&dev_attr_object.attr,
	&dev_attr_update_fw.attr,
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

/*
 **************************************************************
 * debugfs interface
 **************************************************************
*/
static int mxt_debugfs_T37_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt = inode->i_private;
	int ret;
	u8 cmd;

	if (file->f_dentry == mxt->dentry_deltas)
		cmd = MXT_T6_CMD_DELTAS;
	else if (file->f_dentry == mxt->dentry_refs)
		cmd = MXT_T6_CMD_REFS;
	else
		return -EINVAL;

	/* Only allow one T37 debugfs file to be opened at a time */
	ret = mutex_lock_interruptible(&mxt->T37_buf_mutex);
	if (ret)
		return ret;

	if (!i2c_use_client(mxt->client)) {
		ret = -ENODEV;
		goto err_unlock;
	}

	/* Fetch all T37 pages into mxt->T37_buf */
	ret = mxt_T37_fetch(mxt, cmd);
	if (ret)
		goto err_release;

	file->private_data = mxt;

	return 0;

err_release:
	i2c_release_client(mxt->client);
err_unlock:
	mutex_unlock(&mxt->T37_buf_mutex);
	return ret;
}

static int mxt_debugfs_T37_release(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt = file->private_data;

	file->private_data = NULL;

	kfree(mxt->T37_buf);
	mxt->T37_buf = NULL;
	mxt->T37_buf_size = 0;

	i2c_release_client(mxt->client);
	mutex_unlock(&mxt->T37_buf_mutex);

	return 0;
}


/* Return some bytes from the buffered T37 object, starting from *ppos */
static ssize_t mxt_debugfs_T37_read(struct file *file, char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct mxt_data *mxt = file->private_data;

	if (!mxt->T37_buf)
		return -ENODEV;

	if (*ppos >= mxt->T37_buf_size)
		return 0;

	if (count + *ppos > mxt->T37_buf_size)
		count = mxt->T37_buf_size - *ppos;

	if (copy_to_user(buffer, &mxt->T37_buf[*ppos], count))
		return -EFAULT;

	*ppos += count;

	return count;
}

static const struct file_operations mxt_debugfs_T37_fops = {
	.owner = THIS_MODULE,
	.open = mxt_debugfs_T37_open,
	.release = mxt_debugfs_T37_release,
	.read = mxt_debugfs_T37_read
};

static int mxt_debugfs_init(struct mxt_data *mxt)
{
	struct device *dev = &mxt->client->dev;

	if (!mxt_debugfs_root)
		return -ENODEV;

	mxt->dentry_dev = debugfs_create_dir(kobject_name(&dev->kobj),
					     mxt_debugfs_root);

	if (!mxt->dentry_dev)
		return -ENODEV;

	mutex_init(&mxt->T37_buf_mutex);

	mxt->dentry_deltas = debugfs_create_file("deltas", S_IRUSR,
						 mxt->dentry_dev, mxt,
						 &mxt_debugfs_T37_fops);
	mxt->dentry_refs = debugfs_create_file("refs", S_IRUSR,
					       mxt->dentry_dev, mxt,
					       &mxt_debugfs_T37_fops);
	return 0;
}

static int mxt_save_regs(struct mxt_data *data, u8 type, u8 instance,
			 u8 offset, u8 *val, u16 size)
{
	struct mxt_object *object;
	u16 addr;
	int ret;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	addr = object->start_address + instance * mxt_obj_size(object) + offset;
	ret = __mxt_read_reg(data->client, addr, size, val);
	if (ret)
		return -EINVAL;

	return 0;
}

static int mxt_set_regs(struct mxt_data *data, u8 type, u8 instance,
			u8 offset, const u8 *val, u16 size)
{
	struct mxt_object *object;
	u16 addr;
	int ret;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	addr = object->start_address + instance * mxt_obj_size(object) + offset;
	ret = __mxt_write_reg(data->client, addr, size, val);
	if (ret)
		return -EINVAL;

	return 0;
}

static void mxt_start(struct mxt_data *data)
{
	/* Touch enable */
	mxt_write_object(data,
			MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, 0x83);
}

static void mxt_stop(struct mxt_data *data)
{
	/* Touch disable */
	mxt_write_object(data,
			MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, 0);
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_start(data);

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_stop(data);
}

static int mxt_input_dev_create(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	struct input_dev *input_dev;
	int error;
	unsigned int num_mt_slots;
	int max_area_channels;
	int max_touch_major;

	data->input_dev = input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	if (pdata && pdata->is_tp)
		data->is_tp = true;

	input_dev->name = (data->is_tp) ? "Atmel maXTouch Touchpad" :
					  "Atmel maXTouch Touchscreen";
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &data->client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	if (data->is_tp) {
		int i;
		__set_bit(INPUT_PROP_POINTER, input_dev->propbit);
		__set_bit(INPUT_PROP_BUTTONPAD, input_dev->propbit);

		if (!pdata)
			__set_bit(BTN_LEFT, input_dev->keybit);
		for (i = 0; i < MXT_NUM_GPIO; i++)
			if (pdata && pdata->key_map[i] != KEY_RESERVED)
				__set_bit(pdata->key_map[i], input_dev->keybit);

		__set_bit(BTN_TOOL_FINGER, input_dev->keybit);
		__set_bit(BTN_TOOL_DOUBLETAP, input_dev->keybit);
		__set_bit(BTN_TOOL_TRIPLETAP, input_dev->keybit);
		__set_bit(BTN_TOOL_QUADTAP, input_dev->keybit);
		__set_bit(BTN_TOOL_QUINTTAP, input_dev->keybit);

		input_abs_set_res(input_dev, ABS_X, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_Y, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_X,
				  MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_Y,
				  MXT_PIXELS_PER_MM);
	}

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			     0, 255, 0, 0);
	input_abs_set_res(input_dev, ABS_X, MXT_PIXELS_PER_MM);
	input_abs_set_res(input_dev, ABS_Y, MXT_PIXELS_PER_MM);

	/* For multi touch */
	num_mt_slots = data->T9_reportid_max - data->T9_reportid_min + 1;
	error = input_mt_init_slots(input_dev, num_mt_slots, 0);
	if (error)
		goto err_free_device;

	max_area_channels = min(255U, data->max_area_channels);
	max_touch_major = get_touch_major_pixels(data, max_area_channels);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, max_touch_major, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);
	input_abs_set_res(input_dev, ABS_MT_POSITION_X, MXT_PIXELS_PER_MM);
	input_abs_set_res(input_dev, ABS_MT_POSITION_Y, MXT_PIXELS_PER_MM);

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error)
		goto err_free_device;

	return 0;

err_free_device:
	input_free_device(data->input_dev);
	data->input_dev = NULL;
	return error;
}

static int __devinit mxt_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct mxt_platform_data *pdata = dev_get_platdata(&client->dev);
	struct mxt_data *data;
	int error;

	if (!pdata)
		return -EINVAL;

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	data->is_tp = !strcmp(id->name, "atmel_mxt_tp");
	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);

	data->client = client;
	i2c_set_clientdata(client, data);

	data->pdata = pdata;
	data->irq = client->irq;

	init_completion(&data->bl_completion);

	if (mxt_in_bootloader(data)) {
		dev_info(&client->dev, "Device in bootloader at probe\n");
	} else {
		error = mxt_initialize(data);
		if (error)
			goto err_free_mem;

		error = mxt_input_dev_create(data);
		if (error)
			goto err_free_object;
	}

	error = request_threaded_irq(client->irq, NULL, mxt_interrupt,
				     pdata->irqflags | IRQF_ONESHOT,
				     client->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		if (mxt_in_bootloader(data))
			goto err_free_mem;
		else
			goto err_unregister_device;
	}

	if (!mxt_in_bootloader(data)) {
		error = mxt_handle_messages(data);
		if (error)
			goto err_free_irq;
	}

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error)
		goto err_free_irq;

	error = mxt_debugfs_init(data);
	if (error)
		dev_warn(&client->dev, "error creating debugfs entries.\n");

	return 0;

err_free_irq:
	free_irq(client->irq, data);
err_unregister_device:
	input_unregister_device(data->input_dev);
err_free_object:
	kfree(data->object_table);
err_free_mem:
	kfree(data);
	return error;
}

static int mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	if (data->dentry_dev) {
		debugfs_remove_recursive(data->dentry_dev);
		mutex_destroy(&data->T37_buf_mutex);
		kfree(data->T37_buf);
	}
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
	free_irq(data->irq, data);
	if (data->input_dev)
		input_unregister_device(data->input_dev);
	kfree(data->object_table);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	static const u8 T7_config_idle[3] = { 0xfe, 0xfe, 0x00 };
	static const u8 T7_config_deepsleep[3] = { 0x00, 0x00, 0x00 };
	const u8 *power_config;
	u8 T9_ctrl = 0x03;
	int ret;

	if (mxt_in_bootloader(data))
		return 0;

	mutex_lock(&input_dev->mutex);

	/* Save 3 bytes T7 Power config */
	ret = mxt_save_regs(data, MXT_GEN_POWER_T7, 0, 0,
			    data->T7_config, 3);
	if (ret)
		dev_err(dev, "Save T7 Power config failed, %d\n", ret);
	data->T7_config_valid = (ret == 0);

	/*
	 * Set T7 to idle mode if we allow wakeup from touch, otherwise
	 * put it into deepsleep mode.
	 */
	power_config = device_may_wakeup(dev) ? T7_config_idle
					      : T7_config_deepsleep;

	ret = mxt_set_regs(data, MXT_GEN_POWER_T7, 0, 0,
			   power_config, 3);
	if (ret)
		dev_err(dev, "Set T7 Power config failed, %d\n", ret);

	if (device_may_wakeup(dev)) {
		/*
		 * If we allow wakeup from touch, we have to enable T9 so
		 * that IRQ can be generated from touch
		 */

		/* Save 1 byte T9 Ctrl config */
		ret = mxt_save_regs(data, MXT_TOUCH_MULTI_T9, 0, 0,
				    &data->T9_ctrl, 1);
		if (ret)
			dev_err(dev, "Save T9 ctrl config failed, %d\n", ret);
		data->T9_ctrl_valid = (ret == 0);

		/* Enable T9 object */
		ret = mxt_set_regs(data, MXT_TOUCH_MULTI_T9, 0, 0,
				   &T9_ctrl, 1);
		if (ret)
			dev_err(dev, "Set T9 ctrl config failed, %d\n", ret);

		/* Enable wake from IRQ */
		data->irq_wake = (enable_irq_wake(data->irq) == 0);
	} else if (input_dev->users) {
		mxt_stop(data);
	}

	disable_irq(data->irq);

	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	int ret;

	if (mxt_in_bootloader(data))
		return 0;

	/* Process any pending message so that CHG line can be de-asserted */
	ret = mxt_handle_messages(data);
	if (ret)
		dev_err(dev, "Handling message fails upon resume, %d\n", ret);

	mutex_lock(&input_dev->mutex);

	enable_irq(data->irq);

	if (device_may_wakeup(dev) && data->irq_wake)
		disable_irq_wake(data->irq);

	/* Restore the T9 Ctrl config to before-suspend value */
	if (device_may_wakeup(dev) && data->T9_ctrl_valid) {
		ret = mxt_set_regs(data, MXT_TOUCH_MULTI_T9, 0, 0,
				   &data->T9_ctrl, 1);
		if (ret)
			dev_err(dev, "Set T9 ctrl config failed, %d\n", ret);
	} else if (input_dev->users) {
		mxt_start(data);
	}

	/* Restore the T7 Power config to before-suspend value */
	if (data->T7_config_valid) {
		ret = mxt_set_regs(data, MXT_GEN_POWER_T7, 0, 0,
				   data->T7_config, 3);
		if (ret)
			dev_err(dev, "Set T7 power config failed, %d\n", ret);
	}

	mutex_unlock(&input_dev->mutex);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(mxt_pm_ops, mxt_suspend, mxt_resume);

static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "atmel_mxt_tp", 0 },
	{ "mXT224", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
		.pm	= &mxt_pm_ops,
	},
	.probe		= mxt_probe,
	.remove		= mxt_remove,
	.id_table	= mxt_id,
};

static int __init mxt_init(void)
{
	/* Create a global debugfs root for all atmel_mxt_ts devices */
	mxt_debugfs_root = debugfs_create_dir(mxt_driver.driver.name, NULL);
	if (mxt_debugfs_root == ERR_PTR(-ENODEV))
		mxt_debugfs_root = NULL;

	return i2c_add_driver(&mxt_driver);
}

static void __exit mxt_exit(void)
{
	if (mxt_debugfs_root)
		debugfs_remove_recursive(mxt_debugfs_root);

	i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");
