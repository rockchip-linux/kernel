/*
 * Cypress APA trackpad with I2C interface
 *
 * Author: Dudley Du <dudl@cypress.com>
 * Further cleanup and restructuring by:
 *   Daniel Kurtz <djkurtz@chromium.org>
 *   Benson Leung <bleung@chromium.org>
 *
 * Copyright (C) 2011-2012 Cypress Semiconductor, Inc.
 * Copyright (C) 2011-2012 Google, Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/async.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/acpi.h>

/* APA trackpad firmware generation */
#define CYAPA_GEN3   0x03   /* support MT-protocol B with tracking ID. */

#define CYAPA_NAME   "Cypress APA Trackpad (cyapa)"

/* commands for read/write registers of Cypress trackpad */
#define CYAPA_CMD_SOFT_RESET       0x00
#define CYAPA_CMD_POWER_MODE       0x01
#define CYAPA_CMD_DEV_STATUS       0x02
#define CYAPA_CMD_GROUP_DATA       0x03
#define CYAPA_CMD_GROUP_CMD        0x04
#define CYAPA_CMD_GROUP_QUERY      0x05
#define CYAPA_CMD_BL_STATUS        0x06
#define CYAPA_CMD_BL_HEAD          0x07
#define CYAPA_CMD_BL_CMD           0x08
#define CYAPA_CMD_BL_DATA          0x09
#define CYAPA_CMD_BL_ALL           0x0a
#define CYAPA_CMD_BLK_PRODUCT_ID   0x0b
#define CYAPA_CMD_BLK_HEAD         0x0c
#define CYAPA_CMD_MAX_BASELINE     0x0d
#define CYAPA_CMD_MIN_BASELINE     0x0e

/* report data start reg offset address. */
#define DATA_REG_START_OFFSET  0x0000

#define BL_HEAD_OFFSET 0x00
#define BL_DATA_OFFSET 0x10

/*
 * Operational Device Status Register
 *
 * bit 7: Valid interrupt source
 * bit 6 - 4: Reserved
 * bit 3 - 2: Power status
 * bit 1 - 0: Device status
 */
#define REG_OP_STATUS     0x00
#define OP_STATUS_SRC     0x80
#define OP_STATUS_POWER   0x0c
#define OP_STATUS_DEV     0x03
#define OP_STATUS_MASK (OP_STATUS_SRC | OP_STATUS_POWER | OP_STATUS_DEV)

/*
 * Operational Finger Count/Button Flags Register
 *
 * bit 7 - 4: Number of touched finger
 * bit 3: Valid data
 * bit 2: Middle Physical Button
 * bit 1: Right Physical Button
 * bit 0: Left physical Button
 */
#define REG_OP_DATA1       0x01
#define OP_DATA_VALID      0x08
#define OP_DATA_MIDDLE_BTN 0x04
#define OP_DATA_RIGHT_BTN  0x02
#define OP_DATA_LEFT_BTN   0x01
#define OP_DATA_BTN_MASK (OP_DATA_MIDDLE_BTN | OP_DATA_RIGHT_BTN | \
			  OP_DATA_LEFT_BTN)

/*
 * Bootloader Status Register
 *
 * bit 7: Busy
 * bit 6 - 5: Reserved
 * bit 4: Bootloader running
 * bit 3 - 2: Reserved
 * bit 1: Watchdog Reset
 * bit 0: Checksum valid
 */
#define REG_BL_STATUS        0x01
#define BL_STATUS_REV_6_5    0x60
#define BL_STATUS_BUSY       0x80
#define BL_STATUS_RUNNING    0x10
#define BL_STATUS_REV_3_2    0x0c
#define BL_STATUS_WATCHDOG   0x02
#define BL_STATUS_CSUM_VALID 0x01
#define BL_STATUS_REV_MASK (BL_STATUS_WATCHDOG | BL_STATUS_REV_3_2 | \
			    BL_STATUS_REV_6_5)

/*
 * Bootloader Error Register
 *
 * bit 7: Invalid
 * bit 6: Invalid security key
 * bit 5: Bootloading
 * bit 4: Command checksum
 * bit 3: Flash protection error
 * bit 2: Flash checksum error
 * bit 1 - 0: Reserved
 */
#define REG_BL_ERROR         0x02
#define BL_ERROR_INVALID     0x80
#define BL_ERROR_INVALID_KEY 0x40
#define BL_ERROR_BOOTLOADING 0x20
#define BL_ERROR_CMD_CSUM    0x10
#define BL_ERROR_FLASH_PROT  0x08
#define BL_ERROR_FLASH_CSUM  0x04
#define BL_ERROR_RESERVED    0x03

#define BL_STATUS_SIZE  3  /* length of bootloader status registers */
#define BLK_HEAD_BYTES 32

/* Macro for register map group offset. */
#define CYAPA_REG_MAP_SIZE  256

#define PRODUCT_ID_SIZE  16
#define QUERY_DATA_SIZE  27
#define REG_PROTOCOL_GEN_QUERY_OFFSET  20

#define REG_OFFSET_DATA_BASE     0x0000
#define REG_OFFSET_COMMAND_BASE  0x0028
#define REG_OFFSET_QUERY_BASE    0x002a

#define CAPABILITY_LEFT_BTN_MASK	(0x01 << 3)
#define CAPABILITY_RIGHT_BTN_MASK	(0x01 << 4)
#define CAPABILITY_MIDDLE_BTN_MASK	(0x01 << 5)
#define CAPABILITY_BTN_MASK  (CAPABILITY_LEFT_BTN_MASK | \
			      CAPABILITY_RIGHT_BTN_MASK | \
			      CAPABILITY_MIDDLE_BTN_MASK)

#define CYAPA_OFFSET_SOFT_RESET  REG_OFFSET_COMMAND_BASE
#define OP_RECALIBRATION_MASK    0x80
#define OP_REPORT_BASELINE_MASK  0x40
#define REG_OFFSET_MAX_BASELINE  0x0026
#define REG_OFFSET_MIN_BASELINE  0x0027

#define REG_OFFSET_POWER_MODE (REG_OFFSET_COMMAND_BASE + 1)
#define SET_POWER_MODE_DELAY   10000  /* unit: us */
#define SET_POWER_MODE_TRIES   5

#define PWR_MODE_MASK   0xfc
#define PWR_MODE_FULL_ACTIVE (0x3f << 2)
#define PWR_MODE_IDLE        (0x03 << 2) /* default rt suspend scanrate: 30ms */
#define PWR_MODE_SLEEP       (0x05 << 2) /* default suspend scanrate: 50ms */
#define PWR_MODE_BTN_ONLY    (0x01 << 2)
#define PWR_MODE_OFF         (0x00 << 2)

#define BTN_ONLY_MODE_NAME   "buttononly"

#define PWR_STATUS_MASK      0x0c
#define PWR_STATUS_ACTIVE    (0x03 << 2)
#define PWR_STATUS_IDLE      (0x02 << 2)
#define PWR_STATUS_BTN_ONLY  (0x01 << 2)
#define PWR_STATUS_OFF       (0x00 << 2)

#define AUTOSUSPEND_DELAY   2000 /* unit : ms */

/*
 * CYAPA trackpad device states.
 * Used in register 0x00, bit1-0, DeviceStatus field.
 * Other values indicate device is in an abnormal state and must be reset.
 */
#define CYAPA_DEV_NORMAL  0x03
#define CYAPA_DEV_BUSY    0x01

enum cyapa_state {
	CYAPA_STATE_OP,
	CYAPA_STATE_BL_IDLE,
	CYAPA_STATE_BL_ACTIVE,
	CYAPA_STATE_BL_BUSY,
	CYAPA_STATE_NO_DEVICE,
};

struct cyapa_touch {
	/*
	 * high bits or x/y position value
	 * bit 7 - 4: high 4 bits of x position value
	 * bit 3 - 0: high 4 bits of y position value
	 */
	u8 xy_hi;
	u8 x_lo;  /* low 8 bits of x position value. */
	u8 y_lo;  /* low 8 bits of y position value. */
	u8 pressure;
	/* id range is 1 - 15.  It is incremented with every new touch. */
	u8 id;
} __packed;

/* The touch.id is used as the MT slot id, thus max MT slot is 15 */
#define CYAPA_MAX_MT_SLOTS  15

struct cyapa_reg_data {
	/*
	 * bit 0 - 1: device status
	 * bit 3 - 2: power mode
	 * bit 6 - 4: reserved
	 * bit 7: interrupt valid bit
	 */
	u8 device_status;
	/*
	 * bit 7 - 4: number of fingers currently touching pad
	 * bit 3: valid data check bit
	 * bit 2: middle mechanism button state if exists
	 * bit 1: right mechanism button state if exists
	 * bit 0: left mechanism button state if exists
	 */
	u8 finger_btn;
	/* CYAPA reports up to 5 touches per packet. */
	struct cyapa_touch touches[5];
} __packed;

/* The main device structure */
struct cyapa {
	enum cyapa_state state;
	u8 status[BL_STATUS_SIZE];

	struct i2c_client *client;
	struct input_dev *input;
	char phys[32];	/* device physical location */
	int irq;
	bool irq_wake;  /* irq wake is enabled */
	bool smbus;

	bool debug;

	/* power mode settings */
	u8 suspend_power_mode;
#ifdef CONFIG_PM_RUNTIME
	u8 runtime_suspend_power_mode;
#endif /* CONFIG_PM_RUNTIME */

	/* read from query data region. */
	char product_id[16];
	u8 fw_maj_ver;  /* firmware major version. */
	u8 fw_min_ver;  /* firmware minor version. */
	u8 btn_capability;
	u8 gen;
	int max_abs_x;
	int max_abs_y;
	int physical_size_x;
	int physical_size_y;

	bool lid_handler_registered;
	struct input_handler lid_handler;

	struct mutex debugfs_mutex;

	/* per-instance debugfs root */
	struct dentry *dentry_dev;

	/* Buffer to store firmware read using debugfs */
	u8 *read_fw_image;
};

static const u8 bl_activate[] = { 0x00, 0xff, 0x38, 0x00, 0x01, 0x02, 0x03,
		0x04, 0x05, 0x06, 0x07 };
static const u8 bl_deactivate[] = { 0x00, 0xff, 0x3b, 0x00, 0x01, 0x02, 0x03,
		0x04, 0x05, 0x06, 0x07 };
static const u8 bl_exit[] = { 0x00, 0xff, 0xa5, 0x00, 0x01, 0x02, 0x03, 0x04,
		0x05, 0x06, 0x07 };

/* global root node of the cyapa debugfs directory. */
static struct dentry *cyapa_debugfs_root;

struct cyapa_cmd_len {
	u8 cmd;
	u8 len;
};

#define CYAPA_ADAPTER_FUNC_NONE   0
#define CYAPA_ADAPTER_FUNC_I2C    1
#define CYAPA_ADAPTER_FUNC_SMBUS  2
#define CYAPA_ADAPTER_FUNC_BOTH   3

/*
 * macros for SMBus communication
 */
#define SMBUS_READ   0x01
#define SMBUS_WRITE 0x00
#define SMBUS_ENCODE_IDX(cmd, idx) ((cmd) | (((idx) & 0x03) << 1))
#define SMBUS_ENCODE_RW(cmd, rw) ((cmd) | ((rw) & 0x01))
#define SMBUS_BYTE_BLOCK_CMD_MASK 0x80
#define SMBUS_GROUP_BLOCK_CMD_MASK 0x40

 /* for byte read/write command */
#define CMD_RESET 0
#define CMD_POWER_MODE 1
#define CMD_DEV_STATUS 2
#define CMD_REPORT_MAX_BASELINE 3
#define CMD_REPORT_MIN_BASELINE 4
#define SMBUS_BYTE_CMD(cmd) (((cmd) & 0x3f) << 1)
#define CYAPA_SMBUS_RESET SMBUS_BYTE_CMD(CMD_RESET)
#define CYAPA_SMBUS_POWER_MODE SMBUS_BYTE_CMD(CMD_POWER_MODE)
#define CYAPA_SMBUS_DEV_STATUS SMBUS_BYTE_CMD(CMD_DEV_STATUS)
#define CYAPA_SMBUS_MAX_BASELINE SMBUS_BYTE_CMD(CMD_REPORT_MAX_BASELINE)
#define CYAPA_SMBUS_MIN_BASELINE SMBUS_BYTE_CMD(CMD_REPORT_MIN_BASELINE)

 /* for group registers read/write command */
#define REG_GROUP_DATA 0
#define REG_GROUP_CMD 2
#define REG_GROUP_QUERY 3
#define SMBUS_GROUP_CMD(grp) (0x80 | (((grp) & 0x07) << 3))
#define CYAPA_SMBUS_GROUP_DATA SMBUS_GROUP_CMD(REG_GROUP_DATA)
#define CYAPA_SMBUS_GROUP_CMD SMBUS_GROUP_CMD(REG_GROUP_CMD)
#define CYAPA_SMBUS_GROUP_QUERY SMBUS_GROUP_CMD(REG_GROUP_QUERY)

 /* for register block read/write command */
#define CMD_BL_STATUS 0
#define CMD_BL_HEAD 1
#define CMD_BL_CMD 2
#define CMD_BL_DATA 3
#define CMD_BL_ALL 4
#define CMD_BLK_PRODUCT_ID 5
#define CMD_BLK_HEAD 6
#define SMBUS_BLOCK_CMD(cmd) (0xc0 | (((cmd) & 0x1f) << 1))

/* register block read/write command in bootloader mode */
#define CYAPA_SMBUS_BL_STATUS SMBUS_BLOCK_CMD(CMD_BL_STATUS)
#define CYAPA_SMBUS_BL_HEAD SMBUS_BLOCK_CMD(CMD_BL_HEAD)
#define CYAPA_SMBUS_BL_CMD SMBUS_BLOCK_CMD(CMD_BL_CMD)
#define CYAPA_SMBUS_BL_DATA SMBUS_BLOCK_CMD(CMD_BL_DATA)
#define CYAPA_SMBUS_BL_ALL SMBUS_BLOCK_CMD(CMD_BL_ALL)

/* register block read/write command in operational mode */
#define CYAPA_SMBUS_BLK_PRODUCT_ID SMBUS_BLOCK_CMD(CMD_BLK_PRODUCT_ID)
#define CYAPA_SMBUS_BLK_HEAD SMBUS_BLOCK_CMD(CMD_BLK_HEAD)

static const struct cyapa_cmd_len cyapa_i2c_cmds[] = {
	{ CYAPA_OFFSET_SOFT_RESET, 1 },
	{ REG_OFFSET_COMMAND_BASE + 1, 1 },
	{ REG_OFFSET_DATA_BASE, 1 },
	{ REG_OFFSET_DATA_BASE, sizeof(struct cyapa_reg_data) },
	{ REG_OFFSET_COMMAND_BASE, 0 },
	{ REG_OFFSET_QUERY_BASE, QUERY_DATA_SIZE },
	{ BL_HEAD_OFFSET, 3 },
	{ BL_HEAD_OFFSET, 16 },
	{ BL_HEAD_OFFSET, 16 },
	{ BL_DATA_OFFSET, 16 },
	{ BL_HEAD_OFFSET, 32 },
	{ REG_OFFSET_QUERY_BASE, PRODUCT_ID_SIZE },
	{ REG_OFFSET_DATA_BASE, 32 },
	{ REG_OFFSET_MAX_BASELINE, 1 },
	{ REG_OFFSET_MIN_BASELINE, 1 },
};

static const struct cyapa_cmd_len cyapa_smbus_cmds[] = {
	{ CYAPA_SMBUS_RESET, 1 },
	{ CYAPA_SMBUS_POWER_MODE, 1 },
	{ CYAPA_SMBUS_DEV_STATUS, 1 },
	{ CYAPA_SMBUS_GROUP_DATA, sizeof(struct cyapa_reg_data) },
	{ CYAPA_SMBUS_GROUP_CMD, 2 },
	{ CYAPA_SMBUS_GROUP_QUERY, QUERY_DATA_SIZE },
	{ CYAPA_SMBUS_BL_STATUS, 3 },
	{ CYAPA_SMBUS_BL_HEAD, 16 },
	{ CYAPA_SMBUS_BL_CMD, 16 },
	{ CYAPA_SMBUS_BL_DATA, 16 },
	{ CYAPA_SMBUS_BL_ALL, 32 },
	{ CYAPA_SMBUS_BLK_PRODUCT_ID, PRODUCT_ID_SIZE },
	{ CYAPA_SMBUS_BLK_HEAD, 16 },
	{ CYAPA_SMBUS_MAX_BASELINE, 1 },
	{ CYAPA_SMBUS_MIN_BASELINE, 1 },
};

#define CYAPA_DEBUGFS_READ_FW	"read_fw"
#define CYAPA_FW_NAME		"cyapa.bin"
#define CYAPA_FW_BLOCK_SIZE	64
#define CYAPA_FW_READ_SIZE	16
#define CYAPA_FW_HDR_START	0x0780
#define CYAPA_FW_HDR_BLOCK_COUNT  2
#define CYAPA_FW_HDR_BLOCK_START  (CYAPA_FW_HDR_START / CYAPA_FW_BLOCK_SIZE)
#define CYAPA_FW_HDR_SIZE	(CYAPA_FW_HDR_BLOCK_COUNT * \
				 CYAPA_FW_BLOCK_SIZE)
#define CYAPA_FW_DATA_START	0x0800
#define CYAPA_FW_DATA_BLOCK_COUNT  480
#define CYAPA_FW_DATA_BLOCK_START  (CYAPA_FW_DATA_START / CYAPA_FW_BLOCK_SIZE)
#define CYAPA_FW_DATA_SIZE	(CYAPA_FW_DATA_BLOCK_COUNT * \
				 CYAPA_FW_BLOCK_SIZE)
#define CYAPA_FW_SIZE		(CYAPA_FW_HDR_SIZE + CYAPA_FW_DATA_SIZE)
#define CYAPA_CMD_LEN		16

static void cyapa_detect(struct cyapa *cyapa);
static int cyapa_resume(struct device *dev);

#define BYTE_PER_LINE  8
void cyapa_dump_data(struct cyapa *cyapa, size_t length, const u8 *data)
{
#ifdef DEBUG
	struct device *dev = &cyapa->client->dev;
	int i;
	char buf[BYTE_PER_LINE * 3 + 1];
	char *s = buf;

	for (i = 0; i < length; i++) {
		s += sprintf(s, " %02x", data[i]);
		if ((i + 1) == length || ((i + 1) % BYTE_PER_LINE) == 0) {
			dev_dbg(dev, "%s\n", buf);
			s = buf;
		}
	}
#endif
}
#undef BYTE_PER_LINE

#define cyapa_dbg(cyapa, fmt, ...)					\
do {									\
	if (cyapa->debug)						\
		dev_info(&cyapa->client->dev, fmt, ##__VA_ARGS__);	\
	else								\
		dev_dbg(&cyapa->client->dev, fmt, ##__VA_ARGS__);	\
} while (0)

static ssize_t cyapa_i2c_reg_read_block(struct cyapa *cyapa, u8 reg, size_t len,
					u8 *values)
{
	ssize_t ret;

	ret = i2c_smbus_read_i2c_block_data(cyapa->client, reg, len, values);
	cyapa_dbg(cyapa, "i2c read block reg: 0x%02x len: %zu ret: %zd\n",
		  reg, len, ret);
	if (ret > 0)
		cyapa_dump_data(cyapa, ret, values);

	return ret;
}

static ssize_t cyapa_i2c_reg_write_block(struct cyapa *cyapa, u8 reg,
					 size_t len, const u8 *values)
{
	ssize_t ret;

	ret = i2c_smbus_write_i2c_block_data(cyapa->client, reg, len, values);
	cyapa_dbg(cyapa, "i2c write block reg: 0x%02x len: %zu ret: %zd\n",
		  reg, len, ret);
	if (ret > 0)
		cyapa_dump_data(cyapa, ret, values);

	return ret;
}

/*
 * cyapa_smbus_read_block - perform smbus block read command
 * @cyapa  - private data structure of the driver
 * @cmd    - the properly encoded smbus command
 * @len    - expected length of smbus command result
 * @values - buffer to store smbus command result
 *
 * Returns negative errno, else the number of bytes written.
 *
 * Note:
 * In trackpad device, the memory block allocated for I2C register map
 * is 256 bytes, so the max read block for I2C bus is 256 bytes.
 */
static ssize_t cyapa_smbus_read_block(struct cyapa *cyapa, u8 cmd, size_t len,
				      u8 *values)
{
	ssize_t ret;
	u8 index;
	u8 smbus_cmd;
	u8 *buf;
	struct i2c_client *client = cyapa->client;

	if (!(SMBUS_BYTE_BLOCK_CMD_MASK & cmd))
		return -EINVAL;

	if (SMBUS_GROUP_BLOCK_CMD_MASK & cmd) {
		/* read specific block registers command. */
		smbus_cmd = SMBUS_ENCODE_RW(cmd, SMBUS_READ);
		ret = i2c_smbus_read_block_data(client, smbus_cmd, values);
		goto out;
	}

	ret = 0;
	for (index = 0; index * I2C_SMBUS_BLOCK_MAX < len; index++) {
		smbus_cmd = SMBUS_ENCODE_IDX(cmd, index);
		smbus_cmd = SMBUS_ENCODE_RW(smbus_cmd, SMBUS_READ);
		buf = values + I2C_SMBUS_BLOCK_MAX * index;
		ret = i2c_smbus_read_block_data(client, smbus_cmd, buf);
		if (ret < 0)
			goto out;
	}

out:
	cyapa_dbg(cyapa, "smbus read block cmd: 0x%02x len: %zu ret: %zd\n",
		  cmd, len, ret);
	if (ret > 0)
		cyapa_dump_data(cyapa, len, values);

	return ret > 0 ? len : ret;
}

static s32 cyapa_read_byte(struct cyapa *cyapa, u8 cmd_idx)
{
	ssize_t ret;
	u8 cmd;

	if (cyapa->smbus) {
		cmd = cyapa_smbus_cmds[cmd_idx].cmd;
		cmd = SMBUS_ENCODE_RW(cmd, SMBUS_READ);
	} else {
		cmd = cyapa_i2c_cmds[cmd_idx].cmd;
	}
	ret = i2c_smbus_read_byte_data(cyapa->client, cmd);
	cyapa_dbg(cyapa, "read byte [0x%02x] = 0x%02zx  ret: %zd\n",
		  cmd, ret, ret);

	return ret;
}

static s32 cyapa_write_byte(struct cyapa *cyapa, u8 cmd_idx, u8 value)
{
	ssize_t ret;
	u8 cmd;

	if (cyapa->smbus) {
		cmd = cyapa_smbus_cmds[cmd_idx].cmd;
		cmd = SMBUS_ENCODE_RW(cmd, SMBUS_WRITE);
	} else {
		cmd = cyapa_i2c_cmds[cmd_idx].cmd;
	}
	ret = i2c_smbus_write_byte_data(cyapa->client, cmd, value);
	cyapa_dbg(cyapa, "write byte [0x%02x] = 0x%02x  ret: %zd\n",
		  cmd, value, ret);

	return ret;
}

static ssize_t cyapa_read_block(struct cyapa *cyapa, u8 cmd_idx, u8 *values)
{
	u8 cmd;
	size_t len;

	if (cyapa->smbus) {
		cmd = cyapa_smbus_cmds[cmd_idx].cmd;
		len = cyapa_smbus_cmds[cmd_idx].len;
		return cyapa_smbus_read_block(cyapa, cmd, len, values);
	} else {
		cmd = cyapa_i2c_cmds[cmd_idx].cmd;
		len = cyapa_i2c_cmds[cmd_idx].len;
		return cyapa_i2c_reg_read_block(cyapa, cmd, len, values);
	}
}

/*
 * Query device for its current operating state.
 *
 */
static int cyapa_get_state(struct cyapa *cyapa)
{
	int ret;
	u8 status[BL_STATUS_SIZE];

	cyapa->state = CYAPA_STATE_NO_DEVICE;

	/*
	 * Get trackpad status by reading 3 registers starting from 0.
	 * If the device is in the bootloader, this will be BL_HEAD.
	 * If the device is in operation mode, this will be the DATA regs.
	 *
	 */
	ret = cyapa_i2c_reg_read_block(cyapa, BL_HEAD_OFFSET, BL_STATUS_SIZE,
				       status);

	/*
	 * On smbus systems in OP mode, the i2c_reg_read will fail with
	 * -ETIMEDOUT.  In this case, try again using the smbus equivalent
	 * command.  This should return a BL_HEAD indicating CYAPA_STATE_OP.
	 */
	if (cyapa->smbus && (ret == -ETIMEDOUT || ret == -ENXIO)) {
		cyapa_dbg(cyapa, "smbus: probing with BL_STATUS command\n");
		ret = cyapa_read_block(cyapa, CYAPA_CMD_BL_STATUS, status);
	}
	if (ret != BL_STATUS_SIZE)
		goto error;

	cyapa->status[REG_OP_STATUS] = status[REG_OP_STATUS];
	cyapa->status[REG_BL_STATUS] = status[REG_BL_STATUS];
	cyapa->status[REG_BL_ERROR] = status[REG_BL_ERROR];
	cyapa_dbg(cyapa, "status registers = [0x%02x, 0x%02x, 0x%02x]\n",
			status[REG_OP_STATUS], status[REG_BL_STATUS],
			status[REG_BL_ERROR]);
	if ((status[REG_OP_STATUS] & OP_STATUS_SRC) == OP_STATUS_SRC) {
		switch (status[REG_OP_STATUS] & OP_STATUS_DEV) {
		case CYAPA_DEV_NORMAL:
			cyapa_dbg(cyapa, "device state: operational mode\n");
			cyapa->state = CYAPA_STATE_OP;
			break;
		case CYAPA_DEV_BUSY:
			cyapa_dbg(cyapa, "device state: operational busy\n");
			cyapa->state = CYAPA_STATE_OP;
			break;
		default:
			cyapa->debug = true;
			cyapa_dbg(cyapa, "device state: unknown\n");
			cyapa->state = CYAPA_STATE_NO_DEVICE;
			ret = -EAGAIN;
			goto error;
		}
	} else {
		if (status[REG_BL_STATUS] & BL_STATUS_BUSY) {
			cyapa_dbg(cyapa, "device state: bootloader busy\n");
			cyapa->state = CYAPA_STATE_BL_BUSY;
		} else if (status[REG_BL_ERROR] & BL_ERROR_BOOTLOADING) {
			cyapa_dbg(cyapa, "device state: bootloader active\n");
			cyapa->state = CYAPA_STATE_BL_ACTIVE;
		} else {
			cyapa_dbg(cyapa, "device state: bootloader idle\n");
			cyapa->state = CYAPA_STATE_BL_IDLE;
		}
	}

	return 0;
error:
	return (ret < 0) ? ret : -EAGAIN;
}

/*
 * Poll device for its status in a loop, waiting up to timeout for a response.
 *
 * When the device switches state, it usually takes ~300 ms.
 * However, when running a new firmware image, the device must calibrate its
 * sensors, which can take as long as 2 seconds.
 *
 * Note: The timeout has granularity of the polling rate, which is 100 ms.
 *
 * Returns:
 *   0 when the device eventually responds with a valid non-busy state.
 *   -ETIMEDOUT if device never responds (too many -EAGAIN)
 *   < 0    other errors
 */
static int cyapa_poll_state(struct cyapa *cyapa, unsigned int timeout)
{
	int ret;
	int tries = timeout / 100;
	bool debug_prev = cyapa->debug;
	cyapa_dbg(cyapa, "======< cyapa_poll_state >======");

	ret = cyapa_get_state(cyapa);
	if (ret) {
		dev_err(&cyapa->client->dev, "poll state first trial"
				" failed\n");
		cyapa->debug = true;
	}

	while ((ret || cyapa->state >= CYAPA_STATE_BL_BUSY) && tries--) {
		msleep(100);
		ret = cyapa_get_state(cyapa);
		if (ret)
			cyapa->debug = true;
	}
	if (ret == 0 && debug_prev != cyapa->debug)
		cyapa->debug = debug_prev;

	return (ret == -EAGAIN || ret == -ETIMEDOUT) ? -ETIMEDOUT : ret;
}

static const char *cyapa_state_to_string(struct cyapa *cyapa)
{
	switch (cyapa->state) {
	case CYAPA_STATE_BL_ACTIVE: return "BL_ACTIVE";
	case CYAPA_STATE_BL_IDLE: return "BL_IDLE";
	case CYAPA_STATE_BL_BUSY: return "BL_BUSY";
	case CYAPA_STATE_OP: return "OPERATIONAL";
	default: return "UNKNOWN";
	}
}

/*
 * Enter bootloader by soft resetting the device.
 *
 * If device is already in the bootloader, the function just returns.
 * Otherwise, reset the device; after reset, device enters bootloader idle
 * state immediately.
 *
 * Also, if device was unregister device from input core.  Device will
 * re-register after it is detected following resumption of operational mode.
 *
 * Returns:
 *   0 on success
 *   -EAGAIN  device was reset, but is not now in bootloader idle state
 *   < 0 if the device never responds within the timeout
 */
static int cyapa_bl_enter(struct cyapa *cyapa)
{
	int ret;
	cyapa_dbg(cyapa, "======< cyapa_bl_enter >======");

	if (cyapa->input) {
		disable_irq(cyapa->irq);
		input_unregister_device(cyapa->input);
		cyapa->input = NULL;
	}

	ret = cyapa_poll_state(cyapa, 500);
	if (ret < 0)
		return ret;
	if (cyapa->state == CYAPA_STATE_BL_IDLE) {
		cyapa_dbg(cyapa, "Already in BL_IDLE. Skipping exit.\n");
		return 0;
	}

	if (cyapa->state != CYAPA_STATE_OP) {
		cyapa->debug = true;
		cyapa_dbg(cyapa, "Not in OP state. state = %s\n",
			  cyapa_state_to_string(cyapa));
		return -EAGAIN;
	}

	cyapa->state = CYAPA_STATE_NO_DEVICE;
	ret = cyapa_write_byte(cyapa, CYAPA_CMD_SOFT_RESET, 0x01);
	if (ret < 0) {
		cyapa->debug = true;
		cyapa_dbg(cyapa, "bl_enter cyapa_write_byte failed, %d\n",
			  ret);
		return -EIO;
	}

	usleep_range(25000, 50000);
	ret = cyapa_poll_state(cyapa, 500);
	if (ret < 0)
		return ret;
	if ((cyapa->state != CYAPA_STATE_BL_IDLE) ||
		(cyapa->status[REG_BL_STATUS] & BL_STATUS_WATCHDOG)) {
		cyapa->debug = true;
		cyapa_dbg(cyapa, "bl_enter failed. Now in state %s\n",
			  cyapa_state_to_string(cyapa));
		return -EAGAIN;
	}

	return 0;
}

static int cyapa_bl_activate(struct cyapa *cyapa)
{
	int ret;
	cyapa_dbg(cyapa, "======< cyapa_bl_activate >======");

	ret = cyapa_i2c_reg_write_block(cyapa, 0, sizeof(bl_activate),
					bl_activate);
	if (ret < 0) {
		cyapa->debug = true;
		cyapa_dbg(cyapa, "bl_activate i2c_reg_write failed, %d\n",
			  ret);
		return ret;
	}

	/* Wait for bootloader to activate; takes between 2 and 12 seconds */
	msleep(2000);
	ret = cyapa_poll_state(cyapa, 11000);
	if (ret < 0)
		return ret;
	if (cyapa->state != CYAPA_STATE_BL_ACTIVE) {
		cyapa->debug = true;
		cyapa_dbg(cyapa, "bl_activate failed. Now in state %s.\n",
			  cyapa_state_to_string(cyapa));
		return -EAGAIN;
	}

	return 0;
}

static int cyapa_bl_deactivate(struct cyapa *cyapa)
{
	int ret;
	cyapa_dbg(cyapa, "======< cyapa_bl_deactivate >======");

	ret = cyapa_i2c_reg_write_block(cyapa, 0, sizeof(bl_deactivate),
					bl_deactivate);
	if (ret < 0) {
		cyapa->debug = true;
		cyapa_dbg(cyapa, "bl_deactivate i2c_reg_write failed, %d\n",
			  ret);
		return ret;
	}

	/* wait for bootloader to switch to idle state; should take < 100ms */
	msleep(100);
	ret = cyapa_poll_state(cyapa, 500);
	if (ret < 0)
		return ret;
	if (cyapa->state != CYAPA_STATE_BL_IDLE) {
		cyapa->debug = true;
		cyapa_dbg(cyapa, "bl_deactivate failed. Now in state %s.\n",
			  cyapa_state_to_string(cyapa));
		return -EAGAIN;
	}
	return 0;
}

/*
 * Exit bootloader
 *
 * Send bl_exit command, then wait 50 - 100 ms to let device transition to
 * operational mode.  If this is the first time the device's firmware is
 * running, it can take up to 2 seconds to calibrate its sensors.  So, poll
 * the device's new state for up to 2 seconds.
 *
 * Returns:
 *   -EIO    failure while reading from device
 *   -EAGAIN device is stuck in bootloader, b/c it has invalid firmware
 *   0       device is supported and in operational mode
 */
static int cyapa_bl_exit(struct cyapa *cyapa)
{
	int ret;
	cyapa_dbg(cyapa, "======< cyapa_bl_exit >======");

	ret = cyapa_i2c_reg_write_block(cyapa, 0, sizeof(bl_exit), bl_exit);
	if (ret < 0) {
		cyapa->debug = true;
		cyapa_dbg(cyapa, "bl_exit i2c_reg_write failed, %d\n", ret);
		return ret;
	}

	/*
	 * Wait for bootloader to exit, and operation mode to start.
	 * Normally, this takes at least 50 ms.
	 */
	usleep_range(50000, 100000);
	/*
	 * In addition, when a device boots for the first time after being
	 * updated to new firmware, it must first calibrate its sensors, which
	 * can take up to an additional 2 seconds. If the device power is
	 * running low, this may take even longer.
	 */
	ret = cyapa_poll_state(cyapa, 4000);
	if (ret < 0)
		return ret;
	if (cyapa->state != CYAPA_STATE_OP) {
		cyapa->debug = true;
		cyapa_dbg(cyapa, "bl_exit failed. Now in state %s.\n",
			  cyapa_state_to_string(cyapa));
		cyapa_dbg(cyapa, "status registers ="
				" [0x%02x, 0x%02x, 0x%02x]\n",
				cyapa->status[REG_OP_STATUS],
				cyapa->status[REG_BL_STATUS],
				cyapa->status[REG_BL_ERROR]);
		return -EAGAIN;
	}

	return 0;
}

/*
 * cyapa_sleep_time_to_pwr_cmd and cyapa_pwr_cmd_to_sleep_time
 *
 * These are helper functions that convert to and from integer idle
 * times and register settings to write to the PowerMode register.
 * The trackpad supports between 20ms to 1000ms scan intervals.
 * The time will be increased in increments of 10ms from 20ms to 100ms.
 * From 100ms to 1000ms, time will be increased in increments of 20ms.
 *
 * When Idle_Time < 100, the format to convert Idle_Time to Idle_Command is:
 *   Idle_Command = Idle Time / 10;
 * When Idle_Time >= 100, the format to convert Idle_Time to Idle_Command is:
 *   Idle_Command = Idle Time / 20 + 5;
 */
static u8 cyapa_sleep_time_to_pwr_cmd(u16 sleep_time)
{
	if (sleep_time < 20)
		sleep_time = 20;     /* minimal sleep time. */
	else if (sleep_time > 1000)
		sleep_time = 1000;   /* maximal sleep time. */

	if (sleep_time < 100)
		return ((sleep_time / 10) << 2) & PWR_MODE_MASK;
	else
		return ((sleep_time / 20 + 5) << 2) & PWR_MODE_MASK;
}

static u16 cyapa_pwr_cmd_to_sleep_time(u8 pwr_mode)
{
	u8 encoded_time = pwr_mode >> 2;
	return (encoded_time < 10) ? encoded_time * 10
				   : (encoded_time - 5) * 20;
}

/*
 * cyapa_get_wait_time_for_pwr_cmd
 *
 * Compute the amount of time we need to wait after updating the touchpad
 * power mode. The touchpad needs to consume the incoming power mode set
 * command at the current clock rate.
 */

static u16 cyapa_get_wait_time_for_pwr_cmd(u8 pwr_mode)
{
	switch (pwr_mode) {
	case PWR_MODE_FULL_ACTIVE: return 20;
	case PWR_MODE_BTN_ONLY: return 20;
	case PWR_MODE_OFF: return 20;
	default: return cyapa_pwr_cmd_to_sleep_time(pwr_mode) + 50;
	}
}


/*
 * Set device power mode
 *
 * Write to the field to configure power state. Power states include :
 *   Full : Max scans and report rate.
 *   Idle : Report rate set by user specified time.
 *   ButtonOnly : No scans for fingers. When the button is triggered,
 *     a slave interrupt is asserted to notify host to wake up.
 *   Off : Only awake for i2c commands from host. No function for button
 *     or touch sensors.
 *
 * The power_mode command should conform to the following :
 *   Full : 0x3f
 *   Idle : Configurable from 20 to 1000ms. See note below for
 *     cyapa_sleep_time_to_pwr_cmd and cyapa_pwr_cmd_to_sleep_time
 *   ButtonOnly : 0x01
 *   Off : 0x00
 *
 * Device power mode can only be set when device is in operational mode.
 */
static int cyapa_set_power_mode(struct cyapa *cyapa, u8 power_mode)
{
	struct device *dev = &cyapa->client->dev;
	int ret;
	u8 power;
	int tries = SET_POWER_MODE_TRIES;
	u16 sleep_time;
	cyapa_dbg(cyapa, "======< cyapa_set_power_mode >======");

	if (cyapa->state != CYAPA_STATE_OP)
		return 0;

	while (true) {
		ret = cyapa_read_byte(cyapa, CYAPA_CMD_POWER_MODE);
		if (ret >= 0 || --tries < 1)
			break;
		dev_dbg(dev, "set_power_mode read retry. tries left = %d\n",
			tries);
		usleep_range(SET_POWER_MODE_DELAY, 2 * SET_POWER_MODE_DELAY);
	}
	if (ret < 0) {
		dev_err(dev, "set_power_mode failed to read power mode %d\n",
				ret);
		cyapa->debug = true;
		return ret;
	}
	/*
	 * Return early if the power mode to set is the same as the current
	 * one.
	 */
	if ((ret & PWR_MODE_MASK) == power_mode) {
		cyapa_dbg(cyapa, "set_power_mode early return\n");
		return 0;
	}

	sleep_time = cyapa_get_wait_time_for_pwr_cmd(ret & PWR_MODE_MASK);
	power = ret;
	power &= ~PWR_MODE_MASK;
	power |= power_mode & PWR_MODE_MASK;
	while (true) {
		ret = cyapa_write_byte(cyapa, CYAPA_CMD_POWER_MODE, power);
		if (!ret || --tries < 1)
			break;
		dev_dbg(dev, "set_power_mode write retry. tries left = %d\n",
			tries);
		usleep_range(SET_POWER_MODE_DELAY, 2 * SET_POWER_MODE_DELAY);
	}
	if (ret < 0) {
		dev_err(dev, "set_power_mode failed to set power mode"
				" 0x%02x err = %d\n", power_mode, ret);
		cyapa->debug = true;
	}
	/*
	 * Wait for the newly set power command to go in at the previous
	 * clock speed (scanrate) used by the touchpad firmware. Not
	 * doing so before issuing the next command may result in errors
	 * depending on the command's content.
	 */
	msleep(sleep_time);
	return ret;
}

static int cyapa_get_query_data(struct cyapa *cyapa)
{
	u8 query_data[QUERY_DATA_SIZE];
	int ret;
	cyapa_dbg(cyapa, "======< cyapa_get_query_data >======");

	if (cyapa->state != CYAPA_STATE_OP)
		return -EBUSY;

	ret = cyapa_read_block(cyapa, CYAPA_CMD_GROUP_QUERY, query_data);
	if (ret != QUERY_DATA_SIZE) {
		cyapa->debug = true;
		cyapa_dbg(cyapa, "failed to get query data, ret = %d\n", ret);
		return (ret < 0) ? ret : -EIO;
	}

	memcpy(&cyapa->product_id[0], &query_data[0], 5);
	cyapa->product_id[5] = '-';
	memcpy(&cyapa->product_id[6], &query_data[5], 6);
	cyapa->product_id[12] = '-';
	memcpy(&cyapa->product_id[13], &query_data[11], 2);
	cyapa->product_id[15] = '\0';

	cyapa->fw_maj_ver = query_data[15];
	cyapa->fw_min_ver = query_data[16];

	cyapa->btn_capability = query_data[19] & CAPABILITY_BTN_MASK;

	cyapa->gen = query_data[20] & 0x0f;

	cyapa->max_abs_x = ((query_data[21] & 0xf0) << 4) | query_data[22];
	cyapa->max_abs_y = ((query_data[21] & 0x0f) << 8) | query_data[23];

	cyapa->physical_size_x =
		((query_data[24] & 0xf0) << 4) | query_data[25];
	cyapa->physical_size_y =
		((query_data[24] & 0x0f) << 8) | query_data[26];

	return 0;
}

/*
 * Check if device is operational.
 *
 * An operational device is responding, has exited bootloader, and has
 * firmware supported by this driver.
 *
 * Returns:
 *   -EBUSY  no device or in bootloader
 *   -EIO    failure while reading from device
 *   -EAGAIN device is still in bootloader
 *           if ->state = CYAPA_STATE_BL_IDLE, device has invalid firmware
 *   -EINVAL device is in operational mode, but not supported by this driver
 *   0       device is supported
 */
static int cyapa_check_is_operational(struct cyapa *cyapa)
{
	struct device *dev = &cyapa->client->dev;
	static const char unique_str[] = "CYTRA";
	int ret;
	cyapa_dbg(cyapa, "======< cyapa_check_is_operational >======");

	ret = cyapa_poll_state(cyapa, 4000);
	if (ret < 0)
		return ret;
	switch (cyapa->state) {
	case CYAPA_STATE_BL_ACTIVE:
		ret = cyapa_bl_deactivate(cyapa);
		if (ret) {
			dev_err(dev, "failed to bl_deactivate. %d\n", ret);
			return ret;
		}

	/* Fallthrough state */
	case CYAPA_STATE_BL_IDLE:
		ret = cyapa_bl_exit(cyapa);
		if (ret) {
			dev_err(dev, "failed to bl_exit. %d\n", ret);
			return ret;
		}

	/* Fallthrough state */
	case CYAPA_STATE_OP:
		/*
		 * Reading query data before going back to the full mode
		 * may cause problems, so we set the power mode first here.
		 */
		ret = cyapa_set_power_mode(cyapa, PWR_MODE_FULL_ACTIVE);
		if (ret) {
			dev_err(dev, "check_is_operational active power"
					" failed, %d\n", ret);
			cyapa->debug = true;
		}
		ret = cyapa_get_query_data(cyapa);
		if (ret < 0)
			return ret;

		/* only support firmware protocol gen3 */
		if (cyapa->gen != CYAPA_GEN3) {
			dev_err(dev, "unsupported protocol version (%d)",
				cyapa->gen);
			return -EINVAL;
		}

		/* only support product ID starting with CYTRA */
		if (memcmp(cyapa->product_id, unique_str,
			   sizeof(unique_str) - 1) != 0) {
			dev_err(dev, "unsupported product ID (%s)\n",
				cyapa->product_id);
			return -EINVAL;
		}
		return 0;

	default:
		return -EIO;
	}
	return 0;
}

static u16 cyapa_csum(const u8 *buf, size_t count)
{
	int i;
	u16 csum = 0;

	for (i = 0; i < count; i++)
		csum += buf[i];

	return csum;
}

/*
 * Write a |len| byte long buffer |buf| to the device, by chopping it up into a
 * sequence of smaller |CYAPA_CMD_LEN|-length write commands.
 *
 * The data bytes for a write command are prepended with the 1-byte offset
 * of the data relative to the start of |buf|.
 */
static int cyapa_write_buffer(struct cyapa *cyapa, const u8 *buf, size_t len)
{
	int ret;
	size_t i;
	unsigned char cmd[CYAPA_CMD_LEN + 1];
	size_t cmd_len;

	for (i = 0; i < len; i += CYAPA_CMD_LEN) {
		const u8 *payload = &buf[i];
		cmd_len = (len - i >= CYAPA_CMD_LEN) ? CYAPA_CMD_LEN : len - i;
		cmd[0] = i;
		memcpy(&cmd[1], payload, cmd_len);

		ret = cyapa_i2c_reg_write_block(cyapa, 0, cmd_len + 1, cmd);
		if (ret < 0) {
			cyapa->debug = true;
			cyapa_dbg(cyapa, "write_buffer i2c_reg_write"
					" failed, %d\n", ret);
			return ret;
		}
	}
	return 0;
}

/*
 * A firmware block write command writes 64 bytes of data to a single flash
 * page in the device.  The 78-byte block write command has the format:
 *   <0xff> <CMD> <Key> <Start> <Data> <Data-Checksum> <CMD Checksum>
 *
 *  <0xff>  - every command starts with 0xff
 *  <CMD>   - the write command value is 0x39
 *  <Key>   - write commands include an 8-byte key: { 00 01 02 03 04 05 06 07 }
 *  <Block> - Memory Block number (address / 64) (16-bit, big-endian)
 *  <Data>  - 64 bytes of firmware image data
 *  <Data Checksum> - sum of 64 <Data> bytes, modulo 0xff
 *  <CMD Checksum> - sum of 77 bytes, from 0xff to <Data Checksum>
 *
 * Each write command is split into 5 i2c write transactions of up to 16 bytes.
 * Each transaction starts with an i2c register offset: (00, 10, 20, 30, 40).
 */
static int cyapa_write_fw_block(struct cyapa *cyapa, u16 block, const u8 *data)
{
	int ret;
	u8 cmd[78];
	u8 status[BL_STATUS_SIZE];
	/* Programming for one block can take about 100ms. */
	int tries = 11;
	u8 bl_status, bl_error;
	struct device *dev = &cyapa->client->dev;

	/* set write command and security key bytes. */
	cmd[0] = 0xff;
	cmd[1] = 0x39;
	cmd[2] = 0x00;
	cmd[3] = 0x01;
	cmd[4] = 0x02;
	cmd[5] = 0x03;
	cmd[6] = 0x04;
	cmd[7] = 0x05;
	cmd[8] = 0x06;
	cmd[9] = 0x07;
	cmd[10] = block >> 8;
	cmd[11] = block;
	memcpy(&cmd[12], data, CYAPA_FW_BLOCK_SIZE);
	cmd[76] = cyapa_csum(data, CYAPA_FW_BLOCK_SIZE);
	cmd[77] = cyapa_csum(cmd, sizeof(cmd) - 1);

	ret = cyapa_write_buffer(cyapa, cmd, sizeof(cmd));
	if (ret) {
		cyapa_dbg(cyapa, "write_fw_block write_buffer"
				" failed, %d\n", ret);
		return ret;
	}

	/* wait for write to finish */
	do {
		usleep_range(10000, 20000);

		/* check block write command result status. */
		ret = cyapa_i2c_reg_read_block(cyapa, BL_HEAD_OFFSET,
					       BL_STATUS_SIZE, status);
		if (ret != BL_STATUS_SIZE) {
			cyapa->debug = true;
			cyapa_dbg(cyapa, "write_fw_block i2c_reg_read"
					" failed, %d\n", ret);
			return (ret < 0) ? ret : -EIO;
		}
	} while ((status[1] & BL_STATUS_BUSY) && --tries);

	/* ignore WATCHDOG bit and reserved bits. */
	bl_status = status[1] & ~BL_STATUS_REV_MASK;
	bl_error = status[2] & ~BL_ERROR_RESERVED;

	if (status[1] & BL_STATUS_BUSY) {
		dev_warn(dev, "write_fw_block timeout.\n");
		ret = -ETIMEDOUT;
	} else if (bl_status != BL_STATUS_RUNNING ||
		bl_error != BL_ERROR_BOOTLOADING) {
		ret = -EIO;
	} else {
		ret = 0;
	}

	if (ret) {
		cyapa->debug = true;
		cyapa_dbg(cyapa, "status registers ="
				" [0x%02x, 0x%02x, 0x%02x]\n",
				status[REG_OP_STATUS], status[REG_BL_STATUS],
				status[REG_BL_ERROR]);
	}
	return ret;
}

/*
 * A firmware block read command reads 16 bytes of data from flash starting
 * from a given address.  The 12-byte block read command has the format:
 *   <0xff> <CMD> <Key> <Addr>
 *
 *  <0xff>  - every command starts with 0xff
 *  <CMD>   - the read command value is 0x3c
 *  <Key>   - read commands include an 8-byte key: { 00 01 02 03 04 05 06 07 }
 *  <Addr>  - Memory address (16-bit, big-endian)
 *
 * The command is followed by an i2c block read to read the 16 bytes of data.
 */
static int cyapa_read_fw_bytes(struct cyapa *cyapa, u16 addr, u8 *data)
{
	int ret;
	u8 cmd[] = { 0xff, 0x3c, 0, 1, 2, 3, 4, 5, 6, 7, addr >> 8, addr };

	ret = cyapa_write_buffer(cyapa, cmd, sizeof(cmd));
	if (ret) {
		cyapa_dbg(cyapa, "read_fw_bytes write_buffer"
				" failed, %d\n", ret);
		return ret;
	}

	/* read data buffer starting from offset 16 */
	ret = cyapa_i2c_reg_read_block(cyapa, 16, CYAPA_FW_READ_SIZE, data);
	if (ret != CYAPA_FW_READ_SIZE) {
		cyapa->debug = true;
		cyapa_dbg(cyapa, "read_fw_bytes i2c_reg_read failed, %d\n",
				ret);
		return (ret < 0) ? ret : -EIO;
	}

	return 0;
}


/*
 * Verify the integrity of a CYAPA firmware image file.
 *
 * The firmware image file is 30848 bytes, composed of 482 64-byte blocks.
 *
 * The first 2 blocks are the firmware header.
 * The next 480 blocks are the firmware image.
 *
 * The first two bytes of the header hold the header checksum, computed by
 * summing the other 126 bytes of the header.
 * The last two bytes of the header hold the firmware image checksum, computed
 * by summing the 30720 bytes of the image modulo 0xffff.
 *
 * Both checksums are stored little-endian.
 */
static int cyapa_check_fw(struct cyapa *cyapa, const struct firmware *fw)
{
	struct device *dev = &cyapa->client->dev;
	u16 csum;
	u16 csum_expected;

	/* Firmware must match exact 30848 bytes = 482 64-byte blocks. */
	if (fw->size != CYAPA_FW_SIZE) {
		dev_err(dev, "invalid firmware size = %zu, expected %u.\n",
			fw->size, CYAPA_FW_SIZE);
		return -EINVAL;
	}

	/* Verify header block */
	csum_expected = (fw->data[0] << 8) | fw->data[1];
	csum = cyapa_csum(&fw->data[2], CYAPA_FW_HDR_SIZE - 2);
	if (csum != csum_expected) {
		dev_err(dev, "invalid firmware header checksum = %04x,"
			       " expected: %04x\n", csum, csum_expected);
		return -EINVAL;
	}

	/* Verify firmware image */
	csum_expected = (fw->data[CYAPA_FW_HDR_SIZE - 2] << 8) |
			 fw->data[CYAPA_FW_HDR_SIZE - 1];
	csum = cyapa_csum(&fw->data[CYAPA_FW_HDR_SIZE], CYAPA_FW_DATA_SIZE);
	if (csum != csum_expected) {
		dev_err(dev, "invalid firmware header checksum = %04x,"
			       " expected: %04x\n", csum, csum_expected);
		return -EINVAL;
	}
	return 0;
}

static irqreturn_t cyapa_irq(int irq, void *dev_id)
{
	struct cyapa *cyapa = dev_id;
	struct device *dev = &cyapa->client->dev;
	struct input_dev *input = cyapa->input;
	struct cyapa_reg_data data;
	int i;
	int ret;
	int num_fingers;
	cyapa_dbg(cyapa, "======< cyapa_irq >======");

	pm_runtime_get_sync(dev);
	pm_runtime_mark_last_busy(dev);

	/*
	 * Don't read input if input device has not been configured.
	 * This check check solves a race during probe() between irq_request()
	 * and irq_disable(), since there is no way to request an irq that is
	 * initially disabled.
	 */
	if (!input)
		goto out;

	if (device_may_wakeup(dev))
		pm_wakeup_event(dev, 0);

	ret = cyapa_read_block(cyapa, CYAPA_CMD_GROUP_DATA, (u8 *)&data);
	if (ret != sizeof(data)) {
		cyapa->debug = true;
		cyapa_dbg(cyapa, "read_block failed in cyapa_irq. ret = %d\n",
			  ret);
		cyapa_detect(cyapa);
		goto out;
	}

	if ((data.device_status & OP_STATUS_SRC) != OP_STATUS_SRC ||
	    (data.device_status & OP_STATUS_DEV) != CYAPA_DEV_NORMAL ||
	    (data.finger_btn & OP_DATA_VALID) != OP_DATA_VALID) {
		cyapa->debug = true;
		cyapa_dbg(cyapa,
			  "irq error. device_status = 0x%x finger_btn = 0x%x",
			  data.device_status, data.finger_btn);
		cyapa_detect(cyapa);
		goto out;
	}

	num_fingers = (data.finger_btn >> 4) & 0x0f;
	for (i = 0; i < num_fingers; i++) {
		const struct cyapa_touch *touch = &data.touches[i];
		/* Note: touch->id range is 1 to 15; slots are 0 to 14. */
		int slot = touch->id - 1;

		input_mt_slot(input, slot);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
		input_report_abs(input, ABS_MT_POSITION_X,
				 ((touch->xy_hi & 0xf0) << 4) | touch->x_lo);
		input_report_abs(input, ABS_MT_POSITION_Y,
				 ((touch->xy_hi & 0x0f) << 8) | touch->y_lo);
		input_report_abs(input, ABS_MT_PRESSURE, touch->pressure);
	}

	input_mt_sync_frame(input);

	if (cyapa->btn_capability & CAPABILITY_LEFT_BTN_MASK)
		input_report_key(input, BTN_LEFT,
				 data.finger_btn & OP_DATA_LEFT_BTN);

	if (cyapa->btn_capability & CAPABILITY_MIDDLE_BTN_MASK)
		input_report_key(input, BTN_MIDDLE,
				 data.finger_btn & OP_DATA_MIDDLE_BTN);

	if (cyapa->btn_capability & CAPABILITY_RIGHT_BTN_MASK)
		input_report_key(input, BTN_RIGHT,
				 data.finger_btn & OP_DATA_RIGHT_BTN);

	input_sync(input);

out:
	pm_runtime_put_sync_autosuspend(dev);
	return IRQ_HANDLED;
}

static u8 cyapa_check_adapter_functionality(struct i2c_client *client)
{
	u8 ret = CYAPA_ADAPTER_FUNC_NONE;

	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		ret |= CYAPA_ADAPTER_FUNC_I2C;
	if (i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_BLOCK_DATA |
				     I2C_FUNC_SMBUS_I2C_BLOCK))
		ret |= CYAPA_ADAPTER_FUNC_SMBUS;
	return ret;
}

static int cyapa_inhibit(struct input_dev *input)
{
	struct device *dev = &input->dev;
	struct cyapa *cyapa = dev_get_drvdata(dev);

	pm_runtime_disable(dev);

	disable_irq(cyapa->irq);
	cyapa_set_power_mode(cyapa, PWR_MODE_OFF);

	pm_runtime_set_suspended(dev);

	return 0;
}

static int cyapa_uninhibit(struct input_dev *input)
{
	struct device *dev = &input->dev;
	struct cyapa *cyapa = dev_get_drvdata(dev);

	pm_runtime_set_active(dev);

	/* Activate device if it was powered off during resume. */
	cyapa_detect(cyapa);

	cyapa_set_power_mode(cyapa, PWR_MODE_FULL_ACTIVE);
	enable_irq(cyapa->irq);

	pm_runtime_enable(dev);

	return 0;
}

static int cyapa_create_input_dev(struct cyapa *cyapa)
{
	struct device *dev = &cyapa->client->dev;
	int ret;
	struct input_dev *input;
	cyapa_dbg(cyapa, "======< cyapa_create_input_dev >======");

	if (!cyapa->physical_size_x || !cyapa->physical_size_y)
		return -EINVAL;

	input = cyapa->input = input_allocate_device();
	if (!input) {
		dev_err(dev, "allocate memory for input device failed\n");
		return -ENOMEM;
	}

	input->name = CYAPA_NAME;
	input->phys = cyapa->phys;
	input->id.bustype = BUS_I2C;
	input->id.version = 1;
	input->id.product = 0;  /* means any product in eventcomm. */
	input->dev.parent = &cyapa->client->dev;
	input->inhibit = cyapa_inhibit;
	input->uninhibit = cyapa_uninhibit;

	input_set_drvdata(input, cyapa);

	__set_bit(EV_ABS, input->evbit);

	/* finger position */
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, cyapa->max_abs_x, 0,
			     0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, cyapa->max_abs_y, 0,
			     0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	input_abs_set_res(input, ABS_MT_POSITION_X,
			  cyapa->max_abs_x / cyapa->physical_size_x);
	input_abs_set_res(input, ABS_MT_POSITION_Y,
			  cyapa->max_abs_y / cyapa->physical_size_y);

	if (cyapa->btn_capability & CAPABILITY_LEFT_BTN_MASK)
		__set_bit(BTN_LEFT, input->keybit);
	if (cyapa->btn_capability & CAPABILITY_MIDDLE_BTN_MASK)
		__set_bit(BTN_MIDDLE, input->keybit);
	if (cyapa->btn_capability & CAPABILITY_RIGHT_BTN_MASK)
		__set_bit(BTN_RIGHT, input->keybit);

	if (cyapa->btn_capability == CAPABILITY_LEFT_BTN_MASK)
		__set_bit(INPUT_PROP_BUTTONPAD, input->propbit);

	/* handle pointer emulation and unused slots in core */
	ret = input_mt_init_slots(input, CYAPA_MAX_MT_SLOTS,
				  INPUT_MT_POINTER | INPUT_MT_DROP_UNUSED);
	if (ret) {
		dev_err(dev, "allocate memory for MT slots failed, %d\n", ret);
		goto err_free_device;
	}

	/* Register the device in input subsystem */
	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "input device register failed, %d\n", ret);
		goto err_free_device;
	}
	return 0;

err_free_device:
	input_free_device(input);
	cyapa->input = NULL;
	return ret;
}

static void cyapa_detect(struct cyapa *cyapa)
{
	struct device *dev = &cyapa->client->dev;
	char *envp[] = {"ERROR=1", NULL};
	int ret;
	cyapa_dbg(cyapa, "======< cyapa_detect >======");

	ret = cyapa_check_is_operational(cyapa);
	if (ret == -ETIMEDOUT) {
		dev_err(dev, "no device detected, %d\n", ret);
	} else if (ret) {
		dev_err(dev, "device detected, but not operational, %d\n", ret);
	}

	if (ret) {
		kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
		return;
	}

	cyapa->debug = false;
	if (!cyapa->input && ret == 0) {
		ret = cyapa_create_input_dev(cyapa);
		if (ret)
			dev_err(dev, "create input_dev instance failed, %d\n",
				ret);

		enable_irq(cyapa->irq);
		/*
		 * On some systems, a system crash / warm boot does not reset
		 * the device's current power mode to FULL_ACTIVE.
		 * If such an event happens during suspend, after the device
		 * has been put in a low power mode, the device will still be
		 * in low power mode on a subsequent boot, since there was
		 * never a matching resume().
		 * Handle this by always forcing full power here, when a
		 * device is first detected to be in operational mode.
		 */
		ret = cyapa_set_power_mode(cyapa, PWR_MODE_FULL_ACTIVE);
		if (ret)
			dev_warn(dev, "set active power failed, %d\n", ret);
	}
}

static int cyapa_firmware(struct cyapa *cyapa, const char *fw_name)
{
	struct device *dev = &cyapa->client->dev;
	int ret;
	const struct firmware *fw;
	int i;
	cyapa_dbg(cyapa, "======< cyapa_firmware >======");

	ret = request_firmware(&fw, fw_name, dev);
	if (ret) {
		dev_err(dev, "Could not load firmware from %s, %d\n",
			fw_name, ret);
		return ret;
	}

	ret = cyapa_check_fw(cyapa, fw);
	if (ret) {
		dev_err(dev, "Invalid CYAPA firmware image: %s\n", fw_name);
		goto done;
	}

	/*
	 * Resume the potentially suspended device because doing FW
	 * update on a device not in the FULL mode has a chance to
	 * fail.
	 */
	pm_runtime_get_sync(dev);

	ret = cyapa_bl_enter(cyapa);
	if (ret)
		goto err_detect;

	ret = cyapa_bl_activate(cyapa);
	if (ret)
		goto err_detect;

	cyapa_dbg(cyapa, "Start writing firmware\n");
	/* First write data, starting at byte 128  of fw->data */
	for (i = 0; i < CYAPA_FW_DATA_BLOCK_COUNT; i++) {
		size_t block = CYAPA_FW_DATA_BLOCK_START + i;
		size_t addr = (i + CYAPA_FW_HDR_BLOCK_COUNT) *
				CYAPA_FW_BLOCK_SIZE;
		const u8 *data = &fw->data[addr];
		ret = cyapa_write_fw_block(cyapa, block, data);
		if (ret) {
			dev_err(dev, "FW update aborted, %d\n", ret);
			goto err_detect;
		}
	}

	cyapa_dbg(cyapa, "Start writing firmware checksum\n");
	/* Then write checksum */
	for (i = 0; i < CYAPA_FW_HDR_BLOCK_COUNT; i++) {
		size_t block = CYAPA_FW_HDR_BLOCK_START + i;
		size_t addr = i * CYAPA_FW_BLOCK_SIZE;
		const u8 *data = &fw->data[addr];
		ret = cyapa_write_fw_block(cyapa, block, data);
		if (ret) {
			dev_err(dev, "FW update aborted, %d\n", ret);
			goto err_detect;
		}
	}

err_detect:
	pm_runtime_put_noidle(dev);
	cyapa_detect(cyapa);

done:
	release_firmware(fw);
	return ret;
}

/*
 * Read the entire firmware image into ->read_fw_image.
 * If the ->read_fw_image has already been allocated, then this function
 * doesn't do anything and just returns 0.
 * If an error occurs while reading the image, ->read_fw_image is freed, and
 * the error is returned.
 *
 * The firmware is a fixed size (CYAPA_FW_SIZE), and is read out in
 * fixed length (CYAPA_FW_READ_SIZE) chunks.
 */
static int cyapa_read_fw(struct cyapa *cyapa)
{
	int ret;
	int addr;

	if (cyapa->read_fw_image)
		return 0;

	ret = cyapa_bl_enter(cyapa);
	if (ret)
		goto err_detect;

	cyapa->read_fw_image = kmalloc(CYAPA_FW_SIZE, GFP_KERNEL);
	if (!cyapa->read_fw_image) {
		ret = -ENOMEM;
		goto err_detect;
	}

	for (addr = 0; addr < CYAPA_FW_SIZE; addr += CYAPA_FW_READ_SIZE) {
		ret = cyapa_read_fw_bytes(cyapa, CYAPA_FW_HDR_START + addr,
					  &cyapa->read_fw_image[addr]);
		if (ret) {
			cyapa_dbg(cyapa, "read_fw failed at addr = %d,"
					" ret = %d\n", addr, ret);
			kfree(cyapa->read_fw_image);
			cyapa->read_fw_image = NULL;
			break;
		}
	}

err_detect:
	cyapa_detect(cyapa);
	return ret;
}

/*
 **************************************************************
 * debugfs interface
 **************************************************************
*/
static int cyapa_debugfs_open(struct inode *inode, struct file *file)
{
	struct cyapa *cyapa = inode->i_private;
	int ret;

	if (!cyapa)
		return -ENODEV;

	ret = mutex_lock_interruptible(&cyapa->debugfs_mutex);
	if (ret)
		return ret;

	if (!kobject_get(&cyapa->client->dev.kobj)) {
		ret = -ENODEV;
		goto out;
	}

	file->private_data = cyapa;

	/*
	 * If firmware hasn't been read yet, read it all in one pass.
	 * Subsequent opens will reuse the data in this same buffer.
	 */
	ret = cyapa_read_fw(cyapa);

out:
	mutex_unlock(&cyapa->debugfs_mutex);
	return ret;
}

static int cyapa_debugfs_release(struct inode *inode, struct file *file)
{
	struct cyapa *cyapa = file->private_data;
	int ret;

	if (!cyapa)
		return 0;

	ret = mutex_lock_interruptible(&cyapa->debugfs_mutex);
	if (ret)
		return ret;
	file->private_data = NULL;
	kobject_put(&cyapa->client->dev.kobj);
	mutex_unlock(&cyapa->debugfs_mutex);

	return 0;
}

/* Return some bytes from the buffered firmware image, starting from *ppos */
static ssize_t cyapa_debugfs_read_fw(struct file *file, char __user *buffer,
				     size_t count, loff_t *ppos)
{
	struct cyapa *cyapa = file->private_data;

	if (!cyapa->read_fw_image)
		return -EINVAL;

	if (*ppos >= CYAPA_FW_SIZE)
		return 0;

	if (count + *ppos > CYAPA_FW_SIZE)
		count = CYAPA_FW_SIZE - *ppos;

	if (copy_to_user(buffer, &cyapa->read_fw_image[*ppos], count))
		return -EFAULT;

	*ppos += count;
	return count;
}

static const struct file_operations cyapa_read_fw_fops = {
	.open = cyapa_debugfs_open,
	.release = cyapa_debugfs_release,
	.read = cyapa_debugfs_read_fw
};

static int cyapa_debugfs_init(struct cyapa *cyapa)
{
	struct device *dev = &cyapa->client->dev;

	if (!cyapa_debugfs_root)
		return -ENODEV;

	cyapa->dentry_dev = debugfs_create_dir(kobject_name(&dev->kobj),
					       cyapa_debugfs_root);

	if (!cyapa->dentry_dev)
		return -ENODEV;

	mutex_init(&cyapa->debugfs_mutex);

	debugfs_create_file(CYAPA_DEBUGFS_READ_FW, S_IRUSR, cyapa->dentry_dev,
			    cyapa, &cyapa_read_fw_fops);
	return 0;
}

/*
 * Sysfs Interface.
 */

#ifdef CONFIG_PM_SLEEP
static ssize_t cyapa_show_suspend_scanrate(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct cyapa *cyapa = dev_get_drvdata(dev);
	int len;
	u8 pwr_cmd = cyapa->suspend_power_mode;

	if (pwr_cmd == PWR_MODE_BTN_ONLY)
		len = scnprintf(buf, PAGE_SIZE, "%s\n", BTN_ONLY_MODE_NAME);
	else
		len = scnprintf(buf, PAGE_SIZE, "%u\n",
				cyapa_pwr_cmd_to_sleep_time(pwr_cmd));
	return len;
}

static ssize_t cyapa_update_suspend_scanrate(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct cyapa *cyapa = dev_get_drvdata(dev);
	u8 pwr_cmd;
	u16 sleep_time;

	if (buf == NULL || count == 0)
		goto invalidparam;

	if (sysfs_streq(buf, BTN_ONLY_MODE_NAME))
		pwr_cmd = PWR_MODE_BTN_ONLY;
	else if (!kstrtou16(buf, 10, &sleep_time))
		pwr_cmd = cyapa_sleep_time_to_pwr_cmd(sleep_time);
	else
		goto invalidparam;

	cyapa->suspend_power_mode = pwr_cmd;
	return count;

invalidparam:
	dev_err(dev, "invalid suspend scanrate ms parameters\n");
	return -EINVAL;
}

static DEVICE_ATTR(suspend_scanrate_ms, S_IRUGO|S_IWUSR,
		   cyapa_show_suspend_scanrate,
		   cyapa_update_suspend_scanrate);

static struct attribute *cyapa_power_wakeup_entries[] = {
	&dev_attr_suspend_scanrate_ms.attr,
	NULL,
};

static const struct attribute_group cyapa_power_wakeup_group = {
	.name = power_group_name,
	.attrs = cyapa_power_wakeup_entries,
};
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM_RUNTIME
static ssize_t cyapa_show_rt_suspend_scanrate(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct cyapa *cyapa = dev_get_drvdata(dev);
	u8 pwr_cmd = cyapa->runtime_suspend_power_mode;

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 cyapa_pwr_cmd_to_sleep_time(pwr_cmd));
}

static ssize_t cyapa_update_rt_suspend_scanrate(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct cyapa *cyapa = dev_get_drvdata(dev);
	u16 time;
	cyapa_dbg(cyapa, "======< cyapa_update_rt_suspend_scanrate >======");

	if (buf == NULL || count == 0 || kstrtou16(buf, 10, &time)) {
		dev_err(dev, "invalid runtime suspend scanrate ms parameter\n");
		return -EINVAL;
	}

	/*
	 * When the suspend scanrate is changed, pm_runtime_get to resume
	 * a potentially suspended device, update to the new pwr_cmd
	 * and then pm_runtime_put to suspend into the new power mode.
	 */
	pm_runtime_get_sync(dev);
	cyapa->runtime_suspend_power_mode = cyapa_sleep_time_to_pwr_cmd(time);
	pm_runtime_put_sync_autosuspend(dev);
	return count;
}

static DEVICE_ATTR(runtime_suspend_scanrate_ms, S_IRUGO|S_IWUSR,
		   cyapa_show_rt_suspend_scanrate,
		   cyapa_update_rt_suspend_scanrate);

static struct attribute *cyapa_power_runtime_entries[] = {
	&dev_attr_runtime_suspend_scanrate_ms.attr,
	NULL,
};

static const struct attribute_group cyapa_power_runtime_group = {
	.name = power_group_name,
	.attrs = cyapa_power_runtime_entries,
};
#endif /* CONFIG_PM_RUNTIME */

static ssize_t cyapa_show_fm_ver(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct cyapa *cyapa = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d.%d\n", cyapa->fw_maj_ver,
			 cyapa->fw_min_ver);
}

static ssize_t cyapa_show_product_id(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct cyapa *cyapa = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%s\n", cyapa->product_id);
}

static ssize_t cyapa_update_fw_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct cyapa *cyapa = dev_get_drvdata(dev);
	const char *fw_name;
	int ret;

	/* Do not allow paths that step out of /lib/firmware  */
	if (strstr(buf, "../") != NULL)
		return -EINVAL;

	fw_name = !strncmp(buf, "1", count) ||
		  !strncmp(buf, "1\n", count) ? CYAPA_FW_NAME : buf;

	ret = cyapa_firmware(cyapa, fw_name);
	if (ret)
		dev_err(dev, "firmware update failed, %d\n", ret);
	else
		dev_dbg(dev, "firmware update succeeded\n");

	return ret ? ret : count;
}

static ssize_t cyapa_calibrate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct cyapa *cyapa = dev_get_drvdata(dev);
	int tries = 20;  /* max recalibration timeout 2s. */
	int ret;
	cyapa_dbg(cyapa, "======< cyapa_calibrate_store >======");

	disable_irq(cyapa->irq);

	ret = cyapa_read_byte(cyapa, CYAPA_CMD_DEV_STATUS);
	if (ret < 0) {
		dev_err(dev, "Error reading dev status. err = %d\n", ret);
		goto out;
	}
	if ((ret & CYAPA_DEV_NORMAL) != CYAPA_DEV_NORMAL) {
		dev_warn(dev, "Trackpad device is busy. device state = 0x%x\n",
			 ret);
		ret = -EAGAIN;
		goto out;
	}

	ret = cyapa_write_byte(cyapa, CYAPA_CMD_SOFT_RESET,
			       OP_RECALIBRATION_MASK);
	if (ret < 0) {
		dev_err(dev, "Failed to send calibrate command. ret = %d\n",
			ret);
		goto out;
	}

	do {
		/*
		 * For this recalibration, the max time will not exceed 2s.
		 * The average time is approximately 500 - 700 ms, and we
		 * will check the status every 100 - 200ms.
		 */
		usleep_range(100000, 200000);

		ret = cyapa_read_byte(cyapa, CYAPA_CMD_DEV_STATUS);
		if (ret < 0) {
			dev_err(dev, "Error reading dev status. err = %d\n",
				ret);
			goto out;
		}
		if ((ret & CYAPA_DEV_NORMAL) == CYAPA_DEV_NORMAL)
			break;
	} while (--tries);

	if (tries == 0) {
		dev_err(dev, "Failed to calibrate. Timeout.\n");
		ret = -ETIMEDOUT;
		goto out;
	}
	dev_dbg(dev, "Calibration successful.\n");

out:
	enable_irq(cyapa->irq);
	return ret < 0 ? ret : count;
}

static ssize_t cyapa_show_baseline(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct cyapa *cyapa = dev_get_drvdata(dev);
	int max_baseline, min_baseline;
	int tries = 3;
	int ret;
	cyapa_dbg(cyapa, "======< cyapa_show_baseline >======");

	disable_irq(cyapa->irq);

	ret = cyapa_read_byte(cyapa, CYAPA_CMD_DEV_STATUS);
	if (ret < 0) {
		dev_err(dev, "Error reading dev status. err = %d\n", ret);
		goto out;
	}
	if ((ret & CYAPA_DEV_NORMAL) != CYAPA_DEV_NORMAL) {
		dev_warn(dev, "Trackpad device is busy. device state = 0x%x\n",
			 ret);
		ret = -EAGAIN;
		goto out;
	}

	ret = cyapa_write_byte(cyapa, CYAPA_CMD_SOFT_RESET,
			       OP_REPORT_BASELINE_MASK);
	if (ret < 0) {
		dev_err(dev, "Failed to send report baseline command. %d\n",
			ret);
		goto out;
	}

	do {
		usleep_range(10000, 20000);

		ret = cyapa_read_byte(cyapa, CYAPA_CMD_DEV_STATUS);
		if (ret < 0) {
			dev_err(dev, "Error reading dev status. err = %d\n",
				ret);
			goto out;
		}
		if ((ret & CYAPA_DEV_NORMAL) == CYAPA_DEV_NORMAL)
			break;
	} while (--tries);

	if (tries == 0) {
		dev_err(dev, "Device timed out going to Normal state.\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	ret = cyapa_read_byte(cyapa, CYAPA_CMD_MAX_BASELINE);
	if (ret < 0) {
		dev_err(dev, "Failed to read max baseline. err = %d\n", ret);
		goto out;
	}
	max_baseline = ret;

	ret = cyapa_read_byte(cyapa, CYAPA_CMD_MIN_BASELINE);
	if (ret < 0) {
		dev_err(dev, "Failed to read min baseline. err = %d\n", ret);
		goto out;
	}
	min_baseline = ret;

	dev_dbg(dev, "Baseline report successful. Max: %d Min: %d\n",
		max_baseline, min_baseline);
	ret = scnprintf(buf, PAGE_SIZE, "%d %d\n", max_baseline, min_baseline);

out:
	enable_irq(cyapa->irq);
	return ret;
}

static DEVICE_ATTR(firmware_version, S_IRUGO, cyapa_show_fm_ver, NULL);
static DEVICE_ATTR(product_id, S_IRUGO, cyapa_show_product_id, NULL);
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, cyapa_update_fw_store);
static DEVICE_ATTR(baseline, S_IRUGO, cyapa_show_baseline, NULL);
static DEVICE_ATTR(calibrate, S_IWUSR, NULL, cyapa_calibrate_store);

static struct attribute *cyapa_sysfs_entries[] = {
	&dev_attr_firmware_version.attr,
	&dev_attr_product_id.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_baseline.attr,
	&dev_attr_calibrate.attr,
	NULL,
};

static const struct attribute_group cyapa_sysfs_group = {
	.attrs = cyapa_sysfs_entries,
};

#ifdef CONFIG_PM_RUNTIME
static void cyapa_start_runtime(struct cyapa *cyapa)
{
	struct device *dev = &cyapa->client->dev;
	cyapa_dbg(cyapa, "======< cyapa_start_runtime >======");

	cyapa->runtime_suspend_power_mode = PWR_MODE_IDLE;
	if (sysfs_merge_group(&dev->kobj, &cyapa_power_runtime_group))
		dev_warn(dev, "error creating wakeup runtime entries.\n");
	pm_runtime_set_active(dev);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, AUTOSUSPEND_DELAY);
	pm_runtime_enable(dev);
}
#else
static void cyapa_start_runtime(struct cyapa *cyapa) {}
#endif /* CONFIG_PM_RUNTIME */

static void cyapa_detect_and_start(void *data, async_cookie_t cookie)
{
	struct cyapa *cyapa = data;

	cyapa_detect(cyapa);

	cyapa_start_runtime(cyapa);
}

static int cyapa_probe(struct i2c_client *client,
		       const struct i2c_device_id *dev_id)
{
	int ret;
	u8 adapter_func;
	struct cyapa *cyapa;
	struct device *dev = &client->dev;
	union i2c_smbus_data dummy;

	adapter_func = cyapa_check_adapter_functionality(client);
	if (adapter_func == CYAPA_ADAPTER_FUNC_NONE) {
		dev_err(dev, "not a supported I2C/SMBus adapter\n");
		return -EIO;
	}

	/* Make sure there is something at this address */
	if (dev->of_node && i2c_smbus_xfer(client->adapter, client->addr, 0,
			 I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &dummy) < 0)
		return -ENODEV;

	cyapa = kzalloc(sizeof(struct cyapa), GFP_KERNEL);
	if (!cyapa) {
		dev_err(dev, "allocate memory for cyapa failed\n");
		return -ENOMEM;
	}

	cyapa->debug = false;
	cyapa->gen = CYAPA_GEN3;
	cyapa->client = client;
	i2c_set_clientdata(client, cyapa);
	sprintf(cyapa->phys, "i2c-%d-%04x/input0", client->adapter->nr,
		client->addr);

	/* i2c isn't supported, use smbus */
	if (adapter_func == CYAPA_ADAPTER_FUNC_SMBUS)
		cyapa->smbus = true;
	cyapa->state = CYAPA_STATE_NO_DEVICE;
	cyapa->suspend_power_mode = PWR_MODE_SLEEP;

	cyapa->irq = client->irq;

	ret = request_threaded_irq(cyapa->irq,
				   NULL,
				   cyapa_irq,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "cyapa",
				   cyapa);
	if (ret) {
		dev_err(dev, "IRQ request failed: %d\n, ", ret);
		goto err_unregister_device;
	}
	disable_irq(cyapa->irq);

	if (sysfs_create_group(&client->dev.kobj, &cyapa_sysfs_group))
		dev_warn(dev, "error creating sysfs entries.\n");

	if (cyapa_debugfs_init(cyapa))
		dev_warn(dev, "error creating debugfs entries.\n");

#ifdef CONFIG_PM_SLEEP
	if (device_can_wakeup(dev) &&
	    sysfs_merge_group(&client->dev.kobj, &cyapa_power_wakeup_group))
		dev_warn(dev, "error creating wakeup power entries.\n");
#endif /* CONFIG_PM_SLEEP */

	async_schedule(cyapa_detect_and_start, cyapa);
	return 0;

err_unregister_device:
	if (cyapa->input)
		input_unregister_device(cyapa->input);
	kfree(cyapa);

	return ret;
}

static int cyapa_remove(struct i2c_client *client)
{
	struct cyapa *cyapa = i2c_get_clientdata(client);

	pm_runtime_disable(&client->dev);
	sysfs_remove_group(&client->dev.kobj, &cyapa_sysfs_group);

#ifdef CONFIG_PM_SLEEP
	sysfs_unmerge_group(&client->dev.kobj, &cyapa_power_wakeup_group);
#endif

#ifdef CONFIG_PM_RUNTIME
	sysfs_unmerge_group(&client->dev.kobj, &cyapa_power_runtime_group);
#endif

	free_irq(cyapa->irq, cyapa);

	if (cyapa->dentry_dev) {
		debugfs_remove_recursive(cyapa->dentry_dev);
		mutex_destroy(&cyapa->debugfs_mutex);
	}

	input_unregister_device(cyapa->input);
	cyapa_set_power_mode(cyapa, PWR_MODE_OFF);
	kfree(cyapa);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cyapa_suspend(struct device *dev)
{
	int ret;
	u8 power_mode;
	struct cyapa *cyapa = dev_get_drvdata(dev);
	cyapa_dbg(cyapa, "======< cyapa_suspend >======");

	/* Skip the following steps if device is already powered off. */
	if (cyapa->input && cyapa->input->inhibited)
		return 0;

	disable_irq(cyapa->irq);

	/*
	 * Set trackpad device to idle mode if wakeup is allowed,
	 * otherwise turn off.
	 */
	power_mode = device_may_wakeup(dev) ? cyapa->suspend_power_mode
					    : PWR_MODE_OFF;
	ret = cyapa_set_power_mode(cyapa, power_mode);
	if (ret < 0)
		dev_err(dev, "suspend set power mode failed, %d\n", ret);

	if (device_may_wakeup(dev))
		cyapa->irq_wake = (enable_irq_wake(cyapa->irq) == 0);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static int cyapa_resume(struct device *dev)
{
	int ret;
	struct cyapa *cyapa = dev_get_drvdata(dev);
	cyapa_dbg(cyapa, "======< cyapa_resume >======");

	/* If inhibited, defer activation until later. */
	if (cyapa->input && cyapa->input->inhibited)
		return 0;

	if (cyapa->irq_wake) {
		disable_irq_wake(cyapa->irq);
		cyapa->irq_wake = false;
	}

	/* runtime set active to reflect active state. */
	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);

	cyapa_detect(cyapa);

	ret = cyapa_set_power_mode(cyapa, PWR_MODE_FULL_ACTIVE);
	if (ret)
		dev_warn(dev, "set_power_mode after resume failed, %d\n", ret);

	enable_irq(cyapa->irq);
	pm_runtime_enable(dev);

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int cyapa_runtime_suspend(struct device *dev)
{
	int ret;
	struct cyapa *cyapa = dev_get_drvdata(dev);
	cyapa_dbg(cyapa, "======< cyapa_runtime_suspend >======");

	/* set trackpad device to idle mode */
	ret = cyapa_set_power_mode(cyapa, cyapa->runtime_suspend_power_mode);
	if (ret)
		dev_err(dev, "runtime suspend failed, %d\n", ret);
	return ret;
}

static int cyapa_runtime_resume(struct device *dev)
{
	int ret;
	struct cyapa *cyapa = dev_get_drvdata(dev);
	cyapa_dbg(cyapa, "======< cyapa_runtime_resume >======");

	/* resume to full active mode */
	ret = cyapa_set_power_mode(cyapa, PWR_MODE_FULL_ACTIVE);
	if (ret)
		dev_err(dev, "runtime resume failed, %d\n", ret);
	return ret;
}
#endif /* CONFIG_PM_RUNTIME */

static const struct dev_pm_ops cyapa_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cyapa_suspend, cyapa_resume)
	SET_RUNTIME_PM_OPS(cyapa_runtime_suspend, cyapa_runtime_resume, NULL)
};

static const struct i2c_device_id cyapa_id_table[] = {
	{ "cyapa", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, cyapa_id_table);

#ifdef CONFIG_ACPI
static const struct acpi_device_id cyapa_acpi_id[] = {
	{ "CYAP0000", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, cyapa_acpi_id);
#endif

static struct i2c_driver cyapa_driver = {
	.driver = {
		.name = "cyapa",
		.owner = THIS_MODULE,
		.pm = &cyapa_pm_ops,
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(cyapa_acpi_id),
#endif
	},

	.probe = cyapa_probe,
	.remove = cyapa_remove,
	.id_table = cyapa_id_table,
};

static int __init cyapa_init(void)
{
	int ret;

	/* Create a global debugfs root for all cyapa devices */
	cyapa_debugfs_root = debugfs_create_dir("cyapa", NULL);
	if (cyapa_debugfs_root == ERR_PTR(-ENODEV))
		cyapa_debugfs_root = NULL;

	ret = i2c_add_driver(&cyapa_driver);
	if (ret) {
		pr_err("cyapa driver register FAILED.\n");
		return ret;
	}

	return ret;
}

static void __exit cyapa_exit(void)
{
	if (cyapa_debugfs_root)
		debugfs_remove_recursive(cyapa_debugfs_root);

	i2c_del_driver(&cyapa_driver);
}

module_init(cyapa_init);
module_exit(cyapa_exit);

MODULE_DESCRIPTION("Cypress APA I2C Trackpad Driver");
MODULE_AUTHOR("Dudley Du <dudl@cypress.com>");
MODULE_LICENSE("GPL");
