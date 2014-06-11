/*
 * Elan Microelectronics touchpanels with I2C interface
 *
 * Copyright (C) 2014 Elan Microelectronics Corporation.
 * Scott Liu <scott.liu@emc.com.tw>
 *
 * This code is partly based on hid-multitouch.c:
 *
 *  Copyright (c) 2010-2012 Stephane Chatty <chatty@enac.fr>
 *  Copyright (c) 2010-2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 *  Copyright (c) 2010-2012 Ecole Nationale de l'Aviation Civile, France
 *
 *
 * This code is partly based on i2c-hid.c:
 *
 * Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 * Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 * Copyright (c) 2012 Red Hat, Inc
 *
 */

/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/version.h>
#include <linux/kfifo.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/firmware.h>
#include <linux/version.h>
#include <linux/input/mt.h>
#include <linux/acpi.h>
#include <linux/of.h>

/* debug option */
static bool debug = false;
module_param(debug, bool, 0444);
MODULE_PARM_DESC(debug, "print a lot of debug information");

#define elan_dbg(client, fmt, arg...) \
	do { \
		if (debug) \
			dev_printk(KERN_DEBUG, &client->dev, fmt, ##arg); \
	} while (0)

#define ENTER_LOG() \
	dev_dbg(&client->dev, "Enter: %s\n", __func__)


/* Device, Driver information */
#define DEVICE_NAME	"elants_i2c"
#define DRV_VERSION	"1.0.8"

/* Finger report description */
#define MAX_CONTACT_NUM	10

/* Buffer size, used for Read command handshake */
#define FIFO_SIZE		64
#define MAX_MESSAGE_SIZE	256


/*Convert from rows or columns into resolution */
#define ELAN_TS_RESOLUTION(n, m)   ((n - 1) * m)

/* Firmware boot mode packets definition */
#define HELLO_PACKET_LEN	4
static const char hello_packet[HELLO_PACKET_LEN] = {0x55, 0x55, 0x55, 0x55};
static const char recov_packet[HELLO_PACKET_LEN] = {0x55, 0x55, 0x80, 0x80};

/* Elan I2C Address definition */
#define	DEV_MASTER	0x10
#define	DEV_SLAVE1	0x20
#define	DEV_SLAVE2	0x21
#define	MAX_DEVICE	3

/* Finger report information */
#define	REPORT_HEADER_10_FINGER	0x62
#define	PACKET_SIZE	55
#define	MAX_PACKET_LEN	169

/* Buffer mode Queue Header information */
#define	QUEUE_HEADER_SINGLE	0x62
#define	QUEUE_HEADER_NORMAL	0X63
#define	QUEUE_HEADER_WAIT	0x64
#define	QUEUE_HEADER_SIZE	4

/* Command header definition */
#define	CMD_HEADER_WRITE	0x54
#define	CMD_HEADER_READ		0x53
#define	CMD_HEADER_6B_READ	0x5B
#define	CMD_HEADER_RESP		0x52
#define	CMD_HEADER_6B_RESP	0x9B
#define	CMD_HEADER_HELLO	0x55
#define	CMD_HEADER_REK		0x66
#define	CMD_RESP_LEN		4

/* FW information position */
#define	FW_POS_HEADER	0
#define	FW_POS_STATE	1
#define	FW_POS_TOTAL	2
#define	FW_POS_CHECKSUM	34
#define	FW_POS_WIDTH	35
#define	FW_POS_PRESSURE	45

/* test_bit definition */
#define	LOCK_FILE_OPERATE	0
#define	LOCK_CMD_HANDSHAKE	1
#define	LOCK_FINGER_REPORT	2
#define	LOCK_FW_UPDATE	3

/* kfifo return value definition */
#define	RET_OK	0
#define	RET_CMDRSP	1
#define	RET_FAIL	-1

/* define boot condition definition */
#define	E_BOOT_NORM	0
#define	E_BOOT_IAP	1

/* Firmware */
#define	IAP_MODE_ENABLE	1
#define	BOOT_TIME_DELAY_MS	50

/* FW read command, 0x53 0x?? 0x0, 0x01 */
#define	E_ELAN_INFO_FW_VER	0x00
#define	E_ELAN_INFO_BC_VER	0x10
#define	E_ELAN_INFO_TEST_VER	0xE0
#define	E_ELAN_INFO_FW_ID	0xF0
#define	E_INFO_OSR	0xD6
#define	E_INFO_PHY_SCAN	0xD7
#define	E_INFO_PHY_DRIVER	0xD8

#define	MAX_RETRIES	3
#define	MAX_FW_UPDATE_RETRIES	30

#define	ELAN_FW_PAGENUM	351
#define	ELAN_FW_PAGESIZE	132
#define	ELAN_FW_FILENAME	"elants_i2c.bin"

/* calibration timeout definition */
#define	ELAN_CALI_TIMEOUT_MSEC	10000

/*
 * struct multi_queue_header - used by buffer queue header
 *
 * packet_id: packet_id represented status of buffer.
 * report_count: number of finger report in buffer.
 * report_length: total length exclusive queue length.
 */
struct multi_queue_header {
	u8 packet_id;
	u8 report_count;
	u8 report_length;
	u8 reserved;
};

struct mt_slot {
	__s32 x, y, p, w, h;
	__s32 contactid;	/* the device ContactID assigned to this slot */
	bool touch_state;	/* is the touch valid? */
	bool seen_in_this_frame;	/* has this slot been updated */
};

struct mt_device {
	struct mt_slot curdata;	/* placeholder of incoming data */
	__u8 num_received;	/* how many contacts we received */
	bool curvalid;		/* is the current contact valid? */
	unsigned mt_flags;	/* flags to pass to input-mt */
	struct mt_slot *slots;
};


/* struct elants_data - represents a global define of elants device */
struct elants_data {
	bool wake_irq_enabled;

	u16 fw_id;
	u16 fw_version;
	u8 test_version;
	u8 solution_version;
	u8 bc_version;
	u8 iap_version;

	int osr;	/* interpolating  trace */
	int x_res;	/* resolution in units/mm */
	int y_res;
	int rows;	/* trace numbers */
	int cols;
	int x_max;	/* Max ABS resolution */
	int y_max;
	unsigned int iap_mode;
	unsigned int rx_size;

	u8 packet_size;

	struct multi_queue_header mq_header;

	struct i2c_client *client;
	struct input_dev *input;

	struct mutex i2c_mutex;	/* Protects I2C accesses to device */

	unsigned long flags;

	unsigned short i2caddr;

	struct kfifo fifo;
	struct mutex fifo_mutex;
	wait_queue_head_t wait;
	spinlock_t rx_kfifo_lock;

	struct mt_device td;

	/* fields required for debug fs */
	struct mutex dbfs_mutex;
	struct dentry *dbfs_root;

	/* Add for TS driver debug */
	long int irq_count;
	long int packet_count;
	long int touched_sync;
	long int no_touched_sync;
	long int wdt_reset;
	u16 checksum_fail;
	u16 header_fail;
	u16 mq_header_fail;
};

static int elan_touch_pull_frame(struct elants_data *ts, u8 *buf);
static int elan_initialize(struct i2c_client *client);
static int elan_fw_update(struct i2c_client *client);

/*
 *  Function implement
 */
#define elan_set_data(client, data, len)	\
		elan_async_rw(client, data, len, NULL, 0)

#define elan_get_data(client, data, len)	\
		elan_async_rw(client, NULL, 0, data, len)


static inline void elan_msleep(u32 t)
{
	/*
	 * If the sleeping time is 10us - 20ms, usleep_range() is recommended.
	 * Read Documentation/timers/timers-howto.txt
	 */
	usleep_range(t * 1000, t * 1000 + 500);
}

/*
 * Set command or data to our TS.
 *
 * client: the i2c device to recieve from
 * command: command or data which will send to TS
 * cmd_len: The length of the command to send
 * buffer: buffer to store the received data in
 * buf_len: The expected length of the received data.
 */
static int elan_async_rw(struct i2c_client *client,
							const u8 *command,
							const __u16 cmd_len,
							u8 *buffer,
							const __u16 buf_len)
{
	struct elants_data *ts = i2c_get_clientdata(client);
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msgs[2];
	int num_of_msgs = 0;
	int msg_id = 0;
	int rc = 0;

	ENTER_LOG();

	if (cmd_len) {
		elan_dbg(client,
			 "%s cmd: %*phC, addr=%x\n",
			 __func__, (int)cmd_len, command, ts->i2caddr);

		msgs[msg_id].addr = client->addr;
		msgs[msg_id].flags = client->flags & I2C_M_TEN;
		msgs[msg_id].len = cmd_len;
		msgs[msg_id].buf = (u8 *)command;

		num_of_msgs++;
		msg_id++;
	}

	if (buf_len) {
		msgs[msg_id].addr = client->addr;
		msgs[msg_id].flags = client->flags & I2C_M_TEN;
		msgs[msg_id].flags |= I2C_M_RD;
		msgs[msg_id].len = buf_len;
		msgs[msg_id].buf = buffer;

		num_of_msgs++;
	}

	rc = i2c_transfer(adap, msgs, num_of_msgs);
	if (rc < 0) {
		elan_dbg(client, "ts_info error rc=%d\n", rc);
		return rc;
	}

	if (buf_len)
		elan_dbg(client,
			 "%s buffer: %*phC, addr=%x\n",
			 __func__, (int)buf_len, buffer, ts->i2caddr);

	return (rc < 0) ? -EIO : 0;
}

static int elan_i2c_read_block(struct i2c_client *client,
			       u8 *cmd, u8 *val, u16 len)
{
	struct i2c_msg msgs[2];
	int ret;

	ENTER_LOG();

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = 4;
	msgs[0].buf = cmd;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags & I2C_M_TEN;
	msgs[1].flags |= I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = val;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return (ret == 2) ? len : ret;
}

static int elan_dbfs_open(struct inode *inode, struct file *file)
{
	int retval = 0;
	struct elants_data *ts = inode->i_private;
	struct i2c_client *client = ts->client;

	ENTER_LOG();

	if (!ts)
		return -ENODEV;

	disable_irq(ts->client->irq);

	retval = mutex_lock_interruptible(&ts->dbfs_mutex);
	if (retval)
		return retval;

	if (!kobject_get(&ts->client->dev.kobj)) {
		retval = -ENODEV;
		goto dbfs_out;
	}

	file->private_data = ts;
dbfs_out:
	mutex_unlock(&ts->dbfs_mutex);
	return retval;
}

static ssize_t elan_dbfs_read(struct file *file,
			      char __user *buffer, size_t count, loff_t *ppos)
{
	u8 rxbuf[256];
	int ret = -1;
	struct elants_data *ts = file->private_data;
	struct i2c_adapter *adap = ts->client->adapter;
	struct i2c_msg msg;

	if (count > 256)
		return -EMSGSIZE;
	msg.addr = ts->i2caddr;
	msg.flags = ts->client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = rxbuf;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret == 1)
		if (copy_to_user(buffer, rxbuf, count))
			return -EFAULT;

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? count : ret;
}

static ssize_t elan_dbfs_write(struct file *file,
			       const char __user *buffer, size_t count,
			       loff_t *ppos)
{
	int ret;
	u8 txbuf[256];
	struct elants_data *ts = file->private_data;
	struct i2c_adapter *adap = ts->client->adapter;
	struct i2c_msg msg;

	if (count > 256)
		return -EMSGSIZE;

	if (copy_from_user(txbuf, buffer, count))
		return -EFAULT;

	msg.addr = ts->i2caddr;
	msg.flags = ts->client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (char *)txbuf;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret != 1)
		dev_err(&ts->client->dev,
			"i2c_master_send fail, ret=%d\n", ret);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? count : ret;
}

static int elan_dbfs_release(struct inode *inode, struct file *file)
{
	struct elants_data *ts = file->private_data;

	if (!ts)
		return -ENODEV;

	enable_irq(ts->client->irq);
	mutex_unlock(&ts->dbfs_mutex);

	return 0;
}

static const struct file_operations elan_debug_fops = {
	.owner = THIS_MODULE,
	.open = elan_dbfs_open,
	.release = elan_dbfs_release,
	.read = elan_dbfs_read,
	.write = elan_dbfs_write,
};

static int elan_dbfs_init(struct elants_data *ts)
{
	/* Create a global debugfs root for all elan ts devices */
	ts->dbfs_root = debugfs_create_dir(DEVICE_NAME, NULL);
	if (ts->dbfs_root == ERR_PTR(-ENODEV))
		ts->dbfs_root = NULL;

	mutex_init(&ts->dbfs_mutex);

	debugfs_create_file("elan-iap",
		0666, ts->dbfs_root, ts, &elan_debug_fops);

	return 0;
}

static void elan_dbfs_remove(struct elants_data *ts)
{
	debugfs_remove_recursive(ts->dbfs_root);
	mutex_destroy(&ts->dbfs_mutex);

	return;
}

static int elan_calibrate(struct i2c_client *client)
{
	struct elants_data *ts = i2c_get_clientdata(client);
	int ret = 0;
	const u8 w_flashkey[4] = { 0x54, 0xC0, 0xE1, 0x5A };
	const u8 rek[4] = { 0x54, 0x29, 0x00, 0x01 };
	const u8 resp_rek[4] = { CMD_HEADER_REK, 0x66, 0x66, 0x66 };
	u8 rbuf[4] = { 0 };

	ENTER_LOG();

	ts->i2caddr = DEV_MASTER;
	elan_set_data(client, w_flashkey, 4);
	elan_set_data(client, rek, 4);

	/* We will wait for non O_NONBLOCK handles until a signal or data */
	mutex_lock(&ts->fifo_mutex);

	while (kfifo_len(&ts->fifo) == 0) {
		mutex_unlock(&ts->fifo_mutex);
		ret =
		    wait_event_interruptible_timeout(ts->wait,
						     kfifo_len(&ts->fifo),
						     msecs_to_jiffies
						     (ELAN_CALI_TIMEOUT_MSEC));
		if (ret <= 0) {
			ret = -ETIMEDOUT;
			dev_err(&client->dev, "timeout!! wake_up(ts->wait)\n");
			goto err2;
		}
		mutex_lock(&ts->fifo_mutex);
	}
	if (elan_touch_pull_frame(ts, rbuf) < 0) {
		ret = -EINVAL;
		dev_err(&client->dev, "pull_frame fail!!\n");
		goto err1;
	}

	dev_info(&client->dev, "Get Data [%*phC]\n", 4, rbuf);
	ret = 0;
err1:
	mutex_unlock(&ts->fifo_mutex);
err2:
	if (memcmp(resp_rek, rbuf, 4)) {
		ret = -EINVAL;
		dev_err(&client->dev, "reK fail!!\n");
	}

	return ret;
}

/*
 * sysfs interface
 */
static ssize_t show_calibrate(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	ret = elan_calibrate(client);
	return sprintf(buf, "%s\n",
		       (ret == 0) ? "calibrate finish" : "calibrate fail");
}

static ssize_t write_update_fw(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	ret = elan_fw_update(client);
	if (ret)
		dev_err(dev, "firmware update failed.\n");
	else
		dev_dbg(dev, "firmware update succeeded.\n");

	return ret ? ret : count;
}

static ssize_t show_fw_version_value(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elants_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%.4x\n", ts->fw_version);
}

static ssize_t show_hw_version_value(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elants_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%.4x\n", ts->fw_id);
}

static ssize_t show_test_version_value(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elants_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "Test:%.2x\nSolution:%.2x\n",
		       ts->test_version, ts->solution_version);
}

static ssize_t show_bc_version_value(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elants_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "Bootcode:%.2x\n"
		       "IAP:%.2x\n", ts->bc_version, ts->iap_version);
}

static ssize_t show_drv_version_value(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DRV_VERSION);
}

static ssize_t show_iap_mode(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elants_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%s\n",
		       (ts->iap_mode == 0) ? "Normal" : "Recovery");
}

static DEVICE_ATTR(calibrate, S_IRUGO, show_calibrate, NULL);
static DEVICE_ATTR(fw_version, S_IRUGO, show_fw_version_value, NULL);
static DEVICE_ATTR(hw_version, S_IRUGO, show_hw_version_value, NULL);
static DEVICE_ATTR(test_version, S_IRUGO, show_test_version_value, NULL);
static DEVICE_ATTR(bc_version, S_IRUGO, show_bc_version_value, NULL);
static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);
static DEVICE_ATTR(iap_mode, S_IRUGO, show_iap_mode, NULL);
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, write_update_fw);

static struct attribute *elan_attributes[] = {
	&dev_attr_calibrate.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_test_version.attr,
	&dev_attr_bc_version.attr,
	&dev_attr_drv_version.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_iap_mode.attr,
	NULL
};

static struct attribute_group elan_attribute_group = {
	.attrs = elan_attributes,
};

/*
 * Software reset to our TS.
 *
 * ret >0 means reset success,
 * otherwise is fail.
 */
static int elan_sw_reset(struct i2c_client *client)
{
	int ret;
	const u8 soft_rst_cmd[4] = {0x77, 0x77, 0x77, 0x77};
	int len_rst_cmd = sizeof(soft_rst_cmd);

	ENTER_LOG();

	ret = i2c_master_send(client, soft_rst_cmd, len_rst_cmd);

	if (ret != len_rst_cmd) {
		dev_err(&client->dev, "%s: i2c_master_send failed\n", __func__);
		return -ENODEV;
	}

	/* Wait to send fastboot command */
	elan_msleep(10);

	return ret;
}

static int elan_boot(struct i2c_client *client, u16 addr, u8 type)
{
	int rc;
	struct elants_data *ts = i2c_get_clientdata(client);
	u8 command[2][4] = {
		{0x4D, 0x61, 0x69, 0x6E},
		{0x45, 0x49, 0x41, 0x50},
	};

	ENTER_LOG();

	ts->i2caddr = addr;
	rc = elan_set_data(client, command[type], 4);
	if (rc < 0) {
		if (type == E_BOOT_IAP)
			dev_err(&client->dev, "Boot IAP fail, error=%d\n", rc);
		else
			dev_dbg(&client->dev,
				"Boot normal fail, error=%d\n", rc);
		ts->i2caddr = DEV_MASTER;
		return -EINVAL;
	}
	dev_info(&client->dev, "Boot success -- 0x%x\n", addr);
	ts->i2caddr = DEV_MASTER;
	return 0;
}

/*
 * This is always the first packet that comes from
 * firmware and we need to receive it.
 *
 * Normal hello packet is {0x55, 0x55, 0x55, 0x55}
 * Recovery mode packet is {0x55, 0x55, 0x80, 0x80}
 */
static int __hello_packet_handler(struct i2c_client *client)
{
	int rc = 0;
	u8 buf_recv[HELLO_PACKET_LEN] = {0};
	struct elants_data *ts = i2c_get_clientdata(client);

	ENTER_LOG();

	rc = elan_get_data(client, buf_recv, HELLO_PACKET_LEN);

	/* Print buf_recv anyway */
	dev_info(&client->dev, "rc=%d Hello Packet:%*phC\n",
			rc, HELLO_PACKET_LEN, buf_recv);

	if (rc < 0) {
		dev_err(&client->dev,
			"%s: Try recovery because of no hello\n", __func__);
	}

	if (memcmp(buf_recv, hello_packet, HELLO_PACKET_LEN)) {
		if (!memcmp(buf_recv, recov_packet, HELLO_PACKET_LEN)) {
			dev_info(&client->dev,
					"got mainflow recovery message\n");

			ts->iap_mode = IAP_MODE_ENABLE;
			set_bit(LOCK_FW_UPDATE, &ts->flags);
		}

		return -ENODEV;
	}

	ts->i2caddr = DEV_MASTER;

	return rc;
}

static u16 parse_version_number(u8 *buf, size_t len)
{
	u8 version_num[2] = {0};

	if (len != 4)
		return 0xffff;

	version_num[0] = ((buf[1] & 0x0f) << 4) | ((buf[2] & 0xf0) >> 4);
	version_num[1] = ((buf[2] & 0x0f) << 4) | ((buf[3] & 0xf0) >> 4);

	return ((u16)version_num[0] << 8) + (u16)version_num[1];
}

static int __fw_id_packet_handler(struct i2c_client *client)
{
	struct elants_data *ts = i2c_get_clientdata(client);
	int rc, retry_cnt;
	const u8 cmd[] = {CMD_HEADER_READ, E_ELAN_INFO_FW_ID, 0x00, 0x01};
	u8 buf_recv[4] = {0x0};

	ENTER_LOG();

	/* Command not support in IAP recovery mode */
	if (test_bit(LOCK_FW_UPDATE, &ts->flags))
		return 0;

	for (retry_cnt = 0; retry_cnt < MAX_RETRIES; retry_cnt++) {
		rc = elan_i2c_read_block(client, (u8 *) cmd, buf_recv, 4);
		if (rc < 0) {
			elan_dbg(client,
				 "read fw id rc=%d, buf=%*phC\n", rc, 4,
				 buf_recv);
		}

		if (buf_recv[0] == CMD_HEADER_RESP) {
			ts->fw_id =
			    parse_version_number(buf_recv, sizeof(buf_recv));
			if (ts->fw_id == 0xffff) {
				dev_err(&client->dev,
					"FW id is empty, "
					"suggest IAP ELAN chip\n");
				return -EINVAL;
			}
		} else {
			elan_dbg(client, "read fw retry count=%d\n", retry_cnt);
			if (retry_cnt == MAX_RETRIES - 1) {
				ts->fw_id = 0xffff;
				dev_err(&client->dev,
					"Fail to read fw id for %d times, "
					"suggest IAP ELAN chip\n", MAX_RETRIES);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int __fw_version_packet_handler(struct i2c_client *client)
{
	struct elants_data *ts = i2c_get_clientdata(client);
	int rc, retry_cnt;
	const u8 cmd[] = {CMD_HEADER_READ, E_ELAN_INFO_FW_VER, 0x00, 0x01};
	u8 buf_recv[4] = {0x0};

	ENTER_LOG();

	/* Command not support in IAP recovery mode */
	if (test_bit(LOCK_FW_UPDATE, &ts->flags))
		return 0;

	for (retry_cnt = 0; retry_cnt < MAX_RETRIES; retry_cnt++) {
		rc = elan_i2c_read_block(client, (u8 *) cmd, buf_recv, 4);
		if (rc < 0) {
			elan_dbg(client,
				 "read fw version rc=%d, buf=%*phC\n", rc, 4,
				 buf_recv);
		}

		if (buf_recv[0] == CMD_HEADER_RESP) {
			ts->fw_version =
			    parse_version_number(buf_recv, sizeof(buf_recv));
			if ((ts->fw_version == 0x0000) ||
			    (ts->fw_version == 0xffff)) {
				dev_err(&client->dev,
					"FW version is empty, "
					"suggest IAP ELAN chip\n");
				return -EINVAL;
			}
		} else {
			elan_dbg(client, "read fw retry count=%d\n", retry_cnt);
			if (retry_cnt == MAX_RETRIES - 1) {
				ts->fw_version = 0xffff;
				dev_err(&client->dev,
					"Fail to read fw version for %d times, "
					"suggest IAP ELAN chip\n", MAX_RETRIES);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int __test_version_packet_handler(struct i2c_client *client)
{
	struct elants_data *ts = i2c_get_clientdata(client);
	int rc, retry_cnt;
	const u8 cmd[] = { CMD_HEADER_READ,
		E_ELAN_INFO_TEST_VER, 0x00, 0x01
	};
	u8 buf_recv[4] = { 0x0 };

	ENTER_LOG();

	/* Command not support in IAP recovery mode */
	if (test_bit(LOCK_FW_UPDATE, &ts->flags))
		return 0;

	for (retry_cnt = 0; retry_cnt < MAX_RETRIES; retry_cnt++) {
		rc = elan_i2c_read_block(client, (u8 *) cmd, buf_recv, 4);
		if (rc < 0) {
			elan_dbg(client,
				 "read test version error rc=%d, buf=%*phC\n",
				 rc, 4, buf_recv);
			return rc;
		}

		if (buf_recv[0] == CMD_HEADER_RESP) {
			ts->test_version =
			    (((buf_recv[1] & 0x0f) << 4) |
			     ((buf_recv[2] & 0xf0) >> 4));
			ts->solution_version =
			    (((buf_recv[2] & 0x0f) << 4) |
			     ((buf_recv[3] & 0xf0) >> 4));
		} else {
			elan_dbg(client, "read fw retry count=%d\n", retry_cnt);
			if (retry_cnt == MAX_RETRIES - 1) {
				ts->test_version = 0xff;
				ts->solution_version = 0xff;
				dev_err(&client->dev,
					"Fail to get test version for %d times.\n",
					MAX_RETRIES);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int __bc_version_packet_handler(struct i2c_client *client)
{
	struct elants_data *ts = i2c_get_clientdata(client);
	const u8 get_bc_ver_cmd[] = { CMD_HEADER_READ,
		E_ELAN_INFO_BC_VER, 0x00, 0x01
	};
	u8 buf_recv[4];
	int rc;

	ENTER_LOG();

	/* Command not support in IAP recovery mode */
	if (test_bit(LOCK_FW_UPDATE, &ts->flags))
		return 0;

	rc = elan_i2c_read_block(client, (u8 *) get_bc_ver_cmd,
				 buf_recv, sizeof(buf_recv));
	if (rc < 0) {
		dev_err(&client->dev,
			"Read BC version error rc=%d, buf=%*phC\n", rc, 4,
			buf_recv);
		return rc;
	}

	ts->bc_version = (((buf_recv[1] & 0x0f) << 4) |
			  ((buf_recv[2] & 0xf0) >> 4));
	ts->iap_version = (((buf_recv[2] & 0x0f) << 4) |
			   ((buf_recv[3] & 0xf0) >> 4));
	return 0;
}

static int __ts_info_handler(struct i2c_client *client)
{
	struct elants_data *ts = i2c_get_clientdata(client);
	int rc;
	u8 buf_recv[17] = {0};
	u16 phy_x, phy_y;
	const u8 get_resolution_cmd[] = {
		CMD_HEADER_6B_READ, 0x00, 0x00, 0x00, 0x00, 0x00};
	const u8 get_osr_cmd[] = {
		CMD_HEADER_READ, E_INFO_OSR, 0x00, 0x01};
	const u8 get_physical_scan_cmd[] = {
		CMD_HEADER_READ, E_INFO_PHY_SCAN, 0x00, 0x01};
	const u8 get_physical_drive_cmd[] = {
		CMD_HEADER_READ, E_INFO_PHY_DRIVER, 0x00, 0x01};

	ENTER_LOG();

	/* Command not support in IAP recovery mode */
	if (test_bit(LOCK_FW_UPDATE, &ts->flags))
		return 0;

	/* Get trace number */
	rc = elan_async_rw(client, (u8 *)get_resolution_cmd,
				(__u16)sizeof(get_resolution_cmd),
				buf_recv, (__u16)sizeof(buf_recv));
	if (rc < 0) {
		elan_dbg(client, "ts_info error rc=%d\n", rc);
		return rc;
	}

	if (buf_recv[0] == CMD_HEADER_6B_RESP) {
		ts->rows = (buf_recv[2] + buf_recv[6] + buf_recv[10]);
		ts->cols = (buf_recv[3] + buf_recv[7] + buf_recv[11]);
	} else
		dev_warn(&client->dev, "Read TS Information failed!\n");

	/* Process mm_to_pixel information */
	rc = elan_async_rw(client, (u8 *)get_osr_cmd,
				(__u16)sizeof(get_osr_cmd),
				buf_recv, (__u16)sizeof(buf_recv));
	if (rc < 0) {
		elan_dbg(client, "ts_info error rc=%d\n", rc);
		return rc;
	}

	if (buf_recv[0] == CMD_HEADER_RESP)
		ts->osr = buf_recv[3];
	else
		dev_warn(&client->dev, "Read TS OSR failed!\n");

	rc = elan_async_rw(client, (u8 *)get_physical_scan_cmd,
				(__u16)sizeof(get_physical_scan_cmd),
				buf_recv, (__u16)sizeof(buf_recv));
	if (rc < 0) {
		elan_dbg(client, "ts_info error rc=%d\n", rc);
		return rc;
	}

	if (buf_recv[0] == CMD_HEADER_RESP)
		phy_x = (buf_recv[2] << 8) | buf_recv[3];
	else
		dev_warn(&client->dev, "Read TS PHY_SCAN failed!\n");

	rc = elan_async_rw(client, (u8 *)get_physical_drive_cmd,
				(__u16)sizeof(get_physical_drive_cmd),
				buf_recv, (__u16)sizeof(buf_recv));
	if (rc < 0) {
		elan_dbg(client, "ts_info error rc=%d\n", rc);
		return rc;
	}

	if (buf_recv[0] == CMD_HEADER_RESP)
		phy_y = (buf_recv[2] << 8) | buf_recv[3];
	else
		dev_warn(&client->dev, "Read TS PHY_DRIVER failed!\n");

	elan_dbg(client, "phy_x=%d, phy_y=%d\n", phy_x, phy_y);

	if (ts->rows > 0 && ts->cols > 0 && ts->osr > 0) {
		/* translate trace number to TS resolution */
		ts->x_max = ELAN_TS_RESOLUTION(ts->rows, ts->osr);
		ts->y_max = ELAN_TS_RESOLUTION(ts->cols, ts->osr);
	} else
		dev_warn(&client->dev, "trace number error, %d,%d,%d\n",
			 ts->rows, ts->cols, ts->osr);

	ts->x_res = DIV_ROUND_CLOSEST(ts->x_max, phy_x);
	ts->y_res = DIV_ROUND_CLOSEST(ts->y_max, phy_y);

	return 0;
}

static int __elan_fastboot(struct i2c_client *client)
{
	int rc = 0;

	ENTER_LOG();

	rc = elan_boot(client, DEV_MASTER, E_BOOT_NORM);
	if (rc < 0)
		return -1;

	/* Wait for Hello packets */
	msleep(BOOT_TIME_DELAY_MS);

	return rc;
}

/**
 * elan_fw_update - Elan firmware update in driver
 *
 * client: our i2c client
 *
 * The firmware name is elants_i2c.bin
 * The file path is located /system/etc/firmware at Android.
 * The file path usually is located at /lib/firmware.
 */
static int elan_fw_update(struct i2c_client *client)
{
	int rc = 0;
	int page = 0;
	int fw_size = 0, fw_pages;
	int retry;
	bool force = false;
	u8 buf[4];
	u16 send_id = DEV_MASTER;
	const struct firmware *p_fw_entry;
	const u8 *fw_data;
	const u8 enter_iap[4] = { 0x45, 0x49, 0x41, 0x50 };
	const u8 enter_iap2[4] = { 0x54, 0x00, 0x12, 0x34 };
	const u8 iap_rc[4] = { 0x55, 0xaa, 0x33, 0xcc };
	const u8 ack_ok[2] = { 0xaa, 0xaa };
	struct elants_data *ts = i2c_get_clientdata(client);

	ENTER_LOG();

	ts->iap_mode = IAP_MODE_ENABLE;
	if (test_bit(LOCK_FW_UPDATE, &ts->flags))
		force = true;

	elan_dbg(client, "IAP Start.\n");

	rc = test_and_set_bit(LOCK_FW_UPDATE, &ts->flags);
	if (rc)
		dev_info(&client->dev, "Recovery IAP detection\n");

	/* avoid interrupt */
	disable_irq(client->irq);

	dev_info(&client->dev, "request_firmware name = %s\n",
		 ELAN_FW_FILENAME);
	rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME, &client->dev);
	if (rc != 0) {
		dev_err(&client->dev, "rc=%d, request_firmware fail\n", rc);
		goto err;
	} else
		elan_dbg(client, "find FW!! Size=%zu\n", p_fw_entry->size);

	fw_data = p_fw_entry->data;
	fw_size = p_fw_entry->size;

	if (fw_size % ELAN_FW_PAGESIZE) {
		dev_err(&client->dev, "Wrong file length(size=%d)\n", fw_size);
		goto err;
	}

	ts->i2caddr = DEV_MASTER;

	/* Recovery mode detection! */
	if (force) {
		elan_set_data(client, enter_iap2, 4);
	} else {
		/* Start IAP Procedure */
		elan_sw_reset(client);

		ts->i2caddr = DEV_MASTER;
		elan_set_data(client, enter_iap, 4);
	}
	elan_msleep(10);

	/* check IAP state */
	rc = elan_get_data(client, buf, 4);
	if (rc < 0) {
		dev_err(&client->dev, "Enter IAP fail!! [Read IAPRC=%d, addr-%.2x fail]\n",
				rc, ts->i2caddr);
		rc = -ENODEV;
		goto err;
	} else {
		if (unlikely(memcmp(buf, iap_rc, 4))) {
			dev_err(&client->dev, "Enter IAP fail!!");
			rc = -ENODEV;
			goto err;
		} else {
			dev_info(&client->dev, "Enter IAP success!");
		}
	}

	ts->i2caddr = DEV_MASTER;
	send_id = ts->i2caddr;
	rc = elan_set_data(client, (const u8 *)&send_id, 1);
	if (rc < 0) {
		dev_err(&client->dev, "send dummy byte error addr=%.2x\n",
			ts->i2caddr);
		rc = -ENODEV;
		goto err;
	}

	/* Clear the last page of Master */
	ts->i2caddr = DEV_MASTER;
	rc = elan_set_data(client, fw_data, ELAN_FW_PAGESIZE);
	if (rc < 0) {
		dev_err(&client->dev, "Clean the last page fail\n");
		rc = -ENODEV;
		goto err;
	}

	rc = elan_get_data(client, buf, 2);
	if (rc < 0) {
		dev_err(&client->dev, "Stage1 IAP get ack fail!\n");
		rc = -ENODATA;
		goto err;
	}

	fw_pages = (fw_size / ELAN_FW_PAGESIZE);
	elan_dbg(client, "IAP Pages = %d\n", fw_pages);

	ts->i2caddr = DEV_MASTER;
	for (page = 0; page < fw_pages; page++) {
		bool page_written_successfully = false;
		for (retry = 0; retry < MAX_FW_UPDATE_RETRIES; retry++) {
			rc = elan_set_data(
				client, fw_data + (page * ELAN_FW_PAGESIZE),
				ELAN_FW_PAGESIZE);
			elan_dbg(client, "IAP_WRITE1...%d\n", page);
			if (rc < 0) {
				dev_err(&client->dev,
					"IAP Write Page data err!! [rc=%d]\n",
					rc);
			}

			rc = elan_get_data(client, buf, 2);
			if (unlikely(rc < 0)) {
				dev_err(&client->dev, "IAP Ack data err!!\n");
				rc = -ENODATA;
				goto err;
			} else {
				elan_dbg(client, "IAP_WRITE2...%d, ret=%x:%x\n",
					 page, buf[0], buf[1]);
				if (memcmp(buf, ack_ok, 2)) {
					dev_err(&client->dev,
						"IAP Get Ack Error [%02x:%02x]!!\n",
						buf[0], buf[1]);
					elan_dbg(client,
						 "page_rewrite retry %d.\n",
						 retry);
				} else {
					dev_info(&client->dev,
						 "Elan fw update..page-%.2d OK\n",
						 page);
					page_written_successfully = true;
					break;
				}
			}
		}

		if (!page_written_successfully) {
			dev_err(&client->dev,
				"IAP Write Page %.2d timed out!!\n", page);
			rc = -ENODEV;
			goto err;
		}
	}

	dev_info(&client->dev, "fw update finish..check OK??\n");

	/* old iap need wait 200ms for WDT and rest is for hello packets */
	msleep(300);

	clear_bit(LOCK_FW_UPDATE, &ts->flags);
	ts->iap_mode = 0;

	rc = elan_initialize(client);
	if (rc < 0) {
		dev_err(&client->dev, "TS Setup handshake fail!! (%d)\n", rc);

		ts->fw_id = 0xffff;
		ts->fw_version = 0xffff;
		ts->test_version = 0xff;
		ts->solution_version = 0xff;
		ts->bc_version = 0xff;
		ts->iap_version = 0xff;

		dev_err(&client->dev, "IAP Update Failure!!\n");
		goto err;
	} else {
		dev_info(&client->dev, "IAP Update Successful!\n");
	}

err:
	/* We don't have update fw info ...
	 * those is kept as previous fw data
	 * and will update after reboot if err case.
	 */
	clear_bit(LOCK_FW_UPDATE, &ts->flags);
	ts->iap_mode = 0;

	release_firmware(p_fw_entry);
	/* We don't release LOCK_FW_UPDATE flag if fail */
	enable_irq(client->irq);
	elan_msleep(100);
	/* we need to calibrate touchscreen */
	elan_calibrate(client);

	return rc;
}

/*
* Pulls a frame from the FIFO into the provided data buffer.
* The data buffer must be at least CMD_RESP_LEN bytes long.
*
* ts: our elan touch device
* buf: data buffer which stored data from fifo.
*/
static int elan_touch_pull_frame(struct elants_data *ts, u8 *buf)
{
	struct i2c_client *client = ts->client;

	ENTER_LOG();

	WARN_ON(kfifo_out_locked(&ts->fifo, buf, CMD_RESP_LEN,
				 &ts->rx_kfifo_lock)
		!= CMD_RESP_LEN);
	return CMD_RESP_LEN;
}

/*
 * Empty old frames out of the FIFO until we can fit the new one into
 * the other end.
 *
 * ts: our elan touch device
 * room_needed: space needed
 */
static void elan_touch_fifo_clean_old(struct elants_data *ts, int room_needed)
{
	u8 buffer[CMD_RESP_LEN];
	struct i2c_client *client = ts->client;

	ENTER_LOG();

	while (kfifo_len(&ts->fifo) + room_needed >= FIFO_SIZE)
		elan_touch_pull_frame(ts, buffer);
}

/*
 * Parsing report header and get data information.
 *
 * client: the i2c device to recieve from
 * buf: buffer to store finger data header.
 */
static int elan_get_repo_info(struct i2c_client *client, u8 *buf)
{
	struct elants_data *ts = i2c_get_clientdata(client);
	struct multi_queue_header *buff = (struct multi_queue_header *)buf;
	struct multi_queue_header *mq = &ts->mq_header;
	const u8 wait_packet[4] = {0x64, 0x64, 0x64, 0x64};
	int times = 10, rc = 0;

	ENTER_LOG();

	switch (buf[FW_POS_HEADER]) {
	case CMD_HEADER_HELLO:
		if (!memcmp(buf, hello_packet, 4))
			ts->wdt_reset++;
		return RET_CMDRSP;
	case CMD_HEADER_RESP:
	case CMD_HEADER_REK:
		/* Queue the data, using the fifo lock to serialize
		 * the multiple accesses to the FIFO
		 */
		elan_dbg(client, "recv CMD_HEADER_RESP\n");

		mutex_lock(&ts->fifo_mutex);
		if (kfifo_len(&ts->fifo) + CMD_RESP_LEN >= FIFO_SIZE)
			elan_touch_fifo_clean_old(ts, CMD_RESP_LEN);
		/* Push the data */
		kfifo_in_locked(&ts->fifo, buf,
				CMD_RESP_LEN, &ts->rx_kfifo_lock);
		mutex_unlock(&ts->fifo_mutex);

		elan_dbg(client, "wake_up [%*phC]\n", 4, buf);
		wake_up_interruptible(&ts->wait);
		return RET_CMDRSP;
		/* Buffer mode header */
	case QUEUE_HEADER_NORMAL:
		elan_dbg(client,
			 "report_count=%d report_len=%d\n",
			 buff->report_count, buff->report_length);

		if (buff->report_count <= 3) {
			mq->report_count = buff->report_count;
			mq->report_length = buff->report_length;
			ts->rx_size = mq->report_length;
			ts->packet_size = mq->report_length / mq->report_count;
		} else
			return RET_FAIL;

		break;
	case QUEUE_HEADER_WAIT:
		elan_dbg(client,
			 "QUEUE_HEADER_WAIT %x:%x:%x:%x\n",
			 buf[0], buf[1], buf[2], buf[3]);

		if (!memcmp(buf + 1, &wait_packet[1], 3)) {
			do {
				udelay(30);
				elan_get_data(client, (u8 *)buff, ts->rx_size);
			} while ((buff->packet_id != QUEUE_HEADER_NORMAL) &&
				 (--times > 0));
			if (times > 0)
				rc = elan_get_repo_info(client, (u8 *)buff);
			else
				return RET_FAIL;
			elan_dbg(client,
				 "Detect Wait_Header:rx_size=%d, "
				 "report_count=%d report_len=%d\n",
				 ts->rx_size,
				 mq->report_count, mq->report_length);
		} else
			dev_err(&client->dev,
				"ERROR!! wait header:%x:%x:%x:%x\n",
				buf[0], buf[1], buf[2], buf[3]);
		break;
		/* Not buffer mode, it's single word mode */
	case QUEUE_HEADER_SINGLE:
		mq->report_count = 1;
		mq->report_length = PACKET_SIZE;
		ts->rx_size = mq->report_length;
		ts->packet_size = mq->report_length / mq->report_count;
		return ts->rx_size;
	default:
		dev_err(&client->dev,
			"unknown multi-queue command!! --%x %x:%x:%x--\n",
			buf[0], buf[1], buf[2], buf[3]);
		ts->mq_header_fail++;

		/* If glitch causes frame error, drop all finger report */
		ts->rx_size = MAX_PACKET_LEN;
		elan_get_data(client, (u8 *)buff, ts->rx_size);
		return RET_FAIL;
	}

	return ts->rx_size;
}

/*
 * Caculating checksum for make sure all data validity.
 *
 * client : the i2c device to recieve from
 * buf : raw finger data from firmware.
 */
static int elan_touch_checksum(struct i2c_client *client, u8 *buf)
{
	u8 i = 0, checksum = 0;
	struct elants_data *ts = i2c_get_clientdata(client);

	ENTER_LOG();

	for (i = 0; i < FW_POS_CHECKSUM; i++)
		checksum = checksum + buf[i];

	if (checksum != buf[FW_POS_CHECKSUM]) {
		ts->checksum_fail++;
		dev_err(&client->dev,
			"elan touch checksum fail: %02x:%02x\n",
			checksum, buf[FW_POS_CHECKSUM]);
		return -EFAULT;
	}

	return 0;
}

/*
 * Parsing the track id of fingers.
 *
 * data: the input bit stream
 * fid: an array of tracking ID values
 */
static inline void elan_parse_fid(u8 *data, u8 *fid)
{
	fid[0] = (data[0] & 0x01);
	fid[1] = (data[0] & 0x02);
	fid[2] = (data[0] & 0x04);
	fid[3] = (data[0] & 0x08);
	fid[4] = (data[0] & 0x10);
	fid[5] = (data[0] & 0x20);
	fid[6] = (data[0] & 0x40);
	fid[7] = (data[0] & 0x80);
	fid[8] = (data[1] & 0x10);
	fid[9] = (data[1] & 0x20);
}

/*
 * Parsing finger widths data with length of 5 bits.
 *
 * data: the input bit stream
 * width: an array of width level
 */
static inline void elan_parse_widths(u8 *data, u8 *width)
{
	int i;

	for (i = 0; i < MAX_CONTACT_NUM; i++)
		width[i] = (data[i] & 0x1f);

	return;
}

/*
 * Parsing finger pressure data with length of 8 bits.
 *
 * data: the input bit stream
 * pressure: an array of pressure level
 */
static inline void elan_parse_pressures(u8 *data, u8 *pressure)
{
	memcpy(pressure, data, MAX_CONTACT_NUM);
	return;
}

static inline int elan_parse_xy(u8 *data, u16 *x, u16 *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int elan_mt_compute_slot(struct mt_device *td)
{
	int i;
	struct elants_data *ts = container_of(td, struct elants_data, td);

	for (i = 0; i < MAX_CONTACT_NUM; ++i) {
		if (td->slots[i].contactid == td->curdata.contactid &&
		    td->slots[i].touch_state)
			return i;
	}
	for (i = 0; i < MAX_CONTACT_NUM; ++i) {
		if (!td->slots[i].seen_in_this_frame &&
		    !td->slots[i].touch_state)
			return i;
	}

	dev_err(&ts->client->dev,
			"finger number exceed %d, ingore it then\n",
			MAX_CONTACT_NUM);
	/* should not occurs. If this happens that means
	 * that the device sent more touches that it says
	 * in the report descriptor. It is ignored then. */
	return -1;
}

/*
* This function is called when a whole contact has been processed,
* so that it can assign it to a slot and store the data there
*/
static void elan_mt_complete_slot(struct mt_device *td)
{
	td->curdata.seen_in_this_frame = true;
	if (td->curvalid) {
		int slotnum = elan_mt_compute_slot(td);

		if (slotnum >= 0 && slotnum < MAX_CONTACT_NUM)
			td->slots[slotnum] = td->curdata;
	}
	td->num_received++;
}

/*
* This function is called when a whole packet has been received and processed,
* so that it can decide what to send to the input layer.
*/
static void elan_mt_emit_event(struct mt_device *td, struct input_dev *input)
{
	struct elants_data *ts = container_of(td, struct elants_data, td);
	int i;

	for (i = 0; i < MAX_CONTACT_NUM; ++i) {
		struct mt_slot *s = &(td->slots[i]);
		if (!s->seen_in_this_frame)
			s->touch_state = false;

		input_mt_slot(input, i);
		input_mt_report_slot_state(input, MT_TOOL_FINGER,
						s->touch_state);
		if (s->touch_state) {
			/* This finger is on the screen */
			int major = max(s->w, s->h), minor = min(s->w, s->h);

			elan_dbg(ts->client, "i=%d x=%d y=%d p=%d w=%d h=%d.\n",
				 i, s->x, s->y, s->p, major, minor);

			input_event(input, EV_ABS, ABS_MT_POSITION_X, s->x);
			input_event(input, EV_ABS, ABS_MT_POSITION_Y, s->y);
			input_event(input, EV_ABS, ABS_MT_PRESSURE, s->p);
			input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, major);
		}
		s->seen_in_this_frame = false;
	}

	input_mt_report_pointer_emulation(input, true);
	input_sync(input);
	td->num_received = 0;
}

/*
 * Walk the received report and process the finger data, extracting
 * and reporting co-ordinates.
 *
 * ts: our touchscreen
 * num_fingers: number of fingers in packet
 * buf: raw finger data
 */
static int elan_mt_event(struct elants_data *ts, int num_fingers, u8 *buf)
{
	int i;
	u8 fid[MAX_CONTACT_NUM] = {0};
	u8 widths[MAX_CONTACT_NUM] = {0};
	u8 pressures[MAX_CONTACT_NUM] = {0};
	struct i2c_client *client = ts->client;
	struct mt_device *td = &ts->td;

	ENTER_LOG();

	/* Parsing Finger, Width, Pressure field */
	elan_parse_fid(&buf[FW_POS_STATE], fid);
	elan_parse_widths(&buf[FW_POS_WIDTH], widths);
	elan_parse_pressures(&buf[FW_POS_PRESSURE], pressures);

	for (i = 0; (i < MAX_CONTACT_NUM) && (num_fingers > 0); i++) {
		/* tracking id */
		td->curdata.contactid = (fid[i] > 0) ? i + 1 : 0;

		if (td->curdata.contactid == 0)
			continue;

		td->curdata.touch_state = true;

		elan_parse_xy(&buf[3 + i * 3],
					(u16 *)&td->curdata.x,
					(u16 *)&td->curdata.y);
		td->curdata.p = pressures[i];
		td->curdata.w = td->curdata.h = widths[i];

		num_fingers--;

		elan_mt_complete_slot(td);
	}

	if (td->num_received >= num_fingers)
		elan_mt_emit_event(td, ts->input);

	return 1;
}

/*
 * To extract finger data and sanity check
 * then send it out to input layer.
 *
 * client : the i2c device to recieve from
 * buf : raw finger data.
 */
static void elan_report_data(struct i2c_client *client, u8 *buf)
{
	struct elants_data *ts = i2c_get_clientdata(client);

	ENTER_LOG();

	switch (buf[FW_POS_HEADER]) {
	case REPORT_HEADER_10_FINGER:{
			u8 num_fingers = buf[FW_POS_TOTAL] & 0x0f;
			elan_dbg(client, "finger_stat == %d\n", num_fingers);
			elan_dbg(client, "finger:%*phC\n", 10, buf);

			/* Enter right process, reset int_status */
			ts->packet_count++;

			if (likely(num_fingers != 0)) {
				ts->td.curvalid = true;
				ts->touched_sync++;
			} else {
				ts->no_touched_sync++;
			}
			elan_mt_event(ts, num_fingers, buf);
		}
		break;
	default:
		ts->header_fail++;
		dev_warn(&client->dev,
			 "%s: unknown packet type: %*phC\n", __func__, 10, buf);
		break;
	}

	return;
}

/*
 * ISR Routine which handler to get finger data
 * header from TS and resolved it then report it
 * to linux input layer.
*/
static irqreturn_t elan_work_func(int irq, void *work)
{
	struct elants_data *ts = work;
	struct i2c_client *client = ts->client;

	u8 buf[MAX_PACKET_LEN] = {0};
	u8 pos = 0;
	int rc = 0;

	ENTER_LOG();

	ts->irq_count++;

	if (test_bit(LOCK_FW_UPDATE, &ts->flags))
		return IRQ_HANDLED;

	mutex_lock(&ts->i2c_mutex);

	set_bit(LOCK_FINGER_REPORT, &ts->flags);

	/* Read multi_queue header */
	rc = elan_get_data(client, (u8 *)buf, ts->rx_size);
	if (rc < 0)
		goto fail;

	/*  Get multi_queue header info */
	rc = elan_get_repo_info(ts->client, buf);
	if (rc < 0 || rc == RET_CMDRSP)
		goto fail;

	/*  check if packet size is valid */
	if ((ts->packet_size != PACKET_SIZE)) {
		dev_err(&ts->client->dev, "%s: incorrect packet size = %d\n",
			__func__, ts->packet_size);
		goto fail;
	}

	/* Get full finger report data according to header length */
	rc = elan_get_data(client, (u8 *)buf, ts->rx_size);
	if (rc < 0)
		goto fail;

	clear_bit(LOCK_FINGER_REPORT, &ts->flags);

	mutex_unlock(&ts->i2c_mutex);

	/* parsing data and send it out */
	while (ts->mq_header.report_count--) {
		if (elan_touch_checksum(ts->client, buf + pos) == 0)
			elan_report_data(ts->client, buf + pos);
		pos = pos + ts->packet_size;
		udelay(10);
	}

	ts->rx_size = QUEUE_HEADER_SIZE;

	return IRQ_HANDLED;

fail:
	clear_bit(LOCK_FINGER_REPORT, &ts->flags);
	mutex_unlock(&ts->i2c_mutex);
	ts->rx_size = QUEUE_HEADER_SIZE;

	return IRQ_HANDLED;
}

static int elan_remove(struct i2c_client *client)
{
	int ret = 0;
	struct elants_data *ts = i2c_get_clientdata(client);

	/* remove dbfs */
	elan_dbfs_remove(ts);

	/* remove sysfs */
	sysfs_remove_group(&client->dev.kobj, &elan_attribute_group);

	if (client->irq)
		free_irq(client->irq, ts);

	if (ts->input)
		input_unregister_device(ts->input);

	if (&ts->i2c_mutex)
		mutex_destroy(&ts->i2c_mutex);
	if (&ts->fifo_mutex)
		mutex_destroy(&ts->fifo_mutex);

	kfree(ts->td.slots);
	kfifo_free(&ts->fifo);
	kfree(ts);

	return ret;
}

static int elan_input_dev_create(struct elants_data *ts)
{
	int err = 0;
	struct i2c_client *client = ts->client;

	/* Clear the existing one if it exists */
	if (ts->input) {
		input_unregister_device(ts->input);
		ts->input = NULL;
	}

	ts->input = input_allocate_device();
	if (ts->input == NULL) {
		dev_err(&client->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	ts->input->name = "Elan-Touchscreen";
	ts->input->id.bustype = BUS_I2C;
	ts->input->dev.parent = &ts->client->dev;

	__set_bit(BTN_TOUCH, ts->input->keybit);
	__set_bit(EV_ABS, ts->input->evbit);
	__set_bit(EV_KEY, ts->input->evbit);

	/* Single touch input params setup */
	input_set_abs_params(ts->input, ABS_X, 0, ts->x_max, 0, 0);
	input_set_abs_params(ts->input, ABS_Y, 0, ts->y_max, 0, 0);
	input_set_abs_params(ts->input, ABS_PRESSURE, 0, 255, 0, 0);
	input_abs_set_res(ts->input, ABS_X, ts->x_res);
	input_abs_set_res(ts->input, ABS_Y, ts->y_res);

	ts->td.mt_flags |= INPUT_MT_DIRECT;

	/* Multitouch input params setup */
	err =
	    input_mt_init_slots(ts->input, MAX_CONTACT_NUM, ts->td.mt_flags);
	if (err) {
		dev_err(&client->dev,
			"allocate memory for MT slots failed, %d\n", err);
		goto err_free_device;
	}

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, ts->x_max, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, ts->y_max, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_abs_set_res(ts->input, ABS_MT_POSITION_X, ts->x_res);
	input_abs_set_res(ts->input, ABS_MT_POSITION_Y, ts->y_res);

	input_set_drvdata(ts->input, ts);

	ts->td.slots = kzalloc(MAX_CONTACT_NUM * sizeof(struct mt_slot),
			       GFP_KERNEL);
	if (!ts->td.slots) {
		dev_err(&client->dev, "cannot allocate multitouch slots\n");
		goto err_free_device;
	}

	err = input_register_device(ts->input);
	if (err) {
		dev_err(&client->dev, "unable to register input device\n");
		goto err_free_slot;
	}

	return 0;

err_free_slot:
	kfree(ts->td.slots);
err_free_device:
	input_free_device(ts->input);
	ts->input = NULL;
	return err;
}

/*
 * Setup for Elan TS and get necessary information.
 * -To reset Elan TS module
 * -To receive hello packet
 * -To get TS resolution
 */
static int elan_initialize(struct i2c_client *client)
{
	struct elants_data *ts = i2c_get_clientdata(client);
	int rc = 0, retry_cnt = 0;

	ENTER_LOG();

	for (retry_cnt = 0; retry_cnt < MAX_RETRIES; retry_cnt++) {
		rc = elan_sw_reset(client);
		if (rc < 0) {
			dev_err(&client->dev, "Software reset failed\n");
			/* Continue initializing if it's the last try */
			if (retry_cnt < MAX_RETRIES - 1)
				continue;
		}

		ts->rx_size = QUEUE_HEADER_SIZE;

		rc = __elan_fastboot(client);
		if (rc < 0) {
			dev_err(&client->dev, "fastboot failed, rc=%d\n", rc);
			/* Continue initializing if it's the last try */
			if (retry_cnt < MAX_RETRIES - 1)
				continue;
		}

		rc = __hello_packet_handler(client);
		if (rc < 0)
			dev_err(&client->dev, "hello packet error\n");
		else
			break;
	}

	rc = __fw_id_packet_handler(client);
	if (rc < 0) {
		dev_err(&client->dev, "firmware id checking error rc=%d\n", rc);

		if (rc == -EINVAL) {
			set_bit(LOCK_FW_UPDATE, &ts->flags);
			ts->iap_mode = IAP_MODE_ENABLE;
		}
	}

	rc = __fw_version_packet_handler(client);
	if (rc < 0) {
		dev_err(&client->dev, "firmware version checking error rc=%d\n",
			rc);

		if (rc == -EINVAL) {
			set_bit(LOCK_FW_UPDATE, &ts->flags);
			ts->iap_mode = IAP_MODE_ENABLE;
		}
	}

	rc = __test_version_packet_handler(client);
	if (rc < 0)
		dev_err(&client->dev, "test version error\n");

	rc = __bc_version_packet_handler(client);
	if (rc < 0)
		dev_err(&client->dev, "TS error getting BC version\n");

	rc = __ts_info_handler(client);
	if (rc < 0)
		dev_err(&client->dev, "TS information checking error\n");

	return 0;
}

/*
 * Perform real probe for our I2C device and if successful configure
 * it up as an input device. If not then clean up and simply return.
 */
static void elan_initialize_async(void *data, async_cookie_t cookie)
{
	struct elants_data *ts = data;
	struct i2c_client *client = ts->client;
	unsigned long irqflags;
	int err = 0;

	mutex_lock(&ts->i2c_mutex);

	err = elan_initialize(client);
	if (err < 0)
		dev_err(&client->dev, "probe failed! unbind device.\n");

	err = elan_input_dev_create(ts);
	if (err) {
		dev_err(&client->dev, "%s crated failed, %d\n", __func__, err);
		goto err_release;
	}

	/*
	 * Systems using device tree should set up interrupt via DTS,
	 * the rest will use the default falling edge interrupts.
	 */
	irqflags = client->dev.of_node ? 0 : IRQF_TRIGGER_FALLING;

	err = request_threaded_irq(client->irq, NULL,
				   elan_work_func,
				   irqflags | IRQF_ONESHOT,
				   client->name, ts);
	if (err) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_release;
	}

	mutex_unlock(&ts->i2c_mutex);

	return;

err_release:
	mutex_unlock(&ts->i2c_mutex);
	elan_remove(client);
	return;
}

/*
 * Perform setup and probe for our I2C device and if successful configure
 * it up as an input device. If not then clean up and return an error
 * code.
 */
static int elan_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;
	union i2c_smbus_data dummy;
	struct elants_data *ts;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"%s: i2c check functionality error\n", DEVICE_NAME);
		return -ENODEV;
	}

	/* Make sure there is something at this address */
	if (i2c_smbus_xfer(client->adapter, client->addr, 0,
			I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &dummy) < 0) {
		dev_err(&client->dev, "nothing at this address\n");
		return -ENODEV;
	}

	ts = kzalloc(sizeof(struct elants_data), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	mutex_init(&ts->i2c_mutex);
	mutex_init(&ts->fifo_mutex);
	init_waitqueue_head(&ts->wait);
	spin_lock_init(&ts->rx_kfifo_lock);

	/* set dbfs for user-space program */
	err = elan_dbfs_init(ts);
	if (err < 0) {
		dev_err(&client->dev, "error create elan debugfs.\n");
		goto err_release;
	}

	if (!client->dev.platform_data)
		dev_err(&client->dev,
			"%s No platform data provided\n", DEVICE_NAME);

	/* set initial i2c address */
	client->addr = DEV_MASTER;
	ts->i2caddr = client->addr;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	/* initial kfifo */
	err = kfifo_alloc(&ts->fifo, FIFO_SIZE, GFP_KERNEL);
	if (err)
		goto err_release;

	if (!kfifo_initialized(&ts->fifo)) {
		dev_err(&client->dev, "%s error kfifo_alloc\n", __func__);
		goto err_release;
	}

	/* Says HELLO to touch device */
	async_schedule(elan_initialize_async, ts);

	/*
	 * Systems using device tree should set up wakeup via DTS,
	 * the rest will configure device as wakeup source by default.
	 */
	if (!client->dev.of_node)
		device_init_wakeup(&client->dev, true);

	/* register sysfs */
	if (sysfs_create_group(&client->dev.kobj, &elan_attribute_group))
		dev_err(&client->dev, "sysfs create group error\n");

	return 0;

err_release:
	elan_remove(client);
	return -err;
}

static const struct i2c_device_id elan_ts_id[] = {
	{DEVICE_NAME, 0},
	{}
};

#ifdef CONFIG_PM_SLEEP
static int elan_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elants_data *ts = i2c_get_clientdata(client);
	const u8 set_sleep_cmd[] = {0x54, 0x50, 0x00, 0x01};
	int rc = 0, retry_cnt;

	ENTER_LOG();

	/* Command not support in IAP recovery mode */
	if (test_bit(LOCK_FW_UPDATE, &ts->flags))
		return 0;

	mutex_lock(&ts->i2c_mutex);

	for (retry_cnt = 0; retry_cnt < MAX_RETRIES; retry_cnt++) {
		rc = elan_set_data(client, set_sleep_cmd,
				   sizeof(set_sleep_cmd));
		if (rc < 0)
			dev_err(&client->dev, "suspend command failed!\n");
		else
			break;
	}

	if (device_may_wakeup(dev))
		ts->wake_irq_enabled = (enable_irq_wake(client->irq) == 0);

	disable_irq(client->irq);

	mutex_unlock(&ts->i2c_mutex);

	return 0;
}

static int elan_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elants_data *ts = i2c_get_clientdata(client);
	const u8 set_active_cmd[] = {0x54, 0x58, 0x00, 0x01};
	int rc = 0, retry_cnt;

	ENTER_LOG();

	/* Command not support in IAP recovery mode */
	if (test_bit(LOCK_FW_UPDATE, &ts->flags))
		return 0;

	if (device_may_wakeup(dev) && ts->wake_irq_enabled)
		disable_irq_wake(client->irq);

	mutex_lock(&ts->i2c_mutex);

	for (retry_cnt = 0; retry_cnt < MAX_RETRIES; retry_cnt++) {
		rc = elan_set_data(client, set_active_cmd,
				   sizeof(set_active_cmd));
		if (rc < 0)
			dev_err(&client->dev, "resume command failed!\n");
		else
			break;
	}

	enable_irq(client->irq);

	mutex_unlock(&ts->i2c_mutex);

	return 0;
}

static SIMPLE_DEV_PM_OPS(elan_pm_ops, elan_suspend, elan_resume);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id elants_acpi_id[] = {
	{ "ELAN0001", 0 },
	{ }
};

MODULE_DEVICE_TABLE(acpi, elants_acpi_id);
#else
MODULE_DEVICE_TABLE(i2c, elan_ts_id);
#endif

#ifdef CONFIG_OF
static const struct of_device_id elan_of_match[] = {
	{ .compatible = "elan,i2c_touchscreen" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, elan_of_match);
#endif

static struct i2c_driver elan_ts_driver = {
	.probe = elan_probe,
	.remove = elan_remove,
	.id_table = elan_ts_id,
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm = &elan_pm_ops,
#endif
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(elants_acpi_id),
#endif
		.of_match_table = of_match_ptr(elan_of_match),
	},
};

module_i2c_driver(elan_ts_driver);


MODULE_AUTHOR("Scott Liu <scott.liu@emc.com.tw>");
MODULE_DESCRIPTION("Elan I2c Touchscreen driver");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
