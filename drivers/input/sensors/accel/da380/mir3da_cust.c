/* For AllWinner android platform.
 *
 * mir3da.c - Linux kernel modules for 3-Axis Accelerometer
 *
 * Copyright (C) 2011-2013 MiraMEMS Sensing Technology Co., Ltd.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>

#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/of_gpio.h>

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon-vid.h>
#include <linux/sensor-dev.h>
#include "mir3da_core.h"
#include "mir3da_cust.h"

#define MIR3DA_DRV_NAME                 	"da380"
#define MIR3DA_INPUT_DEV_NAME     			MIR3DA_DRV_NAME
#define MIR3DA_MISC_NAME                	MIR3DA_DRV_NAME

#define POLL_INTERVAL_MAX               	500
//#define POLL_INTERVAL                   	50
#define INPUT_FUZZ                      	0
#define INPUT_FLAT                      	0

#define RANGE_2G				0
#define RANGE_4G				1
#define RANGE_8G				2
#define RANGE_16G				3

#define DA380_CHIP_ID_REG		0x01
#define DA380_CHIP_ID			0x13

#define GSENSOR_MIN				2
#define MIR3DA_PRECISION		11
#define MIR3DA_RANGE			1000000
#define DA311_BOUNDARY			(0x1 << (MIR3DA_PRECISION - 1))
#define DA311_GRAVITY_STEP		(MIR3DA_RANGE / DA311_BOUNDARY)

#define MIR3DA_INT1				1
#define MIR3DA_INT2				2

#define INT_CONFIG				0x20

static struct input_polled_dev      *mir3da_idev;
static struct device 				*hwmon_dev;
static MIR_HANDLE                   mir_handle;
static unsigned int slope_th;
static u32 active_dur = 0x01;
static u32 active_th  = 0x10;
static u32 range_g = RANGE_2G; //2g:0 4g:1 8g:2 16g:3
static unsigned int                       delayMs = 50;
static da380_config_t *da380_config_p;
extern int                              	Log_level;

#define MI_DATA(format, ...)         	if(DEBUG_DATA&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
#define MI_MSG(format, ...)             if(DEBUG_MSG&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
#define MI_ERR(format, ...)             	if(DEBUG_ERR&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
#define MI_FUN                          	if(DEBUG_FUNC&Log_level){printk(KERN_ERR MI_TAG "%s is called, line: %d\n", __FUNCTION__,__LINE__);}
#define MI_ASSERT(expr)                 \
	if (!(expr)) {\
		printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n",\
			__FILE__, __LINE__, __func__, #expr);\
	}

/*----------------------------------------------------------------------------*/
#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
static char OffsetFileName[] = "/data/misc/miraGSensorOffset.txt";
#define OFFSET_STRING_LEN               26
struct work_info {
	char        tst1[20];
	char        tst2[20];
	char        buffer[OFFSET_STRING_LEN];
	struct      workqueue_struct *wq;
	struct      delayed_work read_work;
	struct      delayed_work write_work;
	struct      completion completion;
	int         len;
	int         rst;
};

static struct work_info m_work_info = {{0}};
/*----------------------------------------------------------------------------*/
static void sensor_write_work(struct work_struct *work)
{
	struct work_info   *pWorkInfo;
	struct file         *filep;
	u32                 orgfs;
	int                 ret;

	orgfs = get_fs();
	set_fs(KERNEL_DS);

	pWorkInfo = container_of((struct delayed_work *)work, struct work_info,
				 write_work);
	if (pWorkInfo == NULL) {
		MI_ERR("get pWorkInfo failed!");
		return;
	}

	filep = filp_open(OffsetFileName, O_RDWR | O_CREAT, 0600);
	if (IS_ERR(filep)) {
		MI_ERR("write, sys_open %s error!!.\n", OffsetFileName);
		ret =  -1;
	} else {
		filep->f_op->write(filep, pWorkInfo->buffer, pWorkInfo->len, &filep->f_pos);
		filp_close(filep, NULL);
		ret = 0;
	}

	set_fs(orgfs);
	pWorkInfo->rst = ret;
	complete(&pWorkInfo->completion);
}
/*----------------------------------------------------------------------------*/
static void sensor_read_work(struct work_struct *work)
{
	u32 orgfs;
	struct file *filep;
	int ret;
	struct work_info *pWorkInfo;

	orgfs = get_fs();
	set_fs(KERNEL_DS);

	pWorkInfo = container_of((struct delayed_work *)work, struct work_info,
				 read_work);
	/* For AllWinner android platform.
	 *
	 * mir3da.c - Linux kernel modules for 3-Axis Accelerometer
	 *
	 * Copyright (C) 2011-2013 MiraMEMS Sensing Technology Co., Ltd.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>

#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/of_gpio.h>

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon-vid.h>
#include <linux/sensor-dev.h>
#include "mir3da_core.h"
#include "mir3da_cust.h"

#define MIR3DA_DRV_NAME                 	"da380"
#define MIR3DA_INPUT_DEV_NAME     			MIR3DA_DRV_NAME
#define MIR3DA_MISC_NAME                	MIR3DA_DRV_NAME

#define POLL_INTERVAL_MAX               	500
//#define POLL_INTERVAL                   	50
#define INPUT_FUZZ                      	0
#define INPUT_FLAT                      	0

#define RANGE_2G				0
#define RANGE_4G				1
#define RANGE_8G				2
#define RANGE_16G				3

#define DA380_CHIP_ID_REG		0x01
#define DA380_CHIP_ID			0x13

#define GSENSOR_MIN				2
#define MIR3DA_PRECISION		11
#define MIR3DA_RANGE			1000000
#define DA311_BOUNDARY			(0x1 << (MIR3DA_PRECISION - 1))
#define DA311_GRAVITY_STEP		(MIR3DA_RANGE / DA311_BOUNDARY)

#define MIR3DA_INT1				1
#define MIR3DA_INT2				2

	static struct input_polled_dev      *mir3da_idev;
	static struct device 				*hwmon_dev;
	static MIR_HANDLE                   mir_handle;
	static unsigned int slope_th;
	static u32 active_dur = 0x01;
	static u32 active_th  = 0x10;
	static u32 range_g = RANGE_2G; //2g:0 4g:1 8g:2 16g:3
	static unsigned int                       delayMs = 50;
	static da380_config_t *da380_config_p;
	extern int                              	Log_level;

#define MI_DATA(format, ...)         	if(DEBUG_DATA&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
#define MI_MSG(format, ...)             if(DEBUG_MSG&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
#define MI_ERR(format, ...)             	if(DEBUG_ERR&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
#define MI_FUN                          	if(DEBUG_FUNC&Log_level){printk(KERN_ERR MI_TAG "%s is called, line: %d\n", __FUNCTION__,__LINE__);}
#define MI_ASSERT(expr)                 \
	if (!(expr)) {\
		printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n",\
			__FILE__, __LINE__, __func__, #expr);\
	}

	/*----------------------------------------------------------------------------*/
#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
	static char OffsetFileName[] = "/data/misc/miraGSensorOffset.txt";
#define OFFSET_STRING_LEN               26
	struct work_info {
		char        tst1[20];
		char        tst2[20];
		char        buffer[OFFSET_STRING_LEN];
		struct      workqueue_struct *wq;
		struct      delayed_work read_work;
		struct      delayed_work write_work;
		struct      completion completion;
		int         len;
		int         rst;
	};

	static struct work_info m_work_info = {{0}};
	/*----------------------------------------------------------------------------*/
	static void sensor_write_work(struct work_struct * work) {
		struct work_info   *pWorkInfo;
		struct file         *filep;
		u32                 orgfs;
		int                 ret;

		orgfs = get_fs();
		set_fs(KERNEL_DS);

		pWorkInfo = container_of((struct delayed_work *)work, struct work_info,
					 write_work);
		if (pWorkInfo == NULL) {
			MI_ERR("get pWorkInfo failed!");
			return;
		}

		filep = filp_open(OffsetFileName, O_RDWR | O_CREAT, 0600);
		if (IS_ERR(filep)) {
			MI_ERR("write, sys_open %s error!!.\n", OffsetFileName);
			ret =  -1;
		} else {
			filep->f_op->write(filep, pWorkInfo->buffer, pWorkInfo->len, &filep->f_pos);
			filp_close(filep, NULL);
			ret = 0;
		}

		set_fs(orgfs);
		pWorkInfo->rst = ret;
		complete(&pWorkInfo->completion);
	}
	/*----------------------------------------------------------------------------*/
	static void sensor_read_work(struct work_struct * work) {
		u32 orgfs;
		struct file *filep;
		int ret;
		struct work_info *pWorkInfo;

		orgfs = get_fs();
		set_fs(KERNEL_DS);

		pWorkInfo = container_of((struct delayed_work *)work, struct work_info,
					 read_work);
		if (pWorkInfo == NULL) {
			MI_ERR("get pWorkInfo failed!");
			return;
		}

		filep = filp_open(OffsetFileName, O_RDONLY, 0600);
		if (IS_ERR(filep)) {
			MI_MSG("read, sys_open %s error!!.\n", OffsetFileName);
			set_fs(orgfs);
			ret =  -1;
		} else {
			filep->f_op->read(filep, pWorkInfo->buffer,  sizeof(pWorkInfo->buffer),
					  &filep->f_pos);
			filp_close(filep, NULL);
			set_fs(orgfs);
			ret = 0;
		}

		pWorkInfo->rst = ret;
		complete(&(pWorkInfo->completion));
	}
	/*----------------------------------------------------------------------------*/
	static int sensor_sync_read(u8 * offset) {

		int     err;
		int     off[MIR3DA_OFFSET_LEN] = {0};
		struct work_info *pWorkInfo = &m_work_info;

		init_completion(&pWorkInfo->completion);
		queue_delayed_work(pWorkInfo->wq, &pWorkInfo->read_work, msecs_to_jiffies(0));
		err = wait_for_completion_timeout(&pWorkInfo->completion,
						  msecs_to_jiffies(2000));
		if (err == 0) {
			MI_ERR("wait_for_completion_timeout TIMEOUT");
			return -1;
		}

		if (pWorkInfo->rst != 0) {
			MI_ERR("work_info.rst  not equal 0");
			return pWorkInfo->rst;
		}

		sscanf(m_work_info.buffer, "%x,%x,%x,%x,%x,%x,%x,%x,%x", &off[0], &off[1],
		       &off[2], &off[3], &off[4], &off[5], &off[6], &off[7], &off[8]);

		offset[0] = (u8)off[0];
		offset[1] = (u8)off[1];
		offset[2] = (u8)off[2];
		offset[3] = (u8)off[3];
		offset[4] = (u8)off[4];
		offset[5] = (u8)off[5];
		offset[6] = (u8)off[6];
		offset[7] = (u8)off[7];
		offset[8] = (u8)off[8];

		return 0;
	}
	/*----------------------------------------------------------------------------*/
	static int sensor_sync_write(u8 * off) {

		int err = 0;
		struct work_info *pWorkInfo = &m_work_info;

		init_completion(&pWorkInfo->completion);

		sprintf(m_work_info.buffer, "%x,%x,%x,%x,%x,%x,%x,%x,%x\n", off[0], off[1],
			off[2], off[3], off[4], off[5], off[6], off[7], off[8]);

		pWorkInfo->len = sizeof(m_work_info.buffer);

		queue_delayed_work(pWorkInfo->wq, &pWorkInfo->write_work, msecs_to_jiffies(0));
		err = wait_for_completion_timeout(&pWorkInfo->completion,
						  msecs_to_jiffies(2000));
		if (err == 0) {
			MI_ERR("wait_for_completion_timeout TIMEOUT");
			return -1;
		}

		if (pWorkInfo->rst != 0) {
			MI_ERR("work_info.rst  not equal 0");
			return pWorkInfo->rst;
		}


		return 0;
	}
#endif
	/*----------------------------------------------------------------------------*/
#ifdef MIR3DA_AUTO_CALIBRAE
	static bool check_califile_exist(void) {
		u32     orgfs = 0;
		struct  file *filep;

		orgfs = get_fs();
		set_fs(KERNEL_DS);

		filep = filp_open(OffsetFileName, O_RDONLY, 0600);
		if (IS_ERR(filep)) {
			MI_MSG("%s read, sys_open %s error!!.\n", __func__, OffsetFileName);
			set_fs(orgfs);
			return false;
		}

		filp_close(filep, NULL);
		set_fs(orgfs);

		return true;
	}
#endif
	/*----------------------------------------------------------------------------*/
	static void report_abs(void) {
		short x = 0, y = 0, z = 0;
		MIR_HANDLE handle = mir_handle;
		printk("report_abs \n");
		if (mir3da_read_data(handle, &x, &y, &z) != 0) {
			MI_ERR("MIR3DA data read failed!\n");
			return;
		}
		// printk("mir3da_filt: x=%d, y=%d, z=%d\n",  x, y, z);
		input_report_abs(mir3da_idev->input, ABS_X, x);
		input_report_abs(mir3da_idev->input, ABS_Y, y);
		input_report_abs(mir3da_idev->input, ABS_Z, z);
		input_sync(mir3da_idev->input);
	}
	/*----------------------------------------------------------------------------*/
	static void mir3da_dev_poll(struct input_polled_dev * dev) {
		dev->poll_interval = delayMs;
		report_abs();
	}
	/*----------------------------------------------------------------------------*/
	static long mir3da_misc_ioctl(struct file * file, unsigned int cmd,
				      unsigned long arg) {
		void __user     *argp = (void __user *)arg;
		int             err = 0;
		int             interval = 0;
		char            bEnable = 0;
//    int             z_dir = 0;
//    int             range = 0;
		short           xyz[3] = {0};
		MIR_HANDLE      handle = mir_handle;

		if (_IOC_DIR(cmd) & _IOC_READ) {
			err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
		} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
			err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
		}

		if (err) {
			return -EFAULT;
		}

		switch (cmd) {
		case MIR3DA_ACC_IOCTL_GET_DELAY:
			interval = da380_config_p->poll_delay_ms;
			if (copy_to_user(argp, &interval, sizeof(interval)))
				return -EFAULT;
			break;

		case MIR3DA_ACC_IOCTL_SET_DELAY:
			if (copy_from_user(&interval, argp, sizeof(interval)))
				return -EFAULT;
			if (interval < 0 || interval > 1000)
				return -EINVAL;
			if ((interval <= 30) && (interval > 10)) {
				interval = 10;
			}
			delayMs = interval;
			break;

		case MIR3DA_ACC_IOCTL_SET_ENABLE:
			if (copy_from_user(&bEnable, argp, sizeof(bEnable)))
				return -EFAULT;

			err = mir3da_set_enable(handle, bEnable);
			if (err < 0)
				return EINVAL;
			break;

		case MIR3DA_ACC_IOCTL_GET_ENABLE:
			err = mir3da_get_enable(handle, &bEnable);
			if (err < 0) {
				return -EINVAL;
			}

			if (copy_to_user(argp, &bEnable, sizeof(bEnable)))
				return -EINVAL;
			break;

#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
		case MIR3DA_ACC_IOCTL_CALIBRATION:
			if (copy_from_user(&z_dir, argp, sizeof(z_dir)))
				return -EFAULT;

			if (mir3da_calibrate(handle, z_dir)) {
				return -EFAULT;
			}

			if (copy_to_user(argp, &z_dir, sizeof(z_dir)))
				return -EFAULT;
			break;

		case MIR3DA_ACC_IOCTL_UPDATE_OFFSET:
			manual_load_cali_file(handle);
			break;
#endif

		case MIR3DA_ACC_IOCTL_GET_COOR_XYZ:

			if (mir3da_read_data(handle, &xyz[0], &xyz[1], &xyz[2]))
				return -EFAULT;

			if (copy_to_user((void __user *)arg, xyz, sizeof(xyz)))
				return -EFAULT;
			break;

		default:
			return -EINVAL;
		}

		return 0;
	}

	/*----------------------------------------------------------------------------*/
	static const struct file_operations mir3da_misc_fops = {
		.owner = THIS_MODULE,
		.unlocked_ioctl = mir3da_misc_ioctl,
	};

	static struct miscdevice misc_mir3da = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = MIR3DA_MISC_NAME,
		.fops = &mir3da_misc_fops,
	};
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_enable_show(struct device * dev,
					  struct device_attribute * attr, char *buf) {
		int             ret;
		char            bEnable;
		MIR_HANDLE      handle = mir_handle;

		ret = mir3da_get_enable(handle, &bEnable);
		if (ret < 0) {
			ret = -EINVAL;
		} else {
			ret = sprintf(buf, "%d\n", bEnable);
		}

		return ret;
	}
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_enable_store(struct device * dev,
					   struct device_attribute * attr,
					   const char *buf, size_t count) {
		int             ret;
		char            bEnable;
		unsigned long   enable;
		MIR_HANDLE      handle = mir_handle;

		if (buf == NULL) {
			return -1;
		}

		enable = simple_strtoul(buf, NULL, 10);
		bEnable = (enable > 0) ? 1 : 0;

		ret = mir3da_set_enable(handle, bEnable);
		if (ret < 0) {
			ret = -EINVAL;
		} else {
			ret = count;
		}

		return ret;
	}
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_delay_show(struct device * dev,
					 struct device_attribute * attr, char *buf) {
		return sprintf(buf, "%d\n", delayMs);
	}
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_delay_store(struct device * dev,
					  struct device_attribute * attr, const char *buf, size_t count) {
		int interval = 0;

		interval = simple_strtoul(buf, NULL, 10);

		if (interval < 0 || interval > 1000)
			return -EINVAL;

		if ((interval <= 30) && (interval > 10)) {
			interval = 10;
		}

		delayMs = interval;

		return count;
	}
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_axis_data_show(struct device * dev,
					     struct device_attribute * attr, char *buf) {
		int result;
		short x, y, z;
		int count = 0;
		MIR_HANDLE      handle = mir_handle;

		result = mir3da_read_data(handle, &x, &y, &z);
		if (result == 0)
			count += sprintf(buf + count, "x= %d;y=%d;z=%d\n", x, y, z);
		else
			count += sprintf(buf + count, "reading failed!");

		return count;
	}
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_reg_data_store(struct device * dev,
					     struct device_attribute * attr, const char *buf, size_t count) {
		int                 addr, data;
		int                 result;
		MIR_HANDLE          handle = mir_handle;

		sscanf(buf, "0x%x, 0x%x\n", &addr, &data);

		result = mir3da_register_write(handle, addr, data);

		MI_ASSERT(result == 0);

		return count;
	}
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_reg_data_show(struct device * dev,
					    struct device_attribute * attr, char *buf) {
		MIR_HANDLE          handle = mir_handle;

		return mir3da_get_reg_data(handle, buf);
	}
	/*----------------------------------------------------------------------------*/
#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
	static ssize_t mir3da_offset_show(struct device * dev,
					  struct device_attribute * attr, char *buf) {
		ssize_t count = 0;
		int rst = 0;
		u8 off[9] = {0};
		MIR_HANDLE      handle = mir_handle;

		rst = mir3da_read_offset(handle, off);
		if (!rst) {
			count = sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n", off[0], off[1], off[2],
					off[3], off[4], off[5], off[6], off[7], off[8]);
		}
		return count;
	}
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_offset_store(struct device * dev,
					   struct device_attribute * attr,
					   const char *buf, size_t count) {
		int off[9] = {0};
		u8  offset[9] = {0};
		int rst = 0;
		MIR_HANDLE      handle = mir_handle;

		sscanf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n", &off[0], &off[1], &off[2], &off[3],
		       &off[4], &off[5], &off[6], &off[7], &off[8]);

		offset[0] = (u8)off[0];
		offset[1] = (u8)off[1];
		offset[2] = (u8)off[2];
		offset[3] = (u8)off[3];
		offset[4] = (u8)off[4];
		offset[5] = (u8)off[5];
		offset[6] = (u8)off[6];
		offset[7] = (u8)off[7];
		offset[8] = (u8)off[8];

		rst = mir3da_write_offset(handle, offset);
		return count;
	}
#endif

	/*----------------------------------------------------------------------------*/
#if FILTER_AVERAGE_ENHANCE
	static ssize_t mir3da_average_enhance_show(struct device * dev,
			struct device_attribute * attr, char *buf) {
		int                             ret = 0;
		struct mir3da_filter_param_s    param = {0};

		ret = mir3da_get_filter_param(&param);
		ret |= sprintf(buf, "%d %d %d\n", param.filter_param_l, param.filter_param_h,
			       param.filter_threhold);

		return ret;
	}
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_average_enhance_store(struct device * dev,
			struct device_attribute * attr, const char *buf, size_t count) {
		int                             ret = 0;
		struct mir3da_filter_param_s    param = {0};

		sscanf(buf, "%d %d %d\n", &param.filter_param_l, &param.filter_param_h,
		       &param.filter_threhold);

		ret = mir3da_set_filter_param(&param);

		return count;
	}
#endif
	/*----------------------------------------------------------------------------*/
#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
	int bCaliResult = -1;
	static ssize_t mir3da_calibrate_show(struct device * dev,
					     struct device_attribute * attr, char *buf) {
		int ret;

		ret = sprintf(buf, "%d\n", bCaliResult);
		return ret;
	}
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_calibrate_store(struct device * dev,
					      struct device_attribute * attr,
					      const char *buf, size_t count) {
		s8              z_dir = 0;
		MIR_HANDLE      handle = mir_handle;

		z_dir = simple_strtol(buf, NULL, 10);
		bCaliResult = mir3da_calibrate(handle, z_dir);

		return count;
	}
#endif
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_log_level_show(struct device * dev,
					     struct device_attribute * attr, char *buf) {
		int ret;

		ret = sprintf(buf, "%d\n", Log_level);

		return ret;
	}
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_log_level_store(struct device * dev,
					      struct device_attribute * attr,
					      const char *buf, size_t count) {
		Log_level = simple_strtoul(buf, NULL, 10);

		return count;
	}
	static int int2_enable = 0;
	static int int2_statu = 0;

	int da380_interrupt(void) {
		int num ;
		int res = 0;
		MIR_HANDLE      handle = mir_handle;

#ifdef CONFIG_DA280
		num = 1;
#else
		num = 0;
#endif

		MI_ERR("mir3da_int2_enable_store num:%d on:%d slope 0x%x active_dur:0x%02x active_th:0x%02x range_g:%d\n",
		       num, int2_enable, slope_th, active_dur, active_th, range_g);
		//res |=  mir3da_register_mask_write(handle,NSA_REG_INT_LATCH,0x8F,0x8F);
		res |=  mir3da_register_mask_write(handle, NSA_REG_INT_LATCH, 0x8F, slope_th);
		// 83 1s 84 2s 85 4s  86 8s  8f yiz
		res |=  mir3da_register_mask_write(handle, NSA_REG_ACTIVE_DURATION, 0xff,
						   active_dur);
		res |=  mir3da_register_mask_write(handle, NSA_REG_ACTIVE_THRESHOLD, 0xff,
						   active_th);
		mir3da_register_mask_write(handle, NSA_REG_G_RANGE, 0x03, range_g);

		if (int2_enable) {
			res |=  mir3da_register_mask_write(handle, NSA_REG_INTERRUPT_SETTINGS1, 0xff,
							   0x07);
			switch (num) {
			case 0:
				res |=  mir3da_register_mask_write(handle, NSA_REG_INTERRUPT_MAPPING1, 0xff,
								   0x04);
				break;

			case 1:
				res |=  mir3da_register_mask_write(handle, NSA_REG_INTERRUPT_MAPPING3, 0xff,
								   0x04);
				break;
			}
		} else {
			res |=  mir3da_register_mask_write(handle, NSA_REG_INTERRUPT_SETTINGS1, 0xff,
							   0x00);
			res |=  mir3da_register_mask_write(handle, NSA_REG_INTERRUPT_MAPPING1, 0xff,
							   0x00);
			res |=  mir3da_register_mask_write(handle, NSA_REG_INTERRUPT_MAPPING3, 0xff,
							   0x00);
		}

		return res;
	}

	EXPORT_SYMBOL(da380_interrupt);

	static ssize_t mir3da_int2_enable_store(struct device * dev,
						struct device_attribute * attr,
						const char *buf, size_t count) {

		int2_enable = simple_strtoul(buf, NULL, 10);

		return count;
	}

	static ssize_t mir3da_int2_enable_show(struct device * dev,
					       struct device_attribute * attr, char *buf) {
		int ret;

		ret = sprintf(buf, "%d\n", int2_enable);
		printk(" mir3da_int2_enable_show ret [ %d ]\n", ret);
		return ret;
	}

	static ssize_t mir3da_int2_clear_enable_store(struct device * dev,
			struct device_attribute * attr,
			const char *buf, size_t count) {

		MIR_HANDLE      handle = mir_handle;

		printk(" mir3da_int2_clear_enable_store int2_clean \n");
		mir3da_clear_intterrupt(handle);

		return count;
	}

	static ssize_t mir3da_int2_clear_enable_show(struct device * dev,
			struct device_attribute * attr, char *buf) {
		int ret = 0;

		//	ret = sprintf(buf, "%d\n", int2_enable);
		//	printk(" mir3da_int2_clear_enable_show ret [ %d ]\n",ret);
		return ret;
	}

	static ssize_t mir3da_int2_start_statu_store(struct device * dev,
			struct device_attribute * attr,
			const char *buf, size_t count) {

//	MIR_HANDLE      handle = mir_handle;

		int2_statu = simple_strtoul(buf, NULL, 10);

		return count;
	}

	static ssize_t mir3da_int2_start_statu_show(struct device * dev,
			struct device_attribute * attr, char *buf) {
		int ret;
//		 MIR_HANDLE      handle = mir_handle;

		//	int2_statu =  mir3da_read_int_status( handle);

		ret = sprintf(buf, "%d\n", int2_statu);
		printk(" mir3da_int2_enable_show ret [ %d ]\n", ret);
		return ret;
	}
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_primary_offset_show(struct device * dev,
			struct device_attribute * attr, char *buf) {
		MIR_HANDLE   handle = mir_handle;
		int x = 0, y = 0, z = 0;

		mir3da_get_primary_offset(handle, &x, &y, &z);

		return sprintf(buf, "x=%d ,y=%d ,z=%d\n", x, y, z);

	}
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_version_show(struct device * dev,
					   struct device_attribute * attr, char *buf) {

		return sprintf(buf, "%s_%s\n", DRI_VER, CORE_VER);
	}
	/*----------------------------------------------------------------------------*/
	static ssize_t mir3da_vendor_show(struct device * dev,
					  struct device_attribute * attr, char *buf) {
		return sprintf(buf, "%s\n", "MiraMEMS");
	}

	static ssize_t mir3da_slope_th_show(struct device * dev,
					    struct device_attribute * attr, char *buf) {
		return sprintf(buf, "0x%x\n", slope_th);
	}

	static ssize_t mir3da_slope_th_store(struct device * dev,
					     struct device_attribute * attr,
					     const char *buf, size_t count) {
		unsigned long data;
		int error = 0;
		error = strict_strtoul(buf, 16, &data);
		if (error)
			return error;
		//printk("%s:data=0x%02x\n",__func__,data);
		if (data == 0x3) { //high
			data = 0x85;
			active_dur = 0x02;
			active_th = 0x0a;
			range_g = RANGE_2G;
		} else if (data == 0xf) { //low
			data = 0x86;
			active_dur = 0x03;
			active_th = 0x30;
			range_g = RANGE_8G;
		} else if (data == 0x5) { //mid
			data = 0x85;
			active_dur = 0x02;
			active_th = 0x15;
			range_g = RANGE_4G;
		}
		printk("set slope 0x%lx\n", data);
		slope_th = data;
		return count;
	}

	/*----------------------------------------------------------------------------*/
	static DEVICE_ATTR(enable,          S_IRUGO | S_IWUGO,  mir3da_enable_show,
			   mir3da_enable_store);
	static DEVICE_ATTR(delay,      S_IRUGO | S_IWUGO,  mir3da_delay_show,
			   mir3da_delay_store);
	static DEVICE_ATTR(axis_data,       S_IRUGO | S_IWUGO,    mir3da_axis_data_show,
			   NULL);
	static DEVICE_ATTR(reg_data,        S_IWUGO | S_IRUGO,  mir3da_reg_data_show,
			   mir3da_reg_data_store);
	static DEVICE_ATTR(log_level,       S_IWUGO | S_IRUGO,  mir3da_log_level_show,
			   mir3da_log_level_store);
#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
	static DEVICE_ATTR(offset,          S_IWUGO | S_IRUGO,  mir3da_offset_show,
			   mir3da_offset_store);
	static DEVICE_ATTR(calibrate_miraGSensor,       S_IWUGO | S_IRUGO,
			   mir3da_calibrate_show,          mir3da_calibrate_store);
#endif
#ifdef FILTER_AVERAGE_ENHANCE
	static DEVICE_ATTR(average_enhance, S_IWUGO | S_IRUGO,
			   mir3da_average_enhance_show,    mir3da_average_enhance_store);
#endif
// aad cz
	static DEVICE_ATTR(int2_enable,     S_IRUGO | S_IWUGO,  mir3da_int2_enable_show,
			   mir3da_int2_enable_store);
	static DEVICE_ATTR(int2_clear,     S_IRUGO | S_IWUGO,
			   mir3da_int2_clear_enable_show  ,             mir3da_int2_clear_enable_store);
	static DEVICE_ATTR(int2_start_status,     S_IRUGO | S_IWUGO,
			   mir3da_int2_start_statu_show  ,             mir3da_int2_start_statu_store);

	static DEVICE_ATTR(primary_offset,  S_IWUGO,
			   mir3da_primary_offset_show,            NULL);
	static DEVICE_ATTR(version,         S_IRUGO,            mir3da_version_show,
			   NULL);
	static DEVICE_ATTR(vendor,          S_IRUGO,            mir3da_vendor_show,
			   NULL);
	static DEVICE_ATTR(slope_th, S_IRUGO | S_IWUGO, mir3da_slope_th_show,
			   mir3da_slope_th_store);

	/*----------------------------------------------------------------------------*/
	static struct attribute *mir3da_attributes[] = {
		&dev_attr_enable.attr,
		&dev_attr_delay.attr,
		&dev_attr_axis_data.attr,
		&dev_attr_reg_data.attr,
		&dev_attr_log_level.attr,
#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
		&dev_attr_offset.attr,
		&dev_attr_calibrate_miraGSensor.attr,
//	&dev_attr_primary_offset.attr,
#endif
#ifdef FILTER_AVERAGE_ENHANCE
		&dev_attr_average_enhance.attr,
#endif /* ! FILTER_AVERAGE_ENHANCE */
		&dev_attr_int2_enable.attr,
		&dev_attr_int2_clear.attr,
		&dev_attr_int2_start_status.attr,
		&dev_attr_primary_offset.attr,
		&dev_attr_version.attr,
		&dev_attr_vendor.attr,
		&dev_attr_slope_th.attr,
		NULL
	};

	static const struct attribute_group mir3da_attr_group = {
		.attrs  = mir3da_attributes,
	};
	/*----------------------------------------------------------------------------*/
	int i2c_smbus_read(PLAT_HANDLE handle, u8 addr, u8 * data) {
		int                 res = 0;
		struct i2c_client   *client = (struct i2c_client *)handle;

		*data = i2c_smbus_read_byte_data(client, addr);

		return res;
	}
	/*----------------------------------------------------------------------------*/
	int i2c_smbus_read_block(PLAT_HANDLE handle, u8 addr, u8 count, u8 * data) {
		int                 res = 0;
		struct i2c_client   *client = (struct i2c_client *)handle;

		res = i2c_smbus_read_i2c_block_data(client, addr, count, data);

		return res;
	}
	/*----------------------------------------------------------------------------*/
	int i2c_smbus_write(PLAT_HANDLE handle, u8 addr, u8 data) {
		int                 res = 0;
		struct i2c_client   *client = (struct i2c_client *)handle;

		res = i2c_smbus_write_byte_data(client, addr, data);

		return res;
	}
	/*----------------------------------------------------------------------------*/
	void msdelay(int ms) {
		mdelay(ms);
	}

#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
	MIR_GENERAL_OPS_DECLARE(ops_handle, i2c_smbus_read, i2c_smbus_read_block,
				i2c_smbus_write, sensor_sync_write, sensor_sync_read, msdelay, printk, sprintf);
#else
	MIR_GENERAL_OPS_DECLARE(ops_handle, i2c_smbus_read, i2c_smbus_read_block,
				i2c_smbus_write, NULL, NULL, msdelay, printk, sprintf);
#endif

	/*----------------------------------------------------------------------------*/
	static int mir3da_probe(struct i2c_client * client,
				const struct i2c_device_id * id) {
		int                 result = 0;
		struct input_dev    *idev;
		struct device_node *np = client->dev.of_node;

		if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
			MI_ERR("da380 I2C check functionality failed.");
			return -ENODEV;
		}

		if (!np) {
			MI_ERR("no device tree\n");
			return -EINVAL;
		}

		if (mir3da_install_general_ops(&ops_handle)) {
			MI_ERR("Install ops failed !\n");
			goto err_detach_client;
		}

#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
		m_work_info.wq = create_singlethread_workqueue("oo");
		if (NULL == m_work_info.wq) {
			MI_ERR("Failed to create workqueue !");
			goto err_detach_client;
		}

		INIT_DELAYED_WORK(&m_work_info.read_work, sensor_read_work);
		INIT_DELAYED_WORK(&m_work_info.write_work, sensor_write_work);
#endif

		int2_statu =  mir3da_read_int_status((PLAT_HANDLE)client);
		//printk("ParkMonitor powerOn status is %d\n", int2_statu);

		/* Initialize the MIR3DA chip */
		mir_handle = mir3da_core_init((PLAT_HANDLE)client);
		if (NULL == mir_handle) {
			MI_ERR("chip init failed !\n");
			goto err_detach_client;
		}

		hwmon_dev = hwmon_device_register(&client->dev);
		MI_ASSERT(!(IS_ERR(hwmon_dev)));

		/* input poll device register */
		mir3da_idev = input_allocate_polled_device();
		if (!mir3da_idev) {
			MI_ERR("alloc poll device failed!\n");
			result = -ENOMEM;
			goto err_hwmon_device_unregister;
		}
		mir3da_idev->poll = mir3da_dev_poll;
		mir3da_idev->poll_interval = da380_config_p->poll_delay_ms;
		delayMs = da380_config_p->poll_delay_ms;
		mir3da_idev->poll_interval_max = POLL_INTERVAL_MAX;
		idev = mir3da_idev->input;

		idev->name = MIR3DA_INPUT_DEV_NAME;
		idev->id.bustype = BUS_I2C;
		idev->evbit[0] = BIT_MASK(EV_ABS);

		input_set_abs_params(idev, ABS_X, -16384, 16383, INPUT_FUZZ, INPUT_FLAT);
		input_set_abs_params(idev, ABS_Y, -16384, 16383, INPUT_FUZZ, INPUT_FLAT);
		input_set_abs_params(idev, ABS_Z, -16384, 16383, INPUT_FUZZ, INPUT_FLAT);

		result = input_register_polled_device(mir3da_idev);
		if (result) {
			MI_ERR("register poll device failed!\n");
			goto err_free_polled_device;
		}

		/* Sys Attribute Register */
		result = sysfs_create_group(&idev->dev.kobj, &mir3da_attr_group);
		if (result) {
			MI_ERR("create device file failed!\n");
			result = -EINVAL;
			goto err_unregister_polled_device;
		}

		/* Misc device interface Register */
		result = misc_register(&misc_mir3da);
		if (result) {
			MI_ERR("%s: mir3da_dev register failed", __func__);
			goto err_remove_sysfs_group;
		}
		slope_th = 0x86;
		return result;

err_remove_sysfs_group:
		sysfs_remove_group(&idev->dev.kobj, &mir3da_attr_group);
err_unregister_polled_device:
		input_unregister_polled_device(mir3da_idev);
err_free_polled_device:
		input_free_polled_device(mir3da_idev);
err_hwmon_device_unregister:
		hwmon_device_unregister(&client->dev);
err_detach_client:
		return result;
	}
	/*----------------------------------------------------------------------------*/
	static int mir3da_remove(struct i2c_client * client) {
		MIR_HANDLE      handle = mir_handle;

		mir3da_set_enable(handle, 0);

		misc_deregister(&misc_mir3da);

		sysfs_remove_group(&mir3da_idev->input->dev.kobj, &mir3da_attr_group);

		input_unregister_polled_device(mir3da_idev);

		input_free_polled_device(mir3da_idev);

#ifdef  MIR3DA_OFFSET_TEMP_SOLUTION
		flush_workqueue(m_work_info.wq);
		destroy_workqueue(m_work_info.wq);
#endif

		hwmon_device_unregister(hwmon_dev);

		kfree(da380_config_p);

		return 0;
	}
	/*----------------------------------------------------------------------------*/
#ifdef CONFIG_HAS_EARLYSUSPEND
	static int mir3da_suspend(struct i2c_client * client, pm_message_t mesg) {
		return 0;
#if 0
		int result = 0;
		MIR_HANDLE      handle = mir_handle;

		MI_FUN;

		result = mir3da_set_enable(handle, 0);
		if (result) {
			MI_ERR("%s:set disable fail!!\n", __func__);
			return;
		}
		mir3da_idev->input->close(mir3da_idev->input);

		return result;
#endif
	}
	/*----------------------------------------------------------------------------*/
	static int mir3da_resume(struct i2c_client * client) {
		int result = 0;
		MIR_HANDLE      handle = mir_handle;

		MI_FUN;

		result = mir3da_chip_resume(handle);
		if (result) {
			MI_ERR("%s:chip resume fail!!\n", __func__);
			return result;
		}

		result = mir3da_set_enable(handle, 1);
		if (result) {
			MI_ERR("%s:set enable fail!!\n", __func__);
			return result;
		}

		mir3da_idev->input->open(mir3da_idev->input);

		return result;
	}
#endif

	static const struct i2c_device_id mir3da_id[] = {
		{ MIR3DA_DRV_NAME, 0 },
		{ }
	};

	static struct of_device_id mir3da_dt_ids[] = {
		{ .compatible = "mir3da,da380" },
		{ }
	};

	static struct i2c_driver mir3da_driver = {
		.probe      = mir3da_probe,
		.remove     = mir3da_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
		.suspend = mir3da_suspend,
		.resume = mir3da_resume,
#endif
		.id_table   = mir3da_id,
		.driver = {
			.name     = MIR3DA_DRV_NAME,
			.owner    = THIS_MODULE,
			.of_match_table = of_match_ptr(mir3da_dt_ids),
		},
	};
#if 0
	static int da380_interrupt_test(void) {
		int result = 0;
		MIR_HANDLE handle = mir_handle;

		mdelay(10);
		result = mir3da_set_enable(handle, true);
		if (result) {
			printk("sensor_active enable fail!!\n");
			return result;
		}
		mdelay(10);

		mir3da_open_interrupt(handle, 1, 1);

	}
#endif
	static int sensor_init(struct i2c_client * client) {
		int	result = 0;
		struct device_node *np = client->dev.of_node;

		mir3da_driver.id_table = 0x00;

		if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
			MI_ERR("da380 I2C check functionality failed.");
			return -ENODEV;
		}

		if (!np) {
			MI_ERR("no device tree\n");
			return -EINVAL;
		}

		if (mir3da_install_general_ops(&ops_handle)) {
			MI_ERR("Install ops failed !\n");
			goto err_detach_client;
		}

		/* Initialize the MIR3DA chip */
		mir_handle = mir3da_core_init((PLAT_HANDLE)client);
		if (NULL == mir_handle) {
			MI_ERR("chip init failed !\n");
			goto err_detach_client;
		}

		slope_th = 0x86;

		/* wrm test */
		//da380_interrupt_test();

		return result;

err_detach_client:
		return result;
	}

	static int sensor_active(struct i2c_client * client, int enable, int rate) {
		int result = 0;
		MIR_HANDLE handle = mir_handle;

		mdelay(10);
		if (enable) {
			result = mir3da_set_enable(handle, true);
			if (result) {
				printk("sensor_active enable  fail!!\n");
				return result;
			}
		} else {
			result = mir3da_set_enable(handle, false);
			if (result) {
				printk("sensor_active disable  fail!!\n");
				return result;
			}
		}
		mdelay(10);

		return result;
	}

	static int sensor_report_value(struct i2c_client * client) {
		struct sensor_private_data *sensor = (struct sensor_private_data *)
						     i2c_get_clientdata(client);
		struct sensor_platform_data *pdata = sensor->pdata;
		struct sensor_axis axis;
		int tmp_x = 0, tmp_y = 0, tmp_z = 0;
		short x = 0, y = 0, z = 0;
		int ret = 0;

		ret = mir3da_read_data(mir_handle, &x, &y, &z);
		if (ret) {
			MI_ERR("read data failed!");
			return ret;
		}

		//printk(KERN_DEBUG " x = %d, y = %d, z = %d\n", x, y, z);
		//printk("raw: x = %d, y = %d, z = %d\n", x, y, z);

		tmp_x = x * DA311_GRAVITY_STEP;
		tmp_y = y * DA311_GRAVITY_STEP;
		tmp_z = z * DA311_GRAVITY_STEP;

		//printk(KERN_DEBUG " tmp_x = %d, tmp_y = %d, tmp_z = %d\n", tmp_x, tmp_y, tmp_z);
		//printk("gai: tmp_x = %d, tmp_y = %d, tmp_z = %d\n", tmp_x, tmp_y, tmp_z);
		axis.x = (pdata->orientation[0]) * tmp_x + (pdata->orientation[1]) * tmp_y +
			 (pdata->orientation[2]) * tmp_z;
		axis.y = (pdata->orientation[3]) * tmp_x + (pdata->orientation[4]) * tmp_y +
			 (pdata->orientation[5]) * tmp_z;
#if MIR3DA_STK_TEMP_SOLUTION
		axis.z = (pdata->orientation[6]) * tmp_x + (pdata->orientation[7]) * tmp_y +
			 (bzstk ? 1 : (pdata->orientation[8])) * tmp_z;
#else
		axis.z = (pdata->orientation[6]) * tmp_x + (pdata->orientation[7]) * tmp_y +
			 (pdata->orientation[8]) * tmp_z;
#endif
		//printk( "map: axis = %d  %d  %d \n", axis.x, axis.y, axis.z);

		if ((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) ||
		    (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) ||
		    (abs(sensor->axis.z - axis.z) > GSENSOR_MIN)) {
			input_report_abs(sensor->input_dev, ABS_X, axis.x);
			input_report_abs(sensor->input_dev, ABS_Y, axis.y);
			input_report_abs(sensor->input_dev, ABS_Z, axis.z);
			input_sync(sensor->input_dev);

			mutex_lock(&(sensor->data_mutex));
			sensor->axis = axis;
			mutex_unlock(&(sensor->data_mutex));
		}

		return ret;
	}

	int sensor_use_interrupt(struct i2c_client * client, int num, int enable) {
		MIR_HANDLE      handle;
		u8	rvalue;

		handle = mir_handle;

		printk("sensor_use_interrupt num=%d enable=%d\n", num - 1, enable);
		mir3da_open_interrupt(handle, num - 1, enable);
		mdelay(10);
		if (mir3da_register_read(handle, INT_CONFIG, &rvalue) != 0) {
			printk("ERROR: INT_CONFIG = 0x%x\n", rvalue);
		} else {
			printk("0: INT_CONFIG = 0x%x\n", rvalue);
		}
#if 0

		rvalue = 0x03;
		mir3da_register_write(handle, INT_CONFIG, rvalue);

		if (mir3da_register_read(handle, INT_CONFIG, &rvalue) != 0) {
			printk("ERROR: INT_CONFIG = 0x%x\n", rvalue);
		} else {
			printk("1: INT_CONFIG = 0x%x\n", rvalue);
		}
#endif
		return 0;
	}

	struct sensor_operate mirgsensor_ops = {
		.name               = "mir3da",
		.type               = SENSOR_TYPE_ACCEL,            /*sensor type and it should be correct*/
		.id_i2c             = ACCEL_ID_MIR3DA,              /*i2c id number*/
		.read_reg           = -1,        					/*read data*/
		.read_len           = 0,                            /*data length*/
		.id_reg             = DA380_CHIP_ID_REG,   			/* read device id from this register */
		.id_data            = DA380_CHIP_ID,   				/* device id */
		.precision          = MIR3DA_PRECISION,           	/*12 bit*/
		.ctrl_reg           = -1,							/*enable or disable*/
		/*intterupt status register*/
		.int_status_reg     = 0x00,
		.range              = { -MIR3DA_RANGE, MIR3DA_RANGE},	/*range*/
		.trig               = IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
		.active             = sensor_active,
		.init               = sensor_init,
		.report             = sensor_report_value,
		.interrupt_use      = sensor_use_interrupt,
		//.suspend  			= mir3da_suspend,
		//.resume   			= mir3da_resume,
	};

	static struct sensor_operate *gsensor_get_ops(void) {
		return &mirgsensor_ops;
	}

	static int __init mir3da_init(void) {
		struct sensor_operate *ops = gsensor_get_ops();
		int result = 0;
		int type = ops->type;

		result = sensor_register_slave(type, NULL, NULL, gsensor_get_ops);

		return result;
	}

	static void __exit mir3da_exit(void) {
		struct sensor_operate *ops = gsensor_get_ops();
		int type = ops->type;

		sensor_unregister_slave(type, NULL, NULL, gsensor_get_ops);
	}

	module_init(mir3da_init);
	module_exit(mir3da_exit);


#if 0
	/*----------------------------------------------------------------------------*/
	static int __init mir3da_init(void) {
		int res;

		MI_FUN;
		printk("step4 : mir3da_init\n");
		res = i2c_add_driver(&mir3da_driver);
		if (res < 0) {
			MI_ERR("add mir3da i2c driver failed\n");
			return -ENODEV;
		}

		return (res);
	}
	/*----------------------------------------------------------------------------*/
	static void __exit mir3da_exit(void) {
		MI_FUN;

		i2c_del_driver(&mir3da_driver);
	}
	/*----------------------------------------------------------------------------*/
	MODULE_AUTHOR("MiraMEMS <lschen@miramems.com>");
	MODULE_DESCRIPTION("MIR3DA 3-Axis Accelerometer driver");
	MODULE_LICENSE("GPL");
	MODULE_VERSION("1.0");

	arch_initcall(mir3da_init);
//module_init(mir3da_init);
	module_exit(mir3da_exit);
#endif
	if (pWorkInfo == NULL) {
		MI_ERR("get pWorkInfo failed!");
		return;
	}

	filep = filp_open(OffsetFileName, O_RDONLY, 0600);
	if (IS_ERR(filep)) {
		MI_MSG("read, sys_open %s error!!.\n", OffsetFileName);
		set_fs(orgfs);
		ret =  -1;
	} else {
		filep->f_op->read(filep, pWorkInfo->buffer,  sizeof(pWorkInfo->buffer),
				  &filep->f_pos);
		filp_close(filep, NULL);
		set_fs(orgfs);
		ret = 0;
	}

	pWorkInfo->rst = ret;
	complete(&(pWorkInfo->completion));
}
/*----------------------------------------------------------------------------*/
static int sensor_sync_read(u8 *offset)
{

	int     err;
	int     off[MIR3DA_OFFSET_LEN] = {0};
	struct work_info *pWorkInfo = &m_work_info;

	init_completion(&pWorkInfo->completion);
	queue_delayed_work(pWorkInfo->wq, &pWorkInfo->read_work, msecs_to_jiffies(0));
	err = wait_for_completion_timeout(&pWorkInfo->completion,
					  msecs_to_jiffies(2000));
	if (err == 0) {
		MI_ERR("wait_for_completion_timeout TIMEOUT");
		return -1;
	}

	if (pWorkInfo->rst != 0) {
		MI_ERR("work_info.rst  not equal 0");
		return pWorkInfo->rst;
	}

	sscanf(m_work_info.buffer, "%x,%x,%x,%x,%x,%x,%x,%x,%x", &off[0], &off[1],
	       &off[2], &off[3], &off[4], &off[5], &off[6], &off[7], &off[8]);

	offset[0] = (u8)off[0];
	offset[1] = (u8)off[1];
	offset[2] = (u8)off[2];
	offset[3] = (u8)off[3];
	offset[4] = (u8)off[4];
	offset[5] = (u8)off[5];
	offset[6] = (u8)off[6];
	offset[7] = (u8)off[7];
	offset[8] = (u8)off[8];

	return 0;
}
/*----------------------------------------------------------------------------*/
static int sensor_sync_write(u8 *off)
{

	int err = 0;
	struct work_info *pWorkInfo = &m_work_info;

	init_completion(&pWorkInfo->completion);

	sprintf(m_work_info.buffer, "%x,%x,%x,%x,%x,%x,%x,%x,%x\n", off[0], off[1],
		off[2], off[3], off[4], off[5], off[6], off[7], off[8]);

	pWorkInfo->len = sizeof(m_work_info.buffer);

	queue_delayed_work(pWorkInfo->wq, &pWorkInfo->write_work, msecs_to_jiffies(0));
	err = wait_for_completion_timeout(&pWorkInfo->completion,
					  msecs_to_jiffies(2000));
	if (err == 0) {
		MI_ERR("wait_for_completion_timeout TIMEOUT");
		return -1;
	}

	if (pWorkInfo->rst != 0) {
		MI_ERR("work_info.rst  not equal 0");
		return pWorkInfo->rst;
	}


	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
#ifdef MIR3DA_AUTO_CALIBRAE
static bool check_califile_exist(void)
{
	u32     orgfs = 0;
	struct  file *filep;

	orgfs = get_fs();
	set_fs(KERNEL_DS);

	filep = filp_open(OffsetFileName, O_RDONLY, 0600);
	if (IS_ERR(filep)) {
		MI_MSG("%s read, sys_open %s error!!.\n", __func__, OffsetFileName);
		set_fs(orgfs);
		return false;
	}

	filp_close(filep, NULL);
	set_fs(orgfs);

	return true;
}
#endif
/*----------------------------------------------------------------------------*/
static void report_abs(void)
{
	short x = 0, y = 0, z = 0;
	MIR_HANDLE handle = mir_handle;
	printk("report_abs \n");
	if (mir3da_read_data(handle, &x, &y, &z) != 0) {
		MI_ERR("MIR3DA data read failed!\n");
		return;
	}
	// printk("mir3da_filt: x=%d, y=%d, z=%d\n",  x, y, z);
	input_report_abs(mir3da_idev->input, ABS_X, x);
	input_report_abs(mir3da_idev->input, ABS_Y, y);
	input_report_abs(mir3da_idev->input, ABS_Z, z);
	input_sync(mir3da_idev->input);
}
/*----------------------------------------------------------------------------*/
static void mir3da_dev_poll(struct input_polled_dev *dev)
{
	dev->poll_interval = delayMs;
	report_abs();
}
/*----------------------------------------------------------------------------*/
static long mir3da_misc_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	void __user     *argp = (void __user *)arg;
	int             err = 0;
	int             interval = 0;
	char            bEnable = 0;
//    int             z_dir = 0;
//    int             range = 0;
	short           xyz[3] = {0};
	MIR_HANDLE      handle = mir_handle;

	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if (err) {
		return -EFAULT;
	}

	switch (cmd) {
	case MIR3DA_ACC_IOCTL_GET_DELAY:
		interval = da380_config_p->poll_delay_ms;
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EFAULT;
		break;

	case MIR3DA_ACC_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval < 0 || interval > 1000)
			return -EINVAL;
		if ((interval <= 30) && (interval > 10)) {
			interval = 10;
		}
		delayMs = interval;
		break;

	case MIR3DA_ACC_IOCTL_SET_ENABLE:
		if (copy_from_user(&bEnable, argp, sizeof(bEnable)))
			return -EFAULT;

		err = mir3da_set_enable(handle, bEnable);
		if (err < 0)
			return EINVAL;
		break;

	case MIR3DA_ACC_IOCTL_GET_ENABLE:
		err = mir3da_get_enable(handle, &bEnable);
		if (err < 0) {
			return -EINVAL;
		}

		if (copy_to_user(argp, &bEnable, sizeof(bEnable)))
			return -EINVAL;
		break;

#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
	case MIR3DA_ACC_IOCTL_CALIBRATION:
		if (copy_from_user(&z_dir, argp, sizeof(z_dir)))
			return -EFAULT;

		if (mir3da_calibrate(handle, z_dir)) {
			return -EFAULT;
		}

		if (copy_to_user(argp, &z_dir, sizeof(z_dir)))
			return -EFAULT;
		break;

	case MIR3DA_ACC_IOCTL_UPDATE_OFFSET:
		manual_load_cali_file(handle);
		break;
#endif

	case MIR3DA_ACC_IOCTL_GET_COOR_XYZ:

		if (mir3da_read_data(handle, &xyz[0], &xyz[1], &xyz[2]))
			return -EFAULT;

		if (copy_to_user((void __user *)arg, xyz, sizeof(xyz)))
			return -EFAULT;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static const struct file_operations mir3da_misc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = mir3da_misc_ioctl,
};

static struct miscdevice misc_mir3da = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MIR3DA_MISC_NAME,
	.fops = &mir3da_misc_fops,
};
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int             ret;
	char            bEnable;
	MIR_HANDLE      handle = mir_handle;

	ret = mir3da_get_enable(handle, &bEnable);
	if (ret < 0) {
		ret = -EINVAL;
	} else {
		ret = sprintf(buf, "%d\n", bEnable);
	}

	return ret;
}
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int             ret;
	char            bEnable;
	unsigned long   enable;
	MIR_HANDLE      handle = mir_handle;

	if (buf == NULL) {
		return -1;
	}

	enable = simple_strtoul(buf, NULL, 10);
	bEnable = (enable > 0) ? 1 : 0;

	ret = mir3da_set_enable(handle, bEnable);
	if (ret < 0) {
		ret = -EINVAL;
	} else {
		ret = count;
	}

	return ret;
}
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_delay_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", delayMs);
}
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_delay_store(struct device *dev,
				  struct device_attribute *attr, const char *buf, size_t count)
{
	int interval = 0;

	interval = simple_strtoul(buf, NULL, 10);

	if (interval < 0 || interval > 1000)
		return -EINVAL;

	if ((interval <= 30) && (interval > 10)) {
		interval = 10;
	}

	delayMs = interval;

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_axis_data_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int result;
	short x, y, z;
	int count = 0;
	MIR_HANDLE      handle = mir_handle;

	result = mir3da_read_data(handle, &x, &y, &z);
	if (result == 0)
		count += sprintf(buf + count, "x= %d;y=%d;z=%d\n", x, y, z);
	else
		count += sprintf(buf + count, "reading failed!");

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_reg_data_store(struct device *dev,
				     struct device_attribute *attr, const char *buf, size_t count)
{
	int                 addr, data;
	int                 result;
	MIR_HANDLE          handle = mir_handle;

	sscanf(buf, "0x%x, 0x%x\n", &addr, &data);

	result = mir3da_register_write(handle, addr, data);

	MI_ASSERT(result == 0);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_reg_data_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	MIR_HANDLE          handle = mir_handle;

	return mir3da_get_reg_data(handle, buf);
}
/*----------------------------------------------------------------------------*/
#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
static ssize_t mir3da_offset_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	int rst = 0;
	u8 off[9] = {0};
	MIR_HANDLE      handle = mir_handle;

	rst = mir3da_read_offset(handle, off);
	if (!rst) {
		count = sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n", off[0], off[1], off[2],
				off[3], off[4], off[5], off[6], off[7], off[8]);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_offset_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int off[9] = {0};
	u8  offset[9] = {0};
	int rst = 0;
	MIR_HANDLE      handle = mir_handle;

	sscanf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n", &off[0], &off[1], &off[2], &off[3],
	       &off[4], &off[5], &off[6], &off[7], &off[8]);

	offset[0] = (u8)off[0];
	offset[1] = (u8)off[1];
	offset[2] = (u8)off[2];
	offset[3] = (u8)off[3];
	offset[4] = (u8)off[4];
	offset[5] = (u8)off[5];
	offset[6] = (u8)off[6];
	offset[7] = (u8)off[7];
	offset[8] = (u8)off[8];

	rst = mir3da_write_offset(handle, offset);
	return count;
}
#endif

/*----------------------------------------------------------------------------*/
#if FILTER_AVERAGE_ENHANCE
static ssize_t mir3da_average_enhance_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int                             ret = 0;
	struct mir3da_filter_param_s    param = {0};

	ret = mir3da_get_filter_param(&param);
	ret |= sprintf(buf, "%d %d %d\n", param.filter_param_l, param.filter_param_h,
		       param.filter_threhold);

	return ret;
}
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_average_enhance_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int                             ret = 0;
	struct mir3da_filter_param_s    param = {0};

	sscanf(buf, "%d %d %d\n", &param.filter_param_l, &param.filter_param_h,
	       &param.filter_threhold);

	ret = mir3da_set_filter_param(&param);

	return count;
}
#endif
/*----------------------------------------------------------------------------*/
#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
int bCaliResult = -1;
static ssize_t mir3da_calibrate_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "%d\n", bCaliResult);
	return ret;
}
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_calibrate_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	s8              z_dir = 0;
	MIR_HANDLE      handle = mir_handle;

	z_dir = simple_strtol(buf, NULL, 10);
	bCaliResult = mir3da_calibrate(handle, z_dir);

	return count;
}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_log_level_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "%d\n", Log_level);

	return ret;
}
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_log_level_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	Log_level = simple_strtoul(buf, NULL, 10);

	return count;
}
static int int2_enable = 0;
static int int2_statu = 0;

int da380_interrupt(void)
{
	int num ;
	int res = 0;
	MIR_HANDLE      handle = mir_handle;

#ifdef CONFIG_DA280
	num = 1;
#else
	num = 0;
#endif

	MI_ERR("mir3da_int2_enable_store num:%d on:%d slope 0x%x active_dur:0x%02x active_th:0x%02x range_g:%d\n",
	       num, int2_enable, slope_th, active_dur, active_th, range_g);
	//res |=  mir3da_register_mask_write(handle,NSA_REG_INT_LATCH,0x8F,0x8F);
	res |=  mir3da_register_mask_write(handle, NSA_REG_INT_LATCH, 0x8F, slope_th);
	// 83 1s 84 2s 85 4s  86 8s  8f yiz
	res |=  mir3da_register_mask_write(handle, NSA_REG_ACTIVE_DURATION, 0xff,
					   active_dur);
	res |=  mir3da_register_mask_write(handle, NSA_REG_ACTIVE_THRESHOLD, 0xff,
					   active_th);
	mir3da_register_mask_write(handle, NSA_REG_G_RANGE, 0x03, range_g);

	if (int2_enable) {
		res |=  mir3da_register_mask_write(handle, NSA_REG_INTERRUPT_SETTINGS1, 0xff,
						   0x07);
		switch (num) {
		case 0:
			res |=  mir3da_register_mask_write(handle, NSA_REG_INTERRUPT_MAPPING1, 0xff,
							   0x04);
			break;

		case 1:
			res |=  mir3da_register_mask_write(handle, NSA_REG_INTERRUPT_MAPPING3, 0xff,
							   0x04);
			break;
		}
	} else {
		res |=  mir3da_register_mask_write(handle, NSA_REG_INTERRUPT_SETTINGS1, 0xff,
						   0x00);
		res |=  mir3da_register_mask_write(handle, NSA_REG_INTERRUPT_MAPPING1, 0xff,
						   0x00);
		res |=  mir3da_register_mask_write(handle, NSA_REG_INTERRUPT_MAPPING3, 0xff,
						   0x00);
	}

	return res;
}

EXPORT_SYMBOL(da380_interrupt);

static ssize_t mir3da_int2_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{

	int2_enable = simple_strtoul(buf, NULL, 10);

	return count;
}

static ssize_t mir3da_int2_enable_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "%d\n", int2_enable);
	printk(" mir3da_int2_enable_show ret [ %d ]\n", ret);
	return ret;
}

static ssize_t mir3da_int2_clear_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{

	MIR_HANDLE      handle = mir_handle;

	printk(" mir3da_int2_clear_enable_store int2_clean \n");
	mir3da_clear_intterrupt(handle);

	return count;
}

static ssize_t mir3da_int2_clear_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	//	ret = sprintf(buf, "%d\n", int2_enable);
	//	printk(" mir3da_int2_clear_enable_show ret [ %d ]\n",ret);
	return ret;
}

static ssize_t mir3da_int2_start_statu_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{

//	MIR_HANDLE      handle = mir_handle;

	int2_statu = simple_strtoul(buf, NULL, 10);

	return count;
}

static ssize_t mir3da_int2_start_statu_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
//		 MIR_HANDLE      handle = mir_handle;

	//	int2_statu =  mir3da_read_int_status( handle);

	ret = sprintf(buf, "%d\n", int2_statu);
	printk(" mir3da_int2_enable_show ret [ %d ]\n", ret);
	return ret;
}
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_primary_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	MIR_HANDLE   handle = mir_handle;
	int x = 0, y = 0, z = 0;

	mir3da_get_primary_offset(handle, &x, &y, &z);

	return sprintf(buf, "x=%d ,y=%d ,z=%d\n", x, y, z);

}
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%s_%s\n", DRI_VER, CORE_VER);
}
/*----------------------------------------------------------------------------*/
static ssize_t mir3da_vendor_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "MiraMEMS");
}

static ssize_t mir3da_slope_th_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%x\n", slope_th);
}

static ssize_t mir3da_slope_th_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long data;
	int error = 0;
	error = strict_strtoul(buf, 16, &data);
	if (error)
		return error;
	//printk("%s:data=0x%02x\n",__func__,data);
	if (data == 0x3) { //high
		data = 0x85;
		active_dur = 0x02;
		active_th = 0x0a;
		range_g = RANGE_2G;
	} else if (data == 0xf) { //low
		data = 0x86;
		active_dur = 0x03;
		active_th = 0x30;
		range_g = RANGE_8G;
	} else if (data == 0x5) { //mid
		data = 0x85;
		active_dur = 0x02;
		active_th = 0x15;
		range_g = RANGE_4G;
	}
	printk("set slope 0x%lx\n", data);
	slope_th = data;
	return count;
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(enable,          S_IRUGO | S_IWUGO,  mir3da_enable_show,
		   mir3da_enable_store);
static DEVICE_ATTR(delay,      S_IRUGO | S_IWUGO,  mir3da_delay_show,
		   mir3da_delay_store);
static DEVICE_ATTR(axis_data,       S_IRUGO | S_IWUGO,    mir3da_axis_data_show,
		   NULL);
static DEVICE_ATTR(reg_data,        S_IWUGO | S_IRUGO,  mir3da_reg_data_show,
		   mir3da_reg_data_store);
static DEVICE_ATTR(log_level,       S_IWUGO | S_IRUGO,  mir3da_log_level_show,
		   mir3da_log_level_store);
#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
static DEVICE_ATTR(offset,          S_IWUGO | S_IRUGO,  mir3da_offset_show,
		   mir3da_offset_store);
static DEVICE_ATTR(calibrate_miraGSensor,       S_IWUGO | S_IRUGO,
		   mir3da_calibrate_show,          mir3da_calibrate_store);
#endif
#ifdef FILTER_AVERAGE_ENHANCE
static DEVICE_ATTR(average_enhance, S_IWUGO | S_IRUGO,
		   mir3da_average_enhance_show,    mir3da_average_enhance_store);
#endif
// aad cz
static DEVICE_ATTR(int2_enable,     S_IRUGO | S_IWUGO,  mir3da_int2_enable_show,
		   mir3da_int2_enable_store);
static DEVICE_ATTR(int2_clear,     S_IRUGO | S_IWUGO,
		   mir3da_int2_clear_enable_show  ,             mir3da_int2_clear_enable_store);
static DEVICE_ATTR(int2_start_status,     S_IRUGO | S_IWUGO,
		   mir3da_int2_start_statu_show  ,             mir3da_int2_start_statu_store);

static DEVICE_ATTR(primary_offset,  S_IWUGO,
		   mir3da_primary_offset_show,            NULL);
static DEVICE_ATTR(version,         S_IRUGO,            mir3da_version_show,
		   NULL);
static DEVICE_ATTR(vendor,          S_IRUGO,            mir3da_vendor_show,
		   NULL);
static DEVICE_ATTR(slope_th, S_IRUGO | S_IWUGO, mir3da_slope_th_show,
		   mir3da_slope_th_store);

/*----------------------------------------------------------------------------*/
static struct attribute *mir3da_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_axis_data.attr,
	&dev_attr_reg_data.attr,
	&dev_attr_log_level.attr,
#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
	&dev_attr_offset.attr,
	&dev_attr_calibrate_miraGSensor.attr,
//	&dev_attr_primary_offset.attr,
#endif
#ifdef FILTER_AVERAGE_ENHANCE
	&dev_attr_average_enhance.attr,
#endif /* ! FILTER_AVERAGE_ENHANCE */
	&dev_attr_int2_enable.attr,
	&dev_attr_int2_clear.attr,
	&dev_attr_int2_start_status.attr,
	&dev_attr_primary_offset.attr,
	&dev_attr_version.attr,
	&dev_attr_vendor.attr,
	&dev_attr_slope_th.attr,
	NULL
};

static const struct attribute_group mir3da_attr_group = {
	.attrs  = mir3da_attributes,
};
/*----------------------------------------------------------------------------*/
int i2c_smbus_read(PLAT_HANDLE handle, u8 addr, u8 *data)
{
	int                 res = 0;
	struct i2c_client   *client = (struct i2c_client *)handle;

	*data = i2c_smbus_read_byte_data(client, addr);

	return res;
}
/*----------------------------------------------------------------------------*/
int i2c_smbus_read_block(PLAT_HANDLE handle, u8 addr, u8 count, u8 *data)
{
	int                 res = 0;
	struct i2c_client   *client = (struct i2c_client *)handle;

	res = i2c_smbus_read_i2c_block_data(client, addr, count, data);

	return res;
}
/*----------------------------------------------------------------------------*/
int i2c_smbus_write(PLAT_HANDLE handle, u8 addr, u8 data)
{
	int                 res = 0;
	struct i2c_client   *client = (struct i2c_client *)handle;

	res = i2c_smbus_write_byte_data(client, addr, data);

	return res;
}
/*----------------------------------------------------------------------------*/
void msdelay(int ms)
{
	mdelay(ms);
}

#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
MIR_GENERAL_OPS_DECLARE(ops_handle, i2c_smbus_read, i2c_smbus_read_block,
			i2c_smbus_write, sensor_sync_write, sensor_sync_read, msdelay, printk, sprintf);
#else
MIR_GENERAL_OPS_DECLARE(ops_handle, i2c_smbus_read, i2c_smbus_read_block,
			i2c_smbus_write, NULL, NULL, msdelay, printk, sprintf);
#endif

/*----------------------------------------------------------------------------*/
static int mir3da_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int                 result = 0;
	struct input_dev    *idev;
	struct device_node *np = client->dev.of_node;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		MI_ERR("da380 I2C check functionality failed.");
		return -ENODEV;
	}

	if (!np) {
		MI_ERR("no device tree\n");
		return -EINVAL;
	}

	if (mir3da_install_general_ops(&ops_handle)) {
		MI_ERR("Install ops failed !\n");
		goto err_detach_client;
	}

#ifdef MIR3DA_OFFSET_TEMP_SOLUTION
	m_work_info.wq = create_singlethread_workqueue("oo");
	if (NULL == m_work_info.wq) {
		MI_ERR("Failed to create workqueue !");
		goto err_detach_client;
	}

	INIT_DELAYED_WORK(&m_work_info.read_work, sensor_read_work);
	INIT_DELAYED_WORK(&m_work_info.write_work, sensor_write_work);
#endif

	int2_statu =  mir3da_read_int_status((PLAT_HANDLE)client);
	//printk("ParkMonitor powerOn status is %d\n", int2_statu);

	/* Initialize the MIR3DA chip */
	mir_handle = mir3da_core_init((PLAT_HANDLE)client);
	if (NULL == mir_handle) {
		MI_ERR("chip init failed !\n");
		goto err_detach_client;
	}

	hwmon_dev = hwmon_device_register(&client->dev);
	MI_ASSERT(!(IS_ERR(hwmon_dev)));

	/* input poll device register */
	mir3da_idev = input_allocate_polled_device();
	if (!mir3da_idev) {
		MI_ERR("alloc poll device failed!\n");
		result = -ENOMEM;
		goto err_hwmon_device_unregister;
	}
	mir3da_idev->poll = mir3da_dev_poll;
	mir3da_idev->poll_interval = da380_config_p->poll_delay_ms;
	delayMs = da380_config_p->poll_delay_ms;
	mir3da_idev->poll_interval_max = POLL_INTERVAL_MAX;
	idev = mir3da_idev->input;

	idev->name = MIR3DA_INPUT_DEV_NAME;
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);

	input_set_abs_params(idev, ABS_X, -16384, 16383, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Y, -16384, 16383, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Z, -16384, 16383, INPUT_FUZZ, INPUT_FLAT);

	result = input_register_polled_device(mir3da_idev);
	if (result) {
		MI_ERR("register poll device failed!\n");
		goto err_free_polled_device;
	}

	/* Sys Attribute Register */
	result = sysfs_create_group(&idev->dev.kobj, &mir3da_attr_group);
	if (result) {
		MI_ERR("create device file failed!\n");
		result = -EINVAL;
		goto err_unregister_polled_device;
	}

	/* Misc device interface Register */
	result = misc_register(&misc_mir3da);
	if (result) {
		MI_ERR("%s: mir3da_dev register failed", __func__);
		goto err_remove_sysfs_group;
	}
	slope_th = 0x86;
	return result;

err_remove_sysfs_group:
	sysfs_remove_group(&idev->dev.kobj, &mir3da_attr_group);
err_unregister_polled_device:
	input_unregister_polled_device(mir3da_idev);
err_free_polled_device:
	input_free_polled_device(mir3da_idev);
err_hwmon_device_unregister:
	hwmon_device_unregister(&client->dev);
err_detach_client:
	return result;
}
/*----------------------------------------------------------------------------*/
static int mir3da_remove(struct i2c_client *client)
{
	MIR_HANDLE      handle = mir_handle;

	mir3da_set_enable(handle, 0);

	misc_deregister(&misc_mir3da);

	sysfs_remove_group(&mir3da_idev->input->dev.kobj, &mir3da_attr_group);

	input_unregister_polled_device(mir3da_idev);

	input_free_polled_device(mir3da_idev);

#ifdef  MIR3DA_OFFSET_TEMP_SOLUTION
	flush_workqueue(m_work_info.wq);
	destroy_workqueue(m_work_info.wq);
#endif

	hwmon_device_unregister(hwmon_dev);

	kfree(da380_config_p);

	return 0;
}
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_HAS_EARLYSUSPEND
static int mir3da_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
#if 0
	int result = 0;
	MIR_HANDLE      handle = mir_handle;

	MI_FUN;

	result = mir3da_set_enable(handle, 0);
	if (result) {
		MI_ERR("%s:set disable fail!!\n", __func__);
		return;
	}
	mir3da_idev->input->close(mir3da_idev->input);

	return result;
#endif
}
/*----------------------------------------------------------------------------*/
static int mir3da_resume(struct i2c_client *client)
{
	int result = 0;
	MIR_HANDLE      handle = mir_handle;

	MI_FUN;

	result = mir3da_chip_resume(handle);
	if (result) {
		MI_ERR("%s:chip resume fail!!\n", __func__);
		return result;
	}

	result = mir3da_set_enable(handle, 1);
	if (result) {
		MI_ERR("%s:set enable fail!!\n", __func__);
		return result;
	}

	mir3da_idev->input->open(mir3da_idev->input);

	return result;
}
#endif

static const struct i2c_device_id mir3da_id[] = {
	{ MIR3DA_DRV_NAME, 0 },
	{ }
};

static struct of_device_id mir3da_dt_ids[] = {
	{ .compatible = "mir3da,da380" },
	{ }
};

static struct i2c_driver mir3da_driver = {
	.probe      = mir3da_probe,
	.remove     = mir3da_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend = mir3da_suspend,
	.resume = mir3da_resume,
#endif
	.id_table   = mir3da_id,
	.driver = {
		.name     = MIR3DA_DRV_NAME,
		.owner    = THIS_MODULE,
		.of_match_table = of_match_ptr(mir3da_dt_ids),
	},
};
#if 0
static int da380_interrupt_test(void)
{
	int result = 0;
	MIR_HANDLE handle = mir_handle;

	mdelay(10);
	result = mir3da_set_enable(handle, true);
	if (result) {
		printk("sensor_active enable fail!!\n");
		return result;
	}
	mdelay(10);

	mir3da_open_interrupt(handle, 1, 1);

	return 0;
}
#endif
static int sensor_init(struct i2c_client *client)
{
	int	result = 0;
	struct device_node *np = client->dev.of_node;

	mir3da_driver.id_table = 0x00;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		MI_ERR("da380 I2C check functionality failed.");
		return -ENODEV;
	}

	if (!np) {
		MI_ERR("no device tree\n");
		return -EINVAL;
	}

	if (mir3da_install_general_ops(&ops_handle)) {
		MI_ERR("Install ops failed !\n");
		goto err_detach_client;
	}

	/* Initialize the MIR3DA chip */
	mir_handle = mir3da_core_init((PLAT_HANDLE)client);
	if (NULL == mir_handle) {
		MI_ERR("chip init failed !\n");
		goto err_detach_client;
	}

	slope_th = 0x86;

	/* wrm test */
	//da380_interrupt_test();

	return result;

err_detach_client:
	return result;
}

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	int result = 0;
	MIR_HANDLE handle = mir_handle;

	mdelay(10);
	if (enable) {
		result = mir3da_set_enable(handle, true);
		if (result) {
			printk("sensor_active enable  fail!!\n");
			return result;
		}
	} else {
		result = mir3da_set_enable(handle, false);
		if (result) {
			printk("sensor_active disable  fail!!\n");
			return result;
		}
	}
	mdelay(10);

	return result;
}

static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor = (struct sensor_private_data *)
					     i2c_get_clientdata(client);
	struct sensor_platform_data *pdata = sensor->pdata;
	struct sensor_axis axis;
	int tmp_x = 0, tmp_y = 0, tmp_z = 0;
	short x = 0, y = 0, z = 0;
	int ret = 0;

	ret = mir3da_read_data(mir_handle, &x, &y, &z);
	if (ret) {
		MI_ERR("read data failed!");
		return ret;
	}

	//printk(KERN_DEBUG " x = %d, y = %d, z = %d\n", x, y, z);
	//printk("raw: x = %d, y = %d, z = %d\n", x, y, z);

	tmp_x = x * DA311_GRAVITY_STEP;
	tmp_y = y * DA311_GRAVITY_STEP;
	tmp_z = z * DA311_GRAVITY_STEP;

	//printk(KERN_DEBUG " tmp_x = %d, tmp_y = %d, tmp_z = %d\n", tmp_x, tmp_y, tmp_z);
	//printk("gai: tmp_x = %d, tmp_y = %d, tmp_z = %d\n", tmp_x, tmp_y, tmp_z);
	axis.x = (pdata->orientation[0]) * tmp_x + (pdata->orientation[1]) * tmp_y +
		 (pdata->orientation[2]) * tmp_z;
	axis.y = (pdata->orientation[3]) * tmp_x + (pdata->orientation[4]) * tmp_y +
		 (pdata->orientation[5]) * tmp_z;
#if MIR3DA_STK_TEMP_SOLUTION
	axis.z = (pdata->orientation[6]) * tmp_x + (pdata->orientation[7]) * tmp_y +
		 (bzstk ? 1 : (pdata->orientation[8])) * tmp_z;
#else
	axis.z = (pdata->orientation[6]) * tmp_x + (pdata->orientation[7]) * tmp_y +
		 (pdata->orientation[8]) * tmp_z;
#endif
	//printk( "map: axis = %d  %d  %d \n", axis.x, axis.y, axis.z);

	if ((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) ||
	    (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) ||
	    (abs(sensor->axis.z - axis.z) > GSENSOR_MIN)) {
		input_report_abs(sensor->input_dev, ABS_X, axis.x);
		input_report_abs(sensor->input_dev, ABS_Y, axis.y);
		input_report_abs(sensor->input_dev, ABS_Z, axis.z);
		input_sync(sensor->input_dev);

		mutex_lock(&(sensor->data_mutex));
		sensor->axis = axis;
		mutex_unlock(&(sensor->data_mutex));
	}

	return ret;
}

#define INT_CONFIG 0x20

int sensor_use_interrupt(struct i2c_client *client, int num, int enable)
{
	MIR_HANDLE      handle;
	u8	rvalue;

	handle = mir_handle;

	printk("sensor_use_interrupt num=%d enable=%d\n", num - 1, enable);
	mir3da_open_interrupt(handle, num - 1, enable);
	mdelay(10);
	if (mir3da_register_read(handle, INT_CONFIG, &rvalue) != 0) {
		printk("ERROR: INT_CONFIG = 0x%x\n", rvalue);
	} else {
		printk("0: INT_CONFIG = 0x%x\n", rvalue);
	}
#if 0

	rvalue = 0x03;
	mir3da_register_write(handle, INT_CONFIG, rvalue);

	if (mir3da_register_read(handle, INT_CONFIG, &rvalue) != 0) {
		printk("ERROR: INT_CONFIG = 0x%x\n", rvalue);
	} else {
		printk("1: INT_CONFIG = 0x%x\n", rvalue);
	}
#endif
	return 0;
}

struct sensor_operate mirgsensor_ops = {
	.name               = "mir3da",
	.type               = SENSOR_TYPE_ACCEL,            /*sensor type and it should be correct*/
	.id_i2c             = ACCEL_ID_MIR3DA,              /*i2c id number*/
	.read_reg           = -1,        					/*read data*/
	.read_len           = 0,                            /*data length*/
	.id_reg             = DA380_CHIP_ID_REG,   			/* read device id from this register */
	.id_data            = DA380_CHIP_ID,   				/* device id */
	.precision          = MIR3DA_PRECISION,           	/*12 bit*/
	.ctrl_reg           = -1,							/*enable or disable*/
	/*intterupt status register*/
	.int_status_reg     = 0x00,
	.range              = { -MIR3DA_RANGE, MIR3DA_RANGE},	/*range*/
	.trig               = IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
	.active             = sensor_active,
	.init               = sensor_init,
	.report             = sensor_report_value,
	.interrupt_use      = sensor_use_interrupt,
	//.suspend  			= mir3da_suspend,
	//.resume   			= mir3da_resume,
};

static struct sensor_operate *gsensor_get_ops(void)
{
	return &mirgsensor_ops;
}

static int __init mir3da_init(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int result = 0;
	int type = ops->type;

	result = sensor_register_slave(type, NULL, NULL, gsensor_get_ops);

	return result;
}

static void __exit mir3da_exit(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int type = ops->type;

	sensor_unregister_slave(type, NULL, NULL, gsensor_get_ops);
}

module_init(mir3da_init);
module_exit(mir3da_exit);


#if 0
/*----------------------------------------------------------------------------*/
static int __init mir3da_init(void)
{
	int res;

	MI_FUN;
	printk("step4 : mir3da_init\n");
	res = i2c_add_driver(&mir3da_driver);
	if (res < 0) {
		MI_ERR("add mir3da i2c driver failed\n");
		return -ENODEV;
	}

	return (res);
}
/*----------------------------------------------------------------------------*/
static void __exit mir3da_exit(void)
{
	MI_FUN;

	i2c_del_driver(&mir3da_driver);
}
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("MiraMEMS <lschen@miramems.com>");
MODULE_DESCRIPTION("MIR3DA 3-Axis Accelerometer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

arch_initcall(mir3da_init);
//module_init(mir3da_init);
module_exit(mir3da_exit);
#endif
