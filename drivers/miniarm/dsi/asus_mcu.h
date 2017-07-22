#ifndef _RPI_TS_MCU_H_
#define _RPI_TS_MCU_H_

#define LOG_INFO(fmt,arg...) pr_info("%s: "fmt, __func__, ##arg);
#define LOG_ERR(fmt,arg...) pr_err("%s: "fmt, __func__, ##arg);

#define MAX_I2C_LEN 255

struct ts_mcu_data {
	struct device *dev;
	struct i2c_client *client;
	struct delayed_work work;
	struct workqueue_struct *wq;
};

#endif
