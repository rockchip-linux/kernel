#ifndef _ROCKPI_MCU_H_
#define _ROCKPI_MCU_H_

#define LOG_INFO(fmt,arg...) pr_info("rockpi-mcu: %s: "fmt, __func__, ##arg);
#define LOG_ERR(fmt,arg...) pr_err("rockpi-mcu: %s: "fmt, __func__, ##arg);

#define MAX_I2C_LEN 255

struct rockpi_mcu_data {
	struct device *dev;
	struct i2c_client *client;
};

#endif
