#ifndef _ROCKPI_FT5406_H_
#define _ROCKPI_FT5406_H_

#define LOG_DBG(fmt,arg...) pr_debug("rockpi-ft5406: %s: "fmt, __func__, ##arg);
#define LOG_INFO(fmt,arg...) pr_info("rockpi-ft5406: %s: "fmt, __func__, ##arg);
#define LOG_ERR(fmt,arg...) pr_err("rockpi-ft5406: %s: "fmt, __func__, ##arg);

#define XY_REVERSE 1

#define SCREEN_WIDTH	800
#define SCREEN_HEIGHT	480

#define FT_ONE_TCH_LEN	6

#define FT_REG_FW_VER			0xA6
#define FT_REG_FW_MIN_VER		0xB2
#define FT_REG_FW_SUB_MIN_VER	0xB3

#define VALID_TD_STATUS_VAL		10
#define MAX_TOUCH_POINTS		1

#define FT_PRESS			0x7F
#define FT_MAX_ID			0x0F

#define FT_TOUCH_X_H	0
#define FT_TOUCH_X_L	1
#define FT_TOUCH_Y_H	2
#define FT_TOUCH_Y_L	3
#define FT_TOUCH_EVENT	0
#define FT_TOUCH_ID		2

#define FT_TOUCH_X_H_REG	3
#define FT_TOUCH_X_L_REG	4
#define FT_TOUCH_Y_H_REG	5
#define FT_TOUCH_Y_L_REG	6
#define FT_TD_STATUS_REG	2
#define FT_TOUCH_EVENT_REG	3
#define FT_TOUCH_ID_REG		5

#define FT_TOUCH_DOWN		0
#define FT_TOUCH_CONTACT	2

struct ts_event {
	u16 au16_x[MAX_TOUCH_POINTS]; /*x coordinate */
	u16 au16_y[MAX_TOUCH_POINTS]; /*y coordinate */
	u8 au8_touch_event[MAX_TOUCH_POINTS]; /*touch event: 0:down; 1:up; 2:contact */
	u8 au8_finger_id[MAX_TOUCH_POINTS]; /*touch ID */
	u16 pressure;
	u8 touch_point;
	u8 point_num;
};

struct rockpi_ft5406_data {
	struct device *dev;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct work_struct ft5406_work;

	int known_ids;
};

#endif

