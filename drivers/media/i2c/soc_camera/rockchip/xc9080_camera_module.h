/*
 * xc9080_camera_module.h
 *
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * Copyright (C) 2012-2014 Intel Mobile Communications GmbH
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#ifndef XC9080_CAMERA_MODULE_H
#define XC9080_CAMERA_MODULE_H
#include <linux/workqueue.h>
#include <linux/platform_data/rk_isp11_platform_camera_module.h>
#include <linux/platform_data/rk_isp11_platform.h>

/* TODO: references to v4l2 should be reomved from here and go into a
 * platform dependent wrapper
 */

#define XC9080_CAMERA_MODULE_REG_TYPE_DATA \
	PLTFRM_CAMERA_MODULE_REG_TYPE_DATA
#define XC9080_CAMERA_MODULE_REG_TYPE_TIMEOUT \
	PLTFRM_CAMERA_MODULE_REG_TYPE_TIMEOUT
#define XC9080_CAMERA_MODULE_REG_TYPE_DATA_SINGLE \
	PLTFRM_CAMERA_MODULE_REG_TYPE_DATA_SINGLE
#define xc9080_camera_module_reg pltfrm_camera_module_reg

#define XC9080_CAMERA_MODULE_CTRL_UPDT_GAIN		0x01
#define XC9080_CAMERA_MODULE_CTRL_UPDT_EXP_TIME		0x02
#define XC9080_CAMERA_MODULE_CTRL_UPDT_WB_TEMPERATURE	0x04
#define XC9080_CAMERA_MODULE_CTRL_UPDT_AUTO_WB		0x08
#define XC9080_CAMERA_MODULE_CTRL_UPDT_AUTO_GAIN	0x10
#define XC9080_CAMERA_MODULE_CTRL_UPDT_AUTO_EXP		0x20
#define XC9080_CAMERA_MODULE_CTRL_UPDT_FOCUS_ABSOLUTE	0x40
#define XC9080_CAMERA_MODULE_CTRL_UPDT_PRESET_WB	0x80

enum xc9080_sub_i2c_bypass_mode {
	XC9080_SUB_I2C_BYPASS_OFF = 0,
	XC9080_SUB_I2C0_BYPASS = 1,
	XC9080_SUB_I2C1_BYPASS = 2,
	XC9080_SUB_I2C_ALL_BYPASS = 3
};

enum xc9080_camera_module_state {
	XC9080_CAMERA_MODULE_POWER_OFF = 0,
	XC9080_CAMERA_MODULE_HW_STANDBY = 1,
	XC9080_CAMERA_MODULE_SW_STANDBY = 2,
	XC9080_CAMERA_MODULE_STREAMING = 3
};

struct xc9080_camera_module;

struct xc9080_camera_module_timings {
	/* public */
	u32 coarse_integration_time_min;
	u32 coarse_integration_time_max_margin;
	u32 fine_integration_time_min;
	u32 fine_integration_time_max_margin;
	u32 frame_length_lines;
	u32 line_length_pck;
	u32 vt_pix_clk_freq_hz;
	u32 sensor_output_width;
	u32 sensor_output_height;
	u32 crop_horizontal_start; /* Sensor crop start cord. (x0,y0)*/
	u32 crop_vertical_start;
	u32 crop_horizontal_end; /* Sensor crop end cord. (x1,y1)*/
	u32 crop_vertical_end;
	u8 binning_factor_x;
	u8 binning_factor_y;
	u32 exp_time;
	u32 gain;
};

struct xc9080_camera_module_config {
	const char *name;
	struct v4l2_mbus_framefmt frm_fmt;
	struct v4l2_subdev_frame_interval frm_intrvl;
	bool auto_exp_enabled;
	bool auto_gain_enabled;
	bool auto_wb_enabled;
	struct xc9080_camera_module_reg *reg_table;
	u32 reg_table_num_entries;
	struct xc9080_camera_module_reg *reg_diff_table;
	u32 reg_diff_table_num_entries;
	struct xc9080_camera_module_reg *reg_sub_table;
	u32 reg_sub_table_num_entries;
	struct i2c_client *sub_client[2];
	u32 v_blanking_time_us;
	u32 line_length_pck;
	u32 frame_length_lines;
	struct xc9080_camera_module_timings timings;
	bool soft_reset;
	bool ignore_measurement_check;

	struct pltfrm_cam_itf itf_cfg;
};

struct xc9080_camera_module_exp_config {
	s32 exp_time;
	bool auto_exp;
	u16 gain;
	u16 gain_percent;
	bool auto_gain;
	enum v4l2_flash_led_mode flash_mode;
};

struct xc9080_camera_module_wb_config {
	u32 temperature;
	u32 preset_id;
	bool auto_wb;
};

struct xc9080_camera_module_af_config {
	u32 abs_pos;
	u32 rel_pos;
};

struct xc9080_camera_module_ext_ctrl {
	/* public */
	u32 id;
	u32 value;
	__u32 reserved2[1];
};

struct xc9080_camera_module_ext_ctrls {
	/* public */
	u32 count;
	struct xc9080_camera_module_ext_ctrl *ctrls;
};

/*
 * start_streaming: (mandatory) will be called when sensor should be
 * put into streaming mode right after the base config has been
 * written to the sensor. After a successful call of this function
 * the sensor should start delivering frame data.
 *
 * stop_streaming: (mandatory) will be called when sensor should stop
 * delivering data. After a successful call of this function the
 * sensor should not deliver any more frame data.
 *
 * check_camera_id: (optional) will be called when the sensor is
 * powered on. If provided should check the sensor ID/version
 * required by the custom driver. Register access should be
 * possible when this function is invoked.

 * s_ctrl: (mandatory) will be called at the successful end of
 * xc9080_camera_module_s_ctrl with the ctrl_id as argument.
 *
 * priv: (optional) for private data used by the custom driver.
 */

struct xc9080_camera_module_custom_config {
	int (*start_streaming)(struct xc9080_camera_module *cam_mod);
	int (*stop_streaming)(struct xc9080_camera_module *cam_mod);
	int (*check_camera_id)(struct xc9080_camera_module *cam_mod);
	int (*s_ctrl)(struct xc9080_camera_module *cam_mod, u32 ctrl_id);
	int (*g_ctrl)(struct xc9080_camera_module *cam_mod, u32 ctrl_id);
	int (*g_timings)(struct xc9080_camera_module *cam_mod,
		struct xc9080_camera_module_timings *timings);
	int (*s_vts)(struct xc9080_camera_module *cam_mod,
		u32 vts);
	int (*s_ext_ctrls)(struct xc9080_camera_module *cam_mod,
		struct xc9080_camera_module_ext_ctrls *ctrls);
	int (*set_flip)(
		struct xc9080_camera_module *cam_mod,
		struct pltfrm_camera_module_reg reglist[],
		int len);
	int (*init_common)(struct xc9080_camera_module *cam_mod);
	int (*read_otp)(struct xc9080_camera_module *cam_mod);
	struct xc9080_camera_module_config *configs;
	u32 num_configs;
	u32 power_up_delays_ms[3];
	unsigned char exposure_valid_frame[2];
	void *priv;
};

struct xc9080_camera_module {
	/* public */
	struct v4l2_subdev sd;
	struct v4l2_mbus_framefmt frm_fmt;
	struct v4l2_subdev_frame_interval frm_intrvl;
	struct xc9080_camera_module_exp_config exp_config;
	struct xc9080_camera_module_wb_config wb_config;
	struct xc9080_camera_module_af_config af_config;
	struct xc9080_camera_module_custom_config custom;
	enum xc9080_camera_module_state state;
	enum xc9080_camera_module_state state_before_suspend;
	struct xc9080_camera_module_config *active_config;
	u32 ctrl_updt;
	u32 vts_cur;
	u32 vts_min;
	bool auto_adjust_fps;
	bool update_config;
	bool frm_fmt_valid;
	bool frm_intrvl_valid;
	bool hflip;
	bool vflip;
	u32 rotation;
	void *pltfm_data;
	bool inited;
};

#define xc9080_camera_module_pr_info(cam_mod, fmt, arg...) \
	pltfrm_camera_module_pr_info(&(cam_mod)->sd, fmt, ## arg)
#define xc9080_camera_module_pr_debug(cam_mod, fmt, arg...) \
	pltfrm_camera_module_pr_debug(&(cam_mod)->sd, fmt, ## arg)
#define xc9080_camera_module_pr_warn(cam_mod, fmt, arg...) \
	pltfrm_camera_module_pr_warn(&(cam_mod)->sd, fmt, ## arg)
#define xc9080_camera_module_pr_err(cam_mod, fmt, arg...) \
	pltfrm_camera_module_pr_err(&(cam_mod)->sd, fmt, ## arg)

int xc9080_camera_module_write_reglist(
	struct i2c_client *client,
	const struct xc9080_camera_module_reg reglist[],
	int len);

int xc9080_camera_module_write_reg(
	struct i2c_client *client,
	u16 reg,
	u8 val);

int xc9080_camera_module_read_reg(
	struct i2c_client *client,
	u16 data_length,
	u16 reg,
	u32 *val);

int xc9080_camera_module_read_reg_table(
	struct xc9080_camera_module *cam_mod,
	u16 reg,
	u32 *val);

int xc9080_camera_module_try_fmt(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt);

int xc9080_camera_module_s_fmt(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt);

int xc9080_camera_module_g_fmt(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt);

int xc9080_camera_module_s_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_frame_interval *interval);

int xc9080_camera_module_g_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_frame_interval *interval);

int xc9080_camera_module_s_stream(
	struct v4l2_subdev *sd,
	int enable);

int xc9080_camera_module_s_power(
	struct v4l2_subdev *sd,
	int on);

int xc9080_camera_module_g_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl);

int xc9080_camera_module_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl);

int xc9080_camera_module_s_ext_ctrls(
	struct v4l2_subdev *sd,
	struct v4l2_ext_controls *ctrls);

int xc9080_camera_module_enum_frameintervals(
	struct v4l2_subdev *sd,
	struct v4l2_frmivalenum *fival);

int xc9080_camera_module_init(
	struct xc9080_camera_module *cam_mod,
	struct xc9080_camera_module_custom_config *custom);

void xc9080_camera_module_release(
	struct xc9080_camera_module *cam_mod);

long xc9080_camera_module_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd,
	void *arg);

int xc9080_camera_module_get_flip_mirror(
	struct xc9080_camera_module *cam_mod);

int xc9080_camera_module_i2c_bypass(
	struct xc9080_camera_module *cam_mod,
	int index);
#endif
