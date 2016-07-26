/*
**************************************************************************
 * Rockchip driver for CIF ISP 1.1
 * (Based on Intel driver for sofiaxxx)
 *
 * Copyright (C) 2015 Intel Mobile Communications GmbH
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
**************************************************************************
 */

#ifndef _V4L2_CONTROLS_ROCKCHIP_H
#define _V4L2_CONTROLS_ROCKCHIP_H

#include <linux/videodev2.h>

#define CAMERA_STRLEN   32

/* Sensor resolution specific data for AE calculation.*/
struct isp_supplemental_sensor_mode_data {
	unsigned int coarse_integration_time_min;
	unsigned int coarse_integration_time_max_margin;
	unsigned int fine_integration_time_min;
	unsigned int fine_integration_time_max_margin;
	unsigned int frame_length_lines;
	unsigned int line_length_pck;
	unsigned int vt_pix_clk_freq_hz;
	unsigned int crop_horizontal_start; /* Sensor crop start cord. (x0,y0)*/
	unsigned int crop_vertical_start;
	unsigned int crop_horizontal_end; /* Sensor crop end cord. (x1,y1)*/
	unsigned int crop_vertical_end;
	unsigned int sensor_output_width; /* input size to ISP */
	unsigned int sensor_output_height;
	unsigned int isp_input_horizontal_start;
	unsigned int isp_input_vertical_start;
	unsigned int isp_input_width;
	unsigned int isp_input_height;
	unsigned char binning_factor_x; /* horizontal binning factor used */
	unsigned char binning_factor_y; /* vertical binning factor used */
	unsigned char exposure_valid_frame;
	int exp_time;
	unsigned short gain;
};


struct camera_module_info_s {
	char sensor_name[CAMERA_STRLEN];
	char module_name[CAMERA_STRLEN];
	char len_name[CAMERA_STRLEN];
	char fov_h[CAMERA_STRLEN];
	char fov_v[CAMERA_STRLEN];
	char focal_length[CAMERA_STRLEN];
	char focus_distance[CAMERA_STRLEN];
	int facing;
	int orientation;
	bool iq_mirror;
	bool iq_flip;
	int flash_support;
	int flash_exp_percent;
};

struct flash_timeinfo_s {
	struct timeval preflash_start_t;
	struct timeval preflash_end_t;
	struct timeval mainflash_start_t;
	struct timeval mainflash_end_t;
	int flash_turn_on_time;
	int flash_on_timeout;
};

struct frame_timeinfo_s {
	struct timeval vs_t;
	struct timeval fi_t;
	unsigned int frame_id;
	bool exposure_active;
};

struct v4l2_buffer_timeinfo_s {
	unsigned int index;
	struct flash_timeinfo_s flash;
	struct frame_timeinfo_s frame;
};

#define RK_VIDIOC_CAMERA_MODULEINFO \
	_IOWR('v', BASE_VIDIOC_PRIVATE + 10, struct camera_module_info_s)
#define RK_VIDIOC_V4L2BUFFER_TIMEINFO \
	_IOWR('v', BASE_VIDIOC_PRIVATE + 11, struct v4l2_buffer_timeinfo_s)
#define RK_VIDIOC_SENSOR_MODE_DATA \
	_IOR('v', BASE_VIDIOC_PRIVATE, struct isp_supplemental_sensor_mode_data)

#define V4L2_CID_USER_RK_BASE (V4L2_CID_USER_BASE + 0x1080)
#define RK_V4L2_CID_VBLANKING (V4L2_CID_USER_RK_BASE + 1)
#define RK_V4L2_CID_GAIN_PERCENT (V4L2_CID_USER_RK_BASE + 2)
#define RK_V4L2_CID_AUTO_FPS (V4L2_CID_USER_RK_BASE + 3)
#endif
