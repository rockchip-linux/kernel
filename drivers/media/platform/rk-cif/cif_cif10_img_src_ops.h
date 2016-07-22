/*
**************************************************************************
 * Rockchip driver for CIF CIF 1.0
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

#ifndef _CIF_CIF10_IMG_SRC_OPS_H
#define _CIF_CIF10_IMG_SRC_OPS_H
#include <linux/platform_data/rk_cif10_platform.h>
#include "cif_cif10_img_src_v4l2-subdev.h"

struct cif_cif10_img_src_ops {
	void * (*to_img_src)(
		CIF_CIF10_PLTFRM_DEVICE dev,
		struct pltfrm_soc_cfg *soc_cfg);
	int (*s_streaming)(
		void *img_src,
		bool enable);
	int (*s_power)(
		void *img_src,
		bool on);
	int (*enum_strm_fmts)(
		void *img_src,
		u32 index,
		struct cif_cif10_strm_fmt_desc *strm_fmt_desc);
	int (*s_strm_fmt)(
		void *img_src,
		struct cif_cif10_strm_fmt *strm_fmt);
	int (*g_ctrl)(
		void *img_src,
		int id,
		int *val);
	const char * (*g_name)(
		void *img_src);
	int (*s_ctrl)(
		void *img_src,
		int id,
		int val);
	int (*s_ext_ctrls)(
		void *img_src,
	    struct cif_cif10_img_src_ext_ctrl *ctrl);
	long (*ioctl)(
		void *img_src,
		unsigned int cmd,
		void *arg);
};

const struct {
	const char *device_type;
	struct cif_cif10_img_src_ops ops;
} cif_cif10_img_src_ops[] = {
	{
		.device_type = CIF_CIF10_IMG_SRC_V4L2_I2C_SUBDEV,
		.ops = {
			.to_img_src =
				cif_cif10_img_src_v4l2_i2c_subdev_to_img_src,
			.s_streaming =
				cif_cif10_img_src_v4l2_subdev_s_streaming,
			.s_power =
				cif_cif10_img_src_v4l2_subdev_s_power,
			.enum_strm_fmts =
				cif_cif10_img_src_v4l2_subdev_enum_strm_fmts,
			.s_strm_fmt =
				cif_cif10_img_src_v4l2_subdev_s_strm_fmt,
			.g_ctrl =
				cif_cif10_img_src_v4l2_subdev_g_ctrl,
			.g_name =
				cif_cif10_img_src_v4l2_subdev_g_name,
			.s_ctrl =
				cif_cif10_img_src_v4l2_subdev_s_ctrl,
			.s_ext_ctrls =
				cif_cif10_img_src_v4l2_subdev_s_ext_ctrls,
			.ioctl =
				cif_cif10_img_src_v4l2_subdev_ioctl
		}
	},
	{
		.device_type = CIF_CIF10_IMG_SRC_V4L2_PLTFRM_SUBDEV,
		.ops = {
			.to_img_src =
				cif_cif10_img_src_v4l2_pltfrm_subdev_to_img_src,
			.s_streaming =
				cif_cif10_img_src_v4l2_subdev_s_streaming,
			.s_power =
				cif_cif10_img_src_v4l2_subdev_s_power,
			.enum_strm_fmts =
				cif_cif10_img_src_v4l2_subdev_enum_strm_fmts,
			.s_strm_fmt =
				cif_cif10_img_src_v4l2_subdev_s_strm_fmt,
			.g_ctrl =
				cif_cif10_img_src_v4l2_subdev_g_ctrl,
			.g_name =
				cif_cif10_img_src_v4l2_subdev_g_name,
			.s_ctrl =
				cif_cif10_img_src_v4l2_subdev_s_ctrl,
			.s_ext_ctrls =
				cif_cif10_img_src_v4l2_subdev_s_ext_ctrls,
			.ioctl =
				cif_cif10_img_src_v4l2_subdev_ioctl
		}
	},
};

#endif
