/*
**************************************************************************
 * Rockchip driver for NVP6124b
 * (Based on NEXTCHIP driver for nvp)
 *
 * Copyright 2011 by NEXTCHIP Co., Ltd.
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
**************************************************************************
 */
#ifndef _NVP6124_VIDEO_HI_
#define _NVP6124_VIDEO_HI_

#include <media/v4l2-device.h>
#include <media/v4l2-controls_rockchip.h>
#include <linux/platform_data/rk_cif10_platform.h>

#define NVP6124_R0_ID	0x84
#define NVP6114A_R0_ID	0x85
#define NVP6124B_R0_ID	0x86

#define NTSC	0x00
#define PAL	0x01

struct nvp_module_config {
	const char *name;
	struct v4l2_mbus_framefmt frm_fmt;
	struct v4l2_subdev_frame_interval frm_intrvl;
	struct pltfrm_cam_itf itf_cfg;
};

struct nvp {
	u8 device_id;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct workqueue_struct *wq;
	int reset_gpio;
	int irq;

	u8 cif_id;
	u32 channels;
	u32 apio_vol;
	const char *nvp_mode;
	const char *cvbs_mode;
	struct pltfrm_cam_defrect defrects[4];

	struct pltfrm_soc_cfg *soc_cfg;
};

enum nvp_mode {
	NVP6124_VI_SD = 0,      /* 960x576i(480) */
	NVP6124_VI_720P_2530,	/* 1280x720@25p(30) */
	NVP6124_VI_720P_5060,	/* 1280x720@50p(60) */
	NVP6124_VI_1080P_2530,	/* 1920x1080@25p(30) */
	NVP6124_VI_1920H,	/* 1920x576i(480) */
	NVP6124_VI_720H,	/* 720x576i(480) */
	NVP6124_VI_1280H,	/* 1280x576i(480) */
	NVP6124_VI_1440H,	/* 1440x576i(480) */
	NVP6124_VI_960H2EX,	/* 3840x576i(480) */
	NVP6124_VI_HDEX,	/* 2560x720@25p(30) */
	NVP6124_VI_BUTT
};

enum _nvp6124_outmode_sel {
	NVP6124_OUTMODE_1MUX_SD = 0,
	NVP6124_OUTMODE_1MUX_HD,
	NVP6124_OUTMODE_1MUX_HD5060,
	NVP6124_OUTMODE_1MUX_FHD,
	NVP6124_OUTMODE_2MUX_SD,
	NVP6124_OUTMODE_2MUX_HD_X,
	NVP6124_OUTMODE_2MUX_HD,
	NVP6124_OUTMODE_2MUX_FHD_X,
	NVP6124_OUTMODE_4MUX_SD,
	NVP6124_OUTMODE_4MUX_HD_X,
	NVP6124_OUTMODE_4MUX_HD,
	NVP6124_OUTMODE_2MUX_FHD,
	NVP6124_OUTMODE_1MUX_HD_X,
	NVP6124_OUTMODE_1MUX_FHD_X,
	NVP6124_OUTMODE_4MUX_FHD_X,
	NVP6124_OUTMODE_4MUX_MIX,
	NVP6124_OUTMODE_2MUX_MIX,
	NVP6124_OUTMODE_BUTT
};

void nvp6124_each_mode_setting(
		struct nvp *nvp,
		unsigned char ch,
		unsigned char vformat,
		enum nvp_mode chmode);

void nvp6124b_set_portmode(
		struct nvp *nvp,
		unsigned char portsel,
		unsigned char portmode,
		unsigned char chid);

void nvp6124b_common_init(struct nvp *nvp);
int nvp6124b_check_id(struct nvp *nvp);

#endif
