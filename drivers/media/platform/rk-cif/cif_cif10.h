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

#ifndef _CIF_CIF10_H
#define _CIF_CIF10_H

#include <linux/platform_device.h>
#include "cif_cif10_pltfrm.h"
#include "cif_cif10_img_src.h"
#include <linux/platform_data/rk_cif10_platform.h>
#include <media/v4l2-device.h>
#include <media/v4l2-controls_rockchip.h>
/*****************************************************************************/
/* Definitions */

#define CONFIG_CIF_CIF_AUTO_UPD_CFG_BUG

#define CIF_CIF10_NUM_INPUTS 10

#define DRIVER_NAME "cif_cif10"

/* FORMAT */
#define MAX_NB_FORMATS	30

#define CONTRAST_DEF	0x80
#define BRIGHTNESS_DEF	0x0
#define HUE_DEF	0x0

/*
	MIPI CSI2.0
*/
#define CSI2_DT_YUV420_8b	(0x18)
#define CSI2_DT_YUV420_10b	(0x19)
#define CSI2_DT_YUV422_8b	(0x1E)
#define CSI2_DT_YUV422_10b	(0x1F)
#define CSI2_DT_RGB565	(0x22)
#define CSI2_DT_RGB666	(0x23)
#define CSI2_DT_RGB888	(0x24)
#define CSI2_DT_RAW8	(0x2A)
#define CSI2_DT_RAW10	(0x2B)
#define CSI2_DT_RAW12	(0x2C)

enum cif_cif10_img_src_state {
	CIF_CIF10_IMG_SRC_STATE_OFF = 0,
	CIF_CIF10_IMG_SRC_STATE_SW_STNDBY = 1,
	CIF_CIF10_IMG_SRC_STATE_STREAMING = 2
};

enum cif_cif10_state {
	/* path not yet opened: */
	CIF_CIF10_STATE_DISABLED = 0,
	/* path opened but not yet configured: */
	CIF_CIF10_STATE_INACTIVE = 1,
	/* path opened and configured, ready for streaming: */
	CIF_CIF10_STATE_READY = 2,
	/* path is streaming: */
	CIF_CIF10_STATE_STREAMING = 3
};

enum cif_cif10_pm_state {
	CIF_CIF10_PM_STATE_OFF,
	CIF_CIF10_PM_STATE_SUSPENDED,
	CIF_CIF10_PM_STATE_SW_STNDBY,
	CIF_CIF10_PM_STATE_STREAMING
};

enum cif_cif10_inp {
	CIF_CIF10_INP_CSI = 0x10000000,
	CIF_CIF10_INP_CPI = 0x20000000,

	CIF_CIF10_INP_MAX = 0x7fffffff
};
#define CIF_CIF10_INP_IS_MIPI(inp)  ((inp & 0xf0000000) == CIF_CIF10_INP_CSI)
#define CIF_CIF10_INP_IS_DVP(inp)   ((inp & 0xf0000000) == CIF_CIF10_INP_CPI)
#define CIF_CIF10_INP_NEED_CIF(inp) (inp <  CIF_CIF10_INP_DMA_IE)


enum cif_cif10_pinctrl_state {
	CIF_CIF10_PINCTRL_STATE_SLEEP,
	CIF_CIF10_PINCTRL_STATE_INACTIVE,
	CIF_CIF10_PINCTRL_STATE_DEFAULT,
	CIF_CIF10_PINCTRL_STATE_ACTIVE
};

enum cif_cif10_flash_mode {
	CIF_CIF10_FLASH_MODE_OFF,
	CIF_CIF10_FLASH_MODE_FLASH,
	CIF_CIF10_FLASH_MODE_TORCH,
};

enum cif_cif10_cid {
	CIF_CIF10_CID_FLASH_MODE = 0,
	CIF_CIF10_CID_EXPOSURE_TIME = 1,
	CIF_CIF10_CID_ANALOG_GAIN = 2,
	CIF_CIF10_CID_WB_TEMPERATURE = 3,
	CIF_CIF10_CID_BLACK_LEVEL = 4,
	CIF_CIF10_CID_AUTO_GAIN = 5,
	CIF_CIF10_CID_AUTO_EXPOSURE = 6,
	CIF_CIF10_CID_AUTO_WHITE_BALANCE = 7,
	CIF_CIF10_CID_FOCUS_ABSOLUTE = 8,
	CIF_CIF10_CID_AUTO_N_PRESET_WHITE_BALANCE = 9,
	CIF_CIF10_CID_SCENE_MODE = 10,
	CIF_CIF10_CID_SUPER_IMPOSE = 11,
	CIF_CIF10_CID_JPEG_QUALITY = 12,
	CIF_CIF10_CID_IMAGE_EFFECT = 13,
	CIF_CIF10_CID_HFLIP = 14,
	CIF_CIF10_CID_VFLIP = 15,
	CIF_CIF10_CID_AUTO_FPS = 16,
	CIF_CIF10_CID_VBLANKING = 17,
	CIF_CIF10_CID_ISO_SENSITIVITY = 18,

};

/* correspond to bit field values */
enum cif_cif10_image_effect {
	CIF_CIF10_IE_BW = 0,
	CIF_CIF10_IE_NEGATIVE = 1,
	CIF_CIF10_IE_SEPIA = 2,
	CIF_CIF10_IE_C_SEL = 3,
	CIF_CIF10_IE_EMBOSS = 4,
	CIF_CIF10_IE_SKETCH = 5,
	CIF_CIF10_IE_NONE /* not a bit field value */
};

#define CIF_CIF10_PIX_FMT_MASK					0xf0000000
#define CIF_CIF10_PIX_FMT_MASK_BPP				0x0003f000

#define CIF_CIF10_PIX_FMT_YUV_MASK_CPLANES	0x00000003
#define CIF_CIF10_PIX_FMT_YUV_MASK_UVSWAP	0x00000004
#define CIF_CIF10_PIX_FMT_YUV_MASK_YCSWAP		0x00000008
#define CIF_CIF10_PIX_FMT_YUV_MASK_X			0x00000f00
#define CIF_CIF10_PIX_FMT_YUV_MASK_Y			0x000000f0

#define CIF_CIF10_PIX_FMT_RGB_MASK_PAT		0x000000f0

#define CIF_CIF10_PIX_FMT_BAYER_MASK_PAT		0x000000f0

#define CIF_CIF10_PIX_FMT_GET_BPP(pix_fmt) \
	((pix_fmt & CIF_CIF10_PIX_FMT_MASK_BPP) >> 12)
#define cif_cif10_pix_fmt_set_bpp(pix_fmt, bpp) \
	{ \
		pix_fmt = ((pix_fmt & ~CIF_CIF10_PIX_FMT_MASK_BPP) | \
			((bpp << 12) & CIF_CIF10_PIX_FMT_MASK_BPP)); \
	}

#define CIF_CIF10_PIX_FMT_YUV_GET_NUM_CPLANES(pix_fmt) \
	(pix_fmt & CIF_CIF10_PIX_FMT_YUV_MASK_CPLANES)
#define CIF_CIF10_PIX_FMT_YUV_IS_YC_SWAPPED(pix_fmt) \
	(pix_fmt & CIF_CIF10_PIX_FMT_YUV_MASK_YCSWAP)
#define CIF_CIF10_PIX_FMT_YUV_IS_UV_SWAPPED(pix_fmt) \
		(pix_fmt & CIF_CIF10_PIX_FMT_YUV_MASK_UVSWAP)
#define CIF_CIF10_PIX_FMT_YUV_GET_X_SUBS(pix_fmt) \
	((pix_fmt & CIF_CIF10_PIX_FMT_YUV_MASK_X) >> 8)
#define CIF_CIF10_PIX_FMT_YUV_GET_Y_SUBS(pix_fmt) \
	((pix_fmt & CIF_CIF10_PIX_FMT_YUV_MASK_Y) >> 4)
#define cif_cif10_pix_fmt_set_y_subs(pix_fmt, y_subs) \
	{ \
		pix_fmt = ((pix_fmt & ~CIF_CIF10_PIX_FMT_YUV_MASK_Y) | \
			((y_subs << 4) & CIF_CIF10_PIX_FMT_YUV_MASK_Y)); \
	}
#define cif_cif10_pix_fmt_set_x_subs(pix_fmt, x_subs) \
	{ \
		pix_fmt = ((pix_fmt & ~CIF_CIF10_PIX_FMT_YUV_MASK_X) | \
			((x_subs << 8) & CIF_CIF10_PIX_FMT_YUV_MASK_X)); \
	}
#define cif_cif10_pix_fmt_set_yc_swapped(pix_fmt, yc_swapped) \
	{ \
		pix_fmt = ((pix_fmt & ~CIF_CIF10_PIX_FMT_YUV_MASK_YCSWAP) | \
			((yc_swapped << 3) & \
			CIF_CIF10_PIX_FMT_YUV_MASK_YCSWAP)); \
	}

#define CIF_CIF10_PIX_FMT_BAYER_PAT_IS_BGGR(pix_fmt) \
	((pix_fmt & CIF_CIF10_PIX_FMT_BAYER_MASK_PAT) == 0x0)
#define CIF_CIF10_PIX_FMT_BAYER_PAT_IS_GBRG(pix_fmt) \
	((pix_fmt & CIF_CIF10_PIX_FMT_BAYER_MASK_PAT) == 0x10)
#define CIF_CIF10_PIX_FMT_BAYER_PAT_IS_GRBG(pix_fmt) \
	((pix_fmt & CIF_CIF10_PIX_FMT_BAYER_MASK_PAT) == 0x20)
#define CIF_CIF10_PIX_FMT_BAYER_PAT_IS_RGGB(pix_fmt) \
	((pix_fmt & CIF_CIF10_PIX_FMT_BAYER_MASK_PAT) == 0x30)

#define CIF_CIF10_PIX_FMT_IS_YUV(pix_fmt) \
	((pix_fmt & CIF_CIF10_PIX_FMT_MASK) == 0x10000000)
#define CIF_CIF10_PIX_FMT_IS_RGB(pix_fmt) \
	((pix_fmt & CIF_CIF10_PIX_FMT_MASK) == 0x20000000)
#define CIF_CIF10_PIX_FMT_IS_RAW_BAYER(pix_fmt) \
	((pix_fmt & CIF_CIF10_PIX_FMT_MASK) == 0x30000000)
#define CIF_CIF10_PIX_FMT_IS_JPEG(pix_fmt) \
	((pix_fmt & CIF_CIF10_PIX_FMT_MASK) == 0x40000000)

#define CIF_CIF10_PIX_FMT_IS_INTERLEAVED(pix_fmt) \
	(!CIF_CIF10_PIX_FMT_IS_YUV(pix_fmt) ||\
	!CIF_CIF10_PIX_FMT_YUV_GET_NUM_CPLANES(pix_fmt))



enum cif_cif10_pix_fmt {
	/* YUV */
	CIF_YUV400				= 0x10008000,
	CIF_YVU400				= 0x10008004,

	CIF_YUV420I				= 0x1000c220,
	CIF_YUV420SP			= 0x1000c221,	/* NV12 */
	CIF_YUV420P				= 0x1000c222,
	CIF_YVU420I				= 0x1000c224,
	CIF_YVU420SP			= 0x1000c225,	/* NV21 */
	CIF_YVU420P				= 0x1000c226,	/* YV12 */

	CIF_YUV422I				= 0x10010240,
	CIF_YUV422SP			= 0x10010241,
	CIF_YUV422P				= 0x10010242,
	CIF_YVU422I				= 0x10010244,
	CIF_YVU422SP			= 0x10010245,
	CIF_YVU422P				= 0x10010246,

	CIF_YUV444I				= 0x10018440,
	CIF_YUV444SP			= 0x10018441,
	CIF_YUV444P				= 0x10018442,
	CIF_YVU444I				= 0x10018444,
	CIF_YVU444SP			= 0x10018445,
	CIF_YVU444P				= 0x10018446,

	CIF_UYV400				= 0x10008008,

	CIF_UYV420I				= 0x1000c228,
	CIF_UYV420SP			= 0x1000c229,
	CIF_UYV420P				= 0x1000c22a,
	CIF_VYU420I				= 0x1000c22c,
	CIF_VYU420SP			= 0x1000c22d,
	CIF_VYU420P				= 0x1000c22e,

	CIF_UYV422I				= 0x10010248,
	CIF_UYV422SP			= 0x10010249,
	CIF_UYV422P				= 0x1001024a,
	CIF_VYU422I				= 0x1001024c,
	CIF_VYU422SP			= 0x1001024d,
	CIF_VYU422P				= 0x1001024e,

	CIF_UYV444I				= 0x10018448,
	CIF_UYV444SP			= 0x10018449,
	CIF_UYV444P				= 0x1001844a,
	CIF_VYU444I				= 0x1001844c,
	CIF_VYU444SP			= 0x1001844d,
	CIF_VYU444P				= 0x1001844e,

	/* RGB */
	CIF_RGB565				= 0x20010000,
	CIF_RGB666				= 0x20012000,
	CIF_RGB888				= 0x20018000,

	/* RAW Bayer */
	CIF_BAYER_SBGGR8		= 0x30008000,
	CIF_BAYER_SGBRG8		= 0x30008010,
	CIF_BAYER_SGRBG8		= 0x30008020,
	CIF_BAYER_SRGGB8		= 0x30008030,

	CIF_BAYER_SBGGR10		= 0x3000a000,
	CIF_BAYER_SGBRG10		= 0x3000a010,
	CIF_BAYER_SGRBG10		= 0x3000a020,
	CIF_BAYER_SRGGB10		= 0x3000a030,

	CIF_BAYER_SBGGR12		= 0x3000c000,
	CIF_BAYER_SGBRG12		= 0x3000c010,
	CIF_BAYER_SGRBG12		= 0x3000c020,
	CIF_BAYER_SRGGB12		= 0x3000c030,

	/* JPEG */
	CIF_JPEG					= 0x40008000,

	/* Data */
	CIF_DATA					= 0x70000000,

	CIF_UNKNOWN_FORMAT	= 0x80000000
};

#define CIF_CIF10_ALL_STREAMS \
	(CIF_CIF10_STREAM_CVBS | \
	CIF_CIF10_STREAM_NVP | \
	CIF_CIF10_STREAM_CIF)

enum cif_cif10_buff_fmt {
	/* values correspond to bitfield values */
	CIF_CIF10_BUFF_FMT_PLANAR = 0,
	CIF_CIF10_BUFF_FMT_SEMIPLANAR = 1,
	CIF_CIF10_BUFF_FMT_INTERLEAVED = 2,

	CIF_CIF10_BUFF_FMT_RAW8 = 0,
	CIF_CIF10_BUFF_FMT_RAW12 = 2
};

struct cif_cif10_paraport_config {
	u32 cif_vsync;
	u32 cif_hsync;
	u32 cif_pclk;
	/* really used csi */
	u32 used_csi; /* xuhf@rock-chips.com: v1.0.4 */
};

struct cif_cif10_frm_intrvl {
	u32 numerator;
	u32 denominator;
};

struct cif_cif10_frm_fmt {
	u32 width;
	u32 height;
	u32 stride;
	u32 std_id;
	u32 llength;
	enum cif_cif10_pix_fmt pix_fmt;
	struct v4l2_rect defrect;
};

struct cif_cif10_strm_fmt {
	struct cif_cif10_frm_fmt frm_fmt;
	struct cif_cif10_frm_intrvl frm_intrvl;
};

struct cif_cif10_strm_fmt_desc {
	bool discrete_frmsize;
	struct {
		u32 width;
		u32 height;
	} min_frmsize;
	struct {
		u32 width;
		u32 height;
	} max_frmsize;
	enum cif_cif10_pix_fmt pix_fmt;
	bool discrete_intrvl;
	struct cif_cif10_frm_intrvl min_intrvl;
	struct cif_cif10_frm_intrvl max_intrvl;
	struct v4l2_rect defrect;
	u32 std_id;
};

struct cif_cif10_dcrop_config {
	unsigned int h_offs;
	unsigned int v_offs;
	unsigned int h_size;
	unsigned int v_size;
};

struct cif_cif10_zoom_buffer_info {
	u32 width;
	u32 height;
	unsigned long  buff_addr;
	u32 flags;
};

#ifdef NO_YET
struct cif_cif10_buffer {
	struct list_head list;
	u32 dma_addr;
	u32 size;
};
#else
#define cif_cif10_buffer videobuf_buffer
#endif

struct cif_cif10_frame_timeinfo_s {
	unsigned int cnt;
	struct frame_timeinfo_s *frame_t;
};

struct cif_cif10_stream {
	enum cif_cif10_state state;
	enum cif_cif10_state saved_state;
	struct list_head buf_queue;
	struct videobuf_buffer *curr_buf;
	struct videobuf_buffer *next_buf;
	bool updt_cfg;
	bool stall;
	bool stop;
	CIF_CIF10_PLTFRM_EVENT done;
	struct cif_cif10_frame_timeinfo_s frame;
};
struct cif_cif10_config {
	CIF_CIF10_PLTFRM_MEM_IO_ADDR base_addr;
	enum cif_cif10_flash_mode flash_mode;
	enum cif_cif10_inp input_sel;
	struct cif_cif10_frm_fmt output;
	struct cif_cif10_strm_fmt img_src_output;
	struct pltfrm_cam_itf cam_itf;
	bool out_of_buffer_stall;
};

/* ======================================================================== */

struct cif_cif10_fmt {
	char *name;
	u32 fourcc;
	int flags;
	int depth;
	unsigned char rotation;
	unsigned char overlay;
};

struct cif_cif10_irqinfo {
	unsigned int irq;
	short plug;
	short cif_frm0_ok;
	short cif_frm1_ok;
	unsigned long long cifirq_interval;
	unsigned long long cifirq_idx;
	unsigned long long cifirq_normal_idx;
	unsigned long long cifirq_abnormal_idx;
	unsigned long long dmairq_idx;
};
/* ======================================================================== */

/* =============================================== */
struct cif_cif10_pltfrm_data {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	void __iomem *base_addr;
	struct regmap *regmap_cru;
	int irq;
	s32 exp_time;
	u16 gain;
};


struct cif_cif10_device {
	CIF_CIF10_PLTFRM_DEVICE dev;
	struct v4l2_device v4l2_dev;
	enum cif_cif10_pm_state pm_state;
	enum cif_cif10_img_src_state img_src_state;

	spinlock_t vbq_lock;	/* spinlock for videobuf queues */
	spinlock_t vbreq_lock;	/* spinlock for videobuf requeues */

	struct cif_cif10_img_src *img_src;
	struct cif_cif10_img_src *img_src_array[CIF_CIF10_NUM_INPUTS];
	unsigned int img_src_cnt;

	struct cif_cif10_config config;
	struct cif_cif10_stream stream;
	struct timeval curr_frame_time; /* updated each frame */

	void (*sof_event)(__u32 frame_sequence);
	/* requeue_bufs() is used to clean and rebuild the local buffer
	lists xx_stream.buf_queue. This is used e.g. in the CAPTURE use
	case where we start MP and SP separately and needs to shortly
	stop and start SP when start MP */
	void (*requeue_bufs)(void);
	struct flash_timeinfo_s flash_t;

	struct pltfrm_soc_cfg *soc_cfg;

	struct cif_cif10_irqinfo irqinfo;
	struct work_struct work;
	struct workqueue_struct *wq;
};

struct cif_cif10_fmt *get_cif_cif10_output_format(int index);
int get_cif_cif10_output_format_size(void);

struct v4l2_fmtdesc *get_cif_cif10_output_format_desc(int index);
int get_cif_cif10_output_format_desc_size(void);

/*Clean code starts from here*************************************************/

struct cif_cif10_device *cif_cif10_create(
	CIF_CIF10_PLTFRM_DEVICE pdev,
	void (*sof_event)(__u32 frame_sequence),
	void (*requeue_bufs)(void),
	struct pltfrm_soc_cfg *soc_cfg);

void cif_cif10_destroy(
	struct cif_cif10_device *dev);

int cif_cif10_init(
	struct cif_cif10_device *dev);

int cif_cif10_release(
	struct cif_cif10_device *dev);

int cif_cif10_streamon(
	struct cif_cif10_device *dev);

int cif_cif10_streamoff(
	struct cif_cif10_device *dev);

int cif_cif10_s_input(
	struct cif_cif10_device *dev,
	enum cif_cif10_inp inp);

int cif_cif10_s_fmt(
	struct cif_cif10_device *dev,
	struct cif_cif10_strm_fmt *strm_fmt,
	u32 stride);

int cif_cif10_resume(
	struct cif_cif10_device *dev);

int cif_cif10_suspend(
	struct cif_cif10_device *dev);

int cif_cif10_qbuf(
	struct cif_cif10_device *dev,
	struct cif_cif10_buffer *buf);

int cif_cif10_reqbufs(
	struct cif_cif10_device *dev,
	struct v4l2_requestbuffers *req);

int cif_cif10_get_target_frm_size(
	struct cif_cif10_device *dev,
	u32 *target_width,
	u32 *target_height);

int cif_cif10_calc_cif_cropping(
	struct cif_cif10_device *dev,
	u32 *width,
	u32 *height,
	u32 *h_offs,
	u32 *v_offs);

const char *cif_cif10_g_input_name(
	struct cif_cif10_device *dev,
	enum cif_cif10_inp inp);

int cif_cif10_calc_min_out_buff_size(
	struct cif_cif10_device *dev,
	u32 *size);

int cif_cif10_s_ctrl(
	struct cif_cif10_device *dev,
	const enum cif_cif10_cid id,
	int val);

void cif_cif10_dbgfs_fill_sensor_aec_para(
	struct cif_cif10_device *cif_cif10_dev,
	s32 exp_time,
	u16 gain);

u32 cif_cif10_calc_llength(
	u32 width,
	u32 stride,
	enum cif_cif10_pix_fmt pix_fmt);
#endif
