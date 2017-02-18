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

#include <linux/videodev2.h>
#include <media/videobuf-dma-contig.h>
#include "cif_cif10_regs.h"
#include "cif_cif10.h"
#include <linux/pm_runtime.h>
/*
#define MEASURE_VERTICAL_BLANKING
*/
#define CIF_F0_READY        (0x01 << 0)
#define CIF_F1_READY        (0x01 << 1)

#define PAL_HEIGHT          (576)
#define NTSC_HEIGHT         (480)
#define CIF_CROP_X_MASK     (0xFFF)
#define CIF_CROP_START_X(x) ((x) & CIF_CROP_X_MASK)

static void init_output_formats(void);

static struct v4l2_fmtdesc output_formats[MAX_NB_FORMATS];

static struct cif_cif10_fmt cif_cif10_output_format[] = {
/* ************* YUV422 ************* */
{
	.name		= "YUV422-Interleaved",
	.fourcc	= V4L2_PIX_FMT_YUYV,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YUV422-Interleaved",
	.fourcc	= V4L2_PIX_FMT_YUYV,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YVU422-Interleaved",
	.fourcc	= V4L2_PIX_FMT_UYVY,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YUV422-Planar",
	.fourcc	= V4L2_PIX_FMT_YUV422P,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YUV422-Semi-Planar",
	.fourcc	= V4L2_PIX_FMT_NV16,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
/* ************* YUV420 ************* */
{
	.name		= "YUV420-Planar",
	.fourcc	= V4L2_PIX_FMT_YUV420,
	.flags	= 0,
	.depth	= 12,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YUV420-Planar",
	.fourcc	= V4L2_PIX_FMT_YUV420,
	.flags	= 0,
	.depth	= 12,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YVU420-Planar",
	.fourcc	= V4L2_PIX_FMT_YVU420,
	.flags	= 0,
	.depth	= 12,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YUV420-Semi-Planar",
	.fourcc	= V4L2_PIX_FMT_NV12,
	.flags	= 0,
	.depth	= 12,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YVU420-Semi-Planar",
	.fourcc	= V4L2_PIX_FMT_NV21,
	.flags	= 0,
	.depth	= 12,
	.rotation = false,
	.overlay = false,
},
/* ************* YUV400/Y8 ************* */
{
	.name		= "YVU400-Grey-Planar",
	.fourcc	= V4L2_PIX_FMT_GREY,
	.flags	= 0,
	.depth	= 8,
	.rotation = false,
	.overlay = false,
},
/* ************ Y10*********** */
{
	.name		= "Y10",
	.fourcc = V4L2_PIX_FMT_Y10,
	.flags	= 0,
	.depth	= 10,
	.rotation = false,
	.overlay = false,
},
/* ************ Y12*********** */
{
	.name		= "Y12",
	.fourcc = V4L2_PIX_FMT_Y12,
	.flags	= 0,
	.depth	= 12,
	.rotation = false,
	.overlay = false,
},
/* ************* YUV444 ************* */
{
	.name		= "YVU444-Planar",
	.fourcc	= V4L2_PIX_FMT_YUV444,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YVU444-Semi-Planar",
	.fourcc	= V4L2_PIX_FMT_NV24,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
/* ************* RAW8 ************* */
{
	.name	= "SGBRG8",
	.fourcc	= V4L2_PIX_FMT_SGBRG8,
	.flags	= 0,
	.depth	= 8,
	.rotation = false,
	.overlay = false,
},
/* ************* RAW10 ************* */
{
	.name	= "SGBRG10",
	.fourcc	= V4L2_PIX_FMT_SGBRG10,
	.flags	= 0,
	.depth	= 10,
	.rotation = false,
	.overlay = false,
},
/* ************* RAW12 ************* */
{
	.name	= "SGBRG12",
	.fourcc	= V4L2_PIX_FMT_SGBRG12,
	.flags	= 0,
	.depth	= 12,
	.rotation = false,
	.overlay = false,
}
};

/**Structures and Types*******************************************************/

static const char *cif_cif10_img_src_state_string(
	enum cif_cif10_img_src_state state)
{
	switch (state) {
	case CIF_CIF10_IMG_SRC_STATE_OFF:
		return "OFF";
	case CIF_CIF10_IMG_SRC_STATE_SW_STNDBY:
		return "SW_STNDBY";
	case CIF_CIF10_IMG_SRC_STATE_STREAMING:
		return "STREAMING";
	default:
		return "UNKNOWN/UNSUPPORTED";
	}
}

static const char *cif_cif10_state_string(
	enum cif_cif10_state state)
{
	switch (state) {
	case CIF_CIF10_STATE_DISABLED:
		return "DISABLED";
	case CIF_CIF10_STATE_INACTIVE:
		return "INACTIVE";
	case CIF_CIF10_STATE_READY:
		return "READY";
	case CIF_CIF10_STATE_STREAMING:
		return "STREAMING";
	default:
		return "UNKNOWN/UNSUPPORTED";
	}
}

static const char *cif_cif10_pm_state_string(
	enum cif_cif10_pm_state pm_state)
{
	switch (pm_state) {
	case CIF_CIF10_PM_STATE_OFF:
		return "OFF";
	case CIF_CIF10_PM_STATE_SW_STNDBY:
		return "STANDBY";
	case CIF_CIF10_PM_STATE_SUSPENDED:
		return "SUSPENDED";
	case CIF_CIF10_PM_STATE_STREAMING:
		return "STREAMING";
	default:
		return "UNKNOWN/UNSUPPORTED";
	}
}

static const char *cif_cif10_pix_fmt_string(int pixfmt)
{
	switch (pixfmt) {
	case CIF_YUV400:
		return "YUV400";
	case CIF_YUV420I:
		return "YUV420I";
	case CIF_YUV420SP:
		return "YUV420SP";
	case CIF_YUV420P:
		return "YUV420P";
	case CIF_YVU420I:
		return "YVU420I";
	case CIF_YVU420SP:
		return "YVU420SP";
	case CIF_YVU420P:
		return "YVU420P";
	case CIF_YUV422I:
		return "YUV422I";
	case CIF_YUV422SP:
		return "YUV422SP";
	case CIF_YUV422P:
		return "YUV422P";
	case CIF_YVU422I:
		return "YVU422I";
	case CIF_YVU422SP:
		return "YVU422SP";
	case CIF_YVU422P:
		return "YVU422P";
	case CIF_YUV444I:
		return "YUV444I";
	case CIF_YUV444SP:
		return "YUV444SP";
	case CIF_YUV444P:
		return "YUV444P";
	case CIF_YVU444I:
		return "YVU444I";
	case CIF_YVU444SP:
		return "YVU444SP";
	case CIF_YVU444P:
		return "YVU444SP";
	case CIF_UYV400:
		return "UYV400";
	case CIF_UYV420I:
		return "UYV420I";
	case CIF_UYV420SP:
		return "UYV420SP";
	case CIF_UYV420P:
		return "UYV420P";
	case CIF_VYU420I:
		return "VYU420I";
	case CIF_VYU420SP:
		return "VYU420SP";
	case CIF_VYU420P:
		return "VYU420P";
	case CIF_UYV422I:
		return "UYV422I";
	case CIF_UYV422SP:
		return "UYV422I";
	case CIF_UYV422P:
		return "UYV422P";
	case CIF_VYU422I:
		return "VYU422I";
	case CIF_VYU422SP:
		return "VYU422SP";
	case CIF_VYU422P:
		return "VYU422P";
	case CIF_UYV444I:
		return "UYV444I";
	case CIF_UYV444SP:
		return "UYV444SP";
	case CIF_UYV444P:
		return "UYV444P";
	case CIF_VYU444I:
		return "VYU444I";
	case CIF_VYU444SP:
		return "VYU444SP";
	case CIF_VYU444P:
		return "VYU444P";
	case CIF_BAYER_SBGGR8:
		return "BAYER BGGR8";
	case CIF_BAYER_SGBRG8:
		return "BAYER GBRG8";
	case CIF_BAYER_SGRBG8:
		return "BAYER GRBG8";
	case CIF_BAYER_SRGGB8:
		return "BAYER RGGB8";
	case CIF_BAYER_SBGGR10:
		return "BAYER BGGR10";
	case CIF_BAYER_SGBRG10:
		return "BAYER GBRG10";
	case CIF_BAYER_SGRBG10:
		return "BAYER GRBG10";
	case CIF_BAYER_SRGGB10:
		return "BAYER RGGB10";
	case CIF_BAYER_SBGGR12:
		return "BAYER BGGR12";
	case CIF_BAYER_SGBRG12:
		return "BAYER GBRG12";
	case CIF_BAYER_SGRBG12:
		return "BAYER GRBG12";
	case CIF_BAYER_SRGGB12:
		return "BAYER RGGB12";
	case CIF_Y10:
		return "Y10";
	case CIF_Y12:
		return "Y12";
	default:
		return "unknown/unsupported";
	}
}

u32 cif_cif10_calc_llength(
	u32 width,
	u32 stride,
	enum cif_cif10_pix_fmt pix_fmt)
{
	if (stride == 0)
		return width;

	if (CIF_CIF10_PIX_FMT_IS_YUV(pix_fmt)) {
		u32 num_cplanes =
			CIF_CIF10_PIX_FMT_YUV_GET_NUM_CPLANES(pix_fmt);
		if (num_cplanes == 0)
			return 8 * stride / CIF_CIF10_PIX_FMT_GET_BPP(pix_fmt);
		else
			return stride;
	} else if (CIF_CIF10_PIX_FMT_IS_RGB(pix_fmt)) {
		return 8 * stride / CIF_CIF10_PIX_FMT_GET_BPP(pix_fmt);
	} else {
		return width;
	}
}

static int cif_cif10_set_pm_state(
	struct cif_cif10_device *dev,
	enum cif_cif10_pm_state pm_state)
{
	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"%s -> %s\n",
		cif_cif10_pm_state_string(dev->pm_state),
		cif_cif10_pm_state_string(pm_state));

	if (dev->pm_state == pm_state)
		return 0;

	switch (pm_state) {
	case CIF_CIF10_PM_STATE_OFF:
	case CIF_CIF10_PM_STATE_SUSPENDED:
		if (
			(dev->pm_state == CIF_CIF10_PM_STATE_SW_STNDBY) ||
				(dev->pm_state ==
					CIF_CIF10_PM_STATE_STREAMING)) {
			pm_runtime_put_sync(dev->dev);
		}
		break;
	case CIF_CIF10_PM_STATE_SW_STNDBY:
	case CIF_CIF10_PM_STATE_STREAMING:
		if (
			(dev->pm_state == CIF_CIF10_PM_STATE_OFF) ||
				(dev->pm_state ==
					CIF_CIF10_PM_STATE_SUSPENDED)) {
			pm_runtime_get_sync(dev->dev);
		}
		break;
	default:
		cif_cif10_pltfrm_pr_err(
			dev->dev,
			"unknown or unsupported PM state %d\n", pm_state);
		return -EINVAL;
	}

	dev->pm_state = pm_state;

	return 0;
}

static int cif_cif10_img_src_set_state(
	struct cif_cif10_device *dev,
	enum cif_cif10_img_src_state state)
{
	int ret = 0;

	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"%s -> %s\n",
		cif_cif10_img_src_state_string(dev->img_src_state),
		cif_cif10_img_src_state_string(state));

	if (dev->img_src_state == state)
		return 0;

	switch (state) {
	case CIF_CIF10_IMG_SRC_STATE_OFF:
		ret = cif_cif10_img_src_s_power(dev->img_src, false);
		break;
	case CIF_CIF10_IMG_SRC_STATE_SW_STNDBY:
		if (dev->img_src_state == CIF_CIF10_IMG_SRC_STATE_STREAMING) {
			ret = cif_cif10_img_src_s_streaming(
				dev->img_src, false);
		} else {
			ret = cif_cif10_img_src_s_power(dev->img_src, true);
		}
		break;
	case CIF_CIF10_IMG_SRC_STATE_STREAMING:
		if (dev->config.flash_mode !=
			CIF_CIF10_FLASH_MODE_OFF)
			cif_cif10_img_src_s_ctrl(
				dev->img_src,
				CIF_CIF10_CID_FLASH_MODE,
				dev->config.flash_mode);
		ret = cif_cif10_img_src_s_streaming(dev->img_src, true);
		break;
	default:
		break;
	}

	if (
		(dev->config.flash_mode != CIF_CIF10_FLASH_MODE_OFF) &&
			(IS_ERR_VALUE(ret) ||
			 (state == CIF_CIF10_IMG_SRC_STATE_OFF)))
		cif_cif10_img_src_s_ctrl(
			dev->img_src,
			CIF_CIF10_CID_FLASH_MODE,
			CIF_CIF10_FLASH_MODE_OFF);

	if (!IS_ERR_VALUE(ret))
		dev->img_src_state = state;
	else
		cif_cif10_pltfrm_pr_err(
			dev->dev,
			"failed with err %d\n", ret);

	return ret;
}

static int cif_cif10_img_srcs_init(
	struct cif_cif10_device *cif_cif10_dev)
{
	int ret = 0;

	memset(cif_cif10_dev->img_src_array,
	       0x00,
	       sizeof(cif_cif10_dev->img_src_array));

	cif_cif10_dev->img_src_cnt =
			cif_cif10_pltfrm_get_img_src_device(
				cif_cif10_dev->dev,
				cif_cif10_dev->img_src_array,
				CIF_CIF10_NUM_INPUTS
			);

	if (cif_cif10_dev->img_src_cnt > 0) {
		cif_cif10_pltfrm_pr_info(
			cif_cif10_dev->dev,
			"cif_cif10_pltfrm_get_img_src_device success %d\n",
			cif_cif10_dev->img_src_cnt);
		return 0;
	}

	cif_cif10_dev->img_src_cnt = 0;
	ret = -EFAULT;

	cif_cif10_pltfrm_pr_err(
		cif_cif10_dev->dev,
		"failed with error %d\n",
		ret);
	return ret;
}

static int cif_cif10_img_src_select_strm_fmt(
	struct cif_cif10_device *dev)
{
	int ret = 0;
	u32 index;
	struct cif_cif10_strm_fmt_desc strm_fmt_desc;
	struct cif_cif10_strm_fmt request_strm_fmt;
	struct cif_cif10_frm_intrvl *min_intrvl;
	struct cif_cif10_frm_intrvl *frm_intrvl;
	bool matching_format_found = false;
	bool better_match = false;
	u32 target_width, target_height;
	u32 img_src_width, img_src_height;
	u32 best_diff = ~0;
	int vblanking;

	if (IS_ERR_OR_NULL(dev->img_src)) {
		cif_cif10_pltfrm_pr_err(
			dev->dev,
			"no image source selected as input (call s_input first)\n");
		ret = -EFAULT;
		goto err;
	}

	ret = cif_cif10_img_src_set_state(
			dev,
			CIF_CIF10_IMG_SRC_STATE_SW_STNDBY);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = cif_cif10_get_target_frm_size(
			dev,
			&target_width,
			&target_height);
	if (IS_ERR_VALUE(ret))
		goto err;

	/* find the best matching format from the image source */
	/* TODO: frame interval and pixel format handling */
	for (index = 0;; index++) {
		if (IS_ERR_VALUE(cif_cif10_img_src_enum_strm_fmts(
						dev->img_src,
						index,
						&strm_fmt_desc)))
			break;

		if (!strm_fmt_desc.discrete_frmsize) {
			if (strm_fmt_desc.min_frmsize.width >= target_width)
				img_src_width = strm_fmt_desc.min_frmsize.width;
			else if (strm_fmt_desc.max_frmsize.width >=
				target_width)
				img_src_width = target_width;
			else
				img_src_width = strm_fmt_desc.max_frmsize.width;

			if (strm_fmt_desc.min_frmsize.height >= target_height)
				img_src_height =
					strm_fmt_desc.min_frmsize.height;
			else if (strm_fmt_desc.max_frmsize.height >=
				target_height)
				img_src_height = target_height;
			else
				img_src_height =
					strm_fmt_desc.max_frmsize.height;
		} else {
			img_src_width = strm_fmt_desc.min_frmsize.width;
			img_src_height = strm_fmt_desc.min_frmsize.height;
		}

		if ((img_src_width >= target_width) &&
		    (img_src_height >= target_height)) {
			u32 diff = abs(
				target_height -
				(target_width * img_src_height
				/
				img_src_width));

			if (matching_format_found) {
				min_intrvl = &strm_fmt_desc.min_intrvl;
				frm_intrvl = &request_strm_fmt.frm_intrvl;
				if (((img_src_width >=
					request_strm_fmt.frm_fmt.width) &&
					(img_src_height >
					request_strm_fmt.frm_fmt.height)))
					/* for image capturing we try to
						maximize the size */
					better_match = true;
				else if (((min_intrvl->denominator /
							min_intrvl->numerator) >
					(frm_intrvl->denominator /
							frm_intrvl->numerator)))
					/* maximize fps */
					better_match = true;
				else if (
					((min_intrvl->denominator /
						min_intrvl->numerator) ==
					(frm_intrvl->denominator /
						frm_intrvl->numerator)) &&
					(diff < best_diff))
					/* chose better aspect ratio
						match if fps equal */
					better_match = true;
				else
					better_match = false;
			}

			if (!matching_format_found || better_match) {
				request_strm_fmt.frm_fmt.width =
					strm_fmt_desc.min_frmsize.width;
				request_strm_fmt.frm_fmt.height =
					strm_fmt_desc.min_frmsize.height;
				request_strm_fmt.frm_fmt.std_id =
					strm_fmt_desc.std_id;
				request_strm_fmt.frm_fmt.pix_fmt =
					strm_fmt_desc.pix_fmt;
				request_strm_fmt.frm_intrvl.numerator =
					strm_fmt_desc.min_intrvl.numerator;
				request_strm_fmt.frm_intrvl.denominator =
					strm_fmt_desc.min_intrvl.denominator;
				request_strm_fmt.frm_fmt.defrect =
					strm_fmt_desc.defrect;
				best_diff = diff;
				matching_format_found = true;
			}
		}
	}

	if (!matching_format_found) {
		cif_cif10_pltfrm_pr_err(
			dev->dev,
			"no matching image source format (%dx%d) found\n",
			target_width, target_height);
		ret = -EINVAL;
		goto err;
	}

	cif_cif10_pltfrm_pr_info(
		dev->dev,
		"requesting format %s %dx%d(%d,%d,%d,%d)@%d/%dfps from %s\n",
		cif_cif10_pix_fmt_string(request_strm_fmt.frm_fmt.pix_fmt),
		request_strm_fmt.frm_fmt.width,
		request_strm_fmt.frm_fmt.height,
		request_strm_fmt.frm_fmt.defrect.left,
		request_strm_fmt.frm_fmt.defrect.top,
		request_strm_fmt.frm_fmt.defrect.width,
		request_strm_fmt.frm_fmt.defrect.height,
		request_strm_fmt.frm_intrvl.denominator,
		request_strm_fmt.frm_intrvl.numerator,
		cif_cif10_img_src_g_name(dev->img_src));

	ret = cif_cif10_img_src_s_strm_fmt(dev->img_src, &request_strm_fmt);
	if (IS_ERR_VALUE(ret))
		goto err;

	dev->config.img_src_output = request_strm_fmt;

	ret = cif_cif10_img_src_g_ctrl(
			dev->img_src,
			CIF_CIF10_CID_VBLANKING,
			&vblanking);
	if (IS_ERR_VALUE(ret)) {
		cif_cif10_pltfrm_pr_dbg(
			dev->dev,
			"get vblanking failed: %d\n", ret);
			vblanking = 0;
	}

	return 0;
err:
	cif_cif10_pltfrm_pr_err(
		dev->dev,
		"failed with err %d\n",
		ret);
	return ret;
}

static int cif_cif10_config_img_src(
	struct cif_cif10_device *dev)
{
	int ret = 0;

	cif_cif10_pltfrm_pr_dbg(dev->dev, "\n");

	ret = cif_cif10_img_src_set_state(
		dev,
		CIF_CIF10_IMG_SRC_STATE_SW_STNDBY);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = cif_cif10_pltfrm_g_interface_config(
			dev->img_src,
			&dev->config.cam_itf);
	if (IS_ERR_VALUE(ret))
		goto err;

	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"cam_itf: (type: 0x%x, dphy: %d, vc: %d, nb_lanes: %d, bitrate: %d)",
		dev->config.cam_itf.type,
		dev->config.cam_itf.cfg.mipi.dphy_index,
		dev->config.cam_itf.cfg.mipi.vc,
		dev->config.cam_itf.cfg.mipi.nb_lanes,
		dev->config.cam_itf.cfg.mipi.bit_rate);
	return 0;
err:
	cif_cif10_pltfrm_pr_err(
		dev->dev,
		"failed with error %d\n",
		ret);
	return ret;
}

static void cif_cif10_next_buff(
	struct cif_cif10_device *dev)
{
	cif_cif10_pltfrm_pr_dbg(NULL, "\n");

	if (
		!list_empty(&dev->stream.buf_queue) &&
		!dev->stream.stop) {
		if (dev->stream.curr_buf != NULL) {
			pr_info("stream current buffer not null\n");
			BUG();
		}
		dev->stream.curr_buf =
			list_first_entry(
					&dev->stream.buf_queue,
					struct videobuf_buffer, queue);
		list_del_init(&dev->stream.curr_buf->queue);
		dev->stream.curr_buf->state = VIDEOBUF_ACTIVE;
	} else {
		pr_info("queue buf is empty or stream already stop\n");
	}
}

static void cif_cif10_config_for(struct cif_cif10_device *cif_cif10_dev)
{
	unsigned int cif_fmt_val;
	struct cif_cif10_frm_fmt *input_fmt;
	struct cif_cif10_frm_fmt *output_fmt;
	struct pltfrm_cam_itf *cam_itf;

	if (cif_cif10_dev == NULL)
		return;

	input_fmt =
		&(cif_cif10_dev->config.img_src_output.frm_fmt);
	output_fmt =
		&(cif_cif10_dev->config.output);
	cam_itf = &cif_cif10_dev->config.cam_itf;
	if (PLTFRM_CAM_ITF_IS_BT656(cam_itf->type)) {
		if (input_fmt->height == NTSC_HEIGHT) {
			cif_fmt_val = INPUT_MODE_NTSC;
		} else if (input_fmt->height == PAL_HEIGHT) {
			cif_fmt_val = INPUT_MODE_PAL;
		} else {
			if (PLTFRM_CAM_ITF_IS_BT656_MIX(cam_itf->type)) {
				cif_fmt_val = INPUT_MODE_YUV |
					      VSY_LOW_ACTIVE |
					      HSY_LOW_ACTIVE;
			}
		}
	} else if (PLTFRM_CAM_ITF_IS_BT601(cam_itf->type)) {
		if (CIF_CIF10_PIX_FMT_IS_RAW_BAYER(input_fmt->pix_fmt)) {
			if (PLTFRM_CAM_ITF_DVP_BW(cam_itf->type) == 8)
				cif_fmt_val = INPUT_MODE_RAW |
					      RAW_DATA_WIDTH_8;
			else if (PLTFRM_CAM_ITF_DVP_BW(cam_itf->type) == 10)
				cif_fmt_val = INPUT_MODE_RAW |
					      RAW_DATA_WIDTH_10;
			else
				cif_fmt_val = INPUT_MODE_RAW |
					      RAW_DATA_WIDTH_12;
		} else if (CIF_CIF10_PIX_FMT_IS_JPEG(input_fmt->pix_fmt)) {
			cif_fmt_val = INPUT_MODE_JPEG;
		} else {
			cif_fmt_val = INPUT_MODE_YUV |
				      YUV_INPUT_422 |
				      INPUT_420_ORDER_EVEN |
				      OUTPUT_420_ORDER_EVEN;
			switch (input_fmt->pix_fmt) {
			case CIF_UYV422I:
				cif_fmt_val = YUV_INPUT_ORDER_UYVY(cif_fmt_val);
				break;
			case CIF_YUV422I:
				cif_fmt_val = YUV_INPUT_ORDER_YUYV(cif_fmt_val);
				break;
			case CIF_YVU422I:
				cif_fmt_val = YUV_INPUT_ORDER_YVYU(cif_fmt_val);
				break;
			case CIF_VYU422I:
				cif_fmt_val = YUV_INPUT_ORDER_VYUY(cif_fmt_val);
				break;
			default:
				cif_fmt_val = YUV_INPUT_ORDER_YUYV(cif_fmt_val);
				break;
			}
		}

		/* config vsync/hsync */
		if (cam_itf->cfg.dvp.hsync)
			cif_fmt_val |= HSY_LOW_ACTIVE;
		else
			cif_fmt_val &= ~HSY_LOW_ACTIVE;

		if (cam_itf->cfg.dvp.vsync)
			cif_fmt_val &= ~VSY_HIGH_ACTIVE;
		else
			cif_fmt_val |= VSY_HIGH_ACTIVE;
	}

	switch (output_fmt->pix_fmt) {
	case CIF_YUV422SP:
		cif_fmt_val &= ~YUV_OUTPUT_422;
		cif_fmt_val &= ~UV_STORAGE_ORDER_UVUV;
		break;
	case CIF_YVU422SP:
		cif_fmt_val &= ~YUV_OUTPUT_422;
		cif_fmt_val |= UV_STORAGE_ORDER_VUVU;
		break;
	case CIF_YUV420SP:
		cif_fmt_val |= YUV_OUTPUT_420;
		cif_fmt_val &= ~UV_STORAGE_ORDER_UVUV;
		break;
	case CIF_YVU420SP:
		cif_fmt_val |= YUV_OUTPUT_420;
		cif_fmt_val |= UV_STORAGE_ORDER_VUVU;
		break;
	default:
		cif_fmt_val |= YUV_OUTPUT_422;
		break;
	}

	cif_iowrite32OR(
		cif_fmt_val,
		cif_cif10_dev->config.base_addr +
			CIF_CIF_FOR);
}

static void cif_cif10_config_frm_addr(
		struct cif_cif10_device *cif_cif10_dev)
{
	unsigned int cif_frm0_addr_y, cif_frm0_addr_uv;
	unsigned int cif_frm1_addr_y, cif_frm1_addr_uv;
	unsigned int cif_ctrl_val;
	struct cif_cif10_stream *stream;
	struct cif_cif10_frm_fmt *output_fmt;
	struct videobuf_buffer *curr_buf, *next_buf;

	if (cif_cif10_dev == NULL)
		return;

	stream = &cif_cif10_dev->stream;
	cif_ctrl_val =
		cif_ioread32(cif_cif10_dev->config.base_addr +
			     CIF_CIF_CTRL);
	output_fmt =
		&(cif_cif10_dev->config.output);

	cif_cif10_next_buff(cif_cif10_dev);
	curr_buf = stream->curr_buf;
	if (curr_buf == NULL)
		return;

	if (PLTFRM_CAM_ITF_IS_BT601_FIELD(
				cif_cif10_dev->config.cam_itf.type)) {
		if (curr_buf->memory == V4L2_MEMORY_USERPTR)
			cif_frm0_addr_y =
				videobuf_to_dma_contig(curr_buf);
		else if (curr_buf->memory == V4L2_MEMORY_MMAP)
			cif_frm0_addr_y = curr_buf->boff;

		cif_frm0_addr_uv = cif_frm0_addr_y +
				   (output_fmt->defrect.width *
				   output_fmt->defrect.height);
		cif_frm1_addr_y  = cif_frm0_addr_y +
				   output_fmt->defrect.width;
		cif_frm1_addr_uv = cif_frm0_addr_uv +
				   output_fmt->defrect.width;
	} else {
		if (curr_buf->memory == V4L2_MEMORY_USERPTR)
			cif_frm0_addr_y =
				videobuf_to_dma_contig(curr_buf);
		else if (curr_buf->memory == V4L2_MEMORY_MMAP)
			cif_frm0_addr_y = curr_buf->boff;

		cif_frm0_addr_uv = cif_frm0_addr_y +
				   (output_fmt->defrect.width *
				   output_fmt->defrect.height);
		if (cif_ctrl_val & MODE_PINGPONG) {
			dev_info(cif_cif10_dev->dev, "cif work on pingpong\n");

			if (!list_empty(&stream->buf_queue)) {
				stream->next_buf =
					list_first_entry(&stream->buf_queue,
							 struct videobuf_buffer,
							 queue);
				next_buf = stream->next_buf;
				list_del_init(&next_buf->queue);
			} else {
				dev_err(cif_cif10_dev->dev,
					"no buffer\n");
				cif_frm1_addr_y = cif_frm0_addr_y;
				cif_frm1_addr_uv = cif_frm0_addr_uv;
				return;
			}
			next_buf->state = VIDEOBUF_ACTIVE;
			if (next_buf->memory == V4L2_MEMORY_USERPTR)
				cif_frm1_addr_y =
					videobuf_to_dma_contig(next_buf);
			else if (next_buf->memory == V4L2_MEMORY_MMAP)
				cif_frm1_addr_y = next_buf->boff;

			cif_frm1_addr_uv = cif_frm1_addr_y +
					   (output_fmt->defrect.width *
					    output_fmt->defrect.height);
		}
	}

	cif_iowrite32(cif_frm0_addr_y,
		      cif_cif10_dev->config.base_addr +
		      CIF_CIF_FRM0_ADDR_Y);
	cif_iowrite32(cif_frm0_addr_uv,
		      cif_cif10_dev->config.base_addr +
		      CIF_CIF_FRM0_ADDR_UV);

	if (cif_ctrl_val & MODE_PINGPONG) {
		cif_iowrite32(cif_frm1_addr_y,
			      cif_cif10_dev->config.base_addr +
			      CIF_CIF_FRM1_ADDR_Y);
		cif_iowrite32(cif_frm1_addr_uv,
			      cif_cif10_dev->config.base_addr +
			      CIF_CIF_FRM1_ADDR_UV);
	}
}

static int cif_cif10_config_cif_phy(
	struct cif_cif10_device *dev)
{
	unsigned int cif_ctrl_val, cif_width;
	unsigned int cif_fs = 0, cif_crop = 0;
	struct cif_cif10_frm_fmt *output;
	struct cif_cif10_frm_fmt *input;

	if (dev == NULL)
		return -1;

	input = &dev->config.img_src_output.frm_fmt;
	output = &dev->config.output;

	cif_ctrl_val = AXI_BURST_16 | DISABLE_CAPTURE;
	if (PLTFRM_CAM_ITF_IS_BT601_FIELD(
				dev->config.cam_itf.type))
		cif_ctrl_val |=  MODE_PINGPONG;
	else
#if defined(CONFIG_CIF_PINGPONG_MODE)
		cif_ctrl_val |=  MODE_PINGPONG;
#else
		cif_ctrl_val |=  MODE_ONEFRAME;
#endif

	cif_iowrite32(cif_ctrl_val,
		      dev->config.base_addr + CIF_CIF_CTRL);

	cif_cif10_config_for(dev);
	cif_cif10_config_frm_addr(dev);

	cif_iowrite32(INTSTAT_CLS,
		      dev->config.base_addr +
		      CIF_CIF_INTSTAT);

	cif_crop = input->defrect.left +
		   (input->defrect.top << 16);

	if (cif_ctrl_val & MODE_PINGPONG) {
		if (PLTFRM_CAM_ITF_IS_BT601_FIELD(
					dev->config.cam_itf.type)) {
			cif_fs  = ((output->defrect.height / 2) << 16) +
				  output->defrect.width;
			cif_width = output->defrect.width * 2;
		} else {
			cif_fs  = (output->defrect.height << 16) +
				  output->defrect.width;
			if (CIF_CIF10_PIX_FMT_IS_RAW_BAYER(input->pix_fmt))
				cif_width = output->defrect.width * 2;
			else
				cif_width = output->defrect.width;
		}
	} else {/* this is one frame mode */
		cif_fs  = output->defrect.width +
			  (output->defrect.height << 16);
		if (CIF_CIF10_PIX_FMT_IS_RAW_BAYER(input->pix_fmt))
			cif_width = output->defrect.width * 2;
		else
			cif_width = output->defrect.width;
	}

	cif_iowrite32(cif_crop,
		      dev->config.base_addr +
		      CIF_CIF_CROP);
	cif_iowrite32(cif_fs,
		      dev->config.base_addr +
		      CIF_CIF_SET_SIZE);
	cif_iowrite32(cif_width,
		      dev->config.base_addr +
		      CIF_CIF_VIR_LINE_WIDTH);
	cif_iowrite32(FRAME_STAT_CLS,
		      dev->config.base_addr +
		      CIF_CIF_FRAME_STATUS);
	/* MUST bypass scale */
	if (CIF_CIF10_PIX_FMT_IS_RAW_BAYER(input->pix_fmt))
		cif_iowrite32(ENABLE_RAW_16BIT_BYPASS,
			      dev->config.base_addr +
			      CIF_CIF_SCL_CTRL);
	else
		cif_iowrite32(ENABLE_YUV_16BIT_BYPASS,
			      dev->config.base_addr +
			      CIF_CIF_SCL_CTRL);

	return 0;
}

static int cif_cif10_config_mipi(
	struct cif_cif10_device *dev)
{
	int ret = 0;
	return ret;
}

static int cif_cif10_config_cif(
	struct cif_cif10_device *dev)
{
	int ret = 0;
	struct pltfrm_soc_cfg_para cfg_para;
	struct pltfrm_soc_init_para init_para;
	struct platform_device *pdev =
			container_of(dev->dev, struct platform_device, dev);

	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"config cif, img_src state = %s, PM state = %s, strm state = %s\n",
		cif_cif10_img_src_state_string(dev->img_src_state),
		cif_cif10_pm_state_string(dev->pm_state),
		cif_cif10_state_string(dev->stream.state));

	/* configure sensor */
	dev->config.cam_itf.cif_id = pdev->id;
	ret = cif_cif10_config_img_src(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = cif_cif10_set_pm_state(
				dev,
				CIF_CIF10_PM_STATE_SW_STNDBY);
	if (IS_ERR_VALUE(ret))
		goto err;

	if (PLTFRM_CAM_ITF_IS_MIPI(dev->config.cam_itf.type)) {
		ret = cif_cif10_config_mipi(dev);
		if (IS_ERR_VALUE(ret))
			goto err;
	}

	if (!IS_ERR_OR_NULL(dev->soc_cfg) &&
	    !IS_ERR_OR_NULL(dev->soc_cfg->soc_cfg)) {
		init_para.pdev = pdev;
		init_para.cam_itf =
			dev->config.cam_itf;
		cfg_para.cfg_para = &init_para;
		/* set cif clk parent and data source*/
		cfg_para.cmd = PLTFRM_CLKINIT;
		(dev->soc_cfg->soc_cfg)(&cfg_para);
	} else {
		cif_cif10_pltfrm_pr_err(
				dev->dev,
				"soc_cfg is null or soc_cfg->soc_cfg is null\n");
	}

	ret = cif_cif10_config_cif_phy(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_cif10_pltfrm_pr_err(
		dev->dev,
		"failed with error %d\n",
		ret);
	return ret;
}

static void cif_cif10_init_stream(
	struct cif_cif10_device *cif_cif10_dev)
{
	struct cif_cif10_stream *stream
				= &cif_cif10_dev->stream;

	INIT_LIST_HEAD(&stream->buf_queue);
	stream->next_buf = NULL;
	stream->curr_buf = NULL;
	stream->updt_cfg = false;
	stream->stop = false;
	stream->stall = false;

	cif_cif10_pltfrm_event_clear(cif_cif10_dev->dev, &stream->done);
	stream->state = CIF_CIF10_STATE_INACTIVE;
}

static void cif_cif10_requeue_bufs(
	struct cif_cif10_device *dev,
	struct cif_cif10_stream *stream)
{
	INIT_LIST_HEAD(&stream->buf_queue);
	stream->next_buf = NULL;
	stream->curr_buf = NULL;
	dev->requeue_bufs();
}

static int cif_cif10_stop(
	struct cif_cif10_device *cif_cif10_dev,
	bool stop)
{
	cif_cif10_pltfrm_pr_dbg(
		cif_cif10_dev->dev,
		"state = %s, img_src state = %s, stop = %d\n",
		cif_cif10_state_string(cif_cif10_dev->stream.state),
		cif_cif10_img_src_state_string(cif_cif10_dev->img_src_state),
		stop);

	if (!(cif_cif10_dev->stream.state == CIF_CIF10_STATE_STREAMING))
		return 0;

	if (cif_cif10_dev->stream.state ==
					CIF_CIF10_STATE_STREAMING) {
		/* we should not stop during an active transfer */
		cif_cif10_dev->stream.stop = true;
		(void)cif_cif10_pltfrm_event_wait_timeout(
						cif_cif10_dev->dev,
						&cif_cif10_dev->stream.done,
						cif_cif10_dev->stream.state !=
						CIF_CIF10_STATE_STREAMING,
						50000);
		cif_cif10_dev->stream.stop = false;
	}

	cancel_work_sync(&cif_cif10_dev->work);
	if (IS_ERR_VALUE(cif_cif10_img_src_set_state(
					cif_cif10_dev,
					CIF_CIF10_IMG_SRC_STATE_SW_STNDBY)))
		cif_cif10_pltfrm_pr_dbg(
				cif_cif10_dev->dev,
				"unable to put image source into standby\n");

	if (IS_ERR_VALUE(cif_cif10_set_pm_state(
					cif_cif10_dev,
					CIF_CIF10_PM_STATE_SW_STNDBY)))
		cif_cif10_pltfrm_pr_dbg(
				cif_cif10_dev->dev,
				"unable to put CIF into standby\n");

	if (cif_cif10_dev->stream.state == CIF_CIF10_STATE_STREAMING)
		cif_cif10_dev->stream.state = CIF_CIF10_STATE_READY;

	spin_lock(&cif_cif10_dev->vbq_lock);
	if (stop)
		cif_cif10_requeue_bufs(cif_cif10_dev, &cif_cif10_dev->stream);

	spin_unlock(&cif_cif10_dev->vbq_lock);

	return 0;
}

static int cif_cif10_start(
	struct cif_cif10_device *dev,
	bool start)
{
	unsigned int ret;

	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"stream state = %s, img_src state = %s starting\n",
		cif_cif10_state_string(dev->stream.state),
		cif_cif10_img_src_state_string(dev->img_src_state));

	if (dev->stream.state == CIF_CIF10_STATE_STREAMING)
		return 0;

	if (dev->stream.state < CIF_CIF10_STATE_READY) {
		cif_cif10_pltfrm_pr_err(
			NULL,
			"cannot start streaming, input source not ready\n");
		ret = -EFAULT;
		goto err;
	}

	ret = cif_cif10_set_pm_state(
			dev,
			CIF_CIF10_PM_STATE_STREAMING);
	if (IS_ERR_VALUE(ret))
		goto err;
	/* capture complete interrupt enable */
	cif_iowrite32(FRAME_END_EN/* | PST_INF_FRAME_END_EN */,
		      dev->config.base_addr + CIF_CIF_INTEN);
	cif_iowrite32OR(ENABLE_CAPTURE,
			dev->config.base_addr + CIF_CIF_CTRL);
	dev->stream.state = CIF_CIF10_STATE_STREAMING;

	ret = cif_cif10_img_src_set_state(
			dev,
			CIF_CIF10_IMG_SRC_STATE_STREAMING);
	if (IS_ERR_VALUE(ret))
		goto err;

	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"stream state = %s, img_src state = %s\n",
		cif_cif10_state_string(dev->stream.state),
		cif_cif10_img_src_state_string(dev->img_src_state));
	return 0;
err:
	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"stream state = %s, img_src state = %s\n",
		cif_cif10_state_string(dev->stream.state),
		cif_cif10_img_src_state_string(dev->img_src_state));
	cif_cif10_pltfrm_pr_err(
		dev->dev,
		"failed with err %d\n",
		ret);
	return ret;
}

/**Public Functions***********************************************************/

int cif_cif10_streamon(
	struct cif_cif10_device *dev)
{
	int ret = 0;

	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"stream state = %s\n",
		cif_cif10_state_string(dev->stream.state));

	if (dev->stream.state == CIF_CIF10_STATE_STREAMING)
		return 0;

	if (dev->stream.state != CIF_CIF10_STATE_READY) {
		cif_cif10_pltfrm_pr_err(
			dev->dev,
			"cannot start streaming on, it's not enabled\n");
		ret = -EFAULT;
		goto err;
	}

	ret = cif_cif10_config_cif(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = cif_cif10_start(dev, 1);
	if (IS_ERR_VALUE(ret))
		goto err;

	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"stream state = %s\n",
		cif_cif10_state_string(dev->stream.state));

	return 0;
err:
	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"stream state = %s\n",
		cif_cif10_state_string(dev->stream.state));
	cif_cif10_pltfrm_pr_err(
		dev->dev,
		"failed with error %d\n",
		ret);
	return ret;
}

int cif_cif10_streamoff(
	struct cif_cif10_device *dev)
{
	int ret = 0;
	bool streamoff = 1;

	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		" state = %s streamoff = %d\n",
		cif_cif10_state_string(dev->stream.state),
		streamoff);

	if (
		dev->config.flash_mode != CIF_CIF10_FLASH_MODE_OFF &&
		(dev->stream.state == CIF_CIF10_STATE_INACTIVE))
		cif_cif10_img_src_s_ctrl(
			dev->img_src,
			CIF_CIF10_CID_FLASH_MODE,
			CIF_CIF10_FLASH_MODE_OFF);

	ret = cif_cif10_stop(dev, streamoff);
	if (IS_ERR_VALUE(ret))
		goto err;

	if (
		(streamoff) &&
			(dev->stream.state == CIF_CIF10_STATE_READY))
		dev->stream.state = CIF_CIF10_STATE_INACTIVE;

	return 0;
err:
	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"state = %s\n",
		cif_cif10_state_string(dev->stream.state));
	cif_cif10_pltfrm_pr_err(
		dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

int cif_cif10_suspend(
	struct cif_cif10_device *dev)
{
	int ret = 0;

	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"stream state = %s\n",
		cif_cif10_state_string(dev->stream.state));

	if (
		(dev->pm_state == CIF_CIF10_PM_STATE_SUSPENDED) ||
			(dev->pm_state == CIF_CIF10_PM_STATE_OFF))
		return 0;

	dev->stream.saved_state = dev->stream.state;
	ret = cif_cif10_stop(dev, true);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = cif_cif10_set_pm_state(dev, CIF_CIF10_PM_STATE_SUSPENDED);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = cif_cif10_img_src_set_state(dev, CIF_CIF10_IMG_SRC_STATE_OFF);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_cif10_pltfrm_pr_err(
		dev->dev,
		"failed with error %d\n",
		ret);
	return ret;
}

int cif_cif10_resume(
	struct cif_cif10_device *dev)
{
	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"stream state = %s\n",
		cif_cif10_state_string(dev->stream.state));

	if (dev->stream.saved_state == CIF_CIF10_STATE_READY) {
		dev->stream.updt_cfg = true;
		dev->stream.state = CIF_CIF10_STATE_READY;
	}

	return cif_cif10_streamon(dev);
}

int cif_cif10_enum_fmt(
	struct cif_cif10_device *dev,
	struct cif_cif10_strm_fmt_desc *strm_fmt_desc)
{
	u32 index;
	int ret = 0;

	for (index = 0;; index++) {
		ret = cif_cif10_img_src_enum_strm_fmts(dev->img_src,
						       index,
						       strm_fmt_desc);
		if (IS_ERR_VALUE(ret))
			break;
	}

	return 0;
}

int cif_cif10_s_fmt(
	struct cif_cif10_device *dev,
	struct cif_cif10_strm_fmt *strm_fmt,
	u32 stride)
{
	int ret = 0;

	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"%s %dx%d@%d/%dfps, stride = %d\n",
		cif_cif10_pix_fmt_string(strm_fmt->frm_fmt.pix_fmt),
		strm_fmt->frm_fmt.width,
		strm_fmt->frm_fmt.height,
		strm_fmt->frm_intrvl.numerator,
		strm_fmt->frm_intrvl.denominator,
		stride/*,
		strm_fmt->frm_fmt.quantization*/);


	dev->config.output = strm_fmt->frm_fmt;
	dev->config.output.stride = stride;
	dev->config.output.llength = cif_cif10_calc_llength(
						strm_fmt->frm_fmt.width,
						stride,
						strm_fmt->frm_fmt.pix_fmt);
	dev->stream.state = CIF_CIF10_STATE_READY;

	ret = cif_cif10_img_src_select_strm_fmt(dev);
	if (IS_ERR_VALUE(ret)) {
		dev->stream.state = CIF_CIF10_STATE_INACTIVE;
		goto err;
	}


	return 0;
err:
	cif_cif10_pltfrm_pr_err(
		dev->dev,
		"failed with error %d\n",
		ret);
	return ret;
}

int cif_cif10_init(
	struct cif_cif10_device *dev)
{
	cif_cif10_pltfrm_pr_dbg(NULL, "\n");

	/* set default input, failure is not fatal here */
	if (dev->stream.state == CIF_CIF10_STATE_DISABLED)
		(void)cif_cif10_s_input(dev, 0);

	cif_cif10_init_stream(dev);

	return 0;
}

int cif_cif10_release(
	struct cif_cif10_device *dev)
{
	int ret;
	struct cif_cif10_stream *strm;

	cif_cif10_pltfrm_pr_dbg(NULL, "...\n");
	if (dev == NULL)
		return -1;

	strm = &dev->stream;
	if (strm->state == CIF_CIF10_STATE_DISABLED)
		return 0;

	kfree(strm->frame.frame_t);
	strm->frame.frame_t = NULL;
	strm->frame.cnt = 0;

	if (strm->state == CIF_CIF10_STATE_STREAMING) {
		cif_cif10_pltfrm_pr_warn(
			dev->dev,
			"CIF in streaming state, trying to stop it\n");
		ret = cif_cif10_stop(dev, true);
		if (IS_ERR_VALUE(ret))
			goto err;
	}
	strm->state = CIF_CIF10_STATE_DISABLED;

	if (IS_ERR_VALUE(cif_cif10_set_pm_state(
					dev,
					CIF_CIF10_PM_STATE_OFF)))
		cif_cif10_pltfrm_pr_warn(
				dev->dev,
				"CIF power off failed\n");
	if (dev->img_src != NULL) {
		if (IS_ERR_VALUE(cif_cif10_img_src_set_state(
						dev,
						CIF_CIF10_IMG_SRC_STATE_OFF)))
			cif_cif10_pltfrm_pr_warn(
				dev->dev,
				"image source power off failed\n");
		dev->img_src = NULL;
	}

	return 0;
err:
	cif_cif10_pltfrm_pr_err(
		dev->dev,
		"failed with error %d\n",
		ret);
	return ret;
}

struct cif_cif10_device *cif_cif10_create(
	CIF_CIF10_PLTFRM_DEVICE dev,
	void (*sof_event)(__u32 frame_sequence),
	void (*requeue_bufs)(void),
	struct pltfrm_soc_cfg *soc_cfg)
{
	int ret;
	struct cif_cif10_device *cif_cif10_dev;

	cif_cif10_pltfrm_pr_dbg(NULL, "\n");

	/* Allocate needed structures */
	cif_cif10_dev =
		kzalloc(sizeof(struct cif_cif10_device),
			GFP_KERNEL);
	if (NULL == cif_cif10_dev) {
		cif_cif10_pltfrm_pr_err(dev,
					"memory allocation failed\n");
		ret = -ENOMEM;
		goto err;
	}
	cif_cif10_dev->sof_event = sof_event;
	cif_cif10_dev->requeue_bufs = requeue_bufs;

	ret = cif_cif10_pltfrm_dev_init(cif_cif10_dev,
					&dev,
					&cif_cif10_dev->config.base_addr);
	if (IS_ERR_VALUE(ret))
		goto err;

	cif_cif10_pltfrm_soc_init(cif_cif10_dev, soc_cfg);

	ret = cif_cif10_img_srcs_init(cif_cif10_dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	(void)cif_cif10_init(cif_cif10_dev);
	cif_cif10_dev->pm_state = CIF_CIF10_PM_STATE_OFF;
	cif_cif10_dev->stream.state = CIF_CIF10_STATE_DISABLED;
	cif_cif10_pltfrm_event_init(cif_cif10_dev->dev,
				    &cif_cif10_dev->stream.done);

	/* TBD: clean this up */
	init_output_formats();

	return cif_cif10_dev;
err:
	cif_cif10_pltfrm_pr_err(
		NULL,
		"failed with error %d\n",
		ret);
	if (!IS_ERR_OR_NULL(cif_cif10_dev))
		kfree(cif_cif10_dev);
	return ERR_PTR(ret);
}

void cif_cif10_destroy(
	struct cif_cif10_device *dev)
{
	cif_cif10_pltfrm_pr_dbg(NULL, "\n");
	if (!IS_ERR_OR_NULL(dev))
		kfree(dev);
}

int cif_cif10_s_input(
	struct cif_cif10_device *dev,
	unsigned int input)
{
	int ret;

	if (input >= dev->img_src_cnt) {
		cif_cif10_pltfrm_pr_err(
			NULL,
			"invalid input %d\n",
			input);
		ret = -EINVAL;
		goto err;
	}

	dev->img_src = NULL;

	dev->img_src = dev->img_src_array[input];

	return 0;
err:
	cif_cif10_pltfrm_pr_err(
		NULL,
		"failed with error %d\n",
		ret);
	return ret;
}

const char *cif_cif10_g_input_name(
	struct cif_cif10_device *dev,
	unsigned int input_index)
{
	if (input_index >= dev->img_src_cnt) {
		cif_cif10_pltfrm_pr_dbg(
			NULL,
			"index %d out of bounds\n",
			input_index);
		return NULL;
	}

	return cif_cif10_img_src_g_name(
				dev->img_src_array[input_index]);
}

int cif_cif10_qbuf(
	struct cif_cif10_device *dev,
	struct cif_cif10_buffer *buf)
{
	cif_cif10_pltfrm_pr_dbg(dev->dev, ".\n");

	list_add_tail(&buf->queue, &dev->stream.buf_queue);

	return 0;
}

int cif_cif10_reqbufs(
	struct cif_cif10_device *cif_cif10_dev,
	struct v4l2_requestbuffers *req)
{
	struct cif_cif10_stream *strm;

	if (cif_cif10_dev == NULL)
		return -1;
	strm = &cif_cif10_dev->stream;

	if (strm->frame.frame_t != NULL) {
		kfree(strm->frame.frame_t);
		strm->frame.frame_t = NULL;
	}

	strm->frame.frame_t = kzalloc(
		req->count * sizeof(struct frame_timeinfo_s),
		GFP_KERNEL);
	strm->frame.cnt = req->count;
	if (strm->frame.frame_t == NULL) {
		cif_cif10_pltfrm_pr_err(
			cif_cif10_dev->dev,
			"kzalloc frame_timeinfo_s failed\n");
		return -ENOMEM;
	}

	return 0;
}

int cif_cif10_get_target_frm_size(
	struct cif_cif10_device *dev,
	u32 *target_width,
	u32 *target_height)
{
	if (dev->stream.state >= CIF_CIF10_STATE_READY) {
		*target_width = dev->config.output.width;
		*target_height = dev->config.output.height;
	} else {
		cif_cif10_pltfrm_pr_err(
			dev->dev,
			"cannot get target frame size, no ready\n");
		return -EFAULT;
	}
	return 0;
}

int cif_cif10_calc_cif_cropping(
	struct cif_cif10_device *dev,
	u32 *width,
	u32 *height,
	u32 *h_offs,
	u32 *v_offs)
{
	int ret = 0;
	u32 input_width;
	u32 input_height;
	u32 target_width;
	u32 target_height;

	if (IS_ERR_OR_NULL(dev)) {
		cif_cif10_pltfrm_pr_err(
			dev->dev,
			"no input selected for CIF\n");
		ret = -EFAULT;
		goto err;
	}

	input_width = dev->config.img_src_output.frm_fmt.defrect.width;
	input_height = dev->config.img_src_output.frm_fmt.defrect.height;

	ret = cif_cif10_get_target_frm_size(
			dev,
			&target_width,
			&target_height);
	if (IS_ERR_VALUE(ret))
		goto err;

	*width = input_width;
	*height = input_width * target_height / target_width;
	*v_offs = 0;
	*h_offs = 0;
	*height &= ~1;
	if (*height < input_height)
		/* vertical cropping */
		*v_offs = (input_height - *height) >> 1;
	else if (*height > input_height) {
		/* horizontal cropping */
		*height = input_height;
		*width = input_height * target_width / target_height;
		*width &= ~1;
		*h_offs = (input_width - *width) >> 1;
	}

	return 0;
err:
	cif_cif10_pltfrm_pr_err(
		dev->dev,
		"failed with err %d\n",
		ret);
	return ret;
}

int cif_cif10_calc_min_out_buff_size(
	struct cif_cif10_device *dev,
	u32 *size)
{
	int ret = 0;
	enum cif_cif10_pix_fmt pix_fmt;
	u32 llength;
	u32 height;
	u32 bpp;
	struct cif_cif10_stream *stream;
	struct cif_cif10_frm_fmt *frm_fmt;

	cif_cif10_pltfrm_pr_dbg(NULL, "\n");
	if (dev == NULL)
		return -1;

	frm_fmt = &dev->config.output;
	stream = &dev->stream;

	if (stream->state < CIF_CIF10_STATE_READY) {
		cif_cif10_pltfrm_pr_err(
			NULL,
			"cannot calculate buffer size, stream not ready\n");
		ret = -EINVAL;
		goto err;
	}
	pix_fmt = frm_fmt->pix_fmt;
	height  = frm_fmt->defrect.height;
	llength = frm_fmt->llength;

	if (CIF_CIF10_PIX_FMT_IS_RAW_BAYER(pix_fmt))
		/* RAW input is stored with 16BPP by MI */
		bpp = 16;
	else
		bpp = CIF_CIF10_PIX_FMT_GET_BPP(pix_fmt);
	*size = llength * height * bpp / 8;

	cif_cif10_pltfrm_pr_dbg(
		NULL,
		"calculated buffer size: %d\n",
		*size);

	return 0;
err:
	cif_cif10_pltfrm_pr_err(
		dev->dev,
		"failed with err %d\n",
		ret);
	return ret;
}

int cif_cif10_s_ctrl(
	struct cif_cif10_device *dev,
	const enum cif_cif10_cid id,
	int val)
{
	cif_cif10_pltfrm_pr_dbg(
			NULL,
			"id %d, val %d\n",
			id,
			val);

	switch (id) {
	case CIF_CIF10_CID_SUPER_IMPOSE:
		break;
	case CIF_CIF10_CID_IMAGE_EFFECT:
		break;
	case CIF_CIF10_CID_JPEG_QUALITY:
		break;
	case CIF_CIF10_CID_FLASH_MODE:
		if ((u32)val > CIF_CIF10_FLASH_MODE_TORCH) {
			cif_cif10_pltfrm_pr_err(
				NULL,
				"unknown/unsupported flash mode (%d)\n",
				val);
			return -EINVAL;
		}
		dev->config.flash_mode = val;
		cif_cif10_img_src_s_ctrl(
			dev->img_src,
			CIF_CIF10_CID_FLASH_MODE,
			dev->config.flash_mode);
		if (dev->config.flash_mode == CIF_CIF10_FLASH_MODE_FLASH) {
			do_gettimeofday(&dev->flash_t.mainflash_start_t);
			dev->flash_t.mainflash_start_t.tv_usec +=
				dev->flash_t.flash_turn_on_time;
			dev->flash_t.mainflash_end_t =
				dev->flash_t.mainflash_start_t;
			dev->flash_t.mainflash_end_t.tv_sec +=
				dev->flash_t.flash_on_timeout;
		} else if (dev->config.flash_mode ==
			   CIF_CIF10_FLASH_MODE_TORCH) {
			do_gettimeofday(&dev->flash_t.preflash_start_t);
			dev->flash_t.preflash_end_t =
				dev->flash_t.preflash_start_t;
			dev->flash_t.preflash_end_t.tv_sec = 0x00;
			dev->flash_t.preflash_end_t.tv_usec = 0x00;
		} else if (dev->config.flash_mode == CIF_CIF10_FLASH_MODE_OFF) {
			do_gettimeofday(&dev->flash_t.preflash_end_t);
			if (dev->flash_t.preflash_end_t.tv_sec*1000000 +
			    dev->flash_t.preflash_end_t.tv_usec <
			    dev->flash_t.mainflash_end_t.tv_sec*1000000 +
			    dev->flash_t.mainflash_end_t.tv_usec) {
				dev->flash_t.mainflash_end_t =
					dev->flash_t.preflash_end_t;
			}
		}
		break;
	case CIF_CIF10_CID_WB_TEMPERATURE:
	case CIF_CIF10_CID_ANALOG_GAIN:
	case CIF_CIF10_CID_EXPOSURE_TIME:
	case CIF_CIF10_CID_BLACK_LEVEL:
	case CIF_CIF10_CID_FOCUS_ABSOLUTE:
	case CIF_CIF10_CID_AUTO_N_PRESET_WHITE_BALANCE:
	case CIF_CIF10_CID_SCENE_MODE:
	case CIF_CIF10_CID_AUTO_FPS:
	case CIF_CIF10_CID_HFLIP:
	case CIF_CIF10_CID_VFLIP:
		return cif_cif10_img_src_s_ctrl(dev->img_src,
			id, val);
	default:
		cif_cif10_pltfrm_pr_err(
			dev->dev,
			"unknown/unsupported control %d\n", id);
		return -EINVAL;
	}

	return 0;
}

void init_output_formats(void)
{
	unsigned int i = 0;
	int xgold_num_format = 0;	/*RF*/

	xgold_num_format =
	    (sizeof(cif_cif10_output_format) / sizeof(struct cif_cif10_fmt));

	for (i = 0; i < xgold_num_format; i++) {
		struct v4l2_fmtdesc fmtdesc;

		memset(&fmtdesc, 0, sizeof(fmtdesc));
		fmtdesc.index = i;
		fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		strlcpy((&fmtdesc)->description,
			cif_cif10_output_format[(&fmtdesc)->index].name,
			sizeof((&fmtdesc)->description));
		(&fmtdesc)->pixelformat =
		    cif_cif10_output_format[(&fmtdesc)->index].fourcc;
		(&fmtdesc)->flags =
		    cif_cif10_output_format[(&fmtdesc)->index].flags;


		output_formats[i] = fmtdesc;
	}
}

int get_cif_cif10_output_format_size(void)
{
	return sizeof(cif_cif10_output_format) / sizeof(struct cif_cif10_fmt);
}

struct cif_cif10_fmt *get_cif_cif10_output_format(int index)
{
	struct cif_cif10_fmt *fmt = NULL;

	if ((index >= 0) && (index < get_cif_cif10_output_format_size()))
		fmt = &cif_cif10_output_format[index];

	return fmt;
}

struct v4l2_fmtdesc *get_cif_cif10_output_format_desc(int index)
{
	struct v4l2_fmtdesc *desc = NULL;

	if ((index >= 0) && (index < get_cif_cif10_output_format_desc_size()))
		desc = &output_formats[index];

	return desc;
}

int get_cif_cif10_output_format_desc_size(void)
{
	return ARRAY_SIZE(cif_cif10_output_format);
}

irqreturn_t cif_cif10_cifirq(int irq, void *data)
{
	short reset = 0, cif_err = 0, frm_flag;
	unsigned long y_addr, uv_addr;
	struct timeval tv;
	unsigned long long now, interval;
	unsigned int cifctrl, lastpix, lastline;
	unsigned long tmp_cif_frmst, reg_intstat;
	struct cif_cif10_device *cif_cif10_dev = data;
	struct cif_cif10_frm_fmt *frm_fmt = NULL;
	struct videobuf_buffer *curr_buf = NULL, *next_buf = NULL;
	void __iomem *base_addr =
		cif_cif10_dev->config.base_addr;
	static short cvbs_plug_out = 3;

	do_gettimeofday(&tv);
	now = tv.tv_sec*1000000 + tv.tv_usec;
	interval = now -
		cif_cif10_dev->irqinfo.cifirq_interval;
	cif_cif10_dev->irqinfo.cifirq_interval = now;

	frm_fmt =
		&(cif_cif10_dev->config.img_src_output.frm_fmt);

	reg_intstat =
		cif_ioread32(base_addr + CIF_CIF_INTSTAT);

	if ((reg_intstat & PST_INF_FRAME_END) &&
	    (reg_intstat & FRAME_END)) {
		cif_iowrite32(INTSTAT_CLS,
			      base_addr +
			      CIF_CIF_INTSTAT);
		cifctrl  = cif_ioread32(base_addr +
					CIF_CIF_CTRL);
		tmp_cif_frmst = cif_ioread32(base_addr +
					     CIF_CIF_FRAME_STATUS);
		lastpix  = cif_ioread32(base_addr +
					CIF_CIF_LAST_PIX);
		lastline = cif_ioread32(base_addr +
					CIF_CIF_LAST_LINE);

		if ((tmp_cif_frmst & CIF_F0_READY) &&
		    (tmp_cif_frmst & CIF_F1_READY)) {
			cif_cif10_pltfrm_pr_err(cif_cif10_dev->dev,
						"frm0&frm1 now reset frm_stat %#lx\n",
						tmp_cif_frmst);
			cif_err = 1;
			goto cif_rst;
		}
		if (tmp_cif_frmst & CIF_F0_READY) {
			frm_flag = 0;
		} else if (tmp_cif_frmst & CIF_F1_READY) {
			frm_flag = 1;
			cif_iowrite32(FRAME_STAT_CLS,
				      base_addr +
				      CIF_CIF_FRAME_STATUS);
		}

		/*
		 * CVBSIN没有信号输入时,TVD是以一个默认的
		 * 时序输出interval以45ms和20ms交替,当检测
		 * 到这种信号表示CVBSIN被拔出或poweroff,当
		 * 检测到信号频率是以20ms左右持续输出说明
		 * CVBSIN有输入,重设CIF和CVBSIN确保不错场
		 */
		if (interval > 45000) {
			/* recount cvbsin */
			cif_cif10_dev->irqinfo.plug = 3;

			/* detect signal of cvbsin plug out */
			if (cvbs_plug_out == 3)
				cvbs_plug_out = 2;
			/* cvbsin plug out */
			if (cvbs_plug_out == 1)
				cvbs_plug_out = 0;
		} else {
			if (cvbs_plug_out == 2)
				cvbs_plug_out = 1;
			/*
			 * 连续三次中断间隔20ms(pal)或16ms(nstc)左右
			 * 表示有外部信号输入
			 */
			if (cif_cif10_dev->irqinfo.plug > 0)
				cif_cif10_dev->irqinfo.plug--;
		}

		if (cif_cif10_dev->irqinfo.plug == 0) {
			if (cvbs_plug_out == 0) {
				reset = 1;
				dev_info(cif_cif10_dev->dev,
					 "cvbsin plugout\n");
			}
			/* reset flag of cvbsin plugout */
			cvbs_plug_out = 3;

			if (reset)
				goto cif_rst;
		}

		curr_buf = cif_cif10_dev->stream.curr_buf;

		if (!list_empty(&cif_cif10_dev->stream.buf_queue) &&
		    !cif_cif10_dev->stream.stop) {
			next_buf = list_entry(
					cif_cif10_dev->stream.buf_queue.next,
					struct videobuf_buffer,
					queue);
			WARN_ON(next_buf->state != VIDEOBUF_QUEUED);
			if (frm_flag == 0) {
				y_addr = videobuf_to_dma_contig(next_buf);
				uv_addr = y_addr +
					next_buf->width * next_buf->height;
				cif_iowrite32(
					y_addr,
					cif_cif10_dev->config.base_addr +
						CIF_CIF_FRM0_ADDR_Y);
				cif_iowrite32(
					uv_addr,
					cif_cif10_dev->config.base_addr +
						CIF_CIF_FRM0_ADDR_UV);
				 cif_cif10_dev->irqinfo.cif_frm0_ok = 1;
			} else if (frm_flag == 1 &&
				   cif_cif10_dev->irqinfo.cif_frm0_ok) {
				cif_cif10_dev->irqinfo.cif_frm1_ok = 1;

				y_addr = videobuf_to_dma_contig(next_buf) +
					 next_buf->width;
				uv_addr = y_addr +
					next_buf->width * next_buf->height;

				cif_iowrite32(y_addr,
					      base_addr +
					      CIF_CIF_FRM1_ADDR_Y);
				cif_iowrite32(uv_addr,
					      base_addr +
					      CIF_CIF_FRM1_ADDR_UV);

				cif_cif10_dev->stream.curr_buf = next_buf;
				list_del_init(&next_buf->queue);
			}

			if (frm_flag == 1 &&
			    cif_cif10_dev->irqinfo.cif_frm0_ok &&
			    cif_cif10_dev->irqinfo.cif_frm1_ok) {
				do_gettimeofday(&curr_buf->ts);
				cif_cif10_dev->irqinfo.cif_frm0_ok = 0;
				cif_cif10_dev->irqinfo.cif_frm1_ok = 0;
				if ((curr_buf->state == VIDEOBUF_QUEUED) ||
				    (curr_buf->state == VIDEOBUF_ACTIVE)) {
					curr_buf->state =
						VIDEOBUF_DONE;
					curr_buf->field_count++;
				}
				wake_up(&curr_buf->done);
			}
		} else {
			cif_cif10_dev->irqinfo.cif_frm0_ok = 0;
			cif_cif10_dev->irqinfo.cif_frm1_ok = 0;
			goto end;
		}
	} else {
		cif_cif10_pltfrm_pr_err(cif_cif10_dev->dev,
					"error now reset it intsat %#lx\n",
					reg_intstat);
		cif_iowrite32(INTSTAT_CLS,
			      base_addr + CIF_CIF_INTSTAT);
		cif_err = 1;
		goto cif_rst;
	}

cif_rst:
	if (reset || cif_err) {
		cif_iowrite32AND(~ENABLE_CAPTURE,
				 base_addr + CIF_CIF_CTRL);
		cif_iowrite32(INTEN_DISABLE,
			      base_addr + CIF_CIF_INTEN);

		queue_work(cif_cif10_dev->wq, &cif_cif10_dev->work);
	}

end:
	if (cif_cif10_dev->stream.stop &&
	    (cif_cif10_dev->stream.state ==
	     CIF_CIF10_STATE_STREAMING)) {
		cif_cif10_dev->stream.state = CIF_CIF10_STATE_READY;
		cif_iowrite32AND(~ENABLE_CAPTURE,
				 base_addr + CIF_CIF_CTRL);
		cif_iowrite32(INTEN_DISABLE,
			      base_addr + CIF_CIF_INTEN);
		cif_iowrite32(INTSTAT_CLS,
			      base_addr + CIF_CIF_INTSTAT);
		cif_iowrite32(FRAME_STAT_CLS,
			      base_addr + CIF_CIF_FRAME_STATUS);
		cif_cif10_pltfrm_event_signal(
				cif_cif10_dev->dev,
				&cif_cif10_dev->stream.done);
	}

	return IRQ_HANDLED;
}

irqreturn_t cif_cif10_oneframe_irq(int irq, void *data)
{
	short cif_err = 0;
	unsigned long y_addr, uv_addr;
	unsigned int cifctrl, lastpix, lastline;
	unsigned int cif_for, cif_crop;
	unsigned int cif_frmst, reg_intstat;
	struct cif_cif10_device *cif_cif10_dev = data;
	struct videobuf_buffer *curr_buf = NULL, *next_buf = NULL;
	void __iomem *base_addr =
		cif_cif10_dev->config.base_addr;
	struct pltfrm_cam_itf *cam_itf =
		&cif_cif10_dev->config.cam_itf;
	void *vaddr;
	unsigned char uv_1st_byte, uv_2nd_byte;

	cifctrl  = cif_ioread32(base_addr +
				CIF_CIF_CTRL);
	if (cifctrl & ENABLE_CAPTURE) {
		cif_iowrite32(cifctrl & ~ENABLE_CAPTURE,
			      base_addr +
			      CIF_CIF_CTRL);
	}
	reg_intstat = cif_ioread32(base_addr +
				   CIF_CIF_INTSTAT);
	cif_frmst = cif_ioread32(base_addr +
				 CIF_CIF_FRAME_STATUS);
	lastline = cif_ioread32(base_addr +
				CIF_CIF_LAST_LINE);
	if (reg_intstat & INTSTAT_ERR) {
		cif_iowrite32(INTSTAT_CLS,
			      base_addr + CIF_CIF_INTSTAT);
		cif_iowrite32(FRAME_STAT_CLS,
			      base_addr + CIF_CIF_FRAME_STATUS);
		goto cif_rst;
	}
	if ((reg_intstat & PST_INF_FRAME_END) &&
	    (reg_intstat & FRAME_END) &&
	    (cif_frmst & CIF_F0_READY)) {
		cif_iowrite32(INTSTAT_CLS,
			      base_addr + CIF_CIF_INTSTAT);
		cif_iowrite32(FRAME_STAT_CLS,
			      base_addr + CIF_CIF_FRAME_STATUS);
		lastpix  =
			cif_ioread32(base_addr + CIF_CIF_LAST_PIX);

		curr_buf = cif_cif10_dev->stream.curr_buf;
		if (cif_cif10_dev->irqinfo.dmairq_idx == 0 &&
		    PLTFRM_CAM_ITF_IS_BT656_MIX(cam_itf->type)) {
			cif_cif10_dev->irqinfo.dmairq_idx = 1;
			cif_for  = cif_ioread32(base_addr + CIF_CIF_FOR);
			if ((cif_for & 0x1c) == INPUT_MODE_YUV)	{
				cif_crop =
					cif_ioread32(base_addr + CIF_CIF_CROP);
				y_addr =
					videobuf_to_dma_contig(curr_buf);
				uv_addr = y_addr +
					  curr_buf->width *
					  curr_buf->height;
				vaddr = phys_to_virt(uv_addr);
				uv_1st_byte = *((unsigned char *)vaddr);
				uv_2nd_byte = *((unsigned char *)vaddr + 1);
				if (CIF_CROP_START_X(cif_crop) < 0x2)
					cif_crop = (cif_crop &
						    ~CIF_CROP_X_MASK) |
						   0x2;
				if (uv_1st_byte == 0xFF &&
				    uv_2nd_byte == 0x00) {
					cif_iowrite32(cif_crop,
						      base_addr + CIF_CIF_CROP);
				} else {
					/* miss y data, reset yuv order */
					cif_iowrite32(cif_crop,
						      base_addr + CIF_CIF_CROP);
					cif_for = YUV_INPUT_ORDER_YVYU(cif_for);
					cif_iowrite32(cif_for,
						      base_addr + CIF_CIF_FOR);
					goto end;
				}
			}
		}

		if (!list_empty(&cif_cif10_dev->stream.buf_queue) &&
		    !cif_cif10_dev->stream.stop &&
		    curr_buf != NULL) {
			next_buf = list_entry(
					cif_cif10_dev->stream.buf_queue.next,
					struct videobuf_buffer,
					queue);
			WARN_ON(next_buf->state != VIDEOBUF_QUEUED);
			y_addr = videobuf_to_dma_contig(next_buf);
			uv_addr = y_addr +
				  next_buf->width * next_buf->height;
			cif_iowrite32(y_addr,
				      cif_cif10_dev->config.base_addr +
				      CIF_CIF_FRM0_ADDR_Y);
			cif_iowrite32(uv_addr,
				      cif_cif10_dev->config.base_addr +
				      CIF_CIF_FRM0_ADDR_UV);

			do_gettimeofday(&curr_buf->ts);
			cif_cif10_dev->irqinfo.cif_frm0_ok = 0;
			if ((curr_buf->state == VIDEOBUF_QUEUED) ||
			    (curr_buf->state == VIDEOBUF_ACTIVE)) {
				curr_buf->state = VIDEOBUF_DONE;
				curr_buf->field_count += 2;
			}
			cif_cif10_dev->stream.curr_buf = next_buf;
			list_del_init(&next_buf->queue);
			wake_up(&curr_buf->done);
		} else {
			goto end;
		}
	} else {
		cif_cif10_pltfrm_pr_err(cif_cif10_dev->dev,
					"error now reset intsat %#x\n",
					reg_intstat);
		cif_iowrite32(FRAME_STAT_CLS,
			      base_addr +
			      CIF_CIF_FRAME_STATUS);
		cif_iowrite32(INTSTAT_CLS,
			      base_addr +
			      CIF_CIF_INTSTAT);
		cif_err = 1;
		goto cif_rst;
	}

cif_rst:
	if (cifctrl & ENABLE_CAPTURE) {
		cif_iowrite32(cifctrl | ENABLE_CAPTURE,
			      base_addr +
			      CIF_CIF_CTRL);
	}

	if (cif_err) {
		cif_iowrite32AND(~ENABLE_CAPTURE,
				 base_addr +
				 CIF_CIF_CTRL);
		cif_iowrite32(INTEN_DISABLE,
			      base_addr +
			      CIF_CIF_INTEN);

		queue_work(cif_cif10_dev->wq, &cif_cif10_dev->work);
	}

end:
	if (cif_cif10_dev->stream.stop &&
	    (cif_cif10_dev->stream.state ==
	     CIF_CIF10_STATE_STREAMING)) {
		cif_cif10_dev->stream.state = CIF_CIF10_STATE_READY;
		cif_iowrite32AND(~ENABLE_CAPTURE,
				 base_addr + CIF_CIF_CTRL);
		cif_iowrite32(INTEN_DISABLE,
			      base_addr + CIF_CIF_INTEN);
		cif_iowrite32(INTSTAT_CLS,
			      base_addr + CIF_CIF_INTSTAT);
		cif_iowrite32(FRAME_STAT_CLS,
			      base_addr + CIF_CIF_FRAME_STATUS);
		cif_cif10_pltfrm_event_signal(
				cif_cif10_dev->dev,
				&cif_cif10_dev->stream.done);
	}

	return IRQ_HANDLED;
}

irqreturn_t cif_cif10_pingpong_irq(int irq, void *data)
{
	short reset = 0, cif_err = 0, frm_flag = -1;
	unsigned long y_addr, uv_addr;
	unsigned int cif_for, cif_crop;
	unsigned int cifctrl, lastpix, lastline;
	unsigned long tmp_cif_frmst, reg_intstat;
	struct cif_cif10_device *cif_cif10_dev = data;
	struct cif_cif10_frm_fmt *frm_fmt = NULL;
	struct videobuf_buffer *curr_buf = NULL, *next_buf = NULL;
	void __iomem *base_addr =
		cif_cif10_dev->config.base_addr;
	struct pltfrm_cam_itf *cam_itf =
		&cif_cif10_dev->config.cam_itf;
	void *vaddr;
	unsigned char uv_1st_byte, uv_2nd_byte;

	frm_fmt =
		&cif_cif10_dev->config.img_src_output.frm_fmt;

	reg_intstat =
		cif_ioread32(base_addr + CIF_CIF_INTSTAT);

	if (/*(reg_intstat & PST_INF_FRAME_END) &&*/
	    (reg_intstat & FRAME_END)) {
		cif_iowrite32(INTSTAT_CLS,
			      base_addr +
			      CIF_CIF_INTSTAT);
		cifctrl  = cif_ioread32(base_addr +
					CIF_CIF_CTRL);
		tmp_cif_frmst = cif_ioread32(base_addr +
					     CIF_CIF_FRAME_STATUS);
		lastpix  = cif_ioread32(base_addr +
					CIF_CIF_LAST_PIX);
		lastline = cif_ioread32(base_addr +
					CIF_CIF_LAST_LINE);

		if ((tmp_cif_frmst & CIF_F0_READY) &&
		    (tmp_cif_frmst & CIF_F1_READY)) {
			cif_cif10_pltfrm_pr_err(cif_cif10_dev->dev,
						"frm0&frm1 now reset frm_stat %#lx\n",
						tmp_cif_frmst);
			cif_err = 1;
			goto cif_rst;
		}
		if (tmp_cif_frmst & CIF_F0_READY) {
			frm_flag = 0;
			curr_buf = cif_cif10_dev->stream.curr_buf;
		} else if (tmp_cif_frmst & CIF_F1_READY) {
			frm_flag = 1;
			curr_buf = cif_cif10_dev->stream.next_buf;
			cif_iowrite32(FRAME_STAT_CLS,
				      base_addr +
				      CIF_CIF_FRAME_STATUS);
		}

		if (!curr_buf) {
			dev_err(cif_cif10_dev->dev,
				"<pingpong mode>curr buf null\n");
			goto end;
		}

		if (cif_cif10_dev->irqinfo.dmairq_idx == 0 &&
		    PLTFRM_CAM_ITF_IS_BT656_MIX(cam_itf->type)) {
			cif_cif10_dev->irqinfo.dmairq_idx = 1;
			cif_for  = cif_ioread32(base_addr + CIF_CIF_FOR);
			if ((cif_for & 0x1c) == INPUT_MODE_YUV)	{
				cif_crop =
					cif_ioread32(base_addr + CIF_CIF_CROP);
				y_addr =
					videobuf_to_dma_contig(curr_buf);
				uv_addr = y_addr +
					  curr_buf->width *
					  curr_buf->height;
				vaddr = phys_to_virt(uv_addr);
				uv_1st_byte = *((unsigned char *)vaddr);
				uv_2nd_byte = *((unsigned char *)vaddr + 1);
				if (CIF_CROP_START_X(cif_crop) < 0x2)
					cif_crop = (cif_crop &
						    ~CIF_CROP_X_MASK) |
						   0x2;
				if (uv_1st_byte == 0xFF &&
				    uv_2nd_byte == 0x00) {
					cif_iowrite32(cif_crop,
						      base_addr + CIF_CIF_CROP);
				} else {
					/* miss y data, reset yuv order */
					cif_iowrite32(cif_crop,
						      base_addr + CIF_CIF_CROP);
					cif_for = YUV_INPUT_ORDER_YVYU(cif_for);
					cif_iowrite32(cif_for,
						      base_addr + CIF_CIF_FOR);
					goto end;
				}
			}
		}

		if (!list_empty(&cif_cif10_dev->stream.buf_queue) &&
		    !cif_cif10_dev->stream.stop) {
			next_buf = list_entry(
					cif_cif10_dev->stream.buf_queue.next,
					struct videobuf_buffer,
					queue);
			if (frm_flag == 0 && (next_buf->i % 2) != 0)
				goto end;
			if (frm_flag == 1 && (next_buf->i % 2) == 0)
				goto end;
			WARN_ON(next_buf->state != VIDEOBUF_QUEUED);
			y_addr = videobuf_to_dma_contig(next_buf);
			uv_addr = y_addr +
				next_buf->width * next_buf->height;
			if (frm_flag == 0) {
				cif_iowrite32(
					y_addr,
					base_addr +
					CIF_CIF_FRM0_ADDR_Y);
				cif_iowrite32(
					uv_addr,
					base_addr +
					CIF_CIF_FRM0_ADDR_UV);
				cif_cif10_dev->stream.curr_buf = next_buf;
			} else if (frm_flag == 1) {
				cif_iowrite32(y_addr,
					      base_addr +
					      CIF_CIF_FRM1_ADDR_Y);
				cif_iowrite32(uv_addr,
					      base_addr +
					      CIF_CIF_FRM1_ADDR_UV);
				cif_cif10_dev->stream.next_buf = next_buf;
			}
			list_del_init(&next_buf->queue);

			if (frm_flag == 0 || frm_flag == 1) {
				do_gettimeofday(&curr_buf->ts);
				if ((curr_buf->state == VIDEOBUF_QUEUED) ||
				    (curr_buf->state == VIDEOBUF_ACTIVE)) {
					curr_buf->state =
						VIDEOBUF_DONE;
					curr_buf->field_count++;
				}
				wake_up(&curr_buf->done);
			}
		} else {
			goto end;
		}
	} else {
		cif_cif10_pltfrm_pr_err(cif_cif10_dev->dev,
					"error now reset it intsat %#lx\n",
					reg_intstat);
		cif_iowrite32(INTSTAT_CLS,
			      base_addr + CIF_CIF_INTSTAT);
		cif_err = 1;
		goto cif_rst;
	}

cif_rst:
	if (reset || cif_err) {
		cif_iowrite32AND(~ENABLE_CAPTURE,
				 base_addr + CIF_CIF_CTRL);
		cif_iowrite32(INTEN_DISABLE,
			      base_addr + CIF_CIF_INTEN);

		queue_work(cif_cif10_dev->wq, &cif_cif10_dev->work);
	}

end:
	if (cif_cif10_dev->stream.stop &&
	    (cif_cif10_dev->stream.state ==
	     CIF_CIF10_STATE_STREAMING)) {
		cif_cif10_dev->stream.state = CIF_CIF10_STATE_READY;
		cif_iowrite32AND(~ENABLE_CAPTURE,
				 base_addr + CIF_CIF_CTRL);
		cif_iowrite32(INTEN_DISABLE,
			      base_addr + CIF_CIF_INTEN);
		cif_iowrite32(INTSTAT_CLS,
			      base_addr + CIF_CIF_INTSTAT);
		cif_iowrite32(FRAME_STAT_CLS,
			      base_addr + CIF_CIF_FRAME_STATUS);
		cif_cif10_pltfrm_event_signal(
				cif_cif10_dev->dev,
				&cif_cif10_dev->stream.done);
	}

	return IRQ_HANDLED;
}
