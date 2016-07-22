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

#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf-dma-contig.h>
#include "cif_cif10.h"
#include "cif_cif10_regs.h"
#include "cif_cif10_version.h"
#include <linux/module.h>
#include <linux/of.h>
#include <media/v4l2-controls_rockchip.h>
#include <linux/pm_runtime.h>

/* One structure per open file handle */
struct cif_cif10_v4l2_fh {
	struct v4l2_fh fh;
};

/* One structure per video node */
struct cif_cif10_v4l2_node {
	struct videobuf_queue buf_queue;
	struct video_device vdev;
	int users;
	struct cif_cif10_v4l2_fh *owner;
};

/* One structure per device */
struct cif_cif10_v4l2_device {
	struct cif_cif10_v4l2_node node[4];
	struct cif_cif10_v4l2_node *curr_node;
	int node_num;
};

/* TODO: make this a dynamically allocated variable */
static struct cif_cif10_v4l2_device cif_cif10_v4l2_dev;

static struct cif_cif10_v4l2_fh *to_fh(struct file *file)
{
	if (file == NULL || file->private_data == NULL)
		return NULL;

	return container_of(file->private_data, struct cif_cif10_v4l2_fh, fh);
}

static struct cif_cif10_v4l2_node *to_node(struct cif_cif10_v4l2_fh *fh)
{
	struct video_device *vdev = fh ? fh->fh.vdev : NULL;

	if (fh == NULL || vdev == NULL)
		return NULL;

	return container_of(vdev, struct cif_cif10_v4l2_node, vdev);
}

static struct videobuf_queue *to_videobuf_queue(
	struct file *file)
{
	struct cif_cif10_v4l2_fh *fh = to_fh(file);
	struct video_device *vdev = fh ? fh->fh.vdev : NULL;
	struct cif_cif10_v4l2_node *node = to_node(fh);
	struct videobuf_queue *q;

	if (unlikely(vdev == NULL)) {
		cif_cif10_pltfrm_pr_err(
			NULL,
			"vdev is NULL\n");
		BUG();
	}
	q = &node->buf_queue;
	if (unlikely(NULL == q)) {
		cif_cif10_pltfrm_pr_err(
			NULL,
			"buffer queue is NULL\n");
		BUG();
	}

	return q;
}

static struct cif_cif10_device *to_cif_cif10_device(
	struct videobuf_queue *queue)
{
	return queue->priv_data;
}

static const char *cif_cif10_v4l2_buf_type_string(
	enum v4l2_buf_type buf_type)
{
	switch (buf_type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return "VIDEO_CAPTURE";
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		return "VIDEO_OVERLAY";
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return "VIDEO_OUTPUT";
	default:
		break;
	}
	return "UNKNOWN/UNSUPPORTED";
}

const char *cif_cif10_v4l2_pix_fmt_string(
	int pix_fmt)
{
	switch (pix_fmt) {
	case V4L2_PIX_FMT_RGB332:
		return "V4L2-RGB332";
	case V4L2_PIX_FMT_RGB555:
		return "V4L2-RGB555";
	case V4L2_PIX_FMT_RGB565:
		return "V4L2-RGB565";
	case V4L2_PIX_FMT_RGB555X:
		return "V4L2-RGB555X";
	case V4L2_PIX_FMT_RGB565X:
		return "V4L2-RGB565X";
	case V4L2_PIX_FMT_BGR24:
		return "V4L2-BGR24";
	case V4L2_PIX_FMT_RGB24:
		return "V4L2-RGB24";
	case V4L2_PIX_FMT_BGR32:
		return "V4L2-BGR32";
	case V4L2_PIX_FMT_RGB32:
		return "V4L2-RGB32";
	case V4L2_PIX_FMT_GREY:
		return "V4L2-GREY";
	case V4L2_PIX_FMT_YVU410:
		return "V4L2-YVU410";
	case V4L2_PIX_FMT_YVU420:
		return "V4L2-YVU420";
	case V4L2_PIX_FMT_YUYV:
		return "V4L2-YUYV";
	case V4L2_PIX_FMT_UYVY:
		return "V4L2-UYVY";
	case V4L2_PIX_FMT_YUV422P:
		return "V4L2-YUV422P";
	case V4L2_PIX_FMT_YUV411P:
		return "V4L2-YUV411P";
	case V4L2_PIX_FMT_Y41P:
		return "V4L2-Y41P";
	case V4L2_PIX_FMT_NV12:
		return "V4L2-NV12";
	case V4L2_PIX_FMT_NV21:
		return "V4L2-NV21";
	case V4L2_PIX_FMT_YUV410:
		return "V4L2-YUV410";
	case V4L2_PIX_FMT_YUV420:
		return "V4L2--YUV420";
	case V4L2_PIX_FMT_YYUV:
		return "V4L2-YYUV";
	case V4L2_PIX_FMT_HI240:
		return "V4L2-HI240";
	case V4L2_PIX_FMT_WNVA:
		return "V4L2-WNVA";
	case V4L2_PIX_FMT_NV16:
		return "V4L2-NV16";
	case V4L2_PIX_FMT_YUV444:
		return "V4L2-YUV444P";
	case V4L2_PIX_FMT_NV24:
		return "M5-YUV444SP";
	case V4L2_PIX_FMT_JPEG:
		return "V4L2-JPEG";
	case V4L2_PIX_FMT_SGRBG10:
		return "RAW-BAYER-10Bits";
	case V4L2_PIX_FMT_SGRBG8:
		return "RAW-BAYER-8Bits";
	}
	return "UNKNOWN/UNSUPPORTED";
}

static int cif_cif10_v4l2_cid2cif_cif10_cid(u32 v4l2_cid)
{
	switch (v4l2_cid) {
	case V4L2_CID_FLASH_LED_MODE:
		return CIF_CIF10_CID_FLASH_MODE;
	case V4L2_CID_AUTOGAIN:
		return CIF_CIF10_CID_AUTO_GAIN;
	case V4L2_EXPOSURE_AUTO:
		return CIF_CIF10_CID_AUTO_EXPOSURE;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return CIF_CIF10_CID_AUTO_WHITE_BALANCE;
	case V4L2_CID_BLACK_LEVEL:
		return CIF_CIF10_CID_BLACK_LEVEL;
	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		return CIF_CIF10_CID_WB_TEMPERATURE;
	case V4L2_CID_EXPOSURE:
		return CIF_CIF10_CID_EXPOSURE_TIME;
	case V4L2_CID_GAIN:
		return CIF_CIF10_CID_ANALOG_GAIN;
	case V4L2_CID_FOCUS_ABSOLUTE:
		return CIF_CIF10_CID_FOCUS_ABSOLUTE;
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return CIF_CIF10_CID_AUTO_N_PRESET_WHITE_BALANCE;
	case V4L2_CID_SCENE_MODE:
		return CIF_CIF10_CID_SCENE_MODE;
	case V4L2_CID_COLORFX:
		return CIF_CIF10_CID_IMAGE_EFFECT;
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		return CIF_CIF10_CID_JPEG_QUALITY;
	case V4L2_CID_HFLIP:
		return CIF_CIF10_CID_HFLIP;
	case V4L2_CID_VFLIP:
		return CIF_CIF10_CID_VFLIP;
	case V4L2_CID_ISO_SENSITIVITY:
		return CIF_CIF10_CID_ISO_SENSITIVITY;
	case RK_V4L2_CID_AUTO_FPS:
		return CIF_CIF10_CID_AUTO_FPS;
	default:
		cif_cif10_pltfrm_pr_err(
			NULL,
			"unknown/unsupported V4L2 CID 0x%x\n",
			v4l2_cid);
		break;
	}
	return -EINVAL;
}

static enum cif_cif10_image_effect cif_cif10_v4l2_colorfx2cif_cif10_ie(
	u32 v4l2_colorfx)
{
	switch (v4l2_colorfx) {
	case V4L2_COLORFX_SEPIA:
		return CIF_CIF10_IE_SEPIA;
	case V4L2_COLORFX_BW:
		return CIF_CIF10_IE_BW;
	case V4L2_COLORFX_NEGATIVE:
		return CIF_CIF10_IE_NEGATIVE;
	case V4L2_COLORFX_EMBOSS:
		return CIF_CIF10_IE_EMBOSS;
	case V4L2_COLORFX_SKETCH:
		return CIF_CIF10_IE_SKETCH;
	case V4L2_COLORFX_NONE:
		return CIF_CIF10_IE_NONE;
	default:
		cif_cif10_pltfrm_pr_err(
			NULL,
			"unknown/unsupported V4L2 COLORFX %d\n",
			v4l2_colorfx);
		break;
	}
	return -EINVAL;
}

static unsigned int cif_cif10_pix_fmt2bytesperline(
		u32 v4l2_pix_fmt,
		unsigned int width)
{
	unsigned int bpl = (unsigned int)-EINVAL;

	switch (v4l2_pix_fmt) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_YUV422P:
		/* For YcbCr (semi)planar formats */
		/*the v4l2 manual says that */
		/* the bytes per line refers to */
		/*the biggest plane; in this case */
		/* the Y plane, which has 1 byte per pixel */
		bpl = width;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_YUYV:
		/* 2 bytes per pixel */
		bpl = width * 2;
		break;
	case V4L2_PIX_FMT_RGB32:
		/* 4 bytes per pixel */
		bpl = width * 4;
		break;
	case V4L2_PIX_FMT_JPEG:
		/* not used */
		bpl = 0;
	case V4L2_PIX_FMT_MJPEG:
		/* not used */
		bpl = 0;
	case V4L2_PIX_FMT_H264:
		/* not used */
		bpl = 0;
		break;
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
		bpl = width;
	default:
		pr_info(
				"%s: Unsupported V4L2 pixel format %c%c%c%c\n",
				__func__,
				v4l2_pix_fmt & 0xFF,
				(v4l2_pix_fmt >> 8) & 0xFF,
				(v4l2_pix_fmt >> 16) & 0xFF,
				(v4l2_pix_fmt >> 24) & 0xFF);
		break;
	}

	return bpl;
}

static enum cif_cif10_pix_fmt cif_cif10_v4l2_pix_fmt2cif_cif10_pix_fmt(
	u32 v4l2_pix_fmt)
{
	switch (v4l2_pix_fmt) {
	case V4L2_PIX_FMT_GREY:
		return CIF_YUV400;
	case V4L2_PIX_FMT_YUV420:
		return CIF_YUV420P;
	case V4L2_PIX_FMT_YVU420:
		return CIF_YVU420P;
	case V4L2_PIX_FMT_NV12:
		return CIF_YUV420SP;
	case V4L2_PIX_FMT_NV21:
		return CIF_YVU420SP;
	case V4L2_PIX_FMT_YUYV:
		return CIF_YUV422I;
	case V4L2_PIX_FMT_UYVY:
		return CIF_UYV422I;
	case V4L2_PIX_FMT_YUV422P:
		return CIF_YUV422P;
	case V4L2_PIX_FMT_NV16:
		return CIF_YUV422SP;
	case V4L2_PIX_FMT_NV61:
		return CIF_YVU422SP;
	case V4L2_PIX_FMT_YUV444:
		return CIF_YUV444P;
	case V4L2_PIX_FMT_NV24:
		return CIF_YUV444SP;
	case V4L2_PIX_FMT_RGB565:
		return CIF_RGB565;
	case V4L2_PIX_FMT_RGB24:
		return CIF_RGB888;
	case V4L2_PIX_FMT_SBGGR8:
		return CIF_BAYER_SBGGR8;
	case V4L2_PIX_FMT_SGBRG8:
		return CIF_BAYER_SGBRG8;
	case V4L2_PIX_FMT_SGRBG8:
		return CIF_BAYER_SGRBG8;
	case V4L2_PIX_FMT_SRGGB8:
		return CIF_BAYER_SRGGB8;
	case V4L2_PIX_FMT_SBGGR10:
		return CIF_BAYER_SBGGR10;
	case V4L2_PIX_FMT_SGBRG10:
		return CIF_BAYER_SGBRG10;
	case V4L2_PIX_FMT_SGRBG10:
		return CIF_BAYER_SGRBG10;
	case V4L2_PIX_FMT_SRGGB10:
		return CIF_BAYER_SRGGB10;
	case V4L2_PIX_FMT_SBGGR12:
		return CIF_BAYER_SBGGR12;
	case V4L2_PIX_FMT_SGBRG12:
		return CIF_BAYER_SGBRG12;
	case V4L2_PIX_FMT_SGRBG12:
		return CIF_BAYER_SGRBG12;
	case V4L2_PIX_FMT_SRGGB12:
		return CIF_BAYER_SRGGB12;
	case V4L2_PIX_FMT_JPEG:
		return CIF_JPEG;
	default:
		cif_cif10_pltfrm_pr_err(
			NULL,
			"unknown or unsupported V4L2 pixel format %c%c%c%c\n",
			(u8)(v4l2_pix_fmt & 0xff),
			(u8)((v4l2_pix_fmt >> 8) & 0xff),
			(u8)((v4l2_pix_fmt >> 16) & 0xff),
			(u8)((v4l2_pix_fmt >> 24) & 0xff));
		return CIF_UNKNOWN_FORMAT;
	}
}

static int cif_cif10_v4l2_register_video_device(
	struct cif_cif10_device *cif_cif10_dev,
	struct video_device *vdev,
	const char *name,
	int qtype,
	int major,
	const struct v4l2_file_operations *fops,
	const struct v4l2_ioctl_ops *ioctl_ops)
{
	int ret;

	vdev->release = video_device_release;
	strlcpy(vdev->name, name, sizeof(vdev->name));
	vdev->fops = fops;
	video_set_drvdata(vdev, cif_cif10_dev);
	vdev->ioctl_ops = ioctl_ops;
	vdev->v4l2_dev = &cif_cif10_dev->v4l2_dev;

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, major);
	if (IS_ERR_VALUE(ret)) {
		cif_cif10_pltfrm_pr_err(
			NULL,
			"video_register_device failed with error %d\n",
			ret);
		goto err;
	}

	cif_cif10_pltfrm_pr_info(
		NULL,
		"video device video%d.%d (%s) successfully registered\n",
		vdev->num,
		vdev->minor, name);

	return 0;
err:
	video_device_release(vdev);
	cif_cif10_pltfrm_pr_err(
		NULL,
		"failed with err %d\n",
		ret);
	return ret;
}


static int cif_cif10_v4l2_streamon(
	struct file *file,
	void *priv,
	enum v4l2_buf_type buf_type)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);
	struct cif_cif10_v4l2_fh *fh = to_fh(file);
	struct cif_cif10_v4l2_node *node = to_node(fh);
	struct timeval tv;

	if (node->owner != fh)
		return -EBUSY;

	cif_cif10_pltfrm_pr_dbg(
			dev->dev,
			"%s\n",
			cif_cif10_v4l2_buf_type_string(queue->type));

	dev->irqinfo.cifirq_idx = 0;
	dev->irqinfo.cifirq_normal_idx = 0;
	dev->irqinfo.cifirq_abnormal_idx = 0;
	dev->irqinfo.dmairq_idx = 0;
	do_gettimeofday(&tv);
	dev->irqinfo.cifirq_interval =
		tv.tv_sec*1000000 + tv.tv_usec;
	dev->irqinfo.plug = 0;
	dev->irqinfo.cif_frm0_ok = 0;
	dev->irqinfo.cif_frm1_ok = 0;

	ret = videobuf_streamon(queue);
	if (IS_ERR_VALUE(ret)) {
		cif_cif10_pltfrm_pr_err(
				dev->dev,
				"videobuf_streamon failed\n");
		goto err;
	}

	ret = cif_cif10_streamon(dev);
	if (IS_ERR_VALUE(ret)) {
		videobuf_queue_cancel(queue);
		goto err;
	}

	return 0;
err:
	(void)videobuf_mmap_free(queue);
	cif_cif10_pltfrm_pr_err(dev->dev, "failed with error %d\n", ret);
	return ret;
}

static int cif_cif10_v4l2_do_streamoff(
	struct file *file)
{
	int ret = 0;
	int err;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);
	struct cif_cif10_v4l2_fh *fh = to_fh(file);
	struct cif_cif10_v4l2_node *node = to_node(fh);

	cif_cif10_pltfrm_pr_dbg(
			dev->dev,
			"%s\n",
			cif_cif10_v4l2_buf_type_string(queue->type));

	if (node->owner != fh)
		return -EBUSY;

	err = cif_cif10_streamoff(dev);
	if (IS_ERR_VALUE(err))
		ret = -EFAULT;

	err = videobuf_streamoff(queue);
	if (IS_ERR_VALUE(err)) {
		cif_cif10_pltfrm_pr_err(
			dev->dev,
			"videobuf_streamoff failed with error %d\n",
			err);
		ret = -EFAULT;
	}

	err = videobuf_mmap_free(queue);
	if (IS_ERR_VALUE(err)) {
		cif_cif10_pltfrm_pr_err(
			dev->dev,
			"videobuf_mmap_free failed with error %d\n",
			err);
		ret = -EFAULT;
	}

	if (IS_ERR_VALUE(ret))
		cif_cif10_pltfrm_pr_err(
			dev->dev,
			"failed with error %d\n",
			ret);

	return ret;
}

static int cif_cif10_v4l2_streamoff(
	struct file *file,
	void *priv,
	enum v4l2_buf_type buf_type)
{
	int ret = cif_cif10_v4l2_do_streamoff(file);

	if (IS_ERR_VALUE(ret))
		cif_cif10_pltfrm_pr_err(
				NULL,
				"failed with error %d\n",
				ret);

	return ret;
}

static int cif_cif10_v4l2_qbuf(
	struct file *file,
	void *priv,
	struct v4l2_buffer *buf)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_v4l2_fh *fh = to_fh(file);
	struct cif_cif10_v4l2_node *node = to_node(fh);

	cif_cif10_pltfrm_pr_dbg(
		NULL,
		"%s buffer type %s, index %d\n",
		cif_cif10_v4l2_buf_type_string(queue->type),
		cif_cif10_v4l2_buf_type_string(buf->type),
		buf->index);

	if (node->owner != fh)
		return -EBUSY;

	ret = videobuf_qbuf(queue, buf);
	if (IS_ERR_VALUE(ret))
		cif_cif10_pltfrm_pr_err(
			NULL,
			"videobuf_qbuf failed with error %d\n",
			ret);
	return ret;
}

static int cif_cif10_v4l2_dqbuf(
	struct file *file,
	void *priv,
	struct v4l2_buffer *buf)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_v4l2_fh *fh = to_fh(file);
	struct cif_cif10_v4l2_node *node = to_node(fh);

	cif_cif10_pltfrm_pr_dbg(
			NULL,
			"%s\n",
			cif_cif10_v4l2_buf_type_string(queue->type));

	if (node->owner != fh)
		return -EBUSY;

	ret = videobuf_dqbuf(queue, buf, file->f_flags & O_NONBLOCK);
	if (IS_ERR_VALUE(ret) && (ret != -EAGAIN))
		cif_cif10_pltfrm_pr_err(
			NULL,
			"videobuf_dqbuf failed with error %d\n",
			ret);
	else
		cif_cif10_pltfrm_pr_dbg(
			NULL,
			"dequeued buffer %d, size %d\n",
			buf->index,
			buf->length);
	return ret;
}

static void cif_cif10_v4l2_buf_release(
	struct videobuf_queue *queue,
	struct videobuf_buffer *buf)
{
	cif_cif10_pltfrm_pr_dbg(
		NULL,
		"%s\n",
		cif_cif10_v4l2_buf_type_string(queue->type));

	videobuf_dma_contig_free(queue, buf);

	buf->state = VIDEOBUF_NEEDS_INIT;
}

static void cif_cif10_v4l2_buf_queue(
	struct videobuf_queue *queue,
	struct videobuf_buffer *buf)
{
	struct cif_cif10_device *cif_cif10_dev
				= to_cif_cif10_device(queue);

	cif_cif10_pltfrm_pr_dbg(
			NULL,
			"%s %dx%d, size %lu, bytesperline %d\n",
			cif_cif10_v4l2_buf_type_string(queue->type),
			buf->width,
			buf->height,
			buf->size,
			buf->bytesperline);

	if (!IS_ERR_VALUE(cif_cif10_qbuf(cif_cif10_dev, buf)))
		buf->state = VIDEOBUF_QUEUED;
	else
		cif_cif10_pltfrm_pr_err(NULL, "failed\n");
}

static int cif_cif10_v4l2_buf_setup(
	struct videobuf_queue *queue,
	unsigned int *cnt,
	unsigned int *size)
{
	int ret;
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);

	cif_cif10_pltfrm_pr_dbg(
			NULL,
			"%s count %d, size %d\n",
			cif_cif10_v4l2_buf_type_string(queue->type),
			*cnt,
			*size);

	ret = cif_cif10_calc_min_out_buff_size(
		dev, size);
	if (IS_ERR_VALUE(ret)) {
		cif_cif10_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
		return ret;
	}

	cif_cif10_pltfrm_pr_dbg(
			NULL,
			"%s count %d, size %d\n",
			cif_cif10_v4l2_buf_type_string(queue->type),
			*cnt,
			*size);

	return 0;
}

static int cif_cif10_v4l2_buf_prepare(
	struct videobuf_queue *queue,
	struct videobuf_buffer *buf,
	enum v4l2_field field)
{
	int ret;
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);
	u32 size;

	cif_cif10_pltfrm_pr_dbg(
			NULL,
			"%s\n",
			cif_cif10_v4l2_buf_type_string(queue->type));

	ret = cif_cif10_calc_min_out_buff_size(
		dev, &size);
	if (IS_ERR_VALUE(ret))
		goto err;
	buf->size = size;

	buf->width =
		dev->config.output.defrect.width;
	buf->height =
		dev->config.output.defrect.height;

	buf->field = field;

	cif_cif10_pltfrm_pr_dbg(
			NULL,
			"%s buffer prepared %dx%d, size %d\n",
			cif_cif10_v4l2_buf_type_string(queue->type),
			buf->width,
			buf->height,
			size);

	if (buf->state == VIDEOBUF_NEEDS_INIT) {
		ret = videobuf_iolock(queue, buf, NULL);
		if (IS_ERR_VALUE(ret)) {
			cif_cif10_pltfrm_pr_err(
				NULL,
				"videobuf_iolock failed with error %d\n",
				ret);
			goto err;
		}
	}
	buf->state = VIDEOBUF_PREPARED;

	return 0;
err:
	cif_cif10_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	cif_cif10_v4l2_buf_release(queue, buf);
	return ret;
}

static int cif_cif10_v4l2_reqbufs(
	struct file *file,
	void *priv,
	struct v4l2_requestbuffers *req)
{
	struct cif_cif10_v4l2_fh *fh = to_fh(file);
	struct cif_cif10_v4l2_node *node = to_node(fh);
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);

	cif_cif10_pltfrm_pr_dbg(
		NULL,
		"%s requested type %s, count %d\n",
		cif_cif10_v4l2_buf_type_string(queue->type),
		cif_cif10_v4l2_buf_type_string(req->type),
		req->count);

	if (node->owner && node->owner != fh)
		return -EBUSY;
	node->owner = fh;

	ret = videobuf_reqbufs(queue, req);
	if (IS_ERR_VALUE(ret)) {
		cif_cif10_pltfrm_pr_err(
			NULL,
			"videobuf_reqbufs failed with error %d\n",
			ret);
	}
	cif_cif10_reqbufs(dev, req);
	return ret;
}

static int cif_cif10_v4l2_querybuf(
	struct file *file,
	void *priv,
	struct v4l2_buffer *buf)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);

	cif_cif10_pltfrm_pr_dbg(
		NULL,
		"%s, index %d\n",
		cif_cif10_v4l2_buf_type_string(queue->type), buf->index);

	ret = videobuf_querybuf(queue, buf);
	if (IS_ERR_VALUE(ret))
		cif_cif10_pltfrm_pr_err(
			NULL,
			"videobuf_querybuf failed with error %d\n",
			ret);

	return ret;
}

static int cif_cif10_v4l2_s_ctrl(
	struct file *file,
	void *priv,
	struct v4l2_control *vc)
{
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);
	enum cif_cif10_cid id =
		cif_cif10_v4l2_cid2cif_cif10_cid(vc->id);
	int val = vc->value;

	if (IS_ERR_VALUE(id))
		return id;

	switch (vc->id) {
	case V4L2_CID_COLORFX:
		val = cif_cif10_v4l2_colorfx2cif_cif10_ie(val);
		break;
	case V4L2_CID_FLASH_LED_MODE:
		if (vc->value == V4L2_FLASH_LED_MODE_NONE)
			val = CIF_CIF10_FLASH_MODE_OFF;
		else if (vc->value == V4L2_FLASH_LED_MODE_FLASH)
			val = CIF_CIF10_FLASH_MODE_FLASH;
		else if (vc->value == V4L2_FLASH_LED_MODE_TORCH)
			val = CIF_CIF10_FLASH_MODE_TORCH;
		else
			val = -EINVAL;
		break;
	default:
		break;
	}

	return cif_cif10_s_ctrl(dev, id, val);
}

static int cif_cif10_v4l2_s_fmt(
	struct file *file,
	void *priv,
	struct v4l2_format *f)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);
	struct cif_cif10_v4l2_fh *fh = to_fh(file);
	struct cif_cif10_v4l2_node *node = to_node(fh);
	struct cif_cif10_strm_fmt strm_fmt;
	struct cif_cif10_frm_fmt *output;

	cif_cif10_pltfrm_pr_dbg(
		NULL,
		"%s\n",
		cif_cif10_v4l2_buf_type_string(queue->type));

	if (node->owner && node->owner != fh) {
		pr_err("VIDOC_S_FMT: The Dev Busy\n");
		return -EBUSY;
	}

	if (dev == NULL) {
		pr_err("%s: cif_cif10_dev not init l-%d\n", __func__, __LINE__);
		return -1;
	}
	output = &dev->config.output;
	memset(&strm_fmt, 0, sizeof(strm_fmt));
	strm_fmt.frm_fmt.pix_fmt =
		cif_cif10_v4l2_pix_fmt2cif_cif10_pix_fmt(
			f->fmt.pix.pixelformat);
	strm_fmt.frm_fmt.width = f->fmt.pix.width;
	strm_fmt.frm_fmt.height = f->fmt.pix.height;

	ret = cif_cif10_s_fmt(
				dev,
				&strm_fmt,
				f->fmt.pix.bytesperline);
	if (IS_ERR_VALUE(ret))
		goto err;

	/* calculate cropping for aspect ratio */
	ret = cif_cif10_calc_cif_cropping(
		dev,
		&output->defrect.width,
		&output->defrect.height,
		&output->defrect.left,
		&output->defrect.top);

	output->stride = cif_cif10_pix_fmt2bytesperline(
						f->fmt.pix.pixelformat,
						output->defrect.width);
	output->llength = cif_cif10_calc_llength(
						output->defrect.width,
						output->stride,
						output->pix_fmt);

	f->fmt.pix.width  = output->defrect.width;
	f->fmt.pix.height = output->defrect.height;
	f->fmt.pix.bytesperline = output->stride;

	cif_cif10_calc_min_out_buff_size(dev, &f->fmt.pix.sizeimage);
	return 0;
err:
	cif_cif10_pltfrm_pr_err(
		NULL,
		"failed with error %d\n", ret);
	return ret;
}

/* existence of this function is checked by V4L2 */
static int cif_cif10_v4l2_g_fmt(
	struct file *file,
	void *priv,
	struct v4l2_format *f)
{
	return -EFAULT;
}

static int cif_cif10_v4l2_s_input(
	struct file *file,
	void *priv,
	unsigned int i)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);

	cif_cif10_pltfrm_pr_dbg(dev->dev, "setting input to %d\n", i);

	ret = cif_cif10_s_input(dev, i);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_cif10_pltfrm_pr_err(
			NULL,
			"failed with error %d\n",
			ret);
	return ret;
}

static int cif_cif10_v4l2_enum_framesizes(
	struct file *file,
	void *priv,
	struct v4l2_frmsizeenum *fsize)
{
	/* THIS FUNCTION IS UNDER CONSTRUCTION */
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);

	if (IS_ERR_OR_NULL(dev->img_src)) {
		cif_cif10_pltfrm_pr_err(
			NULL,
			"input has not yet been selected, cannot enumerate formats\n");
		ret = -ENODEV;
		goto err;
	}

	return -EINVAL;
err:
	cif_cif10_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	return ret;
}

/* fops **********************************************************************/

const struct videobuf_queue_ops cif_cif10_qops = {
	.buf_setup = cif_cif10_v4l2_buf_setup,
	.buf_prepare = cif_cif10_v4l2_buf_prepare,
	.buf_queue = cif_cif10_v4l2_buf_queue,
	.buf_release = cif_cif10_v4l2_buf_release,
};

static int cif_cif10_v4l2_open(
	struct file *file)
{
	int ret;
	struct video_device *vdev = video_devdata(file);
	struct cif_cif10_device *dev = video_get_drvdata(vdev);
	struct cif_cif10_v4l2_fh *fh;
	struct cif_cif10_v4l2_node *node;
	enum v4l2_buf_type buf_type;

	cif_cif10_pltfrm_pr_dbg(
		NULL,
		"video device video%d.%d (%s)\n",
		vdev->num,
		vdev->minor,
		vdev->name);

	buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (NULL == fh) {
		cif_cif10_pltfrm_pr_err(
				NULL,
				"memory allocation failed\n");
		ret = -ENOMEM;
		goto err;
	}

	file->private_data = &fh->fh;
	v4l2_fh_init(&fh->fh, vdev);
	v4l2_fh_add(&fh->fh);

	node = to_node(fh);
	cif_cif10_v4l2_dev.curr_node = node;
	if (++node->users > 1)
		return 0;

	/* First open of the device, so initialize everything */
	node->owner = NULL;

	videobuf_queue_dma_contig_init(
		to_videobuf_queue(file),
		&cif_cif10_qops,
		dev->dev,
		&dev->vbq_lock,
		buf_type,
		V4L2_FIELD_NONE,
		sizeof(struct videobuf_buffer),
		dev, NULL);

	ret = cif_cif10_init(dev);
	if (IS_ERR_VALUE(ret)) {
		v4l2_fh_del(&fh->fh);
		v4l2_fh_exit(&fh->fh);
		kfree(fh);
		node->users--;
		goto err;
	}

	return 0;
err:
	cif_cif10_pltfrm_pr_err(
			NULL,
			"failed with error %d\n", ret);
	return ret;
}

static int cif_cif10_v4l2_release(struct file *file)
{
	int ret = 0;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);
	struct cif_cif10_v4l2_fh *fh = to_fh(file);
	struct cif_cif10_v4l2_node *node = to_node(fh);

	cif_cif10_pltfrm_pr_dbg(
			dev->dev,
			"%s users %d\n",
			cif_cif10_v4l2_buf_type_string(queue->type),
			node->users);

	if (node->users) {
		--node->users;
	} else {
		cif_cif10_pltfrm_pr_warn(
			dev->dev,
			"number of users for this device is already 0\n");
		return 0;
	}

	if (!node->users) {
		if (queue->streaming)
			if (IS_ERR_VALUE(cif_cif10_v4l2_do_streamoff(file)))
				cif_cif10_pltfrm_pr_warn(
						dev->dev,
						"streamoff failed\n");

		/* Last close, so uninitialize hardware */
		ret = cif_cif10_release(dev);
	}

	if (node->owner == fh)
		node->owner = NULL;

	v4l2_fh_del(&fh->fh);
	v4l2_fh_exit(&fh->fh);
	kfree(fh);

	if (IS_ERR_VALUE(ret))
		cif_cif10_pltfrm_pr_err(
			dev->dev,
			"failed with error %d\n",
			ret);
	return ret;
}

static unsigned int cif_cif10_v4l2_poll(
	struct file *file,
	struct poll_table_struct *wait)
{
	int ret = 0;
	struct cif_cif10_v4l2_fh *fh = to_fh(file);
	struct videobuf_queue *queue = to_videobuf_queue(file);
	unsigned long req_events = poll_requested_events(wait);

	cif_cif10_pltfrm_pr_dbg(
			NULL,
			"%s\n",
			cif_cif10_v4l2_buf_type_string(queue->type));

	if (v4l2_event_pending(&fh->fh))
		ret = POLLPRI;
	else if (req_events & POLLPRI)
		poll_wait(file, &fh->fh.wait, wait);

	if (!(req_events & (POLLIN | POLLOUT | POLLRDNORM)))
		return ret;

	ret |= videobuf_poll_stream(file, queue, wait);
	if (ret & POLLERR) {
		cif_cif10_pltfrm_pr_err(
			NULL,
			"videobuf_poll_stream failed with error 0x%x\n",
			ret);
	}
	return ret;
}

const struct v4l2_file_operations cif_cif10_v4l2_fops = {
	.open = cif_cif10_v4l2_open,
	.unlocked_ioctl = video_ioctl2,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = video_ioctl2,
#endif
	.release = cif_cif10_v4l2_release,
	.poll = cif_cif10_v4l2_poll,
};

/*TBD: clean up code below this line******************************************/

static int v4l2_querycap(struct file *file,
			 void *priv, struct v4l2_capability *cap)
{
	int ret = 0;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);

	strcpy(cap->driver, DRIVER_NAME);
	strcpy(cap->card, dev->soc_cfg->name);

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE|
		V4L2_CAP_STREAMING;
	cap->capabilities |= V4L2_CAP_DEVICE_CAPS;
	cap->device_caps = V4L2_CAP_DEVICE_CAPS;
	return ret;
}

static void cif_cif10_v4l2_event(__u32 frame_sequence)
{
	struct v4l2_event ev;

	memset(&ev, 0, sizeof(ev));
	ev.type = V4L2_EVENT_FRAME_SYNC;
	ev.u.frame_sync.frame_sequence = frame_sequence;
	v4l2_event_queue(&cif_cif10_v4l2_dev.curr_node->vdev, &ev);
}

static void cif_cif10_v4l2_requeue_bufs(void)
{
	struct videobuf_buffer *buf;
	struct videobuf_queue *q;
	struct cif_cif10_device *dev;

	q = &cif_cif10_v4l2_dev.curr_node->buf_queue;

	dev = to_cif_cif10_device(q);

	list_for_each_entry(buf, &q->stream, stream) {
		if (!IS_ERR_VALUE(cif_cif10_qbuf(
			to_cif_cif10_device(q), buf))) {
			spin_lock(&dev->vbreq_lock);
			if ((buf->state == VIDEOBUF_QUEUED) ||
			    (buf->state == VIDEOBUF_ACTIVE) ||
			    (buf->state == VIDEOBUF_DONE))
				buf->state = VIDEOBUF_QUEUED;
			else
				pr_err(
					"ERR: buf->state is: %d\n",
					buf->state);
			spin_unlock(&dev->vbreq_lock);
		} else {
			cif_cif10_pltfrm_pr_err(
				NULL,
				"failed for buffer %d\n",
				buf->i);
		}
	}
}
static long v4l2_default_ioctl(
		struct file *file,
		void *fh,
		bool valid_prio,
		unsigned int cmd,
		void *arg)
{
	return 0;
}

static int v4l2_s_parm(
	struct file *file,
	void *priv,
	struct v4l2_streamparm *a)
{
	return 0;
}

static int v4l2_enum_input(struct file *file, void *priv,
			   struct v4l2_input *input)
{
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);
	const char *inp_name;

	if (
		(queue->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) &&
		(queue->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)) {
		cif_cif10_pltfrm_pr_err(
				NULL,
				"wrong buffer queue %d\n",
				queue->type);
		return -EINVAL;
	}

	inp_name = cif_cif10_g_input_name(dev, input->index);
	if (IS_ERR_OR_NULL(inp_name))
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->std = V4L2_STD_UNKNOWN;
	strcpy(input->name, inp_name);

	return 0;
}

/* ================================================================= */
#ifdef NOT_YET
static int mainpath_try_fmt_cap(struct v4l2_format *f)
{
	int ifmt = 0;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	cif_cif10_pltfrm_pr_dbg(NULL, "\n");

	for (ifmt = 0; ifmt < get_cif_cif10_output_format_size(); ifmt++) {
		if (pix->pixelformat ==
				get_cif_cif10_output_format(ifmt)->fourcc)
			break;
	}

	if (ifmt == get_cif_cif10_output_format_size())
		ifmt = 0;

	pix->bytesperline = pix->width *
		get_cif_cif10_output_format(ifmt)->depth / 8;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_YUV444:
	case V4L2_PIX_FMT_NV24:
	case V4L2_PIX_FMT_JPEG:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_SGRBG10:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	default:
		BUG();
		break;
	}

	return 0;
}
#endif

static int v4l2_enum_fmt_cap(struct file *file, void *fh,
			     struct v4l2_fmtdesc *f)
{
	int ret = 0;
	int xgold_num_format = 0;
	struct v4l2_fmtdesc *fmtdesc = NULL;

	xgold_num_format = get_cif_cif10_output_format_desc_size();
	fmtdesc = get_cif_cif10_output_format_desc(f->index);
	if (
		(f->index >= xgold_num_format) ||
			(fmtdesc->pixelformat == 0)) {
		cif_cif10_pltfrm_pr_err(NULL, "index %d\n", f->index);
		return -EINVAL;
	}
	strlcpy(f->description,
		fmtdesc->description,
			sizeof(f->description));
	f->pixelformat = fmtdesc->pixelformat;
	f->flags = fmtdesc->flags;

	return ret;
}

static int v4l2_g_ctrl(
		struct file *file,
		void *priv,
		struct v4l2_control *vc)
{
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);
	enum cif_cif10_cid id =
		cif_cif10_v4l2_cid2cif_cif10_cid(vc->id);

	return cif_cif10_img_src_g_ctrl(dev->img_src,
		id, &vc->value);
}

int cif_cif10_v4l2_cropcap(
	struct file *file,
	void *fh,
	struct v4l2_cropcap *a)
{
	int ret = 0;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_cif10_device *dev = to_cif_cif10_device(queue);
	u32 h_offs, v_offs;
	u32 crop_width, crop_height;

	/* calculate cropping for aspect ratio */
	ret = cif_cif10_calc_cif_cropping(
			dev,
			&crop_width,
			&crop_height,
			&h_offs,
			&v_offs);

	/* This is the input to Bayer after input formatter cropping */
	a->defrect.top =
		v_offs/* + dev->config.img_src_output.frm_fmt.defrect.top*/;
	a->defrect.left =
		h_offs/* + dev->config.img_src_output.frm_fmt.defrect.left*/;
	a->defrect.width = crop_width;
	a->defrect.height = crop_height;

	a->bounds.top = 0;
	a->bounds.left = 0;
	a->bounds.width = dev->config.img_src_output.frm_fmt.defrect.width;
	a->bounds.height = dev->config.img_src_output.frm_fmt.defrect.height;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;/*V4L2_BUF_TYPE_VIDEO_OVERLAY*/

	cif_cif10_pltfrm_pr_dbg(
		dev->dev,
		"v4l2_cropcap: defrect(%d,%d,%d,%d) bounds(%d,%d,%d,%d)\n",
		a->defrect.width,
		a->defrect.height,
		a->defrect.left,
		a->defrect.top,
		a->bounds.width,
		a->bounds.height,
		a->bounds.left,
		a->bounds.top);

	return ret;
}

int cif_cif10_v4l2_g_crop(struct file *file, void *fh, struct v4l2_crop *a)
{
	return 0;
}

/*
	This is a write only function, so the upper layer
	will ignore the changes to 'a'. So don't use 'a' to pass
	the actual cropping parameters, the upper layer
	should call g_crop to get the actual window.
*/
int cif_cif10_v4l2_s_crop(
	struct file *file,
	void *fh,
	const struct v4l2_crop *a)
{
	return 0;
}

const struct v4l2_ioctl_ops cif_cif10_v4l2_ioctlops = {
	.vidioc_reqbufs = cif_cif10_v4l2_reqbufs,
	.vidioc_querybuf = cif_cif10_v4l2_querybuf,
	.vidioc_qbuf = cif_cif10_v4l2_qbuf,
	.vidioc_dqbuf = cif_cif10_v4l2_dqbuf,
	.vidioc_streamon = cif_cif10_v4l2_streamon,
	.vidioc_streamoff = cif_cif10_v4l2_streamoff,
	.vidioc_s_input = cif_cif10_v4l2_s_input,
	.vidioc_enum_input = v4l2_enum_input,
	.vidioc_g_ctrl = v4l2_g_ctrl,
	.vidioc_s_ctrl = cif_cif10_v4l2_s_ctrl,
	.vidioc_s_fmt_vid_cap = cif_cif10_v4l2_s_fmt,
	.vidioc_g_fmt_vid_cap = cif_cif10_v4l2_g_fmt,
	.vidioc_enum_fmt_vid_cap = v4l2_enum_fmt_cap,
	.vidioc_enum_framesizes = cif_cif10_v4l2_enum_framesizes,
	.vidioc_s_parm = v4l2_s_parm,
	.vidioc_querycap = v4l2_querycap,
	.vidioc_cropcap = cif_cif10_v4l2_cropcap,
	.vidioc_s_crop = cif_cif10_v4l2_s_crop,
	.vidioc_g_crop = cif_cif10_v4l2_g_crop,
	.vidioc_default = v4l2_default_ioctl,
};

static struct pltfrm_soc_cfg rk1108_cfg = {
	.name = CIF_CIF10_SOC_RK1108,
	.soc_cfg = pltfrm_rk1108_cfg,
};

static const struct of_device_id cif_cif10_v4l2_of_match[] = {
	{  .compatible = "rockchip,rk1108-cif-cif",
	   .data = (void *)&rk1108_cfg},
	{},
};
static int cif_cif10_v4l2_drv_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device_node *node = pdev->dev.of_node;
	struct cif_cif10_device *cif_cif10_dev = NULL;
	int ret;
	char vdev_name[32];

	cif_cif10_pltfrm_pr_info(NULL, "probing...\n");


	match = of_match_node(cif_cif10_v4l2_of_match, node);
	cif_cif10_dev = cif_cif10_create(
			&pdev->dev,
			cif_cif10_v4l2_event,
			cif_cif10_v4l2_requeue_bufs,
			(struct pltfrm_soc_cfg *)match->data);
	if (IS_ERR_OR_NULL(cif_cif10_dev)) {
		ret = -ENODEV;
		goto err;
	}

	spin_lock_init(&cif_cif10_dev->vbq_lock);
	spin_lock_init(&cif_cif10_dev->vbreq_lock);

	ret = v4l2_device_register(
			cif_cif10_dev->dev,
			&cif_cif10_dev->v4l2_dev);
	if (IS_ERR_VALUE(ret)) {
		cif_cif10_pltfrm_pr_err(
				NULL,
				"V4L2 device registration failed\n");
		goto err;
	}

	sprintf(vdev_name, "CIF%d", pdev->id);
	ret = cif_cif10_v4l2_register_video_device(
		cif_cif10_dev,
		&cif_cif10_v4l2_dev.node[pdev->id].vdev,
		vdev_name,
		V4L2_CAP_VIDEO_CAPTURE,
		pdev->id,
		&cif_cif10_v4l2_fops,
		&cif_cif10_v4l2_ioctlops);
	if (ret)
		goto err;

	cif_cif10_dev->wq = create_workqueue("cif_wk");
	if (!cif_cif10_dev->wq) {
		cif_cif10_pltfrm_pr_err(
				NULL,
				"V4L2 Create workqueue failed\n");
		goto err;
	}

	cif_cif10_v4l2_dev.node_num++;

	pm_runtime_enable(&pdev->dev);
	return 0;
err:
	cif_cif10_destroy(cif_cif10_dev);
	return ret;
}

/* ======================================================================== */

static int cif_cif10_v4l2_drv_remove(struct platform_device *pdev)
{
	struct cif_cif10_device *cif_cif10_dev =
		(struct cif_cif10_device *)platform_get_drvdata(pdev);

	if (IS_ERR_VALUE(cif_cif10_release(cif_cif10_dev)))
		cif_cif10_pltfrm_pr_warn(
				cif_cif10_dev->dev,
				"CIF%d power off failed\n",
				pdev->id);

	video_unregister_device(&cif_cif10_v4l2_dev.node[pdev->id].vdev);
	v4l2_device_unregister(&cif_cif10_dev->v4l2_dev);
	cif_cif10_pltfrm_dev_release(&pdev->dev);
	cif_cif10_destroy(cif_cif10_dev);

	return 0;
}

static int cif_cif10_v4l2_drv_suspend(
		struct platform_device *pdev,
		pm_message_t state)
{
	int ret = 0;
	struct cif_cif10_device *cif_cif10_dev =
		(struct cif_cif10_device *)platform_get_drvdata(pdev);

	cif_cif10_pltfrm_pr_dbg(cif_cif10_dev->dev, "\n");

	ret = cif_cif10_suspend(cif_cif10_dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	cif_cif10_pltfrm_pinctrl_set_state(
			&pdev->dev,
			CIF_CIF10_PINCTRL_STATE_SLEEP);

	return 0;
err:
	cif_cif10_pltfrm_pr_err(
			cif_cif10_dev->dev,
			"failed with error %d\n",
			ret);
	return ret;
}

static int cif_cif10_v4l2_drv_resume(struct platform_device *pdev)
{
	int ret = 0;
	struct cif_cif10_device *cif_cif10_dev =
		(struct cif_cif10_device *)platform_get_drvdata(pdev);

	cif_cif10_pltfrm_pr_dbg(cif_cif10_dev->dev, "\n");

	if (!cif_cif10_dev->img_src) {
		cif_cif10_pltfrm_pr_err(
			cif_cif10_dev->dev,
			"cif_cif10_dev img_src is null!\n");
		goto err;
	}

	ret = cif_cif10_resume(cif_cif10_dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	cif_cif10_pltfrm_pinctrl_set_state(
			&pdev->dev,
			CIF_CIF10_PINCTRL_STATE_DEFAULT);

	return 0;
err:
	cif_cif10_pltfrm_pr_err(
			cif_cif10_dev->dev,
			"failed with error %d\n",
			ret);
	return ret;
}

static int cif_cif10_runtime_suspend(struct device *dev)
{
	cif_cif10_pltfrm_pr_dbg(dev, "\n");
	return cif_cif10_pltfrm_pm_set_state(dev, CIF_CIF10_PM_STATE_SUSPENDED);
}

static int cif_cif10_runtime_resume(struct device *dev)
{
	cif_cif10_pltfrm_pr_dbg(dev, "\n");
	return cif_cif10_pltfrm_pm_set_state(dev, CIF_CIF10_PM_STATE_SW_STNDBY);
}

static const struct dev_pm_ops cif_cif10_dev_pm_ops = {
	SET_RUNTIME_PM_OPS(cif_cif10_runtime_suspend,
			   cif_cif10_runtime_resume, NULL)
};

static struct platform_driver cif_cif10_v4l2_plat_drv = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(cif_cif10_v4l2_of_match),
		.pm = &cif_cif10_dev_pm_ops,
		   },
	.probe = cif_cif10_v4l2_drv_probe,
	.remove = cif_cif10_v4l2_drv_remove,
	.suspend = cif_cif10_v4l2_drv_suspend,
	.resume = cif_cif10_v4l2_drv_resume,
};

/* ======================================================================== */
static int cif_cif10_v4l2_init(void)
{
	int ret;

	memset(&cif_cif10_v4l2_dev, 0, sizeof(cif_cif10_v4l2_dev));
	ret = platform_driver_register(&cif_cif10_v4l2_plat_drv);
	if (ret) {
		cif_cif10_pltfrm_pr_err(
				NULL,
				"cannot register platfrom driver, failed with %d\n",
				ret);
		return -ENODEV;
	}

	return ret;
}

/* ======================================================================== */
static void __exit cif_cif10_v4l2_exit(void)
{
	platform_driver_unregister(&cif_cif10_v4l2_plat_drv);
}

device_initcall_sync(cif_cif10_v4l2_init);
module_exit(cif_cif10_v4l2_exit);

MODULE_DESCRIPTION("V4L2 interface for CIF CIF10 driver");
MODULE_AUTHOR("George");
MODULE_LICENSE("GPL");
