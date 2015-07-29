/*
 *  go2001 - GO2001 codec driver.
 *
 *  Author : Pawel Osciak <posciak@chromium.org>
 *
 *  Copyright (C) 2015 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kmod.h>
#include <linux/mutex.h>
#include <linux/pci.h>

#include <media/v4l2-event.h>
#include <media/videobuf2-dma-sg.h>

#include "go2001.h"
#include "go2001_hw.h"

#define DRIVER_NAME "go2001"
#define VDEV_NAME_DEC (DRIVER_NAME "-dec")
#define VDEV_NAME_ENC (DRIVER_NAME "-enc")

#define DST_QUEUE_OFF_BASE	(1 << 30)

unsigned go2001_debug_level;
unsigned go2001_fw_debug_level;
module_param(go2001_debug_level, uint, 0644);
MODULE_PARM_DESC(go2001_debug_level, " verbosity level for debug messages.");
module_param(go2001_fw_debug_level, uint, 0644);
MODULE_PARM_DESC(go2001_fw_debug_level,
			" verbosity level for firmware debug messages.");

static void go2001_ctx_error(struct go2001_ctx *ctx)
{
	unsigned long flags;

	go2001_err(ctx->gdev, "Setting error state for ctx %p\n", ctx);
	spin_lock_irqsave(&ctx->qlock, flags);
	ctx->state = ERROR;
	spin_unlock_irqrestore(&ctx->qlock, flags);
}

static void go2001_cleanup_queue(struct go2001_ctx *ctx,
					struct list_head *buf_list)
{
	struct go2001_buffer *buf, *buf_tmp;
	int i;

	assert_spin_locked(&ctx->qlock);

	list_for_each_entry_safe(buf, buf_tmp, buf_list, list) {
		list_del(&buf->list);
		for (i = 0; i < buf->vb.num_planes; ++i)
			vb2_set_plane_payload(&buf->vb, i, 0);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
}

static void go2001_ctx_error_locked(struct go2001_ctx *ctx)
{
	struct go2001_dev *gdev = ctx->gdev;
	unsigned long flags;

	assert_spin_locked(&gdev->irqlock);

	go2001_err(ctx->gdev, "Setting error state for ctx %p, session_id %d\n",
			ctx, ctx->hw_inst.session_id);

	go2001_cancel_hw_inst_locked(gdev, &ctx->hw_inst);

	spin_lock_irqsave(&ctx->qlock, flags);
	ctx->state = ERROR;
	memset(&ctx->job, 0, sizeof(struct go2001_job));
	go2001_cleanup_queue(ctx, &ctx->src_buf_q);
	go2001_cleanup_queue(ctx, &ctx->src_resume_q);
	go2001_cleanup_queue(ctx, &ctx->dst_buf_q);
	spin_unlock_irqrestore(&ctx->qlock, flags);
}

static void go2001_cancel_all_contexts(struct go2001_dev *gdev)
{
	unsigned long flags;
	struct go2001_ctx *ctx;

	go2001_trace(gdev);
	WARN_ON(!mutex_is_locked(&gdev->lock));

	spin_lock_irqsave(&gdev->irqlock, flags);

	go2001_cancel_all_hw_inst_locked(gdev);

	list_for_each_entry(ctx, &gdev->ctx_list, ctx_entry)
		go2001_ctx_error_locked(ctx);

	spin_unlock_irqrestore(&gdev->irqlock, flags);
	wake_up_all(&gdev->reply_wq);
}

static struct go2001_fmt formats[] = {
	{
		.type = FMT_TYPE_RAW,
		.desc = "32bit ARGB",
		.pixelformat = V4L2_PIX_FMT_RGB32,
		.num_planes = 1,
		.hw_format = GO2001_FMT_RGB888,
		.codec_modes = CODEC_MODE_DECODER | CODEC_MODE_ENCODER,
		.h_align = 1,
		.v_align = 1,
		.plane_row_depth = { 32 },
		.plane_depth = { 32 },
	},
	{
		.type = FMT_TYPE_RAW,
		.desc = "NV12, Y/CbCr interleaved",
		.pixelformat = V4L2_PIX_FMT_NV12M,
		.num_planes = 2,
		.hw_format = GO2001_FMT_YUV420_SEMIPLANAR,
		.codec_modes = CODEC_MODE_DECODER | CODEC_MODE_ENCODER,
		.h_align = 4,
		.v_align = 2,
		.plane_row_depth = { 8, 8 },
		.plane_depth = { 8, 4 },
	},
	{
		.type = FMT_TYPE_RAW,
		.desc = "YUV420M, Y, Cb, Cr 3-planar",
		.pixelformat = V4L2_PIX_FMT_YUV420M,
		.num_planes = 3,
		.hw_format = GO2001_FMT_YUV420_PLANAR,
		.codec_modes = CODEC_MODE_ENCODER,
		.h_align = 4,
		.v_align = 2,
		.plane_row_depth = { 8, 4, 4 },
		.plane_depth = { 8, 2, 2 },
	},
	{
		.type = FMT_TYPE_CODED,
		.desc = "VP8 raw stream",
		.pixelformat = V4L2_PIX_FMT_VP8,
		.num_planes = 1,
		.hw_format = GO2001_FMT_VP8,
		.codec_modes = CODEC_MODE_DECODER | CODEC_MODE_ENCODER,
	},
	{
		.type = FMT_TYPE_CODED,
		.desc = "VP9 raw stream",
		.pixelformat = V4L2_PIX_FMT_VP9,
		.num_planes = 1,
		.hw_format = GO2001_FMT_VP9,
		.codec_modes = CODEC_MODE_DECODER,
	},
};

static struct go2001_ctrl go2001_dec_ctrls[] = {
	{
		.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
		.min = 1,
		.max = 32,
		.step = 1,
		.def = 8,
	},
};

static struct go2001_ctrl go2001_enc_ctrls[] = {
	{
		.id = V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 1,
		.max = 1,
		.step = 1,
		.def = GO2001_DEF_RC_ENABLE,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_BITRATE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 10000,
		.max = 40000000,
		.step = 1,
		.def = GO2001_DEF_BITRATE,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE,
		.name = "Force frame type",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE_DISABLED,
		.max = V4L2_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE_I_FRAME,
		.def = V4L2_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE_DISABLED,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	},
};

static const char * const *go2001_get_qmenu(u32 ctrl_id) {
	static const char * const mfc51_video_force_frame[] = {
		"Disabled",
		"I Frame",
		"Not Coded",
		NULL,
	};

	switch (ctrl_id) {
	case V4L2_CID_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE:
		return mfc51_video_force_frame;
	default:
		return NULL;
	}
}

static struct go2001_fmt *go2001_find_fmt(struct go2001_ctx *ctx,
						__u32 pixelformat)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (formats[i].pixelformat == pixelformat
				&& (ctx->codec_mode & formats[i].codec_modes))
			return &formats[i];
	}

	go2001_dbg(ctx->gdev, 1, "Unsupported format %d.\n", pixelformat);
	return NULL;
}

struct vb2_queue *go2001_get_vq(struct go2001_ctx *ctx, enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->src_vq : &ctx->dst_vq;
}

static int go2001_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct go2001_dev *gdev = video_drvdata(file);

	strlcpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, "GO2001 PCIe codec", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s",
			pci_name(gdev->pdev));

	cap->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING
			 | V4L2_CAP_VIDEO_CAPTURE_MPLANE
			 | V4L2_CAP_VIDEO_OUTPUT_MPLANE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int go2001_queue_setup(struct vb2_queue *q,
			const struct v4l2_format *fmt,
			unsigned int *num_buffers, unsigned int *num_planes,
			unsigned int sizes[], void *alloc_ctxs[])
{
	struct go2001_ctx *ctx = vb2_get_drv_priv(q);
	struct go2001_fmt *f;
	int i;

	go2001_trace(ctx->gdev);

	if (fmt) {
		go2001_err(ctx->gdev, "VIDIOC_CREATE_BUFS not supported\n");
		return -EINVAL;
	}

	f = V4L2_TYPE_IS_OUTPUT(q->type) ? ctx->src_fmt : ctx->dst_fmt;
	if (!f) {
		go2001_err(ctx->gdev, "Format(s) not selected\n");
		return -EINVAL;
	}

	if (f->type == FMT_TYPE_CODED) {
		if (*num_buffers < GO2001_MIN_NUM_BITSTREAM_BUFFERS)
			*num_buffers = GO2001_MIN_NUM_BITSTREAM_BUFFERS;
		*num_planes = 1;
		sizes[0] = ctx->bitstream_buf_size;
	} else if (go2001_has_frame_info(ctx)) {
		if (*num_buffers < GO2001_MIN_NUM_FRAME_BUFFERS)
			*num_buffers = GO2001_MIN_NUM_FRAME_BUFFERS;
		*num_planes = f->num_planes;
		for (i = 0; i < f->num_planes; ++i)
			sizes[i] = ctx->finfo.plane_size[i];
	} else {
		go2001_err(ctx->gdev, "Invalid state\n");
		return -EINVAL;
	}

	for (i = 0; i < *num_planes; ++i)
		alloc_ctxs[i] = ctx->gdev->alloc_ctx;

	go2001_dbg(ctx->gdev, 2, "Num buffers: %d, planes: %d\n",
			*num_buffers, *num_planes);
	for (i = 0; i < f->num_planes; ++i)
		go2001_dbg(ctx->gdev, 2, "Plane %d, size: %d\n", i, sizes[i]);

	return 0;
}

static int go2001_buf_init(struct vb2_buffer *vb)
{
	struct go2001_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct go2001_dev *gdev = ctx->gdev;
	struct device *dev = &gdev->pdev->dev;
	struct go2001_buffer *gbuf = vb_to_go2001_buf(vb);
	struct go2001_dma_desc *dma_desc;
	struct go2001_mmap_list_entry *mmap_list;
	enum dma_data_direction dir;
	struct scatterlist *sg;
	struct sg_table *sgt;
	u64 dma_addr;
	u32 dma_len;
	int plane, sgi;
	int ret;

	go2001_trace(gdev);
	BUG_ON(gbuf->mapped);

	dir = V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) ?
		DMA_TO_DEVICE : DMA_FROM_DEVICE;

	for (plane = 0; plane < vb->num_planes; ++plane) {
		dma_desc = &gbuf->dma_desc[plane];
		WARN_ON(!IS_ALIGNED(dma_desc->map_addr, 16));
		sgt = vb2_dma_sg_plane_desc(vb, plane);
		if (!IS_ALIGNED(sgt->sgl->offset, 8) ||
				!IS_ALIGNED(vb2_plane_size(vb, plane), 8)) {
			go2001_err(gdev, "Plane address/size not aligned "
					"%d/%zu\n", sgt->sgl->offset,
					vb2_plane_size(vb, plane));
			ret = -EINVAL;
			goto err;
		}

		dma_desc->num_entries = sgt->nents;
		dma_desc->list_size = dma_desc->num_entries
					* sizeof(struct go2001_mmap_list_entry);
		dma_desc->mmap_list = dma_alloc_coherent(dev,
			dma_desc->list_size, &dma_desc->dma_addr, GFP_KERNEL);
		if (!dma_desc->mmap_list) {
			go2001_err(gdev, "Failed allocating HW memory map\n");
			ret = -ENOMEM;
			goto err;
		}

		go2001_dbg(gdev, 3, "Plane %d: mmap list size: %zu\n",
				plane, dma_desc->list_size);

		mmap_list = dma_desc->mmap_list;
		for_each_sg(sgt->sgl, sg, dma_desc->num_entries, sgi) {
			dma_addr = sg_dma_address(sg);
			dma_len = sg_dma_len(sg);

			mmap_list[sgi].dma_addr = cpu_to_le64(dma_addr);
			mmap_list[sgi].size = cpu_to_le32(dma_len);

			go2001_dbg(gdev, 4, "Chunk %d: 0x%08llx, size %d\n",
					sgi, dma_addr, dma_len);
		}
	}

	ret = go2001_map_buffer(ctx, gbuf);
	if (ret) {
		go2001_err(ctx->gdev, "Failed mapping buffer in HW\n");
		goto err;
	}

	return 0;
err:
	for (; plane > 0; --plane) {
		dma_desc = &gbuf->dma_desc[plane - 1];
		dma_free_coherent(dev, dma_desc->list_size, dma_desc->mmap_list,
					dma_desc->dma_addr);
		memset(dma_desc, 0, sizeof(struct go2001_dma_desc));
	}

	return ret;
}

static int go2001_buf_prepare(struct vb2_buffer *vb)
{
	struct go2001_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct go2001_buffer *gbuf = vb_to_go2001_buf(vb);

	return go2001_prepare_gbuf(ctx, gbuf,
				V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type));
}

static void go2001_buf_finish(struct vb2_buffer *vb)
{
	struct go2001_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct go2001_buffer *gbuf = vb_to_go2001_buf(vb);
	size_t plane_size;
	void *vaddr, *ptr;
	int i;

	go2001_finish_gbuf(ctx, gbuf);

	/*
	 * If this is a CAPTURE encoder buffer, we need to reassemble
	 * partitions, as there may be gaps between them.
	 */
	if (ctx->codec_mode != CODEC_MODE_ENCODER ||
			V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		return;

	plane_size = vb2_plane_size(vb, 0);
	vaddr = vb2_plane_vaddr(vb, 0);
	if (!vaddr) {
		go2001_err(ctx->gdev, "Unable to map buffer\n");
		return;
	}

	ptr = vaddr;

	/*
	 * ith partition resides at partition_off[i] from start of the buffer
	 * and is of size partition_size[i]. As there may be gaps between
	 * partitions, slide them back together to remove the gaps.
	 */
	for (i = 0; i < VP8_MAX_NUM_PARTITIONS; ++i) {
		if (gbuf->partition_size[i] == 0)
			break;

		if (gbuf->partition_off[i] + gbuf->partition_size[i] >
				plane_size) {
			go2001_err(ctx->gdev, "Invalid partition %d info, off: "
					"0x%zx, size: %zu, plane_size: %zu\n",
					i, gbuf->partition_off[i],
					gbuf->partition_size[i],
					plane_size);
			return;
		}

		if (ptr != vaddr + gbuf->partition_off[i])
			memmove(ptr, vaddr + gbuf->partition_off[i],
				gbuf->partition_size[i]);
		ptr += gbuf->partition_size[i];
	}
}

static void go2001_buf_cleanup(struct vb2_buffer *vb)
{
	struct go2001_buffer *gbuf = vb_to_go2001_buf(vb);
	struct go2001_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct go2001_dev *gdev = ctx->gdev;
	struct device *dev = &gdev->pdev->dev;
	struct go2001_dma_desc *dma_desc;
	enum dma_data_direction dir;
	int plane;

	dir = V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) ?
		DMA_TO_DEVICE : DMA_FROM_DEVICE;

	go2001_unmap_buffer(ctx, gbuf);

	/* Clean up regardless of whether unmap in HW succeeded or not. */
	for (plane = 0; plane < vb->num_planes; ++plane) {
		dma_desc = &gbuf->dma_desc[plane];
		if (dma_desc->mmap_list) {
			dma_free_coherent(dev, dma_desc->list_size,
					  dma_desc->mmap_list,
					  dma_desc->dma_addr);
		}

		memset(dma_desc, 0, sizeof(struct go2001_dma_desc));
	}
}

static int go2001_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct go2001_ctx *ctx = vb2_get_drv_priv(q);
	unsigned long flags;
	int ret = 0;

	go2001_trace(ctx->gdev);

	if (ctx->state == ERROR) {
		go2001_dbg(ctx->gdev, 1, "Instance %p in error state\n", ctx);
		return -EIO;
	}

	if (!V4L2_TYPE_IS_OUTPUT(q->type))
		return 0;

	spin_lock_irqsave(&ctx->qlock, flags);
	if (ctx->state == PAUSED)
		ctx->state = RUNNING;
	spin_unlock_irqrestore(&ctx->qlock, flags);

	ret = go2001_schedule_frames(ctx);
	if (ret) {
		go2001_err(ctx->gdev, "Failed to start streaming\n");
		go2001_ctx_error(ctx);
	}

	return ret;
}

static void go2001_stop_streaming(struct vb2_queue *q)
{
	struct go2001_ctx *ctx = vb2_get_drv_priv(q);
	unsigned long flags;

	go2001_trace(ctx->gdev);

	spin_lock_irqsave(&ctx->qlock, flags);
	if (ctx->state == RUNNING)
		ctx->state = PAUSED;
	spin_unlock_irqrestore(&ctx->qlock, flags);

	go2001_wait_for_ctx_done(ctx);

	spin_lock_irqsave(&ctx->qlock, flags);
	WARN_ON(ctx->job.src_buf || ctx->job.dst_buf);

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		go2001_cleanup_queue(ctx, &ctx->src_buf_q);
		go2001_cleanup_queue(ctx, &ctx->src_resume_q);
	} else {
		go2001_cleanup_queue(ctx, &ctx->dst_buf_q);
	}
	spin_unlock_irqrestore(&ctx->qlock, flags);
}

static void go2001_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct go2001_ctx *ctx = vb2_get_drv_priv(vq);
	struct go2001_buffer *gbuf = vb_to_go2001_buf(vb);
	unsigned long flags;

	if (ctx->codec_mode == CODEC_MODE_ENCODER
			&& V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		gbuf->rt_enc_params = ctx->pending_rt_params;
		memset(&ctx->pending_rt_params, 0,
				sizeof(ctx->pending_rt_params));
	}

	spin_lock_irqsave(&ctx->qlock, flags);
	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		list_add_tail(&gbuf->list, &ctx->src_buf_q);
	else
		list_add_tail(&gbuf->list, &ctx->dst_buf_q);
	spin_unlock_irqrestore(&ctx->qlock, flags);

	go2001_schedule_frames(ctx);
}

static const struct vb2_ops go2001_qops = {
	.queue_setup = go2001_queue_setup,

	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,

	.buf_init = go2001_buf_init,
	.buf_prepare = go2001_buf_prepare,
	.buf_finish = go2001_buf_finish,
	.buf_cleanup = go2001_buf_cleanup,

	.start_streaming = go2001_start_streaming,
	.stop_streaming = go2001_stop_streaming,

	.buf_queue = go2001_buf_queue,
};

static int go2001_init_vb2_queue(struct vb2_queue *q, struct go2001_ctx *ctx,
					enum v4l2_buf_type type)
{
	q->type = type;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_USERPTR;
	q->lock = &ctx->lock;
	q->ops = &go2001_qops;
	q->mem_ops = &vb2_dma_sg_memops;
	q->drv_priv = ctx;
	q->buf_struct_size = sizeof(struct go2001_buffer);
	q->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_COPY;

	return vb2_queue_init(q);
}

static int go2001_dec_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	/* TODO */
	return 0;
}

static int go2001_enc_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	/* TODO */
	return 0;
}

static int go2001_enc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct go2001_ctx *ctx = ctrl_to_ctx(ctrl);

	go2001_trace(ctx->gdev);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		break;

	case V4L2_CID_MPEG_VIDEO_BITRATE:
		set_bit(GO2001_BITRATE_CHANGE,
				ctx->pending_rt_params.changed_mask);
		ctx->pending_rt_params.enc_params.bitrate = ctrl->val;
		go2001_dbg(ctx->gdev, 2, "Bitrate changed to %d bps.\n",
				ctx->pending_rt_params.enc_params.bitrate);
		break;

	case V4L2_CID_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE: {
		bool keyframe = (ctrl->val ==
			V4L2_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE_I_FRAME);
		ctx->pending_rt_params.enc_params.request_keyframe = keyframe;
		if (keyframe) {
			set_bit(GO2001_KEYFRAME_REQUESTED,
					ctx->pending_rt_params.changed_mask);
			go2001_dbg(ctx->gdev, 2, "Keyframe requested.\n");
		}
		break;
	}

	default:
		go2001_dbg(ctx->gdev, 1,
			"Unsupported control id=%d, ignoring.\n", ctrl->id);
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops go2001_dec_ctrl_ops = {
	.g_volatile_ctrl = go2001_dec_g_volatile_ctrl,
};

static const struct v4l2_ctrl_ops go2001_enc_ctrl_ops = {
	.s_ctrl = go2001_enc_s_ctrl,
	.g_volatile_ctrl = go2001_enc_g_volatile_ctrl,
};

static int go2001_init_ctrl_handler(struct go2001_ctx *ctx)
{
	struct v4l2_ctrl_handler *hdl = &ctx->ctrl_handler;
	const struct v4l2_ctrl_ops *ops;
	struct go2001_ctrl *ctrls;
	struct go2001_ctrl *ctrl;
	int num_ctrls;
	int ret;
	int i;

	if (ctx->codec_mode == CODEC_MODE_DECODER) {
		ctrls = go2001_dec_ctrls;
		num_ctrls = ARRAY_SIZE(go2001_dec_ctrls);
		ops = &go2001_dec_ctrl_ops;
	} else {
		ctrls = go2001_enc_ctrls;
		num_ctrls = ARRAY_SIZE(go2001_enc_ctrls);
		ops = &go2001_enc_ctrl_ops;
	}

	ret = v4l2_ctrl_handler_init(hdl, num_ctrls);
	if (ret)
		return ret;

	for (i = 0; i < num_ctrls; ++i) {
		ctrl = &ctrls[i];
		if (V4L2_CTRL_DRIVER_PRIV(ctrl->id)) {
			struct v4l2_ctrl_config cfg;
			memset(&cfg, 0, sizeof(struct v4l2_ctrl_config));
			cfg.ops = ops;
			cfg.id = ctrl->id;
			cfg.name = ctrl->name;
			cfg.type = ctrl->type;
			cfg.min = ctrl->min;
			cfg.max = ctrl->max;
			cfg.def = ctrl->def;
			cfg.step = ctrl->step;
			if (cfg.type == V4L2_CTRL_TYPE_MENU)
				cfg.qmenu = go2001_get_qmenu(ctrl->id);
			cfg.flags = ctrl->flags;
			v4l2_ctrl_new_custom(hdl, &cfg, NULL);
		} else if (ctrl->type == V4L2_CTRL_TYPE_MENU) {
			v4l2_ctrl_new_std_menu(hdl, ops, ctrl->id, ctrl->max,
						0, ctrl->def);
		} else {
			v4l2_ctrl_new_std(hdl, ops, ctrl->id, ctrl->min,
					ctrl->max, ctrl->step, ctrl->def);
		}
	}

	ret = hdl->error;
	if (ret)
		v4l2_ctrl_handler_free(hdl);
	else
		ctx->v4l2_fh.ctrl_handler = hdl;

	return ret;
}

static int go2001_init_ctx(struct go2001_dev *gdev, struct go2001_ctx *ctx,
				struct file *file, struct video_device *vdev,
				enum go2001_codec_mode mode)
{
	int ret;

	ctx->gdev = gdev;
	ctx->codec_mode = mode;

	mutex_init(&ctx->lock);
	spin_lock_init(&ctx->qlock);
	INIT_LIST_HEAD(&ctx->src_buf_q);
	INIT_LIST_HEAD(&ctx->src_resume_q);
	INIT_LIST_HEAD(&ctx->dst_buf_q);
	v4l2_fh_init(&ctx->v4l2_fh, vdev);
	ctx->state = UNINITIALIZED;
	go2001_init_hw_inst(&ctx->hw_inst, 0);

	ctx->bitstream_buf_size = GO2001_DEF_BITSTREAM_BUFFER_SIZE;
	if (ctx->codec_mode == CODEC_MODE_DECODER) {
		ctx->src_fmt = go2001_find_fmt(ctx, V4L2_PIX_FMT_VP8);
		ctx->dst_fmt = go2001_find_fmt(ctx, V4L2_PIX_FMT_RGB32);
	} else {
		ctx->src_fmt = go2001_find_fmt(ctx, V4L2_PIX_FMT_NV12M);
		ctx->dst_fmt = go2001_find_fmt(ctx, V4L2_PIX_FMT_VP8);
		ctx->enc_params.rc_enable = GO2001_DEF_RC_ENABLE;
		ctx->enc_params.request_keyframe = true;
		ctx->enc_params.bitrate = GO2001_DEF_BITRATE;
		ctx->enc_params.framerate_num = GO2001_MAX_FPS;
		ctx->enc_params.framerate_denom = 1;
	}

	ret = go2001_init_ctrl_handler(ctx);
	if (ret)
		return ret;

	ret = go2001_init_vb2_queue(&ctx->src_vq, ctx,
					V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (ret)
		return ret;

	ret = go2001_init_vb2_queue(&ctx->dst_vq, ctx,
					V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (ret)
		return ret;

	mutex_lock(&gdev->lock);
	list_add_tail(&ctx->ctx_entry, &gdev->ctx_list);
	mutex_unlock(&gdev->lock);

	file->private_data = &ctx->v4l2_fh;
	v4l2_fh_add(&ctx->v4l2_fh);

	return 0;
}

static void go2001_release_ctx(struct go2001_ctx *ctx)
{
	unsigned long flags;

	vb2_queue_release(&ctx->src_vq);
	vb2_queue_release(&ctx->dst_vq);

	go2001_release_codec(ctx);

	mutex_lock(&ctx->gdev->lock);
	spin_lock_irqsave(&ctx->qlock, flags);
	list_del(&ctx->ctx_entry);
	spin_unlock_irqrestore(&ctx->qlock, flags);
	mutex_unlock(&ctx->gdev->lock);

	v4l2_fh_del(&ctx->v4l2_fh);
	v4l2_fh_exit(&ctx->v4l2_fh);

	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
}

static int go2001_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct go2001_dev *gdev = video_drvdata(file);
	struct go2001_ctx *ctx;
	enum go2001_codec_mode mode;
	int ret = 0;

	go2001_trace(gdev);

	mutex_lock(&gdev->lock);
	if (!gdev->initialized) {
		ret = go2001_init(gdev);
		if (!ret)
			gdev->initialized = true;
	}
	mutex_unlock(&gdev->lock);

	if (ret)
		return ret;

	if (vdev->index == gdev->dec_vdev.index) {
		mode = CODEC_MODE_DECODER;
	} else if (vdev->index == gdev->enc_vdev.index) {
		mode = CODEC_MODE_ENCODER;
	} else {
		go2001_err(gdev, "Invalid video node\n");
		return -ENXIO;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		go2001_err(gdev, "Failed allocating context\n");
		return -ENOMEM;
	}

	ret = go2001_init_ctx(gdev, ctx, file, vdev, mode);
	if (ret) {
		go2001_err(gdev, "Failed initializing context\n");
		kfree(ctx);
		return ret;
	}

	go2001_dbg(ctx->gdev, 1, "Opened ctx %p\n", ctx);
	return 0;
}

static int go2001_release(struct file *file)
{
	struct go2001_ctx *ctx = fh_to_ctx(file->private_data);

	go2001_dbg(ctx->gdev, 1, "Releasing ctx %p\n", ctx);

	go2001_release_ctx(ctx);
	kfree(ctx);

	return 0;
}

static unsigned int go2001_poll(struct file *file,
				struct poll_table_struct *wait)
{
	unsigned long req_events = poll_requested_events(wait);
	struct go2001_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *src_q = &ctx->src_vq;
	struct vb2_queue *dst_q = &ctx->dst_vq;
	struct vb2_buffer *src_vb = NULL, *dst_vb = NULL;
	struct v4l2_fh *fh = &ctx->v4l2_fh;
	unsigned long flags;
	unsigned int rc = 0;

	if (v4l2_event_pending(fh))
		rc = POLLPRI;
	else if (req_events & POLLPRI)
		poll_wait(file, &fh->wait, wait);

	if (!(req_events & (POLLOUT | POLLWRNORM | POLLIN | POLLRDNORM)))
		return rc;

	if ((!vb2_is_streaming(src_q) || list_empty(&src_q->queued_list))
		&& (!vb2_is_streaming(dst_q)
					|| list_empty(&dst_q->queued_list))) {
		rc |= POLLERR;
		return rc;
	}

	poll_wait(file, &src_q->done_wq, wait);
	poll_wait(file, &dst_q->done_wq, wait);

	spin_lock_irqsave(&src_q->done_lock, flags);
	if (!list_empty(&src_q->done_list))
		src_vb = list_first_entry(&src_q->done_list, struct vb2_buffer,
						done_entry);
	if (src_vb && (src_vb->state == VB2_BUF_STATE_DONE
			|| src_vb->state == VB2_BUF_STATE_ERROR))
		rc |= POLLOUT | POLLWRNORM;
	spin_unlock_irqrestore(&src_q->done_lock, flags);

	spin_lock_irqsave(&dst_q->done_lock, flags);
	if (!list_empty(&dst_q->done_list))
		dst_vb = list_first_entry(&dst_q->done_list, struct vb2_buffer,
						done_entry);
	if (dst_vb && (dst_vb->state == VB2_BUF_STATE_DONE
			|| dst_vb->state == VB2_BUF_STATE_ERROR))
		rc |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&dst_q->done_lock, flags);

	return rc;
}

static int go2001_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct go2001_ctx *ctx = fh_to_ctx(file->private_data);
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	int ret;

	if (offset < DST_QUEUE_OFF_BASE) {
		ret = vb2_mmap(&ctx->src_vq, vma);
	} else {
		vma->vm_pgoff -= (DST_QUEUE_OFF_BASE >> PAGE_SHIFT);
		ret = vb2_mmap(&ctx->dst_vq, vma);
	}

	return ret;
}

static void go2001_handle_init_reply(struct go2001_dev *gdev,
					struct go2001_msg *msg)
{
	struct go2001_msg_hdr *hdr = msg_to_hdr(msg);
	struct go2001_init_decoder_reply *reply = msg_to_param(msg);
	struct go2001_hw_inst *hw_inst;
	struct go2001_ctx *ctx = NULL;

	list_for_each_entry(hw_inst, &gdev->new_inst_list, inst_entry) {
		if (hw_inst->sequence_id == gdev->last_reply_seq_id) {
			ctx = hw_inst_to_ctx(hw_inst);
			break;
		}
	}

	if (!ctx) {
		go2001_err(gdev, "No ctx awaiting VM_INIT_DECODER, dropping\n");
		return;
	}

	if (gdev->curr_hw_inst == &ctx->hw_inst)
		gdev->curr_hw_inst = NULL;
	list_del(&ctx->hw_inst.inst_entry);

	go2001_init_hw_inst(&ctx->hw_inst, reply->session_id);

	if (hdr->status == GO2001_STATUS_OK)
		list_add_tail(&ctx->hw_inst.inst_entry, &ctx->gdev->inst_list);
	else
		go2001_ctx_error(ctx);
}

static int go2001_handle_release_instance_reply(struct go2001_ctx *ctx)
{
	unsigned long flags;

	spin_lock_irqsave(&ctx->qlock, flags);
	ctx->state = UNINITIALIZED;
	spin_unlock_irqrestore(&ctx->qlock, flags);
	return 0;
}

static void go2001_calc_finfo(struct go2001_ctx *ctx, struct go2001_fmt *fmt,
				struct go2001_frame_info *finfo,
				 unsigned int width, unsigned int height)
{
	int i;

	finfo->width = round_up(width, fmt->h_align);
	finfo->height = round_up(height, fmt->v_align);
	finfo->coded_width = round_up(finfo->width, GO2001_VPX_MACROBLOCK_SIZE);
	finfo->coded_height = round_up(finfo->height,
					GO2001_VPX_MACROBLOCK_SIZE);

	go2001_dbg(ctx->gdev, 2, "Visible (coded) resolution: %ux%u (%ux%u)\n",
			finfo->width, finfo->height,
			finfo->coded_width, finfo->coded_height);

	for (i = 0; i < fmt->num_planes; ++i) {
		finfo->bytesperline[i] =
			(finfo->coded_width * fmt->plane_row_depth[i]) / 8;
		finfo->plane_size[i] = (finfo->coded_width * finfo->coded_height
				     * fmt->plane_depth[i]) / 8;
		go2001_dbg(ctx->gdev, 2, "Plane %d: bpl: %u, size: %zu\n",
				i, finfo->bytesperline[i],
				finfo->plane_size[i]);
	}
}

static int go2001_handle_new_info(struct go2001_ctx *ctx,
					struct go2001_get_info_reply *reply)
{
	struct go2001_frame_info *finfo = &ctx->finfo;
	struct go2001_fmt *fmt;

	fmt = (ctx->codec_mode == CODEC_MODE_DECODER) ?
		ctx->dst_fmt : ctx->src_fmt;

	go2001_dbg(ctx->gdev, 2, "HW reports new resolution: %ux%u (%ux%u)\n",
			reply->visible_width, reply->visible_height,
			reply->coded_width, reply->coded_height);

	go2001_calc_finfo(ctx, fmt, finfo, reply->visible_width,
				reply->visible_height);

	if (finfo->width != reply->visible_width
			|| finfo->height != reply->visible_height
			|| finfo->coded_width != reply->coded_width
			|| finfo->coded_height != reply->coded_height) {
		go2001_err(ctx->gdev, "Invalid resolution from the HW\n");
		return -EINVAL;
	}

	return 0;
}

static int go2001_handle_get_info_reply(struct go2001_ctx *ctx,
					struct go2001_msg *msg)
{
	struct go2001_get_info_reply *reply = msg_to_param(msg);
	return go2001_handle_new_info(ctx, reply);
}

static int go2001_fill_dst_buf_info(struct go2001_ctx *ctx,
		struct go2001_job *job, struct go2001_msg *reply, bool error) {
	struct vb2_buffer *src_vb = &job->src_buf->vb;
	struct vb2_buffer *dst_vb = &job->dst_buf->vb;
	int i;

	switch (ctx->codec_mode) {
	case CODEC_MODE_DECODER:
		for (i = 0; i < dst_vb->num_planes; ++i) {
			vb2_set_plane_payload(dst_vb, i,
					error ? 0 : ctx->finfo.plane_size[i]);
		}
		break;

	case CODEC_MODE_ENCODER: {
		struct go2001_empty_buffer_enc_reply *enc_reply =
			msg_to_param(reply);
		struct go2001_buffer *gbuf = vb_to_go2001_buf(dst_vb);

		memset(gbuf->partition_off, 0, sizeof(gbuf->partition_off));
		memset(gbuf->partition_size, 0, sizeof(gbuf->partition_size));

		/*
		 * Partitions may have gaps between them, we will have to
		 * reassemble the bitstream later in buf_finish. For now just
		 * save the size and offset of each partition and set the
		 * payload for the buffer to the sum of sizes (returned in
		 * enc_reply->payload_size).
		 */
		if (enc_reply->payload_size > vb2_plane_size(dst_vb, 0)
								|| error) {
			vb2_set_plane_payload(dst_vb, 0, 0);
			return -EINVAL;
		}

		vb2_set_plane_payload(dst_vb, 0, enc_reply->payload_size);

		for (i = 0; i < VP8_MAX_NUM_PARTITIONS; ++i) {
			gbuf->partition_off[i] = enc_reply->partition_off[i];
			gbuf->partition_size[i] = enc_reply->partition_size[i];
		}

		dst_vb->v4l2_buf.flags = 0;
		if (enc_reply->frame_type
				== GO2001_EMPTY_BUF_ENC_FRAME_KEYFRAME)
			dst_vb->v4l2_buf.flags = V4L2_BUF_FLAG_KEYFRAME;
		break;
	}
	default:
		WARN_ON(1);
		break;
	}

	dst_vb->v4l2_buf.timecode = src_vb->v4l2_buf.timecode;
	dst_vb->v4l2_buf.timestamp = src_vb->v4l2_buf.timestamp;

	go2001_dbg(ctx->gdev, 5, "Returning frame ts=%ld.%06ld\n",
			dst_vb->v4l2_buf.timestamp.tv_sec,
			dst_vb->v4l2_buf.timestamp.tv_usec);
	return 0;
}

static int go2001_handle_empty_buffer_reply(struct go2001_ctx *ctx,
						struct go2001_msg *msg)
{
	enum vb2_buffer_state src_state = VB2_BUF_STATE_DONE;
	enum vb2_buffer_state dst_state = VB2_BUF_STATE_DONE;
	struct go2001_job *job;
	struct go2001_buffer *src_buf;
	struct go2001_msg_hdr *hdr = msg_to_hdr(msg);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&ctx->qlock, flags);
	job = &ctx->job;
	src_buf = job->src_buf;
	if (!src_buf) {
		go2001_err(ctx->gdev, "No src buffer!\n");
		ret = -EIO;
		goto out;
	}

	list_del(&src_buf->list);

	switch (hdr->status) {
	case GO2001_STATUS_NEW_PICTURE_SIZE: {
		struct v4l2_event ev;
		struct go2001_empty_buffer_dec_reply *dec_reply =
							msg_to_param(msg);
		WARN_ON(ctx->state != NEED_HEADER_INFO
				&& ctx->state != RUNNING);

		go2001_handle_new_info(ctx, &dec_reply->info);
		memset(&ev, 0, sizeof(struct v4l2_event));
		ev.type = V4L2_EVENT_RESOLUTION_CHANGE;
		v4l2_event_queue_fh(&ctx->v4l2_fh, &ev);
		ctx->state = RES_CHANGE;
		/* Fallthrough */
	}
	case GO2001_STATUS_WAITING_PICTURE_SIZE_CHANGED:
		ctx->need_resume = true;
		/*
		 * We will retry this frame after reallocating output buffers.
		 * Add the source buffer to the resume queue, which will
		 * be spliced in front of the source queue after reallocation.
		 */
		go2001_dbg(ctx->gdev, 3, "Pending buffer %p\n", src_buf);
		list_add_tail(&src_buf->list, &ctx->src_resume_q);
		break;

	case GO2001_STATUS_STREAM_ERROR:
		src_state = VB2_BUF_STATE_ERROR;
		/* Fallthrough */
	case GO2001_STATUS_NO_OUTPUT:
		dst_state = VB2_BUF_STATE_ERROR;
		/* Fallthrough */
	default:
		vb2_buffer_done(&src_buf->vb, src_state);
		if (job->dst_buf) {
			if (go2001_fill_dst_buf_info(ctx, job, msg,
					dst_state == VB2_BUF_STATE_ERROR))
				dst_state = VB2_BUF_STATE_ERROR;
			list_del(&job->dst_buf->list);
			vb2_buffer_done(&job->dst_buf->vb, dst_state);
		}
		break;
	}
out:
	job->src_buf = NULL;
	job->dst_buf = NULL;
	spin_unlock_irqrestore(&ctx->qlock, flags);
	return ret;
}

static void go2001_handle_get_version_reply(struct go2001_dev *gdev,
						struct go2001_msg *msg)
{
	struct go2001_get_version_reply *reply = msg_to_param(msg);

	dev_info(&gdev->pdev->dev, "GO2001 ver: %d/%d, VP8 decoder: %d/%d "
			"VP8 encoder: %d/%d, VP9 decoder: %d/%d\n",
			reply->hw_ver, reply->sw_ver,
			reply->vp8dec_hw_ver, reply->vp8dec_sw_ver,
			reply->vp8enc_hw_ver, reply->vp8enc_sw_ver,
			reply->vp9dec_hw_ver, reply->vp9dec_sw_ver);
}

static struct go2001_hw_inst *find_hw_inst_by_id_locked(struct go2001_dev *gdev,
							u32 session_id)
{
	struct go2001_hw_inst *hw_inst;

	list_for_each_entry(hw_inst, &gdev->inst_list, inst_entry)
		if (hw_inst->session_id == session_id)
			return hw_inst;
	return NULL;
}

static int go2001_process_reply(struct go2001_dev *gdev,
				struct go2001_msg *reply)
{
	struct go2001_hw_inst *hw_inst;
	struct go2001_ctx *ctx = NULL;
	struct go2001_msg_hdr *hdr = msg_to_hdr(reply);
	int ret = 0;

	hw_inst = find_hw_inst_by_id_locked(gdev, hdr->session_id);
	if (!hw_inst) {
		go2001_err(gdev, "Got reply for an invalid instance id %d\n",
				hdr->session_id);
		return -EIO;
	}

	if (hdr->type != GO2001_VM_EVENT_ASSERT
			&& hdr->type != GO2001_VM_EVENT_LOG) {
		if (WARN_ON(gdev->msgs_in_flight == 0)) {
			go2001_err(gdev,
					"Unexpected reply without a request\n");
			return -EIO;
		}

		cancel_delayed_work(&gdev->watchdog_work);
		--gdev->msgs_in_flight;
		if (gdev->msgs_in_flight > 0) {
			schedule_delayed_work(&gdev->watchdog_work,
				msecs_to_jiffies(GO2001_WATCHDOG_TIMEOUT_MS));
		}
	}

	hw_inst->last_reply_seq_id = hdr->sequence_id;

	switch (hdr->status) {
	case GO2001_STATUS_OK:
		break;

	case GO2001_STATUS_NEW_PICTURE_SIZE:
	case GO2001_STATUS_WAITING_PICTURE_SIZE_CHANGED:
	case GO2001_STATUS_STREAM_ERROR:
	case GO2001_STATUS_NO_OUTPUT:
		WARN_ON(hdr->type != GO2001_VM_EMPTY_BUFFER);
		/* These will be handled in VM_EMPTY_BUFFER handler */
		break;

	case GO2001_STATUS_RES_NA:
		go2001_err(gdev, "Hardware ran out of resources\n");
		ret = -ENOMEM;
		break;

	case GO2001_STATUS_INVALID_PARAM:
		go2001_err(gdev, "Invalid parameters\n");
		ret = -EINVAL;
		break;

	default:
		go2001_err(gdev, "Unhandled error in reply\n");
		ret = -EIO;
		break;
	}

	gdev->last_reply_inst_id = hdr->session_id;
	gdev->last_reply_seq_id = hdr->sequence_id;

	if (hdr->session_id != 0)
		ctx = hw_inst_to_ctx(hw_inst);

	switch (hdr->type) {
	case GO2001_VM_INIT_DECODER:
	case GO2001_VM_INIT_ENCODER:
		go2001_dbg(gdev, 5, "VM_INIT reply\n");
		go2001_handle_init_reply(gdev, reply);
		break;

	case GO2001_VM_GET_VERSION:
		go2001_dbg(gdev, 5, "VM_GET_VERSION reply\n");
		go2001_handle_get_version_reply(gdev, reply);
		break;

	case GO2001_VM_SET_MMAP:
	case GO2001_VM_RELEASE_MMAP:
		go2001_dbg(gdev, 5, "VM_*_MMAP reply\n");
		break;

	case GO2001_VM_EMPTY_BUFFER:
		go2001_dbg(gdev, 5, "VM_EMPTY_BUFFER reply\n");
		ret = go2001_handle_empty_buffer_reply(ctx, reply);
		break;

	case GO2001_VM_GET_INFO:
		go2001_dbg(gdev, 5, "VM_GET_INFO reply\n");
		ret = go2001_handle_get_info_reply(ctx, reply);
		break;

	case GO2001_VM_SET_CTRL:
		go2001_dbg(gdev, 5, "VM_SET_CTRL reply\n");
		break;

	case GO2001_VM_RELEASE:
		go2001_dbg(gdev, 5, "VM_RELEASE reply\n");
		go2001_handle_release_instance_reply(ctx);
		break;

	case GO2001_VM_DEC_SET_OUT_FMT:
		go2001_dbg(gdev, 5, "VM_DEC_SET_OUT_FMT reply\n");
		ctx->format_set = true;
		break;

	case GO2001_VM_EVENT_ASSERT: {
		struct go2001_event_assert_reply *a = msg_to_param(reply);
		a->filename[ARRAY_SIZE(a->filename) - 1] = '\0';
		a->funcname[ARRAY_SIZE(a->funcname) - 1] = '\0';
		a->expr[ARRAY_SIZE(a->expr) - 1] = '\0';
		go2001_err(gdev, "FW ASSERT at %s:%d in %s, executing %s\n",
				a->filename, a->line_no, a->funcname, a->expr);
		ret = -EIO;
		break;
	}

	case GO2001_VM_EVENT_LOG: {
		struct go2001_event_log_reply *l = msg_to_param(reply);
		l->data[ARRAY_SIZE(l->data) - 1] = '\0';
		go2001_err(gdev, "VM_EVENT_LOG: %s\n", l->data);
		break;
	}

	case GO2001_VM_SET_LOG_LEVEL: {
		struct go2001_set_log_level_reply *l = msg_to_param(reply);
		go2001_dbg(gdev, 1, "VM_SET_LOG_LEVEL: %d\n", l->level);
		break;
	}

	default:
		go2001_err(gdev, "Unexpected reply [%d:%d], type=0x%x\n",
				hdr->session_id, hdr->sequence_id, hdr->type);
		ret = -EIO;
		break;
	}

	if (ret) {
		go2001_err(gdev,
			"Error %d for reply [%d:%d] type=0x%x, status=0x%x\n",
			ret, hdr->session_id, hdr->sequence_id, hdr->type,
			hdr->status);
		if (ctx)
			go2001_ctx_error(ctx);
	}

	if (ctx && ret == 0)
		go2001_schedule_frames(ctx);
	/*
	 * Critical failure only on EIO, otherwise we may be able to continue
	 * using other instances.
	 */
	return ret == -EIO ? ret : 0;
}

static void go2001_watchdog(struct work_struct *work)
{
	struct go2001_dev *gdev = container_of(to_delayed_work(work),
					struct go2001_dev, watchdog_work);
	int ret;

	go2001_err(gdev, "Watchdog resetting firmware\n");

	mutex_lock(&gdev->lock);

	go2001_cancel_all_contexts(gdev);
	gdev->msgs_in_flight = 0;

	ret = go2001_init(gdev);
	if (ret) {
		gdev->initialized = false;
		go2001_err(gdev, "Failed resetting firmware\n");
	}

	mutex_unlock(&gdev->lock);
}

static irqreturn_t go2001_irq(int irq, void *priv)
{
	struct go2001_dev *gdev = priv;
	struct go2001_msg *reply = &gdev->last_reply;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&gdev->irqlock, flags);
	if (unlikely(!gdev->fw_loaded)) {
		gdev->fw_loaded = true;
		complete(&gdev->fw_completion);
		spin_unlock_irqrestore(&gdev->irqlock, flags);
		return IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&gdev->irqlock, flags);

	while (go2001_get_reply(gdev, reply) == 0) {
		ret = go2001_process_reply(gdev, reply);
		wake_up_all(&gdev->reply_wq);
		if (ret)
			return IRQ_HANDLED;
	}

	go2001_send_pending(gdev);
	return IRQ_HANDLED;
}

static const struct v4l2_file_operations go2001_fops = {
	.owner = THIS_MODULE,
	.open = go2001_open,
	.release = go2001_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = go2001_poll,
	.mmap = go2001_mmap,
};


static int go2001_enum_fmt(enum go2001_codec_mode codec_mode,
			enum go2001_fmt_type type, struct v4l2_fmtdesc *f)
{
	struct go2001_fmt *fmt;
	int num_matched = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		fmt = &formats[i];
		if (!(codec_mode & fmt->codec_modes) || (fmt->type != type))
			continue;

		if (num_matched == f->index) {
			strlcpy(f->description, fmt->desc,
					sizeof(f->description));
			f->pixelformat = fmt->pixelformat;
			if (type == FMT_TYPE_CODED)
				f->flags = V4L2_FMT_FLAG_COMPRESSED;
			return 0;
		}
		++num_matched;
	}

	return -EINVAL;
}

static int go2001_enum_fmt_cap(struct file *file, void *fh,
				struct v4l2_fmtdesc *f)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);

	if (ctx->codec_mode == CODEC_MODE_DECODER)
		return go2001_enum_fmt(ctx->codec_mode, FMT_TYPE_RAW, f);
	else
		return go2001_enum_fmt(ctx->codec_mode, FMT_TYPE_CODED, f);
}

static int go2001_enum_fmt_out(struct file *file, void *fh,
				struct v4l2_fmtdesc *f)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);

	if (ctx->codec_mode == CODEC_MODE_DECODER)
		return go2001_enum_fmt(ctx->codec_mode, FMT_TYPE_CODED, f);
	else
		return go2001_enum_fmt(ctx->codec_mode, FMT_TYPE_RAW, f);
}

static int fill_v4l2_format_raw(struct v4l2_format *f, struct go2001_fmt *gfmt,
				struct go2001_frame_info *finfo)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	int i;

	pix_mp->width = finfo->coded_width;
	pix_mp->height = finfo->coded_height;
	pix_mp->pixelformat = gfmt->pixelformat;
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->num_planes = gfmt->num_planes;

	for (i = 0; i < pix_mp->num_planes; ++i) {
		pix_mp->plane_fmt[i].bytesperline = finfo->bytesperline[i];
		pix_mp->plane_fmt[i].sizeimage = finfo->plane_size[i];
	}

	return 0;
}

static void fill_v4l2_format_coded(struct v4l2_format *f,
				struct go2001_fmt *gfmt, size_t buf_size)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;

	pix_mp->width = 0;
	pix_mp->height = 0;
	pix_mp->pixelformat = gfmt->pixelformat;
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->num_planes = 1;
	pix_mp->plane_fmt[0].bytesperline = 0;
	if (buf_size != 0) {
		pix_mp->plane_fmt[0].sizeimage = buf_size;
	} else if (pix_mp->plane_fmt[0].sizeimage == 0) {
		pix_mp->plane_fmt[0].sizeimage =
				GO2001_DEF_BITSTREAM_BUFFER_SIZE;
	}
}

static int go2001_dec_g_fmt_cap(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);

	if (!go2001_has_frame_info(ctx)) {
		go2001_err(ctx->gdev, "Frame info not available yet\n");
		return -EINVAL;
	}

	return fill_v4l2_format_raw(f, ctx->dst_fmt, &ctx->finfo);
}

static int go2001_enc_g_fmt_out(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);

	if (!go2001_has_frame_info(ctx)) {
		go2001_err(ctx->gdev, "Frame info not available yet\n");
		return -EINVAL;
	}

	return fill_v4l2_format_raw(f, ctx->dst_fmt, &ctx->finfo);
}

static int go2001_dec_g_fmt_out(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);

	if (!ctx->src_fmt) {
		go2001_dbg(ctx->gdev, 1, "Format not ready yet\n");
		return -EINVAL;
	}

	fill_v4l2_format_coded(f, ctx->src_fmt, ctx->bitstream_buf_size);
	return 0;
}

static int go2001_enc_g_fmt_cap(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);

	if (!ctx->dst_fmt) {
		go2001_dbg(ctx->gdev, 1, "Format not ready yet\n");
		return -EINVAL;
	}

	fill_v4l2_format_coded(f, ctx->dst_fmt, ctx->bitstream_buf_size);
	return 0;
}

static struct go2001_fmt *__go2001_dec_try_fmt_out(struct file *file, void *fh,
							struct v4l2_format *f)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct go2001_fmt *fmt;

	fmt = go2001_find_fmt(ctx, pix_mp->pixelformat);
	if (!fmt || fmt->type != FMT_TYPE_CODED)
		return NULL;

	fill_v4l2_format_coded(f, fmt, 0);
	return fmt;
}

static int go2001_dec_try_fmt_out(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct go2001_fmt *fmt = __go2001_dec_try_fmt_out(file, fh, f);
	return fmt ? 0 : -EINVAL;
}

static struct go2001_fmt *__go2001_dec_try_fmt_cap(struct file *file, void *fh,
							struct v4l2_format *f)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct go2001_fmt *fmt;

	/*
	 * S_FMT on CAPTURE allows setting destination fourcc only,
	 * resolution is set after parsing headers and comes from the HW.
	 */
	fmt = go2001_find_fmt(ctx, pix_mp->pixelformat);
	if (!fmt || fmt->type != FMT_TYPE_RAW)
		return NULL;

	return fmt;
}

static int go2001_dec_try_fmt_cap(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct go2001_fmt *fmt = __go2001_dec_try_fmt_cap(file, fh, f);
	return fmt ? 0 : -EINVAL;
}

static struct go2001_fmt *__go2001_enc_try_fmt_out(struct file *file, void *fh,
			struct v4l2_format *f, struct go2001_frame_info *finfo)

{
	struct go2001_ctx *ctx = fh_to_ctx(fh);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct go2001_fmt *fmt;

	fmt = go2001_find_fmt(ctx, pix_mp->pixelformat);
	if (!fmt || fmt->type != FMT_TYPE_RAW)
		return NULL;

	go2001_calc_finfo(ctx, fmt, finfo, pix_mp->width, pix_mp->height);
	fill_v4l2_format_raw(f, fmt, finfo);

	return fmt;
}

static int go2001_enc_try_fmt_out(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct go2001_frame_info finfo;
	struct go2001_fmt *fmt = __go2001_enc_try_fmt_out(file, fh, f, &finfo);
	return fmt ? 0 : -EINVAL;
}

static struct go2001_fmt *__go2001_enc_try_fmt_cap(struct file *file, void *fh,
							struct v4l2_format *f)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);
	struct go2001_fmt *fmt;

	fmt = go2001_find_fmt(ctx, f->fmt.pix_mp.pixelformat);
	if (!fmt || fmt->type != FMT_TYPE_CODED)
		return NULL;

	fill_v4l2_format_coded(f, fmt, 0);

	/* TODO: This should allow setting scaling */
	return fmt;
}

static int go2001_enc_try_fmt_cap(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct go2001_fmt *fmt = __go2001_enc_try_fmt_cap(file, fh, f);
	return fmt ? 0 : -EINVAL;
}

static int go2001_dec_s_fmt_out(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);
	struct go2001_fmt *fmt;

	go2001_trace(ctx->gdev);
	if (ctx->state != UNINITIALIZED) {
		go2001_err(ctx->gdev, "Format cannot be set in this state\n");
		return -EBUSY;
	}

	fmt = __go2001_dec_try_fmt_out(file, fh, f);
	if (!fmt)
		return -EINVAL;

	ctx->src_fmt = fmt;
	ctx->bitstream_buf_size = f->fmt.pix_mp.plane_fmt[0].sizeimage;

	go2001_dbg(ctx->gdev, 1, "S_FMT on OUTPUT to %s\n", fmt->desc);
	return 0;
}

static int go2001_dec_s_fmt_cap(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);
	struct go2001_fmt *fmt;

	go2001_trace(ctx->gdev);
	if (ctx->state != UNINITIALIZED) {
		go2001_err(ctx->gdev, "Format cannot be set in this state\n");
		return -EBUSY;
	}

	fmt = __go2001_dec_try_fmt_cap(file, fh, f);
	if (!fmt)
		return -EINVAL;

	ctx->dst_fmt = fmt;
	memset(&ctx->finfo, 0, sizeof(ctx->finfo));

	go2001_dbg(ctx->gdev, 1, "S_FMT on CAPTURE to %s\n", fmt->desc);
	return 0;
}

static int go2001_enc_s_fmt_out(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);
	struct go2001_frame_info finfo;
	struct go2001_fmt *fmt;

	go2001_trace(ctx->gdev);
	if (ctx->state != UNINITIALIZED) {
		go2001_err(ctx->gdev, "Format cannot be set in this state\n");
		return -EBUSY;
	}

	fmt = __go2001_enc_try_fmt_out(file, fh, f, &finfo);
	if (!fmt)
		return -EINVAL;

	/* At 1280x720 and below go2001 can do three reference frames. */
	ctx->enc_params.multi_ref_frame_mode =
		finfo.width * finfo.height <= 1280 * 720;

	ctx->src_fmt = fmt;
	ctx->finfo = finfo;

	go2001_dbg(ctx->gdev, 1, "S_FMT on OUTPUT to %s, planes:%d\n",
			fmt->desc, f->fmt.pix_mp.num_planes);
	return 0;
}

static int go2001_enc_s_fmt_cap(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);
	struct go2001_fmt *fmt;

	go2001_trace(ctx->gdev);
	if (ctx->state != UNINITIALIZED) {
		go2001_err(ctx->gdev, "Format cannot be set in this state\n");
		return -EBUSY;
	}

	fmt = __go2001_enc_try_fmt_cap(file, fh, f);
	if (!fmt)
		return -EINVAL;

	ctx->dst_fmt = fmt;
	ctx->bitstream_buf_size = f->fmt.pix_mp.plane_fmt[0].sizeimage;

	go2001_dbg(ctx->gdev, 1, "S_FMT on CAPTURE to %s\n", fmt->desc);
	return 0;
}

static int go2001_enc_g_crop(struct file *file, void *fh, struct v4l2_crop *c)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);
	struct go2001_frame_info *finfo = &ctx->finfo;

	go2001_trace(ctx->gdev);
	if (!V4L2_TYPE_IS_OUTPUT(c->type)) {
		go2001_err(ctx->gdev,
				"G_CROP on CAPTURE for encoder unsupported\n");
		return -EINVAL;
	}

	if (!go2001_has_frame_info(ctx)) {
		go2001_err(ctx->gdev, "Frame info not available yet\n");
		return -EINVAL;
	}

	c->c.left = 0;
	c->c.top = 0;
	c->c.width = finfo->width;
	c->c.height = finfo->height;
	go2001_dbg(ctx->gdev, 2, "Crop: (%d, %d) -> (%u, %u)\n",
			c->c.left, c->c.top, c->c.width, c->c.height);
	return 0;
}

static int go2001_enc_s_crop(struct file *file, void *fh,
				const struct v4l2_crop *c)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);
	struct go2001_frame_info *finfo = &ctx->finfo;
	int aligned_w, aligned_h;
	struct go2001_fmt *fmt;

	go2001_trace(ctx->gdev);
	if (!V4L2_TYPE_IS_OUTPUT(c->type)) {
		go2001_err(ctx->gdev,
				"Crop can only be set on the OUTPUT queue\n");
		return -EINVAL;
	}

	if (!go2001_has_frame_info(ctx)) {
		go2001_err(ctx->gdev,
				"Crop must be set after setting format\n");
		return -EINVAL;
	}

	fmt = ctx->src_fmt;

	aligned_w = round_up(c->c.width, fmt->h_align);
	aligned_h = round_up(c->c.height, fmt->v_align);

	if (finfo->width != c->c.width || finfo->height != c->c.height) {
		go2001_dbg(ctx->gdev, 1,
				"Adjusted crop from (%d, %d) to (%d, %d)\n",
				c->c.width, c->c.height, aligned_w, aligned_h);
	}

	if (c->c.left != 0 || c->c.top != 0 || aligned_w > finfo->coded_width
			|| aligned_h > finfo->coded_height) {
		go2001_err(ctx->gdev, "Invalid crop (%d, %d) -> (%u, %u) "
					"for coded size %ux%u\n",
				c->c.left, c->c.top, aligned_w, aligned_h,
				finfo->coded_width, finfo->coded_height);
		return -EINVAL;
	}

	finfo->width = aligned_w;
	finfo->height = aligned_h;
	go2001_dbg(ctx->gdev, 2, "Visible size set to %ux%u\n",
			finfo->width, finfo->height);
	return 0;
}

static int go2001_dec_g_crop(struct file *file, void *fh, struct v4l2_crop *c)
{
	struct go2001_ctx *ctx = fh_to_ctx(fh);
	struct go2001_frame_info *finfo = &ctx->finfo;

	go2001_trace(ctx->gdev);
	if (V4L2_TYPE_IS_OUTPUT(c->type)) {
		go2001_err(ctx->gdev,
				"G_CROP on OUTPUT for decoder unsupported\n");
		return -EINVAL;
	}

	if (!go2001_has_frame_info(ctx)) {
		go2001_dbg(ctx->gdev, 1, "Frame info not ready\n");
		return -EINVAL;
	}

	c->c.left = 0;
	c->c.top = 0;
	c->c.width = finfo->width;
	c->c.height = finfo->height;
	go2001_dbg(ctx->gdev, 2, "Crop: (%d, %d) -> (%u, %u)\n",
			c->c.left, c->c.top, c->c.width, c->c.height);
	return 0;
}

static int go2001_enc_s_parm(struct file *file, void *fh,
				struct v4l2_streamparm *p) {
	struct go2001_ctx *ctx = fh_to_ctx(fh);
	struct v4l2_fract *timeperframe;
	int fps;

	go2001_trace(ctx->gdev);
	if (p->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	timeperframe = &p->parm.output.timeperframe;
	if (timeperframe->numerator == 0 || timeperframe->denominator == 0) {
		go2001_err(ctx->gdev, "Invalid values for timeperframe\n");
		return -EINVAL;
	}

	/* For now just normalize to 1/x sec/frame. */
	fps = timeperframe->denominator / timeperframe->numerator;
	fps = max(1, fps);
	ctx->pending_rt_params.enc_params.framerate_num = fps;
	ctx->pending_rt_params.enc_params.framerate_denom = 1;
	set_bit(GO2001_FRAMERATE_CHANGE, ctx->pending_rt_params.changed_mask);
	go2001_dbg(ctx->gdev, 2, "FPS changed to %d\n", fps);

	return 0;
}

static int go2001_enc_g_parm(struct file *file, void *fh,
				struct v4l2_streamparm *p) {
	struct go2001_ctx *ctx = fh_to_ctx(fh);
	struct v4l2_fract *timeperframe;

	if (p->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	memset(&p->parm, 0, sizeof(p->parm));
	p->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	timeperframe = &p->parm.output.timeperframe;
	timeperframe->numerator = ctx->enc_params.framerate_denom;
	timeperframe->denominator = ctx->enc_params.framerate_num;

	return 0;
}

static int go2001_reqbufs_out(struct go2001_ctx *ctx,
				struct v4l2_requestbuffers *rb)
{
	struct vb2_queue *vq = &ctx->src_vq;
	int ret;

	go2001_dbg(ctx->gdev, 3, "count: %d\n", rb->count);

	if (rb->count == 0)
		return vb2_reqbufs(vq, rb);

	if (!ctx->src_fmt || !ctx->dst_fmt) {
		go2001_err(ctx->gdev, "Formats not set\n");
		return -EINVAL;
	}

	if (ctx->codec_mode == CODEC_MODE_ENCODER
			&& !go2001_has_frame_info(ctx)) {
		go2001_err(ctx->gdev, "No frame info available yet\n");
		return -EINVAL;
	}

	if (ctx->state == UNINITIALIZED) {
		ret = go2001_init_codec(ctx);
		if (ret) {
			go2001_err(ctx->gdev, "Failed initializing codec\n");
			return ret;
		}
	}

	return vb2_reqbufs(vq, rb);
}

static int go2001_move_from_resume_queue(struct go2001_ctx *ctx)
{
	struct list_head temp_list;
	struct go2001_buffer *gbuf;
	unsigned long flags;
	int ret = 0;

	/*
	 * The buffer which triggered the resolution change, and any buffers
	 * which might have followed it and were sent by us to the hardware
	 * before we got the resolution change notification, have to be
	 * processed again, in the same order.
	 * Move them from the resume queue to the front of the source
	 * queue, and reinitialize for processing.
	 *
	 * First, make a copy of the resume list, so we can release
	 * the lock to be able to call go2001_prepare_gbuf() on each element.
	 */
	INIT_LIST_HEAD(&temp_list);
	spin_lock_irqsave(&ctx->qlock, flags);
	list_splice_init(&ctx->src_resume_q, &temp_list);
	spin_unlock_irqrestore(&ctx->qlock, flags);
	list_for_each_entry(gbuf, &temp_list, list) {
		go2001_dbg(ctx->gdev, 3, "Requeuing buffer %p\n", gbuf);
		ret = go2001_prepare_gbuf(ctx, gbuf, true);
		if (ret)
			break;
	}

	/*
	 * Finally, splice the lists so that the resume list is added in
	 * front of the source list.
	 */
	spin_lock_irqsave(&ctx->qlock, flags);
	list_splice_init(&temp_list, &ctx->src_buf_q);
	spin_unlock_irqrestore(&ctx->qlock, flags);

	return ret;
}

static int go2001_reqbufs_cap(struct go2001_ctx *ctx,
				struct v4l2_requestbuffers *rb)
{
	struct vb2_queue *vq = &ctx->dst_vq;
	unsigned long flags;
	int ret;

	go2001_dbg(ctx->gdev, 3, "count: %d\n", rb->count);

	if (rb->count == 0)
		return vb2_reqbufs(vq, rb);

	if (!go2001_has_frame_info(ctx)) {
		go2001_err(ctx->gdev, "No frame info available yet\n");
		return -EINVAL;
	}

	if (ctx->state == UNINITIALIZED) {
		if (ctx->codec_mode == CODEC_MODE_DECODER) {
			go2001_err(ctx->gdev, "Invalid state %d\n", ctx->state);
			return -EINVAL;
		} else {
			ret = go2001_init_codec(ctx);
			if (ret) {
				go2001_err(ctx->gdev,
						"Failed initializing codec\n");
				return ret;
			}
		}
	}

	if (ctx->codec_mode == CODEC_MODE_DECODER) {
		ret = go2001_set_dec_raw_fmt(ctx);
		if (ret) {
			go2001_err(ctx->gdev,
					"Failed setting decoder format\n");
			return ret;
		}

		ret = go2001_move_from_resume_queue(ctx);
		if (ret) {
			go2001_err(ctx->gdev,
					"Failed requeuing pending buffers\n");
			return ret;
		}
	}

	ret = vb2_reqbufs(vq, rb);
	if (ret)
		return ret;

	spin_lock_irqsave(&ctx->qlock, flags);
	ctx->state = RUNNING;
	spin_unlock_irqrestore(&ctx->qlock, flags);

	go2001_schedule_frames(ctx);
	return ret;
}

static int go2001_reqbufs(struct file *file, void *fh,
				struct v4l2_requestbuffers *rb)
{
	struct go2001_ctx *ctx = fh_to_ctx(file->private_data);
	int ret;

	if (V4L2_TYPE_IS_OUTPUT(rb->type))
		ret = go2001_reqbufs_out(ctx, rb);
	else
		ret = go2001_reqbufs_cap(ctx, rb);

	if (ret)
		go2001_err(ctx->gdev, "REQBUFS for type %d failed\n", rb->type);
	else
		go2001_dbg(ctx->gdev, 2, "Allocated %d buffers for type %d\n",
				rb->count, rb->type);

	return ret;
}

static int go2001_querybuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct go2001_ctx *ctx = fh_to_ctx(file->private_data);
	int ret;
	int i;

	go2001_trace(ctx->gdev);

	if (V4L2_TYPE_IS_OUTPUT(b->type)) {
		ret = vb2_querybuf(&ctx->src_vq, b);
	} else {
		ret = vb2_querybuf(&ctx->dst_vq, b);
		if (V4L2_TYPE_IS_MULTIPLANAR(b->type)) {
			for (i = 0; i < b->length; ++i) {
				b->m.planes[i].m.mem_offset +=
					DST_QUEUE_OFF_BASE;
			}
		} else {
			b->m.offset += DST_QUEUE_OFF_BASE;
		}
	}

	return ret;
}

static int go2001_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct go2001_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *vq = go2001_get_vq(ctx, b->type);

	if (ctx->state == ERROR) {
		go2001_dbg(ctx->gdev, 1, "Context %p in error state\n", ctx);
		return -EIO;
	}

	return vb2_qbuf(vq, b);
}

static int go2001_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct go2001_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *vq = go2001_get_vq(ctx, b->type);

	return vb2_dqbuf(vq, b, file->f_flags & O_NONBLOCK);
}

static int go2001_expbuf(struct file *file, void *fh,
				struct v4l2_exportbuffer *e)
{
	struct go2001_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *vq = go2001_get_vq(ctx, e->type);

	return vb2_expbuf(vq, e);
}

static int go2001_streamon(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct go2001_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *vq = go2001_get_vq(ctx, type);

	go2001_trace(ctx->gdev);
	return vb2_streamon(vq, type);
}

static int go2001_streamoff(struct file *file, void *fh,
				enum v4l2_buf_type type)
{
	struct go2001_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *vq = go2001_get_vq(ctx, type);

	go2001_trace(ctx->gdev);
	return vb2_streamoff(vq, type);
}

static int go2001_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_RESOLUTION_CHANGE:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	default:
		return -EINVAL;
	}
}

static int go2001_enum_framesizes(struct file *file, void *fh,
					struct v4l2_frmsizeenum *fsize)
{
	struct go2001_ctx *ctx = fh_to_ctx(file->private_data);
	struct v4l2_frmsize_stepwise *s = &fsize->stepwise;
	struct go2001_fmt *fmt;

	if (fsize->index != 0)
		return -EINVAL;

	fmt = go2001_find_fmt(ctx, fsize->pixel_format);
	if (!fmt) {
		go2001_dbg(ctx->gdev, 1, "Unsupported pixelformat %d\n",
				fsize->pixel_format);
		return -EINVAL;
	}

	if (fmt->type != FMT_TYPE_CODED) {
		go2001_dbg(ctx->gdev, 1, "Only supported for coded formats\n");
		return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	s->min_width = GO2001_VPX_MACROBLOCK_SIZE;
	s->max_width = 1920;
	s->step_width = 2;
	s->min_height = GO2001_VPX_MACROBLOCK_SIZE;
	s->max_height = 1088;
	s->step_height = 2;

	return 0;
}

static const struct v4l2_ioctl_ops go2001_ioctl_dec_ops = {
	.vidioc_querycap = go2001_querycap,

	.vidioc_enum_fmt_vid_cap_mplane = go2001_enum_fmt_cap,
	.vidioc_enum_fmt_vid_out_mplane = go2001_enum_fmt_out,

	.vidioc_g_fmt_vid_cap_mplane = go2001_dec_g_fmt_cap,
	.vidioc_g_fmt_vid_out_mplane = go2001_dec_g_fmt_out,

	.vidioc_s_fmt_vid_cap_mplane = go2001_dec_s_fmt_cap,
	.vidioc_s_fmt_vid_out_mplane = go2001_dec_s_fmt_out,

	.vidioc_try_fmt_vid_cap_mplane = go2001_dec_try_fmt_cap,
	.vidioc_try_fmt_vid_out_mplane = go2001_dec_try_fmt_out,

	.vidioc_g_crop = go2001_dec_g_crop,

	.vidioc_reqbufs = go2001_reqbufs,
	.vidioc_querybuf = go2001_querybuf,
	.vidioc_qbuf = go2001_qbuf,
	.vidioc_dqbuf = go2001_dqbuf,
	.vidioc_streamon = go2001_streamon,
	.vidioc_streamoff = go2001_streamoff,
	.vidioc_expbuf = go2001_expbuf,

	.vidioc_enum_framesizes = go2001_enum_framesizes,

	.vidioc_subscribe_event = go2001_subscribe_event,
};

static const struct v4l2_ioctl_ops go2001_ioctl_enc_ops = {
	.vidioc_querycap = go2001_querycap,

	.vidioc_enum_fmt_vid_cap_mplane = go2001_enum_fmt_cap,
	.vidioc_enum_fmt_vid_out_mplane = go2001_enum_fmt_out,

	.vidioc_g_fmt_vid_cap_mplane = go2001_enc_g_fmt_cap,
	.vidioc_g_fmt_vid_out_mplane = go2001_enc_g_fmt_out,

	.vidioc_s_fmt_vid_cap_mplane = go2001_enc_s_fmt_cap,
	.vidioc_s_fmt_vid_out_mplane = go2001_enc_s_fmt_out,

	.vidioc_try_fmt_vid_cap_mplane = go2001_enc_try_fmt_cap,
	.vidioc_try_fmt_vid_out_mplane = go2001_enc_try_fmt_out,

	.vidioc_g_crop = go2001_enc_g_crop,
	.vidioc_s_crop = go2001_enc_s_crop,

	.vidioc_s_parm = go2001_enc_s_parm,
	.vidioc_g_parm = go2001_enc_g_parm,

	.vidioc_reqbufs = go2001_reqbufs,
	.vidioc_querybuf = go2001_querybuf,
	.vidioc_qbuf = go2001_qbuf,
	.vidioc_dqbuf = go2001_dqbuf,
	.vidioc_streamon = go2001_streamon,
	.vidioc_streamoff = go2001_streamoff,
	.vidioc_expbuf = go2001_expbuf,

	.vidioc_enum_framesizes = go2001_enum_framesizes,
};


static int go2001_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret;
	struct go2001_dev *gdev;
	struct video_device *vdev;

	dev_info(&pdev->dev, "Probing GO2001\n");

	ret = pci_enable_device(pdev);
	if (ret) {
		go2001_err(gdev, "Failed enabling device\n");
		return ret;
	}

	ret = pci_request_regions(pdev, DRIVER_NAME);
	if (ret) {
		go2001_err(gdev, "Failed requesting regions\n");
		goto disable_device;
	}

	pci_set_master(pdev);
	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
	if (!ret)
		ret = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
	if (ret) {
		go2001_err(gdev, "No suitable DMA available\n");
		goto release_regions;
	}

	gdev = devm_kzalloc(&pdev->dev, sizeof(*gdev), GFP_KERNEL);
	if (!gdev) {
		ret = -ENOMEM;
		goto release_regions;
	}

	gdev->pdev = pdev;
	mutex_init(&gdev->lock);

	spin_lock_init(&gdev->irqlock);
	INIT_LIST_HEAD(&gdev->inst_list);
	INIT_LIST_HEAD(&gdev->new_inst_list);
	INIT_LIST_HEAD(&gdev->ctx_list);
	go2001_init_hw_inst(&gdev->ctrl_inst, 0);
	list_add_tail(&gdev->ctrl_inst.inst_entry, &gdev->inst_list);
	gdev->curr_hw_inst = &gdev->ctrl_inst;
	init_waitqueue_head(&gdev->reply_wq);
	init_completion(&gdev->fw_completion);
	INIT_DELAYED_WORK(&gdev->watchdog_work, go2001_watchdog);

	gdev->msg_cache = kmem_cache_create("msg_cache",
		sizeof(struct go2001_msg), 0, SLAB_HWCACHE_ALIGN, NULL);
	if (!gdev->msg_cache) {
		go2001_err(gdev, "Failed creating msg cache\n");
		goto release_regions;
	}

	gdev->alloc_ctx = vb2_dma_sg_init_ctx(&pdev->dev);
	if (IS_ERR(gdev->alloc_ctx))
		goto release_cache;

	ret = go2001_map_iomem(gdev);
	if (ret) {
		go2001_err(gdev, "Failed mapping IO memory\n");
		goto free_alloc_ctx;
	}

	ret = pci_enable_msi(pdev);
	if (ret) {
		go2001_err(gdev, "Failed enabling MSI\n");
		goto unmap;
	}

	ret = request_irq(pdev->irq, go2001_irq, 0, DRIVER_NAME, gdev);
	if (ret) {
		go2001_err(gdev, "Failed requesting IRQ\n");
		goto disable_msi;
	}

	ret = v4l2_device_register(&pdev->dev, &gdev->v4l2_dev);
	if (ret) {
		go2001_err(gdev, "Failed registering V4L2 device\n");
		goto free_irq;
	}

	vdev = &gdev->dec_vdev;
	strlcpy(vdev->name, VDEV_NAME_DEC, sizeof(vdev->name));
	vdev->v4l2_dev = &gdev->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->fops = &go2001_fops;
	vdev->ioctl_ops = &go2001_ioctl_dec_ops;
	vdev->lock = &gdev->lock;
	vdev->release = video_device_release_empty;
	video_set_drvdata(vdev, gdev);
	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto unregister_v4l2_device;

	vdev = &gdev->enc_vdev;
	strlcpy(vdev->name, VDEV_NAME_ENC, sizeof(vdev->name));
	vdev->v4l2_dev = &gdev->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->fops = &go2001_fops;
	vdev->ioctl_ops = &go2001_ioctl_enc_ops;
	vdev->lock = &gdev->lock;
	vdev->release = video_device_release_empty;
	video_set_drvdata(vdev, gdev);
	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto unregister_video_device_dec;

	dev_info(&pdev->dev, "GO2001 successfully initialized.\n");
	return 0;

unregister_video_device_dec:
	video_unregister_device(&gdev->dec_vdev);
unregister_v4l2_device:
	v4l2_device_unregister(&gdev->v4l2_dev);
free_irq:
	free_irq(pdev->irq, gdev);
disable_msi:
	pci_disable_msi(pdev);
unmap:
	go2001_unmap_iomem(gdev);
free_alloc_ctx:
	vb2_dma_sg_cleanup_ctx(gdev->alloc_ctx);
release_cache:
	kmem_cache_destroy(gdev->msg_cache);
release_regions:
	pci_release_regions(pdev);
disable_device:
	pci_disable_device(pdev);
	return ret;
}

static void go2001_remove(struct pci_dev *pdev)
{
	struct v4l2_device *v4l2_dev = pci_get_drvdata(pdev);
	struct go2001_dev *gdev = container_of(v4l2_dev, struct go2001_dev,
						v4l2_dev);

	dev_info(&pdev->dev, "Removing GO2001\n");

	video_unregister_device(&gdev->enc_vdev);
	video_unregister_device(&gdev->dec_vdev);
	v4l2_device_unregister(&gdev->v4l2_dev);
	free_irq(pdev->irq, gdev);
	pci_disable_msi(pdev);
	go2001_unmap_iomem(gdev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

#ifdef CONFIG_PM_SLEEP
static int go2001_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct v4l2_device *v4l2_dev = pci_get_drvdata(pdev);
	struct go2001_dev *gdev = container_of(v4l2_dev, struct go2001_dev,
						v4l2_dev);
	unsigned long flags1, flags2;
	struct go2001_ctx *ctx;
	int ret;

	go2001_trace(gdev);

	spin_lock_irqsave(&gdev->irqlock, flags1);
	list_for_each_entry(ctx, &gdev->ctx_list, ctx_entry) {
		spin_lock_irqsave(&ctx->qlock, flags2);
		ctx->state = ERROR;
		spin_unlock_irqrestore(&ctx->qlock, flags2);
	}
	spin_unlock_irqrestore(&gdev->irqlock, flags1);

	ret = wait_event_timeout(gdev->reply_wq, gdev->msgs_in_flight == 0,
				msecs_to_jiffies(GO2001_WATCHDOG_TIMEOUT_MS));
	if (ret == 0)
		go2001_err(gdev, "Timed out waiting for HW to become idle\n");

	mutex_lock(&gdev->lock);
	go2001_cancel_all_contexts(gdev);
	mutex_unlock(&gdev->lock);

	return 0;
}

static int go2001_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct v4l2_device *v4l2_dev = pci_get_drvdata(pdev);
	struct go2001_dev *gdev = container_of(v4l2_dev, struct go2001_dev,
						v4l2_dev);
	int ret;

	go2001_trace(gdev);

	mutex_lock(&gdev->lock);
	ret = go2001_init(gdev);
	if (ret) {
		gdev->initialized = false;
		go2001_err(gdev, "Failed resetting firmware\n");
	}
	mutex_unlock(&gdev->lock);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(go2001_pm_ops, go2001_suspend, go2001_resume);

static const struct pci_device_id go2001_pci_tbl[] = {
	{ PCI_DEVICE(0x1ae0, 0x001a) },
	{},
};

static struct pci_driver go2001_driver = {
	.name = KBUILD_MODNAME,
	.probe = go2001_probe,
	.remove = go2001_remove,
	.id_table = go2001_pci_tbl,
	.driver.pm = &go2001_pm_ops,
};

module_pci_driver(go2001_driver);

MODULE_DESCRIPTION("GO2001 PCI-E codec driver");
MODULE_AUTHOR("Pawel Osciak <posciak@chromium.org>");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(pci, go2001_pci_tbl);

