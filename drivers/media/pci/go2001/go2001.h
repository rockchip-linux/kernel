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
#ifndef _MEDIA_PCI_GO2001_GO2001_H_
#define _MEDIA_PCI_GO2001_GO2001_H_

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/sizes.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>

#include "go2001_proto.h"

struct go2001_msg {
	struct list_head list_entry;
	struct go2001_msg_payload payload;
};

static inline void *msg_to_hdr(struct go2001_msg *msg)
{
	return &msg->payload.hdr;
}
static inline void *msg_to_param(struct go2001_msg *msg)
{
	return msg->payload.param;
}

struct go2001_msg_ring {
	struct go2001_msg_ring_desc desc;
	void __iomem *desc_iomem;
	void __iomem *data_iomem;
	spinlock_t lock;
};

struct go2001_hw_inst {
	struct list_head inst_entry;

	u32 session_id;
	u32 sequence_id;
	u32 last_reply_seq_id;

	struct list_head pending_list;
};

struct go2001_dev {
	struct pci_dev *pdev;
	struct v4l2_device v4l2_dev;
	struct video_device dec_vdev;
	struct video_device enc_vdev;

	void *alloc_ctx;
	struct mutex lock;

	void __iomem *iomem;
	size_t iomem_size;

	struct go2001_msg_ring tx_ring;
	struct go2001_msg_ring rx_ring;
	size_t max_param_size;

	spinlock_t irqlock;
	struct list_head inst_list;
	struct list_head new_inst_list;
	struct go2001_hw_inst *curr_hw_inst;

	struct list_head ctx_list;

	u32 last_reply_inst_id;
	u32 last_reply_seq_id;
	struct go2001_msg last_reply;

	/* Last seq_id for init instance message. */
	u32 last_init_inst_seq_id;
	wait_queue_head_t reply_wq;

	struct go2001_hw_inst ctrl_inst;
	struct completion fw_completion;
	/* Protected by irqlock */
	bool fw_loaded;
	/* Protected by lock */
	bool initialized;
};

enum go2001_codec_mode {
	CODEC_MODE_DECODER = (1 << 0),
	CODEC_MODE_ENCODER = (1 << 1),
};

enum go2001_fmt_type {
	FMT_TYPE_RAW,
	FMT_TYPE_CODED,
};

struct go2001_fmt {
	enum go2001_fmt_type type;
	const char *desc;
	u32 pixelformat;
	unsigned int num_planes;
	u32 hw_format;
	unsigned long codec_modes;
	unsigned int h_align;
	unsigned int v_align;
	unsigned char plane_bpl[VIDEO_MAX_PLANES];
	unsigned char plane_bpp[VIDEO_MAX_PLANES];
};

struct go2001_ctrl {
	u32 id;
	enum v4l2_ctrl_type type;
	const char *name;
	s32 min;
	s32 max;
	u32 step;
	s32 def;
};

enum go2001_codec_state {
	UNINITIALIZED,
	INITIALIZED,
	NEED_HEADER_INFO,
	RUNNING,
	PAUSED,
	RES_CHANGE,
	ERROR,
};

struct go2001_frame_info {
	unsigned int width;
	unsigned int height;
	unsigned int coded_width;
	unsigned int coded_height;
	unsigned int bytesperline[VIDEO_MAX_PLANES];
	size_t plane_size[VIDEO_MAX_PLANES];
};

#define GO2001_REPLY_TIMEOUT_MS	2000

struct go2001_dma_desc {
	/* DMA address of the map list */
	dma_addr_t dma_addr;
	/* Number of entries on the map list */
	int num_entries;
	/* CPU address of the map list */
	struct go2001_mmap_list_entry *mmap_list;
	/* Size in bytes of the list */
	size_t list_size;
	/*
	 * DMA address of the first entry on the list.
	 * Used to identify the buffer when passing it to the HW.
	 */
	u64 map_addr;
};

struct go2001_enc_params {
	u32 bitrate;
	u32 framerate_num;
	u32 framerate_denom;
	bool rc_enable;
	bool request_keyframe;
};

enum go2001_enc_param_change_bit {
	GO2001_BITRATE_CHANGE,
	GO2001_FRAMERATE_CHANGE,
	GO2001_KEYFRAME_REQUESTED,
	GO2001_PARAM_CHANGE_MAX,
};

struct go2001_runtime_enc_params {
	struct go2001_enc_params enc_params;
	unsigned long changed_mask[BITS_TO_LONGS(GO2001_PARAM_CHANGE_MAX)];
};

struct go2001_buffer {
	struct vb2_buffer vb;
	struct list_head list;
	struct go2001_dma_desc dma_desc[VIDEO_MAX_PLANES];
	bool mapped;

	struct go2001_runtime_enc_params rt_enc_params;
	size_t partition_off[VP8_MAX_NUM_PARTITIONS];
	size_t partition_size[VP8_MAX_NUM_PARTITIONS];
};

struct go2001_job {
	struct go2001_msg msg;
	struct go2001_buffer *src_buf;
	struct go2001_buffer *dst_buf;
};

#define GO2001_MSG_POOL_SIZE	VIDEO_MAX_FRAME
struct go2001_ctx {
	struct list_head ctx_entry;

	struct mutex lock;
	struct v4l2_fh v4l2_fh;
	struct v4l2_ctrl_handler ctrl_handler;
	struct go2001_dev *gdev;
	enum go2001_codec_mode codec_mode;

	struct go2001_fmt *src_fmt;
	struct go2001_fmt *dst_fmt;
	struct go2001_frame_info finfo;
	size_t bitstream_buf_size;

	enum go2001_codec_state state;
	struct vb2_queue src_vq;
	struct vb2_queue dst_vq;

	spinlock_t qlock;
	struct list_head src_buf_q;
	struct list_head dst_buf_q;

	struct go2001_hw_inst hw_inst;

	/* Currently running job, if any. */
	struct go2001_job job;

	struct go2001_enc_params enc_params;
	/* Will be applied to the next source buffer queued. */
	struct go2001_runtime_enc_params pending_rt_params;

	/* One for each buffer and an additional ctrl msg. */

	bool need_resume;
	bool format_set;
};

static inline struct go2001_buffer *vb_to_go2001_buf(struct vb2_buffer *vb)
{
	return container_of(vb, struct go2001_buffer, vb);
}

static inline struct go2001_ctx *hw_inst_to_ctx(struct go2001_hw_inst *hw_inst)
{
	return container_of(hw_inst, struct go2001_ctx, hw_inst);
}

static inline bool go2001_has_frame_info(struct go2001_ctx *ctx)
{
	return ctx->dst_fmt && ctx->finfo.width != 0;
}

static inline struct go2001_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct go2001_ctx, v4l2_fh);
}
static inline struct go2001_ctx *ctrl_to_ctx(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct go2001_ctx, ctrl_handler);
}

extern unsigned go2001_debug_level;

#define go2001_err(gdev, fmt, args...) \
	dev_err(&gdev->pdev->dev, "%s:%d " fmt, __func__, __LINE__, ##args)

#define go2001_info(gdev, fmt, args...) \
	dev_info(&gdev->pdev->dev, "%s:%d " fmt, __func__, __LINE__, ##args)

#define go2001_dbg(gdev, level, fmt, args...)				\
	do {								\
		if (go2001_debug_level >= level)			\
			dev_info(&gdev->pdev->dev, "%s:%d " fmt,	\
				 __func__, __LINE__, ##args);		\
	} while (0)

#define go2001_trace(gdev) \
	go2001_dbg((gdev), 5, "%s\n", __func__)

#define GO2001_MIN_NUM_BITSTREAM_BUFFERS	4
#define GO2001_MIN_NUM_FRAME_BUFFERS		8
#define GO2001_DEF_BITSTREAM_BUFFER_SIZE	SZ_1M
#define GO2001_DEF_BITRATE			1000000
#define GO2001_MAX_FPS				30
#define GO2001_DEF_RC_ENABLE			1

#endif /* _MEDIA_PCI_GO2001_GO2001_H_ */

