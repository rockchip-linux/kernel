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
#ifndef _MEDIA_PCI_GO2001_GO2001_HW_H_
#define _MEDIA_PCI_GO2001_GO2001_HW_H_

#include "go2001.h"

int go2001_map_iomem(struct go2001_dev *gdev);
void go2001_unmap_iomem(struct go2001_dev *gdev);

int go2001_init(struct go2001_dev *gdev);

int go2001_init_codec(struct go2001_ctx *ctx);
void go2001_release_codec(struct go2001_ctx *ctx);

int go2001_set_dec_raw_fmt(struct go2001_ctx *ctx);

int go2001_map_buffer(struct go2001_ctx *ctx, struct go2001_buffer *buf);
void go2001_unmap_buffer(struct go2001_ctx *ctx, struct go2001_buffer *buf);
int go2001_unmap_buffers(struct go2001_ctx *ctx, bool unmap_src,
				bool unmap_dst);

int go2001_prepare_gbuf(struct go2001_ctx *ctx, struct go2001_buffer *gbuf,
				bool is_src);
void go2001_finish_gbuf(struct go2001_ctx *ctx, struct go2001_buffer *gbuf);
int go2001_schedule_frames(struct go2001_ctx *ctx);
void go2001_wait_for_ctx_done(struct go2001_ctx *ctx);
void go2001_send_pending(struct go2001_dev *gdev);
void go2001_cancel_hw_inst_locked(struct go2001_dev *gdev,
					struct go2001_hw_inst *hw_inst);
void go2001_cancel_all_hw_inst_locked(struct go2001_dev *gdev);

void go2001_init_hw_inst(struct go2001_hw_inst *inst, u32 inst_id);

int go2001_get_reply(struct go2001_dev *gdev, struct go2001_msg *msg);

#endif /* _MEDIA_PCI_GO2001_GO2001_HW_H_ */
