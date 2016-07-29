/**
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 * author: ZhiChao Yu zhichao.yu@rock-chips.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "dsp_dbg.h"
#include "dsp_work.h"

#define dsp_work_params_type(p) (((u32 *)p)[0])

int dsp_work_set_type(struct dsp_work *work, enum dsp_work_type type)
{
	work->type = type;
	return 0;
}

int dsp_work_set_params(struct dsp_work *work, void *params)
{
	memcpy(&work->params, params, sizeof(work->params));
	return 0;
}

int dsp_work_copy_from_user(struct dma_pool *dma_pool, struct dsp_work *work,
			    void *user)
{
	int ret = 0;
	struct dsp_user_work user_work;
	struct dsp_render_params *render = &work->params.render;

	dsp_debug_enter();

	ret = copy_from_user(&user_work, (void __user *)user,
			     sizeof(user_work));
	if (ret) {
		dsp_err("copy from user work failed\n");
		goto out;
	}

	if (user_work.magic != DSP_RENDER_WORK_MAGIC) {
		dsp_err("work parameter err\n");
		ret = -EINVAL;
		goto out;
	}

	work->id = user_work.hdl;
	work->type = DSP_RENDER_WORK;
	render->type = user_work.render.type;
	render->size = user_work.render.size;

	render->packet_virt = (u32)dma_pool_alloc(dma_pool, GFP_KERNEL,
						  &render->packet_phys);
	if (!render->packet_virt) {
		dsp_err("cannot alloc dma buffer for render packet\n");
		return -ENOMEM;
	}
	ret = copy_from_user((void *)render->packet_virt,
			     (void __user *)user_work.render.packet_virt,
			     render->size);
	if (ret) {
		dsp_err("copy from user packet failed\n");
		goto out;
	}
out:
	dsp_debug_leave();
	return ret;
}

int dsp_work_copy_to_user(struct dsp_work *work, void *user)
{
	int ret = 0;
	struct dsp_user_work user_work;

	dsp_debug_enter();

	user_work.hdl = work->id;
	user_work.result = work->result;

	ret = copy_to_user((void __user *)user, &user_work, sizeof(user_work));
	if (ret) {
		dsp_err("copy to user failed\n");
		goto out;
	}
out:
	dsp_debug_leave();
	return ret;
}

int dsp_work_create(struct dma_pool *dma_pool, void *session,
		    struct dsp_work **work_out)
{
	int ret = 0;
	dma_addr_t dma_addr;
	struct dsp_work *work;

	dsp_debug_enter();
	work = dma_pool_alloc(dma_pool, GFP_KERNEL, &dma_addr);
	if (!work) {
		dsp_err("cannot alloc mem for dsp work\n");
		ret = -ENOMEM;
		goto out;
	}
	memset(work, 0, sizeof(*work));

	work->dma_addr = dma_addr;
	work->status = DSP_WORK_PENDING;
	work->session = (u32)session;

	(*work_out) = work;
out:
	if (ret)
		(*work_out) = NULL;
	dsp_debug_leave();
	return ret;
}

int dsp_work_destroy(struct dma_pool *dma_pool, struct dsp_work *work)
{
	if (work->type == DSP_RENDER_WORK) {
		struct dsp_render_params *render = &work->params.render;

		dma_pool_free(dma_pool, (void *)render->packet_virt,
			      render->packet_phys);
	}

	dma_pool_free(dma_pool, work, work->dma_addr);
	return 0;
}
