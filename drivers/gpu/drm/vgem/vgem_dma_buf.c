/*
 * Copyright © 2012 Intel Corporation
 * Copyright © 2014 The Chromium OS Authors
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Ben Widawsky <ben@bwidawsk.net>
 *
 */

#include <linux/dma-buf.h>
#include "vgem_drv.h"

#define VGEM_FD_PERMS 0600

static struct sg_table *vgem_gem_map_dma_buf(struct dma_buf_attachment *attach,
					     enum dma_data_direction dir)
{
	struct drm_vgem_gem_object *obj = attach->dmabuf->priv;
	struct sg_table *sg;
	int ret;

	ret = vgem_gem_get_pages(obj);
	if (ret)
		return ERR_PTR(ret);

	/* VGEM assumes cache coherent access. Normally we might have to flush
	 * caches here */

	BUG_ON(obj->pages == NULL);

	sg = drm_prime_pages_to_sg(obj->pages, obj->base.size / PAGE_SIZE);
	if (!sg) {
		vgem_gem_put_pages(obj);
		return NULL;
	}

	return sg;
}

static void vgem_gem_unmap_dma_buf(struct dma_buf_attachment *attach,
			    struct sg_table *sg,
			    enum dma_data_direction data_direction)
{
	sg_free_table(sg);
	kfree(sg);
}

static void *vgem_kmap_atomic_dma_buf(struct dma_buf *dma_buf,
				      unsigned long page_num)
{
	return NULL;
}

static void *vgem_kmap_dma_buf(struct dma_buf *dma_buf,
			       unsigned long page_num)
{
	return NULL;
}

static int vgem_mmap_dma_buf(struct dma_buf *dma_buf,
			     struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct dma_buf_ops vgem_dmabuf_ops = {
	.map_dma_buf	= vgem_gem_map_dma_buf,
	.unmap_dma_buf	= vgem_gem_unmap_dma_buf,
	.release	= drm_gem_dmabuf_release,
	.kmap_atomic	= vgem_kmap_atomic_dma_buf,
	.kmap		= vgem_kmap_dma_buf,
	.mmap		= vgem_mmap_dma_buf,
};

struct dma_buf *vgem_gem_prime_export(struct drm_device *dev,
				      struct drm_gem_object *obj,
				      int flags)
{
	return dma_buf_export(obj, &vgem_dmabuf_ops, obj->size, flags, NULL);
}

struct drm_gem_object *vgem_gem_prime_import(struct drm_device *dev,
					     struct dma_buf *dma_buf)
{
	struct drm_vgem_gem_object *obj = NULL;
	struct dma_buf_attachment *attach = NULL;
	struct sg_table *sg = NULL;
	int num_pages;
	int ret;

	attach = dma_buf_attach(dma_buf, dev->dev);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		goto fail;
	}

	get_dma_buf(dma_buf);

	sg = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sg)) {
		ret = PTR_ERR(sg);
		goto fail_detach;
	}

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (obj == NULL) {
		ret = -ENOMEM;
		goto fail_unmap;
	}

	obj->base.import_attach = attach;
	obj->sg = sg;

	/* As a result of this mmap will not work -yet- */
	ret = drm_gem_object_init(dev, &obj->base, dma_buf->size);
	if (ret) {
		ret = -ENOMEM;
		goto fail_free;
	}

	num_pages = obj->base.size / PAGE_SIZE;

	obj->pages = drm_malloc_ab(num_pages, sizeof(struct page *));
	if (!obj->pages) {
		ret = -ENOMEM;
		goto fail_gem_release;
	}

	ret = drm_prime_sg_to_page_addr_arrays(sg, obj->pages, NULL, num_pages);
	if (ret) {
		ret = -ENOMEM;
		goto fail_free_pages;
	}

	return &obj->base;

fail_free_pages:
	drm_free_large(obj->pages);
fail_gem_release:
	drm_gem_object_release(&obj->base);
fail_free:
	kfree(obj);
fail_unmap:
	dma_buf_unmap_attachment(attach, sg, DMA_BIDIRECTIONAL);
fail_detach:
	dma_buf_detach(dma_buf, attach);
	dma_buf_put(dma_buf);
fail:
	return ERR_PTR(ret);
}
