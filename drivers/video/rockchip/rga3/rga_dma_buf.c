// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 *
 * Author: Huang Lee <Putin.li@rock-chips.com>
 */

#define pr_fmt(fmt) "rga_dma_buf: " fmt

#include "rga_dma_buf.h"
#include "rga.h"
#include "rga_common.h"
#include "rga_job.h"
#include "rga_debugger.h"

static int rga_dma_info_to_prot(enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_BIDIRECTIONAL:
		return IOMMU_READ | IOMMU_WRITE;
	case DMA_TO_DEVICE:
		return IOMMU_READ;
	case DMA_FROM_DEVICE:
		return IOMMU_WRITE;
	default:
		return 0;
	}
}

int rga_buf_size_cal(unsigned long yrgb_addr, unsigned long uv_addr,
		      unsigned long v_addr, int format, uint32_t w,
		      uint32_t h, unsigned long *StartAddr, unsigned long *size)
{
	uint32_t size_yrgb = 0;
	uint32_t size_uv = 0;
	uint32_t size_v = 0;
	uint32_t stride = 0;
	unsigned long start, end;
	uint32_t pageCount;

	switch (format) {
	case RGA_FORMAT_RGBA_8888:
	case RGA_FORMAT_RGBX_8888:
	case RGA_FORMAT_BGRA_8888:
	case RGA_FORMAT_BGRX_8888:
	case RGA_FORMAT_ARGB_8888:
	case RGA_FORMAT_XRGB_8888:
	case RGA_FORMAT_ABGR_8888:
	case RGA_FORMAT_XBGR_8888:
		stride = (w * 4 + 3) & (~3);
		size_yrgb = stride * h;
		start = yrgb_addr >> PAGE_SHIFT;
		end = yrgb_addr + size_yrgb;
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pageCount = end - start;
		break;
	case RGA_FORMAT_RGB_888:
	case RGA_FORMAT_BGR_888:
		stride = (w * 3 + 3) & (~3);
		size_yrgb = stride * h;
		start = yrgb_addr >> PAGE_SHIFT;
		end = yrgb_addr + size_yrgb;
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pageCount = end - start;
		break;
	case RGA_FORMAT_RGB_565:
	case RGA_FORMAT_RGBA_5551:
	case RGA_FORMAT_RGBA_4444:
	case RGA_FORMAT_BGR_565:
	case RGA_FORMAT_BGRA_5551:
	case RGA_FORMAT_BGRA_4444:
	case RGA_FORMAT_ARGB_5551:
	case RGA_FORMAT_ARGB_4444:
	case RGA_FORMAT_ABGR_5551:
	case RGA_FORMAT_ABGR_4444:
		stride = (w * 2 + 3) & (~3);
		size_yrgb = stride * h;
		start = yrgb_addr >> PAGE_SHIFT;
		end = yrgb_addr + size_yrgb;
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pageCount = end - start;
		break;

		/* YUV FORMAT */
	case RGA_FORMAT_YCbCr_422_SP:
	case RGA_FORMAT_YCrCb_422_SP:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = stride * h;
		start = min(yrgb_addr, uv_addr);
		start >>= PAGE_SHIFT;
		end = max((yrgb_addr + size_yrgb), (uv_addr + size_uv));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pageCount = end - start;
		break;
	case RGA_FORMAT_YCbCr_422_P:
	case RGA_FORMAT_YCrCb_422_P:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = ((stride >> 1) * h);
		size_v = ((stride >> 1) * h);
		start = min3(yrgb_addr, uv_addr, v_addr);
		start = start >> PAGE_SHIFT;
		end =
			max3((yrgb_addr + size_yrgb), (uv_addr + size_uv),
			(v_addr + size_v));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pageCount = end - start;
		break;
	case RGA_FORMAT_YCbCr_420_SP:
	case RGA_FORMAT_YCrCb_420_SP:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = (stride * (h >> 1));
		start = min(yrgb_addr, uv_addr);
		start >>= PAGE_SHIFT;
		end = max((yrgb_addr + size_yrgb), (uv_addr + size_uv));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pageCount = end - start;
		break;
	case RGA_FORMAT_YCbCr_420_P:
	case RGA_FORMAT_YCrCb_420_P:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = ((stride >> 1) * (h >> 1));
		size_v = ((stride >> 1) * (h >> 1));
		start = min3(yrgb_addr, uv_addr, v_addr);
		start >>= PAGE_SHIFT;
		end =
			max3((yrgb_addr + size_yrgb), (uv_addr + size_uv),
			(v_addr + size_v));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pageCount = end - start;
		break;
	case RGA_FORMAT_YCbCr_400:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		start = yrgb_addr >> PAGE_SHIFT;
		end = yrgb_addr + size_yrgb;
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pageCount = end - start;
		break;
	case RGA_FORMAT_Y4:
		stride = ((w + 3) & (~3)) >> 1;
		size_yrgb = stride * h;
		start = yrgb_addr >> PAGE_SHIFT;
		end = yrgb_addr + size_yrgb;
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pageCount = end - start;
		break;
	case RGA_FORMAT_YVYU_422:
	case RGA_FORMAT_VYUY_422:
	case RGA_FORMAT_YUYV_422:
	case RGA_FORMAT_UYVY_422:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = stride * h;
		start = min(yrgb_addr, uv_addr);
		start >>= PAGE_SHIFT;
		end = max((yrgb_addr + size_yrgb), (uv_addr + size_uv));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pageCount = end - start;
		break;
	case RGA_FORMAT_YVYU_420:
	case RGA_FORMAT_VYUY_420:
	case RGA_FORMAT_YUYV_420:
	case RGA_FORMAT_UYVY_420:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = (stride * (h >> 1));
		start = min(yrgb_addr, uv_addr);
		start >>= PAGE_SHIFT;
		end = max((yrgb_addr + size_yrgb), (uv_addr + size_uv));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pageCount = end - start;
		break;
	case RGA_FORMAT_YCbCr_420_SP_10B:
	case RGA_FORMAT_YCrCb_420_SP_10B:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = (stride * (h >> 1));
		start = min(yrgb_addr, uv_addr);
		start >>= PAGE_SHIFT;
		end = max((yrgb_addr + size_yrgb), (uv_addr + size_uv));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pageCount = end - start;
		break;
	default:
		pageCount = 0;
		start = 0;
		break;
	}

	*StartAddr = start;

	if (size != NULL)
		*size = size_yrgb + size_uv + size_v;

	return pageCount;
}

static dma_addr_t rga_iommu_dma_alloc_iova(struct iommu_domain *domain,
					    size_t size, u64 dma_limit,
					    struct device *dev)
{
	struct rga_iommu_dma_cookie *cookie = (void *)domain->iova_cookie;
	struct iova_domain *iovad = &cookie->iovad;
	unsigned long shift, iova_len, iova = 0;

	shift = iova_shift(iovad);
	iova_len = size >> shift;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
	/*
	 * Freeing non-power-of-two-sized allocations back into the IOVA caches
	 * will come back to bite us badly, so we have to waste a bit of space
	 * rounding up anything cacheable to make sure that can't happen. The
	 * order of the unadjusted size will still match upon freeing.
	 */
	if (iova_len < (1 << (IOVA_RANGE_CACHE_MAX_SIZE - 1)))
		iova_len = roundup_pow_of_two(iova_len);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	dma_limit = min_not_zero(dma_limit, dev->bus_dma_limit);
#else
	if (dev->bus_dma_mask)
		dma_limit &= dev->bus_dma_mask;
#endif

	if (domain->geometry.force_aperture)
		dma_limit = min(dma_limit, (u64)domain->geometry.aperture_end);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 19, 111) && \
     LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	iova = alloc_iova_fast(iovad, iova_len,
			       min_t(dma_addr_t, dma_limit >> shift, iovad->end_pfn),
			       true);
#else
	iova = alloc_iova_fast(iovad, iova_len, dma_limit >> shift, true);
#endif

	return (dma_addr_t)iova << shift;
}

static void rga_iommu_dma_free_iova(struct iommu_domain *domain,
				    dma_addr_t iova, size_t size)
{
	struct rga_iommu_dma_cookie *cookie = (void *)domain->iova_cookie;
	struct iova_domain *iovad = &cookie->iovad;

	free_iova_fast(iovad, iova_pfn(iovad, iova), size >> iova_shift(iovad));
}

static inline struct iommu_domain *rga_iommu_get_dma_domain(struct device *dev)
{
	return iommu_get_domain_for_dev(dev);
}

void rga_iommu_unmap(struct rga_dma_buffer *buffer)
{
	if (buffer == NULL)
		return;
	if (buffer->iova == 0)
		return;

	iommu_unmap(buffer->domain, buffer->iova, buffer->size);
	rga_iommu_dma_free_iova(buffer->domain, buffer->iova, buffer->size);
}

int rga_iommu_map_sgt(struct sg_table *sgt, size_t size,
		      struct rga_dma_buffer *buffer,
		      struct device *rga_dev)
{
	struct iommu_domain *domain = NULL;
	struct rga_iommu_dma_cookie *cookie;
	struct iova_domain *iovad;
	dma_addr_t iova;
	size_t map_size;
	unsigned long align_size;

	if (sgt == NULL) {
		pr_err("can not map iommu, because sgt is null!\n");
		return -EINVAL;
	}

	domain = rga_iommu_get_dma_domain(rga_dev);
	cookie = (void *)domain->iova_cookie;
	iovad = &cookie->iovad;
	align_size = iova_align(iovad, size);

	if (DEBUGGER_EN(MSG))
		pr_info("iova_align size = %ld", align_size);

	iova = rga_iommu_dma_alloc_iova(domain, align_size, rga_dev->coherent_dma_mask, rga_dev);
	if (!iova) {
		pr_err("rga_iommu_dma_alloc_iova failed");
		return -ENOMEM;
	}

	map_size = iommu_map_sg(domain, iova, sgt->sgl, sgt->orig_nents,
				rga_dma_info_to_prot(DMA_BIDIRECTIONAL));
	if (map_size < align_size) {
		pr_err("iommu can not map sgt to iova");
		rga_iommu_dma_free_iova(domain, iova, align_size);
		return -EINVAL;
	}

	buffer->domain = domain;
	buffer->iova = iova;
	buffer->size = align_size;

	return 0;
}

int rga_iommu_map(phys_addr_t paddr, size_t size,
		  struct rga_dma_buffer *buffer,
		  struct device *rga_dev)
{
	int ret;
	struct iommu_domain *domain = NULL;
	struct rga_iommu_dma_cookie *cookie;
	struct iova_domain *iovad;
	dma_addr_t iova;
	unsigned long align_size;

	if (paddr == 0) {
		pr_err("can not map iommu, because phys_addr is 0!\n");
		return -EINVAL;
	}

	domain = rga_iommu_get_dma_domain(rga_dev);
	cookie = (void *)domain->iova_cookie;
	iovad = &cookie->iovad;
	align_size = iova_align(iovad, size);

	if (DEBUGGER_EN(MSG))
		pr_info("iova_align size = %ld", align_size);

	iova = rga_iommu_dma_alloc_iova(domain, align_size, rga_dev->coherent_dma_mask, rga_dev);
	if (!iova) {
		pr_err("rga_iommu_dma_alloc_iova failed");
		return -ENOMEM;
	}

	ret = iommu_map(domain, iova, paddr, align_size,
			rga_dma_info_to_prot(DMA_BIDIRECTIONAL));
	if (ret) {
		pr_err("iommu can not map phys_addr to iova");
		rga_iommu_dma_free_iova(domain, iova, align_size);
		return ret;
	}

	buffer->domain = domain;
	buffer->iova = iova;
	buffer->size = align_size;

	return 0;
}

int rga_virtual_memory_check(void *vaddr, u32 w, u32 h, u32 format, int fd)
{
	int bits = 32;
	int temp_data = 0;
	void *one_line = NULL;

	bits = rga_get_format_bits(format);
	if (bits < 0)
		return -1;

	one_line = kzalloc(w * 4, GFP_KERNEL);
	if (!one_line) {
		pr_err("kzalloc fail %s[%d]\n", __func__, __LINE__);
		return 0;
	}

	temp_data = w * (h - 1) * bits >> 3;
	if (fd > 0) {
		pr_info("vaddr is%p, bits is %d, fd check\n", vaddr, bits);
		memcpy(one_line, (char *)vaddr + temp_data, w * bits >> 3);
		pr_info("fd check ok\n");
	} else {
		pr_info("vir addr memory check.\n");
		memcpy((void *)((char *)vaddr + temp_data), one_line,
			 w * bits >> 3);
		pr_info("vir addr check ok.\n");
	}

	kfree(one_line);
	return 0;
}

int rga_dma_memory_check(struct rga_dma_buffer *rga_dma_buffer, struct rga_img_info_t *img)
{
	int ret = 0;
	void *vaddr;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
	struct iosys_map map;
#endif
	struct dma_buf *dma_buf;

	dma_buf = rga_dma_buffer->dma_buf;

	if (!IS_ERR_OR_NULL(dma_buf)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
		ret = dma_buf_vmap(dma_buf, &map);
		vaddr = ret ? NULL : map.vaddr;
#else
		vaddr = dma_buf_vmap(dma_buf);
#endif
		if (vaddr) {
			ret = rga_virtual_memory_check(vaddr, img->vir_w,
				img->vir_h, img->format, img->yrgb_addr);
		} else {
			pr_err("can't vmap the dma buffer!\n");
			return -EINVAL;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
		dma_buf_vunmap(dma_buf, &map);
#else
		dma_buf_vunmap(dma_buf, vaddr);
#endif
	}

	return ret;
}

int rga_dma_map_buf(struct dma_buf *dma_buf, struct rga_dma_buffer *rga_dma_buffer,
		    enum dma_data_direction dir, struct device *rga_dev)
{
	struct dma_buf_attachment *attach = NULL;
	struct sg_table *sgt = NULL;
	struct scatterlist *sg = NULL;
	int i, ret = 0;

	if (dma_buf != NULL) {
		get_dma_buf(dma_buf);
	} else {
		pr_err("dma_buf is invalid[%p]\n", dma_buf);
		return -EINVAL;
	}

	attach = dma_buf_attach(dma_buf, rga_dev);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		pr_err("Failed to attach dma_buf, ret[%d]\n", ret);
		goto err_get_attach;
	}

	sgt = dma_buf_map_attachment(attach, dir);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		pr_err("Failed to map attachment, ret[%d]\n", ret);
		goto err_get_sgt;
	}

	rga_dma_buffer->dma_buf = dma_buf;
	rga_dma_buffer->attach = attach;
	rga_dma_buffer->sgt = sgt;
	rga_dma_buffer->iova = sg_dma_address(sgt->sgl);
	rga_dma_buffer->dir = dir;
	rga_dma_buffer->size = 0;
	for_each_sgtable_sg(sgt, sg, i)
		rga_dma_buffer->size += sg_dma_len(sg);

	return ret;

err_get_sgt:
	if (attach)
		dma_buf_detach(dma_buf, attach);
err_get_attach:
	if (dma_buf)
		dma_buf_put(dma_buf);

	return ret;
}

int rga_dma_map_fd(int fd, struct rga_dma_buffer *rga_dma_buffer,
		   enum dma_data_direction dir, struct device *rga_dev)
{
	struct dma_buf *dma_buf = NULL;
	struct dma_buf_attachment *attach = NULL;
	struct sg_table *sgt = NULL;
	struct scatterlist *sg = NULL;
	int i, ret = 0;

	dma_buf = dma_buf_get(fd);
	if (IS_ERR(dma_buf)) {
		ret = PTR_ERR(dma_buf);
		pr_err("Fail to get dma_buf from fd[%d], ret[%d]\n", fd, ret);
		return ret;
	}

	attach = dma_buf_attach(dma_buf, rga_dev);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		pr_err("Failed to attach dma_buf, ret[%d]\n", ret);
		goto err_get_attach;
	}

	sgt = dma_buf_map_attachment(attach, dir);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		pr_err("Failed to map attachment, ret[%d]\n", ret);
		goto err_get_sgt;
	}

	rga_dma_buffer->dma_buf = dma_buf;
	rga_dma_buffer->attach = attach;
	rga_dma_buffer->sgt = sgt;
	rga_dma_buffer->iova = sg_dma_address(sgt->sgl);
	rga_dma_buffer->dir = dir;
	rga_dma_buffer->size = 0;
	for_each_sgtable_sg(sgt, sg, i)
		rga_dma_buffer->size += sg_dma_len(sg);

	return ret;

err_get_sgt:
	if (attach)
		dma_buf_detach(dma_buf, attach);
err_get_attach:
	if (dma_buf)
		dma_buf_put(dma_buf);

	return ret;
}

void rga_dma_unmap_buf(struct rga_dma_buffer *rga_dma_buffer)
{
	if (rga_dma_buffer->attach && rga_dma_buffer->sgt)
		dma_buf_unmap_attachment(rga_dma_buffer->attach,
					 rga_dma_buffer->sgt,
					 rga_dma_buffer->dir);

	if (rga_dma_buffer->attach) {
		dma_buf_detach(rga_dma_buffer->dma_buf, rga_dma_buffer->attach);
		dma_buf_put(rga_dma_buffer->dma_buf);
	}
}

void rga_dma_sync_flush_range(void *pstart, void *pend, struct rga_scheduler_t *scheduler)
{
	dma_sync_single_for_device(scheduler->dev, virt_to_phys(pstart),
				   pend - pstart, DMA_TO_DEVICE);
}
