// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 * Author: Felix Zeng <felix.zeng@rock-chips.com>
 */

#include <linux/dma-map-ops.h>
#include <linux/iova.h>
#include <linux/sizes.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#include "rknpu_iommu.h"

#define RKNPU_SWITCH_DOMAIN_WAIT_TIME_MS 6000

dma_addr_t rknpu_iommu_dma_alloc_iova(struct iommu_domain *domain, size_t size,
				      u64 dma_limit, struct device *dev,
				      bool size_aligned)
{
	struct rknpu_iommu_dma_cookie *cookie = (void *)domain->iova_cookie;
	struct iova_domain *iovad = &cookie->iovad;
	unsigned long shift, iova_len, iova = 0;
	unsigned long limit_pfn;
	struct iova *new_iova = NULL;
	bool alloc_fast = size_aligned;

	shift = iova_shift(iovad);
	iova_len = size >> shift;

#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
	/*
	 * Freeing non-power-of-two-sized allocations back into the IOVA caches
	 * will come back to bite us badly, so we have to waste a bit of space
	 * rounding up anything cacheable to make sure that can't happen. The
	 * order of the unadjusted size will still match upon freeing.
	 */
	if (iova_len < (1 << (IOVA_RANGE_CACHE_MAX_SIZE - 1)))
		iova_len = roundup_pow_of_two(iova_len);
#endif

#if (KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE)
	dma_limit = min_not_zero(dma_limit, dev->bus_dma_limit);
#else
	if (dev->bus_dma_mask)
		dma_limit &= dev->bus_dma_mask;
#endif

	if (domain->geometry.force_aperture)
		dma_limit =
			min_t(u64, dma_limit, domain->geometry.aperture_end);

#if (KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE)
	limit_pfn = dma_limit >> shift;
#else
	limit_pfn = min_t(dma_addr_t, dma_limit >> shift, iovad->end_pfn);
#endif

	if (alloc_fast) {
		iova = alloc_iova_fast(iovad, iova_len, limit_pfn, true);
	} else {
		new_iova = alloc_iova(iovad, iova_len, limit_pfn, size_aligned);
		if (!new_iova)
			return 0;
		iova = new_iova->pfn_lo;
	}

	return (dma_addr_t)iova << shift;
}

void rknpu_iommu_dma_free_iova(struct rknpu_iommu_dma_cookie *cookie,
			       dma_addr_t iova, size_t size, bool size_aligned)
{
	struct iova_domain *iovad = &cookie->iovad;
	bool alloc_fast = size_aligned;

	if (alloc_fast)
		free_iova_fast(iovad, iova_pfn(iovad, iova),
			       size >> iova_shift(iovad));
	else
		free_iova(iovad, iova_pfn(iovad, iova));
}

struct iommu_domain *rknpu_iommu_live_domain(struct device *dev)
{
	struct rknpu_device *rknpu_dev = dev_get_drvdata(dev);
	int id;

	if (!rknpu_dev)
		return NULL;
	id = rknpu_dev->iommu_domain_id;
	if (id < 0 || id >= RKNPU_MAX_IOMMU_DOMAIN_NUM)
		return NULL;
	return rknpu_dev->iommu_domains[id];
}

static int rknpu_dma_info_to_prot(enum dma_data_direction dir, bool coherent)
{
	int prot = coherent ? IOMMU_CACHE : 0;

	switch (dir) {
	case DMA_BIDIRECTIONAL:
		return prot | IOMMU_READ | IOMMU_WRITE;
	case DMA_TO_DEVICE:
		return prot | IOMMU_READ;
	case DMA_FROM_DEVICE:
		return prot | IOMMU_WRITE;
	default:
		return 0;
	}
}

/*
 * Prepare a successfully-mapped scatterlist to give back to the caller.
 *
 * At this point the segments are already laid out by iommu_dma_map_sg() to
 * avoid individually crossing any boundaries, so we merely need to check a
 * segment's start address to avoid concatenating across one.
 */
static int __finalise_sg(struct device *dev, struct scatterlist *sg, int nents,
			 dma_addr_t dma_addr)
{
	struct scatterlist *s, *cur = sg;
	unsigned long seg_mask = dma_get_seg_boundary(dev);
	unsigned int cur_len = 0, max_len = dma_get_max_seg_size(dev);
	int i, count = 0;

	for_each_sg(sg, s, nents, i) {
		/* Restore this segment's original unaligned fields first */
#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
		dma_addr_t s_dma_addr = sg_dma_address(s);
#endif
		unsigned int s_iova_off = sg_dma_address(s);
		unsigned int s_length = sg_dma_len(s);
		unsigned int s_iova_len = s->length;

		sg_dma_address(s) = DMA_MAPPING_ERROR;
		sg_dma_len(s) = 0;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
		if (sg_is_dma_bus_address(s)) {
			if (i > 0)
				cur = sg_next(cur);

			sg_dma_unmark_bus_address(s);
			sg_dma_address(cur) = s_dma_addr;
			sg_dma_len(cur) = s_length;
			sg_dma_mark_bus_address(cur);
			count++;
			cur_len = 0;
			continue;
		}
#endif

		s->offset += s_iova_off;
		s->length = s_length;

		/*
		 * Now fill in the real DMA data. If...
		 * - there is a valid output segment to append to
		 * - and this segment starts on an IOVA page boundary
		 * - but doesn't fall at a segment boundary
		 * - and wouldn't make the resulting output segment too long
		 */
		if (cur_len && !s_iova_off && (dma_addr & seg_mask) &&
		    (max_len - cur_len >= s_length)) {
			/* ...then concatenate it with the previous one */
			cur_len += s_length;
		} else {
			/* Otherwise start the next output segment */
			if (i > 0)
				cur = sg_next(cur);
			cur_len = s_length;
			count++;

			sg_dma_address(cur) = dma_addr + s_iova_off;
		}

		sg_dma_len(cur) = cur_len;
		dma_addr += s_iova_len;

		if (s_length + s_iova_off < s_iova_len)
			cur_len = 0;
	}
	return count;
}

/*
 * If mapping failed, then just restore the original list,
 * but making sure the DMA fields are invalidated.
 */
static void __invalidate_sg(struct scatterlist *sg, int nents)
{
	struct scatterlist *s;
	int i;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	for_each_sg(sg, s, nents, i) {
		if (sg_is_dma_bus_address(s)) {
			sg_dma_unmark_bus_address(s);
		} else {
			if (sg_dma_address(s) != DMA_MAPPING_ERROR)
				s->offset += sg_dma_address(s);
			if (sg_dma_len(s))
				s->length = sg_dma_len(s);
		}
		sg_dma_address(s) = DMA_MAPPING_ERROR;
		sg_dma_len(s) = 0;
	}
#else
	for_each_sg(sg, s, nents, i) {
		if (sg_dma_address(s) != DMA_MAPPING_ERROR)
			s->offset += sg_dma_address(s);
		if (sg_dma_len(s))
			s->length = sg_dma_len(s);
		sg_dma_address(s) = DMA_MAPPING_ERROR;
		sg_dma_len(s) = 0;
	}
#endif
}

int rknpu_iommu_dma_map_sg(struct device *dev, struct scatterlist *sg,
			   int nents, enum dma_data_direction dir,
			   bool iova_aligned)
{
	struct iommu_domain *domain = rknpu_iommu_live_domain(dev);
	struct rknpu_iommu_dma_cookie *cookie = (void *)domain->iova_cookie;
	struct iova_domain *iovad = &cookie->iovad;
	struct scatterlist *s = NULL, *prev = NULL;
	int prot = rknpu_dma_info_to_prot(dir, dev_is_dma_coherent(dev));
	dma_addr_t iova;
	unsigned long iova_len = 0;
	unsigned long mask = dma_get_seg_boundary(dev);
	ssize_t ret = -EINVAL;
	int i = 0;

	if (iova_aligned)
		return dma_map_sg(dev, sg, nents, dir);

	/*
	 * Work out how much IOVA space we need, and align the segments to
	 * IOVA granules for the IOMMU driver to handle. With some clever
	 * trickery we can modify the list in-place, but reversibly, by
	 * stashing the unaligned parts in the as-yet-unused DMA fields.
	 */
	for_each_sg(sg, s, nents, i) {
		size_t s_iova_off = iova_offset(iovad, s->offset);
		size_t s_length = s->length;
		size_t pad_len = (mask - iova_len + 1) & mask;

		sg_dma_address(s) = s_iova_off;
		sg_dma_len(s) = s_length;
		s->offset -= s_iova_off;
		s_length = iova_align(iovad, s_length + s_iova_off);
		s->length = s_length;

		/*
		 * Due to the alignment of our single IOVA allocation, we can
		 * depend on these assumptions about the segment boundary mask:
		 * - If mask size >= IOVA size, then the IOVA range cannot
		 *   possibly fall across a boundary, so we don't care.
		 * - If mask size < IOVA size, then the IOVA range must start
		 *   exactly on a boundary, therefore we can lay things out
		 *   based purely on segment lengths without needing to know
		 *   the actual addresses beforehand.
		 * - The mask must be a power of 2, so pad_len == 0 if
		 *   iova_len == 0, thus we cannot dereference prev the first
		 *   time through here (i.e. before it has a meaningful value).
		 */
		if (pad_len && pad_len < s_length - 1) {
			prev->length += pad_len;
			iova_len += pad_len;
		}

		iova_len += s_length;
		prev = s;
	}

	if (!iova_len) {
		ret = __finalise_sg(dev, sg, nents, 0);
		goto out;
	}

	iova = rknpu_iommu_dma_alloc_iova(domain, iova_len, dma_get_mask(dev),
					  dev, iova_aligned);
	if (!iova) {
		ret = -ENOMEM;
		LOG_ERROR("failed to allocate IOVA: %zd\n", ret);
		goto out_restore_sg;
	}

	ret = iommu_map_sg(domain, iova, sg, nents, prot);
	if (ret < 0 || ret < iova_len) {
		LOG_ERROR("failed to map SG: %zd\n", ret);
		goto out_free_iova;
	}

	return __finalise_sg(dev, sg, nents, iova);

out_free_iova:
	rknpu_iommu_dma_free_iova(cookie, iova, iova_len, iova_aligned);
out_restore_sg:
	__invalidate_sg(sg, nents);
out:

	if (ret < 0)
		ret = 0;

	return ret;
}

void rknpu_iommu_dma_unmap_sg(struct device *dev, struct scatterlist *sg,
			      int nents, enum dma_data_direction dir,
			      bool iova_aligned)
{
	struct iommu_domain *domain = rknpu_iommu_live_domain(dev);
	struct rknpu_iommu_dma_cookie *cookie = (void *)domain->iova_cookie;
	struct iova_domain *iovad = &cookie->iovad;
	size_t iova_off = 0;
	dma_addr_t end = 0, start = 0;
	struct scatterlist *tmp = NULL;
	dma_addr_t dma_addr = 0;
	size_t size = 0;
	int i = 0;

	if (iova_aligned)
		return dma_unmap_sg(dev, sg, nents, dir);

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	/*
	 * The scatterlist segments are mapped into a single
	 * contiguous IOVA allocation, the start and end points
	 * just have to be determined.
	 */
	for_each_sg(sg, tmp, nents, i) {
		if (sg_is_dma_bus_address(tmp)) {
			sg_dma_unmark_bus_address(tmp);
			continue;
		}

		if (sg_dma_len(tmp) == 0)
			break;

		start = sg_dma_address(tmp);
		break;
	}

	nents -= i;
	for_each_sg(tmp, tmp, nents, i) {
		if (sg_is_dma_bus_address(tmp)) {
			sg_dma_unmark_bus_address(tmp);
			continue;
		}

		if (sg_dma_len(tmp) == 0)
			break;

		end = sg_dma_address(tmp) + sg_dma_len(tmp);
	}
#else
	start = sg_dma_address(sg);
	for_each_sg(sg_next(sg), tmp, nents - 1, i) {
		if (sg_dma_len(tmp) == 0)
			break;
		sg = tmp;
	}
	end = sg_dma_address(sg) + sg_dma_len(sg);
#endif

	dma_addr = start;
	size = end - start;
	iova_off = iova_offset(iovad, start);

	if (end) {
		dma_addr -= iova_off;
		size = iova_align(iovad, size + iova_off);
		iommu_unmap(domain, dma_addr, size);
		rknpu_iommu_dma_free_iova(cookie, dma_addr, size, iova_aligned);
	}
}

#if defined(CONFIG_IOMMU_API) && defined(CONFIG_NO_GKI) && !defined(CONFIG_ARM)

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
struct iommu_group {
	struct kobject kobj;
	struct kobject *devices_kobj;
	struct list_head devices;
#ifdef __ANDROID_COMMON_KERNEL__
	struct xarray pasid_array;
#endif
	struct mutex mutex;
	void *iommu_data;
	void (*iommu_data_release)(void *iommu_data);
	char *name;
	int id;
	struct iommu_domain *default_domain;
	struct iommu_domain *blocking_domain;
	struct iommu_domain *domain;
	struct list_head entry;
	unsigned int owner_cnt;
	void *owner;
};
#else
struct iommu_group {
	struct kobject kobj;
	struct kobject *devices_kobj;
	struct list_head devices;
	struct mutex mutex;
	struct blocking_notifier_head notifier;
	void *iommu_data;
	void (*iommu_data_release)(void *iommu_data);
	char *name;
	int id;
	struct iommu_domain *default_domain;
	struct iommu_domain *domain;
	struct list_head entry;
};
#endif

int rknpu_iommu_init_domain(struct rknpu_device *rknpu_dev)
{
	// init domain 0
	if (!rknpu_dev->iommu_domains[0]) {
		rknpu_dev->iommu_domain_id = 0;
		rknpu_dev->iommu_domains[rknpu_dev->iommu_domain_id] =
			iommu_get_domain_for_dev(rknpu_dev->dev);
		rknpu_dev->iommu_domain_num = 1;
	}
	return 0;
}

int rknpu_iommu_switch_domain(struct rknpu_device *rknpu_dev, int domain_id)
{
	struct iommu_domain *src_domain = NULL;
	struct iommu_domain *dst_domain = NULL;
	struct bus_type *bus = NULL;
	int src_domain_id = 0;
	int ret = -EINVAL;

	if (!rknpu_dev->iommu_en)
		return -EINVAL;

	if (domain_id < 0 || domain_id > (RKNPU_MAX_IOMMU_DOMAIN_NUM - 1)) {
		LOG_DEV_ERROR(
			rknpu_dev->dev,
			"invalid iommu domain id: %d, reuse domain id: %d\n",
			domain_id, rknpu_dev->iommu_domain_id);
		return -EINVAL;
	}

	bus = rknpu_dev->dev->bus;
	if (!bus)
		return -EFAULT;

	src_domain_id = rknpu_dev->iommu_domain_id;
	if (domain_id == src_domain_id) {
		return 0;
	}

	/*
	 * The driver's own record is the source of truth for which domain is live.
	 * iommu_get_domain_for_dev() no longer tracks these switches, so comparing
	 * against it could only ever produce a spurious mismatch.
	 */
	src_domain = rknpu_dev->iommu_domains[src_domain_id];

	dst_domain = rknpu_dev->iommu_domains[domain_id];
	if (dst_domain != NULL) {
		ret = rk_iommu_switch_domain(rknpu_dev->dev, dst_domain);
		if (ret) {
			LOG_DEV_ERROR(
				rknpu_dev->dev,
				"failed to switch to iommu domain, id: %d, ret: %d\n",
				domain_id, ret);
			/*
			 * Unlike the detach/attach pair this replaces, a failed
			 * switch leaves the previous page table live rather than
			 * leaving the master attached to nothing, so restoring is
			 * belt and braces.
			 */
			if (rk_iommu_switch_domain(rknpu_dev->dev, src_domain))
				LOG_DEV_ERROR(
					rknpu_dev->dev,
					"failed to restore src iommu domain, id: %d\n",
					src_domain_id);
			return ret;
		}
		rknpu_dev->iommu_domain_id = domain_id;
	} else {
		uint64_t dma_limit = 1ULL << 32;

		dst_domain = iommu_domain_alloc(bus);
		if (!dst_domain) {
			LOG_DEV_ERROR(rknpu_dev->dev,
				      "failed to allocate iommu domain\n");
			return -EIO;
		}
		iommu_get_dma_cookie(dst_domain);

		/*
		 * Initialise this domain's IOVA allocator directly.
		 *
		 * The previous route got one by pretending the domain was a DMA-API
		 * domain -- setting __IOMMU_DOMAIN_DMA_API on the type and calling
		 * iommu_setup_dma_ops(), whose only useful effect here was
		 * iommu_dma_init_domain() initialising the cookie's iovad. That
		 * depends on iommu_get_domain_for_dev() reporting this domain, i.e.
		 * on the default-domain overwrite removed below. Do the two init
		 * calls directly instead: same iovad, and the domain stays a plain
		 * unmanaged domain that iommu_map_sg() and iommu_unmap() serve.
		 *
		 * Granule and start_pfn mirror iommu_dma_init_domain(): 4 KiB pages,
		 * base_pfn 1 so IOVA 0 is never handed out.
		 */
		{
			struct rknpu_iommu_dma_cookie *ck =
				(struct rknpu_iommu_dma_cookie *)dst_domain->iova_cookie;

			if (!ck) {
				LOG_DEV_ERROR(rknpu_dev->dev,
					      "no iova cookie for domain %d\n",
					      domain_id);
				iommu_domain_free(dst_domain);
				return -ENOMEM;
			}
			init_iova_domain(&ck->iovad, SZ_4K, 1);
			if (iova_domain_init_rcaches(&ck->iovad)) {
				LOG_DEV_ERROR(rknpu_dev->dev,
					      "failed to init iova rcaches, domain %d\n",
					      domain_id);
				iommu_domain_free(dst_domain);
				return -ENOMEM;
			}
		}

		ret = rk_iommu_switch_domain(rknpu_dev->dev, dst_domain);
		if (ret) {
			LOG_DEV_ERROR(
				rknpu_dev->dev,
				"failed to switch to iommu domain, id: %d, ret: %d\n",
				domain_id, ret);
			iommu_domain_free(dst_domain);
			return ret;
		}
		(void)dma_limit;

		rknpu_dev->iommu_domain_id = domain_id;
		rknpu_dev->iommu_domains[domain_id] = dst_domain;
		rknpu_dev->iommu_domain_num++;
	}

	/*
	 * The default-domain overwrite that used to live here is gone.
	 *
	 * dma-iommu resolves every DMA-API mapping through
	 * iommu_get_dma_domain() == dev->iommu_group->default_domain, so the only
	 * way to make a DMA-API mapping land in domain N was to tell the core that
	 * N was the group's default -- reaching the field through a private copy of
	 * struct iommu_group. That leaves the core's idea of the default pointing at
	 * a domain this driver may later free, and any core path that reattaches
	 * "the default" then uses it.
	 *
	 * Nothing here needs it any more: the NPU's buffers are mapped explicitly
	 * with iommu_map_sg()/iommu_unmap() against the domain the driver names.
	 */

	LOG_INFO("switch iommu domain from %d to %d\n", src_domain_id,
		 domain_id);

	return ret;
}

int rknpu_iommu_domain_get_and_switch(struct rknpu_device *rknpu_dev,
				      int domain_id)
{
	unsigned long timeout_jiffies =
		msecs_to_jiffies(RKNPU_SWITCH_DOMAIN_WAIT_TIME_MS);
	unsigned long start = jiffies;
	int ret = -EINVAL;

	while (true) {
		mutex_lock(&rknpu_dev->domain_lock);

		if (domain_id == rknpu_dev->iommu_domain_id) {
			atomic_inc(&rknpu_dev->iommu_domain_refcount);
			mutex_unlock(&rknpu_dev->domain_lock);
			break;
		}

		if (atomic_read(&rknpu_dev->iommu_domain_refcount) == 0) {
			ret = rknpu_iommu_switch_domain(rknpu_dev, domain_id);
			if (ret) {
				LOG_DEV_ERROR(
					rknpu_dev->dev,
					"failed to switch iommu domain, id: %d, ret: %d\n",
					domain_id, ret);
				mutex_unlock(&rknpu_dev->domain_lock);
				return ret;
			}
			atomic_inc(&rknpu_dev->iommu_domain_refcount);
			mutex_unlock(&rknpu_dev->domain_lock);
			break;
		}

		mutex_unlock(&rknpu_dev->domain_lock);

		usleep_range(10, 100);
		if (time_after(jiffies, start + timeout_jiffies)) {
			LOG_DEV_ERROR(
				rknpu_dev->dev,
				"switch iommu domain time out, failed to switch iommu domain, id: %d\n",
				domain_id);
			return -EINVAL;
		}
	}

	return 0;
}

int rknpu_iommu_domain_put(struct rknpu_device *rknpu_dev)
{
	/*
	 * Never let the reference count go negative.
	 *
	 * rknpu_iommu_domain_get_and_switch() proceeds only when this reads exactly
	 * zero, so a bare atomic_dec() turns a single unbalanced put into a permanent
	 * wedge: the count never reads zero again, every domain switch burns its full
	 * RKNPU_SWITCH_DOMAIN_WAIT_TIME_MS and fails, and because
	 * rknpu_gem_object_create() switches domains, every subsequent allocation then
	 * fails with -EINVAL until the machine is rebooted.
	 *
	 * Clamping turns an over-put into a bounded anomaly instead of a dead device.
	 * It treats the symptom, not the cause, so warn once per occurrence to keep any
	 * remaining imbalance visible.
	 */
	if (atomic_dec_return(&rknpu_dev->iommu_domain_refcount) < 0) {
		atomic_set(&rknpu_dev->iommu_domain_refcount, 0);
		LOG_DEV_ERROR(rknpu_dev->dev,
			      "iommu domain refcount underflow, clamped to 0\n");
	}

	return 0;
}

void rknpu_iommu_free_domains(struct rknpu_device *rknpu_dev)
{
	int i = 0;

	if (rknpu_iommu_domain_get_and_switch(rknpu_dev, 0)) {
		LOG_DEV_ERROR(rknpu_dev->dev, "%s error\n", __func__);
		return;
	}

	for (i = 1; i < RKNPU_MAX_IOMMU_DOMAIN_NUM; i++) {
		struct iommu_domain *domain = rknpu_dev->iommu_domains[i];

		if (domain == NULL)
			continue;

		/*
		 * No iommu_detach_device(): the core never attached these domains.
		 * The switch to domain 0 above already moved the hardware off this
		 * page table. Release the IOVA allocator initialised in the switch.
		 */
		if (domain->iova_cookie) {
			struct rknpu_iommu_dma_cookie *ck =
				(struct rknpu_iommu_dma_cookie *)domain->iova_cookie;

			put_iova_domain(&ck->iovad);
		}
		iommu_domain_free(domain);

		rknpu_dev->iommu_domains[i] = NULL;
	}

	rknpu_iommu_domain_put(rknpu_dev);
}

#else

int rknpu_iommu_init_domain(struct rknpu_device *rknpu_dev)
{
	return 0;
}

int rknpu_iommu_switch_domain(struct rknpu_device *rknpu_dev, int domain_id)
{
	return 0;
}

int rknpu_iommu_domain_get_and_switch(struct rknpu_device *rknpu_dev,
				      int domain_id)
{
	return 0;
}

int rknpu_iommu_domain_put(struct rknpu_device *rknpu_dev)
{
	return 0;
}

void rknpu_iommu_free_domains(struct rknpu_device *rknpu_dev)
{
}

#endif
