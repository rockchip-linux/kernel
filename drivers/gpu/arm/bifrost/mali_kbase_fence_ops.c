// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2020-2022 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include <linux/atomic.h>
#include <linux/list.h>
#include <mali_kbase_fence.h>
#include <mali_kbase.h>

static const char *
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
kbase_fence_get_driver_name(struct fence *fence)
#else
kbase_fence_get_driver_name(struct dma_fence *fence)
#endif
{
	return kbase_drv_name;
}

static const char *
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
kbase_fence_get_timeline_name(struct fence *fence)
#else
kbase_fence_get_timeline_name(struct dma_fence *fence)
#endif
{
#if MALI_USE_CSF
	struct kbase_kcpu_dma_fence *kcpu_fence = (struct kbase_kcpu_dma_fence *)fence;

	return kcpu_fence->metadata->timeline_name;
#else
	return kbase_timeline_name;
#endif /* MALI_USE_CSF */
}

static bool
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
kbase_fence_enable_signaling(struct fence *fence)
#else
kbase_fence_enable_signaling(struct dma_fence *fence)
#endif
{
	return true;
}

static void
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
kbase_fence_fence_value_str(struct fence *fence, char *str, int size)
#else
kbase_fence_fence_value_str(struct dma_fence *fence, char *str, int size)
#endif
{
#if (KERNEL_VERSION(5, 1, 0) > LINUX_VERSION_CODE)
	const char *format = "%u";
#else
	const char *format = "%llu";
#endif
	if (unlikely(!scnprintf(str, size, format, fence->seqno)))
		pr_err("Fail to encode fence seqno to string");
}

#if MALI_USE_CSF
static void
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
kbase_fence_release(struct fence *fence)
#else
kbase_fence_release(struct dma_fence *fence)
#endif
{
	struct kbase_kcpu_dma_fence *kcpu_fence = (struct kbase_kcpu_dma_fence *)fence;

	kbase_kcpu_dma_fence_meta_put(kcpu_fence->metadata);
	kfree(kcpu_fence);
}
#endif

#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
extern const struct fence_ops kbase_fence_ops; /* silence checker warning */
const struct fence_ops kbase_fence_ops = { .wait = fence_default_wait,
#else
extern const struct dma_fence_ops kbase_fence_ops; /* silence checker warning */
const struct dma_fence_ops kbase_fence_ops = { .wait = dma_fence_default_wait,
#endif
					   .get_driver_name = kbase_fence_get_driver_name,
					   .get_timeline_name = kbase_fence_get_timeline_name,
					   .enable_signaling = kbase_fence_enable_signaling,
#if MALI_USE_CSF
					   .fence_value_str = kbase_fence_fence_value_str,
					   .release = kbase_fence_release
#else
					    .fence_value_str = kbase_fence_fence_value_str
#endif
};
KBASE_EXPORT_TEST_API(kbase_fence_ops);
