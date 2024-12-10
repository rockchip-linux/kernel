// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2012-2022 ARM Limited. All rights reserved.
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

/*
 * Code for supporting explicit Linux fences (CONFIG_SYNC_FILE)
 */

#include <linux/sched.h>
#include <linux/fdtable.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/anon_inodes.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/sync_file.h>
#include <linux/slab.h>
#include "mali_kbase_fence_defs.h"
#include "mali_kbase_sync.h"
#include "mali_kbase_fence.h"
#include "mali_kbase.h"

static const struct file_operations stream_fops = {
	.owner = THIS_MODULE
};

int kbase_sync_fence_stream_create(const char *name, int *const out_fd)
{
	if (!out_fd)
		return -EINVAL;

	*out_fd = anon_inode_getfd(name, &stream_fops, NULL,
				   O_RDONLY | O_CLOEXEC);
	if (*out_fd < 0)
		return -EINVAL;

	return 0;
}

#if !MALI_USE_CSF
int kbase_sync_fence_out_create(struct kbase_jd_atom *katom, int stream_fd)
{
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
	struct fence *fence;
#else
	struct dma_fence *fence;
#endif
	struct sync_file *sync_file;
	int fd;

	fence = kbase_fence_out_new(katom);
	if (!fence)
		return -ENOMEM;

#if (KERNEL_VERSION(4, 9, 67) >= LINUX_VERSION_CODE)
	/* Take an extra reference to the fence on behalf of the sync_file.
	 * This is only needed on older kernels where sync_file_create()
	 * does not take its own reference. This was changed in v4.9.68,
	 * where sync_file_create() now takes its own reference.
	 */
	dma_fence_get(fence);
#endif

	/* create a sync_file fd representing the fence */
	sync_file = sync_file_create(fence);
	if (!sync_file) {
#if (KERNEL_VERSION(4, 9, 67) >= LINUX_VERSION_CODE)
		dma_fence_put(fence);
#endif
		kbase_fence_out_remove(katom);
		return -ENOMEM;
	}

	fd = get_unused_fd_flags(O_CLOEXEC);
	if (fd < 0) {
		fput(sync_file->file);
		kbase_fence_out_remove(katom);
		return fd;
	}

	fd_install(fd, sync_file->file);

	return fd;
}

int kbase_sync_fence_in_from_fd(struct kbase_jd_atom *katom, int fd)
{
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
	struct fence *fence = sync_file_get_fence(fd);
#else
	struct dma_fence *fence = sync_file_get_fence(fd);
#endif

	lockdep_assert_held(&katom->kctx->jctx.lock);

	if (!fence)
		return -ENOENT;

	kbase_fence_fence_in_set(katom, fence);
	katom->dma_fence.fence_cb_added = false;

	return 0;
}
#endif /* !MALI_USE_CSF */

int kbase_sync_fence_validate(int fd)
{
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
	struct fence *fence = sync_file_get_fence(fd);
#else
	struct dma_fence *fence = sync_file_get_fence(fd);
#endif

	if (!fence)
		return -EINVAL;

	dma_fence_put(fence);

	return 0; /* valid */
}

#if !MALI_USE_CSF
enum base_jd_event_code
kbase_sync_fence_out_trigger(struct kbase_jd_atom *katom, int result)
{
	int res;

	if (!kbase_fence_out_is_ours(katom)) {
		/* Not our fence */
		return BASE_JD_EVENT_JOB_CANCELLED;
	}

	res = kbase_fence_out_signal(katom, result);
	if (unlikely(res < 0)) {
		dev_warn(katom->kctx->kbdev->dev,
				"fence_signal() failed with %d\n", res);
	}

	kbase_sync_fence_out_remove(katom);

	return (result != 0) ? BASE_JD_EVENT_JOB_CANCELLED : BASE_JD_EVENT_DONE;
}

#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
static void kbase_fence_wait_callback(struct fence *fence,
				      struct fence_cb *cb)
#else
static void kbase_fence_wait_callback(struct dma_fence *fence,
				      struct dma_fence_cb *cb)
#endif
{
	struct kbase_jd_atom *katom = container_of(cb, struct kbase_jd_atom,
						   dma_fence.fence_cb);
	struct kbase_context *kctx = katom->kctx;

	/* Cancel atom if fence is erroneous */
	if (dma_fence_is_signaled(katom->dma_fence.fence_in) &&
#if (KERNEL_VERSION(4, 11, 0) <= LINUX_VERSION_CODE || \
	 (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE && \
	  KERNEL_VERSION(4, 9, 68) <= LINUX_VERSION_CODE))
	    katom->dma_fence.fence_in->error < 0)
#else
	    katom->dma_fence.fence_in->status < 0)
#endif
		katom->event_code = BASE_JD_EVENT_JOB_CANCELLED;


	/* To prevent a potential deadlock we schedule the work onto the
	 * job_done_wq workqueue
	 *
	 * The issue is that we may signal the timeline while holding
	 * kctx->jctx.lock and the callbacks are run synchronously from
	 * sync_timeline_signal. So we simply defer the work.
	 */
	INIT_WORK(&katom->work, kbase_sync_fence_wait_worker);
	queue_work(kctx->jctx.job_done_wq, &katom->work);
}

int kbase_sync_fence_in_wait(struct kbase_jd_atom *katom)
{
	int err;
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
	struct fence *fence;
#else
	struct dma_fence *fence;
#endif

	lockdep_assert_held(&katom->kctx->jctx.lock);

	fence = katom->dma_fence.fence_in;
	if (!fence)
		return 0; /* no input fence to wait for, good to go! */

	err = dma_fence_add_callback(fence, &katom->dma_fence.fence_cb,
				     kbase_fence_wait_callback);
	if (err == -ENOENT) {
		int fence_status = dma_fence_get_status(fence);

		if (fence_status == 1) {
			/* Fence is already signaled with no error. The completion
			 * for FENCE_WAIT softjob can be done right away.
			 */
			return 0;
		}

		/* Fence shouldn't be in not signaled state */
		if (!fence_status) {
			struct kbase_sync_fence_info info;

			kbase_sync_fence_in_info_get(katom, &info);

			dev_warn(katom->kctx->kbdev->dev,
				 "Unexpected status for fence %s of ctx:%d_%d atom:%d",
				 info.name, katom->kctx->tgid, katom->kctx->id,
				 kbase_jd_atom_id(katom->kctx, katom));
		}

		/* If fence is signaled with an error, then the FENCE_WAIT softjob is
		 * considered to be failed.
		 */
	}

	if (unlikely(err)) {
		/* We should cause the dependent jobs in the bag to be failed. */
		katom->event_code = BASE_JD_EVENT_JOB_CANCELLED;

		/* The completion for FENCE_WAIT softjob can be done right away. */
		return 0;
	}

	/* Callback was successfully installed */
	katom->dma_fence.fence_cb_added = true;

	/* Completion to be done later by callback/worker */
	return 1;
}

void kbase_sync_fence_in_cancel_wait(struct kbase_jd_atom *katom)
{
	lockdep_assert_held(&katom->kctx->jctx.lock);

	if (katom->dma_fence.fence_cb_added) {
		if (!dma_fence_remove_callback(katom->dma_fence.fence_in,
					       &katom->dma_fence.fence_cb)) {
			/* The callback is already removed so leave the cleanup
			 * for kbase_fence_wait_callback.
			 */
			return;
		}
	} else {
		struct kbase_sync_fence_info info;

		kbase_sync_fence_in_info_get(katom, &info);
		dev_warn(katom->kctx->kbdev->dev,
			 "Callback was not added earlier for fence %s of ctx:%d_%d atom:%d",
			 info.name, katom->kctx->tgid, katom->kctx->id,
			 kbase_jd_atom_id(katom->kctx, katom));
	}

	/* Wait was cancelled - zap the atoms */
	katom->event_code = BASE_JD_EVENT_JOB_CANCELLED;

	kbasep_remove_waiting_soft_job(katom);
	kbase_finish_soft_job(katom);

	if (kbase_jd_done_nolock(katom, true))
		kbase_js_sched_all(katom->kctx->kbdev);
}

void kbase_sync_fence_out_remove(struct kbase_jd_atom *katom)
{
	kbase_fence_out_remove(katom);
}

void kbase_sync_fence_in_remove(struct kbase_jd_atom *katom)
{
	lockdep_assert_held(&katom->kctx->jctx.lock);

	if (katom->dma_fence.fence_cb_added) {
		bool removed = dma_fence_remove_callback(katom->dma_fence.fence_in,
							 &katom->dma_fence.fence_cb);

		/* Here it is expected that the callback should have already been removed
		 * previously either by kbase_sync_fence_in_cancel_wait() or when the fence
		 * was signaled and kbase_sync_fence_wait_worker() was called.
		 */
		if (removed) {
			struct kbase_sync_fence_info info;

			kbase_sync_fence_in_info_get(katom, &info);
			dev_warn(katom->kctx->kbdev->dev,
				 "Callback was not removed earlier for fence %s of ctx:%d_%d atom:%d",
				 info.name, katom->kctx->tgid, katom->kctx->id,
				 kbase_jd_atom_id(katom->kctx, katom));
		}
	}

	kbase_fence_in_remove(katom);
	katom->dma_fence.fence_cb_added = false;
}
#endif /* !MALI_USE_CSF */

#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
void kbase_sync_fence_info_get(struct fence *fence,
			       struct kbase_sync_fence_info *info)
#else
void kbase_sync_fence_info_get(struct dma_fence *fence,
			       struct kbase_sync_fence_info *info)
#endif
{
	info->fence = fence;

	/* Translate into the following status, with support for error handling:
	 * < 0 : error
	 * 0 : active
	 * 1 : signaled
	 */
	if (dma_fence_is_signaled(fence)) {
#if (KERNEL_VERSION(4, 11, 0) <= LINUX_VERSION_CODE || \
	 (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE && \
	  KERNEL_VERSION(4, 9, 68) <= LINUX_VERSION_CODE))
		int status = fence->error;
#else
		int status = fence->status;
#endif
		if (status < 0)
			info->status = status; /* signaled with error */
		else
			info->status = 1; /* signaled with success */
	} else  {
		info->status = 0; /* still active (unsignaled) */
	}

#if (KERNEL_VERSION(5, 1, 0) > LINUX_VERSION_CODE)
	scnprintf(info->name, sizeof(info->name), "%llu#%u",
		  fence->context, fence->seqno);
#else
	scnprintf(info->name, sizeof(info->name), "%llu#%llu",
		  fence->context, fence->seqno);
#endif
}

#if !MALI_USE_CSF
int kbase_sync_fence_in_info_get(struct kbase_jd_atom *katom,
				 struct kbase_sync_fence_info *info)
{
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
	struct fence *fence;
#else
	struct dma_fence *fence;
#endif

	fence = kbase_fence_in_get(katom);
	if (!fence)
		return -ENOENT;

	kbase_sync_fence_info_get(fence, info);

	kbase_fence_put(fence);

	return 0;
}

int kbase_sync_fence_out_info_get(struct kbase_jd_atom *katom,
				  struct kbase_sync_fence_info *info)
{
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
	struct fence *fence;
#else
	struct dma_fence *fence;
#endif

	fence = kbase_fence_out_get(katom);
	if (!fence)
		return -ENOENT;

	kbase_sync_fence_info_get(fence, info);

	kbase_fence_put(fence);

	return 0;
}


#ifdef CONFIG_MALI_BIFROST_FENCE_DEBUG
void kbase_sync_fence_in_dump(struct kbase_jd_atom *katom)
{
	/* Not implemented */
}
#endif
#endif /* !MALI_USE_CSF*/
