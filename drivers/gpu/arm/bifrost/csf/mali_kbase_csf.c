// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2018-2023 ARM Limited. All rights reserved.
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

#include <mali_kbase.h>
#include <gpu/mali_kbase_gpu_fault.h>
#include <mali_kbase_reset_gpu.h>
#include "mali_kbase_csf.h"
#include "backend/gpu/mali_kbase_pm_internal.h"
#include <linux/export.h>
#include <linux/priority_control_manager.h>
#include <linux/shmem_fs.h>
#include <csf/mali_kbase_csf_registers.h>
#include "mali_kbase_csf_tiler_heap.h"
#include <mmu/mali_kbase_mmu.h>
#include "mali_kbase_csf_timeout.h"
#include <csf/ipa_control/mali_kbase_csf_ipa_control.h>
#include <mali_kbase_hwaccess_time.h>
#include "mali_kbase_csf_event.h"
#include <tl/mali_kbase_tracepoints.h>
#include "mali_kbase_csf_mcu_shared_reg.h"

#define CS_REQ_EXCEPTION_MASK (CS_REQ_FAULT_MASK | CS_REQ_FATAL_MASK)
#define CS_ACK_EXCEPTION_MASK (CS_ACK_FAULT_MASK | CS_ACK_FATAL_MASK)

#define CS_RING_BUFFER_MAX_SIZE ((uint32_t)(1 << 31)) /* 2GiB */
#define CS_RING_BUFFER_MIN_SIZE ((uint32_t)4096)

#define PROTM_ALLOC_MAX_RETRIES ((u8)5)

const u8 kbasep_csf_queue_group_priority_to_relative[BASE_QUEUE_GROUP_PRIORITY_COUNT] = {
	KBASE_QUEUE_GROUP_PRIORITY_HIGH,
	KBASE_QUEUE_GROUP_PRIORITY_MEDIUM,
	KBASE_QUEUE_GROUP_PRIORITY_LOW,
	KBASE_QUEUE_GROUP_PRIORITY_REALTIME
};
const u8 kbasep_csf_relative_to_queue_group_priority[KBASE_QUEUE_GROUP_PRIORITY_COUNT] = {
	BASE_QUEUE_GROUP_PRIORITY_REALTIME,
	BASE_QUEUE_GROUP_PRIORITY_HIGH,
	BASE_QUEUE_GROUP_PRIORITY_MEDIUM,
	BASE_QUEUE_GROUP_PRIORITY_LOW
};

/*
 * struct irq_idle_and_protm_track - Object that tracks the idle and protected mode
 *                                   request information in an interrupt case across
 *                                   groups.
 *
 * @protm_grp: Possibly schedulable group that requested protected mode in the interrupt.
 *             If NULL, no such case observed in the tracked interrupt case.
 * @idle_seq:  The highest priority group that notified idle. If no such instance in the
 *             interrupt case, marked with the largest field value: U32_MAX.
 * @idle_slot: The slot number if @p idle_seq is valid in the given tracking case.
 */
struct irq_idle_and_protm_track {
	struct kbase_queue_group *protm_grp;
	u32 idle_seq;
	s8 idle_slot;
};

/**
 * kbasep_ctx_user_reg_page_mapping_term() - Terminate resources for USER Register Page.
 *
 * @kctx:   Pointer to the kbase context
 */
static void kbasep_ctx_user_reg_page_mapping_term(struct kbase_context *kctx)
{
	struct kbase_device *kbdev = kctx->kbdev;

	if (unlikely(kctx->csf.user_reg.vma))
		dev_err(kbdev->dev, "VMA for USER Register page exist on termination of ctx %d_%d",
			kctx->tgid, kctx->id);
	if (WARN_ON_ONCE(!list_empty(&kctx->csf.user_reg.link)))
		list_del_init(&kctx->csf.user_reg.link);
}

/**
 * kbasep_ctx_user_reg_page_mapping_init() - Initialize resources for USER Register Page.
 *
 * @kctx:   Pointer to the kbase context
 *
 * @return: 0 on success.
 */
static int kbasep_ctx_user_reg_page_mapping_init(struct kbase_context *kctx)
{
	INIT_LIST_HEAD(&kctx->csf.user_reg.link);
	kctx->csf.user_reg.vma = NULL;
	kctx->csf.user_reg.file_offset = 0;

	return 0;
}

static void put_user_pages_mmap_handle(struct kbase_context *kctx,
			struct kbase_queue *queue)
{
	unsigned long cookie_nr;

	lockdep_assert_held(&kctx->csf.lock);

	if (queue->handle == BASEP_MEM_INVALID_HANDLE)
		return;

	cookie_nr =
		PFN_DOWN(queue->handle - BASEP_MEM_CSF_USER_IO_PAGES_HANDLE);

	if (!WARN_ON(kctx->csf.user_pages_info[cookie_nr] != queue)) {
		/* free up cookie */
		kctx->csf.user_pages_info[cookie_nr] = NULL;
		bitmap_set(kctx->csf.cookies, cookie_nr, 1);
	}

	queue->handle = BASEP_MEM_INVALID_HANDLE;
}

/* Reserve a cookie, to be returned as a handle to userspace for creating
 * the CPU mapping of the pair of input/output pages and Hw doorbell page.
 * Will return 0 in case of success otherwise negative on failure.
 */
static int get_user_pages_mmap_handle(struct kbase_context *kctx,
			struct kbase_queue *queue)
{
	unsigned long cookie, cookie_nr;

	lockdep_assert_held(&kctx->csf.lock);

	if (bitmap_empty(kctx->csf.cookies,
				KBASE_CSF_NUM_USER_IO_PAGES_HANDLE)) {
		dev_err(kctx->kbdev->dev,
			"No csf cookies available for allocation!");
		return -ENOMEM;
	}

	/* allocate a cookie */
	cookie_nr = find_first_bit(kctx->csf.cookies,
				KBASE_CSF_NUM_USER_IO_PAGES_HANDLE);
	if (kctx->csf.user_pages_info[cookie_nr]) {
		dev_err(kctx->kbdev->dev,
			"Inconsistent state of csf cookies!");
		return -EINVAL;
	}
	kctx->csf.user_pages_info[cookie_nr] = queue;
	bitmap_clear(kctx->csf.cookies, cookie_nr, 1);

	/* relocate to correct base */
	cookie = cookie_nr + PFN_DOWN(BASEP_MEM_CSF_USER_IO_PAGES_HANDLE);
	cookie <<= PAGE_SHIFT;

	queue->handle = (u64)cookie;

	return 0;
}

static void init_user_io_pages(struct kbase_queue *queue)
{
	u32 *input_addr = (u32 *)(queue->user_io_addr);
	u32 *output_addr = (u32 *)(queue->user_io_addr + PAGE_SIZE);

	input_addr[CS_INSERT_LO/4] = 0;
	input_addr[CS_INSERT_HI/4] = 0;

	input_addr[CS_EXTRACT_INIT_LO/4] = 0;
	input_addr[CS_EXTRACT_INIT_HI/4] = 0;

	output_addr[CS_EXTRACT_LO/4] = 0;
	output_addr[CS_EXTRACT_HI/4] = 0;

	output_addr[CS_ACTIVE/4] = 0;
}

static void kernel_unmap_user_io_pages(struct kbase_context *kctx,
			struct kbase_queue *queue)
{
	kbase_gpu_vm_lock(kctx);

	vunmap(queue->user_io_addr);

	WARN_ON(atomic_read(&kctx->permanent_mapped_pages) < KBASEP_NUM_CS_USER_IO_PAGES);
	atomic_sub(KBASEP_NUM_CS_USER_IO_PAGES, &kctx->permanent_mapped_pages);

	kbase_gpu_vm_unlock(kctx);
}

static int kernel_map_user_io_pages(struct kbase_context *kctx,
			struct kbase_queue *queue)
{
	struct page *page_list[2];
	pgprot_t cpu_map_prot;
	unsigned long flags;
	char *user_io_addr;
	int ret = 0;
	size_t i;

	kbase_gpu_vm_lock(kctx);

	if (ARRAY_SIZE(page_list) > (KBASE_PERMANENTLY_MAPPED_MEM_LIMIT_PAGES -
			 atomic_read(&kctx->permanent_mapped_pages))) {
		ret = -ENOMEM;
		goto unlock;
	}

	/* The pages are mapped to Userspace also, so use the same mapping
	 * attributes as used inside the CPU page fault handler.
	 */
	if (kctx->kbdev->system_coherency == COHERENCY_NONE)
		cpu_map_prot = pgprot_writecombine(PAGE_KERNEL);
	else
		cpu_map_prot = PAGE_KERNEL;

	for (i = 0; i < ARRAY_SIZE(page_list); i++)
		page_list[i] = as_page(queue->phys[i]);

	user_io_addr = vmap(page_list, ARRAY_SIZE(page_list), VM_MAP, cpu_map_prot);

	if (!user_io_addr) {
		dev_err(kctx->kbdev->dev,
			"%s(): user_io_addr is NULL, queue: %p",
			__func__,
			queue);
		ret = -ENOMEM;
	} else {
		atomic_add(ARRAY_SIZE(page_list), &kctx->permanent_mapped_pages);
	}

	kbase_csf_scheduler_spin_lock(kctx->kbdev, &flags);
	queue->user_io_addr = user_io_addr;
	kbase_csf_scheduler_spin_unlock(kctx->kbdev, flags);

unlock:
	kbase_gpu_vm_unlock(kctx);
	return ret;
}

static void term_queue_group(struct kbase_queue_group *group);
static void get_queue(struct kbase_queue *queue);
static void release_queue(struct kbase_queue *queue);

/**
 * kbase_csf_free_command_stream_user_pages() - Free the resources allocated
 *				    for a queue at the time of bind.
 *
 * @kctx:	Address of the kbase context within which the queue was created.
 * @queue:	Pointer to the queue to be unlinked.
 *
 * This function will free the pair of physical pages allocated for a GPU
 * command queue, and also release the hardware doorbell page, that were mapped
 * into the process address space to enable direct submission of commands to
 * the hardware. Also releases the reference taken on the queue when the mapping
 * was created.
 *
 * This function will be called only when the mapping is being removed and
 * so the resources for queue will not get freed up until the mapping is
 * removed even though userspace could have terminated the queue.
 * Kernel will ensure that the termination of Kbase context would only be
 * triggered after the mapping is removed.
 *
 * If an explicit or implicit unbind was missed by the userspace then the
 * mapping will persist. On process exit kernel itself will remove the mapping.
 */
void kbase_csf_free_command_stream_user_pages(struct kbase_context *kctx, struct kbase_queue *queue)
{
	kernel_unmap_user_io_pages(kctx, queue);

	kbase_mem_pool_free_pages(
		&kctx->mem_pools.small[KBASE_MEM_GROUP_CSF_IO],
		KBASEP_NUM_CS_USER_IO_PAGES, queue->phys, true, false);
	kbase_process_page_usage_dec(kctx, KBASEP_NUM_CS_USER_IO_PAGES);

	/* The user_io_gpu_va should have been unmapped inside the scheduler */
	WARN_ONCE(queue->user_io_gpu_va, "Userio pages appears still have mapping");

	/* If the queue has already been terminated by userspace
	 * then the ref count for queue object will drop to 0 here.
	 */
	release_queue(queue);
}
KBASE_EXPORT_TEST_API(kbase_csf_free_command_stream_user_pages);

int kbase_csf_alloc_command_stream_user_pages(struct kbase_context *kctx, struct kbase_queue *queue)
{
	struct kbase_device *kbdev = kctx->kbdev;
	int ret;

	lockdep_assert_held(&kctx->csf.lock);

	ret = kbase_mem_pool_alloc_pages(&kctx->mem_pools.small[KBASE_MEM_GROUP_CSF_IO],
					 KBASEP_NUM_CS_USER_IO_PAGES,
					 queue->phys, false, kctx->task);
	if (ret != KBASEP_NUM_CS_USER_IO_PAGES) {
		/* Marking both the phys to zero for indicating there is no phys allocated */
		queue->phys[0].tagged_addr = 0;
		queue->phys[1].tagged_addr = 0;
		return -ENOMEM;
	}

	ret = kernel_map_user_io_pages(kctx, queue);
	if (ret)
		goto kernel_map_failed;

	kbase_process_page_usage_inc(kctx, KBASEP_NUM_CS_USER_IO_PAGES);
	init_user_io_pages(queue);

	/* user_io_gpu_va is only mapped when scheduler decides to put the queue
	 * on slot at runtime. Initialize it to 0, signalling no mapping.
	 */
	queue->user_io_gpu_va = 0;

	mutex_lock(&kbdev->csf.reg_lock);
	if (kbdev->csf.db_file_offsets > (U32_MAX - BASEP_QUEUE_NR_MMAP_USER_PAGES + 1))
		kbdev->csf.db_file_offsets = 0;

	queue->db_file_offset = kbdev->csf.db_file_offsets;
	kbdev->csf.db_file_offsets += BASEP_QUEUE_NR_MMAP_USER_PAGES;
	WARN(kbase_refcount_read(&queue->refcount) != 1,
	     "Incorrect refcounting for queue object\n");
	/* This is the second reference taken on the queue object and
	 * would be dropped only when the IO mapping is removed either
	 * explicitly by userspace or implicitly by kernel on process exit.
	 */
	get_queue(queue);
	queue->bind_state = KBASE_CSF_QUEUE_BOUND;
	mutex_unlock(&kbdev->csf.reg_lock);

	return 0;

kernel_map_failed:
	kbase_mem_pool_free_pages(&kctx->mem_pools.small[KBASE_MEM_GROUP_CSF_IO],
				  KBASEP_NUM_CS_USER_IO_PAGES, queue->phys, false, false);
	/* Marking both the phys to zero for indicating there is no phys allocated */
	queue->phys[0].tagged_addr = 0;
	queue->phys[1].tagged_addr = 0;

	return ret;
}
KBASE_EXPORT_TEST_API(kbase_csf_alloc_command_stream_user_pages);

static struct kbase_queue_group *find_queue_group(struct kbase_context *kctx,
	u8 group_handle)
{
	uint index = group_handle;

	lockdep_assert_held(&kctx->csf.lock);

	if (index < MAX_QUEUE_GROUP_NUM && kctx->csf.queue_groups[index]) {
		if (WARN_ON(kctx->csf.queue_groups[index]->handle != index))
			return NULL;
		return kctx->csf.queue_groups[index];
	}

	return NULL;
}

struct kbase_queue_group *kbase_csf_find_queue_group(struct kbase_context *kctx, u8 group_handle)
{
	return find_queue_group(kctx, group_handle);
}
KBASE_EXPORT_TEST_API(kbase_csf_find_queue_group);

int kbase_csf_queue_group_handle_is_valid(struct kbase_context *kctx,
	u8 group_handle)
{
	struct kbase_queue_group *group;

	mutex_lock(&kctx->csf.lock);
	group = find_queue_group(kctx, group_handle);
	mutex_unlock(&kctx->csf.lock);

	return group ? 0 : -EINVAL;
}

static struct kbase_queue *find_queue(struct kbase_context *kctx, u64 base_addr)
{
	struct kbase_queue *queue;

	lockdep_assert_held(&kctx->csf.lock);

	list_for_each_entry(queue, &kctx->csf.queue_list, link) {
		if (base_addr == queue->base_addr)
			return queue;
	}

	return NULL;
}

static void get_queue(struct kbase_queue *queue)
{
	WARN_ON(!kbase_refcount_inc_not_zero(&queue->refcount));
}

static void release_queue(struct kbase_queue *queue)
{
	lockdep_assert_held(&queue->kctx->csf.lock);
	if (kbase_refcount_dec_and_test(&queue->refcount)) {
		/* The queue can't still be on the per context list. */
		WARN_ON(!list_empty(&queue->link));
		WARN_ON(queue->group);
		dev_dbg(queue->kctx->kbdev->dev,
			"Remove any pending command queue fatal from ctx %d_%d",
			queue->kctx->tgid, queue->kctx->id);
		kbase_csf_event_remove_error(queue->kctx, &queue->error);

		/* After this the Userspace would be able to free the
		 * memory for GPU queue. In case the Userspace missed
		 * terminating the queue, the cleanup will happen on
		 * context termination where tear down of region tracker
		 * would free up the GPU queue memory.
		 */
		kbase_gpu_vm_lock(queue->kctx);
		kbase_va_region_no_user_free_dec(queue->queue_reg);
		kbase_gpu_vm_unlock(queue->kctx);

		kfree(queue);
	}
}

static void oom_event_worker(struct work_struct *data);
static void cs_error_worker(struct work_struct *data);

/* Between reg and reg_ex, one and only one must be null */
static int csf_queue_register_internal(struct kbase_context *kctx,
			struct kbase_ioctl_cs_queue_register *reg,
			struct kbase_ioctl_cs_queue_register_ex *reg_ex)
{
	struct kbase_queue *queue;
	int ret = 0;
	struct kbase_va_region *region;
	u64 queue_addr;
	size_t queue_size;

	/* Only one pointer expected, otherwise coding error */
	if ((reg == NULL && reg_ex == NULL) || (reg && reg_ex)) {
		dev_dbg(kctx->kbdev->dev,
			"Error, one and only one param-ptr expected!");
		return -EINVAL;
	}

	/* struct kbase_ioctl_cs_queue_register_ex contains a full
	 * struct kbase_ioctl_cs_queue_register at the start address. So
	 * the pointer can be safely cast to pointing to a
	 * kbase_ioctl_cs_queue_register object.
	 */
	if (reg_ex)
		reg = (struct kbase_ioctl_cs_queue_register *)reg_ex;

	/* Validate the queue priority */
	if (reg->priority > BASE_QUEUE_MAX_PRIORITY)
		return -EINVAL;

	queue_addr = reg->buffer_gpu_addr;
	queue_size = reg->buffer_size >> PAGE_SHIFT;

	mutex_lock(&kctx->csf.lock);

	/* Check if queue is already registered */
	if (find_queue(kctx, queue_addr) != NULL) {
		ret = -EINVAL;
		goto out;
	}

	/* Check if the queue address is valid */
	kbase_gpu_vm_lock(kctx);
	region = kbase_region_tracker_find_region_enclosing_address(kctx,
								    queue_addr);

	if (kbase_is_region_invalid_or_free(region) || kbase_is_region_shrinkable(region) ||
	    region->gpu_alloc->type != KBASE_MEM_TYPE_NATIVE) {
		ret = -ENOENT;
		goto out_unlock_vm;
	}

	if (queue_size > (region->nr_pages -
			  ((queue_addr >> PAGE_SHIFT) - region->start_pfn))) {
		ret = -EINVAL;
		goto out_unlock_vm;
	}

	/* Check address validity on cs_trace buffer etc. Don't care
	 * if not enabled (i.e. when size is 0).
	 */
	if (reg_ex && reg_ex->ex_buffer_size) {
		int buf_pages = (reg_ex->ex_buffer_size +
				 (1 << PAGE_SHIFT) - 1) >> PAGE_SHIFT;
		struct kbase_va_region *region_ex =
			kbase_region_tracker_find_region_enclosing_address(kctx,
									   reg_ex->ex_buffer_base);

		if (kbase_is_region_invalid_or_free(region_ex)) {
			ret = -ENOENT;
			goto out_unlock_vm;
		}

		if (buf_pages > (region_ex->nr_pages -
				 ((reg_ex->ex_buffer_base >> PAGE_SHIFT) - region_ex->start_pfn))) {
			ret = -EINVAL;
			goto out_unlock_vm;
		}

		region_ex = kbase_region_tracker_find_region_enclosing_address(
			kctx, reg_ex->ex_offset_var_addr);
		if (kbase_is_region_invalid_or_free(region_ex)) {
			ret = -ENOENT;
			goto out_unlock_vm;
		}
	}

	queue = kzalloc(sizeof(struct kbase_queue), GFP_KERNEL);

	if (!queue) {
		ret = -ENOMEM;
		goto out_unlock_vm;
	}

	queue->kctx = kctx;
	queue->base_addr = queue_addr;

	queue->queue_reg = region;
	kbase_va_region_no_user_free_inc(region);

	queue->size = (queue_size << PAGE_SHIFT);
	queue->csi_index = KBASEP_IF_NR_INVALID;
	queue->enabled = false;

	queue->priority = reg->priority;
	kbase_refcount_set(&queue->refcount, 1);

	queue->group = NULL;
	queue->bind_state = KBASE_CSF_QUEUE_UNBOUND;
	queue->handle = BASEP_MEM_INVALID_HANDLE;
	queue->doorbell_nr = KBASEP_USER_DB_NR_INVALID;

	queue->status_wait = 0;
	queue->sync_ptr = 0;
	queue->sync_value = 0;

#if IS_ENABLED(CONFIG_DEBUG_FS)
	queue->saved_cmd_ptr = 0;
#endif

	queue->sb_status = 0;
	queue->blocked_reason = CS_STATUS_BLOCKED_REASON_REASON_UNBLOCKED;

	atomic_set(&queue->pending, 0);

	INIT_LIST_HEAD(&queue->link);
	INIT_LIST_HEAD(&queue->error.link);
	INIT_WORK(&queue->oom_event_work, oom_event_worker);
	INIT_WORK(&queue->cs_error_work, cs_error_worker);
	list_add(&queue->link, &kctx->csf.queue_list);

	queue->extract_ofs = 0;

	region->user_data = queue;

	/* Initialize the cs_trace configuration parameters, When buffer_size
	 * is 0, trace is disabled. Here we only update the fields when
	 * enabled, otherwise leave them as default zeros.
	 */
	if (reg_ex && reg_ex->ex_buffer_size) {
		u32 cfg = CS_INSTR_CONFIG_EVENT_SIZE_SET(
					0, reg_ex->ex_event_size);
		cfg = CS_INSTR_CONFIG_EVENT_STATE_SET(
					cfg, reg_ex->ex_event_state);

		queue->trace_cfg = cfg;
		queue->trace_buffer_size = reg_ex->ex_buffer_size;
		queue->trace_buffer_base = reg_ex->ex_buffer_base;
		queue->trace_offset_ptr = reg_ex->ex_offset_var_addr;
	}

out_unlock_vm:
	kbase_gpu_vm_unlock(kctx);
out:
	mutex_unlock(&kctx->csf.lock);

	return ret;
}

int kbase_csf_queue_register(struct kbase_context *kctx,
			     struct kbase_ioctl_cs_queue_register *reg)
{
	/* Validate the ring buffer configuration parameters */
	if (reg->buffer_size < CS_RING_BUFFER_MIN_SIZE ||
	    reg->buffer_size > CS_RING_BUFFER_MAX_SIZE ||
	    reg->buffer_size & (reg->buffer_size - 1) || !reg->buffer_gpu_addr ||
	    reg->buffer_gpu_addr & ~PAGE_MASK)
		return -EINVAL;

	return csf_queue_register_internal(kctx, reg, NULL);
}

int kbase_csf_queue_register_ex(struct kbase_context *kctx,
				struct kbase_ioctl_cs_queue_register_ex *reg)
{
	struct kbase_csf_global_iface const *const iface =
						&kctx->kbdev->csf.global_iface;
	u32 const glb_version = iface->version;
	u32 instr = iface->instr_features;
	u8 max_size = GLB_INSTR_FEATURES_EVENT_SIZE_MAX_GET(instr);
	u32 min_buf_size = (1u << reg->ex_event_size) *
			GLB_INSTR_FEATURES_OFFSET_UPDATE_RATE_GET(instr);

	/* If cs_trace_command not supported, the call fails */
	if (glb_version < kbase_csf_interface_version(1, 1, 0))
		return -EINVAL;

	/* Validate the ring buffer configuration parameters */
	if (reg->buffer_size < CS_RING_BUFFER_MIN_SIZE ||
	    reg->buffer_size > CS_RING_BUFFER_MAX_SIZE ||
	    reg->buffer_size & (reg->buffer_size - 1) || !reg->buffer_gpu_addr ||
	    reg->buffer_gpu_addr & ~PAGE_MASK)
		return -EINVAL;

	/* Validate the cs_trace configuration parameters */
	if (reg->ex_buffer_size &&
		((reg->ex_event_size > max_size) ||
			(reg->ex_buffer_size & (reg->ex_buffer_size - 1)) ||
			(reg->ex_buffer_size < min_buf_size)))
		return -EINVAL;

	return csf_queue_register_internal(kctx, NULL, reg);
}

static void unbind_queue(struct kbase_context *kctx,
		struct kbase_queue *queue);

void kbase_csf_queue_terminate(struct kbase_context *kctx,
			      struct kbase_ioctl_cs_queue_terminate *term)
{
	struct kbase_device *kbdev = kctx->kbdev;
	struct kbase_queue *queue;
	int err;
	bool reset_prevented = false;

	err = kbase_reset_gpu_prevent_and_wait(kbdev);
	if (err)
		dev_warn(
			kbdev->dev,
			"Unsuccessful GPU reset detected when terminating queue (buffer_addr=0x%.16llx), attempting to terminate regardless",
			term->buffer_gpu_addr);
	else
		reset_prevented = true;

	mutex_lock(&kctx->csf.lock);
	queue = find_queue(kctx, term->buffer_gpu_addr);

	if (queue) {
		/* As the GPU queue has been terminated by the
		 * user space, undo the actions that were performed when the
		 * queue was registered i.e. remove the queue from the per
		 * context list & release the initial reference. The subsequent
		 * lookups for the queue in find_queue() would fail.
		 */
		list_del_init(&queue->link);

		/* Stop the CSI to which queue was bound */
		unbind_queue(kctx, queue);

		kbase_gpu_vm_lock(kctx);
		if (!WARN_ON(!queue->queue_reg))
			queue->queue_reg->user_data = NULL;
		kbase_gpu_vm_unlock(kctx);

		release_queue(queue);
	}

	mutex_unlock(&kctx->csf.lock);
	if (reset_prevented)
		kbase_reset_gpu_allow(kbdev);
}

int kbase_csf_queue_bind(struct kbase_context *kctx, union kbase_ioctl_cs_queue_bind *bind)
{
	struct kbase_queue *queue;
	struct kbase_queue_group *group;
	u8 max_streams;
	int ret = -EINVAL;

	mutex_lock(&kctx->csf.lock);

	group = find_queue_group(kctx, bind->in.group_handle);
	queue = find_queue(kctx, bind->in.buffer_gpu_addr);

	if (!group || !queue)
		goto out;

	/* For the time being, all CSGs have the same number of CSs
	 * so we check CSG 0 for this number
	 */
	max_streams = kctx->kbdev->csf.global_iface.groups[0].stream_num;

	if (bind->in.csi_index >= max_streams)
		goto out;

	if (group->run_state == KBASE_CSF_GROUP_TERMINATED)
		goto out;

	if (queue->group || group->bound_queues[bind->in.csi_index])
		goto out;

	ret = get_user_pages_mmap_handle(kctx, queue);
	if (ret)
		goto out;

	bind->out.mmap_handle = queue->handle;
	group->bound_queues[bind->in.csi_index] = queue;
	queue->group = group;
	queue->csi_index = bind->in.csi_index;
	queue->bind_state = KBASE_CSF_QUEUE_BIND_IN_PROGRESS;

out:
	mutex_unlock(&kctx->csf.lock);

	return ret;
}

static struct kbase_queue_group *get_bound_queue_group(
					struct kbase_queue *queue)
{
	struct kbase_context *kctx = queue->kctx;
	struct kbase_queue_group *group;

	if (queue->bind_state == KBASE_CSF_QUEUE_UNBOUND)
		return NULL;

	if (!queue->group)
		return NULL;

	if (queue->csi_index == KBASEP_IF_NR_INVALID) {
		dev_warn(kctx->kbdev->dev, "CS interface index is incorrect\n");
		return NULL;
	}

	group = queue->group;

	if (group->bound_queues[queue->csi_index] != queue) {
		dev_warn(kctx->kbdev->dev, "Incorrect mapping between queues & queue groups\n");
		return NULL;
	}

	return group;
}

static void enqueue_gpu_submission_work(struct kbase_context *const kctx)
{
	queue_work(system_highpri_wq, &kctx->csf.pending_submission_work);
}

/**
 * pending_submission_worker() - Work item to process pending kicked GPU command queues.
 *
 * @work: Pointer to pending_submission_work.
 *
 * This function starts all pending queues, for which the work
 * was previously submitted via ioctl call from application thread.
 * If the queue is already scheduled and resident, it will be started
 * right away, otherwise once the group is made resident.
 */
static void pending_submission_worker(struct work_struct *work)
{
	struct kbase_context *kctx =
		container_of(work, struct kbase_context, csf.pending_submission_work);
	struct kbase_device *kbdev = kctx->kbdev;
	struct kbase_queue *queue;
	int err = kbase_reset_gpu_prevent_and_wait(kbdev);

	if (err) {
		dev_err(kbdev->dev, "Unsuccessful GPU reset detected when kicking queue ");
		return;
	}

	mutex_lock(&kctx->csf.lock);

	/* Iterate through the queue list and schedule the pending ones for submission. */
	list_for_each_entry(queue, &kctx->csf.queue_list, link) {
		if (atomic_cmpxchg(&queue->pending, 1, 0) == 1) {
			struct kbase_queue_group *group = get_bound_queue_group(queue);
			int ret;

			if (!group || queue->bind_state != KBASE_CSF_QUEUE_BOUND) {
				dev_dbg(kbdev->dev, "queue is not bound to a group");
				continue;
			}

			ret = kbase_csf_scheduler_queue_start(queue);
			if (unlikely(ret)) {
				dev_dbg(kbdev->dev, "Failed to start queue");
				if (ret == -EBUSY) {
					atomic_cmpxchg(&queue->pending, 0, 1);
					enqueue_gpu_submission_work(kctx);
				}
			}
		}
	}

	mutex_unlock(&kctx->csf.lock);

	kbase_reset_gpu_allow(kbdev);
}

void kbase_csf_ring_csg_doorbell(struct kbase_device *kbdev, int slot)
{
	if (WARN_ON(slot < 0))
		return;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	kbase_csf_ring_csg_slots_doorbell(kbdev, (u32) (1 << slot));
}

void kbase_csf_ring_csg_slots_doorbell(struct kbase_device *kbdev,
				       u32 slot_bitmap)
{
	const struct kbase_csf_global_iface *const global_iface =
		&kbdev->csf.global_iface;
	const u32 allowed_bitmap =
		(u32) ((1U << kbdev->csf.global_iface.group_num) - 1);
	u32 value;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	if (WARN_ON(slot_bitmap > allowed_bitmap))
		return;

	/* The access to GLB_DB_REQ/ACK needs to be ordered with respect to CSG_REQ/ACK and
	 * CSG_DB_REQ/ACK to avoid a scenario where a CSI request overlaps with a CSG request
	 * or 2 CSI requests overlap and FW ends up missing the 2nd request.
	 * Memory barrier is required, both on Host and FW side, to guarantee the ordering.
	 *
	 * 'osh' is used as CPU and GPU would be in the same Outer shareable domain.
	 */
	dmb(osh);

	value = kbase_csf_firmware_global_output(global_iface, GLB_DB_ACK);
	value ^= slot_bitmap;
	kbase_csf_firmware_global_input_mask(global_iface, GLB_DB_REQ, value,
					     slot_bitmap);

	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);
}

void kbase_csf_ring_cs_user_doorbell(struct kbase_device *kbdev,
			struct kbase_queue *queue)
{
	mutex_lock(&kbdev->csf.reg_lock);

	if (queue->doorbell_nr != KBASEP_USER_DB_NR_INVALID)
		kbase_csf_ring_doorbell(kbdev, queue->doorbell_nr);

	mutex_unlock(&kbdev->csf.reg_lock);
}

void kbase_csf_ring_cs_kernel_doorbell(struct kbase_device *kbdev,
				       int csi_index, int csg_nr,
				       bool ring_csg_doorbell)
{
	struct kbase_csf_cmd_stream_group_info *ginfo;
	u32 value;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	if (WARN_ON(csg_nr < 0) ||
	    WARN_ON(csg_nr >= kbdev->csf.global_iface.group_num))
		return;

	ginfo = &kbdev->csf.global_iface.groups[csg_nr];

	if (WARN_ON(csi_index < 0) ||
	    WARN_ON(csi_index >= ginfo->stream_num))
		return;

	/* The access to CSG_DB_REQ/ACK needs to be ordered with respect to
	 * CS_REQ/ACK to avoid a scenario where CSG_DB_REQ/ACK becomes visibile to
	 * FW before CS_REQ/ACK is set.
	 *
	 * 'osh' is used as CPU and GPU would be in the same outer shareable domain.
	 */
	dmb(osh);

	value = kbase_csf_firmware_csg_output(ginfo, CSG_DB_ACK);
	value ^= (1 << csi_index);
	kbase_csf_firmware_csg_input_mask(ginfo, CSG_DB_REQ, value,
					  1 << csi_index);

	if (likely(ring_csg_doorbell))
		kbase_csf_ring_csg_doorbell(kbdev, csg_nr);
}

int kbase_csf_queue_kick(struct kbase_context *kctx,
			 struct kbase_ioctl_cs_queue_kick *kick)
{
	struct kbase_device *kbdev = kctx->kbdev;
	bool trigger_submission = false;
	struct kbase_va_region *region;
	int err = 0;

	KBASE_TLSTREAM_TL_KBASE_GPUCMDQUEUE_KICK(kbdev, kctx->id, kick->buffer_gpu_addr);

	/* GPU work submission happening asynchronously to prevent the contention with
	 * scheduler lock and as the result blocking application thread. For this reason,
	 * the vm_lock is used here to get the reference to the queue based on its buffer_gpu_addr
	 * from the context list of active va_regions.
	 * Once the target queue is found the pending flag is set to one atomically avoiding
	 * a race between submission ioctl thread and the work item.
	 */
	kbase_gpu_vm_lock(kctx);
	region = kbase_region_tracker_find_region_enclosing_address(kctx, kick->buffer_gpu_addr);
	if (!kbase_is_region_invalid_or_free(region)) {
		struct kbase_queue *queue = region->user_data;

		if (queue) {
			atomic_cmpxchg(&queue->pending, 0, 1);
			trigger_submission = true;
		}
	} else {
		dev_dbg(kbdev->dev,
			"Attempt to kick GPU queue without a valid command buffer region");
		err = -EFAULT;
	}
	kbase_gpu_vm_unlock(kctx);

	if (likely(trigger_submission))
		enqueue_gpu_submission_work(kctx);

	return err;
}

static void unbind_stopped_queue(struct kbase_context *kctx,
			struct kbase_queue *queue)
{
	lockdep_assert_held(&kctx->csf.lock);

	if (WARN_ON(queue->csi_index < 0))
		return;

	if (queue->bind_state != KBASE_CSF_QUEUE_UNBOUND) {
		unsigned long flags;

		kbase_csf_scheduler_spin_lock(kctx->kbdev, &flags);
		bitmap_clear(queue->group->protm_pending_bitmap,
				queue->csi_index, 1);
		KBASE_KTRACE_ADD_CSF_GRP_Q(kctx->kbdev, CSI_PROTM_PEND_CLEAR,
			 queue->group, queue, queue->group->protm_pending_bitmap[0]);
		queue->group->bound_queues[queue->csi_index] = NULL;
		queue->group = NULL;
		kbase_csf_scheduler_spin_unlock(kctx->kbdev, flags);

		put_user_pages_mmap_handle(kctx, queue);
		WARN_ON_ONCE(queue->doorbell_nr != KBASEP_USER_DB_NR_INVALID);
		queue->bind_state = KBASE_CSF_QUEUE_UNBOUND;
	}
}
/**
 * unbind_queue() - Remove the linkage between a GPU command queue and the group
 *		    to which it was bound or being bound.
 *
 * @kctx:	Address of the kbase context within which the queue was created.
 * @queue:	Pointer to the queue to be unlinked.
 *
 * This function will also send the stop request to firmware for the CS
 * if the group to which the GPU command queue was bound is scheduled.
 *
 * This function would be called when :-
 * - queue is being unbound. This would happen when the IO mapping
 *   created on bind is removed explicitly by userspace or the process
 *   is getting exited.
 * - queue group is being terminated which still has queues bound
 *   to it. This could happen on an explicit terminate request from userspace
 *   or when the kbase context is being terminated.
 * - queue is being terminated without completing the bind operation.
 *   This could happen if either the queue group is terminated
 *   after the CS_QUEUE_BIND ioctl but before the 2nd part of bind operation
 *   to create the IO mapping is initiated.
 * - There is a failure in executing the 2nd part of bind operation, inside the
 *   mmap handler, which creates the IO mapping for queue.
 */

static void unbind_queue(struct kbase_context *kctx, struct kbase_queue *queue)
{
	kbase_reset_gpu_assert_failed_or_prevented(kctx->kbdev);
	lockdep_assert_held(&kctx->csf.lock);

	if (queue->bind_state != KBASE_CSF_QUEUE_UNBOUND) {
		if (queue->bind_state == KBASE_CSF_QUEUE_BOUND)
			kbase_csf_scheduler_queue_stop(queue);

		unbind_stopped_queue(kctx, queue);
	}
}

static bool kbase_csf_queue_phys_allocated(struct kbase_queue *queue)
{
	/* The queue's phys are zeroed when allocation fails. Both of them being
	 * zero is an impossible condition for a successful allocated set of phy pages.
	 */

	return (queue->phys[0].tagged_addr | queue->phys[1].tagged_addr);
}

void kbase_csf_queue_unbind(struct kbase_queue *queue, bool process_exit)
{
	struct kbase_context *kctx = queue->kctx;

	lockdep_assert_held(&kctx->csf.lock);

	/* As the process itself is exiting, the termination of queue group can
	 * be done which would be much faster than stopping of individual
	 * queues. This would ensure a faster exit for the process especially
	 * in the case where CSI gets stuck.
	 * The CSI STOP request will wait for the in flight work to drain
	 * whereas CSG TERM request would result in an immediate abort or
	 * cancellation of the pending work.
	 */
	if (process_exit) {
		struct kbase_queue_group *group = get_bound_queue_group(queue);

		if (group)
			term_queue_group(group);

		WARN_ON(queue->bind_state != KBASE_CSF_QUEUE_UNBOUND);
	} else {
		unbind_queue(kctx, queue);
	}

	/* Free the resources, if allocated phys for this queue */
	if (kbase_csf_queue_phys_allocated(queue))
		kbase_csf_free_command_stream_user_pages(kctx, queue);
}

void kbase_csf_queue_unbind_stopped(struct kbase_queue *queue)
{
	struct kbase_context *kctx = queue->kctx;

	lockdep_assert_held(&kctx->csf.lock);

	WARN_ON(queue->bind_state == KBASE_CSF_QUEUE_BOUND);
	unbind_stopped_queue(kctx, queue);

	/* Free the resources, if allocated phys for this queue */
	if (kbase_csf_queue_phys_allocated(queue))
		kbase_csf_free_command_stream_user_pages(kctx, queue);
}

/**
 * find_free_group_handle() - Find a free handle for a queue group
 *
 * @kctx: Address of the kbase context within which the queue group
 *        is to be created.
 *
 * Return: a queue group handle on success, or a negative error code on failure.
 */
static int find_free_group_handle(struct kbase_context *const kctx)
{
	/* find the available index in the array of CSGs per this context */
	int idx, group_handle = -ENOMEM;

	lockdep_assert_held(&kctx->csf.lock);

	for (idx = 0;
		(idx != MAX_QUEUE_GROUP_NUM) && (group_handle < 0);
		idx++) {
		if (!kctx->csf.queue_groups[idx])
			group_handle = idx;
	}

	return group_handle;
}

/**
 * iface_has_enough_streams() - Check that at least one CSG supports
 *                              a given number of CS
 *
 * @kbdev:  Instance of a GPU platform device that implements a CSF interface.
 * @cs_min: Minimum number of CSs required.
 *
 * Return: true if at least one CSG supports the given number
 *         of CSs (or more); otherwise false.
 */
static bool iface_has_enough_streams(struct kbase_device *const kbdev,
	u32 const cs_min)
{
	bool has_enough = false;
	struct kbase_csf_cmd_stream_group_info *const groups =
		kbdev->csf.global_iface.groups;
	const u32 group_num = kbdev->csf.global_iface.group_num;
	u32 i;

	for (i = 0; (i < group_num) && !has_enough; i++) {
		if (groups[i].stream_num >= cs_min)
			has_enough = true;
	}

	return has_enough;
}

/**
 * create_normal_suspend_buffer() - Create normal-mode suspend buffer per
 *					queue group
 *
 * @kctx:	Pointer to kbase context where the queue group is created at
 * @s_buf:	Pointer to suspend buffer that is attached to queue group
 *
 * Return: 0 if phy-pages for the suspend buffer is successfully allocated.
 *	   Otherwise -ENOMEM or error code.
 */
static int create_normal_suspend_buffer(struct kbase_context *const kctx,
		struct kbase_normal_suspend_buffer *s_buf)
{
	const size_t nr_pages =
		PFN_UP(kctx->kbdev->csf.global_iface.groups[0].suspend_size);
	int err;

	lockdep_assert_held(&kctx->csf.lock);

	/* The suspend buffer's mapping address is valid only when the CSG is to
	 * run on slot, initializing it 0, signalling the buffer is not mapped.
	 */
	s_buf->gpu_va = 0;

	s_buf->phy = kcalloc(nr_pages, sizeof(*s_buf->phy), GFP_KERNEL);

	if (!s_buf->phy)
		return -ENOMEM;

	/* Get physical page for a normal suspend buffer */
	err = kbase_mem_pool_alloc_pages(&kctx->mem_pools.small[KBASE_MEM_GROUP_CSF_FW], nr_pages,
					 &s_buf->phy[0], false, kctx->task);

	if (err < 0) {
		kfree(s_buf->phy);
		return err;
	}

	kbase_process_page_usage_inc(kctx, nr_pages);
	return 0;
}

static void timer_event_worker(struct work_struct *data);
static void protm_event_worker(struct work_struct *data);
static void term_normal_suspend_buffer(struct kbase_context *const kctx,
		struct kbase_normal_suspend_buffer *s_buf);

/**
 * create_suspend_buffers - Setup normal and protected mode
 *				suspend buffers.
 *
 * @kctx:	Address of the kbase context within which the queue group
 *		is to be created.
 * @group:	Pointer to GPU command queue group data.
 *
 * Return: 0 if suspend buffers are successfully allocated. Otherwise -ENOMEM.
 */
static int create_suspend_buffers(struct kbase_context *const kctx,
		struct kbase_queue_group * const group)
{
	if (create_normal_suspend_buffer(kctx, &group->normal_suspend_buf)) {
		dev_err(kctx->kbdev->dev, "Failed to create normal suspend buffer\n");
		return -ENOMEM;
	}

	/* Protected suspend buffer, runtime binding so just initialize it */
	group->protected_suspend_buf.gpu_va = 0;
	group->protected_suspend_buf.pma = NULL;
	group->protected_suspend_buf.alloc_retries = 0;

	return 0;
}

/**
 * generate_group_uid() - Makes an ID unique to all kernel base devices
 *                        and contexts, for a queue group and CSG.
 *
 * Return:      A unique ID in the form of an unsigned 32-bit integer
 */
static u32 generate_group_uid(void)
{
	static atomic_t global_csg_uid = ATOMIC_INIT(0);

	return (u32)atomic_inc_return(&global_csg_uid);
}

/**
 * create_queue_group() - Create a queue group
 *
 * @kctx:	Address of the kbase context within which the queue group
 *		is to be created.
 * @create:	Address of a structure which contains details of the
 *		queue group which is to be created.
 *
 * Return: a queue group handle on success, or a negative error code on failure.
 */
static int create_queue_group(struct kbase_context *const kctx,
	union kbase_ioctl_cs_queue_group_create *const create)
{
	int group_handle = find_free_group_handle(kctx);

	if (group_handle < 0) {
		dev_dbg(kctx->kbdev->dev,
			"All queue group handles are already in use");
	} else {
		struct kbase_queue_group * const group =
			kmalloc(sizeof(struct kbase_queue_group),
					GFP_KERNEL);

		lockdep_assert_held(&kctx->csf.lock);

		if (!group) {
			dev_err(kctx->kbdev->dev, "Failed to allocate a queue\n");
			group_handle = -ENOMEM;
		} else {
			int err = 0;

			group->kctx = kctx;
			group->handle = group_handle;
			group->csg_nr = KBASEP_CSG_NR_INVALID;

			group->tiler_mask = create->in.tiler_mask;
			group->fragment_mask = create->in.fragment_mask;
			group->compute_mask = create->in.compute_mask;

			group->tiler_max = create->in.tiler_max;
			group->fragment_max = create->in.fragment_max;
			group->compute_max = create->in.compute_max;
			group->csi_handlers = create->in.csi_handlers;
			group->priority = kbase_csf_priority_queue_group_priority_to_relative(
				kbase_csf_priority_check(kctx->kbdev, create->in.priority));
			group->doorbell_nr = KBASEP_USER_DB_NR_INVALID;
			group->faulted = false;
			group->cs_unrecoverable = false;
			group->reevaluate_idle_status = false;

			group->csg_reg = NULL;
			group->csg_reg_bind_retries = 0;

			group->dvs_buf = create->in.dvs_buf;

#if IS_ENABLED(CONFIG_DEBUG_FS)
			group->deschedule_deferred_cnt = 0;
#endif

			group->group_uid = generate_group_uid();
			create->out.group_uid = group->group_uid;

			INIT_LIST_HEAD(&group->link);
			INIT_LIST_HEAD(&group->link_to_schedule);
			INIT_LIST_HEAD(&group->error_fatal.link);
			INIT_LIST_HEAD(&group->error_timeout.link);
			INIT_LIST_HEAD(&group->error_tiler_oom.link);
			INIT_WORK(&group->timer_event_work, timer_event_worker);
			INIT_WORK(&group->protm_event_work, protm_event_worker);
			bitmap_zero(group->protm_pending_bitmap,
					MAX_SUPPORTED_STREAMS_PER_GROUP);

			group->run_state = KBASE_CSF_GROUP_INACTIVE;
			KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_INACTIVE, group,
						group->run_state);

			err = create_suspend_buffers(kctx, group);

			if (err < 0) {
				kfree(group);
				group_handle = err;
			} else {
				int j;

				kctx->csf.queue_groups[group_handle] = group;
				for (j = 0; j < MAX_SUPPORTED_STREAMS_PER_GROUP;
						j++)
					group->bound_queues[j] = NULL;
			}
		}
	}

	return group_handle;
}

static bool dvs_supported(u32 csf_version)
{
	if (GLB_VERSION_MAJOR_GET(csf_version) < 3)
		return false;

	if (GLB_VERSION_MAJOR_GET(csf_version) == 3)
		if (GLB_VERSION_MINOR_GET(csf_version) < 2)
			return false;

	return true;
}

int kbase_csf_queue_group_create(struct kbase_context *const kctx,
			union kbase_ioctl_cs_queue_group_create *const create)
{
	int err = 0;
	const u32 tiler_count = hweight64(create->in.tiler_mask);
	const u32 fragment_count = hweight64(create->in.fragment_mask);
	const u32 compute_count = hweight64(create->in.compute_mask);
	size_t i;

	for (i = 0; i < sizeof(create->in.padding); i++) {
		if (create->in.padding[i] != 0) {
			dev_warn(kctx->kbdev->dev, "Invalid padding not 0 in queue group create\n");
			return -EINVAL;
		}
	}

	mutex_lock(&kctx->csf.lock);

	if ((create->in.tiler_max > tiler_count) ||
	    (create->in.fragment_max > fragment_count) ||
	    (create->in.compute_max > compute_count)) {
		dev_dbg(kctx->kbdev->dev,
			"Invalid maximum number of endpoints for a queue group");
		err = -EINVAL;
	} else if (create->in.priority >= BASE_QUEUE_GROUP_PRIORITY_COUNT) {
		dev_dbg(kctx->kbdev->dev, "Invalid queue group priority %u",
			(unsigned int)create->in.priority);
		err = -EINVAL;
	} else if (!iface_has_enough_streams(kctx->kbdev, create->in.cs_min)) {
		dev_dbg(kctx->kbdev->dev,
			"No CSG has at least %d CSs",
			create->in.cs_min);
		err = -EINVAL;
	} else if (create->in.csi_handlers & ~BASE_CSF_EXCEPTION_HANDLER_FLAGS_MASK) {
		dev_warn(kctx->kbdev->dev, "Unknown exception handler flags set: %u",
			 create->in.csi_handlers & ~BASE_CSF_EXCEPTION_HANDLER_FLAGS_MASK);
		err = -EINVAL;
	} else if (!dvs_supported(kctx->kbdev->csf.global_iface.version) &&
		   create->in.dvs_buf) {
		dev_warn(
			kctx->kbdev->dev,
			"GPU does not support DVS but userspace is trying to use it");
		err = -EINVAL;
	} else if (dvs_supported(kctx->kbdev->csf.global_iface.version) &&
		   !CSG_DVS_BUF_BUFFER_POINTER_GET(create->in.dvs_buf) &&
		   CSG_DVS_BUF_BUFFER_SIZE_GET(create->in.dvs_buf)) {
		dev_warn(kctx->kbdev->dev,
			 "DVS buffer pointer is null but size is not 0");
		err = -EINVAL;
	} else {
		/* For the CSG which satisfies the condition for having
		 * the needed number of CSs, check whether it also conforms
		 * with the requirements for at least one of its CSs having
		 * the iterator of the needed type
		 * (note: for CSF v1.0 all CSs in a CSG will have access to
		 * the same iterators)
		 */
		const int group_handle = create_queue_group(kctx, create);

		if (group_handle >= 0)
			create->out.group_handle = group_handle;
		else
			err = group_handle;
	}

	mutex_unlock(&kctx->csf.lock);

	return err;
}

/**
 * term_normal_suspend_buffer() - Free normal-mode suspend buffer of queue group
 *
 * @kctx:	Pointer to kbase context where queue group belongs to
 * @s_buf:	Pointer to queue group suspend buffer to be freed
 */
static void term_normal_suspend_buffer(struct kbase_context *const kctx,
				       struct kbase_normal_suspend_buffer *s_buf)
{
	const size_t nr_pages = PFN_UP(kctx->kbdev->csf.global_iface.groups[0].suspend_size);

	lockdep_assert_held(&kctx->csf.lock);

	/* The group should not have a bind remaining on any suspend buf region */
	WARN_ONCE(s_buf->gpu_va, "Suspend buffer address should be 0 at termination");

	kbase_mem_pool_free_pages(&kctx->mem_pools.small[KBASE_MEM_GROUP_CSF_FW], nr_pages,
				  &s_buf->phy[0], false, false);
	kbase_process_page_usage_dec(kctx, nr_pages);

	kfree(s_buf->phy);
	s_buf->phy = NULL;
}

/**
 * term_protected_suspend_buffer() - Free protected-mode suspend buffer of
 *					queue group
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @sbuf: Pointer to queue group suspend buffer to be freed
 */
static void term_protected_suspend_buffer(struct kbase_device *const kbdev,
					  struct kbase_protected_suspend_buffer *sbuf)
{
	WARN_ONCE(sbuf->gpu_va, "Suspend buf should have been unmapped inside scheduler!");
	if (sbuf->pma) {
		const size_t nr_pages = PFN_UP(kbdev->csf.global_iface.groups[0].suspend_size);
		kbase_csf_protected_memory_free(kbdev, sbuf->pma, nr_pages, true);
		sbuf->pma = NULL;
	}
}

void kbase_csf_term_descheduled_queue_group(struct kbase_queue_group *group)
{
	struct kbase_context *kctx = group->kctx;

	/* Currently each group supports the same number of CS */
	u32 max_streams =
		kctx->kbdev->csf.global_iface.groups[0].stream_num;
	u32 i;

	lockdep_assert_held(&kctx->csf.lock);

	WARN_ON(group->run_state != KBASE_CSF_GROUP_INACTIVE &&
		group->run_state != KBASE_CSF_GROUP_FAULT_EVICTED);

	for (i = 0; i < max_streams; i++) {
		struct kbase_queue *queue =
				group->bound_queues[i];

		/* The group is already being evicted from the scheduler */
		if (queue)
			unbind_stopped_queue(kctx, queue);
	}

	term_normal_suspend_buffer(kctx, &group->normal_suspend_buf);
	if (kctx->kbdev->csf.pma_dev)
		term_protected_suspend_buffer(kctx->kbdev,
			&group->protected_suspend_buf);

	group->run_state = KBASE_CSF_GROUP_TERMINATED;
	KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_TERMINATED, group, group->run_state);
}

/**
 * term_queue_group - Terminate a GPU command queue group.
 *
 * @group: Pointer to GPU command queue group data.
 *
 * Terminates a GPU command queue group. From the userspace perspective the
 * group will still exist but it can't bind new queues to it. Userspace can
 * still add work in queues bound to the group but it won't be executed. (This
 * is because the IO mapping created upon binding such queues is still intact.)
 */
static void term_queue_group(struct kbase_queue_group *group)
{
	struct kbase_context *kctx = group->kctx;

	kbase_reset_gpu_assert_failed_or_prevented(kctx->kbdev);
	lockdep_assert_held(&kctx->csf.lock);

	/* Stop the group and evict it from the scheduler */
	kbase_csf_scheduler_group_deschedule(group);

	if (group->run_state == KBASE_CSF_GROUP_TERMINATED)
		return;

	dev_dbg(kctx->kbdev->dev, "group %d terminating", group->handle);

	kbase_csf_term_descheduled_queue_group(group);
}

/**
 * wait_group_deferred_deschedule_completion - Wait for refcount of the group to
 *         become 0 that was taken when the group deschedule had to be deferred.
 *
 * @group: Pointer to GPU command queue group that is being deleted.
 *
 * This function is called when Userspace deletes the group and after the group
 * has been descheduled. The function synchronizes with the other threads that were
 * also trying to deschedule the group whilst the dumping was going on for a fault.
 * Please refer the documentation of wait_for_dump_complete_on_group_deschedule()
 * for more details.
 */
static void wait_group_deferred_deschedule_completion(struct kbase_queue_group *group)
{
#if IS_ENABLED(CONFIG_DEBUG_FS)
	struct kbase_context *kctx = group->kctx;

	lockdep_assert_held(&kctx->csf.lock);

	if (likely(!group->deschedule_deferred_cnt))
		return;

	mutex_unlock(&kctx->csf.lock);
	wait_event(kctx->kbdev->csf.event_wait, !group->deschedule_deferred_cnt);
	mutex_lock(&kctx->csf.lock);
#endif
}

static void cancel_queue_group_events(struct kbase_queue_group *group)
{
	cancel_work_sync(&group->timer_event_work);
	cancel_work_sync(&group->protm_event_work);
}

static void remove_pending_group_fatal_error(struct kbase_queue_group *group)
{
	struct kbase_context *kctx = group->kctx;

	dev_dbg(kctx->kbdev->dev,
		"Remove any pending group fatal error from context %pK\n",
		(void *)group->kctx);

	kbase_csf_event_remove_error(kctx, &group->error_tiler_oom);
	kbase_csf_event_remove_error(kctx, &group->error_timeout);
	kbase_csf_event_remove_error(kctx, &group->error_fatal);
}

void kbase_csf_queue_group_terminate(struct kbase_context *kctx,
				     u8 group_handle)
{
	struct kbase_queue_group *group;
	int err;
	bool reset_prevented = false;
	struct kbase_device *const kbdev = kctx->kbdev;

	err = kbase_reset_gpu_prevent_and_wait(kbdev);
	if (err)
		dev_warn(
			kbdev->dev,
			"Unsuccessful GPU reset detected when terminating group %d, attempting to terminate regardless",
			group_handle);
	else
		reset_prevented = true;

	mutex_lock(&kctx->csf.lock);

	group = find_queue_group(kctx, group_handle);

	if (group) {
		kctx->csf.queue_groups[group_handle] = NULL;
		/* Stop the running of the given group */
		term_queue_group(group);
		mutex_unlock(&kctx->csf.lock);

		if (reset_prevented) {
			/* Allow GPU reset before cancelling the group specific
			 * work item to avoid potential deadlock.
			 * Reset prevention isn't needed after group termination.
			 */
			kbase_reset_gpu_allow(kbdev);
			reset_prevented = false;
		}

		/* Cancel any pending event callbacks. If one is in progress
		 * then this thread waits synchronously for it to complete (which
		 * is why we must unlock the context first). We already ensured
		 * that no more callbacks can be enqueued by terminating the group.
		 */
		cancel_queue_group_events(group);

		mutex_lock(&kctx->csf.lock);

		/* Clean up after the termination */
		remove_pending_group_fatal_error(group);

		wait_group_deferred_deschedule_completion(group);
	}

	mutex_unlock(&kctx->csf.lock);
	if (reset_prevented)
		kbase_reset_gpu_allow(kbdev);

	kfree(group);
}
KBASE_EXPORT_TEST_API(kbase_csf_queue_group_terminate);

#if IS_ENABLED(CONFIG_MALI_VECTOR_DUMP) || MALI_UNIT_TEST
int kbase_csf_queue_group_suspend(struct kbase_context *kctx,
				  struct kbase_suspend_copy_buffer *sus_buf,
				  u8 group_handle)
{
	struct kbase_device *const kbdev = kctx->kbdev;
	int err;
	struct kbase_queue_group *group;

	err = kbase_reset_gpu_prevent_and_wait(kbdev);
	if (err) {
		dev_warn(
			kbdev->dev,
			"Unsuccessful GPU reset detected when suspending group %d",
			group_handle);
		return err;
	}
	mutex_lock(&kctx->csf.lock);

	group = find_queue_group(kctx, group_handle);
	if (group)
		err = kbase_csf_scheduler_group_copy_suspend_buf(group,
								 sus_buf);
	else
		err = -EINVAL;

	mutex_unlock(&kctx->csf.lock);
	kbase_reset_gpu_allow(kbdev);

	return err;
}
#endif

void kbase_csf_add_group_fatal_error(
	struct kbase_queue_group *const group,
	struct base_gpu_queue_group_error const *const err_payload)
{
	struct base_csf_notification error;

	if (WARN_ON(!group))
		return;

	if (WARN_ON(!err_payload))
		return;

	error = (struct base_csf_notification) {
		.type = BASE_CSF_NOTIFICATION_GPU_QUEUE_GROUP_ERROR,
		.payload = {
			.csg_error = {
				.handle = group->handle,
				.error = *err_payload
			}
		}
	};

	kbase_csf_event_add_error(group->kctx, &group->error_fatal, &error);
}

void kbase_csf_active_queue_groups_reset(struct kbase_device *kbdev,
					 struct kbase_context *kctx)
{
	struct list_head evicted_groups;
	struct kbase_queue_group *group;
	int i;

	INIT_LIST_HEAD(&evicted_groups);

	mutex_lock(&kctx->csf.lock);

	kbase_csf_scheduler_evict_ctx_slots(kbdev, kctx, &evicted_groups);
	while (!list_empty(&evicted_groups)) {
		group = list_first_entry(&evicted_groups,
				struct kbase_queue_group, link);

		dev_dbg(kbdev->dev, "Context %d_%d active group %d terminated",
			    kctx->tgid, kctx->id, group->handle);
		kbase_csf_term_descheduled_queue_group(group);
		list_del_init(&group->link);
	}

	/* Acting on the queue groups that are pending to be terminated. */
	for (i = 0; i < MAX_QUEUE_GROUP_NUM; i++) {
		group = kctx->csf.queue_groups[i];
		if (group &&
		    group->run_state == KBASE_CSF_GROUP_FAULT_EVICTED)
			kbase_csf_term_descheduled_queue_group(group);
	}

	mutex_unlock(&kctx->csf.lock);
}

int kbase_csf_ctx_init(struct kbase_context *kctx)
{
	int err = -ENOMEM;

	INIT_LIST_HEAD(&kctx->csf.queue_list);
	INIT_LIST_HEAD(&kctx->csf.link);

	kbase_csf_event_init(kctx);

	/* Mark all the cookies as 'free' */
	bitmap_fill(kctx->csf.cookies, KBASE_CSF_NUM_USER_IO_PAGES_HANDLE);

	kctx->csf.wq = alloc_workqueue("mali_kbase_csf_wq",
					WQ_UNBOUND, 1);

	if (likely(kctx->csf.wq)) {
		err = kbase_csf_scheduler_context_init(kctx);

		if (likely(!err)) {
			err = kbase_csf_kcpu_queue_context_init(kctx);

			if (likely(!err)) {
				err = kbase_csf_tiler_heap_context_init(kctx);

				if (likely(!err)) {
					mutex_init(&kctx->csf.lock);
					INIT_WORK(&kctx->csf.pending_submission_work,
						  pending_submission_worker);

					err = kbasep_ctx_user_reg_page_mapping_init(kctx);

					if (unlikely(err))
						kbase_csf_tiler_heap_context_term(kctx);
				}

				if (unlikely(err))
					kbase_csf_kcpu_queue_context_term(kctx);
			}

			if (unlikely(err))
				kbase_csf_scheduler_context_term(kctx);
		}

		if (unlikely(err))
			destroy_workqueue(kctx->csf.wq);
	}

	return err;
}

void kbase_csf_ctx_handle_fault(struct kbase_context *kctx,
		struct kbase_fault *fault)
{
	int gr;
	bool reported = false;
	struct base_gpu_queue_group_error err_payload;
	int err;
	struct kbase_device *kbdev;

	if (WARN_ON(!kctx))
		return;

	if (WARN_ON(!fault))
		return;

	kbdev = kctx->kbdev;
	err = kbase_reset_gpu_try_prevent(kbdev);
	/* Regardless of whether reset failed or is currently happening, exit
	 * early
	 */
	if (err)
		return;

	err_payload = (struct base_gpu_queue_group_error) {
		.error_type = BASE_GPU_QUEUE_GROUP_ERROR_FATAL,
		.payload = {
			.fatal_group = {
				.sideband = fault->addr,
				.status = fault->status,
			}
		}
	};

	mutex_lock(&kctx->csf.lock);

	for (gr = 0; gr < MAX_QUEUE_GROUP_NUM; gr++) {
		struct kbase_queue_group *const group =
			kctx->csf.queue_groups[gr];

		if (group && group->run_state != KBASE_CSF_GROUP_TERMINATED) {
			term_queue_group(group);
			kbase_csf_add_group_fatal_error(group, &err_payload);
			reported = true;
		}
	}

	mutex_unlock(&kctx->csf.lock);

	if (reported)
		kbase_event_wakeup(kctx);

	kbase_reset_gpu_allow(kbdev);
}

void kbase_csf_ctx_term(struct kbase_context *kctx)
{
	struct kbase_device *kbdev = kctx->kbdev;
	struct kbase_as *as = NULL;
	unsigned long flags;
	u32 i;
	int err;
	bool reset_prevented = false;

	/* As the kbase context is terminating, its debugfs sub-directory would
	 * have been removed already and so would be the debugfs file created
	 * for queue groups & kcpu queues, hence no need to explicitly remove
	 * those debugfs files.
	 */

	/* Wait for a GPU reset if it is happening, prevent it if not happening */
	err = kbase_reset_gpu_prevent_and_wait(kbdev);
	if (err)
		dev_warn(
			kbdev->dev,
			"Unsuccessful GPU reset detected when terminating csf context (%d_%d), attempting to terminate regardless",
			kctx->tgid, kctx->id);
	else
		reset_prevented = true;

	mutex_lock(&kctx->csf.lock);

	/* Iterate through the queue groups that were not terminated by
	 * userspace and issue the term request to firmware for them.
	 */
	for (i = 0; i < MAX_QUEUE_GROUP_NUM; i++) {
		struct kbase_queue_group *group = kctx->csf.queue_groups[i];

		if (group) {
			remove_pending_group_fatal_error(group);
			term_queue_group(group);
		}
	}
	mutex_unlock(&kctx->csf.lock);

	if (reset_prevented)
		kbase_reset_gpu_allow(kbdev);

	cancel_work_sync(&kctx->csf.pending_submission_work);

	/* Now that all queue groups have been terminated, there can be no
	 * more OoM or timer event interrupts but there can be inflight work
	 * items. Destroying the wq will implicitly flush those work items.
	 */
	destroy_workqueue(kctx->csf.wq);

	/* Wait for the firmware error work item to also finish as it could
	 * be affecting this outgoing context also.
	 */
	flush_work(&kctx->kbdev->csf.fw_error_work);

	/* A work item to handle page_fault/bus_fault/gpu_fault could be
	 * pending for the outgoing context. Flush the workqueue that will
	 * execute that work item.
	 */
	spin_lock_irqsave(&kctx->kbdev->hwaccess_lock, flags);
	if (kctx->as_nr != KBASEP_AS_NR_INVALID)
		as = &kctx->kbdev->as[kctx->as_nr];
	spin_unlock_irqrestore(&kctx->kbdev->hwaccess_lock, flags);
	if (as)
		flush_workqueue(as->pf_wq);

	mutex_lock(&kctx->csf.lock);

	for (i = 0; i < MAX_QUEUE_GROUP_NUM; i++) {
		kfree(kctx->csf.queue_groups[i]);
		kctx->csf.queue_groups[i] = NULL;
	}

	/* Iterate through the queues that were not terminated by
	 * userspace and do the required cleanup for them.
	 */
	while (!list_empty(&kctx->csf.queue_list)) {
		struct kbase_queue *queue;

		queue = list_first_entry(&kctx->csf.queue_list,
						struct kbase_queue, link);

		/* The reference held when the IO mapping was created on bind
		 * would have been dropped otherwise the termination of Kbase
		 * context itself wouldn't have kicked-in. So there shall be
		 * only one reference left that was taken when queue was
		 * registered.
		 */
		WARN_ON(kbase_refcount_read(&queue->refcount) != 1);
		list_del_init(&queue->link);
		release_queue(queue);
	}

	mutex_unlock(&kctx->csf.lock);

	kbasep_ctx_user_reg_page_mapping_term(kctx);
	kbase_csf_tiler_heap_context_term(kctx);
	kbase_csf_kcpu_queue_context_term(kctx);
	kbase_csf_scheduler_context_term(kctx);
	kbase_csf_event_term(kctx);

	mutex_destroy(&kctx->csf.lock);
}

/**
 * handle_oom_event - Handle the OoM event generated by the firmware for the
 *                    CSI.
 *
 * @group:  Pointer to the CSG group the oom-event belongs to.
 * @stream: Pointer to the structure containing info provided by the firmware
 *          about the CSI.
 *
 * This function will handle the OoM event request from the firmware for the
 * CS. It will retrieve the address of heap context and heap's
 * statistics (like number of render passes in-flight) from the CS's kernel
 * output page and pass them to the tiler heap function to allocate a
 * new chunk.
 * It will also update the CS's kernel input page with the address
 * of a new chunk that was allocated.
 *
 * Return: 0 if successfully handled the request, otherwise a negative error
 *         code on failure.
 */
static int handle_oom_event(struct kbase_queue_group *const group,
			    struct kbase_csf_cmd_stream_info const *const stream)
{
	struct kbase_context *const kctx = group->kctx;
	u64 gpu_heap_va =
		kbase_csf_firmware_cs_output(stream, CS_HEAP_ADDRESS_LO) |
		((u64)kbase_csf_firmware_cs_output(stream, CS_HEAP_ADDRESS_HI) << 32);
	const u32 vt_start =
		kbase_csf_firmware_cs_output(stream, CS_HEAP_VT_START);
	const u32 vt_end =
		kbase_csf_firmware_cs_output(stream, CS_HEAP_VT_END);
	const u32 frag_end =
		kbase_csf_firmware_cs_output(stream, CS_HEAP_FRAG_END);
	u32 renderpasses_in_flight;
	u32 pending_frag_count;
	u64 new_chunk_ptr;
	int err;
	bool frag_end_err = false;

	if ((frag_end > vt_end) || (vt_end >= vt_start)) {
		frag_end_err = true;
		dev_dbg(kctx->kbdev->dev, "Invalid Heap statistics provided by firmware: vt_start %d, vt_end %d, frag_end %d\n",
			 vt_start, vt_end, frag_end);
	}
	if (frag_end_err) {
		renderpasses_in_flight = 1;
		pending_frag_count = 1;
	} else {
		renderpasses_in_flight = vt_start - frag_end;
		pending_frag_count = vt_end - frag_end;
	}

	err = kbase_csf_tiler_heap_alloc_new_chunk(kctx,
		gpu_heap_va, renderpasses_in_flight, pending_frag_count, &new_chunk_ptr);

	if ((group->csi_handlers & BASE_CSF_TILER_OOM_EXCEPTION_FLAG) &&
	    (pending_frag_count == 0) && (err == -ENOMEM || err == -EBUSY)) {
		/* The group allows incremental rendering, trigger it */
		new_chunk_ptr = 0;
		dev_dbg(kctx->kbdev->dev, "Group-%d (slot-%d) enter incremental render\n",
			group->handle, group->csg_nr);
	} else if (err == -EBUSY) {
		/* Acknowledge with a NULL chunk (firmware will then wait for
		 * the fragment jobs to complete and release chunks)
		 */
		new_chunk_ptr = 0;
	} else if (err)
		return err;

	kbase_csf_firmware_cs_input(stream, CS_TILER_HEAP_START_LO,
				new_chunk_ptr & 0xFFFFFFFF);
	kbase_csf_firmware_cs_input(stream, CS_TILER_HEAP_START_HI,
				new_chunk_ptr >> 32);

	kbase_csf_firmware_cs_input(stream, CS_TILER_HEAP_END_LO,
				new_chunk_ptr & 0xFFFFFFFF);
	kbase_csf_firmware_cs_input(stream, CS_TILER_HEAP_END_HI,
				new_chunk_ptr >> 32);

	return 0;
}

/**
 * report_tiler_oom_error - Report a CSG error due to a tiler heap OOM event
 *
 * @group: Pointer to the GPU command queue group that encountered the error
 */
static void report_tiler_oom_error(struct kbase_queue_group *group)
{
	struct base_csf_notification const
		error = { .type = BASE_CSF_NOTIFICATION_GPU_QUEUE_GROUP_ERROR,
			  .payload = {
				  .csg_error = {
					  .handle = group->handle,
					  .error = {
						  .error_type =
							  BASE_GPU_QUEUE_GROUP_ERROR_TILER_HEAP_OOM,
					  } } } };

	kbase_csf_event_add_error(group->kctx,
				  &group->error_tiler_oom,
				  &error);
	kbase_event_wakeup(group->kctx);
}

static void flush_gpu_cache_on_fatal_error(struct kbase_device *kbdev)
{
	int err;
	const unsigned int cache_flush_wait_timeout_ms = 2000;

	kbase_pm_lock(kbdev);
	/* With the advent of partial cache flush, dirty cache lines could
	 * be left in the GPU L2 caches by terminating the queue group here
	 * without waiting for proper cache maintenance. A full cache flush
	 * here will prevent these dirty cache lines from being arbitrarily
	 * evicted later and possible causing memory corruption.
	 */
	if (kbdev->pm.backend.gpu_powered) {
		kbase_gpu_start_cache_clean(kbdev, GPU_COMMAND_CACHE_CLN_INV_L2_LSC);
		err = kbase_gpu_wait_cache_clean_timeout(kbdev, cache_flush_wait_timeout_ms);

		if (err) {
			dev_warn(
				kbdev->dev,
				"[%llu] Timeout waiting for cache clean to complete after fatal error",
				kbase_backend_get_cycle_cnt(kbdev));

			if (kbase_prepare_to_reset_gpu(kbdev, RESET_FLAGS_HWC_UNRECOVERABLE_ERROR))
				kbase_reset_gpu(kbdev);
		}
	}

	kbase_pm_unlock(kbdev);
}

/**
 * kbase_queue_oom_event - Handle tiler out-of-memory for a GPU command queue.
 *
 * @queue: Pointer to queue for which out-of-memory event was received.
 *
 * Called with the CSF locked for the affected GPU virtual address space.
 * Do not call in interrupt context.
 *
 * Handles tiler out-of-memory for a GPU command queue and then clears the
 * notification to allow the firmware to report out-of-memory again in future.
 * If the out-of-memory condition was successfully handled then this function
 * rings the relevant doorbell to notify the firmware; otherwise, it terminates
 * the GPU command queue group to which the queue is bound and notify a waiting
 * user space client of the failure.
 */
static void kbase_queue_oom_event(struct kbase_queue *const queue)
{
	struct kbase_context *const kctx = queue->kctx;
	struct kbase_device *const kbdev = kctx->kbdev;
	struct kbase_queue_group *group;
	int slot_num, err;
	struct kbase_csf_cmd_stream_group_info const *ginfo;
	struct kbase_csf_cmd_stream_info const *stream;
	int csi_index = queue->csi_index;
	u32 cs_oom_ack, cs_oom_req;
	unsigned long flags;

	lockdep_assert_held(&kctx->csf.lock);

	group = get_bound_queue_group(queue);
	if (!group) {
		dev_warn(kctx->kbdev->dev, "queue not bound\n");
		return;
	}

	kbase_csf_scheduler_lock(kbdev);

	slot_num = kbase_csf_scheduler_group_get_slot(group);

	/* The group could have gone off slot before this work item got
	 * a chance to execute.
	 */
	if (slot_num < 0)
		goto unlock;

	/* If the bound group is on slot yet the kctx is marked with disabled
	 * on address-space fault, the group is pending to be killed. So skip
	 * the inflight oom operation.
	 */
	if (kbase_ctx_flag(kctx, KCTX_AS_DISABLED_ON_FAULT))
		goto unlock;

	ginfo = &kbdev->csf.global_iface.groups[slot_num];
	stream = &ginfo->streams[csi_index];
	cs_oom_ack = kbase_csf_firmware_cs_output(stream, CS_ACK) &
		     CS_ACK_TILER_OOM_MASK;
	cs_oom_req = kbase_csf_firmware_cs_input_read(stream, CS_REQ) &
		     CS_REQ_TILER_OOM_MASK;

	/* The group could have already undergone suspend-resume cycle before
	 * this work item got a chance to execute. On CSG resume the CS_ACK
	 * register is set by firmware to reflect the CS_REQ register, which
	 * implies that all events signaled before suspension are implicitly
	 * acknowledged.
	 * A new OoM event is expected to be generated after resume.
	 */
	if (cs_oom_ack == cs_oom_req)
		goto unlock;

	err = handle_oom_event(group, stream);

	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	kbase_csf_firmware_cs_input_mask(stream, CS_REQ, cs_oom_ack,
					 CS_REQ_TILER_OOM_MASK);
	kbase_csf_ring_cs_kernel_doorbell(kbdev, csi_index, slot_num, true);
	kbase_csf_scheduler_spin_unlock(kbdev, flags);

	if (unlikely(err)) {
		dev_warn(
			kbdev->dev,
			"Queue group to be terminated, couldn't handle the OoM event\n");
		kbase_debug_csf_fault_notify(kbdev, kctx, DF_TILER_OOM);
		kbase_csf_scheduler_unlock(kbdev);
		term_queue_group(group);
		flush_gpu_cache_on_fatal_error(kbdev);
		report_tiler_oom_error(group);
		return;
	}
unlock:
	kbase_csf_scheduler_unlock(kbdev);
}

/**
 * oom_event_worker - Tiler out-of-memory handler called from a workqueue.
 *
 * @data: Pointer to a work_struct embedded in GPU command queue data.
 *
 * Handles a tiler out-of-memory condition for a GPU command queue and then
 * releases a reference that was added to prevent the queue being destroyed
 * while this work item was pending on a workqueue.
 */
static void oom_event_worker(struct work_struct *data)
{
	struct kbase_queue *queue =
		container_of(data, struct kbase_queue, oom_event_work);
	struct kbase_context *kctx = queue->kctx;
	struct kbase_device *const kbdev = kctx->kbdev;

	int err = kbase_reset_gpu_try_prevent(kbdev);

	/* Regardless of whether reset failed or is currently happening, exit
	 * early
	 */
	if (err)
		return;

	mutex_lock(&kctx->csf.lock);

	kbase_queue_oom_event(queue);
	release_queue(queue);

	mutex_unlock(&kctx->csf.lock);
	kbase_reset_gpu_allow(kbdev);
}

/**
 * report_group_timeout_error - Report the timeout error for the group to userspace.
 *
 * @group: Pointer to the group for which timeout error occurred
 */
static void report_group_timeout_error(struct kbase_queue_group *const group)
{
	struct base_csf_notification const
		error = { .type = BASE_CSF_NOTIFICATION_GPU_QUEUE_GROUP_ERROR,
			  .payload = {
				  .csg_error = {
					  .handle = group->handle,
					  .error = {
						  .error_type =
							  BASE_GPU_QUEUE_GROUP_ERROR_TIMEOUT,
					  } } } };

	dev_warn(group->kctx->kbdev->dev,
		 "Notify the event notification thread, forward progress timeout (%llu cycles)\n",
		 kbase_csf_timeout_get(group->kctx->kbdev));

	kbase_csf_event_add_error(group->kctx, &group->error_timeout, &error);
	kbase_event_wakeup(group->kctx);
}

/**
 * timer_event_worker - Handle the progress timeout error for the group
 *
 * @data: Pointer to a work_struct embedded in GPU command queue group data.
 *
 * Terminate the CSG and report the error to userspace
 */
static void timer_event_worker(struct work_struct *data)
{
	struct kbase_queue_group *const group =
		container_of(data, struct kbase_queue_group, timer_event_work);
	struct kbase_context *const kctx = group->kctx;
	struct kbase_device *const kbdev = kctx->kbdev;
	bool reset_prevented = false;
	int err = kbase_reset_gpu_prevent_and_wait(kbdev);

	if (err)
		dev_warn(
			kbdev->dev,
			"Unsuccessful GPU reset detected when terminating group %d on progress timeout, attempting to terminate regardless",
			group->handle);
	else
		reset_prevented = true;

	mutex_lock(&kctx->csf.lock);

	term_queue_group(group);
	flush_gpu_cache_on_fatal_error(kbdev);
	report_group_timeout_error(group);

	mutex_unlock(&kctx->csf.lock);
	if (reset_prevented)
		kbase_reset_gpu_allow(kbdev);
}

/**
 * handle_progress_timer_event - Progress timer timeout event handler.
 *
 * @group: Pointer to GPU queue group for which the timeout event is received.
 *
 * Notify a waiting user space client of the timeout.
 * Enqueue a work item to terminate the group and notify the event notification
 * thread of progress timeout fault for the GPU command queue group.
 */
static void handle_progress_timer_event(struct kbase_queue_group *const group)
{
	kbase_debug_csf_fault_notify(group->kctx->kbdev, group->kctx,
		DF_PROGRESS_TIMER_TIMEOUT);

	queue_work(group->kctx->csf.wq, &group->timer_event_work);
}

/**
 * alloc_grp_protected_suspend_buffer_pages() -  Allocate physical pages from the protected
 *                                               memory for the protected mode suspend buffer.
 * @group: Pointer to the GPU queue group.
 *
 * Return: 0 if suspend buffer allocation is successful or if its already allocated, otherwise
 * negative error value.
 */
static int alloc_grp_protected_suspend_buffer_pages(struct kbase_queue_group *const group)
{
	struct kbase_device *const kbdev = group->kctx->kbdev;
	struct kbase_context *kctx = group->kctx;
	struct tagged_addr *phys = NULL;
	struct kbase_protected_suspend_buffer *sbuf = &group->protected_suspend_buf;
	size_t nr_pages;
	int err = 0;

	if (likely(sbuf->pma))
		return 0;

	nr_pages = PFN_UP(kbdev->csf.global_iface.groups[0].suspend_size);
	phys = kcalloc(nr_pages, sizeof(*phys), GFP_KERNEL);
	if (unlikely(!phys)) {
		err = -ENOMEM;
		goto phys_free;
	}

	mutex_lock(&kctx->csf.lock);
	kbase_csf_scheduler_lock(kbdev);

	if (unlikely(!group->csg_reg)) {
		/* The only chance of the bound csg_reg is removed from the group is
		 * that it has been put off slot by the scheduler and the csg_reg resource
		 * is contended by other groups. In this case, it needs another occasion for
		 * mapping the pma, which needs a bound csg_reg. Since the group is already
		 * off-slot, returning no error is harmless as the scheduler, when place the
		 * group back on-slot again would do the required MMU map operation on the
		 * allocated and retained pma.
		 */
		WARN_ON(group->csg_nr >= 0);
		dev_dbg(kbdev->dev, "No bound csg_reg for group_%d_%d_%d to enter protected mode",
			group->kctx->tgid, group->kctx->id, group->handle);
		goto unlock;
	}

	/* Allocate the protected mode pages */
	sbuf->pma = kbase_csf_protected_memory_alloc(kbdev, phys, nr_pages, true);
	if (unlikely(!sbuf->pma)) {
		err = -ENOMEM;
		goto unlock;
	}

	/* Map the bound susp_reg to the just allocated pma pages */
	err = kbase_csf_mcu_shared_group_update_pmode_map(kbdev, group);

unlock:
	kbase_csf_scheduler_unlock(kbdev);
	mutex_unlock(&kctx->csf.lock);
phys_free:
	kfree(phys);
	return err;
}

static void report_group_fatal_error(struct kbase_queue_group *const group)
{
	struct base_gpu_queue_group_error const
		err_payload = { .error_type = BASE_GPU_QUEUE_GROUP_ERROR_FATAL,
				.payload = { .fatal_group = {
						     .status = GPU_EXCEPTION_TYPE_SW_FAULT_0,
					     } } };

	kbase_csf_add_group_fatal_error(group, &err_payload);
	kbase_event_wakeup(group->kctx);
}

/**
 * protm_event_worker - Protected mode switch request event handler
 *			called from a workqueue.
 *
 * @data: Pointer to a work_struct embedded in GPU command queue group data.
 *
 * Request to switch to protected mode.
 */
static void protm_event_worker(struct work_struct *data)
{
	struct kbase_queue_group *const group =
		container_of(data, struct kbase_queue_group, protm_event_work);
	struct kbase_protected_suspend_buffer *sbuf = &group->protected_suspend_buf;
	int err = 0;

	KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, PROTM_EVENT_WORKER_START,
				 group, 0u);

	err = alloc_grp_protected_suspend_buffer_pages(group);
	if (!err) {
		kbase_csf_scheduler_group_protm_enter(group);
	} else if (err == -ENOMEM && sbuf->alloc_retries <= PROTM_ALLOC_MAX_RETRIES) {
		sbuf->alloc_retries++;
		/* try again to allocate pages */
		queue_work(group->kctx->csf.wq, &group->protm_event_work);
	} else if (sbuf->alloc_retries >= PROTM_ALLOC_MAX_RETRIES || err != -ENOMEM) {
		dev_err(group->kctx->kbdev->dev,
			"Failed to allocate physical pages for Protected mode suspend buffer for the group %d of context %d_%d",
			group->handle, group->kctx->tgid, group->kctx->id);
		report_group_fatal_error(group);
	}

	KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, PROTM_EVENT_WORKER_END,
				 group, 0u);
}

/**
 * handle_fault_event - Handler for CS fault.
 *
 * @queue:  Pointer to queue for which fault event was received.
 * @cs_ack: Value of the CS_ACK register in the CS kernel input page used for
 *          the queue.
 *
 * Print required information about the CS fault and notify the user space client
 * about the fault.
 */
static void
handle_fault_event(struct kbase_queue *const queue, const u32 cs_ack)
{
	struct kbase_device *const kbdev = queue->kctx->kbdev;
	struct kbase_csf_cmd_stream_group_info const *ginfo =
			&kbdev->csf.global_iface.groups[queue->group->csg_nr];
	struct kbase_csf_cmd_stream_info const *stream =
			&ginfo->streams[queue->csi_index];
	const u32 cs_fault = kbase_csf_firmware_cs_output(stream, CS_FAULT);
	const u64 cs_fault_info =
		kbase_csf_firmware_cs_output(stream, CS_FAULT_INFO_LO) |
		((u64)kbase_csf_firmware_cs_output(stream, CS_FAULT_INFO_HI)
		 << 32);
	const u8 cs_fault_exception_type =
		CS_FAULT_EXCEPTION_TYPE_GET(cs_fault);
	const u32 cs_fault_exception_data =
		CS_FAULT_EXCEPTION_DATA_GET(cs_fault);
	const u64 cs_fault_info_exception_data =
		CS_FAULT_INFO_EXCEPTION_DATA_GET(cs_fault_info);

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	dev_warn(kbdev->dev,
		 "Ctx %d_%d Group %d CSG %d CSI: %d\n"
		 "CS_FAULT.EXCEPTION_TYPE: 0x%x (%s)\n"
		 "CS_FAULT.EXCEPTION_DATA: 0x%x\n"
		 "CS_FAULT_INFO.EXCEPTION_DATA: 0x%llx\n",
		 queue->kctx->tgid, queue->kctx->id, queue->group->handle,
		 queue->group->csg_nr, queue->csi_index,
		 cs_fault_exception_type,
		 kbase_gpu_exception_name(cs_fault_exception_type),
		 cs_fault_exception_data, cs_fault_info_exception_data);


#if IS_ENABLED(CONFIG_DEBUG_FS)
	/* CS_RESOURCE_TERMINATED type fault event can be ignored from the
	 * standpoint of dump on error. It is used to report fault for the CSIs
	 * that are associated with the same CSG as the CSI for which the actual
	 * fault was reported by the Iterator.
	 * Dumping would be triggered when the actual fault is reported.
	 *
	 * CS_INHERIT_FAULT can also be ignored. It could happen due to the error
	 * in other types of queues (cpu/kcpu). If a fault had occurred in some
	 * other GPU queue then the dump would have been performed anyways when
	 * that fault was reported.
	 */
	if ((cs_fault_exception_type != CS_FAULT_EXCEPTION_TYPE_CS_INHERIT_FAULT) &&
	    (cs_fault_exception_type != CS_FAULT_EXCEPTION_TYPE_CS_RESOURCE_TERMINATED)) {
		if (unlikely(kbase_debug_csf_fault_notify(kbdev, queue->kctx, DF_CS_FAULT))) {
			get_queue(queue);
			queue->cs_error = cs_fault;
			queue->cs_error_info = cs_fault_info;
			queue->cs_error_fatal = false;
			if (!queue_work(queue->kctx->csf.wq, &queue->cs_error_work))
				release_queue(queue);
			return;
		}
	}
#endif

	kbase_csf_firmware_cs_input_mask(stream, CS_REQ, cs_ack,
					 CS_REQ_FAULT_MASK);
	kbase_csf_ring_cs_kernel_doorbell(kbdev, queue->csi_index, queue->group->csg_nr, true);
}

static void report_queue_fatal_error(struct kbase_queue *const queue,
				     u32 cs_fatal, u64 cs_fatal_info,
				     u8 group_handle)
{
	struct base_csf_notification error = {
		.type = BASE_CSF_NOTIFICATION_GPU_QUEUE_GROUP_ERROR,
		.payload = {
			.csg_error = {
				.handle = group_handle,
				.error = {
					.error_type =
					BASE_GPU_QUEUE_GROUP_QUEUE_ERROR_FATAL,
					.payload = {
						.fatal_queue = {
						.sideband = cs_fatal_info,
						.status = cs_fatal,
						.csi_index = queue->csi_index,
						}
					}
				}
			}
		}
	};

	kbase_csf_event_add_error(queue->kctx, &queue->error, &error);
	kbase_event_wakeup(queue->kctx);
}

/**
 * fatal_event_worker - Handle the CS_FATAL/CS_FAULT error for the GPU queue
 *
 * @data: Pointer to a work_struct embedded in GPU command queue.
 *
 * Terminate the CSG and report the error to userspace.
 */
static void cs_error_worker(struct work_struct *const data)
{
	struct kbase_queue *const queue =
		container_of(data, struct kbase_queue, cs_error_work);
	struct kbase_context *const kctx = queue->kctx;
	struct kbase_device *const kbdev = kctx->kbdev;
	struct kbase_queue_group *group;
	u8 group_handle;
	bool reset_prevented = false;
	int err;

	kbase_debug_csf_fault_wait_completion(kbdev);
	err = kbase_reset_gpu_prevent_and_wait(kbdev);

	if (err)
		dev_warn(
			kbdev->dev,
			"Unsuccessful GPU reset detected when terminating group to handle fatal event, attempting to terminate regardless");
	else
		reset_prevented = true;

	mutex_lock(&kctx->csf.lock);

	group = get_bound_queue_group(queue);
	if (!group) {
		dev_warn(kbdev->dev, "queue not bound when handling fatal event");
		goto unlock;
	}

#if IS_ENABLED(CONFIG_DEBUG_FS)
	if (!queue->cs_error_fatal) {
		unsigned long flags;
		int slot_num;

		kbase_csf_scheduler_spin_lock(kbdev, &flags);
		slot_num = kbase_csf_scheduler_group_get_slot_locked(group);
		if (slot_num >= 0) {
			struct kbase_csf_cmd_stream_group_info const *ginfo =
				&kbdev->csf.global_iface.groups[slot_num];
			struct kbase_csf_cmd_stream_info const *stream =
				&ginfo->streams[queue->csi_index];
			u32 const cs_ack =
				kbase_csf_firmware_cs_output(stream, CS_ACK);

			kbase_csf_firmware_cs_input_mask(stream, CS_REQ, cs_ack,
				CS_REQ_FAULT_MASK);
			kbase_csf_ring_cs_kernel_doorbell(kbdev, queue->csi_index,
				slot_num, true);
		}
		kbase_csf_scheduler_spin_unlock(kbdev, flags);
		goto unlock;
	}
#endif

	group_handle = group->handle;
	term_queue_group(group);
	flush_gpu_cache_on_fatal_error(kbdev);
	report_queue_fatal_error(queue, queue->cs_error, queue->cs_error_info,
				 group_handle);

unlock:
	release_queue(queue);
	mutex_unlock(&kctx->csf.lock);
	if (reset_prevented)
		kbase_reset_gpu_allow(kbdev);
}

/**
 * handle_fatal_event - Handler for CS fatal.
 *
 * @queue:    Pointer to queue for which fatal event was received.
 * @stream:   Pointer to the structure containing info provided by the
 *            firmware about the CSI.
 * @cs_ack: Value of the CS_ACK register in the CS kernel input page used for
 *          the queue.
 *
 * Notify a waiting user space client of the CS fatal and prints meaningful
 * information.
 * Enqueue a work item to terminate the group and report the fatal error
 * to user space.
 */
static void
handle_fatal_event(struct kbase_queue *const queue,
		   struct kbase_csf_cmd_stream_info const *const stream,
		   u32 cs_ack)
{
	const u32 cs_fatal = kbase_csf_firmware_cs_output(stream, CS_FATAL);
	const u64 cs_fatal_info =
		kbase_csf_firmware_cs_output(stream, CS_FATAL_INFO_LO) |
		((u64)kbase_csf_firmware_cs_output(stream, CS_FATAL_INFO_HI)
		 << 32);
	const u32 cs_fatal_exception_type =
		CS_FATAL_EXCEPTION_TYPE_GET(cs_fatal);
	const u32 cs_fatal_exception_data =
		CS_FATAL_EXCEPTION_DATA_GET(cs_fatal);
	const u64 cs_fatal_info_exception_data =
		CS_FATAL_INFO_EXCEPTION_DATA_GET(cs_fatal_info);
	struct kbase_device *const kbdev = queue->kctx->kbdev;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	dev_warn(kbdev->dev,
		 "Ctx %d_%d Group %d CSG %d CSI: %d\n"
		 "CS_FATAL.EXCEPTION_TYPE: 0x%x (%s)\n"
		 "CS_FATAL.EXCEPTION_DATA: 0x%x\n"
		 "CS_FATAL_INFO.EXCEPTION_DATA: 0x%llx\n",
		 queue->kctx->tgid, queue->kctx->id, queue->group->handle,
		 queue->group->csg_nr, queue->csi_index,
		 cs_fatal_exception_type,
		 kbase_gpu_exception_name(cs_fatal_exception_type),
		 cs_fatal_exception_data, cs_fatal_info_exception_data);

	if (cs_fatal_exception_type ==
			CS_FATAL_EXCEPTION_TYPE_FIRMWARE_INTERNAL_ERROR) {
		kbase_debug_csf_fault_notify(kbdev, queue->kctx, DF_FW_INTERNAL_ERROR);
		queue_work(system_wq, &kbdev->csf.fw_error_work);
	} else {
		kbase_debug_csf_fault_notify(kbdev, queue->kctx, DF_CS_FATAL);
		if (cs_fatal_exception_type == CS_FATAL_EXCEPTION_TYPE_CS_UNRECOVERABLE) {
			queue->group->cs_unrecoverable = true;
			if (kbase_prepare_to_reset_gpu(queue->kctx->kbdev, RESET_FLAGS_NONE))
				kbase_reset_gpu(queue->kctx->kbdev);
		}
		get_queue(queue);
		queue->cs_error = cs_fatal;
		queue->cs_error_info = cs_fatal_info;
		queue->cs_error_fatal = true;
		if (!queue_work(queue->kctx->csf.wq, &queue->cs_error_work))
			release_queue(queue);
	}

	kbase_csf_firmware_cs_input_mask(stream, CS_REQ, cs_ack,
					CS_REQ_FATAL_MASK);

}

/**
 * process_cs_interrupts - Process interrupts for a CS.
 *
 * @group:  Pointer to GPU command queue group data.
 * @ginfo:  The CSG interface provided by the firmware.
 * @irqreq: CSG's IRQ request bitmask (one bit per CS).
 * @irqack: CSG's IRQ acknowledge bitmask (one bit per CS).
 * @track: Pointer that tracks the highest scanout priority idle CSG
 *         and any newly potentially viable protected mode requesting
 *          CSG in current IRQ context.
 *
 * If the interrupt request bitmask differs from the acknowledge bitmask
 * then the firmware is notifying the host of an event concerning those
 * CSs indicated by bits whose value differs. The actions required
 * are then determined by examining which notification flags differ between
 * the request and acknowledge registers for the individual CS(s).
 */
static void process_cs_interrupts(struct kbase_queue_group *const group,
				  struct kbase_csf_cmd_stream_group_info const *const ginfo,
				  u32 const irqreq, u32 const irqack,
				  struct irq_idle_and_protm_track *track)
{
	struct kbase_device *const kbdev = group->kctx->kbdev;
	u32 remaining = irqreq ^ irqack;
	bool protm_pend = false;
	const bool group_suspending =
		!kbase_csf_scheduler_group_events_enabled(kbdev, group);

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	while (remaining != 0) {
		int const i = ffs(remaining) - 1;
		struct kbase_queue *const queue = group->bound_queues[i];

		remaining &= ~(1 << i);

		/* The queue pointer can be NULL, but if it isn't NULL then it
		 * cannot disappear since scheduler spinlock is held and before
		 * freeing a bound queue it has to be first unbound which
		 * requires scheduler spinlock.
		 */
		if (queue && !WARN_ON(queue->csi_index != i)) {
			struct kbase_csf_cmd_stream_info const *const stream =
				&ginfo->streams[i];
			u32 const cs_req = kbase_csf_firmware_cs_input_read(
				stream, CS_REQ);
			u32 const cs_ack =
				kbase_csf_firmware_cs_output(stream, CS_ACK);
			struct workqueue_struct *wq = group->kctx->csf.wq;

			if ((cs_ack & CS_ACK_FATAL_MASK) != (cs_req & CS_REQ_FATAL_MASK)) {
				KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_INTERRUPT_FAULT,
							 group, queue, cs_req ^ cs_ack);
				handle_fatal_event(queue, stream, cs_ack);
			}

			if ((cs_ack & CS_ACK_FAULT_MASK) != (cs_req & CS_REQ_FAULT_MASK)) {
				KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_INTERRUPT_FAULT,
							 group, queue, cs_req ^ cs_ack);
				handle_fault_event(queue, cs_ack);
			}

			/* PROTM_PEND and TILER_OOM can be safely ignored
			 * because they will be raised again if the group
			 * is assigned a CSG slot in future.
			 */
			if (group_suspending) {
				u32 const cs_req_remain = cs_req & ~CS_REQ_EXCEPTION_MASK;
				u32 const cs_ack_remain = cs_ack & ~CS_ACK_EXCEPTION_MASK;

				KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev,
							 CSI_INTERRUPT_GROUP_SUSPENDS_IGNORED,
							 group, queue,
							 cs_req_remain ^ cs_ack_remain);
				continue;
			}

			if (((cs_req & CS_REQ_TILER_OOM_MASK) ^
			     (cs_ack & CS_ACK_TILER_OOM_MASK))) {
				get_queue(queue);
				KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_INTERRUPT_TILER_OOM,
							 group, queue, cs_req ^ cs_ack);
				if (!queue_work(wq, &queue->oom_event_work)) {
					/* The work item shall not have been
					 * already queued, there can be only
					 * one pending OoM event for a
					 * queue.
					 */
					dev_warn(
						kbdev->dev,
						"Tiler OOM work pending: queue %d group %d (ctx %d_%d)",
						queue->csi_index, group->handle, queue->kctx->tgid,
						queue->kctx->id);
					release_queue(queue);
				}
			}

			if ((cs_req & CS_REQ_PROTM_PEND_MASK) ^
			    (cs_ack & CS_ACK_PROTM_PEND_MASK)) {
				KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_INTERRUPT_PROTM_PEND,
							 group, queue, cs_req ^ cs_ack);

				dev_dbg(kbdev->dev,
					"Protected mode entry request for queue on csi %d bound to group-%d on slot %d",
					queue->csi_index, group->handle,
					group->csg_nr);

				bitmap_set(group->protm_pending_bitmap, i, 1);
				KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_PROTM_PEND_SET, group, queue,
							   group->protm_pending_bitmap[0]);
				protm_pend = true;
			}
		}
	}

	if (protm_pend) {
		struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

		if (scheduler->tick_protm_pending_seq > group->scan_seq_num) {
			scheduler->tick_protm_pending_seq = group->scan_seq_num;
			track->protm_grp = group;
		}

		if (!group->protected_suspend_buf.pma)
			queue_work(group->kctx->csf.wq, &group->protm_event_work);

		if (test_bit(group->csg_nr, scheduler->csg_slots_idle_mask)) {
			clear_bit(group->csg_nr,
				  scheduler->csg_slots_idle_mask);
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_IDLE_CLEAR, group,
							scheduler->csg_slots_idle_mask[0]);
			dev_dbg(kbdev->dev,
				"Group-%d on slot %d de-idled by protm request",
				group->handle, group->csg_nr);
		}
	}
}

/**
 * process_csg_interrupts - Process interrupts for a CSG.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @csg_nr: CSG number.
 * @track: Pointer that tracks the highest idle CSG and the newly possible viable
 *         protected mode requesting group, in current IRQ context.
 *
 * Handles interrupts for a CSG and for CSs within it.
 *
 * If the CSG's request register value differs from its acknowledge register
 * then the firmware is notifying the host of an event concerning the whole
 * group. The actions required are then determined by examining which
 * notification flags differ between those two register values.
 *
 * See process_cs_interrupts() for details of per-stream interrupt handling.
 */
static void process_csg_interrupts(struct kbase_device *const kbdev, int const csg_nr,
				   struct irq_idle_and_protm_track *track)
{
	struct kbase_csf_cmd_stream_group_info *ginfo;
	struct kbase_queue_group *group = NULL;
	u32 req, ack, irqreq, irqack;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	if (WARN_ON(csg_nr >= kbdev->csf.global_iface.group_num))
		return;

	ginfo = &kbdev->csf.global_iface.groups[csg_nr];
	req = kbase_csf_firmware_csg_input_read(ginfo, CSG_REQ);
	ack = kbase_csf_firmware_csg_output(ginfo, CSG_ACK);
	irqreq = kbase_csf_firmware_csg_output(ginfo, CSG_IRQ_REQ);
	irqack = kbase_csf_firmware_csg_input_read(ginfo, CSG_IRQ_ACK);

	/* There may not be any pending CSG/CS interrupts to process */
	if ((req == ack) && (irqreq == irqack))
		return;

	/* Immediately set IRQ_ACK bits to be same as the IRQ_REQ bits before
	 * examining the CS_ACK & CS_REQ bits. This would ensure that Host
	 * doesn't misses an interrupt for the CS in the race scenario where
	 * whilst Host is servicing an interrupt for the CS, firmware sends
	 * another interrupt for that CS.
	 */
	kbase_csf_firmware_csg_input(ginfo, CSG_IRQ_ACK, irqreq);

	group = kbase_csf_scheduler_get_group_on_slot(kbdev, csg_nr);

	/* The group pointer can be NULL here if interrupts for the group
	 * (like SYNC_UPDATE, IDLE notification) were delayed and arrived
	 * just after the suspension of group completed. However if not NULL
	 * then the group pointer cannot disappear even if User tries to
	 * terminate the group whilst this loop is running as scheduler
	 * spinlock is held and for freeing a group that is resident on a CSG
	 * slot scheduler spinlock is required.
	 */
	if (!group)
		return;

	if (WARN_ON(kbase_csf_scheduler_group_get_slot_locked(group) != csg_nr))
		return;

	KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_INTERRUPT_PROCESS_START, group, csg_nr);

	if ((req ^ ack) & CSG_REQ_SYNC_UPDATE_MASK) {
		kbase_csf_firmware_csg_input_mask(ginfo,
			CSG_REQ, ack, CSG_REQ_SYNC_UPDATE_MASK);

		KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_INTERRUPT_SYNC_UPDATE, group, req ^ ack);

		/* SYNC_UPDATE events shall invalidate GPU idle event */
		atomic_set(&kbdev->csf.scheduler.gpu_no_longer_idle, true);

		kbase_csf_event_signal_cpu_only(group->kctx);
	}

	if ((req ^ ack) & CSG_REQ_IDLE_MASK) {
		struct kbase_csf_scheduler *scheduler =	&kbdev->csf.scheduler;

		KBASE_TLSTREAM_TL_KBASE_DEVICE_CSG_IDLE(
			kbdev, kbdev->gpu_props.props.raw_props.gpu_id, csg_nr);

		kbase_csf_firmware_csg_input_mask(ginfo, CSG_REQ, ack,
			CSG_REQ_IDLE_MASK);

		set_bit(csg_nr, scheduler->csg_slots_idle_mask);
		KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_IDLE_SET, group,
					 scheduler->csg_slots_idle_mask[0]);
		KBASE_KTRACE_ADD_CSF_GRP(kbdev,  CSG_INTERRUPT_IDLE, group, req ^ ack);
		dev_dbg(kbdev->dev, "Idle notification received for Group %u on slot %d\n",
			 group->handle, csg_nr);

		if (atomic_read(&scheduler->non_idle_offslot_grps)) {
			/* If there are non-idle CSGs waiting for a slot, fire
			 * a tock for a replacement.
			 */
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_INTERRUPT_NON_IDLE_GROUPS,
						group, req ^ ack);
			kbase_csf_scheduler_invoke_tock(kbdev);
		} else {
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_INTERRUPT_NO_NON_IDLE_GROUPS,
						group, req ^ ack);
		}

		if (group->scan_seq_num < track->idle_seq) {
			track->idle_seq = group->scan_seq_num;
			track->idle_slot = csg_nr;
		}
	}

	if ((req ^ ack) & CSG_REQ_PROGRESS_TIMER_EVENT_MASK) {
		kbase_csf_firmware_csg_input_mask(ginfo, CSG_REQ, ack,
						  CSG_REQ_PROGRESS_TIMER_EVENT_MASK);

		KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_INTERRUPT_PROGRESS_TIMER_EVENT, group,
					 req ^ ack);
		dev_info(
			kbdev->dev,
			"[%llu] Iterator PROGRESS_TIMER timeout notification received for group %u of ctx %d_%d on slot %d\n",
			kbase_backend_get_cycle_cnt(kbdev), group->handle, group->kctx->tgid,
			group->kctx->id, csg_nr);

		handle_progress_timer_event(group);
	}

	process_cs_interrupts(group, ginfo, irqreq, irqack, track);

	KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_INTERRUPT_PROCESS_END, group,
				 ((u64)req ^ ack) | (((u64)irqreq ^ irqack) << 32));
}

/**
 * process_prfcnt_interrupts - Process performance counter interrupts.
 *
 * @kbdev:   Instance of a GPU platform device that implements a CSF interface.
 * @glb_req: Global request register value.
 * @glb_ack: Global acknowledge register value.
 *
 * Handles interrupts issued by the firmware that relate to the performance
 * counters. For example, on completion of a performance counter sample. It is
 * expected that the scheduler spinlock is already held on calling this
 * function.
 */
static void process_prfcnt_interrupts(struct kbase_device *kbdev, u32 glb_req,
				      u32 glb_ack)
{
	const struct kbase_csf_global_iface *const global_iface =
		&kbdev->csf.global_iface;

	lockdep_assert_held(&kbdev->csf.scheduler.interrupt_lock);

	/* Process PRFCNT_SAMPLE interrupt. */
	if (kbdev->csf.hwcnt.request_pending &&
	    ((glb_req & GLB_REQ_PRFCNT_SAMPLE_MASK) ==
	     (glb_ack & GLB_REQ_PRFCNT_SAMPLE_MASK))) {
		kbdev->csf.hwcnt.request_pending = false;

		dev_dbg(kbdev->dev, "PRFCNT_SAMPLE done interrupt received.");

		kbase_hwcnt_backend_csf_on_prfcnt_sample(
			&kbdev->hwcnt_gpu_iface);
	}

	/* Process PRFCNT_ENABLE interrupt. */
	if (kbdev->csf.hwcnt.enable_pending &&
	    ((glb_req & GLB_REQ_PRFCNT_ENABLE_MASK) ==
	     (glb_ack & GLB_REQ_PRFCNT_ENABLE_MASK))) {
		kbdev->csf.hwcnt.enable_pending = false;

		dev_dbg(kbdev->dev,
			"PRFCNT_ENABLE status changed interrupt received.");

		if (glb_ack & GLB_REQ_PRFCNT_ENABLE_MASK)
			kbase_hwcnt_backend_csf_on_prfcnt_enable(
				&kbdev->hwcnt_gpu_iface);
		else
			kbase_hwcnt_backend_csf_on_prfcnt_disable(
				&kbdev->hwcnt_gpu_iface);
	}

	/* Process PRFCNT_THRESHOLD interrupt. */
	if ((glb_req ^ glb_ack) & GLB_REQ_PRFCNT_THRESHOLD_MASK) {
		dev_dbg(kbdev->dev, "PRFCNT_THRESHOLD interrupt received.");

		kbase_hwcnt_backend_csf_on_prfcnt_threshold(
			&kbdev->hwcnt_gpu_iface);

		/* Set the GLB_REQ.PRFCNT_THRESHOLD flag back to
		 * the same value as GLB_ACK.PRFCNT_THRESHOLD
		 * flag in order to enable reporting of another
		 * PRFCNT_THRESHOLD event.
		 */
		kbase_csf_firmware_global_input_mask(
			global_iface, GLB_REQ, glb_ack,
			GLB_REQ_PRFCNT_THRESHOLD_MASK);
	}

	/* Process PRFCNT_OVERFLOW interrupt. */
	if ((glb_req ^ glb_ack) & GLB_REQ_PRFCNT_OVERFLOW_MASK) {
		dev_dbg(kbdev->dev, "PRFCNT_OVERFLOW interrupt received.");

		kbase_hwcnt_backend_csf_on_prfcnt_overflow(
			&kbdev->hwcnt_gpu_iface);

		/* Set the GLB_REQ.PRFCNT_OVERFLOW flag back to
		 * the same value as GLB_ACK.PRFCNT_OVERFLOW
		 * flag in order to enable reporting of another
		 * PRFCNT_OVERFLOW event.
		 */
		kbase_csf_firmware_global_input_mask(
			global_iface, GLB_REQ, glb_ack,
			GLB_REQ_PRFCNT_OVERFLOW_MASK);
	}
}

/**
 * check_protm_enter_req_complete - Check if PROTM_ENTER request completed
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @glb_req: Global request register value.
 * @glb_ack: Global acknowledge register value.
 *
 * This function checks if the PROTM_ENTER Global request had completed and
 * appropriately sends notification about the protected mode entry to components
 * like IPA, HWC, IPA_CONTROL.
 */
static inline void check_protm_enter_req_complete(struct kbase_device *kbdev,
						  u32 glb_req, u32 glb_ack)
{
	lockdep_assert_held(&kbdev->hwaccess_lock);
	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	if (likely(!kbdev->csf.scheduler.active_protm_grp))
		return;

	if (kbdev->protected_mode)
		return;

	if ((glb_req & GLB_REQ_PROTM_ENTER_MASK) !=
	    (glb_ack & GLB_REQ_PROTM_ENTER_MASK))
		return;

	dev_dbg(kbdev->dev, "Protected mode entry interrupt received");

	kbdev->protected_mode = true;
	kbase_ipa_protection_mode_switch_event(kbdev);
	kbase_ipa_control_protm_entered(kbdev);
	kbase_hwcnt_backend_csf_protm_entered(&kbdev->hwcnt_gpu_iface);
}

/**
 * process_protm_exit - Handle the protected mode exit interrupt
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @glb_ack: Global acknowledge register value.
 *
 * This function handles the PROTM_EXIT interrupt and sends notification
 * about the protected mode exit to components like HWC, IPA_CONTROL.
 */
static inline void process_protm_exit(struct kbase_device *kbdev, u32 glb_ack)
{
	const struct kbase_csf_global_iface *const global_iface =
		&kbdev->csf.global_iface;
	struct kbase_csf_scheduler *scheduler =	&kbdev->csf.scheduler;

	lockdep_assert_held(&kbdev->hwaccess_lock);
	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	dev_dbg(kbdev->dev, "Protected mode exit interrupt received");

	kbase_csf_firmware_global_input_mask(global_iface, GLB_REQ, glb_ack,
					     GLB_REQ_PROTM_EXIT_MASK);

	if (likely(scheduler->active_protm_grp)) {
		KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_PROTM_EXIT,
					 scheduler->active_protm_grp, 0u);
		scheduler->active_protm_grp = NULL;
	} else {
		dev_warn(kbdev->dev, "PROTM_EXIT interrupt after no pmode group");
	}

	if (!WARN_ON(!kbdev->protected_mode)) {
		kbdev->protected_mode = false;
		kbase_ipa_control_protm_exited(kbdev);
		kbase_hwcnt_backend_csf_protm_exited(&kbdev->hwcnt_gpu_iface);
	}

#if IS_ENABLED(CONFIG_MALI_CORESIGHT)
	kbase_debug_coresight_csf_enable_pmode_exit(kbdev);
#endif /* IS_ENABLED(CONFIG_MALI_CORESIGHT) */
}

static inline void process_tracked_info_for_protm(struct kbase_device *kbdev,
						  struct irq_idle_and_protm_track *track)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	struct kbase_queue_group *group = track->protm_grp;
	u32 current_protm_pending_seq = scheduler->tick_protm_pending_seq;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	if (likely(current_protm_pending_seq == KBASEP_TICK_PROTM_PEND_SCAN_SEQ_NR_INVALID))
		return;

	/* Handle protm from the tracked information */
	if (track->idle_seq < current_protm_pending_seq) {
		/* If the protm enter was prevented due to groups priority, then fire a tock
		 * for the scheduler to re-examine the case.
		 */
		dev_dbg(kbdev->dev, "Attempt pending protm from idle slot %d\n", track->idle_slot);
		kbase_csf_scheduler_invoke_tock(kbdev);
	} else if (group) {
		u32 i, num_groups = kbdev->csf.global_iface.group_num;
		struct kbase_queue_group *grp;
		bool tock_triggered = false;

		/* A new protm request, and track->idle_seq is not sufficient, check across
		 * previously notified idle CSGs in the current tick/tock cycle.
		 */
		for_each_set_bit(i, scheduler->csg_slots_idle_mask, num_groups) {
			if (i == track->idle_slot)
				continue;
			grp = kbase_csf_scheduler_get_group_on_slot(kbdev, i);
			/* If not NULL then the group pointer cannot disappear as the
			 * scheduler spinlock is held.
			 */
			if (grp == NULL)
				continue;

			if (grp->scan_seq_num < current_protm_pending_seq) {
				tock_triggered = true;
				dev_dbg(kbdev->dev,
					"Attempt new protm from tick/tock idle slot %d\n", i);
				kbase_csf_scheduler_invoke_tock(kbdev);
				break;
			}
		}

		if (!tock_triggered) {
			dev_dbg(kbdev->dev, "Group-%d on slot-%d start protm work\n",
				group->handle, group->csg_nr);
			queue_work(group->kctx->csf.wq, &group->protm_event_work);
		}
	}
}

static void order_job_irq_clear_with_iface_mem_read(void)
{
	/* Ensure that write to the JOB_IRQ_CLEAR is ordered with regards to the
	 * read from interface memory. The ordering is needed considering the way
	 * FW & Kbase writes to the JOB_IRQ_RAWSTAT and JOB_IRQ_CLEAR registers
	 * without any synchronization. Without the barrier there is no guarantee
	 * about the ordering, the write to IRQ_CLEAR can take effect after the read
	 * from interface memory and that could cause a problem for the scenario where
	 * FW sends back to back notifications for the same CSG for events like
	 * SYNC_UPDATE and IDLE, but Kbase gets a single IRQ and observes only the
	 * first event. Similar thing can happen with glb events like CFG_ALLOC_EN
	 * acknowledgment and GPU idle notification.
	 *
	 *       MCU                                    CPU
	 *  ---------------                         ----------------
	 *  Update interface memory                 Write to IRQ_CLEAR to clear current IRQ
	 *  <barrier>                               <barrier>
	 *  Write to IRQ_RAWSTAT to raise new IRQ   Read interface memory
	 */

	/* CPU and GPU would be in the same Outer shareable domain */
	dmb(osh);
}

void kbase_csf_interrupt(struct kbase_device *kbdev, u32 val)
{
	bool deferred_handling_glb_idle_irq = false;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	KBASE_KTRACE_ADD(kbdev, CSF_INTERRUPT_START, NULL, val);

	do {
		unsigned long flags;
		u32 csg_interrupts = val & ~JOB_IRQ_GLOBAL_IF;
		struct irq_idle_and_protm_track track = { .protm_grp = NULL, .idle_seq = U32_MAX };
		bool glb_idle_irq_received = false;

		kbase_reg_write(kbdev, JOB_CONTROL_REG(JOB_IRQ_CLEAR), val);
		order_job_irq_clear_with_iface_mem_read();

		if (csg_interrupts != 0) {
			kbase_csf_scheduler_spin_lock(kbdev, &flags);
			/* Looping through and track the highest idle and protm groups */
			while (csg_interrupts != 0) {
				int const csg_nr = ffs(csg_interrupts) - 1;

				process_csg_interrupts(kbdev, csg_nr, &track);
				csg_interrupts &= ~(1 << csg_nr);
			}

			/* Handle protm from the tracked information */
			process_tracked_info_for_protm(kbdev, &track);
			kbase_csf_scheduler_spin_unlock(kbdev, flags);
		}

		if (val & JOB_IRQ_GLOBAL_IF) {
			const struct kbase_csf_global_iface *const global_iface =
				&kbdev->csf.global_iface;

			kbdev->csf.interrupt_received = true;

			if (!kbdev->csf.firmware_reloaded)
				kbase_csf_firmware_reload_completed(kbdev);
			else if (global_iface->output) {
				u32 glb_req, glb_ack;

				kbase_csf_scheduler_spin_lock(kbdev, &flags);
				glb_req =
					kbase_csf_firmware_global_input_read(global_iface, GLB_REQ);
				glb_ack = kbase_csf_firmware_global_output(global_iface, GLB_ACK);
				KBASE_KTRACE_ADD(kbdev, CSF_INTERRUPT_GLB_REQ_ACK, NULL,
						 glb_req ^ glb_ack);

				check_protm_enter_req_complete(kbdev, glb_req, glb_ack);

				if ((glb_req ^ glb_ack) & GLB_REQ_PROTM_EXIT_MASK)
					process_protm_exit(kbdev, glb_ack);

				/* Handle IDLE Hysteresis notification event */
				if ((glb_req ^ glb_ack) & GLB_REQ_IDLE_EVENT_MASK) {
					dev_dbg(kbdev->dev, "Idle-hysteresis event flagged");
					kbase_csf_firmware_global_input_mask(
						global_iface, GLB_REQ, glb_ack,
						GLB_REQ_IDLE_EVENT_MASK);

					glb_idle_irq_received = true;
					/* Defer handling this IRQ to account for a race condition
					 * where the idle worker could be executed before we have
					 * finished handling all pending IRQs (including CSG IDLE
					 * IRQs).
					 */
					deferred_handling_glb_idle_irq = true;
				}

				process_prfcnt_interrupts(kbdev, glb_req, glb_ack);

				kbase_csf_scheduler_spin_unlock(kbdev, flags);

				/* Invoke the MCU state machine as a state transition
				 * might have completed.
				 */
				kbase_pm_update_state(kbdev);
			}
		}

		if (!glb_idle_irq_received)
			break;
		/* Attempt to serve potential IRQs that might have occurred
		 * whilst handling the previous IRQ. In case we have observed
		 * the GLB IDLE IRQ without all CSGs having been marked as
		 * idle, the GPU would be treated as no longer idle and left
		 * powered on.
		 */
		val = kbase_reg_read(kbdev, JOB_CONTROL_REG(JOB_IRQ_STATUS));
	} while (val);

	if (deferred_handling_glb_idle_irq) {
		unsigned long flags;

		kbase_csf_scheduler_spin_lock(kbdev, &flags);
		kbase_csf_scheduler_process_gpu_idle_event(kbdev);
		kbase_csf_scheduler_spin_unlock(kbdev, flags);
	}

	wake_up_all(&kbdev->csf.event_wait);

	KBASE_KTRACE_ADD(kbdev, CSF_INTERRUPT_END, NULL, val);
}

void kbase_csf_doorbell_mapping_term(struct kbase_device *kbdev)
{
	if (kbdev->csf.db_filp) {
		struct page *page = as_page(kbdev->csf.dummy_db_page);

		kbase_mem_pool_free(
			&kbdev->mem_pools.small[KBASE_MEM_GROUP_CSF_FW],
			page, false);

		fput(kbdev->csf.db_filp);
	}
}

int kbase_csf_doorbell_mapping_init(struct kbase_device *kbdev)
{
	struct tagged_addr phys;
	struct file *filp;
	int ret;

	filp = shmem_file_setup("mali csf db", MAX_LFS_FILESIZE, VM_NORESERVE);
	if (IS_ERR(filp))
		return PTR_ERR(filp);

	ret = kbase_mem_pool_alloc_pages(&kbdev->mem_pools.small[KBASE_MEM_GROUP_CSF_FW], 1, &phys,
					 false, NULL);

	if (ret <= 0) {
		fput(filp);
		return ret;
	}

	kbdev->csf.db_filp = filp;
	kbdev->csf.dummy_db_page = phys;
	kbdev->csf.db_file_offsets = 0;

	return 0;
}

void kbase_csf_free_dummy_user_reg_page(struct kbase_device *kbdev)
{
	if (kbdev->csf.user_reg.filp) {
		struct page *page = as_page(kbdev->csf.user_reg.dummy_page);

		kbase_mem_pool_free(&kbdev->mem_pools.small[KBASE_MEM_GROUP_CSF_FW], page, false);
		fput(kbdev->csf.user_reg.filp);
	}
}

int kbase_csf_setup_dummy_user_reg_page(struct kbase_device *kbdev)
{
	struct tagged_addr phys;
	struct file *filp;
	struct page *page;
	u32 *addr;

	kbdev->csf.user_reg.filp = NULL;

	filp = shmem_file_setup("mali csf user_reg", MAX_LFS_FILESIZE, VM_NORESERVE);
	if (IS_ERR(filp)) {
		dev_err(kbdev->dev, "failed to get an unlinked file for user_reg");
		return PTR_ERR(filp);
	}

	if (kbase_mem_pool_alloc_pages(&kbdev->mem_pools.small[KBASE_MEM_GROUP_CSF_FW], 1, &phys,
				       false, NULL) <= 0) {
		fput(filp);
		return -ENOMEM;
	}

	page = as_page(phys);
	addr = kmap_atomic(page);

	/* Write a special value for the latest flush register inside the
	 * dummy page
	 */
	addr[LATEST_FLUSH / sizeof(u32)] = POWER_DOWN_LATEST_FLUSH_VALUE;

	kbase_sync_single_for_device(kbdev, kbase_dma_addr(page) + LATEST_FLUSH, sizeof(u32),
				     DMA_BIDIRECTIONAL);
	kunmap_atomic(addr);

	kbdev->csf.user_reg.filp = filp;
	kbdev->csf.user_reg.dummy_page = phys;
	kbdev->csf.user_reg.file_offset = 0;
	return 0;
}

u8 kbase_csf_priority_check(struct kbase_device *kbdev, u8 req_priority)
{
	struct priority_control_manager_device *pcm_device = kbdev->pcm_dev;
	u8 out_priority = req_priority;

	if (pcm_device) {
		req_priority = kbase_csf_priority_queue_group_priority_to_relative(req_priority);
		out_priority = pcm_device->ops.pcm_scheduler_priority_check(pcm_device, current, req_priority);
		out_priority = kbase_csf_priority_relative_to_queue_group_priority(out_priority);
	}

	return out_priority;
}
