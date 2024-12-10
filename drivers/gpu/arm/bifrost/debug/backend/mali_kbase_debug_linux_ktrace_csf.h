/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
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

/*
 * NOTE: This must **only** be included through mali_linux_trace.h,
 * otherwise it will fail to setup tracepoints correctly
 */

#if !defined(_KBASE_DEBUG_LINUX_KTRACE_CSF_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _KBASE_DEBUG_LINUX_KTRACE_CSF_H_

/*
 * Generic CSF events - using the common DEFINE_MALI_ADD_EVENT
 */
DEFINE_MALI_ADD_EVENT(SCHEDULER_EVICT_CTX_SLOTS_START);
DEFINE_MALI_ADD_EVENT(SCHEDULER_EVICT_CTX_SLOTS_END);
DEFINE_MALI_ADD_EVENT(CSF_FIRMWARE_BOOT);
DEFINE_MALI_ADD_EVENT(CSF_FIRMWARE_REBOOT);
DEFINE_MALI_ADD_EVENT(SCHEDULER_TOCK_INVOKE);
DEFINE_MALI_ADD_EVENT(SCHEDULER_TICK_INVOKE);
DEFINE_MALI_ADD_EVENT(SCHEDULER_TOCK_START);
DEFINE_MALI_ADD_EVENT(SCHEDULER_TOCK_END);
DEFINE_MALI_ADD_EVENT(SCHEDULER_TICK_START);
DEFINE_MALI_ADD_EVENT(SCHEDULER_TICK_END);
DEFINE_MALI_ADD_EVENT(SCHEDULER_RESET_START);
DEFINE_MALI_ADD_EVENT(SCHEDULER_RESET_END);
DEFINE_MALI_ADD_EVENT(SCHEDULER_PROTM_WAIT_QUIT_START);
DEFINE_MALI_ADD_EVENT(SCHEDULER_PROTM_WAIT_QUIT_END);
DEFINE_MALI_ADD_EVENT(SCHEDULER_GROUP_SYNC_UPDATE_EVENT);
DEFINE_MALI_ADD_EVENT(CSF_SYNC_UPDATE_NOTIFY_GPU_EVENT);
DEFINE_MALI_ADD_EVENT(CSF_INTERRUPT_START);
DEFINE_MALI_ADD_EVENT(CSF_INTERRUPT_END);
DEFINE_MALI_ADD_EVENT(CSF_INTERRUPT_GLB_REQ_ACK);
DEFINE_MALI_ADD_EVENT(SCHEDULER_GPU_IDLE_EVENT_CAN_SUSPEND);
DEFINE_MALI_ADD_EVENT(SCHEDULER_TICK_ADVANCE);
DEFINE_MALI_ADD_EVENT(SCHEDULER_TICK_NOADVANCE);
DEFINE_MALI_ADD_EVENT(SCHEDULER_RUNNABLE_KCTX_INSERT);
DEFINE_MALI_ADD_EVENT(SCHEDULER_RUNNABLE_KCTX_REMOVE);
DEFINE_MALI_ADD_EVENT(SCHEDULER_RUNNABLE_KCTX_ROTATE);
DEFINE_MALI_ADD_EVENT(SCHEDULER_RUNNABLE_KCTX_HEAD);
DEFINE_MALI_ADD_EVENT(SCHEDULER_GPU_IDLE_WORKER_START);
DEFINE_MALI_ADD_EVENT(SCHEDULER_GPU_IDLE_WORKER_END);
DEFINE_MALI_ADD_EVENT(SCHEDULER_GROUP_SYNC_UPDATE_WORKER_START);
DEFINE_MALI_ADD_EVENT(SCHEDULER_GROUP_SYNC_UPDATE_WORKER_END);
DEFINE_MALI_ADD_EVENT(SCHEDULER_UPDATE_IDLE_SLOTS_ACK);
DEFINE_MALI_ADD_EVENT(SCHEDULER_GPU_IDLE_WORKER_HANDLING_START);
DEFINE_MALI_ADD_EVENT(SCHEDULER_GPU_IDLE_WORKER_HANDLING_END);
DEFINE_MALI_ADD_EVENT(CSF_FIRMWARE_MCU_HALTED);
DEFINE_MALI_ADD_EVENT(CSF_FIRMWARE_MCU_SLEEP);
DEFINE_MALI_ADD_EVENT(SCHED_BUSY);
DEFINE_MALI_ADD_EVENT(SCHED_INACTIVE);
DEFINE_MALI_ADD_EVENT(SCHED_SUSPENDED);
DEFINE_MALI_ADD_EVENT(SCHED_SLEEPING);
#define KBASEP_MCU_STATE(n) DEFINE_MALI_ADD_EVENT(PM_MCU_ ## n);
#include "backend/gpu/mali_kbase_pm_mcu_states.h"
#undef KBASEP_MCU_STATE

DECLARE_EVENT_CLASS(mali_csf_grp_q_template,
	TP_PROTO(struct kbase_device *kbdev, struct kbase_queue_group *group,
			struct kbase_queue *queue, u64 info_val),
	TP_ARGS(kbdev, group, queue, info_val),
	TP_STRUCT__entry(
		__field(u64, info_val)
		__field(pid_t, kctx_tgid)
		__field(u32, kctx_id)
		__field(u8, group_handle)
		__field(s8, csg_nr)
		__field(u8, slot_prio)
		__field(s8, csi_index)
	),
	TP_fast_assign(
		{
			struct kbase_context *kctx = NULL;

			__entry->info_val = info_val;
			/* Note: if required in future, we could record some
			 * flags in __entry about whether the group/queue parts
			 * are valid, and add that to the trace message e.g.
			 * by using __print_flags()/__print_symbolic()
			 */
			if (queue) {
				/* Note: kctx overridden by group->kctx later if group is valid */
				kctx = queue->kctx;
				__entry->csi_index = queue->csi_index;
			} else {
				__entry->csi_index = -1;
			}

			if (group) {
				kctx = group->kctx;
				__entry->group_handle = group->handle;
				__entry->csg_nr = group->csg_nr;
				if (group->csg_nr >= 0)
					__entry->slot_prio = kbdev->csf.scheduler.csg_slots[group->csg_nr].priority;
				else
					__entry->slot_prio = 0u;
			} else {
				__entry->group_handle = 0u;
				__entry->csg_nr = -1;
				__entry->slot_prio = 0u;
			}
			__entry->kctx_id = (kctx) ? kctx->id : 0u;
			__entry->kctx_tgid = (kctx) ? kctx->tgid : 0;
		}

	),
	TP_printk("kctx=%d_%u group=%u slot=%d prio=%u csi=%d info=0x%llx",
			__entry->kctx_tgid, __entry->kctx_id,
			__entry->group_handle, __entry->csg_nr,
			__entry->slot_prio, __entry->csi_index,
			__entry->info_val)
);

/*
 * Group events
 */
#define DEFINE_MALI_CSF_GRP_EVENT(name) \
	DEFINE_EVENT_PRINT(mali_csf_grp_q_template, mali_##name, \
	TP_PROTO(struct kbase_device *kbdev, struct kbase_queue_group *group, \
			struct kbase_queue *queue, u64 info_val), \
	TP_ARGS(kbdev, group, queue, info_val), \
	TP_printk("kctx=%d_%u group=%u slot=%d prio=%u info=0x%llx", \
		__entry->kctx_tgid, __entry->kctx_id, __entry->group_handle, \
		__entry->csg_nr, __entry->slot_prio, __entry->info_val))

DEFINE_MALI_CSF_GRP_EVENT(CSG_SLOT_START_REQ);
DEFINE_MALI_CSF_GRP_EVENT(CSG_SLOT_STOP_REQ);
DEFINE_MALI_CSF_GRP_EVENT(CSG_SLOT_RUNNING);
DEFINE_MALI_CSF_GRP_EVENT(CSG_SLOT_STOPPED);
DEFINE_MALI_CSF_GRP_EVENT(CSG_SLOT_CLEANED);
DEFINE_MALI_CSF_GRP_EVENT(CSG_UPDATE_IDLE_SLOT_REQ);
DEFINE_MALI_CSF_GRP_EVENT(CSG_SLOT_IDLE_SET);
DEFINE_MALI_CSF_GRP_EVENT(CSG_INTERRUPT_NO_NON_IDLE_GROUPS);
DEFINE_MALI_CSF_GRP_EVENT(CSG_INTERRUPT_NON_IDLE_GROUPS);
DEFINE_MALI_CSF_GRP_EVENT(CSG_SLOT_IDLE_CLEAR);
DEFINE_MALI_CSF_GRP_EVENT(CSG_SLOT_PRIO_UPDATE);
DEFINE_MALI_CSF_GRP_EVENT(CSG_INTERRUPT_SYNC_UPDATE);
DEFINE_MALI_CSF_GRP_EVENT(CSG_INTERRUPT_IDLE);
DEFINE_MALI_CSF_GRP_EVENT(CSG_INTERRUPT_PROGRESS_TIMER_EVENT);
DEFINE_MALI_CSF_GRP_EVENT(CSG_INTERRUPT_PROCESS_START);
DEFINE_MALI_CSF_GRP_EVENT(CSG_INTERRUPT_PROCESS_END);
DEFINE_MALI_CSF_GRP_EVENT(GROUP_SYNC_UPDATE_DONE);
DEFINE_MALI_CSF_GRP_EVENT(GROUP_DESCHEDULE);
DEFINE_MALI_CSF_GRP_EVENT(GROUP_SCHEDULE);
DEFINE_MALI_CSF_GRP_EVENT(GROUP_EVICT);
DEFINE_MALI_CSF_GRP_EVENT(GROUP_RUNNABLE_INSERT);
DEFINE_MALI_CSF_GRP_EVENT(GROUP_RUNNABLE_REMOVE);
DEFINE_MALI_CSF_GRP_EVENT(GROUP_RUNNABLE_ROTATE);
DEFINE_MALI_CSF_GRP_EVENT(GROUP_RUNNABLE_HEAD);
DEFINE_MALI_CSF_GRP_EVENT(GROUP_IDLE_WAIT_INSERT);
DEFINE_MALI_CSF_GRP_EVENT(GROUP_IDLE_WAIT_REMOVE);
DEFINE_MALI_CSF_GRP_EVENT(GROUP_IDLE_WAIT_HEAD);
DEFINE_MALI_CSF_GRP_EVENT(SCHEDULER_PROTM_ENTER_CHECK);
DEFINE_MALI_CSF_GRP_EVENT(SCHEDULER_PROTM_ENTER);
DEFINE_MALI_CSF_GRP_EVENT(SCHEDULER_PROTM_EXIT);
DEFINE_MALI_CSF_GRP_EVENT(SCHEDULER_TOP_GRP);
DEFINE_MALI_CSF_GRP_EVENT(SCHEDULER_NONIDLE_OFFSLOT_GRP_INC);
DEFINE_MALI_CSF_GRP_EVENT(SCHEDULER_NONIDLE_OFFSLOT_GRP_DEC);
DEFINE_MALI_CSF_GRP_EVENT(SCHEDULER_HANDLE_IDLE_SLOTS);
DEFINE_MALI_CSF_GRP_EVENT(PROTM_EVENT_WORKER_START);
DEFINE_MALI_CSF_GRP_EVENT(PROTM_EVENT_WORKER_END);
DEFINE_MALI_CSF_GRP_EVENT(CSF_GROUP_INACTIVE);
DEFINE_MALI_CSF_GRP_EVENT(CSF_GROUP_RUNNABLE);
DEFINE_MALI_CSF_GRP_EVENT(CSF_GROUP_IDLE);
DEFINE_MALI_CSF_GRP_EVENT(CSF_GROUP_SUSPENDED);
DEFINE_MALI_CSF_GRP_EVENT(CSF_GROUP_SUSPENDED_ON_IDLE);
DEFINE_MALI_CSF_GRP_EVENT(CSF_GROUP_SUSPENDED_ON_WAIT_SYNC);
DEFINE_MALI_CSF_GRP_EVENT(CSF_GROUP_FAULT_EVICTED);
DEFINE_MALI_CSF_GRP_EVENT(CSF_GROUP_TERMINATED);

#undef DEFINE_MALI_CSF_GRP_EVENT

/*
 * Group + Queue events
 */
#define DEFINE_MALI_CSF_GRP_Q_EVENT(name)  \
	DEFINE_EVENT(mali_csf_grp_q_template, mali_##name, \
	TP_PROTO(struct kbase_device *kbdev, struct kbase_queue_group *group, \
			struct kbase_queue *queue, u64 info_val), \
	TP_ARGS(kbdev, group, queue, info_val))

DEFINE_MALI_CSF_GRP_Q_EVENT(CSI_START);
DEFINE_MALI_CSF_GRP_Q_EVENT(CSI_STOP);
DEFINE_MALI_CSF_GRP_Q_EVENT(CSI_STOP_REQ);
DEFINE_MALI_CSF_GRP_Q_EVENT(CSI_INTERRUPT_GROUP_SUSPENDS_IGNORED);
DEFINE_MALI_CSF_GRP_Q_EVENT(CSI_INTERRUPT_FAULT);
DEFINE_MALI_CSF_GRP_Q_EVENT(CSI_INTERRUPT_TILER_OOM);
DEFINE_MALI_CSF_GRP_Q_EVENT(CSI_INTERRUPT_PROTM_PEND);
DEFINE_MALI_CSF_GRP_Q_EVENT(CSI_PROTM_ACK);
DEFINE_MALI_CSF_GRP_Q_EVENT(QUEUE_START);
DEFINE_MALI_CSF_GRP_Q_EVENT(QUEUE_STOP);
DEFINE_MALI_CSF_GRP_Q_EVENT(QUEUE_SYNC_UPDATE_EVAL_START);
DEFINE_MALI_CSF_GRP_Q_EVENT(QUEUE_SYNC_UPDATE_EVAL_END);
DEFINE_MALI_CSF_GRP_Q_EVENT(QUEUE_SYNC_UPDATE_WAIT_STATUS);
DEFINE_MALI_CSF_GRP_Q_EVENT(QUEUE_SYNC_UPDATE_CUR_VAL);
DEFINE_MALI_CSF_GRP_Q_EVENT(QUEUE_SYNC_UPDATE_TEST_VAL);
DEFINE_MALI_CSF_GRP_Q_EVENT(QUEUE_SYNC_UPDATE_BLOCKED_REASON);
DEFINE_MALI_CSF_GRP_Q_EVENT(CSI_PROTM_PEND_SET);
DEFINE_MALI_CSF_GRP_Q_EVENT(CSI_PROTM_PEND_CLEAR);

#undef DEFINE_MALI_CSF_GRP_Q_EVENT

/*
 * KCPU queue events
 */
DECLARE_EVENT_CLASS(mali_csf_kcpu_queue_template,
	TP_PROTO(struct kbase_kcpu_command_queue *queue,
		 u64 info_val1, u64 info_val2),
	TP_ARGS(queue, info_val1, info_val2),
	TP_STRUCT__entry(
		__field(u64, info_val1)
		__field(u64, info_val2)
		__field(pid_t, kctx_tgid)
		__field(u32, kctx_id)
		__field(u8, id)
	),
	TP_fast_assign(
		{
			__entry->info_val1 = info_val1;
			__entry->info_val2 = info_val2;
			__entry->kctx_id = queue->kctx->id;
			__entry->kctx_tgid = queue->kctx->tgid;
			__entry->id = queue->id;
		}

	),
	TP_printk("kctx=%d_%u id=%u info_val1=0x%llx info_val2=0x%llx",
			__entry->kctx_tgid, __entry->kctx_id, __entry->id,
			__entry->info_val1, __entry->info_val2)
);

#define DEFINE_MALI_CSF_KCPU_EVENT(name)  \
	DEFINE_EVENT(mali_csf_kcpu_queue_template, mali_##name, \
	TP_PROTO(struct kbase_kcpu_command_queue *queue, \
		 u64 info_val1, u64 info_val2), \
	TP_ARGS(queue, info_val1, info_val2))

DEFINE_MALI_CSF_KCPU_EVENT(KCPU_QUEUE_CREATE);
DEFINE_MALI_CSF_KCPU_EVENT(KCPU_QUEUE_DELETE);
DEFINE_MALI_CSF_KCPU_EVENT(KCPU_CQS_SET);
DEFINE_MALI_CSF_KCPU_EVENT(KCPU_CQS_WAIT_START);
DEFINE_MALI_CSF_KCPU_EVENT(KCPU_CQS_WAIT_END);
DEFINE_MALI_CSF_KCPU_EVENT(KCPU_FENCE_SIGNAL);
DEFINE_MALI_CSF_KCPU_EVENT(KCPU_FENCE_WAIT_START);
DEFINE_MALI_CSF_KCPU_EVENT(KCPU_FENCE_WAIT_END);

#undef DEFINE_MALI_CSF_KCPU_EVENT

#endif /* !defined(_KBASE_DEBUG_LINUX_KTRACE_CSF_H_) || defined(TRACE_HEADER_MULTI_READ) */
