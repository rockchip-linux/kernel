/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM ufshcd
#define TRACE_INCLUDE_PATH trace/hooks
#if !defined(_TRACE_HOOK_UFSHCD_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_UFSHCD_H
#include <linux/tracepoint.h>
#include <trace/hooks/vendor_hooks.h>
/*
 * Following tracepoints are not exported in tracefs and provide a
 * mechanism for vendor modules to hook and extend functionality
 */
#if defined(__GENKSYMS__) || !IS_ENABLED(CONFIG_SCSI_UFSHCD)
struct ufs_hba;
struct ufshcd_lrb;
struct uic_command;
struct request;
struct scsi_device;
#else
/* struct ufs_hba, struct ufshcd_lrb, struct uic_command */
#include <../drivers/scsi/ufs/ufshcd.h>
/* struct request */
#include <linux/blkdev.h>
/* struct scsi_device */
#include <scsi/scsi_device.h>
#endif /* __GENKSYMS__ */

DECLARE_HOOK(android_vh_ufs_fill_prdt,
	TP_PROTO(struct ufs_hba *hba, struct ufshcd_lrb *lrbp,
		 unsigned int segments, int *err),
	TP_ARGS(hba, lrbp, segments, err));

DECLARE_RESTRICTED_HOOK(android_rvh_ufs_complete_init,
			TP_PROTO(struct ufs_hba *hba),
			TP_ARGS(hba), 1);

DECLARE_RESTRICTED_HOOK(android_rvh_ufs_reprogram_all_keys,
			TP_PROTO(struct ufs_hba *hba, int *err),
			TP_ARGS(hba, err), 1);

DECLARE_HOOK(android_vh_ufs_prepare_command,
	TP_PROTO(struct ufs_hba *hba, struct request *rq,
		 struct ufshcd_lrb *lrbp, int *err),
	TP_ARGS(hba, rq, lrbp, err));

DECLARE_HOOK(android_vh_ufs_update_sysfs,
	TP_PROTO(struct ufs_hba *hba),
	TP_ARGS(hba));

DECLARE_HOOK(android_vh_ufs_send_command,
	TP_PROTO(struct ufs_hba *hba, struct ufshcd_lrb *lrbp),
	TP_ARGS(hba, lrbp));

DECLARE_HOOK(android_vh_ufs_compl_command,
	TP_PROTO(struct ufs_hba *hba, struct ufshcd_lrb *lrbp),
	TP_ARGS(hba, lrbp));

DECLARE_HOOK(android_vh_ufs_send_uic_command,
	TP_PROTO(struct ufs_hba *hba, struct uic_command *ucmd,
		 const char *str),
	TP_ARGS(hba, ucmd, str));

DECLARE_HOOK(android_vh_ufs_send_tm_command,
	TP_PROTO(struct ufs_hba *hba, int tag, const char *str),
	TP_ARGS(hba, tag, str));

DECLARE_HOOK(android_vh_ufs_check_int_errors,
	TP_PROTO(struct ufs_hba *hba, bool queue_eh_work),
	TP_ARGS(hba, queue_eh_work));

DECLARE_HOOK(android_vh_ufs_update_sdev,
	TP_PROTO(struct scsi_device *sdev),
	TP_ARGS(sdev));

DECLARE_HOOK(android_vh_ufs_clock_scaling,
	TP_PROTO(struct ufs_hba *hba, bool *force_out, bool *force_scaling, bool *scale_up),
	TP_ARGS(hba, force_out, force_scaling, scale_up));
#endif /* _TRACE_HOOK_UFSHCD_H */
/* This part must be outside protection */
#include <trace/define_trace.h>
