/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 *
 * Author: Huang Lee <Putin.li@rock-chips.com>
 */

#ifndef __LINUX_RKRGA_JOB_H_
#define __LINUX_RKRGA_JOB_H_

#include <linux/spinlock.h>
#include <linux/dma-fence.h>

#include "rga_drv.h"

enum job_flags {
	RGA_JOB_DONE			= 1 << 0,
	RGA_JOB_ASYNC			= 1 << 1,
	RGA_JOB_SYNC			= 1 << 2,
	RGA_JOB_USE_HANDLE		= 1 << 3,
	RGA_JOB_UNSUPPORT_RGA_MMU	= 1 << 4,
};

void rga_job_scheduler_dump_info(struct rga_scheduler_t *scheduler);
void rga_job_next(struct rga_scheduler_t *scheduler);
struct rga_job *rga_job_done(struct rga_scheduler_t *scheduler);
struct rga_job *rga_job_commit(struct rga_req *rga_command_base, struct rga_request *request);
int rga_job_mpi_commit(struct rga_req *rga_command_base, struct rga_request *request);

int rga_job_assign(struct rga_job *job);


int rga_request_check(struct rga_user_request *req);
struct rga_request *rga_request_lookup(struct rga_pending_request_manager *request_manager,
				       uint32_t id);

int rga_request_commit(struct rga_request *user_request);
void rga_request_session_destroy_abort(struct rga_session *session);
int rga_request_put(struct rga_request *request);
void rga_request_get(struct rga_request *request);
int rga_request_free(struct rga_request *request);
int rga_request_alloc(uint32_t flags, struct rga_session *session);

struct rga_request *rga_request_config(struct rga_user_request *user_request);
struct rga_request *rga_request_kernel_config(struct rga_user_request *user_request);
int rga_request_submit(struct rga_request *request);
int rga_request_mpi_submit(struct rga_req *req, struct rga_request *request);
int rga_request_release_signal(struct rga_scheduler_t *scheduler, struct rga_job *job);

int rga_request_manager_init(struct rga_pending_request_manager **request_manager_session);
int rga_request_manager_remove(struct rga_pending_request_manager **request_manager_session);

#endif /* __LINUX_RKRGA_JOB_H_ */
