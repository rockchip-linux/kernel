/*
 * Copyright (C) 2011-2014 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * AXD Commands API - Pipes/Buffers Accessing functions.
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "axd_api.h"
#include "axd_cmds.h"
#include "axd_cmds_internal.h"
#include "axd_hdr.h"
#include "axd_module.h"
#include "axd_platform.h"

/*
 * axd_pipe->eos_flg for output pipes is overloaded to mean two things:
 *
 * - EOS_REACHED: indicates that firmware has processed all input buffers
 *   including EOS but userland hasn't read them all yet.
 *
 * - EOF_REACHED: indicates that firmware sent EOS back to us AND userland read
 *   all the data till EOS.
 */
#define EOS_REACHED	1
#define EOF_REACHED	2

/*
 * axd_pipe->enabled_flg for output pipes is overloaded to mean two things:
 *
 * - PIPE_STARTED: indicates that pipe was opened but no buffers were passed.
 *   When stopping the pipes, we know that we don't need to discard anything if
 *   the discard_flg is set in cmd struct. Which allows us to terminate easily
 *   and quickly.
 *
 * - PIPE_RUNNING: indicates that pipe has processed some buffers, so we should
 *   discard if user terminates early (and discard_flg is set in cmd struct).
 */
#define PIPE_STARTED	1
#define PIPE_RUNNING	2

#ifdef CONFIG_AXD_DEBUG_DIAG
static unsigned int inSentCount[AXD_MAX_PIPES];
static unsigned int inRecvCount[AXD_MAX_PIPES];
static unsigned int outSentCount[AXD_MAX_PIPES];
static unsigned int outRecvCount[AXD_MAX_PIPES];
static unsigned int primeupCount[AXD_MAX_PIPES];
static unsigned int read_size[AXD_MAX_PIPES];
static unsigned int write_size[AXD_MAX_PIPES];
static unsigned int recv_size[AXD_MAX_PIPES];
#define debugdiag	printk
#else
#define debugdiag(format, ...)
#endif

static void axd_cmd_inpipe_clear(struct axd_cmd *cmd, unsigned int pipe);
static void axd_cmd_outpipe_clear(struct axd_cmd *cmd, unsigned int pipe);
static void axd_cmd_send_eos(struct axd_pipe *axd_pipe);
static void axd_output_prime_up(struct axd_pipe *axd_pipe);
static void axd_cmd_output_eos_reached(struct axd_cmd *cmd, unsigned int pipe);

/*
 * Send/Clear data{in, out} kicks.
 *
 * NOTE:
 * Must acquire axd_platform_lock() before accessing kick and interrupt status
 * registers as the AXD firmware might be accessing them at the same time.
 */
static inline void axd_datain_kick(struct axd_pipe *axd_pipe)
{
	unsigned long flags;
	struct axd_memory_map __iomem *message = axd_pipe->cmd->message;
	unsigned int pipe = axd_pipe->id;
	unsigned int temp;

#ifdef CONFIG_AXD_DEBUG_DIAG
	inSentCount[pipe]++;
#endif
	pr_debug("----> Send datain kick\n");
	flags = axd_platform_lock();
	temp = ioread32(&message->kick) |
				AXD_ANY_KICK_BIT | AXD_KICK_DATA_IN_BIT;
	iowrite32(temp, &message->kick);
	temp = ioread32(&message->in_kick_count[pipe]) + 1;
	iowrite32(temp, &message->in_kick_count[pipe]);
	axd_platform_unlock(flags);
	axd_platform_kick();
}

static inline void axd_dataout_kick(struct axd_pipe *axd_pipe)
{
	unsigned long flags;
	struct axd_memory_map __iomem *message = axd_pipe->cmd->message;
	unsigned int pipe = axd_pipe->id;
	unsigned int temp;

#ifdef CONFIG_AXD_DEBUG_DIAG
	outSentCount[pipe]++;
#endif
	pr_debug("----> Send dataout kick\n");
	flags = axd_platform_lock();
	temp = ioread32(&message->kick) |
				AXD_ANY_KICK_BIT | AXD_KICK_DATA_OUT_BIT;
	iowrite32(temp, &message->kick);
	temp = ioread32(&message->out_kick_count[pipe]) + 1;
	iowrite32(temp, &message->out_kick_count[pipe]);
	axd_platform_unlock(flags);
	axd_platform_kick();
}

/* Assumes axd_platform_lock() is already acquired before calling this */
static inline int axd_datain_status_clear(struct axd_pipe *axd_pipe)
{
	struct axd_memory_map __iomem *message = axd_pipe->cmd->message;
	unsigned int pipe = axd_pipe->id;
	unsigned int intcount = ioread32(&message->in_int_count[pipe]);

	pr_debug("Clearing in_int_count[%u] = %u\n", pipe, intcount);
	if (intcount == 0)
		return -1;
	atomic_add(intcount, &axd_pipe->intcount);
	iowrite32(0, &message->in_int_count[pipe]);
	return 0;
}

/* Assumes axd_platform_lock() is already acquired before calling this */
static inline int axd_dataout_status_clear(struct axd_pipe *axd_pipe)
{
	struct axd_memory_map __iomem *message = axd_pipe->cmd->message;
	unsigned int pipe = axd_pipe->id;
	unsigned int intcount = ioread32(&message->out_int_count[pipe]);

	pr_debug("Clearing out_int_count[%u] = %u\n", pipe, intcount);
	if (intcount == 0)
		return -1;
	atomic_add(intcount, &axd_pipe->intcount);
	iowrite32(0, &message->out_int_count[pipe]);
	return 0;
}

/* IRQ Handler */
static irqreturn_t axd_irq(int irq, void *data)
{
	struct axd_cmd *cmd = data;
	unsigned int int_status;
	unsigned long flags;
	int i, ret;

	/*
	 * int_status is ioremapped() which means it could page fault. When axd
	 * is running on the same core as the host, holding lock2 would disable
	 * exception handling in that core which means a page fault would stuff
	 * host thread executing the driver. We do a double read here to ensure
	 * that we stall until the memory access is done before lock2 is
	 * acquired, hence ensuring that any page fault is handled outside lock2
	 * region.
	*/
	int_status = ioread32(&cmd->message->int_status);
	int_status = ioread32(&cmd->message->int_status);

	axd_platform_irq_ack();
	flags = axd_platform_lock();
	int_status = ioread32(&cmd->message->int_status);
	iowrite32(0, &cmd->message->int_status);

	if (!int_status)
		goto out;

	pr_debug("<---- Received int_status = 0x%08X\n", int_status);
	if (int_status & AXD_INT_KICK_DONE)
		pr_debug("<---- Received kick done interrupt\n");
	if (int_status & AXD_INT_DATAIN) {
		pr_debug("<---- Received datain interrupt\n");
		for (i = 0; i < AXD_MAX_PIPES; i++) {
			struct axd_pipe *axd_pipe = &cmd->in_pipes[i];

			if (axd_get_flag(&axd_pipe->enabled_flg)) {
				ret = axd_datain_status_clear(axd_pipe);
				if (!ret)
					queue_work(cmd->in_workq, &axd_pipe->work);
			}
		}
	}
	if (int_status & AXD_INT_DATAOUT) {
		pr_debug("<---- Received dataout interrupt\n");
		for (i = 0; i < AXD_MAX_PIPES; i++) {
			struct axd_pipe *axd_pipe = &cmd->out_pipes[i];

			if (axd_get_flag(&axd_pipe->enabled_flg)) {
				ret = axd_dataout_status_clear(axd_pipe);
				if (!ret && !axd_get_flag(&axd_pipe->eos_flg))
					queue_work(cmd->out_workq, &axd_pipe->work);
			}
		}
	}
	if (int_status & AXD_INT_CTRL) {
		pr_debug("<---- Received ctrl interrupt\n");
		axd_set_flag(&cmd->response_flg, 1);
		wake_up(&cmd->wait);
	}
	if (int_status & AXD_INT_ERROR) {
		struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
		int error = ioread32(&cmd->message->error);

		pr_debug("<---- Received error interrupt\n");
		switch (error) {
		default:
		case 0:
			break;
		case 1:
			dev_err(axd->dev, "Fatal error received...\n");
			axd_schedule_reset(cmd);
			break;
		case 2:
			dev_warn(axd->dev, "Failed to set last configuration command\n");
			break;
		}

		iowrite32(0, &cmd->message->error);
	}
out:
	/*
	 * ensure all writes to uncached shared memory are visible to AXD
	 * before releasing axd_platform_lock()
	 */
	wmb();
	axd_platform_unlock(flags);
	return IRQ_HANDLED;
}

/*
 * Initialize the drivers descriptors control structre.
 * @desc_ctrl: the desc control structure to initialize.
 * @buf_desc: pointer to the buffer descriptor to control.
 * @num_desc: total number of descriptors inside @buf_desc.
 */
static void desc_init(struct axd_desc_ctrl *desc_ctrl,
		struct axd_buffer_desc __iomem *buf_desc, unsigned int num_desc)
{
	/* Reset ctrl desc struct */
	desc_ctrl->rd_idx = 0;
	desc_ctrl->wr_idx = 0;
	sema_init(&desc_ctrl->rd_sem, num_desc);
	sema_init(&desc_ctrl->wr_sem, 0);
	spin_lock_init(&desc_ctrl->rd_lock);
	spin_lock_init(&desc_ctrl->wr_lock);
	desc_ctrl->buf_desc = buf_desc;
	desc_ctrl->num_desc = num_desc;
}

/*
 * Prepare a descriptor to be sent to firmware.
 * @desc_ctrl: the control structure of the descriptor.
 * @buf: physical address of the buffer to enqueue.
 * @size: size of the buffer.
 * @last: non-zero of this is the last buffer ie: EOS.
 */
static int desc_enqueue(struct axd_desc_ctrl *desc_ctrl, char *buf,
			unsigned int size, int last, struct axd_pipe *chan)
{
	struct axd_buffer_desc __iomem *buf_desc = desc_ctrl->buf_desc;
	unsigned int num_desc = desc_ctrl->num_desc;
	unsigned int status_size = size | AXD_DESCRIPTOR_READY_BIT;
	int ret;

	pr_debug("Enqueuing a descriptor, pipe[%u]... ", chan->id);
	/* only proceed if we're not full */
	ret = down_trylock(&desc_ctrl->rd_sem);
	if (ret) {
		pr_debug("FAILED - full\n");
		return -1;
	}
	pr_debug("SUCCEEDED\n");

	if (last)
		status_size |= AXD_DESCRIPTOR_EOS_BIT;

	/*
	 * if we could lock the semaphore, then we're guaranteed that the
	 * current rd_idx is valid and ready to be used. So no need to verify
	 * that the status of the descriptor at rd_idx is valid.
	 */
	spin_lock(&desc_ctrl->rd_lock);
	iowrite32(status_size, &buf_desc[desc_ctrl->rd_idx].status_size);
	iowrite32((unsigned int)axd_io_2_phys(buf),
					&buf_desc[desc_ctrl->rd_idx].data_ptr);
	iowrite32(chan->current_ts_high, &buf_desc[desc_ctrl->rd_idx].pts_high);
	iowrite32(chan->current_ts_low, &buf_desc[desc_ctrl->rd_idx].pts_low);
	desc_ctrl->rd_idx++;
	if (desc_ctrl->rd_idx >= num_desc)
		desc_ctrl->rd_idx = 0;
	spin_unlock(&desc_ctrl->rd_lock);
	up(&desc_ctrl->wr_sem); /* we can dequeue 1 more item */
	return 0;
}

/*
 * Takes a buffer out of the descriptor queue.
 * @desc_ctrl: the control structure of the descriptor.
 * @size: sets it tot he size of the buffer returned if not NULL.
 * @last: sets it to non-zero of this is the last buffer ie: EOS (if last is not
 * NULL)
 *
 * On success, a valid pointer is received. NULL otherwise.
 */
static char *desc_dequeue(struct axd_desc_ctrl *desc_ctrl, unsigned int *size,
			  int *last, struct axd_pipe *chan, int is_out)
{
	struct axd_buffer_desc __iomem *buf_desc = desc_ctrl->buf_desc;
	unsigned int num_desc = desc_ctrl->num_desc;
	unsigned int status_size;
	char *buf;
	int ret;

	pr_debug("Dequeuing a descriptor, pipe[%u]... ", chan->id);
	/* only proceed if we're not empty */
	ret = down_trylock(&desc_ctrl->wr_sem);
	if (ret) {
		pr_debug("FAILED - empty\n");
		return NULL;
	}
	spin_lock(&desc_ctrl->wr_lock);
	status_size = ioread32(&buf_desc[desc_ctrl->wr_idx].status_size);
	/*
	 * if ready and in_use bit are set, then the rest of the buffers are
	 * still owned by the AXD fw, we can't dequeue them then. exit.
	 */
	if ((status_size & AXD_DESCRIPTOR_INUSE_BIT) &&
		!(status_size & AXD_DESCRIPTOR_READY_BIT)) {

		pr_debug("SUCCEEDED\n");
		/* clear the in_use bit */
		iowrite32(status_size & ~AXD_DESCRIPTOR_INUSE_BIT,
				&buf_desc[desc_ctrl->wr_idx].status_size);

		/*
		 * Return the pointer to the buffer and its size to caller.
		 * The caller might need to read it or return it to the pool.
		 */
		buf = (char *)ioread32(&buf_desc[desc_ctrl->wr_idx].data_ptr);
		if (size)
			*size = status_size & AXD_DESCRIPTOR_SIZE_MASK;
		if (last)
			*last = status_size & AXD_DESCRIPTOR_EOS_BIT;

		if (is_out) {
			/* update any timestamps if is use */
			chan->current_ts_high =
				ioread32(&buf_desc[desc_ctrl->wr_idx].pts_high);
			chan->current_ts_low =
				ioread32(&buf_desc[desc_ctrl->wr_idx].pts_low);
		}

		desc_ctrl->wr_idx++;
		if (desc_ctrl->wr_idx >= num_desc)
			desc_ctrl->wr_idx = 0;

		spin_unlock(&desc_ctrl->wr_lock);
		up(&desc_ctrl->rd_sem); /* we can enqueue 1 more item */
		return axd_phys_2_io(buf);
	}
	pr_debug("FAILED - AXD holds the rest of the descriptors\n");
	/*
	 * failed due to busy buffer, return writer locks
	 * as we haven't dequeued
	 */
	spin_unlock(&desc_ctrl->wr_lock);
	up(&desc_ctrl->wr_sem);
	return NULL;
}

/*
 * This is the function executed by the workqueue to process return input
 * pipes descriptors.
 * Each pipe will have its own version of this function executed when datain
 * interrupt is received.
 */
static void in_desc_workq(struct work_struct *work)
{
	struct axd_pipe *axd_pipe = container_of(work, struct axd_pipe, work);
	struct axd_bufferq *desc_bufferq = &axd_pipe->desc_bufferq;
	struct axd_desc_ctrl *desc_ctrl = &axd_pipe->desc_ctrl;
	struct axd_cmd *cmd = axd_pipe->cmd;
	unsigned int pipe = axd_pipe->id;
	char *ret_buf;
	int ret, last;

	pr_debug("*** Processing datain[%u] buffer ***\n", pipe);
	do { /* we should have at least 1 desc to process */
		ret_buf = desc_dequeue(desc_ctrl, NULL, &last, axd_pipe, 0);
		if (!ret_buf)
			/*
			 * This could happen if an interrupt occurs while this
			 * work is already running, causing us to run twice in a
			 * row unnecessarily. Not harmful, so just return.
			 */
			return;
#ifdef CONFIG_AXD_DEBUG_DIAG
		inRecvCount[pipe]++;
#endif
		ret = axd_bufferq_put(desc_bufferq, ret_buf, -1);
		if (ret)
			return;
		if (last) {
			pr_debug("Received input[%u] EOS\n", pipe);
			debugdiag("inSentCount[%u]= %u, inRecvCount[%u]= %u, write_size[%u]= %u\n",
				pipe, inSentCount[pipe],
				pipe, inRecvCount[pipe],
				pipe, write_size[pipe]);
			axd_cmd_inpipe_clear(cmd, pipe);
		}
	} while (!atomic_dec_and_test(&axd_pipe->intcount));

	/* Do we need to send EOS? */
	if (axd_get_flag(&axd_pipe->eos_flg))
		axd_cmd_send_eos(axd_pipe);
}

/*
 * This is the function executed by the workqueue to process return output
 * pipes descriptors.
 * Each pipe will have its own version of this function executed when dataout
 * interrupt is received.
 */
static void out_desc_workq(struct work_struct *work)
{
	struct axd_pipe *axd_pipe = container_of(work, struct axd_pipe, work);
	struct axd_bufferq *desc_bufferq = &axd_pipe->desc_bufferq;
	struct axd_bufferq *user_bufferq = &axd_pipe->user_bufferq;
	struct axd_desc_ctrl *desc_ctrl = &axd_pipe->desc_ctrl;
	char *ret_buf;
	unsigned int buf_size;
	int ret, last;

	pr_debug("*** Processing dataout[%u] buffer ***\n", axd_pipe->id);
	do { /* we should have at least 1 desc to process */
		ret_buf = desc_dequeue(desc_ctrl,
						&buf_size, &last, axd_pipe, 1);
		if (!ret_buf || axd_get_flag(&axd_pipe->eos_flg)) {
			/*
			 * This could happen if an interrupt occurs while this
			 * work is already running, causing us to run twice in a
			 * row unnecessarily. Not harmful, so just return.
			 *
			 * OR if we prime up the output bufferq a tad too much
			 * we could end up with extra buffers after eos is
			 * reached, in this case we shouldn't process these
			 * extra buffers and just return.
			 */
			return;
		}
#ifdef CONFIG_AXD_DEBUG_DIAG
		outRecvCount[axd_pipe->id]++;
		recv_size[axd_pipe->id] += buf_size;
#endif
		if (likely(!axd_get_flag(&axd_pipe->discard_flg))) {
			if (last) {
				pr_debug("Received output[%u] EOS\n",
								axd_pipe->id);

				axd_set_flag(&axd_pipe->eos_flg, EOS_REACHED);
			}
			ret = axd_bufferq_put(user_bufferq, ret_buf, buf_size);
			if (ret)
				return;
		} else { /* drop all buffers until EOS is reached */
			if (last) {
				pr_debug("Received output[%u] EOS - discard\n",
								axd_pipe->id);
				axd_set_flag(&axd_pipe->eos_flg, EOS_REACHED);
				axd_cmd_output_eos_reached(axd_pipe->cmd,
								axd_pipe->id);
				return;
			}
			ret = axd_bufferq_put(desc_bufferq, ret_buf, -1);
			if (ret)
				return;
			axd_output_prime_up(axd_pipe);
		}
	} while (!atomic_dec_and_test(&axd_pipe->intcount));
}

/* Send a stream flush command to firmware */
static int axd_flush_input_stream(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	struct axd_memory_map __iomem *message = cmd->message;
	struct mutex *cm_lock = &cmd->cm_lock;
	int ret;

	mutex_lock(cm_lock);
	if (axd_get_flag(&cmd->fw_stopped_flg)) {
		mutex_unlock(cm_lock);
		return -1;
	}
	axd_set_flag(&cmd->response_flg, 0);
	iowrite32(AXD_CTRL_CMD_FLUSH, &message->control_command);
	iowrite32(pipe, &message->control_data);
	axd_ctrl_kick(message);
	ret = wait_event_timeout(cmd->wait,
			axd_get_flag(&cmd->response_flg) != 0, CMD_TIMEOUT);
	mutex_unlock(cm_lock);
	if (!ret) {
		dev_warn(axd->inputdev[pipe], "failed to flush input stream\n");
		return -1;
	}
	return 0;
}

static int axd_flush_output_stream(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	struct axd_memory_map __iomem *message = cmd->message;
	struct mutex *cm_lock = &cmd->cm_lock;
	int ret;

	mutex_lock(cm_lock);
	if (axd_get_flag(&cmd->fw_stopped_flg)) {
		mutex_unlock(cm_lock);
		return -1;
	}
	axd_set_flag(&cmd->response_flg, 0);
	iowrite32(AXD_CTRL_CMD_FLUSH, &message->control_command);
	iowrite32(pipe + AXD_MAX_PIPES, &message->control_data);
	axd_ctrl_kick(message);
	ret = wait_event_timeout(cmd->wait,
			axd_get_flag(&cmd->response_flg) != 0, CMD_TIMEOUT);
	mutex_unlock(cm_lock);
	if (!ret) {
		dev_warn(axd->outputdev[pipe], "failed to flush output stream\n");
		return -1;
	}
	return 0;
}

/* Send a reset buffer descriptor commands to firmware - input */
static int axd_reset_input_bd(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	struct axd_memory_map __iomem *message = cmd->message;
	struct mutex *cm_lock = &cmd->cm_lock;
	int ret;

	mutex_lock(cm_lock);
	if (axd_get_flag(&cmd->fw_stopped_flg)) {
		mutex_unlock(cm_lock);
		return -1;
	}
	axd_set_flag(&cmd->response_flg, 0);
	iowrite32(AXD_CTRL_CMD_RESET_BD, &message->control_command);
	iowrite32(pipe, &message->control_data);
	axd_ctrl_kick(message);
	ret = wait_event_timeout(cmd->wait,
			axd_get_flag(&cmd->response_flg) != 0, CMD_TIMEOUT);
	mutex_unlock(cm_lock);
	if (!ret) {
		dev_warn(axd->inputdev[pipe], "failed to reset input buffer descriptors\n");
		return -1;
	}
	return 0;
}

/* Send a reset buffer descriptor commands to firmware - output */
static int axd_reset_output_bd(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	struct axd_memory_map __iomem *message = cmd->message;
	struct mutex *cm_lock = &cmd->cm_lock;
	int ret;

	mutex_lock(cm_lock);
	if (axd_get_flag(&cmd->fw_stopped_flg)) {
		mutex_unlock(cm_lock);
		return -1;
	}
	axd_set_flag(&cmd->response_flg, 0);
	iowrite32(AXD_CTRL_CMD_RESET_BD, &message->control_command);
	iowrite32(pipe + AXD_MAX_PIPES, &message->control_data);
	axd_ctrl_kick(message);
	ret = wait_event_timeout(cmd->wait,
			axd_get_flag(&cmd->response_flg) != 0, CMD_TIMEOUT);
	mutex_unlock(cm_lock);
	if (!ret) {
		dev_warn(axd->outputdev[pipe], "failed to reset output buffer descriptors\n");
		return -1;
	}
	return 0;
}
/* Send a reset pipe command to the firmware */
int axd_cmd_reset_pipe(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	struct axd_memory_map __iomem *message = cmd->message;
	struct mutex *cm_lock = &cmd->cm_lock;
	int ret;

	mutex_lock(cm_lock);
	if (axd_get_flag(&cmd->fw_stopped_flg)) {
		mutex_unlock(cm_lock);
		return -1;
	}
	axd_set_flag(&cmd->response_flg, 0);
	iowrite32(AXD_CTRL_CMD_RESET_PIPE, &message->control_command);
	iowrite32(pipe, &message->control_data);
	axd_ctrl_kick(message);
	ret = wait_event_timeout(cmd->wait,
			axd_get_flag(&cmd->response_flg) != 0, CMD_TIMEOUT);
	mutex_unlock(cm_lock);
	if (!ret) {
		dev_warn(axd->ctrldev[0], "failed to reset pipe%d", pipe);
		return -1;
	}
	return 0;
}

/* Sends a dummy buffer indicating EOS to a pipe */
static void axd_cmd_send_eos(struct axd_pipe *axd_pipe)
{
	struct axd_dev *axd = container_of(axd_pipe->cmd, struct axd_dev, cmd);
	struct axd_bufferq *desc_bufferq = &axd_pipe->desc_bufferq;
	struct axd_desc_ctrl *desc_ctrl = &axd_pipe->desc_ctrl;
	int ret;
	char *p;

	mutex_lock(&axd_pipe->eos_mutex);
	/*
	 * If eos is cleared, then a previous call successfully sent it, nothing
	 * to do then, so exit.
	 */
	if (!axd_get_flag(&axd_pipe->eos_flg))
		goto out;

	/* Only proceed if we know a buffer is available, don't block */
	if (axd_bufferq_is_empty(desc_bufferq))
		goto out;
	p = axd_bufferq_take(desc_bufferq, NULL);
	if (unlikely(IS_ERR(p)))
		goto out;
	ret = desc_enqueue(desc_ctrl, p, 0, 1, axd_pipe);
	if (unlikely(ret)) {
		/* shouldn't happen, print a warning */
		dev_warn(axd->inputdev[axd_pipe->id], "Warning, failed to enqueue buffer\n");
		goto out;
	}
	/* enqueued successfully, inform the axd firmware */
	axd_datain_kick(axd_pipe);
	pr_debug("Sent input[%u] EOS\n", axd_pipe->id);
	/*
	 * clear if eos sent successfully
	 */
	axd_set_flag(&axd_pipe->eos_flg, 0);
out:
	mutex_unlock(&axd_pipe->eos_mutex);
}

/*
 * Send as many buffers to the output pipe as possible.
 * Keeping the firmware output buffer primed up prevents the firmware from
 * getting deprived of buffers to fill.
 */
static void axd_output_prime_up(struct axd_pipe *axd_pipe)
{
	struct axd_bufferq *desc_bufferq = &axd_pipe->desc_bufferq;
	struct axd_desc_ctrl *desc_ctrl = &axd_pipe->desc_ctrl;
	unsigned int stride;
	char *p;
	int ret;

	/*
	 * Try not to send too much. Make sure to stop as soon as we receive
	 * EOS.
	 */
	if (axd_get_flag(&axd_pipe->eos_flg))
		return;

	/* prime up the output buffer as much as we can */
	while (!axd_bufferq_is_empty(desc_bufferq)) {
#ifdef CONFIG_AXD_DEBUG_DIAG
		primeupCount[axd_pipe->id]++;
#endif
		p = axd_bufferq_take(desc_bufferq, &stride);
		if (IS_ERR(p))
			break;
		ret = desc_enqueue(desc_ctrl, p, stride, 0, axd_pipe);
		if (ret) {
			/*
			 * error, return the buffer to the pool
			 */
			axd_bufferq_put(desc_bufferq, p, -1);
			break;
		}
		/* inform axd firmware */
		axd_dataout_kick(axd_pipe);
	}
}

/* Exported functions */
/* Initialize the input pipe structure */
void axd_cmd_inpipe_init(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->in_pipes[pipe];

	axd_pipe->cmd = cmd;
	axd_pipe->buf_desc = cmd->message->input[pipe].descriptors;
	axd_pipe->id = pipe;

	axd_set_flag(&axd_pipe->enabled_flg, 0);
	axd_set_flag(&axd_pipe->eos_flg, 0);
	mutex_init(&axd_pipe->eos_mutex);
	atomic_set(&axd_pipe->intcount, 0);

	/* default buffer size, could be changed through sysfs */
	axd_pipe->buf_size = 1024*2;
}

/* Initialize the output pipe structure */
void axd_cmd_outpipe_init(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->out_pipes[pipe];

	axd_pipe->cmd = cmd;
	axd_pipe->buf_desc = cmd->message->output[pipe].descriptors;
	axd_pipe->id = pipe;

	axd_set_flag(&axd_pipe->discard_flg, 0);
	axd_set_flag(&axd_pipe->enabled_flg, 0);
	axd_set_flag(&axd_pipe->eos_flg, 0);
	atomic_set(&axd_pipe->intcount, 0);

	/* default buffer size, could be changed through sysfs */
	axd_pipe->buf_size = 1024*16;
}

/* Set up the IRQ handler and workqueues */
int axd_cmd_install_irq(struct axd_cmd *cmd, unsigned int irqnum)
{
	int i;

	cmd->in_workq = create_workqueue("axd_din_q");
	if (!cmd->in_workq)
		return -ENOMEM;
	for (i = 0; i < AXD_MAX_PIPES; i++)
		INIT_WORK(&cmd->in_pipes[i].work, in_desc_workq);
	cmd->out_workq = create_workqueue("axd_dout_q");
	if (!cmd->out_workq) {
		destroy_workqueue(cmd->in_workq);
		return -ENOMEM;
	}
	for (i = 0; i < AXD_MAX_PIPES; i++)
		INIT_WORK(&cmd->out_pipes[i].work, out_desc_workq);
	iowrite32(AXD_INT_KICK_DONE, &cmd->message->int_mask);
	return request_irq(irqnum, axd_irq, 0, "axd_irq", cmd);
}

void axd_cmd_free_irq(struct axd_cmd *cmd, unsigned int irqnum)
{
	flush_workqueue(cmd->in_workq);
	destroy_workqueue(cmd->in_workq);
	flush_workqueue(cmd->out_workq);
	destroy_workqueue(cmd->out_workq);
	free_irq(irqnum, cmd);
}

/*
 * Calculate the starting address of input pipe's buffers based on the
 * information provided in firmware's header
 */
static char *axd_inpipe_datain_address(struct axd_cmd *cmd, unsigned int pipe,
						unsigned int *num_avail_buffers)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	struct axd_pipe *axd_pipe = &cmd->in_pipes[pipe];
	unsigned long base_address = axd_cmd_get_datain_address(cmd);
	unsigned long total_size = axd_cmd_get_datain_size(cmd);
	unsigned long num_desc, offset;

	/*
	 * Based on the defined axd_pipe->buf_size and number of input pipes
	 * supported by the firmware, we calculate the number of descriptors we
	 * need to use using this formula:
	 *
	 *	axd_pipe->buf_size * num_desc = total_size / num_inputs
	 */
	num_desc = total_size / (cmd->num_inputs * axd_pipe->buf_size);
	if (num_desc > AXD_INPUT_DESCRIPTORS) {
		num_desc = AXD_INPUT_DESCRIPTORS;
	} else if (num_desc == 0) {
		dev_err(axd->inputdev[pipe],
			"Error: input buffer element size is too large\n");
		return NULL;
	}
	offset = (total_size / cmd->num_inputs) * pipe;
	if (num_avail_buffers)
		*num_avail_buffers = num_desc;

	return (char *)(base_address + offset);
}

static int axd_cmd_inpipe_buffers_init(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->in_pipes[pipe];
	struct axd_desc_ctrl *desc_ctrl = &axd_pipe->desc_ctrl;
	struct axd_buffer_desc __iomem *in_buf_desc = axd_pipe->buf_desc;
	unsigned int num_avail_buffers;
	char bufname[16];
	int ret;

	char *buf_address = axd_inpipe_datain_address(cmd, pipe,
							&num_avail_buffers);
	if (!buf_address)
		return -EIO;

	/* initialize descriptors & control semaphores/locks */
	desc_init(desc_ctrl, in_buf_desc, AXD_INPUT_DESCRIPTORS);

	/* initialize buffers */
	sprintf(bufname, "in_bufferq[%u]", pipe);
	ret = axd_bufferq_init(&axd_pipe->desc_bufferq, bufname, buf_address,
			num_avail_buffers, axd_pipe->buf_size, cmd->nonblock);
	return ret;
}

/* prepare inpipe for processing data */
static int axd_cmd_inpipe_prepare(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->in_pipes[pipe];
	int ret;

	ret = axd_cmd_inpipe_buffers_init(cmd, pipe);
	if (ret)
		return ret;

	atomic_set(&axd_pipe->intcount, 0);
	axd_set_flag(&axd_pipe->enabled_flg, PIPE_STARTED);
	if (axd_reset_input_bd(cmd, pipe))
		goto out;
	if (axd_flush_input_stream(cmd, pipe))
		goto out;
	if (axd_cmd_input_set_enabled(cmd, pipe, 1))
		goto out;

	/* Set PTS values for streams received without sync data */
	axd_pipe->current_ts_high = -1;
	axd_pipe->current_ts_low = -1;

	return 0;
out:
	axd_set_flag(&axd_pipe->enabled_flg, 0);
	return -EIO;
}

/* Start processing data on input pipe @pipe */
int axd_cmd_inpipe_start(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->in_pipes[pipe];
	int ret;

	/*
	 * If enabled is locked, it means that the firmware is still busy
	 * processing buffers until EOS is reached. Tell to try again shortly.
	 */
	if (axd_get_flag(&axd_pipe->enabled_flg))
		return -EAGAIN;

	pr_debug("Starting input[%u]\n", pipe);
	ret = axd_cmd_inpipe_prepare(cmd, pipe);
	if (ret)
		return ret;
	axd_pipe->tsk = current;
#ifdef CONFIG_AXD_DEBUG_DIAG
	inSentCount[pipe] = 0;
	inRecvCount[pipe] = 0;
	write_size[pipe] = 0;
#endif
	return 0;
}

/* Stop processing data on input pipe @pipe */
void axd_cmd_inpipe_stop(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->in_pipes[pipe];

	pr_debug("Stopping input[%u]\n", pipe);
	/*
	 * If we haven't sent any data to the firmware, then clear ourselves
	 * immediately without having to send EOS which could never return.
	 */
	if (axd_get_flag(&axd_pipe->discard_flg)) {
		/*
		 * Setting eos indicates that an eos buffer need to be sent. In
		 * some cases (ie: error occurs in the application), the buffer
		 * queue would be full and eos would fail to send. When an
		 * interrupt is received then and a buffer becomes free, we
		 * send eos buffer if the eos flag is set.
		 */
		axd_set_flag(&axd_pipe->eos_flg, EOS_REACHED);
		axd_cmd_send_eos(axd_pipe);
	} else {
		axd_cmd_inpipe_clear(cmd, pipe);
	}
	axd_pipe->tsk = NULL;
}

/* clears input pipe so that it can be prepared to start again */
static void axd_cmd_inpipe_clear(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->in_pipes[pipe];

	/* disable input and clear buffers */
	axd_cmd_input_set_enabled(cmd, pipe, 0);
	axd_bufferq_clear(&axd_pipe->desc_bufferq);
	/*
	 * NOTE: disabling the enabled flag must be done at the end to make sure
	 * that the input device can't be opened again before everything else is
	 * cleared up properly. There was a race where setting enabled to 0
	 * before clearing bufferq caused a crash as the device could be opened
	 * after the flag is disabled but before the bufferq is cleared so the
	 * bufferq would be setup then cleared again causing wrong memory access
	 * later when reading.
	 */
	axd_set_flag(&axd_pipe->enabled_flg, 0);
	axd_set_flag(&axd_pipe->discard_flg, 0);
}

/* Reset input pipe to starting state - for error recovery */
void axd_cmd_inpipe_reset(struct axd_cmd *cmd, unsigned int pipe)
{
	axd_cmd_inpipe_clear(cmd, pipe);
}

/* Is the input pipe active? */
int axd_cmd_inpipe_active(struct axd_cmd *cmd, unsigned int pipe)
{
	int state = axd_get_flag(&cmd->in_pipes[pipe].enabled_flg);
	return state == PIPE_STARTED || state == PIPE_RUNNING;
}

/*
 * Calculate the starting address of output pipe's buffers based on the
 * information provided in firmware's header
 */
static char *axd_outpipe_dataout_address(struct axd_cmd *cmd, unsigned int pipe,
						unsigned int *num_avail_buffers)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	struct axd_pipe *axd_pipe = &cmd->out_pipes[pipe];
	unsigned long base_address = axd_cmd_get_dataout_address(cmd);
	unsigned long total_size = axd_cmd_get_dataout_size(cmd);
	unsigned long num_desc, offset;

	/*
	 * Based on the defined axd_pipe->buf_size and number of output pipes
	 * supported by the firmware, we calculate the number of descriptors we
	 * need to use using this formula:
	 *
	 *	axd_pipe->buf_size * num_desc = total_size / num_outputs
	 */
	num_desc = total_size / (cmd->num_outputs * axd_pipe->buf_size);
	if (num_desc > AXD_OUTPUT_DESCRIPTORS) {
		num_desc = AXD_OUTPUT_DESCRIPTORS;
	} else if (num_desc == 0) {
		dev_err(axd->outputdev[pipe], "Error: output buffer element size is too large\n");
		return NULL;
	}
	offset = (total_size / cmd->num_outputs) * pipe;
	if (num_avail_buffers)
		*num_avail_buffers = num_desc;

	return (char *)(base_address + offset);
}

static int axd_cmd_outpipe_buffers_init(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->out_pipes[pipe];
	struct axd_desc_ctrl *desc_ctrl = &axd_pipe->desc_ctrl;
	struct axd_buffer_desc __iomem *out_buf_desc = axd_pipe->buf_desc;
	unsigned int num_avail_buffers;
	char bufname[16];
	int ret;

	char *buf_address = axd_outpipe_dataout_address(cmd, pipe,
							&num_avail_buffers);
	if (!buf_address)
		return -EIO;

	/* initialise descriptors & control semaphores/locks */
	desc_init(desc_ctrl, out_buf_desc, AXD_OUTPUT_DESCRIPTORS);
	/* intialise buffers */
	sprintf(bufname, "out_bufferq[%u]", pipe);
	ret = axd_bufferq_init(&axd_pipe->desc_bufferq,
			bufname, buf_address,
			num_avail_buffers, axd_pipe->buf_size,
			cmd->nonblock);
	if (ret)
		return ret;
	sprintf(bufname, "user_bufferq[%u]", pipe);
	ret = axd_bufferq_init_empty(&axd_pipe->user_bufferq,
				bufname, num_avail_buffers,
				axd_pipe->buf_size, cmd->nonblock);
	if (ret) {
		axd_bufferq_clear(&axd_pipe->desc_bufferq);
		return ret;
	}

	return ret;
}

/* prepare outpipe for processing data */
static int axd_cmd_outpipe_prepare(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->out_pipes[pipe];
	int ret;

	ret = axd_cmd_outpipe_buffers_init(cmd, pipe);
	if (ret)
		return ret;

	atomic_set(&axd_pipe->intcount, 0);
	axd_set_flag(&axd_pipe->enabled_flg, PIPE_STARTED);
	axd_set_flag(&axd_pipe->eos_flg, 0);
	if (axd_reset_output_bd(cmd, pipe))
		goto out;
	if (axd_cmd_output_set_enabled(cmd, pipe, 1))
		goto out;
	return 0;
out:
	axd_set_flag(&axd_pipe->enabled_flg, 0);
	axd_set_flag(&axd_pipe->eos_flg, EOF_REACHED);
	return -EIO;
}

/* Start processing data on output pipe @pipe */
int axd_cmd_outpipe_start(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->out_pipes[pipe];
	int ret;

	pr_debug("Starting output[%u]\n", pipe);
	/*
	 * Fully initialise only if enabled is unlocked.
	 * If enabled is locked, it means someone opened the device then
	 * closed it before reaching EOS. In this case, re-enable output to
	 * continue reading from where we stopped.
	 */
	if (!axd_get_flag(&axd_pipe->enabled_flg)) {
		ret = axd_cmd_outpipe_prepare(cmd, pipe);
		if (ret)
			return ret;
	} else if (axd_get_flag(&axd_pipe->discard_flg)) {
		/*
		 * we're still discarding some data from a previous call to
		 * stop, tell the user to try again shortly
		 */
		return -EAGAIN;
	}
	axd_pipe->tsk = current;
#ifdef CONFIG_AXD_DEBUG_DIAG
	outSentCount[pipe] = 0;
	outRecvCount[pipe] = 0;
	primeupCount[pipe] = 0;
	read_size[pipe] = 0;
	recv_size[pipe] = 0;
#endif
	return 0;
}

/* Stop processing data on output pipe @pipe */
void axd_cmd_outpipe_stop(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->out_pipes[pipe];
	struct axd_bufferq *desc_bufferq = &axd_pipe->desc_bufferq;
	struct axd_bufferq *user_bufferq = &axd_pipe->user_bufferq;
	char *ret_buf;

	pr_debug("Stopping output[%u]\n", pipe);
	axd_pipe->tsk = NULL;
	if (axd_get_flag(&cmd->discard_flg) &&
					axd_get_flag(&axd_pipe->enabled_flg)) {
		/* Is there anything to discard? */
		if (axd_get_flag(&axd_pipe->enabled_flg) == PIPE_STARTED) {
			/*
			 * nothing to clear up too, just disable the input so
			 * we'd initialise ourselves properly again on next
			 * start.
			 */
			axd_set_flag(&axd_pipe->enabled_flg, 0);
			return;
		}
		axd_set_flag(&axd_pipe->discard_flg, 1);

		if (axd_pipe->cur_buf)
			axd_bufferq_put(desc_bufferq, axd_pipe->cur_buf, -1);

		while (!axd_bufferq_is_empty(user_bufferq)) {
			ret_buf = axd_bufferq_take(user_bufferq, NULL);
			axd_bufferq_put(desc_bufferq, ret_buf, -1);
		}

		if (axd_get_flag(&axd_pipe->eos_flg) == EOS_REACHED) {
			axd_cmd_output_eos_reached(cmd, pipe);
			return;
		}

		axd_output_prime_up(axd_pipe);

	}

}

static void axd_cmd_outpipe_clear(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->out_pipes[pipe];
	/*
	 * unlock enabled to fully intialise next time we're
	 * opened.
	 */
	axd_flush_output_stream(cmd, pipe);
	axd_bufferq_clear(&axd_pipe->desc_bufferq);
	axd_bufferq_clear(&axd_pipe->user_bufferq);
	axd_cmd_output_set_enabled(cmd, pipe, 0);
	axd_set_flag(&axd_pipe->enabled_flg, 0);
	axd_set_flag(&axd_pipe->discard_flg, 0);
	axd_pipe->cur_buf = NULL;
	axd_pipe->cur_buf_size = 0;
	axd_pipe->cur_buf_offset = 0;
}

/* Reset output pipe to starting state - for error recovery */
void axd_cmd_outpipe_reset(struct axd_cmd *cmd, unsigned int pipe)
{
	axd_cmd_outpipe_clear(cmd, pipe);
}

/*
 * Send a buffer to input @pipe
 *
 * Returns number of bytes sent, or negative error number.
 */
int axd_cmd_send_buffer(struct axd_cmd *cmd, unsigned int pipe,
				const char __user *buf, unsigned int size)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	struct axd_pipe *axd_pipe = &cmd->in_pipes[pipe];
	struct axd_bufferq *desc_bufferq = &axd_pipe->desc_bufferq;
	struct axd_desc_ctrl *desc_ctrl = &axd_pipe->desc_ctrl;
	unsigned int stride;
	int ret = 0;
	int written = 0;
	int diff;
	unsigned int cp_size;
	char *p;

	/*
	 * Before if we had no data buffer sent to the firmware EOS flag was
	 * sent perfect through, but now we shouldn't send EOS flag if
	 * no data was sent to the firmware. We use the discard variable to
	 * flag if we need to send the EOS at stop or not.
	 * see axd_cmd_inpipe_stop()
	 * NOTE: discard_flg for input pipe is different than discard_flg for
	 * output pipe.
	 */
	if (unlikely(!axd_get_flag(&axd_pipe->discard_flg)))
		axd_set_flag(&axd_pipe->discard_flg, 1);

	pr_debug("Writing %u bytes [%u]\n", size, pipe);
	while (written < size) {
		/*
		 * There's a one to one mapping between the desc buffers and the
		 * descriptors owned by the driver. If the descriptors are
		 * empty, we'll sleep in here and when we wake up/proceed we are
		 * guaranteed that we will enqueue a descriptor successfully
		 */
		p = axd_bufferq_take(desc_bufferq, &stride);
		if (IS_ERR(p)) {
			ret = PTR_ERR(p);
			goto out;
		}
		diff = size - written;
		cp_size = diff < stride ? diff : stride;
		ret = copy_from_user(p, buf, cp_size);
		if (ret) {
			ret = -EFAULT;
			goto out;
		}
		ret = desc_enqueue(desc_ctrl, p, cp_size, 0, axd_pipe);
		if (unlikely(ret)) {
			/* shouldn't happen, print a warning */
			dev_warn(axd->inputdev[pipe], "Warning, failed to enqueue buffer\n");
			goto out;
		}
		/* enqueued successfully, inform the axd firmware */
		axd_datain_kick(axd_pipe);
		written += cp_size;
		buf += cp_size;

		/*
		 * A time-based stream frame with PTS might have to be split
		 * over multiple buffers. We should only provide the PTS for
		 * the first buffer. The rest should have the PTS invalidated.
		 */
		axd->cmd.in_pipes[pipe].current_ts_high = -1;
		axd->cmd.in_pipes[pipe].current_ts_low = -1;
	}
out:
	if (written) {
#ifdef CONFIG_AXD_DEBUG_DIAG
		write_size[pipe] += written;
#endif
		return written;
	}
	return ret;
}

void axd_cmd_send_buffer_abort(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->in_pipes[pipe];
	struct axd_bufferq *desc_bufferq = &axd_pipe->desc_bufferq;

	axd_bufferq_abort_take(desc_bufferq);
}

static void axd_cmd_output_eos_reached(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->out_pipes[pipe];

	/* display diag info only if chan is enabled */
	if (axd_get_flag(&axd_pipe->enabled_flg)) {
		pr_debug("Output[%u] EOS reached\n", pipe);
		debugdiag("outSentCount[%u]= %u, outRecvCount[%u]= %u, read_size[%u]= %u\n",
			pipe, outSentCount[pipe], pipe, outRecvCount[pipe],
			pipe, read_size[pipe]);
		debugdiag("primeupCount[%u]= %u, recv_size[%u]= %u\n",
			pipe, primeupCount[pipe], pipe, recv_size[pipe]);

		/* All buffers are read, clear them. */
		axd_cmd_outpipe_clear(cmd, pipe);
	}
}

/*
 * Receive a buffer from output @pipe
 *
 * The logic in here is that buffers we can copy from are in user_bufferq which
 * is filled when we get an interrupt that the axd firmware filled them up.
 * desc_bufferq holds the buffers are yet to be serviced by the firmware.
 *
 * Returns number of bytes received, or negative error number.
 */
int axd_cmd_recv_buffer(struct axd_cmd *cmd, unsigned int pipe,
					char __user *buf, unsigned int size)
{
	struct axd_pipe *axd_pipe = &cmd->out_pipes[pipe];
	struct axd_bufferq *desc_bufferq = &axd_pipe->desc_bufferq;
	struct axd_bufferq *user_bufferq = &axd_pipe->user_bufferq;
	int ret = 0;
	int read = 0;
	int diff;
	unsigned int cp_size;
	unsigned int cur_buf_size, cur_buf_offset;
	char *cur_buf = axd_pipe->cur_buf;

	if (axd_get_flag(&axd_pipe->eos_flg) == EOF_REACHED) {
		axd_cmd_output_eos_reached(cmd, pipe);
		return 0;
	}

	axd_output_prime_up(axd_pipe);

	pr_debug("Reading %u bytes [%u]\n", size, pipe);
	while (read < size) {
		cur_buf_size = axd_pipe->cur_buf_size;
		cur_buf_offset = axd_pipe->cur_buf_offset;
		if (cur_buf_size) {
			/*
			 * Current buffer points to the current user buffer
			 * we're holding and reading from. We keep hold into it
			 * until it is completely read. The logic is done in
			 * this way because the likelihood of this buffer to be
			 * larger than the read count is quite high if not the
			 * normal case everytime a read is issued.
			 */
			diff = size - read;
			cp_size = diff < cur_buf_size ? diff : cur_buf_size;
			ret = copy_to_user(buf, cur_buf+cur_buf_offset,
								cp_size);
			if (ret)
				goto out;
			read += cp_size;
			buf += cp_size;
			axd_pipe->cur_buf_offset += cp_size;
			axd_pipe->cur_buf_size -= cp_size;
#ifdef CONFIG_AXD_DEBUG_DIAG
			read_size[pipe] += cp_size;
#endif
		} else {
			/*
			 * Current user buffer is completely read, return it to
			 * the desc_bufferq and take another user buffer.
			 * Note that we will sleep on either putting or taking
			 * from the buffer if we're full/empty. ISR should
			 * fill our user buffer once more are available.
			 */
			if (cur_buf) {
				ret = axd_bufferq_put(desc_bufferq, cur_buf, -1);
				if (ret)
					goto out;
				if (axd_bufferq_is_empty(user_bufferq) &&
					axd_get_flag(&axd_pipe->eos_flg)) {
					/* send EOF on next read */
					axd_set_flag(&axd_pipe->eos_flg,
								EOF_REACHED);
					/*
					 * Normally, we only need to clear up
					 * if read is 0. But, if the application
					 * is keeping track of where the stream
					 * ends, it might try to close the
					 * output pipe before the EOF is read.
					 * In this case, then the driver would
					 * lock up. Instead, we always clear up
					 * here to avoid this.
					 */
					axd_cmd_output_eos_reached(cmd, pipe);
					goto out;
				}
				axd_output_prime_up(axd_pipe);
			}
			cur_buf = axd_bufferq_take(user_bufferq, &cp_size);
			if (IS_ERR(cur_buf)) {
				axd_pipe->cur_buf = NULL;
				axd_pipe->cur_buf_offset = 0;
				axd_pipe->cur_buf_size = 0;
				/*
				 * if EOS is set and we get an error from
				 * bufferq_take then it is because we received a
				 * zero byte buffer with a EOS flag set (From
				 * the firmware), in this instance we just
				 * return EOF instead of the error code
				 * (ERESTARTSYS)
				 */
				if (axd_get_flag(&axd_pipe->eos_flg)) {
					axd_set_flag(&axd_pipe->eos_flg,
								EOF_REACHED);
					ret = 0;
					axd_cmd_output_eos_reached(cmd, pipe);
				} else {
					ret = PTR_ERR(cur_buf);
				}
				goto out;
			}
			axd_pipe->cur_buf_offset = 0;
			axd_pipe->cur_buf_size = cp_size;
			axd_pipe->cur_buf = cur_buf;
		}
	}
out:
	if (read) {
		axd_set_flag(&axd_pipe->enabled_flg, PIPE_RUNNING);
		return read;
	}
	return ret;
}

void axd_cmd_recv_buffer_abort(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_pipe *axd_pipe = &cmd->out_pipes[pipe];
	struct axd_bufferq *desc_bufferq = &axd_pipe->desc_bufferq;
	struct axd_bufferq *user_bufferq = &axd_pipe->user_bufferq;

	axd_bufferq_abort_put(desc_bufferq);
	axd_bufferq_abort_take(user_bufferq);
}
