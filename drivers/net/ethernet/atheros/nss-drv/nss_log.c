/*
 **************************************************************************
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 **************************************************************************
 */
/*
 * nss_log.c
 *	NSS FW debug logger retrieval from DDR (memory)
 *
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/posix-timers.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <nss_hal.h>
#include "nss_core.h"
#include "nss_log.h"

/*
 * Gives us important data from NSS platform data
 */
extern struct nss_top_instance nss_top_main;

/*
 * Private data for each device file open instance
 */
struct nss_log_data {
	void *load_mem;		/* Pointer to struct nss_log_descriptor - descriptor data */
	dma_addr_t dma_addr;	/* Handle to DMA */
	uint32_t last_entry;	/* Last known sampled entry (or index) */
	uint32_t nentries;	/* Caches the total number of entries of log buffer */
	int nss_id;		/* NSS Core id being used */
};

/*
 * Saves the ring buffer address for logging per NSS core
 */
struct nss_ring_buffer_addr {
	void *addr;		/* Pointer to struct nss_log_descriptor */
	dma_addr_t dma_addr;	/* DMA Handle */
	uint32_t nentries;	/* Number of entries in the ring buffer */
	int refcnt;		/* Reference count */
};

static struct nss_ring_buffer_addr nss_rbe[NSS_MAX_CORES];

static DEFINE_MUTEX(nss_log_mutex);
static wait_queue_head_t nss_log_wq;
static nss_log_msg_callback_t nss_debug_interface_cb;
static void *nss_debug_interface_app_data = NULL;

static wait_queue_head_t msg_wq;
enum nss_cmn_response msg_response;
static bool msg_event;

/*
 * nss_log_llseek()
 *	Seek operation.
 */
static loff_t nss_log_llseek(struct file *file, loff_t offset, int origin)
{
	struct nss_log_data *data = file->private_data;

	switch (origin) {
	case SEEK_SET:
		break;
	case SEEK_CUR:
		offset += file->f_pos;
		break;
	case SEEK_END:
		offset = ((data->nentries * sizeof(struct nss_log_entry)) + sizeof(struct nss_log_descriptor)) - offset;
		break;
	default:
		return -EINVAL;
	}

	return (offset >= 0) ? (file->f_pos = offset) : -EINVAL;
}

/*
 * nss_log_open()
 *	Open operation for our device. We let as many instance run together
 */
static int nss_log_open(struct inode *inode, struct file *filp)
{
	struct nss_log_data *data = NULL;
	struct nss_top_instance *nss_top;
	struct nss_ctx_instance *nss_ctx;
	int nss_id;

	/*
	 * i_private is passed to us by debug_fs_create()
	 */
	nss_id = (int)inode->i_private;
	if (nss_id < 0 || nss_id >= NSS_MAX_CORES) {
		nss_warning("nss_id is not valid :%d\n", nss_id);
		return -ENODEV;
	}

	nss_top = &nss_top_main;
	nss_ctx = &nss_top->nss[nss_id];

	data = kzalloc(sizeof(struct nss_log_data), GFP_KERNEL);
	if (!data) {
		nss_warning("%p: Failed to allocate memory for log_data", nss_ctx);
		return -ENOMEM;
	}

	mutex_lock(&nss_log_mutex);
	if (!nss_rbe[nss_id].addr) {
		mutex_unlock(&nss_log_mutex);
		kfree(data);
		nss_warning("%p: Ring buffer not configured yet for nss_id:%d", nss_ctx, nss_id);
		return -EIO;
	}

	/*
	 * Actual ring buffer.
	 */
	data->load_mem = nss_rbe[nss_id].addr;
	data->last_entry = 0;
	data->nentries = nss_rbe[nss_id].nentries;
	data->dma_addr = nss_rbe[nss_id].dma_addr;

	/*
	 * Increment the reference count so that we don't free
	 * the memory
	 */
	nss_rbe[nss_id].refcnt++;
	data->nss_id = nss_id;
	filp->private_data = data;
	mutex_unlock(&nss_log_mutex);

	return 0;
}

/*
 * nss_log_release()
 *	release gets called when close() is called on the file
 *	descriptor. We unmap the IO region.
 */
static int nss_log_release(struct inode *inode, struct file *filp)
{
	struct nss_log_data *data = filp->private_data;

	if (!data) {
		return -EINVAL;
	}

	mutex_lock(&nss_log_mutex);
	nss_rbe[data->nss_id].refcnt--;
	BUG_ON(nss_rbe[data->nss_id].refcnt < 0);
	if (nss_rbe[data->nss_id].refcnt == 0) {
		wake_up(&nss_log_wq);
	}
	mutex_unlock(&nss_log_mutex);
	kfree(data);
	return 0;
}

/*
 * nss_log_current_entry()
 *	Reads current entry index from NSS log descriptor.
 */
static uint32_t nss_log_current_entry(struct nss_log_descriptor *desc)
{
	rmb();
	return desc->current_entry;
}

/*
 * nss_log_read()
 *	Read operation lets command like cat and tail read our memory log buffer data.
 */
static ssize_t nss_log_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct nss_log_data *data = filp->private_data;
	struct nss_log_descriptor *desc;
	size_t bytes = 0;
	size_t b;
	struct nss_log_entry *rb;
	uint32_t entry;
	uint32_t offset, index;
	char msg[NSS_LOG_OUTPUT_LINE_SIZE];

	if (!data) {
		return -EINVAL;
	}

	desc = data->load_mem;
	if (!desc) {
		nss_warning("%p: load_mem is NULL", data);
		return -EINVAL;
	}

	/*
	 * If buffer is too small to fit even one entry.
	 */
	if (size < NSS_LOG_OUTPUT_LINE_SIZE) {
		return 0;
	}

	/*
	 * Get the current index
	 */
	dma_sync_single_for_cpu(NULL, data->dma_addr, sizeof (struct nss_log_descriptor), DMA_FROM_DEVICE);
	entry = nss_log_current_entry(desc);

	/*
	 * If the current and last sampled indexes are same then bail out.
	 */
	if (unlikely(data->last_entry == entry)) {
		return 0;
	}

	/*
	 * If this is the first read (after open) on our device file.
	 */
	if (unlikely(*ppos == 0)) {
		/*
		 * If log buffer has rolled over. Almost all the time
		 * it will be true.
		 */
		if (likely(entry > data->nentries)) {
			/*
			 * Determine how much we can stuff in one
			 * buffer passed to us and accordingly
			 * reduce our index.
			 */
			data->last_entry = entry - data->nentries;
		} else {
			data->last_entry = 0;
		}
	} else if (unlikely(entry > data->nentries && ((entry - data->nentries) > data->last_entry))) {
		/*
		 * If FW is producing debug buffer at a pace faster than
		 * we can consume, then we restrict our iteration.
		 */
		data->last_entry = entry - data->nentries;
	}

	/*
	 * Iterate over indexes.
	 */
	while (entry > data->last_entry) {
		index = offset = (data->last_entry % data->nentries);
		offset = (offset * sizeof (struct nss_log_entry))
			 + offsetof(struct nss_log_descriptor, log_ring_buffer);

		dma_sync_single_for_cpu(NULL, data->dma_addr + offset,
			sizeof(struct nss_log_entry), DMA_FROM_DEVICE);
		rb = &desc->log_ring_buffer[index];

		b = snprintf(msg, sizeof(msg), NSS_LOG_LINE_FORMAT,
			rb->thread_num, rb->timestamp, rb->message);

		data->last_entry++;

		/*
		 * Copy to user buffer and if we fail then we return
		 * failure.
		 */
	        if (copy_to_user(buf + bytes, msg, b) == 0) {
			bytes += b;
		} else {
			bytes = -EFAULT;
			break;
		}

		/*
		 * If we ran out of space in the buffer.
		 */
		if ((bytes + NSS_LOG_OUTPUT_LINE_SIZE) >= size)
			break;
	}

	if (bytes > 0)
		*ppos =  bytes;

	return bytes;
}

struct file_operations nss_logs_core_ops = {
	.owner = THIS_MODULE,
	.open = nss_log_open,
	.read = nss_log_read,
	.release = nss_log_release,
	.llseek = nss_log_llseek,
};

/*
 * nss_debug_interface_set_callback()
 *	Sets the callback
 */
void nss_debug_interface_set_callback(nss_log_msg_callback_t cb, void *app_data)
{
	nss_debug_interface_cb = cb;
	nss_debug_interface_app_data = app_data;
}

/*
 * nss_debug_interface_event()
 *	Received an event from NSS FW
 */
static void nss_debug_interface_event(void *app_data, struct nss_debug_interface_msg *nim)
{
	struct nss_cmn_msg *ncm = (struct nss_cmn_msg *)nim;

	msg_response = ncm->response;
	msg_event = true;
	wake_up(&msg_wq);
}

/*
 * nss_debug_interface_handler()
 * 	handle NSS -> HLOS messages for debug interfaces
 */
static void nss_debug_interface_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm, __attribute__((unused))void *app_data)
{
	struct nss_debug_interface_msg *ntm = (struct nss_debug_interface_msg *)ncm;
	nss_log_msg_callback_t cb;

	BUG_ON(ncm->interface != NSS_DEBUG_INTERFACE);

	/*
	 * Is this a valid request/response packet?
	 */
	if (ncm->type > NSS_DEBUG_INTERFACE_TYPE_MAX) {
		nss_warning("%p: received invalid message %d for CAPWAP interface", nss_ctx, ncm->type);
		return;
	}

	if (ncm->len > sizeof(struct nss_debug_interface_msg)) {
		nss_warning("%p: Length of message is greater than required: %d", nss_ctx, ncm->interface);
		return;
	}

	nss_core_log_msg_failures(nss_ctx, ncm);

	/*
	 * Update the callback and app_data for NOTIFY messages.
	 */
	if (ncm->response == NSS_CMM_RESPONSE_NOTIFY) {
		ncm->cb = (uint32_t)nss_debug_interface_cb;
		ncm->app_data = (uint32_t)nss_debug_interface_app_data;
	}

	/*
	 * Do we have a callback
	 */
	if (!ncm->cb) {
		nss_trace("%p: cb is null for interface %d", nss_ctx, ncm->interface);
		return;
	}

	cb = (nss_log_msg_callback_t)ncm->cb;
	cb((void *)ncm->app_data, ntm);
}

/*
 * nss_debug_interface_tx()
 * 	Transmit a debug interface message to NSS FW
 */
static nss_tx_status_t nss_debug_interface_tx(struct nss_ctx_instance *nss_ctx, struct nss_debug_interface_msg *msg)
{
	struct nss_debug_interface_msg *nm;
	struct nss_cmn_msg *ncm = &msg->cm;
	struct sk_buff *nbuf;
	int32_t status;

	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("%p: debug if msg dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}

	/*
	 * Sanity check the message
	 */
	if (ncm->interface != NSS_DEBUG_INTERFACE) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return NSS_TX_FAILURE;
	}

	if (ncm->type > NSS_DEBUG_INTERFACE_TYPE_MAX) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return NSS_TX_FAILURE;
	}

	if (ncm->len > sizeof(struct nss_debug_interface_msg)) {
		nss_warning("%p: message length is invalid: %d", nss_ctx, ncm->len);
		return NSS_TX_FAILURE;
	}

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]);
		nss_warning("%p: msg dropped as command allocation failed", nss_ctx);
		return NSS_TX_FAILURE;
	}

	/*
	 * Copy the message to our skb
	 */
	nm = (struct nss_debug_interface_msg *)skb_put(nbuf, sizeof(struct nss_debug_interface_msg));
	memcpy(nm, msg, sizeof(struct nss_debug_interface_msg));

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_warning("%p: Unable to enqueue 'debug if message' \n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
				NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_CMD_REQ]);
	return NSS_TX_SUCCESS;
}

/*
 * nss_debug_log_buffer_alloc()
 *	Allocates and Initializes log buffer for the use in NSS FW (logging)
 */
bool nss_debug_log_buffer_alloc(uint8_t nss_id, uint32_t nentry)
{
	struct nss_ring_buffer_addr old_rbe;
	struct nss_debug_interface_msg msg;
	struct nss_debug_log_memory_msg *dbg;
	struct nss_top_instance *nss_top;
	struct nss_ctx_instance *nss_ctx;
	dma_addr_t dma_addr;
	uint32_t size;
	void *addr = NULL;
	nss_tx_status_t status;
	bool err = false;
	bool old_state = false;

	if (nss_id > NSS_MAX_CORES) {
		return false;
	}

	nss_top = &nss_top_main;
	nss_ctx = &nss_top->nss[nss_id];

	if (nss_ctx->state != NSS_CORE_STATE_INITIALIZED) {
		nss_warning("%p: NSS Core:%d is not initialized yet\n", nss_ctx, nss_id);
		return false;
	}

	memset(&msg, 0, sizeof(struct nss_debug_interface_msg));

	size = sizeof (struct nss_log_descriptor) + (sizeof (struct nss_log_entry) * nentry);
	addr = kmalloc(size, GFP_ATOMIC);
	if (!addr) {
		nss_warning("%p: Failed to allocate memory for logging (size:%d)\n", nss_ctx, size);
		return false;
	}

	memset(addr, 0, size);
	dma_addr = (uint32_t)dma_map_single(NULL, addr, size, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(NULL, dma_addr))) {
		nss_warning("%p: Failed to map address in DMA", nss_ctx);
		goto fail2;
	}

	/*
	 * If we already have ring buffer associated with nss_id, then
	 * we must wait before we attach a new ring buffer.
	 */
	mutex_lock(&nss_log_mutex);
	if (nss_rbe[nss_id].addr) {
		mutex_unlock(&nss_log_mutex);
		if (!wait_event_timeout(nss_log_wq, nss_rbe[nss_id].refcnt == 0, 5 * HZ)) {
			nss_warning("%p: Timeout waiting for refcnt to become 0\n", nss_ctx);
			goto fail1;
		}

		mutex_lock(&nss_log_mutex);
		if (!nss_rbe[nss_id].addr) {
			mutex_unlock(&nss_log_mutex);
			goto fail1;
		}
		if (nss_rbe[nss_id].refcnt > 0) {
			mutex_unlock(&nss_log_mutex);
			nss_warning("%p: Some other thread is condenting..opting out\n", nss_ctx);
			goto fail1;
		}

		/*
		 * Save the original dma buffer. In case we fail down the line, we will
		 * restore the state. Otherwise, old_state will be freed once we get
		 * ACK from NSS FW.
		 */
		old_state = true;
		memcpy(&old_rbe, &nss_rbe[nss_id], sizeof (struct nss_ring_buffer_addr));
	}

	nss_rbe[nss_id].addr = addr;
	nss_rbe[nss_id].nentries = nentry;
	nss_rbe[nss_id].refcnt = 1;	/* Block other threads till we are done */
	nss_rbe[nss_id].dma_addr = dma_addr;
	mutex_unlock(&nss_log_mutex);

	memset(&msg, 0, sizeof (struct nss_debug_interface_msg));
	nss_cmn_msg_init(&msg.cm, NSS_DEBUG_INTERFACE, NSS_DEBUG_INTERFACE_TYPE_LOG_BUF_INIT,
		sizeof(struct nss_debug_log_memory_msg), nss_debug_interface_event, NULL);

	dbg = &msg.msg.addr;
	dbg->nentry = nentry;
	dbg->version = NSS_DEBUG_LOG_VERSION;
	dbg->phy_addr = dma_addr;

	msg_event = false;
	status = nss_debug_interface_tx(nss_ctx, &msg);
	if (status != NSS_TX_SUCCESS) {
		nss_warning("%p: Failed to send message to debug interface:%d\n", nss_ctx, status);
		err = true;
	} else {
		int r;

		/*
		 * Wait for 5 seconds since this is a critical operation.
		 */
		r = wait_event_timeout(msg_wq, msg_event == true, 5 * HZ);
		if (r == 0) {
			nss_warning("%p: Timeout send message to debug interface\n", nss_ctx);
			err = true;
		} else if (msg_response != NSS_CMN_RESPONSE_ACK) {
			nss_warning("%p: Response error for send message to debug interface:%d\n", nss_ctx, msg_response);
			err = true;
		}
	}

	/*
	 * If we had to free the previous allocation for ring buffer.
	 */
	if (old_state == true) {
		/*
		 * If we didn't fail, then we must unmap and free previous dma buffer
		 */
		if (err == false) {
			uint32_t old_size;

			old_size = sizeof (struct nss_log_descriptor) +
				(sizeof (struct nss_log_entry) * old_rbe.nentries);
			dma_unmap_single(NULL, old_rbe.dma_addr, old_size, DMA_FROM_DEVICE);
			kfree(old_rbe.addr);
		} else {
			/*
			 * Restore the original dma buffer since we failed somewhere.
			 */
			mutex_lock(&nss_log_mutex);
			memcpy(&nss_rbe[nss_id], &old_rbe, sizeof (struct nss_ring_buffer_addr));
			mutex_unlock(&nss_log_mutex);
			wake_up(&nss_log_wq);
		}
	} else {
		/*
		 * There was no logbuffer allocated from host side.
		 */

		/*
		 * If there was error, then we need to reset back. Note that we are
		 * still holding refcnt.
		 */
		if (err == true) {
			mutex_lock(&nss_log_mutex);
			nss_rbe[nss_id].addr = NULL;
			nss_rbe[nss_id].nentries = 0;
			nss_rbe[nss_id].refcnt = 0;
			nss_rbe[nss_id].dma_addr = 0;
			mutex_unlock(&nss_log_mutex);
			wake_up(&nss_log_wq);
		}
	}

	if (err == false) {
		mutex_lock(&nss_log_mutex);
		nss_rbe[nss_id].refcnt--;	/* we are done */
		mutex_unlock(&nss_log_mutex);
		wake_up(&nss_log_wq);
		return true;
	}

fail1:
	if (addr) {
		dma_unmap_single(NULL, dma_addr, size, DMA_FROM_DEVICE);
	}
fail2:
	kfree(addr);
	wake_up(&nss_log_wq);
	return false;
}

/*
 * nss_logbuffer_handler()
 *	Enable NSS debug output
 */
int nss_logbuffer_handler(ctl_table *ctl, int write, void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;
	int i;

	ret = proc_dointvec(ctl, write, buffer, lenp, ppos);
	if (ret)  {
		return ret;
	}

	if (!write) {
		return ret;
	}

	if (nss_ctl_logbuf < 32) {
		printk("Invalid NSS FW logbuffer size:%d (must be > 32)\n", nss_ctl_logbuf);
		nss_ctl_logbuf = 0;
		return ret;
	}

	for (i = 0; i < NSS_MAX_CORES; i++) {
		if (nss_debug_log_buffer_alloc(i, nss_ctl_logbuf) == false) {
			nss_warning("%d: Failed to set debug log buffer on NSS core", i);
		}
	}

	return ret;
}

/*
 * nss_log_init()
 *	Initializes NSS FW logs retrieval logic from /sys
 */
void nss_log_init(void)
{
	int core_status;
	int i;

	memset(nss_rbe, 0, sizeof(nss_rbe));
	init_waitqueue_head(&nss_log_wq);
	init_waitqueue_head(&msg_wq);

	/*
	 * Create directory for obtaining NSS FW logs from each core
	 */
	nss_top_main.logs_dentry = debugfs_create_dir("logs", nss_top_main.top_dentry);
	if (unlikely(!nss_top_main.logs_dentry)) {
		nss_warning("Failed to create qca-nss-drv/logs directory in debugfs");
		return;
	}

	for (i = 0; i < NSS_MAX_CORES; i++) {
		char file[10];
		extern struct file_operations nss_logs_core_ops;

		snprintf(file, sizeof(file), "core%d", i);
		nss_top_main.core_log_dentry = debugfs_create_file(file, 0400,
						nss_top_main.logs_dentry, (void *)i, &nss_logs_core_ops);
		if (unlikely(!nss_top_main.core_log_dentry)) {
			nss_warning("Failed to create qca-nss-drv/logs/%s file in debugfs", file);
			return;
		}
	}

	nss_debug_interface_set_callback(nss_debug_interface_event, NULL);
	core_status = nss_core_register_handler(NSS_DEBUG_INTERFACE, nss_debug_interface_handler, NULL);
	if (core_status != NSS_CORE_STATUS_SUCCESS) {
		nss_warning("NSS logbuffer init failed with register handler:%d\n", core_status);
	}
}
