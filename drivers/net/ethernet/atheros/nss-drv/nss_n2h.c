/*
 **************************************************************************
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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
 * nss_n2h.c
 *	NSS N2H node APIs
 */

#include "nss_tx_rx_common.h"

#define NSS_CORE_0			0
#define NSS_CORE_1			1

/*
 * This number is chosen becuase currently default IPV4 + IPV6
 * connection size is 1024 + 1024 = 2048.
 *  FYI: However this doesnt have any impact on n2h/ipv6 connections
 */
#define NSS_N2H_MIN_EMPTY_POOL_BUF_SZ		2048
#define NSS_N2H_DEFAULT_EMPTY_POOL_BUF_SZ	8192

int nss_n2h_empty_pool_buf_cfg[NSS_MAX_CORES] __read_mostly = {NSS_N2H_DEFAULT_EMPTY_POOL_BUF_SZ, NSS_N2H_DEFAULT_EMPTY_POOL_BUF_SZ};

struct nss_n2h_registered_data {
	nss_n2h_msg_callback_t n2h_callback;
	void *app_data;
};

static struct nss_n2h_cfg_pvt nss_n2h_nepbcfgp[NSS_MAX_CORES];
static struct nss_n2h_registered_data nss_n2h_rd[NSS_MAX_CORES];

/*
 * nss_n2h_stats_sync()
 *	Handle the syncing of NSS statistics.
 */
static void nss_n2h_stats_sync(struct nss_ctx_instance *nss_ctx, struct nss_n2h_stats_sync *nnss)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;

	spin_lock_bh(&nss_top->stats_lock);

	/*
	 * common node stats
	 */
	nss_ctx->stats_n2h[NSS_STATS_NODE_RX_PKTS] += nnss->node_stats.rx_packets;
	nss_ctx->stats_n2h[NSS_STATS_NODE_RX_BYTES] += nnss->node_stats.rx_bytes;
	nss_ctx->stats_n2h[NSS_STATS_NODE_RX_DROPPED] += nnss->node_stats.rx_dropped;
	nss_ctx->stats_n2h[NSS_STATS_NODE_TX_PKTS] += nnss->node_stats.tx_packets;
	nss_ctx->stats_n2h[NSS_STATS_NODE_TX_BYTES] += nnss->node_stats.tx_bytes;

	/*
	 * General N2H stats
	 */
	nss_ctx->stats_n2h[NSS_STATS_N2H_QUEUE_DROPPED] += nnss->queue_dropped;
	nss_ctx->stats_n2h[NSS_STATS_N2H_TOTAL_TICKS] += nnss->total_ticks;
	nss_ctx->stats_n2h[NSS_STATS_N2H_WORST_CASE_TICKS] += nnss->worst_case_ticks;
	nss_ctx->stats_n2h[NSS_STATS_N2H_ITERATIONS] += nnss->iterations;

	/*
	 * pbuf manager ocm and default pool stats
	 */
	nss_ctx->stats_n2h[NSS_STATS_N2H_PBUF_OCM_ALLOC_FAILS] += nnss->pbuf_ocm_stats.pbuf_alloc_fails;
	nss_ctx->stats_n2h[NSS_STATS_N2H_PBUF_OCM_FREE_COUNT] = nnss->pbuf_ocm_stats.pbuf_free_count;
	nss_ctx->stats_n2h[NSS_STATS_N2H_PBUF_OCM_TOTAL_COUNT] = nnss->pbuf_ocm_stats.pbuf_total_count;

	nss_ctx->stats_n2h[NSS_STATS_N2H_PBUF_DEFAULT_ALLOC_FAILS] += nnss->pbuf_default_stats.pbuf_alloc_fails;
	nss_ctx->stats_n2h[NSS_STATS_N2H_PBUF_DEFAULT_FREE_COUNT] = nnss->pbuf_default_stats.pbuf_free_count;
	nss_ctx->stats_n2h[NSS_STATS_N2H_PBUF_DEFAULT_TOTAL_COUNT] = nnss->pbuf_default_stats.pbuf_total_count;

	/*
	 * payload mgr stats
	 */
	nss_ctx->stats_n2h[NSS_STATS_N2H_PAYLOAD_ALLOC_FAILS] += nnss->payload_alloc_fails;

	/*
	 * Host <=> NSS control traffic stats
	 */
	nss_ctx->stats_n2h[NSS_STATS_N2H_H2N_CONTROL_PACKETS] += nnss->h2n_ctrl_pkts;
	nss_ctx->stats_n2h[NSS_STATS_N2H_H2N_CONTROL_BYTES] += nnss->h2n_ctrl_bytes;
	nss_ctx->stats_n2h[NSS_STATS_N2H_N2H_CONTROL_PACKETS] += nnss->n2h_ctrl_pkts;
	nss_ctx->stats_n2h[NSS_STATS_N2H_N2H_CONTROL_BYTES] += nnss->n2h_ctrl_bytes;

	/*
	 * Host <=> NSS control data traffic stats
	 */
	nss_ctx->stats_n2h[NSS_STATS_N2H_H2N_DATA_PACKETS] += nnss->h2n_data_pkts;
	nss_ctx->stats_n2h[NSS_STATS_N2H_H2N_DATA_BYTES] += nnss->h2n_data_bytes;
	nss_ctx->stats_n2h[NSS_STATS_N2H_N2H_DATA_PACKETS] += nnss->n2h_data_pkts;
	nss_ctx->stats_n2h[NSS_STATS_N2H_N2H_DATA_BYTES] += nnss->n2h_data_bytes;

	spin_unlock_bh(&nss_top->stats_lock);
}

/*
 * nss_n2h_interface_handler()
 *	Handle NSS -> HLOS messages for N2H node
 */
static void nss_n2h_interface_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm, __attribute__((unused))void *app_data)
{
	struct nss_n2h_msg *nnm = (struct nss_n2h_msg *)ncm;
	nss_n2h_msg_callback_t cb;

	BUG_ON(ncm->interface != NSS_N2H_INTERFACE);

	/*
	 * Is this a valid request/response packet?
	 */
	if (nnm->cm.type >= NSS_METADATA_TYPE_N2H_MAX) {
		nss_warning("%p: received invalid message %d for Offload stats interface", nss_ctx, nnm->cm.type);
		return;
	}

	switch (nnm->cm.type) {
	case NSS_TX_METADATA_TYPE_N2H_RPS_CFG:
		nss_ctx->n2h_rps_en = nnm->msg.rps_cfg.enable;
		nss_info("NSS N2H rps_en %d \n",nnm->msg.rps_cfg.enable);
		break;

	case NSS_TX_METADATA_TYPE_N2H_EMPTY_POOL_BUF_CFG:
		nss_info("%p: empty pool buf cfg response from FW", nss_ctx);
		break;

	case NSS_RX_METADATA_TYPE_N2H_STATS_SYNC:
		nss_n2h_stats_sync(nss_ctx, &nnm->msg.stats_sync);
		break;

	default:
		if (ncm->response != NSS_CMN_RESPONSE_ACK) {
			/*
			 * Check response
			 */
			nss_info("%p: Received response %d for type %d, interface %d",
						nss_ctx, ncm->response, ncm->type, ncm->interface);
		}
	}

	/*
	 * Update the callback and app_data for NOTIFY messages, IPv4 sends all notify messages
	 * to the same callback/app_data.
	 */
	if (nnm->cm.response == NSS_CMM_RESPONSE_NOTIFY) {
		/*
		 * Place holder for the user to create right call
		 * back and app data when response is NSS_CMM_RESPONSE_NOTIFY
		 */
		ncm->cb = (uint32_t)nss_n2h_rd[nss_ctx->id].n2h_callback;
		ncm->app_data = (uint32_t)nss_n2h_rd[nss_ctx->id].app_data;
	}

	/*
	 * Do we have a callback?
	 */
	if (!ncm->cb) {
		return;
	}

	/*
	 * Callback
	 */
	cb = (nss_n2h_msg_callback_t)ncm->cb;
	cb((void *)ncm->app_data, nnm);
}

/*
 * nss_n2h_empty_pool_buf_cfg_core1_callback()
 *	call back function for the n2h connection configuration handler
 */
static void nss_n2h_empty_pool_buf_cfg_callback(void *app_data,
						struct nss_n2h_msg *nnm)
{
	int core_num = (int)app_data;
	if (nnm->cm.response != NSS_CMN_RESPONSE_ACK) {
		struct nss_n2h_empty_pool_buf *nnepbcm;
		nnepbcm = &nnm->msg.empty_pool_buf_cfg;

		/*
		 * Error, hence we are not updating the nss_n2h_empty_pool_buf
		 * Restore the current_value to its previous state
		 */
		nss_warning("Core %d empty pool buf set failure: %d\n", core_num, nnm->cm.error);
		nss_n2h_nepbcfgp[core_num].response = NSS_FAILURE;
		complete(&nss_n2h_nepbcfgp[core_num].complete);
		return;
	}

	/*
	 * Sucess at NSS FW, hence updating nss_n2h_empty_pool_buf, with the valid value
	 * saved at the sysctl handler.
	 */
	nss_info("Core %d empty pool buf set success: %d\n", core_num, nnm->cm.error);
	nss_n2h_nepbcfgp[core_num].response = NSS_SUCCESS;
	complete(&nss_n2h_nepbcfgp[core_num].complete);
}

/*
 * nss_n2h_empty_pool_buf_core1_handler()
 *	Sets the number of connections for IPv4
 */
static int nss_n2h_set_empty_pool_buf(ctl_table *ctl, int write, void __user *buffer,
					size_t *lenp, loff_t *ppos,
					int core_num, int *new_val)
{
	struct nss_top_instance *nss_top = &nss_top_main;
	struct nss_ctx_instance *nss_ctx = &nss_top->nss[core_num];
	struct nss_n2h_msg nnm;
	struct nss_n2h_empty_pool_buf *nnepbcm;
	nss_tx_status_t nss_tx_status;
	int ret = NSS_FAILURE;

	/*
	 * Acquiring semaphore
	 */
	down(&nss_n2h_nepbcfgp[core_num].sem);

	/*
	 * Take snap shot of current value
	 */
	nss_n2h_nepbcfgp[core_num].current_value = *new_val;

	/*
	 * Write the variable with user input
	 */
	ret = proc_dointvec(ctl, write, buffer, lenp, ppos);
	if (ret || (!write)) {
		up(&nss_n2h_nepbcfgp[core_num].sem);
		return ret;
	}

	/*
	 * Input for n2h should be atleast 2048 to support defalt connections
	 * of 1024 (IPV4) + 1024 (IPV6) connections.
	 */
	if ((*new_val < NSS_N2H_MIN_EMPTY_POOL_BUF_SZ)) {
		nss_warning("%p: core %d setting %d is less than minimum number of buffer",
				nss_ctx, core_num, *new_val);

		/*
		 * Restore the current_value to its previous state
		 */
		*new_val = nss_n2h_nepbcfgp[core_num].current_value;
		up(&nss_n2h_nepbcfgp[core_num].sem);
		return NSS_FAILURE;
	}

	nss_info("%p: core %d number of empty pool buffer is : %d\n",
		nss_ctx, core_num, *new_val);

	nss_n2h_msg_init(&nnm, NSS_N2H_INTERFACE,
			NSS_TX_METADATA_TYPE_N2H_EMPTY_POOL_BUF_CFG,
			sizeof(struct nss_n2h_empty_pool_buf),
			(nss_n2h_msg_callback_t *)nss_n2h_empty_pool_buf_cfg_callback,
			(void *)core_num);

	nnepbcm = &nnm.msg.empty_pool_buf_cfg;
	nnepbcm->pool_size = htonl(*new_val);
	nss_tx_status = nss_n2h_tx_msg(nss_ctx, &nnm);

	if (nss_tx_status != NSS_TX_SUCCESS) {
		nss_warning("%p: core %d nss_tx error setting empty pool buffer: %d\n",
				nss_ctx, core_num, *new_val);

		/*
		 * Restore the current_value to its previous state
		 */
		*new_val = nss_n2h_nepbcfgp[core_num].current_value;
		up(&nss_n2h_nepbcfgp[core_num].sem);
		return NSS_FAILURE;
	}

	/*
	 * Blocking call, wait till we get ACK for this msg.
	 */
	ret = wait_for_completion_timeout(&nss_n2h_nepbcfgp[core_num].complete, msecs_to_jiffies(NSS_CONN_CFG_TIMEOUT));
	if (ret == 0) {
		nss_warning("%p: core %d Waiting for ack timed out\n", nss_ctx, core_num);

		/*
		 * Restore the current_value to its previous state
		 */
		*new_val = nss_n2h_nepbcfgp[core_num].current_value;
		up(&nss_n2h_nepbcfgp[core_num].sem);
		return NSS_FAILURE;
	}

	/*
	 * ACK/NACK received from NSS FW
	 * If ACK: Callback function will update nss_n2h_empty_pool_buf with
	 * nss_n2h_nepbcfgp.num_conn_valid, which holds the user input
	 */
	if (NSS_FAILURE == nss_n2h_nepbcfgp[core_num].response) {

		/*
		 * Restore the current_value to its previous state
		 */
		*new_val = nss_n2h_nepbcfgp[core_num].current_value;
		up(&nss_n2h_nepbcfgp[core_num].sem);
		return NSS_FAILURE;
	}

	up(&nss_n2h_nepbcfgp[core_num].sem);
	return NSS_SUCCESS;
}

/*
 * nss_n2h_empty_pool_buf_core1_handler()
 *	Sets the number of empty buffer for core 1
 */
static int nss_n2h_empty_pool_buf_cfg_core1_handler(ctl_table *ctl,
						    int write, void __user *buffer,
						    size_t *lenp, loff_t *ppos)
{
	return nss_n2h_set_empty_pool_buf(ctl, write, buffer, lenp, ppos,
					NSS_CORE_1, &nss_n2h_empty_pool_buf_cfg[NSS_CORE_1]);
}

/*
 * nss_n2h_empty_pool_buf_core0_handler()
 *	Sets the number of empty buffer for core 0
 */
static int nss_n2h_empty_pool_buf_cfg_core0_handler(ctl_table *ctl,
						    int write, void __user *buffer,
						    size_t *lenp, loff_t *ppos)
{
	return nss_n2h_set_empty_pool_buf(ctl, write, buffer, lenp, ppos,
					NSS_CORE_0, &nss_n2h_empty_pool_buf_cfg[NSS_CORE_0]);
}

static ctl_table nss_n2h_table[] = {
	{
		.procname		= "n2h_empty_pool_buf_core0",
		.data			= &nss_n2h_empty_pool_buf_cfg[NSS_CORE_0],
		.maxlen			= sizeof(int),
		.mode			= 0644,
		.proc_handler   	= &nss_n2h_empty_pool_buf_cfg_core0_handler,
	},
	{
		.procname		= "n2h_empty_pool_buf_core1",
		.data			= &nss_n2h_empty_pool_buf_cfg[NSS_CORE_1],
		.maxlen			= sizeof(int),
		.mode			= 0644,
		.proc_handler   	= &nss_n2h_empty_pool_buf_cfg_core1_handler,
	},

	{ }
};

static ctl_table nss_n2h_dir[] = {
	{
		.procname		= "n2hcfg",
		.mode			= 0555,
		.child			= nss_n2h_table,
	},
	{ }
};


static ctl_table nss_n2h_root_dir[] = {
	{
		.procname		= "nss",
		.mode			= 0555,
		.child			= nss_n2h_dir,
	},
	{ }
};

static ctl_table nss_n2h_root[] = {
	{
		.procname		= "dev",
		.mode			= 0555,
		.child			= nss_n2h_root_dir,
	},
	{ }
};

static struct ctl_table_header *nss_n2h_header;

/*
 * nss_n2h_msg_init()
 *	Initialize IPv4 message.
 */
void nss_n2h_msg_init(struct nss_n2h_msg *nim, uint16_t if_num, uint32_t type,
		      uint32_t len, nss_n2h_msg_callback_t *cb, void *app_data)
{
	nss_cmn_msg_init(&nim->cm, if_num, type, len, (void *)cb, app_data);
}


/*
 * nss_n2h_register_sysctl()
 *	Register sysctl specific to n2h
 */
void nss_n2h_empty_pool_buf_register_sysctl(void)
{
	/*
	 * Register sysctl table.
	 */
	nss_n2h_header = register_sysctl_table(nss_n2h_root);

	/*
	 * Core0
	 */
	sema_init(&nss_n2h_nepbcfgp[NSS_CORE_0].sem, 1);
	init_completion(&nss_n2h_nepbcfgp[NSS_CORE_0].complete);
	nss_n2h_nepbcfgp[NSS_CORE_0].current_value = nss_n2h_empty_pool_buf_cfg[NSS_CORE_0];

	/*
	 * Core1
	 */
	sema_init(&nss_n2h_nepbcfgp[NSS_CORE_1].sem, 1);
	init_completion(&nss_n2h_nepbcfgp[NSS_CORE_1].complete);
	nss_n2h_nepbcfgp[NSS_CORE_1].current_value = nss_n2h_empty_pool_buf_cfg[NSS_CORE_1];
}

/*
 * nss_n2h_unregister_sysctl()
 *	Unregister sysctl specific to n2h
 */
void nss_n2h_empty_pool_buf_unregister_sysctl(void)
{
	/*
	 * Unregister sysctl table.
	 */
	if (nss_n2h_header) {
		unregister_sysctl_table(nss_n2h_header);
	}
}

/*
 * nss_n2h_tx_msg()
 *	Send messages to NSS n2h pacakge
 */
nss_tx_status_t nss_n2h_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_n2h_msg *nnm)
{
	struct nss_n2h_msg *nnm2;
	struct nss_cmn_msg *ncm = &nnm->cm;
	struct sk_buff *nbuf;
	nss_tx_status_t status;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		return NSS_TX_FAILURE_NOT_READY;
	}

	/*
	 * Sanity check the message
	 */
	if (ncm->interface != NSS_N2H_INTERFACE) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return NSS_TX_FAILURE;
	}

	if (ncm->type >= NSS_METADATA_TYPE_N2H_MAX) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return NSS_TX_FAILURE;
	}

	if (ncm->len > sizeof(struct nss_n2h_msg)) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return NSS_TX_FAILURE;
	}


	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		spin_lock_bh(&nss_ctx->nss_top->stats_lock);
		nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]++;
		spin_unlock_bh(&nss_ctx->nss_top->stats_lock);
		return NSS_TX_FAILURE;
	}

	/*
	 * Copy the message to our skb.
	 */
	nnm2 = (struct nss_n2h_msg *)skb_put(nbuf, sizeof(struct nss_n2h_msg));
	memcpy(nnm2, nnm, sizeof(struct nss_n2h_msg));
	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_info("%p: unable to enqueue 'nss frequency change' - marked as stopped\n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap,
				nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
				NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);
	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_CMD_REQ]);
	return NSS_TX_SUCCESS;
}


/*
 * nss_n2h_tx()
 *	Send Message to NSS to enable RPS.
 *
 * This API could be used for any additional RPS related
 * configuration in future.
 *
 * TODO: rename to _rps and rewrite assignment from handler to a callback.
 */
nss_tx_status_t nss_n2h_tx(struct nss_ctx_instance *nss_ctx, uint32_t enable_rps)
{
	struct sk_buff *nbuf;
	nss_tx_status_t status;
	struct nss_n2h_msg *nnhm;
	struct nss_n2h_rps *rps_cfg;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		return NSS_TX_FAILURE_NOT_READY;
	}

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		spin_lock_bh(&nss_ctx->nss_top->stats_lock);
		nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]++;
		spin_unlock_bh(&nss_ctx->nss_top->stats_lock);
		return NSS_TX_FAILURE;
	}

	nnhm = (struct nss_n2h_msg *)skb_put(nbuf, sizeof(struct nss_n2h_msg));

	nss_n2h_msg_init(nnhm, NSS_N2H_INTERFACE, NSS_TX_METADATA_TYPE_N2H_RPS_CFG,
			sizeof(struct nss_n2h_rps),
			NULL, NULL);

	rps_cfg = &nnhm->msg.rps_cfg;

	rps_cfg->enable = enable_rps;

	nss_info("n22_n2h_rps_configure %d \n", enable_rps);

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_info("%p: unable to enqueue 'nss frequency change' - marked as stopped\n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap,
				nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
				NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	return NSS_TX_SUCCESS;
}

/*
 * nss_n2h_notify_register()
 *	Register to received N2H events.
 *
 * NOTE: Do we want to pass an nss_ctx here so that we can register for n2h on any core?
 */
struct nss_ctx_instance *nss_n2h_notify_register(int core, nss_n2h_msg_callback_t cb, void *app_data)
{
	if (core >= NSS_MAX_CORES) {
		nss_warning("Input core number %d is wrong \n", core);
		return NULL;
	}
	/*
	 * TODO: We need to have a new array in support of the new API
	 * TODO: If we use a per-context array, we would move the array into nss_ctx based.
	 */
	nss_n2h_rd[core].n2h_callback = cb;
	nss_n2h_rd[core].app_data = app_data;
	return &nss_top_main.nss[core];
}

/*
 * nss_n2h_register_handler()
 */
void nss_n2h_register_handler()
{
	nss_core_register_handler(NSS_N2H_INTERFACE, nss_n2h_interface_handler, NULL);

	nss_n2h_notify_register(NSS_CORE_0, NULL, NULL);
	nss_n2h_notify_register(NSS_CORE_1, NULL, NULL);

	/*
	 * Registering sysctl for n2h empty pool buffer.
	 */
	nss_n2h_empty_pool_buf_register_sysctl();
}

EXPORT_SYMBOL(nss_n2h_notify_register);
