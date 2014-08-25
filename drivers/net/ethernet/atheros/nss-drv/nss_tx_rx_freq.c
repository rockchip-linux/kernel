/*
 **************************************************************************
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
 * nss_tx_rx_freq.c
 *	NSS frequency change APIs
 */

#include "nss_tx_rx_common.h"

#define NSS_ACK_STARTED 0
#define NSS_ACK_FINISHED 1

extern struct nss_cmd_buffer nss_cmd_buf;
extern struct nss_frequency_statistics nss_freq_stat;
extern struct nss_runtime_sampling nss_runtime_samples;
extern struct workqueue_struct *nss_wq;
extern nss_work_t *nss_work;
extern void *nss_freq_change_context;

/*
 * nss_rx_metadata_nss_freq_ack()
 *	Handle the nss ack of frequency change.
 */
static void nss_rx_metadata_nss_freq_ack(struct nss_ctx_instance *nss_ctx, struct nss_freq_change *nfa)
{
	if (nfa->ack == NSS_ACK_STARTED) {
		/*
		 * NSS finished start noficiation - HW change clocks and send end notification
		 */
		nss_info("%p: NSS ACK Received: %d - Change HW CLK/Send Finish to NSS\n", nss_ctx, nfa->ack);

		return;
	}

	if (nfa->ack == NSS_ACK_FINISHED) {
		/*
		 * NSS finished end notification - Done
		 */
		nss_info("%p: NSS ACK Received: %d - End Notification ACK - Running: %dmhz\n", nss_ctx, nfa->ack, nfa->freq_current);
		nss_runtime_samples.freq_scale_ready = 1;
		return;
	}

	nss_info("%p: NSS had an error - Running: %dmhz\n", nss_ctx, nfa->freq_current);
}

/*
 * nss_freq_change()
 *	NSS frequency change API.
 */
nss_tx_status_t nss_freq_change(void *ctx, uint32_t eng, uint32_t stats_enable, uint32_t start_or_end)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *) ctx;
	struct sk_buff *nbuf;
	int32_t status;
	struct nss_corefreq_msg *ncm;
	struct nss_freq_change *nfc;

	nss_info("%p: Frequency Changing to: %d\n", nss_ctx, eng);

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

	ncm = (struct nss_corefreq_msg *)skb_put(nbuf, sizeof(struct nss_corefreq_msg));
	ncm->cm.type = NSS_TX_METADATA_TYPE_NSS_FREQ_CHANGE;
	ncm->cm.version = NSS_HLOS_MESSAGE_VERSION;
	ncm->cm.interface = NSS_COREFREQ_INTERFACE;
	ncm->cm.len = nbuf->len;

	nfc = &ncm->msg.nfc;
	nfc->frequency = eng;
	nfc->start_or_end = start_or_end;
	nfc->stats_enable = stats_enable;

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_info("%p: unable to enqueue 'nss frequency change' - marked as stopped\n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit, NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	return NSS_TX_SUCCESS;
}

/*
 * nss_frequency_workqueue()
 *	Queue Work to the NSS Workqueue based on Current index.
 */
static void nss_frequency_workqueue(void)
{
	BUG_ON(!nss_wq);

	nss_cmd_buf.current_freq = nss_runtime_samples.freq_scale[nss_runtime_samples.freq_scale_index].frequency;

	nss_work = (nss_work_t *)kmalloc(sizeof(nss_work_t), GFP_KERNEL);
	if (!nss_work) {
		nss_info("NSS FREQ WQ kmalloc fail");
		return;
	}

	INIT_WORK((struct work_struct *)nss_work, nss_wq_function);
	nss_work->frequency = nss_cmd_buf.current_freq;
	nss_work->stats_enable =  1;
	queue_work(nss_wq, (struct work_struct *)nss_work);
}

/*
 *  nss_rx_metadata_nss_core_stats()
 *	Handle the core stats
 */
static void nss_rx_metadata_nss_core_stats(struct nss_ctx_instance *nss_ctx, struct nss_core_stats *core_stats)
{
	uint32_t b_index;
	uint32_t minimum;
	uint32_t maximum;
	uint32_t sample;

	sample = core_stats->inst_cnt_total;

	/*
	 * We do not accept any statistics if auto scaling is off,
	 * we start with a fresh sample set when scaling is
	 * eventually turned on.
	 */
	if (!nss_cmd_buf.auto_scale && nss_runtime_samples.initialized) {
		return;
	}

	/*
	 * Delete Current Index Value, Add New Value, Recalculate new Sum, Shift Index
	 */
	b_index = nss_runtime_samples.buffer_index;

	nss_runtime_samples.sum = nss_runtime_samples.sum - nss_runtime_samples.buffer[b_index];
	nss_runtime_samples.buffer[b_index] = sample;
	nss_runtime_samples.sum = nss_runtime_samples.sum + nss_runtime_samples.buffer[b_index];
	nss_runtime_samples.buffer_index = (b_index + 1) & NSS_SAMPLE_BUFFER_MASK;

	if (nss_runtime_samples.sample_count < NSS_SAMPLE_BUFFER_SIZE) {
		nss_runtime_samples.sample_count++;

		/*
		 * Samples Are All Ready, Start Auto Scale
		 */
		if (nss_runtime_samples.sample_count == NSS_SAMPLE_BUFFER_SIZE ) {
			nss_cmd_buf.auto_scale = 1;
			nss_runtime_samples.freq_scale_ready = 1;
			nss_runtime_samples.initialized = 1;
		}

		return;
	}

	nss_runtime_samples.average = nss_runtime_samples.sum / nss_runtime_samples.sample_count;

	/*
	 * Print out statistics every 10 seconds
	 */
	if (nss_runtime_samples.message_rate_limit == NSS_MESSAGE_RATE_LIMIT) {
		nss_trace("%p: Running AVG:%x Sample:%x Divider:%d\n", nss_ctx, nss_runtime_samples.average, core_stats->inst_cnt_total, nss_runtime_samples.sample_count);
		nss_trace("%p: Current Frequency Index:%d\n", nss_ctx, nss_runtime_samples.freq_scale_index);
		nss_trace("%p: Auto Scale:%d Auto Scale Ready:%d\n", nss_ctx, nss_runtime_samples.freq_scale_ready, nss_cmd_buf.auto_scale);
		nss_trace("%p: Current Rate:%x\n", nss_ctx, nss_runtime_samples.average);

		nss_runtime_samples.message_rate_limit = 0;
	} else {
		nss_runtime_samples.message_rate_limit++;
	}

	/*
	 * Scale Algorithmn UP and DOWN
	 */
	if ((nss_runtime_samples.freq_scale_ready == 1) && (nss_cmd_buf.auto_scale == 1)) {
		if (nss_runtime_samples.freq_scale_rate_limit_up == NSS_FREQUENCY_SCALE_RATE_LIMIT_UP) {
//			nss_info("%p: Preparing Switch Inst_Cnt Avg:%x\n", nss_ctx, nss_runtime_samples.average);

			maximum = nss_runtime_samples.freq_scale[nss_runtime_samples.freq_scale_index].maximum;

			if ((sample > maximum) && (nss_runtime_samples.freq_scale_index < (nss_runtime_samples.freq_scale_sup_max - 1))) {
				nss_runtime_samples.freq_scale_index++;
				nss_runtime_samples.freq_scale_ready = 0;
				nss_frequency_workqueue();
//				nss_info("%p: Switch Up with Sample %x \n", nss_ctx, sample);
			} else {
//				nss_info("%p: No Change at Max\n", nss_ctx);
			}
			nss_runtime_samples.freq_scale_rate_limit_up = 0;
			return;

		} else {
			nss_runtime_samples.freq_scale_rate_limit_up++;
		}

		minimum = nss_runtime_samples.freq_scale[nss_runtime_samples.freq_scale_index].minimum;

		if ((nss_runtime_samples.average < minimum) && (nss_runtime_samples.freq_scale_index > 0)) {
			nss_runtime_samples.freq_scale_rate_limit_down++;

			if (nss_runtime_samples.freq_scale_rate_limit_down == NSS_FREQUENCY_SCALE_RATE_LIMIT_DOWN) {
				nss_runtime_samples.freq_scale_index--;
				nss_runtime_samples.freq_scale_ready = 0;
				nss_frequency_workqueue();
				nss_runtime_samples.freq_scale_rate_limit_down = 0;
			}
		} else {
			nss_runtime_samples.freq_scale_rate_limit_down = 0;
		}
	}
}

/*
 * nss_rx_freq_interface_handler()
 *	Handle NSS -> HLOS messages for Frequency Changes and Statistics
 */
static void nss_rx_freq_interface_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm, __attribute__((unused))void *app_data) {

	struct nss_corefreq_msg *ncfm = (struct nss_corefreq_msg *)ncm;

	switch (ncfm->cm.type) {
	case COREFREQ_METADATA_TYPE_TX_FREQ_ACK:
		nss_rx_metadata_nss_freq_ack(nss_ctx, &ncfm->msg.nfc);
		break;
	case COREFREQ_METADATA_TYPE_TX_CORE_STATS:
		nss_rx_metadata_nss_core_stats(nss_ctx, &ncfm->msg.ncs);
		break;

	default:
		if (ncm->response != NSS_CMN_RESPONSE_ACK) {
			/*
			 * Check response
			 */
			nss_info("%p: Received response %d for type %d, interface %d", nss_ctx, ncm->response, ncm->type, ncm->interface);
		}
	}
}

/*
 * nss_tunipip6_register_handler()
 */
void nss_core_freq_register_handler(void)
{
	nss_core_register_handler(NSS_COREFREQ_INTERFACE, nss_rx_freq_interface_handler, NULL);
}

/*
 * nss_get_frequency_mgr()
 */
void *nss_get_frequency_mgr(void)
{
	return (void *)&nss_top_main.nss[nss_top_main.frequency_handler_id];
}
