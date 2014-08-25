/*
 **************************************************************************
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
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
 * nss_cmn.c
 *	NSS generic APIs
 */

#include "nss_tx_rx_common.h"

/*
 * nss_cmn_msg_init()
 *	Initialize the common message structure.
 */
void nss_cmn_msg_init(struct nss_cmn_msg *ncm, uint16_t if_num, uint32_t type,  uint32_t len, void *cb, void *app_data)
{
	ncm->interface = if_num;
	ncm->version = NSS_HLOS_MESSAGE_VERSION;
	ncm->type = type;
	ncm->len = len;
	ncm->cb = (uint32_t)cb;
	ncm->app_data = (uint32_t)app_data;
}

/*
 * nss_cmn_get_interface_number()
 *	Return the interface number of the NSS net_device.
 *
 * Returns -1 on failure or the interface number of dev is an NSS net_device.
 */
int32_t nss_cmn_get_interface_number(struct nss_ctx_instance *nss_ctx, struct net_device *dev)
{
	int i;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("%p: Interface number could not be found as core not ready", nss_ctx);
		return -1;
	}

	nss_assert(dev != 0);

	/*
	 * Check physical interface table
	 */
	for (i = 0; i < NSS_MAX_NET_INTERFACES; i++) {
		if (dev == ((struct nss_ctx_instance *)nss_ctx)->nss_top->if_ctx[i]) {
			return i;
		}
	}

	nss_warning("%p: Interface number could not be found as interface has not registered yet", nss_ctx);
	return -1;
}

/*
 * nss_cmn_get_interface_dev()
 *	Return the net_device for NSS interface id.
 *
 * Returns NULL on failure or the net_device for NSS interface id.
 */
struct net_device *nss_cmn_get_interface_dev(struct nss_ctx_instance *ctx, uint32_t if_num)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *)ctx;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("%p: Interface device could not be found as core not ready", nss_ctx);
		return NULL;
	}

	if (unlikely(if_num >= NSS_MAX_NET_INTERFACES)) {
		return NULL;
	}

	return nss_ctx->nss_top->if_ctx[if_num];
}

/*
 * nss_cmn_get_state()
 *	return the NSS initialization state
 */
nss_state_t nss_cmn_get_state(struct nss_ctx_instance *ctx)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *)ctx;
	nss_state_t state = NSS_STATE_UNINITIALIZED;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	spin_lock_bh(&nss_top_main.lock);
	if (nss_ctx->state == NSS_CORE_STATE_INITIALIZED) {
		state = NSS_STATE_INITIALIZED;
	}
	spin_unlock_bh(&nss_top_main.lock);

	return state;
}

/*
 * nss_cmn_interface_is_virtual()
 * 	Return true if the interface number is a virtual NSS interface
 */
bool nss_cmn_interface_is_virtual(void *nss_ctx, int32_t interface_num)
{
	return (NSS_IS_IF_TYPE(VIRTUAL, interface_num));
}

/*
 * nss_cmn_register_queue_decongestion()
 *	Register for queue decongestion event
 */
nss_cb_register_status_t nss_cmn_register_queue_decongestion(struct nss_ctx_instance *nss_ctx, nss_cmn_queue_decongestion_callback_t event_callback, void *app_ctx)
{
	uint32_t i;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	spin_lock_bh(&nss_ctx->decongest_cb_lock);

	/*
	 * Find vacant location in callback table
	 */
	for (i = 0; i< NSS_MAX_CLIENTS; i++) {
		if (nss_ctx->queue_decongestion_callback[i] == NULL) {
			nss_ctx->queue_decongestion_callback[i] = event_callback;
			nss_ctx->queue_decongestion_ctx[i] = app_ctx;
			spin_unlock_bh(&nss_ctx->decongest_cb_lock);
			return NSS_CB_REGISTER_SUCCESS;
		}
	}

	spin_unlock_bh(&nss_ctx->decongest_cb_lock);
	return NSS_CB_REGISTER_FAILED;
}

/*
 * nss_cmn_unregister_queue_decongestion()
 *	Unregister for queue decongestion event
 */
nss_cb_unregister_status_t nss_cmn_unregister_queue_decongestion(struct nss_ctx_instance *nss_ctx, nss_cmn_queue_decongestion_callback_t event_callback)
{
	uint32_t i;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	spin_lock_bh(&nss_ctx->decongest_cb_lock);

	/*
	 * Find actual location in callback table
	 */
	for (i = 0; i< NSS_MAX_CLIENTS; i++) {
		if (nss_ctx->queue_decongestion_callback[i] == event_callback) {
			nss_ctx->queue_decongestion_callback[i] = NULL;
			nss_ctx->queue_decongestion_ctx[i] = NULL;
			spin_unlock_bh(&nss_ctx->decongest_cb_lock);
			return NSS_CB_UNREGISTER_SUCCESS;
		}
	}

	spin_unlock_bh(&nss_ctx->decongest_cb_lock);
	return NSS_CB_UNREGISTER_FAILED;
}

EXPORT_SYMBOL(nss_cmn_get_interface_number);
EXPORT_SYMBOL(nss_cmn_get_interface_dev);
EXPORT_SYMBOL(nss_cmn_get_state);
EXPORT_SYMBOL(nss_cmn_interface_is_virtual);
EXPORT_SYMBOL(nss_cmn_msg_init);

EXPORT_SYMBOL(nss_cmn_register_queue_decongestion);
EXPORT_SYMBOL(nss_cmn_unregister_queue_decongestion);

