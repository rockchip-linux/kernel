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
 * nss_tx_rx_crypto.c
 *	NSS crypto APIs
 */

#include "nss_tx_rx_common.h"

/*
 **********************************
 Tx APIs
 **********************************
 */

/*
 * nss_tx_crypto_if_open()
 *	NSS crypto configure API.
 */
nss_tx_status_t nss_tx_crypto_if_open(void *ctx, uint8_t *buf, uint32_t len)
{
	nss_warning("%p: deprecated crypto API(s):%s, switch to newer ones\n", ctx, __func__);
	return NSS_TX_SUCCESS;
}

/*
 * nss_tx_crypto_if_buf()
 *	NSS crypto Tx API. Sends a crypto buffer to NSS.
 */
nss_tx_status_t nss_tx_crypto_if_buf(void *ctx, void *buf, uint32_t buf_paddr, uint16_t len)
{
	nss_warning("%p: deprecated crypto API(s):%s, switch to newer ones\n", ctx, __func__);
	return NSS_TX_SUCCESS;
}


/*
 **********************************
 Register/Unregister/Miscellaneous APIs
 **********************************
 */

/*
 * nss_register_crypto_mgr()
 */
void *nss_register_crypto_if(nss_crypto_data_callback_t crypto_data_callback, void *ctx)
{
	nss_top_main.crypto_ctx = ctx;

	nss_warning("deprecated crypto API(s):%s, switch to newer ones\n", __func__);

	return (void *)&nss_top_main.nss[nss_top_main.crypto_handler_id];
}

/*
 * nss_register_crypto_sync_if()
 */
void nss_register_crypto_sync_if(nss_crypto_sync_callback_t crypto_sync_callback, void *ctx)
{
	nss_warning("deprecated crypto API(s):%s, switch to newer ones\n", __func__);

	nss_top_main.crypto_ctx = ctx;
}

/*
 * nss_unregister_crypto_mgr()
 */
void nss_unregister_crypto_if(void)
{
	nss_warning("deprecated crypto API(s):%s, switch to newer ones\n", __func__);
	nss_top_main.crypto_ctx = NULL;
}


EXPORT_SYMBOL(nss_register_crypto_if);
EXPORT_SYMBOL(nss_register_crypto_sync_if);
EXPORT_SYMBOL(nss_unregister_crypto_if);
EXPORT_SYMBOL(nss_tx_crypto_if_buf);
EXPORT_SYMBOL(nss_tx_crypto_if_open);
