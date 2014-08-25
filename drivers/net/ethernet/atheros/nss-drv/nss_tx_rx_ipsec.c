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
 * nss_tx_rx_ipsec.c
 *	NSS IPsec APIs
 */

#include "nss_tx_rx_common.h"

/*
 **********************************
 Tx APIs
 **********************************
 */

/*
 * nss_tx_ipsec_rule
 *	Send ipsec rule to NSS.
 */
nss_tx_status_t nss_tx_ipsec_rule(void *ctx, uint32_t interface_num, uint32_t type, uint8_t *buf, uint32_t len)
{
	nss_warning("%p:deprecated API(s), switch to newer ones\n", ctx);

	return NSS_TX_FAILURE;
}

/*
 **********************************
 Register/Unregister/Miscellaneous APIs
 **********************************
 */

/*
 * nss_register_ipsec_if()
 */
void *nss_register_ipsec_if(uint32_t if_num,
				nss_ipsec_data_callback_t ipsec_data_cb,
				void *if_ctx)
{
	nss_assert((if_num >= NSS_MAX_PHYSICAL_INTERFACES) && (if_num < NSS_MAX_NET_INTERFACES));

	nss_top_main.if_ctx[if_num] = if_ctx;
	nss_top_main.if_rx_callback[if_num] = ipsec_data_cb;

	return (void *)&nss_top_main.nss[nss_top_main.ipsec_handler_id];
}

/*
 * nss_register_ipsec_event_if()
 */
void nss_register_ipsec_event_if(uint32_t if_num, nss_ipsec_event_callback_t ipsec_event_cb)
{
	nss_assert((if_num >= NSS_MAX_PHYSICAL_INTERFACES) && (if_num < NSS_MAX_NET_INTERFACES));
	nss_warning("deprecated API(s), switch to newer ones : %d\n", if_num);

}

/*
 * nss_unregister_ipsec_if()
 */
void nss_unregister_ipsec_if(uint32_t if_num)
{
	nss_assert((if_num >= NSS_MAX_PHYSICAL_INTERFACES) && (if_num < NSS_MAX_NET_INTERFACES));

	nss_top_main.if_rx_callback[if_num] = NULL;
	nss_top_main.if_ctx[if_num] = NULL;

	nss_warning("deprecated API(s), switch to newer ones : %d\n", if_num);
}

EXPORT_SYMBOL(nss_register_ipsec_if);
EXPORT_SYMBOL(nss_register_ipsec_event_if);
EXPORT_SYMBOL(nss_unregister_ipsec_if);
EXPORT_SYMBOL(nss_tx_ipsec_rule);
