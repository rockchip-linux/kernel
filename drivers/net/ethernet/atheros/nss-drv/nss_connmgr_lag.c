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
 * nss_connmgr_lag.c
 *	NSS LAG APIs
 */

#include <linux/if_bonding.h>

#include <nss_api_if.h>

#define nss_lag_info(fmt, args...) printk(KERN_INFO "nss LAG :"fmt, ##args)

/*
 * nss_send_lag_state()
 *	Send the currnet LAG state of a physical interface.
 */
nss_tx_status_t nss_send_lag_state(struct nss_ctx_instance *nss_ctx, struct net_device *slave)
{
	int32_t lagid = 0;
	int32_t slave_ifnum;
	nss_tx_status_t nss_tx_status;
	struct nss_lag_msg nm;
	struct nss_lag_state_change *nlsc = NULL;

	slave_ifnum = nss_cmn_get_interface_number(nss_ctx, slave);
	if (slave_ifnum < 0) {
		return NSS_TX_FAILURE_BAD_PARAM;
	}

	memset(&nm, 0, sizeof(nm));

	if (netif_is_bond_slave(slave)) {
		lagid = bond_get_id(slave->master) + NSS_LAG0_INTERFACE_NUM;
	} else {
		lagid = NSS_LAG0_INTERFACE_NUM;
	}

	nss_cmn_msg_init(&nm.cm, lagid,
			 NSS_TX_METADATA_LAG_STATE_CHANGE,
			 sizeof(struct nss_lag_state_change),
			 NULL, NULL);

	nlsc = &nm.msg.state;
	if (netif_is_bond_slave(slave)) {
		nlsc->event = NSS_LAG_ENSLAVE;
	} else {
		nlsc->event = NSS_LAG_RELEASE;
	}

	nlsc->interface = slave_ifnum;

	nss_tx_status = nss_lag_tx(nss_ctx, &nm);
	if (nss_tx_status != NSS_TX_SUCCESS) {
		return NSS_TX_FAILURE;
	}

	return NSS_TX_SUCCESS;
}


/**
 * @brief Event Callback to receive events from NSS
 * @param[in] pointer to net device context
 * @param[in] pointer to buffer
 * @return Returns void
 */
void nss_connmgr_lag_event_cb(void *if_ctx, struct nss_lag_msg *msg)
{
	switch (msg->cm.type)
	{
		default:
			nss_lag_info("%s: Unknown LAG event from NSS",__FUNCTION__);
			break;
	}
}
