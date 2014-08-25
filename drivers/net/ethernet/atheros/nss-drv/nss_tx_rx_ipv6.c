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
 * nss_tx_rx_ipv6.c
 *	NSS IPv6 APIs
 */

#include <linux/ppp_channel.h>
#include "nss_tx_rx_common.h"

/*
 * Depreceated callback maintained locally.
 */
nss_ipv6_callback_t nss_tx_rx_ipv6_event_callback = NULL;

/*
 * nss_rx_ipv6_sync()
 *	Handle the syncing of an IPv6 connection.
  */
void nss_rx_ipv6_sync(struct nss_ctx_instance *nss_ctx, struct nss_ipv6_conn_sync *nirs)
{
	struct nss_ipv6_cb_params nicp;

	nicp.reason = NSS_IPV6_CB_REASON_SYNC;
	nicp.params.sync.index = nirs->index;
	nicp.params.sync.protocol = (int32_t)nirs->protocol;
	nicp.params.sync.src_ip[0] = nirs->flow_ip[0];
	nicp.params.sync.src_ip[1] = nirs->flow_ip[1];
	nicp.params.sync.src_ip[2] = nirs->flow_ip[2];
	nicp.params.sync.src_ip[3] = nirs->flow_ip[3];
	nicp.params.sync.src_port = nirs->flow_ident;
	nicp.params.sync.flow_max_window = nirs->flow_max_window;
	nicp.params.sync.flow_end = nirs->flow_end;
	nicp.params.sync.flow_max_end = nirs->flow_max_end;
	nicp.params.sync.flow_rx_packet_count = nirs->flow_rx_packet_count;
	nicp.params.sync.flow_rx_byte_count = nirs->flow_rx_byte_count;
	nicp.params.sync.flow_tx_packet_count = nirs->flow_tx_packet_count;
	nicp.params.sync.flow_tx_byte_count = nirs->flow_tx_byte_count;
	nicp.params.sync.dest_ip[0] = nirs->return_ip[0];
	nicp.params.sync.dest_ip[1] = nirs->return_ip[1];
	nicp.params.sync.dest_ip[2] = nirs->return_ip[2];
	nicp.params.sync.dest_ip[3] = nirs->return_ip[3];
	nicp.params.sync.dest_port = nirs->return_ident;
	nicp.params.sync.return_max_window = nirs->return_max_window;
	nicp.params.sync.return_end = nirs->return_end;
	nicp.params.sync.return_max_end = nirs->return_max_end;
	nicp.params.sync.return_rx_packet_count = nirs->return_rx_packet_count;
	nicp.params.sync.return_rx_byte_count = nirs->return_rx_byte_count;
	nicp.params.sync.return_tx_packet_count = nirs->return_tx_packet_count;
	nicp.params.sync.return_tx_byte_count = nirs->return_tx_byte_count;

	nicp.params.sync.qos_tag = nirs->qos_tag;

	nicp.params.sync.flags = 0;
	if (nirs->flags & NSS_IPV6_RULE_CREATE_FLAG_NO_SEQ_CHECK) {
		nicp.params.sync.flags |= NSS_IPV6_CREATE_FLAG_NO_SEQ_CHECK;
	}

	if (nirs->flags & NSS_IPV6_RULE_CREATE_FLAG_BRIDGE_FLOW) {
		nicp.params.sync.flags |= NSS_IPV6_CREATE_FLAG_BRIDGE_FLOW;
	}

	if (nirs->flags & NSS_IPV6_RULE_CREATE_FLAG_ROUTED) {
		nicp.params.sync.flags |= NSS_IPV6_CREATE_FLAG_ROUTED;
	}

	switch(nirs->reason) {
	case NSS_IPV6_RULE_SYNC_REASON_FLUSH:
	case NSS_IPV6_RULE_SYNC_REASON_DESTROY:
	case NSS_IPV6_RULE_SYNC_REASON_EVICT:
		nicp.params.sync.final_sync = NSS_IPV6_SYNC_REASON_FLUSH;
		break;

	case NSS_IPV6_RULE_SYNC_REASON_STATS:
		nicp.params.sync.final_sync = NSS_IPV6_SYNC_REASON_STATS;
		break;

	default:
		nss_warning("Bad ipv6 sync reason: %d\n", nirs->reason);
		return;
	}

	/*
	 * Convert ms ticks from the NSS to jiffies.  We know that inc_ticks is small
	 * and we expect HZ to be small too so we can multiply without worrying about
	 * wrap-around problems.  We add a rounding constant to ensure that the different
	 * time bases don't cause truncation errors.
	 */
	nss_assert(HZ <= 100000);
	nicp.params.sync.delta_jiffies = ((nirs->inc_ticks * HZ) + (MSEC_PER_SEC / 2)) / MSEC_PER_SEC;

	/*
	 * Call IPv6 manager callback function
	 */
	if (!nss_tx_rx_ipv6_event_callback) {
		nss_info("%p: IPV6 sync message received before connection manager has registered", nss_ctx);
	} else {
		nss_ipv6_callback_t cb = nss_tx_rx_ipv6_event_callback;
		cb(&nicp);
	}
}

/*
 * nss_rx_metadata_ipv6_rule_establish()
 *	Handle the establishment of an IPv6 rule.
 */
void nss_rx_metadata_ipv6_rule_establish(struct nss_ctx_instance *nss_ctx, struct nss_ipv6_rule_establish *nire)
{
	struct nss_ipv6_cb_params nicp;
	nss_ipv6_callback_t cb;

	// GGG FIXME THIS SHOULD NOT BE A MEMCPY
	nicp.reason = NSS_IPV6_CB_REASON_ESTABLISH;
	memcpy(&nicp.params, nire, sizeof(struct nss_ipv6_establish));

	/*
	 * Call IPv6 manager callback function
	 */
	if (!nss_tx_rx_ipv6_event_callback) {
		nss_info("%p: IPV6 establish message received before connection manager has registered", nss_ctx);
		return;
	}
	cb = nss_tx_rx_ipv6_event_callback;
	cb(&nicp);
	nss_info("%p: Establish message - Index: %d\n", nss_ctx, nire->index);
}

/*
 * nss_tx_create_ipv6_rule()
 *	Create a NSS entry to accelerate the given connection
 */
nss_tx_status_t nss_tx_create_ipv6_rule(void *ctx, struct nss_ipv6_create *unic)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *) ctx;
	struct nss_ipv6_msg nim;
	struct nss_ipv6_rule_create_msg *nircm;

	nss_info("%p: Create IPv6: %pI6:%d, %pI6:%d, p: %d\n", nss_ctx,
		unic->src_ip, unic->src_port, unic->dest_ip, unic->dest_port, unic->protocol);

	nss_cmn_msg_init(&nim.cm, NSS_IPV6_RX_INTERFACE, NSS_IPV6_TX_CREATE_RULE_MSG,
			sizeof(struct nss_ipv6_rule_create_msg), NULL, NULL);

	nircm = &nim.msg.rule_create;

	nircm->rule_flags = 0;
	nircm->valid_flags = 0;

	/*
	 * Copy over the 5 tuple information.
	 */
	nircm->tuple.protocol = (uint8_t)unic->protocol;
	nircm->tuple.flow_ip[0] = unic->src_ip[0];
	nircm->tuple.flow_ip[1] = unic->src_ip[1];
	nircm->tuple.flow_ip[2] = unic->src_ip[2];
	nircm->tuple.flow_ip[3] = unic->src_ip[3];
	nircm->tuple.flow_ident = (uint32_t)unic->src_port;
	nircm->tuple.return_ip[0] = unic->dest_ip[0];
	nircm->tuple.return_ip[1] = unic->dest_ip[1];
	nircm->tuple.return_ip[2] = unic->dest_ip[2];
	nircm->tuple.return_ip[3] = unic->dest_ip[3];
	nircm->tuple.return_ident = (uint32_t)unic->dest_port;

	/*
	 * Copy over the connection rules and set CONN_VALID flag
	 */
	nircm->conn_rule.flow_interface_num = unic->src_interface_num;
	nircm->conn_rule.flow_mtu = unic->from_mtu;
	nircm->conn_rule.return_interface_num = unic->dest_interface_num;
	nircm->conn_rule.return_mtu = unic->to_mtu;
	memcpy(nircm->conn_rule.flow_mac, unic->src_mac, 6);
	memcpy(nircm->conn_rule.return_mac, unic->dest_mac, 6);
	nircm->valid_flags |= NSS_IPV6_RULE_CREATE_CONN_VALID;

	/*
	 * Copy over the pppoe rules and set PPPOE_VALID flag
	 */
	nircm->pppoe_rule.flow_pppoe_session_id = unic->flow_pppoe_session_id;
	memcpy(nircm->pppoe_rule.flow_pppoe_remote_mac, unic->flow_pppoe_remote_mac, ETH_ALEN);
	nircm->pppoe_rule.return_pppoe_session_id = unic->return_pppoe_session_id;
	memcpy(nircm->pppoe_rule.return_pppoe_remote_mac, unic->return_pppoe_remote_mac, ETH_ALEN);
	nircm->valid_flags |= NSS_IPV6_RULE_CREATE_PPPOE_VALID;

	/*
	 * Copy over the tcp rules and set TCP_VALID flag
	 */
	nircm->tcp_rule.flow_window_scale = unic->flow_window_scale;
	nircm->tcp_rule.flow_max_window = unic->flow_max_window;
	nircm->tcp_rule.flow_end = unic->flow_end;
	nircm->tcp_rule.flow_max_end = unic->flow_max_end;
	nircm->tcp_rule.return_window_scale = unic->return_window_scale;
	nircm->tcp_rule.return_max_window = unic->return_max_window;
	nircm->tcp_rule.return_end = unic->return_end;
	nircm->tcp_rule.return_max_end = unic->return_max_end;
	nircm->valid_flags |= NSS_IPV6_RULE_CREATE_TCP_VALID;

	/*
	 * Copy over the vlan rules and set the VLAN_VALID flag
	 */
	nircm->vlan_primary_rule.egress_vlan_tag = unic->egress_vlan_tag;
	nircm->vlan_primary_rule.ingress_vlan_tag = unic->ingress_vlan_tag;
	nircm->valid_flags |= NSS_IPV6_RULE_CREATE_VLAN_VALID;

	/*
	 * Copy over the qos rules and set the QOS_VALID flag
	 */
	nircm->qos_rule.qos_tag = unic->qos_tag;
	nircm->valid_flags |= NSS_IPV6_RULE_CREATE_QOS_VALID;

	if (unic->flags & NSS_IPV6_CREATE_FLAG_NO_SEQ_CHECK) {
		nircm->rule_flags |= NSS_IPV6_RULE_CREATE_FLAG_NO_SEQ_CHECK;
	}

	if (unic->flags & NSS_IPV6_CREATE_FLAG_BRIDGE_FLOW) {
		nircm->rule_flags |= NSS_IPV6_RULE_CREATE_FLAG_BRIDGE_FLOW;
	}

	if (unic->flags & NSS_IPV6_CREATE_FLAG_ROUTED) {
		nircm->rule_flags |= NSS_IPV6_RULE_CREATE_FLAG_ROUTED;
	}
	nss_info("IPV6:%p: valid_flags: %x \n", nss_ctx, nircm->valid_flags);

	return nss_ipv6_tx(nss_ctx, &nim);

}

/*
 * nss_tx_create_ipv6_rule1()
 *	Create a NSS entry to accelerate the given connection
 *  This function has been just added to serve the puropose of backward compatibility
 */
nss_tx_status_t nss_tx_create_ipv6_rule1(void *ctx, struct nss_ipv6_create *unic)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *) ctx;
	struct nss_ipv6_msg nim;
	struct nss_ipv6_rule_create_msg *nircm;

	nss_info("%p: Create IPv6: %pI6:%d, %pI6:%d, p: %d\n", nss_ctx,
		unic->src_ip, unic->src_port, unic->dest_ip, unic->dest_port, unic->protocol);


	nss_cmn_msg_init(&nim.cm, NSS_IPV6_RX_INTERFACE, NSS_IPV6_TX_CREATE_RULE_MSG,
			sizeof(struct nss_ipv6_rule_create_msg), NULL, NULL);

	nircm = &nim.msg.rule_create;

	/*
	 * Initialize the flags
	 */
	nircm->rule_flags = 0;
	nircm->valid_flags = 0;

	/*
	 * Copy over the 5 tuple information.
	 */
	nircm->tuple.protocol = (uint8_t)unic->protocol;
	nircm->tuple.flow_ip[0] = unic->src_ip[0];
	nircm->tuple.flow_ip[1] = unic->src_ip[1];
	nircm->tuple.flow_ip[2] = unic->src_ip[2];
	nircm->tuple.flow_ip[3] = unic->src_ip[3];
	nircm->tuple.flow_ident = (uint32_t)unic->src_port;
	nircm->tuple.return_ip[0] = unic->dest_ip[0];
	nircm->tuple.return_ip[1] = unic->dest_ip[1];
	nircm->tuple.return_ip[2] = unic->dest_ip[2];
	nircm->tuple.return_ip[3] = unic->dest_ip[3];
	nircm->tuple.return_ident = (uint32_t)unic->dest_port;

	/*
	 * Copy over the connection rules and set CONN_VALID flag
	 */
	nircm->conn_rule.flow_interface_num = unic->src_interface_num;
	nircm->conn_rule.flow_mtu = unic->from_mtu;
	nircm->conn_rule.return_interface_num = unic->dest_interface_num;
	nircm->conn_rule.return_mtu = unic->to_mtu;
	memcpy(nircm->conn_rule.flow_mac, unic->src_mac, 6);
	memcpy(nircm->conn_rule.return_mac, unic->dest_mac, 6);
	nircm->valid_flags |= NSS_IPV6_RULE_CREATE_CONN_VALID;

	/*
	 * Copy over the pppoe rules and set PPPOE_VALID flag
	 */
	nircm->pppoe_rule.flow_pppoe_session_id = unic->flow_pppoe_session_id;
	memcpy(nircm->pppoe_rule.flow_pppoe_remote_mac, unic->flow_pppoe_remote_mac, ETH_ALEN);
	nircm->pppoe_rule.return_pppoe_session_id = unic->return_pppoe_session_id;
	memcpy(nircm->pppoe_rule.return_pppoe_remote_mac, unic->return_pppoe_remote_mac, ETH_ALEN);
	nircm->valid_flags |= NSS_IPV6_RULE_CREATE_PPPOE_VALID;

	/*
	 * Copy over the tcp rules and set TCP_VALID flag
	 */
	nircm->tcp_rule.flow_window_scale = unic->flow_window_scale;
	nircm->tcp_rule.flow_max_window = unic->flow_max_window;
	nircm->tcp_rule.flow_end = unic->flow_end;
	nircm->tcp_rule.flow_max_end = unic->flow_max_end;
	nircm->tcp_rule.return_window_scale = unic->return_window_scale;
	nircm->tcp_rule.return_max_window = unic->return_max_window;
	nircm->tcp_rule.return_end = unic->return_end;
	nircm->tcp_rule.return_max_end = unic->return_max_end;
	nircm->valid_flags |= NSS_IPV6_RULE_CREATE_TCP_VALID;

	/*
	 * Copy over the vlan rules and set the VLAN_VALID flag
	 */
	nircm->vlan_primary_rule.egress_vlan_tag = unic->egress_vlan_tag;
	nircm->vlan_primary_rule.ingress_vlan_tag = unic->ingress_vlan_tag;
	nircm->valid_flags |= NSS_IPV6_RULE_CREATE_VLAN_VALID;

	/*
	 * Copy over the qos rules and set the QOS_VALID flag
	 */
	nircm->qos_rule.qos_tag = unic->qos_tag;
	nircm->valid_flags |= NSS_IPV6_RULE_CREATE_QOS_VALID;

	/*
	 * Copy over the dscp marking rules and set the DSCP_MARKING_VALID flag
	 */
	nircm->dscp_rule.dscp_itag = unic->dscp_itag ;
	nircm->dscp_rule.dscp_imask = unic->dscp_imask;
	nircm->dscp_rule.dscp_omask = unic->dscp_omask;
	nircm->dscp_rule.dscp_oval = unic->dscp_oval;
	if (unic->flags & NSS_IPV6_CREATE_FLAG_DSCP_MARKING) {
		nircm->rule_flags |= NSS_IPV6_RULE_CREATE_FLAG_DSCP_MARKING;
		nircm->valid_flags |= NSS_IPV6_RULE_CREATE_DSCP_MARKING_VALID;
	}

	/*
	 * Copy over the vlan marking rules and set the VLAN_MARKING_VALID flag
	 */
	nircm->vlan_primary_rule.vlan_imask = unic->vlan_imask;
	nircm->vlan_primary_rule.vlan_itag = unic->vlan_itag;
	nircm->vlan_primary_rule.vlan_omask = unic->vlan_omask;
	nircm->vlan_primary_rule.vlan_oval = unic->vlan_oval;
	if (unic->flags & NSS_IPV6_CREATE_FLAG_VLAN_MARKING) {
		nircm->rule_flags |= NSS_IPV6_RULE_CREATE_FLAG_VLAN_MARKING;
		nircm->valid_flags |= NSS_IPV6_RULE_CREATE_VLAN_MARKING_VALID;
	}

	if (unic->flags & NSS_IPV6_CREATE_FLAG_NO_SEQ_CHECK) {
		nircm->rule_flags |= NSS_IPV6_RULE_CREATE_FLAG_NO_SEQ_CHECK;
	}

	if (unic->flags & NSS_IPV6_CREATE_FLAG_BRIDGE_FLOW) {
		nircm->rule_flags |= NSS_IPV6_RULE_CREATE_FLAG_BRIDGE_FLOW;
	}

	if (unic->flags & NSS_IPV6_CREATE_FLAG_ROUTED) {
		nircm->rule_flags |= NSS_IPV6_RULE_CREATE_FLAG_ROUTED;
	}

	return nss_ipv6_tx(nss_ctx, &nim);
}

/*
 * nss_tx_destroy_ipv6_rule()
 *	Destroy the given connection in the NSS
 */
nss_tx_status_t nss_tx_destroy_ipv6_rule(void *ctx, struct nss_ipv6_destroy *unid)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *) ctx;
	struct nss_ipv6_msg nim;
	struct nss_ipv6_rule_destroy_msg *nirdm;

	nss_info("%p: Destroy IPv6: %pI6:%d, %pI6:%d, p: %d\n", nss_ctx,
		unid->src_ip, unid->src_port, unid->dest_ip, unid->dest_port, unid->protocol);

	nss_cmn_msg_init(&nim.cm, NSS_IPV6_RX_INTERFACE, NSS_IPV6_TX_DESTROY_RULE_MSG,
			sizeof(struct nss_ipv6_rule_destroy_msg), NULL, NULL);

	nirdm = &nim.msg.rule_destroy;

	nirdm->tuple.protocol = (uint8_t)unid->protocol;
	nirdm->tuple.flow_ip[0] = unid->src_ip[0];
	nirdm->tuple.flow_ip[1] = unid->src_ip[1];
	nirdm->tuple.flow_ip[2] = unid->src_ip[2];
	nirdm->tuple.flow_ip[3] = unid->src_ip[3];
	nirdm->tuple.flow_ident = (uint32_t)unid->src_port;
	nirdm->tuple.return_ip[0] = unid->dest_ip[0];
	nirdm->tuple.return_ip[1] = unid->dest_ip[1];
	nirdm->tuple.return_ip[2] = unid->dest_ip[2];
	nirdm->tuple.return_ip[3] = unid->dest_ip[3];
	nirdm->tuple.return_ident = (uint32_t)unid->dest_port;

	return nss_ipv6_tx(nss_ctx, &nim);
}

/*
 **********************************
 Register/Unregister/Miscellaneous APIs
 **********************************
 */

/*
 * nss_register_ipv6_mgr()
 */
void *nss_register_ipv6_mgr(nss_ipv6_callback_t event_callback)
{
	nss_tx_rx_ipv6_event_callback = event_callback;
	return (void *)&nss_top_main.nss[nss_top_main.ipv6_handler_id];
}

/*
 * nss_unregister_ipv6_mgr()
 */
void nss_unregister_ipv6_mgr(void)
{
	nss_tx_rx_ipv6_event_callback = NULL;
}

/*
 * nss_get_ipv6_mgr_ctx()
 */
void *nss_get_ipv6_mgr_ctx(void)
{
	return (void *)nss_ipv6_get_mgr();
}




EXPORT_SYMBOL(nss_register_ipv6_mgr);
EXPORT_SYMBOL(nss_unregister_ipv6_mgr);
EXPORT_SYMBOL(nss_get_ipv6_mgr_ctx);
EXPORT_SYMBOL(nss_tx_create_ipv6_rule);
EXPORT_SYMBOL(nss_tx_create_ipv6_rule1);
EXPORT_SYMBOL(nss_tx_destroy_ipv6_rule);
