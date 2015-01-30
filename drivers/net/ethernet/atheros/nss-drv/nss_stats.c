/*
 **************************************************************************
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
 * nss_stats.c
 *	NSS stats APIs
 *
 */

#include "nss_core.h"

/*
 * Maximum string length:
 * This should be equal to maximum string size of any stats
 * inclusive of stats value
 */
#define NSS_STATS_MAX_STR_LENGTH 96

/*
 * Global variables/extern declarations
 */
extern struct nss_top_instance nss_top_main;

uint64_t stats_shadow_pppoe_except[NSS_PPPOE_NUM_SESSION_PER_INTERFACE][NSS_PPPOE_EXCEPTION_EVENT_MAX];

/*
 * Private data for every file descriptor
 */
struct nss_stats_data {
	uint32_t if_num;	/**< Interface number for CAPWAP stats */
	uint32_t index;		/**< Index for GRE_REDIR stats */
};

/*
 * Statistics structures
 */

/*
 * nss_stats_str_ipv4
 *	IPv4 stats strings
 */
static int8_t *nss_stats_str_ipv4[NSS_STATS_IPV4_MAX] = {
	"rx_pkts",
	"rx_bytes",
	"tx_pkts",
	"tx_bytes",
	"create_requests",
	"create_collisions",
	"create_invalid_interface",
	"destroy_requests",
	"destroy_misses",
	"hash_hits",
	"hash_reorders",
	"flushes",
	"evictions",
	"fragmentations"
};

/*
 * nss_stats_str_ipv4_reasm
 *	IPv4 reassembly stats strings
 */
static int8_t *nss_stats_str_ipv4_reasm[NSS_STATS_IPV4_REASM_MAX] = {
	"evictions",
	"alloc_fails",
	"timeouts",
};

/*
 * nss_stats_str_ipv6
 *	IPv6 stats strings
 */
static int8_t *nss_stats_str_ipv6[NSS_STATS_IPV6_MAX] = {
	"rx_pkts",
	"rx_bytes",
	"tx_pkts",
	"tx_bytes",
	"create_requests",
	"create_collisions",
	"create_invalid_interface",
	"destroy_requests",
	"destroy_misses",
	"hash_hits",
	"hash_reorders",
	"flushes",
	"evictions",
};

/*
 * nss_stats_str_n2h
 *	N2H stats strings
 */
static int8_t *nss_stats_str_n2h[NSS_STATS_N2H_MAX] = {
	"queue_dropped",
	"ticks",
	"worst_ticks",
	"iterations",
	"pbuf_ocm_alloc_fails",
	"pbuf_ocm_free_count",
	"pbuf_ocm_total_count",
	"pbuf_default_alloc_fails",
	"pbuf_default_free_count",
	"pbuf_default_total_count",
	"payload_fails",
	"h2n_control_packets",
	"h2n_control_bytes",
	"n2h_control_packets",
	"n2h_control_bytes",
	"h2n_data_packets",
	"h2n_data_bytes",
	"n2h_data_packets",
	"n2h_data_bytes",
};

/*
 * nss_stats_str_lso_rx
 *	LSO_RX stats strings
 */
static int8_t *nss_stats_str_lso_rx[NSS_STATS_LSO_RX_MAX] = {
	"tx_dropped",
	"dropped",
	"pbuf_alloc_fail",
	"pbuf_reference_fail"
};

/*
 * nss_stats_str_drv
 *	Host driver stats strings
 */
static int8_t *nss_stats_str_drv[NSS_STATS_DRV_MAX] = {
	"nbuf_alloc_errors",
	"tx_queue_full[0]",
	"tx_queue_full[1]",
	"tx_buffers_empty",
	"tx_buffers_pkt",
	"tx_buffers_cmd",
	"tx_buffers_crypto",
	"rx_buffers_empty",
	"rx_buffers_pkt",
	"rx_buffers_cmd_resp",
	"rx_buffers_status_sync",
	"rx_buffers_crypto",
	"rx_buffers_virtual",
	"tx_skb_simple",
	"tx_skb_nr_frags",
	"tx_skb_fraglist",
	"rx_skb_simple",
	"rx_skb_nr_frags",
	"rx_skb_fraglist",
	"rx_bad_desciptor"
};

/*
 * nss_stats_str_pppoe
 *	PPPoE stats strings
 */
static int8_t *nss_stats_str_pppoe[NSS_STATS_PPPOE_MAX] = {
	"create_requests",
	"create_failures",
	"destroy_requests",
	"destroy_misses"
};

/*
 * nss_stats_str_gmac
 *	GMAC stats strings
 */
static int8_t *nss_stats_str_gmac[NSS_STATS_GMAC_MAX] = {
	"ticks",
	"worst_ticks",
	"iterations"
};

/*
 * nss_stats_str_node
 *	Interface stats strings per node
 */
static int8_t *nss_stats_str_node[NSS_STATS_NODE_MAX] = {
	"rx_packets",
	"rx_bytes",
	"rx_dropped",
	"tx_packets",
	"tx_bytes"
};

/*
 * nss_stats_str_eth_rx
 *	eth_rx stats strings
 */
static int8_t *nss_stats_str_eth_rx[NSS_STATS_ETH_RX_MAX] = {
	"ticks",
	"worst_ticks",
	"iterations"
};

/*
 * nss_stats_str_if_exception_unknown
 *	Interface stats strings for unknown exceptions
 */
static int8_t *nss_stats_str_if_exception_eth_rx[NSS_EXCEPTION_EVENT_ETH_RX_MAX] = {
	"UNKNOWN_L3_PROTOCOL"
};

/*
 * nss_stats_str_if_exception_ipv4
 *	Interface stats strings for ipv4 exceptions
 */
static int8_t *nss_stats_str_if_exception_ipv4[NSS_EXCEPTION_EVENT_IPV4_MAX] = {
	"IPV4_ICMP_HEADER_INCOMPLETE",
	"IPV4_ICMP_UNHANDLED_TYPE",
	"IPV4_ICMP_IPV4_HEADER_INCOMPLETE",
	"IPV4_ICMP_IPV4_UDP_HEADER_INCOMPLETE",
	"IPV4_ICMP_IPV4_TCP_HEADER_INCOMPLETE",
	"IPV4_ICMP_IPV4_UNKNOWN_PROTOCOL",
	"IPV4_ICMP_NO_ICME",
	"IPV4_ICMP_FLUSH_TO_HOST",
	"IPV4_TCP_HEADER_INCOMPLETE",
	"IPV4_TCP_NO_ICME",
	"IPV4_TCP_IP_OPTION",
	"IPV4_TCP_IP_FRAGMENT",
	"IPV4_TCP_SMALL_TTL",
	"IPV4_TCP_NEEDS_FRAGMENTATION",
	"IPV4_TCP_FLAGS",
	"IPV4_TCP_SEQ_EXCEEDS_RIGHT_EDGE",
	"IPV4_TCP_SMALL_DATA_OFFS",
	"IPV4_TCP_BAD_SACK",
	"IPV4_TCP_BIG_DATA_OFFS",
	"IPV4_TCP_SEQ_BEFORE_LEFT_EDGE",
	"IPV4_TCP_ACK_EXCEEDS_RIGHT_EDGE",
	"IPV4_TCP_ACK_BEFORE_LEFT_EDGE",
	"IPV4_UDP_HEADER_INCOMPLETE",
	"IPV4_UDP_NO_ICME",
	"IPV4_UDP_IP_OPTION",
	"IPV4_UDP_IP_FRAGMENT",
	"IPV4_UDP_SMALL_TTL",
	"IPV4_UDP_NEEDS_FRAGMENTATION",
	"IPV4_WRONG_TARGET_MAC",
	"IPV4_HEADER_INCOMPLETE",
	"IPV4_BAD_TOTAL_LENGTH",
	"IPV4_BAD_CHECKSUM",
	"IPV4_NON_INITIAL_FRAGMENT",
	"IPV4_DATAGRAM_INCOMPLETE",
	"IPV4_OPTIONS_INCOMPLETE",
	"IPV4_UNKNOWN_PROTOCOL",
	"IPV4_ESP_HEADER_INCOMPLETE",
	"IPV4_ESP_NO_ICME",
	"IPV4_ESP_IP_OPTION",
	"IPV4_ESP_IP_FRAGMENT",
	"IPV4_ESP_SMALL_TTL",
	"IPV4_ESP_NEEDS_FRAGMENTATION",
	"IPV4_INGRESS_VID_MISMATCH",
	"IPV4_INGRESS_VID_MISSING",
	"IPV4_6RD_NO_ICME",
	"IPV4_6RD_IP_OPTION",
	"IPV4_6RD_IP_FRAGMENT",
	"IPV4_6RD_NEEDS_FRAGMENTATION",
	"IPV4_DSCP_MARKING_MISMATCH",
	"IPV4_VLAN_MARKING_MISMATCH",
	"IPV4_INTERFACE_MISMATCH",
	"IPV4_GRE_HEADER_INCOMPLETE",
	"IPV4_GRE_NO_ICME",
	"IPV4_GRE_IP_OPTION",
	"IPV4_GRE_IP_FRAGMENT",
	"IPV4_GRE_SMALL_TTL",
	"IPV4_GRE_NEEDS_FRAGMENTATION",
	"IPV4_FRAG_DF_SET",
	"IPV4_FRAG_FAIL",
	"IPV4_DESTROY"
};

/*
 * nss_stats_str_if_exception_ipv6
 *	Interface stats strings for ipv6 exceptions
 */
static int8_t *nss_stats_str_if_exception_ipv6[NSS_EXCEPTION_EVENT_IPV6_MAX] = {
	"IPV6_ICMP_HEADER_INCOMPLETE",
	"IPV6_ICMP_UNHANDLED_TYPE",
	"IPV6_ICMP_IPV6_HEADER_INCOMPLETE",
	"IPV6_ICMP_IPV6_UDP_HEADER_INCOMPLETE",
	"IPV6_ICMP_IPV6_TCP_HEADER_INCOMPLETE",
	"IPV6_ICMP_IPV6_UNKNOWN_PROTOCOL",
	"IPV6_ICMP_NO_ICME",
	"IPV6_ICMP_FLUSH_TO_HOST",
	"IPV6_TCP_HEADER_INCOMPLETE",
	"IPV6_TCP_NO_ICME",
	"IPV6_TCP_SMALL_HOP_LIMIT",
	"IPV6_TCP_NEEDS_FRAGMENTATION",
	"IPV6_TCP_FLAGS",
	"IPV6_TCP_SEQ_EXCEEDS_RIGHT_EDGE",
	"IPV6_TCP_SMALL_DATA_OFFS",
	"IPV6_TCP_BAD_SACK",
	"IPV6_TCP_BIG_DATA_OFFS",
	"IPV6_TCP_SEQ_BEFORE_LEFT_EDGE",
	"IPV6_TCP_ACK_EXCEEDS_RIGHT_EDGE",
	"IPV6_TCP_ACK_BEFORE_LEFT_EDGE",
	"IPV6_UDP_HEADER_INCOMPLETE",
	"IPV6_UDP_NO_ICME",
	"IPV6_UDP_SMALL_HOP_LIMIT",
	"IPV6_UDP_NEEDS_FRAGMENTATION",
	"IPV6_WRONG_TARGET_MAC",
	"IPV6_HEADER_INCOMPLETE",
	"IPV6_UNKNOWN_PROTOCOL",
	"IPV6_INGRESS_VID_MISMATCH",
	"IPV6_INGRESS_VID_MISSING",
	"IPV6_DSCP_MARKING_MISMATCH",
	"IPV6_VLAN_MARKING_MISMATCH",
	"IPV6_INTERFACE_MISMATCH",
	"IPV6_DESTROY"
};

/*
 * nss_stats_str_if_exception_pppoe
 *	Interface stats strings for PPPoE exceptions
 */
static int8_t *nss_stats_str_if_exception_pppoe[NSS_PPPOE_EXCEPTION_EVENT_MAX] = {
	"PPPOE_WRONG_VERSION_OR_TYPE",
	"PPPOE_WRONG_CODE",
	"PPPOE_HEADER_INCOMPLETE",
	"PPPOE_UNSUPPORTED_PPP_PROTOCOL",
	"PPPOE_INTERFACE_MISMATCH"
};

/*
 * nss_stats_ipv4_read()
 *	Read IPV4 stats
 */
static ssize_t nss_stats_ipv4_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	int32_t i;
	/*
	 * max output lines = #stats + start tag line + end tag line + three blank lines
	 */
	uint32_t max_output_lines = (NSS_STATS_NODE_MAX + 2) + (NSS_STATS_IPV4_MAX + 3) + (NSS_EXCEPTION_EVENT_IPV4_MAX + 3) + 5;
	size_t size_al = NSS_STATS_MAX_STR_LENGTH * max_output_lines;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;
	uint64_t *stats_shadow;

	char *lbuf = kzalloc(size_al, GFP_KERNEL);
	if (unlikely(lbuf == NULL)) {
		nss_warning("Could not allocate memory for local statistics buffer");
		return 0;
	}

	/*
	 * Note: The assumption here is that exception event count is larger than other statistics count for IPv4
	 */
	stats_shadow = kzalloc(NSS_EXCEPTION_EVENT_IPV4_MAX * 8, GFP_KERNEL);
	if (unlikely(stats_shadow == NULL)) {
		nss_warning("Could not allocate memory for local shadow buffer");
		kfree(lbuf);
		return 0;
	}

	size_wr = scnprintf(lbuf, size_al, "ipv4 stats start:\n\n");

	/*
	 * Common node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "common node stats:\n\n");
	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_node[NSS_IPV4_RX_INTERFACE][i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_node[i], stats_shadow[i]);
	}

	/*
	 * IPv4 node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nipv4 node stats:\n\n");

	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_IPV4_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_ipv4[i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_IPV4_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_ipv4[i], stats_shadow[i]);
	}

	/*
	 * Exception stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nipv4 exception stats:\n\n");

	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_EXCEPTION_EVENT_IPV4_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_if_exception_ipv4[i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_EXCEPTION_EVENT_IPV4_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_if_exception_ipv4[i], stats_shadow[i]);
	}

	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nipv4 stats end\n\n");
	bytes_read = simple_read_from_buffer(ubuf, sz, ppos, lbuf, strlen(lbuf));
	kfree(lbuf);
	kfree(stats_shadow);

	return bytes_read;
}

/*
 * nss_stats_ipv4_reasm_read()
 *	Read IPV4 reassembly stats
 */
static ssize_t nss_stats_ipv4_reasm_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	int32_t i;
	/*
	 * max output lines = #stats + start tag line + end tag line + three blank lines
	 */
	uint32_t max_output_lines = (NSS_STATS_NODE_MAX + 2) + (NSS_STATS_IPV4_REASM_MAX + 3) + 5;
	size_t size_al = NSS_STATS_MAX_STR_LENGTH * max_output_lines;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;
	uint64_t *stats_shadow;

	char *lbuf = kzalloc(size_al, GFP_KERNEL);
	if (unlikely(lbuf == NULL)) {
		nss_warning("Could not allocate memory for local statistics buffer");
		return 0;
	}

	stats_shadow = kzalloc(NSS_STATS_IPV4_REASM_MAX * 8, GFP_KERNEL);
	if (unlikely(stats_shadow == NULL)) {
		nss_warning("Could not allocate memory for local shadow buffer");
		kfree(lbuf);
		return 0;
	}

	size_wr = scnprintf(lbuf, size_al, "ipv4 reasm stats start:\n\n");

	/*
	 * Common node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "common node stats:\n\n");
	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_node[NSS_IPV4_REASM_INTERFACE][i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_node[i], stats_shadow[i]);
	}

	/*
	 * IPv4 reasm node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nipv4 reasm node stats:\n\n");

	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_IPV4_REASM_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_ipv4_reasm[i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_IPV4_REASM_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_ipv4_reasm[i], stats_shadow[i]);
	}

	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nipv4 reasm stats end\n\n");
	bytes_read = simple_read_from_buffer(ubuf, sz, ppos, lbuf, strlen(lbuf));
	kfree(lbuf);
	kfree(stats_shadow);

	return bytes_read;
}

/*
 * nss_stats_ipv6_read()
 *	Read IPV6 stats
 */
static ssize_t nss_stats_ipv6_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	int32_t i;

	/*
	 * max output lines = #stats + start tag line + end tag line + three blank lines
	 */
	uint32_t max_output_lines = (NSS_STATS_NODE_MAX + 2) + (NSS_STATS_IPV6_MAX + 3) + (NSS_EXCEPTION_EVENT_IPV6_MAX + 3) + 5;
	size_t size_al = NSS_STATS_MAX_STR_LENGTH * max_output_lines;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;
	uint64_t *stats_shadow;

	char *lbuf = kzalloc(size_al, GFP_KERNEL);
	if (unlikely(lbuf == NULL)) {
		nss_warning("Could not allocate memory for local statistics buffer");
		return 0;
	}

	/*
	 * Note: The assumption here is that exception event count is larger than other statistics count for IPv4
	 */
	stats_shadow = kzalloc(NSS_EXCEPTION_EVENT_IPV6_MAX * 8, GFP_KERNEL);
	if (unlikely(stats_shadow == NULL)) {
		nss_warning("Could not allocate memory for local shadow buffer");
		kfree(lbuf);
		return 0;
	}

	size_wr = scnprintf(lbuf, size_al, "ipv6 stats start:\n\n");

	/*
	 * Common node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "common node stats:\n\n");
	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_node[NSS_IPV6_RX_INTERFACE][i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_node[i], stats_shadow[i]);
	}

	/*
	 * IPv6 node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nipv6 node stats:\n\n");

	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_IPV6_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_ipv6[i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_IPV6_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_ipv6[i], stats_shadow[i]);
	}

	/*
	 * Exception stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nipv6 exception stats:\n\n");

	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_EXCEPTION_EVENT_IPV6_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_if_exception_ipv6[i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_EXCEPTION_EVENT_IPV6_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_if_exception_ipv6[i], stats_shadow[i]);
	}

	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,"\nipv6 stats end\n\n");
	bytes_read = simple_read_from_buffer(ubuf, sz, ppos, lbuf, strlen(lbuf));
	kfree(lbuf);
	kfree(stats_shadow);

	return bytes_read;
}

/*
 * nss_stats_eth_rx_read()
 *	Read ETH_RX stats
 */
static ssize_t nss_stats_eth_rx_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	int32_t i;

	/*
	 * max output lines = #stats + start tag line + end tag line + three blank lines
	 */
	uint32_t max_output_lines = (NSS_STATS_NODE_MAX + 2) + (NSS_STATS_ETH_RX_MAX + 3) + (NSS_EXCEPTION_EVENT_ETH_RX_MAX + 3) + 5;
	size_t size_al = NSS_STATS_MAX_STR_LENGTH * max_output_lines;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;
	uint64_t *stats_shadow;

	char *lbuf = kzalloc(size_al, GFP_KERNEL);
	if (unlikely(lbuf == NULL)) {
		nss_warning("Could not allocate memory for local statistics buffer");
		return 0;
	}

	/*
	 * Note: The assumption here is that we do not have more than 64 stats
	 */
	stats_shadow = kzalloc(64 * 8, GFP_KERNEL);
	if (unlikely(stats_shadow == NULL)) {
		nss_warning("Could not allocate memory for local shadow buffer");
		kfree(lbuf);
		return 0;
	}

	size_wr = scnprintf(lbuf, size_al,"eth_rx stats start:\n\n");

	/*
	 * Common node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "common node stats:\n\n");
	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_node[NSS_ETH_RX_INTERFACE][i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_node[i], stats_shadow[i]);
	}

	/*
	 * eth_rx node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\neth_rx node stats:\n\n");
	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_ETH_RX_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_eth_rx[i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_ETH_RX_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_eth_rx[i], stats_shadow[i]);
	}

	/*
	 * Exception stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\neth_rx exception stats:\n\n");

	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_EXCEPTION_EVENT_ETH_RX_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_if_exception_eth_rx[i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_EXCEPTION_EVENT_ETH_RX_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_if_exception_eth_rx[i], stats_shadow[i]);
	}

	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,"\neth_rx stats end\n\n");
	bytes_read = simple_read_from_buffer(ubuf, sz, ppos, lbuf, strlen(lbuf));
	kfree(lbuf);
	kfree(stats_shadow);

	return bytes_read;
}

/*
 * nss_stats_n2h_read()
 *	Read N2H stats
 */
static ssize_t nss_stats_n2h_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	int32_t i;

	/*
	 * max output lines = #stats + start tag line + end tag line + three blank lines
	 */
	uint32_t max_output_lines = (NSS_STATS_NODE_MAX + 2) + (NSS_STATS_N2H_MAX + 3) + 5;
	size_t size_al = NSS_STATS_MAX_STR_LENGTH * max_output_lines;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;
	uint64_t *stats_shadow;
	int max = NSS_STATS_N2H_MAX - NSS_STATS_NODE_MAX;

	char *lbuf = kzalloc(size_al, GFP_KERNEL);
	if (unlikely(lbuf == NULL)) {
		nss_warning("Could not allocate memory for local statistics buffer");
		return 0;
	}

	stats_shadow = kzalloc(NSS_STATS_N2H_MAX * 8, GFP_KERNEL);
	if (unlikely(stats_shadow == NULL)) {
		nss_warning("Could not allocate memory for local shadow buffer");
		kfree(lbuf);
		return 0;
	}

	size_wr = scnprintf(lbuf, size_al, "n2h stats start:\n\n");

	/*
	 * Common node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "common node stats:\n\n");
	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		stats_shadow[i] = nss_top_main.nss[0].stats_n2h[i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_node[i], stats_shadow[i]);
	}

	/*
	 * N2H node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nn2h node stats:\n\n");
	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = NSS_STATS_NODE_MAX; (i < NSS_STATS_N2H_MAX); i++) {
		stats_shadow[i] = nss_top_main.nss[0].stats_n2h[i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; i < max; i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_n2h[i], stats_shadow[i + NSS_STATS_NODE_MAX]);
	}

	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nn2h stats end\n\n");
	bytes_read = simple_read_from_buffer(ubuf, sz, ppos, lbuf, strlen(lbuf));
	kfree(lbuf);
	kfree(stats_shadow);

	return bytes_read;
}

/*
 * nss_stats_lso_rx_read()
 *	Read LSO_RX stats
 */
static ssize_t nss_stats_lso_rx_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	int32_t i;

	/*
	 * max output lines = #stats + start tag line + end tag line + three blank lines
	 */
	uint32_t max_output_lines = (NSS_STATS_NODE_MAX + 2) + (NSS_STATS_LSO_RX_MAX + 3) + 5;
	size_t size_al = NSS_STATS_MAX_STR_LENGTH * max_output_lines;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;
	uint64_t *stats_shadow;

	char *lbuf = kzalloc(size_al, GFP_KERNEL);
	if (unlikely(lbuf == NULL)) {
		nss_warning("Could not allocate memory for local statistics buffer");
		return 0;
	}

	stats_shadow = kzalloc(NSS_STATS_LSO_RX_MAX * 8, GFP_KERNEL);
	if (unlikely(stats_shadow == NULL)) {
		nss_warning("Could not allocate memory for local shadow buffer");
		kfree(lbuf);
		return 0;
	}

	size_wr = scnprintf(lbuf, size_al, "lso_rx stats start:\n\n");

	/*
	 * Common node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "common node stats:\n\n");
	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_node[NSS_LSO_RX_INTERFACE][i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_node[i], stats_shadow[i]);
	}

	/*
	 * lso_rx node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nlso_rx node stats:\n\n");
	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_LSO_RX_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_lso_rx[i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; i < NSS_STATS_LSO_RX_MAX; i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_lso_rx[i], stats_shadow[i]);
	}

	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nlso_rx stats end\n\n");
	bytes_read = simple_read_from_buffer(ubuf, sz, ppos, lbuf, strlen(lbuf));
	kfree(lbuf);
	kfree(stats_shadow);

	return bytes_read;
}

/*
 * nss_stats_drv_read()
 *	Read HLOS driver stats
 */
static ssize_t nss_stats_drv_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	int32_t i;

	/*
	 * max output lines = #stats + start tag line + end tag line + three blank lines
	 */
	uint32_t max_output_lines = NSS_STATS_DRV_MAX + 5;
	size_t size_al = NSS_STATS_MAX_STR_LENGTH * max_output_lines;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;
	uint64_t *stats_shadow;

	char *lbuf = kzalloc(size_al, GFP_KERNEL);
	if (unlikely(lbuf == NULL)) {
		nss_warning("Could not allocate memory for local statistics buffer");
		return 0;
	}

	stats_shadow = kzalloc(NSS_STATS_DRV_MAX * 8, GFP_KERNEL);
	if (unlikely(stats_shadow == NULL)) {
		nss_warning("Could not allocate memory for local shadow buffer");
		kfree(lbuf);
		return 0;
	}

	size_wr = scnprintf(lbuf, size_al, "drv stats start:\n\n");
	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_DRV_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_drv[i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_DRV_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_drv[i], stats_shadow[i]);
	}

	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\ndrv stats end\n\n");
	bytes_read = simple_read_from_buffer(ubuf, sz, ppos, lbuf, strlen(lbuf));
	kfree(lbuf);
	kfree(stats_shadow);

	return bytes_read;
}

/*
 * nss_stats_pppoe_read()
 *	Read PPPoE stats
 */
static ssize_t nss_stats_pppoe_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	int32_t i, j, k;

	/*
	 * max output lines = #stats + start tag line + end tag line + three blank lines
	 */
	uint32_t max_output_lines = (NSS_STATS_NODE_MAX + 2) + (NSS_STATS_PPPOE_MAX + 3) +
					((NSS_MAX_PHYSICAL_INTERFACES * NSS_PPPOE_NUM_SESSION_PER_INTERFACE * (NSS_PPPOE_EXCEPTION_EVENT_MAX + 5)) + 3) + 5;
	size_t size_al = NSS_STATS_MAX_STR_LENGTH * max_output_lines;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;
	uint64_t *stats_shadow;

	char *lbuf = kzalloc(size_al, GFP_KERNEL);
	if (unlikely(lbuf == NULL)) {
		nss_warning("Could not allocate memory for local statistics buffer");
		return 0;
	}

	stats_shadow = kzalloc(64 * 8, GFP_KERNEL);
	if (unlikely(stats_shadow == NULL)) {
		nss_warning("Could not allocate memory for local shadow buffer");
		kfree(lbuf);
		return 0;
	}

	size_wr = scnprintf(lbuf, size_al, "pppoe stats start:\n\n");

	/*
	 * Common node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "common node stats:\n\n");
	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_node[NSS_PPPOE_RX_INTERFACE][i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
				 "%s = %llu\n", nss_stats_str_node[i], stats_shadow[i]);
	}

	/*
	 * PPPoE node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "pppoe node stats:\n\n");
	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_PPPOE_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_pppoe[i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_PPPOE_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_pppoe[i], stats_shadow[i]);
	}

	/*
	 * Exception stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nException PPPoE:\n\n");

	for (j = 0; j < NSS_MAX_PHYSICAL_INTERFACES; j++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nInterface %d:\n\n", j);

		spin_lock_bh(&nss_top_main.stats_lock);
		for (k = 0; k < NSS_PPPOE_NUM_SESSION_PER_INTERFACE; k++) {
			for (i = 0; (i < NSS_PPPOE_EXCEPTION_EVENT_MAX); i++) {
				stats_shadow_pppoe_except[k][i] = nss_top_main.stats_if_exception_pppoe[j][k][i];
			}
		}

		spin_unlock_bh(&nss_top_main.stats_lock);

		for (k = 0; k < NSS_PPPOE_NUM_SESSION_PER_INTERFACE; k++) {
			size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "%d. Session\n", k);
			for (i = 0; (i < NSS_PPPOE_EXCEPTION_EVENT_MAX); i++) {
				size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
						"%s = %llu\n",
						nss_stats_str_if_exception_pppoe[i],
						stats_shadow_pppoe_except[k][i]);
			}
		}

		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\npppoe stats end\n\n");
	}

	bytes_read = simple_read_from_buffer(ubuf, sz, ppos, lbuf, strlen(lbuf));
	kfree(lbuf);
	kfree(stats_shadow);

	return bytes_read;
}

/*
 * nss_stats_gmac_read()
 *	Read GMAC stats
 */
static ssize_t nss_stats_gmac_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	uint32_t i, id;

	/*
	 * max output lines = ((#stats + start tag + one blank) * #GMACs) + start/end tag + 3 blank
	 */
	uint32_t max_output_lines = ((NSS_STATS_GMAC_MAX + 2) * NSS_MAX_PHYSICAL_INTERFACES) + 5;
	size_t size_al = NSS_STATS_MAX_STR_LENGTH * max_output_lines;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;
	uint64_t *stats_shadow;

	char *lbuf = kzalloc(size_al, GFP_KERNEL);
	if (unlikely(lbuf == NULL)) {
		nss_warning("Could not allocate memory for local statistics buffer");
		return 0;
	}

	stats_shadow = kzalloc(NSS_STATS_GMAC_MAX * 8, GFP_KERNEL);
	if (unlikely(stats_shadow == NULL)) {
		nss_warning("Could not allocate memory for local shadow buffer");
		kfree(lbuf);
		return 0;
	}

	size_wr = scnprintf(lbuf, size_al, "gmac stats start:\n\n");

	for (id = 0; id < NSS_MAX_PHYSICAL_INTERFACES; id++) {
		spin_lock_bh(&nss_top_main.stats_lock);
		for (i = 0; (i < NSS_STATS_GMAC_MAX); i++) {
			stats_shadow[i] = nss_top_main.stats_gmac[id][i];
		}

		spin_unlock_bh(&nss_top_main.stats_lock);

		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "GMAC ID: %d\n", id);
		for (i = 0; (i < NSS_STATS_GMAC_MAX); i++) {
			size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_gmac[i], stats_shadow[i]);
		}
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,"\n");
	}

	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\ngmac stats end\n\n");
	bytes_read = simple_read_from_buffer(ubuf, sz, ppos, lbuf, strlen(lbuf));
	kfree(lbuf);
	kfree(stats_shadow);

	return bytes_read;
}

/*
 * nss_stats_sjack_read()
 *	Read SJACK stats
 */
static ssize_t nss_stats_sjack_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	int32_t i;
	/*
	 * max output lines = #stats + start tag line + end tag line + three blank lines
	 */
	uint32_t max_output_lines = NSS_STATS_NODE_MAX + 5;
	size_t size_al = NSS_STATS_MAX_STR_LENGTH * max_output_lines;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;
	uint64_t *stats_shadow;

	char *lbuf = kzalloc(size_al, GFP_KERNEL);
	if (unlikely(lbuf == NULL)) {
		nss_warning("Could not allocate memory for local statistics buffer");
		return 0;
	}

	stats_shadow = kzalloc(NSS_STATS_NODE_MAX * 8, GFP_KERNEL);
	if (unlikely(stats_shadow == NULL)) {
		nss_warning("Could not allocate memory for local shadow buffer");
		kfree(lbuf);
		return 0;
	}

	size_wr = scnprintf(lbuf, size_al, "sjack stats start:\n\n");

	/*
	 * Common node stats
	 */
	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "common node stats:\n\n");
	spin_lock_bh(&nss_top_main.stats_lock);
	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		stats_shadow[i] = nss_top_main.stats_node[NSS_SJACK_INTERFACE][i];
	}

	spin_unlock_bh(&nss_top_main.stats_lock);

	for (i = 0; (i < NSS_STATS_NODE_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %llu\n", nss_stats_str_node[i], stats_shadow[i]);
	}

	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "\nsjack stats end\n\n");

	bytes_read = simple_read_from_buffer(ubuf, sz, ppos, lbuf, strlen(lbuf));
	kfree(lbuf);
	kfree(stats_shadow);

	return bytes_read;
}

/*
 * Make a row for CAPWAP encap stats.
 */
static ssize_t nss_stats_capwap_encap(char *line, int len, int i, struct nss_capwap_tunnel_stats *s)
{
	char *header[] = { "TX Packets", "TX Bytes", "TX Drops", "Fragments", "QFull", "MemFail", "Unknown" };
	uint64_t tcnt = 0;

	switch (i) {
	case 0:
		tcnt = s->pnode_stats.tx_packets;
		break;
	case 1:
		tcnt = s->pnode_stats.tx_bytes;
		break;
	case 2:
		tcnt = s->tx_dropped;
		break;
	case 3:
		tcnt = s->tx_segments;
		break;
	case 4:
		tcnt = s->tx_queue_full_drops;
		break;
	case 5:
		tcnt = s->tx_mem_failure_drops;
		break;
	default:
		i = 6;
		break;
	}

	return (snprintf(line, len, "%14s %llu\n", header[i], tcnt));
}

/*
 * Make a row for CAPWAP decap stats.
 */
static ssize_t nss_stats_capwap_decap(char *line, int len, int i, struct nss_capwap_tunnel_stats *s)
{
	char *header[] = { "RX Packets", "RX Bytes", "RX Dropped", "DTLS pkts", "Fragments", "OSzDrop", "FTimeout", "FDup", "QFull", "MemFail", "Unknown" };
	uint64_t tcnt = 0;

	switch(i) {
	case 0:
		tcnt = s->pnode_stats.rx_packets;
		break;
	case 1:
		tcnt = s->pnode_stats.rx_bytes;
		break;
	case 2:
		tcnt = s->pnode_stats.rx_dropped;
		break;
	case 3:
		tcnt = s->dtls_pkts;
		break;
	case 4:
		tcnt = s->rx_segments;
		break;
	case 5:
		tcnt = s->oversize_drops;
		break;
	case 6:
		tcnt = s->frag_timeout_drops;
		break;
	case 7:
		tcnt = s->rx_dup_frag;
		break;
	case 8:
		tcnt = s->rx_queue_full_drops;
		return (snprintf(line, len, "%14s: %llu (n2h: %llu)\n", header[i], tcnt, s->rx_n2h_queue_full_drops));
	case 9:
		tcnt = s->rx_mem_failure_drops;
		break;
	default:
		i = 10;
		break;
	}

	return (snprintf(line, len, "%14s: %llu\n", header[i], tcnt));
}

/*
 * nss_stats_capwap_read()
 *	Read CAPWAP stats
 */
static ssize_t nss_stats_capwap_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos, uint16_t type)
{
	struct nss_stats_data *data = fp->private_data;
	ssize_t bytes_read = 0;
	struct nss_capwap_tunnel_stats stats;
	size_t bytes;
	char line[80];
	int start, end;
	uint32_t if_num = NSS_DYNAMIC_IF_START;
	uint32_t max_if_num = NSS_DYNAMIC_IF_START + NSS_MAX_DYNAMIC_INTERFACES;

	if (data) {
		if_num = data->if_num;
	}

	/*
	 * If we are done accomodating all the CAPWAP tunnels.
	 */
	if (if_num > max_if_num) {
		return 0;
	}

	for (; if_num <= max_if_num; if_num++) {
		bool isthere;

		if (nss_is_dynamic_interface(if_num) == false) {
			continue;
		}

		if (nss_dynamic_interface_get_type(if_num) != NSS_DYNAMIC_INTERFACE_TYPE_CAPWAP) {
			continue;
		}

		/*
		 * If CAPWAP tunnel does not exists, then isthere will be false.
		 */
		isthere = nss_capwap_get_stats(if_num, &stats);
		if (!isthere) {
			continue;
		}

		bytes = snprintf(line, sizeof(line), "(%2d) %9s %s\n", if_num, "Stats", "Total");
		if ((bytes_read + bytes) > sz) {
			break;
		}

		if (copy_to_user(ubuf + bytes_read, line, bytes) != 0) {
			bytes_read = -EFAULT;
			goto fail;
		}
		bytes_read += bytes;
		start = 0;
		if (type == 1) {
			end = 5;	/* encap */
		} else {
			end = 9;	/* decap */
		}
		while (bytes_read < sz && start < end) {
			if (type == 1) {
				bytes = nss_stats_capwap_encap(line, sizeof(line), start, &stats);
			} else {
				bytes = nss_stats_capwap_decap(line, sizeof(line), start, &stats);
			}

			if ((bytes_read + bytes) > sz)
				break;

			if (copy_to_user(ubuf + bytes_read, line, bytes) != 0) {
				bytes_read = -EFAULT;
				goto fail;
			}

			bytes_read += bytes;
			start++;
		}
	}

	if (bytes_read > 0) {
		*ppos = bytes_read;
	}

	if (data) {
		data->if_num = if_num;
	}
fail:
	return bytes_read;
}

/*
 * nss_stats_capwap_decap_read()
 *	Read CAPWAP decap stats
 */
static ssize_t nss_stats_capwap_decap_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	return (nss_stats_capwap_read(fp, ubuf, sz, ppos, 0));
}

/*
 * nss_stats_capwap_encap_read()
 *	Read CAPWAP encap stats
 */
static ssize_t nss_stats_capwap_encap_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	return (nss_stats_capwap_read(fp, ubuf, sz, ppos, 1));
}

/*
 * nss_stats_gre_redir()
 * 	Make a row for GRE_REDIR stats.
 */
static ssize_t nss_stats_gre_redir(char *line, int len, int i, struct nss_gre_redir_tunnel_stats *s)
{
	char *header[] = { "TX Packets", "TX Bytes", "TX Drops", "RX Packets", "RX Bytes", "Rx Drops" };
	uint64_t tcnt = 0;

	switch (i) {
	case 0:
		tcnt = s->node_stats.tx_packets;
		break;
	case 1:
		tcnt = s->node_stats.tx_bytes;
		break;
	case 2:
		tcnt = s->tx_dropped;
		break;
	case 3:
		tcnt = s->node_stats.rx_packets;
		break;
	case 4:
		tcnt = s->node_stats.rx_bytes;
		break;
	case 5:
		tcnt = s->node_stats.rx_dropped;
		break;
	default:
		i = 6;
		break;
	}

	return (snprintf(line, len, "%s = %llu\n", header[i], tcnt));
}

/*
 * nss_stats_gre_redir_read()
 * 	READ gre_redir tunnel stats.
 */
static ssize_t nss_stats_gre_redir_read(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	struct nss_stats_data *data = fp->private_data;
	ssize_t bytes_read = 0;
	struct nss_gre_redir_tunnel_stats stats;
	size_t bytes;
	char line[80];
	int start, end;
	int index = 0;

	if (data) {
		index = data->index;
	}

	/*
	 * If we are done accomodating all the GRE_REDIR tunnels.
	 */
	if (index >= NSS_GRE_REDIR_MAX_INTERFACES) {
		return 0;
	}

	for (; index < NSS_GRE_REDIR_MAX_INTERFACES; index++) {
		bool isthere;

		/*
		 * If gre_redir tunnel does not exists, then isthere will be false.
		 */
		isthere = nss_gre_redir_get_stats(index, &stats);
		if (!isthere) {
			continue;
		}

		bytes = snprintf(line, sizeof(line), "\nTunnel if_num: %2d\n", stats.if_num);
		if ((bytes_read + bytes) > sz) {
			break;
		}

		if (copy_to_user(ubuf + bytes_read, line, bytes) != 0) {
			bytes_read = -EFAULT;
			goto fail;
		}
		bytes_read += bytes;
		start = 0;
		end = 6;
		while (bytes_read < sz && start < end) {
			bytes = nss_stats_gre_redir(line, sizeof(line), start, &stats);

			if ((bytes_read + bytes) > sz)
				break;

			if (copy_to_user(ubuf + bytes_read, line, bytes) != 0) {
				bytes_read = -EFAULT;
				goto fail;
			}

			bytes_read += bytes;
			start++;
		}
	}

	if (bytes_read > 0) {
		*ppos = bytes_read;
	}

	if (data) {
		data->index = index;
	}

fail:
	return bytes_read;
}

/*
 * nss_stats_open()
 */
static int nss_stats_open(struct inode *inode, struct file *filp)
{
	struct nss_stats_data *data = NULL;

	data = kzalloc(sizeof(struct nss_stats_data), GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
	}
	memset(data, 0, sizeof (struct nss_stats_data));
	data->if_num = NSS_DYNAMIC_IF_START;
	data->index = 0;
	filp->private_data = data;

	return 0;
}

/*
 * nss_stats_release()
 */
static int nss_stats_release(struct inode *inode, struct file *filp)
{
	struct nss_stats_data *data = filp->private_data;

	if (data) {
		kfree(data);
	}

	return 0;
}

#define NSS_STATS_DECLARE_FILE_OPERATIONS(name) \
static const struct file_operations nss_stats_##name##_ops = { \
	.open = nss_stats_open, \
	.read = nss_stats_##name##_read, \
	.llseek = generic_file_llseek, \
	.release = nss_stats_release, \
};

/*
 * nss_ipv4_stats_ops
 */
NSS_STATS_DECLARE_FILE_OPERATIONS(ipv4)

/*
 * ipv4_reasm_stats_ops
 */
NSS_STATS_DECLARE_FILE_OPERATIONS(ipv4_reasm)

/*
 * ipv6_stats_ops
 */
NSS_STATS_DECLARE_FILE_OPERATIONS(ipv6)

/*
 * n2h_stats_ops
 */
NSS_STATS_DECLARE_FILE_OPERATIONS(n2h)

/*
 * lso_rx_stats_ops
 */
NSS_STATS_DECLARE_FILE_OPERATIONS(lso_rx)

/*
 * drv_stats_ops
 */
NSS_STATS_DECLARE_FILE_OPERATIONS(drv)

/*
 * pppoe_stats_ops
 */
NSS_STATS_DECLARE_FILE_OPERATIONS(pppoe)

/*
 * gmac_stats_ops
 */
NSS_STATS_DECLARE_FILE_OPERATIONS(gmac)

/*
 * capwap_stats_ops
 */
NSS_STATS_DECLARE_FILE_OPERATIONS(capwap_encap)
NSS_STATS_DECLARE_FILE_OPERATIONS(capwap_decap)

/*
 * eth_rx_stats_ops
 */
NSS_STATS_DECLARE_FILE_OPERATIONS(eth_rx)

/*
 * gre_redir_ops
 */
NSS_STATS_DECLARE_FILE_OPERATIONS(gre_redir)

/*
 * sjack_stats_ops
 */
NSS_STATS_DECLARE_FILE_OPERATIONS(sjack)

/*
 * nss_stats_init()
 * 	Enable NSS statistics
 */
void nss_stats_init(void)
{
	/*
	 * NSS driver entry
	 */
	nss_top_main.top_dentry = debugfs_create_dir("qca-nss-drv", NULL);
	if (unlikely(nss_top_main.top_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv directory in debugfs");

		/*
		 * Non availability of debugfs directory is not a catastrophy
		 * We can still go ahead with other initialization
		 */
		return;
	}

	nss_top_main.stats_dentry = debugfs_create_dir("stats", nss_top_main.top_dentry);
	if (unlikely(nss_top_main.stats_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv directory in debugfs");

		/*
		 * Non availability of debugfs directory is not a catastrophy
		 * We can still go ahead with rest of initialization
		 */
		return;
	}

	/*
	 * Create files to obtain statistics
	 */

	/*
	 * ipv4_stats
	 */
	nss_top_main.ipv4_dentry = debugfs_create_file("ipv4", 0400,
						nss_top_main.stats_dentry, &nss_top_main, &nss_stats_ipv4_ops);
	if (unlikely(nss_top_main.ipv4_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv/stats/ipv4 file in debugfs");
		return;
	}

	/*
	 * ipv4_reasm_stats
	 */
	nss_top_main.ipv4_reasm_dentry = debugfs_create_file("ipv4_reasm", 0400,
						nss_top_main.stats_dentry, &nss_top_main, &nss_stats_ipv4_reasm_ops);
	if (unlikely(nss_top_main.ipv4_reasm_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv/stats/ipv4_reasm file in debugfs");
		return;
	}

	/*
	 * ipv6_stats
	 */
	nss_top_main.ipv6_dentry = debugfs_create_file("ipv6", 0400,
						nss_top_main.stats_dentry, &nss_top_main, &nss_stats_ipv6_ops);
	if (unlikely(nss_top_main.ipv6_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv/stats/ipv6 file in debugfs");
		return;
	}

	/*
	 * ipv6_stats
	 */
	nss_top_main.eth_rx_dentry = debugfs_create_file("eth_rx", 0400,
						nss_top_main.stats_dentry, &nss_top_main, &nss_stats_eth_rx_ops);
	if (unlikely(nss_top_main.eth_rx_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv/stats/eth_rx file in debugfs");
		return;
	}

	/*
	 * n2h_stats
	 */
	nss_top_main.n2h_dentry = debugfs_create_file("n2h", 0400,
						nss_top_main.stats_dentry, &nss_top_main, &nss_stats_n2h_ops);
	if (unlikely(nss_top_main.n2h_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv/stats/n2h directory in debugfs");
		return;
	}

	/*
	 * lso_rx_stats
	 */
	nss_top_main.lso_rx_dentry = debugfs_create_file("lso_rx", 0400,
						nss_top_main.stats_dentry, &nss_top_main, &nss_stats_lso_rx_ops);
	if (unlikely(nss_top_main.lso_rx_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv/stats/lso_rx file in debugfs");
		return;
	}

	/*
	 * drv_stats
	 */
	nss_top_main.drv_dentry = debugfs_create_file("drv", 0400,
						nss_top_main.stats_dentry, &nss_top_main, &nss_stats_drv_ops);
	if (unlikely(nss_top_main.drv_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv/stats/drv directory in debugfs");
		return;
	}

	/*
	 * pppoe_stats
	 */
	nss_top_main.pppoe_dentry = debugfs_create_file("pppoe", 0400,
						nss_top_main.stats_dentry, &nss_top_main, &nss_stats_pppoe_ops);
	if (unlikely(nss_top_main.pppoe_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv/stats/pppoe file in debugfs");
		return;
	}

	/*
	 * gmac_stats
	 */
	nss_top_main.gmac_dentry = debugfs_create_file("gmac", 0400,
						nss_top_main.stats_dentry, &nss_top_main, &nss_stats_gmac_ops);
	if (unlikely(nss_top_main.gmac_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv/stats/gmac file in debugfs");
		return;
	}

	/*
	 * CAPWAP stats.
	 */
	nss_top_main.capwap_encap_dentry = debugfs_create_file("capwap_encap", 0400,
	nss_top_main.stats_dentry, &nss_top_main, &nss_stats_capwap_encap_ops);
	if (unlikely(nss_top_main.capwap_encap_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv/stats/capwap_encap file in debugfs");
		return;
	}

	nss_top_main.capwap_decap_dentry = debugfs_create_file("capwap_decap", 0400,
	nss_top_main.stats_dentry, &nss_top_main, &nss_stats_capwap_decap_ops);
	if (unlikely(nss_top_main.capwap_decap_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv/stats/capwap_decap file in debugfs");
		return;
	}

	/*
	 * GRE_REDIR stats
	 */
	nss_top_main.gre_redir_dentry = debugfs_create_file("gre_redir", 0400,
						nss_top_main.stats_dentry, &nss_top_main, &nss_stats_gre_redir_ops);
	if (unlikely(nss_top_main.gre_redir_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv/stats/gre_redir file in debugfs");
		return;
	}

	/*
	 * SJACK stats
	 */
	nss_top_main.sjack_dentry = debugfs_create_file("sjack", 0400,
						nss_top_main.stats_dentry, &nss_top_main, &nss_stats_sjack_ops);
	if (unlikely(nss_top_main.sjack_dentry == NULL)) {
		nss_warning("Failed to create qca-nss-drv/stats/sjack file in debugfs");
		return;
	}

	nss_log_init();
}


/*
 * nss_stats_clean()
 * 	Cleanup NSS statistics files
 */
void nss_stats_clean(void)
{
	/*
	 * Remove debugfs tree
	 */
	if (likely(nss_top_main.top_dentry != NULL)) {
		debugfs_remove_recursive(nss_top_main.top_dentry);
	}
}
