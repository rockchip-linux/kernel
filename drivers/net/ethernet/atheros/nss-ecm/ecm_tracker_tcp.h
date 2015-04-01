/*
 **************************************************************************
 * Copyright (c) 2014, 2015, The Linux Foundation.  All rights reserved.
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

struct ecm_tracker_tcp_instance;


struct ecm_tracker_tcp_instance {
	struct ecm_tracker_instance base;				/* MUST BE FIRST FIELD */

};

/*
 * TCP tracker
 *	Records the two streams of bytes sent in either direction over a TCP connection
 */
struct tcphdr *ecm_tracker_tcp_check_header_and_read(struct sk_buff *skb, struct ecm_tracker_ip_header *ip_hdr, struct tcphdr *port_buffer);
void ecm_tracker_tcp_init(struct ecm_tracker_tcp_instance *tti, int32_t data_limit, uint16_t src_mss_default, uint16_t dest_mss_default);
struct ecm_tracker_tcp_instance *ecm_tracker_tcp_alloc(void);


