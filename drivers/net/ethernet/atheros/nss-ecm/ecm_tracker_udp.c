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

#include <linux/version.h>
#include <linux/types.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/icmp.h>
#include <linux/sysctl.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/pkt_sched.h>
#include <linux/string.h>
#include <net/route.h>
#include <net/ip.h>
#include <net/tcp.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>	/* for put_user */
#include <net/ipv6.h>
#include <linux/inet.h>
#include <linux/in.h>
#include <linux/udp.h>
#include <linux/tcp.h>

#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_bridge.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_helper.h>
#include <net/netfilter/nf_conntrack_l4proto.h>
#include <net/netfilter/nf_conntrack_l3proto.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/ipv4/nf_conntrack_ipv4.h>
#include <net/netfilter/ipv4/nf_defrag_ipv4.h>

/*
 * Debug output levels
 * 0 = OFF
 * 1 = ASSERTS / ERRORS
 * 2 = 1 + WARN
 * 3 = 2 + INFO
 * 4 = 3 + TRACE
 */
#define DEBUG_LEVEL ECM_TRACKER_UDP_DEBUG_LEVEL

#include "ecm_types.h"
#include "ecm_db_types.h"
#include "ecm_tracker.h"
#include "ecm_tracker_udp.h"

/*
 * Magic numbers
 */
#define ECM_TRACKER_UDP_INSTANCE_MAGIC 0x7765
#define ECM_TRACKER_UDP_SKB_CB_MAGIC 0xAAAB

/*
 * Useful constants
 */
#define ECM_TRACKER_UDP_HEADER_SIZE 8		/* UDP header is always 8 bytes RFC 768 Page 1 */


/*
 * struct ecm_tracker_udp_internal_instance
 */
struct ecm_tracker_udp_internal_instance {
	struct ecm_tracker_udp_instance udp_base;			/* MUST BE FIRST FIELD */


	ecm_tracker_sender_state_t sender_state[ECM_TRACKER_SENDER_MAX];
						/* State of each sender */
	ecm_db_timer_group_t timer_group;	/* Recommended timer group for connection that is using this tracker */

	spinlock_t lock;			/* lock */

	int refs;				/* Integer to trap we never go negative */
#if (DEBUG_LEVEL > 0)
	uint16_t magic;
#endif
};

int ecm_tracker_udp_count = 0;		/* Counts the number of UDP data trackers right now */
spinlock_t ecm_tracker_udp_lock;		/* Global lock for the tracker globals */

/*
 * ecm_trracker_udp_connection_state_matrix[][]
 *	Matrix to convert from/to states to connection state
 */
static ecm_tracker_connection_state_t ecm_tracker_udp_connection_state_matrix[ECM_TRACKER_SENDER_STATE_MAX][ECM_TRACKER_SENDER_STATE_MAX] =
{	/* 			Unknown						Establishing					Established					Closing					Closed					Fault */
	/* Unknown */		{ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHED,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Establishing */	{ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHED,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Established */	{ECM_TRACKER_CONNECTION_STATE_ESTABLISHED,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHED,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHED,	ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Closing */		{ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_CLOSING,		ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Closed */		{ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_CLOSING,		ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_CLOSED, 	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Fault */		{ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT},
};

/*
 * ecm_tracker_udp_check_header_and_read()
 *	Check for validly sized header and read the udp protocol header
 */
struct udphdr *ecm_tracker_udp_check_header_and_read(struct sk_buff *skb, struct ecm_tracker_ip_header *ip_hdr, struct udphdr *port_buffer)
{
	struct ecm_tracker_ip_protocol_header *header;

	/*
	 * Is there a UDP header?
	 */
	header = &ip_hdr->headers[ECM_TRACKER_IP_PROTOCOL_TYPE_UDP];
	if (header->header_size != ECM_TRACKER_UDP_HEADER_SIZE) {
		DEBUG_WARN("Skb: %p, UDP header size bad %u\n", skb, header->header_size);
		return NULL;
	}

	return skb_header_pointer(skb, header->offset, sizeof(*port_buffer), port_buffer);
}
EXPORT_SYMBOL(ecm_tracker_udp_check_header_and_read);


/*
 * ecm_tracker_udp_ref()
 */
static void ecm_tracker_udp_ref(struct ecm_tracker_udp_internal_instance *utii)
{
	DEBUG_CHECK_MAGIC(utii, ECM_TRACKER_UDP_INSTANCE_MAGIC, "%p: magic failed", utii);

	spin_lock_bh(&utii->lock);

	utii->refs++;
	DEBUG_ASSERT(utii->refs > 0, "%p: ref wrap", utii);
	DEBUG_TRACE("%p: ref %d\n", utii, utii->refs);

	spin_unlock_bh(&utii->lock);
}

/*
 * ecm_tracker_udp_ref_callback()
 */
void ecm_tracker_udp_ref_callback(struct ecm_tracker_instance *ti)
{
	struct ecm_tracker_udp_internal_instance *utii = (struct ecm_tracker_udp_internal_instance *)ti;
	ecm_tracker_udp_ref(utii);
}

/*
 * ecm_tracker_udp_deref()
 */
static int ecm_tracker_udp_deref(struct ecm_tracker_udp_internal_instance *utii)
{
	int refs;
	DEBUG_CHECK_MAGIC(utii, ECM_TRACKER_UDP_INSTANCE_MAGIC, "%p: magic failed", utii);

	spin_lock_bh(&utii->lock);
	utii->refs--;
	refs = utii->refs;
	DEBUG_ASSERT(utii->refs >= 0, "%p: ref wrap", utii);
	DEBUG_TRACE("%p: deref %d\n", utii, utii->refs);

	if (utii->refs > 0) {
		spin_unlock_bh(&utii->lock);
		return refs;
	}
	spin_unlock_bh(&utii->lock);

	DEBUG_TRACE("%p: final\n", utii);


	spin_lock_bh(&ecm_tracker_udp_lock);
	ecm_tracker_udp_count--;
	DEBUG_ASSERT(ecm_tracker_udp_count >= 0, "%p: tracker count wrap", utii);
	spin_unlock_bh(&ecm_tracker_udp_lock);

	DEBUG_INFO("%p: Udp tracker final\n", utii);
	DEBUG_CLEAR_MAGIC(utii);
	kfree(utii);

	return 0;
}

/*
 * _ecm_tracker_udp_deref_callback()
 */
int ecm_tracker_udp_deref_callback(struct ecm_tracker_instance *ti)
{
	struct ecm_tracker_udp_internal_instance *utii = (struct ecm_tracker_udp_internal_instance *)ti;
	return ecm_tracker_udp_deref(utii);
}


/*
 * ecm_tracker_udp_state_update_callback()
 * 	Update connection state based on the knowledge we have and the skb given
 */
static void ecm_tracker_udp_state_update_callback(struct ecm_tracker_instance *ti, ecm_tracker_sender_type_t sender, struct ecm_tracker_ip_header *ip_hdr, struct sk_buff *skb)
{
	struct ecm_tracker_udp_internal_instance *utii = (struct ecm_tracker_udp_internal_instance *)ti;
	DEBUG_CHECK_MAGIC(utii, ECM_TRACKER_UDP_INSTANCE_MAGIC, "%p: magic failed", utii);

	/*
	 * As long as a sender has seen data then we consider the sender established
	 */
	spin_lock_bh(&utii->lock);
	utii->sender_state[sender] = ECM_TRACKER_SENDER_STATE_ESTABLISHED;
	spin_unlock_bh(&utii->lock);
}

/*
 * ecm_tracker_udp_state_get_callback()
 * 	Get state
 */
static void ecm_tracker_udp_state_get_callback(struct ecm_tracker_instance *ti, ecm_tracker_sender_state_t *src_state,
					ecm_tracker_sender_state_t *dest_state, ecm_tracker_connection_state_t *state, ecm_db_timer_group_t *tg)
{
	struct ecm_tracker_udp_internal_instance *utii = (struct ecm_tracker_udp_internal_instance *)ti;
	DEBUG_CHECK_MAGIC(utii, ECM_TRACKER_UDP_INSTANCE_MAGIC, "%p: magic failed", utii);
	spin_lock_bh(&utii->lock);
	*src_state = utii->sender_state[ECM_TRACKER_SENDER_TYPE_SRC];
	*dest_state = utii->sender_state[ECM_TRACKER_SENDER_TYPE_DEST];
	*tg = utii->timer_group;
	spin_unlock_bh(&utii->lock);
	*state = ecm_tracker_udp_connection_state_matrix[*src_state][*dest_state];
}


/*
 * ecm_tracker_udp_init()
 *	Initialise the two host addresses that define the two directions we track data for
 */
void ecm_tracker_udp_init(struct ecm_tracker_udp_instance *uti, int32_t data_limit, int src_port, int dest_port)
{
	struct ecm_tracker_udp_internal_instance *utii = (struct ecm_tracker_udp_internal_instance *)uti;
	DEBUG_CHECK_MAGIC(utii, ECM_TRACKER_UDP_INSTANCE_MAGIC, "%p: magic failed", utii);
	DEBUG_TRACE("%p: init udp tracker\n", utii);

	spin_lock_bh(&utii->lock);
	if ((src_port < 1024) || (dest_port < 1024)) {
		/*
		 * Because UDP connections can be reaped we assign well known ports to the WKP timer group.
		 * The WKP group is not reaped thus preserving connections involving known services.
		 * NOTE: Classifiers are still free to change the group as they see fit.
		 */
		utii->timer_group = ECM_DB_TIMER_GROUPS_CONNECTION_UDP_WKP_TIMEOUT;
		spin_unlock_bh(&utii->lock);
		return;
	}
	utii->timer_group = ECM_DB_TIMER_GROUPS_CONNECTION_UDP_GENERIC_TIMEOUT;
	spin_unlock_bh(&utii->lock);
}
EXPORT_SYMBOL(ecm_tracker_udp_init);

/*
 * ecm_tracker_udp_alloc()
 */
struct ecm_tracker_udp_instance *ecm_tracker_udp_alloc(void)
{
	struct ecm_tracker_udp_internal_instance *utii;

	utii = (struct ecm_tracker_udp_internal_instance *)kzalloc(sizeof(struct ecm_tracker_udp_internal_instance), GFP_ATOMIC | __GFP_NOWARN);
	if (!utii) {
		DEBUG_WARN("Failed to allocate udp tracker instance\n");
		return NULL;
	}

	utii->udp_base.base.ref = ecm_tracker_udp_ref_callback;
	utii->udp_base.base.deref = ecm_tracker_udp_deref_callback;
	utii->udp_base.base.state_update = ecm_tracker_udp_state_update_callback;
	utii->udp_base.base.state_get = ecm_tracker_udp_state_get_callback;

	spin_lock_init(&utii->lock);

	utii->refs = 1;
	DEBUG_SET_MAGIC(utii, ECM_TRACKER_UDP_INSTANCE_MAGIC);

	spin_lock_bh(&ecm_tracker_udp_lock);
	ecm_tracker_udp_count++;
	DEBUG_ASSERT(ecm_tracker_udp_count > 0, "%p: udp tracker count wrap\n", utii);
	spin_unlock_bh(&ecm_tracker_udp_lock);

	DEBUG_TRACE("UDP tracker created %p\n", utii);
	return (struct ecm_tracker_udp_instance *)utii;
}
EXPORT_SYMBOL(ecm_tracker_udp_alloc);

/*
 * ecm_tracker_udp_module_init()
 */
int ecm_tracker_udp_module_init(void)
{
	DEBUG_INFO("UDP Tracker Module init\n");
	spin_lock_init(&ecm_tracker_udp_lock);
	return 0;
}
EXPORT_SYMBOL(ecm_tracker_udp_module_init);

/*
 * ecm_tracker_udp_module_exit()
 */
void ecm_tracker_udp_module_exit(void)
{
	DEBUG_INFO("UDP Tracker Module exit\n");
}
EXPORT_SYMBOL(ecm_tracker_udp_module_exit);
