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
#define DEBUG_LEVEL ECM_TRACKER_DATAGRAM_DEBUG_LEVEL

#include "ecm_types.h"
#include "ecm_db_types.h"
#include "ecm_tracker.h"
#include "ecm_tracker_datagram.h"

/*
 * Magic numbers
 */
#define ECM_TRACKER_DATAGRAM_INSTANCE_MAGIC 0x3AbC

/*
 * struct ecm_tracker_datagram_internal_instance
 */
struct ecm_tracker_datagram_internal_instance {
	struct ecm_tracker_datagram_instance datagram_base;	/* MUST BE FIRST FIELD */


	ecm_tracker_sender_state_t sender_state[ECM_TRACKER_SENDER_MAX];
						/* State of each sender */

	spinlock_t lock;			/* lock */

	int refs;				/* Integer to trap we never go negative */
#if (DEBUG_LEVEL > 0)
	uint16_t magic;
#endif
};

int ecm_tracker_datagram_count = 0;		/* Counts the number of DATAGRAM data trackers right now */
spinlock_t ecm_tracker_datagram_lock;		/* Global lock for the tracker globals */

/*
 * ecm_trracker_datagram_connection_state_matrix[][]
 *	Matrix to convert from/to states to connection state
 */
static ecm_tracker_connection_state_t ecm_tracker_datagram_connection_state_matrix[ECM_TRACKER_SENDER_STATE_MAX][ECM_TRACKER_SENDER_STATE_MAX] =
{	/* 			Unknown						Establishing					Established					Closing					Closed					Fault */
	/* Unknown */		{ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHED,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Establishing */	{ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHED,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Established */	{ECM_TRACKER_CONNECTION_STATE_ESTABLISHED,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHED,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHED,	ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Closing */		{ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_CLOSING,		ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Closed */		{ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_CLOSING,		ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_CLOSED, 	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Fault */		{ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT},
};


/*
 * ecm_tracker_datagram_ref()
 */
static void ecm_tracker_datagram_ref(struct ecm_tracker_datagram_internal_instance *dtii)
{

	DEBUG_CHECK_MAGIC(dtii, ECM_TRACKER_DATAGRAM_INSTANCE_MAGIC, "%p: magic failed", dtii);

	spin_lock_bh(&dtii->lock);

	dtii->refs++;
	DEBUG_ASSERT(dtii->refs > 0, "%p: ref wrap", dtii);
	DEBUG_TRACE("%p: ref %d\n", dtii, dtii->refs);

	spin_unlock_bh(&dtii->lock);
}

/*
 * ecm_tracker_datagram_ref_callback()
 */
static void ecm_tracker_datagram_ref_callback(struct ecm_tracker_instance *ti)
{
	struct ecm_tracker_datagram_internal_instance *dtii = (struct ecm_tracker_datagram_internal_instance *)ti;
	ecm_tracker_datagram_ref(dtii);
}

/*
 * ecm_tracker_datagram_deref()
 */
static int ecm_tracker_datagram_deref(struct ecm_tracker_datagram_internal_instance *dtii)
{

	int refs;
	DEBUG_CHECK_MAGIC(dtii, ECM_TRACKER_DATAGRAM_INSTANCE_MAGIC, "%p: magic failed", dtii);

	spin_lock_bh(&dtii->lock);
	dtii->refs--;
	refs = dtii->refs;
	DEBUG_ASSERT(dtii->refs >= 0, "%p: ref wrap", dtii);
	DEBUG_TRACE("%p: deref %d\n", dtii, dtii->refs);

	if (dtii->refs > 0) {
		spin_unlock_bh(&dtii->lock);
		return refs;
	}
	spin_unlock_bh(&dtii->lock);

	DEBUG_TRACE("%p: final\n", dtii);


	spin_lock_bh(&ecm_tracker_datagram_lock);
	ecm_tracker_datagram_count--;
	DEBUG_ASSERT(ecm_tracker_datagram_count >= 0, "%p: tracker count wrap", dtii);
	spin_unlock_bh(&ecm_tracker_datagram_lock);

	DEBUG_INFO("%p: Udp tracker final\n", dtii);
	DEBUG_CLEAR_MAGIC(dtii);
	kfree(dtii);

	return 0;
}

/*
 * _ecm_tracker_datagram_deref_callback()
 */
static int ecm_tracker_datagram_deref_callback(struct ecm_tracker_instance *ti)
{
	struct ecm_tracker_datagram_internal_instance *dtii = (struct ecm_tracker_datagram_internal_instance *)ti;
	return ecm_tracker_datagram_deref(dtii);
}


/*
 * ecm_tracker_datagram_state_update_callback()
 * 	Update connection state based on the knowledge we have and the skb given
 */
static void ecm_tracker_datagram_state_update_callback(struct ecm_tracker_instance *ti, ecm_tracker_sender_type_t sender, struct ecm_tracker_ip_header *ip_hdr, struct sk_buff *skb)
{
	struct ecm_tracker_datagram_internal_instance *dtii = (struct ecm_tracker_datagram_internal_instance *)ti;
	DEBUG_CHECK_MAGIC(dtii, ECM_TRACKER_DATAGRAM_INSTANCE_MAGIC, "%p: magic failed", dtii);

	/*
	 * As long as a sender has seen data then we consider the sender established
	 */
	spin_lock_bh(&dtii->lock);
	dtii->sender_state[sender] = ECM_TRACKER_SENDER_STATE_ESTABLISHED;
	spin_unlock_bh(&dtii->lock);
}

/*
 * ecm_tracker_datagram_state_get_callback()
 * 	Get state
 */
static void ecm_tracker_datagram_state_get_callback(struct ecm_tracker_instance *ti, ecm_tracker_sender_state_t *src_state,
					ecm_tracker_sender_state_t *dest_state, ecm_tracker_connection_state_t *state, ecm_db_timer_group_t *tg)
{
	struct ecm_tracker_datagram_internal_instance *dtii = (struct ecm_tracker_datagram_internal_instance *)ti;
	DEBUG_CHECK_MAGIC(dtii, ECM_TRACKER_DATAGRAM_INSTANCE_MAGIC, "%p: magic failed", dtii);
	spin_lock_bh(&dtii->lock);
	*src_state = dtii->sender_state[ECM_TRACKER_SENDER_TYPE_SRC];
	*dest_state = dtii->sender_state[ECM_TRACKER_SENDER_TYPE_DEST];
	spin_unlock_bh(&dtii->lock);
	*state = ecm_tracker_datagram_connection_state_matrix[*src_state][*dest_state];
	*tg = ECM_DB_TIMER_GROUPS_CONNECTION_GENERIC_TIMEOUT;
}


/*
 * ecm_tracker_datagram_init()
 *	Initialise the two host addresses that define the two directions we track data for
 */
void ecm_tracker_datagram_init(struct ecm_tracker_datagram_instance *uti, int32_t data_limit)
{
	struct ecm_tracker_datagram_internal_instance *dtii = (struct ecm_tracker_datagram_internal_instance *)uti;
	DEBUG_CHECK_MAGIC(dtii, ECM_TRACKER_DATAGRAM_INSTANCE_MAGIC, "%p: magic failed", dtii);
	DEBUG_TRACE("%p: init tracker\n", uti);

}
EXPORT_SYMBOL(ecm_tracker_datagram_init);

/*
 * ecm_tracker_datagram_alloc()
 */
struct ecm_tracker_datagram_instance *ecm_tracker_datagram_alloc(void)
{
	struct ecm_tracker_datagram_internal_instance *dtii;

	dtii = (struct ecm_tracker_datagram_internal_instance *)kzalloc(sizeof(struct ecm_tracker_datagram_internal_instance), GFP_ATOMIC | __GFP_NOWARN);
	if (!dtii) {
		DEBUG_WARN("Failed to allocate datagram tracker instance\n");
		return NULL;
	}

	dtii->datagram_base.base.ref = ecm_tracker_datagram_ref_callback;
	dtii->datagram_base.base.deref = ecm_tracker_datagram_deref_callback;
	dtii->datagram_base.base.state_update = ecm_tracker_datagram_state_update_callback;
	dtii->datagram_base.base.state_get = ecm_tracker_datagram_state_get_callback;

	// GGG TODO IMPLEMENT METHODS SPECIFIC TO WORKING WITH datagram e.g. reading without worrying about the datagram header content

	spin_lock_init(&dtii->lock);

	dtii->refs = 1;
	DEBUG_SET_MAGIC(dtii, ECM_TRACKER_DATAGRAM_INSTANCE_MAGIC);

	spin_lock_bh(&ecm_tracker_datagram_lock);
	ecm_tracker_datagram_count++;
	DEBUG_ASSERT(ecm_tracker_datagram_count > 0, "%p: datagram tracker count wrap\n", dtii);
	spin_unlock_bh(&ecm_tracker_datagram_lock);

	DEBUG_TRACE("DATAGRAM tracker created %p\n", dtii);
	return (struct ecm_tracker_datagram_instance *)dtii;
}
EXPORT_SYMBOL(ecm_tracker_datagram_alloc);

/*
 * ecm_tracker_datagram_module_init()
 */
int ecm_tracker_datagram_module_init(void)
{
	DEBUG_INFO("Datagram Tracker Module init\n");
	spin_lock_init(&ecm_tracker_datagram_lock);
	return 0;
}
EXPORT_SYMBOL(ecm_tracker_datagram_module_init);

/*
 * ecm_tracker_datagram_module_exit()
 */
void ecm_tracker_datagram_module_exit(void)
{
	DEBUG_INFO("Datagram Tracker Module exit\n");
}
EXPORT_SYMBOL(ecm_tracker_datagram_module_exit);
