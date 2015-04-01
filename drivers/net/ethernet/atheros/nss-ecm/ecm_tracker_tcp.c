/*
 **************************************************************************
 * Copyright (c) 2014,2015 The Linux Foundation.  All rights reserved.
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
#define DEBUG_LEVEL ECM_TRACKER_TCP_DEBUG_LEVEL

#include "ecm_types.h"
#include "ecm_db_types.h"
#include "ecm_tracker.h"
#include "ecm_tracker_tcp.h"

/*
 * TCP control flags.
 */
#define TCF_FIN 0x01
#define TCF_SYN 0x02
#define TCF_RST 0x04
#define TCF_PSH 0x08
#define TCF_ACK 0x10
#define TCF_URG 0x20

/*
 * Extract a byte tcp flags field from a union tcp_word_hdr.
 */
#define ECM_TRACKER_TCP_WORD_HDR_TO_FLAGS(word_hdr) ((ntohl(word_hdr->words[3]) >> 16) & 0xff)

/*
 * Valid TCP flag combinations.
 */
#define ECM_TRACKER_TCP_VALID_FLAGS_MAX ((TCF_URG | TCF_ACK | TCF_PSH | TCF_RST | TCF_SYN | TCF_FIN) + 1)

static uint32_t ecm_tracker_tcp_valid_flags[ECM_TRACKER_TCP_VALID_FLAGS_MAX] = {
	[TCF_FIN | TCF_RST | TCF_ACK] = 1,
	[TCF_FIN | TCF_PSH | TCF_ACK] = 1,
	[TCF_FIN | TCF_PSH | TCF_ACK | TCF_URG] = 1,
	[TCF_FIN | TCF_ACK] = 1,
	[TCF_FIN | TCF_ACK | TCF_URG] = 1,
	[TCF_SYN] = 1,
	[TCF_SYN | TCF_ACK] = 1,
	[TCF_RST] = 1,
	[TCF_RST | TCF_PSH | TCF_ACK] = 1,
	[TCF_RST | TCF_ACK] = 1,
	[TCF_PSH | TCF_ACK] = 1,
	[TCF_PSH | TCF_ACK | TCF_URG] = 1,
	[TCF_ACK] = 1,
	[TCF_ACK | TCF_URG] = 1,
};

/*
 * Magic numbers
 */
#define ECM_TRACKER_TCP_INSTANCE_MAGIC 0x5129
#define ECM_TRACKER_TCP_SKB_CB_MAGIC 0x4398
#define ECM_TRACKER_TCP_READER_INSTANCE_MAGIC 0x1123


/*
 * struct ecm_tracker_tcp_sender_state
 *	Connection state of each sender
 */
struct ecm_tracker_tcp_sender_state {
	ecm_tracker_sender_state_t state;		/* State of the sender */
	uint32_t syn_seq;				/* Sequence number of the syn (ISN): used to detect when peer has ack'd the syn, transitioned this sender to established. */
	uint32_t fin_seq;				/* Sequence number of the fin: used to detect when peer has ack'd the fin, transitioned this sender to closed. */
};

/*
 * struct ecm_tracker_tcp_internal_instance
 */
struct ecm_tracker_tcp_internal_instance  {
	struct ecm_tracker_tcp_instance tcp_base;					/* MUST BE FIRST FIELD */
	struct ecm_tracker_tcp_sender_state sender_states[ECM_TRACKER_SENDER_MAX];	/* Sender states */
	int refs;									/* Integer to trap we never go negative */
	spinlock_t lock;
#if (DEBUG_LEVEL > 0)
	uint16_t magic;
#endif
};


/*
 * ecm_tracker_tcp_timer_group_from_state[]
 *	Given a prevailing connection state return the appropriate timer group.
 *
 * Different states have different timeouts, for example, established state is the longest timeout.
 * Using these timeouts ensures efficient resource uses and avoids connections hanging around when it is unnecessary.
 */
static ecm_db_timer_group_t ecm_tracker_tcp_timer_group_from_state[] = {
							ECM_DB_TIMER_GROUPS_CONNECTION_TCP_SHORT_TIMEOUT,	/* ECM_TRACKER_CONNECTION_STATE_ESTABLISHING */
							ECM_DB_TIMER_GROUPS_CONNECTION_TCP_LONG_TIMEOUT,	/* ECM_TRACKER_CONNECTION_STATE_ESTABLISHED */
							ECM_DB_TIMER_GROUPS_CONNECTION_TCP_SHORT_TIMEOUT,	/* ECM_TRACKER_CONNECTION_STATE_CLOSING */
							ECM_DB_TIMER_GROUPS_CONNECTION_TCP_SHORT_TIMEOUT,	/* ECM_TRACKER_CONNECTION_STATE_CLOSED */
							ECM_DB_TIMER_GROUPS_CONNECTION_TCP_RESET_TIMEOUT,	/* ECM_TRACKER_CONNECTION_STATE_FAULT */
							};
int ecm_tracker_tcp_count = 0;		/* Counts the number of TCP data trackers right now */
spinlock_t ecm_tracker_tcp_lock;	/* Global lock for the tracker globals */

/*
 * ecm_tracker_tcp_connection_state_matrix[][]
 *	Matrix to convert from/to states to connection state
 */
static ecm_tracker_connection_state_t ecm_tracker_tcp_connection_state_matrix[ECM_TRACKER_SENDER_STATE_MAX][ECM_TRACKER_SENDER_STATE_MAX] =
{	/* 			Unknown						Establishing					Established					Closing					Closed					Fault */
	/* Unknown */		{ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Establishing */	{ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Established */	{ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHING,	ECM_TRACKER_CONNECTION_STATE_ESTABLISHED,	ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Closing */		{ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_CLOSING,		ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Closed */		{ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_CLOSING,		ECM_TRACKER_CONNECTION_STATE_CLOSING,	ECM_TRACKER_CONNECTION_STATE_CLOSED, 	ECM_TRACKER_CONNECTION_STATE_FAULT},
	/* Fault */		{ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,		ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT,	ECM_TRACKER_CONNECTION_STATE_FAULT},
};

/*
 * ecm_tracker_tcp_check_header_and_read()
 *	Check that we have a complete transport-level header, check it and return it if it's okay.
 */
struct tcphdr *ecm_tracker_tcp_check_header_and_read(struct sk_buff *skb, struct ecm_tracker_ip_header *ip_hdr, struct tcphdr *port_buffer)
{
	struct tcphdr *hdr;
	union tcp_word_hdr *word_hdr;
	uint16_t tcp_hdr_len;
	uint32_t tcp_flags;
	struct ecm_tracker_ip_protocol_header *header;

	/*
	 * Is there a TCP header?
	 */
	header = &ip_hdr->headers[ECM_TRACKER_IP_PROTOCOL_TYPE_TCP];
	if (header->size == 0) {
		DEBUG_WARN("No TCP header %p\n", skb);
		return NULL;
	}
	hdr = (struct tcphdr *)skb_header_pointer(skb, header->offset, sizeof(*port_buffer), port_buffer);
	if (unlikely(!hdr)) {
		DEBUG_WARN("Cant read TCP header %p\n", skb);
		return NULL;
	}
	word_hdr = (union tcp_word_hdr *)hdr;

	/*
	 * Check that we have all of the TCP data we're supposed to have
	 */
	tcp_hdr_len = (uint16_t)hdr->doff;
	tcp_hdr_len <<= 2;
	if (unlikely(tcp_hdr_len < 20)) {
		DEBUG_WARN("Packet %p TCP header to short %u\n", skb, tcp_hdr_len);
		return NULL;
	}

	if (unlikely(ip_hdr->total_length < (header->offset + header->size))) {
		DEBUG_WARN("TCP packet %p too short (ip_total_len = %u, size needed=%u)\n", skb, ip_hdr->total_length, header->offset + header->size);
		return NULL;
	}

	/*
	 * Validate flags
	 */
	tcp_flags = ECM_TRACKER_TCP_WORD_HDR_TO_FLAGS(word_hdr);
	DEBUG_TRACE("%p: TCP flags = 0x%04x\n", skb, (unsigned)tcp_flags);
	if (unlikely(!ecm_tracker_tcp_valid_flags[tcp_flags & 0x3f])) {
		DEBUG_WARN("%p: Invalid flags 0x%x", skb, (unsigned)tcp_flags);
		return NULL;
	}

	return hdr;
}
EXPORT_SYMBOL(ecm_tracker_tcp_check_header_and_read);


/*
 * ecm_tracker_tcp_ref_callback()
 */
void ecm_tracker_tcp_ref_callback(struct ecm_tracker_instance *ti)
{
	struct ecm_tracker_tcp_internal_instance *ttii = (struct ecm_tracker_tcp_internal_instance *)ti;
	DEBUG_CHECK_MAGIC(ttii, ECM_TRACKER_TCP_INSTANCE_MAGIC, "%p: magic failed", ttii);

	spin_lock_bh(&ttii->lock);

	ttii->refs++;
	DEBUG_ASSERT(ttii->refs > 0, "%p: ref wrap", ttii);
	DEBUG_TRACE("%p: ref %d\n", ttii, ttii->refs);

	spin_unlock_bh(&ttii->lock);
}

/*
 * ecm_tracker_tcp_deref_callback()
 */
int ecm_tracker_tcp_deref_callback(struct ecm_tracker_instance *ti)
{
	struct ecm_tracker_tcp_internal_instance *ttii = (struct ecm_tracker_tcp_internal_instance *)ti;
	int refs;
	DEBUG_CHECK_MAGIC(ttii, ECM_TRACKER_TCP_INSTANCE_MAGIC, "%p: magic failed", ttii);

	spin_lock_bh(&ttii->lock);
	ttii->refs--;
	refs = ttii->refs;
	DEBUG_ASSERT(ttii->refs >= 0, "%p: ref wrap", ttii);
	DEBUG_TRACE("%p: deref %d\n", ttii, ttii->refs);

	if (ttii->refs > 0) {
		spin_unlock_bh(&ttii->lock);
		return refs;
	}

	DEBUG_TRACE("%p: final\n", ttii);
	spin_unlock_bh(&ttii->lock);

	spin_lock_bh(&ecm_tracker_tcp_lock);
	ecm_tracker_tcp_count--;
	DEBUG_ASSERT(ecm_tracker_tcp_count >= 0, "%p: tracker count wrap", ttii);
	spin_unlock_bh(&ecm_tracker_tcp_lock);

	DEBUG_INFO("%p: TCP tracker final\n", ttii);
	DEBUG_CLEAR_MAGIC(ttii);
	kfree(ttii);

	return 0;
}


/*
 * ecm_tracker_tcp_state_update_callback()
 * 	Update connection state based on the knowledge we have and the skb given
 */
static void ecm_tracker_tcp_state_update_callback(struct ecm_tracker_instance *ti, ecm_tracker_sender_type_t sender, struct ecm_tracker_ip_header *ip_hdr, struct sk_buff *skb)
{
	struct ecm_tracker_tcp_internal_instance *ttii = (struct ecm_tracker_tcp_internal_instance *)ti;
	struct tcphdr tcp_hdr_buff;
	struct tcphdr *tcp_hdr;
	struct ecm_tracker_tcp_sender_state *sender_state;
	struct ecm_tracker_tcp_sender_state *peer_state;

	DEBUG_CHECK_MAGIC(ttii, ECM_TRACKER_TCP_INSTANCE_MAGIC, "%p: magic failed", ttii);

	/*
	 * Get refereces to states
	 */
	DEBUG_ASSERT((sender >= 0) && (sender <= 1), "%p: invalid sender %d\n", ttii, sender);
	sender_state = &ttii->sender_states[sender];
	peer_state = &ttii->sender_states[!sender];

	/*
	 * Get tcp header
	 */
	tcp_hdr = ecm_tracker_tcp_check_header_and_read(skb, ip_hdr, &tcp_hdr_buff);
	if (unlikely(!tcp_hdr)) {
		DEBUG_WARN("%p: no tcp_hdr for %p\n", ttii, skb);
		spin_lock_bh(&ttii->lock);
		sender_state->state = ECM_TRACKER_SENDER_STATE_FAULT;
		peer_state->state = ECM_TRACKER_SENDER_STATE_FAULT;
		spin_unlock_bh(&ttii->lock);
		return;
	}

	/*
	 * If either side reports a reset this is catastrophic for the connection
	 */
	if (unlikely(tcp_hdr->rst)) {
		DEBUG_INFO("%p: RESET\n", ttii);
		spin_lock_bh(&ttii->lock);
		sender_state->state = ECM_TRACKER_SENDER_STATE_FAULT;
		peer_state->state = ECM_TRACKER_SENDER_STATE_FAULT;
		spin_unlock_bh(&ttii->lock);
		return;
	}

	/*
	 * Likely ack is set - this constitutes the mainstay of a TCP connection
	 * The sending of an ack may put the other side of the connection into a different state
	 */
	spin_lock_bh(&ttii->lock);
	if (likely(tcp_hdr->ack)) {
		ecm_tracker_sender_state_t peer_state_current = peer_state->state;
		uint32_t ack_seq = ntohl(tcp_hdr->ack_seq);

		switch (peer_state_current) {
		case ECM_TRACKER_SENDER_STATE_UNKNOWN: {
			/*
			 * Looks like we came into this connection mid-flow.
			 * Flag that the peer is established which is all we can infer right now and
			 * initialise the peers SYN sequence for further analysis of sequence space.
			 */
			peer_state->state = ECM_TRACKER_SENDER_STATE_ESTABLISHED;
			peer_state->syn_seq = (ack_seq - 1);		/* -1 is because the ACK has ack'd our ficticious SYN */
			DEBUG_INFO("%p: From unkown to established, ack_seq: %u, syn_seq: %u\n", ttii, ack_seq, peer_state->syn_seq);
			break;
		}
		case ECM_TRACKER_SENDER_STATE_ESTABLISHING: {
			int32_t ackd;
			uint32_t syn_seq;

			syn_seq = peer_state->syn_seq;
			ackd = (int32_t)(ack_seq - syn_seq);
			DEBUG_TRACE("%p: ack %u for syn_seq %u? ackd = %d\n", ttii, ack_seq, syn_seq, ackd);

			if (ackd <= 0) {
				DEBUG_TRACE("%p: No change\n", ttii);
			} else {
				DEBUG_INFO("%p: Established\n", ttii);
				peer_state->state = ECM_TRACKER_SENDER_STATE_ESTABLISHED;
			}
			break;
		}
		case ECM_TRACKER_SENDER_STATE_CLOSING: {
			int32_t ackd;
			uint32_t fin_seq;

			fin_seq = peer_state->fin_seq;
			ackd = (int32_t)(ack_seq - fin_seq);
			DEBUG_TRACE("%p: ack %u for fin_seq %u? ackd = %d\n", ttii, ack_seq, fin_seq, ackd);

			if (ackd <= 0) {
				DEBUG_TRACE("%p: No change\n", ttii);
			} else {
				DEBUG_TRACE("%p: Closed\n", ttii);
				peer_state->state = ECM_TRACKER_SENDER_STATE_CLOSED;
			}
			break;
		}
		case ECM_TRACKER_SENDER_STATE_ESTABLISHED:
		case ECM_TRACKER_SENDER_STATE_CLOSED:
		case ECM_TRACKER_SENDER_STATE_FAULT:
			/*
			 * No change
			 */
			break;
		default:
			DEBUG_ASSERT(false, "%p: unhandled state: %d\n", ttii, peer_state_current);
		}
	}

	/*
	 * Handle control flags sent by the sender (SYN & FIN)
	 * Handle SYN first because, in sequence space, SYN is first.
	 */
	if (tcp_hdr->syn) {
		ecm_tracker_sender_state_t sender_state_current = sender_state->state;
		uint32_t seq = ntohl(tcp_hdr->seq);

		switch (sender_state_current) {
		case ECM_TRACKER_SENDER_STATE_UNKNOWN:
			sender_state->state = ECM_TRACKER_SENDER_STATE_ESTABLISHING;
			sender_state->syn_seq = seq;		/* Seq is the sequence number of the SYN */
			DEBUG_INFO("%p: From unkown to establishing, syn_seq: %u\n", ttii, sender_state->syn_seq);
			break;
		case ECM_TRACKER_SENDER_STATE_CLOSING:
		case ECM_TRACKER_SENDER_STATE_CLOSED:
			/*
			 * SYN after seeing a FIN?  FAULT!
			 */
			sender_state->state = ECM_TRACKER_SENDER_STATE_FAULT;
			DEBUG_INFO("%p: SYN after FIN - fault\n", ttii);
			break;
		case ECM_TRACKER_SENDER_STATE_ESTABLISHED:
			/*
			 * SYN when established is just a duplicate down to syn/ack timing subtleties
			 */
		case ECM_TRACKER_SENDER_STATE_ESTABLISHING:
		case ECM_TRACKER_SENDER_STATE_FAULT:
			/*
			 * No change
			 */
			break;
		default:
			DEBUG_ASSERT(false, "%p: unhandled state: %d\n", ttii, sender_state_current);
		}
	}

	if (tcp_hdr->fin) {
		ecm_tracker_sender_state_t sender_state_current = sender_state->state;
		uint32_t seq = ntohl(tcp_hdr->seq);

		switch (sender_state_current) {
		case ECM_TRACKER_SENDER_STATE_UNKNOWN:
			/*
			 * Looks like we joined mid-flow.
			 * Have to set up both SYN and FIN.
			 * NOTE: It's possible that SYN is in the same packet as FIN, account for that in the seq numbers
			 */
			sender_state->state = ECM_TRACKER_SENDER_STATE_CLOSING;
			if (tcp_hdr->syn) {
				sender_state->syn_seq = seq;
				sender_state->fin_seq = seq + 1;
			} else {
				sender_state->fin_seq = seq;		/* seq is the FIN sequence */
				sender_state->syn_seq = seq - 1;	/* Make a guess at what the SYN was */
			}
			DEBUG_INFO("%p: From unkown to closing, syn_seq: %u, fin_seq: %u\n", ttii, sender_state->syn_seq, sender_state->fin_seq);
			break;
		case ECM_TRACKER_SENDER_STATE_ESTABLISHED:
			/*
			 * Connection becomes closing.
			 */
			sender_state->state = ECM_TRACKER_SENDER_STATE_CLOSING;
			sender_state->fin_seq = seq;
			DEBUG_INFO("%p: From established to closing, fin_seq: %u\n", ttii, sender_state->fin_seq);
			break;
		case ECM_TRACKER_SENDER_STATE_ESTABLISHING: {
			int32_t newer;

			/*
			 * FIN while waiting for SYN to be acknowledged is possible but only if it
			 * it is in the same packet or later sequence space
			 */
			newer = (int32_t)(seq - sender_state->syn_seq);
			if (!tcp_hdr->syn || (newer <= 0)) {
				DEBUG_INFO("%p: From establishing to fault - odd FIN seen, syn: %u, syn_seq: %u, newer: %d\n",
						ttii, tcp_hdr->syn, sender_state->syn_seq, newer);
				sender_state->state = ECM_TRACKER_SENDER_STATE_FAULT;
			} else {
				uint32_t fin_seq = seq;
				if (tcp_hdr->syn) {
					fin_seq++;
				}
				sender_state->state = ECM_TRACKER_SENDER_STATE_CLOSING;
				DEBUG_INFO("%p: From establishing to closing, syn: %u, syn_seq: %u, fin_seq: %u\n",
						ttii, tcp_hdr->syn, sender_state->syn_seq, sender_state->fin_seq);
			}
			break;
		}
		case ECM_TRACKER_SENDER_STATE_CLOSING:
		case ECM_TRACKER_SENDER_STATE_CLOSED:
		case ECM_TRACKER_SENDER_STATE_FAULT:
			/*
			 * No change
			 */
			break;
		default:
			DEBUG_ASSERT(false, "%p: unhandled state: %d\n", ttii, sender_state_current);
		}
	}

	spin_unlock_bh(&ttii->lock);
}

/*
 * ecm_tracker_tcp_state_get_callback()
 * 	Get state
 */
static void ecm_tracker_tcp_state_get_callback(struct ecm_tracker_instance *ti, ecm_tracker_sender_state_t *src_state,
					ecm_tracker_sender_state_t *dest_state, ecm_tracker_connection_state_t *state, ecm_db_timer_group_t *tg)
{
	struct ecm_tracker_tcp_internal_instance *ttii = (struct ecm_tracker_tcp_internal_instance *)ti;
	DEBUG_CHECK_MAGIC(ttii, ECM_TRACKER_TCP_INSTANCE_MAGIC, "%p: magic failed", ttii);
	spin_lock_bh(&ttii->lock);
	*src_state = ttii->sender_states[ECM_TRACKER_SENDER_TYPE_SRC].state;
	*dest_state = ttii->sender_states[ECM_TRACKER_SENDER_TYPE_DEST].state;
	spin_unlock_bh(&ttii->lock);
	*state = ecm_tracker_tcp_connection_state_matrix[*src_state][*dest_state];
	*tg = ecm_tracker_tcp_timer_group_from_state[*state];
}


/*
 * ecm_tracker_tcp_init()
 *	Initialise the instance
 */
void ecm_tracker_tcp_init(struct ecm_tracker_tcp_instance *tti, int32_t data_limit, uint16_t mss_src_default, uint16_t mss_dest_default)
{
	struct ecm_tracker_tcp_internal_instance *ttii = (struct ecm_tracker_tcp_internal_instance *)tti;
	DEBUG_CHECK_MAGIC(ttii, ECM_TRACKER_TCP_INSTANCE_MAGIC, "%p: magic failed", ttii);
	DEBUG_TRACE("%p: init host addresses src mss: %d, dest mss: %d\n", ttii, mss_src_default, mss_dest_default);
}
EXPORT_SYMBOL(ecm_tracker_tcp_init);

/*
 * ecm_tracker_tcp_alloc()
 */
struct ecm_tracker_tcp_instance *ecm_tracker_tcp_alloc(void)
{
	struct ecm_tracker_tcp_internal_instance *ttii;

	ttii = (struct ecm_tracker_tcp_internal_instance *)kzalloc(sizeof(struct ecm_tracker_tcp_internal_instance), GFP_ATOMIC | __GFP_NOWARN);
	if (!ttii) {
		DEBUG_WARN("Failed to allocate tcp tracker instance\n");
		return NULL;
	}

	ttii->tcp_base.base.ref = ecm_tracker_tcp_ref_callback;
	ttii->tcp_base.base.deref = ecm_tracker_tcp_deref_callback;
	ttii->tcp_base.base.state_update = ecm_tracker_tcp_state_update_callback;
	ttii->tcp_base.base.state_get = ecm_tracker_tcp_state_get_callback;

	spin_lock_init(&ttii->lock);

	ttii->refs = 1;
	DEBUG_SET_MAGIC(ttii, ECM_TRACKER_TCP_INSTANCE_MAGIC);

	spin_lock_bh(&ecm_tracker_tcp_lock);
	ecm_tracker_tcp_count++;
	DEBUG_ASSERT(ecm_tracker_tcp_count > 0, "%p: tcp tracker count wrap\n", ttii);
	spin_unlock_bh(&ecm_tracker_tcp_lock);

	DEBUG_TRACE("TCP tracker created %p\n", ttii);
	return (struct ecm_tracker_tcp_instance *)ttii;
}
EXPORT_SYMBOL(ecm_tracker_tcp_alloc);


/*
 * ecm_tracker_tcp_module_init()
 */
int ecm_tracker_tcp_module_init(void)
{
	DEBUG_INFO("TCP Tracker Module init\n");
	spin_lock_init(&ecm_tracker_tcp_lock);
	return 0;
}
EXPORT_SYMBOL(ecm_tracker_tcp_module_init);

/*
 * ecm_tracker_tcp_module_exit()
 */
void ecm_tracker_tcp_module_exit(void)
{
	DEBUG_INFO("TCP Tracker Module exit\n");
}
EXPORT_SYMBOL(ecm_tracker_tcp_module_exit);
