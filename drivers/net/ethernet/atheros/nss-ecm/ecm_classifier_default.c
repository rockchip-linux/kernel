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
#include <linux/ctype.h>
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
#define DEBUG_LEVEL ECM_CLASSIFIER_DEFAULT_DEBUG_LEVEL

#include <nss_api_if.h>

#include "ecm_types.h"
#include "ecm_db_types.h"
#include "ecm_tracker.h"
#include "ecm_classifier.h"
#include "ecm_front_end_types.h"
#include "ecm_tracker_datagram.h"
#include "ecm_tracker_udp.h"
#include "ecm_tracker_tcp.h"
#include "ecm_db.h"
#include "ecm_classifier_default.h"

/*
 * Magic numbers
 */
#define ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC 0x8761
#define ECM_CLASSIFIER_DEFAULT_STATE_FILE_INSTANCE_MAGIC 0x3321

/*
 * struct ecm_classifier_default_internal_instance
 * 	State to allow tracking of dynamic priority for a connection
 */
struct ecm_classifier_default_internal_instance {
	struct ecm_classifier_default_instance base;		/* Base type */

	uint32_t ci_serial;					/* RO: Serial of the connection */
	int protocol;						/* RO: Protocol of the connection */

	struct ecm_classifier_process_response process_response;
								/* Last process response computed */

	ecm_db_timer_group_t timer_group;			/* The timer group the connection should be in based on state */

	ecm_tracker_sender_type_t ingress_sender;		/* RO: Which sender is sending ingress data */
	ecm_tracker_sender_type_t egress_sender;		/* RO: Which sender is sending egress data */

	struct ecm_tracker_instance *ti;			/* RO: Tracker used for state and timer group checking. Pointer will not change so safe to access outside of lock. */

	int refs;						/* Integer to trap we never go negative */
#if (DEBUG_LEVEL > 0)
	uint16_t magic;
#endif
};

static spinlock_t ecm_classifier_default_lock;			/* Concurrency control SMP access */
static int ecm_classifier_default_count = 0;			/* Tracks number of instances allocated */

/*
 * Operational control
 */
static ecm_classifier_acceleration_mode_t ecm_classifier_default_accel_mode = ECM_CLASSIFIER_ACCELERATION_MODE_ACCEL;
								/* Cause connections whose hosts are both on-link to be accelerated */
static bool ecm_classifier_default_enabled = true;		/* When disabled the qos algorithm will not be applied to skb's */

/*
 * Management thread control
 */
static bool ecm_classifier_default_terminate_pending = false;	/* True when the user wants us to terminate */

/*
 * Character device stuff - used to communicate status back to user space
 */
#define ECM_CLASSIFIER_DEFAULT_STATE_FILE_BUFFER_SIZE 1024
struct ecm_classifier_default_state_file_instance {
	struct ecm_classifier_default_internal_instance *cdii;
	bool doc_start_written;
	bool doc_end_written;
	char msg_buffer[ECM_CLASSIFIER_DEFAULT_STATE_FILE_BUFFER_SIZE];	/* Used to hold the current state message being output */
	char *msgp;							/* Points into the msg buffer as we output it piece by piece */
	int msg_len;							/* Length of the buffer still to be written out */
#if (DEBUG_LEVEL > 0)
	uint16_t magic;
#endif
};
static struct device ecm_classifier_default_dev;		/* System device linkage */

/*
 * _ecm_classifier_default_ref()
 *	Ref
 */
static void _ecm_classifier_default_ref(struct ecm_classifier_default_internal_instance *cdii)
{
	cdii->refs++;
	DEBUG_TRACE("%p: cdii ref %d\n", cdii, cdii->refs);
	DEBUG_ASSERT(cdii->refs > 0, "%p: ref wrap\n", cdii);
}

/*
 * ecm_classifier_default_ref()
 *	Ref
 */
static void ecm_classifier_default_ref(struct ecm_classifier_instance *ci)
{
	struct ecm_classifier_default_internal_instance *cdii;
	cdii = (struct ecm_classifier_default_internal_instance *)ci;

	DEBUG_CHECK_MAGIC(cdii, ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC, "%p: magic failed", cdii);
	spin_lock_bh(&ecm_classifier_default_lock);
	_ecm_classifier_default_ref(cdii);
	spin_unlock_bh(&ecm_classifier_default_lock);
}

/*
 * ecm_classifier_default_deref()
 *	Deref
 */
static int ecm_classifier_default_deref(struct ecm_classifier_instance *ci)
{
	struct ecm_classifier_default_internal_instance *cdii;
	cdii = (struct ecm_classifier_default_internal_instance *)ci;

	DEBUG_CHECK_MAGIC(cdii, ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC, "%p: magic failed", cdii);
	spin_lock_bh(&ecm_classifier_default_lock);
	cdii->refs--;
	DEBUG_ASSERT(cdii->refs >= 0, "%p: refs wrapped\n", cdii);
	DEBUG_TRACE("%p: Default classifier deref %d\n", cdii, cdii->refs);
	if (cdii->refs) {
		int refs = cdii->refs;
		spin_unlock_bh(&ecm_classifier_default_lock);
		return refs;
	}

	/*
	 * Object to be destroyed
	 */
	ecm_classifier_default_count--;
	DEBUG_ASSERT(ecm_classifier_default_count >= 0, "%p: ecm_classifier_default_count wrap\n", cdii);

	spin_unlock_bh(&ecm_classifier_default_lock);

	/*
	 * Release our tracker
	 */
	cdii->ti->deref(cdii->ti);

	/*
	 * Final
	 */
	DEBUG_INFO("%p: Final default classifier instance\n", cdii);
	kfree(cdii);

	return 0;
}

/*
 * ecm_classifier_default_process_callback()
 *	Process new data updating the priority
 *
 * NOTE: This function would only ever be called if all other classifiers have failed.
 */
static void ecm_classifier_default_process(struct ecm_classifier_instance *aci, ecm_tracker_sender_type_t sender,
									struct ecm_tracker_ip_header *ip_hdr, struct sk_buff *skb,
									struct ecm_classifier_process_response *process_response)
{
	struct ecm_tracker_instance *ti;
	ecm_tracker_sender_state_t from_state;
	ecm_tracker_sender_state_t to_state;
	ecm_tracker_connection_state_t prevailing_state;
	ecm_db_timer_group_t tg;
	struct ecm_classifier_default_internal_instance *cdii = (struct ecm_classifier_default_internal_instance *)aci;
	struct nf_conn *ct;
	enum ip_conntrack_info ctinfo;
	DEBUG_CHECK_MAGIC(cdii, ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC, "%p: invalid state magic\n", cdii);


	spin_lock_bh(&ecm_classifier_default_lock);

	/*
	 * Get qos result and accel mode
	 * Default classifier is rarely disabled.
	 */
	if (unlikely(!ecm_classifier_default_enabled)) {
		/*
		 * Still relevant but have no actions that need processing
		 */
		cdii->process_response.process_actions = 0;
		*process_response = cdii->process_response;
		spin_unlock_bh(&ecm_classifier_default_lock);
		return;
	}

	/*
	 * Accel?
	 */
	if (ecm_classifier_default_accel_mode != ECM_CLASSIFIER_ACCELERATION_MODE_DONT_CARE) {
		cdii->process_response.accel_mode = ecm_classifier_default_accel_mode;
		cdii->process_response.process_actions |= ECM_CLASSIFIER_PROCESS_ACTION_ACCEL_MODE;
	} else {
		cdii->process_response.process_actions &= ~ECM_CLASSIFIER_PROCESS_ACTION_ACCEL_MODE;
	}
	spin_unlock_bh(&ecm_classifier_default_lock);

	/*
	 * Update connection state
	 * Compute the timer group this connection should be in.
	 * For this we need the tracker and the state to be updated.
	 * NOTE: Tracker does not need to be ref'd it will exist for as long as this default classifier instance does
	 * which is at least for the duration of this call.
	 */
	ti = cdii->ti;
	ti->state_update(ti, sender, ip_hdr, skb);
	ti->state_get(ti, &from_state, &to_state, &prevailing_state, &tg);
	spin_lock_bh(&ecm_classifier_default_lock);
	if (unlikely(cdii->timer_group != tg)) {
		/*
		 * Timer group has changed
		 */
		cdii->process_response.process_actions |= ECM_CLASSIFIER_PROCESS_ACTION_TIMER_GROUP;
		cdii->process_response.timer_group = tg;

		/*
		 * Record for future change comparisons
		 */
		cdii->timer_group = tg;
	}
	spin_unlock_bh(&ecm_classifier_default_lock);

	/*
	 * Handle non-TCP case
	 */
	if (cdii->protocol != IPPROTO_TCP) {
		if (unlikely(prevailing_state != ECM_TRACKER_CONNECTION_STATE_ESTABLISHED)) {
			cdii->process_response.accel_mode = ECM_CLASSIFIER_ACCELERATION_MODE_NO;
		}
		goto return_response;
	}

	/*
	 * Check the TCP connection state.
	 * If we are not established then we deny acceleration.
	 * Take lead from conntrack if exists.
	 */
	ct = nf_ct_get(skb, &ctinfo);
	if (ct == NULL) {
		DEBUG_TRACE("%p: No Conntrack found for packet, using ECM tracker state\n", cdii);
		if (unlikely(prevailing_state != ECM_TRACKER_CONNECTION_STATE_ESTABLISHED)) {
			cdii->process_response.accel_mode = ECM_CLASSIFIER_ACCELERATION_MODE_NO;
			goto return_response;
		}
	} else {
		/*
		 * Don't try to manage a non-established connection.
		 */
		if (!test_bit(IPS_ASSURED_BIT, &ct->status)) {
			DEBUG_TRACE("%p: Non-established connection\n", ct);
			cdii->process_response.accel_mode = ECM_CLASSIFIER_ACCELERATION_MODE_NO;
			goto return_response;
		}

		/*
		 * If the connection is shutting down do not manage it.
		 * state can not be SYN_SENT, SYN_RECV because connection is assured
		 * Not managed states: FIN_WAIT, CLOSE_WAIT, LAST_ACK, TIME_WAIT, CLOSE.
		 */
		spin_lock_bh(&ct->lock);
		if (ct->proto.tcp.state != TCP_CONNTRACK_ESTABLISHED) {
			spin_unlock_bh(&ct->lock);
			DEBUG_TRACE("%p: Connection in termination state %#X\n", ct, ct->proto.tcp.state);
			cdii->process_response.accel_mode = ECM_CLASSIFIER_ACCELERATION_MODE_NO;
			goto return_response;
		}
		spin_unlock_bh(&ct->lock);
	}

return_response:
	;

	/*
	 * Return the process response
	 */
	spin_lock_bh(&ecm_classifier_default_lock);
	*process_response = cdii->process_response;
	spin_unlock_bh(&ecm_classifier_default_lock);
}

/*
 * ecm_classifier_default_type_get()
 *	Get type of classifier this is
 */
static ecm_classifier_type_t ecm_classifier_default_type_get(struct ecm_classifier_instance *aci)
{
	struct ecm_classifier_default_internal_instance *cdii;
	cdii = (struct ecm_classifier_default_internal_instance *)aci;

	DEBUG_CHECK_MAGIC(cdii, ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC, "%p: magic failed", cdii);
	return ECM_CLASSIFIER_TYPE_DEFAULT;
}

/*
 * ecm_classifier_default_reclassify_allowed()
 *	Get whether reclassification is allowed
 */
static bool ecm_classifier_default_reclassify_allowed(struct ecm_classifier_instance *aci)
{
	struct ecm_classifier_default_internal_instance *cdii;
	cdii = (struct ecm_classifier_default_internal_instance *)aci;

	DEBUG_CHECK_MAGIC(cdii, ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC, "%p: magic failed", cdii);
	return true;
}

/*
 * ecm_classifier_default_reclassify()
 *	Reclassify
 */
static void ecm_classifier_default_reclassify(struct ecm_classifier_instance *aci)
{
	struct ecm_classifier_default_internal_instance *cdii;
	cdii = (struct ecm_classifier_default_internal_instance *)aci;
	DEBUG_CHECK_MAGIC(cdii, ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC, "%p: magic failed", cdii);
}

/*
 * ecm_classifier_default_last_process_response_get()
 *	Get result code returned by the last process call
 */
static void ecm_classifier_default_last_process_response_get(struct ecm_classifier_instance *aci,
							struct ecm_classifier_process_response *process_response)
{
	struct ecm_classifier_default_internal_instance *cdii;
	cdii = (struct ecm_classifier_default_internal_instance *)aci;
	DEBUG_CHECK_MAGIC(cdii, ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC, "%p: magic failed", cdii);

	spin_lock_bh(&ecm_classifier_default_lock);
	*process_response = cdii->process_response;
	spin_unlock_bh(&ecm_classifier_default_lock);
}

/*
 * ecm_classifier_default_sync_to_v4()
 *	Front end is pushing NSS state to us
 */
static void ecm_classifier_default_sync_to_v4(struct ecm_classifier_instance *aci, struct nss_ipv4_conn_sync *sync)
{
	struct ecm_classifier_default_internal_instance *cdii __attribute__((unused));

	cdii = (struct ecm_classifier_default_internal_instance *)aci;
	DEBUG_CHECK_MAGIC(cdii, ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC, "%p: magic failed", cdii);
}

/*
 * ecm_classifier_default_sync_from_v4()
 *	Front end is retrieving NSS state from us
 */
static void ecm_classifier_default_sync_from_v4(struct ecm_classifier_instance *aci, struct nss_ipv4_rule_create_msg *nircm)
{
	struct ecm_classifier_default_internal_instance *cdii __attribute__((unused));

	cdii = (struct ecm_classifier_default_internal_instance *)aci;
	DEBUG_CHECK_MAGIC(cdii, ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC, "%p: magic failed", cdii);
}

/*
 * ecm_classifier_default_sync_to_v6()
 *	Front end is pushing NSS state to us
 */
static void ecm_classifier_default_sync_to_v6(struct ecm_classifier_instance *aci, struct nss_ipv6_conn_sync *sync)
{
	struct ecm_classifier_default_internal_instance *cdii __attribute__((unused));

	cdii = (struct ecm_classifier_default_internal_instance *)aci;
	DEBUG_CHECK_MAGIC(cdii, ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC, "%p: magic failed", cdii);
}

/*
 * ecm_classifier_default_sync_from_v6()
 *	Front end is retrieving NSS state from us
 */
static void ecm_classifier_default_sync_from_v6(struct ecm_classifier_instance *aci, struct nss_ipv6_rule_create_msg *nircm)
{
	struct ecm_classifier_default_internal_instance *cdii __attribute__((unused));

	cdii = (struct ecm_classifier_default_internal_instance *)aci;
	DEBUG_CHECK_MAGIC(cdii, ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC, "%p: magic failed", cdii);
}

/*
 * ecm_classifier_tracker_get_and_ref()
 *	Obtain default classifiers tracker (usually for state tracking for the connection as it always exists for the connection)
 */
static struct ecm_tracker_instance *ecm_classifier_tracker_get_and_ref(struct ecm_classifier_default_instance *dci)
{
	struct ecm_classifier_default_internal_instance *cdii;
	struct ecm_tracker_instance *ti;

	cdii = (struct ecm_classifier_default_internal_instance *)dci;
	DEBUG_CHECK_MAGIC(cdii, ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC, "%p: magic failed", cdii);

	ti = cdii->ti;
	ti->ref(ti);
	return ti;
}


/*
 * ecm_classifier_default_instance_alloc()
 *	Allocate an instance of the default classifier
 */
struct ecm_classifier_default_instance *ecm_classifier_default_instance_alloc(struct ecm_db_connection_instance *ci, int protocol, ecm_db_direction_t dir, int from_port, int to_port)
{
	struct ecm_classifier_default_internal_instance *cdii;
	struct ecm_classifier_default_instance *cdi;

	/*
	 * Allocate the instance
	 */
	cdii = (struct ecm_classifier_default_internal_instance *)kzalloc(sizeof(struct ecm_classifier_default_internal_instance), GFP_ATOMIC | __GFP_NOWARN);
	if (!cdii) {
		DEBUG_WARN("Failed to allocate default instance\n");
		return NULL;
	}

	/*
	 * Allocate a tracker for state etc.
	 */
	if (protocol == IPPROTO_TCP) {
		DEBUG_TRACE("%p: Alloc tracker for TCP connection: %p\n", cdii, ci);
		cdii->ti = (struct ecm_tracker_instance *)ecm_tracker_tcp_alloc();
		if (!cdii->ti) {
			DEBUG_WARN("%p: Failed to alloc tracker\n", cdii);
			kfree(cdii);
			return NULL;
		}
		ecm_tracker_tcp_init((struct ecm_tracker_tcp_instance *)cdii->ti, ECM_TRACKER_CONNECTION_TRACKING_LIMIT_DEFAULT, 1500, 1500);
	} else if (protocol == IPPROTO_UDP) {
		DEBUG_TRACE("%p: Alloc tracker for UDP connection: %p\n", cdii, ci);
		cdii->ti = (struct ecm_tracker_instance *)ecm_tracker_udp_alloc();
		if (!cdii->ti) {
			DEBUG_WARN("%p: Failed to alloc tracker\n", cdii);
			kfree(cdii);
			return NULL;
		}
		ecm_tracker_udp_init((struct ecm_tracker_udp_instance *)cdii->ti, ECM_TRACKER_CONNECTION_TRACKING_LIMIT_DEFAULT, from_port, to_port);
	} else {
		DEBUG_TRACE("%p: Alloc tracker for non-ported connection: %p\n", cdii, ci);
		cdii->ti = (struct ecm_tracker_instance *)ecm_tracker_datagram_alloc();
		if (!cdii->ti) {
			DEBUG_WARN("%p: Failed to alloc tracker\n", cdii);
			kfree(cdii);
			return NULL;
		}
		ecm_tracker_datagram_init((struct ecm_tracker_datagram_instance *)cdii->ti, ECM_TRACKER_CONNECTION_TRACKING_LIMIT_DEFAULT);
	}

	DEBUG_SET_MAGIC(cdii, ECM_CLASSIFIER_DEFAULT_INTERNAL_INSTANCE_MAGIC);
	cdii->refs = 1;
	cdii->ci_serial = ecm_db_connection_serial_get(ci);
	cdii->protocol = protocol;

	/*
	 * We are always relevant to the connection
	 */
	cdii->process_response.relevance = ECM_CLASSIFIER_RELEVANCE_YES;

	/*
	 * Using the connection direction identify egress and ingress host addresses
	 */
	if (dir == ECM_DB_DIRECTION_INGRESS_NAT) {
		cdii->ingress_sender = ECM_TRACKER_SENDER_TYPE_SRC;
		cdii->egress_sender = ECM_TRACKER_SENDER_TYPE_DEST;
	} else {
		cdii->egress_sender = ECM_TRACKER_SENDER_TYPE_SRC;
		cdii->ingress_sender = ECM_TRACKER_SENDER_TYPE_DEST;
	}
	DEBUG_TRACE("%p: Ingress sender = %d egress sender = %d\n", cdii, cdii->ingress_sender, cdii->egress_sender);

	/*
	 * Methods specific to the default classifier
	 */
	cdi = (struct ecm_classifier_default_instance *)cdii;
	cdi->tracker_get_and_ref = ecm_classifier_tracker_get_and_ref;

	/*
	 * Methods generic to all classifiers.
	 */
	cdi->base.process = ecm_classifier_default_process;
	cdi->base.sync_from_v4 = ecm_classifier_default_sync_from_v4;
	cdi->base.sync_to_v4 = ecm_classifier_default_sync_to_v4;
	cdi->base.sync_from_v6 = ecm_classifier_default_sync_from_v6;
	cdi->base.sync_to_v6 = ecm_classifier_default_sync_to_v6;
	cdi->base.type_get = ecm_classifier_default_type_get;
	cdi->base.reclassify_allowed = ecm_classifier_default_reclassify_allowed;
	cdi->base.reclassify = ecm_classifier_default_reclassify;
	cdi->base.last_process_response_get = ecm_classifier_default_last_process_response_get;
	cdi->base.ref = ecm_classifier_default_ref;
	cdi->base.deref = ecm_classifier_default_deref;

	spin_lock_bh(&ecm_classifier_default_lock);

	/*
	 * Final check if we are pending termination
	 */
	if (ecm_classifier_default_terminate_pending) {
		spin_unlock_bh(&ecm_classifier_default_lock);
		DEBUG_INFO("%p: Terminating\n", ci);
		cdii->ti->deref(cdii->ti);
		kfree(cdii);
		return NULL;
	}

	/*
	 * Increment stats
	 */
	ecm_classifier_default_count++;
	DEBUG_ASSERT(ecm_classifier_default_count > 0, "%p: ecm_classifier_default_count wrap\n", cdii);
	spin_unlock_bh(&ecm_classifier_default_lock);

	DEBUG_INFO("Default classifier instance alloc: %p\n", cdii);
	return cdi;
}
EXPORT_SYMBOL(ecm_classifier_default_instance_alloc);

/*
 * ecm_classifier_default_get_accel_mode()
 *	Display accel_mode of classifier.
 */
static ssize_t ecm_classifier_default_get_accel_mode(struct device *dev,
		  struct device_attribute *attr,
		  char *buf)
{
	ssize_t count;
	uint32_t num;

	DEBUG_INFO("ecm_classifier_default_get_accel_mode\n");

	spin_lock_bh(&ecm_classifier_default_lock);
	num = (uint32_t)ecm_classifier_default_accel_mode;
	spin_unlock_bh(&ecm_classifier_default_lock);

	count = snprintf(buf, (ssize_t)PAGE_SIZE, "%u", num);
	return count;
}

/*
 * ecm_classifier_default_set_accel_mode()
 *	Set new accel mode value for default classifier.
 */
static ssize_t ecm_classifier_default_set_accel_mode(struct device *dev,
		  struct device_attribute *attr,
		  const char *buf, size_t count)
{
	char num_buf[12];
	uint32_t num;

	/*
	 * Get the number from buf into a properly z-termed number buffer
	 */
	if (count >= sizeof(num_buf)) return 0;
	memcpy(num_buf, buf, count);
	num_buf[count] = '\0';
	sscanf(num_buf, "%u", &num);

	DEBUG_TRACE("ecm_classifier_default_set_accel_mode = %u\n", num);

	spin_lock_bh(&ecm_classifier_default_lock);
	ecm_classifier_default_accel_mode = (ecm_classifier_acceleration_mode_t)num;
	spin_unlock_bh(&ecm_classifier_default_lock);
	return count;
}

/*
 * ecm_classifier_default_get_enabled()
 *	Display enabled of classifier.
 */
static ssize_t ecm_classifier_default_get_enabled(struct device *dev,
		  struct device_attribute *attr,
		  char *buf)
{
	ssize_t count;
	uint32_t num;

	DEBUG_INFO("ecm_classifier_default_get_enabled\n");

	spin_lock_bh(&ecm_classifier_default_lock);
	num = (uint32_t)ecm_classifier_default_enabled;
	spin_unlock_bh(&ecm_classifier_default_lock);

	count = snprintf(buf, (ssize_t)PAGE_SIZE, "%u", num);
	return count;
}

/*
 * ecm_classifier_default_set_enabled()
 *	Set new enabled value for default classifier.
 */
static ssize_t ecm_classifier_default_set_enabled(struct device *dev,
		  struct device_attribute *attr,
		  const char *buf, size_t count)
{
	char num_buf[12];
	uint32_t num;

	/*
	 * Get the number from buf into a properly z-termed number buffer
	 */
	if (count >= sizeof(num_buf)) return 0;
	memcpy(num_buf, buf, count);
	num_buf[count] = '\0';
	sscanf(num_buf, "%u", &num);

	DEBUG_TRACE("ecm_classifier_default_set_enabled = %u\n", num);

	spin_lock_bh(&ecm_classifier_default_lock);
	ecm_classifier_default_enabled = (bool)num;
	spin_unlock_bh(&ecm_classifier_default_lock);
	return count;
}

static DEVICE_ATTR(accel_mode, 0644, ecm_classifier_default_get_accel_mode, ecm_classifier_default_set_accel_mode);
static DEVICE_ATTR(enabled, 0644, ecm_classifier_default_get_enabled, ecm_classifier_default_set_enabled);

/*
 * System device attribute array.
 */
static struct device_attribute *ecm_classifier_default_attrs[] = {
	&dev_attr_accel_mode,
	&dev_attr_enabled,
};

/*
 * System device node the ECM default classifier
 * Sysdevice control points can be found at /sys/devices/system/ecm_classifier_default/ecm_classifier_defaultX/
 */
static struct bus_type ecm_classifier_default_subsys = {
	.name = "ecm_classifier_default",
	.dev_name = "ecm_classifier_default",
};

/*
 * ecm_classifier_default_dev_release()
 *	This is a dummy release function for device.
 */
static void ecm_classifier_default_dev_release(struct device *dev)
{

}

/*
 * ecm_classifier_default_init()
 */
int ecm_classifier_default_init(void)
{
	int result;
	int i;
	DEBUG_INFO("Default classifier Module init\n");

	DEBUG_ASSERT(ECM_CLASSIFIER_TYPE_DEFAULT == 0, "DO NOT CHANGE DEFAULT PRIORITY");

	/*
	 * Initialise our global lock
	 */
	spin_lock_init(&ecm_classifier_default_lock);

	/*
	 * Register the Sub system
	 */
	result = subsys_system_register(&ecm_classifier_default_subsys, NULL);
	if (result) {
		DEBUG_ERROR("Failed to register sub system %d\n", result);
		return result;
	}

	/*
	 * Register System device control
	 */
	memset(&ecm_classifier_default_dev, 0, sizeof(ecm_classifier_default_dev));
	ecm_classifier_default_dev.id = 0;
	ecm_classifier_default_dev.bus = &ecm_classifier_default_subsys;
	ecm_classifier_default_dev.release = &ecm_classifier_default_dev_release;
	result = device_register(&ecm_classifier_default_dev);
	if (result) {
		DEBUG_ERROR("Failed to register System device %d\n", result);
		goto classifier_task_cleanup_1;
	}

	/*
	 * Create files, one for each parameter supported by this module
	 */
	for (i = 0; i < ARRAY_SIZE(ecm_classifier_default_attrs); i++) {
		result = device_create_file(&ecm_classifier_default_dev, ecm_classifier_default_attrs[i]);
		if (result) {
			DEBUG_ERROR("Failed to register system device file %d\n", result);
			goto classifier_task_cleanup_2;
		}
	}

	return 0;

classifier_task_cleanup_2:
	while (--i >= 0) {
		device_remove_file(&ecm_classifier_default_dev, ecm_classifier_default_attrs[i]);
	}
	device_unregister(&ecm_classifier_default_dev);
classifier_task_cleanup_1:
	bus_unregister(&ecm_classifier_default_subsys);

	return result;
}
EXPORT_SYMBOL(ecm_classifier_default_init);

/*
 * ecm_classifier_default_exit()
 */
void ecm_classifier_default_exit(void)
{
	int i;
	DEBUG_INFO("Default classifier Module exit\n");
	spin_lock_bh(&ecm_classifier_default_lock);
	ecm_classifier_default_terminate_pending = true;
	spin_unlock_bh(&ecm_classifier_default_lock);

	for (i = 0; i < ARRAY_SIZE(ecm_classifier_default_attrs); i++) {
		device_remove_file(&ecm_classifier_default_dev, ecm_classifier_default_attrs[i]);
	}

	device_unregister(&ecm_classifier_default_dev);
	bus_unregister(&ecm_classifier_default_subsys);
}
EXPORT_SYMBOL(ecm_classifier_default_exit);
