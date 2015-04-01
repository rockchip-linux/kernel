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

extern int ecm_nss_ipv4_no_action_limit_default;		/* Default no-action limit. */
extern int ecm_nss_ipv4_driver_fail_limit_default;		/* Default driver fail limit. */
extern int ecm_nss_ipv4_nack_limit_default;			/* Default nack limit. */
extern int ecm_nss_ipv4_accelerated_count;			/* Total offloads */
extern int ecm_nss_ipv4_pending_accel_count;			/* Total pending offloads issued to the NSS / awaiting completion */
extern int ecm_nss_ipv4_pending_decel_count;			/* Total pending deceleration requests issued to the NSS / awaiting completion */

/*
 * Limiting the acceleration of connections.
 *
 * By default there is no acceleration limiting.
 * This means that when ECM has more connections (that can be accelerated) than the acceleration
 * engine will allow the ECM will continue to try to accelerate.
 * In this scenario the acceleration engine will begin removal of existing rules to make way for new ones.
 * When the accel_limit_mode is set to FIXED ECM will not permit more rules to be issued than the engine will allow.
 */
extern uint32_t ecm_nss_ipv4_accel_limit_mode;

/*
 * Locking of the classifier - concurrency control for file global parameters.
 * NOTE: It is safe to take this lock WHILE HOLDING a feci->lock.  The reverse is NOT SAFE.
 */
extern spinlock_t ecm_nss_ipv4_lock;			/* Protect against SMP access between netfilter, events and private threaded function. */

/*
 * Management thread control
 */
extern bool ecm_nss_ipv4_terminate_pending;		/* True when the user has signalled we should quit */

/*
 * NSS driver linkage
 */
extern struct nss_ctx_instance *ecm_nss_ipv4_nss_ipv4_mgr;

/*
 * ecm_nss_ipv4_accel_pending_set()
 *	Set pending acceleration for the connection object.
 *
 * Return false if the acceleration is not permitted or is already in progress.
 */
static inline bool ecm_nss_ipv4_accel_pending_set(struct ecm_front_end_connection_instance *feci)
{
	/*
	 * Can this connection be accelerated at all?
	 */
	DEBUG_INFO("%p: Accel conn: %p\n", feci, feci->ci);
	spin_lock_bh(&feci->lock);
	if (ECM_FRONT_END_ACCELERATION_FAILED(feci->accel_mode)) {
		spin_unlock_bh(&feci->lock);
		DEBUG_TRACE("%p: accel %p failed\n", feci, feci->ci);
		return false;
	}

	/*
	 * If acceleration mode is anything other than "not accelerated" then ignore.
	 */
	if (feci->accel_mode != ECM_FRONT_END_ACCELERATION_MODE_DECEL) {
		spin_unlock_bh(&feci->lock);
		DEBUG_TRACE("%p: Ignoring wrong mode accel for conn: %p\n", feci, feci->ci);
		return false;
	}

	/*
	 * Do we have a fixed upper limit for acceleration?
	 */
	spin_lock_bh(&ecm_nss_ipv4_lock);
	if (ecm_nss_ipv4_accel_limit_mode & ECM_FRONT_END_ACCEL_LIMIT_MODE_FIXED) {
		if ((ecm_nss_ipv4_pending_accel_count + ecm_nss_ipv4_accelerated_count) >= nss_ipv4_max_conn_count()) {
			spin_unlock_bh(&ecm_nss_ipv4_lock);
			spin_unlock_bh(&feci->lock);
			DEBUG_INFO("%p: Accel limit reached, accel denied: %p\n", feci, feci->ci);
			return false;
		}
	}

	/*
	 * Okay to accelerate
	 */
	ecm_nss_ipv4_pending_accel_count++;
	spin_unlock_bh(&ecm_nss_ipv4_lock);

	/*
	 * Okay connection can be set to pending acceleration
	 */
	feci->accel_mode = ECM_FRONT_END_ACCELERATION_MODE_ACCEL_PENDING;
	spin_unlock_bh(&feci->lock);
	return true;
}

/*
 * _ecm_nss_ipv4_accel_pending_clear()
 *	Clear pending acceleration for the connection object, setting it to the desired state.
 *
 * Returns true if "decelerate was pending".
 *
 * The feci->lock AND ecm_nss_ipv4_lock must be held on entry.
 */
static inline bool _ecm_nss_ipv4_accel_pending_clear(struct ecm_front_end_connection_instance *feci, ecm_front_end_acceleration_mode_t mode)
{
	bool decel_pending;

	/*
	 * Set the mode away from its accel pending state.
	 */
	DEBUG_ASSERT(feci->accel_mode == ECM_FRONT_END_ACCELERATION_MODE_ACCEL_PENDING, "%p: Accel mode unexpected: %d\n", feci, feci->accel_mode);
	feci->accel_mode = mode;

	/*
	 * Clear decelerate pending flag.
	 * This flag is only set when we are ACCEL_PENDING -
	 * and we are moving from that to the given mode anyway.
	 */
	decel_pending = feci->stats.decelerate_pending;
	feci->stats.decelerate_pending = false;

	/*
	 * Decrement pending counter
	 */
	ecm_nss_ipv4_pending_accel_count--;
	DEBUG_ASSERT(ecm_nss_ipv4_pending_accel_count >= 0, "Accel pending underflow\n");
	return decel_pending;
}

/*
 * ecm_nss_ipv4_accel_pending_clear()
 *	Clear pending acceleration for the connection object, setting it to the desired state.
 */
static inline bool ecm_nss_ipv4_accel_pending_clear(struct ecm_front_end_connection_instance *feci, ecm_front_end_acceleration_mode_t mode)
{
	bool decel_pending;
	spin_lock_bh(&feci->lock);
	spin_lock_bh(&ecm_nss_ipv4_lock);
	decel_pending = _ecm_nss_ipv4_accel_pending_clear(feci, mode);
	spin_unlock_bh(&ecm_nss_ipv4_lock);
	spin_unlock_bh(&feci->lock);
	return decel_pending;
}

extern int ecm_nss_ipv4_conntrack_event(unsigned long events, struct nf_conn *ct);
extern void ecm_nss_ipv4_accel_done_time_update(struct ecm_front_end_connection_instance *feci);
extern void ecm_nss_ipv4_decel_done_time_update(struct ecm_front_end_connection_instance *feci);
extern struct ecm_classifier_instance *ecm_nss_ipv4_assign_classifier(struct ecm_db_connection_instance *ci, ecm_classifier_type_t type);
extern bool ecm_nss_ipv4_reclassify(struct ecm_db_connection_instance *ci, int assignment_count, struct ecm_classifier_instance *assignments[]);
extern bool ecm_nss_ipv4_connection_regenerate(struct ecm_db_connection_instance *ci, ecm_tracker_sender_type_t sender,
							struct net_device *out_dev, struct net_device *out_dev_nat,
							struct net_device *in_dev, struct net_device *in_dev_nat);
extern struct ecm_db_node_instance *ecm_nss_ipv4_node_establish_and_ref(struct net_device *dev, ip_addr_t addr,
							struct ecm_db_iface_instance *interface_list[], int32_t interface_list_first,
							uint8_t *given_node_addr);
extern struct ecm_db_host_instance *ecm_nss_ipv4_host_establish_and_ref(ip_addr_t addr);
extern struct ecm_db_mapping_instance *ecm_nss_ipv4_mapping_establish_and_ref(ip_addr_t addr, int port);
