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

struct ecm_classifier_instance;

/*
 * Classifier types.
 * MUST BE RECORDED IN ASCENDING ORDER OF PRIORITY
 */
enum ecm_classifier_types {
	ECM_CLASSIFIER_TYPE_DEFAULT = 0,	/* MUST BE FIRST, Default classifier */
	ECM_CLASSIFIER_TYPES,			/* MUST BE LAST */
};
typedef enum ecm_classifier_types ecm_classifier_type_t;

/*
 * enum ecm_classifier_relevances
 *	Whether a classifier is relevant to a connection
 */
enum ecm_classifier_relevances {
	ECM_CLASSIFIER_RELEVANCE_MAYBE = 0,	/* Classifier has not yet determined relevance */
	ECM_CLASSIFIER_RELEVANCE_NO,		/* Classifier is not relevant to a connection (classifier will be unassigned from the connection after returning this from a process() call) */
	ECM_CLASSIFIER_RELEVANCE_YES,		/* Classifier is relevant to the connection, process actions will be inspected by the front end when returning this from a process() call */
};
typedef enum ecm_classifier_relevances ecm_classifier_relevence_t;

/*
 * enum ecm_classifier_acceleration_modes
 *	Modes in which a connection may be accelerated
 *
 * These are used by a classifier to indicate its desire to accelerate.
 */
enum ecm_classifier_acceleration_modes {
	ECM_CLASSIFIER_ACCELERATION_MODE_DONT_CARE = 0,		/* Classifier does not care if the connection is accelerated */
	ECM_CLASSIFIER_ACCELERATION_MODE_NO,			/* Connection must not be accelerated */
	ECM_CLASSIFIER_ACCELERATION_MODE_ACCEL,			/* Connection can be accelerated whenever */
};
typedef enum ecm_classifier_acceleration_modes ecm_classifier_acceleration_mode_t;

/*
 * Process actions
 * A process result, that is relevant, may contain zero or more actions for the front end.
 * Due to the parallel processing nature of classifiers, *usually* the action(s) of the highest priority
 * classifier will override any lower priority actions.  This is up to front end discretion, of course.
 */
#define ECM_CLASSIFIER_PROCESS_ACTION_DROP 0x00000001		/* Drop */
#define ECM_CLASSIFIER_PROCESS_ACTION_QOS_TAG 0x00000002	/* Contains flow & return qos tags */
#define ECM_CLASSIFIER_PROCESS_ACTION_ACCEL_MODE 0x00000004	/* Contains an accel mode */
#define ECM_CLASSIFIER_PROCESS_ACTION_TIMER_GROUP 0x00000008	/* Contains a timer group change */


/*
 * struct ecm_classifier_process_response
 *	Response structure returned by a process call
 */
struct ecm_classifier_process_response {
	ecm_classifier_relevence_t relevance;		/* Is this classifier relevant to the connection? */
	uint32_t became_relevant;			/* DB time the classifier became relevant or not relevant, if relevance is maybe this field is not relevant! */

	uint32_t process_actions;			/* Actions this process response contains */

	/*
	 * The following fields are only to be inspected if this response is relevant AND the process_actions indicates so
	 */
	bool drop;					/* Drop packet at hand */
	uint32_t flow_qos_tag;				/* QoS tag to use for the packet */
	uint32_t return_qos_tag;			/* QoS tag to use for the packet */
	ecm_classifier_acceleration_mode_t accel_mode;	/* Acceleration needed for this connection */
	ecm_db_timer_group_t timer_group;		/* Timer group the connection should be in */
};

/*
 * To be implemented by all classifiers
 */
typedef void (*ecm_classifier_ref_method_t)(struct ecm_classifier_instance *ci);
typedef int (*ecm_classifier_deref_callback_t)(struct ecm_classifier_instance *ci);
typedef void (*ecm_classifier_process_callback_t)(struct ecm_classifier_instance *ci, ecm_tracker_sender_type_t sender, struct ecm_tracker_ip_header *ip_hdr, struct sk_buff *skb, struct ecm_classifier_process_response *process_response);
											/* Process new data for connection, process_response is populated with the response of processing */
typedef void (*ecm_classifier_sync_from_v4_callback_t)(struct ecm_classifier_instance *ci, struct nss_ipv4_rule_create_msg *nircm);
											/* Sync the NSS state with state from the classifier */
typedef void (*ecm_classifier_sync_to_v4_callback_t)(struct ecm_classifier_instance *ci, struct nss_ipv4_conn_sync *sync);
											/* Sync the classifier state with current NSS state */
typedef void (*ecm_classifier_sync_from_v6_callback_t)(struct ecm_classifier_instance *ci, struct nss_ipv6_rule_create_msg *nircm);
											/* Sync the NSS state with state from the classifier */
typedef void (*ecm_classifier_sync_to_v6_callback_t)(struct ecm_classifier_instance *ci, struct nss_ipv6_conn_sync *sync);
											/* Sync the classifier state with current NSS state */
typedef ecm_classifier_type_t (*ecm_classifier_type_get_callback_t)(struct ecm_classifier_instance *ci);
											/* Get type of classifier this is */
typedef bool (*ecm_classifier_reclassify_allowed_get_callback_t)(struct ecm_classifier_instance *ci);
											/* Get whether reclassification is allowed */
typedef void (*ecm_classifier_reclassify_callback_t)(struct ecm_classifier_instance *ci);
											/* Reclassify */
typedef void (*ecm_classifier_last_process_response_get_callback_t)(struct ecm_classifier_instance *ci, struct ecm_classifier_process_response *process_response);
											/* Get last process response */

/*
 * Base class for all types of classifiers
 */
struct ecm_classifier_instance {
	struct ecm_classifier_instance *ca_next;	/* DB use only: Connection assignment next pointer */
	struct ecm_classifier_instance *ca_prev;	/* DB use only: Connection assignment prev pointer */

	ecm_classifier_process_callback_t process;	/* Process new skb */
	ecm_classifier_sync_from_v4_callback_t sync_from_v4;
							/* Sync the NSS with state from the classifier */
	ecm_classifier_sync_to_v4_callback_t sync_to_v4;/* Sync the classifier with state from the NSS */
	ecm_classifier_sync_from_v6_callback_t sync_from_v6;
							/* Sync the NSS with state from the classifier */
	ecm_classifier_sync_to_v6_callback_t sync_to_v6;/* Sync the classifier with state from the NSS */
	ecm_classifier_type_get_callback_t type_get;	/* Get type of classifier */
	ecm_classifier_reclassify_allowed_get_callback_t reclassify_allowed;
							/* Get whether reclassification is allowed */
	ecm_classifier_reclassify_callback_t reclassify;
							/* Reclassify */
	ecm_classifier_last_process_response_get_callback_t last_process_response_get;
							/* Return last process response */
	ecm_classifier_ref_method_t ref;
	ecm_classifier_deref_callback_t deref;
};




