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

/*
 * Bridge device macros
 */
#define ecm_front_end_is_bridge_port(dev) (dev && (dev->priv_flags & IFF_BRIDGE_PORT))
#define ecm_front_end_is_bridge_device(dev) (dev->priv_flags & IFF_EBRIDGE)


/*
 * enum ecm_front_end_acceleration_modes
 *	Acceleration mode of a connection as tracked by the front end.
 *
 * Typically when classifiers permit acceleration the front end will then accelerate the connection.
 * These states indicate the front end record of acceleration mode for the connection.
 * An acceleration mode less than zero indicates a connection that cannot be accelerated, maybe due to error.
 */
enum ecm_front_end_acceleration_modes {
	ECM_FRONT_END_ACCELERATION_MODE_FAIL_DEFUNCT = -7,	/* Acceleration has permanently failed due to the connection has become defunct */
	ECM_FRONT_END_ACCELERATION_MODE_FAIL_DECEL = -6,	/* Acceleration has permanently failed due to deceleration malfunction */
	ECM_FRONT_END_ACCELERATION_MODE_FAIL_NO_ACTION = -5,	/* Acceleration has permanently failed due to too many offloads that were rejected without any packets being offloaded */
	ECM_FRONT_END_ACCELERATION_MODE_FAIL_NSS = -4,		/* Acceleration has permanently failed due to too many NSS NAK's */
	ECM_FRONT_END_ACCELERATION_MODE_FAIL_DRIVER = -3,	/* Acceleration has permanently failed due to too many driver interaction failures */
	ECM_FRONT_END_ACCELERATION_MODE_FAIL_RULE = -2,		/* Acceleration has permanently failed due to bad rule data */
	ECM_FRONT_END_ACCELERATION_MODE_FAIL_DENIED = -1,	/* Acceleration has permanently failed due to can_accel denying accel */
	ECM_FRONT_END_ACCELERATION_MODE_DECEL = 0,		/* Connection is not accelerated */
	ECM_FRONT_END_ACCELERATION_MODE_ACCEL_PENDING,		/* Connection is in the process of being accelerated */
	ECM_FRONT_END_ACCELERATION_MODE_ACCEL,			/* Connection is accelerated */
	ECM_FRONT_END_ACCELERATION_MODE_DECEL_PENDING,		/* Connection is in the process of being decelerated */
};
typedef enum ecm_front_end_acceleration_modes ecm_front_end_acceleration_mode_t;

#define ECM_FRONT_END_ACCELERATION_NOT_POSSIBLE(accel_mode) ((accel_mode < 0) || (accel_mode == ECM_FRONT_END_ACCELERATION_MODE_DECEL_PENDING))
#define ECM_FRONT_END_ACCELERATION_POSSIBLE(accel_mode) (accel_mode >= 0)
#define ECM_FRONT_END_ACCELERATION_FAILED(accel_mode) (accel_mode < 0)

/*
 * Front end methods
 */
struct ecm_front_end_connection_instance;
typedef void (*ecm_front_end_connection_decelerate_method_t)(struct ecm_front_end_connection_instance *feci);
typedef ecm_front_end_acceleration_mode_t (*ecm_front_end_connection_accel_state_get_method_t)(struct ecm_front_end_connection_instance *feci);
typedef void (*ecm_front_end_connection_ref_method_t)(struct ecm_front_end_connection_instance *feci);
typedef int (*ecm_front_end_connection_deref_callback_t)(struct ecm_front_end_connection_instance *feci);
typedef void (*ecm_front_end_connection_action_seen_method_t)(struct ecm_front_end_connection_instance *feci);
typedef void (*ecm_front_end_connection_accel_ceased_method_t)(struct ecm_front_end_connection_instance *feci);

/*
 * Acceleration limiting modes.
 *	Used to apply limiting to accelerated connections.
 */
#define ECM_FRONT_END_ACCEL_LIMIT_MODE_UNLIMITED 0x00	/* No limits on acceleration rule creation */
#define ECM_FRONT_END_ACCEL_LIMIT_MODE_FIXED 0x01	/* Fixed upper limit for connection acceleration based on information from driver */

/*
 * Accel/decel mode statistics data structure.
 */
struct ecm_front_end_connection_mode_stats {
	bool decelerate_pending;		/* Decel was attempted during pending accel - will be actioned when accel is done */
	bool flush_happened;			/* A flush message was received from NSS before we received an ACK or NACK. (NSS Messaging sequence/ordering workaround) */
	uint32_t flush_happened_total;		/* Total of times we see flush_happened */
	uint32_t no_action_seen_total;		/* Total of times acceleration was ended by the NSS itself without any offload action */
	uint32_t no_action_seen;		/* Count of times consecutive  acceleration was ended by the NSS itself without any offload action */
	uint32_t no_action_seen_limit;		/* Limit on consecutive no-action at which point offload permanently fails out */
	uint32_t driver_fail_total;		/* Total times driver failed to send our command */
	uint32_t driver_fail;			/* Count of consecutive times driver failed to send our command, when this reaches driver_fail_limit acceleration will permanently fail */
	uint32_t driver_fail_limit;		/* Limit on drivers consecutive fails at which point offload permanently fails out */
	uint32_t nss_nack_total;		/* Total times NSS NAK's an accel command */
	uint32_t nss_nack;			/* Count of consecutive times driver failed to ack */
	uint32_t nss_nack_limit;		/* Limit on consecutive nacks at which point offload permanently fails out */
	unsigned long cmd_time_begun;		/* Time captured when an accel or decel request begun */
	unsigned long cmd_time_completed;	/* Time captured when request finished */
};

/*
 * Connection front end instance
 *	Each new connection requires it to also have one of these to maintain front end specific information and operations
 */
struct ecm_front_end_connection_instance {
	ecm_front_end_connection_ref_method_t ref;				/* Ref the instance */
	ecm_front_end_connection_deref_callback_t deref;			/* Deref the instance */
	ecm_front_end_connection_decelerate_method_t decelerate;		/* Decelerate a connection */
	ecm_front_end_connection_accel_state_get_method_t accel_state_get;	/* Get the acceleration state */
	ecm_front_end_connection_action_seen_method_t action_seen;		/* Acceleration action has occurred */
	ecm_front_end_connection_accel_ceased_method_t accel_ceased;		/* Acceleration has stopped */

	/*
	 * Accel/decel mode statistics.
	 */
	struct ecm_front_end_connection_mode_stats stats;

	/*
	 * Common control items to all front end instances
	 */
	struct ecm_db_connection_instance *ci;			/* RO: The connection instance relating to this instance. */
	bool can_accel;						/* RO: True when the connection can be accelerated */
	bool is_defunct;					/* True if the connection has become defunct */
	ecm_front_end_acceleration_mode_t accel_mode;		/* Indicates the type of acceleration being applied to a connection, if any. */
	spinlock_t lock;					/* Lock for structure data */
	int refs;						/* Integer to trap we never go negative */

};

