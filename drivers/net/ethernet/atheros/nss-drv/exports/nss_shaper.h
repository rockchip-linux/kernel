/*
 **************************************************************************
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

/**
 * nss_shaper.c
 * 	NSS shaper definitions
 */
#ifndef __NSS_SHAPER_H
#define __NSS_SHAPER_H

/*
 * enum nss_shaper_node_types
 *	Types of shaper node we export to the HLOS
 */
enum nss_shaper_node_types {
	NSS_SHAPER_NODE_TYPE_CODEL = 1,
	NSS_SHAPER_NODE_TYPE_PRIO = 3,
	NSS_SHAPER_NODE_TYPE_FIFO = 4,
	NSS_SHAPER_NODE_TYPE_TBL = 5,
	NSS_SHAPER_NODE_TYPE_BF = 6,
	NSS_SHAPER_NODE_TYPE_BF_GROUP = 7,
	NSS_SHAPER_NODE_TYPE_WRR = 9,
	NSS_SHAPER_NODE_TYPE_WRR_GROUP = 10,
	NSS_SHAPER_NODE_TYPE_HTB = 11,
	NSS_SHAPER_NODE_TYPE_HTB_GROUP = 12,
	NSS_SHAPER_NODE_TYPE_WRED = 13,
};
typedef enum nss_shaper_node_types nss_shaper_node_type_t;

/*
 * enum nss_shaper_config_types
 *	Types of shaper configuration messages
 */
enum nss_shaper_config_types {
	NSS_SHAPER_CONFIG_TYPE_ALLOC_SHAPER_NODE,	/* Allocate a type of shaper node and give it a qos tag */
	NSS_SHAPER_CONFIG_TYPE_FREE_SHAPER_NODE,	/* Free a shaper node */
	NSS_SHAPER_CONFIG_TYPE_PRIO_ATTACH,		/* Configure prio to attach a node with a given priority */
	NSS_SHAPER_CONFIG_TYPE_PRIO_DETACH,		/* Configure prio to detach a node at a given priority */
	NSS_SHAPER_CONFIG_TYPE_SET_DEFAULT,		/* Configure shaper to have a default node */
	NSS_SHAPER_CONFIG_TYPE_SET_ROOT,		/* Configure shaper to have a root node */
	NSS_SHAPER_CONFIG_TYPE_CODEL_CHANGE_PARAM,	/* Configure codel parameters */
	NSS_SHAPER_CONFIG_TYPE_TBL_ATTACH,		/* Configure tbl to attach a child node */
	NSS_SHAPER_CONFIG_TYPE_TBL_DETACH,		/* Configure tbl to detach its child node */
	NSS_SHAPER_CONFIG_TYPE_TBL_CHANGE_PARAM,	/* Configure tbl to tune its parameters */
	NSS_SHAPER_CONFIG_TYPE_BF_ATTACH,		/* Configure bf to attach a node to its round robin list */
	NSS_SHAPER_CONFIG_TYPE_BF_DETACH,		/* Configure bf to detach a node with a particular QoS tag */
	NSS_SHAPER_CONFIG_TYPE_BF_GROUP_ATTACH,	/* Configure bf group to attach a node as child */
	NSS_SHAPER_CONFIG_TYPE_BF_GROUP_DETACH,	/* Configure bf group to detach its child */
	NSS_SHAPER_CONFIG_TYPE_BF_GROUP_CHANGE_PARAM,	/* Configure bf group to tune its parameters */
	NSS_SHAPER_CONFIG_TYPE_FIFO_CHANGE_PARAM,		/* Configure fifo queue limit */
	NSS_SHAPER_CONFIG_TYPE_SHAPER_NODE_BASIC_STATS_GET,
							/* Get shaper node basic stats */
	NSS_SHAPER_CONFIG_TYPE_WRR_ATTACH,		/* Configure wrr to attach a node to its round robin list */
	NSS_SHAPER_CONFIG_TYPE_WRR_DETACH,		/* Configure wrr to detach a node with a particular QoS tag */
	NSS_SHAPER_CONFIG_TYPE_WRR_CHANGE_PARAM,	/* Configure wrr group to tune its parameters */
	NSS_SHAPER_CONFIG_TYPE_WRR_GROUP_ATTACH,	/* Configure wrr group to attach a node as child */
	NSS_SHAPER_CONFIG_TYPE_WRR_GROUP_DETACH,	/* Configure wrr group to detach its child */
	NSS_SHAPER_CONFIG_TYPE_WRR_GROUP_CHANGE_PARAM,	/* Configure wrr group to tune its parameters */
	/*
	 * Generic shaper node commands
	 *
	 * TODO: The per type repetition of messages (above) needs to be
	 * removed. This is not necessary in the new messaging
	 * framework.
	 */
	NSS_SHAPER_CONFIG_TYPE_SHAPER_NODE_ATTACH,	/* Command to attach a shaper node as child */
	NSS_SHAPER_CONFIG_TYPE_SHAPER_NODE_DETACH,	/* Command to detach a child shaper node */
	NSS_SHAPER_CONFIG_TYPE_SHAPER_NODE_CHANGE_PARAM,/* Command to configure the parameters of a shaper node */
};
typedef enum nss_shaper_config_types nss_shaper_config_type_t;

/*
 * enum nss_shaper_response_types
 *	Types of shaper configuration response messages
 */
enum nss_shaper_response_types {
	/*
	 * Failure messages are < 0
	 */
	NSS_SHAPER_RESPONSE_TYPE_NO_SHAPER_NODE = -65536,		/* No shaper node to which to issue a configuration message */
	NSS_SHAPER_RESPONSE_TYPE_NO_SHAPER_NODES,			/* No available shaper nodes available of the type requested */
	NSS_SHAPER_RESPONSE_TYPE_OLD,					/* Request is old / environment changed by the time the request was processed */
	NSS_SHAPER_RESPONSE_TYPE_UNRECOGNISED,				/* Request is not recognised by the recipient */
	NSS_SHAPER_RESPONSE_TYPE_FIFO_QUEUE_LIMIT_INVALID,		/* Fifo queue Limit is bad */
	NSS_SHAPER_RESPONSE_TYPE_FIFO_DROP_MODE_INVALID,		/* Fifo Drop mode is bad */
	NSS_SHAPER_RESPONSE_TYPE_BAD_DEFAULT_CHOICE,			/* Node selected has no queue to enqueue to */
	NSS_SHAPER_RESPONSE_TYPE_DUPLICATE_QOS_TAG,			/* Duplicate QoS Tag as another node */
	NSS_SHAPER_RESPONSE_TYPE_TBL_CIR_RATE_AND_BURST_REQUIRED,	/* CIR rate and burst are mandatory */
	NSS_SHAPER_RESPONSE_TYPE_TBL_CIR_BURST_LESS_THAN_MTU,		/* CIR burst size is smaller than MTU */
	NSS_SHAPER_RESPONSE_TYPE_TBL_PIR_BURST_LESS_THAN_MTU,		/* PIR burst size is smaller than MTU */
	NSS_SHAPER_RESPONSE_TYPE_TBL_PIR_BURST_REQUIRED,		/* PIR burst size must be provided if peakrate
									 * limiting is required.
									 */
	NSS_SHAPER_RESPONSE_TYPE_CODEL_ALL_PARAMS_REQUIRED,		/* Codel requires non-zero value for target,
									 * interval and limit.
									 */
	NSS_SHAPER_RESPONSE_TYPE_BF_GROUP_RATE_AND_BURST_REQUIRED,	/* Burst and rate are mandatory */
	NSS_SHAPER_RESPONSE_TYPE_BF_GROUP_BURST_LESS_THAN_MTU,		/* Burst size should be latger than MTU */
	NSS_SHAPER_RESPONSE_TYPE_CHILD_NOT_BF_GROUP,			/*
									 * Bf can have only bf_group as
									 * child nodes.
									 */
	NSS_SHAPER_RESPONSE_TYPE_WRR_GROUP_INVALID_QUANTUM,		/* Quantum cannot be zero */
	NSS_SHAPER_RESPONSE_TYPE_CHILD_NOT_WRR_GROUP,			/* Wrr cannot have non-wrr_group as a
									 * child node */
	NSS_SHAPER_RESPONSE_TYPE_WRR_INVALID_OPERATION_MODE,		/* Wrr requires a valid mode */
	NSS_SHAPER_RESPONSE_TYPE_WRED_WEIGHT_MODE_INVALID,		/* Invalid wred weight mode */
	NSS_SHAPER_RESPONSE_TYPE_HTB_GROUP_BURST_LESS_THAN_MTU,		/* Burst and rate are mandatory */
	NSS_SHAPER_RESPONSE_TYPE_HTB_GROUP_PRIORITY_OUT_OF_RANGE,	/* Assigned priority larger than max priority */
	NSS_SHAPER_RESPONSE_TYPE_CHILDREN_BELONG_TO_MIXED_TYPES,	/* The class cannot have a mix of class and qdisc as child nodes */
	NSS_SHAPER_RESPONSE_TYPE_CHILD_ALREADY_PRESENT,			/* Child already present for this qdisc/class */
	NSS_SHAPER_RESPONSE_TYPE_CHILD_MISMATCH,			/* The QoS tag of child does not match with the one provided */
	NSS_SHAPER_RESPONSE_TYPE_CHILD_UNSUPPORTED,			/* This type of qdisc/class cannot be attached as a child */
	NSS_SHAPER_RESPONSE_TYPE_CHILD_NOT_FOUND,			/* Child with provided Qos tag not found */
	NSS_SHAPER_RESPONSE_TYPE_ATTACH_FAIL,				/* The attach process failed */

	/*
	 * Success messages are >= 0
	 */
	SHAPER_RESPONSE_TYPE_SHAPER_NODE_ALLOC_SUCCESS = 0,	/* Shaper node alloc success */
	NSS_SHAPER_RESPONSE_TYPE_PRIO_ATTACH_SUCCESS,		/* Prio attach success */
	NSS_SHAPER_RESPONSE_TYPE_PRIO_DETACH_SUCCESS,		/* Prio detach success */
	NSS_SHAPER_RESPONSE_TYPE_CODEL_CHANGE_PARAM_SUCCESS,	/* Codel parameter configuration success */
	NSS_SHAPER_RESPONSE_TYPE_TBL_ATTACH_SUCCESS,		/* Tbl attach success */
	NSS_SHAPER_RESPONSE_TYPE_TBL_DETACH_SUCCESS,		/* Tbl detach success */
	NSS_SHAPER_RESPONSE_TYPE_TBL_CHANGE_PARAM_SUCCESS,	/* Tbl parameter configuration success */
	NSS_SHAPER_RESPONSE_TYPE_BF_ATTACH_SUCCESS,		/* Bf attach success */
	NSS_SHAPER_RESPONSE_TYPE_BF_DETACH_SUCCESS,		/* Bf detach success */
	NSS_SHAPER_RESPONSE_TYPE_BF_GROUP_ATTACH_SUCCESS,	/* Bf group attach success */
	NSS_SHAPER_RESPONSE_TYPE_BF_GROUP_DETACH_SUCCESS,	/* Bf group detach success */
	NSS_SHAPER_RESPONSE_TYPE_BF_GROUP_CHANGE_PARAM_SUCCESS,
								/* Bf group parameter configuration success */
	NSS_SHAPER_RESPONSE_TYPE_SHAPER_SET_ROOT_SUCCESS,	/* Setting of root successful */
	NSS_SHAPER_RESPONSE_TYPE_SHAPER_SET_DEFAULT_SUCCESS,	/* Setting of default successful */
	NSS_SHAPER_RESPONSE_TYPE_SHAPER_NODE_FREE_SUCCESS,	/* Free shaper node request successful */
	NSS_SHAPER_RESPONSE_TYPE_SHAPER_UNASSIGN_SUCCESS,	/* Successfully unassigned a shaper */
	NSS_SHAPER_RESPONSE_TYPE_FIFO_CHANGE_PARAM_SUCCESS,	/* Fifo limit set success */
	NSS_SHAPER_RESPONSE_TYPE_SHAPER_NODE_BASIC_STATS_GET_SUCCESS,
								/* Success response for a shaper node basic stats get request */
	NSS_SHAPER_RESPONSE_TYPE_WRR_ATTACH_SUCCESS,		/* Wrr attach success */
	NSS_SHAPER_RESPONSE_TYPE_WRR_DETACH_SUCCESS,		/* Wrr detach success */
	NSS_SHAPER_RESPONSE_TYPE_WRR_CHANGE_PARAM_SUCCESS,	/* Wrr parameter configuration success */
	NSS_SHAPER_RESPONSE_TYPE_WRR_GROUP_ATTACH_SUCCESS,	/* Wrr group attach success */
	NSS_SHAPER_RESPONSE_TYPE_WRR_GROUP_DETACH_SUCCESS,	/* Wrr group detach success */
	NSS_SHAPER_RESPONSE_TYPE_WRR_GROUP_CHANGE_PARAM_SUCCESS,/* Wrr group parameter configuration success */
	/*
	 * Generic success response.
	 *
	 * TODO: The per message success responses (above) needs
	 * to be removed. This is not necessary in the new messaging
	 * framework.
	 */
	NSS_SHAPER_RESPONSE_TYPE_SUCCESS,			/* Response on successful command execution */
};
typedef enum nss_shaper_response_types nss_shaper_response_type_t;

/*
 * struct nss_shaper_config_alloc_shaper_node
 *	A shaper node with this qos_tag will be allocated in the NSS.
 */
struct nss_shaper_config_alloc_shaper_node {
	nss_shaper_node_type_t node_type;	/* Type of shaper node */
	uint32_t qos_tag;			/* The qos tag to give the new node */
};

/*
 * struct nss_shaper_config_free_shaper_node
 *	Frees the shaper node with this qos_tag will be freed.
 */
struct nss_shaper_config_free_shaper_node {
	uint32_t qos_tag;		/* The qos tag of the node to free */
};

/*
 * struct nss_shaper_config_set_root_node
 *	The shaper node with this qos_tag will be set as the root shaper node.
 */
struct nss_shaper_config_set_root_node {
	uint32_t qos_tag;		/* The qos tag of the node that becomes root */
};

/*
 * struct nss_shaper_config_set_default_node
 *	The shaper node with this qos_tag will be set as the default node for qneueue.
 */
struct nss_shaper_config_set_default_node {
	uint32_t qos_tag;		/* The qos tag of the node that becomes default */
};

/*
 * struct nss_shaper_config_prio_attach
 *	The shaper node with qos_tag 'child_qos_tag' will be attached to the mentioned priority.
 */
struct nss_shaper_config_prio_attach {
	uint32_t child_qos_tag;		/* Qos tag of shaper node to add as child */
	uint32_t priority;		/* Priority of the child */
};

/*
 * struct nss_shaper_config_prio_detach
 *	Prio detaches shaper node at mentioned priority.
 */
struct nss_shaper_config_prio_detach {
	uint32_t priority;		/* Priority of the child to detach */
};

/*
 * struct nss_shaper_config_codel_alg_param
 *	List of codel algorithm parameters.
 */
struct nss_shaper_config_codel_alg_param {
	uint16_t interval;		/* Buffer time to smoothen state transition */
	uint16_t target;		/* Acceptable delay associated with a queue */
	uint16_t mtu;			/* MTU for the associated interface */
	uint16_t reserved;		/* Reserved for alignment */
};

/*
 * struct nss_shaper_config_codel_param
 *	Configures codel shaper node with the mentioned parameters.
 */
struct nss_shaper_config_codel_param {
	int32_t qlen_max;					/* Max no. of packets that can be enqueued */
	struct nss_shaper_config_codel_alg_param cap;		/* Config structure for codel algorithm */
};

/*
 * struct nss_shaper_config_rate_param
 *	Parameters related to the rate limiter algorithm.
 */
struct nss_shaper_config_rate_param {
	uint32_t rate;		/* Allowed Traffic rate measured in bytes per second */
	uint32_t burst;		/* Max bytes that can be sent before the next token update */
	uint32_t max_size;	/* The maximum size of packets (in bytes) supported */
	bool short_circuit;	/* When set, limiter will stop limiting the sending rate */
};

/*
 * struct nss_shaper_configure_tbl_attach
 *	Attaches the shaper node with the mentioned qos_tag to tbl.
 */
struct nss_shaper_config_tbl_attach {
	uint32_t child_qos_tag;	/* Qos tag of shaper node to add as child */
};

/*
 * struct nss_shaper_configure_tbl_param
 *	Configures tbl with the mentioned parameters.
 */
struct nss_shaper_config_tbl_param {
	struct nss_shaper_config_rate_param lap_cir;		/* Config committed information rate */
	struct nss_shaper_config_rate_param lap_pir;		/* Config committed information rate */
};

/*
 * struct nss_shaper_config_bf_attach
 *	Attaches shaper node with qos_tag to bf shaper node.
 */
struct nss_shaper_config_bf_attach {
	uint32_t child_qos_tag;		/* Qos tag of the shaper node to add as child */
};

/*
 * struct nss_shaper_config_bf_detach
 *	Detaches the child node with qos_tag 'child_qos_tag' from bf shaper node.
 */
struct nss_shaper_config_bf_detach {
	uint32_t child_qos_tag;		/* Qos tag of the shaper node to add as child */
};

/*
 * struct nss_shaper_config_bf_group_attach
 *	Attaches shaper node with the specified qos_tag to bf group shaper.
 */
struct nss_shaper_config_bf_group_attach {
	uint32_t child_qos_tag;		/* Qos tag of shaper node to add as child */
};

/*
 * struct nss_shaper_config_bf_group_param
 *	Configures bf group shaper node with the parameters mentioned in the structure.
 */
struct nss_shaper_config_bf_group_param {
	uint32_t quantum;				/* Smallest increment value for the DRRs */
	struct nss_shaper_config_rate_param lap;	/* Config structure for rate control algorithm */
};

/*
 * struct nss_shaper_config_fifo_limit_set
 *	Drop modes for fifo shaper in the NSS
 */
enum nss_shaper_config_fifo_drop_modes {
	NSS_SHAPER_FIFO_DROP_MODE_HEAD = 0,
	NSS_SHAPER_FIFO_DROP_MODE_TAIL,
	NSS_SHAPER_FIFO_DROP_MODES,
};
typedef enum nss_shaper_config_fifo_drop_modes nss_shaper_config_fifo_drop_mode_t;

/*
 * struct nss_shaper_config_fifo_param
 *	Configures fifo with the limit and drop mentioned in this structure
 */
struct nss_shaper_config_fifo_param {
	uint32_t limit;					/* Queue limit in packets */
	nss_shaper_config_fifo_drop_mode_t drop_mode;	/* FIFO drop mode when queue is full */
};

/*
 * enum nss_shaper_config_wred_weight_modes
 *	Weight modes supported
 */
enum nss_shaper_config_wred_weight_modes {
	NSS_SHAPER_WRED_WEIGHT_MODE_DSCP = 0,	/* Weight mode is DSCP */
	NSS_SHAPER_WRED_WEIGHT_MODES,
};
typedef enum nss_shaper_config_wred_weight_modes nss_shaper_config_wred_weight_mode_t;

/*
 * nss_shaper_red_alg_param
 *	RED algorithm parameters
 */
struct nss_shaper_red_alg_param {
	uint32_t min;			/* qlen_avg min */
	uint32_t max;			/* qlen_avg max */
	uint32_t probability;		/* Drop probability at qlen_avg = max */
	uint32_t exp_weight_factor;	/* exp_weight_factor to calculate qlen_avg */
};

/*
 * struct nss_shaper_config_wred_param
 *      Configures wred with the limit and drop mentioned in this structure
 */
struct nss_shaper_config_wred_param {
	uint32_t limit;						/* Queue limit */
	nss_shaper_config_wred_weight_mode_t weight_mode;	/* Weight mode */
	uint32_t traffic_classes;				/* How many traffic classes: DPs */
	uint32_t def_traffic_class;				/* Default traffic if no match: def_DP */
	uint32_t traffic_id;					/* Traffic ID to configure: DP */
	uint32_t weight_mode_value;				/* Weight mode value */
	struct nss_shaper_red_alg_param rap;			/* RED alg paramter */
	uint8_t ecn;						/* Mark ECN bit or drop packet */
};

/*
 * struct nss_shaper_config_wrr_attach
 *	Attaches shaper node with qos_tag to wrr shaper node.
 */
struct nss_shaper_config_wrr_attach {
	uint32_t child_qos_tag;		/* Qos tag of the shaper node to add as child */
};

/*
 * struct nss_shaper_config_wrr_detach
 *	Detaches the child node with qos_tag 'child_qos_tag' from wrr shaper node.
 */
struct nss_shaper_config_wrr_detach {
	uint32_t child_qos_tag;		/* Qos tag of the shaper node to add as child */
};

/*
 * struct nss_shaper_config_wrr_group_attach
 *	Attaches shaper node with the specified qos_tag to wrr group shaper.
 */
struct nss_shaper_config_wrr_group_attach {
	uint32_t child_qos_tag;		/* Qos tag of shaper node to add as child */
};

/*
 * Modes of wrr operation
 */
enum nss_shaper_wrr_operation_modes {
	NSS_SHAPER_WRR_MODE_ROUND_ROBIN = 0,
	NSS_SHAPER_WRR_MODE_FAIR_QUEUEING = 1,
	NSS_SHAPER_WRR_MODE_TYPE_MAX,
};

/*
 * struct nss_shaper_config_wrr_param
 *	Configures wrr shaper to operate in the mode specified.
 */
struct nss_shaper_config_wrr_param {
	uint32_t operation_mode;	/* Mode in which to operate in */
};

/*
 * struct nss_shaper_config_wrr_group_param
 *	Configures wrr group shaper node with specified quantum value.
 */
struct nss_shaper_config_wrr_group_param {
	uint32_t quantum;	/* Smallest increment value for the DRRs */
};

/*
 * struct nss_shaper_config_htb_attach
 *	Attaches shaper node with qos_tag to htb shaper node.
 */
struct nss_shaper_config_htb_attach {
	uint32_t child_qos_tag;		/* Qos tag of the shaper node to add as child */
};

/*
 * struct nss_shaper_config_htb_group_attach
 *	Attaches shaper node with the specified qos_tag to htb group shaper.
 */
struct nss_shaper_config_htb_group_attach {
	uint32_t child_qos_tag;		/* Qos tag of shaper node to add as child */
};

/*
 * struct nss_shaper_config_htb_group_detach
 *	Detaches shaper node with the specified qos_tag to htb group shaper.
 */
struct nss_shaper_config_htb_group_detach {
	uint32_t child_qos_tag;		/* Qos tag of shaper node to detach from child list */
};

/*
 * struct nss_shaper_config_htb_group_param
 *	Configures htb group shaper node with the parameters mentioned in the structure.
 */
struct nss_shaper_config_htb_group_param {
	uint32_t quantum;				/* Smallest increment value for the DRRs */
	uint32_t priority;				/* Value of priority for this group */
	uint32_t overhead;				/* Overhead in bytes to be added per packet */
	struct nss_shaper_config_rate_param rate_police;/* Config structure for police rate */
	struct nss_shaper_config_rate_param rate_ceil;	/* Config structure for ceil rate */
};
/*
 * struct nss_shaper_node_config
 *	Configurartion messages for all types of shaper nodes
 */
struct nss_shaper_node_config {
	uint32_t qos_tag;		/* Identifier of the shaper node to which the config is targetted */

	union {
		struct nss_shaper_config_prio_attach prio_attach;
		struct nss_shaper_config_prio_detach prio_detach;

		struct nss_shaper_config_codel_param codel_param;

		struct nss_shaper_config_tbl_attach tbl_attach;
		struct nss_shaper_config_tbl_param tbl_param;

		struct nss_shaper_config_bf_attach bf_attach;
		struct nss_shaper_config_bf_detach bf_detach;
		struct nss_shaper_config_bf_group_attach bf_group_attach;
		struct nss_shaper_config_bf_group_param bf_group_param;

		struct nss_shaper_config_fifo_param fifo_param;

		struct nss_shaper_config_wrr_attach wrr_attach;
		struct nss_shaper_config_wrr_detach wrr_detach;
		struct nss_shaper_config_wrr_param wrr_param;
		struct nss_shaper_config_wrr_group_attach wrr_group_attach;
		struct nss_shaper_config_wrr_group_param wrr_group_param;

		struct nss_shaper_config_htb_attach htb_attach;
		struct nss_shaper_config_htb_group_attach htb_group_attach;
		struct nss_shaper_config_htb_group_detach htb_group_detach;
		struct nss_shaper_config_htb_group_param htb_group_param;
		struct nss_shaper_config_wred_param wred_param;
	} snc;
};

/*
 * struct nss_shaper_node_basic_statistics_delta
 *	Statistics that are sent as deltas
 */
struct nss_shaper_node_basic_statistics_delta {
	uint32_t enqueued_bytes;			/* Bytes enqueued successfully */
	uint32_t enqueued_packets;			/* Packets enqueued successfully */
	uint32_t enqueued_bytes_dropped;		/* Bytes dropped during an enqueue operation due to node limits */
	uint32_t enqueued_packets_dropped;		/* Packets dropped during an enqueue operation due to node limits */
	uint32_t dequeued_bytes;			/* Bytes dequeued successfully from a shaper node */
	uint32_t dequeued_packets;			/* Packets dequeued successfully from a shaper node */
	uint32_t dequeued_bytes_dropped;		/* Bytes dropped by this node during dequeue (some nodes drop packets during dequeue rather than enqueue) */
	uint32_t dequeued_packets_dropped;		/* Packets dropped by this node during dequeue (some nodes drop packets during dequeue rather than enqueue) */
	uint32_t queue_overrun;				/* Number of times any queue limit has been overrun / perhaps leading to a drop of packet(s) */
};

/*
 * struct nss_shaper_shaper_node_basic_stats_get
 *	Obtain basic stats for a shaper node
 */
struct nss_shaper_shaper_node_basic_stats_get {

	/*
	 * Request
	 */
	uint32_t qos_tag;		/* The qos tag of the node from which to obtain basic stats */

	/*
	 * Response
	 */
	uint32_t qlen_bytes;				/* Total size of packets waiting in queue */
	uint32_t qlen_packets;				/* Number of packets waiting in queue */
	uint32_t packet_latency_peak_msec_dequeued;	/* Maximum milliseconds a packet was in this shaper node before being dequeued */
	uint32_t packet_latency_minimum_msec_dequeued;	/* Minimum milliseconds a packet was in this shaper node before being dequeued */
	uint32_t packet_latency_peak_msec_dropped;	/* Maximum milliseconds a packet was in this shaper node before being dropped */
	uint32_t packet_latency_minimum_msec_dropped;	/* Minimum milliseconds a packet was in this shaper node before being dropped */
	struct nss_shaper_node_basic_statistics_delta delta;
							/* Stastics that are sent as deltas */
};

/*
 * struct nss_shaper_configure
 *	Shaper configuration message
 */
struct nss_shaper_configure {
	nss_shaper_config_type_t request_type;		/* Request type */
	nss_shaper_response_type_t response_type;	/* Response type */
	union {
		struct nss_shaper_config_alloc_shaper_node alloc_shaper_node;
		struct nss_shaper_config_free_shaper_node free_shaper_node;
		struct nss_shaper_config_set_default_node set_default_node;
		struct nss_shaper_config_set_root_node set_root_node;
		struct nss_shaper_node_config shaper_node_config;
		struct nss_shaper_shaper_node_basic_stats_get shaper_node_basic_stats_get;
	} msg;
};

typedef void (*nss_shaper_bounced_callback_t)(void *app_data, struct sk_buff *skb);	/* Registrant callback to receive shaper bounced packets */

/**
 * @brief Register for basic shaping operations
 *
 * @return void* NSS context
 */
extern void *nss_shaper_register_shaping(void);

/**
 * @brief Unregister for basic shaping operations
 * @param ctx NSS context
 */
extern void nss_shaper_unregister_shaping(void *ctx);

/**
 * @brief Register to received shaper bounced packets for (interface bounce)
 * @param if_num Interface to be registered on
 * @param cb Callback invoked when the NSS returns a sk_buff after shaping
 * @param app_data Given to the callback along with the sk_buff to provide context to the registrant (state)
 * @param owner Pass THIS_MODULE for this parameter - your module is held until you unregister
 * @return void * NSS context or NULL on failure
 */
extern void *nss_shaper_register_shaper_bounce_interface(uint32_t if_num, nss_shaper_bounced_callback_t cb, void *app_data, struct module *owner);

/**
 * @brief Unregister for interface shaper bouncing
 * @param if_num Interface to be unregistered
 */
extern void nss_shaper_unregister_shaper_bounce_interface(uint32_t if_num);

/**
 * @brief Register to received shaper bounced packets for (bridge bounce)
 * @param if_num Interface to be registered on
 * @param cb Callback invoked when the NSS returns a sk_buff after shaping
 * @param app_data Given to the callback along with the sk_buff to provide context to the registrant (state)
 * @param owner Pass THIS_MODULE for this parameter - your module is held until you unregister
 * @return void * NSS context or NULL on failure
 */
extern void *nss_shaper_register_shaper_bounce_bridge(uint32_t if_num, nss_shaper_bounced_callback_t cb, void *app_data, struct module *owner);

/**
 * @brief Unregister for bridge shaper bouncing
 * @param if_num Interface to be unregistered
 */
extern void nss_shaper_unregister_shaper_bounce_bridge (uint32_t if_num);

/**
 * @brief Issue a packet for shaping via a bounce operation
 * @param ctx NSS context you were given when you registered for shaper bouncing
 * @param if_num Interface to be bounced to
 * @param skb The packet
 * @return nss_tx_status_t Succes or failure to issue packet to NSS
 */
extern nss_tx_status_t nss_shaper_bounce_interface_packet(void *ctx, uint32_t if_num, struct sk_buff *skb);

/**
 * @brief Issue a packet for shaping via a bounce operation
 * @param ctx NSS context you were given when you registered for shaper bouncing
 * @param if_num Interface to be bounced to
 * @param skb The packet
 * @return nss_tx_status_t Succes or failure to issue packet to NSS
 */
extern nss_tx_status_t nss_shaper_bounce_bridge_packet(void *ctx, uint32_t if_num, struct sk_buff *skb);

/**
 * @brief Send a shaping configuration message
 * @param ctx NSS context
 * @param config The config message
 *
 * @return nss_tx_status_t Indication if the configuration message was issued.  This does not mean that the configuration message was successfully processed, that will be determined by the response issued to your given callback function as specified in the config structure.
 */
nss_tx_status_t nss_shaper_config_send(void *ctx, struct nss_shaper_configure *config);

#endif
