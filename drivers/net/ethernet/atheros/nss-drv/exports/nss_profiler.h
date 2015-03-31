/*
 **************************************************************************
 * Copyright (c) 2014,2015, The Linux Foundation. All rights reserved.
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

#ifndef __NSS_PROFILER_H
#define __NSS_PROFILER_H

/**
 * nss_profiler.h
 *	NSS Profiler APIs
 */

/**
 * values chosen so all counter values fit in a single 1400 byte UDP packet
 */
#define PROFILE_COUNTER_NAME_LENGTH 20
#define PROFILE_MAX_APP_COUNTERS 24

/**
 * counter holds statistics
 */
struct nss_profile_counter {
	char name[PROFILE_COUNTER_NAME_LENGTH];	/**< counter name */
	uint32_t value;				/**< current value */
};

/**
 * DO not alter this enum; adding more is Ok
 */
enum nss_profiler_message_types {
	NSS_PROFILER_CHANGE_SIMPLING_RATE_MSG,	/**< H2N: iask to do rate change */
	NSS_PROFILER_START_MSG,		/**< H2N: start NSS profiler */
	NSS_PROFILER_STOP_MSG,		/**< H2N: stop NSS profiler */
	NSS_PROFILER_FLOWCTRL_MSG,	/**< H2N: do flow contrl on sampling */
	NSS_PROFILER_DEBUG_RD_MSG,	/**< H2N: debug output */
	NSS_PROFILER_DEBUG_WR_MSG,	/**< H2N: debug input */
	NSS_PROFILER_DEBUG_REPLY_MSG,	/**< N2H: debug response */
	NSS_PROFILER_REPLY_MSG,		/**< check response */
	NSS_PROFILER_FIXED_INFO_MSG,	/**< N2H: constant data */
	NSS_PROFILER_COUNTERS_MSG,	/**< N2H: counters information */
	NSS_PROFILER_SAMPLES_MSG,	/**< N2H: main sample data */
	NSS_PROFILER_MAX_MSG_TYPES,	/**< end mark */
};

/**
 * error type returned from NSS
 */
enum nss_profile_errors {
	PROFILE_ERROR_NO_PROF_INIT = 1,
	PROFILE_ERROR_EMEM,
	PROFILE_ERROR_BAD_PKT,
	PROFILE_ERROR_UNKNOWN_CMD,
};

/**
 * NSS profiler request
 */
struct nss_profiler_cmd_param {	/**< use for per session command -- START/STOP/FLOWCTRL/RATE */
	uint32_t hd_magic;	/**< common ovarlay in all headers */
	uint32_t num_counters;	/**< how many registered performance (app) counters -- may change */
	uint32_t ocm_size;
	uint32_t sram_start;

	uint32_t rate;		/**< sampling rate */
	uint32_t cpu_id;	/**< chip_id register */
	uint32_t cpu_freq;	/**< chip clock */
	uint32_t ddr_freq;	/**< DDR MEM speed */
	struct nss_profile_counter counters[PROFILE_MAX_APP_COUNTERS];
};

/**
 * Message DATA structure to send/receive proflier messages
 */
struct nss_profiler_data_msg {
	uint32_t hd_magic;	/**< Magic Header -- for verification */
	uint32_t msg_data[1];	/**< Message: private data -- variable length */
};

/**
 * structure to send/receive proflier debug messages
 */
struct nss_profiler_debug_msg {
	uint32_t hd_magic;	/**< Magic Header -- for verification */
	uint32_t debug_data[256];	/**< Message: fixed length */
};

/**
 * Message control structure to send/receive proflier messages
 */
struct nss_profiler_msg {
	struct nss_cmn_msg cm;	/**< Message Header -- N2H control */
	union {			/**< length is described in cm */
		struct nss_profiler_cmd_param pcmdp;	/**< command parameters */
		struct nss_profiler_debug_msg pdm;	/**< debug pkt */
		struct nss_profiler_data_msg msg;	/**< sampling data */
	} payload;			/**< Message data */
};

/**
 * Callback to receive profiler messages
 *
 * @note Memory pointed by buf (ncm) is owned by caller (i.e. NSS driver)
 */
typedef void (*nss_profiler_callback_t)(void *ctx, struct nss_profiler_msg *npm);

/**
 * @brief Register to send/receive profiler messages
 *
 * @param profiler_callback Profiler callback
 * @param core_id NSS core id
 * @param ctx Profiler context
 *
 * @return void* NSS context
 *
 * @note Caller must provide valid core_id that is being profiled. This function must be called once for each core.
 *	Context (ctx) will be provided back to caller in the registered callback function
 */
extern void *nss_profiler_notify_register(nss_core_id_t core_id, nss_profiler_callback_t profiler_callback, void *ctx);

/**
 * @brief Unregister profiler interface
 *
 * @param core_id NSS core id
 *
 */
extern void nss_profiler_notify_unregister(nss_core_id_t core_id);

/**
 * @brief Send profiler command to NSS
 *
 * @param nss_ctx NSS context
 * @param buf Buffer to send to NSS
 * @param len Length of buffer
 *
 * @return nss_tx_status_t Tx status
 *
 * @note Valid context must be provided (for the right core).
 *	This context was returned during registration.
 */
extern nss_tx_status_t nss_profiler_if_tx_buf(void *nss_ctx, void *buf, uint32_t len, void *cb);

/**
 * @brief Handling NSS less changed control information change
 *
 * @param arg application data
 * @param npm NSS profiler message
 */
extern void profile_handle_constant_info(void *arg, struct nss_profiler_msg *npm);

/**
 * @brief register a Linux counter for any variableS
 *
 * @param counter	a variable address (pointer)
 * @param name		variable name (meaningful for read, and 23 bytes or less)
 */
extern int profile_register_performance_counter(volatile unsigned int *counter, char *name);


/*
 * @brief Initialize the profiler specific message
 *
 * @return void
 */
extern void nss_profiler_msg_init(struct nss_profiler_msg *npm, uint16_t if_num, uint32_t type, uint32_t len,
					nss_profiler_callback_t cb, void *app_data);

#endif
