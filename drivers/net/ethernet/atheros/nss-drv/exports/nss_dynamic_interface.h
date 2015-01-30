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

 /*
  * nss_dynamic_interface.h
  * 	dynamic interface definitions.
  */

#ifndef __NSS_DYNAMIC_INTERFACE_H
#define __NSS_DYNAMIC_INTERFACE_H

/**
 * Dynamic Interface types
 */
enum nss_dynamic_interface_type {
	NSS_DYNAMIC_INTERFACE_TYPE_NONE = 0,
	NSS_DYNAMIC_INTERFACE_TYPE_GRE_REDIR = 1,	/* GRE_REDIR Interface type */
	NSS_DYNAMIC_INTERFACE_TYPE_CAPWAP = 2,		/* CAPWAP Interface type */
	NSS_DYNAMIC_INTERFACE_TYPE_TUN6RD = 3,		/* TUN6RD Interface type */
	NSS_DYNAMIC_INTERFACE_TYPE_802_3_REDIR = 4,	/* 802.3 redirect Interface type */
	NSS_DYNAMIC_INTERFACE_TYPE_MAX
};

/**
 * @brief allocate node for dynamic interface on NSS
 *
 * @param type nss dynamic interface type
 *
 * @return interface number for dynamic interface created on NSS or -1 in case of failure
 */
extern int nss_dynamic_interface_alloc_node(enum nss_dynamic_interface_type type);

/**
 * @brief Deallocate node created for dynamic interface on NSS
 *
 * @param if_num interface number of dynamic interface
 * @param type nss dynamic interface type
 *
 * @return nss_tx_status_t Tx status
 */
extern nss_tx_status_t nss_dynamic_interface_dealloc_node(int if_num, enum nss_dynamic_interface_type type);

/**
 * @brief The inferface number belong to the dynamic interface
 *
 * @param if_num interface number of dynamic interface
 *
 * @return bool true or false
 */
extern bool nss_is_dynamic_interface(int if_num);

/**
 * @brief Returns the type of dynamic interface
 *
 * @param if_num interface number of dynamic interface
 *
 * @return type nss dynamic interface type
 */
extern enum nss_dynamic_interface_type nss_dynamic_interface_get_type(int if_num);

#endif /* __NSS_DYNAMIC_INTERFACE_H*/
