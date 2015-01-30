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
 * nss_data_plane
 *	Data plane used for communication between qca-nss-drv & qca-nss-gmac
 */

#ifndef __NSS_DATA_PLANE_H
#define __NSS_DATA_PLANE_H

#include <nss_api_if.h>

/*
 * nss_data_plane_param
 *	Holds the information that is going to pass to gmac as a cookie
 */
struct nss_data_plane_param {
	int if_num;				/* gmac interface number */
	struct net_device *dev;			/* net_device instance of this gmac */
	struct nss_ctx_instance *nss_ctx;	/* which nss core */
	int notify_open;			/* This gmac interface has been opened or not */
	int enabled;				/* This gmac is enabled or not */
	uint32_t features;			/* skb types supported by this interface */
};

/*
 * nss_data_plane_set_enabled
 *	Mark this data plane enabled, so when nss_init complete, we can call register_to_nss_gmac
 */
void nss_data_plane_set_enabled(int if_num);

/*
 * nss_data_plane_register_to_nss_gmac()
 *	Called from nss_init, this keeps the data_plane_ops to be static
 */
bool nss_data_plane_register_to_nss_gmac(struct nss_ctx_instance *nss_ctx, int if_num);

/*
 * nss_data_plane_unregister_from_nss_gmac()
 *	Called from nss_remove to ask gmac to restore to slowpath data plane
 */
void nss_data_plane_unregister_from_nss_gmac(int if_num);
#endif
