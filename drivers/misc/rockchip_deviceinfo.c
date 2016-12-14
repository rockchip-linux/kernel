/*
 * Copyright (C) 2015 ROCKCHIP, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>

#define ETH_MAC_OFFSET	36

static u8 eth_mac[6];

void rk_devinfo_get_eth_mac(u8 *mac)
{
	memcpy(mac, eth_mac, 6);
	pr_info("%s: %02x.%02x.%02x.%02x.%02x.%02x\n", __func__,
		eth_mac[0], eth_mac[1], eth_mac[2],
		eth_mac[3], eth_mac[4], eth_mac[5]);
}
EXPORT_SYMBOL_GPL(rk_devinfo_get_eth_mac);

static int __init rk_devinfo_init(void)
{
	u8 *vaddr;
	phys_addr_t *pbase, *psize;

	pbase = __symbol_get("devinfo_base");
	if (!pbase)
		return 0;

	psize = __symbol_get("devinfo_size");
	if (!psize) {
		__symbol_put("devinfo_base");
		return 0;
	}

	if (*psize == 0)
		goto out;

	vaddr = phys_to_virt(*pbase);
	/* copy eth mac addr */
	memcpy(eth_mac, vaddr + ETH_MAC_OFFSET, 6);

out:
	__symbol_put("devinfo_base");
	__symbol_put("devinfo_size");

	return 0;
}

module_init(rk_devinfo_init);
