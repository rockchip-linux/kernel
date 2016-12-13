/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/

#ifndef	__PHYDMDYMICRXPATH_H__
#define    __PHYDMDYMICRXPATH_H__

#define DYNAMIC_RX_PATH_VERSION	"1.0"  /*2016.07.15  Dino */

#if (CONFIG_DYNAMIC_RX_PATH == 1)


struct _DYNAMIC_RX_PATH_ {
	u8	rx_path;

};



void
phydm_process_phy_status_for_dynamic_rx_path(
	void			*p_dm_void,
	void			*p_phy_info_void,
	void			*p_pkt_info_void
);

void
phydm_dynamic_rx_path(
	void			*p_dm_void
);

void
phydm_dynamic_rx_path_init(
	void			*p_dm_void
);

#endif
#endif
