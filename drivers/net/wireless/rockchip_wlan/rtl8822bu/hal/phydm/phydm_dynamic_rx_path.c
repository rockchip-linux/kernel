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

/* ************************************************************
 * include files
 * ************************************************************ */
#include "mp_precomp.h"
#include "phydm_precomp.h"

#if (CONFIG_DYNAMIC_RX_PATH == 1)

void
phydm_process_phy_status_for_dynamic_rx_path(
	void			*p_dm_void,
	void			*p_phy_info_void,
	void			*p_pkt_info_void
)
{
	struct PHY_DM_STRUCT				*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _odm_phy_status_info_		*p_phy_info = (struct _odm_phy_status_info_ *)p_phy_info_void;
	struct _odm_per_pkt_info_		*p_pktinfo = (struct _odm_per_pkt_info_ *)p_pkt_info_void;
	struct _DYNAMIC_RX_PATH_					*p_dm_drp_table	= &(p_dm_odm->dm_drp_table);
	/*u8					is_cck_rate=0;*/



}


void
phydm_dynamic_rx_path(
	void			*p_dm_void
)
{
	struct PHY_DM_STRUCT				*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _DYNAMIC_RX_PATH_					*p_dm_drp_table	= &(p_dm_odm->dm_drp_table);

	if (!(p_dm_odm->support_ability & ODM_BB_DYNAMIC_RX_PATH)) {
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_DYNAMIC_RX_PATH, ODM_DBG_LOUD, ("[Return Init]   Not Support Dynamic RX PAth\n"));
		return;
	}



	phydm_config_ofdm_rx_path(p_dm_odm, PHYDM_AB);
}


void
phydm_dynamic_rx_path_init(
	void			*p_dm_void
)
{
	struct PHY_DM_STRUCT				*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _DYNAMIC_RX_PATH_					*p_dm_drp_table	= &(p_dm_odm->dm_drp_table);

	if (!(p_dm_odm->support_ability & ODM_BB_DYNAMIC_RX_PATH)) {
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_DYNAMIC_RX_PATH, ODM_DBG_LOUD, ("[Return]   Not Support Dynamic RX PAth\n"));
		return;
	}



}


#endif
