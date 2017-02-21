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

#if 0
	/* ============================================================
	*  include files
	* ============================================================ */
#endif

#include "mp_precomp.h"
#include "../phydm_precomp.h"

#if (RTL8822B_SUPPORT == 1)


void
phydm_dynamic_switch_htstf_mumimo_8822b(
	struct PHY_DM_STRUCT		*p_dm_odm
)
{
	/*if rssi > 40dBm, enable HT-STF gain controller, otherwise, if rssi < 40dBm, disable the controller*/
	/*add by Chun-Hung Ho 20160711 */
	if (p_dm_odm->rssi_min >= 40)
		odm_set_bb_reg(p_dm_odm, 0x8d8, BIT(17), 0x1);
	else if (p_dm_odm->rssi_min < 35)
		odm_set_bb_reg(p_dm_odm, 0x8d8, BIT(17), 0x0);

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_COMMON, ODM_DBG_LOUD, ("%s, rssi_min = %d\n", __func__, p_dm_odm->rssi_min));
}

void
phydm_hwsetting_8822b(
	struct PHY_DM_STRUCT		*p_dm_odm
)
{
	phydm_dynamic_switch_htstf_mumimo_8822b(p_dm_odm);
}

#endif	/* RTL8822B_SUPPORT == 1 */
