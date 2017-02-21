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

/*Image2HeaderVersion: 3.1*/
#if (RTL8822B_SUPPORT == 1)
#ifndef __INC_MP_BB_HW_IMG_8822B_H
#define __INC_MP_BB_HW_IMG_8822B_H


/******************************************************************************
*                           agc_tab.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_agc_tab(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_agc_tab(void);

/******************************************************************************
*                           phy_reg.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_phy_reg(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_phy_reg(void);

/******************************************************************************
*                           phy_reg_pg.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_phy_reg_pg(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_phy_reg_pg(void);

#endif
#endif /* end of HWIMG_SUPPORT*/

