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
#ifndef __INC_MP_RF_HW_IMG_8822B_H
#define __INC_MP_RF_HW_IMG_8822B_H


/******************************************************************************
*                           radioa.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_radioa(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_radioa(void);

/******************************************************************************
*                           radiob.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_radiob(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_radiob(void);

/******************************************************************************
*                           txpowertrack.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_txpowertrack(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_txpowertrack(void);

/******************************************************************************
*                           txpowertrack_type0.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_txpowertrack_type0(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_txpowertrack_type0(void);

/******************************************************************************
*                           txpowertrack_type1.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_txpowertrack_type1(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_txpowertrack_type1(void);

/******************************************************************************
*                           txpowertrack_type2.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_txpowertrack_type2(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_txpowertrack_type2(void);

/******************************************************************************
*                           txpowertrack_type3_type5.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_txpowertrack_type3_type5(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_txpowertrack_type3_type5(void);

/******************************************************************************
*                           txpowertrack_type4.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_txpowertrack_type4(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_txpowertrack_type4(void);

/******************************************************************************
*                           txpowertrack_type6.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_txpowertrack_type6(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_txpowertrack_type6(void);

/******************************************************************************
*                           txpowertrack_type7.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_txpowertrack_type7(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_txpowertrack_type7(void);

/******************************************************************************
*                           txpowertrack_type8.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_txpowertrack_type8(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_txpowertrack_type8(void);

/******************************************************************************
*                           txpowertrack_type9.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_txpowertrack_type9(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_txpowertrack_type9(void);

/******************************************************************************
*                           txpwr_lmt.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_txpwr_lmt(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_txpwr_lmt(void);

/******************************************************************************
*                           txpwr_lmt_type5.TXT
******************************************************************************/

void
odm_read_and_config_mp_8822b_txpwr_lmt_type5(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm_odm
);
u32	odm_get_version_mp_8822b_txpwr_lmt_type5(void);

#endif
#endif /* end of HWIMG_SUPPORT*/

