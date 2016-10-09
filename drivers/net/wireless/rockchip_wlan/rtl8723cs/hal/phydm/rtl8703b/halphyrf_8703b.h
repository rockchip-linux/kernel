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

#ifndef __HAL_PHY_RF_8703B_H__
#define __HAL_PHY_RF_8703B_H__

/*--------------------------Define Parameters-------------------------------*/
#define	index_mapping_NUM_8703B		15
#define AVG_THERMAL_NUM_8703B		4
#define RF_T_METER_8703B		0x42

#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN))
#include "../halphyrf_win.h"
#elif (DM_ODM_SUPPORT_TYPE & (ODM_CE))
#include "../halphyrf_ce.h"
#endif

void ConfigureTxpowerTrack_8703B(
	PTXPWRTRACK_CFG	pConfig
	);

void DoIQK_8703B(
	PVOID		pDM_VOID,
	u1Byte 		DeltaThermalIndex,
	u1Byte		ThermalValue,	
	u1Byte 		Threshold
	);

VOID
ODM_TxPwrTrackSetPwr_8703B(
	IN	PVOID		pDM_VOID,
	PWRTRACK_METHOD 	Method,
	u1Byte 				RFPath,
	u1Byte 				ChannelMappedIndex
	);

//1 7.	IQK

void	
PHY_IQCalibrate_8703B(	
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN PADAPTER	Adapter,
#endif
	IN	BOOLEAN 	bReCovery);

BOOLEAN
ODM_SetIQCbyRFpath_8703B(
	IN PDM_ODM_T		pDM_Odm
	);

//
// LC calibrate
//
void	
PHY_LCCalibrate_8703B(
	PVOID		pDM_VOID
);

//
// AP calibrate
//
void	
PHY_APCalibrate_8703B(		
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
							IN 	s1Byte		delta);
void	
PHY_DigitalPredistortion_8703B(		IN	PADAPTER	pAdapter);


VOID
_PHY_SaveADDARegisters_8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		ADDAReg,
	IN	pu4Byte		ADDABackup,
	IN	u4Byte		RegisterNum
	);

VOID
_PHY_PathADDAOn_8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		ADDAReg,
	IN	BOOLEAN		isPathAOn,
	IN	BOOLEAN		is2T
	);

VOID
_PHY_MACSettingCalibration_8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		MACReg,
	IN	pu4Byte		MACBackup	
	);
							
#endif	// #ifndef __HAL_PHY_RF_8188E_H__								

