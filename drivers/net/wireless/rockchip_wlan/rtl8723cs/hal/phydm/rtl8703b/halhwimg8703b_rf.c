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

/*Image2HeaderVersion: 2.18*/
#include "mp_precomp.h"
#include "../phydm_precomp.h"

#if (RTL8703B_SUPPORT == 1)
static BOOLEAN
CheckPositive(
	IN  PDM_ODM_T     pDM_Odm,
	IN  const u4Byte  Condition1,
	IN  const u4Byte  Condition2,
	IN	const u4Byte  Condition3,
	IN	const u4Byte  Condition4
)
{
	u1Byte    _BoardType = ((pDM_Odm->BoardType & BIT4) >> 4) << 0 | /* _GLNA*/
				((pDM_Odm->BoardType & BIT3) >> 3) << 1 | /* _GPA*/ 
				((pDM_Odm->BoardType & BIT7) >> 7) << 2 | /* _ALNA*/
				((pDM_Odm->BoardType & BIT6) >> 6) << 3 | /* _APA */
				((pDM_Odm->BoardType & BIT2) >> 2) << 4;  /* _BT*/  

	u4Byte	cond1   = Condition1, cond2 = Condition2, cond3 = Condition3, cond4 = Condition4;
	u4Byte    driver1 = pDM_Odm->CutVersion       << 24 | 
				(pDM_Odm->SupportInterface & 0xF0) << 16 | 
				pDM_Odm->SupportPlatform  << 16 | 
				pDM_Odm->PackageType      << 12 | 
				(pDM_Odm->SupportInterface & 0x0F) << 8  |
				_BoardType;

	u4Byte    driver2 = (pDM_Odm->TypeGLNA & 0xFF) <<  0 |  
				(pDM_Odm->TypeGPA & 0xFF)  <<  8 | 
				(pDM_Odm->TypeALNA & 0xFF) << 16 | 
				(pDM_Odm->TypeAPA & 0xFF)  << 24; 

u4Byte    driver3 = 0;

	u4Byte    driver4 = (pDM_Odm->TypeGLNA & 0xFF00) >>  8 |
				(pDM_Odm->TypeGPA & 0xFF00) |
				(pDM_Odm->TypeALNA & 0xFF00) << 8 |
				(pDM_Odm->TypeAPA & 0xFF00)  << 16;

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_TRACE, 
	("===> CheckPositive (cond1, cond2, cond3, cond4) = (0x%X 0x%X 0x%X 0x%X)\n", cond1, cond2, cond3, cond4));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_TRACE, 
	("===> CheckPositive (driver1, driver2, driver3, driver4) = (0x%X 0x%X 0x%X 0x%X)\n", driver1, driver2, driver3, driver4));

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_TRACE, 
	("	(Platform, Interface) = (0x%X, 0x%X)\n", pDM_Odm->SupportPlatform, pDM_Odm->SupportInterface));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_TRACE, 
	("	(Board, Package) = (0x%X, 0x%X)\n", pDM_Odm->BoardType, pDM_Odm->PackageType));


	/*============== Value Defined Check ===============*/
	/*QFN Type [15:12] and Cut Version [27:24] need to do value check*/
	
	if (((cond1 & 0x0000F000) != 0) && ((cond1 & 0x0000F000) != (driver1 & 0x0000F000)))
		return FALSE;
	if (((cond1 & 0x0F000000) != 0) && ((cond1 & 0x0F000000) != (driver1 & 0x0F000000)))
		return FALSE;

	/*=============== Bit Defined Check ================*/
	/* We don't care [31:28] */

	cond1   &= 0x00FF0FFF; 
	driver1 &= 0x00FF0FFF; 

	if ((cond1 & driver1) == cond1) {
		u4Byte bitMask = 0;

		if ((cond1 & 0x0F) == 0) /* BoardType is DONTCARE*/
			return TRUE;

		if ((cond1 & BIT0) != 0) /*GLNA*/
			bitMask |= 0x000000FF;
		if ((cond1 & BIT1) != 0) /*GPA*/
			bitMask |= 0x0000FF00;
		if ((cond1 & BIT2) != 0) /*ALNA*/
			bitMask |= 0x00FF0000;
		if ((cond1 & BIT3) != 0) /*APA*/
			bitMask |= 0xFF000000;

		if (((cond2 & bitMask) == (driver2 & bitMask)) && ((cond4 & bitMask) == (driver4 & bitMask)))  /* BoardType of each RF path is matched*/
			return TRUE;
		else
			return FALSE;
	} else
		return FALSE;
}
static BOOLEAN
CheckNegative(
	IN  PDM_ODM_T     pDM_Odm,
	IN  const u4Byte  Condition1,
	IN  const u4Byte  Condition2
)
{
	return TRUE;
}

/******************************************************************************
*                           RadioA.TXT
******************************************************************************/

u4Byte Array_MP_8703B_RadioA[] = { 
		0x018, 0x00008C01,
		0x0B5, 0x0008C050,
		0x0B1, 0x00054258,
		0x0B2, 0x00054C00,
		0x030, 0x00018000,
		0x031, 0x00000027,
		0x032, 0x000A7F07,
		0x030, 0x00020000,
		0x031, 0x00000027,
		0x032, 0x000E7D87,
		0x01C, 0x000F8635,
		0x0EF, 0x00080000,
		0x030, 0x00008000,
		0x031, 0x00000004,
		0x032, 0x00006105,
		0x0EF, 0x00000000,
		0x0EF, 0x00000400,
		0x041, 0x0000BD54,
		0x041, 0x00003DD4,
		0x041, 0x0000FDD4,
		0x0EF, 0x00000000,
		0x0DF, 0x00000600,
		0x050, 0x0000C6DB,
		0x051, 0x00004505,
		0x052, 0x0000E31D,
		0x053, 0x00040579,
		0x054, 0x00000000,
		0x055, 0x0008206E,
		0x056, 0x00040000,
		0x0EF, 0x00000100,
		0x034, 0x0000ADD7,
		0x034, 0x00009DD4,
		0x034, 0x00008DD1,
		0x034, 0x00007DCE,
		0x034, 0x00006DCB,
		0x034, 0x00005CCE,
		0x034, 0x000048CD,
		0x034, 0x000034CC,
		0x034, 0x0000244F,
		0x034, 0x0000144C,
		0x034, 0x0000004E,
		0x0EF, 0x00000000,
		0x0EF, 0x00002000,
		0x03B, 0x0003801F,
		0x03B, 0x00030002,
		0x03B, 0x00028001,
		0x03B, 0x00020000,
		0x03B, 0x00018003,
		0x03B, 0x00010002,
		0x03B, 0x00008001,
		0x03B, 0x00000000,
		0x0EF, 0x00000000,
		0x082, 0x000C0000,
		0x083, 0x000AF025,
		0x01E, 0x00000C08,

};

void
ODM_ReadAndConfig_MP_8703B_RadioA(
	IN   PDM_ODM_T  pDM_Odm
)
{
	u4Byte     i         = 0;
	u1Byte     cCond;
	BOOLEAN bMatched = TRUE, bSkipped = FALSE;
	u4Byte     ArrayLen    = sizeof(Array_MP_8703B_RadioA)/sizeof(u4Byte);
	pu4Byte    Array       = Array_MP_8703B_RadioA;
	
	u4Byte	v1 = 0, v2 = 0, pre_v1 = 0, pre_v2 = 0;

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("===> ODM_ReadAndConfig_MP_8703B_RadioA\n"));

	while ((i + 1) < ArrayLen) {
		v1 = Array[i];
		v2 = Array[i + 1];

		if (v1 & (BIT31 | BIT30)) {/*positive & negative condition*/
			if (v1 & BIT31) {/* positive condition*/
				cCond  = (u1Byte)((v1 & (BIT29|BIT28)) >> 28);
				if (cCond == COND_ENDIF) {/*end*/
					bMatched = TRUE;
					bSkipped = FALSE;
					ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("ENDIF\n"));
				} else if (cCond == COND_ELSE) { /*else*/
					bMatched = bSkipped?FALSE:TRUE;
					ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("ELSE\n"));
				} else {/*if , else if*/
					pre_v1 = v1;
					pre_v2 = v2;
					ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("IF or ELSE IF\n"));
				}
			} else if (v1 & BIT30) { /*negative condition*/
				if (bSkipped == FALSE) {
					if (CheckPositive(pDM_Odm, pre_v1, pre_v2, v1, v2)) {
						bMatched = TRUE;
						bSkipped = TRUE;
					} else {
						bMatched = FALSE;
						bSkipped = FALSE;
					}
				} else
					bMatched = FALSE;
			}
		} else {
			if (bMatched)
				odm_ConfigRF_RadioA_8703B(pDM_Odm, v1, v2);
		}
		i = i + 2;
	}
}

u4Byte
ODM_GetVersion_MP_8703B_RadioA(void)
{
	   return 17;
}

/******************************************************************************
*                           TxPowerTrack_SDIO.TXT
******************************************************************************/

#if DEV_BUS_TYPE == RT_SDIO_INTERFACE
u1Byte gDeltaSwingTableIdx_MP_5GB_N_TxPowerTrack_SDIO_8703B[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 2, 3, 3, 5, 5, 6, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 16, 17, 17, 17, 17, 18, 18, 18},
	{0, 1, 2, 3, 3, 5, 5, 6, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 18, 18, 18},
	{0, 1, 2, 3, 3, 5, 5, 6, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 18, 18, 18},
};
u1Byte gDeltaSwingTableIdx_MP_5GB_P_TxPowerTrack_SDIO_8703B[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15},
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15},
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15},
};
u1Byte gDeltaSwingTableIdx_MP_5GA_N_TxPowerTrack_SDIO_8703B[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 2, 3, 3, 5, 5, 6, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 16, 17, 17, 17, 17, 18, 18, 18},
	{0, 1, 2, 3, 3, 5, 5, 6, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 18, 18, 18},
	{0, 1, 2, 3, 3, 5, 5, 6, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 18, 18, 18},
};
u1Byte gDeltaSwingTableIdx_MP_5GA_P_TxPowerTrack_SDIO_8703B[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15},
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15},
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15},
};
u1Byte gDeltaSwingTableIdx_MP_2GB_N_TxPowerTrack_SDIO_8703B[]    = {0, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6, 7, 7, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11};
u1Byte gDeltaSwingTableIdx_MP_2GB_P_TxPowerTrack_SDIO_8703B[]    = {0, 1, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 7, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15};
u1Byte gDeltaSwingTableIdx_MP_2GA_N_TxPowerTrack_SDIO_8703B[]    = {0, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6, 7, 7, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11};
u1Byte gDeltaSwingTableIdx_MP_2GA_P_TxPowerTrack_SDIO_8703B[]    = {0, 1, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 7, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKB_N_TxPowerTrack_SDIO_8703B[] = {0, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6, 7, 7, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11};
u1Byte gDeltaSwingTableIdx_MP_2GCCKB_P_TxPowerTrack_SDIO_8703B[] = {0, 0, 1, 1, 2, 3, 3, 3, 4, 4, 4, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13};
u1Byte gDeltaSwingTableIdx_MP_2GCCKA_N_TxPowerTrack_SDIO_8703B[] = {0, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6, 7, 7, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11};
u1Byte gDeltaSwingTableIdx_MP_2GCCKA_P_TxPowerTrack_SDIO_8703B[] = {0, 0, 1, 1, 2, 3, 3, 3, 4, 4, 4, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13};
#endif

void
ODM_ReadAndConfig_MP_8703B_TxPowerTrack_SDIO(
	IN   PDM_ODM_T  pDM_Odm
)
{
#if DEV_BUS_TYPE == RT_SDIO_INTERFACE
	PODM_RF_CAL_T  pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("===> ODM_ReadAndConfig_MP_MP_8703B\n"));


	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GA_P, gDeltaSwingTableIdx_MP_2GA_P_TxPowerTrack_SDIO_8703B, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GA_N, gDeltaSwingTableIdx_MP_2GA_N_TxPowerTrack_SDIO_8703B, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GB_P, gDeltaSwingTableIdx_MP_2GB_P_TxPowerTrack_SDIO_8703B, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GB_N, gDeltaSwingTableIdx_MP_2GB_N_TxPowerTrack_SDIO_8703B, DELTA_SWINGIDX_SIZE);

	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_P, gDeltaSwingTableIdx_MP_2GCCKA_P_TxPowerTrack_SDIO_8703B, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_N, gDeltaSwingTableIdx_MP_2GCCKA_N_TxPowerTrack_SDIO_8703B, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_P, gDeltaSwingTableIdx_MP_2GCCKB_P_TxPowerTrack_SDIO_8703B, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_N, gDeltaSwingTableIdx_MP_2GCCKB_N_TxPowerTrack_SDIO_8703B, DELTA_SWINGIDX_SIZE);

	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GA_P, gDeltaSwingTableIdx_MP_5GA_P_TxPowerTrack_SDIO_8703B, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GA_N, gDeltaSwingTableIdx_MP_5GA_N_TxPowerTrack_SDIO_8703B, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GB_P, gDeltaSwingTableIdx_MP_5GB_P_TxPowerTrack_SDIO_8703B, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GB_N, gDeltaSwingTableIdx_MP_5GB_N_TxPowerTrack_SDIO_8703B, DELTA_SWINGIDX_SIZE*3);
#endif
}

/******************************************************************************
*                           TxPowerTrack_USB.TXT
******************************************************************************/

#if DEV_BUS_TYPE == RT_USB_INTERFACE
u1Byte gDeltaSwingTableIdx_MP_5GB_N_TxPowerTrack_USB_8703B[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 2, 3, 3, 5, 5, 6, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 16, 17, 17, 17, 17, 18, 18, 18},
	{0, 1, 2, 3, 3, 5, 5, 6, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 18, 18, 18},
	{0, 1, 2, 3, 3, 5, 5, 6, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 18, 18, 18},
};
u1Byte gDeltaSwingTableIdx_MP_5GB_P_TxPowerTrack_USB_8703B[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15},
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15},
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15},
};
u1Byte gDeltaSwingTableIdx_MP_5GA_N_TxPowerTrack_USB_8703B[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 2, 3, 3, 5, 5, 6, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 16, 17, 17, 17, 17, 18, 18, 18},
	{0, 1, 2, 3, 3, 5, 5, 6, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 18, 18, 18},
	{0, 1, 2, 3, 3, 5, 5, 6, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 18, 18, 18},
};
u1Byte gDeltaSwingTableIdx_MP_5GA_P_TxPowerTrack_USB_8703B[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15},
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15},
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15},
};
u1Byte gDeltaSwingTableIdx_MP_2GB_N_TxPowerTrack_USB_8703B[]    = {0, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6, 7, 7, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11};
u1Byte gDeltaSwingTableIdx_MP_2GB_P_TxPowerTrack_USB_8703B[]    = {0, 1, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 7, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15};
u1Byte gDeltaSwingTableIdx_MP_2GA_N_TxPowerTrack_USB_8703B[]    = {0, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6, 7, 7, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11};
u1Byte gDeltaSwingTableIdx_MP_2GA_P_TxPowerTrack_USB_8703B[]    = {0, 1, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 7, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKB_N_TxPowerTrack_USB_8703B[] = {0, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6, 7, 7, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11};
u1Byte gDeltaSwingTableIdx_MP_2GCCKB_P_TxPowerTrack_USB_8703B[] = {0, 0, 1, 1, 2, 3, 3, 3, 4, 4, 4, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13};
u1Byte gDeltaSwingTableIdx_MP_2GCCKA_N_TxPowerTrack_USB_8703B[] = {0, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6, 7, 7, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11};
u1Byte gDeltaSwingTableIdx_MP_2GCCKA_P_TxPowerTrack_USB_8703B[] = {0, 0, 1, 1, 2, 3, 3, 3, 4, 4, 4, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13};
#endif

void
ODM_ReadAndConfig_MP_8703B_TxPowerTrack_USB(
	IN   PDM_ODM_T  pDM_Odm
)
{
#if DEV_BUS_TYPE == RT_USB_INTERFACE
	PODM_RF_CAL_T  pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("===> ODM_ReadAndConfig_MP_MP_8703B\n"));


	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GA_P, gDeltaSwingTableIdx_MP_2GA_P_TxPowerTrack_USB_8703B, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GA_N, gDeltaSwingTableIdx_MP_2GA_N_TxPowerTrack_USB_8703B, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GB_P, gDeltaSwingTableIdx_MP_2GB_P_TxPowerTrack_USB_8703B, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GB_N, gDeltaSwingTableIdx_MP_2GB_N_TxPowerTrack_USB_8703B, DELTA_SWINGIDX_SIZE);

	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_P, gDeltaSwingTableIdx_MP_2GCCKA_P_TxPowerTrack_USB_8703B, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_N, gDeltaSwingTableIdx_MP_2GCCKA_N_TxPowerTrack_USB_8703B, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_P, gDeltaSwingTableIdx_MP_2GCCKB_P_TxPowerTrack_USB_8703B, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_N, gDeltaSwingTableIdx_MP_2GCCKB_N_TxPowerTrack_USB_8703B, DELTA_SWINGIDX_SIZE);

	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GA_P, gDeltaSwingTableIdx_MP_5GA_P_TxPowerTrack_USB_8703B, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GA_N, gDeltaSwingTableIdx_MP_5GA_N_TxPowerTrack_USB_8703B, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GB_P, gDeltaSwingTableIdx_MP_5GB_P_TxPowerTrack_USB_8703B, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GB_N, gDeltaSwingTableIdx_MP_5GB_N_TxPowerTrack_USB_8703B, DELTA_SWINGIDX_SIZE*3);
#endif
}

/******************************************************************************
*                           TXPWR_LMT.TXT
******************************************************************************/

const char *Array_MP_8703B_TXPWR_LMT[] = { 
	"FCC", "2.4G", "20M", "CCK", "1T", "01", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "01", "26", 
	"MKK", "2.4G", "20M", "CCK", "1T", "01", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "02", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "02", "26", 
	"MKK", "2.4G", "20M", "CCK", "1T", "02", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "03", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "03", "26", 
	"MKK", "2.4G", "20M", "CCK", "1T", "03", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "04", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "04", "26", 
	"MKK", "2.4G", "20M", "CCK", "1T", "04", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "05", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "05", "26", 
	"MKK", "2.4G", "20M", "CCK", "1T", "05", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "06", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "06", "26", 
	"MKK", "2.4G", "20M", "CCK", "1T", "06", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "07", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "07", "26", 
	"MKK", "2.4G", "20M", "CCK", "1T", "07", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "08", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "08", "26", 
	"MKK", "2.4G", "20M", "CCK", "1T", "08", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "09", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "09", "26", 
	"MKK", "2.4G", "20M", "CCK", "1T", "09", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "10", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "10", "26", 
	"MKK", "2.4G", "20M", "CCK", "1T", "10", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "11", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "11", "26", 
	"MKK", "2.4G", "20M", "CCK", "1T", "11", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "12", "63", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "12", "26", 
	"MKK", "2.4G", "20M", "CCK", "1T", "12", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "13", "63", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "13", "26", 
	"MKK", "2.4G", "20M", "CCK", "1T", "13", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "14", "63", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "14", "63", 
	"MKK", "2.4G", "20M", "CCK", "1T", "14", "32",
	"FCC", "2.4G", "20M", "OFDM", "1T", "01", "28", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "01", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "01", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "02", "28", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "02", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "02", "32",
	"FCC", "2.4G", "20M", "OFDM", "1T", "03", "32", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "03", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "03", "32",
	"FCC", "2.4G", "20M", "OFDM", "1T", "04", "32", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "04", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "04", "32",
	"FCC", "2.4G", "20M", "OFDM", "1T", "05", "32", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "05", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "05", "32",
	"FCC", "2.4G", "20M", "OFDM", "1T", "06", "32", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "06", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "06", "32",
	"FCC", "2.4G", "20M", "OFDM", "1T", "07", "32", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "07", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "07", "32",
	"FCC", "2.4G", "20M", "OFDM", "1T", "08", "32", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "08", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "08", "32",
	"FCC", "2.4G", "20M", "OFDM", "1T", "09", "32", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "09", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "09", "32",
	"FCC", "2.4G", "20M", "OFDM", "1T", "10", "28", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "10", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "10", "32",
	"FCC", "2.4G", "20M", "OFDM", "1T", "11", "28", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "11", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "11", "32",
	"FCC", "2.4G", "20M", "OFDM", "1T", "12", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "12", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "12", "32",
	"FCC", "2.4G", "20M", "OFDM", "1T", "13", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "13", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "13", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "14", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "14", "63", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "14", "63",
	"FCC", "2.4G", "20M", "HT", "1T", "01", "26", 
	"ETSI", "2.4G", "20M", "HT", "1T", "01", "26", 
	"MKK", "2.4G", "20M", "HT", "1T", "01", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "02", "26", 
	"ETSI", "2.4G", "20M", "HT", "1T", "02", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "02", "32",
	"FCC", "2.4G", "20M", "HT", "1T", "03", "32", 
	"ETSI", "2.4G", "20M", "HT", "1T", "03", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "03", "32",
	"FCC", "2.4G", "20M", "HT", "1T", "04", "32", 
	"ETSI", "2.4G", "20M", "HT", "1T", "04", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "04", "32",
	"FCC", "2.4G", "20M", "HT", "1T", "05", "32", 
	"ETSI", "2.4G", "20M", "HT", "1T", "05", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "05", "32",
	"FCC", "2.4G", "20M", "HT", "1T", "06", "32", 
	"ETSI", "2.4G", "20M", "HT", "1T", "06", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "06", "32",
	"FCC", "2.4G", "20M", "HT", "1T", "07", "32", 
	"ETSI", "2.4G", "20M", "HT", "1T", "07", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "07", "32",
	"FCC", "2.4G", "20M", "HT", "1T", "08", "32", 
	"ETSI", "2.4G", "20M", "HT", "1T", "08", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "08", "32",
	"FCC", "2.4G", "20M", "HT", "1T", "09", "32", 
	"ETSI", "2.4G", "20M", "HT", "1T", "09", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "09", "32",
	"FCC", "2.4G", "20M", "HT", "1T", "10", "26", 
	"ETSI", "2.4G", "20M", "HT", "1T", "10", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "10", "32",
	"FCC", "2.4G", "20M", "HT", "1T", "11", "26", 
	"ETSI", "2.4G", "20M", "HT", "1T", "11", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "11", "32",
	"FCC", "2.4G", "20M", "HT", "1T", "12", "63", 
	"ETSI", "2.4G", "20M", "HT", "1T", "12", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "12", "32",
	"FCC", "2.4G", "20M", "HT", "1T", "13", "63", 
	"ETSI", "2.4G", "20M", "HT", "1T", "13", "26", 
	"MKK", "2.4G", "20M", "HT", "1T", "13", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "14", "63", 
	"ETSI", "2.4G", "20M", "HT", "1T", "14", "63", 
	"MKK", "2.4G", "20M", "HT", "1T", "14", "63",
	"FCC", "2.4G", "20M", "HT", "2T", "01", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "01", "30", 
	"MKK", "2.4G", "20M", "HT", "2T", "01", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "02", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "02", "30", 
	"MKK", "2.4G", "20M", "HT", "2T", "02", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "03", "30", 
	"ETSI", "2.4G", "20M", "HT", "2T", "03", "30", 
	"MKK", "2.4G", "20M", "HT", "2T", "03", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "04", "30", 
	"ETSI", "2.4G", "20M", "HT", "2T", "04", "30", 
	"MKK", "2.4G", "20M", "HT", "2T", "04", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "05", "30", 
	"ETSI", "2.4G", "20M", "HT", "2T", "05", "30", 
	"MKK", "2.4G", "20M", "HT", "2T", "05", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "06", "30", 
	"ETSI", "2.4G", "20M", "HT", "2T", "06", "30", 
	"MKK", "2.4G", "20M", "HT", "2T", "06", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "07", "30", 
	"ETSI", "2.4G", "20M", "HT", "2T", "07", "30", 
	"MKK", "2.4G", "20M", "HT", "2T", "07", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "08", "30", 
	"ETSI", "2.4G", "20M", "HT", "2T", "08", "30", 
	"MKK", "2.4G", "20M", "HT", "2T", "08", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "09", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "09", "30", 
	"MKK", "2.4G", "20M", "HT", "2T", "09", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "10", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "10", "30", 
	"MKK", "2.4G", "20M", "HT", "2T", "10", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "11", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "11", "30", 
	"MKK", "2.4G", "20M", "HT", "2T", "11", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "12", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "12", "30", 
	"MKK", "2.4G", "20M", "HT", "2T", "12", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "13", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "13", "30", 
	"MKK", "2.4G", "20M", "HT", "2T", "13", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "14", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "14", "63", 
	"MKK", "2.4G", "20M", "HT", "2T", "14", "63",
	"FCC", "2.4G", "40M", "HT", "1T", "01", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "01", "63", 
	"MKK", "2.4G", "40M", "HT", "1T", "01", "63",
	"FCC", "2.4G", "40M", "HT", "1T", "02", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "02", "63", 
	"MKK", "2.4G", "40M", "HT", "1T", "02", "63",
	"FCC", "2.4G", "40M", "HT", "1T", "03", "26", 
	"ETSI", "2.4G", "40M", "HT", "1T", "03", "26", 
	"MKK", "2.4G", "40M", "HT", "1T", "03", "26",
	"FCC", "2.4G", "40M", "HT", "1T", "04", "26", 
	"ETSI", "2.4G", "40M", "HT", "1T", "04", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "04", "26",
	"FCC", "2.4G", "40M", "HT", "1T", "05", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "05", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "05", "26",
	"FCC", "2.4G", "40M", "HT", "1T", "06", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "06", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "06", "26",
	"FCC", "2.4G", "40M", "HT", "1T", "07", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "07", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "07", "26",
	"FCC", "2.4G", "40M", "HT", "1T", "08", "26", 
	"ETSI", "2.4G", "40M", "HT", "1T", "08", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "08", "26",
	"FCC", "2.4G", "40M", "HT", "1T", "09", "26", 
	"ETSI", "2.4G", "40M", "HT", "1T", "09", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "09", "26",
	"FCC", "2.4G", "40M", "HT", "1T", "10", "26", 
	"ETSI", "2.4G", "40M", "HT", "1T", "10", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "10", "26",
	"FCC", "2.4G", "40M", "HT", "1T", "11", "26", 
	"ETSI", "2.4G", "40M", "HT", "1T", "11", "26", 
	"MKK", "2.4G", "40M", "HT", "1T", "11", "26",
	"FCC", "2.4G", "40M", "HT", "1T", "12", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "12", "26", 
	"MKK", "2.4G", "40M", "HT", "1T", "12", "26",
	"FCC", "2.4G", "40M", "HT", "1T", "13", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "13", "26", 
	"MKK", "2.4G", "40M", "HT", "1T", "13", "26",
	"FCC", "2.4G", "40M", "HT", "1T", "14", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "14", "63", 
	"MKK", "2.4G", "40M", "HT", "1T", "14", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "01", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "01", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "01", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "02", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "02", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "02", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "03", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "03", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "03", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "04", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "04", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "04", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "05", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "05", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "05", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "06", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "06", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "06", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "07", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "07", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "07", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "08", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "08", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "08", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "09", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "09", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "09", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "10", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "10", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "10", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "11", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "11", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "11", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "12", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "12", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "12", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "13", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "13", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "13", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "14", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "14", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "14", "63",
	"FCC", "5G", "20M", "OFDM", "1T", "36", "30", 
	"ETSI", "5G", "20M", "OFDM", "1T", "36", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "36", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "40", "30", 
	"ETSI", "5G", "20M", "OFDM", "1T", "40", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "40", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "44", "30", 
	"ETSI", "5G", "20M", "OFDM", "1T", "44", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "44", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "48", "30", 
	"ETSI", "5G", "20M", "OFDM", "1T", "48", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "48", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "52", "34", 
	"ETSI", "5G", "20M", "OFDM", "1T", "52", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "52", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "56", "34", 
	"ETSI", "5G", "20M", "OFDM", "1T", "56", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "56", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "60", "32", 
	"ETSI", "5G", "20M", "OFDM", "1T", "60", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "60", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "64", "28", 
	"ETSI", "5G", "20M", "OFDM", "1T", "64", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "64", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "100", "30", 
	"ETSI", "5G", "20M", "OFDM", "1T", "100", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "100", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "114", "30", 
	"ETSI", "5G", "20M", "OFDM", "1T", "114", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "114", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "108", "32", 
	"ETSI", "5G", "20M", "OFDM", "1T", "108", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "108", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "112", "34", 
	"ETSI", "5G", "20M", "OFDM", "1T", "112", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "112", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "116", "34", 
	"ETSI", "5G", "20M", "OFDM", "1T", "116", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "116", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "120", "34", 
	"ETSI", "5G", "20M", "OFDM", "1T", "120", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "120", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "124", "34", 
	"ETSI", "5G", "20M", "OFDM", "1T", "124", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "124", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "128", "32", 
	"ETSI", "5G", "20M", "OFDM", "1T", "128", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "128", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "132", "30", 
	"ETSI", "5G", "20M", "OFDM", "1T", "132", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "132", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "136", "30", 
	"ETSI", "5G", "20M", "OFDM", "1T", "136", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "136", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "140", "28", 
	"ETSI", "5G", "20M", "OFDM", "1T", "140", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "140", "32",
	"FCC", "5G", "20M", "OFDM", "1T", "149", "34", 
	"ETSI", "5G", "20M", "OFDM", "1T", "149", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "149", "63",
	"FCC", "5G", "20M", "OFDM", "1T", "153", "34", 
	"ETSI", "5G", "20M", "OFDM", "1T", "153", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "153", "63",
	"FCC", "5G", "20M", "OFDM", "1T", "157", "34", 
	"ETSI", "5G", "20M", "OFDM", "1T", "157", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "157", "63",
	"FCC", "5G", "20M", "OFDM", "1T", "161", "34", 
	"ETSI", "5G", "20M", "OFDM", "1T", "161", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "161", "63",
	"FCC", "5G", "20M", "OFDM", "1T", "165", "34", 
	"ETSI", "5G", "20M", "OFDM", "1T", "165", "32", 
	"MKK", "5G", "20M", "OFDM", "1T", "165", "63",
	"FCC", "5G", "20M", "HT", "1T", "36", "30", 
	"ETSI", "5G", "20M", "HT", "1T", "36", "32", 
	"MKK", "5G", "20M", "HT", "1T", "36", "32",
	"FCC", "5G", "20M", "HT", "1T", "40", "30", 
	"ETSI", "5G", "20M", "HT", "1T", "40", "32", 
	"MKK", "5G", "20M", "HT", "1T", "40", "32",
	"FCC", "5G", "20M", "HT", "1T", "44", "30", 
	"ETSI", "5G", "20M", "HT", "1T", "44", "32", 
	"MKK", "5G", "20M", "HT", "1T", "44", "32",
	"FCC", "5G", "20M", "HT", "1T", "48", "30", 
	"ETSI", "5G", "20M", "HT", "1T", "48", "32", 
	"MKK", "5G", "20M", "HT", "1T", "48", "32",
	"FCC", "5G", "20M", "HT", "1T", "52", "34", 
	"ETSI", "5G", "20M", "HT", "1T", "52", "32", 
	"MKK", "5G", "20M", "HT", "1T", "52", "32",
	"FCC", "5G", "20M", "HT", "1T", "56", "34", 
	"ETSI", "5G", "20M", "HT", "1T", "56", "32", 
	"MKK", "5G", "20M", "HT", "1T", "56", "32",
	"FCC", "5G", "20M", "HT", "1T", "60", "32", 
	"ETSI", "5G", "20M", "HT", "1T", "60", "32", 
	"MKK", "5G", "20M", "HT", "1T", "60", "32",
	"FCC", "5G", "20M", "HT", "1T", "64", "28", 
	"ETSI", "5G", "20M", "HT", "1T", "64", "32", 
	"MKK", "5G", "20M", "HT", "1T", "64", "32",
	"FCC", "5G", "20M", "HT", "1T", "100", "30", 
	"ETSI", "5G", "20M", "HT", "1T", "100", "32", 
	"MKK", "5G", "20M", "HT", "1T", "100", "32",
	"FCC", "5G", "20M", "HT", "1T", "114", "30", 
	"ETSI", "5G", "20M", "HT", "1T", "114", "32", 
	"MKK", "5G", "20M", "HT", "1T", "114", "32",
	"FCC", "5G", "20M", "HT", "1T", "108", "32", 
	"ETSI", "5G", "20M", "HT", "1T", "108", "32", 
	"MKK", "5G", "20M", "HT", "1T", "108", "32",
	"FCC", "5G", "20M", "HT", "1T", "112", "34", 
	"ETSI", "5G", "20M", "HT", "1T", "112", "32", 
	"MKK", "5G", "20M", "HT", "1T", "112", "32",
	"FCC", "5G", "20M", "HT", "1T", "116", "34", 
	"ETSI", "5G", "20M", "HT", "1T", "116", "32", 
	"MKK", "5G", "20M", "HT", "1T", "116", "32",
	"FCC", "5G", "20M", "HT", "1T", "120", "34", 
	"ETSI", "5G", "20M", "HT", "1T", "120", "32", 
	"MKK", "5G", "20M", "HT", "1T", "120", "32",
	"FCC", "5G", "20M", "HT", "1T", "124", "34", 
	"ETSI", "5G", "20M", "HT", "1T", "124", "32", 
	"MKK", "5G", "20M", "HT", "1T", "124", "32",
	"FCC", "5G", "20M", "HT", "1T", "128", "32", 
	"ETSI", "5G", "20M", "HT", "1T", "128", "32", 
	"MKK", "5G", "20M", "HT", "1T", "128", "32",
	"FCC", "5G", "20M", "HT", "1T", "132", "30", 
	"ETSI", "5G", "20M", "HT", "1T", "132", "32", 
	"MKK", "5G", "20M", "HT", "1T", "132", "32",
	"FCC", "5G", "20M", "HT", "1T", "136", "30", 
	"ETSI", "5G", "20M", "HT", "1T", "136", "32", 
	"MKK", "5G", "20M", "HT", "1T", "136", "32",
	"FCC", "5G", "20M", "HT", "1T", "140", "28", 
	"ETSI", "5G", "20M", "HT", "1T", "140", "32", 
	"MKK", "5G", "20M", "HT", "1T", "140", "32",
	"FCC", "5G", "20M", "HT", "1T", "149", "34", 
	"ETSI", "5G", "20M", "HT", "1T", "149", "32", 
	"MKK", "5G", "20M", "HT", "1T", "149", "63",
	"FCC", "5G", "20M", "HT", "1T", "153", "34", 
	"ETSI", "5G", "20M", "HT", "1T", "153", "32", 
	"MKK", "5G", "20M", "HT", "1T", "153", "63",
	"FCC", "5G", "20M", "HT", "1T", "157", "34", 
	"ETSI", "5G", "20M", "HT", "1T", "157", "32", 
	"MKK", "5G", "20M", "HT", "1T", "157", "63",
	"FCC", "5G", "20M", "HT", "1T", "161", "34", 
	"ETSI", "5G", "20M", "HT", "1T", "161", "32", 
	"MKK", "5G", "20M", "HT", "1T", "161", "63",
	"FCC", "5G", "20M", "HT", "1T", "165", "34", 
	"ETSI", "5G", "20M", "HT", "1T", "165", "32", 
	"MKK", "5G", "20M", "HT", "1T", "165", "63",
	"FCC", "5G", "20M", "HT", "2T", "36", "28", 
	"ETSI", "5G", "20M", "HT", "2T", "36", "30", 
	"MKK", "5G", "20M", "HT", "2T", "36", "30",
	"FCC", "5G", "20M", "HT", "2T", "40", "28", 
	"ETSI", "5G", "20M", "HT", "2T", "40", "30", 
	"MKK", "5G", "20M", "HT", "2T", "40", "30",
	"FCC", "5G", "20M", "HT", "2T", "44", "28", 
	"ETSI", "5G", "20M", "HT", "2T", "44", "30", 
	"MKK", "5G", "20M", "HT", "2T", "44", "30",
	"FCC", "5G", "20M", "HT", "2T", "48", "28", 
	"ETSI", "5G", "20M", "HT", "2T", "48", "30", 
	"MKK", "5G", "20M", "HT", "2T", "48", "30",
	"FCC", "5G", "20M", "HT", "2T", "52", "34", 
	"ETSI", "5G", "20M", "HT", "2T", "52", "30", 
	"MKK", "5G", "20M", "HT", "2T", "52", "30",
	"FCC", "5G", "20M", "HT", "2T", "56", "32", 
	"ETSI", "5G", "20M", "HT", "2T", "56", "30", 
	"MKK", "5G", "20M", "HT", "2T", "56", "30",
	"FCC", "5G", "20M", "HT", "2T", "60", "30", 
	"ETSI", "5G", "20M", "HT", "2T", "60", "30", 
	"MKK", "5G", "20M", "HT", "2T", "60", "30",
	"FCC", "5G", "20M", "HT", "2T", "64", "26", 
	"ETSI", "5G", "20M", "HT", "2T", "64", "30", 
	"MKK", "5G", "20M", "HT", "2T", "64", "30",
	"FCC", "5G", "20M", "HT", "2T", "100", "28", 
	"ETSI", "5G", "20M", "HT", "2T", "100", "30", 
	"MKK", "5G", "20M", "HT", "2T", "100", "30",
	"FCC", "5G", "20M", "HT", "2T", "114", "28", 
	"ETSI", "5G", "20M", "HT", "2T", "114", "30", 
	"MKK", "5G", "20M", "HT", "2T", "114", "30",
	"FCC", "5G", "20M", "HT", "2T", "108", "30", 
	"ETSI", "5G", "20M", "HT", "2T", "108", "30", 
	"MKK", "5G", "20M", "HT", "2T", "108", "30",
	"FCC", "5G", "20M", "HT", "2T", "112", "32", 
	"ETSI", "5G", "20M", "HT", "2T", "112", "30", 
	"MKK", "5G", "20M", "HT", "2T", "112", "30",
	"FCC", "5G", "20M", "HT", "2T", "116", "32", 
	"ETSI", "5G", "20M", "HT", "2T", "116", "30", 
	"MKK", "5G", "20M", "HT", "2T", "116", "30",
	"FCC", "5G", "20M", "HT", "2T", "120", "34", 
	"ETSI", "5G", "20M", "HT", "2T", "120", "30", 
	"MKK", "5G", "20M", "HT", "2T", "120", "30",
	"FCC", "5G", "20M", "HT", "2T", "124", "32", 
	"ETSI", "5G", "20M", "HT", "2T", "124", "30", 
	"MKK", "5G", "20M", "HT", "2T", "124", "30",
	"FCC", "5G", "20M", "HT", "2T", "128", "30", 
	"ETSI", "5G", "20M", "HT", "2T", "128", "30", 
	"MKK", "5G", "20M", "HT", "2T", "128", "30",
	"FCC", "5G", "20M", "HT", "2T", "132", "28", 
	"ETSI", "5G", "20M", "HT", "2T", "132", "30", 
	"MKK", "5G", "20M", "HT", "2T", "132", "30",
	"FCC", "5G", "20M", "HT", "2T", "136", "28", 
	"ETSI", "5G", "20M", "HT", "2T", "136", "30", 
	"MKK", "5G", "20M", "HT", "2T", "136", "30",
	"FCC", "5G", "20M", "HT", "2T", "140", "26", 
	"ETSI", "5G", "20M", "HT", "2T", "140", "30", 
	"MKK", "5G", "20M", "HT", "2T", "140", "30",
	"FCC", "5G", "20M", "HT", "2T", "149", "34", 
	"ETSI", "5G", "20M", "HT", "2T", "149", "30", 
	"MKK", "5G", "20M", "HT", "2T", "149", "63",
	"FCC", "5G", "20M", "HT", "2T", "153", "34", 
	"ETSI", "5G", "20M", "HT", "2T", "153", "30", 
	"MKK", "5G", "20M", "HT", "2T", "153", "63",
	"FCC", "5G", "20M", "HT", "2T", "157", "34", 
	"ETSI", "5G", "20M", "HT", "2T", "157", "30", 
	"MKK", "5G", "20M", "HT", "2T", "157", "63",
	"FCC", "5G", "20M", "HT", "2T", "161", "34", 
	"ETSI", "5G", "20M", "HT", "2T", "161", "30", 
	"MKK", "5G", "20M", "HT", "2T", "161", "63",
	"FCC", "5G", "20M", "HT", "2T", "165", "34", 
	"ETSI", "5G", "20M", "HT", "2T", "165", "30", 
	"MKK", "5G", "20M", "HT", "2T", "165", "63",
	"FCC", "5G", "40M", "HT", "1T", "38", "30", 
	"ETSI", "5G", "40M", "HT", "1T", "38", "32", 
	"MKK", "5G", "40M", "HT", "1T", "38", "32",
	"FCC", "5G", "40M", "HT", "1T", "46", "30", 
	"ETSI", "5G", "40M", "HT", "1T", "46", "32", 
	"MKK", "5G", "40M", "HT", "1T", "46", "32",
	"FCC", "5G", "40M", "HT", "1T", "54", "32", 
	"ETSI", "5G", "40M", "HT", "1T", "54", "32", 
	"MKK", "5G", "40M", "HT", "1T", "54", "32",
	"FCC", "5G", "40M", "HT", "1T", "62", "32", 
	"ETSI", "5G", "40M", "HT", "1T", "62", "32", 
	"MKK", "5G", "40M", "HT", "1T", "62", "32",
	"FCC", "5G", "40M", "HT", "1T", "102", "28", 
	"ETSI", "5G", "40M", "HT", "1T", "102", "32", 
	"MKK", "5G", "40M", "HT", "1T", "102", "32",
	"FCC", "5G", "40M", "HT", "1T", "110", "32", 
	"ETSI", "5G", "40M", "HT", "1T", "110", "32", 
	"MKK", "5G", "40M", "HT", "1T", "110", "32",
	"FCC", "5G", "40M", "HT", "1T", "118", "34", 
	"ETSI", "5G", "40M", "HT", "1T", "118", "32", 
	"MKK", "5G", "40M", "HT", "1T", "118", "32",
	"FCC", "5G", "40M", "HT", "1T", "126", "34", 
	"ETSI", "5G", "40M", "HT", "1T", "126", "32", 
	"MKK", "5G", "40M", "HT", "1T", "126", "32",
	"FCC", "5G", "40M", "HT", "1T", "134", "32", 
	"ETSI", "5G", "40M", "HT", "1T", "134", "32", 
	"MKK", "5G", "40M", "HT", "1T", "134", "32",
	"FCC", "5G", "40M", "HT", "1T", "151", "34", 
	"ETSI", "5G", "40M", "HT", "1T", "151", "32", 
	"MKK", "5G", "40M", "HT", "1T", "151", "63",
	"FCC", "5G", "40M", "HT", "1T", "159", "34", 
	"ETSI", "5G", "40M", "HT", "1T", "159", "32", 
	"MKK", "5G", "40M", "HT", "1T", "159", "63",
	"FCC", "5G", "40M", "HT", "2T", "38", "28", 
	"ETSI", "5G", "40M", "HT", "2T", "38", "30", 
	"MKK", "5G", "40M", "HT", "2T", "38", "30",
	"FCC", "5G", "40M", "HT", "2T", "46", "28", 
	"ETSI", "5G", "40M", "HT", "2T", "46", "30", 
	"MKK", "5G", "40M", "HT", "2T", "46", "30",
	"FCC", "5G", "40M", "HT", "2T", "54", "30", 
	"ETSI", "5G", "40M", "HT", "2T", "54", "30", 
	"MKK", "5G", "40M", "HT", "2T", "54", "30",
	"FCC", "5G", "40M", "HT", "2T", "62", "30", 
	"ETSI", "5G", "40M", "HT", "2T", "62", "30", 
	"MKK", "5G", "40M", "HT", "2T", "62", "30",
	"FCC", "5G", "40M", "HT", "2T", "102", "26", 
	"ETSI", "5G", "40M", "HT", "2T", "102", "30", 
	"MKK", "5G", "40M", "HT", "2T", "102", "30",
	"FCC", "5G", "40M", "HT", "2T", "110", "30", 
	"ETSI", "5G", "40M", "HT", "2T", "110", "30", 
	"MKK", "5G", "40M", "HT", "2T", "110", "30",
	"FCC", "5G", "40M", "HT", "2T", "118", "34", 
	"ETSI", "5G", "40M", "HT", "2T", "118", "30", 
	"MKK", "5G", "40M", "HT", "2T", "118", "30",
	"FCC", "5G", "40M", "HT", "2T", "126", "32", 
	"ETSI", "5G", "40M", "HT", "2T", "126", "30", 
	"MKK", "5G", "40M", "HT", "2T", "126", "30",
	"FCC", "5G", "40M", "HT", "2T", "134", "30", 
	"ETSI", "5G", "40M", "HT", "2T", "134", "30", 
	"MKK", "5G", "40M", "HT", "2T", "134", "30",
	"FCC", "5G", "40M", "HT", "2T", "151", "34", 
	"ETSI", "5G", "40M", "HT", "2T", "151", "30", 
	"MKK", "5G", "40M", "HT", "2T", "151", "63",
	"FCC", "5G", "40M", "HT", "2T", "159", "34", 
	"ETSI", "5G", "40M", "HT", "2T", "159", "30", 
	"MKK", "5G", "40M", "HT", "2T", "159", "63",
	"FCC", "5G", "80M", "VHT", "1T", "42", "30", 
	"ETSI", "5G", "80M", "VHT", "1T", "42", "32", 
	"MKK", "5G", "80M", "VHT", "1T", "42", "32",
	"FCC", "5G", "80M", "VHT", "1T", "58", "28", 
	"ETSI", "5G", "80M", "VHT", "1T", "58", "32", 
	"MKK", "5G", "80M", "VHT", "1T", "58", "32",
	"FCC", "5G", "80M", "VHT", "1T", "106", "30", 
	"ETSI", "5G", "80M", "VHT", "1T", "106", "32", 
	"MKK", "5G", "80M", "VHT", "1T", "106", "32",
	"FCC", "5G", "80M", "VHT", "1T", "122", "34", 
	"ETSI", "5G", "80M", "VHT", "1T", "122", "32", 
	"MKK", "5G", "80M", "VHT", "1T", "122", "32",
	"FCC", "5G", "80M", "VHT", "1T", "155", "34", 
	"ETSI", "5G", "80M", "VHT", "1T", "155", "32", 
	"MKK", "5G", "80M", "VHT", "1T", "155", "63",
	"FCC", "5G", "80M", "VHT", "2T", "42", "28", 
	"ETSI", "5G", "80M", "VHT", "2T", "42", "30", 
	"MKK", "5G", "80M", "VHT", "2T", "42", "30",
	"FCC", "5G", "80M", "VHT", "2T", "58", "26", 
	"ETSI", "5G", "80M", "VHT", "2T", "58", "30", 
	"MKK", "5G", "80M", "VHT", "2T", "58", "30",
	"FCC", "5G", "80M", "VHT", "2T", "106", "28", 
	"ETSI", "5G", "80M", "VHT", "2T", "106", "30", 
	"MKK", "5G", "80M", "VHT", "2T", "106", "30",
	"FCC", "5G", "80M", "VHT", "2T", "122", "32", 
	"ETSI", "5G", "80M", "VHT", "2T", "122", "30", 
	"MKK", "5G", "80M", "VHT", "2T", "122", "30",
	"FCC", "5G", "80M", "VHT", "2T", "155", "34", 
	"ETSI", "5G", "80M", "VHT", "2T", "155", "30", 
	"MKK", "5G", "80M", "VHT", "2T", "155", "63"
};

void
ODM_ReadAndConfig_MP_8703B_TXPWR_LMT(
	IN   PDM_ODM_T  pDM_Odm
)
{
	u4Byte     i           = 0;
	u4Byte     ArrayLen    = sizeof(Array_MP_8703B_TXPWR_LMT)/sizeof(pu1Byte);
	pu1Byte    *Array      = (pu1Byte *)Array_MP_8703B_TXPWR_LMT;

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PADAPTER		Adapter = pDM_Odm->Adapter;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);

	PlatformZeroMemory(pHalData->BufOfLinesPwrLmt, MAX_LINES_HWCONFIG_TXT*MAX_BYTES_LINE_HWCONFIG_TXT);
	pHalData->nLinesReadPwrLmt = ArrayLen/7;
#endif

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("===> ODM_ReadAndConfig_MP_8703B_TXPWR_LMT\n"));

	for (i = 0; i < ArrayLen; i += 7) {
		pu1Byte regulation = Array[i];
		pu1Byte band = Array[i+1];
		pu1Byte bandwidth = Array[i+2];
		pu1Byte rate = Array[i+3];
		pu1Byte rfPath = Array[i+4];
		pu1Byte chnl = Array[i+5];
		pu1Byte val = Array[i+6];
	
		odm_ConfigBB_TXPWR_LMT_8703B(pDM_Odm, regulation, band, bandwidth, rate, rfPath, chnl, val);
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
		rsprintf((char *)pHalData->BufOfLinesPwrLmt[i/7], 100, "\"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\",",
			regulation, band, bandwidth, rate, rfPath, chnl, val);
#endif
	}

}

#endif /* end of HWIMG_SUPPORT*/

