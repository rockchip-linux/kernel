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

#include "mp_precomp.h"
#include "../phydm_precomp.h"



/*---------------------------Define Local Constant---------------------------*/
//IQK
#define IQK_DELAY_TIME_8703B	10
#define LCK_DELAY_TIME_8703B	100

/* LTE_COEX */
#define REG_LTECOEX_CTRL 0x07C0
#define REG_LTECOEX_WRITE_DATA 0x07C4
#define REG_LTECOEX_READ_DATA 0x07C8
#define REG_LTECOEX_PATH_CONTROL 0x70



// 2010/04/25 MH Define the max tx power tracking tx agc power.
#define		ODM_TXPWRTRACK_MAX_IDX8703B		6

#define     IDX_0xC94                       0
#define     IDX_0xC80                       1
#define     IDX_0xC4C                       2

#define     IDX_0xC14                       0
#define     IDX_0xCA0                       1

#define     KEY                             0
#define     VAL                             1

/*---------------------------Define Local Constant---------------------------*/


//3============================================================
//3 Tx Power Tracking
//3============================================================


void setIqkMatrix_8703B(
	PDM_ODM_T	pDM_Odm,
	u1Byte		OFDM_index,
	u1Byte		RFPath,
	s4Byte		IqkResult_X,
	s4Byte		IqkResult_Y
)
{
	s4Byte			ele_A = 0, ele_D = 0, ele_C = 0, value32, tmp;
	s4Byte			ele_A_ext = 0, ele_C_ext = 0, ele_D_ext = 0;

	RFPath = ODM_RF_PATH_A;


	if (OFDM_index >= OFDM_TABLE_SIZE)
		OFDM_index = OFDM_TABLE_SIZE-1;
	else if (OFDM_index < 0)
		OFDM_index = 0;

	if ((IqkResult_X != 0) && (*(pDM_Odm->pBandType) == ODM_BAND_2_4G)) {

		/* new element D */
		ele_D = (OFDMSwingTable_New[OFDM_index] & 0xFFC00000)>>22;
		ele_D_ext = (((IqkResult_X * ele_D)>>7)&0x01);

		/* new element A */
		if ((IqkResult_X & 0x00000200) != 0)		/* consider minus */
			IqkResult_X = IqkResult_X | 0xFFFFFC00;
		ele_A = ((IqkResult_X * ele_D)>>8)&0x000003FF;
		ele_A_ext = ((IqkResult_X * ele_D)>>7) & 0x1;
		/* new element C */
		if ((IqkResult_Y & 0x00000200) != 0)
			IqkResult_Y = IqkResult_Y | 0xFFFFFC00;
		ele_C = ((IqkResult_Y * ele_D)>>8)&0x000003FF;
		ele_C_ext = ((IqkResult_Y * ele_D)>>7) & 0x1;

		switch (RFPath) {
		case ODM_RF_PATH_A:
			/* write new elements A, C, D to regC80, regC94, reg0xc4c, and element B is always 0 */
			/* write 0xc80 */
			value32 = (ele_D << 22) | ((ele_C & 0x3F) << 16) | ele_A;
			ODM_SetBBReg(pDM_Odm, rOFDM0_XATxIQImbalance, bMaskDWord, value32);
			/* write 0xc94 */
			value32 = (ele_C & 0x000003C0) >> 6;
			ODM_SetBBReg(pDM_Odm, rOFDM0_XCTxAFE, bMaskH4Bits, value32);
			/* write 0xc4c */
			value32 = (ele_D_ext << 28) | (ele_A_ext << 31) | (ele_C_ext << 29);
			value32 = (ODM_GetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, bMaskDWord)&(~(BIT31|BIT29|BIT28))) | value32;
			ODM_SetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, bMaskDWord, value32);
			break;
		case ODM_RF_PATH_B:
			/* write new elements A, C, D to regC88, regC9C, regC4C, and element B is always 0 */
			/* write 0xc88 */
			value32 = (ele_D << 22) | ((ele_C & 0x3F) << 16) | ele_A;
			ODM_SetBBReg(pDM_Odm, rOFDM0_XBTxIQImbalance, bMaskDWord, value32);
			/* write 0xc9c */
			value32 = (ele_C & 0x000003C0) >> 6;
			ODM_SetBBReg(pDM_Odm, rOFDM0_XDTxAFE, bMaskH4Bits, value32);
			/* write 0xc4c */
			value32 = (ele_D_ext << 24) | (ele_A_ext << 27) | (ele_C_ext << 25);
			value32 = (ODM_GetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, bMaskDWord)&(~(BIT24|BIT27|BIT25))) | value32;
			ODM_SetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, bMaskDWord, value32);
			break;
		default:
			break;
		}
	} else {
		switch (RFPath) {
		case ODM_RF_PATH_A:
			ODM_SetBBReg(pDM_Odm, rOFDM0_XATxIQImbalance, bMaskDWord, OFDMSwingTable_New[OFDM_index]);
			ODM_SetBBReg(pDM_Odm, rOFDM0_XCTxAFE, bMaskH4Bits, 0x00);
			value32 = ODM_GetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, bMaskDWord)&(~(BIT31|BIT29|BIT28));
			ODM_SetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, bMaskDWord, value32);
			break;

		case ODM_RF_PATH_B:
			ODM_SetBBReg(pDM_Odm, rOFDM0_XBTxIQImbalance, bMaskDWord, OFDMSwingTable_New[OFDM_index]);
			ODM_SetBBReg(pDM_Odm, rOFDM0_XDTxAFE, bMaskH4Bits, 0x00);
			value32 = ODM_GetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, bMaskDWord)&(~(BIT24|BIT27|BIT25));
			ODM_SetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, bMaskDWord, value32);
			break;

		default:
			break;
		}
	}

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("TxPwrTracking path %c: X = 0x%x, Y = 0x%x ele_A = 0x%x ele_C = 0x%x ele_D = 0x%x ele_A_ext = 0x%x ele_C_ext = 0x%x ele_D_ext = 0x%x\n",
				 (RFPath == ODM_RF_PATH_A ? 'A' : 'B'), (u4Byte)IqkResult_X, (u4Byte)IqkResult_Y, (u4Byte)ele_A, (u4Byte)ele_C, (u4Byte)ele_D, (u4Byte)ele_A_ext, (u4Byte)ele_C_ext, (u4Byte)ele_D_ext));
}

VOID
setCCKFilterCoefficient_8703B(
	PDM_ODM_T	pDM_Odm,
	u1Byte 		CCKSwingIndex
)
{
	ODM_Write1Byte(pDM_Odm, 0xa22, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][0]);
	ODM_Write1Byte(pDM_Odm, 0xa23, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][1]);
	ODM_Write1Byte(pDM_Odm, 0xa24, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][2]);
	ODM_Write1Byte(pDM_Odm, 0xa25, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][3]);
	ODM_Write1Byte(pDM_Odm, 0xa26, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][4]);
	ODM_Write1Byte(pDM_Odm, 0xa27, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][5]);
	ODM_Write1Byte(pDM_Odm, 0xa28, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][6]);
	ODM_Write1Byte(pDM_Odm, 0xa29, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][7]);
	ODM_Write1Byte(pDM_Odm, 0xa9a, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][8]);
	ODM_Write1Byte(pDM_Odm, 0xa9b, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][9]);
	ODM_Write1Byte(pDM_Odm, 0xa9c, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][10]);
	ODM_Write1Byte(pDM_Odm, 0xa9d, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][11]);
	ODM_Write1Byte(pDM_Odm, 0xaa0, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][12]);
	ODM_Write1Byte(pDM_Odm, 0xaa1, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][13]);
	ODM_Write1Byte(pDM_Odm, 0xaa2, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][14]);
	ODM_Write1Byte(pDM_Odm, 0xaa3, CCKSwingTable_Ch1_Ch14_88F[CCKSwingIndex][15]);
}

void DoIQK_8703B(
	PVOID		pDM_VOID,
	u1Byte		DeltaThermalIndex,
	u1Byte		ThermalValue,
	u1Byte		Threshold
)
{
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;

#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	PADAPTER 		Adapter = pDM_Odm->Adapter;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
#endif

	ODM_ResetIQKResult(pDM_Odm);


	pDM_Odm->RFCalibrateInfo.ThermalValue_IQK= ThermalValue;
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	PHY_IQCalibrate_8703B(pDM_Odm, FALSE);
#else
	PHY_IQCalibrate_8703B(Adapter, FALSE);
#endif


}

/*-----------------------------------------------------------------------------
 * Function:	odm_TxPwrTrackSetPwr88E()
 *
 * Overview:	88E change all channel tx power accordign to flag.
 *				OFDM & CCK are all different.
 *
 * Input:		NONE
 *
 * Output:		NONE
 *
 * Return:		NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	04/23/2012	MHC		Create Version 0.
 *
 *---------------------------------------------------------------------------*/
VOID
ODM_TxPwrTrackSetPwr_8703B(
	IN	PVOID		pDM_VOID,
	PWRTRACK_METHOD 	Method,
	u1Byte 				RFPath,
	u1Byte 				ChannelMappedIndex
)
{
	PDM_ODM_T		pDM_Odm = (PDM_ODM_T)pDM_VOID;
	PADAPTER	Adapter = pDM_Odm->Adapter;
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(Adapter);
	u1Byte		PwrTrackingLimit_OFDM = 34; /* +0dB */
	u1Byte		PwrTrackingLimit_CCK = CCK_TABLE_SIZE_88F - 1;   /* -2dB */
	u1Byte		TxRate = 0xFF;
	u1Byte		Final_OFDM_Swing_Index = 0;
	u1Byte		Final_CCK_Swing_Index = 0;
	u1Byte		i = 0;
	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);
	
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN))
#if (MP_DRIVER == 1)	/*win MP */
	PMPT_CONTEXT			pMptCtx = &(Adapter->MptCtx);
	TxRate = MptToMgntRate(pMptCtx->MptRateIndex);
#else	/*win normal*/
	PMGNT_INFO      		pMgntInfo = &(Adapter->MgntInfo);
	if (!pMgntInfo->ForcedDataRate) {	/*auto rate*/
		if (pDM_Odm->TxRate != 0xFF)
			TxRate = Adapter->HalFunc.GetHwRateFromMRateHandler(pDM_Odm->TxRate);
	} else {
		TxRate = (u1Byte) pMgntInfo->ForcedDataRate;
	}
#endif
#elif (DM_ODM_SUPPORT_TYPE & (ODM_CE))
	if (pDM_Odm->mp_mode == TRUE) {	/*CE MP*/
		PMPT_CONTEXT		pMptCtx = &(Adapter->mppriv.MptCtx);

		TxRate = MptToMgntRate(pMptCtx->MptRateIndex);
	} else {	/*CE normal*/
		u2Byte	rate	 = *(pDM_Odm->pForcedDataRate);

		if (!rate) {	/*auto rate*/
			if (pDM_Odm->TxRate != 0xFF)
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN))
				TxRate = Adapter->HalFunc.GetHwRateFromMRateHandler(pDM_Odm->TxRate);
#elif (DM_ODM_SUPPORT_TYPE & (ODM_CE))
				TxRate = HwRateToMRate(pDM_Odm->TxRate);
#endif
		} else {	/*force rate*/
			TxRate = (u1Byte)rate;
		}
	}
#endif


	ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("===>ODM_TxPwrTrackSetPwr8703B\n"));

	if (TxRate != 0xFF) {
		/*2 CCK*/
		if ((TxRate >= MGN_1M) && (TxRate <= MGN_11M))
			PwrTrackingLimit_CCK = CCK_TABLE_SIZE_88F - 1;
		/*2 OFDM*/
		else if ((TxRate >= MGN_6M) && (TxRate <= MGN_48M))
			PwrTrackingLimit_OFDM = 36;	/*+3dB*/
		else if (TxRate == MGN_54M)
			PwrTrackingLimit_OFDM = 34;	/*+2dB*/
		/*2 HT*/
		else if ((TxRate >= MGN_MCS0) && (TxRate <= MGN_MCS2))	/*QPSK/BPSK*/
			PwrTrackingLimit_OFDM = 38;	/*+4dB*/
		else if ((TxRate >= MGN_MCS3) && (TxRate <= MGN_MCS4))	/*16QAM*/
			PwrTrackingLimit_OFDM = 36;	/*+3dB*/
		else if ((TxRate >= MGN_MCS5) && (TxRate <= MGN_MCS7))	/*64QAM*/
			PwrTrackingLimit_OFDM = 34;	/*+2dB*/
		else
			PwrTrackingLimit_OFDM =  pRFCalibrateInfo->DefaultOfdmIndex;   /*Default OFDM index = 30*/
	}
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("TxRate=0x%x, PwrTrackingLimit=%d\n", TxRate, PwrTrackingLimit_OFDM));

	if (Method == TXAGC) {
		u1Byte	rf = 0;
		u4Byte	pwr = 0, TxAGC = 0;
		PADAPTER Adapter = pDM_Odm->Adapter;

		ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("odm_TxPwrTrackSetPwr8703B CH=%d\n", *(pDM_Odm->pChannel)));

		pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath] = pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath];

#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN|ODM_CE ))
#if (MP_DRIVER != 1)
		pRFCalibrateInfo->Modify_TxAGC_Flag_PathA = TRUE;
		pRFCalibrateInfo->Modify_TxAGC_Flag_PathA_CCK = TRUE;

		PHY_SetTxPowerIndexByRateSection(Adapter, RFPath, pHalData->CurrentChannel, CCK);
		PHY_SetTxPowerIndexByRateSection(Adapter, RFPath, pHalData->CurrentChannel, OFDM);
		PHY_SetTxPowerIndexByRateSection(Adapter, RFPath, pHalData->CurrentChannel, HT_MCS0_MCS7);
#else
		pwr = PHY_QueryBBReg(Adapter, rTxAGC_A_Rate18_06, 0xFF);
		pwr += pRFCalibrateInfo->PowerIndexOffset[RFPath];
		PHY_SetBBReg(Adapter, rTxAGC_A_CCK1_Mcs32, bMaskByte1, pwr);
		TxAGC = (pwr<<16)|(pwr<<8)|(pwr);
		PHY_SetBBReg(Adapter, rTxAGC_B_CCK11_A_CCK2_11, 0xffffff00, TxAGC);
		RT_DISP(FPHY, PHY_TXPWR, ("ODM_TxPwrTrackSetPwr8703B: CCK Tx-rf(A) Power = 0x%x\n", TxAGC));

		pwr = PHY_QueryBBReg(Adapter, rTxAGC_A_Rate18_06, 0xFF);
		pwr += (pRFCalibrateInfo->BbSwingIdxOfdm[RFPath] - pRFCalibrateInfo->BbSwingIdxOfdmBase[RFPath]);
		TxAGC |= ((pwr<<24)|(pwr<<16)|(pwr<<8)|pwr);
		PHY_SetBBReg(Adapter, rTxAGC_A_Rate18_06, bMaskDWord, TxAGC);
		PHY_SetBBReg(Adapter, rTxAGC_A_Rate54_24, bMaskDWord, TxAGC);
		PHY_SetBBReg(Adapter, rTxAGC_A_Mcs03_Mcs00, bMaskDWord, TxAGC);
		PHY_SetBBReg(Adapter, rTxAGC_A_Mcs07_Mcs04, bMaskDWord, TxAGC);
		PHY_SetBBReg(Adapter, rTxAGC_A_Mcs11_Mcs08, bMaskDWord, TxAGC);
		PHY_SetBBReg(Adapter, rTxAGC_A_Mcs15_Mcs12, bMaskDWord, TxAGC);
		RT_DISP(FPHY, PHY_TXPWR, ("ODM_TxPwrTrackSetPwr8703B: OFDM Tx-rf(A) Power = 0x%x\n", TxAGC));
#endif
#endif
	} else if (Method == BBSWING) {
		Final_OFDM_Swing_Index = pRFCalibrateInfo->DefaultOfdmIndex + pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath];
		Final_CCK_Swing_Index = pRFCalibrateInfo->DefaultCckIndex + pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath];

		ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
					 (" pRFCalibrateInfo->DefaultOfdmIndex=%d,  pRFCalibrateInfo->DefaultCCKIndex=%d, pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath]=%d, pRFCalibrateInfo->Remnant_CCKSwingIdx=%d   RF_Path = %d\n",
					  pRFCalibrateInfo->DefaultOfdmIndex, pRFCalibrateInfo->DefaultCckIndex, pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath], pRFCalibrateInfo->Remnant_CCKSwingIdx, RFPath));

		/* Adjust BB swing by OFDM IQ matrix */
		if (Final_OFDM_Swing_Index >= PwrTrackingLimit_OFDM)
			Final_OFDM_Swing_Index = PwrTrackingLimit_OFDM;
		else if (Final_OFDM_Swing_Index < 0)
			Final_OFDM_Swing_Index = 0;

		if (Final_CCK_Swing_Index >= CCK_TABLE_SIZE)
			Final_CCK_Swing_Index = CCK_TABLE_SIZE-1;
		else if (pRFCalibrateInfo->BbSwingIdxCck < 0)
			Final_CCK_Swing_Index = 0;

		setIqkMatrix_8703B(pDM_Odm, Final_OFDM_Swing_Index, ODM_RF_PATH_A,
						   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][0],
						   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][1]);

		setCCKFilterCoefficient_8703B(pDM_Odm, Final_CCK_Swing_Index);

	} else if (Method == MIX_MODE) {
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
					 (" pDM_Odm->DefaultOfdmIndex=%d,  pDM_Odm->DefaultCCKIndex=%d, pDM_Odm->Absolute_OFDMSwingIdx[RFPath]=%d, pDM_Odm->Remnant_CCKSwingIdx=%d   RF_Path = %d\n",
					  pRFCalibrateInfo->DefaultOfdmIndex, pRFCalibrateInfo->DefaultCckIndex, pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath], pRFCalibrateInfo->Remnant_CCKSwingIdx, RFPath));

		Final_OFDM_Swing_Index = pRFCalibrateInfo->DefaultOfdmIndex + pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath];
		Final_CCK_Swing_Index  = pRFCalibrateInfo->DefaultCckIndex + pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath];

		ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
					 (" pDM_Odm->DefaultOfdmIndex=%d,  pDM_Odm->DefaultCCKIndex=%d, pDM_Odm->Absolute_OFDMSwingIdx[RFPath]=%d   RF_Path = %d\n",
					  pRFCalibrateInfo->DefaultOfdmIndex, pRFCalibrateInfo->DefaultCckIndex, pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath], RFPath));

		ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
					 (" Final_OFDM_Swing_Index=%d,  Final_CCK_Swing_Index=%d RFPath=%d\n",
					  Final_OFDM_Swing_Index, Final_CCK_Swing_Index, RFPath));


		if (Final_OFDM_Swing_Index > PwrTrackingLimit_OFDM) {   /*BBSwing higher then Limit*/
			pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath] = Final_OFDM_Swing_Index - PwrTrackingLimit_OFDM;

			setIqkMatrix_8703B(pDM_Odm, PwrTrackingLimit_OFDM, ODM_RF_PATH_A,
							   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][0],
							   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][1]);

			pRFCalibrateInfo->Modify_TxAGC_Flag_PathA = TRUE;
			PHY_SetTxPowerIndexByRateSection(Adapter, RFPath, pHalData->CurrentChannel, OFDM);
			PHY_SetTxPowerIndexByRateSection(Adapter, RFPath, pHalData->CurrentChannel, HT_MCS0_MCS7);

			ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
						 (" ******Path_A Over BBSwing Limit , PwrTrackingLimit = %d , Remnant TxAGC Value = %d\n",
						  PwrTrackingLimit_OFDM, pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath]));
		} else if (Final_OFDM_Swing_Index < 0) {
			pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath] = Final_OFDM_Swing_Index;

			setIqkMatrix_8703B(pDM_Odm, 0, ODM_RF_PATH_A,
							   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][0],
							   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][1]);

			pRFCalibrateInfo->Modify_TxAGC_Flag_PathA = TRUE;
			PHY_SetTxPowerIndexByRateSection(Adapter, RFPath, pHalData->CurrentChannel, OFDM);
			PHY_SetTxPowerIndexByRateSection(Adapter, RFPath, pHalData->CurrentChannel, HT_MCS0_MCS7);

			ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
						 (" ******Path_A Lower then BBSwing lower bound  0 , Remnant TxAGC Value = %d\n",
						  pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath]));
		} else {
			setIqkMatrix_8703B(pDM_Odm, Final_OFDM_Swing_Index, ODM_RF_PATH_A,
							   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][0],
							   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][1]);

			ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
						 (" ******Path_A Compensate with BBSwing , Final_OFDM_Swing_Index = %d\n", Final_OFDM_Swing_Index));

			if (pRFCalibrateInfo->Modify_TxAGC_Flag_PathA) {		/*If TxAGC has changed, reset TxAGC again*/
				pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath] = 0;
				PHY_SetTxPowerIndexByRateSection(Adapter, RFPath, pHalData->CurrentChannel, OFDM);
				PHY_SetTxPowerIndexByRateSection(Adapter, RFPath, pHalData->CurrentChannel, HT_MCS0_MCS7);
				pRFCalibrateInfo->Modify_TxAGC_Flag_PathA = FALSE;

				ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
							 (" ******Path_A pDM_Odm->Modify_TxAGC_Flag = FALSE\n"));
			}
		}
		if (Final_CCK_Swing_Index > PwrTrackingLimit_CCK) {
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
						 (" Final_CCK_Swing_Index(%d) > PwrTrackingLimit_CCK(%d)\n", Final_CCK_Swing_Index, PwrTrackingLimit_CCK));

			pRFCalibrateInfo->Remnant_CCKSwingIdx = Final_CCK_Swing_Index - PwrTrackingLimit_CCK;

			setCCKFilterCoefficient_8703B(pDM_Odm, PwrTrackingLimit_CCK);

			ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A CCK Over Limit , PwrTrackingLimit_CCK = %d , pDM_Odm->Remnant_CCKSwingIdx  = %d\n", PwrTrackingLimit_CCK, pRFCalibrateInfo->Remnant_CCKSwingIdx));

			pRFCalibrateInfo->Modify_TxAGC_Flag_PathA_CCK = TRUE;

			PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, CCK);

		} else if (Final_CCK_Swing_Index < 0) {		/* Lowest CCK Index = 0 */

			ODM_RT_TRACE(pDM_Odm,ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
						 (" Final_CCK_Swing_Index(%d) < 0      PwrTrackingLimit_CCK(%d)\n", Final_CCK_Swing_Index, PwrTrackingLimit_CCK));

			pRFCalibrateInfo->Remnant_CCKSwingIdx = Final_CCK_Swing_Index;

			setCCKFilterCoefficient_8703B(pDM_Odm, 0);

			ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A CCK Under Limit , PwrTrackingLimit_CCK = %d , pDM_Odm->Remnant_CCKSwingIdx  = %d\n", 0, pRFCalibrateInfo->Remnant_CCKSwingIdx));

			pRFCalibrateInfo->Modify_TxAGC_Flag_PathA_CCK = TRUE;

			PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, CCK);

		} else {

			ODM_RT_TRACE(pDM_Odm,ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
						 (" else Final_CCK_Swing_Index=%d      PwrTrackingLimit_CCK(%d)\n", Final_CCK_Swing_Index, PwrTrackingLimit_CCK));

			setCCKFilterCoefficient_8703B(pDM_Odm, Final_CCK_Swing_Index);

			ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A CCK Compensate with BBSwing , Final_CCK_Swing_Index = %d\n", Final_CCK_Swing_Index));

			pRFCalibrateInfo->Modify_TxAGC_Flag_PathA_CCK = FALSE;

			pRFCalibrateInfo->Remnant_CCKSwingIdx = 0;

			if (pRFCalibrateInfo->Modify_TxAGC_Flag_PathA_CCK) {		/*If TxAGC has changed, reset TxAGC again*/
				pRFCalibrateInfo->Remnant_CCKSwingIdx = 0;
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, CCK);
				pRFCalibrateInfo->Modify_TxAGC_Flag_PathA_CCK = FALSE;
			}



		}

	} else {
		return; // This method is not supported.
	}
}

VOID
GetDeltaSwingTable_8703B(
	IN	PVOID		pDM_VOID,
	OUT pu1Byte		*TemperatureUP_A,
	OUT pu1Byte		*TemperatureDOWN_A,
	OUT pu1Byte		*TemperatureUP_B,
	OUT pu1Byte		*TemperatureDOWN_B
)
{
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;
	PADAPTER		Adapter 		 = pDM_Odm->Adapter;
	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);
	HAL_DATA_TYPE	*pHalData		 = GET_HAL_DATA(Adapter);
	u1Byte			TxRate			= 0xFF;
	u1Byte			channel 		 = pHalData->CurrentChannel;

	
	if (pDM_Odm->mp_mode == TRUE) {
	#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
		#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
			#if (MP_DRIVER == 1)
					PMPT_CONTEXT pMptCtx = &(Adapter->MptCtx);
					
					TxRate = MptToMgntRate(pMptCtx->MptRateIndex);
			#endif
		#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
				PMPT_CONTEXT pMptCtx = &(Adapter->mppriv.MptCtx);
				
				TxRate = MptToMgntRate(pMptCtx->MptRateIndex);
		#endif	
	#endif
	} else {
		u2Byte	rate	 = *(pDM_Odm->pForcedDataRate);
		
		if (!rate) { /*auto rate*/
			if (rate != 0xFF) {
			#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
						TxRate = Adapter->HalFunc.GetHwRateFromMRateHandler(pDM_Odm->TxRate);
			#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
						TxRate = HwRateToMRate(pDM_Odm->TxRate);
			#endif
			}
		} else { /*force rate*/
			TxRate = (u1Byte)rate;
		}
	}
		
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("Power Tracking TxRate=0x%X\n", TxRate));

	if ( 1 <= channel && channel <= 14) {
		if (IS_CCK_RATE(TxRate)) {
			*TemperatureUP_A   = pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_P;
			*TemperatureDOWN_A = pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_N;
			*TemperatureUP_B   = pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_P;
			*TemperatureDOWN_B = pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_N;
		} else {
			*TemperatureUP_A   = pRFCalibrateInfo->DeltaSwingTableIdx_2GA_P;
			*TemperatureDOWN_A = pRFCalibrateInfo->DeltaSwingTableIdx_2GA_N;
			*TemperatureUP_B   = pRFCalibrateInfo->DeltaSwingTableIdx_2GB_P;
			*TemperatureDOWN_B = pRFCalibrateInfo->DeltaSwingTableIdx_2GB_N;
		}
	} else {
		*TemperatureUP_A   = (pu1Byte)DeltaSwingTableIdx_2GA_P_8188E;
		*TemperatureDOWN_A = (pu1Byte)DeltaSwingTableIdx_2GA_N_8188E;
		*TemperatureUP_B   = (pu1Byte)DeltaSwingTableIdx_2GA_P_8188E;
		*TemperatureDOWN_B = (pu1Byte)DeltaSwingTableIdx_2GA_N_8188E;
	}

	return;
}


void ConfigureTxpowerTrack_8703B(
	PTXPWRTRACK_CFG	pConfig
)
{
	pConfig->SwingTableSize_CCK = CCK_TABLE_SIZE;
	pConfig->SwingTableSize_OFDM = OFDM_TABLE_SIZE;
	pConfig->Threshold_IQK = IQK_THRESHOLD;
	pConfig->AverageThermalNum = AVG_THERMAL_NUM_8703B;
	pConfig->RfPathCount = MAX_PATH_NUM_8703B;
	pConfig->ThermalRegAddr = RF_T_METER_8703B;

	pConfig->ODM_TxPwrTrackSetPwr = ODM_TxPwrTrackSetPwr_8703B;
	pConfig->DoIQK = DoIQK_8703B;
	pConfig->PHY_LCCalibrate = PHY_LCCalibrate_8703B;
	pConfig->GetDeltaSwingTable = GetDeltaSwingTable_8703B;
}

//1 7.	IQK
#define MAX_TOLERANCE		5
#define IQK_DELAY_TIME		1		//ms

u1Byte			//bit0 = 1 => Tx OK, bit1 = 1 => Rx OK
phy_PathA_IQK_8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm
#else
	IN	PADAPTER	pAdapter
#endif
)
{
	u4Byte regEAC, regE94, regE9C, tmp/*, regEA4*/;
	u1Byte result = 0x00, Ktime;
	u4Byte originalPath, originalGNT;

#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
#endif
#endif
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]TX IQK!\n"));

	/*8703b IQK v2.0 20150713*/
	/*1 Tx IQK*/
	/*IQK setting*/
	ODM_SetBBReg(pDM_Odm, rTx_IQK, bMaskDWord, 0x01007c00);
	ODM_SetBBReg(pDM_Odm, rRx_IQK, bMaskDWord, 0x01004800);
	/*path-A IQK setting*/
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_A, bMaskDWord, 0x18008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_A, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_B, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_B, bMaskDWord, 0x38008c1c);
/*	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_A, bMaskDWord, 0x8214010a);*/
	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_A, bMaskDWord, 0x8214030f);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_A, bMaskDWord, 0x28110000);
	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_B, bMaskDWord, 0x82110000);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_B, bMaskDWord, 0x28110000);

	/*LO calibration setting*/
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Rsp, bMaskDWord, 0x00462911);

	/*leave IQK mode*/
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);

	/*PA, PAD setting*/
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0xdf, 0x800, 0x1);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0x55, 0x0007f, 0x7);	
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0x7f, bRFRegOffsetMask, 0x0d400);

	/*enter IQK mode*/
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x808000);

#if 1
	/*path setting*/
	/*Save Original Path Owner, Original GNT*/
	originalPath = ODM_GetMACReg(pDM_Odm, REG_LTECOEX_PATH_CONTROL, bMaskDWord);  /*save 0x70*/
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_CTRL, bMaskDWord, 0x800f0038);
	ODM_delay_ms(1);
	originalGNT = ODM_GetBBReg(pDM_Odm, REG_LTECOEX_READ_DATA, bMaskDWord);  /*save 0x38*/

	/*set GNT_WL=1/GNT_BT=0  and Path owner to WiFi for pause BT traffic*/
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_WRITE_DATA, bMaskDWord, 0x00007700);
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_CTRL, bMaskDWord, 0xc0020038);	/*0x38[15:8] = 0x77*/
	ODM_SetMACReg(pDM_Odm, REG_LTECOEX_PATH_CONTROL, BIT26, 0x1);  /*0x70[26] =1 --> Path Owner to WiFi*/
#endif

	/*One shot, path A LOK & IQK*/
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf9000000);
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf8000000);

	/* delay x ms */
	ODM_delay_ms(IQK_DELAY_TIME_8703B);
	Ktime = 0;
	while ((ODM_GetBBReg(pDM_Odm, 0xe90, bMaskDWord) == 0) && Ktime < 10) {
		ODM_delay_ms(5);
		Ktime++;
	}

#if 1
	/*path setting*/
	/*Restore GNT_WL/GNT_BT  and Path owner*/
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_WRITE_DATA, bMaskDWord, originalGNT);
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_CTRL, bMaskDWord, 0xc00f0038);
	ODM_SetMACReg(pDM_Odm, REG_LTECOEX_PATH_CONTROL, 0xffffffff, originalPath);

	originalPath = ODM_GetMACReg(pDM_Odm, REG_LTECOEX_PATH_CONTROL, bMaskDWord);  /*save 0x70*/
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_CTRL, bMaskDWord, 0x800f0038);
	ODM_delay_ms(1);
	originalGNT = ODM_GetBBReg(pDM_Odm, REG_LTECOEX_READ_DATA, bMaskDWord);  /*save 0x38*/

/*	DBG_871X("[COEXDBG] REG 0x38 = 0x%08X, 0x70=0x%08x After Tx IQK !!!!!!!!!!\n", originalGNT, originalPath);*/
#endif

	/*leave IQK mode*/
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
	/*	PA/PAD controlled by 0x0*/
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0xdf, 0x800, 0x0);

	/* Check failed*/
	regEAC = ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_A_2, bMaskDWord);
	regE94 = ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_A, bMaskDWord);
	regE9C= ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_A, bMaskDWord);
	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xeac = 0x%x\n", regEAC));
	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe94 = 0x%x, 0xe9c = 0x%x\n", regE94, regE9C));
	/*monitor image power before & after IQK*/
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe90(before IQK)= 0x%x, 0xe98(afer IQK) = 0x%x\n",
				 ODM_GetBBReg(pDM_Odm, 0xe90, bMaskDWord), ODM_GetBBReg(pDM_Odm, 0xe98, bMaskDWord)));

	if (!(regEAC & BIT28) &&
	   (((regE94 & 0x03FF0000)>>16) != 0x142) &&
	   (((regE9C & 0x03FF0000)>>16) != 0x42))

		result |= 0x01;

	return result;

}

u1Byte			//bit0 = 1 => Tx OK, bit1 = 1 => Rx OK
phy_PathA_RxIQK8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm
#else
	IN	PADAPTER	pAdapter
#endif
)
{
	u4Byte regEAC, regE94, regE9C, regEA4, u4tmp,tmp;
	u1Byte result = 0x00, Ktime;
	u4Byte originalPath, originalGNT;

#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
#endif
#endif


	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]RX IQK:Get TXIMR setting\n"));
	//1 Get TX_XY

	//IQK setting
	ODM_SetBBReg(pDM_Odm, rTx_IQK, bMaskDWord, 0x01007c00);
	ODM_SetBBReg(pDM_Odm, rRx_IQK, bMaskDWord, 0x01004800);

	//path-A IQK setting
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_A, bMaskDWord, 0x18008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_A, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_B, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_B, bMaskDWord, 0x38008c1c);

//	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_A, bMaskDWord, 0x82160c1f);
	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_A, bMaskDWord, 0x8216000f);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_A, bMaskDWord, 0x28110000);
	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_B, bMaskDWord, 0x82110000);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_B, bMaskDWord, 0x28110000);

	//LO calibration setting
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Rsp, bMaskDWord, 0x0046a911);

	//leave IQK mode
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);

	//modify RXIQK mode table
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_WE_LUT, 0x80000, 0x1);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_RCK_OS, bRFRegOffsetMask, 0x30000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G1, bRFRegOffsetMask, 0x00007);
	/*IQK PA off*/
//	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G2, bRFRegOffsetMask, 0xf7fb7);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G2, bRFRegOffsetMask, 0x57db7);

	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x808000);

#if 1
	/*path setting*/
	/*Save Original Path Owner, Original GNT*/
	originalPath = ODM_GetMACReg(pDM_Odm, REG_LTECOEX_PATH_CONTROL, bMaskDWord);  /*save 0x70*/
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_CTRL, bMaskDWord, 0x800f0038);
	ODM_delay_ms(1);
	originalGNT = ODM_GetBBReg(pDM_Odm, REG_LTECOEX_READ_DATA, bMaskDWord);  /*save 0x38*/

	/*set GNT_WL=1/GNT_BT=0  and Path owner to WiFi for pause BT traffic*/
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_WRITE_DATA, bMaskDWord, 0x00007700);
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_CTRL, bMaskDWord, 0xc0020038);	/*0x38[15:8] = 0x77*/
	ODM_SetMACReg(pDM_Odm, REG_LTECOEX_PATH_CONTROL, BIT26, 0x1);  /*0x70[26] =1 --> Path Owner to WiFi*/
#endif

	//One shot, path A LOK & IQK
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf9000000);
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf8000000);

	// delay x ms
	ODM_delay_ms(IQK_DELAY_TIME_8703B);
	Ktime = 0;
	while ((ODM_GetBBReg(pDM_Odm, 0xe90, bMaskDWord) == 0) && Ktime < 10) {
		ODM_delay_ms(5);
		Ktime++;
	}

#if 1
	/*path setting*/
	/*Restore GNT_WL/GNT_BT  and Path owner*/
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_WRITE_DATA, bMaskDWord, originalGNT);
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_CTRL, bMaskDWord, 0xc00f0038);
	ODM_SetMACReg(pDM_Odm, REG_LTECOEX_PATH_CONTROL, 0xffffffff, originalPath);

	originalPath = ODM_GetMACReg(pDM_Odm, REG_LTECOEX_PATH_CONTROL, bMaskDWord);  /*save 0x70*/
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_CTRL, bMaskDWord, 0x800f0038);
	ODM_delay_ms(1);
	originalGNT = ODM_GetBBReg(pDM_Odm, REG_LTECOEX_READ_DATA, bMaskDWord);  /*save 0x38*/

/*	DBG_871X("[COEXDBG] REG 0x38 = 0x%08X, 0x70=0x%08x After Get TXIMR !!!!!!!!!!\n", originalGNT, originalPath);*/

#endif

	//leave IQK mode
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);

	// Check failed
	regEAC = ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_A_2, bMaskDWord);
	regE94 = ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_A, bMaskDWord);
	regE9C= ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_A, bMaskDWord);
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xeac = 0x%x\n", regEAC));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe94 = 0x%x, 0xe9c = 0x%x\n", regE94, regE9C));
	/*monitor image power before & after IQK*/
	ODM_RT_TRACE(pDM_Odm , ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe90(before IQK)= 0x%x, 0xe98(afer IQK) = 0x%x\n",
				 ODM_GetBBReg(pDM_Odm, 0xe90, bMaskDWord), ODM_GetBBReg(pDM_Odm, 0xe98, bMaskDWord)));

	//Allen 20131125
	tmp = (regE9C & 0x03FF0000)>>16;
	if ((tmp & 0x200) > 0)
		tmp = 0x400 - tmp;

	if (!(regEAC & BIT28) &&
	   (((regE94 & 0x03FF0000)>>16) != 0x142) &&
	   (((regE9C & 0x03FF0000)>>16) != 0x42))

		result |= 0x01;
	else							//if Tx not OK, ignore Rx
		return result;



	u4tmp = 0x80007C00 | (regE94&0x3FF0000)  | ((regE9C&0x3FF0000) >> 16);
	ODM_SetBBReg(pDM_Odm, rTx_IQK, bMaskDWord, u4tmp);
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe40 = 0x%x u4tmp = 0x%x\n", ODM_GetBBReg(pDM_Odm, rTx_IQK, bMaskDWord), u4tmp));

	//1 RX IQK
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]RX IQK\n"));

	//IQK setting
	ODM_SetBBReg(pDM_Odm, rRx_IQK, bMaskDWord, 0x01004800);

	//path-A IQK setting
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_A, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_A, bMaskDWord, 0x18008c1c);
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_B, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_B, bMaskDWord, 0x38008c1c);

	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_A, bMaskDWord, 0x82110000);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_A, bMaskDWord, 0x28160c1f);
/*	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_A, bMaskDWord, 0x2816001f);*/
	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_B, bMaskDWord, 0x82110000);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_B, bMaskDWord, 0x28110000);

	//LO calibration setting
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Rsp, bMaskDWord, 0x0046a8d1);


	//modify RXIQK mode table
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_WE_LUT, 0x80000, 0x1);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_RCK_OS, bRFRegOffsetMask, 0x30000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G1, bRFRegOffsetMask, 0x00007);
	/*PA off*/
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G2, bRFRegOffsetMask, 0xf7d77);

	/*PA, PAD setting*/
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0xdf, 0x800, 0x1);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0x55, 0x0007f, 0x5);	

	/*enter IQK mode*/
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x808000);

#if 1
	/*path setting*/
	/*Save Original Path Owner, Original GNT*/
	originalPath = ODM_GetMACReg(pDM_Odm, REG_LTECOEX_PATH_CONTROL, bMaskDWord);  /*save 0x70*/
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_CTRL, bMaskDWord, 0x800f0038);
	ODM_delay_ms(1);
	originalGNT = ODM_GetBBReg(pDM_Odm, REG_LTECOEX_READ_DATA, bMaskDWord);  /*save 0x38*/

	/*set GNT_WL=1/GNT_BT=0  and Path owner to WiFi for pause BT traffic*/
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_WRITE_DATA, bMaskDWord, 0x00007700);
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_CTRL, bMaskDWord, 0xc0020038);	/*0x38[15:8] = 0x77*/
	ODM_SetMACReg(pDM_Odm, REG_LTECOEX_PATH_CONTROL, BIT26, 0x1);  /*0x70[26] =1 --> Path Owner to WiFi*/
#endif

	//One shot, path A LOK & IQK
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf9000000);
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf8000000);

	// delay x ms
	ODM_delay_ms(IQK_DELAY_TIME_8703B);
	Ktime = 0;
	while ((ODM_GetBBReg(pDM_Odm, 0xe90, bMaskDWord) == 0) && Ktime < 10) {
		ODM_delay_ms(5);
		Ktime++;
	}

#if 1
	/*path setting*/
	/*Restore GNT_WL/GNT_BT  and Path owner*/
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_WRITE_DATA, bMaskDWord, originalGNT);
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_CTRL, bMaskDWord, 0xc00f0038);
	ODM_SetMACReg(pDM_Odm, REG_LTECOEX_PATH_CONTROL, 0xffffffff, originalPath);

	originalPath = ODM_GetMACReg(pDM_Odm, REG_LTECOEX_PATH_CONTROL, bMaskDWord);  /*save 0x70*/
	ODM_SetBBReg(pDM_Odm, REG_LTECOEX_CTRL, bMaskDWord, 0x800f0038);
	ODM_delay_ms(1);
	originalGNT = ODM_GetBBReg(pDM_Odm, REG_LTECOEX_READ_DATA, bMaskDWord);  /*save 0x38*/

/*	DBG_871X("[COEXDBG] REG 0x38 = 0x%08X, 0x70=0x%08x After Rx IQK !!!!!!!!!!\n", originalGNT, originalPath);*/

#endif

	/*leave IQK mode*/
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
	//	PA/PAD controlled by 0x0
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0xdf, 0x800, 0x0);

	// Check failed
	regEAC = ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_A_2, bMaskDWord);
	regEA4= ODM_GetBBReg(pDM_Odm, rRx_Power_Before_IQK_A_2, bMaskDWord);
	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]0xeac = 0x%x\n", regEAC));
	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xea4 = 0x%x, 0xeac = 0x%x\n", regEA4, regEAC));
	//monitor image power before & after IQK
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xea0(before IQK)= 0x%x, 0xea8(afer IQK) = 0x%x\n",
				 ODM_GetBBReg(pDM_Odm, 0xea0, bMaskDWord), ODM_GetBBReg(pDM_Odm, 0xea8, bMaskDWord)));

	//Allen 20131125
	tmp = (regEAC & 0x03FF0000)>>16;
	if ((tmp & 0x200) > 0)
		tmp = 0x400 - tmp;

	if (!(regEAC & BIT27) &&		/*if Tx is OK, check whether Rx is OK*/
	   (((regEA4 & 0x03FF0000)>>16) != 0x132) &&
	   (((regEAC & 0x03FF0000)>>16) != 0x36) &&
	   (((regEA4 & 0x03FF0000)>>16) < 0x11a) &&
	   (((regEA4 & 0x03FF0000)>>16) > 0xe6) &&
	   (tmp < 0x1a))
		result |= 0x02;
	else							//if Tx not OK, ignore Rx
		ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("Path A Rx IQK fail!!\n"));

	return result;
}


VOID
_PHY_PathAFillIQKMatrix8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN  BOOLEAN    	bIQKOK,
	IN	s4Byte		result[][8],
	IN	u1Byte		final_candidate,
	IN  BOOLEAN		bTxOnly
)
{
	u4Byte	Oldval_0, X, TX0_A, reg, tmp0xc80, tmp0xc94, tmp0xc4c, tmp0xc14, tmp0xca0;
	s4Byte	Y, TX0_C;
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
#endif
#endif
	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);

	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]Path A IQ Calibration %s !\n",(bIQKOK)?"Success":"Failed"));

	if(final_candidate == 0xFF)
		return;

	else if (bIQKOK) {

		Oldval_0 = (ODM_GetBBReg(pDM_Odm, rOFDM0_XATxIQImbalance, bMaskDWord) >> 22) & 0x3FF;

		X = result[final_candidate][0];
		if ((X & 0x00000200) != 0)
			X = X | 0xFFFFFC00;
		TX0_A = (X * Oldval_0) >> 8;
		ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]X = 0x%x, TX0_A = 0x%x, Oldval_0 0x%x\n", X, TX0_A, Oldval_0));
		tmp0xc80 = (ODM_GetBBReg(pDM_Odm, rOFDM0_XATxIQImbalance, bMaskDWord) & 0xfffffc00) | (TX0_A & 0x3ff);
		tmp0xc4c = (((X* Oldval_0>>7) & 0x1) << 31 ) | (ODM_GetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, bMaskDWord) & 0x7fffffff);

		Y = result[final_candidate][1];
		if ((Y & 0x00000200) != 0)
			Y = Y | 0xFFFFFC00;

		//2 Tx IQC
		TX0_C = (Y * Oldval_0) >> 8;
		ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]Y = 0x%x, TX = 0x%x\n", Y, TX0_C));

		tmp0xc94 = (((TX0_C&0x3C0)>>6) << 28) | (ODM_GetBBReg(pDM_Odm, rOFDM0_XCTxAFE, bMaskDWord) & 0x0fffffff);

		pRFCalibrateInfo->TxIQC_8703B[IDX_0xC94][KEY] = rOFDM0_XCTxAFE;
		pRFCalibrateInfo->TxIQC_8703B[IDX_0xC94][VAL] = tmp0xc94;

		tmp0xc80 = (tmp0xc80 & 0xffc0ffff) | (TX0_C & 0x3F)<<16;

		pRFCalibrateInfo->TxIQC_8703B[IDX_0xC80][KEY] = rOFDM0_XATxIQImbalance;
		pRFCalibrateInfo->TxIQC_8703B[IDX_0xC80][VAL] = tmp0xc80;

		tmp0xc4c = (tmp0xc4c & 0xdfffffff) | (((Y* Oldval_0>>7) & 0x1)<<29);

		pRFCalibrateInfo->TxIQC_8703B[IDX_0xC4C][KEY] = rOFDM0_ECCAThreshold;
		pRFCalibrateInfo->TxIQC_8703B[IDX_0xC4C][VAL] = tmp0xc4c;

		if (bTxOnly) {
			ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]_PHY_PathAFillIQKMatrix8703B only Tx OK\n"));

			// <20130226, Kordan> Saving RxIQC, otherwise not initialized.
			pRFCalibrateInfo->RxIQC_8703B[IDX_0xCA0][KEY] = rOFDM0_RxIQExtAnta;
			pRFCalibrateInfo->RxIQC_8703B[IDX_0xCA0][VAL] = 0xfffffff & ODM_GetBBReg(pDM_Odm, rOFDM0_RxIQExtAnta, bMaskDWord);
			pRFCalibrateInfo->RxIQC_8703B[IDX_0xC14][KEY] = rOFDM0_XARxIQImbalance;
			pRFCalibrateInfo->RxIQC_8703B[IDX_0xC14][VAL] = 0x40000100;
			return;
		}

		reg = result[final_candidate][2];
#if (DM_ODM_SUPPORT_TYPE == ODM_AP)
		if (RTL_ABS(reg , 0x100) >= 16)
			reg = 0x100;
#endif

		//2 Rx IQC
		tmp0xc14 = (0x40000100 & 0xfffffc00) | reg;

		reg = result[final_candidate][3] & 0x3F;
		tmp0xc14 = (tmp0xc14 & 0xffff03ff) | (reg << 10);

		pRFCalibrateInfo->RxIQC_8703B[IDX_0xC14][KEY] = rOFDM0_XARxIQImbalance;
		pRFCalibrateInfo->RxIQC_8703B[IDX_0xC14][VAL] = tmp0xc14;

		reg = (result[final_candidate][3] >> 6) & 0xF;
		tmp0xca0 = ODM_GetBBReg(pDM_Odm, rOFDM0_RxIQExtAnta, 0x0fffffff) | (reg << 28);

		pRFCalibrateInfo->RxIQC_8703B[IDX_0xCA0][KEY] = rOFDM0_RxIQExtAnta;
		pRFCalibrateInfo->RxIQC_8703B[IDX_0xCA0][VAL] = tmp0xca0;
	}
}

#if 0
VOID
_PHY_PathBFillIQKMatrix8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN  BOOLEAN   	bIQKOK,
	IN	s4Byte		result[][8],
	IN	u1Byte		final_candidate,
	IN	BOOLEAN		bTxOnly			//do Tx only
	)
{
	u4Byte	Oldval_1, X, TX1_A, reg, tmp0xc80, tmp0xc94, tmp0xc4c, tmp0xc14, tmp0xca0;
	s4Byte	Y, TX1_C;
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
#endif
	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);

	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]Path B IQ Calibration %s !\n",(bIQKOK)?"Success":"Failed"));

	if(final_candidate == 0xFF)
		return;

	else if(bIQKOK)
	{
		Oldval_1 = (ODM_GetBBReg(pDM_Odm, rOFDM0_XATxIQImbalance, bMaskDWord) >> 22) & 0x3FF;


		X = result[final_candidate][4];
		if ((X & 0x00000200) != 0)
			X = X | 0xFFFFFC00;
		TX1_A = (X * Oldval_1) >> 8;
		ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]X = 0x%x, TX1_A = 0x%x\n", X, TX1_A));

		tmp0xc80 = (ODM_GetBBReg(pDM_Odm, rOFDM0_XATxIQImbalance, bMaskDWord) & 0xfffffc00) | (TX1_A & 0x3ff);
		tmp0xc4c = (((X* Oldval_1>>7) & 0x1) << 31 ) | (ODM_GetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, bMaskDWord) & 0x7fffffff);

		Y = result[final_candidate][5];
		if ((Y & 0x00000200) != 0)
			Y = Y | 0xFFFFFC00;

		TX1_C = (Y * Oldval_1) >> 8;
		ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]Y = 0x%x, TX1_C = 0x%x\n", Y, TX1_C));

		/*2 Tx IQC*/

		tmp0xc94 = (((TX1_C&0x3C0)>>6) << 28) | (ODM_GetBBReg(pDM_Odm,rOFDM0_XCTxAFE, bMaskDWord) & 0x0fffffff);

		pRFCalibrateInfo->TxIQC_8703B[PATH_S0][IDX_0xC94][KEY] = rOFDM0_XCTxAFE;
		pRFCalibrateInfo->TxIQC_8703B[PATH_S0][IDX_0xC94][VAL] = tmp0xc94;

        tmp0xc80 = (tmp0xc80 & 0xffc0ffff) | (TX1_C&0x3F)<<16;
		pRFCalibrateInfo->TxIQC_8703B[PATH_S0][IDX_0xC80][KEY] = rOFDM0_XATxIQImbalance;
		pRFCalibrateInfo->TxIQC_8703B[PATH_S0][IDX_0xC80][VAL] = tmp0xc80;

		tmp0xc4c = (tmp0xc4c & 0xdfffffff) | (((Y* Oldval_1>>7) & 0x1)<<29);
		pRFCalibrateInfo->TxIQC_8703B[PATH_S0][IDX_0xC4C][KEY] = rOFDM0_ECCAThreshold;
		pRFCalibrateInfo->TxIQC_8703B[PATH_S0][IDX_0xC4C][VAL] = tmp0xc4c;

		if(bTxOnly) {
			ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]_PHY_PathBFillIQKMatrix8703B only Tx OK\n"));

			pRFCalibrateInfo->RxIQC_8703B[PATH_S0][IDX_0xC14][KEY] = rOFDM0_XARxIQImbalance;
			pRFCalibrateInfo->RxIQC_8703B[PATH_S0][IDX_0xC14][VAL] = 0x40000100;
			pRFCalibrateInfo->RxIQC_8703B[PATH_S0][IDX_0xCA0][KEY] = rOFDM0_RxIQExtAnta;
			pRFCalibrateInfo->RxIQC_8703B[PATH_S0][IDX_0xCA0][VAL] = 0x0fffffff & ODM_GetBBReg(pDM_Odm, rOFDM0_RxIQExtAnta, bMaskDWord);
			return;
		}

		//2 Rx IQC
		reg = result[final_candidate][6];
		tmp0xc14 = (0x40000100 & 0xfffffc00) | reg;

		reg = result[final_candidate][7] & 0x3F;
		tmp0xc14 = (tmp0xc14 & 0xffff03ff) | (reg << 10);

		pRFCalibrateInfo->RxIQC_8703B[PATH_S0][IDX_0xC14][KEY] = rOFDM0_XARxIQImbalance;
		pRFCalibrateInfo->RxIQC_8703B[PATH_S0][IDX_0xC14][VAL] = tmp0xc14;

		reg = (result[final_candidate][7] >> 6) & 0xF;
		tmp0xca0 = ODM_GetBBReg(pDM_Odm, rOFDM0_RxIQExtAnta, 0x0fffffff) | (reg << 28);

		pRFCalibrateInfo->RxIQC_8703B[PATH_S0][IDX_0xCA0][KEY] = rOFDM0_RxIQExtAnta;
		pRFCalibrateInfo->RxIQC_8703B[PATH_S0][IDX_0xCA0][VAL] = tmp0xca0;
	}
}
#endif

BOOLEAN
ODM_SetIQCbyRFpath_8703B(
	IN PDM_ODM_T		pDM_Odm
)
{

	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);


	if ((pRFCalibrateInfo->TxIQC_8703B[IDX_0xC80][VAL] != 0x0) && (pRFCalibrateInfo->RxIQC_8703B[IDX_0xC14][VAL] != 0x0)) {

		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]reload RF IQC!!!\n"));
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]0xc80 = 0x%x!!!\n", pRFCalibrateInfo->TxIQC_8703B[IDX_0xC80][VAL]));
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]0xc14 = 0x%x!!!\n", pRFCalibrateInfo->TxIQC_8703B[IDX_0xC14][VAL]));

		/* TX IQC */
		ODM_SetBBReg(pDM_Odm, pRFCalibrateInfo->TxIQC_8703B[IDX_0xC94][KEY], bMaskH4Bits, (pRFCalibrateInfo->TxIQC_8703B[IDX_0xC94][VAL]>>28));
		ODM_SetBBReg(pDM_Odm, pRFCalibrateInfo->TxIQC_8703B[IDX_0xC80][KEY], bMaskDWord, pRFCalibrateInfo->TxIQC_8703B[IDX_0xC80][VAL]);
		ODM_SetBBReg(pDM_Odm, pRFCalibrateInfo->TxIQC_8703B[IDX_0xC4C][KEY], BIT31, (pRFCalibrateInfo->TxIQC_8703B[IDX_0xC4C][VAL]>>31));
		ODM_SetBBReg(pDM_Odm, pRFCalibrateInfo->TxIQC_8703B[IDX_0xC4C][KEY], BIT29, ((pRFCalibrateInfo->TxIQC_8703B[IDX_0xC4C][VAL] & BIT29)>>29));

		/* RX IQC */
		ODM_SetBBReg(pDM_Odm, pRFCalibrateInfo->RxIQC_8703B[IDX_0xC14][KEY], bMaskDWord, pRFCalibrateInfo->RxIQC_8703B[IDX_0xC14][VAL]);
		ODM_SetBBReg(pDM_Odm, pRFCalibrateInfo->RxIQC_8703B[IDX_0xCA0][KEY], bMaskDWord, pRFCalibrateInfo->RxIQC_8703B[IDX_0xCA0][VAL]);
		return TRUE;
		
	} else {
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]IQC value invalid!!!\n"));
		return FALSE;
	}
}


#if !(DM_ODM_SUPPORT_TYPE & ODM_WIN)
BOOLEAN
ODM_CheckPowerStatus(
	IN	PADAPTER		Adapter)
{
#if 0
		HAL_DATA_TYPE			*pHalData = GET_HAL_DATA(Adapter);
		PDM_ODM_T				pDM_Odm = &pHalData->DM_OutSrc;
		RT_RF_POWER_STATE	rtState;
		PMGNT_INFO				pMgntInfo	= &(Adapter->MgntInfo);

		/* 2011/07/27 MH We are not testing ready~~!! We may fail to get correct value when init sequence.*/
		if (pMgntInfo->init_adpt_in_progress == TRUE) {
			ODM_RT_TRACE(pDM_Odm, COMP_INIT, DBG_LOUD, ("ODM_CheckPowerStatus Return TRUE, due to initadapter"));
			return	TRUE;
		}


		/*	2011/07/19 MH We can not execute tx power tracking/ LLC calibrate or IQK.*/

		Adapter->HalFunc.GetHwRegHandler(Adapter, HW_VAR_RF_STATE, (pu1Byte)(&rtState));
		if (Adapter->bDriverStopped || Adapter->bDriverIsGoingToPnpSetPowerSleep || rtState == eRfOff) {
			ODM_RT_TRACE(pDM_Odm, COMP_INIT, DBG_LOUD, ("ODM_CheckPowerStatus Return FALSE, due to %d/%d/%d\n",
			Adapter->bDriverStopped, Adapter->bDriverIsGoingToPnpSetPowerSleep, rtState));
			return	FALSE;
		}
#endif
	return	TRUE;
}
#endif

VOID
_PHY_SaveADDARegisters8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		ADDAReg,
	IN	pu4Byte		ADDABackup,
	IN	u4Byte		RegisterNum
)
{
	u4Byte	i;
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
#endif

	if (ODM_CheckPowerStatus(pAdapter) == FALSE)
		return;
#endif

//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Save ADDA parameters.\n"));
	for (i = 0; i < RegisterNum; i++)
		ADDABackup[i] = ODM_GetBBReg(pDM_Odm, ADDAReg[i], bMaskDWord);
}


VOID
_PHY_SaveMACRegisters8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		MACReg,
	IN	pu4Byte		MACBackup
)
{
	u4Byte	i;
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
#endif
#endif
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Save MAC parameters.\n"));
	for (i = 0; i < (IQK_MAC_REG_NUM - 1); i++) 
		MACBackup[i] = ODM_Read1Byte(pDM_Odm, MACReg[i]);
	
	MACBackup[i] = ODM_Read4Byte(pDM_Odm, MACReg[i]);

}


VOID
_PHY_ReloadADDARegisters8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		ADDAReg,
	IN	pu4Byte		ADDABackup,
	IN	u4Byte		RegiesterNum
)
{
	u4Byte	i;
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
#endif
#endif

//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Reload ADDA power saving parameters !\n"));
	for (i = 0 ; i < RegiesterNum; i++) 
		ODM_SetBBReg(pDM_Odm, ADDAReg[i], bMaskDWord, ADDABackup[i]);
	
}

VOID
_PHY_ReloadMACRegisters8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		MACReg,
	IN	pu4Byte		MACBackup
)
{
	u4Byte	i;
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
#endif
#endif
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("Reload MAC parameters !\n"));
	for (i = 0 ; i < (IQK_MAC_REG_NUM - 1); i++)
		ODM_Write1Byte(pDM_Odm, MACReg[i], (u1Byte)MACBackup[i]);

	ODM_Write4Byte(pDM_Odm, MACReg[i], MACBackup[i]);
}


VOID
_PHY_PathADDAOn8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		ADDAReg,
	IN	BOOLEAN		isPathAOn
)
{
	u4Byte	pathOn;
	u4Byte	i;
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
#endif
#endif
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("ADDA ON.\n"));

	pathOn = 0x03c00014;


	for (i = 0; i < IQK_ADDA_REG_NUM; i++)
		ODM_SetBBReg(pDM_Odm,ADDAReg[i], bMaskDWord, pathOn);

}

VOID
_PHY_MACSettingCalibration8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		MACReg,
	IN	pu4Byte		MACBackup
)
{
	u4Byte	i = 0;
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
#endif
#endif
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("MAC settings for Calibration.\n"));

	ODM_Write1Byte(pDM_Odm, MACReg[i], 0x3F);

	for (i = 1 ; i < (IQK_MAC_REG_NUM - 1); i++) 
		ODM_Write1Byte(pDM_Odm, MACReg[i], (u1Byte)(MACBackup[i]&(~BIT3)));

	ODM_Write1Byte(pDM_Odm, MACReg[i], (u1Byte)(MACBackup[i]&(~BIT5)));

}

BOOLEAN
phy_SimularityCompare_8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	s4Byte 		result[][8],
	IN	u1Byte		 c1,
	IN	u1Byte		 c2
)
{
	u4Byte		i, j, diff, SimularityBitMap, bound = 0;
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
#endif
#endif
	u1Byte		final_candidate[2] = {0xFF, 0xFF};	//for path A and path B
	BOOLEAN		bResult = TRUE;
//#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
//	BOOLEAN		is2T = IS_92C_SERIAL( pHalData->VersionID);
//#else
	BOOLEAN		is2T = TRUE;
//#endif

	s4Byte tmp1 = 0,tmp2 = 0;

	if(is2T)
		bound = 8;
	else
		bound = 4;

//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("===> IQK:phy_SimularityCompare_8192E c1 %d c2 %d!!!\n", c1, c2));


	SimularityBitMap = 0;

	for (i = 0; i < bound; i++) {

		if ((i == 1) || (i == 3) || (i == 5) || (i == 7)) {
			if((result[c1][i]& 0x00000200) != 0)
				tmp1 = result[c1][i] | 0xFFFFFC00;
			else
				tmp1 = result[c1][i];

			if((result[c2][i]& 0x00000200) != 0)
				tmp2 = result[c2][i] | 0xFFFFFC00;
			else
				tmp2 = result[c2][i];
		} else {
			tmp1 = result[c1][i];
			tmp2 = result[c2][i];
		}

		diff = (tmp1 > tmp2) ? (tmp1 - tmp2) : (tmp2 - tmp1);

		if (diff > MAX_TOLERANCE) {
//			ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("IQK:differnece overflow %d index %d compare1 0x%x compare2 0x%x!!!\n",  diff, i, result[c1][i], result[c2][i]));

			if ((i == 2 || i == 6) && !SimularityBitMap) {
				if(result[c1][i]+result[c1][i+1] == 0)
					final_candidate[(i/4)] = c2;
				else if (result[c2][i]+result[c2][i+1] == 0)
					final_candidate[(i/4)] = c1;
				else
					SimularityBitMap = SimularityBitMap|(1<<i);
			} else
				SimularityBitMap = SimularityBitMap|(1<<i);
		}
	}

//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK:phy_SimularityCompare_8192E SimularityBitMap   %x !!!\n", SimularityBitMap));

	if (SimularityBitMap == 0) {
		for (i = 0; i < (bound/4); i++) {
			if (final_candidate[i] != 0xFF) {
				for (j = i*4; j < (i+1)*4-2; j++)
					result[3][j] = result[final_candidate[i]][j];
				bResult = FALSE;
			}
		}
		return bResult;
	} else {

		if (!(SimularityBitMap & 0x03)) {		/*path A TX OK*/
			for (i = 0; i < 2; i++)
				result[3][i] = result[c1][i];
		}

		if (!(SimularityBitMap & 0x0c)) {		/*path A RX OK*/
			for (i = 2; i < 4; i++)
				result[3][i] = result[c1][i];
		}

		if (!(SimularityBitMap & 0x30)) {	/*path B TX OK*/
			for (i = 4; i < 6; i++)
				result[3][i] = result[c1][i];

		}

		if (!(SimularityBitMap & 0xc0)) {	/*path B RX OK*/
			for (i = 6; i < 8; i++)
				result[3][i] = result[c1][i];
		}
		return FALSE;
	}
}



VOID
phy_IQCalibrate_8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	s4Byte 		result[][8],
	IN	u1Byte		t
)
{
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
#endif
#endif
	u4Byte			i;
	u1Byte			PathAOK, PathBOK;
	u1Byte 			tmp0xc50 = (u1Byte)ODM_GetBBReg(pDM_Odm, 0xC50, bMaskByte0);
	u1Byte			tmp0xc58 = (u1Byte)ODM_GetBBReg(pDM_Odm, 0xC58, bMaskByte0);
	u4Byte			ADDA_REG[IQK_ADDA_REG_NUM] = {
		rFPGA0_XCD_SwitchControl,		rBlue_Tooth,
		rRx_Wait_CCA,		rTx_CCK_RFON,
		rTx_CCK_BBON,		rTx_OFDM_RFON,
		rTx_OFDM_BBON,		rTx_To_Rx,
		rTx_To_Tx,		rRx_CCK,
		rRx_OFDM,		rRx_Wait_RIFS,
		rRx_TO_Rx,		rStandby,
		rSleep,		rPMPD_ANAEN
	};
	u4Byte			IQK_MAC_REG[IQK_MAC_REG_NUM] = {
		REG_TXPAUSE,		REG_BCN_CTRL,
		REG_BCN_CTRL_1,	REG_GPIO_MUXCFG
	};

	/*since 92C & 92D have the different define in IQK_BB_REG*/
	u4Byte	IQK_BB_REG_92C[IQK_BB_REG_NUM] = {
		rOFDM0_TRxPathEnable,		rOFDM0_TRMuxPar,
		rFPGA0_XCD_RFInterfaceSW,	rConfig_AntA,	rConfig_AntB,
		rFPGA0_XAB_RFInterfaceSW,	rFPGA0_XA_RFInterfaceOE,
		rFPGA0_XB_RFInterfaceOE,		rCCK0_AFESetting
	};



#if (DM_ODM_SUPPORT_TYPE & (ODM_AP))
	u4Byte	retryCount = 2;
#else
#if MP_DRIVER
	const u4Byte	retryCount = 1;
#else
	const u4Byte	retryCount = 2;
#endif
#endif

	/* Note: IQ calibration must be performed after loading*/
	/*PHY_REG.txt , and radio_a, radio_b.txt*/

	//u4Byte bbvalue;

#if (DM_ODM_SUPPORT_TYPE & (ODM_AP))
#ifdef MP_TEST
	if (pDM_Odm->priv->pshare->rf_ft_var.mp_specific)
		retryCount = 9;
#endif
#endif


	if (t == 0) {
//	 	 bbvalue = ODM_GetBBReg(pDM_Odm, rFPGA0_RFMOD, bMaskDWord);
//			RT_DISP(FINIT, INIT_IQK, ("phy_IQCalibrate_8188E()==>0x%08x\n",bbvalue));

		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]IQ Calibration for %d times\n", t));

		/* Save ADDA parameters, turn Path A ADDA on*/
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		_PHY_SaveADDARegisters8703B(pAdapter, ADDA_REG, pDM_Odm->RFCalibrateInfo.ADDA_backup, IQK_ADDA_REG_NUM);
		_PHY_SaveMACRegisters8703B(pAdapter, IQK_MAC_REG, pDM_Odm->RFCalibrateInfo.IQK_MAC_backup);
		_PHY_SaveADDARegisters8703B(pAdapter, IQK_BB_REG_92C, pDM_Odm->RFCalibrateInfo.IQK_BB_backup, IQK_BB_REG_NUM);
#else
		_PHY_SaveADDARegisters8703B(pDM_Odm, ADDA_REG, pDM_Odm->RFCalibrateInfo.ADDA_backup, IQK_ADDA_REG_NUM);
		_PHY_SaveMACRegisters8703B(pDM_Odm, IQK_MAC_REG, pDM_Odm->RFCalibrateInfo.IQK_MAC_backup);
		_PHY_SaveADDARegisters8703B(pDM_Odm, IQK_BB_REG_92C, pDM_Odm->RFCalibrateInfo.IQK_BB_backup, IQK_BB_REG_NUM);
#endif
	}
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]IQ Calibration for %d times\n", t));

#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)

	_PHY_PathADDAOn8703B(pAdapter, ADDA_REG, TRUE);
#else
	_PHY_PathADDAOn8703B(pDM_Odm, ADDA_REG, TRUE);
#endif


	//MAC settings
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_PHY_MACSettingCalibration8703B(pAdapter, IQK_MAC_REG, pDM_Odm->RFCalibrateInfo.IQK_MAC_backup);
#else
	_PHY_MACSettingCalibration8703B(pDM_Odm, IQK_MAC_REG, pDM_Odm->RFCalibrateInfo.IQK_MAC_backup);
#endif

	//BB setting
	/*ODM_SetBBReg(pDM_Odm, rFPGA0_RFMOD, BIT24, 0x00);*/
	ODM_SetBBReg(pDM_Odm, rCCK0_AFESetting, 0x0f000000, 0xf);
	ODM_SetBBReg(pDM_Odm, rOFDM0_TRxPathEnable, bMaskDWord, 0x03a05600);
	ODM_SetBBReg(pDM_Odm, rOFDM0_TRMuxPar, bMaskDWord, 0x000800e4);
	ODM_SetBBReg(pDM_Odm, rFPGA0_XCD_RFInterfaceSW, bMaskDWord, 0x25204000);

//path A TX IQK
#if 1

	for (i = 0 ; i < retryCount ; i++) {
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		PathAOK = phy_PathA_IQK_8703B(pAdapter);
#else
		PathAOK = phy_PathA_IQK_8703B(pDM_Odm);
#endif

		if (PathAOK == 0x01) {
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]Tx IQK Success!!\n"));
			result[t][0] = (ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_A, bMaskDWord)&0x3FF0000)>>16;
			result[t][1] = (ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_A, bMaskDWord)&0x3FF0000)>>16;
			break;
		}

	}
#endif

//path A RXIQK
#if 1

	for (i = 0 ; i < retryCount ; i++) {
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		PathAOK = phy_PathA_RxIQK8703B(pAdapter);
#else
		PathAOK = phy_PathA_RxIQK8703B(pDM_Odm);
#endif
		if (PathAOK == 0x03) {
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]Rx IQK Success!!\n"));
//			result[t][0] = (ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_A, bMaskDWord)&0x3FF0000)>>16;
//			result[t][1] = (ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_A, bMaskDWord)&0x3FF0000)>>16;
			result[t][2] = (ODM_GetBBReg(pDM_Odm, rRx_Power_Before_IQK_A_2, bMaskDWord)&0x3FF0000)>>16;
			result[t][3] = (ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_A_2, bMaskDWord)&0x3FF0000)>>16;
			break;
		} else {
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]Rx IQK Fail!!\n"));
		}
	}

	if (0x00 == PathAOK)
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]IQK failed!!\n"));

#endif


//path B TX IQK
#if 0

#if MP_DRIVER != 1
	if ((*pDM_Odm->pIs1Antenna == FALSE) || ((*pDM_Odm->pIs1Antenna == TRUE) && (*pDM_Odm->pRFDefaultPath == 1))
	   || (pDM_Odm->SupportInterface == ODM_ITRF_USB))
#endif
	{

		for (i = 0 ; i < retryCount ; i++) {
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			PathBOK = phy_PathB_IQK_8703B(pAdapter);
#else
			PathBOK = phy_PathB_IQK_8703B(pDM_Odm);
#endif
//		if(PathBOK == 0x03){
			if (PathBOK == 0x01) {
				ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]S0 Tx IQK Success!!\n"));
				result[t][4] = (ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_A, bMaskDWord)&0x3FF0000)>>16;
				result[t][5] = (ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_A, bMaskDWord)&0x3FF0000)>>16;
				break;
			}

		}
#endif

//path B RX IQK
#if 0

		for (i = 0 ; i < retryCount ; i++) {
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			PathBOK = phy_PathB_RxIQK8703B(pAdapter);
#else
			PathBOK = phy_PathB_RxIQK8703B(pDM_Odm);
#endif
			if (PathBOK == 0x03) {
				ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]S0 Rx IQK Success!!\n"));
//				result[t][0] = (ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_A, bMaskDWord)&0x3FF0000)>>16;
//				result[t][1] = (ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_A, bMaskDWord)&0x3FF0000)>>16;
				result[t][6] = (ODM_GetBBReg(pDM_Odm, rRx_Power_Before_IQK_A_2, bMaskDWord)&0x3FF0000)>>16;
				result[t][7] = (ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_A_2, bMaskDWord)&0x3FF0000)>>16;
				break;
				
			} else
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]S0 Rx IQK Fail!!\n"));

		}



		if (0x00 == PathBOK)
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]S0 IQK failed!!\n"));

	}
#endif

	//Back to BB mode, load original value
	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]IQK:Back to BB mode, load original value!\n"));
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);

	if (t != 0) {
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)

		/* Reload ADDA power saving parameters*/
		_PHY_ReloadADDARegisters8703B(pAdapter, ADDA_REG, pDM_Odm->RFCalibrateInfo.ADDA_backup, IQK_ADDA_REG_NUM);

		/* Reload MAC parameters*/
		_PHY_ReloadMACRegisters8703B(pAdapter, IQK_MAC_REG, pDM_Odm->RFCalibrateInfo.IQK_MAC_backup);

		_PHY_ReloadADDARegisters8703B(pAdapter, IQK_BB_REG_92C, pDM_Odm->RFCalibrateInfo.IQK_BB_backup, IQK_BB_REG_NUM);
#else
		/* Reload ADDA power saving parameters*/
		_PHY_ReloadADDARegisters8703B(pDM_Odm, ADDA_REG, pDM_Odm->RFCalibrateInfo.ADDA_backup, IQK_ADDA_REG_NUM);

		/* Reload MAC parameters*/
		_PHY_ReloadMACRegisters8703B(pDM_Odm, IQK_MAC_REG, pDM_Odm->RFCalibrateInfo.IQK_MAC_backup);

		_PHY_ReloadADDARegisters8703B(pDM_Odm, IQK_BB_REG_92C, pDM_Odm->RFCalibrateInfo.IQK_BB_backup, IQK_BB_REG_NUM);
#endif

		//Allen initial gain 0xc50
		// Restore RX initial gain
		ODM_SetBBReg(pDM_Odm, 0xc50, bMaskByte0, 0x50);
		ODM_SetBBReg(pDM_Odm, 0xc50, bMaskByte0, tmp0xc50);

		//load 0xe30 IQC default value
		ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_A, bMaskDWord, 0x01008c00);
		ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_A, bMaskDWord, 0x01008c00);

	}
	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]phy_IQCalibrate_8703B() <==\n"));

}


VOID
phy_LCCalibrate_8703B(
	IN PDM_ODM_T		pDM_Odm,
	IN	BOOLEAN		is2T
)
{
	u1Byte	tmpReg;
	u4Byte	RF_Amode=0, RF_Bmode=0, LC_Cal;
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	PADAPTER pAdapter = pDM_Odm->Adapter;
#endif

	//Check continuous TX and Packet TX
	tmpReg = ODM_Read1Byte(pDM_Odm, 0xd03);

	if ((tmpReg&0x70) != 0)			/*Deal with contisuous TX case*/
		ODM_Write1Byte(pDM_Odm, 0xd03, tmpReg&0x8F);	//disable all continuous TX
	else							// Deal with Packet TX case
		ODM_Write1Byte(pDM_Odm, REG_TXPAUSE, 0xFF);			// block all queues

	if ((tmpReg&0x70) != 0) {
		//1. Read original RF mode
		//Path-A
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		RF_Amode = PHY_QueryRFReg(pAdapter, ODM_RF_PATH_A, RF_AC, bMask12Bits);

		//Path-B
		if(is2T)
			RF_Bmode = PHY_QueryRFReg(pAdapter, ODM_RF_PATH_B, RF_AC, bMask12Bits);
#else
		RF_Amode = ODM_GetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_AC, bMask12Bits);

		//Path-B
		if(is2T)
			RF_Bmode = ODM_GetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_AC, bMask12Bits);
#endif

		//2. Set RF mode = standby mode
		//Path-A
		ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_AC, bMask12Bits, (RF_Amode&0x8FFFF)|0x10000);

		//Path-B
		if(is2T)
			ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_AC, bMask12Bits, (RF_Bmode&0x8FFFF)|0x10000);
	}

	//3. Read RF reg18
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	LC_Cal = PHY_QueryRFReg(pAdapter, ODM_RF_PATH_A, RF_CHNLBW, bMask12Bits);
#else
	LC_Cal = ODM_GetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_CHNLBW, bMask12Bits);
#endif

	//4. Set LC calibration begin	bit15
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0xB0, bRFRegOffsetMask, 0xDFBE0); // LDO ON
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_CHNLBW, bMask12Bits, LC_Cal|0x08000);

	ODM_delay_ms(LCK_DELAY_TIME_8703B);

	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0xB0, bRFRegOffsetMask, 0xDFFE0); // LDO OFF

    /*disable below for 8703B by jerrychou 20150720
	// Channel 10 LC calibration issue for 8703Bs with 26M xtal
	if (pDM_Odm->SupportInterface == ODM_ITRF_SDIO && pDM_Odm->PackageType >= 0x2)
		ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_CHNLBW, bMask12Bits, LC_Cal);
       */

	//Restore original situation
	if ((tmpReg&0x70) != 0) {	/*Deal with contisuous TX case*/
		//Path-A
		ODM_Write1Byte(pDM_Odm, 0xd03, tmpReg);
		ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_AC, bMask12Bits, RF_Amode);

		//Path-B
		if(is2T)
			ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_AC, bMask12Bits, RF_Bmode);
	} else { /* Deal with Packet TX case*/
		ODM_Write1Byte(pDM_Odm, REG_TXPAUSE, 0x00);
	}
}

/* IQK version:V0.4*/
/* 1. add coex. related setting*/

VOID
PHY_IQCalibrate_8703B(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	BOOLEAN 	bReCovery
)
{
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
#else  /* (DM_ODM_SUPPORT_TYPE == ODM_CE)*/
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
#endif

#if (MP_DRIVER == 1)
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PMPT_CONTEXT	pMptCtx = &(pAdapter->MptCtx);
#else/* (DM_ODM_SUPPORT_TYPE == ODM_CE)*/
	PMPT_CONTEXT	pMptCtx = &(pAdapter->mppriv.MptCtx);
#endif
#endif/*(MP_DRIVER == 1)*/

	u1Byte			u1bTmp;
	u2Byte			count=0;
#endif

	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);

	s4Byte			result[4][8];	//last is final result
	u1Byte			i, final_candidate, Indexforchannel;
	BOOLEAN			bPathAOK, bPathBOK;
	s4Byte			RegE94, RegE9C, RegEA4, RegEAC, RegEB4, RegEBC, RegEC4, RegECC, RegTmp = 0;
	BOOLEAN			is12simular, is13simular, is23simular;
	BOOLEAN 		bStartContTx = FALSE, bSingleTone = FALSE, bCarrierSuppression = FALSE;
	u4Byte			IQK_BB_REG_92C[IQK_BB_REG_NUM] = {
		rOFDM0_XARxIQImbalance,		rOFDM0_XBRxIQImbalance,
		rOFDM0_ECCAThreshold,		rOFDM0_AGCRSSITable,
		rOFDM0_XATxIQImbalance,		rOFDM0_XBTxIQImbalance,
		rOFDM0_XCTxAFE,			rOFDM0_XDTxAFE,
		rOFDM0_RxIQExtAnta
	};
	BOOLEAN			bReloadIQK = FALSE;

#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN|ODM_CE) )
	if (ODM_CheckPowerStatus(pAdapter) == FALSE)
		return;
#else
	prtl8192cd_priv	priv = pDM_Odm->priv;

#ifdef MP_TEST
	if (priv->pshare->rf_ft_var.mp_specific) {
		if((OPMODE & WIFI_MP_CTX_PACKET) || (OPMODE & WIFI_MP_CTX_ST))
			return;
	}
#endif

	if(priv->pshare->IQK_88E_done)
		bReCovery= 1;
	priv->pshare->IQK_88E_done = 1;
#endif


#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	if (!(pDM_Odm->SupportAbility & ODM_RF_CALIBRATION)) 
		return;

#endif


	if (pDM_Odm->mp_mode == TRUE) {
#if MP_DRIVER == 1
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
		/* <VincentL, 131231> Add to determine IQK ON/OFF in certain case, Suggested by Cheng.*/
		if (!pHalData->IQK_MP_Switch)
			return;
#endif

		bStartContTx = pMptCtx->bStartContTx;
		bSingleTone = pMptCtx->bSingleTone;
		bCarrierSuppression = pMptCtx->bCarrierSuppression;

		/* 20120213<Kordan> Turn on when continuous Tx to pass lab testing. (required by Edlu)*/
		if (bSingleTone || bCarrierSuppression)
			return;
#endif
	}

#if DISABLE_BB_RF
	return;
#endif


	if (pDM_Odm->RFCalibrateInfo.bIQKInProgress)
		return;

#if (DM_ODM_SUPPORT_TYPE & (ODM_CE|ODM_AP))
	if(bReCovery)
#else//for ODM_WIN
	if(bReCovery && (!pAdapter->bInHctTest))  //YJ,add for PowerTest,120405
#endif
	{
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("[IQK]PHY_IQCalibrate_8703B: Return due to bReCovery!\n"));
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		_PHY_ReloadADDARegisters8703B(pAdapter, IQK_BB_REG_92C, pDM_Odm->RFCalibrateInfo.IQK_BB_backup_recover, 9);
#else
		_PHY_ReloadADDARegisters8703B(pDM_Odm, IQK_BB_REG_92C, pDM_Odm->RFCalibrateInfo.IQK_BB_backup_recover, 9);
#endif
		return;
	}


	if (pDM_Odm->mp_mode == FALSE) {
#if MP_DRIVER != 1
//check if IQK had been done before!!

		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK] 0xc80 = 0x%x\n", pRFCalibrateInfo->TxIQC_8703B[IDX_0xC80][VAL]));
		if (ODM_SetIQCbyRFpath_8703B(pDM_Odm)) {
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]IQK value is reloaded!!!\n"));
			bReloadIQK = TRUE;
		}

		if(bReloadIQK)
			return;
#endif
	}
	/*Check & wait if BT is doing IQK*/

	if (pDM_Odm->mp_mode == FALSE) {
#if MP_DRIVER != 1
		// Set H2C cmd to inform FW (enable).
		SetFwWiFiCalibrationCmd(pAdapter, 1);
		// Check 0x1e6
		count=0;
		u1bTmp = PlatformEFIORead1Byte(pAdapter, 0x1e6);
		while (u1bTmp != 0x1 && count < 1000) {
			PlatformStallExecution(10);
			u1bTmp = PlatformEFIORead1Byte(pAdapter, 0x1e6);
			count++;
		}

		if (count >= 1000) 
			RT_TRACE(COMP_INIT, DBG_LOUD, ("[IQK]Polling 0x1e6 to 1 for WiFi calibration H2C cmd FAIL! count(%d)", count));


		// Wait BT IQK finished.
		// polling 0x1e7[0]=1 or 300ms timeout
		u1bTmp = PlatformEFIORead1Byte(pAdapter, 0x1e7);
		while ((!(u1bTmp&BIT0)) && count < 6000) {
			PlatformStallExecution(50);
			u1bTmp = PlatformEFIORead1Byte(pAdapter, 0x1e7);
			count++;
		}
#endif
	}

	//IQK start!!!!!!!!!!

	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]IQK:Start!!!\n"));
	ODM_AcquireSpinLock(pDM_Odm, RT_IQK_SPINLOCK);
	pDM_Odm->RFCalibrateInfo.bIQKInProgress = TRUE;
	ODM_ReleaseSpinLock(pDM_Odm, RT_IQK_SPINLOCK);


	for (i = 0; i < 8; i++) {
		result[0][i] = 0;
		result[1][i] = 0;
		result[2][i] = 0;
		result[3][i] = 0;
	}
	final_candidate = 0xff;
	bPathAOK = FALSE;
	bPathBOK = FALSE;
	is12simular = FALSE;
	is23simular = FALSE;
	is13simular = FALSE;


	for (i = 0; i < 3; i++) {
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		phy_IQCalibrate_8703B(pAdapter, result, i);
#else
		phy_IQCalibrate_8703B(pDM_Odm, result, i);
#endif


		if (i == 1) {
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			is12simular = phy_SimularityCompare_8703B(pAdapter, result, 0, 1);
#else
			is12simular = phy_SimularityCompare_8703B(pDM_Odm, result, 0, 1);
#endif
			if (is12simular) {
				final_candidate = 0;
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]IQK: is12simular final_candidate is %x\n", final_candidate));
				break;
			}
		}

		if (i == 2) {
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			is13simular = phy_SimularityCompare_8703B(pAdapter, result, 0, 2);
#else
			is13simular = phy_SimularityCompare_8703B(pDM_Odm, result, 0, 2);
#endif
			if (is13simular) {
				final_candidate = 0;
				ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]IQK: is13simular final_candidate is %x\n",final_candidate));

				break;
			}
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			is23simular = phy_SimularityCompare_8703B(pAdapter, result, 1, 2);
#else
			is23simular = phy_SimularityCompare_8703B(pDM_Odm, result, 1, 2);
#endif
			if (is23simular) {
				final_candidate = 1;
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]IQK: is23simular final_candidate is %x\n", final_candidate));
			} else {
				for(i = 0; i < 8; i++)
					RegTmp += result[3][i];

				if(RegTmp != 0)
					final_candidate = 3;
				else
					final_candidate = 0xFF;
			}
		}
	}
/*	RT_TRACE(COMP_INIT,DBG_LOUD,("Release Mutex in IQCalibrate\n"));*/

	for (i = 0; i < 4; i++) {
		RegE94 = result[i][0];
		RegE9C = result[i][1];
		RegEA4 = result[i][2];
		RegEAC = result[i][3];
		RegEB4 = result[i][4];
		RegEBC = result[i][5];
		RegEC4 = result[i][6];
		RegECC = result[i][7];
		ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]IQK: RegE94=%x RegE9C=%x RegEA4=%x RegEAC=%x RegEB4=%x RegEBC=%x RegEC4=%x RegECC=%x\n ", RegE94, RegE9C, RegEA4, RegEAC, RegEB4, RegEBC, RegEC4, RegECC));
	}

	if (final_candidate != 0xff) {
		pDM_Odm->RFCalibrateInfo.RegE94 = RegE94 = result[final_candidate][0];
		pDM_Odm->RFCalibrateInfo.RegE9C = RegE9C = result[final_candidate][1];
		RegEA4 = result[final_candidate][2];
		RegEAC = result[final_candidate][3];
		pDM_Odm->RFCalibrateInfo.RegEB4 = RegEB4 = result[final_candidate][4];
		pDM_Odm->RFCalibrateInfo.RegEBC = RegEBC = result[final_candidate][5];
		RegEC4 = result[final_candidate][6];
		RegECC = result[final_candidate][7];
		ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]IQK: final_candidate is %x\n",final_candidate));
		ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]IQK: RegE94=%x RegE9C=%x RegEA4=%x RegEAC=%x RegEB4=%x RegEBC=%x RegEC4=%x RegECC=%x\n ", RegE94, RegE9C, RegEA4, RegEAC, RegEB4, RegEBC, RegEC4, RegECC));
		bPathAOK = bPathBOK = TRUE;
	} else {
		ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]IQK: FAIL use default value\n"));
		pDM_Odm->RFCalibrateInfo.RegE94 = pDM_Odm->RFCalibrateInfo.RegEB4 = 0x100;	//X default value
		pDM_Odm->RFCalibrateInfo.RegE9C = pDM_Odm->RFCalibrateInfo.RegEBC = 0x0;		//Y default value
	}

	// fill IQK matrix
	if (RegE94 != 0)
		_PHY_PathAFillIQKMatrix8703B(pAdapter, bPathAOK, result, final_candidate, (RegEA4 == 0));
//	if (RegEB4 != 0)
//		_PHY_PathBFillIQKMatrix8703B(pAdapter, bPathBOK, result, final_candidate, (RegEC4 == 0));

#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	Indexforchannel = ODM_GetRightChnlPlaceforIQK(pHalData->CurrentChannel);
#else
	Indexforchannel = 0;
#endif

//To Fix BSOD when final_candidate is 0xff
//by sherry 20120321
	if (final_candidate < 4) {
		for(i = 0; i < IQK_Matrix_REG_NUM; i++)
			pDM_Odm->RFCalibrateInfo.IQKMatrixRegSetting[Indexforchannel].Value[0][i] = result[final_candidate][i];
		pDM_Odm->RFCalibrateInfo.IQKMatrixRegSetting[Indexforchannel].bIQKDone = TRUE;
	}
	//RT_DISP(FINIT, INIT_IQK, ("\nIQK OK Indexforchannel %d.\n", Indexforchannel));
	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]\nIQK OK Indexforchannel %d.\n", Indexforchannel));


#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_PHY_SaveADDARegisters8703B(pAdapter, IQK_BB_REG_92C, pDM_Odm->RFCalibrateInfo.IQK_BB_backup_recover, 9);
#else
	_PHY_SaveADDARegisters8703B(pDM_Odm, IQK_BB_REG_92C, pDM_Odm->RFCalibrateInfo.IQK_BB_backup_recover, IQK_BB_REG_NUM);
#endif

//fill IQK register
	ODM_SetIQCbyRFpath_8703B(pDM_Odm);

	if (pDM_Odm->mp_mode == FALSE) {
#if MP_DRIVER != 1
		// Set H2C cmd to inform FW (disable).
		SetFwWiFiCalibrationCmd(pAdapter, 0);

		// Check 0x1e6
		count = 0;
		u1bTmp = PlatformEFIORead1Byte(pAdapter, 0x1e6);
		while (u1bTmp != 0 && count < 1000) {
			PlatformStallExecution(10);
			u1bTmp = PlatformEFIORead1Byte(pAdapter, 0x1e6);
			count++;
		}

		if (count >= 1000) 
			RT_TRACE(COMP_INIT, DBG_LOUD, ("[IQK]Polling 0x1e6 to 0 for WiFi calibration H2C cmd FAIL! count(%d)", count));

#endif
	}

	ODM_AcquireSpinLock(pDM_Odm, RT_IQK_SPINLOCK);
	pDM_Odm->RFCalibrateInfo.bIQKInProgress = FALSE;
	ODM_ReleaseSpinLock(pDM_Odm, RT_IQK_SPINLOCK);

	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD,  ("[IQK]IQK finished\n"));
}


VOID
PHY_LCCalibrate_8703B(
	PVOID		pDM_VOID
)
{
	BOOLEAN 		bStartContTx = FALSE, bSingleTone = FALSE, bCarrierSuppression = FALSE;
	u4Byte			timeout = 2000, timecount = 0;
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	PADAPTER	pAdapter = pDM_Odm->Adapter;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);

#if (MP_DRIVER == 1)
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PMPT_CONTEXT	pMptCtx = &(pAdapter->MptCtx);
#else/* (DM_ODM_SUPPORT_TYPE == ODM_CE)*/
	PMPT_CONTEXT	pMptCtx = &(pAdapter->mppriv.MptCtx);
#endif
#endif/*(MP_DRIVER == 1)*/
#endif




#if MP_DRIVER == 1
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	// <VincentL, 140319> Add to Disable Default LCK when Cont Tx, For Lab Test Usage.
	if (!pHalData->IQK_MP_Switch)
		return;
#endif

	bStartContTx = pMptCtx->bStartContTx;
	bSingleTone = pMptCtx->bSingleTone;
	bCarrierSuppression = pMptCtx->bCarrierSuppression;
#endif


#if DISABLE_BB_RF
	return;
#endif

#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	if (!(pDM_Odm->SupportAbility & ODM_RF_CALIBRATION)) 
		return;

#endif
	// 20120213<Kordan> Turn on when continuous Tx to pass lab testing. (required by Edlu)
	if(bSingleTone || bCarrierSuppression)
		return;

	while (*(pDM_Odm->pbScanInProcess) && timecount < timeout) {
		ODM_delay_ms(50);
		timecount += 50;
	}

	pDM_Odm->RFCalibrateInfo.bLCKInProgress = TRUE;


	phy_LCCalibrate_8703B(pDM_Odm, FALSE);


	pDM_Odm->RFCalibrateInfo.bLCKInProgress = FALSE;

	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("LCK:Finish!!!interface %d\n", pDM_Odm->InterfaceIndex));

}


