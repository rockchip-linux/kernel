#include "mp_precomp.h"
#include "phydm_precomp.h"
#include "rtl8197f/Hal8197FPhyReg.h"
#if ((RTL8197F_SUPPORT == 1)||(RTL8822B_SUPPORT == 1))
#include "WlanHAL/HalMac88XX/halmac_reg2.h"
#else
#include "WlanHAL/HalHeader/HalComReg.h"
#endif

#if (PHYDM_LA_MODE_SUPPORT == 1)

#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
BOOLEAN
ADCSmp_BufferAllocate(
	IN	PADAPTER			Adapter,
	IN	PRT_ADCSMP			AdcSmp
	)
{
	PRT_ADCSMP_STRING	ADCSmpBuf = &(AdcSmp->ADCSmpBuf);

	if (ADCSmpBuf->Length == 0) {
		if (PlatformAllocateMemoryWithZero(Adapter, (void **)&(ADCSmpBuf->Octet), 0x10000) == RT_STATUS_SUCCESS)
			ADCSmpBuf->Length = 0x10000;
		else
			return FALSE;
	}

	return TRUE;
}
#endif

VOID
ADCSmp_GetTxPktBuf(
	IN		PVOID			pDM_VOID,
	IN	PRT_ADCSMP_STRING	ADCSmpBuf
	)
{
	PDM_ODM_T				pDM_Odm = (PDM_ODM_T)pDM_VOID;
	u4Byte				i = 0, value32, DataL = 0, DataH = 0;
	u4Byte				Addr, Finish_Addr;
	u4Byte				End_Addr = (ADCSmpBuf->start_pos  + ADCSmpBuf->buffer_size)-1;	/*End_Addr = 0x3ffff;*/
	BOOLEAN				bRoundUp;
	static u4Byte			page = 0xFF;


	PlatformZeroMemory(ADCSmpBuf->Octet, ADCSmpBuf->Length);

	ODM_Write1Byte(pDM_Odm, REG_PKT_BUFF_ACCESS_CTRL, 0x69);
	/*PlatformEFIOWrite1Byte(Adapter, REG_PKT_BUFF_ACCESS_CTRL_8814A, 0x69);*/
	/*0x106[7:0]=0x69: access TXPKT Buffer*/
	/*			0xA5: access RXPKT Buffer*/
	/*			0x7F: access TXREPORT buffer*/

	DbgPrint("%s\n", __func__);

	value32 = ODM_Read4Byte(pDM_Odm, REG_IQ_DUMP);
	bRoundUp = (BOOLEAN)((value32 & BIT31) >> 31);
	Finish_Addr = (value32 & 0x7FFF0000) >> 16;	/*Reg7C0[30:16]: finish addr (unit: 8byte)*/

	if (bRoundUp)
		Addr = (Finish_Addr+1)<<3;
	else	
		Addr = ADCSmpBuf->start_pos;

	DbgPrint("bRoundUp = %d, Finish_Addr=0x%x, value32=0x%x\n", bRoundUp, Finish_Addr, value32);
	DbgPrint("End_Addr = %x, ADCSmpBuf->start_pos = 0x%x, ADCSmpBuf->buffer_size = 0x%x\n", End_Addr, ADCSmpBuf->start_pos, ADCSmpBuf->buffer_size);

#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	watchdog_stop(pDM_Odm->priv);
#endif

	if (pDM_Odm->SupportICType & ODM_RTL8197F) {
		for (Addr = 0x0, i = 0; Addr < End_Addr; Addr += 8, i += 2) {	/*64K byte*/
			if ((Addr&0xfff) == 0)
				ODM_Write2Byte(pDM_Odm, REG_PKTBUF_DBG_CTRL, 0x780+(Addr >> 12));
			DataL = ODM_Read4Byte(pDM_Odm, 0x8000+(Addr&0xfff));
			DataH = ODM_Read4Byte(pDM_Odm, 0x8000+(Addr&0xfff)+4);

			DbgPrint("%08x%08x\n", DataH, DataL);		
		}
	} else {
		while (Addr != (Finish_Addr<<3)) {
			if (page != (Addr >> 12)) {
				/*Reg140=0x780+(Addr>>12), Addr=0x30~0x3F, total 16 pages*/
				page = (Addr >> 12);
				ODM_Write2Byte(pDM_Odm, REG_PKTBUF_DBG_CTRL, 0x780+page);
			}
			/*pDataL = 0x8000+(Addr&0xfff);*/
			DataL = ODM_Read4Byte(pDM_Odm, 0x8000+(Addr&0xfff));
			DataH = ODM_Read4Byte(pDM_Odm, 0x8000+(Addr&0xfff)+4);

			/*ADCSmpBuf->Octet[i] = DataH;*/
			/*ADCSmpBuf->Octet[i+1] = DataL;*/
			/*DbgPrint("%08x%08x\n", ADCSmpBuf->Octet[i], ADCSmpBuf->Octet[i+1]);*/
			DbgPrint("%08x%08x\n", DataH, DataL);
			i = i + 2;
			
			if ((Addr+8) >= End_Addr)
				Addr = ADCSmpBuf->start_pos;
			else
				Addr = Addr + 8;
		}
	}
	
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	watchdog_resume(pDM_Odm->priv);
#endif
}	


VOID
ADCSmp_Start(
	IN		PVOID			pDM_VOID,
	IN		PRT_ADCSMP		AdcSmp
	)
{
	PDM_ODM_T				pDM_Odm = (PDM_ODM_T)pDM_VOID;
	u1Byte					tmpU1b;
	PRT_ADCSMP_STRING		Buffer = &(AdcSmp->ADCSmpBuf);
	RT_ADCSMP_TRIG_SIG_SEL	TrigSigSel = AdcSmp->ADCSmpTrigSigSel;
	u1Byte					backup_DMA;

	DbgPrint("%s\n", __func__);

	if (pDM_Odm->SupportICType & ODM_RTL8197F)
		ODM_SetBBReg(pDM_Odm, r_dma_trigger_8197F, 0xf00, AdcSmp->ADCSmpDmaDataSigSel);	/*0x9A0[11:8]*/
	else
		ODM_SetBBReg(pDM_Odm , ODM_ADC_TRIGGER_Jaguar2, 0xf00, AdcSmp->ADCSmpDmaDataSigSel);	/*0x95C[11:8]*/

	ODM_Write1Byte(pDM_Odm, REG_IQ_DUMP+1, AdcSmp->ADCSmpTriggerTime);


	if (pDM_Odm->SupportICType & ODM_RTL8197F)
		ODM_SetBBReg(pDM_Odm, r_reset_cfo_rpt_ctrl_8197F, BIT26, 0x1);
	else {	/*for 8814A and 8822B?*/
		ODM_Write1Byte(pDM_Odm, 0x198c, 0x7);
		ODM_Write1Byte(pDM_Odm, 0x8b4, 0x80);
	}
	
	if (AdcSmp->ADCSmpTrigSel == ADCSMP_MAC_TRIG) {	/* trigger by MAC*/
		if (TrigSigSel == ADCSMP_TRIG_REG) {			/* manual trigger 0x7C0[5] = 0 -> 1*/
			ODM_Write1Byte(pDM_Odm, REG_IQ_DUMP, 0xCB);		/*0x7C0[7:0]=8'b1100_1011*/
			ODM_Write1Byte(pDM_Odm, REG_IQ_DUMP, 0xEB);		/*0x7C0[7:0]=8'b1110_1011*/
		} else if (TrigSigSel == ADCSMP_TRIG_CCA)
			ODM_Write1Byte(pDM_Odm, REG_IQ_DUMP, 0x8B);		/*0x7C0[7:0]=8'b1000_1011*/
		else if (TrigSigSel == ADCSMP_TRIG_CRCFAIL)
			ODM_Write1Byte(pDM_Odm, REG_IQ_DUMP, 0x4B);		/*0x7C0[7:0]=8'b0100_1011*/
		else if (TrigSigSel == ADCSMP_TRIG_CRCOK)
			ODM_Write1Byte(pDM_Odm, REG_IQ_DUMP, 0x0B);		/*0x7C0[7:0]=8'b0000_1011*/
	} else {												/*trigger by BB*/
		if (pDM_Odm->SupportICType & ODM_RTL8197F)
			ODM_SetBBReg(pDM_Odm, r_dma_trigger_8197F, 0x1f, TrigSigSel);	/*0x9A0[4:0]*/
		else
			ODM_SetBBReg(pDM_Odm , ODM_ADC_TRIGGER_Jaguar2, 0x1f, TrigSigSel);	/*0x95C[4:0], 0x1F: trigger by CCA*/
		ODM_Write1Byte(pDM_Odm, REG_IQ_DUMP, 0x03);	/*0x7C0[7:0]=8'b0000_0011*/
	}
	
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	watchdog_stop(pDM_Odm->priv);
#endif

	/*Polling*/
	do {
		tmpU1b = ODM_Read1Byte(pDM_Odm, REG_IQ_DUMP);

		if (AdcSmp->ADCSmpState != ADCSMP_STATE_SET) {
			DbgPrint("ADCSmpState != ADCSMP_STATE_SET\n");
			break;
			
		} else if (tmpU1b & BIT1) {
			ODM_delay_us(AdcSmp->ADCSmpPollingTime);
			continue;
		} else {
			DbgPrint("%s Query OK\n", __func__);
			if (pDM_Odm->SupportICType & ODM_RTL8197F)
				ODM_SetBBReg(pDM_Odm, REG_IQ_DUMP, BIT0, 0x0);
			break;
		}
	} while (1);
	
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	watchdog_resume(pDM_Odm->priv);
#if (RTL8197F_SUPPORT == 1)
	if (pDM_Odm->SupportICType & ODM_RTL8197F) {
		/*Stop DMA*/
		backup_DMA = ODM_GetMACReg(pDM_Odm, 0x300, bMaskLWord);
		ODM_SetMACReg(pDM_Odm, 0x300, 0x7fff, backup_DMA|0x7fff);
		
		/*move LA mode content from IMEM to TxPktBuffer 
			Src : OCPBASE_IMEM 0x00000000
			Dest : OCPBASE_TXBUF 0x18780000
			Len : 64K*/
		GET_HAL_INTERFACE(pDM_Odm->priv)->InitDDMAHandler(pDM_Odm->priv, OCPBASE_IMEM, OCPBASE_TXBUF, 0x10000);
	}
#endif
#endif

	if (AdcSmp->ADCSmpState == ADCSMP_STATE_SET)
		ADCSmp_GetTxPktBuf(pDM_Odm, Buffer);

#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	if (pDM_Odm->SupportICType & ODM_RTL8197F) 
		ODM_SetMACReg(pDM_Odm, 0x300, 0x7fff, backup_DMA);	/*Resume DMA*/
#endif

#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
	if (AdcSmp->ADCSmpState == ADCSMP_STATE_SET)
		AdcSmp->ADCSmpState = ADCSMP_STATE_QUERY;
#endif

	DbgPrint("%s Status %d\n", __func__, AdcSmp->ADCSmpState);
}

#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
VOID
ADCSmpWorkItemCallback(
	IN	PVOID	pContext
	)
{
	PADAPTER			Adapter = (PADAPTER)pContext;
	PHAL_DATA_TYPE		pHalData = GET_HAL_DATA(Adapter);
	PRT_ADCSMP			AdcSmp = &(pHalData->ADCSmp);

	ADCSmp_Start(Adapter, AdcSmp); 
}
#endif

VOID
ADCSmp_Set(
	IN		PVOID			pDM_VOID,
	IN	RT_ADCSMP_TRIG_SEL		TrigSel,
	IN	RT_ADCSMP_TRIG_SIG_SEL	TrigSigSel,
	IN	u1Byte					DmaDataSigSel,
	IN	u1Byte					TriggerTime,
	IN	u2Byte					PollingTime
	)
{
	PDM_ODM_T				pDM_Odm = (PDM_ODM_T)pDM_VOID;
	BOOLEAN				retValue = TRUE;

	PRT_ADCSMP			AdcSmp = &(pDM_Odm->adcsmp);
/*	
	DbgPrint("%s\n ADCSmpState %d ADCSmpTrigSig %d ADCSmpTrigSigSel %d\n", 
			__FUNCTION__, AdcSmp->ADCSmpState, TrigSel, TrigSigSel);

	DbgPrint("ADCSmpDmaDataSigSel %d, ADCSmpTriggerTime %d ADCSmpPollingTime %d\n", 
			DmaDataSigSel, TriggerTime, PollingTime);
*/
	AdcSmp->ADCSmpTrigSel = TrigSel;
	AdcSmp->ADCSmpTrigSigSel = TrigSigSel;
	AdcSmp->ADCSmpDmaDataSigSel = DmaDataSigSel;
	AdcSmp->ADCSmpTriggerTime = TriggerTime;
	AdcSmp->ADCSmpPollingTime = PollingTime;

#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
	if (AdcSmp->ADCSmpState != ADCSMP_STATE_IDLE)
		retValue = FALSE;
	else if (AdcSmp->ADCSmpBuf.Length == 0)
		retValue = ADCSmp_BufferAllocate(pDM_Odm, AdcSmp);
#endif

	if (retValue) {
		AdcSmp->ADCSmpState = ADCSMP_STATE_SET;
#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
		PlatformScheduleWorkItem(&(pHalData->ADCSmpWorkItem));
#elif (DM_ODM_SUPPORT_TYPE & ODM_AP)
		ADCSmp_Start(pDM_Odm, AdcSmp); 
#endif
	}	

	DbgPrint("ADCSmpState %d Return Status %d\n", AdcSmp->ADCSmpState, retValue);
}

#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
RT_STATUS
ADCSmp_Query(
	IN	PADAPTER			Adapter,
	IN	ULONG				InformationBufferLength, 
	OUT	PVOID				InformationBuffer, 
	OUT	PULONG				BytesWritten
	)
{
	RT_STATUS			retStatus = RT_STATUS_SUCCESS;
	PHAL_DATA_TYPE		pHalData = GET_HAL_DATA(Adapter);
	PRT_ADCSMP			AdcSmp = &(pHalData->ADCSmp);
	PRT_ADCSMP_STRING	ADCSmpBuf = &(AdcSmp->ADCSmpBuf);

	DbgPrint("%s ADCSmpState %d", __func__, AdcSmp->ADCSmpState);

	if (InformationBufferLength != ADCSmpBuf->buffer_size)	{
		*BytesWritten = 0;
		retStatus = RT_STATUS_RESOURCE;
	} else if (ADCSmpBuf->Length != ADCSmpBuf->buffer_size) {
		*BytesWritten = 0;
		retStatus = RT_STATUS_RESOURCE;
	} else if (AdcSmp->ADCSmpState != ADCSMP_STATE_QUERY) {
		*BytesWritten = 0;
		retStatus = RT_STATUS_PENDING;
	} else {
		PlatformMoveMemory(InformationBuffer, ADCSmpBuf->Octet, ADCSmpBuf->buffer_size);
		*BytesWritten = ADCSmpBuf->buffer_size;

		AdcSmp->ADCSmpState = ADCSMP_STATE_IDLE;
	}

	DbgPrint("Return Status %d\n", retStatus);

	return retStatus;
}
#endif

VOID
ADCSmp_Stop(
	IN		PVOID			pDM_VOID
	)
{
	PDM_ODM_T			pDM_Odm = (PDM_ODM_T)pDM_VOID;	
	PRT_ADCSMP			AdcSmp = &(pDM_Odm->adcsmp);

	AdcSmp->ADCSmpState = ADCSMP_STATE_IDLE;

	DbgPrint("%s status %d\n", __func__, AdcSmp->ADCSmpState);
}




#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
u1Byte ADC_buffer[0x20000];
#endif

VOID
ADCSmp_Init(
	IN		PVOID			pDM_VOID
	)
{
	PDM_ODM_T			pDM_Odm = (PDM_ODM_T)pDM_VOID;	
	PRT_ADCSMP			AdcSmp = &(pDM_Odm->adcsmp);
	PRT_ADCSMP_STRING	ADCSmpBuf = &(AdcSmp->ADCSmpBuf);

	AdcSmp->ADCSmpState = ADCSMP_STATE_IDLE;

	if (pDM_Odm->SupportICType & ODM_RTL8814A) {
		ADCSmpBuf->start_pos = 0x30000;
		ADCSmpBuf->buffer_size = 0x10000;	
	} else if (pDM_Odm->SupportICType & ODM_RTL8822B) {
		ADCSmpBuf->start_pos = 0x20000;
		ADCSmpBuf->buffer_size = 0x20000;	
	} else if (pDM_Odm->SupportICType & ODM_RTL8197F) {
		ADCSmpBuf->start_pos = 0x00000;
		ADCSmpBuf->buffer_size = 0x10000;	
	}
	
#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
	PlatformInitializeWorkItem(
		Adapter,
		&(pHalData->ADCSmpWorkItem), 
		(RT_WORKITEM_CALL_BACK)ADCSmpWorkItemCallback, 
		(PVOID)Adapter,
		"ADCSmpWorkItem");
#endif
}

#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
VOID
ADCSmp_DeInit(
	PADAPTER		Adapter
	)
{
	PHAL_DATA_TYPE		pHalData = GET_HAL_DATA(Adapter);
	PRT_ADCSMP			AdcSmp = &(pHalData->ADCSmp);
	PRT_ADCSMP_STRING	ADCSmpBuf = &(AdcSmp->ADCSmpBuf);

	ADCSmp_Stop(Adapter);

	PlatformFreeWorkItem(&(pHalData->ADCSmpWorkItem));

	if (ADCSmpBuf->Length != 0x0) {
		PlatformFreeMemory(ADCSmpBuf->Octet, ADCSmpBuf->Length);
		ADCSmpBuf->Length = 0x0;
	}
}	


VOID
Dump_MAC(
	PADAPTER		Adapter
	)
{

	u4Byte	Addr = 0;
	
	for (Addr = 0; Addr < 0x1A3D; Addr++)
		DbgPrint("%04x %04x\n", Addr, PlatformEFIORead4Byte(Adapter, Addr));
}


VOID
Dump_BB(
	PADAPTER		Adapter
	)
{
	u4Byte	Addr = 0;
	
	for (Addr = 0; Addr < 0x1AFD; Addr++)
		DbgPrint("%04x %04x\n", Addr, PHY_QueryBBReg(Adapter, Addr, bMaskDWord));
}


VOID
Dump_RF(
	PADAPTER		Adapter
	)
{
	u1Byte	Addr = 0, Path = 0;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	
	for (Path = ODM_RF_PATH_A; Path < pHalData->NumTotalRFPath; Path++) {
		for (Addr = 0; Addr < 0xF6; Addr++)
			DbgPrint("%04x %04x\n", Addr, PHY_QueryRFReg(Adapter, Path, Addr, bRFRegOffsetMask));
	}
}
#endif

#endif

