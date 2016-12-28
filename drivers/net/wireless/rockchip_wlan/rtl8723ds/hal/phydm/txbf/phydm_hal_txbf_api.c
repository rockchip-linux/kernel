#include "mp_precomp.h"
#include "phydm_precomp.h"

#if (BEAMFORMING_SUPPORT == 1)

/*Add by YuChen for 8822B MU-MIMO API*/

/*this function is only used for BFer*/
u1Byte
phydm_get_ndpa_rate(
	IN PVOID		pDM_VOID
	)
{
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;
	u1Byte		NDPARate = ODM_RATE6M;

	if (pDM_Odm->RSSI_Min >= 30)	/*link RSSI > 30%*/
		NDPARate = ODM_RATE24M;
	else if (pDM_Odm->RSSI_Min <= 25)
		NDPARate = ODM_RATE6M;

	ODM_RT_TRACE(pDM_Odm, PHYDM_COMP_TXBF, ODM_DBG_TRACE, ("[%s] NDPARate = 0x%x\n", __func__, NDPARate));

	return NDPARate;

}

/*this function is only used for BFer*/
u1Byte
phydm_get_beamforming_sounding_info(
	IN PVOID		pDM_VOID,
	IN pu2Byte	Troughput,
	IN u1Byte	Total_BFee_Num,
	IN pu1Byte	TxRate
	)
{
	u1Byte	idx = 0;
	u1Byte	soundingdecision = 0xff;
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;

	for (idx = 0; idx < Total_BFee_Num; idx++) {
		if (((TxRate[idx] >= ODM_RATEVHTSS3MCS7) && (TxRate[idx] <= ODM_RATEVHTSS3MCS9)))
			soundingdecision = soundingdecision & ~(1<<idx);
	}

	for (idx = 0; idx < Total_BFee_Num; idx++) {
		if (Troughput[idx] <= 10)
			soundingdecision = soundingdecision & ~(1<<idx);
	}

	ODM_RT_TRACE(pDM_Odm, PHYDM_COMP_TXBF, ODM_DBG_TRACE, ("[%s] soundingdecision = 0x%x\n", __func__, soundingdecision));

	return soundingdecision;

}



#endif

