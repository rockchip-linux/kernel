/******************************************************************************
 *
 * Copyright(c) 2013 Realtek Corporation. All rights reserved.
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

#include <rtw_odm.h>
#include <hal_data.h>



/* set ODM_CMNINFO_IC_TYPE based on chip_type */
void rtw_odm_init_ic_type(_adapter *adapter)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	struct PHY_DM_STRUCT *odm = &hal_data->odmpriv;
	u4Byte ic_type = chip_type_to_odm_ic_type(rtw_get_chip_type(adapter));

	rtw_warn_on(!ic_type);

	odm_cmn_info_init(odm, ODM_CMNINFO_IC_TYPE, ic_type);
}

inline void rtw_odm_set_force_igi_lb(_adapter *adapter, u8 lb)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);

	hal_data->u1ForcedIgiLb = lb;
}

inline u8 rtw_odm_get_force_igi_lb(_adapter *adapter)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);

	return hal_data->u1ForcedIgiLb;
}

void rtw_odm_adaptivity_ver_msg(void *sel, _adapter *adapter)
{
	RTW_PRINT_SEL(sel, "ADAPTIVITY_VERSION "ADAPTIVITY_VERSION"\n");
}

#define RTW_ADAPTIVITY_EN_DISABLE 0
#define RTW_ADAPTIVITY_EN_ENABLE 1

void rtw_odm_adaptivity_en_msg(void *sel, _adapter *adapter)
{
	struct registry_priv *regsty = &adapter->registrypriv;
	struct mlme_priv *mlme = &adapter->mlmepriv;
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	struct PHY_DM_STRUCT *odm = &hal_data->odmpriv;

	RTW_PRINT_SEL(sel, "RTW_ADAPTIVITY_EN_");

	if (regsty->adaptivity_en == RTW_ADAPTIVITY_EN_DISABLE)
		_RTW_PRINT_SEL(sel, "DISABLE\n");
	else if (regsty->adaptivity_en == RTW_ADAPTIVITY_EN_ENABLE)
		_RTW_PRINT_SEL(sel, "ENABLE\n");
	else
		_RTW_PRINT_SEL(sel, "INVALID\n");
}

#define RTW_ADAPTIVITY_MODE_NORMAL 0
#define RTW_ADAPTIVITY_MODE_CARRIER_SENSE 1

void rtw_odm_adaptivity_mode_msg(void *sel, _adapter *adapter)
{
	struct registry_priv *regsty = &adapter->registrypriv;

	RTW_PRINT_SEL(sel, "RTW_ADAPTIVITY_MODE_");

	if (regsty->adaptivity_mode == RTW_ADAPTIVITY_MODE_NORMAL)
		_RTW_PRINT_SEL(sel, "NORMAL\n");
	else if (regsty->adaptivity_mode == RTW_ADAPTIVITY_MODE_CARRIER_SENSE)
		_RTW_PRINT_SEL(sel, "CARRIER_SENSE\n");
	else
		_RTW_PRINT_SEL(sel, "INVALID\n");
}

#define RTW_ADAPTIVITY_DML_DISABLE 0
#define RTW_ADAPTIVITY_DML_ENABLE 1

void rtw_odm_adaptivity_dml_msg(void *sel, _adapter *adapter)
{
	struct registry_priv *regsty = &adapter->registrypriv;

	RTW_PRINT_SEL(sel, "RTW_ADAPTIVITY_DML_");

	if (regsty->adaptivity_dml == RTW_ADAPTIVITY_DML_DISABLE)
		_RTW_PRINT_SEL(sel, "DISABLE\n");
	else if (regsty->adaptivity_dml == RTW_ADAPTIVITY_DML_ENABLE)
		_RTW_PRINT_SEL(sel, "ENABLE\n");
	else
		_RTW_PRINT_SEL(sel, "INVALID\n");
}

void rtw_odm_adaptivity_dc_backoff_msg(void *sel, _adapter *adapter)
{
	struct registry_priv *regsty = &adapter->registrypriv;

	RTW_PRINT_SEL(sel, "RTW_ADAPTIVITY_DC_BACKOFF:%u\n", regsty->adaptivity_dc_backoff);
}

void rtw_odm_adaptivity_config_msg(void *sel, _adapter *adapter)
{
	rtw_odm_adaptivity_ver_msg(sel, adapter);
	rtw_odm_adaptivity_en_msg(sel, adapter);
	rtw_odm_adaptivity_mode_msg(sel, adapter);
	rtw_odm_adaptivity_dml_msg(sel, adapter);
	rtw_odm_adaptivity_dc_backoff_msg(sel, adapter);
}

bool rtw_odm_adaptivity_needed(_adapter *adapter)
{
	struct registry_priv *regsty = &adapter->registrypriv;
	struct mlme_priv *mlme = &adapter->mlmepriv;
	bool ret = _FALSE;

	if (regsty->adaptivity_en == RTW_ADAPTIVITY_EN_ENABLE)
		ret = _TRUE;

	return ret;
}

void rtw_odm_adaptivity_parm_msg(void *sel, _adapter *adapter)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(adapter);
	struct PHY_DM_STRUCT *odm = &pHalData->odmpriv;

	rtw_odm_adaptivity_config_msg(sel, adapter);

	RTW_PRINT_SEL(sel, "%10s %16s %16s %22s %12s\n"
		, "th_l2h_ini", "th_edcca_hl_diff", "th_l2h_ini_mode2", "th_edcca_hl_diff_mode2", "edcca_enable");
	RTW_PRINT_SEL(sel, "0x%-8x %-16d 0x%-14x %-22d %-12d\n"
		, (u8)odm->th_l2h_ini
		, odm->th_edcca_hl_diff
		, (u8)odm->th_l2h_ini_mode2
		, odm->th_edcca_hl_diff_mode2
		, odm->edcca_enable
	);

	RTW_PRINT_SEL(sel, "%15s %9s\n", "AdapEnableState", "Adap_Flag");
	RTW_PRINT_SEL(sel, "%-15x %-9x\n"
		, odm->adaptivity_enable
		, odm->adaptivity_flag
	);
}

void rtw_odm_adaptivity_parm_set(_adapter *adapter, s8 th_l2h_ini, s8 th_edcca_hl_diff, s8 th_l2h_ini_mode2, s8 th_edcca_hl_diff_mode2, u8 edcca_enable)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(adapter);
	struct PHY_DM_STRUCT *odm = &pHalData->odmpriv;

	odm->th_l2h_ini = th_l2h_ini;
	odm->th_edcca_hl_diff = th_edcca_hl_diff;
	odm->th_l2h_ini_mode2 = th_l2h_ini_mode2;
	odm->th_edcca_hl_diff_mode2 = th_edcca_hl_diff_mode2;
	odm->edcca_enable = edcca_enable;
}

void rtw_odm_get_perpkt_rssi(void *sel, _adapter *adapter)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	struct PHY_DM_STRUCT *odm = &(hal_data->odmpriv);

	RTW_PRINT_SEL(sel, "rx_rate = %s, RSSI_A = %d(%%), RSSI_B = %d(%%)\n",
		      HDATA_RATE(odm->rx_rate), odm->RSSI_A, odm->RSSI_B);
}


void rtw_odm_acquirespinlock(_adapter *adapter,	enum rt_spinlock_type type)
{
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(adapter);
	_irqL irqL;

	switch (type) {
	case RT_IQK_SPINLOCK:
		_enter_critical_bh(&pHalData->IQKSpinLock, &irqL);
	default:
		break;
	}
}

void rtw_odm_releasespinlock(_adapter *adapter,	enum rt_spinlock_type type)
{
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(adapter);
	_irqL irqL;

	switch (type) {
	case RT_IQK_SPINLOCK:
		_exit_critical_bh(&pHalData->IQKSpinLock, &irqL);
	default:
		break;
	}
}

#ifdef CONFIG_DFS_MASTER
inline u8 rtw_odm_get_dfs_domain(_adapter *adapter)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	struct PHY_DM_STRUCT *pDM_Odm = &(hal_data->odmpriv);

	return pDM_Odm->dfs_region_domain;
}

inline VOID rtw_odm_radar_detect_reset(_adapter *adapter)
{
	phydm_radar_detect_reset(GET_ODM(adapter));
}

inline VOID rtw_odm_radar_detect_disable(_adapter *adapter)
{
	phydm_radar_detect_disable(GET_ODM(adapter));
}

/* called after ch, bw is set */
inline VOID rtw_odm_radar_detect_enable(_adapter *adapter)
{
	phydm_radar_detect_enable(GET_ODM(adapter));
}

inline BOOLEAN rtw_odm_radar_detect(_adapter *adapter)
{
	return phydm_radar_detect(GET_ODM(adapter));
}
#endif /* CONFIG_DFS_MASTER */
