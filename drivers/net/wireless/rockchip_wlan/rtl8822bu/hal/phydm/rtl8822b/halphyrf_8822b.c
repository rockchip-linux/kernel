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

#if (RTL8822B_SUPPORT == 1)

bool
get_mix_mode_tx_agc_bb_swing_offset_8822b(
	void				*p_dm_void,
	enum pwrtrack_method	method,
	u8				rf_path,
	u8				tx_power_index_offest
)
{
	struct PHY_DM_STRUCT		*p_dm_odm	=	(struct PHY_DM_STRUCT *)p_dm_void;
	struct odm_rf_calibration_structure	*p_rf_calibrate_info = &(p_dm_odm->rf_calibrate_info);

	u8	bb_swing_upper_bound = p_rf_calibrate_info->default_ofdm_index + 10;
	u8	bb_swing_lower_bound = 0;

	s8	tx_agc_index = 0;
	u8	tx_bb_swing_index = p_rf_calibrate_info->default_ofdm_index;

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
		("Path_%d p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path]=%d, tx_power_index_offest=%d\n",
		rf_path, p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path], tx_power_index_offest));

	if (tx_power_index_offest > 0XF)
		tx_power_index_offest = 0XF;

	if (p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path] >= 0 && p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path] <= tx_power_index_offest) {
		tx_agc_index = p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path];
		tx_bb_swing_index = p_rf_calibrate_info->default_ofdm_index;
	} else if (p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path] > tx_power_index_offest) {
		tx_agc_index = tx_power_index_offest;
		p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path] = p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path] - tx_power_index_offest;
		tx_bb_swing_index = p_rf_calibrate_info->default_ofdm_index + p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path];

		if (tx_bb_swing_index > bb_swing_upper_bound)
			tx_bb_swing_index = bb_swing_upper_bound;
	} else {
		tx_agc_index = 0;

		if (p_rf_calibrate_info->default_ofdm_index > (p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path] * (-1)))
			tx_bb_swing_index = p_rf_calibrate_info->default_ofdm_index + p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path];
		else
			tx_bb_swing_index = bb_swing_lower_bound;

		if (tx_bb_swing_index <  bb_swing_lower_bound)
			tx_bb_swing_index = bb_swing_lower_bound;
	}

	p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path] = tx_agc_index;
	p_rf_calibrate_info->bb_swing_idx_ofdm[rf_path] = tx_bb_swing_index;

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
		("MixMode Offset Path_%d   p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path]=%d   p_rf_calibrate_info->bb_swing_idx_ofdm[rf_path]=%d   tx_power_index_offest=%d\n",
		rf_path, p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path], p_rf_calibrate_info->bb_swing_idx_ofdm[rf_path], tx_power_index_offest));

	return true;
}


void
odm_tx_pwr_track_set_pwr8822b(
	void				*p_dm_void,
	enum pwrtrack_method	method,
	u8				rf_path,
	u8				channel_mapped_index
)
{

#if 0
	struct PHY_DM_STRUCT	*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _ADAPTER	*adapter = p_dm_odm->adapter;
	HAL_DATA_TYPE	*p_hal_data = GET_HAL_DATA(adapter);
	struct odm_rf_calibration_structure	*p_rf_calibrate_info = &(p_dm_odm->rf_calibrate_info);
	u8			channel  = p_hal_data->current_channel;
	u8			band_width  = p_hal_data->current_channel_bw;
	u8			tx_power_index = 0;
	u8			tx_rate = 0xFF;
	enum rt_status		status = RT_STATUS_SUCCESS;

	PHALMAC_PWR_TRACKING_OPTION p_pwr_tracking_opt = &(p_rf_calibrate_info->HALMAC_PWR_TRACKING_INFO);

	if (p_dm_odm->mp_mode == true) {
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
#if (MP_DRIVER == 1)
		PMPT_CONTEXT p_mpt_ctx = &(adapter->mpt_ctx);

		tx_rate = mpt_to_mgnt_rate(p_mpt_ctx->mpt_rate_index);
#endif
#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
		PMPT_CONTEXT p_mpt_ctx = &(adapter->mppriv.mpt_ctx);

		tx_rate = mpt_to_mgnt_rate(p_mpt_ctx->mpt_rate_index);
#endif
#endif
	} else {
		u16	rate	 = *(p_dm_odm->p_forced_data_rate);

		if (!rate) { /*auto rate*/
#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
			tx_rate = adapter->hal_func.get_hw_rate_from_m_rate_handler(p_dm_odm->tx_rate);
#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
			if (p_dm_odm->number_linked_client != 0)
				tx_rate = hw_rate_to_m_rate(p_dm_odm->tx_rate);
#endif
		} else   /*force rate*/
			tx_rate = (u8) rate;
	}

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("Call:%s tx_rate=0x%X\n", __func__, tx_rate));

	tx_power_index = phy_get_tx_power_index(adapter, (enum odm_rf_radio_path_e) rf_path, tx_rate, band_width, channel);

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
		("type=%d   tx_power_index=%d	 p_rf_calibrate_info->absolute_ofdm_swing_idx=%d   p_rf_calibrate_info->default_ofdm_index=%d   rf_path=%d\n", method, tx_power_index, p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path], p_rf_calibrate_info->default_ofdm_index, rf_path));

	p_pwr_tracking_opt->type = method;
	p_pwr_tracking_opt->bbswing_index = p_rf_calibrate_info->default_ofdm_index;
	p_pwr_tracking_opt->pwr_tracking_para[rf_path].enable = 1;
	p_pwr_tracking_opt->pwr_tracking_para[rf_path].tx_pwr_index = tx_power_index;
	p_pwr_tracking_opt->pwr_tracking_para[rf_path].pwr_tracking_offset_value = p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path];
	p_pwr_tracking_opt->pwr_tracking_para[rf_path].tssi_value = 0;


	if (rf_path == (MAX_PATH_NUM_8822B - 1)) {
		status = hal_mac_send_power_tracking_info(&GET_HAL_MAC_INFO(adapter), p_pwr_tracking_opt);

		if (status == RT_STATUS_SUCCESS) {
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
				     ("path A  0xC94=0x%X   0xC1C=0x%X\n",
				odm_get_bb_reg(p_dm_odm, 0xC94, BIT(29) | BIT(28) | BIT(27) | BIT(26) | BIT(25)),
				odm_get_bb_reg(p_dm_odm, 0xC1C, 0xFFE00000)
				     ));
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
				     ("path B  0xE94=0x%X   0xE1C=0x%X\n",
				odm_get_bb_reg(p_dm_odm, 0xE94, BIT(29) | BIT(28) | BIT(27) | BIT(26) | BIT(25)),
				odm_get_bb_reg(p_dm_odm, 0xE1C, 0xFFE00000)
				     ));
		} else {
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
				("Power Tracking to FW Fail ret code = %d\n", status));
		}
	}

#endif

	struct PHY_DM_STRUCT		*p_dm_odm		= (struct PHY_DM_STRUCT *)p_dm_void;
	struct odm_rf_calibration_structure	*p_rf_calibrate_info	= &(p_dm_odm->rf_calibrate_info);
	u8			tx_power_index_offest = 0;
	u8			tx_power_index = 0;


#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
	struct _ADAPTER		*adapter = p_dm_odm->adapter;
	HAL_DATA_TYPE	*p_hal_data = GET_HAL_DATA(adapter);
	u8			channel  = p_hal_data->current_channel;
	u8			band_width  = p_hal_data->current_channel_bw;
	u8			tx_rate = 0xFF;

	if (p_dm_odm->mp_mode == true) {
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
#if (MP_DRIVER == 1)
		PMPT_CONTEXT p_mpt_ctx = &(adapter->mpt_ctx);

		tx_rate = mpt_to_mgnt_rate(p_mpt_ctx->mpt_rate_index);
#endif
#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
		PMPT_CONTEXT p_mpt_ctx = &(adapter->mppriv.mpt_ctx);

		tx_rate = mpt_to_mgnt_rate(p_mpt_ctx->mpt_rate_index);
#endif
#endif
	} else {
		u16	rate	 = *(p_dm_odm->p_forced_data_rate);

		if (!rate) { /*auto rate*/
#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
			tx_rate = adapter->hal_func.get_hw_rate_from_m_rate_handler(p_dm_odm->tx_rate);
#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
			if (p_dm_odm->number_linked_client != 0)
				tx_rate = hw_rate_to_m_rate(p_dm_odm->tx_rate);
#endif
		} else   /*force rate*/
			tx_rate = (u8) rate;
	}

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("Call:%s tx_rate=0x%X\n", __func__, tx_rate));

#endif

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
		("pRF->default_ofdm_index=%d   pRF->default_cck_index=%d\n", p_rf_calibrate_info->default_ofdm_index, p_rf_calibrate_info->default_cck_index));

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
		("pRF->absolute_ofdm_swing_idx=%d   pRF->remnant_ofdm_swing_idx=%d   pRF->absolute_cck_swing_idx=%d   pRF->remnant_cck_swing_idx=%d   rf_path=%d\n",
		p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path], p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path], p_rf_calibrate_info->absolute_cck_swing_idx[rf_path], p_rf_calibrate_info->remnant_cck_swing_idx, rf_path));

#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
	tx_power_index = phy_get_tx_power_index(adapter, (enum odm_rf_radio_path_e) rf_path, tx_rate, band_width, channel);
#else
	tx_power_index = config_phydm_read_txagc_8822b(p_dm_odm, rf_path, 0x04); /*0x04(TX_AGC_OFDM_6M)*/
#endif

	if (tx_power_index >= 63)
		tx_power_index = 63;

	tx_power_index_offest = 63 - tx_power_index;

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
		("tx_power_index=%d tx_power_index_offest=%d rf_path=%d\n", tx_power_index, tx_power_index_offest, rf_path));

	if (method == BBSWING) {	/*use for mp driver clean power tracking status*/
		switch (rf_path) {
		case ODM_RF_PATH_A:
			odm_set_bb_reg(p_dm_odm, 0xC94, (BIT(29) | BIT(28) | BIT(27) | BIT(26) | BIT(25)), p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path]);
			odm_set_bb_reg(p_dm_odm, REG_A_TX_SCALE_JAGUAR, 0xFFE00000, tx_scaling_table_jaguar[p_rf_calibrate_info->bb_swing_idx_ofdm[rf_path]]);
			break;
		case ODM_RF_PATH_B:
			odm_set_bb_reg(p_dm_odm, 0xE94, (BIT(29) | BIT(28) | BIT(27) | BIT(26) | BIT(25)), p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path]);
			odm_set_bb_reg(p_dm_odm, REG_B_TX_SCALE_JAGUAR, 0xFFE00000, tx_scaling_table_jaguar[p_rf_calibrate_info->bb_swing_idx_ofdm[rf_path]]);
			break;

		default:
			break;
		}
	} else if (method == MIX_MODE) {
		switch (rf_path) {
		case ODM_RF_PATH_A:
			get_mix_mode_tx_agc_bb_swing_offset_8822b(p_dm_odm, method, rf_path, tx_power_index_offest);
			odm_set_bb_reg(p_dm_odm, 0xC94, (BIT(29) | BIT(28) | BIT(27) | BIT(26) | BIT(25)), p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path]);
			odm_set_bb_reg(p_dm_odm, REG_A_TX_SCALE_JAGUAR, 0xFFE00000, tx_scaling_table_jaguar[p_rf_calibrate_info->bb_swing_idx_ofdm[rf_path]]);

			ODM_RT_TRACE(p_dm_odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
				("TXAGC(0xC94)=0x%x BBSwing(0xc1c)=0x%x BBSwingIndex=%d rf_path=%d\n",
				odm_get_bb_reg(p_dm_odm, 0xC94, (BIT(29) | BIT(28) | BIT(27) | BIT(26) | BIT(25))),
				odm_get_bb_reg(p_dm_odm, 0xc1c, 0xFFE00000),
				p_rf_calibrate_info->bb_swing_idx_ofdm[rf_path], rf_path));
			break;

		case ODM_RF_PATH_B:
			get_mix_mode_tx_agc_bb_swing_offset_8822b(p_dm_odm, method, rf_path, tx_power_index_offest);
			odm_set_bb_reg(p_dm_odm, 0xE94, (BIT(29) | BIT(28) | BIT(27) | BIT(26) | BIT(25)), p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path]);
			odm_set_bb_reg(p_dm_odm, REG_B_TX_SCALE_JAGUAR, 0xFFE00000, tx_scaling_table_jaguar[p_rf_calibrate_info->bb_swing_idx_ofdm[rf_path]]);

			ODM_RT_TRACE(p_dm_odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
				("TXAGC(0xE94)=0x%x BBSwing(0xe1c)=0x%x BBSwingIndex=%d rf_path=%d\n",
				odm_get_bb_reg(p_dm_odm, 0xE94, (BIT(29) | BIT(28) | BIT(27) | BIT(26) | BIT(25))),
				odm_get_bb_reg(p_dm_odm, 0xe1c, 0xFFE00000),
				p_rf_calibrate_info->bb_swing_idx_ofdm[rf_path], rf_path));
			break;

		default:
			break;
		}
	}
}


void
get_delta_swing_table_8822b(
	void		*p_dm_void,
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	u8 **temperature_up_a,
	u8 **temperature_down_a,
	u8 **temperature_up_b,
	u8 **temperature_down_b,
	u8 **temperature_up_cck_a,
	u8 **temperature_down_cck_a,
	u8 **temperature_up_cck_b,
	u8 **temperature_down_cck_b
#else
	u8 **temperature_up_a,
	u8 **temperature_down_a,
	u8 **temperature_up_b,
	u8 **temperature_down_b
#endif
)
{
	struct PHY_DM_STRUCT		*p_dm_odm		= (struct PHY_DM_STRUCT *)p_dm_void;
	struct odm_rf_calibration_structure	*p_rf_calibrate_info	= &(p_dm_odm->rf_calibrate_info);

#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	u8			channel			= *(p_dm_odm->p_channel);
#else
	struct _ADAPTER		*adapter			= p_dm_odm->adapter;
	HAL_DATA_TYPE	*p_hal_data		= GET_HAL_DATA(adapter);
	u8			channel			= p_hal_data->current_channel;
#endif

#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	*temperature_up_cck_a   = p_rf_calibrate_info->delta_swing_table_idx_2g_cck_a_p;
	*temperature_down_cck_a = p_rf_calibrate_info->delta_swing_table_idx_2g_cck_a_n;
	*temperature_up_cck_b   = p_rf_calibrate_info->delta_swing_table_idx_2g_cck_b_p;
	*temperature_down_cck_b = p_rf_calibrate_info->delta_swing_table_idx_2g_cck_b_n;
#endif

	*temperature_up_a   = p_rf_calibrate_info->delta_swing_table_idx_2ga_p;
	*temperature_down_a = p_rf_calibrate_info->delta_swing_table_idx_2ga_n;
	*temperature_up_b   = p_rf_calibrate_info->delta_swing_table_idx_2gb_p;
	*temperature_down_b = p_rf_calibrate_info->delta_swing_table_idx_2gb_n;

	if (36 <= channel && channel <= 64) {
		*temperature_up_a   = p_rf_calibrate_info->delta_swing_table_idx_5ga_p[0];
		*temperature_down_a = p_rf_calibrate_info->delta_swing_table_idx_5ga_n[0];
		*temperature_up_b   = p_rf_calibrate_info->delta_swing_table_idx_5gb_p[0];
		*temperature_down_b = p_rf_calibrate_info->delta_swing_table_idx_5gb_n[0];
	} else if (100 <= channel && channel <= 144)	{
		*temperature_up_a   = p_rf_calibrate_info->delta_swing_table_idx_5ga_p[1];
		*temperature_down_a = p_rf_calibrate_info->delta_swing_table_idx_5ga_n[1];
		*temperature_up_b   = p_rf_calibrate_info->delta_swing_table_idx_5gb_p[1];
		*temperature_down_b = p_rf_calibrate_info->delta_swing_table_idx_5gb_n[1];
	} else if (149 <= channel && channel <= 177)	{
		*temperature_up_a   = p_rf_calibrate_info->delta_swing_table_idx_5ga_p[2];
		*temperature_down_a = p_rf_calibrate_info->delta_swing_table_idx_5ga_n[2];
		*temperature_up_b   = p_rf_calibrate_info->delta_swing_table_idx_5gb_p[2];
		*temperature_down_b = p_rf_calibrate_info->delta_swing_table_idx_5gb_n[2];
	}
}


void
_phy_lc_calibrate_8822b(
	struct PHY_DM_STRUCT	*p_dm_odm
)
{
	u32 lc_cal = 0, cnt = 0;

	/*backup RF0x18*/
	lc_cal = odm_get_rf_reg(p_dm_odm, ODM_RF_PATH_A, RF_CHNLBW, RFREGOFFSETMASK);

	/*Start LCK*/
	odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_A, RF_CHNLBW, RFREGOFFSETMASK, lc_cal | 0x08000);

	ODM_delay_ms(100);

	for (cnt = 0; cnt < 100; cnt++) {
		if (odm_get_rf_reg(p_dm_odm, ODM_RF_PATH_A, RF_CHNLBW, 0x8000) != 0x1)
			break;
		ODM_delay_ms(10);
	}

	/*Recover channel number*/
	odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_A, RF_CHNLBW, RFREGOFFSETMASK, lc_cal);
}



void
phy_lc_calibrate_8822b(
	void	*p_dm_void
)
{
	struct PHY_DM_STRUCT	*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	bool	is_start_cont_tx = false, is_single_tone = false, is_carrier_suppression = false;
	u64		start_time;
	u64		progressing_time;


#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	struct _ADAPTER		*p_adapter = p_dm_odm->adapter;

#if (MP_DRIVER == 1)
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PMPT_CONTEXT	p_mpt_ctx = &(p_adapter->mpt_ctx);
#else
	PMPT_CONTEXT	p_mpt_ctx = &(p_adapter->mppriv.mpt_ctx);
#endif
	is_start_cont_tx = p_mpt_ctx->is_start_cont_tx;
	is_single_tone = p_mpt_ctx->is_single_tone;
	is_carrier_suppression = p_mpt_ctx->is_carrier_suppression;
#endif
#endif

	if (is_start_cont_tx || is_single_tone || is_carrier_suppression) {
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[LCK]continues TX ing !!! LCK return\n"));
		return;
	}

	start_time = odm_get_current_time(p_dm_odm);
	_phy_lc_calibrate_8822b(p_dm_odm);
	progressing_time = odm_get_progressing_time(p_dm_odm, start_time);
	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[LCK]LCK progressing_time = %lld\n", progressing_time));
}



void configure_txpower_track_8822b(
	struct _TXPWRTRACK_CFG	*p_config
)
{
	p_config->swing_table_size_cck = TXSCALE_TABLE_SIZE;
	p_config->swing_table_size_ofdm = TXSCALE_TABLE_SIZE;
	p_config->threshold_iqk = IQK_THRESHOLD;
	p_config->threshold_dpk = DPK_THRESHOLD;
	p_config->average_thermal_num = AVG_THERMAL_NUM_8822B;
	p_config->rf_path_count = MAX_PATH_NUM_8822B;
	p_config->thermal_reg_addr = RF_T_METER_8822B;

	p_config->odm_tx_pwr_track_set_pwr = odm_tx_pwr_track_set_pwr8822b;
	p_config->do_iqk = do_iqk_8822b;
	p_config->phy_lc_calibrate = phy_lc_calibrate_8822b;

#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	p_config->get_delta_all_swing_table = get_delta_swing_table_8822b;
#else
	p_config->get_delta_swing_table = get_delta_swing_table_8822b;
#endif
}


void phy_set_rf_path_switch_8822b(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	struct PHY_DM_STRUCT		*p_dm_odm,
#else
	struct _ADAPTER	*p_adapter,
#endif
	bool		is_main
)
{
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*p_hal_data = GET_HAL_DATA(p_adapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	struct PHY_DM_STRUCT		*p_dm_odm = &p_hal_data->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	struct PHY_DM_STRUCT		*p_dm_odm = &p_hal_data->dm_out_src;
#endif
#endif

	/*BY SY Request */
	odm_set_bb_reg(p_dm_odm, 0x4C, (BIT(24) | BIT(23)), 0x2);
	odm_set_bb_reg(p_dm_odm, 0x974, 0xff, 0xff);

	/*odm_set_bb_reg(p_dm_odm, 0x1991, 0x3, 0x0);*/
	odm_set_bb_reg(p_dm_odm, 0x1990, (BIT(9) | BIT(8)), 0x0);

	/*odm_set_bb_reg(p_dm_odm, 0xCBE, 0x8, 0x0);*/
	odm_set_bb_reg(p_dm_odm, 0xCBC, BIT(19), 0x0);

	odm_set_bb_reg(p_dm_odm, 0xCB4, 0xff, 0x77);

	odm_set_bb_reg(p_dm_odm, 0x70, MASKBYTE3, 0x0e);
	odm_set_bb_reg(p_dm_odm, 0x1704, MASKDWORD, 0x0000ff00);
	odm_set_bb_reg(p_dm_odm, 0x1700, MASKDWORD, 0xc00f0038);

	if (is_main) {
		/*odm_set_bb_reg(p_dm_odm, 0xCBD, 0x3, 0x2);		WiFi */
		odm_set_bb_reg(p_dm_odm, 0xCBC, (BIT(9) | BIT8), 0x2);		/*WiFi */
	} else {
		/*odm_set_bb_reg(p_dm_odm, 0xCBD, 0x3, 0x1);	 BT*/
		odm_set_bb_reg(p_dm_odm, 0xCBC, (BIT(9) | BIT8), 0x1);	 /*BT*/
	}
}

bool
_phy_query_rf_path_switch_8822b(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	struct PHY_DM_STRUCT	*p_dm_odm
#else
	struct _ADAPTER	*p_adapter
#endif
)
{
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*p_hal_data = GET_HAL_DATA(p_adapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	struct PHY_DM_STRUCT		*p_dm_odm = &p_hal_data->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	struct PHY_DM_STRUCT		*p_dm_odm = &p_hal_data->dm_out_src;
#endif
#endif

	if (odm_get_bb_reg(p_dm_odm, 0xCBC, (BIT(9) | BIT8)) == 0x2)	/*WiFi */
		return true;
	else
		return false;
}


bool phy_query_rf_path_switch_8822b(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	struct PHY_DM_STRUCT		*p_dm_odm
#else
	struct _ADAPTER	*p_adapter
#endif
)
{

#if DISABLE_BB_RF
	return true;
#endif

#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	return _phy_query_rf_path_switch_8822b(p_dm_odm);
#else
	return _phy_query_rf_path_switch_8822b(p_adapter);
#endif
}


#endif	/* (RTL8822B_SUPPORT == 0)*/
