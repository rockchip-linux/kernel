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


/*---------------------------Define Local Constant---------------------------*/


/*---------------------------Define Local Constant---------------------------*/


#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
void do_iqk_8822b(
	void		*p_dm_void,
	u8		delta_thermal_index,
	u8		thermal_value,
	u8		threshold
)
{
	struct PHY_DM_STRUCT	*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;

	struct _ADAPTER		*adapter = p_dm_odm->adapter;
	HAL_DATA_TYPE	*p_hal_data = GET_HAL_DATA(adapter);

	odm_reset_iqk_result(p_dm_odm);

	p_dm_odm->rf_calibrate_info.thermal_value_iqk = thermal_value;

	phy_iq_calibrate_8822b(p_dm_odm, true);

}
#else
/*Originally p_config->do_iqk is hooked phy_iq_calibrate_8822b, but do_iqk_8822b and phy_iq_calibrate_8822b have different arguments*/
void do_iqk_8822b(
	void		*p_dm_void,
	u8	delta_thermal_index,
	u8	thermal_value,
	u8	threshold
)
{
	struct PHY_DM_STRUCT	*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	bool		is_recovery = (bool) delta_thermal_index;

	phy_iq_calibrate_8822b(p_dm_odm, true);
}
#endif

void
_iqk_fill_iqk_report_8822b(
	void		*p_dm_void,
	u8			channel
)
{
	struct PHY_DM_STRUCT	*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _IQK_INFORMATION	*p_iqk_info = &p_dm_odm->IQK_info;
	u32		tmp1 = 0x0, tmp2 = 0x0, tmp3 = 0x0;
	u8		i;

	for (i = 0; i < SS_8822B; i++) {
		tmp1 = tmp1 + ((p_iqk_info->IQK_fail_report[channel][i][TX_IQK] & 0x1) << i);
		tmp2 = tmp2 + ((p_iqk_info->IQK_fail_report[channel][i][RX_IQK] & 0x1) << (i + 4));
		tmp3 = tmp3 + ((p_iqk_info->RXIQK_fail_code[channel][i] & 0x3) << (i * 2 + 8));
	}
	odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008);
	odm_set_bb_reg(p_dm_odm, 0x1bf0, 0x0000ffff, tmp1 | tmp2 | tmp3);

	for (i = 0; i < 2; i++)
		odm_write_4byte(p_dm_odm, 0x1be8 + (i * 4), (p_iqk_info->RXIQK_AGC[channel][(i * 2) + 1] << 16) | p_iqk_info->RXIQK_AGC[channel][i * 2]);
}


void
_iqk_iqk_fail_report_8822b(
	struct PHY_DM_STRUCT	*p_dm_odm
)
{
	u32		tmp1bf0 = 0x0;
	u8		i;

	tmp1bf0 = odm_read_4byte(p_dm_odm, 0x1bf0);

	for (i = 0; i < 4; i++) {
		if (tmp1bf0 & (0x1 << i))
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK] please check S%d TXIQK\n", i));
#else
			panic_printk("[IQK] please check S%d TXIQK\n", i);
#endif
		if (tmp1bf0 & (0x1 << (i + 12)))
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK] please check S%d RXIQK\n", i));
#else
			panic_printk("[IQK] please check S%d RXIQK\n", i);
#endif

	}
}


void
_iqk_backup_mac_bb_8822b(
	struct PHY_DM_STRUCT	*p_dm_odm,
	u32		*MAC_backup,
	u32		*BB_backup,
	u32		*backup_mac_reg,
	u32		*backup_bb_reg
)
{
	u32 i;
	for (i = 0; i < MAC_REG_NUM_8822B; i++)
		MAC_backup[i] = odm_read_4byte(p_dm_odm, backup_mac_reg[i]);

	for (i = 0; i < BB_REG_NUM_8822B; i++)
		BB_backup[i] = odm_read_4byte(p_dm_odm, backup_bb_reg[i]);

	/*	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]BackupMacBB Success!!!!\n")); */
}


void
_iqk_backup_rf_8822b(
	struct PHY_DM_STRUCT	*p_dm_odm,
	u32		RF_backup[][2],
	u32		*backup_rf_reg
)
{
	u32 i;

	for (i = 0; i < RF_REG_NUM_8822B; i++) {
		RF_backup[i][ODM_RF_PATH_A] = odm_get_rf_reg(p_dm_odm, ODM_RF_PATH_A, backup_rf_reg[i], RFREGOFFSETMASK);
		RF_backup[i][ODM_RF_PATH_B] = odm_get_rf_reg(p_dm_odm, ODM_RF_PATH_B, backup_rf_reg[i], RFREGOFFSETMASK);
	}
	/*	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]BackupRF Success!!!!\n")); */
}


void
_iqk_agc_bnd_int_8822b(
	struct PHY_DM_STRUCT	*p_dm_odm
)
{
	/*initialize RX AGC bnd, it must do after bbreset*/
	odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008);
	odm_write_4byte(p_dm_odm, 0x1b00, 0xf80a7008);
	odm_write_4byte(p_dm_odm, 0x1b00, 0xf8015008);
	odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008);
	/*ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE, ("[IQK]init. rx agc bnd\n"));*/
}


void
_iqk_bb_reset_8822b(
	struct PHY_DM_STRUCT	*p_dm_odm
)
{
	bool		cca_ing = false;
	u32		count = 0;

	odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_A, 0x0, RFREGOFFSETMASK, 0x10000);
	odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_B, 0x0, RFREGOFFSETMASK, 0x10000);

	while (1) {
		odm_write_4byte(p_dm_odm, 0x8fc, 0x0);
		odm_set_bb_reg(p_dm_odm, 0x198c, 0x7, 0x7);
		cca_ing = (bool) odm_get_bb_reg(p_dm_odm, 0xfa0, BIT(3));

		if (count > 30)
			cca_ing = false;

		if (cca_ing) {
			ODM_delay_ms(1);
			count++;
		} else {
			odm_write_1byte(p_dm_odm, 0x808, 0x0);	/*RX ant off*/
			odm_set_bb_reg(p_dm_odm, 0xa04, BIT(27) | BIT26 | BIT25 | BIT24, 0x0);		/*CCK RX path off*/

			/*BBreset*/
			odm_set_bb_reg(p_dm_odm, 0x0, BIT(16), 0x0);
			odm_set_bb_reg(p_dm_odm, 0x0, BIT(16), 0x1);

			if (odm_get_bb_reg(p_dm_odm, 0x660, BIT(16)))
				odm_write_4byte(p_dm_odm, 0x6b4, 0x89000006);
			/*ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]BBreset!!!!\n"));*/
			break;
		}
	}
}

void
_iqk_afe_setting_8822b(
	struct PHY_DM_STRUCT	*p_dm_odm,
	bool		do_iqk
)
{
	if (do_iqk) {
		odm_write_4byte(p_dm_odm, 0xc60, 0x50000000);
		odm_write_4byte(p_dm_odm, 0xc60, 0x70070040);
		odm_write_4byte(p_dm_odm, 0xe60, 0x50000000);
		odm_write_4byte(p_dm_odm, 0xe60, 0x70070040);

		odm_write_4byte(p_dm_odm, 0xc58, 0xd8000402);
		odm_write_4byte(p_dm_odm, 0xc5c, 0xd1000120);
		odm_write_4byte(p_dm_odm, 0xc6c, 0x00000a15);
		odm_write_4byte(p_dm_odm, 0xe58, 0xd8000402);
		odm_write_4byte(p_dm_odm, 0xe5c, 0xd1000120);
		odm_write_4byte(p_dm_odm, 0xe6c, 0x00000a15);
		_iqk_bb_reset_8822b(p_dm_odm);
		/*		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]AFE setting for IQK mode!!!!\n")); */
	} else {
		odm_write_4byte(p_dm_odm, 0xc60, 0x50000000);
		odm_write_4byte(p_dm_odm, 0xc60, 0x70038040);
		odm_write_4byte(p_dm_odm, 0xe60, 0x50000000);
		odm_write_4byte(p_dm_odm, 0xe60, 0x70038040);

		odm_write_4byte(p_dm_odm, 0xc58, 0xd8020402);
		odm_write_4byte(p_dm_odm, 0xc5c, 0xde000120);
		odm_write_4byte(p_dm_odm, 0xc6c, 0x0000122a);
		odm_write_4byte(p_dm_odm, 0xe58, 0xd8020402);
		odm_write_4byte(p_dm_odm, 0xe5c, 0xde000120);
		odm_write_4byte(p_dm_odm, 0xe6c, 0x0000122a);
		/*		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]AFE setting for Normal mode!!!!\n")); */
	}
}

void
_iqk_restore_mac_bb_8822b(
	struct PHY_DM_STRUCT		*p_dm_odm,
	u32		*MAC_backup,
	u32		*BB_backup,
	u32		*backup_mac_reg,
	u32		*backup_bb_reg
)
{
	u32 i;

	for (i = 0; i < MAC_REG_NUM_8822B; i++)
		odm_write_4byte(p_dm_odm, backup_mac_reg[i], MAC_backup[i]);
	for (i = 0; i < BB_REG_NUM_8822B; i++)
		odm_write_4byte(p_dm_odm, backup_bb_reg[i], BB_backup[i]);
	/*	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]RestoreMacBB Success!!!!\n")); */
}

void
_iqk_restore_rf_8822b(
	struct PHY_DM_STRUCT			*p_dm_odm,
	u32			*backup_rf_reg,
	u32			RF_backup[][2]
)
{
	u32 i;

	odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_A, 0xef, RFREGOFFSETMASK, 0x0);
	odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_B, 0xef, RFREGOFFSETMASK, 0x0);
	/*0xdf[4]=0*/
	odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_A, 0xdf, RFREGOFFSETMASK, RF_backup[0][ODM_RF_PATH_A] & (~BIT(4)));
	odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_B, 0xdf, RFREGOFFSETMASK, RF_backup[0][ODM_RF_PATH_B] & (~BIT(4)));

	for (i = 1; i < RF_REG_NUM_8822B; i++) {
		odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_A, backup_rf_reg[i], RFREGOFFSETMASK, RF_backup[i][ODM_RF_PATH_A]);
		odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_B, backup_rf_reg[i], RFREGOFFSETMASK, RF_backup[i][ODM_RF_PATH_B]);
	}
	/*	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]RestoreRF Success!!!!\n")); */

}


void
_iqk_backup_iqk_8822b(
	struct PHY_DM_STRUCT			*p_dm_odm,
	u8				step
)
{
	struct _IQK_INFORMATION	*p_iqk_info = &p_dm_odm->IQK_info;
	u8		i, j, k, path, idx;
	u32		tmp;
	u16		iqk_apply[2] = {0xc94, 0xe94};

	if (step == 0x0) {
		p_iqk_info->iqk_channel[1] = p_iqk_info->iqk_channel[0];
		for (i = 0; i < 2; i++) {
			p_iqk_info->LOK_IDAC[1][i] = p_iqk_info->LOK_IDAC[0][i];
			p_iqk_info->RXIQK_AGC[1][i] = p_iqk_info->RXIQK_AGC[0][i];
			p_iqk_info->bypass_iqk[1][i] = p_iqk_info->bypass_iqk[0][i];
			p_iqk_info->RXIQK_fail_code[1][i] = p_iqk_info->RXIQK_fail_code[0][i];
			for (j = 0; j < 2; j++) {
				p_iqk_info->IQK_fail_report[1][i][j] = p_iqk_info->IQK_fail_report[0][i][j];
				for (k = 0; k < 8; k++) {
					p_iqk_info->IQK_CFIR_real[1][i][j][k] = p_iqk_info->IQK_CFIR_real[0][i][j][k];
					p_iqk_info->IQK_CFIR_imag[1][i][j][k] = p_iqk_info->IQK_CFIR_imag[0][i][j][k];
				}
			}
		}

		for (i = 0; i < 4; i++) {
			p_iqk_info->RXIQK_fail_code[0][i] = 0x0;
			p_iqk_info->RXIQK_AGC[0][i] = 0x0;
			for (j = 0; j < 2; j++) {
				p_iqk_info->IQK_fail_report[0][i][j] = true;
				p_iqk_info->gs_retry_count[0][i][j] = 0x0;
			}
			for (j = 0; j < 3; j++)
				p_iqk_info->retry_count[0][i][j] = 0x0;
		}
	} else {
		p_iqk_info->iqk_channel[0] = p_iqk_info->rf_reg18;
		for (path = 0; path < 2; path++) {
			p_iqk_info->LOK_IDAC[0][path] = odm_get_rf_reg(p_dm_odm, path, 0x58, RFREGOFFSETMASK);
			p_iqk_info->bypass_iqk[0][path] = odm_get_bb_reg(p_dm_odm, iqk_apply[path], MASKDWORD);

			for (idx = 0; idx < 2; idx++) {
				odm_set_bb_reg(p_dm_odm, 0x1b00, MASKDWORD, 0xf8000008 | path << 1);

				if (idx == 0)
					odm_set_bb_reg(p_dm_odm, 0x1b0c, BIT(13) | BIT(12), 0x3);
				else
					odm_set_bb_reg(p_dm_odm, 0x1b0c, BIT(13) | BIT(12), 0x1);

				odm_set_bb_reg(p_dm_odm, 0x1bd4, BIT(20) | BIT(19) | BIT(18) | BIT(17) | BIT(16), 0x10);

				for (i = 0; i < 8; i++) {
					odm_set_bb_reg(p_dm_odm, 0x1bd8, MASKDWORD, 0xe0000001 + (i * 4));
					tmp = odm_get_bb_reg(p_dm_odm, 0x1bfc, MASKDWORD);
					p_iqk_info->IQK_CFIR_real[0][path][idx][i] = (tmp & 0x0fff0000) >> 16;
					p_iqk_info->IQK_CFIR_imag[0][path][idx][i] = tmp & 0xfff;
				}
			}
			odm_set_bb_reg(p_dm_odm, 0x1bd8, MASKDWORD, 0x0);
			odm_set_bb_reg(p_dm_odm, 0x1b0c, BIT(13) | BIT(12), 0x0);
		}
	}
}

void
_iqk_reload_iqk_setting_8822b(
	struct PHY_DM_STRUCT			*p_dm_odm,
	u8				channel,
	u8				reload_idx  /*1: reload TX, 2: reload LO, TX, RX*/
)
{
	struct _IQK_INFORMATION	*p_iqk_info = &p_dm_odm->IQK_info;
	u8 i, path, idx;
	u16		iqk_apply[2] = {0xc94, 0xe94};

	for (path = 0; path < 2; path++) {
		if (reload_idx == 2) {
			odm_set_rf_reg(p_dm_odm, path, 0xdf, BIT(4), 0x1);
			odm_set_rf_reg(p_dm_odm, path, 0x58, RFREGOFFSETMASK, p_iqk_info->LOK_IDAC[channel][path]);
		}

		for (idx = 0; idx < reload_idx; idx++) {
			odm_set_bb_reg(p_dm_odm, 0x1b00, MASKDWORD, 0xf8000008 | path << 1);
			odm_set_bb_reg(p_dm_odm, 0x1b2c, MASKDWORD, 0x7);
			odm_set_bb_reg(p_dm_odm, 0x1b38, MASKDWORD, 0x20000000);
			odm_set_bb_reg(p_dm_odm, 0x1b3c, MASKDWORD, 0x20000000);
			odm_set_bb_reg(p_dm_odm, 0x1bcc, MASKDWORD, 0x00000000);

			if (idx == 0)
				odm_set_bb_reg(p_dm_odm, 0x1b0c, BIT(13) | BIT(12), 0x3);
			else
				odm_set_bb_reg(p_dm_odm, 0x1b0c, BIT(13) | BIT(12), 0x1);

			odm_set_bb_reg(p_dm_odm, 0x1bd4, BIT(20) | BIT(19) | BIT(18) | BIT(17) | BIT(16), 0x10);

			for (i = 0; i < 8; i++) {
				odm_write_4byte(p_dm_odm, 0x1bd8,	((0xc0000000 >> idx) + 0x3) + (i * 4) + (p_iqk_info->IQK_CFIR_real[channel][path][idx][i] << 9));
				odm_write_4byte(p_dm_odm, 0x1bd8, ((0xc0000000 >> idx) + 0x1) + (i * 4) + (p_iqk_info->IQK_CFIR_imag[channel][path][idx][i] << 9));
			}
		}
		odm_set_bb_reg(p_dm_odm, iqk_apply[path], MASKDWORD, p_iqk_info->bypass_iqk[channel][path]);

		odm_set_bb_reg(p_dm_odm, 0x1bd8, MASKDWORD, 0x0);
		odm_set_bb_reg(p_dm_odm, 0x1b0c, BIT(13) | BIT(12), 0x0);
	}
}


bool
_iqk_reload_iqk_8822b(
	struct PHY_DM_STRUCT			*p_dm_odm,
	bool			reset
)
{
	struct _IQK_INFORMATION	*p_iqk_info = &p_dm_odm->IQK_info;
	u8 i;
	bool reload = false;

	if (reset) {
		for (i = 0; i < 2; i++)
			p_iqk_info->iqk_channel[i] = 0x0;
	} else {
		p_iqk_info->rf_reg18 = odm_get_rf_reg(p_dm_odm, 0, 0x18, RFREGOFFSETMASK);

		for (i = 0; i < 2; i++) {
			if (p_iqk_info->rf_reg18 == p_iqk_info->iqk_channel[i]) {
				_iqk_reload_iqk_setting_8822b(p_dm_odm, i, 2);
				_iqk_fill_iqk_report_8822b(p_dm_odm, i);
				ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]reload IQK result before!!!!\n"));
				reload = true;
			}
		}
	}
	return reload;
}


void
_iqk_rfe_setting_8822b(
	struct PHY_DM_STRUCT	*p_dm_odm,
	bool		ext_pa_on
)
{
	if (ext_pa_on) {
		/*RFE setting*/
		odm_write_4byte(p_dm_odm, 0xcb0, 0x77777777);
		odm_write_4byte(p_dm_odm, 0xcb4, 0x00007777);
		odm_write_4byte(p_dm_odm, 0xcbc, 0x0000083B);
		odm_write_4byte(p_dm_odm, 0xeb0, 0x77777777);
		odm_write_4byte(p_dm_odm, 0xeb4, 0x00007777);
		odm_write_4byte(p_dm_odm, 0xebc, 0x0000083B);
		/*odm_write_4byte(p_dm_odm, 0x1990, 0x00000c30);*/
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]external PA on!!!!\n"));
	} else {
		/*RFE setting*/
		odm_write_4byte(p_dm_odm, 0xcb0, 0x77777777);
		odm_write_4byte(p_dm_odm, 0xcb4, 0x00007777);
		odm_write_4byte(p_dm_odm, 0xcbc, 0x00000100);
		odm_write_4byte(p_dm_odm, 0xeb0, 0x77777777);
		odm_write_4byte(p_dm_odm, 0xeb4, 0x00007777);
		odm_write_4byte(p_dm_odm, 0xebc, 0x00000100);
		/*odm_write_4byte(p_dm_odm, 0x1990, 0x00000c30);*/
		/*		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]external PA off!!!!\n"));*/
	}
}


void
_iqk_rf_setting_8822b(
	struct PHY_DM_STRUCT	*p_dm_odm
)
{
	u8 path;
	u32 tmp;

	odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008);
	odm_write_4byte(p_dm_odm, 0x1bb8, 0x00000000);

	for (path = 0; path < 2; path++) {
		/*0xdf:B11 = 1,B4 = 0, B1 = 1*/
		tmp = odm_get_rf_reg(p_dm_odm, path, 0xdf, RFREGOFFSETMASK);
		tmp = (tmp & (~BIT(4))) | BIT(1) | BIT(11);
		odm_set_rf_reg(p_dm_odm, path, 0xdf, RFREGOFFSETMASK, tmp);

		/*release 0x56 TXBB*/
		odm_set_rf_reg(p_dm_odm, path, 0x65, RFREGOFFSETMASK, 0x09000);

		if (*p_dm_odm->p_band_type == ODM_BAND_5G) {
			odm_set_rf_reg(p_dm_odm, path, 0xef, BIT(19), 0x1);
			odm_set_rf_reg(p_dm_odm, path, 0x33, RFREGOFFSETMASK, 0x00026);
			odm_set_rf_reg(p_dm_odm, path, 0x3e, RFREGOFFSETMASK, 0x00037);
			odm_set_rf_reg(p_dm_odm, path, 0x3f, RFREGOFFSETMASK, 0xdefce);
			odm_set_rf_reg(p_dm_odm, path, 0xef, BIT(19), 0x0);
		} else {
			odm_set_rf_reg(p_dm_odm, path, 0xef, BIT(19), 0x1);
			odm_set_rf_reg(p_dm_odm, path, 0x33, RFREGOFFSETMASK, 0x00026);
			odm_set_rf_reg(p_dm_odm, path, 0x3e, RFREGOFFSETMASK, 0x00037);
			odm_set_rf_reg(p_dm_odm, path, 0x3f, RFREGOFFSETMASK, 0x5efce);
			odm_set_rf_reg(p_dm_odm, path, 0xef, BIT(19), 0x0);
		}
	}
}



void
_iqk_configure_macbb_8822b(
	struct PHY_DM_STRUCT		*p_dm_odm
)
{
	/*MACBB register setting*/
	odm_write_1byte(p_dm_odm, 0x522, 0x7f);
	odm_set_bb_reg(p_dm_odm, 0x550, BIT(11) | BIT(3), 0x0);
	odm_set_bb_reg(p_dm_odm, 0x90c, BIT(15), 0x1);			/*0x90c[15]=1: dac_buf reset selection*/
	odm_set_bb_reg(p_dm_odm, 0x9a4, BIT(31), 0x0);         /*0x9a4[31]=0: Select da clock*/
	/*0xc94[0]=1, 0xe94[0]=1: 讓tx從iqk打出來*/
	odm_set_bb_reg(p_dm_odm, 0xc94, BIT(0), 0x1);
	odm_set_bb_reg(p_dm_odm, 0xe94, BIT(0), 0x1);
	/* 3-wire off*/
	odm_write_4byte(p_dm_odm, 0xc00, 0x00000004);
	odm_write_4byte(p_dm_odm, 0xe00, 0x00000004);
	/*	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]Set MACBB setting for IQK!!!!\n"));*/

}

void
_iqk_lok_setting_8822b(
	struct PHY_DM_STRUCT	*p_dm_odm,

	u8 path
)
{
	odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008 | path << 1);
	odm_write_4byte(p_dm_odm, 0x1bcc, 0x9);
	odm_write_1byte(p_dm_odm, 0x1b23, 0x00);

	switch (*p_dm_odm->p_band_type) {
	case ODM_BAND_2_4G:
		odm_write_1byte(p_dm_odm, 0x1b2b, 0x00);
		odm_set_rf_reg(p_dm_odm, path, 0x56, RFREGOFFSETMASK, 0x50df2);
		odm_set_rf_reg(p_dm_odm, path, 0x8f, RFREGOFFSETMASK, 0xadc00);
		/* WE_LUT_TX_LOK*/
		odm_set_rf_reg(p_dm_odm, path, 0xef, BIT(4), 0x1);
		odm_set_rf_reg(p_dm_odm, path, 0x33, BIT(1) | BIT(0), 0x0);
		break;
	case ODM_BAND_5G:
		odm_write_1byte(p_dm_odm, 0x1b2b, 0x80);
		odm_set_rf_reg(p_dm_odm, path, 0x56, RFREGOFFSETMASK, 0x5086c);
		odm_set_rf_reg(p_dm_odm, path, 0x8f, RFREGOFFSETMASK, 0xa9c00);
		/* WE_LUT_TX_LOK*/
		odm_set_rf_reg(p_dm_odm, path, 0xef, BIT(4), 0x1);
		odm_set_rf_reg(p_dm_odm, path, 0x33, BIT(1) | BIT(0), 0x1);
		break;
	}
	/*	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]Set LOK setting!!!!\n"));*/
}


void
_iqk_txk_setting_8822b(
	struct PHY_DM_STRUCT	*p_dm_odm,
	u8 path
)
{
	odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008 | path << 1);
	odm_write_4byte(p_dm_odm, 0x1bcc, 0x9);
	odm_write_4byte(p_dm_odm, 0x1b20, 0x01440008);

	if (path == 0x0)
		odm_write_4byte(p_dm_odm, 0x1b00, 0xf800000a);
	else
		odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008);
	odm_write_4byte(p_dm_odm, 0x1bcc, 0x3f);

	switch (*p_dm_odm->p_band_type) {
	case ODM_BAND_2_4G:
		odm_set_rf_reg(p_dm_odm, path, 0x56, RFREGOFFSETMASK, 0x50df2);
		odm_set_rf_reg(p_dm_odm, path, 0x8f, RFREGOFFSETMASK, 0xadc00);
		odm_write_1byte(p_dm_odm, 0x1b2b, 0x00);
		break;
	case ODM_BAND_5G:
		odm_set_rf_reg(p_dm_odm, path, 0x56, RFREGOFFSETMASK, 0x500ef);
		odm_set_rf_reg(p_dm_odm, path, 0x8f, RFREGOFFSETMASK, 0xa9c00);
		odm_write_1byte(p_dm_odm, 0x1b2b, 0x80);
		break;
	}
	/*	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]Set TXK setting!!!!\n"));*/

}


void
_iqk_rxk1_setting_8822b(
	struct PHY_DM_STRUCT	*p_dm_odm,
	u8 path
)
{
	struct _IQK_INFORMATION	*p_iqk_info = &p_dm_odm->IQK_info;

	odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008 | path << 1);

	switch (*p_dm_odm->p_band_type) {
	case ODM_BAND_2_4G:
		p_iqk_info->tmp1bcc = 0x09;
		odm_write_1byte(p_dm_odm, 0x1bcc, p_iqk_info->tmp1bcc);
		odm_write_1byte(p_dm_odm, 0x1b2b, 0x00);
		odm_write_4byte(p_dm_odm, 0x1b20, 0x01450008);
		odm_write_4byte(p_dm_odm, 0x1b24, 0x01460c88);
		odm_set_rf_reg(p_dm_odm, path, 0x56, RFREGOFFSETMASK, 0x510e0);
		odm_set_rf_reg(p_dm_odm, path, 0x8f, RFREGOFFSETMASK, 0xacc00);
		break;
	case ODM_BAND_5G:
		p_iqk_info->tmp1bcc = 0x09;
		odm_write_1byte(p_dm_odm, 0x1bcc, 0x09);
		odm_write_1byte(p_dm_odm, 0x1b2b, 0x80);
		odm_write_4byte(p_dm_odm, 0x1b20, 0x00850008);
		odm_write_4byte(p_dm_odm, 0x1b24, 0x00460048);
		odm_set_rf_reg(p_dm_odm, path, 0x56, RFREGOFFSETMASK, 0x510e0);
		odm_set_rf_reg(p_dm_odm, path, 0x8f, RFREGOFFSETMASK, 0xadc00);
		break;
	}
	/*ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]Set RXK setting!!!!\n"));*/

}


void
_iqk_rxk2_setting_8822b(
	struct PHY_DM_STRUCT	*p_dm_odm,
	u8 path
)
{
	struct _IQK_INFORMATION	*p_iqk_info = &p_dm_odm->IQK_info;

	odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008 | path << 1);

	switch (*p_dm_odm->p_band_type) {
	case ODM_BAND_2_4G:
		p_iqk_info->tmp1bcc = 0x12;
		odm_write_1byte(p_dm_odm, 0x1bcc, p_iqk_info->tmp1bcc);
		odm_write_1byte(p_dm_odm, 0x1b2b, 0x00);
		odm_write_4byte(p_dm_odm, 0x1b20, 0x01450008);
		odm_write_4byte(p_dm_odm, 0x1b24, 0x01460848);
		odm_set_rf_reg(p_dm_odm, path, 0x56, RFREGOFFSETMASK, 0x510e0);
		odm_set_rf_reg(p_dm_odm, path, 0x8f, RFREGOFFSETMASK, 0xa9c00);
		break;
	case ODM_BAND_5G:
		if (path == ODM_RF_PATH_A) {
			p_iqk_info->tmp1bcc = 0x12;
			odm_write_1byte(p_dm_odm, 0x1bcc, p_iqk_info->tmp1bcc);
		} else {
			p_iqk_info->tmp1bcc = 0x09;
			odm_write_1byte(p_dm_odm, 0x1bcc, p_iqk_info->tmp1bcc);
		}
		odm_write_1byte(p_dm_odm, 0x1b2b, 0x80);
		odm_write_4byte(p_dm_odm, 0x1b20, 0x00850008);
		odm_write_4byte(p_dm_odm, 0x1b24, 0x00460848);
		odm_set_rf_reg(p_dm_odm, path, 0x56, RFREGOFFSETMASK, 0x51060);
		odm_set_rf_reg(p_dm_odm, path, 0x8f, RFREGOFFSETMASK, 0xa9c00);
		break;
	}
	/*	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]Set RXK setting!!!!\n"));*/

}


bool
_iqk_check_cal_8822b(
	struct PHY_DM_STRUCT			*p_dm_odm,
	u32				IQK_CMD
)
{
	bool		notready = true, fail = true;
	u32		delay_count = 0x0;

	while (notready) {
		if (odm_read_4byte(p_dm_odm, 0x1b00) == (IQK_CMD & 0xffffff0f)) {
			fail = (bool) odm_get_bb_reg(p_dm_odm, 0x1b08, BIT(26));
			notready = false;
		} else {
			ODM_delay_ms(1);
			delay_count++;
		}

		if (delay_count >= 50) {
			fail = true;
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
				     ("[IQK]IQK timeout!!!\n"));
			break;
		}
	}
	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
		     ("[IQK]delay count = 0x%x!!!\n", delay_count));
	return fail;
}


bool
_iqk_rx_iqk_gain_search_fail_8822b(
	struct PHY_DM_STRUCT			*p_dm_odm,
	u8		path,
	u8		step
)
{

	struct _IQK_INFORMATION	*p_iqk_info = &p_dm_odm->IQK_info;
	bool	fail = true;
	u32	IQK_CMD = 0x0, rf_reg0, tmp, bb_idx;

	odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008 | path << 1);

	if (step == RXIQK1)
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]============ S%d RXIQK GainSearch ============\n", path));

	if (step == RXIQK1)
		IQK_CMD = 0xf8000208 | (1 << (path + 4));
	else
		IQK_CMD = 0xf8000308 | (1 << (path + 4));

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE, ("[IQK]S%d GS%d_Trigger = 0x%x\n", path, step, IQK_CMD));

	odm_write_4byte(p_dm_odm, 0x1b00, IQK_CMD);
	odm_write_4byte(p_dm_odm, 0x1b00, IQK_CMD + 0x1);
	ODM_delay_ms(GS_delay_8822B);
	fail = _iqk_check_cal_8822b(p_dm_odm, IQK_CMD);

	odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008 | path << 1);
	odm_write_4byte(p_dm_odm, 0x1bcc, p_iqk_info->tmp1bcc);

	if (step == RXIQK2) {
		rf_reg0 = odm_get_rf_reg(p_dm_odm, path, 0x0, RFREGOFFSETMASK);
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE,
			     ("[IQK]S%d ==> RF0x0 = 0x%x\n", path, rf_reg0));
		tmp = (rf_reg0 & 0x1fe0) >> 5;
		p_iqk_info->lna_idx = tmp >> 5;
		bb_idx = tmp & 0x1f;

		if (bb_idx == 0x1) {
			if (p_iqk_info->lna_idx != 0x0)
				p_iqk_info->lna_idx--;
			fail = true;
		} else if (bb_idx == 0xa) {
			if (p_iqk_info->lna_idx != 0x7)
				p_iqk_info->lna_idx++;
			fail = true;
		} else
			fail = false;

		if (fail) {
			odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008 | path << 1);
			odm_write_4byte(p_dm_odm, 0x1b24, (odm_read_4byte(p_dm_odm, 0x1b24) & 0xffffe3ff) | (p_iqk_info->lna_idx << 10));
		}
	}

	return fail;
}

bool
_lok_one_shot_8822b(
	void		*p_dm_void,
	u8			path
)
{
	struct PHY_DM_STRUCT	*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _IQK_INFORMATION	*p_iqk_info = &p_dm_odm->IQK_info;
	u8		delay_count = 0, ii;
	bool		LOK_notready = false;
	u32		LOK_temp = 0;
	u32		IQK_CMD = 0x0;

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE,
		     ("[IQK]==========S%d LOK ==========\n", path));

	IQK_CMD = 0xf8000008 | (1 << (4 + path));

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE, ("[IQK]LOK_Trigger = 0x%x\n", IQK_CMD));

	odm_write_4byte(p_dm_odm, 0x1b00, IQK_CMD);
	odm_write_4byte(p_dm_odm, 0x1b00, IQK_CMD + 1);
	/*LOK: CMD ID = 0	{0xf8000018, 0xf8000028}*/
	/*LOK: CMD ID = 0	{0xf8000019, 0xf8000029}*/
	ODM_delay_ms(LOK_delay_8822B);

	delay_count = 0;
	LOK_notready = true;

	while (LOK_notready) {
		if (odm_read_4byte(p_dm_odm, 0x1b00) == (IQK_CMD & 0xffffff0f))
			LOK_notready = false;
		else
			LOK_notready = true;

		if (LOK_notready) {
			ODM_delay_ms(1);
			delay_count++;
		}

		if (delay_count >= 50) {
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
				     ("[IQK]S%d LOK timeout!!!\n", path));
			break;
		}
	}

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE,
		     ("[IQK]S%d ==> delay_count = 0x%x\n", path, delay_count));
	if (ODM_COMP_CALIBRATION) {
		if (!LOK_notready) {
			LOK_temp = odm_get_rf_reg(p_dm_odm, (enum odm_rf_radio_path_e)path, 0x58, RFREGOFFSETMASK);
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE, ("[IQK]0x58 = 0x%x\n", LOK_temp));
		} else
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE, ("[IQK]==>S%d LOK Fail!!!\n", path));
	}
	p_iqk_info->LOK_fail[path] = LOK_notready;
	return LOK_notready;
}




bool
_iqk_one_shot_8822b(
	void		*p_dm_void,
	u8		path,
	u8		idx
)
{
	struct PHY_DM_STRUCT	*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _IQK_INFORMATION	*p_iqk_info = &p_dm_odm->IQK_info;
	u8		delay_count = 0;
	bool		notready = true, fail = true, search_fail = true;
	u32		IQK_CMD = 0x0, tmp;
	u16		iqk_apply[2]	= {0xc94, 0xe94};

	if (idx == TXIQK)
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]============ S%d WBTXIQK ============\n", path));
	else if (idx == RXIQK1)
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]============ S%d WBRXIQK STEP1============\n", path));
	else
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]============ S%d WBRXIQK STEP2============\n", path));

	if (idx == TXIQK) {
		IQK_CMD = 0xf8000008 | ((*p_dm_odm->p_band_width + 4) << 8) | (1 << (path + 4));
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE, ("[IQK]TXK_Trigger = 0x%x\n", IQK_CMD));
		/*{0xf8000418, 0xf800042a} ==> 20 WBTXK (CMD = 4)*/
		/*{0xf8000518, 0xf800052a} ==> 40 WBTXK (CMD = 5)*/
		/*{0xf8000618, 0xf800062a} ==> 80 WBTXK (CMD = 6)*/
	} else if (idx == RXIQK1) {
		if (*p_dm_odm->p_band_width == 2)
			IQK_CMD = 0xf8000808 | (1 << (path + 4));
		else
			IQK_CMD = 0xf8000708 | (1 << (path + 4));
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE, ("[IQK]RXK1_Trigger = 0x%x\n", IQK_CMD));
		/*{0xf8000718, 0xf800072a} ==> 20 WBTXK (CMD = 7)*/
		/*{0xf8000718, 0xf800072a} ==> 40 WBTXK (CMD = 7)*/
		/*{0xf8000818, 0xf800082a} ==> 80 WBTXK (CMD = 8)*/
	} else if (idx == RXIQK2) {
		IQK_CMD = 0xf8000008 | ((*p_dm_odm->p_band_width + 9) << 8) | (1 << (path + 4));
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE, ("[IQK]RXK2_Trigger = 0x%x\n", IQK_CMD));
		/*{0xf8000918, 0xf800092a} ==> 20 WBRXK (CMD = 9)*/
		/*{0xf8000a18, 0xf8000a2a} ==> 40 WBRXK (CMD = 10)*/
		/*{0xf8000b18, 0xf8000b2a} ==> 80 WBRXK (CMD = 11)*/
		odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008 | path << 1);
		odm_write_4byte(p_dm_odm, 0x1b24, (odm_read_4byte(p_dm_odm, 0x1b24) & 0xffffe3ff) | ((p_iqk_info->lna_idx & 0x7) << 10));
	}

	odm_write_4byte(p_dm_odm, 0x1b00, IQK_CMD);
	odm_write_4byte(p_dm_odm, 0x1b00, IQK_CMD + 0x1);
	ODM_delay_ms(WBIQK_delay_8822B);

	while (notready) {
		if (odm_read_4byte(p_dm_odm, 0x1b00) == (IQK_CMD & 0xffffff0f))
			notready = false;
		else
			notready = true;

		if (notready) {
			ODM_delay_ms(1);
			delay_count++;
		} else {
			fail = (bool) odm_get_bb_reg(p_dm_odm, 0x1b08, BIT(26));
			break;
		}

		if (delay_count >= 50) {
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
				     ("[IQK]S%d IQK timeout!!!\n", path));
			break;
		}
	}

	if (p_dm_odm->debug_components && ODM_COMP_CALIBRATION) {
		odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008 | path << 1);
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE,
			("[IQK]S%d ==> 0x1b00 = 0x%x, 0x1b08 = 0x%x\n", path, odm_read_4byte(p_dm_odm, 0x1b00), odm_read_4byte(p_dm_odm, 0x1b08)));
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE,
			("[IQK]S%d ==> delay_count = 0x%x\n", path, delay_count));
		if (idx != TXIQK)
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE,
				("[IQK]S%d ==> RF0x0 = 0x%x, RF0x56 = 0x%x\n", path, odm_get_rf_reg(p_dm_odm, path, 0x0, RFREGOFFSETMASK), odm_get_rf_reg(p_dm_odm, path, 0x56, RFREGOFFSETMASK)));
	}

	odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008 | path << 1);

	if (idx == RXIQK2) {
		p_iqk_info->RXIQK_AGC[0][path] = odm_get_rf_reg(p_dm_odm, path, 0x0, RFREGOFFSETMASK) >> 4;
		odm_write_4byte(p_dm_odm, 0x1b38, 0x20000000);

		if (!fail)												/*RXIQK success*/
			odm_set_bb_reg(p_dm_odm, iqk_apply[path], (BIT(11) | BIT(10)), 0x1);
		else
			odm_set_bb_reg(p_dm_odm, iqk_apply[path], (BIT(11) | BIT(10)), 0x0);
	}

	if (idx == TXIQK)
		p_iqk_info->IQK_fail_report[0][path][TXIQK] = fail;
	else
		p_iqk_info->IQK_fail_report[0][path][RXIQK] = fail;

	return fail;
}


bool
_iqk_rx_iqk_by_path_8822b(
	void		*p_dm_void,
	u8		path
)
{
	struct PHY_DM_STRUCT	*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _IQK_INFORMATION	*p_iqk_info = &p_dm_odm->IQK_info;
	bool		KFAIL = true, gonext;
	u8		i;

#if 1
	/*while (1) {*/
	switch (p_iqk_info->rxiqk_step) {
	case 1:		/*gain search_RXK1*/
		_iqk_rxk1_setting_8822b(p_dm_odm, path);
		gonext = false;
		while (1) {
			KFAIL = _iqk_rx_iqk_gain_search_fail_8822b(p_dm_odm, path, RXIQK1);
			if (KFAIL && (p_iqk_info->gs_retry_count[0][path][RXIQK1] < 2))
				p_iqk_info->gs_retry_count[0][path][RXIQK1]++;
			else if (KFAIL) {
				p_iqk_info->RXIQK_fail_code[0][path] = 0;
				p_iqk_info->rxiqk_step = 5;
				gonext = true;
			} else {
				p_iqk_info->rxiqk_step++;
				gonext = true;
			}

			if (gonext)
				break;
		}
		break;
	case 2:		/*gain search_RXK2*/
		_iqk_rxk2_setting_8822b(p_dm_odm, path);
		while (1) {
			KFAIL = _iqk_rx_iqk_gain_search_fail_8822b(p_dm_odm, path, RXIQK2);
			if (KFAIL && (p_iqk_info->gs_retry_count[0][path][RXIQK2] < 2))
				p_iqk_info->gs_retry_count[0][path][RXIQK2]++;
			else {
				p_iqk_info->rxiqk_step++;
				break;
			}
		}
		break;
	case 3:		/*RXK1*/
		_iqk_rxk1_setting_8822b(p_dm_odm, path);
		gonext = false;
		while (1) {
			KFAIL = _iqk_one_shot_8822b(p_dm_odm, path, RXIQK1);
			if (KFAIL && (p_iqk_info->retry_count[0][path][RXIQK1] < 2))
				p_iqk_info->retry_count[0][path][RXIQK1]++;
			else if (KFAIL) {
				p_iqk_info->RXIQK_fail_code[0][path] = 1;
				p_iqk_info->rxiqk_step = 5;
				gonext = true;
			} else {
				p_iqk_info->rxiqk_step++;
				gonext = true;
			}
			if (gonext)
				break;
		}
		break;
	case 4:		/*RXK2*/
		_iqk_rxk2_setting_8822b(p_dm_odm, path);
		gonext = false;
		while (1) {
			KFAIL = _iqk_one_shot_8822b(p_dm_odm, path,	RXIQK2);
			if (KFAIL && (p_iqk_info->retry_count[0][path][RXIQK2] < 2))
				p_iqk_info->retry_count[0][path][RXIQK2]++;
			else if (KFAIL) {
				p_iqk_info->RXIQK_fail_code[0][path] = 2;
				p_iqk_info->rxiqk_step = 5;
				gonext = true;
			} else {
				p_iqk_info->rxiqk_step++;
				gonext = true;
			}
			if (gonext)
				break;
		}
		break;
	}

	return KFAIL;

#endif
}


void
_iqk_iqk_by_path_8822b(
	void		*p_dm_void
)
{
	struct PHY_DM_STRUCT	*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _IQK_INFORMATION	*p_iqk_info = &p_dm_odm->IQK_info;
	bool		KFAIL = true;
	u8		i;

	/*	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE, ("[IQK]iqk_step = 0x%x\n", p_dm_odm->rf_calibrate_info.iqk_step)); */
#if 1
	switch (p_dm_odm->rf_calibrate_info.iqk_step) {
	case 1:		/*S0 LOK*/
#if 1
		_iqk_lok_setting_8822b(p_dm_odm, ODM_RF_PATH_A);
		_lok_one_shot_8822b(p_dm_odm, ODM_RF_PATH_A);
#endif
		p_dm_odm->rf_calibrate_info.iqk_step++;
		break;
	case 2:		/*S1 LOK*/
#if 1
		_iqk_lok_setting_8822b(p_dm_odm, ODM_RF_PATH_B);
		_lok_one_shot_8822b(p_dm_odm, ODM_RF_PATH_B);
#endif
		p_dm_odm->rf_calibrate_info.iqk_step++;
		break;
	case 3:		/*S0 TXIQK*/
#if 1
		_iqk_txk_setting_8822b(p_dm_odm, ODM_RF_PATH_A);
		KFAIL = _iqk_one_shot_8822b(p_dm_odm, ODM_RF_PATH_A, TXIQK);
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE, ("[IQK]S0TXK KFail = 0x%x\n", KFAIL));

		if (KFAIL && (p_iqk_info->retry_count[0][ODM_RF_PATH_A][TXIQK] < 3))
			p_iqk_info->retry_count[0][ODM_RF_PATH_A][TXIQK]++;
		else
#endif
			p_dm_odm->rf_calibrate_info.iqk_step++;
		break;
	case 4:		/*S1 TXIQK*/
#if 1
		_iqk_txk_setting_8822b(p_dm_odm, ODM_RF_PATH_B);
		KFAIL = _iqk_one_shot_8822b(p_dm_odm, ODM_RF_PATH_B,	TXIQK);
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE, ("[IQK]S1TXK KFail = 0x%x\n", KFAIL));

		if (KFAIL && p_iqk_info->retry_count[0][ODM_RF_PATH_B][TXIQK] < 3)
			p_iqk_info->retry_count[0][ODM_RF_PATH_B][TXIQK]++;
		else
#endif
			p_dm_odm->rf_calibrate_info.iqk_step++;
		break;
	case 5:		/*S0 RXIQK*/
		KFAIL = _iqk_rx_iqk_by_path_8822b(p_dm_odm, ODM_RF_PATH_A);
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE, ("[IQK]S0RXK KFail = 0x%x\n", KFAIL));
		if (p_iqk_info->rxiqk_step == 5) {
			p_dm_odm->rf_calibrate_info.iqk_step++;
			p_iqk_info->rxiqk_step = 1;
			if (KFAIL)
				ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
					("[IQK]S0RXK fail code: %d!!!\n", p_iqk_info->RXIQK_fail_code[0][ODM_RF_PATH_A]));
		}
		break;
	case 6:		/*S1 RXIQK*/
		KFAIL = _iqk_rx_iqk_by_path_8822b(p_dm_odm, ODM_RF_PATH_B);
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE, ("[IQK]S1RXK KFail = 0x%x\n", KFAIL));
		if (p_iqk_info->rxiqk_step == 5) {
			p_dm_odm->rf_calibrate_info.iqk_step++;
			p_iqk_info->rxiqk_step = 1;
			if (KFAIL)
				ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
					("[IQK]S1RXK fail code: %d!!!\n", p_iqk_info->RXIQK_fail_code[0][ODM_RF_PATH_B]));
		}
		break;
	}

	if (p_dm_odm->rf_calibrate_info.iqk_step == 7) {
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE,
			     ("[IQK]==========LOK summary ==========\n"));
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
			("[IQK]PathA_LOK_notready = %d, PathB_LOK1_notready = %d\n",
			p_iqk_info->LOK_fail[ODM_RF_PATH_A], p_iqk_info->LOK_fail[ODM_RF_PATH_B]));
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE,
			     ("[IQK]==========IQK summary ==========\n"));
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
			("[IQK]PathA_TXIQK_fail = %d, PathB_TXIQK_fail = %d\n",
			p_iqk_info->IQK_fail_report[0][ODM_RF_PATH_A][TXIQK], p_iqk_info->IQK_fail_report[0][ODM_RF_PATH_B][TXIQK]));
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
			("[IQK]PathA_RXIQK_fail = %d, PathB_RXIQK_fail = %d\n",
			p_iqk_info->IQK_fail_report[0][ODM_RF_PATH_A][RXIQK], p_iqk_info->IQK_fail_report[0][ODM_RF_PATH_B][RXIQK]));
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
			("[IQK]PathA_TXIQK_retry = %d, PathB_TXIQK_retry = %d\n",
			p_iqk_info->retry_count[0][ODM_RF_PATH_A][TXIQK], p_iqk_info->retry_count[0][ODM_RF_PATH_B][TXIQK]));
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
			("[IQK]PathA_RXK1_retry = %d, PathA_RXK2_retry = %d, PathB_RXK1_retry = %d, PathB_RXK2_retry = %d\n",
			p_iqk_info->retry_count[0][ODM_RF_PATH_A][RXIQK1], p_iqk_info->retry_count[0][ODM_RF_PATH_A][RXIQK2],
			p_iqk_info->retry_count[0][ODM_RF_PATH_B][RXIQK1], p_iqk_info->retry_count[0][ODM_RF_PATH_B][RXIQK2]));
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
			("[IQK]PathA_GS1_retry = %d, PathA_GS2_retry = %d, PathB_GS1_retry = %d, PathB_GS2_retry = %d\n",
			p_iqk_info->gs_retry_count[0][ODM_RF_PATH_A][RXIQK1], p_iqk_info->gs_retry_count[0][ODM_RF_PATH_A][RXIQK2],
			p_iqk_info->gs_retry_count[0][ODM_RF_PATH_B][RXIQK1], p_iqk_info->gs_retry_count[0][ODM_RF_PATH_B][RXIQK2]));
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE,
			     ("[IQK]================================\n"));

		for (i = 0; i < 2; i++) {
			odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008 | i << 1);
			odm_write_4byte(p_dm_odm, 0x1b2c, 0x7);
			odm_write_4byte(p_dm_odm, 0x1bcc, 0x0);
		}
	}
#endif
}

void
_iqk_start_iqk_8822b(
	struct PHY_DM_STRUCT		*p_dm_odm
)
{
	u32 tmp;

	/*GNT_WL = 1*/
	tmp = odm_get_rf_reg(p_dm_odm, ODM_RF_PATH_A, 0x1, RFREGOFFSETMASK);
	tmp = tmp | BIT(5) | BIT(0);
	odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_A, 0x1, RFREGOFFSETMASK, tmp);

	tmp = odm_get_rf_reg(p_dm_odm, ODM_RF_PATH_B, 0x1, RFREGOFFSETMASK);
	tmp = tmp | BIT(5) | BIT(0);
	odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_B, 0x1, RFREGOFFSETMASK, tmp);

	_iqk_iqk_by_path_8822b(p_dm_odm);


}

void
_iq_calibrate_8822b_init(
	void		*p_dm_void
)
{
	struct PHY_DM_STRUCT	*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _IQK_INFORMATION	*p_iqk_info = &p_dm_odm->IQK_info;
	u8	i, j, k, m;

	if (p_iqk_info->iqk_times == 0) {
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]=====>PHY_IQCalibrate_8822B_Init\n"));

		for (i = 0; i < SS_8822B; i++) {
			for (j = 0; j < 2; j++) {
				p_iqk_info->LOK_fail[i] = true;
				p_iqk_info->IQK_fail[j][i] = true;
				p_iqk_info->iqc_matrix[j][i] = 0x20000000;
			}
		}

		for (i = 0; i < 2; i++) {
			p_iqk_info->iqk_channel[i] = 0x0;

			for (j = 0; j < SS_8822B; j++) {
				p_iqk_info->LOK_IDAC[i][j] = 0x0;
				p_iqk_info->RXIQK_AGC[i][j] = 0x0;
				p_iqk_info->bypass_iqk[i][j] = 0x0;

				for (k = 0; k < 2; k++) {
					p_iqk_info->IQK_fail_report[i][j][k] = true;
					for (m = 0; m < 8; m++) {
						p_iqk_info->IQK_CFIR_real[i][j][k][m] = 0x0;
						p_iqk_info->IQK_CFIR_imag[i][j][k][m] = 0x0;
					}
				}

				for (k = 0; k < 3; k++)
					p_iqk_info->retry_count[i][j][k] = 0x0;

			}
		}
	}
}


void
_phy_iq_calibrate_8822b(
	struct PHY_DM_STRUCT		*p_dm_odm,
	bool			reset
)
{

	u32	MAC_backup[MAC_REG_NUM_8822B], BB_backup[BB_REG_NUM_8822B], RF_backup[RF_REG_NUM_8822B][SS_8822B];
	u32	backup_mac_reg[MAC_REG_NUM_8822B] = {0x520, 0x550};
	u32	backup_bb_reg[BB_REG_NUM_8822B] = {0x808, 0x90c, 0xc00, 0xcb0, 0xcb4, 0xcbc, 0xe00, 0xeb0, 0xeb4, 0xebc, 0x1990, 0x9a4, 0xa04};
	u32	backup_rf_reg[RF_REG_NUM_8822B] = {0xdf, 0x8f, 0x65, 0x0, 0x1};
	u8	i, j;

	struct _IQK_INFORMATION	*p_iqk_info = &p_dm_odm->IQK_info;

	if (!p_dm_odm->mp_mode)
		if (_iqk_reload_iqk_8822b(p_dm_odm, reset))
			return;

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE,
		     ("[IQK]==========IQK strat!!!!!==========\n"));

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
		("[IQK]p_band_type = %s, band_width = %d, ExtPA2G = %d, ext_pa_5g = %d\n", (*p_dm_odm->p_band_type == ODM_BAND_5G) ? "5G" : "2G", *p_dm_odm->p_band_width, p_dm_odm->ext_pa, p_dm_odm->ext_pa_5g));
	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
		("[IQK]Interface = %d, cut_version = %x\n", p_dm_odm->support_interface, p_dm_odm->cut_version));

	p_iqk_info->iqk_times++;

	p_dm_odm->rf_calibrate_info.iqk_total_progressing_time = 0;
	p_dm_odm->rf_calibrate_info.iqk_step = 1;
	p_iqk_info->rxiqk_step = 1;

	_iqk_backup_iqk_8822b(p_dm_odm, 0);
	_iqk_backup_mac_bb_8822b(p_dm_odm, MAC_backup, BB_backup, backup_mac_reg, backup_bb_reg);
	_iqk_backup_rf_8822b(p_dm_odm, RF_backup, backup_rf_reg);

	_iqk_configure_macbb_8822b(p_dm_odm);
	_iqk_afe_setting_8822b(p_dm_odm, true);
	_iqk_rfe_setting_8822b(p_dm_odm, false);
	_iqk_agc_bnd_int_8822b(p_dm_odm);
	_iqk_rf_setting_8822b(p_dm_odm);

	while (1) {
		if (!p_dm_odm->mp_mode)
			p_dm_odm->rf_calibrate_info.iqk_start_time = odm_get_current_time(p_dm_odm);

		_iqk_start_iqk_8822b(p_dm_odm);

		if (!p_dm_odm->mp_mode) {
			p_dm_odm->rf_calibrate_info.iqk_progressing_time = odm_get_progressing_time(p_dm_odm, p_dm_odm->rf_calibrate_info.iqk_start_time);
			p_dm_odm->rf_calibrate_info.iqk_total_progressing_time += odm_get_progressing_time(p_dm_odm, p_dm_odm->rf_calibrate_info.iqk_start_time);
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
				("[IQK]IQK progressing_time = %lld ms\n", p_dm_odm->rf_calibrate_info.iqk_progressing_time));
		}

		if (p_dm_odm->rf_calibrate_info.iqk_step == 7)
			break;
	};

	_iqk_backup_iqk_8822b(p_dm_odm, 1);
	_iqk_afe_setting_8822b(p_dm_odm, false);
	_iqk_restore_mac_bb_8822b(p_dm_odm, MAC_backup, BB_backup, backup_mac_reg, backup_bb_reg);
	_iqk_restore_rf_8822b(p_dm_odm, backup_rf_reg, RF_backup);
	_iqk_fill_iqk_report_8822b(p_dm_odm, 0);

	p_dm_odm->rf_calibrate_info.iqk_total_progressing_time += odm_get_progressing_time(p_dm_odm, p_dm_odm->rf_calibrate_info.iqk_start_time);
	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD,
		("[IQK]Total IQK progressing_time = %lld ms\n", p_dm_odm->rf_calibrate_info.iqk_total_progressing_time));

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE,
		     ("[IQK]==========IQK end!!!!!==========\n"));
}


void
_phy_iq_calibrate_by_fw_8822b(
	void		*p_dm_void,
	u8		clear
)
{
}


/*IQK version:v3.2 , NCTL v0.6*/
/*1.add LNA boundary of RX gain search*/
void
phy_iq_calibrate_8822b(
	void		*p_dm_void,
	bool		clear
)
{
	struct PHY_DM_STRUCT	*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;

	u32 counter = 0x0;

#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	struct _ADAPTER		*p_adapter = p_dm_odm->adapter;
	HAL_DATA_TYPE	*p_hal_data = GET_HAL_DATA(p_adapter);

#if (MP_DRIVER == 1)
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PMPT_CONTEXT	p_mpt_ctx = &(p_adapter->mpt_ctx);
#else
	PMPT_CONTEXT	p_mpt_ctx = &(p_adapter->mppriv.mpt_ctx);
#endif
#endif
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN))
	if (odm_check_power_status(p_adapter) == false)
		return;
#endif

#if MP_DRIVER == 1
	if (p_mpt_ctx->is_single_tone || p_mpt_ctx->is_carrier_suppression)
		return;
#endif

#endif

	p_dm_odm->iqk_fw_offload = 0;

	/*FW IQK*/
	if (p_dm_odm->iqk_fw_offload) {
		if (!p_dm_odm->rf_calibrate_info.is_iqk_in_progress) {
			odm_acquire_spin_lock(p_dm_odm, RT_IQK_SPINLOCK);
			p_dm_odm->rf_calibrate_info.is_iqk_in_progress = true;
			odm_release_spin_lock(p_dm_odm, RT_IQK_SPINLOCK);

			p_dm_odm->rf_calibrate_info.iqk_start_time = odm_get_current_time(p_dm_odm);

			odm_write_4byte(p_dm_odm, 0x1b00, 0xf8000008);
			odm_set_bb_reg(p_dm_odm, 0x1bf0, 0xff000000, 0xff);
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE,
				("[IQK]0x1bf0 = 0x%x\n", odm_read_4byte(p_dm_odm, 0x1bf0)));

			_phy_iq_calibrate_by_fw_8822b(p_dm_odm, clear);

			while (1) {
				if (((odm_read_4byte(p_dm_odm, 0x1bf0) >> 24) == 0x7f) || (counter > 300))
					break;

				counter++;
				ODM_delay_ms(1);
			};

			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_TRACE,
				     ("[IQK]counter = %d\n", counter));

			p_dm_odm->rf_calibrate_info.iqk_progressing_time = odm_get_progressing_time(p_dm_odm, p_dm_odm->rf_calibrate_info.iqk_start_time);

			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]IQK progressing_time = %lld ms\n", p_dm_odm->rf_calibrate_info.iqk_progressing_time));

			odm_acquire_spin_lock(p_dm_odm, RT_IQK_SPINLOCK);
			p_dm_odm->rf_calibrate_info.is_iqk_in_progress = false;
			odm_release_spin_lock(p_dm_odm, RT_IQK_SPINLOCK);
		}	else
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("== Return the IQK CMD, because the IQK in Progress ==\n"));

	} else {

		_iq_calibrate_8822b_init(p_dm_void);

		if (!p_dm_odm->rf_calibrate_info.is_iqk_in_progress) {

			odm_acquire_spin_lock(p_dm_odm, RT_IQK_SPINLOCK);
			p_dm_odm->rf_calibrate_info.is_iqk_in_progress = true;
			odm_release_spin_lock(p_dm_odm, RT_IQK_SPINLOCK);
			if (p_dm_odm->mp_mode)
				p_dm_odm->rf_calibrate_info.iqk_start_time = odm_get_current_time(p_dm_odm);

#if (DM_ODM_SUPPORT_TYPE & (ODM_CE))
			_phy_iq_calibrate_8822b(p_dm_odm, clear);
			/*DBG_871X("%s,%d, do IQK %u ms\n", __func__, __LINE__, rtw_get_passing_time_ms(time_iqk));*/
#else
			_phy_iq_calibrate_8822b(p_dm_odm, clear);
#endif
			if (p_dm_odm->mp_mode) {
				p_dm_odm->rf_calibrate_info.iqk_progressing_time = odm_get_progressing_time(p_dm_odm, p_dm_odm->rf_calibrate_info.iqk_start_time);
				ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]IQK progressing_time = %lld ms\n", p_dm_odm->rf_calibrate_info.iqk_progressing_time));
			}
			odm_acquire_spin_lock(p_dm_odm, RT_IQK_SPINLOCK);
			p_dm_odm->rf_calibrate_info.is_iqk_in_progress = false;
			odm_release_spin_lock(p_dm_odm, RT_IQK_SPINLOCK);
		} else
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]== Return the IQK CMD, because the IQK in Progress ==\n"));

	}

#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	_iqk_iqk_fail_report_8822b(p_dm_odm);
#endif

}

#endif
