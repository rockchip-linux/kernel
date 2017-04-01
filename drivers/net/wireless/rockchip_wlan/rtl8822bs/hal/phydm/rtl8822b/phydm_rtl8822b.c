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

#if 0
	/* ============================================================
	*  include files
	* ============================================================ */
#endif

#include "mp_precomp.h"
#include "../phydm_precomp.h"

#if (RTL8822B_SUPPORT == 1)


void
phydm_dynamic_switch_htstf_mumimo_8822b(
	struct PHY_DM_STRUCT		*p_dm_odm
)
{
	/*if rssi > 40dBm, enable HT-STF gain controller, otherwise, if rssi < 40dBm, disable the controller*/
	/*add by Chun-Hung Ho 20160711 */
	if (p_dm_odm->rssi_min >= 40)
		odm_set_bb_reg(p_dm_odm, 0x8d8, BIT(17), 0x1);
	else if (p_dm_odm->rssi_min < 35)
		odm_set_bb_reg(p_dm_odm, 0x8d8, BIT(17), 0x0);

	ODM_RT_TRACE(p_dm_odm, ODM_PHY_CONFIG, ODM_DBG_LOUD, ("%s, rssi_min = %d\n", __func__, p_dm_odm->rssi_min));
}

static
void
_setTxACaliValue(
	struct PHY_DM_STRUCT		*p_dm_odm,
	u8						eRFPath,
	u8 						offset,
	u8 						TxABiaOffset
	)
{
	u32 modiTxAValue = 0;
	u8 tmp1Byte = 0;
	boolean bMinus = false;
	u8 compValue = 0;

	
		switch (offset) {
		case 0x0:
			odm_set_rf_reg(p_dm_odm, eRFPath, 0x18, 0xFFFFF, 0X10124);
			break;
		case 0x1:
			odm_set_rf_reg(p_dm_odm, eRFPath, 0x18, 0xFFFFF, 0X10524);
			break;
		case 0x2:
			odm_set_rf_reg(p_dm_odm, eRFPath, 0x18, 0xFFFFF, 0X10924);
			break;
		case 0x3:
			odm_set_rf_reg(p_dm_odm, eRFPath, 0x18, 0xFFFFF, 0X10D24);
			break;
		case 0x4:
			odm_set_rf_reg(p_dm_odm, eRFPath, 0x18, 0xFFFFF, 0X30164);
			break;
		case 0x5:
			odm_set_rf_reg(p_dm_odm, eRFPath, 0x18, 0xFFFFF, 0X30564);
			break;
		case 0x6:
			odm_set_rf_reg(p_dm_odm, eRFPath, 0x18, 0xFFFFF, 0X30964);
			break;
		case 0x7:
			odm_set_rf_reg(p_dm_odm, eRFPath, 0x18, 0xFFFFF, 0X30D64);
			break;
		case 0x8:
			odm_set_rf_reg(p_dm_odm, eRFPath, 0x18, 0xFFFFF, 0X50195);
			break;
		case 0x9:
			odm_set_rf_reg(p_dm_odm, eRFPath, 0x18, 0xFFFFF, 0X50595);
			break;
		case 0xa:
			odm_set_rf_reg(p_dm_odm, eRFPath, 0x18, 0xFFFFF, 0X50995);
			break;
		case 0xb:
			odm_set_rf_reg(p_dm_odm, eRFPath, 0x18, 0xFFFFF, 0X50D95);
			break;
		default:
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_COMMON, ODM_DBG_LOUD, ("Invalid TxA band offset...\n"));
			return;
			break;
	}

	/* Get TxA value */
	modiTxAValue = odm_get_rf_reg(p_dm_odm, eRFPath, 0x61, 0xFFFFF);
	tmp1Byte = (u8)modiTxAValue&(BIT(3)|BIT(2)|BIT(1)|BIT(0));

	/* check how much need to calibration */
		switch (TxABiaOffset) {
		case 0xF6:
			bMinus = true;
			compValue = 3;
			break;
			
		case 0xF4:
			bMinus = true;
			compValue = 2;
			break;
			
		case 0xF2:
			bMinus = true;
			compValue = 1;
			break;
			
		case 0xF3:
			bMinus = false;
			compValue = 1;
			break;
			
		case 0xF5:
			bMinus = false;
			compValue = 2;	
			break;
			
		case 0xF7:
			bMinus = false;
			compValue = 3;
			break;
			
		case 0xF9:
			bMinus = false;
			compValue = 4;
			break;
		
		/* do nothing case */
		case 0xF0:
		default:
			ODM_RT_TRACE(p_dm_odm, ODM_COMP_COMMON, ODM_DBG_LOUD, ("No need to do TxA bias current calibration\n"));
			return;
			break;
	}

	/* calc correct value to calibrate */
	if (bMinus) {
		if (tmp1Byte >= compValue) {
			tmp1Byte -= compValue;
			//modiTxAValue += tmp1Byte;
		} else {
			tmp1Byte = 0;
		}
	} else {
		tmp1Byte += compValue;
		if (tmp1Byte >= 7) {
			tmp1Byte = 7;
		}
	}

	/* Write back to RF reg */
	odm_set_rf_reg(p_dm_odm, eRFPath, 0x30, 0xFFFF, (offset<<12|(modiTxAValue&0xFF0)|tmp1Byte));
}

static
void
_txaBiasCali4eachPath(
	struct PHY_DM_STRUCT		*p_dm_odm,
	u8	 eRFPath,
	u8	 efuseValue
	)
{
	/* switch on set TxA bias */
	odm_set_rf_reg(p_dm_odm, eRFPath, 0xEF, 0xFFFFF, 0x200);

	/* Set 12 sets of TxA value */
	_setTxACaliValue(p_dm_odm, eRFPath, 0x0, efuseValue);
	_setTxACaliValue(p_dm_odm, eRFPath, 0x1, efuseValue);
	_setTxACaliValue(p_dm_odm, eRFPath, 0x2, efuseValue);
	_setTxACaliValue(p_dm_odm, eRFPath, 0x3, efuseValue);
	_setTxACaliValue(p_dm_odm, eRFPath, 0x4, efuseValue);
	_setTxACaliValue(p_dm_odm, eRFPath, 0x5, efuseValue);
	_setTxACaliValue(p_dm_odm, eRFPath, 0x6, efuseValue);
	_setTxACaliValue(p_dm_odm, eRFPath, 0x7, efuseValue);
	_setTxACaliValue(p_dm_odm, eRFPath, 0x8, efuseValue);
	_setTxACaliValue(p_dm_odm, eRFPath, 0x9, efuseValue);
	_setTxACaliValue(p_dm_odm, eRFPath, 0xa, efuseValue);
	_setTxACaliValue(p_dm_odm, eRFPath, 0xb, efuseValue);

	// switch off set TxA bias
	odm_set_rf_reg(p_dm_odm, eRFPath, 0xEF, 0xFFFFF, 0x0);
}

/* for 8822B PCIE D-cut patch only */
/* Normal driver and MP driver need this patch */

void
phydm_txcurrentcalibration(
	struct PHY_DM_STRUCT		*p_dm_odm
	)
{
	u8			efuse0x3D8, efuse0x3D7;
	u32			origRF0x18PathA = 0, origRF0x18PathB = 0;

	/* save original 0x18 value */
	origRF0x18PathA = odm_get_rf_reg(p_dm_odm, ODM_RF_PATH_A, 0x18, 0xFFFFF);
	origRF0x18PathB = odm_get_rf_reg(p_dm_odm, ODM_RF_PATH_B, 0x18, 0xFFFFF);
	
	/* define efuse content */
		efuse0x3D8 = p_dm_odm->efuse0x3d8;
		efuse0x3D7 = p_dm_odm->efuse0x3d7;
	
	/* check efuse content to judge whether need to calibration or not */
	if (0xFF == efuse0x3D7) {
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_COMMON, ODM_DBG_LOUD, ("efuse content 0x3D7 == 0xFF, No need to do TxA cali\n"));
		return;
	}

	/* write RF register for calibration */
	_txaBiasCali4eachPath(p_dm_odm, ODM_RF_PATH_A, efuse0x3D7);
	_txaBiasCali4eachPath(p_dm_odm, ODM_RF_PATH_B, efuse0x3D8);
	
	/* restore original 0x18 value */
	odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_A, 0x18, 0xFFFFF, origRF0x18PathA);
	odm_set_rf_reg(p_dm_odm, ODM_RF_PATH_B, 0x18, 0xFFFFF, origRF0x18PathB);
}

void
phydm_hwsetting_8822b(
	struct PHY_DM_STRUCT		*p_dm_odm
)
{
	if (p_dm_odm->bhtstfenabled == TRUE)
	phydm_dynamic_switch_htstf_mumimo_8822b(p_dm_odm);
	else
		odm_set_bb_reg(p_dm_odm, 0x8d8, BIT(17), 0x1);
}

#endif	/* RTL8822B_SUPPORT == 1 */
