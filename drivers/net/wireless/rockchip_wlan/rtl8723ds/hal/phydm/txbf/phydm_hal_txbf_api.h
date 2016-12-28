/********************************************************************************/
/**/
/*Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.*/
/**/                                      
/*This program is free software; you can redistribute it and/or modify it*/
/*under the terms of version 2 of the GNU General Public License as*/
/*published by the Free Software Foundation.*/
/**/
/*This program is distributed in the hope that it will be useful, but WITHOUT*/
/*ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or*/
/*FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for*/
/*more details.*/
/*You should have received a copy of the GNU General Public License along with*/
/*this program; if not, write to the Free Software Foundation, Inc.,*/
/*51 Franklin Street, Fifth Floor, Boston, MA 02110, USA*/
/**/
/**/
/********************************************************************************/
#ifndef	__PHYDM_HAL_TXBF_API_H__
#define __PHYDM_HAL_TXBF_API_H__

u1Byte
phydm_get_beamforming_sounding_info(
	IN PVOID		pDM_VOID,
	IN pu2Byte	Troughput,
	IN u1Byte	Total_BFee_Num,
	IN pu1Byte	TxRate
	);


u1Byte
phydm_get_ndpa_rate(
	IN PVOID		pDM_VOID
	);

#endif
