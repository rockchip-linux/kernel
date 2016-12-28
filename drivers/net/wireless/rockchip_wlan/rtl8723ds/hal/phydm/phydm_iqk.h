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

#ifndef	__PHYDMIQK_H__
#define __PHYDMIQK_H__

/*--------------------------Define Parameters-------------------------------*/
#define	LOK_delay 1
#define	WBIQK_delay 10
#define	TX_IQK 0
#define	RX_IQK 1
#define	NUM 4	
/*---------------------------End Define Parameters-------------------------------*/

typedef struct _IQK_INFORMATION {
	BOOLEAN		LOK_fail[NUM];
	BOOLEAN		IQK_fail[2][NUM];
	u4Byte		IQC_Matrix[2][NUM];
	u1Byte      IQKtimes;

} IQK_INFO, *PIQK_INFO;

#endif
