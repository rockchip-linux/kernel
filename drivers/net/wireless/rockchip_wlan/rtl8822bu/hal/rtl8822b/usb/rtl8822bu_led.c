/******************************************************************************
 *
 * Copyright(c) 2015 - 2016 Realtek Corporation. All rights reserved.
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
#define _RTL8822BU_LED_C_

#include <drv_types.h>		/* PADAPTER */
#include <hal_data.h>		/* PHAL_DATA_TYPE */
#include <hal_com_led.h>	/* PLED_USB */

/*
 * =============================================================================
 * LED object.
 * =============================================================================
 */


/*
 * =============================================================================
 * Prototype of protected function.
 * =============================================================================
 */

/*
 * =============================================================================
 * LED routines.
 * =============================================================================
 */

/*
 * Description:
 * Turn on LED according to LedPin specified.
 */
void swledon(PADAPTER padapter, PLED_USB pLed)
{
	u8 LedCfg;
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);

	if (RTW_CANNOT_RUN(padapter))
		return;

	pLed->bLedOn = _TRUE;
}


/*
 * Description:
 * Turn off LED according to LedPin specified.
 */
void swledoff(PADAPTER padapter, PLED_USB pLed)
{
	u8 LedCfg;
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);

	if (RTW_CANNOT_RUN(padapter))
		goto exit;

exit:
	pLed->bLedOn = _FALSE;
}

/*
 * =============================================================================
 * Interface to manipulate LED objects.
 * =============================================================================
 */

/*
 * =============================================================================
 * Default LED behavior.
 * =============================================================================
 */

/*
 * Description:
 * Initialize all LED_871x objects.
 */
void rtl8822bu_initswleds(PADAPTER padapter)
{
}

/*
 * Description:
 * DeInitialize all LED_819xUsb objects.
 */
void rtl8822bu_deinitswleds(PADAPTER padapter)
{
}
