/******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation.
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
 *****************************************************************************/
#include "rtl8723d_hal.h"
#ifdef CONFIG_RTW_SW_LED

/*
 * ================================================================================
 * LED object.
 * ================================================================================
 */


/*
 * ================================================================================
 * Prototype of protected function.
 * ================================================================================
 */

/*
 * ================================================================================
 * LED_819xUsb routines.
 * ================================================================================
 */

/*
 * Description:
 * Turn on LED according to LedPin specified.
 */
void
SwLedOn_8723DU(
	PADAPTER padapter,
	PLED_USB pLed
)
{
	u8 LedCfg;

	if (RTW_CANNOT_RUN(padapter))
		return;

	pLed->bLedOn = _TRUE;

}


/*
 * Description:
 * Turn off LED according to LedPin specified.
 */
void
SwLedOff_8723DU(
	PADAPTER padapter,
	PLED_USB pLed
)
{
	u8 LedCfg;

	if (RTW_CANNOT_RUN(padapter))
		goto exit;

exit:
	pLed->bLedOn = _FALSE;

}

/*
 * ================================================================================
 * Interface to manipulate LED objects.
 * ================================================================================
 */

/*
 * ================================================================================
 * Default LED behavior.
 * ================================================================================
 */

/*
 * Description:
 * Initialize all LED_871x objects.
 */
void
rtl8723du_InitSwLeds(
	PADAPTER padapter
)
{
	struct led_priv *pledpriv = &(padapter->ledpriv);

	pledpriv->LedControlHandler = LedControlUSB;

	pledpriv->SwLedOn = SwLedOn_8723DU;
	pledpriv->SwLedOff = SwLedOff_8723DU;

	InitLed(padapter, &(pledpriv->SwLed0), LED_PIN_LED0);

	InitLed(padapter, &(pledpriv->SwLed1), LED_PIN_LED1);
}


/*
 * Description:
 * DeInitialize all LED_819xUsb objects.
 */
void
rtl8723du_DeInitSwLeds(
	PADAPTER padapter
)
{
	struct led_priv *ledpriv = &(padapter->ledpriv);

	DeInitLed(&(ledpriv->SwLed0));
	DeInitLed(&(ledpriv->SwLed1));
}
#endif
