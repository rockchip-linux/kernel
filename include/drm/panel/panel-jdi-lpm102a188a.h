/*
 * Copyright (C) 2014 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _PANEL_JDI_LPM102A188A_H_
#define _PANEL_JDI_LPM102A188A_H_

#define MIPI_DCS_RSP_WRITE_DISPLAY_BRIGHTNESS		0x51
#define RSP_WRITE_DISPLAY_BRIGHTNESS(x)			((x) & 0xFF)

#define MIPI_DCS_RSP_WRITE_CONTROL_DISPLAY		0x53
#define RSP_WRITE_CONTROL_DISPLAY_BL_ON			(1 << 2)
#define RSP_WRITE_CONTROL_DISPLAY_DD_ON			(1 << 3)
#define RSP_WRITE_CONTROL_DISPLAY_BCTRL_LEDPWM		(1 << 5)

#define MIPI_DCS_RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL	0x55
#define RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL_C_OFF	(0 << 0)
#define RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL_C_UI	(1 << 0)
#define RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL_C_STILL	(2 << 0)
#define RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL_C_VIDEO	(3 << 0)
#define RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL_SRE_USER	(0 << 4)
#define RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL_SRE_WEAK	(1 << 4)
#define RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL_SRE_MID	(2 << 4)
#define RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL_SRE_HIGH	(3 << 4)
#define RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL_SRE_ON	(1 << 6)


#endif
