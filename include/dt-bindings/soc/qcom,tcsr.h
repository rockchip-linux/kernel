/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __DT_BINDINGS_QCOM_TCSR_H
#define __DT_BINDINGS_QCOM_TCSR_H

#define TCSR_USB_SELECT_USB3_P0		0x1
#define TCSR_USB_SELECT_USB3_P1		0x2
#define TCSR_USB_SELECT_USB3_DUAL	0x3

#define GSBI1				1
#define GSBI2				2
#define GSBI3				3
#define GSBI4				4
#define GSBI5				5
#define GSBI6				6
#define GSBI7				7

#define ADM_CRCI_QUP			0
#define ADM_CRCI_UART			1

#define TCSR_ADM_CRCI_SEL(gsbi, sel)	(sel*0x3 << ((gsbi-1)*2))

#endif
