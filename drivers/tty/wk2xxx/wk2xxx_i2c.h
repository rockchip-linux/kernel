/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Wang Jianhui <wjh@rock-chips.com>
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
#ifndef	_SERIAL_WK2XXX_I2C_H
#define  _SERIAL_WK2XXX_I2C_H

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/console.h>

#define WK2XXX_GENA		0X00
#define WK2XXX_GRST		0X01
#define WK2XXX_GMUT		0X02
#define WK2XXX_GIER		0X10
#define WK2XXX_GIFR		0X11
#define WK2XXX_GPDIR		0X21
#define WK2XXX_GPDAT		0X31
#define WK2XXX_GPORT		1

#define WK2XXX_SPAGE		0X03

#define WK2XXX_SCR		0X04
#define WK2XXX_LCR		0X05
#define WK2XXX_FCR		0X06
#define WK2XXX_SIER		0X07
#define WK2XXX_SIFR		0X08
#define WK2XXX_TFCNT		0X09
#define WK2XXX_RFCNT		0X0A
#define WK2XXX_FSR		0X0B
#define WK2XXX_LSR		0X0C
#define WK2XXX_FDAT		0X0D
#define WK2XXX_FWCR		0X0D
#define WK2XXX_RS485		0X0F

#define WK2XXX_BAUD1		0X04
#define WK2XXX_BAUD0		0X05
#define WK2XXX_PRES		0X06
#define WK2XXX_RFTL		0X07
#define WK2XXX_TFTL		0X08
#define WK2XXX_FWTH		0X09
#define WK2XXX_FWTL		0X0A
#define WK2XXX_XON1		0X0B
#define WK2XXX_XOFF1		0X0C
#define WK2XXX_SADR		0X0D
#define WK2XXX_SAEN		0X0D
#define WK2XXX_RRSDLY		0X0F

#define WK2XXX_UT4EN		0x08
#define WK2XXX_UT3EN		0x04
#define WK2XXX_UT2EN		0x02
#define WK2XXX_UT1EN		0x01

#define WK2XXX_UT4SLEEP		0x80
#define WK2XXX_UT3SLEEP		0x40
#define WK2XXX_UT2SLEEP		0x20
#define WK2XXX_UT1SLEEP		0x10
#define WK2XXX_UT4RST		0x08
#define WK2XXX_UT3RST		0x04
#define WK2XXX_UT2RST		0x02
#define WK2XXX_UT1RST		0x01

#define WK2XXX_UT4IE		0x08
#define WK2XXX_UT3IE		0x04
#define WK2XXX_UT2IE		0x02
#define WK2XXX_UT1IE		0x01

#define WK2XXX_UT4INT		0x08
#define WK2XXX_UT3INT		0x04
#define WK2XXX_UT2INT		0x02
#define WK2XXX_UT1INT		0x01

#define WK2XXX_SPAGE0		0x00
#define WK2XXX_SPAGE1		0x01

#define WK2XXX_SLEEPEN		0x04
#define WK2XXX_TXEN		0x02
#define WK2XXX_RXEN		0x01

#define WK2XXX_BREAK		0x20
#define WK2XXX_IREN		0x10
#define WK2XXX_PAEN		0x08
#define WK2XXX_PAM1		0x04
#define WK2XXX_PAM0		0x02
#define WK2XXX_STPL		0x01

#define WK2XXX_FERR_IEN		0x80
#define WK2XXX_CTS_IEN		0x40
#define WK2XXX_RTS_IEN		0x20
#define WK2XXX_XOFF_IEN		0x10
#define WK2XXX_TFEMPTY_IEN	0x08
#define WK2XXX_TFTRIG_IEN	0x04
#define WK2XXX_RXOUT_IEN	0x02
#define WK2XXX_RFTRIG_IEN	0x01

#define WK2XXX_FERR_INT		0x80
#define WK2XXX_CTS_INT		0x40
#define WK2XXX_RTS_INT		0x20
#define WK2XXX_XOFF_INT		0x10
#define WK2XXX_TFEMPTY_INT	0x08
#define WK2XXX_TFTRIG_INT	0x04
#define WK2XXX_RXOVT_INT	0x02
#define WK2XXX_RFTRIG_INT	0x01

#define WK2XXX_RFOE	0x80
#define WK2XXX_RFBI	0x40
#define WK2XXX_RFFE	0x20
#define WK2XXX_RFPE	0x10
#define WK2XXX_RDAT	0x08
#define WK2XXX_TDAT	0x04
#define WK2XXX_TFULL	0x02
#define WK2XXX_TBUSY	0x01

#define WK2XXX_OE	0x08
#define WK2XXX_BI	0x04
#define WK2XXX_FE	0x02
#define WK2XXX_PE	0x01

#define NR_PORTS	4

#define SERIAL_WK2XXX_MAJOR		207
#define CALLOUT_WK2XXX_MAJOR		208
#define MINOR_START			5

#define WK_CRASTAL_CLK			(3686400 * 2)

#define MAX_WK2XXX			4

#define WK2XXX_ISR_PASS_LIMIT		50
#define PORT_WK2XXX			1
#endif

