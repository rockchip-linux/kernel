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
#define _RTL8822BS_HALMAC_C_

#include <drv_types.h>		/* struct dvobj_priv and etc. */
#include <rtw_sdio.h>		/* rtw_sdio_write_cmd53() */
#include "../../hal_halmac.h"	/* PHALMAC_ADAPTER, PHALMAC_API and etc. */
#include "rtl8822bs.h"		/* rtl8822bs_write_port() */


static u8 sdio_write_data_rsvd_page(void *d, u8 *pBuf, u32 size)
{
	PHALMAC_ADAPTER halmac;
	PHALMAC_API api;
	u32 desclen, len;
	u8 *buf;
	u8 ret;


	halmac = dvobj_to_halmac((struct dvobj_priv *)d);
	api = HALMAC_GET_API(halmac);

	desclen = HALMAC_TX_DESC_SIZE_8822B;
	len = desclen + size;
	buf = rtw_zmalloc(len);
	if (!buf)
		return _FALSE;
	_rtw_memcpy(buf + desclen, pBuf, size);

	SET_TX_DESC_TXPKTSIZE_8822B(buf, size);
	SET_TX_DESC_OFFSET_8822B(buf, desclen);
	SET_TX_DESC_QSEL_8822B(buf, HALMAC_QUEUE_SELECT_BCN);
	api->halmac_fill_txdesc_checksum(halmac, buf);

	ret = rtl8822bs_write_port(d, len, buf);
	if (_SUCCESS == ret)
		ret = _TRUE;
	else
		ret = _FALSE;

	rtw_mfree(buf, len);

	return ret;
}

static u8 sdio_write_data_h2c(void *d, u8 *pBuf, u32 size)
{
	PHALMAC_ADAPTER halmac;
	PHALMAC_API api;
	u32 addr, desclen, len;
	u8 *buf;
	u8 ret;


	halmac = dvobj_to_halmac((struct dvobj_priv *)d);
	api = HALMAC_GET_API(halmac);

	desclen = HALMAC_TX_DESC_SIZE_8822B;
	len = desclen + size;
	buf = rtw_zmalloc(len);
	if (!buf)
		return _FALSE;
	_rtw_memcpy(buf + desclen, pBuf, size);

	SET_TX_DESC_TXPKTSIZE_8822B(buf, size);
	SET_TX_DESC_QSEL_8822B(buf, HALMAC_QUEUE_SELECT_CMD);
	api->halmac_fill_txdesc_checksum(halmac, buf);

	ret = rtl8822bs_write_port(d, len, buf);
	if (_SUCCESS == ret)
		ret = _TRUE;
	else
		ret = _FALSE;

	rtw_mfree(buf, len);

	return ret;
}

int rtl8822bs_halmac_init_adapter(PADAPTER adapter)
{
	struct dvobj_priv *d;
	PHALMAC_PLATFORM_API api;
	int err;


	d = adapter_to_dvobj(adapter);
	api = &rtw_halmac_platform_api;
	api->SEND_RSVD_PAGE = sdio_write_data_rsvd_page;
	api->SEND_H2C_PKT = sdio_write_data_h2c;

	err = rtw_halmac_init_adapter(d, api);

	return err;
}
