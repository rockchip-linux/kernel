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
#define _RTL8822BU_HALMAC_C_

#include <drv_types.h>		/* struct dvobj_priv and etc. */
#include "../../hal_halmac.h"

static u8 usb_write_data_rsvd_page(void *d, u8 *pBuf, u32 size)
{
	struct dvobj_priv *pobj = (struct dvobj_priv *)d;
	PADAPTER padapter = dvobj_get_primary_adapter(pobj);
	PHALMAC_ADAPTER halmac = dvobj_to_halmac((struct dvobj_priv *)d);
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	struct xmit_frame	*pcmdframe = NULL;
	struct pkt_attrib	*pattrib = NULL;
	PHALMAC_API api = HALMAC_GET_API(halmac);
	u8 txdesoffset = 0;
	u8 *buf = NULL;

	if (size + TXDESC_OFFSET > MAX_CMDBUF_SZ) {
		RTW_INFO("%s: total buffer size(%d) > MAX_CMDSZE(%d)\n"
			 , __func__, size + TXDESC_OFFSET, MAX_CMDSZ);
		return _FALSE;
	}

	pcmdframe = rtw_alloc_cmdxmitframe(pxmitpriv);

	if (pcmdframe == NULL) {
		RTW_INFO("%s: alloc cmd frame fail!\n", __func__);
		return _FALSE;
	}

	txdesoffset = TXDESC_OFFSET;
	buf = pcmdframe->buf_addr;
	_rtw_memcpy((buf + txdesoffset), pBuf, size); /* shift desclen */

	/* update attribute */
	pattrib = &pcmdframe->attrib;
	update_mgntframe_attrib(padapter, pattrib);
	pattrib->qsel = QSLT_BEACON;
	pattrib->pktlen = size;
	pattrib->last_txcmdsz = size;

	/* fill tx desc in dump_mgntframe */
	dump_mgntframe(padapter, pcmdframe);

	return _TRUE;
}

static u8 usb_write_data_h2c(void *d, u8 *pBuf, u32 size)
{
	struct dvobj_priv *pobj = (struct dvobj_priv *)d;
	PADAPTER padapter = dvobj_get_primary_adapter(pobj);
	PHALMAC_ADAPTER halmac = dvobj_to_halmac((struct dvobj_priv *)d);
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	struct xmit_frame	*pcmdframe = NULL;
	struct pkt_attrib	*pattrib = NULL;
	PHALMAC_API api = HALMAC_GET_API(halmac);
	u8 txdesoffset = 0;
	u8 *buf = NULL;

	if (size + TXDESC_OFFSET > MAX_XMIT_EXTBUF_SZ) {
		RTW_INFO("%s: total buffer size(%d) > MAX_XMIT_EXTBUF_SZ(%d)\n"
			 , __func__, size + TXDESC_OFFSET, MAX_XMIT_EXTBUF_SZ);
		return _FALSE;
	}

	pcmdframe = alloc_mgtxmitframe(pxmitpriv);

	if (pcmdframe == NULL) {
		RTW_INFO("%s: alloc cmd frame fail!\n", __func__);
		return _FALSE;
	}

	txdesoffset = TXDESC_SIZE;
	buf = pcmdframe->buf_addr;
	_rtw_memcpy(buf + txdesoffset, pBuf, size); /* shift desclen */

	/* update attribute */
	pattrib = &pcmdframe->attrib;
	update_mgntframe_attrib(padapter, pattrib);
	pattrib->qsel = HALMAC_TXDESC_QSEL_H2C_CMD;
	pattrib->pktlen = size;
	pattrib->last_txcmdsz = size;

	/* fill tx desc in dump_mgntframe */
	dump_mgntframe(padapter, pcmdframe);

	return _TRUE;

}

int rtl8822bu_halmac_init_adapter(PADAPTER padapter)
{
	struct dvobj_priv *d;
	PHALMAC_PLATFORM_API api;
	int err;


	d = adapter_to_dvobj(padapter);
	api = &rtw_halmac_platform_api;
	api->SEND_RSVD_PAGE = usb_write_data_rsvd_page;
	api->SEND_H2C_PKT = usb_write_data_h2c;

	err = rtw_halmac_init_adapter(d, api);

	return err;
}
