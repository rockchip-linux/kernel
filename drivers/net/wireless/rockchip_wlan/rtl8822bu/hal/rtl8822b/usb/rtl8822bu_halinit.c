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
#define _RTL8822BU_HALINIT_C_

#include <hal_data.h>			/* HAL DATA */
#include "../../hal_halmac.h"		/* HALMAC API */
#include "../rtl8822b.h"		/* rtl8822b hal common define */
#include "rtl8822bu.h"

#ifndef CONFIG_USB_HCI

	#error "CONFIG_USB_HCI shall be on!\n"

#endif


static void _dbg_dump_macreg(PADAPTER padapter)
{
	u32 offset = 0;
	u32 val32 = 0;
	u32 index = 0;

	for (index = 0; index < 64; index++) {
		offset = index * 4;
		val32 = rtw_read32(padapter, offset);
		RTW_INFO("offset : 0x%02x ,val:0x%08x\n", offset, val32);
	}
}

u32 rtl8822bu_init(PADAPTER padapter)
{
	u8 status = _SUCCESS;
	u32 init_start_time = rtw_get_current_time();

	rtl8822b_init(padapter);

exit:
	RTW_INFO("%s in %dms\n", __func__, rtw_get_passing_time_ms(init_start_time));

	return status;
}

static void hal_deinit_misc(PADAPTER padapter)
{

}

u32 rtl8822bu_deinit(PADAPTER padapter)
{
	struct pwrctrl_priv *pwrctl = adapter_to_pwrctl(padapter);
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct dvobj_priv *pobj_priv = adapter_to_dvobj(padapter);
	u8 status = _TRUE;

	RTW_INFO("==> %s\n", __func__);


	hal_deinit_misc(padapter);
	status = rtl8822b_deinit(padapter);
	if (status == _FALSE) {
		RTW_INFO("%s: rtl8822b_hal_deinit fail\n", __func__);
		return _FAIL;
	}

	RTW_INFO("%s <==\n", __func__);
	return _SUCCESS;
}


u32 rtl8822bu_inirp_init(PADAPTER padapter)
{
	u8 i, status;
	struct recv_buf *precvbuf;
	struct dvobj_priv *pdev = adapter_to_dvobj(padapter);
	struct intf_hdl *pintfhdl = &padapter->iopriv.intf;
	struct recv_priv *precvpriv = &(padapter->recvpriv);
	u32(*_read_port)(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *pmem);
#ifdef CONFIG_USB_INTERRUPT_IN_PIPE
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	u32(*_read_interrupt)(struct intf_hdl *pintfhdl, u32 addr);
#endif


	_read_port = pintfhdl->io_ops._read_port;

	status = _SUCCESS;


	precvpriv->ff_hwaddr = RECV_BULK_IN_ADDR;

	/* issue Rx irp to receive data */
	precvbuf = (struct recv_buf *)precvpriv->precv_buf;
	for (i = 0; i < NR_RECVBUFF; i++) {
		if (_read_port(pintfhdl, precvpriv->ff_hwaddr, 0, (u8 *)precvbuf) == _FALSE) {
			status = _FAIL;
			goto exit;
		}

		precvbuf++;
		precvpriv->free_recv_buf_queue_cnt--;
	}

#ifdef CONFIG_USB_INTERRUPT_IN_PIPE
	if (pdev->RtInPipe[REALTEK_USB_IN_INT_EP_IDX] != 0x05) {
		status = _FAIL;
		RTW_INFO("%s =>Warning !! Have not USB Int-IN pipe, RtIntInPipe(%d)!!!\n", __func__, pdev->RtInPipe[REALTEK_USB_IN_INT_EP_IDX]);
		goto exit;
	}
	_read_interrupt = pintfhdl->io_ops._read_interrupt;
	if (_read_interrupt(pintfhdl, RECV_INT_IN_ADDR) == _FALSE) {
		status = _FAIL;
	}
#endif

exit:



	return status;

}

u32 rtl8822bu_inirp_deinit(PADAPTER padapter)
{

	rtw_read_port_cancel(padapter);


	return _SUCCESS;
}

void rtl8822bu_update_interrupt_mask(PADAPTER padapter, u8 bHIMR0 , u32 AddMSR, u32 RemoveMSR)
{
	HAL_DATA_TYPE *pHalData;
	u32 *himr;

	pHalData = GET_HAL_DATA(padapter);

	if (bHIMR0)
		himr = &(pHalData->IntrMask[0]);
	else
		himr = &(pHalData->IntrMask[1]);

	if (AddMSR)
		*himr |= AddMSR;

	if (RemoveMSR)
		*himr &= (~RemoveMSR);

	if (bHIMR0)
		rtw_write32(padapter, REG_HIMR0_8822B, *himr);
	else
		rtw_write32(padapter, REG_HIMR1_8822B, *himr);

}

static void config_chip_out_EP(PADAPTER padapter, u8 NumOutPipe)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);


	pHalData->OutEpQueueSel = 0;
	pHalData->OutEpNumber = 0;

	switch (NumOutPipe) {
	case 4:
		pHalData->OutEpQueueSel = TX_SELE_HQ | TX_SELE_LQ | TX_SELE_NQ;
		pHalData->OutEpNumber = 4;
		break;
	case 3:
		pHalData->OutEpQueueSel = TX_SELE_HQ | TX_SELE_LQ | TX_SELE_NQ;
		pHalData->OutEpNumber = 3;
		break;
	case 2:
		pHalData->OutEpQueueSel = TX_SELE_HQ | TX_SELE_NQ;
		pHalData->OutEpNumber = 2;
		break;
	case  1:
		pHalData->OutEpQueueSel = TX_SELE_HQ;
		pHalData->OutEpNumber = 1;
		break;
	default:
		break;
	}

	RTW_INFO("%s OutEpQueueSel(0x%02x), OutEpNumber(%d)\n", __func__, pHalData->OutEpQueueSel, pHalData->OutEpNumber);

}

static u8 usb_set_queue_pipe_mapping(PADAPTER padapter, u8 NumInPipe, u8 NumOutPipe)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	u8 result = _FALSE;

	config_chip_out_EP(padapter, NumOutPipe);

	/* Normal chip with one IN and one OUT doesn't have interrupt IN EP. */
	if (1 == pHalData->OutEpNumber) {
		if (1 != NumInPipe)
			return result;
	}

	result = Hal_MappingOutPipe(padapter, NumOutPipe);

	return result;

}

void rtl8822bu_interface_configure(PADAPTER padapter)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	struct dvobj_priv *pdvobjpriv = adapter_to_dvobj(padapter);

	if (IS_SUPER_SPEED_USB(padapter))
		pHalData->UsbBulkOutSize = USB_SUPER_SPEED_BULK_SIZE;
	else if (IS_HIGH_SPEED_USB(padapter))
		pHalData->UsbBulkOutSize = USB_HIGH_SPEED_BULK_SIZE;
	else
		pHalData->UsbBulkOutSize = USB_FULL_SPEED_BULK_SIZE;

	pHalData->interfaceIndex = pdvobjpriv->InterfaceNumber;

#ifdef CONFIG_USB_TX_AGGREGATION
	/* according to value defined by halmac */
	pHalData->UsbTxAggMode		= 1;
	pHalData->UsbTxAggDescNum	= HALMAC_BLK_DESC_NUM_8822B;
#endif /* CONFIG_USB_TX_AGGREGATION */

#ifdef CONFIG_USB_RX_AGGREGATION
	/* according to value defined by halmac */
	pHalData->rxagg_mode = RX_AGG_USB;
#if 0
	pHalData->rxagg_usb_size = 8;
	pHalData->rxagg_usb_timeout = 0x6;
	pHalData->rxagg_dma_size = 16;
	pHalData->rxagg_dma_timeout = 0x6;
#endif
#endif /* CONFIG_USB_RX_AGGREGATION */

	usb_set_queue_pipe_mapping(padapter,
			   pdvobjpriv->RtNumInPipes, pdvobjpriv->RtNumOutPipes);
}
