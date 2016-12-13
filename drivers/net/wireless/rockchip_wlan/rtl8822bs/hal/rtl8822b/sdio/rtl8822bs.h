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
#ifndef _RTL8822BS_H_
#define _RTL8822BS_H_

#include <drv_types.h>		/* PADAPTER, struct dvobj_priv and etc. */


/* rtl8822bs_halinit.c */
u32 rtl8822bs_init(PADAPTER);
void rtl8822bs_init_default_value(PADAPTER);

/* rtl8822bs_halmac.c */
int rtl8822bs_halmac_init_adapter(PADAPTER);

/* rtl8822bs_io.c */
u32 rtl8822bs_read_port(struct dvobj_priv *, u32 cnt, u8 *mem);
u32 rtl8822bs_write_port(struct dvobj_priv *, u32 cnt, u8 *mem);

/* rtl8822bs_led.c */
void rtl8822bs_initswleds(PADAPTER);
void rtl8822bs_deinitswleds(PADAPTER);

/* rtl8822bs_xmit.c */
s32 rtl8822bs_init_xmit_priv(PADAPTER);
void rtl8822bs_free_xmit_priv(PADAPTER);
s32 rtl8822bs_hal_xmit_enqueue(PADAPTER, struct xmit_frame *);
s32 rtl8822bs_hal_xmit(PADAPTER, struct xmit_frame *);
s32 rtl8822bs_mgnt_xmit(PADAPTER, struct xmit_frame *);
s32 rtl8822bs_xmit_buf_handler(PADAPTER);
thread_return rtl8822bs_xmit_thread(thread_context);

/* rtl8822bs_recv.c */
s32 rtl8822bs_init_recv_priv(PADAPTER);
void rtl8822bs_free_recv_priv(PADAPTER);
_pkt *rtl8822bs_alloc_recvbuf_skb(struct recv_buf *, u32 size);
void rtl8822bs_free_recvbuf_skb(struct recv_buf *);
s32 rtl8822bs_recv_hdl(_adapter *adapter);
void rtl8822bs_rxhandler(PADAPTER, struct recv_buf *);
#ifdef CONFIG_SDIO_RX_READ_IN_THREAD
void rtl8822bs_rx_polling_init(struct dvobj_priv *);
void rtl8822bs_rx_polling_deinit(struct dvobj_priv *);
void rtl8822bs_rx_polling_thread_start(struct dvobj_priv *);
void rtl8822bs_rx_polling_thread_stop(struct dvobj_priv *);
void rtl8822bs_rx_polling_start(struct dvobj_priv *);
#endif /* CONFIG_SDIO_RX_READ_IN_THREAD */

/* rtl8822bs_ops.c */
void rtl8822bs_get_interrupt(PADAPTER, u32 *hisr, u16 *rx_len);
void rtl8822bs_clear_interrupt(PADAPTER, u32 hisr);
void rtl8822bs_init_interrupt(PADAPTER);
#ifdef CONFIG_SDIO_RX_READ_IN_THREAD
void rtl8822bs_enable_rx_interrupt(struct dvobj_priv *);
void rtl8822bs_disable_rx_interrupt(struct dvobj_priv *);
#endif /* CONFIG_SDIO_RX_READ_IN_THREAD */

#endif /* _RTL8822BS_H_ */
