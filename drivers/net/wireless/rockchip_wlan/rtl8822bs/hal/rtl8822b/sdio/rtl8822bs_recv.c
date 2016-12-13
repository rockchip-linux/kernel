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
#define _RTL8822BS_RECV_C_

#include <drv_types.h>		/* PADAPTER and etc. */
#include <hal_data.h>		/* HAL_DATA_TYPE */
#include "../../hal_halmac.h"	/* BIT_ACRC32_8822B, HALMAC_RX_DESC_SIZE_8822B and etc. */
#include "../rtl8822b.h"	/* rtl8822b_rxdesc2attribute(), rtl8822b_c2h_handler_no_io() */
#ifdef CONFIG_SDIO_RX_READ_IN_THREAD
#include "rtl8822bs.h"		/* rtl8822bs_read_port(), rtl8822bs_get_interrupt() and etc. */
#endif /* CONFIG_SDIO_RX_READ_IN_THREAD */


static s32 initrecvbuf(struct recv_buf *precvbuf, PADAPTER adapter)
{
	_rtw_init_listhead(&precvbuf->list);
	_rtw_spinlock_init(&precvbuf->recvbuf_lock);

	precvbuf->adapter = adapter;

	return _SUCCESS;
}

static void freerecvbuf(struct recv_buf *precvbuf)
{
	_rtw_spinlock_free(&precvbuf->recvbuf_lock);
}

static void start_rx_handle(PADAPTER p)
{
#ifdef CONFIG_RECV_THREAD_MODE
	_rtw_up_sema(&p->recvpriv.recv_sema);
#else
	#ifdef PLATFORM_LINUX
	tasklet_schedule(&p->recvpriv.recv_tasklet);
	#endif
#endif
}

static void stop_rx_handle(PADAPTER p)
{
#ifdef CONFIG_RECV_THREAD_MODE
#else
	#ifdef PLATFORM_LINUX
	tasklet_kill(&p->recvpriv.recv_tasklet);
	#endif
#endif
}

/*
 * Return:
 *	Pointer of _pkt, otherwise NULL.
 */
static _pkt *alloc_recvbuf_skb(struct recv_buf *recvbuf, u32 size)
{
	_pkt *skb;
	u32 alignsz = RECVBUFF_ALIGN_SZ;
#ifdef PLATFORM_LINUX
	SIZE_PTR tmpaddr = 0;
	SIZE_PTR alignment = 0;
#endif /* PLATFORM_LINUX */


	size += alignsz;
	skb = rtw_skb_alloc(size);
	if (!skb) {
		RTW_WARN("%s: alloc_skb fail! size=%d\n", __FUNCTION__, size);
		return NULL;
	}
	recvbuf->pskb = skb;

#ifdef PLATFORM_LINUX
	skb->dev = recvbuf->adapter->pnetdev;

	tmpaddr = (SIZE_PTR)skb->data;
	alignment = tmpaddr & (alignsz - 1);
	skb_reserve(skb, alignsz - alignment);

	recvbuf->pbuf = skb->data;
	recvbuf->len = 0;
	recvbuf->phead = skb->head;
	recvbuf->pdata = skb->data;
	recvbuf->ptail = skb->data; /*skb_tail_pointer(skb);*/
	recvbuf->pend = skb_end_pointer(skb);
#else /* !PLATFORM_LINUX */
#error "Please handle the pointer in recvbuf!!"
#endif /* !PLATFORM_LINUX */

	return skb;
}

/*
 * Description:
 *	Allocate skb for recv_buf, the size is MAX_RECVBUF_SZ
 *
 * Parameters:
 *	recvbuf	pointer of struct recv_buf
 *	size	skb size, only valid when NOT define CONFIG_SDIO_RX_COPY.
 *		If CONFIG_SDIO_RX_COPY, size always be MAX_RECVBUF_SZ.
 *
 * Return:
 *	Pointer of _pkt, otherwise NULL.
 */
_pkt *rtl8822bs_alloc_recvbuf_skb(struct recv_buf *recvbuf, u32 size)
{
	_pkt *skb;


	skb = recvbuf->pskb;
#ifdef CONFIG_SDIO_RX_COPY
	if (skb) {
		/* reset skb pointer and length */
		skb_reset_tail_pointer(skb);
		skb->len = 0;
		/* reset recv_buf pointer and length */
		recvbuf->pdata = recvbuf->pbuf;
		recvbuf->ptail = recvbuf->pbuf;
		recvbuf->len = 0;
		return skb;
	}

	RTW_WARN("%s: skb not exist in recv_buf!\n", __FUNCTION__);
	size = MAX_RECVBUF_SZ;
#else /* !CONFIG_SDIO_RX_COPY */
	if (skb) {
		RTW_WARN("%s: skb already exist in recv_buf!\n", __FUNCTION__);
		rtl8822bs_free_recvbuf_skb(recvbuf);
	}
#endif /* !CONFIG_SDIO_RX_COPY */

	skb = alloc_recvbuf_skb(recvbuf, size);
	if (!skb)
		return NULL;

	return skb;
}

static void free_recvbuf_skb(struct recv_buf *recvbuf)
{
	_pkt *skb;


	skb = recvbuf->pskb;
	if (!skb)
		return;
	recvbuf->pskb = NULL;
	rtw_skb_free(skb);
}

void rtl8822bs_free_recvbuf_skb(struct recv_buf *recvbuf)
{
#ifndef CONFIG_SDIO_RX_COPY
	free_recvbuf_skb(recvbuf);
#endif /* !CONFIG_SDIO_RX_COPY */
}

/*
 * Return:
 *	_SUCCESS	Allocate resource OK.
 *	_FAIL		Fail to allocate resource.
 */
static inline s32 os_recvbuf_resource_alloc(PADAPTER adapter, struct recv_buf *recvbuf)
{
	s32 ret = _SUCCESS;

#ifdef CONFIG_SDIO_RX_COPY
	alloc_recvbuf_skb(recvbuf, MAX_RECVBUF_SZ);
#endif /* CONFIG_SDIO_RX_COPY */

	return ret;
}

static inline void os_recvbuf_resource_free(PADAPTER adapter, struct recv_buf *recvbuf)
{
#ifdef CONFIG_SDIO_RX_COPY
	free_recvbuf_skb(recvbuf);
#endif /* CONFIG_SDIO_RX_COPY */
}
#if 0
static union recv_frame *copy_recvframe(union recv_frame *recvframe, PADAPTER adapter)
{
	PHAL_DATA_TYPE hal;
	struct recv_priv *precvpriv;
	_queue *pfree_recv_queue;
	struct rx_pkt_attrib *attrib = NULL;
	union recv_frame *copyframe = NULL;
	_pkt *copypkt = NULL;


	hal = GET_HAL_DATA(adapter);
	precvpriv = &adapter->recvpriv;
	pfree_recv_queue = &precvpriv->free_recv_queue;
	attrib = &recvframe->u.hdr.attrib;

	copyframe = rtw_alloc_recvframe(pfree_recv_queue);
	if (!copyframe) {
		RTW_INFO(FUNC_ADPT_FMT ": Alloc recvframe FAIL!\n",
			 FUNC_ADPT_ARG(adapter));
		return NULL;
	}
	copyframe->u.hdr.adapter = adapter;
	_rtw_memcpy(&copyframe->u.hdr.attrib, attrib, sizeof(struct rx_pkt_attrib));
#if 0
	/*
	 * driver need to set skb len for skb_copy().
	 * If skb->len is zero, skb_copy() will not copy data from original skb.
	 */
	skb_put(recvframe->u.hdr.pkt, attrib->pkt_len);
#else
	RTW_INFO(FUNC_ADPT_FMT ": skb len=%d!\n",
		 FUNC_ADPT_ARG(adapter), recvframe->u.hdr.pkt->len);
#endif

	copypkt = rtw_skb_copy(recvframe->u.hdr.pkt);
	if (!copypkt) {
		if ((attrib->mfrag == 1) && (attrib->frag_num == 0)) {
			RTW_INFO(FUNC_ADPT_FMT ": <ERR> rtw_skb_copy fail for first fragment!\n",
				 FUNC_ADPT_ARG(adapter));
			rtw_free_recvframe(recvframe, &precvpriv->free_recv_queue);
			return NULL;
		}

		copypkt = rtw_skb_clone(recvframe->u.hdr.pkt);
		if (!copypkt) {
			RTW_INFO(FUNC_ADPT_FMT ": <ERR> rtw_skb_clone fail, drop frame!\n",
				 FUNC_ADPT_ARG(adapter));
			rtw_free_recvframe(recvframe, &precvpriv->free_recv_queue);
			return NULL;
		}
	}
	copypkt->dev = adapter->pnetdev;

	copyframe->u.hdr.pkt = copypkt;
	copyframe->u.hdr.len = copypkt->len;
	copyframe->u.hdr.rx_head = copypkt->head;
	copyframe->u.hdr.rx_data = copypkt->data;
	copyframe->u.hdr.rx_tail = skb_tail_pointer(copypkt);
	copyframe->u.hdr.rx_end = skb_end_pointer(copypkt);

	return copyframe;
}
#endif
/*
 * Return:
 *	_SUCCESS	OK to send packet
 *	_FAIL		FAIL to send packet
 */
static s32 recv_entry(union recv_frame *recvframe, u8 *phy_status)
{
	s32 ret = _SUCCESS;
	PADAPTER adapter;
	struct rx_pkt_attrib *attrib = NULL;
#ifdef CONFIG_CONCURRENT_MODE
	struct dvobj_priv *d;
	u8 *addr1, *macaddr;
	u8 mcast, i;
	union recv_frame *copyframe = NULL;
#endif /* CONFIG_CONCURRENT_MODE */


	attrib = &recvframe->u.hdr.attrib;

#ifdef CONFIG_CONCURRENT_MODE
#if 0
	d = adapter_to_dvobj(recvframe->u.hdr.adapter);
	addr1 = GetAddr1Ptr(recvframe->u.hdr.rx_data);
	mcast = IS_MCAST(addr1);
	if (_TRUE == mcast) {
		/* BC/MC packets */
		for (i = 1; i < d->iface_nums; i++) {
			adapter = d->adapters[i];

			if (rtw_if_up(adapter) == _FALSE)
				continue;

			copyframe = copy_recvframe(recvframe, adapter);
			if (!copyframe)
				break;

			if (attrib->physt)
				rx_query_phy_status(copyframe, phy_status);

			ret = rtw_recv_entry(copyframe);
		}
	} else {
		/* unicast packets */
		for (i = 0; i < d->iface_nums; i++) {
			adapter = d->adapters[i];

			if (rtw_if_up(adapter) == _FALSE)
				continue;

			macaddr = adapter_mac_addr(adapter);
			if (_rtw_memcmp(addr1, macaddr, ETH_ALEN) == _FALSE)
				continue;

			/* change to target interface */
			recvframe->u.hdr.adapter = adapter;
			recvframe->u.hdr.pkt->dev = adapter->pnetdev;
			break;
		}
	}
#else
	pre_recv_entry(recvframe, phy_status);
#endif
#endif /* CONFIG_CONCURRENT_MODE */

	if (attrib->physt)
		rx_query_phy_status(recvframe, phy_status);

	ret = rtw_recv_entry(recvframe);

	return ret;
}

/*
 * Return:
 *	_TRUE	Finish preparing recv_frame
 *	_FALSE	Something fail to prepare recv_frame
 */
static _pkt *prepare_recvframe_pkt(struct recv_buf *recvbuf, union recv_frame *recvframe)
{
	_pkt *pkt = NULL;
	struct rx_pkt_attrib *attrib;
	u32 skb_len;
	u8 *data;
#ifdef CONFIG_SDIO_RX_COPY
	u32 shift_sz, alloc_sz;
#endif /* CONFIG_SDIO_RX_COPY */


	pkt = recvframe->u.hdr.pkt;
	if (pkt) {
		RTW_WARN("%s: recvframe pkt already exist!\n", __FUNCTION__);
		return pkt;
	}

	attrib = &recvframe->u.hdr.attrib;
	skb_len = attrib->pkt_len;
	if (rtl8822b_rx_fcs_appended(recvbuf->adapter) == _TRUE)
		skb_len -= IEEE80211_FCS_LEN;
	data = recvbuf->pdata + HALMAC_RX_DESC_SIZE_8822B + attrib->drvinfo_sz;
#if 0
	data += attrib->shift_sz;
#endif

#ifdef CONFIG_SDIO_RX_COPY
	/* For 8 bytes IP header alignment. */
	if (attrib->qos)
		/* Qos data, wireless lan header length is 26 */
		shift_sz = 6;
	else
		shift_sz = 0;

	/*
	 * For first fragment packet, driver need allocate
	 * (1536 + drvinfo_sz + RXDESC_SIZE) to defrag packet.
	 * In 8822B, drvinfo_sz = 32, RXDESC_SIZE = 24, 1536 + 32 + 24 = 1592.
	 * And need 8 is for skb->data 8 bytes alignment.
	 * Round (1536 + 24 + 32 + shift_sz + 8) to 128 bytes alignment,
	 * and finally get 1664.
	 */
	if ((attrib->mfrag == 1) && (attrib->frag_num == 0)) {
		if (skb_len <= 1650)
			alloc_sz = 1664;
		else
			alloc_sz = skb_len + 14;
	} else {
		alloc_sz = skb_len;
		/*
		 * 6 is for IP header 8 bytes alignment in QoS packet case.
		 * 8 is for skb->data 4 bytes alignment.
		 */
		alloc_sz += 14;
	}

	pkt = rtw_skb_alloc(alloc_sz);
	if (pkt) {
		pkt->dev = recvframe->u.hdr.adapter->pnetdev;
		/* force pkt->data at 8-byte alignment address */
		skb_reserve(pkt, 8 - ((SIZE_PTR)pkt->data & 7));
		/* force ip_hdr at 8-byte alignment address according to shift_sz. */
		skb_reserve(pkt, shift_sz);
		_rtw_memcpy(skb_put(pkt, skb_len), data, skb_len);
	} else if ((attrib->mfrag == 1) && (attrib->frag_num == 0)) {
		RTW_ERR("%s: alloc_skb fail for first fragement\n", __FUNCTION__);
		return NULL;
	}
#endif /* CONFIG_SDIO_RX_COPY */

	if (!pkt) {
		pkt = rtw_skb_clone(recvbuf->pskb);
		if (!pkt) {
			RTW_ERR("%s: rtw_skb_clone fail\n", __FUNCTION__);
			return NULL;
		}
		pkt->data = data;
		skb_set_tail_pointer(pkt, skb_len);
		pkt->len = skb_len;
	}

	recvframe->u.hdr.pkt = pkt;
	recvframe->u.hdr.len = pkt->len;
	recvframe->u.hdr.rx_head = pkt->head;
	recvframe->u.hdr.rx_data = pkt->data;
	recvframe->u.hdr.rx_tail = skb_tail_pointer(pkt);
	recvframe->u.hdr.rx_end = skb_end_pointer(pkt);

	return pkt;
}

/*
 * Return:
 *	_SUCCESS	Finish processing recv_buf
 *	others		Something fail to process recv_buf
 */
static u8 recvbuf_handler(struct recv_buf *recvbuf)
{
	PADAPTER p;
	struct recv_priv *recvpriv;
	union recv_frame *recvframe;
	struct rx_pkt_attrib *attrib;
	_pkt *pkt;
	u32 rx_report_sz, pkt_offset;
	u8 *ptr;
	u8 ret = _SUCCESS;


	p = recvbuf->adapter;
	recvpriv = &p->recvpriv;
	ptr = recvbuf->pdata;

	while (ptr < recvbuf->ptail) {
		recvframe = rtw_alloc_recvframe(&recvpriv->free_recv_queue);
		if (!recvframe) {
			RTW_WARN("%s: no enough recv frame!\n", __FUNCTION__);
			ret = RTW_RFRAME_UNAVAIL;
			break;
		}

		/* rx desc parsing */
		attrib = &recvframe->u.hdr.attrib;
		rtl8822b_rxdesc2attribute(attrib, ptr);

		rx_report_sz = HALMAC_RX_DESC_SIZE_8822B + attrib->drvinfo_sz;
		pkt_offset = rx_report_sz + attrib->shift_sz + attrib->pkt_len;

		if ((ptr + pkt_offset) > recvbuf->ptail) {
			RTW_WARN("%s: next pkt len(%p,%d) exceed ptail(%p)!\n",
				 __FUNCTION__, ptr, pkt_offset, recvbuf->ptail);
			rtw_free_recvframe(recvframe, &recvpriv->free_recv_queue);
			break;
		}

		/* fix Hardware RX data error, drop whole recv_buffer */
		if ((rtl8822b_rcr_check(p, BIT_ACRC32_8822B) == _FALSE)
		    && attrib->crc_err) {
			RTW_WARN("%s: Received unexpected CRC error packet!!\n", __FUNCTION__);
			rtw_free_recvframe(recvframe, &recvpriv->free_recv_queue);
			break;
		}

		if ((attrib->crc_err) || (attrib->icv_err)) {
#ifdef CONFIG_MP_INCLUDED
			if (p->registrypriv.mp_mode == 1) {
				if (check_fwstate(&p->mlmepriv, WIFI_MP_STATE) == _TRUE) {
					if (attrib->crc_err == 1)
						p->mppriv.rx_crcerrpktcount++;
				}
			} else
#endif /* CONFIG_MP_INCLUDED */
			{
				RTW_INFO("%s: crc_err=%d icv_err=%d, skip!\n",
					__FUNCTION__, attrib->crc_err, attrib->icv_err);
			}
			rtw_free_recvframe(recvframe, &recvpriv->free_recv_queue);
		} else {
			pkt = prepare_recvframe_pkt(recvbuf, recvframe);
			if (!pkt) {
				rtw_free_recvframe(recvframe, &recvpriv->free_recv_queue);
				ret = RTW_RFRAME_PKT_UNAVAIL;
				break;
			}

			/* move to start of PHY_STATUS */
			ptr += HALMAC_RX_DESC_SIZE_8822B;
			if (rtl8822b_rx_ba_ssn_appended(p) == _TRUE)
				ptr += RTW_HALMAC_BA_SSN_RPT_SIZE;

			recv_entry(recvframe, ptr);
		}

		pkt_offset = _RND8(pkt_offset);
		recvbuf->pdata += pkt_offset;
		ptr = recvbuf->pdata;
	}

	return ret;
}

s32 rtl8822bs_recv_hdl(_adapter *adapter)
{
	struct recv_priv *recvpriv;
	struct recv_buf *recvbuf;
	u8 c2h = 0;
	s32 ret = _SUCCESS;

	recvpriv = &adapter->recvpriv;

	do {
		recvbuf = rtw_dequeue_recvbuf(&recvpriv->recv_buf_pending_queue);
		if (NULL == recvbuf)
			break;

		c2h = GET_RX_DESC_C2H_8822B(recvbuf->pdata);
		if (c2h) {
			if (recvbuf->len <= 256)
				rtl8822b_c2h_handler_no_io(adapter, recvbuf->pdata, recvbuf->len);
		}
		else
			ret = recvbuf_handler(recvbuf);

		if (_SUCCESS != ret) {
			rtw_enqueue_recvbuf_to_head(recvbuf, &recvpriv->recv_buf_pending_queue);
			break;
		}

		/* free recv_buf */
		rtl8822bs_free_recvbuf_skb(recvbuf);
		rtw_enqueue_recvbuf(recvbuf, &recvpriv->free_recv_buf_queue);
#ifdef CONFIG_SDIO_RX_READ_IN_THREAD
		wake_up_process(GET_HAL_DATA(adapter)->rx_polling_thread);
#endif /* CONFIG_SDIO_RX_READ_IN_THREAD */
	} while (1);

#ifdef CONFIG_RTW_NAPI
#ifdef CONFIG_RTW_NAPI_V2
	if (adapter->registrypriv.en_napi) {
		struct dvobj_priv *d;
		struct _ADAPTER *a;
		u8 i;

		d = adapter_to_dvobj(adapter);
		for (i = 0; i < d->iface_nums; i++) {
			a = d->padapters[i];
			if (rtw_if_up(a) == _TRUE)
				napi_schedule(&a->napi);

		}
	}
#endif /* CONFIG_RTW_NAPI_V2 */
#endif /* CONFIG_RTW_NAPI */

	return ret;
}

static void recv_tasklet(void *priv)
{
	PADAPTER adapter;
	s32 ret;

	adapter = (PADAPTER)priv;

	ret = rtl8822bs_recv_hdl(adapter);
	if (ret == RTW_RFRAME_UNAVAIL
		|| ret == RTW_RFRAME_PKT_UNAVAIL)
		start_rx_handle(adapter);
}

#ifdef CONFIG_SDIO_RX_READ_IN_THREAD
void rtl8822bs_rx_polling_init(struct dvobj_priv *d)
{
	struct hal_com_data *hal;


	hal = GET_HAL_DATA(dvobj_get_primary_adapter(d));

	hal->rx_polling_thread = NULL;
	_rtw_init_sema(&hal->rx_polling_sema, 0);
	_rtw_init_sema(&hal->rx_polling_terminate_sema, 0);
}

void rtl8822bs_rx_polling_deinit(struct dvobj_priv *d)
{
	struct hal_com_data *hal;


	hal = GET_HAL_DATA(dvobj_get_primary_adapter(d));

	if (hal->rx_polling_thread)
		rtl8822bs_rx_polling_thread_stop(d);
}

static void rx_polling_handle(struct dvobj_priv *d)
{
	struct _ADAPTER *a;
	struct hal_com_data *hal;
	struct recv_priv *recvpriv;
	struct recv_buf *recvbuf;
	_pkt *pkt;
	u32 blksz;
	u16 size, bufsz;
	u8 *rbuf;
	s32 ret;
	u32 count = 0, err_count;


	a = dvobj_get_primary_adapter(d);
	recvpriv = &a->recvpriv;
	hal = GET_HAL_DATA(a);

	blksz = d->intf_data.block_transfer_len;

	size = hal->SdioRxFIFOSize;
	do {
		if (!size)
			break;
		count++;

		/*
		 * Patch for some SDIO Host 4 bytes issue
		 * ex. RK3188
		 */
		bufsz = RND4(size);

		/* round to block size */
		if (bufsz > blksz)
			bufsz = _RND(bufsz, blksz);

		/* 1. alloc recvbuf */
		err_count = 0;
		do {
			recvbuf = rtw_dequeue_recvbuf(&recvpriv->free_recv_buf_queue);
			if (recvbuf)
				break;
			err_count++;
			if ((err_count&0x7F) == 0)
				RTW_WARN("%s: alloc recvbuf FAIL! count=%u\n", __FUNCTION__, err_count);
			if (RTW_CANNOT_RUN(a) || kthread_should_stop())
				break;
#if 0
			rtw_yield_os();
#else
			set_current_state(TASK_INTERRUPTIBLE);
#ifdef CONFIG_RECV_THREAD_MODE
			wake_up_process(a->recvThread);
#endif /* CONFIG_RECV_THREAD_MODE */
			if (!kthread_should_stop())
				schedule_timeout(MAX_SCHEDULE_TIMEOUT);
			set_current_state(TASK_RUNNING);
#endif
		} while (1);
		if (!recvbuf) {
			RTW_ERR("%s: stop running and alloc recvbuf FAIL! count=%u\n", __FUNCTION__, err_count);
			break;
		}

		/* 2. alloc skb */
		err_count = 0;
		do {
			pkt = rtl8822bs_alloc_recvbuf_skb(recvbuf, bufsz);
			if (pkt)
				break;
			err_count++;
			if ((err_count&0x7F) == 0)
				RTW_WARN("%s: alloc_skb fail! alloc=%u read=%u count=%u\n", __FUNCTION__, bufsz, size, err_count);
			if (RTW_CANNOT_RUN(a) || kthread_should_stop())
				break;
#if 0
			rtw_yield_os();
#else
			set_current_state(TASK_INTERRUPTIBLE);
			if (!kthread_should_stop())
				schedule_timeout(1);
			set_current_state(TASK_RUNNING);
#endif
		} while (1);
		if (!pkt) {
			RTW_ERR("%s: stop running and alloc_skb fail! alloc=%u read=%u count=%u\n", __FUNCTION__, bufsz, size, err_count);
			rtw_enqueue_recvbuf(recvbuf, &recvpriv->free_recv_buf_queue);
			break;
		}

		/* 3. read data from rxfifo */
		rbuf = skb_put(pkt, size);
		ret = rtl8822bs_read_port(d, bufsz, rbuf);
		if (_FAIL == ret) {
			RTW_ERR("%s: read port FAIL!\n", __FUNCTION__);
			rtl8822bs_free_recvbuf_skb(recvbuf);
			rtw_enqueue_recvbuf(recvbuf, &recvpriv->free_recv_buf_queue);
			break;
		}

		/* 4. init recvbuf */
		recvbuf->len = pkt->len;
		recvbuf->phead = pkt->head;
		recvbuf->pdata = pkt->data;
		recvbuf->ptail = skb_tail_pointer(pkt);
		recvbuf->pend = skb_end_pointer(pkt);

		rtl8822bs_rxhandler(a, recvbuf);

		size = 0;
		rtl8822bs_get_interrupt(a, NULL, &size);
	} while (1);
}

static thread_return rx_polling_thread(thread_context context)
{
	struct sched_param param = { .sched_priority = 1 };
	struct dvobj_priv *d;
	struct _ADAPTER *a;
	struct hal_com_data *hal;
	u32 ret;


	sched_setscheduler(current, SCHED_FIFO, &param);
	RTW_INFO("%s: RX polling thread start running at CPU:%d PID:%d\n",
	         __FUNCTION__, raw_smp_processor_id(), current->pid);

	d = (struct dvobj_priv *)context;
	a = dvobj_get_primary_adapter(d);
	hal = GET_HAL_DATA(a);

	/*thread_enter("RTW_RECV_POLLING");*/

	do {
		ret = _rtw_down_sema(&hal->rx_polling_sema);
		if (_FAIL == ret) {
			RTW_ERR("%s: down sema fail!\n", __FUNCTION__);
			break;
		}

		if (RTW_CANNOT_RUN(a) || kthread_should_stop())
			break;

		rx_polling_handle(d);
		rtl8822bs_enable_rx_interrupt(d);
	} while (!RTW_CANNOT_RUN(a) && !kthread_should_stop());

	_rtw_up_sema(&hal->rx_polling_terminate_sema);

	RTW_INFO("%s: RX polling thread stop running at CPU:%d PID:%d\n",
	         __FUNCTION__, raw_smp_processor_id(), current->pid);

	thread_exit();
}

void rtl8822bs_rx_polling_thread_start(struct dvobj_priv *d)
{
	struct _ADAPTER *a; 
	struct hal_com_data *hal;
	u8 start_thread = _FALSE;


	a = dvobj_get_primary_adapter(d);
	hal = GET_HAL_DATA(a);

	if (hal->rx_polling_thread) {
		RTW_WARN("%s: rx polling thread is running!\n", __FUNCTION__);
		return;
	}

#ifdef PLATFORM_LINUX
	hal->rx_polling_thread = kthread_run(rx_polling_thread, d, "RTW_RECV_POLLING");
	if (!IS_ERR(hal->rx_polling_thread))
		start_thread = _TRUE;
#endif /* PLATFORM_LINUX */
	if (_TRUE != start_thread)
		RTW_ERR("%s: Start rx polling thread FAIL!\n", __FUNCTION__);
}

void rtl8822bs_rx_polling_thread_stop(struct dvobj_priv *d)
{
	struct _ADAPTER *a;
	struct hal_com_data *hal;


	a = dvobj_get_primary_adapter(d);
	hal = GET_HAL_DATA(a);

	if (hal->rx_polling_thread) {
		_rtw_up_sema(&hal->rx_polling_sema);
		kthread_stop(hal->rx_polling_thread);
		_rtw_down_sema(&hal->rx_polling_terminate_sema);
		hal->rx_polling_thread = 0;
	}
}

void rx_drop(struct dvobj_priv *d, u16 size)
{
	struct _ADAPTER *a;
	struct hal_com_data *hal;
	u32 blksz;
	u16 bufsz;
	u8 *buf;


	a = dvobj_get_primary_adapter(d);
	hal = GET_HAL_DATA(a);
	blksz = d->intf_data.block_transfer_len;

	/*
	 * Patch for some SDIO Host 4 bytes issue
	 * ex. RK3188
	 */
	bufsz = RND4(size);

	/* round to block size */
	if (bufsz > blksz)
		bufsz = _RND(bufsz, blksz);

	buf = rtw_zmalloc(bufsz);
	if (!buf)
		return;

	rtl8822bs_read_port(d, bufsz, buf);

	rtw_mfree(buf, bufsz);
}

void rtl8822bs_rx_polling_start(struct dvobj_priv *d)
{
	struct _ADAPTER *a;
	struct hal_com_data *hal;


	a = dvobj_get_primary_adapter(d);
	hal = GET_HAL_DATA(a);

	if (!hal->rx_polling_thread ||
	    rtw_is_drv_stopped(a)) {
		RTW_WARN("%s: drop rx because %s!\n",
		         __FUNCTION__, rtw_is_drv_stopped(a)?"Drv Stop":"not ready");
		rx_drop(d, hal->SdioRxFIFOSize);
		return;
	}

	rtl8822bs_disable_rx_interrupt(d);
	_rtw_up_sema(&hal->rx_polling_sema);
}
#endif /* CONFIG_SDIO_RX_READ_IN_THREAD */

/*
 * Initialize recv private variable for hardware dependent
 * 1. recv buf
 * 2. recv tasklet
 * 3. recv polling thread
 */
s32 rtl8822bs_init_recv_priv(PADAPTER adapter)
{
	s32 res;
	u32 i, n;
	struct recv_priv *precvpriv;
	struct recv_buf *precvbuf;


	res = _SUCCESS;
	precvpriv = &adapter->recvpriv;

	/* 1. init recv buffer */
	_rtw_init_queue(&precvpriv->free_recv_buf_queue);
	_rtw_init_queue(&precvpriv->recv_buf_pending_queue);

	n = NR_RECVBUFF * sizeof(struct recv_buf) + 4;
	precvpriv->pallocated_recv_buf = rtw_zmalloc(n);
	if (precvpriv->pallocated_recv_buf == NULL) {
		res = _FAIL;
		goto exit;
	}

	precvpriv->precv_buf = (u8 *)N_BYTE_ALIGMENT((SIZE_PTR)(precvpriv->pallocated_recv_buf), 4);

	/* init each recv buffer */
	precvbuf = (struct recv_buf *)precvpriv->precv_buf;
	for (i = 0; i < NR_RECVBUFF; i++) {
		res = initrecvbuf(precvbuf, adapter);
		if (res == _FAIL)
			break;

		res = rtw_os_recvbuf_resource_alloc(adapter, precvbuf);
		if (res == _FAIL) {
			freerecvbuf(precvbuf);
			break;
		}

		res = os_recvbuf_resource_alloc(adapter, precvbuf);
		if (res == _FAIL) {
			freerecvbuf(precvbuf);
			break;
		}

		rtw_list_insert_tail(&precvbuf->list, &precvpriv->free_recv_buf_queue.queue);

		precvbuf++;
	}
	precvpriv->free_recv_buf_queue_cnt = i;

	if (res == _FAIL)
		goto initbuferror;

	/* 2. init tasklet */
#ifdef PLATFORM_LINUX
	tasklet_init(&precvpriv->recv_tasklet,
		     (void(*)(unsigned long))recv_tasklet,
		     (unsigned long)adapter);
#endif

#ifdef CONFIG_SDIO_RX_READ_IN_THREAD
	/* 3. init recv polling thread */
	rtl8822bs_rx_polling_init(adapter_to_dvobj(adapter));
#endif /* CONFIG_SDIO_RX_READ_IN_THREAD */

	goto exit;

initbuferror:
	precvbuf = (struct recv_buf *)precvpriv->precv_buf;
	if (precvbuf) {
		n = precvpriv->free_recv_buf_queue_cnt;
		precvpriv->free_recv_buf_queue_cnt = 0;
		for (i = 0; i < n ; i++) {
			rtw_list_delete(&precvbuf->list);
			os_recvbuf_resource_free(adapter, precvbuf);
			rtw_os_recvbuf_resource_free(adapter, precvbuf);
			freerecvbuf(precvbuf);
			precvbuf++;
		}
		precvpriv->precv_buf = NULL;
	}

	if (precvpriv->pallocated_recv_buf) {
		n = NR_RECVBUFF * sizeof(struct recv_buf) + 4;
		rtw_mfree(precvpriv->pallocated_recv_buf, n);
		precvpriv->pallocated_recv_buf = NULL;
	}

exit:
	return res;
}

/*
 * Free recv private variable of hardware dependent
 * 1. recv buf
 * 2. recv tasklet
 */
void rtl8822bs_free_recv_priv(PADAPTER adapter)
{
	u32 i, n;
	struct recv_priv *precvpriv;
	struct recv_buf *precvbuf;


	precvpriv = &adapter->recvpriv;

	/* 1. kill tasklet */
	stop_rx_handle(adapter);

#ifdef CONFIG_SDIO_RX_READ_IN_THREAD
	/* 1.1. kill recv polling thread */
	rtl8822bs_rx_polling_deinit(adapter_to_dvobj(adapter));
#endif /* CONFIG_SDIO_RX_READ_IN_THREAD */

	/* 2. free all recv buffers */
	precvbuf = (struct recv_buf *)precvpriv->precv_buf;
	if (precvbuf) {
		n = precvpriv->free_recv_buf_queue_cnt;
		precvpriv->free_recv_buf_queue_cnt = 0;
		for (i = 0; i < n ; i++) {
			rtw_list_delete(&precvbuf->list);
			os_recvbuf_resource_free(adapter, precvbuf);
			rtw_os_recvbuf_resource_free(adapter, precvbuf);
			freerecvbuf(precvbuf);
			precvbuf++;
		}
		precvpriv->precv_buf = NULL;
	}

	if (precvpriv->pallocated_recv_buf) {
		n = NR_RECVBUFF * sizeof(struct recv_buf) + 4;
		rtw_mfree(precvpriv->pallocated_recv_buf, n);
		precvpriv->pallocated_recv_buf = NULL;
	}
}

void rtl8822bs_rxhandler(PADAPTER adapter, struct recv_buf *recvbuf)
{
	struct recv_priv *recvpriv;
	_queue *pending_queue;


	recvpriv = &adapter->recvpriv;
	pending_queue = &recvpriv->recv_buf_pending_queue;

	/* 1. enqueue recvbuf */
	rtw_enqueue_recvbuf(recvbuf, pending_queue);

	/* 2. schedule tasklet */
	start_rx_handle(adapter);
}
