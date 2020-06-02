/*
 *
 *  Realtek Bluetooth USB driver
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <linux/usb.h>
#include <linux/dcache.h>
#include <net/sock.h>
#include <asm/unaligned.h>

#include "rtk_bt.h"
#include "rtk_misc.h"

#define VERSION "3.1.a2fd257.20190430-133301"

#ifdef BTCOEX
#include "rtk_coex.h"
#endif

#ifdef RTKBT_SWITCH_PATCH
#include <linux/semaphore.h>
#include <net/bluetooth/hci_core.h>
DEFINE_SEMAPHORE(switch_sem);
#endif

#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 7, 1)
static bool reset = 0;
#endif

static struct usb_driver btusb_driver;
static struct usb_device_id btusb_table[] = {
	{
		.match_flags = USB_DEVICE_ID_MATCH_VENDOR |
			USB_DEVICE_ID_MATCH_INT_INFO,
		.idVendor = 0x0bda,
		.bInterfaceClass = 0xe0,
		.bInterfaceSubClass = 0x01,
		.bInterfaceProtocol = 0x01
	}, {
		.match_flags = USB_DEVICE_ID_MATCH_VENDOR |
			USB_DEVICE_ID_MATCH_INT_INFO,
		.idVendor = 0x13d3,
		.bInterfaceClass = 0xe0,
		.bInterfaceSubClass = 0x01,
		.bInterfaceProtocol = 0x01
	}, {
		.match_flags = USB_DEVICE_ID_MATCH_VENDOR |
			USB_DEVICE_ID_MATCH_INT_INFO,
		.idVendor = 0x0489,
		.bInterfaceClass = 0xe0,
		.bInterfaceSubClass = 0x01,
		.bInterfaceProtocol = 0x01
	}, { }
};

static void rtk_free(struct btusb_data *data)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 7, 1)
	kfree(data);
#endif
	return;
}

static struct btusb_data *rtk_alloc(struct usb_interface *intf)
{
	struct btusb_data *data;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 7, 1)
	data = kzalloc(sizeof(*data), GFP_KERNEL);
#else
	data = devm_kzalloc(&intf->dev, sizeof(*data), GFP_KERNEL);
#endif
	return data;
}

MODULE_DEVICE_TABLE(usb, btusb_table);

static int inc_tx(struct btusb_data *data)
{
	unsigned long flags;
	int rv;

	spin_lock_irqsave(&data->txlock, flags);
	rv = test_bit(BTUSB_SUSPENDING, &data->flags);
	if (!rv)
		data->tx_in_flight++;
	spin_unlock_irqrestore(&data->txlock, flags);

	return rv;
}

#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
static inline void btusb_free_frags(struct btusb_data *data)
{
	unsigned long flags;

	spin_lock_irqsave(&data->rxlock, flags);

	kfree_skb(data->evt_skb);
	data->evt_skb = NULL;

	kfree_skb(data->acl_skb);
	data->acl_skb = NULL;

	kfree_skb(data->sco_skb);
	data->sco_skb = NULL;

	spin_unlock_irqrestore(&data->rxlock, flags);
}

static int btusb_recv_intr(struct btusb_data *data, void *buffer, int count)
{
	struct sk_buff *skb;
	int err = 0;

	spin_lock(&data->rxlock);
	skb = data->evt_skb;

	while (count) {
		int len;

		if (!skb) {
			skb = bt_skb_alloc(HCI_MAX_EVENT_SIZE, GFP_ATOMIC);
			if (!skb) {
				err = -ENOMEM;
				break;
			}

			bt_cb(skb)->pkt_type = HCI_EVENT_PKT;
			bt_cb(skb)->expect = HCI_EVENT_HDR_SIZE;
		}

		len = min_t(uint, bt_cb(skb)->expect, count);
		memcpy(skb_put(skb, len), buffer, len);

		count -= len;
		buffer += len;
		bt_cb(skb)->expect -= len;

		if (skb->len == HCI_EVENT_HDR_SIZE) {
			/* Complete event header */
			bt_cb(skb)->expect = hci_event_hdr(skb)->plen;

			if (skb_tailroom(skb) < bt_cb(skb)->expect) {
				kfree_skb(skb);
				skb = NULL;

				err = -EILSEQ;
				break;
			}
		}

		if (bt_cb(skb)->expect == 0) {
			/* Complete frame */
			hci_recv_frame(data->hdev, skb);
			skb = NULL;
		}
	}

	data->evt_skb = skb;
	spin_unlock(&data->rxlock);

	return err;
}

static int btusb_recv_bulk(struct btusb_data *data, void *buffer, int count)
{
	struct sk_buff *skb;
	int err = 0;

	spin_lock(&data->rxlock);
	skb = data->acl_skb;

	while (count) {
		int len;

		if (!skb) {
			skb = bt_skb_alloc(HCI_MAX_FRAME_SIZE, GFP_ATOMIC);
			if (!skb) {
				err = -ENOMEM;
				break;
			}

			bt_cb(skb)->pkt_type = HCI_ACLDATA_PKT;
			bt_cb(skb)->expect = HCI_ACL_HDR_SIZE;
		}

		len = min_t(uint, bt_cb(skb)->expect, count);
		memcpy(skb_put(skb, len), buffer, len);

		count -= len;
		buffer += len;
		bt_cb(skb)->expect -= len;

		if (skb->len == HCI_ACL_HDR_SIZE) {
			__le16 dlen = hci_acl_hdr(skb)->dlen;

			/* Complete ACL header */
			bt_cb(skb)->expect = __le16_to_cpu(dlen);

			if (skb_tailroom(skb) < bt_cb(skb)->expect) {
				kfree_skb(skb);
				skb = NULL;

				err = -EILSEQ;
				break;
			}
		}

		if (bt_cb(skb)->expect == 0) {
			/* Complete frame */
			hci_recv_frame(data->hdev, skb);
			skb = NULL;
		}
	}

	data->acl_skb = skb;
	spin_unlock(&data->rxlock);

	return err;
}

static int btusb_recv_isoc(struct btusb_data *data, void *buffer, int count)
{
	struct sk_buff *skb;
	int err = 0;

	spin_lock(&data->rxlock);
	skb = data->sco_skb;

	while (count) {
		int len;

		if (!skb) {
			skb = bt_skb_alloc(HCI_MAX_SCO_SIZE, GFP_ATOMIC);
			if (!skb) {
				err = -ENOMEM;
				break;
			}

			bt_cb(skb)->pkt_type = HCI_SCODATA_PKT;
			bt_cb(skb)->expect = HCI_SCO_HDR_SIZE;
		}

		len = min_t(uint, bt_cb(skb)->expect, count);
		memcpy(skb_put(skb, len), buffer, len);

		count -= len;
		buffer += len;
		bt_cb(skb)->expect -= len;

		if (skb->len == HCI_SCO_HDR_SIZE) {
			/* Complete SCO header */
			bt_cb(skb)->expect = hci_sco_hdr(skb)->dlen;

			if (skb_tailroom(skb) < bt_cb(skb)->expect) {
				kfree_skb(skb);
				skb = NULL;

				err = -EILSEQ;
				break;
			}
		}

		if (bt_cb(skb)->expect == 0) {
			/* Complete frame */
			hci_recv_frame(data->hdev, skb);
			skb = NULL;
		}
	}

	data->sco_skb = skb;
	spin_unlock(&data->rxlock);

	return err;
}
#endif

static void btusb_intr_complete(struct urb *urb)
{
	struct hci_dev *hdev = urb->context;
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int err;

	//RTKBT_DBG("%s: urb %p status %d count %d ", __func__,
	//urb, urb->status, urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return;

	if (urb->status == 0) {
		hdev->stat.byte_rx += urb->actual_length;

#ifdef BTCOEX
		rtk_btcoex_parse_event(urb->transfer_buffer,
				urb->actual_length);
#endif
#if HCI_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
		if (hci_recv_fragment(hdev, HCI_EVENT_PKT,
				      urb->transfer_buffer,
				      urb->actual_length) < 0) {
			RTKBT_ERR("%s: Corrupted event packet", __func__);
			hdev->stat.err_rx++;
		}
#else
		if (btusb_recv_intr(data, urb->transfer_buffer,
				    urb->actual_length) < 0) {
			RTKBT_ERR("%s corrupted event packet", hdev->name);
			hdev->stat.err_rx++;
		}
#endif
	}
	/* Avoid suspend failed when usb_kill_urb */
	else if (urb->status == -ENOENT) {
		return;
	}

	if (!test_bit(BTUSB_INTR_RUNNING, &data->flags))
		return;

	usb_mark_last_busy(data->udev);
	usb_anchor_urb(urb, &data->intr_anchor);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected */
		if (err != -EPERM && err != -ENODEV)
			RTKBT_ERR("%s: Failed to re-submit urb %p, err %d",
				  __func__, urb, err);
		usb_unanchor_urb(urb);
	}
}

static int btusb_submit_intr_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size;

	//RTKBT_DBG("%s", hdev->name);

	if (!data->intr_ep)
		return -ENODEV;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;

	size = le16_to_cpu(data->intr_ep->wMaxPacketSize);

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvintpipe(data->udev, data->intr_ep->bEndpointAddress);

	usb_fill_int_urb(urb, data->udev, pipe, buf, size,
			 btusb_intr_complete, hdev, data->intr_ep->bInterval);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_anchor_urb(urb, &data->intr_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		RTKBT_ERR
		    ("btusb_submit_intr_urb %s urb %p submission failed (%d)",
		     hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_bulk_complete(struct urb *urb)
{
	struct hci_dev *hdev = urb->context;
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int err;

	//RTKBT_DBG("%s: urb %p status %d count %d",
	//__func__, urb, urb->status, urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return;

#ifdef BTCOEX
	if (urb->status == 0)
		rtk_btcoex_parse_l2cap_data_rx(urb->transfer_buffer,
				urb->actual_length);
#endif

	if (urb->status == 0) {
		hdev->stat.byte_rx += urb->actual_length;

#if HCI_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
		if (hci_recv_fragment(hdev, HCI_ACLDATA_PKT,
				      urb->transfer_buffer,
				      urb->actual_length) < 0) {
			RTKBT_ERR("%s: Corrupted ACL packet", __func__);
			hdev->stat.err_rx++;
		}
#else
		if (data->recv_bulk(data, urb->transfer_buffer,
				    urb->actual_length) < 0) {
			RTKBT_ERR("%s corrupted ACL packet", hdev->name);
			hdev->stat.err_rx++;
		}
#endif
	}
	/* Avoid suspend failed when usb_kill_urb */
	else if (urb->status == -ENOENT) {
		return;
	}

	if (!test_bit(BTUSB_BULK_RUNNING, &data->flags))
		return;

	usb_anchor_urb(urb, &data->bulk_anchor);
	usb_mark_last_busy(data->udev);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected */
		if (err != -EPERM && err != -ENODEV)
			RTKBT_ERR
			    ("btusb_bulk_complete %s urb %p failed to resubmit (%d)",
			     hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}
}

static int btusb_submit_bulk_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size = HCI_MAX_FRAME_SIZE;

	//RTKBT_DBG("%s: hdev name %s", __func__, hdev->name);

	if (!data->bulk_rx_ep)
		return -ENODEV;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvbulkpipe(data->udev, data->bulk_rx_ep->bEndpointAddress);

	usb_fill_bulk_urb(urb, data->udev, pipe,
			  buf, size, btusb_bulk_complete, hdev);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_mark_last_busy(data->udev);
	usb_anchor_urb(urb, &data->bulk_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		RTKBT_ERR("%s: Failed to submit urb %p, err %d", __func__, urb,
			  err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_isoc_complete(struct urb *urb)
{
	struct hci_dev *hdev = urb->context;
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int i, err;

	/*
	   RTKBT_DBG("%s urb %p status %d count %d", hdev->name,
	   urb, urb->status, urb->actual_length);
	 */
	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return;

	if (urb->status == 0) {
		for (i = 0; i < urb->number_of_packets; i++) {
			unsigned int offset = urb->iso_frame_desc[i].offset;
			unsigned int length =
			    urb->iso_frame_desc[i].actual_length;

			if (urb->iso_frame_desc[i].status)
				continue;

			hdev->stat.byte_rx += length;

#if HCI_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
			if (hci_recv_fragment(hdev, HCI_SCODATA_PKT,
					      urb->transfer_buffer + offset,
					      length) < 0) {
				RTKBT_ERR("%s: Corrupted SCO packet", __func__);
				hdev->stat.err_rx++;
			}
#else
			if (btusb_recv_isoc(data, urb->transfer_buffer + offset,
					    length) < 0) {
				RTKBT_ERR("%s corrupted SCO packet",
					  hdev->name);
				hdev->stat.err_rx++;
			}
#endif
		}
	}
	/* Avoid suspend failed when usb_kill_urb */
	else if (urb->status == -ENOENT) {
		return;
	}

	if (!test_bit(BTUSB_ISOC_RUNNING, &data->flags))
		return;

	usb_anchor_urb(urb, &data->isoc_anchor);
	i = 0;
retry:
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected */
		if (err != -EPERM && err != -ENODEV)
			RTKBT_ERR
			    ("%s: Failed to re-sumbit urb %p, retry %d, err %d",
			     __func__, urb, i, err);
		if (i < 10) {
			i++;
			mdelay(1);
			goto retry;
		}

		usb_unanchor_urb(urb);
	}
}

static inline void __fill_isoc_descriptor(struct urb *urb, int len, int mtu)
{
	int i, offset = 0;

	//RTKBT_DBG("len %d mtu %d", len, mtu);

	for (i = 0; i < BTUSB_MAX_ISOC_FRAMES && len >= mtu;
	     i++, offset += mtu, len -= mtu) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = mtu;
	}

	if (len && i < BTUSB_MAX_ISOC_FRAMES) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = len;
		i++;
	}

	urb->number_of_packets = i;
}

static int btusb_submit_isoc_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size;

	//RTKBT_DBG("%s", hdev->name);

	if (!data->isoc_rx_ep)
		return -ENODEV;

	urb = usb_alloc_urb(BTUSB_MAX_ISOC_FRAMES, mem_flags);
	if (!urb)
		return -ENOMEM;

	size = le16_to_cpu(data->isoc_rx_ep->wMaxPacketSize) *
	    BTUSB_MAX_ISOC_FRAMES;

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvisocpipe(data->udev, data->isoc_rx_ep->bEndpointAddress);

	urb->dev = data->udev;
	urb->pipe = pipe;
	urb->context = hdev;
	urb->complete = btusb_isoc_complete;
	urb->interval = data->isoc_rx_ep->bInterval;

	urb->transfer_flags = URB_FREE_BUFFER | URB_ISO_ASAP;
	urb->transfer_buffer = buf;
	urb->transfer_buffer_length = size;

	__fill_isoc_descriptor(urb, size,
			       le16_to_cpu(data->isoc_rx_ep->wMaxPacketSize));

	usb_anchor_urb(urb, &data->isoc_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		RTKBT_ERR("%s %s urb %p submission failed (%d)",
			  __func__, hdev->name, urb, err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_tx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;
	struct btusb_data *data = GET_DRV_DATA(hdev);

//      RTKBT_DBG("btusb_tx_complete %s urb %p status %d count %d", hdev->name,
//                                      urb, urb->status, urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		goto done;

	if (!urb->status)
		hdev->stat.byte_tx += urb->transfer_buffer_length;
	else
		hdev->stat.err_tx++;

done:
	spin_lock(&data->txlock);
	data->tx_in_flight--;
	spin_unlock(&data->txlock);

	kfree(urb->setup_packet);

	kfree_skb(skb);
}

static void btusb_isoc_tx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;

	//RTKBT_DBG("%s: urb %p status %d count %d",
	//__func__, urb, urb->status, urb->actual_length);

	if (skb && hdev) {
		if (!test_bit(HCI_RUNNING, &hdev->flags))
			goto done;

		if (!urb->status)
			hdev->stat.byte_tx += urb->transfer_buffer_length;
		else
			hdev->stat.err_tx++;
	} else
		RTKBT_ERR("%s: skb 0x%p hdev 0x%p", __func__, skb, hdev);

done:
	kfree(urb->setup_packet);

	kfree_skb(skb);
}

static int btusb_open(struct hci_dev *hdev)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int err;

	err = usb_autopm_get_interface(data->intf);
	if (err < 0)
		return err;

	data->intf->needs_remote_wakeup = 1;
	RTKBT_DBG("%s start pm_usage_cnt(0x%x)", __func__,
		  atomic_read(&(data->intf->pm_usage_cnt)));

	/*******************************/
	if (0 == atomic_read(&hdev->promisc)) {
		RTKBT_ERR("btusb_open hdev->promisc ==0");
		err = -1;
		//goto failed;
	}

	err = download_patch(data->intf);
	if (err < 0)
		goto failed;
	/*******************************/

	RTKBT_INFO("%s set HCI_RUNNING", __func__);
	if (test_and_set_bit(HCI_RUNNING, &hdev->flags))
		goto done;

	if (test_and_set_bit(BTUSB_INTR_RUNNING, &data->flags))
		goto done;

	err = btusb_submit_intr_urb(hdev, GFP_KERNEL);
	if (err < 0)
		goto failed;

	err = btusb_submit_bulk_urb(hdev, GFP_KERNEL);
	if (err < 0) {
		mdelay(URB_CANCELING_DELAY_MS);	// Added by Realtek
		usb_kill_anchored_urbs(&data->intr_anchor);
		goto failed;
	}

	set_bit(BTUSB_BULK_RUNNING, &data->flags);
	btusb_submit_bulk_urb(hdev, GFP_KERNEL);

done:
	usb_autopm_put_interface(data->intf);

#ifdef BTCOEX
	rtk_btcoex_open(hdev);
#endif
	RTKBT_DBG("%s end  pm_usage_cnt(0x%x)", __FUNCTION__,
		  atomic_read(&(data->intf->pm_usage_cnt)));

	return 0;

failed:
	clear_bit(BTUSB_INTR_RUNNING, &data->flags);
	clear_bit(HCI_RUNNING, &hdev->flags);
	usb_autopm_put_interface(data->intf);
	RTKBT_ERR("%s failed  pm_usage_cnt(0x%x)", __FUNCTION__,
		  atomic_read(&(data->intf->pm_usage_cnt)));
	return err;
}

static void btusb_stop_traffic(struct btusb_data *data)
{
	mdelay(URB_CANCELING_DELAY_MS);	// Added by Realtek
	usb_kill_anchored_urbs(&data->intr_anchor);
	usb_kill_anchored_urbs(&data->bulk_anchor);
	usb_kill_anchored_urbs(&data->isoc_anchor);
}

static int btusb_close(struct hci_dev *hdev)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int err;

#if HCI_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	int i;
#endif

	/* When in kernel 4.4.0 and greater, the HCI_RUNNING bit is
	 * cleared in hci_dev_do_close(). */
#if HCI_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
	if (!test_and_clear_bit(HCI_RUNNING, &hdev->flags))
		return 0;
#else
	if (test_bit(HCI_RUNNING, &hdev->flags)) {
		RTKBT_ERR("HCI_RUNNING is not cleared before.");
		return -1;
	}
#endif

	RTKBT_DBG("btusb_close");
#if HCI_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	/*******************************/
	for (i = 0; i < NUM_REASSEMBLY; i++) {
		if (hdev->reassembly[i]) {
			kfree_skb(hdev->reassembly[i]);
			hdev->reassembly[i] = NULL;
			RTKBT_DBG("%s free ressembly i=%d", __FUNCTION__, i);
		}
	}
	/*******************************/
#endif
	cancel_work_sync(&data->work);
	cancel_work_sync(&data->waker);

	clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
	clear_bit(BTUSB_BULK_RUNNING, &data->flags);
	clear_bit(BTUSB_INTR_RUNNING, &data->flags);

	btusb_stop_traffic(data);
#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	btusb_free_frags(data);
#endif

	err = usb_autopm_get_interface(data->intf);
	if (err < 0)
		goto failed;

	data->intf->needs_remote_wakeup = 0;
	usb_autopm_put_interface(data->intf);

#ifdef BTCOEX
	rtk_btcoex_close();
#endif

failed:
	mdelay(URB_CANCELING_DELAY_MS);	// Added by Realtek
	usb_scuttle_anchored_urbs(&data->deferred);

#ifdef RTKBT_SWITCH_PATCH
	down(&switch_sem);
	if (data->context) {
		struct api_context *ctx = data->context;

		if (ctx->flags & RTLBT_CLOSE) {
			ctx->flags &= ~RTLBT_CLOSE;
			ctx->status = 0;
			complete(&ctx->done);
		}
	}
	up(&switch_sem);
#endif

	return 0;
}

static int btusb_flush(struct hci_dev *hdev)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);

	RTKBT_DBG("%s add delay ", __FUNCTION__);
	mdelay(URB_CANCELING_DELAY_MS);	// Added by Realtek
	usb_kill_anchored_urbs(&data->tx_anchor);
#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	btusb_free_frags(data);
#endif

	return 0;
}

const char pkt_ind[][8] = {
	[HCI_COMMAND_PKT] = "cmd",
	[HCI_ACLDATA_PKT] = "acl",
	[HCI_SCODATA_PKT] = "sco",
};

#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
int btusb_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
#else
int btusb_send_frame(struct sk_buff *skb)
{
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;
#endif

	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct usb_ctrlrequest *dr;
	struct urb *urb;
	unsigned int pipe;
	int err;

	//RTKBT_DBG("%s", hdev->name);

	if (!test_bit(HCI_RUNNING, &hdev->flags)) {
		/* If the parameter is wrong, the hdev isn't the correct
		 * one. Then no HCI commands can be sent.
		 * This issue is related to the wrong HCI_VERSION_CODE set */
		RTKBT_ERR("HCI is not running");
		return -EBUSY;
	}

	/* Before kernel/hci version 3.13.0, the skb->dev is set before
	 * entering btusb_send_frame(). So there is no need to set it here.
	 *
	 * The skb->dev will be used in the callbacks when urb transfer
	 * completes. See btusb_tx_complete() and btusb_isoc_tx_complete() */
#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
	skb->dev = (void *)hdev;
#endif

	switch (bt_cb(skb)->pkt_type) {
	case HCI_COMMAND_PKT:
		print_command(skb);

#ifdef BTCOEX
		rtk_btcoex_parse_cmd(skb->data, skb->len);
#endif
		urb = usb_alloc_urb(0, GFP_ATOMIC);
		if (!urb)
			return -ENOMEM;

		dr = kmalloc(sizeof(*dr), GFP_ATOMIC);
		if (!dr) {
			usb_free_urb(urb);
			return -ENOMEM;
		}

		dr->bRequestType = data->cmdreq_type;
		dr->bRequest = 0;
		dr->wIndex = 0;
		dr->wValue = 0;
		dr->wLength = __cpu_to_le16(skb->len);

		pipe = usb_sndctrlpipe(data->udev, 0x00);

		usb_fill_control_urb(urb, data->udev, pipe, (void *)dr,
				     skb->data, skb->len, btusb_tx_complete,
				     skb);

		hdev->stat.cmd_tx++;
		break;

	case HCI_ACLDATA_PKT:
		print_acl(skb, 1);
#ifdef BTCOEX
		rtk_btcoex_parse_l2cap_data_tx(skb->data, skb->len);
#endif
		if (!data->bulk_tx_ep)
			return -ENODEV;

		urb = usb_alloc_urb(0, GFP_ATOMIC);
		if (!urb)
			return -ENOMEM;

		pipe = usb_sndbulkpipe(data->udev,
				       data->bulk_tx_ep->bEndpointAddress);

		usb_fill_bulk_urb(urb, data->udev, pipe,
				  skb->data, skb->len, btusb_tx_complete, skb);

		hdev->stat.acl_tx++;
		break;

	case HCI_SCODATA_PKT:
		if (!data->isoc_tx_ep || SCO_NUM < 1)
			return -ENODEV;

		urb = usb_alloc_urb(BTUSB_MAX_ISOC_FRAMES, GFP_ATOMIC);
		if (!urb)
			return -ENOMEM;

		pipe = usb_sndisocpipe(data->udev,
				       data->isoc_tx_ep->bEndpointAddress);

		usb_fill_int_urb(urb, data->udev, pipe,
				 skb->data, skb->len, btusb_isoc_tx_complete,
				 skb, data->isoc_tx_ep->bInterval);

		urb->transfer_flags = URB_ISO_ASAP;

		__fill_isoc_descriptor(urb, skb->len,
				       le16_to_cpu(data->isoc_tx_ep->
						   wMaxPacketSize));

		hdev->stat.sco_tx++;
		goto skip_waking;

	default:
		return -EILSEQ;
	}

	err = inc_tx(data);
	if (err) {
		usb_anchor_urb(urb, &data->deferred);
		schedule_work(&data->waker);
		err = 0;
		goto done;
	}

skip_waking:
	usb_anchor_urb(urb, &data->tx_anchor);
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		RTKBT_ERR("%s %s urb %p submission for %s failed, err %d",
			  __func__, hdev->name, urb,
			  pkt_ind[bt_cb(skb)->pkt_type], err);
		kfree(urb->setup_packet);
		usb_unanchor_urb(urb);
	} else {
		usb_mark_last_busy(data->udev);
	}
	usb_free_urb(urb);

done:
	return err;
}

#if HCI_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
static void btusb_destruct(struct hci_dev *hdev)
{
	RTKBT_DBG("btusb_destruct %s", hdev->name);
	hci_free_dev(hdev);
}
#endif

static void btusb_notify(struct hci_dev *hdev, unsigned int evt)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);

	RTKBT_DBG("%s: %s evt %d", __func__, hdev->name, evt);

	if (SCO_NUM != data->sco_num) {
		data->sco_num = SCO_NUM;
		RTKBT_DBG("%s: Update sco num %d", __func__, data->sco_num);
		schedule_work(&data->work);
	}
}

static inline int __set_isoc_interface(struct hci_dev *hdev, int altsetting)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct usb_interface *intf = data->isoc;
	struct usb_endpoint_descriptor *ep_desc;
	int i, err;

	if (!data->isoc)
		return -ENODEV;

	RTKBT_INFO("set isoc interface: alt %d", altsetting);

	err = usb_set_interface(data->udev, 1, altsetting);
	if (err < 0) {
		RTKBT_ERR("%s setting interface failed (%d)", hdev->name, -err);
		return err;
	}

	data->isoc_altsetting = altsetting;

	data->isoc_tx_ep = NULL;
	data->isoc_rx_ep = NULL;

	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		ep_desc = &intf->cur_altsetting->endpoint[i].desc;

		if (!data->isoc_tx_ep && usb_endpoint_is_isoc_out(ep_desc)) {
			data->isoc_tx_ep = ep_desc;
			continue;
		}

		if (!data->isoc_rx_ep && usb_endpoint_is_isoc_in(ep_desc)) {
			data->isoc_rx_ep = ep_desc;
			continue;
		}
	}

	if (!data->isoc_tx_ep || !data->isoc_rx_ep) {
		RTKBT_ERR("%s invalid SCO descriptors", hdev->name);
		return -ENODEV;
	}

	return 0;
}

static void btusb_work(struct work_struct *work)
{
	struct btusb_data *data = container_of(work, struct btusb_data, work);
	struct hci_dev *hdev = data->hdev;
	int err;
	int new_alts;

	RTKBT_DBG("%s: sco num %d", __func__, data->sco_num);
	if (data->sco_num > 0) {
		if (!test_bit(BTUSB_DID_ISO_RESUME, &data->flags)) {
			err =
			    usb_autopm_get_interface(data->isoc ? data->
						     isoc : data->intf);
			if (err < 0) {
				clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
				mdelay(URB_CANCELING_DELAY_MS);
				usb_kill_anchored_urbs(&data->isoc_anchor);
				return;
			}

			set_bit(BTUSB_DID_ISO_RESUME, &data->flags);
		}
#if HCI_VERSION_CODE > KERNEL_VERSION(3, 7, 1)
		if (hdev->voice_setting & 0x0020) {
			static const int alts[3] = { 2, 4, 5 };
			new_alts = alts[data->sco_num - 1];
		} else {
			new_alts = data->sco_num;
		}
		if (data->isoc_altsetting != new_alts) {
#else
		if (data->isoc_altsetting != 2) {
			new_alts = 2;
#endif

			clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
			mdelay(URB_CANCELING_DELAY_MS);
			usb_kill_anchored_urbs(&data->isoc_anchor);

			if (__set_isoc_interface(hdev, new_alts) < 0)
				return;
		}

		if (!test_and_set_bit(BTUSB_ISOC_RUNNING, &data->flags)) {
			RTKBT_INFO("submit SCO RX urb.");
			if (btusb_submit_isoc_urb(hdev, GFP_KERNEL) < 0)
				clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
			else
				btusb_submit_isoc_urb(hdev, GFP_KERNEL);
		}
	} else {
		clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
		mdelay(URB_CANCELING_DELAY_MS);
		usb_kill_anchored_urbs(&data->isoc_anchor);

		__set_isoc_interface(hdev, 0);
		if (test_and_clear_bit(BTUSB_DID_ISO_RESUME, &data->flags))
			usb_autopm_put_interface(data->isoc ? data->
						 isoc : data->intf);
	}
}

static void btusb_waker(struct work_struct *work)
{
	struct btusb_data *data = container_of(work, struct btusb_data, waker);
	int err;

	err = usb_autopm_get_interface(data->intf);
	RTKBT_DBG("%s start  pm_usage_cnt(0x%x)", __FUNCTION__,
		  atomic_read(&(data->intf->pm_usage_cnt)));
	if (err < 0)
		return;

	usb_autopm_put_interface(data->intf);
	RTKBT_DBG("%s end  pm_usage_cnt(0x%x)", __FUNCTION__,
		  atomic_read(&(data->intf->pm_usage_cnt)));
}

int rtkbt_pm_notify(struct notifier_block *notifier,
		    ulong pm_event, void *unused)
{
	struct btusb_data *data;
	struct usb_device *udev;
	struct usb_interface *intf;
	struct hci_dev *hdev;
	/* int err; */
#ifdef RTKBT_SWITCH_PATCH
	u8 *cmd;
	int result;
	static u8 hci_state = 0;
	struct api_context ctx;
#endif

	data = container_of(notifier, struct btusb_data, pm_notifier);
	udev = data->udev;
	intf = data->intf;
	hdev = data->hdev;

	RTKBT_DBG("%s: pm_event %ld", __func__, pm_event);
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		/* No need to load firmware because the download firmware
		 * process is deprecated in resume.
		 * We use rebind after resume instead */
		/* err = usb_autopm_get_interface(data->intf);
		 * if (err < 0)
		 * 	return err;
		 * patch_entry->fw_len =
		 *     load_firmware(dev_entry, &patch_entry->fw_cache);
		 * usb_autopm_put_interface(data->intf);
		 * if (patch_entry->fw_len <= 0) {
		 * 	RTKBT_DBG("rtkbt_pm_notify return NOTIFY_BAD");
		 * 	return NOTIFY_BAD;
		 * } */

		RTKBT_DBG("%s: suspend prepare", __func__);

		if (!device_may_wakeup(&udev->dev)) {
#ifdef CONFIG_NEEDS_BINDING
			intf->needs_binding = 1;
			RTKBT_DBG("Remote wakeup not support, set "
				  "intf->needs_binding = 1");
#else
			RTKBT_DBG("Remote wakeup not support, no needs binding");
#endif
		}

#ifdef RTKBT_SWITCH_PATCH
		if (test_bit(HCI_UP, &hdev->flags)) {
			unsigned long expire;

			init_completion(&ctx.done);
			hci_state = 1;

			down(&switch_sem);
			data->context = &ctx;
			ctx.flags = RTLBT_CLOSE;
			queue_work(hdev->req_workqueue, &hdev->power_off.work);
			up(&switch_sem);

			expire = msecs_to_jiffies(1000);
			if (!wait_for_completion_timeout(&ctx.done, expire))
				RTKBT_ERR("hdev close timeout");

			down(&switch_sem);
			data->context = NULL;
			up(&switch_sem);
		}

		cmd = kzalloc(16, GFP_ATOMIC);
		if (!cmd) {
			RTKBT_ERR("Can't allocate memory for cmd");
			return -ENOMEM;
		}

		/* Clear patch */
		cmd[0] = 0x66;
		cmd[1] = 0xfc;
		cmd[2] = 0x00;

		result = __rtk_send_hci_cmd(udev, cmd, 3);
		kfree(cmd);
		msleep(100); /* From FW colleague's recommendation */
		result = download_lps_patch(intf);

		/* Tell the controller to wake up host if received special
		 * advertising packet
		 */
		set_scan(intf);

		/* Send special vendor commands */
#endif

		break;

	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
		/* if (patch_entry->fw_len > 0) {
		 * 	kfree(patch_entry->fw_cache);
		 * 	patch_entry->fw_cache = NULL;
		 * 	patch_entry->fw_len = 0;
		 * } */

#ifdef RTKBT_SWITCH_PATCH
		cmd = kzalloc(16, GFP_ATOMIC);
		if (!cmd) {
			RTKBT_ERR("Can't allocate memory for cmd");
			return -ENOMEM;
		}

		/* Clear patch */
		cmd[0] = 0x66;
		cmd[1] = 0xfc;
		cmd[2] = 0x00;

		result = __rtk_send_hci_cmd(udev, cmd, 3);
		kfree(cmd);
		msleep(100); /* From FW colleague's recommendation */
		result = download_patch(intf);
		if (hci_state) {
			hci_state = 0;
			queue_work(hdev->req_workqueue, &hdev->power_on);
		}
#endif

#if BTUSB_RPM
		RTKBT_DBG("%s: Re-enable autosuspend", __func__);
		/* pm_runtime_use_autosuspend(&udev->dev);
		 * pm_runtime_set_autosuspend_delay(&udev->dev, 2000);
		 * pm_runtime_set_active(&udev->dev);
		 * pm_runtime_allow(&udev->dev);
		 * pm_runtime_mark_last_busy(&udev->dev);
		 * pm_runtime_autosuspend(&udev->dev);
		 * pm_runtime_put_autosuspend(&udev->dev);
		 * usb_disable_autosuspend(udev); */
		/* FIXME: usb_enable_autosuspend(udev) is useless here.
		 * Because it is always enabled after enabled in btusb_probe()
		 */
		usb_enable_autosuspend(udev);
		pm_runtime_mark_last_busy(&udev->dev);
#endif
		break;

	default:
		break;
	}

	return NOTIFY_DONE;
}


static int btusb_probe(struct usb_interface *intf,
		       const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *ep_desc;
	struct btusb_data *data;
	struct hci_dev *hdev;
	int i, err, flag1, flag2;
	struct usb_device *udev;
	udev = interface_to_usbdev(intf);

	RTKBT_DBG("btusb_probe intf->cur_altsetting->desc.bInterfaceNumber %d",
		  intf->cur_altsetting->desc.bInterfaceNumber);

	/* interface numbers are hardcoded in the spec */
	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return -ENODEV;

	/*******************************/
	flag1 = device_can_wakeup(&udev->dev);
	flag2 = device_may_wakeup(&udev->dev);
	RTKBT_DBG("btusb_probe can_wakeup %x, may wakeup %x", flag1, flag2);
#if BTUSB_WAKEUP_HOST
	device_wakeup_enable(&udev->dev);
#endif
	//device_wakeup_enable(&udev->dev);
	/*device_wakeup_disable(&udev->dev);
	   flag1=device_can_wakeup(&udev->dev);
	   flag2=device_may_wakeup(&udev->dev);
	   RTKBT_DBG("btusb_probe can_wakeup=%x  flag2=%x",flag1,flag2);
	 */
	err = patch_add(intf);
	if (err < 0)
		return -1;
	/*******************************/

	data = rtk_alloc(intf);
	if (!data)
		return -ENOMEM;

	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		ep_desc = &intf->cur_altsetting->endpoint[i].desc;

		if (!data->intr_ep && usb_endpoint_is_int_in(ep_desc)) {
			data->intr_ep = ep_desc;
			continue;
		}

		if (!data->bulk_tx_ep && usb_endpoint_is_bulk_out(ep_desc)) {
			data->bulk_tx_ep = ep_desc;
			continue;
		}

		if (!data->bulk_rx_ep && usb_endpoint_is_bulk_in(ep_desc)) {
			data->bulk_rx_ep = ep_desc;
			continue;
		}
	}

	if (!data->intr_ep || !data->bulk_tx_ep || !data->bulk_rx_ep) {
		rtk_free(data);
		return -ENODEV;
	}

	data->cmdreq_type = USB_TYPE_CLASS;

	data->udev = interface_to_usbdev(intf);
	data->intf = intf;

	spin_lock_init(&data->lock);

	INIT_WORK(&data->work, btusb_work);
	INIT_WORK(&data->waker, btusb_waker);
	spin_lock_init(&data->txlock);

	init_usb_anchor(&data->tx_anchor);
	init_usb_anchor(&data->intr_anchor);
	init_usb_anchor(&data->bulk_anchor);
	init_usb_anchor(&data->isoc_anchor);
	init_usb_anchor(&data->deferred);

#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	spin_lock_init(&data->rxlock);
	data->recv_bulk = btusb_recv_bulk;
#endif

	hdev = hci_alloc_dev();
	if (!hdev) {
		rtk_free(data);
		return -ENOMEM;
	}

	HDEV_BUS = HCI_USB;

	data->hdev = hdev;

	SET_HCIDEV_DEV(hdev, &intf->dev);

	hdev->open = btusb_open;
	hdev->close = btusb_close;
	hdev->flush = btusb_flush;
	hdev->send = btusb_send_frame;
	hdev->notify = btusb_notify;

#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
	hci_set_drvdata(hdev, data);
#else
	hdev->driver_data = data;
	hdev->destruct = btusb_destruct;
	hdev->owner = THIS_MODULE;
#endif

#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 7, 1)
	if (!reset)
		set_bit(HCI_QUIRK_RESET_ON_CLOSE, &hdev->quirks);
	RTKBT_DBG("set_bit(HCI_QUIRK_RESET_ON_CLOSE, &hdev->quirks);");
#endif

	/* Interface numbers are hardcoded in the specification */
	data->isoc = usb_ifnum_to_if(data->udev, 1);

	if (data->isoc) {
		err = usb_driver_claim_interface(&btusb_driver,
						 data->isoc, data);
		if (err < 0) {
			hci_free_dev(hdev);
			rtk_free(data);
			return err;
		}
	}

#if HCI_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	set_bit(HCI_QUIRK_SIMULTANEOUS_DISCOVERY, &hdev->quirks);
#endif

	err = hci_register_dev(hdev);
	if (err < 0) {
		hci_free_dev(hdev);
		rtk_free(data);
		return err;
	}

	usb_set_intfdata(intf, data);

	/* Register PM notifier */
	data->pm_notifier.notifier_call = rtkbt_pm_notify;
	register_pm_notifier(&data->pm_notifier);

#ifdef BTCOEX
	rtk_btcoex_probe(hdev);
#endif

	RTKBT_DBG("%s: done", __func__);

	return 0;
}

static void btusb_disconnect(struct usb_interface *intf)
{
	struct btusb_data *data = usb_get_intfdata(intf);
	struct hci_dev *hdev;
	struct usb_device *udev;
	udev = interface_to_usbdev(intf);

	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return;

	if (!data)
		return;

	RTKBT_DBG("btusb_disconnect");

	/* Un-register PM notifier */
	unregister_pm_notifier(&data->pm_notifier);

	/*******************************/
	patch_remove(intf);
	/*******************************/

	hdev = data->hdev;

#if HCI_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
	__hci_dev_hold(hdev);
#endif

	usb_set_intfdata(data->intf, NULL);

	if (data->isoc)
		usb_set_intfdata(data->isoc, NULL);

	hci_unregister_dev(hdev);

	if (intf == data->isoc)
		usb_driver_release_interface(&btusb_driver, data->intf);
	else if (data->isoc)
		usb_driver_release_interface(&btusb_driver, data->isoc);

#if HCI_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
	__hci_dev_put(hdev);
#endif

#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	btusb_free_frags(data);
#endif

	hci_free_dev(hdev);
	rtk_free(data);
}

#ifdef CONFIG_PM
static int btusb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct btusb_data *data = usb_get_intfdata(intf);

	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return 0;

	/*******************************/
	RTKBT_DBG("btusb_suspend message.event 0x%x, data->suspend_count %d",
		  message.event, data->suspend_count);
	if (!test_bit(HCI_RUNNING, &data->hdev->flags)) {
		RTKBT_INFO("%s: hdev is not HCI_RUNNING", __func__);
		/* set_scan(data->intf); */
	}
	/*******************************/

	if (data->suspend_count++)
		return 0;

	spin_lock_irq(&data->txlock);
	if (!((message.event & PM_EVENT_AUTO) && data->tx_in_flight)) {
		set_bit(BTUSB_SUSPENDING, &data->flags);
		spin_unlock_irq(&data->txlock);
		RTKBT_INFO("%s: suspending...", __func__);
	} else {
		spin_unlock_irq(&data->txlock);
		data->suspend_count--;
		return -EBUSY;
	}

	cancel_work_sync(&data->work);

	btusb_stop_traffic(data);
	mdelay(URB_CANCELING_DELAY_MS);	// Added by Realtek
	usb_kill_anchored_urbs(&data->tx_anchor);

	return 0;
}

static void play_deferred(struct btusb_data *data)
{
	struct urb *urb;
	int err;

	while ((urb = usb_get_from_anchor(&data->deferred))) {
	    /************************************/
		usb_anchor_urb(urb, &data->tx_anchor);
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err < 0) {
			RTKBT_ERR("play_deferred urb %p submission failed",
				  urb);
			kfree(urb->setup_packet);
			usb_unanchor_urb(urb);
		} else {
			usb_mark_last_busy(data->udev);
		}
		usb_free_urb(urb);
		/************************************/
		data->tx_in_flight++;
	}
	mdelay(URB_CANCELING_DELAY_MS);	// Added by Realtek
	usb_scuttle_anchored_urbs(&data->deferred);
}

static int btusb_resume(struct usb_interface *intf)
{
	struct btusb_data *data = usb_get_intfdata(intf);
	struct hci_dev *hdev = data->hdev;
	int err = 0;

	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return 0;

	/*******************************/
	RTKBT_DBG("%s: data->suspend_count %d", __func__, data->suspend_count);

	/* if intf->needs_binding is set, driver will be rebind.
	 * The probe will be called instead of resume */
	/* if (!test_bit(HCI_RUNNING, &hdev->flags)) {
	 * 	RTKBT_DBG("btusb_resume-----bt is off,download patch");
	 * 	download_patch(intf);
	 * } else
	 * 	RTKBT_DBG("btusb_resume,----bt is on");
	 */
	/*******************************/
	if (--data->suspend_count)
		return 0;

	if (test_bit(BTUSB_INTR_RUNNING, &data->flags)) {
		err = btusb_submit_intr_urb(hdev, GFP_NOIO);
		if (err < 0) {
			clear_bit(BTUSB_INTR_RUNNING, &data->flags);
			goto failed;
		}
	}

	if (test_bit(BTUSB_BULK_RUNNING, &data->flags)) {
		err = btusb_submit_bulk_urb(hdev, GFP_NOIO);
		if (err < 0) {
			clear_bit(BTUSB_BULK_RUNNING, &data->flags);
			goto failed;
		}

		btusb_submit_bulk_urb(hdev, GFP_NOIO);
	}

	if (test_bit(BTUSB_ISOC_RUNNING, &data->flags)) {
		if (btusb_submit_isoc_urb(hdev, GFP_NOIO) < 0)
			clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
		else
			btusb_submit_isoc_urb(hdev, GFP_NOIO);
	}

	spin_lock_irq(&data->txlock);
	play_deferred(data);
	clear_bit(BTUSB_SUSPENDING, &data->flags);
	spin_unlock_irq(&data->txlock);
	schedule_work(&data->work);

	RTKBT_DBG("%s: data->suspend_count %d, done", __func__,
		  data->suspend_count);

	return 0;

failed:
	mdelay(URB_CANCELING_DELAY_MS);	// Added by Realtek
	usb_scuttle_anchored_urbs(&data->deferred);
//done:
	spin_lock_irq(&data->txlock);
	clear_bit(BTUSB_SUSPENDING, &data->flags);
	spin_unlock_irq(&data->txlock);
	RTKBT_DBG("%s: data->suspend_count %d, fail", __func__,
		  data->suspend_count);

	return err;
}
#endif

static struct usb_driver btusb_driver = {
	.name = "rtk_btusb",
	.probe = btusb_probe,
	.disconnect = btusb_disconnect,
#ifdef CONFIG_PM
	.suspend = btusb_suspend,
	.resume = btusb_resume,
#ifdef RTKBT_SWITCH_PATCH
	.reset_resume = btusb_resume,
#endif
#endif
	.id_table = btusb_table,
	.supports_autosuspend = 1,
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 1)
	.disable_hub_initiated_lpm = 1,
#endif
};

static int __init btusb_init(void)
{
	RTKBT_DBG("Realtek Bluetooth USB driver ver %s", VERSION);
#ifdef BTCOEX
	rtk_btcoex_init();
#endif
	return usb_register(&btusb_driver);
}

static void __exit btusb_exit(void)
{
	RTKBT_DBG("rtk_btusb: btusb_exit");
	usb_deregister(&btusb_driver);

#ifdef BTCOEX
	rtk_btcoex_exit();
#endif
}

module_init(btusb_init);
module_exit(btusb_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("Realtek Bluetooth USB driver ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
