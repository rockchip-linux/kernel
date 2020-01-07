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
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/errno.h>
#include <linux/usb.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>

#include <linux/version.h>
#include <linux/pm_runtime.h>
#include <linux/firmware.h>
#include <linux/suspend.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/hci.h>

/* #define HCI_VERSION_CODE KERNEL_VERSION(3, 14, 41) */
#define HCI_VERSION_CODE LINUX_VERSION_CODE

#define BTCOEX

/***********************************
** Realtek - For rtk_btusb driver **
***********************************/
#define BTUSB_WAKEUP_HOST		0	/* 1  enable; 0  disable */

#define URB_CANCELING_DELAY_MS	10	// Added by Realtek
#if HCI_VERSION_CODE > KERNEL_VERSION(2, 6, 33)
#define HDEV_BUS		hdev->bus
#else
#define HDEV_BUS		hdev->type
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 33)
#define USB_RPM			1
#else
#define USB_RPM			0
#endif

#if HCI_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
#define NUM_REASSEMBLY 3
#endif

#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
#define GET_DRV_DATA(x)		hci_get_drvdata(x)
#else
#define GET_DRV_DATA(x)		x->driver_data
#endif

#if HCI_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
#define SCO_NUM    hdev->conn_hash.sco_num
#else
#define SCO_NUM     hci_conn_num(hdev, SCO_LINK)
#endif

int patch_add(struct usb_interface *intf);
void patch_remove(struct usb_interface *intf);
int download_patch(struct usb_interface *intf);
int set_btoff(struct usb_interface *intf);
void print_event(struct sk_buff *skb);
void print_command(struct sk_buff *skb);
void print_acl(struct sk_buff *skb, int dataOut);

#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
int btusb_send_frame(struct hci_dev *hdev, struct sk_buff *skb);
#else
int btusb_send_frame(struct sk_buff *skb);
#endif

#define BTUSB_MAX_ISOC_FRAMES	10
#define BTUSB_INTR_RUNNING		0
#define BTUSB_BULK_RUNNING		1
#define BTUSB_ISOC_RUNNING		2
#define BTUSB_SUSPENDING		3
#define BTUSB_DID_ISO_RESUME	4

struct btusb_data {
	struct hci_dev *hdev;
	struct usb_device *udev;
	struct usb_interface *intf;
	struct usb_interface *isoc;

	spinlock_t lock;

	unsigned long flags;

	struct work_struct work;
	struct work_struct waker;

	struct usb_anchor tx_anchor;
	struct usb_anchor intr_anchor;
	struct usb_anchor bulk_anchor;
	struct usb_anchor isoc_anchor;
	struct usb_anchor deferred;
	int tx_in_flight;
	spinlock_t txlock;

#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	spinlock_t rxlock;
	struct sk_buff *evt_skb;
	struct sk_buff *acl_skb;
	struct sk_buff *sco_skb;
#endif

	struct usb_endpoint_descriptor *intr_ep;
	struct usb_endpoint_descriptor *bulk_tx_ep;
	struct usb_endpoint_descriptor *bulk_rx_ep;
	struct usb_endpoint_descriptor *isoc_tx_ep;
	struct usb_endpoint_descriptor *isoc_rx_ep;

	__u8 cmdreq_type;

	unsigned int sco_num;
	int isoc_altsetting;
	int suspend_count;

#if HCI_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	int (*recv_bulk) (struct btusb_data * data, void *buffer, int count);
#endif
	struct notifier_block pm_notifier;
	void *context;
};
