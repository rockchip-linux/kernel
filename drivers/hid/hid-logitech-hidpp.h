#ifndef __HID_LOGITECH_HIDPP_H
#define __HID_LOGITECH_HIDPP_H

/*
 *  HIDPP protocol
 *
 *  Copyright (c) 2011 Logitech (c)
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Should you need to contact me, the author, you can do so by e-mail send
 * your message to Benjamin Tissoires <benjamin.tissoires at gmail com>
 *
 */
#include <linux/kfifo.h>

#define REPORT_ID_HIDPP_SHORT			0x10
#define REPORT_ID_HIDPP_LONG			0x11
#define REPORT_ID_HIDPP_REL			0x20

#define HIDPP_REPORT_SHORT_LENGTH		7
#define HIDPP_REPORT_LONG_LENGTH		20

/* There are two hidpp protocols in use, the first version hidpp10 is known
 * as register access protocol or RAP, the second version hidpp20 is known as
 * feature access protocol or FAP
 *
 * Most older devices (including the Unifying usb receiver) use the RAP protocol
 * where as most newer devices use the FAP protocol. Both protocols are
 * compatible with the underlying transport, which could be usb, Unifiying, or
 * bluetooth. The message lengths are defined by the hid vendor specific report
 * descriptor for the HIDPP_SHORT report type (total message lenth 7 bytes) and
 * the HIDPP_LONG report type (total message length 20 bytes)
 *
 * The RAP protocol uses both report types, whereas the FAP only uses HIDPP_LONG
 * messages. The Unifying receiver itself responds to RAP messages (device index is
 * 0xFF for the receiver), and all messages (short or long) with a device index
 * between 1 and 6 are passed untouched to the corresponding paired Unifying device.
 *
 * The paired device can be RAP or FAP, it will receive the message untouched from
 * the Unifiying receiver.
 */

struct fap {
	u8 feature_index;
	u8 funcindex_clientid;
	u8 params[HIDPP_REPORT_LONG_LENGTH - 4U];
};

struct rap {
	u8 sub_id;
	u8 reg_address;
	u8 params[HIDPP_REPORT_LONG_LENGTH - 4U];
};

struct hidpp_report {
	u8 report_id;
	u8 device_index;
	union {
		struct fap fap;
		struct rap rap;
		u8 rawbytes[sizeof(struct fap)];
	};
} __packed;

struct hidpp_device {
	struct hid_device *hid_dev;
	void *driver_data;

	int (*raw_event)(struct hidpp_device *hidpp_dev, struct hidpp_report *report);
	int (*device_init)(struct hidpp_device *hidpp_dev);
	void (*connect_change)(struct hidpp_device *hidpp_dev, bool connected);

	/* private */
	struct work_struct work;
	struct mutex send_mutex;
	struct kfifo delayed_work_fifo;
	spinlock_t lock;
	void *send_receive_buf;
	wait_queue_head_t wait;
	bool answer_available;
	bool initialized;
	int init_retry;
};

extern int hidpp_raw_event(struct hid_device *hdev, struct hid_report *report,
			u8 *data, int size);

extern void hidpp_connect_change(struct hidpp_device *hidpp_dev, bool connected);
extern int hidpp_init(struct hidpp_device *hidpp_dev, struct hid_device *hid_dev);
extern void hidpp_delayed_init(struct hidpp_device *hidpp_device);
extern void hidpp_remove(struct hidpp_device *hidpp_dev);

extern int hidpp_send_command_sync(struct hidpp_device *hidpp_dev,
			u16 feature, u8 cmd, u8 *params, int param_count,
			struct hidpp_report *response);


extern int hidpp_send_fap_command_sync(struct hidpp_device *hidpp_dev,
		u8 feat_index, u8 funcindex_clientid, u8 *params, int param_count,
		struct hidpp_report *response);

extern int hidpp_send_rap_command_sync(struct hidpp_device *hidpp_dev,
			u8 report_id, u8 sub_id, u8 reg_address, u8 *params,
			int param_count, struct hidpp_report *response);

static inline int hidpp_send_hidpp2_sync(struct hidpp_device *hidpp_dev,
					u8 type,
					u8 feature_index,
					u8 sub_index,
					u8 software_id,
					u8 *params,
					int params_count,
					struct hidpp_report *response)
{
	return hidpp_send_rap_command_sync(hidpp_dev,
					type,
					feature_index,
					(sub_index << 4) | software_id,
					params,
					params_count,
					response);
}

extern int hidpp_get_hidpp2_feature_index(struct hidpp_device *hidpp_dev,
						u8 software_id,
						u16 feature_id,
						u8 *feature_idx);

#define HIDPP_ERROR				0x8f
#define HIDPP_ERROR_SUCCESS			0x00
#define HIDPP_ERROR_INVALID_SUBID		0x01
#define HIDPP_ERROR_INVALID_ADRESS		0x02
#define HIDPP_ERROR_INVALID_VALUE		0x03
#define HIDPP_ERROR_CONNECT_FAIL		0x04
#define HIDPP_ERROR_TOO_MANY_DEVICES		0x05
#define HIDPP_ERROR_ALREADY_EXISTS		0x06
#define HIDPP_ERROR_BUSY			0x07
#define HIDPP_ERROR_UNKNOWN_DEVICE		0x08
#define HIDPP_ERROR_RESOURCE_ERROR		0x09
#define HIDPP_ERROR_REQUEST_UNAVAILABLE		0x0a
#define HIDPP_ERROR_INVALID_PARAM_VALUE		0x0b
#define HIDPP_ERROR_WRONG_PIN_CODE		0x0c

#define HIDPP_TYPE_KEYBOARD			0x00
#define HIDPP_TYPE_REMOTE_CONTROL		0x01
#define HIDPP_TYPE_NUMPAD			0x02
#define HIDPP_TYPE_MOUSE			0x03
#define HIDPP_TYPE_TOUCHPAD			0x04
#define HIDPP_TYPE_TRACKBALL			0x05
#define HIDPP_TYPE_PRESENTER			0x06
#define HIDPP_TYPE_RECEIVER			0x07

#define T651_REPORT_TYPE_MOUSE			0x02

#endif
