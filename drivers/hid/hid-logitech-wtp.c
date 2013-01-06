/*
 *  HID driver for Logitech Wireless Touchpad device
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

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input/mt.h>

MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_AUTHOR("Nestor Lopez Casado <nlopezcasad@logitech.com>");
MODULE_DESCRIPTION("Logitech Wireless Touchpad");
MODULE_LICENSE("GPL");

#include "hid-ids.h"
#include "hid-logitech-hidpp.h"

#define SOFTWARE_ID 0xB

#define CMD_TOUCHPAD_GET_RAW_INFO		0x01
#define CMD_TOUCHPAD_GET_RAW_REPORT_STATE	0x11
#define CMD_TOUCHPAD_SET_RAW_REPORT_STATE	0x21
#define WTP_RAW_XY_FEAT_ID			0x6100
#define WTP_RAW_XY_EVENT_INDEX			0

#define ORIGIN_LOWER_LEFT 0x1
#define ORIGIN_LOWER_RIGHT 0x2
#define ORIGIN_UPPER_LEFT 0x3
#define ORIGIN_UPPER_RIGHT 0x4

#define ORIGIN_IS_HIGH(origin) ((origin - 1) & 2)
#define ORIGIN_IS_RIGHT(origin) ((origin - 1) & 1)

/* Converts a value in DPI to dots-per-millimeter */
#define DPI_TO_DPMM(dpi) (((dpi) * 5) / 127)

#define SLOT_COUNT 16

/* The WTP touchpad doesn't supply a resolution, so we hardcode it in
   this driver. */
#define WTP_RES 1000  /* DPI */

#define CONTACT_STATUS_RELEASED 0
#define CONTACT_STATUS_TOUCH 1
#define CONTACT_STATUS_HOVER 2
#define CONTACT_STATUS_RESERVED 3

/* These two structs represent a single raw data input message */
struct hidpp_touchpad_raw_xy_finger {
	u8 contact_type;
	u8 contact_status;
	u16 x;
	u16 y;
	u8 z;
	u8 area;
	u8 finger_id;
};
struct hidpp_touchpad_raw_xy {
	u16 timestamp;
	struct hidpp_touchpad_raw_xy_finger fingers[2];
	u8 fingers_this_frame;
	bool proximity_detection:1;
	bool mechanical_button:1;
	bool spurious_flag:1;
	bool end_of_frame:1;
};

struct wtp_data {
	struct input_dev *input;

	/* Properties of the device. Filled by hidpp_touchpad_get_raw_info() */
	__u16 x_size, y_size;
	__u8 origin;
	__u8 z_range, area_range;
	__u8 maxcontacts;
	__u16 res;  /* points per inch */

	/* Feature index of the raw data feature */
	__u8 mt_feature_index;

	/* For assigning tracking IDs for MT-B protocol. */
	__u16 next_tracking_id;
	__u16 current_slots_used;  /* slots = device IDs. Bitmask. */
	__u16 prev_slots_used;  /* slots = device IDs. Bitmask. */
};

static void wtp_touch_event(struct wtp_data *fd,
	struct hidpp_touchpad_raw_xy_finger *touch_report)
{
	int slot = touch_report->finger_id - 1;

	bool new_finger = !(fd->prev_slots_used & (1 << slot));
	fd->current_slots_used |= 1 << slot;

	input_mt_slot(fd->input, slot);
	if (new_finger) {
		input_event(fd->input, EV_ABS, ABS_MT_TRACKING_ID,
				fd->next_tracking_id++);
		if (fd->next_tracking_id == 0xffff)
			fd->next_tracking_id = 1;
	}
	input_mt_report_slot_state(fd->input, MT_TOOL_FINGER, 1);
	input_event(fd->input, EV_ABS, ABS_MT_POSITION_X,
			ORIGIN_IS_RIGHT(fd->origin) ?
			fd->x_size - touch_report->x : touch_report->x);
	input_event(fd->input, EV_ABS, ABS_MT_POSITION_Y,
			ORIGIN_IS_HIGH(fd->origin) ?
			touch_report->y : fd->y_size - touch_report->y);
	input_event(fd->input, EV_ABS, ABS_MT_PRESSURE, touch_report->area);
}

static int wtp_touchpad_raw_xy_event(struct hidpp_device *hidpp_dev,
				struct hidpp_touchpad_raw_xy *event)
{
	struct wtp_data *fd = (struct wtp_data *)hidpp_dev->driver_data;
	u8 finger_count = event->fingers_this_frame;

	int i;

	if (!hidpp_dev->initialized)
		return 0;

	for (i = 0; i < 2; i++) {
		if (event->fingers[i].contact_status != CONTACT_STATUS_RELEASED)
			wtp_touch_event(fd, &event->fingers[i]);
	}

	if (event->end_of_frame || finger_count <= 2) {
		for (i = 0; i < SLOT_COUNT; i++) {
			__u16 slot_mask = 1 << i;
			bool released = (fd->prev_slots_used & slot_mask) &&
				!(fd->current_slots_used & slot_mask);
			if (!released)
				continue;
			input_mt_slot(fd->input, i);
			input_event(fd->input, EV_ABS, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(fd->input, MT_TOOL_FINGER, 0);
		}
		input_mt_report_pointer_emulation(fd->input, true);
		input_report_key(fd->input, BTN_TOOL_FINGER,
				 finger_count == 1);
		input_report_key(fd->input, BTN_TOOL_DOUBLETAP,
				 finger_count == 2);
		input_report_key(fd->input, BTN_TOOL_TRIPLETAP,
				 finger_count == 3);
		input_report_key(fd->input, BTN_TOOL_QUADTAP,
				 finger_count == 4);
		input_report_key(fd->input, BTN_TOOL_QUINTTAP,
				 finger_count == 5);
		/* WTP Uses normal mouse reports for button state */
		if (hidpp_dev->hid_dev->product !=
			UNIFYING_DEVICE_ID_WIRELESS_TOUCHPAD)
			input_report_key(fd->input, BTN_LEFT,
					 event->mechanical_button);
		input_sync(fd->input);

		fd->prev_slots_used = fd->current_slots_used;
		fd->current_slots_used = 0;
	}
	return 1;
}


static int hidpp_touchpad_raw_xy_event(struct hidpp_device *hidpp_device,
		struct hidpp_report *hidpp_report)
{
	u8 *buf = &hidpp_report->rap.params[0];  /* 4 strips off the DJ header */
	struct wtp_data *fd = (struct wtp_data *)hidpp_device->driver_data;

	/* Parse the message into a more convenient struct */
	struct hidpp_touchpad_raw_xy raw_xy = {
		(buf[0] << 8) | buf[1],  /* Timestamp */
		{ {
			buf[2] >> 6,  /* Contact type */
			buf[4] >> 6,  /* Contact status */
			((buf[2] & 0x3f) << 8) | buf[3],  /* X */
			((buf[4] & 0x3f) << 8) | buf[5],  /* Y */
			buf[6],  /* Z/Force */
			buf[7],  /* Area */
			buf[8] >> 4  /* Finger ID */
		}, {
			buf[9] >> 6,  /* Contact type */
			buf[11] >> 6,  /* Contact status */
			((buf[9] & 0x3f) << 8) | buf[10],  /* X */
			((buf[11] & 0x3f) << 8) | buf[12],  /* Y */
			buf[13],  /* Z/Force */
			buf[14],  /* Area */
			buf[15] >> 4  /* Finger ID */
		} },
		buf[15] & 0xf,  /* Fingers this frame */
		(buf[8] & (1 << 3)) != 0,  /* Proximity detection */
		(buf[8] & (1 << 2)) != 0,  /* Mechanical button */
		(buf[8] & (1 << 1)) != 0,  /* Spurious flag */
		(buf[8] & (1 << 0)) != 0,  /* End-of-frame */
	};

	/* Ensure we get the proper raw data report here. We do this after
	   the parsing above to avoid mixed declarations and code. */
	if (hidpp_report->report_id != REPORT_ID_HIDPP_LONG ||
		hidpp_report->rap.sub_id != fd->mt_feature_index ||
		(hidpp_report->rap.reg_address >> 4) != WTP_RAW_XY_EVENT_INDEX) {
		dbg_hid("Unhandled event type\n");
		return 0;
	}

	dbg_hid("EVT: %d {ty: %d, st: %d, (%d,%d,%d,%d) id:%d} {ty: %d, st: %d, (%d,%d,%d,%d) id:%d} cnt:%d pr:%d but:%d sf:%d eof:%d\n",
		raw_xy.timestamp,
		raw_xy.fingers[0].contact_type,
		raw_xy.fingers[0].contact_status,
		raw_xy.fingers[0].x,
		raw_xy.fingers[0].y,
		raw_xy.fingers[0].z,
		raw_xy.fingers[0].area,
		raw_xy.fingers[0].finger_id,
		raw_xy.fingers[1].contact_type,
		raw_xy.fingers[1].contact_status,
		raw_xy.fingers[1].x,
		raw_xy.fingers[1].y,
		raw_xy.fingers[1].z,
		raw_xy.fingers[1].area,
		raw_xy.fingers[1].finger_id,
		raw_xy.fingers_this_frame,
		raw_xy.proximity_detection,
		raw_xy.mechanical_button,
		raw_xy.spurious_flag,
		raw_xy.end_of_frame);
	return wtp_touchpad_raw_xy_event(hidpp_device, &raw_xy);
}

static int hidpp_touchpad_get_raw_info(struct hidpp_device *hidpp_dev)
{
	struct wtp_data *fd = hidpp_dev->driver_data;
	struct hidpp_report response;
	int ret;
	u8 *params = (u8 *)response.fap.params;

	ret = hidpp_send_fap_command_sync(hidpp_dev, fd->mt_feature_index,
			CMD_TOUCHPAD_GET_RAW_INFO, NULL, 0, &response);

	if (ret)
		return -ret;

	fd->x_size = (params[0] << 8) | params[1];
	fd->y_size = (params[2] << 8) | params[3];
	fd->z_range = params[4];
	fd->area_range = params[5];
	fd->maxcontacts = params[7];
	fd->origin = params[8];
	fd->res = (params[13] << 8) | params[14];
	if (!fd->res)
		fd->res = WTP_RES;

	return ret;
}

static int hidpp_touchpad_set_raw_report_state(struct hidpp_device *hidpp_dev)
{
	struct wtp_data *fd = hidpp_dev->driver_data;
	struct hidpp_report response;
	int ret;

	/* Params:
		0x01 - enable raw
		0x02 - 16bit Z, no area
		0x04 - enhanced sensitivity
		0x08 - width, height instead of area
		0x10 - send raw + gestures (degrades smoothness)
		remaining bits - reserved */
	u8 params = 0x5;

	ret = hidpp_send_fap_command_sync(hidpp_dev, fd->mt_feature_index,
		CMD_TOUCHPAD_SET_RAW_REPORT_STATE, &params, 1, &response);

	if (ret)
		return -ret;

	return ret;
}

static int wtp_input_mapping(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	struct hidpp_device *hidpp_dev = hid_get_drvdata(hdev);
	struct wtp_data *fd = (struct wtp_data *)hidpp_dev->driver_data;
	struct input_dev *input = hi->input;
	int res_mm;

	dbg_hid("%s:\n", __func__);

	if ((usage->hid & HID_USAGE_PAGE) != HID_UP_BUTTON)
		return -1;

	fd->input = hi->input;

	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(BTN_TOOL_FINGER, input->keybit);
	__set_bit(BTN_TOOL_DOUBLETAP, input->keybit);
	__set_bit(BTN_TOOL_TRIPLETAP, input->keybit);
	__set_bit(BTN_TOOL_QUADTAP, input->keybit);
	__set_bit(BTN_TOOL_QUINTTAP, input->keybit);

	__set_bit(EV_ABS, input->evbit);

	input_mt_init_slots(input, SLOT_COUNT, 0);

	input_set_capability(input, EV_KEY, BTN_TOUCH);

	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, fd->x_size, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, fd->y_size, 0, 0);
	input_set_abs_params(input, ABS_X, 0, fd->x_size, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, fd->y_size, 0, 0);

	res_mm = DPI_TO_DPMM(fd->res);

	input_abs_set_res(input, ABS_MT_POSITION_X, res_mm);
	input_abs_set_res(input, ABS_MT_POSITION_Y, res_mm);
	input_abs_set_res(input, ABS_X, res_mm);
	input_abs_set_res(input, ABS_Y, res_mm);

	return 0;
}

static void wtp_connect_change(struct hidpp_device *hidpp_dev, bool connected)
{
	dbg_hid("%s: connected:%d\n", __func__, connected);
	if (connected && hidpp_dev->initialized)
		hidpp_touchpad_set_raw_report_state(hidpp_dev);
}

static int wtp_device_init(struct hidpp_device *hidpp_dev)
{
	int ret;

	dbg_hid("%s\n", __func__);

	ret = hidpp_touchpad_set_raw_report_state(hidpp_dev);

	if (ret) {
		hid_err(hidpp_dev->hid_dev, "unable to set to raw report mode. "
			"The device may not be in range.\n");
		return ret;
	}

	ret = hidpp_touchpad_get_raw_info(hidpp_dev);
	return ret;
}

static int wtp_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct wtp_data *fd = NULL;
	struct hidpp_device *hidpp_device = NULL;
	int ret;
	struct hidpp_report response;

	dbg_hid("%s\n", __func__);

	hidpp_device = kzalloc(sizeof(struct hidpp_device), GFP_KERNEL);
	if (!hidpp_device) {
		hid_err(hdev, "cannot allocate hidpp_device\n");
		ret = -ENOMEM;
		goto hidpp_alloc_failed;
	}

	fd = kzalloc(sizeof(struct wtp_data), GFP_KERNEL);
	if (!fd) {
		hid_err(hdev, "cannot allocate wtp Touch data\n");
		ret = -ENOMEM;
		goto fd_alloc_failed;
	}
	fd->next_tracking_id = 1;

	hidpp_device->driver_data = (void *)fd;
	hid_set_drvdata(hdev, hidpp_device);

	hidpp_device->connect_change = wtp_connect_change;

	ret = hid_parse(hdev);
	if (ret) {
		ret = -ENODEV;
		goto failed;
	}

	ret = hidpp_init(hidpp_device, hdev);
	if (ret) {
		ret = -ENODEV;
		goto failed;
	}

	hid_device_io_start(hdev);

	/* Get hid++ version number */
	ret = hidpp_send_hidpp2_sync(hidpp_device, REPORT_ID_HIDPP_SHORT,
					0, 1,
					SOFTWARE_ID,
					NULL, 0, &response);
	if (ret) {
		dbg_hid("send root cmd returned: %d", ret);
		ret = -ENODEV;
		goto failed;
	}

	dbg_hid("HID++ version: %d.%d\n", response.rap.params[0],
		response.rap.params[1]);

	/* TODO(adlr): Consider requiring a specific/minimum HID++ version. */

	ret = hidpp_get_hidpp2_feature_index(hidpp_device,
						SOFTWARE_ID,
						WTP_RAW_XY_FEAT_ID,
						&fd->mt_feature_index);
	if (ret) {
		dbg_hid("Get raw_xy feature idx failed: %d", ret);
		ret = -ENODEV;
		goto failed;
	}

	wtp_device_init(hidpp_device);

	hid_device_io_stop(hdev);

	hidpp_device->raw_event = hidpp_touchpad_raw_xy_event;

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		ret = -ENODEV;
		goto failed;
	}

	return 0;

failed:
	hid_set_drvdata(hdev, NULL);
	kfree(fd);
fd_alloc_failed:
	kfree(hidpp_device);
hidpp_alloc_failed:
	return ret;
}

static void wtp_remove(struct hid_device *hdev)
{
	struct hidpp_device *hidpp_dev = hid_get_drvdata(hdev);
	struct wtp_data *fd = hidpp_dev->driver_data;
	dbg_hid("%s\n", __func__);
	hid_hw_stop(hdev);
	hidpp_remove(hidpp_dev);
	kfree(fd);
	kfree(hidpp_dev);
	hid_set_drvdata(hdev, NULL);
}

static const struct hid_device_id wtp_devices[] = {
	{HID_DEVICE(BUS_DJ, 0, USB_VENDOR_ID_LOGITECH, UNIFYING_DEVICE_ID_WIRELESS_TOUCHPAD) },
	{HID_DEVICE(BUS_DJ, 0, USB_VENDOR_ID_LOGITECH, UNIFYING_DEVICE_ID_WIRELESS_TOUCHPAD_T650) },
	{ }
};
MODULE_DEVICE_TABLE(hid, wtp_devices);

static struct hid_driver wtp_driver = {
	.name = "wtp-touch",
	.id_table = wtp_devices,
	.probe = wtp_probe,
	.remove = wtp_remove,
	.input_mapping = wtp_input_mapping,
	.raw_event = hidpp_raw_event,
};

static int __init wtp_init(void)
{
	return hid_register_driver(&wtp_driver);
}

static void __exit wtp_exit(void)
{
	hid_unregister_driver(&wtp_driver);
}

module_init(wtp_init);
module_exit(wtp_exit);
