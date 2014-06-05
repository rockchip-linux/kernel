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

#define ORIGIN_LOWER_LEFT 0x1
#define ORIGIN_LOWER_RIGHT 0x2
#define ORIGIN_UPPER_LEFT 0x3
#define ORIGIN_UPPER_RIGHT 0x4
#define ORIGIN_IS_HIGH(origin) ((origin - 1) & 2)
#define ORIGIN_IS_RIGHT(origin) ((origin - 1) & 1)

/* Converts a value in DPI to dots-per-millimeter */
#define DPI_TO_DPMM(dpi) (((dpi) * 5) / 127)

#define SLOT_COUNT 16

#define CONTACT_STATUS_RELEASED 0
#define CONTACT_STATUS_TOUCH 1
#define CONTACT_STATUS_HOVER 2
#define CONTACT_STATUS_RESERVED 3

#define BUTTON_LEFT 0
#define BUTTON_RIGHT 1
#define BUTTON_MIDDLE 2
/* T400 reports 2 different buttons for middle clicks,
	but we report both as BTN_MIDDLE input_event */
#define BUTTON_MIDDLE_ALT 3

#define BUTTON_LEFT_MASK (1 << BUTTON_LEFT)
#define BUTTON_RIGHT_MASK (1 << BUTTON_RIGHT)
#define BUTTON_MIDDLE_MASK (1 << BUTTON_MIDDLE)
#define BUTTON_MIDDLE_ALT_MASK (1 << BUTTON_MIDDLE_ALT)

#define UNIFYING_KBD_INPUT_REPORT_SIZE 8

#define ARRAYSIZE(array) (sizeof(array) / sizeof(*(array)))

/* Supported Devices */
static const struct hid_device_id wtp_devices[] = {
	{HID_DJ_DEVICE(USB_VENDOR_ID_LOGITECH, UNIFYING_DEVICE_ID_WIRELESS_TOUCHPAD) },
	{HID_DJ_DEVICE(USB_VENDOR_ID_LOGITECH, UNIFYING_DEVICE_ID_WIRELESS_TOUCHPAD_T650) },
	{HID_DJ_DEVICE(USB_VENDOR_ID_LOGITECH, UNIFYING_DEVICE_ID_ALLINONE_KBD_TK820) },
	{HID_DJ_DEVICE(USB_VENDOR_ID_LOGITECH, UNIFYING_DEVICE_ID_ZONE_MOUSE_T400) },
	{HID_DJ_DEVICE(USB_VENDOR_ID_LOGITECH, UNIFYING_DEVICE_ID_TOUCH_MOUSE_T620) },
	{HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_LOGITECH, USB_DEVICE_ID_WIRELESS_TOUCHPAD_T651) },
	{HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_LOGITECH, USB_DEVICE_ID_BLUETOOTH_TOUCHMOUSE) },
	{ }
};
MODULE_DEVICE_TABLE(hid, wtp_devices);


/* This struct represents a finger including location, pressure and
	its status */
struct wtp_event_finger {
	u8 status;
	u16 abs_x;
	u16 abs_y;
	u8 pressure;
	u8 id;
};

/* This struct represents a touch data message from a touchpad  */
struct wtp_event {
	struct wtp_event_finger fingers[4];

	/* relative mouse movement */
	s16 rel_x;
	s16 rel_y;

	/* bitmask of button states. 1=down, 0=up. */
	u8 buttons;

	/* stores an incoming keyboard report to transmit to hid_input */
	u8 kbd_report[UNIFYING_KBD_INPUT_REPORT_SIZE];

	/* bitmask of buttons included within this event */
	u8 has_buttons;

	/* true if this event includes finger data */
	bool has_abs:1;

	/* true if this event includes mouse movement data */
	bool has_rel:1;

	/* true if this event includes keyboard input */
	bool has_keys:1;

	/* false if there are events following with more data,
		true if this is the last one */
	bool end_of_frame:1;
};

/* Struct describing the touch devices axis properties */
struct wtp_device_info {
	u16 abs_max_x;
	u16 abs_res_x;

	u16 abs_max_y;
	u16 abs_res_y;

	bool has_rel:1;

	u8 origin;

	u16 abs_min_pressure;
	u16 abs_max_pressure;

	u8 max_contacts;
};

struct wtp_data;

/* Struct storing feature-specific information. Each feature_id
	has methods assigned that process messages from this feature. */
struct wtp_feature {
	u16 id;
	u8 index;
	u8 event_format;
	int (*init)(struct hidpp_device *);
	int (*probe)(struct hidpp_device *, struct wtp_device_info *);
	int (*parse_feature_event)(struct wtp_data *, struct hidpp_report *,
			struct wtp_event *);
	int (*parse_other_event)(struct wtp_data *, struct hidpp_report *,
			struct wtp_event *);
};

/* Structure containing device data */
struct wtp_data {
	struct input_dev *input;

	struct wtp_device_info info;

	/* the touch feature supported by this device */
	struct wtp_feature feature;

	/* keep track of which buttons are down */
	u8 buttons;

	/* For assigning tracking IDs for MT-B protocol. */
	u16 next_tracking_id;
	u16 current_slots_used;  /* slots = device IDs. Bitmask. */
	u16 prev_slots_used;  /* slots = device IDs. Bitmask. */

	u8 fingers_seen_this_frame;
};

/* Bit Operations Helper */
static u16 make_u16(u8 high, u8 low)
{
	return (high << 8) | low;
}
static u8 low_nib(u8 val)
{
	return val & 0xf;
}
static u8 high_nib(u8 val)
{
	return val >> 4;
}
static bool get_bit(u8 mask, u8 idx)
{
	return mask & (1 << idx);
}
static u16 make_u12_4_8(u8 high, u8 low)
{
	return make_u16(high, low) & 0x0fff;
}
static u16 make_u12_8_4(u8 high, u8 low)
{
	return make_u16(high, low << 4) >> 4;
}
static s16 u12_to_s12(u16 val)
{
	return ((s16)(val << 4)) >> 4;
}
/*
	Helper methods for parsing mouse events. Some devices
	use mouse events to report buttons.
*/

#define GENERIC_EVENT_MOUSE 0x02
#define GENERIC_EVENT_KEYBOARD 0x01

static void generic_parse_mouse_button(u8 raw,
		struct wtp_event *event) {
	event->has_buttons = BUTTON_LEFT_MASK | BUTTON_RIGHT_MASK |
		BUTTON_MIDDLE_MASK;
	event->buttons = raw & event->has_buttons;
}

static void generic_parse_mouse_rel(u8 *raw,
		struct wtp_event *event) {
	event->rel_x = u12_to_s12(make_u12_4_8(low_nib(raw[1]), raw[0]));
	event->rel_y = u12_to_s12(make_u12_8_4(raw[2], high_nib(raw[1])));
	event->has_rel = true;
}

static void generic_parse_kbd_keys(u8 *raw,
	     struct wtp_event *event) {
	dbg_hid("%s\n", __func__);
	memcpy(event->kbd_report, raw, UNIFYING_KBD_INPUT_REPORT_SIZE);
	event->has_keys = true;
}

/*
	TouchPadRawXY (TPRXY) Feature
*/

#define TPRXY_FEATURE 0x6100
#define TPRXY_CMD_GET_TOUCHPAD_INFO (0x00 | SOFTWARE_ID)
#define TPRXY_CMD_GET_RAW_REPORT_STATE (0x10 | SOFTWARE_ID)
#define TPRXY_CMD_SET_RAW_REPORT_STATE (0x20 | SOFTWARE_ID)
#define TPRXY_EVENT_TOUCHPAD_RAW_TOUCH_POINTS 0x00

#define TPRXY_FORMAT_RAW 0x01
#define TPRXY_FORMAT_RAW_SEPARATE_BUTTON_REPORT 0x02
#define TPRXY_FORMAT_MOUSE_EXTENDED 0x03

#define TPRXY_DEFAULT_RES 1000  /* DPI */

#define TPRXY_SLOTS_PER_FRAME 2

/* Initialize TouchPadRawXY feature:
	Set touchpad into raw mode (except for T651, which allows for raw touch
	data appended to mouse events). */
static int tprxy_init(struct hidpp_device *hidpp_dev)
{
	struct wtp_data *fd = hidpp_dev->driver_data;
	struct hidpp_report response;
	int ret;
	u8 params;

	dbg_hid("%s\n", __func__);

	if (hidpp_dev->hid_dev->product ==
			USB_DEVICE_ID_WIRELESS_TOUCHPAD_T651) {
		params = 0x4; /* enhanced sensitivity */
		fd->feature.event_format = TPRXY_FORMAT_MOUSE_EXTENDED;
	} else {
		params = 0x5; /* enhanced sensitivity + raw */
		fd->feature.event_format = TPRXY_FORMAT_RAW;
		if (hidpp_dev->hid_dev->product == UNIFYING_DEVICE_ID_WIRELESS_TOUCHPAD)
			fd->feature.event_format = TPRXY_FORMAT_RAW_SEPARATE_BUTTON_REPORT;
	}

	ret = hidpp_send_fap_command_sync(hidpp_dev, fd->feature.index,
		TPRXY_CMD_SET_RAW_REPORT_STATE, &params, 1, &response);

	return -ret;
}

/* Probe TouchPadRawXY feature */
static int tprxy_probe(struct hidpp_device *hidpp_dev,
		struct wtp_device_info *info)
{
	struct wtp_data *fd = hidpp_dev->driver_data;
	struct hidpp_report response;
	int ret;
	u16 res;
	u8 *params = (u8 *)response.fap.params;

	dbg_hid("%s\n", __func__);

	ret = hidpp_send_fap_command_sync(hidpp_dev, fd->feature.index,
			TPRXY_CMD_GET_TOUCHPAD_INFO, NULL, 0, &response);

	if (ret)
		return -ret;

	info->abs_max_x = make_u16(params[0], params[1]);
	info->abs_max_y = make_u16(params[2], params[3]);
	info->abs_max_pressure = params[5];
	info->max_contacts = params[7];
	info->origin = params[8];

	res = make_u16(params[13], params[14]);
	if (!res)
		res = TPRXY_DEFAULT_RES;
	info->abs_res_x = res;
	info->abs_res_y = res;
	return ret;
}

/* Parse other events while using TouchPadRawXY feature:
	Mouse events might still be received in the following cases:
	- Touchpad with separate buttons send mouse events on click
	- Touchpad using extended mouse events send touch data as
		mouse events */
static int tprxy_parse_other_event(struct wtp_data *wtp,
		struct hidpp_report *report,
		struct wtp_event *event) {
	int i;
	u8 *raw = (u8 *)report;
	u8 *buf = &report->rap.params[0];

	dbg_hid("%s:report_id:%x\n", __func__,report->report_id);

	if (report->report_id == GENERIC_EVENT_KEYBOARD) {
		generic_parse_kbd_keys(&raw[0], event);
		return 0;
	}

	if (report->report_id != GENERIC_EVENT_MOUSE)
		return -1;

	if (wtp->feature.event_format != TPRXY_FORMAT_RAW)
		generic_parse_mouse_button(raw[1], event);

	if (wtp->feature.event_format != TPRXY_FORMAT_MOUSE_EXTENDED)
		return 0;

	for (i = 0; i < TPRXY_SLOTS_PER_FRAME; ++i) {
		struct wtp_event_finger *finger = &event->fingers[i];
		u8 *raw = buf + (5 + i * 6);
		u8 width = low_nib(raw[5]);
		u8 height = high_nib(raw[5]);

		finger->pressure = (width * width + height * height) / 2;
		finger->status = finger->pressure > 0;
		finger->abs_x = make_u16(raw[2], raw[1]);
		finger->abs_y = make_u16(raw[4], raw[3]);
		finger->id = raw[0];
	}
	event->end_of_frame = !get_bit(buf[3], 7);
	event->has_abs = true;

	return 0;
}

/* Parse TouchPadRawXY events */
static int tprxy_parse_feature_event(struct wtp_data *wtp,
		struct hidpp_report *report,
		struct wtp_event *event) {
	int i;
	u8 *buf = &report->rap.params[0];
	u8 fingers_this_frame;

	dbg_hid("%s\n", __func__);

	if (wtp->feature.event_format == TPRXY_FORMAT_MOUSE_EXTENDED)
		return -1;

	for (i = 0; i < TPRXY_SLOTS_PER_FRAME; ++i) {
		u8 *raw = buf + (2 + i * 7);
		event->fingers[i].status = get_bit(raw[2], 6);
		event->fingers[i].abs_x = make_u16(raw[0] & 0x3f, raw[1]);
		event->fingers[i].abs_y = make_u16(raw[2] & 0x3f, raw[3]);
		event->fingers[i].pressure = raw[5];
		event->fingers[i].id = high_nib(raw[6]);
	}
	event->buttons = get_bit(buf[8], 2);
	event->end_of_frame = get_bit(buf[8], 0);

	/* For single event frames, the end of frame flag is implied. */
	fingers_this_frame = low_nib(buf[15]);
	if (fingers_this_frame <= TPRXY_SLOTS_PER_FRAME)
		event->end_of_frame = true;

	event->has_abs = true;
	if (wtp->feature.event_format == TPRXY_FORMAT_RAW)
		event->has_buttons = 0x1;
	return 0;
}

/*
	TouchMouseRawTouchPoints (TMRTP) Feature
*/

#define TMRTP_FEATURE 0x6110
#define TMRTP_CMD_GET_TOUCHPAD_INFO (0x00 | SOFTWARE_ID)
#define TMRTP_CMD_GET_RAW_MODE (0x10 | SOFTWARE_ID)
#define TMRTP_CMD_SET_RAW_MODE (0x20 | SOFTWARE_ID)
#define TMRTP_EVENT_TOUCH_MOUSE_RAW_TOUCH_POINTS 0x00
#define TMRTP_EVENT_STATUS_CHANGED 0x01

static int tmrtp_init(struct hidpp_device *hidpp_dev)
{
	struct wtp_data *fd = hidpp_dev->driver_data;
	struct hidpp_report response;
	int ret;

	u8 params = 2; /* raw data NOT filtered */

	dbg_hid("%s\n", __func__);

	ret = hidpp_send_fap_command_sync(hidpp_dev, fd->feature.index,
		TMRTP_CMD_SET_RAW_MODE, &params, 1, &response);

	return -ret;
}

static int tmrtp_probe(struct hidpp_device *hidpp_dev,
		struct wtp_device_info *info)
{
	struct wtp_data *fd = hidpp_dev->driver_data;
	struct hidpp_report response;
	int ret;
	u16 res;
	u8 *params = (u8 *)response.fap.params;

	dbg_hid("%s\n", __func__);

	ret = hidpp_send_fap_command_sync(hidpp_dev, fd->feature.index,
			TMRTP_CMD_GET_TOUCHPAD_INFO, NULL, 0, &response);

	if (ret)
		return -ret;

	info->abs_max_x = make_u16(params[0], params[1]);
	info->abs_max_y = make_u16(params[2], params[3]);
	info->abs_max_pressure = params[8];
	info->max_contacts = params[7];
	info->origin = params[6];

	res = make_u16(params[4], params[5]);
	info->abs_res_x = res;
	info->abs_res_y = res;
	info->has_rel = true;
	return ret;
}

static int tmrtp_parse_other_event(struct wtp_data *wtp,
		struct hidpp_report *report,
		struct wtp_event *event) {
	u8 *raw = (u8 *)report;
	if (report->report_id == GENERIC_EVENT_MOUSE) {
		generic_parse_mouse_rel(&raw[3], event);
		generic_parse_mouse_button(raw[1], event);
	} else if (report->report_id == GENERIC_EVENT_KEYBOARD) {
		/* In some cases T400 sends keyboard events for middle buttons */
		event->has_buttons = BUTTON_MIDDLE_ALT_MASK;
		event->buttons = raw[1] & BUTTON_MIDDLE_ALT_MASK;
	} else {
		return -1;
	}
	return 0;
}

static int tmrtp_parse_feature_event(struct wtp_data *wtp,
		struct hidpp_report *report,
		struct wtp_event *event) {
	int i;
	u8 *buf = &report->rap.params[0];

	dbg_hid("%s\n", __func__);

	if (high_nib(report->fap.funcindex_clientid) !=
			TMRTP_EVENT_TOUCH_MOUSE_RAW_TOUCH_POINTS)
		return 0;

	for (i = 0; i < ARRAYSIZE(event->fingers); ++i) {
		u8 *raw = buf + i * 4;
		event->fingers[i].status = (raw[3] != 0xff);
		event->fingers[i].abs_x =
			make_u12_8_4(raw[0], low_nib(raw[2]));
		event->fingers[i].abs_y =
			make_u12_8_4(raw[1], high_nib(raw[2]));
		event->fingers[i].pressure = raw[3];
		event->fingers[i].id = i + 1;
	}
	event->has_abs = true;
	event->end_of_frame = true;
	return 0;
}


/*
	BTTouchMouseSettings (BTTMS) Feature
*/

#define BTTMS_FEATURE 0x6120
#define BTTMS_CMD_GET_TOUCHPAD_INFO (0x00 | SOFTWARE_ID)
#define BTTMS_CMD_GET_RAW_MODE (0x10 | SOFTWARE_ID)
#define BTTMS_CMD_SET_RAW_MODE (0x20 | SOFTWARE_ID)

#define BTTMS_FORMAT_01 0x01
#define BTTMS_NUM_SLOTS 6

static int bttms_init(struct hidpp_device *hidpp_dev)
{
	/* We do not use RAW mode in this feature */
	return 0;
}

static int bttms_probe(struct hidpp_device *hidpp_dev,
		struct wtp_device_info *info)
{
	struct wtp_data *fd = hidpp_dev->driver_data;
	struct hidpp_report response;
	int ret;
	u16 res;
	u8 *params = (u8 *)response.fap.params;

	dbg_hid("%s\n", __func__);

	ret = hidpp_send_fap_command_sync(hidpp_dev, fd->feature.index,
			BTTMS_CMD_GET_TOUCHPAD_INFO, NULL, 0, &response);

	if (ret)
		return -ret;

	info->abs_max_x = make_u16(params[0], params[1]);
	info->abs_max_y = make_u16(params[2], params[3]);
	info->abs_max_pressure = params[5];
	info->max_contacts = params[7];
	info->origin = params[6];
	fd->feature.event_format = params[9];

	res = make_u16(params[4], params[5]);
	info->abs_res_x = res;
	info->abs_res_y = res;
	info->has_rel = true;
	return ret;
}

static int bttms_parse_other_event(struct wtp_data *wtp,
		struct hidpp_report *report,
		struct wtp_event *event) {
	int id;
	u8 *raw = (u8 *)report;
	u8 map = raw[8];
	int current_finger = 0;

	dbg_hid("%s\n", __func__);

	if (report->report_id != GENERIC_EVENT_MOUSE &&
		wtp->feature.event_format != BTTMS_FORMAT_01)
		return -1;

	generic_parse_mouse_rel(&raw[2], event);
	generic_parse_mouse_button(raw[1], event);

	for (id = 0; id < BTTMS_NUM_SLOTS; ++id) {
		if (get_bit(map, id)) {
			struct wtp_event_finger *finger =
				&event->fingers[current_finger];
			u8 *f_raw = raw + (9 + current_finger * 4);
			u8 width = low_nib(f_raw[3]);
			u8 height = high_nib(f_raw[3]);
			finger->status = CONTACT_STATUS_TOUCH;
			finger->pressure = (width * width +
				height * height) / 2;
			finger->abs_x = make_u12_8_4(f_raw[0],
				low_nib(f_raw[2]));
			finger->abs_y = make_u12_8_4(f_raw[1],
				high_nib(f_raw[2]));
			finger->id = id + 1;
			current_finger++;
		}
	}

	event->has_abs = true;
	event->end_of_frame = !get_bit(raw[7], 7);
	return 0;
}

static int bttms_parse_feature_event(struct wtp_data *wtp,
		struct hidpp_report *report,
		struct wtp_event *event) {
	/* We are not using raw events */
	return -1;
}


/* Array enumerating all supported hidpp features */
static struct wtp_feature wtp_supported_features[] = {
	{
		TPRXY_FEATURE,
		0, 0,
		&tprxy_init,
		&tprxy_probe,
		&tprxy_parse_feature_event,
		&tprxy_parse_other_event
	},
	{
		TMRTP_FEATURE,
		0, 0,
		&tmrtp_init,
		&tmrtp_probe,
		&tmrtp_parse_feature_event,
		&tmrtp_parse_other_event
	},
	{
		BTTMS_FEATURE,
		0, 0,
		&bttms_init,
		&bttms_probe,
		&bttms_parse_feature_event,
		&bttms_parse_other_event
	},
	{ }
};

/*
	Common code for all devices/features
*/

/* Report a single finger as input_events. This method will also assign
	tracking ids to each new finger. */
static void wtp_process_event_finger(struct wtp_data *fd,
	struct wtp_event_finger *finger)
{
	int slot = finger->id - 1;

	bool new_finger = !(fd->prev_slots_used & (1 << slot));
	fd->current_slots_used |= 1 << slot;
	fd->fingers_seen_this_frame++;

	dbg_hid("Finger %d: (%d,%d,%d) s=%d\n",
		slot,
		finger->abs_x,
		finger->abs_y,
		finger->pressure,
		finger->status);

	input_mt_slot(fd->input, slot);
	if (new_finger) {
		input_event(fd->input, EV_ABS, ABS_MT_TRACKING_ID,
				fd->next_tracking_id++);
		if (fd->next_tracking_id == 0xffff)
			fd->next_tracking_id = 1;
	}
	input_mt_report_slot_state(fd->input, MT_TOOL_FINGER, 1);
	input_event(fd->input, EV_ABS, ABS_MT_POSITION_X,
			ORIGIN_IS_RIGHT(fd->info.origin) ?
			fd->info.abs_max_x - finger->abs_x : finger->abs_x);
	input_event(fd->input, EV_ABS, ABS_MT_POSITION_Y,
			ORIGIN_IS_HIGH(fd->info.origin) ?
			finger->abs_y : fd->info.abs_max_y - finger->abs_y);
	input_event(fd->input, EV_ABS, ABS_MT_PRESSURE, finger->pressure);
}

/* Report an event as input_events */
static int wtp_process_event(struct hidpp_device *hidpp_dev,
				struct wtp_event *event)
{
	struct wtp_data *fd = (struct wtp_data *)hidpp_dev->driver_data;

	int i;
	if (!hidpp_dev->initialized)
		return -1;

	/* report buttons */
	if (event->has_buttons != 0) {
		fd->buttons &= ~event->has_buttons;
		fd->buttons |= event->buttons & event->has_buttons;
		dbg_hid("Button: 0x%x\n", fd->buttons);
		input_report_key(fd->input, BTN_LEFT,
			get_bit(fd->buttons, BUTTON_LEFT));
		input_report_key(fd->input, BTN_RIGHT,
			get_bit(fd->buttons, BUTTON_RIGHT));
		input_report_key(fd->input, BTN_MIDDLE,
			get_bit(fd->buttons, BUTTON_MIDDLE) |
			get_bit(fd->buttons, BUTTON_MIDDLE_ALT));
	}

	if (event->has_keys) {
		hid_report_raw_event(hidpp_dev->hid_dev, HID_INPUT_REPORT,
				event->kbd_report, UNIFYING_KBD_INPUT_REPORT_SIZE, 1);
		input_sync(fd->input);
		return 1;
	}


	/* report relative mouse movement */
	if (event->has_rel) {
		dbg_hid("REL: (%d, %d)\n", event->rel_x, event->rel_y);
		input_report_rel(fd->input, REL_X, event->rel_x);
		input_report_rel(fd->input, REL_Y, event->rel_y);
	}

	/* sync now if there is no touch data following */
	if (!event->has_abs) {
		input_sync(fd->input);
		return 1; /* we successfully consumed the event */
	}

	/* update fingers */
	for (i = 0; i < ARRAYSIZE(event->fingers); i++) {
		if (event->fingers[i].status != CONTACT_STATUS_RELEASED)
			wtp_process_event_finger(fd, &event->fingers[i]);
	}

	/* update released fingers and sync */
	if (event->end_of_frame) {
		for (i = 0; i < SLOT_COUNT; i++) {
			__u16 slot_mask = 1 << i;
			bool released = (fd->prev_slots_used & slot_mask) &&
				!(fd->current_slots_used & slot_mask);
			if (!released)
				continue;
			dbg_hid("Finger %d: released\n", i);
			input_mt_slot(fd->input, i);
			input_event(fd->input, EV_ABS, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(fd->input, MT_TOOL_FINGER, 0);
		}
		input_mt_report_pointer_emulation(fd->input, true);

		input_sync(fd->input);

		fd->prev_slots_used = fd->current_slots_used;
		fd->current_slots_used = 0;
		fd->fingers_seen_this_frame = 0;
	}
	return 1; /* we successfully consumed the event */
}

/* dispatches events to feature event parsing methods and reports
	the result as input_events */
static int wtp_hidpp_event_handler(struct hidpp_device *hidpp_device,
		struct hidpp_report *report)
{
	struct wtp_event event;
	struct wtp_data *fd = (struct wtp_data *)hidpp_device->driver_data;
	int ret;

	memset(&event, 0, sizeof(event));

	if (report->report_id == REPORT_ID_HIDPP_LONG &&
			report->rap.sub_id == fd->feature.index) {
		ret = (*fd->feature.parse_feature_event)(fd, report, &event);
	} else {
		ret = (*fd->feature.parse_other_event)(fd, report, &event);
	}
	if (ret)
		return ret;
	return wtp_process_event(hidpp_device, &event);
}

/* report device info */
static int wtp_input_mapping(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	struct hidpp_device *hidpp_dev = hid_get_drvdata(hdev);
	struct wtp_data *fd = (struct wtp_data *)hidpp_dev->driver_data;
	struct input_dev *input = hi->input;
	int res_x_mm, res_y_mm;

	dbg_hid("%s:\n", __func__);

	if ( ((usage->hid & HID_USAGE_PAGE) == HID_UP_KEYBOARD) ||
		((usage->hid & HID_USAGE_PAGE) == HID_UP_LED) )
		return 0;

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
	input_set_abs_params(input, ABS_MT_POSITION_X,
			0, fd->info.abs_max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y,
			0, fd->info.abs_max_y, 0, 0);
	input_set_abs_params(input, ABS_X, 0, fd->info.abs_max_x, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, fd->info.abs_max_y, 0, 0);

	if (fd->info.has_rel) {
		input_set_capability(input, EV_REL, REL_X);
		input_set_capability(input, EV_REL, REL_Y);
	}

	res_x_mm = DPI_TO_DPMM(fd->info.abs_res_x);
	res_y_mm = DPI_TO_DPMM(fd->info.abs_res_y);

	input_abs_set_res(input, ABS_MT_POSITION_X, res_x_mm);
	input_abs_set_res(input, ABS_MT_POSITION_Y, res_y_mm);
	input_abs_set_res(input, ABS_X, res_x_mm);
	input_abs_set_res(input, ABS_Y, res_y_mm);

	return 0;
}

/* probes for all supported features and uses the first available
	to initialize the device */
static int wtp_init_feature(struct hidpp_device *hidpp_device)
{
	struct wtp_feature *feature = wtp_supported_features;
	struct wtp_data *fd = (struct wtp_data *)hidpp_device->driver_data;
	dbg_hid("%s\n", __func__);

	while (feature->id) {
		int ret;
		dbg_hid("Probing feature 0x%x\n", feature->id);
		ret = hidpp_get_hidpp2_feature_index(hidpp_device,
						SOFTWARE_ID,
						feature->id,
						&feature->index);
		if (ret)
			return -ENODEV;

		if (feature->index != 0) {
			dbg_hid("Feature found at index: %d\n", feature->index);
			fd->feature = *feature;
			ret = (*fd->feature.probe)(hidpp_device, &fd->info);
			if (ret)
				return ret;
			ret = (*fd->feature.init)(hidpp_device);
			return ret;
		} else {
			dbg_hid("unavailable feature 0x%x\n", feature->id);
		}
		++feature;
	}
	/* no supported feature found on this device */
	return -ENODEV;
}

static void wtp_connect_change(struct hidpp_device *hidpp_dev, bool connected)
{
	struct wtp_data *fd = (struct wtp_data *)hidpp_dev->driver_data;
	dbg_hid("%s: connected:%d\n", __func__, connected);
	if (connected && hidpp_dev->initialized && fd->feature.init)
		(*fd->feature.init)(hidpp_dev);
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
	ret = hidpp_send_hidpp2_sync(hidpp_device, REPORT_ID_HIDPP_LONG,
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

	ret = wtp_init_feature(hidpp_device);
	if (ret) {
		dbg_hid("wtp_init_feature returned: %d", ret);
		ret = -ENODEV;
		goto failed;
	}

	hid_device_io_stop(hdev);

	hidpp_device->raw_event = wtp_hidpp_event_handler;

	hdev->quirks |= HID_QUIRK_NO_INIT_REPORTS;
	/* Needed because some input reports are split across multiple
	 * incoming packets, and we need the HID system to not call
	 * input_sync() after each incoming packet: */
	hdev->quirks |= HID_QUIRK_NO_INPUT_SYNC;
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
