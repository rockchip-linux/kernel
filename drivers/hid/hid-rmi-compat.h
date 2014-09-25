/*
 *  Copyright (c) 2014 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 *  Copyright (c) 2014 Red Hat, Inc
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

static inline int hid_hw_raw_request(struct hid_device *hdev,
				  unsigned char reportnum, __u8 *buf,
				  size_t len, unsigned char rtype, int reqtype)
{
	if (len < 1 || len > HID_MAX_BUFFER_SIZE || !buf)
		return -EINVAL;

	switch (reqtype) {
	case HID_REQ_SET_REPORT:
		if (reportnum != buf[0])
			return -EINVAL;
		return hdev->hid_output_raw_report(hdev, buf, len, rtype);
	case HID_REQ_GET_REPORT:
		return hdev->hid_get_raw_report(hdev, reportnum, buf, len,
						rtype);
	}

	return -ENOSYS;
}

static inline int hid_hw_output_report(struct hid_device *hdev, __u8 *buf,
					size_t len)
{
	if (len < 1 || len > HID_MAX_BUFFER_SIZE || !buf)
		return -EINVAL;

	return hdev->hid_output_raw_report(hdev, buf, len, HID_OUTPUT_REPORT);
}
