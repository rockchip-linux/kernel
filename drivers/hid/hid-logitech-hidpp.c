/*
 *  HIDPP protocol for Logitech Unifying receivers
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
#include <linux/sched.h>
#include "hid-ids.h"
#include "hid-logitech-hidpp.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_AUTHOR("Nestor Lopez Casado <nlopezcasad@logitech.com>");

#define MAX_INIT_RETRY 5

enum delayed_work_type {
	HIDPP_INIT = 0
};

static void hidpp_print_raw_event(const char *header, u8 *data, int size)
{
	int i;
	unsigned char log[96];
	unsigned char tmpstr[60];

	snprintf(log, sizeof(tmpstr), "%s (size=%d)", header, size);

	for (i = 0; i < size; i++) {
		snprintf(tmpstr, sizeof(tmpstr), " %02x", data[i]);
		strlcat(log, tmpstr, sizeof(log));
	}

	dbg_hid("%s\n", log);
}

static int __hidpp_send_report(struct hid_device *hdev,
				struct hidpp_report *hidpp_rept)
{
	int sent_bytes;

	if (!hdev->hid_output_raw_report) {
		dev_err(&hdev->dev, "%s:"
			"hid_output_raw_report is null\n", __func__);
		return -ENODEV;
	}

	hidpp_print_raw_event("sending ", (u8 *)hidpp_rept,
				HIDPP_REPORT_LONG_LENGTH);
	sent_bytes = hdev->hid_output_raw_report(hdev, (u8 *) hidpp_rept,
						 sizeof(struct hidpp_report),
						 HID_OUTPUT_REPORT);

	/* It seems that sending via bluetooth can return -EIO even
	 * when the message is delivered, so we have this hack: */
	return (sent_bytes < 0 && sent_bytes != -EIO) ? sent_bytes : 0;
}

static int hidpp_send_message_sync(struct hidpp_device *hidpp_dev,
	struct hidpp_report *message,
	struct hidpp_report *response)
{
	int ret;

	mutex_lock(&hidpp_dev->send_mutex);

	hidpp_dev->send_receive_buf = response;
	hidpp_dev->answer_available = false;

	/* So that we can later validate the answer when it arrives
	 * in hidpp_raw_event */
	*response = *message;

	ret = __hidpp_send_report(hidpp_dev->hid_dev, message);

	if (ret) {
          dbg_hid("__hidpp_send_report returned err: %d\n", ret);
		memset(response, 0, sizeof(struct hidpp_report));
		goto exit;
	}

	if (!wait_event_timeout(hidpp_dev->wait, hidpp_dev->answer_available, 10*HZ)) {
		dbg_hid("%s:timeout waiting for response\n", __func__);
		memset(response, 0, sizeof(struct hidpp_report));
		ret = -1;
	}

	if (response->report_id == REPORT_ID_HIDPP_SHORT &&
	    response->fap.feature_index == HIDPP_ERROR) {
		ret = response->fap.params[0];
          dbg_hid("__hidpp_send_report got hidpp error %d\n", ret);
		goto exit;
	}

exit:
	mutex_unlock(&hidpp_dev->send_mutex);
	return ret;

}

int hidpp_send_fap_command_sync(struct hidpp_device *hidpp_dev,
	u8 feat_index, u8 funcindex_clientid, u8 *params, int param_count,
	struct hidpp_report *response)
{
	struct hidpp_report message;

	if (param_count > sizeof(message.rap.params))
		return -EINVAL;

	memset(&message, 0, sizeof(message));
	message.report_id = REPORT_ID_HIDPP_LONG;
	message.fap.feature_index = feat_index;
	message.fap.funcindex_clientid = funcindex_clientid;
	memcpy(&message.fap.params, params, param_count);

	return hidpp_send_message_sync(hidpp_dev, &message, response);
}
EXPORT_SYMBOL_GPL(hidpp_send_fap_command_sync);

int hidpp_send_rap_command_sync(struct hidpp_device *hidpp_dev,
	u8 report_id, u8 sub_id, u8 reg_address, u8 *params,
	int param_count,
	struct hidpp_report *response)
{
	struct hidpp_report message;

	if ((report_id != REPORT_ID_HIDPP_SHORT) &&
	    (report_id != REPORT_ID_HIDPP_LONG))
		return -EINVAL;

	if (param_count > sizeof(message.rap.params))
		return -EINVAL;

	memset(&message, 0, sizeof(message));
	message.report_id = report_id;
	/* If sending to a non-DJ device, device expects 0xff. If sending to
	 * a DJ device, this device_index will be overwritten by the DJ code: */
	message.device_index = 0xff;
	message.rap.sub_id = sub_id;
	message.rap.reg_address = reg_address;
	memcpy(&message.rap.params, params, param_count);

	return hidpp_send_message_sync(hidpp_dev, &message, response);
}
EXPORT_SYMBOL_GPL(hidpp_send_rap_command_sync);

int hidpp_get_hidpp2_feature_index(struct hidpp_device *hidpp_dev,
					u8 software_id,
					u16 feature_id,
					u8 *feature_idx)
{
	struct hidpp_report response;
	u8 params[2];
	int ret;

	params[0] = feature_id >> 8;
	params[1] = feature_id & 0xff;
	ret = hidpp_send_hidpp2_sync(hidpp_dev,
					REPORT_ID_HIDPP_LONG,
					0,
					0,
					software_id,
					params,
					sizeof(params),
					&response);
	if (ret)
		return ret;
	*feature_idx = response.rap.params[0];
	return ret;
}
EXPORT_SYMBOL_GPL(hidpp_get_hidpp2_feature_index);

static void schedule_delayed_hidpp_init(struct hidpp_device *hidpp_dev)
{
	enum delayed_work_type work_type = HIDPP_INIT;

	kfifo_in(&hidpp_dev->delayed_work_fifo, &work_type,
				sizeof(enum delayed_work_type));

	if (schedule_work(&hidpp_dev->work) == 0) {
		dbg_hid("%s: did not schedule the work item,"
			" was already queued\n",
			__func__);
	}
}

void hidpp_delayed_init(struct hidpp_device *hidpp_device)
{
	struct hid_device *hdev = hidpp_device->hid_dev;
	int ret = 0;

	dbg_hid("%s: hdev:%p\n", __func__, hdev);

	if (hidpp_device->initialized)
		return;

	if (down_trylock(&hidpp_device->hid_dev->driver_lock)) {
		if (hidpp_device->init_retry < MAX_INIT_RETRY) {
			dbg_hid("%s: we need to reschedule the work item."
				"Semaphore still held on device\n", __func__);
			schedule_delayed_hidpp_init(hidpp_device);
			hidpp_device->init_retry++;
		} else {
			dbg_hid("%s: giving up initialization now.", __func__);
			hidpp_device->init_retry = 0;
		}
		return;
	}
	up(&hidpp_device->hid_dev->driver_lock);

	if (hidpp_device->device_init)
		ret = hidpp_device->device_init(hidpp_device);

	if (!ret)
		hidpp_device->initialized = true;
}
EXPORT_SYMBOL_GPL(hidpp_delayed_init);

static void delayed_work_cb(struct work_struct *work)
{
	struct hidpp_device *hidpp_device =
		container_of(work, struct hidpp_device, work);
	unsigned long flags;
	int count;
	enum delayed_work_type work_type;

	dbg_hid("%s\n", __func__);

	spin_lock_irqsave(&hidpp_device->lock, flags);

	count = kfifo_out(&hidpp_device->delayed_work_fifo, &work_type,
				sizeof(enum delayed_work_type));

	if (count != sizeof(enum delayed_work_type)) {
		dev_err(&hidpp_device->hid_dev->dev, "%s: workitem triggered without "
			"notifications available\n", __func__);
		spin_unlock_irqrestore(&hidpp_device->lock, flags);
		return;
	}

	if (!kfifo_is_empty(&hidpp_device->delayed_work_fifo)) {
		if (schedule_work(&hidpp_device->work) == 0) {
			dbg_hid("%s: did not schedule the work item, was "
				"already queued\n", __func__);
		}
	}

	spin_unlock_irqrestore(&hidpp_device->lock, flags);

	switch (work_type) {
	case HIDPP_INIT:
		hidpp_delayed_init(hidpp_device);
		break;
	default:
		dbg_hid("%s: unexpected report type\n", __func__);
	}
}

int hidpp_init(struct hidpp_device *hidpp_dev, struct hid_device *hid_dev)
{
	struct hid_report *report;

	if (hidpp_dev->initialized)
		return 0;

	hidpp_dev->init_retry = 0;
	hidpp_dev->hid_dev = hid_dev;
	hidpp_dev->initialized = 1;

	INIT_WORK(&hidpp_dev->work, delayed_work_cb);
	mutex_init(&hidpp_dev->send_mutex);
	init_waitqueue_head(&hidpp_dev->wait);

	spin_lock_init(&hidpp_dev->lock);
	if (kfifo_alloc(&hidpp_dev->delayed_work_fifo,
			4 * sizeof(struct hidpp_report),
			GFP_KERNEL)) {
		dev_err(&hidpp_dev->hid_dev->dev,
			"%s:failed allocating delayed_work_fifo\n", __func__);
		mutex_destroy(&hidpp_dev->send_mutex);
		return -ENOMEM;
	}

	/* register SHORT/LONG reports for hid++ feature/register access */
	report = hid_register_report(hid_dev, HID_INPUT_REPORT,
		REPORT_ID_HIDPP_SHORT);
	report->size = HIDPP_REPORT_SHORT_LENGTH;
	report = hid_register_report(hid_dev, HID_INPUT_REPORT,
		REPORT_ID_HIDPP_LONG);
	report->size = HIDPP_REPORT_LONG_LENGTH;

	return 0;
}
EXPORT_SYMBOL_GPL(hidpp_init);

void hidpp_connect_change(struct hidpp_device *hidpp_dev, bool connected)
{
	if ((!hidpp_dev->initialized) && (connected))
		hidpp_delayed_init(hidpp_dev);

	if (hidpp_dev->connect_change)
		hidpp_dev->connect_change(hidpp_dev, connected);
}
EXPORT_SYMBOL_GPL(hidpp_connect_change);

void hidpp_remove(struct hidpp_device *hidpp_dev)
{
	dbg_hid("%s\n", __func__);
	cancel_work_sync(&hidpp_dev->work);
	mutex_destroy(&hidpp_dev->send_mutex);
	kfifo_free(&hidpp_dev->delayed_work_fifo);
	hidpp_dev->initialized = false;
	hidpp_dev->hid_dev = NULL;
}
EXPORT_SYMBOL_GPL(hidpp_remove);

int hidpp_raw_event(struct hid_device *hdev, struct hid_report *hid_report,
			u8 *data, int size)
{
	struct hidpp_device *hidpp_dev = hid_get_drvdata(hdev);
	struct hidpp_report *report = (struct hidpp_report *)data;
	struct hidpp_report *question = hidpp_dev->send_receive_buf;
	struct hidpp_report *answer = hidpp_dev->send_receive_buf;

	dbg_hid("%s\n", __func__);

	hidpp_print_raw_event("hidpp_raw_event", data, size);

	/* If the mutex is locked then we have a pending answer from a
	 * previoulsly sent command
	 */
	if (unlikely(mutex_is_locked(&hidpp_dev->send_mutex))) {
		/* Check for a correct hidpp20 answer */
		bool correct_answer =
			report->fap.feature_index == question->fap.feature_index &&
		report->fap.funcindex_clientid == question->fap.funcindex_clientid;
		dbg_hid("%s mutex is locked, waiting for reply\n", __func__);

		/* Check for a "correct" hidpp10 error message, this means the
		 * device is hidpp10 and does not support the command sent */
		correct_answer = correct_answer ||
			(report->fap.feature_index == HIDPP_ERROR &&
		report->fap.funcindex_clientid == question->fap.feature_index &&
		report->fap.params[0] == question->fap.funcindex_clientid);

		if (correct_answer) {
			hidpp_print_raw_event("answer", data, size);
			*answer = *report;
			hidpp_dev->answer_available = true;
			wake_up(&hidpp_dev->wait);
			/* This was an answer to a command that this driver sent
			 * we return 1 to hid-core to avoid forwarding the command
			 * upstream as it has been treated by the driver */

			return 1;
		}
	}

	if (hidpp_dev->raw_event != NULL) {
		return hidpp_dev->raw_event(hidpp_dev, report);
	}

	hidpp_print_raw_event("event not treated", data, size);

	return 0;
}
EXPORT_SYMBOL_GPL(hidpp_raw_event);
