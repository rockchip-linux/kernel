// SPDX-License-Identifier: GPL-2.0+
/*
 *	uvc_gadget.c  --  USB Video Class Gadget driver
 *
 *	Copyright (C) 2009-2010
 *	    Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/g_uvc.h>
#include <linux/usb/video.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>

#include "u_uvc.h"
#include "uvc.h"
#include "uvc_configfs.h"
#include "uvc_v4l2.h"
#include "uvc_video.h"

unsigned int uvc_gadget_trace_param;
module_param_named(trace, uvc_gadget_trace_param, uint, 0644);
MODULE_PARM_DESC(trace, "Trace level bitmask");

/* --------------------------------------------------------------------------
 * Function descriptors
 */

/* string IDs are assigned dynamically */

#define UVC_STRING_CONTROL_IDX			0
#define UVC_STRING_STREAMING_IDX		1

static struct usb_string uvc_en_us_strings[] = {
	/* [UVC_STRING_CONTROL_IDX].s = DYNAMIC, */
	[UVC_STRING_STREAMING_IDX].s = "Video Streaming",
	{  }
};

static struct usb_gadget_strings uvc_stringtab = {
	.language = 0x0409,	/* en-us */
	.strings = uvc_en_us_strings,
};

static struct usb_gadget_strings *uvc_function_strings[] = {
	&uvc_stringtab,
	NULL,
};

#define UVC_INTF_VIDEO_CONTROL			0
#define UVC_INTF_VIDEO_STREAMING		1

#define UVC_STATUS_MAX_PACKET_SIZE		16	/* 16 bytes status */

static struct usb_interface_assoc_descriptor uvc_iad = {
	.bLength		= sizeof(uvc_iad),
	.bDescriptorType	= USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface	= 0,
	.bInterfaceCount	= 2,
	.bFunctionClass		= USB_CLASS_VIDEO,
	.bFunctionSubClass	= UVC_SC_VIDEO_INTERFACE_COLLECTION,
	.bFunctionProtocol	= 0x00,
	.iFunction		= 0,
};

static struct usb_interface_descriptor uvc_control_intf = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bInterfaceNumber	= UVC_INTF_VIDEO_CONTROL,
	.bAlternateSetting	= 0,
	.bNumEndpoints		= 1,
	.bInterfaceClass	= USB_CLASS_VIDEO,
	.bInterfaceSubClass	= UVC_SC_VIDEOCONTROL,
	.bInterfaceProtocol	= 0x00,
	.iInterface		= 0,
};

static struct usb_endpoint_descriptor uvc_control_ep = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize		= cpu_to_le16(UVC_STATUS_MAX_PACKET_SIZE),
	.bInterval		= 8,
};

static struct usb_ss_ep_comp_descriptor uvc_ss_control_comp = {
	.bLength		= sizeof(uvc_ss_control_comp),
	.bDescriptorType	= USB_DT_SS_ENDPOINT_COMP,
	/* The following 3 values can be tweaked if necessary. */
	.bMaxBurst		= 0,
	.bmAttributes		= 0,
	.wBytesPerInterval	= cpu_to_le16(UVC_STATUS_MAX_PACKET_SIZE),
};

static struct uvc_control_endpoint_descriptor uvc_control_cs_ep = {
	.bLength		= UVC_DT_CONTROL_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_CS_ENDPOINT,
	.bDescriptorSubType	= UVC_EP_INTERRUPT,
	.wMaxTransferSize	= cpu_to_le16(UVC_STATUS_MAX_PACKET_SIZE),
};

static struct usb_interface_descriptor uvc_streaming_intf_alt0 = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bInterfaceNumber	= UVC_INTF_VIDEO_STREAMING,
	.bAlternateSetting	= 0,
	.bNumEndpoints		= 0,
	.bInterfaceClass	= USB_CLASS_VIDEO,
	.bInterfaceSubClass	= UVC_SC_VIDEOSTREAMING,
	.bInterfaceProtocol	= 0x00,
	.iInterface		= 0,
};

static struct usb_interface_descriptor uvc_bulk_streaming_intf_alt0 = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bInterfaceNumber	= UVC_INTF_VIDEO_STREAMING,
	.bAlternateSetting	= 0,
	.bNumEndpoints		= 1,
	.bInterfaceClass	= USB_CLASS_VIDEO,
	.bInterfaceSubClass	= UVC_SC_VIDEOSTREAMING,
	.bInterfaceProtocol	= 0x00,
	.iInterface		= 0,
};

static struct usb_interface_descriptor uvc_streaming_intf_alt1 = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bInterfaceNumber	= UVC_INTF_VIDEO_STREAMING,
	.bAlternateSetting	= 1,
	.bNumEndpoints		= 1,
	.bInterfaceClass	= USB_CLASS_VIDEO,
	.bInterfaceSubClass	= UVC_SC_VIDEOSTREAMING,
	.bInterfaceProtocol	= 0x00,
	.iInterface		= 0,
};

static struct usb_endpoint_descriptor uvc_fs_streaming_ep = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_SYNC_ASYNC
				| USB_ENDPOINT_XFER_ISOC,
	/* The wMaxPacketSize and bInterval values will be initialized from
	 * module parameters.
	 */
};

static struct usb_endpoint_descriptor uvc_fs_bulk_streaming_ep = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	/* The wMaxPacketSize and bInterval values will be initialized from
	 * module parameters.
	 */
};

static struct usb_endpoint_descriptor uvc_hs_streaming_ep = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_SYNC_ASYNC
				| USB_ENDPOINT_XFER_ISOC,
	/* The wMaxPacketSize and bInterval values will be initialized from
	 * module parameters.
	 */
};

static struct usb_endpoint_descriptor uvc_hs_bulk_streaming_ep = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	/* The wMaxPacketSize and bInterval values will be initialized from
	 * module parameters.
	 */
};

static struct usb_endpoint_descriptor uvc_ss_streaming_ep = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,

	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_SYNC_ASYNC
				| USB_ENDPOINT_XFER_ISOC,
	/* The wMaxPacketSize and bInterval values will be initialized from
	 * module parameters.
	 */
};

static struct usb_endpoint_descriptor uvc_ss_bulk_streaming_ep = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,

	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	/* The wMaxPacketSize and bInterval values will be initialized from
	 * module parameters.
	 */
};

static struct usb_ss_ep_comp_descriptor uvc_ss_streaming_comp = {
	.bLength		= sizeof(uvc_ss_streaming_comp),
	.bDescriptorType	= USB_DT_SS_ENDPOINT_COMP,
	/* The bMaxBurst, bmAttributes and wBytesPerInterval values will be
	 * initialized from module parameters.
	 */
};

static struct usb_ss_ep_comp_descriptor uvc_ss_bulk_streaming_comp = {
	.bLength		= sizeof(uvc_ss_bulk_streaming_comp),
	.bDescriptorType	= USB_DT_SS_ENDPOINT_COMP,
	/* The bMaxBurst, bmAttributes and wBytesPerInterval values will be
	 * initialized from module parameters.
	 */
};

static const struct usb_descriptor_header * const uvc_fs_streaming[] = {
	(struct usb_descriptor_header *) &uvc_streaming_intf_alt1,
	(struct usb_descriptor_header *) &uvc_fs_streaming_ep,
	NULL,
};

static const struct usb_descriptor_header * const uvc_fs_bulk_streaming[] = {
	(struct usb_descriptor_header *)&uvc_fs_bulk_streaming_ep,
	NULL,
};

static const struct usb_descriptor_header * const uvc_hs_streaming[] = {
	(struct usb_descriptor_header *) &uvc_streaming_intf_alt1,
	(struct usb_descriptor_header *) &uvc_hs_streaming_ep,
	NULL,
};

static const struct usb_descriptor_header * const uvc_hs_bulk_streaming[] = {
	(struct usb_descriptor_header *)&uvc_hs_bulk_streaming_ep,
	NULL,
};

static const struct usb_descriptor_header * const uvc_ss_streaming[] = {
	(struct usb_descriptor_header *) &uvc_streaming_intf_alt1,
	(struct usb_descriptor_header *) &uvc_ss_streaming_ep,
	(struct usb_descriptor_header *) &uvc_ss_streaming_comp,
	NULL,
};

static const struct usb_descriptor_header * const uvc_ss_bulk_streaming[] = {
	(struct usb_descriptor_header *)&uvc_ss_bulk_streaming_ep,
	(struct usb_descriptor_header *)&uvc_ss_bulk_streaming_comp,
	NULL,
};

/* --------------------------------------------------------------------------
 * Control requests
 */

static void
uvc_function_ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct uvc_device *uvc = req->context;
	struct v4l2_event v4l2_event;
	struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;

	uvc_trace(UVC_TRACE_CONTROL,
		  "event_setup_out %d, data len %d\n",
		  uvc->event_setup_out, req->actual);

	if (uvc->event_setup_out) {
		uvc->event_setup_out = 0;

		memset(&v4l2_event, 0, sizeof(v4l2_event));
		v4l2_event.type = UVC_EVENT_DATA;
		uvc_event->data.length = min_t(unsigned int, req->actual,
			sizeof(uvc_event->data.data));
		memcpy(&uvc_event->data.data, req->buf, uvc_event->data.length);
		v4l2_event_queue(&uvc->vdev, &v4l2_event);
	}
}

static int
uvc_function_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct uvc_device *uvc = to_uvc(f);
	struct v4l2_event v4l2_event;
	struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;

	uvc_trace(UVC_TRACE_CONTROL,
		  "setup request %02x %02x value %04x index %04x %04x\n",
		  ctrl->bRequestType, ctrl->bRequest, le16_to_cpu(ctrl->wValue),
		  le16_to_cpu(ctrl->wIndex), le16_to_cpu(ctrl->wLength));

	if ((ctrl->bRequestType & USB_TYPE_MASK) != USB_TYPE_CLASS) {
		uvcg_info(f, "invalid request type\n");
		return -EINVAL;
	}

	/* Stall too big requests. */
	if (le16_to_cpu(ctrl->wLength) > UVC_MAX_REQUEST_SIZE)
		return -EINVAL;

	/* Tell the complete callback to generate an event for the next request
	 * that will be enqueued by UVCIOC_SEND_RESPONSE.
	 */
	uvc->event_setup_out = !(ctrl->bRequestType & USB_DIR_IN);
	uvc->event_length = le16_to_cpu(ctrl->wLength);

	memset(&v4l2_event, 0, sizeof(v4l2_event));
	v4l2_event.type = UVC_EVENT_SETUP;
	memcpy(&uvc_event->req, ctrl, sizeof(uvc_event->req));
	v4l2_event_queue(&uvc->vdev, &v4l2_event);

	return 0;
}

void uvc_function_setup_continue(struct uvc_device *uvc)
{
	struct usb_composite_dev *cdev = uvc->func.config->cdev;

	usb_composite_setup_continue(cdev);
}

static int
uvc_function_get_alt(struct usb_function *f, unsigned interface)
{
	struct uvc_device *uvc = to_uvc(f);
	struct f_uvc_opts *opts;

	uvcg_info(f, "%s(%u)\n", __func__, interface);

	opts = fi_to_f_uvc_opts(f->fi);

	if (interface == uvc->control_intf)
		return 0;
	else if (interface != uvc->streaming_intf)
		return -EINVAL;
	else if (!opts->streaming_bulk)
		return uvc->video.ep->enabled ? 1 : 0;
	else
		/*
		 * Alt settings in an interface are supported only for
		 * ISOC endpoints as there are different alt-settings for
		 * zero-bandwidth and full-bandwidth cases, but the same
		 * is not true for BULK endpoints, as they have a single
		 * alt-setting.
		 */
		return 0;
}

static int
uvc_function_set_alt(struct usb_function *f, unsigned interface, unsigned alt)
{
	struct uvc_device *uvc = to_uvc(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct v4l2_event v4l2_event;
	struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;
	struct f_uvc_opts *opts;
	int ret;

	uvcg_info(f, "%s(%u, %u)\n", __func__, interface, alt);

	opts = fi_to_f_uvc_opts(f->fi);

	if (interface == uvc->control_intf) {
		if (alt)
			return -EINVAL;

		uvcg_info(f, "reset UVC Control\n");
		usb_ep_disable(uvc->control_ep);

		if (!uvc->control_ep->desc)
			if (config_ep_by_speed(cdev->gadget, f, uvc->control_ep))
				return -EINVAL;

		usb_ep_enable(uvc->control_ep);

		if (uvc->event_suspend) {
			memset(&v4l2_event, 0, sizeof(v4l2_event));
			v4l2_event.type = UVC_EVENT_RESUME;
			v4l2_event_queue(&uvc->vdev, &v4l2_event);
			uvc->event_suspend = 0;
			uvc_trace(UVC_TRACE_SUSPEND, "send UVC_EVENT_RESUME\n");
		}

		if (uvc->state == UVC_STATE_DISCONNECTED) {
			memset(&v4l2_event, 0, sizeof(v4l2_event));
			v4l2_event.type = UVC_EVENT_CONNECT;
			uvc_event->speed = cdev->gadget->speed;
			v4l2_event_queue(&uvc->vdev, &v4l2_event);

			uvc->state = UVC_STATE_CONNECTED;
		}

		return 0;
	}

	if (interface != uvc->streaming_intf)
		return -EINVAL;

	if (!opts->streaming_bulk) {
		switch (alt) {
		case 0:
			if (uvc->state != UVC_STATE_STREAMING)
				return 0;

			if (uvc->video.ep)
				usb_ep_disable(uvc->video.ep);

			memset(&v4l2_event, 0, sizeof(v4l2_event));
			v4l2_event.type = UVC_EVENT_STREAMOFF;
			v4l2_event_queue(&uvc->vdev, &v4l2_event);

			uvc->state = UVC_STATE_CONNECTED;
			return 0;

		case 1:
			if (uvc->state != UVC_STATE_CONNECTED)
				return 0;

			if (!uvc->video.ep)
				return -EINVAL;

			INFO(cdev, "reset UVC\n");
			usb_ep_disable(uvc->video.ep);

			ret = config_ep_by_speed(f->config->cdev->gadget,
						 &uvc->func, uvc->video.ep);
			if (ret)
				return ret;
			usb_ep_enable(uvc->video.ep);

			memset(&v4l2_event, 0, sizeof(v4l2_event));
			v4l2_event.type = UVC_EVENT_STREAMON;
			v4l2_event_queue(&uvc->vdev, &v4l2_event);
			return USB_GADGET_DELAYED_STATUS;

		default:
			return -EINVAL;
		}
	} else {
		switch (uvc->state) {
		case UVC_STATE_CONNECTED:
			if (uvc->video.ep &&
			    !uvc->video.ep->enabled) {
				/*
				 * Enable the video streaming endpoint,
				 * but don't change the 'uvc->state'.
				 */
				ret = config_ep_by_speed(cdev->gadget,
							 &uvc->func,
							 uvc->video.ep);
				if (ret)
					return ret;
				ret = usb_ep_enable(uvc->video.ep);
				if (ret)
					return ret;
			} else {
				memset(&v4l2_event, 0, sizeof(v4l2_event));
				v4l2_event.type = UVC_EVENT_STREAMON;
				v4l2_event_queue(&uvc->vdev, &v4l2_event);

				uvc->state = UVC_STATE_STREAMING;
			}
			return 0;

		case UVC_STATE_STREAMING:
			if (!alt) {
				INFO(cdev, "bulk streaming intf not support alt 0\n");
				return 0;
			}

			if (uvc->video.ep &&
			    uvc->video.ep->enabled) {
				ret = usb_ep_disable(uvc->video.ep);
				if (ret)
					return ret;
			}

			memset(&v4l2_event, 0, sizeof(v4l2_event));
			v4l2_event.type = UVC_EVENT_STREAMOFF;
			v4l2_event_queue(&uvc->vdev, &v4l2_event);
			uvc->state = UVC_STATE_CONNECTED;
			return 0;

		default:
			return -EINVAL;
		}
	}
}

static void
uvc_function_disable(struct usb_function *f)
{
	struct uvc_device *uvc = to_uvc(f);
	struct v4l2_event v4l2_event;

	uvcg_info(f, "%s()\n", __func__);

	memset(&v4l2_event, 0, sizeof(v4l2_event));
	v4l2_event.type = UVC_EVENT_DISCONNECT;
	v4l2_event_queue(&uvc->vdev, &v4l2_event);

	uvc->state = UVC_STATE_DISCONNECTED;

	usb_ep_disable(uvc->video.ep);
	usb_ep_disable(uvc->control_ep);
}

static void uvc_function_suspend(struct usb_function *f)
{
	struct uvc_device *uvc = to_uvc(f);
	struct v4l2_event v4l2_event;

	memset(&v4l2_event, 0, sizeof(v4l2_event));
	v4l2_event.type = UVC_EVENT_SUSPEND;
	v4l2_event_queue(&uvc->vdev, &v4l2_event);
	uvc->event_suspend = 1;
	uvc_trace(UVC_TRACE_SUSPEND, "send UVC_EVENT_SUSPEND\n");
}

static void uvc_function_resume(struct usb_function *f)
{
	struct uvc_device *uvc = to_uvc(f);
	struct v4l2_event v4l2_event;

	memset(&v4l2_event, 0, sizeof(v4l2_event));
	v4l2_event.type = UVC_EVENT_RESUME;
	v4l2_event_queue(&uvc->vdev, &v4l2_event);
	uvc->event_suspend = 0;
	uvc_trace(UVC_TRACE_SUSPEND, "send UVC_EVENT_RESUME\n");
}

/* --------------------------------------------------------------------------
 * Connection / disconnection
 */

void
uvc_function_connect(struct uvc_device *uvc)
{
	int ret;

	if ((ret = usb_function_activate(&uvc->func)) < 0)
		uvcg_info(&uvc->func, "UVC connect failed with %d\n", ret);
}

void
uvc_function_disconnect(struct uvc_device *uvc)
{
	int ret;

	if ((ret = usb_function_deactivate(&uvc->func)) < 0)
		uvcg_info(&uvc->func, "UVC disconnect failed with %d\n", ret);
}

/* --------------------------------------------------------------------------
 * USB probe and disconnect
 */

static ssize_t function_name_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct uvc_device *uvc = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", uvc->func.fi->group.cg_item.ci_name);
}

static DEVICE_ATTR_RO(function_name);

static int
uvc_register_video(struct uvc_device *uvc)
{
	struct usb_composite_dev *cdev = uvc->func.config->cdev;
	int ret;

	/* TODO reference counting. */
	memset(&uvc->vdev, 0, sizeof(uvc->vdev));
	uvc->vdev.v4l2_dev = &uvc->v4l2_dev;
	uvc->vdev.fops = &uvc_v4l2_fops;
	uvc->vdev.ioctl_ops = &uvc_v4l2_ioctl_ops;
	uvc->vdev.release = video_device_release_empty;
	uvc->vdev.vfl_dir = VFL_DIR_TX;
	uvc->vdev.lock = &uvc->video.mutex;
	uvc->vdev.device_caps = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING;
	strlcpy(uvc->vdev.name, cdev->gadget->name, sizeof(uvc->vdev.name));

	video_set_drvdata(&uvc->vdev, uvc);

	ret = video_register_device(&uvc->vdev, VFL_TYPE_VIDEO, -1);
	if (ret < 0)
		return ret;

	ret = device_create_file(&uvc->vdev.dev, &dev_attr_function_name);
	if (ret < 0) {
		video_unregister_device(&uvc->vdev);
		return ret;
	}

	return 0;
}

#define UVC_COPY_DESCRIPTOR(mem, dst, desc) \
	do { \
		memcpy(mem, desc, (desc)->bLength); \
		*(dst)++ = mem; \
		mem += (desc)->bLength; \
	} while (0);

#define UVC_COPY_DESCRIPTORS(mem, dst, src) \
	do { \
		const struct usb_descriptor_header * const *__src; \
		for (__src = src; *__src; ++__src) { \
			memcpy(mem, *__src, (*__src)->bLength); \
			*dst++ = mem; \
			mem += (*__src)->bLength; \
		} \
	} while (0)

static struct usb_descriptor_header **
uvc_copy_descriptors(struct uvc_device *uvc, enum usb_device_speed speed)
{
	struct uvc_input_header_descriptor *uvc_streaming_header;
	struct uvc_header_descriptor *uvc_control_header;
	const struct uvc_descriptor_header * const *uvc_control_desc;
	const struct uvc_descriptor_header * const *uvc_streaming_cls;
	const struct usb_descriptor_header * const *uvc_streaming_std;
	const struct usb_descriptor_header * const *src;
	struct usb_interface_descriptor *streaming_intf_alt0;
	struct usb_descriptor_header **dst;
	struct usb_descriptor_header **hdr;
	struct f_uvc_opts *opts;
	unsigned int control_size;
	unsigned int streaming_size;
	unsigned int n_desc;
	unsigned int bytes;
	void *mem;

	opts = fi_to_f_uvc_opts(uvc->func.fi);

	switch (speed) {
	case USB_SPEED_SUPER:
		uvc_control_desc = uvc->desc.ss_control;
		uvc_streaming_cls = uvc->desc.ss_streaming;
		if (!opts->streaming_bulk)
			uvc_streaming_std = uvc_ss_streaming;
		else
			uvc_streaming_std = uvc_ss_bulk_streaming;
		break;

	case USB_SPEED_HIGH:
		uvc_control_desc = uvc->desc.fs_control;
		uvc_streaming_cls = uvc->desc.hs_streaming;
		if (!opts->streaming_bulk)
			uvc_streaming_std = uvc_hs_streaming;
		else
			uvc_streaming_std = uvc_hs_bulk_streaming;
		break;

	case USB_SPEED_FULL:
	default:
		uvc_control_desc = uvc->desc.fs_control;
		uvc_streaming_cls = uvc->desc.fs_streaming;
		if (!opts->streaming_bulk)
			uvc_streaming_std = uvc_fs_streaming;
		else
			uvc_streaming_std = uvc_fs_bulk_streaming;
		break;
	}

	if (!uvc_control_desc || !uvc_streaming_cls)
		return ERR_PTR(-ENODEV);

	/* Descriptors layout
	 *
	 * uvc_iad
	 * uvc_control_intf
	 * Class-specific UVC control descriptors
	 * uvc_control_ep
	 * uvc_control_cs_ep
	 * uvc_ss_control_comp (for SS only)
	 * uvc_streaming_intf_alt0
	 * Class-specific UVC streaming descriptors
	 * uvc_{fs|hs}_streaming
	 */

	if (!opts->streaming_bulk)
		streaming_intf_alt0 = &uvc_streaming_intf_alt0;
	else
		streaming_intf_alt0 = &uvc_bulk_streaming_intf_alt0;

	/* Count descriptors and compute their size. */
	control_size = 0;
	streaming_size = 0;
	bytes = uvc_iad.bLength + uvc_control_intf.bLength
	      + uvc_control_ep.bLength + uvc_control_cs_ep.bLength
	      + streaming_intf_alt0->bLength;

	if (speed == USB_SPEED_SUPER) {
		bytes += uvc_ss_control_comp.bLength;
		n_desc = 6;
	} else {
		n_desc = 5;
	}

	for (src = (const struct usb_descriptor_header **)uvc_control_desc;
	     *src; ++src) {
		control_size += (*src)->bLength;
		bytes += (*src)->bLength;
		n_desc++;
	}
	for (src = (const struct usb_descriptor_header **)uvc_streaming_cls;
	     *src; ++src) {
		streaming_size += (*src)->bLength;
		bytes += (*src)->bLength;
		n_desc++;
	}
	for (src = uvc_streaming_std; *src; ++src) {
		bytes += (*src)->bLength;
		n_desc++;
	}

	mem = kmalloc((n_desc + 1) * sizeof(*src) + bytes, GFP_KERNEL);
	if (mem == NULL)
		return NULL;

	hdr = mem;
	dst = mem;
	mem += (n_desc + 1) * sizeof(*src);

	/* Copy the descriptors. */
	UVC_COPY_DESCRIPTOR(mem, dst, &uvc_iad);
	UVC_COPY_DESCRIPTOR(mem, dst, &uvc_control_intf);

	uvc_control_header = mem;
	UVC_COPY_DESCRIPTORS(mem, dst,
		(const struct usb_descriptor_header **)uvc_control_desc);
	uvc_control_header->wTotalLength = cpu_to_le16(control_size);
	uvc_control_header->bInCollection = 1;
	uvc_control_header->baInterfaceNr[0] = uvc->streaming_intf;

	UVC_COPY_DESCRIPTOR(mem, dst, &uvc_control_ep);
	if (speed == USB_SPEED_SUPER)
		UVC_COPY_DESCRIPTOR(mem, dst, &uvc_ss_control_comp);

	UVC_COPY_DESCRIPTOR(mem, dst, &uvc_control_cs_ep);
	UVC_COPY_DESCRIPTOR(mem, dst, streaming_intf_alt0);

	uvc_streaming_header = mem;
	UVC_COPY_DESCRIPTORS(mem, dst,
		(const struct usb_descriptor_header**)uvc_streaming_cls);
	uvc_streaming_header->wTotalLength = cpu_to_le16(streaming_size);
	uvc_streaming_header->bEndpointAddress = uvc->video.ep->address;

	UVC_COPY_DESCRIPTORS(mem, dst, uvc_streaming_std);

	*dst = NULL;
	return hdr;
}

static int
uvc_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct uvc_device *uvc = to_uvc(f);
	struct usb_string *us;
	unsigned int max_packet_mult;
	unsigned int max_packet_size;
	struct usb_ep *ep;
	struct f_uvc_opts *opts;
	int ret = -EINVAL;
	u8 address;

	uvcg_info(f, "%s()\n", __func__);

	opts = fi_to_f_uvc_opts(f->fi);
	/* Sanity check the streaming endpoint module parameters.
	 */
	if (!opts->streaming_bulk) {
		opts->streaming_interval = clamp(opts->streaming_interval,
						 1U, 16U);
		opts->streaming_maxpacket = clamp(opts->streaming_maxpacket,
						  1U, 3072U);
		opts->streaming_maxburst = min(opts->streaming_maxburst, 15U);
	} else {
		opts->streaming_maxpacket = clamp(opts->streaming_maxpacket,
						  1U, 1024U);
		opts->streaming_maxburst = min(opts->streaming_maxburst, 15U);
	}

	/* For SS, wMaxPacketSize has to be 1024 if bMaxBurst is not 0 */
	if (opts->streaming_maxburst &&
	    (opts->streaming_maxpacket % 1024) != 0) {
		opts->streaming_maxpacket = roundup(opts->streaming_maxpacket, 1024);
		uvcg_info(f, "overriding streaming_maxpacket to %d\n",
			  opts->streaming_maxpacket);
	}

	/* Fill in the FS/HS/SS Video Streaming specific descriptors from the
	 * module parameters.
	 *
	 * NOTE: We assume that the user knows what they are doing and won't
	 * give parameters that their UDC doesn't support.
	 */
	if (opts->streaming_maxpacket <= 1024) {
		max_packet_mult = 1;
		max_packet_size = opts->streaming_maxpacket;
	} else if (opts->streaming_maxpacket <= 2048) {
		max_packet_mult = 2;
		max_packet_size = opts->streaming_maxpacket / 2;
	} else {
		max_packet_mult = 3;
		max_packet_size = opts->streaming_maxpacket / 3;
	}

	if (!opts->streaming_bulk) {
		uvc_fs_streaming_ep.wMaxPacketSize =
			cpu_to_le16(min(opts->streaming_maxpacket, 1023U));
		uvc_fs_streaming_ep.bInterval = opts->streaming_interval;

		uvc_hs_streaming_ep.wMaxPacketSize =
			cpu_to_le16(max_packet_size |
				    ((max_packet_mult - 1) << 11));

		/* A high-bandwidth endpoint must specify a bInterval value of 1 */
		if (max_packet_mult > 1)
			uvc_hs_streaming_ep.bInterval = 1;
		else
			uvc_hs_streaming_ep.bInterval = opts->streaming_interval;

		uvc_ss_streaming_ep.wMaxPacketSize =
			cpu_to_le16(max_packet_size);
		uvc_ss_streaming_ep.bInterval = opts->streaming_interval;
		uvc_ss_streaming_comp.bmAttributes = max_packet_mult - 1;
		uvc_ss_streaming_comp.bMaxBurst = opts->streaming_maxburst;
		uvc_ss_streaming_comp.wBytesPerInterval =
			cpu_to_le16(max_packet_size * max_packet_mult *
				    (opts->streaming_maxburst + 1));
	} else {
		uvc_fs_bulk_streaming_ep.wMaxPacketSize =
			cpu_to_le16(min(opts->streaming_maxpacket, 64U));

		uvc_hs_bulk_streaming_ep.wMaxPacketSize =
			cpu_to_le16(min(opts->streaming_maxpacket, 512U));

		uvc_ss_bulk_streaming_ep.wMaxPacketSize =
			cpu_to_le16(max_packet_size);
		uvc_ss_bulk_streaming_comp.bMaxBurst = opts->streaming_maxburst;
		/*
		 * As per USB 3.1 spec "Table 9-26. SuperSpeed Endpoint
		 * Companion Descriptor", the wBytesPerInterval must be
		 * set to zero for bulk endpoints.
		 */
		uvc_ss_bulk_streaming_comp.wBytesPerInterval = 0;
	}

	/* Allocate endpoints. */
	ep = usb_ep_autoconfig(cdev->gadget, &uvc_control_ep);
	if (!ep) {
		uvcg_info(f, "Unable to allocate control EP\n");
		goto error;
	}
	uvc->control_ep = ep;

	if (gadget_is_superspeed(c->cdev->gadget)) {
		if (!opts->streaming_bulk)
			ep = usb_ep_autoconfig_ss(cdev->gadget,
						  &uvc_ss_streaming_ep,
						  &uvc_ss_streaming_comp);
		else
			ep = usb_ep_autoconfig_ss(cdev->gadget,
						  &uvc_ss_bulk_streaming_ep,
						  &uvc_ss_bulk_streaming_comp);
	} else if (gadget_is_dualspeed(cdev->gadget)) {
		if (!opts->streaming_bulk) {
			ep = usb_ep_autoconfig(cdev->gadget,
					       &uvc_hs_streaming_ep);
		} else {
			ep = usb_ep_autoconfig(cdev->gadget,
					       &uvc_hs_bulk_streaming_ep);
			/*
			 * In ep_matches(), it will set wMaxPacketSize to 64
			 * bytes if ep is Bulk and ep_comp is NULL for hs/fs
			 * bulk maxpacket. So we need to set hs bulk maxpacket
			 * 512 bytes again here.
			 */
			uvc_hs_bulk_streaming_ep.wMaxPacketSize =
				cpu_to_le16(min(opts->streaming_maxpacket,
						512U));
		}
	} else {
		if (!opts->streaming_bulk)
			ep = usb_ep_autoconfig(cdev->gadget,
					       &uvc_fs_streaming_ep);
		else
			ep = usb_ep_autoconfig(cdev->gadget,
					       &uvc_fs_bulk_streaming_ep);
	}

	if (!ep) {
		uvcg_info(f, "Unable to allocate streaming EP\n");
		goto error;
	}
	uvc->video.ep = ep;
	address = uvc->video.ep->address;

	if (!opts->streaming_bulk) {
		uvc_fs_streaming_ep.bEndpointAddress = address;
		uvc_hs_streaming_ep.bEndpointAddress = address;
		uvc_ss_streaming_ep.bEndpointAddress = address;
	} else {
		uvc_fs_bulk_streaming_ep.bEndpointAddress = address;
		uvc_hs_bulk_streaming_ep.bEndpointAddress = address;
		uvc_ss_bulk_streaming_ep.bEndpointAddress = address;
	}

#if defined(CONFIG_ARCH_ROCKCHIP) && defined(CONFIG_NO_GKI)
	if (opts->device_name)
		uvc_en_us_strings[UVC_STRING_CONTROL_IDX].s = opts->device_name;
#endif

	uvc_en_us_strings[UVC_STRING_CONTROL_IDX].s = opts->function_name;
	us = usb_gstrings_attach(cdev, uvc_function_strings,
				 ARRAY_SIZE(uvc_en_us_strings));
	if (IS_ERR(us)) {
		ret = PTR_ERR(us);
		goto error;
	}
	uvc_iad.iFunction = us[UVC_STRING_CONTROL_IDX].id;
	uvc_control_intf.iInterface = us[UVC_STRING_CONTROL_IDX].id;
	ret = us[UVC_STRING_STREAMING_IDX].id;
	if (!opts->streaming_bulk) {
		uvc_streaming_intf_alt0.iInterface = ret;
		uvc_streaming_intf_alt1.iInterface = ret;
	} else {
		uvc_bulk_streaming_intf_alt0.iInterface = ret;
	}

	/* Allocate interface IDs. */
	if ((ret = usb_interface_id(c, f)) < 0)
		goto error;
	uvc_iad.bFirstInterface = ret;
	uvc_control_intf.bInterfaceNumber = ret;
	uvc->control_intf = ret;
	opts->control_interface = ret;

	if ((ret = usb_interface_id(c, f)) < 0)
		goto error;

	if (!opts->streaming_bulk) {
		uvc_streaming_intf_alt0.bInterfaceNumber = ret;
		uvc_streaming_intf_alt1.bInterfaceNumber = ret;
	} else {
		uvc_bulk_streaming_intf_alt0.bInterfaceNumber = ret;
	}

	uvc->streaming_intf = ret;
	opts->streaming_interface = ret;

	/* Copy descriptors */
	f->fs_descriptors = uvc_copy_descriptors(uvc, USB_SPEED_FULL);
	if (IS_ERR(f->fs_descriptors)) {
		ret = PTR_ERR(f->fs_descriptors);
		f->fs_descriptors = NULL;
		goto error;
	}
	if (gadget_is_dualspeed(cdev->gadget)) {
		f->hs_descriptors = uvc_copy_descriptors(uvc, USB_SPEED_HIGH);
		if (IS_ERR(f->hs_descriptors)) {
			ret = PTR_ERR(f->hs_descriptors);
			f->hs_descriptors = NULL;
			goto error;
		}
	}
	if (gadget_is_superspeed(c->cdev->gadget)) {
		f->ss_descriptors = uvc_copy_descriptors(uvc, USB_SPEED_SUPER);
		if (IS_ERR(f->ss_descriptors)) {
			ret = PTR_ERR(f->ss_descriptors);
			f->ss_descriptors = NULL;
			goto error;
		}
	}

	/* Preallocate control endpoint request. */
	uvc->control_req = usb_ep_alloc_request(cdev->gadget->ep0, GFP_KERNEL);
	uvc->control_buf = kmalloc(UVC_MAX_REQUEST_SIZE, GFP_KERNEL);
	if (uvc->control_req == NULL || uvc->control_buf == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	uvc->control_req->buf = uvc->control_buf;
	uvc->control_req->complete = uvc_function_ep0_complete;
	uvc->control_req->context = uvc;

	if (v4l2_device_register(&cdev->gadget->dev, &uvc->v4l2_dev)) {
		uvcg_err(f, "failed to register V4L2 device\n");
		goto error;
	}

	/* Initialise video. */
	ret = uvcg_video_init(&uvc->video, uvc);
	if (ret < 0)
		goto v4l2_error;

	if (opts->streaming_bulk)
		uvc->video.max_payload_size = uvc->video.imagesize;
	/* Register a V4L2 device. */
	ret = uvc_register_video(uvc);
	if (ret < 0) {
		uvcg_err(f, "failed to register video device\n");
		goto v4l2_error;
	}

	return 0;

v4l2_error:
	v4l2_device_unregister(&uvc->v4l2_dev);
error:
	if (uvc->control_req)
		usb_ep_free_request(cdev->gadget->ep0, uvc->control_req);
	kfree(uvc->control_buf);

	usb_free_all_descriptors(f);
	return ret;
}

/* --------------------------------------------------------------------------
 * USB gadget function
 */

static void uvc_free_inst(struct usb_function_instance *f)
{
	struct f_uvc_opts *opts = fi_to_f_uvc_opts(f);

	mutex_destroy(&opts->lock);

#if defined(CONFIG_ARCH_ROCKCHIP) && defined(CONFIG_NO_GKI)
	if (opts->device_name_allocated) {
		opts->device_name_allocated = false;
		kfree(opts->device_name);
		opts->device_name = NULL;
	}
#endif

	kfree(opts);
}

static struct usb_function_instance *uvc_alloc_inst(void)
{
	struct f_uvc_opts *opts;
	struct uvc_camera_terminal_descriptor *cd;
	struct uvc_processing_unit_descriptor *pd;
	struct uvc_output_terminal_descriptor *od;
	struct UVC_EXTENSION_UNIT_DESCRIPTOR(1, 1) *ed;
	struct uvc_color_matching_descriptor *md;
	struct uvc_descriptor_header **ctl_cls;
	int ret;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);
	opts->func_inst.free_func_inst = uvc_free_inst;
	mutex_init(&opts->lock);

	cd = &opts->uvc_camera_terminal;
	cd->bLength			= UVC_DT_CAMERA_TERMINAL_SIZE(3);
	cd->bDescriptorType		= USB_DT_CS_INTERFACE;
	cd->bDescriptorSubType		= UVC_VC_INPUT_TERMINAL;
	cd->bTerminalID			= 1;
	cd->wTerminalType		= cpu_to_le16(0x0201);
	cd->bAssocTerminal		= 0;
	cd->iTerminal			= 0;
	cd->wObjectiveFocalLengthMin	= cpu_to_le16(0);
	cd->wObjectiveFocalLengthMax	= cpu_to_le16(0);
	cd->wOcularFocalLength		= cpu_to_le16(0);
	cd->bControlSize		= 3;
	cd->bmControls[0]		= 2;
	cd->bmControls[1]		= 0;
	cd->bmControls[2]		= 0;

	pd = &opts->uvc_processing;
	pd->bLength			= UVC_DT_PROCESSING_UNIT_SIZE(2);
	pd->bDescriptorType		= USB_DT_CS_INTERFACE;
	pd->bDescriptorSubType		= UVC_VC_PROCESSING_UNIT;
	pd->bUnitID			= 2;
	pd->bSourceID			= 1;
	pd->wMaxMultiplier		= cpu_to_le16(16*1024);
	pd->bControlSize		= 2;
	pd->bmControls[0]		= 1;
	pd->bmControls[1]		= 0;
	pd->iProcessing			= 0;
	pd->bmVideoStandards		= 0;

	od = &opts->uvc_output_terminal;
	od->bLength			= UVC_DT_OUTPUT_TERMINAL_SIZE;
	od->bDescriptorType		= USB_DT_CS_INTERFACE;
	od->bDescriptorSubType		= UVC_VC_OUTPUT_TERMINAL;
	od->bTerminalID			= 3;
	od->wTerminalType		= cpu_to_le16(0x0101);
	od->bAssocTerminal		= 0;
	od->bSourceID			= 2;
	od->iTerminal			= 0;

	ed = &opts->uvc_extension;
	ed->bLength = UVC_DT_EXTENSION_UNIT_SIZE(1, 1);
	ed->bDescriptorType = USB_DT_CS_INTERFACE;
	ed->bDescriptorSubType = UVC_VC_EXTENSION_UNIT;
	ed->bUnitID = 6;
	ed->guidExtensionCode[0] = 0xa2;
	ed->guidExtensionCode[1] = 0x9e;
	ed->guidExtensionCode[2] = 0x76;
	ed->guidExtensionCode[3] = 0x41;
	ed->guidExtensionCode[4] = 0xde;
	ed->guidExtensionCode[5] = 0x04;
	ed->guidExtensionCode[6] = 0x47;
	ed->guidExtensionCode[7] = 0xe3;
	ed->guidExtensionCode[8] = 0x8b;
	ed->guidExtensionCode[9] = 0x2b;
	ed->guidExtensionCode[10] = 0xf4;
	ed->guidExtensionCode[11] = 0x34;
	ed->guidExtensionCode[12] = 0x1a;
	ed->guidExtensionCode[13] = 0xff;
	ed->guidExtensionCode[14] = 0x00;
	ed->guidExtensionCode[15] = 0x3b;
	ed->bNumControls = 3;
	ed->bNrInPins = 1;
	ed->baSourceID[0] = 2;
	ed->bControlSize = 1;
	ed->bmControls[0] = 7;
	ed->iExtension = 0;

	md = &opts->uvc_color_matching;
	md->bLength			= UVC_DT_COLOR_MATCHING_SIZE;
	md->bDescriptorType		= USB_DT_CS_INTERFACE;
	md->bDescriptorSubType		= UVC_VS_COLORFORMAT;
	md->bColorPrimaries		= 1;
	md->bTransferCharacteristics	= 1;
	md->bMatrixCoefficients		= 4;

	/* Prepare fs control class descriptors for configfs-based gadgets */
	ctl_cls = opts->uvc_fs_control_cls;
	ctl_cls[0] = NULL;	/* assigned elsewhere by configfs */
	ctl_cls[1] = (struct uvc_descriptor_header *)cd;
	ctl_cls[2] = (struct uvc_descriptor_header *)pd;
	ctl_cls[3] = (struct uvc_descriptor_header *)od;
	ctl_cls[4] = (struct uvc_descriptor_header *)ed;
	ctl_cls[5] = NULL;	/* NULL-terminate */
	opts->fs_control =
		(const struct uvc_descriptor_header * const *)ctl_cls;

	/* Prepare hs control class descriptors for configfs-based gadgets */
	ctl_cls = opts->uvc_ss_control_cls;
	ctl_cls[0] = NULL;	/* assigned elsewhere by configfs */
	ctl_cls[1] = (struct uvc_descriptor_header *)cd;
	ctl_cls[2] = (struct uvc_descriptor_header *)pd;
	ctl_cls[3] = (struct uvc_descriptor_header *)od;
	ctl_cls[4] = (struct uvc_descriptor_header *)ed;
	ctl_cls[5] = NULL;	/* NULL-terminate */
	opts->ss_control =
		(const struct uvc_descriptor_header * const *)ctl_cls;

	opts->streaming_interval = 1;
	opts->streaming_maxpacket = 1024;
	opts->pm_qos_latency = 0;
	snprintf(opts->function_name, sizeof(opts->function_name), "UVC Camera");

	ret = uvcg_attach_configfs(opts);
	if (ret < 0) {
		kfree(opts);
		return ERR_PTR(ret);
	}

	return &opts->func_inst;
}

static void uvc_free(struct usb_function *f)
{
	struct uvc_device *uvc = to_uvc(f);
	struct f_uvc_opts *opts = container_of(f->fi, struct f_uvc_opts,
					       func_inst);
	--opts->refcnt;
	kfree(uvc);
}

static void uvc_function_unbind(struct usb_configuration *c,
				struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct uvc_device *uvc = to_uvc(f);
	struct uvc_video *video = &uvc->video;
	long wait_ret = 1;

	uvcg_info(f, "%s()\n", __func__);

	if (video->async_wq)
		destroy_workqueue(video->async_wq);

	/* If we know we're connected via v4l2, then there should be a cleanup
	 * of the device from userspace either via UVC_EVENT_DISCONNECT or
	 * though the video device removal uevent. Allow some time for the
	 * application to close out before things get deleted.
	 */
	if (uvc->func_connected) {
		uvcg_dbg(f, "waiting for clean disconnect\n");
		wait_ret = wait_event_interruptible_timeout(uvc->func_connected_queue,
				uvc->func_connected == false, msecs_to_jiffies(500));
		uvcg_dbg(f, "done waiting with ret: %ld\n", wait_ret);
	}

	device_remove_file(&uvc->vdev.dev, &dev_attr_function_name);
	video_unregister_device(&uvc->vdev);
	v4l2_device_unregister(&uvc->v4l2_dev);

	if (uvc->func_connected) {
		/* Wait for the release to occur to ensure there are no longer any
		 * pending operations that may cause panics when resources are cleaned
		 * up.
		 */
		uvcg_warn(f, "%s no clean disconnect, wait for release\n", __func__);
		wait_ret = wait_event_interruptible_timeout(uvc->func_connected_queue,
				uvc->func_connected == false, msecs_to_jiffies(1000));
		uvcg_dbg(f, "done waiting for release with ret: %ld\n", wait_ret);
	}

	usb_ep_free_request(cdev->gadget->ep0, uvc->control_req);
	kfree(uvc->control_buf);

	usb_free_all_descriptors(f);
}

static struct usb_function *uvc_alloc(struct usb_function_instance *fi)
{
	struct uvc_device *uvc;
	struct f_uvc_opts *opts;
	struct uvc_descriptor_header **strm_cls;

	uvc = kzalloc(sizeof(*uvc), GFP_KERNEL);
	if (uvc == NULL)
		return ERR_PTR(-ENOMEM);

	mutex_init(&uvc->video.mutex);
	uvc->state = UVC_STATE_DISCONNECTED;
	init_waitqueue_head(&uvc->func_connected_queue);
	opts = fi_to_f_uvc_opts(fi);

	mutex_lock(&opts->lock);
	if (opts->uvc_fs_streaming_cls) {
		strm_cls = opts->uvc_fs_streaming_cls;
		opts->fs_streaming =
			(const struct uvc_descriptor_header * const *)strm_cls;
	}
	if (opts->uvc_hs_streaming_cls) {
		strm_cls = opts->uvc_hs_streaming_cls;
		opts->hs_streaming =
			(const struct uvc_descriptor_header * const *)strm_cls;
	}
	if (opts->uvc_ss_streaming_cls) {
		strm_cls = opts->uvc_ss_streaming_cls;
		opts->ss_streaming =
			(const struct uvc_descriptor_header * const *)strm_cls;
	}

	uvc->desc.fs_control = opts->fs_control;
	uvc->desc.ss_control = opts->ss_control;
	uvc->desc.fs_streaming = opts->fs_streaming;
	uvc->desc.hs_streaming = opts->hs_streaming;
	uvc->desc.ss_streaming = opts->ss_streaming;
	++opts->refcnt;
	mutex_unlock(&opts->lock);

	/* Register the function. */
	uvc->func.name = "uvc";
	uvc->func.bind = uvc_function_bind;
	uvc->func.unbind = uvc_function_unbind;
	uvc->func.get_alt = uvc_function_get_alt;
	uvc->func.set_alt = uvc_function_set_alt;
	uvc->func.disable = uvc_function_disable;
	uvc->func.setup = uvc_function_setup;
	uvc->func.free_func = uvc_free;
	uvc->func.suspend = uvc_function_suspend;
	uvc->func.resume = uvc_function_resume;
	uvc->func.bind_deactivated = true;

	return &uvc->func;
}

DECLARE_USB_FUNCTION_INIT(uvc, uvc_alloc_inst, uvc_alloc);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Laurent Pinchart");
