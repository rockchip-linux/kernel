// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip NPU USB ACM driver
 *
 * Copyright (C) 2019 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * This driver was originally based on:
 * drivers/usb/class/cdc-acm.c
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/serial.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <linux/usb/npu-acm.h>

/*
 * Requests.
 */
#define USB_RT_ACM		(USB_TYPE_CLASS | USB_RECIP_INTERFACE)

/*
 * Output control lines.
 */
#define ACM_CTRL_DTR		0x01
#define ACM_CTRL_RTS		0x02

/*
 * The only reason to have several buffers is to accommodate assumptions
 * in line disciplines. They ask for empty space amount, receive our URB size,
 * and proceed to issue several 1-character writes, assuming they will fit.
 * The very first write takes a complete URB. Fortunately, this only happens
 * when processing onlcr, so we only need 2 buffers. These values must be
 * powers of 2.
 */
#define ACM_NW	16
#define ACM_NR	16

#define ACM_READ_TIMEOUT	5

struct acm_wb {
	unsigned char	*buf;
	dma_addr_t	dmah;
	int		len;
	int		use;
	struct urb	*urb;
	struct acm	*instance;
};

struct acm_rb {
	int		size;
	unsigned char	*base;
	dma_addr_t	dma;
	int		index;
	struct acm	*instance;
};

struct acm {
	/* the corresponding usb device */
	struct usb_device	*dev;
	struct usb_interface	*control;	/* control interface */
	struct usb_interface	*data;		/* data interface */
	struct urb		*ctrlurb;	/* urbs */
	u8			*ctrl_buffer;	/* buffers of urbs */
	dma_addr_t		ctrl_dma;	/* dma handles of buffers */
	struct acm_wb		wb[ACM_NW];
	unsigned long		read_urbs_free;
	struct urb		*read_urbs[ACM_NR];
	struct acm_rb		read_buffers[ACM_NR];
	int			rx_buflimit;
	spinlock_t		read_lock;
	int			transmitting;
	spinlock_t		write_lock;
	struct mutex		mutex;
	bool			disconnected;
	int			rb_index;
	bool			readable;
	wait_queue_head_t	read_wait;

	/* max packet size for the output bulk endpoint */
	unsigned int		writesize;
	/* buffer sizes for freeing */
	unsigned int		readsize, ctrlsize;
	/* number of suspended interfaces */
	unsigned int		susp_count;
	/* interrupt endpoints contrary to spec used */
	unsigned int		is_int_ep:1;
	u8			bInterval;

	/* writes queued for a device about to be woken */
	struct usb_anchor	delayed;

	/* bits, stop, parity */
	struct usb_cdc_line_coding	line;
};

static struct usb_driver npu_acm_driver;
static struct acm *npu_acm;

/*
 * Functions for ACM control messages.
 */
static int acm_ctrl_msg(struct acm *acm, int request, int value,
			void *buf, int len)
{
	int retval;
	int ifnum = acm->control->altsetting[0].desc.bInterfaceNumber;

	retval = usb_autopm_get_interface(acm->control);
	if (retval)
		return retval;

	retval = usb_control_msg(acm->dev, usb_sndctrlpipe(acm->dev, 0),
				 request, USB_RT_ACM, value,
				 ifnum, buf, len, 5000);
	dev_dbg(&acm->control->dev,
		"%s - rq 0x%02x, val %#x, len %#x, result %d\n",
		__func__, request, value, len, retval);

	usb_autopm_put_interface(acm->control);

	return retval < 0 ? retval : 0;
}

static inline int acm_set_control(struct acm *acm, int control)
{
	return acm_ctrl_msg(acm, USB_CDC_REQ_SET_CONTROL_LINE_STATE,
			    control, NULL, 0);
}

static inline int acm_set_line(struct acm *acm,
			       struct usb_cdc_line_coding *line)
{
	return acm_ctrl_msg(acm, USB_CDC_REQ_SET_LINE_CODING,
			    0, line, sizeof *(line));
}

/*
 * Write buffer management.
 * All of these assume proper locks taken by the caller.
 */
static int acm_wb_alloc(struct acm *acm)
{
	int i, wbn;
	struct acm_wb *wb;

	wbn = 0;
	i = 0;
	for (;;) {
		wb = &acm->wb[wbn];
		if (!wb->use) {
			wb->use = 1;
			return wbn;
		}
		wbn = (wbn + 1) % ACM_NW;
		if (++i >= ACM_NW)
			return -1;
	}
}

static int acm_wb_is_avail(struct acm *acm)
{
	int i, n;
	unsigned long flags;

	n = ACM_NW;
	spin_lock_irqsave(&acm->write_lock, flags);
	for (i = 0; i < ACM_NW; i++)
		n -= acm->wb[i].use;
	spin_unlock_irqrestore(&acm->write_lock, flags);
	return n;
}

/*
 * Finish write. Caller must hold acm->write_lock
 */
static void acm_write_done(struct acm *acm, struct acm_wb *wb)
{
	wb->use = 0;
	acm->transmitting--;
	usb_autopm_put_interface_async(acm->control);
}

/*
 * Poke write.
 *
 * the caller is responsible for locking
 */
static int acm_start_wb(struct acm *acm, struct acm_wb *wb)
{
	int rc;

	acm->transmitting++;

	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = acm->dev;

	rc = usb_submit_urb(wb->urb, GFP_ATOMIC);
	if (rc < 0) {
		dev_err(&acm->data->dev,
			"%s - usb_submit_urb(write bulk) failed: %d\n",
			__func__, rc);
		acm_write_done(acm, wb);
	}
	return rc;
}

/*
 * Interrupt handlers for various ACM device responses
 */

/* control interface reports status changes with "interrupt" transfers */
static void acm_ctrl_irq(struct urb *urb)
{
	struct acm *acm = urb->context;
	struct usb_cdc_notification *dr = urb->transfer_buffer;
	int retval;
	int status = urb->status;

	switch (status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(&acm->control->dev,
			"%s - urb shutting down with status: %d\n",
			__func__, status);
		return;
	default:
		dev_dbg(&acm->control->dev,
			"%s - nonzero urb status received: %d\n",
			__func__, status);
		goto exit;
	}

	usb_mark_last_busy(acm->dev);

	switch (dr->bNotificationType) {
	case USB_CDC_NOTIFY_NETWORK_CONNECTION:
		dev_dbg(&acm->control->dev, "%s - network connection: %d\n",
			__func__, dr->wValue);
		break;

	case USB_CDC_NOTIFY_SERIAL_STATE:
		if (le16_to_cpu(dr->wLength) != 2) {
			dev_dbg(&acm->control->dev,
				"%s - malformed serial state\n", __func__);
			break;
		}

		break;

	default:
		dev_dbg(&acm->control->dev,
			"%s - unknown notification %d received: index %d len %d\n",
			__func__,
			dr->bNotificationType, dr->wIndex, dr->wLength);

		break;
	}
exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval && retval != -EPERM)
		dev_err(&acm->control->dev, "%s - usb_submit_urb failed: %d\n",
			__func__, retval);
}

static int acm_submit_read_urb(struct acm *acm, int index, gfp_t mem_flags)
{
	int res;

	if (!test_and_clear_bit(index, &acm->read_urbs_free))
		return 0;

	dev_vdbg(&acm->data->dev, "%s - urb %d\n", __func__, index);

	res = usb_submit_urb(acm->read_urbs[index], mem_flags);
	if (res) {
		if (res != -EPERM && res != -ENODEV) {
			dev_err(&acm->data->dev,
				"%s - usb_submit_urb failed: %d\n",
				__func__, res);
		}
		set_bit(index, &acm->read_urbs_free);
		return res;
	}

	return 0;
}

static int acm_submit_read_urbs(struct acm *acm, gfp_t mem_flags)
{
	int res;
	int i;

	for (i = 0; i < acm->rx_buflimit; ++i) {
		res = acm_submit_read_urb(acm, i, mem_flags);
		if (res)
			return res;
	}

	return 0;
}

static void acm_process_read_urb(struct acm *acm, struct urb *urb)
{
	struct acm_rb *rb = urb->context;

	if (!urb->actual_length)
		return;

	rb->size = urb->actual_length;

	if (!memcmp(urb->transfer_buffer, "\r\n", urb->actual_length)) {
		acm->readable = true;
		wake_up_interruptible(&acm->read_wait);
	} else {
		acm->rb_index = rb->index;
	}
}

static void acm_read_bulk_callback(struct urb *urb)
{
	struct acm_rb *rb = urb->context;
	struct acm *acm = rb->instance;
	int status = urb->status;

	dev_vdbg(&acm->data->dev, "%s - urb %d, len %d\n", __func__,
		 rb->index, urb->actual_length);

	if (!acm->dev) {
		set_bit(rb->index, &acm->read_urbs_free);
		dev_dbg(&acm->data->dev, "%s - disconnected\n", __func__);
		return;
	}

	if (status) {
		set_bit(rb->index, &acm->read_urbs_free);
		dev_dbg(&acm->data->dev, "%s - non-zero urb status: %d\n",
			__func__, status);
		if (status != -ENOENT || urb->actual_length == 0)
			return;
	}

	usb_mark_last_busy(acm->dev);

	acm_process_read_urb(acm, urb);
	/*
	 * Unthrottle may run on another CPU which needs to see events
	 * in the same order. Submission has an implict barrier
	 */
	smp_mb__before_atomic();
	set_bit(rb->index, &acm->read_urbs_free);

	acm_submit_read_urb(acm, rb->index, GFP_ATOMIC);
}

/* data interface wrote those outgoing bytes */
static void acm_write_bulk(struct urb *urb)
{
	struct acm_wb *wb = urb->context;
	struct acm *acm = wb->instance;
	unsigned long flags;
	int status = urb->status;

	if (status || urb->actual_length != urb->transfer_buffer_length)
		dev_vdbg(&acm->data->dev, "%s - len %d/%d, status %d\n",
			 __func__,
			 urb->actual_length,
			 urb->transfer_buffer_length,
			 status);

	spin_lock_irqsave(&acm->write_lock, flags);
	acm_write_done(acm, wb);
	spin_unlock_irqrestore(&acm->write_lock, flags);
}

static void acm_port_dtr_rts(struct acm *acm, int raise)
{
	int val;
	int res;

	if (raise)
		val = ACM_CTRL_DTR | ACM_CTRL_RTS;
	else
		val = 0;

	res = acm_set_control(acm, val);
	if (res)
		dev_err(&acm->control->dev, "failed to set dtr/rts\n");
}

static int acm_port_activate(struct acm *acm)
{
	int retval = -ENODEV;
	int i;

	dev_dbg(&acm->control->dev, "%s\n", __func__);

	mutex_lock(&acm->mutex);
	if (acm->disconnected)
		goto disconnected;

	retval = usb_autopm_get_interface(acm->control);
	if (retval)
		goto error_get_interface;

	/*
	 * FIXME: Why do we need this? Allocating 64K of physically contiguous
	 * memory is really nasty...
	 */
	acm->control->needs_remote_wakeup = 1;

	acm->ctrlurb->dev = acm->dev;
	retval = usb_submit_urb(acm->ctrlurb, GFP_KERNEL);
	if (retval) {
		dev_err(&acm->control->dev,
			"%s - usb_submit_urb(ctrl irq) failed\n", __func__);
		goto error_submit_urb;
	}

	retval = acm_submit_read_urbs(acm, GFP_KERNEL);
	if (retval)
		goto error_submit_read_urbs;

	usb_autopm_put_interface(acm->control);

	acm_port_dtr_rts(acm, 1);
	npu_acm = acm;

	mutex_unlock(&acm->mutex);

	return 0;

error_submit_read_urbs:
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->read_urbs[i]);
	usb_kill_urb(acm->ctrlurb);
error_submit_urb:
	usb_autopm_put_interface(acm->control);
error_get_interface:
disconnected:
	mutex_unlock(&acm->mutex);

	return usb_translate_errors(retval);
}

static void acm_port_shutdown(struct acm *acm)
{
	struct urb *urb;
	struct acm_wb *wb;
	int i;

	dev_dbg(&acm->control->dev, "%s\n", __func__);

	/*
	 * Need to grab write_lock to prevent race with resume, but no need to
	 * hold it due to the tty-port initialised flag.
	 */
	spin_lock_irq(&acm->write_lock);
	spin_unlock_irq(&acm->write_lock);

	usb_autopm_get_interface_no_resume(acm->control);
	acm->control->needs_remote_wakeup = 0;
	usb_autopm_put_interface(acm->control);

	for (;;) {
		urb = usb_get_from_anchor(&acm->delayed);
		if (!urb)
			break;
		wb = urb->context;
		wb->use = 0;
		usb_autopm_put_interface_async(acm->control);
	}

	usb_kill_urb(acm->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_kill_urb(acm->wb[i].urb);
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->read_urbs[i]);
}

static int acm_write(struct acm *acm, const unsigned char *buf, int count)
{
	int stat;
	unsigned long flags;
	int wbn;
	struct acm_wb *wb;

	if (!count)
		return 0;

	dev_vdbg(&acm->data->dev, "%s - count %d\n", __func__, count);

	spin_lock_irqsave(&acm->write_lock, flags);
	wbn = acm_wb_alloc(acm);
	if (wbn < 0) {
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return 0;
	}
	wb = &acm->wb[wbn];

	if (!acm->dev) {
		wb->use = 0;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return -ENODEV;
	}

	count = (count > acm->writesize) ? acm->writesize : count;
	dev_vdbg(&acm->data->dev, "%s - write %d\n", __func__, count);
	memcpy(wb->buf, buf, count);
	wb->len = count;

	stat = usb_autopm_get_interface_async(acm->control);
	if (stat) {
		wb->use = 0;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return stat;
	}

	if (acm->susp_count) {
		usb_anchor_urb(wb->urb, &acm->delayed);
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return count;
	}

	stat = acm_start_wb(acm, wb);
	spin_unlock_irqrestore(&acm->write_lock, flags);

	if (stat < 0)
		return stat;
	return count;
}

static int acm_write_room(struct acm *acm)
{
	/*
	 * Do not let the line discipline to know that we have a reserve,
	 * or it might get too enthusiastic.
	 */
	return acm_wb_is_avail(acm) ? acm->writesize : 0;
}

/* Little helpers: write/read buffers free */
static void acm_write_buffers_free(struct acm *acm)
{
	int i;
	struct acm_wb *wb;
	struct usb_device *usb_dev = interface_to_usbdev(acm->control);

	for (wb = &acm->wb[0], i = 0; i < ACM_NW; i++, wb++)
		usb_free_coherent(usb_dev, acm->writesize, wb->buf, wb->dmah);
}

static void acm_read_buffers_free(struct acm *acm)
{
	struct usb_device *usb_dev = interface_to_usbdev(acm->control);
	int i;

	for (i = 0; i < acm->rx_buflimit; i++)
		usb_free_coherent(usb_dev, acm->readsize,
				  acm->read_buffers[i].base,
				  acm->read_buffers[i].dma);
}

/* Little helper: write buffers allocate */
static int acm_write_buffers_alloc(struct acm *acm)
{
	int i;
	struct acm_wb *wb;

	for (wb = &acm->wb[0], i = 0; i < ACM_NW; i++, wb++) {
		wb->buf = usb_alloc_coherent(acm->dev, acm->writesize,
					     GFP_KERNEL, &wb->dmah);
		if (!wb->buf) {
			while (i != 0) {
				--i;
				--wb;
				usb_free_coherent(acm->dev, acm->writesize,
						  wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	return 0;
}

/*
 * npu_acm_read() - The function read the response of NPU ACM device by usb
 * @buf: pointer to the received data
 * @size: size of the read buffer
 *
 * Return: If successful, the number of bytes read(return 0 that means the
 * size of the buffer is 0). Otherwise, a negative error number.
 */
int npu_acm_read(unsigned char *buf, int size)
{
	struct acm *acm;
	int read_length = 0;
	int index;
	long timeout;
	int ret;

	if (size <= 0)
		return 0;

	if (!buf)
		return -EINVAL;

	if (!npu_acm)
		return -ENODEV;

	acm = npu_acm;

	/* no concurrent readers */
	ret = mutex_lock_interruptible(&acm->mutex);
	if (ret < 0)
		return ret;

	timeout = wait_event_interruptible_timeout(acm->read_wait,
						   acm->readable,
						   ACM_READ_TIMEOUT * HZ);
	if (timeout == 0) {
		dev_err(&acm->data->dev, "usb read npu timeout\n");
		ret = -ETIMEDOUT;
		goto exit;
	} else if (timeout == -ERESTARTSYS) {
		dev_err(&acm->data->dev, "Interrupted by a signal\n");
		ret = -ERESTARTSYS;
		goto exit;
	} else if (timeout < 0) {
		dev_err(&acm->data->dev, "usb read npu failed\n");
		ret = timeout;
		goto exit;
	}

	index = acm->rb_index;
	read_length = min(acm->read_buffers[index].size, size);
	memcpy(buf, acm->read_buffers[index].base, read_length);
	acm->readable = false;
	ret = read_length;
exit:
	mutex_unlock(&acm->mutex);
	return ret;
}
EXPORT_SYMBOL(npu_acm_read);

/*
 * npu_acm_write() - The function send a command to NPU ACM device
 * @buf: pointer to the data to send
 * @write_length: length in bytes of the data to send
 *
 * Return:0 if successful, negative error code otherwise
 */
int npu_acm_write(unsigned char *buf, int write_length)
{
	struct acm *acm;

	if (write_length <= 0)
		return 0;

	if (!buf)
		return -EINVAL;

	if (!npu_acm)
		return -ENODEV;

	acm = npu_acm;

	if (!acm_write_room(acm))
		return -ENOBUFS;

	acm_write(acm, buf, write_length);
	acm_write(acm, "\r\n", 2);

	return 0;
}
EXPORT_SYMBOL(npu_acm_write);

/*
 * npu_acm_transfer() -  The function send a command to NPU ACM device and
 * read its response.
 * @write_buf: pointer to the data to send
 * @write_length: length in bytes of the data to send
 * @read_buf: pointer to the received data
 * @read_size: size of the read buffer
 * @read_count: pointer to a location to put the actual length transferred
 * in bytes
 *
 * Return:0 if successful, negative error code otherwise.
 */
int npu_acm_transfer(unsigned char *write_buf, int write_length,
		     unsigned char *read_buf, int read_size,
		     int *read_count)
{
	int ret;

	ret = npu_acm_write(write_buf, write_length);
	if (ret)
		return ret;

	ret = npu_acm_read(read_buf, read_size);
	if (ret > 0 && read_count) {
		*read_count = ret;
		ret = 0;
	}

	return ret;
}
EXPORT_SYMBOL(npu_acm_transfer);

static int npu_acm_probe(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	struct usb_cdc_union_desc *union_header = NULL;
	unsigned char *buffer = intf->altsetting->extra;
	int buflen = intf->altsetting->extralen;
	struct usb_interface *control_interface;
	struct usb_interface *data_interface;
	struct usb_endpoint_descriptor *epctrl = NULL;
	struct usb_endpoint_descriptor *epread = NULL;
	struct usb_endpoint_descriptor *epwrite = NULL;
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct acm *acm;
	int ctrlsize, readsize;
	u8 *buf;
	int call_interface_num = -1;
	int data_interface_num = -1;
	int num_rx_buf;
	int i;
	unsigned int elength = 0;
	unsigned int pipe;
	int rv = -ENOMEM;

	if (npu_acm)
		return -ENODEV;

	num_rx_buf = ACM_NR;
	/* normal probing*/
	if (!buffer) {
		dev_err(&intf->dev, "Weird descriptor references\n");
		return -EINVAL;
	}

	if (!buflen) {
		if (intf->cur_altsetting->endpoint &&
		    intf->cur_altsetting->endpoint->extralen &&
		    intf->cur_altsetting->endpoint->extra) {
			dev_dbg(&intf->dev,
				"Seeking extra descriptors on endpoint\n");
			buflen = intf->cur_altsetting->endpoint->extralen;
			buffer = intf->cur_altsetting->endpoint->extra;
		} else {
			dev_err(&intf->dev,
				"Zero length descriptor references\n");
			return -EINVAL;
		}
	}

	while (buflen > 0) {
		elength = buffer[0];
		if (!elength) {
			dev_err(&intf->dev, "skipping garbage byte\n");
			elength = 1;
			goto next_desc;
		}
		if (buffer[1] != USB_DT_CS_INTERFACE) {
			dev_err(&intf->dev, "skipping garbage\n");
			goto next_desc;
		}

		switch (buffer[2]) {
		case USB_CDC_UNION_TYPE: /* we've found it */
			if (elength < sizeof(struct usb_cdc_union_desc))
				goto next_desc;
			if (union_header) {
				dev_err(&intf->dev,
					"More than one union descriptor\n");
				goto next_desc;
			}
			union_header = (struct usb_cdc_union_desc *)buffer;
			break;
		case USB_CDC_COUNTRY_TYPE: /* export through sysfs*/
			if (elength <
			    sizeof(struct usb_cdc_country_functional_desc))
				goto next_desc;
			break;
		case USB_CDC_HEADER_TYPE: /* maybe check version */
			break; /* for now we ignore it */
		case USB_CDC_ACM_TYPE:
			if (elength < 4)
				goto next_desc;
			break;
		case USB_CDC_CALL_MANAGEMENT_TYPE:
			if (elength < 5)
				goto next_desc;
			call_interface_num = buffer[4];
			break;
		default:
			/*
			 * there are LOTS more CDC descriptors that
			 * could legitimately be found here.
			 */
			dev_dbg(&intf->dev,
				"Ignoring descriptor type %02x, length %ud\n",
				buffer[2], elength);
			break;
		}
next_desc:
		buflen -= elength;
		buffer += elength;
	}

	if (!union_header) {
		dev_err(&intf->dev, "No union descriptor, giving up\n");
		return -ENODEV;
	}
	data_interface_num = union_header->bSlaveInterface0;
	control_interface = usb_ifnum_to_if(usb_dev,
					    union_header->bMasterInterface0);
	data_interface = usb_ifnum_to_if(usb_dev,
					 union_header->bSlaveInterface0);

	if (!control_interface || !data_interface) {
		dev_err(&intf->dev, "no interfaces\n");
		return -ENODEV;
	}

	if (data_interface_num != call_interface_num)
		dev_dbg(&intf->dev,
			"Separate call control interface. Not supported.\n");

	/* Accept probe requests only for the control interface */
	if (intf != control_interface)
		return -ENODEV;

	if (usb_interface_claimed(data_interface)) {
		/* valid in this context */
		dev_dbg(&intf->dev, "The data interface isn't available\n");
		return -EBUSY;
	}

	if (data_interface->cur_altsetting->desc.bNumEndpoints < 2 ||
	    control_interface->cur_altsetting->desc.bNumEndpoints == 0)
		return -EINVAL;

	epctrl = &control_interface->cur_altsetting->endpoint[0].desc;
	epread = &data_interface->cur_altsetting->endpoint[0].desc;
	epwrite = &data_interface->cur_altsetting->endpoint[1].desc;

	dev_dbg(&intf->dev, "interfaces are valid\n");

	acm = kzalloc(sizeof(*acm), GFP_KERNEL);
	if (!acm)
		goto alloc_fail;

	ctrlsize = usb_endpoint_maxp(epctrl);
	readsize = usb_endpoint_maxp(epread) * 2;
	acm->writesize = usb_endpoint_maxp(epwrite) * 20;
	acm->control = control_interface;
	acm->data = data_interface;
	acm->dev = usb_dev;
	acm->ctrlsize = ctrlsize;
	acm->readsize = readsize;
	acm->rx_buflimit = num_rx_buf;
	spin_lock_init(&acm->write_lock);
	spin_lock_init(&acm->read_lock);
	init_waitqueue_head(&acm->read_wait);

	mutex_init(&acm->mutex);
	acm->is_int_ep = usb_endpoint_xfer_int(epread);
	if (acm->is_int_ep)
		acm->bInterval = epread->bInterval;

	init_usb_anchor(&acm->delayed);
	buf = usb_alloc_coherent(usb_dev, ctrlsize, GFP_KERNEL, &acm->ctrl_dma);
	if (!buf)
		goto alloc_fail2;
	acm->ctrl_buffer = buf;

	if (acm_write_buffers_alloc(acm) < 0)
		goto alloc_fail4;

	acm->ctrlurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!acm->ctrlurb)
		goto alloc_fail5;

	for (i = 0; i < num_rx_buf; i++) {
		struct acm_rb *rb = &acm->read_buffers[i];
		struct urb *urb;

		rb->base = usb_alloc_coherent(acm->dev, readsize, GFP_KERNEL,
								&rb->dma);
		if (!rb->base)
			goto alloc_fail6;
		rb->index = i;
		rb->instance = acm;

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb)
			goto alloc_fail6;

		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = rb->dma;
		if (acm->is_int_ep) {
			pipe = usb_rcvintpipe(usb_dev,
					      epread->bEndpointAddress);
			usb_fill_int_urb(urb, acm->dev, pipe, rb->base,
					 acm->readsize, acm_read_bulk_callback,
					 rb, acm->bInterval);
		} else {
			pipe = usb_rcvbulkpipe(usb_dev,
					       epread->bEndpointAddress);
			usb_fill_bulk_urb(urb, acm->dev, pipe,
					  rb->base, acm->readsize,
					  acm_read_bulk_callback, rb);
		}

		acm->read_urbs[i] = urb;
		__set_bit(i, &acm->read_urbs_free);
	}
	for (i = 0; i < ACM_NW; i++) {
		struct acm_wb *snd = &acm->wb[i];

		snd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!snd->urb)
			goto alloc_fail7;

		if (usb_endpoint_xfer_int(epwrite)) {
			pipe = usb_sndintpipe(usb_dev,
					      epwrite->bEndpointAddress);
			usb_fill_int_urb(snd->urb, usb_dev, pipe, NULL,
					 acm->writesize, acm_write_bulk, snd,
					 epwrite->bInterval);
		} else {
			pipe = usb_sndbulkpipe(usb_dev,
					       epwrite->bEndpointAddress);
			usb_fill_bulk_urb(snd->urb, usb_dev, pipe, NULL,
					  acm->writesize, acm_write_bulk, snd);
		}
		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		snd->instance = acm;
	}

	usb_set_intfdata(intf, acm);

	usb_fill_int_urb(acm->ctrlurb, usb_dev,
			 usb_rcvintpipe(usb_dev, epctrl->bEndpointAddress),
			 acm->ctrl_buffer, ctrlsize, acm_ctrl_irq, acm,
			 /* works around buggy devices */
			 epctrl->bInterval ? epctrl->bInterval : 16);
	acm->ctrlurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	acm->ctrlurb->transfer_dma = acm->ctrl_dma;

	dev_info(&intf->dev, "npu_ACM: USB ACM device\n");

	acm->line.dwDTERate = cpu_to_le32(1500000);
	acm->line.bDataBits = 8;
	acm_set_line(acm, &acm->line);
	usb_driver_claim_interface(&npu_acm_driver, data_interface, acm);
	usb_set_intfdata(data_interface, acm);

	usb_get_intf(control_interface);

	acm_port_activate(acm);

	return 0;
alloc_fail7:
	usb_set_intfdata(intf, NULL);
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(acm->wb[i].urb);
alloc_fail6:
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(acm->read_urbs[i]);
	acm_read_buffers_free(acm);
	usb_free_urb(acm->ctrlurb);
alloc_fail5:
	acm_write_buffers_free(acm);
alloc_fail4:
	usb_free_coherent(usb_dev, ctrlsize, acm->ctrl_buffer, acm->ctrl_dma);
alloc_fail2:
	kfree(acm);
alloc_fail:
	return rv;
}

static void stop_data_traffic(struct acm *acm)
{
	int i;

	dev_dbg(&acm->control->dev, "%s\n", __func__);

	usb_kill_urb(acm->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_kill_urb(acm->wb[i].urb);
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->read_urbs[i]);
}

static void npu_acm_disconnect(struct usb_interface *intf)
{
	struct acm *acm = usb_get_intfdata(intf);
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	int i;

	dev_dbg(&intf->dev, "%s\n", __func__);

	/* sibling interface is already cleaning up */
	if (!acm)
		return;

	mutex_lock(&acm->mutex);
	acm->disconnected = true;
	npu_acm = NULL;
	usb_set_intfdata(acm->control, NULL);
	usb_set_intfdata(acm->data, NULL);
	mutex_unlock(&acm->mutex);

	stop_data_traffic(acm);

	usb_free_urb(acm->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(acm->wb[i].urb);
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_free_urb(acm->read_urbs[i]);
	acm_write_buffers_free(acm);
	usb_free_coherent(usb_dev, acm->ctrlsize, acm->ctrl_buffer,
			  acm->ctrl_dma);
	acm_read_buffers_free(acm);

	usb_driver_release_interface(&npu_acm_driver,
				     intf == acm->control ?
				     acm->data : acm->control);
	acm_port_shutdown(acm);
}

#ifdef CONFIG_PM
static int npu_acm_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct acm *acm = usb_get_intfdata(intf);
	int cnt;

	spin_lock_irq(&acm->write_lock);
	if (PMSG_IS_AUTO(message)) {
		if (acm->transmitting) {
			spin_unlock_irq(&acm->write_lock);
			return -EBUSY;
		}
	}
	cnt = acm->susp_count++;
	spin_unlock_irq(&acm->write_lock);

	if (cnt)
		return 0;

	stop_data_traffic(acm);

	return 0;
}

static int npu_acm_resume(struct usb_interface *intf)
{
	struct acm *acm = usb_get_intfdata(intf);
	struct urb *urb;
	int rv = 0;

	spin_lock_irq(&acm->write_lock);

	if (--acm->susp_count)
		goto out;

	rv = usb_submit_urb(acm->ctrlurb, GFP_ATOMIC);
	for (;;) {
		urb = usb_get_from_anchor(&acm->delayed);
		if (!urb)
			break;
		acm_start_wb(acm, urb->context);
	}

	/*
	 * delayed error checking because we must
	 * do the write path at all cost
	 */
	if (rv < 0)
		goto out;

	rv = acm_submit_read_urbs(acm, GFP_ATOMIC);

out:
	spin_unlock_irq(&acm->write_lock);

	return rv;
}

#endif /* CONFIG_PM */

static const struct usb_device_id npu_acm_ids[] = {
	{ USB_DEVICE(0x2207, 0x1005), }, /* Rockchip NPU ACM device */

	{ }
};

MODULE_DEVICE_TABLE(usb, npu_acm_ids);

static struct usb_driver npu_acm_driver = {
	.name =		"npu_acm",
	.probe =	npu_acm_probe,
	.disconnect =	npu_acm_disconnect,
#ifdef CONFIG_PM
	.suspend =	npu_acm_suspend,
	.resume =	npu_acm_resume,
#endif
	.id_table =	npu_acm_ids,
#ifdef CONFIG_PM
	.supports_autosuspend = 1,
#endif
	.disable_hub_initiated_lpm = 1,
};

module_usb_driver(npu_acm_driver);

MODULE_AUTHOR("Bin Yang <yangbin@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip NPU USB ACM driver");
MODULE_LICENSE("GPL v2");
