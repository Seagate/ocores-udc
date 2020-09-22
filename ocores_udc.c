// SPDX-License-Identifier: GPL-2.0+
/*
 * OpenCores USB 1.1 Device Controller Driver
 *
 * Copyright (C) 2020 by Seagate Technology, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Note: Some parts of this driver code were based on the Xilinx UDC
 * driver (udc-xilinx.c). Also, a previous version of this same driver,
 * written by David W. Miller <david.w.miller@seagate.com> was used for
 * reference.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/prefetch.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#define DRIVER_NAME	"ocores-udc"
#define DRIVER_DESC	"OpenCores USB Device Controller (UDC) driver"

#define MAX_ENDPOINTS		4
#define EP_NAME_LEN		4
#define MAX_PACKETSZ		64

/* register definitions */
#define SLAVE_BASE			0x100
#define ENDPX_BASE(x)			(SLAVE_BASE + (x) * 0x10)
#define ENDPX_CTRL(x)			(ENDPX_BASE(x) + 0x00)
#define ENDPX_STAT(x)			(ENDPX_BASE(x) + 0x04)
#define ENDPX_TRANSTYPE_STAT(x)		(ENDPX_BASE(x) + 0x08)
#define ENDPX_NAK_TRANSTYPE_STAT(x)	(ENDPX_BASE(x) + 0x0c)

#define SC_CTRL				(SLAVE_BASE + 0x40)
#define SC_LINE_STAT			(SLAVE_BASE + 0x44)
#define SC_INTERRUPT_STAT		(SLAVE_BASE + 0x48)
#define SC_INTERRUPT_MASK		(SLAVE_BASE + 0x4c)
#define SC_ADDRESS			(SLAVE_BASE + 0x50)
#define SC_FRAME_NUM_MSP		(SLAVE_BASE + 0x54)
#define SC_FRAME_NUM_LSP		(SLAVE_BASE + 0x58)

#define EPX_BASE(x)			(SLAVE_BASE + (x) * 0x080 + 0x080)
#define EPX_RX_FIFO_DATA(x)		(EPX_BASE(x) + 0x00)
#define EPX_RX_FIFO_DATA_CNT_MSB(x)	(EPX_BASE(x) + 0x08)
#define EPX_RX_FIFO_DATA_CNT_LSB(x)	(EPX_BASE(x) + 0x0c)
#define EPX_RX_FIFO_CTRL(x)		(EPX_BASE(x) + 0x10)
#define EPX_TX_FIFO_DATA(x)		(EPX_BASE(x) + 0x40)
#define EPX_TX_FIFO_CTRL(x)		(EPX_BASE(x) + 0x50)

#define HOST_SLAVE_CONTROL		(SLAVE_BASE + 0x280)
#define HOST_SLAVE_VERSION		(SLAVE_BASE + 0x284)

/* interrupts */
#define SC_INT_TRANS		BIT(0)
#define SC_INT_RESUME		BIT(1)
#define SC_INT_RESET		BIT(2)
#define SC_INT_SOF		BIT(3)
#define SC_INT_NAK		BIT(4)
#define SC_INT_VBUS		BIT(5)

#define SC_INT_EVENTS		(SC_INT_RESET)
#define SC_INT_ALL		(SC_INT_TRANS | SC_INT_EVENTS)

static const char driver_name[]	= DRIVER_NAME;
static const char ep0name[]	= "ep0";

/* control endpoint descriptor */
static const struct usb_endpoint_descriptor ocores_ep0_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= 0,
	.bmAttributes		= USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize		= cpu_to_le16(MAX_PACKETSZ),
};

enum ocores_trans_type {
	TRANS_TYPE_NONE = 0,
	TRANS_TYPE_SETUP,
	TRANS_TYPE_IN,
	TRANS_TYPE_OUT,
	TRANS_TYPE_ERROR,
};

struct ocores_udc;
struct ocores_ep;

struct ocores_req {
	struct usb_request	ureq;
	struct list_head	queue;
	struct ocores_ep	*ep;
	enum ocores_trans_type	type;
	unsigned		active:1;
};

#define to_ocores_req(req)	container_of((req), struct ocores_req, ureq)

struct ocores_ep {
	struct usb_ep		uep;
	struct list_head	queue;
	struct ocores_udc	*udc;
	const struct usb_endpoint_descriptor *desc;
	u16			epnum;
	char			name[EP_NAME_LEN];
	unsigned		is_in:1;
};

#define to_ocores_ep(e)	container_of((e), struct ocores_ep, uep)

struct ocores_udc {
	struct usb_gadget	gadget;
	struct ocores_ep	ep[MAX_ENDPOINTS];
	struct usb_gadget_driver *driver;
	void __iomem		*ioaddr;
	struct device		*dev;
	/* general lock */
	spinlock_t		lock;
	/* pull-up quirks */
	bool			broken_pullup;
	int			pullup_pin;
	/* control endpoint only */
	struct ocores_req	*req;
	struct usb_ctrlrequest	setup;
};

#define to_ocores_udc(g)	container_of((g), struct ocores_udc, gadget)

/* ------------------------------------------------------------------------- */
/* register helper functions */

static inline u32 ocores_udc_read(struct ocores_udc *udc, u32 reg)
{
	return readb(udc->ioaddr + reg);
}

static inline void ocores_udc_write(struct ocores_udc *udc, u32 reg, u32 val)
{
	u8 v = (u8)(val & 0xFF);

	writeb(v, udc->ioaddr + reg);
}

static inline void ocores_udc_update_bits(struct ocores_udc *udc, u32 reg,
					  u32 mask, u32 val)
{
	u32 tmp;

	tmp = ocores_udc_read(udc, reg);
	tmp &= ~mask;
	tmp |= (val & mask);
	ocores_udc_write(udc, reg, tmp);
}

static inline void ocores_udc_set_bits(struct ocores_udc *udc, u32 reg, u32 mask)
{
	ocores_udc_update_bits(udc, reg, mask, mask);
}

static inline void ocores_udc_clr_bits(struct ocores_udc *udc, u32 reg, u32 mask)
{
	ocores_udc_update_bits(udc, reg, mask, 0);
}

/* ------------------------------------------------------------------------- */
/* ep state helper functions */

static bool ocores_ep_enabled(struct ocores_ep *ep)
{
	struct ocores_udc *udc = ep->udc;
	int x = ep->epnum;

	/* check Enable bit */
	return (ocores_udc_read(udc, ENDPX_CTRL(x)) & BIT(0)) != 0;
}

static bool ocores_ep_pending(struct ocores_ep *ep)
{
	struct ocores_udc *udc = ep->udc;
	int x = ep->epnum;

	/* check Ready bit */
	return (ocores_udc_read(udc, ENDPX_CTRL(x)) & BIT(1)) != 0;
}

static void ocores_ep_ready(struct ocores_ep *ep)
{
	struct ocores_udc *udc = ep->udc;
	int x = ep->epnum;

	/* set Ready bit */
	ocores_udc_set_bits(udc, ENDPX_CTRL(x), BIT(1));
}

static bool ocores_ep_is_toggled(struct ocores_ep *ep)
{
	struct ocores_udc *udc = ep->udc;
	int x = ep->epnum;

	return (ocores_udc_read(udc, ENDPX_CTRL(x)) & BIT(2)) != 0;
}

static void ocores_ep_toggle(struct ocores_ep *ep, int data1)
{
	struct ocores_udc *udc = ep->udc;
	int x = ep->epnum;

	/* if < 0, just toggle the bit */
	if (data1 < 0) {
		bool curr = ocores_ep_is_toggled(ep);
		data1 = curr ? 0 : 1;
	}

	/* set/clear Outdata Sequence bit */
	if (data1)
		ocores_udc_set_bits(udc, ENDPX_CTRL(x), BIT(2));
	else
		ocores_udc_clr_bits(udc, ENDPX_CTRL(x), BIT(2));
}

static bool ocores_ep_stalled(struct ocores_ep *ep)
{
	struct ocores_udc *udc = ep->udc;
	int x = ep->epnum;

	/* check Stall bit */
	return (ocores_udc_read(udc, ENDPX_CTRL(x)) & BIT(3)) != 0;
}

static void ocores_ep_stall(struct ocores_ep *ep)
{
	struct ocores_udc *udc = ep->udc;

	ocores_udc_set_bits(udc, ENDPX_CTRL(ep->epnum), BIT(3));
}

static void ocores_ep_unstall(struct ocores_ep *ep)
{
	struct ocores_udc *udc = ep->udc;

	ocores_udc_clr_bits(udc, ENDPX_CTRL(ep->epnum), BIT(3));
}

/* ------------------------------------------------------------------------- */
/* ep fifo helper functions */

static void ocores_ep_rx_flush(struct ocores_ep *ep)
{
	struct ocores_udc *udc = ep->udc;

	ocores_udc_write(udc, EPX_RX_FIFO_CTRL(ep->epnum), 1);
}

static void ocores_ep_tx_flush(struct ocores_ep *ep)
{
	struct ocores_udc *udc = ep->udc;

	ocores_udc_write(udc, EPX_TX_FIFO_CTRL(ep->epnum), 1);
}

static u32 ocores_ep_rx_count(struct ocores_ep *ep)
{
	struct ocores_udc *udc = ep->udc;
	int x = ep->epnum;
	u32 count;

	count  = (ocores_udc_read(udc, EPX_RX_FIFO_DATA_CNT_MSB(x)) & 0xFF) << 8;
	count |= (ocores_udc_read(udc, EPX_RX_FIFO_DATA_CNT_LSB(x)) & 0xFF);

	return count;
}

static int ocores_ep_rx(struct ocores_ep *ep, u8 *buf, u32 len)
{
	struct ocores_udc *udc = ep->udc;
	u32 count;
	u32 i, x;

	x = ep->epnum;

	/* make sure endpoint is done with prev transaction */
	if (ocores_ep_pending(ep)) {
		dev_dbg(udc->dev, "%s attempted rx whilst busy\n", ep->name);
		return -EAGAIN;
	}

	count = ocores_ep_rx_count(ep);
	len = min(len, count);

	for (i = 0; i < len; i++)
		buf[i] = (u8)(ocores_udc_read(udc, EPX_RX_FIFO_DATA(x)) & 0xFF);

	return 0;
}

static int ocores_ep_tx(struct ocores_ep *ep, u8 *buf, u32 len)
{
	struct ocores_udc *udc = ep->udc;
	u32 i, x;

	x = ep->epnum;

	if (ocores_ep_pending(ep)) {
		dev_dbg(udc->dev, "%s attempted tx whilst busy\n", ep->name);
		return -EAGAIN;
	}

	for (i = 0; i < len; i++)
		ocores_udc_write(udc, EPX_TX_FIFO_DATA(x), buf[i]);

	return 0;
}

/* ------------------------------------------------------------------------- */
/* request handling */

static int ocores_ep_queue(struct ocores_ep *ep, struct ocores_req *req);
static void ocores_ep_nuke(struct ocores_ep *ep, int status);
static void ocores_ep_process(struct ocores_ep *ep);

static void ocores_req_ready(struct ocores_req *req)
{
	struct ocores_ep *ep = req->ep;

	if (req->type != TRANS_TYPE_IN)
		ocores_ep_rx_flush(ep);

	req->active = true;
	ocores_ep_ready(ep);
}

static int __setup_get_status(struct ocores_req *req)
{
	struct ocores_ep  *ep0 = req->ep;
	struct ocores_udc *udc = ep0->udc;
	struct ocores_ep  *target_ep;
	u16 status = 0;
	int epnum;

	switch (udc->setup.bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		/* get device status */
		status = 1 << USB_DEVICE_SELF_POWERED;
		break;
	case USB_RECIP_INTERFACE:
		break;
	case USB_RECIP_ENDPOINT:
		epnum = udc->setup.wIndex & USB_ENDPOINT_NUMBER_MASK;
		target_ep = &udc->ep[epnum];

		if (ocores_ep_stalled(target_ep))
			status = 1 << USB_ENDPOINT_HALT;

		if (!epnum)
			break;

		if (udc->setup.wIndex & USB_DIR_IN) {
			if (!target_ep->is_in)
				return -EINVAL;
		} else {
			if (target_ep->is_in)
				return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	req->type = TRANS_TYPE_IN;
	req->ureq.length = 2;
	*(u16 *)req->ureq.buf = cpu_to_le16(status);

	return ocores_ep_queue(ep0, req);
}

static int __setup_set_address(struct ocores_req *req)
{
	struct ocores_ep *ep0 = req->ep;

	req->type = TRANS_TYPE_IN;
	req->ureq.length = 0;

	return ocores_ep_queue(ep0, req);
}

static int __setup_set_clr_feature(struct ocores_req *req)
{
	struct ocores_ep  *ep0 = req->ep;
	struct ocores_udc *udc = ep0->udc;
	struct ocores_ep  *target_ep;
	u8 epnum;
	u8 outinbit;
	bool is_set = (udc->setup.bRequest == USB_REQ_SET_FEATURE);

	switch (udc->setup.bRequestType) {
	case USB_RECIP_DEVICE:
		switch (udc->setup.wValue) {
		case USB_DEVICE_TEST_MODE:
		case USB_DEVICE_REMOTE_WAKEUP:
			/*
			 * ignore until we figure out how to handle
			 */
			break;
		default:
			return -EINVAL;
		}
		break;
	case USB_RECIP_ENDPOINT:
		if (!udc->setup.wValue) {
			epnum = udc->setup.wIndex & USB_ENDPOINT_NUMBER_MASK;
			target_ep = &udc->ep[epnum];

			outinbit = udc->setup.wIndex & USB_ENDPOINT_DIR_MASK;
			outinbit = outinbit >> 7;

			/* make sure direction matches */
			if (outinbit != target_ep->is_in)
				return -EINVAL;

			if (!epnum) {
				ocores_ep_unstall(ep0);
			} else {
				if (is_set)
					ocores_ep_stall(target_ep);
				else
					ocores_ep_unstall(target_ep);
			}
		}
		break;
	default:
		return -EINVAL;
	}

	req->type = TRANS_TYPE_IN;
	req->ureq.length = 0;

	return ocores_ep_queue(ep0, req);
}

static int ocores_req_handle_setup(struct ocores_req *req)
{
	struct ocores_ep  *ep  = req->ep;
	struct ocores_udc *udc = ep->udc;
	struct usb_ctrlrequest *setup;
	int rc;

	/* get setup packet from req buffer */
	setup = (struct usb_ctrlrequest *)req->ureq.buf;
	udc->setup = *setup;
	udc->setup.wValue = cpu_to_le16(setup->wValue);
	udc->setup.wIndex = cpu_to_le16(setup->wIndex);
	udc->setup.wLength = cpu_to_le16(setup->wLength);

	/* clear pending requests (if any) */
	ocores_ep_nuke(ep, -ECONNRESET);

	/* next IN packet uses DATA1 */
	ocores_ep_toggle(ep, 1);

	switch (udc->setup.bRequest) {
	case USB_REQ_GET_STATUS:
		if ((udc->setup.bRequestType &
				(USB_DIR_IN | USB_TYPE_MASK)) !=
				(USB_DIR_IN | USB_TYPE_STANDARD))
			break;
		return __setup_get_status(req);
	case USB_REQ_SET_ADDRESS:
		if (udc->setup.bRequestType != (USB_DIR_OUT |
				USB_TYPE_STANDARD | USB_RECIP_DEVICE))
			break;
		return __setup_set_address(req);
	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
		if ((udc->setup.bRequestType & USB_TYPE_MASK)
				!= USB_TYPE_STANDARD)
			break;
		return __setup_set_clr_feature(req);
	default:
		break;
	}

	spin_unlock(&udc->lock);
	rc = udc->driver->setup(&udc->gadget, setup);
	spin_lock(&udc->lock);

	return rc;
}

static void ocores_req_done(struct ocores_req *req, int status)
{
	struct ocores_ep  *ep  = req->ep;
	struct ocores_udc *udc = ep->udc;
	unsigned int length = req->ureq.length;
	enum ocores_trans_type type = req->type;
	int rc;

	list_del_init(&req->queue);
	req->active = false;

	dev_vdbg(udc->dev, "%s done %p, status %d\n",
		 ep->uep.name, req, status);

	if (req->ureq.status == -EINPROGRESS)
		req->ureq.status = status;
	else
		status = req->ureq.status;

	if (req->ureq.complete) {
		spin_unlock(&udc->lock);
		usb_gadget_giveback_request(&ep->uep, &req->ureq);
		spin_lock(&udc->lock);
	}

	/* for error status, stop here */
	if (status)
		return;

	/* check if endpoint needs extra work */
	if (ep->epnum) {
		if (!list_empty(&ep->queue))
			ocores_ep_process(ep);
		return;
	}

	/* ctrl endpoint (ep0) may need extra work */
	switch (type) {
	case TRANS_TYPE_SETUP:
		/* handle setup */
		rc = ocores_req_handle_setup(req);
		if (rc) {
			ocores_ep_stall(ep);
			ocores_ep_ready(ep);
			/* requeue setup request */
			ocores_ep_queue(ep, req);
		}
		break;
	case TRANS_TYPE_OUT:
		if (!length) {
			/* end of status phase */
			udc->req->type = TRANS_TYPE_SETUP;
			udc->req->ureq.length = 8;
			ocores_ep_queue(ep, udc->req);

		} else if (list_empty(&ep->queue)) {
			/* end of data phase */
			ocores_ep_toggle(ep, 1);
			udc->req->type = TRANS_TYPE_IN;
			udc->req->ureq.length = 0;
			ocores_ep_queue(ep, udc->req);
		}
		break;
	case TRANS_TYPE_IN:
		if (!length) {
			/* end of status phase */
			if (udc->setup.bRequest == USB_REQ_SET_ADDRESS)
				ocores_udc_write(udc, SC_ADDRESS,
					      udc->setup.wValue);

			/* queue setup request */
			udc->req->type = TRANS_TYPE_SETUP;
			udc->req->ureq.length = 8;
			ocores_ep_queue(ep, udc->req);

		} else if (list_empty(&ep->queue)) {
			/* end of data phase */
			ocores_ep_toggle(ep, 1);
			udc->req->type = TRANS_TYPE_OUT;
			udc->req->ureq.length = 0;
			ocores_ep_queue(ep, udc->req);
		}
		break;
	default:
		dev_dbg(udc->dev, "invalid request type %u\n", req->type);
	}
}

static void ocores_ep_nuke(struct ocores_ep *ep, int status)
{
	struct ocores_req *req;

	/* clear stall state */
	ocores_ep_unstall(ep);

	/* clear toggle state */
	ocores_ep_toggle(ep, 0);

	/* drop all pending requests */
	while (!list_empty(&ep->queue)) {
		req = list_first_entry(&ep->queue, struct ocores_req, queue);
		ocores_req_done(req, status);
	}
}

static void ocores_req_read_fifo(struct ocores_req *req)
{
	struct ocores_ep  *ep  = req->ep;
	struct ocores_udc *udc = ep->udc;
	u32 len, count;
	u8 *buf;
	int x, rc;
	bool is_short;

	x = ep->epnum;

	count = ocores_ep_rx_count(ep);

	buf = req->ureq.buf + req->ureq.actual;
	prefetchw(buf);
	len = req->ureq.length - req->ureq.actual;
	is_short = count < ep->uep.maxpacket;

	len = min(len, count);

	rc = ocores_ep_rx(ep, buf, len);
	switch (rc) {
	case 0:
		req->ureq.actual += len;

		dev_vdbg(udc->dev, "read %s, %d bytes%s req %p %d/%d\n",
			 ep->uep.name, count, is_short ? "/S" : "", req,
			 req->ureq.actual, req->ureq.length);

		if ((req->ureq.actual != req->ureq.length) && !is_short)
			rc = -EINPROGRESS;
		break;
	case -EAGAIN:
		dev_dbg(udc->dev, "%s rx busy\n", ep->uep.name);
		rc = -EINPROGRESS;
		break;
	default:
		/* any other error, dequeue request */
		rc = -ECONNRESET;
		break;
	}

	req->ureq.status = rc;
}

static void ocores_req_write_fifo(struct ocores_req *req)
{
	struct ocores_ep  *ep  = req->ep;
	struct ocores_udc *udc = ep->udc;
	int is_last, is_short = 0;
	u32 len, maxp;
	u8 *buf;
	int rc;

	maxp = usb_endpoint_maxp(ep->desc);
	buf  = req->ureq.buf + req->ureq.actual;
	prefetch(buf);
	len = req->ureq.length - req->ureq.actual;
	len = min(len, maxp);

	ocores_ep_tx_flush(ep);

	rc = ocores_ep_tx(ep, buf, len);
	switch (rc) {
	case 0:
		/* send data */
		ocores_req_ready(req);

		req->ureq.actual += len;
		if (unlikely(len != maxp)) {
			is_last = is_short = 1;
		} else {
			if (likely(req->ureq.length != req->ureq.actual))
				is_last = 0;
			else
				is_last = 1;
		}
		dev_vdbg(udc->dev, "wrote %s %d bytes%s%s %d left %p\n",
			 ep->uep.name, len, is_last ? "/L" : "",
			 is_short ? "/S" : "",
			 req->ureq.length - req->ureq.actual, req);

		if (!is_last)
			rc = -EINPROGRESS;
		break;
	case -EAGAIN:
		dev_dbg(udc->dev, "%s tx busy\n", ep->uep.name);
		rc = -EINPROGRESS;
		break;
	default:
		/* any other error, dequeue the request */
		rc = -ECONNRESET;
		break;
	}

	req->ureq.status = rc;
}

static void ocores_ep_process(struct ocores_ep *ep)
{
	struct ocores_udc *udc = ep->udc;
	struct ocores_req *req;

	if (ocores_ep_pending(ep))
		return;

	if (ocores_ep_stalled(ep)) {
		if (ep->epnum) {
			ocores_ep_ready(ep);
			return;
		}
		ocores_ep_unstall(ep);
	}

	if (list_empty(&ep->queue))
		return;

	req = list_first_entry(&ep->queue, struct ocores_req, queue);

	/* check if this is a zero length (status) request */
	if (!req->ureq.length) {
		if (!req->active)
			ocores_req_ready(req);
		else
			ocores_req_done(req, 0);
		return;
	}

	/* non-empty requests */
	switch (req->type) {
	case TRANS_TYPE_SETUP:
	case TRANS_TYPE_OUT:
		if (!req->active)
			return ocores_req_ready(req);

		ocores_req_read_fifo(req);
		if (req->ureq.status == -EINPROGRESS)
			ocores_req_ready(req);
		else
			ocores_req_done(req, 0);
		break;
	case TRANS_TYPE_IN:
		if (req->active)
			ocores_ep_toggle(ep, -1);
		else
			req->active = true;

		if (req->ureq.status == -EINPROGRESS)
			ocores_req_write_fifo(req);
		else
			ocores_req_done(req, 0);
		break;
	default:
		/* invalid request type, discard */
		dev_err(udc->dev, "invalid request type %u\n", req->type);
		ocores_req_done(req, -ECONNRESET);
	}
}

/* ------------------------------------------------------------------------- */
/* endpoint operations */

static int ocores_ep_enable(struct ocores_ep *ep,
			    const struct usb_endpoint_descriptor *desc)
{
	struct ocores_udc *udc = ep->udc;
	int maxp;

	ep->is_in = usb_endpoint_dir_in(desc);
	ep->epnum = usb_endpoint_num(desc);
	ep->desc = desc;
	ep->uep.desc = desc;
	maxp = usb_endpoint_maxp(desc);
	ep->uep.maxpacket = maxp;

	switch (usb_endpoint_type(desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		if (ep->epnum != 0) {
			dev_dbg(udc->dev, "only one control endpoint\n");
			return -EINVAL;
		}
		break;
	case USB_ENDPOINT_XFER_INT:
		if (maxp > MAX_PACKETSZ) {
			dev_dbg(udc->dev, "bogus maxpacket %d\n", maxp);
			return -EINVAL;
		}
		break;
	case USB_ENDPOINT_XFER_BULK:
		if (!(is_power_of_2(maxp) && maxp >= 8 &&
		    maxp <= MAX_PACKETSZ)) {
			dev_dbg(udc->dev, "bogus maxpacket %d\n", maxp);
			return -EINVAL;
		}
		break;
	case USB_ENDPOINT_XFER_ISOC:
		/* ISO not supported for now */
		dev_dbg(udc->dev, "isochronous not supported\n");
		return -EINVAL;
	}

	/* enable endpoint */
	ocores_udc_write(udc, ENDPX_CTRL(ep->epnum), BIT(0));

	return 0;
}

static int ocores_uep_enable(struct usb_ep *uep,
			     const struct usb_endpoint_descriptor *desc)
{
	struct ocores_ep *ep;
	struct ocores_udc *udc;
	unsigned long flags;
	int rc;

	if (!uep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		pr_debug("%s: bad ep or descriptor\n", __func__);
		return -EINVAL;
	}

	ep = to_ocores_ep(uep);
	udc = ep->udc;

	dev_dbg(udc->dev, "enable ep%u for %s\n", (u32)ep->epnum,
		ep->is_in ? "IN" : "OUT");

	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		dev_dbg(udc->dev, "bogus device state\n");
		return -ESHUTDOWN;
	}

	spin_lock_irqsave(&udc->lock, flags);
	rc = ocores_ep_enable(ep, desc);
	spin_unlock_irqrestore(&udc->lock, flags);

	return rc;
}

static int ocores_uep_disable(struct usb_ep *uep)
{
	struct ocores_ep *ep;
	struct ocores_udc *udc;
	unsigned long flags;

	if (!uep) {
		pr_debug("%s: invalid ep\n", __func__);
		return -EINVAL;
	}

	ep = to_ocores_ep(uep);
	udc = ep->udc;

	dev_dbg(udc->dev, "disable ep%u\n", (u32)ep->epnum);

	spin_lock_irqsave(&udc->lock, flags);

	ocores_ep_nuke(ep, -ESHUTDOWN);

	/* restore the endpoint's pristine config */
	ep->desc = NULL;
	ep->uep.desc = NULL;

	/* disable the endpoint */
	ocores_udc_clr_bits(udc, ENDPX_CTRL(ep->epnum), BIT(0));

	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

static struct usb_request *ocores_uep_alloc_request(struct usb_ep *uep,
						    gfp_t gfp_flags)
{
	struct ocores_ep *ep = to_ocores_ep(uep);
	struct ocores_req *req;

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		return NULL;

	req->ep = ep;
	INIT_LIST_HEAD(&req->queue);
	return &req->ureq;
}

static void ocores_uep_free_request(struct usb_ep *uep,
				    struct usb_request *ureq)
{
	struct ocores_req *req = to_ocores_req(ureq);

	kfree(req);
}

static int ocores_ep_queue(struct ocores_ep *ep, struct ocores_req *req)
{
	bool is_first = list_empty(&ep->queue);

	req->ureq.status = -EINPROGRESS;
	req->ureq.actual = 0;
	req->active = false;

	list_add_tail(&req->queue, &ep->queue);

	if (is_first)
		ocores_ep_process(ep);

	return 0;
}

static int ocores_uep_queue(struct usb_ep *uep, struct usb_request *ureq,
			    gfp_t gfp_flags)
{
	struct ocores_req *req = to_ocores_req(ureq);
	struct ocores_ep  *ep  = to_ocores_ep(uep);
	struct ocores_udc *udc = ep->udc;
	unsigned long flags;
	int rc;

	if (!ep->desc) {
		dev_dbg(udc->dev, "%s: queuing request to disabled %s\n",
			__func__, ep->name);
		return -ESHUTDOWN;
	}

	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		dev_dbg(udc->dev, "%s, bogus device state\n", __func__);
		return -EINVAL;
	}

	if (ep->epnum == 0)
		ep->is_in = !!(udc->setup.bRequestType & USB_DIR_IN);

	if (ep->is_in)
		req->type = TRANS_TYPE_IN;
	else
		req->type = TRANS_TYPE_OUT;

	spin_lock_irqsave(&udc->lock, flags);
	rc = ocores_ep_queue(ep, req);
	spin_unlock_irqrestore(&udc->lock, flags);

	return rc;
}

static int ocores_uep_dequeue(struct usb_ep *uep, struct usb_request *ureq)
{
	struct ocores_ep *ep   = to_ocores_ep(uep);
	struct ocores_req *req = to_ocores_req(ureq);
	struct ocores_udc *udc = ep->udc;
	unsigned long flags;

	spin_lock_irqsave(&udc->lock, flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->ureq == ureq)
			break;
	}

	if (&req->ureq != ureq) {
		spin_unlock_irqrestore(&udc->lock, flags);
		return -EINVAL;
	}

	ocores_req_done(req, -ECONNRESET);

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int ocores_uep_set_halt(struct usb_ep *uep, int value)
{
	struct ocores_ep *ep = to_ocores_ep(uep);
	struct ocores_udc *udc;
	unsigned long flags;

	if (!uep || (!ep->desc && ep->epnum)) {
		pr_debug("%s: bad ep or descriptor\n", __func__);
		return -EINVAL;
	}

	udc = ep->udc;

	dev_dbg(udc->dev, "%s halt set to %d\n", ep->name, value);

	spin_lock_irqsave(&udc->lock, flags);

	if (ep->is_in && (!list_empty(&ep->queue)) && value) {
		dev_dbg(udc->dev, "can't halt %s while busy\n", ep->name);
		spin_unlock_irqrestore(&udc->lock, flags);
		return -EAGAIN;
	}

	if (value)
		ocores_ep_stall(ep);
	else
		ocores_ep_unstall(ep);

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

int ocores_uep_fifo_status(struct usb_ep *uep)
{
	struct ocores_ep *ep;
	struct ocores_udc *udc;

	if (!uep)
		return -ENODEV;

	ep = to_ocores_ep(uep);
	udc = ep->udc;

	if (!ep || !ep->epnum)
		return -ENODEV;

	if (ep->is_in)
		return -EOPNOTSUPP;

	if (udc->gadget.speed == USB_SPEED_UNKNOWN)
		return 0;

	return ocores_ep_rx_count(ep);
}

void ocores_uep_fifo_flush(struct usb_ep *uep)
{
	struct ocores_ep *ep;
	struct ocores_udc *udc;
	unsigned long flags;

	if (!uep)
		return;

	ep = to_ocores_ep(uep);
	udc = ep->udc;

	if (!ep || !ep->epnum)
		return;

	spin_lock_irqsave(&udc->lock, flags);

	if (ep->is_in)
		ocores_ep_tx_flush(ep);
	else
		ocores_ep_rx_flush(ep);

	spin_unlock_irqrestore(&udc->lock, flags);
}

static const struct usb_ep_ops ocores_ep_ops = {
	.enable		= ocores_uep_enable,
	.disable	= ocores_uep_disable,
	.alloc_request	= ocores_uep_alloc_request,
	.free_request	= ocores_uep_free_request,
	.queue		= ocores_uep_queue,
	.dequeue	= ocores_uep_dequeue,
	.set_halt	= ocores_uep_set_halt,
	.fifo_status	= ocores_uep_fifo_status,
	.fifo_flush	= ocores_uep_fifo_flush,
};

/* ------------------------------------------------------------------------- */
/* gadget helper functions */

static void ocores_udc_hw_reset(struct ocores_udc *udc)
{
	/* According to the manual, we need to wait 10
	 * usbClk cycles for reset to complete. Since
	 * usbClk = 48MHz, 1 cycle is ~208us, so let's
	 * wait 3ms just to be sure.
	 */
	ocores_udc_write(udc, HOST_SLAVE_CONTROL, 0x02);
	msleep(3);
}

static void ocores_udc_init_eps(struct ocores_udc *udc)
{
	u32 i;

	INIT_LIST_HEAD(&udc->gadget.ep_list);

	for (i = 0; i < MAX_ENDPOINTS; i++) {
		struct ocores_ep *ep = &udc->ep[i];

		if (i) {
			list_add_tail(&ep->uep.ep_list, &udc->gadget.ep_list);
			snprintf(ep->name, EP_NAME_LEN, "ep%d", i);
			ep->uep.name = ep->name;

			ep->uep.caps.type_iso = false;
			ep->uep.caps.type_bulk = true;
			ep->uep.caps.type_int = true;
		} else {
			snprintf(ep->name, EP_NAME_LEN, ep0name);
			ep->uep.name = ep0name;
			ep->uep.caps.type_control = true;
		}

		ep->uep.ops = &ocores_ep_ops;
		usb_ep_set_maxpacket_limit(&ep->uep, MAX_PACKETSZ);
		ep->uep.caps.dir_in = true;
		ep->uep.caps.dir_out = true;

		ep->udc    = udc;
		ep->epnum  = i;
		ep->desc   = NULL;
		ep->is_in  = false;

		/* initialize one queue per endpoint */
		INIT_LIST_HEAD(&ep->queue);
	}
}

static void ocores_udc_enable(struct ocores_udc *udc)
{
	/* enable usb device at full-speed */
	ocores_udc_set_bits(udc, SC_CTRL, 0x31);
}

static void ocores_udc_disable(struct ocores_udc *udc)
{
	/* disable usb device */
	ocores_udc_clr_bits(udc, SC_CTRL, BIT(0));
}

static void ocores_udc_connect(struct ocores_udc *udc)
{
	/* enable D+ pull-up */
	ocores_udc_set_bits(udc, SC_CTRL, BIT(6));
}

static void ocores_udc_disconnect(struct ocores_udc *udc)
{
	/* disable D+ pull-up */
	ocores_udc_clr_bits(udc, SC_CTRL, BIT(6));
}

static void ocores_udc_enable_events(struct ocores_udc *udc)
{
	ocores_udc_set_bits(udc, SC_INTERRUPT_MASK, SC_INT_EVENTS);
}

static void ocores_udc_disable_events(struct ocores_udc *udc)
{
	ocores_udc_clr_bits(udc, SC_INTERRUPT_MASK, SC_INT_EVENTS);
}

static void ocores_udc_enable_interrupts(struct ocores_udc *udc)
{
	ocores_udc_write(udc, SC_INTERRUPT_STAT, 0xFF);
	ocores_udc_write(udc, SC_INTERRUPT_MASK, SC_INT_ALL);
}

static void ocores_udc_disable_interrupts(struct ocores_udc *udc)
{
	ocores_udc_write(udc, SC_INTERRUPT_MASK, 0x00);
	ocores_udc_write(udc, SC_INTERRUPT_STAT, 0xFF);
}

static void ocores_udc_stop_activity(struct ocores_udc *udc, int status)
{
	struct ocores_ep *ep;
	int i;

	for (i = 0; i < MAX_ENDPOINTS; i++) {
		ep = &udc->ep[i];
		ocores_ep_nuke(ep, status);
	}
}

static void ocores_udc_usb_reset(struct ocores_udc *udc)
{
	udc->gadget.speed = udc->gadget.max_speed;

	/* stop all endpoints */
	ocores_udc_stop_activity(udc, -ECONNRESET);

	/* set device address to 0 */
	ocores_udc_write(udc, SC_ADDRESS, 0x00);
}

/* ------------------------------------------------------------------------- */
/* gadget operations */

static int ocores_udc_get_frame(struct usb_gadget *gadget)
{
	struct ocores_udc *udc;
	int frame;

	if (!gadget)
		return -ENODEV;

	udc = to_ocores_udc(gadget);

	frame = (ocores_udc_read(udc, SC_FRAME_NUM_MSP) << 8) |
		ocores_udc_read(udc, SC_FRAME_NUM_LSP);

	return frame;
}

static int ocores_udc_pullup(struct usb_gadget *gadget, int is_on)
{
	struct ocores_udc *udc = to_ocores_udc(gadget);
	unsigned long flags;

	dev_dbg(udc->dev, "pullup %s\n", is_on ? "on" : "off");

	spin_lock_irqsave(&udc->lock, flags);

	if (is_on)
		ocores_udc_connect(udc);
	else
		ocores_udc_disconnect(udc);

	/* if gpio provided, use it to toggle pull-up */
	if (gpio_is_valid(udc->pullup_pin))
		gpio_set_value(udc->pullup_pin, is_on);

	/* quirk for an always-on pull-up */
	if (udc->broken_pullup) {
		dev_dbg(udc->dev, "broken pullup\n");
		/*
		 * if the pull-up is fixed, we need to manually bounce the
		 * D+ line to make it look like we're toggling the pull-up
		 */
		ocores_udc_set_bits(udc, SC_CTRL, BIT(3));
		if (is_on) {
			udelay(1000);
			ocores_udc_clr_bits(udc, SC_CTRL, BIT(3));
		}
	}

	if (!is_on) {
		/* clear any pending interrupts */
		ocores_udc_write(udc, SC_INTERRUPT_STAT, 0xFF);
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int ocores_udc_start(struct usb_gadget *gadget,
			 struct usb_gadget_driver *driver)
{
	struct ocores_udc *udc = to_ocores_udc(gadget);
	struct ocores_ep  *ep0 = &udc->ep[0];
	unsigned long flags;
	int rc;

	dev_dbg(udc->dev, "udc_start\n");

	spin_lock_irqsave(&udc->lock, flags);

	if (udc->driver) {
		dev_err(udc->dev, "%s is already bound to %s\n",
			udc->gadget.name, udc->driver->driver.name);
		rc = -EBUSY;
		goto out;
	}

	/* hook up the driver */
	udc->driver = driver;

	/* clean state */
	ocores_udc_usb_reset(udc);

	ocores_udc_enable(udc);

	rc = ocores_ep_enable(ep0, &ocores_ep0_desc);
	if (rc) {
		dev_err(udc->dev, "failed to enable endopoint 0: %d", rc);
		goto out;
	}

	ocores_udc_enable_interrupts(udc);

	/* queue setup request */
	udc->req->type = TRANS_TYPE_SETUP;
	udc->req->ureq.length = 8;
	ocores_ep_queue(&udc->ep[0], udc->req);
out:
	spin_unlock_irqrestore(&udc->lock, flags);
	return rc;
}

static int ocores_udc_stop(struct usb_gadget *gadget)
{
	struct ocores_udc *udc = to_ocores_udc(gadget);
	unsigned long flags;

	dev_dbg(udc->dev, "udc_stop\n");

	spin_lock_irqsave(&udc->lock, flags);

	ocores_udc_disable_interrupts(udc);

	ocores_udc_disable(udc);

	ocores_udc_stop_activity(udc, -ESHUTDOWN);

	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->driver = NULL;

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static const struct usb_gadget_ops ocores_udc_ops = {
	.get_frame	= ocores_udc_get_frame,
	.pullup		= ocores_udc_pullup,
	.udc_start	= ocores_udc_start,
	.udc_stop	= ocores_udc_stop,
};

/* ------------------------------------------------------------------------- */
/* interrupt handling */

static void ocores_udc_trans_handler(struct ocores_udc *udc, u32 *status)
{
	struct ocores_ep *ep;
	int i;

	*status &= ~SC_INT_TRANS;

	for (i = 0; i < MAX_ENDPOINTS; i++) {
		ep = &udc->ep[i];

		if (!ocores_ep_enabled(ep))
			return;

		ocores_ep_process(ep);
	}
}

static void ocores_udc_event_handler(struct ocores_udc *udc, u32 *status)
{
	if (*status & SC_INT_SOF) {
		/* NOP for now */
		*status &= ~SC_INT_SOF;
	}

	if (*status & SC_INT_RESET) {
		struct ocores_req *req = udc->req;
		struct ocores_ep  *ep0 = &udc->ep[0];

		dev_vdbg(udc->dev, "reset event\n");

		*status &= ~SC_INT_RESET;

		/* let upper layer know of reset */
		if (udc->driver && udc->driver->reset) {
			spin_unlock(&udc->lock);
			udc->driver->reset(&udc->gadget);
			spin_lock(&udc->lock);
		}

		/* perform reset */
		ocores_udc_usb_reset(udc);

		/* queue setup request */
		req->type = TRANS_TYPE_SETUP;
		req->ureq.length = 8;
		ocores_ep_queue(ep0, req);
		/* enforce active (in case ep0 was already busy) */
		req->active = true;
	}

	if (*status & SC_INT_RESUME) {
		/* NOP for now */
		dev_vdbg(udc->dev, "resume event\n");
		*status &= ~SC_INT_RESUME;
	}

	if (*status & SC_INT_NAK) {
		/* NOP for now */
		dev_vdbg(udc->dev, "nak event\n");
		*status &= ~SC_INT_NAK;
	}
}

static irqreturn_t ocores_udc_irq(int irq, void *data)
{
	struct ocores_udc *udc = data;
	unsigned long flags;
	u32 mask, status;

	spin_lock_irqsave(&udc->lock, flags);

	/*
	 * event interrupts are level sensitive hence first disable
	 * event interrupts, read status and figure out active interrupts
	 */
	mask = ocores_udc_read(udc, SC_INTERRUPT_MASK);
	ocores_udc_disable_events(udc);

	/* read the interrupt status */
	status = ocores_udc_read(udc, SC_INTERRUPT_STAT);
	ocores_udc_write(udc, SC_INTERRUPT_STAT, status);

	/* don't handle disabled interrupts */
	status &= mask;

	/* check for transaction completions */
	if (status & SC_INT_TRANS) {
		ocores_udc_enable_events(udc);
		ocores_udc_trans_handler(udc, &status);
	}

	/* check for events */
	if (status & SC_INT_EVENTS)
		ocores_udc_event_handler(udc, &status);

	spin_unlock_irqrestore(&udc->lock, flags);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------------- */
/* module probe, remove and of matching */

static int ocores_udc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	struct ocores_udc *udc;
	struct resource *res;
	int rc, irq;
	u8 *buf;

	udc = devm_kzalloc(dev, sizeof(*udc), GFP_KERNEL);
	if (!udc)
		return -ENOMEM;

	/* create a reusable request for ctrl (ep0) operations */
	udc->req = devm_kzalloc(dev, sizeof(*udc->req), GFP_KERNEL);
	if (!udc->req)
		return -ENOMEM;

	buf = devm_kzalloc(dev, MAX_PACKETSZ, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	udc->req->ep = &udc->ep[0];
	INIT_LIST_HEAD(&udc->req->queue);
	udc->req->ureq.buf = buf;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	udc->ioaddr = devm_ioremap_resource(dev, res);
	if (IS_ERR(udc->ioaddr)) {
		dev_err(dev, "failed to ioremap resource 0\n");
		return PTR_ERR(udc->ioaddr);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "failed to obtain irq: %d\n", irq);
		return irq;
	}

	udc->pullup_pin = of_get_named_gpio_flags(np, "ocores,pullup-gpio",
						  0, &flags);

	if (gpio_is_valid(udc->pullup_pin)) {
		rc = devm_gpio_request(dev, udc->pullup_pin, "udc_pullup");
		if (rc)
			dev_warn(dev, "couldn't get pull-up gpio: %d\n", rc);
		else
			gpio_direction_output(udc->pullup_pin, 0);
	}

	if (device_property_present(dev, "ocores,broken-pullup"))
		udc->broken_pullup = true;

	/* init lock */
	spin_lock_init(&udc->lock);

	/* setup gadget structure */
	udc->gadget.ops = &ocores_udc_ops;
	udc->gadget.max_speed = USB_SPEED_FULL;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.ep0 = &udc->ep[0].uep;
	udc->gadget.name = driver_name;
	udc->gadget.is_selfpowered = 1;

	/* initialize endpoints */
	ocores_udc_init_eps(udc);

	/* reset core and select slave mode */
	ocores_udc_hw_reset(udc);

	/* request irq after reset to avoid unwanted interrupts */
	rc = devm_request_irq(dev, irq, ocores_udc_irq, 0, dev_name(dev), udc);
	if (rc) {
		dev_err(dev, "failed to request irq %d: %d\n", irq, rc);
		return rc;
	}

	rc = usb_add_gadget_udc(dev, &udc->gadget);
	if (rc) {
		dev_err(dev, "failed to add USB gadget: %d\n", rc);
		return rc;
	}

	udc->dev = &udc->gadget.dev;

	platform_set_drvdata(pdev, udc);

	return 0;
}

static int ocores_udc_remove(struct platform_device *pdev)
{
	struct ocores_udc *udc = platform_get_drvdata(pdev);

	usb_del_gadget_udc(&udc->gadget);

	return 0;
}

static const struct of_device_id ocores_udc_of_match[] = {
	{ .compatible = "ocores,udc" },
	{},
};
MODULE_DEVICE_TABLE(of, ocores_udc_of_match);

static struct platform_driver ocores_udc_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = ocores_udc_of_match,
	},
	.probe  = ocores_udc_probe,
	.remove = ocores_udc_remove,
};
module_platform_driver(ocores_udc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luis Tanica <luis.f.tanica@seagate.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:" DRIVER_NAME);

