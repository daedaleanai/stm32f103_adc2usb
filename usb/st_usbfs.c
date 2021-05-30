
#include "st_usbfs.h"
#include "usb_private.h"
#include "usbd.h"

enum { USBD_PM_TOP = 0x40 };

static uint8_t             st_usbfs_force_nak[8];
static struct _usbd_device st_usbfs_dev;

static void st_usbfs_set_address(usbd_device* dev, uint8_t addr) {
	(void)dev;
	usb_daddr_set_add(addr);
	USB.DADDR |= USB_DADDR_EF;
}

/**
 * Set the receive buffer size for a given USB endpoint.
 *
 * @param ep Index of endpoint to configure.
 * @param size Size in bytes of the RX buffer.
 */
static void st_usbfs_set_ep_rx_bufsize(usbd_device* dev, uint8_t ep, uint32_t size) {
	(void)dev;
	if (size > 62) {
		if (size & 0x1f) {
			size -= 32;
		}
		USB_PMA.btable[ep].rx_count = (size << 5) | 0x8000;
	} else {
		if (size & 1) {
			size++;
		}
		USB_PMA.btable[ep].rx_count = (size << 10);
	}
}



/* Endpoint type bits for USB_EP_TYPE bit field */
#define USB_EP_TYPE_BULK        0x0000
#define USB_EP_TYPE_CONTROL     0x0200
#define USB_EP_TYPE_ISO         0x0400
#define USB_EP_TYPE_INTERRUPT   0x0600
/* Translate USB standard type codes to STM32. */
static const uint16_t typelookup[] = {
    [USB_ENDPOINT_ATTR_CONTROL]     = USB_EP_TYPE_CONTROL,
    [USB_ENDPOINT_ATTR_ISOCHRONOUS] = USB_EP_TYPE_ISO,
    [USB_ENDPOINT_ATTR_BULK]        = USB_EP_TYPE_BULK,
    [USB_ENDPOINT_ATTR_INTERRUPT]   = USB_EP_TYPE_INTERRUPT,
};

static void
st_usbfs_ep_setup(usbd_device* dev, uint8_t addr, uint8_t type, uint16_t max_size, void (*callback)(usbd_device* usbd_dev, uint8_t ep)) {
	/* Translate USB standard type codes to STM32. */

	uint8_t dir = addr & 0x80;
	addr &= 0x7f;

	/* Assign address. */
	USB_SET_EP_ADDR(addr, addr);
	USB_SET_EP_TYPE(addr, typelookup[type]);

	if (dir || (addr == 0)) {
		USB_SET_EP_TX_ADDR(addr, dev->pm_top);
		if (callback) {
			dev->user_callback_ctr[addr][USB_TRANSACTION_IN] = (void*)callback;
		}
		USB_CLR_EP_TX_DTOG(addr);
		USB_SET_EP_TX_STAT(addr, USB_EP_TX_STAT_NAK);
		dev->pm_top += max_size;
	}

	if (!dir) {
		USB_SET_EP_RX_ADDR(addr, dev->pm_top);
		st_usbfs_set_ep_rx_bufsize(dev, addr, max_size);
		if (callback) {
			dev->user_callback_ctr[addr][USB_TRANSACTION_OUT] = (void*)callback;
		}
		USB_CLR_EP_RX_DTOG(addr);
		USB_SET_EP_RX_STAT(addr, USB_EP_RX_STAT_VALID);
		dev->pm_top += max_size;
	}
}

void st_usbfs_endpoints_reset(usbd_device* dev) {
	int i;

	/* Reset all endpoints. */
	for (i = 1; i < 8; i++) {
		USB_SET_EP_TX_STAT(i, USB_EP_TX_STAT_DISABLED);
		USB_SET_EP_RX_STAT(i, USB_EP_RX_STAT_DISABLED);
	}
	dev->pm_top = USBD_PM_TOP + (2 * dev->desc->bMaxPacketSize0);
}

void st_usbfs_ep_stall_set(usbd_device* dev, uint8_t addr, uint8_t stall) {
	(void)dev;
	if (addr == 0) {
		USB_SET_EP_TX_STAT(addr, stall ? USB_EP_TX_STAT_STALL : USB_EP_TX_STAT_NAK);
	}

	if (addr & 0x80) {
		addr &= 0x7F;

		USB_SET_EP_TX_STAT(addr, stall ? USB_EP_TX_STAT_STALL : USB_EP_TX_STAT_NAK);

		/* Reset to DATA0 if clearing stall condition. */
		if (!stall) {
			USB_CLR_EP_TX_DTOG(addr);
		}
	} else {
		/* Reset to DATA0 if clearing stall condition. */
		if (!stall) {
			USB_CLR_EP_RX_DTOG(addr);
		}

		USB_SET_EP_RX_STAT(addr, stall ? USB_EP_RX_STAT_STALL : USB_EP_RX_STAT_VALID);
	}
}

uint8_t st_usbfs_ep_stall_get(usbd_device* dev, uint8_t addr) {
	(void)dev;
	if (addr & 0x80) {
		if ((USB.EPR[addr & 0x7F] & USB_EPR_STAT_TX) == USB_EP_TX_STAT_STALL) {
			return 1;
		}
	} else {
		if ((USB.EPR[addr] & USB_EPR_STAT_RX) == USB_EP_RX_STAT_STALL) {
			return 1;
		}
	}
	return 0;
}

void st_usbfs_ep_nak_set(usbd_device* dev, uint8_t addr, uint8_t nak) {
	(void)dev;
	/* It does not make sense to force NAK on IN endpoints. */
	if (addr & 0x80) {
		return;
	}

	st_usbfs_force_nak[addr] = nak;

	if (nak) {
		USB_SET_EP_RX_STAT(addr, USB_EP_RX_STAT_NAK);
	} else {
		USB_SET_EP_RX_STAT(addr, USB_EP_RX_STAT_VALID);
	}
}

static void st_usbfs_copy_to_pm(volatile void* vPM, const void* buf, uint16_t len) {
	const uint16_t*    lbuf = buf;
	volatile uint32_t* PM   = vPM;
	for (len = (len + 1) >> 1; len; len--) {
		*PM++ = *lbuf++;
	}
}

static uint16_t st_usbfs_ep_write_packet(usbd_device* dev, uint8_t addr, const void* buf, uint16_t len) {
	(void)dev;
	addr &= 0x7F;

	if ((USB.EPR[addr] & USB_EPR_STAT_TX) == USB_EP_TX_STAT_VALID) {
		return 0;
	}

	st_usbfs_copy_to_pm(USB_GET_EP_TX_BUFF(addr), buf, len);
	USB_SET_EP_TX_COUNT(addr, len);
	USB_SET_EP_TX_STAT(addr, USB_EP_TX_STAT_VALID);

	return len;
}

static void st_usbfs_copy_from_pm(void* buf, const volatile void* vPM, uint16_t len) {
	uint16_t*                lbuf = buf;
	const volatile uint16_t* PM   = vPM;
	uint8_t                  odd  = len & 1;

	for (len >>= 1; len; PM += 2, lbuf++, len--) {
		*lbuf = *PM;
	}

	if (odd) {
		*(uint8_t*)lbuf = *(uint8_t*)PM;
	}
}

static uint16_t st_usbfs_ep_read_packet(usbd_device* dev, uint8_t addr, void* buf, uint16_t len) {
	(void)dev;
	if ((USB.EPR[addr] & USB_EPR_STAT_RX) == USB_EP_RX_STAT_VALID) {
		return 0;
	}

	len = MIN(USB_GET_EP_RX_COUNT(addr) & 0x3ff, len);
	st_usbfs_copy_from_pm(buf, USB_GET_EP_RX_BUFF(addr), len);
	USB_CLR_EP_RX_CTR(addr);

	if (!st_usbfs_force_nak[addr]) {
		USB_SET_EP_RX_STAT(addr, USB_EP_RX_STAT_VALID);
	}

	return len;
}

static void st_usbfs_poll(usbd_device* dev) {
	uint16_t istr = USB.ISTR;

	if (istr & USB_ISTR_RESET) {
		USB.ISTR &= ~USB_ISTR_RESET;
		dev->pm_top = USBD_PM_TOP;
		_usbd_reset(dev);
		return;
	}

	if (istr & USB_ISTR_CTR) {
		uint8_t ep = istr & USB_ISTR_EP_ID;
		uint8_t type;

		if (istr & USB_ISTR_DIR) {
			/* OUT or SETUP? */
			if (USB.EPR[ep] & USB_EPR_SETUP) {
				type = USB_TRANSACTION_SETUP;
			} else {
				type = USB_TRANSACTION_OUT;
			}
		} else {
			type = USB_TRANSACTION_IN;
			USB_CLR_EP_TX_CTR(ep);
		}

		if (dev->user_callback_ctr[ep][type]) {
			dev->user_callback_ctr[ep][type](dev, ep);
		} else {
			USB_CLR_EP_RX_CTR(ep);
		}
	}

	if (istr & USB_ISTR_SUSP) {
		USB.ISTR &= ~USB_ISTR_SUSP;
		if (dev->user_callback_suspend) {
			dev->user_callback_suspend();
		}
	}

	if (istr & USB_ISTR_WKUP) {
		USB.ISTR &= ~USB_ISTR_WKUP;
		if (dev->user_callback_resume) {
			dev->user_callback_resume();
		}
	}

	if (istr & USB_ISTR_SOF) {
		USB.ISTR &= ~USB_ISTR_SOF;
		if (dev->user_callback_sof) {
			dev->user_callback_sof();
		}
	}

	if (dev->user_callback_sof) {
		USB.CNTR |= USB_CNTR_SOFM;
	} else {
		USB.CNTR &= ~USB_CNTR_SOFM;
	}
}

/** Initialize the USB device controller hardware of the STM32. */
static usbd_device* st_usbfs_v1_usbd_init(void) {
	USB.CNTR   = 0;
	USB.BTABLE = 0;
	USB.ISTR   = 0;
	USB.CNTR   = USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
	return &st_usbfs_dev;
}

const struct _usbd_driver st_usbfs_v1_usb_driver = {
    .init            = st_usbfs_v1_usbd_init,
    .set_address     = st_usbfs_set_address,
    .ep_setup        = st_usbfs_ep_setup,
    .ep_reset        = st_usbfs_endpoints_reset,
    .ep_stall_set    = st_usbfs_ep_stall_set,
    .ep_stall_get    = st_usbfs_ep_stall_get,
    .ep_nak_set      = st_usbfs_ep_nak_set,
    .ep_write_packet = st_usbfs_ep_write_packet,
    .ep_read_packet  = st_usbfs_ep_read_packet,
    .poll            = st_usbfs_poll,
};
