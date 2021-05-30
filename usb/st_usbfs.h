#pragma once

#include <stdint.h>

struct USB_Type {
	volatile uint32_t EPR[8];
	const    uint32_t RESERVED[8];
	volatile uint32_t CNTR;
	volatile uint32_t ISTR;
	volatile uint32_t FNR;
	volatile uint32_t DADDR;
	volatile uint32_t BTABLE;  // this driver assumes this is set to zero 
};

extern struct USB_Type USB;  // @ 0x40005c00

// USB->EPR[i] endpoint i register
enum {
        USB_EPR_CTR_RX 	= 1UL<<15, // Correct transfer for reception
        USB_EPR_DTOG_RX = 1UL<<14, // Data Toggle, for reception transfers
        USB_EPR_STAT_RX = ((1UL<<2)-1) << 12, // Status bits, for reception transfers
        USB_EPR_SETUP 	= 1UL<<11, // Setup transaction completed
        USB_EPR_TYPE    = ((1UL<<2)-1) << 9, // Endpoint type
        USB_EPR_KIND    = 1UL<<8, // Endpoint kind  type=bulk -> double buffer  type=control -> status out 
        USB_EPR_CTR_TX 	= 1UL<<7, // Correct Transfer for transmission
        USB_EPR_DTOG_TX = 1UL<<6, // Data Toggle, for transmission transfers
        USB_EPR_STAT_TX = ((1UL<<2)-1) << 4, // Status bits, for transmission transfers
        USB_EPR_EA 	= ((1UL<<4)-1) << 0, // Endpoint address

        /* Masking all toggle bits */
        USB_EPR_NTOGGLE_MSK = (USB_EPR_CTR_RX | USB_EPR_SETUP | USB_EPR_TYPE | USB_EPR_KIND | USB_EPR_CTR_TX | USB_EPR_EA),
        USB_EPR_RX_STAT_TOG_MSK = (USB_EPR_STAT_RX | USB_EPR_NTOGGLE_MSK),
        USB_EPR_TX_STAT_TOG_MSK = (USB_EPR_STAT_TX | USB_EPR_NTOGGLE_MSK),
};

inline void 	usb_epr_set_ep_type(int ep, uint32_t val) { USB.EPR[ep] = (USB.EPR[ep] & ~USB_EPR_TYPE) | ((val<<9) & USB_EPR_TYPE); }
inline void 	usb_epr_set_ea(int ep, uint32_t val) { USB.EPR[ep] = (USB.EPR[ep] & ~USB_EPR_EA) | ((val<<0) & USB_EPR_EA); }
inline uint32_t usb_epr_get_stat_rx(int ep) 	{ return (USB.EPR[ep] & USB_EPR_STAT_RX) >> 12 ; }
inline uint32_t usb_epr_get_ep_type(int ep) 	{ return (USB.EPR[ep] & USB_EPR_TYPE) >> 9 ; }
inline uint32_t usb_epr_get_stat_tx(int ep) 	{ return (USB.EPR[ep] & USB_EPR_STAT_TX) >> 4 ; }
inline uint32_t usb_epr_get_ea(int ep) 		{ return (USB.EPR[ep] & USB_EPR_EA) >> 0 ; }
inline void     usb_epr_set_stat_rx(int ep, uint32_t val) { USB.EPR[ep] = (USB.EPR[ep] & ~USB_EPR_STAT_RX) | ((val<<12) & USB_EPR_STAT_RX); }
inline void     usb_epr_set_stat_tx(int ep, uint32_t val) { USB.EPR[ep] = (USB.EPR[ep] & ~USB_EPR_STAT_TX) | ((val<<4) & USB_EPR_STAT_TX); }


// USB->CNTR control register
enum {
        USB_CNTR_CTRM 		= 1UL<<15, // Correct transfer interrupt mask
        USB_CNTR_PMAOVRM 	= 1UL<<14, // Packet memory area over / underrun interrupt mask
        USB_CNTR_ERRM 		= 1UL<<13, // Error interrupt mask
        USB_CNTR_WKUPM 		= 1UL<<12, // Wakeup interrupt mask
        USB_CNTR_SUSPM 		= 1UL<<11, // Suspend mode interrupt mask
        USB_CNTR_RESETM 	= 1UL<<10, // USB reset interrupt mask
        USB_CNTR_SOFM 		= 1UL<<9, // Start of frame interrupt mask
        USB_CNTR_ESOFM 		= 1UL<<8, // Expected start of frame interrupt mask
        USB_CNTR_RESUME 	= 1UL<<4, // Resume request
        USB_CNTR_FSUSP 		= 1UL<<3, // Force suspend
        USB_CNTR_LPMODE 	= 1UL<<2, // Low-power mode
        USB_CNTR_PDWN 		= 1UL<<1, // Power down
        USB_CNTR_FRES 		= 1UL<<0, // Force USB Reset
};

// USB->ISTR interrupt status register
enum {
        USB_ISTR_CTR 	= 1UL<<15, // Correct transfer
        USB_ISTR_PMAOVR = 1UL<<14, // Packet memory area over / underrun
        USB_ISTR_ERR 	= 1UL<<13, // Error
        USB_ISTR_WKUP 	= 1UL<<12, // Wakeup
        USB_ISTR_SUSP 	= 1UL<<11, // Suspend mode request
        USB_ISTR_RESET 	= 1UL<<10, // reset request
        USB_ISTR_SOF 	= 1UL<<9, // start of frame
        USB_ISTR_ESOF 	= 1UL<<8, // Expected start frame
        USB_ISTR_DIR 	= 1UL<<4, // Direction of transaction
        USB_ISTR_EP_ID 	= ((1UL<<4)-1) << 0, // Endpoint Identifier
};
inline void usb_istr_set_ep_id(uint32_t val) { USB.ISTR = (USB.ISTR & ~USB_ISTR_EP_ID) | ((val<<0) & USB_ISTR_EP_ID); }
inline uint32_t usb_istr_get_ep_id(void) { return (USB.ISTR & USB_ISTR_EP_ID) >> 0 ; }

// USB->FNR frame number register
enum {
        USB_FNR_RXDP = 1UL<<15, // Receive data + line status
        USB_FNR_RXDM = 1UL<<14, // Receive data - line status
        USB_FNR_LCK  = 1UL<<13, // Locked
        USB_FNR_LSOF = ((1UL<<2)-1) << 11, // Lost SOF
        USB_FNR_FN   = ((1UL<<11)-1) << 0, // Frame number
};
inline uint32_t usb_fnr_get_lsof(void) { return (USB.FNR & USB_FNR_LSOF) >> 11 ; }
inline uint32_t usb_fnr_get_fn(void) { return (USB.FNR & USB_FNR_FN) >> 0 ; }


// USB->DADDR device address
enum {
        USB_DADDR_EF  = 1UL<<7, // Enable function
        USB_DADDR_ADD = ((1UL<<7)-1) << 0, // Device address
};
inline void usb_daddr_set_add(uint32_t val) { USB.DADDR = (USB.DADDR & ~USB_DADDR_ADD) | ((val<<0) & USB_DADDR_ADD); }
inline uint32_t usb_daddr_get_add(void) { return (USB.DADDR & USB_DADDR_ADD) >> 0 ; }




extern union {
        struct {
                volatile uint32_t tx_offs;  // in units of uint16_t
                volatile uint32_t tx_count;
                volatile uint32_t rx_offs;
                volatile uint32_t rx_count;
        } btable[8];  // located here by virtue of USB.BTABLE being zero
        uint16_t buf[512];
} USB_PMA; // @ 0x50006000

inline uint8_t* usb_ep_tx_buf(int ep)   { return (uint8_t*)(&USB_PMA.buf[USB_PMA.btable[ep].tx_offs]); }
inline uint32_t usb_ep_tx_count(int ep) { return USB_PMA.btable[ep].tx_count; }
inline uint8_t* usb_ep_rx_buf(int ep)   { return (uint8_t*)(&USB_PMA.buf[USB_PMA.btable[ep].rx_offs]); }
inline uint32_t usb_ep_rx_count(int ep) { return USB_PMA.btable[ep].rx_count; }


#if 1
#define USB_GET_EP_TX_ADDR(EP)          ( USB_PMA.btable[EP].tx_offs  )
#define USB_GET_EP_TX_COUNT(EP)         ( USB_PMA.btable[EP].tx_count )
#define USB_GET_EP_RX_ADDR(EP)          ( USB_PMA.btable[EP].rx_offs  )
#define USB_GET_EP_RX_COUNT(EP)         ( USB_PMA.btable[EP].rx_count )

#define USB_SET_EP_TX_ADDR(EP, ADDR)    { USB_PMA.btable[EP].tx_offs  = ADDR; }
#define USB_SET_EP_RX_ADDR(EP, ADDR)    { USB_PMA.btable[EP].rx_offs  = ADDR; }

#define USB_GET_EP_TX_BUFF(EP)          ( &USB_PMA.buf[USB_GET_EP_TX_ADDR(EP)] )
#define USB_GET_EP_RX_BUFF(EP)          ( &USB_PMA.buf[USB_GET_EP_RX_ADDR(EP)] )

#define USB_SET_EP_TX_COUNT(EP, COUNT)  { USB_PMA.btable[EP].tx_count  = COUNT; }
#define USB_SET_EP_RX_COUNT(EP, COUNT)  { USB_PMA.btable[EP].rx_count  = COUNT; }



/* Endpoint status bits for USB_EP_RX_STAT bit field */
#define USB_EP_RX_STAT_DISABLED 0x0000
#define USB_EP_RX_STAT_STALL    0x1000
#define USB_EP_RX_STAT_NAK      0x2000
#define USB_EP_RX_STAT_VALID    0x3000

/* Endpoint status bits for USB_EP_TX_STAT bit field */
#define USB_EP_TX_STAT_DISABLED 0x0000
#define USB_EP_TX_STAT_STALL    0x0010
#define USB_EP_TX_STAT_NAK      0x0020
#define USB_EP_TX_STAT_VALID    0x0030


#define USB_SET_EP_RX_STAT(EP, STAT)  { USB.EPR[EP] = ((USB.EPR[EP] & USB_EPR_RX_STAT_TOG_MSK) ^ STAT) | (USB_EPR_CTR_RX | USB_EPR_CTR_TX); }
#define USB_SET_EP_TX_STAT(EP, STAT)  { USB.EPR[EP] = ((USB.EPR[EP] & USB_EPR_TX_STAT_TOG_MSK) ^ STAT) | (USB_EPR_CTR_RX | USB_EPR_CTR_TX); } 



#define USB_CLR_EP_RX_CTR(EP)   { USB.EPR[EP] = (USB.EPR[EP] & USB_EPR_NTOGGLE_MSK & (~(USB_EPR_CTR_RX))) | (USB_EPR_CTR_TX); } 
#define USB_CLR_EP_TX_CTR(EP)   { USB.EPR[EP] = (USB.EPR[EP] & USB_EPR_NTOGGLE_MSK & (~(USB_EPR_CTR_TX))) | (USB_EPR_CTR_RX); } 


#define USB_SET_EP_TYPE(EP, TYPE) { USB.EPR[EP] = (USB.EPR[EP] & (USB_EPR_NTOGGLE_MSK & (~USB_EPR_TYPE))) | TYPE; }


#define USB_SET_EP_KIND(EP)      { USB.EPR[EP] = (USB.EPR[EP] & (USB_EPR_NTOGGLE_MSK & (~USB_EPR_KIND))) | USB_EPR_KIND; }
#define USB_CLR_EP_KIND(EP)      { USB.EPR[EP] = (USB.EPR[EP] & (USB_EPR_NTOGGLE_MSK & (~USB_EPR_KIND))); }

#define USB_SET_EP_STAT_OUT(EP) USB_SET_EP_KIND(EP)
#define USB_CLR_EP_STAT_OUT(EP) USB_CLR_EP_KIND(EP)

#define USB_SET_EP_ADDR(EP, ADDR) { USB.EPR[EP] =  (((USB.EPR[EP]) & (USB_EPR_NTOGGLE_MSK & (~USB_EPR_EA))) | ADDR); }

/* Macros for clearing DTOG bits */
#define USB_CLR_EP_TX_DTOG(EP) { USB.EPR[EP] = USB.EPR[EP] & (USB_EPR_NTOGGLE_MSK | USB_EPR_DTOG_TX); }

#define USB_CLR_EP_RX_DTOG(EP) { USB.EPR[EP] = USB.EPR[EP] & (USB_EPR_NTOGGLE_MSK | USB_EPR_DTOG_RX); } 

#endif

