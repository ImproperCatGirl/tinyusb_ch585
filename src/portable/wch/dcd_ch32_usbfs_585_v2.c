/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Matthew Tran
 * Copyright (c) 2024 hathach
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

 #include "tusb_option.h"
#include "common/tusb_types.h"


#include "CH585SFR.h" //WCH no longer use the structure + base address header, rather, they are using absolute address for everything.
//it also included all register bit values.
//all bit values have detailed comment in this header too.
#include "CH58x_common.h"

#if 1

#include "device/dcd.h"

 
 /* private defines */
 #define EP_MAX (8)


#define USBFS_INT_ST_MASK_UIS_ENDP(x)  (((x) >> 0) & 0x0F)


 /* Array for DMA address registers (R32) */
static volatile uint32_t * const USB_EP_DMA_ADDR[] = {
    &R32_UEP0_DMA, &R32_UEP1_DMA, &R32_UEP2_DMA, &R32_UEP3_DMA,
    NULL, // Note: Ensure R32_UEP4_DMA is defined in your header; 
                   // if not, use (volatile uint32_t *)(0x40008010 + (4 * 4))
    &R32_UEP5_DMA, &R32_UEP6_DMA, &R32_UEP7_DMA
};

/* Array for Transmission Length registers (R8) */
static volatile uint8_t * const USB_EP_TX_LEN_ADDR[] = {
    &R8_UEP0_T_LEN, &R8_UEP1_T_LEN, &R8_UEP2_T_LEN, &R8_UEP3_T_LEN,
    &R8_UEP4_T_LEN, &R8_UEP5_T_LEN, &R8_UEP6_T_LEN, &R8_UEP7_T_LEN
};

/* Array for Control registers (R8) */
static volatile uint8_t * const USB_EP_CTRL_ADDR[] = {
    &R8_UEP0_CTRL, &R8_UEP1_CTRL, &R8_UEP2_CTRL, &R8_UEP3_CTRL,
    &R8_UEP4_CTRL, &R8_UEP5_CTRL, &R8_UEP6_CTRL, &R8_UEP7_CTRL
};
 
/* Access the DMA Address Register for a given endpoint */
#define EP_DMA(ep)      (*USB_EP_DMA_ADDR[ep])

/* Access the TX Length Register for a given endpoint */
#define EP_TX_LEN(ep)   (*USB_EP_TX_LEN_ADDR[ep])

/* Access the Control Register for a given endpoint */
#define EP_CTRL(ep)     (*USB_EP_CTRL_ADDR[ep])

 /* private data */
 struct usb_xfer {
   bool valid;
   uint8_t* buffer;
   size_t len;
   size_t processed_len;
   size_t max_size;
 };
 
 static struct {
   //bool ep0_tog;
   bool ep_0_tog_in;
   bool ep_0_tog_out;
   bool isochronous[EP_MAX];
   struct usb_xfer xfer[EP_MAX][2];
   TU_ATTR_ALIGNED(4) uint8_t buffer[EP_MAX][2][64];
   TU_ATTR_ALIGNED(4) struct {
     // OUT transfers >64 bytes will overwrite queued IN data!
     uint8_t out[64];
     uint8_t in[1023];
     uint8_t pad;
   } ep4_buffer;
 } data;
 
 /* private helpers */
 static void update_in(uint8_t rhport, uint8_t ep, bool force) {
   struct usb_xfer* xfer = &data.xfer[ep][TUSB_DIR_IN];
   if (xfer->valid) {
     if (force || xfer->len) {
       size_t len = TU_MIN(xfer->max_size, xfer->len);
       if (ep == 0) {
         memcpy(data.buffer[ep][TUSB_DIR_OUT], xfer->buffer, len); // ep0 uses same chunk
       } else if (ep == 4) {
         memcpy(data.ep4_buffer.in, xfer->buffer, len);
       } else {
         memcpy(data.buffer[ep][TUSB_DIR_IN], xfer->buffer, len);
       }
       xfer->buffer += len;
       xfer->len -= len;
       xfer->processed_len += len;
 
       EP_TX_LEN(ep) = len;
       if (ep == 0) {
         //EP_TX_CTRL(0) = USBFS_EP_T_RES_ACK | (data.ep0_tog ? USBFS_EP_T_TOG : 0);
         EP_CTRL(0) = (EP_CTRL(0) & ~(MASK_UEP_T_RES | RB_UEP_T_TOG)) 
             | UEP_T_RES_ACK 
             | (data.ep_0_tog_in ? RB_UEP_T_TOG : 0);
         //data.ep0_tog = !data.ep0_tog;
         data.ep_0_tog_in = !data.ep_0_tog_in;
       } else if (data.isochronous[ep]) {
         //EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~(USBFS_EP_T_RES_MASK)) | USBFS_EP_T_RES_NYET;
       } else {
         //EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~(USBFS_EP_T_RES_MASK)) | USBFS_EP_T_RES_ACK;
         EP_CTRL(ep) = (EP_CTRL(ep) & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
       }
     } else {
       xfer->valid = false;
       //EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~(USBFS_EP_T_RES_MASK)) | USBFS_EP_T_RES_NAK;
       EP_CTRL(ep) = (EP_CTRL(ep) & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
       dcd_event_xfer_complete(
           rhport, ep | TUSB_DIR_IN_MASK, xfer->processed_len,
           XFER_RESULT_SUCCESS, true);
     }
   }
 }
 
 static void update_out(uint8_t rhport, uint8_t ep, size_t rx_len) {
   struct usb_xfer* xfer = &data.xfer[ep][TUSB_DIR_OUT];
   if (xfer->valid) {
     size_t len = TU_MIN(xfer->max_size, TU_MIN(xfer->len, rx_len));
     if (ep == 4) {
       memcpy(xfer->buffer, data.ep4_buffer.out, len);
     } else {
       memcpy(xfer->buffer, data.buffer[ep][TUSB_DIR_OUT], len);
     }
     xfer->buffer += len;
     xfer->len -= len;
     xfer->processed_len += len;
 
     if (xfer->len == 0 || len < xfer->max_size) {
       xfer->valid = false;
       dcd_event_xfer_complete(rhport, ep, xfer->processed_len, XFER_RESULT_SUCCESS, true);
     }
 
     if (ep == 0) {
       //EP_RX_CTRL(0) = USBFS_EP_R_RES_ACK;
       //EP_CTRL(0) = (EP_CTRL(0) & ~MASK_UEP_R_RES) | UEP_R_RES_ACK;
       EP_CTRL(0) = (EP_CTRL(0) & ~(MASK_UEP_R_RES | RB_UEP_R_TOG)) 
       | UEP_R_RES_ACK 
       | (data.ep_0_tog_out ? RB_UEP_R_TOG : 0);
       data.ep_0_tog_out = !data.ep_0_tog_out;
     }
   }
 }
 
 /* public functions */
 /*bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
   (void) rh_init;
   // init registers
   USBOTG_FS->BASE_CTRL = USBFS_CTRL_SYS_CTRL | USBFS_CTRL_INT_BUSY | USBFS_CTRL_DMA_EN;
   USBOTG_FS->UDEV_CTRL = USBFS_UDEV_CTRL_PD_DIS | USBFS_UDEV_CTRL_PORT_EN;
   USBOTG_FS->DEV_ADDR = 0x00;
 
   USBOTG_FS->INT_FG = 0xFF;
   USBOTG_FS->INT_EN = USBFS_INT_EN_BUS_RST | USBFS_INT_EN_TRANSFER | USBFS_INT_EN_SUSPEND;
 
   // setup endpoint 0
   EP_DMA(0) = (uint32_t) &data.buffer[0][0];
   EP_TX_LEN(0) = 0;
   EP_TX_CTRL(0) = USBFS_EP_T_RES_NAK;
   EP_RX_CTRL(0) = USBFS_EP_R_RES_ACK;
 
   // enable other endpoints but NAK everything
   USBOTG_FS->UEP4_1_MOD = 0xCC;
   USBOTG_FS->UEP2_3_MOD = 0xCC;
   USBOTG_FS->UEP5_6_MOD = 0xCC;
   USBOTG_FS->UEP7_MOD = 0x0C;
 
   for (uint8_t ep = 1; ep < EP_MAX; ep++) {
     EP_DMA(ep) = (uint32_t) &data.buffer[ep][0];
     EP_TX_LEN(ep) = 0;
     EP_TX_CTRL(ep) = USBFS_EP_T_AUTO_TOG | USBFS_EP_T_RES_NAK;
     EP_RX_CTRL(ep) = USBFS_EP_R_AUTO_TOG | USBFS_EP_R_RES_NAK;
   }
   EP_DMA(3) = (uint32_t) &data.ep3_buffer.out[0];
 
   dcd_connect(rhport);
 
   return true;
 }*/


bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init)
{
    (void)rh_init;
    (void) rhport;
    R8_USB_CTRL = 0x00; // clear RB_UC_CLR_ALL
    R8_USB_CTRL |= RB_UC_RESET_SIE;
    DelayMs(10);
    R8_USB_CTRL = 0x00; // clear RB_UC_CLR_ALL

    R8_UEP4_1_MOD = RB_UEP4_RX_EN | RB_UEP4_TX_EN | RB_UEP1_RX_EN | RB_UEP1_TX_EN; // endpoint 4 OUT+IN,endpoint1 OUT+IN
    R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_RX_EN | RB_UEP3_TX_EN; // endpoint2 OUT+IN,endpoint3 OUT+IN
    R8_UEP567_MOD = RB_UEP7_RX_EN | RB_UEP7_TX_EN | RB_UEP6_RX_EN | RB_UEP6_TX_EN | RB_UEP5_RX_EN | RB_UEP5_TX_EN;

    R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP4_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; // endpoint 4 again
    R8_UEP5_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP6_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP7_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;

    R32_UEP0_DMA = (uint32_t) &data.buffer[0];
    //R32_UEP0_DMA = (uint32_t) &data.ep0_DMA_buffer;
    R32_UEP1_DMA = (uint32_t) &data.buffer[1];
    R32_UEP2_DMA = (uint32_t) &data.buffer[2];
    R32_UEP3_DMA = (uint32_t) &data.buffer[3];
    //the fourth endpoint does not have DMA capability, as the datasheet suggets
    R32_UEP5_DMA = (uint32_t) &data.buffer[5];
    R32_UEP6_DMA = (uint32_t) &data.buffer[6];
    R32_UEP7_DMA = (uint32_t) &data.buffer[7];

    R8_USB_DEV_AD = 0x00;
    R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN; // enable USB and DMA, 
    R16_PIN_CONFIG |= RB_PIN_USB_EN | RB_UDP_PU_EN;         // enable USB PIN and its pullup resistor
    R8_USB_INT_FG = 0xFF;                                          // clear interrupt
    R8_UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;                   // configure USb device
    R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER;
    dcd_connect(rhport);
    data.ep_0_tog_in = 1;
    data.ep_0_tog_out = 1;
    return 1;
}

 
 void dcd_int_handler(uint8_t rhport) {
   (void) rhport;
   uint8_t status = R8_USB_INT_FG;
   uint8_t int_st = R8_USB_INT_ST;
   if (status & RB_UIF_TRANSFER ) {

    uint8_t ep = USBFS_INT_ST_MASK_UIS_ENDP(int_st);
    //uint8_t token = USBFS_INT_ST_MASK_UIS_TOKEN(R8_USB_INT_ST);
    uint8_t token = int_st & MASK_UIS_TOKEN;
    bool setup = int_st & RB_UIS_SETUP_ACT; 

    bool setup2 = (token == UIS_TOKEN_SETUP);

    if(setup != setup2) printf("Registers do not agree on if the packet token is SETUP\n");

     switch (token) {
       case UIS_TOKEN_OUT: {
         uint16_t rx_len = R8_USB_RX_LEN;
         update_out(rhport, ep, rx_len);
         break;
       }
 
       case UIS_TOKEN_IN:
         update_in(rhport, ep, false);
         break;
 
       case UIS_TOKEN_SETUP:
         // setup clears stall
         //EP_TX_CTRL(0) = USBFS_EP_T_RES_NAK;
         //EP_RX_CTRL(0) = USBFS_EP_R_RES_ACK;
        // Sets Endpoint 0 to NAK on IN (TX) and ACK on OUT (RX)
        EP_CTRL(0) = (EP_CTRL(0) & ~(MASK_UEP_T_RES | MASK_UEP_R_RES)) | (UEP_T_RES_NAK | UEP_R_RES_ACK);
         data.ep_0_tog_in = true;
         data.ep_0_tog_out = true;
         dcd_event_setup_received(rhport, &data.buffer[0][TUSB_DIR_OUT][0], true);
         break;
     }
 
     //USBOTG_FS->INT_FG = USBFS_INT_FG_TRANSFER;
     R8_USB_INT_FG = RB_UIF_TRANSFER;
   } else if (status & RB_UIF_BUS_RST) {
     data.ep_0_tog_in = true;
     data.ep_0_tog_out = true;
     data.xfer[0][TUSB_DIR_OUT].max_size = 64;
     data.xfer[0][TUSB_DIR_IN].max_size = 64;
 
     //dcd_event_bus_reset(rhport, (USBOTG_FS->BASE_CTRL & USBFS_CTRL_LOW_SPEED) ? TUSB_SPEED_LOW : TUSB_SPEED_FULL, true);
     dcd_event_bus_reset(rhport, (R8_USB_CTRL & RB_UC_LOW_SPEED) ? TUSB_SPEED_LOW : TUSB_SPEED_FULL, true);
 
     R8_USB_DEV_AD = 0x00;
     //EP_RX_CTRL(0) = USBFS_EP_R_RES_ACK;
     EP_CTRL(0) = (EP_CTRL(0) & ~MASK_UEP_R_RES) | UEP_R_RES_ACK;
     //USBOTG_FS->INT_FG = USBFS_INT_FG_BUS_RST;
     R8_USB_INT_FG = RB_UIF_BUS_RST;
   } else if (status & RB_UIF_SUSPEND) {
     dcd_event_t event = {.rhport = rhport, .event_id = DCD_EVENT_SUSPEND};
     dcd_event_handler(&event, true);
     //USBOTG_FS->INT_FG = USBFS_INT_FG_SUSPEND;
     R8_USB_INT_FG = RB_UIF_SUSPEND;
   }
 }
 
 void dcd_int_enable(uint8_t rhport) {
   (void) rhport;
   PFIC_EnableIRQ(USB_IRQn);
 }
 
 void dcd_int_disable(uint8_t rhport) {
   (void) rhport;
   PFIC_DisableIRQ(USB_IRQn);
 }
 
 void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
   (void) dev_addr;
   dcd_edpt_xfer(rhport, 0x80, NULL, 0); // zlp status response
 }
 
 void dcd_remote_wakeup(uint8_t rhport) {
   (void) rhport;
   // TODO optional
 }
 
 void dcd_connect(uint8_t rhport) {
   (void) rhport;

  R8_USB_CTRL |= RB_UC_DEV_PU_EN;
 }
 
 void dcd_disconnect(uint8_t rhport) {
   (void) rhport;

  R8_USB_CTRL &= ~ RB_UC_DEV_PU_EN;
 }
 
 void dcd_sof_enable(uint8_t rhport, bool en) {
   (void) rhport;
   (void) en;
 
   // TODO implement later
 }
 
 void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const* request) {
   (void) rhport;
   if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
       request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
       request->bRequest == TUSB_REQ_SET_ADDRESS) {
     //USBOTG_FS->DEV_ADDR = (uint8_t) request->wValue;
     R8_USB_DEV_AD = (uint8_t) request->wValue;
   }
   //EP_TX_CTRL(0) = USBFS_EP_T_RES_NAK;
   //EP_RX_CTRL(0) = USBFS_EP_R_RES_ACK;
   // Sets Endpoint 0 to NAK on IN (TX) and ACK on OUT (RX)
EP_CTRL(0) = (EP_CTRL(0) & ~(MASK_UEP_T_RES | MASK_UEP_R_RES)) | (UEP_T_RES_NAK | UEP_R_RES_ACK);
 }
 
 bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const* desc_ep) {
   (void) rhport;
   uint8_t ep = tu_edpt_number(desc_ep->bEndpointAddress);
   uint8_t dir = tu_edpt_dir(desc_ep->bEndpointAddress);
   TU_ASSERT(ep < EP_MAX);
 
   data.isochronous[ep] = desc_ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS;
   data.xfer[ep][dir].max_size = tu_edpt_packet_size(desc_ep);
 
   if (ep != 0) {
     if (dir == TUSB_DIR_OUT) {
       if (data.isochronous[ep]) {
         //EP_RX_CTRL(ep) = USBFS_EP_R_AUTO_TOG | USBFS_EP_R_RES_NYET;
       } else {
         //EP_RX_CTRL(ep) = USBFS_EP_R_AUTO_TOG | USBFS_EP_R_RES_ACK;
         EP_CTRL(ep) = (EP_CTRL(ep) & ~(MASK_UEP_R_RES | RB_UEP_AUTO_TOG)) | (UEP_R_RES_ACK | RB_UEP_AUTO_TOG);
       }
     } else {
       EP_TX_LEN(ep) = 0;
       //EP_TX_CTRL(ep) = USBFS_EP_T_AUTO_TOG | USBFS_EP_T_RES_NAK;
       EP_CTRL(ep) = (EP_CTRL(ep) & ~(MASK_UEP_T_RES | RB_UEP_AUTO_TOG)) | (UEP_T_RES_NAK | RB_UEP_AUTO_TOG);
     }
   }
   return true;
 }
 
 void dcd_edpt_close_all(uint8_t rhport) {
   (void) rhport;
   // TODO optional
 }
 
 void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
   (void) rhport;
   (void) ep_addr;
   // TODO optional
 }
 
 bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes) {
   (void) rhport;
   uint8_t ep = tu_edpt_number(ep_addr);
   uint8_t dir = tu_edpt_dir(ep_addr);
 
   struct usb_xfer* xfer = &data.xfer[ep][dir];
   dcd_int_disable(rhport);
   xfer->valid = true;
   xfer->buffer = buffer;
   xfer->len = total_bytes;
   xfer->processed_len = 0;
   dcd_int_enable(rhport);
 
   if (dir == TUSB_DIR_IN) {
     update_in(rhport, ep, true);
   }
   return true;
 }
 
 void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
   (void) rhport;
   uint8_t ep = tu_edpt_number(ep_addr);
   uint8_t dir = tu_edpt_dir(ep_addr);
   /*if (ep == 0) {
     if (dir == TUSB_DIR_OUT) {
       EP_RX_CTRL(0) = USBFS_EP_R_RES_STALL;
     } else {
       EP_TX_LEN(0) = 0;
       EP_TX_CTRL(0) = USBFS_EP_T_RES_STALL;
     }
   } else {
     if (dir == TUSB_DIR_OUT) {
       EP_RX_CTRL(ep) = (EP_RX_CTRL(ep) & ~USBFS_EP_R_RES_MASK) | USBFS_EP_R_RES_STALL;
     } else {
       EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~USBFS_EP_T_RES_MASK) | USBFS_EP_T_RES_STALL;
     }
   }*/
   if (ep == 0) {
    if (dir == TUSB_DIR_OUT) {
        EP_CTRL(0) = (EP_CTRL(0) & ~MASK_UEP_R_RES) | UEP_R_RES_STALL;
    }
    } else {
        if (dir == TUSB_DIR_OUT) {
            EP_CTRL(ep) = (EP_CTRL(ep) & ~(MASK_UEP_R_RES | RB_UEP_R_TOG)) | UEP_R_RES_STALL;
        } else {
            EP_CTRL(ep) = (EP_CTRL(ep) & ~(MASK_UEP_T_RES | RB_UEP_T_TOG)) | UEP_T_RES_STALL;
        }
    }
 }
 
 void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
   (void) rhport;
   uint8_t ep = tu_edpt_number(ep_addr);
   uint8_t dir = tu_edpt_dir(ep_addr);
   /*if (ep == 0) {
     if (dir == TUSB_DIR_OUT) {
       EP_RX_CTRL(0) = USBFS_EP_R_RES_ACK;
     }
   } else {
     if (dir == TUSB_DIR_OUT) {
       EP_RX_CTRL(ep) = (EP_RX_CTRL(ep) & ~(USBFS_EP_R_RES_MASK | USBFS_EP_R_TOG)) | USBFS_EP_R_RES_ACK;
     } else {
       EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~(USBFS_EP_T_RES_MASK | USBFS_EP_T_TOG)) | USBFS_EP_T_RES_NAK;
     }
   }*/
   if (ep == 0) {
    if (dir == TUSB_DIR_OUT) {
        // For EP0 OUT: Reset RX response to ACK, preserve TX response and Toggles
        EP_CTRL(0) = (EP_CTRL(0) & ~MASK_UEP_R_RES) | UEP_R_RES_ACK;
    }
} else {
    if (dir == TUSB_DIR_OUT) {
        /* * For EPn OUT: 
         * 1. Clear MASK_UEP_R_RES to set response bits to 00 (ACK)
         * 2. Clear RB_UEP_R_TOG to reset expected toggle to DATA0
         */
        EP_CTRL(ep) = (EP_CTRL(ep) & ~(MASK_UEP_R_RES | RB_UEP_R_TOG)) | UEP_R_RES_ACK;
    } else {
        /* * For EPn IN: 
         * 1. Mask and set response to NAK
         * 2. Clear RB_UEP_T_TOG to reset transmit toggle to DATA0
         */
        EP_CTRL(ep) = (EP_CTRL(ep) & ~(MASK_UEP_T_RES | RB_UEP_T_TOG)) | UEP_T_RES_NAK;
    }
}
 }
 
 #endif
 