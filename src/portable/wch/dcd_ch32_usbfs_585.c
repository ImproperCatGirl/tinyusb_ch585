/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Matthew Tran
 * Copyright (c) 2024 hathach
 * Copyright (c) 2025 AImproperCatGirl
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

#include "common/tusb_types.h"


#include "CH585SFR.h" //WCH no longer use the structure + base address header, rather, they are using absolute address for everything.
//it also included all register bit values.
//all bit values have detailed comment in this header too.
#include "CH58x_common.h"

#if 1

#include "device/dcd.h"

#define PRINT_  // supposed to print strings into a buffer, no printf in ISR

#define USBFS_INT_ST_MASK_UIS_ENDP(x)  (((x) >> 0) & 0x0F)

/* private defines */
#define EP_MAX (8)

bool first_setup = 1;

static inline void set_endpoint_t_len(uint8_t ep, uint8_t len) {
  switch (ep) {
    case 0: R8_UEP0_T_LEN = len; break;
    case 1: R8_UEP1_T_LEN = len; break;
    case 2: R8_UEP2_T_LEN = len; break;
    case 3: R8_UEP3_T_LEN = len; break;
    case 4: R8_UEP4_T_LEN = len; break;
    case 5: R8_UEP5_T_LEN = len; break;
    case 6: R8_UEP6_T_LEN = len; break;
    case 7: R8_UEP7_T_LEN = len; break;
    default: break; // Invalid endpoint
  }
}


static inline void set_endpoint_tx_ctrl(uint8_t ep, uint8_t res, bool auto_toggle) {
  uint8_t ctrl_val = (res & MASK_UEP_T_RES) | (auto_toggle && ep != 0 && ep != 4 ? RB_UEP_AUTO_TOG : 0);
  switch (ep) {
    case 0: R8_UEP0_CTRL = (R8_UEP0_CTRL & ~(MASK_UEP_T_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 1: R8_UEP1_CTRL = (R8_UEP1_CTRL & ~(MASK_UEP_T_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 2: R8_UEP2_CTRL = (R8_UEP2_CTRL & ~(MASK_UEP_T_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 3: R8_UEP3_CTRL = (R8_UEP3_CTRL & ~(MASK_UEP_T_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 4: R8_UEP4_CTRL = (R8_UEP4_CTRL & ~(MASK_UEP_T_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 5: R8_UEP5_CTRL = (R8_UEP5_CTRL & ~(MASK_UEP_T_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 6: R8_UEP6_CTRL = (R8_UEP6_CTRL & ~(MASK_UEP_T_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 7: R8_UEP7_CTRL = (R8_UEP7_CTRL & ~(MASK_UEP_T_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    default: break; // Invalid endpoint
  }
}

static inline void set_endpoint_rx_ctrl(uint8_t ep, uint8_t res, bool auto_toggle) {
  uint8_t ctrl_val = (res & MASK_UEP_R_RES) | (auto_toggle && ep != 0 && ep != 4 ? RB_UEP_AUTO_TOG : 0);
  switch (ep) {
    case 0: R8_UEP0_CTRL = (R8_UEP0_CTRL & ~(MASK_UEP_R_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 1: R8_UEP1_CTRL = (R8_UEP1_CTRL & ~(MASK_UEP_R_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 2: R8_UEP2_CTRL = (R8_UEP2_CTRL & ~(MASK_UEP_R_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 3: R8_UEP3_CTRL = (R8_UEP3_CTRL & ~(MASK_UEP_R_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 4: R8_UEP4_CTRL = (R8_UEP4_CTRL & ~(MASK_UEP_R_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 5: R8_UEP5_CTRL = (R8_UEP5_CTRL & ~(MASK_UEP_R_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 6: R8_UEP6_CTRL = (R8_UEP6_CTRL & ~(MASK_UEP_R_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    case 7: R8_UEP7_CTRL = (R8_UEP7_CTRL & ~(MASK_UEP_R_RES | RB_UEP_AUTO_TOG)) | ctrl_val; break;
    default: break; // Invalid endpoint
  }
}

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
  bool ep0_tog_in;
  bool ep0_tog_out;
  bool isochronous[EP_MAX];
  struct usb_xfer xfer[EP_MAX][2];
  TU_ATTR_ALIGNED(4) uint8_t buffer[EP_MAX][2][64];
  TU_ATTR_ALIGNED(4) struct {
    uint8_t out[64];
    uint8_t in[64];
    uint8_t pad;
  } ep4_buffer;
  TU_ATTR_ALIGNED(4) uint8_t ep0_DMA_buffer[64];
  TU_ATTR_ALIGNED(4) uint8_t ep0_in_buffer[64];
  TU_ATTR_ALIGNED(4) uint8_t ep0_out_buffer[64];
} data;

/* private helpers */
static void update_in(uint8_t rhport, uint8_t ep, bool force) {
  PRINT_("update_in: ep=%d, ep0_in_tog=%d\n", ep, data.ep0_tog_in);
  //R8_UEP0_CTRL = (R8_UEP0_CTRL & ~RB_UEP_T_TOG) | (data.ep0_tog_in ? RB_UEP_T_TOG : 0);
  struct usb_xfer* xfer = &data.xfer[ep][TUSB_DIR_IN];
  if (xfer->valid) {
    if (force || xfer->len) {
      size_t len = TU_MIN(xfer->max_size, xfer->len);
      if (ep == 0) 
      {
        memcpy(data.ep0_in_buffer, xfer->buffer, len);
        //todo
        dcd_int_disable(rhport);

        memcpy(data.ep0_DMA_buffer, data.ep0_in_buffer, len);

        dcd_int_enable(rhport);
        //memcpy(data.buffer[ep][TUSB_DIR_OUT], xfer->buffer, len); // ep0 uses same chunk
      } else if (ep == 4) {
        memcpy(data.ep4_buffer.in, xfer->buffer, len);
      } else {
        memcpy(data.buffer[ep][TUSB_DIR_IN], xfer->buffer, len);
      }
      xfer->buffer += len;
      xfer->len -= len;
      xfer->processed_len += len;

      set_endpoint_t_len(ep, len);
      if (ep == 0) {
        set_endpoint_tx_ctrl(ep, UEP_T_RES_ACK, false);
        R8_UEP0_CTRL = (R8_UEP0_CTRL & ~RB_UEP_T_TOG) | (data.ep0_tog_in ? RB_UEP_T_TOG : 0);
        data.ep0_tog_in = !data.ep0_tog_in;
      } else if (data.isochronous[ep]) {
        set_endpoint_tx_ctrl(ep, UEP_T_RES_TOUT, true);//NYET?
      } else {
        set_endpoint_tx_ctrl(ep, UEP_T_RES_ACK, true);
      }
    } else {
      xfer->valid = false;
      set_endpoint_tx_ctrl(ep, UEP_T_RES_NAK, true);
      dcd_event_xfer_complete(rhport, ep | TUSB_DIR_IN_MASK, xfer->processed_len, XFER_RESULT_SUCCESS, true);
    }
  }
}

static void update_out(uint8_t rhport, uint8_t ep, size_t rx_len) {
  PRINT_("update_out: ep=%d, ep0_out_tog=%d\n", ep, data.ep0_tog_out);
  struct usb_xfer *xfer = &data.xfer[ep][TUSB_DIR_OUT];
  //size_t len = TU_MIN(xfer->max_size, TU_MIN(xfer->len, rx_len));
  // Handle ZLP or data packet
  size_t len = (ep == 0 && rx_len == 0) ? 0 : TU_MIN(xfer->max_size, TU_MIN(xfer->len, rx_len));
  if(len > 0)
  {
    if(ep == 0)
    {
      dcd_int_disable(rhport);
      memcpy(data.ep0_out_buffer, data.ep0_DMA_buffer, len);
      dcd_int_enable(rhport);

      memcpy(xfer->buffer, data.ep0_out_buffer, len); // Copy to xfer buffer
      PRINT_("copied OUT data, the first 16 bytes:");
      for(int i = 0; i < 16; i ++)
      {
        PRINT_("%02X ", xfer->buffer[i]);
      }
      PRINT_("\n");
      //todo
      //logic
    }
    else if (ep == 4) {
      memcpy(xfer->buffer, data.ep4_buffer.out, len);
    } 
    else {
      memcpy(xfer->buffer, data.buffer[ep][TUSB_DIR_OUT], len);
    }
    xfer->buffer += len;
    xfer->len -= len;
    xfer->processed_len += len;
  
  }
  else 
  {
    PRINT_("ZLP processing\n");
  }
  if (xfer->len == 0 || len < xfer->max_size) {
    xfer->valid = false;
    PRINT_("processed_len = %d", xfer->processed_len);
    dcd_event_xfer_complete(rhport, ep, xfer->processed_len, XFER_RESULT_SUCCESS, true);
  }

  if (ep == 0) {
    set_endpoint_rx_ctrl(ep, UEP_R_RES_ACK, false);
    R8_UEP0_CTRL = (R8_UEP0_CTRL & ~RB_UEP_R_TOG) | (data.ep0_tog_out ? RB_UEP_R_TOG : 0);
    data.ep0_tog_out = !data.ep0_tog_out;
  }
}

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

    //R32_UEP0_DMA = (uint32_t) &data.buffer[0];
    R32_UEP0_DMA = (uint32_t) &data.ep0_DMA_buffer;
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
    data.ep0_tog_in = 1;
    data.ep0_tog_out = 1;
    return 1;
}


void dcd_int_handler(uint8_t rhport) {
  (void) rhport;
  bool mis_busy = !(R8_USB_MIS_ST & RB_UMS_SIE_FREE);
  bool fg_busy = !(R8_USB_INT_FG & RB_U_SIE_FREE);

  if(mis_busy != fg_busy)
  {
    PRINT_("Registers do not agree on if the SIE is busy\n");
    //while(1);
    //return;
  }
  if((mis_busy == 1) || (fg_busy == 1))
  {
    return;
  }
  //dcd_int_disable(rhport);
  uint8_t status = R8_USB_INT_FG;
  uint8_t int_st = R8_USB_INT_ST; // Capture INT_ST early
  //PRINT_("USB INT_FG=0x%02x, INT_ST=0x%02x\n", status, int_st); // Log for debugging
  if (status & RB_UIF_TRANSFER) 
  {
    uint8_t ep = USBFS_INT_ST_MASK_UIS_ENDP(int_st);
    //uint8_t token = USBFS_INT_ST_MASK_UIS_TOKEN(R8_USB_INT_ST);
    uint8_t token = int_st & MASK_UIS_TOKEN;
    //PRINT_("ep = %d, token = %02X\n", ep, token);
    bool tog_ok = int_st & RB_UIS_TOG_OK;
    bool setup = int_st & RB_UIS_SETUP_ACT; // fuck wch

    bool setup2 = (token == UIS_TOKEN_SETUP);

    if(setup != setup2) PRINT_("Registers do not agree on if the packet token is SETUP\n");
    PRINT_("Transfer: ep=%d, token=0x%02x, setup_act=%d, tog_ok=%d\n", ep, token, setup, tog_ok);
    if(ep == 0)
    {
        if(setup)
        {
          data.ep0_tog_in = 1;
          data.ep0_tog_out = 1;
          R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
          memcpy(data.ep0_out_buffer, data.ep0_DMA_buffer, 8);
          dcd_event_setup_received(rhport, data.ep0_out_buffer, true);
          R8_USB_INT_FG = RB_UIF_TRANSFER;
          return;
        }
    }
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
        PRINT_("Missed a SETUP packet!\n");
        data.ep0_tog_in = 1;
        data.ep0_tog_out = 1;
        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        memcpy(data.ep0_out_buffer, data.ep0_DMA_buffer, 8);
        dcd_event_setup_received(rhport, data.ep0_out_buffer, true);
        R8_USB_INT_FG = RB_UIF_TRANSFER;
        return;
        
    }
    R8_USB_INT_FG = RB_UIF_TRANSFER;
  } else if (status & RB_UIF_BUS_RST) {
    PRINT_("bus reset!\n");
    first_setup = 1;
    data.ep0_tog_in = 1;
    data.ep0_tog_out = 1;
    data.xfer[0][TUSB_DIR_OUT].max_size = 64;
    data.xfer[0][TUSB_DIR_IN].max_size = 64;

    dcd_event_bus_reset(rhport, (R8_UDEV_CTRL & RB_UD_LOW_SPEED) ? TUSB_SPEED_LOW : TUSB_SPEED_FULL, true);

    R8_USB_DEV_AD = 0x00;
    R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    //R8_UEP0_CTRL = (R8_UEP0_CTRL & ~RB_UEP_T_TOG) | (data.ep0_tog_in ? RB_UEP_T_TOG : 0);
    R8_USB_INT_FG = RB_UIF_BUS_RST;
    
    
  } else if (status & RB_UIF_SUSPEND ) {
    dcd_event_t event = {.rhport = rhport, .event_id = DCD_EVENT_SUSPEND};
    dcd_event_handler(&event, true);
    R8_USB_INT_FG = RB_UIF_SUSPEND;
  }
  //dcd_int_enable(rhport);
}

void dcd_int_enable(uint8_t rhport) {
  (void) rhport;
  PFIC_EnableIRQ(USB_IRQn); // Use CH58x interrupt
}

void dcd_int_disable(uint8_t rhport) {
  (void) rhport;
  PFIC_DisableIRQ(USB_IRQn); // Use CH58x interrupt
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
  (void) dev_addr;
  dcd_edpt_xfer(rhport, 0x80, NULL, 0); // ZLP status response
  //R8_USB_DEV_AD = dev_addr;
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
    R8_USB_DEV_AD = (uint8_t) request->wValue;
  }
  set_endpoint_tx_ctrl(0, UEP_T_RES_NAK, false);
  //data.ep0_tog = !data.ep0_tog;
  set_endpoint_rx_ctrl(0, UEP_R_RES_ACK, false);
  //data.ep0_tog = !data.ep0_tog;
  
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
        set_endpoint_rx_ctrl(ep, UEP_T_RES_TOUT, true);
      } else {
        set_endpoint_rx_ctrl(ep, UEP_R_RES_ACK, true);
      }
    } else {
      set_endpoint_t_len(ep, 0);
      set_endpoint_tx_ctrl(ep, UEP_T_RES_NAK, 1);
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
  //dcd_int_disable(rhport);
  xfer->valid = true;
  xfer->buffer = buffer;
  xfer->len = total_bytes;
  xfer->processed_len = 0;
  //dcd_int_enable(rhport);

  if (dir == TUSB_DIR_IN) {
    update_in(rhport, ep, true);
  }
  return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;
  uint8_t ep = tu_edpt_number(ep_addr);
  uint8_t dir = tu_edpt_dir(ep_addr);
  if (ep == 0) {
    if (dir == TUSB_DIR_OUT) {
      set_endpoint_rx_ctrl(ep,UEP_R_RES_STALL , false);
      data.ep0_tog_out = !data.ep0_tog_out;
    } else {
      //EP_TX_LEN(ep) = 0;
      set_endpoint_t_len(ep, 0);
      set_endpoint_tx_ctrl(ep, UEP_T_RES_STALL, false);
      data.ep0_tog_in = !data.ep0_tog_in;
    }
  } else {
    if (dir == TUSB_DIR_OUT) {
      set_endpoint_rx_ctrl(ep,UEP_R_RES_STALL , true);
    } else {
      set_endpoint_tx_ctrl(ep, UEP_T_RES_STALL, true);
    }
  }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;
  uint8_t ep = tu_edpt_number(ep_addr);
  uint8_t dir = tu_edpt_dir(ep_addr);
  if (ep == 0) {
    if (dir == TUSB_DIR_OUT) {
      set_endpoint_rx_ctrl(ep,UEP_R_RES_ACK , false);
      data.ep0_tog_out = !data.ep0_tog_out;
    }
  } else {
    if (dir == TUSB_DIR_OUT) {
      set_endpoint_rx_ctrl(ep,UEP_R_RES_ACK , true);
    } else {
      set_endpoint_tx_ctrl(ep, UEP_T_RES_NAK, true);
    }
  }
}
#endif
