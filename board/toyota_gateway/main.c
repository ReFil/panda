// ********************* Includes *********************
#include "../config.h"
#include "libc.h"

#include "main_declarations.h"
#include "critical.h"
#include "faults.h"

#include "drivers/registers.h"
#include "drivers/interrupts.h"
#include "drivers/llcan.h"
#include "drivers/llgpio.h"
#include "drivers/adc.h"

#include "board.h"

#include "drivers/clock.h"
#include "drivers/timer.h"

#include "gpio.h"
#include "crc.h"

// uncomment for usb debugging via debug_console.py
#define TGW_USB
#define DEBUG

#ifdef TGW_USB
  #include "drivers/uart.h"
  #include "drivers/usb.h"
#else
  // no serial either
  void puts(const char *a) {
    UNUSED(a);
  }
  void puth(unsigned int i) {
    UNUSED(i);
  }
  void puth2(unsigned int i) {
    UNUSED(i);
  }
#endif

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

#ifdef TGW_USB

#include "ibst/can.h"

// ********************* usb debugging *********************
void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

int usb_cb_ep1_in(void *usbdata, int len, bool hardwired) {
  UNUSED(hardwired);
  CAN_FIFOMailBox_TypeDef *reply = (CAN_FIFOMailBox_TypeDef *)usbdata;
  int ilen = 0;
  while (ilen < MIN(len/0x10, 4) && can_pop(&can_rx_q, &reply[ilen])) {
    ilen++;
  }
  return ilen*0x10;
}
// send on serial, first byte to select the ring
void usb_cb_ep2_out(void *usbdata, int len, bool hardwired) {
  UNUSED(hardwired);
  uint8_t *usbdata8 = (uint8_t *)usbdata;
  uart_ring *ur = get_ring_by_number(usbdata8[0]);
  if ((len != 0) && (ur != NULL)) {
    if ((usbdata8[0] < 2U)) {
      for (int i = 1; i < len; i++) {
        while (!putc(ur, usbdata8[i])) {
          // wait
        }
      }
    }
  }
}
// send on CAN
void usb_cb_ep3_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out_complete() {
  if (can_tx_check_min_slots_free(MAX_CAN_MSGS_PER_BULK_TRANSFER)) {
    usb_outep3_resume_if_paused();
  }
}

void usb_cb_enumeration_complete() {
  puts("USB enumeration complete\n");
  is_enumerated = 1;
}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired) {
  UNUSED(hardwired);
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest) {
    // **** 0xd1: enter bootloader mode
    case 0xd1:
      // this allows reflashing of the bootstub
      // so it's blocked over wifi
      switch (setup->b.wValue.w) {
        case 0:
          // only allow bootloader entry on debug builds
          #ifdef ALLOW_DEBUG
            if (hardwired) {
              puts("-> entering bootloader\n");
              enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
              NVIC_SystemReset();
            }
          #endif
          break;
        case 1:
          puts("-> entering softloader\n");
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
          break;
        default:
          puts("Bootloader mode invalid\n");
          break;
      }
      break;
    // **** 0xd8: reset ST
    case 0xd8:
      NVIC_SystemReset();
      break;
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) {
        break;
      }
      // read
      while ((resp_len < MIN(setup->b.wLength.w, MAX_RESP_LEN)) &&
                         getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    // **** 0xf1: Clear CAN ring buffer.
    case 0xf1:
      if (setup->b.wValue.w == 0xFFFFU) {
        puts("Clearing CAN Rx queue\n");
        can_clear(&can_rx_q);
      } else if (setup->b.wValue.w < BUS_MAX) {
        puts("Clearing CAN Tx queue\n");
        can_clear(can_queues[setup->b.wValue.w]);
      } else {
        puts("Clearing CAN CAN ring buffer failed: wrong bus number\n");
      }
      break;
    // **** 0xf2: Clear UART ring buffer.
    case 0xf2:
      {
        uart_ring * rb = get_ring_by_number(setup->b.wValue.w);
        if (rb != NULL) {
          puts("Clearing UART queue.\n");
          clear_uart_buff(rb);
        }
        break;
      }
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}

#endif

// ***************************** can port *****************************

// Toyota Checksum algorithm
uint8_t toyota_checksum(int addr, uint8_t *dat, int len){
  int cksum = 0;
  for(int ii = 0; ii < (len - 1); ii++){
    cksum = (cksum + dat[ii]); 
  }
  cksum += len;
  cksum += ((addr << 8U) & 0xFF); // idh
  cksum += ((addr >> 8U) & 0xFF); // idl
  return cksum & 0xFF;
}

#define CAN_UPDATE  0x341 //bootloader
#define COUNTER_CYCLE 0xFU

void CAN1_TX_IRQ_Handler(void) {
  process_can(0);
}

void CAN2_TX_IRQ_Handler(void) {
  process_can(1);
}

void CAN3_TX_IRQ_Handler(void) {
  process_can(2);
}

#define MAX_TIMEOUT 50U
uint32_t timeout = 0;

#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
#define FAULT_COUNTER 7U

uint8_t state = FAULT_STARTUP;

// flags
bool enable = 0;

// LKA_CMD 0x2E4
int steer_cmd = 0;
uint8_t steer_cnt = 0;
bool steer_req = 0;

// ACC_CONTROL 0x343
int acc_cmd = 0;
bool acc_cancel = 0;
bool permit_braking = 0;

// AEB_CMD 0x344
int aeb_cmd = 0;
uint8_t aeb_cnt = 0;
bool brkhld = 0;
uint8_t brkstate = 0;

// CRC8
const uint8_t crc_poly = 0x1D;  // standard crc8 SAE J1850
uint8_t crc8_lut_1d[256];

void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {

    CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR = CAN1->sFIFOMailBox[0].RIR;
    to_fwd.RDTR = CAN1->sFIFOMailBox[0].RDTR;

    uint16_t address = CAN1->sFIFOMailBox[0].RIR >> 21;

    #ifdef DEBUG
    puts("CAN1 RX: ");
    puth(address);
    puts("\n");
    #endif

    // CAN data buffer
    uint8_t dat[8];

    switch (address) {
      case CAN_UPDATE:
        if (GET_BYTES_04(&CAN1->sFIFOMailBox[0]) == 0xdeadface) {
          if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x0ab00b1e) {
            enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
            NVIC_SystemReset();
          } else if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x02b00b1e) {
            enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
            NVIC_SystemReset();
          } else {
            puts("Failed entering Softloader or Bootloader\n");
          }
        }
        break;
      case 0x20E: // TODO: figure out what the ctrl msgs should translate to
        // logic!
        UNUSED(dat);
        break;
      default:
        // FWD as-is
        to_fwd.RDLR = CAN1->sFIFOMailBox[0].RDLR;
        to_fwd.RDHR = CAN1->sFIFOMailBox[0].RDHR;
        break;
    }
    // send 
    can_send(&to_fwd, 2, false);
    // next
    can_rx(0);
    // CAN1->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN1);
  llcan_clear_send(CAN1);
}

void CAN2_RX0_IRQ_Handler(void) {
  while ((CAN2->RF0R & CAN_RF0R_FMP0) != 0) {

    CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR = CAN2->sFIFOMailBox[0].RIR;
    to_fwd.RDTR = CAN2->sFIFOMailBox[0].RDTR;

    uint16_t address = CAN2->sFIFOMailBox[0].RIR >> 21;
    
    #ifdef DEBUG
    puts("CAN2 RX: ");
    puth(address);
    puts("\n");
    #endif
    
    // CAN data buffer
    uint8_t dat[8];

    switch (address) {
      case 0x343: 
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN2->sFIFOMailBox[0], i);
        }
        if(dat[7] == toyota_checksum(address, dat, 8)) {
          if (enable){ 
            // modify this message before sending to the car only if requested
            dat[1] = (acc_cmd >> 8U);
            dat[2] = (acc_cmd & 0xFF);
            dat[3] |= (permit_braking << 5U);
            dat[7] = toyota_checksum(address, dat, 8);
          }
          to_fwd.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
          to_fwd.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
        }
        break;
      case 0x344: 
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN2->sFIFOMailBox[0], i);
        }
        if(dat[7] == toyota_checksum(address, dat, 8)) {
          if (enable){ 
            // modify this message before sending to the car only if requested
            dat[1] = (aeb_cmd >> 8U); // 10 bit msg
            dat[2] = (aeb_cmd & 0xFF);
            dat[7] = toyota_checksum(address, dat, 8);
          }
          to_fwd.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
          to_fwd.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
        }
        break;
      default:
        // FWD as-is
        to_fwd.RDLR = CAN2->sFIFOMailBox[0].RDLR;
        to_fwd.RDHR = CAN2->sFIFOMailBox[0].RDHR;
        break;
    }
    // send 
    can_send(&to_fwd, 1, false);
    // next
    can_rx(1);
  }
}

void CAN2_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN2);
  llcan_clear_send(CAN2);
}

void CAN3_RX0_IRQ_Handler(void) {
  while ((CAN3->RF0R & CAN_RF0R_FMP0) != 0) {
    uint16_t address = CAN3->sFIFOMailBox[0].RIR >> 21;
    #ifdef DEBUG
    puts("CAN3 RX: ");
    puth(address);
    puts("\n");
    #else
    UNUSED(address);
    #endif

    // next
    can_rx(2);
  }
}

void CAN3_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN3);
  llcan_clear_send(CAN3);
}

void TIM3_IRQ_Handler(void) {

  //send to EON
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[5];
    dat[4] = 0;
    dat[3] = 0;
    dat[2] = 0;
    dat[1] = ((state & 0xFU) << 4);
    dat[0] = lut_checksum(dat, 5, crc8_lut_1d);

    CAN_FIFOMailBox_TypeDef to_send;
    to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    to_send.RDHR = dat[4];
    to_send.RDTR = 8;
    to_send.RIR = (0x20F << 21) | 1U;
    can_send(&to_send, 0, false);

  }
  else {
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG
      puts("CAN1 MISS1\n");
    #endif
  }
  // blink the LED

  TIM3->SR = 0;

  // up timeout
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
  } else {
    timeout += 1U;
  }
}

// ***************************** main code *****************************


void gw(void) {
  // read/write
  watchdog_feed();

}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN2_TX_IRQn, CAN2_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN2_RX0_IRQn, CAN2_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN2_SCE_IRQn, CAN2_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN3_TX_IRQn, CAN3_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_RX0_IRQn, CAN3_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_SCE_IRQn, CAN3_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)

  // Should run at around 732Hz (see init below)
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)

  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();
  detect_configuration();
  detect_board_type();

  // init board
  current_board->init();
  // enable USB
  #ifdef TGW_USB
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  usb_init();
  #endif

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan1 speed");
  }
  llcan_speed_set = llcan_set_speed(CAN2, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan2 speed");
  }
  llcan_speed_set = llcan_set_speed(CAN3, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan3 speed");
  }

  bool ret = llcan_init(CAN1);
  ret = llcan_init(CAN2);
  ret = llcan_init(CAN3);
  UNUSED(ret);

  gen_crc_lookup_table(crc_poly, crc8_lut_1d);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 7);
  NVIC_EnableIRQ(TIM3_IRQn);

  // power on ibooster. needs to power on AFTER sending CAN to prevent ibst state from being 4
  set_gpio_mode(GPIOB, 12, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 12, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output(GPIOB, 12, 1);

  //Brake switch relay
  set_gpio_mode(GPIOB, 13, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 13, OUTPUT_TYPE_PUSH_PULL);

  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main pedal loop
  while (1) {
    gw();
  }

  return 0;
}
