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

#include "board.h"

#include "drivers/clock.h"
#include "drivers/timer.h"

#include "gpio.h"
#include "crc.h"

#include "drivers/uart.h"
#include "drivers/usb.h"

#define CAN1
#define CAN2
#define CAN3

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

// ********************* usb debugging *********************
void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

int usb_cb_ep1_in(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
  return 0;
}
void usb_cb_ep2_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out_complete(void) {}
void usb_cb_enumeration_complete(void) {}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired) {
  UNUSED(hardwired);
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest) {
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
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}


// ***************************** can port *****************************

// addresses to be used on CAN
#define CAN_UPDATE  0x341
#define COUNTER_CYCLE 0xFU

void CAN1_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN1->TSR |= CAN_TSR_RQCP0;
}

void CAN2_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN2->TSR |= CAN_TSR_RQCP0;
}

void CAN3_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN3->TSR |= CAN_TSR_RQCP0;
}

// two independent values
uint8_t current_speed = 0;
uint16_t q_target_ext = 0;
bool q_target_ext_qf = 0;

uint8_t can1_count_out = 0;
uint8_t can1_count_in;
uint8_t can2_count_out_1 = 0;
uint8_t can2_count_out_2 = 0;
uint8_t can2_count_in_1;
uint8_t can2_count_in_2;

#define MAX_TIMEOUT 10U
uint32_t timeout = 0;
uint32_t current_index = 0;

#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
uint8_t state = FAULT_STARTUP;

#define NO_EXTFAULT1 0U
#define EXTFAULT1_CHECKSUM1 1U
#define EXTFAULT1_CHECKSUM2 2U
#define EXTFAULT1_SCE 3U
#define EXTFAULT1_COUNTER1 4U
#define EXTFAULT1_COUNTER2 5U
#define EXTFAULT1_TIMEOUT 6U

#define NO_EXTFAULT2 0U
#define EXTFAULT2_CHECKSUM1 1U
#define EXTFAULT2_CHECKSUM2 2U
#define EXTFAULT2_SCE 3U
#define EXTFAULT2_COUNTER1 4U
#define EXTFAULT2_COUNTER2 5U
#define EXTFAULT2_TIMEOUT 6U

const uint8_t crc_poly = 0x1D;  // standard crc8 SAE J1850
uint8_t crc8_lut_1d[256];


void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {
    puts("CAN1 RX\n");
    uint16_t address = CAN1->sFIFOMailBox[0].RIR >> 21;
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
      case 0x20E:
        uint8_t dat[8];
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        break;

      default:
    }
    // next
    CAN1->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN1);
}

void CAN2_RX0_IRQ_Handler(void) {
  while ((CAN2->RF0R & CAN_RF0R_FMP0) != 0) {
    puts("CAN2 RX\n");
    uint16_t address = CAN2->sFIFOMailBox[0].RIR >> 21;
    switch (address) {
      case 0x38E:
        uint8_t dat[8];
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        break;
      case 0x38F:
        uint8_t dat[8];
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        break;
      default:
    }
    // next
    CAN2->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN2_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN2);
}

void CAN3_RX0_IRQ_Handler(void) {
  while ((CAN3->RF0R & CAN_RF0R_FMP0) != 0) {
    puts("CAN3 RX\n");
    uint16_t address = CAN3->sFIFOMailBox[0].RIR >> 21;

    // next
    CAN3->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN3_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN3);
}





void TIM3_IRQ_Handler(void) {
  // check timer for sending the user pedal and clearing the CAN
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8];
    dat[1] = (pdl0 >> 0) & 0xFFU;
    dat[2] = (pdl0 >> 8) & 0xFFU;
    dat[3] = (pdl1 >> 0) & 0xFFU;
    dat[4] = (pdl1 >> 8) & 0xFFU;
    dat[5] = ((state & 0xFU) << 4) | pkt_idx;
    dat[0] = lut_checksum(dat, CAN_GAS_SIZE, crc8_lut_1d);
    CAN->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    CAN->sTxMailBox[0].TDHR = dat[4] | (dat[5] << 8);
    CAN->sTxMailBox[0].TDTR = 6;  // len of packet is 5
    CAN->sTxMailBox[0].TIR = (CAN_GAS_OUTPUT << 21) | 1U;
    ++pkt_idx;
    pkt_idx &= COUNTER_CYCLE;
  }
  else {
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG
      puts("CAN MISS\n");
    #endif
  }

  // blink the LED
  current_board->set_led(LED_GREEN, led_value);
  led_value = !led_value;

  TIM3->SR = 0;

  // up timeout for gas set
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
  } else {
    timeout += 1U;
  }
}

// ***************************** main code *****************************

void pedal(void) {
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
  usb_init();

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
  timer_init(TIM3, 8);
  NVIC_EnableIRQ(TIM3_IRQn);


  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main pedal loop
  while (1) {
    ibst();
  }

  return 0;
}
