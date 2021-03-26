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

#include "drivers/uart.h"
#include "drivers/usb.h"


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
#define CAN_UPDATE  0x341 //bootloader
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

uint16_t output_rod_target = 0;
bool brake_applied = 0;
bool brake_ok = 0;

uint8_t ibst_status;
uint8_t ext_req_status;


uint8_t can1_count_out = 0;
uint8_t can1_count_in;
uint8_t can2_count_out_1 = 0;
uint8_t can2_count_out_2 = 0;
uint8_t can2_count_in_1;
uint8_t can2_count_in_2;
uint8_t can2_count_in_3;

#define MAX_TIMEOUT 10U
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

#define NO_EXTFAULT1 0U
#define EXTFAULT1_CHECKSUM1 1U
#define EXTFAULT1_CHECKSUM2 2U
#define EXTFAULT1_CHECKSUM3 3U
#define EXTFAULT1_SCE 4U
#define EXTFAULT1_COUNTER1 5U
#define EXTFAULT1_COUNTER2 6U
#define EXTFAULT1_COUNTER3 7U
#define EXTFAULT1_TIMEOUT 8U
#define EXTFAULT1_SEND1 9U
#define EXTFAULT1_SEND2 10U
#define EXTFAULT1_SEND3 11U

uint8_t can2state = NO_EXTFAULT1;

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
      case 0x20E: ;
        uint64_t data; //sendESP_private2
        uint8_t *dat = (uint8_t *)&data;
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        uint8_t index = dat[1] & COUNTER_CYCLE;
        if(dat[0] == lut_checksum(dat, 8, crc8_lut_1d)) {
          if (((can1_count_in + 1U) & COUNTER_CYCLE) == index) {
            //if counter and checksum valid accept commands
            q_target_ext = (data >> 14);
            q_target_ext_qf = (data >> 12);
            can1_count_in++;
          }
          else {
            state = FAULT_COUNTER;
          }
        }
        else {
          state = FAULT_BAD_CHECKSUM;
        }
        break;
      case 0x366: ;
        uint8_t dat2[4];
        for (int i=0; i<4; i++) {
          dat2[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        if(dat2[0] == lut_checksum(dat2, 4, crc8_lut_1d)) {
          current_speed = dat2[3];
        }
        else {
          state = FAULT_BAD_CHECKSUM;
        }
      default: ;
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
    /*  case 0x391:
        uint8_t dat[5];
        for (int i=0; i<5; i++) {
          dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        uint64_t *data = (uint8_t *)&dat;
        uint8_t index = dat[6] & COUNTER_CYCLE;
        if(dat[0] = lut_checksum(dat, 8, crc8_lut_1d)) {
          if (((can2_count_in1 + 1U) & COUNTER_CYCLE) == index) {
            //if counter and checksum valid accept commands
            ebr_mode = (dat[1] >> 4) & 0x7;
            ebr_system_mode = (data >> 15) & 0x7;
            can2_count_in1++;
          }
          else {
            state = EXTFAULT1_COUNTER1;
          }
        }
        else {
          state = EXTFAULT1_CHECKSUM1;
        }
        break;*/
      case 0x38E: ;
        uint64_t data; //sendESP_private2
        uint8_t *dat = (uint8_t *)&data;
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        uint8_t index = dat[6] & COUNTER_CYCLE;
        if(dat[0] == lut_checksum(dat, 8, crc8_lut_1d)) {
          if (((can2_count_in_1 + 1U) & COUNTER_CYCLE) == index) {
            //if counter and checksum valid accept commands
            output_rod_target = ((data >> 24) & 0x3FU);
            can2_count_in_1++;
          }
          else {
            state = EXTFAULT1_COUNTER2;
          }
        }
        else {
          state = EXTFAULT1_CHECKSUM2;
        }
        break;
      case 0x38F: ;
        uint64_t data2; //sendESP_private2
        uint8_t *dat2 = (uint8_t *)&data2;
        for (int i=0; i<8; i++) {
          dat2[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        uint8_t index2 = dat2[1] & COUNTER_CYCLE;
        if(dat2[0] == lut_checksum(dat2, 8, crc8_lut_1d)) {
          if (((can2_count_in_3 + 1U) & COUNTER_CYCLE) == index2) {
            //if counter and checksum valid accept commands
            ibst_status = (data2 >> 19) & 0x7;
            brake_applied = (dat2[2] & 0x1) | !((dat2[2] >> 1) & 0x1); //Sends brake applied if ibooster says brake applied or if there's a fault with the brake sensor, assumes worst case scenario

            can2_count_in_3++;
          }
          else {
            state = EXTFAULT1_COUNTER3;
          }
        }
        else {
          state = EXTFAULT1_CHECKSUM3;
        }
        break;
      default: ;
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
    //uint16_t address = CAN3->sFIFOMailBox[0].RIR >> 21;

    // next
    CAN3->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN3_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN3);
}


bool sent;

#define P_LIMIT_EXTERNAL 120

#define P_EST_MAX 0
#define P_EST_MAX_QF 1
#define VEHICLE_QF 1
#define IGNITION_ON 0

#define P_TARGET_DRIVER 0
#define P_TARGET_DRIVER_QF 0
#define ABS_ACTIVE 0
#define P_MC 0
#define P_MC_QF 1


void TIM3_IRQ_Handler(void) {
  // check timer for sending the user pedal and clearing the CAN
  if ((CAN2->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8]; //sendESP_private3
    uint16_t pTargetDriver = P_TARGET_DRIVER * 4;
    dat[2] = pTargetDriver & 0xFFU;
    dat[3] = (pTargetDriver & 0x3U) >> 8;
    dat[4] = 0x0;
    dat[5] = 0x0;
    dat[6] = (uint8_t) P_MC_QF >> 5;
    dat[7] = 0x0;
    dat[1] = can2_count_out_1;
    dat[0] = lut_checksum(dat, 8, crc8_lut_1d);
    CAN2->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    CAN2->sTxMailBox[0].TDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
    CAN2->sTxMailBox[0].TDTR = 8;  // len of packet is 5
    CAN2->sTxMailBox[0].TIR = (0x38D << 21) | 1U;

  }
  else {
    // old can packet hasn't sent!
    state = EXTFAULT1_SEND1;
    #ifdef DEBUG
      puts("CAN2 MISS1\n");
    #endif
  }
  if ((CAN2->TSR & CAN_TSR_TME1) == CAN_TSR_TME1) {
    uint64_t data; //sendESP_private2
    uint16_t p_limit_external = P_LIMIT_EXTERNAL * 2;
    uint8_t *dat = (uint8_t *)&data;

    data = (uint64_t) (p_limit_external & 0x1FF) << 16;
    data |= (uint64_t) q_target_ext << 28;
    data |= (uint64_t) q_target_ext_qf << 44;

    dat[1] = can2_count_out_1;
    dat[0] = lut_checksum(dat, 8, crc8_lut_1d);
    CAN2->sTxMailBox[1].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    CAN2->sTxMailBox[1].TDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16);
    CAN2->sTxMailBox[1].TDTR = 7;  // len of packet is 5
    CAN2->sTxMailBox[1].TIR = (0x38C << 21) | 1U;
    can2_count_out_1++;
    can2_count_out_1 &= COUNTER_CYCLE;
  }
  else {
    // old can packet hasn't sent!
    state = EXTFAULT1_SEND2;
    #ifdef DEBUG
      puts("CAN2 MISS2\n");
    #endif
  }
  if (((CAN2->TSR & CAN_TSR_TME2) == CAN_TSR_TME2) & sent) {
    uint64_t data; //sendESP_private1 every 20ms
    uint8_t *dat = (uint8_t *)&data;

    data = P_EST_MAX << 16;
    data |= P_EST_MAX_QF << 24;
    data |= ((((uint32_t) current_speed*16)/9)& 0x3FFF) << 24;
    data |= (uint64_t) VEHICLE_QF << 40;
    data |= (uint64_t) IGNITION_ON << 43;

    dat[1] = can2_count_out_2;
    dat[0] = lut_checksum(dat, 8, crc8_lut_1d);
    CAN2->sTxMailBox[2].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    CAN2->sTxMailBox[2].TDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
    CAN2->sTxMailBox[2].TDTR = 8;  // len of packet is 5
    CAN2->sTxMailBox[2].TIR = (0x38B << 21) | 1U;
    can2_count_out_2++;
    can2_count_out_2 &= COUNTER_CYCLE;
  }
  else {
    // old can packet hasn't sent!
    state = EXTFAULT1_SEND3;
    #ifdef DEBUG
      puts("CAN2 MISS3\n");
    #endif
  }
  sent = !sent;


  //send to EON
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[5];
    brake_ok = (ibst_status && 0x7);
    uint16_t shftpedpos = (output_rod_target << 2);
    dat[2] = brake_ok | brake_applied << 1 | (shftpedpos & 0xFC);
    dat[3] = ((output_rod_target & 0x00) >> 8);
    dat[4] = (can2state & 0xFU) << 4;

    dat[1] = ((state & 0xFU) << 4) | can1_count_out;
    dat[0] = lut_checksum(dat, 8, crc8_lut_1d);
    CAN1->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    CAN1->sTxMailBox[0].TDHR = dat[4];
    CAN1->sTxMailBox[0].TDTR = 5;  // len of packet is 5
    CAN1->sTxMailBox[0].TIR = (0x38D << 21) | 1U;
    can1_count_out++;
    can1_count_out &= COUNTER_CYCLE;
  }
  else {
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG
      puts("CAN2 MISS1\n");
    #endif
  }
  // blink the LED

  TIM3->SR = 0;

  // up timeout for gas set
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
  } else {
    timeout += 1U;
  }
}

// ***************************** main code *****************************

void ibst(void) {
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
  timer_init(TIM3, 7);
  NVIC_EnableIRQ(TIM3_IRQn);
  //power on ibooster
  set_gpio_mode(GPIOB, 12, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 12, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output(GPIOB, 12, 1);


  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main pedal loop
  while (1) {
    ibst();
  }

  return 0;
}
