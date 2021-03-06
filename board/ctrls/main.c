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
#include "drivers/dac.h"
#include "drivers/timer.h"

#include "gpio.h"
#include "crc.h"

#include "drivers/uart.h"
#include "drivers/usb.h"

#define CAN CAN1

//#define ADC
//#define ENCODER
#define BUTTONS





#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

// ********************* serial debugging *********************

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
#define CAN_CTRLS_INPUT  0x366
#define CAN_CTRLS_OUTPUT 0x365U
#define CAN_CTRLS_SIZE 3

void CAN1_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN->TSR |= CAN_TSR_RQCP0;
}

// set state
bool enabled;
uint8_t setspeed;

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

const uint8_t crc_poly = 0x1D;  // standard crc8
uint8_t crc8_lut_1d[256];

void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN->RF0R & CAN_RF0R_FMP0) != 0) {
    #ifdef DEBUG
      puts("CAN RX\n");
    #endif
    int address = CAN->sFIFOMailBox[0].RIR >> 21;
    if (address == CAN_CTRLS_INPUT) {
      // softloader entry
      if (GET_BYTES_04(&CAN->sFIFOMailBox[0]) == 0xdeadface) {
        if (GET_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x0ab00b1e) {
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
        } else if (GET_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x02b00b1e) {
          enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
          NVIC_SystemReset();
        } else {
          puts("Failed entering Softloader or Bootloader\n");
        }
      }

      // normal packet
      uint8_t dat[8];
      for (int i=0; i<8; i++) {
        dat[i] = GET_BYTE(&CAN->sFIFOMailBox[0], i);
      }
      if (lut_checksum(dat, 3, crc8_lut_1d) == dat[0]) {
        enabled = ((dat[1] >> 7) & 1U) != 0U;
        setspeed = dat[2];
        #ifdef DEBUG
          puts("enable detected");
          puth(setspeed);
          puts("\n");
        #endif
        if (enabled) {
        }
        else {
            state = NO_FAULT;
        }
        // clear the timeout
        timeout = 0;
      } else {
        // wrong checksum = fault
        state = FAULT_BAD_CHECKSUM;
      }
    }
    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN);
}


volatile uint8_t btns[4]; // order set, cancel, speed up, speed down
volatile uint8_t oldbtns[4];

bool led_value = 0;

void update_eon(void) {
  #ifdef DEBUG
    puth(TIM3->CNT);
    puts(" ");
    puth(state);
    puts(" ");
    puth(set_btn);
    puts("\n");
  #endif

  // check timer for sending the user pedal and clearing the CAN
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[3];
    dat[0] = 0;
    dat[1] = (btns[0] | btns[1] << 1 | btns[2] << 2 | btns[3] << 3) & 0xFFU;
    dat[2] = (state & 0xFU);
    dat[0] = lut_checksum(dat, 3, crc8_lut_1d);
    CAN->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16);
    CAN->sTxMailBox[0].TDTR = 3;  // len of packet is 3
    CAN->sTxMailBox[0].TIR = (CAN_CTRLS_OUTPUT << 21) | 1U;
  } else {
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

volatile uint8_t encoderCount = 2;

void TIM3_IRQ_Handler(void) {
  static uint8_t ABs = 0;
  ABs = (ABs << 2) & 0x0f; //left 2 bits now contain the previous AB key read-out;
  ABs |= (get_gpio_input(GPIOA, 8) << 1) | get_gpio_input(GPIOA, 9);
  encoderCount = 2;
  switch (ABs)
  {
    case 0x0d:
      encoderCount = 3;
      break;
    case 0x0e:
      encoderCount = 1;
      break;
  }
}


// ***************************** main code *****************************

void loop(void) {
  // read/write

#ifdef ADC
  uint16_t value;
  value = adc_get(ADCCHAN_ACCEL0);
  puth(value);
  puts("\n");
  if(value < 1)
  {}
  else if((value < 2) && (value > 1))
  {}
  else if((value < 3) && (value > 2))
  {}
#endif
#ifdef ENCODER
  switch (encoderCount) {
    case 1:
      btns[2] = 1;
      btns[3] = 0;
      break;
    case 2:
      btns[2] = 0;
      btns[3] = 0;
      break;
    case 3:
      btns[2] = 0;
      btns[3] = 1;
      break;
  }
  btns[0] = !get_gpio_input(GPIOA, 10);
  btns[1] = !get_gpio_input(GPIOC, 0);
#endif
#ifdef BUTTONS
  btns[0] = !get_gpio_input(GPIOA, 8);
  btns[1] = !get_gpio_input(GPIOC, 0);
  btns[2] = !get_gpio_input(GPIOA, 10);
  btns[3] = !get_gpio_input(GPIOA, 9);
#endif

  if(btns[0] && enabled && !oldbtns[0]){ //if set button pressed but system is already enabled
    btns[0] = 0;
    btns[1] = 1; //cancel instead
  }
  if(btns[0] != oldbtns[0] || btns[1] != oldbtns[1] || btns[2] != oldbtns[2] || btns[3] != oldbtns[3]) {//if button values have changed
    update_eon(); //send new button values to eon
  }

  oldbtns[0] = btns[0];
  oldbtns[1] = btns[1];
  oldbtns[2] = btns[2];
  oldbtns[3] = btns[3];

  watchdog_feed();
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)

  // Should run at around 732Hz (see init below)
  #ifdef ENCODER
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)
  #endif

  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();
  detect_configuration();
  detect_board_type();

  // init board
  current_board->init();

#ifdef CTRLS_USB
  // enable USB
  usb_init();
#endif

#ifdef ADC
  adc_init();
  set_gpio_mode(GPIOC, 0, MODE_ANALOG);
#endif
#ifdef BUTTONS
  set_gpio_mode(GPIOC, 0, MODE_INPUT);
  set_gpio_pullup(GPIOC, 0, PULL_UP);
#endif
  set_gpio_mode(GPIOA, 8, MODE_INPUT);
  set_gpio_mode(GPIOA, 9, MODE_INPUT);
  set_gpio_mode(GPIOA, 10, MODE_INPUT);
  set_gpio_pullup(GPIOA, 8, PULL_UP);
  set_gpio_pullup(GPIOA, 9, PULL_UP);
  set_gpio_pullup(GPIOA, 10, PULL_UP);

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan speed");
  }

  bool ret = llcan_init(CAN1);
  UNUSED(ret);

  // 48mhz / 65536 ~= 732
#ifdef ENCODER
  timer_init(TIM3, 15);
  NVIC_EnableIRQ(TIM3_IRQn);
#endif
  btns[0] = 0;
  btns[1] = 0;
  btns[2] = 0;
  btns[3] = 0;

  gen_crc_lookup_table(crc_poly, crc8_lut_1d);
  update_eon();
  watchdog_init();
  current_board->set_led(LED_GREEN, 1);

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main CTRLS loop
  while (1) {
    loop();
  }

  return 0;
}
