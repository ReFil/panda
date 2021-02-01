// ///// //
// ctrls //
// ///// //

void ctrls_enable_can_transceiver(uint8_t transceiver, bool enabled) {
  switch (transceiver){
    case 1:
      set_gpio_output(GPIOB, 3, !enabled);
      break;
    default:
      puts("Invalid CAN transceiver ("); puth(transceiver); puts("): enabling failed\n");
      break;
  }
}

void ctrls_enable_can_transceivers(bool enabled) {
  ctrls_enable_can_transceiver(1U, enabled);
}

void ctrls_set_led(uint8_t color, bool enabled) {
  switch (color){
    case LED_RED:
      set_gpio_output(GPIOB, 10, !enabled);
      break;
     case LED_GREEN:
      set_gpio_output(GPIOB, 11, !enabled);
      break;
    default:
      break;
  }
}

void ctrls_set_usb_power_mode(uint8_t mode){
  usb_power_mode = mode;
  puts("Trying to set USB power mode on ctrls. This is not supported.\n");
}

void ctrls_set_gps_mode(uint8_t mode) {
  UNUSED(mode);
  puts("Trying to set ESP/GPS mode on ctrls. This is not supported.\n");
}

void ctrls_set_can_mode(uint8_t mode){
  switch (mode) {
    case CAN_MODE_NORMAL:
      break;
    default:
      puts("Tried to set unsupported CAN mode: "); puth(mode); puts("\n");
      break;
  }
}

void ctrls_usb_power_mode_tick(uint32_t uptime){
  UNUSED(uptime);
  // Not applicable
}

bool ctrls_check_ignition(void){
  // not supported on ctrls
  return false;
}

uint32_t ctrls_read_current(void){
  // No current sense on ctrls
  return 0U;
}

void ctrls_set_ir_power(uint8_t percentage){
  UNUSED(percentage);
}

void ctrls_set_fan_power(uint8_t percentage){
  UNUSED(percentage);
}

void ctrls_set_phone_power(bool enabled){
  UNUSED(enabled);
}

void ctrls_set_clock_source_mode(uint8_t mode){
  UNUSED(mode);
}

void ctrls_set_siren(bool enabled){
  UNUSED(enabled);
}

void ctrls_init(void) {
  common_init_gpio();

  // C0, C1: Throttle inputs
#ifdef ADC
  set_gpio_mode(GPIOC, 0, MODE_ANALOG);
#else
  set_gpio_mode(GPIOC, 0, MODE_INPUT);
  set_gpio_pullup(GPIO, 0, PULL_UP);
#endif
  set_gpio_mode(GPIOA, 8, MODE_INPUT);
  set_gpio_mode(GPIOA, 9, MODE_INPUT);
  set_gpio_mode(GPIOA, 10, MODE_INPUT);

  set_gpio_pullup(GPIOA, 8, PULL_UP);
  set_gpio_pullup(GPIOA, 9, PULL_UP);
  set_gpio_pullup(GPIOA, 10, PULL_UP);
  // Enable transceiver
  ctrls_enable_can_transceivers(true);

  // Disable LEDs
  ctrls_set_led(LED_RED, false);
  ctrls_set_led(LED_GREEN, false);
}

const harness_configuration ctrls_harness_config = {
  .has_harness = false
};

const board board_ctrls = {
  .board_type = "Ctrls",
  .harness_config = &ctrls_harness_config,
  .init = ctrls_init,
  .enable_can_transceiver = ctrls_enable_can_transceiver,
  .enable_can_transceivers = ctrls_enable_can_transceivers,
  .set_led = ctrls_set_led,
  .set_usb_power_mode = ctrls_set_usb_power_mode,
  .set_gps_mode = ctrls_set_gps_mode,
  .set_can_mode = ctrls_set_can_mode,
  .usb_power_mode_tick = ctrls_usb_power_mode_tick,
  .check_ignition = ctrls_check_ignition,
  .read_current = ctrls_read_current,
  .set_fan_power = ctrls_set_fan_power,
  .set_ir_power = ctrls_set_ir_power,
  .set_phone_power = ctrls_set_phone_power,
  .set_clock_source_mode = ctrls_set_clock_source_mode,
  .set_siren = ctrls_set_siren
};
