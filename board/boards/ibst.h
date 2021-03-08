// /////////// //
// ibooster gateway //
// /////////// //

void ibst_enable_can_transceiver(uint8_t transceiver, bool enabled) {
  switch (transceiver){
    case 1U:
      set_gpio_output(GPIOC, 1, !enabled);
      break;
    case 2U:
      set_gpio_output(GPIOC, 13, !enabled);
      break;
    case 3U:
      set_gpio_output(GPIOA, 0, !enabled);
      break;
    default:
      puts("Invalid CAN transceiver ("); puth(transceiver); puts("): enabling failed\n");
      break;
  }
}

void ibst_enable_can_transceivers(bool enabled) {
  uint8_t t1 = enabled ? 1U : 2U;  // leave transceiver 1 enabled to detect CAN ignition
  for(uint8_t i=t1; i<=3U; i++) {
    ibst_enable_can_transceiver(i, enabled);
  }
}

void ibst_set_led(uint8_t color, bool enabled) {
  UNUSED(color);
  UNUSED(enabled);
}

void ibst_set_usb_power_mode(uint8_t mode){
  UNUSED(mode);
}

void ibst_set_gps_mode(uint8_t mode) {
  UNUSED(mode);
}

void ibst_set_can_mode(uint8_t mode){
  switch (mode) {
    case CAN_MODE_NORMAL:
      set_gpio_alternate(GPIOB, 8, GPIO_AF8_CAN1);
      set_gpio_alternate(GPIOB, 9, GPIO_AF8_CAN1);
      // B5,B6: normal CAN2 mode
      set_gpio_alternate(GPIOB, 5, GPIO_AF9_CAN2);
      set_gpio_alternate(GPIOB, 6, GPIO_AF9_CAN2);

      // A8,A15: normal CAN3 mode
      set_gpio_alternate(GPIOA, 8, GPIO_AF11_CAN3);
      set_gpio_alternate(GPIOA, 15, GPIO_AF11_CAN3);
      break;
    default:
      puts("Tried to set unsupported CAN mode: "); puth(mode); puts("\n");
      break;
  }
}

uint32_t ibst_read_current(void){
  return 0U;
}

void ibst_usb_power_mode_tick(uint32_t uptime){
  UNUSED(uptime);
}

void ibst_set_ir_power(uint8_t percentage){
  UNUSED(percentage);
}

void ibst_set_fan_power(uint8_t percentage){
  UNUSED(percentage);
}

bool ibst_check_ignition(void){
  // ignition is on PA1
  return 0U;
}

void ibst_set_phone_power(bool enabled){
  UNUSED(enabled);
}

void ibst_set_clock_source_mode(uint8_t mode){
  UNUSED(mode);
}

void ibst_set_siren(bool enabled){
  UNUSED(enabled);
}

void ibst_init(void) {
  common_init_gpio();

  // Enable CAN transceivers
  ibst_enable_can_transceivers(true);
  // Set normal CAN mode
  ibst_set_can_mode(CAN_MODE_NORMAL);
}

const harness_configuration ibst_harness_config = {
  .has_harness = false
};

const board board_ibst = {
  .board_type = "White",
  .harness_config = &ibst_harness_config,
  .init = ibst_init,
  .enable_can_transceiver = ibst_enable_can_transceiver,
  .enable_can_transceivers = ibst_enable_can_transceivers,
  .set_led = ibst_set_led,
  .set_usb_power_mode = ibst_set_usb_power_mode,
  .set_gps_mode = ibst_set_gps_mode,
  .set_can_mode = ibst_set_can_mode,
  .usb_power_mode_tick = ibst_usb_power_mode_tick,
  .check_ignition = ibst_check_ignition,
  .read_current = ibst_read_current,
  .set_fan_power = ibst_set_fan_power,
  .set_ir_power = ibst_set_ir_power,
  .set_phone_power = ibst_set_phone_power,
  .set_clock_source_mode = ibst_set_clock_source_mode,
  .set_siren = ibst_set_siren
};
