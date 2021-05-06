static void ocelot_init(int16_t param) {
  UNUSED(param);
  controls_allowed = true;
  relay_malfunction_reset();
}

static int ocelot_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  UNUSED(bus_num);
  UNUSED(to_fwd);
  return -1;
}


static int ocelot_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  UNUSED(to_send);
  return true;
}

int ocelot_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  UNUSED(to_push);
  return true;
}

static int ocelot_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return true;
}

const safety_hooks ocelot_hooks = {
  .init = ocelot_init,
  .rx = ocelot_rx_hook,
  .tx = ocelot_tx_hook,
  .tx_lin = ocelot_tx_lin_hook,
  .fwd = ocelot_fwd_hook,
};
