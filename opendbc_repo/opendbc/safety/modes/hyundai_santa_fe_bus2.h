#pragma once

// The retrofit radar controls the cadence and the rolling counter. The host
// can supply one bounded request for a fresh stock cycle, but cannot transmit
// SCC12 by itself under this safety model.
#define SANTA_FE_SCC12 0x421U
#define SANTA_FE_FCA11 0x38DU
#define SANTA_FE_STOCK_BUS 2U
#define SANTA_FE_CAR_BUS 0U
#define SANTA_FE_SOURCE_MAX_AGE_US 60000U
#define SANTA_FE_FCA_PAIR_MAX_AGE_US 15000U
#define SANTA_FE_REQUEST_MAX_AGE_US 40000U

static struct {
  bool have_scc;
  bool have_fca;
  bool fca_clear;
  bool pending;
  bool replaced;
  uint8_t scc_counter;
  uint8_t fca_counter;
  uint32_t last_scc;
  uint32_t last_fca;
  uint32_t staged;
  CANPacket_t request;
} santa_fe_scc;

static bool santa_fe_source_healthy(uint32_t now) {
  return santa_fe_scc.have_scc && santa_fe_scc.have_fca && santa_fe_scc.fca_clear &&
         safety_get_ts_elapsed(now, santa_fe_scc.last_scc) <= SANTA_FE_SOURCE_MAX_AGE_US &&
         safety_get_ts_elapsed(now, santa_fe_scc.last_fca) <= SANTA_FE_SOURCE_MAX_AGE_US;
}

static void santa_fe_yield_to_stock(void) {
  santa_fe_scc.pending = false;
  santa_fe_scc.replaced = false;
  controls_allowed = false;
  cruise_button_prev = HYUNDAI_BTN_NONE;
}

static bool santa_fe_tx_hook(const CANPacket_t *msg) {
  bool allowed = hyundai_tx_hook(msg);
  if (msg->addr == SANTA_FE_SCC12) {
    allowed &= !msg->fd && !msg->extended && ((msg->data[1] & 0x7FU) == 0x20U) &&
               (hyundai_compute_checksum(msg) == hyundai_get_checksum(msg));
  }
  return allowed;
}

static bool santa_fe_stage_request(const CANPacket_t *msg) {
  const bool consumed = (msg->addr == SANTA_FE_SCC12) && (msg->bus == SANTA_FE_CAR_BUS);
  if (consumed) {
    uint32_t now = microsecond_timer_get();
    santa_fe_scc.pending = get_longitudinal_allowed() && !brake_pressed && !brake_pressed_prev &&
                           aol_rx_healthy() && santa_fe_source_healthy(now);
    if (santa_fe_scc.pending) {
      santa_fe_scc.request = *msg;
      santa_fe_scc.staged = now;
    }
  }
  return consumed;
}

static bool santa_fe_fca_crc_valid(const CANPacket_t *msg) {
  uint8_t crc = 0xFDU ^ 0xDFU;  // Match the initial-register convention of Hyundai's CRC8 helper.
  for (int i = 0; i < 7; i++) {
    crc = crc8_update(crc, msg->data[i], 0x1DU);
  }
  return (crc ^ 0xDFU) == msg->data[7];
}

static void santa_fe_prepare_forward(CANPacket_t *stock) {
  if ((stock->bus != SANTA_FE_STOCK_BUS) || ((stock->addr != SANTA_FE_SCC12) && (stock->addr != SANTA_FE_FCA11))) {
    return;
  }
  if (stock->extended || stock->fd || (GET_LEN(stock) != 8U)) {
    santa_fe_yield_to_stock();
    return;
  }

  const uint32_t now = microsecond_timer_get();
  if (stock->addr == SANTA_FE_FCA11) {
    const uint8_t counter = (stock->data[4] >> 3U) & 0xFU;
    const bool integrity_ok = (counter < 15U) && santa_fe_fca_crc_valid(stock);
    const bool continuous = santa_fe_scc.have_fca && integrity_ok &&
                            (counter == ((santa_fe_scc.fca_counter + 1U) % 15U)) &&
                            safety_get_ts_elapsed(now, santa_fe_scc.last_fca) <= SANTA_FE_SOURCE_MAX_AGE_US;
    santa_fe_scc.have_fca = integrity_ok;
    santa_fe_scc.last_fca = now;
    santa_fe_scc.fca_counter = counter;
    // FCA warning, prefill, braking, stop or failure all keep the stock path.
    santa_fe_scc.fca_clear = continuous && (stock->data[0] & 0x1FU) == 0U && stock->data[1] == 0U &&
                             (stock->data[2] & 0x30U) == 0U && (stock->data[3] & 0x80U) == 0U &&
                             (stock->data[4] & 7U) == 0U;
    if (!santa_fe_scc.fca_clear) {
      santa_fe_yield_to_stock();
    }
    return;
  }
  const bool pending = santa_fe_scc.pending;
  santa_fe_scc.pending = false;
  const uint8_t counter = stock->data[7] & 0xFU;
  const bool integrity_ok = (counter < 15U) && (hyundai_compute_checksum(stock) == hyundai_get_checksum(stock));
  const bool continuous = santa_fe_scc.have_scc && integrity_ok &&
                          (counter == ((santa_fe_scc.scc_counter + 1U) % 15U)) &&
                          safety_get_ts_elapsed(now, santa_fe_scc.last_scc) <= SANTA_FE_SOURCE_MAX_AGE_US;
  santa_fe_scc.have_scc = integrity_ok;
  santa_fe_scc.last_scc = now;
  santa_fe_scc.scc_counter = counter;
  // Leave stock stop requests, override/ACC failure, warnings, prefill and
  // all OEM emergency fields byte-for-byte intact. A new SET/RES is required
  // after an emergency or continuity loss before admitting host requests.
  const bool stock_clear = ((stock->data[0] & 0x3FU) == 0U) && ((stock->data[1] & 0x98U) == 0U) &&
                           (stock->data[2] == 0U) && ((stock->data[4] & 0x18U) == 0U) && ((stock->data[6] & 0xCCU) == 0U);
  // A previous cycle's FCA11 cannot clear SCC12 for acceleration replacement.
  const bool fca_for_this_cycle = (santa_fe_scc.fca_counter == counter) &&
                                 safety_get_ts_elapsed(now, santa_fe_scc.last_fca) <= SANTA_FE_FCA_PAIR_MAX_AGE_US;
  if (!continuous || !stock_clear || !fca_for_this_cycle || !santa_fe_source_healthy(now)) {
    santa_fe_yield_to_stock();
    return;
  }
  if (!get_longitudinal_allowed() || brake_pressed || brake_pressed_prev) {
    santa_fe_scc.replaced = false;
    return;
  }
  // The radar can take a cycle to enter active SCC after SET/RES.
  if ((stock->data[1] & 0x60U) != 0x20U) {
    if (santa_fe_scc.replaced || ((stock->data[1] & 0x60U) != 0U)) {
      santa_fe_yield_to_stock();
    }
    return;
  }
  if (!aol_rx_healthy() || (pending && (safety_get_ts_elapsed(now, santa_fe_scc.staged) > SANTA_FE_REQUEST_MAX_AGE_US))) {
    santa_fe_yield_to_stock();
    return;
  }
  if (!pending) {
    // After SET/RES, wait in stock mode for the very first host proposal.
    // Losing a proposal after taking over returns authority to stock.
    if (santa_fe_scc.replaced) {
      santa_fe_yield_to_stock();
    }
    return;
  }

  // Recheck control limits at the point of forwarding, then copy only the
  // accelerations and allow an added stop request. Keep the OEM rolling counter.
  const CANPacket_t *request = &santa_fe_scc.request;
  if (hyundai_tx_hook(request)) {
    stock->data[1] |= request->data[1] & 0x80U;
    stock->data[3] = request->data[3];
    stock->data[4] = (stock->data[4] & 0x18U) | (request->data[4] & 0xE7U);
    stock->data[5] = request->data[5];
    stock->data[7] = counter | ((uint8_t)hyundai_compute_checksum(stock) << 4U);
    santa_fe_scc.replaced = true;
  } else {
    santa_fe_yield_to_stock();
  }
}

static safety_config hyundai_santa_fe_bus2_init(uint16_t param) {
  (void)memset(&santa_fe_scc, 0, sizeof(santa_fe_scc));
  if (param != 12U) { // exact CAMERA_SCC | LONG, classic CAN
    return (safety_config){.disable_forwarding = true};
  }
  static const CanMsg tx[] = {
    HYUNDAI_COMMON_TX_MSGS(2, false)
    {SANTA_FE_SCC12, SANTA_FE_CAR_BUS, 8, .check_relay = true, .disable_static_blocking = true},
  };
  safety_config config = hyundai_init(param);
  SET_TX_MSGS(tx, config);
  return config;
}

const safety_hooks hyundai_santa_fe_bus2_hooks = {
  .init = hyundai_santa_fe_bus2_init,
  .rx = hyundai_rx_hook,
  .tx = santa_fe_tx_hook,
  .stage_tx = santa_fe_stage_request,
  .prepare_fwd = santa_fe_prepare_forward,
  .get_counter = hyundai_get_counter,
  .get_checksum = hyundai_get_checksum,
  .compute_checksum = hyundai_compute_checksum,
};
