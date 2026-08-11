#include <Arduino.h>
#include "driver/twai.h"
#include "state.h"

#define CAN_TX_GPIO   GPIO_NUM_5
#define CAN_RX_GPIO   GPIO_NUM_4

#define BROADCAST_ID  0x200
#define BROADCAST_MS  10

const uint16_t RPM_STEP = 250;
const uint16_t RPM_MAX  = 12000;
const uint16_t RPM_MIN  = 500;
const uint32_t STEP_MS  = 1000;

uint32_t lastStep = 0;
uint32_t lastTx   = 0;

// ---------------- setup ----------------
void soloInit() {
  // use TWAI_MODE_NO_ACK for fire-and-forget
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  g.tx_queue_len = 16;
  g.rx_queue_len = 16;

  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    Serial.println("TWAI install failed");
    while (1) delay(1000);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("TWAI start failed");
    while (1) delay(1000);
  }
  Serial.println("CAN up @ 500 kbit/s");
}

void broadcast(EcuSnapshot* ecu) {
  uint32_t now = millis();
  if (now - lastTx < BROADCAST_MS) return;
  lastTx = now;

  Serial.printf("sending over CAN: gear=%d, tps=%d", ecu->gearRaw, ecu->tpsRaw);
  Serial.println();
  twai_message_t m = {};
  m.identifier       = BROADCAST_ID;
  m.flags            = TWAI_MSG_FLAG_NONE;
  m.data_length_code = 8;
  uint16_t rpm = ecu->rpmRaw; // consider scaling
  m.data[0] = rpm >> 8;        // rpm, big endian, 1 rpm/bit
  m.data[1] = rpm & 0xFF;
  m.data[2] = ecu->gearRaw; // this is good
  m.data[3] = ecu->tpsRaw; // consider scaling
  m.data[4] = 0; 
  m.data[5] = 0; 
  m.data[6] = 0;
  m.data[7] = 0;

  twai_transmit(&m, pdMS_TO_TICKS(10));
}

// ---------------- loop ----------------
void soloSend(EcuSnapshot* ecu) {
  broadcast(ecu);

  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_BUS_OFF) {
    Serial.println("bus-off, recovering");
    twai_initiate_recovery();
    delay(100);
    twai_start();
  }
}
