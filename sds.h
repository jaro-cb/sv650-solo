#pragma once

#define KLINE_RX_GPIO   18
#define KLINE_TX_GPIO   19
#define KLINE_SLP_GPIO  21

#define KLINE_BAUD      10400
#define ECU_ADDR        0x12
#define TESTER_ADDR     0xF1

#define TX_BYTE_GAP_MS  10       // inter-byte pacing — do not remove
#define POST_PULSE_MS   1        // wake pulse -> UART handover — keep small
#define KEEPALIVE_MS    2000     // session drops after ~5 s of silence

#include "state.h"

bool sdsRead(EcuSnapshot* ecu);
bool sdsInit();
//bool decodeLid08(const uint8_t *d, uint8_t n);