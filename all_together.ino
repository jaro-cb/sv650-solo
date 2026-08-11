#include <Arduino.h>


// sv650 side
#include "sds.h"
#include "solo.h"
#include "state.h"

Ecu ecu;
HardwareSerial KLine(2);
bool     g_connected = false;
uint32_t g_lastTx    = 0;

// ------------------------------------------------------------------
// Once calibrated, replace the above with these and fix the constants
// ------------------------------------------------------------------
uint16_t rpm()  { return ecu.rpmRaw;             /* adjust divisor */ }
uint8_t gear()  { return ecu.gearRaw; }
uint8_t  tps()  { return (uint16_t)ecu.tpsRaw * 100 / 255; }
int8_t   ect()  { return (int)ecu.ectRaw - 40;   /* adjust */ }
int8_t   iat()  { return (int)ecu.iatRaw - 40;   /* adjust */ }

// ------------------------------------------------------------------
void setup() {
  // Serial.begin(115200);
  // delay(1200);
  // Serial.println("\n\n######## SDS reader ########");

  pinMode(KLINE_SLP_GPIO, OUTPUT);
  digitalWrite(KLINE_SLP_GPIO, HIGH);
  delay(20);

  uint8_t tries = 0;
  while (!g_connected && tries++ < 5) {
    Serial.printf("init attempt %u\n", tries);
    g_connected = sdsInit();
    if (!g_connected) delay(500);
  }
  if (!g_connected) {
    Serial.println("init failed — check pacing and POST_PULSE_MS");
    return;
  }

  soloInit();
}

// ------------------------------------------------------------------
void loop() {
  if (!g_connected) { 
    delay(1000); 
    return; 
  }
  sdsRead();
  soloSend();

 
  //delay(1000);
}
