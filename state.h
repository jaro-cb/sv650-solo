#pragma once

struct EcuSnapshot {
  uint16_t rpmRaw;
  uint8_t tpsRaw;
  uint8_t gearRaw;
  // uint8_t iatRaw;
  // uint8_t ectRaw;
  // uint8_t battRaw;
  bool valid;
};
