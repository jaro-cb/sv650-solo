#pragma once

struct Ecu {
  uint16_t rpmRaw;
  uint8_t  tpsRaw, gearRaw, iatRaw, ectRaw, battRaw;
  bool     valid;
};

extern Ecu ecu;
