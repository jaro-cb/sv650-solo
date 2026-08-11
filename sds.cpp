#include <Arduino.h>
#include "sds.h"
#include "state.h"

static void sendShort(const uint8_t *d, uint8_t len);
static void dump(const char *tag, const uint8_t *b, uint8_t n);
static uint8_t checksum(const uint8_t *b, uint8_t n);

static HardwareSerial KLine(2);
static uint32_t g_lastTx    = 0;
static bool connected = false;

static uint8_t readFrame(uint8_t *out, uint8_t outMax, uint32_t timeoutMs = 400) {
  uint32_t t0 = millis();
  uint8_t buf[80], got = 0, len = 0, hdr = 0, total = 0;

  while (millis() - t0 < timeoutMs) {
    if (!KLine.available()) { delay(1); continue; }
    buf[got++] = KLine.read();

    if (got == 1) {
      if ((buf[0] & 0xC0) != 0x80) { got = 0; continue; }
      len = buf[0] & 0x3F;
      hdr = len ? 3 : 4;
      if (len) total = hdr + len + 1;
    }
    if (!len && got == 4) {
      len = buf[3];
      total = hdr + len + 1;
      if (!len || total > sizeof(buf)) { got = 0; len = 0; continue; }
    }
    if (total && got == total) {
      if (checksum(buf, total - 1) != buf[total - 1]) {
        Serial.print("  bad checksum: ");
        dump("raw", buf, total);
        return 0;
      }
      uint8_t n = min((int)len, (int)outMax);
      memcpy(out, &buf[hdr], n);
      return n;
    }
    if (got >= sizeof(buf)) return 0;
  }
  return 0;
}

static bool sdsHandshake() {
  KLine.end();
  pinMode(KLINE_TX_GPIO, OUTPUT);
  digitalWrite(KLINE_TX_GPIO, HIGH);
  delay(350);                                   // W5 bus idle

  digitalWrite(KLINE_TX_GPIO, LOW);   delay(25);
  digitalWrite(KLINE_TX_GPIO, HIGH);  delay(25);

  KLine.begin(KLINE_BAUD, SERIAL_8N1, KLINE_RX_GPIO, KLINE_TX_GPIO);
  delay(POST_PULSE_MS);                         // must stay small
  while (KLine.available()) KLine.read();

  uint8_t req[] = { 0x81 };                     // StartCommunication
  sendShort(req, 1);

  uint8_t resp[16];
  uint8_t n = readFrame(resp, sizeof(resp), 500);
  dump("init resp", resp, n);

  // expect C1 EA 8F
  if (n >= 3 && resp[0] == 0xC1) {
    Serial.printf("  connected — key bytes %02X %02X\n", resp[1], resp[2]);
    return true;
  }
  return false;
}

bool sdsInit() {
  pinMode(KLINE_SLP_GPIO, OUTPUT);
  digitalWrite(KLINE_SLP_GPIO, HIGH);
  delay(20);

  uint8_t tries = 0;
  while (!connected && tries++ < 20) {
    Serial.printf("init attempt %u\n", tries);
    connected = sdsHandshake();
    if (!connected) {
      delay(500);
    }
  }
  if (!connected) {
    Serial.println("init failed — check pacing and POST_PULSE_MS");
    return false;
  }
}

// ------------------------------------------------------------------
// low level
// ------------------------------------------------------------------
static uint8_t checksum(const uint8_t *b, uint8_t n) {
  uint16_t s = 0;
  for (uint8_t i = 0; i < n; i++) s += b[i];
  return s & 0xFF;
}

static void dump(const char *tag, const uint8_t *b, uint8_t n) {
  Serial.printf("%d  %s:", millis(), tag);
  for (uint8_t i = 0; i < n; i++) Serial.printf(" %02X", b[i]);
  Serial.println();
}

// Every byte we send comes straight back. Discard exactly our own count.
static void eatEcho(uint8_t n, uint32_t timeoutMs = 300) {
  uint32_t t0 = millis();
  while (n && millis() - t0 < timeoutMs) {
    if (KLine.available()) { KLine.read(); n--; }
  }
}

// Paced transmit. The gap between bytes is the whole point.
static void sendPaced(const uint8_t *f, uint8_t n) {
  while (KLine.available()) KLine.read();
  for (uint8_t i = 0; i < n; i++) {
    KLine.write(f[i]);
    KLine.flush();
    if (i < n - 1) delay(TX_BYTE_GAP_MS);
  }
  eatEcho(n);
  g_lastTx = millis();
}

// short format: [0x80|len][tgt][src][data...][cs]
static void sendShort(const uint8_t *d, uint8_t len) {
  uint8_t f[16];
  f[0] = 0x80 | len; f[1] = ECU_ADDR; f[2] = TESTER_ADDR;
  memcpy(&f[3], d, len);
  f[3 + len] = checksum(f, 3 + len);
  sendPaced(f, 4 + len);
}

// long format: [0x80][tgt][src][len][data...][cs]
static void sendLong(const uint8_t *d, uint8_t len) {
  uint8_t f[24];
  f[0] = 0x80; f[1] = ECU_ADDR; f[2] = TESTER_ADDR; f[3] = len;
  memcpy(&f[4], d, len);
  f[4 + len] = checksum(f, 4 + len);
  sendPaced(f, 5 + len);
}


void testerPresent() {
  uint8_t tp[] = { 0x3E };
  sendLong(tp, 1);
  uint8_t junk[16];
  readFrame(junk, sizeof(junk), 250);
}

// ------------------------------------------------------------------
// data requests
// ------------------------------------------------------------------
// ReadDataByLocalIdentifier. Returns payload length after the 61 <lid>.
uint8_t readLid(uint8_t lid, uint8_t *out, uint8_t outMax) {
  uint8_t req[] = { 0x21, lid };
  sendLong(req, 2);

  uint8_t resp[64];
  uint8_t n = readFrame(resp, sizeof(resp));
  if (n == 0) return 0;

  if (resp[0] == 0x7F) {                        // negative response
    Serial.printf("  LID %02X rejected, code %02X\n", lid, n >= 3 ? resp[2] : 0);
    return 0;
  }
  if (resp[0] != 0x61) {
    Serial.printf("  LID %02X unexpected service %02X\n", lid, resp[0]);
    return 0;
  }
  uint8_t payload = (n >= 2) ? n - 2 : 0;
  uint8_t c = min((int)payload, (int)outMax);
  memcpy(out, &resp[2], c);
  return c;
}


#define FRAME_OFFSET   6      // set to 0 if indices turn out payload-relative

#define OFF_TPS       13
#define OFF_GEAR      20

#define OFF_RPM_HI    (25 - FRAME_OFFSET)
#define OFF_RPM_LO    (26 - FRAME_OFFSET)
#define OFF_TPS2      (27 - FRAME_OFFSET)
#define OFF_ECT       (29 - FRAME_OFFSET)
#define OFF_IAT       (30 - FRAME_OFFSET)
#define OFF_BATT      (32 - FRAME_OFFSET)


bool decodeLid08(const uint8_t *d, uint8_t n, EcuSnapshot* ecu) {
  if (n <= OFF_GEAR) { 
    ecu->valid = false; 
    return false; 
  }

  ecu->rpmRaw  = ((uint16_t)d[OFF_RPM_HI] << 8) | d[OFF_RPM_LO];
  ecu->tpsRaw  = d[OFF_TPS];
  ecu->gearRaw = d[OFF_GEAR];
  // ecu.iatRaw  = d[OFF_IAT];
  // ecu.ectRaw  = d[OFF_ECT];
  // ecu.battRaw = d[OFF_BATT];
  ecu->valid   = true;
  return true;
}


bool sdsRead(EcuSnapshot* ecu) {
  uint8_t d[48];
  uint8_t n = readLid(0x08, d, sizeof(d));

  if (n)  {
    dump("08", d, n);
    decodeLid08(d, n, ecu);
    return true;
  }
  return false;
}