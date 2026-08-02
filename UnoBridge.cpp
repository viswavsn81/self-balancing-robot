#include "UnoBridge.h"
#include <SoftwareSerial.h>

static SoftwareSerial ss(BRIDGE_RX_PIN, BRIDGE_TX_PIN);

void UnoBridge::begin() {
  ss.begin(BRIDGE_BAUD);
}

void UnoBridge::queueByte(uint8_t b) {
  uint8_t next = (uint8_t)((txHead + 1) % sizeof(txBuf));
  if (next == txTail) return;      // full: drop (lossy by design)
  txBuf[txHead] = b;
  txHead = next;
}

void UnoBridge::sendLine(const char* prefix, const char* text) {
  uint8_t cs = 0;
  queueByte('$');
  for (const char* p = prefix; *p; p++) { cs ^= (uint8_t)*p; queueByte((uint8_t)*p); }
  for (const char* p = text; *p; p++)   { cs ^= (uint8_t)*p; queueByte((uint8_t)*p); }
  queueByte('*');
  const char* hex = "0123456789ABCDEF";
  queueByte((uint8_t)hex[cs >> 4]);
  queueByte((uint8_t)hex[cs & 0x0F]);
  queueByte('\n');
}

bool UnoBridge::poll(char* cmd, bool* estop) {
  *estop = false;

  // Drain a few TX bytes (each blocks ~260 us at 38400)
  for (uint8_t i = 0; i < BRIDGE_TX_DRAIN_PER_POLL && txTail != txHead; i++) {
    ss.write(txBuf[txTail]);
    txTail = (uint8_t)((txTail + 1) % sizeof(txBuf));
  }

  // Ingest RX
  while (ss.available() > 0) {
    char c = (char)ss.read();
    if (c == 'x' || c == 'X') {        // immediate e-stop, frame or not
      *estop = true;
      inFrame = false;
      rxLen = 0;
      continue;
    }
    if (c == '$') { inFrame = true; rxLen = 0; continue; }
    if (!inFrame) continue;
    if (c == '\n' || c == '\r') {
      // expect payload*HH in rxBuf[0..rxLen)
      inFrame = false;
      if (rxLen < 3 || rxBuf[rxLen - 3] != '*') { rxLen = 0; continue; }
      uint8_t want = 0;
      for (uint8_t i = 0; i < 2; i++) {
        char h = rxBuf[rxLen - 2 + i];
        uint8_t v = (h >= '0' && h <= '9') ? h - '0'
                  : (h >= 'A' && h <= 'F') ? h - 'A' + 10
                  : (h >= 'a' && h <= 'f') ? h - 'a' + 10 : 0xFF;
        if (v == 0xFF) { want = 0xFF; break; }
        want = (uint8_t)((want << 4) | v);
      }
      uint8_t got = 0;
      for (uint8_t i = 0; i < rxLen - 3; i++) got ^= (uint8_t)rxBuf[i];
      if (want == got) {
        uint8_t n = (uint8_t)(rxLen - 3);
        memcpy(cmd, rxBuf, n);
        cmd[n] = '\0';
        rxLen = 0;
        return true;
      }
      rxLen = 0;
      continue;
    }
    if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
    else { inFrame = false; rxLen = 0; }   // overlong: discard
  }
  return false;
}
