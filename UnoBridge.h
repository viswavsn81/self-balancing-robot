#ifndef UNOBRIDGE_H
#define UNOBRIDGE_H

#include <Arduino.h>

// Wireless link to the ESP32 camera/bridge board over SoftwareSerial.
//
// Pins (chosen to avoid motor driver 3,5,6,7,8 / I2C A4,A5 / vbat A0 /
// USB 0,1): RX = D2 (from ESP32 TX, 3.3 V — fine for AVR VIH),
//           TX = D4 (to ESP32 RX **through a 1k/2k divider** — 5 V would
//           damage the ESP32).
//
// Frame format, both directions, ASCII (NMEA-style):
//   $<payload>*HH\n     HH = uppercase hex XOR of payload bytes
// Downlink payloads are exactly the USB console commands ("a", "x",
// "p 15", "g 80", ...). A bare 'x'/'X' byte OUTSIDE a frame is also
// honored (immediate e-stop, same as the USB path).
// Uplink payloads: "#<console message>" and "T,<compact telemetry>".
//
// SoftwareSerial TX blocks ~260 us/byte at 38400, so all output goes
// through a ring buffer drained a few bytes per control cycle — never
// print synchronously from the control loop.

#define BRIDGE_RX_PIN 2
#define BRIDGE_TX_PIN 4
#define BRIDGE_BAUD 38400
#define BRIDGE_TX_DRAIN_PER_POLL 2   // bytes per control cycle (~0.5 ms)

class UnoBridge {
public:
  void begin();

  // Drain a little TX, ingest RX. Returns true when a complete frame with
  // a valid checksum is ready; payload is NUL-terminated in `cmd` (which
  // must hold >= 32 bytes). Returns at most one command per call.
  // If a bare 'x' arrives (even mid-frame garbage), *estop is set.
  bool poll(char* cmd, bool* estop);

  // Queue a framed line for the ESP32 (drops if the buffer is full —
  // telemetry is lossy by design; never blocks).
  void sendLine(const char* prefix, const char* text);

private:
  void queueByte(uint8_t b);
  uint8_t txBuf[96];
  uint8_t txHead = 0, txTail = 0;
  char rxBuf[32];
  uint8_t rxLen = 0;
  bool inFrame = false;
};

#endif
