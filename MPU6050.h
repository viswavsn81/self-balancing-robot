#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>

// Configured for +/-4g (8192 LSB/g) and +/-500 dps (65.5 LSB/dps).
// init() MUST write these ranges: the chip's power-on default is +/-2g and
// +/-250 dps, which silently halves every reading if left unconfigured.
class MPU6050 {
public:
  bool init();  // returns false if WHO_AM_I doesn't answer
  void readAccelerometer(int16_t& ax, int16_t& ay, int16_t& az);
  void readGyroscope(int16_t& gx, int16_t& gy, int16_t& gz);
  // Single 14-byte burst: accel, temp (discarded), gyro. Faster than two
  // separate reads; keeps the control loop tight.
  void readMotion(int16_t& ax, int16_t& ay, int16_t& az,
                  int16_t& gx, int16_t& gy, int16_t& gz);
  // Average `samples` gyro readings with the robot at rest; stores offsets
  // subtracted by readMotion/readGyroscope. Returns false and keeps the
  // previous offsets if movement is detected during sampling (the board
  // resets on every serial port open, so boot cal can run mid-handling).
  bool calibrateGyro(uint16_t samples);

private:
  const uint8_t MPU6050_ADDR = 0x68;
  int16_t gyroOffX = 0, gyroOffY = 0, gyroOffZ = 0;
  void writeRegister(uint8_t reg, uint8_t data);
  void readRegisters(uint8_t startReg, uint8_t* buffer, uint8_t length);
};

#endif
