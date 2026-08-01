#include "MPU6050.h"
#include <Arduino.h>

bool MPU6050::init() {
  Wire.begin();
  // Without this, a missing/unpowered MPU hangs the AVR Wire library
  // forever inside the first transaction (observed on USB-only power).
  Wire.setWireTimeout(3000, true);
  writeRegister(0x6B, 0x01); // Wake up, clock source = PLL with X gyro ref
  writeRegister(0x1A, 0x03); // DLPF ~44 Hz accel / 42 Hz gyro
  writeRegister(0x1B, 0x08); // Gyro full scale +/-500 dps (65.5 LSB/dps)
  writeRegister(0x1C, 0x08); // Accel full scale +/-4 g (8192 LSB/g)

  uint8_t whoami = 0;
  readRegisters(0x75, &whoami, 1);
  return whoami == 0x68;
}

void MPU6050::readAccelerometer(int16_t& ax, int16_t& ay, int16_t& az) {
  uint8_t buffer[6];
  readRegisters(0x3B, buffer, 6);
  ax = (int16_t)(buffer[0] << 8 | buffer[1]);
  ay = (int16_t)(buffer[2] << 8 | buffer[3]);
  az = (int16_t)(buffer[4] << 8 | buffer[5]);
}

void MPU6050::readGyroscope(int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t buffer[6];
  readRegisters(0x43, buffer, 6);
  gx = (int16_t)(buffer[0] << 8 | buffer[1]) - gyroOffX;
  gy = (int16_t)(buffer[2] << 8 | buffer[3]) - gyroOffY;
  gz = (int16_t)(buffer[4] << 8 | buffer[5]) - gyroOffZ;
}

void MPU6050::readMotion(int16_t& ax, int16_t& ay, int16_t& az,
                         int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t b[14];
  readRegisters(0x3B, b, 14); // accel[6], temp[2], gyro[6]
  ax = (int16_t)(b[0] << 8 | b[1]);
  ay = (int16_t)(b[2] << 8 | b[3]);
  az = (int16_t)(b[4] << 8 | b[5]);
  gx = (int16_t)(b[8] << 8 | b[9])  - gyroOffX;
  gy = (int16_t)(b[10] << 8 | b[11]) - gyroOffY;
  gz = (int16_t)(b[12] << 8 | b[13]) - gyroOffZ;
}

void MPU6050::calibrateGyro(uint16_t samples) {
  long sx = 0, sy = 0, sz = 0;
  gyroOffX = gyroOffY = gyroOffZ = 0;
  for (uint16_t i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    readGyroscope(gx, gy, gz);
    sx += gx; sy += gy; sz += gz;
    delay(2);
  }
  gyroOffX = (int16_t)(sx / (long)samples);
  gyroOffY = (int16_t)(sy / (long)samples);
  gyroOffZ = (int16_t)(sz / (long)samples);
}

void MPU6050::writeRegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void MPU6050::readRegisters(uint8_t startReg, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, length);
  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }
}
