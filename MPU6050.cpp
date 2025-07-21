#include "MPU6050.h"

void MPU6050::init() {
  Wire.begin();
  writeRegister(0x6B, 0x00); // Wake up MPU6050 (clear sleep bit)
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
  gx = (int16_t)(buffer[0] << 8 | buffer[1]);
  gy = (int16_t)(buffer[2] << 8 | buffer[3]);
  gz = (int16_t)(buffer[4] << 8 | buffer[5]);
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
