#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>

class MPU6050 {
public:
  void init();
  void readAccelerometer(int16_t& ax, int16_t& ay, int16_t& az);
  void readGyroscope(int16_t& gx, int16_t& gy, int16_t& gz);

private:
  const uint8_t MPU6050_ADDR = 0x68;
  void writeRegister(uint8_t reg, uint8_t data);
  void readRegisters(uint8_t startReg, uint8_t* buffer, uint8_t length);
};

#endif
