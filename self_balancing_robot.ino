#include <Wire.h>
#include "MPU6050.h"
#include "kalman.h"
#include "MotorDriver.h"
#include "PIDController.h"

#define ACC_LSB_PER_G 8192.0
#define GYRO_LSB_PER_DPS 65.5

MPU6050 mpu;
Kalman kalman;
MotorDriver motors;

// PID variables
double Setpoint = 0;     // Desired angle (upright)
double Input = 0;        // Current angle from Kalman
double Output = 0;       // PID output to motors

double Kp = 25;
double Ki = 5;
double Kd = 0.5;


unsigned long prevTime = 0;

PIDController myPID(Kp, Ki, Kd);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.init();
  motors.init();

  
  myPID.setTunings(Kp, Ki, Kd);
  


  prevTime = millis();
  Setpoint = 0; // upright
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.readAccelerometer(ax, ay, az);
  mpu.readGyroscope(gx, gy, gz);

  float ay_g = (float)ay / ACC_LSB_PER_G;
  float az_g = (float)az / ACC_LSB_PER_G;
  float gy_dps = (float)gy / GYRO_LSB_PER_DPS;

  float acc_angle = atan2f(ay_g, az_g) * 180.0f / PI;

  unsigned long now = millis();
  float dt = (now - prevTime) * 0.001f;
  prevTime = now;

  float kalman_angle = kalman.getAngle(acc_angle, gy_dps, dt);
  Input = kalman_angle;

  Output = myPID.compute(Setpoint, Input, dt);

  int drive = (int)constrain(Output, -255, 255);
  motors.drive(drive, drive);

  Serial.print(kalman_angle, 2);
  Serial.print(',');
  Serial.println(drive);
}
