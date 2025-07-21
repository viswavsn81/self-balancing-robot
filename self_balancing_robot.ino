#include <Wire.h>
#include "MPU6050.h"
#include "kalman.h"
#include "MotorDriver.h"
#include <PID_v1.h>

#define ACC_LSB_PER_G 8192.0
#define GYRO_LSB_PER_DPS 65.5

MPU6050 mpu;
Kalman kalman;
MotorDriver motors;

// PID variables
double Setpoint = 0;     // Desired angle (upright)
double Input = 0;        // Current angle from Kalman
double Output = 0;       // PID output to motors

double Kp = 8.0;
double Ki = 0.5;
double Kd = 1.0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.init();
  motors.init();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);

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

  myPID.Compute();
  int drive = (int)constrain(Output, -255, 255);
  motors.drive(drive, drive);

  Serial.print(kalman_angle, 2);
  Serial.print(',');
  Serial.println(drive);
}
