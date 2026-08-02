#include "MotorDriver.h"
#include <Arduino.h>

// Define motor driver pins
#define AIN1 7
#define PWMA 5
#define BIN1 8
#define PWMB 6
#define STBY 3

// Motors are mounted mirrored: positive command = robot forward requires
// negating both. If the Phase 1 direction test shows wheels driving the
// wrong way, flip signs HERE and nowhere else.
#define MOTOR_SIGN_LEFT  (-1)
#define MOTOR_SIGN_RIGHT (-1)

void MotorDriver::init(){
    pinMode(AIN1, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);
    // Boot state per POWER PROTOCOL: PWM zero, driver in standby.
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(AIN1, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(STBY, LOW);
    enabled = false;
}

void MotorDriver::enable(){
    stop();
    digitalWrite(STBY, HIGH);
    enabled = true;
}

void MotorDriver::disable(){
    stop();
    digitalWrite(STBY, LOW);
    enabled = false;
}

void MotorDriver::setMotor(int in1, int pwm_pin, int speed){
    if(speed > 0){
        digitalWrite(in1, HIGH);
        analogWrite(pwm_pin, speed > 255 ? 255 : speed);
    }
    else{
        digitalWrite(in1, LOW);
        analogWrite(pwm_pin, -speed > 255 ? 255 : -speed);
    }
}

int MotorDriver::applyDeadband(int v, uint8_t db) const {
    if (v == 0 || db == 0) return v;
    long mapped = (long)db + ((long)abs(v) * (255 - db)) / 255;
    return v > 0 ? (int)mapped : -(int)mapped;
}

void MotorDriver::drive(int left_speed, int right_speed){
    if (!enabled) return;
    int l = (int)(applyDeadband(left_speed, db_left) * volt_comp);
    int r = (int)(applyDeadband(right_speed, db_right) * volt_comp);
    setMotor(AIN1, PWMA, MOTOR_SIGN_LEFT  * l);
    setMotor(BIN1, PWMB, MOTOR_SIGN_RIGHT * r);
}

void MotorDriver::driveRaw(int left_pwm, int right_pwm){
    if (!enabled) return;
    setMotor(AIN1, PWMA, MOTOR_SIGN_LEFT  * left_pwm);
    setMotor(BIN1, PWMB, MOTOR_SIGN_RIGHT * right_pwm);
}

void MotorDriver::stop() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
}

void MotorDriver::setDeadband(uint8_t left, uint8_t right){
    db_left = left;
    db_right = right;
}
