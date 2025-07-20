#include "MotorDriver.h"
#include <Arduino.h>

// Define motor driver pins
#define AIN1 7
#define AIN2 8
#define PWMA 6
#define BIN1 4
#define BIN2 9
#define PWMB 5
#define STBY 3

void MotorDriver::init(){

}

void MotorDriver::setMotor(int in1,int in2, int pwm_pin, int speed){
    // Set the direction pins
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    
    // Set the PWM pin for speed control
    analogWrite(pwm_pin, speed);
    
    // Ensure standby pin is set to active
    digitalWrite(STBY, HIGH);

}

void MotorDriver::drive(int left_speed, int right_speed){
    // Set left motor
    if (left_speed > 0) {
        setMotor(AIN1, AIN2, PWMA, left_speed);
    } else if (left_speed < 0) {
        setMotor(AIN2, AIN1, PWMA, -left_speed);
    } else {
        setMotor(AIN1, AIN2, PWMA, 0);
    }

    // Set right motor
    if (right_speed > 0) {
        setMotor(BIN1, BIN2, PWMB, right_speed);
    } else if (right_speed < 0) {
        setMotor(BIN2, BIN1, PWMB, -right_speed);
    } else {
        setMotor(BIN1, BIN2, PWMB, 0);
    }
}