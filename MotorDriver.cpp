#include "MotorDriver.h"
#include <Arduino.h>

// Define motor driver pins
#define AIN1 7
#define PWMA 5
#define BIN1 8
#define PWMB 6
#define STBY 3

void MotorDriver::init(){
    pinMode(AIN1, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
}

void MotorDriver::setMotor(int in1, int pwm_pin, int speed){
    
    if(speed > 0){
        //Clockwise
        digitalWrite(in1, HIGH);
         // Set the PWM pin for speed control
        analogWrite(pwm_pin, speed);
    
    }
    else{
        //CCW
        digitalWrite(in1,LOW);
         // Set the PWM pin for speed control
        analogWrite(pwm_pin, -speed);
    }

    // Set the direction pins
    
    
   
    
    // Ensure standby pin is set to active
    digitalWrite(STBY, HIGH);

}

void MotorDriver::drive(int left_speed, int right_speed){
    // Set left motor
    setMotor(AIN1, PWMA, -left_speed);

    //Set right motor
    setMotor(BIN1, PWMB, -right_speed);
    /*
    if (left_speed > 0) {
        
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
    */
}

void MotorDriver::stop() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW); 
}