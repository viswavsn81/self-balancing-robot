#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd)
    : Kp(kp), Ki(ki), Kd(kd), prev_error(0), integral(0) {}

float PIDController::compute(float setpoint, float measured_value, float dt) {
    float error = setpoint - measured_value;
    integral += error * dt;
    float derivative = (error - prev_error) / dt;

    float output = Kp * error + Ki * integral + Kd * derivative;

    prev_error = error;
    return output;
}

void PIDController::setTunings(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void PIDController::reset() {
    prev_error = 0;
    integral = 0;
}
