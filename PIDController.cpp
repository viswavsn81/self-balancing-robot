#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd)
    : Kp(kp), Ki(ki), Kd(kd), out_min(-255), out_max(255),
      prev_measured(0), integral(0),
      p_term(0), i_term(0), d_term(0), first_compute(true) {}

float PIDController::compute(float setpoint, float measured_value,
                             float measured_rate, float dt) {
    float error = setpoint - measured_value;

    p_term = Kp * error;

    // Integral accumulates pre-multiplied by Ki so live Ki changes don't
    // kick the output; clamped to the output range (anti-windup).
    integral += Ki * error * dt;
    if (integral > out_max) integral = out_max;
    else if (integral < out_min) integral = out_min;
    i_term = integral;

    // Derivative on measurement, from the sensor rate: no setpoint kick,
    // no differentiation noise, no filter lag (kalman-angle differentiation
    // destabilized the ~10 Hz mode — trials 019/020).
    d_term = -Kd * measured_rate;
    prev_measured = measured_value;
    first_compute = false;

    float output = p_term + i_term + d_term;
    if (output > out_max) output = out_max;
    else if (output < out_min) output = out_min;
    return output;
}

void PIDController::setTunings(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void PIDController::setOutputLimits(float min, float max) {
    out_min = min;
    out_max = max;
    if (integral > out_max) integral = out_max;
    else if (integral < out_min) integral = out_min;
}

void PIDController::reset() {
    integral = 0;
    p_term = i_term = d_term = 0;
    first_compute = true;
}
