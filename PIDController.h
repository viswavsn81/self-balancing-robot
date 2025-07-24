#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    float compute(float setpoint, float measured_value, float dt);

    void setTunings(float kp, float ki, float kd);
    void reset();

private:
    float Kp, Ki, Kd;
    float prev_error;
    float integral;
};

#endif
