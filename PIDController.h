#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    float compute(float setpoint, float measured_value, float dt);

    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void reset();

    float getKp() const { return Kp; }
    float getKi() const { return Ki; }
    float getKd() const { return Kd; }
    // Last computed terms, for telemetry
    float pTerm() const { return p_term; }
    float iTerm() const { return i_term; }
    float dTerm() const { return d_term; }

private:
    float Kp, Ki, Kd;
    float out_min, out_max;
    float prev_measured;
    float integral;          // stores Ki*integral (the clamped I term)
    float p_term, i_term, d_term;
    bool first_compute;
};

#endif
