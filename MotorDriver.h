#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

class MotorDriver{
    public:
        void init();
        void drive(int left_speed, int right_speed);
        void stop();

    private:
        void setMotor(int in1, int in2, int pwm_pin, int speed);
};

#endif