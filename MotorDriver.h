#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <stdint.h>

// TB6612FNG with a single direction pin per motor (the second IN pin is
// hardware-inverted, see datasheets/sn74lvc2g14.pdf). Driver starts in
// STANDBY (disabled) — no output is possible until enable() is called.
class MotorDriver{
    public:
        void init();               // pins configured, driver left DISABLED
        void enable();             // STBY high — allows motion
        void disable();            // STBY low — driver standby, wheels free
        bool isEnabled() const { return enabled; }

        // Control-effort drive, -255..255 per side, positive = robot
        // forward. Applies deadband compensation: |v|>0 maps onto
        // [deadband..255]. No-op when disabled.
        void drive(int left_speed, int right_speed);

        // Raw PWM drive for bench tests (deadband ramp, direction check):
        // no deadband mapping, only the direction-sign convention applied.
        void driveRaw(int left_pwm, int right_pwm);

        void stop();               // PWM to zero (driver stays enabled)

        void setDeadband(uint8_t left, uint8_t right);
        // Battery-sag compensation: drive() output (incl. deadband mapping)
        // is multiplied by this factor (V_ref / V_battery). driveRaw is
        // intentionally unscaled.
        void setVoltComp(float f) { volt_comp = f; }
        uint8_t deadbandLeft() const { return db_left; }
        uint8_t deadbandRight() const { return db_right; }

    private:
        void setMotor(int in1, int pwm_pin, int speed);
        int applyDeadband(int v, uint8_t db) const;
        bool enabled = false;
        uint8_t db_left = 0, db_right = 0;
        float volt_comp = 1.0f;
};

#endif
