// Self-balancing robot — Phase 0 firmware
//
// POWER PROTOCOL (see CLAUDE.md): motors are DISARMED at boot — PWM zero,
// TB6612 in standby. No motion is possible until an explicit 'a' (arm) or
// 'm' (motor test) serial command. 'x' is the emergency stop from any state.
//
// Serial console 115200 baud. Lines starting with "# " are human-readable;
// bare CSV lines are telemetry (see TELEM_HEADER).

#include <Wire.h>
#include <EEPROM.h>
#include "MPU6050.h"
#include "kalman.h"
#include "MotorDriver.h"
#include "PIDController.h"

#define ACC_LSB_PER_G 8192.0f
// Datasheet value for ±500 dps is 65.5, but this chip (likely a clone)
// underreads by a constant 1.335x — measured in trial_014 by integrating
// the gyro across hand tilts and comparing with the accel angle change
// between stationary holds (ratio 1.335, consistent 26°-58° both ways).
#define GYRO_LSB_PER_DPS 49.06f

// Control timing
#define LOOP_US 5000UL          // 200 Hz control loop
#define TELEM_DECIM 4           // telemetry every 4th cycle (50 Hz)

// Safety
#define TILT_CUTOFF_DEG 40.0f   // kill motors beyond this tilt
#define ARM_TOL_DEG 5.0f        // must be this close to upright to arm
#define ARM_HOLD_MS 2000UL      // ...continuously for this long
#define MOTOR_TEST_TIMEOUT_MS 2000UL  // test PWM auto-zeroes without fresh cmd

#define TRIM_CAP_CYCLES 1000    // 'T' trim capture: 1000 cycles = 5 s

// gyro_rate = pitch rate (X gyro, the control axis); gyro_y/z for diagnostics
#define TELEM_HEADER "time_ms,raw_angle,kalman_angle,gyro_rate,p_term,i_term,d_term,motor_out,loop_dt_us,gyro_y,gyro_z"

enum State : uint8_t { DISARMED, ARMING, BALANCING, MOTOR_TEST };

MPU6050 mpu;
Kalman kalman;
MotorDriver motors;
PIDController pid(25.0f, 5.0f, 0.5f);

State state = DISARMED;
float angleTrim = 0.0f;         // setpoint offset: mechanical balance point
bool streaming = false;
bool mpuOk = false;             // IMU may be unpowered on USB-only power
uint32_t lastMpuRetryMs = 0;

uint32_t lastControlUs = 0;
uint32_t armHoldStartMs = 0;
bool armHolding = false;
uint32_t motorTestLastCmdMs = 0;
bool motorTestZeroed = true;
int testPwmL = 0, testPwmR = 0;

uint16_t trimCapRemaining = 0;
float trimCapSum = 0.0f;

uint8_t telemCount = 0;
char lineBuf[24];
uint8_t lineLen = 0;

// ---------------------------------------------------------------- EEPROM --
struct Settings {
  uint16_t magic;               // 0xB07A when valid
  float kp, ki, kd, trim;
  uint8_t dbLeft, dbRight;
};
#define SETTINGS_MAGIC 0xB07A

void saveSettings() {
  Settings s = { SETTINGS_MAGIC, pid.getKp(), pid.getKi(), pid.getKd(),
                 angleTrim, motors.deadbandLeft(), motors.deadbandRight() };
  EEPROM.put(0, s);
  Serial.println(F("# saved to EEPROM"));
}

void loadSettings() {
  Settings s;
  EEPROM.get(0, s);
  if (s.magic == SETTINGS_MAGIC) {
    pid.setTunings(s.kp, s.ki, s.kd);
    angleTrim = s.trim;
    motors.setDeadband(s.dbLeft, s.dbRight);
    Serial.println(F("# EEPROM settings loaded"));
  } else {
    Serial.println(F("# no EEPROM settings, using defaults"));
  }
}

// ---------------------------------------------------------------- helpers --
void printGains() {
  Serial.print(F("# Kp=")); Serial.print(pid.getKp(), 3);
  Serial.print(F(" Ki=")); Serial.print(pid.getKi(), 3);
  Serial.print(F(" Kd=")); Serial.print(pid.getKd(), 3);
  Serial.print(F(" trim=")); Serial.print(angleTrim, 2);
  Serial.print(F(" db=")); Serial.print(motors.deadbandLeft());
  Serial.print(F("/")); Serial.println(motors.deadbandRight());
}

void printStatus() {
  Serial.print(F("# state="));
  switch (state) {
    case DISARMED:   Serial.print(F("DISARMED")); break;
    case ARMING:     Serial.print(F("ARMING")); break;
    case BALANCING:  Serial.print(F("BALANCING")); break;
    case MOTOR_TEST: Serial.print(F("MOTOR_TEST")); break;
  }
  Serial.print(F(" mpu=")); Serial.print(mpuOk ? F("OK") : F("OFFLINE"));
  Serial.print(F(" stream=")); Serial.print(streaming ? 1 : 0);
  Serial.print(F(" t_ms=")); Serial.println(millis());
  printGains();
}

void printHelp() {
  Serial.println(F("# ?  status | h help"));
  Serial.println(F("# a  arm (hold upright 2s) | x  E-STOP -> disarm (immediate)"));
  Serial.println(F("# p/i/d <v>  set gain | z  reset integral"));
  Serial.println(F("# t <deg>  set angle trim | T  capture trim (avg 5s, hold at balance)"));
  Serial.println(F("# b <l> [r]  motor deadband PWM"));
  Serial.println(F("# s  toggle telemetry | c  gyro recal (still, disarmed)"));
  Serial.println(F("# m  motor test mode; then l <pwm>, r <pwm> (-255..255), x exits"));
  Serial.println(F("# W  save settings to EEPROM"));
}

void disarm(const __FlashStringHelper* why) {
  motors.disable();
  pid.reset();
  testPwmL = testPwmR = 0;
  state = DISARMED;
  Serial.print(F("# DISARMED: "));
  Serial.println(why);
}

// ------------------------------------------------------------------ serial --
void parseLine(char* line) {
  // strip leading spaces
  while (*line == ' ') line++;
  if (*line == '\0') return;
  char cmd = *line;
  char* arg = line + 1;
  while (*arg == ' ') arg++;

  switch (cmd) {
    case '?': printStatus(); break;
    case 'h': printHelp(); break;
    case 'a':
      if (!mpuOk) {
        Serial.println(F("# arm refused: MPU6050 offline (battery off?)"));
      } else if (state == DISARMED) {
        state = ARMING;
        armHolding = false;
        Serial.println(F("# ARMING: hold upright ~2 s"));
      } else {
        Serial.println(F("# arm refused: not disarmed"));
      }
      break;
    case 'p': pid.setTunings(atof(arg), pid.getKi(), pid.getKd()); printGains(); break;
    case 'i': pid.setTunings(pid.getKp(), atof(arg), pid.getKd()); printGains(); break;
    case 'd': pid.setTunings(pid.getKp(), pid.getKi(), atof(arg)); printGains(); break;
    case 't':
      if (*arg) { angleTrim = atof(arg); printGains(); }
      else Serial.println(F("# t needs a value (or use T to capture)"));
      break;
    case 'T':
      trimCapRemaining = TRIM_CAP_CYCLES;
      trimCapSum = 0.0f;
      Serial.println(F("# capturing trim: hold robot at balance point 5 s"));
      break;
    case 'z': pid.reset(); Serial.println(F("# integral reset")); break;
    case 's':
      streaming = !streaming;
      if (streaming) Serial.println(F(TELEM_HEADER));
      else Serial.println(F("# stream off"));
      break;
    case 'b': {
      int l = atoi(arg);
      char* sp = strchr(arg, ' ');
      int r = sp ? atoi(sp + 1) : l;
      motors.setDeadband((uint8_t)constrain(l, 0, 200), (uint8_t)constrain(r, 0, 200));
      printGains();
      break;
    }
    case 'c':
      if (!mpuOk) {
        Serial.println(F("# gyro cal refused: MPU6050 offline"));
      } else if (state == BALANCING || state == MOTOR_TEST) {
        Serial.println(F("# gyro cal refused: disarm first"));
      } else {
        Serial.println(F("# gyro cal: keep robot STILL..."));
        if (mpu.calibrateGyro(500))
          Serial.println(F("# gyro cal done"));
        else
          Serial.println(F("# gyro cal REJECTED (movement) — old bias kept"));
      }
      break;
    case 'm':
      if (state == DISARMED) {
        state = MOTOR_TEST;
        testPwmL = testPwmR = 0;
        motorTestZeroed = true;
        motors.enable();
        Serial.println(F("# MOTOR TEST: wheels OFF the ground, battery on."));
        Serial.println(F("# l <pwm> / r <pwm> (-255..255); auto-zero after 2 s idle; x exits"));
      } else {
        Serial.println(F("# motor test refused: not disarmed"));
      }
      break;
    case 'l':
    case 'r':
      if (state == MOTOR_TEST) {
        int v = constrain(atoi(arg), -255, 255);
        if (cmd == 'l') testPwmL = v; else testPwmR = v;
        motorTestLastCmdMs = millis();
        motorTestZeroed = false;
        motors.driveRaw(testPwmL, testPwmR);
        Serial.print(F("# test pwm L=")); Serial.print(testPwmL);
        Serial.print(F(" R=")); Serial.println(testPwmR);
      } else {
        Serial.println(F("# l/r only in motor test mode ('m')"));
      }
      break;
    case 'W': saveSettings(); break;
    default:
      Serial.print(F("# unknown cmd: ")); Serial.println(cmd);
      break;
  }
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'x' || c == 'X') {          // e-stop acts immediately, mid-line
      lineLen = 0;
      disarm(F("emergency stop"));
      continue;
    }
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      lineLen = 0;
      parseLine(lineBuf);
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

void calibrateAndSeed() {
  Serial.println(F("# gyro cal: keep robot still..."));
  if (mpu.calibrateGyro(500))
    Serial.println(F("# gyro cal done ('c' to redo)"));
  else
    Serial.println(F("# gyro cal REJECTED (movement) — old bias kept, run 'c' when still"));
  int16_t ax, ay, az;
  mpu.readAccelerometer(ax, ay, az);
  kalman.setAngle(atan2f((float)ay / ACC_LSB_PER_G, (float)az / ACC_LSB_PER_G)
                  * 180.0f / PI);
}

// ------------------------------------------------------------------- setup --
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  motors.init();                 // driver in standby, PWM zero
  Serial.println(F("# self-balancing-robot fw2 — motors DISARMED at boot"));

  loadSettings();

  mpuOk = mpu.init();
  if (mpuOk) {
    calibrateAndSeed();
  } else {
    Serial.println(F("# WARNING: MPU6050 offline (unpowered? wiring?) — will retry"));
  }

  Serial.println(F("# 'h' for help, 'a' to arm, 'x' = E-STOP"));
  printGains();
  lastControlUs = micros();
}

// -------------------------------------------------------------------- loop --
void loop() {
  handleSerial();

  uint32_t nowUs = micros();
  uint32_t dtUs = nowUs - lastControlUs;
  if (dtUs < LOOP_US) return;
  lastControlUs = nowUs;
  float dt = dtUs * 1e-6f;

  float acc_angle = 0, angle = 0, gy_dps = 0, gx_dps = 0, gz_dps = 0;
  if (!mpuOk) {
    // IMU likely on the battery rail: keep the console alive, retry quietly.
    if (millis() - lastMpuRetryMs > 2000) {
      lastMpuRetryMs = millis();
      if (mpu.init()) {
        mpuOk = true;
        Serial.println(F("# MPU6050 online"));
        calibrateAndSeed();
      }
    }
  } else {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.readMotion(ax, ay, az, gx, gy, gz);

    float ay_g = (float)ay / ACC_LSB_PER_G;
    float az_g = (float)az / ACC_LSB_PER_G;
    gy_dps = (float)gy / GYRO_LSB_PER_DPS;
    gx_dps = (float)gx / GYRO_LSB_PER_DPS;
    gz_dps = (float)gz / GYRO_LSB_PER_DPS;

    acc_angle = atan2f(ay_g, az_g) * 180.0f / PI;
    // Pitch (fall) axis is the X gyro: IMU X runs along the wheel axle,
    // positive gx = angle increasing (verified empirically in trial_014).
    // The original code fed gy here, which is why it barely balanced.
    angle = kalman.getAngle(acc_angle, gx_dps, dt);
  }

  if (mpuOk && trimCapRemaining > 0) {
    trimCapSum += angle;
    if (--trimCapRemaining == 0) {
      angleTrim = trimCapSum / TRIM_CAP_CYCLES;
      Serial.print(F("# trim captured: "));
      Serial.println(angleTrim, 2);
    }
  }

  int motorOut = 0;

  switch (state) {
    case ARMING: {
      if (fabsf(angle - angleTrim) < ARM_TOL_DEG) {
        if (!armHolding) { armHolding = true; armHoldStartMs = millis(); }
        else if (millis() - armHoldStartMs >= ARM_HOLD_MS) {
          pid.reset();
          kalman.setAngle(acc_angle);
          motors.enable();
          state = BALANCING;
          Serial.println(F("# ARMED — balancing"));
        }
      } else {
        armHolding = false;
      }
      break;
    }
    case BALANCING: {
      if (fabsf(angle - angleTrim) > TILT_CUTOFF_DEG) {
        disarm(F("tilt cutoff"));
        break;
      }
      float out = pid.compute(angleTrim, angle, dt);
      motorOut = (int)constrain(out, -255, 255);
      motors.drive(motorOut, motorOut);
      break;
    }
    case MOTOR_TEST: {
      if (!motorTestZeroed &&
          millis() - motorTestLastCmdMs > MOTOR_TEST_TIMEOUT_MS) {
        testPwmL = testPwmR = 0;
        motors.stop();
        motorTestZeroed = true;
        Serial.println(F("# test pwm auto-zeroed (2 s idle)"));
      }
      break;
    }
    case DISARMED:
      break;
  }

  if (streaming && mpuOk && ++telemCount >= TELEM_DECIM) {
    telemCount = 0;
    Serial.print(millis());          Serial.print(',');
    Serial.print(acc_angle, 2);      Serial.print(',');
    Serial.print(angle, 2);          Serial.print(',');
    Serial.print(gx_dps, 2);         Serial.print(',');
    Serial.print(pid.pTerm(), 1);    Serial.print(',');
    Serial.print(pid.iTerm(), 1);    Serial.print(',');
    Serial.print(pid.dTerm(), 1);    Serial.print(',');
    Serial.print(motorOut);          Serial.print(',');
    Serial.print(dtUs);              Serial.print(',');
    Serial.print(gy_dps, 2);         Serial.print(',');
    Serial.println(gz_dps, 2);
  }
}
