// Self-balancing robot — Phase 3 firmware (cascaded control, encoderless)
//
// POWER PROTOCOL (see CLAUDE.md): motors are DISARMED at boot — PWM zero,
// TB6612 in standby. No motion is possible until an explicit 'a' (arm) or
// 'm' (motor test) serial command. 'x' is the emergency stop from any state.
//
// Control structure (inside-out):
//   inner:  angle PID @ 200 Hz -> motor PWM        (Phase 2, tuned)
//   middle: velocity PI @ 20 Hz -> tilt setpoint   (encoderless: v estimated
//           from low-pass-filtered commanded PWM; ±3° tilt clamp)
//   outer:  distance dead-reckoning + trapezoid -> velocity setpoint ('g')
//   turn:   z-gyro yaw hold, differential PWM      ('t' relative turns)
//
// Serial console 38400 baud (camera-link level shifter limit). Lines starting with "# " are human-readable;
// bare CSV lines are telemetry (see TELEM_HEADER).

#include <Wire.h>
#include <EEPROM.h>
#include <avr/interrupt.h>
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
#define LOOP_US 5000UL          // 200 Hz inner loop
#define MID_DECIM 10            // middle/outer loops every 10th cycle (20 Hz)
#define TELEM_DECIM 8           // telemetry every 8th cycle (25 Hz; fits 38400)

// Safety
// Asymmetric cutoff (see-saw ruler bumper, measured 2026-08-02): the rear
// tip grounds at +43.8 abs (~+29 rel trim 15) — motors must cut BEFORE the
// robot beaches on it; the front tip grounds at -39.5 abs (~-55 rel), so
// the forward side keeps the original 40.
#define TILT_CUTOFF_FWD_DEG 40.0f   // angle BELOW trim by more than this
#define TILT_CUTOFF_BACK_DEG 26.0f  // angle ABOVE trim by more than this
#define CATCH_RATE_MAX_DPS 60.0f    // max |pitch rate| for a swing-up catch
#define CATCH_TIMEOUT_MS 10000UL

// Firmware-native swing-up ('S'): from a ruler rest, a settle pre-pulse,
// then bang-bang energy pumping under the 200 Hz loop — drive forward
// (torque reaction lifts, empirically from BOTH rests) while the body
// moves toward upright, coast while it moves away — with the native
// instant catch. Feedback beats the open-loop kick variance that killed
// the remote campaign (logs 063/064).
#define SWING_PUMP_PWM 85
#define SWING_SETTLE_PWM 30
#define SWING_TIMEOUT_MS 8000UL
#define SWING_TAPER_DEG 18.0f
#define ARM_TOL_DEG 5.0f        // must be this close to upright to arm
#define ARM_HOLD_MS 2000UL      // ...continuously for this long
#define MOTOR_TEST_TIMEOUT_MS 2000UL  // test PWM auto-zeroes without fresh cmd

#define TRIM_CAP_CYCLES 1000    // 'T' trim capture: 1000 cycles = 5 s

// Battery divider on A0: Vbat -> 10k -> A0 -> 4.7k -> GND, so
// V = ADC * 5/1023 * (14.7/4.7). Full 2S pack 8.4 V reads ~2.69 V (~550).
#define VBAT_PIN A0
#define VBAT_V_PER_COUNT (5.0f / 1023.0f * (14.7f / 4.7f))
#define VBAT_REF 8.2f           // voltage at which deadband/gains were tuned
#define VBAT_MIN_ARM 6.8f       // refuse to arm below this (pack nearly done)
#define VBAT_PRESENT 5.0f       // below this = running on USB, no comp

// Arming guide: sub-motion PWM hum while the angle is inside the arm gate
// (breakaway is ~20-24, so 15 makes sound but cannot move the wheels).
// The pin-13 LED is hidden under the board on this build.
#define ARM_HUM_PWM 15

// Motion profile (units: cm, cm/s, deg)
#define TILT_CMD_MAX 3.0f       // velocity loop authority over tilt setpoint
#define V_CRUISE 12.0f          // 'g' cruise speed
#define V_ACCEL 20.0f           // setpoint slew, cm/s per s
#define V_DECEL 15.0f           // braking decel used for trapezoid
#define V_SET_MAX 25.0f         // manual 'v' clamp
#define GO_DONE_CM 2.0f
#define TURN_OUT_MAX 45         // differential PWM clamp
#define TURN_DONE_DEG 3.0f
#define VEL_LPF_TAU 0.4f        // s, commanded-PWM -> velocity estimate filter
// Below this filtered-PWM magnitude the robot isn't really moving (stiction/
// hover jitter); count it as zero so dead reckoning doesn't accumulate
// phantom distance while station-keeping (trial 052: -75 fictitious cm).
#define VEL_EST_DEADZONE 8.0f
// Velocity loop fades in over this long after arming: the release transient
// otherwise reads as "speeding forward", and the loop's backward tilt demand
// feeds the non-minimum-phase spiral that toppled every arm in trial 058.
#define VEL_RAMP_MS 3000UL

#define TELEM_HEADER "time_ms,raw_angle,kalman_angle,gyro_rate,p_term,i_term,d_term,motor_out,loop_dt_us,gyro_y,gyro_z,tilt_cmd,v_est,v_set,dist_cm,yaw_deg,vbat"

enum State : uint8_t { DISARMED, ARMING, CATCH, SWINGUP, BALANCING, MOTOR_TEST };
enum Mode : uint8_t { M_HOLD, M_GO, M_TURN };  // sub-mode while BALANCING

MPU6050 mpu;
Kalman kalman;
MotorDriver motors;
PIDController pid(15.0f, 0.5f, 0.2f);

State state = DISARMED;
Mode mode = M_HOLD;
float angleTrim = 6.0f;         // setpoint offset: mechanical balance point
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

// Middle/outer loop state
float kvScale = 0.235f;         // cm/s per PWM count ('k', calibrate on floor)
float kvp = 0.04f;              // deg tilt per cm/s velocity error
float kvi = 0.02f;              // deg tilt per cm/s-s
float kyp = 2.0f;               // differential PWM per deg yaw error
float velLpf = 0.0f;            // filtered commanded PWM
float vEst = 0.0f;              // cm/s (estimated)
float vSet = 0.0f;              // slewed setpoint actually tracked
float vSetTarget = 0.0f;        // requested setpoint ('v' or profile)
float velI = 0.0f;              // velocity integral, in deg of tilt
float tiltCmd = 0.0f;           // deg; + = lean forward (angle setpoint down)
float distCm = 0.0f;            // dead-reckoned since arm
uint32_t armedAtMs = 0;         // for the velocity-loop soft-start ramp
float goTargetCm = 0.0f;
float yawDeg = 0.0f;            // integrated z-gyro since arm
float yawTarget = 0.0f;
uint8_t turnSettleCount = 0;
uint8_t midCount = 0;

uint8_t telemCount = 0;
char lineBuf[24];
uint8_t lineLen = 0;

float vbat = 0.0f;              // smoothed battery voltage from A0 divider
uint8_t vbatCount = 0;

// ---------------------------------------------------------------- EEPROM --
struct Settings {
  uint16_t magic;
  float kp, ki, kd, trim;
  uint8_t dbLeft, dbRight;
  int16_t gbx, gby, gbz;        // last good gyro bias (LSB)
  float kvp_, kvi_, kyp_, kvScale_;
};
#define SETTINGS_MAGIC 0xB07C   // bump when the struct layout changes

void saveSettings() {
  Settings s = { SETTINGS_MAGIC, pid.getKp(), pid.getKi(), pid.getKd(),
                 angleTrim, motors.deadbandLeft(), motors.deadbandRight(),
                 0, 0, 0, kvp, kvi, kyp, kvScale };
  mpu.getGyroOffsets(s.gbx, s.gby, s.gbz);
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
    mpu.setGyroOffsets(s.gbx, s.gby, s.gbz);
    kvp = s.kvp_; kvi = s.kvi_; kyp = s.kyp_; kvScale = s.kvScale_;
    Serial.println(F("# EEPROM settings loaded"));
  } else {
    Serial.println(F("# no/old EEPROM settings, using defaults"));
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
  Serial.print(F("# vp=")); Serial.print(kvp, 3);
  Serial.print(F(" vi=")); Serial.print(kvi, 3);
  Serial.print(F(" yp=")); Serial.print(kyp, 2);
  Serial.print(F(" k=")); Serial.println(kvScale, 4);
}

void printStatus() {
  Serial.print(F("# state="));
  switch (state) {
    case DISARMED:   Serial.print(F("DISARMED")); break;
    case ARMING:     Serial.print(F("ARMING")); break;
    case CATCH:      Serial.print(F("CATCH")); break;
    case SWINGUP:    Serial.print(F("SWINGUP")); break;
    case BALANCING:  Serial.print(F("BALANCING")); break;
    case MOTOR_TEST: Serial.print(F("MOTOR_TEST")); break;
  }
  Serial.print(F(" mode="));
  switch (mode) {
    case M_HOLD: Serial.print(F("HOLD")); break;
    case M_GO:   Serial.print(F("GO")); break;
    case M_TURN: Serial.print(F("TURN")); break;
  }
  Serial.print(F(" mpu=")); Serial.print(mpuOk ? F("OK") : F("OFFLINE"));
  Serial.print(F(" stream=")); Serial.print(streaming ? 1 : 0);
  Serial.print(F(" dist=")); Serial.print(distCm, 1);
  Serial.print(F(" yaw=")); Serial.print(yawDeg, 1);
  Serial.print(F(" vbat=")); Serial.print(vbat, 2);
  Serial.print(F(" t_ms=")); Serial.println(millis());
  printGains();
}

void printHelp() {
  Serial.println(F("# ?  status | h help | x  E-STOP (immediate)"));
  Serial.println(F("# a  arm (hold upright 2s; then station-keeps)"));
  Serial.println(F("# C  catch mode: arms INSTANTLY when swing crosses gate (10s window)"));
  Serial.println(F("# S  swing-up: self-erect from ruler rest (pump + catch, 8s cap)"));
  Serial.println(F("# R  reboot into bootloader (remote reflash)"));
  Serial.println(F("# g <cm>  go distance | t <deg>  turn (+ = left/CCW)"));
  Serial.println(F("# v <cm/s>  velocity setpoint (0 = station keep)"));
  Serial.println(F("# p/i/d <v>  angle gains | vp/vi <v> vel gains | yp <v> yaw gain"));
  Serial.println(F("# k <v>  cm/s per PWM | o <deg>  trim | T capture trim 5s"));
  Serial.println(F("# b <l> [r] deadband | z reset integrals | s telemetry"));
  Serial.println(F("# c gyro recal (still+disarmed) | m motor test (l/r <pwm>)"));
  Serial.println(F("# W  save settings to EEPROM"));
}

void resetMotion() {
  velLpf = vEst = vSet = vSetTarget = 0;
  velI = tiltCmd = 0;
  distCm = goTargetCm = 0;
  yawDeg = yawTarget = 0;
  turnSettleCount = 0;
  mode = M_HOLD;
}

void disarm(const __FlashStringHelper* why) {
  motors.disable();
  pid.reset();
  resetMotion();
  testPwmL = testPwmR = 0;
  state = DISARMED;
  Serial.print(F("# DISARMED: "));
  Serial.println(why);
}

// ------------------------------------------------------------------ serial --
// The console arrives over hardware Serial (pins 0/1) from EITHER the USB
// cable OR the ESP32 camera link, selected by the shield's mode switch.
// Plain lines work as always; the ESP32 additionally wraps commands as
// $payload*HH (HH = hex XOR of payload) for integrity — accept both.
void parseLine(char* line) {
  while (*line == ' ') line++;
  if (*line == '\0') return;

  if (*line == '$') {                       // checksummed frame
    char* star = strrchr(line, '*');
    if (star == NULL || star[1] == '\0' || star[2] == '\0') {
      Serial.println(F("# err: bad frame")); return;
    }
    uint8_t want = 0;
    for (uint8_t i = 1; i <= 2; i++) {
      char h = star[i];
      uint8_t v = (h >= '0' && h <= '9') ? h - '0'
                : (h >= 'A' && h <= 'F') ? h - 'A' + 10
                : (h >= 'a' && h <= 'f') ? h - 'a' + 10 : 0xFF;
      if (v == 0xFF) { Serial.println(F("# err: bad frame")); return; }
      want = (uint8_t)((want << 4) | v);
    }
    uint8_t got = 0;
    for (char* p = line + 1; p < star; p++) got ^= (uint8_t)*p;
    if (got != want) { Serial.println(F("# err: checksum")); return; }
    *star = '\0';
    line++;                                 // parse the verified payload
    while (*line == ' ') line++;
    if (*line == '\0') return;
  }
  char cmd = *line;
  char sub = *(line + 1);
  char* arg = line + 1;
  // two-character commands: vp, vi, yp
  if ((cmd == 'v' && (sub == 'p' || sub == 'i')) ||
      (cmd == 'y' && sub == 'p')) {
    arg = line + 2;
    while (*arg == ' ') arg++;
    float v = atof(arg);
    if (cmd == 'v' && sub == 'p') kvp = v;
    else if (cmd == 'v' && sub == 'i') kvi = v;
    else kyp = v;
    printGains();
    return;
  }
  while (*arg == ' ') arg++;

  switch (cmd) {
    case '?': printStatus(); break;
    case 'h': printHelp(); break;
    case 'a':
      if (!mpuOk) {
        Serial.println(F("# arm refused: MPU6050 offline (battery off?)"));
      } else if (vbat > VBAT_PRESENT && vbat < VBAT_MIN_ARM) {
        Serial.print(F("# arm refused: battery low ("));
        Serial.print(vbat, 2); Serial.println(F("V) — charge it"));
      } else if (state == DISARMED) {
        state = ARMING;
        armHolding = false;
        motors.enable();   // for the in-gate hum only; PWM stays sub-motion
        Serial.println(F("# ARMING: hold upright ~2 s (hum = in gate)"));
      } else {
        Serial.println(F("# arm refused: not disarmed"));
      }
      break;
    case 'p': pid.setTunings(atof(arg), pid.getKi(), pid.getKd()); printGains(); break;
    case 'i': pid.setTunings(pid.getKp(), atof(arg), pid.getKd()); printGains(); break;
    case 'd': pid.setTunings(pid.getKp(), pid.getKi(), atof(arg)); printGains(); break;
    case 'o':
      if (*arg) { angleTrim = atof(arg); printGains(); }
      else Serial.println(F("# o needs a value (or use T to capture)"));
      break;
    case 'C':
      if (!mpuOk) {
        Serial.println(F("# catch refused: MPU offline"));
      } else if (vbat > VBAT_PRESENT && vbat < VBAT_MIN_ARM) {
        Serial.println(F("# catch refused: battery low"));
      } else if (state == DISARMED) {
        state = CATCH;
        armHoldStartMs = millis();     // reused as catch-window start
        Serial.println(F("# CATCH armed: waiting for gate crossing"));
      } else {
        Serial.println(F("# catch refused: not disarmed"));
      }
      break;
    case 'S':
      if (!mpuOk) {
        Serial.println(F("# swingup refused: MPU offline"));
      } else if (vbat > VBAT_PRESENT && vbat < VBAT_MIN_ARM) {
        Serial.println(F("# swingup refused: battery low"));
      } else if (state == DISARMED) {
        state = SWINGUP;
        armHoldStartMs = millis();
        motors.enable();
        Serial.println(F("# SWINGUP: pumping"));
      } else {
        Serial.println(F("# swingup refused: not disarmed"));
      }
      break;
    case 'R':
      Serial.println(F("# rebooting to bootloader"));
      Serial.flush();
      {
        // Direct jump to optiboot (0x7E00 on a 512-byte-bootloader 328P).
        // WDT reset would skip the bootloader (optiboot boot-cause check).
        cli();
        asm volatile ("jmp 0x7E00");
      }
      break;
    case 'T':
      trimCapRemaining = TRIM_CAP_CYCLES;
      trimCapSum = 0.0f;
      Serial.println(F("# capturing trim: hold robot at balance point 5 s"));
      break;
    case 'v':
      if (state == BALANCING) {
        mode = M_HOLD;
        vSetTarget = constrain(atof(arg), -V_SET_MAX, V_SET_MAX);
        Serial.print(F("# v_set -> ")); Serial.println(vSetTarget, 1);
      } else Serial.println(F("# v refused: not balancing"));
      break;
    case 'g':
      if (state == BALANCING) {
        goTargetCm = distCm + atof(arg);
        mode = M_GO;
        yawTarget = yawDeg;          // hold current heading while driving
        Serial.print(F("# GO ")); Serial.print(atof(arg), 0);
        Serial.println(F(" cm"));
      } else Serial.println(F("# g refused: not balancing"));
      break;
    case 't':
      if (state == BALANCING) {
        yawTarget = yawDeg + atof(arg);
        mode = M_TURN;
        turnSettleCount = 0;
        vSetTarget = 0;
        Serial.print(F("# TURN to yaw ")); Serial.println(yawTarget, 1);
      } else Serial.println(F("# t refused: not balancing (trim is 'o' now)"));
      break;
    case 'k':
      if (*arg) { kvScale = atof(arg); printGains(); }
      break;
    case 'z':
      pid.reset(); velI = 0;
      Serial.println(F("# integrals reset"));
      break;
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
        if (mpu.calibrateGyro(500)) {
          Serial.println(F("# gyro cal done"));
          saveSettings();
        } else {
          Serial.println(F("# gyro cal REJECTED (movement) — old bias kept"));
        }
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
    case 'V':
      Serial.print(F("# vbat=")); Serial.print(vbat, 2);
      Serial.print(F("V (raw A0=")); Serial.print(analogRead(VBAT_PIN));
      Serial.println(F(")"));
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
  if (mpu.calibrateGyro(500)) {
    Serial.println(F("# gyro cal done ('c' to redo)"));
    saveSettings();   // persist bias for boots where cal gets rejected
  } else {
    Serial.println(F("# gyro cal REJECTED (movement) — stored bias in use, run 'c' when still"));
  }
  int16_t ax, ay, az;
  mpu.readAccelerometer(ax, ay, az);
  kalman.setAngle(atan2f((float)ay / ACC_LSB_PER_G, (float)az / ACC_LSB_PER_G)
                  * 180.0f / PI);
}

// -------------------------------------------------- middle/outer loops 20Hz --
void runMidLoops(float midDt) {
  // Outer: mission profiles set vSetTarget
  if (mode == M_GO) {
    float remaining = goTargetCm - distCm;
    if (fabsf(remaining) < GO_DONE_CM) {
      mode = M_HOLD;
      vSetTarget = 0;
      Serial.print(F("# GO done, dist=")); Serial.println(distCm, 1);
    } else {
      // trapezoid: braking-limited speed toward target, capped at cruise
      float vLim = sqrtf(2.0f * V_DECEL * fabsf(remaining));
      if (vLim > V_CRUISE) vLim = V_CRUISE;
      vSetTarget = (remaining > 0) ? vLim : -vLim;
    }
  } else if (mode == M_TURN) {
    vSetTarget = 0;
    if (fabsf(yawTarget - yawDeg) < TURN_DONE_DEG) {
      if (++turnSettleCount >= 10) {   // 0.5 s settled
        mode = M_HOLD;
        Serial.print(F("# TURN done, yaw=")); Serial.println(yawDeg, 1);
      }
    } else {
      turnSettleCount = 0;
    }
  }

  // Slew-rate limit the velocity setpoint (balance loop never sees steps)
  float dvMax = V_ACCEL * midDt;
  float dv = vSetTarget - vSet;
  if (dv > dvMax) dv = dvMax;
  else if (dv < -dvMax) dv = -dvMax;
  vSet += dv;

  // Velocity PI -> tilt command (+ = lean forward). Integral clamped to the
  // tilt authority so it can't wind up.
  float velErr = vSet - vEst;
  velI += kvi * velErr * midDt;
  if (velI > TILT_CMD_MAX) velI = TILT_CMD_MAX;
  else if (velI < -TILT_CMD_MAX) velI = -TILT_CMD_MAX;
  tiltCmd = kvp * velErr + velI;
  if (tiltCmd > TILT_CMD_MAX) tiltCmd = TILT_CMD_MAX;
  else if (tiltCmd < -TILT_CMD_MAX) tiltCmd = -TILT_CMD_MAX;

  // Soft-start: fade the velocity loop in after arming
  uint32_t sinceArm = millis() - armedAtMs;
  if (sinceArm < VEL_RAMP_MS) {
    tiltCmd *= (float)sinceArm / VEL_RAMP_MS;
  }
}

// ------------------------------------------------------------------- setup --
void setup() {
  Serial.begin(38400);   // shared console: USB and ESP32 link (shifter-limited)
  Wire.begin();
  Wire.setClock(400000);

  motors.init();                 // driver in standby, PWM zero
  pinMode(LED_BUILTIN, OUTPUT);  // arming guide: lit = angle inside arm gate
  Serial.println(F("# self-balancing-robot fw3 — motors DISARMED at boot"));

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

  // Battery voltage: sample every 100th cycle (2 Hz), light smoothing
  if (++vbatCount >= 100) {
    vbatCount = 0;
    float v = analogRead(VBAT_PIN) * VBAT_V_PER_COUNT;
    vbat = (vbat == 0.0f) ? v : vbat + 0.2f * (v - vbat);
    // Sag compensation: same physical torque per commanded count as at
    // the calibration voltage. Only when actually on battery power.
    if (vbat > VBAT_PRESENT) {
      motors.setVoltComp(constrain(VBAT_REF / vbat, 0.8f, 1.4f));
    } else {
      motors.setVoltComp(1.0f);
    }
  }

  float acc_angle = 0, angle = 0, gy_dps = 0, gx_dps = 0, gz_dps = 0;
  if (!mpuOk) {
    // IMU may be recovering: keep the console alive, retry quietly.
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

    // Freshness watchdog: a live sensor always jitters by >=1 LSB. If all
    // six raw values are bit-identical for 100 cycles (0.5 s), the MPU's
    // measurement core has frozen (I2C still answers!) — the tilt cutoff
    // would never fire on frozen data, so disarm NOW and force a re-init.
    static int16_t lastRaw[6];
    static uint8_t staleCount = 0;
    if (ax == lastRaw[0] && ay == lastRaw[1] && az == lastRaw[2] &&
        gx == lastRaw[3] && gy == lastRaw[4] && gz == lastRaw[5]) {
      if (staleCount < 255) staleCount++;
    } else {
      staleCount = 0;
    }
    lastRaw[0] = ax; lastRaw[1] = ay; lastRaw[2] = az;
    lastRaw[3] = gx; lastRaw[4] = gy; lastRaw[5] = gz;
    if (staleCount >= 100) {
      staleCount = 0;
      mpuOk = false;
      lastMpuRetryMs = 0;          // retry (with device reset) immediately
      if (state == BALANCING || state == ARMING) {
        disarm(F("IMU FROZEN"));
      } else {
        Serial.println(F("# IMU FROZEN — resetting"));
      }
      return;
    }

    float ay_g = (float)ay / ACC_LSB_PER_G;
    float az_g = (float)az / ACC_LSB_PER_G;
    gy_dps = (float)gy / GYRO_LSB_PER_DPS;
    gx_dps = (float)gx / GYRO_LSB_PER_DPS;
    gz_dps = (float)gz / GYRO_LSB_PER_DPS;

    acc_angle = atan2f(ay_g, az_g) * 180.0f / PI;
    // Pitch (fall) axis is the X gyro: IMU X runs along the wheel axle,
    // positive gx = angle increasing (verified empirically in trial_014).
    angle = kalman.getAngle(acc_angle, gx_dps, dt);
    // Yaw integrates whenever the IMU is up (bench-testable); re-zeroed
    // at arm, so missions always start from yaw 0.
    yawDeg += gz_dps * dt;
  }

  if (mpuOk && trimCapRemaining > 0) {
    trimCapSum += angle;
    if (--trimCapRemaining == 0) {
      angleTrim = trimCapSum / TRIM_CAP_CYCLES;
      Serial.print(F("# trim captured: "));
      Serial.println(angleTrim, 2);
    }
  }

  // LED: in ARMING, lit while the angle is inside the gate (hold-still
  // guide for Vish); solid while BALANCING; off otherwise.
  if (state == ARMING)
    digitalWrite(LED_BUILTIN, fabsf(angle - angleTrim) < ARM_TOL_DEG);
  else
    digitalWrite(LED_BUILTIN, state == BALANCING);

  int motorOut = 0;

  switch (state) {
    case ARMING: {
      if (fabsf(angle - angleTrim) < ARM_TOL_DEG) {
        motors.driveRaw(ARM_HUM_PWM, ARM_HUM_PWM);  // audible in-gate cue
        if (!armHolding) { armHolding = true; armHoldStartMs = millis(); }
        else if (millis() - armHoldStartMs >= ARM_HOLD_MS) {
          pid.reset();
          resetMotion();               // dist/yaw zeroed at arm point
          kalman.setAngle(acc_angle);
          armedAtMs = millis();
          state = BALANCING;
          Serial.println(F("# ARMED — station keeping"));
        }
      } else {
        motors.driveRaw(0, 0);
        armHolding = false;
      }
      break;
    }
    case CATCH: {
      // Motors stay disabled until the swing crosses the gate slowly
      // enough; then hand off to the balance controller instantly.
      if (millis() - armHoldStartMs > CATCH_TIMEOUT_MS) {
        disarm(F("catch window expired"));
        break;
      }
      if (fabsf(angle - angleTrim) < ARM_TOL_DEG &&
          fabsf(gx_dps) < CATCH_RATE_MAX_DPS) {
        pid.reset();
        resetMotion();
        kalman.setAngle(acc_angle);
        armedAtMs = millis();
        motors.enable();
        state = BALANCING;
        Serial.println(F("# CAUGHT — balancing"));
      }
      break;
    }
    case SWINGUP: {
      if (millis() - armHoldStartMs > SWING_TIMEOUT_MS) {
        disarm(F("swingup timeout"));
        break;
      }
      float rel = angle - angleTrim;
      // native catch, same condition as CATCH state
      if (fabsf(rel) < ARM_TOL_DEG && fabsf(gx_dps) < CATCH_RATE_MAX_DPS) {
        pid.reset();
        resetMotion();
        kalman.setAngle(acc_angle);
        armedAtMs = millis();
        state = BALANCING;
        Serial.println(F("# CAUGHT (swingup) — balancing"));
        break;
      }
      // bang-bang energy pump with top taper: pump while moving toward
      // upright, but COAST once within SWING_TAPER_DEG of the top so the
      // gate crossing is slow enough to catch (trial 067: un-tapered
      // pumping blew through the gate at >60 dps from the rear rest).
      float closing = (rel > 0 ? gx_dps : -gx_dps);
      if (fabsf(rel) < SWING_TAPER_DEG) {
        motors.driveRaw(0, 0);
      } else if (closing < -5.0f) {
        motors.driveRaw(SWING_PUMP_PWM, SWING_PUMP_PWM);
      } else if (closing > 5.0f) {
        motors.driveRaw(0, 0);
      } else {
        // near-stationary on the rest: seed motion with the settle pulse
        motors.driveRaw(SWING_SETTLE_PWM + (int)(millis() % 40),
                        SWING_SETTLE_PWM + (int)(millis() % 40));
      }
      break;
    }
    case BALANCING: {
      float rel = angle - angleTrim;
      if (rel < -TILT_CUTOFF_FWD_DEG || rel > TILT_CUTOFF_BACK_DEG) {
        disarm(F("tilt cutoff"));
        break;
      }
      // Dead reckoning, every inner cycle
      vEst = (fabsf(velLpf) < VEL_EST_DEADZONE) ? 0.0f : kvScale * velLpf;
      distCm += vEst * dt;

      if (++midCount >= MID_DECIM) {
        midCount = 0;
        runMidLoops(LOOP_US * 1e-6f * MID_DECIM);
      }

      // Inner loop: tiltCmd + = lean forward = setpoint below trim
      float out = pid.compute(angleTrim - tiltCmd, angle, gx_dps, dt);
      motorOut = (int)constrain(out, -255, 255);

      // Commanded-PWM velocity estimate feed (pre-differential)
      float alpha = dt / (VEL_LPF_TAU + dt);
      velLpf += alpha * ((float)motorOut - velLpf);

      // Heading hold / turn: differential term
      int turnOut = (int)constrain(kyp * (yawTarget - yawDeg),
                                   (float)-TURN_OUT_MAX, (float)TURN_OUT_MAX);
      motors.drive(constrain(motorOut - turnOut, -255, 255),
                   constrain(motorOut + turnOut, -255, 255));
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
    Serial.print(gz_dps, 2);         Serial.print(',');
    Serial.print(tiltCmd, 2);        Serial.print(',');
    Serial.print(vEst, 1);           Serial.print(',');
    Serial.print(vSet, 1);           Serial.print(',');
    Serial.print(distCm, 1);         Serial.print(',');
    Serial.print(yawDeg, 1);         Serial.print(',');
    Serial.println(vbat, 2);
  }
}
