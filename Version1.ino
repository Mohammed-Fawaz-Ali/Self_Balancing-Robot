/*
 * ============================================================
 *  Self-Balancing Robot — Version 1.0
 * ============================================================
 *  Controller : PID
 *  Sensor     : MPU6050 (I2C, no external library needed)
 *  Angle est. : Complementary Filter
 *  Board      : Arduino Uno
 *
 *  Pin Map:
 *    MPU6050 SDA → A4
 *    MPU6050 SCL → A5
 *    MPU6050 VCC → 5V
 *    MPU6050 GND → GND
 *
 *    L298N IN1   → D5  (Left  motor, direction A / PWM)
 *    L298N IN2   → D6  (Left  motor, direction B / PWM)
 *    L298N IN3   → D9  (Right motor, direction A / PWM)
 *    L298N IN4   → D10 (Right motor, direction B / PWM)
 *    L298N ENA   → jumper to 5V (always enabled)
 *    L298N ENB   → jumper to 5V (always enabled)
 *
 *  NOTE: Uses Wire (built-in). No extra libraries required.
 *
 *  Author  : Fawaz
 *  Version : 1.0.0
 *  License : MIT
 * ============================================================
 */

#include <Wire.h>

// ─── Motor Pins ─────────────────────────────────────────────
#define L_IN1   5    // Left  motor — PWM pin A
#define L_IN2   6    // Left  motor — PWM pin B
#define R_IN1   9    // Right motor — PWM pin A
#define R_IN2  10    // Right motor — PWM pin B

// ─── MPU6050 ────────────────────────────────────────────────
#define MPU_ADDR      0x68
#define ACCEL_SCALE   16384.0   // ±2g  → LSB/g
#define GYRO_SCALE    131.0     // ±250°/s → LSB/(°/s)

// ─── PID Constants ──────────────────────────────────────────
// START HERE when tuning:
//  1. Set Ki=0, Kd=0. Raise Kp until robot balances but oscillates.
//  2. Raise Kd until oscillation damps out.
//  3. Add small Ki to correct slow drift.
float Kp = 25.0;
float Ki =  1.0;
float Kd =  0.8;

// ─── Setpoint ───────────────────────────────────────────────
// The angle (in degrees) the robot tries to maintain.
// 0.0 = perfectly vertical. Adjust ±2° to account for
// your robot's actual center-of-mass lean.
float SETPOINT = 0.0;

// ─── Safety cutoff ──────────────────────────────────────────
// If tilt exceeds this the robot has fallen — stop motors
// to prevent damage / runaway.
#define FALL_ANGLE  35.0

// ─── Complementary filter weight ────────────────────────────
// 0.98 trusts gyro 98% (smooth), accelerometer 2% (drift fix)
#define ALPHA  0.98

// ─── Globals ────────────────────────────────────────────────
float  angle    = 0.0;    // fused tilt angle (degrees)
float  pidError = 0.0;
float  pidPrev  = 0.0;
float  pidI     = 0.0;
long   lastTime = 0;

// Raw sensor data
int16_t ax, ay, az;
int16_t gx, gy, gz;

// ─── MPU6050 helpers ────────────────────────────────────────

void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0x00);   // wake up
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);   // GYRO_CONFIG
  Wire.write(0x00);   // ±250°/s
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);   // ACCEL_CONFIG
  Wire.write(0x00);   // ±2g
  Wire.endTransmission();
}

void mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // start at ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();             // skip temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

// ─── Angle calculation ──────────────────────────────────────
// ORIENTATION NOTE:
//   This assumes the MPU6050 is mounted with the chip face
//   pointing FORWARD and the X-axis pointing UP on the chassis.
//   If the angle reads backwards, negate accelAngle and gyroRate.

float getAngle(float dt) {
  float accelAngle = atan2((float)ay, (float)az) * 180.0 / PI;
  float gyroRate   = (float)gx / GYRO_SCALE;   // deg/s around X-axis

  // Complementary filter: blend gyro integration with accel correction
  angle = ALPHA * (angle + gyroRate * dt) + (1.0 - ALPHA) * accelAngle;
  return angle;
}

// ─── Motor control ──────────────────────────────────────────

/** Drive left motor. speed -255…+255. Positive = forward. */
void leftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    analogWrite(L_IN1, speed);
    analogWrite(L_IN2, 0);
  } else {
    analogWrite(L_IN1, 0);
    analogWrite(L_IN2, -speed);
  }
}

/** Drive right motor. speed -255…+255. Positive = forward. */
void rightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    analogWrite(R_IN1, speed);
    analogWrite(R_IN2, 0);
  } else {
    analogWrite(R_IN1, 0);
    analogWrite(R_IN2, -speed);
  }
}

void stopMotors() {
  analogWrite(L_IN1, 0); analogWrite(L_IN2, 0);
  analogWrite(R_IN1, 0); analogWrite(R_IN2, 0);
}

// ─── Setup ──────────────────────────────────────────────────
void setup() {
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  stopMotors();

  Wire.begin();
  mpuInit();

  Serial.begin(115200);
  Serial.println(F("Self-Balancing Robot v1.0"));
  Serial.println(F("Warming up MPU6050..."));

  // Let the complementary filter settle
  delay(2000);

  // Prime the angle with a few readings
  for (int i = 0; i < 100; i++) {
    mpuRead();
    angle = atan2((float)ay, (float)az) * 180.0 / PI;
    delay(5);
  }

  lastTime = millis();
  Serial.println(F("Running. Place robot upright."));
}

// ─── Main Loop ──────────────────────────────────────────────
void loop() {
  long now = millis();
  float dt  = (now - lastTime) / 1000.0;   // seconds
  lastTime  = now;

  mpuRead();
  float currentAngle = getAngle(dt);

  // Safety: if fallen, stop everything
  if (abs(currentAngle) > FALL_ANGLE) {
    stopMotors();
    pidI = 0;   // reset integral to prevent windup on restart
    Serial.println(F("FALLEN — motors stopped."));
    delay(500);
    return;
  }

  // ── PID ─────────────────────────────────────────────────
  pidError = SETPOINT - currentAngle;

  float pidP = Kp * pidError;
  pidI      += Ki * pidError * dt;
  pidI       = constrain(pidI, -150, 150);   // anti-windup clamp
  float pidD = Kd * (pidError - pidPrev) / dt;
  pidPrev    = pidError;

  float output = pidP + pidI + pidD;
  output = constrain(output, -255, 255);

  // Positive output → robot leaning forward → drive forward to catch it
  leftMotor((int) output);
  rightMotor((int) output);

  // ── Serial debug ────────────────────────────────────────
  // Comment out if loop feels slow
  Serial.print(F("Angle: ")); Serial.print(currentAngle, 2);
  Serial.print(F("  P: ")); Serial.print(pidP, 1);
  Serial.print(F("  I: ")); Serial.print(pidI, 1);
  Serial.print(F("  D: ")); Serial.print(pidD, 1);
  Serial.print(F("  Out: ")); Serial.println(output, 1);
}
