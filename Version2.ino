/*
 * ============================================================
 *  Self-Balancing Robot — Version 2.0 (Bluetooth Control)
 * ============================================================
 *  Added     : HC-05 Bluetooth module for smartphone control
 *  Control   : Shift the PID setpoint to drive forward/backward
 *
 *  Extra pin (vs v1.0):
 *    HC-05 TX  → D2  (SoftwareSerial RX)
 *    HC-05 RX  → D3  via 1kΩ + 2kΩ voltage divider
 *                    (Arduino 5V → HC-05 3.3V logic)
 *    HC-05 VCC → 5V
 *    HC-05 GND → GND
 *
 *  Bluetooth Commands (send from any Serial Bluetooth app):
 *    'F'  → lean forward  (drive forward)
 *    'B'  → lean backward (drive backward)
 *    'S'  → stop moving   (return to balance setpoint)
 *    '+'  → increase base speed
 *    '-'  → decrease base speed
 *
 *  All other pins same as v1.0.
 *
 *  NOTE: Disconnect HC-05 TX/RX from D2/D3 before uploading.
 *
 *  Author  : Fawaz
 *  Version : 2.0.0
 *  License : MIT
 * ============================================================
 */

#include <Wire.h>
#include <SoftwareSerial.h>

// ─── Bluetooth ──────────────────────────────────────────────
#define BT_RX   2    // connect to HC-05 TX
#define BT_TX   3    // connect to HC-05 RX (via voltage divider)
SoftwareSerial bluetooth(BT_RX, BT_TX);

// ─── Motor Pins ─────────────────────────────────────────────
#define L_IN1   5
#define L_IN2   6
#define R_IN1   9
#define R_IN2  10

// ─── MPU6050 ────────────────────────────────────────────────
#define MPU_ADDR      0x68
#define ACCEL_SCALE   16384.0
#define GYRO_SCALE    131.0

// ─── PID Constants ──────────────────────────────────────────
float Kp = 25.0;
float Ki =  1.0;
float Kd =  0.8;

// ─── Setpoint control ───────────────────────────────────────
// BALANCE_POINT: calibrated resting angle (same as v1.0 SETPOINT)
// DRIVE_OFFSET:  how many degrees to tilt for forward/backward motion
float BALANCE_POINT = 0.0;
float DRIVE_OFFSET  = 3.0;     // degrees of lean to drive
float setpoint      = BALANCE_POINT;

#define FALL_ANGLE  35.0
#define ALPHA       0.98

// ─── Globals ────────────────────────────────────────────────
float  angle    = 0.0;
float  pidError = 0.0;
float  pidPrev  = 0.0;
float  pidI     = 0.0;
long   lastTime = 0;
int16_t ax, ay, az, gx, gy, gz;

// ─── MPU6050 helpers ────────────────────────────────────────

void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00);
  Wire.endTransmission();
}

void mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

float getAngle(float dt) {
  float accelAngle = atan2((float)ay, (float)az) * 180.0 / PI;
  float gyroRate   = (float)gx / GYRO_SCALE;
  angle = ALPHA * (angle + gyroRate * dt) + (1.0 - ALPHA) * accelAngle;
  return angle;
}

// ─── Motor control ──────────────────────────────────────────

void leftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { analogWrite(L_IN1, speed);  analogWrite(L_IN2, 0); }
  else            { analogWrite(L_IN1, 0);       analogWrite(L_IN2, -speed); }
}

void rightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { analogWrite(R_IN1, speed);  analogWrite(R_IN2, 0); }
  else            { analogWrite(R_IN1, 0);       analogWrite(R_IN2, -speed); }
}

void stopMotors() {
  analogWrite(L_IN1, 0); analogWrite(L_IN2, 0);
  analogWrite(R_IN1, 0); analogWrite(R_IN2, 0);
}

// ─── Bluetooth command handler ──────────────────────────────

void handleBluetooth() {
  if (!bluetooth.available()) return;

  char cmd = bluetooth.read();

  switch (cmd) {
    case 'F':
      // Lean forward: shift setpoint in direction of forward fall
      setpoint = BALANCE_POINT + DRIVE_OFFSET;
      Serial.println(F("BT: Forward"));
      break;

    case 'B':
      // Lean backward
      setpoint = BALANCE_POINT - DRIVE_OFFSET;
      Serial.println(F("BT: Backward"));
      break;

    case 'S':
      // Return to balance
      setpoint = BALANCE_POINT;
      Serial.println(F("BT: Stop"));
      break;

    case '+':
      // Increase drive offset for faster movement
      DRIVE_OFFSET = min(DRIVE_OFFSET + 0.5, 8.0);
      Serial.print(F("BT: Speed up → offset = "));
      Serial.println(DRIVE_OFFSET);
      break;

    case '-':
      // Decrease drive offset for slower movement
      DRIVE_OFFSET = max(DRIVE_OFFSET - 0.5, 1.0);
      Serial.print(F("BT: Speed down → offset = "));
      Serial.println(DRIVE_OFFSET);
      break;

    default:
      break;
  }
}

// ─── Setup ──────────────────────────────────────────────────
void setup() {
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  stopMotors();

  Wire.begin();
  mpuInit();

  Serial.begin(115200);
  bluetooth.begin(9600);    // HC-05 default baud rate

  Serial.println(F("Self-Balancing Robot v2.0 — Bluetooth enabled"));
  Serial.println(F("Commands: F=Forward  B=Backward  S=Stop  +=Faster  -=Slower"));

  delay(2000);

  // Prime angle estimate
  for (int i = 0; i < 100; i++) {
    mpuRead();
    angle = atan2((float)ay, (float)az) * 180.0 / PI;
    delay(5);
  }

  setpoint = BALANCE_POINT;
  lastTime = millis();
  Serial.println(F("Ready. Place robot upright."));
}

// ─── Main Loop ──────────────────────────────────────────────
void loop() {
  // Check Bluetooth first (non-blocking)
  handleBluetooth();

  long now = millis();
  float dt  = (now - lastTime) / 1000.0;
  lastTime  = now;

  mpuRead();
  float currentAngle = getAngle(dt);

  // Safety fall detection
  if (abs(currentAngle) > FALL_ANGLE) {
    stopMotors();
    pidI     = 0;
    setpoint = BALANCE_POINT;   // reset to balance on restart
    Serial.println(F("FALLEN — motors stopped."));
    delay(500);
    return;
  }

  // ── PID ─────────────────────────────────────────────────
  pidError = setpoint - currentAngle;

  float pidP = Kp * pidError;
  pidI      += Ki * pidError * dt;
  pidI       = constrain(pidI, -150, 150);
  float pidD = Kd * (pidError - pidPrev) / dt;
  pidPrev    = pidError;

  float output = pidP + pidI + pidD;
  output = constrain(output, -255, 255);

  leftMotor((int) output);
  rightMotor((int) output);

  // ── Serial debug ────────────────────────────────────────
  Serial.print(F("Angle: ")); Serial.print(currentAngle, 2);
  Serial.print(F("  SP: ")); Serial.print(setpoint, 1);
  Serial.print(F("  Out: ")); Serial.println(output, 1);
}
