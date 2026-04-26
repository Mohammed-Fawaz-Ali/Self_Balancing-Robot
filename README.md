# ⚖️ Self-Balancing Robot — Arduino Uno

A two-wheeled self-balancing robot that implements the classic **Inverted Pendulum** control problem using a PID algorithm, MPU6050 sensor fusion, and an Arduino Uno. The robot continuously reads its tilt angle and drives its wheels to stay upright — no external support needed.

---

## 📋 Table of Contents

- [Demo](#-demo)
- [Objective](#-objective)
- [Physics](#-physics--the-inverted-pendulum)
- [Hardware Required](#-hardware-required)
- [Circuit Connections](#-circuit-connections)
- [Code](#-code)
- [How It Works](#-how-it-works)
- [Version 2.0 — Bluetooth Control](#-version-20--bluetooth-control)
- [Project Structure](#-project-structure)
- [License](#-license)

---

## 🎥 Demo

**Robot in action — push test and carry load:**

https://github.com/user-attachments/assets/Video

---

## 📸 Photos

| Front view | Side view |
|---|---|
| ![Robot front view](Image3) | ![Robot side view](Image2) |

---

## 🎯 Objective

The primary objective is to design and construct a two-wheeled robotic vehicle that can maintain an upright position without external support. Specific goals include:

- **Mechanical Design** — Build a chassis where the Center of Mass (COM) is balanced above the wheel axis
- **Sensor Fusion** — Combine accelerometer and gyroscope data (MPU6050) to obtain a noise-free, accurate tilt angle
- **Closed-Loop Control** — Implement a PID algorithm that continuously calculates and applies the precise motor speed required to counteract gravity

---

## ⚛️ Physics — The Inverted Pendulum

### The Broomstick Analogy

Imagine balancing a broomstick upright on your palm. If it leans forward, you run forward to "catch" it — by moving your hand under the center of mass, you restore balance. The robot does exactly this, but using wheels.

### Forces at Play

**1. Unstable Equilibrium**
The robot is perfectly balanced only when the Center of Mass (COM) is directly vertical above the wheel axis (pivot point). Any deviation is unstable.

**2. Gravitational Torque**
The moment the robot tilts by angle θ, gravity creates a twisting torque trying to rotate the robot into the ground:

```
τ_gravity = m · g · L · sin(θ)
```

Where `m` = mass, `g` = 9.8 m/s², `L` = distance from pivot to COM, `θ` = tilt angle.

**3. Restoring Torque**
To counteract the fall, motors drive the wheels *in the direction of the fall*. This acceleration creates a reaction force that pushes the wheels back under the COM. For the robot to recover:

```
τ_motor > m · g · sin(θ) · L
```

**Goal: Keep θ ≈ 0** — achieved through active PID control.

![Inverted Pendulum](/photos/Formula.jpg)

---

## 🛒 Hardware Required

| # | Component | Price (approx.) |
|---|---|---|
| 1 | Arduino Uno | ₹200–₹300 |
| 2 | MPU6050 (6-axis IMU) | ₹150–₹200 |
| 3 | BO Motors × 2 | ₹100 |
| 4 | Motor Driver (L298N module) | ₹200 |
| 5 | 7.4V Li-ion Battery (2× 18650) | ₹400 |
| — | Wheels, chassis, jumper wires | ₹150–₹200 |

**Total estimated cost: ₹1,200–₹1,400**

---

## 🔌 Circuit Connections

![Circuit diagram](/photos/Components/circuit_diagram.png)

> *Circuit designed using Cirkit Designer*

### Pin Map

| Arduino Uno Pin | Connected To | Notes |
|---|---|---|
| `A4 (SDA)` | MPU6050 SDA | I2C data |
| `A5 (SCL)` | MPU6050 SCL | I2C clock |
| `5V` | MPU6050 VCC | Logic power |
| `GND` | MPU6050 GND | Common ground |
| `D5` | Motor Driver IN1 | Left motor direction A (PWM) |
| `D6` | Motor Driver IN2 | Left motor direction B (PWM) |
| `D9` | Motor Driver IN3 | Right motor direction A (PWM) |
| `D10` | Motor Driver IN4 | Right motor direction B (PWM) |
| `Vin` | Motor Driver 5V out | Arduino power from driver |
| `GND` | Motor Driver GND | Common ground |

> ⚠️ **Power:** Connect the 7.4V battery to the motor driver's **12V terminal**. The driver's onboard 5V regulator powers the Arduino via `Vin`. Do **not** run motors directly from the Arduino's 5V pin.

---

## 💻 Code

**Version 1.0 — Self-balancing (PID):**
[Download from Google Drive](Version1.ino)

**Version 2.0 — Bluetooth control:**
[Download from Google Drive](Version2.ino)

### How to Upload

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Install required libraries:
   - `MPU6050` by Electronic Cats — *Sketch → Include Library → Manage Libraries*
   - `Wire` (built-in)
3. Connect Arduino Uno via USB
4. Select **Tools → Board → Arduino Uno**
5. Select the correct COM port
6. Upload the sketch

---

## ⚙️ How It Works

The robot runs a continuous control loop:

```
1. Read raw accelerometer + gyroscope data from MPU6050
2. Apply Complementary Filter → fused tilt angle θ
        angle = 0.98 × (angle + gyro_rate × dt)
              + 0.02 × (accel_angle)
3. Compute PID error:
        error = setpoint (0°) − current_angle (θ)
4. Calculate correction:
        output = Kp×error + Ki×∫error + Kd×(Δerror/dt)
5. Apply output as PWM to motors in direction of the fall
6. Repeat at ~100Hz
```

### PID Tuning Guide

| Term | Effect | Start value |
|---|---|---|
| `Kp` | Immediate correction strength — too high = oscillation | 20–30 |
| `Ki` | Corrects steady-state drift — too high = windup | 0.5–2 |
| `Kd` | Dampens oscillation — too high = noise amplification | 0.5–1.5 |

**Tuning order:** Set Ki=0, Kd=0 → increase Kp until it balances but oscillates → add Kd to reduce oscillation → add small Ki to correct drift.

---

## 📡 Version 2.0 — Bluetooth Control

Version 2.0 adds an **HC-05 Bluetooth module**, allowing you to control the robot's target angle (setpoint) from a smartphone app. By shifting the setpoint slightly forward or backward, the robot drives in that direction.

**Demo:**

https://github.com/user-attachments/assets/Video2

**Code:** [Download from Google Drive](https://drive.google.com/file/d/1PlQHFsJB8ggDSAgfRgIGTlxw6PcYEtx/view?usp=sharing)

---

## 📁 Project Structure

```
self-balancing-robot/
├── Image1                    ← Robot front-view photo
├── Image2                    ← Robot side-view photo
├── Video                     ← Demo video (push test + load)
├── circuit_diagram.png       ← Cirkit Designer schematic
├── README.md                 ← You are here
└── LICENSE
```

---

## 🗺️ Roadmap / Upgrade Ideas

- [ ] Replace Complementary Filter with Kalman Filter for better angle accuracy
- [ ] Add OLED display to show live PID values and tilt angle
- [ ] Upgrade to ESP32 for WiFi-based PID tuning dashboard
- [ ] Add rotary encoders for velocity feedback (full state-space control)

---

## 📄 License

MIT License — free to use, modify, and distribute with attribution.

---

## 🙋 Author

**Fawaz**
SR University, Telangana | Centre for Creative Cognition

> *Practical application of the Inverted Pendulum problem — a benchmark in control engineering used in Segways, rockets, and bipedal robots.*
