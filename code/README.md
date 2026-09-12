# Arduino Self-Balancing Robot: Setup and Tuning Guide

A compact two-wheel self-balancing robot built with an Arduino Nano, MPU6050 IMU, DRV8833 motor driver, TT geared motors, and a 7.2 V battery.

This project uses a 500 Hz balance control loop, an adaptive complementary filter for angle estimation, gyroscope-based derivative feedback, motor deadband compensation, and high-frequency PWM motor control.

The repository also includes STL files for the 3D-printed mechanical parts.

---

## Features

- Arduino Nano based controller
- MPU6050 accelerometer and gyroscope
- DRV8833 dual motor driver
- Two TT geared DC motors
- 500 Hz control loop
- 400 kHz I2C communication
- Approximately 31.4 kHz motor PWM
- Adaptive complementary filter
- PID-based balance control
- Gyroscope-based derivative feedback
- Automatic gyroscope bias calibration
- Motor deadband compensation
- Left/right motor compensation
- Integral anti-windup
- Automatic arming
- Automatic motor shutdown after falling
- Real-time PID tuning through the Serial Monitor

---

## Hardware

The main components used in this project are:

| Component | Quantity |
|---|---:|
| Arduino Nano | 1 |
| MPU6050 IMU | 1 |
| DRV8833 motor driver | 1 |
| TT geared DC motor | 2 |
| Wheels | 2 |
| 7.2 V battery | 1 |
| 3D-printed frame | 1 set |

Additional hardware such as screws, spacers, wires, and a power switch may be required depending on your mechanical design.

---

## Wiring

### MPU6050

For an Arduino Nano:

| MPU6050 | Arduino Nano |
|---|---|
| SDA | A4 |
| SCL | A5 |
| GND | GND |
| VCC | According to your MPU6050 breakout board specification |

> Important: MPU6050 breakout boards are not all identical. Check the voltage requirements of your specific module before connecting VCC.

---

### DRV8833

The default pin configuration used by the program is:

| DRV8833 input | Arduino Nano |
|---|---:|
| AIN1 | D9 |
| AIN2 | D10 |
| BIN1 | D11 |
| BIN2 | D3 |
| nSLEEP / STBY, if used | D7 |

These pins were selected because they allow the Arduino Nano timers to generate approximately 31.4 kHz PWM.

Connect one motor to Motor A and the other motor to Motor B.

The Arduino, MPU6050, DRV8833, and battery system must share a common ground.

---

## Software Requirements

The program is designed for an Arduino Nano using the ATmega328P.

You can upload it using the Arduino IDE.

No external Arduino library is required.

The program only uses the built-in:

```cpp
#include <Wire.h>
```

library.

If uploading to an older or clone Arduino Nano fails, you may need to try:

```text
Tools
→ Processor
→ ATmega328P (Old Bootloader)
```

depending on your board.

---

# Before Running the Robot

Do not immediately place the robot on the floor after uploading the program.

A wrong motor direction or IMU direction can create positive feedback and make the robot accelerate instead of balancing.

For the first test, keep the wheels off the ground.

It is recommended to perform the setup in the following order:

1. Check the MPU6050 axis
2. Check the angle direction
3. Check both motor directions
4. Check the controller direction
5. Measure the upright angle offset
6. Tune the PD controller
7. Adjust the target angle
8. Tune the motor deadband
9. Add integral control if necessary

---

# 1. Start the Robot

Upload the program and open the Arduino Serial Monitor.

Set the baud rate to:

```text
115200
```

After startup, you should see output similar to:

```text
Self Balancing Robot

MPU6050 WHO_AM_I = 0x68

Gyro calibration...
DO NOT MOVE ROBOT

Gyro bias X = ...
Gyro bias Y = ...
Gyro bias Z = ...

Calibration done.
Ready.
```

Keep the robot completely still while:

```text
Gyro calibration...
DO NOT MOVE ROBOT
```

is displayed.

The startup calibration measures the gyroscope zero-rate bias.

---

# 2. Understand the Serial Output

The controller periodically sends data such as:

```text
A:1.253 Acc:1.301 G:0.21 U:0.0 Arm:0
```

The values mean:

| Value | Meaning |
|---|---|
| `A` | Estimated robot angle |
| `Acc` | Angle calculated from the accelerometer |
| `G` | Gyroscope angular velocity in deg/s |
| `U` | Motor controller output |
| `Arm` | Controller armed state |

For example:

```text
Arm:0
```

means the motors are disabled.

```text
Arm:1
```

means the balance controller is active.

---

# 3. Select the Correct MPU6050 Axis

The program supports rotation around either the X or Y axis.

The current sketch uses:

```cpp
const BalanceAxis BALANCE_AXIS = ROTATION_X;
```

Slowly tilt the robot forward and backward while watching:

```text
A:
```

in the Serial Monitor.

If the value changes significantly, the selected axis is probably correct.

If it barely changes, try:

```cpp
const BalanceAxis BALANCE_AXIS = ROTATION_X;
```

instead.

---

# 4. Check the IMU Direction

Slowly tilt the robot forward.

The angle should increase or decrease smoothly.

If the sign is opposite to the direction expected by your mechanical configuration, change:

```cpp
const int8_t IMU_SIGN = 1;
```

to:

```cpp
const int8_t IMU_SIGN = -1;
```

---

# 5. Check Motor Direction

Before placing the robot on the ground, lift it so that both wheels can rotate freely.

Both wheels must move in the same physical direction when the controller commands forward motion.

The default settings are:

```cpp
const int8_t LEFT_MOTOR_SIGN  = 1;
const int8_t RIGHT_MOTOR_SIGN = -1;
```

If one wheel rotates in the wrong direction, change the corresponding sign.

For example:

```cpp
const int8_t LEFT_MOTOR_SIGN = -1;
```

Do not change both motor signs unless both motors are reversed.

---

# 6. Check the Balance Control Direction

This is one of the most important steps.

When the robot starts falling forward, the wheels must move forward underneath the robot.

For example:

```text
Robot falls forward
        ↓
Wheels move forward
        ↓
Wheel axle moves underneath the center of mass
        ↓
Robot recovers
```

If the wheels move in the opposite direction and make the robot fall faster, change:

```cpp
const int8_t CONTROL_SIGN = 1;
```

to:

```cpp
const int8_t CONTROL_SIGN = -1;
```

or vice versa.

Always perform this test with the wheels off the ground first.

---

# 7. Set the Upright Angle Offset

The MPU6050 may not be mounted perfectly parallel to the robot body.

Because of this, the mechanically upright robot may not report exactly:

```text
0.0 degrees
```

Place the robot in the desired upright position and check the:

```text
Acc:
```

value.

For example, if the Serial Monitor shows:

```text
Acc:1.37
```

set:

```cpp
float ACC_ANGLE_OFFSET_DEG = 1.37f;
```

You can also change this value while the program is running by entering:

```text
o1.37
```

in the Serial Monitor.

---

# 8. PID Controller

The controller uses:

```text
P = KP × angle error
```

```text
D = -KD × gyroscope angular velocity
```

and an optional integral term.

The derivative term uses the MPU6050 gyroscope directly instead of calculating the numerical derivative of the estimated angle.

This reduces noise in the derivative feedback.

---

## Recommended Initial Tuning

It is easier to start with only proportional and derivative control.

For example:

```cpp
float KP = 12.0f;
float KI = 0.0f;
float KD = 0.5f;
```

Do not start by aggressively tuning all three gains at the same time.

Use this order:

```text
KI = 0
  ↓
Tune KP
  ↓
Tune KD
  ↓
Tune target angle
  ↓
Add a small amount of KI if necessary
```

---

# 9. Tune KP

Increase `KP` gradually.

You can change it from the Serial Monitor without uploading the program again.

For example:

```text
p12
p15
p18
p21
```

If `KP` is too low:

```text
Robot falls
→ Motors react too weakly
→ Robot cannot recover
```

If `KP` is too high:

```text
Robot rapidly oscillates forward and backward
```

Find a value strong enough to recover the robot but not excessively oscillatory.

---

# 10. Tune KD

After finding a useful `KP`, increase `KD` to reduce oscillation.

For example:

```text
d0.4
d0.6
d0.8
d1.0
```

Increasing `KD` adds damping.

If `KD` is too low, the robot tends to oscillate.

If `KD` is too high, the response may become slow or require excessive motor torque.

---

# 11. Adjust the Target Balance Angle

The true balance point may not be exactly zero degrees.

This can happen because:

- The battery is not centered
- The PCB is not centered
- The frame is asymmetric
- The MPU6050 is not mounted perfectly
- The center of mass is not exactly above the wheel axle

The current sketch starts with:

```cpp
float TARGET_ANGLE_DEG = 0.4f;
```

You can change it while running.

For example:

```text
t0.2
t0.4
t0.6
```

Change it in small increments.

A change of only:

```text
0.1 to 0.2 degrees
```

can noticeably affect whether the robot slowly moves forward or backward.

---

# 12. Tune the Motor Deadband

Geared DC motors usually do not start moving at very small PWM values because of static friction.

This program compensates for this using:

```cpp
int MOTOR_DEADBAND = 25;
```

If the controller produces small commands but the motor does not move, increase this value gradually.

For example:

```text
20
25
30
35
40
```

Use the smallest value that produces reliable motor response.

If the value is too high, the robot may become jerky around the balance point.

---

# 13. Left/Right Motor Compensation

Two inexpensive TT motors are rarely perfectly identical.

If the robot consistently turns while trying to balance, adjust:

```cpp
float LEFT_MOTOR_SCALE  = 1.00f;
float RIGHT_MOTOR_SCALE = 1.00f;
```

For example:

```cpp
float LEFT_MOTOR_SCALE  = 1.00f;
float RIGHT_MOTOR_SCALE = 0.95f;
```

can be used if the right motor is slightly stronger.

---

# 14. Integral Control

Only add integral control after the robot can already balance reasonably well using `KP` and `KD`.

For example:

```text
i0.1
i0.2
i0.4
```

The integral term can help compensate for small steady-state errors.

However, too much integral action can cause slow oscillation or instability.

The program includes integral limiting and anti-windup protection.

---

# Serial Commands

Controller parameters can be changed from the Arduino Serial Monitor.

| Command | Function | Example |
|---|---|---|
| `p[value]` | Set KP | `p18.0` |
| `i[value]` | Set KI | `i0.8` |
| `d[value]` | Set KD | `d0.75` |
| `t[value]` | Set target angle | `t0.5` |
| `o[value]` | Set accelerometer angle offset | `o1.2` |
| `q` | Immediately disarm the controller | `q` |
| `?` | Display current parameters | `?` |

Set the Serial Monitor line ending to:

```text
Newline
```

or:

```text
Both NL & CR
```

---

# Angle Estimation

The robot uses an adaptive complementary filter that combines:

- Accelerometer tilt angle
- Gyroscope angular velocity

The gyroscope provides fast short-term response, while the accelerometer corrects long-term drift.

A simple complementary filter can be expressed as:

```text
angle =
gyro prediction
+
accelerometer correction
```

However, a balancing robot accelerates continuously.

During strong horizontal acceleration, the accelerometer does not measure gravity alone.

To reduce this error, the program calculates:

```text
sqrt(ax² + ay² + az²)
```

When this value is close to:

```text
1 g
```

the accelerometer is considered reliable.

When it deviates significantly from 1 g, the controller temporarily reduces the accelerometer correction and relies more strongly on the gyroscope.

---

# Control Frequency

The balance controller runs at:

```text
500 Hz
```

which corresponds to a control period of:

```text
2 ms
```

This provides sufficiently fast feedback for a small TT-motor balancing robot while leaving enough processing time for:

- MPU6050 communication
- Angle estimation
- Floating-point calculations
- PID control
- Serial communication

---

# High-Frequency Motor PWM

The default Arduino `analogWrite()` PWM frequency is relatively low for precise motor control.

This project directly configures the ATmega328P timers to generate approximately:

```text
31.4 kHz
```

PWM on:

```text
D9
D10
D11
D3
```

This allows smoother motor control and moves the PWM switching frequency above the normal human hearing range.

Because Timer1 and Timer2 are directly configured by this program, other Arduino functions or libraries that depend on these timers may not operate normally.

---

# Automatic Arming

The motors are initially disabled.

The controller automatically arms when the robot remains within:

```cpp
ARM_ANGLE_DEG
```

for:

```cpp
ARM_HOLD_TIME_MS
```

milliseconds.

The default values are:

```cpp
const float ARM_ANGLE_DEG = 10.0f;
const unsigned long ARM_HOLD_TIME_MS = 300;
```

---

# Fall Protection

If the estimated tilt angle exceeds:

```cpp
const float FALL_ANGLE_DEG = 35.0f;
```

the controller immediately disables the motors.

The Serial Monitor will display:

```text
FALL -> DISARM
```

Return the robot to the upright position to arm it again.

---

# MPU6050 Communication Error

If communication with the MPU6050 fails repeatedly, the program automatically disables the motors.

The Serial Monitor will display:

```text
IMU ERROR
```

Check:

- MPU6050 power
- SDA connection
- SCL connection
- Ground connection
- I2C wiring length
- Loose jumper wires

---

# Recommended Tuning Procedure

For a new robot, use the following order.

```text
1. Keep the wheels off the ground
        ↓
2. Confirm MPU6050 communication
        ↓
3. Select the correct balance axis
        ↓
4. Confirm angle direction
        ↓
5. Confirm left/right motor directions
        ↓
6. Confirm controller direction
        ↓
7. Measure accelerometer angle offset
        ↓
8. Set KI = 0
        ↓
9. Tune KP
        ↓
10. Tune KD
        ↓
11. Adjust TARGET_ANGLE_DEG
        ↓
12. Adjust MOTOR_DEADBAND
        ↓
13. Adjust motor scale factors
        ↓
14. Add a small KI if needed
```

Following this order makes troubleshooting much easier because it separates wiring and sign errors from actual controller tuning problems.

---

# Important Limitation

This version of the robot does not use wheel encoders.

The controller can observe approximately:

```text
Robot angle
Robot angular velocity
```

but it cannot directly observe:

```text
Wheel position
Wheel velocity
```

Therefore, the controller can achieve good balancing performance, but maintaining exactly the same position on the floor is more difficult.

A robot with wheel encoders can additionally estimate:

```text
Body angle
Body angular velocity
Wheel position
Wheel velocity
```

and can use a cascaded controller or a state-space controller such as LQR for improved position control.

---

# Troubleshooting

## The motors make the robot fall faster

The controller direction is probably reversed.

Change:

```cpp
const int8_t CONTROL_SIGN = 1;
```

to:

```cpp
const int8_t CONTROL_SIGN = -1;
```

---

## One wheel rotates backward

Reverse the corresponding motor sign.

Example:

```cpp
const int8_t RIGHT_MOTOR_SIGN = 1;
```

---

## The displayed angle barely changes

The wrong MPU6050 axis may be selected.

Try changing:

```cpp
ROTATION_Y
```

to:

```cpp
ROTATION_X
```

or vice versa.

---

## The robot vibrates very quickly

Possible causes include:

- KP is too high
- KD is too low
- Motor deadband is too large
- Mechanical backlash
- Loose MPU6050 mounting
- Flexible frame
- Excessive vibration reaching the MPU6050

---

## The robot reacts too slowly

Possible causes include:

- KP is too low
- KD is too high
- Maximum PWM is too low
- Battery voltage is low
- Motor torque is insufficient
- Robot center of mass is too low or too far from the wheel axle

---

## The robot balances but slowly moves away

Try adjusting:

```cpp
TARGET_ANGLE_DEG
```

in approximately:

```text
0.1 degree
```

increments.

If the robot still cannot hold its position, remember that this version does not use wheel encoders.

---

# Safety

A self-balancing robot can move unexpectedly during controller tuning.

For initial testing:

- Keep the wheels off the ground
- Keep fingers away from the wheels
- Be ready to disconnect the battery
- Use the `q` Serial command to disarm the controller
- Do not test near the edge of a desk
- Do not allow the motors to stall for long periods
- Verify motor and battery voltage ratings before prolonged operation

---

# Project Structure

A simple repository structure could look like:

```text
Self-Balancing-Robot/
│
├── README.md
│
├── LICENSE
│
│
├── firmware/
│   └── Self_Balancing_Robot.ino
│
├── stl/
│   ├── upper_plate.stl
│   ├── lower_plate.stl
│   └── other_parts.stl
│
└── images/
    ├── robot.jpg
    └── wiring.jpg
```

Rename the STL files according to your actual parts.

---

# Controller Overview

The control system can be summarized as:

```text
                MPU6050
                   │
       ┌───────────┴───────────┐
       │                       │
 Accelerometer             Gyroscope
       │                       │
       └──── Adaptive ─────────┘
          Complementary
             Filter
                │
                ▼
        Estimated Angle
                │
                ▼
        Balance Controller
          P + I + Gyro D
                │
                ▼
       Deadband Compensation
                │
                ▼
          DRV8833 Driver
                │
                ▼
            TT Motors
```

---

# License

Choose a license that matches how you want other people to use your code and STL files.

For example, the firmware and the mechanical files can either use the same license or separate licenses depending on your preference.
