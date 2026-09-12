# Basic Self-Balancing Robot Using TT Motors

A compact two-wheel self-balancing robot built with an Arduino Nano, MPU6050 IMU, DRV8833 motor driver, two TT geared motors, and a 7.2 V battery.

The controller runs at 500 Hz and combines an adaptive complementary filter, PID balance control, gyroscope-based derivative feedback, motor deadband compensation, and approximately 31.4 kHz PWM.

## Repository Contents

- [`code/self_balancing_robot/self_balancing_robot.ino`](code/self_balancing_robot/self_balancing_robot.ino) — Arduino Nano firmware
- [`code/README.md`](code/README.md) — complete setup, calibration, tuning, and troubleshooting guide
- [`stl/`](stl/) — two STL files for the 3D-printed parts
- [`stl/README.md`](stl/README.md) — STL file list and printing notes

## Main Hardware

| Component | Quantity |
| --- | ---: |
| Arduino Nano (ATmega328P) | 1 |
| MPU6050 IMU | 1 |
| DRV8833 dual motor driver | 1 |
| TT geared DC motor | 2 |
| Wheel | 2 |
| 7.2 V battery | 1 |
| 3D-printed parts | 1 set |

Screws, spacers, wiring, and a power switch may also be required for your build.

## Wiring

### MPU6050 to Arduino Nano

| MPU6050 | Arduino Nano |
| --- | --- |
| SDA | A4 |
| SCL | A5 |
| GND | GND |
| VCC | Follow the voltage specification of your breakout board |

### DRV8833 to Arduino Nano

| DRV8833 | Arduino Nano |
| --- | ---: |
| AIN1 | D9 |
| AIN2 | D10 |
| BIN1 | D11 |
| BIN2 | D3 |
| nSLEEP / STBY, if used | D7 |

Connect one motor to Motor A and the other to Motor B. The Arduino, MPU6050, DRV8833, and battery system must share a common ground.

> MPU6050 and DRV8833 breakout boards are not all wired or labeled identically. Check the voltage limits and pin labels of the exact boards you are using before applying power.

## Quick Start

1. Open [`self_balancing_robot.ino`](code/self_balancing_robot/self_balancing_robot.ino) in the Arduino IDE.
2. Select **Arduino Nano** and the correct processor/bootloader for your board.
3. Upload the sketch. No external Arduino library is required; it uses the built-in `Wire` library.
4. Keep the robot completely still during startup gyroscope calibration.
5. Open the Serial Monitor at **115200 baud** with **Newline** enabled.
6. Keep the wheels off the ground while checking the IMU axis, angle direction, motor directions, and controller direction.
7. Follow the [complete calibration and PID tuning guide](code/README.md) before testing on the floor.

## Current Sketch Defaults

These values are specific to the prototype and will usually need adjustment for another build.

| Setting | Default |
| --- | ---: |
| Balance axis | `ROTATION_X` |
| Control frequency | 500 Hz |
| Motor PWM | approximately 31.4 kHz |
| `KP` / `KI` / `KD` | `50.0` / `0.3` / `0.5` |
| Target angle | `0.4°` |
| Accelerometer angle offset | `9.8°` |
| Motor deadband | `25` |
| Maximum PWM | `210` |

Controller values can be adjusted from the Serial Monitor without uploading the sketch again. See the [serial command reference](code/README.md#serial-commands).

## Safety

- Perform the first tests with both wheels off the ground.
- Keep fingers, hair, and loose clothing away from the wheels.
- Be ready to disconnect the battery.
- Use the `q` Serial command to disarm the motors.
- Do not test near the edge of a desk.
- Do not allow the motors to stall for long periods.
- Verify motor, driver, sensor, and battery voltage ratings before prolonged operation.

This robot can move unexpectedly while the controller is being tuned. Read the [full setup guide](code/README.md#before-running-the-robot) before powering the motors.

## Contact

Instagram: [@takeru_robot](https://www.instagram.com/takeru_robot/)
