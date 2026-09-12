#include <Wire.h>

// ============================================================
// Self-Balancing Robot
// Arduino Nano + MPU6050 + DRV8833 + TT Motors
//
// Control frequency : 500 Hz
// I2C clock         : 400 kHz
// Motor PWM         : ~31.4 kHz
//
// No external Arduino library is required.
// ============================================================


// ============================================================
// 1. USER CONFIGURATION
// ============================================================

// ------------------------------------------------------------
// MPU6050 configuration
// ------------------------------------------------------------

const uint8_t MPU_ADDR = 0x68;


// Select the axis around which the robot rotates.
//
// ROTATION_X:
//   Accelerometer angle = atan2(ay, az)
//   Gyroscope rate      = gx
//
// ROTATION_Y:
//   Accelerometer angle = atan2(-ax, az)
//   Gyroscope rate      = gy
//
enum BalanceAxis {
  ROTATION_X,
  ROTATION_Y
};

const BalanceAxis BALANCE_AXIS = ROTATION_X;


// Change this to -1 if the measured angle direction is reversed.
const int8_t IMU_SIGN = 1;


// ------------------------------------------------------------
// MPU6050 offsets
// ------------------------------------------------------------
//
// These values can be replaced with calibrated sensor offsets.
// The gyro bias is also measured automatically at startup.
//

const float ACC_X_OFFSET = -5116;
const float ACC_Y_OFFSET = -157;
const float ACC_Z_OFFSET = 904;

const float GYRO_X_OFFSET = -318;
const float GYRO_Y_OFFSET = 286;
const float GYRO_Z_OFFSET = -361;


// Accelerometer angle offset.
//
// Place the robot at the desired upright position and read the
// "Acc:" value from the Serial Monitor.
//
// Example:
// If the upright position gives:
//
//   Acc: 1.35
//
// set:
//
//   ACC_ANGLE_OFFSET_DEG = 1.35
//
float ACC_ANGLE_OFFSET_DEG = 9.8;


// Automatically measure gyroscope bias during startup.
//
// IMPORTANT:
// Keep the robot completely still for a few seconds after power-on.
//
const bool AUTO_GYRO_CALIBRATION = true;


// ------------------------------------------------------------
// Motor pins
// ------------------------------------------------------------
//
// These pins are selected so that Timer1 and Timer2 can generate
// approximately 31.4 kHz PWM.
//
// Timer1:
//   D9  = OC1A
//   D10 = OC1B
//
// Timer2:
//   D11 = OC2A
//   D3  = OC2B
//

const uint8_t LEFT_IN1  = 9;
const uint8_t LEFT_IN2  = 10;
const uint8_t STBY_PIN  = 7;
const uint8_t RIGHT_IN1 = 11;
const uint8_t RIGHT_IN2 = 3;


// ------------------------------------------------------------
// Motor direction
// ------------------------------------------------------------
//
// Positive motor commands should make both wheels move in the
// same physical direction.
//
// If one wheel rotates in the wrong direction, change its sign.
//
const int8_t LEFT_MOTOR_SIGN  = 1;
const int8_t RIGHT_MOTOR_SIGN = -1;


// ------------------------------------------------------------
// Controller output direction
// ------------------------------------------------------------
//
// When the robot falls forward, the wheels must move forward
// underneath the robot.
//
// If the controller makes the robot fall faster instead of
// recovering, change this value from 1 to -1 or vice versa.
//
const int8_t CONTROL_SIGN = -1;


// ------------------------------------------------------------
// Left/right motor compensation
// ------------------------------------------------------------
//
// Use these scale factors if the two motors have slightly
// different speeds or torque.
//
float LEFT_MOTOR_SCALE  = 1.00f;
float RIGHT_MOTOR_SCALE = 1.00f;


// ------------------------------------------------------------
// Motor deadband compensation
// ------------------------------------------------------------
//
// Small PWM values may not produce enough torque to move a geared
// DC motor because of friction.
//
// Increase this value until the motors respond reliably to small
// commands.
//
// Typical values may be around 20 to 50, but every motor is different.
//
int MOTOR_DEADBAND = 25;


// Maximum PWM output.
//
// With motors rated below the battery voltage, limiting the
// maximum PWM can reduce motor stress.
//
// Range: 0 to 255
//
int MAX_PWM = 210;


// Commands smaller than this value are treated as zero.
const float PWM_ZERO_ZONE = 2.0f;


// ============================================================
// 2. BALANCE CONTROLLER PARAMETERS
// ============================================================
//
// These values must be tuned for the actual robot.
//
// Recommended tuning order:
//
//   1. Set KI = 0
//   2. Tune KP
//   3. Tune KD
//   4. Adjust TARGET_ANGLE_DEG
//   5. Add a small KI if necessary
//

float KP = 50.0f;
float KI = 0.3f;
float KD = 0.5f;


// Desired balance angle.
//
// The mechanically vertical position is not always the true
// balance point because the center of mass may not be exactly
// above the wheel axle.
//
// Adjust this value if the robot slowly drives forward or backward
// while balancing.
//
float TARGET_ANGLE_DEG = 0.4f;


// Maximum contribution from the integral term.
const float I_MAX = 40.0f;


// Only integrate the error while the robot is reasonably close
// to the upright position.
const float I_ACTIVE_ANGLE = 8.0f;


// ============================================================
// 3. SAFETY PARAMETERS
// ============================================================

// The robot can arm only when its angle is within this range.
const float ARM_ANGLE_DEG = 10.0f;


// The robot must remain upright for this long before the motors
// are automatically enabled.
const unsigned long ARM_HOLD_TIME_MS = 300;


// If the robot exceeds this angle, the controller assumes that
// the robot has fallen and immediately disables the motors.
const float FALL_ANGLE_DEG = 35.0f;


// ============================================================
// 4. CONTROL LOOP PARAMETERS
// ============================================================

const uint32_t CONTROL_FREQ = 500;

const uint32_t CONTROL_PERIOD_US =
    1000000UL / CONTROL_FREQ;


// Complementary filter time constant.
//
// Larger values:
//   Trust the gyroscope more.
//
// Smaller values:
//   Correct the angle more aggressively using the accelerometer.
//
const float FILTER_TAU = 0.40f;


// ============================================================
// Runtime variables
// ============================================================

float ax, ay, az;
float gx, gy, gz;

float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

float accAngle = 0;
float angle = 0;
float gyroRate = 0;

float integralOutput = 0;
float motorOutput = 0;

bool armed = false;

unsigned long uprightStartTime = 0;

uint32_t nextControlTime = 0;

uint8_t imuErrorCount = 0;


// ============================================================
// MPU6050 low-level functions
// ============================================================

// Write one byte to an MPU6050 register.
void writeMPU(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}


// Read one byte from an MPU6050 register.
uint8_t readMPURegister(uint8_t reg)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, (uint8_t)1);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0xFF;
}


// ============================================================
// MPU6050 initialization
// ============================================================

void setupMPU()
{
  delay(100);

  // Wake the MPU6050 and use the X-axis gyroscope PLL
  // as the clock reference.
  writeMPU(0x6B, 0x01);

  delay(100);

  // Sample rate:
  //
  // With the digital low-pass filter enabled,
  // the internal gyroscope output rate is 1 kHz.
  //
  // Sample Rate = 1 kHz / (1 + SMPLRT_DIV)
  //
  // SMPLRT_DIV = 0 -> 1 kHz
  writeMPU(0x19, 0x00);

  // Digital low-pass filter configuration.
  //
  // DLPF_CFG = 3
  //
  // Approximate bandwidth:
  //   Gyroscope     : 42 Hz
  //   Accelerometer : 44 Hz
  //
  // This helps reduce high-frequency vibration from the motors.
  writeMPU(0x1A, 0x03);

  // Gyroscope range: ±500 deg/s
  //
  // Sensitivity:
  //   65.5 LSB/(deg/s)
  writeMPU(0x1B, 0x08);

  // Accelerometer range: ±4 g
  //
  // Sensitivity:
  //   8192 LSB/g
  writeMPU(0x1C, 0x08);

  delay(100);
}


// ============================================================
// Read MPU6050 sensor data
// ============================================================

bool readMPU()
{
  // Start reading from ACCEL_XOUT_H.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received =
      Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  if (received != 14) {
    return false;
  }


  // Read raw accelerometer data.
  int16_t rawAX =
      ((int16_t)Wire.read() << 8) | Wire.read();

  int16_t rawAY =
      ((int16_t)Wire.read() << 8) | Wire.read();

  int16_t rawAZ =
      ((int16_t)Wire.read() << 8) | Wire.read();


  // Temperature data is not used.
  Wire.read();
  Wire.read();


  // Read raw gyroscope data.
  int16_t rawGX =
      ((int16_t)Wire.read() << 8) | Wire.read();

  int16_t rawGY =
      ((int16_t)Wire.read() << 8) | Wire.read();

  int16_t rawGZ =
      ((int16_t)Wire.read() << 8) | Wire.read();


  // Convert accelerometer values to g.
  ax = rawAX / 8192.0f - ACC_X_OFFSET;
  ay = rawAY / 8192.0f - ACC_Y_OFFSET;
  az = rawAZ / 8192.0f - ACC_Z_OFFSET;


  // Convert gyroscope values to degrees per second.
  gx = rawGX / 65.5f - GYRO_X_OFFSET - gyroBiasX;
  gy = rawGY / 65.5f - GYRO_Y_OFFSET - gyroBiasY;
  gz = rawGZ / 65.5f - GYRO_Z_OFFSET - gyroBiasZ;


  return true;
}


// ============================================================
// Automatic gyroscope calibration
// ============================================================

void calibrateGyro()
{
  Serial.println(F("Gyro calibration..."));
  Serial.println(F("DO NOT MOVE ROBOT"));

  const int samples = 1000;

  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;


  // Average many gyroscope samples while the robot is stationary.
  for (int i = 0; i < samples; i++)
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true);


    int16_t rawGX =
        ((int16_t)Wire.read() << 8) | Wire.read();

    int16_t rawGY =
        ((int16_t)Wire.read() << 8) | Wire.read();

    int16_t rawGZ =
        ((int16_t)Wire.read() << 8) | Wire.read();


    sumX += rawGX / 65.5f - GYRO_X_OFFSET;
    sumY += rawGY / 65.5f - GYRO_Y_OFFSET;
    sumZ += rawGZ / 65.5f - GYRO_Z_OFFSET;

    delay(2);
  }


  gyroBiasX = sumX / samples;
  gyroBiasY = sumY / samples;
  gyroBiasZ = sumZ / samples;


  Serial.print(F("Gyro bias X = "));
  Serial.println(gyroBiasX, 5);

  Serial.print(F("Gyro bias Y = "));
  Serial.println(gyroBiasY, 5);

  Serial.print(F("Gyro bias Z = "));
  Serial.println(gyroBiasZ, 5);

  Serial.println(F("Calibration done."));
}


// ============================================================
// Calculate tilt angle from accelerometer
// ============================================================

float calculateAccAngle()
{
  float a;


  if (BALANCE_AXIS == ROTATION_X)
  {
    a = atan2f(ay, -az) * 180.0f / PI;
  }
  else
  {
    a = atan2f(-ax, az) * 180.0f / PI;
  }


  // Correct the IMU orientation if required.
  a *= IMU_SIGN;


  // Shift the measured angle so that the desired upright position
  // becomes approximately zero degrees.
  a -= ACC_ANGLE_OFFSET_DEG;


  return a;
}


// ============================================================
// Get angular velocity from gyroscope
// ============================================================

float getGyroRate()
{
  float rate;


  if (BALANCE_AXIS == ROTATION_X) {
    rate = gx;
  }
  else {
    rate = gy;
  }


  return rate * IMU_SIGN;
}


// ============================================================
// Adaptive complementary filter
// ============================================================

void updateAngleFilter(float dt)
{
  accAngle = calculateAccAngle();

  gyroRate = getGyroRate();


  // Integrate the gyroscope angular velocity.
  angle += gyroRate * dt;


  // ----------------------------------------------------------
  // Accelerometer reliability estimation
  // ----------------------------------------------------------
  //
  // A moving balancing robot experiences both gravity and
  // translational acceleration.
  //
  // Therefore, the accelerometer angle becomes less reliable
  // during strong acceleration.
  //
  // The magnitude of the measured acceleration should be close
  // to 1 g when gravity is the dominant acceleration.
  //

  float norm =
      sqrtf(ax * ax +
            ay * ay +
            az * az);


  float deviation =
      fabsf(norm - 1.0f);


  // Reduce accelerometer influence when the measured acceleration
  // magnitude deviates significantly from 1 g.
  float accelTrust =
      1.0f - deviation / 0.25f;


  accelTrust =
      constrain(accelTrust, 0.0f, 1.0f);


  // Complementary filter correction coefficient.
  float correction =
      dt / (FILTER_TAU + dt);


  correction *= accelTrust;


  // Correct the integrated gyroscope angle using the
  // accelerometer angle.
  angle +=
      correction *
      (accAngle - angle);
}


// ============================================================
// Configure high-frequency motor PWM
// ============================================================

void setupMotorPWM()
{
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);


  // ----------------------------------------------------------
  // Timer1
  //
  // D9  -> OC1A
  // D10 -> OC1B
  //
  // 8-bit phase-correct PWM
  // Prescaler = 1
  //
  // PWM frequency:
  //
  //   16 MHz / 510 ≈ 31.37 kHz
  // ----------------------------------------------------------

  TCCR1A =
      _BV(COM1A1) |
      _BV(COM1B1) |
      _BV(WGM10);

  TCCR1B =
      _BV(CS10);


  // ----------------------------------------------------------
  // Timer2
  //
  // D11 -> OC2A
  // D3  -> OC2B
  //
  // 8-bit phase-correct PWM
  // Prescaler = 1
  //
  // PWM frequency:
  //
  //   approximately 31.37 kHz
  // ----------------------------------------------------------

  TCCR2A =
      _BV(COM2A1) |
      _BV(COM2B1) |
      _BV(WGM20);

  TCCR2B =
      _BV(CS20);


  // Start with all motor outputs disabled.
  OCR1A = 0;
  OCR1B = 0;

  OCR2A = 0;
  OCR2B = 0;
}


// ============================================================
// Set left motor output
// ============================================================

void setLeftPWM(int pwm)
{
  pwm *= LEFT_MOTOR_SIGN;

  pwm = constrain(pwm, -255, 255);


  if (pwm > 0)
  {
    OCR1A = pwm;
    OCR1B = 0;
  }
  else if (pwm < 0)
  {
    OCR1A = 0;
    OCR1B = -pwm;
  }
  else
  {
    OCR1A = 0;
    OCR1B = 0;
  }
}


// ============================================================
// Set right motor output
// ============================================================

void setRightPWM(int pwm)
{
  pwm *= RIGHT_MOTOR_SIGN;

  pwm = constrain(pwm, -255, 255);


  if (pwm > 0)
  {
    OCR2A = pwm;   // D11
    OCR2B = 0;     // D3
  }
  else if (pwm < 0)
  {
    OCR2A = 0;
    OCR2B = -pwm;
  }
  else
  {
    OCR2A = 0;
    OCR2B = 0;
  }
}


// ============================================================
// Compensate for motor deadband
// ============================================================

int compensateMotor(float command)
{
  float magnitude =
      fabsf(command);


  // Do not move the motor for extremely small commands.
  if (magnitude < PWM_ZERO_ZONE) {
    return 0;
  }


  magnitude =
      constrain(magnitude,
                0.0f,
                (float)MAX_PWM);


  // Map the controller output to a PWM range that starts above
  // the motor deadband.
  float pwm =
      MOTOR_DEADBAND +
      magnitude *
      ((float)MAX_PWM - MOTOR_DEADBAND) /
      MAX_PWM;


  pwm =
      constrain(pwm,
                0.0f,
                (float)MAX_PWM);


  if (command < 0) {
    pwm = -pwm;
  }


  return (int)pwm;
}


// ============================================================
// Drive both motors
// ============================================================

void driveMotors(float command)
{
  // Apply the global controller direction.
  command *= CONTROL_SIGN;


  // Apply individual motor compensation.
  float left =
      command *
      LEFT_MOTOR_SCALE;

  float right =
      command *
      RIGHT_MOTOR_SCALE;


  left =
      constrain(left,
                -(float)MAX_PWM,
                (float)MAX_PWM);

  right =
      constrain(right,
                -(float)MAX_PWM,
                (float)MAX_PWM);


  int leftPWM =
      compensateMotor(left);

  int rightPWM =
      compensateMotor(right);


  setLeftPWM(leftPWM);
  setRightPWM(rightPWM);
}


// ============================================================
// Immediately stop both motors
// ============================================================

void stopMotors()
{
  OCR1A = 0;
  OCR1B = 0;

  OCR2A = 0;
  OCR2B = 0;
}


// ============================================================
// Balance controller
// ============================================================

float calculateController(float dt)
{
  // Angle error.
  float error =
      TARGET_ANGLE_DEG - angle;


  // ----------------------------------------------------------
  // Proportional term
  // ----------------------------------------------------------

  float P =
      KP * error;


  // ----------------------------------------------------------
  // Derivative term
  // ----------------------------------------------------------
  //
  // The derivative term uses the gyroscope angular velocity
  // directly instead of numerically differentiating the angle.
  //
  // This reduces noise and avoids unnecessary differentiation.
  //

  float D =
      -KD * gyroRate;


  // ----------------------------------------------------------
  // Integral term
  // ----------------------------------------------------------

  float nextIntegral =
      integralOutput;


  if (fabsf(error) < I_ACTIVE_ANGLE)
  {
    nextIntegral +=
        KI * error * dt;

    nextIntegral =
        constrain(nextIntegral,
                  -I_MAX,
                  I_MAX);
  }
  else
  {
    // Slowly discharge the integrator while far from upright.
    nextIntegral *= 0.995f;
  }


  // ----------------------------------------------------------
  // Anti-windup
  // ----------------------------------------------------------
  //
  // Prevent the integral term from continuing to grow when the
  // controller output is saturated, unless the integral action
  // is helping the output return from saturation.
  //

  float testOutput =
      P +
      nextIntegral +
      D;


  bool notSaturated =
      fabsf(testOutput) < MAX_PWM;


  bool correctingPositiveSaturation =
      testOutput > MAX_PWM &&
      error < 0;


  bool correctingNegativeSaturation =
      testOutput < -MAX_PWM &&
      error > 0;


  if (notSaturated ||
      correctingPositiveSaturation ||
      correctingNegativeSaturation)
  {
    integralOutput =
        nextIntegral;
  }


  // Final controller output.
  float output =
      P +
      integralOutput +
      D;


  output =
      constrain(output,
                -(float)MAX_PWM,
                (float)MAX_PWM);


  return output;
}


// ============================================================
// Automatic arming and fall detection
// ============================================================

void updateSafety()
{
  if (armed)
  {
    // Disable the motors immediately if the robot falls too far.
    if (fabsf(angle) > FALL_ANGLE_DEG)
    {
      armed = false;

      integralOutput = 0;

      motorOutput = 0;

      stopMotors();

      Serial.println(F("FALL -> DISARM"));
    }

    return;
  }


  // ----------------------------------------------------------
  // Wait until the robot is held near the upright position.
  // ----------------------------------------------------------

  if (fabsf(angle) < ARM_ANGLE_DEG)
  {
    if (uprightStartTime == 0)
    {
      uprightStartTime =
          millis();
    }


    // Arm the controller after the robot has remained upright
    // for the required amount of time.
    if (millis() -
        uprightStartTime >
        ARM_HOLD_TIME_MS)
    {
      armed = true;

      integralOutput = 0;

      Serial.println(F("ARMED"));
    }
  }
  else
  {
    uprightStartTime = 0;
  }
}


// ============================================================
// Serial parameter tuning
// ============================================================
//
// Commands can be sent from the Arduino Serial Monitor.
//
// Examples:
//
//   p18.0   -> KP = 18.0
//   i0.8    -> KI = 0.8
//   d0.75   -> KD = 0.75
//   t0.5    -> target angle = 0.5 deg
//   o1.2    -> accelerometer angle offset = 1.2 deg
//   q       -> disarm controller
//   ?       -> print current parameters
//

void processSerialCommand()
{
  static char buffer[32];
  static uint8_t index = 0;


  while (Serial.available())
  {
    char c =
        Serial.read();


    if (c == '\n' ||
        c == '\r')
    {
      if (index == 0) {
        continue;
      }


      buffer[index] = '\0';

      char command =
          buffer[0];

      float value =
          atof(buffer + 1);


      switch (command)
      {
        case 'p':

          KP = value;

          Serial.print(F("KP="));
          Serial.println(KP);

          break;


        case 'i':

          KI = value;

          Serial.print(F("KI="));
          Serial.println(KI);

          break;


        case 'd':

          KD = value;

          Serial.print(F("KD="));
          Serial.println(KD);

          break;


        case 't':

          TARGET_ANGLE_DEG = value;

          Serial.print(F("Target angle="));
          Serial.println(TARGET_ANGLE_DEG);

          break;


        case 'o':

          ACC_ANGLE_OFFSET_DEG = value;

          Serial.print(F("Angle offset="));
          Serial.println(ACC_ANGLE_OFFSET_DEG);

          break;


        case 'q':

          armed = false;

          integralOutput = 0;

          stopMotors();

          Serial.println(F("DISARM"));

          break;


        case '?':

          Serial.print(F("KP="));
          Serial.println(KP);

          Serial.print(F("KI="));
          Serial.println(KI);

          Serial.print(F("KD="));
          Serial.println(KD);

          Serial.print(F("Target="));
          Serial.println(TARGET_ANGLE_DEG);

          Serial.print(F("Offset="));
          Serial.println(ACC_ANGLE_OFFSET_DEG);

          break;
      }


      index = 0;
    }
    else
    {
      if (index < sizeof(buffer) - 1)
      {
        buffer[index++] = c;
      }
    }
  }
}


// ============================================================
// Serial telemetry
// ============================================================

void sendTelemetry()
{
  static unsigned long previous = 0;


  // Send telemetry at approximately 25 Hz.
  if (millis() - previous < 40) {
    return;
  }

  previous = millis();


  Serial.print(F("A:"));
  Serial.print(angle, 3);

  Serial.print(F(" Acc:"));
  Serial.print(accAngle, 3);

  Serial.print(F(" G:"));
  Serial.print(gyroRate, 2);

  Serial.print(F(" U:"));
  Serial.print(motorOutput, 1);

  Serial.print(F(" Arm:"));
  Serial.println(armed);
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(500);

  digitalWrite(STBY_PIN, HIGH);


  Serial.println();
  Serial.println(F("=============================="));
  Serial.println(F("Self Balancing Robot"));
  Serial.println(F("=============================="));


  // ----------------------------------------------------------
  // Initialize I2C
  // ----------------------------------------------------------

  Wire.begin();

  Wire.setClock(400000UL);


  // ----------------------------------------------------------
  // Initialize MPU6050
  // ----------------------------------------------------------

  setupMPU();


  uint8_t whoAmI =
      readMPURegister(0x75);


  Serial.print(F("MPU6050 WHO_AM_I = 0x"));
  Serial.println(whoAmI, HEX);


  // ----------------------------------------------------------
  // Measure gyroscope bias
  // ----------------------------------------------------------

  if (AUTO_GYRO_CALIBRATION)
  {
    calibrateGyro();
  }


  // ----------------------------------------------------------
  // Initialize the estimated angle using the accelerometer
  // ----------------------------------------------------------

  if (readMPU())
  {
    accAngle =
        calculateAccAngle();

    angle =
        accAngle;
  }


  // ----------------------------------------------------------
  // Configure motor PWM
  // ----------------------------------------------------------

  setupMotorPWM();

  stopMotors();


  Serial.println();
  Serial.println(F("Ready."));
  Serial.println(F("Hold robot upright to ARM."));

  Serial.println();
  Serial.println(F("Serial commands:"));
  Serial.println(F("p18.0 -> KP"));
  Serial.println(F("i0.8  -> KI"));
  Serial.println(F("d0.75 -> KD"));
  Serial.println(F("t0.5  -> target angle"));
  Serial.println(F("o1.2  -> angle offset"));
  Serial.println(F("q      -> DISARM"));
  Serial.println(F("?      -> parameters"));


  nextControlTime =
      micros() + CONTROL_PERIOD_US;
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // Process parameter updates from the Serial Monitor.
  processSerialCommand();


  uint32_t now =
      micros();


  // Wait until the next control cycle.
  if ((int32_t)
      (now - nextControlTime) < 0)
  {
    return;
  }


  nextControlTime +=
      CONTROL_PERIOD_US;


  // If execution was delayed by several control periods,
  // resynchronize the scheduler instead of trying to catch up.
  if ((int32_t)
      (now - nextControlTime) >
      (int32_t)(CONTROL_PERIOD_US * 3))
  {
    nextControlTime =
        now + CONTROL_PERIOD_US;
  }


  const float dt =
      1.0f / CONTROL_FREQ;


  // ----------------------------------------------------------
  // Read IMU
  // ----------------------------------------------------------

  if (!readMPU())
  {
    imuErrorCount++;


    // Disable the motors after repeated communication failures.
    if (imuErrorCount > 5)
    {
      armed = false;

      stopMotors();

      Serial.println(F("IMU ERROR"));
    }

    return;
  }


  imuErrorCount = 0;


  // ----------------------------------------------------------
  // Estimate robot angle
  // ----------------------------------------------------------

  updateAngleFilter(dt);


  // ----------------------------------------------------------
  // Check safety conditions
  // ----------------------------------------------------------

  updateSafety();


  // ----------------------------------------------------------
  // Run balance controller
  // ----------------------------------------------------------

  if (armed)
  {
    motorOutput =
        calculateController(dt);

    driveMotors(motorOutput);
  }
  else
  {
    motorOutput = 0;

    integralOutput = 0;

    stopMotors();
  }


  // ----------------------------------------------------------
  // Send diagnostic information
  // ----------------------------------------------------------

  sendTelemetry();
}