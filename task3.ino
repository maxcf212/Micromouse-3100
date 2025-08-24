#include <Wire.h>
#include <MPU6050.h>
#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

using mtrn3100::Motor;

// === PIN DEFINITIONS ===
#define LEFT_PWM 11
#define LEFT_DIR 12
#define RIGHT_PWM 9
#define RIGHT_DIR 10
#define LEFT_ENC_A 2
#define LEFT_ENC_B 7
#define RIGHT_ENC_A 3
#define RIGHT_ENC_B 8

// === ENCODER SETTINGS ===
const float WHEEL_DIAMETER = 0.032;  // meters
const int TICKS_PER_REV = 20;
const float TICK_DISTANCE = (PI * WHEEL_DIAMETER) / TICKS_PER_REV; // meters
const float TARGET_DISTANCE = 2.5; // meters 

// === ENCODER COUNTS ===
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// === MOTOR AND IMU OBJECTS ===
MPU6050 imu;
Motor leftMotor(LEFT_PWM, LEFT_DIR);
Motor rightMotor(RIGHT_PWM, RIGHT_DIR);

// === CONTROL SETTINGS ===
const float Kp = 2.0;
const float Ki = 0.2;
const int maxSpeed = 80;
const int minSpeed = 20;

// === TURN SETTINGS ===
const float TURN_ANGLE = -90.0;  // degrees
float initialYaw = 0.0;
float targetYaw = 0.0;
float integralError = 0.0;
bool turning = false;

// === FORWARD SETTINGS ===
const int forwardSpeed = 150;
bool movingForward = false;

// === TIME TRACKING ===
unsigned long lastLoopTime = 0;

// === SEQUENCE CONTROL ===
const char* motionSequence = "lfrfflfr";  // Add your sequence here
int sequenceIndex = 0;

// === FUNCTION DECLARATIONS ===
void processNextCommand();
float getYaw();
float angleDiff(float target, float current);
void leftEncoderISR();
void rightEncoderISR();

void setup() {
  delay(2000);
  Serial.begin(9600);
  Wire.begin();
  imu.initialize();
  delay(2000);

  // Encoder pin setup
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  initialYaw = getYaw();
  lastLoopTime = millis();
  processNextCommand();
}

void loop() {
  // === TURNING ===
  if (turning) {
    float currentYaw = getYaw();
    float error = angleDiff(targetYaw, currentYaw);

    const float tolerance = 2.0;
    if (abs(error) < tolerance) {
      turning = false;
      leftMotor.setPWM(0);
      rightMotor.setPWM(0);
      Serial.println("Turn complete.");
      delay(500);
      processNextCommand();
      return;
    }

    unsigned long now = millis();
    float dt = (now - lastLoopTime) / 1000.0;
    lastLoopTime = now;

    integralError += error * dt;
    integralError = constrain(integralError, -100.0, 100.0);

    float control = Kp * error + Ki * integralError;
    int speed = constrain(abs(control), minSpeed, maxSpeed);
    int direction = (control > 0) ? 1 : -1;

    leftMotor.setPWM(speed * direction);
    rightMotor.setPWM(speed * direction);

    Serial.print("Yaw: "); Serial.print(currentYaw);
    Serial.print(" | Target: "); Serial.print(targetYaw);
    Serial.print(" | Error: "); Serial.print(error);
    Serial.print(" | Speed: "); Serial.println(speed);
  }

  // === FORWARD ===
  else if (movingForward) {
    float leftDistance = leftEncoderTicks * TICK_DISTANCE;
    float rightDistance = rightEncoderTicks * TICK_DISTANCE;
    float avgDistance = (abs(leftDistance) + abs(rightDistance)) / 2.0;

    if (avgDistance >= TARGET_DISTANCE) {
      movingForward = false;
      leftMotor.setPWM(0);
      rightMotor.setPWM(0);
      Serial.println("Forward motion complete.");
      delay(500);
      processNextCommand();
    }
  }
}

// === Command Dispatcher ===
void processNextCommand() {
  char cmd = motionSequence[sequenceIndex];
  if (cmd == '\0') {
    Serial.println("Sequence complete.");
    leftMotor.setPWM(0);
    rightMotor.setPWM(0);
    return;
  }

  Serial.print("Executing command: ");
  Serial.println(cmd);

  if (cmd == 'l' || cmd == 'r') {
    int dir = (cmd == 'r') ? 1 : -1;
    initialYaw = getYaw();
    targetYaw = fmod(initialYaw + dir * TURN_ANGLE + 360.0, 360.0);
    integralError = 0.0;
    turning = true;
  } 
  else if (cmd == 'f') {
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    leftMotor.setPWM(-forwardSpeed);
    rightMotor.setPWM(forwardSpeed);
    movingForward = true;
  }

  sequenceIndex++;
}

// === Get Current Yaw by Integrating Gyro ===
float getYaw() {
  static float yaw = 0.0;
  static unsigned long lastTime = millis();

  int16_t gx, gy, gz;
  imu.getRotation(&gx, &gy, &gz);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float yawRate = gz / 131.0;
  yaw += yawRate * dt;

  return fmod(yaw + 360.0, 360.0);
}

// === Calculate Shortest Angle Between Two Yaws ===
float angleDiff(float target, float current) {
  float diff = target - current;
  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return diff;
}

// === Encoder Interrupt Handlers ===
void leftEncoderISR() {
  leftEncoderTicks++;
}

void rightEncoderISR() {
  rightEncoderTicks++;
}
