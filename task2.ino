#include <Wire.h>
#include <MPU6050.h>
#include "Motor.hpp"

using mtrn3100::Motor;

// === PIN DEFINITIONS ===
#define LEFT_PWM 11
#define LEFT_DIR 12
#define RIGHT_PWM 9
#define RIGHT_DIR 10

// === OBJECTS ===
MPU6050 imu;
Motor leftMotor(LEFT_PWM, LEFT_DIR);
Motor rightMotor(RIGHT_PWM, RIGHT_DIR);

// === TURN SETTINGS ===
const int TURN_DIRECTION = 1;        // 1 = CW, -1 = CCW
const float TURN_ANGLE = -90.0;       // degrees
float initialYaw = 0.0;
float targetYaw = 0.0;
bool turning = true;

// === CONTROL SETTINGS ===
const float Kp = 2.0;
const float Ki = 0.02;                // Tune this value!
const int maxSpeed = 70;
const int minSpeed = 20;

// === INTEGRAL STATE ===
float integralError = 0.0;
unsigned long lastLoopTime = 0;

// === FUNCTIONS ===
float getYaw();
float angleDiff(float target, float current);

void setup() {
  delay(2000);
  Serial.begin(9600);
  Wire.begin();
  imu.initialize();

  delay(2000);  // allow sensor to settle
  initialYaw = getYaw();
  targetYaw = fmod(initialYaw + TURN_DIRECTION * TURN_ANGLE + 360.0, 360.0);

  lastLoopTime = millis();
}

void loop() {
  float currentYaw = getYaw();
  float error = angleDiff(targetYaw, currentYaw);

  unsigned long now = millis();
  float dt = (now - lastLoopTime) / 1000.0;
  lastLoopTime = now;

  // === Integrate error ===
  integralError += error * dt;

  // Optional: prevent integral wind-up
  integralError = constrain(integralError, -100.0, 100.0);

  // === PI Control ===
  float control = Kp * error + Ki * integralError;
  int speed = constrain(abs(control), minSpeed, maxSpeed);
  int direction = (control > 0) ? 1 : -1;

  leftMotor.setPWM(speed * direction);
  rightMotor.setPWM(speed * direction);

  // === Debug Output ===
  Serial.print("Yaw: "); Serial.print(currentYaw);
  Serial.print(" | Target: "); Serial.print(targetYaw);
  Serial.print(" | Error: "); Serial.print(error);
  Serial.print(" | Integral: "); Serial.print(integralError);
  Serial.print(" | Speed: "); Serial.println(speed);
}

float getYaw() {
  static float yaw = 0.0;
  static unsigned long lastTime = millis();

  int16_t gx, gy, gz;
  imu.getRotation(&gx, &gy, &gz);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float yawRate = gz / 131.0;  // deg/sec
  yaw += yawRate * dt;

  // keep yaw between 0–360
  return fmod(yaw + 360.0, 360.0);
}

float angleDiff(float target, float current) {
  float diff = target - current;
  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return diff;
}
