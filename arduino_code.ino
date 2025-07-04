#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "BangBangController.hpp"
#include <VL6180X.h>
#include <Wire.h>

VL6180X sensor1;

#define MOT1PWM 11 // PIN 11 is a PWM pin for motor1
#define MOT1DIR 12 // PIN 12 is a DIR pin for motor1
mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);

#define MOT2PWM 9 // PIN 9 is a PWM pin
#define MOT2DIR 10 // PIN 10 is a DIR pin
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

#define EN1_A 2 // PIN 2 is an interupt
#define EN1_B 7 // PIN 7 is an interupt
mtrn3100::Encoder encoder(EN1_A, EN1_B);

// #define EN2_A 3 // PIN 2 is an interupt
// #define EN2_B 7 // PIN 7 is an interupt
// mtrn3100::Encoder encoder(EN_A, EN_B);

mtrn3100::BangBangController controller(120,0);

void setup() {
  Wire.begin();
  Serial.begin(9600);
  controller.zeroAndSetTarget(encoder.getRotation(), 2.0); // Set the target as 2 Radians

  // SET UP ENABLE PINS AND DISABLE SENSORS
  pinMode(A0, OUTPUT);
  digitalWrite(A0, LOW);

  // ENABLE FIRST SENSOR AND CHANGE THE ADDRESS 
  digitalWrite(A0, HIGH);
  delay(50);
  sensor1.init();
  sensor1.configureDefault();
  sensor1.setTimeout(250);
  sensor1.setAddress(0x54);
  delay(50);

}

void loop() {
  Serial.print("Distance moved: ");
  Serial.print(encoder.getRotation());
  Serial.println(" mm");


  // int distance = sensor1.readRangeSingleMillimeters();
  // Serial.print(sensor1.readRangeSingleMillimeters());
  // if (distance <= 150) {
  //   motor1.setPWM(0); 
  //   motor2.setPWM(0);
  // } else {
  //   motor1.setPWM(150);
  //   motor2.setPWM(-150);
  // }


  if (encoder.getRotation() >= 200.0) {
    motor1.setPWM(0);
    motor2.setPWM(0);
  } else {
    motor1.setPWM(100);
    motor2.setPWM(-150);
  }
}