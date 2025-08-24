#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Lidar.hpp"

using mtrn3100::Motor;
using mtrn3100::Lidar;
using mtrn3100::PIDController;

// === PID PARAMETERS ===
constexpr float KP = 0.4f;
constexpr float KI = 0.1f;
constexpr float KD = 0.03f;

// === DESIRED CONTROL TARGET ===
constexpr float DESIRED_DISTANCE_MM = 100.0f;

// === MOTOR PIN MAPPINGS ===
#define PWM_LEFT 11
#define DIR_LEFT 12
#define PWM_RIGHT 9
#define DIR_RIGHT 10

// === DEVICE OBJECTS ===
Motor leftMotor(PWM_LEFT, DIR_LEFT);
Motor rightMotor(PWM_RIGHT, DIR_RIGHT);
Lidar distanceSensors;
PIDController distancePID(KP, KI, KD);

void setup() {
    Serial.begin(9600);
    distanceSensors.begin();

    //initialize PID target based on current offset
    distancePID.zeroAndSetTarget(0, DESIRED_DISTANCE_MM);

    //debug
    Serial.print("Desired range: ");
    Serial.print(DESIRED_DISTANCE_MM);
    Serial.println(" mm");
}

void loop() {
    // === READ LIDAR ===
    uint8_t* lidarValues = distanceSensors.getDistance();
    float measuredDistance = static_cast<float>(lidarValues[1]);  // centre sensor

    if (distanceSensors.timeoutOccurred(2)) {
        Serial.println("warning: LIDAR timeout");
        leftMotor.setPWM(0);
        rightMotor.setPWM(0);
        delay(50);
        return;
    }

    // === PERFORM PID CONTROL ===
    float controlSignal = distancePID.compute(measuredDistance);

    leftMotor.setPWM(static_cast<int>(controlSignal));
    rightMotor.setPWM(static_cast<int>(-controlSignal));

    // === MONITOR OUTPUT ===
    Serial.print("Measured distance: ");
    Serial.print(measuredDistance);
    Serial.print(" mm  |  Output PWM: ");
    Serial.println(controlSignal);

    delay(30);
}