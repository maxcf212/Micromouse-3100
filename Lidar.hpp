#pragma once

#include <Wire.h>
#include <VL6180X.h>

#define LIDAR1_PIN 14
#define LIDAR2_PIN 15
#define LIDAR3_PIN 16

namespace mtrn3100 {

class Lidar {
public:
    Lidar() = default;

    void initialize() {
        Wire.begin();

        // config sensor pins
        pinMode(LIDAR1_PIN, OUTPUT);
        pinMode(LIDAR2_PIN, OUTPUT);
        pinMode(LIDAR3_PIN, OUTPUT);

        // all sensors are initially off
        digitalWrite(LIDAR1_PIN, LOW);
        digitalWrite(LIDAR2_PIN, LOW);
        digitalWrite(LIDAR3_PIN, LOW);
        delay(10);

        // power up and initialize each sensor
        setupSensor(sensor1, LIDAR1_PIN, 0x54);
        setupSensor(sensor2, LIDAR2_PIN, 0x56);
        setupSensor(sensor3, LIDAR3_PIN, 0x58);
    }

    uint8_t* measureDistances() {
        static uint8_t readings[3];
        readings[0] = sensor1.readRangeSingle();
        readings[1] = sensor2.readRangeSingle();
        readings[2] = sensor3.readRangeSingle();
        return readings;
    }

    bool didTimeoutOccur(int index) {
        switch (index) {
            case 1: return sensor1.timeoutOccurred();
            case 2: return sensor2.timeoutOccurred();
            case 3: return sensor3.timeoutOccurred();
            default: return false;
        }
    }

private:
    VL6180X sensor1, sensor2, sensor3;

    void setupSensor(VL6180X& sensor, int pin, uint8_t address) {
        digitalWrite(pin, HIGH);
        delay(50);
        sensor.init();
        sensor.configureDefault();
        sensor.setAddress(address);
        sensor.setTimeout(500);
    }
};

}  // namespace mtrn3100