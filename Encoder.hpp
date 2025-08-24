#pragma once

#include <Arduino.h>

namespace mtrn3100 {


// The encoder class is a simple interface which counts and stores an encoders count.
// Encoder pin 1 is attached to the interupt on the arduino and used to trigger the count.
// Encoder pin 2 is attached to any digital pin and used to derrive rotation direction.
// The count is stored as a volatile variable due to the high frequency updates. 
class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2, int id) : encoder1_pin(enc1), encoder2_pin(enc2) {
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        // TODO: attach the interrupt on pin one such that it calls the readEncoderISR function on a rising edge
        if (id == 1) {
            instance1 = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR1, RISING);
        } 
        else if (id == 2) {
            instance2 = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR2, RISING);
        }
    }


    // Encoder function used to update the encoder
    void readEncoder() {
        noInterrupts();

        // NOTE: DO NOT PLACE SERIAL PRINT STATEMENTS IN THIS FUNCTION
        // NOTE: DO NOT CALL THIS FUNCTION MANUALLY IT WILL ONLY WORK IF CALLED BY THE INTERRUPT
        // TODO: Increase or Decrease the count by one based on the reading on encoder pin 2
        if (digitalRead(encoder2_pin) == HIGH) {
        count++;
        } 
        else {
        count--;
        }

        interrupts();
    }

    // Helper function which to convert encouder count to radians
    float getRotation() {
        // TODO: Convert encoder count to radians
        if (counts_per_revolution == 0) {
            return 0;
        } 
        return (2.0f * M_PI * count) / counts_per_revolution;
    }

private:
    static void readEncoderISR1() {
        if (instance1 != nullptr) {
            instance1->readEncoder();
        }
    }

    static void readEncoderISR2() {
        if (instance2 != nullptr) {
            instance2->readEncoder();
        }
    }

    static Encoder* instance1;
    static Encoder* instance2;

public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile int8_t direction;
    float position = 0;
    uint16_t counts_per_revolution = 0;
    volatile long count = 0;
    uint32_t prev_time;
    bool read = false;
};

Encoder* Encoder::instance1 = nullptr;
Encoder* Encoder::instance2 = nullptr;

}  // namespace mtrn3100
