#pragma once

#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    // Compute the output signal required from the current/actual value.
    float compute(float input) {

        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        if (dt <= 0) dt = 1e-6; //avoid div by 0

        error = setpoint - (input - zero_ref);

        // TODO: IMPLIMENT PID CONTROLLER

        // calculate new i and d
        integral += error * dt;
        derivative = (error - prev_error) / dt;

        // PID output
        output = (kp * error) + (ki * integral) + (kd * derivative);

        prev_error = error;

        return output;
    }

    // Function used to return the last calculated error.
    // The error is the difference between the desired position and current position. 
    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    float getError() {
      return error;
    }

    // This must be called before trying to achieve a setpoint.
    // The first argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
        prev_time = micros();
        zero_ref = zero;
        setpoint = target;
        // reset pid state
        integral = 0;
        prev_error = 0;
    }

public:
    uint32_t prev_time, curr_time = micros();
    float dt;

private:
    float kp, ki, kd;
    float error, derivative, integral, output;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;


};

}  // namespace mtrn3100
