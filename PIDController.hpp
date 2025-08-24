#pragma once

#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd)
        : kp_(kp), ki_(ki), kd_(kd) {}

    // compute new signal
    float compute(float input) {
        curr_time_ = micros();
        dt_ = static_cast<float>(curr_time_ - prev_time_) * 1e-6f;
        prev_time_ = curr_time_;

        float measured = input - zero_reference_;
        error_ = setpoint_ - measured;
        integral_ += error_ * dt_;
        integral_ = constrain(integral_, -integral_max_, integral_max_); //integral limiting
        derivative_ = (error_ - previous_error_) / dt_;

        output_ = (kp_ * error_) + (ki_ * integral_) + (kd_ * derivative_);
        previous_error_ = error_;

        return output_;
    }

    // Tuning
    void tune(float p, float i, float d) {
        kp_ = p;
        ki_ = i;
        kd_ = d;
    }

    // Returns the most recently calculated error value
    float getError() {
        return error_;
    }

    // Set initial reference point and desired setpoint
    void zeroAndSetTarget(float zero_point, float target_value) {
        prev_time_ = micros();
        zero_reference_ = zero_point;
        setpoint_ = target_value;
    }

public:
    uint32_t curr_time_ = micros();
    uint32_t prev_time_;
    float dt_;

private:
    float kp_, ki_, kd_;
    float error_ = 0.0f;
    float previous_error_ = 0.0f;
    float derivative_ = 0.0f;
    float integral_ = 0.0f;
    float output_ = 0.0f;
    float setpoint_ = 0.0f;
    float zero_reference_ = 0.0f;
    const float integral_max_ = 200.0f;

    float constrain(float val, float min_val, float max_val) {
        if (val > max_val) return max_val;
        if (val < min_val) return min_val;
        return val;
    }
};

}  // namespace mtrn3100