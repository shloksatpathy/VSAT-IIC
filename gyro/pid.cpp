#include "pid.h"

namespace {

constexpr float TWO_PI_F = 6.28318530718f;

float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

PID::PID(float kp, float ki, float kd, float outMin, float outMax, float dCutoffHz)
    : kp_(kp), ki_(ki), kd_(kd),
      outMin_(outMin), outMax_(outMax),
      dCutoffHz_(dCutoffHz) {}

float PID::update(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) {
        return clampf(integral_, outMin_, outMax_);
    }

    const float error = setpoint - measurement;

    // Derivative on measurement avoids a kick when the setpoint steps.
    float dMeas = 0.0f;
    if (hasPrevMeas_) {
        dMeas = (measurement - prevMeas_) / dt;
    }
    prevMeas_    = measurement;
    hasPrevMeas_ = true;

    // First-order low-pass on the derivative to suppress gyro noise.
    if (dCutoffHz_ > 0.0f) {
        const float rc    = 1.0f / (TWO_PI_F * dCutoffHz_);
        const float alpha = dt / (rc + dt);
        dFiltered_ += alpha * (dMeas - dFiltered_);
    } else {
        dFiltered_ = dMeas;
    }

    const float p = kp_ * error;
    const float d = -kd_ * dFiltered_;

    // Conditional integration: stop accumulating when the output is saturated
    // and the error would push it further into saturation.
    const float candidateI = clampf(integral_ + ki_ * error * dt, outMin_, outMax_);
    const float unclamped  = p + candidateI + d;
    const bool  saturatedHigh = unclamped > outMax_ && error > 0.0f;
    const bool  saturatedLow  = unclamped < outMin_ && error < 0.0f;
    if (!saturatedHigh && !saturatedLow) {
        integral_ = candidateI;
    }

    return clampf(p + integral_ + d, outMin_, outMax_);
}

void PID::reset() {
    integral_    = 0.0f;
    prevMeas_    = 0.0f;
    dFiltered_   = 0.0f;
    hasPrevMeas_ = false;
}

void PID::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::setOutputLimits(float outMin, float outMax) {
    outMin_   = outMin;
    outMax_   = outMax;
    integral_ = clampf(integral_, outMin_, outMax_);
}
