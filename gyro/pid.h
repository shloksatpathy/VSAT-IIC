// Discrete PID controller with derivative-on-measurement, a low-pass filtered
// D term, output clamping, and conditional-integration anti-windup.
#pragma once

class PID {
public:
    PID(float kp, float ki, float kd,
        float outMin, float outMax,
        float dCutoffHz = 20.0f);

    // Returns the controller output for one step of length dt (seconds).
    float update(float setpoint, float measurement, float dt);

    // Clears the integrator and derivative history (call when (re)arming).
    void reset();

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float outMin, float outMax);

    float integral() const { return integral_; }

private:
    float kp_, ki_, kd_;
    float outMin_, outMax_;
    float dCutoffHz_;

    float integral_     = 0.0f;
    float prevMeas_     = 0.0f;
    float dFiltered_    = 0.0f;
    bool  hasPrevMeas_  = false;
};
