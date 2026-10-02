// Cascaded angle -> rate PID controller for roll, pitch and yaw.
#pragma once

#include "attitude_estimator.h"
#include "pid.h"

struct AttitudeSetpoint {
    float roll, pitch, yaw;  // deg
};

struct ActuatorCommand {
    float roll, pitch, yaw;  // normalised, [-1, 1]
};

class AttitudeController {
public:
    AttitudeController();

    ActuatorCommand update(const AttitudeSetpoint& sp, const Attitude& att, float dt);

    void reset();

private:
    PID rollAngle_, pitchAngle_, yawAngle_;
    PID rollRate_,  pitchRate_,  yawRate_;
};
