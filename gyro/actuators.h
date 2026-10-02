// Drives the control-moment-gyro gimbal servos, the parachute yaw servo and
// the flywheel ESC from normalised controller commands.
#pragma once

#include "attitude_controller.h"

class Actuators {
public:
    void begin();

    // Spins the flywheel up; blocks for FLYWHEEL_SPINUP_MS.
    void startFlywheel();
    void stopFlywheel();

    void write(const ActuatorCommand& cmd);

    // Centres every servo (failsafe / disarmed state).
    void centre();
};
