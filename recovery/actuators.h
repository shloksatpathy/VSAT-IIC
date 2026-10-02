// Primary-parachute cutter, canopy release and the left/right brake servos.
#pragma once

#include <stdint.h>

class RecoveryActuators {
public:
    void begin();

    // Ends timed outputs (cutter pulse). Call every loop.
    void update(uint32_t nowMs);

    // Fires the cutter for CUTTER_PULSE_MS (non-blocking).
    void cutPrimary(uint32_t nowMs);
    void releaseCanopy();

    // Brake pull fractions in [0, 1]; 0 = neutral, 1 = fully pulled.
    void setBrakes(float left, float right);
    void brakesNeutral() { setBrakes(0.0f, 0.0f); }
    void brakesFull()    { setBrakes(1.0f, 1.0f); }

    // Detaches the servos and switches off their supply after landing.
    void cutServoPower();

private:
    bool     cutterOn_ = false;
    uint32_t cutterStartMs_ = 0;
    bool     servosPowered_ = false;
};
