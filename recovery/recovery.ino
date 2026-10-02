// CAN7U-SAT secondary recovery and navigation - main loop.
//
// After ejection the CanSat descends on the primary parachute, cuts it at
// 600 m, deploys a steerable parafoil and glides to the target, managing its
// altitude budget, then flares and lands. See README.md for the stages.
//
// Sensor failures never halt the program: recovery continues on whatever
// sensors remain, falling back to dead reckoning and timeouts.

#include <Wire.h>

#include "actuators.h"
#include "config.h"
#include "recovery_fsm.h"
#include "sensors.h"

Sensors            sensors;
RecoveryActuators  actuators;
RecoveryController recovery(sensors, actuators);

uint32_t lastLoopUs = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);

    actuators.begin();

    Serial.println("Calibrating - keep the CanSat still");
    if (!sensors.begin()) {
        Serial.println("WARNING: sensor init failed, running on fallbacks");
    }
    if (TARGET_LAT == 0.0 && TARGET_LON == 0.0) {
        Serial.println("WARNING: target coordinates not set in config.h");
    }

    recovery.begin(millis());
    lastLoopUs = micros();
}

void loop() {
    // Keep the GNSS UART drained between control cycles.
    sensors.pollGnss(millis());

    const uint32_t now = micros();
    if (now - lastLoopUs < LOOP_PERIOD_US) return;

    const float dt = (now - lastLoopUs) * 1e-6f;
    lastLoopUs = now;

    const uint32_t nowMs = millis();
    sensors.update(nowMs);
    recovery.update(nowMs, dt);
}
