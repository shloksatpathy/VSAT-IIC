// CAN7U-SAT gyro attitude control - main loop.
//
// Pipeline each cycle:  IMU -> complementary filter -> cascaded PID -> actuators
// Failsafe: repeated IMU read failures centre the actuators and reset the PIDs.

#include <Wire.h>

#include "actuators.h"
#include "attitude_controller.h"
#include "attitude_estimator.h"
#include "config.h"
#include "imu.h"

Imu                imu(IMU_I2C_ADDR);
AttitudeEstimator  estimator(COMP_ALPHA, ACCEL_TRUST_BAND_G);
AttitudeController controller;
Actuators          actuators;

// Hold the payload level and keep the heading it had at startup.
AttitudeSetpoint setpoint = {0.0f, 0.0f, 0.0f};

uint32_t lastLoopUs     = 0;
uint32_t lastDebugUs    = 0;
uint8_t  imuFailCount   = 0;
bool     failsafeActive = false;

void haltWithError(const char* msg) {
    actuators.centre();
    actuators.stopFlywheel();
    while (true) {
        Serial.println(msg);
        delay(1000);
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(IMU_I2C_CLOCK_HZ);

    actuators.begin();

    if (!imu.begin()) haltWithError("IMU not found");

    Serial.println("Calibrating gyro - keep still");
    if (!imu.calibrateGyro(IMU_CALIB_SAMPLES)) haltWithError("Gyro calibration failed");

    ImuSample s;
    if (!imu.read(s)) haltWithError("IMU read failed");
    estimator.init(s);
    setpoint.yaw = estimator.attitude().yaw;

    Serial.println("Spinning up flywheel");
    actuators.startFlywheel();

    controller.reset();
    lastLoopUs = micros();
    Serial.println("Attitude control running");
}

void loop() {
    const uint32_t now = micros();
    if (now - lastLoopUs < LOOP_PERIOD_US) return;

    const float dt = (now - lastLoopUs) * 1e-6f;
    lastLoopUs = now;

    ImuSample s;
    if (!imu.read(s)) {
        if (imuFailCount < 255) ++imuFailCount;
        if (imuFailCount >= IMU_MAX_READ_FAILS && !failsafeActive) {
            failsafeActive = true;
            actuators.centre();
            controller.reset();
            Serial.println("FAILSAFE: IMU lost, actuators centred");
        }
        return;
    }

    if (failsafeActive) {
        // IMU is back: re-seed the estimator and resume from a clean state.
        failsafeActive = false;
        estimator.init(s);
        controller.reset();
        Serial.println("IMU recovered, control resumed");
    }
    imuFailCount = 0;

    estimator.update(s, dt);
    const Attitude& att = estimator.attitude();
    const ActuatorCommand cmd = controller.update(setpoint, att, dt);
    actuators.write(cmd);

    if (now - lastDebugUs >= static_cast<uint32_t>(1e6f / DEBUG_HZ)) {
        lastDebugUs = now;
        Serial.print("R:");    Serial.print(att.roll, 1);
        Serial.print(" P:");   Serial.print(att.pitch, 1);
        Serial.print(" Y:");   Serial.print(att.yaw, 1);
        Serial.print(" cR:");  Serial.print(cmd.roll, 2);
        Serial.print(" cP:");  Serial.print(cmd.pitch, 2);
        Serial.print(" cY:");  Serial.print(cmd.yaw, 2);
        Serial.print(" acc:"); Serial.println(estimator.accelTrusted() ? 1 : 0);
    }
}
