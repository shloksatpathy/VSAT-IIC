// Complementary filter that fuses gyro and accelerometer data into roll,
// pitch and yaw. Yaw is gyro-only (no magnetometer) and will drift; correct it
// from the GNSS ground track once that integration exists.
#pragma once

#include "imu.h"

struct Attitude {
    float roll, pitch, yaw;              // deg (yaw is continuous, not wrapped)
    float rollRate, pitchRate, yawRate;  // deg/s
};

class AttitudeEstimator {
public:
    AttitudeEstimator(float alpha, float accelTrustBandG);

    // Seeds roll/pitch from the accelerometer so the filter starts converged.
    void init(const ImuSample& s);

    void update(const ImuSample& s, float dt);

    // Nudges yaw toward an external heading (e.g. GNSS course), weight in [0, 1].
    void correctYaw(float headingDeg, float weight);

    const Attitude& attitude() const { return att_; }

    // True if the last update used the accelerometer correction.
    bool accelTrusted() const { return accelTrusted_; }

private:
    float    alpha_;
    float    accelTrustBandG_;
    Attitude att_ = {};
    bool     accelTrusted_ = false;
};

// Wraps an angle to (-180, 180].
float wrap180(float deg);
