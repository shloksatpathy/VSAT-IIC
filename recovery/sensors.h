// Barometer (BMP280), GNSS (any NMEA receiver via TinyGPSPlus) and IMU
// (MPU6050) for the recovery system. Altitudes are AGL at the launch site.
#pragma once

#include <stdint.h>

class Sensors {
public:
    // Initialises all sensors and records the ground reference pressure and
    // gyro bias. Power on at the launch site with the CanSat still.
    // Returns false if any sensor failed; recovery still runs on the rest.
    bool begin();

    // Drains the GNSS serial buffer. Call as often as possible.
    void pollGnss(uint32_t nowMs);

    // Samples the barometer and IMU. Call once per control loop.
    void update(uint32_t nowMs);

    float baroAltitudeAgl() const;            // NAN if invalid
    float gnssAltitudeAgl(uint32_t nowMs) const;  // NAN if invalid or stale

    // True once per new GNSS position fix.
    bool takeNewFix();
    double latitude()  const { return lat_; }
    double longitude() const { return lon_; }

    bool gnssFresh(uint32_t nowMs) const;
    bool imuFresh(uint32_t nowMs) const;

    float accelMagnitudeG() const { return accelMagG_; }
    float yawRateDps()      const { return yawRateDps_; }  // + = clockwise (heading convention)
    float maxBodyRateDps()  const { return maxRateDps_; }

private:
    bool readImu(uint32_t nowMs);

    bool  baroOk_ = false;
    bool  imuOk_  = false;
    float groundPressureHpa_ = 1013.25f;
    float baroAgl_ = 0.0f;
    bool  baroValid_ = false;

    double   lat_ = 0.0, lon_ = 0.0;
    float    gnssAltMsl_ = 0.0f;
    bool     gnssAltValid_ = false;
    float    groundGnssMsl_ = 0.0f;
    bool     groundGnssSet_ = false;
    bool     newFix_ = false;
    bool     hasFix_ = false;
    uint32_t lastGnssFixMs_ = 0;

    float    gyroBias_[3] = {0.0f, 0.0f, 0.0f};
    float    accelMagG_ = 1.0f;
    float    yawRateDps_ = 0.0f;
    float    maxRateDps_ = 0.0f;
    bool     hasImu_ = false;
    uint32_t lastImuMs_ = 0;
};
