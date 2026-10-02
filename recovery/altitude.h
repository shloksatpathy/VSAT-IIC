// Altitude source selection with dead-reckoning fallback, plus a filtered
// descent rate.
//
//   1. Barometer, if valid and it agrees with GNSS (or GNSS is unavailable)
//   2. GNSS altitude
//   3. Dead reckoning: last good altitude minus expected descent rate * time
#pragma once

#include <stdint.h>

enum class AltSource : uint8_t { Baro, Gnss, DeadReckoning };

class AltitudeEstimator {
public:
    AltitudeEstimator(float maxBaroGnssDiffM, float rateTauS);

    // Pass NAN for a reading that is unavailable. Returns altitude AGL (m).
    float update(float baroAgl, float gnssAgl, uint32_t nowMs);

    // Forces a known altitude (used at ejection if no sensor is valid).
    void seed(float altitude, uint32_t nowMs);

    // Descent rate assumed while dead reckoning (depends on the flight stage).
    void setDeadReckoningRate(float mps) { drRate_ = mps; }

    float     altitude()    const { return altitude_; }
    float     descentRate() const { return rate_; }  // m/s, positive = descending
    AltSource source()      const { return source_; }

private:
    float maxDiff_;
    float tau_;
    float drRate_ = 0.0f;

    float     altitude_ = 0.0f;
    float     rate_     = 0.0f;
    AltSource source_   = AltSource::DeadReckoning;

    float    lastGoodAlt_ = 0.0f;
    uint32_t lastGoodMs_  = 0;

    float    prevAlt_ = 0.0f;
    uint32_t prevMs_  = 0;
    bool     hasPrev_ = false;
};
