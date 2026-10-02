#include "altitude.h"

#include <math.h>

AltitudeEstimator::AltitudeEstimator(float maxBaroGnssDiffM, float rateTauS)
    : maxDiff_(maxBaroGnssDiffM), tau_(rateTauS) {}

void AltitudeEstimator::seed(float altitude, uint32_t nowMs) {
    altitude_    = altitude;
    lastGoodAlt_ = altitude;
    lastGoodMs_  = nowMs;
    hasPrev_     = false;
}

float AltitudeEstimator::update(float baroAgl, float gnssAgl, uint32_t nowMs) {
    const bool baroOk = !isnan(baroAgl);
    const bool gnssOk = !isnan(gnssAgl);

    AltSource src;
    float     alt;
    if (baroOk && (!gnssOk || fabsf(baroAgl - gnssAgl) < maxDiff_)) {
        src = AltSource::Baro;
        alt = baroAgl;
    } else if (gnssOk) {
        src = AltSource::Gnss;
        alt = gnssAgl;
    } else {
        src = AltSource::DeadReckoning;
        alt = lastGoodAlt_ - drRate_ * (nowMs - lastGoodMs_) * 1e-3f;
    }

    if (src == AltSource::DeadReckoning) {
        rate_    = drRate_;
        hasPrev_ = false;
    } else {
        // Skip the derivative when the source changes to avoid a step spike.
        if (hasPrev_ && src == source_ && nowMs > prevMs_) {
            const float dt    = (nowMs - prevMs_) * 1e-3f;
            const float raw   = (prevAlt_ - alt) / dt;
            const float alpha = dt / (tau_ + dt);
            rate_ += alpha * (raw - rate_);
        }
        prevAlt_     = alt;
        prevMs_      = nowMs;
        hasPrev_     = true;
        lastGoodAlt_ = alt;
        lastGoodMs_  = nowMs;
    }

    source_   = src;
    altitude_ = alt;
    return alt;
}
