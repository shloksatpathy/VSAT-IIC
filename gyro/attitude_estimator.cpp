#include "attitude_estimator.h"

#include <math.h>

namespace {

constexpr float RAD_TO_DEG_F = 57.2957795131f;

float accelRoll(const ImuSample& s) {
    return atan2f(s.ay, s.az) * RAD_TO_DEG_F;
}

float accelPitch(const ImuSample& s) {
    return atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * RAD_TO_DEG_F;
}

}  // namespace

float wrap180(float deg) {
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg - 180.0f;
}

AttitudeEstimator::AttitudeEstimator(float alpha, float accelTrustBandG)
    : alpha_(alpha), accelTrustBandG_(accelTrustBandG) {}

void AttitudeEstimator::init(const ImuSample& s) {
    att_       = {};
    att_.roll  = accelRoll(s);
    att_.pitch = accelPitch(s);
}

void AttitudeEstimator::update(const ImuSample& s, float dt) {
    att_.rollRate  = s.gx;
    att_.pitchRate = s.gy;
    att_.yawRate   = s.gz;

    // Integrate gyro rates. Treating body rates as Euler-angle rates is a
    // small-angle approximation, fine while the payload stays near level.
    const float gyroRoll  = att_.roll  + s.gx * dt;
    const float gyroPitch = att_.pitch + s.gy * dt;
    att_.yaw += s.gz * dt;

    // Only trust the accelerometer when it is measuring roughly gravity alone.
    const float accelMag = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
    accelTrusted_ = fabsf(accelMag - 1.0f) < accelTrustBandG_;

    if (accelTrusted_) {
        att_.roll  = alpha_ * gyroRoll  + (1.0f - alpha_) * accelRoll(s);
        att_.pitch = alpha_ * gyroPitch + (1.0f - alpha_) * accelPitch(s);
    } else {
        att_.roll  = gyroRoll;
        att_.pitch = gyroPitch;
    }
}

void AttitudeEstimator::correctYaw(float headingDeg, float weight) {
    att_.yaw += weight * wrap180(headingDeg - att_.yaw);
}
