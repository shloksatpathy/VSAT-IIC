#include "guidance.h"

#include <math.h>

#include "config.h"

namespace {

constexpr float DEG_TO_RAD_F = 0.0174532925f;
constexpr float RAD_TO_DEG_F = 57.2957795131f;

float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

void Guidance::reset(const Vec2& target) {
    target_      = target;
    hasFix_      = false;
    hasHeading_  = false;
    groundspeed_ = 0.0f;
    wind_        = {0.0f, 0.0f};
    resetPid();
}

void Guidance::resetPid() {
    integral_     = 0.0f;
    prevError_    = 0.0f;
    hasPrevError_ = false;
}

void Guidance::onGnssFix(const Vec2& pos, uint32_t nowMs) {
    pos_ = pos;
    if (!hasFix_) {
        hasFix_   = true;
        anchor_   = pos;
        anchorMs_ = nowMs;
        return;
    }

    // Ground track from successive positions. Wait for a minimum baseline so
    // GNSS position noise does not dominate the bearing.
    const Vec2  delta    = {pos.x - anchor_.x, pos.y - anchor_.y};
    const float baseline = magnitude(delta);
    if (baseline < MIN_TRACK_BASELINE_M || nowMs <= anchorMs_) return;

    const float track = bearingDeg(anchor_, pos);
    groundspeed_      = baseline / ((nowMs - anchorMs_) * 1e-3f);
    anchor_           = pos;
    anchorMs_         = nowMs;

    // Complementary filter: gyro integration between fixes, pulled toward
    // the ground track to cancel gyro drift.
    if (!hasHeading_) {
        heading_    = track;
        hasHeading_ = true;
    } else {
        heading_ = wrap180(heading_ + HEADING_GNSS_WEIGHT * wrap180(track - heading_));
    }

    // Wind = ground velocity - air velocity (canopy flies along its heading).
    const float trackRad   = track * DEG_TO_RAD_F;
    const float headingRad = heading_ * DEG_TO_RAD_F;
    const float windX = groundspeed_ * sinf(trackRad) - CANOPY_AIRSPEED_MPS * sinf(headingRad);
    const float windY = groundspeed_ * cosf(trackRad) - CANOPY_AIRSPEED_MPS * cosf(headingRad);
    wind_.x += WIND_FILTER_ALPHA * (windX - wind_.x);
    wind_.y += WIND_FILTER_ALPHA * (windY - wind_.y);
}

void Guidance::onImu(float yawRateDps, float dt) {
    if (hasHeading_) {
        heading_ = wrap180(heading_ + yawRateDps * dt);
    }
}

GuidanceOutput Guidance::update(float altitudeAgl, bool gnssFresh, bool imuFresh, float dt) {
    GuidanceOutput out = {};
    out.mode = GlideMode::OnTarget;

    // Sensor fail-safe: wings-level glide until both sensors are fresh and a
    // heading has been established.
    if (!gnssFresh || !imuFresh || !hasFix_ || !hasHeading_) {
        resetPid();
        return out;
    }
    out.active = true;

    const Vec2 toTarget = {target_.x - pos_.x, target_.y - pos_.y};
    out.distance        = magnitude(toTarget);
    const float targetBearing = atan2f(toTarget.x, toTarget.y) * RAD_TO_DEG_F;
    out.angularError    = wrap180(targetBearing - heading_);

    // Wind component along the direction to the target: + tailwind, - headwind.
    float windAlong = 0.0f;
    if (out.distance > 1.0f) {
        windAlong = (wind_.x * toTarget.x + wind_.y * toTarget.y) / out.distance;
    }

    // Glide ratio over the ground scales with groundspeed / airspeed.
    out.effectiveRatio = CANOPY_MAX_GLIDE_RATIO * (CANOPY_AIRSPEED_MPS + windAlong) / CANOPY_AIRSPEED_MPS;
    if (out.effectiveRatio < MIN_EFFECTIVE_GLIDE_RATIO) out.effectiveRatio = MIN_EFFECTIVE_GLIDE_RATIO;
    out.requiredRatio = out.distance / (altitudeAgl > 1.0f ? altitudeAgl : 1.0f);

    if (out.requiredRatio < out.effectiveRatio * TOO_HIGH_MARGIN) {
        out.mode = GlideMode::TooHigh;
    } else if (out.requiredRatio > out.effectiveRatio) {
        out.mode = GlideMode::TooLow;
    }

    // Heading PID. The derivative uses the wrapped error difference so a
    // crossing of +/-180 deg does not produce a spike.
    integral_ = clampf(integral_ + out.angularError * dt, -HEADING_INTEGRAL_LIMIT, HEADING_INTEGRAL_LIMIT);
    float derivative = 0.0f;
    if (hasPrevError_ && dt > 0.0f) {
        derivative = wrap180(out.angularError - prevError_) / dt;
    }
    prevError_    = out.angularError;
    hasPrevError_ = true;

    float correction = HEADING_KP * out.angularError + HEADING_KI * integral_ + HEADING_KD * derivative;

    if (out.mode == GlideMode::TooHigh) {
        correction *= TURN_BOOST_FACTOR;  // tighter turns burn altitude
    } else if (out.mode == GlideMode::TooLow) {
        if (fabsf(out.angularError) < LARGE_ERROR_DEG) {
            correction = 0.0f;            // fly straight, preserve altitude
        } else {
            correction *= MINIMAL_FACTOR; // bare-minimum redirect
        }
    }
    correction = clampf(correction, -1.0f, 1.0f);

    // Pull-only brakes: positive error means the target is to the right, and
    // pulling the right brake turns a parafoil right.
    out.rightBrake = correction > 0.0f ? correction : 0.0f;
    out.leftBrake  = correction < 0.0f ? -correction : 0.0f;
    return out;
}
