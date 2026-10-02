// Stage 4 guidance: heading estimate, wind estimate, altitude-budget mode
// gating and the heading PID that drives the pull-only brake lines.
#pragma once

#include <stdint.h>

#include "geo.h"

enum class GlideMode : uint8_t { OnTarget, TooHigh, TooLow };

struct GuidanceOutput {
    float     leftBrake;       // pull fraction, [0, 1]
    float     rightBrake;      // pull fraction, [0, 1]
    GlideMode mode;
    float     angularError;    // deg, + = target is to the right
    float     distance;        // m to target
    float     effectiveRatio;
    float     requiredRatio;
    bool      active;          // false = holding wings level (no heading / stale sensors)
};

class Guidance {
public:
    // Clears all state and sets the target (local XY).
    void reset(const Vec2& target);

    // Call on every new GNSS fix (local XY).
    void onGnssFix(const Vec2& pos, uint32_t nowMs);

    // Call every loop with the yaw rate in heading convention (+ = clockwise).
    void onImu(float yawRateDps, float dt);

    GuidanceOutput update(float altitudeAgl, bool gnssFresh, bool imuFresh, float dt);

    const Vec2& position()    const { return pos_; }
    float       heading()     const { return heading_; }
    float       groundspeed() const { return groundspeed_; }
    const Vec2& wind()        const { return wind_; }

private:
    void resetPid();

    Vec2 target_ = {0.0f, 0.0f};

    Vec2     pos_       = {0.0f, 0.0f};
    bool     hasFix_    = false;
    Vec2     anchor_    = {0.0f, 0.0f};  // start of the current track baseline
    uint32_t anchorMs_  = 0;

    float heading_     = 0.0f;
    bool  hasHeading_  = false;
    float groundspeed_ = 0.0f;
    Vec2  wind_        = {0.0f, 0.0f};   // m/s, direction the wind blows toward

    float integral_     = 0.0f;
    float prevError_    = 0.0f;
    bool  hasPrevError_ = false;
};
