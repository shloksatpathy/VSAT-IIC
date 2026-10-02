// Recovery state machine: ejection -> primary descent -> primary cut at 600 m
// -> separation check -> canopy deployment -> guided glide -> flare -> landed.
#pragma once

#include <stdint.h>

#include "actuators.h"
#include "altitude.h"
#include "geo.h"
#include "guidance.h"
#include "sensors.h"

enum class RecoveryState : uint8_t {
    WaitEjection,
    DescendPrimary,
    ConfirmSeparation,
    DeployCanopy,
    Guidance,
    Flare,
    Landed,
};

const char* stateName(RecoveryState s);

// True once a condition has held continuously for a given time.
class HoldTimer {
public:
    bool update(bool condition, uint32_t nowMs, uint32_t holdMs);
    void reset() { active_ = false; }

private:
    bool     active_ = false;
    uint32_t sinceMs_ = 0;
};

class RecoveryController {
public:
    RecoveryController(Sensors& sensors, RecoveryActuators& actuators);

    void begin(uint32_t nowMs);
    void update(uint32_t nowMs, float dt);

    RecoveryState state() const { return state_; }

private:
    enum class CanopyPhase : uint8_t { Inflating, Settling };

    void enter(RecoveryState next, uint32_t nowMs);
    void setOrigin(double lat, double lon);
    void handleGnssFix(uint32_t nowMs);
    bool landingDetected(uint32_t nowMs);
    void logTelemetry(uint32_t nowMs);
    void logLanding();

    Sensors&           sensors_;
    RecoveryActuators& actuators_;
    AltitudeEstimator  altitude_;
    Guidance           guidance_;
    GuidanceOutput     lastGuidance_ = {};

    RecoveryState state_ = RecoveryState::WaitEjection;
    uint32_t      stateEnteredMs_ = 0;

    bool   originSet_ = false;
    double originLat_ = 0.0, originLon_ = 0.0;
    Vec2   target_    = {0.0f, 0.0f};

    float    maxAltitude_ = 0.0f;
    uint8_t  belowCutCount_ = 0;
    uint32_t cutMs_ = 0;

    CanopyPhase canopyPhase_ = CanopyPhase::Inflating;
    uint32_t    phaseStartMs_ = 0;

    HoldTimer freefallHold_;
    HoldTimer inflatedHold_;
    HoldTimer settledHold_;
    HoldTimer stillHold_;

    uint32_t lastTelemetryMs_ = 0;
};
