#include "recovery_fsm.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"

const char* stateName(RecoveryState s) {
    switch (s) {
        case RecoveryState::WaitEjection:      return "WAIT_EJECTION";
        case RecoveryState::DescendPrimary:    return "DESCEND_PRIMARY";
        case RecoveryState::ConfirmSeparation: return "CONFIRM_SEPARATION";
        case RecoveryState::DeployCanopy:      return "DEPLOY_CANOPY";
        case RecoveryState::Guidance:          return "GUIDANCE";
        case RecoveryState::Flare:             return "FLARE";
        case RecoveryState::Landed:            return "LANDED";
    }
    return "?";
}

namespace {

const char* modeName(GlideMode m) {
    switch (m) {
        case GlideMode::OnTarget: return "ON_TARGET";
        case GlideMode::TooHigh:  return "TOO_HIGH";
        case GlideMode::TooLow:   return "TOO_LOW";
    }
    return "?";
}

const char* sourceName(AltSource s) {
    switch (s) {
        case AltSource::Baro:          return "BARO";
        case AltSource::Gnss:          return "GNSS";
        case AltSource::DeadReckoning: return "DR";
    }
    return "?";
}

}  // namespace

bool HoldTimer::update(bool condition, uint32_t nowMs, uint32_t holdMs) {
    if (!condition) {
        active_ = false;
        return false;
    }
    if (!active_) {
        active_  = true;
        sinceMs_ = nowMs;
    }
    return nowMs - sinceMs_ >= holdMs;
}

RecoveryController::RecoveryController(Sensors& sensors, RecoveryActuators& actuators)
    : sensors_(sensors),
      actuators_(actuators),
      altitude_(BARO_GNSS_MAX_DIFF_M, DESCENT_RATE_TAU_S) {}

void RecoveryController::begin(uint32_t nowMs) {
    if (PIN_EJECTION_SWITCH >= 0) pinMode(PIN_EJECTION_SWITCH, INPUT_PULLUP);
    altitude_.setDeadReckoningRate(0.0f);
    enter(RecoveryState::WaitEjection, nowMs);
}

void RecoveryController::setOrigin(double lat, double lon) {
    originLat_ = lat;
    originLon_ = lon;
    target_    = toLocalXY(TARGET_LAT, TARGET_LON, originLat_, originLon_);
    originSet_ = true;
    guidance_.reset(target_);

    Serial.print("Origin set, target at X=");
    Serial.print(target_.x, 1);
    Serial.print(" Y=");
    Serial.println(target_.y, 1);
}

void RecoveryController::handleGnssFix(uint32_t nowMs) {
    // The origin is the ejection point. If there was no fix at ejection,
    // the first fix afterwards is used instead.
    if (!originSet_ && state_ != RecoveryState::WaitEjection) {
        setOrigin(sensors_.latitude(), sensors_.longitude());
    }
    if (originSet_) {
        guidance_.onGnssFix(toLocalXY(sensors_.latitude(), sensors_.longitude(), originLat_, originLon_), nowMs);
    }
}

void RecoveryController::enter(RecoveryState next, uint32_t nowMs) {
    state_          = next;
    stateEnteredMs_ = nowMs;

    switch (next) {
        case RecoveryState::DescendPrimary:
            altitude_.setDeadReckoningRate(PRIMARY_DESCENT_RATE_MPS);
            belowCutCount_ = 0;
            break;
        case RecoveryState::ConfirmSeparation:
            freefallHold_.reset();
            break;
        case RecoveryState::DeployCanopy:
            actuators_.releaseCanopy();
            altitude_.setDeadReckoningRate(CANOPY_DESCENT_RATE_MPS);
            canopyPhase_  = CanopyPhase::Inflating;
            phaseStartMs_ = nowMs;
            inflatedHold_.reset();
            settledHold_.reset();
            break;
        case RecoveryState::Guidance:
            // Fresh heading and wind estimate: the canopy may have rotated
            // during deployment.
            if (originSet_) guidance_.reset(target_);
            stillHold_.reset();
            break;
        case RecoveryState::Flare:
            actuators_.brakesFull();
            stillHold_.reset();
            break;
        case RecoveryState::Landed:
            actuators_.cutServoPower();
            logLanding();
            break;
        case RecoveryState::WaitEjection:
            break;
    }

    Serial.print("STATE -> ");
    Serial.println(stateName(next));
}

bool RecoveryController::landingDetected(uint32_t nowMs) {
    if (sensors_.imuFresh(nowMs) && sensors_.accelMagnitudeG() > LANDING_ACCEL_G) return true;
    const bool still = altitude_.source() != AltSource::DeadReckoning &&
                       fabsf(altitude_.descentRate()) < LANDED_DESCENT_RATE_MPS;
    return stillHold_.update(still, nowMs, LANDED_CONFIRM_MS);
}

void RecoveryController::update(uint32_t nowMs, float dt) {
    const float alt = altitude_.update(sensors_.baroAltitudeAgl(), sensors_.gnssAltitudeAgl(nowMs), nowMs);
    if (sensors_.takeNewFix()) handleGnssFix(nowMs);

    switch (state_) {
        case RecoveryState::WaitEjection: {
            if (altitude_.source() != AltSource::DeadReckoning && alt > maxAltitude_) {
                maxAltitude_ = alt;
            }
            const bool switchTripped = PIN_EJECTION_SWITCH >= 0 && digitalRead(PIN_EJECTION_SWITCH) == LOW;
            const bool pastApogee    = maxAltitude_ > EJECTION_MIN_ALTITUDE_M &&
                                       alt < maxAltitude_ - APOGEE_DROP_M;
            if (switchTripped || pastApogee) {
                // No valid altitude at all: assume the nominal ejection altitude.
                if (altitude_.source() == AltSource::DeadReckoning) {
                    altitude_.seed(EJECTION_ALTITUDE_M, nowMs);
                }
                if (sensors_.gnssFresh(nowMs)) setOrigin(sensors_.latitude(), sensors_.longitude());
                enter(RecoveryState::DescendPrimary, nowMs);
            }
            break;
        }

        // ---- Stage 1: altitude trigger ----
        case RecoveryState::DescendPrimary:
            belowCutCount_ = alt <= CUT_ALTITUDE_M ? belowCutCount_ + 1 : 0;
            if (belowCutCount_ >= CUT_CONFIRM_SAMPLES) {
                actuators_.cutPrimary(nowMs);
                cutMs_ = nowMs;
                enter(RecoveryState::ConfirmSeparation, nowMs);
            }
            break;

        // ---- Stage 2: confirm separation ----
        case RecoveryState::ConfirmSeparation: {
            const bool freefall = sensors_.imuFresh(nowMs) && sensors_.accelMagnitudeG() < FREEFALL_G;
            if (freefallHold_.update(freefall, nowMs, FREEFALL_CONFIRM_MS)) {
                enter(RecoveryState::DeployCanopy, nowMs);
            } else if (nowMs - cutMs_ > SEPARATION_TIMEOUT_MS) {
                Serial.println("Separation not confirmed, deploying anyway");
                enter(RecoveryState::DeployCanopy, nowMs);
            }
            break;
        }

        // ---- Stage 3: deploy canopy ----
        case RecoveryState::DeployCanopy:
            if (canopyPhase_ == CanopyPhase::Inflating) {
                const bool slow = altitude_.source() != AltSource::DeadReckoning &&
                                  altitude_.descentRate() < INFLATED_DESCENT_RATE_MPS;
                const bool timedOut = nowMs - phaseStartMs_ > INFLATION_TIMEOUT_MS;
                if (inflatedHold_.update(slow, nowMs, INFLATION_CONFIRM_MS) || timedOut) {
                    if (timedOut) Serial.println("Inflation not confirmed, continuing");
                    canopyPhase_  = CanopyPhase::Settling;
                    phaseStartMs_ = nowMs;
                }
            } else {
                const bool calm = sensors_.imuFresh(nowMs) &&
                                  sensors_.maxBodyRateDps() < SWING_SETTLED_RATE_DPS;
                if (settledHold_.update(calm, nowMs, SWING_SETTLE_MS) ||
                    nowMs - phaseStartMs_ > SWING_TIMEOUT_MS) {
                    enter(RecoveryState::Guidance, nowMs);
                }
            }
            break;

        // ---- Stage 4: guidance + altitude-budget control ----
        case RecoveryState::Guidance: {
            const bool imuFresh = sensors_.imuFresh(nowMs);
            if (imuFresh) guidance_.onImu(sensors_.yawRateDps(), dt);
            lastGuidance_ = guidance_.update(alt, originSet_ && sensors_.gnssFresh(nowMs), imuFresh, dt);
            actuators_.setBrakes(lastGuidance_.leftBrake, lastGuidance_.rightBrake);

            if (alt < FLARE_ALTITUDE_M) {
                enter(RecoveryState::Flare, nowMs);
            } else if (landingDetected(nowMs)) {
                enter(RecoveryState::Landed, nowMs);  // missed the flare altitude
            }
            break;
        }

        // ---- Stage 5: flare and landing ----
        case RecoveryState::Flare:
            if (landingDetected(nowMs)) enter(RecoveryState::Landed, nowMs);
            break;

        case RecoveryState::Landed:
            break;
    }

    actuators_.update(nowMs);
    logTelemetry(nowMs);
}

void RecoveryController::logTelemetry(uint32_t nowMs) {
    if (state_ == RecoveryState::Landed) return;
    if (nowMs - lastTelemetryMs_ < static_cast<uint32_t>(1000.0f / TELEMETRY_HZ)) return;
    lastTelemetryMs_ = nowMs;

    Serial.print(stateName(state_));
    Serial.print(" alt:");  Serial.print(altitude_.altitude(), 1);
    Serial.print(" src:");  Serial.print(sourceName(altitude_.source()));
    Serial.print(" vz:");   Serial.print(altitude_.descentRate(), 2);
    if (state_ == RecoveryState::Guidance) {
        const Vec2& p = guidance_.position();
        Serial.print(" X:");    Serial.print(p.x, 1);
        Serial.print(" Y:");    Serial.print(p.y, 1);
        Serial.print(" hdg:");  Serial.print(guidance_.heading(), 1);
        Serial.print(" err:");  Serial.print(lastGuidance_.angularError, 1);
        Serial.print(" dist:"); Serial.print(lastGuidance_.distance, 1);
        Serial.print(" req:");  Serial.print(lastGuidance_.requiredRatio, 2);
        Serial.print(" eff:");  Serial.print(lastGuidance_.effectiveRatio, 2);
        Serial.print(" mode:"); Serial.print(lastGuidance_.active ? modeName(lastGuidance_.mode) : "HOLD");
        Serial.print(" L:");    Serial.print(lastGuidance_.leftBrake, 2);
        Serial.print(" R:");    Serial.print(lastGuidance_.rightBrake, 2);
    }
    Serial.println();
}

void RecoveryController::logLanding() {
    if (!originSet_) {
        Serial.println("LANDED: no GNSS origin, position unknown");
        return;
    }
    const Vec2  pos  = toLocalXY(sensors_.latitude(), sensors_.longitude(), originLat_, originLon_);
    const Vec2  miss = {target_.x - pos.x, target_.y - pos.y};
    Serial.print("LANDED final_X:");
    Serial.print(pos.x, 1);
    Serial.print(" final_Y:");
    Serial.print(pos.y, 1);
    Serial.print(" distance_from_target:");
    Serial.println(magnitude(miss), 1);
}
