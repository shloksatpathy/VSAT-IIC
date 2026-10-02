#include "attitude_controller.h"

#include "config.h"

namespace {

PID makeAnglePid(const PidGains& g) {
    // Angle-loop kd is usually zero because the inner rate loop already
    // provides damping.
    return PID(g.kp, g.ki, g.kd, -MAX_RATE_SETPOINT_DPS, MAX_RATE_SETPOINT_DPS);
}

PID makeRatePid(const PidGains& g) {
    return PID(g.kp, g.ki, g.kd, -1.0f, 1.0f, D_TERM_CUTOFF_HZ);
}

}  // namespace

AttitudeController::AttitudeController()
    : rollAngle_(makeAnglePid(ROLL_ANGLE_GAINS)),
      pitchAngle_(makeAnglePid(PITCH_ANGLE_GAINS)),
      yawAngle_(makeAnglePid(YAW_ANGLE_GAINS)),
      rollRate_(makeRatePid(ROLL_RATE_GAINS)),
      pitchRate_(makeRatePid(PITCH_RATE_GAINS)),
      yawRate_(makeRatePid(YAW_RATE_GAINS)) {}

ActuatorCommand AttitudeController::update(const AttitudeSetpoint& sp,
                                           const Attitude& att, float dt) {
    // Yaw is continuous in the estimator; steer the shortest way to the
    // target heading by expressing the setpoint relative to the current yaw.
    const float yawSp = att.yaw + wrap180(sp.yaw - att.yaw);

    const float rollRateSp  = rollAngle_.update(sp.roll, att.roll, dt);
    const float pitchRateSp = pitchAngle_.update(sp.pitch, att.pitch, dt);
    const float yawRateSp   = yawAngle_.update(yawSp, att.yaw, dt);

    ActuatorCommand cmd;
    cmd.roll  = rollRate_.update(rollRateSp, att.rollRate, dt);
    cmd.pitch = pitchRate_.update(pitchRateSp, att.pitchRate, dt);
    cmd.yaw   = yawRate_.update(yawRateSp, att.yawRate, dt);
    return cmd;
}

void AttitudeController::reset() {
    rollAngle_.reset();
    pitchAngle_.reset();
    yawAngle_.reset();
    rollRate_.reset();
    pitchRate_.reset();
    yawRate_.reset();
}
