// Tunable parameters for the gyro attitude-control system.
// All angles are in degrees, rates in deg/s, actuator commands in [-1, 1].
#pragma once

#include <stdint.h>

// ---------------------------------------------------------------------------
// Loop timing
// ---------------------------------------------------------------------------
constexpr float    LOOP_HZ        = 200.0f;
constexpr uint32_t LOOP_PERIOD_US = static_cast<uint32_t>(1000000.0f / LOOP_HZ);
constexpr float    DEBUG_HZ       = 20.0f;

// ---------------------------------------------------------------------------
// IMU (MPU6050 / MPU6500-class, I2C)
// ---------------------------------------------------------------------------
constexpr uint8_t  IMU_I2C_ADDR        = 0x68;
constexpr uint32_t IMU_I2C_CLOCK_HZ    = 400000;
constexpr uint16_t IMU_CALIB_SAMPLES   = 1000;  // payload must be still during calibration
constexpr uint8_t  IMU_MAX_READ_FAILS  = 10;    // consecutive failures before failsafe

// ---------------------------------------------------------------------------
// Attitude estimator (complementary filter)
// ---------------------------------------------------------------------------
// Weight on the gyro-integrated angle. Higher = smoother, slower drift correction.
constexpr float COMP_ALPHA = 0.98f;
// Ignore the accelerometer when |a| deviates from 1 g by more than this
// (parachute deployment shocks, swinging under canopy).
constexpr float ACCEL_TRUST_BAND_G = 0.15f;

// ---------------------------------------------------------------------------
// Cascaded PID gains
//   Outer loop: angle error (deg)   -> rate setpoint (deg/s)
//   Inner loop: rate error  (deg/s) -> actuator command [-1, 1]
// Start with the inner loop only (outer P = 0), tune it, then add the outer loop.
// ---------------------------------------------------------------------------
struct PidGains { float kp, ki, kd; };

constexpr PidGains ROLL_ANGLE_GAINS  = { 4.0f,   0.0f,   0.0f   };
constexpr PidGains PITCH_ANGLE_GAINS = { 4.0f,   0.0f,   0.0f   };
constexpr PidGains YAW_ANGLE_GAINS   = { 2.0f,   0.0f,   0.0f   };

constexpr PidGains ROLL_RATE_GAINS   = { 0.010f, 0.002f, 0.0004f };
constexpr PidGains PITCH_RATE_GAINS  = { 0.010f, 0.002f, 0.0004f };
constexpr PidGains YAW_RATE_GAINS    = { 0.015f, 0.003f, 0.0f    };

constexpr float MAX_RATE_SETPOINT_DPS = 90.0f;  // clamp on outer-loop output
constexpr float D_TERM_CUTOFF_HZ      = 20.0f;  // low-pass on the derivative term

// ---------------------------------------------------------------------------
// Actuators
// ---------------------------------------------------------------------------
// Gimbal servos tilt the spinning flywheel (control moment gyro); the yaw servo
// steers the glider parachute. Remap in actuators.cpp to match the hardware.
constexpr uint8_t PIN_GIMBAL_ROLL_SERVO  = 9;
constexpr uint8_t PIN_GIMBAL_PITCH_SERVO = 10;
constexpr uint8_t PIN_YAW_SERVO          = 11;
constexpr uint8_t PIN_FLYWHEEL_ESC       = 6;

constexpr uint16_t SERVO_CENTER_US = 1500;
constexpr uint16_t SERVO_RANGE_US  = 400;   // +/- from centre at command = +/-1
constexpr uint16_t ESC_MIN_US      = 1000;  // flywheel stopped
constexpr uint16_t ESC_RUN_US      = 1600;  // flywheel running speed
constexpr uint32_t FLYWHEEL_SPINUP_MS = 3000;

// Flip to -1 if an axis moves the wrong way during bench testing.
constexpr int8_t ROLL_DIR  = 1;
constexpr int8_t PITCH_DIR = 1;
constexpr int8_t YAW_DIR   = 1;
