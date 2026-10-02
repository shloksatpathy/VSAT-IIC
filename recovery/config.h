// Tunable parameters for secondary recovery and navigation.
// Altitudes are metres above ground level (AGL) at the launch site, angles are
// degrees, bearings are measured clockwise from true north.
#pragma once

#include <stdint.h>

// ---------------------------------------------------------------------------
// Mission
// ---------------------------------------------------------------------------
// Landing target. Set these before every flight.
constexpr double TARGET_LAT = 0.0;
constexpr double TARGET_LON = 0.0;

constexpr float EJECTION_ALTITUDE_M = 1000.0f;  // nominal; seeds dead reckoning
constexpr float CUT_ALTITUDE_M      = 600.0f;   // primary parachute cut
constexpr uint8_t CUT_CONFIRM_SAMPLES = 3;      // consecutive readings below cut altitude
constexpr float FLARE_ALTITUDE_M    = 8.0f;

// ---------------------------------------------------------------------------
// Ejection detection (whichever triggers first)
// ---------------------------------------------------------------------------
constexpr int8_t PIN_EJECTION_SWITCH     = -1;     // breakaway switch, active LOW; -1 = not fitted
constexpr float  EJECTION_MIN_ALTITUDE_M = 500.0f; // apogee must be above this
constexpr float  APOGEE_DROP_M           = 10.0f;  // drop below max altitude that confirms apogee

// ---------------------------------------------------------------------------
// Altitude fusion
// ---------------------------------------------------------------------------
constexpr float BARO_GNSS_MAX_DIFF_M     = 30.0f;  // larger disagreement -> distrust baro
constexpr float PRIMARY_DESCENT_RATE_MPS = 20.0f;  // from drop test
constexpr float CANOPY_DESCENT_RATE_MPS  = 2.0f;   // from drop test
constexpr float DESCENT_RATE_TAU_S       = 0.5f;   // low-pass time constant
constexpr float GROUND_LOCK_MAX_AGL_M    = 20.0f;  // GNSS ground altitude only latched near the pad

// ---------------------------------------------------------------------------
// Stage 2: separation confirmation
// ---------------------------------------------------------------------------
constexpr float    FREEFALL_G            = 0.3f;
constexpr uint32_t FREEFALL_CONFIRM_MS   = 150;
constexpr uint32_t SEPARATION_TIMEOUT_MS = 2000;

// ---------------------------------------------------------------------------
// Stage 3: canopy deployment
// ---------------------------------------------------------------------------
constexpr float    INFLATED_DESCENT_RATE_MPS = 3.0f;  // canopy flying at 1-3 m/s
constexpr uint32_t INFLATION_CONFIRM_MS      = 1000;
constexpr uint32_t INFLATION_TIMEOUT_MS      = 8000;
constexpr float    SWING_SETTLED_RATE_DPS    = 30.0f; // max body rate when settled
constexpr uint32_t SWING_SETTLE_MS           = 1500;
constexpr uint32_t SWING_TIMEOUT_MS          = 5000;

// ---------------------------------------------------------------------------
// Stage 4: guidance
// ---------------------------------------------------------------------------
constexpr float CANOPY_MAX_GLIDE_RATIO = 2.5f;  // from drop test
constexpr float CANOPY_AIRSPEED_MPS    = CANOPY_MAX_GLIDE_RATIO * CANOPY_DESCENT_RATE_MPS;
constexpr float MIN_EFFECTIVE_GLIDE_RATIO = 0.3f;
constexpr float TOO_HIGH_MARGIN        = 0.8f;  // required < effective * margin -> TOO_HIGH

// Heading PID: error in degrees -> brake command in [-1, 1].
constexpr float HEADING_KP             = 0.015f;
constexpr float HEADING_KI             = 0.0005f;
constexpr float HEADING_KD             = 0.003f;
constexpr float HEADING_INTEGRAL_LIMIT = 200.0f;  // deg*s

constexpr float TURN_BOOST_FACTOR = 1.5f;   // TOO_HIGH: tighter turns burn altitude
constexpr float MINIMAL_FACTOR    = 0.3f;   // TOO_LOW: gentle redirect only
constexpr float LARGE_ERROR_DEG   = 45.0f;  // TOO_LOW: below this, fly straight

// Heading estimate: gyro yaw rate, corrected toward the GNSS ground track.
constexpr float HEADING_GYRO_SIGN      = -1.0f;  // MPU6050 z-up: +gz is counter-clockwise
constexpr float HEADING_GNSS_WEIGHT    = 0.2f;   // blend per ground-track update
constexpr float MIN_TRACK_BASELINE_M   = 5.0f;   // distance between fixes for a track
constexpr float WIND_FILTER_ALPHA      = 0.2f;   // per ground-track update

constexpr uint32_t SENSOR_STALE_MS = 1000;

// ---------------------------------------------------------------------------
// Stage 5: landing detection
// ---------------------------------------------------------------------------
constexpr float    LANDING_ACCEL_G         = 3.0f;
constexpr float    LANDED_DESCENT_RATE_MPS = 0.3f;
constexpr uint32_t LANDED_CONFIRM_MS       = 2000;

// ---------------------------------------------------------------------------
// Loop timing and logging
// ---------------------------------------------------------------------------
constexpr float    LOOP_HZ        = 50.0f;
constexpr uint32_t LOOP_PERIOD_US = static_cast<uint32_t>(1000000.0f / LOOP_HZ);
constexpr float    TELEMETRY_HZ   = 5.0f;

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
#define GNSS_SERIAL Serial1
constexpr uint32_t GNSS_BAUD          = 9600;
constexpr uint8_t  GNSS_MIN_SATS      = 4;
constexpr uint8_t  BARO_I2C_ADDR      = 0x76;  // BMP280
constexpr uint8_t  IMU_I2C_ADDR       = 0x68;  // MPU6050
constexpr uint32_t I2C_CLOCK_HZ       = 400000;
constexpr uint16_t IMU_CALIB_SAMPLES  = 500;

// ---------------------------------------------------------------------------
// Actuators
// ---------------------------------------------------------------------------
constexpr uint8_t  PIN_PRIMARY_CUTTER = 4;     // nichrome / pyro MOSFET gate
constexpr uint32_t CUTTER_PULSE_MS    = 1500;

constexpr uint8_t  PIN_CANOPY_RELEASE  = 5;
constexpr uint16_t CANOPY_LOCKED_US    = 1000;
constexpr uint16_t CANOPY_RELEASED_US  = 2000;

// Brake servos. Swap NEUTRAL/FULL values for a mirrored servo.
constexpr uint8_t  PIN_LEFT_BRAKE         = 12;
constexpr uint16_t LEFT_BRAKE_NEUTRAL_US  = 1100;
constexpr uint16_t LEFT_BRAKE_FULL_US     = 1900;
constexpr uint8_t  PIN_RIGHT_BRAKE        = 13;
constexpr uint16_t RIGHT_BRAKE_NEUTRAL_US = 1900;
constexpr uint16_t RIGHT_BRAKE_FULL_US    = 1100;

constexpr int8_t PIN_SERVO_POWER = 7;  // servo supply MOSFET, HIGH = on; -1 = not fitted
