// Brake servo + MPU test for ESP32-S3.
// Pins and end points copied from recovery/config.h.
// Filtered roll drives the brakes: tilt left pulls the left brake, tilt right
// pulls the right brake. GNSS steering toward the reference location (gnss.cpp)
// is added on top. Keep the board still and level during startup
// calibration; that position becomes zero roll.

#include <Wire.h>
#include <ESP32Servo.h>

#include "gnss.h"

constexpr uint8_t  PIN_LEFT_BRAKE         = 12;
constexpr uint16_t LEFT_BRAKE_NEUTRAL_US  = 1100;
constexpr uint16_t LEFT_BRAKE_FULL_US     = 1900;
constexpr uint8_t  PIN_RIGHT_BRAKE        = 13;
constexpr uint16_t RIGHT_BRAKE_NEUTRAL_US = 1900;
constexpr uint16_t RIGHT_BRAKE_FULL_US    = 1100;

constexpr int8_t PIN_SERVO_POWER = 7;  // servo supply MOSFET, HIGH = on; -1 = not fitted

constexpr uint8_t IMU_I2C_ADDR     = 0x68;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;

constexpr float GYRO_LSB_PER_DPS = 65.5f;  // +/-500 deg/s

// Tuning
constexpr float    FILTER_ALPHA        = 0.98f;  // complementary filter: gyro weight (0..1)
constexpr float    DEADBAND_DEG        = 2.0f;   // roll ignored inside +/- this
constexpr float    FULL_BRAKE_ROLL_DEG = 45.0f;  // roll (past the deadband) at which a brake is fully pulled
constexpr uint16_t CALIB_SAMPLES       = 500;
constexpr uint32_t LOOP_PERIOD_US      = 10000;  // 100 Hz
constexpr uint32_t PRINT_PERIOD_MS     = 100;    // 10 Hz

Servo leftBrake;
Servo rightBrake;

float gyroBiasDps = 0.0f;
float rollTrimDeg = 0.0f;
float rollDeg     = 0.0f;
uint32_t lastUs   = 0;
uint32_t lastPrintMs = 0;

uint16_t fractionToUs(float f, uint16_t neutralUs, uint16_t fullUs) {
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    return static_cast<uint16_t>(neutralUs + f * (static_cast<float>(fullUs) - neutralUs));
}

void setBrakes(float left, float right) {
    leftBrake.writeMicroseconds(fractionToUs(left, LEFT_BRAKE_NEUTRAL_US, LEFT_BRAKE_FULL_US));
    rightBrake.writeMicroseconds(fractionToUs(right, RIGHT_BRAKE_NEUTRAL_US, RIGHT_BRAKE_FULL_US));
}

void imuWrite(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// Roll from the accelerometer (deg) and roll rate from the gyro (deg/s).
void readImu(float &accelRollDeg, float &gyroRateDps) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_I2C_ADDR, static_cast<uint8_t>(14));
    int16_t raw[7];
    for (uint8_t i = 0; i < 7; ++i) raw[i] = (Wire.read() << 8) | Wire.read();
    accelRollDeg = atan2f(raw[1], raw[2]) * RAD_TO_DEG;  // ay, az
    gyroRateDps  = raw[4] / GYRO_LSB_PER_DPS;            // gx
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    imuWrite(REG_PWR_MGMT_1, 0x01);   // wake, gyro clock
    delay(100);
    imuWrite(REG_CONFIG, 0x03);       // DLPF ~44 Hz
    imuWrite(REG_GYRO_CONFIG, 0x08);  // +/-500 deg/s

    if (PIN_SERVO_POWER >= 0) {
        pinMode(PIN_SERVO_POWER, OUTPUT);
        digitalWrite(PIN_SERVO_POWER, HIGH);
    }

    leftBrake.attach(PIN_LEFT_BRAKE);
    rightBrake.attach(PIN_RIGHT_BRAKE);
    setBrakes(0.0f, 0.0f);

    Serial.println("Calibrating - keep the board still and level");
    float rollSum = 0.0f;
    float rateSum = 0.0f;
    for (uint16_t i = 0; i < CALIB_SAMPLES; ++i) {
        float accelRoll, rate;
        readImu(accelRoll, rate);
        rollSum += accelRoll;
        rateSum += rate;
        delay(2);
    }
    rollTrimDeg = rollSum / CALIB_SAMPLES;
    gyroBiasDps = rateSum / CALIB_SAMPLES;
    Serial.print("Roll trim:");
    Serial.print(rollTrimDeg, 2);
    Serial.print(" Gyro bias:");
    Serial.println(gyroBiasDps, 3);

    gnssBegin();
    lastUs = micros();
}

void loop() {
    gnssUpdate();

    uint32_t now = micros();
    if (now - lastUs < LOOP_PERIOD_US) return;
    float dt = (now - lastUs) * 1e-6f;
    lastUs = now;

    float accelRoll, rate;
    readImu(accelRoll, rate);
    accelRoll -= rollTrimDeg;
    rate      -= gyroBiasDps;

    // Complementary filter: gyro for fast changes, accelerometer to stop drift.
    rollDeg = FILTER_ALPHA * (rollDeg + rate * dt) + (1.0f - FILTER_ALPHA) * accelRoll;

    float cmd = 0.0f;
    if (rollDeg > DEADBAND_DEG)  cmd = (rollDeg - DEADBAND_DEG) / FULL_BRAKE_ROLL_DEG;
    if (rollDeg < -DEADBAND_DEG) cmd = (rollDeg + DEADBAND_DEG) / FULL_BRAKE_ROLL_DEG;
    cmd += gnssSteer();
    float left  = -cmd;
    float right =  cmd;
    setBrakes(left, right);

    if (millis() - lastPrintMs < PRINT_PERIOD_MS) return;
    lastPrintMs = millis();
    Serial.print("Acc:");
    Serial.print(accelRoll, 1);
    Serial.print(" Roll:");
    Serial.print(rollDeg, 1);
    Serial.print(" L:");
    Serial.print(left < 0 ? 0 : left, 2);
    Serial.print(" R:");
    Serial.print(right < 0 ? 0 : right, 2);
    gnssPrint();
    Serial.println();
}
