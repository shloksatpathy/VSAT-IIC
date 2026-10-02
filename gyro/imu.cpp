#include "imu.h"

#include <Arduino.h>
#include <Wire.h>

namespace {

constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;

constexpr float GYRO_LSB_PER_DPS = 65.5f;    // +/-500 deg/s
constexpr float ACCEL_LSB_PER_G  = 4096.0f;  // +/-8 g

}  // namespace

Imu::Imu(uint8_t address) : address_(address) {}

bool Imu::begin() {
    if (!writeRegister(REG_PWR_MGMT_1, 0x01)) return false;  // wake, PLL on gyro X
    delay(100);
    if (!writeRegister(REG_CONFIG, 0x03)) return false;        // DLPF ~44 Hz
    if (!writeRegister(REG_GYRO_CONFIG, 0x08)) return false;   // +/-500 deg/s
    if (!writeRegister(REG_ACCEL_CONFIG, 0x10)) return false;  // +/-8 g
    return true;
}

bool Imu::calibrateGyro(uint16_t samples) {
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    uint16_t good = 0;
    int16_t raw[7];

    for (uint16_t i = 0; i < samples; ++i) {
        if (readRaw(raw)) {
            sumX += raw[4] / GYRO_LSB_PER_DPS;
            sumY += raw[5] / GYRO_LSB_PER_DPS;
            sumZ += raw[6] / GYRO_LSB_PER_DPS;
            ++good;
        }
        delay(2);
    }

    if (good < samples / 2) return false;

    biasX_ = sumX / good;
    biasY_ = sumY / good;
    biasZ_ = sumZ / good;
    return true;
}

bool Imu::read(ImuSample& out) {
    int16_t raw[7];
    if (!readRaw(raw)) return false;

    out.ax = raw[0] / ACCEL_LSB_PER_G;
    out.ay = raw[1] / ACCEL_LSB_PER_G;
    out.az = raw[2] / ACCEL_LSB_PER_G;
    // raw[3] is temperature (unused)
    out.gx = raw[4] / GYRO_LSB_PER_DPS - biasX_;
    out.gy = raw[5] / GYRO_LSB_PER_DPS - biasY_;
    out.gz = raw[6] / GYRO_LSB_PER_DPS - biasZ_;
    return true;
}

bool Imu::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool Imu::readRaw(int16_t raw[7]) {
    Wire.beginTransmission(address_);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom(address_, static_cast<uint8_t>(14)) != 14) return false;

    for (uint8_t i = 0; i < 7; ++i) {
        const uint8_t hi = Wire.read();
        const uint8_t lo = Wire.read();
        raw[i] = static_cast<int16_t>((hi << 8) | lo);
    }
    return true;
}
