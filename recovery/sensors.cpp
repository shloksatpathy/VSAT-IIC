#include "sensors.h"

#include <Adafruit_BMP280.h>
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <math.h>

#include "config.h"

namespace {

Adafruit_BMP280 bmp;
TinyGPSPlus     gps;

constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;

constexpr float GYRO_LSB_PER_DPS = 65.5f;    // +/-500 deg/s
constexpr float ACCEL_LSB_PER_G  = 2048.0f;  // +/-16 g (ejection and landing shocks)

constexpr float BARO_MIN_VALID_M = -200.0f;
constexpr float BARO_MAX_VALID_M = 5000.0f;

bool imuWrite(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool imuReadRaw(int16_t raw[7]) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(IMU_I2C_ADDR, static_cast<uint8_t>(14)) != 14) return false;
    for (uint8_t i = 0; i < 7; ++i) {
        const uint8_t hi = Wire.read();
        const uint8_t lo = Wire.read();
        raw[i] = static_cast<int16_t>((hi << 8) | lo);
    }
    return true;
}

}  // namespace

bool Sensors::begin() {
    GNSS_SERIAL.begin(GNSS_BAUD);

    baroOk_ = bmp.begin(BARO_I2C_ADDR);
    if (baroOk_) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_X4,
                        Adafruit_BMP280::STANDBY_MS_1);
        delay(100);
        float sum = 0.0f;
        for (uint8_t i = 0; i < 50; ++i) {
            sum += bmp.readPressure() / 100.0f;
            delay(10);
        }
        groundPressureHpa_ = sum / 50.0f;
    }

    imuOk_ = imuWrite(REG_PWR_MGMT_1, 0x01);
    delay(100);
    imuOk_ = imuOk_ && imuWrite(REG_CONFIG, 0x03)         // DLPF ~44 Hz
                    && imuWrite(REG_GYRO_CONFIG, 0x08)    // +/-500 deg/s
                    && imuWrite(REG_ACCEL_CONFIG, 0x18);  // +/-16 g
    if (imuOk_) {
        float sum[3] = {0.0f, 0.0f, 0.0f};
        uint16_t good = 0;
        int16_t raw[7];
        for (uint16_t i = 0; i < IMU_CALIB_SAMPLES; ++i) {
            if (imuReadRaw(raw)) {
                for (uint8_t a = 0; a < 3; ++a) sum[a] += raw[4 + a] / GYRO_LSB_PER_DPS;
                ++good;
            }
            delay(2);
        }
        if (good > 0) {
            for (uint8_t a = 0; a < 3; ++a) gyroBias_[a] = sum[a] / good;
        }
    }

    return baroOk_ && imuOk_;
}

void Sensors::pollGnss(uint32_t nowMs) {
    while (GNSS_SERIAL.available()) {
        gps.encode(GNSS_SERIAL.read());
    }

    if (gps.location.isUpdated() && gps.location.isValid() &&
        gps.satellites.isValid() && gps.satellites.value() >= GNSS_MIN_SATS) {
        lat_           = gps.location.lat();
        lon_           = gps.location.lng();
        lastGnssFixMs_ = nowMs;
        newFix_        = true;
        hasFix_        = true;

        gnssAltValid_ = gps.altitude.isValid();
        if (gnssAltValid_) gnssAltMsl_ = gps.altitude.meters();

        // Latch the ground GNSS altitude on the pad so GNSS altitude can be
        // expressed AGL. Requires the barometer to confirm we are still low.
        if (!groundGnssSet_ && gnssAltValid_ && baroValid_ && baroAgl_ < GROUND_LOCK_MAX_AGL_M) {
            groundGnssMsl_ = gnssAltMsl_ - baroAgl_;
            groundGnssSet_ = true;
        }
    }
}

void Sensors::update(uint32_t nowMs) {
    if (baroOk_) {
        const float alt = bmp.readAltitude(groundPressureHpa_);
        baroValid_ = !isnan(alt) && alt > BARO_MIN_VALID_M && alt < BARO_MAX_VALID_M;
        if (baroValid_) baroAgl_ = alt;
    }

    if (imuOk_) readImu(nowMs);
}

bool Sensors::readImu(uint32_t nowMs) {
    int16_t raw[7];
    if (!imuReadRaw(raw)) return false;

    const float ax = raw[0] / ACCEL_LSB_PER_G;
    const float ay = raw[1] / ACCEL_LSB_PER_G;
    const float az = raw[2] / ACCEL_LSB_PER_G;
    accelMagG_ = sqrtf(ax * ax + ay * ay + az * az);

    const float gx = raw[4] / GYRO_LSB_PER_DPS - gyroBias_[0];
    const float gy = raw[5] / GYRO_LSB_PER_DPS - gyroBias_[1];
    const float gz = raw[6] / GYRO_LSB_PER_DPS - gyroBias_[2];
    yawRateDps_ = HEADING_GYRO_SIGN * gz;
    maxRateDps_ = fmaxf(fabsf(gx), fmaxf(fabsf(gy), fabsf(gz)));

    lastImuMs_ = nowMs;
    hasImu_    = true;
    return true;
}

float Sensors::baroAltitudeAgl() const {
    return baroValid_ ? baroAgl_ : NAN;
}

float Sensors::gnssAltitudeAgl(uint32_t nowMs) const {
    if (!gnssAltValid_ || !groundGnssSet_ || !gnssFresh(nowMs)) return NAN;
    return gnssAltMsl_ - groundGnssMsl_;
}

bool Sensors::takeNewFix() {
    const bool fix = newFix_;
    newFix_ = false;
    return fix;
}

bool Sensors::gnssFresh(uint32_t nowMs) const {
    return hasFix_ && (nowMs - lastGnssFixMs_) <= SENSOR_STALE_MS;
}

bool Sensors::imuFresh(uint32_t nowMs) const {
    return hasImu_ && (nowMs - lastImuMs_) <= SENSOR_STALE_MS;
}
