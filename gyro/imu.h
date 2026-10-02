// Minimal I2C driver for an MPU6050 / MPU6500-class 6-axis IMU.
#pragma once

#include <stdint.h>

struct ImuSample {
    float ax, ay, az;  // acceleration, g
    float gx, gy, gz;  // angular rate, deg/s (bias removed)
};

class Imu {
public:
    explicit Imu(uint8_t address);

    // Wakes the sensor and sets ranges (+/-500 deg/s, +/-8 g, ~44 Hz DLPF).
    bool begin();

    // Averages gyro output at rest to find the bias. Keep the payload still.
    bool calibrateGyro(uint16_t samples);

    // Reads one sample. Returns false on an I2C error.
    bool read(ImuSample& out);

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRaw(int16_t raw[7]);

    uint8_t address_;
    float   biasX_ = 0.0f, biasY_ = 0.0f, biasZ_ = 0.0f;
};
