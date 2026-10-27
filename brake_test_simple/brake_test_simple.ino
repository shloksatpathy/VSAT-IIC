// Brake servo + MPU test for ESP32-S3.
// Pins and end points copied from recovery/config.h.
// Roll from the raw accelerometer drives the brakes: tilt left pulls the
// left brake, tilt right pulls the right brake. No filtering or tuning.

#include <Wire.h>
#include <ESP32Servo.h>

constexpr uint8_t  PIN_LEFT_BRAKE         = 12;
constexpr uint16_t LEFT_BRAKE_NEUTRAL_US  = 1100;
constexpr uint16_t LEFT_BRAKE_FULL_US     = 1900;
constexpr uint8_t  PIN_RIGHT_BRAKE        = 13;
constexpr uint16_t RIGHT_BRAKE_NEUTRAL_US = 1900;
constexpr uint16_t RIGHT_BRAKE_FULL_US    = 1100;

constexpr int8_t PIN_SERVO_POWER = 7;  // servo supply MOSFET, HIGH = on; -1 = not fitted

constexpr uint8_t IMU_I2C_ADDR    = 0x68;
constexpr uint8_t REG_PWR_MGMT_1  = 0x6B;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

constexpr float FULL_BRAKE_ROLL_DEG = 45.0f;  // roll at which a brake is fully pulled

Servo leftBrake;
Servo rightBrake;

uint16_t fractionToUs(float f, uint16_t neutralUs, uint16_t fullUs) {
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    return static_cast<uint16_t>(neutralUs + f * (static_cast<float>(fullUs) - neutralUs));
}

void setBrakes(float left, float right) {
    leftBrake.writeMicroseconds(fractionToUs(left, LEFT_BRAKE_NEUTRAL_US, LEFT_BRAKE_FULL_US));
    rightBrake.writeMicroseconds(fractionToUs(right, RIGHT_BRAKE_NEUTRAL_US, RIGHT_BRAKE_FULL_US));
}

float readRollDeg() {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_I2C_ADDR, static_cast<uint8_t>(6));
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    (void)ax;
    return atan2f(ay, az) * RAD_TO_DEG;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Wake the MPU (defaults: +/-2 g accel)
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01);
    Wire.endTransmission();

    if (PIN_SERVO_POWER >= 0) {
        pinMode(PIN_SERVO_POWER, OUTPUT);
        digitalWrite(PIN_SERVO_POWER, HIGH);
    }

    leftBrake.attach(PIN_LEFT_BRAKE);
    rightBrake.attach(PIN_RIGHT_BRAKE);
    setBrakes(0.0f, 0.0f);
    delay(1000);
}

void loop() {
    float roll = readRollDeg();
    float left  = -roll / FULL_BRAKE_ROLL_DEG;
    float right =  roll / FULL_BRAKE_ROLL_DEG;
    setBrakes(left, right);

    Serial.print("Roll:");
    Serial.print(roll, 1);
    Serial.print(" L:");
    Serial.print(left < 0 ? 0 : left, 2);
    Serial.print(" R:");
    Serial.println(right < 0 ? 0 : right, 2);
    delay(20);
}
