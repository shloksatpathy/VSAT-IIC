// Brake servo bench test for ESP32-S3.
// Pins and end points copied from recovery/config.h.
// Cycles: left pulls, left releases, right pulls, right releases.

#include <ESP32Servo.h>

constexpr uint8_t  PIN_LEFT_BRAKE         = 12;
constexpr uint16_t LEFT_BRAKE_NEUTRAL_US  = 1100;
constexpr uint16_t LEFT_BRAKE_FULL_US     = 1900;
constexpr uint8_t  PIN_RIGHT_BRAKE        = 13;
constexpr uint16_t RIGHT_BRAKE_NEUTRAL_US = 1900;
constexpr uint16_t RIGHT_BRAKE_FULL_US    = 1100;

constexpr int8_t PIN_SERVO_POWER = 7;  // servo supply MOSFET, HIGH = on; -1 = not fitted

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
    Serial.print("L:");
    Serial.print(left, 2);
    Serial.print(" R:");
    Serial.println(right, 2);
}

void setup() {
    Serial.begin(115200);

    if (PIN_SERVO_POWER >= 0) {
        pinMode(PIN_SERVO_POWER, OUTPUT);
        digitalWrite(PIN_SERVO_POWER, HIGH);
    }

    leftBrake.attach(PIN_LEFT_BRAKE);
    rightBrake.attach(PIN_RIGHT_BRAKE);
    setBrakes(0.0f, 0.0f);
    delay(2000);
}

void loop() {
    setBrakes(1.0f, 0.0f);
    delay(2000);
    setBrakes(0.0f, 0.0f);
    delay(2000);
    setBrakes(0.0f, 1.0f);
    delay(2000);
    setBrakes(0.0f, 0.0f);
    delay(2000);
}
