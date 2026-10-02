#include "actuators.h"

#include <Arduino.h>
// On ESP32, install the ESP32Servo library and include <ESP32Servo.h> instead.
#include <Servo.h>

#include "config.h"

namespace {

Servo canopyRelease;
Servo leftBrake;
Servo rightBrake;

uint16_t fractionToUs(float f, uint16_t neutralUs, uint16_t fullUs) {
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    return static_cast<uint16_t>(neutralUs + f * (static_cast<float>(fullUs) - neutralUs));
}

}  // namespace

void RecoveryActuators::begin() {
    pinMode(PIN_PRIMARY_CUTTER, OUTPUT);
    digitalWrite(PIN_PRIMARY_CUTTER, LOW);

    if (PIN_SERVO_POWER >= 0) {
        pinMode(PIN_SERVO_POWER, OUTPUT);
        digitalWrite(PIN_SERVO_POWER, HIGH);
    }

    canopyRelease.attach(PIN_CANOPY_RELEASE);
    leftBrake.attach(PIN_LEFT_BRAKE);
    rightBrake.attach(PIN_RIGHT_BRAKE);
    servosPowered_ = true;

    canopyRelease.writeMicroseconds(CANOPY_LOCKED_US);
    brakesNeutral();
}

void RecoveryActuators::update(uint32_t nowMs) {
    if (cutterOn_ && nowMs - cutterStartMs_ >= CUTTER_PULSE_MS) {
        digitalWrite(PIN_PRIMARY_CUTTER, LOW);
        cutterOn_ = false;
    }
}

void RecoveryActuators::cutPrimary(uint32_t nowMs) {
    digitalWrite(PIN_PRIMARY_CUTTER, HIGH);
    cutterOn_      = true;
    cutterStartMs_ = nowMs;
}

void RecoveryActuators::releaseCanopy() {
    if (servosPowered_) canopyRelease.writeMicroseconds(CANOPY_RELEASED_US);
}

void RecoveryActuators::setBrakes(float left, float right) {
    if (!servosPowered_) return;
    leftBrake.writeMicroseconds(fractionToUs(left, LEFT_BRAKE_NEUTRAL_US, LEFT_BRAKE_FULL_US));
    rightBrake.writeMicroseconds(fractionToUs(right, RIGHT_BRAKE_NEUTRAL_US, RIGHT_BRAKE_FULL_US));
}

void RecoveryActuators::cutServoPower() {
    canopyRelease.detach();
    leftBrake.detach();
    rightBrake.detach();
    if (PIN_SERVO_POWER >= 0) digitalWrite(PIN_SERVO_POWER, LOW);
    servosPowered_ = false;
}
