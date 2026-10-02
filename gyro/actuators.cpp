#include "actuators.h"

#include <Arduino.h>
// On ESP32, install the ESP32Servo library and include <ESP32Servo.h> instead.
#include <Servo.h>

#include "config.h"

namespace {

Servo gimbalRoll;
Servo gimbalPitch;
Servo yawServo;
Servo flywheelEsc;

uint16_t commandToUs(float cmd, int8_t dir) {
    if (cmd > 1.0f)  cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
    return static_cast<uint16_t>(SERVO_CENTER_US + dir * cmd * SERVO_RANGE_US);
}

}  // namespace

void Actuators::begin() {
    gimbalRoll.attach(PIN_GIMBAL_ROLL_SERVO);
    gimbalPitch.attach(PIN_GIMBAL_PITCH_SERVO);
    yawServo.attach(PIN_YAW_SERVO);
    flywheelEsc.attach(PIN_FLYWHEEL_ESC);

    flywheelEsc.writeMicroseconds(ESC_MIN_US);  // arm the ESC at zero throttle
    centre();
}

void Actuators::startFlywheel() {
    flywheelEsc.writeMicroseconds(ESC_RUN_US);
    delay(FLYWHEEL_SPINUP_MS);
}

void Actuators::stopFlywheel() {
    flywheelEsc.writeMicroseconds(ESC_MIN_US);
}

void Actuators::write(const ActuatorCommand& cmd) {
    // Axis mapping: a gimbal tilt about one axis makes the spinning flywheel
    // precess and produce torque about the perpendicular axis. Swap the
    // roll/pitch assignments here if the bench test shows that coupling.
    gimbalRoll.writeMicroseconds(commandToUs(cmd.roll, ROLL_DIR));
    gimbalPitch.writeMicroseconds(commandToUs(cmd.pitch, PITCH_DIR));
    yawServo.writeMicroseconds(commandToUs(cmd.yaw, YAW_DIR));
}

void Actuators::centre() {
    gimbalRoll.writeMicroseconds(SERVO_CENTER_US);
    gimbalPitch.writeMicroseconds(SERVO_CENTER_US);
    yawServo.writeMicroseconds(SERVO_CENTER_US);
}
