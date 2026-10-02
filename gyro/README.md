# Gyro Attitude Control

PID-based attitude control for CAN7U-SAT. An IMU measures the payload's orientation, a complementary filter turns that into roll/pitch/yaw, and a cascaded PID controller drives the actuators to hold the commanded attitude.

The actuation follows the active-gyroscope approach shown in [James Bruton's "How this Active Gyroscope Balances"](https://youtu.be/UVJx8T8wTQA). A spinning flywheel sits in a servo-driven gimbal. Tilting the gimbal makes the flywheel precess, and that precession puts a torque on the body. The controller steers that torque to keep the payload stable. A separate yaw servo steers the glider parachute toward the target heading.

## Control Pipeline

```
 IMU (gyro + accel)
        │  200 Hz
        ▼
 Complementary filter ──► roll, pitch, yaw + body rates
        │
        ▼
 Angle PID (outer) ──► rate setpoint (deg/s, clamped)
        │
        ▼
 Rate PID (inner)  ──► actuator command [-1, 1]
        │
        ▼
 Gimbal servos (CMG) + parachute yaw servo
```

## Files

| File | Purpose |
| --- | --- |
| `gyro.ino` | Main loop: setup, fixed-rate control loop, IMU failsafe, debug output |
| `config.h` | All tunable values: loop rate, filter weight, PID gains, pins, servo limits |
| `pid.h/.cpp` | PID controller with derivative-on-measurement, filtered D term, anti-windup |
| `imu.h/.cpp` | MPU6050/MPU6500 I2C driver with gyro bias calibration |
| `attitude_estimator.h/.cpp` | Complementary filter for roll/pitch/yaw |
| `attitude_controller.h/.cpp` | Cascaded angle → rate PID for all three axes |
| `actuators.h/.cpp` | Gimbal servos, yaw servo and flywheel ESC output |

## Design Notes

- **Cascaded PID.** The inner rate loop uses the gyro directly, so it reacts fast and adds damping. The outer angle loop corrects slow attitude error. Tune the inner loop first.
- **Derivative on measurement.** When the setpoint changes suddenly, the D term doesn't spike.
- **Anti-windup.** The integrator stops building up while the output is saturated, so the controller doesn't overshoot when the actuators come out of saturation.
- **Accelerometer gating.** The filter only uses the accelerometer when it reads about 1 g. Parachute-deployment shocks and swinging under the canopy would otherwise corrupt the roll and pitch estimate.
- **Yaw drift.** Yaw comes from the gyro alone, so it drifts. `AttitudeEstimator::correctYaw()` can pull it toward the GNSS ground-track heading once that integration is ready.
- **Failsafe.** If the IMU fails `IMU_MAX_READ_FAILS` reads in a row, every servo is centred and the PID state is reset. Control resumes cleanly once the IMU recovers.

## Building

This code uses the Arduino framework (ESP32, Teensy, STM32 or AVR) until the final MCU is chosen.

1. Open `gyro/gyro.ino` in the Arduino IDE. The folder name must match the sketch name.
2. Install a servo library. Use `Servo` on most boards. On ESP32, install `ESP32Servo` and change the include in `actuators.cpp`.
3. Set the pins in `config.h` to match your wiring, then build and flash.

## Tuning Procedure

1. **Bench setup:** mount the payload on a single-axis test rig. Keep props and parachute lines clear.
2. **Check directions:** with the flywheel spinning, tilt the payload by hand. The gimbal should push back against the tilt. If it doesn't, flip `ROLL_DIR`, `PITCH_DIR` or `YAW_DIR`.
3. **Inner loop:** set the angle-loop `kp` to 0 and command a rate setpoint. Raise the rate `kp` until it starts to oscillate, then back off by about 30%. Add `kd` to damp it, then a small `ki` to remove steady-state error.
4. **Outer loop:** restore the angle-loop `kp` and raise it until the payload returns to level quickly without overshooting.
5. **Watch the serial output** (115200 baud) or use the Serial Plotter. It shows the attitude, the commands, and whether the accelerometer is being trusted.
