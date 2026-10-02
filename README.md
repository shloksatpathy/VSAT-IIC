# CAN7U-SAT — Team VSAT-064

**CanSat India 2026**

Flight software and firmware for **CAN7U-SAT**, Team VSAT-064's entry in CanSat India 2026. This repository contains:

- the **gyro-based attitude control** (PID, sensor fusion, control-moment-gyro actuation) that stabilises the payload,
- the **control and actuation algorithms** that steer the glider parachute during descent,
- the **secondary recovery and navigation** system that cuts the primary parachute, deploys a parafoil and glides to the landing target,
- the **sensor integration code** that runs on the main MCU,
- the **communication system** that links the CanSat to the ground station, and
- the **failsafe logic** that keeps every subsystem operating safely.

The main MCU coordinates all subsystems. It reads the sensors, runs the control loop, drives the actuators, and sends telemetry to the ground.

---

## Table of Contents

- [System Overview](#system-overview)
- [Subsystems](#subsystems)
  - [Actuation and Controls](#1-actuation-and-controls)
  - [Gyro Attitude Control](#2-gyro-attitude-control)
  - [Secondary Recovery and Navigation](#3-secondary-recovery-and-navigation)
  - [Sensor Fusion and Integration](#4-sensor-fusion-and-integration)
  - [Communication](#5-communication)
  - [Failsafes](#6-failsafes)
- [Hardware Components](#hardware-components)
- [Arduino Libraries](#arduino-libraries)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [Contributing](#contributing)
- [Team](#team)

---

## System Overview

```
             ┌──────────────────────────────────────────┐
             │                 Main MCU                 │
             │                                          │
 Sensors ───►│  Sensor fusion ──► State estimate        │
 (Alt, Gyro, │                        │                 │
  GNSS, UV)  │                        ▼                 │
             │                 Control algorithm ───────┼──► Actuators (gyro gimbal,
             │                        │                 │    glider parachute)
             │                        ▼                 │
             │  Failsafe monitor ◄── Health checks      │
             │                                          │
             │  Telemetry packer ───────────────────────┼──► LoRa ──► Ground station
             └──────────────────────────────────────────┘
```

Each cycle of the flight loop does four things:

1. **Read** raw data from every sensor.
2. **Estimate** altitude, attitude, and position by fusing those readings.
3. **Control** attitude with the gyro PID loop, and steer the parachute toward the target heading.
4. **Report** telemetry over LoRa, with failsafes watching every stage.

---

## Subsystems

### 1. Actuation and Controls

[`controls/`](controls/)

These algorithms steer the glider parachute during descent.

- Actuation commands that change the parachute's glide path and heading
- Heading commands passed to the gyro attitude controller
- Control loop tuning and actuator limits

### 2. Gyro Attitude Control

[`gyro/`](gyro/) · [details](gyro/README.md)

PID control that holds the payload's attitude steady. It is based on the active-gyroscope approach in [James Bruton's "How this Active Gyroscope Balances"](https://youtu.be/UVJx8T8wTQA). A spinning flywheel sits in a servo-driven gimbal. Tilting the gimbal makes the flywheel precess, and that precession puts a torque on the body.

```
IMU ──► Complementary filter ──► Angle PID ──► Rate PID ──► Gimbal servos + yaw servo
        (roll, pitch, yaw)       (outer)       (inner)      (CMG)       (parachute)
```

- **200 Hz control loop** with cascaded angle → rate PID on roll, pitch and yaw
- **Complementary filter** that ignores the accelerometer during deployment shocks and canopy swing
- **PID protections:** derivative on measurement, filtered D term, anti-windup and output clamping
- **IMU failsafe:** if the IMU stops responding, every servo is centred and the PID state is reset
- **Single config file:** all gains, pins and limits live in [`gyro/config.h`](gyro/config.h)
- **Arduino framework:** runs on ESP32, Teensy or STM32 until the final MCU is chosen

See [`gyro/README.md`](gyro/README.md) for the file layout, build steps and the tuning procedure.

### 3. Secondary Recovery and Navigation

[`recovery/`](recovery/) · [details](recovery/README.md)

After ejection at about 1000 m, the CanSat descends on the primary parachute and cuts it away at 600 m. It then deploys a steerable parafoil and glides to the landing target.

```
WAIT_EJECTION ─► DESCEND_PRIMARY ─► CONFIRM_SEPARATION ─► DEPLOY_CANOPY ─► GUIDANCE ─► FLARE ─► LANDED
                 (cut at 600 m)     (~0 g, 2 s timeout)   (inflate, settle) (heading PID)
```

- **Altitude fusion:** uses the barometer, cross-checked against GNSS, with dead reckoning as the last fallback
- **Navigation:** steers from GNSS positions in local X/Y metres, using a gyro + ground-track heading filter and a wind estimate
- **Altitude-budget control:** `TOO_HIGH` mode turns harder to burn off altitude, and `TOO_LOW` mode flies straight to stretch the glide
- **Steering:** a heading PID drives the pull-only left/right brake lines
- **Failsafes:** brakes go neutral (wings level) if sensor data is stale, every stage has a timeout, and the servos are powered off after landing

| Stage | Trigger | Action |
| --- | --- | --- |
| Primary descent | Altitude ≤ 600 m (3 readings in a row) | Fire the primary-parachute cutter |
| Confirm separation | About 0 g for 150 ms, or 2 s timeout | Move on to canopy deployment |
| Deploy canopy | Descent rate < 3 m/s, then the swing settles | Release the parafoil, then start guidance |
| Guidance | Altitude above 8 m | Heading PID steers the brakes, in `TOO_HIGH` / `ON_TARGET` / `TOO_LOW` mode |
| Flare | Altitude below 8 m | Pull both brakes fully |
| Landed | Shock > 3 g, or no descent for 2 s | Cut servo power and log the miss distance |

**Changes from the original pseudocode:** the brake direction is flipped (the pseudocode would have turned the canopy away from the target), the barometer is used even when GNSS is unavailable, a `WAIT_EJECTION` state detects ejection, and timeouts were added to the canopy stages.

**In simulation** with stand-in sensors, a full flight from ejection landed about 2–3 m from the target in light wind, including with a 5 s GNSS outage. With a headwind that put the target out of reach, it switched to `TOO_LOW` and flew straight to stretch the glide.

**Before flight**, set `TARGET_LAT` / `TARGET_LON` in [`recovery/config.h`](recovery/config.h), and replace the glide ratio and descent rates there with drop-test values.

See [`recovery/README.md`](recovery/README.md) for the guidance maths, file layout, build steps and the full pre-flight checklist.

### 4. Sensor Fusion and Integration

Drivers and integration code for every onboard sensor, plus the fusion logic that combines them into a single state estimate for the controller.

- Sensor initialisation and calibration
- Periodic sampling and data validation
- Altitude, attitude, and position estimation

### 5. Communication

[`communication/`](communication/)

A **LoRa** radio link carries telemetry to the ground station.

- Telemetry packet format and encoding
- Transmission scheduling
- Ground-station receive and decode

### 6. Failsafes

Safety logic that detects faults and moves the system into a safe state.

- Sensor health and plausibility checks
- Fallback behaviour when a sensor or link fails
- Protection against actuator faults

---

## Hardware Components

| Component          | Part                                                                                   | Purpose                                                              |
| ------------------ | -------------------------------------------------------------------------------------- | -------------------------------------------------------------------- |
| **MCU**            | ESP32-S3-N8R2 (8 MB flash, 2 MB PSRAM)                                                 | Main flight computer: sensors, control loops, actuators, telemetry   |
| **PID controller** | Multispan UTC-221P dual-display universal PID controller, 68×68 (J / K / PT-100 2- or 3-wire, configurable) | Temperature PID control                                 |
| **Altimeter**      | GY-63 MS5611-01BA03 high-precision pressure sensor module                              | Barometric altitude and descent rate                                 |
| **IMU**            | Adafruit MPU-9250 9-DOF (MPU-6500 accel/gyro + AK8963 magnetometer, I2C `0x68`)        | Attitude, angular rates, heading, and separation/landing detection   |
| **GNSS**           | *TBD*                                                                                  | Global position, ground track, and heading                           |
| **UV sensor**      | *TBD*                                                                                  | Ultraviolet measurements collected during the mission                |
| **Communication**  | LoRa transceiver (*model TBD*)                                                         | Telemetry link to the ground station                                 |

> **Firmware status:**
> - **IMU:** the drivers in [`gyro/`](gyro/) and [`recovery/`](recovery/) were written for an MPU6050. The MPU-9250's accel/gyro core (MPU-6500) uses the same registers, so they should work with it. Confirm this on the bench. The AK8963 magnetometer isn't used yet. Adding it will give the recovery guidance an absolute heading.
> - **Barometer:** [`recovery/`](recovery/) still reads a BMP280. It needs porting to the MS5611.
> - **Servos:** on the ESP32-S3, use the `ESP32Servo` library in place of `Servo`.

---

## Arduino Libraries

These are the libraries the code uses right now. Install them with the Arduino IDE Library Manager (**Sketch → Include Library → Manage Libraries**).

| Library | Author | Used in | Purpose | Install |
| ------- | ------ | ------- | ------- | ------- |
| `Wire` | Arduino / board core | `gyro/`, `recovery/` | I2C bus for the IMU and barometer | Built in |
| `Servo` | Arduino | `gyro/`, `recovery/` | Servo and ESC PWM output | Built in on AVR. **Not available on ESP32-S3**, see `ESP32Servo` below |
| `ESP32Servo` | Kevin Harrington, John K. Bennett | `gyro/`, `recovery/` (on ESP32-S3) | Servo/ESC PWM on the ESP32-S3; replaces `Servo` | Library Manager |
| `TinyGPSPlus` | Mikal Hart | `recovery/` | NMEA parsing for GNSS position, altitude and satellite count | Library Manager |
| `Adafruit BMP280 Library` | Adafruit | `recovery/` | Barometer driver (placeholder until the MS5611 port) | Library Manager |
| `Adafruit Unified Sensor` | Adafruit | `recovery/` (dependency) | Required by the BMP280 library | Library Manager (offered automatically) |
| `Adafruit BusIO` | Adafruit | `recovery/` (dependency) | Required by the BMP280 library | Library Manager (offered automatically) |

**Board package:** install **esp32 by Espressif Systems** from the Boards Manager, then select **ESP32S3 Dev Module**.

**No library needed:** the MPU-9250 IMU is driven directly over I2C (`gyro/imu.cpp`, `recovery/sensors.cpp`), and the PID, filters and guidance maths are written from scratch.

**Planned:** once the barometer is ported, an MS5611 library (for example `MS5611` by Rob Tillaart) will replace the three Adafruit BMP280 libraries.

---

## Repository Structure

```
Vsat/
├── communication/   # LoRa telemetry and ground-station link
├── controls/        # Parachute steering and actuation
├── gyro/            # PID attitude control, IMU fusion, gimbal/flywheel actuation
├── recovery/        # Primary cut, parafoil deployment, guided glide and landing
└── README.md
```

> More directories will be added for sensor drivers, failsafes, and the ground station as development continues.

---

## Getting Started

> **Work in progress.** Build and flash instructions will be added once the MCU platform and toolchain are finalised.

Steps to document here:

1. MCU platform and required toolchain / IDE
2. Library dependencies
3. Build and flash steps
4. Hardware wiring and pin map
5. Running the ground station

---

## Contributing

1. Create a branch for your change (`feature/<subsystem>-<short-description>`).
2. Keep each subsystem's code in its own directory.
3. Test on hardware or in a bench setup before merging into `main`.
4. Write down any changes to pin assignments, packet formats, or failsafe behaviour.

---

## Team

**Team VSAT-064**, CanSat India 2026
