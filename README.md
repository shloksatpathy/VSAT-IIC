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
- [Pin Connections](#pin-connections)
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

[`communication/`](communication/) · [Net ID / PAN ID setup](communication/lora-net-id-pan-id.md)

A **LoRa** radio link carries telemetry to the ground station.

- Telemetry packet format and encoding
- Transmission scheduling
- Ground-station receive and decode

#### LoRa commands ([`final/final.ino`](final/final.ino))

The ground station sends each command as a text line ending in a newline. The CanSat sends nothing on LoRa until it receives `CONNECT`, so the ACK is always the first packet.

| Command | What the CanSat does | Reply |
| --- | --- | --- |
| `CONNECT` | Opens the link. Sending it again repeats the ACK | `ACK,IMU=1,BARO=1,INA=1,SD=1,GNSS=1,FIX=0,REF=<lat>,<lon>` |
| `START` (default) | Sends telemetry at 1 Hz on LoRa and XBee | `ACK,START` |
| `STOP` | Standby: no telemetry. Brakes and SD logging keep running | `ACK,STOP` |
| `RECALIBRATE` | Brakes to neutral, then recalibrates gyro bias, level roll trim and ground pressure (about 1.5 s). Returns to the previous mode | `ACK,RECALIBRATE` (when done) |
| `<lat>,<lon>` | Sets the reference (target) location | `Reference set: <lat>,<lon>` |
| `here` | Sets the reference to the current GNSS position | `Reference set: <lat>,<lon>`, or `No GNSS fix yet` |

Every command except `CONNECT` is ignored until the link is open.

**ACK fields:** `IMU`, `BARO`, `INA` and `SD` are the init results (1 = OK). `GNSS=1` means NMEA data is arriving, `FIX=1` means a valid fix (at least 4 satellites), and `REF` is the current reference location (`0,0` = not set).

**Telemetry line (CSV):**

```
time_ms,lat,lon,sats,gnss_alt_m,alt_m,pressure_hpa,temp_c,roll_deg,dist_m,bearing_deg,steer,left,right,bus_v,current_ma,uv_mw_cm2
```

The same line is logged to `/log.csv` on the SD card at 10 Hz, and printed to USB Serial.

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

## Pin Connections

Taken from the board schematic (EasyEDA `Sheet_1`, rev 1.0, 2026-10-05). The MCU on the schematic is an **ESP32-S3-DevKitC-1-N16R8**. All pins are ESP32-S3 GPIO numbers.

### Board wiring

| Device | Device pin | ESP32-S3 pin | Notes |
| --- | --- | --- | --- |
| IMU (MPU-9250) | SDA / SCL | GPIO 8 / GPIO 9 | I2C, address `0x68`; VCC 3.3 V |
| Barometer (MS5611) | SDA / SCL | GPIO 8 / GPIO 9 | I2C; VCC 3.3 V |
| Barometer (BMP390L) | SDA / SCL | GPIO 8 / GPIO 9 | I2C; VCC 3.3 V |
| Current sensor (INA260) | SDA / SCL | GPIO 8 / GPIO 9 | I2C; VCC 3.3 V |
| GNSS (7Semi L89HA) | TX / RX | GPIO 5 / GPIO 4 | GNSS TX → ESP RX on 5, GNSS RX ← ESP TX on 4; VCC 3.3 V |
| LoRa (E220-900T30D) | TXD / RXD | GPIO 18 / GPIO 17 | E220 TXD → ESP RX on 18, E220 RXD ← ESP TX on 17; VCC 3.3 V |
| LoRa (E220-900T30D) | M0 / M1 | GND | Tied to GND on the PCB, so the module is always in normal mode |
| LoRa (E220-900T30D) | AUX | Not connected | |
| XBee Pro S2C | TX (DIO13) / RX (DIN) | GPIO 15 / GPIO 16 | VCC 3.3 V |
| Micro SD (SPI) | CS / DI / DO / SCLK | GPIO 6 / 11 / 13 / 12 | VCC 3.3 V |
| UV sensor (ML8511) | OUT | GPIO 7 | Analog input; VIN 3.3 V |
| Servo S1 (header H1) | Signal | GPIO 1 | Through Q1 (IRF540N) |
| Servo S2 (header H2) | Signal | GPIO 38 | Through Q2 (IRF540N) |
| Servo S3 (header H3) | Signal | GPIO 21 | Through Q3 (IRF540N) |
| Servo S4 (header H4) | Signal | GPIO 48 | Through Q4 (IRF540N) |
| Servo S5 (header H5) | Signal | GPIO 47 | Through Q5 (IRF540N) |
| Buzzer | + | GPIO 2 | |
| Solenoid (J5), motor (J4) | Power | No GPIO | J5 on 5 V, J4 on VIN |

### Power

| Rail | Source | Feeds |
| --- | --- | --- |
| VIN (11.2 V, J3) | Battery | 5 V regulator, motor (J4) |
| 5 V | AMS1117-5.0 module from VIN | ESP32 5V pin, servo headers H1–H5, solenoid (J5), 3.3 V regulator |
| 3.3 V | AMS1117-3.3 module from 5 V | All sensors, GNSS, LoRa, XBee, SD card |

All grounds are common.

### Code vs board

The firmware pins were written before the schematic, so several don't match the board yet.

| Code | Code pin | Board | Status |
| --- | --- | --- | --- |
| I2C (all sketches) | SDA 8, SCL 9 | SDA 8, SCL 9 | ✅ Matches |
| `brake_test/` GNSS | RX 18, TX 17 | GNSS on 5 / 4; 18 / 17 is LoRa | ❌ |
| `recovery/` GNSS | `Serial1` default: RX 15, TX 16 | GNSS on 5 / 4; 15 / 16 is XBee | ❌ |
| `recovery/`, `brake_test/` brake servos | GPIO 12, 13 | Servos on 1, 38, 21, 48, 47; 12 / 13 are SD SCLK / DO | ❌ |
| `recovery/` canopy release servo | GPIO 5 | GNSS TX | ❌ |
| `recovery/` primary cutter | GPIO 4 | GNSS RX; no cutter on the board | ❌ |
| `recovery/`, `brake_test/` servo power switch | GPIO 7 | UV sensor; no servo power switch on the board | ❌ |
| `recovery/` barometer | BMP280 at `0x76` | MS5611 and BMP390L | ❌ Driver needs porting |
| `gyro/` gimbal roll servo | GPIO 9 | I2C SCL | ❌ |
| `gyro/` gimbal pitch / yaw servo | GPIO 10 / 11 | 11 is SD DI; 10 is free | ❌ |
| `gyro/` flywheel ESC | GPIO 6 | SD CS | ❌ |
| `communication/` E220 config (ESP32-S3) | RX 18, TX 17, M0 5, M1 6, AUX 7 | Not run on the PCB | ✅ Breadboard only, see [E220 configuration](#e220-configuration-breadboard) |
| `communication/sensor_test.ino` GNSS | RX 18, TX 17 at 115200 | GNSS on 5 / 4 | ❌ |

### Schematic notes

- **Servo signals go through IRF540N MOSFETs (Q1–Q5).** Each drain drives a servo signal line with no pull-up shown, so the signal can't go high and the PWM would be inverted. The IRF540N also needs about 10 V on the gate to switch fully, and the ESP32 gives 3.3 V. Servos accept a 3.3 V signal directly, so GPIO → 330 Ω → servo signal is enough. To cut servo power after landing, switch the servo 5 V rail with one logic-level MOSFET instead.
- **Three UART modules (GNSS, LoRa, XBee) use all three ESP32-S3 UARTs.** Enable **USB CDC On Boot** and use the board's USB port for the Serial Monitor, so `Serial0`, `Serial1` and `Serial2` are all free for the modules.
- **N16R8 variant:** the octal PSRAM uses GPIO 35–37, which none of the wiring uses. The Hardware Components table above lists the N8R2.

### E220 configuration (breadboard)

M0 and M1 are tied to GND on the PCB, so the E220 can't enter configuration mode there. Configure each E220 on a breadboard before fitting it. The module keeps its settings when powered off.

| E220 pin | ESP32-S3 (`E220_Config_ESP32S3.ino`) | ESP32 DevKit (`E220_Config_ESP32_DevKit.ino`) |
| --- | --- | --- |
| TXD | GPIO 18 | GPIO 16 |
| RXD | GPIO 17 | GPIO 17 |
| M0 | GPIO 5 | GPIO 18 |
| M1 | GPIO 6 | GPIO 19 |
| AUX | GPIO 7 | GPIO 4 |
| VCC | 3.3 V or 5 V | 3.3 V or 5 V |
| GND | GND | GND |

1. Flash the config sketch. It sets M0 and M1 HIGH, writes the configuration, and reads it back to the Serial Monitor.
2. Check the readback: address, channel, Net ID / PAN ID (see [`communication/lora-net-id-pan-id.md`](communication/lora-net-id-pan-id.md)) and UART speed. Do this for both the CanSat and ground-station modules.
3. Fit the module on the PCB. It starts in normal mode.

The flight code must use the UART speed set during configuration (9600 in the sketch). With AUX not connected, the code can't tell when the module is busy, so leave a short gap between packets.

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
