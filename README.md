# CAN7U-SAT — Team VSAT-064

**CanSat India 2026**

Flight software and firmware for **CAN7U-SAT**, Team VSAT-064's entry in CanSat India 2026. This repository contains:

- the **gyro-based control and actuation algorithms** that steer the glider parachute during descent,
- the **sensor integration code** that runs on the main MCU,
- the **communication system** that links the CanSat to the ground station, and
- the **failsafe logic** that keeps every subsystem operating safely.

The main MCU coordinates all subsystems. It reads the sensors, runs the control loop, drives the actuators, and sends telemetry to the ground.

---

## Table of Contents

- [System Overview](#system-overview)
- [Subsystems](#subsystems)
  - [Actuation and Controls](#1-actuation-and-controls)
  - [Sensor Fusion and Integration](#2-sensor-fusion-and-integration)
  - [Communication](#3-communication)
  - [Failsafes](#4-failsafes)
- [Sensor Suite](#sensor-suite)
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
             │                 Control algorithm ───────┼──► Actuators
             │                        │                 │    (glider parachute)
             │                        ▼                 │
             │  Failsafe monitor ◄── Health checks      │
             │                                          │
             │  Telemetry packer ───────────────────────┼──► LoRa ──► Ground station
             └──────────────────────────────────────────┘
```

Each cycle of the flight loop does four things:

1. **Read** raw data from every sensor.
2. **Estimate** altitude, attitude, and position by fusing those readings.
3. **Control** the parachute actuators based on that estimate and the target heading.
4. **Report** telemetry over LoRa, with failsafes watching every stage.

---

## Subsystems

### 1. Actuation and Controls

[`controls/`](controls/)

These algorithms steer the glider parachute during descent.

- Gyro-based attitude and rate feedback to stabilise the payload
- Actuation commands that change the parachute's glide path and heading
- Control loop tuning and actuator limits

### 2. Sensor Fusion and Integration

Drivers and integration code for every onboard sensor, plus the fusion logic that combines them into a single state estimate for the controller.

- Sensor initialisation and calibration
- Periodic sampling and data validation
- Altitude, attitude, and position estimation

### 3. Communication

[`communication/`](communication/)

A **LoRa** radio link carries telemetry to the ground station.

- Telemetry packet format and encoding
- Transmission scheduling
- Ground-station receive and decode

### 4. Failsafes

Safety logic that detects faults and moves the system into a safe state.

- Sensor health and plausibility checks
- Fallback behaviour when a sensor or link fails
- Protection against actuator faults

---

## Sensor Suite

| Sensor           | Purpose                                                        |
| ---------------- | -------------------------------------------------------------- |
| **Altimeter**    | Barometric altitude and descent rate                           |
| **Gyroscope**    | Angular rates for attitude estimation and parachute control    |
| **GNSS**         | Global position, ground track, and heading                     |
| **UV sensor**    | Ultraviolet measurements collected during the mission          |

**Communication module:** LoRa transceiver

---

## Repository Structure

```
Vsat/
├── communication/   # LoRa telemetry and ground-station link
├── controls/        # Gyro-based control and parachute actuation
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
