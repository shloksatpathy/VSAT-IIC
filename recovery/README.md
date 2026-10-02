# Secondary Recovery and Navigation

Recovery for CAN7U-SAT after ejection. The CanSat comes down on the primary parachute, cuts it away at 600 m and deploys a steerable parafoil. It then glides toward the landing target while managing how much altitude it has left, and finally flares and lands.

## Mission Stages

```
WAIT_EJECTION ──► DESCEND_PRIMARY ──► CONFIRM_SEPARATION ──► DEPLOY_CANOPY ──► GUIDANCE ──► FLARE ──► LANDED
  apogee or        alt <= 600 m:        ~0 g for 150 ms        release canopy,    heading PID    both brakes   cut servo power,
  ejection         cut primary          (or 2 s timeout)       wait for inflation  + glide-ratio  fully pulled  log landing point
  switch                                                       and swing to settle mode gating
```

| Stage | Exits when | Fallback |
| --- | --- | --- |
| **Wait for ejection** | The ejection switch trips, or altitude drops 10 m below an apogee above 500 m | — |
| **1. Primary descent** | Altitude is ≤ 600 m for 3 readings in a row → fire the cutter | Altitude falls back from barometer → GNSS → dead reckoning at 20 m/s |
| **2. Confirm separation** | Acceleration stays below 0.3 g for 150 ms | Deploy anyway after 2 s |
| **3. Deploy canopy** | Descent rate < 3 m/s for 1 s, then body rates < 30 °/s for 1.5 s | 8 s and 5 s timeouts |
| **4. Guidance** | Altitude < flare altitude (8 m) | Brakes go neutral (wings level) if GNSS or IMU data is more than 1 s old |
| **5. Flare** | A landing shock (> 3 g), or descent rate ≈ 0 for 2 s | — |

## Guidance (Stage 4)

On every control cycle (50 Hz):

1. **Position.** Each GNSS fix is converted into local X/Y metres relative to the ejection point (X = east, Y = north).
2. **Heading.** The gyro yaw rate is integrated between fixes. A complementary filter pulls this heading toward the GNSS ground track to cancel gyro drift. The ground track is only measured once the CanSat has moved at least 5 m, so GNSS position noise can't swamp it.
3. **Wind.** Wind is estimated as ground velocity minus canopy air velocity. Its component along the line to the target (positive = tailwind) adjusts the glide ratio:
   `effective_ratio = max_glide_ratio × (airspeed + wind) / airspeed`
4. **Altitude-budget mode.** This compares the glide ratio needed to reach the target (`distance / altitude`) with the ratio the canopy can actually achieve:

   | Mode | Condition | Steering |
   | --- | --- | --- |
   | `TOO_HIGH` | required < 0.8 × effective | PID output × 1.5. Tighter turns burn off altitude |
   | `ON_TARGET` | in between | Normal PID |
   | `TOO_LOW` | required > effective | Fly straight. Only make a reduced-strength correction (× 0.3) if the heading error exceeds 45° |

5. **Heading PID → brakes.** The brake lines can only be pulled, not pushed. A positive error means the target is to the right, so the controller pulls the **right** brake.

## Files

| File | Purpose |
| --- | --- |
| `recovery.ino` | Main loop: setup, 50 Hz control loop, GNSS polling |
| `config.h` | Target coordinates, altitudes, thresholds, PID gains, pins, servo limits |
| `recovery_fsm.h/.cpp` | The state machine for every stage, plus telemetry and landing logs |
| `guidance.h/.cpp` | Heading filter, wind estimate, mode gating, heading PID, brake outputs |
| `altitude.h/.cpp` | Altitude source selection (baro → GNSS → dead reckoning) and descent rate |
| `geo.h/.cpp` | Lat/lon → local X/Y, bearing, angle wrapping |
| `sensors.h/.cpp` | BMP280 barometer, NMEA GNSS (TinyGPSPlus), MPU6050 IMU |
| `actuators.h/.cpp` | Primary cutter, canopy release servo, brake servos, servo power switch |

`geo`, `altitude` and `guidance` use no Arduino code, so they can be tested on a PC.

## Changes from the Original Pseudocode

- **Brake direction is flipped.** The pseudocode pulled the left brake when the target was to the right. On a parafoil that turns *away* from the target, so this code pulls the brake on the side of the turn.
- **Barometer without a GNSS cross-check.** If GNSS is unavailable but the barometer reads normally, the barometer is used. The pseudocode would have switched to dead reckoning.
- **Ejection detection.** The pseudocode assumed the ejection moment was already known. A `WAIT_EJECTION` state now detects it by apogee or by an optional breakaway switch.
- **Timeouts.** Canopy inflation and swing settling each have a timeout, so the CanSat can never get stuck waiting.
- **Noise protection.** The 600 m cut needs 3 consecutive readings below the threshold. The ground track needs a 5 m baseline.
- **Landing check during guidance.** Landing is also checked in `GUIDANCE`, in case the flare altitude is missed.

## Simulation Results

The state machine, guidance and altitude code was run on a PC with stand-in sensors and a simple parafoil model. The model glides at 5 m/s and sinks at 2 m/s, and braking increases the sink rate. The sensors had added noise (GNSS ±1.5 m, barometer ±0.3 m). The target was 500 m from the ejection point.

| Scenario | Result |
| --- | --- |
| Light crosswind (1.6 m/s) | All stages ran in order. Landed **2.0 m** from the target after 244 s, circling in `TOO_HIGH` mode to burn off excess altitude |
| GNSS outage for 5 s during guidance | Held wings level during the outage and resumed when fixes returned. Landed **2.9 m** from the target |
| 3 m/s headwind (target out of reach) | Switched to `TOO_LOW` and flew straight to stretch the glide. Landed 112 m short, about the limit of what the canopy can reach |

These results check the logic only. The sensor and actuator drivers still need bench and drop testing on real hardware.

## Building

1. Open `recovery/recovery.ino` in the Arduino IDE.
2. Install these libraries: **Adafruit BMP280**, **TinyGPSPlus** and **Servo** (on ESP32 use **ESP32Servo** and change the include in `actuators.cpp`).
3. Set `TARGET_LAT` and `TARGET_LON` in `config.h`, plus your pins and servo end points, then build and flash.

## Before Flight

- **Power on at the launch site and keep the CanSat still.** At startup the code records the ground pressure (altitudes are measured from it) and the gyro bias.
- **Wait for a GNSS fix before launch.** The ground GNSS altitude is recorded while the barometer still reads within 20 m of the ground.
- **Replace the placeholder values with drop-test results:** `CANOPY_MAX_GLIDE_RATIO`, `PRIMARY_DESCENT_RATE_MPS` and `CANOPY_DESCENT_RATE_MPS`.
- **Check the brakes on the bench.** At `0` each servo should sit slack at neutral, and at `1` it should pull fully. For a mirrored servo, swap its `NEUTRAL`/`FULL` values.

## Known Limitations

- **No magnetometer.** Heading comes from gyro integration corrected by the ground track. In a crosswind, the gap between heading and track (the crab angle) is only partly observable, so the wind estimate is approximate. Adding a magnetometer would fix this.
- **Equirectangular projection.** The flat-earth X/Y conversion is accurate to well under 1 m over the few kilometres a CanSat covers.
