# Flix Quadcopter (PlatformIO)

Flight controller firmware for an ESP32-based quadcopter. This project is derived from [Flix](https://github.com/okalachev/flix) by Oleg Kalachev, restructured for PlatformIO and extended with improved IMU calibration and arming safety.

For hardware documentation, schematics, assembly instructions, simulation and the full command reference, refer to the upstream repository. Those materials apply to this project unless noted below.

## Changes from upstream

- Restructured as a PlatformIO project for use with VS Code.
- IMU initialization is checked at startup. Flight control stays disabled if the sensor fails to initialize or configure.
- Gyroscope bias is measured automatically at every boot and refreshed in the background whenever the drone is disarmed and stationary. It is not stored in flash.
- Accelerometer calibration is performed once and stored in flash. Results are validated, and implausible values are rejected.
- A central arming interlock refuses to arm until the IMU is ready and a minimum delay has elapsed since boot or the last disarm.

## Hardware

| Component | Current configuration |
|---|---|
| Microcontroller | ESP32 |
| IMU | MPU6050 |
| Control interface | MAVLink over Wi-Fi (QGroundControl), optional SBUS receiver |

## Build and upload

Requires [PlatformIO](https://platformio.org/).

```
pio run                  # build
pio run -t upload        # upload to the board
pio device monitor       # serial console (115200 baud)
```

## IMU calibration

### Accelerometer (one-time)

The accelerometer must be calibrated once. The resulting bias and scale values are saved to flash and reused on every boot.

1. Place the drone on a level surface and make sure it is disarmed.
2. Send `ca` from the serial or MAVLink console.
3. Follow the on-screen prompts. The drone must be held level and still in each of six orientations (8 seconds each).
4. Send `imu` to verify the stored bias and scale values.

If the result is out of range, the calibration is rejected and the previous values are kept. Repeat the procedure in that case.

**Recalibrate the accelerometer only when:**

- the IMU board is replaced or re-mounted, or
- flash memory is erased or parameters are reset with `preset`.

### Gyroscope (automatic)

No manual gyroscope calibration is required. The bias is measured at boot and refreshed while the drone is disarmed and stationary. Keep the drone still on a level surface for a few seconds after powering on. The `cg` command is still available for a manual measurement while disarmed.

### Arming conditions

Arming from the sticks, the CLI or MAVLink is refused until all of the following are true:

- the IMU is initialized and healthy,
- a gyroscope bias has been measured, and
- at least 3 seconds have passed since boot or the last disarm.

## Current status

The drone flies stably. Response to smartphone control is sensitive and may be demanding for inexperienced pilots; it can be reduced by lowering the tilt and rate limits (for example `CTL_TILT_MAX`) in the parameter settings of the ground control software.

Known limitations with the MPU6050:

- No altitude hold.
- No absolute heading reference, so yaw drifts over time.

## Roadmap

- Integrate an MPU9250 to add a magnetometer for heading estimation. Magnetometer initialization is implemented; processing and use in the estimator are not yet.
- Evaluate a time-of-flight sensor for altitude control.

> **Hardware note:** many inexpensive breakout boards sold as "MPU9250", "MPU6500" or "MPU-92/65" are not genuine MPU9250 boards. They are often MPU6500-based and may lack a working magnetometer. Purchase sensors from verified suppliers to avoid difficult debugging.

## Credits

Based on [Flix](https://github.com/okalachev/flix) by Oleg Kalachev. See the upstream repository for license terms.