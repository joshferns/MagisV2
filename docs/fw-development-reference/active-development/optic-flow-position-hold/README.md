# Optic flow position hold and GPS build

**Status:** In progress - builds and passes the warning gate, **not yet bench or flight tested** ( 13 Oct 2026 )
**Branch:** `ccr-f11d8196-rtb2ga` ( on top of `BugFix-June26` @ `386599b` )
**Target:** `PRIMUS_X2_v1` ( `OPTIC_FLOW` and `PAW3903_SPI` still off by default in `target.h` )

## Summary

A review of the optic flow path found estimator bugs, a velocity filter too slow for the
velocity loop, no surface-quality fallback, and a side effect that corrupted altitude hold
whenever `OPTIC_FLOW` was enabled. GPS did not compile at all. This topic fixes those in
firmware. Nothing here changes behaviour while `OPTIC_FLOW` / `GPS` stay undefined.

## Problems found

| # | Problem | Where |
|---|---|---|
| 1 | `flow_rate_rps[0]` assigned twice, `[1]` never set: height-from-flow saw one axis | `opticflow.cpp` ( old 220-221 ) |
| 2 | `total_weight` was `uint8_t` summing sub-1 rad/s weights: always 0, height-from-flow never updated | `opticflow.cpp` ( old 263 ) |
| 3 | Flow reset `imuResetAccelerationSum(0)`, which zeroes `accTimeSum`, the altitude estimator's integration interval: enabling flow degraded altitude hold | `opticflow.cpp`, `imu.cpp` |
| 4 | SQUAL / shutter read into a debug variable and never checked: featureless floor or low light reads as zero motion | `opticflow_paw3903.cpp` |
| 5 | Counts scaled by a fixed `0.051` assuming exactly 40 ms; measured `dt` unused, so read jitter became velocity error | `opticflow_paw3903.cpp` |
| 6 | Rotation compensation used one instantaneous gyro sample against a 40 ms flow window | `opticflow_paw3903.cpp` |
| 7 | Flow scaled by `NewSensorRange` ( VL53L0X ) only; with the VL53L1X fitted it read 0 and clamped to 0.1 m. No tilt compensation | `opticflow.cpp` |
| 8 | 25 Hz flow behind a 0.05 IIR at 100 Hz ( ~0.19 s, < 1 Hz cutoff ) as the velocity loop's feedback | `opticflow.cpp` |
| 9 | Velocity-loop integral accumulated per call, not per unit time | `posControl.cpp` |
| 10 | Position integrated in a frame that turns with yaw: the hold point moved after any yaw | `opticflow.cpp`, `posControl.cpp` |
| 11 | `#define GPS` failed to compile: `gpsProfile_t` / `gpsConfig_t` used before their headers | `config_profile.h`, `config_master.h` |

## Changes

- **Driver** ( `drivers/opticflow_paw3903.cpp` `updateSpiOpticFlow()` ): read every 20 ms ( 50 Hz,
  was 40 ms ); counts divided by the measured read period ( `OPTICFLOW_RAD_PER_COUNT` keeps the old
  calibration at 40 ms ); frames rejected when SQUAL < 0x19 **and** Shutter_Upper is at 0x1F;
  `opticFlowSampleValid` / `opticFlowSampleCount` exported. New `opticFlowAccumulateGyro()`, called
  every loop after `imuUpdate()` in `mw.cpp`, averages the gyro over the flow window.
- **IMU**: `imuResetAccelerationSumXYZ()` resets only the flow's level-frame sums.
- **Estimator** ( `flight/opticflow.cpp` ): fixes 1 and 2; height from the VL53L1X
  ( `LASER_TOF_L1x` ) or VL53L0X ( `LASER_TOF` ), tilt-compensated, else the altitude estimate;
  complementary velocity filter ( accel predicts, flow corrects, tau 0.15 s; `FLOW_ACC_BLEND 0`
  falls back to flow only with tau 0.04 s ); health flag with a 200 ms timeout; position integrated
  in the arm-heading frame.
- **Controller** ( `flight/posControl.cpp` ): position errors rotated into the current heading
  frame in `selectVelOrPosmode()`, `PosVelController()` and `SimpleController()`; velocity-loop
  integral scaled by elapsed time normalised to the 3500 us loop, so existing `PIDUSER` I gains
  keep their meaning.
- **Fallback** ( `mw.cpp` ): when flow is unhealthy, roll/pitch go back to the pilot, integrals
  reset and the hold point re-latches when flow returns. The user command sequence is not reset.
- **GPS**: `config_profile.h` / `config_master.h` include `navigation.h` / `gps.h` under `GPS`.

## Build evidence ( PRIMUS_X2_v1 )

| Variant | Flash | RAM |
|---|---|---|
| Default ( unchanged config ) | 100.8 KB | 14.7 KB |
| + `OPTIC_FLOW` + `PAW3903_SPI` | 105.4 KB ( was 104.8 ) | 15.0 KB |
| + `LASER_TOF_L1x` | 114.1 KB | 16.0 KB |
| + `GPS` | 108.9 KB ( did not compile before ) | 15.6 KB |

`tools/warnings.py check`: no new warnings. The optic-flow build has no warnings on changed lines.
PRIMUS_V5 and PRIMUSX2 still build.

## Gotchas / must verify on hardware

1. **Accel sign.** `FLOW_ACC_BLEND` assumes `accel_hf[i]` is `d VelocityX/Y / dt`, the same
   assumption the velocity loop's D term has always made. Check before flying: armed on the bench
   with props off, slide the craft forward by hand and log `VelocityX`, `debugOpticFlow1[0]`
   ( flow-only cm/s ) and `accel_hf[0]`. The fused and flow-only traces must agree in sign and
   the accel must lead them. If they fight, build with `FLOW_ACC_BLEND 0` and report.
2. **Yaw sign.** Hold position, yaw 90°, and check the craft returns to the same spot rather than
   orbiting. A mirrored result means the `psi` sign in `updateYawFrame()` is wrong for this IMU.
3. **SQUAL threshold.** 0x19 / 0x1F are the PMW3901 values; confirm on a plain floor and in low
   light that `bodyRate1[0]` ( SQUAL ) and `bodyRate1[1]` ( shutter ) trip the fallback where expected.
4. **50 Hz read.** Should be within the PAW3903 frame rate; if `opticFlowSampleValid` toggles or
   velocity becomes noisier than at 40 ms, raise `OPTICFLOW_READ_INTERVAL_MS`.
5. `selectVelOrPosmode()` still reads `rcData[]`, not `rcDataPilot[]` ( pre-existing ).

## GPS: open items

The build is fixed; the feature is not usable on PRIMUS_X2_v1 yet:

- **No free UART.** USART1 = ESP, USART2 = serial RX ( ELRS ), USART3 commented out because
  PB10/PB11 are the motor 7/8 pins. `USE_QUAD_MIXER_ONLY` is set, so USART3 may be freeable:
  check `PIN_MAP.md` / `TIMER_MAP.md` before enabling it.
- **No port is assigned `FUNCTION_GPS`** in the default serial config, and `FEATURE_GPS` is off.
- **Navigation is MultiWii-era**: 5 Hz, GPS position only, a 5-sample moving average
  ( ~0.5-1 s lag ), velocity from differenced positions. Not suitable for position hold on a
  micro quad without fusing GPS ( VELNED velocity + position ) with the accelerometer.

## Next actions

1. Bench checks 1-3 above, then a hover over a textured floor with a flight log.
2. Decide the GPS hardware path ( UART3 vs sharing USART2 ), then scope GPS/IMU fusion.
