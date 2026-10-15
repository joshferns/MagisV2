# MagisV2 GPS API Wiki

Read-only GPS data for user code: fix status, satellites, position, altitude, velocity and accuracy.
Everything comes from `Gps_Get ( option )` in `API/GPS.h`, which returns an `int32_t`.

API version **1.5.0** ( firmware with gps-spi-i2c; pending release ).

## Quick Start

```cpp
#include "PlutoPilot.h"

void plutoLoop ( void ) {
    if ( Gps_Get ( Gps_Fix_3D ) ) {
        Monitor_Print ( "Sats:", Gps_Get ( Gps_Satellites ) );
        Monitor_Print ( " Lat:", Gps_Get ( Gps_Latitude ) );
        Monitor_Println ( " Lon:", Gps_Get ( Gps_Longitude ) );
    } else {
        Monitor_Println ( "No fix, sats:", Gps_Get ( Gps_Satellites ) );
    }
}
```

`Gps_Latitude` / `Gps_Longitude` are degrees × 10,000,000. Divide by `10000000.0f` for degrees, or keep
them as integers: one unit is about 1.1 cm north-south.

---

## Which firmware has GPS

The API is in every build, so user code that calls it always compiles. It returns data only on a
firmware built with `GPS` and a transport in `target.h`:

| `target.h` | GPS module | Notes |
|---|---|---|
| `GPS` + `GPS_SPI` | u-blox M8 / M9 / M10 on SPI2, chip-select `GPS_SPI_CS_PIN` ( default PB7 ) | 10 Hz. PB7 is then not available as `PWM_3` / `GPIO_4`. |
| `GPS` + `GPS_I2C` | u-blox M8 / M9 / M10 on I2C1 at `GPS_I2C_ADDR` ( default 0x42 ) | 5 Hz. **Any single failed I2C transfer, at power-up or in flight, turns GPS off until the next power-up** ( it protects the I2C bus the IMU shares ). |
| `GPS` only | Serial GPS on a UART port with the GPS function ( NMEA or UBX, from the GPS config ) | 5 Hz, pedestrian motion model. No free UART on PRIMUS_X2_v1; see "Serial GPS limits". |
| no `GPS` | - | Every option reads 0. |

On PRIMUS_X2_v1 GPS is off by default.

## Options

| Option | Unit | What it returns |
|---|---|---|
| `Gps_Available` | 0 / 1 | 1 while a module is sending data. Drops to 0 after 2.5 s without data. |
| `Gps_Fix_3D` | 0 / 1 | 1 with a valid 3D fix. Use the position only while this is 1. |
| `Gps_Fix_Type` | - | u-blox fix type: 0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS + dead reckoning, 5 time only. On SPI / I2C the firmware selects the airborne motion model, which gives 0 or 3. |
| `Gps_Satellites` | count | Satellites used in the solution. |
| `Gps_Latitude` | deg × 1e7 | North positive. |
| `Gps_Longitude` | deg × 1e7 | East positive. |
| `Gps_Altitude` | m | Above mean sea level ( whole metres ). For height above the take-off point, use the altitude estimate ( `FC-Data.h` ), not GPS. |
| `Gps_Vel_North` | cm/s | North positive. |
| `Gps_Vel_East` | cm/s | East positive. |
| `Gps_Vel_Down` | cm/s | Down positive, so negative while climbing. |
| `Gps_Ground_Speed` | cm/s | Horizontal speed. |
| `Gps_Course` | deg × 10 | Direction of motion, 0-3599, 0 = north, clockwise. Unreliable at walking speed and below. |
| `Gps_H_Accuracy` | cm | The module's horizontal accuracy estimate. 65535 = unknown. |
| `Gps_V_Accuracy` | cm | Vertical accuracy estimate. 65535 = unknown. |
| `Gps_PDOP` | × 100 | Position dilution of precision: 100 = 1.0, lower is better. 9999 = unknown. |

Every option reads 0 while `Gps_Available` is 0. Values update at the module rate ( 10 Hz on SPI,
5 Hz on I2C and serial ); reading faster returns the same value again.

### Serial GPS limits

On a serial GPS with the **NMEA** provider the module reports less: `Gps_Fix_Type` stays 0 while
`Gps_Fix_3D` can be 1 ( set for any fix, 2D included ), `Gps_PDOP` reads 9999, `Gps_H_Accuracy` /
`Gps_V_Accuracy` read 65535 and the three velocity options read 0 ( use `Gps_Ground_Speed` and
`Gps_Course` ). With the **UBX** provider every option is filled, but with the pedestrian motion model
a 2D fix ( `Gps_Fix_Type` 2 ) is possible.

## What GPS does not do yet

- **No GPS position hold or return-home.** The firmware's GPS_HOLD / GPS_HOME modes are locked out
  ( `GPS_NAV_MODES_LOCKED` ) and GPS never moves the drone. Position hold from GPS needs GPS / IMU
  fusion, a later topic.
- **Indoors there is no fix.** `Gps_Available` can be 1 with `Gps_Fix_3D` 0 and few or no satellites.
- **Accuracy is metres, not centimetres.** A small consumer module's position wanders by metres even when
  still; measured figures for the Pluto set-up come with the bench and outdoor test.

## Changes in 1.5.0

- New header `API/GPS.h`, included by `PlutoPilot.h`: `Gps_Get ( GPS_Option_e )`.
