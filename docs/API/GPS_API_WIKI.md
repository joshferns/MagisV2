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
| `GPS` + `GPS_I2C` | u-blox M8 / M9 / M10 on I2C1 at `GPS_I2C_ADDR` ( default 0x42 ) | 5 Hz. If the module does not answer at power-up, GPS stays off until the next power-up. |
| neither | - | Every option reads 0. |

On PRIMUS_X2_v1 GPS is off by default.

## Options

| Option | Unit | What it returns |
|---|---|---|
| `Gps_Available` | 0 / 1 | 1 while a module is sending data. Drops to 0 after 2.5 s without data. |
| `Gps_Fix_3D` | 0 / 1 | 1 with a valid 3D fix. Use the position only while this is 1. |
| `Gps_Fix_Type` | - | u-blox fix type: 0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS + dead reckoning, 5 time only. With the airborne motion model the firmware selects, expect 0 or 3. |
| `Gps_Satellites` | count | Satellites used in the solution. |
| `Gps_Latitude` | deg × 1e7 | North positive. |
| `Gps_Longitude` | deg × 1e7 | East positive. |
| `Gps_Altitude` | m | Above mean sea level ( whole metres ). For height above the take-off point, use the altitude estimate ( `FC-Data.h` ), not GPS. |
| `Gps_Vel_North` | cm/s | North positive. |
| `Gps_Vel_East` | cm/s | East positive. |
| `Gps_Vel_Down` | cm/s | Down positive, so negative while climbing. |
| `Gps_Ground_Speed` | cm/s | Horizontal speed. |
| `Gps_Course` | deg × 10 | Direction of motion, 0-3599, 0 = north, clockwise. Not meaningful below ~50 cm/s. |
| `Gps_H_Accuracy` | cm | The module's horizontal accuracy estimate ( 1 sigma ). 65535 = unknown. Typical outdoors with a clear sky: 150-300. |
| `Gps_V_Accuracy` | cm | Vertical accuracy estimate. 65535 = unknown. |
| `Gps_PDOP` | × 100 | Position dilution of precision: 100 = 1.0. Below 250 is good. |

Every option reads 0 while `Gps_Available` is 0. Values update at the module rate ( 10 Hz on SPI,
5 Hz on I2C ); reading faster returns the same value again.

## What GPS does not do yet

- **No GPS position hold or return-home.** The firmware's GPS_HOLD / GPS_HOME modes are locked out
  ( `GPS_NAV_MODES_LOCKED` ) and GPS never moves the drone. Position hold from GPS needs GPS / IMU
  fusion, a later topic.
- **Indoors there is no fix.** `Gps_Available` can be 1 with `Gps_Fix_3D` 0 and few or no satellites.
- **Accuracy is metres, not centimetres.** A small consumer module wanders 1-3 m even when still.

## Changes in 1.5.0

- New header `API/GPS.h`, included by `PlutoPilot.h`: `Gps_Get ( GPS_Option_e )`.
