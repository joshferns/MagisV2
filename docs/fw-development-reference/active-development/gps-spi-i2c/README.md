# GPS over SPI / I2C

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md)

| | |
|---|---|
| **Status** | **Active** ( planned 13 Oct 2026 ) |
| **Mode** | feature |
| **Branch** | `feat/gps` ( from `ccr-f11d8196-rtb2ga` @ `e766113` ) |
| **Target** | `PRIMUS_X2_v1` |
| **Origin** | [optic-flow-position-hold](../optic-flow-position-hold/README.md) ( GPS open items ) |

**Summary.** PRIMUS_X2_v1 has no free UART for a GPS ( USART3's pins are motors M3/M4 ). The team
will fit a u-blox breakout on **SPI2** ( default ) or **I2C1**. This topic adds a small transport
layer under the existing UBX parser in `io/gps.cpp`, selected with `GPS_SPI` or `GPS_I2C` in
`target.h`, adds NAV-PVT, configures the module over the chosen bus, and exposes the data to the
flight log, a user API, the app ( MSP ) and `navigation.cpp`. GPS position hold is **not** part of
this topic: GPS_HOLD / GPS_HOME and `GPS_angle` stay locked out until a GPS/IMU fusion topic.

**Hardware not yet known** ( defaults until the team confirms ): u-blox M8 / M9 / M10, interface
select set for SPI, chip-select on **PB7** ( `GPS_SPI_CS`, gives up `PWM_3` / `GPIO_4` ), I2C
address **0x42**.

## Open items

- Module part number and D_SEL / interface jumper ( team ).
- PB7 free on the team's wiring ( team ).
- ESP or ELRS receiver mode on the GPS craft, and whether OPTIC_FLOW flies at the same time.
- 5 Hz or 10 Hz update ( default: 10 Hz on SPI, 5 Hz on I2C ).
