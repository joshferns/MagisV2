# GPS over SPI / I2C - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [CHANGES](CHANGES.md)

## Task 1 - Bus transport driver ( 2026-10-14 )

| File | Change | Why |
|---|---|---|
| [drivers/gps_ublox_bus.h](../../../../src/main/drivers/gps_ublox_bus.h) | New. `GPS_ON_BUS` = `GPS` and ( `GPS_SPI` or `GPS_I2C` ). `gpsBusInit()`, `gpsBusRead()`, `gpsBusWrite()`, status `gpsBusStatus` ( 1 ok, 0 not init, -2 not found, -3 disabled after I2C errors ), `gpsBusErrorCount`, `gpsBusMaxReadUs`. `GPS_BUS_READ_MAX` / `GPS_BUS_WRITE_MAX` = 64 / 64 on SPI, 8 / 10 on I2C. | One seam under the UBX parser ( DESIGN option A ); caps keep one call under ~300 µs. |
| [drivers/gps_ublox_bus.c](../../../../src/main/drivers/gps_ublox_bus.c) | New. **SPI2:** PB7 push-pull CS, set high before it becomes an output. Each access saves CR1, waits BSY ( bounded ), clears SPE, sets mode 0 / prescaler 8 ( 4.5 MHz ), re-enables, drains ≤ 8 stale RX bytes, transfers with `spiTransferByte()`, then restores CR1 the same way. Probe: MON-VER poll, then ≤ 200 ms of reads looking for `B5 62` or `$G`. **I2C1:** reads 8 B from register 0xFF ( stream ) per call; writes 2-10 B with `i2cWriteBufferwithoutregister()`; probe = one read of 0xFD at boot; 3 failures in a row → `GPS_BUS_DISABLED`. | u-blox SPI is mode 0 ≤ 5.5 MHz while flash and PAW3903 run mode 3 at 9 MHz. The I2C driver cannot see a NACK and recovers the IMU's bus on timeout, so a missing module must not be polled. `i2cReadDevice()` is avoided: it never sends a byte between its TXIS and TC waits. |
| [Makefile](../../../../Makefile) | `drivers/gps_ublox_bus.c` added to `PRIMUSX2_DRIVERS`. | No source glob. |
| [target/PRIMUS_X2_v1/target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h) | Commented `GPS_SPI` / `GPS_I2C`; `GPS_SPI_CS_GPIO` / `GPS_SPI_CS_PIN` ( PB7 ), `GPS_I2C_ADDR` ( 0x42 ). | Off by default ( decision ); one-line overrides for the team's wiring. |
| [PIN_MAP.md](../../../PIN_MAP.md), [TIMER_MAP.md](../../../TIMER_MAP.md) | PB7 / `PWM_3` marked as GPS chip-select when `GPS_SPI` is on. | Map kept in sync with the pin claim. |

Sizes ( `gps_ublox_bus.o`, before linking ): SPI 648 B text + 71 B bss; I2C 288 B text + 6 B bss.
Not linked into the image yet ( nothing calls it until task 2 ).

Timing by calculation: SPI read 64 B × ~2.5 µs ≈ 160-200 µs; I2C read 8 B = 11 bytes on the bus
≈ 250-260 µs; I2C write 10 B ≈ 250 µs. Measured worst case is kept in `gpsBusMaxReadUs` for task 7's log.

### Review fixes ( pluto-reviewer, no blocking findings )

| Finding | Fix |
|---|---|
| User `SPI_Init` can set LSB-first on SPI2 | `SPI_CR1_LSBFIRST` added to the CR1 mask, so the GPS always runs MSB-first |
| 3 I2C failures = ~30 ms of stalled loop and 3 IMU-bus resets | `GPS_I2C_MAX_CONSECUTIVE_ERRORS` 1 |
| A 1-byte I2C write tail is dropped silently | Header documents the ≥ 2 B rule and the no-1-byte-tail split ( task 4 ) |
| One missed 200 ms SPI probe disabled GPS for good | `gpsBusInit()` resets status and may be repeated on SPI ( task 2 retries ~1 s ) |
| 8 B every 20 ms is 400 B/s < NAV-PVT's 500 B/s on I2C | Plan: I2C polled every 10 ms, NAV-PVT only ( tasks 2 and 4 ) |
| `gpsBusRead ( buf, 0 )` counted as an I2C error | early return on `max == 0` |
| CS clock hard-coded to GPIOB | `GPS_SPI_CS_RCC`, default `RCC_AHBPeriph_GPIOB` |
| Header gating needs target.h | header includes `platform.h` |

## Task 2 - gps.cpp on the bus transport ( 2026-10-15 )

| File | Change | Why |
|---|---|---|
| [io/gps.cpp](../../../../src/main/io/gps.cpp) `gpsInit()` | Under `GPS_ON_BUS`: no serial-port lookup; `gpsBusInit()` probes once. I2C miss → `featureClear(FEATURE_GPS)`. SPI miss → carry on. State → `GPS_INITIALIZING`. | No UART on this board; never poll a missing I2C module ( IMU bus ); a booting SPI module misses the probe. |
| `gpsProvider()`, `gpsPortIsBus()` | Bus path always UBX ( stored `provider` defaults to NMEA ); `gpsNewFrame()` and `gpsInitHardware()` use it. | No runtime write into `masterConfig`. |
| `gpsInitUbloxBus()` + `gpsBusSendList()` | 500 ms boot wait, then `ubloxInit` and the SBAS frame one write per call: ≤ `GPS_BUS_WRITE_MAX`, never across a frame, never a 1-byte tail; SPI bytes clocked back are parsed. No baud states. | Replaces the serial `gpsInitUblox()` steps that need a UART. |
| `gpsThread()` | Bus path: one transfer per call, every 20 ms ( SPI ) / 10 ms ( I2C ); write while configuring, else a read; GPS sensor cleared and state `GPS_UNKNOWN` if the I2C driver disabled itself. `autoBaud` skipped. | Bounded loop cost; I2C rate per the task 1 review. |
| `gpsNewData()` | `gpsBusStatus = GPS_BUS_OK` on the first parsed frame. | Marks a module that missed the SPI probe as found. |
| `gpsEnablePassthrough()` | Returns if `gpsPort` is NULL. | CLI passthrough dereferenced a NULL port without a serial GPS. |
| [drivers/gps_ublox_bus.c](../../../../src/main/drivers/gps_ublox_bus.c) | SPI reads / writes no longer gated on `gpsBusStatus`. | The reconfigure-on-timeout loop ( 2.5 s no data → re-init ) is the SPI retry; no blocking re-probe. |

USART2 keeps its `FUNCTION_GPS` default ( config.cpp:336 ): the bus path never opens it, so no
`EEPROM_CONF_VERSION` bump.

Builds ( PRIMUS_X2_v1 ): GPS serial 108.9 KB / 15.6 KB ( unchanged ), GPS + SPI 107.6 KB / 15.7 KB,
GPS + I2C 107.2 KB / 15.6 KB. No `gps.cpp` / `gps_ublox_bus` warnings beyond the 21 already in the
serial GPS build. Default build gate clean.

A missing SPI module costs one reconfigure cycle every ~2.5 s ( ~16 writes, each < 200 µs ) and one
64 B read a second ( review fix: reads every 20 ms only once a frame has been parsed ).

### Review fixes ( pluto-reviewer, no blocking findings )

| Finding | Fix |
|---|---|
| No SPI module = a 64 B SPI2 read every 20 ms forever ( ~1 % CPU, constant traffic beside flash / PAW3903 ) | Until `gpsBusStatus` is `GPS_BUS_OK`, reads drop to one a second ( `GPS_BUS_SEARCH_POLL_MS` ); config writes keep their rate |
| Docs said "probe repeated about once a second" / "~3.4 s cycle" | DESIGN, TASKS decision and this file corrected |
| Interim `ubloxInit` asks ~820 B/s on I2C ( 800 B/s link ) | Left to task 4 ( PVT-only on I2C ); an I2C bench test before task 4 will drop frames |
| `autoConfig` OFF on the bus path leaves the module on NMEA | Logged; default is ON |

## Task 3 - NAV-PVT parser ( 2026-10-15 )

| File | Change | Why |
|---|---|---|
| [io/gps_ubx_pvt.h](../../../../src/main/io/gps_ubx_pvt.h) | New, dependency-free. `ubx_nav_pvt` ( 92 B, `static_assert` on size; all fields naturally aligned so it matches the wire layout ), `gpsPvt_t` in firmware units, `ubxDecodePvt()` ( length check, mm → cm with saturation, heading 1e-5 deg → deg × 10 wrapped 0-3599, fix = gnssFixOK and 3D ). | Testable on a host without the firmware headers. |
| [io/gps.cpp](../../../../src/main/io/gps.cpp) `UBLOX_parse_gps()` | Class check: ACK-ACK / ACK-NAK counted ( `GPS_ubxAckCount` / `GPS_ubxNakCount` ), non-NAV classes ignored. NAV-PVT ( `ubloxApplyPvt()` ) sets position, altitude, NED velocity, speed, course, hAcc / vAcc, PDOP ( into `GPS_hdop` ), numSat, fix type and both new-data flags, so one PVT is one navigation update. While a PVT arrived in the last 1 s, legacy POSLLH / STATUS / SOL / VELNED are ignored ( SVINFO still read ). | A CFG or ACK message with id 0x02 / 0x03 / 0x06 used to be parsed as NAV data. Legacy + PVT of the same epoch would call `onGpsNewData()` twice with a near-zero time step. |
| `io/gps.cpp` legacy path | POSLLH also fills `GPS_hAcc` / `GPS_vAcc`, VELNED fills `GPS_velNED`, SOL fills `GPS_fixType`. | Same globals whichever message set the module sends. |
| [io/gps.h](../../../../src/main/io/gps.h) | Externs for `GPS_velNED`, `GPS_hAcc`, `GPS_vAcc`, `GPS_fixType`, `GPS_ubxAckCount`, `GPS_ubxNakCount`. | For the user API ( task 6 ), log ( task 7 ), config check ( task 4 ). |
| [tests/pvt_decode_test.cpp](tests/pvt_decode_test.cpp) | Host test: builds a NAV-PVT frame at the documented offsets, checks the checksum, struct alignment and every decoded field, plus no-fix, bad length, saturation and course wrap. | Task 3 "done when". Result: all checks pass ( frame checksum 66 53 ). |

Builds ( PRIMUS_X2_v1 ): GPS serial 109.5 KB, GPS + SPI 108.3 KB / 15.7 KB, GPS + I2C 107.9 KB / 15.6 KB
( +0.6-0.7 KB ); no new warnings; default build gate clean. RAM unchanged ( union already 344 B ).

Units note: `GPS_speed` is cm/s on both paths ( gps.h still says 0.1 m/s, pre-existing ); `GPS_hdop`
holds PDOP × 100 on the PVT path ( SOL also gave position DOP ).

### Review notes ( pluto-reviewer, no blocking or should-fix findings )

| Note | Action |
|---|---|
| New globals keep stale values after GPS loss | Reset in `GPS_LOST_COMMUNICATION`: fix type 0, hAcc / vAcc 65535 ( unknown ), velocity 0 |
| `gps.h` said `GPS_altitude` 0.1 m and `GPS_speed` 0.1 m/s | Comments corrected to m and cm/s ( what both paths store ) |
| 84-byte M7 NAV-PVT rejected silently | Out of scope ( decisions log ) |
| Legacy-suppression window | Verified: one update per epoch for in-order messages; at most 1 s gap when PVT stops |

## Task 4 - Module configuration over SPI / I2C ( 2026-10-15 )

| File | Change | Why |
|---|---|---|
| [io/gps.cpp](../../../../src/main/io/gps.cpp) `ubloxBusInit` | New setup list per bus, sent by `gpsInitUbloxBus()` in place of the serial `ubloxInit`. **SPI ( 185 B, 9 frames ):** CFG-VALSET RAM ( SPI out UBX on / NMEA off, NAV-PVT on SPI, 100 ms, nav 1, dyn model 6 ), CFG-PRT port 4 ( mode 0, ffCnt 50, in UBX+NMEA+RTCM, out UBX ), CFG-MSG NAV-PVT / POSLLH / STATUS / SOL / VELNED, CFG-RATE 100 ms, CFG-NAV5 dyn model only. **I2C ( 141 B, 5 frames ):** CFG-VALSET RAM ( I2C out UBX on / NMEA off, NAV-PVT on I2C, 200 ms, dyn 6 ), CFG-PRT port 0 ( DDC address 0x42, out UBX ), CFG-MSG NAV-PVT, CFG-RATE 200 ms, CFG-NAV5 dyn model only. SBAS frame unchanged after the list. | VALSET for M9 / M10, legacy frames for M8; the module NAKs the set it does not know ( `GPS_ubxNakCount` ). Port output set to UBX only, so no NMEA disables. I2C stays PVT-only at 5 Hz ( 500 B/s vs 800 B/s read ). |
| [tests/ubx_frames_gen.py](tests/ubx_frames_gen.py) | Generates the frames ( checksums computed, not typed ). Reproduces the firmware's existing POSLLH and 5 Hz CFG-RATE frames byte for byte. | No hand-made checksums. |
| [tests/ubx_frames_check.py](tests/ubx_frames_check.py) | Reads every UBX array and SBAS message out of `gps.cpp` and checks sync, lengths and checksums. Result: all 8 lists valid ( `ubloxInit`, both `ubloxBusInit`, 5 SBAS ). A corrupted byte is caught. | Task 4 "done when". |

( Sizes above are the first version; see the review fixes below for the final lists. )

**Dynamic model change:** the serial `ubloxInit` uses pedestrian ( 3 ); the bus lists use airborne
< 1 g ( 6 ), and set only that field ( NAV5 mask 0x0001 ) so the module's other navigation settings stay as they are.

VALSET keys were checked against the documented key table; one recalled value was wrong
( SPI output protocol is 0x107A0001, 0x10790001 is SPI *input* ) and was corrected before use.

Builds ( PRIMUS_X2_v1 ): GPS + SPI 108.3 KB / 15.7 KB, GPS + I2C 107.9 KB / 15.6 KB; no new warnings;
default build gate clean.

### Review fixes ( pluto-reviewer, no blocking findings )

| Finding | Fix |
|---|---|
| I2C list reconfigured the DDC port from the DDC port: one NACK while it restarts stops GPS for good ( `GPS_I2C_MAX_CONSECUTIVE_ERRORS` 1 ) and resets the IMU's bus | DDC CFG-PRT removed. NMEA turned off by CFG-MSG F0-00..05 rate 0 ( this port ); VALSET still sets I2C output UBX-only on M9 / M10. Final I2C list 179 B, 10 frames |
| SPI at 264 B per fix × 10 Hz = 83 % of the 3200 B/s read capacity | NAV-SOL dropped ( PVT carries numSV, fix and DOP ): 204 B per fix, 64 %. Final SPI list 174 B, 8 frames |
| Warning gate never ran on a GPS build | Gate run on GPS + SPI, GPS + I2C and GPS serial, compared with a serial-GPS build of `e766113` ( before this topic ): no new gate entries. Two new `DISABLE_STATE ( GPS_FIX )` uses had added two `runtime_config.h` sign-conversion warnings; replaced by `gpsSetFixState()` with explicit types |
| `ubx_frames_check.py` only ran from the repo root | Path resolved from the script's location |
| New arrays used 4-space indent | Generator emits 2-space, matching the file |
| Notes | Dyn model 6 gives no 2D fixes ( expect 3D or none in the hand-held test ); check `GPS_ubxNakCount` and the real PVT rate on an M10 at 10 Hz in task 9 |

## Task 5 - Lock out GPS hold / home and GPS_angle; MSP check ( 2026-10-15 )

**Found while tracing the mode paths:** `updateGpsWaypointsAndMode()` ( navigation.cpp, called from
`mw.cpp:994` on every RX update whenever `SENSOR_GPS` is set ) force-activates `BOXGPSHOLD` on every
call, and on entering GPS_HOLD sets the target to hard-coded coordinates `GPS_hold = 192326790,
729157020` ( ~19.23° N, 72.92° E ) instead of the craft's position. With a 3D fix, ≥ 5 satellites and
sticks within ±40 of centre ( `ap_mode` 40 ), GPS_HOLD would engage by itself and
`updateGpsStateForHomeAndHoldMode()` ( `mw.cpp:1402`, once `GPS_FIX_HOME` is set at arm ) would feed
`GPS_angle` into every PID controller's angle error, steering towards that fixed point. It has been in
the code since `8b06067` ( 2025-03-01 ) but never ran, because `#define GPS` did not compile until this
work.

| File | Change | Why |
|---|---|---|
| [flight/navigation.h](../../../../src/main/flight/navigation.h) | `GPS_NAV_MODES_LOCKED`, default 1. | One switch for the fusion topic to lift. |
| [flight/navigation.cpp](../../../../src/main/flight/navigation.cpp) `updateGpsWaypointsAndMode()` | Locked: clears GPS_HOLD / GPS_HOME and `nav_mode`, calls `GPS_reset_nav()` ( zeroes `GPS_angle`, nav PIDs ) if any was set, keeps the "GPS ready" beep, returns before the forced `ACTIVATE_RC_MODE ( BOXGPSHOLD )`. | Neither mode can be engaged by RC switches, MSP waypoints ( `MSP_SET_WP` writes `nav_mode` ) or the forced box. |
| `updateGpsStateForHomeAndHoldMode()` | Locked: sets `GPS_angle` to 0 and returns. | Second barrier in front of the PID input. |

Verification by inspection: `GPS_angle` is written only at navigation.cpp:63 ( init 0 ), :369
( `GPS_reset_nav`, 0 ), :646-647 ( lock, 0 ) and :655-659 ( unreachable while locked ). MSP_RAW_GPS /
MSP_COMP_GPS ( serial_msp.cpp:1146-1160 ) read `GPS_FIX`, `GPS_numSat`, `GPS_coord`, `GPS_altitude`,
`GPS_speed`, `GPS_ground_course`, distance / direction to home: same globals and units on the PVT path
( task 3 ), so no change. `onGpsNewData()` still runs ( home position, distance / direction to home ).

Builds: GPS serial 109.1 KB, GPS + SPI 107.8 KB, GPS + I2C 107.4 KB ( −0.5 KB, locked code is dead );
no new gate entries vs the pre-topic GPS build; default gate clean.

### Review notes ( pluto-reviewer, no blocking or should-fix findings )

Whole-tree check confirmed: no reachable path sets GPS_HOLD_MODE / GPS_HOME_MODE or a non-zero
`GPS_angle`; the lock does not depend on the call order in `loop()` ( `updateGpsWaypointsAndMode()`
runs on RX loops, `updateGpsStateForHomeAndHoldMode()` on control loops ).

| Note | Action |
|---|---|
| `GPS_NAV_MODES_LOCKED 0` from target.h / `-D` would silently bring back the forced box and hard-coded target | [navigation.h](../../../../src/main/flight/navigation.h): `#error` when unlocked; verified the build stops |
| With GPS compiled in, `MSP_SET_WP` ( wp 0 and 16, `alt != 0` ) writes `AltHold` in flight; reachable only since GPS compiles | [serial_msp.cpp](../../../../src/main/io/serial_msp.cpp): the `AltHold` writes are compiled only when unlocked |
| `MSP_SET_WP` wp 0 overwrites `GPS_home` and sets `GPS_FIX_HOME` | Kept ( data only: distance / direction to home in MSP_COMP_GPS ); logged |
| `GPS_FILTERING 0` with `#if defined()` always compiles the filter | Pre-existing, harmless while `nav_mode` cannot reach POSHOLD; logged for the fusion topic |

## Task 6 - User API: GPS read functions ( 2026-10-15 )

| File | Change | Why |
|---|---|---|
| [API/GPS.h](../../../../src/main/API/GPS.h) | New public header: `GPS_Option_e` ( `Gps_Available`, `Gps_Fix_3D`, `Gps_Fix_Type`, `Gps_Satellites`, `Gps_Latitude`, `Gps_Longitude`, `Gps_Altitude`, `Gps_Vel_North/East/Down`, `Gps_Ground_Speed`, `Gps_Course`, `Gps_H/V_Accuracy`, `Gps_PDOP` ) and `int32_t Gps_Get ( option )`. | Same shape as `Bms_Get`; `int32_t` carries lat / lon. `Gps_` prefix avoids firmware names such as `GPS_FIX`. |
| [API-Src/GPS.cpp](../../../../src/main/API-Src/GPS.cpp) | Implementation; every option 0 when `SENSOR_GPS` is clear or the firmware has no `GPS`. Altitude cast back through `int16_t` so heights below sea level read negative. | User code compiles and runs on every build. |
| [io/gps_data.h](../../../../src/main/io/gps_data.h) | New: the GPS solution globals and `LAT` / `LON`, moved out of `io/gps.h` ( which includes it ). | `io/gps.h` includes `common/atomic.h`, whose `#warning` fires in every includer: the API would have added 3 gate warnings. |
| [PlutoPilot.h](../../../../PlutoPilot.h), [Makefile](../../../../Makefile) | `#include "src/main/API/GPS.h"`; `API-Src/GPS.cpp` in `DRONA_API`. | Wiring. |
| [docs/API/GPS_API_WIKI.md](../../../API/GPS_API_WIKI.md) | New wiki: quick start, builds with GPS, option table with units, limits. Marks API 1.5.0 as pending. | Public API change ( pluto-rules ). `API_Version` 1.4.0 → 1.5.0 and `FW_Version` bump happen at commit ( task 12, pluto-commit ). |

Builds: default 100.8 KB ( unchanged; `Gps_Get` is not linked unless called ), GPS serial 109.0 KB,
GPS + SPI 107.8 KB, GPS + I2C 107.4 KB; PRIMUS_V5 and PRIMUSX2 build; no new gate entries.

### Review fixes ( pluto-reviewer, no code defects )

| Finding | Fix |
|---|---|
| Wiki said a build without SPI / I2C reads 0; the serial GPS path fills the same globals | Wiki: "`GPS` only" row and "Serial GPS limits" ( NMEA: fix type 0, PDOP 9999, accuracy 65535, velocity 0; UBX: pedestrian, 2D possible ) |
| Wiki understated I2C: any single failed transfer, also in flight, turns GPS off | Wiki row reworded |
| Unbacked figures ( accuracy, PDOP, course speed, wander ) | Removed or made qualitative until task 9 measures them |
| Task 6 done-when contradicted the always-present header; `User_Space_API.md` missing from pipeline docs | TASKS.md updated |

## Task 7 - Flight-log fields and bench procedure ( 2026-10-15 )

| File | Change | Why |
|---|---|---|
| [PlutoPilot.cpp](../../../../PlutoPilot.cpp) | GPS bench log behind `#define GPS_BENCH_LOG` ( commented out ): every 100 ms a solution line ( `t n Fx Sv hA dN dE vN vE`, ≤ 125 B ), every 10th record a status line instead ( `t Av Bs Be Bu Ak Nk`, ≤ 88 B ), and a `GPSLOG_START` marker on its own tick from `onLoopStart ( )`. Reads GPS through `Gps_Get ( )`; bus counters via `extern "C"`. | Fits the ~130 B / tick budget ( 15 fields on one line would be ~190 B ). The template file stays clean for users. **Temporary: remove or keep disabled before release.** |
| [TESTING.md](TESTING.md) | Wiring, enabling, field table, tests A ( indoor comms ), B ( shared bus ), C ( outdoor hand-held ), D ( loss / recovery ) with pass criteria. | Task 9 procedure. |

Builds: log off = default unchanged, gate clean; log on: GPS serial 110.1 KB, SPI 109.0 KB, I2C 108.6 KB,
no warnings in PlutoPilot.cpp, no new gate entries.
