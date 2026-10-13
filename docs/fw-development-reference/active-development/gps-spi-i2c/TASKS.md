# GPS over SPI / I2C - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md)

## Resume

| | |
|---|---|
| **Current task** | none: run /pluto-task next |
| **Next step** | Task 1: write `drivers/gps_ublox_bus.c` ( SPI and I2C transport ) |
| **Open questions** | module part number, D_SEL, PB7 free, ESP or ELRS mode ( team ) |
| **Blocked on** | nothing for tasks 1-8; hardware for task 9 |
| **Last updated** | 2026-10-13 |

## Summary

| | |
|---|---|
| **Mode** | feature |
| **Goal** | GPS data from a u-blox module over SPI2 ( default ) or I2C1 on PRIMUS_X2_v1, off by default |
| **Done when** | Indoors: module detected, configured, streaming NAV-PVT ( no fix ). Outdoors, hand-held: 3D fix, numSat ≥ 6, position and velocity consistent with walking, visible in the flight log, the user API, the app ( MSP_RAW_GPS ) and `navigation.cpp`. GPS_HOLD / HOME and `GPS_angle` locked out. |
| **Target** | PRIMUS_X2_v1 |
| **Analysis** | Option A, transport shim ( [DESIGN.md](DESIGN.md) ) |
| **Pipeline docs affected** | `subsystems/Hardware_Bus_Pipeline.md`, new GPS subsystem doc + link from `Firmware_Pipeline.md`, `PIN_MAP.md`, `TIMER_MAP.md`, `docs/API/` GPS wiki |
| **Order** | 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 11 → 12 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Bus transport driver ( SPI2 + I2C1 ) | todo | - | pluto-driver, pluto-rules, c-pro |
| 2 | gps.cpp on the bus transport | todo | 1 | pluto-rules, cpp-pro |
| 3 | NAV-PVT parser | todo | 2 | pluto-rules |
| 4 | Module configuration over SPI / I2C | todo | 2, 3 | pluto-rules, spec-to-code-compliance |
| 5 | Lock out GPS hold / home and GPS_angle; MSP check | todo | 2 | pluto-rules |
| 6 | User API: GPS read functions | todo | 3 | pluto-rules, cpp-pro |
| 7 | Flight-log fields and bench procedure | todo | 3 | pluto-flighttest |
| 8 | Build gate | todo | 1-7 | pluto-build |
| 9 | Hardware validation | todo | 8 | pluto-flighttest |
| 10 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | todo | 9 | graphify, spec-to-code-compliance |
| 11 | Topic review | todo | 10 | pluto-reviewer agent |
| 12 | Graph refresh & commit | todo | 11 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.

## Tasks

### 1. Bus transport driver ( SPI2 + I2C1 )

**Description.** New `drivers/gps_ublox_bus.{c,h}` with `gpsBusInit()`, `gpsBusDetect()`,
`gpsBusRead(buf, max)` and `gpsBusWrite(buf, len)`, compiled under `GPS_SPI` or `GPS_I2C`
( `target.h`, both commented out; `GPS_SPI_CS` defaults to PB7, `GPS_I2C_ADDR` to 0x42 ). SPI: PB7
as a push-pull CS, SPI2 CR1 saved, switched to Mode 0 / prescaler 8 ( 4.5 MHz ) and restored around
every access; read ≤ 64 B per call, stop on a run of 0xFF. I2C: bytes-available from 0xFD/0xFE,
read ≤ 12 B from 0xFF; `gpsBusDetect()` probes once at boot. Register in the Makefile.

- **Depends on:** -
- **Skills / agent:** pluto-driver, pluto-rules; c-pro for the driver
- **Files:** [target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h), new `drivers/gps_ublox_bus.c`, [bus_spi.c](../../../../src/main/drivers/bus_spi.c), [Makefile](../../../../Makefile)
- **Safety impact:** shares SPI2 with flash and PAW3903 and I2C1 with the IMU; a wrong CR1 restore breaks optic flow, an I2C timeout resets the IMU bus. Every path bounded.
- **Done when:** builds with `GPS` + `GPS_SPI`, and with `GPS` + `GPS_I2C`; default build unchanged; one call measured ( `micros()` bracket ) under 300 µs by calculation.
- **Rollback:** comment out `GPS_SPI` / `GPS_I2C`.
- **Status:** todo
- **Result:**

### 2. gps.cpp on the bus transport

**Description.** Under `GPS_SPI || GPS_I2C`: `gpsInit()` calls `gpsBusInit()` / `gpsBusDetect()`
instead of the serial-port lookup ( `featureClear(FEATURE_GPS)` if absent ), forces
`GPS_UBLOX`, and starts the state machine at configuration; baud negotiation, NMEA and
passthrough are compiled out ( passthrough also gets the missing `gpsPort` NULL check ).
`gpsThread()` polls at most every 20 ms and feeds `gpsNewData()` from `gpsBusRead()`. Check whether
the USART2 `FUNCTION_GPS` default ( config.cpp:336 ) still claims the port in ESP mode; if it must
go, bump `EEPROM_CONF_VERSION`.

- **Depends on:** 1
- **Skills / agent:** pluto-rules; cpp-pro
- **Files:** [gps.cpp](../../../../src/main/io/gps.cpp), [config.cpp](../../../../src/main/config/config.cpp), [mw.cpp](../../../../src/main/mw.cpp)
- **Safety impact:** none in flight while modes are locked out ( task 5 ); boot path changes when GPS is on.
- **Done when:** builds; serial GPS build ( `GPS` alone ) unchanged; no unbounded loop in `gpsThread()`.
- **Rollback:** comment out `GPS_SPI` / `GPS_I2C`.
- **Status:** todo
- **Result:**

### 3. NAV-PVT parser

**Description.** Add NAV-PVT ( 0x01 / 0x07, 92 B ) to the `_buffer` union and a class check in
`UBLOX_parse_gps()`. PVT sets `GPS_coord`, `GPS_altitude`, `GPS_numSat`, fix state, ground speed,
course, NED velocity, hAcc and both `_new_position` / `_new_speed`, so `onGpsNewData()` runs on
PVT alone. Legacy messages keep working. No RAM change ( union already 344 B ).

- **Depends on:** 2
- **Skills / agent:** pluto-rules
- **Files:** [gps.cpp](../../../../src/main/io/gps.cpp), [gps.h](../../../../src/main/io/gps.h)
- **Safety impact:** none ( data only ).
- **Done when:** builds; a hand-made PVT frame ( host-side check script ) decodes to the expected values.
- **Rollback:** revert the commit.
- **Status:** todo
- **Result:**

### 4. Module configuration over SPI / I2C

**Description.** Replace the serial init sequence for the bus path: CFG-VALSET ( RAM layer ) for
M9 / M10 and the legacy CFG-PRT ( SPI port 4 or DDC port 0, UBX out only ), CFG-MSG ( NAV-PVT every
fix, legacy messages ), CFG-RATE ( 100 ms on SPI, 200 ms on I2C ) and CFG-NAV5 for M8. One message
per `gpsThread()` call. Checksums checked against the u-blox interface description.

- **Depends on:** 2, 3
- **Skills / agent:** pluto-rules; spec-to-code-compliance against the u-blox M8 / M10 interface descriptions
- **Files:** [gps.cpp](../../../../src/main/io/gps.cpp)
- **Safety impact:** none.
- **Done when:** builds; every UBX frame's checksum verified by a host-side script.
- **Rollback:** revert the commit.
- **Status:** todo
- **Result:**

### 5. Lock out GPS hold / home and GPS_angle; MSP check

**Description.** While the fusion topic is open, GPS_HOLD_MODE / GPS_HOME_MODE cannot be engaged
and `GPS_angle` stays 0 ( it is added into every PID controller's angle error, pid.cpp:153 ).
Confirm MSP_RAW_GPS / MSP_COMP_GPS report the new data unchanged.

- **Depends on:** 2
- **Skills / agent:** pluto-rules
- **Files:** [mw.cpp](../../../../src/main/mw.cpp), [navigation.cpp](../../../../src/main/flight/navigation.cpp), [serial_msp.cpp](../../../../src/main/io/serial_msp.cpp)
- **Safety impact:** **flight path**: prevents the old GPS-only hold from tilting the craft.
- **Done when:** builds; code inspection shows no path that makes `GPS_angle` non-zero.
- **Rollback:** revert the commit.
- **Status:** todo
- **Result:**

### 6. User API: GPS read functions

**Description.** New public header `API/GPS.h` with read-only functions ( fix type, numSat,
lat / lon in 1e-7 deg, altitude cm, NED velocity cm/s, hAcc cm ) implemented in `API-Src/`, gated on
`GPS`. Wiki page in `docs/API/`; bump `API_Version` / `FW_Version` at commit.

- **Depends on:** 3
- **Skills / agent:** pluto-rules; cpp-pro
- **Files:** new `API/GPS.h`, `API-Src/GPS.cpp`, [PlutoPilot.h](../../../../PlutoPilot.h), Makefile, `docs/API/`
- **Safety impact:** none ( read-only ).
- **Done when:** builds with and without `GPS`; public headers unchanged for non-GPS builds.
- **Rollback:** revert the commit.
- **Status:** todo
- **Result:**

### 7. Flight-log fields and bench procedure

**Description.** Add GPS fields to the PlutoPilot.cpp diagnostic log ( fix, numSat, hAcc, lat / lon
deltas, velN / velE, bus error count ) within the 130-byte tick budget, and write `TESTING.md` with
the indoor comms test and the outdoor hand-held fix test.

- **Depends on:** 3
- **Skills / agent:** pluto-flighttest
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp), `TESTING.md`
- **Safety impact:** none; byte budget checked.
- **Done when:** builds; byte count per tick written down and under 130.
- **Rollback:** revert the commit.
- **Status:** todo
- **Result:**

### 8. Build gate

**Description.** `pluto-build --gate PRIMUS_X2_v1` for default, `GPS` + `GPS_SPI`, `GPS` + `GPS_I2C`,
and `GPS` + `GPS_SPI` + `OPTIC_FLOW`.

- **Depends on:** 1-7
- **Skills / agent:** pluto-build
- **Done when:** no new `src/` warnings; flash / RAM recorded for each variant.
- **Status:** todo
- **Result:**

### 9. Hardware validation

**Description.** Indoors: module detected, configuration accepted ( ACKs counted ), NAV-PVT
streaming without a fix, optic flow unaffected if fitted. Outdoors, hand-held: 3D fix, numSat ≥ 6,
position and velocity consistent with a walked path. Both buses if the breakout allows.

- **Depends on:** 8
- **Skills / agent:** pluto-flighttest; pluto-log-analyst for long logs
- **Done when:** results and logs in `TESTING.md`.
- **Status:** todo
- **Result:**

### 10. Architecture & pipeline docs ( PIPELINE_UPDATE.md )

**Description.** Stage new versions of `Hardware_Bus_Pipeline.md` ( third SPI2 device, mode
switching, fix the drift in SCOUT.md ), a new GPS subsystem doc linked from `Firmware_Pipeline.md`,
`PIN_MAP.md` / `TIMER_MAP.md` ( PB7 = GPS_CS ), the API wiki, and correct the GPS notes in
`optic-flow-position-hold/README.md` ( USART2 carries MSP under ELRS; FUNCTION_GPS / FEATURE_GPS
defaults ).

- **Depends on:** 9
- **Skills / agent:** graphify, spec-to-code-compliance
- **Done when:** PIPELINE_UPDATE.md written; maps updated; every Mermaid edge a real call path.
- **Status:** todo
- **Result:**

### 11. Topic review

**Description.** `pluto-reviewer` on the whole `feat/gps` diff; BLOCKING findings fixed and re-gated.

- **Depends on:** 10
- **Skills / agent:** pluto-reviewer agent
- **Done when:** no open BLOCKING findings.
- **Status:** todo
- **Result:**

### 12. Graph refresh & commit

**Description.** `graphify update .`, `python tools/graph_labels.py`, then `/pluto-commit`
( all-target build, version bump, pipeline promotion, CHANGELOG ). Commits go to the fork only.

- **Depends on:** 11
- **Skills / agent:** graphify, pluto-commit
- **Done when:** committed and pushed to `feat/gps` on `joshferns/MagisV2`.
- **Status:** todo
- **Result:**

## Decisions log

- 2026-10-13 [decision] Feature mode; target PRIMUS_X2_v1; branch `feat/gps`.
- 2026-10-13 [decision] Done = fix + data, no position hold; GPS hold is a later fusion topic.
- 2026-10-13 [decision] Consumers: flight log, user API, app ( MSP ), navigation.cpp.
- 2026-10-13 [decision] NAV-PVT preferred, legacy UBX messages kept as fallback.
- 2026-10-13 [decision] GPS off by default until bench-tested.
- 2026-10-13 [decision] GPS_HOLD / GPS_HOME and GPS_angle locked out in this topic.
- 2026-10-13 [decision] CS default PB7 ( `GPS_SPI_CS`, overridable ); PB6 rejected ( also RGB_2 ).
- 2026-10-13 [decision] GPS bus reads capped at 300 µs per poll; 10 Hz on SPI, 5 Hz on I2C.
- 2026-10-13 [decision] Validation: comms indoors, hand-held fix outdoors; no flight needed to close.
- 2026-10-13 [decision] Design option A, transport shim ( DESIGN.md ).
- 2026-10-13 [decision] Commits allowed by the user on the fork ( `joshferns/MagisV2` ) only.
- 2026-10-13 [assumption] u-blox M8 / M9 / M10 speaking UBX; configuration sent in both VALSET and legacy forms.
- 2026-10-13 [assumption] Breakout interface select set for SPI ( or I2C ) by the team.
- 2026-10-13 [risk] I2C driver does not detect NACK and recovers the IMU bus on timeout: probe once at boot, disable GPS if absent.
- 2026-10-13 [risk] SPI2 CR1 save / restore must be exact or optic flow / flash break.
- 2026-10-13 [out-of-scope] GPS / IMU fusion and GPS position hold.
