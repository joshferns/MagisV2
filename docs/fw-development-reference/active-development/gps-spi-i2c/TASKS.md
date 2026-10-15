# GPS over SPI / I2C - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md)

## Resume

| | |
|---|---|
| **Current task** | 5 - Lock out GPS hold / home and GPS_angle |
| **Next step** | Task 5: lock out GPS_HOLD / GPS_HOME and keep GPS_angle at 0; confirm MSP GPS frames |
| **Open questions** | module part number, D_SEL, PB7 free, ESP or ELRS mode ( team ) |
| **Blocked on** | nothing for tasks 1-8; hardware for task 9 |
| **Last updated** | 2026-10-15 |

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
| 1 | Bus transport driver ( SPI2 + I2C1 ) | done 2026-10-14 | - | pluto-driver, pluto-rules, c-pro |
| 2 | gps.cpp on the bus transport | done 2026-10-15 | 1 | pluto-rules, cpp-pro |
| 3 | NAV-PVT parser | done 2026-10-15 | 2 | pluto-rules |
| 4 | Module configuration over SPI / I2C | done 2026-10-15 | 2, 3 | pluto-rules, spec-to-code-compliance |
| 5 | Lock out GPS hold / home and GPS_angle; MSP check | in-progress | 2 | pluto-rules |
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
- **Status:** done 2026-10-14
- **Result:** `b0c28aa` + review fixes. SPI 648 B / I2C 288 B object; reads ~160-200 µs ( SPI 64 B ) and ~260 µs ( I2C 8 B ) by calculation; builds clean. As built: probe is inside `gpsBusInit()` ( no separate detect ), SPI reads a fixed 64 B ( 0xFF fill goes to the parser ), I2C reads 8 B from 0xFF without the 0xFD/0xFE count, `gpsBusWrite()` returns SPI's clocked-back bytes, I2C writes need ≥ 2 B. Reviewer: no blocking; 5 should-fix fixed ( LSBFIRST forced, I2C stops on first failure, SPI probe repeatable, ≥ 2 B write rule documented, I2C rate below ).

### 2. gps.cpp on the bus transport

**Description.** Under `GPS_SPI || GPS_I2C`: `gpsInit()` calls `gpsBusInit()` / `gpsBusDetect()`
instead of the serial-port lookup ( `featureClear(FEATURE_GPS)` if absent ), forces
`GPS_UBLOX`, and starts the state machine at configuration; baud negotiation, NMEA and
passthrough are compiled out ( passthrough also gets the missing `gpsPort` NULL check ).
`gpsThread()` polls every 20 ms on SPI and every 10 ms on I2C ( 8 B / 10 ms = 800 B/s ) and feeds `gpsNewData()` from `gpsBusRead()`, plus the bytes `gpsBusWrite()` clocks back on SPI. On SPI a missed probe is retried by the reconfigure-on-timeout loop ( reads one a second until a frame parses ); on I2C GPS stays off. Check whether
the USART2 `FUNCTION_GPS` default ( config.cpp:336 ) still claims the port in ESP mode; if it must
go, bump `EEPROM_CONF_VERSION`.

- **Depends on:** 1
- **Skills / agent:** pluto-rules; cpp-pro
- **Files:** [gps.cpp](../../../../src/main/io/gps.cpp), [config.cpp](../../../../src/main/config/config.cpp), [mw.cpp](../../../../src/main/mw.cpp)
- **Safety impact:** none in flight while modes are locked out ( task 5 ); boot path changes when GPS is on.
- **Done when:** builds; serial GPS build ( `GPS` alone ) unchanged; no unbounded loop in `gpsThread()`.
- **Rollback:** comment out `GPS_SPI` / `GPS_I2C`.
- **Status:** done 2026-10-15
- **Result:** `3dddd36` + review fix. Bus path skips the serial port, forces UBX, sends setup in ≤ `GPS_BUS_WRITE_MAX` pieces ( no 1-byte tail ), one transfer per call at 20 ms ( SPI ) / 10 ms ( I2C ), reads once a second until a frame parses. SPI 107.6 KB / I2C 107.2 KB, no new warnings. Reviewer: no blocking; search-rate fix applied.

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
- **Status:** done 2026-10-15
- **Result:** `ad803ec` + review notes. NAV-PVT decoded via `io/gps_ubx_pvt.h` ( host test: all checks pass ), class check, ACK / NAK counters, legacy messages suppressed while PVT is current, new globals reset on loss. +0.6 KB flash, no new warnings. Reviewer: no blocking, no should-fix.

### 4. Module configuration over SPI / I2C

**Description.** Replace the serial init sequence for the bus path: CFG-VALSET ( RAM layer ) for
M9 / M10 and the legacy CFG-PRT ( SPI port 4 or DDC port 0, UBX out only ), CFG-MSG ( NAV-PVT every
fix, legacy messages ), CFG-RATE ( 100 ms on SPI, 200 ms on I2C ) and CFG-NAV5 for M8. One message
per `gpsThread()` call; on I2C split into ≤ 10 B writes with no 1-byte tail. **I2C output is NAV-PVT only at 5 Hz** ( 500 B/s against 800 B/s read capacity ); SPI keeps the legacy messages as well. Checksums checked against the u-blox interface description.

- **Depends on:** 2, 3
- **Skills / agent:** pluto-rules; spec-to-code-compliance against the u-blox M8 / M10 interface descriptions
- **Files:** [gps.cpp](../../../../src/main/io/gps.cpp)
- **Safety impact:** none.
- **Done when:** builds; every UBX frame's checksum verified by a host-side script.
- **Rollback:** revert the commit.
- **Status:** done 2026-10-15
- **Result:** `147ad0b` + review fixes. SPI list 174 B / 8 frames ( VALSET, CFG-PRT port 4, MSG PVT / POSLLH / STATUS / VELNED, RATE 100 ms, NAV5 dyn 6 ); I2C list 179 B / 10 frames ( VALSET, NMEA off ×6, MSG PVT, RATE 200 ms, NAV5 ), no DDC CFG-PRT. Frames generated and checked by script ( all valid ); VALSET keys checked against the key table. No new warnings on any GPS build vs pre-topic. Reviewer: no blocking; 2 should-fix applied.

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
- **Status:** in-progress
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
- 2026-10-14 [decision] I2C: 8 B per read ( 300 µs cap ), poll every 10 ms, NAV-PVT only at 5 Hz; GPS stops on the first I2C failure ( each costs a ~10 ms timeout and an IMU-bus reset ).
- 2026-10-14 [decision] SPI CR1 switch also forces MSB-first.
- 2026-10-15 [decision] ( replaces the 2026-10-14 "SPI probe repeatable" note ) SPI probe runs once at boot; a miss is retried by the reconfigure-on-timeout loop ( ~2.5 s ), and SPI reads drop to one a second until a frame parses.
- 2026-10-15 [decision] USART2 keeps its `FUNCTION_GPS` default; the bus path never opens it, so no `EEPROM_CONF_VERSION` bump.
- 2026-10-15 [decision] Bus setup lists use dynamic model 6 ( airborne < 1 g ) through a NAV5 / VALSET that changes only that field; the serial path keeps pedestrian.
- 2026-10-15 [decision] Configuration goes to the RAM layer only ( VALSET layer RAM, no CFG-CFG save ): the firmware reconfigures after every boot and every communication loss.
- 2026-10-15 [decision] I2C: the DDC port is never reconfigured from itself ( CFG-PRT ); NMEA off per message instead. SPI drops NAV-SOL ( 64 % of read capacity at 10 Hz ).
- 2026-10-15 [decision] Warning check for GPS builds: gate output compared with a serial-GPS build of `e766113`; must add no entries.
- 2026-10-15 [risk] Found: `updateGpsWaypointsAndMode()` forces BOXGPSHOLD on and holds towards hard-coded `GPS_hold` coordinates ( since `8b06067` ). Locked out by `GPS_NAV_MODES_LOCKED`; the fusion topic must remove the forced box and the hard-coded point before lifting the lock.
- 2026-10-15 [out-of-scope] u-blox M7 ( protocol 14, 84-byte NAV-PVT ): rejected by the length check; M7 modules would fall back to the legacy messages on SPI only.
- 2026-10-15 [risk] `autoConfig` OFF on the bus path leaves the module on NMEA and the UBX parser never syncs ( default is ON ).
- 2026-10-14 [risk] `i2cReadDevice()` ( bus_i2c_stm32f30x.c:405 ) never sends a byte between its TXIS and TC waits, so every call times out and resets I2C1. Not used here; pre-existing, outside this topic.
