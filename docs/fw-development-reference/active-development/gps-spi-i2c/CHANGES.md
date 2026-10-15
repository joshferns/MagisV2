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
