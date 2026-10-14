# GPS over SPI / I2C - Design

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md)

## Options

| | A: transport shim ( **chosen** ) | B: fake `serialPort_t` | C: separate driver |
|---|---|---|---|
| How | New `drivers/gps_ublox_bus.c`: `gpsBusInit()`, `gpsBusRead(buf, max)`, `gpsBusWrite(buf, len)` over SPI2 or I2C1. `gps.cpp` skips the serial-port lookup, baud negotiation and NMEA under `GPS_SPI` / `GPS_I2C` and goes straight to UBX configuration. | SPI/I2C wrapped as a `serialPort_t` vTable so `gps.cpp` is almost untouched. | Standalone driver with its own init and timeouts feeding `gpsNewFrame()`. |
| Files | `gps_ublox_bus.{c,h}`, `gps.cpp`, `target.h`, Makefile | new serial vTable file, `serial.h`, `gps.cpp` | new driver, `gps.cpp` ( duplicate state machine ) |
| Cost | ~+1 KB flash, ~64 B RAM | +64-256 B RAM ring buffer, no-op baud functions | duplicate state machine and timeout code |
| Loop | SPI ~0.15 ms per poll, I2C ≤ 0.3 ms per poll ( capped ) | same bus cost, baud states still run | same bus cost |
| Risk | low; parser unchanged | baud states run pointlessly; `serialTotalRxWaiting` is `uint8_t` | two state machines drift apart |

**Why A.** It reuses the parser and the state machine, removes the serial-only steps instead of
faking them, and keeps the bus code in one driver that `pluto-driver` and the maps can track.

## Key design points

- **SPI2 sharing.** Flash and PAW3903 run SPI Mode 3 at 9 MHz. u-blox SPI is Mode 0, ≤ 5.5 MHz.
  Every GPS access saves the SPI2 CR1 mode/prescaler, switches to Mode 0 / 4.5 MHz ( /8 ), drives
  `GPS_SPI_CS` ( PB7 ), and restores CR1 before returning. No ISR or DMA uses SPI2.
- **SPI read.** u-blox streams 0xFF when it has nothing to send. Read in chunks ( ≤ 64 B ) and stop
  a poll after the chunk or a run of 0xFF; writes clock in the module's output, so bytes read
  during a write are fed to the parser too.
- **I2C read.** 8 B per read from the `0xFF` stream ( empty reads as 0xFF, skipped by the parser ),
  ~260 µs at 400 kHz; no `0xFD/0xFE` count, which would push a read past 300 µs. Polled every 10 ms
  ( 800 B/s ), so I2C carries NAV-PVT only at 5 Hz. Writes 2-10 B; GPS stops on the first failure. The I2C driver does not detect a NACK and recovers the bus
  shared with the IMU on timeout, so the module is **probed once at boot** and GPS is disabled
  if it does not answer.
- **Rate.** `gpsThread()` polls every 20 ms on SPI, 10 ms on I2C ( it is called on every non-RX spin ).
  Hard cap per poll: 300 µs. A failed SPI probe is repeated about once a second.
- **Configuration.** Sent once per call, one message at a time: CFG-VALSET ( M9 / M10 ) and the
  legacy CFG-PRT / CFG-MSG / CFG-RATE / CFG-NAV5 ( M8 ). A module ignores or NAKs the set it does
  not support. Output: UBX only, NAV-PVT every fix; on SPI also legacy POSLLH / VELNED / SOL / STATUS
  where the module still has them. 10 Hz on SPI, 5 Hz PVT-only on I2C.
- **Parser.** Add NAV-PVT ( class 0x01, id 0x07, 92 B ) and a class check; PVT alone sets
  position, altitude, velocity, course, numSV, fix and both "new data" flags.
- **Lock-out.** With GPS on, GPS_HOLD / GPS_HOME cannot be engaged and `GPS_angle` is forced to 0
  until the fusion topic.
