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
