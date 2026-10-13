# GPS over SPI / I2C - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md)

( pluto-scout output, 13 Oct 2026, verbatim. Line numbers refer to `e766113`. )

## Scout brief: u-blox GPS on SPI2 (default) or I2C1, feeding the existing UBX parser, with IMU fusion later
Graph: stale (built cbd1c72a, HEAD e766113). The `graphify` CLI is not installed, so I searched `graphify-out/graph.json` directly, then used grep.

**Subsystems:** GPS (io/gps), Hardware Bus, Optic-flow position. Pipeline docs: `subsystems/Hardware_Bus_Pipeline.md`. No GPS pipeline doc exists, and `Firmware_Pipeline.md` does not mention GPS.

**Key code**
- `src/main/io/gps.cpp:204` `gpsInit`: finds the serial port (`findSerialPortConfig(FUNCTION_GPS)`, :220) and opens it (`openSerialPort`, :241). If either fails it calls `featureClear(FEATURE_GPS)`.
- `gps.cpp:305` `gpsInitUblox`: needs `isSerialTransmitBufferEmpty` (:310), changes baud (:327, :341), sends `serialPrint` PUBX (:332), then `serialWrite` one byte per call for `ubloxInit` (:359) and SBAS (:369).
- `gps.cpp:396` `gpsThread`: reads bytes with `while(serialRxBytesWaiting) gpsNewData(serialRead)` (:399-400), then runs the state machine.
- `gps.cpp:966` `gpsNewFrameUBLOX` / `:900` `UBLOX_parse_gps`: parse one byte at a time. They take no port argument, so the parser itself does not need to change.
- `mw.cpp:1283` calls `gpsThread()` on every non-RX spin of `loop()`, with no rate limit.
- `drivers/bus_spi.c:139` `initSpi2`: Mode 3 (CPOL High, CPHA 2Edge), prescaler /4. SPI2 runs off APB1 (36 MHz), so that is 9 MHz.
- `drivers/bus_i2c_stm32f30x.c:331` `i2cRead`: `len` is a `uint8_t` (at most 255), AutoEnd, polled.
- `flight/opticflow.cpp:320-391` `updateVelocity` / `updatePosition`: the existing fusion structure.

**Mode:** feature

**Current behaviour:** the serial-only GPS path converts each byte into the globals `GPS_coord`, `GPS_numSat` and `STATE(GPS_FIX)`, then calls `onGpsNewData()` (navigation.cpp:260). That call happens only after both a POSLLH and a VELNED frame have arrived (:956). Consumers of the GPS data:
- MSP_RAW_GPS / COMP_GPS (serial_msp.cpp:1146).
- `GPS_angle` is added into the angle error of every PID controller (pid.cpp:153…787).
- `GPS_HOLD/HOME_MODE` (mw.cpp:1401).
- No public API header exposes GPS. `user_GPS_coord` / `app_GPS_coord` exist only in API-Utils.

### Analysis
**Q1 – the seam.** Every serial dependency sits in four places: `gpsInit` (:220-245), `gpsInitUblox` (:310, :325-341, :332, :359, :369), `gpsThread` (:399-400) and `gpsEnablePassthrough` (:1051, CLI). That last one dereferences `gpsPort` without a NULL check.
- The parser handles POSLLH, STATUS, SOL, VELNED and SVINFO.
- It never checks `_class` (:905).
- It has **no NAV-PVT** (0x01/0x07, 92 bytes).

What I expect from u-blox interface knowledge (not checked against a datasheet): M9/M10 drop NAV-SOL, so `GPS_numSat` would stay 0, and M10 rejects the legacy CFG-MSG, CFG-RATE and CFG-NAV5 messages, so it needs CFG-VALSET.

Adding PVT would touch these parts of gps.cpp, with no RAM change because the payload union is already 344 B:
- a struct in the `_buffer` union (:883);
- `MSG_PVT=0x07` and a class check;
- a case that sets position, altitude, numSV, fix, speed and course, and both `_new_*` flags;
- a CFG-MSG entry in `ubloxInit` (:127).

**Q2 – SPI2.**
- The M25P16 flash uses `spiTransfer` directly (flash_m25p16.cpp:102-303) and keeps the init mode. Its divisor change is commented out (:205).
- The PAW3903 goes through the API `SPI_Read`/`SPI_Write` (API-Src/Serial-IO-SPI.cpp:146-210). That API hardcodes CS to PB5 (:30) and adds 50-200 µs `delayMicroseconds` per access. `initOpticFlow` never calls `SPI_Init(mode,…)`, so both devices run Mode 3 at 9 MHz.
- A third device needs its own CS GPIO plus a save/restore of mode and prescaler. A u-blox port defaults to Mode 0 and tops out around 5.5 MHz (datasheet), so it needs /8 = 4.5 MHz.
- There are no SPI2 calls from an ISR and no DMA. BLACKBOX and OPTIC_FLOW are commented out in target.h:154/179. Flash is touched only at init (main.cpp:450) and by MSP dataflash commands.
- Each `spiTransferByte` wait gives up after 1000 polls, so SPI cannot hang.

**Q3 – I2C.**
- Calls that fit u-blox DDC: `i2cRead(0x42,0xFD,2,…)` to read bytes available, then `i2cRead(0x42,0xFF,n≤255,…)`. Writes go through `i2cWriteBufferwithoutregister`.
- Each wait gives up after 0x10000 polls (several ms). It then runs `i2cRecoverPeripheral` (:113), which re-inits the port **and unsticks the bus that the IMU shares**.
- A NACK at 0x42 is not detected. The driver waits for TXIS until the timeout, so a missing module would reset the bus on every poll. Probe once at init and disable GPS if the module does not answer.
- Cost: about 23 µs per byte at 400 kHz, so a 32-byte chunk is about 0.75 ms of blocking.
- `imuUpdate` runs every `looptime`=3500 µs (mw.cpp:1291-1294, config.cpp:518).

**Q4 – reachability.** `FEATURE_GPS` is on by default (config.cpp:415) and **USART2 is `FUNCTION_GPS` by default** (config.cpp:336).
- In ESP mode, gpsInit would open USART2 at 9600.
- In ELRS mode, RxConfig.cpp:156-157 moves ELRS to USART1 and MSP to USART2. GPS then finds no port and `featureClear(FEATURE_GPS)`, so `gpsThread` never runs.
- The default `provider` is `GPS_NMEA` (config.cpp:510).
- The SPI/I2C path must skip the port lookup and force UBX.

**Q7 – options** (current GPS build is 108.9 KB of 256 KB):
- **A (recommended): a transport shim inside gps.cpp,** chosen by `#if GPS_SPI/GPS_I2C`. `gpsTransportRead(buf,max)` and `gpsTransportWrite(buf,len)` go in a new `drivers/gps_ublox_bus.c` registered in the Makefile.
  - gpsInit skips the serial lookup; the state machine jumps to `GPS_CONFIGURE` and writes one whole UBX message per call.
  - gpsThread rate-limits itself to one poll every 20-25 ms and reads 32-64 B per poll. It stops on a run of 0xFF on SPI, or on a count of 0 on I2C.
  - The NMEA, baud and passthrough code can be compiled out.
  - Cost: about +0.8-1.2 KB flash and 64 B RAM. Loop cost: SPI about 0.15 ms per poll, I2C about 0.75-1.5 ms per poll.
  - No API impact.
- **B: a fake `serialPort_t` vTable** (serial.h:66). gps.cpp stays nearly untouched, but this needs a ring buffer (+64-256 B), no-op baud functions and vTable wiring. `serialTotalRxWaiting` returns `uint8_t`. The baud-negotiation states still run pointlessly.
- **C: a separate driver that calls the public `gpsNewFrame` (gps.h:160)** with its own init. This duplicates the state machine and the timeout handling.

**Q8 – fusion.**
- opticflow.cpp keeps `velocity_hf[3]` (cm/s, level heading frame, :136). It is accel-integrated and complementary-corrected (:325, :358).
- It also keeps `PositionX/Y` in the arm-heading frame (:155, :390), with `opticFlowEarthToBody` at :165.
- posEstimate.cpp `PosXYEstimate` (:237) serves UWB.
- GPS NED (VELNED or PVT velN/E) would need to be rotated by the yaw measured at arm and fed into the same correction step.

### Constraints / invariants that apply
- **pluto-rules:117-119:** no blocking I2C inside an ISR. I2C is polled, so each transaction blocks the loop.
- **pluto-rules:195:** no unbounded loop in the control path. That rules out a `while` drain without a chunk cap.
- **CLAUDE.md:** register every new .c/.cpp file in a Makefile group, gate the code on target.h defines, check the maps when touching pins (pluto-rules:30), and keep `-Wconversion` clean.
- Changing the default config or EEPROM layout needs an `EEPROM_CONF_VERSION` bump (107, config.cpp:153).

### Resource conflicts
- **PB7:** user `GPIO_4` (Peripheral-GPIO.cpp:48), `PWM_3` TIM4_CH2 (TIMER_MAP:101), and alternate I2C1_SDA / USART1_RX (PIN_MAP:42). It is not an RGB pin option. No DMA is needed.
- **PB6:** additionally `RGB_2` (RGB-LED.h:118, DMA1_Ch1), so PB7 is the better choice.
- **I2C:** none; 0x42 is a free address.

### Already answered by the code
- SPI2 already runs Mode 3 for both existing devices.
- SPI has no DMA or ISR users.
- The I2C read length limit is 255.
- GPS is cleared under ELRS.
- No public API header carries GPS.

### Questions for the user
1. Which module or breakout is it (M8, M9 or M10)? This decides between legacy CFG messages and VALSET, and whether NAV-PVT is needed.
2. Is the breakout's D_SEL / interface-select set to SPI, and are 3.3 V logic and power available?
3. Is PB7 free on the team's wiring, i.e. does no user code use GPIO_4 or PWM_3?
4. Will OPTIC_FLOW and GPS fly together? If so, SPI2 timing and CS sharing with the PAW3903 matter.
5. Which receiver mode is used, ESP or ELRS? This affects whether the USART2 `FUNCTION_GPS` default has to go.
6. Is a 5 Hz update rate enough, or do you want 10 Hz?
7. Should GPS data go out over MSP_RAW_GPS to the app, or into a new user API?

### Doc drift
- `Hardware_Bus_Pipeline.md:12-14`: names the types `spiDevice_t` and `i2cError_e`, which do not exist. The code has `I2CDevice` (bus_i2c.h).
- `Hardware_Bus_Pipeline.md:17-18`: says the buses configure DMA and offer "non-blocking" transfers. Both are polled and blocking, with no DMA in either bus_spi.c or bus_i2c_stm32f30x.c.
- `active-development/optic-flow-position-hold/README.md:81`: says USART2 carries ELRS. It actually carries MSP under ELRS (RxConfig.cpp:157). The brief's own starting facts said the same and need the same correction.
- `active-development/optic-flow-position-hold/README.md:86`: says "no port is assigned FUNCTION_GPS, FEATURE_GPS off". The code has config.cpp:336 and :415.

### Prior work
- `active-development/optic-flow-position-hold/` ("GPS: open items").
- Commits 578eb8e and e766113. The last commit to bus_i2c was 43fb651 (I2C recovery).

### Suggested skills
pluto-rules, pluto-driver (pins and CS), pluto-build `--gate`, the c-pro agent for the bus driver, pluto-flighttest, pluto-commit.

### Pipeline docs to update at the end
- `Hardware_Bus_Pipeline.md`: add the third SPI2 device and the mode/prescaler switching; fix the drift listed above.
- A new GPS subsystem doc, linked from `Firmware_Pipeline.md`.
- `PIN_MAP.md` and `TIMER_MAP.md`: PB7 becomes GPS_CS.
- Update the active-topic README.
