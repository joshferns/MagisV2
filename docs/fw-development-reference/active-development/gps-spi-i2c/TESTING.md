# GPS over SPI / I2C - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

Status: **procedure written, not run** ( needs the team's GPS breakout ). Results go in the tables
below; logs in `logs/log-N.txt` ( one file per run, never overwritten ).

## Set-up

1. **Wiring** ( 3.3 V logic and supply; check the breakout's interface jumper / D_SEL first ):

   | Bus | Breakout pin → board | Notes |
   |---|---|---|
   | SPI | SCK → PB13, MISO → PB14, MOSI → PB15, CS → **PB7**, 3V3, GND | SPI2, shared with the flash and the optic-flow sensor. PB7 is then not `PWM_3` / `GPIO_4` in user code. |
   | I2C | SCL → PB8, SDA → PB9, 3V3, GND | I2C1, shared with the IMU. Remove the breakout's pull-ups if it has them and the bus already has its own. |

2. **`target.h`** ( PRIMUS_X2_v1 ): uncomment `#define GPS` and **one** of `#define GPS_SPI` /
   `#define GPS_I2C`. Change `GPS_SPI_CS_PIN` / `GPS_I2C_ADDR` if the wiring differs.
3. **`PlutoPilot.cpp`**: uncomment `#define GPS_BENCH_LOG`. Build, flash, and open PlutoMonitor in
   Developer Mode.

## Log fields

One record every 100 ms. Every 10th record is a status line instead of a solution line.

| Line | Field | Meaning |
|---|---|---|
| marker | `GPSLOG_START t:` | Developer Mode ( re )started at `t`. |
| solution | `t` | `millis ( )`, ms |
| | `n` | record counter ( steps by 2 across a lost record, by 1 across a stall ) |
| | `Fx` | fix type: 0 none, 3 3D ( airborne model gives no 2D ) |
| | `Sv` | satellites used |
| | `hA` | horizontal accuracy estimate, cm ( 65535 = unknown ) |
| | `dN`, `dE` | cm north / east of the first 3D fix ( 0 until a fix ) |
| | `vN`, `vE` | velocity north / east, cm/s |
| status | `Av` | 1 while the module sends data |
| | `Bs` | bus status: 1 ok, 0 not initialised, -2 not found at boot, -3 stopped after an I2C error |
| | `Be` | failed bus transfers since boot |
| | `Bu` | worst bus transfer, µs ( budget 300 ) |
| | `Ak`, `Nk` | UBX configuration messages accepted / rejected |

Bytes per tick: solution ≤ 125, status ≤ 88, marker ≤ 35 ( budget ~130 ).
Analyse with `python tools/flightlog.py summary logs/log-N.txt` and `table`.

## Tests

### A. Indoor communication ( props off, disarmed, no fix expected )

Power on, wait 30 s, save the log.

| Check | Pass | Result |
|---|---|---|
| `Av` 1 within 5 s of power-up | yes | |
| `Bs` | 1 | |
| `Be` | 0 ( I2C: any error stops GPS, `Bs` -3 ) | |
| `Bu` | ≤ 300 µs | |
| `Ak` | ≥ 4 ( M8: CFG-PRT / MSG / RATE / NAV5 accepted ) | |
| `Nk` | M8: ~1 ( VALSET ); M10: the legacy frames; M9: ~0 | |
| Records without the `t` field / app disconnects | none | |
| Configuration repeats ( `Ak` rising every ~2.5 s ) | no, once data flows | |

### B. Shared-bus check ( SPI with OPTIC_FLOW, or I2C )

Repeat A with `OPTIC_FLOW` + `PAW3903_SPI` ( SPI case ), or watch altitude hold / IMU on the bench ( I2C
case ). Pass: optic-flow values unchanged with and without GPS; no IMU dropouts; `Be` 0.

### C. Outdoor, hand-held ( clear sky, away from buildings )

1. Hold still for 5 min after power-up ( cold start ). Note time to first `Fx` 3.
2. Walk a measured straight line ( e.g. 20 m tape, north or east ) at a steady pace, stop 30 s, walk back.

| Check | Pass | Result |
|---|---|---|
| Time to first 3D fix ( cold ) | note it ( typically under a minute or two ) | |
| `Sv` with fix | ≥ 6 | |
| `hA` while still | note the range | |
| `dN` / `dE` wander while still ( 5 min ) | note the range ( metres expected ) | |
| Walked distance vs tape | within the wander | |
| `vN` / `vE` while walking | consistent with pace ( ~100-150 cm/s ), sign matches direction | |
| Update rate | SPI ~10 Hz, I2C ~5 Hz ( count changes of `dN` per second ) | |

### D. Loss and recovery

Unplug the module's supply for 5 s, reconnect.

| Bus | Pass |
|---|---|
| SPI | `Av` 0 after ~2.5 s; after reconnect `Ak` rises again ( reconfigured ) and `Av` 1 |
| I2C | `Bs` -3 and GPS stays off until power-up ( by design: protects the IMU bus ); flight unaffected |

## Results

( none yet )
