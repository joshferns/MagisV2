// Do not remove the include below
#include "PlutoPilot.h"

// GPS bench / outdoor log ( gps-spi-i2c task 7 ): uncomment to print GPS fields in Developer Mode.
// Temporary diagnostics - see docs/fw-development-reference/active-development/gps-spi-i2c/TESTING.md.
// #define GPS_BENCH_LOG

#ifdef GPS_BENCH_LOG
  #include <math.h>
  #include "platform.h"    // target defines: GPS, GPS_SPI, GPS_I2C

  #if defined( GPS ) && ( defined( GPS_SPI ) || defined( GPS_I2C ) )
extern "C" {
extern int8_t gpsBusStatus;          // 1 ok, 0 not init, -2 not found, -3 disabled after an I2C error
extern uint16_t gpsBusErrorCount;    // failed bus transfers since boot
extern uint16_t gpsBusMaxReadUs;     // worst bus transfer, us
}
  #endif
  #ifdef GPS
extern "C" {
extern uint16_t GPS_ubxAckCount;     // UBX-ACK-ACK ( configuration accepted )
extern uint16_t GPS_ubxNakCount;     // UBX-ACK-NAK ( configuration rejected )
}
  #endif

static bool gpsLogRestart   = false;
static uint32_t gpsLogLastMs = 0;
static uint32_t gpsLogTick   = 0;
static bool gpsLogHaveOrigin = false;
static int32_t gpsLogOriginLat, gpsLogOriginLon;    // deg * 1e7, first 3D fix

// One record every 100 ms. Every tenth record is a status line in place of the solution line,
// so no tick prints more than ~110 bytes ( Monitor_Print budget ~130 B / tick ).
static void gpsBenchLog ( void ) {
  uint32_t now = millis ( );
  if ( now - gpsLogLastMs < 100 ) {
    return;
  }
  gpsLogLastMs = now;
  gpsLogTick++;

  if ( gpsLogRestart ) {    // Developer Mode ( re ) started: marker on its own tick
    gpsLogRestart = false;
    Monitor_Println ( "GPSLOG_START t:", ( int ) now );
    return;
  }

  if ( gpsLogTick % 10 == 0 ) {
    // status line, ~75 B
    Monitor_Print ( "t:", ( int ) now );
    Monitor_Print ( " Av:", ( int ) Gps_Get ( Gps_Available ) );
  #if defined( GPS ) && ( defined( GPS_SPI ) || defined( GPS_I2C ) )
    Monitor_Print ( " Bs:", ( int ) gpsBusStatus );
    Monitor_Print ( " Be:", ( int ) gpsBusErrorCount );
    Monitor_Print ( " Bu:", ( int ) gpsBusMaxReadUs );
  #endif
  #ifdef GPS
    Monitor_Print ( " Ak:", ( int ) GPS_ubxAckCount );
    Monitor_Println ( " Nk:", ( int ) GPS_ubxNakCount );
  #else
    Monitor_Println ( "" );
  #endif
    return;
  }

  // solution line, ~110 B
  int32_t lat = Gps_Get ( Gps_Latitude );
  int32_t lon = Gps_Get ( Gps_Longitude );
  if ( ! gpsLogHaveOrigin && Gps_Get ( Gps_Fix_3D ) ) {
    gpsLogOriginLat  = lat;
    gpsLogOriginLon  = lon;
    gpsLogHaveOrigin = true;
  }
  int32_t northCm = 0, eastCm = 0;    // from the first 3D fix; 1e-7 deg of latitude = 1.1132 cm
  if ( gpsLogHaveOrigin ) {
    float cosLat = cosf ( ( float ) gpsLogOriginLat * 1.0e-7f * 0.0174532925f );
    northCm      = ( int32_t ) ( ( float ) ( lat - gpsLogOriginLat ) * 1.1132f );
    eastCm       = ( int32_t ) ( ( float ) ( lon - gpsLogOriginLon ) * 1.1132f * cosLat );
  }

  Monitor_Print ( "t:", ( int ) now );
  Monitor_Print ( " n:", ( int ) gpsLogTick );
  Monitor_Print ( " Fx:", ( int ) Gps_Get ( Gps_Fix_Type ) );
  Monitor_Print ( " Sv:", ( int ) Gps_Get ( Gps_Satellites ) );
  Monitor_Print ( " hA:", ( int ) Gps_Get ( Gps_H_Accuracy ) );
  Monitor_Print ( " dN:", ( int ) northCm );
  Monitor_Print ( " dE:", ( int ) eastCm );
  Monitor_Print ( " vN:", ( int ) Gps_Get ( Gps_Vel_North ) );
  Monitor_Println ( " vE:", ( int ) Gps_Get ( Gps_Vel_East ) );
}
#endif

/**
 * Configures Pluto's receiver mode.
 * AUX channel configurations for ELRS:
 * ARM mode      : Rx_AUX1, range 1300 to 2100 (2-pos switch)
 * ANGLE mode    : Rx_AUX2, range 1300 to 2100 (3-pos switch: mid+high = ANGLE, low = ACRO)
 * MAG mode      : Rx_AUX3, range 1500 to 2100
 * DEV mode      : Rx_AUX4, range 1500 to 2100
 * ALT HOLD / THROTTLE mode : Rx_AUX5, range 1500 to 2100
 *                            (2-pos switch: low = THROTTLE mode, high = ALT HOLD mode)
 */
void plutoRxConfig ( void ) {
  // Receiver mode: Uncomment one line matching your setup.
  Receiver_Mode ( Rx_ESP );    // Onboard ESP
  // Receiver_Mode ( Rx_CAM );    // WiFi CAMERA
  // Receiver_Mode ( Rx_PPM );    // PPM based
  // Receiver_Mode ( Rx_ELRS );      // ExpressLRS (CRSF) on USART1
}

// The setup function is called once at Pluto's hardware startup
void plutoInit ( void ) {
  // Add your hardware initialization code here
}

// The function is called once before plutoLoop when you activate Developer Mode
void onLoopStart ( void ) {
  // do your one time stuffs here
#ifdef GPS_BENCH_LOG
  gpsLogRestart = true;
#endif
}

// The loop function is called in an endless loop
void plutoLoop ( void ) {
#ifdef GPS_BENCH_LOG
  gpsBenchLog ( );
#endif
}

// The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish ( void ) {
  // do your cleanup stuffs here
}