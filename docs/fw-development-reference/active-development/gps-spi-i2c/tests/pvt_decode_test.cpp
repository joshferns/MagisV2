// Host-side check of src/main/io/gps_ubx_pvt.h ( gps-spi-i2c task 3 ).
// Build and run from the repo root:
//   g++ -std=gnu++17 -Wall -Wextra -Wconversion -I src/main <this file> -o /tmp/pvt_test && /tmp/pvt_test
// Builds a NAV-PVT frame byte by byte at the documented offsets ( u-blox M8 / M10 interface
// description, UBX-NAV-PVT ), checks the frame checksum, then decodes the payload.
#include <cstdio>
#include <cstring>
#include <cstdint>

#include "io/gps_ubx_pvt.h"

static int failures = 0;
#define CHECK( cond ) do { if ( ! ( cond ) ) { std::printf ( "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond ); failures++; } } while ( 0 )

static void put32 ( uint8_t *p, uint32_t v ) { for ( int i = 0; i < 4; i++ ) p [ i ] = ( uint8_t ) ( v >> ( 8 * i ) ); }
static void put16 ( uint8_t *p, uint16_t v ) { p [ 0 ] = ( uint8_t ) v; p [ 1 ] = ( uint8_t ) ( v >> 8 ); }

int main ( ) {
  uint8_t frame [ 6 + UBX_NAV_PVT_LEN + 2 ] = { 0xB5, 0x62, UBX_CLASS_NAV, UBX_MSG_NAV_PVT, UBX_NAV_PVT_LEN, 0 };
  uint8_t *p = frame + 6;

  put32 ( p + 0, 123456789u );                 // iTOW
  p [ 20 ] = 3;                                // fixType 3D
  p [ 21 ] = 0x01;                             // gnssFixOK
  p [ 23 ] = 11;                               // numSV
  put32 ( p + 24, ( uint32_t ) 739812345 );    // lon  73.9812345 deg
  put32 ( p + 28, ( uint32_t ) 189876543 );    // lat  18.9876543 deg
  put32 ( p + 36, ( uint32_t ) 560123 );       // hMSL 560.123 m
  put32 ( p + 40, 2345u );                     // hAcc 2.345 m
  put32 ( p + 44, 4567u );                     // vAcc 4.567 m
  put32 ( p + 48, ( uint32_t ) 1234 );         // velN  1.234 m/s
  put32 ( p + 52, ( uint32_t ) -567 );         // velE -0.567 m/s
  put32 ( p + 56, ( uint32_t ) -89 );          // velD -0.089 m/s ( climbing )
  put32 ( p + 60, ( uint32_t ) 1358 );         // gSpeed 1.358 m/s
  put32 ( p + 64, ( uint32_t ) 33456789 );     // headMot 334.56789 deg
  put16 ( p + 76, 156 );                       // pDOP 1.56

  uint8_t ckA = 0, ckB = 0;
  for ( int i = 2; i < 6 + UBX_NAV_PVT_LEN; i++ ) { ckA = ( uint8_t ) ( ckA + frame [ i ] ); ckB = ( uint8_t ) ( ckB + ckA ); }
  frame [ 6 + UBX_NAV_PVT_LEN ]     = ckA;
  frame [ 6 + UBX_NAV_PVT_LEN + 1 ] = ckB;

  // the struct must line up with the wire offsets
  ubx_nav_pvt pvt;
  std::memcpy ( &pvt, p, sizeof ( pvt ) );
  CHECK ( pvt.iTOW == 123456789u );
  CHECK ( pvt.numSV == 11 );
  CHECK ( pvt.pDOP == 156 );

  gpsPvt_t out;
  CHECK ( ubxDecodePvt ( &pvt, UBX_NAV_PVT_LEN, &out ) );
  CHECK ( out.lat == 189876543 );
  CHECK ( out.lon == 739812345 );
  CHECK ( out.altitudeCm == 56012 );
  CHECK ( out.velNedCm [ 0 ] == 123 );
  CHECK ( out.velNedCm [ 1 ] == -56 );
  CHECK ( out.velNedCm [ 2 ] == -8 );
  CHECK ( out.groundSpeedCm == 135 );
  CHECK ( out.courseDeci == 3345 );
  CHECK ( out.hAccCm == 234 );
  CHECK ( out.vAccCm == 456 );
  CHECK ( out.pDop == 156 );
  CHECK ( out.numSat == 11 );
  CHECK ( out.fixType == 3 );
  CHECK ( out.fix3d );

  // no gnssFixOK -> no 3D fix even with fixType 3
  pvt.flags = 0;
  CHECK ( ubxDecodePvt ( &pvt, UBX_NAV_PVT_LEN, &out ) && ! out.fix3d );

  // wrong payload length is rejected
  CHECK ( ! ubxDecodePvt ( &pvt, 84, &out ) );

  // saturation and course wrap
  pvt.hAcc    = 0xFFFFFFFFu;
  pvt.velN    = 400000;     // 400 m/s -> clamps at 32767 cm/s
  pvt.headMot = -1000000;   // -10 deg -> 350.0
  CHECK ( ubxDecodePvt ( &pvt, UBX_NAV_PVT_LEN, &out ) );
  CHECK ( out.hAccCm == 65535 );
  CHECK ( out.velNedCm [ 0 ] == 32767 );
  CHECK ( out.courseDeci == 3500 );

  std::printf ( "frame checksum %02X %02X, %s ( %d failures )\n", ckA, ckB, failures ? "FAILED" : "all checks passed", failures );
  return failures ? 1 : 0;
}
