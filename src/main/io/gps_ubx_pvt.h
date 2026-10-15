/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Project: MagisV2                                                           #
 #  File: \src\main\io\gps_ubx_pvt.h                                           #
 #  Created Date: Thu, 15th Oct 2026                                           #
 #  Brief: UBX-NAV-PVT payload layout and decoding ( no firmware dependencies, #
 #         so it can be built and checked on a host )                          #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 15th Oct 2026                                          #
 #  Modified By: Claude                                                        #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-15	Claude	First version ( gps-spi-i2c task 3 )                       #
*******************************************************************************/
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UBX_CLASS_NAV       0x01
#define UBX_MSG_NAV_PVT     0x07
#define UBX_NAV_PVT_LEN     92

#define UBX_PVT_FLAGS_FIX_OK 0x01    // flags bit 0: gnssFixOK
#define UBX_PVT_FIX_3D       3

// UBX-NAV-PVT payload, identical on u-blox M8 ( protocol 15+ ), M9 and M10. Every field sits at its
// natural alignment, so the struct has no padding and matches the wire layout byte for byte.
typedef struct {
  uint32_t iTOW;         // ms, GPS time of week
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  uint32_t tAcc;         // ns
  int32_t nano;          // ns
  uint8_t fixType;       // 0 none, 1 DR, 2 2D, 3 3D, 4 GNSS + DR, 5 time only
  uint8_t flags;         // bit 0 gnssFixOK
  uint8_t flags2;
  uint8_t numSV;
  int32_t lon;           // deg * 1e7
  int32_t lat;           // deg * 1e7
  int32_t height;        // mm above ellipsoid
  int32_t hMSL;          // mm above mean sea level
  uint32_t hAcc;         // mm
  uint32_t vAcc;         // mm
  int32_t velN;          // mm/s
  int32_t velE;          // mm/s
  int32_t velD;          // mm/s
  int32_t gSpeed;        // mm/s, ground speed ( 2D )
  int32_t headMot;       // deg * 1e5, heading of motion ( 2D )
  uint32_t sAcc;         // mm/s
  uint32_t headAcc;      // deg * 1e5
  uint16_t pDOP;         // * 0.01
  uint8_t reserved1 [ 6 ];    // M9 / M10: flags3 + reserved
  int32_t headVeh;       // deg * 1e5
  int16_t magDec;        // deg * 1e2
  uint16_t magAcc;       // deg * 1e2
} ubx_nav_pvt;

#ifdef __cplusplus
static_assert ( sizeof ( ubx_nav_pvt ) == UBX_NAV_PVT_LEN, "NAV-PVT struct must match the 92-byte payload" );
#else
_Static_assert ( sizeof ( ubx_nav_pvt ) == UBX_NAV_PVT_LEN, "NAV-PVT struct must match the 92-byte payload" );
#endif

// The NAV-PVT values in the units the firmware's GPS globals use.
typedef struct {
  int32_t lat;             // deg * 1e7
  int32_t lon;             // deg * 1e7
  int32_t altitudeCm;      // cm above mean sea level
  int16_t velNedCm [ 3 ];  // cm/s, north / east / down
  uint16_t groundSpeedCm;  // cm/s
  uint16_t courseDeci;     // deg * 10, 0-3599
  uint16_t hAccCm;         // cm, saturates at 65535
  uint16_t vAccCm;         // cm, saturates at 65535
  uint16_t pDop;           // * 0.01
  uint8_t numSat;
  uint8_t fixType;
  bool fix3d;              // gnssFixOK and a 3D fix
} gpsPvt_t;

static inline int16_t ubxMmToCm16 ( int32_t mm ) {
  int32_t cm = mm / 10;
  return ( int16_t ) ( cm > 32767 ? 32767 : ( cm < -32768 ? -32768 : cm ) );
}

static inline uint16_t ubxMmToCmU16 ( uint32_t mm ) {
  uint32_t cm = mm / 10u;
  return ( uint16_t ) ( cm > 65535u ? 65535u : cm );
}

// Converts a received NAV-PVT payload. Returns false if the payload length is wrong.
static inline bool ubxDecodePvt ( const ubx_nav_pvt *pvt, uint16_t payloadLength, gpsPvt_t *out ) {
  if ( payloadLength != UBX_NAV_PVT_LEN ) {
    return false;
  }

  out->lat           = pvt->lat;
  out->lon           = pvt->lon;
  out->altitudeCm    = pvt->hMSL / 10;
  out->velNedCm [ 0 ] = ubxMmToCm16 ( pvt->velN );
  out->velNedCm [ 1 ] = ubxMmToCm16 ( pvt->velE );
  out->velNedCm [ 2 ] = ubxMmToCm16 ( pvt->velD );
  out->groundSpeedCm = ( uint16_t ) ubxMmToCmU16 ( pvt->gSpeed > 0 ? ( uint32_t ) pvt->gSpeed : 0u );

  int32_t course = ( pvt->headMot / 10000 ) % 3600;    // deg * 1e5 -> deg * 10, wrapped to 0-3599
  if ( course < 0 ) {
    course += 3600;
  }
  out->courseDeci = ( uint16_t ) course;

  out->hAccCm  = ubxMmToCmU16 ( pvt->hAcc );
  out->vAccCm  = ubxMmToCmU16 ( pvt->vAcc );
  out->pDop    = pvt->pDOP;
  out->numSat  = pvt->numSV;
  out->fixType = pvt->fixType;
  out->fix3d   = ( pvt->flags & UBX_PVT_FLAGS_FIX_OK ) && ( pvt->fixType == UBX_PVT_FIX_3D );
  return true;
}

#ifdef __cplusplus
}
#endif
