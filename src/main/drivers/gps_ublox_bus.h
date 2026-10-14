/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\gps_ublox_bus.h                                    #
 #  Created Date: Wed, 14th Oct 2026                                           #
 #  Brief: u-blox GPS byte transport over SPI2 ( GPS_SPI ) or I2C1 ( GPS_I2C ) #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Wed, 14th Oct 2026                                          #
 #  Modified By: Claude                                                        #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-14	Claude	First version ( gps-spi-i2c task 1 )                       #
*******************************************************************************/
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined( GPS ) && ( defined( GPS_SPI ) || defined( GPS_I2C ) )
  #define GPS_ON_BUS
#endif

#ifdef GPS_ON_BUS

  // Bytes returned by one gpsBusRead ( ) call at most. Sized so one call stays under ~300 us:
  // SPI at 4.5 MHz ~3 us/byte, I2C at 400 kHz ~23 us/byte plus address and register bytes.
  // Writes are capped the same way; the caller splits longer UBX messages into several
  // writes ( the module's input is a byte stream, so a message may span writes ).
  #ifdef GPS_SPI
    #define GPS_BUS_READ_MAX  64
    #define GPS_BUS_WRITE_MAX 64
  #else
    #define GPS_BUS_READ_MAX  8
    #define GPS_BUS_WRITE_MAX 10
  #endif

typedef enum {
  GPS_BUS_NOT_INIT    = 0,
  GPS_BUS_OK          = 1,
  GPS_BUS_NOT_FOUND   = -2,    // no answer at boot; GPS disabled
  GPS_BUS_DISABLED    = -3,    // too many I2C errors in a row; stopped to protect the IMU bus
} gpsBusStatus_e;

extern int8_t gpsBusStatus;          // gpsBusStatus_e
extern uint16_t gpsBusErrorCount;    // failed transfers since boot
extern uint16_t gpsBusMaxReadUs;     // worst gpsBusRead ( ) / gpsBusWrite ( ) duration seen, us

// Sets up the chip-select pin ( SPI ) and probes for the module, blocking for at most
// ~200 ms. Returns true when a module answered.
bool gpsBusInit ( void );

// Reads up to max bytes ( capped at GPS_BUS_READ_MAX ) from the module into buf and returns
// how many were stored. Idle fill bytes ( 0xFF ) are returned too: the UBX parser skips them.
uint16_t gpsBusRead ( uint8_t *buf, uint16_t max );

// Sends len bytes ( at most GPS_BUS_WRITE_MAX ) to the module. On SPI the module clocks its own output back at the same
// time; those bytes are stored in rxBuf ( which must hold len bytes ) and their count is
// returned, so the caller can feed them to the parser. rxBuf may be NULL to discard them.
// Returns 0 on I2C or on failure.
uint16_t gpsBusWrite ( const uint8_t *buf, uint16_t len, uint8_t *rxBuf );

#endif

#ifdef __cplusplus
}
#endif
