/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\gps_ublox_bus.c                                    #
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

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "platform.h"

#include "drivers/gps_ublox_bus.h"

#ifdef GPS_ON_BUS

  #if defined( GPS_SPI ) && defined( GPS_I2C )
    #error "Define only one of GPS_SPI and GPS_I2C"
  #endif

  #include "drivers/system.h"

  #ifdef GPS_SPI
    #include "drivers/bus_spi.h"
  #else
    #include "drivers/bus_i2c.h"
  #endif

int8_t gpsBusStatus       = GPS_BUS_NOT_INIT;
uint16_t gpsBusErrorCount = 0;
uint16_t gpsBusMaxReadUs  = 0;

  #define GPS_BUS_PROBE_MS 200    // per call; the caller repeats the probe on SPI ( a booting module drops the poll )

static void noteDuration ( uint32_t startUs ) {
  uint32_t took = micros ( ) - startUs;
  if ( took > gpsBusMaxReadUs ) {
    gpsBusMaxReadUs = ( uint16_t ) ( took > 0xFFFFu ? 0xFFFFu : took );
  }
}

  /* ---------------------------------------------------------------- SPI2 */
  #ifdef GPS_SPI

    #ifndef GPS_SPI_CS_GPIO
      #define GPS_SPI_CS_GPIO GPIOB
    #endif
    #ifndef GPS_SPI_CS_PIN
      #define GPS_SPI_CS_PIN GPIO_Pin_7
    #endif
    #ifndef GPS_SPI_CS_RCC
      #define GPS_SPI_CS_RCC RCC_AHBPeriph_GPIOB
    #endif
    #define GPS_SPI_INSTANCE SPI2

    // u-blox SPI: mode 0 ( CPOL 0, CPHA 0 ), at most 5.5 MHz. SPI2 runs off APB1 ( 36 MHz ), so
    // prescaler 8 gives 4.5 MHz. The flash and the PAW3903 use mode 3 at 9 MHz: every GPS access
    // saves CR1, switches, and restores it before returning.
    // LSBFIRST is in the mask too: the user SPI API can set LSB-first on SPI2.
    #define GPS_SPI_CR1_MASK ( ( uint16_t ) ( SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR | SPI_CR1_LSBFIRST ) )
    #define GPS_SPI_CR1_BITS ( ( uint16_t ) SPI_BaudRatePrescaler_8 )

static uint16_t savedCr1;

// UBX-MON-VER poll: any u-blox answers it on every enabled port.
static const uint8_t ubxMonVerPoll [] = { 0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34 };

static void spiWaitIdle ( void ) {
  uint16_t timeout = 1000;
  while ( ( SPI_I2S_GetFlagStatus ( GPS_SPI_INSTANCE, SPI_I2S_FLAG_BSY ) == SET ) && timeout ) {
    timeout--;
  }
}

static void spiSetCr1 ( uint16_t cr1 ) {
  spiWaitIdle ( );
  GPS_SPI_INSTANCE->CR1 = ( uint16_t ) ( GPS_SPI_INSTANCE->CR1 & ( uint16_t ) ~SPI_CR1_SPE );    // CPOL/CPHA/BR only change with SPE off
  GPS_SPI_INSTANCE->CR1 = ( uint16_t ) ( cr1 & ( uint16_t ) ~SPI_CR1_SPE );
  GPS_SPI_INSTANCE->CR1 = ( uint16_t ) ( cr1 | SPI_CR1_SPE );
}

static void spiBegin ( void ) {
  savedCr1 = GPS_SPI_INSTANCE->CR1;
  spiSetCr1 ( ( uint16_t ) ( ( savedCr1 & ( uint16_t ) ~GPS_SPI_CR1_MASK ) | GPS_SPI_CR1_BITS ) );

  // drop anything left in the RX FIFO by the previous device
  uint8_t guard = 8;
  while ( ( SPI_I2S_GetFlagStatus ( GPS_SPI_INSTANCE, SPI_I2S_FLAG_RXNE ) == SET ) && guard ) {
    ( void ) SPI_ReceiveData8 ( GPS_SPI_INSTANCE );
    guard--;
  }

  GPIO_ResetBits ( GPS_SPI_CS_GPIO, GPS_SPI_CS_PIN );
}

static void spiEnd ( void ) {
  spiWaitIdle ( );
  GPIO_SetBits ( GPS_SPI_CS_GPIO, GPS_SPI_CS_PIN );
  spiSetCr1 ( savedCr1 );
}

// Full-duplex exchange of len bytes. tx may be NULL ( sends 0xFF ), rx may be NULL.
static void spiExchange ( const uint8_t *tx, uint8_t *rx, uint16_t len ) {
  uint16_t errorsBefore = spiGetErrorCounter ( GPS_SPI_INSTANCE );

  for ( uint16_t i = 0; i < len; i++ ) {
    uint8_t in = spiTransferByte ( GPS_SPI_INSTANCE, tx ? tx [ i ] : 0xFF );
    if ( rx ) {
      rx [ i ] = in;
    }
  }

  if ( spiGetErrorCounter ( GPS_SPI_INSTANCE ) != errorsBefore ) {
    gpsBusErrorCount++;
  }
}

static void busPinInit ( void ) {
  GPIO_InitTypeDef gpio;

  RCC_AHBPeriphClockCmd ( GPS_SPI_CS_RCC, ENABLE );

  GPIO_SetBits ( GPS_SPI_CS_GPIO, GPS_SPI_CS_PIN );    // deselected before it becomes an output
  gpio.GPIO_Pin   = GPS_SPI_CS_PIN;
  gpio.GPIO_Mode  = GPIO_Mode_OUT;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init ( GPS_SPI_CS_GPIO, &gpio );
}

// SPI transfers are allowed in every state: a module that missed the boot probe is found by
// the GPS state machine's reconfigure-on-timeout loop, at the cost of a few idle transfers.
uint16_t gpsBusRead ( uint8_t *buf, uint16_t max ) {
  if ( max == 0 ) {
    return 0;
  }

  uint16_t want = max < GPS_BUS_READ_MAX ? max : GPS_BUS_READ_MAX;
  uint32_t start = micros ( );

  spiBegin ( );
  spiExchange ( NULL, buf, want );
  spiEnd ( );

  noteDuration ( start );
  return want;
}

uint16_t gpsBusWrite ( const uint8_t *buf, uint16_t len, uint8_t *rxBuf ) {
  if ( len > GPS_BUS_WRITE_MAX ) {
    return 0;
  }

  uint32_t start = micros ( );

  spiBegin ( );
  spiExchange ( buf, rxBuf, len );
  spiEnd ( );

  noteDuration ( start );
  return rxBuf ? len : 0;
}

// The module answers MON-VER ( and by default also streams NMEA ) on SPI. A floating or
// grounded MISO reads as all 0xFF or all 0x00, never as a UBX or NMEA start.
static bool looksLikeGps ( const uint8_t *data, uint16_t len, uint8_t *prev ) {
  for ( uint16_t i = 0; i < len; i++ ) {
    if ( ( *prev == 0xB5 && data [ i ] == 0x62 ) || ( *prev == '$' && data [ i ] == 'G' ) ) {
      return true;
    }
    *prev = data [ i ];
  }
  return false;
}

bool gpsBusInit ( void ) {
  static uint8_t probe [ GPS_BUS_READ_MAX ];
  uint8_t prev = 0;

  busPinInit ( );
  gpsBusStatus = GPS_BUS_NOT_INIT;

  bool found = false;
  uint16_t got = gpsBusWrite ( ubxMonVerPoll, sizeof ( ubxMonVerPoll ), probe );
  found        = looksLikeGps ( probe, got, &prev );

  uint32_t startMs = millis ( );
  while ( ! found && ( millis ( ) - startMs ) < GPS_BUS_PROBE_MS ) {
    got   = gpsBusRead ( probe, sizeof ( probe ) );
    found = looksLikeGps ( probe, got, &prev );
    if ( ! found ) {
      delay ( 2 );
    }
  }

  gpsBusStatus = found ? GPS_BUS_OK : GPS_BUS_NOT_FOUND;
  return found;
}

  #endif    // GPS_SPI

  /* ---------------------------------------------------------------- I2C1 */
  #ifdef GPS_I2C

    #ifndef GPS_I2C_ADDR
      #define GPS_I2C_ADDR 0x42
    #endif

    #define UBX_DDC_REG_BYTES_HI 0xFD    // bytes available, high byte; 0xFE low byte
    #define UBX_DDC_REG_STREAM   0xFF    // data stream; reads 0xFF when empty

    // The I2C driver cannot see a NACK: a missing or stuck module costs a full timeout and a
    // bus recovery that the IMU shares. After this many failures in a row GPS stops for good.
    #define GPS_I2C_MAX_CONSECUTIVE_ERRORS 1    // each failure is a ~10 ms timeout plus an IMU-bus reset

static uint8_t consecutiveErrors = 0;

static void noteI2cResult ( bool ok ) {
  if ( ok ) {
    consecutiveErrors = 0;
    return;
  }

  gpsBusErrorCount++;
  if ( ++consecutiveErrors >= GPS_I2C_MAX_CONSECUTIVE_ERRORS ) {
    gpsBusStatus = GPS_BUS_DISABLED;
  }
}

uint16_t gpsBusRead ( uint8_t *buf, uint16_t max ) {
  if ( gpsBusStatus != GPS_BUS_OK || max == 0 ) {
    return 0;
  }

  uint8_t want   = ( uint8_t ) ( max < GPS_BUS_READ_MAX ? max : GPS_BUS_READ_MAX );
  uint32_t start = micros ( );
  bool ok        = i2cRead ( GPS_I2C_ADDR, UBX_DDC_REG_STREAM, want, buf ) != 0;
  noteDuration ( start );

  noteI2cResult ( ok );
  return ok ? want : 0;
}

uint16_t gpsBusWrite ( const uint8_t *buf, uint16_t len, uint8_t *rxBuf ) {
  ( void ) rxBuf;

  if ( gpsBusStatus != GPS_BUS_OK || len < 2 || len > GPS_BUS_WRITE_MAX ) {
    return 0;    // a 1-byte write would only set the module's register pointer
  }

  uint32_t start = micros ( );
  bool ok        = i2cWriteBufferwithoutregister ( GPS_I2C_ADDR, ( uint8_t ) len, ( uint8_t * ) buf );
  noteDuration ( start );

  noteI2cResult ( ok );
  return 0;
}

bool gpsBusInit ( void ) {
  uint8_t avail [ 2 ];

  // one register read at boot: a missing module times out once here instead of every poll
  bool found   = i2cRead ( GPS_I2C_ADDR, UBX_DDC_REG_BYTES_HI, 2, avail ) != 0;
  gpsBusStatus = found ? GPS_BUS_OK : GPS_BUS_NOT_FOUND;
  if ( ! found ) {
    gpsBusErrorCount++;
  }
  return found;
}

  #endif    // GPS_I2C

#endif    // GPS_ON_BUS
