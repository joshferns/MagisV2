/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Project: MagisV2                                                           #
 #  File: \src\main\io\gps_data.h                                             #
 #  Created Date: Thu, 15th Oct 2026                                           #
 #  Brief: The GPS solution globals filled by io/gps.cpp ( split out of gps.h  #
 #         so readers need not include common/atomic.h )                       #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 15th Oct 2026                                          #
 #  Modified By: Claude                                                        #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-15	Claude	Moved from io/gps.h ( gps-spi-i2c task 6 )                 #
*******************************************************************************/
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LAT 0                             // GPS_coord index
#define LON 1

extern int32_t GPS_coord[2];               // LAT/LON, degrees * 1e7

extern uint8_t GPS_numSat;
extern uint16_t GPS_hdop;                  // GPS signal quality
extern uint8_t GPS_update;                 // it's a binary toogle to distinct a GPS position update
extern uint32_t GPS_packetCount;
extern uint32_t GPS_svInfoReceivedCount;
extern uint16_t GPS_altitude;              // altitude in m above mean sea level
extern uint16_t GPS_speed;                 // ground speed in cm/s
extern uint16_t GPS_ground_course;         // degrees * 10
extern int16_t GPS_velNED[3];              // cm/s north / east / down
extern uint16_t GPS_hAcc;                  // cm, horizontal accuracy estimate ( 65535 = unknown )
extern uint16_t GPS_vAcc;                  // cm, vertical accuracy estimate ( 65535 = unknown )
extern uint8_t GPS_fixType;                // UBX fix type: 0 none, 2 2D, 3 3D, ...
extern uint16_t GPS_ubxAckCount;           // UBX-ACK-ACK received
extern uint16_t GPS_ubxNakCount;           // UBX-ACK-NAK received

#ifdef __cplusplus
}
#endif
