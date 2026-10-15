/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\GPS.h                                                  #
 #  Created Date: Thu, 15th Oct 2026                                           #
 #  Brief: Read-only GPS data for user code                                    #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 15th Oct 2026                                          #
 #  Modified By: Claude                                                        #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-15	Claude	Gps_Get ( gps-spi-i2c task 6 )                             #
*******************************************************************************/
#ifndef GPS_API_H
#define GPS_API_H
#include "stdint.h"

/**
 * @enum GPS_Option_e
 * @brief The GPS values user code can read with Gps_Get ( ). Units and validity: docs/API/GPS_API_WIKI.md.
 *
 * Every option reads 0 on a firmware built without GPS, and while no GPS is connected
 * ( Gps_Available = 0 ). Position values are only meaningful while Gps_Fix_3D is 1.
 */
typedef enum GPS_Option {
  Gps_Available,       // 1 while a GPS module is sending data, else 0.
  Gps_Fix_3D,          // 1 with a valid 3D fix, else 0.
  Gps_Fix_Type,        // u-blox fix type: 0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS + dead reckoning, 5 time only.
  Gps_Satellites,      // Satellites used in the solution.
  Gps_Latitude,        // Degrees x 10,000,000 ( 1 = ~1.1 cm ), north positive.
  Gps_Longitude,       // Degrees x 10,000,000, east positive.
  Gps_Altitude,        // Metres above mean sea level.
  Gps_Vel_North,       // cm/s, north positive.
  Gps_Vel_East,        // cm/s, east positive.
  Gps_Vel_Down,        // cm/s, down positive ( negative while climbing ).
  Gps_Ground_Speed,    // cm/s, horizontal speed.
  Gps_Course,          // Degrees x 10 ( 0-3599 ), direction of motion, 0 = north, clockwise.
  Gps_H_Accuracy,      // cm, the module's horizontal accuracy estimate; 65535 = unknown.
  Gps_V_Accuracy,      // cm, the module's vertical accuracy estimate; 65535 = unknown.
  Gps_PDOP             // Position dilution of precision x 100 ( 100 = 1.0, lower is better ).
} GPS_Option_e;

/**
 * @brief Reads one GPS value.
 *
 * `_gps_option` An enumerator of type GPS_Option_e indicating which value to read.
 *        @param Gps_Available, Gps_Fix_3D, Gps_Fix_Type, Gps_Satellites: status.
 *        @param Gps_Latitude, Gps_Longitude: position in degrees x 10,000,000.
 *        @param Gps_Altitude: metres above mean sea level.
 *        @param Gps_Vel_North, Gps_Vel_East, Gps_Vel_Down, Gps_Ground_Speed: cm/s.
 *        @param Gps_Course: degrees x 10.
 *        @param Gps_H_Accuracy, Gps_V_Accuracy: cm. Gps_PDOP: x 100.
 *
 * @return `int32_t` The requested value; 0 for an undefined option or a build without GPS.
 */
int32_t Gps_Get ( GPS_Option_e _gps_option );
#endif
