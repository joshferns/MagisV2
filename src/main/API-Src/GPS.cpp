/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\GPS.cpp                                            #
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
#include "API/GPS.h"

#include "platform.h"

#ifdef GPS
  #include "common/axis.h"
  #include "config/runtime_config.h"
  #include "sensors/sensors.h"
  #include "io/gps_data.h"
#endif

int32_t Gps_Get ( GPS_Option_e _gps_option ) {
#ifdef GPS
  if ( ! sensors ( SENSOR_GPS ) ) {
    return 0;    // no module, or no data for 2.5 s: every value reads 0
  }

  switch ( _gps_option ) {
    case Gps_Available:
      return 1;
    case Gps_Fix_3D:
      return STATE ( GPS_FIX ) ? 1 : 0;
    case Gps_Fix_Type:
      return GPS_fixType;
    case Gps_Satellites:
      return GPS_numSat;
    case Gps_Latitude:
      return GPS_coord [ LAT ];
    case Gps_Longitude:
      return GPS_coord [ LON ];
    case Gps_Altitude:
      return ( int16_t ) GPS_altitude;    // stored as uint16 metres; this recovers heights below sea level
    case Gps_Vel_North:
      return GPS_velNED [ 0 ];
    case Gps_Vel_East:
      return GPS_velNED [ 1 ];
    case Gps_Vel_Down:
      return GPS_velNED [ 2 ];
    case Gps_Ground_Speed:
      return GPS_speed;
    case Gps_Course:
      return GPS_ground_course;
    case Gps_H_Accuracy:
      return GPS_hAcc;
    case Gps_V_Accuracy:
      return GPS_vAcc;
    case Gps_PDOP:
      return GPS_hdop;
    default:
      return 0;
  }
#else
  ( void ) _gps_option;
  return 0;
#endif
}
