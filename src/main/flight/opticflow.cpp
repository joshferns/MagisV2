/*
 * This file is part of Magis.
 *
 * Magis is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Magis is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */



#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/flash_m25p16.h"
#include "drivers/flash.h"
#include "drivers/ranging_vl53l0x.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"
#include "flight/filter.h"
#include "flight/acrobats.h"
#include "flight/posEstimate.h"
#include "flight/posControl.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "opticflow.h"

#include "../drivers/opticflow_paw3903.h"
#include "mw.h"

#ifdef LASER_TOF_L1x
  #include "drivers/ranging_vl53l1x.h"
#endif

#define UPDATE_FREQUENCY       ( 1000 * 10 )    // estimator period in us ( 100 Hz )
#define FLOW_HEALTH_TIMEOUT_MS 200              // no trusted flow sample for this long -> unhealthy

// Velocity estimator. The flow velocity is blended with the level-frame accelerometer in a
// complementary filter: accel carries the fast part, flow corrects it below ~1 / ( 2 pi tau ).
// This replaces a 0.05 IIR on flow alone ( ~0.19 s time constant with nothing to fill the lag ).
// FLOW_ACC_BLEND uses the same accel_hf sign convention as the velocity loop's D term
// ( accel_hf = d VelocityX / dt ). Set it to 0 to run on flow alone with a short filter.
#ifndef FLOW_ACC_BLEND
  #define FLOW_ACC_BLEND 1
#endif
#if FLOW_ACC_BLEND
  #define FLOW_VEL_TAU_S 0.15f    // s
#else
  #define FLOW_VEL_TAU_S 0.04f    // s
#endif

// minimum assumed height ( m )
const float height_min = 0.1;

// maximum scaling height ( m )
const float height_max = 3.0;

float delta_velocity_ne [ 2 ];    // m/s, level-frame velocity change since the last flow sample
// last flow rate in radians/sec in level frame
float last_flow_rate_rps [ 2 ];

float flow_rate_rps [ 2 ] = { 0 };

float last_ins_height;
float height_offset;

float sensor_flow [ 3 ]    = { 0 };
float sensor_flow_hf [ 2 ] = { 0 };    // m/s, flow-only velocity in the level frame

float accel_hf [ 3 ]      = { 0 };    // cm/s^2, level frame
float accel_hf_prev [ 3 ] = { 0 };
float velocity_hf [ 3 ]   = { 0 };    // cm/s, fused velocity in the level frame
float opticflowHeight     = 0;        // m, height-from-flow estimate ( diagnostic )
float opticflowRangeM     = 0;        // m, height used to scale flow this cycle

float flow_max = 0.6;

float debugOpticFlowVar  = 0;
float debugOpticFlowVar1 = 0;
float debugOpticFlowVar2 = 0;
float debugOpticFlowVar3 = 0;
float debugOpticFlowVar4 = 0;
float debugOpticFlow [ 2 ];
float debugOpticFlow1 [ 2 ];
float debugOpticFlow2 [ 3 ];

static bool flowHealthy          = false;
static uint32_t lastValidFlowMs  = 0;
static uint32_t lastSampleCount  = 0;
static uint32_t lastSampleMs     = 0;
static int16_t flowYawRefDeg     = 0;    // heading at arm; position is kept in this fixed frame
static float flowYawCos          = 1.0f;
static float flowYawSin          = 0.0f;

bool opticFlowIsHealthy ( void ) {
  return flowHealthy;
}

// Rotate a vector from the arm-heading ( position ) frame into the current heading frame
// that VelocityX/Y and the pitch/roll commands use. X forward, Y left.
void opticFlowEarthToBody ( float earthX, float earthY, float *bodyX, float *bodyY ) {
  *bodyX = flowYawCos * earthX + flowYawSin * earthY;
  *bodyY = -flowYawSin * earthX + flowYawCos * earthY;
}

static void updateYawFrame ( void ) {
  if ( ! ARMING_FLAG ( ARMED ) ) {
    flowYawRefDeg = heading;
  }

  int32_t yawDeg = heading - flowYawRefDeg;
  if ( yawDeg > 180 ) yawDeg -= 360;
  if ( yawDeg < -180 ) yawDeg += 360;

  // heading is clockwise-positive; the X-forward / Y-left frame is counter-clockwise-positive
  float psi  = -degreesToRadians ( ( int16_t ) yawDeg );
  flowYawCos = cos_approx ( psi );
  flowYawSin = sin_approx ( psi );
}

// Vertical distance to the surface the sensor sees, in m.
static float flowHeightM ( void ) {
  float range = 0;    // m, along the body Z axis

#if defined( LASER_TOF_L1x )
  range = ( float ) NewSensorRange_L1 * 0.001f;
#elif defined( LASER_TOF )
  range = ( float ) NewSensorRange * 0.001f;
#endif

  float height;
  if ( range > 0 ) {
    height = range * cos_approx ( anglerad [ AI_ROLL ] ) * cos_approx ( anglerad [ AI_PITCH ] );
  } else {
    // no rangefinder fitted or no reading yet: estimated altitude above the arm point
    height = ( float ) altitudeHoldGetEstimatedAltitude ( ) * 0.01f;
  }

  return constrainf ( height, height_min, height_max );
}

static void updateAccel ( float stepS ) {
  accel_hf_prev [ 0 ] = accel_hf [ 0 ];
  accel_hf_prev [ 1 ] = accel_hf [ 1 ];

  if ( accSumCountXYZ ) {
    accel_hf [ 0 ] = ( float ) accSumXYZ [ 0 ] / ( float ) accSumCountXYZ * accVelScale;
    accel_hf [ 1 ] = ( float ) accSumXYZ [ 1 ] / ( float ) accSumCountXYZ * accVelScale;
  } else {
    accel_hf [ 0 ] = 0;
    accel_hf [ 1 ] = 0;
  }

  imuResetAccelerationSumXYZ ( );

  delta_velocity_ne [ 0 ] += accel_hf [ 0 ] * 0.01f * stepS;
  delta_velocity_ne [ 1 ] += accel_hf [ 1 ] * 0.01f * stepS;
}

static void updateHeightEstimate ( bool newSample ) {

  float ins_height = ( float ) altitudeHoldGetEstimatedAltitude ( ) * 0.01f;

  if ( ! ARMING_FLAG ( ARMED ) || ( int32_t ) ( millis ( ) - arm_time ) < 2500 ) {
    height_offset           = -ins_height;
    last_ins_height         = ins_height;
    delta_velocity_ne [ 0 ] = 0;
    delta_velocity_ne [ 1 ] = 0;
    return;
  }

  if ( ! newSample ) {
    return;
  }

  if ( ! opticFlowSampleValid ) {
    delta_velocity_ne [ 0 ] = 0;
    delta_velocity_ne [ 1 ] = 0;
    return;
  }

  float delta_vel_rate [ 2 ];
  delta_vel_rate [ 0 ] = delta_velocity_ne [ 0 ];
  delta_vel_rate [ 1 ] = delta_velocity_ne [ 1 ];

  flow_rate_rps [ 0 ] = flowRate [ 0 ] + bodyRate [ 1 ];
  flow_rate_rps [ 1 ] = -( flowRate [ 1 ] + bodyRate [ 0 ] );

  float delta_flowrate [ 3 ];
  delta_flowrate [ 0 ] = flow_rate_rps [ 0 ] - last_flow_rate_rps [ 0 ];
  delta_flowrate [ 1 ] = flow_rate_rps [ 1 ] - last_flow_rate_rps [ 1 ];
  delta_flowrate [ 2 ] = 0;

  last_flow_rate_rps [ 0 ] = flow_rate_rps [ 0 ];
  last_flow_rate_rps [ 1 ] = flow_rate_rps [ 1 ];

  float *tempVector    = dcmBodyToEarth3D ( delta_flowrate );
  delta_flowrate [ 0 ] = tempVector [ 0 ];
  delta_flowrate [ 1 ] = tempVector [ 1 ];

  //  update height estimate

  const float min_velocity_change = 0.04;
  const float min_flow_change     = 0.04;
  const float height_delta_max    = 0.25;

  float delta_height    = 0;
  float total_weight    = 0;
  float height_estimate = ins_height + height_offset;

  for ( uint8_t i = 0; i < 2; i++ ) {
    // only use height estimates when we have significant delta-velocity and significant delta-flow
    float abs_flow = fabsf ( delta_flowrate [ i ] );
    if ( abs_flow < min_flow_change || fabsf ( delta_vel_rate [ i ] ) < min_velocity_change ) {
      continue;
    }
    // get instantaneous height estimate
    float height = delta_vel_rate [ i ] / delta_flowrate [ i ];

    if ( height <= 0 ) {
      // discard negative heights
      continue;
    }
    delta_height += ( height - height_estimate ) * abs_flow;
    total_weight += abs_flow;
  }
  if ( total_weight > 0 ) {
    delta_height /= total_weight;
  }

  if ( delta_height < 0 ) {
    // bias towards lower heights, as we'd rather have too low
    // gain than have oscillation. This also compensates a bit for
    // the discard of negative heights above
    delta_height *= 2;
  }

  // don't update height by more than height_delta_max, this is a simple way of rejecting noise
  float new_offset = height_offset + constrainf ( delta_height, -height_delta_max, height_delta_max );

  // apply a simple filter
  height_offset = 0.8f * height_offset + 0.2f * new_offset;

  if ( ins_height + height_offset < height_min ) {
    // height estimate is never allowed below the minimum
    height_offset = height_min - ins_height;
  }

  opticflowHeight = ins_height + height_offset;
  last_ins_height = ins_height;

  delta_velocity_ne [ 0 ] = 0;
  delta_velocity_ne [ 1 ] = 0;
}

static void updateVelocity ( float stepS, bool newSample ) {

  uint32_t now = millis ( );

#if FLOW_ACC_BLEND
  velocity_hf [ 0 ] += accel_hf [ 0 ] * stepS;
  velocity_hf [ 1 ] += accel_hf [ 1 ] * stepS;
#else
  UNUSED ( stepS );
#endif

  if ( newSample && opticFlowSampleValid ) {

    float raw_flow [ 2 ];    // rad/s, rotation removed
    raw_flow [ 0 ] = flowRate [ 0 ] + bodyRate [ 1 ];
    raw_flow [ 1 ] = -( flowRate [ 1 ] + bodyRate [ 0 ] );

    debugOpticFlow [ 0 ] = raw_flow [ 0 ];
    debugOpticFlow [ 1 ] = raw_flow [ 1 ];

    opticflowRangeM  = flowHeightM ( );
    sensor_flow [ 0 ] = raw_flow [ 0 ] * opticflowRangeM;
    sensor_flow [ 1 ] = raw_flow [ 1 ] * opticflowRangeM;
    sensor_flow [ 2 ] = 0;

    float *tempVector   = dcmBodyToEarth3D ( sensor_flow );
    sensor_flow_hf [ 0 ] = tempVector [ 0 ];
    sensor_flow_hf [ 1 ] = tempVector [ 1 ];

    float flowVel [ 2 ] = { sensor_flow_hf [ 0 ] * 100.0f, sensor_flow_hf [ 1 ] * 100.0f };    // cm/s

    if ( ! flowHealthy ) {
      // ( re ) start from the measurement instead of blending from a stale state
      velocity_hf [ 0 ] = flowVel [ 0 ];
      velocity_hf [ 1 ] = flowVel [ 1 ];
    } else {
      float samplePeriod = constrainf ( ( float ) ( now - lastSampleMs ) * 0.001f, 0.001f, 0.2f );    // s
      float k            = samplePeriod / ( FLOW_VEL_TAU_S + samplePeriod );
      velocity_hf [ 0 ] += k * ( flowVel [ 0 ] - velocity_hf [ 0 ] );
      velocity_hf [ 1 ] += k * ( flowVel [ 1 ] - velocity_hf [ 1 ] );
    }

    debugOpticFlow1 [ 0 ] = flowVel [ 0 ];
    debugOpticFlow1 [ 1 ] = flowVel [ 1 ];

    lastValidFlowMs = now;
  }

  if ( newSample ) {
    lastSampleMs = now;
  }

  flowHealthy = ( lastValidFlowMs != 0 ) && ( ( now - lastValidFlowMs ) < FLOW_HEALTH_TIMEOUT_MS );
  if ( ! flowHealthy ) {
    velocity_hf [ 0 ] = 0;
    velocity_hf [ 1 ] = 0;
  }
}

static void updatePosition ( float stepS ) {

  updateYawFrame ( );

  if ( ARMING_FLAG ( ARMED ) ) {

    VelocityX = ( int16_t ) lrintf ( constrainf ( velocity_hf [ 0 ], -1000.0f, 1000.0f ) );
    VelocityY = ( int16_t ) lrintf ( constrainf ( velocity_hf [ 1 ], -1000.0f, 1000.0f ) );

    if ( flowHealthy ) {
      // heading frame -> arm-heading frame, so the hold point stays put when the drone yaws
      PositionX += ( flowYawCos * velocity_hf [ 0 ] - flowYawSin * velocity_hf [ 1 ] ) * stepS;
      PositionY += ( flowYawSin * velocity_hf [ 0 ] + flowYawCos * velocity_hf [ 1 ] ) * stepS;
    }
  } else {

    VelocityX = 0;
    VelocityY = 0;
    PositionX = 0;
    PositionY = 0;
  }
}

void runFlowHold ( uint32_t currentTime ) {

  static uint32_t previousTime = 0;

  uint32_t dTime = currentTime - previousTime;

  if ( dTime < UPDATE_FREQUENCY ) {
    return;
  }

  previousTime = currentTime;

  if ( dTime > 2 * UPDATE_FREQUENCY ) {
    // first call or a long stall: don't integrate across the gap
    dTime = UPDATE_FREQUENCY;
  }

  float stepS = ( float ) dTime * 1.0e-6f;    // s

  bool newSample  = ( opticFlowSampleCount != lastSampleCount );
  lastSampleCount = opticFlowSampleCount;

  updateAccel ( stepS );
  updateHeightEstimate ( newSample );
  updateVelocity ( stepS, newSample );
  updatePosition ( stepS );
}
