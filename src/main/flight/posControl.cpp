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
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"

#include "command/command.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/accgyro.h"
#include "drivers/system.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "io/serial_msp.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/opticflow.h"

#include "config/runtime_config.h"

#include "posEstimate.h"

#include "posControl.h"

#include "API/API-Utils.h"
#include "io/rc_controls.h"

#define corr_scale 512/100
#define corr_scale2 1096/100

#define TOLERANCE_XY 10
#define CONTROL_ENDPOINT 100

static rcControlsConfig_t *rcControlsConfig;
int32_t setVelocityX = 0;
int32_t setVelocityY = 0;
uint8_t velocityControlX = 1;
uint8_t velocityControlY = 1;
int32_t Poshold;
int32_t error;
int32_t initialPitchHold;
int32_t PitchAdjustment;
int32_t estPosition = 0;
int32_t desiredPositionX = 0;
int32_t desiredPositionY = 0;
int32_t desiredVelo = 0;
int32_t errorAlt;
static uint8_t isPositionXChanged = 1;
static uint8_t isPositionYChanged = 1;
static uint8_t isAltHoldChanged = 0;

uint8_t kp_posx;
uint8_t kp_posy;
uint8_t kp_velx;
uint8_t kp_vely;
uint8_t ki_velx;
uint8_t ki_vely;
uint8_t kd_velx;
uint8_t kd_vely;

uint8_t _new_kpx;
uint8_t _new_kpy;
uint8_t _new_kdx;
uint8_t _new_kdy;
uint8_t _new_kix;
uint8_t _new_kiy;

int32_t AltHoldfromPos = 100;
int16_t PID_x = 0;
int16_t PID_y = 0;
int16_t _new_IoutX = 0;
int16_t _new_IoutY = 0;
int16_t IoutX = 0;
int16_t IoutY = 0;
int16_t VdesiredX = 0;
int16_t VdesiredY = 0;
uint8_t HeightAchieved = 0;
static pidProfile_t *pidProfile; //PS2

void configurePosHold(pidProfile_t *initialPidProfile, rcControlsConfig_t *rcControlsConfigPointer) //PS2
{
    pidProfile = initialPidProfile;
    rcControlsConfig = rcControlsConfigPointer;
}

// Position ( and its error ) lives in the arm-heading frame; velocity and the pitch/roll
// outputs live in the current heading frame. Without optic flow both are the same frame.
static void positionErrorToBody(float errorX, float errorY, float *bodyX, float *bodyY)
{
#ifdef OPTIC_FLOW
    opticFlowEarthToBody(errorX, errorY, bodyX, bodyY);
#else
    *bodyX = errorX;
    *bodyY = errorY;
#endif
}

#ifdef OPTIC_FLOW
void selectVelOrPosmode(void)
{
    int16_t positionXError = 0;
    int16_t positionYError = 0;

    if (ABS(rcData[PITCH] - 1500) > rcControlsConfig->alt_hold_deadband) {

        setVelocityX = (rcData[PITCH] - 1500) / 2;
        setVelocityX = constrain(setVelocityX, -100, 100);
        velocityControlX = 1;
        isPositionXChanged = 1;

    } else {

        velocityControlX = 0;

        if (isPositionXChanged) {

            if (ABS(VelocityX) <= 10 && isTookOff) { //After velocity is zero, then set pos
                desiredPositionX = PositionX;
                isPositionXChanged = 0;
            } else { //try to make velocity 0

                setVelocityX = 0;
            }

        }


    }

    //ROLL
    if (ABS(rcData[ROLL] - 1500) > rcControlsConfig->alt_hold_deadband)//velocity mode
    {
        setVelocityY = (rcData[ROLL] - 1500) / 2;
        setVelocityY = -constrain(setVelocityY, -100, 100);
        velocityControlY = 1;
        isPositionYChanged = 1;

    } else {

        velocityControlY = 0;

        if (isPositionYChanged) {

            if (ABS(VelocityY) <= 10 && isTookOff) {  //After velocity is zero, then set pos
                desiredPositionY = PositionY;
                isPositionYChanged = 0;
            } else {  //try to make velocity 0

                setVelocityY = 0;
            }

        }

    }

    // Position-hold axes: error in the arm-heading frame -> desired velocity -> heading frame
    if (!isPositionXChanged || !isPositionYChanged) {
        float posVelX = 0, posVelY = 0;    // cm/s, arm-heading frame
        float bodyVelX, bodyVelY;          // cm/s, heading frame

        if (!isPositionXChanged) {
            positionXError = (int16_t) constrain((int32_t) lrintf((float) desiredPositionX - PositionX), -200, 200);
            posVelX = (float) positionXError * ((float) pidProfile->P8[PIDPOS] / 100.0f);
        }
        if (!isPositionYChanged) {
            positionYError = (int16_t) constrain((int32_t) lrintf((float) desiredPositionY - PositionY), -200, 200);
            posVelY = (float) positionYError * ((float) pidProfile->P8[PIDPOS] / 100.0f);
        }

        positionErrorToBody(posVelX, posVelY, &bodyVelX, &bodyVelY);

        if (!isPositionXChanged) {
            setVelocityX = constrain((int32_t) lrintf(bodyVelX), -100, 100);
        }
        if (!isPositionYChanged) {
            setVelocityY = constrain((int32_t) lrintf(bodyVelY), -100, 100);
        }

    }

    VelocityController(setVelocityX, setVelocityY);

}
#endif

void VelocityController(int16_t VdesiredX, int16_t VdesiredY)
{
    kp_velx = kp_vely = pidProfile->P8[PIDUSER];
    ki_velx = ki_vely = pidProfile->I8[PIDUSER];
    kd_velx = kd_vely = pidProfile->D8[PIDUSER];

    int16_t errorVelX, errorVelY;
    int16_t PoutX, PoutY, DoutX, DoutY;

    errorVelX = VdesiredX - VelocityX;
    errorVelY = VdesiredY - VelocityY;

    PoutX = (kp_velx * errorVelX) / 100;
    PoutY = (kp_vely * errorVelY) / 100;

    PoutX = constrain(PoutX, -200, 200);
    PoutY = constrain(PoutY, -200, 200);

    // The integral is scaled by elapsed time, normalised to the 3500 us loop it was tuned at,
    // so the I gain no longer depends on the loop rate or on how often this is called.
    static uint32_t lastIntegralUs = 0;
    uint32_t nowUs = micros();
    float integralScale = constrainf((float) (nowUs - lastIntegralUs) / 3500.0f, 0.0f, 4.0f);
    lastIntegralUs = nowUs;

    if (ki_velx && ARMING_FLAG(ARMED) && isLocalisationOn) {
        IoutX = (int16_t) constrain((int32_t) IoutX + (int32_t) lrintf((float) (ki_velx * errorVelX) * integralScale / 100.0f), -20000, 20000);
        IoutY = (int16_t) constrain((int32_t) IoutY + (int32_t) lrintf((float) (ki_vely * errorVelY) * integralScale / 100.0f), -20000, 20000);

    } else { //Take out effects of integral if ki is zero
        IoutX = 0;
        IoutY = 0;
    }

    DoutX = (kd_velx * (accel_hf[0] + accel_hf_prev[0])) / 100;
    DoutY = (kd_vely * (accel_hf[1] + accel_hf_prev[1])) / 100;

    switch (localisationType) {

    case LOC_UWB:

        PID_x = constrain(PoutX + IoutX / 500 - DoutX, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);
        PID_y = -constrain(PoutY + IoutY / 500 - DoutY, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);

        break;

    case LOC_WHYCON:

        PID_x = constrain(PoutX + IoutX / 500 - DoutX, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);
        PID_y = -constrain(PoutY + IoutY / 500 - DoutY, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);

        break;

    case LOC_VICON:

        break;

    default:

        PID_x = constrain(PoutX + IoutX / 500 - DoutX, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);
        PID_y = -constrain(PoutY + IoutY / 500 - DoutY, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);

        break;

    }

}

void resetPosController(void)
{
    isPositionYChanged = 1;
    isPositionXChanged = 1;

}

void PosVelController(int16_t desiredX, int16_t desiredY, int16_t desiredVel)
{

    kp_posx = kp_posy = pidProfile->P8[PIDPOS];

    int16_t errorX, errorY;

    float bodyErrorX, bodyErrorY;    // cm, heading frame

    positionErrorToBody((float) desiredX - PositionX, (float) desiredY - PositionY, &bodyErrorX, &bodyErrorY);
    errorX = (int16_t) constrainf(bodyErrorX, -30000.0f, 30000.0f);
    errorY = (int16_t) constrainf(bodyErrorY, -30000.0f, 30000.0f);

    VdesiredX = constrain((kp_posx * errorX) / 100, -desiredVel, desiredVel);
    VdesiredY = constrain((kp_posy * errorY) / 100, -desiredVel, desiredVel);

    VelocityController(VdesiredX, VdesiredY);

}

bool PositionController(int16_t desiredX, int16_t desiredY, int16_t desiredZ)
{
    bool Status = false;
    //setdesiredHeight(desiredZ);

    PosVelController(desiredX, desiredY, 50);
    //SimpleController(desiredX, desiredY, 50);
    if ((abs(desiredX - PositionX) < TOLERANCE_XY) && (abs(desiredY - PositionY) < TOLERANCE_XY) && (HeightAchieved == 1)) {
        if (isLocalisationOn) {	//&&ARMING_FLAG(ARMED))
            Status = true;
        } else {
            Status = false;
        }
    }

    return Status;
}

void setdesiredHeight(int32_t desiredZ)
{
    AltHoldfromPos = desiredZ;
}

int32_t getdesiredHeight(void)
{
    return AltHoldfromPos;
}

int16_t getrcDataRoll(void)
{
    return PID_y;
}

int16_t getrcDataPitch(void)
{
    return PID_x;
}

int16_t getDesiredVelocityX()
{
    return VdesiredX;

}
int16_t getDesiredVelocityY()
{

    return VdesiredY;
}

void resetPosIntegral(void)
{	//To Eliminate windup
    _new_IoutX = 0;
    _new_IoutY = 0;
    IoutX = 0;
    IoutY = 0;

}

bool SimpleController(int16_t desiredX, int16_t desiredY, int16_t desiredZ)
{
    int16_t errorX, errorY;
    bool Status = false;

    //setdesiredHeight(desiredZ); //Set Desired Height

    _new_kpx = _new_kpy = pidProfile->P8[PIDPOSR];
    _new_kix = _new_kiy = pidProfile->I8[PIDPOSR];
    _new_kdx = _new_kdy = pidProfile->D8[PIDPOSR];

    float bodyErrorX, bodyErrorY;    // cm, heading frame

    positionErrorToBody((float) desiredX - PositionX, (float) desiredY - PositionY, &bodyErrorX, &bodyErrorY);
    errorX = (int16_t) constrainf(bodyErrorX, -50.0f, 50.0f);
    errorY = (int16_t) constrainf(bodyErrorY, -50.0f, 50.0f);
    //errorX = desiredX-PositionX;
    //errorY = desiredY-PositionY;

    if (_new_kix && ARMING_FLAG(ARMED) && isLocalisationOn) {
        _new_IoutX += (_new_kix * (errorX)) / 10;
        _new_IoutX = constrain(_new_IoutX, -15000, 15000);

        _new_IoutY += (_new_kiy * (errorY)) / 10;
        _new_IoutY = constrain(_new_IoutY, -15000, 15000);

    } else {
        _new_IoutX = 0;
        _new_IoutY = 0;
    }

    PID_x = (_new_kpx * errorX) / 10 - constrain(_new_kdx * VelocityX, -120, 120) + _new_IoutX / 500;
    PID_y = -((_new_kpy * errorY) / 10 - constrain(_new_kdy * VelocityY, -120, 120) + _new_IoutY / 500);
    PID_x = constrain(PID_x, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);
    PID_y = constrain(PID_y, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);

    //Has the command been executed??
    if ((abs(errorX) < TOLERANCE_XY) && (abs(errorY) < TOLERANCE_XY) && (HeightAchieved == 1)) {
        if (isLocalisationOn) {	//&&ARMING_FLAG(ARMED))
            Status = true;
        } else {
            Status = false;
        }
    }

    return Status;
}
