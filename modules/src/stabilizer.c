/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "ms5611.h"
#include "math.h"
#include "param.h"
#include "ledseq.h"
#include "global.h"
#include "pid.h"
#include "pm.h"
/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2 //500hz/2  = 250hz
#define ALTITUDE_UPDATE_RATE_DIVIDER  5 // 500hz/5 = 100hz for barometer measurements
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER))
#define BARO_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTITUDE_UPDATE_RATE_DIVIDER))

#define LOGGING_ENABLED
#ifdef LOGGING_ENABLED
#define PRIVATE
#else
#define PRIVATE static
#endif

PRIVATE Axis3f gyro; // Gyro axis data in deg/s
PRIVATE Axis3f acc;  // Accelerometer axis data in mG
PRIVATE Axis3f accWorld;  // Accelerometer world axis data in mG
PRIVATE Axis3f mag;  //
PRIVATE float zSpeed=0.0; // Vertical speed (world frame) integrated from vertical acceleration
PRIVATE float zReduce= 0.995; // Factor to continuously reduce speed -> stops accerleration integration getting out of control

PRIVATE float zAccelAvgLong= 0.; // longterm avg z accel
PRIVATE float zAccelAvgAlpha= 0.9; // smooth factor
PRIVATE float zAccelAvgShort=0;
PRIVATE float zAccelAlpha= 0.98;
//PRIVATE float zAccel;

PRIVATE float voltageLong = 0;
PRIVATE float voltageAlpha = 0.98;

PRIVATE float temperature; // temp of barometer
PRIVATE float asl_baro; // m above ground
PRIVATE float asl_baro_raw; // m above ground
PRIVATE float asl_baro_long; // long term, used to estimate vertical speed using baro
PRIVATE float baro_alpha = 0.92;
PRIVATE float baro_alpha_long = 0.98;
PRIVATE float asl_vspeed = 0.0;
PRIVATE float pressure; // pressure
PRIVATE float hover_error;
PRIVATE float baro_asl_err_max = 0.2; //meters
PRIVATE float zSpeedKp = 0;
//PRIVATE float q0, q1, q2, q3;

PRIVATE float eulerRollActual;
PRIVATE float eulerPitchActual;
PRIVATE float eulerYawActual;
PRIVATE float eulerRollDesired;
PRIVATE float eulerPitchDesired;
PRIVATE float eulerYawDesired;
PRIVATE float rollRateDesired;
PRIVATE float pitchRateDesired;
PRIVATE float yawRateDesired;
PRIVATE float fusionDt;
PRIVATE PidObject altitude_pid;

RPYType rollType;
RPYType pitchType;
RPYType yawType;

uint16_t actuatorThrust;
int16_t actuatorRoll;
int16_t actuatorPitch;
int16_t actuatorYaw;

uint32_t motorPowerLeft;
uint32_t motorPowerRight;
uint32_t motorPowerFront;
uint32_t motorPowerRear;

bool hover = false;
bool set_hover = false;
float hover_change = 0;
float hover_target = -1;
float hover_kp=1.0;
float hover_ki=0.0;
float hover_kd=0.0;
float hover_pid;
float hoverPidAlpha = 0.75;
float pid_mag = 1.0; //relates meters asl to thrust
uint16_t hover_minThrust = 38000;
uint16_t hover_maxThrust = 44000;
LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)



LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerLeft)
LOG_ADD(LOG_INT32, m1, &motorPowerFront)
LOG_ADD(LOG_INT32, m2, &motorPowerRight)
LOG_ADD(LOG_INT32, m3, &motorPowerRear)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, vLong, &voltageLong)
//LOG_ADD(LOG_UINT8, hover, &hover)
//LOG_ADD(LOG_UINT8, set_hover, &set_hover)
LOG_GROUP_STOP(motor)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_ADD(LOG_FLOAT, xw, &accWorld.x)
LOG_ADD(LOG_FLOAT, yw, &accWorld.y)
LOG_ADD(LOG_FLOAT, zw, &accWorld.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &asl_baro)
LOG_ADD(LOG_FLOAT, asl_raw, &asl_baro_raw)
LOG_ADD(LOG_FLOAT, asl_long, &asl_baro_long)
LOG_ADD(LOG_FLOAT, temp, &temperature)
LOG_ADD(LOG_FLOAT, pressure, &pressure)
LOG_ADD(LOG_FLOAT, asl_vspeed, &asl_vspeed)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(hover)
LOG_ADD(LOG_FLOAT, pid, &hover_pid)
//LOG_ADD(LOG_FLOAT, err, &altitude_pid.error)
//LOG_ADD(LOG_FLOAT, p, &altitude_pid.outP)
//LOG_ADD(LOG_FLOAT, i, &altitude_pid.outI)
//LOG_ADD(LOG_FLOAT, d, &altitude_pid.outD)
LOG_ADD(LOG_FLOAT, err, &hover_error)
LOG_ADD(LOG_FLOAT, target, &hover_target)
LOG_ADD(LOG_FLOAT, zSpeed, &zSpeed)
LOG_ADD(LOG_FLOAT, zAccLong, &zAccelAvgLong)
LOG_ADD(LOG_FLOAT, zAccShort, &zAccelAvgShort)
LOG_GROUP_STOP(hover)

PARAM_GROUP_START(hover)
PARAM_ADD(PARAM_FLOAT, aslAlpha, &baro_alpha)
PARAM_ADD(PARAM_FLOAT, aslAlphaLong, &baro_alpha_long)
PARAM_ADD(PARAM_FLOAT, kp, &hover_kp)
PARAM_ADD(PARAM_FLOAT, ki, &hover_ki)
PARAM_ADD(PARAM_FLOAT, kd, &hover_kd)
PARAM_ADD(PARAM_UINT16, maxThrust, &hover_maxThrust)
PARAM_ADD(PARAM_UINT16, minThrust, &hover_minThrust)
PARAM_ADD(PARAM_FLOAT, zAccAlphaLong, &zAccelAvgAlpha)
PARAM_ADD(PARAM_FLOAT, zAccAlphaShort, &zAccelAlpha)
PARAM_ADD(PARAM_FLOAT, maxAslErr, &baro_asl_err_max)
PARAM_ADD(PARAM_FLOAT, hoverPidAlpha, &hoverPidAlpha)
PARAM_ADD(PARAM_FLOAT, zSpeedKp, &zSpeedKp)
PARAM_ADD(PARAM_FLOAT, pid_mag, &pid_mag)
PARAM_ADD(PARAM_FLOAT, voltageAlpha, &voltageAlpha)
PARAM_GROUP_STOP(hover)



static bool isInit;

static void distributePower(const uint16_t thrust, const int16_t roll, const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);

void stabilizerInit(void) {
    if (isInit)
        return;

    motorsInit();
    imu6Init();
    sensfusion6Init();
    controllerInit();


    rollRateDesired = 0;
    pitchRateDesired = 0;
    yawRateDesired = 0;

    xTaskCreate(stabilizerTask, (const signed char * const )"STABILIZER", 2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);

    isInit = TRUE;
}

bool stabilizerTest(void) {
    bool pass = true;

    pass &= motorsTest();
    pass &= imu6Test();
    pass &= sensfusion6Test();
    pass &= controllerTest();

    return pass;
}

static void stabilizerTask(void* param) {
    uint32_t attitudeCounter = 0;
    uint32_t altitudeCounter = 0;
    uint32_t lastWakeTime;

    vTaskSetApplicationTaskTag(0, (void*) TASK_STABILIZER_ID_NBR);

    //Wait for the system to be fully started to start stabilization loop
    systemWaitStart();

    lastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ) );

        imu9Read(&gyro, &acc, &mag);

        if (imu6IsCalibrated()) {
            commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
            commanderGetRPYType(&rollType, &pitchType, &yawType);


            // 100hz
            if (++altitudeCounter >= ALTITUDE_UPDATE_RATE_DIVIDER) {
                //TODO: do the smoothing within getData
                ms5611GetData(&pressure, &temperature, &asl_baro_raw);
                asl_baro = asl_baro*baro_alpha + asl_baro_raw * (1-baro_alpha);
                asl_baro_long = asl_baro_long*baro_alpha_long + asl_baro_raw * (1-baro_alpha_long);
                asl_vspeed = asl_baro-asl_baro_long;
                altitudeCounter = 0;

                commanderGetHover(&hover, &set_hover, &hover_change);
                // Set hover alititude
                if (set_hover){
                    hover_target = asl_baro;
                    pidInit(&altitude_pid, asl_baro, hover_kp, hover_ki, hover_kd, BARO_UPDATE_DT );
                    hover_pid = 0;
                    voltageLong = pmGetBatteryVoltagePercent();
                }

                // If altitude vs hover_target is invalid we cannot hover
                if (hover){

                    // Update target altitude from joycontroller input
                    hover_target += hover_change/200;
                    pidSetDesired(&altitude_pid, hover_target);

                    // LED on
                    if (altitudeCounter==0){
                        ledseqRun(LED_RED, seq_hover);
                        voltageLong = voltageLong*voltageAlpha + pmGetBatteryVoltagePercent()*(1.f-voltageAlpha);
                    }

                    // Compute error, limit the magnitude
                    hover_error = min(baro_asl_err_max, max(-baro_asl_err_max, asl_baro-hover_target));
                    pidSetError(&altitude_pid, -hover_error);

                    // Get control from PID controller, dont update the error (done above)

                    if (set_hover){
                        // Reset hover_pid
                        hover_pid = pidUpdate(&altitude_pid, asl_baro, false);
                    } else {
                        // Smooth it //TODO: same as smoothing the error??
                        hover_pid = hover_pid * (hoverPidAlpha) + (asl_vspeed * zSpeedKp + pidUpdate(&altitude_pid, asl_baro, false)) * (1.f-hoverPidAlpha);
                    }


                    // use vSpeed
                    actuatorThrust = limitThrust( (int32_t)actuatorThrust + (int32_t)(hover_pid*pid_mag));

                } else {
                    hover_target = 0.0;
                    hover_error = 0.0;
                    hover_pid = 0.0;
                }
            }



            // 250hz
            if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER) {
                sensfusion6UpdateQ(gyro, acc, FUSION_UPDATE_DT /*, &q0, &q1, &q2, &q3*/);
                sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
                //TODO: the offsets dont work in WorldAcc?
                sensfusion6UpdateWorldAcc(&acc, (fabs(eulerRollActual)<2 && fabs(eulerPitchActual)<2));
                sensfusion6GetWorldAcc(&accWorld);


                //zSpeed += accWorld.z * FUSION_UPDATE_DT;
                //zSpeed *= zReduce;

                //zAccelAvgLong  = zAccelAvgLong  * zAccelAvgAlpha + accWorld.z * (1.f - zAccelAvgAlpha);
                //zAccelAvgShort = zAccelAvgShort * zAccelAlpha    + accWorld.z * (1.f - zAccelAlpha);
                //zSpeed = zAccelAvgShort - zAccelAvgLong;

                controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual, eulerRollDesired, eulerPitchDesired, -eulerYawDesired, &rollRateDesired, &pitchRateDesired, &yawRateDesired);
                attitudeCounter = 0;

            }

            if (rollType == RATE) {
                rollRateDesired = eulerRollDesired;
            }
            if (pitchType == RATE) {
                pitchRateDesired = eulerPitchDesired;
            }
            if (yawType == RATE) {
                yawRateDesired = -eulerYawDesired;
            }

            // TODO: Investigate possibility to subtract gyro drift.
            controllerCorrectRatePID(gyro.x, -gyro.y, gyro.z, rollRateDesired, pitchRateDesired, yawRateDesired);

            controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);

            if (!hover){
                commanderGetThrust(&actuatorThrust);
            } else {
                // hover code
                // use acceleration
                // use hover_target
                // use asl_baro
            }


//            // sum of last 0.25 secs > thresh -> avoid spike
//            if (accWorld.z < -0.4 && actuatorThrust>12000){ // FREE FALL
//                actuatorThrust = 60000;
//            }

            if (abs(eulerRollActual)>75 || abs(eulerPitchActual)>75){
                if (altitudeCounter==0){
                    ledseqRun(LED_RED, seq_hover);
                }
                actuatorThrust = 0;
            }




            if (actuatorThrust > 0) {
#if defined(TUNE_ROLL)
                distributePower(actuatorThrust, actuatorRoll, 0, 0);
#elif defined(TUNE_PITCH)
                distributePower(actuatorThrust, 0, actuatorPitch, 0);
#elif defined(TUNE_YAW)
                distributePower(actuatorThrust, 0, 0, -actuatorYaw);
#else
                distributePower(actuatorThrust, actuatorRoll, actuatorPitch, -actuatorYaw);
#endif
            } else {
                distributePower(0, 0, 0, 0);
                controllerResetAllPID();
            }
#if 0
            static int i = 0;
            if (++i > 19)
            {
                uartPrintf("%i, %i, %i\n",
                        (int32_t)(eulerRollActual*100),
                        (int32_t)(eulerPitchActual*100),
                        (int32_t)(eulerYawActual*100));
                i = 0;
            }
#endif
        }
    }
}

static void distributePower(const uint16_t thrust, const int16_t roll, const int16_t pitch, const int16_t yaw) {
#ifdef QUAD_FORMATION_X
    roll = roll >> 1;
    pitch = pitch >> 1;
    motorPowerLeft = limitThrust(thrust + roll + pitch - yaw);
    motorPowerRight = limitThrust(thrust - roll - pitch - yaw);
    motorPowerFront = limitThrust(thrust - roll + pitch + yaw);
    motorPowerRear = limitThrust(thrust + roll - pitch + yaw);
#else // QUAD_FORMATION_NORMAL
    motorPowerLeft = limitThrust(thrust + roll - yaw);
    motorPowerRight = limitThrust(thrust - roll - yaw);
    motorPowerFront = limitThrust(thrust + pitch + yaw);
    motorPowerRear = limitThrust(thrust - pitch + yaw);
#endif

    motorsSetRatio(MOTOR_LEFT, motorPowerLeft);
    motorsSetRatio(MOTOR_RIGHT, motorPowerRight);
    motorsSetRatio(MOTOR_FRONT, motorPowerFront);
    motorsSetRatio(MOTOR_REAR, motorPowerRear);
}

static uint16_t limitThrust(int32_t value) {
    if (value > UINT16_MAX) {
        value = UINT16_MAX;
    } else if (value < 0) {
        value = 0;
    }

    return (uint16_t) value;
}

