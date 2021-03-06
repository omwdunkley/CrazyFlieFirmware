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
#include <math.h>

#include "sensfusion6.h"
#include "imu.h"
#include "param.h"
#include "global.h"
#include "log.h"
#include "debug.h"
//#define MADWICK_QUATERNION_IMU

#define TWO_KP_DEF  (2.0f * 0.4f) // 2 * proportional gain
#define TWO_KI_DEF  (2.0f * 0.001f) // 2 * integral gain

float twoKp = TWO_KP_DEF;    // 2 * proportional gain (Kp)
float twoKi = TWO_KI_DEF;    // 2 * integral gain (Ki)
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;  // integral error terms scaled by Ki

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame

//Currently using mag data for 9d fusion
bool magImu = false;
//Previously using mag data for 9d fusion
bool preMagImu = false;

Axis3f grav; // estimated gravity direction
Axis3f grav_offset; // estimated offset. computed when the flie has no thrust and is level //TODO: got to find a better way to do this
Axis3f accWorldRaw; //not compensated world acceleration

LOG_GROUP_START(attitude)
LOG_ADD(LOG_FLOAT, q0, &q0)
LOG_ADD(LOG_FLOAT, q1, &q1)
LOG_ADD(LOG_FLOAT, q2, &q2)
LOG_ADD(LOG_FLOAT, q3, &q3)
LOG_GROUP_STOP(attitude)

LOG_GROUP_START(gravoffset)
LOG_ADD(LOG_FLOAT, x, &grav_offset.x)
LOG_ADD(LOG_FLOAT, y, &grav_offset.y)
LOG_ADD(LOG_FLOAT, z, &grav_offset.z)
LOG_GROUP_STOP(gravoffset)


// hmc calibration data
static MagCalibObject magCalib;

PARAM_GROUP_START(magCalib)
PARAM_ADD(PARAM_FLOAT, off_x, &magCalib.offset.x)
PARAM_ADD(PARAM_FLOAT, off_y, &magCalib.offset.y)
PARAM_ADD(PARAM_FLOAT, off_z, &magCalib.offset.z)
PARAM_ADD(PARAM_FLOAT, scale_x, &magCalib.scale.x)
PARAM_ADD(PARAM_FLOAT, scale_y, &magCalib.scale.y)
PARAM_ADD(PARAM_FLOAT, scale_z, &magCalib.scale.z)
PARAM_ADD(PARAM_FLOAT, thrust_x, &magCalib.thrust_comp.x)
PARAM_ADD(PARAM_FLOAT, thrust_y, &magCalib.thrust_comp.y)
PARAM_ADD(PARAM_FLOAT, thrust_z, &magCalib.thrust_comp.z)
PARAM_GROUP_STOP(magCalib)




static bool isInit;

// TODO: Make math util file
static float invSqrt(float x);

void sensfusion6Init() {
    if (isInit)
        return;
    grav_offset.x = 0;
    grav_offset.y = 0;
    grav_offset.z = 0;
    isInit = TRUE;

    magCalib.scale.x = 0;
    magCalib.scale.y = 0;
    magCalib.scale.z = 0;
    magCalib.offset.x = 0;
    magCalib.offset.y = 0;
    magCalib.offset.z = 0;
    magCalib.thrust_comp.x = 0;
    magCalib.thrust_comp.y = 0;
    magCalib.thrust_comp.z = 0;
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

bool sensfusion6Test(void) {
    return isInit;
}
// MAHONY_QUATERNION_IMU
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date     Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
void sensfusion6UpdateQ(Axis3f g, Axis3f a, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    //TODO WHy is this in degrees?
    g.x = g.x * DEG2RAD;
    g.y = g.y * DEG2RAD;
    g.z = g.z * DEG2RAD;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        a.x *= recipNorm;
        a.y *= recipNorm;
        a.z *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (a.y * halfvz - a.z * halfvy);
        halfey = (a.z * halfvx - a.x * halfvz);
        halfez = (a.x * halfvy - a.y * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            g.x += integralFBx;  // apply integral feedback
            g.y += integralFBy;
            g.z += integralFBz;
        } else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        g.x += twoKp * halfex;
        g.y += twoKp * halfey;
        g.z += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    g.x *= (0.5f * dt);   // pre-multiply common factors
    g.y *= (0.5f * dt);
    g.z *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * g.x - qc * g.y - q3 * g.z);
    q1 += (qa * g.x + qc * g.z - q3 * g.y);
    q2 += (qa * g.y - qb * g.z + q3 * g.x);
    q3 += (qa * g.z + qb * g.y - qc * g.x);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    grav.x = 2.f * (q1 * q3 - q0 * q2);
    grav.y = 2.f * (q0 * q1 + q2 * q3);
    grav.z = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}



void getMagnetometerCalibrated(const Axis3f* magIn, const uint16_t thrust, Axis3f* magOut){

    // If calibrated, return calibrated values, else return (0,0,0)
    if ((magCalib.scale.x  != 0.f) && //check divide by 0
            (magCalib.scale.y  != 0.f) &&
            (magCalib.scale.z  != 0.f)) {
        const float thrust_f = (float) thrust;
        magOut->x = (magIn->x - magCalib.offset.x) / magCalib.scale.x - thrust_f * magCalib.thrust_comp.x;
        magOut->y = (magIn->y - magCalib.offset.y) / magCalib.scale.y - thrust_f * magCalib.thrust_comp.y;
        magOut->z = (magIn->z - magCalib.offset.z) / magCalib.scale.z - thrust_f * magCalib.thrust_comp.z;
    } else {
        magOut->x = 0;
        magOut->y = 0;
        magOut->z = 0;
    }
}

// Madgwick's implementation of Mayhony's AHRS algorithm.
void sensfusion9UpdateQ(Axis3f g, Axis3f a, Axis3f magRaw, uint16_t actuatorThrust, Axis3f* m, float dt) {
    if (preMagImu != magImu){
        DEBUG_PRINT("IMU Method changed, SenFu reInit.\n");
        // Changed imu method, reset
        // TODO check led is doing what it should
        isInit = false;
        sensfusion6Init();

    }
    preMagImu = magImu;

    if (!magImu){
        return sensfusion6UpdateQ(g,a,dt);
    }

    getMagnetometerCalibrated(&magRaw, actuatorThrust, m);


    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((m->x == 0.0f) && (m->y == 0.0f) && (m->z == 0.0f)) {
        sensfusion6UpdateQ(g, a, dt);
        return;
    }

    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        a.x *= recipNorm;
        a.y *= recipNorm;
        a.z *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(m->x * m->x + m->y * m->y + m->z * m->z);
        m->x *= recipNorm;
        m->y *= recipNorm;
        m->z *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (m->x * (0.5f - q2q2 - q3q3) + m->y * (q1q2 - q0q3) + m->z * (q1q3 + q0q2));
        hy = 2.0f * (m->x * (q1q2 + q0q3) + m->y * (0.5f - q1q1 - q3q3) + m->z * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (m->x * (q1q3 - q0q2) + m->y * (q2q3 + q0q1) + m->z * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (a.y * halfvz - a.z * halfvy) + (m->y * halfwz - m->z * halfwy);
        halfey = (a.z * halfvx - a.x * halfvz) + (m->z * halfwx - m->x * halfwz);
        halfez = (a.x * halfvy - a.y * halfvx) + (m->x * halfwy - m->y * halfwx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / dt);    // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / dt);
            integralFBz += twoKi * halfez * (1.0f / dt);
            g.x += integralFBx;  // apply integral feedback
            g.y += integralFBy;
            g.z += integralFBz;
        } else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        g.x += twoKp * halfex;
        g.y += twoKp * halfey;
        g.z += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    g.x *= (0.5f * (1.0f / dt));     // pre-multiply common factors
    g.y *= (0.5f * (1.0f / dt));
    g.z *= (0.5f * (1.0f / dt));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * g.x - qc * g.y - q3 * g.z);
    q1 += (qa * g.x + qc * g.z - q3 * g.y);
    q2 += (qa * g.y - qb * g.z + q3 * g.x);
    q3 += (qa * g.z + qb * g.y - qc * g.x);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    grav.x = 2.f * (q1 * q3 - q0 * q2);
    grav.y = 2.f * (q0 * q1 + q2 * q3);
    grav.z = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw) {
    // TODO why go back to degrees...?
    *yaw = atan2(2 * q1 * q2 - 2 * q0 * q3, 2 * q0 * q0 + 2 * q1 * q1 - 1) * RAD2DEG;
    *pitch = atan(grav.x / sqrt(grav.y * grav.y + grav.z * grav.z)) * RAD2DEG;
    *roll = atan(grav.y / sqrt(grav.x * grav.x + grav.z * grav.z)) * RAD2DEG;
}

void quatMul(const float q1w, const float q1x, const float q1y, const float q1z, const float q2w, const float q2x, const float q2y, const float q2z, float* q3w, float* q3x, float* q3y, float* q3z) {
    *q3w = q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z;
    *q3x = q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y;
    *q3y = q1w * q2y + q1y * q2w + q1z * q2x - q1x * q2z;
    *q3z = q1w * q2z + q1z * q2w + q1x * q2y - q1y * q2x;
}

void quatConj(const float q1w, const float q1x, const float q1y, const float q1z, float* q2w, float* q2x, float* q2y, float* q2z) {
    *q2w = q1w;
    *q2x = -q1y;
    *q2y = -q1z;
    *q2z = -q1x;
}

void sensfusion6UpdateWorldAccBias() {
    //Basically a lowpass filter
    grav_offset.x = grav_offset.x * 0.98 + accWorldRaw.x * 0.02;
    grav_offset.y = grav_offset.y * 0.98 + accWorldRaw.y * 0.02;
    grav_offset.z = grav_offset.z * 0.98 + accWorldRaw.z * 0.02;
}

void sensfusion6GetWorldAcc(Axis3f* acc_out) {
    acc_out->x = accWorldRaw.x - grav_offset.x;
    acc_out->y = accWorldRaw.y - grav_offset.y;
    acc_out->z = accWorldRaw.z - grav_offset.z;
}
void sensfusion6UpdateWorldAcc(const Axis3f* acc, bool update_offset) {
    // Gravity compensated acceleration
    accWorldRaw.x = acc->x - grav.x;
    accWorldRaw.y = acc->y - grav.y;
    accWorldRaw.z = acc->z - grav.z;
    float temp;

    // Mul q with grav
    float q2w, q2x, q2y, q2z;
    quatMul(q0, q1, q2, q3, 0.f, accWorldRaw.x, accWorldRaw.y, accWorldRaw.z, &q2w, &q2x, &q2y, &q2z);

//    // Conjugate
//    float q1w,q1x,q1y,q1z;
//    quatConj(q0,   q1,   q2,   q3,
//              &q1w, &q1x, &q1y, &q1z);
//
//    // Mul that with Conj to get world frame
//    quatMul(q2w,q2x,q2y,q2z,
//             q1w,q1x,q1y,q1z,
//             0, &accWorldRaw.x, &accWorldRaw.y, &accWorldRaw.z);

    // Mul that with Conj to get world frame
    quatMul(q2w, q2x, q2y, q2z, q0, -q1, -q2, -q3, //conjugate of q
            &temp, &(accWorldRaw.x), &(accWorldRaw.y), &(accWorldRaw.z));

    if (update_offset) {
        sensfusion6UpdateWorldAccBias();
    }
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
PARAM_GROUP_START(sensorfusion6)
PARAM_ADD(PARAM_FLOAT, kp, &twoKp)
PARAM_ADD(PARAM_FLOAT, ki, &twoKi)
PARAM_ADD(PARAM_UINT8, magImu, &magImu)
PARAM_GROUP_STOP(sensorfusion6)
