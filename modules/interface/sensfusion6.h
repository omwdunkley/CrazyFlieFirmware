/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
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
 */

#ifndef SENSORFUSION6_H_
#define SENSORFUSION6_H_
#include <stdbool.h>
#include "global.h"

typedef struct {
  Axis3f offset;       //< centers the magnetic field ellipse to origin
  Axis3f scale;        //< spherises the ellipse to a unit sphere
  Axis3f thrust_comp;  //< compensates for thrust. Vector points in direction of magnetic distortion with length*thrust = offset in this direction
} MagCalibObject;



void sensfusion6Init(void);
bool sensfusion6Test(void);

void sensfusion6UpdateQ(Axis3f g, Axis3f a, float dt);
void sensfusion9UpdateQ(Axis3f g, Axis3f a, Axis3f magRaw, uint16_t actuatorThrust, Axis3f* m, float dt);

void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw);
void sensfusion6UpdateWorldAcc(const Axis3f* acc, bool update_offset);
void sensfusion6UpdateWorldAccBias();
void sensfusion6GetWorldAcc(Axis3f* acc_out);



#endif /* SENSORFUSION6_H_ */
