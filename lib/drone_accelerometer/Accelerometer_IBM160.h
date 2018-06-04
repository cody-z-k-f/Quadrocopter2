/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <droneMath.h>
#ifndef _AEROQUAD_ACCELEROMETER_IBM160_H_
#define _AEROQUAD_ACCELEROMETER_IBM160_H_

//#include "Serial.h"
#include "CurieIMU.h"
bool accelInitialized = false;
bool accelCalibrated = false;

double accelZero[3] = {0.0,0.0,0.0};
double accel[3] = {0,0,0};
double accelDataRateHz = 400.0;        //sample frequence

void calibrateAccel();
void initializeAccel() {
  while (accelInitialized == false) {

        CurieIMU.begin();
        CurieIMU.setAccelerometerRate(accelDataRateHz);
        CurieIMU.setAccelerometerRange(4);
        accelInitialized = true;
      //} else { Serial.println("Accel error cannot connect"); }
  }//while

  while (accelCalibrated == false) {
    calibrateAccel();
  }

}

void calibrateAccel()  {
  /* we take a bunch of samples, even them out and offset for what is down. */
  int calibrationsamples = 128;
  int findZero[128];
  int accelraw[3];

  for (byte axis = 0; axis <= 2; axis++) {
    //sample data for this axis
    for (int i=0; i < calibrationsamples; i++) {
        CurieIMU.readAccelerometer(accelraw[0], accelraw[1], accelraw[2]);
        findZero[i] = accelraw[axis];
    }
    //find average.
    double axisavg = calculateAverage(findZero, calibrationsamples);
    accelZero[axis] = axisavg;
    //Serial.println(accelZero[axis]);
  }
  accelCalibrated = true;
}

void measureAccel() {
  if (accelInitialized == false) { Serial.print("Warning Accelerometer not initialized."); }
  if (accelCalibrated == false) { Serial.print("Warning Accelerometer not calibrated."); }

  // combine high and low bytes, then shift right to discard lowest 4 bits (which are meaningless)
  // GCC performs an arithmetic right shift for signed negative numbers, but this code will not work
  // if you port it to a compiler that does a logical right shift instead.
  int accelraw[3];
  CurieIMU.readAccelerometer(accelraw[0], accelraw[1], accelraw[2]);
  // Serial.print("unprocessed accelraw: ");
  // Serial.print(accelraw[0]);
  // Serial.print(" ");
  // Serial.print(accelraw[1] );
  // Serial.print(" ");
  // Serial.print(accelraw[2]);
  // Serial.println();
  double accelrawd[3];
  accelrawd[0] = (double) accelraw[0]-accelZero[0];
  accelrawd[1] = (double) accelraw[1]-accelZero[1];
  accelrawd[2] = (double) accelraw[2]-accelZero[2];

  accel[0] = accelrawd[0];
  accel[1] = accelrawd[1];
  accel[2] = accelrawd[2];
}

#endif
