/*  AeroQuad v3.0.1 - June 2012
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

  October 2013 - Implemented on L3GD20 by Rouan van der Ende

*/


#ifndef _DRONE_GYROSCOPE_IBM160_H_
#define _DRONE_GYROSCOPE_IBM160_H_

#include "droneMath.h"
#include "CurieIMU.h"

bool gyroInitialized = false;
bool gyroCalibrated = false;
double gyroDataRateHz = 400.0;				//sample frequency
double gyroZero[3] = {0.0,0.0,0.0};		//calibration offsets //important!
double gyro[3]         = {0,0,0};
double gyro0[10]; //actually use 5
double gyro1[10]; //actually use 5
double gyro2[10]; //actually use 5
// Axis inversion: -1 = invert, 1 = don't invert
int gyroAxisInversionFactor[3] = {1,-1,-1};

void calibrateGyro();
void initializeGyro() {
  while (gyroInitialized == false) {
      CurieIMU.begin();
      CurieIMU.setGyroRate(gyroDataRateHz);
      CurieIMU.setGyroRange(250);
      gyroInitialized = true;
  }
  while (gyroCalibrated == false) {
    calibrateGyro();
  }
}

void measureGyro() {

    int gyroraw[3];
    CurieIMU.readGyro(gyroraw[0], gyroraw[1], gyroraw[2]);
    // Serial.print("original gyo: " );
    // Serial.print(gyroraw[0]);
    // Serial.print(",");
    // Serial.print(gyroraw[1]);
    // Serial.print(",");
    // Serial.print(gyroraw[2]);
    // Serial.println();
    //apply calibration zero
    gyro[0] = (double) gyroraw[0]- gyroZero[0];
    gyro[1] = (double) gyroraw[1]- gyroZero[1];
    gyro[2] = (double) gyroraw[2]- gyroZero[2];
}


void calibrateGyro() {
  int calibrationsamples = 128;
  int findZero[calibrationsamples];
  int gyroraw[3];

  bool moveTrigger = false;
  for (byte axis = 0; axis <= 2; axis++)
  {
    //collect samples for axis
    for (int i=0; i < calibrationsamples; i++)
    {
      CurieIMU.readGyro(gyroraw[0], gyroraw[1], gyroraw[2]);
      findZero[i] = gyroraw[axis];
      delayMicroseconds(2500); //400hz
    }
    //find average.
    double axisdrift = calculateAverage(findZero, calibrationsamples);
    if (abs(axisdrift) > 300) {
      //bomb if we think the gyro was moved. Not just drift.
      moveTrigger = true;
      //serialStatus("Gyro Calibration failed, moved.");
    } else {
      gyroZero[axis] = axisdrift;
    }
  }
  //There was no problem.
  if (moveTrigger == false) { gyroCalibrated = true; }
}

#endif
