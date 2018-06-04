/*
  AeroQuad v3.0 - April 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
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


#ifndef _AEROQUAD_MAGNETOMETER_MAG3110_H_
#define _AEROQUAD_MAGNETOMETER_MAG3110_H_

#include "Arduino.h"
//#include "Compass.h"
#include "Wire.h"

bool compassInitialized = false;
byte id;
byte xhm,xlm,yhm,ylm,zhm,zlm;
int x_max=767,x_min=138,y_max=-485,y_min=-1130,z_max=1316,z_min=747;
int x_offset,y_offset,z_offset;
double compass[3]      = {0,0,0};              // L3GD20_read(); to update
//void readSpecificMag(float *rawMag);

void initializeMagnetometer()
{
  while (compassInitialized == false)
  {
    Wire.begin();
    Wire.beginTransmission(0x0E);
    Wire.write(0x07);
    Wire.endTransmission();
    Wire.requestFrom(0x0E, 1);
    byte id=Wire.read();
    Serial.begin(9600);
    if(id==0xc4)
    {

    Wire.beginTransmission(0x0E);
    Wire.write(0x11);
    Wire.write(0x80);
    Wire.endTransmission();

    Wire.beginTransmission(0x0E);
    Wire.write(0x10);
    Wire.write(0x19);
    Wire.endTransmission();
   }
   compassInitialized = true;
 }
 x_offset=(x_max+x_min)/2;
 y_offset=(y_max+y_min)/2;
 z_offset=(z_max+z_min)/2;

 }

void measureMagnetometerRaw() {
  Wire.beginTransmission(0x0E);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.requestFrom(0x0E, 6);
  if(6<=Wire.available()){
  xhm=Wire.read();
  xlm=Wire.read();
  yhm=Wire.read();
  ylm=Wire.read();
  zhm=Wire.read();
  zlm=Wire.read();
 }
 // Serial.print("singledata:");
 // Serial.print(xhm);
 // Serial.print(",");
 // Serial.print(xlm);
 // Serial.print(",");
 // Serial.print(yhm);
 // Serial.print(",");
 // Serial.print(ylm);
 // Serial.print(",");
 // Serial.print(zhm);
 // Serial.print(",");
 // Serial.print(zlm);
 // Serial.println();
  // combine high and low bytes
  int compassx = (int16_t)(xhm << 8 | xlm);
  int compassy = (int16_t)(yhm << 8 | ylm);
  int compassz = (int16_t)(zhm << 8 | zlm);



  compass[0] = compassx-x_offset;
  compass[1] = compassy-y_offset;
  compass[2] = compassz-z_offset;


  // Serial.print("c:");
  // Serial.print(compass[0]);
  // Serial.print(",");
  // Serial.print(compass[1]);
  // Serial.print(",");
  // Serial.print(compass[2]);
  // Serial.println();

//
  // if(compass[0]>x_max){
  //   x_max=compass[0];
  // }
  // if(compass[0]<x_min){
  //   x_min=compass[0];
  // }
  //
  // if(compass[1]>y_max){
  //   y_max=compass[1];
  // }
  // if(compass[1]<y_min){
  //   y_min=compass[1];
  // }
  //
  // if(compass[2]>z_max){
  //   z_max=compass[2];
  // }
  // if(compass[2]<z_min){
  //   z_min=compass[2];
  // }
  //
  // Serial.print("of:");
  // Serial.print(x_max);
  // Serial.print(",");
  // Serial.print(x_min);
  // Serial.print(",");
  // Serial.print(y_max);
  // Serial.print(",");
  // Serial.print(y_min);
  // Serial.print(",");
  // Serial.print(z_max);
  // Serial.print(",");
  // Serial.println(z_min);

}

#endif
