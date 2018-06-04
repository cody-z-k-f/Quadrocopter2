#ifndef _DRONE_MOTORS_101_H_
#define _DRONE_MOTORS_101_H_

#include "Arduino.h"
#include "CurieTimerOne.h"
//#include "Motors.h"
  #define MOTORPIN0    3
  #define MOTORPIN1    5
  #define MOTORPIN2    6
  #define MOTORPIN3    9
  //#define Pwm_duty
  int motorCommand[8] = {0,0,0,0,0,0,0,0};  // LASTMOTOR not know here, so, default at 8 @todo : Kenny, find a better way

  double a=0.255,c=1.023;
  int b = 255,d=1023;

//Motors input
void writeMotors()
{
  for (int ch = 0;ch < 4; ch++) {
    if (motorCommand[ch] <= 1000) { motorCommand[ch] = 1000;}
    if (motorCommand[ch] >= 2000) { motorCommand[ch] = 2000;}
  }
  analogWrite(MOTORPIN0, int(motorCommand[0]*a-b));
  analogWrite(MOTORPIN1, int(motorCommand[1]*a-b));
  analogWrite(MOTORPIN2, int(motorCommand[2]*a-b));
  analogWrite(MOTORPIN3, int(motorCommand[3]*a-b));

  }

void commandAllMotors(int command) {
    analogWrite(3, (command*a-b));
    analogWrite(5, (command*a-b));
    analogWrite(6, (command*a-b));
    analogWrite(9, (command*a-b));
}

void initializeMotors() {

    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    //change the frequeny of the four PWM i/o.
    //Only Refresh the Arduino library can use analogWriteFrequency().
    analogWriteFrequency(3,1000);
    analogWriteFrequency(9,1000);
    analogWriteFrequency(5,1000);
    analogWriteFrequency(6,1000);
    commandAllMotors(1000);                                     // Initialise motors to 1000us (stopped)

}

#endif
