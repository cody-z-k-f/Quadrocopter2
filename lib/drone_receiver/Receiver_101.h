#ifndef _DRONE_RECEIVER_101_H_
#define _DRONE_RECEIVER_101_H_

#include "Arduino.h"

//SET YOUR PINS! TO MATCH RECIEVER CHANNELS
#define CHAN1PIN 7
#define CHAN2PIN 8
#define CHAN3PIN 10
#define CHAN4PIN 11
#define CHAN5PIN 12
#define CHAN6PIN 13

 unsigned long PPM[7]={1000};
 unsigned long pwmLast[6]={0};

//read the time of high pulse.
void pwmHandler(int ch,int pin) {
   if (digitalRead(pin)) {
     pwmLast[ch] = micros();
   }
   else
    {
      PPM[ch] = micros()-pwmLast[ch];
    }

}

void ch1Handler() { pwmHandler(1, CHAN1PIN); }
void ch2Handler() { pwmHandler(2, CHAN2PIN); }
void ch3Handler() { pwmHandler(3, CHAN3PIN); }
void ch4Handler() { pwmHandler(4, CHAN4PIN); }
void ch5Handler() { pwmHandler(5, CHAN5PIN); }
void ch6Handler() { pwmHandler(6, CHAN6PIN); }


void initializeReceiver() {
  pinMode(CHAN1PIN,INPUT_PULLUP);
  pinMode(CHAN2PIN,INPUT_PULLUP);
  pinMode(CHAN3PIN,INPUT);
  pinMode(CHAN4PIN,INPUT);
  pinMode(CHAN5PIN,INPUT);
  pinMode(CHAN6PIN,INPUT);

  attachInterrupt(CHAN1PIN,ch1Handler,CHANGE);
  attachInterrupt(CHAN2PIN,ch2Handler,CHANGE);
  attachInterrupt(CHAN3PIN,ch3Handler,CHANGE);
  attachInterrupt(CHAN4PIN,ch4Handler,CHANGE);
  attachInterrupt(CHAN5PIN,ch5Handler,CHANGE);
  attachInterrupt(CHAN6PIN,ch6Handler,CHANGE);

}

//FUNCTION of reading controller's command.
int getRawChannelValue(int channel) {
  if (PPM[channel]>900&&PPM[channel]<2100) {
    return PPM[channel];
}
else return 0;
}

#endif
