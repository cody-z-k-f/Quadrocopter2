/**********************************************************************
  DRONE v1.1
  open flight controller

  http://fluentart.com
  https://github.com/fluentart/drone2

**********************************************************************/


//#include "UserConfiguration.h"  // Edit this file first before uploading to the drone
//#include <Wire.h>               // Needed for I2C sensors
#include <droneMath.h>
#include "stabilisation.h"
#include "Motors_101.h"
#include "Gyroscope_IBM160.h"
#include <Accelerometer_IBM160.h>
#include <Magnetometer_MAG3110.h>
#include <Receiver_101.h>
#include <MadgwickAHRS.h>
#include <NewOrientation.h>

#define SAMPLELENGTH 15  //sample number of Accelerometer

double pidoutA = 0;
double pidoutB = 0;
double pidoutC = 0;

// SENSORS
int recievercounter = 0;
int gyrocounter = 0;
int compasscounter = 0;
int orientationcounter = 0; //the times of refreshing attitude.

//array for accelerometer to Mean filter
double accel0[SAMPLELENGTH];
double accel1[SAMPLELENGTH];
double accel2[SAMPLELENGTH];

double motorthrottles[4] = {0,0,0,0};
double recieverThrotttle = 1000.0;
double recieverPitch = 0;
double recieverRoll = 0;
double recieverYaw = 0;
//double recieverPIDratio = 1;
int channle[7]={0};

int counter = 0;
int sw = 0;  //safe switch.
//Madgwick filter;
float roll, pitch, heading;  //
float I_anglel_yaw=0.0,I_anglel_pitch=0.0,I_anglel_roll=0.0;//Initialization of attitude angle.

//output of attitude angle.
float out_anglel_yaw;
float out_anglel_pitch;
float out_anglel_roll;
int initial_Yaw = 1,initial_plot=1;
float I_yaw=0.0;
float out_yaw;


/**********************************************************************
  LOAD
*********************************************************************/



/**********************************************************************
  SETUP
*********************************************************************/
int bootcounter = 0;
void setup() {

  initializeAccel();  //zero calibration is now part of initialization process.

  initializeGyro();  //zero calibration is now part of initialization process.

  initializeMagnetometer();

  initializeReceiver();

  initializeMotors();

  Serial.begin(115200);
}

/**********************************************************************
  LOOP
*********************************************************************/

void loop() {

  if(getRawChannelValue(5)>1500)
     {
       sw=1;  //turn on the safe switch.
     }
  if(getRawChannelValue(5)<1500)
     {
       sw=0;  //turn off .
     }

  newOrientationUpdate();
  //testreciever();
  //printtests();
  //testaccel();
  //testPID();
  //testgyro();
  //viewPPM();

  //############################
  // UNCOMMENT FOR DEBUG INFO!
  //############################
  // printtests();
  //############################
  counter++;
}



/////////////////////////////////////////////
// ORIENTATION
unsigned long timerOrientation;

bool newOrientationUpdate() {
 if(abs(micros() - timerOrientation) >= (1/250)*1000000) //abs for when micros() rolls over. gyroDataRateSec is set in Gyroscope.h and your Gyroscope_XXXXX.h
  {
   orientationcounter++;
   double deltatimeseconds = (double) abs(micros() - timerOrientation) / 1000000.0;
   timerOrientation = micros();
   //Serial.print("deltatimeseconds: ");
   //Serial.println(deltatimeseconds,5);
   measureAccel();  //get readings from your accelerometer

   pushShiftArray(accel0, SAMPLELENGTH, accel[0]); //push into array;
   pushShiftArray(accel1, SAMPLELENGTH, accel[1]); //push into array;
   pushShiftArray(accel2, SAMPLELENGTH, accel[2]); //push into array;
   accel[0] = calculateAverageDouble(accel0, SAMPLELENGTH); //calculates new moving average of last samples. This should clean the noise.
   accel[1] = calculateAverageDouble(accel1, SAMPLELENGTH); //calculates new moving average of last samples. This should clean the noise.
   accel[2] = calculateAverageDouble(accel2, SAMPLELENGTH); //calculates new moving average of last samples. This should clean the noise.

    measureMagnetometerRaw();  //get reading from Mag.

    measureGyro();  //get readings from your gyro.

    gyro[0] = convertRawGyro(gyro[0]);
    gyro[1] = convertRawGyro(gyro[1]);
    gyro[2] = convertRawGyro(gyro[2]);

    accel[0] = convertRawAcceleration(accel[0]);
    accel[1] = convertRawAcceleration(accel[1]);
    accel[2] = convertRawAcceleration(accel[2]);

    AHRSupdate(gyro[0], gyro[1], gyro[2],
               accel[0],accel[1],accel[2],
               compass[0],compass[1],compass[2],  //compass can be zero.
               deltatimeseconds);
     //orientationDrift(accel[0],accel[1],accel[2],compass[0],compass[1],compass[2]);
     //orientationUpdate(gyro[0], gyro[1], gyro[2], deltatimeseconds);
     //filter.update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2],compass[0],compass[1],compass[2]);
     //filter.updateIMU(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
  // print the heading, pitch and roll
  // roll = filter.getRoll();
  // pitch = filter.getPitch();
  // heading = filter.getYaw();
  // Serial.print("Orientation: ");
  // Serial.print(heading);
  // Serial.print(" ");
  // Serial.print(pitch);
  // Serial.print(" ");
  // Serial.println(roll);


  //get command from receiver.
    recieverThrotttle=getRawChannelValue(3);
    if(recieverThrotttle>1800){
      recieverThrotttle=1800;
    }
    recieverPitch=getRawChannelValue(2);
    if(recieverPitch>1490&&recieverPitch<1510){
      recieverPitch=1500;
    }
    recieverRoll=getRawChannelValue(1);
    if(recieverRoll>1490&&recieverRoll<1510){
      recieverRoll=1500;
    }
    recieverYaw=getRawChannelValue(4);

   //get the attitude angle.
    float anglel_pitch = -asin(2*q0*q2-2*q1*q3)*57.3;
    float anglel_roll = -atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.3;
    float anglel_yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.3;

   //Initialise Pitch,Roll,Yaw to zero.
    if(initial_plot&&orientationcounter>=1500)
    {
      I_anglel_pitch=anglel_pitch;
      I_anglel_roll=anglel_roll;
      initial_plot=0;
    }

   // initialize the yaw
    if(initial_Yaw&&orientationcounter>=1500&&sw==1){
      I_yaw=anglel_yaw;
      initial_Yaw=0;
      orientationcounter=0;
    }

    if(!initial_plot){
      out_anglel_pitch=anglel_pitch-I_anglel_pitch;
      out_anglel_roll=anglel_roll-I_anglel_roll;
    }

    if(!initial_Yaw){
      out_yaw=convertYaw(anglel_yaw,I_yaw);
    }

      //Serial.print("p:");
      //Serial.print("Orientation: ");
      Serial.print(out_anglel_pitch);
      //Serial.print(100);
      Serial.print(" ");
      Serial.println(out_anglel_roll);
      //Serial.print(" ");
      //Serial.println(out_yaw);
      //Serial.println(orientationcounter);

    if (abs(deltatimeseconds) < 0.6) {
      pidoutA = pid_A_calcPID(out_anglel_pitch, (0.09*(recieverPitch-1500)), deltatimeseconds);  //WHITE   FRONT  PITCH/ELEV
      pidoutB = pid_B_calcPID(out_anglel_roll, (-0.09*(recieverRoll-1500)), deltatimeseconds);   //RED     LEFT   ROLL
      pidoutC = pid_C_calcPID(out_yaw, (-0.18*(recieverYaw-1500)), deltatimeseconds);      //GREEN   UP     YAW

      //PID safe.
      if(recieverThrotttle<1100){
        pidoutA=0;
        pidoutB=0;
        pidoutC=0;
      }
    }


    //do proportional control. SEE stabilisation.ino and api.ino
        //QUAD + SETUP
        //motorCommand[0] = recieverThrotttle - pidoutA;//+ pidoutC;
        //motorCommand[1] = recieverThrotttle - pidoutB;//- pidoutC;
        //motorCommand[2] = recieverThrotttle + pidoutA;//+ pidoutC;
        //motorCommand[3] = recieverThrotttle + pidoutB;//- pidoutC;


        //QUAD X SETUP
        //  if (pidoutC > 100.0) { pidoutC = 100.0;}
        //  if (pidoutC < -100.0) { pidoutC = -100.0;}
          motorCommand[0] = recieverThrotttle- pidoutA- pidoutB - pidoutC;
          motorCommand[1] = recieverThrotttle- pidoutA+ pidoutB + pidoutC;
          motorCommand[2] = recieverThrotttle+ pidoutA+ pidoutB - pidoutC;
          motorCommand[3] = recieverThrotttle+ pidoutA- pidoutB + pidoutC;

      //if safe switch is on send signal to motors.
      if(sw==1)
      {
          writeMotors();
      }
    return true;
  }
  else
  {
    return false;
  }
}


float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

float convertYaw(float YawRaw,float zero_Yaw){
  //ensure the yaw is always between 0 to 90.
   float o_yaw;
    if(abs(zero_Yaw)<=90){
        o_yaw=YawRaw-zero_Yaw;
    }
    else{
      if(zero_Yaw>0&&YawRaw<0){
        o_yaw=360+YawRaw-zero_Yaw;
      }
      else if(zero_Yaw<0&&YawRaw>0){
        o_yaw=YawRaw-zero_Yaw-360;
      }
      else o_yaw=YawRaw-zero_Yaw;
    }
  return o_yaw;
}



//Serial teset function.

void testreciever() {
    Serial.print(" CH1: ");
    Serial.print(getRawChannelValue(1));  //ELEV UP/DOWN
    Serial.print(" CH2: ");
    Serial.print(getRawChannelValue(2));  //AILER ROLL LEFT/RIGHT
    Serial.print(" CH3: ");
    Serial.print(getRawChannelValue(3));  //THROTTLE
    Serial.print(" CH4: ");
    Serial.print(getRawChannelValue(4));  //RUDDER TURN LEFT/RIGHT
    Serial.print(" CH5: ");
    Serial.print(getRawChannelValue(5));  //EXTRA SWITCH G
    Serial.print(" CH6: ");
    Serial.print(getRawChannelValue(6));  //EXTRA DIAL VR
    Serial.println();
    Serial.print("],\"motorthrottles\":[");
    Serial.print(motorCommand[0]);
    Serial.print(",");
    Serial.print(motorCommand[1]);
    Serial.print(",");
    Serial.print(motorCommand[2]);
    Serial.print(",");
    Serial.print(motorCommand[3]);
    Serial.println();
}

void testaccel() {
    Serial.print(" ax: ");
    Serial.print(accel[0],4);
    Serial.print(" ay: ");
    Serial.print(accel[1],4);
    Serial.print(" az: ");
    Serial.print(accel[2],4);
    Serial.println();
}


void testgyro() {
  /*
  DEBUG gyro raw array before averaging
  for (int x = 0; x < SAMPLELENGTH; x++) {
    Serial.print(gyro0[x], 1);
    Serial.print(" ");
  }
  */
  Serial.print(" gx: ");
  Serial.print(gyro[0],4);
  Serial.print(" gy: ");
  Serial.print(gyro[1],4);
  Serial.print(" gz: ");
  Serial.print(gyro[2],4);
  Serial.println();
}

void testPID()
{
  Serial.print("PID:");
  Serial.print(pidoutA,4);
  Serial.print(",");
  Serial.print(pidoutB,4);
  Serial.print(",");
  Serial.print(pidoutC,4);
  //Serial.print(",");
  //Serial.print(recieverPIDratio,4);
  Serial.println();

}
