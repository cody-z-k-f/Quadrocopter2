// PIDs!!
//////////////////////////////////////////////////////////////////////////////////////

double pid_A_Pgain      =0.9;//1.0;//2.5;    //wobbles too high?
double pid_A_Igain      =0.01;//0.0;//0.02;//0.03;//0.0;//0.2;   //wobbles too high?
double pid_A_Dgain      =0.6;//0.7;//0.8;//0.8;//0.8;//0.4;     //wobbles too high?

double pid_A_I          = 0.0;
double pid_A_inputlast  = 0.0;
short limMax=20,limMin=-20;  //limits of integration
double pid_A_calcPID(double inputA, double target, double timedelta) {
  double outpidA;

  double P = (inputA-target)*pid_A_Pgain;
  pid_A_I += ((inputA-target)*timedelta)*pid_A_Igain;  // I
  double D = (inputA - pid_A_inputlast)/timedelta;       // D
  if(pid_A_I>limMax){
    pid_A_I=limMax;
  }
  if(pid_A_I<limMin){
    pid_A_I=limMin;
  }
  outpidA = P + pid_A_I +(D*pid_A_Dgain);
  // Serial.print("PIDA:");
  // Serial.print(outpidA,2);
  // Serial.println();
  pid_A_inputlast = inputA;
  return outpidA;
}

//////////////////////////////////////////////////////////////////////////////////////

//RED QUAD ARM AND OPPOSITE MOTOR

double pid_B_Pgain      =0.9;  //wobbles too high?
double pid_B_Igain      =0.02;//0.02;//0.0;//0.2; //wobbles too high?
double pid_B_Dgain      =0.6;//0.8;//0.8;   //wobbles too high?

double pid_B_I          = 0.0;
double pid_B_inputlast  = 0.0;

double pid_B_calcPID(double inputB, double target, double timedelta) {
  double outpidB;

  double P = (inputB - target);                          // P
  pid_B_I += ((inputB - target)*timedelta)*pid_B_Igain;  // I
  double D = (inputB - pid_B_inputlast)/timedelta;       // D

  if(pid_B_I>limMax){
    pid_B_I=limMax;
  }
  if(pid_B_I<-limMin){
    pid_B_I=-limMin;
  }

  outpidB = (P*pid_B_Pgain) + pid_B_I +(D*pid_B_Dgain);
  // Serial.print("PIDB:");
  // Serial.println(outpidB,4);

  pid_B_inputlast = inputB;
  return outpidB;
}

//////////////////////////////////////////////////////////////////////////////////////
// YAW PID


double pid_C_Pgain      =2.0; //2.5; //perfect
double pid_C_Igain      =0.0;//0.0;//0.0;//0.0;//0.0; //0.0;  //perfect
double pid_C_Dgain      =0.8;//0.4; //0.8;  //perfect

double pid_C_I          = 0.0;
double pid_C_inputlast  = 0.0;

double pid_C_calcPID(double input, double target, double timedelta) {
  double outpid;

  double P = (input - target);                          // P
  pid_C_I += ((input - target)*timedelta)*pid_C_Igain;  // I
  double D = (input - pid_C_inputlast)/timedelta;       // D
  outpid = (P*pid_C_Pgain) + pid_C_I + (D*pid_C_Dgain);
  // Serial.print("PIDC:");
  // Serial.print(P,4);
  // Serial.print(",");
  // Serial.print(pid_C_I,4);
  // Serial.print(",");
  // Serial.print(D,4);
  // Serial.println();

  pid_C_inputlast = input;
  return outpid;
}

//////////////////////////////////////////////////////////////////////////////////////
