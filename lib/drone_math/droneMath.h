#ifndef _DRONE_MATH_H_
#define _DRONE_MATH_H_
#include "Arduino.h"

double limitDouble(double input, double min, double max) {
  if (input < min) { return min; }
  if (input > max) { return max; }
  return input;
}


double veclength(double v[3]) {
  return sqrt((v[0]*v[0]) + (v[1]*v[1]) + (v[2]*v[2]));
}

double vecdot(double a[3],double b[3])
{
  return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

void normalize(double out[3]) {
  double lengthval = veclength(out);
  if (lengthval != 0) {
    out[0] /= lengthval;
    out[1] /= lengthval;
    out[2] /= lengthval;
  }
}


double angle(double a[3],double b[3]) {
  //returns the Angle between two vectors. 0-2PI
  double anorm[3] = {a[0],a[1],a[2]};
  normalize(anorm);
  double bnorm[3] = {b[0],b[1],b[2]};
  normalize(bnorm);
  double dotval = vecdot(anorm,bnorm);
  return acos(dotval);
}



void copy(double out[3],double source[3])
{
  out[0] = source[0];
  out[1] = source[1];
  out[2] = source[2];
}

void cross(double out[3], double a[3],double b[3])
{
  out[0] = (a[1]*b[2]) - (a[2]*b[1]);
  out[1] = (a[2]*b[0]) - (a[0]*b[2]);
  out[2] = (a[0]*b[1]) - (a[1]*b[0]);
}

void scale(double out[3],double vectorIn[3], double factor)
{
  for(int c=0; c<3; c++) { out[c]=vectorIn[c]*factor; }
}

void add(double out[3],double a[3], double b[3])
{
  for(int c=0; c<3; c++) { out[c]=a[c]+b[c]; }
}



//Calculate average of an array
double calculateAverage(int *data, int arraySize) {
  double calculate = 0.0f;
  for (int i=0; i<(arraySize-1); i++) {
    calculate += (double) data[i];
  }
  return calculate/(double) (arraySize-1);
}

double calculateAverageDouble(double *data, int arraySize) {
  double calculate = 0.0;
  for (int i=0; i<(arraySize-1); i++) {
    calculate += data[i];
  }
  return calculate/(double) (arraySize-1);
}

void pushShiftArray(double *data, int arraySize, double newdata) {
  //shifts all entries in an an array to the right. puts newdata on the left [0]
  for (int i=(arraySize-1); i>0; i--) {
    data[i] = data[i-1];
  }
  data[0] = newdata;
}

#endif
