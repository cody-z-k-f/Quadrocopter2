#include "Arduino.h"
#define Ki 0.005f          // integral gain governs rate of convergenceof gyroscope biases
#define Kp 2.0f                       // proportional gain governs rate of convergence toaccelerometer
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;        // quaternion elements representing theestimated orientation
static float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
void AHRSupdate(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                double dt) {
            float norm;
            double  halfT=dt/2;         // half the sample period
            float hx, hy, hz, bx, bz;
            float vx, vy, vz, wx, wy, wz; //v*The component of the gravity on the three axis calculated by the present attitude
            float ex=0.0, ey=0.0, ez=0.0;

            // auxiliary variables to reduce number of repeated operations
            float q0q0 = q0*q0;
            float q0q1 = q0*q1;
            float q0q2 = q0*q2;
            float q0q3 = q0*q3;
            float q1q1 = q1*q1;
            float q1q2 = q1*q2;
            float q1q3 = q1*q3;
            float q2q2 = q2*q2;
            float q2q3 = q2*q3;
            float q3q3 = q3*q3;

           //change dgree to radin.
            gx *= 0.0174533f;
	          gy *= 0.0174533f;
	          gz *= 0.0174533f;

            // normalise the measurements
            norm = sqrt(ax*ax + ay*ay + az*az);
            ax = ax / norm;
            ay = ay / norm;
            az = az / norm;

            if(mx!=0&&my!=0&&mz!=0){
              norm = sqrt(mx*mx + my*my + mz*mz);
              mx = mx / norm;
              my = my / norm;
              mz = mz / norm;

              // compute reference direction of magnetic field
              hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
              hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
              hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);
              bx = sqrt((hx*hx) + (hy*hy));
              bz = hz;

              wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
              wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
              wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);

              ex = my*wz - mz*wy;
              ey = mz*wx - mx*wz;
              ez = mx*wy - my*wx;
            }


// estimated direction of gravity and magnetic field (v and w)
            vx = 2*(q1q3 - q0q2);
            vy = 2*(q0q1 + q2q3);
            vz = q0q0 - q1q1 - q2q2 + q3q3;


// error is sum of cross product between reference direction of fields and direction measured by sensors 体现在加速计补偿和磁力计补偿，因为仅仅依靠加速计补偿没法修正Z轴的变差，所以还需要通过磁力计来修正Z轴。（公式28）。《四元数解算姿态完全解析及资料汇总》的作者把这部分理解错了，不是什么叉积的差，而叉积的计算就是这样的。计算方法是公式10。
            ex+= (ay*vz - az*vy);
            ey+= (az*vx - ax*vz);
            ez+= (ax*vy - ay*vx);

            // integral error scaled integral gain
            exInt = exInt + ex*Ki*dt;
            eyInt = eyInt + ey*Ki*dt;
            ezInt = ezInt + ez*Ki*dt;
            // adjusted gyroscope measurements
            gx = gx + Kp*ex + exInt;
            gy = gy + Kp*ey + eyInt;
            gz = gz + Kp*ez + ezInt;

            // integrate quaternion rate and normalize

            q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
            q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
            q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
            q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

            // normalise quaternion
            norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
            q0 = q0 / norm;
            q1 = q1 / norm;
            q2 = q2 / norm;
            q3 = q3 / norm;
}
