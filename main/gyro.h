#ifndef _GYRO_H_
#define _GYRO_H_
#include "main.h"

#define LIMIT_ANGLE 20

typedef struct{
  float axis_x, axis_y, axis_z;
}Gyro_Typedef;

void GyroInit();
void OffsetCalc();
void GetRawAngle();
void GetRawGyro();
void KalmanInit();
void GetEstAngle();
void GetEstGyro();
void WheelBrake();

#endif  // _GYRO_H_
