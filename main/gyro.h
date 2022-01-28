#ifndef _GYRO_H_
#define _GYRO_H_
#include "main.h"

void GyroInit();
void OffsetCalc();
void GetRawAngle();
void GetRawGyro();
void KalmanInit();
void GetEstAngle();
void GetEstGyro();

#endif  // _GYRO_H_