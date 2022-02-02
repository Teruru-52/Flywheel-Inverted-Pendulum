#ifndef _SERVO_H_
#define _SERVO_H_
#include "main.h"

#define servoL 4
#define servoR 13
#define servoC 14

#define CH_ServoL 13
#define CH_ServoR 14
#define CH_ServoC 15

#define SERVO_INIT_L   4600
#define SERVO_INIT_R   4600
#define SERVO_INIT_C   5000

#define SERVO_BRAKE_LR 1500
#define SERVO_BRAKE_C  1500

void SetUpServo();

#endif // _SERVO_H_