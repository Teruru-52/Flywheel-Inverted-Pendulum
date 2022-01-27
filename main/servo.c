#include "servo.h"

void SetUpServo(){
    ledcSetup(CH_ServoL, 50, 16);
    ledcAttachPin(servoL, CH_ServoL);
    ledcWrite(CH_ServoL, SERVO_INIT_L);
    ledcSetup(CH_ServoR, 50, 16);
    ledcAttachPin(servoR, CH_ServoR);
    ledcWrite(CH_ServoR, SERVO_INIT_R);
    ledcSetup(CH_ServoC, 50, 16);
    ledcAttachPin(servoC, CH_ServoC);
    ledcWrite(CH_ServoC, SERVO_INIT_C);
}