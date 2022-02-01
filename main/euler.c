#include "euler.h"

// XYZ Euler

//float roll = 0, pitch = 0, yaw = 0;
//float dot_roll, dot_pitch, dot_yaw;
//
//void GetEulerVel(float p, float q, float r)
//{
//    dot_roll = p + (q * sin(roll * PI / 180) + r * cos(roll * PI / 180)) * atan2(pitch);
//    dot_pitch = q * cos(roll * PI / 180) - r * sin(roll * PI / 180);
//    dot_yaw = p + (q * sin(roll * PI / 180) + r * cos(roll * PI / 180)) * atan2(pitch * PI / 180);
//}
//
//void GetEulerAngle(float ts){
//    roll += dot_roll * ts;
//    pitch += dot_pitch * ts;
//    yaw += dot_yaw * ts;
//}
