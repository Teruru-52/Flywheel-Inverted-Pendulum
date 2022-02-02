#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "main.h"

#define ENCL_A 32
#define ENCL_B 33
#define ENCR_A 39
#define ENCR_B 36
#define ENCC_A 35
#define ENCC_B 34

typedef struct{
    byte pos;
    int phaseA, phaseB, enc_count;
    float wheel_vel;
}Encoder_Typedef;

void SetUpEncoder();
void EncoderRead(Encoder_Typedef *encoder);
void GetWheelVel(Encoder_Typedef *encoder_l, Encoder_Typedef *encoder_r, Encoder_Typedef *encoder_c, float ts);

#endif // _ENCODER_H_
