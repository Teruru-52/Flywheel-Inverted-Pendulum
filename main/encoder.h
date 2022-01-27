#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "main.h"

#define ENCL_A 32
#define ENCL_B 33
#define ENCR_A 39
#define ENCR_B 36
#define ENCC_A 35
#define ENCC_B 34

// typedef struct{
//     byte pos;
// }Encoder_Typedef;

// void enc_read(Encoder_Typedef *encoder);
// void get_wheel_vel(Encoder_Typedef *encoder);

void SetUpEncoder();
void ENCL_READ();
void ENCR_READ();
void ENCC_READ();
void GetWheelVel(Encoder_Typedef *encoder, Encoder_Typedef *encoder, Encoder_Typedef *encoder);

#endif // _ENCODER_H_
