#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"

#define PWM_pinL 27
#define brakeL 19
#define rote_pinL 23

#define PWM_pinR 25
#define brakeR 17
#define rote_pinR 16

#define PWM_pinC 26
#define brakeC 18
#define rote_pinC 5

#define CH_L 0
#define CH_R 1
#define CH_C 2

#define D_FILTER_COFF       0.025f //3.98Hz

// typedef struct{
//     float KpLR, KdLR, KwLR;
//     float KpC, KdC, KwC;
//     float KC, KC2, KLR;
// }Control_Typedef;

void SetUpWheel();
void disp();
void getupY();

#endif // _CONTROL_H_