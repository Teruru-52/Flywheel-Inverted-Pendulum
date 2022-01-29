#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"

#define CHANNEL_L 0
#define CHANNEL_R 1
#define CHANNEL_C 2

#define ROT_DIR_L 23
#define ROT_DIR_R 16
#define ROT_DIR_C 5

#define PWM_PIN_L 27
#define PWM_PIN_R 25
#define PWM_PIN_C 26

#define BRAKE_L 19
#define BRAKE_R 17
#define BRAKE_C 18

#define INPUT_LIMIT_L 360
#define INPUT_LIMIT_R 360
#define INPUT_LIMIT_C 360

#define D_FILTER_COFF       0.025f //3.98Hz

// typedef struct{
//     float KpLR, KdLR, KwLR;
//     float KpC, KdC, KwC;
//     float KC, KC2, KLR;
// }Control_Typedef;

void SetUpWheel();
void getupY();
void ControlC();
void ControlL();
void ControlR();

#endif // _CONTROL_H_