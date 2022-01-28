#include "main.h"
#include "my_header.h"

#define GetUpBt 2
#define ModeBt 12

extern Encoder_Typedef Enc_l, Enc_r, Enc_c;

extern float accX, accY, accZ;
extern float gyroX, gyroY, gyroZ;

extern float theta_X, theta_Y, theta_L, theta_R;
extern float theta_Ldot, theta_Rdot, theta_Ydot, theta_Zdot;
extern float kalAngleL, kalAngleL2, kalAngleR;
extern float kalAngleR2, kalAngleDotL, kalAngleDotR, kalAngleC, kalAngleDotC;

extern float dt;

float degL = 45, degR = 45;

int Mode = 0;
int GetUpBtState = 0;

int DutyIniL = 1020, pwmDutyL, DutyIniR = DutyIniL, pwmDutyR, DutyIniC = DutyIniL, pwmDutyC;
int GetUpX = 0, GetUpY = 0;

int Lok = 0, Rok = 0, Cok = 0;
int GetUpcntL = 0, GetUpcntR = 0, GetUpcntC = 0;

float MtL, MtR, MtC;
float omegaZ = 0.0;
float AjC = 0.0, AjC2 = 0.0, AjL = 0.0, AjR = 0.0;

float getUpDeg = -5.0;

int loopDelay = 5;
//----------------------------

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(GetUpBt, INPUT_PULLUP);
  pinMode(ModeBt, INPUT_PULLUP);
  attachInterrupt(GetUpBt, GetUp, FALLING);
  attachInterrupt(ModeBt, ModeOnOff, FALLING);

  SetUpWheel();
  SetUpEncoder();

  GyroInit();
  OffsetCalc();
  KalmanInit();

  ledcSetup(CH_L, 20000, 10);
  ledcAttachPin(PWM_pinL, CH_L);
  ledcSetup(CH_R, 20000, 10);
  ledcAttachPin(PWM_pinR, CH_R);
  ledcSetup(CH_C, 20000, 10);
  ledcAttachPin(PWM_pinC, CH_C);

  // SetUpServo();
  delay(500);

  //display
  xTaskCreatePinnedToCore(
    disp
    ,  "disp"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL
    ,  0);
}


void loop() {
  GetSamplingTime();
  GetRawAngle();
  GetRawGyro();
  GetEstAngle();
  GetEstGyro();

  GetEulerVel(theta_Xdot, theta_Ydot, theta_Zdot);
  GetEulerAngle(dt);

  GetWheelVel(&Enc_l, &Enc_r, &Enc_c, dt);

  Control();

  if (GetUpY == 0) {
    delay(loopDelay);
  } else {
    delay(1);
  }
}
