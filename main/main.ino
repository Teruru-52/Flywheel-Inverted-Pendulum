#include "main.h"
#include "my_header.h"

#define GETUP_PIN 2
#define MODE_PIN 12

extern Encoder_Typedef Enc_l, Enc_r, Enc_c;

extern float accX, accY, accZ;
extern float gyroX, gyroY, gyroZ;

extern float theta_X, theta_Y, theta_L, theta_R;
extern float theta_Ldot, theta_Rdot, theta_Ydot, theta_Zdot;
extern float kalAngleL, kalAngleL2, kalAngleR;
extern float kalAngleR2, kalAngleDotL, kalAngleDotR, kalAngleC, kalAngleDotC;

extern float dt;

int mode = 0;
int state = 0;

float omegaZ = 0.0;
//----------------------------

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(GETUP_PIN, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT_PULLUP);
  attachInterrupt(GETUP_PIN, GetUp, FALLING);
  attachInterrupt(MODE_PIN, ModeOnOff, FALLING);

  SetUpWheel();
  // SetUpEncoder();

  GyroInit();
  OffsetCalc();
  KalmanInit();

  // SetUpServo();
  // delay(500);

  DispInit();
}


void loop() {
  GetSamplingTime();
  GetRawAngle();
  GetRawGyro();
  GetEstAngle();
  GetEstGyro();

  // GetWheelVel(&Enc_l, &Enc_r, &Enc_c, dt);
  
  // side inverted
  // if (mode == 0)
  // {
  //   ControlC();
  // }
  // point inverted
  // else
  // {
  //   ControlC();
  //   ControlL();
  //   ControlR();
  // }
  WheelBrake();
}
