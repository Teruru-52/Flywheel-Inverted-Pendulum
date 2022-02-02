#include "main.hpp"
#include "wheel.hpp"

#define GETUP_PIN 2
#define MODE_PIN 12

unsigned long nowTime, oldTime;
float dt;

int mode = 0;
int state = 0;

float omegaZ = 0.0;

WheelsCotroller controller(2.5, 3.0, 0.1, 8.0, 15.0, 1.4, 8.0, 15.0, 1.4);
// Kpc, Kdc, Kwc, Kpl, Kdl, Kwl, Kpr, Kdr, Kwr

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
  nowTime = micros();
  dt = (float)(nowTime - oldTime) / 1000000.0; // sec
  oldTime = nowTime;

  GetSamplingTime();
  GetRawAngle();
  GetRawGyro();
  GetEstAngle();
  GetEstGyro();

  // GetWheelVel(&Enc_l, &Enc_r, &Enc_c, dt);
  
  // side inverted
  // if (mode == 0)
  // {
  //   controller.Control_1d();
  // }
  // point inverted
  // else
  // {
  //   controller.Control_3d();
  // }
  WheelBrake();
}
