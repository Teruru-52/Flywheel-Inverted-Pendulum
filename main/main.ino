#include "main.hpp"
#include "wheel.hpp"
#include "gyro.hpp"
#include "encoder.hpp"

#define GETUP_PIN 2
#define MODE_PIN 12

unsigned long nowTime, oldTime;
float dt;

int Mode = 0;
int state = 0;

Gyro gyro;
Wheels wheels;
WheelsController controller(1.0, 0.0, 0.0, 8.0, 15.0, 1.4, 8.0, 15.0, 1.4);
// Kpc, Kdc, Kwc, Kpl, Kdl, Kwl, Kpr, Kdr, Kwr

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(GETUP_PIN, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT_PULLUP);
  // attachInterrupt(MODE_PIN, ModeOnOff, FALLING);

  wheels.SetUpWheel();
  wheels.SetUpEncoder();

  gyro.GyroInit();
  gyro.OffsetCalc();
  gyro.KalmanInit();

  // DispInit();
}


void loop() {
  nowTime = micros();
  dt = (float)(nowTime - oldTime) / 1000000.0; // sec
  oldTime = nowTime;

  gyro.GetRawAngle();
  gyro.GetRawGyro();
  float y = gyro.GetEstAngle(dt);
  float dot_y = gyro.GetEstGyro();
  float omega = wheels.GetWheelVel(dt);

  controller.Control_1d(y, dot_y, omega);
  Serial.print(y);
  Serial.print(",");
  Serial.println(dt);
  
  // side inverted
  // if (mode == 0)
  // {
  //   controller.Control_1d(dt);
  // }
  // point inverted
  // else
  // {
  //   controller.Control_3d(dt);
  // }
  
  wheels.WheelBrake(y);
}
