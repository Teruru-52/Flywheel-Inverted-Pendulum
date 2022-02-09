#include "main.hpp"
#include "wheel.hpp"
#include "gyro.hpp"
#include "encoder.hpp"
#include "display.hpp"

#define GETUP_PIN 2
#define MODE_PIN 12

unsigned long nowTime, oldTime;
float dt;

int Mode = 0;

std::array<float, 3> theta;
std::array<float, 3> dot_theta;
std::array<float, 3> omega;

Gyro gyro;
Wheels wheels;
WheelsController controller(2000, 100.0, 1.0, 8.0, 15.0, 1.4, 8.0, 15.0, 1.4);
// Kpc, Kdc, Kwc, Kpl, Kdl, Kwl, Kpr, Kdr, Kwr

//void IRAM_ATTR Interrupt() {
//  Control();
//}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(GETUP_PIN, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT_PULLUP);
  attachInterrupt(MODE_PIN, ModeOnOff, FALLING);

  DispInit();

  wheels.SetUpWheel();
  SetUpEncoder();

  gyro.GyroInit();
  gyro.OffsetCalc();
  gyro.KalmanInit();

  wheels.WheelBrakeOff();
  
//   timer interruption
//  hw_timer_t * timer = NULL;
//  timer = timerBegin(0, 0.8, true); //timer = 10us
//  timerAttachInterrupt(timer, &Interrupt, true);
//  timerAlarmWrite(timer, 100, true); // 10ms
//  timerAlarmEnable(timer);
}

void loop() {
  nowTime = micros();
  dt = (float)(nowTime - oldTime) / 1000000.0; // sec
  oldTime = nowTime;

  gyro.GetRawAngle();
  gyro.GetRawGyro();
  theta = gyro.GetEstAngle(dt);
  dot_theta = gyro.GetEstGyro();
  omega = wheels.GetWheelVel(dt);

//    Serial.println(omega[0]);
//    Serial.print(",");
  //  Serial.println(dt,4);

  // side inverted
  if (Mode == 0)
  {
    controller.Control_1d(theta[0], dot_theta[0], omega[0]);
  }

  wheels.WheelBrakeOn(theta[0]);
}

//void Control() {
//  gyro.GetRawAngle();
//  gyro.GetRawGyro();
//  y = gyro.GetEstAngle(0.01);
//  dot_y = gyro.GetEstGyro();
//  omega = wheels.GetWheelVel(0.01);
//  Serial.println(y);

  // side inverted
//  if (Mode == 0)
//  {
//    controller.Control_1d(y, dot_y, omega);
//  }
//  wheels.WheelBrakeOn(y);
//}

void SetUpEncoder()
{
  pinMode(ENCL_A, INPUT);
  pinMode(ENCL_B, INPUT);
  pinMode(ENCR_A, INPUT);
  pinMode(ENCR_B, INPUT);
  pinMode(ENCC_A, INPUT);
  pinMode(ENCC_B, INPUT);

  attachInterrupt(ENCL_A, ReadEncoderL, CHANGE);
  attachInterrupt(ENCL_B, ReadEncoderL, CHANGE);
  attachInterrupt(ENCR_A, ReadEncoderR, CHANGE);
  attachInterrupt(ENCR_B, ReadEncoderR, CHANGE);
  attachInterrupt(ENCC_A, ReadEncoderC, CHANGE);
  attachInterrupt(ENCC_B, ReadEncoderC, CHANGE);
}

void ReadEncoderL() {
  wheels.EncoderReadL();
}

void ReadEncoderR() {
  wheels.EncoderReadR();
}

void ReadEncoderC() {
  wheels.EncoderReadC();
}

void ModeOnOff() {
  Serial.println("ModeOnOff");
  if (Mode) {
    Mode = 0;
  } else {
    Mode = 1;
  }
}
