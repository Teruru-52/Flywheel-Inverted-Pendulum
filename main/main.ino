#include "main.hpp"
#include "wheel.hpp"
#include "gyro.hpp"
#include "encoder.hpp"
#include "display.hpp"

#define RESET_PIN 2
#define MODE_PIN 12

unsigned long nowTime, oldTime;
float dt;

int Mode = 1;
int tuning_mode = 0;

std::array<float, 3> theta;
std::array<float, 3> dot_theta;
std::array<float, 3> omega;
//float kp = 0;
//float ki = 0;
//float kd = 0;
float kp = 900;
float ki = 150.0;
float kd = 0.83;

Gyro gyro;
Wheels wheels;
//WheelsController controller(1800, 700.0, 3.0, 1000.0, 750.0, 0.0, 3.0, 5.0, 0.026);
WheelsController controller(900, 150.0, 0.83, 1330.0, 290.0, 0.73, 1220.0, 100.0, 0.49);
// Kpc, Kdc, Kwc, Kpl, Kdl, Kwl, Kpr, Kdr, Kwr

//void IRAM_ATTR Interrupt() {
//  Control();
//}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT_PULLUP);
  //  attachInterrupt(RESET_PIN, Reset, FALLING);
  //  attachInterrupt(MODE_PIN, ModeOnOff, FALLING);

  DispInit();

  wheels.SetUpWheel();
  SetUpEncoder();

  gyro.GyroInit();
  gyro.OffsetCalc();
  gyro.KalmanInit();

  //   timer interruption
  //  hw_timer_t * timer = NULL;
  //  timer = timerBegin(0, 0.8, true); //timer = 10us
  //  timerAttachInterrupt(timer, &Interrupt, true);
  //  timerAlarmWrite(timer, 100, true); // 10ms
  //  timerAlarmEnable(timer);
}

void loop() {
  nowTime = micros();
  dt = (float)(nowTime - oldTime) / 1000000.0; // [µs]to[s]
  oldTime = nowTime;

  gyro.GetRawAngle();
  gyro.GetRawGyro();
  theta = gyro.GetEstAngle(dt);
  dot_theta = gyro.GetEstGyro();
  omega = wheels.GetWheelVel(dt);

  if (digitalRead(RESET_PIN) == LOW) {
    tuning_mode++;
    if (tuning_mode == 3) tuning_mode = 0;
  }
  if (digitalRead(MODE_PIN) == LOW) {
    if (tuning_mode == 0) {
      kp += 10;
      if (kp > 5000) kp = 0;
    }
    if (tuning_mode == 1) {
      ki += 5;
      if (ki > 1000) ki = 0;
    }
    if (tuning_mode == 2) {
      kd += 0.01;
      if (kd > 5) kd = 0;
    }
  }
    Serial.print(tuning_mode);
    Serial.print(",");
    Serial.print(kp);
    Serial.print(",");
    Serial.print(ki);
    Serial.print(",");
    Serial.print(kd);

//  Serial.print(theta[1]);
//  Serial.print(",");
//  Serial.print(dot_theta[1]);
//  Serial.print(",");
//  Serial.print(omega[1]);

  // side inverted
  //  controller.AngleControl(Mode, theta, dot_theta, kp, ki, kd);
  controller.Control_1d(Mode, theta, dot_theta, omega, kp, ki, kd);

  // point inverted
  if (Mode == 4) {
    controller.Control_3d(theta, dot_theta, omega);
  }

  wheels.WheelBrakeOn(Mode, theta);
  //  Serial.print(omega[0]);
  //  Serial.print(",");
  //  Serial.println(dt * 1000);

  delay(6);
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
  pinMode(ENCC_A, INPUT);
  pinMode(ENCC_B, INPUT);
  pinMode(ENCL_A, INPUT);
  pinMode(ENCL_B, INPUT);
  pinMode(ENCR_A, INPUT);
  pinMode(ENCR_B, INPUT);

  attachInterrupt(ENCC_A, ReadEncoderC, CHANGE);
  attachInterrupt(ENCC_B, ReadEncoderC, CHANGE);
  attachInterrupt(ENCL_A, ReadEncoderL, CHANGE);
  attachInterrupt(ENCL_B, ReadEncoderL, CHANGE);
  attachInterrupt(ENCR_A, ReadEncoderR, CHANGE);
  attachInterrupt(ENCR_B, ReadEncoderR, CHANGE);
}

void ReadEncoderC() {
  wheels.EncoderReadC();
}

void ReadEncoderL() {
  wheels.EncoderReadL();
}

void ReadEncoderR() {
  wheels.EncoderReadR();
}
