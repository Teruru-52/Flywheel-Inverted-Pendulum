#include "main.hpp"
#include "wheel.hpp"
#include "gyro.hpp"
#include "display.hpp"

#define RESET_PIN 2
#define MODE_PIN 12

unsigned long nowTime, oldTime;
float dt;
int Mode = 4;
int tuning_mode = 0;
std::array<float, 3> theta;
std::array<float, 3> dot_theta;
std::array<float, 3> omega;
std::array<int16_t, 3> input;
std::array<float, 3> gain;

Gyro imu;
Wheels wheels;
WheelsController controller(10.0, 10.0, 0.001); // kp, ki, kd
void setup()
{
  Wire.begin();
  Serial.begin(115200);

  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT_PULLUP);
  attachInterrupt(RESET_PIN, Reset, FALLING);
//  attachInterrupt(MODE_PIN, ModeOnOff, FALLING);
  DispInit();
  wheels.SetUpWheel();
  SetUpEncoder();
  imu.GyroInit();
  imu.OffsetCalc();
  imu.KalmanInit();
}

void loop()
{
  nowTime = micros();
  dt = (float)(nowTime - oldTime) / 1000000.0; // [µs]to[s]
  oldTime = nowTime;

  theta = imu.GetEstAngle(dt);
  dot_theta = imu.GetEstGyro();
  omega = wheels.GetWheelVelocity(dt);
  input = controller.GetInput();
  gain = controller.GetGain();

  if (digitalRead(RESET_PIN) == LOW)
  {
    tuning_mode++;
    if (tuning_mode == 3)
      tuning_mode = 0;
  }
  if (digitalRead(MODE_PIN) == LOW)
  {
    controller.PID_tuning(tuning_mode);
  }
  controller.Invert_point(theta, dot_theta, omega);

//    Serial.print(theta[0]);
//    Serial.print(",");
//    Serial.println(theta[1]);
//    Serial.print(",");
//    Serial.println(omega[2]);
//   imu.ExecuteLogger();
//  controller.TestControl(Mode);

  // side inverted
//  if (Mode == 1)
//  {
//    controller.Invert_side_C(theta, dot_theta, omega);
//  }
//  else if (Mode == 2)
//  {
//    controller.Invert_side_L(theta, dot_theta, omega);
//  }
// else if (Mode == 3)
//  {
//    controller.Invert_side_R(theta, dot_theta, omega);
//  }
  
//  if (Mode == 4)
//  {
    // point inverted
//    controller.Invert_point(theta, dot_theta, omega);
//  }
}

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

void ReadEncoderC()
{
  wheels.EncoderReadC();
}

void ReadEncoderL()
{
  wheels.EncoderReadL();
}

void ReadEncoderR()
{
  wheels.EncoderReadR();
}

void Reset() {
  Mode = 0;
}
void ModeOnOff() {
  Serial.println("ModeOnOff");
  if (Mode == 0) {
    Mode = 1;
  }
  else if (Mode == 1) {
    Mode = 2;
  }
  else if (Mode == 2) {
    Mode = 3;
  }
  else if (Mode == 3) {
    Mode = 4;
  }
  else if (Mode == 4) {
    Mode = 0;
  }
}
