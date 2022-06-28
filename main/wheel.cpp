#include "wheel.hpp"
#include "Arduino.h"

Wheels::Wheels()
    : enc_l(ENCL_A, ENCL_B),
      enc_r(ENCR_A, ENCR_B),
      enc_c(ENCC_A, ENCC_B) {}

void Wheels::SetUpWheel()
{
  pinMode(BRAKE_C, OUTPUT);
  pinMode(BRAKE_L, OUTPUT);
  pinMode(BRAKE_R, OUTPUT);
  pinMode(ROT_DIR_C, OUTPUT);
  pinMode(ROT_DIR_L, OUTPUT);
  pinMode(ROT_DIR_R, OUTPUT);

  digitalWrite(BRAKE_C, LOW);
  digitalWrite(BRAKE_L, LOW);
  digitalWrite(BRAKE_R, LOW);

  // ledcSetup(uint8_t chan, double freq, uint8_t bit_num);
  // PWM 20kHz, 8bit
  ledcSetup(CHANNEL_L, 20000, 8);
  ledcAttachPin(PWM_PIN_L, CHANNEL_L);
  ledcSetup(CHANNEL_R, 20000, 8);
  ledcAttachPin(PWM_PIN_R, CHANNEL_R);
  ledcSetup(CHANNEL_C, 20000, 8);
  ledcAttachPin(PWM_PIN_C, CHANNEL_C);

  ledcWrite(CHANNEL_C, 255);
  ledcWrite(CHANNEL_L, 255);
  ledcWrite(CHANNEL_R, 255);
}

// void Wheels::EncoderReadL()
// {
//   enc_l.EncoderRead();
// }

// void Wheels::EncoderReadR()
// {
//   enc_r.EncoderRead();
// }

// void Wheels::EncoderReadC()
// {
//   enc_c.EncoderRead();
// }

std::array<float, 3> Wheels::GetWheelVelocity(float dt)
{
  wheel_vel[0] = 1.0 * float(enc_c.count) * 2.0 * M_PI / ppr / dt; // 2×π/100=0.0628 [rad]
  enc_c.count = 0;
  wheel_vel[1] = 1.0 * float(enc_l.count) * 2.0 * M_PI / ppr / dt;
  enc_l.count = 0;
  wheel_vel[2] = 1.0 * float(enc_r.count) * 2.0 * M_PI / ppr / dt;
  enc_r.count = 0;

  return wheel_vel;
}

WheelsController::WheelsController(float kp, float ki, float kd)
    : kp(kp),
      ki(ki),
      kd(kd),
      error_X(0),
      error_Y(0),
      motor_speed_X(0),
      motor_speed_Y(0) {}

void WheelsController::Invert_side_C(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega)
{
  error_X = ref_theta_X1 - theta[0];
  pwm_X = constrain(kp * error_X + ki * dot_theta[2] + kd * omega[0], -255, 255);

  if (abs(error_X) <= angle_limit)
    WheelDrive_C(pwm_X);
  else
    WheelDrive_C(0);
}

void WheelsController::Invert_side_L(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega)
{
  error_Y = ref_theta_Y1 - theta[1];
  pwm_Y = constrain(kp * error_Y + ki * dot_theta[1] + kd * omega[1], -255, 255);

  if (abs(error_Y) <= angle_limit)
    WheelDrive_L(pwm_Y);
  else
    WheelDrive_L(0);
}

void WheelsController::Invert_side_R(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega)
{
  error_Y = ref_theta_Y1 - theta[1];
  pwm_Y = constrain(kp * error_Y + ki * dot_theta[1] + kd * omega[1], -255, 255);

  if (abs(error_Y) <= angle_limit)
    WheelDrive_R(-pwm_Y);
  else
    WheelDrive_R(0);
}

void WheelsController::Invert_point(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega)
{
  if (abs(theta[0]) < 0.174 && abs(theta[1]) < 0.174)
  {
    pwm_X = constrain(kp * theta[0] + ki * dot_theta[0] + kd * motor_speed_X, -255, 255);
    pwm_Y = constrain(kp * theta[1] + ki * dot_theta[1] + kd * motor_speed_Y, -255, 255);
    motor_speed_X += pwm_X;
    motor_speed_Y += pwm_Y;

    input_C = pwm_X;
    input_L = constrain(round(0.5 * pwm_X - 0.75 * pwm_Y), -255, 255);
    input_R = constrain(round(0.5 * pwm_X + 0.75 * pwm_Y), -255, 255);

    WheelDrive_C(input_C);
    //    WheelDrive_L(input_L);
    //    WheelDrive_R(input_R);
  }
  else
  {
    WheelBrakeOff();
    pwm_X = 0;
    pwm_Y = 0;
    motor_speed_X = 0;
    motor_speed_Y = 0;
  }
}

void WheelsController::WheelDrive_C(int input)
{
  if (input > 0)
  {
    input = 255 - input;
    digitalWrite(ROT_DIR_C, HIGH);
  }
  else
  {
    input = 255 + input;
    digitalWrite(ROT_DIR_C, LOW);
  }
  ledcWrite(CHANNEL_C, input);
  // Serial.print("input = ");
  // Serial.println(input);
}

void WheelsController::WheelDrive_L(int input)
{
  if (input > 0)
  {
    input = 255 - input;
    digitalWrite(ROT_DIR_L, HIGH);
  }
  else
  {
    input = 255 + input;
    digitalWrite(ROT_DIR_L, LOW);
  }
  ledcWrite(CHANNEL_L, input);
  // Serial.print("input = ");
  // Serial.println(input);
}

void WheelsController::WheelDrive_R(int input)
{
  if (input > 0)
  {
    input = 255 - input;
    digitalWrite(ROT_DIR_R, HIGH);
  }
  else
  {
    input = 255 + input;
    digitalWrite(ROT_DIR_R, LOW);
  }
  ledcWrite(CHANNEL_R, input);
  // Serial.print("input = ");
  // Serial.println(input);
}

void WheelsController::PID_Tuning(int tuning_mode)
{
  if (tuning_mode == 0)
  {
    kp += 1;
    if (kp > 500)
      kp = 0;
  }
  if (tuning_mode == 1)
  {
    ki += 1;
    if (ki > 100)
      ki = 0;
  }
  if (tuning_mode == 2)
  {
    kd += 0.001;
    if (kd > 5)
      kd = 0;
  }

  Serial.print(tuning_mode);
  Serial.print(",");
  Serial.print(kp);
  Serial.print(",");
  Serial.print(ki);
  Serial.print(",");
  Serial.println(kd);
}

void WheelsController::WheelBrakeOn()
{
  digitalWrite(BRAKE_C, HIGH);
  digitalWrite(BRAKE_L, HIGH);
  digitalWrite(BRAKE_R, HIGH);
}

void WheelsController::WheelBrakeOff()
{
  digitalWrite(BRAKE_C, LOW);
  digitalWrite(BRAKE_L, LOW);
  digitalWrite(BRAKE_R, LOW);
}

void WheelsController::TestControl()
{
  WheelBrakeOff();
  WheelDrive_C(50);
  WheelDrive_L(50);
  WheelDrive_R(50);
  delay(2000);
  WheelBrakeOn();
  delay(2000);
  WheelBrakeOff();
  WheelDrive_C(-50);
  WheelDrive_L(-50);
  WheelDrive_R(-50);
  delay(2000);
  WheelBrakeOn();
  delay(2000);
}

std::array<int16_t, 3> WheelsController::GetInput()
{
  return {input_C, input_L, input_R};
}

std::array<float, 3> WheelsController::GetGain()
{
  return {kp, ki, kd};
}
