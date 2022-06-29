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
  ledcSetup(CHANNEL_L, 20000, 10);
  ledcAttachPin(PWM_PIN_L, CHANNEL_L);
  ledcSetup(CHANNEL_R, 20000, 10);
  ledcAttachPin(PWM_PIN_R, CHANNEL_R);
  ledcSetup(CHANNEL_C, 20000, 10);
  ledcAttachPin(PWM_PIN_C, CHANNEL_C);

  ledcWrite(CHANNEL_C, 1024);
  ledcWrite(CHANNEL_L, 1024);
  ledcWrite(CHANNEL_R, 1024);
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
    : flag_control(false),
      kp(kp),
      ki(ki),
      kd(kd),
      error_X(0),
      error_Y(0),
      input_sum_X(0),
      input_sum_Y(0),
      input_sum_Z(0) {}

// void WheelsController::Invert_side_C(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega)
// {
//   error_X = ref_theta_X1 - theta[0];
//   input_X = constrain(kp * error_X + ki * dot_theta[2] + kd * omega[0], -255, 255);

//   if (abs(error_X) <= limit_angle)
//     WheelDrive_C(input_X);
//   else
//     WheelDrive_C(0);
// }

// void WheelsController::Invert_side_L(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega)
// {
//   error_Y = ref_theta_Y1 - theta[1];
//   input_Y = constrain(kp * error_Y + ki * dot_theta[1] + kd * omega[1], -255, 255);

//   if (abs(error_Y) <= limit_angle)
//     WheelDrive_L(input_Y);
//   else
//     WheelDrive_L(0);
// }

// void WheelsController::Invert_side_R(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega)
// {
//   error_Y = ref_theta_Y1 - theta[1];
//   input_Y = constrain(kp * error_Y + ki * dot_theta[1] + kd * omega[1], -255, 255);

//   if (abs(error_Y) <= limit_angle)
//     WheelDrive_R(-input_Y);
//   else
//     WheelDrive_R(0);
// }

void WheelsController::Invert_point(std::array<float, 3> theta, std::array<float, 3> dot_theta)
{
  if (abs(theta[0]) < start_angle && abs(theta[1]) < start_angle && flag_control == false)
  {
    flag_control = true;
    Serial.println("Start Point Invert");
    WheelBrakeOff();
  }

  if (flag_control == true)
  {
    if (abs(theta[0]) < limit_angle && abs(theta[1]) < limit_angle)
    {
      input_X = kp * theta[0] + ki * dot_theta[0] + kd * input_sum_X;
      input_Y = kp * theta[1] + ki * dot_theta[1] + kd * input_sum_Y;
      // input_Z = ki * dot_theta[2] + kd * input_sum_Z;
      input_Z = 0;

      input_sum_X += input_X;
      input_sum_Y += input_Y;
      input_sum_Z += input_Z;

      input_C = constrain(2.0 * coeff1 * input_X - coeff2 * input_Z, -limit_input, limit_input);
      input_L = constrain(-coeff1 * input_X - coeff3 * input_Y - coeff2 * input_Z, -limit_input, limit_input);
      input_R = constrain(-coeff1 * input_X + coeff3 * input_Y - coeff2 * input_Z, -limit_input, limit_input);

      WheelDrive_C(input_C);
      WheelDrive_L(input_L);
      WheelDrive_R(input_R);
    }
    else
    {
      flag_control = false;
      Serial.println("Stop Point Invert");
      WheelBrakeOn();
      ResetInput();
    }
  }
}

void WheelsController::WheelDrive_C(float input)
{
  if (input > 0)
  {
    pwm_C = map((long)(input * map_coeff), 0, map_coeff, 0, max_input);
    digitalWrite(ROT_DIR_C, HIGH);
  }
  else
  {
    pwm_C = map((long)(-input * map_coeff), 0, map_coeff, 0, max_input);
    digitalWrite(ROT_DIR_C, LOW);
  }
  ledcWrite(CHANNEL_C, max_input - pwm_C);
  // Serial.print(pwm_C);
  // Serial.print(", ");
}

void WheelsController::WheelDrive_L(float input)
{
  if (input > 0)
  {
    pwm_L = map((long)(input * map_coeff), 0, map_coeff, 0, max_input);
    digitalWrite(ROT_DIR_L, HIGH);
  }
  else
  {
    pwm_L = map((long)(-input * map_coeff), 0, map_coeff, 0, max_input);
    digitalWrite(ROT_DIR_L, LOW);
  }
  ledcWrite(CHANNEL_L, max_input - pwm_L);
  // Serial.print(pwm_L);
  // Serial.print(", ");
}

void WheelsController::WheelDrive_R(float input)
{
  if (input > 0)
  {
    pwm_R = map((long)(input * map_coeff), 0, map_coeff, 0, max_input);
    digitalWrite(ROT_DIR_R, HIGH);
  }
  else
  {
    pwm_R = map((long)(-input * map_coeff), 0, map_coeff, 0, max_input);
    digitalWrite(ROT_DIR_R, LOW);
  }
  ledcWrite(CHANNEL_R, max_input - pwm_R);
  // Serial.println(pwm_R);
}

void WheelsController::ResetInput()
{
  error_X = 0;
  error_Y = 0;
  input_X = 0;
  input_Y = 0;
  input_Z = 0;
  input_sum_X = 0;
  input_sum_Y = 0;
  input_sum_Z = 0;
  input_C = 0;
  input_L = 0;
  input_R = 0;
}

void WheelsController::PID_Tuning(int tuning_mode)
{
  if (tuning_mode == 0)
  {
    kp += 0.01;
    if (kp > 15)
      kp = 0;
  }
  if (tuning_mode == 1)
  {
    ki += 0.01;
    if (ki > 15)
      ki = 0;
  }
  if (tuning_mode == 2)
  {
    kd += 0.0001;
    if (kd > 10)
      kd = 0;
  }
}

void WheelsController::WheelBrakeOff()
{
  Serial.println("Wheel Brake Off");
  digitalWrite(BRAKE_C, HIGH);
  digitalWrite(BRAKE_L, HIGH);
  digitalWrite(BRAKE_R, HIGH);
}

void WheelsController::WheelBrakeOn()
{
  Serial.println("Wheel Brake On");
  digitalWrite(BRAKE_C, LOW);
  digitalWrite(BRAKE_L, LOW);
  digitalWrite(BRAKE_R, LOW);
}

void WheelsController::TestControl()
{
  WheelBrakeOff();
  WheelDrive_C(10);
  WheelDrive_L(10);
  WheelDrive_R(10);
  delay(2000);
  WheelBrakeOn();
  delay(2000);
  // WheelBrakeOff();
  // WheelDrive_C(-50);
  // WheelDrive_L(-50);
  // WheelDrive_R(-50);
  // delay(2000);
  // WheelBrakeOn();
  // delay(2000);
}

std::array<float, 3> WheelsController::GetInput()
{
  return {input_C, input_L, input_R};
  // return {input_X, input_Y, input_Z};
}

std::array<float, 3> WheelsController::GetGain()
{
  return {kp, ki, kd};
}
