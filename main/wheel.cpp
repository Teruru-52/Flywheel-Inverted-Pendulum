#include "wheel.hpp"
#include "Arduino.h"

Wheel::Wheel(uint8_t enc_a, uint8_t enc_b, uint8_t brake, uint8_t dir, uint8_t chan, uint8_t pwm_pin)
    : enc(enc_a, enc_b),
      brake(brake),
      dir(dir),
      chan(chan),
      pwm_pin(pwm_pin)
{
  pinMode(brake, OUTPUT);
  pinMode(dir, OUTPUT);
  // ledcSetup(uint8_t chan, double freq, uint8_t bit_num);
  // PWM 20kHz, 8bit
  ledcSetup(chan, 20000, 10);
  ledcAttachPin(pwm_pin, chan);
  ledcWrite(chan, 1024);
}

void Wheel::ReadEncoder()
{
  enc.ReadEncoder();
}

float Wheel::GetVelocity(float dt)
{
  vel = enc.count * 2.0 * M_PI / ppr / dt; // 2×π/100=0.0628 [rad]
  enc.count = 0;

  return vel;
}

void Wheel::Drive(float input)
{
  if (input > 0)
  {
    pwm = map((long)(input * map_coeff), 0, map_coeff, 0, max_input);
    digitalWrite(dir, HIGH);
  }
  else
  {
    pwm = map((long)(-input * map_coeff), 0, map_coeff, 0, max_input);
    digitalWrite(dir, LOW);
  }
  ledcWrite(chan, max_input - pwm);
}

void Wheel::TestDrive(long input)
{
  if (input > 0)
  {
    pwm = max_input - input;
    digitalWrite(dir, HIGH);
  }
  else
  {
    pwm = max_input + input;
    digitalWrite(dir, LOW);
  }
  ledcWrite(chan, pwm);
}

void Wheel::BrakeOff()
{
  digitalWrite(brake, HIGH);
}

void Wheel::BrakeOn()
{
  digitalWrite(brake, LOW);
}

WheelsController::WheelsController(float kp, float ki, float kd)
    : wheel_c(ENCC_A, ENCC_B, BRAKE_C, DIR_C, CHANNEL_C, PWM_PIN_C),
      wheel_l(ENCL_A, ENCL_B, BRAKE_L, DIR_L, CHANNEL_L, PWM_PIN_L),
      wheel_r(ENCR_A, ENCR_B, BRAKE_R, DIR_R, CHANNEL_R, PWM_PIN_R),
      flag_control(false),
      kp(kp),
      ki(ki),
      kd(kd),
      error_X(0),
      error_Y(0),
      input_sum_X(0),
      input_sum_Y(0),
      input_sum_Z(0) {}

std::array<float, 3> WheelsController::GetWheelVelocity(float dt)
{
  wheel_vels[0] = wheel_c.GetVelocity(dt);
  wheel_vels[1] = wheel_l.GetVelocity(dt);
  wheel_vels[2] = wheel_r.GetVelocity(dt);

  return wheel_vels;
}
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

      wheel_c.Drive(input_C);
      wheel_l.Drive(input_L);
      wheel_r.Drive(input_R);
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
    if (ki > 10)
      ki = 0;
  }
  if (tuning_mode == 2)
  {
    kd += 0.001;
    if (kd > 1.0)
      kd = 0;
  }
}

void WheelsController::WheelBrakeOn()
{
  Serial.println("Wheel Brake On");
  wheel_c.BrakeOn();
  wheel_l.BrakeOn();
  wheel_r.BrakeOn();
}

void WheelsController::WheelBrakeOff()
{
  Serial.println("Wheel Brake Off");
  wheel_c.BrakeOff();
  wheel_l.BrakeOff();
  wheel_r.BrakeOff();
}

void WheelsController::TestDrive()
{
  WheelBrakeOff();
  wheel_c.TestDrive(40);
  wheel_l.TestDrive(40);
  wheel_r.TestDrive(40);
  delay(2000);
  WheelBrakeOn();
  delay(2000);
  WheelBrakeOff();
  wheel_c.TestDrive(-50);
  wheel_l.TestDrive(-50);
  wheel_r.TestDrive(-50);
  delay(2000);
  WheelBrakeOn();
  delay(2000);
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
