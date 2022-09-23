#ifndef _WHEEL_HPP_
#define _WHEEL_HPP_
#include "main.hpp"
#include "encoder.hpp"
#include "imu.hpp"

#define BRAKE_L 19
#define BRAKE_R 17
#define BRAKE_C 18

#define DIR_L 23
#define DIR_R 16
#define DIR_C 5

#define CHANNEL_L 0
#define CHANNEL_R 1
#define CHANNEL_C 2

#define PWM_PIN_L 27
#define PWM_PIN_R 25
#define PWM_PIN_C 26

class Wheel
{
private:
  Encoder enc;
  float vel;
  const float ppr = 100.0;
  uint8_t brake;
  uint8_t dir;
  uint8_t chan;
  uint8_t pwm_pin;
  long max_input = 1024;
  long map_coeff = 1000;
  long pwm;

public:
  Wheel(uint8_t enc_a, uint8_t enc_b, uint8_t brake, uint8_t dir, uint8_t chan, uint8_t pwm_pin);

  void ReadEncoder();
  float GetVelocity(float dt);
  void Drive(float input);
  void TestDrive(long input);
  void BrakeOn();
  void BrakeOff();
};

class WheelsController
{
private:
  float kp;
  float ki;
  float kd;
  const float start_angle = 1.0 * M_PI / 180; // [rad]
  const float limit_angle = 5.0 * M_PI / 180; // [rad]
  bool flag_control;
  float limit_input = 1.0;

  std::array<float, 3> wheel_vels;
  float input_X;
  float input_Y;
  float input_Z;
  float error_X;
  float error_Y;
  float ref_theta_X1 = 0.0;
  float ref_theta_Y1 = 0.0;
  float ref_theta_X4 = 0.0;
  float ref_theta_Y4 = 0.0;
  float input_sum_X;
  float input_sum_Y;
  float input_sum_Z;
  float input_C;
  float input_L;
  float input_R;

  const float coeff1 = 0.4082;
  const float coeff2 = 0.5774;
  const float coeff3 = 0.7071;

public:
  Wheel wheel_c;
  Wheel wheel_l;
  Wheel wheel_r;
  WheelsController(float kp, float ki, float kd);

  std::array<float, 3> GetWheelVelocity(float dt);
  void Invert_side_C(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega);
  void Invert_side_L(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega);
  void Invert_side_R(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega);
  void Invert_point(std::array<float, 3> theta, std::array<float, 3> dot_theta);
  void ResetInput();
  void PID_Tuning(int tuning_mode);
  void WheelBrakeOn();
  void WheelBrakeOff();
  void TestDrive();
  std::array<float, 3> GetInput();
  std::array<float, 3> GetGain();
};
#endif // _WHEEL_HPP_
