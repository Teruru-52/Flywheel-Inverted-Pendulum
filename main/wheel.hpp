#ifndef _WHEEL_HPP_
#define _WHEEL_HPP_
#include "main.hpp"
#include "encoder.hpp"
#include "gyro.hpp"

#define CHANNEL_L 0
#define CHANNEL_R 1
#define CHANNEL_C 2

#define ROT_DIR_L 23
#define ROT_DIR_R 16
#define ROT_DIR_C 5

#define PWM_PIN_L 27
#define PWM_PIN_R 25
#define PWM_PIN_C 26

#define BRAKE_L 19
#define BRAKE_R 17
#define BRAKE_C 18

class Wheels
{
  private:
    Encoder enc_l;
    Encoder enc_r;
    Encoder enc_c;
    const float angle_limit = 5.0 * M_PI / 180; // [rad]
    std::array<float, 3> wheel_vel;
    const float ppr = 100.0;

  public:
    Wheels();

    void SetUpWheel();
    void EncoderReadL();
    void EncoderReadR();
    void EncoderReadC();
    std::array<float, 3> GetWheelVelocity(float dt);
};

class WheelsController
{
  private:
    float kp;
    float ki;
    float kd;
    const float angle_limit = 5.0 * M_PI / 180; // [rad]
    const float start_angle = 0.1 * M_PI / 180; // [rad]

    int pwm_Y;
    int pwm_X;
    float error_X;
    float error_Y;
    float ref_theta_X1 = 0.0;
    float ref_theta_Y1 = 0.0;
    float ref_theta_X4 = 0.0;
    float ref_theta_Y4 = 0.0;
    float motor_speed_X;
    float motor_speed_Y;
    int16_t input_C;
    int16_t input_L;
    int16_t input_R;

  public:
    WheelsController(float kp, float ki, float kd);

    void Invert_side_C(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega);
    void Invert_side_L(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega);
    void Invert_side_R(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega);
    void Invert_point(std::array<float, 3> theta, std::array<float, 3> dot_theta, std::array<float, 3> omega);
    void WheelDrive_C(int input);
    void WheelDrive_L(int input);
    void WheelDrive_R(int input);
    void PID_tuning(int tuning_mode);
    void WheelBrakeOn();
    void WheelBrakeOff();
    void TestControl(int Mode);
    std::array<int16_t, 3> GetInput();
    std::array<float, 3> GetGain();
};
#endif // _WHEEL_HPP_
