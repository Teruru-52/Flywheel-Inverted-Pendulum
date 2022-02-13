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
    const float angle_limit = 10.0 * M_PI / 180; // [rad]
    std::array<float, 3> wheel_vel;

  public:
    Wheels();

    void SetUpWheel();
    //    void SetUpEncoder();
    void EncoderReadL();
    void EncoderReadR();
    void EncoderReadC();
    std::array<float, 3> GetWheelVel(float dt);
    void WheelBrakeOn(int Mode, std::array<float, 3> theta);
};

class WheelsController
{
  private:

    float Kpc, Kdc, Kwc;
    float Kpl, Kdl, Kwl;
    float Kpr, Kdr, Kwr;
    const int input_limit = 1023;
    //    const int input_offset = 43;
    float sum_error;
    float pre_error;
    float sum_error2;
    float pre_error2;
    float pre_input;
    const float angle_limit = 10.0 * M_PI / 180; // [rad]
    //    float Kp = 10.8;
    //    float Ki = 150.0;
    //    float Kp = 0.021;
    //    float Ki = 4.01;
    float Kp = 33.7;
    float Ki = 205.0;

    float kp = 4000.0;
    float kd = 150.0;
    float ki = 1000.0;
    float kp2 = 2000.0;
    float kd2 = 0.0;
    float ki2 = 700.0;

  public:
    WheelsController(float Kpc, float Kdc, float Kwc, float Kpl, float Kdl, float Kwl, float Kpr, float Kdr, float Kwr);

    void DirControl(int Mode, float input, int dir);
    void WheelDrive(int Mode, float input);
    void Control_1d(int Mode, std::array<float, 3>  theta, std::array<float, 3>  dot_theta, std::array<float, 3>  omega);
    void Control_3d(std::array<float, 3>  theta, std::array<float, 3>  dot_theta, std::array<float, 3>  omega);
    void VelocityControl(int Mode, std::array<float, 3>  theta, std::array<float, 3>  dot_theta, std::array<float, 3>  omega);
    void AngleControl(int Mode, std::array<float, 3>  theta, std::array<float, 3>  dot_theta);
    void TestControl(int Mode);
};
#endif // _WHEEL_HPP_
