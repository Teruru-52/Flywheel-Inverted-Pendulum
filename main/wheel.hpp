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
    const float angle_limit = 20.0 * M_PI / 180; // [rad]
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
    void WheelBrakeOff(int Mode);
};

class WheelsController
{
private:

    float Kpc, Kdc, Kwc;
    float Kpl, Kdl, Kwl;
    float Kpr, Kdr, Kwr;
    const int input_limit = 800;
    const int input_offset = 43;
    int input_l, input_r, input_c;

public:
    WheelsController(float Kpc, float Kdc, float Kwc, float Kpl, float Kdl, float Kwl, float Kpr, float Kdr, float Kwr);

    void Control_1d(int Mode, std::array<float, 3>  theta, std::array<float, 3>  dot_theta, std::array<float, 3>  omega);
    void Control_3d();
    void TestControl();
};
#endif // _WHEEL_HPP_
