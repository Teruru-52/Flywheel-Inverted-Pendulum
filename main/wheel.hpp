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

#define INPUT_LIMIT_L 360
#define INPUT_LIMIT_R 360
#define INPUT_LIMIT_C 360

class Wheels
{
private:
    Encoder enc_l;
    Encoder enc_r;
    Encoder enc_c;

    const float angle_limit = 20; //deg
    float wheel_vel;

public:
    Wheels();

    void SetUpWheel();
    void SetUpEncoder();
    float GetWheelVel();
    void WheelBrake();
};

class WheelsController
{
private:

    float Kpc, Kdc, Kwc;
    float Kpl, Kdl, Kwl;
    float Kpr, Kdr, Kwr;
    const float input_limit = 200;
    float MtL, MtR, MtC;
    int DutyIniL = 1020, DutyIniR = 1020, DutyIniC = 1020;
    int input_l, input_r, input_c;

public:
    WheelsController(float Kpc, float Kdc, float Kwc, float Kpl, float Kdl, float Kwl, float Kpr, float Kdr, float Kwr);

    void Control_1d();
    void Control_3d();
    void TestControl();
};
#endif // _WHEEL_HPP_