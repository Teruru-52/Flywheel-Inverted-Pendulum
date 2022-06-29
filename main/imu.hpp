#ifndef _GYRO_HPP_
#define _GYRO_HPP_
#include "main.hpp"
#include "mpu6050.hpp"
#include "kalman.hpp"

class IMU
{
private:
    MPU6050 mpu;
    Kalman kalmanZ, kalmanC, kalmanL, kalmanR;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
    float accX = 0, accY = 0, accZ = 0;
    float gyroX = 0, gyroY = 0, gyroZ = 0;
    std::array<float, 3> theta;
    std::array<float, 3> theta_est;
    std::array<float, 3> dot_theta;
    std::array<float, 3> dot_theta_est;

public:
    void Init();
    void OffsetCalc();
    void GetRawAngle();
    void GetRawGyro();
    void KalmanInit();
    std::array<float, 3> GetEstAngle(float dt);
    std::array<float, 3> GetEstGyro();
};

#endif // _GYRO_HPP_
