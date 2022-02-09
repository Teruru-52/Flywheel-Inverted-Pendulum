#ifndef _GYRO_HPP_
#define _GYRO_HPP_
#include "main.hpp"
#include "mpu6050.hpp"
#include "kalman.hpp"

class Gyro
{
private:
    MPU6050 mpu;
    Kalman kalmanZ, kalmanC, kalmanL, kalmanR;
    
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    float accXoffset = 0, accYoffset = 0, accZoffset = 0;
    float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
    float accX = 0, accY = 0, accZ = 0;
    float gyroX = 0, gyroY = 0, gyroZ = 0;
    float theta_z;
    float theta_z_est;
    float dot_theta_z;
    float dot_theta_z_est;
    std::array<float, 3> theta;
    std::array<float, 3> theta_est;
    std::array<float, 3> dot_theta;
    std::array<float, 3> dot_theta_est;

public:
    void GyroInit();
    void OffsetCalc();
    void GetRawAngle();
    void GetRawGyro();
    void KalmanInit();
    std::array<float, 3> GetEstAngle(float dt);
    std::array<float, 3>  GetEstGyro();
};

#endif  // _GYRO_HPP_
