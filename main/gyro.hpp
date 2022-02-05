#ifndef _GYRO_HPP_
#define _GYRO_HPP_
#include "main.hpp"
#include "mpu6050.hpp"
#include "kalman.hpp"

class Gyro
{
private:
    MPU6050 mpu;
    Kalman kalmanX, kalmanY;
    
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    float accXoffset = 0, accYoffset = 0, accZoffset = 0;
    float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
    float accX = 0, accY = 0, accZ = 0;
    float gyroX = 0, gyroY = 0, gyroZ = 0;
    float theta_x, theta_y;
    float dot_theta_x, dot_theta_y;
    float theta_x_est, theta_y_est;
    float dot_theta_x_est, dot_theta_y_est;

public:
    void GyroInit();
    void OffsetCalc();
    void GetRawAngle();
    void GetRawGyro();
    void KalmanInit();
    float GetEstAngle(float dt);
    float GetEstGyro();
};

#endif  // _GYRO_HPP_
