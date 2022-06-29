#include "imu.hpp"

void IMU::Init()
{
  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  mpu.setXAccelOffset(-2549);
  mpu.setYAccelOffset(-1439);
  mpu.setZAccelOffset(1537);
  mpu.setXGyroOffset(-433);
  mpu.setYGyroOffset(3);
  mpu.setZGyroOffset(60);
}

void IMU::OffsetCalc()
{
  delay(700);

  for (int i = 0; i < 100; i++)
  {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyroX = gx / 131.072;
    gyroY = gy / 131.072;
    gyroZ = gz / 131.072;

    gyroXoffset += gyroX;
    gyroYoffset += gyroY;
    gyroZoffset += gyroZ;

    delay(10);
  }
  gyroXoffset /= 100.0;
  gyroYoffset /= 100.0;
  gyroZoffset /= 100.0;

  Serial.println("Finish Offset Calculation");
}

void IMU::GetRawAngle()
{
  mpu.getAcceleration(&ax, &ay, &az);
  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  theta[0] = -atan2(accY, sqrt(accX * accX + accZ * accZ)); // x
  theta[1] = atan2(accX, sqrt(accY * accY + accZ * accZ));  // y
}

void IMU::GetRawGyro()
{
  mpu.getRotation(&gx, &gy, &gz);
  gyroX = gx / 131.072 - gyroXoffset;
  gyroY = gy / 131.072 - gyroYoffset;
  gyroZ = gz / 131.072 - gyroZoffset;

  dot_theta[0] = -gyroX * M_PI / 180.0; // x
  dot_theta[1] = -gyroY * M_PI / 180.0; // y
  dot_theta[2] = gyroZ * M_PI / 180.0;  // z
}

void IMU::KalmanInit()
{
  GetRawAngle();
  // set initial angle
  kalmanC.setAngle(theta[0]);
  kalmanL.setAngle(theta[1]);
  //  kalmanR.setAngle(theta[2]);
  Serial.println("Finish Kalman Filter Initialization");
}

std::array<float, 3> IMU::GetEstAngle(float dt)
{
  GetRawAngle();
  GetRawGyro();

  theta_est[0] = kalmanC.getAngle(theta[0], dot_theta[0], dt); // x
  theta_est[1] = kalmanL.getAngle(theta[1], dot_theta[1], dt); // y
  //  theta_est[2] = kalmanR.getAngle(theta[2], dot_theta[2], dt);

  return theta_est;
}

std::array<float, 3> IMU::GetEstGyro()
{
  dot_theta_est[0] = kalmanC.getRate();
  dot_theta_est[1] = kalmanL.getRate();
  dot_theta_est[2] = dot_theta[2];
  //  dot_theta_est[2] = kalmanR.getRate();

  return dot_theta_est;
}
