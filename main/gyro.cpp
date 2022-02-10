#include "gyro.hpp"

void Gyro::GyroInit()
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

void Gyro::OffsetCalc()
{
  delay(700);

  for (int i = 0; i < 100; i++)
  {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accX = ax / 16384.0;
    accY = ay / 16384.0;
    accZ = az / 16384.0;
    gyroX = gx / 131.072;
    gyroY = gy / 131.072;
    gyroZ = gz / 131.072;

    delay(30);

    accXoffset += accX;
    accYoffset += accY;
    accZoffset += accZ;
    gyroXoffset += gyroX;
    gyroYoffset += gyroY;
    gyroZoffset += gyroZ;
  }

  if (accXoffset < 0)
  {
    accXoffset = accXoffset / 100 + 1.0 / sqrt(2.0);
  }
  else
  {
    accXoffset = accXoffset / 100 - 1.0 / sqrt(2.0);
  }
  accYoffset /= 10;
  accZoffset = accZoffset / 100 - 1.0 / sqrt(2.0);
  gyroXoffset /= 100;
  gyroYoffset /= 100;
  gyroZoffset /= 100;
}

void Gyro::GetRawAngle()
{
  mpu.getAcceleration(&ax, &ay, &az);
  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

//  theta_z = atan2(-1.0 * (accY - accYoffset), (accZ - accZoffset)); // [rad]
  theta[0] = atan2(-1.0 * (accX - accXoffset), (accZ - accZoffset)); // center
  theta[1] = atan2(accY - accYoffset , -(accX - accXoffset) * sin(M_PI / 4.0) + (accZ - accZoffset) * cos(M_PI / 4.0)) - M_PI / 4.0; // left
  theta[2] = atan2(accY - accYoffset , -(accX - accXoffset) * sin(-M_PI / 4.0) + (accZ - accZoffset) * cos(-M_PI / 4.0)) - M_PI / 4.0; //right
}

void Gyro::GetRawGyro()
{
  mpu.getRotation(&gx, &gy, &gz);
  gyroX = gx / 131.072;
  gyroY = gy / 131.072;
  gyroZ = gz / 131.072;

//  dot_theta_z = (gyroZ - gyroZoffset) * M_PI / 180; // [rad/s]
  dot_theta[0] = (gyroY - gyroYoffset) * M_PI / 180; // center
  dot_theta[1] = (gyroX - gyroXoffset + (gyroZ - gyroZoffset)) * M_PI / 180; // left
  dot_theta[2] = (gyroX - gyroXoffset - (gyroZ - gyroZoffset)) * M_PI / 180; // right
}

void Gyro::KalmanInit()
{
  // set initial angle
//  kalmanZ.setAngle(theta_z);
  kalmanC.setAngle(theta[0]);
  kalmanL.setAngle(theta[1]);
  kalmanR.setAngle(theta[2]);
}

std::array<float, 3>  Gyro::GetEstAngle(float dt)
{
//  theta_z_est = kalmanZ.getAngle(theta_z, dot_theta_z, dt);
  theta_est[0] = kalmanC.getAngle(theta[0], dot_theta[0], dt); // center
  theta_est[1] = kalmanL.getAngle(theta[1], dot_theta[1], dt); // left
  theta_est[2] = kalmanR.getAngle(theta[2], dot_theta[2], dt); // right

  return theta_est;
}

std::array<float, 3>  Gyro::GetEstGyro()
{
//  dot_theta_z_est = kalmanZ.getRate();
  dot_theta_est[0] = kalmanC.getRate(); // center
  dot_theta_est[1] = kalmanL.getRate(); // left
  dot_theta_est[2] = kalmanR.getRate(); // right

  return dot_theta_est;
}
