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
    //    accX = ax / 16384.0;
    //    accY = ay / 16384.0;
    //    accZ = az / 16384.0;
    gyroX = gx / 131.072;
    gyroY = gy / 131.072;
    gyroZ = gz / 131.072;

    //    accXoffset += accX;
    //    accYoffset += accY;
    //    accZoffset += accZ;
    gyroXoffset += gyroX;
    gyroYoffset += gyroY;
    gyroZoffset += gyroZ;

    delay(10);
  }

  //  if (accXoffset < 0)
  //  {
  //    accXoffset = accXoffset / 100 + 1.0 / sqrt(2.0);
  //  }
  //  else
  //  {
  //    accXoffset = accXoffset / 100 - 1.0 / sqrt(2.0);
  //  }

  //  accXoffset /= 100.0;
  //  accYoffset /= 100.0;
  //  accZoffset /= 100.0;
  //  accZoffset = accZoffset / 100 - 1.0 / sqrt(2.0);
  gyroXoffset /= 100.0;
  gyroYoffset /= 100.0;
  gyroZoffset /= 100.0;
}

void Gyro::GetRawAngle()
{
  mpu.getAcceleration(&ax, &ay, &az);
  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  theta[0] = atan2(accY, sqrt(accX * accX + accZ * accZ)); // x
  theta[1] = atan2(accX, sqrt(accY * accY + accZ * accZ)); // y
  

  //  theta[0] = atan2(-1.0 * accX, accZ);                                        // center
  //  theta[1] = atan2(accY, -accX * sin(M_PI / 4.0) + accZ * cos(M_PI / 4.0));   // left
  //  theta[2] = atan2(accY, -accX * sin(-M_PI / 4.0) + accZ * cos(-M_PI / 4.0)); // right
}

void Gyro::GetRawGyro()
{
  mpu.getRotation(&gx, &gy, &gz);
  gyroX = gx / 131.072 - gyroXoffset;
  gyroY = gy / 131.072 - gyroYoffset;
  gyroZ = gz / 131.072 - gyroZoffset;

  dot_theta[0] = gyroX * M_PI / 180.0; // x
  dot_theta[1] = gyroY * M_PI / 180.0; // y
  dot_theta[2] = gyroZ * M_PI / 180.0; // z

  //  dot_theta[0] = gyroY * M_PI / 180.0;          // center
  //  dot_theta[1] = gyroX + gyroZ) * M_PI / 180.0; // left
  //  dot_theta[2] = gyroX - gyroZ) * M_PI / 180.0; // right
}

void Gyro::KalmanInit()
{
  // set initial angle
  kalmanC.setAngle(theta[0]);
  kalmanL.setAngle(theta[1]);
  //  kalmanR.setAngle(theta[2]);
}

std::array<float, 3> Gyro::GetEstAngle(float dt)
{
  GetRawAngle();
  GetRawGyro();

  theta_est[0] = kalmanC.getAngle(theta[0], dot_theta[0], dt); // x
  theta_est[1] = kalmanL.getAngle(theta[1], dot_theta[1], dt); // y
  //  theta_est[2] = kalmanR.getAngle(theta[2], dot_theta[2], dt);

  return theta_est;
}

std::array<float, 3> Gyro::GetEstGyro()
{
  dot_theta_est[0] = kalmanC.getRate();
  dot_theta_est[1] = kalmanL.getRate();
  dot_theta_est[2] = dot_theta[2];
  //  dot_theta_est[2] = kalmanR.getRate();

  return dot_theta_est;
}

void Gyro::ExecuteLogger() {
  Serial.print(theta[0]);
  Serial.print(",");
  Serial.println(dot_theta[0]);
}
