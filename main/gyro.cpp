#include "gyro.hpp"

extern float dt;

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

    theta_x = atan2(-1.0 * (accY - accYoffset), (accZ - accZoffset)) * 180.0 / PI;
    theta_y = atan2(-1.0 * (accX - accXoffset), (accZ - accZoffset)) * 180.0 / PI;
}

void Gyro::GetRawGyro()
{
    mpu.getRotation(&gx, &gy, &gz);
    gyroX = gx / 131.072;
    gyroY = gy / 131.072;
    gyroZ = gz / 131.072;

    dot_theta_x = gyroZ - gyroZoffset;
    dot_theta_y = gyroY - gyroYoffset;
}

void Gyro::KalmanInit()
{
  // get theta
//  GetAngleRaw();
  // set initial angle
  kalmanX.setAngle(theta_x);
  kalmanY.setAngle(theta_y);
}

float Gyro::GetEstAngle()
{
  theta_x_est = kalmanX.getAngle(theta_x, dot_theta_x, dt);
  theta_y_est = kalmanY.getAngle(theta_y, dot_theta_y, dt);

  return theta_y_est;
}

float Gyro::GetEstGyro()
{
  dot_theta_x_est = kalmanX.getRate();
  dot_theta_y_est = kalmanY.getRate();

  return dot_theta_y_est;
}
