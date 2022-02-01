#include "gyro.h"

MPU6050 mpu;
Kalman kalmanL, kalmanR, kalmanC;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

static float accXoffset = 0, accYoffset = 0, accZoffset = 0;
static float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;

float theta_X = 0, theta_Y = 0, theta_L = 0, theta_R = 0;
float theta_Ldot = 0, theta_Rdot = 0, theta_Ydot = 0, theta_Zdot = 0;
extern float kalAngleL, kalAngleL2, kalAngleR;
extern float kalAngleR2, kalAngleDotL, kalAngleDotR, kalAngleC, kalAngleDotC;

float AjC = 0.0, AjC2 = 0.0, AjL = 0.0, AjR = 0.0;

void GyroInit(){
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

void OffsetCalc()
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

void GetRawAngle()
{
  mpu.getAcceleration(&ax, &ay, &az);
  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  theta_X = atan2(-1.0 * (accY - accYoffset), (accZ - accZoffset)) * 180.0 / PI;
  theta_Y = atan2(-1.0 * (accX - accXoffset), (accZ - accZoffset)) * 180.0 / PI;
  theta_L = atan2(accY - accYoffset, -(accX - accXoffset) * sin(PI / 4.0) + (accZ - accZoffset) * cos(PI / 4.0)) * 180.0 / PI;
  theta_R = atan2(accY - accYoffset, -(accX - accXoffset) * sin(-PI / 4.0) + (accZ - accZoffset) * cos(-PI / 4.0)) * 180.0 / PI;
}

void GetRawGyro()
{
  mpu.getRotation(&gx, &gy, &gz);
  gyroX = gx / 131.072;
  gyroY = gy / 131.072;
  gyroZ = gz / 131.072;

  theta_Ydot = gyroY - gyroYoffset;
  theta_Zdot = gyroZ - gyroZoffset;
  theta_Ldot = gyroX - gyroXoffset + (gyroZ - gyroZoffset);
  theta_Rdot = gyroX - gyroXoffset - (gyroZ - gyroZoffset);
}

void KalmanInit(){
  // get theta
  GetAngleRaw();
  // set initial angle
  kalmanL.setAngle(theta_L);
  kalmanR.setAngle(theta_R);
  kalmanC.setAngle(theta_Y);
}

void GetEstAngle(){
  kalAngleL = kalmanL.getAngle(theta_L, theta_Ldot, dt);
  kalAngleL2 = kalAngleL - degL;
  kalAngleR = kalmanR.getAngle(theta_R, theta_Rdot, dt);
  kalAngleR2 = kalAngleR - degR;
  kalAngleC = kalmanC.getAngle(theta_Y, theta_Ydot, dt);
}

void GetEstGyro(){
  kalAngleDotL = kalmanL.getRate();
  kalAngleDotR = kalmanR.getRate();
  kalAngleDotC = kalmanC.getRate();
}

void WheelBrake(){
  if(fabs(kalAngleL2) > LIMIT_ANGLE)
    digitalWrite(BRAKE_L, LOW);
  if(fabs(kalAngleLC) > LIMIT_ANGLE)
    digitalWrite(BRAKE_C, LOW);
  if(fabs(kalAngleR2) > LIMIT_ANGLE)
    digitalWrite(BRAKE_R, LOW);
}
