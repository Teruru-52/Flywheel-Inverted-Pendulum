//センサオフセット算出
void offset_cal(){
  delay(700);
  accXoffset = 0;
  accYoffset = 0;
  accZoffset = 0;
  gyroXoffset = 0;
  gyroYoffset = 0;
  gyroZoffset = 0;

  for(int i=0; i<10; i++) {
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

  if(accXoffset < 0){
    accXoffset = accXoffset / 10 + 1.0 / sqrt(2.0);
  }else{
    accXoffset = accXoffset / 10 - 1.0 / sqrt(2.0);
  }
  accYoffset /= 10;
  accZoffset = accZoffset / 10 - 1.0 / sqrt(2.0);
  gyroXoffset /= 10;
  gyroYoffset /= 10;
  gyroZoffset /= 10;
}

//加速度センサから傾きデータ取得 [deg]
void get_theta() {
  mpu.getAcceleration(&ax, &ay, &az);
  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;
  
  //傾斜角導出 単位はdeg
  theta_X  = atan2(-1.0 * (accY - accYoffset) , (accZ - accZoffset)) * 180.0/PI;
  theta_Y  = atan2(-1.0 * (accX - accXoffset) , (accZ - accZoffset)) * 180.0/PI;
  theta_L  = atan2(accY - accYoffset , -(accX - accXoffset) * sin(PI/4.0) + (accZ - accZoffset) * cos(PI/4.0)) * 180.0/PI;
  theta_R  = atan2(accY - accYoffset , -(accX - accXoffset) * sin(-PI/4.0) + (accZ - accZoffset) * cos(-PI/4.0)) * 180.0/PI;
}

//角速度取得
void get_gyro_data() {
  mpu.getRotation(&gx, &gy, &gz);
  gyroX = gx / 131.072;
  gyroY = gy / 131.072;
  gyroZ = gz / 131.072;

  theta_Ydot = gyroY - gyroYoffset;
  theta_Zdot = gyroZ - gyroZoffset;
  theta_Ldot = gyroX - gyroXoffset + (gyroZ - gyroZoffset); 
  theta_Rdot = gyroX - gyroXoffset - (gyroZ - gyroZoffset); 
}
