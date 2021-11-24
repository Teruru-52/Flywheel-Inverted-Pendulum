#include <Kalman.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define GetUpBt 2
#define ModeBt 12

#define ENCL_A 32
#define ENCL_B 33
#define PWM_pinL 27
#define brakeL 19
#define rote_pinL 23
#define servoL 4

#define ENCR_A 39
#define ENCR_B 36
#define PWM_pinR 25
#define brakeR 17
#define rote_pinR 16
#define servoR 13

#define ENCC_A 35
#define ENCC_B 34
#define PWM_pinC 26
#define brakeC 18
#define rote_pinC 5
#define servoC 14

#define CH_L 0
#define CH_R 1
#define CH_C 2
#define CH_ServoL 13
#define CH_ServoR 14
#define CH_ServoC 15


unsigned long oldTime = 0, loopTime, nowTime;
float dt;

volatile byte posL, posR, posC;
volatile int  enc_countL = 0, enc_countR = 0, enc_countC = 0;
float degL = 45, degR = 45;

int Mode = 0;
int GetUpBtState = 0;

int DutyIniL = 1020, pwmDutyL, DutyIniR = DutyIniL, pwmDutyR, DutyIniC = DutyIniL, pwmDutyC;
int GetUpX = 0, GetUpY = 0;

int Lok = 0, Rok = 0, Cok = 0;
int GetUpcntL = 0, GetUpcntR = 0, GetUpcntC = 0;

float MtL, MtR, MtC;
float omegaZ = 0.0;
float AjC = 0.0, AjC2 = 0.0, AjL = 0.0, AjR = 0.0;

//----------------------------
//調整パラメータ
//default
/*float KpLR = 8.0;
float KdLR = 15.0;
float KwLR = 1.4;*/
float KpLR = 8.0;
float KdLR = 15.0;
float KwLR = 1.4;

//default
/*float KpC = 12.0;
float KdC = 22.0;
float KwC = 1.4;*/
//これで一応倒立
/*float KpC = 15.0;
float KdC = 20.0;
float KwC = 1.5;*/
float KpC = 17.0;
float KdC = 23.0;
float KwC = 1.4;


float KC = 9.0, KC2 = 5.0, KLR = 12.0;

int servoIniL = 4600, servoIniR = 4600, servoIniC = 5000;
int servoBrakeLR = 1500 , servoBrakeC = 2500;
int rotMaxLR = 360, rotMaxCL = 70, rotMaxCR = 85;

float getUpDeg = -5.0;

int loopDelay = 5;
//----------------------------


MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float temp = 0;

float theta_L = 0.0, theta_R = 0.0, theta_X = 0.0, theta_Y = 0.0;
float theta_Ldot = 0.0, theta_Rdot = 0.0, theta_Ydot = 0.0, theta_Zdot = 0.0;

//オフセット
float accXoffset = 0, accYoffset = 0, accZoffset = 0;
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;

Kalman kalmanL, kalmanR, kalmanC;
float kalAngleL, kalAngleL2, kalAngleR, kalAngleR2, kalAngleDotL, kalAngleDotR, kalAngleC, kalAngleDotC;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);


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


//起き上がりY
void getupY(){
  digitalWrite(brakeC, HIGH);
  int rotMax;
  GetUpY = 1;
  
  //回転方向
  if(kalAngleC < 0.0){
    rotMax = rotMaxCR;
    digitalWrite(rote_pinC, HIGH);
    getUpDeg = -5.0;
    Cok = 8;
  }else{
    rotMax = rotMaxCL;
    digitalWrite(rote_pinC, LOW);
    getUpDeg = 5.0;
    Cok = 9;
  }

  for(int i = 1023; i >= rotMax; i--){
    ledcWrite(CH_C, i);
    delay(5);
  }
  ledcWrite(CH_C, rotMax);
  delay(300);

  ledcWrite(CH_C, 1023);
  if(kalAngleC < 0.0){
    ledcWrite(CH_ServoC, servoIniC + servoBrakeC); //servo brake
  }else{
    ledcWrite(CH_ServoC, servoIniC - servoBrakeC); //servo brake
  }
  delay(160);
  ledcWrite(CH_ServoC, servoIniC);
  delay(240);
}

//Core0
void disp(void *pvParameters) {
  Wire1.begin(0, 15); //SDA,SCL
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Clear the buffer
  display.clearDisplay();
  
  for (;;){
    disableCore0WDT();
    
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    
    display.setCursor(0,0);            
    display.println(kalAngleL + AjL, 1);
    display.setCursor(50,0);            
    display.println(kalAngleC + AjC, 1);
    display.setCursor(90,0);            
    display.println(kalAngleR + AjR, 1);
    
    display.setCursor(0,9);  
    if(Mode){
      display.println("Point inverted");          
    }else{
      display.println("Side inverted");
    }

    int LineC = map(kalAngleC + AjC, 20, -20, 0, 127);
    if(LineC >= 0 && LineC < 128){
      if(abs(kalAngleC) <= 1.0){
        display.fillRect(LineC-2, 16, 5, display.height()-1, SSD1306_WHITE); //(左上x, 左上y, 幅, 高さ, 線の色
      }else{
        display.drawLine(LineC, 16, LineC, display.height()-1, SSD1306_WHITE); //始点x, 始点y, 終点x, 終点y, 線の色
      }
    }
    
    int LineL = map(kalAngleL + AjL, 45+8, 45-8, 16, 63);
    if(LineL >= 16 && LineL < 64){
      if(abs(kalAngleL2) <= 1.0){
        display.fillRect(0, LineL - 2 , display.width()/2-1, 5, SSD1306_WHITE); 
      }else{
        display.drawLine(0, LineL, display.width()/2-1, LineL, SSD1306_WHITE);
      }
    }
    
    int LineR = map(kalAngleR + AjR, 45+8, 45-8, 16, 63);
    if(LineR >= 16 && LineR < 64){
      if(abs(kalAngleR2) <= 1.0){
        display.fillRect(display.width()/2, LineR - 2, display.width()-1, 5, SSD1306_WHITE); //(左上x, 左上y, 幅, 高さ, 線の色
      }else{
        display.drawLine(display.width()/2, LineR, display.width()-1, LineR, SSD1306_WHITE);
      }
    }
      
    display.display(); 
    if(Lok != 9 && Rok != 9) delay(30);
    display.clearDisplay();

    
    //起き上がりX
    if(GetUpX == 1){
      digitalWrite(brakeL, HIGH);
      digitalWrite(brakeR, HIGH);
      
      //回転方向
      digitalWrite(rote_pinL, HIGH);
      digitalWrite(rote_pinR, LOW);
      
    
      for(int i = 1023; i >= rotMaxLR; i--){
        ledcWrite(CH_L, i);
        ledcWrite(CH_R, i);
        delay(5);
      }
      ledcWrite(CH_L, rotMaxLR);
      ledcWrite(CH_R, rotMaxLR);
      delay(300);
    
      ledcWrite(CH_L, 1023);
      ledcWrite(CH_R, 1023);
      ledcWrite(CH_ServoL, servoIniL + servoBrakeLR); //servo brake
      ledcWrite(CH_ServoR, servoIniR - servoBrakeLR);
      delay(160);
      ledcWrite(CH_ServoL, servoIniL);
      ledcWrite(CH_ServoR, servoIniR);

      Lok = 9;
      Rok = 9;
      
      Mode = 1;
      GetUpX = 0;
    }

    //起き上がりX軸角度計測
    if((Lok == 9 || Rok == 9) && dt < 1.0){
      if(kalAngleL2 > getUpDeg || kalAngleR2 > getUpDeg){
        Lok = 2;
        Rok = 2;
      }
    }
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(GetUpBt, INPUT_PULLUP);
  pinMode(ModeBt, INPUT_PULLUP);

  pinMode(ENCL_A, INPUT);
  pinMode(ENCL_B, INPUT);
  pinMode(ENCR_A, INPUT);
  pinMode(ENCR_B, INPUT); 
  pinMode(ENCC_A, INPUT);
  pinMode(ENCC_B, INPUT);

  pinMode(brakeL, OUTPUT);
  pinMode(brakeR, OUTPUT);
  pinMode(brakeC, OUTPUT);
  pinMode(rote_pinL, OUTPUT);
  pinMode(rote_pinR, OUTPUT);
  pinMode(rote_pinC, OUTPUT);

  digitalWrite(brakeL, LOW);
  digitalWrite(brakeR, LOW);
  digitalWrite(brakeC, LOW);

  attachInterrupt(GetUpBt, GetUp, FALLING);
  attachInterrupt(ModeBt, ModeOnOff, FALLING);
  
  attachInterrupt(ENCL_A, ENCL_READ, CHANGE);
  attachInterrupt(ENCL_B, ENCL_READ, CHANGE);
  attachInterrupt(ENCR_A, ENCR_READ, CHANGE);
  attachInterrupt(ENCR_B, ENCR_READ, CHANGE);  
  attachInterrupt(ENCC_A, ENCC_READ, CHANGE);
  attachInterrupt(ENCC_B, ENCC_READ, CHANGE);

  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //MPU6050 offset
  mpu.setXAccelOffset(-2549);
  mpu.setYAccelOffset(-1439);
  mpu.setZAccelOffset(1537);
  mpu.setXGyroOffset(-433);
  mpu.setYGyroOffset(3);
  mpu.setZGyroOffset(60);
  
  //センサオフセット算出
  offset_cal();

  get_theta();
  kalmanL.setAngle(theta_L);
  kalmanR.setAngle(theta_R);
  kalmanC.setAngle(theta_Y);

  ledcSetup(CH_L, 20000, 10);
  ledcAttachPin(PWM_pinL, CH_L);
  ledcSetup(CH_R, 20000, 10);
  ledcAttachPin(PWM_pinR, CH_R);
  ledcSetup(CH_C, 20000, 10);
  ledcAttachPin(PWM_pinC, CH_C);

  ledcSetup(CH_ServoL, 50, 16);
  ledcAttachPin(servoL, CH_ServoL);
  ledcWrite(CH_ServoL, servoIniL);
  ledcSetup(CH_ServoR, 50, 16);
  ledcAttachPin(servoR, CH_ServoR);
  ledcWrite(CH_ServoR, servoIniR);
  ledcSetup(CH_ServoC, 50, 16);
  ledcAttachPin(servoC, CH_ServoC);
  ledcWrite(CH_ServoC, servoIniC);
  delay(500); 

  //ディスプレイ表示 タスク
  xTaskCreatePinnedToCore(
    disp
    ,  "disp"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);
}


void loop() {
  nowTime = micros();
  loopTime = nowTime - oldTime;
  oldTime = nowTime;
  
  dt = (float)loopTime / 1000000.0; //sec
  
  //モータの角速度算出
  float theta_LdotWheel = -1.0 * float(enc_countL) * 3.6 / dt; //2×180°/100=3.6
  enc_countL = 0;
  float theta_RdotWheel = 1.0 * float(enc_countR) * 3.6 / dt;
  enc_countR = 0;
  float theta_YdotWheel = -1.0 * float(enc_countC) * 3.6 / dt;
  enc_countC = 0;

  get_theta();
  get_gyro_data();
    
  //カルマンフィルタ 姿勢 傾き
  kalAngleL = kalmanL.getAngle(theta_L, theta_Ldot, dt);
  kalAngleL2 = kalAngleL - degL;
  kalAngleR = kalmanR.getAngle(theta_R, theta_Rdot, dt);
  kalAngleR2 = kalAngleR - degR;
  kalAngleC = kalmanC.getAngle(theta_Y, theta_Ydot, dt);
  
  //カルマンフィルタ 姿勢 角速度
  kalAngleDotL = kalmanL.getRate();
  kalAngleDotR = kalmanR.getRate();
  kalAngleDotC = kalmanC.getRate();

  //Omega Z
  omegaZ = theta_Ydot * sin(theta_X * PI/180.0) / cos(theta_Y * PI/180.0) + theta_Zdot * cos(theta_X * PI/180.0) / cos(theta_Y * PI/180.0);

  //ブレーキ
  if(Lok == 3 && fabs(kalAngleL2) > 20.0){
    digitalWrite(brakeL, LOW);
    if(Cok != 0){
      digitalWrite(brakeC, LOW);
      Cok = 0;
      AjC = 0.0;
      AjC2 = 0.0; 
    }
    Lok = 0;
    AjL = 0.0;
  }
  if(Rok == 3 && fabs(kalAngleR2) > 20.0){
    digitalWrite(brakeR, LOW);
    if(Cok != 0){
      digitalWrite(brakeC, LOW);
      Cok = 0;
      AjC = 0.0;
      AjC2 = 0.0; 
    }
    Rok = 0;
    AjR= 0.0;
  }
  if(Cok == 2 && fabs(kalAngleC) > 20.0){
    digitalWrite(brakeC, LOW);
    Cok = 0;
    AjC = 0.0;
    AjC2 = 0.0; 
  }

  //起き上がりY軸角度計測
  if(Cok == 8 && dt < 1.0){
    if(kalAngleC > getUpDeg){
      Cok = 2;
    }
  }
  if(Cok == 9 && dt < 1.0){
    if(kalAngleC < getUpDeg){
      Cok = 2;
    }
  }

  //GETUPボタンプッシュ時の処理
  if(GetUpBtState){
    GetUpBtState = 0;
    
    if(Cok == 0 && (fabs(kalAngleC + AjC) > 40.0)){
      AjC = 0.0;
      AjC2 = 0.0; 
      AjL = 0.0;
      AjR = 0.0;
      offset_cal();
      Mode = 0;
      getupY();
    }
    
    if(Cok == 2 && (fabs(kalAngleC + AjC) < 2.0)){
      AjL = 0.0;
      AjR = 0.0;
      GetUpX = 1;
    }
  }
          
  //モータ駆動
  if (Lok == 0 && fabs(kalAngleL2) < 1.0 && kalAngleC > -30.0){
    Lok = 1;
    digitalWrite(brakeL, HIGH); //ブレーキ解除
  }
  
  if (Rok == 0 && fabs(kalAngleR2) < 1.0 && kalAngleC < 30.0){
    Rok = 1;
    digitalWrite(brakeR, HIGH); //ブレーキ解除
  }
  
  if(Mode == 1){
    if (Lok != 0 && Rok != 0 && Cok == 0 && fabs(kalAngleC) < 1.0){
      Cok = 1;
      digitalWrite(brakeC, HIGH); //ブレーキ解除
    }
  }else{
    if (Cok == 0 && fabs(kalAngleC) < 1.0){
      Cok = 1;
      digitalWrite(brakeC, HIGH); //ブレーキ解除
    }
  }

  if (Lok == 1){
    MtL = KpLR* kalAngleL2 / 90.0  + KdLR * kalAngleDotL / 1000.0 + KwLR * theta_LdotWheel / 20000.0;
    GetUpcntL++;
    if(GetUpcntL > 20){
      GetUpcntL = 0;
      Lok = 2;
    }
  }
  if (Rok == 1){
    MtR = KpLR * kalAngleR2 / 90.0  + KdLR * kalAngleDotR / 1000.0 + KwLR * theta_RdotWheel / 20000.0;
    GetUpcntR++;
    if(GetUpcntR > 20){
      GetUpcntR = 0;
      Rok = 2;
    }
  }
  
  if (Cok == 1){
    MtC = KpC * kalAngleC / 90.0  + KdC * kalAngleDotC / 1000.0 + KwC * theta_YdotWheel / 20000.0;
    GetUpcntC++;
    if(GetUpcntC > 20){
      GetUpcntC = 0;
      Cok = 2;
    }
  }

  if (Lok == 2){
    AjL +=  3 * theta_LdotWheel / 1000000.0;
    MtL = KpLR * (kalAngleL2 + AjL) / 90.0 + KdLR * kalAngleDotL / 500.0 + KwLR * theta_LdotWheel / 10000.0;
    GetUpcntL++;
    if(GetUpcntL > 100){
      GetUpcntL = 0;
      Lok = 3;
    }
  }
  if (Rok == 2){
    AjR +=  3 * theta_RdotWheel / 1000000.0;
    MtR = KpLR * (kalAngleR2 + AjR) / 90.0 + KdLR * kalAngleDotR / 500.0 + KwLR * theta_RdotWheel / 10000.0;
    GetUpcntR++;
    if(GetUpcntR > 100){
      GetUpcntR = 0;
      Rok = 3;
    }
  }
  if (Lok == 3){
    AjL +=  KLR * theta_LdotWheel / 1000000.0;
    MtL = KpLR * (kalAngleL2 + AjL) / 90.0 + KdLR * kalAngleDotL / 500.0 + KwLR * theta_LdotWheel / 10000.0;
  }
  if (Rok == 3){
    AjR +=  KLR * theta_RdotWheel / 1000000.0;
    MtR = KpLR * (kalAngleR2 + AjR) / 90.0 + KdLR * kalAngleDotR / 500.0 + KwLR * theta_RdotWheel / 10000.0;
  }
  if (Cok == 2){
    GetUpY = 0;
    if(Mode == 1){
      AjC =  -KC * theta_YdotWheel / 10000.0;
      AjC2 += KC2 * omegaZ / 1000.0;
    }else{
      AjC +=  3.0 * theta_YdotWheel / 1000000.0;
      AjC2= 0.0;
    }
     
    MtC = KpC * (kalAngleC + AjC) / 90.0 + KdC * (kalAngleDotC + AjC2) / 500.0 + KwC * theta_YdotWheel / 10000.0;
  }
    
  MtL = max(-1.0f, min(1.0f, MtL));
  pwmDutyL = 1023 * (1.0 - fabs(MtL));
  MtR = max(-1.0f, min(1.0f, MtR));
  pwmDutyR = 1023 * (1.0 - fabs(MtR));
  MtC = max(-1.0f, min(1.0f, MtC));
  pwmDutyC = 1023 * (1.0 - fabs(MtC));
  
        
  //回転方向 
  if (Lok != 0 && Lok != 9){ 
    if(pwmDutyL > DutyIniL){
      digitalWrite(brakeL, LOW);
      ledcWrite(CH_L, 1023);
    }else if(MtL < 0.0){
      digitalWrite(brakeL, HIGH);
      digitalWrite(rote_pinL, LOW);
      ledcWrite(CH_L, pwmDutyL);
    }else{
      digitalWrite(brakeL, HIGH);
      digitalWrite(rote_pinL, HIGH);
      ledcWrite(CH_L, pwmDutyL);
    }
  }

  if(Rok != 0 && Rok != 9){
    if(pwmDutyR > DutyIniR){
      digitalWrite(brakeR, LOW);
      ledcWrite(CH_R, 1023);
    }else if(MtR < 0.0){
      digitalWrite(brakeR, HIGH);
      digitalWrite(rote_pinR, HIGH);
      ledcWrite(CH_R, pwmDutyR);
    }else{
      digitalWrite(brakeR, HIGH);
      digitalWrite(rote_pinR, LOW);
      ledcWrite(CH_R, pwmDutyR);
    }
  }

  if(Cok != 0 && Cok != 8 && Cok != 9){
    if(pwmDutyC > DutyIniC){
      digitalWrite(brakeC, LOW);
      ledcWrite(CH_C, 1023);
      if(Mode == 1){
        AjC2 = AjC2 / 2.0;
      }
    }else if(MtC < 0.0){
      digitalWrite(brakeC, HIGH);
      digitalWrite(rote_pinC, LOW);
      ledcWrite(CH_C, pwmDutyC);
    }else{
      digitalWrite(brakeC, HIGH);
      digitalWrite(rote_pinC, HIGH);
      ledcWrite(CH_C, pwmDutyC);
    }
  }
  
  Serial.print(", loopTime: ");
  Serial.print((float)loopTime / 1000.0);
  Serial.println("");
    
  if(GetUpY == 0){
    delay(loopDelay);
  }else{
    delay(1);
  }
}


void ENCL_READ() {
  byte cur = (!digitalRead(ENCL_B) << 1) + !digitalRead(ENCL_A);
  byte old = posL & B00000011;
  byte dir = (posL & B00110000) >> 4;
 
  if (cur == 3) cur = 2;
  else if (cur == 2) cur = 3;
 
  if (cur != old) //チャタリング防止
  {
    if (dir == 0) //回転開始と終了、方向を示す判定
    {
      if (cur == 1 || cur == 3) dir = cur;
    } 
    else {
      if (cur == 0)
      {
        if (dir == 1 && old == 3) enc_countL--;
        else if (dir == 3 && old == 1) enc_countL++;
        dir = 0;
      }
    }
 
    bool rote = 0; //回転方向
    if (cur == 3 && old == 0) rote = 0;
    else if (cur == 0 && old == 3) rote = 1;
    else if (cur > old) rote = 1;
 
    posL = (dir << 4) + (old << 2) + cur;
  }
}

void ENCR_READ() {
  byte cur = (!digitalRead(ENCR_B) << 1) + !digitalRead(ENCR_A);
  byte old = posR & B00000011;
  byte dir = (posR & B00110000) >> 4;
 
  if (cur == 3) cur = 2;
  else if (cur == 2) cur = 3;
 
  if (cur != old)
  {
    if (dir == 0)
    {
      if (cur == 1 || cur == 3) dir = cur;
    } 
    else {
      if (cur == 0)
      {
        if (dir == 1 && old == 3) enc_countR--;
        else if (dir == 3 && old == 1) enc_countR++;
        dir = 0;
      }
    }
 
    bool rote = 0;
    if (cur == 3 && old == 0) rote = 0;
    else if (cur == 0 && old == 3) rote = 1;
    else if (cur > old) rote = 1;
 
    posR = (dir << 4) + (old << 2) + cur;
  }
}

void ENCC_READ() {
  byte cur = (!digitalRead(ENCC_B) << 1) + !digitalRead(ENCC_A);
  byte old = posC & B00000011;
  byte dir = (posC & B00110000) >> 4;
 
  if (cur == 3) cur = 2;
  else if (cur == 2) cur = 3;
 
  if (cur != old)
  {
    if (dir == 0)
    {
      if (cur == 1 || cur == 3) dir = cur;
    } 
    else {
      if (cur == 0)
      {
        if (dir == 1 && old == 3) enc_countC--;
        else if (dir == 3 && old == 1) enc_countC++;
        dir = 0;
      }
    }
 
    bool rote = 0;
    if (cur == 3 && old == 0) rote = 0;
    else if (cur == 0 && old == 3) rote = 1;
    else if (cur > old) rote = 1;
 
    posC = (dir << 4) + (old << 2) + cur;
  }
}

void GetUp() {
  Serial.println("GetUp!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  GetUpBtState = 1;
}

void ModeOnOff() {
  Serial.println("ModeOnOff!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Lok = 0;
  Rok = 0;
  Cok = 0;
  if(Mode){
    Mode = 0;
  }else{
    Mode = 1;
  }
}
