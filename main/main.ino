#include <Kalman.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define ENCL_A 32
#define ENCL_B 33
#define ENCR_A 39
#define ENCR_B 36
#define ENCC_A 35
#define ENCC_B 34

#define GetUpBt 2
#define ModeBt 12


#define PWM_pinL 27
#define brakeL 19
#define rote_pinL 23
#define servoL 4

#define PWM_pinR 25
#define brakeR 17
#define rote_pinR 16
#define servoR 13

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
/*float KpC = 17.0;
float KdC = 23.0;
float KwC = 1.4;*/

//調整用
float KpC = 2.5;
float KdC = 3.0;
float KwC = 0.1;

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
  omegaZ = theta_Ydot * sin(theta_X * PI / 180.0) / cos(theta_Y * PI / 180.0) + theta_Zdot * cos(theta_X * PI / 180.0) / cos(theta_Y * PI / 180.0);

  digitalWrite(brakeC, HIGH);
  MtC = KpC * (kalAngleC + AjC) / 90.0 + KdC * (kalAngleDotC + AjC2) / 500.0 + KwC * theta_YdotWheel / 10000.0;
  MtC = max(-1.0f, min(1.0f, MtC));
  pwmDutyC = 1023 * (1.0 - fabs(MtC)) - 43.0;
  //pwmDutyC = 980;
  if (kalAngleC >= 0.0 && kalAngleC <= 5.0) {
    digitalWrite(rote_pinC, HIGH);
    ledcWrite(CH_C, pwmDutyC);
  }
  else if (kalAngleC < 0.0 && kalAngleC >= -5.0){
    digitalWrite(rote_pinC, LOW);
    ledcWrite(CH_C, pwmDutyC);
  }
  else{
    digitalWrite(rote_pinC, HIGH);
    ledcWrite(CH_C, 1023);
  }

  Serial.print(", loopTime: ");
  Serial.print((float)loopTime / 1000.0);
  Serial.println("");

  if (GetUpY == 0) {
    delay(loopDelay);
  } else {
    delay(1);
  }
}
