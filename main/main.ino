#include "main.h"
#include "my_header.h"

#define GetUpBt 2
#define ModeBt 12

Encoder_Typedef Enc_l, Enc_r, Enc_c;
MPU6050 mpu;
Kalman kalmanL, kalmanR, kalmanC;

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

float KC = 9.0, KC2 = 5.0, KLR = 12.0;

float getUpDeg = -5.0;

int loopDelay = 5;
//----------------------------
float kalAngleL, kalAngleL2, kalAngleR, kalAngleR2, kalAngleDotL, kalAngleDotR, kalAngleC, kalAngleDotC;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(GetUpBt, INPUT_PULLUP);
  pinMode(ModeBt, INPUT_PULLUP);
  attachInterrupt(GetUpBt, GetUp, FALLING);
  attachInterrupt(ModeBt, ModeOnOff, FALLING);

  SetUpWheel();
  SetUpEncoder();

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

  SetUpServo();
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
  GetSamplingTime();

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

  // Serial.print(", loopTime: ");
  // Serial.print((float)loopTime / 1000.0);
  // Serial.println("");

  if (GetUpY == 0) {
    delay(loopDelay);
  } else {
    delay(1);
  }
}
