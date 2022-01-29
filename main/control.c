#include "control.h"

// ゲインは後でマクロ定義してもいい
static const float KC = 9.0, KC2 = 5.0, KLR = 12.0;

// default
static const float KpR = 8.0;
static const float KdR = 15.0;
static const float KwR = 1.4;
static const float KpL = 8.0;
static const float KdL = 15.0;
static const float KwL = 1.4;
static const float KpC = 2.5;
static const float KdC = 3.0;
static const float KwC = 0.1;

//default
// static const float KpC = 12.0;
// static const float KdC = 22.0;
// static const float KwC = 1.4;
//これで一応倒立
// static const float KpC = 15.0;
// static const float KdC = 20.0;
// static const float KwC = 1.5;

static const float degL = 45, degR = 45;
static float MtL, MtR, MtC;
static int DutyIniL = 1020, DutyIniR = DutyIniL, DutyIniC = DutyIniL;
static int input_c, input_r, input_c;

void SetUpWheel()
{
  pinMode(BRAKE_L, OUTPUT);
  pinMode(BRAKE_R, OUTPUT);
  pinMode(BRAKE_C, OUTPUT);
  pinMode(ROT_DIR_L, OUTPUT);
  pinMode(ROT_DIR_R, OUTPUT);
  pinMode(ROT_DIR_C, OUTPUT);

  digitalWrite(BRAKE_L, LOW);
  digitalWrite(BRAKE_R, LOW);
  digitalWrite(BRAKE_C, LOW);

  // ledcSetup(uint8_t chan, double freq, uint8_t bit_num);
  // PWM 20kHz, 10bit
  ledcSetup(CHANNEL_L, 20000, 10);
  ledcAttachPin(PWM_PIN_L, CHANNEL_L);
  ledcSetup(CHANNEL_R, 20000, 10);
  ledcAttachPin(PWM_PIN_R, CHANNEL_R);
  ledcSetup(CHANNEL_C, 20000, 10);
  ledcAttachPin(PWM_PIN_C, CHANNEL_C);
}

void ControlC(){
  MtC = KpC * kalAngleC  + KdC * kalAngleDotC + KwC * theta_YdotWheel;
  MtC = max(-1.0f, min(1.0f, MtC));
  input_c = 1023 * (1.0 - fabs(MtC)) - 43.0;
  //input_c = 980;
  if (kalAngleC >= 0.0 && kalAngleC <= 5.0) {
    digitalWrite(ROT_DIR_C, HIGH);
    ledcWrite(CHANNEL_C, input_c);
  }
  else if (kalAngleC < 0.0 && kalAngleC >= -5.0){
    digitalWrite(ROT_DIR_C, LOW);
    ledcWrite(CHANNEL_C, input_c);
  }
  else{
    digitalWrite(ROT_DIR_C, HIGH);
    ledcWrite(CHANNEL_C, 1023);
  }
}

void ControlL(){
  MtL = KpL * kalAngleL + KdL * kalAngleDotL + KwL * theta_YdotWheel;
  MtL = max(-1.0f, min(1.0f, MtL));
  input_c = 1023 * (1.0 - fabs(MtL)) - 43.0;
}

void ControlR(){
  MtR = KpR * kalAngleR + KdR * kalAngleDotR + KwR * theta_YdotWheel;
  MtR = max(-1.0f, min(1.0f, MtR));
  input_r = 1023 * (1.0 - fabs(MtR)) - 43.0;
}
