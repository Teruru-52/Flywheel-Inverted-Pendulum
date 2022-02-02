#include "wheel.hpp"

void Wheels::Wheels()
    : enc_l(32, 33),
      enc_r(39, 36),
      enc_c(35, 34) {}

void Wheels::SetUpWheel()
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

void WheelsController::WheelsCotroller(float Kpc, float Kdc, float Kwc, float Kpl, float Kdl, float Kwl, float Kpr, float Kdr, float Kwr)
    : Kpc(Kpc),
      Kdc(Kdc),
      Kwc(Kwc),
      Kpl(Kpl),
      Kdl(Kdl),
      Kwl(Kwl),
      Kpr(Kpr),
      Kdr(Kdr),
      Kwr(Kwr) {}

void WheelsController::GetWheelVel(){
  wheel_vel = -1.0 * float(enc_count) * 3.6 / ts; //2×180°/100=3.6
  enc_count = 0;
}

void WheelsController::Control_1d(){
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

// void WheelsController::Control_3d(){
//   MtL = KpL * kalAngleL + KdL * kalAngleDotL + KwL * theta_YdotWheel;
//   MtL = max(-1.0f, min(1.0f, MtL));
//   input_c = 1023 * (1.0 - fabs(MtL)) - 43.0;
// }

void WheelsController::TestControl(){
  digitalWrite(ROT_DIR_C, HIGH);
  ledcWrite(CHANNEL_C, 800);
}

void WheelsController::WheelBrake(){
  if(fabs(theta_x_est) > angle_limit)
    digitalWrite(BRAKE_L, LOW);
    digitalWrite(BRAKE_R, LOW);
  if(fabs(theta_y_est) > angle_limit)
    digitalWrite(BRAKE_C, LOW);
}
