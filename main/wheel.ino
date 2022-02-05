#include "wheel.hpp"

Wheels::Wheels()
    : enc_l(ENCL_A, ENCL_B),
      enc_r(ENCR_A, ENCR_B),
      enc_c(ENCC_A, ENCC_B) {}

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

void Wheels::SetUpEncoder()
{
  pinMode(ENCL_A, INPUT);
  pinMode(ENCL_B, INPUT);
  pinMode(ENCR_A, INPUT);
  pinMode(ENCR_B, INPUT);
  pinMode(ENCC_A, INPUT);
  pinMode(ENCC_B, INPUT);

  attachInterrupt(ENCL_A, enc_l.EncoderRead, CHANGE);
  attachInterrupt(ENCL_B, enc_l.EncoderRead, CHANGE);
  attachInterrupt(ENCR_A, enc_r.EncoderRead, CHANGE);
  attachInterrupt(ENCR_B, enc_r.EncoderRead, CHANGE);
  attachInterrupt(ENCC_A, enc_c.EncoderRead, CHANGE);
  attachInterrupt(ENCC_B, enc_c.EncoderRead, CHANGE);
}

float Wheels::GetWheelVel(float dt)
{
  wheel_vel_l = -1.0 * float(enc_l.count) * M_PI / 50.0 / dt; // 2×π/100=0.0628 [rad]
  enc_l.count = 0;
  wheel_vel_r = -1.0 * float(enc_r.count) * M_PI / 50.0 / dt;
  enc_r.count = 0;
  wheel_vel_c = -1.0 * float(enc_c.count) * M_PI / 50.0 / dt;
  enc_c.count = 0;

  return wheel_vel_c;
}

void Wheels::WheelBrake(float theta)
{
  // if(fabs(theta_x_est) > angle_limit)
  //   digitalWrite(BRAKE_L, LOW);
  //   digitalWrite(BRAKE_R, LOW);
  if (fabs(theta) > angle_limit)
  {
    digitalWrite(BRAKE_C, LOW);
    digitalWrite(BRAKE_L, LOW);
    digitalWrite(BRAKE_R, LOW);
  }
}

WheelsController::WheelsController(float Kpc, float Kdc, float Kwc, float Kpl, float Kdl, float Kwl, float Kpr, float Kdr, float Kwr)
    : Kpc(Kpc),
      Kdc(Kdc),
      Kwc(Kwc),
      Kpl(Kpl),
      Kdl(Kdl),
      Kwl(Kwl),
      Kpr(Kpr),
      Kdr(Kdr),
      Kwr(Kwr) {}

void WheelsController::Control_1d(float theta, float dot_theta, float omega)
{
  input_c = Kpc * theta + Kdc * dot_theta + Kwc * omega;

  if (input_c > input_limit)
    input_c = input_limit;
  if (input_c < -input_limit)
    input_c = -input_limit;

  if (input_c > 0)
  {
    input_c = 1023 - input_offset - input_c; 
    digitalWrite(ROT_DIR_C, HIGH);
    ledcWrite(CHANNEL_C, input_c);
  }
  else if (input_c < 0)
  {
    input_c = 1023 - input_offset + input_c;
    digitalWrite(ROT_DIR_C, LOW);
    ledcWrite(CHANNEL_C, input_c);
  }
  else // input_c == 0
  {
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
