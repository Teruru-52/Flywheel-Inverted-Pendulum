#include "wheel.hpp"

Wheels::Wheels()
  : enc_l(ENCL_A, ENCL_B),
    enc_r(ENCR_A, ENCR_B),
    enc_c(ENCC_A, ENCC_B) {}

void Wheels::SetUpWheel()
{
  pinMode(BRAKE_C, OUTPUT);
  pinMode(BRAKE_L, OUTPUT);
  pinMode(BRAKE_R, OUTPUT);
  pinMode(ROT_DIR_C, OUTPUT);
  pinMode(ROT_DIR_L, OUTPUT);
  pinMode(ROT_DIR_R, OUTPUT);

  digitalWrite(BRAKE_C, LOW);
  digitalWrite(BRAKE_L, LOW);
  digitalWrite(BRAKE_R, LOW);

  // ledcSetup(uint8_t chan, double freq, uint8_t bit_num);
  // PWM 20kHz, 10bit
  ledcSetup(CHANNEL_L, 20000, 10);
  ledcAttachPin(PWM_PIN_L, CHANNEL_L);
  ledcSetup(CHANNEL_R, 20000, 10);
  ledcAttachPin(PWM_PIN_R, CHANNEL_R);
  ledcSetup(CHANNEL_C, 20000, 10);
  ledcAttachPin(PWM_PIN_C, CHANNEL_C);

  ledcWrite(CHANNEL_C, 1023);
  ledcWrite(CHANNEL_L, 1023);
  ledcWrite(CHANNEL_R, 1023);
}

void Wheels::EncoderReadL()
{
  enc_l.EncoderRead();
}

void Wheels::EncoderReadR()
{
  enc_r.EncoderRead();
}

void Wheels::EncoderReadC()
{
  enc_c.EncoderRead();
}

std::array<float, 3> Wheels::GetWheelVel(float dt)
{
  wheel_vel[0] = -1.0 * float(enc_c.count) * M_PI / 50.0 / dt; // 2×π/100=0.0628 [rad]
  enc_c.count = 0;
  wheel_vel[1] = -1.0 * float(enc_l.count) * M_PI / 50.0 / dt;
  enc_l.count = 0;
  wheel_vel[2] = 1.0 * float(enc_r.count) * M_PI / 50.0 / dt;
  enc_r.count = 0;

  return wheel_vel;
}

void Wheels::WheelBrakeOn(int Mode, std::array<float, 3> theta)
{
  if (Mode == 0) {
    digitalWrite(BRAKE_C, LOW);
    digitalWrite(BRAKE_L, LOW);
    digitalWrite(BRAKE_R, LOW);
  }
  if (Mode == 1) {
    if (fabs(theta[Mode - 1]) > angle_limit) {
      ledcWrite(CHANNEL_C, 1023);
      digitalWrite(BRAKE_C, LOW);
    }
    else digitalWrite(BRAKE_C, HIGH);
  }
  if (Mode == 2) {
    if (fabs(theta[Mode - 1]) > angle_limit) {
      ledcWrite(CHANNEL_L, 1023);
      digitalWrite(BRAKE_L, LOW);
    }
    else digitalWrite(BRAKE_L, HIGH);
  }
  if (Mode == 3) {
    if (fabs(theta[Mode - 1]) > angle_limit) {
      ledcWrite(CHANNEL_R, 1023);
      digitalWrite(BRAKE_R, LOW);
    }
    else digitalWrite(BRAKE_R, HIGH);
  }
}

void Wheels::WheelBrakeOff(int Mode)
{
  if (Mode == 1) {
    digitalWrite(BRAKE_C, HIGH);
    digitalWrite(BRAKE_L, LOW);
    digitalWrite(BRAKE_R, LOW);
  }
  if (Mode == 2) {
    digitalWrite(BRAKE_C, LOW);
    digitalWrite(BRAKE_L, HIGH);
    digitalWrite(BRAKE_R, LOW);
  }
  if (Mode == 3) {
    digitalWrite(BRAKE_C, LOW);
    digitalWrite(BRAKE_L, LOW);
    digitalWrite(BRAKE_R, HIGH);
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

void WheelsController::Control_1d(int Mode, std::array<float, 3>  theta, std::array<float, 3>  dot_theta, std::array<float, 3>  omega)
{
  if (Mode == 1) {
    input_c = Kpc * theta[0] + Kdc * dot_theta[0] + Kwc * omega[0];

    if (input_c > input_limit)
      input_c = input_limit;
    if (input_c < -input_limit)
      input_c = -input_limit;

    if (input_c > 0)
    {
      input_c = 1023 - input_offset - input_c;
      digitalWrite(ROT_DIR_C, HIGH);
      ledcWrite(CHANNEL_C, input_c);
      //    Serial.print(input_c);
      //    Serial.print(",");
    }
    else if (input_c < 0)
    {
      input_c = 1023 - input_offset + input_c;
      digitalWrite(ROT_DIR_C, LOW);
      ledcWrite(CHANNEL_C, input_c);
      //    Serial.print(input_c);
      //    Serial.print(",");
    }
    else // input_c == 0
    {
      digitalWrite(ROT_DIR_C, HIGH);
      ledcWrite(CHANNEL_C, 1023);
      //    Serial.print(0);
      //    Serial.print(",");
    }
  }
  if (Mode == 2) {
    input_l = Kpl * theta[1] + Kdl * dot_theta[1] + Kwl * omega[1];

    if (input_l > input_limit)
      input_l = input_limit;
    if (input_l < -input_limit)
      input_l = -input_limit;

    if (input_l > 0)
    {
      input_l = 1023 - input_offset - input_l;
      digitalWrite(ROT_DIR_L, HIGH);
      ledcWrite(CHANNEL_L, input_l);
      //    Serial.print(input_l);
      //    Serial.print(",");
    }
    else if (input_l < 0)
    {
      input_l = 1023 - input_offset + input_l;
      digitalWrite(ROT_DIR_L, LOW);
      ledcWrite(CHANNEL_L, input_l);
      //    Serial.print(input_l);
      //    Serial.print(",");
    }
    else // input_l == 0
    {
      digitalWrite(ROT_DIR_L, HIGH);
      ledcWrite(CHANNEL_L, 1023);
      //    Serial.print(0);
      //    Serial.print(",");
    }
  }
  if (Mode == 3) {
    input_r = Kpr * theta[2] + Kdr * dot_theta[2] + Kwr * omega[2];

    if (input_r > input_limit)
      input_r = input_limit;
    if (input_r < -input_limit)
      input_r = -input_limit;

    if (input_r > 0)
    {
      input_r = 1023 - input_offset - input_r;
      digitalWrite(ROT_DIR_R, LOW);
      ledcWrite(CHANNEL_R, input_r);
      //    Serial.print(input_r);
      //    Serial.print(",");
    }
    else if (input_r < 0)
    {
      input_r = 1023 - input_offset + input_r;
      digitalWrite(ROT_DIR_R, HIGH);
      ledcWrite(CHANNEL_R, input_r);
      //    Serial.print(input_r);
      //    Serial.print(",");
    }
    else // input_r == 0
    {
      digitalWrite(ROT_DIR_R, HIGH);
      ledcWrite(CHANNEL_R, 1023);
      //    Serial.print(0);
      //    Serial.print(",");
    }
  }
}

// void WheelsController::Control_3d(){
//   MtL = KpL * kalAngleL + KdL * kalAngleDotL + KwL * theta_YdotWheel;
//   MtL = max(-1.0f, min(1.0f, MtL));
//   input_r = 1023 * (1.0 - fabs(MtL)) - 43.0;
// }

void WheelsController::TestControl() {
  digitalWrite(ROT_DIR_C, HIGH); // ホイール右回転
  ledcWrite(CHANNEL_C, 980);
}
