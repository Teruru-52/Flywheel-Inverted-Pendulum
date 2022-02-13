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
    digitalWrite(BRAKE_L, LOW);
    digitalWrite(BRAKE_R, LOW);
    if (fabs(theta[Mode - 1]) > angle_limit) {
      ledcWrite(CHANNEL_C, 1023);
      digitalWrite(BRAKE_C, LOW);
    }
    else digitalWrite(BRAKE_C, HIGH);
  }
  if (Mode == 2) {
    digitalWrite(BRAKE_C, LOW);
    digitalWrite(BRAKE_R, LOW);
    if (fabs(theta[Mode - 1]) > angle_limit) {
      ledcWrite(CHANNEL_L, 1023);
      digitalWrite(BRAKE_L, LOW);
    }
    else digitalWrite(BRAKE_L, HIGH);
  }
  if (Mode == 3) {
    digitalWrite(BRAKE_C, LOW);
    digitalWrite(BRAKE_L, LOW);
    if (fabs(theta[Mode - 1]) > angle_limit) {
      ledcWrite(CHANNEL_R, 1023);
      digitalWrite(BRAKE_R, LOW);
    }
    else digitalWrite(BRAKE_R, HIGH);
  }
  if (Mode == 4) {
    if ((fabs(theta[0]) > angle_limit) || (fabs(theta[1]) > angle_limit) || (fabs(theta[2]) > angle_limit)) {
      ledcWrite(CHANNEL_C, 1023);
      digitalWrite(BRAKE_C, LOW);
      ledcWrite(CHANNEL_L, 1023);
      digitalWrite(BRAKE_L, LOW);
      ledcWrite(CHANNEL_R, 1023);
      digitalWrite(BRAKE_R, LOW);
    }
    else {
      digitalWrite(BRAKE_C, HIGH);
      digitalWrite(BRAKE_L, HIGH);
      digitalWrite(BRAKE_R, HIGH);
    }
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
    Kwr(Kwr),
    pre_input(0),
    pre_error(0),
    sum_error(0),
    pre_error2(0),
    sum_error2(0) {}

void WheelsController::DirControl(int Mode, float input, int dir) {
  if (Mode == 1) {
    if (dir)
      digitalWrite(ROT_DIR_C, LOW);
    else
      digitalWrite(ROT_DIR_C, HIGH);
    ledcWrite(CHANNEL_C, input);
  }
  else if (Mode == 2) {
    if (dir)
      digitalWrite(ROT_DIR_L, LOW);
    else
      digitalWrite(ROT_DIR_L, HIGH);
    ledcWrite(CHANNEL_L, input);
  }
  else if (Mode == 3) {
    if (dir)
      digitalWrite(ROT_DIR_R, HIGH);
    else
      digitalWrite(ROT_DIR_R, LOW);
    ledcWrite(CHANNEL_R, input);
  }
}

void WheelsController::WheelDrive(int Mode, float input) {
  if (input > input_limit)
    input = input_limit;
  else if (input < -input_limit)
    input = -input_limit;

  if (input > 0)
  {
    //    Serial.println(input_r);
    input = 1023 - input;
    DirControl(Mode, input, 1);
  }
  else if (input < 0)
  {
    //    Serial.println(input_r);
    input = 1023 + input;
    DirControl(Mode, input, 0);
  }
  else // input == 0
  {
    input = 1023;
    DirControl(Mode, input, 0);
  }
  Serial.println(input);
}


void WheelsController::Control_1d(int Mode, std::array<float, 3>  theta, std::array<float, 3>  dot_theta, std::array<float, 3>  omega)
{
  float input;
  if (Mode == 1)
    input = Kpc * theta[0] + Kdc * dot_theta[0] + Kwc * omega[0];
  if (Mode == 2)
    input = Kpl * theta[1] + Kdl * dot_theta[1] + Kwl * omega[1];
  if (Mode == 3)
    input = Kpr * theta[2] + Kdr * dot_theta[2] + Kwr * omega[2];

  WheelDrive(Mode, input);
}

void WheelsController::Control_3d(std::array<float, 3>  theta, std::array<float, 3>  dot_theta, std::array<float, 3>  omega) {
  Control_1d(1, theta, dot_theta, omega);
  Control_1d(2, theta, dot_theta, omega);
  Control_1d(3, theta, dot_theta, omega);
}

void WheelsController::VelocityControl(int Mode, std::array<float, 3>  theta, std::array<float, 3>  dot_theta, std::array<float, 3>  omega) {
  if (fabs(theta[Mode - 1])  <= angle_limit) {
    float tau_ref = Kpr * theta[Mode - 1] + Kdr * dot_theta[Mode - 1] + Kwr * omega[Mode - 1];
    //  float omega_ref = omega[2] + tau_ref;
    //float omega_ref = 300.0;
    float error = tau_ref;
    //  float error = omega_ref - omega[2];
    //  float deriv = (pre_error - error) * 100;
    //  pre_error = error;
    sum_error += (error * 0.01);
    float input = Kp * error + Ki * sum_error;

    Serial.print(tau_ref);
    Serial.print(",");
    Serial.println(input);

    WheelDrive(Mode, input);
  }
  else {
    sum_error = 0;
  }
}

void WheelsController::AngleControl(int Mode, std::array<float, 3>  theta, std::array<float, 3>  dot_theta) {
  if (fabs(theta[Mode - 1])  <= angle_limit) {
    float error = - theta[Mode - 1];
    float error2 = - dot_theta[Mode - 1];
    sum_error += (error * 0.01);
    sum_error2 += (error2 * 0.01);
    float deriv = (pre_error - error) * 100;
    float deriv2 = (pre_error2 - error) * 100;
    pre_error = error;
    pre_error2 = error2;
    //    float input = kp2 * error2 + kd2 * deriv2 + ki2 * sum_error2;
    float input = kp * error + kd * deriv + ki * sum_error;
    if ((pre_input > 0 && input < 0) || (pre_input < 0 && input > 0))
      sum_error = 0;
    pre_input = input;
    Serial.println(input);

    WheelDrive(Mode, input);
  }
  else {
    sum_error = 0;
    pre_error = 0;
    sum_error2 = 0;
    pre_error2 = 0;
  }
}

void WheelsController::TestControl(int Mode) {
  WheelDrive(Mode, 300);
}
