#include "encoder.hpp"

Encoder::Encoder(int enc_a, int enc_b)
    : enc_a(enc_a),
      enc_b(enc_b) {}

// void Encoder::SetUpEncoder(){
//   pinMode(ENCL_A, INPUT);
//   pinMode(ENCL_B, INPUT);
//   pinMode(ENCR_A, INPUT);
//   pinMode(ENCR_B, INPUT);
//   pinMode(ENCC_A, INPUT);
//   pinMode(ENCC_B, INPUT);

//   attachInterrupt(ENCL_A, EncoderRead(&Enc_l), CHANGE);
//   attachInterrupt(ENCL_B, EncoderRead(&Enc_l), CHANGE);
//   attachInterrupt(ENCR_A, EncoderRead(&Enc_r), CHANGE);
//   attachInterrupt(ENCR_B, EncoderRead(&Enc_r), CHANGE);
//   attachInterrupt(ENCC_A, EncoderRead(&Enc_c), CHANGE);
//   attachInterrupt(ENCC_B, EncoderRead(&Enc_c), CHANGE);

//   Enc_l.enc_count = 0;
//   Enc_r.enc_count = 0;
//   Enc_c.enc_count = 0;
//   Enc_l.phaseA = ENCL_A;
//   Enc_l.phaseB = ENCL_B;
//   Enc_r.phaseA = ENCR_A;
//   Enc_r.phaseB = ENCR_B;
//   Enc_c.phaseA = ENCC_A;
//   Enc_c.phaseB = ENCC_B;
// }

void Encoder::EncoderRead() {
  byte cur = (!digitalRead(enc_b) << 1) + !digitalRead(enc_a);
  byte old = pos & B00000011;
  byte dir = (pos & B00110000) >> 4;
 
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
        if (dir == 1 && old == 3) enc_count--;
        else if (dir == 3 && old == 1) enc_count++;
        dir = 0;
      }
    }
 
    bool rote = 0; //回転方向
    if (cur == 3 && old == 0) rote = 0;
    else if (cur == 0 && old == 3) rote = 1;
    else if (cur > old) rote = 1;
 
    pos = (dir << 4) + (old << 2) + cur;
  }
}