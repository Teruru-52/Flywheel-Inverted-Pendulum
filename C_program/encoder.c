#include "encoder.h"

Encoder_Typedef Enc_l, Enc_r, Enc_c;

// void SetUpEncoder(){
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

// void EncoderRead(Encoder_Typedef *encoder) {
//   byte cur = (!digitalRead(encoder->phaseB) << 1) + !digitalRead(encoder->phaseA);
//   byte old = encoder->pos & B00000011;
//   byte dir = (encoder->pos & B00110000) >> 4;
 
//   if (cur == 3) cur = 2;
//   else if (cur == 2) cur = 3;
 
//   if (cur != old) //チャタリング防止
//   {
//     if (dir == 0) //回転開始と終了、方向を示す判定
//     {
//       if (cur == 1 || cur == 3) dir = cur;
//     } 
//     else {
//       if (cur == 0)
//       {
//         if (dir == 1 && old == 3) encoder->enc_count--;
//         else if (dir == 3 && old == 1) encoder->enc_count++;
//         dir = 0;
//       }
//     }
 
//     bool rote = 0; //回転方向
//     if (cur == 3 && old == 0) rote = 0;
//     else if (cur == 0 && old == 3) rote = 1;
//     else if (cur > old) rote = 1;
 
//     encoder->pos = (dir << 4) + (old << 2) + cur;
//   }
// }

void GetWheelVel(Encoder_Typedef *encoder_l, Encoder_Typedef *encoder_r, Encoder_Typedef *encoder_c, float ts){
  encoder_l->wheel_vel = -1.0 * float(encoder_l->enc_count) * 3.6 / ts; //2×180°/100=3.6
  encoder_l->enc_count = 0;
  encoder_r->wheel_vel = -1.0 * float(encoder_r->enc_count) * 3.6 / ts;
  encoder_r->enc_count = 0;
  encoder_c->wheel_vel = -1.0 * float(encoder_c->enc_count) * 3.6 / ts; 
  encoder_c->enc_count = 0;
}
