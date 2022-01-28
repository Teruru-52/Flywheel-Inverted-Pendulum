#include "encoder.h"

void SetUpEncoder(){
  pinMode(ENCL_A, INPUT);
  pinMode(ENCL_B, INPUT);
  pinMode(ENCR_A, INPUT);
  pinMode(ENCR_B, INPUT);
  pinMode(ENCC_A, INPUT);
  pinMode(ENCC_B, INPUT);

  attachInterrupt(ENCL_A, ENCL_READ, CHANGE);
  attachInterrupt(ENCL_B, ENCL_READ, CHANGE);
  attachInterrupt(ENCR_A, ENCR_READ, CHANGE);
  attachInterrupt(ENCR_B, ENCR_READ, CHANGE);
  attachInterrupt(ENCC_A, ENCC_READ, CHANGE);
  attachInterrupt(ENCC_B, ENCC_READ, CHANGE);
}

void EncRead(Encoder_Typedef *encoder) {
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

void GetWheelVel(Encoder_typedef *encoder, Encoder_typedef *encoder, Encoder_typedef *encoder){
  float theta_LdotWheel = -1.0 * float(enc_countL) * 3.6 / dt; //2×180°/100=3.6
  enc_countL = 0;
  float theta_RdotWheel = 1.0 * float(enc_countR) * 3.6 / dt;
  enc_countR = 0;
  float theta_YdotWheel = -1.0 * float(enc_countC) * 3.6 / dt;
  enc_countC = 0;
}
