
void ENCL_READ() {
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
