#define servoL 4
#define servoR 13
#define servoC 14

#define CH_L 0
#define CH_R 1
#define CH_C 2

#define CH_ServoL 13
#define CH_ServoR 14
#define CH_ServoC 15

int servoIniL = 4600, servoIniR = 4600, servoIniC = 5000;

void setup() {
  ledcSetup(CH_ServoL, 50, 16);
  ledcAttachPin(servoL, CH_ServoL);
  
  ledcSetup(CH_ServoR, 50, 16);
  ledcAttachPin(servoR, CH_ServoR);
  
  ledcSetup(CH_ServoC, 50, 16);
  ledcAttachPin(servoC, CH_ServoC);
  
  delay(500); 
}


void loop() {
  ledcWrite(CH_ServoL, servoIniL);
  ledcWrite(CH_ServoR, servoIniR);
  ledcWrite(CH_ServoC, servoIniC);
  delay(500); 
}
