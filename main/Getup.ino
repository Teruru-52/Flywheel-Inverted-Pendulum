//Core0
void disp(void *pvParameters) {
  Wire1.begin(0, 15); //SDA,SCL
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Clear the buffer
  display.clearDisplay();
  
  for (;;){
    disableCore0WDT();
    
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    
    display.setCursor(0,0);            
    display.println(kalAngleL + AjL, 1);
    display.setCursor(50,0);            
    display.println(kalAngleC + AjC, 1);
    display.setCursor(90,0);            
    display.println(kalAngleR + AjR, 1);
    
    display.setCursor(0,9);  
    if(Mode){
      display.println("Point inverted");          
    }else{
      display.println("Side inverted");
    }

    int LineC = map(kalAngleC + AjC, 20, -20, 0, 127);
    if(LineC >= 0 && LineC < 128){
      if(abs(kalAngleC) <= 1.0){
        display.fillRect(LineC-2, 16, 5, display.height()-1, SSD1306_WHITE); //(左上x, 左上y, 幅, 高さ, 線の色
      }else{
        display.drawLine(LineC, 16, LineC, display.height()-1, SSD1306_WHITE); //始点x, 始点y, 終点x, 終点y, 線の色
      }
    }
    
    int LineL = map(kalAngleL + AjL, 45+8, 45-8, 16, 63);
    if(LineL >= 16 && LineL < 64){
      if(abs(kalAngleL2) <= 1.0){
        display.fillRect(0, LineL - 2 , display.width()/2-1, 5, SSD1306_WHITE); 
      }else{
        display.drawLine(0, LineL, display.width()/2-1, LineL, SSD1306_WHITE);
      }
    }
    
    int LineR = map(kalAngleR + AjR, 45+8, 45-8, 16, 63);
    if(LineR >= 16 && LineR < 64){
      if(abs(kalAngleR2) <= 1.0){
        display.fillRect(display.width()/2, LineR - 2, display.width()-1, 5, SSD1306_WHITE); //(左上x, 左上y, 幅, 高さ, 線の色
      }else{
        display.drawLine(display.width()/2, LineR, display.width()-1, LineR, SSD1306_WHITE);
      }
    }
      
    display.display(); 
    if(Lok != 9 && Rok != 9) delay(30);
    display.clearDisplay();

    
    //起き上がりX
    if(GetUpX == 1){
      digitalWrite(brakeL, HIGH);
      digitalWrite(brakeR, HIGH);
      
      //回転方向
      digitalWrite(rote_pinL, HIGH);
      digitalWrite(rote_pinR, LOW);
      
    
      for(int i = 1023; i >= rotMaxLR; i--){
        ledcWrite(CH_L, i);
        ledcWrite(CH_R, i);
        delay(5);
      }
      ledcWrite(CH_L, rotMaxLR);
      ledcWrite(CH_R, rotMaxLR);
      delay(300);
    
      ledcWrite(CH_L, 1023);
      ledcWrite(CH_R, 1023);
      ledcWrite(CH_ServoL, servoIniL + servoBrakeLR); //servo brake
      ledcWrite(CH_ServoR, servoIniR - servoBrakeLR);
      delay(160);
      ledcWrite(CH_ServoL, servoIniL);
      ledcWrite(CH_ServoR, servoIniR);

      Lok = 9;
      Rok = 9;
      
      Mode = 1;
      GetUpX = 0;
    }

    //起き上がりX軸角度計測
    if((Lok == 9 || Rok == 9) && dt < 1.0){
      if(kalAngleL2 > getUpDeg || kalAngleR2 > getUpDeg){
        Lok = 2;
        Rok = 2;
      }
    }
  }
}

//起き上がりY
void getupY(){
  digitalWrite(brakeC, HIGH);
  int rotMax;
  GetUpY = 1;
  
  //回転方向
  if(kalAngleC < 0.0){
    rotMax = rotMaxCR;
    digitalWrite(rote_pinC, HIGH);
    getUpDeg = -5.0;
    Cok = 8;
  }else{
    rotMax = rotMaxCL;
    digitalWrite(rote_pinC, LOW);
    getUpDeg = 5.0;
    Cok = 9;
  }

  for(int i = 1023; i >= rotMax; i--){
    ledcWrite(CH_C, i);
    delay(5);
  }
  ledcWrite(CH_C, rotMax);
  delay(300);

  ledcWrite(CH_C, 1023);
  if(kalAngleC < 0.0){
    ledcWrite(CH_ServoC, servoIniC + servoBrakeC); //servo brake
  }else{
    ledcWrite(CH_ServoC, servoIniC - servoBrakeC); //servo brake
  }
  delay(160);
  ledcWrite(CH_ServoC, servoIniC);
  delay(240);
}
