#include "control.h"

// ゲインは後でマクロ定義してもいい
static float KC = 9.0, KC2 = 5.0, KLR = 12.0;

// default
static float KpLR = 8.0;
static float KdLR = 15.0;
static float KwLR = 1.4;
static float KpLR = 8.0;
static float KdLR = 15.0;
static float KwLR = 1.4;

//default
/*float KpC = 12.0;
  float KdC = 22.0;
  float KwC = 1.4;*/
//これで一応倒立
/*float KpC = 15.0;
  float KdC = 20.0;
  float KwC = 1.5;*/

/*float KpC = 17.0;
float KdC = 23.0;
float KwC = 1.4;*/

//調整用
float KpC = 2.5;
float KdC = 3.0;
float KwC = 0.1;

int rotMaxLR = 360, rotMaxCL = 70, rotMaxCR = 85;

void SetUpWheel()
{
    pinMode(brakeL, OUTPUT);
    pinMode(brakeR, OUTPUT);
    pinMode(brakeC, OUTPUT);
    pinMode(rote_pinL, OUTPUT);
    pinMode(rote_pinR, OUTPUT);
    pinMode(rote_pinC, OUTPUT);

    digitalWrite(brakeL, LOW);
    digitalWrite(brakeR, LOW);
    digitalWrite(brakeC, LOW);
}

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
  }
}

void Control(){
  MtC = KpC * (kalAngleC + AjC) / 90.0 + KdC * (kalAngleDotC + AjC2) / 500.0 + KwC * theta_YdotWheel / 10000.0;
  MtC = max(-1.0f, min(1.0f, MtC));
  pwmDutyC = 1023 * (1.0 - fabs(MtC)) - 43.0;
  //pwmDutyC = 980;
  if (kalAngleC >= 0.0 && kalAngleC <= 5.0) {
    digitalWrite(rote_pinC, HIGH);
    ledcWrite(CH_C, pwmDutyC);
  }
  else if (kalAngleC < 0.0 && kalAngleC >= -5.0){
    digitalWrite(rote_pinC, LOW);
    ledcWrite(CH_C, pwmDutyC);
  }
  else{
    digitalWrite(rote_pinC, HIGH);
    ledcWrite(CH_C, 1023);
  }
}
