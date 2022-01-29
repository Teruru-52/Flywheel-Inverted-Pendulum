#include "display.h"

extern float kalAngleL, kalAngleR, kalAngleC;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

void DispInit(){
  xTaskCreatePinnedToCore(
    disp
    ,  "disp"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL
    ,  0);
}

void Disp(void *pvParameters) {
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

    int LineC = map(kalAngleC, 20, -20, 0, 127);
    if(LineC >= 0 && LineC < 128){
      if(abs(kalAngleC) <= 1.0){
        display.fillRect(LineC-2, 16, 5, display.height()-1, SSD1306_WHITE); //(左上x, 左上y, 幅, 高さ, 線の色
      }else{
        display.drawLine(LineC, 16, LineC, display.height()-1, SSD1306_WHITE); //始点x, 始点y, 終点x, 終点y, 線の色
      }
    }
    
    int LineL = map(kalAngleL, 45+8, 45-8, 16, 63);
    if(LineL >= 16 && LineL < 64){
      if(abs(kalAngleL2) <= 1.0){
        display.fillRect(0, LineL - 2 , display.width()/2-1, 5, SSD1306_WHITE); 
      }else{
        display.drawLine(0, LineL, display.width()/2-1, LineL, SSD1306_WHITE);
      }
    }
    
    int LineR = map(kalAngleR, 45+8, 45-8, 16, 63);
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