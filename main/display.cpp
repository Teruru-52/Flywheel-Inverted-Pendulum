#include "display.hpp"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

extern float dt;
extern int invert_mode;
extern int tuning_mode;
extern std::array<float, 3> theta;
extern std::array<float, 3> dot_theta;
extern std::array<float, 3> input;
extern std::array<float, 3> gain;

void DispInit()
{
  xTaskCreatePinnedToCore(
      Disp, "Disp" // A name just for humans
      ,
      4096 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL, 1 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL, 0);
}

void Disp(void *pvParameters)
{
  Wire1.begin(0, 15); // SDA,SCL
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Clear the buffer
  display.clearDisplay();

  for (;;)
  {
    disableCore0WDT();

    display.setTextSize(1);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text

    display.setCursor(0, 0);
    display.println(theta[0], 2);
    display.setCursor(50, 0);
    display.println(theta[1], 2);
    display.setCursor(90, 0);
    display.println(dt, 3);

    display.setCursor(0, 9);
    display.println(dot_theta[0], 2);
    display.setCursor(50, 9);
    display.println(dot_theta[1], 2);
    display.setCursor(90, 9);
    display.println(dot_theta[2], 2);

    display.setCursor(0, 18);
    display.println(input[1], 2); // input_L
    display.setCursor(50, 18);
    display.println(input[0], 2); // input_C
    display.setCursor(90, 18);
    display.println(input[2], 2); // input_R

    display.setCursor(0, 27);
    display.println(gain[0], 2);
    display.setCursor(50, 27);
    display.println(gain[1], 2);
    display.setCursor(90, 27);
    display.println(gain[2], 4);

    display.setCursor(0, 36);
    if (tuning_mode == 0)
      display.println("Tuning kp");
    else if (tuning_mode == 1)
      display.println("Tuning ki");
    else
      display.println("Tuning kd");

    display.setCursor(0, 45);
    if (invert_mode == 0)
    {
      display.println("Select Invert Mode");
    }
    else if (invert_mode == 1)
    {
      display.println("Side inverted C");
    }
    else if (invert_mode == 2)
    {
      display.println("Side inverted L");
    }
    else if (invert_mode == 3)
    {
      display.println("Side inverted R");
    }
    else if (invert_mode == 4)
    {
      display.println("Point inverted");
    }
    display.print("Invert Mode ");
    display.println(invert_mode);

    //    int LineC = map(theta[0] * 180 / M_PI, 20, -20, 0, 127);
    //    if (LineC >= 0 && LineC < 128) {
    //      if (abs(theta[0]) <= 0.0174) { // 1[°]=0.0174[rad]
    //        display.fillRect(LineC - 2, 16, 5, display.height() - 1, SSD1306_WHITE); //(左上x, 左上y, 幅, 高さ, 線の色
    //      } else {
    //        display.drawLine(LineC, 16, LineC, display.height() - 1, SSD1306_WHITE); //始点x, 始点y, 終点x, 終点y, 線の色
    //      }
    //    }
    //
    //    int LineL = map(theta[1] * 180 / M_PI, 8, -8, 16, 63);
    //    if (LineL >= 16 && LineL < 64) {
    //      if (abs(theta[1]) <= 0.0174) {
    //        display.fillRect(0, LineL - 2 , display.width() / 2 - 1, 5, SSD1306_WHITE);
    //      } else {
    //        display.drawLine(0, LineL, display.width() / 2 - 1, LineL, SSD1306_WHITE);
    //      }
    //    }
    //
    //    int LineR = map(theta[2] * 180 / M_PI, 8, -8, 16, 63);
    //    if (LineR >= 16 && LineR < 64) {
    //      if (abs(theta[2]) <= 0.0174) {
    //        display.fillRect(display.width() / 2, LineR - 2, display.width() - 1, 5, SSD1306_WHITE); //(左上x, 左上y, 幅, 高さ, 線の色
    //      } else {
    //        display.drawLine(display.width() / 2, LineR, display.width() - 1, LineR, SSD1306_WHITE);
    //      }
    //    }

    display.display();
    display.clearDisplay();
  }
}
