#ifndef _DISPLAY_H_
#define _DISPLAY_H_
#include "main.hpp"
#include "I2Cdev.hpp"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

void DispInit();
void Disp(void *pvParameters);

#endif // _DISPLAY_H_
