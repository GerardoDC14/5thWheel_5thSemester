/*
 * display.h
 *
 *  Created on: Nov 20, 2023
 *      Author: gerar
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include "fonts.h"  //font declarations for the display
#include "ssd1306.h"  //SSD1306 OLED display driver header

// Function Declarations
void UpdateDisplay(int currentPWM, float gx, float gy, float temperature, float speedKmH, float battery_voltage);

#endif /* INC_DISPLAY_H_ */
