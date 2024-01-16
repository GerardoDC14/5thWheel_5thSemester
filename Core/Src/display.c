/*
 * display.c
 *
 *  Created on: Nov 20, 2023
 *      Author: gerar
 */
#include "display.h"

/* Update the display values */
void UpdateDisplay(int currentPWM, float gx, float gy,float temperature,float speedKmH,float battery_voltage) {
    char buffer[30];

    // Clear the previous data
    // Display X-Angle label and value
    SSD1306_GotoXY(1, 0);
    SSD1306_Puts("PWM:", &Font_7x10, 1);
    sprintf(buffer, "%d", currentPWM);
    SSD1306_GotoXY(60, 0);
    SSD1306_Puts(buffer, &Font_7x10, 1);

    SSD1306_GotoXY(1, 10);
    SSD1306_Puts("X-Angle:", &Font_7x10, 1);
    sprintf(buffer, "%.2f", gx);
    SSD1306_GotoXY(60, 10);
    SSD1306_Puts(buffer, &Font_7x10, 1);

    // Display Y-Angle label and value
    SSD1306_GotoXY(1, 20);
    SSD1306_Puts("Y-Angle:", &Font_7x10, 1);
    sprintf(buffer, "%.2f", gy);
    SSD1306_GotoXY(60, 20);
    SSD1306_Puts(buffer, &Font_7x10, 1);

    SSD1306_GotoXY(1, 30);
    SSD1306_Puts("Temp:", &Font_7x10, 1);
    sprintf(buffer, "%.2f", temperature);
    SSD1306_GotoXY(40,30 );
    SSD1306_Puts(buffer, &Font_7x10, 1);
    SSD1306_GotoXY(75, 30);
    SSD1306_Puts("C", &Font_7x10, 1);

    SSD1306_GotoXY(1, 40);
    SSD1306_Puts("Km/h:", &Font_7x10, 1);
    sprintf(buffer, "%.2f", speedKmH);
    SSD1306_GotoXY(60, 40);
    SSD1306_Puts(buffer, &Font_7x10, 1);

    SSD1306_GotoXY(1, 50);
    SSD1306_Puts("Voltage", &Font_7x10, 1);
    sprintf(buffer, "%.2f", battery_voltage);
    SSD1306_GotoXY(60, 50);
    SSD1306_Puts(buffer, &Font_7x10, 1);
    // Update the screen
    SSD1306_UpdateScreen();
}

