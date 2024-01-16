/*
 * utilities.c
 *
 *  Created on: Nov 20, 2023
 *      Author: gerar
 */

#include "utilities.h"

// Function to sound buzzer for a specified duration
void soundBuzzer(uint32_t duration_ms) {
    GPIOA->BSRR = (1 << 1); // Turn on Buzzer
    HAL_Delay(duration_ms); // Delay for duration
    GPIOA->BSRR = (1 << (1 + 16)); // Turn off Buzzer
}

