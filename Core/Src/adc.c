/*
 * adc.c
 *
 *  Created on: Nov 20, 2023
 *      Author: gerar
 */
#include "adc.h"
#include "stm32f051x8.h"
// Initialize ADC
void init_ADC(void) {
    RCC->APB2ENR |= (1 << 9); // Enable ADC clock
    ADC1->CHSELR |= (1 << 10) | (1 << 11) | (1 << 12); // Select channels 10, 11 and 12 for ADC
    ADC1->CR |= (1 << 0); // Enable ADC
    while(!(ADC1->ISR & (1 << 0))); // Wait for ADC to be ready
}

// Read a specific ADC channel
uint16_t read_ADC_Channel(uint8_t channel) {
    ADC1->CHSELR = (1 << channel); // Select desired channel
    ADC1->CR |= (1 << 2); // Start ADC conversion
    while(!(ADC1->ISR & (1 << 2))); // Wait for conversion to complete
    return ADC1->DR; // Return ADC result
}

// Convert ADC value to temperature using LM35
float ADC_to_Temperature(uint16_t adc_value) {
    float voltage = (adc_value * V_REF) / ADC_RESOLUTION; // Convert ADC value to voltage
    return voltage / LM35_SCALE_FACTOR; // Convert voltage to temperature
}

// Convert ADC value to battery voltage
float ADC_to_BatteryVoltage(uint16_t adc_value) {
    float voltage = (adc_value * V_REF) / ADC_RESOLUTION; // Convert ADC value to voltage
    return voltage / VOLTAGE_DIVIDER_RATIO; // Adjust for voltage divider
}

// Convert ADC value to current using ACS712 sensor
float ADC_to_Current(uint16_t adc_value) {
    float voltage = (adc_value * V_REF) / ADC_RESOLUTION; // Convert ADC value to voltage
    return (voltage - ACS712_ZERO_CURRENT_OFFSET) / ACS712_SENSITIVITY; // Convert to current
}



