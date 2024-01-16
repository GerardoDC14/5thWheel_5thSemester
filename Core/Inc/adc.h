/*
 * adc.h
 *
 *  Created on: Nov 20, 2023
 *      Author: gerar
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdint.h>

// Constants and Macros
#define V_REF 3.3
#define ADC_RESOLUTION 4096.0
#define LM35_SCALE_FACTOR 0.01
#define VOLTAGE_DIVIDER_RATIO (1.2/15.0)
#define ACS712_SENSITIVITY 0.1
#define ACS712_ZERO_CURRENT_OFFSET 2.5

// Function Declarations
void init_ADC(void);
uint16_t read_ADC_Channel(uint8_t channel);
float ADC_to_Temperature(uint16_t adc_value);
float ADC_to_BatteryVoltage(uint16_t adc_value);
float ADC_to_Current(uint16_t adc_value);


#endif /* INC_ADC_H_ */
