//
// Created by Tony Manschula on 3/23/22.
//

#ifndef LAB8_ADC_H
#define LAB8_ADC_H
#include "stdint.h"
#include "tm4c123gh6pm.h"

void adc_init(void);

uint16_t adc_read(void);

void adc_print(uint32_t irVal, double dist);

double adc_getDistance(uint32_t irVal);

#endif //LAB8_ADC_H
