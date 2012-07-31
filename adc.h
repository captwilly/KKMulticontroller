/*
 * adc.h
 *
 *  Created on: Aug 1, 2012
 *      Author: dart
 */

#ifndef ADC_H_
#define ADC_H_

#include "common.h"

void adc_init(void);
uint16_t adc_read(uint8_t channel);


#endif /* ADC_H_ */
