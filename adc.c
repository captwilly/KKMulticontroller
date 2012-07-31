/*
 * adc.c
 *
 *  Created on: Aug 1, 2012
 *      Author: dart
 */
#include "adc.h"

void adc_init(void) {
    // Digital Input Disable Register - ADC5..0 Digital Input Disable
    DIDR0 = ADC_CHANNELS_MASK;
    // ADC Control and Status Register B - ADTS2:0
    ADCSRB = 0;
}

uint16_t adc_read(uint8_t channel) {
    ADMUX = channel; // set channel
    ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADPS1) | _BV(ADPS2); // 0b11000110

    // wait to complete
    while (ADCSRA & _BV(ADSC));

    return ADCW;
}
