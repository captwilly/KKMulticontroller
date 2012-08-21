/*
 * timer.c
 *
 *  Created on: Aug 21, 2012
 *      Author: dart
 */
#include "timer.h"

#include <avr/interrupt.h>
#include <util/atomic.h>

static uint8_t timer1_msb = 0;

void timerInit(void){
    // TODO: remove timer0 initialization (make sure timer is not used anywhere)
#if T0_FREQ == F_CPU
    TCCR0B = _BV(CS00); /* NOTE: Specified again below with FOC0x bits */
#else
#error "Unsupported timer0 configuration"
#endif


    // Clear interrupt flag (just for case)
    TIFR1 |= _BV(TOV1);
    // Enable overflow interrupt
    TIMSK1 |= _BV(TOIE1);
#if T1_FREQ == F_CPU
    TCCR1B = _BV(CS10);
#else
#error "Unsupported timer1 configuration"
#endif

#if T2_FREQ == F_CPU / 1024
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
#else
#error "Unsupported timer2 configuration"
#endif
}

ISR(TIMER1_OVF_vect) {
    timer1_msb++;
}

uint32_t timerGetTime(void) {
    uint16_t tcnt1;
    uint8_t msb;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        tcnt1 = TCNT1;
        if((TIFR1 & _BV(TOV1)) && tcnt1 < T1_FREQ * 20 / F_CPU) {
            msb = timer1_msb + 1;
        } else {
            msb = timer1_msb;
        }
    }
    return tcnt1 + ((uint32_t)msb << 16);
}
