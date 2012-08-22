/*
 * Filename:    timer.c
 * Description: this module provides 32bit timer using 16 bit hardware
 *  implementation Timer1. Module Also initializes additional timer (Timer2) and
 *  provides define (T2_FREQ) for its frequency.
 *
 *  Created on: Aug 21, 2012
 *      Author: dart
 */
#include "timer.h"

#include <avr/interrupt.h>
#include <util/atomic.h>


/******************************************************************************
 ***    Module variables                                                    ***
 ******************************************************************************/
static volatile uint16_t timer1_msb = 0;


/******************************************************************************
 ***    Interrupt service routines                                          ***
 ******************************************************************************/
ISR(TIMER1_OVF_vect) {
    timer1_msb++;
}


/******************************************************************************
 ***    Public functions                                                    ***
 ******************************************************************************/
void timerInit(void){
    // Clear interrupt flag (just for case)
    TIFR1 |= _BV(TOV1);
    // Enable overflow interrupt
    TIMSK1 |= _BV(TOIE1);
#if T1_FREQ == F_CPU
    TCCR1B = _BV(CS10);
#else
#error "Unsupported Timer1 configuration"
#endif

#if T2_FREQ == F_CPU / 1024
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
#else
#error "Unsupported Timer2 configuration"
#endif
}


// Get current time
uint32_t timerGetTime(void) {
    uint16_t msb;
    uint16_t tcnt1;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        tcnt1 = TCNT1;
        /* For the case when timer have already overflowed but corresponding ISR
         *  had no time to increment "timer1_msb" variable*/
        if((TIFR1 & _BV(TOV1)) && tcnt1 < T1_FREQ * 20 / F_CPU) {
            msb = timer1_msb + 1;
        } else {
            msb = timer1_msb;
        }
    }
    return tcnt1 + ((uint32_t)msb << 16);
}

// Same as above but can only be called within interrupt context
uint32_t timerGetTimeUnsafe(void) {
    uint16_t msb;
    uint16_t tcnt1;

//    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        tcnt1 = TCNT1;
        if((TIFR1 & _BV(TOV1)) && tcnt1 < T1_FREQ * 20 / F_CPU) {
            msb = timer1_msb + 1;
        } else {
            msb = timer1_msb;
        }
//    }
    return tcnt1 + ((uint32_t)msb << 16);
}
