/*
 * att_sensor.c
 *
 *  Created on: Aug 22, 2012
 *      Author: dart
 */
#include "att_sensor.h"

#include "timer.h"
#include <util/atomic.h>

#include "led.h"
#include "debug.h"

// 340290 (speed of sound in mm/s) / F_CPU (timer1 clock) / 2 (two-way travel)
#define SOUND_SPEED_FACTOR 0.021268125


static volatile uint32_t distance;


// Call only under disabled interrupts
void attISR(bool state) {
    static uint32_t last_time;
    if(state) {
        last_time = timerGetTimeUnsafe();
    } else {
        uint32_t curr_time = timerGetTimeUnsafe();
        distance = curr_time - last_time;
        if (distance > 4702) {
            struct {
                uint32_t last;
                uint32_t curr;
            } tmp = {
                    .last = last_time,
                    .curr = curr_time,
            };
            DEBUG(tmp);
        }
    }
}

uint16_t attGetDistance(void) {
    uint16_t retval;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        retval = distance;
    }
    return retval * SOUND_SPEED_FACTOR;
}
