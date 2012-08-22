/*
 * Filename: att_sensor.c
 * Description: Ultrasonic attitude sensor module. Used "receiver" module to get
 *  ISR's and "timer" module to measure pulses length.
 *
 *  Created on: Aug 22, 2012
 *      Author: dart
 */
#include "att_sensor.h"

// Don'n compile anything unless attitude sensor support is enabled
#ifdef ATTITUDE_SENSOR

#include "timer.h"
#include <util/atomic.h>
#include <util/delay.h>


/******************************************************************************
 ***    Defines                                                             ***
 ******************************************************************************/
// 340290 (speed of sound in mm/s) / F_CPU (timer1 clock) / 2 (two-way travel)
#define SOUND_SPEED_FACTOR  0.021268125

// 4 meters maximum measurable attitude
// TODO: rework this using "TIMER_FREQ" constant
#define MAX_DIFF            188075UL


/******************************************************************************
 ***    Module variables                                                    ***
 ******************************************************************************/
static volatile uint32_t distance;


/******************************************************************************
 ***    Interrupt service routines                                          ***
 ******************************************************************************/
// Call only under disabled interrupts
void attISR(bool state) {
    static uint32_t last_time;

    if(state) {
        // Rising edge - start counting time
        last_time = timerGetTimeUnsafe();
    } else {
        // Falling edge - get difference (pulse length)
        uint32_t curr_time = timerGetTimeUnsafe();
        distance = curr_time - last_time;
    }
}


/******************************************************************************
 ***    Public functions                                                    ***
 ******************************************************************************/
void attInit(void) {
    ATT_TRIG_DIR = OUTPUT;
    ATT_ECHO_DIR = INPUT;

    /*
     * Initialize Timer0 to continuously trigger attitude sensor
     *  (128us pulse every 32ms - 31.25Hz)
     */
    // Fast PWM mode, generate non-inverting output on OC0A
    TCCR0A = _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
    // Single-clock (128us) positive pulse at each overflow
    OCR0A = 1;
#if T0_FREQ == F_CPU / 1024
    // Clock is F_CPU / 1024
    TCCR0B = _BV(CS02) | _BV(CS00);
#else
#error "Unsupported Timer0 configuration"
#endif
}

// Returns measured distance in millimeters
uint16_t attGetDistance(void) {
    static uint32_t last_dist;
    uint32_t curr_dist;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        curr_dist = distance;
    }

    /* In case there's no reflected signal detected by the sensor pulse time
     *  will be too long. So try to protect from that using maximum allowed
     *  distance. In case of bad reading last good one will be returned */
    if (curr_dist < MAX_DIFF) {
        last_dist = curr_dist;
    }

    return last_dist * SOUND_SPEED_FACTOR;
}

#endif // ATTITUDE_SENSOR
