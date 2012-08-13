#include "motors.h"

#include "receiver.h"
#include <util/delay.h>
#include <util/atomic.h>

// It's 80 CPU tacts - should be enough for IRQ handler to complete
#define MIN_DIST    10

#define ESC_PERIOD  F_CPU / ESC_RATE
#if ESC_RATE >= 500
#error "ESC rate is too high (protocol restriction)"
#endif
#if ESC_PERIOD >= (1 << 16)
#error "ESC rate is too low (timer width and prescaler restriction)"
#endif

#if M5_USED && M6_USED
#define MOTOR_COUNT 6
#elif M5_USED || M6_USED
#define MOTOR_COUNT 5
#else
#define MOTOR_COUNT 4
#endif

static struct {
    uint16_t offset;
    uint8_t number;
} motors_list[MOTOR_COUNT - 1];
static uint16_t motor_next;


#if defined(SINGLE_COPTER) \
    || defined(DUAL_COPTER) \
    || defined(TWIN_COPTER) \
    || defined(TRI_COPTER)
uint8_t servo_skip;
uint16_t servo_skip_divider;
#endif

static volatile bool motorReady = true;

void motorsSetup() {
    M1_DIR = OUTPUT;
    M2_DIR = OUTPUT;
    M3_DIR = OUTPUT;
    M4_DIR = OUTPUT;
    M5_DIR = OUTPUT;
    M6_DIR = OUTPUT;
    M1 = 0;
    M2 = 0;
    M3 = 0;
    M4 = 0;
    M5 = 0;
    M6 = 0;

    /*
     * timer0 (8bit) - run at 8MHz, used to control ESC pulses
     * We use 8Mhz instead of 1MHz (1 usec) to avoid alignment jitter.
     */
    TCCR0B = _BV(CS00); /* NOTE: Specified again below with FOC0x bits */

#if defined(SINGLE_COPTER) \
    || defined(DUAL_COPTER) \
    || defined(TWIN_COPTER) \
    || defined(TRI_COPTER)
    /*
     * Calculate the servo rate divider (pulse loop skip count
     * needed to avoid burning analog servos)
     */
    for(servo_skip_divider = 1;;servo_skip_divider++) {
        if(servo_skip_divider * SERVO_RATE >= ESC_RATE) {
            break;
        }
    }
#endif
}

void motorOutputPPM(struct MT_STATE_S *state){
    // Ensure all values are in their ranges
    state->m1out = MIN(1000, state->m1out);
    state->m1out = MAX(0, state->m1out);
    state->m2out = MIN(1000, state->m2out);
    state->m2out = MAX(0, state->m2out);
    state->m3out = MIN(1000, state->m3out);
    state->m3out = MAX(0, state->m3out);
    state->m4out = MIN(1000, state->m4out);
    state->m4out = MAX(0, state->m4out);
#if M5_USED
    state->m5out = MIN(1000, state->m5out);
    state->m5out = MAX(0, state->m5out);
#endif
#if M6_USED
    state->m6out = MIN(1000, state->m6out);
    state->m6out = MAX(0, state->m6out);
#endif

    // Temporary array used for sorting
    uint16_t sort_tmp[MOTOR_COUNT];
    sort_tmp[0] = state->m1out;
    sort_tmp[1] = state->m2out;
    sort_tmp[2] = state->m3out;
    sort_tmp[3] = state->m4out;
#if M5_USED
    sort_tmp[4] = state->m5out;
#endif
#if M6_USED
    sort_tmp[5] = state->m6out;
#endif

    uint8_t sort_result[MOTOR_COUNT] = {0, 1, 2, 3,
#if MOTOR_COUNT == 5
            5
#endif
#if MOTOR_COUNT == 6
            5, 6
#endif
    };

    // Selection sort
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        uint8_t max = i;
        for (uint8_t j = i + 1; j < MOTOR_COUNT; j++) {
            if (sort_tmp[sort_result[j]] > sort_tmp[sort_result[max]]) {
                max = j;
            }
        }
        if (i != max) {
            uint8_t tmp = sort_result[i];
            sort_result[i] = sort_result[max];
            sort_result[max] = tmp;
        }
    }


    for (uint8_t i = 0; i < MOTOR_COUNT - 1; i++) {
        motors_list[i].number = sort_result[i];
        if (sort_result[i] - MIN_DIST >= sort_result[i + 1]) {
            motors_list[i].offset = 1000 * 8 + sort_tmp[sort_result[i]] * 8;
        } else {
            motors_list[i].offset = 0;
            sort_tmp[i + 1] = (sort_tmp[i] + sort_tmp[i + 1]) / 2;
        }
    }
    // Last index in array
    motor_next = MOTOR_COUNT - 2;


    // Wait previous output to finish
    while(!motorReady);
    motorReady = false;

    M1 = 1;
    M2 = 1;
    M3 = 1;
    M4 = 1;
    M5 = 1;
    M6 = 1;

    uint16_t curr_cnt;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        curr_cnt = TCNT1;
    }

    uint16_t pulse_delay = curr_cnt + 1000 * 8 + (state->m1out << 3);
    uint16_t pause_delay = curr_cnt + ESC_PERIOD;

    for(uint8_t i = 0; i < MOTOR_COUNT - 1; i++) {
        if (motors_list[i].offset != 0) {
            motors_list[i].offset += curr_cnt;
        }
    }

    // Clear interrupt flags
    TIFR1 = _BV(OCF1A) | _BV(OCF1B);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        OCR1A = pulse_delay;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        OCR1B = pause_delay;
    }
    TIMSK1 |= _BV(OCIE1A) | _BV(OCIE1B);
}

ISR(TIMER1_COMPA_vect) {
    /* Loop through motor list (from highest index set in motorOutputPPM to
     * zero) */
    for (;; motor_next--) {
        // Decide which line should go down now
        switch (motors_list[motor_next].number) {
        case 0:
            M1 = 0;
            break;
        case 1:
            M2 = 0;
            break;
        case 2:
            M3 = 0;
#if !M5_USED
            M5 = 0;
#endif
            break;
        case 3:
            M4 = 0;
#if !M6_USED
            M6 = 0;
#endif
            break;
#if M5_USED
        case 4:
            M5 = 0;
            break;
#endif
#if M5_USED
        case 5:
            M6 = 0;
            break;
#endif
        }
        if (motor_next == 0) {
            // We are on finish, disable further interrupts
            TIMSK1 &= ~_BV(OCIE1A);
            break;
        }
        if (motors_list[motor_next - 1].offset != 0) {
            // Normal case - next line will be release in some time later
            OCR1A = motors_list[motor_next].offset;
            // Need to decrement manually as we are breaking the loop
            motor_next--;
            break;
        }
        // Otherwise go and release other line
    }
}

ISR(TIMER1_COMPB_vect) {
    /* This happens when pause required to care ESC_RATE is finished.
     *  Indicate readiness for the next cycle and disable further interrupts */
    TIMSK1 &= ~_BV(OCIE1B);
    motorReady = true;
}

#if 0
void motorOutputPPM(struct MT_STATE_S *state) {
    static int16_t MotorStartTCNT1;
    int16_t t;

    /*
     * Bound pulse length to 1ms <= pulse <= 2ms.
     */

    t = 1000;
    if (state->m1out < 0)
        state->m1out = 0;
    else if (state->m1out > t)
        state->m1out = t;
#ifdef SINGLE_COPTER
    t = 2000;
#endif
    if (state->m2out < 0)
        state->m2out = 0;
    else if (state->m2out > t)
        state->m2out = t;
    if (state->m3out < 0)
        state->m3out = 0;
    else if (state->m3out > t)
        state->m3out = t;
    if (state->m4out < 0)
        state->m4out = 0;
    else if (state->m4out > t)
        state->m4out = t;
#if M5_USED
    if(state->m5out < 0)
        state->m5out = 0;
    else if(state->m5out > t)
        state->m5out = t;
#endif
#if M6_USED
    if(state->m6out < 0)
        state->m6out = 0;
    else if(state->m6out > t)
        state->m6out = t;
#endif

    t = 1000;
    state->m1out += t;
#ifndef SINGLE_COPTER
    state->m2out += t;
    state->m3out += t;
    state->m4out += t;
#if M5_USED
    state->m5out+= t;
#endif
#if M6_USED
    state->m6out+= t;
#endif
#endif

    state->m1out <<= 3;
    state->m2out <<= 3;
    state->m3out <<= 3;
    state->m4out <<= 3;
#if M5_USED
    state->m5out<<= 3;
#endif
#if M6_USED
    state->m6out<<= 3;
#endif

    /*
     * We can use timer compare output mode to provide jitter-free
     * PPM output on M1, M2, M5 and M6 by using OC0A and OC0B from
     * timer 0 (8-bit) and OC1A and OC1B from timer 1 (16-bit) to
     * turn off the pins. Since we are counting in steps of 1us and
     * need to wait up to 2ms, we need to delay the turn-on of the
     * 8-bit pins to avoid early triggering.
     *
     * Once entering compare match output mode, we cannot directly
     * set the pins. We can use the "force output compare" (which
     * doesn't actually force a compare but pretends the comparison
     * was true) to fiddle output high or low, but this would still
     * have interrupt and instruction-timing-induced jitter. Instead,
     * we just set the next desired switch state and set the OCRnx
     * registers to a known time in the future. The 8-bit ones will
     * set the pin the same way several times, so we have to make
     * sure that we don't change the high/low mode too early.
     *
     * Hardware PPM (timer compare output mode) pin mapping:
     *
     * M1 (PB2): OCR1B (COM1B) 16-bit
     * M2 (PB1): OCR1A (COM1A) 16-bit
     * M3 (PB0): software only
     * M4 (PD7): software only
     * M5 (PD6): OCR0A (COM0A) 8-bit
     * M6 (PD5): OCR0B (COM0B) 8-bit
     *
     * We must disable interrupts while setting the 16-bit registers
     * to avoid Rx interrupts clobbering the internal temporary
     * register for the associated 16-bit timer. 8 cycles is one
     * microsecond at 8 MHz, so we try not to leave interrupts
     * disabled for more than 8 cycles.
     *
     * We turn OFF the pins here, then wait for the ON cycle start.
     */
    t = MotorStartTCNT1 + state->m1out;
    asm(""::"r" (t)); // Avoid reordering of add after cli
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        OCR1B = t;
    }
    t = MotorStartTCNT1 + state->m2out;
    asm(""::"r" (t)); // Avoid reordering of add after cli
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        OCR1A = t;
    }
    TCCR1A = _BV(COM1A1) | _BV(COM1B1); /* Next match will clear pins */

    /*
     * Only 8 bits will make it to the OCR0x registers, so leave the
     * mode as setting pins ON here and then change to OFF mode after
     * the last wrap before the actual time.
     *
     * We hope that TCNT0 and TCNT1 are always synchronized.
     */
#if M5_USED
    OCR0A = MotorStartTCNT1 + state->m5out;
#else
    OCR0A = MotorStartTCNT1 + state->m3out;
#endif
#if M6_USED
    OCR0B = MotorStartTCNT1 + state->m6out;
#else
    OCR0B = MotorStartTCNT1 + state->m4out;
#endif

    do {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            t = TCNT1;
        }
        t -= MotorStartTCNT1;
        if (t >= state->m3out)
            M3 = 0;
        if (t >= state->m4out)
            M4 = 0;
        if (t + 0xff >=
#if M5_USED
                state->m5out
#else
                state->m3out
#endif
        )
            TCCR0A &= ~_BV(COM0A0); /* Clear pin on match */
        if (t + 0xff >=
#if M6_USED
                state->m6out
#else
                state->m4out
#endif
        )
            TCCR0A &= ~_BV(COM0B0); /* Clear pin on match */
        t -= ((2000 + PWM_LOW_PULSE_US) << 3) - 0xff;
    } while (t < 0);

    /*
     * We should now be <= 0xff ticks before the next on cycle.
     *
     * Set up the timer compare values, wait for the on time, then
     * turn on software pins. We hope that we will be called again
     * within 1ms so that we can turn them off again in time.
     *
     * Timer compare output mode must stay enabled, and disables
     * regular output when enabled. The value of the COMnx0 bits set
     * the pin high or low when the timer value matches the OCRnx
     * value, or immediately when forced with the FOCnx bits.
     */

    MotorStartTCNT1 += (2000 + PWM_LOW_PULSE_US) << 3;
#if 0
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        t = TCNT1;
    }
    t+= 0x3f;
    t-= MotorStartTCNT1;
    if(t >= 0) {
        /*
         * We've already passed the on cycle, hmm.
         * Push it into the future.
         */
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            t = TCNT1;
        }
        MotorStartTCNT1 = t + 0xff;
    }
#endif
    t = MotorStartTCNT1;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        OCR1B = t;
    }
    OCR0A = t;
    OCR0B = t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        OCR1A = t;
    }

#ifdef SINGLE_COPTER
    if(servo_skip == 0) {
        TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
        //    TCCR1C = _BV(FOC1A) | _BV(FOC1B);
        TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0);
        //    TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
    } else {
        TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0);
        //    TCCR1C = _BV(FOC1A) | _BV(FOC1B);
    }
#elif defined(DUAL_COPTER) || defined(TWIN_COPTER)
    TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
    //  TCCR1C = _BV(FOC1A) | _BV(FOC1B);
    if(servo_skip == 0) {
        TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0);
        //    TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
    }
#elif defined(TRI_COPTER)
    TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
    //  TCCR1C = _BV(FOC1A) | _BV(FOC1B);
    if(servo_skip == 0) {
        TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0);
        //    TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
    } else {
        TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1);
        //    TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
    }
#elif defined(QUAD_COPTER) \
    || defined(QUAD_X_COPTER) \
    || defined(Y4_COPTER) \
    || defined(HEX_COPTER) \
    || defined(Y6_COPTER)
    TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
    //  TCCR1C = _BV(FOC1A) | _BV(FOC1B);
    TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0);
    //  TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
#endif

    /*
     * Wait for the on time so we can turn on the software pins.
     */
    do {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            t = TCNT1;
        }
        t -= MotorStartTCNT1;
    } while (t < 0);

#ifdef SINGLE_COPTER
    if(servo_skip == 0) {
        M3 = 1;
        M4 = 1;
        servo_skip = servo_skip_divider;
    }
    servo_skip--;
#elif defined(DUAL_COPTER) || defined(TWIN_COPTER)
    if(servo_skip == 0) {
        M3 = 1;
        M4 = 1;
        servo_skip = servo_skip_divider;
    }
    servo_skip--;
#elif defined(TRI_COPTER)
    M3 = 1;
    if(servo_skip == 0) {
        M4 = 1;
        servo_skip = servo_skip_divider;
    }
    servo_skip--;
#elif defined(QUAD_COPTER) \
    || defined(QUAD_X_COPTER) \
    || defined(Y4_COPTER) \
    || defined(HEX_COPTER) \
    || defined(Y6_COPTER)
    M3 = 1;
    M4 = 1;
#endif
    /*
     * We leave with the output pins ON.
     */
}
#endif

void motorsIdentify() {
    LED = 0;
    int8_t motor = 0;
    uint16_t delay = 0;
    uint16_t time = TCNT2;
    bool escInit = true; // Wait until the ESCs have initialized
    struct MT_STATE_S motors;

    while (true) {
        delay += (uint8_t) (TCNT2 - time);
        time = TCNT2;

        if (escInit) {
            if (delay > 23437) { // 3.00 second delay (3.00 / .000128 = 23437.5)
                escInit = false;
                delay = 0;
            }
        } else if (LED) {
            if (delay > 1171) { // 0.15 second delay (0.15 / .000128 = 1171.8)
                if (++motor > 6) {
                    motor = 0;
                }
                delay = 0;
                LED = !LED;
            }
        } else {
            if (delay > 7812) { // 1.00 second delay (1.00 / .000128 = 7812.5)
                delay = 0;
                LED = !LED;
            }
        }

        motors.m1out = 0;
        motors.m2out = 0;
        motors.m3out = 0;
        motors.m4out = 0;
#if M5_USED
        motors.m5out = 0;
#endif
#if M6_USED
        motors.m6out = 0;
#endif

        if (LED) {
            if (motor == 1) {
                motors.m1out = MOTOR_LOWEST_VALUE;
            }
            if (motor == 2) {
                motors.m2out = MOTOR_LOWEST_VALUE;
            }
            if (motor == 3) {
                motors.m3out = MOTOR_LOWEST_VALUE;
            }
            if (motor == 4) {
                motors.m4out = MOTOR_LOWEST_VALUE;
            }
#if M5_USED
            if (motor == 5) {
                motors.m5out = MOTOR_LOWEST_VALUE;
            }
#endif
#if M6_USED
            if (motor == 6) {
                motors.m6out = MOTOR_LOWEST_VALUE;
            }
#endif
        }

        motorOutputPPM(&motors);
    }
}

void motorsThrottleCalibration() {
    struct MT_STATE_S motors;
    struct RX_STATE_S rxState;

    // flash LED 3 times
    for (uint8_t i = 0; i < 3; i++) {
        LED = 1;
        _delay_ms(25);
        LED = 0;
        _delay_ms(25);
    }

    while (true) {
        receiverGetChannels(&rxState);
#ifdef SINGLE_COPTER
        motors.m1out = rxState.collective;
        motors.m2out = 1400; // Center: 140
        motors.m3out = 1400;
        motors.m4out = 1400;
        motors.m5out = 1400;
#elif defined(DUAL_COPTER)
        motors.m1out = rxState.collective;
        motors.m2out = rxState.collective;
        motors.m3out = 500; // Center: 50
        motors.m4out = 500;
#elif defined(TWIN_COPTER)
        motors.m1out = rxState.collective;
        motors.m2out = rxState.collective;
        motors.m3out = 500; // Center: 50
        motors.m4out = 500;
        motors.m5out = 500;
        motors.m6out = 500; // Center: 50, Reverse
#elif defined(TRI_COPTER)
        motors.m1out = rxState.collective;
        motors.m2out = rxState.collective;
        motors.m3out = rxState.collective;
        motors.m4out = 500 + rxState.yaw * 2; // Center: 50
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
        motors.m1out = rxState.collective;
        motors.m2out = rxState.collective;
        motors.m3out = rxState.collective;
        motors.m4out = rxState.collective;
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
        motors.m1out = rxState.collective;
        motors.m2out = rxState.collective;
        motors.m3out = rxState.collective;
        motors.m4out = rxState.collective;
        motors.m5out = rxState.collective;
        motors.m6out = rxState.collective;
#else
#error No Copter configuration defined !!!!
#endif
        // this regulates rate at which we output signals
        motorOutputPPM(&motors);
    }
}
