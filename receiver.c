#include "receiver.h"

#include "settings.h"
#include <stdlib.h>
#include <util/atomic.h>
#include <util/delay.h>

/*** BEGIN VARIABLES ***/
static volatile uint16_t RxChannel1;
static volatile uint16_t RxChannel2;
static volatile uint16_t RxChannel3;
static volatile uint16_t RxChannel4;

static struct SETTINGS_S settings;
/*** END VARIABLES ***/

/*** BEGIN RECEIVER INTERRUPTS ***/
ISR(PCINT2_vect) {
    static volatile uint16_t RxChannel1Start;
    if (RX_ROLL) { // rising
        RxChannel1Start = TCNT1;
    } else { // falling
        RxChannel1 = TCNT1 - RxChannel1Start;
    }
}

ISR(INT0_vect) {
    static volatile uint16_t RxChannel2Start;
    if (RX_PITCH) { // rising
        RxChannel2Start = TCNT1;
    } else { // falling
        RxChannel2 = TCNT1 - RxChannel2Start;
    }
}

ISR(INT1_vect) {
    static volatile uint16_t RxChannel3Start;
    if (RX_COLL) { // rising
        RxChannel3Start = TCNT1;
    } else { // falling
        RxChannel3 = TCNT1 - RxChannel3Start;
    }
}

ISR(PCINT0_vect) {
    static volatile uint16_t RxChannel4Start;
    if (RX_YAW) { // rising
        RxChannel4Start = TCNT1;
    } else { // falling
        RxChannel4 = TCNT1 - RxChannel4Start;
    }
}
/*** END RECEIVER INTERRUPTS ***/

void receiverSetup() {
    RX_ROLL_DIR =   INPUT;
    RX_PITCH_DIR =  INPUT;
    RX_COLL_DIR =   INPUT;
    RX_YAW_DIR =    INPUT;

    RX_ROLL =       0;
    RX_PITCH =      0;
    RX_COLL =       0;
    RX_YAW =        0;

    /*
     * timer1 (16bit) - run at 8MHz (prescaler 1), used to measure Rx pulses
     * and to control ESC/servo pulse
     */TCCR1B = _BV(CS10);

    /*
     * Enable Rx pin interrupts
     */
    PCICR = _BV(PCIE0) | _BV(PCIE2); // PCINT0..7, PCINT16..23 enable
    PCMSK0 = _BV(PCINT7); // PB7
    PCMSK2 = _BV(PCINT17); // PD1
    EICRA = _BV(ISC00) | _BV(ISC10); // Any change INT0, INT1
    EIMSK = _BV(INT0) | _BV(INT1); // External Interrupt Mask Register

    settingsRead(&settings);
}

/*
 * Copy, scale, and offset the Rx inputs from the interrupt-modified
 * variables.
 * [1000 to 2000] microseconds pulse corresponds [-500 to 500] result.
 */
static void receiverGetChannelsClean(struct RX_STATE_S *state) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state->roll = RxChannel1;
        state->pitch = RxChannel2;
        state->collective = RxChannel3;
        state->yaw = RxChannel4;
    }

    /* Divide by 8 (8MHz timer), offset by 1500 (1000us constant pulse, 500
     *  offset to zero) */
    state->roll = (state->roll >> 3) - 1500;
    state->pitch = (state->pitch >> 3) - 1500;
    state->yaw = (state->yaw >> 3) - 1500;
    state->collective = (state->collective >> 3) - 1500;
#ifdef TWIN_COPTER
    state->orgPitch = state->pitch;
#endif
}

void receiverGetChannels(struct RX_STATE_S *state) {
    receiverGetChannelsClean(state);
    state->roll -= settings.RxRollZero;
    state->pitch -= settings.RxPitchZero;
    state->yaw -= settings.RxYawZero;
}

void receiverStickCenterManual(void) {
    struct RX_STATE_S rxState;
    uint8_t i;
    while (true) {
        receiverGetChannelsClean(&rxState);
        i = abs(rxState.roll) + abs(rxState.pitch) + abs(rxState.yaw);
        i = i / 4;
        i = i >= 25 ? 25 : i;
        LED = 0;
        _delay_ms(25 - i);
        LED = 1;
        _delay_ms(i);
    }
}

void receiverStickCenterAutomatic(void) {
    struct RX_STATE_S rx;
    receiverGetChannelsClean(&rx);
    settingsRead(&settings);

    settings.RxRollZero = rx.roll;
    settings.RxPitchZero = rx.pitch;
    settings.RxYawZero = rx.yaw;

    settingsWrite(&settings);

    while (true) {
        LED = 0;
        _delay_ms(100);
        LED = 1;
        _delay_ms(100);
    }
}
