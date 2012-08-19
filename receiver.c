#include "receiver.h"

#include "settings.h"
#include "led.h"
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
// NOTE: we can save average 4 tacts rewriting those in assembly
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

static int16_t receiverScale(int16_t value, int16_t min, int16_t max){
    /* Calculate value per mil (in 1/1000th),
     * cast to S32 so product of multiplying by 1000 will fit into variable */
    return  ((int32_t)(value - min) * 1000) / (max - min);
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

    /*
     * Scale
     */
    state->roll = receiverScale(state->roll,
            settings.RxRollMin, settings.RxRollMax);
    state->pitch = receiverScale(state->pitch,
            settings.RxPitchMin, settings.RxPitchMax);
    state->yaw = receiverScale(state->yaw,
            settings.RxYawMin, settings.RxYawMax);
    state->collective = receiverScale(state->collective,
            settings.RxCollectiveMin, settings.RxCollectiveMax);

    // Apply zero offset
    state->roll -= settings.RxRollZero;
    state->pitch -= settings.RxPitchZero;
    state->yaw -= settings.RxYawZero;
    state->collective -= settings.RxCollectiveZero;
}

void receiverStickCenterManual(void) {
    struct RX_STATE_S rxState;
    uint8_t i;
    FOREVER {
        receiverGetChannelsClean(&rxState);
        i = abs(rxState.roll) + abs(rxState.pitch) + abs(rxState.yaw);
        i = i / 4;
        i = i >= 25 ? 25 : i;
        LED_ON();
        _delay_ms(25 - i);
        LED_OFF();
        _delay_ms(i);
    }
}

void receiverStickCenterAutomatic(void) {
    struct RX_STATE_S rx;
    settingsRead(&settings);

    receiverGetChannelsClean(&rx);

    // This will be final value for zero offset
    settings.RxRollZero = rx.roll;
    settings.RxPitchZero = rx.pitch;
    settings.RxYawZero = rx.yaw;
    settings.RxCollectiveZero = rx.collective;

    // Initial value for max and min
    settings.RxRollMax = settings.RxRollZero;
    settings.RxPitchMax = settings.RxPitchZero;
    settings.RxYawMax = settings.RxYawZero;
    settings.RxCollectiveMax = settings.RxCollectiveZero;
    settings.RxRollMin = settings.RxRollZero;
    settings.RxPitchMin = settings.RxPitchZero;
    settings.RxYawMin = settings.RxYawZero;
    settings.RxCollectiveMin = settings.RxCollectiveZero;


    uint8_t finish_counter = 0;
    FOREVER {
        receiverGetChannelsClean(&rx);

        // Find minimum and maximum
        settings.RxRollMin = MIN(rx.roll, settings.RxRollMin);
        settings.RxRollMax = MAX(rx.roll, settings.RxRollMax);
        settings.RxPitchMin = MIN(rx.pitch, settings.RxPitchMin);
        settings.RxPitchMax = MAX(rx.pitch, settings.RxPitchMax);
        settings.RxYawMin = MIN(rx.yaw, settings.RxYawMin);
        settings.RxYawMax = MAX(rx.yaw, settings.RxYawMax);
        settings.RxCollectiveMin = MIN(rx.collective, settings.RxCollectiveMin);
        settings.RxCollectiveMax = MAX(rx.collective, settings.RxCollectiveMax);

        /* Check if all sticks have returned back to zero position
         * (5% tolerance) for at least 2.4 seconds than we are done */
        if (false
                || rx.roll - settings.RxRollZero > 50
                || rx.roll - settings.RxRollZero < -50
                || rx.pitch - settings.RxPitchZero > 50
                || rx.pitch - settings.RxPitchZero < -50
                || rx.yaw - settings.RxYawZero > 50
                || rx.yaw - settings.RxYawZero < -50
                || rx.collective - settings.RxCollectiveZero > 50
                || rx.collective - settings.RxCollectiveZero < -50) {
            finish_counter = 0;
        } else {
            finish_counter++;
        }
        if (finish_counter == 40) {
            break;
        }

        // Blink
        LED_INVERT();
        /* Wait enough time for Rx data to update (60ms should be enough
         *  considering that receiver gives pulse every 50ms) */
        _delay_ms(60);
    }

    // Scale zero offset
    settings.RxRollZero = receiverScale(settings.RxRollZero,
            settings.RxRollMin, settings.RxRollMax);
    settings.RxPitchZero = receiverScale(settings.RxPitchZero,
            settings.RxPitchMin, settings.RxPitchMax);
    settings.RxYawZero = receiverScale(settings.RxYawZero,
            settings.RxYawMin, settings.RxYawMax);
    settings.RxCollectiveZero = receiverScale(settings.RxCollectiveZero,
            settings.RxCollectiveMin, settings.RxCollectiveMax);

    settingsWrite(&settings);
    // Wait for reboot
    LED_BLINK_FOREVER(2000);
}
