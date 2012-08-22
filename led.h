/*
 * led.h
 *
 *  Created on: Aug 19, 2012
 *      Author: dart
 */

#ifndef LED_H_
#define LED_H_

#include "common.h"
#include <util/delay.h>

#define LED_INIT()      LED_DIR = OUTPUT

#define LED_ON()        LED = 1
#define LED_OFF()       LED = 0
#define LED_INVERT()    LED = ~LED
#define LED_WRITE(val)  LED = (bool)val

/* TODO: add uniform interface which indicates errors (infinite loop with LED
 *  pattern corresponding to error type) and actions performed */
#define LED_BLINK(period, count)    {                                       \
    uint16_t cnt = count;                                                   \
    while (cnt-- > 0) {                                                     \
        LED_ON();                                                           \
        _delay_ms(period / 2);                                              \
        LED_OFF();                                                          \
        _delay_ms(period / 2);                                              \
    }                                                                       \
}                                                                           \

#define LED_BLINK_FOREVER(period)                                           \
    FOREVER {                                                               \
        LED_ON();                                                           \
        _delay_ms(period / 2);                                              \
        LED_OFF();                                                          \
        _delay_ms(period / 2);                                              \
    }                                                                       \


#endif /* LED_H_ */
