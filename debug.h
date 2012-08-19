/*
 * debug.h
 *
 *  Created on: Aug 19, 2012
 *      Author: dart
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include <avr/eeprom.h>


#define DEBUG_INFO_EEP_POSITION 512

#define DEBUG(x) {                                                          \
        eeprom_write_block(&x, (void*)DEBUG_INFO_EEP_POSITION, sizeof(x));  \
        FOREVER{}                                                           \
    }

#endif /* DEBUG_H_ */
