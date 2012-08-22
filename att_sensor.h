/*
 * att_sensor.h
 *
 *  Created on: Aug 22, 2012
 *      Author: dart
 */

#ifndef ATT_SENSOR_H_
#define ATT_SENSOR_H_

#include "common.h"


// Call only under disabled interrupts
void attISR(bool state);
uint16_t attGetDistance(void);

#endif /* ATT_SENSOR_H_ */
