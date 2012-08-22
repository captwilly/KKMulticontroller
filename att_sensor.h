/*
 * att_sensor.h
 *
 *  Created on: Aug 22, 2012
 *      Author: dart
 */

#include "common.h"

// Don't define anything unless we have attitude sensor enabled
#if !defined(ATT_SENSOR_H_) && defined(ATTITUDE_SENSOR)
#define ATT_SENSOR_H_

/******************************************************************************
 ***    Defines                                                             ***
 ******************************************************************************/
#define T0_FREQ             (8000000 / 1024)
// Actual attitude measurement rate (freq / 2^(timer width))
#define ATT_RATE            (T0_FREQ / 256)


/******************************************************************************
 ***    Interface                                                           ***
 ******************************************************************************/
// Call only under disabled interrupts
void attISR(bool state);
void attInit(void);
uint16_t attGetDistance(void);

#endif // !defined(ATT_SENSOR_H_) && defined(ATTITUDE_SENSOR)
