/*
 * att_sensor.h
 *
 *  Created on: Aug 22, 2012
 *      Author: dart
 */

#include "common.h"

// Don't define anything unless we have attitude sensor enabled
#if !defined(ATT_SENSOR_H_) && defined(ATTITUDE_SENSOR) && !defined(RX_CONFIG)
#define ATT_SENSOR_H_

/******************************************************************************
 ***    Defines                                                             ***
 ******************************************************************************/
// Minimal recommended period for measurements
#define ATT_MEAS_PERIOD_MS  60
// and it's corresponding frequency
#define ATT_RATE            (1000/ATT_MEAS_PERIOD_MS)


/******************************************************************************
 ***    Interface                                                           ***
 ******************************************************************************/
// Call only under disabled interrupts
void attISR(bool state);
void attInit(void);
void attTrigger(void);
uint16_t attGetDistance(void);

#endif // !defined(ATT_SENSOR_H_) && defined(ATTITUDE_SENSOR)
