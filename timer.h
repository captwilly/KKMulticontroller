/*
 * timer.h
 *
 *  Created on: Aug 21, 2012
 *      Author: dart
 */
#include "common.h"

#ifndef TIMER_H_
#define TIMER_H_


/******************************************************************************
 ***    Defines                                                             ***
 ******************************************************************************/
#define T0_FREQ     8000000
#define T1_FREQ     8000000
#define T2_FREQ     (8000000 / 1024)

#define TIMER_FREQ  T1_FREQ


/******************************************************************************
 ***    Interface                                                           ***
 ******************************************************************************/
void timerInit(void);
uint32_t timerGetTime(void);
uint32_t timerGetTimeUnsafe(void);

#endif /* TIMER_H_ */
