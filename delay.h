/*
 * delay.h
 *
 *  Created on: Mar 08, 2013
 *      Author: seb
 */
#include "common.h"
#include <util/delay.h>

#ifndef DELAY_H_
#define DELAY_H_


/******************************************************************************
 ***    Defines                                                             ***
 ******************************************************************************/

/******************************************************************************
 ***    Interface                                                           ***
 ******************************************************************************/
void delay_ms (uint16_t time);
void delay_us (uint16_t time);
#endif /* DELAY_H_ */