/*
 * Filename:    delay.c
 * Description: this module provides more or less accurate delay functions
 * 
 * 
 *
 *  Created on: Mar 08, 2013
 *      Author: seb
 */
#include "delay.h"

/******************************************************************************
 ***    Module variables                                                    ***
 ******************************************************************************/

/******************************************************************************
 ***    Interrupt service routines                                          ***
 ******************************************************************************/

/******************************************************************************
 ***    Public functions                                                    ***
 ******************************************************************************/
// Delay loop functions
void delay_ms (uint16_t time)
{
  while(time --)
    _delay_ms(1);
}

void delay_us (uint16_t time)
{
  while(time--)
    _delay_us(1);
}

