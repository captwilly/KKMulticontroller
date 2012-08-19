#ifndef MOTORS_H
#define MOTORS_H

#include "common.h"


/******************************************************************************
 ***    Types                                                               ***
 ******************************************************************************/
struct MT_STATE_S {
    int16_t m1out;
    int16_t m2out;
    int16_t m3out;
    int16_t m4out;
#if M5_USED
int16_t m5out;
#endif
#if M6_USED
int16_t m6out;
#endif
};


/******************************************************************************
 ***    Interface                                                           ***
 ******************************************************************************/
void motorsSetup(void);
void motorsIdentify(void);
void motorsThrottleCalibration(void);
void motorOutputPPM(struct MT_STATE_S *state);

#endif
