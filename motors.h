#ifndef MOTORS_H
#define MOTORS_H

#include "common.h"

/******************************************************************************
 ***    Defines                                                             ***
 ******************************************************************************/
#if defined(SINGLE_COPTER)                                                  \
    || defined(TWIN_COPTER)                                                 \
    || defined(HEX_COPTER)                                                  \
    || defined(Y6_COPTER)
#define M5_USED 1
#else
#define M5_USED 0
#endif
#if defined(TWIN_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
#define M6_USED 1
#else
#define M6_USED 0
#endif

#if defined(ATTITUDE_SENSOR) && (M5_USED || M6_USED)
#error "Your copter configuration require M5 and/or M6. Attitude sensor is not supported"
#endif

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
