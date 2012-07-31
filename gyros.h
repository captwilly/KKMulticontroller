#ifndef GYROS_H
#define GYROS_H

#include "common.h"

/*** BEGIN DEFINES ***/
#define ADC_MAX 1023
/*** END DEFINES ***/

/*** BEGIN HELPER MACROS ***/
#ifdef GAIN_POT_REVERSE
#undef GAIN_POT_REVERSE
#define GAIN_POT_REVERSE ADC_MAX -
#else
#define GAIN_POT_REVERSE
#endif
/*** END HELPER MACROS ***/

/*** BEGIN TYPES ***/
enum GyroDirection {
    GYRO_NORMAL = 0,
    GYRO_REVERSED
};

struct GYRO_GAIN_ADC_S {
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
};
struct GYRO_STATE_S {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
};
/*** END TYPES ***/

/*** BEGIN PROTOTYPES ***/
void gyrosReadGainPots(struct GYRO_GAIN_ADC_S *pots);
void gyrosRead(struct GYRO_STATE_S *state);
void gyrosCalibrate(void);
void gyrosSetup(void);
void gyrosReverse(void);
/*** END PROTOTYPES ***/

#endif
