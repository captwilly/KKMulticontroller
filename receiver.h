#ifndef RECEIVER_H
#define RECEIVER_H

#include "common.h"


/*** BEGIN TYPES ***/
struct RX_STATE_S {
    int16_t roll;
    int16_t pitch;
    int16_t collective;
    int16_t yaw;
#ifdef TWIN_COPTER
int16_t orgPitch;
#endif
};
/*** END TYPES ***/

/*** BEGIN PROTOTYPES ***/
void receiverSetup(void);
void receiverGetChannels(struct RX_STATE_S *state);
void receiverStickCenterManual(void);
void receiverStickCenterAutomatic(void);
/*** END PROTOTYPES ***/

#endif
